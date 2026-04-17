/**
 * @file riccati_solver_hls.cpp
 * @brief Riccati-ADMM Solver — HLS-Synthesizable Implementation
 * @details Solves constrained LQR using ADMM with Riccati recursion and
 *          fixed-size matrices tailored for FPGA synthesis. The implementation
 *          exploits the augmented-state sparsity pattern to reduce arithmetic
 *          and memory pressure in backward and forward passes.
 * @dependencies riccati_solver_hls.h, fp_math_hls.h
 */

#include "../include/riccati_solver_hls.h"
#include "../include/fp_math_hls.h"
#include <cstdint>
#include <climits>

/*===========================================================================
 * Riccati Backward + Forward Pass
 *
 * Backward: compute K_k, kk_k for k = N-1 .. 0
 * Forward:  roll out x_k, u_k from x0
 *
 * Exploits A/B sparsity:
 *   A: dense 6x6 block (rows/cols 0-5), rows 6-7 and cols 6-7 zero
 *   B: rows 0-1 zero, rows 2-5 have body dynamics entries,
 *      B[5][0]=dt, B[6][0]=1, B[7][1]=1
 *===========================================================================*/

void riccati_pass_hls(
    const StepData_t step_data[MPC_HORIZON],
    const fp_QP_t *terminal_q_diag,
    const fp_QP_t *terminal_q_linear,
    const fp_QP_t *terminal_x_lb,
    const fp_QP_t *terminal_x_ub,
    const fp_QP_t *x0,
    fp_QP_t rho, fp_QP_t rho_u,
    const fp_QP_t z_x[][MPC_NX_AUG],
    const fp_QP_t y_x[][MPC_NX_AUG],
    const fp_QP_t z_u[][MPC_NU],
    const fp_QP_t y_u[][MPC_NU],
    fp_QP_t x_out[][MPC_NX_AUG],
    fp_QP_t u_out[][MPC_NU])
{
    const int nx = MPC_NX_AUG;
    const int nu = MPC_NU;
    const int N = MPC_HORIZON;

    /* Gains stored for forward pass */
    fp_QP_t K[MPC_HORIZON][MPC_NU][MPC_NX_AUG];
    fp_QP_t kk[MPC_HORIZON][MPC_NU];
#pragma HLS ARRAY_PARTITION variable=K complete dim=3
#pragma HLS ARRAY_PARTITION variable=K complete dim=2
#pragma HLS ARRAY_PARTITION variable=kk complete dim=2

    /* Value function P (nx x nx) and p (nx x 1) in int64 */
    fp_raw_acc_t P[MPC_NX_AUG][MPC_NX_AUG];
    fp_raw_acc_t p[MPC_NX_AUG];
#pragma HLS ARRAY_PARTITION variable=P cyclic factor=8 dim=1
#pragma HLS ARRAY_PARTITION variable=p complete dim=1
#pragma HLS ALLOCATION operation instances=mul limit=MPC_HLS_RICCATI_MUL_LIMIT

    /* Initialize terminal cost: P_N = Q_N [+ rho*I if constrained] */
    int s, i, j, a, b, k;

    for (i = 0; i < nx; i++) {
#pragma HLS UNROLL
        for (j = 0; j < nx; j++) {
#pragma HLS UNROLL
            P[i][j] = 0;
        }
        p[i] = 0;
    }

    {
        for (s = 0; s < nx; s++) {
            int is_con = (terminal_x_ub[s] < BOUND_THRESHOLD ||
                          terminal_x_lb[s] > -BOUND_THRESHOLD);
            if (is_con) {
                P[s][s] = fp_raw_acc_from_qp(terminal_q_diag[s]) + fp_raw_acc_from_qp(rho);
                p[s] = fp_raw_acc_from_qp(terminal_q_linear[s])
                     - ((fp_raw_acc_from_qp(rho) * (fp_raw_acc_from_qp(z_x[N][s]) - fp_raw_acc_from_qp(y_x[N][s]))) >> FP_FRAC_BITS);
            } else {
                P[s][s] = fp_raw_acc_from_qp(terminal_q_diag[s]);
                p[s] = fp_raw_acc_from_qp(terminal_q_linear[s]);
            }
        }
    }

    /* ===== Backward pass: k = N-1 down to 0 ===== */
    for (k = N - 1; k >= 0; k--) {
#pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON max=MPC_HORIZON
#pragma HLS LOOP_FLATTEN off
        const StepData_t *sd = &step_data[k];
        fp_QP_t A_local[MPC_NX_DENSE][MPC_NX_DENSE];
#pragma HLS ARRAY_PARTITION variable=A_local complete dim=0
        for (i = 0; i < 6; i++) {
            for (j = 0; j < 6; j++) {
#pragma HLS PIPELINE II=1
                A_local[i][j] = sd->A[i][j];
            }
        }

        /* Augmented costs */
        fp_raw_acc_t q_aug_diag[MPC_NX_AUG];
        fp_raw_acc_t q_aug_linear[MPC_NX_AUG];
        fp_raw_acc_t r_aug_diag[MPC_NU];
        fp_raw_acc_t r_aug_linear[MPC_NU];

        for (s = 0; s < nx; s++) {
#pragma HLS UNROLL
            int is_con = (sd->x_ub[s] < BOUND_THRESHOLD ||
                          sd->x_lb[s] > -BOUND_THRESHOLD);
            if (is_con) {
                q_aug_diag[s] = fp_raw_acc_from_qp(sd->Q_diag[s]) + fp_raw_acc_from_qp(rho);
                fp_raw_acc_t zx_minus_yx = fp_raw_acc_from_qp(z_x[k][s]) - fp_raw_acc_from_qp(y_x[k][s]);
                fp_raw_acc_t rho_state_mul = fp_raw_acc_from_qp(rho) * zx_minus_yx;
#pragma HLS BIND_OP variable=rho_state_mul op=mul impl=dsp latency=4
                q_aug_linear[s] = fp_raw_acc_from_qp(sd->q[s]) - (rho_state_mul >> FP_FRAC_BITS);
            } else {
                q_aug_diag[s] = fp_raw_acc_from_qp(sd->Q_diag[s]);
                q_aug_linear[s] = fp_raw_acc_from_qp(sd->q[s]);
            }
        }
        for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
            r_aug_diag[a] = fp_raw_acc_from_qp(sd->R_diag[a]) + fp_raw_acc_from_qp(rho_u);
            fp_raw_acc_t zu_minus_yu = fp_raw_acc_from_qp(z_u[k][a]) - fp_raw_acc_from_qp(y_u[k][a]);
            fp_raw_acc_t rho_ctrl_mul = fp_raw_acc_from_qp(rho_u) * zu_minus_yu;
#pragma HLS BIND_OP variable=rho_ctrl_mul op=mul impl=dsp latency=4
            r_aug_linear[a] = fp_raw_acc_from_qp(sd->r[a]) - (rho_ctrl_mul >> FP_FRAC_BITS);
        }

        /* Step 1: M = B^T * P (nu x nx) — accumulate only nonzero B rows */
        fp_QP_t M[MPC_NU][MPC_NX_AUG];
#pragma HLS ARRAY_PARTITION variable=M complete dim=1
#pragma HLS ARRAY_PARTITION variable=M cyclic factor=4 dim=2
        for (j = 0; j < nx; j++) {
#pragma HLS PIPELINE II=MPC_HLS_M_BT_P_II
            fp_raw_acc_t s0 = 0;
            fp_raw_acc_t s1 = 0;
            for (i = 0; i < nx; i++) {
#pragma HLS UNROLL factor=4
                s0 += fp_raw_acc_from_qp(sd->B[i][0]) * P[i][j];
                s1 += fp_raw_acc_from_qp(sd->B[i][1]) * P[i][j];
            }
            fp_raw_acc_t m0 = (s0 >> FP_FRAC_BITS);
            fp_raw_acc_t m1 = (s1 >> FP_FRAC_BITS);
            m0 = fp_clip_raw_to_qp(m0);
            m1 = fp_clip_raw_to_qp(m1);
            M[0][j] = fp_qp_from_raw_acc(m0);
            M[1][j] = fp_qp_from_raw_acc(m1);
        }

        /* Step 2: S = r_aug_diag + M*B (2x2) */
        fp_raw_acc_t S[2][2];
        {
            fp_raw_acc_t MB[2][2];
#pragma HLS ARRAY_PARTITION variable=MB complete dim=0

            /* Stage A: multiply terms for M*B */
            for (a = 0; a < nu; a++) {
#pragma HLS PIPELINE II=1
                fp_raw_acc_t mb0 = 0;
                fp_raw_acc_t mb1 = 0;
                for (i = 0; i < nx; i++) {
#pragma HLS UNROLL factor=4
                    mb0 += fp_raw_acc_from_qp(M[a][i]) * fp_raw_acc_from_qp(sd->B[i][0]);
                    mb1 += fp_raw_acc_from_qp(M[a][i]) * fp_raw_acc_from_qp(sd->B[i][1]);
                }
                MB[a][0] = mb0;
                MB[a][1] = mb1;
            }

            /* Stage B: scale and assemble S */
            for (a = 0; a < nu; a++) {
#pragma HLS PIPELINE II=1
                fp_raw_acc_t mb0 = MB[a][0] >> FP_FRAC_BITS;
                fp_raw_acc_t mb1 = MB[a][1] >> FP_FRAC_BITS;
                S[a][0] = mb0;
                S[a][1] = mb1;
            }

            S[0][0] += r_aug_diag[0];
            S[1][1] += r_aug_diag[1];
            fp_raw_acc_t s01 = (S[0][1] + S[1][0]) >> 1;
            S[0][1] = s01;
            S[1][0] = s01;
        }

        /* Step 3: Invert S (2x2) */
        fp_raw_acc_t Si[2][2];
        if (invert_2x2_hls(S, Si) < 0) {
            /* Near-singular safeguard: use diagonal reciprocal fallback
             * instead of identity, preserving units/scaling of S^{-1}. */
            const fp_raw_acc_t eps = ((fp_raw_acc_t)1 << (FP_FRAC_BITS - 8));
            fp_raw_acc_t s00 = (S[0][0] > eps) ? S[0][0] : eps;
            fp_raw_acc_t s11 = (S[1][1] > eps) ? S[1][1] : eps;
            Si[0][0] = reciprocal_raw(s00);
            Si[0][1] = 0;
            Si[1][0] = 0;
            Si[1][1] = reciprocal_raw(s11);
        }

        /* Step 4: G = M*A + N^T (nu x nx) — exploit A sparsity */
        fp_QP_t G[MPC_NU][MPC_NX_AUG];
#pragma HLS ARRAY_PARTITION variable=G complete dim=1
#pragma HLS ARRAY_PARTITION variable=G cyclic factor=4 dim=2
        for (a = 0; a < nu; a++) {
            /* Cols 0..5: M*A uses A rows 0..5 (6x6 dense block) */
            for (j = 0; j < 6; j++) {
#pragma HLS PIPELINE II=1
                fp_raw_acc_t sum = 0;
                for (s = 0; s < 6; s++) {
#pragma HLS UNROLL factor=3
                    fp_raw_acc_t gma_prod = fp_raw_acc_from_qp(M[a][s]) * fp_raw_acc_from_qp(A_local[s][j]);
                    sum += gma_prod;
                }
                fp_raw_acc_t g_val = fp_raw_acc_from_qp(sd->N_cross[j][a]) + (sum >> FP_FRAC_BITS);
                g_val = fp_clip_raw_to_qp(g_val);
                G[a][j] = fp_qp_from_raw_acc(g_val);
            }
            /* Cols 6,7: A cols 6,7 are zero */
            G[a][6] = sd->N_cross[6][a];
            G[a][7] = sd->N_cross[7][a];
        }

        /* Step 5: K = -S^{-1} * G (nu x nx) */
        for (a = 0; a < nu; a++) {
            for (j = 0; j < nx; j++) {
#pragma HLS PIPELINE II=1
                fp_raw_acc_t val = 0;
                for (b = 0; b < nu; b++) {
#pragma HLS UNROLL
                    fp_raw_acc_t k_prod = Si[a][b] * fp_raw_acc_from_qp(G[b][j]);
                    val += k_prod;
                }
                val = -(val >> FP_FRAC_BITS);
                val = fp_clip_raw_to_qp(val);
                K[k][a][j] = fp_qp_from_raw_acc(val);
            }
        }

        /* Step 6: kk = -S^{-1} * (r_aug_linear + B^T * p) */
        fp_raw_acc_t p_shift[MPC_NX_AUG];
#pragma HLS ARRAY_PARTITION variable=p_shift complete dim=1
        for (i = 0; i < nx; i++) {
#pragma HLS UNROLL
            fp_raw_acc_t pd_sum = 0;
            for (j = 0; j < nx; j++) {
#pragma HLS UNROLL factor=2
                pd_sum += P[i][j] * fp_raw_acc_from_qp(sd->d[j]);
            }
            p_shift[i] = p[i] + (pd_sum >> FP_FRAC_BITS);
        }

        fp_raw_acc_t Bp[MPC_NU];
        {
            for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
                fp_raw_acc_t bp = 0;
                for (i = 0; i < nx; i++) {
#pragma HLS UNROLL factor=4
                    bp += fp_raw_acc_from_qp(sd->B[i][a]) * p_shift[i];
                }
                Bp[a] = bp >> FP_FRAC_BITS;
            }
        }

        for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
            fp_raw_acc_t val = 0;
            for (b = 0; b < nu; b++) {
                val += Si[a][b] * (r_aug_linear[b] + Bp[b]);
            }
            val = -(val >> FP_FRAC_BITS);
            val = fp_clip_raw_to_qp(val);
            kk[k][a] = fp_qp_from_raw_acc(val);
        }

        /* Step 7: P = Q_diag + A^T*P*A + G^T*K */
        /* PA = P * A (only 6x6 dense block needed)
         * A sparsity: col 0 has only A[0][0]=1 (rows 1-5 are 0).
         * Col 5 has up to 4 nonzero entries (steering coupling).
         * Exploit col-0 identity: PA[i][0] = P[i][0], no multiply. */
        fp_raw_acc_t PA[MPC_NX_DENSE][MPC_NX_DENSE];
#pragma HLS ARRAY_PARTITION variable=PA complete dim=1
#pragma HLS ARRAY_PARTITION variable=PA complete dim=2
        /* Col 0: A[0][0]=1, rest zero → PA[i][0] = P[i][0] */
        for (i = 0; i < 6; i++) {
#pragma HLS UNROLL
            PA[i][0] = P[i][0];
        }
        /* Cols 1-5: full inner product */
        for (i = 0; i < 6; i++) {
            for (j = 1; j < 6; j++) {
#pragma HLS PIPELINE II=1
                fp_raw_acc_t sum = 0;
                for (s = 0; s < 6; s++) {
#pragma HLS UNROLL
                    fp_raw_acc_t pa_prod = P[i][s] * fp_raw_acc_from_qp(sd->A[s][j]);
                    sum += pa_prod;
                }
                PA[i][j] = sum >> FP_FRAC_BITS;
            }
        }

        /* P = Q_diag + A^T*PA + G^T*K  */
        /* Dense block: rows 0..5, cols 0..5 — exploit P symmetry */
        {
            /* Upper-triangle decode tables (i,j) for 21 entries */
            static const int sym_i[21] = {0,0,0,0,0,0, 1,1,1,1,1, 2,2,2,2, 3,3,3, 4,4, 5};
            static const int sym_j[21] = {0,1,2,3,4,5, 1,2,3,4,5, 2,3,4,5, 3,4,5, 4,5, 5};
            for (int idx = 0; idx < 21; idx++) {
#pragma HLS PIPELINE II=1
                int ii = sym_i[idx];
                int jj = sym_j[idx];
                fp_raw_acc_t sum = 0;
                /* A^T * PA: A_local in registers */
                for (s = 0; s < 6; s++) {
#pragma HLS UNROLL
                    fp_raw_acc_t atpa_prod = fp_raw_acc_from_qp(A_local[s][ii]) * PA[s][jj];
                    sum += atpa_prod;
                }
                /* G^T * K contribution */
                for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
                    fp_raw_acc_t gtk_prod = fp_raw_acc_from_qp(G[a][ii]) * fp_raw_acc_from_qp(K[k][a][jj]);
                    sum += gtk_prod;
                }
                fp_raw_acc_t val = ((ii == jj) ? q_aug_diag[ii] : (fp_raw_acc_t)0) + (sum >> FP_FRAC_BITS);
                P[ii][jj] = val;
                if (ii != jj) P[jj][ii] = val;  /* Symmetric mirror */
            }
        }
        /* Cols 6,7: AtPA=0, only G^T*K contributes */
        for (i = 0; i < 6; i++) {
            for (j = 6; j < nx; j++) {
#pragma HLS PIPELINE II=1
                fp_raw_acc_t sum = 0;
                for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
                    sum += fp_raw_acc_from_qp(G[a][i]) * fp_raw_acc_from_qp(K[k][a][j]);
                }
                P[i][j] = sum >> FP_FRAC_BITS;
                P[j][i] = P[i][j];
            }
        }
        /* Rows 6,7: fill 2x2 lower-right block only; 6/7 cross-block above is mirrored. */
        for (j = 6; j < nx; j++) {
#pragma HLS PIPELINE II=1
            fp_raw_acc_t s6 = 0;
            fp_raw_acc_t s7 = 0;
            for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
                s6 += fp_raw_acc_from_qp(G[a][6]) * fp_raw_acc_from_qp(K[k][a][j]);
                s7 += fp_raw_acc_from_qp(G[a][7]) * fp_raw_acc_from_qp(K[k][a][j]);
            }
            P[6][j] = ((j == 6) ? q_aug_diag[6] : (fp_raw_acc_t)0) + (s6 >> FP_FRAC_BITS);
            P[7][j] = ((j == 7) ? q_aug_diag[7] : (fp_raw_acc_t)0) + (s7 >> FP_FRAC_BITS);
        }
        P[7][6] = P[6][7];

        /* Step 8: p_new = q_aug_linear + A^T*p + G^T*kk */
        fp_raw_acc_t p_new[MPC_NX_AUG];
        for (i = 0; i < 6; i++) {
    #pragma HLS PIPELINE II=1
            fp_raw_acc_t Atp = 0;
            for (s = 0; s < 6; s++) {
#pragma HLS UNROLL
                fp_raw_acc_t step8_prod = fp_raw_acc_from_qp(A_local[s][i]) * p_shift[s];
                Atp += step8_prod;
            }
            fp_raw_acc_t Gtk = 0;
            for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
                Gtk += fp_raw_acc_from_qp(G[a][i]) * fp_raw_acc_from_qp(kk[k][a]);
            }
            p_new[i] = q_aug_linear[i] + (Atp >> FP_FRAC_BITS) + (Gtk >> FP_FRAC_BITS);
        }
        for (i = 6; i < nx; i++) {
    #pragma HLS PIPELINE II=1
            fp_raw_acc_t Gtk = 0;
            for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
                Gtk += fp_raw_acc_from_qp(G[a][i]) * fp_raw_acc_from_qp(kk[k][a]);
            }
            p_new[i] = q_aug_linear[i] + (Gtk >> FP_FRAC_BITS);
        }
        for (i = 0; i < nx; i++) p[i] = p_new[i];

    } /* end backward pass */

    /* ===== Forward pass: roll out x, u from x0 ===== */
    for (s = 0; s < nx; s++) {
#pragma HLS UNROLL
        x_out[0][s] = x0[s];
    }

    for (k = 0; k < N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON max=MPC_HORIZON
#pragma HLS LOOP_FLATTEN off
        const StepData_t *sd = &step_data[k];

        /* Buffer A locally in registers for the forward rollout
         * x_{k+1} = A_k x_k + B_k u_k. */
        fp_QP_t A_fwd[MPC_NX_DENSE][MPC_NX_DENSE];
#pragma HLS ARRAY_PARTITION variable=A_fwd complete dim=0
        for (i = 0; i < 6; i++) {
            for (j = 0; j < 6; j++) {
#pragma HLS PIPELINE II=1
                A_fwd[i][j] = sd->A[i][j];
            }
        }

        /* u_k = K_k * x_k + kk_k */
        for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
            fp_raw_acc_t prod_sum = 0;
            for (s = 0; s < nx; s++) {
    #pragma HLS UNROLL factor=2
                prod_sum += fp_raw_acc_from_qp(K[k][a][s]) * fp_raw_acc_from_qp(x_out[k][s]);
            }
            fp_raw_acc_t sum = fp_raw_acc_from_qp(kk[k][a]) + (prod_sum >> FP_FRAC_BITS);
            sum = fp_clip_raw_to_qp(sum);
            u_out[k][a] = fp_qp_from_raw_acc(sum);
        }

        /* x_{k+1} = A_k * x_k + B_k * u_k
         * Dense rows 0..5: A*x + B*u */
        for (i = 0; i < 6; i++) {
#pragma HLS PIPELINE II=1
            fp_raw_acc_t sum = fp_raw_acc_from_qp(sd->d[i]) << FP_FRAC_BITS;
            for (s = 0; s < 6; s++) {
    #pragma HLS UNROLL factor=2
            sum += fp_raw_acc_from_qp(A_fwd[i][s]) * fp_raw_acc_from_qp(x_out[k][s]);
            }
            for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
            sum += fp_raw_acc_from_qp(sd->B[i][a]) * fp_raw_acc_from_qp(u_out[k][a]);
            }
            fp_raw_acc_t result = sum >> FP_FRAC_BITS;
            result = fp_clip_raw_to_qp(result);
            x_out[k + 1][i] = fp_qp_from_raw_acc(result);
        }
        /* Rows 6,7: x_prev = u (identity in B, zero in A) */
        x_out[k + 1][6] = fp_qp_from_raw_acc(fp_clip_raw_to_qp(fp_raw_acc_from_qp(u_out[k][0]) + fp_raw_acc_from_qp(sd->d[6])));
        x_out[k + 1][7] = fp_qp_from_raw_acc(fp_clip_raw_to_qp(fp_raw_acc_from_qp(u_out[k][1]) + fp_raw_acc_from_qp(sd->d[7])));
    }
}

/*===========================================================================
 * Main Solver: Riccati-ADMM
 *===========================================================================*/

MpcStatus_t riccati_admm_solve_hls(
    const StepData_t step_data[MPC_HORIZON],
    const fp_QP_t terminal_q_diag[MPC_NX_AUG],
    const fp_QP_t terminal_q_linear[MPC_NX_AUG],
    const fp_QP_t terminal_x_lb[MPC_NX_AUG],
    const fp_QP_t terminal_x_ub[MPC_NX_AUG],
    const fp_QP_t x0[MPC_NX_AUG],
    const AdmmConfig_t *config,
    AdmmState_t *admm_state,
    MpcSolution_t *solution)
{
    const int nx = MPC_NX_AUG;
    const int nu = MPC_NU;
    const int N = MPC_HORIZON;

#pragma HLS ALLOCATION function instances=riccati_pass_hls limit=1

    /* Runtime config is authoritative on each solve invocation. */
    fp_QP_t rho   = config->rho;
    fp_QP_t rho_u = (config->rho_u > 0) ? config->rho_u : rho;
    int max_iter = config->max_iterations;
    fp_QP_t abs_tolerance = (config->tolerance > FP_QP_CONST(1e-6))
                                ? config->tolerance
                                : FP_QP_CONST(1e-6);
    const fp_QP_t rel_tolerance = FP_QP_CONST(0.02);

    /* Local ADMM variables */
    fp_QP_t z_x[MPC_HORIZON + 1][MPC_NX_AUG];
    fp_QP_t z_u[MPC_HORIZON][MPC_NU];
    fp_QP_t y_x[MPC_HORIZON + 1][MPC_NX_AUG];
    fp_QP_t y_u[MPC_HORIZON][MPC_NU];
#pragma HLS ARRAY_PARTITION variable=z_x complete dim=2
#pragma HLS ARRAY_PARTITION variable=z_u complete dim=2
#pragma HLS ARRAY_PARTITION variable=y_x complete dim=2
#pragma HLS ARRAY_PARTITION variable=y_u complete dim=2

    /* Precompute constrained flags */
    uint8_t x_is_con[MPC_HORIZON + 1][MPC_NX_AUG];
#pragma HLS ARRAY_PARTITION variable=x_is_con complete dim=2
    int k, s, a;

    for (k = 0; k <= N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON_PLUS_ONE max=MPC_HORIZON_PLUS_ONE
        for (s = 0; s < nx; s++) {
#pragma HLS UNROLL
            const fp_QP_t xlb = (k < N) ? step_data[k].x_lb[s] : terminal_x_lb[s];
            const fp_QP_t xub = (k < N) ? step_data[k].x_ub[s] : terminal_x_ub[s];
            x_is_con[k][s] = (xub < BOUND_THRESHOLD ||
                               xlb > -BOUND_THRESHOLD);
        }
    }

    if (admm_state->initialized) {
        /* Warm-start from previous solve — direct copy of ADMM state */
        for (k = 0; k <= N; k++) {
#pragma HLS PIPELINE II=1
            for (s = 0; s < nx; s++) {
                z_x[k][s] = admm_state->z_x[k][s];
                y_x[k][s] = admm_state->y_x[k][s];
            }
        }
        for (k = 0; k < N; k++) {
#pragma HLS PIPELINE II=1
            for (a = 0; a < nu; a++) {
                z_u[k][a] = admm_state->z_u[k][a];
                y_u[k][a] = admm_state->y_u[k][a];
            }
        }
    } else {
        /* Cold start: unconstrained Riccati pass to initialize */
        for (k = 0; k <= N; k++) {
#pragma HLS PIPELINE II=1
            for (s = 0; s < nx; s++) {
                z_x[k][s] = 0;
                y_x[k][s] = 0;
            }
        }
        for (k = 0; k < N; k++) {
#pragma HLS PIPELINE II=1
            for (a = 0; a < nu; a++) {
                z_u[k][a] = 0;
                y_u[k][a] = 0;
            }
        }

        riccati_pass_hls(
            step_data, terminal_q_diag, terminal_q_linear,
            terminal_x_lb, terminal_x_ub,
            x0, 0, 0,
            (const fp_QP_t (*)[MPC_NX_AUG])z_x,
            (const fp_QP_t (*)[MPC_NX_AUG])y_x,
            (const fp_QP_t (*)[MPC_NU])z_u,
            (const fp_QP_t (*)[MPC_NU])y_u,
            solution->x, solution->u);

        /* Initialize z from projection of unconstrained solution */
        for (k = 0; k <= N; k++) {
    #pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON_PLUS_ONE max=MPC_HORIZON_PLUS_ONE
            for (s = 0; s < nx; s++) {
#pragma HLS UNROLL
                fp_QP_t val = solution->x[k][s];
                const fp_QP_t xlb = (k < N) ? step_data[k].x_lb[s] : terminal_x_lb[s];
                const fp_QP_t xub = (k < N) ? step_data[k].x_ub[s] : terminal_x_ub[s];
                if (val < xlb) val = xlb;
                if (val > xub) val = xub;
                z_x[k][s] = val;
            }
        }
        for (k = 0; k < N; k++) {
    #pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON max=MPC_HORIZON
            for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
                fp_QP_t val = solution->u[k][a];
                if (val < step_data[k].u_lb[a]) val = step_data[k].u_lb[a];
                if (val > step_data[k].u_ub[a]) val = step_data[k].u_ub[a];
                z_u[k][a] = val;
            }
        }

        /* Initialize y (dual) from constraint violation */
        for (k = 0; k <= N; k++) {
    #pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON_PLUS_ONE max=MPC_HORIZON_PLUS_ONE
            for (s = 0; s < nx; s++) {
#pragma HLS UNROLL
                if (x_is_con[k][s]) {
                    y_x[k][s] = solution->x[k][s] - z_x[k][s];
                }
            }
        }
        for (k = 0; k < N; k++) {
    #pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON max=MPC_HORIZON
            for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
                y_u[k][a] = solution->u[k][a] - z_u[k][a];
            }
        }
    }

    MpcStatus_t status = MPC_STATUS_MAX_ITER;
    int iter;

    for (iter = 0; iter < max_iter; iter++) {
#pragma HLS LOOP_TRIPCOUNT min=1 max=MPC_MAX_ADMM_ITER avg=2

        /* --- Primal update: Riccati pass with augmented costs --- */
        riccati_pass_hls(
            step_data, terminal_q_diag, terminal_q_linear,
            terminal_x_lb, terminal_x_ub,
            x0, rho, rho_u,
            (const fp_QP_t (*)[MPC_NX_AUG])z_x,
            (const fp_QP_t (*)[MPC_NX_AUG])y_x,
            (const fp_QP_t (*)[MPC_NU])z_u,
            (const fp_QP_t (*)[MPC_NU])y_u,
            solution->x, solution->u);

        /* Norm computation split into separate loops to allow pipelined II tightening */
        fp_QP_t x_norm_cand[MPC_HORIZON_PLUS_ONE][MPC_NX_AUG];
        fp_QP_t u_norm_cand[MPC_HORIZON][MPC_NU];
#pragma HLS ARRAY_PARTITION variable=x_norm_cand complete dim=2
#pragma HLS ARRAY_PARTITION variable=u_norm_cand complete dim=2

        for (k = 0; k <= N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON_PLUS_ONE max=MPC_HORIZON_PLUS_ONE
            for (s = 0; s < nx; s++) {
#pragma HLS UNROLL
                x_norm_cand[k][s] = fp_abs(solution->x[k][s]);
            }
        }
        for (k = 0; k < N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON max=MPC_HORIZON
            for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
                u_norm_cand[k][a] = fp_abs(solution->u[k][a]);
            }
        }

        fp_QP_t x_norm = 0;
        fp_QP_t u_norm = 0;
        for (k = 0; k <= N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON_PLUS_ONE max=MPC_HORIZON_PLUS_ONE
            for (s = 0; s < nx; s++) {
#pragma HLS UNROLL
                if (x_norm_cand[k][s] > x_norm) x_norm = x_norm_cand[k][s];
            }
        }
        for (k = 0; k < N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON max=MPC_HORIZON
            for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
                if (u_norm_cand[k][a] > u_norm) u_norm = u_norm_cand[k][a];
            }
        }

        /* --- Fused z-update, y-update, and residual computation ---
         * Dual residual uses rho*(z_new - z_old), where z_old is read
         * before writing z_new in each component. */
        fp_QP_t state_primal = 0, state_dual = 0;
        fp_QP_t ctrl_primal = 0, ctrl_dual = 0;
        fp_QP_t z_norm = 0, lambda_norm = 0;

        /* State z/y update */
        for (k = 0; k <= N; k++) {
    #pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON_PLUS_ONE max=MPC_HORIZON_PLUS_ONE
    #pragma HLS PIPELINE II=MPC_HLS_STATE_ZY_II
            for (s = 0; s < nx; s++) {
#pragma HLS UNROLL factor=MPC_HLS_UNROLL_STATE_FACTOR
                if (x_is_con[k][s]) {
                    fp_QP_t x_val = solution->x[k][s];
                    fp_raw_acc_t x_hat_raw = fp_raw_acc_from_qp(x_val);
                    fp_raw_acc_t val = x_hat_raw + fp_raw_acc_from_qp(y_x[k][s]);
                    const fp_QP_t xlb = (k < N) ? step_data[k].x_lb[s] : terminal_x_lb[s];
                    const fp_QP_t xub = (k < N) ? step_data[k].x_ub[s] : terminal_x_ub[s];
                    if (val < fp_raw_acc_from_qp(xlb)) val = fp_raw_acc_from_qp(xlb);
                    if (val > fp_raw_acc_from_qp(xub)) val = fp_raw_acc_from_qp(xub);
                    fp_QP_t z_new = fp_qp_from_raw_acc(val);

                    /* Dual residual contribution: rho*(z_new - z_old) */
                    fp_QP_t z_prev = z_x[k][s];
                    fp_raw_acc_t d64 = (fp_raw_acc_from_qp(rho) * (fp_raw_acc_from_qp(z_new) - fp_raw_acc_from_qp(z_prev))) >> FP_FRAC_BITS;
                    fp_raw_acc_t d64_abs = (d64 < 0) ? (fp_raw_acc_t)(-d64) : d64;
                    fp_QP_t dd = fp_qp_from_raw_acc(d64_abs);
                    if (dd > state_dual) state_dual = dd;

                    fp_QP_t y_new_x = fp_qp_from_raw_acc(fp_raw_acc_from_qp(x_val) - fp_raw_acc_from_qp(z_new) + fp_raw_acc_from_qp(y_x[k][s]));
                    y_x[k][s] = y_new_x;

                    fp_QP_t pd = x_val - z_new;
                    if (pd < 0) pd = -pd;
                    if (pd > state_primal) state_primal = pd;

                    fp_QP_t abs_z = fp_abs(z_new);
                    if (abs_z > z_norm) z_norm = abs_z;
                    fp_QP_t abs_l = fp_abs(fp_mul(rho, y_new_x));
                    if (abs_l > lambda_norm) lambda_norm = abs_l;

                    z_x[k][s] = z_new;
                } else {
                    z_x[k][s] = solution->x[k][s];
                }
            }
        }

        /* Control z/y update — dual residual computed inline */
        for (k = 0; k < N; k++) {
    #pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON max=MPC_HORIZON
#pragma HLS PIPELINE II=MPC_HLS_CTRL_ZY_II
            const StepData_t *sd = &step_data[k];
            for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
                fp_QP_t u_val = solution->u[k][a];
                fp_raw_acc_t u_hat_raw = fp_raw_acc_from_qp(u_val);
                fp_raw_acc_t val = u_hat_raw + fp_raw_acc_from_qp(y_u[k][a]);
                if (val < fp_raw_acc_from_qp(sd->u_lb[a])) val = fp_raw_acc_from_qp(sd->u_lb[a]);
                if (val > fp_raw_acc_from_qp(sd->u_ub[a])) val = fp_raw_acc_from_qp(sd->u_ub[a]);
                fp_QP_t z_new = fp_qp_from_raw_acc(val);

                /* Dual residual: rho_u * (z_new - z_old) */
                fp_QP_t z_prev = z_u[k][a];
                fp_raw_acc_t d64 = (fp_raw_acc_from_qp(rho_u) * (fp_raw_acc_from_qp(z_new) - fp_raw_acc_from_qp(z_prev))) >> FP_FRAC_BITS;
                fp_raw_acc_t d64_abs = (d64 < 0) ? (fp_raw_acc_t)(-d64) : d64;
                fp_QP_t dd = fp_qp_from_raw_acc(d64_abs);
                if (dd > ctrl_dual) ctrl_dual = dd;

                fp_QP_t y_new_u = fp_qp_from_raw_acc(fp_raw_acc_from_qp(u_val) - fp_raw_acc_from_qp(z_new) + fp_raw_acc_from_qp(y_u[k][a]));
                y_u[k][a] = y_new_u;

                fp_QP_t pd = u_val - z_new;
                if (pd < 0) pd = -pd;
                if (pd > ctrl_primal) ctrl_primal = pd;

                fp_QP_t abs_z = fp_abs(z_new);
                if (abs_z > z_norm) z_norm = abs_z;
                fp_QP_t abs_l = fp_abs(fp_mul(rho_u, y_new_u));
                if (abs_l > lambda_norm) lambda_norm = abs_l;

                z_u[k][a] = z_new;
            }
        }

        fp_QP_t primal_res = state_primal > ctrl_primal ? state_primal : ctrl_primal;
        fp_QP_t dual_res   = state_dual > ctrl_dual ? state_dual : ctrl_dual;
        fp_QP_t primal_scale = x_norm;
        if (u_norm > primal_scale) primal_scale = u_norm;
        if (z_norm > primal_scale) primal_scale = z_norm;
        fp_QP_t eps_primal = abs_tolerance + fp_mul(rel_tolerance, primal_scale);
        fp_QP_t eps_dual   = abs_tolerance + fp_mul(rel_tolerance, lambda_norm);

        solution->iterations = iter + 1;
        solution->primal_residual = primal_res;
        solution->dual_residual = dual_res;

        /* Convergence check */
        if (primal_res <= eps_primal && dual_res <= eps_dual) {
            status = MPC_STATUS_OPTIMAL;
            break;
        }

        /* Adaptive rho: rebalance state and control channels independently. */
        if (config->adaptive_rho && iter > 0) {
            const fp_QP_t adapt_ratio_state = FP_QP_CONST(6.0);
            const fp_QP_t adapt_ratio_ctrl = FP_QP_CONST(10.0);
            int scale_rho = 0;
            int scale_rho_u = 0;

            if (state_primal > fp_mul(adapt_ratio_state, state_dual) && rho <= FP_QP_CONST(50.0)) {
                scale_rho = 1;
            } else if (state_dual > fp_mul(adapt_ratio_state, state_primal) && rho >= FP_QP_CONST(1.0)) {
                scale_rho = -1;
            }

            if (ctrl_primal > fp_mul(adapt_ratio_ctrl, ctrl_dual) && rho_u <= FP_QP_CONST(50.0)) {
                scale_rho_u = 1;
            } else if (ctrl_dual > fp_mul(adapt_ratio_ctrl, ctrl_primal) && rho_u >= FP_QP_CONST(1.0)) {
                scale_rho_u = -1;
            }

            if (scale_rho > 0) {
                rho <<= 1;
                if (rho > FP_QP_CONST(100.0)) rho = FP_QP_CONST(100.0);
                for (k = 0; k <= N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON_PLUS_ONE max=MPC_HORIZON_PLUS_ONE
#pragma HLS PIPELINE II=1
                    for (s = 0; s < nx; s++) {
#pragma HLS UNROLL
                        y_x[k][s] >>= 1;
                    }
                }
            } else if (scale_rho < 0) {
                rho >>= 1;
                if (rho < FP_QP_CONST(0.5)) rho = FP_QP_CONST(0.5);
                for (k = 0; k <= N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON_PLUS_ONE max=MPC_HORIZON_PLUS_ONE
#pragma HLS PIPELINE II=1
                    for (s = 0; s < nx; s++) {
#pragma HLS UNROLL
                        y_x[k][s] <<= 1;
                    }
                }
            }

            if (scale_rho_u > 0) {
                rho_u <<= 1;
                if (rho_u > FP_QP_CONST(100.0)) rho_u = FP_QP_CONST(100.0);
                for (k = 0; k < N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON max=MPC_HORIZON
#pragma HLS PIPELINE II=1
                    for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
                        y_u[k][a] >>= 1;
                    }
                }
            } else if (scale_rho_u < 0) {
                rho_u >>= 1;
                if (rho_u < FP_QP_CONST(0.5)) rho_u = FP_QP_CONST(0.5);
                for (k = 0; k < N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON max=MPC_HORIZON
#pragma HLS PIPELINE II=1
                    for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
                        y_u[k][a] <<= 1;
                    }
                }
            }
        }
    } /* end ADMM loop */

    /* Save ADMM state for warm-starting — pipelined loops */
    for (k = 0; k <= N; k++) {
#pragma HLS PIPELINE II=1
        for (s = 0; s < nx; s++) {
            admm_state->z_x[k][s] = z_x[k][s];
            admm_state->y_x[k][s] = y_x[k][s];
        }
    }
    for (k = 0; k < N; k++) {
#pragma HLS PIPELINE II=1
        for (a = 0; a < nu; a++) {
            admm_state->z_u[k][a] = z_u[k][a];
            admm_state->y_u[k][a] = y_u[k][a];
        }
    }
    /* Save adapted rho for next warm-start */
    admm_state->rho = rho;
    admm_state->rho_u = rho_u;
    admm_state->initialized = 1;

    /* Output feasible controls (projected z_u, always within bounds) */
    for (k = 0; k < N; k++) {
#pragma HLS PIPELINE II=1
        for (a = 0; a < nu; a++) {
            solution->u[k][a] = z_u[k][a];
        }
    }

    solution->status = status;
    return status;
}
