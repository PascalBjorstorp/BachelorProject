/**
 * @file riccati_solver_hls.c
 * @brief Riccati-ADMM Solver — HLS-Synthesizable Implementation
 *
 * Solves constrained LQR using ADMM with Riccati recursion.
 * Exploits the 8-state augmented formulation's sparsity:
 *   - Dense block: rows/cols 0-5 (Frenet + delta_actual)
 *   - Zero block: rows/cols 6-7 (previous controls)
 *
 * Key HLS optimizations:
 *   - PIPELINE on inner matrix multiplication loops
 *   - LOOP_TRIPCOUNT for scheduling estimates
 *   - int64_t intermediates for fixed-point multiplication accuracy
 *   - No dynamic memory, no recursion, no printf
 */

#include "../include/riccati_solver_hls.h"
#include "../include/fp_math_hls.h"

/*===========================================================================
 * 64-bit Reciprocal: 1/det via Newton-Raphson (multiply-only, no division)
 *
 * Input:  det in Q16.16 (int64_t)
 * Output: 1/det in Q16.16 (int64_t)
 * Eliminates ~10,000 LUT hardware dividers from invert_2x2_hls.
 *===========================================================================*/

static int64_t reciprocal_64(int64_t det)
{
#pragma HLS INLINE
    if (det == 0) return 0;

    /* Handle sign: work with absolute value */
    int64_t sign = (det < 0) ? -1 : 1;
    int64_t abs_det = (det < 0) ? -det : det;

    /* Initial guess via leading-zero count (same approach as fp_recip) */
    int lead_zeros = 0;
    int64_t temp = abs_det;
    int lz_i;
    for (lz_i = 0; lz_i < 31; lz_i++) {
#pragma HLS LOOP_TRIPCOUNT min=1 max=31
        if (temp & 0x40000000LL) break;
        temp <<= 1;
        lead_zeros++;
    }

    int64_t est = (int64_t)1 << lead_zeros;

    /* Newton-Raphson: x_{n+1} = x_n + x_n * (1 - det * x_n)
     * 4 iterations sufficient for Q16.16 (each doubles correct bits) */
    int i;
    for (i = 0; i < 4; i++) {
#pragma HLS PIPELINE II=4
#pragma HLS LOOP_TRIPCOUNT min=4 max=4
        int64_t prod = abs_det * est;
#pragma HLS BIND_OP variable=prod op=mul impl=fabric
        prod >>= FP_FRAC_BITS;
        int64_t corr = (int64_t)FP_ONE - prod;
        int64_t adj  = est * corr;
#pragma HLS BIND_OP variable=adj op=mul impl=fabric
        adj >>= FP_FRAC_BITS;
        est = est + adj;
    }

    return (sign < 0) ? -est : est;
}

/*===========================================================================
 * 2x2 Matrix Inverse (for S = R + B^T P B)
 * Uses reciprocal_64() — multiplications only, no hardware dividers.
 *===========================================================================*/

static int invert_2x2_hls(const int64_t S[2][2], int64_t Si[2][2])
{
#pragma HLS INLINE
#pragma HLS ALLOCATION operation instances=sdiv limit=1
    int64_t det = ((S[0][0] * S[1][1]) >> FP_FRAC_BITS)
               - ((S[0][1] * S[1][0]) >> FP_FRAC_BITS);

    if (det == 0 || (det > -16 && det < 16)) {
        return -1;
    }

    int64_t inv_det = reciprocal_64(det);

    int64_t si00 = S[1][1] * inv_det;
#pragma HLS BIND_OP variable=si00 op=mul impl=fabric
    Si[0][0] =  si00 >> FP_FRAC_BITS;
    int64_t si01 = S[0][1] * inv_det;
#pragma HLS BIND_OP variable=si01 op=mul impl=fabric
    Si[0][1] = -(si01 >> FP_FRAC_BITS);
    int64_t si10 = S[1][0] * inv_det;
#pragma HLS BIND_OP variable=si10 op=mul impl=fabric
    Si[1][0] = -(si10 >> FP_FRAC_BITS);
    int64_t si11 = S[0][0] * inv_det;
#pragma HLS BIND_OP variable=si11 op=mul impl=fabric
    Si[1][1] =  si11 >> FP_FRAC_BITS;

    return 0;
}

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

static void riccati_pass_hls(
    const StepData_t step_data[MPC_HORIZON],
    const fixed_point_t *terminal_Q,
    const fixed_point_t *terminal_q,
    const fixed_point_t *x0,
    fixed_point_t rho, fixed_point_t rho_u,
    const fixed_point_t z_x[][MPC_NX_AUG],
    const fixed_point_t y_x[][MPC_NX_AUG],
    const fixed_point_t z_u[][MPC_NU],
    const fixed_point_t y_u[][MPC_NU],
    fixed_point_t x_out[][MPC_NX_AUG],
    fixed_point_t u_out[][MPC_NU])
{
    const int nx = MPC_NX_AUG;
    const int nu = MPC_NU;
    const int N = MPC_HORIZON;

    /* Gains stored for forward pass */
    fixed_point_t K[MPC_HORIZON][MPC_NU][MPC_NX_AUG];
    fixed_point_t kk[MPC_HORIZON][MPC_NU];
#pragma HLS ARRAY_PARTITION variable=K complete dim=3
#pragma HLS BIND_STORAGE variable=K type=ram_2p impl=bram
#pragma HLS ARRAY_PARTITION variable=kk complete dim=2

    /* Value function P (nx x nx) and p (nx x 1) in int64 */
    int64_t P[MPC_NX_AUG][MPC_NX_AUG];
    int64_t p[MPC_NX_AUG];
#pragma HLS ARRAY_PARTITION variable=P cyclic factor=8 dim=1
#pragma HLS ALLOCATION operation instances=mul limit=MPC_HLS_MUL_LIMIT
#pragma HLS ARRAY_PARTITION variable=P cyclic factor=2 dim=2

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
        const StepData_t *last_sd = &step_data[N - 1];
        for (s = 0; s < nx; s++) {
            /* No UNROLL: saves ~12-14 DSP by serializing terminal cost init */
            int is_con = (last_sd->x_ub[s] < BOUND_THRESHOLD ||
                          last_sd->x_lb[s] > -BOUND_THRESHOLD);
            if (is_con) {
                P[s][s] = (int64_t)terminal_Q[s] + (int64_t)rho;
                p[s] = (int64_t)terminal_q[s]
                     - (((int64_t)rho * ((int64_t)z_x[N][s] - (int64_t)y_x[N][s])) >> FP_FRAC_BITS);
            } else {
                P[s][s] = (int64_t)terminal_Q[s];
                p[s] = (int64_t)terminal_q[s];
            }
        }
    }

    /* ===== Backward pass: k = N-1 down to 0 ===== */
    for (k = N - 1; k >= 0; k--) {
#pragma HLS LOOP_TRIPCOUNT min=20 max=20
#pragma HLS LOOP_FLATTEN off
        const StepData_t *sd = &step_data[k];

        /* Buffer A 6x6 from BRAM into fully-partitioned local registers.
         * BRAM restricts sd->A to 1-2 reads/cycle; with A_local in registers
         * all 36 elements are simultaneously accessible, enabling UNROLL on
         * every s-loop below with zero extra DSPs. */
        fixed_point_t A_local[MPC_NX_DENSE][MPC_NX_DENSE];
#pragma HLS ARRAY_PARTITION variable=A_local complete dim=0
        for (i = 0; i < 6; i++) {
            for (j = 0; j < 6; j++) {
#pragma HLS PIPELINE II=1
                A_local[i][j] = sd->A[i][j];
            }
        }

        /* Augmented costs */
        int64_t Q_aug[MPC_NX_AUG];
        int64_t q_aug[MPC_NX_AUG];
        int64_t R_aug[MPC_NU];
        int64_t r_aug[MPC_NU];

        for (s = 0; s < nx; s++) {
#pragma HLS UNROLL
            int is_con = (sd->x_ub[s] < BOUND_THRESHOLD ||
                          sd->x_lb[s] > -BOUND_THRESHOLD);
            if (is_con) {
                Q_aug[s] = (int64_t)sd->Q_diag[s] + (int64_t)rho;
                q_aug[s] = (int64_t)sd->q[s]
                         - (((int64_t)rho * ((int64_t)z_x[k][s] - (int64_t)y_x[k][s])) >> FP_FRAC_BITS);
            } else {
                Q_aug[s] = (int64_t)sd->Q_diag[s];
                q_aug[s] = (int64_t)sd->q[s];
            }
        }
        for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
            R_aug[a] = (int64_t)sd->R_diag[a] + (int64_t)rho_u;
            r_aug[a] = (int64_t)sd->r[a]
                     - (((int64_t)rho_u * ((int64_t)z_u[k][a] - (int64_t)y_u[k][a])) >> FP_FRAC_BITS);
        }

        /* Step 1: M = B^T * P (nu x nx) — B col-0 sparsity: B[2..4][0]=0, only B[5][0]=dt */
        int64_t M[MPC_NU][MPC_NX_AUG];
#pragma HLS ARRAY_PARTITION variable=M complete dim=1
#pragma HLS ARRAY_PARTITION variable=M cyclic factor=4 dim=2
        for (j = 0; j < nx; j++) {
#pragma HLS PIPELINE II=1
            /* Col 0: only B[5][0] nonzero */
            int64_t s0 = (int64_t)sd->B[5][0] * P[5][j];
#pragma HLS BIND_OP variable=s0 op=mul impl=fabric
            /* Col 1: only B[2][1] = dt is nonzero */
            int64_t s1 = (int64_t)sd->B[2][1] * P[2][j];
#pragma HLS BIND_OP variable=s1 op=mul impl=fabric
            M[0][j] = (s0 >> FP_FRAC_BITS) + P[6][j];   /* B[6][0] = 1 */
            M[1][j] = (s1 >> FP_FRAC_BITS) + P[7][j];   /* B[7][1] = 1 */
        }

        /* Step 2: S = R_aug + M*B (2x2) — B col-0 sparsity: only B[5][0] nonzero */
        int64_t S[2][2];
        {
            /* Col 0: only B[5][0] */
            int64_t s00 = M[0][5] * (int64_t)sd->B[5][0];
            int64_t s10 = M[1][5] * (int64_t)sd->B[5][0];
            /* Col 1: only B[2][1] = dt is nonzero */
            int64_t s01 = M[0][2] * (int64_t)sd->B[2][1];
            int64_t s11 = M[1][2] * (int64_t)sd->B[2][1];
            S[0][0] = R_aug[0] + (s00 >> FP_FRAC_BITS) + M[0][6];
            S[0][1] = (s01 >> FP_FRAC_BITS) + M[0][7];
            S[1][0] = (s10 >> FP_FRAC_BITS) + M[1][6];
            S[1][1] = R_aug[1] + (s11 >> FP_FRAC_BITS) + M[1][7];
        }

        /* Step 3: Invert S (2x2) */
        int64_t Si[2][2];
        if (invert_2x2_hls(S, Si) < 0) {
            Si[0][0] = S[0][0] != 0 ? ((int64_t)FP_ONE << FP_FRAC_BITS) / S[0][0] : 0;
            Si[0][1] = 0; Si[1][0] = 0;
            Si[1][1] = S[1][1] != 0 ? ((int64_t)FP_ONE << FP_FRAC_BITS) / S[1][1] : 0;
        }

        /* Step 4: G = M*A + N^T (nu x nx) — exploit A sparsity */
        int64_t G[MPC_NU][MPC_NX_AUG];
#pragma HLS ARRAY_PARTITION variable=G complete dim=1
#pragma HLS ARRAY_PARTITION variable=G cyclic factor=4 dim=2
        for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
            /* Cols 0..5: M*A uses A rows 0..5 (6x6 dense block) */
            for (j = 0; j < 6; j++) {
                int64_t sum = 0;
                for (s = 0; s < 6; s++) {
                    int64_t gma_prod = M[a][s] * (int64_t)A_local[s][j];
                    sum += gma_prod;
                }
                G[a][j] = (int64_t)sd->N_cross[j][a] + (sum >> FP_FRAC_BITS);
            }
            /* Cols 6,7: A cols 6,7 are zero */
            G[a][6] = (int64_t)sd->N_cross[6][a];
            G[a][7] = (int64_t)sd->N_cross[7][a];
        }

        /* Step 5: K = -S^{-1} * G (nu x nx) */
        for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
            for (j = 0; j < nx; j++) {
                int64_t val = 0;
                for (b = 0; b < nu; b++) {
                    val += Si[a][b] * G[b][j];
                }
                val = -(val >> FP_FRAC_BITS);
                if (val > INT32_MAX) val = INT32_MAX;
                else if (val < INT32_MIN) val = INT32_MIN;
                K[k][a][j] = (fixed_point_t)val;
            }
        }

        /* Step 6: kk = -S^{-1} * (r_aug + B^T * p) — B col-0 sparsity */
        int64_t Bp[MPC_NU];
        {
            /* Col 0: only B[5][0] nonzero */
            int64_t bp0 = (int64_t)sd->B[5][0] * p[5];
            /* Col 1: only B[2][1] = dt is nonzero */
            int64_t bp1 = (int64_t)sd->B[2][1] * p[2];
            Bp[0] = (bp0 >> FP_FRAC_BITS) + p[6];
            Bp[1] = (bp1 >> FP_FRAC_BITS) + p[7];
        }

        for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
            int64_t val = 0;
            for (b = 0; b < nu; b++) {
                val += Si[a][b] * (r_aug[b] + Bp[b]);
            }
            val = -(val >> FP_FRAC_BITS);
            if (val > INT32_MAX) val = INT32_MAX;
            else if (val < INT32_MIN) val = INT32_MIN;
            kk[k][a] = (fixed_point_t)val;
        }

        /* Step 7: P = Q_diag + A^T*P*A + G^T*K  (fused) */
        /* PA = P * A (only 6x6 dense block needed) */
        /* A sparsity: col 0 has only A[0][0]=1 (rows 1-5 are 0).
         * Col 5 has up to 4 nonzero entries (steering coupling).
         * Exploit col-0 identity: PA[i][0] = P[i][0], no multiply. */
        int64_t PA[MPC_NX_DENSE][MPC_NX_DENSE];
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
#pragma HLS PIPELINE II=2
                int64_t sum = 0;
                for (s = 0; s < 6; s++) {
                    int64_t pa_prod = P[i][s] * (int64_t)sd->A[s][j];
#pragma HLS BIND_OP variable=pa_prod op=mul impl=fabric
                    sum += pa_prod;
                }
                PA[i][j] = sum >> FP_FRAC_BITS;
            }
        }

        /* Fused: P = Q_diag + A^T*PA + G^T*K (written directly) */
        /* Dense block: rows 0..5, cols 0..5 */
        for (i = 0; i < 6; i++) {
            for (j = 0; j < 6; j++) {
#pragma HLS PIPELINE II=1
                int64_t sum = 0;
                /* A^T * PA: A_local in registers (no BRAM bottleneck) */
                for (s = 0; s < 6; s++) {
#pragma HLS UNROLL factor=2
                    int64_t atpa_prod = (int64_t)A_local[s][i] * PA[s][j];
#pragma HLS BIND_OP variable=atpa_prod op=mul impl=fabric
                    sum += atpa_prod;
                }
                /* G^T * K contribution */
                for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
                    int64_t gtk_prod = G[a][i] * (int64_t)K[k][a][j];
#pragma HLS BIND_OP variable=gtk_prod op=mul impl=fabric
                    sum += gtk_prod;
                }
                P[i][j] = ((i == j) ? Q_aug[i] : 0) + (sum >> FP_FRAC_BITS);
            }
            /* Cols 6,7: AtPA=0, only G^T*K contributes */
            for (j = 6; j < nx; j++) {
                int64_t sum = 0;
                for (a = 0; a < nu; a++) {
                    sum += G[a][i] * (int64_t)K[k][a][j];
                }
                P[i][j] = sum >> FP_FRAC_BITS;
            }
        }
        /* Rows 6,7: A^T rows 6,7 zero, only G^T*K + Q_diag */
        for (i = 6; i < nx; i++) {
            for (j = 0; j < nx; j++) {
                int64_t sum = 0;
                for (a = 0; a < nu; a++) {
                    sum += G[a][i] * (int64_t)K[k][a][j];
                }
                P[i][j] = ((i == j) ? Q_aug[i] : 0) + (sum >> FP_FRAC_BITS);
            }
        }

        /* Step 8: p_new = q_aug + A^T*p + G^T*kk */
        int64_t p_new[MPC_NX_AUG];
        for (i = 0; i < 6; i++) {
            int64_t Atp = 0;
            for (s = 0; s < 6; s++) {
#pragma HLS UNROLL factor=2
                int64_t step8_prod = (int64_t)A_local[s][i] * p[s];
#pragma HLS BIND_OP variable=step8_prod op=mul impl=fabric
                Atp += step8_prod;
            }
            int64_t Gtk = 0;
            for (a = 0; a < nu; a++) {
                Gtk += G[a][i] * (int64_t)kk[k][a];
            }
            p_new[i] = q_aug[i] + (Atp >> FP_FRAC_BITS) + (Gtk >> FP_FRAC_BITS);
        }
        for (i = 6; i < nx; i++) {
#pragma HLS UNROLL
            int64_t Gtk = 0;
            for (a = 0; a < nu; a++) {
                Gtk += G[a][i] * (int64_t)kk[k][a];
            }
            p_new[i] = q_aug[i] + (Gtk >> FP_FRAC_BITS);
        }
        for (i = 0; i < nx; i++) p[i] = p_new[i];

    } /* end backward pass */

    /* ===== Forward pass: roll out x, u from x0 ===== */
    for (s = 0; s < nx; s++) {
#pragma HLS UNROLL
        x_out[0][s] = x0[s];
    }

    for (k = 0; k < N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=20 max=20
#pragma HLS LOOP_FLATTEN off
        const StepData_t *sd = &step_data[k];

        /* Buffer A locally: same register technique as backward pass.
         * Eliminates BRAM bottleneck in x_{k+1} = A*x + B*u step. */
        fixed_point_t A_fwd[MPC_NX_DENSE][MPC_NX_DENSE];
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
            int64_t sum = (int64_t)kk[k][a];
            for (s = 0; s < nx; s++) {
                int64_t kx_prod = (int64_t)K[k][a][s] * (int64_t)x_out[k][s];
#pragma HLS BIND_OP variable=kx_prod op=mul impl=fabric
                sum += kx_prod >> FP_FRAC_BITS;
            }
            if (sum > INT32_MAX) sum = INT32_MAX;
            else if (sum < INT32_MIN) sum = INT32_MIN;
            u_out[k][a] = (fixed_point_t)sum;
        }

        /* x_{k+1} = A_k * x_k + B_k * u_k
         * Dense rows 0..5: A*x + B*u */
        for (i = 0; i < 6; i++) {
            int64_t sum = 0;
            for (s = 0; s < 6; s++) {
                int64_t ax_prod = (int64_t)A_fwd[i][s] * (int64_t)x_out[k][s];
#pragma HLS BIND_OP variable=ax_prod op=mul impl=fabric
                sum += ax_prod >> FP_FRAC_BITS;
            }
            for (a = 0; a < nu; a++) {
                int64_t bu_prod = (int64_t)sd->B[i][a] * (int64_t)u_out[k][a];
#pragma HLS BIND_OP variable=bu_prod op=mul impl=fabric
                sum += bu_prod >> FP_FRAC_BITS;
            }
            if (sum > INT32_MAX) sum = INT32_MAX;
            else if (sum < INT32_MIN) sum = INT32_MIN;
            x_out[k + 1][i] = (fixed_point_t)sum;
        }
        /* Rows 6,7: x_prev = u (identity in B, zero in A) */
        x_out[k + 1][6] = u_out[k][0];
        x_out[k + 1][7] = u_out[k][1];
    }
}

/*===========================================================================
 * Main Solver: Riccati-ADMM
 *===========================================================================*/

MpcStatus_t riccati_admm_solve_hls(
    const StepData_t step_data[MPC_HORIZON],
    const fixed_point_t terminal_Q[MPC_NX_AUG],
    const fixed_point_t terminal_q[MPC_NX_AUG],
    const fixed_point_t x0[MPC_NX_AUG],
    const AdmmConfig_t *config,
    AdmmState_t *admm_state,
    MpcSolution_t *solution)
{
    const int nx = MPC_NX_AUG;
    const int nu = MPC_NU;
    const int N = MPC_HORIZON;

    /* Restore persisted rho from warm-start if available */
    fixed_point_t rho   = (admm_state->initialized && admm_state->rho > 0)
                        ? admm_state->rho : config->rho;
    fixed_point_t rho_u = (admm_state->initialized && admm_state->rho_u > 0)
                        ? admm_state->rho_u : (config->rho_u > 0 ? config->rho_u : rho);
    int max_iter = config->max_iterations;

    /* Local ADMM variables */
    fixed_point_t z_x[MPC_HORIZON + 1][MPC_NX_AUG];
    fixed_point_t z_u[MPC_HORIZON][MPC_NU];
    fixed_point_t y_x[MPC_HORIZON + 1][MPC_NX_AUG];
    fixed_point_t y_u[MPC_HORIZON][MPC_NU];
#pragma HLS BIND_STORAGE variable=z_x type=ram_2p impl=bram
#pragma HLS ARRAY_PARTITION variable=z_x cyclic factor=4 dim=2
#pragma HLS BIND_STORAGE variable=z_u type=ram_2p impl=bram
#pragma HLS ARRAY_PARTITION variable=z_u complete dim=2
#pragma HLS BIND_STORAGE variable=y_x type=ram_2p impl=bram
#pragma HLS ARRAY_PARTITION variable=y_x cyclic factor=4 dim=2
#pragma HLS BIND_STORAGE variable=y_u type=ram_2p impl=bram
#pragma HLS ARRAY_PARTITION variable=y_u complete dim=2

    /* Precompute constrained flags */
    uint8_t x_is_con[MPC_HORIZON + 1][MPC_NX_AUG];
#pragma HLS ARRAY_PARTITION variable=x_is_con complete dim=2
    int k, s, a;

    for (k = 0; k <= N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=21 max=21
        const StepData_t *sd = (k < N) ? &step_data[k] : &step_data[N - 1];
        for (s = 0; s < nx; s++) {
#pragma HLS UNROLL
            x_is_con[k][s] = (sd->x_ub[s] < BOUND_THRESHOLD ||
                               sd->x_lb[s] > -BOUND_THRESHOLD);
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
            step_data, terminal_Q, terminal_q, x0, 0, 0,
            (const fixed_point_t (*)[MPC_NX_AUG])z_x,
            (const fixed_point_t (*)[MPC_NX_AUG])y_x,
            (const fixed_point_t (*)[MPC_NU])z_u,
            (const fixed_point_t (*)[MPC_NU])y_u,
            solution->x, solution->u);

        /* Initialize z from projection of unconstrained solution */
        for (k = 0; k <= N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=21 max=21
            const StepData_t *sd = (k < N) ? &step_data[k] : &step_data[N - 1];
            for (s = 0; s < nx; s++) {
#pragma HLS UNROLL
                fixed_point_t val = solution->x[k][s];
                if (val < sd->x_lb[s]) val = sd->x_lb[s];
                if (val > sd->x_ub[s]) val = sd->x_ub[s];
                z_x[k][s] = val;
            }
        }
        for (k = 0; k < N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=20 max=20
            for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
                fixed_point_t val = solution->u[k][a];
                if (val < step_data[k].u_lb[a]) val = step_data[k].u_lb[a];
                if (val > step_data[k].u_ub[a]) val = step_data[k].u_ub[a];
                z_u[k][a] = val;
            }
        }

        /* Initialize y (dual) from constraint violation */
        for (k = 0; k <= N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=21 max=21
            for (s = 0; s < nx; s++) {
#pragma HLS UNROLL
                if (x_is_con[k][s]) {
                    y_x[k][s] = solution->x[k][s] - z_x[k][s];
                }
            }
        }
        for (k = 0; k < N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=20 max=20
            for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
                y_u[k][a] = solution->u[k][a] - z_u[k][a];
            }
        }
    }

    MpcStatus_t status = MPC_STATUS_MAX_ITER;
    int iter;

    for (iter = 0; iter < max_iter; iter++) {
#pragma HLS LOOP_TRIPCOUNT min=1 max=8 avg=2

        /* --- Primal update: Riccati pass with augmented costs --- */
        riccati_pass_hls(
            step_data, terminal_Q, terminal_q, x0, rho, rho_u,
            (const fixed_point_t (*)[MPC_NX_AUG])z_x,
            (const fixed_point_t (*)[MPC_NX_AUG])y_x,
            (const fixed_point_t (*)[MPC_NU])z_u,
            (const fixed_point_t (*)[MPC_NU])y_u,
            solution->x, solution->u);

        /* --- Fused z-update, y-update, and residual computation ---
         * Dual residual computed inline: save z_old before overwrite,
         * eliminating the z_x_old/z_u_old arrays and copy loop. */
        fixed_point_t state_primal = 0, state_dual = 0;
        fixed_point_t ctrl_primal = 0, ctrl_dual = 0;

        /* State z/y update — dual residual computed inline
         * Over-relaxation: use x_hat = alpha*x + (1-alpha)*z_old instead of x
         * in the z-update projection. This accelerates ADMM convergence. */
        for (k = 0; k <= N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=21 max=21
#pragma HLS PIPELINE II=4
            const StepData_t *sd = (k < N) ? &step_data[k] : &step_data[N - 1];
            for (s = 0; s < nx; s++) {
#pragma HLS UNROLL factor=4
                if (x_is_con[k][s]) {
                    fixed_point_t x_val = solution->x[k][s];
                    /* Over-relaxation: x_hat = x + (alpha-1)*(x - z_old)
                     * Shift for alpha=1.5: (alpha-1)=0.5 = >>1 */
                    int64_t diff_x = (int64_t)x_val - (int64_t)z_x[k][s];
                    int64_t x_hat64 = (int64_t)x_val
                                    + (diff_x >> 1);
                    int64_t val = x_hat64 + (int64_t)y_x[k][s];
                    if (val < (int64_t)sd->x_lb[s]) val = (int64_t)sd->x_lb[s];
                    if (val > (int64_t)sd->x_ub[s]) val = (int64_t)sd->x_ub[s];
                    fixed_point_t z_new = (fixed_point_t)val;
                    fixed_point_t x_hat = (fixed_point_t)x_hat64;

                    /* Dual residual: rho * (z_new - z_old), save z_old before overwrite */
                    fixed_point_t z_prev = z_x[k][s];
                    int64_t d64 = ((int64_t)rho * ((int64_t)z_new - (int64_t)z_prev)) >> FP_FRAC_BITS;
                    fixed_point_t dd = (fixed_point_t)(d64 < 0 ? -d64 : d64);
                    if (dd > state_dual) state_dual = dd;

                    /* y-update uses x_hat (over-relaxed) per Boyd et al. */
                    fixed_point_t y_new_x = (fixed_point_t)((int64_t)x_hat - (int64_t)z_new + (int64_t)y_x[k][s]);
                    /* Saturate y to prevent dual variable explosion */
                    if (y_new_x > FP_CONST(50.0)) y_new_x = FP_CONST(50.0);
                    if (y_new_x < FP_CONST(-50.0)) y_new_x = FP_CONST(-50.0);
                    y_x[k][s] = y_new_x;

                    fixed_point_t pd = x_hat - z_new;
                    if (pd < 0) pd = -pd;
                    if (pd > state_primal) state_primal = pd;

                    z_x[k][s] = z_new;
                } else {
                    z_x[k][s] = solution->x[k][s];
                }
            }
        }

        /* Control z/y update — dual residual computed inline (with over-relaxation) */
        for (k = 0; k < N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=20 max=20
#pragma HLS PIPELINE II=1
            const StepData_t *sd = &step_data[k];
            for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
                fixed_point_t u_val = solution->u[k][a];
                /* Over-relaxation: u_hat = u + (alpha-1)*(u - z_old)
                 * Shift for alpha=1.5: (alpha-1)=0.5 = >>1 */
                int64_t diff_u = (int64_t)u_val - (int64_t)z_u[k][a];
                int64_t u_hat64 = (int64_t)u_val
                                + (diff_u >> 1);
                int64_t val = u_hat64 + (int64_t)y_u[k][a];
                if (val < (int64_t)sd->u_lb[a]) val = (int64_t)sd->u_lb[a];
                if (val > (int64_t)sd->u_ub[a]) val = (int64_t)sd->u_ub[a];
                fixed_point_t z_new = (fixed_point_t)val;
                fixed_point_t u_hat = (fixed_point_t)u_hat64;

                /* Dual residual: rho_u * (z_new - z_old) */
                fixed_point_t z_prev = z_u[k][a];
                int64_t d64 = ((int64_t)rho_u * ((int64_t)z_new - (int64_t)z_prev)) >> FP_FRAC_BITS;
                fixed_point_t dd = (fixed_point_t)(d64 < 0 ? -d64 : d64);
                if (dd > ctrl_dual) ctrl_dual = dd;

                /* y-update uses u_hat (over-relaxed) per Boyd et al. */
                fixed_point_t y_new_u = (fixed_point_t)((int64_t)u_hat - (int64_t)z_new + (int64_t)y_u[k][a]);
                /* Saturate y to prevent dual variable explosion */
                if (y_new_u > FP_CONST(50.0)) y_new_u = FP_CONST(50.0);
                if (y_new_u < FP_CONST(-50.0)) y_new_u = FP_CONST(-50.0);
                y_u[k][a] = y_new_u;

                fixed_point_t pd = u_hat - z_new;
                if (pd < 0) pd = -pd;
                if (pd > ctrl_primal) ctrl_primal = pd;

                z_u[k][a] = z_new;
            }
        }

        fixed_point_t primal_res = state_primal > ctrl_primal ? state_primal : ctrl_primal;
        fixed_point_t dual_res   = state_dual > ctrl_dual ? state_dual : ctrl_dual;

        solution->iterations = iter + 1;
        solution->primal_residual = primal_res;
        solution->dual_residual = dual_res;

        /* Convergence check */
        if (primal_res <= config->tolerance && dual_res <= config->tolerance) {
            status = MPC_STATUS_OPTIMAL;
            break;
        }

        /* Adaptive rho: balance convergence rates (checked every 2 iterations) */
        if (config->adaptive_rho && iter > 0 && (iter & 1) == 0) {
            if (primal_res > 10 * dual_res && rho < FP_CONST(100.0)) {
                rho = fp_mul(rho, FP_TWO);
                if (rho_u < FP_CONST(100.0))
                    rho_u = fp_mul(rho_u, FP_TWO);
                for (k = 0; k <= N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=21 max=21
#pragma HLS PIPELINE II=1
                    for (s = 0; s < nx; s++) {
#pragma HLS UNROLL
                        y_x[k][s] >>= 1;
                    }
                }
                for (k = 0; k < N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=20 max=20
#pragma HLS PIPELINE II=1
                    for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
                        y_u[k][a] >>= 1;
                    }
                }
            } else if (dual_res > 10 * primal_res && rho > FP_CONST(0.5)) {
                rho = fp_mul(rho, FP_HALF);
                if (rho_u > FP_CONST(0.5))
                    rho_u = fp_mul(rho_u, FP_HALF);
                for (k = 0; k <= N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=21 max=21
#pragma HLS PIPELINE II=1
                    for (s = 0; s < nx; s++) {
#pragma HLS UNROLL
                        y_x[k][s] <<= 1;
                    }
                }
                for (k = 0; k < N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=20 max=20
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
