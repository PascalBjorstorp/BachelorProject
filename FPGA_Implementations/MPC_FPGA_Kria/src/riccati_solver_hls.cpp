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
#include <climits>

static fp_QP_t fp_max2(fp_QP_t a, fp_QP_t b)
{
#pragma HLS INLINE
    return (a > b) ? a : b;
}

static fp_QP_t max_abs_state8(
    fp_QP_t x0, fp_QP_t x1, fp_QP_t x2, fp_QP_t x3,
    fp_QP_t x4, fp_QP_t x5, fp_QP_t x6, fp_QP_t x7)
{
#pragma HLS INLINE
    fp_QP_t m0 = fp_max2(fp_abs(x0), fp_abs(x1));
    fp_QP_t m1 = fp_max2(fp_abs(x2), fp_abs(x3));
    fp_QP_t m2 = fp_max2(fp_abs(x4), fp_abs(x5));
    fp_QP_t m3 = fp_max2(fp_abs(x6), fp_abs(x7));
    fp_QP_t m4 = fp_max2(m0, m1);
    fp_QP_t m5 = fp_max2(m2, m3);
    return fp_max2(m4, m5);
}

static fp_QP_t max_abs_ctrl2(fp_QP_t x0, fp_QP_t x1)
{
#pragma HLS INLINE
    return fp_max2(fp_abs(x0), fp_abs(x1));
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
    const fp_QP_t *terminal_q_diag,
    const fp_QP_t *terminal_q_linear,
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
#pragma HLS ARRAY_PARTITION variable=K complete dim=2
#pragma HLS ARRAY_PARTITION variable=K cyclic factor=8 dim=3
#pragma HLS ARRAY_PARTITION variable=kk complete dim=2

    /* Value function P (nx x nx) and p (nx x 1) in int64 */
    fp_raw_acc_t P[MPC_NX_AUG][MPC_NX_AUG];
    fp_raw_acc_t p[MPC_NX_AUG];
#pragma HLS ARRAY_PARTITION variable=P complete dim=0
#pragma HLS ARRAY_PARTITION variable=p complete dim=1

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
            P[s][s] = fp_raw_acc_from_qp(terminal_q_diag[s]);
            p[s] = fp_raw_acc_from_qp(terminal_q_linear[s]);
        }
        {
            const int idx = IDX_EY;
            P[idx][idx] += fp_raw_acc_from_qp(rho);
            p[idx] -= ((fp_mul_qp_acc(fp_qp_raw_from_QP(rho), (fp_raw_acc_from_qp(z_x[N][idx]) - fp_raw_acc_from_qp(y_x[N][idx])))) >> FP_FRAC_BITS);
        }
        {
            const int idx = IDX_DELTA_ACT;
            P[idx][idx] += fp_raw_acc_from_qp(rho);
            p[idx] -= ((fp_mul_qp_acc(fp_qp_raw_from_QP(rho), (fp_raw_acc_from_qp(z_x[N][idx]) - fp_raw_acc_from_qp(y_x[N][idx])))) >> FP_FRAC_BITS);
        }
    }

    /* ===== Backward pass: k = N-1 down to 0 ===== */
    for (k = N - 1; k >= 0; k--) {
#pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON max=MPC_HORIZON
#pragma HLS LOOP_FLATTEN off
        const StepData_t *sd = &step_data[k];
        fp_QP_t A_local[MPC_NX_DENSE][MPC_NX_DENSE];
#pragma HLS ARRAY_PARTITION variable=A_local complete dim=0
        fp_qp_raw_t d_local[MPC_NX_DENSE];
#pragma HLS ARRAY_PARTITION variable=d_local complete dim=1
        for (i = 0; i < 6; i++) {
            for (j = 0; j < 6; j++) {
#pragma HLS UNROLL
                A_local[i][j] = sd->A[i][j];
            }
        }
        d_local[0] = fp_qp_raw_from_QP(sd->d0);
        d_local[1] = fp_qp_raw_from_QP(sd->d1);
        d_local[2] = fp_qp_raw_from_QP(sd->d2);
        d_local[3] = fp_qp_raw_from_QP(sd->d3);
        d_local[4] = fp_qp_raw_from_QP(sd->d4);
        d_local[5] = fp_qp_raw_from_QP(sd->d5);

        /* Augmented costs */
        fp_raw_acc_t q_aug_diag[MPC_NX_AUG];
        fp_raw_acc_t q_aug_linear[MPC_NX_AUG];
        fp_raw_acc_t r_aug_diag[MPC_NU];
        fp_raw_acc_t r_aug_linear[MPC_NU];

        for (s = 0; s < nx; s++) {
#pragma HLS UNROLL
            q_aug_diag[s] = fp_raw_acc_from_qp(sd->Q_diag[s]);
            q_aug_linear[s] = fp_raw_acc_from_qp(sd->q[s]);
        }
        {
            const int idx = IDX_EY;
            q_aug_diag[idx] += fp_raw_acc_from_qp(rho);
            fp_raw_acc_t zx_minus_yx = fp_raw_acc_from_qp(z_x[k][idx]) - fp_raw_acc_from_qp(y_x[k][idx]);
            fp_raw_acc_t rho_state_mul = fp_mul_qp_acc(fp_qp_raw_from_QP(rho), zx_minus_yx);
            q_aug_linear[idx] -= (rho_state_mul >> FP_FRAC_BITS);
        }
        {
            const int idx = IDX_DELTA_ACT;
            q_aug_diag[idx] += fp_raw_acc_from_qp(rho);
            fp_raw_acc_t zx_minus_yx = fp_raw_acc_from_qp(z_x[k][idx]) - fp_raw_acc_from_qp(y_x[k][idx]);
            fp_raw_acc_t rho_state_mul = fp_mul_qp_acc(fp_qp_raw_from_QP(rho), zx_minus_yx);
            q_aug_linear[idx] -= (rho_state_mul >> FP_FRAC_BITS);
        }
        for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
            r_aug_diag[a] = fp_raw_acc_from_qp(sd->R_diag[a]) + fp_raw_acc_from_qp(rho_u);
            fp_raw_acc_t zu_minus_yu = fp_raw_acc_from_qp(z_u[k][a]) - fp_raw_acc_from_qp(y_u[k][a]);
            fp_raw_acc_t rho_ctrl_mul = fp_mul_qp_acc(fp_qp_raw_from_QP(rho_u), zu_minus_yu);
            r_aug_linear[a] = fp_raw_acc_from_qp(sd->r[a]) - (rho_ctrl_mul >> FP_FRAC_BITS);
        }

        /* Step 1: M = B^T * P (nu x nx) — accumulate only nonzero B rows */
        fp_QP_t M[MPC_NU][MPC_NX_AUG];
#pragma HLS ARRAY_PARTITION variable=M complete dim=1
#pragma HLS ARRAY_PARTITION variable=M complete dim=2
        fp_qp_raw_t b00 = fp_qp_raw_from_QP(sd->B[IDX_DELTA_ACT][0]);
        fp_qp_raw_t b10 = fp_qp_raw_from_QP(sd->B[2][1]);
        fp_qp_raw_t b11 = fp_qp_raw_from_QP(sd->B[3][1]);
        fp_qp_raw_t b12 = fp_qp_raw_from_QP(sd->B[4][1]);
        for (j = 0; j < nx; j++) {
#pragma HLS UNROLL
            fp_raw_acc_t s0 = fp_mul_qp_acc(b00, P[IDX_DELTA_ACT][j])
                            + (P[IDX_DELTA_RATE_PREV][j] << FP_FRAC_BITS);
            fp_raw_acc_t s1 = fp_mul_qp_acc(b10, P[2][j])
                            + fp_mul_qp_acc(b11, P[3][j])
                            + fp_mul_qp_acc(b12, P[4][j])
                            + (P[IDX_ACCEL_PREV][j] << FP_FRAC_BITS);
            fp_raw_acc_t m0 = (s0 >> FP_FRAC_BITS);
            fp_raw_acc_t m1 = (s1 >> FP_FRAC_BITS);
            m0 = fp_clip_raw_to_qp(m0);
            m1 = fp_clip_raw_to_qp(m1);
            M[0][j] = fp_QP_from_qp_raw((fp_qp_raw_t)m0);
            M[1][j] = fp_QP_from_qp_raw((fp_qp_raw_t)m1);
        }

        /* Step 2: S = r_aug_diag + M*B (2x2) */
        fp_raw_acc_t S[2][2];
        {
            fp_raw_acc_t mb00 = fp_mul_acc_qp(fp_raw_acc_from_qp(M[0][IDX_DELTA_ACT]), b00)
                              + (fp_raw_acc_from_qp(M[0][IDX_DELTA_RATE_PREV]) << FP_FRAC_BITS);
            fp_raw_acc_t mb01 = fp_mul_acc_qp(fp_raw_acc_from_qp(M[0][2]), b10)
                              + fp_mul_acc_qp(fp_raw_acc_from_qp(M[0][3]), b11)
                              + fp_mul_acc_qp(fp_raw_acc_from_qp(M[0][4]), b12)
                              + (fp_raw_acc_from_qp(M[0][IDX_ACCEL_PREV]) << FP_FRAC_BITS);
            fp_raw_acc_t mb10 = fp_mul_acc_qp(fp_raw_acc_from_qp(M[1][IDX_DELTA_ACT]), b00)
                              + (fp_raw_acc_from_qp(M[1][IDX_DELTA_RATE_PREV]) << FP_FRAC_BITS);
            fp_raw_acc_t mb11 = fp_mul_acc_qp(fp_raw_acc_from_qp(M[1][2]), b10)
                              + fp_mul_acc_qp(fp_raw_acc_from_qp(M[1][3]), b11)
                              + fp_mul_acc_qp(fp_raw_acc_from_qp(M[1][4]), b12)
                              + (fp_raw_acc_from_qp(M[1][IDX_ACCEL_PREV]) << FP_FRAC_BITS);

            S[0][0] = r_aug_diag[0] + (mb00 >> FP_FRAC_BITS);
            S[0][1] = (mb01 >> FP_FRAC_BITS);
            S[1][0] = (mb10 >> FP_FRAC_BITS);
            S[1][1] = r_aug_diag[1] + (mb11 >> FP_FRAC_BITS);

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
#pragma HLS UNROLL
                    fp_raw_acc_t gma_prod = fp_mul_qp_raw(fp_qp_raw_from_QP(M[a][s]), fp_qp_raw_from_QP(A_local[s][j]));
                    sum += gma_prod;
                }
                fp_raw_acc_t g_val = sum >> FP_FRAC_BITS;
                g_val = fp_clip_raw_to_qp(g_val);
                G[a][j] = fp_QP_from_qp_raw((fp_qp_raw_t)g_val);
            }
        }
        /* Cols 6,7: A cols 6,7 are zero and N_cross has only these nonzeros. */
        G[0][6] = sd->N_delta_rate;
        G[1][6] = 0;
        G[0][7] = 0;
        G[1][7] = sd->N_accel;

        /* Step 5: K = -S^{-1} * G (nu x nx) */
        for (a = 0; a < nu; a++) {
            for (j = 0; j < nx; j++) {
#pragma HLS PIPELINE II=1
                fp_raw_acc_t val = 0;
                for (b = 0; b < nu; b++) {
#pragma HLS UNROLL
                    fp_raw_acc_t k_prod = fp_mul_acc_qp(Si[a][b], fp_qp_raw_from_QP(G[b][j]));
                    val += k_prod;
                }
                val = -(val >> FP_FRAC_BITS);
                val = fp_clip_raw_to_qp(val);
                K[k][a][j] = fp_QP_from_qp_raw((fp_qp_raw_t)val);
            }
        }

        /* Step 6: kk = -S^{-1} * (r_aug_linear + B^T * p) */
        fp_raw_acc_t p_shift[MPC_NX_AUG];
#pragma HLS ARRAY_PARTITION variable=p_shift complete dim=1
        for (i = 0; i < nx; i++) {
#pragma HLS PIPELINE II=1
            fp_raw_acc_t pd0 = fp_mul_acc_qp(P[i][0], d_local[0]);
            fp_raw_acc_t pd1 = fp_mul_acc_qp(P[i][1], d_local[1]);
            fp_raw_acc_t pd2 = fp_mul_acc_qp(P[i][2], d_local[2]);
            fp_raw_acc_t pd3 = fp_mul_acc_qp(P[i][3], d_local[3]);
            fp_raw_acc_t pd4 = fp_mul_acc_qp(P[i][4], d_local[4]);
            fp_raw_acc_t pd5 = fp_mul_acc_qp(P[i][5], d_local[5]);
            fp_raw_acc_t pd01 = pd0 + pd1;
            fp_raw_acc_t pd23 = pd2 + pd3;
            fp_raw_acc_t pd45 = pd4 + pd5;
            fp_raw_acc_t pd0123 = pd01 + pd23;
            fp_raw_acc_t pd_sum = pd0123 + pd45;
            p_shift[i] = p[i] + (pd_sum >> FP_FRAC_BITS);
        }

        fp_raw_acc_t Bp[MPC_NU];
        {
            fp_raw_acc_t ps2 = p_shift[2];
            fp_raw_acc_t ps3 = p_shift[3];
            fp_raw_acc_t ps4 = p_shift[4];
            fp_raw_acc_t ps5 = p_shift[IDX_DELTA_ACT];
            fp_raw_acc_t ps6 = p_shift[IDX_DELTA_RATE_PREV];
            fp_raw_acc_t ps7 = p_shift[IDX_ACCEL_PREV];

            fp_raw_acc_t bp0_mul = fp_mul_qp_acc(b00, ps5);
            Bp[0] = (bp0_mul + (ps6 << FP_FRAC_BITS)) >> FP_FRAC_BITS;

            fp_raw_acc_t bp1_m0 = fp_mul_qp_acc(b10, ps2);
            fp_raw_acc_t bp1_m1 = fp_mul_qp_acc(b11, ps3);
            fp_raw_acc_t bp1_m2 = fp_mul_qp_acc(b12, ps4);
            Bp[1] = (bp1_m0 + bp1_m1 + bp1_m2 + (ps7 << FP_FRAC_BITS)) >> FP_FRAC_BITS;
        }

        for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
            fp_raw_acc_t val = 0;
            for (b = 0; b < nu; b++) {
                val += fp_mul_raw_acc(Si[a][b], (r_aug_linear[b] + Bp[b]));
            }
            val = -(val >> FP_FRAC_BITS);
            val = fp_clip_raw_to_qp(val);
            kk[k][a] = fp_QP_from_qp_raw((fp_qp_raw_t)val);
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
                fp_raw_acc_t pa0 = fp_mul_acc_qp(P[i][0], fp_qp_raw_from_QP(sd->A[0][j]));
                fp_raw_acc_t pa1 = fp_mul_acc_qp(P[i][1], fp_qp_raw_from_QP(sd->A[1][j]));
                fp_raw_acc_t pa2 = fp_mul_acc_qp(P[i][2], fp_qp_raw_from_QP(sd->A[2][j]));
                fp_raw_acc_t pa3 = fp_mul_acc_qp(P[i][3], fp_qp_raw_from_QP(sd->A[3][j]));
                fp_raw_acc_t pa4 = fp_mul_acc_qp(P[i][4], fp_qp_raw_from_QP(sd->A[4][j]));
                fp_raw_acc_t pa5 = fp_mul_acc_qp(P[i][5], fp_qp_raw_from_QP(sd->A[5][j]));
                fp_raw_acc_t pa01 = pa0 + pa1;
                fp_raw_acc_t pa23 = pa2 + pa3;
                fp_raw_acc_t pa45 = pa4 + pa5;
                fp_raw_acc_t pa0123 = pa01 + pa23;
                fp_raw_acc_t sum = pa0123 + pa45;
                PA[i][j] = sum >> FP_FRAC_BITS;
            }
        }

        /* P = Q_diag + A^T*PA + G^T*K  */
        /* Dense block: rows 0..5, cols 0..5 — exploit P symmetry */
        {
            /* Upper-triangle decode tables (i,j) for 21 entries */
            static const int sym_i[21] = {0,0,0,0,0,0, 1,1,1,1,1, 2,2,2,2, 3,3,3, 4,4, 5};
            static const int sym_j[21] = {0,1,2,3,4,5, 1,2,3,4,5, 2,3,4,5, 3,4,5, 4,5, 5};
            fp_raw_acc_t P_ut_val[21];
#pragma HLS BIND_STORAGE variable=P_ut_val type=ram_1p impl=lutram

            for (int idx = 0; idx < 21; idx++) {
#pragma HLS UNROLL
                int ii = sym_i[idx];
                int jj = sym_j[idx];
                fp_raw_acc_t sum = 0;
                /* A^T * PA: A_local in registers */
                for (s = 0; s < 6; s++) {
#pragma HLS UNROLL
                    fp_raw_acc_t atpa_prod = fp_mul_qp_acc(fp_qp_raw_from_QP(A_local[s][ii]), PA[s][jj]);
                    sum += atpa_prod;
                }
                /* G^T * K contribution */
                for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
                    fp_raw_acc_t gtk_prod = fp_mul_qp_raw(fp_qp_raw_from_QP(G[a][ii]), fp_qp_raw_from_QP(K[k][a][jj]));
                    sum += gtk_prod;

                }
                fp_raw_acc_t val = ((ii == jj) ? q_aug_diag[ii] : (fp_raw_acc_t)0) + (sum >> FP_FRAC_BITS);
                P_ut_val[idx] = val;
            }

            /* Write upper-triangle and mirror in one pass to reduce loop overhead. */
            for (int idx = 0; idx < 21; idx++) {
#pragma HLS UNROLL
                int ii = sym_i[idx];
                int jj = sym_j[idx];
                fp_raw_acc_t pval = P_ut_val[idx];
                P[ii][jj] = pval;
                if (ii != jj) {
                    P[jj][ii] = pval;
                }
            }
        }
        /* Cols 6,7: AtPA=0, only G^T*K contributes */
        for (i = 0; i < 6; i++) {
            for (j = 6; j < nx; j++) {
#pragma HLS PIPELINE II=1
                fp_raw_acc_t sum = 0;
                for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
                    sum += fp_mul_qp_raw(fp_qp_raw_from_QP(G[a][i]), fp_qp_raw_from_QP(K[k][a][j]));
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
                s6 += fp_mul_qp_raw(fp_qp_raw_from_QP(G[a][6]), fp_qp_raw_from_QP(K[k][a][j]));
                s7 += fp_mul_qp_raw(fp_qp_raw_from_QP(G[a][7]), fp_qp_raw_from_QP(K[k][a][j]));
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
                fp_raw_acc_t step8_prod = fp_mul_qp_acc(fp_qp_raw_from_QP(A_local[s][i]), p_shift[s]);
                Atp += step8_prod;
            }
            fp_raw_acc_t Gtk = 0;
            for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
                Gtk += fp_mul_qp_raw(fp_qp_raw_from_QP(G[a][i]), fp_qp_raw_from_QP(kk[k][a]));
            }
            p_new[i] = q_aug_linear[i] + (Atp >> FP_FRAC_BITS) + (Gtk >> FP_FRAC_BITS);
        }
        for (i = 6; i < nx; i++) {
    #pragma HLS PIPELINE II=1
            fp_raw_acc_t Gtk = 0;
            for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
                Gtk += fp_mul_qp_raw(fp_qp_raw_from_QP(G[a][i]), fp_qp_raw_from_QP(kk[k][a]));
            }
            p_new[i] = q_aug_linear[i] + (Gtk >> FP_FRAC_BITS);
        }
        for (i = 0; i < nx; i++) {
#pragma HLS UNROLL factor=MPC_HLS_AFFINE_PNEW_UNROLL
            p[i] = p_new[i];
        }

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
        fp_raw_acc_t d_fwd[MPC_NX_DENSE];
#pragma HLS ARRAY_PARTITION variable=d_fwd complete dim=1
        for (i = 0; i < 6; i++) {
            for (j = 0; j < 6; j++) {
#pragma HLS UNROLL
                A_fwd[i][j] = sd->A[i][j];
            }
        }
        d_fwd[0] = fp_raw_acc_from_qp(sd->d0);
        d_fwd[1] = fp_raw_acc_from_qp(sd->d1);
        d_fwd[2] = fp_raw_acc_from_qp(sd->d2);
        d_fwd[3] = fp_raw_acc_from_qp(sd->d3);
        d_fwd[4] = fp_raw_acc_from_qp(sd->d4);
        d_fwd[5] = fp_raw_acc_from_qp(sd->d5);

        /* u_k = K_k * x_k + kk_k */
        for (a = 0; a < nu; a++) {
#pragma HLS UNROLL factor=MPC_HLS_AFFINE_CTRL_UNROLL
            fp_raw_acc_t prod_sum = 0;
            for (s = 0; s < nx; s++) {
    #pragma HLS UNROLL factor=MPC_HLS_UNROLL_KX_FACTOR
                prod_sum += fp_mul_qp_raw(fp_qp_raw_from_QP(K[k][a][s]), fp_qp_raw_from_QP(x_out[k][s]));
            }
            fp_raw_acc_t sum = fp_raw_acc_from_qp(kk[k][a]) + (prod_sum >> FP_FRAC_BITS);
            sum = fp_clip_raw_to_qp(sum);
            u_out[k][a] = fp_QP_from_qp_raw((fp_qp_raw_t)sum);
        }

        /* x_{k+1} = A_k * x_k + B_k * u_k
         * Dense rows 0..5: A*x + B*u */
        for (i = 0; i < 6; i++) {
#pragma HLS PIPELINE II=1
            fp_raw_acc_t sum = d_fwd[i] << FP_FRAC_BITS;
            for (s = 0; s < 6; s++) {
        #pragma HLS UNROLL
            sum += fp_mul_qp_raw(fp_qp_raw_from_QP(A_fwd[i][s]), fp_qp_raw_from_QP(x_out[k][s]));
            }
            if (i == 2) {
                sum += fp_mul_qp_raw(fp_qp_raw_from_QP(sd->B[2][1]), fp_qp_raw_from_QP(u_out[k][1]));
            } else if (i == 3) {
                sum += fp_mul_qp_raw(fp_qp_raw_from_QP(sd->B[3][1]), fp_qp_raw_from_QP(u_out[k][1]));
            } else if (i == 4) {
                sum += fp_mul_qp_raw(fp_qp_raw_from_QP(sd->B[4][1]), fp_qp_raw_from_QP(u_out[k][1]));
            } else if (i == 5) {
                sum += fp_mul_qp_raw(fp_qp_raw_from_QP(sd->B[5][0]), fp_qp_raw_from_QP(u_out[k][0]));
            }
            fp_raw_acc_t result = sum >> FP_FRAC_BITS;
            result = fp_clip_raw_to_qp(result);
            x_out[k + 1][i] = fp_QP_from_qp_raw((fp_qp_raw_t)result);
        }
        /* Rows 6,7: x_prev = u (identity in B, zero in A) */
        x_out[k + 1][IDX_DELTA_RATE_PREV] = u_out[k][0];
        x_out[k + 1][IDX_ACCEL_PREV] = u_out[k][1];
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
    if (admm_state->initialized) {
        if (admm_state->rho > 0) {
            rho = admm_state->rho;
        }
        if (admm_state->rho_u > 0) {
            rho_u = admm_state->rho_u;
        }
    }
    if (rho < FP_QP_CONST(0.5)) rho = FP_QP_CONST(0.5);
    if (rho_u < FP_QP_CONST(0.5)) rho_u = FP_QP_CONST(0.5);
    if (rho > FP_QP_CONST(100.0)) rho = FP_QP_CONST(100.0);
    if (rho_u > FP_QP_CONST(100.0)) rho_u = FP_QP_CONST(100.0);
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
    fp_QP_t sol_x[MPC_HORIZON + 1][MPC_NX_AUG];
    fp_QP_t sol_u[MPC_HORIZON][MPC_NU];
#pragma HLS ARRAY_PARTITION variable=z_x complete dim=2
#pragma HLS ARRAY_PARTITION variable=z_u complete dim=2
#pragma HLS ARRAY_PARTITION variable=y_x complete dim=2
#pragma HLS ARRAY_PARTITION variable=y_u complete dim=2
#pragma HLS ARRAY_PARTITION variable=sol_x complete dim=2
#pragma HLS ARRAY_PARTITION variable=sol_u complete dim=2

    int k, s, a;

    const bool cold_start = !admm_state->initialized;

    if (!cold_start) {
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
    }

    MpcStatus_t status = MPC_STATUS_MAX_ITER;
    solution->iterations = 0;
    const int total_passes = max_iter + (cold_start ? 1 : 0);
    int iter;

    for (iter = 0; iter < total_passes; iter++) {
#pragma HLS LOOP_TRIPCOUNT min=1 max=MPC_MAX_ADMM_PASS_COUNT avg=2
        const bool bootstrap_pass = cold_start && (iter == 0);
        const int admm_iter = cold_start ? (iter - 1) : iter;
        const fp_QP_t pass_rho = bootstrap_pass ? (fp_QP_t)0 : rho;
        const fp_QP_t pass_rho_u = bootstrap_pass ? (fp_QP_t)0 : rho_u;

        /* Shared call site prevents HLS from cloning the Riccati datapath for
         * cold-start and ADMM passes. */
        riccati_pass_hls(
            step_data, terminal_q_diag, terminal_q_linear,
            x0, pass_rho, pass_rho_u,
            (const fp_QP_t (*)[MPC_NX_AUG])z_x,
            (const fp_QP_t (*)[MPC_NX_AUG])y_x,
            (const fp_QP_t (*)[MPC_NU])z_u,
            (const fp_QP_t (*)[MPC_NU])y_u,
            sol_x, sol_u);

        if (bootstrap_pass) {
            /* Initialize z from projection of unconstrained solution. */
            for (k = 0; k <= N; k++) {
    #pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON_PLUS_ONE max=MPC_HORIZON_PLUS_ONE
                for (s = 0; s < nx; s++) {
#pragma HLS UNROLL
                    z_x[k][s] = sol_x[k][s];
                }
                {
                    const int idx = IDX_EY;
                    fp_QP_t val = sol_x[k][idx];
                    const fp_QP_t xlb = (k < N) ? step_data[k].x_lb[idx] : terminal_x_lb[idx];
                    const fp_QP_t xub = (k < N) ? step_data[k].x_ub[idx] : terminal_x_ub[idx];
                    if (val < xlb) val = xlb;
                    if (val > xub) val = xub;
                    z_x[k][idx] = val;
                }
                {
                    const int idx = IDX_DELTA_ACT;
                    fp_QP_t val = sol_x[k][idx];
                    const fp_QP_t xlb = (k < N) ? step_data[k].x_lb[idx] : terminal_x_lb[idx];
                    const fp_QP_t xub = (k < N) ? step_data[k].x_ub[idx] : terminal_x_ub[idx];
                    if (val < xlb) val = xlb;
                    if (val > xub) val = xub;
                    z_x[k][idx] = val;
                }
            }
            for (k = 0; k < N; k++) {
    #pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON max=MPC_HORIZON
                for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
                    fp_QP_t val = sol_u[k][a];
                    if (val < step_data[k].u_lb[a]) val = step_data[k].u_lb[a];
                    if (val > step_data[k].u_ub[a]) val = step_data[k].u_ub[a];
                    z_u[k][a] = val;
                }
            }

            /* Initialize y (dual) from constraint violation. */
            for (k = 0; k <= N; k++) {
    #pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON_PLUS_ONE max=MPC_HORIZON_PLUS_ONE
                y_x[k][IDX_EY] = sol_x[k][IDX_EY] - z_x[k][IDX_EY];
                y_x[k][IDX_DELTA_ACT] = sol_x[k][IDX_DELTA_ACT] - z_x[k][IDX_DELTA_ACT];
            }
            for (k = 0; k < N; k++) {
    #pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON max=MPC_HORIZON
                for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
                    y_u[k][a] = sol_u[k][a] - z_u[k][a];
                }
            }
            continue;
        }

        /* Compute primal scaling norms directly to avoid redundant candidate buffers. */
        fp_QP_t x_norm = 0;
        fp_QP_t u_norm = 0;
        for (k = 0; k <= N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON_PLUS_ONE max=MPC_HORIZON_PLUS_ONE
            fp_QP_t x_norm_k = max_abs_state8(
                sol_x[k][0], sol_x[k][1], sol_x[k][2], sol_x[k][3],
                sol_x[k][4], sol_x[k][5], sol_x[k][6], sol_x[k][7]);
            if (x_norm_k > x_norm) x_norm = x_norm_k;
        }
        for (k = 0; k < N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON max=MPC_HORIZON
            fp_QP_t u_norm_k = max_abs_ctrl2(sol_u[k][0], sol_u[k][1]);
            if (u_norm_k > u_norm) u_norm = u_norm_k;
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
            /* QP assembly only gives finite bounds to e_y and delta_actual.
             * Keep unconstrained states out of the expensive ADMM projection path. */
            for (s = 0; s < nx; s++) {
#pragma HLS UNROLL
                if (s != IDX_EY && s != IDX_DELTA_ACT) {
                    z_x[k][s] = sol_x[k][s];
                }
            }

            fp_QP_t z_norm_k = 0;
            fp_QP_t lambda_norm_k = 0;

            {
                const int idx = IDX_EY;
                fp_QP_t x_val = sol_x[k][idx];
                fp_raw_acc_t val = fp_raw_acc_from_qp(x_val) + fp_raw_acc_from_qp(y_x[k][idx]);
                const fp_QP_t xlb = (k < N) ? step_data[k].x_lb[idx] : terminal_x_lb[idx];
                const fp_QP_t xub = (k < N) ? step_data[k].x_ub[idx] : terminal_x_ub[idx];
                if (val < fp_raw_acc_from_qp(xlb)) val = fp_raw_acc_from_qp(xlb);
                if (val > fp_raw_acc_from_qp(xub)) val = fp_raw_acc_from_qp(xub);
                fp_QP_t z_new = fp_qp_from_raw_acc(val);

                fp_QP_t z_prev = z_x[k][idx];
                fp_QP_t dz_qp = fp_qp_from_raw_acc(fp_raw_acc_from_qp(z_new) - fp_raw_acc_from_qp(z_prev));
                fp_QP_t dd = fp_abs(fp_mul(rho, dz_qp));
                if (dd > state_dual) state_dual = dd;

                fp_QP_t y_new_x = fp_qp_from_raw_acc(fp_raw_acc_from_qp(x_val) - fp_raw_acc_from_qp(z_new) + fp_raw_acc_from_qp(y_x[k][idx]));
                y_x[k][idx] = y_new_x;

                fp_QP_t pd = x_val - z_new;
                if (pd < 0) pd = -pd;
                if (pd > state_primal) state_primal = pd;

                fp_QP_t abs_z = fp_abs(z_new);
                fp_QP_t abs_l = fp_abs(fp_mul(rho, y_new_x));
                if (abs_z > z_norm_k) z_norm_k = abs_z;
                if (abs_l > lambda_norm_k) lambda_norm_k = abs_l;
                z_x[k][idx] = z_new;
            }

            {
                const int idx = IDX_DELTA_ACT;
                fp_QP_t x_val = sol_x[k][idx];
                fp_raw_acc_t val = fp_raw_acc_from_qp(x_val) + fp_raw_acc_from_qp(y_x[k][idx]);
                const fp_QP_t xlb = (k < N) ? step_data[k].x_lb[idx] : terminal_x_lb[idx];
                const fp_QP_t xub = (k < N) ? step_data[k].x_ub[idx] : terminal_x_ub[idx];
                if (val < fp_raw_acc_from_qp(xlb)) val = fp_raw_acc_from_qp(xlb);
                if (val > fp_raw_acc_from_qp(xub)) val = fp_raw_acc_from_qp(xub);
                fp_QP_t z_new = fp_qp_from_raw_acc(val);

                fp_QP_t z_prev = z_x[k][idx];
                fp_QP_t dz_qp = fp_qp_from_raw_acc(fp_raw_acc_from_qp(z_new) - fp_raw_acc_from_qp(z_prev));
                fp_QP_t dd = fp_abs(fp_mul(rho, dz_qp));
                if (dd > state_dual) state_dual = dd;

                fp_QP_t y_new_x = fp_qp_from_raw_acc(fp_raw_acc_from_qp(x_val) - fp_raw_acc_from_qp(z_new) + fp_raw_acc_from_qp(y_x[k][idx]));
                y_x[k][idx] = y_new_x;

                fp_QP_t pd = x_val - z_new;
                if (pd < 0) pd = -pd;
                if (pd > state_primal) state_primal = pd;

                fp_QP_t abs_z = fp_abs(z_new);
                fp_QP_t abs_l = fp_abs(fp_mul(rho, y_new_x));
                if (abs_z > z_norm_k) z_norm_k = abs_z;
                if (abs_l > lambda_norm_k) lambda_norm_k = abs_l;
                z_x[k][idx] = z_new;
            }

            if (z_norm_k > z_norm) z_norm = z_norm_k;
            if (lambda_norm_k > lambda_norm) lambda_norm = lambda_norm_k;
        }

        /* Control z/y update — dual residual computed inline */
        for (k = 0; k < N; k++) {
    #pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON max=MPC_HORIZON
#pragma HLS PIPELINE II=MPC_HLS_CTRL_ZY_II
            const StepData_t *sd = &step_data[k];
            fp_QP_t z_norm_k = z_norm;
            fp_QP_t lambda_norm_k = lambda_norm;
            for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
                fp_QP_t u_val = sol_u[k][a];
                fp_raw_acc_t u_hat_raw = fp_raw_acc_from_qp(u_val);
                fp_raw_acc_t val = u_hat_raw + fp_raw_acc_from_qp(y_u[k][a]);
                if (val < fp_raw_acc_from_qp(sd->u_lb[a])) val = fp_raw_acc_from_qp(sd->u_lb[a]);
                if (val > fp_raw_acc_from_qp(sd->u_ub[a])) val = fp_raw_acc_from_qp(sd->u_ub[a]);
                fp_QP_t z_new = fp_qp_from_raw_acc(val);

                /* Dual residual: rho_u * (z_new - z_old) */
                fp_QP_t z_prev = z_u[k][a];
                fp_raw_acc_t dz_raw = fp_raw_acc_from_qp(z_new) - fp_raw_acc_from_qp(z_prev);
                fp_QP_t dz_qp = fp_qp_from_raw_acc(dz_raw);
                fp_QP_t dd = fp_abs(fp_mul(rho_u, dz_qp));
                if (dd > ctrl_dual) ctrl_dual = dd;

                fp_QP_t y_new_u = fp_qp_from_raw_acc(fp_raw_acc_from_qp(u_val) - fp_raw_acc_from_qp(z_new) + fp_raw_acc_from_qp(y_u[k][a]));
                y_u[k][a] = y_new_u;

                fp_QP_t pd = u_val - z_new;
                if (pd < 0) pd = -pd;
                if (pd > ctrl_primal) ctrl_primal = pd;

                fp_QP_t abs_z = fp_abs(z_new);
                if (abs_z > z_norm_k) z_norm_k = abs_z;
                fp_QP_t abs_l = fp_abs(fp_mul(rho_u, y_new_u));
                if (abs_l > lambda_norm_k) lambda_norm_k = abs_l;

                z_u[k][a] = z_new;
            }
            if (z_norm_k > z_norm) z_norm = z_norm_k;
            if (lambda_norm_k > lambda_norm) lambda_norm = lambda_norm_k;
        }

        fp_QP_t primal_res = state_primal > ctrl_primal ? state_primal : ctrl_primal;
        fp_QP_t dual_res   = state_dual > ctrl_dual ? state_dual : ctrl_dual;
        fp_QP_t max_xu = (x_norm > u_norm) ? x_norm : u_norm;
        fp_QP_t rel_primal_xu = fp_mul(rel_tolerance, max_xu);
        fp_QP_t rel_primal_z  = fp_mul(rel_tolerance, z_norm);
        fp_QP_t rel_primal = (rel_primal_xu > rel_primal_z) ? rel_primal_xu : rel_primal_z;
        fp_QP_t eps_primal = abs_tolerance + rel_primal;
        fp_QP_t eps_dual   = abs_tolerance + fp_mul(rel_tolerance, lambda_norm);

        solution->iterations = admm_iter + 1;
        solution->primal_residual = primal_res;
        solution->dual_residual = dual_res;

        /* Convergence check */
        if (primal_res <= eps_primal && dual_res <= eps_dual) {
            status = MPC_STATUS_OPTIMAL;
            break;
        }

        /* Adaptive rho: rebalance state and control channels independently. */
        if (config->adaptive_rho && admm_iter > 0) {
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
                    y_x[k][IDX_EY] >>= 1;
                    y_x[k][IDX_DELTA_ACT] >>= 1;
                }
            } else if (scale_rho < 0) {
                rho >>= 1;
                if (rho < FP_QP_CONST(0.5)) rho = FP_QP_CONST(0.5);
                for (k = 0; k <= N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON_PLUS_ONE max=MPC_HORIZON_PLUS_ONE
#pragma HLS PIPELINE II=1
                    y_x[k][IDX_EY] <<= 1;
                    y_x[k][IDX_DELTA_ACT] <<= 1;
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

    solution->status = status;
    return status;
}
