/**
 * @file riccati_solver_hls.c
 * @brief Riccati-ADMM Solver — HLS-Synthesizable Implementation
 * @details Solves constrained LQR using ADMM with Riccati recursion and
 *          fixed-size matrices tailored for FPGA synthesis. The implementation
 *          exploits the augmented-state sparsity pattern to reduce arithmetic
 *          and memory pressure in backward and forward passes.
 * @dependencies riccati_solver_hls.h, fp_math_hls.h
 */

#include "../include/riccati_solver_hls.h"
#include "../include/fp_math_hls.h"

/* Performance-tuning knobs (math unchanged). Override via compile flags.
 * For maximum speed with unlimited resources, set all unroll factors to
 * complete (6) and partition modes to complete (0). */
#ifndef MPC_HLS_RECIP_II
#define MPC_HLS_RECIP_II 2
#endif

#ifndef MPC_HLS_STATE_UPDATE_II
#define MPC_HLS_STATE_UPDATE_II 1
#endif

#ifndef MPC_HLS_UNROLL_GMA_FACTOR
#define MPC_HLS_UNROLL_GMA_FACTOR 6
#endif

#ifndef MPC_HLS_UNROLL_PA_FACTOR
#define MPC_HLS_UNROLL_PA_FACTOR 6
#endif

#ifndef MPC_HLS_UNROLL_ATPA_FACTOR
#define MPC_HLS_UNROLL_ATPA_FACTOR 6
#endif

#ifndef MPC_HLS_UNROLL_SYM_FACTOR
#define MPC_HLS_UNROLL_SYM_FACTOR 1
#endif

#ifndef MPC_HLS_UNROLL_FORWARD_FACTOR
#define MPC_HLS_UNROLL_FORWARD_FACTOR 8
#endif

#ifndef MPC_HLS_UNROLL_STATE_FACTOR
#define MPC_HLS_UNROLL_STATE_FACTOR 8
#endif

#ifndef MPC_HLS_K_PARTITION_MODE
#define MPC_HLS_K_PARTITION_MODE 0
#endif

#ifndef MPC_HLS_K_PARTITION_FACTOR
#define MPC_HLS_K_PARTITION_FACTOR 8
#endif

#ifndef MPC_HLS_CTRL_EARLY_EXIT_MIN_ITER
#define MPC_HLS_CTRL_EARLY_EXIT_MIN_ITER 2
#endif

#ifndef MPC_HLS_CTRL_EARLY_EXIT_THRESH
#define MPC_HLS_CTRL_EARLY_EXIT_THRESH FP_CONST(0.001)
#endif

/*===========================================================================
 * 64-bit Reciprocal: 1/det via Newton-Raphson (Resource-Optimized)
 *
 * Input:  det in Q16.16 (int64_t)
 * Output: 1/det in Q16.16 (int64_t)
 *
 * Optimization: Reduced to 3 iterations with DSP allocation limit.
 * Uses 4-cycle latency for timing closure at higher clocks.
 *===========================================================================*/

static int64_t reciprocal_64(int64_t det)
{
#pragma HLS INLINE
#pragma HLS ALLOCATION operation instances=mul limit=6

    if (det == 0) return 0;

    /* Handle sign: work with absolute value */
    int64_t sign = (det < 0) ? -1 : 1;
    int64_t abs_det = (det < 0) ? ((det == INT64_MIN) ? INT64_MAX : -det) : det;

    /* Initial guess via leading-zero count — single-cycle priority encoder.
     * For Q16.16 in int64_t: true 1/det ≈ 2^(32-p) where p = MSB position.
     * clzll = 63 - p, so 1/det ≈ 2^(clzll-31). Use 2^(clzll-32) for safe
     * underestimate keeping det*x_0 ∈ [0.5, 1.0]. */
    int lead_zeros = __builtin_clzll((unsigned long long)abs_det) - 32;
    if (lead_zeros < 0) lead_zeros = 0;
    if (lead_zeros > 30) lead_zeros = 30;

    int64_t est = (int64_t)1 << lead_zeros;

    /* Newton-Raphson: x_{n+1} = x_n + x_n * (1 - det * x_n)
     * Use 2 iterations to cut reciprocal latency while retaining
     * sufficient accuracy for the 2x2 S^{-1} path.
     */

    /* Iteration 1 */
    int64_t prod1 = abs_det * est;
    int64_t corr1 = (int64_t)FP_ONE - (prod1 >> FP_FRAC_BITS);
    int64_t adj1 = est * corr1;
    int64_t est1 = est + (adj1 >> FP_FRAC_BITS);

    /* Iteration 2 */
    int64_t prod2 = abs_det * est1;
    int64_t corr2 = (int64_t)FP_ONE - (prod2 >> FP_FRAC_BITS);
    int64_t adj2 = est1 * corr2;
    int64_t est2 = est1 + (adj2 >> FP_FRAC_BITS);

    return (sign < 0) ? -est2 : est2;
}

/*===========================================================================
 * 2x2 Matrix Inverse (for S = R + B^T P B)
 *===========================================================================*/

static int invert_2x2_hls(int64_t S[2][2], int64_t Si[2][2])
{
#pragma HLS INLINE
#pragma HLS ALLOCATION operation instances=mul limit=4
    int64_t det = ((S[0][0] * S[1][1]) >> FP_FRAC_BITS)
               - ((S[0][1] * S[1][0]) >> FP_FRAC_BITS);

    if (det == 0 || (det > -16 && det < 16)) {
        return -1;
    }

    int64_t inv_det = reciprocal_64(det);

    int64_t si00 = S[1][1] * inv_det;
    Si[0][0] =  si00 >> FP_FRAC_BITS;
    int64_t si01 = S[0][1] * inv_det;
    Si[0][1] = -(si01 >> FP_FRAC_BITS);
    int64_t si10 = S[1][0] * inv_det;
    Si[1][0] = -(si10 >> FP_FRAC_BITS);
    int64_t si11 = S[0][0] * inv_det;
    Si[1][1] =  si11 >> FP_FRAC_BITS;

    return 0;
}

/* Narrow int64 Q16.16 intermediates to fixed_point_t with saturation.
 * This shortens multiplier operand widths in hot loops (32x32 instead of 64x32). */
static inline fixed_point_t narrow_q16_sat(int64_t v)
{
#pragma HLS INLINE
    if (v > INT32_MAX) return INT32_MAX;
    if (v < INT32_MIN) return INT32_MIN;
    return (fixed_point_t)v;
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
    const fixed_point_t A_dense[MPC_HORIZON][MPC_NX_DENSE][MPC_NX_DENSE],
    const fixed_point_t b50_hist[MPC_HORIZON],
    const fixed_point_t b21_hist[MPC_HORIZON],
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
#if MPC_HLS_K_PARTITION_MODE == 0
#pragma HLS ARRAY_PARTITION variable=K complete dim=3
#else
#pragma HLS ARRAY_PARTITION variable=K cyclic factor=MPC_HLS_K_PARTITION_FACTOR dim=3
#endif
#pragma HLS ARRAY_PARTITION variable=K complete dim=2
#pragma HLS BIND_STORAGE variable=K type=ram_2p impl=bram latency=2
#pragma HLS ARRAY_PARTITION variable=kk complete dim=2

    /* Value function P (nx x nx) and p (nx x 1) in int64 */
    int64_t P[MPC_NX_AUG][MPC_NX_AUG];
    int64_t p[MPC_NX_AUG];
#pragma HLS ARRAY_PARTITION variable=P complete dim=0
#pragma HLS ARRAY_PARTITION variable=p complete dim=0
/* Force P into registers for minimum latency (investigation doc 7.3) */
#pragma HLS BIND_STORAGE variable=P type=register
#pragma HLS BIND_STORAGE variable=p type=register

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
#pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON max=MPC_HORIZON
#pragma HLS LOOP_FLATTEN off
        const StepData_t *sd = &step_data[k];
        const fixed_point_t (*A_local)[MPC_NX_DENSE] = A_dense[k];

        /* Augmented costs */
        int64_t Q_aug[MPC_NX_AUG];
        int64_t q_aug[MPC_NX_AUG];
        int64_t R_aug[MPC_NU];
        int64_t r_aug[MPC_NU];
#pragma HLS ARRAY_PARTITION variable=Q_aug complete dim=0
#pragma HLS ARRAY_PARTITION variable=q_aug complete dim=0
#pragma HLS ARRAY_PARTITION variable=R_aug complete dim=0
#pragma HLS ARRAY_PARTITION variable=r_aug complete dim=0

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

        const fixed_point_t b50 = b50_hist[k];
        const fixed_point_t b21 = b21_hist[k];

        /* Step 1: M = B^T * P (nu x nx) — B col-0 sparsity: B[2..4][0]=0, only B[5][0]=dt */
        fixed_point_t M[MPC_NU][MPC_NX_AUG];
#pragma HLS ARRAY_PARTITION variable=M complete dim=0
        for (j = 0; j < nx; j++) {
#pragma HLS PIPELINE II=1
            /* Col 0: only B[5][0] nonzero */
            fixed_point_t p5j = narrow_q16_sat(P[5][j]);
            int64_t s0 = (int64_t)b50 * (int64_t)p5j;
            /* Col 1: only B[2][1] = dt is nonzero */
            fixed_point_t p2j = narrow_q16_sat(P[2][j]);
            int64_t s1 = (int64_t)b21 * (int64_t)p2j;
            int64_t m0 = (s0 >> FP_FRAC_BITS) + (int64_t)narrow_q16_sat(P[6][j]);
            int64_t m1 = (s1 >> FP_FRAC_BITS) + (int64_t)narrow_q16_sat(P[7][j]);
            if (m0 > INT32_MAX) m0 = INT32_MAX;
            else if (m0 < INT32_MIN) m0 = INT32_MIN;
            if (m1 > INT32_MAX) m1 = INT32_MAX;
            else if (m1 < INT32_MIN) m1 = INT32_MIN;
            M[0][j] = (fixed_point_t)m0;   /* B[6][0] = 1 */
            M[1][j] = (fixed_point_t)m1;   /* B[7][1] = 1 */
        }

        /* Step 2: S = R_aug + M*B (2x2) — B col-0 sparsity: only B[5][0] nonzero */
        int64_t S[2][2];
#pragma HLS ARRAY_PARTITION variable=S complete dim=0
        {
            /* Col 0: only B[5][0] */
            int64_t s00 = M[0][5] * (int64_t)b50;
            int64_t s10 = M[1][5] * (int64_t)b50;
            /* Col 1: only B[2][1] = dt is nonzero */
            int64_t s01 = M[0][2] * (int64_t)b21;
            int64_t s11 = M[1][2] * (int64_t)b21;
            S[0][0] = R_aug[0] + (s00 >> FP_FRAC_BITS) + M[0][6];
            S[0][1] = (s01 >> FP_FRAC_BITS) + M[0][7];
            S[1][0] = (s10 >> FP_FRAC_BITS) + M[1][6];
            S[1][1] = R_aug[1] + (s11 >> FP_FRAC_BITS) + M[1][7];
        }

        /* Step 3: Invert S (2x2) */
        int64_t Si[2][2];
#pragma HLS ARRAY_PARTITION variable=Si complete dim=0
        if (invert_2x2_hls(S, Si) < 0) {
            /* Near-singular safeguard for S:
             * use a diagonal approximation of S^{-1} via Newton-Raphson reciprocal.
             * This preserves numerical progress when det(S) is very small
             * and avoids synthesizing a sequential hardware divider. */
            Si[0][0] = S[0][0] != 0 ? reciprocal_64(S[0][0]) : 0;
            Si[0][1] = 0; Si[1][0] = 0;
            Si[1][1] = S[1][1] != 0 ? reciprocal_64(S[1][1]) : 0;
        }

        /* Step 4: G = M*A + N^T (nu x nx) — exploit A sparsity */
        fixed_point_t G[MPC_NU][MPC_NX_AUG];
#pragma HLS ARRAY_PARTITION variable=G complete dim=0
        for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
            /* Cols 0..5: M*A uses A rows 0..5 (6x6 dense block) */
            for (j = 0; j < 6; j++) {
#pragma HLS UNROLL
                int64_t sum = 0;
                for (s = 0; s < 6; s++) {
#pragma HLS UNROLL factor=MPC_HLS_UNROLL_GMA_FACTOR
                    int64_t gma_prod = M[a][s] * (int64_t)A_local[s][j];
#pragma HLS BIND_OP variable=gma_prod op=mul impl=dsp latency=3
                    sum += gma_prod;
                }
                int64_t g_val = (int64_t)sd->N_cross[j][a] + (sum >> FP_FRAC_BITS);
                if (g_val > INT32_MAX) g_val = INT32_MAX;
                else if (g_val < INT32_MIN) g_val = INT32_MIN;
                G[a][j] = (fixed_point_t)g_val;
            }
            /* Cols 6,7: A cols 6,7 are zero */
            G[a][6] = sd->N_cross[6][a];
            G[a][7] = sd->N_cross[7][a];
        }

        /* Step 5: K = -S^{-1} * G (nu x nx) */
        for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
            for (j = 0; j < nx; j++) {
#pragma HLS UNROLL
                int64_t val = 0;
                for (b = 0; b < nu; b++) {
#pragma HLS UNROLL
                    int64_t k_prod = Si[a][b] * G[b][j];
#pragma HLS BIND_OP variable=k_prod op=mul impl=dsp latency=3
                    val += k_prod;
                }
                val = -(val >> FP_FRAC_BITS);
                if (val > INT32_MAX) val = INT32_MAX;
                else if (val < INT32_MIN) val = INT32_MIN;
                K[k][a][j] = (fixed_point_t)val;
            }
        }

        /* Step 6: kk = -S^{-1} * (r_aug + B^T * p) — B col-0 sparsity */
        int64_t Bp[MPC_NU];
#pragma HLS ARRAY_PARTITION variable=Bp complete dim=0
        {
            /* Col 0: only B[5][0] nonzero */
            int64_t bp0 = (int64_t)b50 * p[5];
            /* Col 1: only B[2][1] = dt is nonzero */
            int64_t bp1 = (int64_t)b21 * p[2];
            Bp[0] = (bp0 >> FP_FRAC_BITS) + p[6];
            Bp[1] = (bp1 >> FP_FRAC_BITS) + p[7];
        }

        for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
            int64_t val = 0;
            for (b = 0; b < nu; b++) {
#pragma HLS UNROLL
                int64_t kk_prod = Si[a][b] * (r_aug[b] + Bp[b]);
#pragma HLS BIND_OP variable=kk_prod op=mul impl=dsp latency=3
                val += kk_prod;
            }
            val = -(val >> FP_FRAC_BITS);
            if (val > INT32_MAX) val = INT32_MAX;
            else if (val < INT32_MIN) val = INT32_MIN;
            kk[k][a] = (fixed_point_t)val;
        }

        /* Step 7: P = Q_diag + A^T*P*A + G^T*K */
        /* PA = P * A (only 6x6 dense block needed) */
        /* A sparsity: col 0 has only A[0][0]=1 (rows 1-5 are 0).
         * Col 5 has up to 4 nonzero entries (steering coupling).
         * Exploit col-0 identity: PA[i][0] = P[i][0], no multiply. */
        int64_t PA[MPC_NX_DENSE][MPC_NX_DENSE];
#pragma HLS ARRAY_PARTITION variable=PA complete dim=0
        /* PA = P*A on dense 6x6 block with full inner-loop unrolling. */
        for (i = 0; i < 6; i++) {
#pragma HLS PIPELINE II=1
            for (j = 0; j < 6; j++) {
#pragma HLS UNROLL
                if (j == 0) {
                    /* A col-0 is identity on row 0, zero otherwise. */
                    PA[i][0] = (int64_t)narrow_q16_sat(P[i][0]);
                } else {
                    int64_t sum = 0;
                    for (s = 0; s < 6; s++) {
#pragma HLS UNROLL
                        fixed_point_t p_is = narrow_q16_sat(P[i][s]);
                        int64_t pa_prod = (int64_t)p_is * (int64_t)A_local[s][j];
#pragma HLS BIND_OP variable=pa_prod op=mul impl=dsp latency=3
                        sum += pa_prod;
                    }
                    PA[i][j] = sum >> FP_FRAC_BITS;
                }
            }
        }

        /* P = Q_diag + A^T*PA + G^T*K  on dense 6x6 upper-triangle. */
        for (i = 0; i < 6; i++) {
#pragma HLS PIPELINE II=1
            for (j = 0; j < 6; j++) {
#pragma HLS UNROLL
                if (j >= i) {
                    int64_t sum = 0;
                    for (s = 0; s < 6; s++) {
#pragma HLS UNROLL
                        int64_t atpa_prod = (int64_t)A_local[s][i] * PA[s][j];
#pragma HLS BIND_OP variable=atpa_prod op=mul impl=dsp latency=3
                        sum += atpa_prod;
                    }
                    for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
                        int64_t gtk_prod = G[a][i] * (int64_t)K[k][a][j];
#pragma HLS BIND_OP variable=gtk_prod op=mul impl=dsp latency=3
                        sum += gtk_prod;
                    }
                    int64_t val = ((i == j) ? Q_aug[i] : 0) + (sum >> FP_FRAC_BITS);
                    P[i][j] = val;
                    if (i != j) P[j][i] = val;
                }
            }
        }
        /* Cols 6,7: AtPA=0, only G^T*K contributes */
        for (i = 0; i < 6; i++) {
            for (j = 6; j < nx; j++) {
#pragma HLS PIPELINE II=1
                int64_t sum = 0;
                for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
                    int64_t gk_prod = G[a][i] * (int64_t)K[k][a][j];
#pragma HLS BIND_OP variable=gk_prod op=mul impl=dsp latency=3
                    sum += gk_prod;
                }
                P[i][j] = sum >> FP_FRAC_BITS;
            }
        }
        /* Rows 6,7: A^T rows 6,7 zero, only G^T*K + Q_diag.
         * Exploit N_cross sparsity: G[1][6]=0, G[0][7]=0 */
        for (j = 0; j < nx; j++) {
#pragma HLS PIPELINE II=1
            int64_t s6 = G[0][6] * (int64_t)K[k][0][j];
            P[6][j] = ((j == 6) ? Q_aug[6] : 0) + (s6 >> FP_FRAC_BITS);
            int64_t s7 = G[1][7] * (int64_t)K[k][1][j];
            P[7][j] = ((j == 7) ? Q_aug[7] : 0) + (s7 >> FP_FRAC_BITS);
        }

        /* Step 8: p_new = q_aug + A^T*p + G^T*kk */
        int64_t p_new[MPC_NX_AUG];
#pragma HLS ARRAY_PARTITION variable=p_new complete dim=0
        for (i = 0; i < 6; i++) {
    #pragma HLS PIPELINE II=1
            int64_t Atp = 0;
            for (s = 0; s < 6; s++) {
#pragma HLS UNROLL
                int64_t step8_prod = (int64_t)A_local[s][i] * p[s];
                Atp += step8_prod;
            }
            int64_t Gtk = 0;
            for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
                Gtk += G[a][i] * (int64_t)kk[k][a];
            }
            p_new[i] = q_aug[i] + (Atp >> FP_FRAC_BITS) + (Gtk >> FP_FRAC_BITS);
        }
        for (i = 6; i < nx; i++) {
    #pragma HLS PIPELINE II=1
            int64_t Gtk = 0;
            for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
                Gtk += G[a][i] * (int64_t)kk[k][a];
            }
            p_new[i] = q_aug[i] + (Gtk >> FP_FRAC_BITS);
        }
        for (i = 0; i < nx; i++) {
#pragma HLS UNROLL
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
        const fixed_point_t (*A_fwd)[MPC_NX_DENSE] = A_dense[k];

        /* Prefetch K[k] into registers when K is cyclic-partitioned. */
        fixed_point_t K_local[MPC_NU][MPC_NX_AUG];
#pragma HLS ARRAY_PARTITION variable=K_local complete dim=0
        for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
            for (s = 0; s < nx; s++) {
#pragma HLS UNROLL
                K_local[a][s] = K[k][a][s];
            }
        }

        /* u_k = K_k * x_k + kk_k */
        for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
            int64_t prod_sum = 0;
            for (s = 0; s < nx; s++) {
    #pragma HLS UNROLL
                prod_sum += (int64_t)K_local[a][s] * (int64_t)x_out[k][s];
            }
            int64_t sum = (int64_t)kk[k][a] + (prod_sum >> FP_FRAC_BITS);
            if (sum > INT32_MAX) sum = INT32_MAX;
            else if (sum < INT32_MIN) sum = INT32_MIN;
            u_out[k][a] = (fixed_point_t)sum;
        }

        /* x_{k+1} = A_k * x_k + B_k * u_k
         * Dense rows 0..5: A*x + B*u */
        for (i = 0; i < 6; i++) {
#pragma HLS UNROLL
            int64_t sum = 0;
            for (s = 0; s < 6; s++) {
    #pragma HLS UNROLL
                sum += (int64_t)A_fwd[i][s] * (int64_t)x_out[k][s];
            }
            for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
                sum += (int64_t)sd->B[i][a] * (int64_t)u_out[k][a];
            }
            int64_t result = sum >> FP_FRAC_BITS;
            if (result > INT32_MAX) result = INT32_MAX;
            else if (result < INT32_MIN) result = INT32_MIN;
            x_out[k + 1][i] = (fixed_point_t)result;
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
#ifndef MPC_HLS_TARGET
    if (!step_data || !terminal_Q || !terminal_q || !x0 ||
        !config || !admm_state || !solution) {
        if (solution) {
            solution->iterations = 0;
            solution->primal_residual = 0;
            solution->dual_residual = 0;
            solution->status = MPC_STATUS_ERROR;
        }
        return MPC_STATUS_ERROR;
    }
#endif

    const int nx = MPC_NX_AUG;
    const int nu = MPC_NU;
    const int N = MPC_HORIZON;

    /* Step data SoA cache for hot-path fields used by Riccati recursions. */
    fixed_point_t A_dense[MPC_HORIZON][MPC_NX_DENSE][MPC_NX_DENSE];
    fixed_point_t b50_hist[MPC_HORIZON];
    fixed_point_t b21_hist[MPC_HORIZON];
#pragma HLS ARRAY_PARTITION variable=A_dense complete dim=2
#pragma HLS ARRAY_PARTITION variable=A_dense complete dim=3
#pragma HLS BIND_STORAGE variable=A_dense type=ram_2p impl=bram latency=2

    int k, s, a;
    for (k = 0; k < N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON max=MPC_HORIZON
#pragma HLS PIPELINE II=1
        for (int ii = 0; ii < MPC_NX_DENSE; ii++) {
#pragma HLS UNROLL
            for (int jj = 0; jj < MPC_NX_DENSE; jj++) {
#pragma HLS UNROLL
                A_dense[k][ii][jj] = step_data[k].A[ii][jj];
            }
        }
        b50_hist[k] = step_data[k].B[5][0];
        b21_hist[k] = step_data[k].B[2][1];
    }

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
#pragma HLS ARRAY_PARTITION variable=z_x complete dim=2
#pragma HLS BIND_STORAGE variable=z_u type=ram_2p impl=bram latency=2
#pragma HLS ARRAY_PARTITION variable=z_u complete dim=2
#pragma HLS ARRAY_PARTITION variable=y_x complete dim=2
#pragma HLS BIND_STORAGE variable=y_u type=ram_2p impl=bram latency=2
#pragma HLS ARRAY_PARTITION variable=y_u complete dim=2

    /* Precompute constrained flags */
    uint8_t x_con_mask[MPC_HORIZON + 1];
#pragma HLS ARRAY_PARTITION variable=x_con_mask complete dim=1

    for (k = 0; k <= N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON_PLUS_ONE max=MPC_HORIZON_PLUS_ONE
        const StepData_t *sd = (k < N) ? &step_data[k] : &step_data[N - 1];
        uint8_t mask = 0;
        for (s = 0; s < nx; s++) {
#pragma HLS UNROLL
            uint8_t is_con = (sd->x_ub[s] < BOUND_THRESHOLD ||
                              sd->x_lb[s] > -BOUND_THRESHOLD) ? 1 : 0;
            mask |= (uint8_t)(is_con << s);
        }
        x_con_mask[k] = mask;
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
            step_data, A_dense, b50_hist, b21_hist, terminal_Q, terminal_q, x0, 0, 0,
            (const fixed_point_t (*)[MPC_NX_AUG])z_x,
            (const fixed_point_t (*)[MPC_NX_AUG])y_x,
            (const fixed_point_t (*)[MPC_NU])z_u,
            (const fixed_point_t (*)[MPC_NU])y_u,
            solution->x, solution->u);

        /* Initialize z from projection of unconstrained solution */
        for (k = 0; k <= N; k++) {
    #pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON_PLUS_ONE max=MPC_HORIZON_PLUS_ONE
            const StepData_t *sd = (k < N) ? &step_data[k] : &step_data[N - 1];
            for (s = 0; s < nx; s++) {
#pragma HLS PIPELINE II=1
                fixed_point_t val = solution->x[k][s];
                if (val < sd->x_lb[s]) val = sd->x_lb[s];
                if (val > sd->x_ub[s]) val = sd->x_ub[s];
                z_x[k][s] = val;
            }
        }
        for (k = 0; k < N; k++) {
    #pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON max=MPC_HORIZON
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
    #pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON_PLUS_ONE max=MPC_HORIZON_PLUS_ONE
            uint8_t mask = x_con_mask[k];
            for (s = 0; s < nx; s++) {
#pragma HLS UNROLL
                if ((mask >> s) & 1U) {
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
#pragma HLS LOOP_TRIPCOUNT min=1 max=8 avg=2

        fixed_point_t z_u_prev0 = z_u[0][0];
        fixed_point_t z_u_prev1 = z_u[0][1];

        /* --- Primal update: Riccati pass with augmented costs --- */
        riccati_pass_hls(
            step_data, A_dense, b50_hist, b21_hist, terminal_Q, terminal_q, x0, rho, rho_u,
            (const fixed_point_t (*)[MPC_NX_AUG])z_x,
            (const fixed_point_t (*)[MPC_NX_AUG])y_x,
            (const fixed_point_t (*)[MPC_NU])z_u,
            (const fixed_point_t (*)[MPC_NU])y_u,
            solution->x, solution->u);

        /* --- Fused z-update, y-update, and residual computation ---
         * Dual residual uses rho*(z_new - z_old), where z_old is read
         * before writing z_new in each component. */
        fixed_point_t state_primal = 0, state_dual = 0;
        fixed_point_t ctrl_primal = 0, ctrl_dual = 0;

        /* State z/y update with over-relaxation:
         * x_hat = alpha*x + (1-alpha)*z_old, alpha = 1.5. */
        for (k = 0; k <= N; k++) {
    #pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON_PLUS_ONE max=MPC_HORIZON_PLUS_ONE
    #pragma HLS PIPELINE II=MPC_HLS_STATE_UPDATE_II
            const StepData_t *sd = (k < N) ? &step_data[k] : &step_data[N - 1];
            uint8_t mask = x_con_mask[k];
            for (s = 0; s < nx; s++) {
#pragma HLS UNROLL factor=MPC_HLS_UNROLL_STATE_FACTOR
                if ((mask >> s) & 1U) {
                    fixed_point_t x_val = solution->x[k][s];
                    /* x_hat = x + (alpha-1)*(x - z_old), alpha=1.5 */
                    int64_t diff_x = (int64_t)x_val - (int64_t)z_x[k][s];
                    int64_t x_hat64 = (int64_t)x_val
                                    + (diff_x >> 1);
                    int64_t val = x_hat64 + (int64_t)y_x[k][s];
                    if (val < (int64_t)sd->x_lb[s]) val = (int64_t)sd->x_lb[s];
                    if (val > (int64_t)sd->x_ub[s]) val = (int64_t)sd->x_ub[s];
                    fixed_point_t z_new = (fixed_point_t)val;
                    fixed_point_t x_hat = (fixed_point_t)x_hat64;

                    /* Dual residual contribution: rho*(z_new - z_old) */
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
    #pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON max=MPC_HORIZON
#pragma HLS PIPELINE II=1
            const StepData_t *sd = &step_data[k];
            for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
                fixed_point_t u_val = solution->u[k][a];
                /* u_hat = u + (alpha-1)*(u - z_old), alpha=1.5 */
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

                /* y-update uses u_hat */
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

        /* Fast control-convergence gate: break when first control converges. */
        if (iter >= MPC_HLS_CTRL_EARLY_EXIT_MIN_ITER) {
            fixed_point_t dc0 = z_u[0][0] - z_u_prev0;
            fixed_point_t dc1 = z_u[0][1] - z_u_prev1;
            if (dc0 < 0) dc0 = -dc0;
            if (dc1 < 0) dc1 = -dc1;
            if ((dc0 + dc1) < MPC_HLS_CTRL_EARLY_EXIT_THRESH) {
                status = MPC_STATUS_OPTIMAL;
                break;
            }
        }

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
#pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON_PLUS_ONE max=MPC_HORIZON_PLUS_ONE
#pragma HLS PIPELINE II=1
                    for (s = 0; s < nx; s++) {
#pragma HLS UNROLL
                        y_x[k][s] >>= 1;
                    }
                }
                for (k = 0; k < N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON max=MPC_HORIZON
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
#pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON_PLUS_ONE max=MPC_HORIZON_PLUS_ONE
#pragma HLS PIPELINE II=1
                    for (s = 0; s < nx; s++) {
#pragma HLS UNROLL
                        y_x[k][s] <<= 1;
                    }
                }
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
