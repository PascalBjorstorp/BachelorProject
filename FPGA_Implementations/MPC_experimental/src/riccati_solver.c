/**
 * @file riccati_solver.c
 * @brief Riccati-ADMM Solver Implementation (Q16.16 Fixed-Point)
 *
 * Solves constrained LQR using ADMM with Riccati recursion for the
 * unconstrained sub-problem. Each ADMM iteration is O(N × nx³).
 *
 * Riccati backward pass (per step):
 *   M    = B^T P_{k+1}           (nu×nx)
 *   S    = R + M B               (nu×nu, invert via 2×2 formula)
 *   G    = M A + N^T             (nu×nx, includes cross-cost)
 *   K_k  = -S^{-1} G            (nu×nx, feedback gain)
 *   kk_k = -S^{-1} (r + B^T p)  (nu×1, feedforward)
 *   P_k  = Q + A^T P A + G^T K  (nx×nx)
 *   p_k  = q + A^T p + G^T kk   (nx×1)
 *
 * Riccati forward pass:
 *   x_0 = given
 *   u_k = K_k x_k + kk_k
 *   x_{k+1} = A x_k + B u_k
 *
 * ADMM loop:
 *   1. Riccati pass with augmented costs (Q+ρI, R+ρI)
 *   2. z = clip(x+y, lb, ub) and z_u = clip(u+y_u, u_lb, u_ub)
 *   3. y += x - z, y_u += u - z_u
 *   4. Check convergence
 *
 * All matrix ops use int64_t intermediates to avoid Q16.16 overflow.
 */

#include "../include/riccati_solver.h"
#include <string.h>
#include <stdio.h>

/* Debug flag: set to 1 from tests to print ADMM iteration details */
int riccati_admm_debug = 0;

/*===========================================================================
 * Configuration Defaults
 *===========================================================================*/

void riccati_admm_config_init(RiccatiAdmmConfig_t *config)
{
    config->rho            = FP_CONST(5.0);   /* State constraint penalty — moderate, balanced with costs */
    config->rho_u          = FP_CONST(5.0);   /* Control constraint penalty — equal to rho, adaptive handles rest */
    config->tolerance      = FP_CONST(0.1);   /* Relaxed for fixed-point Q16.16 precision */
    config->max_iterations = 200;
    config->adaptive_rho   = 1;               /* Adaptive rho by default */
    config->alpha          = FP_CONST(1.0);   /* No over-relaxation — prevents chattering at saturation */
}

void riccati_admm_state_init(RiccatiAdmmState_t *state)
{
    memset(state, 0, sizeof(*state));
    state->initialized = 0;
}

/*===========================================================================
 * 2×2 Matrix Inverse (for S = R + B^T P B)
 *===========================================================================
 * S = [[a, b], [c, d]]
 * det = ad - bc
 * S^{-1} = (1/det) * [[d, -b], [-c, a]]
 *===========================================================================*/

static int invert_2x2(
    const int64_t S[2][2],
    int64_t Si[2][2])
{
    int64_t det = ((S[0][0] * S[1][1]) >> FP_FRAC_BITS)
               - ((S[0][1] * S[1][0]) >> FP_FRAC_BITS);

    if (det == 0 || (det > -16 && det < 16)) {
        /* Singular or near-singular */
        return -1;
    }

    /* Si[i][j] = cofactor / det, all in Q16.16 */
    Si[0][0] = (S[1][1] << FP_FRAC_BITS) / det;
    Si[0][1] = -(S[0][1] << FP_FRAC_BITS) / det;
    Si[1][0] = -(S[1][0] << FP_FRAC_BITS) / det;
    Si[1][1] = (S[0][0] << FP_FRAC_BITS) / det;

    return 0;
}

/*===========================================================================
 * Riccati Backward + Forward Pass (iLQR variant)
 *===========================================================================
 * Computes the LQR solution with ADMM-augmented costs.
 *
 * **Selective ADMM augmentation**: rho is only added to STATE dimensions
 * that have finite constraints (|bound| < BOUND_THRESHOLD). Unconstrained
 * state dimensions skip augmentation entirely.
 *
 * **Control clipping in forward pass (iLQR)**: Controls are clipped to
 * their box bounds during the forward rollout. This ensures the trajectory
 * is always control-feasible, so ADMM only handles state constraints.
 * The Riccati backward pass uses rho_u to regularize controls toward
 * their clipped targets from the previous ADMM iteration.
 *
 * For constrained state dimensions:
 *   Q_tilde[s]  = Q[s] + rho
 *   q_tilde[s]  = q[s] - rho*(z_x[s] - y_x[s])
 * For unconstrained state dimensions:
 *   Q_tilde[s]  = Q[s]
 *   q_tilde[s]  = q[s]
 *
 * For controls (always constrained):
 *   R_tilde[a]  = R[a] + rho_u
 *   r_tilde[a]  = r[a] - rho_u*(z_u[a] - y_u[a])
 * where z_u is the clipped control from the previous iteration.
 *
 * The cross-cost N is NOT augmented by ADMM.
 *===========================================================================*/

/** Threshold to detect "effectively unconstrained" bounds.
 *  If |bound| >= this value, the state is treated as unconstrained. */
#define BOUND_THRESHOLD  FP_CONST(100.0)

static void riccati_pass(
    const RiccatiStepData_t *step_data,
    const fixed_point_t *terminal_Q,
    const fixed_point_t *terminal_q,
    const fixed_point_t *x0,
    int nx, int nu, int N,
    fixed_point_t rho,       /* state constraint penalty */
    fixed_point_t rho_u,     /* control constraint penalty */
    /* ADMM offsets (z - y) for augmented linear cost */
    const fixed_point_t z_x[][RICCATI_MAX_NX],
    const fixed_point_t y_x[][RICCATI_MAX_NX],
    const fixed_point_t z_u[][RICCATI_MAX_NU],
    const fixed_point_t y_u[][RICCATI_MAX_NU],
    /* Output trajectories */
    fixed_point_t x_out[][RICCATI_MAX_NX],
    fixed_point_t u_out[][RICCATI_MAX_NU])
{
    /*-------------------------------------------------------------------
     * Backward pass: compute P_k, p_k, K_k, kk_k for k = N-1 .. 0
     *
     * P and p are stored for all steps (needed by forward pass only for
     * the gains K_k and kk_k, but we store them to allow reuse).
     * Actually, we only need K_k and kk_k for the forward pass.
     *-------------------------------------------------------------------*/

    /* Gains stored for forward pass */
    fixed_point_t K[RICCATI_MAX_HORIZON][RICCATI_MAX_NU][RICCATI_MAX_NX];
    fixed_point_t kk[RICCATI_MAX_HORIZON][RICCATI_MAX_NU];

    /* Value function: P (nx×nx symmetric), p (nx×1) */
    /* We only need P_{k+1} at each step, so use two buffers */
    int64_t P[RICCATI_MAX_NX][RICCATI_MAX_NX];  /* Current P_{k+1} in int64 */
    int64_t p[RICCATI_MAX_NX];                    /* Current p_{k+1} in int64 */

    /* Initialize terminal cost: P_N = Q_N [+ rho*I if constrained], p_N similar */
    memset(P, 0, sizeof(P));
    memset(p, 0, sizeof(p));
    {
        /* Use last step's bounds for terminal cost constraint detection */
        const RiccatiStepData_t *last_sd = &step_data[N - 1];
        for (int s = 0; s < nx; s++) {
            int is_constrained = (last_sd->x_ub[s] < BOUND_THRESHOLD ||
                                  last_sd->x_lb[s] > -BOUND_THRESHOLD);
            if (is_constrained) {
                P[s][s] = (int64_t)terminal_Q[s] + (int64_t)rho;
                p[s] = (int64_t)terminal_q[s]
                     - (((int64_t)rho * ((int64_t)z_x[N][s] - (int64_t)y_x[N][s])) >> FP_FRAC_BITS);
            } else {
                P[s][s] = (int64_t)terminal_Q[s];
                p[s] = (int64_t)terminal_q[s];
            }
        }
    }

    /* Backward pass: k = N-1 down to 0 */
    for (int k = N - 1; k >= 0; k--) {
        const RiccatiStepData_t *sd = &step_data[k];

        /* Augmented costs: only add rho to constrained dimensions */
        int64_t Q_aug[RICCATI_MAX_NX];
        int64_t q_aug[RICCATI_MAX_NX];
        int64_t R_aug[RICCATI_MAX_NU];
        int64_t r_aug[RICCATI_MAX_NU];

        for (int s = 0; s < nx; s++) {
            int is_constrained = (sd->x_ub[s] < BOUND_THRESHOLD ||
                                  sd->x_lb[s] > -BOUND_THRESHOLD);
            if (is_constrained) {
                Q_aug[s] = (int64_t)sd->Q_diag[s] + (int64_t)rho;
                q_aug[s] = (int64_t)sd->q[s]
                         - (((int64_t)rho * ((int64_t)z_x[k][s] - (int64_t)y_x[k][s])) >> FP_FRAC_BITS);
            } else {
                Q_aug[s] = (int64_t)sd->Q_diag[s];
                q_aug[s] = (int64_t)sd->q[s];
            }
        }
        /* All controls are constrained (always have box bounds) */
        for (int a = 0; a < nu; a++) {
            R_aug[a] = (int64_t)sd->R_diag[a] + (int64_t)rho_u;
            r_aug[a] = (int64_t)sd->r[a]
                     - (((int64_t)rho_u * ((int64_t)z_u[k][a] - (int64_t)y_u[k][a])) >> FP_FRAC_BITS);
        }

        /* Step 1: M = B^T P (nu×nx) in int64 */
        int64_t M[RICCATI_MAX_NU][RICCATI_MAX_NX];
        for (int a = 0; a < nu; a++) {
            for (int j = 0; j < nx; j++) {
                int64_t sum = 0;
                for (int s = 0; s < nx; s++) {
                    sum += ((int64_t)sd->B[s][a] * P[s][j]) >> FP_FRAC_BITS;
                }
                M[a][j] = sum;
            }
        }

        /* Step 2: S = R_aug + M*B (nu×nu) in int64 */
        int64_t S[2][2] = {{0}};
        for (int a = 0; a < nu; a++) {
            for (int b = 0; b < nu; b++) {
                int64_t sum = (a == b) ? R_aug[a] : 0;
                for (int s = 0; s < nx; s++) {
                    sum += (M[a][s] * (int64_t)sd->B[s][b]) >> FP_FRAC_BITS;
                }
                S[a][b] = sum;
            }
        }

        /* Step 3: Invert S (2×2) */
        int64_t Si[2][2];
        if (invert_2x2(S, Si) < 0) {
            /* Fallback: use diagonal approximation */
            Si[0][0] = S[0][0] != 0 ? ((int64_t)FP_ONE << FP_FRAC_BITS) / S[0][0] : 0;
            Si[0][1] = 0;
            Si[1][0] = 0;
            Si[1][1] = S[1][1] != 0 ? ((int64_t)FP_ONE << FP_FRAC_BITS) / S[1][1] : 0;
        }

        /* Step 4: G = M*A + N^T (nu×nx) in int64 */
        int64_t G[RICCATI_MAX_NU][RICCATI_MAX_NX];
        for (int a = 0; a < nu; a++) {
            for (int j = 0; j < nx; j++) {
                int64_t sum = (int64_t)sd->N[j][a];  /* N^T[a][j] = N[j][a] */
                for (int s = 0; s < nx; s++) {
                    sum += (M[a][s] * (int64_t)sd->A[s][j]) >> FP_FRAC_BITS;
                }
                G[a][j] = sum;
            }
        }

        /* Step 5: K = -S^{-1} G (nu×nx) — store as Q16.16 for forward pass */
        for (int a = 0; a < nu; a++) {
            for (int j = 0; j < nx; j++) {
                int64_t val = 0;
                for (int b = 0; b < nu; b++) {
                    val += (Si[a][b] * G[b][j]) >> FP_FRAC_BITS;
                }
                /* K = -S^{-1} G */
                val = -val;
                /* Clamp to int32 */
                if (val > INT32_MAX) val = INT32_MAX;
                else if (val < INT32_MIN) val = INT32_MIN;
                K[k][a][j] = (fixed_point_t)val;
            }
        }

        /* Step 6: kk = -S^{-1} (r_aug + B^T p) (nu×1) */
        int64_t Bp[RICCATI_MAX_NU];
        for (int a = 0; a < nu; a++) {
            int64_t sum = 0;
            for (int s = 0; s < nx; s++) {
                sum += ((int64_t)sd->B[s][a] * p[s]) >> FP_FRAC_BITS;
            }
            Bp[a] = sum;
        }

        for (int a = 0; a < nu; a++) {
            int64_t rhs_a = r_aug[a] + Bp[a];
            int64_t val = 0;
            for (int b = 0; b < nu; b++) {
                val += (Si[a][b] * (r_aug[b] + Bp[b])) >> FP_FRAC_BITS;
            }
            val = -val;
            if (val > INT32_MAX) val = INT32_MAX;
            else if (val < INT32_MIN) val = INT32_MIN;
            kk[k][a] = (fixed_point_t)val;
            (void)rhs_a;  /* suppress unused warning */
        }

        /* Step 7: P_k = Q_aug_diag + A^T P A + G^T K (nx×nx)
         *
         * G^T K comes from the substitution: P_k = Q + A^T P A - G^T S^{-1} G.
         * Since K = -S^{-1} G, we have G^T K = -G^T S^{-1} G,
         * so P_k = Q + A^T P A + G^T K = Q + A^T P A - G^T S^{-1} G.
         */
        /* Compute PA = P * A (nx×nx) in int64 */
        int64_t PA[RICCATI_MAX_NX][RICCATI_MAX_NX];
        for (int i = 0; i < nx; i++) {
            for (int j = 0; j < nx; j++) {
                int64_t sum = 0;
                for (int s = 0; s < nx; s++) {
                    sum += (P[i][s] * (int64_t)sd->A[s][j]) >> FP_FRAC_BITS;
                }
                PA[i][j] = sum;
            }
        }

        /* Compute AtPA = A^T * PA (nx×nx) */
        int64_t AtPA[RICCATI_MAX_NX][RICCATI_MAX_NX];
        for (int i = 0; i < nx; i++) {
            for (int j = 0; j < nx; j++) {
                int64_t sum = 0;
                for (int s = 0; s < nx; s++) {
                    sum += ((int64_t)sd->A[s][i] * PA[s][j]) >> FP_FRAC_BITS;
                }
                AtPA[i][j] = sum;
            }
        }

        /* Compute GtK = G^T * K (nx×nx) using int64 G and int32 K */
        int64_t GtK[RICCATI_MAX_NX][RICCATI_MAX_NX];
        for (int i = 0; i < nx; i++) {
            for (int j = 0; j < nx; j++) {
                int64_t sum = 0;
                for (int a = 0; a < nu; a++) {
                    sum += (G[a][i] * (int64_t)K[k][a][j]) >> FP_FRAC_BITS;
                }
                GtK[i][j] = sum;
            }
        }

        /* P_k = Q_aug_diag + AtPA + GtK */
        int64_t P_new[RICCATI_MAX_NX][RICCATI_MAX_NX];
        for (int i = 0; i < nx; i++) {
            for (int j = 0; j < nx; j++) {
                P_new[i][j] = AtPA[i][j] + GtK[i][j];
                if (i == j) P_new[i][j] += Q_aug[i];
            }
        }
        memcpy(P, P_new, sizeof(P));

        /* Step 8: p_k = q_aug + A^T p_{k+1} + G^T kk (nx×1) */
        int64_t p_new[RICCATI_MAX_NX];
        for (int i = 0; i < nx; i++) {
            /* A^T p */
            int64_t Atp = 0;
            for (int s = 0; s < nx; s++) {
                Atp += ((int64_t)sd->A[s][i] * p[s]) >> FP_FRAC_BITS;
            }
            /* G^T kk */
            int64_t Gtk = 0;
            for (int a = 0; a < nu; a++) {
                Gtk += (G[a][i] * (int64_t)kk[k][a]) >> FP_FRAC_BITS;
            }
            p_new[i] = q_aug[i] + Atp + Gtk;
        }
        memcpy(p, p_new, sizeof(p));
    }

    /*-------------------------------------------------------------------
     * Forward pass: roll out x, u from x0 using gains K, kk
     *
     * ADMM: controls are NOT clipped here — the z-update projects
     * controls onto the feasible set. The Riccati backward pass
     * augments R with rho_u*(u - z_u + y_u), pulling u toward the
     * projected controls from the previous iteration.
     *-------------------------------------------------------------------*/
    for (int s = 0; s < nx; s++) {
        x_out[0][s] = x0[s];
    }

    for (int k = 0; k < N; k++) {
        const RiccatiStepData_t *sd = &step_data[k];

        /* u_k = K_k x_k + kk_k (unconstrained — ADMM handles bounds) */
        for (int a = 0; a < nu; a++) {
            int64_t sum = (int64_t)kk[k][a];
            for (int s = 0; s < nx; s++) {
                sum += ((int64_t)K[k][a][s] * (int64_t)x_out[k][s]) >> FP_FRAC_BITS;
            }
            if (sum > INT32_MAX) sum = INT32_MAX;
            else if (sum < INT32_MIN) sum = INT32_MIN;
            u_out[k][a] = (fixed_point_t)sum;
        }

        /* x_{k+1} = A_k x_k + B_k u_k */
        for (int i = 0; i < nx; i++) {
            int64_t sum = 0;
            for (int s = 0; s < nx; s++) {
                sum += ((int64_t)sd->A[i][s] * (int64_t)x_out[k][s]) >> FP_FRAC_BITS;
            }
            for (int a = 0; a < nu; a++) {
                sum += ((int64_t)sd->B[i][a] * (int64_t)u_out[k][a]) >> FP_FRAC_BITS;
            }
            if (sum > INT32_MAX) sum = INT32_MAX;
            else if (sum < INT32_MIN) sum = INT32_MIN;
            x_out[k + 1][i] = (fixed_point_t)sum;
        }
    }
}

/*===========================================================================
 * Main Solver: Riccati-ADMM
 *===========================================================================*/

RiccatiStatus_t riccati_admm_solve(
    const RiccatiStepData_t *step_data,
    const fixed_point_t *terminal_Q,
    const fixed_point_t *terminal_q,
    const fixed_point_t *x0,
    int nx, int nu, int N,
    const RiccatiAdmmConfig_t *config,
    RiccatiAdmmState_t *admm_state,
    RiccatiSolution_t *solution)
{
    if (nx <= 0 || nx > RICCATI_MAX_NX || nu <= 0 || nu > RICCATI_MAX_NU ||
        N <= 0 || N > RICCATI_MAX_HORIZON) {
        solution->status = RICCATI_STATUS_ERROR;
        return RICCATI_STATUS_ERROR;
    }

    fixed_point_t rho = config->rho;
    fixed_point_t rho_u = config->rho_u > 0 ? config->rho_u : rho;
    int max_iter = config->max_iterations;

    /* Initialize ADMM variables (or use warm-start) */
    /* z = slack (projected), y = dual (scaled) */
    fixed_point_t z_x[RICCATI_MAX_HORIZON + 1][RICCATI_MAX_NX];
    fixed_point_t z_u[RICCATI_MAX_HORIZON][RICCATI_MAX_NU];
    fixed_point_t y_x[RICCATI_MAX_HORIZON + 1][RICCATI_MAX_NX];
    fixed_point_t y_u[RICCATI_MAX_HORIZON][RICCATI_MAX_NU];

    if (admm_state->initialized) {
        memcpy(z_x, admm_state->z_x, sizeof(z_x));
        memcpy(z_u, admm_state->z_u, sizeof(z_u));
        memcpy(y_x, admm_state->y_x, sizeof(y_x));
        memcpy(y_u, admm_state->y_u, sizeof(y_u));
    } else {
        /* Cold start: run one unconstrained Riccati pass to initialize
         * z and y from the constraint violations. This dramatically
         * accelerates convergence when constraints are heavily active. */
        memset(z_x, 0, sizeof(z_x));
        memset(z_u, 0, sizeof(z_u));
        memset(y_x, 0, sizeof(y_x));
        memset(y_u, 0, sizeof(y_u));

        /* Solve unconstrained (rho=0 effectively, or use base costs) */
        riccati_pass(
            step_data, terminal_Q, terminal_q, x0,
            nx, nu, N, 0, 0,  /* rho=0, rho_u=0: no ADMM augmentation */
            (const fixed_point_t (*)[RICCATI_MAX_NX])z_x,
            (const fixed_point_t (*)[RICCATI_MAX_NX])y_x,
            (const fixed_point_t (*)[RICCATI_MAX_NU])z_u,
            (const fixed_point_t (*)[RICCATI_MAX_NU])y_u,
            solution->x, solution->u);

        /* Initialize z from projection of unconstrained solution */
        for (int k = 0; k <= N; k++) {
            const RiccatiStepData_t *sd = (k < N) ? &step_data[k] : &step_data[N - 1];
            for (int s = 0; s < nx; s++) {
                fixed_point_t val = solution->x[k][s];
                if (val < sd->x_lb[s]) val = sd->x_lb[s];
                if (val > sd->x_ub[s]) val = sd->x_ub[s];
                z_x[k][s] = val;
            }
        }
        for (int k = 0; k < N; k++) {
            for (int a = 0; a < nu; a++) {
                fixed_point_t val = solution->u[k][a];
                if (val < step_data[k].u_lb[a]) val = step_data[k].u_lb[a];
                if (val > step_data[k].u_ub[a]) val = step_data[k].u_ub[a];
                z_u[k][a] = val;
            }
        }

        /* Initialize y (dual) from constraint violation.
         * y = x_unconstrained - z = violation. This primes the dual
         * variables instead of slowly accumulating from zero. */
        for (int k = 0; k <= N; k++) {
            const RiccatiStepData_t *sd = (k < N) ? &step_data[k] : &step_data[N - 1];
            for (int s = 0; s < nx; s++) {
                int is_constrained = (sd->x_ub[s] < BOUND_THRESHOLD ||
                                      sd->x_lb[s] > -BOUND_THRESHOLD);
                if (is_constrained) {
                    y_x[k][s] = solution->x[k][s] - z_x[k][s];
                }
            }
        }
        for (int k = 0; k < N; k++) {
            for (int a = 0; a < nu; a++) {
                y_u[k][a] = solution->u[k][a] - z_u[k][a];
            }
        }
    }

    /* Previous z for dual residual computation */
    fixed_point_t z_x_old[RICCATI_MAX_HORIZON + 1][RICCATI_MAX_NX];
    fixed_point_t z_u_old[RICCATI_MAX_HORIZON][RICCATI_MAX_NU];

    RiccatiStatus_t status = RICCATI_STATUS_MAX_ITERATIONS;

    for (int iter = 0; iter < max_iter; iter++) {

        /* Save previous z for dual residual */
        memcpy(z_x_old, z_x, sizeof(z_x));
        memcpy(z_u_old, z_u, sizeof(z_u));

        /*--- Primal update: Riccati pass with augmented costs ---*/
        riccati_pass(
            step_data, terminal_Q, terminal_q, x0,
            nx, nu, N, rho, rho_u,
            (const fixed_point_t (*)[RICCATI_MAX_NX])z_x,
            (const fixed_point_t (*)[RICCATI_MAX_NX])y_x,
            (const fixed_point_t (*)[RICCATI_MAX_NU])z_u,
            (const fixed_point_t (*)[RICCATI_MAX_NU])y_u,
            solution->x, solution->u);

        /*--- z-update with over-relaxation ---*/
        /* x̂ = α*x + (1-α)*z_old, then z_new = clip(x̂ + y, lb, ub)
         * Standard ADMM uses x̂ = x (α=1). Over-relaxation (α>1)
         * extrapolates beyond the primal update for faster convergence. */
        fixed_point_t alpha = config->alpha;
        fixed_point_t one_minus_alpha = FP_ONE - alpha;  /* negative when α>1 */

        /* State constraints: z_x = clip(x̂ + y_x, x_lb, x_ub)
         * Only process constrained dimensions; unconstrained ones
         * keep z_x = x (ADMM augmentation was zero for them) */
        for (int k = 0; k <= N; k++) {
            const RiccatiStepData_t *sd = (k < N) ? &step_data[k] : &step_data[N - 1];
            for (int s = 0; s < nx; s++) {
                int is_constrained = (sd->x_ub[s] < BOUND_THRESHOLD ||
                                      sd->x_lb[s] > -BOUND_THRESHOLD);
                if (is_constrained) {
                    int64_t x_hat = (((int64_t)alpha * (int64_t)solution->x[k][s]) >> FP_FRAC_BITS)
                                  + (((int64_t)one_minus_alpha * (int64_t)z_x_old[k][s]) >> FP_FRAC_BITS);
                    int64_t val = x_hat + (int64_t)y_x[k][s];
                    if (val < (int64_t)sd->x_lb[s]) val = (int64_t)sd->x_lb[s];
                    if (val > (int64_t)sd->x_ub[s]) val = (int64_t)sd->x_ub[s];
                    z_x[k][s] = (fixed_point_t)val;
                } else {
                    /* Unconstrained: z tracks x exactly (no ADMM projection) */
                    z_x[k][s] = solution->x[k][s];
                }
            }
        }

        /* Control constraints: z_u = clip(û + y_u, u_lb, u_ub)
         * Full ADMM consensus for controls — the z-update projects
         * the over-relaxed primal + dual onto the control bounds. */
        for (int k = 0; k < N; k++) {
            const RiccatiStepData_t *sd = &step_data[k];
            for (int a = 0; a < nu; a++) {
                int64_t u_hat = (((int64_t)alpha * (int64_t)solution->u[k][a]) >> FP_FRAC_BITS)
                              + (((int64_t)one_minus_alpha * (int64_t)z_u_old[k][a]) >> FP_FRAC_BITS);
                int64_t val = u_hat + (int64_t)y_u[k][a];
                if (val < (int64_t)sd->u_lb[a]) val = (int64_t)sd->u_lb[a];
                if (val > (int64_t)sd->u_ub[a]) val = (int64_t)sd->u_ub[a];
                z_u[k][a] = (fixed_point_t)val;
            }
        }

        /*--- y-update: only for constrained STATE dimensions ---*/
        for (int k = 0; k <= N; k++) {
            const RiccatiStepData_t *sd = (k < N) ? &step_data[k] : &step_data[N - 1];
            for (int s = 0; s < nx; s++) {
                int is_constrained = (sd->x_ub[s] < BOUND_THRESHOLD ||
                                      sd->x_lb[s] > -BOUND_THRESHOLD);
                if (is_constrained) {
                    int64_t x_hat = (((int64_t)alpha * (int64_t)solution->x[k][s]) >> FP_FRAC_BITS)
                                  + (((int64_t)one_minus_alpha * (int64_t)z_x_old[k][s]) >> FP_FRAC_BITS);
                    y_x[k][s] = (fixed_point_t)(x_hat - (int64_t)z_x[k][s]
                        + (int64_t)y_x[k][s]);
                }
                /* Unconstrained: y stays at 0 (no dual penalty needed) */
            }
        }
        /* Control y-update: full ADMM dual accumulation */
        for (int k = 0; k < N; k++) {
            for (int a = 0; a < nu; a++) {
                int64_t u_hat = (((int64_t)alpha * (int64_t)solution->u[k][a]) >> FP_FRAC_BITS)
                              + (((int64_t)one_minus_alpha * (int64_t)z_u_old[k][a]) >> FP_FRAC_BITS);
                y_u[k][a] = (fixed_point_t)(u_hat - (int64_t)z_u[k][a]
                    + (int64_t)y_u[k][a]);
            }
        }

        /*--- Convergence check: state constraints determine convergence ---*/
        /* Controls are always feasible via z_u projection. State constraint
         * satisfaction is what matters for safety (wall avoidance).
         * We track both but converge only on state residuals. */

        /* State primal residual: ||x - z_x||_inf for constrained states */
        fixed_point_t state_primal = 0;
        for (int k = 0; k <= N; k++) {
            const RiccatiStepData_t *sd2 = (k < N) ? &step_data[k] : &step_data[N - 1];
            for (int s = 0; s < nx; s++) {
                int is_constrained = (sd2->x_ub[s] < BOUND_THRESHOLD ||
                                      sd2->x_lb[s] > -BOUND_THRESHOLD);
                if (is_constrained) {
                    fixed_point_t d = solution->x[k][s] - z_x[k][s];
                    if (d < 0) d = -d;
                    if (d > state_primal) state_primal = d;
                }
            }
        }

        /* Control primal residual: ||u - z_u||_inf (tracked but not blocking) */
        fixed_point_t ctrl_primal = 0;
        for (int k = 0; k < N; k++) {
            for (int a = 0; a < nu; a++) {
                fixed_point_t d = solution->u[k][a] - z_u[k][a];
                if (d < 0) d = -d;
                if (d > ctrl_primal) ctrl_primal = d;
            }
        }

        /* Combined primal for reporting (max of state + control) */
        fixed_point_t primal_res = state_primal > ctrl_primal ? state_primal : ctrl_primal;

        /* Dual residual: state constraints only for convergence */
        fixed_point_t state_dual = 0;
        for (int k = 0; k <= N; k++) {
            const RiccatiStepData_t *sd2 = (k < N) ? &step_data[k] : &step_data[N - 1];
            for (int s = 0; s < nx; s++) {
                int is_constrained = (sd2->x_ub[s] < BOUND_THRESHOLD ||
                                      sd2->x_lb[s] > -BOUND_THRESHOLD);
                if (is_constrained) {
                    int64_t d64 = ((int64_t)rho * ((int64_t)z_x[k][s] - (int64_t)z_x_old[k][s])) >> FP_FRAC_BITS;
                    fixed_point_t d = (fixed_point_t)(d64 < 0 ? -d64 : d64);
                    if (d > state_dual) state_dual = d;
                }
            }
        }
        /* Control dual tracked separately for reporting */
        fixed_point_t ctrl_dual = 0;
        for (int k = 0; k < N; k++) {
            for (int a = 0; a < nu; a++) {
                int64_t d64 = ((int64_t)rho_u * ((int64_t)z_u[k][a] - (int64_t)z_u_old[k][a])) >> FP_FRAC_BITS;
                fixed_point_t d = (fixed_point_t)(d64 < 0 ? -d64 : d64);
                if (d > ctrl_dual) ctrl_dual = d;
            }
        }
        fixed_point_t dual_res = state_dual > ctrl_dual ? state_dual : ctrl_dual;

        solution->iterations = iter + 1;
        solution->primal_residual = primal_res;
        solution->dual_residual = dual_res;

        /* Debug: print iteration details for first few iterations and periodically */
        if (riccati_admm_debug && (iter < 5 || iter % 50 == 0 || iter == max_iter - 1)) {
            printf("    ADMM[%3d] p=%.4f(s=%.4f,c=%.4f) d=%.4f(s=%.4f,c=%.4f) rho=%.2f rho_u=%.2f u0=[%.4f,%.3f] z0=[%.4f,%.3f] y0=[%.4f,%.3f]\n",
                   iter,
                   FP_TO_DOUBLE(primal_res), FP_TO_DOUBLE(state_primal), FP_TO_DOUBLE(ctrl_primal),
                   FP_TO_DOUBLE(dual_res), FP_TO_DOUBLE(state_dual), FP_TO_DOUBLE(ctrl_dual),
                   FP_TO_DOUBLE(rho), FP_TO_DOUBLE(rho_u),
                   FP_TO_DOUBLE(solution->u[0][0]), FP_TO_DOUBLE(solution->u[0][1]),
                   FP_TO_DOUBLE(z_u[0][0]), FP_TO_DOUBLE(z_u[0][1]),
                   FP_TO_DOUBLE(y_u[0][0]), FP_TO_DOUBLE(y_u[0][1]));
        }

        /* Converge on BOTH state and control residuals.
         * Previously only state residuals were checked, causing false
         * convergence in 1-2 iterations when controls were heavily
         * saturated but no wall constraints were active. */
        if (primal_res <= config->tolerance && dual_res <= config->tolerance) {
            status = RICCATI_STATUS_OPTIMAL;
            break;
        }

        /*--- Adaptive rho: balance primal/dual convergence rates ---*/
        if (config->adaptive_rho && iter > 0 && (iter & 3) == 0) {
            /* Check every 4 iterations to avoid oscillation */
            if (primal_res > 10 * dual_res && rho < FP_CONST(50.0)) {
                /* Primal lagging: increase rho to penalize constraint violation more */
                rho = fp_mul(rho, FP_CONST(2.0));
                if (rho_u < FP_CONST(50.0))
                    rho_u = fp_mul(rho_u, FP_CONST(2.0));
                /* Scale dual variables: y = y / 2 (compensates for rho doubling) */
                for (int k = 0; k <= N; k++)
                    for (int s = 0; s < nx; s++)
                        y_x[k][s] >>= 1;
                for (int k = 0; k < N; k++)
                    for (int a = 0; a < nu; a++)
                        y_u[k][a] >>= 1;
            } else if (dual_res > 10 * primal_res && rho > FP_CONST(0.5)) {
                /* Dual lagging: decrease rho to let the cost dominate */
                rho = fp_mul(rho, FP_CONST(0.5));
                if (rho_u > FP_CONST(0.5))
                    rho_u = fp_mul(rho_u, FP_CONST(0.5));
                /* Scale dual variables: y = y * 2 */
                for (int k = 0; k <= N; k++)
                    for (int s = 0; s < nx; s++)
                        y_x[k][s] <<= 1;
                for (int k = 0; k < N; k++)
                    for (int a = 0; a < nu; a++)
                        y_u[k][a] <<= 1;
            }
        }
    }

    /* Save ADMM state for warm-starting */
    memcpy(admm_state->z_x, z_x, sizeof(z_x));
    memcpy(admm_state->z_u, z_u, sizeof(z_u));
    memcpy(admm_state->y_x, y_x, sizeof(y_x));
    /* Preserve y_u across calls for warm-starting.
     * Previously y_u was zeroed because state-only convergence allowed
     * control duals to accumulate indefinitely. With the fixed combined
     * convergence check, y_u converges properly and warm-starting is safe. */
    memcpy(admm_state->y_u, y_u, sizeof(y_u));
    admm_state->initialized = 1;

    /* Output feasible controls: z_u is the ADMM projection, always
     * within bounds. The raw solution->u from the forward pass is
     * unconstrained (Riccati gains may exceed limits). */
    memcpy(solution->u, z_u, sizeof(z_u));

    solution->status = status;
    return status;
}
