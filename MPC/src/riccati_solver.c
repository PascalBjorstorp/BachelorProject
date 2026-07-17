/**
 * @file riccati_solver.c
 * @brief Riccati-ADMM Solver Implementation
 * @details Solves constrained LQR using ADMM with Riccati recursion for the
 *          unconstrained sub-problem. Each ADMM iteration is O(N × nx^3).
 *
 * Riccati backward pass (per step):
 *   M    = B^T P_{k+1}           (nu×nx)
 *   S    = R + M B               (nu×nu, invert via 2×2 formula)
 *   G    = M A + N^T             (nu×nx, includes cross-cost)
 *   K_k  = -S^{-1} G             (nu×nx, feedback gain)
 *   kk_k = -S^{-1} (r + B^T p)   (nu×1, feedforward)
 *   P_k  = Q + A^T P A + G^T K   (nx×nx)
 *   p_k  = q + A^T p + G^T kk    (nx×1)
 *
 * Riccati forward pass:
 *   x_0 = given
 *   u_k = K_k x_k + kk_k
 *   x_{k+1} = A x_k + B u_k
 *
 * ADMM loop:
 *   1. Riccati pass with augmented costs (Q+rhoI, R+rhoI)
 *   2. z = clip(x+y, lb, ub) and z_u = clip(u+y_u, u_lb, u_ub)
 *   3. y += x - z, y_u += u - z_u
 *   4. Check convergence
 *
 * All operations use native float32 arithmetic.
 * @dependencies riccati_solver.h, <string.h>, <stdio.h>, <math.h>
 */

#include "riccati_solver.h"

/* Debug flag: set to 1 from tests to print ADMM iteration details */
int riccati_admm_debug = 0;
static RiccatiDebugInfo_t g_riccati_debug_last;
static RiccatiDebugIterSample_t g_riccati_debug_trace[RICCATI_DEBUG_TRACE_MAX];
static int g_riccati_debug_trace_count = 0;
static int g_riccati_debug_trace_enabled = 0;

void riccati_debug_get_last(RiccatiDebugInfo_t *out)
{
    if (!out) return;
    *out = g_riccati_debug_last;
}

int riccati_debug_get_trace_count(void)
{
    return g_riccati_debug_trace_count;
}

int riccati_debug_get_trace_sample(int index, RiccatiDebugIterSample_t *out)
{
    if (!out || index < 0 || index >= g_riccati_debug_trace_count) return -1;
    *out = g_riccati_debug_trace[index];
    return 0;
}

void riccati_debug_set_trace_enabled(int enabled)
{
    g_riccati_debug_trace_enabled = enabled != 0;
    if (!g_riccati_debug_trace_enabled) {
        g_riccati_debug_trace_count = 0;
    }
}

/*===========================================================================
 * ADMM State Initialization
 *===========================================================================*/

void riccati_admm_state_init(RiccatiAdmmState_t *state)
{
    // Guard against NULL pointer input
    if (!state) {
        return;
    }

    // Clear all buffers and reset initialized flag
    memset(state, 0, sizeof(*state));
    state->initialized = 0;
}

/*===========================================================================
 * 2x2 Matrix Inverse (for S = R + B^T P B)
 *===========================================================================*/

int riccati_invert_2x2(
    float S[2][2],
    float Si[2][2])
{
    float det = S[0][0] * S[1][1] - S[0][1] * S[1][0];

    if (fabsf(det) < 1e-10f) {
        /* Singular or near-singular */
        g_riccati_debug_last.invert_fallback_count++;
        g_riccati_debug_last.last_invert_det = det;
        return -1;
    }

    float inv_det = 1.0f / det;
    Si[0][0] =  S[1][1] * inv_det;
    Si[0][1] = -S[0][1] * inv_det;
    Si[1][0] = -S[1][0] * inv_det;
    Si[1][1] =  S[0][0] * inv_det;

    return 0;
}

/*===========================================================================
 * Riccati Backward + Forward Pass
 *===========================================================================*/

void riccati_solver_pass(
    const RiccatiStepData_t * restrict step_data,
    const float * restrict terminal_Q,
    const float * restrict terminal_q,
    const float * restrict terminal_x_lb,
    const float * restrict terminal_x_ub,
    const float * restrict x0,
    int nx, int nu, int N,
    float rho,
    float rho_u,
    const float z_x[][RICCATI_MAX_NX],
    const float y_x[][RICCATI_MAX_NX],
    const float z_u[][RICCATI_MAX_NU],
    const float y_u[][RICCATI_MAX_NU],
    float x_out[][RICCATI_MAX_NX],
    float u_out[][RICCATI_MAX_NU])
{
    /* Gains stored for forward pass */
    float K[PREDICTION_HORIZON][RICCATI_MAX_NU][RICCATI_MAX_NX];
    float kk[PREDICTION_HORIZON][RICCATI_MAX_NU];

    /* Value function: P (nx x nx symmetric), p (nx x 1) */
    float P[RICCATI_MAX_NX][RICCATI_MAX_NX];
    float p[RICCATI_MAX_NX];

    /* Initialize terminal cost */
    memset(P, 0, sizeof(P));
    memset(p, 0, sizeof(p));
    {
        // Add ADMM penalty to terminal cost for constrained state channels
        for (int s = 0; s < nx; s++) {
            int is_constrained = (terminal_x_ub[s] < BIG_BOUND ||
                                  terminal_x_lb[s] > -BIG_BOUND);
            if (is_constrained) {
                P[s][s] = terminal_Q[s] + rho;
                p[s] = terminal_q[s] - rho * (z_x[N][s] - y_x[N][s]);
            } else {
                P[s][s] = terminal_Q[s];
                p[s] = terminal_q[s];
            }
        }
    }

    /* Backward pass: k = N-1 down to 0 */
    for (int k = N - 1; k >= 0; k--) {
        const RiccatiStepData_t *sd = &step_data[k];

        /* Augmented costs */
        float q_aug_diag[RICCATI_MAX_NX];
        float q_aug_linear[RICCATI_MAX_NX];
        float r_aug_diag[RICCATI_MAX_NU];
        float r_aug_linear[RICCATI_MAX_NU];

        // Add ADMM penalty to stage cost for constrained state channels
        for (int s = 0; s < nx; s++) {
            /* x_0 is fixed by the caller, not an optimization variable.  Its
             * box violation must therefore not enter the ADMM residuals (or
             * the augmented objective).  Bounds become actionable at x_1. */
            int is_constrained = k > 0 &&
                                 (sd->x_ub[s] < BIG_BOUND ||
                                  sd->x_lb[s] > -BIG_BOUND);
            if (is_constrained) {
                q_aug_diag[s] = sd->Q_diag[s] + rho;
                q_aug_linear[s] = sd->q[s] - rho * (z_x[k][s] - y_x[k][s]);
            } else {
                q_aug_diag[s] = sd->Q_diag[s];
                q_aug_linear[s] = sd->q[s];
            }
        }
        // Add ADMM penalty to control cost for all control channels
        for (int a = 0; a < nu; a++) {
            r_aug_diag[a] = sd->R_diag[a] + rho_u;
            r_aug_linear[a] = sd->r[a] - rho_u * (z_u[k][a] - y_u[k][a]);
        }

        /* Step 1: M = B^T P (nu x nx) */
        float M[RICCATI_MAX_NU][RICCATI_MAX_NX];
        for (int j = 0; j < nx; j++) {
            float s0 = 0.0f, s1 = 0.0f;
            /* Iterate the dense prefix [IDX_SPARSE_B_FIRST_ROW, IDX_DRATE_PREV);
             * rows 0 and 1 are structural zeros and are skipped.
             * The two previous-control tail rows are handled explicitly below
             * via +P[IDX_DRATE_PREV][j] and +P[IDX_ACCEL_PREV][j]. */
            for (int s = IDX_SPARSE_B_FIRST_ROW; s < IDX_DRATE_PREV; s++) {
                s0 += sd->B[s][0] * P[s][j];
                s1 += sd->B[s][1] * P[s][j];
            }
            M[0][j] = s0 + P[IDX_DRATE_PREV][j];
            M[1][j] = s1 + P[IDX_ACCEL_PREV][j];
        }

        /* Step 2: S = R_aug + M*B (nu x nu) */
        float S[2][2];
        S[0][0] = r_aug_diag[0]; S[0][1] = 0.0f; S[1][0] = 0.0f; S[1][1] = r_aug_diag[1];
        /* Same pattern as above across
         * [IDX_SPARSE_B_FIRST_ROW, IDX_DRATE_PREV),
         * with identity-channel terms injected below for rows 6 and 7. */
        for (int s = IDX_SPARSE_B_FIRST_ROW; s < IDX_DRATE_PREV; s++) {
            S[0][0] += M[0][s] * sd->B[s][0];
            S[0][1] += M[0][s] * sd->B[s][1];
            S[1][0] += M[1][s] * sd->B[s][0];
            S[1][1] += M[1][s] * sd->B[s][1];
        }
        // Add identity-channel contributions from rows 6 and 7 (if any)
        S[0][0] += M[0][IDX_DRATE_PREV];
        S[0][1] += M[0][IDX_ACCEL_PREV];
        S[1][0] += M[1][IDX_DRATE_PREV];
        S[1][1] += M[1][IDX_ACCEL_PREV];

        /* Step 3: Invert S (2x2) */
        float Si[2][2];
        if (riccati_invert_2x2(S, Si) < 0) {
            g_riccati_debug_last.last_fallback_s00 = S[0][0];
            g_riccati_debug_last.last_fallback_s11 = S[1][1];
            Si[0][0] = S[0][0] != 0.0f ? 1.0f / S[0][0] : 0.0f;
            Si[0][1] = 0.0f;
            Si[1][0] = 0.0f;
            Si[1][1] = S[1][1] != 0.0f ? 1.0f / S[1][1] : 0.0f;
        }

        /* Step 4: G = M*A + N^T (nu x nx) */
        float G[RICCATI_MAX_NU][RICCATI_MAX_NX];
        for (int a = 0; a < nu; a++) {
            /* A has a dense leading block and two zero tail columns.
             * The final two G columns therefore come only from N^T. */
            for (int j = 0; j < NX_DENSE; j++) {
                float sum = sd->N[j][a];  /* N^T[a][j] = N[j][a] */
                for (int s = 0; s < NX_DENSE; s++) {
                    sum += M[a][s] * sd->A[s][j];
                }
                G[a][j] = sum;
            }
            G[a][IDX_DRATE_PREV] = sd->N[IDX_DRATE_PREV][a];
            G[a][IDX_ACCEL_PREV] = sd->N[IDX_ACCEL_PREV][a];
        }

        /* Step 5: K = -S^{-1} G (nu x nx) */
        for (int a = 0; a < nu; a++) {
            for (int j = 0; j < nx; j++) {
                float val = 0.0f;
                for (int b = 0; b < nu; b++) {
                    val += Si[a][b] * G[b][j];
                }
                K[k][a][j] = -val;
            }
        }

        /* Shift p by affine dynamics bias: p_shift = p + P*d */
        float p_shift[RICCATI_MAX_NX];
        for (int i = 0; i < nx; i++) {
            float sum = p[i];
            for (int s = 0; s < NX_DENSE; s++) {
                sum += P[i][s] * sd->d[s];
            }
            p_shift[i] = sum;
        }
        /* Step 6: kk = -S^{-1} (r_aug_linear + B^T p) (nu x 1) */
        float Bp[RICCATI_MAX_NU];
        {
            float bp0 = 0.0f, bp1 = 0.0f;
            for (int s = IDX_SPARSE_B_FIRST_ROW; s < IDX_DRATE_PREV; s++) {
                bp0 += sd->B[s][0] * p_shift[s];
                bp1 += sd->B[s][1] * p_shift[s];
            }
            Bp[0] = bp0 + p_shift[IDX_DRATE_PREV];
            Bp[1] = bp1 + p_shift[IDX_ACCEL_PREV];

        }

        for (int a = 0; a < nu; a++) {
            float val = 0.0f;
            for (int b = 0; b < nu; b++) {
                val += Si[a][b] * (r_aug_linear[b] + Bp[b]);
            }
            kk[k][a] = -val;
        }
        /* Step 7: P_k = Q_aug_diag + A^T*P*A + G^T*K (nx x nx) */
        /* PA = P * A: only the leading NX_DENSE block is non-zero. */
        float PA[RICCATI_MAX_NX][RICCATI_MAX_NX];
        for (int i = 0; i < nx; i++) {
            for (int j = 0; j < NX_DENSE; j++) {
                float sum = 0.0f;
                for (int s = 0; s < NX_DENSE; s++) {
                    sum += P[i][s] * sd->A[s][j];
                }
                PA[i][j] = sum;
            }
            PA[i][IDX_DRATE_PREV] = 0.0f;
            PA[i][IDX_ACCEL_PREV] = 0.0f;
        }

        /* Fused: P = Q_diag + A^T*PA + G^T*K */
        /* Dense block: rows 0..5, cols 0..5 */
        for (int i = 0; i < NX_DENSE; i++) {
            for (int j = 0; j < NX_DENSE; j++) {
                float sum = (i == j) ? q_aug_diag[i] : 0.0f;
                for (int s = 0; s < NX_DENSE; s++) {
                    sum += sd->A[s][i] * PA[s][j];
                }
                for (int a = 0; a < nu; a++) {
                    sum += G[a][i] * K[k][a][j];
                }
                P[i][j] = sum;
            }
            for (int j = IDX_DRATE_PREV; j < nx; j++) {
                float sum = 0.0f;
                for (int a = 0; a < nu; a++) {
                    sum += G[a][i] * K[k][a][j];
                }
                P[i][j] = sum;
            }
        }
        /* Previous-control tail rows have zero A^T rows. */
        for (int i = IDX_DRATE_PREV; i < nx; i++) {
            for (int j = 0; j < nx; j++) {
                float sum = (i == j) ? q_aug_diag[i] : 0.0f;
                for (int a = 0; a < nu; a++) {
                    sum += G[a][i] * K[k][a][j];
                }
                P[i][j] = sum;
            }
        }

        /* Step 8: p_k = q_aug_linear + A^T p_{k+1} + G^T kk (nx x 1) */
        float Atp_vec[NX_DENSE];
        for (int i = 0; i < NX_DENSE; i++) {
            float Atp = 0.0f;
            for (int s = 0; s < NX_DENSE; s++) {
                Atp += sd->A[s][i] * p_shift[s];
            }
            Atp_vec[i] = Atp;
        }
        for (int i = 0; i < NX_DENSE; i++) {
            float Gtk = 0.0f;
            for (int a = 0; a < nu; a++) {
                Gtk += G[a][i] * kk[k][a];
            }
            p[i] = q_aug_linear[i] + Atp_vec[i] + Gtk;
        }
        for (int i = IDX_DRATE_PREV; i < nx; i++) {
            float Gtk = 0.0f;
            for (int a = 0; a < nu; a++) {
                Gtk += G[a][i] * kk[k][a];
            }
            p[i] = q_aug_linear[i] + Gtk;
        }
    }

    /*-------------------------------------------------------------------
     * Forward pass: roll out x, u from x0 using gains K, kk
     *-------------------------------------------------------------------*/
    for (int s = 0; s < nx; s++) {
        x_out[0][s] = x0[s];
    }

    for (int k = 0; k < N; k++) {
        const RiccatiStepData_t *sd = &step_data[k];

        /* u_k = K_k x_k + kk_k */
        for (int a = 0; a < nu; a++) {
            float sum = kk[k][a];
            for (int s = 0; s < nx; s++) {
                sum += K[k][a][s] * x_out[k][s];
            }
            u_out[k][a] = sum;
        }

        /* x_{k+1} = A_k x_k + B_k u_k + d_k */
        for (int i = 0; i < NX_DENSE; i++) {
            float sum = sd->d[i];
            for (int s = 0; s < NX_DENSE; s++) {
                sum += sd->A[i][s] * x_out[k][s];
            }
            for (int a = 0; a < nu; a++) {
                sum += sd->B[i][a] * u_out[k][a];
            }
            x_out[k + 1][i] = sum;
        }
        /* Previous-control tail states copy the current inputs. */
        x_out[k + 1][IDX_DRATE_PREV] = u_out[k][0] + sd->d[IDX_DRATE_PREV];
        x_out[k + 1][IDX_ACCEL_PREV] = u_out[k][1] + sd->d[IDX_ACCEL_PREV];
    }
}

/*===========================================================================
 * Main Solver: Riccati-ADMM
 *===========================================================================*/

RiccatiStatus_t riccati_admm_solve(
    const RiccatiStepData_t *step_data,
    const float *terminal_Q,
    const float *terminal_q,
    const float *terminal_x_lb,
    const float *terminal_x_ub,
    const float *x0,
    int nx, int nu, int N,
    const RiccatiAdmmConfig_t *config,
    RiccatiAdmmState_t *admm_state,
    RiccatiSolution_t *solution)
{
    if (!step_data || !terminal_Q || !terminal_q || !terminal_x_lb || !terminal_x_ub || !x0 || !admm_state || !solution) {
        if (solution) {
            solution->status = RICCATI_STATUS_ERROR;
        }
        return RICCATI_STATUS_ERROR;
    }

    if (nx <= 0 || nx > RICCATI_MAX_NX || nu <= 0 || nu > RICCATI_MAX_NU ||
        N <= 0 || N > PREDICTION_HORIZON) {
        solution->status = RICCATI_STATUS_ERROR;
        return RICCATI_STATUS_ERROR;
    }

    const float cfg_rho = (config && config->rho > 0.0f) ? config->rho : ADMM_RHO;
    const float cfg_rho_u = (config && config->rho_u > 0.0f) ? config->rho_u : ADMM_RHO_U;
    const int cfg_max_iter = (config && config->max_iterations > 0)
                                 ? config->max_iterations
                                 : MAX_ITERATIONS;
    const float tolerance = (config && config->tolerance > 0.0f)
                                ? config->tolerance
                                : CONVERGENCE_TOLERANCE;
    const float abs_tolerance = (tolerance > 1e-6f) ? tolerance : 1e-6f;
    const float rel_tolerance = 0.02f;
    const int adaptive_rho = config ? config->adaptive_rho : 1;
    const int shared_rho = config ? config->shared_rho : 0;

    memset(&g_riccati_debug_last, 0, sizeof(g_riccati_debug_last));
    g_riccati_debug_trace_count = 0;

    float rho = (admm_state->initialized && admm_state->rho > 0.0f)
                    ? admm_state->rho : cfg_rho;
    float rho_u = (admm_state->initialized && admm_state->rho_u > 0.0f)
                    ? admm_state->rho_u : (cfg_rho_u > 0.0f ? cfg_rho_u : rho);
    /* Clamp initial penalties to the configured CPU/replay adaptive range. */
    if (rho < 1.0f) rho = 1.0f;
    if (rho_u < 1.0f) rho_u = 1.0f;
    if (rho > 127.0f) rho = 127.0f;
    if (rho_u > 127.0f) rho_u = 127.0f;
    if (shared_rho) {
        /* A public-API caller may switch an existing warm start from
         * independent penalties to one shared penalty.  Preserve the
         * unscaled control dual before adopting the state rho. */
        if (admm_state->initialized && rho_u != rho) {
            const float y_u_scale = rho_u / rho;
            for (int k = 0; k < N; k++) {
                for (int a = 0; a < nu; a++) {
                    admm_state->y_u[k][a] *= y_u_scale;
                }
            }
        }
        rho_u = rho;
    }
    int max_iter = cfg_max_iter;

    /* ADMM variables (persistent buffers for warm-start reuse). */
    float (*z_x)[RICCATI_MAX_NX] = admm_state->z_x;
    float (*z_u)[RICCATI_MAX_NU] = admm_state->z_u;
    float (*y_x)[RICCATI_MAX_NX] = admm_state->y_x;
    float (*y_u)[RICCATI_MAX_NU] = admm_state->y_u;

    /* Precompute constrained flags */
    uint8_t x_is_constrained[PREDICTION_HORIZON + 1][RICCATI_MAX_NX];
    memset(x_is_constrained, 0, sizeof(x_is_constrained));
    for (int k = 0; k <= N; k++) {
        for (int s = 0; s < nx; s++) {
            const float xub = (k < N) ? step_data[k].x_ub[s] : terminal_x_ub[s];
            const float xlb = (k < N) ? step_data[k].x_lb[s] : terminal_x_lb[s];
            /* x_0 is fixed data.  Only predicted states x_1..x_N can be
             * projected by ADMM. */
            x_is_constrained[k][s] = k > 0 &&
                                     (xub < BIG_BOUND || xlb > -BIG_BOUND);
        }
    }

    if (admm_state->initialized) {
        /* A warm start may contain a projected (or simply stale) stage-zero
         * value.  Keep the fixed initial state out of the split variables. */
        for (int s = 0; s < nx; s++) {
            z_x[0][s] = x0[s];
            y_x[0][s] = 0.0f;
        }
    }

    if (!admm_state->initialized) {
        /* Cold start */
        memset(z_x, 0, sizeof(admm_state->z_x));
        memset(z_u, 0, sizeof(admm_state->z_u));
        memset(y_x, 0, sizeof(admm_state->y_x));
        memset(y_u, 0, sizeof(admm_state->y_u));

        riccati_solver_pass(
            step_data, terminal_Q, terminal_q, terminal_x_lb, terminal_x_ub, x0,
            nx, nu, N, 0.0f, 0.0f,
            (const float (*)[RICCATI_MAX_NX])z_x,
            (const float (*)[RICCATI_MAX_NX])y_x,
            (const float (*)[RICCATI_MAX_NU])z_u,
            (const float (*)[RICCATI_MAX_NU])y_u,
            solution->x, solution->u);

        /* Initialize z from projection of unconstrained solution */
        for (int k = 0; k <= N; k++) {
            for (int s = 0; s < nx; s++) {
                float val = solution->x[k][s];
                const float xlb = (k < N) ? step_data[k].x_lb[s] : terminal_x_lb[s];
                const float xub = (k < N) ? step_data[k].x_ub[s] : terminal_x_ub[s];
                if (x_is_constrained[k][s]) {
                    if (val < xlb) val = xlb;
                    if (val > xub) val = xub;
                }
                z_x[k][s] = val;
            }
        }
        for (int k = 0; k < N; k++) {
            for (int a = 0; a < nu; a++) {
                float val = solution->u[k][a];
                if (val < step_data[k].u_lb[a]) val = step_data[k].u_lb[a];
                if (val > step_data[k].u_ub[a]) val = step_data[k].u_ub[a];
                z_u[k][a] = val;
            }
        }

        /* Initialize y (dual) from constraint violation */
        for (int k = 0; k <= N; k++) {
            for (int s = 0; s < nx; s++) {
                if (x_is_constrained[k][s]) {
                    y_x[k][s] = solution->x[k][s] - z_x[k][s];
                }
            }
        }
        for (int k = 0; k < N; k++) {
            for (int a = 0; a < nu; a++) {
                y_u[k][a] = solution->u[k][a] - z_u[k][a];
            }
        }

        if (g_riccati_debug_trace_enabled &&
            g_riccati_debug_trace_count < RICCATI_DEBUG_TRACE_MAX) {
            RiccatiDebugIterSample_t *sample =
                &g_riccati_debug_trace[g_riccati_debug_trace_count++];
            memset(sample, 0, sizeof(*sample));
            sample->iter = -1;
            sample->rho = 0.0f;
            sample->rho_u = 0.0f;
            sample->u0_steer = solution->u[0][0];
            sample->u0_accel = solution->u[0][1];
            sample->z0_steer = z_u[0][0];
            sample->z0_accel = z_u[0][1];
            sample->y0_steer = y_u[0][0];
            sample->y0_accel = y_u[0][1];
        }
    }

    RiccatiStatus_t status = RICCATI_STATUS_MAX_ITERATIONS;

    for (int iter = 0; iter < max_iter; iter++) {

        /*--- Primal update: Riccati pass with augmented costs ---*/
        riccati_solver_pass(
            step_data, terminal_Q, terminal_q, terminal_x_lb, terminal_x_ub, x0,
            nx, nu, N, rho, rho_u,
            (const float (*)[RICCATI_MAX_NX])z_x,
            (const float (*)[RICCATI_MAX_NX])y_x,
            (const float (*)[RICCATI_MAX_NU])z_u,
            (const float (*)[RICCATI_MAX_NU])y_u,
            solution->x, solution->u);

        float x_norm = 0.0f;
        float u_norm = 0.0f;
        for (int k = 0; k <= N; k++) {
            for (int s = 0; s < nx; s++) {
                x_norm = fmaxf(x_norm, fabsf(solution->x[k][s]));
            }
        }
        for (int k = 0; k < N; k++) {
            for (int a = 0; a < nu; a++) {
                u_norm = fmaxf(u_norm, fabsf(solution->u[k][a]));
            }
        }

        /*--- Fused z-update, y-update, and residual computation ---*/
        float state_primal = 0.0f, state_dual = 0.0f;
        float ctrl_primal = 0.0f, ctrl_dual = 0.0f;
        float z_norm = 0.0f, lambda_norm = 0.0f;

        /* State loop */
        for (int k = 0; k <= N; k++) {
            for (int s = 0; s < nx; s++) {
                if (x_is_constrained[k][s]) {
                    float x_val = solution->x[k][s];
                    float x_hat = x_val;
                    float val = x_hat + y_x[k][s];
                    const float xlb = (k < N) ? step_data[k].x_lb[s] : terminal_x_lb[s];
                    const float xub = (k < N) ? step_data[k].x_ub[s] : terminal_x_ub[s];

                    /* z-update: hard constraint projection (box clipping) */
                    if (val < xlb) val = xlb;
                    if (val > xub) val = xub;

                    float z_new = val;
                    /* Dual residual */
                    float z_prev = z_x[k][s];
                    float dd = fabsf(rho * (z_new - z_prev));
                    state_dual = fmaxf(state_dual, dd);
                    /* y-update: y += x - z */
                    y_x[k][s] = x_hat - z_new + y_x[k][s];
                    /* Primal residual */
                    float pd = fabsf(x_hat - z_new);
                    state_primal = fmaxf(state_primal, pd);
                    z_norm = fmaxf(z_norm, fabsf(z_new));
                    lambda_norm = fmaxf(lambda_norm, fabsf(rho * y_x[k][s]));
                    z_x[k][s] = z_new;
                } else {
                    z_x[k][s] = solution->x[k][s];
                }
            }
        }

        /* Control loop */
        for (int k = 0; k < N; k++) {
            const RiccatiStepData_t *sd = &step_data[k];
            for (int a = 0; a < nu; a++) {
                float u_val = solution->u[k][a];
                float u_hat = u_val;
                /* z-update: z = clip(u_hat + y, lb, ub) */
                float val = u_hat + y_u[k][a];
                if (val < sd->u_lb[a]) val = sd->u_lb[a];
                if (val > sd->u_ub[a]) val = sd->u_ub[a];
                float z_new = val;
                /* Dual residual */
                float z_prev = z_u[k][a];
                float dd = fabsf(rho_u * (z_new - z_prev));
                ctrl_dual = fmaxf(ctrl_dual, dd);
                /* y-update: y += u - z */
                y_u[k][a] = u_hat - z_new + y_u[k][a];
                /* Primal residual */
                float pd = fabsf(u_hat - z_new);
                ctrl_primal = fmaxf(ctrl_primal, pd);
                z_norm = fmaxf(z_norm, fabsf(z_new));
                lambda_norm = fmaxf(lambda_norm, fabsf(rho_u * y_u[k][a]));
                z_u[k][a] = z_new;
            }
        }

        float primal_res = state_primal > ctrl_primal ? state_primal : ctrl_primal;
        float dual_res = state_dual > ctrl_dual ? state_dual : ctrl_dual;
        float primal_scale = fmaxf(fmaxf(x_norm, u_norm), z_norm);
        float eps_primal = abs_tolerance + rel_tolerance * primal_scale;
        float eps_dual = abs_tolerance + rel_tolerance * lambda_norm;

        solution->iterations = iter + 1;
        solution->primal_residual = primal_res;
        solution->dual_residual = dual_res;

        /* Debug output */
        if (riccati_admm_debug && (iter < 5 || iter % 50 == 0 || iter == max_iter - 1)) {
            printf("    ADMM[%3d] p=%.4f<=%.4f(s=%.4f,c=%.4f) d=%.4f<=%.4f(s=%.4f,c=%.4f) rho=%.2f rho_u=%.2f u0=[%.4f,%.3f] z0=[%.4f,%.3f] y0=[%.4f,%.3f]\n",
                   iter,
                   (double)primal_res, (double)eps_primal, (double)state_primal, (double)ctrl_primal,
                   (double)dual_res, (double)eps_dual, (double)state_dual, (double)ctrl_dual,
                   (double)rho, (double)rho_u,
                   (double)solution->u[0][0], (double)solution->u[0][1],
                   (double)z_u[0][0], (double)z_u[0][1],
                   (double)y_u[0][0], (double)y_u[0][1]);
        }

        if (g_riccati_debug_trace_enabled &&
            g_riccati_debug_trace_count < RICCATI_DEBUG_TRACE_MAX) {
            RiccatiDebugIterSample_t *sample =
                &g_riccati_debug_trace[g_riccati_debug_trace_count++];
            sample->iter = iter;
            sample->primal_residual = primal_res;
            sample->dual_residual = dual_res;
            sample->state_primal_residual = state_primal;
            sample->state_dual_residual = state_dual;
            sample->ctrl_primal_residual = ctrl_primal;
            sample->ctrl_dual_residual = ctrl_dual;
            sample->rho = rho;
            sample->rho_u = rho_u;
            sample->u0_steer = solution->u[0][0];
            sample->u0_accel = solution->u[0][1];
            sample->z0_steer = z_u[0][0];
            sample->z0_accel = z_u[0][1];
            sample->y0_steer = y_u[0][0];
            sample->y0_accel = y_u[0][1];
            sample->scale_rho = 0;
            sample->scale_rho_u = 0;
        }

        if (primal_res <= eps_primal && dual_res <= eps_dual) {
            status = RICCATI_STATUS_OPTIMAL;
            break;
        }

        /*--- Adaptive rho (aligned with FPGA HLS solver) ---*/
        if (adaptive_rho && iter > 0) {
            const float adapt_ratio_state = 2.0f;
            const float adapt_ratio_ctrl = 2.0f;
            const float adapt_ratio_shared = 5.0f;
            const float rho_min = 1.0f;
            const float rho_max = 127.0f;

            int scale_rho = 0;
            int scale_rho_u = 0;

            if (shared_rho) {
                if (primal_res > adapt_ratio_shared * dual_res && rho < rho_max) {
                    scale_rho = 1;
                } else if (dual_res > adapt_ratio_shared * primal_res && rho > rho_min) {
                    scale_rho = -1;
                }
                scale_rho_u = scale_rho;
            } else {
                if (state_primal > adapt_ratio_state * state_dual && rho < rho_max) {
                    scale_rho = 1;
                } else if (state_dual > adapt_ratio_state * state_primal && rho > rho_min) {
                    scale_rho = -1;
                }

                if (ctrl_primal > adapt_ratio_ctrl * ctrl_dual && rho_u < rho_max) {
                    scale_rho_u = 1;
                } else if (ctrl_dual > adapt_ratio_ctrl * ctrl_primal && rho_u > rho_min) {
                    scale_rho_u = -1;
                }
            }

            if (g_riccati_debug_trace_enabled &&
                g_riccati_debug_trace_count > 0) {
                RiccatiDebugIterSample_t *sample =
                    &g_riccati_debug_trace[g_riccati_debug_trace_count - 1];
                sample->scale_rho = scale_rho;
                sample->scale_rho_u = scale_rho_u;
            }

            /* Update penalties and rescale the scaled dual variables to keep
             * lambda = rho*y invariant.  Deriving the multiplier from the
             * clamped old/new values is important: rho changes by 1.25/0.75
             * for states, by 2/0.5 for controls, and may hit either bound. */
            if (scale_rho != 0) {
                const float old_rho = rho;
                if (scale_rho > 0) {
                    rho *= 1.25f;
                    if (rho > rho_max) rho = rho_max;
                } else {
                    rho *= 0.75f;
                    if (rho < rho_min) rho = rho_min;
                }
                const float y_x_scale = old_rho / rho;
                for (int k = 1; k <= N; k++) {
                    for (int s = 0; s < nx; s++) {
                        if (x_is_constrained[k][s]) {
                            y_x[k][s] *= y_x_scale;
                        }
                    }
                }
            }

            if (shared_rho) {
                const float old_rho_u = rho_u;
                rho_u = rho;
                if (rho_u != old_rho_u) {
                    const float y_u_scale = old_rho_u / rho_u;
                    for (int k = 0; k < N; k++) {
                        for (int a = 0; a < nu; a++) {
                            y_u[k][a] *= y_u_scale;
                        }
                    }
                }
            } else if (scale_rho_u != 0) {
                const float old_rho_u = rho_u;
                if (scale_rho_u > 0) {
                    rho_u *= 2.0f;
                    if (rho_u > rho_max) rho_u = rho_max;
                } else {
                    rho_u *= 0.5f;
                    if (rho_u < rho_min) rho_u = rho_min;
                }
                const float y_u_scale = old_rho_u / rho_u;
                for (int k = 0; k < N; k++) {
                    for (int a = 0; a < nu; a++) {
                        y_u[k][a] *= y_u_scale;
                    }
                }
            }
        }
    }

    /* Save scalar warm-start metadata. Buffers are already updated in-place. */
    admm_state->rho = rho;
    admm_state->rho_u = shared_rho ? rho : rho_u;
    admm_state->initialized = 1;

    g_riccati_debug_last.rho = rho;
    g_riccati_debug_last.rho_u = rho_u;
    g_riccati_debug_last.primal_residual = solution->primal_residual;
    g_riccati_debug_last.dual_residual = solution->dual_residual;

    /* Output feasible controls: z_u is the ADMM projection */
    /* Return the ADMM projection z_u (not the primal u) as the feasible control,
     * because z_u is guaranteed to satisfy box constraints whereas u may not be. */
    memcpy(solution->u, z_u, sizeof(admm_state->z_u));

    solution->status = status;
    return status;
}
