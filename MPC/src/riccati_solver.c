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
        const RiccatiStepData_t *last_sd = &step_data[N - 1];
        // Add ADMM penalty to terminal cost for constrained state channels
        for (int s = 0; s < nx; s++) {
            int is_constrained = (last_sd->x_ub[s] < BIG_BOUND ||
                                  last_sd->x_lb[s] > -BIG_BOUND);
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
            int is_constrained = (sd->x_ub[s] < BIG_BOUND ||
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
             * Rows 6 and 7 are handled explicitly below
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
            Si[0][0] = S[0][0] != 0.0f ? 1.0f / S[0][0] : 0.0f;
            Si[0][1] = 0.0f;
            Si[1][0] = 0.0f;
            Si[1][1] = S[1][1] != 0.0f ? 1.0f / S[1][1] : 0.0f;
        }

        /* Step 4: G = M*A + N^T (nu x nx) */
        float G[RICCATI_MAX_NU][RICCATI_MAX_NX];
        for (int a = 0; a < nu; a++) {
            /* A has a dense 6x6 leading block and zero columns 6..7.
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

        /* Step 6: kk = -S^{-1} (r_aug_linear + B^T p) (nu x 1) */
        float Bp[RICCATI_MAX_NU];
        {
            float bp0 = 0.0f, bp1 = 0.0f;
            for (int s = IDX_SPARSE_B_FIRST_ROW; s < IDX_DRATE_PREV; s++) {
                bp0 += sd->B[s][0] * p[s];
                bp1 += sd->B[s][1] * p[s];
            }
            Bp[0] = bp0 + p[IDX_DRATE_PREV];
            Bp[1] = bp1 + p[IDX_ACCEL_PREV];
        }

        for (int a = 0; a < nu; a++) {
            float val = 0.0f;
            for (int b = 0; b < nu; b++) {
                val += Si[a][b] * (r_aug_linear[b] + Bp[b]);
            }
            kk[k][a] = -val;
        }

        /* Step 7: P_k = Q_aug_diag + A^T*P*A + G^T*K (nx x nx) */
        /* PA = P * A: only 6x6 dense block */
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
        /* Rows 6,7: A^T rows 6,7 zero, only G^T*K + Q_diag */
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
                Atp += sd->A[s][i] * p[s];
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

        /* x_{k+1} = A_k x_k + B_k u_k */
        for (int i = 0; i < NX_DENSE; i++) {
            float sum = 0.0f;
            for (int s = 0; s < NX_DENSE; s++) {
                sum += sd->A[i][s] * x_out[k][s];
            }
            for (int a = 0; a < nu; a++) {
                sum += sd->B[i][a] * u_out[k][a];
            }
            x_out[k + 1][i] = sum;
        }
        /* Rows 6,7: x_prev = u */
        x_out[k + 1][IDX_DRATE_PREV] = u_out[k][0];
        x_out[k + 1][IDX_ACCEL_PREV] = u_out[k][1];
    }
}

/*===========================================================================
 * Main Solver: Riccati-ADMM
 *===========================================================================*/

RiccatiStatus_t riccati_admm_solve(
    const RiccatiStepData_t *step_data,
    const float *terminal_Q,
    const float *terminal_q,
    const float *x0,
    int nx, int nu, int N,
    const RiccatiAdmmConfig_t *config,
    RiccatiAdmmState_t *admm_state,
    RiccatiSolution_t *solution)
{
    if (!step_data || !terminal_Q || !terminal_q || !x0 || !admm_state || !solution) {
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
    const int adaptive_rho = config ? config->adaptive_rho : 1;
    const float alpha_or = (config && config->alpha > 0.0f) ? config->alpha : ADMM_ALPHA;

    float rho = (admm_state->initialized && admm_state->rho > 0.0f)
                    ? admm_state->rho : cfg_rho;
    float rho_u = (admm_state->initialized && admm_state->rho_u > 0.0f)
                    ? admm_state->rho_u : (cfg_rho_u > 0.0f ? cfg_rho_u : rho);
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
        const RiccatiStepData_t *sd = (k < N) ? &step_data[k] : &step_data[N - 1];
        for (int s = 0; s < nx; s++) {
            x_is_constrained[k][s] = (sd->x_ub[s] < BIG_BOUND ||
                                       sd->x_lb[s] > -BIG_BOUND);
        }
    }

    if (!admm_state->initialized) {
        /* Cold start */
        memset(z_x, 0, sizeof(admm_state->z_x));
        memset(z_u, 0, sizeof(admm_state->z_u));
        memset(y_x, 0, sizeof(admm_state->y_x));
        memset(y_u, 0, sizeof(admm_state->y_u));

        riccati_solver_pass(
            step_data, terminal_Q, terminal_q, x0,
            nx, nu, N, 0.0f, 0.0f,
            (const float (*)[RICCATI_MAX_NX])z_x,
            (const float (*)[RICCATI_MAX_NX])y_x,
            (const float (*)[RICCATI_MAX_NU])z_u,
            (const float (*)[RICCATI_MAX_NU])y_u,
            solution->x, solution->u);

        /* Initialize z from projection of unconstrained solution */
        for (int k = 0; k <= N; k++) {
            const RiccatiStepData_t *sd = (k < N) ? &step_data[k] : &step_data[N - 1];
            for (int s = 0; s < nx; s++) {
                float val = solution->x[k][s];
                if (val < sd->x_lb[s]) val = sd->x_lb[s];
                if (val > sd->x_ub[s]) val = sd->x_ub[s];
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
    }

    RiccatiStatus_t status = RICCATI_STATUS_MAX_ITERATIONS;

    for (int iter = 0; iter < max_iter; iter++) {

        /*--- Primal update: Riccati pass with augmented costs ---*/
        riccati_solver_pass(
            step_data, terminal_Q, terminal_q, x0,
            nx, nu, N, rho, rho_u,
            (const float (*)[RICCATI_MAX_NX])z_x,
            (const float (*)[RICCATI_MAX_NX])y_x,
            (const float (*)[RICCATI_MAX_NU])z_u,
            (const float (*)[RICCATI_MAX_NU])y_u,
            solution->x, solution->u);

        /*--- Fused z-update, y-update, and residual computation ---*/
        float state_primal = 0.0f, state_dual = 0.0f;
        float ctrl_primal = 0.0f, ctrl_dual = 0.0f;

        /* Over-relaxation parameters */
        const float one_minus_alpha = 1.0f - alpha_or;

        /* State loop */
        for (int k = 0; k <= N; k++) {
            const RiccatiStepData_t *sd = (k < N) ? &step_data[k] : &step_data[N - 1];
            for (int s = 0; s < nx; s++) {
                if (x_is_constrained[k][s]) {
                    float x_val = solution->x[k][s];
                    /* Over-relaxation: x_hat = alpha*x + (1-alpha)*z_old */
                    float x_hat = alpha_or * x_val + one_minus_alpha * z_x[k][s];
                    float val = x_hat + y_x[k][s];

                    /* z-update: hard constraint projection (box clipping) */
                    if (val < sd->x_lb[s]) val = sd->x_lb[s];
                    if (val > sd->x_ub[s]) val = sd->x_ub[s];

                    float z_new = val;
                    /* Dual residual */
                    float z_prev = z_x[k][s];
                    float dd = fabsf(rho * (z_new - z_prev));
                    state_dual = fmaxf(state_dual, dd);
                    /* y-update: y += x_hat - z (over-relaxed per Boyd et al.) */
                    y_x[k][s] = x_hat - z_new + y_x[k][s];
                    /* Primal residual */
                    float pd = fabsf(x_hat - z_new);
                    state_primal = fmaxf(state_primal, pd);
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
                /* Over-relaxation: u_hat = alpha*u + (1-alpha)*z_old */
                float u_hat = alpha_or * u_val + one_minus_alpha * z_u[k][a];
                /* z-update: z = clip(u_hat + y, lb, ub) */
                float val = u_hat + y_u[k][a];
                if (val < sd->u_lb[a]) val = sd->u_lb[a];
                if (val > sd->u_ub[a]) val = sd->u_ub[a];
                float z_new = val;
                /* Dual residual */
                float z_prev = z_u[k][a];
                float dd = fabsf(rho_u * (z_new - z_prev));
                ctrl_dual = fmaxf(ctrl_dual, dd);
                /* y-update: y += u_hat - z (over-relaxed per Boyd et al.) */
                y_u[k][a] = u_hat - z_new + y_u[k][a];
                /* Primal residual */
                float pd = fabsf(u_hat - z_new);
                ctrl_primal = fmaxf(ctrl_primal, pd);
                z_u[k][a] = z_new;
            }
        }

        float primal_res = state_primal > ctrl_primal ? state_primal : ctrl_primal;
        float dual_res = state_dual > ctrl_dual ? state_dual : ctrl_dual;

        solution->iterations = iter + 1;
        solution->primal_residual = primal_res;
        solution->dual_residual = dual_res;

        /* Debug output */
        if (riccati_admm_debug && (iter < 5 || iter % 50 == 0 || iter == max_iter - 1)) {
            printf("    ADMM[%3d] p=%.4f(s=%.4f,c=%.4f) d=%.4f(s=%.4f,c=%.4f) rho=%.2f rho_u=%.2f u0=[%.4f,%.3f] z0=[%.4f,%.3f] y0=[%.4f,%.3f]\n",
                   iter,
                   (double)primal_res, (double)state_primal, (double)ctrl_primal,
                   (double)dual_res, (double)state_dual, (double)ctrl_dual,
                   (double)rho, (double)rho_u,
                   (double)solution->u[0][0], (double)solution->u[0][1],
                   (double)z_u[0][0], (double)z_u[0][1],
                   (double)y_u[0][0], (double)y_u[0][1]);
        }

        if (primal_res <= tolerance && dual_res <= tolerance) {
            status = RICCATI_STATUS_OPTIMAL;
            break;
        }

        /*--- Adaptive rho: every 2 iterations ---*/
        if (adaptive_rho && iter > 0 && (iter & 1) == 0) {
            if (primal_res > 10.0f * dual_res && rho < 100.0f) {
                rho *= 2.0f;
                if (rho_u < 100.0f)
                    rho_u *= 2.0f;
                /* Scale dual variables: y = y / 2 */
                for (int k = 0; k <= N; k++)
                    for (int s = 0; s < nx; s++)
                        y_x[k][s] *= 0.5f;
                for (int k = 0; k < N; k++)
                    for (int a = 0; a < nu; a++)
                        y_u[k][a] *= 0.5f;
            } else if (dual_res > 10.0f * primal_res && rho > 0.5f) {
                rho *= 0.5f;
                if (rho_u > 0.5f)
                    rho_u *= 0.5f;
                /* Scale dual variables: y = y * 2 */
                for (int k = 0; k <= N; k++)
                    for (int s = 0; s < nx; s++)
                        y_x[k][s] *= 2.0f;
                for (int k = 0; k < N; k++)
                    for (int a = 0; a < nu; a++)
                        y_u[k][a] *= 2.0f;
            }
        }
    }

    /* Save scalar warm-start metadata. Buffers are already updated in-place. */
    admm_state->rho = rho;
    admm_state->rho_u = rho_u;
    admm_state->initialized = 1;

    /* Output feasible controls: z_u is the ADMM projection */
    /* Return the ADMM projection z_u (not the primal u) as the feasible control,
     * because z_u is guaranteed to satisfy box constraints whereas u may not be. */
    memcpy(solution->u, z_u, sizeof(admm_state->z_u));

    solution->status = status;
    return status;
}
