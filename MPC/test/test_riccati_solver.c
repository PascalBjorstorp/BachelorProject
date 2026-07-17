#include "riccati_solver.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define TEST_HORIZON 2
#define DENSE_REFERENCE_HORIZON 3

static int failures = 0;

static void check_true(int condition, const char *message)
{
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", message);
        failures++;
    }
}

static void check_close(float actual, float expected, float tolerance,
                        const char *message)
{
    if (!isfinite(actual) || fabsf(actual - expected) > tolerance) {
        fprintf(stderr, "FAIL: %s (actual=%.9g expected=%.9g tol=%.3g)\n",
                message, (double)actual, (double)expected,
                (double)tolerance);
        failures++;
    }
}

/* Generic dense Riccati recursion used only as an independent numerical
 * reference for the production solver's structured matrix products. */
static int dense_reference_pass(
    const RiccatiStepData_t step_data[DENSE_REFERENCE_HORIZON],
    const float terminal_Q[NX_AUG],
    const float terminal_q[NX_AUG],
    const float terminal_lb[NX_AUG],
    const float terminal_ub[NX_AUG],
    const float x0[NX_AUG],
    float rho,
    float rho_u,
    const float z_x[][RICCATI_MAX_NX],
    const float y_x[][RICCATI_MAX_NX],
    const float z_u[][RICCATI_MAX_NU],
    const float y_u[][RICCATI_MAX_NU],
    double x_out[][RICCATI_MAX_NX],
    double u_out[][RICCATI_MAX_NU])
{
    double gains[DENSE_REFERENCE_HORIZON][NU][NX_AUG];
    double feedforward[DENSE_REFERENCE_HORIZON][NU];
    double P[NX_AUG][NX_AUG] = {{0.0}};
    double p[NX_AUG] = {0.0};

    for (int i = 0; i < NX_AUG; i++) {
        const int constrained = terminal_ub[i] < BIG_BOUND ||
                                terminal_lb[i] > -BIG_BOUND;
        P[i][i] = terminal_Q[i] + (constrained ? rho : 0.0);
        p[i] = terminal_q[i];
        if (constrained) {
            p[i] -= rho * (z_x[DENSE_REFERENCE_HORIZON][i] -
                           y_x[DENSE_REFERENCE_HORIZON][i]);
        }
    }

    for (int k = DENSE_REFERENCE_HORIZON - 1; k >= 0; k--) {
        const RiccatiStepData_t *sd = &step_data[k];
        double q_diag[NX_AUG];
        double q_linear[NX_AUG];
        double r_diag[NU];
        double r_linear[NU];
        double M[NU][NX_AUG];
        double S[NU][NU] = {{0.0}};
        double S_inv[NU][NU];
        double G[NU][NX_AUG];
        double p_shift[NX_AUG];
        double PA[NX_AUG][NX_AUG];
        double P_next[NX_AUG][NX_AUG];
        double p_next[NX_AUG];

        for (int i = 0; i < NX_AUG; i++) {
            const int constrained = k > 0 &&
                                    (sd->x_ub[i] < BIG_BOUND ||
                                     sd->x_lb[i] > -BIG_BOUND);
            q_diag[i] = sd->Q_diag[i] + (constrained ? rho : 0.0);
            q_linear[i] = sd->q[i];
            if (constrained) {
                q_linear[i] -= rho * (z_x[k][i] - y_x[k][i]);
            }
        }
        for (int a = 0; a < NU; a++) {
            r_diag[a] = sd->R_diag[a] + rho_u;
            r_linear[a] = sd->r[a] -
                          rho_u * (z_u[k][a] - y_u[k][a]);
        }

        for (int a = 0; a < NU; a++) {
            for (int j = 0; j < NX_AUG; j++) {
                double sum = 0.0;
                for (int i = 0; i < NX_AUG; i++) {
                    sum += sd->B[i][a] * P[i][j];
                }
                M[a][j] = sum;
            }
        }
        for (int a = 0; a < NU; a++) {
            for (int b = 0; b < NU; b++) {
                double sum = a == b ? r_diag[a] : 0.0;
                for (int i = 0; i < NX_AUG; i++) {
                    sum += M[a][i] * sd->B[i][b];
                }
                S[a][b] = sum;
            }
        }

        const double determinant = S[0][0] * S[1][1] -
                                   S[0][1] * S[1][0];
        if (fabs(determinant) < 1e-12) {
            return -1;
        }
        S_inv[0][0] = S[1][1] / determinant;
        S_inv[0][1] = -S[0][1] / determinant;
        S_inv[1][0] = -S[1][0] / determinant;
        S_inv[1][1] = S[0][0] / determinant;

        for (int a = 0; a < NU; a++) {
            for (int j = 0; j < NX_AUG; j++) {
                double sum = sd->N[j][a];
                for (int i = 0; i < NX_AUG; i++) {
                    sum += M[a][i] * sd->A[i][j];
                }
                G[a][j] = sum;
            }
        }
        for (int a = 0; a < NU; a++) {
            for (int j = 0; j < NX_AUG; j++) {
                double sum = 0.0;
                for (int b = 0; b < NU; b++) {
                    sum += S_inv[a][b] * G[b][j];
                }
                gains[k][a][j] = -sum;
            }
        }

        for (int i = 0; i < NX_AUG; i++) {
            double sum = p[i];
            for (int j = 0; j < NX_AUG; j++) {
                sum += P[i][j] * sd->d[j];
            }
            p_shift[i] = sum;
        }
        for (int a = 0; a < NU; a++) {
            feedforward[k][a] = 0.0;
            for (int b = 0; b < NU; b++) {
                double b_gradient = r_linear[b];
                for (int i = 0; i < NX_AUG; i++) {
                    b_gradient += sd->B[i][b] * p_shift[i];
                }
                feedforward[k][a] -= S_inv[a][b] * b_gradient;
            }
        }

        for (int i = 0; i < NX_AUG; i++) {
            for (int j = 0; j < NX_AUG; j++) {
                double sum = 0.0;
                for (int s = 0; s < NX_AUG; s++) {
                    sum += P[i][s] * sd->A[s][j];
                }
                PA[i][j] = sum;
            }
        }
        for (int i = 0; i < NX_AUG; i++) {
            for (int j = 0; j < NX_AUG; j++) {
                double sum = i == j ? q_diag[i] : 0.0;
                for (int s = 0; s < NX_AUG; s++) {
                    sum += sd->A[s][i] * PA[s][j];
                }
                for (int a = 0; a < NU; a++) {
                    sum += G[a][i] * gains[k][a][j];
                }
                P_next[i][j] = sum;
            }

            double sum = q_linear[i];
            for (int s = 0; s < NX_AUG; s++) {
                sum += sd->A[s][i] * p_shift[s];
            }
            for (int a = 0; a < NU; a++) {
                sum += G[a][i] * feedforward[k][a];
            }
            p_next[i] = sum;
        }
        memcpy(P, P_next, sizeof(P));
        memcpy(p, p_next, sizeof(p));
    }

    for (int i = 0; i < NX_AUG; i++) {
        x_out[0][i] = x0[i];
    }
    for (int k = 0; k < DENSE_REFERENCE_HORIZON; k++) {
        const RiccatiStepData_t *sd = &step_data[k];
        for (int a = 0; a < NU; a++) {
            double sum = feedforward[k][a];
            for (int i = 0; i < NX_AUG; i++) {
                sum += gains[k][a][i] * x_out[k][i];
            }
            u_out[k][a] = sum;
        }
        for (int i = 0; i < NX_AUG; i++) {
            double sum = sd->d[i];
            for (int j = 0; j < NX_AUG; j++) {
                sum += sd->A[i][j] * x_out[k][j];
            }
            for (int a = 0; a < NU; a++) {
                sum += sd->B[i][a] * u_out[k][a];
            }
            x_out[k + 1][i] = sum;
        }
    }
    return 0;
}

static void test_nine_state_sparse_pass_matches_dense_reference(void)
{
    RiccatiStepData_t step_data[DENSE_REFERENCE_HORIZON];
    float terminal_Q[NX_AUG], terminal_q[NX_AUG];
    float terminal_lb[NX_AUG], terminal_ub[NX_AUG];
    float x0[NX_AUG];
    float z_x[PREDICTION_HORIZON + 1][RICCATI_MAX_NX] = {{0.0f}};
    float y_x[PREDICTION_HORIZON + 1][RICCATI_MAX_NX] = {{0.0f}};
    float z_u[PREDICTION_HORIZON][RICCATI_MAX_NU] = {{0.0f}};
    float y_u[PREDICTION_HORIZON][RICCATI_MAX_NU] = {{0.0f}};
    float sparse_x[PREDICTION_HORIZON + 1][RICCATI_MAX_NX] = {{0.0f}};
    float sparse_u[PREDICTION_HORIZON][RICCATI_MAX_NU] = {{0.0f}};
    double dense_x[PREDICTION_HORIZON + 1][RICCATI_MAX_NX] = {{0.0}};
    double dense_u[PREDICTION_HORIZON][RICCATI_MAX_NU] = {{0.0}};
    const float rho = 3.25f;
    const float rho_u = 2.75f;

    check_true(NX_AUG == 9, "effective-steering model must have nine states");
    check_true(NX_DENSE == 7, "effective-steering dense prefix must have seven states");
    check_true(IDX_DRATE_PREV == NX_DENSE,
               "previous-control tail must start after the dense prefix");
    check_true(IDX_ACCEL_PREV == NX_DENSE + 1,
               "acceleration tail state must follow steering-rate state");

    memset(step_data, 0, sizeof(step_data));
    for (int i = 0; i < NX_AUG; i++) {
        x0[i] = 0.04f * (float)(i - 3);
        terminal_Q[i] = 0.9f + 0.13f * (float)i;
        terminal_q[i] = 0.017f * (float)(3 - i);
        terminal_lb[i] = -BIG_BOUND;
        terminal_ub[i] = BIG_BOUND;
        for (int k = 0; k <= DENSE_REFERENCE_HORIZON; k++) {
            z_x[k][i] = 0.011f * (float)((k + 2) * (i + 1));
            y_x[k][i] = -0.007f * (float)((k + 1) * (i - 2));
        }
    }
    terminal_lb[IDX_EY] = -0.8f;
    terminal_ub[IDX_EY] = 0.7f;
    terminal_lb[IDX_DELTA_COMMAND] = -0.39f;
    terminal_ub[IDX_DELTA_COMMAND] = 0.39f;

    for (int k = 0; k < DENSE_REFERENCE_HORIZON; k++) {
        RiccatiStepData_t *sd = &step_data[k];
        for (int i = 0; i < NX_AUG; i++) {
            sd->Q_diag[i] = 0.55f + 0.09f * (float)(i + k);
            sd->q[i] = 0.013f * (float)((i % 3) - k);
            sd->x_lb[i] = -BIG_BOUND;
            sd->x_ub[i] = BIG_BOUND;
        }
        for (int i = 0; i < NX_DENSE; i++) {
            sd->d[i] = 0.003f * (float)(i - 2 * k);
            for (int j = 0; j < NX_DENSE; j++) {
                if (i == j) {
                    sd->A[i][j] = 0.72f + 0.015f * (float)(i + k);
                } else {
                    const int pattern = ((i + 2 * j + k) % 5) - 2;
                    sd->A[i][j] = 0.008f * (float)pattern;
                }
            }
        }
        for (int i = IDX_SPARSE_B_FIRST_ROW; i < NX_DENSE; i++) {
            sd->B[i][0] = 0.012f * (float)(i - 1 + k);
            sd->B[i][1] = -0.009f * (float)(NX_DENSE - i + k);
        }
        sd->B[IDX_DRATE_PREV][0] = 1.0f;
        sd->B[IDX_ACCEL_PREV][1] = 1.0f;
        sd->N[IDX_DELTA_EFFECTIVE][0] = 0.006f * (float)(k + 1);
        sd->N[IDX_DRATE_PREV][0] = -0.08f;
        sd->N[IDX_ACCEL_PREV][1] = -0.06f;
        sd->R_diag[0] = 1.15f + 0.1f * (float)k;
        sd->R_diag[1] = 1.4f + 0.08f * (float)k;
        sd->r[0] = -0.025f * (float)(k + 1);
        sd->r[1] = 0.019f * (float)(k + 1);
        sd->u_lb[0] = -2.0f;
        sd->u_ub[0] = 2.0f;
        sd->u_lb[1] = -3.0f;
        sd->u_ub[1] = 3.0f;
        sd->x_lb[IDX_EY] = -0.65f;
        sd->x_ub[IDX_EY] = 0.6f;
        sd->x_lb[IDX_DELTA_COMMAND] = -0.39f;
        sd->x_ub[IDX_DELTA_COMMAND] = 0.39f;
        for (int a = 0; a < NU; a++) {
            z_u[k][a] = 0.037f * (float)((k + 1) * (a + 1));
            y_u[k][a] = -0.021f * (float)(k + a + 1);
        }
    }

    riccati_solver_pass(
        step_data, terminal_Q, terminal_q, terminal_lb, terminal_ub, x0,
        NX_AUG, NU, DENSE_REFERENCE_HORIZON, rho, rho_u,
        (const float (*)[RICCATI_MAX_NX])z_x,
        (const float (*)[RICCATI_MAX_NX])y_x,
        (const float (*)[RICCATI_MAX_NU])z_u,
        (const float (*)[RICCATI_MAX_NU])y_u,
        sparse_x, sparse_u);
    const int reference_status = dense_reference_pass(
        step_data, terminal_Q, terminal_q, terminal_lb, terminal_ub, x0,
        rho, rho_u,
        (const float (*)[RICCATI_MAX_NX])z_x,
        (const float (*)[RICCATI_MAX_NX])y_x,
        (const float (*)[RICCATI_MAX_NU])z_u,
        (const float (*)[RICCATI_MAX_NU])y_u,
        dense_x, dense_u);
    check_true(reference_status == 0,
               "dense reference control Hessian must be invertible");
    if (reference_status != 0) {
        return;
    }

    for (int k = 0; k <= DENSE_REFERENCE_HORIZON; k++) {
        for (int i = 0; i < NX_AUG; i++) {
            char message[112];
            snprintf(message, sizeof(message),
                     "nine-state sparse/dense state mismatch at k=%d i=%d",
                     k, i);
            check_close(sparse_x[k][i], (float)dense_x[k][i], 5e-5f,
                        message);
        }
    }
    for (int k = 0; k < DENSE_REFERENCE_HORIZON; k++) {
        for (int a = 0; a < NU; a++) {
            char message[112];
            snprintf(message, sizeof(message),
                     "nine-state sparse/dense control mismatch at k=%d a=%d",
                     k, a);
            check_close(sparse_u[k][a], (float)dense_u[k][a], 5e-5f,
                        message);
        }
    }
}

static void initialize_problem(
    RiccatiStepData_t step_data[TEST_HORIZON],
    float terminal_Q[NX_AUG],
    float terminal_q[NX_AUG],
    float terminal_lb[NX_AUG],
    float terminal_ub[NX_AUG])
{
    memset(step_data, 0, sizeof(RiccatiStepData_t) * TEST_HORIZON);
    memset(terminal_Q, 0, sizeof(float) * NX_AUG);
    memset(terminal_q, 0, sizeof(float) * NX_AUG);

    for (int s = 0; s < NX_AUG; s++) {
        terminal_lb[s] = -BIG_BOUND;
        terminal_ub[s] = BIG_BOUND;
    }

    for (int k = 0; k < TEST_HORIZON; k++) {
        for (int s = 0; s < NX_AUG; s++) {
            step_data[k].x_lb[s] = -BIG_BOUND;
            step_data[k].x_ub[s] = BIG_BOUND;
        }
        for (int s = 0; s < NX_DENSE; s++) {
            step_data[k].A[s][s] = 1.0f;
        }
        for (int a = 0; a < NU; a++) {
            step_data[k].R_diag[a] = 1.0f;
            step_data[k].u_lb[a] = -BIG_BOUND;
            step_data[k].u_ub[a] = BIG_BOUND;
        }
    }
}

static RiccatiAdmmConfig_t solver_config(float rho, float rho_u,
                                         int adaptive_rho, int shared_rho)
{
    RiccatiAdmmConfig_t config;
    config.rho = rho;
    config.rho_u = rho_u;
    config.tolerance = 1e-6f;
    config.max_iterations = 2;
    config.adaptive_rho = adaptive_rho;
    config.shared_rho = shared_rho;
    return config;
}

static void test_fixed_x0_is_not_an_admm_constraint(void)
{
    RiccatiStepData_t step_data[TEST_HORIZON];
    float terminal_Q[NX_AUG], terminal_q[NX_AUG];
    float terminal_lb[NX_AUG], terminal_ub[NX_AUG];
    float x0[NX_AUG] = {0.0f};
    RiccatiAdmmState_t state;
    RiccatiSolution_t solution;

    initialize_problem(step_data, terminal_Q, terminal_q,
                       terminal_lb, terminal_ub);
    x0[IDX_EY] = 10.0f;
    step_data[0].x_lb[IDX_EY] = -1.0f;
    step_data[0].x_ub[IDX_EY] = 1.0f;

    /* Seed a legacy warm start that projected x_0 and accumulated a dual. */
    riccati_admm_state_init(&state);
    state.initialized = 1;
    state.rho = 4.0f;
    state.rho_u = 4.0f;
    state.z_x[0][IDX_EY] = 1.0f;
    state.y_x[0][IDX_EY] = 9.0f;

    RiccatiAdmmConfig_t config = solver_config(4.0f, 4.0f, 0, 0);
    RiccatiStatus_t status = riccati_admm_solve(
        step_data, terminal_Q, terminal_q, terminal_lb, terminal_ub, x0,
        NX_AUG, NU, TEST_HORIZON, &config, &state, &solution);

    check_true(status == RICCATI_STATUS_OPTIMAL,
               "an out-of-bounds fixed x0 must not prevent convergence");
    check_close(solution.primal_residual, 0.0f, 1e-7f,
                "fixed x0 must not contribute to the primal residual");
    check_close(state.z_x[0][IDX_EY], x0[IDX_EY], 1e-7f,
                "the stage-zero split value must equal x0");
    check_close(state.y_x[0][IDX_EY], 0.0f, 1e-7f,
                "the stage-zero dual must be cleared");

    riccati_admm_state_init(&state);
    status = riccati_admm_solve(
        step_data, terminal_Q, terminal_q, terminal_lb, terminal_ub, x0,
        NX_AUG, NU, TEST_HORIZON, &config, &state, &solution);
    check_true(status == RICCATI_STATUS_OPTIMAL,
               "cold-start x0 bounds must also be ignored");
    check_close(state.z_x[0][IDX_EY], x0[IDX_EY], 1e-7f,
                "cold-start stage-zero split value must equal x0");
    check_close(state.y_x[0][IDX_EY], 0.0f, 1e-7f,
                "cold-start stage-zero dual must remain zero");
}

static void run_state_rho_invariance_case(float initial_rho,
                                          float expected_adapted_rho)
{
    RiccatiStepData_t step_data[TEST_HORIZON];
    float terminal_Q[NX_AUG], terminal_q[NX_AUG];
    float terminal_lb[NX_AUG], terminal_ub[NX_AUG];
    float x0[NX_AUG] = {0.0f};
    RiccatiAdmmState_t adaptive_state, fixed_state;
    RiccatiSolution_t adaptive_solution, fixed_solution;

    initialize_problem(step_data, terminal_Q, terminal_q,
                       terminal_lb, terminal_ub);

    /* x_1 and x_2 are fixed outside their boxes.  z settles on the box in
     * iteration zero, so iteration one has a primal-only state residual and
     * deterministically requests a rho increase. */
    step_data[0].d[IDX_EY] = 2.0f;
    step_data[1].A[IDX_EY][IDX_EY] = 0.0f;
    step_data[1].d[IDX_EY] = 2.0f;
    step_data[1].x_lb[IDX_EY] = -1.0f;
    step_data[1].x_ub[IDX_EY] = 1.0f;
    terminal_lb[IDX_EY] = -1.0f;
    terminal_ub[IDX_EY] = 1.0f;

    riccati_admm_state_init(&adaptive_state);
    adaptive_state.initialized = 1;
    adaptive_state.rho = initial_rho;
    adaptive_state.rho_u = 4.0f;
    fixed_state = adaptive_state;

    RiccatiAdmmConfig_t adaptive_config =
        solver_config(initial_rho, 4.0f, 1, 0);
    RiccatiAdmmConfig_t fixed_config =
        solver_config(initial_rho, 4.0f, 0, 0);

    (void)riccati_admm_solve(
        step_data, terminal_Q, terminal_q, terminal_lb, terminal_ub, x0,
        NX_AUG, NU, TEST_HORIZON, &adaptive_config,
        &adaptive_state, &adaptive_solution);
    (void)riccati_admm_solve(
        step_data, terminal_Q, terminal_q, terminal_lb, terminal_ub, x0,
        NX_AUG, NU, TEST_HORIZON, &fixed_config,
        &fixed_state, &fixed_solution);

    check_close(adaptive_state.rho, expected_adapted_rho, 1e-6f,
                "adaptive state rho must keep its configured update cadence");
    check_close(fixed_state.rho, initial_rho, 1e-6f,
                "rho must remain fixed when adaptation is disabled");

    for (int k = 1; k <= TEST_HORIZON; k++) {
        const float adaptive_lambda =
            adaptive_state.rho * adaptive_state.y_x[k][IDX_EY];
        const float fixed_lambda =
            fixed_state.rho * fixed_state.y_x[k][IDX_EY];
        check_close(adaptive_lambda, fixed_lambda, 2e-5f,
                    "state rho adaptation must preserve lambda=rho*y");
    }
}

static void test_state_rho_rescaling(void)
{
    run_state_rho_invariance_case(4.0f, 5.0f);
    /* Also verifies that a clamped update uses 126/127, not a hard-coded
     * multiplier intended for an unclamped 1.25x change. */
    run_state_rho_invariance_case(126.0f, 127.0f);
}

static void test_state_rho_decrease_rescaling(void)
{
    RiccatiStepData_t step_data[TEST_HORIZON];
    float terminal_Q[NX_AUG], terminal_q[NX_AUG];
    float terminal_lb[NX_AUG], terminal_ub[NX_AUG];
    float x0[NX_AUG] = {0.0f};
    RiccatiAdmmState_t adaptive_state, fixed_state;
    RiccatiSolution_t adaptive_solution, fixed_solution;

    initialize_problem(step_data, terminal_Q, terminal_q,
                       terminal_lb, terminal_ub);

    /* A small persistent violation on e_y leaves a non-zero dual to test.
     * A separate controllable state starts with a displaced warm-start split
     * and supplies the larger dual residual that requests a rho decrease. */
    for (int k = 0; k < TEST_HORIZON; k++) {
        step_data[k].A[IDX_EY][IDX_EY] = 0.0f;
        step_data[k].d[IDX_EY] = 1.01f;
        step_data[k].B[2][0] = 1.0f;
    }
    step_data[1].x_lb[IDX_EY] = -1.0f;
    step_data[1].x_ub[IDX_EY] = 1.0f;
    step_data[1].x_lb[2] = -1.0f;
    step_data[1].x_ub[2] = 1.0f;
    terminal_lb[IDX_EY] = -1.0f;
    terminal_ub[IDX_EY] = 1.0f;
    terminal_lb[2] = -1.0f;
    terminal_ub[2] = 1.0f;

    riccati_admm_state_init(&adaptive_state);
    adaptive_state.initialized = 1;
    adaptive_state.rho = 4.0f;
    adaptive_state.rho_u = 1.0f;
    for (int k = 1; k <= TEST_HORIZON; k++) {
        adaptive_state.y_x[k][2] = 5.0f;
    }
    fixed_state = adaptive_state;

    RiccatiAdmmConfig_t adaptive_config = solver_config(4.0f, 1.0f, 1, 0);
    RiccatiAdmmConfig_t fixed_config = solver_config(4.0f, 1.0f, 0, 0);
    (void)riccati_admm_solve(
        step_data, terminal_Q, terminal_q, terminal_lb, terminal_ub, x0,
        NX_AUG, NU, TEST_HORIZON, &adaptive_config,
        &adaptive_state, &adaptive_solution);
    (void)riccati_admm_solve(
        step_data, terminal_Q, terminal_q, terminal_lb, terminal_ub, x0,
        NX_AUG, NU, TEST_HORIZON, &fixed_config,
        &fixed_state, &fixed_solution);

    check_close(adaptive_state.rho, 3.0f, 1e-6f,
                "state rho must retain the 0.75x decrease cadence");
    for (int k = 1; k <= TEST_HORIZON; k++) {
        const float adaptive_lambda =
            adaptive_state.rho * adaptive_state.y_x[k][IDX_EY];
        const float fixed_lambda =
            fixed_state.rho * fixed_state.y_x[k][IDX_EY];
        check_close(adaptive_lambda, fixed_lambda, 2e-5f,
                    "rho decrease must preserve non-zero state lambda=rho*y");
    }
}

static void test_shared_rho_rescales_control_duals_by_old_over_new(void)
{
    RiccatiStepData_t step_data[TEST_HORIZON];
    float terminal_Q[NX_AUG], terminal_q[NX_AUG];
    float terminal_lb[NX_AUG], terminal_ub[NX_AUG];
    float x0[NX_AUG] = {0.0f};
    RiccatiAdmmState_t adaptive_state, fixed_state;
    RiccatiSolution_t adaptive_solution, fixed_solution;

    initialize_problem(step_data, terminal_Q, terminal_q,
                       terminal_lb, terminal_ub);
    for (int k = 0; k < TEST_HORIZON; k++) {
        step_data[k].u_lb[0] = 1.0f;
        step_data[k].u_ub[0] = 1.0f;
    }

    riccati_admm_state_init(&adaptive_state);
    adaptive_state.initialized = 1;
    adaptive_state.rho = 4.0f;
    adaptive_state.rho_u = 4.0f;
    fixed_state = adaptive_state;

    RiccatiAdmmConfig_t adaptive_config = solver_config(4.0f, 4.0f, 1, 1);
    RiccatiAdmmConfig_t fixed_config = solver_config(4.0f, 4.0f, 0, 1);

    (void)riccati_admm_solve(
        step_data, terminal_Q, terminal_q, terminal_lb, terminal_ub, x0,
        NX_AUG, NU, TEST_HORIZON, &adaptive_config,
        &adaptive_state, &adaptive_solution);
    (void)riccati_admm_solve(
        step_data, terminal_Q, terminal_q, terminal_lb, terminal_ub, x0,
        NX_AUG, NU, TEST_HORIZON, &fixed_config,
        &fixed_state, &fixed_solution);

    check_close(adaptive_state.rho, 5.0f, 1e-6f,
                "shared rho must retain the 1.25x adaptation step");
    check_close(adaptive_state.rho_u, adaptive_state.rho, 1e-6f,
                "shared control rho must follow state rho");
    for (int k = 0; k < TEST_HORIZON; k++) {
        const float adaptive_lambda =
            adaptive_state.rho_u * adaptive_state.y_u[k][0];
        const float fixed_lambda = fixed_state.rho_u * fixed_state.y_u[k][0];
        check_close(adaptive_lambda, fixed_lambda, 2e-5f,
                    "shared rho adaptation must preserve control lambda=rho*y");
    }
}

static void test_switch_to_shared_rho_preserves_control_duals(void)
{
    RiccatiStepData_t step_data[TEST_HORIZON];
    float terminal_Q[NX_AUG], terminal_q[NX_AUG];
    float terminal_lb[NX_AUG], terminal_ub[NX_AUG];
    float x0[NX_AUG] = {0.0f};
    RiccatiAdmmState_t switched_state, normalized_state;
    RiccatiSolution_t switched_solution, normalized_solution;

    initialize_problem(step_data, terminal_Q, terminal_q,
                       terminal_lb, terminal_ub);
    riccati_admm_state_init(&switched_state);
    switched_state.initialized = 1;
    switched_state.rho = 4.0f;
    switched_state.rho_u = 8.0f;
    for (int k = 0; k < TEST_HORIZON; k++) {
        switched_state.y_u[k][0] = 0.5f;
        switched_state.y_u[k][1] = -0.25f;
    }

    normalized_state = switched_state;
    normalized_state.rho_u = normalized_state.rho;
    for (int k = 0; k < TEST_HORIZON; k++) {
        for (int a = 0; a < NU; a++) {
            normalized_state.y_u[k][a] *= 2.0f;
        }
    }

    RiccatiAdmmConfig_t config = solver_config(4.0f, 8.0f, 0, 1);
    (void)riccati_admm_solve(
        step_data, terminal_Q, terminal_q, terminal_lb, terminal_ub, x0,
        NX_AUG, NU, TEST_HORIZON, &config,
        &switched_state, &switched_solution);
    (void)riccati_admm_solve(
        step_data, terminal_Q, terminal_q, terminal_lb, terminal_ub, x0,
        NX_AUG, NU, TEST_HORIZON, &config,
        &normalized_state, &normalized_solution);

    check_close(switched_state.rho_u, switched_state.rho, 1e-7f,
                "switching to shared rho must align the penalties");
    for (int k = 0; k < TEST_HORIZON; k++) {
        for (int a = 0; a < NU; a++) {
            check_close(switched_state.y_u[k][a],
                        normalized_state.y_u[k][a], 2e-6f,
                        "switching to shared rho must preserve control lambda");
        }
    }
}

static void test_independent_control_rho_clamp_rescaling(void)
{
    RiccatiStepData_t step_data[TEST_HORIZON];
    float terminal_Q[NX_AUG], terminal_q[NX_AUG];
    float terminal_lb[NX_AUG], terminal_ub[NX_AUG];
    float x0[NX_AUG] = {0.0f};
    RiccatiAdmmState_t adaptive_state, fixed_state;
    RiccatiSolution_t adaptive_solution, fixed_solution;

    initialize_problem(step_data, terminal_Q, terminal_q,
                       terminal_lb, terminal_ub);
    for (int k = 0; k < TEST_HORIZON; k++) {
        step_data[k].u_lb[0] = 1.0f;
        step_data[k].u_ub[0] = 1.0f;
    }

    riccati_admm_state_init(&adaptive_state);
    adaptive_state.initialized = 1;
    adaptive_state.rho = 4.0f;
    adaptive_state.rho_u = 126.0f;
    fixed_state = adaptive_state;

    RiccatiAdmmConfig_t adaptive_config =
        solver_config(4.0f, 126.0f, 1, 0);
    RiccatiAdmmConfig_t fixed_config =
        solver_config(4.0f, 126.0f, 0, 0);
    (void)riccati_admm_solve(
        step_data, terminal_Q, terminal_q, terminal_lb, terminal_ub, x0,
        NX_AUG, NU, TEST_HORIZON, &adaptive_config,
        &adaptive_state, &adaptive_solution);
    (void)riccati_admm_solve(
        step_data, terminal_Q, terminal_q, terminal_lb, terminal_ub, x0,
        NX_AUG, NU, TEST_HORIZON, &fixed_config,
        &fixed_state, &fixed_solution);

    check_close(adaptive_state.rho_u, 127.0f, 1e-6f,
                "independent control rho must clamp at its upper bound");
    for (int k = 0; k < TEST_HORIZON; k++) {
        const float adaptive_lambda =
            adaptive_state.rho_u * adaptive_state.y_u[k][0];
        const float fixed_lambda = fixed_state.rho_u * fixed_state.y_u[k][0];
        check_close(adaptive_lambda, fixed_lambda, 2e-5f,
                    "clamped control rho update must preserve lambda=rho*y");
    }
}

static void test_trace_capture_is_explicit(void)
{
    RiccatiStepData_t step_data[TEST_HORIZON];
    float terminal_Q[NX_AUG], terminal_q[NX_AUG];
    float terminal_lb[NX_AUG], terminal_ub[NX_AUG];
    float x0[NX_AUG] = {0.0f};
    RiccatiAdmmState_t state;
    RiccatiSolution_t solution;
    RiccatiAdmmConfig_t config = solver_config(4.0f, 4.0f, 0, 0);

    initialize_problem(step_data, terminal_Q, terminal_q,
                       terminal_lb, terminal_ub);
    riccati_admm_state_init(&state);
    riccati_debug_set_trace_enabled(0);
    (void)riccati_admm_solve(
        step_data, terminal_Q, terminal_q, terminal_lb, terminal_ub, x0,
        NX_AUG, NU, TEST_HORIZON, &config, &state, &solution);
    check_true(riccati_debug_get_trace_count() == 0,
               "detailed traces must be disabled by default/on request");

    riccati_admm_state_init(&state);
    riccati_debug_set_trace_enabled(1);
    (void)riccati_admm_solve(
        step_data, terminal_Q, terminal_q, terminal_lb, terminal_ub, x0,
        NX_AUG, NU, TEST_HORIZON, &config, &state, &solution);
    check_true(riccati_debug_get_trace_count() > 0,
               "explicitly enabled detailed traces must remain observable");
    riccati_debug_set_trace_enabled(0);
}

int main(void)
{
    test_nine_state_sparse_pass_matches_dense_reference();
    test_fixed_x0_is_not_an_admm_constraint();
    test_state_rho_rescaling();
    test_state_rho_decrease_rescaling();
    test_shared_rho_rescales_control_duals_by_old_over_new();
    test_switch_to_shared_rho_preserves_control_duals();
    test_independent_control_rho_clamp_rescaling();
    test_trace_capture_is_explicit();

    if (failures != 0) {
        fprintf(stderr, "%d Riccati solver regression(s) failed\n", failures);
        return 1;
    }

    puts("Riccati solver regressions passed");
    return 0;
}
