/**
 * @file test_riccati_solver.c
 * @brief Tests for the Riccati-ADMM solver (non-condensed MPC)
 *
 * Tests:
 * 1. Unconstrained LQR (Riccati only, no ADMM projection needed)
 * 2. Box-constrained LQR (ADMM must activate)
 * 3. Full MPC via mpc_riccati (straight path tracking)
 * 4. Full MPC via mpc_riccati (curve tracking)
 * 5. Wall constraint enforcement
 */

#include "riccati_solver.h"
#include "mpc_types.h"
#include "vehicle_model.h"
#include "fp_math.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* External Riccati MPC API (from mpc_riccati.c) */
extern void mpc_riccati_initialize(void);
extern void mpc_riccati_initialize_with_configuration(const MpcConfiguration_t *cfg);
extern void mpc_riccati_reset(void);
extern MpcSolverStatus_t mpc_riccati_compute_optimal_control(
    const FrenetState_t *current_frenet_state,
    const TrajectoryReferencePoint_t *reference_trajectory,
    MpcSolverResult_t *result);

static int tests_passed = 0;
static int tests_failed = 0;

#define CHECK(cond, msg) do { \
    if (cond) { tests_passed++; } \
    else { tests_failed++; printf("FAIL [line %d]: %s\n", __LINE__, msg); } \
} while(0)

/*===========================================================================
 * Test 1: Unconstrained scalar LQR
 *
 * System: x_{k+1} = 1.0 * x_k + 1.0 * u_k  (integrator)
 * Cost: Q=1, R=1, N=5 steps
 * x0 = 1.0
 *
 * Without constraints, ADMM should converge quickly since the
 * unconstrained solution is feasible.
 *===========================================================================*/

static void test_unconstrained_scalar_lqr(void)
{
    printf("\n=== Test 1: Unconstrained Scalar LQR ===\n");

    int nx = 1, nu = 1, N = 5;

    RiccatiStepData_t step_data[5];
    memset(step_data, 0, sizeof(step_data));

    for (int k = 0; k < N; k++) {
        step_data[k].A[0][0] = FP_ONE;
        step_data[k].B[0][0] = FP_ONE;
        step_data[k].Q_diag[0] = FP_CONST(2.0);  /* 0.5 * x^T * 2 * x = x^2 */
        step_data[k].R_diag[0] = FP_CONST(2.0);  /* 0.5 * u^T * 2 * u = u^2 */
        /* No linear cost (regulation to zero) */
        step_data[k].x_lb[0] = FP_CONST(-100.0);
        step_data[k].x_ub[0] = FP_CONST(100.0);
        step_data[k].u_lb[0] = FP_CONST(-100.0);
        step_data[k].u_ub[0] = FP_CONST(100.0);
    }

    fixed_point_t terminal_Q[RICCATI_MAX_NX] = {FP_CONST(2.0)};
    fixed_point_t terminal_q[RICCATI_MAX_NX] = {0};

    fixed_point_t x0[RICCATI_MAX_NX] = { FP_CONST(1.0) };

    RiccatiAdmmConfig_t cfg;
    riccati_admm_config_init(&cfg);
    cfg.max_iterations = 50;

    RiccatiAdmmState_t admm_state;
    riccati_admm_state_init(&admm_state);

    RiccatiSolution_t sol;
    memset(&sol, 0, sizeof(sol));

    RiccatiStatus_t status = riccati_admm_solve(
        step_data, terminal_Q, terminal_q, x0,
        nx, nu, N, &cfg, &admm_state, &sol);

    printf("  Status: %d, Iterations: %d\n", status, sol.iterations);
    printf("  u[0] = %.4f\n", FP_TO_DOUBLE(sol.u[0][0]));
    printf("  x[1] = %.4f\n", FP_TO_DOUBLE(sol.x[1][0]));
    printf("  x[N] = %.4f\n", FP_TO_DOUBLE(sol.x[N][0]));

    /* The optimal control should drive state toward zero */
    CHECK(FP_TO_DOUBLE(sol.u[0][0]) < 0, "Control should be negative (drive x toward 0)");
    CHECK(fabs(FP_TO_DOUBLE(sol.x[N][0])) < 0.5, "Terminal state should be close to 0");
    CHECK(status == RICCATI_STATUS_OPTIMAL || status == RICCATI_STATUS_MAX_ITERATIONS,
          "Solver should succeed or hit max iterations");
}

/*===========================================================================
 * Test 2: Box-constrained control
 *
 * Same integrator, but |u| <= 0.2.
 * With x0 = 1.0 and tight control bounds, the state can't decrease fast.
 *===========================================================================*/

static void test_box_constrained_control(void)
{
    printf("\n=== Test 2: Box-Constrained Control ===\n");

    int nx = 1, nu = 1, N = 5;

    RiccatiStepData_t step_data[5];
    memset(step_data, 0, sizeof(step_data));

    for (int k = 0; k < N; k++) {
        step_data[k].A[0][0] = FP_ONE;
        step_data[k].B[0][0] = FP_ONE;
        step_data[k].Q_diag[0] = FP_CONST(2.0);
        step_data[k].R_diag[0] = FP_CONST(0.2);  /* Low control cost to encourage saturation */
        step_data[k].x_lb[0] = FP_CONST(-100.0);
        step_data[k].x_ub[0] = FP_CONST(100.0);
        step_data[k].u_lb[0] = FP_CONST(-0.2);  /* Tight control bound */
        step_data[k].u_ub[0] = FP_CONST(0.2);
    }

    fixed_point_t terminal_Q[RICCATI_MAX_NX] = {FP_CONST(2.0)};
    fixed_point_t terminal_q[RICCATI_MAX_NX] = {0};
    fixed_point_t x0[RICCATI_MAX_NX] = { FP_CONST(1.0) };

    RiccatiAdmmConfig_t cfg;
    riccati_admm_config_init(&cfg);
    cfg.max_iterations = 50;

    RiccatiAdmmState_t admm_state;
    riccati_admm_state_init(&admm_state);

    RiccatiSolution_t sol;
    memset(&sol, 0, sizeof(sol));

    riccati_admm_solve(step_data, terminal_Q, terminal_q, x0,
                       nx, nu, N, &cfg, &admm_state, &sol);

    double u0 = FP_TO_DOUBLE(sol.u[0][0]);
    printf("  u[0] = %.4f (expect near -0.2)\n", u0);

    /* Control should be at or near the lower bound (-0.2) */
    CHECK(u0 >= -0.25 && u0 <= 0.0, "u[0] should be near lower bound");

    /* All controls should respect bounds */
    int all_feasible = 1;
    for (int k = 0; k < N; k++) {
        double uk = FP_TO_DOUBLE(sol.u[k][0]);
        if (uk < -0.25 || uk > 0.25) all_feasible = 0;
    }
    CHECK(all_feasible, "All controls should be within bounds (with tolerance)");
}

/*===========================================================================
 * Test 3: Full MPC - Straight path tracking
 *
 * Uses the Riccati-based MPC (mpc_riccati.c) to track a straight path
 * with an initial lateral error.
 *===========================================================================*/

static void test_mpc_straight_path(void)
{
    printf("\n=== Test 3: MPC Straight Path ===\n");

    MpcConfiguration_t cfg = get_default_configuration();
    cfg.prediction_horizon_steps = 10;
    mpc_riccati_initialize_with_configuration(&cfg);

    FrenetState_t state;
    memset(&state, 0, sizeof(state));
    state.lateral_error_meters = FP_CONST(0.3);  /* 30cm left */
    state.longitudinal_velocity_meters_per_second = FP_CONST(3.0);

    TrajectoryReferencePoint_t refs[20];
    memset(refs, 0, sizeof(refs));
    for (int i = 0; i < 20; i++) {
        refs[i].reference_velocity_meters_per_second = FP_CONST(3.0);
        refs[i].left_wall_bound_meters = FP_CONST(5.0);
        refs[i].right_wall_bound_meters = FP_CONST(5.0);
    }

    MpcSolverResult_t result;
    memset(&result, 0, sizeof(result));
    MpcSolverStatus_t status = mpc_riccati_compute_optimal_control(&state, refs, &result);

    double steer = FP_TO_DOUBLE(result.optimal_control.steering_angle_radians);
    printf("  Status: %d, Iterations: %d\n", status, result.iterations_used);
    printf("  Steering: %.4f rad\n", steer);

    CHECK(status != MPC_STATUS_ERROR, "Should not error");
    CHECK(steer < -0.001, "Should steer right (negative) to correct leftward error");
}

/*===========================================================================
 * Test 4: Full MPC - Curve tracking
 *
 * Track a reference path with curvature, no initial error.
 * The controller should produce positive feedforward steering.
 *===========================================================================*/

static void test_mpc_curve(void)
{
    printf("\n=== Test 4: MPC Curve Tracking ===\n");

    MpcConfiguration_t cfg = get_default_configuration();
    cfg.prediction_horizon_steps = 10;
    mpc_riccati_initialize_with_configuration(&cfg);

    FrenetState_t state;
    memset(&state, 0, sizeof(state));
    state.longitudinal_velocity_meters_per_second = FP_CONST(3.0);

    /* Left turn with curvature 1.0 rad/m (1m radius) */
    TrajectoryReferencePoint_t refs[20];
    memset(refs, 0, sizeof(refs));
    for (int i = 0; i < 20; i++) {
        refs[i].reference_velocity_meters_per_second = FP_CONST(3.0);
        refs[i].path_curvature_radians_per_meter = FP_CONST(1.0);
        refs[i].left_wall_bound_meters = FP_CONST(5.0);
        refs[i].right_wall_bound_meters = FP_CONST(5.0);
    }

    MpcSolverResult_t result;
    memset(&result, 0, sizeof(result));
    MpcSolverStatus_t status = mpc_riccati_compute_optimal_control(&state, refs, &result);

    double steer = FP_TO_DOUBLE(result.optimal_control.steering_angle_radians);
    printf("  Status: %d, Iterations: %d\n", status, result.iterations_used);
    printf("  Steering: %.4f rad (expect positive for left curve)\n", steer);

    CHECK(status != MPC_STATUS_ERROR, "Should not error");
    /* For a left turn (positive curvature), steering should be positive */
    CHECK(steer > 0.01, "Should steer left (positive) for left curve");
}

/*===========================================================================
 * Test 5: Wall constraint enforcement
 *
 * Start with lateral error and tight wall bounds.
 * MPC should respect the bounds over all prediction steps.
 *===========================================================================*/

static void test_wall_constraints(void)
{
    printf("\n=== Test 5: Wall Constraints ===\n");

    MpcConfiguration_t cfg = get_default_configuration();
    cfg.prediction_horizon_steps = 10;
    mpc_riccati_initialize_with_configuration(&cfg);

    FrenetState_t state;
    memset(&state, 0, sizeof(state));
    state.lateral_error_meters = FP_CONST(0.2);  /* Near left wall */
    state.longitudinal_velocity_meters_per_second = FP_CONST(3.0);

    /* Tight walls: ±0.3m from centerline */
    TrajectoryReferencePoint_t refs[20];
    memset(refs, 0, sizeof(refs));
    for (int i = 0; i < 20; i++) {
        refs[i].reference_velocity_meters_per_second = FP_CONST(3.0);
        refs[i].left_wall_bound_meters = FP_CONST(0.3);   /* Only 0.1m clearance! */
        refs[i].right_wall_bound_meters = FP_CONST(0.3);
    }

    MpcSolverResult_t result;
    memset(&result, 0, sizeof(result));
    MpcSolverStatus_t status = mpc_riccati_compute_optimal_control(&state, refs, &result);

    double steer = FP_TO_DOUBLE(result.optimal_control.steering_angle_radians);
    printf("  Status: %d, Iterations: %d\n", status, result.iterations_used);
    printf("  Steering: %.4f rad (expect negative to avoid left wall)\n", steer);

    CHECK(status != MPC_STATUS_ERROR, "Should not error");
    CHECK(steer < 0.0, "Should steer right to stay away from left wall");
}

/*===========================================================================
 * Test 6: 2D double integrator (nx=2, nu=1)
 *
 * x1_{k+1} = x1_k + dt * x2_k        (position)
 * x2_{k+1} = x2_k + dt * u_k          (velocity)
 *
 * Regulate to origin from x0 = [1, 0].
 *===========================================================================*/

static void test_double_integrator(void)
{
    printf("\n=== Test 6: Double Integrator (2-state) ===\n");

    int nx = 2, nu = 1, N = 10;
    fixed_point_t dt = FP_CONST(0.1);

    RiccatiStepData_t step_data[10];
    memset(step_data, 0, sizeof(step_data));

    for (int k = 0; k < N; k++) {
        step_data[k].A[0][0] = FP_ONE;       /* x1 = x1 + dt*x2 */
        step_data[k].A[0][1] = dt;
        step_data[k].A[1][1] = FP_ONE;       /* x2 = x2 + dt*u */
        step_data[k].B[1][0] = dt;

        step_data[k].Q_diag[0] = FP_CONST(2.0);  /* Position penalty */
        step_data[k].Q_diag[1] = FP_CONST(0.2);  /* Velocity penalty */
        step_data[k].R_diag[0] = FP_CONST(0.2);  /* Control penalty */

        step_data[k].x_lb[0] = FP_CONST(-100.0);
        step_data[k].x_ub[0] = FP_CONST(100.0);
        step_data[k].x_lb[1] = FP_CONST(-100.0);
        step_data[k].x_ub[1] = FP_CONST(100.0);
        step_data[k].u_lb[0] = FP_CONST(-10.0);
        step_data[k].u_ub[0] = FP_CONST(10.0);
    }

    fixed_point_t terminal_Q[RICCATI_MAX_NX] = {FP_CONST(4.0), FP_CONST(0.4)};
    fixed_point_t terminal_q[RICCATI_MAX_NX] = {0};
    fixed_point_t x0[RICCATI_MAX_NX] = { FP_CONST(1.0), 0 };

    RiccatiAdmmConfig_t cfg;
    riccati_admm_config_init(&cfg);
    cfg.max_iterations = 50;

    RiccatiAdmmState_t admm_state;
    riccati_admm_state_init(&admm_state);

    RiccatiSolution_t sol;
    memset(&sol, 0, sizeof(sol));

    RiccatiStatus_t status = riccati_admm_solve(
        step_data, terminal_Q, terminal_q, x0,
        nx, nu, N, &cfg, &admm_state, &sol);

    printf("  Status: %d, Iterations: %d\n", status, sol.iterations);
    printf("  u[0] = %.4f\n", FP_TO_DOUBLE(sol.u[0][0]));
    printf("  x[N] = [%.4f, %.4f]\n",
           FP_TO_DOUBLE(sol.x[N][0]), FP_TO_DOUBLE(sol.x[N][1]));

    /* Control should be negative (decelerate position) */
    CHECK(FP_TO_DOUBLE(sol.u[0][0]) < 0, "First control should be negative");
    /* Terminal position should be closer to zero */
    CHECK(fabs(FP_TO_DOUBLE(sol.x[N][0])) < 1.0, "Terminal position should approach 0");
}

/*===========================================================================
 * Main
 *===========================================================================*/

int main(void)
{
    printf("=== Riccati-ADMM Solver Tests ===\n");

    test_unconstrained_scalar_lqr();
    test_box_constrained_control();
    test_mpc_straight_path();
    test_mpc_curve();
    test_wall_constraints();
    test_double_integrator();

    printf("\n=== Results: %d passed, %d failed ===\n",
           tests_passed, tests_failed);

    return tests_failed > 0 ? 1 : 0;
}
