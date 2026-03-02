/**
 * @file test_admm_solver.c
 * @brief Tests for the ADMM QP solver (dense, fixed-point)
 *
 * Tests the ADMM solver on:
 * 1. Simple 2D box-constrained QP (analytical solution known)
 * 2. Cholesky factorization correctness
 * 3. Drop-in replacement: full MPC solve (with MPC_SOLVER_ADMM compiled)
 * 4. Warm-start acceleration
 */

#include "mpc.h"
#include "mpc_types.h"
#include "qp_solver.h"
#include "qp_solver_admm.h"
#include "fp_math.h"
#include "vehicle_model.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static int tests_passed = 0;
static int tests_failed = 0;

#define CHECK(cond, msg) do { \
    if (cond) { tests_passed++; } \
    else { tests_failed++; printf("FAIL [line %d]: %s\n", __LINE__, msg); } \
} while(0)

/*===========================================================================
 * Test 1: Simple 2×2 QP
 *
 * min 0.5 x^T H x + f^T x
 * s.t. -1 <= x[i] <= 1
 *
 * H = [[2, 0], [0, 4]]   (diagonal)
 * f = [-1, -2]
 *
 * Unconstrained: x* = -H^{-1} f = [0.5, 0.5]
 * Within bounds → optimal = [0.5, 0.5]
 *===========================================================================*/

static void test_simple_2d_qp(void)
{
    printf("\n=== Test 1: Simple 2D QP ===\n");

    QuadraticProgramProblem_t prob;
    memset(&prob, 0, sizeof(prob));

    prob.variable_count = 2;
    prob.hessian_matrix[0*2 + 0] = FP_CONST(2.0);
    prob.hessian_matrix[0*2 + 1] = 0;
    prob.hessian_matrix[1*2 + 0] = 0;
    prob.hessian_matrix[1*2 + 1] = FP_CONST(4.0);

    prob.linear_cost_vector[0] = FP_CONST(-1.0);
    prob.linear_cost_vector[1] = FP_CONST(-2.0);

    /* Box constraints: -1 <= x <= 1, encoded as identity rows */
    prob.constraint_count = 4;
    /* x[0] <= 1 */
    prob.constraint_matrix[0*2 + 0] = FP_ONE;
    prob.constraint_bounds[0] = FP_CONST(1.0);
    /* -x[0] <= 1 */
    prob.constraint_matrix[1*2 + 0] = -FP_ONE;
    prob.constraint_bounds[1] = FP_CONST(1.0);
    /* x[1] <= 1 */
    prob.constraint_matrix[2*2 + 1] = FP_ONE;
    prob.constraint_bounds[2] = FP_CONST(1.0);
    /* -x[1] <= 1 */
    prob.constraint_matrix[3*2 + 1] = -FP_ONE;
    prob.constraint_bounds[3] = FP_CONST(1.0);

    prob.use_warm_start = 0;

    AdmmConfig_t cfg;
    admm_config_init(&cfg);

    AdmmState_t state;
    admm_state_init(&state);

    QuadraticProgramSolution_t sol;
    memset(&sol, 0, sizeof(sol));

    QuadraticProgramStatus_t status = admm_solve(&prob, &cfg, &state, &sol);

    double x0 = FP_TO_DOUBLE(sol.optimal_variables[0]);
    double x1 = FP_TO_DOUBLE(sol.optimal_variables[1]);

    printf("  Status: %d, Iterations: %d\n", status, sol.iteration_count);
    printf("  x[0] = %.4f (expect 0.5)\n", x0);
    printf("  x[1] = %.4f (expect 0.5)\n", x1);

    CHECK(status == QP_STATUS_OPTIMAL, "Should converge to optimal");
    CHECK(fabs(x0 - 0.5) < 0.05, "x[0] should be ~0.5");
    CHECK(fabs(x1 - 0.5) < 0.05, "x[1] should be ~0.5");
}

/*===========================================================================
 * Test 2: Active constraint
 *
 * H = [[2, 0], [0, 2]]
 * f = [-4, -4]
 *
 * Unconstrained: x* = [2, 2]
 * With bounds -1 <= x <= 1: optimal = [1, 1] (both at upper bound)
 *===========================================================================*/

static void test_active_constraint(void)
{
    printf("\n=== Test 2: Active Constraint ===\n");

    QuadraticProgramProblem_t prob;
    memset(&prob, 0, sizeof(prob));

    prob.variable_count = 2;
    prob.hessian_matrix[0*2 + 0] = FP_CONST(2.0);
    prob.hessian_matrix[1*2 + 1] = FP_CONST(2.0);

    prob.linear_cost_vector[0] = FP_CONST(-4.0);
    prob.linear_cost_vector[1] = FP_CONST(-4.0);

    prob.constraint_count = 4;
    prob.constraint_matrix[0*2 + 0] = FP_ONE;
    prob.constraint_bounds[0] = FP_CONST(1.0);
    prob.constraint_matrix[1*2 + 0] = -FP_ONE;
    prob.constraint_bounds[1] = FP_CONST(1.0);
    prob.constraint_matrix[2*2 + 1] = FP_ONE;
    prob.constraint_bounds[2] = FP_CONST(1.0);
    prob.constraint_matrix[3*2 + 1] = -FP_ONE;
    prob.constraint_bounds[3] = FP_CONST(1.0);

    prob.use_warm_start = 0;

    AdmmConfig_t cfg;
    admm_config_init(&cfg);

    AdmmState_t state;
    admm_state_init(&state);

    QuadraticProgramSolution_t sol;
    memset(&sol, 0, sizeof(sol));

    QuadraticProgramStatus_t status = admm_solve(&prob, &cfg, &state, &sol);

    double x0 = FP_TO_DOUBLE(sol.optimal_variables[0]);
    double x1 = FP_TO_DOUBLE(sol.optimal_variables[1]);

    printf("  Status: %d, Iterations: %d\n", status, sol.iteration_count);
    printf("  x[0] = %.4f (expect 1.0)\n", x0);
    printf("  x[1] = %.4f (expect 1.0)\n", x1);

    CHECK(fabs(x0 - 1.0) < 0.05, "x[0] should be ~1.0 (active bound)");
    CHECK(fabs(x1 - 1.0) < 0.05, "x[1] should be ~1.0 (active bound)");
}

/*===========================================================================
 * Test 3: Cholesky factorization
 *
 * Factor H + rho*I and verify L*L^T = H + rho*I
 *===========================================================================*/

static void test_cholesky(void)
{
    printf("\n=== Test 3: Cholesky Factorization ===\n");

    fixed_point_t H[4] = {
        FP_CONST(4.0), FP_CONST(1.0),
        FP_CONST(1.0), FP_CONST(3.0)
    };
    fixed_point_t rho = FP_CONST(1.0);
    fixed_point_t L[4];

    int ret = admm_cholesky_factorize(H, rho, 2, L);
    CHECK(ret == 0, "Cholesky should succeed");

    /* Verify L*L^T ≈ H + rho*I */
    /* Expected: H + I = [[5, 1], [1, 4]] */
    double LLt00 = FP_TO_DOUBLE(L[0]) * FP_TO_DOUBLE(L[0]);
    double LLt01 = FP_TO_DOUBLE(L[2]) * FP_TO_DOUBLE(L[0]);
    double LLt11 = FP_TO_DOUBLE(L[2]) * FP_TO_DOUBLE(L[2])
                 + FP_TO_DOUBLE(L[3]) * FP_TO_DOUBLE(L[3]);

    printf("  L*L^T[0][0] = %.3f (expect 5.0)\n", LLt00);
    printf("  L*L^T[0][1] = %.3f (expect 1.0)\n", LLt01);
    printf("  L*L^T[1][1] = %.3f (expect 4.0)\n", LLt11);

    CHECK(fabs(LLt00 - 5.0) < 0.1, "LLt[0][0] should be ~5.0");
    CHECK(fabs(LLt01 - 1.0) < 0.1, "LLt[0][1] should be ~1.0");
    CHECK(fabs(LLt11 - 4.0) < 0.1, "LLt[1][1] should be ~4.0");
}

/*===========================================================================
 * Test 4: Cholesky solve
 *
 * Solve (H + rho*I) x = b and verify.
 *===========================================================================*/

static void test_cholesky_solve(void)
{
    printf("\n=== Test 4: Cholesky Solve ===\n");

    fixed_point_t H[4] = {
        FP_CONST(4.0), FP_CONST(1.0),
        FP_CONST(1.0), FP_CONST(3.0)
    };
    fixed_point_t rho = FP_CONST(1.0);
    fixed_point_t L[4];

    admm_cholesky_factorize(H, rho, 2, L);

    /* Solve (H+I) x = b where b = [11, 7] */
    /* (H+I) = [[5,1],[1,4]], so x = [2, 1.25] */
    /* Check: 5*2 + 1*1.25 = 11.25 ≈ 11, 1*2 + 4*1.25 = 7 ✓ */
    /* Actually let me compute properly:
     * [[5,1],[1,4]] x = [11, 7]
     * x[0] = (11*4 - 7*1) / (5*4 - 1*1) = 37/19 ≈ 1.947
     * x[1] = (7*5 - 11*1) / 19 = 24/19 ≈ 1.263
     */
    fixed_point_t b[2] = { FP_CONST(11.0), FP_CONST(7.0) };
    fixed_point_t x[2];

    admm_cholesky_solve(L, b, 2, x);

    double x0 = FP_TO_DOUBLE(x[0]);
    double x1 = FP_TO_DOUBLE(x[1]);

    printf("  x[0] = %.4f (expect 1.947)\n", x0);
    printf("  x[1] = %.4f (expect 1.263)\n", x1);

    CHECK(fabs(x0 - 1.947) < 0.1, "x[0] should be ~1.947");
    CHECK(fabs(x1 - 1.263) < 0.1, "x[1] should be ~1.263");
}

/*===========================================================================
 * Test 5: Full MPC with ADMM (compile with -DMPC_SOLVER_ADMM)
 *
 * Same scenario as the basic accuracy test: straight path, small
 * lateral error. Verify the controller steers back to center.
 *===========================================================================*/

static void test_mpc_admm_basic(void)
{
    printf("\n=== Test 5: Full MPC with ADMM ===\n");

    /* Initialize MPC (this will use ADMM if compiled with MPC_SOLVER_ADMM) */
    MpcConfiguration_t cfg = get_default_configuration();
    cfg.prediction_horizon_steps = 10;
    mpc_initialize_with_configuration(&cfg);

    /* Straight path with small lateral error */
    FrenetState_t state;
    memset(&state, 0, sizeof(state));
    state.lateral_error_meters = FP_CONST(0.3);  /* 30cm to the left */
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
    MpcSolverStatus_t status = mpc_compute_optimal_control(&state, refs, &result);

    double steer = FP_TO_DOUBLE(result.optimal_control.steering_angle_radians);
    printf("  MPC status: %d, iterations: %d\n", status, result.iterations_used);
    printf("  Steering: %.4f rad (expect negative to correct left error)\n", steer);

    CHECK(status == MPC_STATUS_SUCCESS || status == MPC_STATUS_MAXIMUM_ITERATIONS_REACHED,
          "MPC should produce a result");
    CHECK(steer < -0.001, "Should steer right (negative) to correct leftward error");
}

/*===========================================================================
 * Test 6: Warm-starting acceleration
 *
 * Call ADMM twice with same problem; second call should take fewer iterations.
 *===========================================================================*/

static void test_warm_start(void)
{
    printf("\n=== Test 6: Warm-start ===\n");

    QuadraticProgramProblem_t prob;
    memset(&prob, 0, sizeof(prob));

    prob.variable_count = 2;
    prob.hessian_matrix[0*2 + 0] = FP_CONST(2.0);
    prob.hessian_matrix[1*2 + 1] = FP_CONST(2.0);
    prob.linear_cost_vector[0] = FP_CONST(-1.0);
    prob.linear_cost_vector[1] = FP_CONST(-1.0);

    prob.constraint_count = 4;
    prob.constraint_matrix[0*2 + 0] = FP_ONE;
    prob.constraint_bounds[0] = FP_CONST(2.0);
    prob.constraint_matrix[1*2 + 0] = -FP_ONE;
    prob.constraint_bounds[1] = FP_CONST(2.0);
    prob.constraint_matrix[2*2 + 1] = FP_ONE;
    prob.constraint_bounds[2] = FP_CONST(2.0);
    prob.constraint_matrix[3*2 + 1] = -FP_ONE;
    prob.constraint_bounds[3] = FP_CONST(2.0);

    prob.use_warm_start = 0;

    AdmmConfig_t cfg;
    admm_config_init(&cfg);
    cfg.max_iterations = 100;

    AdmmState_t state;
    admm_state_init(&state);

    QuadraticProgramSolution_t sol;

    /* First solve: cold start */
    memset(&sol, 0, sizeof(sol));
    admm_solve(&prob, &cfg, &state, &sol);
    int iter1 = sol.iteration_count;

    /* Second solve: warm start (use previous z, u) */
    prob.use_warm_start = 1;
    memcpy(prob.initial_point, sol.optimal_variables, sizeof(prob.initial_point));
    memset(&sol, 0, sizeof(sol));
    admm_solve(&prob, &cfg, &state, &sol);
    int iter2 = sol.iteration_count;

    printf("  Cold start iterations: %d\n", iter1);
    printf("  Warm start iterations: %d\n", iter2);
    CHECK(iter2 <= iter1, "Warm start should take ≤ iterations than cold start");
}

/*===========================================================================
 * Main
 *===========================================================================*/

int main(void)
{
    printf("=== ADMM QP Solver Tests ===\n");

    test_simple_2d_qp();
    test_active_constraint();
    test_cholesky();
    test_cholesky_solve();
    test_mpc_admm_basic();
    test_warm_start();

    printf("\n=== Results: %d passed, %d failed ===\n",
           tests_passed, tests_failed);

    return tests_failed > 0 ? 1 : 0;
}
