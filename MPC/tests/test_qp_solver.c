/**
 * @file test_qp_solver.c
 * @brief Unit Tests for Quadratic Programming Solver
 *
 * Tests the QP solver with known optimization problems.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "qp_solver.h"
#include "fixed_point.h"

/*===========================================================================
 * Test Infrastructure
 *===========================================================================*/

static int total_tests_passed = 0;
static int total_tests_failed = 0;

#define COLOR_GREEN "\033[32m"
#define COLOR_RED "\033[31m"
#define COLOR_RESET "\033[0m"

#define TEST_ASSERT(condition, message)                                     \
    do                                                                      \
    {                                                                       \
        if (condition)                                                      \
        {                                                                   \
            printf("  [%sPASS%s] %s\n", COLOR_GREEN, COLOR_RESET, message); \
            total_tests_passed++;                                           \
        }                                                                   \
        else                                                                \
        {                                                                   \
            printf("  [%sFAIL%s] %s\n", COLOR_RED, COLOR_RESET, message);   \
            total_tests_failed++;                                           \
        }                                                                   \
    } while (0)

static int values_are_approximately_equal(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) < tolerance;
}

/*===========================================================================
 * Test: Solver Initialization
 *===========================================================================*/

void test_solver_initialization(void)
{
    printf("\n--- Test: Solver Initialization ---\n");

    QuadraticProgramProblem_t problem;
    QuadraticProgramConfig_t config;

    qp_solver_initialize_problem(&problem);
    qp_solver_initialize_config(&config);

    TEST_ASSERT(problem.variable_count == 0, "Problem initialized with 0 variables");
    TEST_ASSERT(problem.constraint_count == 0, "Problem initialized with 0 constraints");
    TEST_ASSERT(config.maximum_iterations == QP_MAXIMUM_ITERATIONS,
                "Config has default max iterations");
    TEST_ASSERT(config.enable_verbose_output == 0, "Verbose output disabled by default");
}

/*===========================================================================
 * Test: Unconstrained Quadratic (Simple Minimum)
 *===========================================================================*/

void test_unconstrained_quadratic(void)
{
    printf("\n--- Test: Unconstrained Quadratic ---\n");

    /*
     * Problem: minimize 0.5 * x^2 + 2*x
     *
     * Solution: x* = -2 (where derivative 0.5*2*x + 2 = 0)
     * But since we start at 0 and have no constraints,
     * gradient descent should move toward negative x.
     *
     * Actually let's do: minimize 0.5 * x^2 - 2*x
     * Solution: x* = 2
     */
    QuadraticProgramProblem_t problem;
    QuadraticProgramConfig_t config;
    QuadraticProgramSolution_t solution;

    qp_solver_initialize_problem(&problem);
    qp_solver_initialize_config(&config);

    /* Single variable */
    problem.variable_count = 1;
    problem.constraint_count = 0;

    /* H = [1] (so 0.5 * x^T * H * x = 0.5 * x^2) */
    problem.hessian_matrix[0] = fixed_point_from_float(1.0f);

    /* f = [-2] (so f^T * x = -2*x) */
    problem.linear_cost_vector[0] = fixed_point_from_float(-2.0f);

    /* Solve */
    QuadraticProgramStatus_t status = qp_solver_solve(&problem, &config, &solution);

    printf("  Solver status: %d\n", status);
    printf("  Iterations: %d\n", solution.iteration_count);
    printf("  Solution x = %.4f\n", fixed_point_to_float(solution.optimal_variables[0]));

    /* With no constraints, solution should approach x* = 2 */
    TEST_ASSERT(status == QP_STATUS_OPTIMAL || status == QP_STATUS_MAXIMUM_ITERATIONS_REACHED,
                "Solver completed successfully");
    TEST_ASSERT(values_are_approximately_equal(
                    fixed_point_to_float(solution.optimal_variables[0]), 2.0f, 0.5f),
                "Solution x ≈ 2.0");
}

/*===========================================================================
 * Test: Box-Constrained Problem
 *===========================================================================*/

void test_box_constrained_problem(void)
{
    printf("\n--- Test: Box-Constrained Problem ---\n");

    /*
     * Problem: minimize 0.5 * x^2 - 10*x
     *          subject to x ≤ 3
     *
     * Unconstrained solution: x* = 10
     * With constraint: x* = 3 (constrained to upper bound)
     */
    QuadraticProgramProblem_t problem;
    QuadraticProgramConfig_t config;
    QuadraticProgramSolution_t solution;

    qp_solver_initialize_problem(&problem);
    qp_solver_initialize_config(&config);

    problem.variable_count = 1;
    problem.constraint_count = 1;

    /* H = [1] */
    problem.hessian_matrix[0] = fixed_point_from_float(1.0f);

    /* f = [-10] */
    problem.linear_cost_vector[0] = fixed_point_from_float(-10.0f);

    /* Constraint: x ≤ 3 (A = [1], b = [3]) */
    problem.constraint_matrix[0] = fixed_point_from_float(1.0f);
    problem.constraint_bounds[0] = fixed_point_from_float(3.0f);

    /* Solve */
    QuadraticProgramStatus_t status = qp_solver_solve(&problem, &config, &solution);

    printf("  Solver status: %d\n", status);
    printf("  Iterations: %d\n", solution.iteration_count);
    printf("  Solution x = %.4f\n", fixed_point_to_float(solution.optimal_variables[0]));
    printf("  Constraint residual: %.4f\n", fixed_point_to_float(solution.constraint_residual));

    /* Solution should be at the constraint boundary */
    TEST_ASSERT(status == QP_STATUS_OPTIMAL || status == QP_STATUS_MAXIMUM_ITERATIONS_REACHED,
                "Solver completed");

    float solution_x = fixed_point_to_float(solution.optimal_variables[0]);
    TEST_ASSERT(solution_x <= 3.1f, "Solution respects constraint x ≤ 3");
}

/*===========================================================================
 * Test: Two-Variable Problem
 *===========================================================================*/

void test_two_variable_problem(void)
{
    printf("\n--- Test: Two-Variable Problem ---\n");

    /*
     * Problem: minimize 0.5 * (x1^2 + x2^2) - x1 - x2
     *          subject to x1 + x2 ≤ 1
     *
     * Unconstrained solution: x1* = x2* = 1
     * With constraint: x1* = x2* = 0.5 (on constraint boundary)
     */
    QuadraticProgramProblem_t problem;
    QuadraticProgramConfig_t config;
    QuadraticProgramSolution_t solution;

    qp_solver_initialize_problem(&problem);
    qp_solver_initialize_config(&config);

    problem.variable_count = 2;
    problem.constraint_count = 1;

    /* H = [[1, 0], [0, 1]] (identity) */
    problem.hessian_matrix[0] = fixed_point_from_float(1.0f); /* H[0,0] */
    problem.hessian_matrix[1] = fixed_point_from_float(0.0f); /* H[0,1] */
    problem.hessian_matrix[2] = fixed_point_from_float(0.0f); /* H[1,0] */
    problem.hessian_matrix[3] = fixed_point_from_float(1.0f); /* H[1,1] */

    /* f = [-1, -1] */
    problem.linear_cost_vector[0] = fixed_point_from_float(-1.0f);
    problem.linear_cost_vector[1] = fixed_point_from_float(-1.0f);

    /* Constraint: x1 + x2 ≤ 1 */
    problem.constraint_matrix[0] = fixed_point_from_float(1.0f); /* A[0,0] */
    problem.constraint_matrix[1] = fixed_point_from_float(1.0f); /* A[0,1] */
    problem.constraint_bounds[0] = fixed_point_from_float(1.0f);

    /* Solve */
    QuadraticProgramStatus_t status = qp_solver_solve(&problem, &config, &solution);

    float x1 = fixed_point_to_float(solution.optimal_variables[0]);
    float x2 = fixed_point_to_float(solution.optimal_variables[1]);

    printf("  Solver status: %d\n", status);
    printf("  Iterations: %d\n", solution.iteration_count);
    printf("  Solution: x1 = %.4f, x2 = %.4f\n", x1, x2);
    printf("  Sum x1 + x2 = %.4f\n", x1 + x2);

    TEST_ASSERT(status == QP_STATUS_OPTIMAL || status == QP_STATUS_MAXIMUM_ITERATIONS_REACHED,
                "Solver completed");
    TEST_ASSERT(x1 + x2 <= 1.1f, "Constraint x1 + x2 ≤ 1 satisfied");
}

/*===========================================================================
 * Test: Zero Problem (Trivial)
 *===========================================================================*/

void test_trivial_problem(void)
{
    printf("\n--- Test: Trivial Problem (Zero Cost) ---\n");

    /*
     * Problem: minimize 0.5 * x^2 (minimum at x = 0)
     */
    QuadraticProgramProblem_t problem;
    QuadraticProgramConfig_t config;
    QuadraticProgramSolution_t solution;

    qp_solver_initialize_problem(&problem);
    qp_solver_initialize_config(&config);

    problem.variable_count = 1;
    problem.constraint_count = 0;

    /* H = [1] */
    problem.hessian_matrix[0] = fixed_point_from_float(1.0f);

    /* f = [0] (no linear term) */
    problem.linear_cost_vector[0] = 0;

    QuadraticProgramStatus_t status = qp_solver_solve(&problem, &config, &solution);

    printf("  Solver status: %d\n", status);
    printf("  Solution x = %.4f\n", fixed_point_to_float(solution.optimal_variables[0]));

    TEST_ASSERT(status == QP_STATUS_OPTIMAL, "Trivial problem solved optimally");
    TEST_ASSERT(values_are_approximately_equal(
                    fixed_point_to_float(solution.optimal_variables[0]), 0.0f, 0.01f),
                "Solution x = 0 (minimum of x^2)");
}

/*===========================================================================
 * Main Test Runner
 *===========================================================================*/

int main(void)
{
    printf("===========================================================\n");
    printf("   QP Solver Unit Tests\n");
    printf("===========================================================\n");

    test_solver_initialization();
    test_unconstrained_quadratic();
    test_box_constrained_problem();
    test_two_variable_problem();
    test_trivial_problem();

    printf("\n===========================================================\n");
    printf("   Results: %d passed, %d failed\n", total_tests_passed, total_tests_failed);
    printf("===========================================================\n");

    return (total_tests_failed > 0) ? 1 : 0;
}
