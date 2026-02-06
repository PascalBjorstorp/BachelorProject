/**
 * @file test_qp_solver.c
 * @brief Unit Tests for Quadratic Programming Solver
 *
 * Tests the QP solver with known optimization problems.
 * No floating-point operations at runtime — all reference values
 * are compile-time constants via the FP() macro.
 */

#include <stdio.h>
#include <stdlib.h>
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

/**
 * Compile-time conversion from decimal to Q16.16 fixed-point.
 * The compiler folds this to an integer constant — NO runtime float ops.
 */
#define FP(x) ((fixed_point_t)((double)(x) * (1 << FIXED_POINT_FRACTIONAL_BITS)))

/**
 * Fixed-point approximate equality check.
 * Returns 1 if |actual - expected| < tolerance.
 */
static int fp_approx_equal(fixed_point_t actual, fixed_point_t expected, fixed_point_t tolerance)
{
    fixed_point_t diff = fixed_point_abs(fixed_point_sub(actual, expected));
    return diff < tolerance;
}

/* Tolerance levels */
#define TOL_TIGHT    FP(0.01)
#define TOL_NORMAL   FP(0.1)
#define TOL_LOOSE    FP(0.5)
#define TOL_WIDE     FP(1.0)

/**
 * Print a fixed-point value as integer and fractional parts for debugging.
 * Prints as "integer_part.fractional" using only integer arithmetic.
 */
static void print_fp(const char *label, fixed_point_t value)
{
    int32_t integer_part = value >> FIXED_POINT_FRACTIONAL_BITS;
    /* Fractional part: take lower 16 bits, multiply by 10000, shift right 16 */
    uint32_t frac_raw = (uint32_t)(value & 0xFFFF);
    uint32_t frac_display = (frac_raw * 10000) >> FIXED_POINT_FRACTIONAL_BITS;

    if (value < 0 && integer_part == 0)
    {
        printf("  %s = -%d.%04u (raw: %d)\n", label, 0, frac_display, (int)value);
    }
    else
    {
        printf("  %s = %d.%04u (raw: %d)\n", label, (int)integer_part, frac_display, (int)value);
    }
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

    /* Verify config defaults are in fixed-point (not raw floats) */
    TEST_ASSERT(config.gradient_step_size == FIXED_POINT_HALF,
                "Default step size = 0.5 (FIXED_POINT_HALF)");
    TEST_ASSERT(config.convergence_tolerance > 0 && config.convergence_tolerance < FP(0.01),
                "Default convergence tolerance is small positive value");
}

/*===========================================================================
 * Test: Unconstrained Quadratic (Simple Minimum)
 *===========================================================================*/

void test_unconstrained_quadratic(void)
{
    printf("\n--- Test: Unconstrained Quadratic ---\n");

    /*
     * Problem: minimize 0.5 * x^2 - 2*x
     * Solution: x* = 2 (where derivative x - 2 = 0)
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
    problem.hessian_matrix[0] = FP(1.0);

    /* f = [-2] (so f^T * x = -2*x) */
    problem.linear_cost_vector[0] = FP(-2.0);

    /* Solve */
    QuadraticProgramStatus_t status = qp_solver_solve(&problem, &config, &solution);

    printf("  Solver status: %d\n", status);
    printf("  Iterations: %d\n", solution.iteration_count);
    print_fp("Solution x", solution.optimal_variables[0]);

    /* With no constraints, solution should approach x* = 2 */
    TEST_ASSERT(status == QP_STATUS_OPTIMAL || status == QP_STATUS_MAXIMUM_ITERATIONS_REACHED,
                "Solver completed successfully");
    TEST_ASSERT(fp_approx_equal(solution.optimal_variables[0], FP(2.0), TOL_LOOSE),
                "Solution x ~ 2.0");
}

/*===========================================================================
 * Test: Box-Constrained Problem
 *===========================================================================*/

void test_box_constrained_problem(void)
{
    printf("\n--- Test: Box-Constrained Problem ---\n");

    /*
     * Problem: minimize 0.5 * x^2 - 10*x
     *          subject to x <= 3
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
    problem.hessian_matrix[0] = FP(1.0);

    /* f = [-10] */
    problem.linear_cost_vector[0] = FP(-10.0);

    /* Constraint: x <= 3 (A = [1], b = [3]) */
    problem.constraint_matrix[0] = FP(1.0);
    problem.constraint_bounds[0] = FP(3.0);

    /* Solve */
    QuadraticProgramStatus_t status = qp_solver_solve(&problem, &config, &solution);

    printf("  Solver status: %d\n", status);
    printf("  Iterations: %d\n", solution.iteration_count);
    print_fp("Solution x", solution.optimal_variables[0]);
    print_fp("Constraint residual", solution.constraint_residual);

    /* Solution should be at the constraint boundary */
    TEST_ASSERT(status == QP_STATUS_OPTIMAL || status == QP_STATUS_MAXIMUM_ITERATIONS_REACHED,
                "Solver completed");

    /* x should be <= 3 (with some tolerance for projection accuracy) */
    TEST_ASSERT(solution.optimal_variables[0] <= FP(3.0) + TOL_NORMAL,
                "Solution respects constraint x <= 3");
}

/*===========================================================================
 * Test: Two-Variable Problem
 *===========================================================================*/

void test_two_variable_problem(void)
{
    printf("\n--- Test: Two-Variable Problem ---\n");

    /*
     * Problem: minimize 0.5 * (x1^2 + x2^2) - x1 - x2
     *          subject to x1 + x2 <= 1
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
    problem.hessian_matrix[0] = FP(1.0); /* H[0,0] */
    problem.hessian_matrix[1] = FP(0.0); /* H[0,1] */
    problem.hessian_matrix[2] = FP(0.0); /* H[1,0] */
    problem.hessian_matrix[3] = FP(1.0); /* H[1,1] */

    /* f = [-1, -1] */
    problem.linear_cost_vector[0] = FP(-1.0);
    problem.linear_cost_vector[1] = FP(-1.0);

    /* Constraint: x1 + x2 <= 1 */
    problem.constraint_matrix[0] = FP(1.0); /* A[0,0] */
    problem.constraint_matrix[1] = FP(1.0); /* A[0,1] */
    problem.constraint_bounds[0] = FP(1.0);

    /* Solve */
    QuadraticProgramStatus_t status = qp_solver_solve(&problem, &config, &solution);

    fixed_point_t x1 = solution.optimal_variables[0];
    fixed_point_t x2 = solution.optimal_variables[1];
    fixed_point_t sum_x = fixed_point_add(x1, x2);

    printf("  Solver status: %d\n", status);
    printf("  Iterations: %d\n", solution.iteration_count);
    print_fp("x1", x1);
    print_fp("x2", x2);
    print_fp("x1 + x2", sum_x);

    TEST_ASSERT(status == QP_STATUS_OPTIMAL || status == QP_STATUS_MAXIMUM_ITERATIONS_REACHED,
                "Solver completed");
    TEST_ASSERT(sum_x <= FP(1.0) + TOL_NORMAL,
                "Constraint x1 + x2 <= 1 satisfied");
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
    problem.hessian_matrix[0] = FP(1.0);

    /* f = [0] (no linear term) */
    problem.linear_cost_vector[0] = 0;

    QuadraticProgramStatus_t status = qp_solver_solve(&problem, &config, &solution);

    printf("  Solver status: %d\n", status);
    print_fp("Solution x", solution.optimal_variables[0]);

    TEST_ASSERT(status == QP_STATUS_OPTIMAL, "Trivial problem solved optimally");
    TEST_ASSERT(fp_approx_equal(solution.optimal_variables[0], 0, TOL_TIGHT),
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
