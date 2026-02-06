/**
 * @file test_linear_algebra.c
 * @brief Unit Tests for Linear Algebra Module
 *
 * Tests matrix and vector operations for correctness.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "linear_algebra.h"
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
 * Test: Matrix-Vector Multiplication
 *===========================================================================*/

void test_matrix_vector_multiply(void)
{
    printf("\n--- Test: Matrix-Vector Multiplication ---\n");

    /* Test: 2×3 matrix × 3×1 vector
     * | 1  2  3 |   | 1 |   | 14 |
     * | 4  5  6 | × | 2 | = | 32 |
     *               | 3 |
     */
    fixed_point_t matrix[6] = {
        fixed_point_from_float(1.0f), fixed_point_from_float(2.0f), fixed_point_from_float(3.0f),
        fixed_point_from_float(4.0f), fixed_point_from_float(5.0f), fixed_point_from_float(6.0f)};

    fixed_point_t input_vector[3] = {
        fixed_point_from_float(1.0f),
        fixed_point_from_float(2.0f),
        fixed_point_from_float(3.0f)};

    fixed_point_t result_vector[2];

    linear_algebra_matrix_vector_multiply(matrix, input_vector, result_vector, 2, 3);

    float result_0 = fixed_point_to_float(result_vector[0]);
    float result_1 = fixed_point_to_float(result_vector[1]);

    TEST_ASSERT(values_are_approximately_equal(result_0, 14.0f, 0.1f),
                "Matrix-vector product element [0] = 14");
    TEST_ASSERT(values_are_approximately_equal(result_1, 32.0f, 0.1f),
                "Matrix-vector product element [1] = 32");
}

/*===========================================================================
 * Test: Dot Product
 *===========================================================================*/

void test_dot_product(void)
{
    printf("\n--- Test: Dot Product ---\n");

    /* [1, 2, 3] · [4, 5, 6] = 4 + 10 + 18 = 32 */
    fixed_point_t vector_a[3] = {
        fixed_point_from_float(1.0f),
        fixed_point_from_float(2.0f),
        fixed_point_from_float(3.0f)};

    fixed_point_t vector_b[3] = {
        fixed_point_from_float(4.0f),
        fixed_point_from_float(5.0f),
        fixed_point_from_float(6.0f)};

    fixed_point_t dot_result = linear_algebra_dot_product(vector_a, vector_b, 3);
    float dot_float = fixed_point_to_float(dot_result);

    TEST_ASSERT(values_are_approximately_equal(dot_float, 32.0f, 0.1f),
                "Dot product [1,2,3]·[4,5,6] = 32");
}

/*===========================================================================
 * Test: Vector Norm
 *===========================================================================*/

void test_vector_norm(void)
{
    printf("\n--- Test: Vector Norm ---\n");

    /* ||[3, 4]|| = √(9 + 16) = √25 = 5 */
    fixed_point_t vector[2] = {
        fixed_point_from_float(3.0f),
        fixed_point_from_float(4.0f)};

    fixed_point_t norm = linear_algebra_vector_norm(vector, 2);
    float norm_float = fixed_point_to_float(norm);

    TEST_ASSERT(values_are_approximately_equal(norm_float, 5.0f, 0.1f),
                "Norm of [3,4] = 5");

    /* ||[1, 1, 1]|| = √3 ≈ 1.732 */
    fixed_point_t unit_vector[3] = {
        fixed_point_from_float(1.0f),
        fixed_point_from_float(1.0f),
        fixed_point_from_float(1.0f)};

    fixed_point_t unit_norm = linear_algebra_vector_norm(unit_vector, 3);
    float unit_norm_float = fixed_point_to_float(unit_norm);

    TEST_ASSERT(values_are_approximately_equal(unit_norm_float, 1.732f, 0.05f),
                "Norm of [1,1,1] ≈ 1.732");
}

/*===========================================================================
 * Test: Vector Scaling
 *===========================================================================*/

void test_vector_scale(void)
{
    printf("\n--- Test: Vector Scaling ---\n");

    fixed_point_t input_vector[3] = {
        fixed_point_from_float(1.0f),
        fixed_point_from_float(2.0f),
        fixed_point_from_float(3.0f)};

    fixed_point_t scalar = fixed_point_from_float(2.5f);
    fixed_point_t result_vector[3];

    linear_algebra_vector_scale(input_vector, scalar, result_vector, 3);

    TEST_ASSERT(values_are_approximately_equal(fixed_point_to_float(result_vector[0]), 2.5f, 0.01f),
                "Scale [1,2,3] by 2.5: element [0] = 2.5");
    TEST_ASSERT(values_are_approximately_equal(fixed_point_to_float(result_vector[1]), 5.0f, 0.01f),
                "Scale [1,2,3] by 2.5: element [1] = 5.0");
    TEST_ASSERT(values_are_approximately_equal(fixed_point_to_float(result_vector[2]), 7.5f, 0.01f),
                "Scale [1,2,3] by 2.5: element [2] = 7.5");
}

/*===========================================================================
 * Test: Vector Add Scaled (AXPY)
 *===========================================================================*/

void test_vector_add_scaled(void)
{
    printf("\n--- Test: Vector Add Scaled (AXPY) ---\n");

    /* result = a + 2 × b = [1,2,3] + 2×[1,1,1] = [3,4,5] */
    fixed_point_t vector_a[3] = {
        fixed_point_from_float(1.0f),
        fixed_point_from_float(2.0f),
        fixed_point_from_float(3.0f)};

    fixed_point_t vector_b[3] = {
        fixed_point_from_float(1.0f),
        fixed_point_from_float(1.0f),
        fixed_point_from_float(1.0f)};

    fixed_point_t scalar = fixed_point_from_float(2.0f);
    fixed_point_t result_vector[3];

    linear_algebra_vector_add_scaled(vector_a, vector_b, scalar, result_vector, 3);

    TEST_ASSERT(values_are_approximately_equal(fixed_point_to_float(result_vector[0]), 3.0f, 0.01f),
                "AXPY result [0] = 3.0");
    TEST_ASSERT(values_are_approximately_equal(fixed_point_to_float(result_vector[1]), 4.0f, 0.01f),
                "AXPY result [1] = 4.0");
    TEST_ASSERT(values_are_approximately_equal(fixed_point_to_float(result_vector[2]), 5.0f, 0.01f),
                "AXPY result [2] = 5.0");
}

/*===========================================================================
 * Test: Constraint Violation
 *===========================================================================*/

void test_constraint_violation(void)
{
    printf("\n--- Test: Constraint Violation ---\n");

    /* Constraint: x[0] + x[1] ≤ 3
     * Test with x = [1, 1] → 2 ≤ 3 (satisfied, violation = 0)
     * Test with x = [2, 2] → 4 ≤ 3 (violated by 1)
     */
    fixed_point_t constraint_matrix[2] = {
        fixed_point_from_float(1.0f),
        fixed_point_from_float(1.0f)};

    fixed_point_t bound = fixed_point_from_float(3.0f);

    /* Feasible case */
    fixed_point_t feasible_x[2] = {
        fixed_point_from_float(1.0f),
        fixed_point_from_float(1.0f)};

    fixed_point_t violation_feasible = linear_algebra_max_constraint_violation(
        constraint_matrix, feasible_x, &bound, 1, 2);

    TEST_ASSERT(fixed_point_to_float(violation_feasible) < 0.01f,
                "Feasible point has zero violation");

    /* Infeasible case */
    fixed_point_t infeasible_x[2] = {
        fixed_point_from_float(2.0f),
        fixed_point_from_float(2.0f)};

    fixed_point_t violation_infeasible = linear_algebra_max_constraint_violation(
        constraint_matrix, infeasible_x, &bound, 1, 2);

    TEST_ASSERT(values_are_approximately_equal(fixed_point_to_float(violation_infeasible), 1.0f, 0.1f),
                "Infeasible point has violation ≈ 1.0");
}

/*===========================================================================
 * Test: Vector Clamping
 *===========================================================================*/

void test_vector_clamping(void)
{
    printf("\n--- Test: Vector Clamping ---\n");

    fixed_point_t input_vector[4] = {
        fixed_point_from_float(-5.0f), /* Below lower bound */
        fixed_point_from_float(2.5f),  /* Within bounds */
        fixed_point_from_float(10.0f), /* Above upper bound */
        fixed_point_from_float(0.0f)   /* At lower bound */
    };

    fixed_point_t lower = fixed_point_from_float(0.0f);
    fixed_point_t upper = fixed_point_from_float(5.0f);
    fixed_point_t result_vector[4];

    linear_algebra_clamp_vector_scalar(input_vector, lower, upper, result_vector, 4);

    TEST_ASSERT(values_are_approximately_equal(fixed_point_to_float(result_vector[0]), 0.0f, 0.01f),
                "Clamp -5 to [0,5] = 0");
    TEST_ASSERT(values_are_approximately_equal(fixed_point_to_float(result_vector[1]), 2.5f, 0.01f),
                "Clamp 2.5 to [0,5] = 2.5 (unchanged)");
    TEST_ASSERT(values_are_approximately_equal(fixed_point_to_float(result_vector[2]), 5.0f, 0.01f),
                "Clamp 10 to [0,5] = 5");
    TEST_ASSERT(values_are_approximately_equal(fixed_point_to_float(result_vector[3]), 0.0f, 0.01f),
                "Clamp 0 to [0,5] = 0 (at bound)");
}

/*===========================================================================
 * Main Test Runner
 *===========================================================================*/

int main(void)
{
    printf("===========================================================\n");
    printf("   Linear Algebra Unit Tests\n");
    printf("===========================================================\n");

    test_matrix_vector_multiply();
    test_dot_product();
    test_vector_norm();
    test_vector_scale();
    test_vector_add_scaled();
    test_constraint_violation();
    test_vector_clamping();

    printf("\n===========================================================\n");
    printf("   Results: %d passed, %d failed\n", total_tests_passed, total_tests_failed);
    printf("===========================================================\n");

    return (total_tests_failed > 0) ? 1 : 0;
}
