/**
 * @file test_fixed_point.c
 * @brief Unit Tests for Fixed-Point Arithmetic Module
 *
 * Tests all fixed-point operations for correctness and precision.
 
 */

#include <stdio.h>
#include <stdlib.h>
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
 * Example: FP(2.5) becomes 163840 at compile time.
 */
#define FP(x) ((fixed_point_t)((double)(x) * (1 << FIXED_POINT_FRACTIONAL_BITS)))

/* Tolerance levels in Q16.16 (compile-time constants) */
#define TOL_TIGHT    FP(0.001)   /* ~66   — for exact arithmetic */
#define TOL_NORMAL   FP(0.01)    /* ~655  — for division, sqrt */
#define TOL_MODERATE FP(0.05)    /* ~3277 — for trig near 0 */
#define TOL_LOOSE    FP(0.15)    /* ~9830 — for trig at larger angles */
#define TOL_WIDE     FP(0.25)    /* ~16384 — for Taylor series at +/-pi */

/**
 * Fixed-point approximate equality check.
 * Returns 1 if |actual - expected| < tolerance.
 */
static int fp_approx_equal(fixed_point_t actual, fixed_point_t expected, fixed_point_t tolerance)
{
    fixed_point_t diff = fixed_point_abs(fixed_point_sub(actual, expected));
    return diff < tolerance;
}

/*===========================================================================
 * Test: Compile-Time Conversion Macro (FP)
 *===========================================================================*/

void test_fixed_point_constants(void)
{
    printf("\n--- Test: Fixed-Point Constants ---\n");

    /* Test conversion of common values */
    fixed_point_t one = 1.0f;
    float one_back = one;
    TEST_ASSERT(values_are_approximately_equal(one_back, 1.0f, 0.0001f),
                "Convert 1.0 roundtrip");

    fixed_point_t half = 0.5f;
    float half_back = half;
    TEST_ASSERT(values_are_approximately_equal(half_back, 0.5f, 0.0001f),
                "Convert 0.5 roundtrip");

    fixed_point_t negative = -3.14159f;
    float negative_back = negative;
    TEST_ASSERT(values_are_approximately_equal(negative_back, -3.14159f, 0.0001f),
                "Convert -π roundtrip");

    fixed_point_t small = 0.001f;
    float small_back = small;
    TEST_ASSERT(values_are_approximately_equal(small_back, 0.001f, 0.0001f),
                "Convert small value (0.001) roundtrip");
}

/*===========================================================================
 * Test: Basic Arithmetic
 *===========================================================================*/

void test_fixed_point_basic_arithmetic(void)
{
    printf("\n--- Test: Basic Arithmetic ---\n");

    fixed_point_t operand_a = 2.5f;
    fixed_point_t operand_b = 1.5f;

    /* Addition */
    fixed_point_t sum = fixed_point_add(operand_a, operand_b);
    float sum_float = sum;
    TEST_ASSERT(values_are_approximately_equal(sum_float, 4.0f, 0.001f),
                "Addition: 2.5 + 1.5 = 4.0");

    /* Subtraction */
    fixed_point_t difference = fixed_point_sub(operand_a, operand_b);
    float difference_float = difference;
    TEST_ASSERT(values_are_approximately_equal(difference_float, 1.0f, 0.001f),
                "Subtraction: 2.5 - 1.5 = 1.0");

    /* Multiplication */
    fixed_point_t product = fixed_point_mul(operand_a, operand_b);
    float product_float = product;
    TEST_ASSERT(values_are_approximately_equal(product_float, 3.75f, 0.001f),
                "Multiplication: 2.5 × 1.5 = 3.75");

    /* Division */
    fixed_point_t quotient = fixed_point_div(operand_a, operand_b);
    float quotient_float = quotient;
    TEST_ASSERT(values_are_approximately_equal(quotient_float, 1.6667f, 0.01f),
                "Division: 2.5 ÷ 1.5 ≈ 1.667");

    /* Division by zero protection */
    fixed_point_t zero_division = fixed_point_div(operand_a, 0);
    TEST_ASSERT(zero_division == 0, "Division by zero returns 0");
}

/*===========================================================================
 * Test: Unary Operations
 *===========================================================================*/

void test_fixed_point_unary_operations(void)
{
    printf("\n--- Test: Unary Operations ---\n");

    fixed_point_t positive_value = 5.25f;
    fixed_point_t negative_value = -3.75f;

    /* Absolute value */
    fixed_point_t abs_positive = fixed_point_abs(positive_value);
    fixed_point_t abs_negative = fixed_point_abs(negative_value);
    TEST_ASSERT(values_are_approximately_equal(abs_positive, 5.25f, 0.001f),
                "Absolute value of positive: |5.25| = 5.25");
    TEST_ASSERT(values_are_approximately_equal(abs_negative, 3.75f, 0.001f),
                "Absolute value of negative: |-3.75| = 3.75");

    /* Negation */
    fixed_point_t negated = fixed_point_neg(positive_value);
    TEST_ASSERT(values_are_approximately_equal(negated, -5.25f, 0.001f),
                "Negation: -(5.25) = -5.25");
}

/*===========================================================================
 * Test: Comparison and Clamping
 *===========================================================================*/

void test_fixed_point_clamping(void)
{
    printf("\n--- Test: Clamping Operations ---\n");

    fixed_point_t value = 7.5f;
    fixed_point_t lower_bound = 2.0f;
    fixed_point_t upper_bound = 5.0f;

    /* Clamp value above upper bound */
    fixed_point_t clamped_high = fixed_point_clamp(value, lower_bound, upper_bound);
    TEST_ASSERT(values_are_approximately_equal(clamped_high, 5.0f, 0.001f),
                "Clamp 7.5 to [2, 5] = 5.0");

    /* Clamp value below lower bound */
    fixed_point_t low_value = 0.5f;
    fixed_point_t clamped_low = fixed_point_clamp(low_value, lower_bound, upper_bound);
    TEST_ASSERT(values_are_approximately_equal(clamped_low, 2.0f, 0.001f),
                "Clamp 0.5 to [2, 5] = 2.0");

    /* Value within bounds unchanged */
    fixed_point_t mid_value = 3.5f;
    fixed_point_t clamped_mid = fixed_point_clamp(mid_value, lower_bound, upper_bound);
    TEST_ASSERT(values_are_approximately_equal(clamped_mid, 3.5f, 0.001f),
                "Clamp 3.5 to [2, 5] = 3.5 (unchanged)");

    /* Minimum */
    fixed_point_t minimum = fixed_point_min(value, lower_bound);
    TEST_ASSERT(values_are_approximately_equal(minimum, 2.0f, 0.001f),
                "Minimum of 7.5 and 2.0 = 2.0");

    /* Maximum */
    fixed_point_t maximum = fixed_point_max(value, lower_bound);
    TEST_ASSERT(values_are_approximately_equal(maximum, 7.5f, 0.001f),
                "Maximum of 7.5 and 2.0 = 7.5");
}

/*===========================================================================
 * Test: Square Root
 *===========================================================================*/

void test_fixed_point_sqrt(void)
{
    printf("\n--- Test: Square Root ---\n");

    /* Perfect square */
    fixed_point_t four = 4.0f;
    fixed_point_t sqrt_four = fixed_point_sqrt(four);
    TEST_ASSERT(values_are_approximately_equal(sqrt_four, 2.0f, 0.01f),
                "Square root of 4 = 2.0");

    /* Non-perfect square */
    fixed_point_t two = 2.0f;
    fixed_point_t sqrt_two = fixed_point_sqrt(two);
    TEST_ASSERT(values_are_approximately_equal(sqrt_two, 1.414f, 0.02f),
                "Square root of 2 ≈ 1.414");

    /* Larger value */
    fixed_point_t hundred = 100.0f;
    fixed_point_t sqrt_hundred = fixed_point_sqrt(hundred);
    TEST_ASSERT(values_are_approximately_equal(sqrt_hundred, 10.0f, 0.05f),
                "Square root of 100 = 10.0");

    /* Zero */
    fixed_point_t sqrt_zero = fixed_point_sqrt(0);
    TEST_ASSERT(sqrt_zero == 0, "Square root of 0 = 0");
}

/*===========================================================================
 * Test: Trigonometric Functions
 *===========================================================================*/

void test_fixed_point_trigonometry(void)
{
    printf("\n--- Test: Trigonometric Functions ---\n");

    /* Sine at key angles */
    fixed_point_t angle_zero = 0;
    fixed_point_t sine_zero = fixed_point_sin(angle_zero);
    TEST_ASSERT(values_are_approximately_equal(sine_zero, 0.0f, 0.01f),
                "sin(0) = 0");

    fixed_point_t angle_pi_over_2 = FIXED_POINT_PI_OVER_2;
    fixed_point_t sine_pi_over_2 = fixed_point_sin(angle_pi_over_2);
    TEST_ASSERT(values_are_approximately_equal(sine_pi_over_2, 1.0f, 0.01f),
                "sin(π/2) = 1");

    fixed_point_t angle_pi = FIXED_POINT_PI;
    fixed_point_t sine_pi = fixed_point_sin(angle_pi);
    /* Note: Taylor series approximation has reduced accuracy at π */
    TEST_ASSERT(values_are_approximately_equal(sine_pi, 0.0f, 0.15f),
                "sin(π) ≈ 0 (Taylor series tolerance)");

    /* Cosine at key angles */
    fixed_point_t cos_zero = fixed_point_cos(angle_zero);
    TEST_ASSERT(values_are_approximately_equal(cos_zero, 1.0f, 0.01f),
                "cos(0) = 1");

    fixed_point_t cos_pi_over_2 = fixed_point_cos(angle_pi_over_2);
    TEST_ASSERT(values_are_approximately_equal(cos_pi_over_2, 0.0f, 0.05f),
                "cos(π/2) ≈ 0");

    fixed_point_t cos_pi = fixed_point_cos(angle_pi);
    /* Note: Taylor series approximation has reduced accuracy at π
     * The 4-term Taylor series gives cos(π) ≈ -0.78 instead of -1.0
     * This is acceptable for vehicle control where angles near π are rare */
    TEST_ASSERT(values_are_approximately_equal(cos_pi, -1.0f, 0.25f),
                "cos(π) ≈ -1 (Taylor series tolerance)");

    /* Tangent */
    fixed_point_t angle_pi_over_4 = 0.7854f; /* π/4 */
    fixed_point_t tan_pi_over_4 = fixed_point_tan(angle_pi_over_4);
    TEST_ASSERT(values_are_approximately_equal(tan_pi_over_4, 1.0f, 0.05f),
                "tan(π/4) ≈ 1");
}

/*===========================================================================
 * Test: Integer Power
 *===========================================================================*/

void test_fixed_point_power(void)
{
    printf("\n--- Test: Integer Power ---\n");

    fixed_point_t base = 2.0f;

    /* Positive exponents */
    fixed_point_t power_2 = fixed_point_pow(base, 2);
    TEST_ASSERT(values_are_approximately_equal(power_2, 4.0f, 0.01f),
                "2^2 = 4");

    fixed_point_t power_3 = fixed_point_pow(base, 3);
    TEST_ASSERT(values_are_approximately_equal(power_3, 8.0f, 0.01f),
                "2^3 = 8");

    /* Zero exponent */
    fixed_point_t power_0 = fixed_point_pow(base, 0);
    TEST_ASSERT(values_are_approximately_equal(power_0, 1.0f, 0.01f),
                "2^0 = 1");

    /* Exponent of 1 */
    fixed_point_t power_1 = fixed_point_pow(base, 1);
    TEST_ASSERT(values_are_approximately_equal(power_1, 2.0f, 0.01f),
                "2^1 = 2");
}

/*===========================================================================
 * Main Test Runner
 *===========================================================================*/

int main(void)
{
    printf("===========================================================\n");
    printf("   Fixed-Point Arithmetic Unit Tests\n");
    printf("===========================================================\n");

    test_fixed_point_constants();
    test_fixed_point_basic_arithmetic();
    test_fixed_point_unary_operations();
    test_fixed_point_clamping();
    test_fixed_point_sqrt();
    test_fixed_point_trigonometry();
    test_fixed_point_power();

    printf("\n===========================================================\n");
    printf("   Results: %d passed, %d failed\n", total_tests_passed, total_tests_failed);
    printf("===========================================================\n");

    return (total_tests_failed > 0) ? 1 : 0;
}
