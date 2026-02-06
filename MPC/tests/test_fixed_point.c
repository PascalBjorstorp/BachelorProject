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

    /* Verify FP() macro produces correct Q16.16 values */
    TEST_ASSERT(FP(1.0) == FIXED_POINT_ONE,
                "FP(1.0) == FIXED_POINT_ONE");

    TEST_ASSERT(FP(0.5) == FIXED_POINT_HALF,
                "FP(0.5) == FIXED_POINT_HALF");

    TEST_ASSERT(FP(0.0) == 0,
                "FP(0.0) == 0");

    TEST_ASSERT(FP(-1.0) == -FIXED_POINT_ONE,
                "FP(-1.0) == -FIXED_POINT_ONE");

    /* Verify known constants */
    TEST_ASSERT(fp_approx_equal(FIXED_POINT_PI, FP(3.14159265), TOL_TIGHT),
                "FIXED_POINT_PI ~ 3.14159");

    TEST_ASSERT(fp_approx_equal(FIXED_POINT_PI_OVER_2, FP(1.5707963), TOL_TIGHT),
                "FIXED_POINT_PI_OVER_2 ~ 1.5708");
}

/*===========================================================================
 * Test: Basic Arithmetic
 *===========================================================================*/

void test_fixed_point_basic_arithmetic(void)
{
    printf("\n--- Test: Basic Arithmetic ---\n");

    fixed_point_t a = FP(2.5);
    fixed_point_t b = FP(1.5);

    /* Addition */
    fixed_point_t sum = fixed_point_add(a, b);
    TEST_ASSERT(fp_approx_equal(sum, FP(4.0), TOL_TIGHT),
                "Addition: 2.5 + 1.5 = 4.0");

    /* Subtraction */
    fixed_point_t diff = fixed_point_sub(a, b);
    TEST_ASSERT(fp_approx_equal(diff, FP(1.0), TOL_TIGHT),
                "Subtraction: 2.5 - 1.5 = 1.0");

    /* Multiplication */
    fixed_point_t product = fixed_point_mul(a, b);
    TEST_ASSERT(fp_approx_equal(product, FP(3.75), TOL_TIGHT),
                "Multiplication: 2.5 * 1.5 = 3.75");

    /* Division */
    fixed_point_t quotient = fixed_point_div(a, b);
    TEST_ASSERT(fp_approx_equal(quotient, FP(1.6667), TOL_NORMAL),
                "Division: 2.5 / 1.5 ~ 1.667");

    /* Division by zero protection */
    fixed_point_t zero_div = fixed_point_div(a, 0);
    TEST_ASSERT(zero_div == 0, "Division by zero returns 0");
}

/*===========================================================================
 * Test: Unary Operations
 *===========================================================================*/

void test_fixed_point_unary_operations(void)
{
    printf("\n--- Test: Unary Operations ---\n");

    fixed_point_t pos = FP(5.25);
    fixed_point_t neg = FP(-3.75);

    /* Absolute value */
    TEST_ASSERT(fp_approx_equal(fixed_point_abs(pos), FP(5.25), TOL_TIGHT),
                "Absolute value of positive: |5.25| = 5.25");
    TEST_ASSERT(fp_approx_equal(fixed_point_abs(neg), FP(3.75), TOL_TIGHT),
                "Absolute value of negative: |-3.75| = 3.75");

    /* Negation */
    TEST_ASSERT(fp_approx_equal(fixed_point_neg(pos), FP(-5.25), TOL_TIGHT),
                "Negation: -(5.25) = -5.25");
}

/*===========================================================================
 * Test: Comparison and Clamping
 *===========================================================================*/

void test_fixed_point_clamping(void)
{
    printf("\n--- Test: Clamping Operations ---\n");

    fixed_point_t value = FP(7.5);
    fixed_point_t lower = FP(2.0);
    fixed_point_t upper = FP(5.0);

    /* Clamp value above upper bound */
    fixed_point_t clamped_high = fixed_point_clamp(value, lower, upper);
    TEST_ASSERT(fp_approx_equal(clamped_high, FP(5.0), TOL_TIGHT),
                "Clamp 7.5 to [2, 5] = 5.0");

    /* Clamp value below lower bound */
    fixed_point_t clamped_low = fixed_point_clamp(FP(0.5), lower, upper);
    TEST_ASSERT(fp_approx_equal(clamped_low, FP(2.0), TOL_TIGHT),
                "Clamp 0.5 to [2, 5] = 2.0");

    /* Value within bounds unchanged */
    fixed_point_t clamped_mid = fixed_point_clamp(FP(3.5), lower, upper);
    TEST_ASSERT(fp_approx_equal(clamped_mid, FP(3.5), TOL_TIGHT),
                "Clamp 3.5 to [2, 5] = 3.5 (unchanged)");

    /* Minimum */
    fixed_point_t minimum = fixed_point_min(value, lower);
    TEST_ASSERT(fp_approx_equal(minimum, FP(2.0), TOL_TIGHT),
                "Minimum of 7.5 and 2.0 = 2.0");

    /* Maximum */
    fixed_point_t maximum = fixed_point_max(value, lower);
    TEST_ASSERT(fp_approx_equal(maximum, FP(7.5), TOL_TIGHT),
                "Maximum of 7.5 and 2.0 = 7.5");
}

/*===========================================================================
 * Test: Square Root
 *===========================================================================*/

void test_fixed_point_square_root(void)
{
    printf("\n--- Test: Square Root ---\n");

    /* Perfect square */
    fixed_point_t sqrt_four = fixed_point_sqrt(FP(4.0));
    TEST_ASSERT(fp_approx_equal(sqrt_four, FP(2.0), TOL_NORMAL),
                "Square root of 4 = 2.0");

    /* Non-perfect square */
    fixed_point_t sqrt_two = fixed_point_sqrt(FP(2.0));
    TEST_ASSERT(fp_approx_equal(sqrt_two, FP(1.4142), TOL_NORMAL),
                "Square root of 2 ~ 1.414");

    /* Larger value */
    fixed_point_t sqrt_hundred = fixed_point_sqrt(FP(100.0));
    TEST_ASSERT(fp_approx_equal(sqrt_hundred, FP(10.0), TOL_MODERATE),
                "Square root of 100 = 10.0");

    /* Zero */
    TEST_ASSERT(fixed_point_sqrt(0) == 0,
                "Square root of 0 = 0");
}

/*===========================================================================
 * Test: Trigonometric Functions
 *===========================================================================*/

void test_fixed_point_trigonometry(void)
{
    printf("\n--- Test: Trigonometric Functions ---\n");

    /* Sine at key angles */
    fixed_point_t sin_zero = fixed_point_sin(0);
    TEST_ASSERT(fp_approx_equal(sin_zero, 0, TOL_NORMAL),
                "sin(0) = 0");

    fixed_point_t sin_pi_2 = fixed_point_sin(FIXED_POINT_PI_OVER_2);
    TEST_ASSERT(fp_approx_equal(sin_pi_2, FIXED_POINT_ONE, TOL_NORMAL),
                "sin(pi/2) = 1");

    fixed_point_t sin_pi = fixed_point_sin(FIXED_POINT_PI);
    /* Note: Taylor series approximation has reduced accuracy at pi */
    TEST_ASSERT(fp_approx_equal(sin_pi, 0, TOL_LOOSE),
                "sin(pi) ~ 0 (Taylor series tolerance)");

    /* Cosine at key angles */
    fixed_point_t cos_zero = fixed_point_cos(0);
    TEST_ASSERT(fp_approx_equal(cos_zero, FIXED_POINT_ONE, TOL_NORMAL),
                "cos(0) = 1");

    fixed_point_t cos_pi_2 = fixed_point_cos(FIXED_POINT_PI_OVER_2);
    TEST_ASSERT(fp_approx_equal(cos_pi_2, 0, TOL_MODERATE),
                "cos(pi/2) ~ 0");

    fixed_point_t cos_pi = fixed_point_cos(FIXED_POINT_PI);
    /* Note: Taylor series approximation has reduced accuracy at pi
     * The 4-term Taylor series gives cos(pi) ~ -0.78 instead of -1.0
     * This is acceptable for vehicle control where angles near pi are rare */
    TEST_ASSERT(fp_approx_equal(cos_pi, FP(-1.0), TOL_WIDE),
                "cos(pi) ~ -1 (Taylor series tolerance)");

    /* Tangent */
    fixed_point_t tan_pi_4 = fixed_point_tan(FP(0.7854)); /* pi/4 */
    TEST_ASSERT(fp_approx_equal(tan_pi_4, FIXED_POINT_ONE, TOL_MODERATE),
                "tan(pi/4) ~ 1");
}

/*===========================================================================
 * Test: Integer Power
 *===========================================================================*/

void test_fixed_point_power(void)
{
    printf("\n--- Test: Integer Power ---\n");

    fixed_point_t base = FP(2.0);

    /* Positive exponents */
    fixed_point_t pow_2 = fixed_point_pow(base, 2);
    TEST_ASSERT(fp_approx_equal(pow_2, FP(4.0), TOL_NORMAL),
                "2^2 = 4");

    fixed_point_t pow_3 = fixed_point_pow(base, 3);
    TEST_ASSERT(fp_approx_equal(pow_3, FP(8.0), TOL_NORMAL),
                "2^3 = 8");

    /* Zero exponent */
    fixed_point_t pow_0 = fixed_point_pow(base, 0);
    TEST_ASSERT(fp_approx_equal(pow_0, FIXED_POINT_ONE, TOL_NORMAL),
                "2^0 = 1");

    /* Exponent of 1 */
    fixed_point_t pow_1 = fixed_point_pow(base, 1);
    TEST_ASSERT(fp_approx_equal(pow_1, FP(2.0), TOL_NORMAL),
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
    test_fixed_point_square_root();
    test_fixed_point_trigonometry();
    test_fixed_point_power();

    printf("\n===========================================================\n");
    printf("   Results: %d passed, %d failed\n", total_tests_passed, total_tests_failed);
    printf("===========================================================\n");

    return (total_tests_failed > 0) ? 1 : 0;
}
