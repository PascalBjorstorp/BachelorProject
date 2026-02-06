/**
 * @file fixed_point.c
 * @brief Fixed-Point Arithmetic Implementation
 *
 * Implements advanced mathematical functions using only integer operations.
 * All functions are FPGA-compatible (no floating-point hardware required).
 *
 * Algorithms used:
 * - Newton-Raphson iteration for reciprocal and square root
 * - Taylor series for trigonometric functions
 */

#include "fixed_point.h"

/*===========================================================================
 * Newton-Raphson Iteration Constants
 *===========================================================================*/

/** Number of Newton-Raphson iterations for reciprocal */
#define RECIPROCAL_ITERATIONS 4

/** Number of Newton-Raphson iterations for square root */
#define SQUARE_ROOT_ITERATIONS 6

/** Convergence threshold for square root (in fixed-point units) */
#define SQUARE_ROOT_CONVERGENCE 10

/*===========================================================================
 * Trigonometric Constants (precomputed factorials in fixed-point)
 *===========================================================================*/

/** 1/3! = 1/6 in fixed-point */
#define FACTORIAL_3_RECIPROCAL fixed_point_from_float(0.166666667f)

/** 1/5! = 1/120 in fixed-point */
#define FACTORIAL_5_RECIPROCAL fixed_point_from_float(0.008333333f)

/** 1/7! = 1/5040 in fixed-point */
#define FACTORIAL_7_RECIPROCAL fixed_point_from_float(0.000198413f)

/** 1/2! = 1/2 in fixed-point */
#define FACTORIAL_2_RECIPROCAL FIXED_POINT_HALF

/** 1/4! = 1/24 in fixed-point */
#define FACTORIAL_4_RECIPROCAL fixed_point_from_float(0.041666667f)

/** 1/6! = 1/720 in fixed-point */
#define FACTORIAL_6_RECIPROCAL fixed_point_from_float(0.001388889f)

/** Threshold for tangent overflow protection */
#define TANGENT_COSINE_THRESHOLD fixed_point_from_float(0.01f)

/** Maximum tangent value (clamped to avoid overflow) */
#define TANGENT_MAXIMUM_VALUE fixed_point_from_float(100.0f)

/*===========================================================================
 * Helper: Normalize angle to [-π, π] range
 *===========================================================================*/

/**
 * Normalize angle to the range [-π, π]
 * This ensures Taylor series approximations are accurate.
 */
static fixed_point_t normalize_angle_to_pi_range(fixed_point_t angle_radians)
{
    while (angle_radians > FIXED_POINT_PI)
    {
        angle_radians = fixed_point_subtract(angle_radians, FIXED_POINT_TWO_PI);
    }
    while (angle_radians < -FIXED_POINT_PI)
    {
        angle_radians = fixed_point_add(angle_radians, FIXED_POINT_TWO_PI);
    }
    return angle_radians;
}

/*===========================================================================
 * Reciprocal: 1/x using Newton-Raphson
 *===========================================================================*/

/**
 * Compute 1/value using Newton-Raphson iteration.
 *
 * Algorithm: x_{n+1} = x_n × (2 - value × x_n)
 * Converges quadratically to 1/value.
 */
fixed_point_t fixed_point_reciprocal(fixed_point_t value)
{
    if (value == 0)
    {
        return 0; /* Avoid division by zero */
    }

    /* Handle sign separately */
    int32_t sign = (value < 0) ? -1 : 1;
    value = fixed_point_absolute(value);

    /* Initial guess: start with 1.0 */
    fixed_point_t estimate = FIXED_POINT_ONE;

    /* Newton-Raphson iterations */
    for (int iteration = 0; iteration < RECIPROCAL_ITERATIONS; iteration++)
    {
        /* correction = 1 - value × estimate */
        fixed_point_t product = fixed_point_multiply(value, estimate);
        fixed_point_t correction = fixed_point_subtract(FIXED_POINT_ONE, product);

        /* estimate = estimate + estimate × correction */
        fixed_point_t adjustment = fixed_point_multiply(estimate, correction);
        estimate = fixed_point_add(estimate, adjustment);
    }

    return sign * estimate;
}

/*===========================================================================
 * Square Root using Newton-Raphson
 *===========================================================================*/

/**
 * Compute √value using Newton-Raphson iteration.
 *
 * Algorithm: x_{n+1} = 0.5 × (x_n + value/x_n)
 * Converges quadratically to √value.
 */
fixed_point_t fixed_point_square_root(fixed_point_t value)
{
    if (value <= 0)
    {
        return 0; /* Square root of non-positive is zero */
    }

    /* Initial guess: value/2 */
    fixed_point_t estimate = value >> 1;
    if (estimate == 0)
    {
        estimate = 1; /* Ensure non-zero initial guess for small values */
    }

    /* Newton-Raphson iterations */
    for (int iteration = 0; iteration < SQUARE_ROOT_ITERATIONS; iteration++)
    {
        /* quotient = value / estimate */
        fixed_point_t quotient = fixed_point_divide(value, estimate);

        /* new_estimate = (estimate + quotient) / 2 */
        fixed_point_t sum = fixed_point_add(estimate, quotient);
        fixed_point_t new_estimate = sum >> 1;

        /* Check for convergence */
        fixed_point_t difference = fixed_point_absolute(
            fixed_point_subtract(new_estimate, estimate));
        if (difference < SQUARE_ROOT_CONVERGENCE)
        {
            return new_estimate;
        }

        estimate = new_estimate;
    }

    return estimate;
}

/*===========================================================================
 * Sine using Taylor Series
 *===========================================================================*/

/**
 * Compute sin(angle) using Taylor series expansion.
 *
 * Taylor series: sin(x) = x - x³/3! + x⁵/5! - x⁷/7! + ...
 * Accurate for x in [-π, π].
 */
fixed_point_t fixed_point_sine(fixed_point_t angle_radians)
{
    /* Normalize angle to [-π, π] for accuracy */
    angle_radians = normalize_angle_to_pi_range(angle_radians);

    /* Compute powers of angle */
    fixed_point_t angle_squared = fixed_point_multiply(angle_radians, angle_radians);
    fixed_point_t angle_cubed = fixed_point_multiply(angle_squared, angle_radians);
    fixed_point_t angle_fifth = fixed_point_multiply(angle_cubed, angle_squared);
    fixed_point_t angle_seventh = fixed_point_multiply(angle_fifth, angle_squared);

    /* Taylor series terms */
    fixed_point_t term_1 = angle_radians;
    fixed_point_t term_3 = fixed_point_divide(angle_cubed,
                                              fixed_point_from_float(6.0f));
    fixed_point_t term_5 = fixed_point_divide(angle_fifth,
                                              fixed_point_from_float(120.0f));
    fixed_point_t term_7 = fixed_point_divide(angle_seventh,
                                              fixed_point_from_float(5040.0f));

    /* sin(x) = x - x³/6 + x⁵/120 - x⁷/5040 */
    fixed_point_t result = term_1;
    result = fixed_point_subtract(result, term_3);
    result = fixed_point_add(result, term_5);
    result = fixed_point_subtract(result, term_7);

    return result;
}

/*===========================================================================
 * Cosine using Taylor Series
 *===========================================================================*/

/**
 * Compute cos(angle) using Taylor series expansion.
 *
 * Taylor series: cos(x) = 1 - x²/2! + x⁴/4! - x⁶/6! + ...
 * Accurate for x in [-π, π].
 */
fixed_point_t fixed_point_cosine(fixed_point_t angle_radians)
{
    /* Normalize angle to [-π, π] for accuracy */
    angle_radians = normalize_angle_to_pi_range(angle_radians);

    /* Compute powers of angle */
    fixed_point_t angle_squared = fixed_point_multiply(angle_radians, angle_radians);
    fixed_point_t angle_fourth = fixed_point_multiply(angle_squared, angle_squared);
    fixed_point_t angle_sixth = fixed_point_multiply(angle_fourth, angle_squared);

    /* Taylor series terms */
    fixed_point_t term_0 = FIXED_POINT_ONE;
    fixed_point_t term_2 = fixed_point_divide(angle_squared,
                                              fixed_point_from_float(2.0f));
    fixed_point_t term_4 = fixed_point_divide(angle_fourth,
                                              fixed_point_from_float(24.0f));
    fixed_point_t term_6 = fixed_point_divide(angle_sixth,
                                              fixed_point_from_float(720.0f));

    /* cos(x) = 1 - x²/2 + x⁴/24 - x⁶/720 */
    fixed_point_t result = term_0;
    result = fixed_point_subtract(result, term_2);
    result = fixed_point_add(result, term_4);
    result = fixed_point_subtract(result, term_6);

    return result;
}

/*===========================================================================
 * Tangent = Sine / Cosine
 *===========================================================================*/

/**
 * Compute tan(angle) = sin(angle) / cos(angle).
 *
 * Includes overflow protection near ±π/2 where tangent approaches infinity.
 */
fixed_point_t fixed_point_tangent(fixed_point_t angle_radians)
{
    fixed_point_t sine_value = fixed_point_sine(angle_radians);
    fixed_point_t cosine_value = fixed_point_cosine(angle_radians);

    /* Protect against division by near-zero cosine */
    if (fixed_point_absolute(cosine_value) < TANGENT_COSINE_THRESHOLD)
    {
        /* Return clamped large value with correct sign */
        if (sine_value >= 0)
        {
            return TANGENT_MAXIMUM_VALUE;
        }
        else
        {
            return fixed_point_negate(TANGENT_MAXIMUM_VALUE);
        }
    }

    return fixed_point_divide(sine_value, cosine_value);
}

/*===========================================================================
 * Integer Power
 *===========================================================================*/

/**
 * Compute base^exponent for integer exponents.
 *
 * Handles positive, negative, and zero exponents.
 * Uses iterative multiplication (FPGA-friendly, no recursion).
 */
fixed_point_t fixed_point_power_integer(fixed_point_t base, int exponent)
{
    /* Handle special cases */
    if (exponent == 0)
    {
        return FIXED_POINT_ONE; /* x^0 = 1 */
    }

    if (exponent < 0)
    {
        /* x^(-n) = 1 / x^n */
        fixed_point_t positive_power = fixed_point_power_integer(base, -exponent);
        return fixed_point_reciprocal(positive_power);
    }

    if (exponent == 1)
    {
        return base;
    }

    if (exponent == 2)
    {
        return fixed_point_multiply(base, base);
    }

    /* Iterative multiplication for larger exponents */
    fixed_point_t result = FIXED_POINT_ONE;
    for (int i = 0; i < exponent; i++)
    {
        result = fixed_point_multiply(result, base);
    }

    return result;
}
