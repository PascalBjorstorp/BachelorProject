/**
 * @file fixed_point.h
 * @brief Fixed-Point Arithmetic Library for FPGA Compatibility
 *
 * This module provides Q16.16 fixed-point arithmetic operations.
 * All operations avoid floating-point hardware, making them suitable
 * for direct translation to HSL/FPGA implementations.
 *
 * Format: Q16.16 (16 integer bits, 16 fractional bits)
 * - Total: 32-bit signed integer
 * - Range: -32768.0 to +32767.99998 (approximately)
 * - Precision: 1/65536 ≈ 0.0000153
 *
 * Example representations:
 *   1.0     = 0x00010000 = 65536
 *   0.5     = 0x00008000 = 32768
 *   -1.0    = 0xFFFF0000 = -65536
 *   π       = 0x0003243F ≈ 205887
 *
 * @note For FPGA: All operations use only integer arithmetic.
 *       Multiplication requires 64-bit intermediate results.
 */

#ifndef FIXED_POINT_H
#define FIXED_POINT_H

#include <stdint.h>

/*===========================================================================
 * Type Definition
 *===========================================================================*/

/** Fixed-point number type (Q16.16 format) */
typedef int32_t fixed_point_t;

/*===========================================================================
 * Constants
 *===========================================================================*/

/** Number of bits used for the fractional part */
#define FIXED_POINT_FRACTIONAL_BITS 16

/** Scale factor: 2^16 = 65536 */
#define FIXED_POINT_SCALE (1 << FIXED_POINT_FRACTIONAL_BITS)

/** Fixed-point representation of 1.0 */
#define FIXED_POINT_ONE (1 << FIXED_POINT_FRACTIONAL_BITS)

/** Fixed-point representation of 0.5 */
#define FIXED_POINT_HALF (1 << (FIXED_POINT_FRACTIONAL_BITS - 1))

/** Fixed-point representation of π (3.14159265...) */
#define FIXED_POINT_PI 205887

/** Fixed-point representation of π/2 */
#define FIXED_POINT_PI_OVER_TWO 102943

/** Fixed-point representation of 2π */
#define FIXED_POINT_TWO_PI 411775

/*===========================================================================
 * Conversion Functions (for testing/debugging only - not for FPGA)
 *===========================================================================*/

/**
 * Convert floating-point to fixed-point
 * @note Uses float hardware - for testing only, not FPGA synthesis
 */
static inline fixed_point_t fixed_point_from_float(float value)
{
    return (fixed_point_t)(value * FIXED_POINT_SCALE + 0.5f);
}

/**
 * Convert fixed-point to floating-point
 * @note Uses float hardware - for testing only, not FPGA synthesis
 */
static inline float fixed_point_to_float(fixed_point_t value)
{
    return (float)value / FIXED_POINT_SCALE;
}

/*===========================================================================
 * Basic Arithmetic (FPGA-safe: integer operations only)
 *===========================================================================*/

/**
 * Addition: result = operand_a + operand_b
 */
static inline fixed_point_t fixed_point_add(fixed_point_t operand_a,
                                            fixed_point_t operand_b)
{
    return operand_a + operand_b;
}

/**
 * Subtraction: result = operand_a - operand_b
 */
static inline fixed_point_t fixed_point_subtract(fixed_point_t operand_a,
                                                 fixed_point_t operand_b)
{
    return operand_a - operand_b;
}

/**
 * Multiplication: result = operand_a × operand_b
 * @note Requires 64-bit intermediate to preserve precision
 */
static inline fixed_point_t fixed_point_multiply(fixed_point_t operand_a,
                                                 fixed_point_t operand_b)
{
    int64_t product = (int64_t)operand_a * (int64_t)operand_b;
    return (fixed_point_t)(product >> FIXED_POINT_FRACTIONAL_BITS);
}

/**
 * Division: result = numerator ÷ denominator
 * @note Returns 0 if denominator is zero (safe for FPGA)
 */
static inline fixed_point_t fixed_point_divide(fixed_point_t numerator,
                                               fixed_point_t denominator)
{
    if (denominator == 0)
    {
        return 0;
    }
    int64_t scaled_numerator = (int64_t)numerator << FIXED_POINT_FRACTIONAL_BITS;
    return (fixed_point_t)(scaled_numerator / denominator);
}

/*===========================================================================
 * Unary Operations
 *===========================================================================*/

/**
 * Absolute value: result = |value|
 */
static inline fixed_point_t fixed_point_absolute(fixed_point_t value)
{
    return (value < 0) ? -value : value;
}

/**
 * Negation: result = -value
 */
static inline fixed_point_t fixed_point_negate(fixed_point_t value)
{
    return -value;
}

/*===========================================================================
 * Comparison and Clamping
 *===========================================================================*/

/**
 * Minimum: result = min(value_a, value_b)
 */
static inline fixed_point_t fixed_point_minimum(fixed_point_t value_a,
                                                fixed_point_t value_b)
{
    return (value_a < value_b) ? value_a : value_b;
}

/**
 * Maximum: result = max(value_a, value_b)
 */
static inline fixed_point_t fixed_point_maximum(fixed_point_t value_a,
                                                fixed_point_t value_b)
{
    return (value_a > value_b) ? value_a : value_b;
}

/**
 * Clamp value to range [lower_bound, upper_bound]
 */
static inline fixed_point_t fixed_point_clamp(fixed_point_t value,
                                              fixed_point_t lower_bound,
                                              fixed_point_t upper_bound)
{
    if (value < lower_bound)
        return lower_bound;
    if (value > upper_bound)
        return upper_bound;
    return value;
}

/*===========================================================================
 * Advanced Math Functions (implemented in fixed_point.c)
 *===========================================================================*/

/**
 * Reciprocal: result = 1 / value
 * Uses Newton-Raphson iteration (FPGA-compatible)
 */
fixed_point_t fixed_point_reciprocal(fixed_point_t value);

/**
 * Square root: result = √value
 * Uses Newton-Raphson iteration (FPGA-compatible)
 */
fixed_point_t fixed_point_square_root(fixed_point_t value);

/**
 * Sine: result = sin(angle_radians)
 * Uses Taylor series approximation (FPGA-compatible)
 * @param angle_radians Angle in fixed-point radians
 */
fixed_point_t fixed_point_sine(fixed_point_t angle_radians);

/**
 * Cosine: result = cos(angle_radians)
 * Uses Taylor series approximation (FPGA-compatible)
 * @param angle_radians Angle in fixed-point radians
 */
fixed_point_t fixed_point_cosine(fixed_point_t angle_radians);

/**
 * Tangent: result = tan(angle_radians)
 * Computed as sin/cos with overflow protection
 * @param angle_radians Angle in fixed-point radians
 */
fixed_point_t fixed_point_tangent(fixed_point_t angle_radians);

/**
 * Integer power: result = base^exponent
 * @param base The base value
 * @param exponent Integer exponent (can be negative)
 */
fixed_point_t fixed_point_power_integer(fixed_point_t base, int exponent);

/*===========================================================================
 * Bit Shift Operations (for efficient scaling)
 *===========================================================================*/

/** Left shift (multiply by 2^shift_amount) */
#define fixed_point_shift_left(value, shift_amount) \
    ((fixed_point_t)((int64_t)(value) << (shift_amount)))

/** Right shift (divide by 2^shift_amount) */
#define fixed_point_shift_right(value, shift_amount) \
    ((fixed_point_t)((value) >> (shift_amount)))

#endif /* FIXED_POINT_H */
