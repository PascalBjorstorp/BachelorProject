/**
 * @file fp_math_hls.h
 * @brief Q16.16 fixed-point math helpers for HLS synthesis.
 * @details Provides inline fixed-point arithmetic primitives and declarations
 *          for non-inline transcendental and reciprocal kernels used by the
 *          FPGA MPC pipeline. All values are Q16.16 signed fixed-point unless
 *          otherwise noted.
 * @dependencies <stdint.h>, <limits.h>
 */

#ifndef FP_MATH_HLS_H
#define FP_MATH_HLS_H

#include <stdint.h>
#include <limits.h>

/*===========================================================================
 * Fixed-Point Type and Constants (Q16.16)
 *===========================================================================*/

typedef int32_t fixed_point_t;

#define FP_FRAC_BITS    16
#define FP_ONE          (1 << FP_FRAC_BITS)       /* 65536 (2^16) */
#define FP_TWO          (2 << FP_FRAC_BITS)       /* 131072 (2^17) */
#define FP_HALF         (FP_ONE >> 1)             /* 32768 (2^15) */
#define FP_PI           205887                    /* pi (3.14159 * 65536) */
#define FP_PI_HALF      102943                    /* pi/2 (1.5708 * 65536) */
#define FP_TWO_PI       411775                    /* 2*pi (6.28318 * 65536) */

/** Compile-time float to Q16.16 conversion
 *  Rounding is applied to minimize quantization error in constants.
*/
#define FP_CONST(x) ((fixed_point_t)(((double)(x) >= 0) ? \
                    ((double)(x) * FP_ONE + 0.5) : \
                    ((double)(x) * FP_ONE - 0.5)))

/* Fixed-point arithmetic helper macros (Q16.16) */
#define FP_MUL(a, b) ((fixed_point_t)(((int64_t)(a) * (int64_t)(b)) >> FP_FRAC_BITS))
#define FP_DIV(a, b) ((fixed_point_t)(((int64_t)(a) << FP_FRAC_BITS) / (int64_t)(b)))
#define FP_TO_FLOAT(x) ((float)(x) / FP_ONE)
#define FLOAT_TO_FP(x) ((fixed_point_t)((x) * FP_ONE))
#define FP_TO_DOUBLE(x) ((double)(x) / FP_ONE)
#define DOUBLE_TO_FP(x) ((fixed_point_t)((x) * FP_ONE))

/*===========================================================================
 * Inline Arithmetic
 *===========================================================================*/

/**
 * @brief Add two Q16.16 values.
 * @param a First addend.
 * @param b Second addend.
 * @return Sum a + b.
 */
static inline fixed_point_t fp_add(fixed_point_t a, fixed_point_t b)
{
    return a + b;
}

/**
 * @brief Subtract two Q16.16 values.
 * @param a Minuend.
 * @param b Subtrahend.
 * @return Difference a - b.
 */
static inline fixed_point_t fp_sub(fixed_point_t a, fixed_point_t b)
{
    return a - b;
}

/**
 * @brief Multiply two Q16.16 values.
 * @param a First multiplicand.
 * @param b Second multiplicand.
 * @return Product in Q16.16.
 */
static inline fixed_point_t fp_mul(fixed_point_t a, fixed_point_t b)
{
    int64_t product = (int64_t)a * (int64_t)b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp latency=3
    return (fixed_point_t)(product >> FP_FRAC_BITS);
}

/**
 * @brief Divide two Q16.16 values.
 * @param a Dividend.
 * @param b Divisor.
 * @return Quotient a/b, or zero when either operand is zero.
 */
static inline fixed_point_t fp_div(fixed_point_t a, fixed_point_t b)
{
    /* Return 0 for undefined divisions to avoid divide-by-zero hardware paths. */
    if (a == 0 || b == 0) return 0;
    return (fixed_point_t)(((int64_t)a << FP_FRAC_BITS) / b);
}

/**
 * @brief Compute absolute value with saturation safety.
 * @param a Input value.
 * @return |a|, saturated to INT32_MAX for INT32_MIN input.
 */
static inline fixed_point_t fp_abs(fixed_point_t a)
{
    if (a == INT32_MIN) return INT32_MAX;
    return (a < 0) ? -a : a;
}

/**
 * @brief Negate a Q16.16 value.
 * @param a Input value.
 * @return -a.
 */
static inline fixed_point_t fp_neg(fixed_point_t a)
{
    return -a;
}

/**
 * @brief Return the minimum of two Q16.16 values.
 * @param a First value.
 * @param b Second value.
 * @return The smaller of a and b.
 */
static inline fixed_point_t fp_min(fixed_point_t a, fixed_point_t b)
{
    return (a < b) ? a : b;
}

/**
 * @brief Return the maximum of two Q16.16 values.
 * @param a First value.
 * @param b Second value.
 * @return The larger of a and b.
 */
static inline fixed_point_t fp_max(fixed_point_t a, fixed_point_t b)
{
    return (a > b) ? a : b;
}

/**
 * @brief Clamp a Q16.16 value to a closed interval.
 * @param val Input value.
 * @param lo Lower bound (inclusive).
 * @param hi Upper bound (inclusive).
 * @return val constrained to [lo, hi].
 */
static inline fixed_point_t fp_clamp(fixed_point_t val,
                                     fixed_point_t lo,
                                     fixed_point_t hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/**
 * @brief Add two values with signed 32-bit saturation.
 * @param a First addend.
 * @param b Second addend.
 * @return Saturating sum in INT32 range.
 */
static inline fixed_point_t fp_add_sat(fixed_point_t a, fixed_point_t b)
{
    int64_t sum = (int64_t)a + (int64_t)b;
    if (sum > INT32_MAX) return INT32_MAX;
    if (sum < INT32_MIN) return INT32_MIN;
    return (fixed_point_t)sum;
}

/*===========================================================================
 * Non-Inline Function Declarations
 *===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Normalize an angle to the range [-pi, pi].
 * @param angle Input angle in Q16.16 radians.
 * @return Equivalent wrapped angle in [-pi, pi].
 */
fixed_point_t fp_normalize_angle(fixed_point_t angle);

/**
 * @brief Compute reciprocal 1/x using fixed-iteration Newton-Raphson.
 * @param x Input value.
 * @return Reciprocal of x, or zero when x is zero.
 */
fixed_point_t fp_recip(fixed_point_t x);

/**
 * @brief Compute sine using range reduction and polynomial approximation.
 * @param angle Input angle in Q16.16 radians.
 * @return Approximate sine(angle) in Q16.16.
 */
fixed_point_t fp_sin(fixed_point_t angle);

/**
 * @brief Compute cosine using range reduction and polynomial approximation.
 * @param angle Input angle in Q16.16 radians.
 * @return Approximate cosine(angle) in Q16.16.
 */
fixed_point_t fp_cos(fixed_point_t angle);

/**
 * @brief Compute arctangent using piecewise range reduction.
 * @param x Input value.
 * @return Approximate atan(x) in Q16.16 radians.
 */
fixed_point_t fp_atan(fixed_point_t x);

/**
 * @brief Fast cubic arctangent approximation used in tire-model paths.
 * @param x Input value.
 * @return Approximate atan(x) in Q16.16 radians.
 */
fixed_point_t fp_atan_tire_approx(fixed_point_t x);

#ifdef __cplusplus
}
#endif

#endif /* FP_MATH_HLS_H */
