/**
 * @file fp_math_mpcc.h
 * @brief Q16.16 Fixed-Point Math for MPCC Solver
 *
 * Self-contained fixed-point arithmetic for the MPCC.
 * Uses int32_t with int64_t intermediates
 */

#ifndef FP_MATH_H
#define FP_MATH_H

#include <stdint.h>

/*===========================================================================
 * Type Definition — Q16.16 Fixed-Point (int32_t)
 *===========================================================================*/

typedef int32_t fixed_point_t;

/*===========================================================================
 * Constants
 *===========================================================================*/

#define FP_FRAC_BITS    16
#define FP_ONE          (1 << FP_FRAC_BITS)       /* 65536 */
#define FP_TWO          (2 << FP_FRAC_BITS)       /* 131072 */
#define FP_HALF         (FP_ONE >> 1)              /* 32768 */
#define FP_PI           205887                     /* pi */
#define FP_PI_HALF      102943                     /* pi/2 */
#define FP_TWO_PI       411775                     /* 2*pi */

/*===========================================================================
 * Conversion Macros
 *===========================================================================*/

/** Compile-time float/double to Q16.16 conversion */
#define FP_CONST(x) ((fixed_point_t)(((double)(x) >= 0) ? \
                    ((double)(x) * FP_ONE + 0.5) : \
                    ((double)(x) * FP_ONE - 0.5)))

/** Runtime conversions */
#define DOUBLE_TO_FP(x) ((fixed_point_t)(((x) >= 0) ? \
                        ((x) * FP_ONE + 0.5) : ((x) * FP_ONE - 0.5)))
#define FP_TO_DOUBLE(x) ((double)(x) / (double)FP_ONE)
#define FP_TO_FLOAT(x)  ((float)(x) / (float)FP_ONE)

/*===========================================================================
 * Basic Arithmetic (inline)
 *===========================================================================*/

static inline fixed_point_t fp_add(fixed_point_t a, fixed_point_t b)
{
    return a + b;
}

static inline fixed_point_t fp_sub(fixed_point_t a, fixed_point_t b)
{
    return a - b;
}

static inline fixed_point_t fp_mul(fixed_point_t a, fixed_point_t b)
{
    int64_t product = (int64_t)a * (int64_t)b;
    return (fixed_point_t)(product >> FP_FRAC_BITS);
}

static inline fixed_point_t fp_div(fixed_point_t a, fixed_point_t b)
{
    if (a == 0) return 0;
    if (b == 0) return (a >= 0) ? INT32_MAX : INT32_MIN;
    return (fixed_point_t)(((int64_t)a << FP_FRAC_BITS) / b);
}

static inline fixed_point_t fp_abs(fixed_point_t a)
{
    return (a < 0) ? -a : a;
}

static inline fixed_point_t fp_neg(fixed_point_t a)
{
    return -a;
}

static inline fixed_point_t fp_min(fixed_point_t a, fixed_point_t b)
{
    return (a < b) ? a : b;
}

static inline fixed_point_t fp_max(fixed_point_t a, fixed_point_t b)
{
    return (a > b) ? a : b;
}

static inline fixed_point_t fp_clamp(fixed_point_t val,
                                     fixed_point_t lo,
                                     fixed_point_t hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

static inline fixed_point_t fp_add_sat(fixed_point_t a, fixed_point_t b)
{
    int64_t sum = (int64_t)a + (int64_t)b;
    if (sum > INT32_MAX) return INT32_MAX;
    if (sum < INT32_MIN) return INT32_MIN;
    return (fixed_point_t)sum;
}

static inline fixed_point_t fp_mul_sat(fixed_point_t a, fixed_point_t b)
{
    int64_t product = (int64_t)a * (int64_t)b;
    int64_t result = product >> FP_FRAC_BITS;
    if (result > INT32_MAX) return INT32_MAX;
    if (result < INT32_MIN) return INT32_MIN;
    return (fixed_point_t)result;
}

/*===========================================================================
 * Non-Inline Function Declarations
 *===========================================================================*/

/** Normalize angle to [-pi, pi] */
fixed_point_t fp_normalize_angle(fixed_point_t angle);

/** Reciprocal 1/x using Newton-Raphson */
fixed_point_t fp_recip(fixed_point_t x);

/** Sine via Taylor series with range reduction */
fixed_point_t fp_sin(fixed_point_t angle);

/** Cosine via Taylor series with range reduction */
fixed_point_t fp_cos(fixed_point_t angle);

/** Arctangent with range reduction */
fixed_point_t fp_atan(fixed_point_t x);

#endif /* FP_MATH_H */
