/**
 * @file fp_math_hls.h
 * @brief Q16.16 Fixed-Point Math for HLS Synthesis
 *
 * Inline arithmetic operations and declarations for non-inline functions.
 * All operations use int32_t with int64_t intermediates for FPGA synthesis.
 */

#ifndef FP_MATH_HLS_H
#define FP_MATH_HLS_H

#include "mpc_fpga_types.h"

/*===========================================================================
 * Inline Arithmetic
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
#pragma HLS BIND_OP variable=product op=mul impl=dsp
    return (fixed_point_t)(product >> FP_FRAC_BITS);
}

static inline fixed_point_t fp_div(fixed_point_t a, fixed_point_t b)
{
    if (a == 0 || b == 0) return 0;
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

/*===========================================================================
 * Non-Inline Function Declarations
 *===========================================================================*/

/** Normalize angle to [-pi, pi] */
fixed_point_t fp_normalize_angle(fixed_point_t angle);

/** Reciprocal 1/x using Newton-Raphson (4 iterations) */
fixed_point_t fp_recip(fixed_point_t x);

/** Sine via Taylor series with range reduction */
fixed_point_t fp_sin(fixed_point_t angle);

/** Cosine via Taylor series with range reduction */
fixed_point_t fp_cos(fixed_point_t angle);

/** Arctangent with range reduction */
fixed_point_t fp_atan(fixed_point_t x);

#endif /* FP_MATH_HLS_H */
