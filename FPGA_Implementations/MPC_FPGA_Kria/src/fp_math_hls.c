/**
 * @file fp_math_hls.c
 * @brief Q16.16 fixed-point math kernels for HLS synthesis.
 * @details Implements non-inline trigonometric and reciprocal kernels used by
 *          the FPGA MPC pipeline. Arithmetic is performed in fixed-point with
 *          deterministic iteration counts suited for synthesis scheduling.
 * @dependencies fp_math_hls.h
 */

#include "../include/fp_math_hls.h"

#ifndef MPC_HLS_RECIP_II
#define MPC_HLS_RECIP_II 2
#endif

/*===========================================================================
 * Constants
 *===========================================================================*/

#define RECIP_ITERATIONS    3       /*  Number of Newton-Raphson iterations for reciprocal approximation. */

#define INV_FACT_2          32768   /* 1/2! */
#define INV_FACT_3          10923   /* 1/3! in Q16.16 */
#define INV_FACT_4          2731    /* 1/4! */
#define INV_FACT_5          546     /* 1/5! */

/* atan polynomial coefficients */
#define ATAN_COEF_3         21845   /* 1/3 */
#define ATAN_COEF_5         13107   /* 1/5 */
#define ATAN_COEF_7         9362    /* 1/7 */
#define FP_HALF_CONST       32768   /* 0.5 */
#define FP_ATAN_HALF        30386   /* atan(0.5) */
#define FP_INV_TWO_PI       10430   /* 1/(2*pi) */

/*===========================================================================
 * Normalize Angle to [-pi, pi]
 *===========================================================================*/

fixed_point_t fp_normalize_angle(fixed_point_t angle)
{
#pragma HLS INLINE
    /*
     * Compute floor((angle + pi) / (2*pi)) using fixed-point multiply.
     * Adding pi first maps [-pi, pi] to [0, 2*pi], which keeps boundaries stable.
     */
    fixed_point_t shifted = fp_add(angle, FP_PI);
    fixed_point_t q = fp_mul(shifted, FP_INV_TWO_PI);

    /* Q16.16 to integer floor. */
    int32_t q_int = q >> FP_FRAC_BITS;

    /* Subtract the nearest integer multiple of 2*pi in fixed-point units. */
    if (q_int != 0)
        angle -= (fixed_point_t)(q_int * FP_TWO_PI);

    /* Final correction for rounding edge cases near boundaries. */
    if (angle > FP_PI)
        angle -= FP_TWO_PI;
        
    if (angle < fp_neg(FP_PI)) 
        angle += FP_TWO_PI;

    return angle;
}

/*===========================================================================
 * Reciprocal: 1/x (Newton-Raphson, Resource-Optimized)
 *
 * Optimization: 3-iteration Newton-Raphson with explicit DSP binding
 * and 4-cycle latency for timing closure at higher clocks.
 *===========================================================================*/

fixed_point_t fp_recip(fixed_point_t x)
{
#pragma HLS INLINE off
#pragma HLS PIPELINE II=MPC_HLS_RECIP_II
#pragma HLS ALLOCATION operation instances=mul limit=6

    if (x == 0) return 0;

    int32_t sign = (x < 0) ? -1 : 1;
    fixed_point_t abs_x = fp_abs(x);

    /* Initial guess via leading-zero count (priority encoder).
     * For Q16.16: true 1/x is approximately 2^(32-p), where p = MSB position.
     * clz = 31 - p, so 1/x is approximately 2^(clz+1). Use 2^clz for safe
     * underestimate keeping a*x_0 in [0.5, 1.0]. */
    int lead_zeros = __builtin_clz((unsigned int)abs_x);

    fixed_point_t est = (fixed_point_t)(1 << lead_zeros);

    /* Newton-Raphson: est = est + est*(1 - x*est)
     * 3 iterations with explicit multiply binding for timing closure.
     */

    /* Iteration 1 */
    int64_t prod1 = (int64_t)abs_x * (int64_t)est;
    fixed_point_t corr1 = FP_ONE - (fixed_point_t)(prod1 >> FP_FRAC_BITS);
    int64_t adj1 = (int64_t)est * (int64_t)corr1;
    fixed_point_t est1 = est + (fixed_point_t)(adj1 >> FP_FRAC_BITS);

    /* Iteration 2 */
    int64_t prod2 = (int64_t)abs_x * (int64_t)est1;
    fixed_point_t corr2 = FP_ONE - (fixed_point_t)(prod2 >> FP_FRAC_BITS);
    int64_t adj2 = (int64_t)est1 * (int64_t)corr2;
    fixed_point_t est2 = est1 + (fixed_point_t)(adj2 >> FP_FRAC_BITS);

    /* Iteration 3 (final) */
    int64_t prod3 = (int64_t)abs_x * (int64_t)est2;
    fixed_point_t corr3 = FP_ONE - (fixed_point_t)(prod3 >> FP_FRAC_BITS);
    int64_t adj3 = (int64_t)est2 * (int64_t)corr3;
    fixed_point_t est_final = est2 + (fixed_point_t)(adj3 >> FP_FRAC_BITS);

    return (sign < 0) ? fp_neg(est_final) : est_final;
}

/*===========================================================================
 * Sine: range reduction + truncated Taylor series
 *===========================================================================*/

fixed_point_t fp_sin(fixed_point_t angle)
{
#pragma HLS INLINE
    angle = fp_normalize_angle(angle);

    int negate = 0;
    if (angle > FP_PI_HALF) {
        angle = fp_sub(FP_PI, angle);
    } else if (angle < -FP_PI_HALF) {
        angle = fp_add(FP_PI, angle);
        negate = 1;
    }

    fixed_point_t x2 = fp_mul(angle, angle);
    fixed_point_t result = angle;
    fixed_point_t term = angle;

    term = fp_mul(term, x2);
    result = fp_sub(result, fp_mul(term, INV_FACT_3));

    term = fp_mul(term, x2);
    result = fp_add(result, fp_mul(term, INV_FACT_5));

    return negate ? fp_neg(result) : result;
}

/*===========================================================================
 * Cosine: range reduction + truncated Taylor series
 *===========================================================================*/

fixed_point_t fp_cos(fixed_point_t angle)
{
#pragma HLS INLINE
    angle = fp_normalize_angle(angle);
    angle = fp_abs(angle);

    int negate = 0;
    if (angle > FP_PI_HALF) {
        angle = fp_sub(FP_PI, angle);
        negate = 1;
    }

    fixed_point_t x2 = fp_mul(angle, angle);
    fixed_point_t result = FP_ONE;
    fixed_point_t term = x2;

    result = fp_sub(result, fp_mul(term, INV_FACT_2));
    term = fp_mul(term, x2);
    result = fp_add(result, fp_mul(term, INV_FACT_4));

    return negate ? fp_neg(result) : result;
}

/*===========================================================================
 * Arctangent helper for |x| <= 0.5
 *===========================================================================*/

static fixed_point_t fp_atan_small(fixed_point_t x)
{
#pragma HLS INLINE
    fixed_point_t x2 = fp_mul(x, x);
    fixed_point_t term = x;
    fixed_point_t result = x;

    term = fp_mul(term, x2);
    result = fp_sub(result, fp_mul(term, ATAN_COEF_3));
    term = fp_mul(term, x2);
    result = fp_add(result, fp_mul(term, ATAN_COEF_5));
    term = fp_mul(term, x2);
    result = fp_sub(result, fp_mul(term, ATAN_COEF_7));

    return result;
}

/*===========================================================================
 * Arctangent with piecewise range reduction
 *===========================================================================*/

fixed_point_t fp_atan(fixed_point_t x)
{
#pragma HLS INLINE
    if (x == 0) return 0;

    int32_t sign = (x < 0) ? -1 : 1;
    fixed_point_t abs_x = fp_abs(x);
    fixed_point_t result;

    if (abs_x <= FP_HALF_CONST) {
        result = fp_atan_small(abs_x);
    } else if (abs_x <= FP_ONE) {
        /* atan(x) = atan(0.5) + atan((x-0.5)/(1+0.5*x)) */
        fixed_point_t num = fp_sub(abs_x, FP_HALF_CONST);
        fixed_point_t den = fp_add(FP_ONE, (abs_x >> 1));  /* 0.5*x via shift */
        fixed_point_t inv_den = fp_recip(den);
        fixed_point_t reduced = fp_mul(num, inv_den);
        result = fp_add(FP_ATAN_HALF, fp_atan_small(reduced));
    } else {
        /* atan(x) = pi/2 - atan(1/x) */
        fixed_point_t inv_x = fp_recip(abs_x);
        if (inv_x <= FP_HALF_CONST) {
            result = fp_sub(FP_PI_HALF, fp_atan_small(inv_x));
        } else {
            fixed_point_t num = fp_sub(inv_x, FP_HALF_CONST);
            fixed_point_t den = fp_add(FP_ONE, (inv_x >> 1));  /* 0.5*x via shift */
            fixed_point_t inv_den = fp_recip(den);
            fixed_point_t reduced = fp_mul(num, inv_den);
            fixed_point_t atan_inv = fp_add(FP_ATAN_HALF, fp_atan_small(reduced));
            result = fp_sub(FP_PI_HALF, atan_inv);
        }
    }

    return (sign < 0) ? fp_neg(result) : result;
}

/*===========================================================================
 * Cubic atan approximation for tire-model angle terms
 *===========================================================================*/

fixed_point_t fp_atan_tire_approx(fixed_point_t x)
{
#pragma HLS INLINE
    fixed_point_t x2 = fp_mul(x, x);
    fixed_point_t x3 = fp_mul(x2, x);
    return fp_sub(x, fp_mul(x3, FP_CONST(0.33333333)));
}
