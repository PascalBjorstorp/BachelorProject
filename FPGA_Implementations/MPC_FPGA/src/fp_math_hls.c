/**
 * @file fp_math_hls.c
 * @brief Q16.16 Fixed-Point Math — HLS-Synthesizable Implementation
 *
 * Contains only the functions needed by the MPC solver:
 *   normalize_angle, recip, sin, cos, atan
 * Stripped of sqrt, tan, pow, matrix ops (unused by Riccati-ADMM path).
 */

#include "../include/fp_math_hls.h"

/*===========================================================================
 * Constants
 *===========================================================================*/

#define RECIP_ITERATIONS    4
#define INV_FACT_3          10923   /* 1/3! in Q16.16 */
#define INV_FACT_4          2731    /* 1/4! */
#define INV_FACT_5          546     /* 1/5! */
#define INV_FACT_6          91      /* 1/6! */
#define INV_FACT_7          13      /* 1/7! */
#define INV_FACT_2          32768   /* 1/2! */

/* Atan Taylor coefficients */
#define ATAN_COEF_3         21845   /* 1/3 */
#define ATAN_COEF_5         13107   /* 1/5 */
#define ATAN_COEF_7         9362    /* 1/7 */
#define ATAN_COEF_9         7282    /* 1/9 */
#define ATAN_COEF_11        5958    /* 1/11 */
#define ATAN_COEF_13        5041    /* 1/13 */
#define FP_HALF_CONST       32768   /* 0.5 */
#define FP_ATAN_HALF        30386   /* atan(0.5) */

/*===========================================================================
 * Normalize Angle to [-pi, pi]
 *===========================================================================*/

fixed_point_t fp_normalize_angle(fixed_point_t angle)
{
#pragma HLS INLINE
    /* Loop-based: subtract/add 2*pi until in [-pi, pi]
     * 4 iterations handles angles up to ±5*pi — sufficient for MPC */
    int _i;
    for (_i = 0; _i < 4; _i++) {
#pragma HLS LOOP_TRIPCOUNT min=0 max=4
        if (angle <= FP_PI) break;
        angle -= FP_TWO_PI;
    }
    for (_i = 0; _i < 4; _i++) {
#pragma HLS LOOP_TRIPCOUNT min=0 max=4
        if (angle >= fp_neg(FP_PI)) break;
        angle += FP_TWO_PI;
    }
    return angle;
}

/*===========================================================================
 * Reciprocal: 1/x (Newton-Raphson, 6 iterations)
 *===========================================================================*/

fixed_point_t fp_recip(fixed_point_t x)
{
#pragma HLS INLINE off
    if (x == 0) return 0;

    int32_t sign = (x < 0) ? -1 : 1;
    fixed_point_t abs_x = fp_abs(x);

    /* Initial guess via leading-zero count (priority encoder) */
    int lead_zeros = __builtin_clz((unsigned int)abs_x) - 1;
    if (lead_zeros < 0) lead_zeros = 0;

    fixed_point_t est = (fixed_point_t)(1 << lead_zeros);

    /* Newton-Raphson: est = est + est*(1 - x*est) */
    int i;
    for (i = 0; i < RECIP_ITERATIONS; i++) {
#pragma HLS PIPELINE II=4
#pragma HLS LOOP_TRIPCOUNT min=6 max=6
        fixed_point_t prod = fp_mul(abs_x, est);
        fixed_point_t corr = fp_sub(FP_ONE, prod);
        fixed_point_t adj = fp_mul(est, corr);
        est = fp_add(est, adj);
    }

    return (sign < 0) ? fp_neg(est) : est;
}

/*===========================================================================
 * Sine: Taylor series with range reduction to [-pi/2, pi/2]
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

    term = fp_mul(term, x2);
    result = fp_sub(result, fp_mul(term, INV_FACT_7));

    return negate ? fp_neg(result) : result;
}

/*===========================================================================
 * Cosine: Taylor series with range reduction
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
    term = fp_mul(term, x2);
    result = fp_sub(result, fp_mul(term, INV_FACT_6));

    return negate ? fp_neg(result) : result;
}

/*===========================================================================
 * Arctangent helper: |x| <= 0.5 using Taylor series
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
    /* x^9 term removed: adds ~14 LSB error at |x|=0.5.
     * Combined with x^11/x^13 removal, total worst-case ~18 LSB.
     * Saves 2 fp_mul per call (DSP savings). */

    return result;
}

/*===========================================================================
 * Arctangent with range reduction
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
