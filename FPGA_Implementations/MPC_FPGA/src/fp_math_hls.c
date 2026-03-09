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

#define RECIP_ITERATIONS    4  /* CLZ gives ~1-bit initial guess (power-of-2),
                                  NR doubles bits each iter → 4 iters = 16 bits.
                                  Sufficient for Q16.16 (< 1 LSB worst-case). */
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
    /* DSP-pipelined normalization to [-pi, pi].
     *
     * The old bounded-loop approach (subtract/add 2*pi until in range) caused
     * HLS to optimize the loop trip-count into a division by FP_TWO_PI, which
     * it then strength-reduced to a 32×34-bit multiply by ceil(2^51/411775).
     * That multiply was purely combinational (0 pipeline, 22 LUT, 47 logic
     * levels), creating the critical timing path at 12.5 ns — the sole cause
     * of WNS = -2.7 ns.
     *
     * This version computes the same quotient via fp_mul (int64 multiply with
     * BIND_OP impl=dsp latency=3), routing it through 3-stage pipelined
     * DSP48E2 instead of LUT fabric.  Cost: ~1 extra DSP slice (shared across
     * all inlined normalize_angle calls via HLS resource sharing). */

    /* 1/(2*pi) in Q16.16 = round(65536 / (2*pi)) = 10430 */
#define FP_INV_TWO_PI  10430

    /* Compute floor((angle + pi) / (2*pi)) via DSP-pipelined multiply.
     * Adding pi first maps [-pi,pi] → [0,2pi] so floor gives 0 for that range,
     * avoiding boundary issues at exactly ±pi. */
    fixed_point_t shifted = fp_add(angle, FP_PI);
    fixed_point_t q = fp_mul(shifted, FP_INV_TWO_PI);  /* DSP via BIND_OP */

    /* Q16.16 → integer floor (arithmetic right-shift preserves sign) */
    int32_t q_int = q >> FP_FRAC_BITS;

    /* Subtract the integer multiple of 2*pi.
     * q_int is tiny (|q_int| <= 4 for MPC angles), so the multiply
     * q_int * FP_TWO_PI is implemented as shift-add, not a multiplier. */
    if (q_int != 0)
        angle -= (fixed_point_t)(q_int * FP_TWO_PI);

    /* Fine adjustment for any remaining off-by-one from rounding */
    if (angle > FP_PI)       angle -= FP_TWO_PI;
    if (angle < fp_neg(FP_PI)) angle += FP_TWO_PI;

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

    /* Initial guess via leading-zero count (priority encoder).
     * For Q16.16: true 1/x ≈ 2^(32-p) where p = MSB position.
     * clz = 31 - p, so 1/x ≈ 2^(clz+1). Use 2^clz for safe
     * underestimate keeping a*x_0 ∈ [0.5, 1.0]. */
    int lead_zeros = __builtin_clz((unsigned int)abs_x);

    fixed_point_t est = (fixed_point_t)(1 << lead_zeros);

    /* Newton-Raphson: est = est + est*(1 - x*est) */
    int i;
    for (i = 0; i < RECIP_ITERATIONS; i++) {
#pragma HLS PIPELINE II=2
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
