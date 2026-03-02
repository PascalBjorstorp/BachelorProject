/**
 * @file fp_math.c
 * @brief Q16.16 Fixed-Point Math Implementation
 */

#include "fp_math.h"
#include <string.h>
#include <stdint.h>

/*===========================================================================
 * Newton-Raphson Constants
 *===========================================================================*/

#define RECIP_ITERATIONS 4
#define SQRT_ITERATIONS  6
#define SQRT_CONVERGE    10

/*===========================================================================
 * Trigonometric Constants (Taylor series coefficients)
 *===========================================================================*/

#define INV_FACT_2  32768   /* 1/2! = 1/2  = 0.5 in Q16.16 */
#define INV_FACT_3  10923   /* 1/3! = 1/6  ≈ 0.1667 in Q16.16 */
#define INV_FACT_4  2731    /* 1/4! = 1/24 ≈ 0.0417 in Q16.16 */
#define INV_FACT_5  546     /* 1/5! = 1/120 */
#define INV_FACT_6  91      /* 1/6! = 1/720 */
#define INV_FACT_7  13      /* 1/7! = 1/5040 ≈ 0.000198 in Q16.16 */

#define TAN_THRESHOLD 4096  /* ~0.0625 rad = 3.6° from ±π/2 */
#define TAN_MAX       ((fixed_point_t)(1 << 30))

/*===========================================================================
 * Helper: Normalize angle to [-π, π]
 *===========================================================================*/

fixed_point_t fp_normalize_angle(fixed_point_t angle)
{
#ifdef MPC_HLS_TARGET
#pragma HLS INLINE
#endif
    /* Fast path: compute wraps needed via integer division.
     * This handles arbitrarily large angles in O(1) instead of O(n).
     * The final bounded loop is only for rounding correction (≤2 iterations). */
    if (angle > FP_PI || angle < -FP_PI) {
        /* n = floor((angle + π) / (2π)) */
        int32_t shifted = angle + FP_PI;
        int32_t n = shifted / FP_TWO_PI;
        if (shifted < 0 && (shifted % FP_TWO_PI) != 0) n--;  /* floor for negatives */
        angle -= n * FP_TWO_PI;
    }
    /* Correct any rounding residual (at most 2 iterations) */
    for (int _i = 0; _i < 2 && angle > FP_PI; _i++)
        angle = fp_sub(angle, FP_TWO_PI);
    for (int _i = 0; _i < 2 && angle < -FP_PI; _i++)
        angle = fp_add(angle, FP_TWO_PI);
    return angle;
}

/*===========================================================================
 * Reciprocal: 1/x using Newton-Raphson
 *===========================================================================*/

fixed_point_t fp_recip(fixed_point_t x)
{
#ifdef MPC_HLS_TARGET
#pragma HLS INLINE
#endif
    if (x == 0) return 0;

    int32_t sign = (x < 0) ? -1 : 1;
    fixed_point_t abs_x = fp_abs(x);

    /* Initial guess via leading zeros */
    int lead_zeros = 0;
    int32_t temp = abs_x;
    while (!(temp & 0x40000000) && lead_zeros < 31)
    {
        temp <<= 1;
        lead_zeros++;
    }

    fixed_point_t est = (fixed_point_t)(1 << lead_zeros);

    /* Newton-Raphson: est = est + est*(1 - x*est) */
    for (int i = 0; i < RECIP_ITERATIONS; i++)
    {
        fixed_point_t prod = fp_mul(abs_x, est);
        fixed_point_t corr = fp_sub(FP_ONE, prod);
        fixed_point_t adj = fp_mul(est, corr);
        est = fp_add(est, adj);
    }

    return (sign < 0) ? fp_neg(est) : est;
}

/*===========================================================================
 * Square Root using Newton-Raphson (inverse sqrt method)
 *===========================================================================*/

fixed_point_t fp_sqrt(fixed_point_t x)
{
#ifdef MPC_HLS_TARGET
#pragma HLS INLINE
#endif
    if (x <= 0) return 0;

    /* Initial guess */
    int lead_zeros = 0;
    int32_t temp = x;
    while (!(temp & 0x40000000) && lead_zeros < 31)
    {
        temp <<= 1;
        lead_zeros++;
    }

    fixed_point_t y = (fixed_point_t)(1 << (9 + (lead_zeros >> 1)));
    fixed_point_t three = FP_CONST(3.0);

    /* Newton-Raphson for inverse sqrt: y = (y/2) * (3 - x*y²) */
    for (int i = 0; i < SQRT_ITERATIONS; i++)
    {
        fixed_point_t y_sq = fp_mul(y, y);
        fixed_point_t x_y_sq = fp_mul(x, y_sq);
        fixed_point_t factor = fp_sub(three, x_y_sq);
        fixed_point_t next_y = fp_mul(y >> 1, factor);

        if (fp_abs(fp_sub(next_y, y)) < SQRT_CONVERGE)
        {
            y = next_y;
            break;
        }
        y = next_y;
    }

    /* sqrt(x) = x * (1/sqrt(x)) */
    return fp_mul(x, y);
}

/*===========================================================================
 * Sine: Taylor series with range reduction to [-π/2, π/2]
 * sin(x) = x - x³/3! + x⁵/5! - x⁷/7!
 *===========================================================================*/

fixed_point_t fp_sin(fixed_point_t angle)
{
#ifdef MPC_HLS_TARGET
#pragma HLS INLINE
#endif
    /* Normalize to [-π, π] */
    angle = fp_normalize_angle(angle);
    
    /* Range reduction: reduce to [-π/2, π/2] where Taylor is accurate */
    int negate = 0;
    
    if (angle > FP_PI_HALF) {
        /* sin(x) = sin(π - x) for x in (π/2, π] */
        angle = fp_sub(FP_PI, angle);
    } else if (angle < -FP_PI_HALF) {
        /* sin(x) = -sin(-π - x) = sin(-(π + x)) for x in [-π, -π/2) */
        angle = fp_add(FP_PI, angle);
        negate = 1;
    }
    
    /* Taylor series: x - x³/3! + x⁵/5! - x⁷/7! */
    fixed_point_t x2 = fp_mul(angle, angle);

    fixed_point_t result = angle;
    fixed_point_t term = angle;

    term = fp_mul(term, x2);  /* x³ */
    result = fp_sub(result, fp_mul(term, INV_FACT_3));

    term = fp_mul(term, x2);  /* x⁵ */
    result = fp_add(result, fp_mul(term, INV_FACT_5));

    term = fp_mul(term, x2);  /* x⁷ */
    result = fp_sub(result, fp_mul(term, INV_FACT_7));

    return negate ? fp_neg(result) : result;
}

/*===========================================================================
 * Cosine: Taylor series with range reduction to [-π/2, π/2]
 * cos(x) = 1 - x²/2! + x⁴/4! - x⁶/6!
 *===========================================================================*/

fixed_point_t fp_cos(fixed_point_t angle)
{
#ifdef MPC_HLS_TARGET
#pragma HLS INLINE
#endif
    /* Normalize to [-π, π] */
    angle = fp_normalize_angle(angle);
    
    /* cos(-x) = cos(x), so work with absolute value */
    angle = fp_abs(angle);
    
    /* Range reduction: reduce to [0, π/2] where Taylor is accurate */
    int negate = 0;
    
    if (angle > FP_PI_HALF) {
        /* cos(x) = -cos(π - x) for x in (π/2, π] */
        angle = fp_sub(FP_PI, angle);
        negate = 1;
    }
    
    /* Taylor series: 1 - x²/2! + x⁴/4! - x⁶/6! */
    fixed_point_t x2 = fp_mul(angle, angle);

    fixed_point_t result = FP_ONE;
    fixed_point_t term = x2;  /* x² */

    result = fp_sub(result, fp_mul(term, INV_FACT_2));  /* -x²/2! */

    term = fp_mul(term, x2);  /* x⁴ */
    result = fp_add(result, fp_mul(term, INV_FACT_4));  /* +x⁴/4! */

    term = fp_mul(term, x2);  /* x⁶ */
    result = fp_sub(result, fp_mul(term, INV_FACT_6));  /* -x⁶/6! */

    return negate ? fp_neg(result) : result;
}

/*===========================================================================
 * Tangent: sin/cos with overflow protection near ±π/2
 *===========================================================================*/

fixed_point_t fp_tan(fixed_point_t angle)
{
#ifdef MPC_HLS_TARGET
#pragma HLS INLINE
#endif
    angle = fp_normalize_angle(angle);

    /* Protect against overflow near ±π/2 */
    fixed_point_t dist_to_pi_half = fp_abs(fp_sub(fp_abs(angle), FP_PI_HALF));
    
    if (dist_to_pi_half < TAN_THRESHOLD) {
        /* Very close to ±π/2, return large value with correct sign */
        return (angle > 0) ? TAN_MAX : fp_neg(TAN_MAX);
    }

    fixed_point_t cos_val = fp_cos(angle);
    
    /* Additional safety: if cos is very small, clamp */
    if (fp_abs(cos_val) < 256) {  /* ~0.004 */
        return (angle > 0) ? TAN_MAX : fp_neg(TAN_MAX);
    }

    return fp_div(fp_sin(angle), cos_val);
}

/*===========================================================================
 * Integer Power: base^exponent
 *===========================================================================*/

fixed_point_t fp_pow(fixed_point_t base, int exp)
{
#ifdef MPC_HLS_TARGET
#pragma HLS INLINE
#endif
    if (exp == 0) return FP_ONE;
    if (exp == 1) return base;
    if (exp == -1) return fp_recip(base);
    if (exp == 2) return fp_mul(base, base);

    fixed_point_t result = FP_ONE;
    int abs_exp = (exp < 0) ? -exp : exp;
    fixed_point_t curr = base;

    while (abs_exp > 0)
    {
        if (abs_exp & 1)
            result = fp_mul(result, curr);
        curr = fp_mul(curr, curr);
        abs_exp >>= 1;
    }

    if (exp < 0)
        result = fp_recip(result);

    return result;
}

/*===========================================================================
 * Arctangent: Taylor series with range reduction (matches FPGA)
 * 
 * Strategy:
 *   |x| <= 0.5:     Use Taylor series (converges fast)
 *   0.5 < |x| <= 1: Use atan(x) = atan(0.5) + atan((x-0.5)/(1+0.5*x))
 *   |x| > 1:        Use atan(x) = π/2 - atan(1/x)
 * 
 * This avoids the slow convergence at x=1.
 *===========================================================================*/

/* Taylor series coefficients for atan(x): atan(x) = x - x³/3 + x⁵/5 - ... */
#define ATAN_COEF_3  21845    /* 1/3 in Q16.16 */
#define ATAN_COEF_5  13107    /* 1/5 in Q16.16 */
#define ATAN_COEF_7  9362     /* 1/7 in Q16.16 */
#define ATAN_COEF_9  7282     /* 1/9 in Q16.16 */
#define ATAN_COEF_11 5958     /* 1/11 in Q16.16 */
#define ATAN_COEF_13 5041     /* 1/13 in Q16.16 */

/* Precomputed constants */
#define FP_HALF_CONST   32768  /* 0.5 in Q16.16 */
#define FP_ATAN_HALF    30386  /* atan(0.5) ≈ 0.4636 in Q16.16 */

/* Internal helper: atan for |x| <= 0.5 using Taylor series */
static fixed_point_t fp_atan_small(fixed_point_t x)
{
#ifdef MPC_HLS_TARGET
#pragma HLS INLINE
#endif
    fixed_point_t x2 = fp_mul(x, x);
    fixed_point_t term = x;
    fixed_point_t result = x;
    
    term = fp_mul(term, x2);  /* x³ */
    result = fp_sub(result, fp_mul(term, ATAN_COEF_3));
    
    term = fp_mul(term, x2);  /* x⁵ */
    result = fp_add(result, fp_mul(term, ATAN_COEF_5));
    
    term = fp_mul(term, x2);  /* x⁷ */
    result = fp_sub(result, fp_mul(term, ATAN_COEF_7));
    
    term = fp_mul(term, x2);  /* x⁹ */
    result = fp_add(result, fp_mul(term, ATAN_COEF_9));
    
    term = fp_mul(term, x2);  /* x¹¹ */
    result = fp_sub(result, fp_mul(term, ATAN_COEF_11));
    
    term = fp_mul(term, x2);  /* x¹³ */
    result = fp_add(result, fp_mul(term, ATAN_COEF_13));
    
    return result;
}

fixed_point_t fp_atan(fixed_point_t x)
{
#ifdef MPC_HLS_TARGET
#pragma HLS INLINE
#endif
    if (x == 0) return 0;
    
    int32_t sign = (x < 0) ? -1 : 1;
    fixed_point_t abs_x = fp_abs(x);
    fixed_point_t result;
    
    if (abs_x <= FP_HALF_CONST)
    {
        /* |x| <= 0.5: Direct Taylor series (fast convergence) */
        result = fp_atan_small(abs_x);
    }
    else if (abs_x <= FP_ONE)
    {
        /* 0.5 < |x| <= 1: Range reduction */
        /* atan(x) = atan(0.5) + atan((x - 0.5) / (1 + 0.5*x)) */
        fixed_point_t numerator = fp_sub(abs_x, FP_HALF_CONST);
        fixed_point_t denominator = fp_add(FP_ONE, fp_mul(FP_HALF_CONST, abs_x));
        fixed_point_t reduced = fp_div(numerator, denominator);
        result = fp_add(FP_ATAN_HALF, fp_atan_small(reduced));
    }
    else
    {
        /* |x| > 1: Use atan(x) = π/2 - atan(1/x) */
        fixed_point_t inv_x = fp_div(FP_ONE, abs_x);
        
        if (inv_x <= FP_HALF_CONST)
        {
            result = fp_sub(FP_PI_HALF, fp_atan_small(inv_x));
        }
        else
        {
            /* 0.5 < 1/x <= 1: Range reduction on inverse */
            fixed_point_t numerator = fp_sub(inv_x, FP_HALF_CONST);
            fixed_point_t denominator = fp_add(FP_ONE, fp_mul(FP_HALF_CONST, inv_x));
            fixed_point_t reduced = fp_div(numerator, denominator);
            fixed_point_t atan_inv = fp_add(FP_ATAN_HALF, fp_atan_small(reduced));
            result = fp_sub(FP_PI_HALF, atan_inv);
        }
    }
    
    return (sign < 0) ? fp_neg(result) : result;
}

/*===========================================================================
 * Two-argument Arctangent: atan2(y, x)
 * 
 * Returns angle in [-π, π] with correct quadrant handling.
 * Essential for steering calculations.
 *===========================================================================*/

fixed_point_t fp_atan2(fixed_point_t y, fixed_point_t x)
{
#ifdef MPC_HLS_TARGET
#pragma HLS INLINE
#endif
    /* Handle special cases */
    if (x == 0)
    {
        if (y > 0) return FP_PI_HALF;
        if (y < 0) return fp_neg(FP_PI_HALF);
        return 0;  /* x=0, y=0: undefined, return 0 */
    }
    
    if (y == 0)
    {
        return (x > 0) ? 0 : FP_PI;
    }
    
    fixed_point_t angle;
    fixed_point_t abs_y = fp_abs(y);
    fixed_point_t abs_x = fp_abs(x);
    
    if (abs_x >= abs_y)
    {
        /* |x| >= |y|: safe to compute y/x (|ratio| <= 1, no overflow) */
        fixed_point_t ratio = fp_div(y, x);
        angle = fp_atan(ratio);
        
        /* Adjust for quadrant when x < 0 */
        if (x < 0)
        {
            if (y >= 0)
                angle = fp_add(angle, FP_PI);   /* Quadrant II */
            else
                angle = fp_sub(angle, FP_PI);   /* Quadrant III */
        }
    }
    else
    {
        /* |y| > |x|: compute x/y instead to avoid overflow in division.
         * Use identity: atan2(y,x) = sign(y)*π/2 - atan(x/y) */
        fixed_point_t ratio = fp_div(x, y);
        fixed_point_t atan_val = fp_atan(ratio);
        
        if (y > 0)
            angle = fp_sub(FP_PI_HALF, atan_val);
        else
            angle = fp_sub(fp_neg(FP_PI_HALF), atan_val);
    }
    
    return angle;
}

/*===========================================================================
 * Matrix-Vector Operations
 *===========================================================================*/

void fp_mat_vec_mul(
    const fixed_point_t *matrix,
    const fixed_point_t *vec,
    fixed_point_t *result,
    uint16_t rows,
    uint16_t cols)
{
    for (uint16_t r = 0; r < rows; r++)
    {
#ifdef MPC_HLS_TARGET
#pragma HLS LOOP_TRIPCOUNT min=20 max=26 avg=26
#pragma HLS PIPELINE II=1
#endif
        /* Use int64_t accumulator to prevent overflow with large matrices.
         * Critical for the QP Hessian-vector product where 20+ terms are
         * summed, each potentially large in Q16.16 fixed-point. */
        int64_t sum64 = 0;
        for (uint16_t c = 0; c < cols; c++)
        {
#ifdef MPC_HLS_TARGET
#pragma HLS LOOP_TRIPCOUNT min=20 max=26 avg=26
#endif
            int64_t product = (int64_t)matrix[r * cols + c] * vec[c];
#ifdef MPC_HLS_TARGET
#pragma HLS BIND_OP variable=product op=mul impl=dsp
#endif
            sum64 += product >> FP_FRAC_BITS;
        }
        /* Clamp to int32_t range */
        if (sum64 > INT32_MAX) sum64 = INT32_MAX;
        else if (sum64 < INT32_MIN) sum64 = INT32_MIN;
        result[r] = (fixed_point_t)sum64;
    }
}

void fp_symmetric_mat_vec_mul(
    const fixed_point_t *matrix,
    const fixed_point_t *vec,
    fixed_point_t *result,
    uint16_t n)
{
    /* Odd n: fall back to standard multiply (not expected in MPC usage) */
    if (n & 1)
    {
        fp_mat_vec_mul(matrix, vec, result, n, n);
        return;
    }

    /* int64 accumulators prevent overflow during multi-term summation.
     * Size matches QP_MAXIMUM_VARIABLES (46 for FPGA, 80 for CPU). */
#ifndef QP_MAXIMUM_VARIABLES
#define QP_MAXIMUM_VARIABLES 80
#endif
    int64_t accum[QP_MAXIMUM_VARIABLES];
    {
        uint16_t ai;
        for (ai = 0; ai < QP_MAXIMUM_VARIABLES; ai++)
        {
#ifdef MPC_HLS_TARGET
#pragma HLS PIPELINE II=1
#endif
            accum[ai] = 0;
        }
    }

    uint16_t n_blocks = n >> 1;

    for (uint16_t bi = 0; bi < n_blocks; bi++)
    {
#ifdef MPC_HLS_TARGET
#pragma HLS LOOP_TRIPCOUNT min=10 max=13 avg=13
#endif
        uint16_t ri = bi << 1;
        int64_t vi0 = (int64_t)vec[ri];
        int64_t vi1 = (int64_t)vec[ri + 1];

        /* Diagonal 2x2 block: symmetric within block, read 3 of 4 entries.
         * h10 == h01 by symmetry, so we skip reading H[(ri+1)*n + ri]. */
        {
            int64_t h00 = (int64_t)matrix[ri * n + ri];
            int64_t h01 = (int64_t)matrix[ri * n + ri + 1];
            int64_t h11 = (int64_t)matrix[(ri + 1) * n + ri + 1];

            accum[ri]     += (h00 * vi0 >> FP_FRAC_BITS) + (h01 * vi1 >> FP_FRAC_BITS);
            accum[ri + 1] += (h01 * vi0 >> FP_FRAC_BITS) + (h11 * vi1 >> FP_FRAC_BITS);
        }

        /* Off-diagonal 2x2 blocks: read upper-triangle block once,
         * apply to both result[ri:] and result[rj:] via symmetry.
         * Each block read serves double duty, halving H memory accesses. */
        for (uint16_t bj = bi + 1; bj < n_blocks; bj++)
        {
#ifdef MPC_HLS_TARGET
#pragma HLS LOOP_TRIPCOUNT min=0 max=12 avg=6
#pragma HLS PIPELINE II=1
#endif
            uint16_t rj = bj << 1;
            int64_t vj0 = (int64_t)vec[rj];
            int64_t vj1 = (int64_t)vec[rj + 1];

            int64_t a00 = (int64_t)matrix[ri * n + rj];
            int64_t a01 = (int64_t)matrix[ri * n + rj + 1];
            int64_t a10 = (int64_t)matrix[(ri + 1) * n + rj];
            int64_t a11 = (int64_t)matrix[(ri + 1) * n + rj + 1];

            /* Upper-triangle block x v[rj:rj+2] -> accum[ri:ri+2] */
            accum[ri]     += (a00 * vj0 >> FP_FRAC_BITS) + (a01 * vj1 >> FP_FRAC_BITS);
            accum[ri + 1] += (a10 * vj0 >> FP_FRAC_BITS) + (a11 * vj1 >> FP_FRAC_BITS);

            /* Transposed block x v[ri:ri+2] -> accum[rj:rj+2] (symmetry) */
            accum[rj]     += (a00 * vi0 >> FP_FRAC_BITS) + (a10 * vi1 >> FP_FRAC_BITS);
            accum[rj + 1] += (a01 * vi0 >> FP_FRAC_BITS) + (a11 * vi1 >> FP_FRAC_BITS);
        }
    }

    /* Clamp int64 accumulators to int32 range */
    for (uint16_t i = 0; i < n; i++)
    {
#ifdef MPC_HLS_TARGET
#pragma HLS LOOP_TRIPCOUNT min=20 max=26 avg=26
#pragma HLS PIPELINE II=1
#endif
        if (accum[i] > INT32_MAX) accum[i] = INT32_MAX;
        else if (accum[i] < INT32_MIN) accum[i] = INT32_MIN;
        result[i] = (fixed_point_t)accum[i];
    }
}

void fp_vec_add_scaled(
    const fixed_point_t *a,
    const fixed_point_t *b,
    fixed_point_t scalar,
    fixed_point_t *result,
    uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        result[i] = fp_add(a[i], fp_mul(scalar, b[i]));
    }
}

fixed_point_t fp_max_violation(
    const fixed_point_t *A,
    const fixed_point_t *x,
    const fixed_point_t *b,
    uint16_t constraints,
    uint16_t vars)
{
    fixed_point_t max_viol = 0;

    for (uint16_t ci = 0; ci < constraints; ci++)
    {
        fixed_point_t val = 0;
        for (uint16_t vi = 0; vi < vars; vi++)
        {
            val = fp_add(val, fp_mul(A[ci * vars + vi], x[vi]));
        }

        fixed_point_t viol = fp_sub(val, b[ci]);
        if (viol > max_viol)
            max_viol = viol;
    }

    return max_viol;
}
