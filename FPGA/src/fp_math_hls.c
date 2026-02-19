/**
 * @file fp_math_hls.c
 * @brief Q16.16 Fixed-Point Math Implementation (HLS Version)
 *
 * This file is identical to MPC/src/fp_math.c but includes HLS pragmas
 * for FPGA synthesis.
 */

#include "../include/fp_math_hls.h"

/*===========================================================================
 * Newton-Raphson Constants
 *===========================================================================*/

#define RECIP_ITERATIONS 4
#define SQRT_ITERATIONS  6
#define SQRT_CONVERGE    10

/*===========================================================================
 * Trigonometric Constants (Taylor series coefficients)
 *===========================================================================*/

#define INV_FACT_2  32768   /* 1/2! = 1/2 in Q16.16 */
#define INV_FACT_3  10923   /* 1/3! = 1/6 in Q16.16 */
#define INV_FACT_4  2731    /* 1/4! = 1/24 in Q16.16 */
#define INV_FACT_5  546     /* 1/5! = 1/120 in Q16.16 */
#define INV_FACT_6  91      /* 1/6! = 1/720 in Q16.16 */
#define INV_FACT_7  13      /* 1/7! = 1/5040 in Q16.16 */
#define INV_FACT_8  2       /* 1/8! = 1/40320 in Q16.16 */
#define INV_FACT_9  0       /* 1/9! = 1/362880 ≈ 0 in Q16.16 (too small) */

#define TAN_THRESHOLD (FP_PI_HALF - (FP_ONE >> 4))
#define TAN_MAX       ((fixed_point_t)(1 << 30))

/*===========================================================================
 * Helper: Normalize angle to [-π, π]
 * 
 * HLS-friendly version with bounded iterations.
 * Max 4 iterations handles angles up to ±4π (beyond typical range).
 *===========================================================================*/

#define NORMALIZE_MAX_ITERS 4

fixed_point_t fp_normalize_angle(fixed_point_t angle)
{
#pragma HLS INLINE
    
    // Handle positive angles > π
    NORM_POS: for (int i = 0; i < NORMALIZE_MAX_ITERS; i++) {
        #pragma HLS UNROLL
        if (angle > FP_PI) {
            angle = fp_sub(angle, FP_TWO_PI);
        }
    }
    
    // Handle negative angles < -π
    NORM_NEG: for (int i = 0; i < NORMALIZE_MAX_ITERS; i++) {
        #pragma HLS UNROLL
        if (angle < -FP_PI) {
            angle = fp_add(angle, FP_TWO_PI);
        }
    }
    
    return angle;
}

/*===========================================================================
 * Reciprocal: 1/x using Newton-Raphson
 *===========================================================================*/

fixed_point_t fp_recip(fixed_point_t x)
{
#pragma HLS INLINE off
    if (x == 0) return 0;

    int32_t sign = (x < 0) ? -1 : 1;
    fixed_point_t abs_x = fp_abs(x);

    /* Initial guess via leading zeros */
    int lead_zeros = 0;
    int32_t temp = abs_x;
    while (!(temp & 0x40000000) && lead_zeros < 31)
    {
#pragma HLS LOOP_TRIPCOUNT min=1 max=31
        temp <<= 1;
        lead_zeros++;
    }

    fixed_point_t est = (fixed_point_t)(1 << lead_zeros);

    /* Newton-Raphson: est = est + est*(1 - x*est) */
    for (int i = 0; i < RECIP_ITERATIONS; i++)
    {
#pragma HLS PIPELINE II=1
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
#pragma HLS INLINE off
    if (x <= 0) return 0;

    /* Initial guess */
    int lead_zeros = 0;
    int32_t temp = x;
    while (!(temp & 0x40000000) && lead_zeros < 31)
    {
#pragma HLS LOOP_TRIPCOUNT min=1 max=31
        temp <<= 1;
        lead_zeros++;
    }

    fixed_point_t y = (fixed_point_t)(1 << (9 + (lead_zeros >> 1)));
    fixed_point_t three = FP_CONST(3.0);

    /* Newton-Raphson for inverse sqrt: y = (y/2) * (3 - x*y²) */
    for (int i = 0; i < SQRT_ITERATIONS; i++)
    {
#pragma HLS PIPELINE II=1
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
 * Sine with range reduction
 * 
 * sin(x) = sin(x) for |x| <= π/2
 * sin(x) = sin(π - x) for π/2 < x <= π
 * sin(x) = -sin(-x) for x < 0
 * 
 * Taylor series: sin(x) = x - x³/3! + x⁵/5! - x⁷/7!
 * Accurate to < 0.1% for |x| <= π/2
 *===========================================================================*/

/* Internal helper: Taylor series for small angle */
static fixed_point_t fp_sin_taylor(fixed_point_t x)
{
#pragma HLS INLINE
    fixed_point_t x2 = fp_mul(x, x);
    fixed_point_t term = x;
    fixed_point_t result = x;
    
    term = fp_mul(term, x2);  /* x³ */
    result = fp_sub(result, fp_mul(term, INV_FACT_3));
    
    term = fp_mul(term, x2);  /* x⁵ */
    result = fp_add(result, fp_mul(term, INV_FACT_5));
    
    term = fp_mul(term, x2);  /* x⁷ */
    result = fp_sub(result, fp_mul(term, INV_FACT_7));
    
    return result;
}

fixed_point_t fp_sin(fixed_point_t angle)
{
#pragma HLS INLINE
    angle = fp_normalize_angle(angle);
    
    /* Reduce to first quadrant: |angle| <= π/2 */
    int32_t sign = 1;
    if (angle < 0) {
        sign = -1;
        angle = fp_neg(angle);
    }
    
    /* If angle > π/2, use sin(π - x) = sin(x) */
    if (angle > FP_PI_HALF) {
        angle = fp_sub(FP_PI, angle);
    }
    
    fixed_point_t result = fp_sin_taylor(angle);
    return (sign < 0) ? fp_neg(result) : result;
}

/*===========================================================================
 * Cosine with range reduction
 * 
 * cos(x) = cos(|x|)  (even function)
 * cos(x) = sin(π/2 - x) for x <= π/2
 * cos(x) = -sin(x - π/2) for x > π/2
 *===========================================================================*/

fixed_point_t fp_cos(fixed_point_t angle)
{
#pragma HLS INLINE
    angle = fp_normalize_angle(angle);
    
    /* cos is even: cos(-x) = cos(x) */
    if (angle < 0) {
        angle = fp_neg(angle);
    }
    
    /* cos(x) = sin(π/2 - x) */
    if (angle <= FP_PI_HALF) {
        return fp_sin_taylor(fp_sub(FP_PI_HALF, angle));
    } else {
        /* cos(x) = -sin(x - π/2) for π/2 < x <= π */
        return fp_neg(fp_sin_taylor(fp_sub(angle, FP_PI_HALF)));
    }
}

/*===========================================================================
 * Tangent: sin/cos with overflow protection
 *===========================================================================*/

fixed_point_t fp_tan(fixed_point_t angle)
{
#pragma HLS INLINE off
    angle = fp_normalize_angle(angle);

    /* Protect against overflow near ±π/2 */
    if (fp_abs(fp_sub(angle, FP_PI_HALF)) < TAN_THRESHOLD)
        return (angle > 0) ? TAN_MAX : fp_neg(TAN_MAX);
    if (fp_abs(fp_add(angle, FP_PI_HALF)) < TAN_THRESHOLD)
        return (angle > 0) ? TAN_MAX : fp_neg(TAN_MAX);

    return fp_div(fp_sin(angle), fp_cos(angle));
}

/*===========================================================================
 * Integer Power: base^exponent
 *===========================================================================*/

fixed_point_t fp_pow(fixed_point_t base, int exp)
{
#pragma HLS INLINE off
    if (exp == 0) return FP_ONE;
    if (exp == 1) return base;
    if (exp == -1) return fp_recip(base);
    if (exp == 2) return fp_mul(base, base);

    fixed_point_t result = FP_ONE;
    int abs_exp = (exp < 0) ? -exp : exp;
    fixed_point_t curr = base;

    while (abs_exp > 0)
    {
#pragma HLS LOOP_TRIPCOUNT min=1 max=16
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
 * Arctangent using range reduction and polynomial approximation
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
#pragma HLS INLINE
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
#pragma HLS INLINE off
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
#pragma HLS INLINE off
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
    
    /* Compute atan(y/x) */
    fixed_point_t ratio = fp_div(y, x);
    angle = fp_atan(ratio);
    
    /* Adjust for quadrant */
    if (x < 0)
    {
        if (y >= 0)
            angle = fp_add(angle, FP_PI);   /* Quadrant II */
        else
            angle = fp_sub(angle, FP_PI);   /* Quadrant III */
    }
    /* Quadrant I and IV are already correct */
    
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
#pragma HLS INLINE off
    for (uint16_t r = 0; r < rows; r++)
    {
#pragma HLS LOOP_TRIPCOUNT min=1 max=64
        fixed_point_t sum = 0;
        for (uint16_t c = 0; c < cols; c++)
        {
#pragma HLS LOOP_TRIPCOUNT min=1 max=64
#pragma HLS PIPELINE II=1
            sum = fp_add(sum, fp_mul(matrix[r * cols + c], vec[c]));
        }
        result[r] = sum;
    }
}

void fp_vec_add_scaled(
    const fixed_point_t *a,
    const fixed_point_t *b,
    fixed_point_t scalar,
    fixed_point_t *result,
    uint16_t len)
{
#pragma HLS INLINE off
    for (uint16_t i = 0; i < len; i++)
    {
#pragma HLS LOOP_TRIPCOUNT min=1 max=128
#pragma HLS PIPELINE II=1
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
#pragma HLS INLINE off
    fixed_point_t max_viol = 0;

    for (uint16_t ci = 0; ci < constraints; ci++)
    {
#pragma HLS LOOP_TRIPCOUNT min=1 max=32
        fixed_point_t val = 0;
        for (uint16_t vi = 0; vi < vars; vi++)
        {
#pragma HLS LOOP_TRIPCOUNT min=1 max=32
#pragma HLS PIPELINE II=1
            val = fp_add(val, fp_mul(A[ci * vars + vi], x[vi]));
        }

        fixed_point_t viol = fp_sub(val, b[ci]);
        if (viol > max_viol)
            max_viol = viol;
    }

    return max_viol;
}
