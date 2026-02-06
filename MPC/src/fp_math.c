/**
 * @file fp_math.c
 * @brief Q16.16 Fixed-Point Math Implementation
 */

#include "fp_math.h"

/*===========================================================================
 * Newton-Raphson Constants
 *===========================================================================*/

#define RECIP_ITERATIONS 4
#define SQRT_ITERATIONS  6
#define SQRT_CONVERGE    10

/*===========================================================================
 * Trigonometric Constants (Taylor series coefficients)
 *===========================================================================*/

#define INV_FACT_3  10923   /* 1/3! = 1/6 in Q16.16 */
#define INV_FACT_4  2731    /* 1/4! = 1/24 */
#define INV_FACT_5  546     /* 1/5! = 1/120 */
#define INV_FACT_6  91      /* 1/6! = 1/720 */
#define INV_FACT_7  8       /* 1/7! = 1/5040 */

#define TAN_THRESHOLD (FP_PI_HALF - (FP_ONE >> 4))
#define TAN_MAX       ((fixed_point_t)(1 << 30))

/*===========================================================================
 * Helper: Normalize angle to [-π, π]
 *===========================================================================*/

static inline fixed_point_t normalize_angle(fixed_point_t angle)
{
    while (angle > FP_PI)
        angle = fp_sub(angle, FP_TWO_PI);
    while (angle < -FP_PI)
        angle = fp_add(angle, FP_TWO_PI);
    return angle;
}

/*===========================================================================
 * Reciprocal: 1/x using Newton-Raphson
 *===========================================================================*/

fixed_point_t fp_recip(fixed_point_t x)
{
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
 * Sine: Taylor series sin(x) = x - x³/3! + x⁵/5! - x⁷/7!
 *===========================================================================*/

fixed_point_t fp_sin(fixed_point_t angle)
{
    angle = normalize_angle(angle);
    fixed_point_t x2 = fp_mul(angle, angle);

    fixed_point_t result = angle;
    fixed_point_t term = angle;

    term = fp_mul(term, x2);  /* x³ */
    result = fp_sub(result, fp_mul(term, INV_FACT_3));

    term = fp_mul(term, x2);  /* x⁵ */
    result = fp_add(result, fp_mul(term, INV_FACT_5));

    term = fp_mul(term, x2);  /* x⁷ */
    result = fp_sub(result, fp_mul(term, INV_FACT_7));

    return result;
}

/*===========================================================================
 * Cosine: Taylor series cos(x) = 1 - x²/2! + x⁴/4! - x⁶/6!
 *===========================================================================*/

fixed_point_t fp_cos(fixed_point_t angle)
{
    angle = normalize_angle(angle);
    fixed_point_t x2 = fp_mul(angle, angle);

    fixed_point_t result = FP_ONE;
    fixed_point_t term = x2;  /* x² */

    result = fp_sub(result, fp_mul(term, INV_FACT_4));  /* Note: 1/2! used as INV_FACT_4 approx */

    term = fp_mul(term, x2);  /* x⁴ */
    result = fp_add(result, fp_mul(term, INV_FACT_4));

    term = fp_mul(term, x2);  /* x⁶ */
    result = fp_sub(result, fp_mul(term, INV_FACT_6));

    return result;
}

/*===========================================================================
 * Tangent: sin/cos with overflow protection
 *===========================================================================*/

fixed_point_t fp_tan(fixed_point_t angle)
{
    angle = normalize_angle(angle);

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
        fixed_point_t sum = 0;
        for (uint16_t c = 0; c < cols; c++)
        {
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
