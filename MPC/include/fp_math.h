/**
 * @file fp_math.h
 * @brief Q16.16 Fixed-Point Arithmetic and Math Operations
 *
 * Format: Q16.16
 * - 16 bits integer, 16 bits fractional
 * - Range: -32768.0 to 32767.99998
 * - Precision: ~0.000015 (1/65536)
 *
 * All operations use integer arithmetic (FPGA compatible).
 */

#ifndef FP_MATH_H
#define FP_MATH_H

#include <stdint.h>

/*===========================================================================
 * Type Definition
 *===========================================================================*/

typedef int32_t fixed_point_t;

/*===========================================================================
 * Constants
 *===========================================================================*/

#define FP_FRAC_BITS    16
#define FP_ONE          (1 << FP_FRAC_BITS)
#define FP_TWO          (2 << FP_FRAC_BITS)
#define FP_HALF         (FP_ONE >> 1)
#define FP_PI           205887      /* π in Q16.16 */
#define FP_PI_HALF      102943      /* π/2 in Q16.16 */
#define FP_TWO_PI       411775      /* 2π in Q16.16 */

/*===========================================================================
 * Conversion Macros
 *===========================================================================*/

/** Compile-time float → Q16.16 */
#define FP_CONST(x)     ((fixed_point_t)((double)(x) * FP_ONE))

/** Runtime double → Q16.16 */
#define DOUBLE_TO_FP(x) ((fixed_point_t)((x) * FP_ONE))

/** Runtime Q16.16 → double */
#define FP_TO_DOUBLE(x) ((double)(x) / (double)FP_ONE)

/** Runtime Q16.16 → float */
#define FP_TO_FLOAT(x)  ((float)(x) / (float)FP_ONE)

/*===========================================================================
 * Bit Shift Operations
 *===========================================================================*/

#define fp_shift_left(val, n)  ((fixed_point_t)((int64_t)(val) << (n)))
#define fp_shift_right(val, n) ((fixed_point_t)((int64_t)(val) >> (n)))

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
    return (fixed_point_t)((int64_t)a * b >> FP_FRAC_BITS);
}

static inline fixed_point_t fp_div(fixed_point_t a, fixed_point_t b)
{
    if (a == 0) return 0;
    return (fixed_point_t)(((int64_t)a << FP_FRAC_BITS) / b);
}

/*===========================================================================
 * Unary Operations (inline)
 *===========================================================================*/

static inline fixed_point_t fp_abs(fixed_point_t a)
{
    return (a < 0) ? -a : a;
}

static inline fixed_point_t fp_neg(fixed_point_t a)
{
    return -a;
}

/*===========================================================================
 * Comparison and Clamping (inline)
 *===========================================================================*/

static inline fixed_point_t fp_min(fixed_point_t a, fixed_point_t b)
{
    return (a < b) ? a : b;
}

static inline fixed_point_t fp_max(fixed_point_t a, fixed_point_t b)
{
    return (a > b) ? a : b;
}

static inline fixed_point_t fp_clamp(fixed_point_t val, fixed_point_t lo, fixed_point_t hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/*===========================================================================
 * Advanced Math Functions (implemented in fp_math.c)
 *===========================================================================*/

/** Reciprocal: 1/x using Newton-Raphson */
fixed_point_t fp_recip(fixed_point_t x);

/** Square root using Newton-Raphson */
fixed_point_t fp_sqrt(fixed_point_t x);

/** Sine using Taylor series */
fixed_point_t fp_sin(fixed_point_t angle);

/** Cosine using Taylor series */
fixed_point_t fp_cos(fixed_point_t angle);

/** Tangent: sin/cos with overflow protection */
fixed_point_t fp_tan(fixed_point_t angle);

/** Integer power: base^exponent */
fixed_point_t fp_pow(fixed_point_t base, int exp);

/*===========================================================================
 * Matrix-Vector Operations
 *===========================================================================*/

/** Matrix-vector multiply: result = M × v */
void fp_mat_vec_mul(
    const fixed_point_t *matrix,
    const fixed_point_t *vec,
    fixed_point_t *result,
    uint16_t rows,
    uint16_t cols);

/** Vector add scaled: result = a + scalar × b */
void fp_vec_add_scaled(
    const fixed_point_t *a,
    const fixed_point_t *b,
    fixed_point_t scalar,
    fixed_point_t *result,
    uint16_t len);

/** Max constraint violation: max(A×x - b, 0) */
fixed_point_t fp_max_violation(
    const fixed_point_t *A,
    const fixed_point_t *x,
    const fixed_point_t *b,
    uint16_t constraints,
    uint16_t vars);

#endif /* FP_MATH_H */
