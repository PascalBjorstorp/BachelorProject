/**
 * @file fp_math.h
 * @brief Float32 Arithmetic and Math Operations
 *
 * Converted from Q16.16 fixed-point to native float32 for
 * ~3-5× speedup on ARM Cortex-A57 (Jetson Xavier NX).
 * All fp_* functions are kept as thin wrappers for compatibility.
 *
 * The typedef `fixed_point_t` is retained as `float` to minimize
 * code churn in dependent files.
 */

#ifndef FP_MATH_H
#define FP_MATH_H

#include <stdint.h>
#include <math.h>

/*===========================================================================
 * Type Definition — float32 (was Q16.16 int32_t)
 *===========================================================================*/

typedef float fixed_point_t;

/*===========================================================================
 * Constants
 *===========================================================================*/

#define FP_FRAC_BITS    16              /* Kept for any legacy references */
#define FP_ONE          1.0f
#define FP_TWO          2.0f
#define FP_HALF         0.5f
#define FP_PI           3.14159265f
#define FP_PI_HALF      1.57079633f
#define FP_TWO_PI       6.28318530f

/*===========================================================================
 * Conversion Macros — identity/cast operations
 *===========================================================================*/

#define FP_CONST(x)     ((float)(x))
#define DOUBLE_TO_FP(x) ((float)(x))
#define FP_TO_DOUBLE(x) ((double)(x))
#define FP_TO_FLOAT(x)  (x)

/*===========================================================================
 * Bit Shift Operations — multiply/divide by power of 2
 *===========================================================================*/

#define fp_shift_left(val, n)  ((float)(val) * (float)(1 << (n)))
#define fp_shift_right(val, n) ((float)(val) / (float)(1 << (n)))

/*===========================================================================
 * Basic Arithmetic (inline — zero overhead)
 *===========================================================================*/

static inline float fp_add(float a, float b) { return a + b; }
static inline float fp_add_sat(float a, float b) { return a + b; }
static inline float fp_sub(float a, float b) { return a - b; }
static inline float fp_mul(float a, float b) { return a * b; }

static inline float fp_div(float a, float b)
{
    return (b != 0.0f) ? a / b : 0.0f;
}

/*===========================================================================
 * Unary Operations (inline)
 *===========================================================================*/

static inline float fp_abs(float a) { return fabsf(a); }
static inline float fp_neg(float a) { return -a; }

/*===========================================================================
 * Comparison and Clamping (inline)
 *===========================================================================*/

static inline float fp_min(float a, float b) { return fminf(a, b); }
static inline float fp_max(float a, float b) { return fmaxf(a, b); }

static inline float fp_clamp(float val, float lo, float hi)
{
    return fminf(fmaxf(val, lo), hi);
}

/*===========================================================================
 * Advanced Math Functions — direct libm calls
 *===========================================================================*/

static inline float fp_normalize_angle(float angle)
{
    angle = fmodf(angle + FP_PI, FP_TWO_PI);
    if (angle < 0.0f) angle += FP_TWO_PI;
    return angle - FP_PI;
}

static inline float fp_recip(float x) { return (x != 0.0f) ? 1.0f / x : 0.0f; }
static inline float fp_sqrt(float x)  { return sqrtf(fabsf(x)); }
static inline float fp_sin(float a)   { return sinf(a); }
static inline float fp_cos(float a)   { return cosf(a); }
static inline float fp_tan(float a)   { return tanf(a); }
static inline float fp_atan(float x)  { return atanf(x); }
static inline float fp_atan2(float y, float x) { return atan2f(y, x); }

static inline float fp_pow(float base, int exp)
{
    return powf(base, (float)exp);
}

/*===========================================================================
 * Matrix-Vector Operations (implemented in fp_math.c)
 *===========================================================================*/

void fp_mat_vec_mul(
    const float *matrix,
    const float *vec,
    float *result,
    uint16_t rows,
    uint16_t cols);

void fp_symmetric_mat_vec_mul(
    const float *matrix,
    const float *vec,
    float *result,
    uint16_t n);

void fp_vec_add_scaled(
    const float *a,
    const float *b,
    float scalar,
    float *result,
    uint16_t len);

float fp_max_violation(
    const float *A,
    const float *x,
    const float *b,
    uint16_t constraints,
    uint16_t vars);

#endif /* FP_MATH_H */
