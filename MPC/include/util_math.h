#ifndef UTIL_MATH_H
#define UTIL_MATH_H

#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*===========================================================================
 * Defines
 *===========================================================================*/

#ifndef QP_MAXIMUM_VARIABLES
#define QP_MAXIMUM_VARIABLES 80
#endif

/*===========================================================================
 * Division with zero check
 *===========================================================================*/

/* Returns 0.0f for b == 0.0f to avoid propagating inf/nan values. */
static inline float util_div(float a, float b)
{
    return (b != 0.0f) ? a / b : 0.0f;
}

/*===========================================================================
 * Clamping
 *===========================================================================*/

static inline float util_clamp(float val, float lo, float hi)
{
    return fminf(fmaxf(val, lo), hi);
}

/*===========================================================================
 * Advanced Math Functions
 *===========================================================================*/

static inline float util_normalize_angle(float angle)
{
    angle = fmodf(angle + M_PI, 2*M_PI);
    if (angle < 0.0f) angle += 2*M_PI;
    return angle - M_PI;
}

/* Returns 0.0f for x == 0.0f to avoid division by zero. */
static inline float util_recip(float x) { return (x != 0.0f) ? 1.0f / x : 0.0f; }

/* Uses absolute input so the result remains real-valued for negative x. */
static inline float util_sqrt(float x)  { return sqrtf(fabsf(x)); }

/*===========================================================================
 * Matrix-Vector Operations (implemented in fp_math.c)
 *===========================================================================*/

/* result[rows] = matrix[rows x cols] * vec[cols] */
void util_mat_vec_mul(
    const float *matrix,
    const float *vec,
    float *result,
    uint16_t rows,
    uint16_t cols);

/*
 * Symmetric matrix-vector multiply optimized for even n.
 * Falls back to fp_mat_vec_mul for odd n.
 */
void util_symmetric_mat_vec_mul(
    const float *matrix,
    const float *vec,
    float *result,
    uint16_t n);

/* result[i] = a[i] + scalar * b[i] */
void util_vec_add_scaled(
    const float *a,
    const float *b,
    float scalar,
    float *result,
    uint16_t len);

/* Maximum positive violation of A*x <= b across all constraints. */
float util_max_violation(
    const float *A,
    const float *x,
    const float *b,
    uint16_t constraints,
    uint16_t vars);

#endif /* UTIL_MATH_H */
