/**
 * @file util_math.h
 * @brief Shared scalar, vector, and matrix math utilities.
 * @details Provides platform-independent fixed-size math primitives used
 *          throughout the MPC pipeline: safe division, clamping, angle
 *          normalization, matrix-vector multiplication, and constraint
 *          violation checking. All scalar functions are static inline for
 *          zero-overhead inlining into callers.
 * @dependencies <stdint.h>, <math.h>
 */
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

/* Returns zero for a zero denominator to prevent propagation of non-finite values. */
/**
 * @brief Perform safe scalar division, returning zero for a zero denominator.
 * @param a Dividend (numerator).
 * @param b Divisor (denominator).
 * @return Quotient a/b, or zero when b is zero to prevent non-finite propagation.
 */
static inline float util_div(float a, float b)
{
    return (b != 0.0f) ? a / b : 0.0f;
}

/*===========================================================================
 * Clamping
 *===========================================================================*/

/**
 * @brief Clamp a scalar value to a closed interval.
 * @param val Input value to constrain.
 * @param lo Lower bound of the interval (inclusive).
 * @param hi Upper bound of the interval (inclusive).
 * @return val constrained to [lo, hi].
 */
static inline float util_clamp(float val, float lo, float hi)
{
    return fminf(fmaxf(val, lo), hi);
}

/*===========================================================================
 * Advanced Math Functions
 *===========================================================================*/

/**
 * @brief Wrap an angle to the canonical interval (-pi, pi].
 * @param angle Raw angle value [radians], any finite magnitude.
 * @return Equivalent angle in (-pi, pi] [radians].
 */
static inline float util_normalize_angle(float angle)
{
    angle = fmodf(angle + M_PI, 2*M_PI);
    if (angle < 0.0f) angle += 2*M_PI;
    return angle - M_PI;
}

/**
 * @brief Compute the multiplicative reciprocal, returning zero for a zero input.
 * @param x Input scalar.
 * @return Reciprocal 1/x, or zero when x is zero to prevent non-finite propagation.
 */
static inline float util_recip(float x) { return (x != 0.0f) ? 1.0f / x : 0.0f; }

/**
 * @brief Compute the square root of the absolute value of the input.
 * @param x Input scalar (sign is ignored to keep the result real-valued).
 * @return Non-negative square root of |x|.
 */
static inline float util_sqrt(float x)  { return sqrtf(fabsf(x)); }

/*===========================================================================
 * Matrix-Vector Operations (implemented in util_math.c)
 *===========================================================================*/

/**
 * @brief Multiply a dense row-major matrix by a column vector.
 * @param matrix Pointer to row-major matrix data [rows × cols floats].
 * @param vec Input column vector [cols floats].
 * @param result Output vector [rows floats]; must not alias matrix or vec.
 * @param rows Number of matrix rows.
 * @param cols Number of matrix columns.
 * @return None.
 */
void util_mat_vec_mul(
    const float *matrix,
    const float *vec,
    float *result,
    uint16_t rows,
    uint16_t cols);

/**
 * @brief Multiply a symmetric square matrix by a vector using 2x2 block tiling.
 * @details Exploits symmetry to halve memory accesses for even-dimension matrices.
 *          Falls back to the generic multiply for odd dimensions.
 * @param matrix Pointer to the upper-triangular-accessible symmetric matrix [n x n].
 * @param vec Input vector [n floats].
 * @param result Output vector [n floats]; must not alias matrix or vec.
 * @param n Matrix dimension; must be <= QP_MAXIMUM_VARIABLES.
 * @return None.
 */
void util_symmetric_mat_vec_mul(
    const float *matrix,
    const float *vec,
    float *result,
    uint16_t n);

/**
 * @brief Compute the scaled vector sum result[i] = a[i] + scalar * b[i].
 * @param a First operand vector [len floats].
 * @param b Second operand vector [len floats].
 * @param scalar Scale factor applied to b before addition.
 * @param result Output vector [len floats]; may alias a but must not alias b.
 * @param len Vector length.
 * @return None.
 */
void util_vec_add_scaled(
    const float *a,
    const float *b,
    float scalar,
    float *result,
    uint16_t len);

/**
 * @brief Compute the maximum positive constraint violation of A*x <= b.
 * @param A Constraint matrix [constraints x vars], row-major.
 * @param x Primal variable vector [vars floats].
 * @param b Right-hand side vector [constraints floats].
 * @param constraints Number of inequality constraints (rows of A).
 * @param vars Number of variables (columns of A).
 * @return Maximum violation max_i(A_i * x - b_i) over all rows;
 *         zero if all constraints are satisfied.
 */
float util_max_violation(
    const float *A,
    const float *x,
    const float *b,
    uint16_t constraints,
    uint16_t vars);

#endif /* UTIL_MATH_H */
