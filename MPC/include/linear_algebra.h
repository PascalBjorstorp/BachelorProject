/**
 * @file linear_algebra.h
 * @brief Fixed-Point Linear Algebra Operations for MPC Solver
 *
 * Provides matrix and vector operations using fixed-point arithmetic.
 * All operations are FPGA-compatible with no dynamic memory allocation.
 *
 * Matrix storage: Row-major order
 * - Element (i,j) of matrix A with 'columns' columns is at: A[i * columns + j]
 *
 * @note All functions operate on pre-allocated arrays.
 *       Caller is responsible for ensuring correct dimensions.
 */

#ifndef LINEAR_ALGEBRA_H
#define LINEAR_ALGEBRA_H

#include "fixed_point.h"
#include <stdint.h>

/*===========================================================================
 * Matrix-Vector Operations
 *===========================================================================*/

/**
 * Matrix-vector product: result_vector = matrix × input_vector
 *
 * Computes y = A × x where:
 * - A is a (row_count × column_count) matrix in row-major order
 * - x is a vector of length column_count
 * - y is a vector of length row_count
 *
 * @param matrix        Input matrix A (row_count × column_count), row-major
 * @param input_vector  Input vector x (length column_count)
 * @param result_vector Output vector y (length row_count)
 * @param row_count     Number of rows in matrix
 * @param column_count  Number of columns in matrix
 */
void linear_algebra_matrix_vector_multiply(
    const fixed_point_t *matrix,
    const fixed_point_t *input_vector,
    fixed_point_t *result_vector,
    uint16_t row_count,
    uint16_t column_count);

/**
 * Matrix-matrix product: result_matrix = matrix_a × matrix_b
 *
 * Computes C = A × B where:
 * - A is (rows_a × cols_a_rows_b) matrix
 * - B is (cols_a_rows_b × cols_b) matrix
 * - C is (rows_a × cols_b) matrix
 *
 * @param matrix_a          First input matrix
 * @param matrix_b          Second input matrix
 * @param result_matrix     Output matrix
 * @param rows_a            Rows in matrix A (and result)
 * @param cols_a_rows_b     Columns in A / Rows in B (shared dimension)
 * @param cols_b            Columns in matrix B (and result)
 */
void linear_algebra_matrix_matrix_multiply(
    const fixed_point_t *matrix_a,
    const fixed_point_t *matrix_b,
    fixed_point_t *result_matrix,
    uint16_t rows_a,
    uint16_t cols_a_rows_b,
    uint16_t cols_b);

/*===========================================================================
 * Vector Operations
 *===========================================================================*/

/**
 * Vector dot product: result = vector_a · vector_b
 *
 * @param vector_a  First input vector
 * @param vector_b  Second input vector
 * @param length    Number of elements in vectors
 * @return Scalar dot product
 */
fixed_point_t linear_algebra_dot_product(
    const fixed_point_t *vector_a,
    const fixed_point_t *vector_b,
    uint16_t length);

/**
 * Vector L2 norm: result = ||vector||₂ = √(Σ vector[i]²)
 *
 * @param vector  Input vector
 * @param length  Number of elements
 * @return Euclidean norm of vector
 */
fixed_point_t linear_algebra_vector_norm(
    const fixed_point_t *vector,
    uint16_t length);

/**
 * Vector addition with scalar: result = vector_a + scalar × vector_b
 *
 * This is the AXPY operation (a*x plus y).
 *
 * @param vector_a       First input vector (also base for result)
 * @param vector_b       Second input vector (to be scaled and added)
 * @param scalar         Scalar multiplier for vector_b
 * @param result_vector  Output vector
 * @param length         Number of elements
 */
void linear_algebra_vector_add_scaled(
    const fixed_point_t *vector_a,
    const fixed_point_t *vector_b,
    fixed_point_t scalar,
    fixed_point_t *result_vector,
    uint16_t length);

/**
 * Vector scaling: result_vector = scalar × input_vector
 *
 * @param input_vector   Input vector
 * @param scalar         Scalar multiplier
 * @param result_vector  Output vector
 * @param length         Number of elements
 */
void linear_algebra_vector_scale(
    const fixed_point_t *input_vector,
    fixed_point_t scalar,
    fixed_point_t *result_vector,
    uint16_t length);

/*===========================================================================
 * Constraint Operations (for QP Solver)
 *===========================================================================*/

/**
 * Compute maximum constraint violation: max(0, A×x - b)
 *
 * For inequality constraints A×x ≤ b, computes the largest violation.
 * Returns 0 if all constraints are satisfied.
 *
 * @param constraint_matrix  Matrix A (constraint_count × variable_count)
 * @param variable_vector    Vector x (length variable_count)
 * @param bound_vector       Vector b (length constraint_count)
 * @param constraint_count   Number of constraints (rows in A)
 * @param variable_count     Number of variables (columns in A)
 * @return Maximum positive violation, or 0 if feasible
 */
fixed_point_t linear_algebra_max_constraint_violation(
    const fixed_point_t *constraint_matrix,
    const fixed_point_t *variable_vector,
    const fixed_point_t *bound_vector,
    uint16_t constraint_count,
    uint16_t variable_count);

/*===========================================================================
 * Element-wise Clamping
 *===========================================================================*/

/**
 * Clamp vector elements to range: result[i] = clamp(input[i], lower, upper)
 *
 * @param input_vector   Input vector
 * @param lower_bound    Lower bound (scalar, applied to all elements)
 * @param upper_bound    Upper bound (scalar, applied to all elements)
 * @param result_vector  Output vector
 * @param length         Number of elements
 */
void linear_algebra_clamp_vector_scalar(
    const fixed_point_t *input_vector,
    fixed_point_t lower_bound,
    fixed_point_t upper_bound,
    fixed_point_t *result_vector,
    uint16_t length);

/**
 * Clamp vector elements to per-element bounds
 *
 * @param input_vector        Input vector
 * @param lower_bound_vector  Lower bounds (one per element)
 * @param upper_bound_vector  Upper bounds (one per element)
 * @param result_vector       Output vector
 * @param length              Number of elements
 */
void linear_algebra_clamp_vector(
    const fixed_point_t *input_vector,
    const fixed_point_t *lower_bound_vector,
    const fixed_point_t *upper_bound_vector,
    fixed_point_t *result_vector,
    uint16_t length);

#endif /* LINEAR_ALGEBRA_H */
