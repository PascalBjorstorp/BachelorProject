/**
 * @file linear_algebra.c
 * @brief Fixed-Point Linear Algebra Implementation
 *
 * Implements matrix and vector operations using fixed-point arithmetic.
 * All operations use only integer arithmetic for FPGA compatibility.
 */

#include "linear_algebra.h"
#include <string.h>

/*===========================================================================
 * Matrix-Vector Operations
 *===========================================================================*/

void linear_algebra_matrix_vector_multiply(
    const fixed_point_t *matrix,
    const fixed_point_t *input_vector,
    fixed_point_t *result_vector,
    uint16_t row_count,
    uint16_t column_count)
{
    for (uint16_t row_index = 0; row_index < row_count; row_index++)
    {
        fixed_point_t row_sum = 0;

        for (uint16_t column_index = 0; column_index < column_count; column_index++)
        {
            /* Access element (row_index, column_index) in row-major matrix */
            uint16_t matrix_index = row_index * column_count + column_index;
            fixed_point_t product = fixed_point_mul(
                matrix[matrix_index],
                input_vector[column_index]);
            row_sum = fixed_point_add(row_sum, product);
        }

        result_vector[row_index] = row_sum;
    }
}

void linear_algebra_matrix_matrix_multiply(
    const fixed_point_t *matrix_a,
    const fixed_point_t *matrix_b,
    fixed_point_t *result_matrix,
    uint16_t rows_a,
    uint16_t cols_a_rows_b,
    uint16_t cols_b)
{
    for (uint16_t row_index = 0; row_index < rows_a; row_index++)
    {
        for (uint16_t col_index = 0; col_index < cols_b; col_index++)
        {
            fixed_point_t element_sum = 0;

            for (uint16_t inner_index = 0; inner_index < cols_a_rows_b; inner_index++)
            {
                /* A[row_index, inner_index] × B[inner_index, col_index] */
                uint16_t index_a = row_index * cols_a_rows_b + inner_index;
                uint16_t index_b = inner_index * cols_b + col_index;

                fixed_point_t product = fixed_point_mul(
                    matrix_a[index_a],
                    matrix_b[index_b]);
                element_sum = fixed_point_add(element_sum, product);
            }

            /* Result[row_index, col_index] */
            uint16_t result_index = row_index * cols_b + col_index;
            result_matrix[result_index] = element_sum;
        }
    }
}

/*===========================================================================
 * Vector Operations
 *===========================================================================*/

fixed_point_t linear_algebra_dot_product(
    const fixed_point_t *vector_a,
    const fixed_point_t *vector_b,
    uint16_t length)
{
    fixed_point_t sum = 0;

    for (uint16_t index = 0; index < length; index++)
    {
        fixed_point_t product = fixed_point_mul(vector_a[index], vector_b[index]);
        sum = fixed_point_add(sum, product);
    }

    return sum;
}

fixed_point_t linear_algebra_vector_norm(
    const fixed_point_t *vector,
    uint16_t length)
{
    fixed_point_t sum_of_squares = 0;

    for (uint16_t index = 0; index < length; index++)
    {
        fixed_point_t square = fixed_point_mul(vector[index], vector[index]);
        sum_of_squares = fixed_point_add(sum_of_squares, square);
    }

    return fixed_point_sqrt(sum_of_squares);
}

void linear_algebra_vector_add_scaled(
    const fixed_point_t *vector_a,
    const fixed_point_t *vector_b,
    fixed_point_t scalar,
    fixed_point_t *result_vector,
    uint16_t length)
{
    for (uint16_t index = 0; index < length; index++)
    {
        /* result[i] = a[i] + scalar × b[i] */
        fixed_point_t scaled_b = fixed_point_mul(scalar, vector_b[index]);
        result_vector[index] = fixed_point_add(vector_a[index], scaled_b);
    }
}

void linear_algebra_vector_scale(
    const fixed_point_t *input_vector,
    fixed_point_t scalar,
    fixed_point_t *result_vector,
    uint16_t length)
{
    for (uint16_t index = 0; index < length; index++)
    {
        result_vector[index] = fixed_point_mul(scalar, input_vector[index]);
    }
}

/*===========================================================================
 * Constraint Operations
 *===========================================================================*/

fixed_point_t linear_algebra_max_constraint_violation(
    const fixed_point_t *constraint_matrix,
    const fixed_point_t *variable_vector,
    const fixed_point_t *bound_vector,
    uint16_t constraint_count,
    uint16_t variable_count)
{
    fixed_point_t maximum_violation = 0;

    for (uint16_t constraint_index = 0; constraint_index < constraint_count; constraint_index++)
    {
        /* Compute A[constraint_index, :] × x */
        fixed_point_t constraint_value = 0;

        for (uint16_t variable_index = 0; variable_index < variable_count; variable_index++)
        {
            uint16_t matrix_index = constraint_index * variable_count + variable_index;
            fixed_point_t product = fixed_point_mul(
                constraint_matrix[matrix_index],
                variable_vector[variable_index]);
            constraint_value = fixed_point_add(constraint_value, product);
        }

        /* Compute violation: A×x - b (positive means violated) */
        fixed_point_t violation = fixed_point_sub(
            constraint_value,
            bound_vector[constraint_index]);

        /* Only positive violations count (constraint is A×x ≤ b) */
        if (violation > maximum_violation)
        {
            maximum_violation = violation;
        }
    }

    return maximum_violation;
}

/*===========================================================================
 * Element-wise Clamping
 *===========================================================================*/

void linear_algebra_clamp_vector_scalar(
    const fixed_point_t *input_vector,
    fixed_point_t lower_bound,
    fixed_point_t upper_bound,
    fixed_point_t *result_vector,
    uint16_t length)
{
    for (uint16_t index = 0; index < length; index++)
    {
        result_vector[index] = fixed_point_clamp(
            input_vector[index],
            lower_bound,
            upper_bound);
    }
}

void linear_algebra_clamp_vector(
    const fixed_point_t *input_vector,
    const fixed_point_t *lower_bound_vector,
    const fixed_point_t *upper_bound_vector,
    fixed_point_t *result_vector,
    uint16_t length)
{
    for (uint16_t index = 0; index < length; index++)
    {
        fixed_point_t value = input_vector[index];

        if (value < lower_bound_vector[index])
        {
            value = lower_bound_vector[index];
        }
        if (value > upper_bound_vector[index])
        {
            value = upper_bound_vector[index];
        }

        result_vector[index] = value;
    }
}
