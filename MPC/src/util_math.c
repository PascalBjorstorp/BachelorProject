/**
 * @file util_math.c
 * @brief Implementation of non-inline math utility functions.
 * @details Implements matrix-vector multiplication, scaled vector addition,
 *          and constraint violation checking declared in util_math.h.
 *          Inline utilities are defined entirely in the header.
 * @dependencies util_math.h, <string.h>, <stdint.h>
 */
#include "util_math.h"

/*===========================================================================
 * Matrix-Vector Operations
 *===========================================================================*/

void util_mat_vec_mul(
    const float *matrix,
    const float *vec,
    float *result,
    uint16_t rows,
    uint16_t cols)
{
    // Return without writing output when any pointer is NULL or when rows/cols are zero.
    if (!matrix || !vec || !result || rows == 0 || cols == 0)
    {
        return;
    }

    for (uint16_t r = 0; r < rows; r++)
    {
        float sum = 0.0f;
        for (uint16_t c = 0; c < cols; c++)
        {
            sum += matrix[r * cols + c] * vec[c];
        }
        result[r] = sum;
    }
}

void util_symmetric_mat_vec_mul(
    const float *matrix,
    const float *vec,
    float *result,
    uint16_t n)
{
    if (!matrix || !vec || !result || n == 0)
    {
        return;
    }

    /* Fallback to generic multiplication when n is odd or exceeds scratch size. */
    if ((n & 1) || (n > QP_MAXIMUM_VARIABLES))
    {
        util_mat_vec_mul(matrix, vec, result, n, n);
        return;
    }

    /* Workspace sized to the configured maximum to avoid variable-length arrays. */
    float accum[QP_MAXIMUM_VARIABLES];
    for (uint16_t ai = 0; ai < n && ai < QP_MAXIMUM_VARIABLES; ai++)
        accum[ai] = 0.0f;

    uint16_t n_blocks = n >> 1;

    for (uint16_t bi = 0; bi < n_blocks; bi++)
    {
        uint16_t ri = bi << 1;
        float vi0 = vec[ri];
        float vi1 = vec[ri + 1];

        /* Diagonal 2x2 block */
        {
            float h00 = matrix[ri * n + ri];
            float h01 = matrix[ri * n + ri + 1];
            float h11 = matrix[(ri + 1) * n + ri + 1];

            accum[ri]     += h00 * vi0 + h01 * vi1;
            accum[ri + 1] += h01 * vi0 + h11 * vi1;
        }

        /* Off-diagonal 2x2 blocks */
        for (uint16_t bj = bi + 1; bj < n_blocks; bj++)
        {
            uint16_t rj = bj << 1;
            float vj0 = vec[rj];
            float vj1 = vec[rj + 1];

            float a00 = matrix[ri * n + rj];
            float a01 = matrix[ri * n + rj + 1];
            float a10 = matrix[(ri + 1) * n + rj];
            float a11 = matrix[(ri + 1) * n + rj + 1];

            accum[ri]     += a00 * vj0 + a01 * vj1;
            accum[ri + 1] += a10 * vj0 + a11 * vj1;
            accum[rj]     += a00 * vi0 + a10 * vi1;
            accum[rj + 1] += a01 * vi0 + a11 * vi1;
        }
    }

    for (uint16_t i = 0; i < n; i++)
        result[i] = accum[i];
}

void util_vec_add_scaled(
    const float *a,
    const float *b,
    float scalar,
    float *result,
    uint16_t len)
{
    if (!a || !b || !result || len == 0)
    {
        return;
    }

    for (uint16_t i = 0; i < len; i++)
    {
        result[i] = a[i] + scalar * b[i];
    }
}

float util_max_violation(
    const float *A,
    const float *x,
    const float *b,
    uint16_t constraints,
    uint16_t vars)
{
    if (!A || !x || !b || constraints == 0 || vars == 0)
    {
        return 0.0f;
    }

    float max_viol = 0.0f;
    for (uint16_t ci = 0; ci < constraints; ci++)
    {
        float val = 0.0f;
        for (uint16_t vi = 0; vi < vars; vi++)
        {
            val += A[ci * vars + vi] * x[vi];
        }
        float viol = val - b[ci];
        if (viol > max_viol)
            max_viol = viol;
    }
    return max_viol;
}
