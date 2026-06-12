/**
 * @file qp_solver_mpcc.c
 * @brief ADMM + Riccati QP Solver — Implementation
 *
 * Solves the structured multistage QP from the global-frame MPCC using
 * ADMM with Riccati recursion for the equality-constrained subproblem.
 *
 * Matrix dimensions:
 *   NX = MPCC_NX = 7   (state: [s, vx, vy, omega, X, Y, psi])
 *   NU = MPCC_NU = 3   (controls: delta, a_x, v_theta)
 *   G_k inversion is 3x3 -> Cramer's rule (FPGA-friendly, no loops)
 *
 * Track corridor constraints are applied per-stage in the ADMM projection
 * step by clamping the Cartesian position states against the local path
 * frame, enabling tight corridor constraints.
 *
 * All arithmetic uses Q16.16 fixed-point for FPGA compatibility.
 */

#include "qp_solver_mpcc.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

#ifdef MPCC_DEBUG_PRINT
#include <stdio.h>
#endif

/*===========================================================================
 * Fixed-Size Matrix/Vector Helpers
 *===========================================================================
 * These operate on MPCC_NX and MPCC_NU dimensions.
 * Loop bounds follow the MPCC_NX and MPCC_NU macros directly.
 */

/*---------------------------------------------------------------------------
 * NX x NX matrix operations
 *---------------------------------------------------------------------------*/

/** C[NX][NX] = A[NX][NX] * B[NX][NX] */
static void mat_nx_mul(
    const float A[MPCC_NX][MPCC_NX],
    const float B[MPCC_NX][MPCC_NX],
    float C[MPCC_NX][MPCC_NX])
{
    for (int i = 0; i < MPCC_NX; i++)
    {
        for (int j = 0; j < MPCC_NX; j++)
        {
            float sum = 0.0f;
            for (int k = 0; k < MPCC_NX; k++)
                sum += A[i][k] * B[k][j];
            C[i][j] = sum;
        }
    }
}

/** C[NX][NX] = A[NX][NX]^T * B[NX][NX] */
static void mat_nx_trmul(
    const float A[MPCC_NX][MPCC_NX],
    const float B[MPCC_NX][MPCC_NX],
    float C[MPCC_NX][MPCC_NX])
{
    for (int i = 0; i < MPCC_NX; i++)
    {
        for (int j = 0; j < MPCC_NX; j++)
        {
            float sum = 0.0f;
            for (int k = 0; k < MPCC_NX; k++)
                sum += A[k][i] * B[k][j];
            C[i][j] = sum;
        }
    }
}

/** C[NX][NX] = A[NX][NX] + B[NX][NX] */
static void mat_nx_add(
    const float A[MPCC_NX][MPCC_NX],
    const float B[MPCC_NX][MPCC_NX],
    float C[MPCC_NX][MPCC_NX])
{
    for (int i = 0; i < MPCC_NX; i++)
        for (int j = 0; j < MPCC_NX; j++)
            C[i][j] = A[i][j] + B[i][j];
}

/** A[NX][NX] += rho * I */
static void mat_nx_add_rhoI(
    float A[MPCC_NX][MPCC_NX],
    float rho)
{
    for (int i = 0; i < MPCC_NX; i++)
        A[i][i] += rho;
}

/*---------------------------------------------------------------------------
 * Cross-dimension operations (B is NX x NU)
 *---------------------------------------------------------------------------*/

/** result[NU][NX] = B^T * P * A */
static void mat_BtPA(
    const float B[MPCC_NX][MPCC_NU],
    const float P[MPCC_NX][MPCC_NX],
    const float A[MPCC_NX][MPCC_NX],
    float result[MPCC_NU][MPCC_NX])
{
    float PA[MPCC_NX][MPCC_NX];
    mat_nx_mul(P, A, PA);

    for (int i = 0; i < MPCC_NU; i++)
    {
        for (int j = 0; j < MPCC_NX; j++)
        {
            float sum = 0.0f;
            for (int k = 0; k < MPCC_NX; k++)
                sum += B[k][i] * PA[k][j];
            result[i][j] = sum;
        }
    }
}

/** result[NU][NU] = B^T * P * B */
static void mat_BtPB(
    const float B[MPCC_NX][MPCC_NU],
    const float P[MPCC_NX][MPCC_NX],
    float result[MPCC_NU][MPCC_NU])
{
    float PB[MPCC_NX][MPCC_NU];
    for (int i = 0; i < MPCC_NX; i++)
    {
        for (int j = 0; j < MPCC_NU; j++)
        {
            float sum = 0.0f;
            for (int k = 0; k < MPCC_NX; k++)
                sum += P[i][k] * B[k][j];
            PB[i][j] = sum;
        }
    }

    for (int i = 0; i < MPCC_NU; i++)
    {
        for (int j = 0; j < MPCC_NU; j++)
        {
            float sum = 0.0f;
            for (int k = 0; k < MPCC_NX; k++)
                sum += B[k][i] * PB[k][j];
            result[i][j] = sum;
        }
    }
}

/** result[NU] = B^T * v */
static void mat_Btv(
    const float B[MPCC_NX][MPCC_NU],
    const float v[MPCC_NX],
    float result[MPCC_NU])
{
    for (int i = 0; i < MPCC_NU; i++)
    {
        float sum = 0.0f;
        for (int k = 0; k < MPCC_NX; k++)
            sum += B[k][i] * v[k];
        result[i] = sum;
    }
}

/** result[NX] = A^T * v */
static void mat_Atv(
    const float A[MPCC_NX][MPCC_NX],
    const float v[MPCC_NX],
    float result[MPCC_NX])
{
    for (int i = 0; i < MPCC_NX; i++)
    {
        float sum = 0.0f;
        for (int k = 0; k < MPCC_NX; k++)
            sum += A[k][i] * v[k];
        result[i] = sum;
    }
}

/*---------------------------------------------------------------------------
 * Matrix-vector multiplies for forward pass
 *---------------------------------------------------------------------------*/

/** result[NX] = A * x */
static void matvec_nx(
    const float A[MPCC_NX][MPCC_NX],
    const float x[MPCC_NX],
    float result[MPCC_NX])
{
    for (int i = 0; i < MPCC_NX; i++)
    {
        float sum = 0.0f;
        for (int j = 0; j < MPCC_NX; j++)
            sum += A[i][j] * x[j];
        result[i] = sum;
    }
}

/** result[NX] = B * u */
static void matvec_Bu(
    const float B[MPCC_NX][MPCC_NU],
    const float u[MPCC_NU],
    float result[MPCC_NX])
{
    for (int i = 0; i < MPCC_NX; i++)
    {
        float sum = 0.0f;
        for (int j = 0; j < MPCC_NU; j++)
            sum += B[i][j] * u[j];
        result[i] = sum;
    }
}

/** result[NU] = K * x */
static void matvec_Kx(
    const float K[MPCC_NU][MPCC_NX],
    const float x[MPCC_NX],
    float result[MPCC_NU])
{
    for (int i = 0; i < MPCC_NU; i++)
    {
        float sum = 0.0f;
        for (int j = 0; j < MPCC_NX; j++)
            sum += K[i][j] * x[j];
        result[i] = sum;
    }
}

/** K[NU][NX] = -Ginv * H */
static void mat_neg_GinvH(
    const float Ginv[MPCC_NU][MPCC_NU],
    const float H[MPCC_NU][MPCC_NX],
    float K[MPCC_NU][MPCC_NX])
{
    for (int i = 0; i < MPCC_NU; i++)
    {
        for (int j = 0; j < MPCC_NX; j++)
        {
            float sum = 0.0f;
            for (int k = 0; k < MPCC_NU; k++)
                sum += Ginv[i][k] * H[k][j];
            K[i][j] = -sum;
        }
    }
}

/** result[NU] = -Ginv * v */
static void vec_neg_Ginv_v(
    const float Ginv[MPCC_NU][MPCC_NU],
    const float v[MPCC_NU],
    float result[MPCC_NU])
{
    for (int i = 0; i < MPCC_NU; i++)
    {
        float sum = 0.0f;
        for (int k = 0; k < MPCC_NU; k++)
            sum += Ginv[i][k] * v[k];
        result[i] = -sum;
    }
}

/** C[NX][NX] += H^T * K */
static void mat_accum_HtK(
    float C[MPCC_NX][MPCC_NX],
    const float H[MPCC_NU][MPCC_NX],
    const float K[MPCC_NU][MPCC_NX])
{
    for (int i = 0; i < MPCC_NX; i++)
    {
        for (int j = 0; j < MPCC_NX; j++)
        {
            float sum = 0.0f;
            for (int k = 0; k < MPCC_NU; k++)
                sum += H[k][i] * K[k][j];
            C[i][j] += sum;
        }
    }
}

/** result[NX] += H^T * v */
static void vec_accum_Htv(
    float result[MPCC_NX],
    const float H[MPCC_NU][MPCC_NX],
    const float v[MPCC_NU])
{
    for (int i = 0; i < MPCC_NX; i++)
    {
        float sum = 0.0f;
        for (int k = 0; k < MPCC_NU; k++)
            sum += H[k][i] * v[k];
        result[i] += sum;
    }
}

/*===========================================================================
 * 3x3 Matrix Inverse (Cramer's Rule — FPGA-friendly)
 *===========================================================================
 * Uses cofactor expansion: inv(M) = adj(M) / det(M)
 * No loops with data dependencies — fully parallelizable for HLS.
 * With ADMM regularization (rho*I), G_k is always well-conditioned.
 */

int mat_nu_inverse(
    const float M[MPCC_NU][MPCC_NU],
    float inv[MPCC_NU][MPCC_NU])
{
    /* Cofactors of first row */
    float c00 = M[1][1] * M[2][2] - M[1][2] * M[2][1];
    float c01 = -(M[1][0] * M[2][2] - M[1][2] * M[2][0]);
    float c02 = M[1][0] * M[2][1] - M[1][1] * M[2][0];

    /* Determinant */
    float det = M[0][0] * c00 + M[0][1] * c01 + M[0][2] * c02;

    if (fabsf(det) < 1e-10f)
    {
        memset(inv, 0, sizeof(float) * MPCC_NU * MPCC_NU);
        return -1;
    }

    float inv_det = 1.0f / det;

    /* Remaining cofactors */
    float c10 = -(M[0][1] * M[2][2] - M[0][2] * M[2][1]);
    float c11 = M[0][0] * M[2][2] - M[0][2] * M[2][0];
    float c12 = -(M[0][0] * M[2][1] - M[0][1] * M[2][0]);
    float c20 = M[0][1] * M[1][2] - M[0][2] * M[1][1];
    float c21 = -(M[0][0] * M[1][2] - M[0][2] * M[1][0]);
    float c22 = M[0][0] * M[1][1] - M[0][1] * M[1][0];

    /* inv = adjugate / det = cofactor^T / det */
    inv[0][0] = c00 * inv_det;
    inv[0][1] = c10 * inv_det;
    inv[0][2] = c20 * inv_det;
    inv[1][0] = c01 * inv_det;
    inv[1][1] = c11 * inv_det;
    inv[1][2] = c21 * inv_det;
    inv[2][0] = c02 * inv_det;
    inv[2][1] = c12 * inv_det;
    inv[2][2] = c22 * inv_det;

    return 0;
}

/*===========================================================================
 * Initialization
 *===========================================================================*/

void admm_solver_initialize(ADMMWorkspace_t *workspace)
{
    memset(workspace, 0, sizeof(*workspace));
}

void admm_solver_default_config(ADMMConfig_t *config)
{
    config->rho = MPCC_DEFAULT_ADMM_RHO;
    config->max_iterations = MPCC_DEFAULT_ADMM_MAX_ITER;
    config->eps_primal = MPCC_DEFAULT_ADMM_TOLERANCE;
    config->eps_dual = MPCC_DEFAULT_ADMM_TOLERANCE;
    config->warm_start = 1;
    config->rho_u = 4;            /* 0 = use same rho as states */
    config->adaptive_rho = 1;     /* Application configuration may override this setting. */
    config->alpha_relax = 1.6f;  /* Over-relaxation: accelerates ADMM convergence */
}

/*===========================================================================
 * 3x3 Float Matrix Inverse (Cramer's Rule)
 *===========================================================================*/

static int invert_3x3(const float S[3][3], float Si[3][3])
{
    /* Cofactors of first row */
    float c00 = S[1][1] * S[2][2] - S[1][2] * S[2][1];
    float c01 = -(S[1][0] * S[2][2] - S[1][2] * S[2][0]);
    float c02 = S[1][0] * S[2][1] - S[1][1] * S[2][0];

    /* Determinant */
    float det = S[0][0] * c00 + S[0][1] * c01 + S[0][2] * c02;

    if (fabsf(det) < 1e-12f) {
        return -1;
    }

    float inv_det = 1.0f / det;

    /* Remaining cofactors */
    float c10 = -(S[0][1] * S[2][2] - S[0][2] * S[2][1]);
    float c11 = S[0][0] * S[2][2] - S[0][2] * S[2][0];
    float c12 = -(S[0][0] * S[2][1] - S[0][1] * S[2][0]);
    float c20 = S[0][1] * S[1][2] - S[0][2] * S[1][1];
    float c21 = -(S[0][0] * S[1][2] - S[0][2] * S[1][0]);
    float c22 = S[0][0] * S[1][1] - S[0][1] * S[1][0];

    /* inv = cofactor^T / det */
    Si[0][0] = c00 * inv_det;
    Si[0][1] = c10 * inv_det;
    Si[0][2] = c20 * inv_det;
    Si[1][0] = c01 * inv_det;
    Si[1][1] = c11 * inv_det;
    Si[1][2] = c21 * inv_det;
    Si[2][0] = c02 * inv_det;
    Si[2][1] = c12 * inv_det;
    Si[2][2] = c22 * inv_det;

    return 0;
}

/*===========================================================================
 * Sparse A/B Matrix Helpers — Fixed Sparsity Pattern
 *===========================================================================
 * A is 7×7, 20/49 nonzeros. Column-nonzero rows:
 *   col 0: {0}           A[0][0]=1 (identity, s has no state coupling)
 *   col 1: {1,2,3,4,5}
 *   col 2: {1,2,3,4,5}
 *   col 3: {1,2,3,6}
 *   col 4: {4}           A[4][4]=1 (identity, X)
 *   col 5: {5}           A[5][5]=1 (identity, Y)
 *   col 6: {4,5,6}       A[6][6]=1 (identity, psi)
 *
 * B is 7×3, 7/21 nonzeros. Column-nonzero rows:
 *   col 0 (delta):  {1,2,3}
 *   col 1 (ax):     {1,2,3}
 *   col 2 (vtheta): {0}
 *===========================================================================*/

static void BtP_sparse(
    const float B[MPCC_NX][MPCC_NU],
    const float P[MPCC_NX][MPCC_NX],
    float M[MPCC_NU][MPCC_NX])
{
    for (int j = 0; j < MPCC_NX; j++) {
        M[MPCC_IDX_DELTA][j]  = B[1][MPCC_IDX_DELTA]  * P[1][j]
                                + B[2][MPCC_IDX_DELTA]  * P[2][j]
                                + B[3][MPCC_IDX_DELTA]  * P[3][j];
        M[MPCC_IDX_AX][j]     = B[1][MPCC_IDX_AX]     * P[1][j]
                                + B[2][MPCC_IDX_AX]     * P[2][j]
                                + B[3][MPCC_IDX_AX]     * P[3][j];
        M[MPCC_IDX_VTHETA][j] = B[0][MPCC_IDX_VTHETA] * P[0][j];
    }
}

static void MA_plus_scS_sparse(
    const float M[MPCC_NU][MPCC_NX],
    const float A[MPCC_NX][MPCC_NX],
    const float sc_S[MPCC_NU][MPCC_NX],
    float H[MPCC_NU][MPCC_NX])
{
    for (int i = 0; i < MPCC_NU; i++) {
        H[i][0] = M[i][0] + sc_S[i][0];
        H[i][1] = M[i][1]*A[1][1] + M[i][2]*A[2][1] + M[i][3]*A[3][1]
                + M[i][4]*A[4][1] + M[i][5]*A[5][1] + sc_S[i][1];
        H[i][2] = M[i][1]*A[1][2] + M[i][2]*A[2][2] + M[i][3]*A[3][2]
                + M[i][4]*A[4][2] + M[i][5]*A[5][2] + sc_S[i][2];
        H[i][3] = M[i][1]*A[1][3] + M[i][2]*A[2][3]
                + M[i][3]*A[3][3] + M[i][6]*A[6][3] + sc_S[i][3];
        H[i][4] = M[i][4] + sc_S[i][4];
        H[i][5] = M[i][5] + sc_S[i][5];
        H[i][6] = M[i][4]*A[4][6] + M[i][5]*A[5][6] + M[i][6] + sc_S[i][6];
    }
}

static void PA_sparse(
    const float P[MPCC_NX][MPCC_NX],
    const float A[MPCC_NX][MPCC_NX],
    float PA[MPCC_NX][MPCC_NX])
{
    for (int i = 0; i < MPCC_NX; i++) {
        PA[i][0] = P[i][0];
        PA[i][1] = P[i][1]*A[1][1] + P[i][2]*A[2][1] + P[i][3]*A[3][1]
                 + P[i][4]*A[4][1] + P[i][5]*A[5][1];
        PA[i][2] = P[i][1]*A[1][2] + P[i][2]*A[2][2] + P[i][3]*A[3][2]
                 + P[i][4]*A[4][2] + P[i][5]*A[5][2];
        PA[i][3] = P[i][1]*A[1][3] + P[i][2]*A[2][3]
                 + P[i][3]*A[3][3] + P[i][6]*A[6][3];
        PA[i][4] = P[i][4];
        PA[i][5] = P[i][5];
        PA[i][6] = P[i][4]*A[4][6] + P[i][5]*A[5][6] + P[i][6];
    }
}

static void AtPA_sparse(
    const float A[MPCC_NX][MPCC_NX],
    const float PA[MPCC_NX][MPCC_NX],
    float APA[MPCC_NX][MPCC_NX])
{
    for (int j = 0; j < MPCC_NX; j++) {
        APA[0][j] = PA[0][j];
        APA[1][j] = A[1][1]*PA[1][j] + A[2][1]*PA[2][j] + A[3][1]*PA[3][j]
                  + A[4][1]*PA[4][j] + A[5][1]*PA[5][j];
        APA[2][j] = A[1][2]*PA[1][j] + A[2][2]*PA[2][j] + A[3][2]*PA[3][j]
                  + A[4][2]*PA[4][j] + A[5][2]*PA[5][j];
        APA[3][j] = A[1][3]*PA[1][j] + A[2][3]*PA[2][j]
                  + A[3][3]*PA[3][j] + A[6][3]*PA[6][j];
        APA[4][j] = PA[4][j];
        APA[5][j] = PA[5][j];
        APA[6][j] = A[4][6]*PA[4][j] + A[5][6]*PA[5][j] + PA[6][j];
    }
}

static void Atv_sparse(
    const float A[MPCC_NX][MPCC_NX],
    const float v[MPCC_NX],
    float result[MPCC_NX])
{
    result[0] = v[0];
    result[1] = A[1][1]*v[1] + A[2][1]*v[2] + A[3][1]*v[3]
              + A[4][1]*v[4] + A[5][1]*v[5];
    result[2] = A[1][2]*v[1] + A[2][2]*v[2] + A[3][2]*v[3]
              + A[4][2]*v[4] + A[5][2]*v[5];
    result[3] = A[1][3]*v[1] + A[2][3]*v[2]
              + A[3][3]*v[3] + A[6][3]*v[6];
    result[4] = v[4];
    result[5] = v[5];
    result[6] = A[4][6]*v[4] + A[5][6]*v[5] + v[6];
}

static void Btv_sparse(
    const float B[MPCC_NX][MPCC_NU],
    const float v[MPCC_NX],
    float result[MPCC_NU])
{
    result[MPCC_IDX_DELTA]  = B[1][MPCC_IDX_DELTA]  * v[1]
                             + B[2][MPCC_IDX_DELTA]  * v[2]
                             + B[3][MPCC_IDX_DELTA]  * v[3];
    result[MPCC_IDX_AX]     = B[1][MPCC_IDX_AX]     * v[1]
                             + B[2][MPCC_IDX_AX]     * v[2]
                             + B[3][MPCC_IDX_AX]     * v[3];
    result[MPCC_IDX_VTHETA] = B[0][MPCC_IDX_VTHETA] * v[0];
}

static void Ax_sparse(
    const float A[MPCC_NX][MPCC_NX],
    const float x[MPCC_NX],
    float result[MPCC_NX])
{
    result[0] = x[0];
    result[1] = A[1][1]*x[1] + A[1][2]*x[2] + A[1][3]*x[3];
    result[2] = A[2][1]*x[1] + A[2][2]*x[2] + A[2][3]*x[3];
    result[3] = A[3][1]*x[1] + A[3][2]*x[2] + A[3][3]*x[3];
    result[4] = A[4][1]*x[1] + A[4][2]*x[2] + x[4] + A[4][6]*x[6];
    result[5] = A[5][1]*x[1] + A[5][2]*x[2] + x[5] + A[5][6]*x[6];
    result[6] = A[6][3]*x[3] + x[6];
}

static void Bu_sparse(
    const float B[MPCC_NX][MPCC_NU],
    const float u[MPCC_NU],
    float result[MPCC_NX])
{
    result[0] = B[0][MPCC_IDX_VTHETA] * u[MPCC_IDX_VTHETA];
    result[1] = B[1][MPCC_IDX_DELTA] * u[MPCC_IDX_DELTA]
              + B[1][MPCC_IDX_AX]    * u[MPCC_IDX_AX];
    result[2] = B[2][MPCC_IDX_DELTA] * u[MPCC_IDX_DELTA]
              + B[2][MPCC_IDX_AX]    * u[MPCC_IDX_AX];
    result[3] = B[3][MPCC_IDX_DELTA] * u[MPCC_IDX_DELTA]
              + B[3][MPCC_IDX_AX]    * u[MPCC_IDX_AX];
    result[4] = 0.0f;
    result[5] = 0.0f;
    result[6] = 0.0f;
}

/** Threshold to detect "effectively unconstrained" bounds. */
#define MPCC_BOUND_THRESHOLD  100.0f
#define MPCC_ADMM_RHO_MIN       1.0f
#define MPCC_ADMM_RHO_MAX      80.0f
#define MPCC_ADMM_RHO_UPDATE_INTERVAL  5u
#define MPCC_ADMM_RHO_ADAPT_RATIO       5.0f
#define MPCC_ADMM_RHO_SCALE             1.5f

static int admm_run_final_riccati_pass(void)
{
    static int initialized = 0;
    static int run_final_pass = 0;

    if (!initialized)
    {
        const char *value = getenv("MPCC_RUN_FINAL_RICCATI_PASS");
        run_final_pass = (value != NULL && atoi(value) != 0) ? 1 : 0;
        initialized = 1;
    }

    return run_final_pass;
}

void riccati_backward_pass(
    const MPCCQPProblem_t *problem,
    ADMMWorkspace_t *ws,
    float rho,
    float rho_u)
{
    uint16_t N = problem->N;

    /* All states get ADMM rho augmentation for proper consensus.
     * Previously only "constrained" states (bounds < threshold) got rho,
     * leaving vy, omega, psi without consensus penalty — causing poor
     * P-matrix conditioning and ADMM non-convergence. */
    uint8_t x_constrained[MPCC_NX];
    for (int i = 0; i < MPCC_NX; i++) {
        x_constrained[i] = 1;
    }

    /* Rolling value function */
    float P[MPCC_NX][MPCC_NX];
    float p[MPCC_NX];

    /* Initialize P_N and p_N from the terminal cost matrices */
    for (int i = 0; i < MPCC_NX; i++) {
        for (int j = 0; j < MPCC_NX; j++)
            P[i][j] = problem->terminal_cost.Q[i][j];
    }

    /* ADMM augmentation of terminal cost-to-go:
     *   P_N[i][i] += rho          (constrained states)
     *   p_N[i]     = q_N[i] + rho*(lambda_x[N][i] - w_x[N][i])
     */
    for (int i = 0; i < MPCC_NX; i++) {
        if (x_constrained[i]) {
            P[i][i] += rho;
            p[i] = problem->terminal_cost.q[i]
                 + rho * (ws->lambda_x[N][i] - ws->w_x[N][i]);
        } else {
            p[i] = problem->terminal_cost.q[i];
        }
    }

    /* Symmetrize P_N */
    for (int i = 0; i < MPCC_NX; i++) {
        for (int j = i + 1; j < MPCC_NX; j++) {
            float sym = 0.5f * (P[i][j] + P[j][i]);
            P[i][j] = sym;
            P[j][i] = sym;
        }
    }

    for (int i = 0; i < MPCC_NX; i++)
        ws->p[N][i] = p[i];

    /* Backward sweep: k = N-1 down to 0 */
    for (int k = N - 1; k >= 0; k--)
    {
        const MPCCLinearSystem_t *dyn = &problem->dynamics[k];
        const MPCCStageCost_t *sc = &problem->stage_cost[k];

        /* Step 1: s_next = P * d + p  (affine offset from dynamics) */
        float s_next[MPCC_NX];
        for (int i = 0; i < MPCC_NX; i++) {
            float dot = 0.0f;
            for (int j = 0; j < MPCC_NX; j++)
                dot += P[i][j] * dyn->d[j];
            s_next[i] = p[i] + dot;
        }

        /* Step 2: M = B^T * P  (NU x NX) — sparse B (7 of 21 nonzeros). */
        float M[MPCC_NU][MPCC_NX];
        BtP_sparse(dyn->B, P, M);

        /* Step 3: S = R_tilde + M*B  (3x3) — sparse B column multiply. */
        float S[MPCC_NU][MPCC_NU];
        for (int i = 0; i < MPCC_NU; i++) {
            S[i][MPCC_IDX_DELTA]  = sc->R[i][MPCC_IDX_DELTA]
                                   + M[i][1]*dyn->B[1][MPCC_IDX_DELTA]
                                   + M[i][2]*dyn->B[2][MPCC_IDX_DELTA]
                                   + M[i][3]*dyn->B[3][MPCC_IDX_DELTA];
            S[i][MPCC_IDX_AX]     = sc->R[i][MPCC_IDX_AX]
                                   + M[i][1]*dyn->B[1][MPCC_IDX_AX]
                                   + M[i][2]*dyn->B[2][MPCC_IDX_AX]
                                   + M[i][3]*dyn->B[3][MPCC_IDX_AX];
            S[i][MPCC_IDX_VTHETA] = sc->R[i][MPCC_IDX_VTHETA]
                                   + M[i][0]*dyn->B[0][MPCC_IDX_VTHETA];
        }
        S[MPCC_IDX_DELTA][MPCC_IDX_DELTA]   += rho_u;
        S[MPCC_IDX_AX][MPCC_IDX_AX]         += rho_u;
        S[MPCC_IDX_VTHETA][MPCC_IDX_VTHETA] += rho_u;

        /* Step 4: H = M * A + sc->S  (NU x NX) — sparse A column multiply. */
        float H[MPCC_NU][MPCC_NX];
        MA_plus_scS_sparse(M, dyn->A, sc->S, H);

        /* Step 5: Invert S (3x3, Cramer's rule) */
        float Si[MPCC_NU][MPCC_NU];
        if (invert_3x3(S, Si) < 0) {
            /* Near-singular: add regularization and retry */
            for (int i = 0; i < MPCC_NU; i++)
                S[i][i] += 10.0f;
            if (invert_3x3(S, Si) < 0) {
                /* Still singular: diagonal fallback */
                memset(Si, 0, sizeof(Si));
                for (int i = 0; i < MPCC_NU; i++)
                    Si[i][i] = (S[i][i] != 0.0f) ? (1.0f / S[i][i]) : 0.0f;
            }
        }

        /* Step 6: K = -Sinv * H  (NU x NX) */
        float K_local[MPCC_NU][MPCC_NX];
        for (int i = 0; i < MPCC_NU; i++) {
            for (int j = 0; j < MPCC_NX; j++) {
                float sum = 0.0f;
                for (int a = 0; a < MPCC_NU; a++)
                    sum += Si[i][a] * H[a][j];
                K_local[i][j] = -sum;
                ws->K[k][i][j] = -sum;
            }
        }

        /* Step 7: kk = -Sinv * (r_tilde + B^T s_next) — sparse B. */
        float Bts[MPCC_NU];
        Btv_sparse(dyn->B, s_next, Bts);

        float r_tilde[MPCC_NU];
        for (int i = 0; i < MPCC_NU; i++) {
            r_tilde[i] = sc->r[i]
                       + rho_u * (ws->lambda_u[k][i] - ws->w_u[k][i]);
        }

        for (int i = 0; i < MPCC_NU; i++) {
            float sum = 0.0f;
            for (int a = 0; a < MPCC_NU; a++)
                sum += Si[i][a] * (r_tilde[a] + Bts[a]);
            ws->kk[k][i] = -sum;
        }

        /* Step 8: P_k = Q_tilde + A^T P A + H^T K — sparse A (20/49 nonzeros). */
        float PA[MPCC_NX][MPCC_NX];
        PA_sparse(P, dyn->A, PA);

        float APA[MPCC_NX][MPCC_NX];
        AtPA_sparse(dyn->A, PA, APA);

        float P_new[MPCC_NX][MPCC_NX];
        for (int i = 0; i < MPCC_NX; i++) {
            for (int j = 0; j < MPCC_NX; j++) {
                float htk = 0.0f;
                for (int a = 0; a < MPCC_NU; a++)
                    htk += H[a][i] * K_local[a][j];
                P_new[i][j] = sc->Q[i][j] + APA[i][j] + htk;
            }
            if (x_constrained[i])
                P_new[i][i] += rho;
        }

        /* Symmetrize P to prevent floating-point asymmetry drift
         * accumulating over the backward sweep. Mathematically P is
         * symmetric — this enforces it numerically at each stage. */
        for (int i = 0; i < MPCC_NX; i++) {
            for (int j = i + 1; j < MPCC_NX; j++) {
                float sym = 0.5f * (P_new[i][j] + P_new[j][i]);
                P_new[i][j] = sym;
                P_new[j][i] = sym;
            }
        }

        memcpy(P, P_new, sizeof(P));

        /* Step 9: p_k = q_tilde + A^T s_next + H^T kk — sparse A^T. */
        float ats[MPCC_NX];
        Atv_sparse(dyn->A, s_next, ats);

        float p_new[MPCC_NX];
        for (int i = 0; i < MPCC_NX; i++) {
            float q_tilde_i;
            if (x_constrained[i]) {
                q_tilde_i = sc->q[i]
                          + rho * (ws->lambda_x[k][i] - ws->w_x[k][i]);
            } else {
                q_tilde_i = sc->q[i];
            }
            float htk = 0.0f;
            for (int a = 0; a < MPCC_NU; a++)
                htk += H[a][i] * ws->kk[k][a];
            p_new[i] = q_tilde_i + ats[i] + htk;
        }
        memcpy(p, p_new, sizeof(p));

        for (int i = 0; i < MPCC_NX; i++)
            ws->p[k][i] = p[i];
    }
}

/*===========================================================================
 * Riccati Forward Pass
 *===========================================================================*/

void riccati_forward_pass(
    const MPCCQPProblem_t *problem,
    ADMMWorkspace_t *ws)
{
    uint16_t N = problem->N;

    /* x_0 = x_init */
    memcpy(ws->z_x[0], problem->x0, sizeof(float) * MPCC_NX);

    for (uint16_t k = 0; k < N; k++)
    {
        /* u_k = K_k * x_k + kk_k */
        matvec_Kx(ws->K[k], ws->z_x[k], ws->z_u[k]);
        for (int i = 0; i < MPCC_NU; i++)
            ws->z_u[k][i] = (ws->z_u[k][i] + ws->kk[k][i]);

        /* x_{k+1} = A_k * x_k + B_k * u_k + d_k — sparse A and B. */
        float Ax[MPCC_NX], Bu[MPCC_NX];
        Ax_sparse(problem->dynamics[k].A, ws->z_x[k], Ax);
        Bu_sparse(problem->dynamics[k].B, ws->z_u[k], Bu);
        for (int i = 0; i < MPCC_NX; i++)
            ws->z_x[k + 1][i] = Ax[i] + Bu[i] + problem->dynamics[k].d[i];
    }
}

/*===========================================================================
 * ADMM Over-Relaxation Step
 *
 * Replaces z with z_hat = alpha*z + (1-alpha)*w in-place.
 * This accelerates ADMM convergence for alpha in (1.0, 2.0).
 * Typical sweet-spot: alpha=1.5 to 1.7.
 *
 * FPGA-friendly: one multiply + one add per variable, no branching.
 *===========================================================================*/

static void admm_over_relax(
    ADMMWorkspace_t *ws,
    uint16_t N,
    float alpha)
{
    /* Skip if alpha == 1.0 (no relaxation) */
    if (alpha == 1.0f) return;

    float one_minus_alpha = (1.0f - alpha);

    /* Relax states: z_hat = alpha*z + (1-alpha)*w */
    for (uint16_t k = 1; k <= N; k++)   /* skip k=0 (initial state fixed) */
        for (int i = 0; i < MPCC_NX; i++)
            ws->z_x[k][i] = alpha * ws->z_x[k][i]
                           + one_minus_alpha * ws->w_x[k][i];

    /* Relax controls: z_hat = alpha*z + (1-alpha)*w */
    for (uint16_t k = 0; k < N; k++)
        for (int i = 0; i < MPCC_NU; i++)
            ws->z_u[k][i] = alpha * ws->z_u[k][i]
                           + one_minus_alpha * ws->w_u[k][i];
}

/*===========================================================================
 * ADMM Projection Step (w-update)
 *===========================================================================*/

void admm_projection_step(
    const MPCCQPProblem_t *problem,
    ADMMWorkspace_t *ws)
{
    uint16_t N = problem->N;

    memcpy(ws->w_x_prev, ws->w_x,
           sizeof(float) * (MPCC_MAX_HORIZON + 1) * MPCC_NX);
    memcpy(ws->w_u_prev, ws->w_u,
           sizeof(float) * MPCC_MAX_HORIZON * MPCC_NU);

    /* Project states */
    for (uint16_t k = 0; k <= N; k++)
    {
        if (k == 0) {
            memcpy(ws->w_x[0], problem->x0, sizeof(float) * MPCC_NX);
            continue;
        }

        /* 1. Global box constraints (vx bounds, s >= 0, etc.) */
        for (int i = 0; i < MPCC_NX; i++) {
            float val = ws->z_x[k][i] + ws->lambda_x[k][i];
            if (val < problem->x_lower[i]) val = problem->x_lower[i];
            if (val > problem->x_upper[i]) val = problem->x_upper[i];
            if (i == MPCC_IDX_S &&
                problem->s_upper_stage[k] > problem->s_lower_stage[k]) {
                if (val < problem->s_lower_stage[k]) val = problem->s_lower_stage[k];
                if (val > problem->s_upper_stage[k]) val = problem->s_upper_stage[k];
            }
            ws->w_x[k][i] = val;
        }

        /* 1b. Optional per-stage vx speed limit */
        if (problem->vx_max_stage[k] > 0.0f &&
            ws->w_x[k][MPCC_IDX_VX] > problem->vx_max_stage[k])
        {
            ws->w_x[k][MPCC_IDX_VX] = problem->vx_max_stage[k];
        }

        /* 2. Per-stage track corridor constraint on (X, Y).
         *
         * Transform proposed Cartesian position into the local Frenet
         * frame, clamp the lateral deviation, transform back.
         *
         * Frenet frame at arc-length s_k:
         *   tangent  t = [ cos(phi),  sin(phi) ]
         *   normal   n = [-sin(phi),  cos(phi) ]  (positive = left)
         *
         * Lateral deviation (contouring error):
         *   e_c = sin(phi)*(X - x_ref) - cos(phi)*(Y - y_ref)
         * Longitudinal deviation (lag error — NOT clamped):
         *   e_l = -cos(phi)*(X - x_ref) - sin(phi)*(Y - y_ref)
         *
         * Inverse (exact):
         *   X_new = x_ref + sin(phi)*e_c_clamped - cos(phi)*e_l
         *   Y_new = y_ref - cos(phi)*e_c_clamped - sin(phi)*e_l
         */
        {
            float x_ref   = problem->path_x_ref[k];
            float y_ref   = problem->path_y_ref[k];
            /* sin/cos precomputed once in build_qp_problem — saves 2 trig calls
             * per stage per ADMM iteration (typically ~20 iterations × 80 stages) */
            float sin_phi = problem->path_sin_phi[k];
            float cos_phi = problem->path_cos_phi[k];

            float X_prop = ws->w_x[k][MPCC_IDX_X];
            float Y_prop = ws->w_x[k][MPCC_IDX_Y];

            float dX = X_prop - x_ref;
            float dY = Y_prop - y_ref;

            /* Decompose into Frenet components */
            float e_c = (sin_phi * dX) - (cos_phi * dY);   /* lateral  */
            float e_l = (-cos_phi * dX) - (sin_phi * dY);  /* longitudinal — preserved */

            /* Clamp lateral deviation to corridor.
             * e_c sign: positive = vehicle is RIGHT of reference path.
             * left_b/right_b are positive max distances to each wall. */
            float left_b  = problem->track_left[k];   /* positive: max left  */
            float right_b = problem->track_right[k];  /* positive: max right */
            if (e_c >  right_b) e_c =  right_b;
            if (e_c < -left_b)  e_c = -left_b;

            /* Reconstruct clamped Cartesian position */
            ws->w_x[k][MPCC_IDX_X] = x_ref + (sin_phi * e_c) - (cos_phi * e_l);
            ws->w_x[k][MPCC_IDX_Y] = y_ref - (cos_phi * e_c) - (sin_phi * e_l);
        }
    }

    /* Project controls (unchanged) */
    for (uint16_t k = 0; k < N; k++) {
        for (int i = 0; i < MPCC_NU; i++) {
            float val = ws->z_u[k][i] + ws->lambda_u[k][i];

            if (i == MPCC_IDX_DELTA) {
                float lower = problem->delta_lower_stage[k];
                float upper = problem->delta_upper_stage[k];

                if (!(upper > lower)) {
                    lower = problem->u_lower[i];
                    upper = problem->u_upper[i];
                }

                if (val < lower) val = lower;
                if (val > upper) val = upper;
            } else {
                if (val < problem->u_lower[i]) val = problem->u_lower[i];
                if (val > problem->u_upper[i]) val = problem->u_upper[i];
            }

            ws->w_u[k][i] = val;
        }
        if (problem->mu_g_sq > 0.0f) {
            float ax_lim = problem->ax_lim_stage[k];
            float ax = ws->w_u[k][MPCC_IDX_AX];
            if (ax >  ax_lim) ws->w_u[k][MPCC_IDX_AX] =  ax_lim;
            if (ax < -ax_lim) ws->w_u[k][MPCC_IDX_AX] = -ax_lim;
        }
    }
}

/*===========================================================================
 * ADMM Dual Update
 *===========================================================================*/

void admm_dual_update(
    ADMMWorkspace_t *ws,
    uint16_t N)
{
    for (uint16_t k = 0; k <= N; k++)
        for (int i = 0; i < MPCC_NX; i++)
            ws->lambda_x[k][i] = ws->lambda_x[k][i]
                + (ws->z_x[k][i] - ws->w_x[k][i]);

    for (uint16_t k = 0; k < N; k++)
        for (int i = 0; i < MPCC_NU; i++)
            ws->lambda_u[k][i] = ws->lambda_u[k][i]
                + (ws->z_u[k][i] - ws->w_u[k][i]);
}

/*===========================================================================
 * Residual Computation
 *===========================================================================*/

void admm_compute_residuals(
    const ADMMWorkspace_t *ws,
    float rho,
    float rho_u,
    uint16_t N,
    float *primal_res,
    float *dual_res,
    /* Optional outputs: per-domain residuals (states / controls). */
    float *primal_x_res,
    float *dual_x_res,
    float *primal_u_res,
    float *dual_u_res)
{
    float max_prim = 0;
    float max_dual = 0;
    float max_prim_x = 0;
    float max_dual_x = 0;
    float max_prim_u = 0;
    float max_dual_u = 0;

    if (!isfinite((double)rho) || !isfinite((double)rho_u)) {
        *primal_res = INFINITY;
        *dual_res = INFINITY;
        if (primal_x_res) *primal_x_res = INFINITY;
        if (dual_x_res) *dual_x_res = INFINITY;
        if (primal_u_res) *primal_u_res = INFINITY;
        if (dual_u_res) *dual_u_res = INFINITY;
        return;
    }

    /* State residuals */
    for (uint16_t k = 0; k <= N; k++)
    {
        for (int i = 0; i < MPCC_NX; i++)
        {
            float diff = (ws->z_x[k][i] - ws->w_x[k][i]);
            if (!isfinite((double)diff)) {
                *primal_res = INFINITY;
                *dual_res = INFINITY;
                return;
            }
            float abs_diff = diff < 0 ? (0 - diff) : diff;
            if (abs_diff > max_prim) max_prim = abs_diff;
            if (abs_diff > max_prim_x) max_prim_x = abs_diff;

            float w_diff = (ws->w_x[k][i] - ws->w_x_prev[k][i]);
            if (!isfinite((double)w_diff)) {
                *primal_res = INFINITY;
                *dual_res = INFINITY;
                return;
            }
            float abs_w = rho * (w_diff < 0 ? -w_diff : w_diff);
            if (!isfinite((double)abs_w)) {
                *primal_res = INFINITY;
                *dual_res = INFINITY;
                return;
            }
            if (abs_w > max_dual) max_dual = abs_w;
            if (abs_w > max_dual_x) max_dual_x = abs_w;
        }
    }

    /* Control residuals */
    for (uint16_t k = 0; k < N; k++)
    {
        for (int i = 0; i < MPCC_NU; i++)
        {
            float diff = (ws->z_u[k][i] - ws->w_u[k][i]);
            if (!isfinite((double)diff)) {
                *primal_res = INFINITY;
                *dual_res = INFINITY;
                return;
            }
            float abs_diff = diff < 0 ? (0 - diff) : diff;
            if (abs_diff > max_prim) max_prim = abs_diff;
            if (abs_diff > max_prim_u) max_prim_u = abs_diff;

            float w_diff = (ws->w_u[k][i] - ws->w_u_prev[k][i]);
            if (!isfinite((double)w_diff)) {
                *primal_res = INFINITY;
                *dual_res = INFINITY;
                return;
            }
            float abs_w = rho_u * (w_diff < 0 ? -w_diff : w_diff);
            if (!isfinite((double)abs_w)) {
                *primal_res = INFINITY;
                *dual_res = INFINITY;
                return;
            }
            if (abs_w > max_dual) max_dual = abs_w;
            if (abs_w > max_dual_u) max_dual_u = abs_w;
        }
    }

    /* Export per-domain residuals when requested */
    if (primal_x_res) *primal_x_res = max_prim_x;
    if (dual_x_res) *dual_x_res = max_dual_x;
    if (primal_u_res) *primal_u_res = max_prim_u;
    if (dual_u_res) *dual_u_res = max_dual_u;

    /* Combined residuals (max across domains) */
    if (max_prim_u > max_prim_x) max_prim = max_prim_u;
    if (max_dual_u > max_dual_x) max_dual = max_dual_u;

    *primal_res = max_prim;
    *dual_res = max_dual;
}

/*===========================================================================
 * Main ADMM Solver
 *===========================================================================*/

MPCCStatus_t admm_solver_solve(
    const MPCCQPProblem_t *problem,
    const ADMMConfig_t *config,
    ADMMWorkspace_t *workspace,
    ADMMResult_t *result)
{
    if (!config || !problem || !workspace || !result)
    {
        if (result) result->status = MPCC_STATUS_ERROR;
        return MPCC_STATUS_ERROR;
    }

    uint16_t N = problem->N;
    if (N == 0 || N > MPCC_MAX_HORIZON)
    {
        result->status = MPCC_STATUS_ERROR;
        return MPCC_STATUS_ERROR;
    }

    workspace->adaptive_rho_updates = 0;
    workspace->adaptive_rho_state_updates = 0;
    workspace->adaptive_rho_control_updates = 0;
    workspace->numeric_clip_count = 0;

    /* Cold start: zero all ADMM variables, then smart-init */
    if (!config->warm_start)
    {
        memset(workspace->z_x, 0, sizeof(workspace->z_x));
        memset(workspace->z_u, 0, sizeof(workspace->z_u));
        memset(workspace->w_x, 0, sizeof(workspace->w_x));
        memset(workspace->w_u, 0, sizeof(workspace->w_u));
        memset(workspace->lambda_x, 0, sizeof(workspace->lambda_x));
        memset(workspace->lambda_u, 0, sizeof(workspace->lambda_u));
        memset(workspace->w_x_prev, 0, sizeof(workspace->w_x_prev));
        memset(workspace->w_u_prev, 0, sizeof(workspace->w_u_prev));
    }

    /* Enforce initial state */
    for (int i = 0; i < MPCC_NX; i++)
    {
        workspace->z_x[0][i] = problem->x0[i];
        workspace->w_x[0][i] = problem->x0[i];
    }

    float rho;
    float rho_u;

    /* Fixed-rho solves always use the configured penalties.  With adaptive
     * rho enabled, a valid warm start continues with the penalty found in
     * the preceding solve; a workspace reset clears these saved values. */
    rho = config->rho > 0 ? config->rho : 10.0f;
    rho_u = config->rho_u > 0 ? config->rho_u : rho;
    if (config->adaptive_rho && config->warm_start) {
        if (isfinite((double)workspace->rho_state) &&
            workspace->rho_state > 0.0f)
            rho = workspace->rho_state;
        if (isfinite((double)workspace->rho_u_state) &&
            workspace->rho_u_state > 0.0f)
            rho_u = workspace->rho_u_state;
    }
    if (config->adaptive_rho) {
        if (rho < MPCC_ADMM_RHO_MIN) rho = MPCC_ADMM_RHO_MIN;
        if (rho > MPCC_ADMM_RHO_MAX) rho = MPCC_ADMM_RHO_MAX;
        if (rho_u < MPCC_ADMM_RHO_MIN) rho_u = MPCC_ADMM_RHO_MIN;
        if (rho_u > MPCC_ADMM_RHO_MAX) rho_u = MPCC_ADMM_RHO_MAX;
    }
    MPCCStatus_t status = MPCC_STATUS_MAX_ITERATIONS;

    /* Smart cold-start (from MPC): run one unconstrained Riccati pass
     * to seed z from projection and lambda from violation. This
     * dramatically accelerates convergence vs starting from zero. */
    if (!config->warm_start)
    {
        riccati_backward_pass(problem, workspace, 0, 0);
        riccati_forward_pass(problem, workspace);

#ifdef MPCC_DEBUG_PRINT
        printf("  [DBG] K[0][0][0]=%.8f kk[0][0]=%.8f\n",
               (double)workspace->K[0][0][0], (double)workspace->kk[0][0]);
        printf("  [DBG] z_u[0][0]=%.8f z_x[0][0]=%.8f z_x[1][1]=%.8f\n",
               (double)workspace->z_u[0][0], (double)workspace->z_x[0][0],
               (double)workspace->z_x[1][1]);
#endif

        /* Initialize w from projection of unconstrained solution */
        admm_projection_step(problem, workspace);

        /* Initialize lambda (dual) from constraint violation:
         * lambda = z_unconstrained - w_projected. This primes the
         * dual variables instead of slowly accumulating from zero. */
        for (uint16_t k = 0; k <= N; k++) {
            for (int i = 0; i < MPCC_NX; i++) {
                workspace->lambda_x[k][i] =
                    workspace->z_x[k][i] - workspace->w_x[k][i];
            }
        }
        for (uint16_t k = 0; k < N; k++) {
            for (int i = 0; i < MPCC_NU; i++) {
                workspace->lambda_u[k][i] =
                    workspace->z_u[k][i] - workspace->w_u[k][i];
            }
        }
    }
    else
    {
        /* Warm-start re-consensus against current constraints. */
        admm_projection_step(problem, workspace);
        memset(workspace->lambda_x[0], 0, sizeof(workspace->lambda_x[0]));
    }

    /* Over-relaxation only on warm starts. */
    float alpha_relax = config->warm_start ? config->alpha_relax : 1.0f;

    /* === ADMM Iteration Loop === */
    for (uint16_t iter = 0; iter < config->max_iterations; iter++)
    {
        /* Step 1: z-update (Riccati solve) */
        riccati_backward_pass(problem, workspace, rho, rho_u);
        riccati_forward_pass(problem, workspace);

        /* Step 1.5: Over-relaxation z_hat = α*z + (1-α)*w */
        admm_over_relax(workspace, N, alpha_relax);

        /* Step 2: w-update (box projection with track bounds) */
        admm_projection_step(problem, workspace);

        /* Step 3: Dual update */
        admm_dual_update(workspace, N);

        /* Step 4: Convergence check — evaluated every 2 iterations to halve
         * the residual-scan overhead (807 comparisons per check × N_iters).
         * We still track iterations accurately; the extra half-iteration of
         * work on an already-converged solution is negligible. */
        workspace->iterations = iter + 1;
        if ((iter & 1u) == 0u)
        {
            float prim_res, dual_res;
            float prim_x = 0.0f, dual_x = 0.0f, prim_u = 0.0f, dual_u = 0.0f;
            admm_compute_residuals(workspace, rho, rho_u, N, &prim_res, &dual_res,
                           &prim_x, &dual_x, &prim_u, &dual_u);

            if (!isfinite((double)prim_res) || !isfinite((double)dual_res)) {
                status = MPCC_STATUS_ERROR;
                workspace->primal_residual = INFINITY;
                workspace->dual_residual = INFINITY;
                break;
            }

            workspace->primal_residual = prim_res;
            workspace->dual_residual = dual_res;

#ifdef MPCC_DEBUG_PRINT
            if ((iter + 1) % 10 == 0 || iter == 0)
            {
                printf("  ADMM iter %3u: prim=%.6f  dual=%.6f\n",
                       iter + 1, (double)(prim_res), (double)(dual_res));
            }
#endif

            if (prim_res <= config->eps_primal &&
                dual_res <= config->eps_dual)
            {
                status = MPCC_STATUS_SUCCESS;
                break;
            }

            /* Adaptive rho uses per-domain residuals — only update on check iters */
            if (config->adaptive_rho &&
                (((unsigned)iter + 1u) % MPCC_ADMM_RHO_UPDATE_INTERVAL) == 0u) {
                float old_rho = rho;
                if (prim_x > MPCC_ADMM_RHO_ADAPT_RATIO * dual_x &&
                    rho < MPCC_ADMM_RHO_MAX) {
                    rho *= MPCC_ADMM_RHO_SCALE;
                    if (rho > MPCC_ADMM_RHO_MAX) rho = MPCC_ADMM_RHO_MAX;
                } else if (dual_x > MPCC_ADMM_RHO_ADAPT_RATIO * prim_x &&
                           rho > MPCC_ADMM_RHO_MIN) {
                    rho /= MPCC_ADMM_RHO_SCALE;
                    if (rho < MPCC_ADMM_RHO_MIN) rho = MPCC_ADMM_RHO_MIN;
                }
                if (rho != old_rho) {
                    float lambda_scale = old_rho / rho;
                    workspace->adaptive_rho_updates++;
                    workspace->adaptive_rho_state_updates++;
                    for (uint16_t kk = 0; kk <= N; kk++)
                        for (int i = 0; i < MPCC_NX; i++)
                            workspace->lambda_x[kk][i] *= lambda_scale;
                }

                float old_rho_u = rho_u;
                if (prim_u > MPCC_ADMM_RHO_ADAPT_RATIO * dual_u &&
                    rho_u < MPCC_ADMM_RHO_MAX) {
                    rho_u *= MPCC_ADMM_RHO_SCALE;
                    if (rho_u > MPCC_ADMM_RHO_MAX) rho_u = MPCC_ADMM_RHO_MAX;
                } else if (dual_u > MPCC_ADMM_RHO_ADAPT_RATIO * prim_u &&
                           rho_u > MPCC_ADMM_RHO_MIN) {
                    rho_u /= MPCC_ADMM_RHO_SCALE;
                    if (rho_u < MPCC_ADMM_RHO_MIN) rho_u = MPCC_ADMM_RHO_MIN;
                }
                if (rho_u != old_rho_u) {
                    float lambda_scale = old_rho_u / rho_u;
                    workspace->adaptive_rho_updates++;
                    workspace->adaptive_rho_control_updates++;
                    for (uint16_t kk = 0; kk < N; kk++)
                        for (int i = 0; i < MPCC_NU; i++)
                            workspace->lambda_u[kk][i] *= lambda_scale;
                }
            }
        }

    }

    /* The exported solution below is rolled out from the projected controls
     * w_u, which are the feasible ADMM controls. The old final Riccati
     * recompute is kept as a runtime fallback for A/B testing, but skipped by
     * default to save roughly one Riccati iteration per solve. */
    if (admm_run_final_riccati_pass())
    {
        riccati_backward_pass(problem, workspace, rho, rho_u);
        riccati_forward_pass(problem, workspace);
    }

    /* Extract solution */
    result->status = status;
    result->iterations = workspace->iterations;
    result->primal_residual = workspace->primal_residual;
    result->dual_residual = workspace->dual_residual;
    result->rho_final = rho;
    result->rho_u_final = rho_u;
    result->adaptive_rho_updates = workspace->adaptive_rho_updates;
    result->adaptive_rho_state_updates = workspace->adaptive_rho_state_updates;
    result->adaptive_rho_control_updates = workspace->adaptive_rho_control_updates;
    result->numeric_clip_count = workspace->numeric_clip_count;

    /* Export a dynamics-consistent state rollout for the feasible controls. */
    memcpy(result->x_opt[0], problem->x0, sizeof(float) * MPCC_NX);
    for (uint16_t k = 0; k < N; k++) {
        float Ax[MPCC_NX], Bu[MPCC_NX];
        matvec_nx(problem->dynamics[k].A, result->x_opt[k], Ax);
        matvec_Bu(problem->dynamics[k].B, workspace->w_u[k], Bu);
        for (int i = 0; i < MPCC_NX; i++)
            result->x_opt[k + 1][i] = Ax[i] + Bu[i] + problem->dynamics[k].d[i];
    }
    /* Output feasible controls: w_u is the ADMM projection, always
     * within bounds. The raw z_u from the Riccati forward pass is
     * unconstrained and may exceed control limits. */
    memcpy(result->u_opt, workspace->w_u,
           sizeof(float) * N * MPCC_NU);

    /* Retain the final values for an adaptive warm-started solve.  They are
     * ignored when adaptive rho is disabled. */
    workspace->rho_state = rho;
    workspace->rho_u_state = rho_u;

    return status;
}
