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
    config->warm_start = 0;
    config->rho_u = 0;            /* 0 = use same rho as states */
    config->adaptive_rho = 1;     /* DISABLED: adaptive rho oscillates at tight corners */
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

/** Threshold to detect "effectively unconstrained" bounds. */
#define MPCC_BOUND_THRESHOLD  100.0f

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

    /* Store P_N, p_N */
    for (int i = 0; i < MPCC_NX; i++) {
        for (int j = 0; j < MPCC_NX; j++)
            ws->P[N][i][j] = P[i][j];
        ws->p[N][i] = p[i];
    }

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

        /* Step 2: M = B^T * P  (NU x NX) */
        float M[MPCC_NU][MPCC_NX];
        for (int i = 0; i < MPCC_NU; i++) {
            for (int j = 0; j < MPCC_NX; j++) {
                float sum = 0.0f;
                for (int s = 0; s < MPCC_NX; s++)
                    sum += dyn->B[s][i] * P[s][j];
                M[i][j] = sum;
            }
        }

        /* Step 3: S = R_tilde + M*B  (3x3) */
        float S[MPCC_NU][MPCC_NU];
        for (int i = 0; i < MPCC_NU; i++) {
            for (int j = 0; j < MPCC_NU; j++) {
                float base = sc->R[i][j];
                if (i == j) base += rho_u;
                float sum = 0.0f;
                for (int s = 0; s < MPCC_NX; s++)
                    sum += M[i][s] * dyn->B[s][j];
                S[i][j] = base + sum;
            }
        }

        /* Step 4: H = M * A  (NU x NX) */
        float H[MPCC_NU][MPCC_NX];
        for (int i = 0; i < MPCC_NU; i++) {
            for (int j = 0; j < MPCC_NX; j++) {
                float sum = 0.0f;
                for (int s = 0; s < MPCC_NX; s++)
                    sum += M[i][s] * dyn->A[s][j];
                H[i][j] = sum;
            }
        }

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

        /* Step 7: kk = -Sinv * (r_tilde + B^T s_next) */
        float Bts[MPCC_NU];
        for (int i = 0; i < MPCC_NU; i++) {
            float sum = 0.0f;
            for (int s = 0; s < MPCC_NX; s++)
                sum += dyn->B[s][i] * s_next[s];
            Bts[i] = sum;
        }

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

        /* Step 8: P_k = Q_tilde + A^T P A + H^T K */
        float PA[MPCC_NX][MPCC_NX];
        for (int i = 0; i < MPCC_NX; i++) {
            for (int j = 0; j < MPCC_NX; j++) {
                float sum = 0.0f;
                for (int s = 0; s < MPCC_NX; s++)
                    sum += P[i][s] * dyn->A[s][j];
                PA[i][j] = sum;
            }
        }

        float P_new[MPCC_NX][MPCC_NX];
        for (int i = 0; i < MPCC_NX; i++) {
            for (int j = 0; j < MPCC_NX; j++) {
                float sum = sc->Q[i][j];
                /* A^T * PA */
                float apa = 0.0f;
                for (int s = 0; s < MPCC_NX; s++)
                    apa += dyn->A[s][i] * PA[s][j];
                sum += apa;
                /* H^T * K */
                float htk = 0.0f;
                for (int a = 0; a < MPCC_NU; a++)
                    htk += H[a][i] * K_local[a][j];
                sum += htk;
                P_new[i][j] = sum;
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

        /* Store P_k, p_k in workspace */
        for (int i = 0; i < MPCC_NX; i++) {
            for (int j = 0; j < MPCC_NX; j++)
                ws->P[k][i][j] = P[i][j];
            ws->p[k][i] = p[i];
        }

        /* Step 9: p_k = q_tilde + A^T s_next + H^T kk */
        float p_new[MPCC_NX];
        for (int i = 0; i < MPCC_NX; i++) {
            float q_tilde_i;
            if (x_constrained[i]) {
                q_tilde_i = sc->q[i]
                          + rho * (ws->lambda_x[k][i] - ws->w_x[k][i]);
            } else {
                q_tilde_i = sc->q[i];
            }
            float ats = 0.0f;
            for (int s = 0; s < MPCC_NX; s++)
                ats += dyn->A[s][i] * s_next[s];
            float htk = 0.0f;
            for (int a = 0; a < MPCC_NU; a++)
                htk += H[a][i] * ws->kk[k][a];
            p_new[i] = q_tilde_i + ats + htk;
        }
        memcpy(p, p_new, sizeof(p));

        /* Store P_k, p_k in workspace */
        for (int i = 0; i < MPCC_NX; i++) {
            for (int j = 0; j < MPCC_NX; j++)
                ws->P[k][i][j] = P[i][j];
            ws->p[k][i] = p[i];
        }
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

        /* x_{k+1} = A_k * x_k + B_k * u_k + d_k (with saturation) */
        float Ax[MPCC_NX], Bu[MPCC_NX];
        matvec_nx(problem->dynamics[k].A, ws->z_x[k], Ax);
        matvec_Bu(problem->dynamics[k].B, ws->z_u[k], Bu);
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
            ws->w_x[k][i] = val;
        }

        /* 1b. Per-stage vx speed limit (curvature-based braking) */
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
            float phi     = problem->path_phi_ref[k];
            float x_ref   = problem->path_x_ref[k];
            float y_ref   = problem->path_y_ref[k];
            float sin_phi = sinf(phi);
            float cos_phi = cosf(phi);

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
            if (val < problem->u_lower[i]) val = problem->u_lower[i];
            if (val > problem->u_upper[i]) val = problem->u_upper[i];
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
    float *dual_res)
{
    float max_prim = 0;
    float max_dual = 0;

    if (!isfinite((double)rho) || !isfinite((double)rho_u)) {
        *primal_res = INFINITY;
        *dual_res = INFINITY;
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
        }
    }

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

    if (config->warm_start && workspace->rho_state > 0)
        rho = workspace->rho_state;
    else
        rho = config->rho;

    if (config->warm_start && workspace->rho_u_state > 0)
        rho_u = workspace->rho_u_state;
    else
        rho_u = config->rho_u > 0 ? config->rho_u : rho;

    /* Cold start uses the configured rho (not adapted) */
    if (!config->warm_start) {
        rho   = config->rho > 0 ? config->rho : 10.0f;
        rho_u = rho;
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

        /* Step 4: Convergence check */
        float prim_res, dual_res;
        admm_compute_residuals(workspace, rho, rho_u, N, &prim_res, &dual_res);

        if (!isfinite((double)prim_res) || !isfinite((double)dual_res)) {
            status = MPCC_STATUS_ERROR;
            workspace->primal_residual = INFINITY;
            workspace->dual_residual = INFINITY;
            workspace->iterations = iter + 1;
            break;
        }

        workspace->primal_residual = prim_res;
        workspace->dual_residual = dual_res;
        workspace->iterations = iter + 1;

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

        /*--- Adaptive rho: balance primal/dual convergence rates ---
         * Check every 25 iterations (not every 2) and use 1.5x scaling
         * (not 2x) to avoid ping-ponging that disrupts convergence. */
        if (config->adaptive_rho && iter >= 25 && (iter % 25) == 0) {
            if (prim_res > 5.0f * dual_res &&
                rho < 100.0f) {
                float scale = 1.5f;
                float inv_scale = 1.0f / scale;
                rho = (rho * scale);
                if (rho_u < 100.0f)
                    rho_u = (rho_u * scale);
                if (rho > 100.0f) rho = 100.0f;
                if (rho_u > 100.0f) rho_u = 100.0f;
                workspace->adaptive_rho_updates++;
                /* Scale dual variables: lambda /= scale */
                for (uint16_t kk = 0; kk <= N; kk++)
                    for (int i = 0; i < MPCC_NX; i++)
                        workspace->lambda_x[kk][i] *= inv_scale;
                for (uint16_t kk = 0; kk < N; kk++)
                    for (int i = 0; i < MPCC_NU; i++)
                        workspace->lambda_u[kk][i] *= inv_scale;
            } else if (dual_res > 5.0f * prim_res &&
                       rho > 0.5f) {
                float scale = 1.5f;
                rho = (rho / scale);
                if (rho_u > 0.5f)
                    rho_u = (rho_u / scale);
                if (rho < 0.5f) rho = 0.5f;
                if (rho_u < 0.5f) rho_u = 0.5f;
                workspace->adaptive_rho_updates++;
                /* Scale dual variables: lambda *= scale */
                for (uint16_t kk = 0; kk <= N; kk++)
                    for (int i = 0; i < MPCC_NX; i++)
                        workspace->lambda_x[kk][i] *= scale;
                for (uint16_t kk = 0; kk < N; kk++)
                    for (int i = 0; i < MPCC_NU; i++)
                        workspace->lambda_u[kk][i] *= scale;
            }
        }
    }

    /* Recompute primal trajectory after ADMM loop so exported x_opt remains
     * dynamics-consistent and suitable for warm-start linearization. */
    riccati_backward_pass(problem, workspace, rho, rho_u);
    riccati_forward_pass(problem, workspace);

    /* Extract solution */
    result->status = status;
    result->iterations = workspace->iterations;
    result->primal_residual = workspace->primal_residual;
    result->dual_residual = workspace->dual_residual;
    result->rho_final = rho;
    result->rho_u_final = rho_u;
    result->adaptive_rho_updates = workspace->adaptive_rho_updates;
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

    /* Persist adapted penalties for the next warm-started solve. */
    workspace->rho_state = rho;
    workspace->rho_u_state = rho_u;

    return status;
}
