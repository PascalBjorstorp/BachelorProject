/**
 * @file qp_solver_mpcc.c
 * @brief ADMM + Riccati QP Solver — Implementation (Lifted ODE)
 *
 * Solves the structured multistage QP from the Lifted ODE MPCC using
 * ADMM with Riccati recursion for the equality-constrained subproblem.
 *
 * Matrix dimensions:
 *   NX = MPCC_NX = 9   (Lifted ODE: Frenet + Cartesian)
 *   NU = MPCC_NU = 3   (controls: delta, a_x, v_theta)
 *   G_k inversion is 3x3 -> Cramer's rule (FPGA-friendly, no loops)
 *
 * Track bounds on n (state index 1) are applied per-stage in the
 * ADMM projection step, enabling tight corridor constraints.
 *
 * All arithmetic uses Q16.16 fixed-point for FPGA compatibility.
 */

#include "qp_solver_mpcc.h"
#include <string.h>

#ifdef MPCC_DEBUG_PRINT
#include <stdio.h>
#endif

/*===========================================================================
 * Fixed-Size Matrix/Vector Helpers
 *===========================================================================
 * These operate on MPCC_NX and MPCC_NU dimensions.
 * Since the macros changed (NX=10, NU=2), all loops auto-adapt.
 */

/*---------------------------------------------------------------------------
 * NX x NX matrix operations
 *---------------------------------------------------------------------------*/

/** C[NX][NX] = A[NX][NX] * B[NX][NX] */
static void mat_nx_mul(
    const fixed_point_t A[MPCC_NX][MPCC_NX],
    const fixed_point_t B[MPCC_NX][MPCC_NX],
    fixed_point_t C[MPCC_NX][MPCC_NX])
{
    for (int i = 0; i < MPCC_NX; i++)
    {
        for (int j = 0; j < MPCC_NX; j++)
        {
            int64_t sum = 0;
            for (int k = 0; k < MPCC_NX; k++)
                sum += (int64_t)A[i][k] * B[k][j];
            C[i][j] = (fixed_point_t)(sum >> FP_FRAC_BITS);
        }
    }
}

/** C[NX][NX] = A[NX][NX]^T * B[NX][NX] */
static void mat_nx_trmul(
    const fixed_point_t A[MPCC_NX][MPCC_NX],
    const fixed_point_t B[MPCC_NX][MPCC_NX],
    fixed_point_t C[MPCC_NX][MPCC_NX])
{
    for (int i = 0; i < MPCC_NX; i++)
    {
        for (int j = 0; j < MPCC_NX; j++)
        {
            int64_t sum = 0;
            for (int k = 0; k < MPCC_NX; k++)
                sum += (int64_t)A[k][i] * B[k][j];
            C[i][j] = (fixed_point_t)(sum >> FP_FRAC_BITS);
        }
    }
}

/** C[NX][NX] = A[NX][NX] + B[NX][NX] */
static void mat_nx_add(
    const fixed_point_t A[MPCC_NX][MPCC_NX],
    const fixed_point_t B[MPCC_NX][MPCC_NX],
    fixed_point_t C[MPCC_NX][MPCC_NX])
{
    for (int i = 0; i < MPCC_NX; i++)
        for (int j = 0; j < MPCC_NX; j++)
            C[i][j] = fp_add(A[i][j], B[i][j]);
}

/** A[NX][NX] += rho * I */
static void mat_nx_add_rhoI(
    fixed_point_t A[MPCC_NX][MPCC_NX],
    fixed_point_t rho)
{
    for (int i = 0; i < MPCC_NX; i++)
        A[i][i] = fp_add(A[i][i], rho);
}

/*---------------------------------------------------------------------------
 * Cross-dimension operations (B is NX x NU)
 *---------------------------------------------------------------------------*/

/** result[NU][NX] = B^T * P * A */
static void mat_BtPA(
    const fixed_point_t B[MPCC_NX][MPCC_NU],
    const fixed_point_t P[MPCC_NX][MPCC_NX],
    const fixed_point_t A[MPCC_NX][MPCC_NX],
    fixed_point_t result[MPCC_NU][MPCC_NX])
{
    fixed_point_t PA[MPCC_NX][MPCC_NX];
    mat_nx_mul(P, A, PA);

    for (int i = 0; i < MPCC_NU; i++)
    {
        for (int j = 0; j < MPCC_NX; j++)
        {
            int64_t sum = 0;
            for (int k = 0; k < MPCC_NX; k++)
                sum += (int64_t)B[k][i] * PA[k][j];
            result[i][j] = (fixed_point_t)(sum >> FP_FRAC_BITS);
        }
    }
}

/** result[NU][NU] = B^T * P * B */
static void mat_BtPB(
    const fixed_point_t B[MPCC_NX][MPCC_NU],
    const fixed_point_t P[MPCC_NX][MPCC_NX],
    fixed_point_t result[MPCC_NU][MPCC_NU])
{
    fixed_point_t PB[MPCC_NX][MPCC_NU];
    for (int i = 0; i < MPCC_NX; i++)
    {
        for (int j = 0; j < MPCC_NU; j++)
        {
            int64_t sum = 0;
            for (int k = 0; k < MPCC_NX; k++)
                sum += (int64_t)P[i][k] * B[k][j];
            PB[i][j] = (fixed_point_t)(sum >> FP_FRAC_BITS);
        }
    }

    for (int i = 0; i < MPCC_NU; i++)
    {
        for (int j = 0; j < MPCC_NU; j++)
        {
            int64_t sum = 0;
            for (int k = 0; k < MPCC_NX; k++)
                sum += (int64_t)B[k][i] * PB[k][j];
            result[i][j] = (fixed_point_t)(sum >> FP_FRAC_BITS);
        }
    }
}

/** result[NU] = B^T * v */
static void mat_Btv(
    const fixed_point_t B[MPCC_NX][MPCC_NU],
    const fixed_point_t v[MPCC_NX],
    fixed_point_t result[MPCC_NU])
{
    for (int i = 0; i < MPCC_NU; i++)
    {
        int64_t sum = 0;
        for (int k = 0; k < MPCC_NX; k++)
            sum += (int64_t)B[k][i] * v[k];
        result[i] = (fixed_point_t)(sum >> FP_FRAC_BITS);
    }
}

/** result[NX] = A^T * v */
static void mat_Atv(
    const fixed_point_t A[MPCC_NX][MPCC_NX],
    const fixed_point_t v[MPCC_NX],
    fixed_point_t result[MPCC_NX])
{
    for (int i = 0; i < MPCC_NX; i++)
    {
        int64_t sum = 0;
        for (int k = 0; k < MPCC_NX; k++)
            sum += (int64_t)A[k][i] * v[k];
        result[i] = (fixed_point_t)(sum >> FP_FRAC_BITS);
    }
}

/*---------------------------------------------------------------------------
 * Matrix-vector multiplies for forward pass
 *---------------------------------------------------------------------------*/

/** result[NX] = A * x */
static void matvec_nx(
    const fixed_point_t A[MPCC_NX][MPCC_NX],
    const fixed_point_t x[MPCC_NX],
    fixed_point_t result[MPCC_NX])
{
    for (int i = 0; i < MPCC_NX; i++)
    {
        int64_t sum = 0;
        for (int j = 0; j < MPCC_NX; j++)
            sum += (int64_t)A[i][j] * x[j];
        result[i] = (fixed_point_t)(sum >> FP_FRAC_BITS);
    }
}

/** result[NX] = B * u */
static void matvec_Bu(
    const fixed_point_t B[MPCC_NX][MPCC_NU],
    const fixed_point_t u[MPCC_NU],
    fixed_point_t result[MPCC_NX])
{
    for (int i = 0; i < MPCC_NX; i++)
    {
        int64_t sum = 0;
        for (int j = 0; j < MPCC_NU; j++)
            sum += (int64_t)B[i][j] * u[j];
        result[i] = (fixed_point_t)(sum >> FP_FRAC_BITS);
    }
}

/** result[NU] = K * x */
static void matvec_Kx(
    const fixed_point_t K[MPCC_NU][MPCC_NX],
    const fixed_point_t x[MPCC_NX],
    fixed_point_t result[MPCC_NU])
{
    for (int i = 0; i < MPCC_NU; i++)
    {
        int64_t sum = 0;
        for (int j = 0; j < MPCC_NX; j++)
            sum += (int64_t)K[i][j] * x[j];
        result[i] = (fixed_point_t)(sum >> FP_FRAC_BITS);
    }
}

/** K[NU][NX] = -Ginv * H */
static void mat_neg_GinvH(
    const fixed_point_t Ginv[MPCC_NU][MPCC_NU],
    const fixed_point_t H[MPCC_NU][MPCC_NX],
    fixed_point_t K[MPCC_NU][MPCC_NX])
{
    for (int i = 0; i < MPCC_NU; i++)
    {
        for (int j = 0; j < MPCC_NX; j++)
        {
            int64_t sum = 0;
            for (int k = 0; k < MPCC_NU; k++)
                sum += (int64_t)Ginv[i][k] * H[k][j];
            K[i][j] = (fixed_point_t)(-(sum >> FP_FRAC_BITS));
        }
    }
}

/** result[NU] = -Ginv * v */
static void vec_neg_Ginv_v(
    const fixed_point_t Ginv[MPCC_NU][MPCC_NU],
    const fixed_point_t v[MPCC_NU],
    fixed_point_t result[MPCC_NU])
{
    for (int i = 0; i < MPCC_NU; i++)
    {
        int64_t sum = 0;
        for (int k = 0; k < MPCC_NU; k++)
            sum += (int64_t)Ginv[i][k] * v[k];
        result[i] = (fixed_point_t)(-(sum >> FP_FRAC_BITS));
    }
}

/** C[NX][NX] += H^T * K */
static void mat_accum_HtK(
    fixed_point_t C[MPCC_NX][MPCC_NX],
    const fixed_point_t H[MPCC_NU][MPCC_NX],
    const fixed_point_t K[MPCC_NU][MPCC_NX])
{
    for (int i = 0; i < MPCC_NX; i++)
    {
        for (int j = 0; j < MPCC_NX; j++)
        {
            int64_t sum = 0;
            for (int k = 0; k < MPCC_NU; k++)
                sum += (int64_t)H[k][i] * K[k][j];
            C[i][j] = fp_add(C[i][j], (fixed_point_t)(sum >> FP_FRAC_BITS));
        }
    }
}

/** result[NX] += H^T * v */
static void vec_accum_Htv(
    fixed_point_t result[MPCC_NX],
    const fixed_point_t H[MPCC_NU][MPCC_NX],
    const fixed_point_t v[MPCC_NU])
{
    for (int i = 0; i < MPCC_NX; i++)
    {
        int64_t sum = 0;
        for (int k = 0; k < MPCC_NU; k++)
            sum += (int64_t)H[k][i] * v[k];
        result[i] = fp_add(result[i], (fixed_point_t)(sum >> FP_FRAC_BITS));
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
    const fixed_point_t M[MPCC_NU][MPCC_NU],
    fixed_point_t inv[MPCC_NU][MPCC_NU])
{
    /* Cofactors (2x2 minors) via int64 intermediates */
    int64_t c00 = ((int64_t)M[1][1] * M[2][2] - (int64_t)M[1][2] * M[2][1]) >> FP_FRAC_BITS;
    int64_t c01 = -(((int64_t)M[1][0] * M[2][2] - (int64_t)M[1][2] * M[2][0]) >> FP_FRAC_BITS);
    int64_t c02 = ((int64_t)M[1][0] * M[2][1] - (int64_t)M[1][1] * M[2][0]) >> FP_FRAC_BITS;

    /* Determinant via first row expansion */
    int64_t det = ((int64_t)M[0][0] * c00 + (int64_t)M[0][1] * c01 + (int64_t)M[0][2] * c02) >> FP_FRAC_BITS;

    /* Check for singularity */
    fixed_point_t eps = FP_CONST(0.0001);
    int64_t abs_det = det < 0 ? -det : det;
    if (abs_det < eps)
    {
        memset(inv, 0, sizeof(fixed_point_t) * MPCC_NU * MPCC_NU);
        return -1;
    }

    /* Use int64 division to avoid int32 truncation of large determinants */
    int64_t inv_det_64 = ((int64_t)FP_ONE << FP_FRAC_BITS) / det;

    /* Remaining cofactors */
    int64_t c10 = -(((int64_t)M[0][1] * M[2][2] - (int64_t)M[0][2] * M[2][1]) >> FP_FRAC_BITS);
    int64_t c11 = ((int64_t)M[0][0] * M[2][2] - (int64_t)M[0][2] * M[2][0]) >> FP_FRAC_BITS;
    int64_t c12 = -(((int64_t)M[0][0] * M[2][1] - (int64_t)M[0][1] * M[2][0]) >> FP_FRAC_BITS);
    int64_t c20 = ((int64_t)M[0][1] * M[1][2] - (int64_t)M[0][2] * M[1][1]) >> FP_FRAC_BITS;
    int64_t c21 = -(((int64_t)M[0][0] * M[1][2] - (int64_t)M[0][2] * M[1][0]) >> FP_FRAC_BITS);
    int64_t c22 = ((int64_t)M[0][0] * M[1][1] - (int64_t)M[0][1] * M[1][0]) >> FP_FRAC_BITS;

    /* inv = adjugate / det = cofactor^T / det (using int64 arithmetic) */
    inv[0][0] = (fixed_point_t)((c00 * inv_det_64) >> FP_FRAC_BITS);
    inv[0][1] = (fixed_point_t)((c10 * inv_det_64) >> FP_FRAC_BITS);
    inv[0][2] = (fixed_point_t)((c20 * inv_det_64) >> FP_FRAC_BITS);
    inv[1][0] = (fixed_point_t)((c01 * inv_det_64) >> FP_FRAC_BITS);
    inv[1][1] = (fixed_point_t)((c11 * inv_det_64) >> FP_FRAC_BITS);
    inv[1][2] = (fixed_point_t)((c21 * inv_det_64) >> FP_FRAC_BITS);
    inv[2][0] = (fixed_point_t)((c02 * inv_det_64) >> FP_FRAC_BITS);
    inv[2][1] = (fixed_point_t)((c12 * inv_det_64) >> FP_FRAC_BITS);
    inv[2][2] = (fixed_point_t)((c22 * inv_det_64) >> FP_FRAC_BITS);

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
    config->adaptive_rho = 1;     /* Enable adaptive rho by default */
    config->alpha_relax = FP_CONST(1.5); /* Over-relaxation: 1.5 is a robust default */
}

static inline fixed_point_t sat_i64_to_i32_count(int64_t v, uint32_t *clip_counter)
{
    if (v > INT32_MAX) {
        if (clip_counter) (*clip_counter)++;
        return INT32_MAX;
    }
    if (v < INT32_MIN) {
        if (clip_counter) (*clip_counter)++;
        return INT32_MIN;
    }
    return (fixed_point_t)v;
}

/*===========================================================================
 * Riccati Backward Pass (int64 precision, selective augmentation)
 *
 * Matches the FPGA HLS design (riccati_solver_hls.c):
 *   - int64_t rolling P and p buffers to avoid truncation across sweep
 *   - int64_t intermediates with >> FP_FRAC_BITS for fixed-point multiply
 *   - Newton-Raphson reciprocal for 2x2 inverse (no hardware dividers)
 *   - Adaptive for dense Q[NX][NX] and R[NU][NU] (MPCC-specific)
 *   - Affine dynamics d term (MPCC-specific)
 *===========================================================================*/

/** Threshold to detect "effectively unconstrained" bounds.
 *  If |bound| >= this value, the state is treated as unconstrained
 *  and rho augmentation is skipped for that dimension. */
#define MPCC_BOUND_THRESHOLD  FP_CONST(100.0)

/** Reciprocal of Q16.16 int64 value using direct int64 division.
 *  Returns FP_ONE^2 / det = (1 << 32) / det, giving 1/det in Q16.16.
 *  Replaces Newton-Raphson which diverged for large determinants. */
static int64_t reciprocal_64(int64_t det)
{
    if (det == 0) return 0;

    /* Direct 64-bit division: 1/det in Q16.16 = (1 << 32) / det */
    return ((int64_t)1 << 32) / det;
}

/** 3x3 inverse using Cramer's rule + reciprocal (multiply-only, no division).
 *  FPGA-friendly: cofactor expansion is fully parallel, no data-dependent loops. */
static int invert_3x3(const int64_t S[3][3], int64_t Si[3][3])
{
    /* Cofactors of first row (used for determinant) */
    int64_t c00 = (S[1][1] * S[2][2] - S[1][2] * S[2][1]) >> FP_FRAC_BITS;
    int64_t c01 = -((S[1][0] * S[2][2] - S[1][2] * S[2][0]) >> FP_FRAC_BITS);
    int64_t c02 = (S[1][0] * S[2][1] - S[1][1] * S[2][0]) >> FP_FRAC_BITS;

    /* Determinant via first row expansion */
    int64_t det = (S[0][0] * c00 + S[0][1] * c01 + S[0][2] * c02) >> FP_FRAC_BITS;

    if (det == 0 || (det > -16 && det < 16)) {
        return -1;
    }

    int64_t inv_det = reciprocal_64(det);

    /* Remaining cofactors */
    int64_t c10 = -((S[0][1] * S[2][2] - S[0][2] * S[2][1]) >> FP_FRAC_BITS);
    int64_t c11 = (S[0][0] * S[2][2] - S[0][2] * S[2][0]) >> FP_FRAC_BITS;
    int64_t c12 = -((S[0][0] * S[2][1] - S[0][1] * S[2][0]) >> FP_FRAC_BITS);
    int64_t c20 = (S[0][1] * S[1][2] - S[0][2] * S[1][1]) >> FP_FRAC_BITS;
    int64_t c21 = -((S[0][0] * S[1][2] - S[0][2] * S[1][0]) >> FP_FRAC_BITS);
    int64_t c22 = (S[0][0] * S[1][1] - S[0][1] * S[1][0]) >> FP_FRAC_BITS;

    /* inv = adjugate / det = cofactor^T / det */
    Si[0][0] = (c00 * inv_det) >> FP_FRAC_BITS;
    Si[0][1] = (c10 * inv_det) >> FP_FRAC_BITS;
    Si[0][2] = (c20 * inv_det) >> FP_FRAC_BITS;
    Si[1][0] = (c01 * inv_det) >> FP_FRAC_BITS;
    Si[1][1] = (c11 * inv_det) >> FP_FRAC_BITS;
    Si[1][2] = (c21 * inv_det) >> FP_FRAC_BITS;
    Si[2][0] = (c02 * inv_det) >> FP_FRAC_BITS;
    Si[2][1] = (c12 * inv_det) >> FP_FRAC_BITS;
    Si[2][2] = (c22 * inv_det) >> FP_FRAC_BITS;

    return 0;
}

void riccati_backward_pass(
    const MPCCQPProblem_t *problem,
    ADMMWorkspace_t *ws,
    fixed_point_t rho,
    fixed_point_t rho_u)
{
    uint16_t N = problem->N;

    /* Precompute constrained flags from global bounds */
    uint8_t x_constrained[MPCC_NX];
    for (int i = 0; i < MPCC_NX; i++) {
        x_constrained[i] = (problem->x_upper[i] < MPCC_BOUND_THRESHOLD ||
                            problem->x_lower[i] > -MPCC_BOUND_THRESHOLD);
    }
    x_constrained[MPCC_IDX_N] = 1;

    /* Rolling value function in int64 for precision across backward sweep */
    int64_t P[MPCC_NX][MPCC_NX];
    int64_t p[MPCC_NX];

    /* Initialize terminal cost-to-go */
    for (int i = 0; i < MPCC_NX; i++) {
        for (int j = 0; j < MPCC_NX; j++)
            P[i][j] = (int64_t)problem->terminal_cost.Q[i][j];
        if (x_constrained[i]) {
            P[i][i] += (int64_t)rho;
            p[i] = (int64_t)problem->terminal_cost.q[i]
                 + (((int64_t)rho * ((int64_t)ws->lambda_x[N][i]
                 - (int64_t)ws->w_x[N][i])) >> FP_FRAC_BITS);
        } else {
            p[i] = (int64_t)problem->terminal_cost.q[i];
        }
    }

    /* Store P_N in workspace (saturate to int32) */
    for (int i = 0; i < MPCC_NX; i++) {
        for (int j = 0; j < MPCC_NX; j++) {
            ws->P[N][i][j] = sat_i64_to_i32_count(P[i][j], &ws->numeric_clip_count);
        }
        ws->p[N][i] = sat_i64_to_i32_count(p[i], &ws->numeric_clip_count);
    }

    /* Backward sweep: k = N-1 down to 0 */
    for (int k = N - 1; k >= 0; k--)
    {
        const MPCCLinearSystem_t *dyn = &problem->dynamics[k];
        const MPCCStageCost_t *sc = &problem->stage_cost[k];

        /* Step 1: s_next = P * d + p  (affine offset from dynamics) */
        int64_t s_next[MPCC_NX];
        for (int i = 0; i < MPCC_NX; i++) {
            int64_t raw = 0;
            for (int j = 0; j < MPCC_NX; j++)
                raw += P[i][j] * (int64_t)dyn->d[j];
            s_next[i] = p[i] + (raw >> FP_FRAC_BITS);
        }

        /* Step 2: M = B^T * P  (NU x NX) — accumulate then shift */
        int64_t M[MPCC_NU][MPCC_NX];
        for (int i = 0; i < MPCC_NU; i++) {
            for (int j = 0; j < MPCC_NX; j++) {
                int64_t raw = 0;
                for (int s = 0; s < MPCC_NX; s++)
                    raw += (int64_t)dyn->B[s][i] * P[s][j];
                M[i][j] = raw >> FP_FRAC_BITS;
            }
        }

        /* Step 3: S = R_tilde + M*B  (3x3) — accumulate then shift */
        int64_t S[MPCC_NU][MPCC_NU];
        for (int i = 0; i < MPCC_NU; i++) {
            for (int j = 0; j < MPCC_NU; j++) {
                int64_t base = (int64_t)sc->R[i][j];
                if (i == j) base += (int64_t)rho_u;
                int64_t raw = 0;
                for (int s = 0; s < MPCC_NX; s++)
                    raw += M[i][s] * (int64_t)dyn->B[s][j];
                S[i][j] = base + (raw >> FP_FRAC_BITS);
            }
        }

        /* Step 4: H = M * A  (NU x NX) — accumulate then shift */
        int64_t H[MPCC_NU][MPCC_NX];
        for (int i = 0; i < MPCC_NU; i++) {
            for (int j = 0; j < MPCC_NX; j++) {
                int64_t raw = 0;
                for (int s = 0; s < MPCC_NX; s++)
                    raw += M[i][s] * (int64_t)dyn->A[s][j];
                H[i][j] = raw >> FP_FRAC_BITS;
            }
        }

        /* Step 5: Invert S (3x3, Cramer's rule + Newton-Raphson reciprocal) */
        int64_t Si[MPCC_NU][MPCC_NU];
        if (invert_3x3(S, Si) < 0) {
            /* Near-singular: add regularization and retry */
            for (int i = 0; i < MPCC_NU; i++)
                S[i][i] += (int64_t)FP_CONST(10.0);
            if (invert_3x3(S, Si) < 0) {
                /* Still singular: diagonal fallback */
                memset(Si, 0, sizeof(Si));
                for (int i = 0; i < MPCC_NU; i++)
                    Si[i][i] = S[i][i] != 0
                        ? ((int64_t)FP_ONE << FP_FRAC_BITS) / S[i][i] : 0;
            }
        }

        /* Step 6: K = -Sinv * H  (NU x NX) — accumulate then shift */
        int64_t K64[MPCC_NU][MPCC_NX];
        for (int i = 0; i < MPCC_NU; i++) {
            for (int j = 0; j < MPCC_NX; j++) {
                int64_t raw = 0;
                for (int a = 0; a < MPCC_NU; a++)
                    raw += Si[i][a] * H[a][j];
                int64_t val = -(raw >> FP_FRAC_BITS);
                K64[i][j] = val;
                ws->K[k][i][j] = sat_i64_to_i32_count(val, &ws->numeric_clip_count);
            }
        }

        /* Step 7: kk = -Sinv * (r_tilde + B^T s_next) — accumulate then shift */
        int64_t Bts[MPCC_NU];
        for (int i = 0; i < MPCC_NU; i++) {
            int64_t raw = 0;
            for (int s = 0; s < MPCC_NX; s++)
                raw += (int64_t)dyn->B[s][i] * s_next[s];
            Bts[i] = raw >> FP_FRAC_BITS;
        }

        int64_t r_tilde[MPCC_NU];
        for (int i = 0; i < MPCC_NU; i++) {
            r_tilde[i] = (int64_t)sc->r[i]
                       + (((int64_t)rho_u * ((int64_t)ws->lambda_u[k][i]
                       - (int64_t)ws->w_u[k][i])) >> FP_FRAC_BITS);
        }

        for (int i = 0; i < MPCC_NU; i++) {
            int64_t raw = 0;
            for (int a = 0; a < MPCC_NU; a++)
                raw += Si[i][a] * (r_tilde[a] + Bts[a]);
            int64_t val = -(raw >> FP_FRAC_BITS);
            ws->kk[k][i] = sat_i64_to_i32_count(val, &ws->numeric_clip_count);
        }

        /* Step 8: P_k = Q_tilde + A^T P A + H^T K
         * Use K64 (int64) for H'K to avoid int32 bottleneck */
        int64_t PA[MPCC_NX][MPCC_NX];
        for (int i = 0; i < MPCC_NX; i++) {
            for (int j = 0; j < MPCC_NX; j++) {
                int64_t raw = 0;
                for (int s = 0; s < MPCC_NX; s++)
                    raw += P[i][s] * (int64_t)dyn->A[s][j];
                PA[i][j] = raw >> FP_FRAC_BITS;
            }
        }

        int64_t P_new[MPCC_NX][MPCC_NX];
        for (int i = 0; i < MPCC_NX; i++) {
            for (int j = 0; j < MPCC_NX; j++) {
                int64_t sum = (int64_t)sc->Q[i][j];
                /* A^T * PA — accumulate then shift */
                int64_t raw_apa = 0;
                for (int s = 0; s < MPCC_NX; s++)
                    raw_apa += (int64_t)dyn->A[s][i] * PA[s][j];
                sum += raw_apa >> FP_FRAC_BITS;
                /* H^T * K64 (using int64 K to avoid precision loss) */
                int64_t raw_htk = 0;
                for (int a = 0; a < MPCC_NU; a++)
                    raw_htk += H[a][i] * K64[a][j];
                sum += raw_htk >> FP_FRAC_BITS;
                P_new[i][j] = sum;
            }
            if (x_constrained[i])
                P_new[i][i] += (int64_t)rho;
        }
        memcpy(P, P_new, sizeof(P));

        /* Step 9: p_k = q_tilde + A^T s_next + H^T kk */
        int64_t p_new[MPCC_NX];
        for (int i = 0; i < MPCC_NX; i++) {
            int64_t q_tilde_i;
            if (x_constrained[i]) {
                q_tilde_i = (int64_t)sc->q[i]
                          + (((int64_t)rho * ((int64_t)ws->lambda_x[k][i]
                          - (int64_t)ws->w_x[k][i])) >> FP_FRAC_BITS);
            } else {
                q_tilde_i = (int64_t)sc->q[i];
            }
            int64_t raw_ats = 0;
            for (int s = 0; s < MPCC_NX; s++)
                raw_ats += (int64_t)dyn->A[s][i] * s_next[s];
            int64_t raw_htk = 0;
            for (int a = 0; a < MPCC_NU; a++)
                raw_htk += H[a][i] * (int64_t)ws->kk[k][a];
            p_new[i] = q_tilde_i + (raw_ats >> FP_FRAC_BITS) + (raw_htk >> FP_FRAC_BITS);
        }
        memcpy(p, p_new, sizeof(p));

        /* Store P_k, p_k in workspace (saturate to int32) */
        for (int i = 0; i < MPCC_NX; i++) {
            for (int j = 0; j < MPCC_NX; j++) {
                ws->P[k][i][j] = sat_i64_to_i32_count(P[i][j], &ws->numeric_clip_count);
            }
            ws->p[k][i] = sat_i64_to_i32_count(p[i], &ws->numeric_clip_count);
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
    memcpy(ws->z_x[0], problem->x0, sizeof(fixed_point_t) * MPCC_NX);

    for (uint16_t k = 0; k < N; k++)
    {
        /* u_k = K_k * x_k + kk_k */
        matvec_Kx(ws->K[k], ws->z_x[k], ws->z_u[k]);
        for (int i = 0; i < MPCC_NU; i++)
            ws->z_u[k][i] = fp_add(ws->z_u[k][i], ws->kk[k][i]);

        /* x_{k+1} = A_k * x_k + B_k * u_k + d_k (with saturation) */
        fixed_point_t Ax[MPCC_NX], Bu[MPCC_NX];
        matvec_nx(problem->dynamics[k].A, ws->z_x[k], Ax);
        matvec_Bu(problem->dynamics[k].B, ws->z_u[k], Bu);
        for (int i = 0; i < MPCC_NX; i++)
            ws->z_x[k + 1][i] = fp_add_sat(fp_add_sat(Ax[i], Bu[i]),
                                            problem->dynamics[k].d[i]);
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
    fixed_point_t alpha)
{
    /* Skip if alpha == 1.0 (no relaxation) */
    if (alpha == FP_ONE) return;

    fixed_point_t one_minus_alpha = fp_sub(FP_ONE, alpha);

    /* Relax states: z_hat = alpha*z + (1-alpha)*w */
    for (uint16_t k = 1; k <= N; k++)   /* skip k=0 (initial state fixed) */
        for (int i = 0; i < MPCC_NX; i++)
            ws->z_x[k][i] = fp_add(fp_mul(alpha, ws->z_x[k][i]),
                                    fp_mul(one_minus_alpha, ws->w_x[k][i]));

    /* Relax controls: z_hat = alpha*z + (1-alpha)*w */
    for (uint16_t k = 0; k < N; k++)
        for (int i = 0; i < MPCC_NU; i++)
            ws->z_u[k][i] = fp_add(fp_mul(alpha, ws->z_u[k][i]),
                                    fp_mul(one_minus_alpha, ws->w_u[k][i]));
}

/*===========================================================================
 * ADMM Projection Step (w-update)
 *===========================================================================*/

void admm_projection_step(
    const MPCCQPProblem_t *problem,
    ADMMWorkspace_t *ws)
{
    uint16_t N = problem->N;

    /* Save current w for dual residual */
    memcpy(ws->w_x_prev, ws->w_x,
           sizeof(fixed_point_t) * (MPCC_MAX_HORIZON + 1) * MPCC_NX);
    memcpy(ws->w_u_prev, ws->w_u,
           sizeof(fixed_point_t) * MPCC_MAX_HORIZON * MPCC_NU);

    /* Project states */
    for (uint16_t k = 0; k <= N; k++)
    {
        if (k == 0)
        {
            /* Initial state: hard equality constraint */
            memcpy(ws->w_x[0], problem->x0,
                   sizeof(fixed_point_t) * MPCC_NX);
            continue;
        }

        for (int i = 0; i < MPCC_NX; i++)
        {
            fixed_point_t val = fp_add(ws->z_x[k][i], ws->lambda_x[k][i]);

            /* Apply global box constraints */
            if (val < problem->x_lower[i])
                val = problem->x_lower[i];
            if (val > problem->x_upper[i])
                val = problem->x_upper[i];

            ws->w_x[k][i] = val;
        }

        /* Override n bounds with per-stage track bounds */
        {
            fixed_point_t val_n = fp_add(ws->z_x[k][MPCC_IDX_N],
                                          ws->lambda_x[k][MPCC_IDX_N]);
            fixed_point_t n_lb = fp_sub(0, problem->track_right[k]);
            fixed_point_t n_ub = problem->track_left[k];

            /* Use per-stage if set; fall back to global otherwise */
            if (n_ub > 0 || n_lb < 0)
            {
                if (val_n < n_lb) val_n = n_lb;
                if (val_n > n_ub) val_n = n_ub;
                ws->w_x[k][MPCC_IDX_N] = val_n;
            }
        }

    }

    /* Project controls */
    for (uint16_t k = 0; k < N; k++)
    {
        for (int i = 0; i < MPCC_NU; i++)
        {
            fixed_point_t val = fp_add(ws->z_u[k][i], ws->lambda_u[k][i]);
            if (val < problem->u_lower[i])
                val = problem->u_lower[i];
            if (val > problem->u_upper[i])
                val = problem->u_upper[i];
            ws->w_u[k][i] = val;
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
            ws->lambda_x[k][i] = fp_add(ws->lambda_x[k][i],
                fp_sub(ws->z_x[k][i], ws->w_x[k][i]));

    for (uint16_t k = 0; k < N; k++)
        for (int i = 0; i < MPCC_NU; i++)
            ws->lambda_u[k][i] = fp_add(ws->lambda_u[k][i],
                fp_sub(ws->z_u[k][i], ws->w_u[k][i]));
}

/*===========================================================================
 * Residual Computation
 *===========================================================================*/

void admm_compute_residuals(
    const ADMMWorkspace_t *ws,
    fixed_point_t rho,
    fixed_point_t rho_u,
    uint16_t N,
    fixed_point_t *primal_res,
    fixed_point_t *dual_res)
{
    fixed_point_t max_prim = 0;
    fixed_point_t max_dual = 0;

    /* State residuals */
    for (uint16_t k = 0; k <= N; k++)
    {
        for (int i = 0; i < MPCC_NX; i++)
        {
            fixed_point_t diff = fp_sub(ws->z_x[k][i], ws->w_x[k][i]);
            fixed_point_t abs_diff = diff < 0 ? fp_sub(0, diff) : diff;
            if (abs_diff > max_prim) max_prim = abs_diff;

            fixed_point_t w_diff = fp_sub(ws->w_x[k][i], ws->w_x_prev[k][i]);
            fixed_point_t abs_w = fp_mul(rho, w_diff < 0 ? fp_sub(0, w_diff) : w_diff);
            if (abs_w > max_dual) max_dual = abs_w;
        }
    }

    /* Control residuals */
    for (uint16_t k = 0; k < N; k++)
    {
        for (int i = 0; i < MPCC_NU; i++)
        {
            fixed_point_t diff = fp_sub(ws->z_u[k][i], ws->w_u[k][i]);
            fixed_point_t abs_diff = diff < 0 ? fp_sub(0, diff) : diff;
            if (abs_diff > max_prim) max_prim = abs_diff;

            fixed_point_t w_diff = fp_sub(ws->w_u[k][i], ws->w_u_prev[k][i]);
            fixed_point_t abs_w = fp_mul(rho_u, w_diff < 0 ? fp_sub(0, w_diff) : w_diff);
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

    fixed_point_t rho;
    fixed_point_t rho_u;

    if (config->warm_start && workspace->rho_state > 0)
        rho = workspace->rho_state;
    else
        rho = config->rho;

    if (config->warm_start && workspace->rho_u_state > 0)
        rho_u = workspace->rho_u_state;
    else
        rho_u = config->rho_u > 0 ? config->rho_u : rho;

    /* Cold start gets a higher initial rho for faster feasibility */
    if (!config->warm_start) {
        rho   = FP_CONST(10.0);
        rho_u = FP_CONST(10.0);
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
                workspace->lambda_x[k][i] = fp_sub(
                    workspace->z_x[k][i], workspace->w_x[k][i]);
            }
        }
        for (uint16_t k = 0; k < N; k++) {
            for (int i = 0; i < MPCC_NU; i++) {
                workspace->lambda_u[k][i] = fp_sub(
                    workspace->z_u[k][i], workspace->w_u[k][i]);
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
    fixed_point_t alpha_relax = config->warm_start ? config->alpha_relax : FP_ONE;

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
        fixed_point_t prim_res, dual_res;
        admm_compute_residuals(workspace, rho, rho_u, N, &prim_res, &dual_res);

        workspace->primal_residual = prim_res;
        workspace->dual_residual = dual_res;
        workspace->iterations = iter + 1;

#ifdef MPCC_DEBUG_PRINT
        if ((iter + 1) % 10 == 0 || iter == 0)
        {
            printf("  ADMM iter %3u: prim=%.6f  dual=%.6f\n",
                   iter + 1, FP_TO_DOUBLE(prim_res), FP_TO_DOUBLE(dual_res));
        }
#endif

        if (prim_res <= config->eps_primal &&
            dual_res <= config->eps_dual)
        {
            status = MPCC_STATUS_SUCCESS;
            break;
        }

        /*--- Adaptive rho: balance primal/dual convergence rates ---*/
        if (config->adaptive_rho && iter > 0 && (iter & 1) == 0) {
            if (prim_res / 10 > dual_res &&
                rho < FP_CONST(100.0)) {
                rho = fp_mul(rho, FP_TWO);
                if (rho_u < FP_CONST(100.0))
                    rho_u = fp_mul(rho_u, FP_TWO);
                if (rho > FP_CONST(100.0)) rho = FP_CONST(100.0);
                if (rho_u > FP_CONST(100.0)) rho_u = FP_CONST(100.0);
                workspace->adaptive_rho_updates++;
                /* Scale dual variables: lambda /= 2 (with rounding) */
                for (uint16_t kk = 0; kk <= N; kk++)
                    for (int i = 0; i < MPCC_NX; i++)
                        workspace->lambda_x[kk][i] = (fixed_point_t)(((int64_t)workspace->lambda_x[kk][i] + 1) >> 1);
                for (uint16_t kk = 0; kk < N; kk++)
                    for (int i = 0; i < MPCC_NU; i++)
                        workspace->lambda_u[kk][i] = (fixed_point_t)(((int64_t)workspace->lambda_u[kk][i] + 1) >> 1);
            } else if (dual_res / 10 > prim_res &&
                       rho > FP_HALF) {
                rho = fp_mul(rho, FP_HALF);
                if (rho_u > FP_HALF)
                    rho_u = fp_mul(rho_u, FP_HALF);
                if (rho < FP_HALF) rho = FP_HALF;
                if (rho_u < FP_HALF) rho_u = FP_HALF;
                workspace->adaptive_rho_updates++;
                /* Scale dual variables: lambda *= 2 (with saturation) */
                for (uint16_t kk = 0; kk <= N; kk++)
                    for (int i = 0; i < MPCC_NX; i++) {
                        int64_t v = (int64_t)workspace->lambda_x[kk][i] << 1;
                        workspace->lambda_x[kk][i] = sat_i64_to_i32_count(v, &workspace->numeric_clip_count);
                    }
                for (uint16_t kk = 0; kk < N; kk++)
                    for (int i = 0; i < MPCC_NU; i++) {
                        int64_t v = (int64_t)workspace->lambda_u[kk][i] << 1;
                        workspace->lambda_u[kk][i] = sat_i64_to_i32_count(v, &workspace->numeric_clip_count);
                    }
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

    memcpy(result->x_opt, workspace->z_x,
           sizeof(fixed_point_t) * (N + 1) * MPCC_NX);
    /* Output feasible controls: w_u is the ADMM projection, always
     * within bounds. The raw z_u from the Riccati forward pass is
     * unconstrained and may exceed control limits. */
    memcpy(result->u_opt, workspace->w_u,
           sizeof(fixed_point_t) * N * MPCC_NU);

    /* Persist adapted penalties for the next warm-started solve. */
    workspace->rho_state = rho;
    workspace->rho_u_state = rho_u;

    return status;
}
