/**
 * @file qp_solver_osqp.c
 * @brief OSQP QP Solver Bridge for MPCC
 *
 * Converts MPCCQPProblem_t into OSQP's sparse CSC format and solves.
 * See qp_solver_osqp.h for constraint/variable layout.
 *
 * Persistence: the OSQP workspace is kept alive between calls.
 * First call: osqp_setup + osqp_solve.
 * Subsequent calls: osqp_update_{P,A,lin_cost,bounds} + warm_start + osqp_solve.
 * This avoids re-factorizing the KKT matrix and leverages warm-starting.
 *
 * To ensure fixed sparsity patterns, P always emits the same entries
 * per column (including zeros). A also has a fixed pattern since all
 * A_k/B_k entries are always emitted.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "qp_solver_osqp.h"
#include <osqp/osqp.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/* ============================================================
 * Dimension constants
 * ============================================================ */
#define NX      MPCC_NX             /* 7  */
#define NU      MPCC_NU             /* 3  */
#define NXNU    (NX + NU)           /* 10 */
#define MAX_N   MPCC_MAX_HORIZON

/* Maximum sizes for N = MAX_N */
#define MAX_VARS       (MAX_N * NXNU + NX)
#define MAX_DYN_ROWS   (MAX_N * NX)
#define MAX_TRACK_ROWS (MAX_N + 1)
#define MAX_BOX_ROWS   MAX_VARS
#define MAX_CONSTRAINTS (MAX_DYN_ROWS + MAX_TRACK_ROWS + MAX_BOX_ROWS)

/* Upper bounds on nonzeros for the fixed sparsity patterns below. */
#define MAX_Q_P_NNZ ((NX * (NX + 1)) / 2)
#define MAX_R_P_NNZ ((NU * (NU + 1)) / 2)
#define MAX_S_P_NNZ (NX * NU)
#define MAX_P_NNZ (((MAX_N + 1) * MAX_Q_P_NNZ) + (MAX_N * (MAX_S_P_NNZ + MAX_R_P_NNZ)))
#define MAX_A_NNZ (((MAX_N + 1) * NX * (NX + 3)) + (MAX_N * NU * (NX + 1)))

/* ============================================================
 * Static workspace (avoids malloc in real-time path)
 * ============================================================ */

/* P matrix (upper triangle, CSC) */
static c_float  s_P_x[MAX_P_NNZ];
static c_int    s_P_i[MAX_P_NNZ];
static c_int    s_P_p[MAX_VARS + 1];

/* A matrix (full, CSC) */
static c_float  s_A_x[MAX_A_NNZ];
static c_int    s_A_i[MAX_A_NNZ];
static c_int    s_A_p[MAX_VARS + 1];

/* Linear cost */
static c_float  s_q[MAX_VARS];

/* Constraint bounds */
static c_float  s_l[MAX_CONSTRAINTS];
static c_float  s_u[MAX_CONSTRAINTS];

/* Warm-start: previous solution */
static c_float s_prev_x[MAX_VARS];
static int s_has_prev_solution = 0;

static int osqp_env_int(const char *name, int default_value)
{
    const char *value = getenv(name);
    return (value && value[0] != '\0') ? atoi(value) : default_value;
}

static c_float osqp_env_float(const char *name, c_float default_value)
{
    const char *value = getenv(name);
    return (value && value[0] != '\0') ? (c_float)atof(value) : default_value;
}

/* ============================================================
 * Index helpers
 * ============================================================ */

/** Variable index for x_k[ix].  Valid for k = 0..N, ix = 0..NX-1. */
static inline c_int idx_x(int k, int ix)
{
    return (c_int)(k * NXNU + ix);
}

/** Variable index for u_k[iu].  Valid for k = 0..N-1, iu = 0..NU-1. */
static inline c_int idx_u(int k, int iu)
{
    return (c_int)(k * NXNU + NX + iu);
}

/** Total number of decision variables for horizon N. */
static inline c_int n_vars(int N)
{
    return (c_int)(N * NXNU + NX);
}

/** Dynamics constraint row index for stage k, equation r. */
static inline c_int row_dyn(int k, int r)
{
    return (c_int)(k * NX + r);
}

/** Track corridor constraint row index for stage k. */
static inline c_int row_track(int N, int k)
{
    return (c_int)(N * NX + k);
}

/** Box constraint row index for variable j. */
static inline c_int row_box(int N, c_int j)
{
    return (c_int)(N * NX + (N + 1) + j);
}

static inline int append_P_entry(c_int *nnz, c_int row, c_int col, c_float value)
{
    if (row > col || *nnz >= (c_int)MAX_P_NNZ)
        return 0;

    s_P_i[*nnz] = row;
    s_P_x[*nnz] = value;
    (*nnz)++;
    return 1;
}

static inline int append_A_entry(c_int *nnz, c_int row, c_float value)
{
    if (*nnz >= (c_int)MAX_A_NNZ)
        return 0;

    s_A_i[*nnz] = row;
    s_A_x[*nnz] = value;
    (*nnz)++;
    return 1;
}

/* ============================================================
 * Build P matrix (upper triangular CSC)
 *
 * P uses fixed per-stage sparsity:
 *   stages k=0..N-1: [Q_k(7x7), S_k^T; S_k, R_k(3x3)]
 *   terminal k=N:    [Q_N(7x7)]
 * Only upper triangle stored.
 *
 * FIXED SPARSITY: always emits all possible upper-triangle entries
 * for each block, even zeros.  This ensures the sparsity pattern
 * never changes between calls (needed for osqp_update_P).
 *
 * Upper triangle Q entries per stage: full NX x NX upper triangle.
 *
 * Upper triangle R entries per stage:
 *   (0,0) (0,1) (1,1) (0,2) (1,2) (2,2)
 * ============================================================ */

static c_int build_P(const MPCCQPProblem_t *prob, c_int n)
{
    const int N = (int)prob->N;
    c_int nnz = 0;

    for (c_int col = 0; col < n; col++)
    {
        s_P_p[col] = nnz;

        int k, sub;
        if (col < (c_int)(N * NXNU))
        {
            k   = (int)(col / NXNU);
            sub = (int)(col % NXNU);
        }
        else
        {
            k   = N;
            sub = (int)(col - N * NXNU);
        }

        if (sub < NX)
        {
            /* State variable x_k[ix], ix = sub */
            int ix = sub;
            const float (*Q)[MPCC_NX] = (k < N)
                ? prob->stage_cost[k].Q
                : prob->terminal_cost.Q;

            /* Emit all Q upper-triangle entries for this column. */
            for (int er = 0; er <= ix; er++)
                if (!append_P_entry(&nnz, idx_x(k, er), col, (c_float)Q[er][ix]))
                    return -1;
        }
        else if (k < N)
        {
            /* Control variable u_k[iu], iu = sub - NX */
            int iu = sub - NX;
            const float (*R)[MPCC_NU] = prob->stage_cost[k].R;

            /* Control-state cross terms u^T S x. */
            for (int ix = 0; ix < NX; ix++)
                if (!append_P_entry(&nnz, idx_x(k, ix), col,
                                    (c_float)prob->stage_cost[k].S[iu][ix]))
                    return -1;

            /* Always emit diagonal R entries for this column */
            for (int i = 0; i <= iu; i++)
            {
                if (!append_P_entry(&nnz, idx_u(k, i), col, (c_float)R[i][iu]))
                    return -1;
            }
        }
    }

    s_P_p[n] = nnz;
    return nnz;
}

/* ============================================================
 * Build A matrix (full CSC) and bound vectors l, u
 *
 * Row layout:
 *   [0, N*NX)              : dynamics equality
 *   [N*NX, N*NX + N+1)     : track corridor
 *   [N*NX + N+1, ...)      : box constraints
 * ============================================================ */
static c_int build_A_and_bounds(const MPCCQPProblem_t *prob, c_int n)
{
    const int N = (int)prob->N;
    c_int nnz = 0;

    /* ---- Initialize bounds ---- */
    /* Dynamics equality: l = u = -d_k */
    for (int k = 0; k < N; k++)
        for (int r = 0; r < NX; r++)
        {
            c_int row = row_dyn(k, r);
            s_l[row] = (c_float)(-prob->dynamics[k].d[r]);
            s_u[row] = s_l[row];
        }

    /* Track corridor bounds */
    for (int k = 0; k <= N; k++)
    {
        c_int row = row_track(N, k);
        float phi   = prob->path_phi_ref[k];
        float x_ref = prob->path_x_ref[k];
        float y_ref = prob->path_y_ref[k];
        float C_k   = sinf(phi) * x_ref - cosf(phi) * y_ref;

        /* e_c = sin(phi) * X - cos(phi) * Y - C_k, positive to the
         * right of the reference path. Enforce -left <= e_c <= right. */
        s_l[row] = (c_float)(C_k - prob->track_left[k]);
        s_u[row] = (c_float)(C_k + prob->track_right[k]);
    }

    /* Box constraint bounds */
    for (c_int j = 0; j < n; j++)
    {
        c_int row = row_box(N, j);

        int k, sub;
        if (j < (c_int)(N * NXNU))
        {
            k   = (int)(j / NXNU);
            sub = (int)(j % NXNU);
        }
        else
        {
            k   = N;
            sub = (int)(j - N * NXNU);
        }

        if (sub < NX)
        {
            int ix = sub;
            if (k == 0)
            {
                /* Fix initial state */
                s_l[row] = (c_float)prob->x0[ix];
                s_u[row] = (c_float)prob->x0[ix];
            }
            else
            {
                s_l[row] = (c_float)prob->x_lower[ix];
                s_u[row] = (c_float)prob->x_upper[ix];

                if (ix == MPCC_IDX_S &&
                    prob->s_upper_stage[k] > prob->s_lower_stage[k])
                {
                    if (s_l[row] < (c_float)prob->s_lower_stage[k])
                        s_l[row] = (c_float)prob->s_lower_stage[k];
                    if (s_u[row] > (c_float)prob->s_upper_stage[k])
                        s_u[row] = (c_float)prob->s_upper_stage[k];
                }

            }
        }
        else if (k < N)
        {
            int iu = sub - NX;

            if (iu == MPCC_IDX_DELTA) {
                s_l[row] = (c_float)prob->delta_lower_stage[k];
                s_u[row] = (c_float)prob->delta_upper_stage[k];
            } else {
                s_l[row] = (c_float)prob->u_lower[iu];
                s_u[row] = (c_float)prob->u_upper[iu];
            }

        }
    }

    /* ---- Build A column by column ---- */
    for (c_int col = 0; col < n; col++)
    {
        s_A_p[col] = nnz;

        int k, sub;
        if (col < (c_int)(N * NXNU))
        {
            k   = (int)(col / NXNU);
            sub = (int)(col % NXNU);
        }
        else
        {
            k   = N;
            sub = (int)(col - N * NXNU);
        }

        if (sub < NX)
        {
            /* STATE variable x_k[ix] */
            int ix = sub;

            /* 1. Stage k-1 dynamics: x_k appears as -I (single row) */
            if (k > 0)
            {
                if (!append_A_entry(&nnz, row_dyn(k - 1, ix), -1.0))
                    return -1;
            }

            /* 2. Stage k dynamics: x_k appears as A_k (NX rows) */
            if (k < N)
            {
                for (int r = 0; r < NX; r++)
                {
                    float val = prob->dynamics[k].A[r][ix];
                    /* Include all entries (even zero) for fixed sparsity pattern */
                    if (!append_A_entry(&nnz, row_dyn(k, r), (c_float)val))
                        return -1;
                }
            }

            /* 3. Track corridor constraint */
            if (ix == MPCC_IDX_X || ix == MPCC_IDX_Y)
            {
                float phi = prob->path_phi_ref[k];
                c_float coeff = (ix == MPCC_IDX_X)
                    ? (c_float)sinf(phi)
                    : (c_float)(-cosf(phi));

                if (!append_A_entry(&nnz, row_track(N, k), coeff))
                    return -1;
            }

            /* 4. Box constraint (identity) */
            if (!append_A_entry(&nnz, row_box(N, col), 1.0))
                return -1;
        }
        else if (k < N)
        {
            /* CONTROL variable u_k[iu] */
            int iu = sub - NX;

            /* 1. Stage k dynamics: u_k appears as B_k (NX rows) */
            for (int r = 0; r < NX; r++)
            {
                float val = prob->dynamics[k].B[r][iu];
                if (!append_A_entry(&nnz, row_dyn(k, r), (c_float)val))
                    return -1;
            }

            /* 2. Box constraint (identity) */
            if (!append_A_entry(&nnz, row_box(N, col), 1.0))
                return -1;
        }
    }

    s_A_p[n] = nnz;
    return nnz;
}

/* ============================================================
 * Build linear cost vector q
 * ============================================================ */
static void build_q(const MPCCQPProblem_t *prob, c_int n)
{
    const int N = (int)prob->N;

    for (int k = 0; k < N; k++)
    {
        for (int ix = 0; ix < NX; ix++)
            s_q[idx_x(k, ix)] = (c_float)prob->stage_cost[k].q[ix];
        for (int iu = 0; iu < NU; iu++)
            s_q[idx_u(k, iu)] = (c_float)prob->stage_cost[k].r[iu];
    }
    /* Terminal state cost */
    for (int ix = 0; ix < NX; ix++)
        s_q[idx_x(N, ix)] = (c_float)prob->terminal_cost.q[ix];

    (void)n;
}

/* ============================================================
 * Build warm-start vector from shifted previous solution
 * ============================================================ */
static void build_warm_start(const MPCCQPProblem_t *prob, c_float *warm_x, c_int n)
{
    const int N = (int)prob->N;

    /* Shift: new stage k gets old stage k+1 */
    for (int k = 0; k < N - 1; k++)
    {
        for (int ix = 0; ix < NX; ix++)
            warm_x[idx_x(k, ix)] = s_prev_x[idx_x(k + 1, ix)];
        for (int iu = 0; iu < NU; iu++)
            warm_x[idx_u(k, iu)] = s_prev_x[idx_u(k + 1, iu)];
    }
    /* Last running stage: repeat from old terminal */
    for (int ix = 0; ix < NX; ix++)
        warm_x[idx_x(N - 1, ix)] = s_prev_x[idx_x(N, ix)];
    for (int iu = 0; iu < NU; iu++)
        warm_x[idx_u(N - 1, iu)] = s_prev_x[idx_u(N - 1, iu)];
    /* Terminal: repeat */
    for (int ix = 0; ix < NX; ix++)
        warm_x[idx_x(N, ix)] = s_prev_x[idx_x(N, ix)];

    /* Override x_0 with actual initial state (hard constraint) */
    for (int ix = 0; ix < NX; ix++)
        warm_x[idx_x(0, ix)] = (c_float)prob->x0[ix];

    (void)n;
}

/* ============================================================
 * Main solver entry point
 * ============================================================ */
MPCCStatus_t osqp_solver_solve(
    const MPCCQPProblem_t *problem,
    ADMMResult_t *result)
{
    if (!problem || !result)
    {
        if (result) result->status = MPCC_STATUS_ERROR;
        return MPCC_STATUS_ERROR;
    }
    memset(result, 0, sizeof(*result));

    const int N = (int)problem->N;
    if (N == 0 || N > MAX_N)
    {
        result->status = MPCC_STATUS_ERROR;
        return MPCC_STATUS_ERROR;
    }

    const c_int n = n_vars(N);
    const c_int m = (c_int)(N * NX + (N + 1) + n);

    /* ---- Build QP data ---- */
    c_int P_nnz = build_P(problem, n);
    if (P_nnz < 0)
    {
        fprintf(stderr, "[OSQP] P matrix assembly exceeded bounds or emitted a lower-triangular entry\n");
        result->status = MPCC_STATUS_ERROR;
        return MPCC_STATUS_ERROR;
    }
    build_q(problem, n);
    c_int A_nnz = build_A_and_bounds(problem, n);
    if (A_nnz < 0)
    {
        fprintf(stderr, "[OSQP] A matrix assembly exceeded bounds\n");
        result->status = MPCC_STATUS_ERROR;
        return MPCC_STATUS_ERROR;
    }

    /* ---- Create CSC wrappers ---- */
    csc *P = csc_matrix(n, n, P_nnz, s_P_x, s_P_i, s_P_p);
    csc *A = csc_matrix(m, n, A_nnz, s_A_x, s_A_i, s_A_p);
    if (!P || !A)
    {
        if (P) c_free(P);
        if (A) c_free(A);
        result->status = MPCC_STATUS_ERROR;
        return MPCC_STATUS_ERROR;
    }

    OSQPData data;
    data.n = n;
    data.m = m;
    data.P = P;
    data.A = A;
    data.q = s_q;
    data.l = s_l;
    data.u = s_u;

    /* ---- OSQP settings ---- */
    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.max_iter       = osqp_env_int("OSQP_MAX_ITER", 4000);
    settings.eps_abs        = osqp_env_float("OSQP_EPS_ABS", 1e-3);
    settings.eps_rel        = osqp_env_float("OSQP_EPS_REL", 1e-3);
    settings.verbose        = osqp_env_int("OSQP_VERBOSE", 0);
    settings.warm_start     = osqp_env_int("OSQP_WARM_START", 1);
    settings.adaptive_rho   = osqp_env_int("OSQP_ADAPTIVE_RHO", 1);
    settings.adaptive_rho_interval =
        osqp_env_int("OSQP_ADAPTIVE_RHO_INTERVAL", settings.adaptive_rho_interval);
    settings.adaptive_rho_tolerance =
        osqp_env_float("OSQP_ADAPTIVE_RHO_TOLERANCE", settings.adaptive_rho_tolerance);
    settings.polish         = osqp_env_int("OSQP_POLISH", 1);
    settings.polish_refine_iter =
        osqp_env_int("OSQP_POLISH_REFINE_ITER", settings.polish_refine_iter);
    settings.scaling        = osqp_env_int("OSQP_SCALING", 10);
    settings.alpha          = osqp_env_float("OSQP_ALPHA", 1.6);
    settings.rho            = osqp_env_float("OSQP_RHO", 0.1);
    settings.sigma          = osqp_env_float("OSQP_SIGMA", 1e-6);
    settings.delta          = osqp_env_float("OSQP_DELTA", settings.delta);
    settings.scaled_termination =
        osqp_env_int("OSQP_SCALED_TERMINATION", settings.scaled_termination);
    settings.check_termination =
        osqp_env_int("OSQP_CHECK_TERMINATION", settings.check_termination);
    settings.eps_prim_inf   = osqp_env_float("OSQP_EPS_PRIM_INF", 1e-15);
    settings.eps_dual_inf   = osqp_env_float("OSQP_EPS_DUAL_INF", 1e-15);

    /* ---- Setup ---- */
    OSQPWorkspace *work = NULL;
    c_int flag = osqp_setup(&work, &data, &settings);
    c_free(P);
    c_free(A);

    if (flag != 0 || !work)
    {
        fprintf(stderr, "[OSQP] Setup failed (flag=%lld)\n", (long long)flag);
        result->status = MPCC_STATUS_ERROR;
        return MPCC_STATUS_ERROR;
    }

    /* ---- Warm-start with shifted previous solution ---- */
    if (s_has_prev_solution)
    {
        c_float warm_x[MAX_VARS];
        build_warm_start(problem, warm_x, n);
        osqp_warm_start_x(work, warm_x);
    }

    /* ---- Solve ---- */
    flag = osqp_solve(work);

    /* ---- Extract results ---- */
    MPCCStatus_t status;
    c_int osqp_status = work->info->status_val;

    if (osqp_status == OSQP_SOLVED || osqp_status == OSQP_SOLVED_INACCURATE)
    {
        status = MPCC_STATUS_SUCCESS;
    }
    else if (osqp_status == OSQP_MAX_ITER_REACHED)
    {
        status = MPCC_STATUS_MAX_ITERATIONS;
    }
    else if (osqp_status == OSQP_PRIMAL_INFEASIBLE ||
             osqp_status == OSQP_PRIMAL_INFEASIBLE_INACCURATE)
    {
        status = MPCC_STATUS_INFEASIBLE;
    }
    else
    {
        status = MPCC_STATUS_ERROR;
    }

    result->status     = status;
    result->iterations = (uint16_t)work->info->iter;
    result->primal_residual = (float)work->info->pri_res;
    result->dual_residual   = (float)work->info->dua_res;
    result->rho_final       = (float)work->settings->rho;
    result->rho_u_final     = (float)work->settings->rho;
    result->adaptive_rho_updates = (uint16_t)work->info->rho_updates;
    result->adaptive_rho_state_updates = 0;
    result->adaptive_rho_control_updates = 0;
    result->numeric_clip_count   = 0;

    /* Copy solution into ADMMResult_t arrays */
    if (status == MPCC_STATUS_SUCCESS || status == MPCC_STATUS_MAX_ITERATIONS)
    {
        const c_float *sol = work->solution->x;

        for (int k = 0; k <= N; k++)
            for (int ix = 0; ix < NX; ix++)
                result->x_opt[k][ix] = (float)sol[idx_x(k, ix)];

        for (int k = 0; k < N; k++)
            for (int iu = 0; iu < NU; iu++)
                result->u_opt[k][iu] = (float)sol[idx_u(k, iu)];

        /* Save for next call's warm-start */
        memcpy(s_prev_x, sol, sizeof(c_float) * (size_t)n);
        s_has_prev_solution = 1;
    }

#ifdef MPCC_DEBUG_PRINT
    printf("[OSQP] status=%lld iter=%lld pri_res=%.2e dua_res=%.2e\n",
           (long long)osqp_status,
           (long long)work->info->iter,
           work->info->pri_res,
           work->info->dua_res);
#endif

    osqp_cleanup(work);

    result->status = status;
    return status;
}
