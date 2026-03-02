/**
 * @file qp_solver_admm.c
 * @brief ADMM Quadratic Programming Solver (Dense, Q16.16 Fixed-Point)
 *
 * Solves box-constrained QPs using the Alternating Direction Method of
 * Multipliers (ADMM). The Hessian is factored once via Cholesky, then
 * each ADMM iteration uses forward/backward substitution (O(n²)) plus
 * element-wise clipping (O(n)).
 *
 * Convergence is checked via primal/dual residual norms.
 *
 * All arithmetic uses Q16.16 fixed-point. The Cholesky factorization
 * uses int64_t intermediates to avoid overflow on 40×40 matrices.
 */

#include "../include/qp_solver_admm.h"
#include <string.h>

/*===========================================================================
 * Configuration defaults
 *===========================================================================*/

void admm_config_init(AdmmConfig_t *config)
{
    config->rho            = ADMM_DEFAULT_RHO;
    config->alpha          = ADMM_DEFAULT_ALPHA;
    config->abs_tolerance  = ADMM_DEFAULT_ABS_TOL;
    config->rel_tolerance  = ADMM_DEFAULT_REL_TOL;
    config->max_iterations = ADMM_DEFAULT_MAX_ITER;
}

void admm_state_init(AdmmState_t *state)
{
    memset(state, 0, sizeof(*state));
    state->initialized = 0;
    state->n = 0;
}

/*===========================================================================
 * Cholesky Factorization (Q16.16)
 *===========================================================================
 * Computes L such that (H + rho*I) = L * L^T.
 * Uses int64_t for intermediate products to avoid overflow.
 *
 * L is stored row-major: L[i][j] at L[i * n + j], j <= i.
 * Upper triangle of L is zero.
 *===========================================================================*/

/**
 * Fixed-point square root for Cholesky diagonal.
 * Uses Newton's method with int64_t arithmetic.
 * Input and output are Q16.16.
 */
static fixed_point_t fp_sqrt_chol(int64_t val)
{
    if (val <= 0) return 0;

    /* Initial guess: shift to get approximate integer sqrt, then adjust */
    int64_t guess = 1LL << 16;  /* Start at 1.0 in Q16.16 */

    /* Scale guess to be in the right ballpark */
    if (val > (1LL << 40)) guess = 1LL << 24;
    else if (val > (1LL << 32)) guess = 1LL << 20;
    else if (val > (1LL << 24)) guess = 1LL << 16;
    else guess = 1LL << 12;

    /* Newton iterations: guess = (guess + val/guess) / 2
     * We need sqrt in Q16.16 of a Q16.16 value.
     * sqrt(val_q16) in Q16.16 = sqrt(val_q16 << 16) as integer */
    int64_t val_shifted = val << 16;  /* Q32.16 for proper Q16.16 sqrt */

    for (int i = 0; i < 20; i++) {
        if (guess == 0) break;
        int64_t next = (guess + val_shifted / guess) / 2;
        if (next >= guess - 1 && next <= guess + 1) break;
        guess = next;
    }

    return (fixed_point_t)guess;
}

int admm_cholesky_factorize(
    const fixed_point_t *H, fixed_point_t rho,
    int n, fixed_point_t *L)
{
    /* Zero out L */
    memset(L, 0, (size_t)n * (size_t)n * sizeof(fixed_point_t));

    for (int i = 0; i < n; i++) {
        /* Diagonal: L[i][i] = sqrt(H[i][i] + rho - sum_{k<i} L[i][k]^2) */
        int64_t sum = 0;
        for (int k = 0; k < i; k++) {
            int64_t lik = (int64_t)L[i * n + k];
            sum += (lik * lik) >> FP_FRAC_BITS;
        }

        int64_t diag = (int64_t)H[i * n + i] + (int64_t)rho - sum;
        if (diag <= 0) {
            /* Not positive definite */
            return -1;
        }

        fixed_point_t lii = fp_sqrt_chol(diag);
        if (lii == 0) return -1;
        L[i * n + i] = lii;

        /* Off-diagonal: L[j][i] = (H[j][i] - sum_{k<i} L[j][k]*L[i][k]) / L[i][i] */
        for (int j = i + 1; j < n; j++) {
            int64_t s = 0;
            for (int k = 0; k < i; k++) {
                s += ((int64_t)L[j * n + k] * (int64_t)L[i * n + k]) >> FP_FRAC_BITS;
            }
            int64_t num = (int64_t)H[j * n + i] - s;
            /* L[j][i] = num / lii, both in Q16.16 */
            L[j * n + i] = (fixed_point_t)((num << FP_FRAC_BITS) / (int64_t)lii);
        }
    }

    return 0;
}

/*===========================================================================
 * Cholesky Solve: L L^T x = b
 *===========================================================================
 * 1. Forward substitution: L y = b
 * 2. Backward substitution: L^T x = y
 *===========================================================================*/

void admm_cholesky_solve(
    const fixed_point_t *L, const fixed_point_t *b,
    int n, fixed_point_t *x)
{
    /* Forward substitution: L y = b */
    fixed_point_t y[QP_MAXIMUM_VARIABLES];

    for (int i = 0; i < n; i++) {
        int64_t sum = 0;
        for (int k = 0; k < i; k++) {
            sum += ((int64_t)L[i * n + k] * (int64_t)y[k]) >> FP_FRAC_BITS;
        }
        int64_t num = (int64_t)b[i] - sum;
        int64_t denom = (int64_t)L[i * n + i];
        if (denom == 0) {
            y[i] = 0;
        } else {
            y[i] = (fixed_point_t)((num << FP_FRAC_BITS) / denom);
        }
    }

    /* Backward substitution: L^T x = y */
    for (int i = n - 1; i >= 0; i--) {
        int64_t sum = 0;
        for (int k = i + 1; k < n; k++) {
            sum += ((int64_t)L[k * n + i] * (int64_t)x[k]) >> FP_FRAC_BITS;
        }
        int64_t num = (int64_t)y[i] - sum;
        int64_t denom = (int64_t)L[i * n + i];
        if (denom == 0) {
            x[i] = 0;
        } else {
            x[i] = (fixed_point_t)((num << FP_FRAC_BITS) / denom);
        }
    }
}

/*===========================================================================
 * Extract Box Constraints from QP Problem
 *===========================================================================
 * The projected gradient solver uses A×u ≤ b with identity rows for
 * box constraints (upper and lower bounds on each variable).
 * We extract these into simple lb/ub arrays for ADMM's clipping step.
 *===========================================================================*/

static void extract_box_bounds(
    const QuadraticProgramProblem_t *problem,
    int n,
    fixed_point_t *lb, fixed_point_t *ub)
{
    /* Default: very wide bounds */
    const fixed_point_t BIG = FP_CONST(100.0);
    for (int i = 0; i < n; i++) {
        lb[i] = -BIG;
        ub[i] =  BIG;
    }

    /* Scan constraint matrix for identity-row patterns.
     * A row with exactly one non-zero entry A[row][j] = +1 or -1
     * encodes a box constraint:
     *   +1: u[j] ≤ b[row]  (upper bound)
     *   -1: -u[j] ≤ b[row], i.e. u[j] ≥ -b[row]  (lower bound)
     */
    int nc = problem->constraint_count;
    for (int row = 0; row < nc; row++) {
        /* Find non-zero entries in this constraint row */
        int nz_col = -1;
        int nz_count = 0;
        fixed_point_t nz_val = 0;

        for (int j = 0; j < n; j++) {
            fixed_point_t val = problem->constraint_matrix[row * n + j];
            if (val != 0) {
                nz_col = j;
                nz_val = val;
                nz_count++;
                if (nz_count > 1) break;  /* Not a box constraint row */
            }
        }

        if (nz_count == 1 && nz_col >= 0) {
            fixed_point_t bound = problem->constraint_bounds[row];
            if (nz_val > 0) {
                /* +a*u[j] <= b  →  u[j] <= b/a */
                fixed_point_t ub_val = fp_div(bound, nz_val);
                if (ub_val < ub[nz_col]) ub[nz_col] = ub_val;
            } else {
                /* -a*u[j] <= b  →  u[j] >= -b/a */
                fixed_point_t lb_val = fp_div(-bound, -nz_val);
                /* lb_val = -b / |a|, but we need u[j] >= -b/|a| */
                fixed_point_t neg_nz = -nz_val;
                lb_val = -fp_div(bound, neg_nz);
                if (lb_val > lb[nz_col]) lb[nz_col] = lb_val;
            }
        }
    }
}

/*===========================================================================
 * ADMM Solve
 *===========================================================================*/

QuadraticProgramStatus_t admm_solve(
    const QuadraticProgramProblem_t *problem,
    const AdmmConfig_t *config,
    AdmmState_t *state,
    QuadraticProgramSolution_t *solution)
{
    int n = problem->variable_count;
    if (n <= 0 || n > QP_MAXIMUM_VARIABLES) {
        solution->status = QP_STATUS_ERROR;
        return QP_STATUS_ERROR;
    }

    fixed_point_t rho = config->rho;

    /* --- Step 0: Cholesky factorize (H + rho * I) --- */
    /* Redo factorization if dimensions changed or first call */
    if (!state->initialized || state->n != n) {
        int ret = admm_cholesky_factorize(
            problem->hessian_matrix, rho, n, state->L);
        if (ret < 0) {
            solution->status = QP_STATUS_ERROR;
            return QP_STATUS_ERROR;
        }
        /* Reset warm-start on new factorization */
        memset(state->z, 0, sizeof(state->z));
        memset(state->u, 0, sizeof(state->u));
        state->n = n;
        state->initialized = 1;
    } else {
        /* Re-factorize with new Hessian (changes each MPC call) */
        int ret = admm_cholesky_factorize(
            problem->hessian_matrix, rho, n, state->L);
        if (ret < 0) {
            solution->status = QP_STATUS_ERROR;
            return QP_STATUS_ERROR;
        }
    }

    /* --- Extract box bounds from constraint structure --- */
    fixed_point_t lb[QP_MAXIMUM_VARIABLES];
    fixed_point_t ub[QP_MAXIMUM_VARIABLES];
    extract_box_bounds(problem, n, lb, ub);

    /* --- Initialize primal from warm-start or problem initial point --- */
    fixed_point_t x[QP_MAXIMUM_VARIABLES];
    fixed_point_t z[QP_MAXIMUM_VARIABLES];
    fixed_point_t u[QP_MAXIMUM_VARIABLES];
    fixed_point_t z_old[QP_MAXIMUM_VARIABLES];

    if (problem->use_warm_start) {
        for (int i = 0; i < n; i++) {
            x[i] = problem->initial_point[i];
            z[i] = state->z[i];
            u[i] = state->u[i];
        }
    } else {
        memset(x, 0, (size_t)n * sizeof(fixed_point_t));
        memset(z, 0, (size_t)n * sizeof(fixed_point_t));
        memset(u, 0, (size_t)n * sizeof(fixed_point_t));
    }

    /* --- ADMM iterations --- */
    fixed_point_t alpha_relax = config->alpha;
    int max_iter = config->max_iterations;
    QuadraticProgramStatus_t status = QP_STATUS_MAXIMUM_ITERATIONS_REACHED;

    for (int iter = 0; iter < max_iter; iter++) {

        /* --- x-update: x = (H + rho*I)^{-1} * (-f + rho*(z - u)) --- */
        fixed_point_t rhs[QP_MAXIMUM_VARIABLES];
        for (int i = 0; i < n; i++) {
            /* rhs[i] = -f[i] + rho * (z[i] - u[i]) */
            int64_t r = -(int64_t)problem->linear_cost_vector[i]
                        + (((int64_t)rho * ((int64_t)z[i] - (int64_t)u[i])) >> FP_FRAC_BITS);
            rhs[i] = (fixed_point_t)r;
        }
        admm_cholesky_solve(state->L, rhs, n, x);

        /* --- z-update with over-relaxation --- */
        memcpy(z_old, z, (size_t)n * sizeof(fixed_point_t));

        for (int i = 0; i < n; i++) {
            /* Over-relaxation: x_hat = alpha * x + (1 - alpha) * z_old */
            int64_t x_hat = ((int64_t)alpha_relax * (int64_t)x[i]) >> FP_FRAC_BITS;
            x_hat += (((int64_t)(FP_ONE - alpha_relax) * (int64_t)z_old[i]) >> FP_FRAC_BITS);

            /* z = clip(x_hat + u, lb, ub) */
            int64_t val = x_hat + (int64_t)u[i];
            if (val < (int64_t)lb[i]) val = (int64_t)lb[i];
            if (val > (int64_t)ub[i]) val = (int64_t)ub[i];
            z[i] = (fixed_point_t)val;
        }

        /* --- u-update: u = u + x_hat - z --- */
        for (int i = 0; i < n; i++) {
            int64_t x_hat = ((int64_t)alpha_relax * (int64_t)x[i]) >> FP_FRAC_BITS;
            x_hat += (((int64_t)(FP_ONE - alpha_relax) * (int64_t)z_old[i]) >> FP_FRAC_BITS);
            u[i] = (fixed_point_t)((int64_t)u[i] + x_hat - (int64_t)z[i]);
        }

        /* --- Convergence check --- */
        /* Primal residual: ||x - z||_inf */
        /* Dual residual: ||rho * (z - z_old)||_inf */
        fixed_point_t primal_res = 0;
        fixed_point_t dual_res = 0;
        for (int i = 0; i < n; i++) {
            fixed_point_t pr = x[i] - z[i];
            if (pr < 0) pr = -pr;
            if (pr > primal_res) primal_res = pr;

            int64_t dr64 = ((int64_t)rho * ((int64_t)z[i] - (int64_t)z_old[i])) >> FP_FRAC_BITS;
            fixed_point_t dr = (fixed_point_t)(dr64 < 0 ? -dr64 : dr64);
            if (dr > dual_res) dual_res = dr;
        }

        /* Check convergence */
        if (primal_res <= config->abs_tolerance && dual_res <= config->abs_tolerance) {
            status = QP_STATUS_OPTIMAL;
            solution->iteration_count = iter + 1;
            break;
        }

        solution->iteration_count = iter + 1;
    }

    /* --- Store solution --- */
    for (int i = 0; i < n; i++) {
        solution->optimal_variables[i] = z[i];  /* z is the feasible point */
    }
    solution->status = status;
    solution->constraint_residual = 0;

    /* Compute constraint residual */
    for (int i = 0; i < n; i++) {
        fixed_point_t viol_lo = lb[i] - z[i];
        fixed_point_t viol_hi = z[i] - ub[i];
        if (viol_lo > solution->constraint_residual) solution->constraint_residual = viol_lo;
        if (viol_hi > solution->constraint_residual) solution->constraint_residual = viol_hi;
    }

    /* Save state for warm-starting next call */
    memcpy(state->z, z, (size_t)n * sizeof(fixed_point_t));
    memcpy(state->u, u, (size_t)n * sizeof(fixed_point_t));

    return status;
}
