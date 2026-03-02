/**
 * @file qp_solver_admm.h
 * @brief ADMM-Based Quadratic Programming Solver (Dense, Fixed-Point)
 *
 * Drop-in replacement for qp_solver.h using the Alternating Direction
 * Method of Multipliers (ADMM). Key advantages over projected gradient:
 *   - Better convergence on ill-conditioned problems
 *   - Natural handling of inequality constraints via splitting
 *   - FPGA-friendly: each iteration is a linear solve + element-wise clip
 *
 * Algorithm:
 *   min  0.5 x^T H x + f^T x
 *   s.t. lb <= x <= ub     (box constraints from problem bounds)
 *        C x <= d          (inequality constraints)
 *
 * ADMM Splitting:
 *   x-update: x = (H + rho*I)^{-1} * (-f + rho*(z - u))
 *   z-update: z = clip(x + u, lb, ub)  [+ constraint projection]
 *   u-update: u = u + x - z
 *
 * The (H + rho*I) factorization is done once per QP via Cholesky,
 * then reused for all ADMM iterations (forward/backward substitution).
 */

#ifndef QP_SOLVER_ADMM_H
#define QP_SOLVER_ADMM_H

#include "qp_solver.h"  /* Reuse existing problem/solution types */
#include "fp_math.h"

/*===========================================================================
 * ADMM-Specific Configuration
 *===========================================================================*/

/** Default ADMM penalty parameter rho (Q16.16) */
#define ADMM_DEFAULT_RHO         FP_CONST(10.0)

/** Default over-relaxation parameter alpha (1.0 = no relaxation, 1.5-1.8 typical) */
#define ADMM_DEFAULT_ALPHA       FP_CONST(1.6)

/** Default absolute tolerance */
#define ADMM_DEFAULT_ABS_TOL     FP_CONST(0.001)

/** Default relative tolerance */
#define ADMM_DEFAULT_REL_TOL     FP_CONST(0.001)

/** Maximum ADMM iterations */
#define ADMM_DEFAULT_MAX_ITER    50

typedef struct
{
    /** ADMM penalty parameter rho. Larger = faster constraint satisfaction
     *  but slower cost minimization. Typical: 1.0 - 100.0 */
    fixed_point_t rho;

    /** Over-relaxation parameter alpha. 1.0 = standard, 1.5-1.8 = accelerated.
     *  Must be in (0, 2) for convergence. */
    fixed_point_t alpha;

    /** Primal residual tolerance for convergence */
    fixed_point_t abs_tolerance;

    /** Relative tolerance for convergence */
    fixed_point_t rel_tolerance;

    /** Maximum ADMM iterations */
    int max_iterations;

} AdmmConfig_t;

/*===========================================================================
 * Internal State (for warm-starting between MPC calls)
 *===========================================================================*/

typedef struct
{
    /** Cholesky factor L of (H + rho*I) = L L^T, stored row-major lower triangular */
    fixed_point_t L[QP_MAXIMUM_VARIABLES * QP_MAXIMUM_VARIABLES];

    /** Dual variables from previous solve (warm start) */
    fixed_point_t u[QP_MAXIMUM_VARIABLES];

    /** Slack variables from previous solve (warm start) */
    fixed_point_t z[QP_MAXIMUM_VARIABLES];

    /** Whether factorization and warm-start data is valid */
    int initialized;

    /** Number of variables in the factorization */
    int n;

} AdmmState_t;

/*===========================================================================
 * API
 *===========================================================================*/

/**
 * Initialize ADMM configuration with default values.
 */
void admm_config_init(AdmmConfig_t *config);

/**
 * Initialize ADMM solver state.
 */
void admm_state_init(AdmmState_t *state);

/**
 * Solve a box-constrained QP using ADMM.
 *
 * Uses the same QuadraticProgramProblem_t input as the projected gradient
 * solver, but internally uses ADMM for better convergence.
 *
 * The box constraints are derived from the problem's constraint_matrix
 * and constraint_bounds (identity rows for variable bounds).
 *
 * @param problem   QP problem (H, f, variable bounds, constraints)
 * @param config    ADMM configuration
 * @param state     ADMM internal state (factorization, warm-start)
 * @param solution  Output: optimal variables, iterations, status
 * @return Solver status code
 */
QuadraticProgramStatus_t admm_solve(
    const QuadraticProgramProblem_t *problem,
    const AdmmConfig_t *config,
    AdmmState_t *state,
    QuadraticProgramSolution_t *solution);

/**
 * Perform Cholesky factorization of (H + rho*I).
 * Called automatically by admm_solve when factorization is needed,
 * but can be called explicitly if the same H is reused.
 *
 * @param H     Symmetric PD matrix (n x n), row-major
 * @param rho   ADMM penalty parameter
 * @param n     Matrix dimension
 * @param L     Output: lower triangular Cholesky factor (n x n)
 * @return 0 on success, -1 if not positive definite
 */
int admm_cholesky_factorize(
    const fixed_point_t *H, fixed_point_t rho,
    int n, fixed_point_t *L);

/**
 * Solve L L^T x = b using forward/backward substitution.
 *
 * @param L     Lower triangular Cholesky factor (n x n)
 * @param b     Right-hand side vector (length n)
 * @param n     System dimension
 * @param x     Output: solution vector (length n)
 */
void admm_cholesky_solve(
    const fixed_point_t *L, const fixed_point_t *b,
    int n, fixed_point_t *x);

#endif /* QP_SOLVER_ADMM_H */
