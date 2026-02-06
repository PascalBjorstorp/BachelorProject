/**
 * @file qp_solver.c
 * @brief Projected Gradient Descent QP Solver
 *
 * Algorithm:
 * 1. Start with x = 0
 * 2. Compute gradient: g = H×x + f
 * 3. Take step: x_new = x - step × g
 * 4. Project onto feasible region (box constraints)
 * 5. Check convergence
 */

#include "qp_solver.h"
#include "fp_math.h"
#include <string.h>

/*===========================================================================
 * Box Constraint Projection
 *===========================================================================*/

/**
 * Project onto feasible region using simple clamping.
 * Assumes box constraints (each row of A has single ±1).
 */
static void project_box(
    fixed_point_t *x,
    const fixed_point_t *A,
    const fixed_point_t *b,
    uint16_t n_vars,
    uint16_t n_constraints)
{
    for (uint16_t ci = 0; ci < n_constraints; ci++)
    {
        const fixed_point_t *row = &A[ci * n_vars];
        fixed_point_t bound = b[ci];

        for (uint16_t vi = 0; vi < n_vars; vi++)
        {
            if (row[vi] > 0)
            {
                /* +1 × x[vi] <= b → x[vi] = min(x[vi], b) */
                if (x[vi] > bound)
                    x[vi] = bound;
                break;
            }
            else if (row[vi] < 0)
            {
                /* -1 × x[vi] <= b → x[vi] >= -b */
                fixed_point_t lower = fp_neg(bound);
                if (x[vi] < lower)
                    x[vi] = lower;
                break;
            }
        }
    }
}

/*===========================================================================
 * Main Solver
 *===========================================================================*/

SolverStatus_t qp_solve(
    const QpProblem_t *problem,
    const QpConfig_t *config,
    QpSolution_t *solution)
{
    uint16_t n = problem->n_vars;
    uint16_t m = problem->n_constraints;

    fixed_point_t grad[QP_MAX_VARS];
    fixed_point_t x_next[QP_MAX_VARS];
    fixed_point_t Hx[QP_MAX_VARS];

    /* Initialize x = 0 */
    memset(solution->x, 0, n * sizeof(fixed_point_t));
    memset(solution->dual, 0, m * sizeof(fixed_point_t));
    solution->iterations = 0;
    solution->status = SOLVER_ERROR;

    /* Main optimization loop */
    for (int iter = 0; iter < config->max_iter; iter++)
    {
        solution->iterations = iter;

        /* gradient = H×x + f */
        fp_mat_vec_mul(problem->H, solution->x, Hx, n, n);
        for (uint16_t i = 0; i < n; i++)
            grad[i] = fp_add(Hx[i], problem->f[i]);

        /* x_next = x - step × grad */
        fixed_point_t neg_step = fp_neg(config->step_size);
        fp_vec_add_scaled(solution->x, grad, neg_step, x_next, n);

        /* Project onto box constraints */
        project_box(x_next, problem->A, problem->b, n, m);

        /* Check convergence: ||x_next - x|| */
        fixed_point_t norm_sq = 0;
        for (uint16_t i = 0; i < n; i++)
        {
            fixed_point_t diff = fp_sub(x_next[i], solution->x[i]);
            norm_sq = fp_add(norm_sq, fp_mul(diff, diff));
        }
        fixed_point_t norm = fp_sqrt(norm_sq);

        /* Update solution */
        memcpy(solution->x, x_next, n * sizeof(fixed_point_t));

        /* Compute constraint residual */
        solution->residual = fp_max_violation(
            problem->A, solution->x, problem->b, m, n);

        /* Check termination */
        if (norm < config->tolerance && solution->residual < config->tolerance)
        {
            solution->status = SOLVER_SUCCESS;
            return SOLVER_SUCCESS;
        }
    }

    /* Max iterations reached */
    solution->residual = fp_max_violation(
        problem->A, solution->x, problem->b, m, n);

    if (solution->residual < config->tolerance)
    {
        solution->status = SOLVER_MAX_ITER;
        return SOLVER_MAX_ITER;
    }

    solution->status = SOLVER_INFEASIBLE;
    return SOLVER_INFEASIBLE;
}

/*===========================================================================
 * Initialization Functions
 *===========================================================================*/

void qp_init_problem(QpProblem_t *problem)
{
    memset(problem->H, 0, QP_MAX_VARS * QP_MAX_VARS * sizeof(fixed_point_t));
    memset(problem->f, 0, QP_MAX_VARS * sizeof(fixed_point_t));
    memset(problem->A, 0, QP_MAX_CONSTRAINTS * QP_MAX_VARS * sizeof(fixed_point_t));
    memset(problem->b, 0, QP_MAX_CONSTRAINTS * sizeof(fixed_point_t));
    problem->n_vars = 0;
    problem->n_constraints = 0;
}

void qp_init_config(QpConfig_t *config)
{
    config->step_size = FP_CONST(0.05);
    config->tolerance = FP_CONST(0.001);
    config->max_iter = QP_MAX_ITER;
    config->verbose = 0;
}
