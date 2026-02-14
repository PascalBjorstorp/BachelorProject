/**
 * @file qp_solver.c
 * @brief Projected Gradient Descent QP Solver Implementation
 *
 * Solves convex quadratic programs using projected gradient descent.
 * Designed for real-time MPC with deterministic execution time.
 *
 * Algorithm:
 * 1. Start with u = 0
 * 2. Compute gradient: gradient = H×u + f
 * 3. Take gradient step: u_new = u - step_size × gradient
 * 4. Project onto feasible region: enforce A×u ≤ b
 * 5. Check convergence: ||u_new - u|| < tolerance
 * 6. Repeat until converged or maximum iterations reached
 */

#include "qp_solver.h"
#include "fp_math.h"
#include <string.h>

/*===========================================================================
 * Internal Constants
 *===========================================================================*/

/*===========================================================================
 * Internal Helper: Project onto Feasible Region
 *===========================================================================*/

/**
 * Project the solution onto the feasible region defined by A×u ≤ b.

 * Since our MPC uses box constraints (each row of A has a single ±1),
 * the correct projection is simple per-variable clamping:
 *   If A[i,:] * u > b[i], clamp the offending variable.
 *
 * For a row with A[i,j] = +1: u[j] = min(u[j], b[i])
 * For a row with A[i,j] = -1: u[j] = max(u[j], -b[i])
 *
 * @param variable_vector     Solution vector to project (modified in place)
 * @param constraint_matrix   Matrix A (constraint_count × variable_count)
 * @param constraint_bounds   Vector b (length constraint_count)
 * @param variable_count      Number of optimization variables
 * @param constraint_count    Number of constraints
 */
static void project_onto_feasible_region(
    fixed_point_t *variable_vector,
    const fixed_point_t *constraint_matrix,
    const fixed_point_t *constraint_bounds,
    uint16_t variable_count,
    uint16_t constraint_count)
{
    /*
     * For each constraint row, find the non-zero column and clamp.
     * Box constraints have exactly one non-zero entry per row.
     */
    for (uint16_t constraint_index = 0; constraint_index < constraint_count; constraint_index++)
    {
        const fixed_point_t *row = &constraint_matrix[constraint_index * variable_count];
        fixed_point_t bound = constraint_bounds[constraint_index];

        /* Find the non-zero entry in this constraint row */
        for (uint16_t var_index = 0; var_index < variable_count; var_index++)
        {
            if (row[var_index] > 0)
            {
                /* Constraint: +1 * u[j] <= b  →  u[j] = min(u[j], b) */
                if (variable_vector[var_index] > bound)
                {
                    variable_vector[var_index] = bound;
                }
                break;  /* Only one non-zero per row in box constraints */
            }
            else if (row[var_index] < 0)
            {
                /* Constraint: -1 * u[j] <= b  →  u[j] >= -b  →  u[j] = max(u[j], -b) */
                fixed_point_t lower = fp_neg(bound);
                if (variable_vector[var_index] < lower)
                {
                    variable_vector[var_index] = lower;
                }
                break;
            }
        }
    }
}

/*===========================================================================
 * Main Solver Implementation
 *===========================================================================*/

QuadraticProgramStatus_t qp_solver_solve(
    const QuadraticProgramProblem_t *problem,
    const QuadraticProgramConfig_t *config,
    QuadraticProgramSolution_t *solution)
{
    uint16_t variable_count = problem->variable_count;
    uint16_t constraint_count = problem->constraint_count;

    /* Temporary arrays for computation */
    fixed_point_t gradient[QP_MAXIMUM_VARIABLES];
    fixed_point_t next_variables[QP_MAXIMUM_VARIABLES];
    fixed_point_t hessian_times_variables[QP_MAXIMUM_VARIABLES];

    /* Initialize solution to zero */
    memset(solution->optimal_variables, 0, variable_count * sizeof(fixed_point_t));
    memset(solution->dual_variables, 0, constraint_count * sizeof(fixed_point_t));

    solution->iteration_count = 0;
    solution->status = QP_STATUS_ERROR;

    /*
     * Compute per-variable step sizes using Gershgorin row sums.
     *
     * For projected gradient descent, convergence requires:
     *   step_size < 2 / lambda_max(H)
     *
     * Using a GLOBAL step size is problematic because different variables
     * (steering vs velocity) have very different Hessian eigenvalues.
     * At high velocity, steering Hessian diagonal ≈ 184, while
     * velocity diagonal ≈ 2.4, requiring step < 0.001 globally — far
     * too slow for velocity convergence.
     *
     * Solution: Use per-variable step sizes based on Gershgorin row sums.
     * For each variable i, the step size is:
     *
     *   step[i] = 1 / gershgorin_radius[i]
     *   gershgorin_radius[i] = |H[i][i]| + Σ_{j≠i} |H[i][j]|
     *
     * This guarantees convergence for all variables simultaneously while
     * allowing each to converge at its own natural rate.
     *
     * CRITICAL FIX: The previous fixed step_size of 0.03 caused divergence
     * at high velocities where the steering Hessian eigenvalue >> 2/0.03.
     * This was the root cause of the "car randomly turning" bug.
     */
    fixed_point_t inv_row_sum[QP_MAXIMUM_VARIABLES];

    for (uint16_t i = 0; i < variable_count; i++)
    {
        /* Compute Gershgorin row sum: |H[i][i]| + sum_{j!=i} |H[i][j]| */
        fixed_point_t row_sum = 0;
        for (uint16_t j = 0; j < variable_count; j++)
        {
            fixed_point_t hij = problem->hessian_matrix[i * variable_count + j];
            if (hij < 0) hij = fp_neg(hij);
            row_sum = fp_add(row_sum, hij);
        }

        /* step[i] = 1 / row_sum, with minimum to avoid division by zero */
        if (row_sum > FP_ONE)
        {
            inv_row_sum[i] = fp_div(FP_ONE, row_sum);
        }
        else
        {
            inv_row_sum[i] = config->gradient_step_size;
        }
    }

    /* Main optimization loop */
    for (int iteration = 0; iteration < config->maximum_iterations; iteration++)
    {
        solution->iteration_count = iteration;

        /*
         * Step 1: Compute gradient = H×u + f
         */
        fp_mat_vec_mul(
            problem->hessian_matrix,
            solution->optimal_variables,
            hessian_times_variables,
            variable_count,
            variable_count);

        for (uint16_t index = 0; index < variable_count; index++)
        {
            gradient[index] = fp_add(
                hessian_times_variables[index],
                problem->linear_cost_vector[index]);
        }

        /*
         * Step 2: Gradient descent step with per-variable Gershgorin steps.
         *
         * u_new[i] = u[i] - step[i] × gradient[i]
         *
         * Each variable uses its own step size based on the Gershgorin
         * row sum, ensuring convergence regardless of conditioning.
         */
        for (uint16_t index = 0; index < variable_count; index++)
        {
            next_variables[index] = fp_sub(
                solution->optimal_variables[index],
                fp_mul(inv_row_sum[index], gradient[index]));
        }

        /*
         * Step 3: Project onto feasible region (enforce A×u ≤ b)
         */
        project_onto_feasible_region(
            next_variables,
            problem->constraint_matrix,
            problem->constraint_bounds,
            variable_count,
            constraint_count);

        /*
         * Step 4: Check convergence ||u_new - u||
         */
        fixed_point_t step_norm_squared = 0;

        for (uint16_t index = 0; index < variable_count; index++)
        {
            fixed_point_t difference = fp_sub(
                next_variables[index],
                solution->optimal_variables[index]);

            fixed_point_t difference_squared = fp_mul(difference, difference);
            step_norm_squared = fp_add(step_norm_squared, difference_squared);
        }

        fixed_point_t step_norm = fp_sqrt(step_norm_squared);

        /* Update solution with new variables */
        memcpy(solution->optimal_variables, next_variables,
               variable_count * sizeof(fixed_point_t));

        /*
         * Step 5: Compute constraint residual
         */
        solution->constraint_residual = fp_max_violation(
            problem->constraint_matrix,
            solution->optimal_variables,
            problem->constraint_bounds,
            constraint_count,
            variable_count);

        /*
         * Step 6: Check termination criteria
         */
        if (step_norm < config->convergence_tolerance &&
            solution->constraint_residual < config->convergence_tolerance)
        {
            solution->status = QP_STATUS_OPTIMAL;
            return QP_STATUS_OPTIMAL;
        }
    }

    /*
     * Maximum iterations reached - check if solution is at least feasible
     */
    solution->constraint_residual = fp_max_violation(
        problem->constraint_matrix,
        solution->optimal_variables,
        problem->constraint_bounds,
        constraint_count,
        variable_count);

    if (solution->constraint_residual < config->convergence_tolerance)
    {
        /* Feasible but may not be optimal */
        solution->status = QP_STATUS_MAXIMUM_ITERATIONS_REACHED;
        return QP_STATUS_MAXIMUM_ITERATIONS_REACHED;
    }

    /* Could not find feasible solution */
    solution->status = QP_STATUS_INFEASIBLE;
    return QP_STATUS_INFEASIBLE;
}

/*===========================================================================
 * Initialization Functions
 *===========================================================================*/

void qp_solver_initialize_problem(QuadraticProgramProblem_t *problem)
{
    memset(problem->hessian_matrix, 0,
           QP_MAXIMUM_VARIABLES * QP_MAXIMUM_VARIABLES * sizeof(fixed_point_t));
    memset(problem->linear_cost_vector, 0,
           QP_MAXIMUM_VARIABLES * sizeof(fixed_point_t));
    memset(problem->constraint_matrix, 0,
           QP_MAXIMUM_CONSTRAINTS * QP_MAXIMUM_VARIABLES * sizeof(fixed_point_t));
    memset(problem->constraint_bounds, 0,
           QP_MAXIMUM_CONSTRAINTS * sizeof(fixed_point_t));

    problem->variable_count = 0;
    problem->constraint_count = 0;
}

void qp_solver_initialize_config(QuadraticProgramConfig_t *config)
{
    /*
     * Step size (legacy parameter, kept for API compatibility).
     *
     * The solver now uses per-variable adaptive step sizes based on
     * Gershgorin row sums (computed at the start of each solve call).
     * This field serves as a fallback when the Gershgorin row sum is
     * too small (< 1.0) but is otherwise unused.
     *
     * Historical note: A fixed step of 0.03 caused gradient descent to
     * DIVERGE at high velocities where the Hessian eigenvalue >> 2/0.03.
     * At v=20 m/s, the steering Hessian row sum reaches ~1000, requiring
     * step < 0.002. The fixed step of 0.03 made steering oscillate
     * between ±max every iteration — the root cause of the "car randomly
     * turning" bug. The Gershgorin adaptive step fix eliminates this.
     */
    config->gradient_step_size = (fixed_point_t)1966;  /* 0.03 in Q16.16 = 0.03 × 65536 ≈ 1966 */

    /* Convergence tolerance of 0.001 (about 65 in fixed-point) */
    config->convergence_tolerance = (fixed_point_t)65;  /* 0.001 in Q16.16 ≈ 65 */

    /* Maximum iterations */
    config->maximum_iterations = QP_MAXIMUM_ITERATIONS;

    /* Verbose output disabled by default */
    config->enable_verbose_output = 0;
}
