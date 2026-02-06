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
#include "linear_algebra.h"
#include <string.h>

/*===========================================================================
 * Internal Constants
 *===========================================================================*/

/** Number of projection iterations for constraint enforcement */
#define PROJECTION_ITERATIONS 20

/** Small epsilon to avoid division by zero in projection */
#define PROJECTION_EPSILON 100

/*===========================================================================
 * Internal Helper: Project onto Feasible Region
 *===========================================================================*/

/**
 * Project the solution onto the feasible region defined by A×u ≤ b.
 *
 * Uses iterative scaling: if any constraint is violated,
 * scale down the solution to satisfy it.
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
    /* Temporary storage for constraint evaluation */
    fixed_point_t constraint_values[QP_MAXIMUM_CONSTRAINTS];

    for (int projection_iteration = 0; projection_iteration < PROJECTION_ITERATIONS; projection_iteration++)
    {
        /* Compute A×u for all constraints */
        linear_algebra_matrix_vector_multiply(
            constraint_matrix,
            variable_vector,
            constraint_values,
            constraint_count,
            variable_count);

        /* Find maximum scaling factor needed to satisfy all constraints */
        fixed_point_t minimum_scale_factor = FIXED_POINT_ONE;

        for (uint16_t constraint_index = 0; constraint_index < constraint_count; constraint_index++)
        {
            /* If A×u > b, constraint is violated */
            if (constraint_values[constraint_index] > constraint_bounds[constraint_index])
            {
                /* Compute scale factor: scale = b / (A×u) */
                fixed_point_t denominator = constraint_values[constraint_index] + PROJECTION_EPSILON;
                fixed_point_t scale_factor = fixed_point_div(
                    constraint_bounds[constraint_index],
                    denominator);

                if (scale_factor < minimum_scale_factor)
                {
                    minimum_scale_factor = scale_factor;
                }
            }
        }

        /* Apply scaling if any constraint was violated */
        if (minimum_scale_factor < FIXED_POINT_ONE)
        {
            linear_algebra_vector_scale(
                variable_vector,
                minimum_scale_factor,
                variable_vector,
                variable_count);
        }
        else
        {
            /* All constraints satisfied, projection complete */
            break;
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

    /* Main optimization loop */
    for (int iteration = 0; iteration < config->maximum_iterations; iteration++)
    {
        solution->iteration_count = iteration;

        /*
         * Step 1: Compute gradient = H×u + f
         */
        linear_algebra_matrix_vector_multiply(
            problem->hessian_matrix,
            solution->optimal_variables,
            hessian_times_variables,
            variable_count,
            variable_count);

        for (uint16_t index = 0; index < variable_count; index++)
        {
            gradient[index] = fixed_point_add(
                hessian_times_variables[index],
                problem->linear_cost_vector[index]);
        }

        /*
         * Step 2: Gradient descent step: u_new = u - step_size × gradient
         */
        fixed_point_t negative_step_size = fixed_point_neg(config->gradient_step_size);

        linear_algebra_vector_add_scaled(
            solution->optimal_variables,
            gradient,
            negative_step_size,
            next_variables,
            variable_count);

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
            fixed_point_t difference = fixed_point_sub(
                next_variables[index],
                solution->optimal_variables[index]);

            fixed_point_t difference_squared = fixed_point_mul(difference, difference);
            step_norm_squared = fixed_point_add(step_norm_squared, difference_squared);
        }

        fixed_point_t step_norm = fixed_point_sqrt(step_norm_squared);

        /* Update solution with new variables */
        memcpy(solution->optimal_variables, next_variables,
               variable_count * sizeof(fixed_point_t));

        /*
         * Step 5: Compute constraint residual
         */
        solution->constraint_residual = linear_algebra_max_constraint_violation(
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
    solution->constraint_residual = linear_algebra_max_constraint_violation(
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
    /* Step size of 0.5 provides good balance of speed and stability */
    config->gradient_step_size = FIXED_POINT_HALF;  /* 0.5 in Q16.16 = 32768 */

    /* Convergence tolerance of 0.001 (about 65 in fixed-point) */
    config->convergence_tolerance = (fixed_point_t)65;  /* 0.001 in Q16.16 ≈ 65 */

    /* Maximum iterations */
    config->maximum_iterations = QP_MAXIMUM_ITERATIONS;

    /* Verbose output disabled by default */
    config->enable_verbose_output = 0;
}
