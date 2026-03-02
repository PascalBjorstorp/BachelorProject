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
#ifndef MPC_HLS_TARGET
#include <stdio.h>
#endif

typedef struct
{
    uint8_t nonzero_count;
    int16_t single_nonzero_index;
    fixed_point_t norm_squared;
    fixed_point_t inv_norm_squared;  /* precomputed 1/norm_squared for division-free projection */
    uint8_t nonzero_cols[QP_MAXIMUM_VARIABLES]; /* column indices of nonzero entries */
} ConstraintMetadata_t;

static void build_constraint_metadata(
    const fixed_point_t *constraint_matrix,
    uint16_t variable_count,
    uint16_t constraint_count,
    ConstraintMetadata_t *metadata)
{
    for (uint16_t constraint_index = 0; constraint_index < constraint_count; constraint_index++)
    {
#ifdef MPC_HLS_TARGET
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=10 max=52 avg=40
#endif
        const fixed_point_t *row = &constraint_matrix[constraint_index * variable_count];
        int16_t first_nonzero_index = -1;
        uint8_t nonzero_count = 0;
        int64_t norm_squared_64 = 0;

        for (uint16_t variable_index = 0; variable_index < variable_count; variable_index++)
        {
#ifdef MPC_HLS_TARGET
#pragma HLS LOOP_TRIPCOUNT min=10 max=26 avg=20
#endif
            fixed_point_t coeff = row[variable_index];
            if (coeff != 0)
            {
                if (nonzero_count == 0)
                {
                    first_nonzero_index = (int16_t)variable_index;
                }
                metadata[constraint_index].nonzero_cols[nonzero_count] = (uint8_t)variable_index;
                nonzero_count++;
                norm_squared_64 += ((int64_t)coeff * coeff) >> FP_FRAC_BITS;
            }
        }

        metadata[constraint_index].nonzero_count = nonzero_count;
        metadata[constraint_index].single_nonzero_index =
            (nonzero_count == 1) ? first_nonzero_index : -1;
        fixed_point_t ns = (norm_squared_64 > INT32_MAX) ? INT32_MAX : (fixed_point_t)norm_squared_64;
        metadata[constraint_index].norm_squared = ns;
        metadata[constraint_index].inv_norm_squared = (ns > 0) ? fp_recip(ns) : 0;
    }
}

/*===========================================================================
 * Internal Constants
 *===========================================================================*/

/*===========================================================================
 * Internal Helper: Sparse Max Constraint Violation
 *===========================================================================*/

/**
 * Compute maximum constraint violation using precomputed sparsity metadata.
 * Box constraints (single nonzero per row) use one multiply instead of n.
 * General constraints skip zero entries in the row.
 * Replaces the O(constraints * variables) fp_max_violation with a
 * sparsity-aware O(constraints * avg_nonzeros) version.
 */
static fixed_point_t compute_max_violation_sparse(
    const fixed_point_t *constraint_matrix,
    const ConstraintMetadata_t *metadata,
    const fixed_point_t *variables,
    const fixed_point_t *bounds,
    uint16_t constraint_count,
    uint16_t variable_count)
{
    fixed_point_t max_viol = 0;

    for (uint16_t ci = 0; ci < constraint_count; ci++)
    {
#ifdef MPC_HLS_TARGET
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=10 max=52 avg=40
#endif
        fixed_point_t val;
        const fixed_point_t *row = &constraint_matrix[ci * variable_count];

        if (metadata[ci].nonzero_count == 0)
        {
            continue;
        }
        else if (metadata[ci].single_nonzero_index >= 0)
        {
            /* Box constraint: single multiply */
            int idx = metadata[ci].single_nonzero_index;
            val = fp_mul(row[idx], variables[idx]);
        }
        else
        {
            /* General constraint: indexed sparse dot product (no branch per var) */
            int64_t dot64 = 0;
            for (uint8_t nz = 0; nz < metadata[ci].nonzero_count; nz++)
            {
#ifdef MPC_HLS_TARGET
#pragma HLS LOOP_TRIPCOUNT min=1 max=7 avg=2
#endif
                uint8_t j = metadata[ci].nonzero_cols[nz];
                dot64 += ((int64_t)row[j] * variables[j]) >> FP_FRAC_BITS;
            }
            val = (dot64 > INT32_MAX)  ? INT32_MAX :
                  (dot64 < INT32_MIN)  ? INT32_MIN :
                  (fixed_point_t)dot64;
        }

        fixed_point_t viol = fp_sub(val, bounds[ci]);
        if (viol > max_viol)
            max_viol = viol;
    }

    return max_viol;
}

/*===========================================================================
 * Internal Helper: Project onto Feasible Region
 *===========================================================================*/

/**
 * Project the solution onto the feasible region defined by A×u ≤ b.
 *
 * Handles two types of constraints:
 * 1. Box constraints: single non-zero entry per row (fast clamping)
 * 2. General linear constraints: multiple non-zero entries (projection
 *    along constraint normal for wall boundary enforcement)
 *
 * For box constraints (A[i,j] = ±1, one per row):
 *   If A[i,j] = +1: u[j] = min(u[j], b[i])
 *   If A[i,j] = -1: u[j] = max(u[j], -b[i])
 *
 * For general constraints (wall boundaries, multiple non-zeros):
 *   If A[i,:] * u > b[i]:
 *     u -= (violation / ||A[i,:]||²) * A[i,:]
 *   This projects u onto the constraint halfplane.
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
    const ConstraintMetadata_t *constraint_metadata,
    const fixed_point_t *constraint_bounds,
    uint16_t variable_count,
    uint16_t constraint_count)
{
    /*
     * Two-phase projection to ensure actuator limits always hold:
     *   Phase 1: General linear constraints (wall boundaries)
     *   Phase 2: Box constraints (actuator limits) — always enforced last
     *
     * This ordering prevents wall constraint projections from pushing
     * actuator variables outside their box bounds.
     */

    /* Phase 1: General linear constraints (halfplane projection) */
    for (uint16_t constraint_index = 0; constraint_index < constraint_count; constraint_index++)
    {
#ifdef MPC_HLS_TARGET
#pragma HLS LOOP_TRIPCOUNT min=10 max=52 avg=40
#endif
        const fixed_point_t *row = &constraint_matrix[constraint_index * variable_count];
        const ConstraintMetadata_t *meta = &constraint_metadata[constraint_index];
        fixed_point_t bound = constraint_bounds[constraint_index];

        if (meta->nonzero_count <= 1)
        {
            continue;
        }

        int64_t dot64 = 0;

        /* Indexed access: iterate only over nonzero columns (no branch) */
        for (uint8_t nz = 0; nz < meta->nonzero_count; nz++)
        {
#ifdef MPC_HLS_TARGET
#pragma HLS LOOP_TRIPCOUNT min=2 max=7 avg=4
#endif
            uint8_t j = meta->nonzero_cols[nz];
            dot64 += ((int64_t)row[j] * variable_vector[j]) >> FP_FRAC_BITS;
        }

        /*
         * General linear constraint: project onto halfplane.
         * violation = A[i] · u - b[i]
         * If violation > 0: u -= (violation / ||A[i]||²) * A[i]
         */
        fixed_point_t dot = (dot64 > INT32_MAX) ? INT32_MAX :
                            (dot64 < INT32_MIN) ? INT32_MIN :
                            (fixed_point_t)dot64;
        fixed_point_t violation = fp_sub(dot, bound);

        if (violation > 0)
        {
            if (meta->inv_norm_squared != 0)
            {
                /* Multiply by precomputed reciprocal instead of division */
                fixed_point_t scale = fp_mul(violation, meta->inv_norm_squared);

                /* Indexed projection update: no branch per variable */
                for (uint8_t nz = 0; nz < meta->nonzero_count; nz++)
                {
#ifdef MPC_HLS_TARGET
#pragma HLS LOOP_TRIPCOUNT min=2 max=7 avg=4
#endif
                    uint8_t j = meta->nonzero_cols[nz];
                    variable_vector[j] = fp_sub(
                        variable_vector[j],
                        fp_mul(scale, row[j]));
                }
            }
        }
    }

    /* Phase 2: Box constraints (clamping — always last for guaranteed enforcement) */
    for (uint16_t constraint_index = 0; constraint_index < constraint_count; constraint_index++)
    {
#ifdef MPC_HLS_TARGET
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=10 max=52 avg=40
#endif
        const fixed_point_t *row = &constraint_matrix[constraint_index * variable_count];
        const ConstraintMetadata_t *meta = &constraint_metadata[constraint_index];
        fixed_point_t bound = constraint_bounds[constraint_index];

        int single_nonzero_idx = meta->single_nonzero_index;
        if (single_nonzero_idx < 0) continue;

        /* Box constraint: fast per-variable clamping */
        if (row[single_nonzero_idx] > 0)
        {
            if (variable_vector[single_nonzero_idx] > bound)
            {
                variable_vector[single_nonzero_idx] = bound;
            }
        }
        else
        {
            fixed_point_t lower = fp_neg(bound);
            if (variable_vector[single_nonzero_idx] < lower)
            {
                variable_vector[single_nonzero_idx] = lower;
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
    ConstraintMetadata_t constraint_metadata[QP_MAXIMUM_CONSTRAINTS];

    /* Initialize solution: use warm-start if provided, otherwise zero */
    {
        uint16_t i;
        for (i = 0; i < QP_MAXIMUM_VARIABLES; i++)
        {
#ifdef MPC_HLS_TARGET
#pragma HLS PIPELINE II=1
#endif
            if (i < variable_count)
            {
                solution->optimal_variables[i] =
                    problem->use_warm_start ? problem->initial_point[i] : 0;
            }
        }
        for (i = 0; i < QP_MAXIMUM_CONSTRAINTS; i++)
        {
#ifdef MPC_HLS_TARGET
#pragma HLS PIPELINE II=1
#endif
            if (i < constraint_count)
            {
                solution->dual_variables[i] = 0;
            }
        }
    }

    solution->iteration_count = 0;
    solution->status = QP_STATUS_ERROR;

    build_constraint_metadata(
        problem->constraint_matrix,
        variable_count,
        constraint_count,
        constraint_metadata);

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
#ifdef MPC_HLS_TARGET
#pragma HLS LOOP_TRIPCOUNT min=20 max=26 avg=26
#pragma HLS PIPELINE II=1
#endif
        /* Compute Gershgorin row sum using int64_t to avoid overflow */
        int64_t row_sum_64 = 0;
        for (uint16_t j = 0; j < variable_count; j++)
        {
#ifdef MPC_HLS_TARGET
#pragma HLS LOOP_TRIPCOUNT min=10 max=26 avg=20
#endif
            int64_t hij = (int64_t)problem->hessian_matrix[i * variable_count + j];
            if (hij < 0) hij = -hij;
            row_sum_64 += hij;
        }

        /* Clamp to Q16.16 range before division */
        fixed_point_t row_sum;
        if (row_sum_64 > INT32_MAX)
            row_sum = INT32_MAX;
        else
            row_sum = (fixed_point_t)row_sum_64;

        /* step[i] = 1 / row_sum using Newton-Raphson reciprocal
         * (multiplication-only, no hardware division — FPGA-friendly) */
        if (row_sum > FP_ONE)
        {
            inv_row_sum[i] = fp_recip(row_sum);
        }
        else
        {
            inv_row_sum[i] = config->gradient_step_size;
        }

        /* Debug: print first call's Hessian diagnostics */
#ifndef MPC_HLS_TARGET
        if (config->enable_verbose_output && i < 4)
        {
            printf("[QP] H_diag[%d]=%d row_sum_64=%lld step=%d\n",
                   i, (int)problem->hessian_matrix[i * variable_count + i],
                   (long long)row_sum_64, (int)inv_row_sum[i]);
        }
#endif
    }

    fixed_point_t tolerance_squared = fp_mul(
        config->convergence_tolerance,
        config->convergence_tolerance);

    /* Main optimization loop */
    for (int iteration = 0; iteration < config->maximum_iterations; iteration++)
    {
#ifdef MPC_HLS_TARGET
#pragma HLS LOOP_TRIPCOUNT min=1 max=50 avg=10
#endif
        solution->iteration_count = iteration;

        /*
         * Step 1: Compute gradient = H×u + f
         * Uses symmetric mat-vec: Hessian is symmetric, so only the
         * upper triangle is read. 2×2 block processing matches the
         * control-pair structure. Halves memory reads.
         */
        fp_symmetric_mat_vec_mul(
            problem->hessian_matrix,
            solution->optimal_variables,
            hessian_times_variables,
            variable_count);

        for (uint16_t index = 0; index < QP_MAXIMUM_VARIABLES; index++)
        {
#ifdef MPC_HLS_TARGET
#pragma HLS PIPELINE II=1
#endif
            if (index < variable_count)
            {
                gradient[index] = fp_add_sat(
                    hessian_times_variables[index],
                    problem->linear_cost_vector[index]);
            }
        }

        /*
         * Step 2: Gradient descent step with per-variable Gershgorin steps.
         *
         * u_new[i] = u[i] - step[i] × gradient[i]
         *
         * Each variable uses its own step size based on the Gershgorin
         * row sum, ensuring convergence regardless of conditioning.
         */
        for (uint16_t index = 0; index < QP_MAXIMUM_VARIABLES; index++)
        {
#ifdef MPC_HLS_TARGET
#pragma HLS PIPELINE II=1
#endif
            if (index < variable_count)
            {
                next_variables[index] = fp_sub(
                    solution->optimal_variables[index],
                    fp_mul(inv_row_sum[index], gradient[index]));
            }
        }

        /*
         * Step 3: Project onto feasible region (enforce A×u ≤ b)
         */
        project_onto_feasible_region(
            next_variables,
            problem->constraint_matrix,
            constraint_metadata,
            problem->constraint_bounds,
            variable_count,
            constraint_count);

        /*
         * Step 4: Check convergence ||u_new - u||
         */
        fixed_point_t step_norm_squared = 0;

        for (uint16_t index = 0; index < QP_MAXIMUM_VARIABLES; index++)
        {
#ifdef MPC_HLS_TARGET
#pragma HLS PIPELINE II=1
#endif
            if (index < variable_count)
            {
                fixed_point_t difference = fp_sub(
                    next_variables[index],
                    solution->optimal_variables[index]);

                fixed_point_t difference_squared = fp_mul(difference, difference);
                step_norm_squared = fp_add(step_norm_squared, difference_squared);
            }
        }

        /* Update solution with new variables */
        {
            uint16_t copy_i;
            for (copy_i = 0; copy_i < QP_MAXIMUM_VARIABLES; copy_i++)
            {
#ifdef MPC_HLS_TARGET
#pragma HLS PIPELINE II=1
#endif
                if (copy_i < variable_count)
                {
                    solution->optimal_variables[copy_i] = next_variables[copy_i];
                }
            }
        }

        /*
         * Step 5+6: Check termination criteria.
         * Constraint residual is DEFERRED until the norm check passes,
         * avoiding the O(constraints) scan on every iteration.
         * This is the single largest per-iteration savings since
         * max_violation_sparse scans all constraint rows.
         */
        if (step_norm_squared <= tolerance_squared)
        {
            solution->constraint_residual = compute_max_violation_sparse(
                problem->constraint_matrix,
                constraint_metadata,
                solution->optimal_variables,
                problem->constraint_bounds,
                constraint_count,
                variable_count);

            if (solution->constraint_residual <= config->convergence_tolerance)
            {
                solution->status = QP_STATUS_OPTIMAL;
                return QP_STATUS_OPTIMAL;
            }
        }

        /* Debug: print progress every 500 iterations */
#ifndef MPC_HLS_TARGET
        if (config->enable_verbose_output && (iteration % 500 == 0 || iteration == config->maximum_iterations - 1))
        {
             printf("[QP] iter=%d step_norm_sq=%d tol_sq=%d residual=%d\n",
                 iteration, (int)step_norm_squared, (int)tolerance_squared,
                   (int)solution->constraint_residual);
        }
#endif
    }

    /*
     * Maximum iterations reached - check if solution is at least feasible
     */
    solution->constraint_residual = compute_max_violation_sparse(
        problem->constraint_matrix,
        constraint_metadata,
        solution->optimal_variables,
        problem->constraint_bounds,
        constraint_count,
        variable_count);

    if (solution->constraint_residual <= config->convergence_tolerance)
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
    problem->use_warm_start = 0;
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
