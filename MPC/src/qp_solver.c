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
#include <stdio.h>

/**
 * Integer square root of a non-negative int64 value.
 * Uses Newton's method. Returns floor(sqrt(n)).
 */
static uint32_t isqrt64(int64_t n)
{
    if (n <= 0) return 0;
    uint64_t x = (uint64_t)n;
    /* Initial estimate: 2^(bits/2) */
    int bits = 0;
    uint64_t tmp = x;
    while (tmp > 0) { tmp >>= 1; bits++; }
    uint64_t r = 1ULL << (bits / 2);
    /* Newton iterations (converges in ~6 for 64-bit) */
    for (int i = 0; i < 8; i++)
    {
        uint64_t next = (r + x / r) / 2;
        if (next >= r) break; /* converged */
        r = next;
    }
    /* Fine-tune */
    while (r * r > x) r--;
    while ((r + 1) * (r + 1) <= x) r++;
    return (uint32_t)r;
}

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
        const fixed_point_t *row = &constraint_matrix[constraint_index * variable_count];
        int16_t first_nonzero_index = -1;
        uint8_t nonzero_count = 0;
        int64_t norm_squared_64 = 0;

        for (uint16_t variable_index = 0; variable_index < variable_count; variable_index++)
        {
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
    if (problem->use_warm_start)
    {
        memcpy(solution->optimal_variables, problem->initial_point,
               variable_count * sizeof(fixed_point_t));
    }
    else
    {
        memset(solution->optimal_variables, 0, variable_count * sizeof(fixed_point_t));
    }
    memset(solution->dual_variables, 0, constraint_count * sizeof(fixed_point_t));

    solution->iteration_count = 0;
    solution->status = QP_STATUS_ERROR;

    build_constraint_metadata(
        problem->constraint_matrix,
        variable_count,
        constraint_count,
        constraint_metadata);

    /*
     * Compute per-variable step sizes using diagonal preconditioning.
     *
     * Problem: the raw Gershgorin row sums treat all variables equally,
     * but steering has H_diag ~240K while accel has ~30M — a 125x ratio.
     * This causes PGD to take tiny steps for steering (row_sum ~120M
     * dominated by off-diagonal coupling) while accel converges quickly.
     *
     * Solution: Diagonal preconditioning.  Scale the QP by D = diag(sqrt(H_ii)):
     *   H_tilde = D^{-1} H D^{-1}   (has unit diagonal)
     *   Gershgorin on H_tilde: L_tilde_i = sum_j |H[i][j]| / (sqrt(H_ii)*sqrt(H_jj))
     *
     * The effective step size in original u-space becomes:
     *   step_denom[i] = sqrt(H_ii) * sum_j (|H[i][j]| / sqrt(H_jj))
     *
     * This gives much tighter bounds for poorly-conditioned variables
     * while maintaining convergence guarantees.
     */
    int64_t step_denom_64[QP_MAXIMUM_VARIABLES];

    /* Phase 1: compute sqrt of each diagonal element */
    uint32_t sqrt_diag[QP_MAXIMUM_VARIABLES];
    for (uint16_t j = 0; j < variable_count; j++)
    {
        int64_t hd = (int64_t)problem->hessian_matrix[j * variable_count + j];
        if (hd < 0) hd = -hd;
        if (hd == 0) hd = 1;
        sqrt_diag[j] = isqrt64(hd);
        if (sqrt_diag[j] == 0) sqrt_diag[j] = 1;
    }

    /* Phase 2: compute preconditioned Gershgorin step denominators */
    for (uint16_t i = 0; i < variable_count; i++)
    {
        /* Scaled row sum: sum_j |H[i][j]| / sqrt(H[j][j]) */
        int64_t scaled_row_sum = 0;
        for (uint16_t j = 0; j < variable_count; j++)
        {
            int64_t hij = (int64_t)problem->hessian_matrix[i * variable_count + j];
            if (hij < 0) hij = -hij;
            scaled_row_sum += hij / (int64_t)sqrt_diag[j];
        }

        /* step_denom = sqrt(H[i][i]) * scaled_row_sum */
        step_denom_64[i] = (int64_t)sqrt_diag[i] * scaled_row_sum;

        /* Floor: prevent division by zero or huge steps */
        if (step_denom_64[i] < (int64_t)FP_ONE)
            step_denom_64[i] = (int64_t)FP_ONE;

        /* Debug: print first call's Hessian diagnostics */
        if (config->enable_verbose_output && i < 4)
        {
            printf("[QP] H_diag[%d]=%d sqrt_diag=%u step_denom=%lld scaled_rs=%lld\n",
                   i, (int)problem->hessian_matrix[i * variable_count + i],
                   sqrt_diag[i],
                   (long long)step_denom_64[i], (long long)scaled_row_sum);
        }
    }

    /* Convergence threshold using int64 to avoid Q16.16 precision loss.
     * In Q16.16: tol=1310 (0.02). fp_mul(1310,1310) = 26 (rounds to ~0).
     * Using int64: (1310 * 1310) >> 16 with full precision = 26, still small.
     * Instead compute in raw int64: tol_raw² = 1310² = 1,716,100.
     * Then the step-norm is also accumulated in int64 raw units.
     * This gives proper convergence detection. Trust-region bounds in
     * the constraint matrix now prevent full-lock divergence instead of
     * relying on broken convergence as an implicit trust region. */
    /* Main optimization loop: projected gradient descent with int64 step
     * computation and max-element convergence check. */
    for (int iteration = 0; iteration < config->maximum_iterations; iteration++)
    {
        solution->iteration_count = iteration;

        /*
         * Step 1: Compute gradient = H×u + f
         */
        fp_symmetric_mat_vec_mul(
            problem->hessian_matrix,
            solution->optimal_variables,
            hessian_times_variables,
            variable_count);

        for (uint16_t index = 0; index < variable_count; index++)
        {
            gradient[index] = fp_add_sat(
                hessian_times_variables[index],
                problem->linear_cost_vector[index]);
        }

        /*
         * Step 2: Gradient descent step with per-variable Gershgorin steps.
         *
         * u_new[i] = u[i] - gradient[i] / gershgorin_radius[i]
         *
         * Uses int64 division to avoid Q16.16 underflow when the
         * Gershgorin radius exceeds INT32_MAX (common at high speed
         * where the auto-scaled Hessian has row sums of 1-10B).
         */
        for (uint16_t index = 0; index < variable_count; index++)
        {
            int64_t step64 = (int64_t)gradient[index] * (int64_t)FP_ONE
                             / step_denom_64[index];
            if (step64 > INT32_MAX) step64 = INT32_MAX;
            else if (step64 < INT32_MIN) step64 = INT32_MIN;
            next_variables[index] = fp_sub(
                solution->optimal_variables[index],
                (fixed_point_t)step64);
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
         * Step 4: Check convergence using max-abs per-variable step.
         *
         * max_i |u_new[i] - u[i]| < tolerance
         *
         * Sum-of-squares converged prematurely at high speed because
         * Gershgorin steps (~100-500 raw) fell below the squared tolerance
         * (1310² = 1.72M) with only 10 variables. Max-element is scale-
         * independent and prevents false convergence.
         */
        fixed_point_t max_step = 0;

        for (uint16_t index = 0; index < variable_count; index++)
        {
            fixed_point_t diff = next_variables[index] -
                                 solution->optimal_variables[index];
            if (diff < 0) diff = -diff;
            if (diff > max_step) max_step = diff;
        }

        /* Update solution with new variables */
        memcpy(solution->optimal_variables, next_variables,
               variable_count * sizeof(fixed_point_t));

        /*
         * Step 5+6: Check termination criteria.
         * Constraint residual is DEFERRED until the step check passes,
         * avoiding the O(constraints) scan on every iteration.
         */
        if (max_step < config->convergence_tolerance)
        {
            solution->constraint_residual = compute_max_violation_sparse(
                problem->constraint_matrix,
                constraint_metadata,
                solution->optimal_variables,
                problem->constraint_bounds,
                constraint_count,
                variable_count);

            if (solution->constraint_residual < config->convergence_tolerance)
            {
                solution->status = QP_STATUS_OPTIMAL;
                return QP_STATUS_OPTIMAL;
            }
        }

        /* Debug: print progress every 500 iterations */
        if (config->enable_verbose_output && (iteration % 500 == 0 || iteration == config->maximum_iterations - 1))
        {
             printf("[QP] iter=%d max_step=%d tol=%d\n",
                 iteration, (int)max_step, (int)config->convergence_tolerance);
        }
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

    /* Convergence tolerance: 0.001 in Q16.16 (raw=65).
     * The MPC layer overrides this with its own tolerance (typically 0.005).
     * Convergence check uses max-absolute-element (not sum-of-squares)
     * to avoid Q16.16 quantization issues at high speed. */
    config->convergence_tolerance = (fixed_point_t)65;  /* 0.001 in Q16.16 ≈ 65 */

    /* Maximum iterations */
    config->maximum_iterations = QP_MAXIMUM_ITERATIONS;

    /* Verbose output disabled by default */
    config->enable_verbose_output = 0;
}
