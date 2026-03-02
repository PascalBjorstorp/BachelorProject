/**
 * @file qp_solver.h
 * @brief Quadratic Programming Solver for MPC
 *
 * Solves convex QP problems using Projected Gradient Descent:
 *
 *   minimize    0.5 × u^T × H × u + f^T × u
 *   subject to  A × u ≤ b
 *
 * Where:
 * - u is the optimization variable (control sequence)
 * - H is the positive semi-definite Hessian matrix
 * - f is the linear cost term
 * - A and b define inequality constraints
 *
 * The solver is designed for real-time MPC:
 * - Fixed memory allocation (no malloc)
 * - Deterministic iteration count
 * - FPGA-compatible (fixed-point only)
 */

#ifndef QP_SOLVER_H
#define QP_SOLVER_H

#include "fp_math.h"
#include <stdint.h>

/*===========================================================================
 * Problem Dimension Limits
 *===========================================================================*/

/**
 * Maximum number of optimization variables.
 * CPU:  80 (generic, supports various horizon lengths)
 * FPGA: 46 (tight bound: N=20 × 2 controls + 6 slacks)
 */
#ifdef MPC_HLS_TARGET
#define QP_MAXIMUM_VARIABLES        42
#else
#define QP_MAXIMUM_VARIABLES        80
#endif

/**
 * Maximum number of inequality constraints.
 * CPU:  200 (generic)
 * FPGA:  96 (tight bound: 80 actuator + 6 wall + 6 slack + 4 margin)
 */
#ifdef MPC_HLS_TARGET
#define QP_MAXIMUM_CONSTRAINTS      88
#else
#define QP_MAXIMUM_CONSTRAINTS      200
#endif

/**
 * Maximum solver iterations before termination.
 * Ensures bounded execution time for real-time systems.
 */
#define QP_MAXIMUM_ITERATIONS       1000

/*===========================================================================
 * Solver Status Codes
 *===========================================================================*/

typedef enum
{
    /** Optimal solution found within tolerance */
    QP_STATUS_OPTIMAL = 0,

    /** Maximum iterations reached (may still be feasible) */
    QP_STATUS_MAXIMUM_ITERATIONS_REACHED = 1,

    /** Problem appears infeasible (constraints cannot be satisfied) */
    QP_STATUS_INFEASIBLE = 2,

    /** Solver encountered an error */
    QP_STATUS_ERROR = 3

} QuadraticProgramStatus_t;

/*===========================================================================
 * Problem Definition Structure
 *===========================================================================*/

/**
 * Complete QP problem definition.
 * All matrices are stored in row-major order.
 */
typedef struct
{
    /**
     * Hessian matrix H (variable_count × variable_count).
     * Must be positive semi-definite for convex QP.
     * Stored in row-major: H[i,j] at index [i * variable_count + j]
     */
    fixed_point_t hessian_matrix[QP_MAXIMUM_VARIABLES * QP_MAXIMUM_VARIABLES];

    /**
     * Linear cost vector f (length variable_count).
     */
    fixed_point_t linear_cost_vector[QP_MAXIMUM_VARIABLES];

    /**
     * Constraint matrix A (constraint_count × variable_count).
     * Defines inequality constraints: A × u ≤ b
     */
    fixed_point_t constraint_matrix[QP_MAXIMUM_CONSTRAINTS * QP_MAXIMUM_VARIABLES];

    /**
     * Constraint bounds vector b (length constraint_count).
     */
    fixed_point_t constraint_bounds[QP_MAXIMUM_CONSTRAINTS];

    /** Actual number of optimization variables in this problem */
    uint16_t variable_count;

    /** Actual number of inequality constraints in this problem */
    uint16_t constraint_count;

    /**
     * Optional warm-start initial point.
     * If use_warm_start is non-zero, the solver initializes from this
     * instead of zero. Set use_warm_start=0 for cold start.
     */
    fixed_point_t initial_point[QP_MAXIMUM_VARIABLES];
    uint8_t use_warm_start;

} QuadraticProgramProblem_t;

/*===========================================================================
 * Solver Configuration
 *===========================================================================*/

/**
 * Solver tuning parameters.
 */
typedef struct
{
    /**
     * Gradient descent step size (learning rate).
     * Typical value: 0.01 to 0.5
     * Smaller = more stable but slower convergence
     */
    fixed_point_t gradient_step_size;

    /**
     * Convergence tolerance.
     * Solver stops when step change < tolerance AND constraints satisfied.
     */
    fixed_point_t convergence_tolerance;

    /** Maximum number of iterations */
    int maximum_iterations;

    /** Enable verbose output (for debugging only) */
    uint8_t enable_verbose_output;

} QuadraticProgramConfig_t;

/*===========================================================================
 * Solution Structure
 *===========================================================================*/

/**
 * Solver output containing optimal solution and diagnostics.
 */
typedef struct
{
    /** Optimal control sequence (length variable_count) */
    fixed_point_t optimal_variables[QP_MAXIMUM_VARIABLES];

    /** Lagrange multipliers for constraints (length constraint_count) */
    fixed_point_t dual_variables[QP_MAXIMUM_CONSTRAINTS];

    /** Number of iterations performed */
    int iteration_count;

    /** Maximum constraint violation at solution */
    fixed_point_t constraint_residual;

    /** Solver termination status */
    QuadraticProgramStatus_t status;

} QuadraticProgramSolution_t;

/*===========================================================================
 * Solver API
 *===========================================================================*/

/**
 * Solve the quadratic programming problem.
 *
 * Algorithm: Projected Gradient Descent
 * 1. Initialize u = 0
 * 2. Compute gradient: g = H×u + f
 * 3. Gradient step: u_new = u - α×g
 * 4. Project onto feasible region (enforce A×u ≤ b)
 * 5. Check convergence: ||u_new - u|| < tolerance
 * 6. Repeat until converged or max iterations
 *
 * @param problem   QP problem definition (H, f, A, b)
 * @param config    Solver configuration parameters
 * @param solution  Output: optimal solution and status
 * @return Solver status code
 */
QuadraticProgramStatus_t qp_solver_solve(
    const QuadraticProgramProblem_t *problem,
    const QuadraticProgramConfig_t *config,
    QuadraticProgramSolution_t *solution);

/**
 * Initialize a QP problem structure to zeros.
 * Call before populating problem data.
 *
 * @param problem  Problem structure to initialize
 */
void qp_solver_initialize_problem(QuadraticProgramProblem_t *problem);

/**
 * Initialize solver configuration with default values.
 *
 * Default values:
 * - gradient_step_size: 0.03 (fallback; adaptive Gershgorin steps used)
 * - convergence_tolerance: 0.001
 * - maximum_iterations: 1000
 * - enable_verbose_output: 0 (disabled)
 *
 * @param config  Configuration structure to initialize
 */
void qp_solver_initialize_config(QuadraticProgramConfig_t *config);

#endif /* QP_SOLVER_H */
