/**
 * @file qp_solver.h
 * @brief Quadratic Programming Solver for MPC
 *
 * Solves convex QP using Projected Gradient Descent:
 *   minimize    0.5 × u^T × H × u + f^T × u
 *   subject to  A × u ≤ b
 */

#ifndef QP_SOLVER_H
#define QP_SOLVER_H

#include "fp_math.h"
#include "mpc_types.h"
#include <stdint.h>

/*===========================================================================
 * Problem Dimension Limits
 *===========================================================================*/

#define QP_MAX_VARS         60      /**< Max optimization variables */
#define QP_MAX_CONSTRAINTS  240     /**< Max inequality constraints (box + rate + path bounds) */
#define QP_MAX_ITER         1000    /**< Max solver iterations */

/*===========================================================================
 * QP Problem Definition
 *===========================================================================*/

typedef struct
{
    fixed_point_t H[QP_MAX_VARS * QP_MAX_VARS];     /**< Hessian (n×n) */
    fixed_point_t f[QP_MAX_VARS];                   /**< Linear cost (n) */
    fixed_point_t A[QP_MAX_CONSTRAINTS * QP_MAX_VARS]; /**< Constraints (m×n) */
    fixed_point_t b[QP_MAX_CONSTRAINTS];            /**< Bounds (m) */
    uint16_t n_vars;                                /**< Number of variables */
    uint16_t n_constraints;                         /**< Number of constraints */
} QpProblem_t;

/*===========================================================================
 * QP Solver Configuration
 *===========================================================================*/

typedef struct
{
    fixed_point_t step_size;     /**< Gradient descent step size */
    fixed_point_t tolerance;     /**< Convergence tolerance */
    uint16_t max_iter;           /**< Maximum iterations */
    uint8_t verbose;             /**< Enable debug output */
} QpConfig_t;

/*===========================================================================
 * QP Solution
 *===========================================================================*/

typedef struct
{
    fixed_point_t x[QP_MAX_VARS];    /**< Optimal variables */
    fixed_point_t dual[QP_MAX_CONSTRAINTS]; /**< Dual variables */
    SolverStatus_t status;           /**< Solver status */
    uint16_t iterations;             /**< Iterations used */
    fixed_point_t residual;          /**< Constraint residual */
} QpSolution_t;

/*===========================================================================
 * Solver Functions
 *===========================================================================*/

/** Solve QP problem */
SolverStatus_t qp_solve(
    const QpProblem_t *problem,
    const QpConfig_t *config,
    QpSolution_t *solution);

/** Initialize problem struct (zeros) */
void qp_init_problem(QpProblem_t *problem);

/** Initialize config with defaults */
void qp_init_config(QpConfig_t *config);

#endif /* QP_SOLVER_H */
