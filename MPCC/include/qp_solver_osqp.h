/**
 * @file qp_solver_osqp.h
 * @brief OSQP QP Solver Bridge for MPCC
 *
 * Converts the structured multistage QP (MPCCQPProblem_t) into
 * OSQP's sparse CSC format and solves it using OSQP.
 *
 * Decision variable ordering:
 *   z = [x_0(7), u_0(3), x_1(7), u_1(3), ..., x_{N-1}(7), u_{N-1}(3), x_N(7)]
 *   Total variables: N*10 + 7  (for N=20: 207)
 *
 * Constraint structure (l <= A*z <= u):
 *   1. Dynamics equality:  A_k x_k + B_k u_k - I x_{k+1} = -d_k  (N*7 rows)
 *   2. Track corridor:     sin(phi)*X_k - cos(phi)*Y_k in bounds   (N+1 rows)
 *   3. Box constraints:    identity rows for all variables          (n_vars rows)
 *
 * Cost (0.5 z^T P z + q^T z):
 *   P is block-diagonal with Q_k(7x7) and R_k(3x3) per stage, Q_N(7x7) terminal.
 *   q is stacked [q_k(7), r_k(3)] per stage, q_N(7) terminal.
 */

#ifndef QP_SOLVER_OSQP_H
#define QP_SOLVER_OSQP_H

#include "qp_solver_mpcc.h"   /* MPCCQPProblem_t, ADMMResult_t, etc. */

/**
 * @brief Solve the MPCC QP using OSQP
 *
 * @param problem   Fully populated QP problem (from build_qp_problem)
 * @param result    Output: optimal states/controls and solver diagnostics
 * @return MPCCStatus_t   SUCCESS, MAX_ITERATIONS, INFEASIBLE, or ERROR
 *
 * This function:
 *  1. Builds P (cost), A (constraints) in CSC format
 *  2. Builds q (linear cost), l/u (constraint bounds)
 *  3. Calls osqp_setup, osqp_solve, osqp_cleanup
 *  4. Extracts solution into ADMMResult_t
 */
MPCCStatus_t osqp_solver_solve(
    const MPCCQPProblem_t *problem,
    ADMMResult_t *result);

#endif /* QP_SOLVER_OSQP_H */
