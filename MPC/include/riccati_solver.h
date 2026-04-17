/**
 * @file riccati_solver.h
 * @brief Riccati-ADMM solver for non-condensed MPC.
 * @details Declares API contracts for solving the constrained linear-quadratic
 *          MPC subproblem with an ADMM outer loop and a Riccati recursion inner
 *          solve. Solves the constrained optimal control problem:
 *
 *   min  Σ_{k=0}^{N-1} [l_k(x_k, u_k)] + l_N(x_N)
 *   s.t. x_{k+1} = A_k x_k + B_k u_k
 *        x_lb_k <= x_k <= x_ub_k    (state box constraints)
 *        u_lb_k <= u_k <= u_ub_k    (control box constraints)
 *
 * Using ADMM to handle constraints, with Riccati recursion for the
 * unconstrained LQR sub-problem (O(N) per ADMM iteration).
 *
 * Supports cross-cost term x^T N u for rate-penalty formulations
 * where the state is augmented with previous control inputs.
 *
 * All arithmetic uses native float operations.
 * @dependencies mpc_types.h
 */

#ifndef RICCATI_SOLVER_H
#define RICCATI_SOLVER_H

#include "mpc_types.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/**
 * @brief Compute the analytical inverse of a 2x2 matrix.
 * @details Inputs: S is a 2x2 matrix in row-major layout.
 *          Purpose: provide a fast closed-form inverse for the Riccati control
 *          Hessian block.
 *          Outputs: writes inverse entries to Si when invertible.
 * @param S Input 2x2 matrix.
 * @param Si Output 2x2 inverse matrix.
 * @return 0 on success, -1 when the matrix is singular or near-singular.
 */
int riccati_invert_2x2(
    float S[2][2],
    float Si[2][2]);

/**
 * @brief Execute one Riccati backward-forward pass for ADMM primal update.
 * @details Inputs: per-stage model/cost data, terminal costs, initial state,
 *          dimensions, current ADMM penalties, and ADMM z/y iterates.
 *          Purpose: compute unconstrained primal trajectories under the current
 *          augmented-Lagrangian objective.
 *          Outputs: writes x_out/u_out trajectories for all stages.
 * @param step_data Per-step dynamics and cost data.
 * @param terminal_Q Terminal diagonal state cost.
 * @param terminal_q Terminal linear state cost.
 * @param x0 Initial state vector.
 * @param nx State dimension.
 * @param nu Control dimension.
 * @param N Prediction horizon length.
 * @param rho ADMM penalty for constrained state channels.
 * @param rho_u ADMM penalty for control channels.
 * @param z_x ADMM projected state variables.
 * @param y_x ADMM dual state variables.
 * @param z_u ADMM projected control variables.
 * @param y_u ADMM dual control variables.
 * @param x_out Output state trajectory.
 * @param u_out Output control trajectory.
 * @return None.
 */
void riccati_solver_pass(
    const RiccatiStepData_t * restrict step_data,
    const float * restrict terminal_Q,
    const float * restrict terminal_q,
    const float * restrict terminal_x_lb,
    const float * restrict terminal_x_ub,
    const float * restrict x0,
    int nx, int nu, int N,
    float rho,
    float rho_u,
    const float z_x[][RICCATI_MAX_NX],
    const float y_x[][RICCATI_MAX_NX],
    const float z_u[][RICCATI_MAX_NU],
    const float y_u[][RICCATI_MAX_NU],
    float x_out[][RICCATI_MAX_NX],
    float u_out[][RICCATI_MAX_NU]);

/**
 * @brief Zero all ADMM warm-start buffers and mark the state as uninitialized.
 * @details Inputs: state points to writable warm-start buffers.
 *          Purpose: reset ADMM history so the next solve uses a cold start.
 *          Outputs: clears z/y buffers, stored rho values, and sets
 *          initialized = 0.
 *          Call before the first solve, and whenever the problem changes enough
 *          to make the previous warm-start invalid (for example, a large
 *          operating-point change). If state is NULL, the function returns
 *          without side effects.
 * @param state Pointer to ADMM state to clear.
 * @return None.
 */
void riccati_admm_state_init(RiccatiAdmmState_t *state);

/**
 * @brief Solve constrained LQR over a finite horizon using Riccati-ADMM.
 * @details Inputs: per-step dynamics/cost data, terminal cost vectors,
 *          initial state, dimensions, horizon, and solver configuration.
 *          Purpose: compute a control/state trajectory that minimizes stage and
 *          terminal costs while satisfying box constraints through ADMM
 *          projections.
 *          Outputs: writes state and control trajectories, residuals,
 *          iteration count, and solver status to *solution, and updates
 *          *admm_state for warm-start reuse.
 *
 * Each ADMM iteration consists of:
 * 1. Riccati backward pass: compute gains K_k, k_k using
 *    augmented costs (Q + rho*I, R + rho*I) and ADMM offsets
 * 2. Riccati forward pass: roll out x_k, u_k from x0
 * 3. z-update: project (x + y, u + y) onto box constraints
 * 4. y-update: dual variable update
 * 5. Convergence check on primal/dual residuals
 *
 * Complexity: O(N × nx³) per iteration (dominated by 5×5 matrix ops)
 *
 * @param step_data   Per-step dynamics and cost data (array of N elements)
 * @param terminal_Q  Terminal state cost (diagonal weights, length nx)
 * @param terminal_q  Terminal state linear cost (length nx)
 * @param x0          Initial state (length nx)
 * @param nx          State dimension (≤ RICCATI_MAX_NX)
 * @param nu          Control dimension (≤ RICCATI_MAX_NU)
 * @param horizon     Prediction steps N (≤ PREDICTION_HORIZON)
 * @param config      ADMM configuration (if NULL, compile-time defaults are used).
 * @param admm_state  ADMM warm-start state (modified in-place)
 * @param solution    Output: optimal trajectories and status
 * @return Status code. Returns RICCATI_STATUS_ERROR if dimensions are invalid
 *         or any mandatory pointer input is NULL.
 */
RiccatiStatus_t riccati_admm_solve(
    const RiccatiStepData_t *step_data,
    const float *terminal_Q,
    const float *terminal_q,
    const float *terminal_x_lb,
    const float *terminal_x_ub,
    const float *x0,
    int nx, int nu, int horizon,
    const RiccatiAdmmConfig_t *config,
    RiccatiAdmmState_t *admm_state,
    RiccatiSolution_t *solution);

/** Debug flag: set to 1 to print ADMM iteration details */
extern int riccati_admm_debug;

#endif /* RICCATI_SOLVER_H */
