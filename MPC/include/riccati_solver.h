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

typedef struct
{
    float rho;
    float rho_u;
    float primal_residual;
    float dual_residual;
    float last_invert_det;
    float last_fallback_s00;
    float last_fallback_s11;
    int invert_fallback_count;
} RiccatiDebugInfo_t;

typedef struct
{
    float r_aug_linear_steer;
    float r_aug_linear_accel;
    float bp_steer;
    float bp_accel;
    float kk_steer;
    float kk_accel;
    float s00;
    float s11;
    float p_shift_vx;
    float p_shift_vy;
    float p_shift_omega;
    float p_shift_accel_prev;
    float p_shift_ey;
    float p_shift_epsi;
    float p_vx;
    float p_vy;
    float p_omega;
    float p_accel_prev;
    float p_ey;
    float p_epsi;
    float pd_vx;
    float pd_vy;
    float pd_omega;
    float pd_accel_prev;
    float pd_ey;
    float pd_epsi;
    float p_atp_vx;
    float p_atp_vy;
    float p_atp_omega;
    float p_atp_accel_prev;
    float p_gtk_vx;
    float p_gtk_vy;
    float p_gtk_omega;
    float p_gtk_accel_prev;
    float bp_accel_vx;
    float bp_accel_vy;
    float bp_accel_omega;
    float bp_accel_prev;
    float si10;
    float si11;
    float rhs_accel;
    float k0_r_lin_accel;
    float k0_bp_accel;
    float k0_kk_accel;
    float k0_s11;
    float k0_si11;
    float k0_rhs_accel;
    float k0_p_shift_vx;
    float k0_p_shift_vy;
    float k0_p_shift_omega;
    float k0_p_shift_accel_prev;
    float k0_p_vx;
    float k0_p_vy;
    float k0_p_omega;
    float k0_p_accel_prev;
    float k0_pd_vx;
    float k0_pd_vy;
    float k0_pd_omega;
    float k0_pd_accel_prev;
    float k0_bp_accel_vx;
    float k0_bp_accel_vy;
    float k0_bp_accel_omega;
    float k0_bp_accel_prev;
    float k1_p_vx;
    float k1_p_vy;
    float k1_p_omega;
    float k1_p_accel_prev;
    float k1_pd_vx;
    float k1_pd_vy;
    float k1_pd_omega;
    float k1_pd_accel_prev;
    float k1_atp_vx;
    float k1_atp_vy;
    float k1_atp_omega;
    float k1_gtk_vx;
    float k1_gtk_vy;
    float k1_gtk_omega;
} RiccatiPassDebug_t;

#define RICCATI_DEBUG_TRACE_MAX 256

typedef struct
{
    int iter;
    float primal_residual;
    float dual_residual;
    float state_primal_residual;
    float state_dual_residual;
    float ctrl_primal_residual;
    float ctrl_dual_residual;
    float rho;
    float rho_u;
    float u0_steer;
    float u0_accel;
    float z0_steer;
    float z0_accel;
    float y0_steer;
    float y0_accel;
    float pass_r_lin_steer;
    float pass_r_lin_accel;
    float pass_bp_steer;
    float pass_bp_accel;
    float pass_kk_steer;
    float pass_kk_accel;
    float pass_s00;
    float pass_s11;
    float pass_p_shift_vx;
    float pass_p_shift_vy;
    float pass_p_shift_omega;
    float pass_p_shift_accel_prev;
    float pass_p_shift_ey;
    float pass_p_shift_epsi;
    float pass_p_vx;
    float pass_p_vy;
    float pass_p_omega;
    float pass_p_accel_prev;
    float pass_p_ey;
    float pass_p_epsi;
    float pass_pd_vx;
    float pass_pd_vy;
    float pass_pd_omega;
    float pass_pd_accel_prev;
    float pass_pd_ey;
    float pass_pd_epsi;
    float pass_p_atp_vx;
    float pass_p_atp_vy;
    float pass_p_atp_omega;
    float pass_p_atp_accel_prev;
    float pass_p_gtk_vx;
    float pass_p_gtk_vy;
    float pass_p_gtk_omega;
    float pass_p_gtk_accel_prev;
    float pass_bp_accel_vx;
    float pass_bp_accel_vy;
    float pass_bp_accel_omega;
    float pass_bp_accel_prev;
    float pass_si10;
    float pass_si11;
    float pass_rhs_accel;
    float pass_k0_r_lin_accel;
    float pass_k0_bp_accel;
    float pass_k0_kk_accel;
    float pass_k0_s11;
    float pass_k0_si11;
    float pass_k0_rhs_accel;
    float pass_k0_p_shift_vx;
    float pass_k0_p_shift_vy;
    float pass_k0_p_shift_omega;
    float pass_k0_p_shift_accel_prev;
    float pass_k0_p_vx;
    float pass_k0_p_vy;
    float pass_k0_p_omega;
    float pass_k0_p_accel_prev;
    float pass_k0_pd_vx;
    float pass_k0_pd_vy;
    float pass_k0_pd_omega;
    float pass_k0_pd_accel_prev;
    float pass_k0_bp_accel_vx;
    float pass_k0_bp_accel_vy;
    float pass_k0_bp_accel_omega;
    float pass_k0_bp_accel_prev;
    float pass_k1_p_vx;
    float pass_k1_p_vy;
    float pass_k1_p_omega;
    float pass_k1_p_accel_prev;
    float pass_k1_pd_vx;
    float pass_k1_pd_vy;
    float pass_k1_pd_omega;
    float pass_k1_pd_accel_prev;
    float pass_k1_atp_vx;
    float pass_k1_atp_vy;
    float pass_k1_atp_omega;
    float pass_k1_gtk_vx;
    float pass_k1_gtk_vy;
    float pass_k1_gtk_omega;
    int scale_rho;
    int scale_rho_u;
} RiccatiDebugIterSample_t;

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

void riccati_debug_get_last(RiccatiDebugInfo_t *out);
int riccati_debug_get_trace_count(void);
int riccati_debug_get_trace_sample(int index, RiccatiDebugIterSample_t *out);
void riccati_debug_get_last_pass(RiccatiPassDebug_t *out);

/** Debug flag: set to 1 to print ADMM iteration details */
extern int riccati_admm_debug;

#endif /* RICCATI_SOLVER_H */
