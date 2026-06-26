/**
 * @file vehicle_model.h
 * @brief Dynamic nonlinear bicycle model for F1/10th vehicle.
 *
 * Provides vehicle dynamics prediction for Model Predictive Control.
 * Uses the dynamic bicycle model with linear tire forces and wheel
 * dynamics, appropriate for high-speed autonomous vehicles like F1/10th.
 *
 * State vector (6 states): [x, y, psi, v_x, v_y, omega]
 *   x, y       = position in world frame [meters]
 *   psi        = yaw angle (heading) [radians]
 *   v_x        = longitudinal velocity in body frame [m/s]
 *   v_y        = lateral velocity in body frame [m/s]
 *   omega      = yaw rate [rad/s]
 *
 * Control vector (2 inputs): [delta, acceleration]
 *   delta      = front wheel steering angle [radians]
 *   acceleration = longitudinal acceleration [m/s²]
 *
 * Model Equations (continuous time):
 *   dx/dt      = v_x * cos(psi) - v_y * sin(psi)
 *   dy/dt      = v_x * sin(psi) + v_y * cos(psi)
 *   dpsi/dt    = omega
 *   dv_x/dt    = (F_x - F_yf * sin(delta) + m * v_y * omega) / m
 *   dv_y/dt    = (F_yf * cos(delta) + F_yr - m * v_x * omega) / m
 *   domega/dt  = (l_f * F_yf * cos(delta) - l_r * F_yr) / I_z
 *
 * Longitudinal force: F_x = m * a_cmd (direct acceleration input)
 *
 * Tire model:
 *   Prediction uses a linear model:
 *     F_yf = mu * C_Sf * alpha_f * F_zf
 *     F_yr = mu * C_Sr * alpha_r * F_zr
 *   Linearization uses a Pacejka-like model for tire force saturation.
 *
 * Discretization:
 *   - Pose states (x, y, psi): analytical SE(2) integration of constant body twist
 *   - Body dynamic states (v_x, v_y, omega): forward Euler update
 * @dependencies mpc_types.h, util_math.h
 */

#ifndef VEHICLE_MODEL_H
#define VEHICLE_MODEL_H

#include "mpc_types.h"
#include "util_math.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/*===========================================================================
 * Shared Dynamics Helpers
 *===========================================================================*/

/**
 * @brief Compute shared slip-angle intermediate terms at one operating point.
 * @details Evaluates front/rear slip numerators, ratios, and slip angles with
 *          a low-speed longitudinal-velocity floor for numerical conditioning.
 * @param vx Longitudinal velocity in body frame [meters per second].
 * @param vy Lateral velocity in body frame [meters per second].
 * @param omega Yaw rate [radians per second].
 * @param delta Front steering angle [radians].
 * @param slip_terms Output shared slip-angle terms.
 * @return None.
 */
void vehicle_model_compute_slip_terms(
    float vx,
    float vy,
    float omega,
    float delta,
    SlipTerms_t *slip_terms);

/**
 * @brief Compute front and rear normal loads under longitudinal load transfer.
 * @details Uses quasi-static load transfer with the configured vehicle geometry
 *          and weight constants to estimate axle normal loads.
 * @param longitudinal_force Longitudinal force applied at CG [newtons].
 * @param front_normal_load Output front-axle normal load [newtons].
 * @param rear_normal_load Output rear-axle normal load [newtons].
 * @return None.
 */
void vehicle_model_compute_normal_loads(
    float longitudinal_force,
    float *front_normal_load,
    float *rear_normal_load);

/*===========================================================================
 * Control Input Saturation
 *===========================================================================*/

/**
 * @brief Clamp control inputs to physical vehicle limits.
 *
 * Ensures:
 * - Steering angle within [-max_steering, +max_steering]
 * - Longitudinal acceleration within configured min/max bounds
 *
 * @param raw_control  Unconstrained control input
 * @return Constrained control input within physical limits.
 */
ControlInput_t vehicle_model_saturate_control(const ControlInput_t *raw_control);

/*===========================================================================
 * State Prediction (Single Step)
 *===========================================================================*/

/**
 * @brief Predict the next vehicle state using the dynamic bicycle model.
 *
 * Uses analytical pose integration for (x, y, psi) with constant body twist
 * over the step, and Forward Euler for body dynamic states (v_x, v_y, omega).
 *
 * Includes tire force computation using linear tire model.
 * The control input is automatically saturated to physical limits.
 *
 * @param current_state   Current vehicle state (6 states)
 * @param control_input   Control input (steering, acceleration)
 * @param time_step       Integration time step duration [seconds].
 * @return Predicted state after time_step seconds.
 */
VehicleState_t vehicle_model_predict_next_state(
    const VehicleState_t *current_state,
    const ControlInput_t *control_input,
    float time_step);

/*===========================================================================
 * Trajectory Prediction (Multiple Steps)
 *===========================================================================*/

/**
 * @brief Predict vehicle trajectory over multiple time steps.
 *
 * Useful for MPC prediction horizon computation.
 *
 * @param initial_state      Starting vehicle state
 * @param control_sequence   Array of control inputs (length = step_count)
 * @param time_step          Time between steps [seconds]
 * @param step_count         Number of prediction steps
 * @param predicted_trajectory  Output array (length = step_count + 1)
 *                              First element is initial_state
 * @return None. Results are written to predicted_trajectory.
 *
 * @note predicted_trajectory must have space for (step_count + 1) states
 */
void vehicle_model_predict_trajectory(
    const VehicleState_t *initial_state,
    const ControlInput_t *control_sequence,
    float time_step,
    uint16_t step_count,
    VehicleState_t *predicted_trajectory);

/*===========================================================================
 * Tire Linearization Helper
 *===========================================================================*/

/**
 * @brief Compute local effective lateral stiffness for a Pacejka-like tire law.
 * @details Evaluates the nonlinear tire law at the operating slip angle and
 *          returns the local slope dF_y/dalpha, clamped to a minimum
 *          stiffness floor for numerical robustness near saturation.
 * @param use_front_axle Set to 1 for front-axle constants, 0 for rear-axle constants.
 * @param normal_load Tire normal load [newtons].
 * @param slip_angle Operating slip angle [radians].
 * @param effective_stiffness Output local stiffness dF_y/dalpha [newtons per radian].
 * @param lateral_force Optional output lateral force at the operating point [newtons];
 *                      pass NULL when the force value is not needed.
 * @return None.
 */
void vehicle_model_compute_effective_lateral_stiffness(
     uint8_t use_front_axle,
    float normal_load,
    float slip_angle,
    float *effective_stiffness,
    float *lateral_force);

/*===========================================================================
 * Frenet Frame Linearization
 *===========================================================================*/

/**
 * @brief Compute linearized Frenet-frame state-space matrices.
 *
 * Frenet state: [e_y, e_psi, v_x, v_y, omega]
 *   e_y    = lateral error from reference path [meters]
 *   e_psi  = heading error from path tangent [radians]
 *   v_x, v_y, omega = same body-frame dynamics
 *
 * Frenet kinematic relations:
 *   e_y_dot   = v_x * sin(e_psi) + v_y * cos(e_psi)  ≈ v_x * e_psi + v_y
 *   e_psi_dot = omega - kappa * v_x * cos(e_psi) / (1 - kappa * e_y)
 *             ≈ omega - kappa * v_x
 *
 * The body-frame dynamics (rows 2-4) are identical to the global model.
 * The Frenet rows (0-1) add path curvature coupling.
 *
 * @param frenet_state       Frenet state to linearize around
 * @param operating_control  Control to linearize around
 * @param time_step          Discretization time step [seconds]
 * @param path_curvature     Path curvature kappa at current point [rad/m]
 * @param state_matrix_A     Output: 5×5 Frenet state transition matrix
 * @param input_matrix_B     Output: 5×2 Frenet input matrix
 * @return None. Matrices are written to state_matrix_A and input_matrix_B.
 */
void vehicle_model_compute_frenet_linearization(
    const FrenetState_t *frenet_state,
    const ControlInput_t *operating_control,
    float time_step,
    float path_curvature,
    float reference_velocity,
    float state_matrix_A[NX_FRENET][NX_FRENET],
    float input_matrix_B[NX_FRENET][2]);

#endif /* VEHICLE_MODEL_H */
