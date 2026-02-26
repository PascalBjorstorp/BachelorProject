/**
 * @file vehicle_model.h
 * @brief Dynamic Nonlinear Bicycle Model for F1/10th Vehicle
 *
 * Provides vehicle dynamics prediction for Model Predictive Control.
 * Uses the dynamic bicycle model with linear tire forces and wheel
 * dynamics, appropriate for high-speed autonomous vehicles like F1/10th.
 *
 * State vector (7 states): [x, y, psi, v_x, v_y, omega, omega_w]
 *   x, y       = position in world frame [meters]
 *   psi        = yaw angle (heading) [radians]
 *   v_x        = longitudinal velocity in body frame [m/s]
 *   v_y        = lateral velocity in body frame [m/s]
 *   omega      = yaw rate [rad/s]
 *   omega_w    = wheel angular velocity [rad/s]
 *
 * Control vector (2 inputs): [delta, T_motor]
 *   delta      = front wheel steering angle [radians]
 *   T_motor    = motor torque [N·m]
 *
 * Model Equations (continuous time):
 *   dx/dt      = v_x * cos(psi) - v_y * sin(psi)
 *   dy/dt      = v_x * sin(psi) + v_y * cos(psi)
 *   dpsi/dt    = omega
 *   dv_x/dt    = (F_x - F_yf * sin(delta) + m * v_y * omega) / m
 *   dv_y/dt    = (F_yf * cos(delta) + F_yr - m * v_x * omega) / m
 *   domega/dt  = (l_f * F_yf * cos(delta) - l_r * F_yr) / I_z
 *   domega_w/dt = (T_motor / G_ratio - F_x * R_w) / I_w
 *
 * Longitudinal force via slip ratio:
 *   κ = (R_w * ω_w - v_x) / max(|v_x|, ε)
 *   F_x = C_x * κ
 *
 * Tire model (linear):
 *   alpha_f = delta - atan((v_y + l_f * omega) / v_x)
 *   alpha_r = -atan((v_y - l_r * omega) / v_x)
 *   F_yf = C_Sf * alpha_f * F_zf
 *   F_yr = C_Sr * alpha_r * F_zr
 *
 * Discretization: Forward Euler method
 *   state[k+1] = state[k] + dt * derivative[k]
 *
 * All calculations use fixed-point arithmetic for FPGA compatibility.
 */

#ifndef VEHICLE_MODEL_H
#define VEHICLE_MODEL_H

#include "mpc_types.h"
#include "fp_math.h"


/*===========================================================================
 * Initialization Opcomming
 *===========================================================================*/

/* - Slip angle for braking
   - Slip angle for steering
   - Wheel radius                                            - Measured 
   - Braking torques for front and rear wheels               - From VESC
   - Applied acceleration for throttle                       - From VESC
   - Angular velocity for wheels 
   - Front and rear normal loads
   - Yaw rate                                                - From (IMU)
   - Lateral velocity                                        - From (IMU) / måske observer
   */


/*===========================================================================
 * Model Initialization
 *===========================================================================*/

/**
 * Initialize vehicle model with default F1/10th parameters.
 *
 * Default values:
 * - Wheelbase: 0.3302 m (l_f=0.15875, l_r=0.17145)
 * - Mass: 3.74 kg
 * - Yaw inertia: 0.04712 kg*m^2
 * - Front cornering stiffness: 4.718 N/rad
 * - Rear cornering stiffness: 5.4562 N/rad
 * - Max steering: 0.4189 rad (24 deg)
 * - Max velocity: 20.0 m/s
 * - Max motor torque: 22.9 N·m, Min: -24.1 N·m
 * - Wheel radius: 0.0545 m, Gear ratio: 11.82
 * - Drivetrain inertia: 0.002 kg·m², C_x: 300 N
 */
void vehicle_model_initialize(void);

/**
 * Initialize vehicle model with custom parameters.
 *
 * @param parameters  Pointer to custom vehicle parameter structure
 */
void vehicle_model_initialize_with_parameters(
    const VehicleParameters_t *parameters);

/**
 * Get current vehicle parameters.
 *
 * @return Copy of current vehicle parameter structure
 */
VehicleParameters_t vehicle_model_get_parameters(void);

/*===========================================================================
 * Control Input Saturation
 *===========================================================================*/

/**
 * Clamp control inputs to physical vehicle limits.
 *
 * Ensures:
 * - Steering angle within [-max_steering, +max_steering]
 * - Motor torque within [min_torque, max_torque]
 *
 * @param raw_control  Unconstrained control input
 * @return Constrained control input within physical limits
 */
ControlInput_t vehicle_model_saturate_control(
    const ControlInput_t *raw_control);

/*===========================================================================
 * State Prediction (Single Step)
 *===========================================================================*/

/**
 * Predict the next vehicle state using the dynamic bicycle model.
 *
 * Uses Forward Euler integration:
 *   state[k+1] = state[k] + dt * f(state[k], control[k])
 *
 * Includes tire force computation using linear tire model.
 * The control input is automatically saturated to physical limits.
 *
 * @param current_state   Current vehicle state (7 states)
 * @param control_input   Control input (steering, motor torque)
 * @param time_step       Time step duration [seconds] in fixed-point
 * @return Predicted state after time_step seconds
 */
VehicleState_t vehicle_model_predict_next_state(
    const VehicleState_t *current_state,
    const ControlInput_t *control_input,
    fixed_point_t time_step);

/*===========================================================================
 * Trajectory Prediction (Multiple Steps)
 *===========================================================================*/

/**
 * Predict vehicle trajectory over multiple time steps.
 *
 * Useful for MPC prediction horizon computation.
 *
 * @param initial_state      Starting vehicle state
 * @param control_sequence   Array of control inputs (length = step_count)
 * @param time_step          Time between steps [seconds] in fixed-point
 * @param step_count         Number of prediction steps
 * @param predicted_trajectory  Output array (length = step_count + 1)
 *                              First element is initial_state
 *
 * @note predicted_trajectory must have space for (step_count + 1) states
 */
void vehicle_model_predict_trajectory(
    const VehicleState_t *initial_state,
    const ControlInput_t *control_sequence,
    fixed_point_t time_step,
    uint16_t step_count,
    VehicleState_t *predicted_trajectory);

/*===========================================================================
 * Model Linearization (for Linear MPC)
 *===========================================================================*/

/**
 * Compute linearized state-space matrices at an operating point.
 *
 * Linearizes the nonlinear dynamic bicycle model around (state, control):
 *
 *   state[k+1] ≈ A × state[k] + B × control[k]
 *
 * Where:
 *   A = I + dt × (∂f/∂state)    [7×7 discrete state matrix]
 *   B = dt × (∂f/∂control)      [7×2 discrete input matrix]
 *
 * State ordering: [x, y, heading, v_x, v_y, omega, omega_w]
 * Control ordering: [steering, motor_torque]
 *
 * @param operating_state    State to linearize around
 * @param operating_control  Control to linearize around
 * @param time_step          Discretization time step [seconds]
 * @param state_matrix_A     Output: 7×7 state transition matrix
 * @param input_matrix_B     Output: 7×2 input matrix
 */
void vehicle_model_compute_linearization(
    const VehicleState_t *operating_state,
    const ControlInput_t *operating_control,
    fixed_point_t time_step,
    fixed_point_t state_matrix_A[7][7],
    fixed_point_t input_matrix_B[7][2]);

/*===========================================================================
 * Frenet Frame Linearization
 *===========================================================================*/

/**
 * Compute linearized Frenet-frame state-space matrices.
 *
 * Frenet state: [e_y, e_psi, v_x, v_y, omega, omega_w]
 *   e_y    = lateral error from reference path [meters]
 *   e_psi  = heading error from path tangent [radians]
 *   v_x, v_y, omega, omega_w = same body-frame dynamics
 *
 * Frenet kinematic relations:
 *   e_y_dot   = v_x * sin(e_psi) + v_y * cos(e_psi)  ≈ v_x * e_psi + v_y
 *   e_psi_dot = omega - kappa * v_x * cos(e_psi) / (1 - kappa * e_y)
 *             ≈ omega - kappa * v_x
 *
 * The body-frame dynamics (rows 2-5) are identical to the global model.
 * The Frenet rows (0-1) add path curvature coupling.
 *
 * @param frenet_state       Frenet state to linearize around
 * @param operating_control  Control to linearize around
 * @param time_step          Discretization time step [seconds]
 * @param path_curvature     Path curvature kappa at current point [rad/m]
 * @param state_matrix_A     Output: 6×6 Frenet state transition matrix
 * @param input_matrix_B     Output: 6×2 Frenet input matrix
 */
void vehicle_model_compute_frenet_linearization(
    const FrenetState_t *frenet_state,
    const ControlInput_t *operating_control,
    fixed_point_t time_step,
    fixed_point_t path_curvature,
    fixed_point_t state_matrix_A[FRENET_STATE_DIMENSION][FRENET_STATE_DIMENSION],
    fixed_point_t input_matrix_B[FRENET_STATE_DIMENSION][2]);

#endif /* VEHICLE_MODEL_H */
