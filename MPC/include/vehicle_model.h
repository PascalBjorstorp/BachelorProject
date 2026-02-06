/**
 * @file vehicle_model.h
 * @brief Kinematic Bicycle Model for F1/10th Vehicle
 *
 * Provides vehicle dynamics prediction for Model Predictive Control.
 * Uses the kinematic bicycle model which is appropriate for low-speed
 * autonomous vehicles like F1/10th cars.
 *
 * Model Equations (continuous time):
 *
 *   dx/dt     = velocity × cos(heading)
 *   dy/dt     = velocity × sin(heading)
 *   dheading/dt = (velocity / wheelbase) × tan(steering)
 *   dvelocity/dt = acceleration
 *
 * Where:
 *   (x, y)      = position in world frame [meters]
 *   heading     = yaw angle [radians]
 *   velocity    = longitudinal velocity [m/s]
 *   steering    = front wheel steering angle [radians]
 *   acceleration = longitudinal acceleration [m/s²]
 *   wheelbase   = distance between front and rear axles [meters]
 *
 * Discretization: Forward Euler method
 *   state[k+1] = state[k] + dt × derivative[k]
 *
 * All calculations use fixed-point arithmetic for FPGA compatibility.
 */

#ifndef VEHICLE_MODEL_H
#define VEHICLE_MODEL_H

#include "mpc_types.h"
#include "fixed_point.h"

/*===========================================================================
 * Model Initialization
 *===========================================================================*/

/**
 * Initialize vehicle model with default F1/10th parameters.
 *
 * Default values:
 * - Wheelbase: 0.32 m
 * - Max steering: 0.42 rad (24°)
 * - Max velocity: 6.0 m/s
 * - Max acceleration: ±4.0 m/s²
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
 * - Acceleration within [min_accel, max_accel]
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
 * Predict the next vehicle state using the kinematic bicycle model.
 *
 * Uses Forward Euler integration:
 *   state[k+1] = state[k] + dt × f(state[k], control[k])
 *
 * The control input is automatically saturated to physical limits.
 *
 * @param current_state   Current vehicle state
 * @param control_input   Control input (steering, acceleration)
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
 * Linearizes the nonlinear bicycle model around (state, control):
 *
 *   state[k+1] ≈ A × state[k] + B × control[k]
 *
 * Where:
 *   A = I + dt × (∂f/∂state)    [4×4 discrete state matrix]
 *   B = dt × (∂f/∂control)      [4×2 discrete input matrix]
 *
 * State ordering: [x, y, heading, velocity]
 * Control ordering: [steering, acceleration]
 *
 * @param operating_state    State to linearize around
 * @param operating_control  Control to linearize around
 * @param time_step          Discretization time step [seconds]
 * @param state_matrix_A     Output: 4×4 state transition matrix
 * @param input_matrix_B     Output: 4×2 input matrix
 */
void vehicle_model_compute_linearization(
    const VehicleState_t *operating_state,
    const ControlInput_t *operating_control,
    fixed_point_t time_step,
    fixed_point_t state_matrix_A[4][4],
    fixed_point_t input_matrix_B[4][2]);

#endif /* VEHICLE_MODEL_H */
