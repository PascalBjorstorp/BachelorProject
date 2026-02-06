/**
 * @file vehicle_model.c
 * @brief Kinematic Bicycle Model Implementation
 *
 * Implements the kinematic bicycle model for F1/10th vehicle dynamics.
 * All calculations use fixed-point arithmetic for FPGA compatibility.
 *
 * Model Equations:
 *   dx/dt = v × cos(ψ)
 *   dy/dt = v × sin(ψ)
 *   dψ/dt = (v / L) × tan(δ)
 *   dv/dt = a
 *
 * Where:
 *   (x, y) = position, ψ = heading, v = velocity
 *   δ = steering angle, a = acceleration, L = wheelbase
 */

#include "vehicle_model.h"
#include "fixed_point.h"

/*===========================================================================
 * Module State (Vehicle Parameters)
 *===========================================================================*/

/** Current vehicle parameters (initialized by vehicle_model_initialize) */
static VehicleParameters_t stored_vehicle_parameters;

/** Flag indicating if model has been initialized */
static uint8_t model_is_initialized = 0;

/*===========================================================================
 * Initialization Functions
 *===========================================================================*/

void vehicle_model_initialize(void)
{
    stored_vehicle_parameters.wheelbase_meters =
        F110_DEFAULT_WHEELBASE_METERS;

    stored_vehicle_parameters.maximum_steering_angle_radians =
        F110_DEFAULT_MAXIMUM_STEERING_RADIANS;

    stored_vehicle_parameters.maximum_velocity_meters_per_second =
        F110_DEFAULT_MAXIMUM_VELOCITY_METERS_PER_SECOND;

    stored_vehicle_parameters.maximum_acceleration_meters_per_second_squared =
        F110_DEFAULT_MAXIMUM_ACCELERATION;

    stored_vehicle_parameters.minimum_acceleration_meters_per_second_squared =
        F110_DEFAULT_MINIMUM_ACCELERATION;

    model_is_initialized = 1;
}

void vehicle_model_initialize_with_parameters(
    const VehicleParameters_t *parameters)
{
    stored_vehicle_parameters = *parameters;
    model_is_initialized = 1;
}

VehicleParameters_t vehicle_model_get_parameters(void)
{
    return stored_vehicle_parameters;
}

/*===========================================================================
 * Control Saturation
 *===========================================================================*/

ControlInput_t vehicle_model_saturate_control(
    const ControlInput_t *raw_control)
{
    ControlInput_t saturated_control;

    /* Clamp steering angle to physical limits */
    saturated_control.steering_angle_radians = fixed_point_clamp(
        raw_control->steering_angle_radians,
        fixed_point_negate(stored_vehicle_parameters.maximum_steering_angle_radians),
        stored_vehicle_parameters.maximum_steering_angle_radians);

    /* Clamp acceleration to physical limits */
    saturated_control.acceleration_meters_per_second_squared = fixed_point_clamp(
        raw_control->acceleration_meters_per_second_squared,
        stored_vehicle_parameters.minimum_acceleration_meters_per_second_squared,
        stored_vehicle_parameters.maximum_acceleration_meters_per_second_squared);

    return saturated_control;
}

/*===========================================================================
 * Single-Step State Prediction
 *===========================================================================*/

VehicleState_t vehicle_model_predict_next_state(
    const VehicleState_t *current_state,
    const ControlInput_t *control_input,
    fixed_point_t time_step)
{
    VehicleState_t next_state;

    /* Apply control saturation */
    ControlInput_t saturated_control = vehicle_model_saturate_control(control_input);

    /*
     * Compute trigonometric values
     */
    fixed_point_t cosine_of_heading = fixed_point_cosine(
        current_state->heading_angle_radians);

    fixed_point_t sine_of_heading = fixed_point_sine(
        current_state->heading_angle_radians);

    fixed_point_t tangent_of_steering = fixed_point_tangent(
        saturated_control.steering_angle_radians);

    /*
     * Compute state derivatives
     */

    /* dx/dt = velocity × cos(heading) */
    fixed_point_t position_x_derivative = fixed_point_multiply(
        current_state->velocity_meters_per_second,
        cosine_of_heading);

    /* dy/dt = velocity × sin(heading) */
    fixed_point_t position_y_derivative = fixed_point_multiply(
        current_state->velocity_meters_per_second,
        sine_of_heading);

    /* dheading/dt = (velocity / wheelbase) × tan(steering) */
    fixed_point_t velocity_over_wheelbase = fixed_point_divide(
        current_state->velocity_meters_per_second,
        stored_vehicle_parameters.wheelbase_meters);

    fixed_point_t heading_derivative = fixed_point_multiply(
        velocity_over_wheelbase,
        tangent_of_steering);

    /* dvelocity/dt = acceleration */
    fixed_point_t velocity_derivative =
        saturated_control.acceleration_meters_per_second_squared;

    /*
     * Forward Euler integration: state[k+1] = state[k] + dt × derivative
     */

    /* x[k+1] = x[k] + dt × dx/dt */
    next_state.position_x_meters = fixed_point_add(
        current_state->position_x_meters,
        fixed_point_multiply(time_step, position_x_derivative));

    /* y[k+1] = y[k] + dt × dy/dt */
    next_state.position_y_meters = fixed_point_add(
        current_state->position_y_meters,
        fixed_point_multiply(time_step, position_y_derivative));

    /* heading[k+1] = heading[k] + dt × dheading/dt */
    next_state.heading_angle_radians = fixed_point_add(
        current_state->heading_angle_radians,
        fixed_point_multiply(time_step, heading_derivative));

    /* velocity[k+1] = velocity[k] + dt × dvelocity/dt */
    next_state.velocity_meters_per_second = fixed_point_add(
        current_state->velocity_meters_per_second,
        fixed_point_multiply(time_step, velocity_derivative));

    /*
     * Apply state constraints
     */

    /* Clamp velocity to [0, max_velocity] (no reverse) */
    next_state.velocity_meters_per_second = fixed_point_clamp(
        next_state.velocity_meters_per_second,
        0,
        stored_vehicle_parameters.maximum_velocity_meters_per_second);

    /* Normalize heading angle to [-π, +π] */
    while (next_state.heading_angle_radians > FIXED_POINT_PI)
    {
        next_state.heading_angle_radians = fixed_point_subtract(
            next_state.heading_angle_radians,
            FIXED_POINT_TWO_PI);
    }
    while (next_state.heading_angle_radians < -FIXED_POINT_PI)
    {
        next_state.heading_angle_radians = fixed_point_add(
            next_state.heading_angle_radians,
            FIXED_POINT_TWO_PI);
    }

    return next_state;
}

/*===========================================================================
 * Multi-Step Trajectory Prediction
 *===========================================================================*/

void vehicle_model_predict_trajectory(
    const VehicleState_t *initial_state,
    const ControlInput_t *control_sequence,
    fixed_point_t time_step,
    uint16_t step_count,
    VehicleState_t *predicted_trajectory)
{
    /* First element of trajectory is the initial state */
    predicted_trajectory[0] = *initial_state;

    /* Predict each subsequent state */
    for (uint16_t step_index = 0; step_index < step_count; step_index++)
    {
        predicted_trajectory[step_index + 1] = vehicle_model_predict_next_state(
            &predicted_trajectory[step_index],
            &control_sequence[step_index],
            time_step);
    }
}

/*===========================================================================
 * Model Linearization
 *===========================================================================*/

void vehicle_model_compute_linearization(
    const VehicleState_t *operating_state,
    const ControlInput_t *operating_control,
    fixed_point_t time_step,
    fixed_point_t state_matrix_A[4][4],
    fixed_point_t input_matrix_B[4][2])
{
    /*
     * Compute trigonometric values at operating point
     */
    fixed_point_t cosine_heading = fixed_point_cosine(
        operating_state->heading_angle_radians);

    fixed_point_t sine_heading = fixed_point_sine(
        operating_state->heading_angle_radians);

    fixed_point_t tangent_steering = fixed_point_tangent(
        operating_control->steering_angle_radians);

    fixed_point_t cosine_steering = fixed_point_cosine(
        operating_control->steering_angle_radians);

    /* cos²(steering) for derivative calculation */
    fixed_point_t cosine_steering_squared = fixed_point_multiply(
        cosine_steering,
        cosine_steering);

    /*
     * Initialize A matrix as identity matrix
     * A_discrete = I + dt × A_continuous
     */
    for (int row = 0; row < 4; row++)
    {
        for (int col = 0; col < 4; col++)
        {
            state_matrix_A[row][col] = (row == col) ? FIXED_POINT_ONE : 0;
        }
    }

    /*
     * Add continuous-time Jacobian terms multiplied by dt
     *
     * Continuous A matrix (∂f/∂state):
     *   | 0  0  -v×sin(ψ)  cos(ψ)    |
     *   | 0  0   v×cos(ψ)  sin(ψ)    |
     *   | 0  0   0         tan(δ)/L  |
     *   | 0  0   0         0         |
     */

    /* A[0][2] = dt × (-v × sin(heading)) */
    fixed_point_t velocity_times_sine = fixed_point_multiply(
        operating_state->velocity_meters_per_second,
        sine_heading);

    state_matrix_A[0][2] = fixed_point_multiply(
        time_step,
        fixed_point_negate(velocity_times_sine));

    /* A[0][3] = dt × cos(heading) */
    state_matrix_A[0][3] = fixed_point_multiply(time_step, cosine_heading);

    /* A[1][2] = dt × (v × cos(heading)) */
    fixed_point_t velocity_times_cosine = fixed_point_multiply(
        operating_state->velocity_meters_per_second,
        cosine_heading);

    state_matrix_A[1][2] = fixed_point_multiply(time_step, velocity_times_cosine);

    /* A[1][3] = dt × sin(heading) */
    state_matrix_A[1][3] = fixed_point_multiply(time_step, sine_heading);

    /* A[2][3] = dt × (tan(steering) / wheelbase) */
    fixed_point_t tangent_over_wheelbase = fixed_point_divide(
        tangent_steering,
        stored_vehicle_parameters.wheelbase_meters);

    state_matrix_A[2][3] = fixed_point_multiply(time_step, tangent_over_wheelbase);

    /*
     * Initialize B matrix as zeros and add continuous terms × dt
     *
     * Continuous B matrix (∂f/∂control):
     *   | 0                       0 |
     *   | 0                       0 |
     *   | v / (L × cos²(δ))       0 |
     *   | 0                       1 |
     */
    for (int row = 0; row < 4; row++)
    {
        for (int col = 0; col < 2; col++)
        {
            input_matrix_B[row][col] = 0;
        }
    }

    /* B[2][0] = dt × (v / (L × cos²(steering))) */
    fixed_point_t wheelbase_times_cos_squared = fixed_point_multiply(
        stored_vehicle_parameters.wheelbase_meters,
        cosine_steering_squared);

    fixed_point_t velocity_over_denominator = fixed_point_divide(
        operating_state->velocity_meters_per_second,
        wheelbase_times_cos_squared);

    input_matrix_B[2][0] = fixed_point_multiply(time_step, velocity_over_denominator);

    /* B[3][1] = dt × 1 = dt */
    input_matrix_B[3][1] = time_step;
}
