/**
 * @file mpc.c
 * @brief Model Predictive Control Implementation
 *
 * Implements the MPC controller for F1/10th autonomous vehicle.
 * Uses quadratic programming to optimize control inputs over a
 * prediction horizon while respecting vehicle constraints.
 *
 * Algorithm Overview:
 * 1. Linearize vehicle model around current state
 * 2. Build QP cost matrices from tracking error and control effort
 * 3. Build QP constraint matrices from actuator limits
 * 4. Solve QP using projected gradient descent
 * 5. Extract first control input from optimal sequence
 *
 * All arithmetic uses Q16.16 fixed-point for FPGA compatibility.
 */

#include "mpc.h"
#include "qp_solver.h"
#include "linear_algebra.h"
#include "vehicle_model.h"
#include <string.h>

/*===========================================================================
 * Internal Constants
 *===========================================================================*/

/** Number of states in vehicle model: [x, y, heading, velocity] */
#define STATE_DIMENSION 4

/** Number of control inputs: [steering, acceleration] */
#define CONTROL_DIMENSION 2

/** Maximum prediction horizon supported */
#define MAXIMUM_HORIZON_STEPS 20

/*===========================================================================
 * Module State (Static)
 *===========================================================================*/

/** Current MPC configuration */
static MpcConfiguration_t current_configuration;

/** Flag indicating MPC has been initialized */
static int mpc_initialized_flag = 0;

/** Previous control input (for rate limiting) */
static ControlInput_t previous_control_input;

/*===========================================================================
 * Default Configuration
 *===========================================================================*/

static MpcConfiguration_t get_default_configuration(void)
{
    MpcConfiguration_t config;

    /* Prediction horizon and timing */
    config.prediction_horizon_steps = MPC_DEFAULT_PREDICTION_HORIZON;
    config.time_step_seconds = MPC_DEFAULT_TIME_STEP_SECONDS;

    /* State tracking weights */
    config.weight_position_x = fixed_point_from_float(10.0f);
    config.weight_position_y = fixed_point_from_float(10.0f);
    config.weight_heading = fixed_point_from_float(5.0f);
    config.weight_velocity = fixed_point_from_float(2.0f);

    /* Control effort weights */
    config.weight_steering_effort = fixed_point_from_float(1.0f);
    config.weight_acceleration_effort = fixed_point_from_float(0.5f);

    /* Control rate weights (smooth control) */
    config.weight_steering_rate = fixed_point_from_float(10.0f);
    config.weight_acceleration_rate = fixed_point_from_float(5.0f);

    /* Solver parameters */
    config.maximum_solver_iterations = MPC_DEFAULT_MAXIMUM_ITERATIONS;
    config.solver_convergence_tolerance = MPC_DEFAULT_CONVERGENCE_TOLERANCE;

    return config;
}

/*===========================================================================
 * QP Problem Construction
 *===========================================================================*/

/**
 * Build the QP Hessian matrix from cost weights.
 *
 * The Hessian H is a block-diagonal matrix where each block
 * corresponds to the control weights at one time step.
 *
 * H = diag(R, R, ..., R) where R = [w_steering, 0; 0, w_accel]
 *
 * Plus additional terms for rate penalties:
 * H_rate penalizes (u_k - u_{k-1})^2
 *
 * @param horizon_steps Number of prediction steps
 * @param hessian_matrix Output Hessian matrix (row-major)
 */
static void build_qp_hessian_matrix(
    int horizon_steps,
    fixed_point_t *hessian_matrix)
{
    int total_control_variables = horizon_steps * CONTROL_DIMENSION;

    /* Clear matrix to zero */
    memset(hessian_matrix, 0,
           total_control_variables * total_control_variables * sizeof(fixed_point_t));

    /* Control effort weights (diagonal blocks) */
    for (int step = 0; step < horizon_steps; step++)
    {
        int base_index = step * CONTROL_DIMENSION;

        /* Steering weight: H[base, base] */
        int steering_idx = base_index * total_control_variables + base_index;
        hessian_matrix[steering_idx] = current_configuration.weight_steering_effort;

        /* Acceleration weight: H[base+1, base+1] */
        int accel_idx = (base_index + 1) * total_control_variables + (base_index + 1);
        hessian_matrix[accel_idx] = current_configuration.weight_acceleration_effort;
    }

    /* Control rate weights (penalize u_k - u_{k-1}) */
    /* This adds cross-terms to the Hessian */
    fixed_point_t rate_weight_steering = current_configuration.weight_steering_rate;
    fixed_point_t rate_weight_accel = current_configuration.weight_acceleration_rate;

    for (int step = 0; step < horizon_steps; step++)
    {
        int base_index = step * CONTROL_DIMENSION;
        int steering_diag = base_index * total_control_variables + base_index;
        int accel_diag = (base_index + 1) * total_control_variables + (base_index + 1);

        /* Add rate weight to diagonal */
        hessian_matrix[steering_diag] = fixed_point_add(
            hessian_matrix[steering_diag], rate_weight_steering);
        hessian_matrix[accel_diag] = fixed_point_add(
            hessian_matrix[accel_diag], rate_weight_accel);

        /* Add negative rate weight to off-diagonal (u_k * u_{k-1} terms) */
        if (step > 0)
        {
            int prev_base = (step - 1) * CONTROL_DIMENSION;

            /* H[prev_steering, current_steering] and symmetric */
            int cross_steer = prev_base * total_control_variables + base_index;
            int cross_steer_sym = base_index * total_control_variables + prev_base;
            hessian_matrix[cross_steer] = fixed_point_negate(rate_weight_steering);
            hessian_matrix[cross_steer_sym] = fixed_point_negate(rate_weight_steering);

            /* H[prev_accel, current_accel] and symmetric */
            int cross_accel = (prev_base + 1) * total_control_variables + (base_index + 1);
            int cross_accel_sym = (base_index + 1) * total_control_variables + (prev_base + 1);
            hessian_matrix[cross_accel] = fixed_point_negate(rate_weight_accel);
            hessian_matrix[cross_accel_sym] = fixed_point_negate(rate_weight_accel);
        }
    }
}

/**
 * Build the QP linear cost vector from reference trajectory.
 *
 * The linear term f comes from tracking the reference trajectory.
 * For each predicted state, we penalize deviation from reference.
 *
 * After linearization, this becomes:
 * f = -2 * (Q * reference_deviation) transformed to control space
 *
 * For now, simplified version: penalize control relative to expected
 *
 * @param current_state Current vehicle state
 * @param reference_trajectory Array of reference points
 * @param horizon_steps Number of prediction steps
 * @param linear_cost_vector Output linear cost vector
 */
static void build_qp_linear_cost_vector(
    const VehicleState_t *current_state,
    const TrajectoryReferencePoint_t *reference_trajectory,
    int horizon_steps,
    fixed_point_t *linear_cost_vector)
{
    int total_control_variables = horizon_steps * CONTROL_DIMENSION;

    /* Clear vector to zero */
    memset(linear_cost_vector, 0, total_control_variables * sizeof(fixed_point_t));

    /*
     * Compute feedforward control to track reference.
     * Using simple proportional approach: steering proportional to heading error.
     */
    for (int step = 0; step < horizon_steps; step++)
    {
        int base_index = step * CONTROL_DIMENSION;

        /* Desired heading change */
        fixed_point_t heading_error = fixed_point_subtract(
            reference_trajectory[step].reference_heading_radians,
            current_state->heading_angle_radians);

        /* Feedforward steering: proportional to heading error */
        fixed_point_t feedforward_steering = fixed_point_multiply(
            heading_error,
            fixed_point_from_float(0.5f)); /* Gain */

        /* Velocity error for feedforward acceleration */
        fixed_point_t velocity_error = fixed_point_subtract(
            reference_trajectory[step].reference_velocity_meters_per_second,
            current_state->velocity_meters_per_second);

        fixed_point_t feedforward_acceleration = fixed_point_multiply(
            velocity_error,
            fixed_point_from_float(1.0f)); /* Gain */

        /* Linear cost encourages these feedforward values */
        /* f = -2 * w * u_desired */
        linear_cost_vector[base_index] = fixed_point_negate(
            fixed_point_multiply(
                fixed_point_from_float(2.0f),
                fixed_point_multiply(
                    current_configuration.weight_steering_effort,
                    feedforward_steering)));

        linear_cost_vector[base_index + 1] = fixed_point_negate(
            fixed_point_multiply(
                fixed_point_from_float(2.0f),
                fixed_point_multiply(
                    current_configuration.weight_acceleration_effort,
                    feedforward_acceleration)));
    }

    /* Add rate penalty for first control (relative to previous) */
    linear_cost_vector[0] = fixed_point_add(
        linear_cost_vector[0],
        fixed_point_multiply(
            fixed_point_from_float(2.0f),
            fixed_point_multiply(
                current_configuration.weight_steering_rate,
                previous_control_input.steering_angle_radians)));

    linear_cost_vector[1] = fixed_point_add(
        linear_cost_vector[1],
        fixed_point_multiply(
            fixed_point_from_float(2.0f),
            fixed_point_multiply(
                current_configuration.weight_acceleration_rate,
                previous_control_input.acceleration_meters_per_second_squared)));
}

/**
 * Build constraint matrices for actuator limits.
 *
 * Constraints:
 * - Steering angle: |delta| <= max_steering
 * - Acceleration: min_accel <= a <= max_accel
 *
 * In matrix form: A * u <= b
 * Where each control has upper and lower bounds.
 *
 * @param horizon_steps Number of prediction steps
 * @param constraint_matrix Output constraint matrix
 * @param constraint_bounds Output constraint bounds vector
 * @param constraint_count Output number of constraints
 */
static void build_qp_constraints(
    int horizon_steps,
    fixed_point_t *constraint_matrix,
    fixed_point_t *constraint_bounds,
    uint16_t *constraint_count)
{
    VehicleParameters_t vehicle_params = vehicle_model_get_parameters();
    int total_controls = horizon_steps * CONTROL_DIMENSION;

    /* 4 constraints per time step: upper/lower for steering and accel */
    int constraints_per_step = 4;
    *constraint_count = (uint16_t)(horizon_steps * constraints_per_step);
    int total_constraints = horizon_steps * constraints_per_step;

    /* Clear matrices */
    memset(constraint_matrix, 0,
           total_constraints * total_controls * sizeof(fixed_point_t));

    for (int step = 0; step < horizon_steps; step++)
    {
        int control_base = step * CONTROL_DIMENSION;
        int constraint_base = step * constraints_per_step;

        /* Constraint 0: steering <= max_steering */
        constraint_matrix[(constraint_base + 0) * total_controls + control_base] =
            FIXED_POINT_ONE;
        constraint_bounds[constraint_base + 0] =
            vehicle_params.maximum_steering_angle_radians;

        /* Constraint 1: -steering <= max_steering (i.e., steering >= -max) */
        constraint_matrix[(constraint_base + 1) * total_controls + control_base] =
            fixed_point_negate(FIXED_POINT_ONE);
        constraint_bounds[constraint_base + 1] =
            vehicle_params.maximum_steering_angle_radians;

        /* Constraint 2: acceleration <= max_accel */
        constraint_matrix[(constraint_base + 2) * total_controls + (control_base + 1)] =
            FIXED_POINT_ONE;
        constraint_bounds[constraint_base + 2] =
            vehicle_params.maximum_acceleration_meters_per_second_squared;

        /* Constraint 3: -acceleration <= -min_accel (i.e., accel >= min_accel) */
        constraint_matrix[(constraint_base + 3) * total_controls + (control_base + 1)] =
            fixed_point_negate(FIXED_POINT_ONE);
        constraint_bounds[constraint_base + 3] =
            fixed_point_negate(vehicle_params.minimum_acceleration_meters_per_second_squared);
    }
}

/*===========================================================================
 * Public API Implementation
 *===========================================================================*/

void mpc_initialize(void)
{
    current_configuration = get_default_configuration();

    /* Initialize vehicle model */
    vehicle_model_initialize();

    /* Clear previous control */
    previous_control_input.steering_angle_radians = 0;
    previous_control_input.acceleration_meters_per_second_squared = 0;

    mpc_initialized_flag = 1;
}

void mpc_initialize_with_configuration(const MpcConfiguration_t *configuration)
{
    if (configuration != NULL)
    {
        current_configuration = *configuration;
    }
    else
    {
        current_configuration = get_default_configuration();
    }

    /* Initialize vehicle model */
    vehicle_model_initialize();

    /* Clear previous control */
    previous_control_input.steering_angle_radians = 0;
    previous_control_input.acceleration_meters_per_second_squared = 0;

    mpc_initialized_flag = 1;
}

MpcSolverStatus_t mpc_compute_optimal_control(
    const VehicleState_t *current_vehicle_state,
    const TrajectoryReferencePoint_t *reference_trajectory,
    MpcSolverResult_t *result)
{
    /* Validate inputs */
    if (current_vehicle_state == NULL ||
        reference_trajectory == NULL ||
        result == NULL)
    {
        if (result != NULL)
        {
            result->solver_status = MPC_STATUS_ERROR;
        }
        return MPC_STATUS_ERROR;
    }

    if (!mpc_initialized_flag)
    {
        mpc_initialize();
    }

    /* Get horizon (capped to maximum) */
    int horizon = current_configuration.prediction_horizon_steps;
    if (horizon > MAXIMUM_HORIZON_STEPS)
    {
        horizon = MAXIMUM_HORIZON_STEPS;
    }

    int total_controls = horizon * CONTROL_DIMENSION;

    /* Build QP problem */
    QuadraticProgramProblem_t qp_problem;
    QuadraticProgramConfig_t qp_config;
    QuadraticProgramSolution_t qp_solution;

    qp_solver_initialize_problem(&qp_problem);
    qp_solver_initialize_config(&qp_config);

    /* Set problem dimensions */
    qp_problem.variable_count = total_controls;

    /* Build Hessian matrix */
    build_qp_hessian_matrix(horizon, qp_problem.hessian_matrix);

    /* Build linear cost vector */
    build_qp_linear_cost_vector(
        current_vehicle_state,
        reference_trajectory,
        horizon,
        qp_problem.linear_cost_vector);

    /* Build constraints */
    build_qp_constraints(
        horizon,
        qp_problem.constraint_matrix,
        qp_problem.constraint_bounds,
        &qp_problem.constraint_count);

    /* Configure solver */
    qp_config.maximum_iterations = current_configuration.maximum_solver_iterations;
    qp_config.convergence_tolerance = current_configuration.solver_convergence_tolerance;

    /* Solve QP */
    QuadraticProgramStatus_t qp_status = qp_solver_solve(
        &qp_problem, &qp_config, &qp_solution);

    /* Extract first control from solution */
    fixed_point_t optimal_steering = qp_solution.optimal_variables[0];
    fixed_point_t optimal_acceleration = qp_solution.optimal_variables[1];

    /* Saturate control to vehicle limits */
    ControlInput_t raw_control;
    raw_control.steering_angle_radians = optimal_steering;
    raw_control.acceleration_meters_per_second_squared = optimal_acceleration;

    ControlInput_t saturated_control = vehicle_model_saturate_control(&raw_control);

    /* Fill result structure */
    result->optimal_control = saturated_control;
    result->iterations_used = qp_solution.iteration_count;
    result->final_cost = qp_solution.constraint_residual; /* Using residual as cost proxy */

    /* Map QP status to MPC status */
    switch (qp_status)
    {
    case QP_STATUS_OPTIMAL:
        result->solver_status = MPC_STATUS_SUCCESS;
        break;
    case QP_STATUS_MAXIMUM_ITERATIONS_REACHED:
        result->solver_status = MPC_STATUS_MAXIMUM_ITERATIONS_REACHED;
        break;
    case QP_STATUS_INFEASIBLE:
        result->solver_status = MPC_STATUS_INFEASIBLE;
        break;
    default:
        result->solver_status = MPC_STATUS_ERROR;
        break;
    }

    /* Store control for next iteration's rate penalty */
    previous_control_input = saturated_control;

    return result->solver_status;
}

MpcConfiguration_t mpc_get_configuration(void)
{
    return current_configuration;
}

void mpc_set_configuration(const MpcConfiguration_t *configuration)
{
    if (configuration != NULL)
    {
        current_configuration = *configuration;
    }
}

void mpc_reset(void)
{
    /* Clear previous control (no rate penalty on first control after reset) */
    previous_control_input.steering_angle_radians = 0;
    previous_control_input.acceleration_meters_per_second_squared = 0;
}
