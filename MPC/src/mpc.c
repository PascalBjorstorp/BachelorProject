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

/** Number of control inputs: [steering, velocity] */
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

    /* State tracking weights (raw Q16.16 values, no float) */
    config.weight_position_x = (fixed_point_t)(10 * FIXED_POINT_ONE);  /* 10.0 */
    config.weight_position_y = (fixed_point_t)(10 * FIXED_POINT_ONE);  /* 10.0 */
    config.weight_heading    = (fixed_point_t)(5 * FIXED_POINT_ONE);   /* 5.0  */
    config.weight_velocity   = FIXED_POINT_TWO;                        /* 2.0  */

    /* Control effort weights */
    config.weight_steering_effort  = FIXED_POINT_ONE;   /* 1.0 */
    config.weight_velocity_effort  = FIXED_POINT_HALF;  /* 0.5 */

    /* Control rate weights (smooth control) */
    config.weight_steering_rate  = (fixed_point_t)(10 * FIXED_POINT_ONE);  /* 10.0 */
    config.weight_velocity_rate  = (fixed_point_t)(5 * FIXED_POINT_ONE);   /* 5.0  */

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

    /*
     * QP form: min 0.5 * u^T * H * u + f^T * u
     * For a cost term w*u^2, Hessian entry = 2*w (factor of 2 for the 0.5 to cancel).
     *
     * Rate penalty: sum_k w_r*(u_k - u_{k-1})^2
     * Expanding (u_k - u_{k-1})^2 = u_k^2 - 2*u_k*u_{k-1} + u_{k-1}^2
     *
     * Each interior u_k appears in TWO rate terms:
     *   (u_k - u_{k-1})^2 contributes +w_r to u_k^2 diagonal
     *   (u_{k+1} - u_k)^2 contributes +w_r to u_k^2 diagonal
     * So diagonal gets 2*w_r for interior nodes, w_r for the last node.
     * Then multiply everything by 2 for the QP 0.5 convention.
     */

    fixed_point_t two = FIXED_POINT_TWO;
    fixed_point_t w_steer_effort = current_configuration.weight_steering_effort;
    fixed_point_t w_vel_effort = current_configuration.weight_velocity_effort;
    fixed_point_t w_steer_rate = current_configuration.weight_steering_rate;
    fixed_point_t w_vel_rate = current_configuration.weight_velocity_rate;

    for (int step = 0; step < horizon_steps; step++)
    {
        int base_index = step * CONTROL_DIMENSION;
        int steering_diag = base_index * total_control_variables + base_index;
        int vel_diag = (base_index + 1) * total_control_variables + (base_index + 1);

        /*
         * Diagonal = 2 * (w_effort + rate_contribution)
         * rate_contribution = 2*w_rate for steps 0..N-2 (two adjacent rate terms)
         * rate_contribution =   w_rate for step N-1     (only one rate term)
         */
        fixed_point_t steer_rate_diag;
        fixed_point_t vel_rate_diag;

        if (step < horizon_steps - 1)
        {
            /* Interior or first: u_k appears in (u_k - u_{k-1})^2 AND (u_{k+1} - u_k)^2 */
            steer_rate_diag = fixed_point_mul(two, w_steer_rate);
            vel_rate_diag = fixed_point_mul(two, w_vel_rate);
        }
        else
        {
            /* Last step: u_{N-1} only appears in (u_{N-1} - u_{N-2})^2 */
            steer_rate_diag = w_steer_rate;
            vel_rate_diag = w_vel_rate;
        }

        /* H[k,k] = 2 * (w_effort + rate_diag) */
        hessian_matrix[steering_diag] = fixed_point_mul(two,
            fixed_point_add(w_steer_effort, steer_rate_diag));
        hessian_matrix[vel_diag] = fixed_point_mul(two,
            fixed_point_add(w_vel_effort, vel_rate_diag));

        /* Off-diagonal: H[k-1,k] = H[k,k-1] = -2*w_rate */
        if (step > 0)
        {
            int prev_base = (step - 1) * CONTROL_DIMENSION;

            fixed_point_t neg_2_steer = fixed_point_neg(fixed_point_mul(two, w_steer_rate));
            fixed_point_t neg_2_vel = fixed_point_neg(fixed_point_mul(two, w_vel_rate));

            /* H[prev_steering, current_steering] and symmetric */
            int cross_steer = prev_base * total_control_variables + base_index;
            int cross_steer_sym = base_index * total_control_variables + prev_base;
            hessian_matrix[cross_steer] = neg_2_steer;
            hessian_matrix[cross_steer_sym] = neg_2_steer;

            /* H[prev_velocity, current_velocity] and symmetric */
            int cross_vel = (prev_base + 1) * total_control_variables + (base_index + 1);
            int cross_vel_sym = (base_index + 1) * total_control_variables + (prev_base + 1);
            hessian_matrix[cross_vel] = neg_2_vel;
            hessian_matrix[cross_vel_sym] = neg_2_vel;
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
        fixed_point_t heading_error = fixed_point_sub(
            reference_trajectory[step].reference_heading_radians,
            current_state->heading_angle_radians);

        /* Feedforward steering: proportional to heading error */
        fixed_point_t feedforward_steering = fixed_point_mul(
            heading_error,
            FP_CONST(10.0)); /* Gain = 0.5 */

        /* Feedforward velocity: track reference velocity directly */
        fixed_point_t feedforward_velocity =
            reference_trajectory[step].reference_velocity_meters_per_second;

        /* Linear cost encourages these feedforward values */
        /* f = -2 * w * u_desired */
        linear_cost_vector[base_index] = fixed_point_neg(
            fixed_point_mul(
                FIXED_POINT_TWO,
                fixed_point_mul(
                    current_configuration.weight_steering_effort,
                    feedforward_steering)));

        linear_cost_vector[base_index + 1] = fixed_point_neg(
            fixed_point_mul(
                FIXED_POINT_TWO,
                fixed_point_mul(
                    current_configuration.weight_velocity_effort,
                    feedforward_velocity)));
    }

    /* Add rate penalty for first control (relative to previous)
     * From (u_0 - u_prev)^2: linear contribution is f[0] += -2*w_rate*u_prev
     * The negative sign makes the solver favor u_0 close to u_prev.
     */
    linear_cost_vector[0] = fixed_point_sub(
        linear_cost_vector[0],
        fixed_point_mul(
            FIXED_POINT_TWO,
            fixed_point_mul(
                current_configuration.weight_steering_rate,
                previous_control_input.steering_angle_radians)));

    linear_cost_vector[1] = fixed_point_sub(
        linear_cost_vector[1],
        fixed_point_mul(
            FIXED_POINT_TWO,
            fixed_point_mul(
                current_configuration.weight_velocity_rate,
                previous_control_input.velocity_meters_per_second)));
}

/**
 * Build constraint matrices for actuator limits.
 *
 * Constraints:
 * - Steering angle: |delta| <= max_steering
 * - Velocity: min_velocity <= v <= max_velocity
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

    /* 4 constraints per time step: upper/lower for steering and velocity */
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
            fixed_point_neg(FIXED_POINT_ONE);
        constraint_bounds[constraint_base + 1] =
            vehicle_params.maximum_steering_angle_radians;

        /* Constraint 2: velocity <= max_velocity */
        constraint_matrix[(constraint_base + 2) * total_controls + (control_base + 1)] =
            FIXED_POINT_ONE;
        constraint_bounds[constraint_base + 2] =
            vehicle_params.maximum_velocity_meters_per_second;

        /* Constraint 3: -velocity <= -min_velocity (i.e., v >= min_velocity) */
        constraint_matrix[(constraint_base + 3) * total_controls + (control_base + 1)] =
            fixed_point_neg(FIXED_POINT_ONE);
        constraint_bounds[constraint_base + 3] =
            fixed_point_neg(vehicle_params.minimum_velocity_meters_per_second);
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
    previous_control_input.velocity_meters_per_second = 0;

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
    previous_control_input.velocity_meters_per_second = 0;

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
    fixed_point_t optimal_velocity = qp_solution.optimal_variables[1];

    /* Saturate control to vehicle limits */
    ControlInput_t raw_control;
    raw_control.steering_angle_radians = optimal_steering;
    raw_control.velocity_meters_per_second = optimal_velocity;

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
    previous_control_input.velocity_meters_per_second = 0;
}
