/**
 * @file mpc_types.h
 * @brief Type Definitions for Model Predictive Control System
 *
 * Defines all data structures used in the MPC system:
 * - Vehicle state representation
 * - Control inputs
 * - Vehicle physical parameters
 * - MPC configuration
 * - Trajectory references
 * - Solver results
 *
 * All numerical values use Q16.16 fixed-point for FPGA compatibility.
 * Units: SI (meters, radians, seconds)
 */

#ifndef MPC_TYPES_H
#define MPC_TYPES_H

#include "fixed_point.h"
#include <stdint.h>

/*===========================================================================
 * Vehicle State
 *===========================================================================
 * Represents the current state of the vehicle in the world frame.
 * This is the INPUT to the MPC solver (from localization or simulator).
 */

typedef struct
{
    /** X position in world frame [meters] */
    fixed_point_t position_x_meters;

    /** Y position in world frame [meters] */
    fixed_point_t position_y_meters;

    /** Yaw angle (heading) relative to world X-axis [radians] */
    fixed_point_t heading_angle_radians;

    /** Longitudinal velocity [meters per second] */
    fixed_point_t velocity_meters_per_second;

} VehicleState_t;

/*===========================================================================
 * Control Input
 *===========================================================================
 * The control signals sent to the vehicle actuators (VESC).
 * This is the OUTPUT of the MPC solver.
 */

typedef struct
{
    /** Front wheel steering angle [radians] */
    fixed_point_t steering_angle_radians;

    /** Longitudinal acceleration command [meters per second squared] */
    fixed_point_t acceleration_meters_per_second_squared;

} ControlInput_t;

/*===========================================================================
 * Vehicle Physical Parameters
 *===========================================================================
 * Constants describing the physical properties of the F1/10th car.
 */

typedef struct
{
    /**
     * Wheelbase: distance between front and rear axles [meters]
     * Determines the turning radius for a given steering angle.
     * Typical F1/10th value: ~0.32 m
     */
    fixed_point_t wheelbase_meters;

    /**
     * Maximum steering angle magnitude [radians]
     * Physical limit of the steering servo.
     * Typical F1/10th value: ~0.42 rad (24 degrees)
     */
    fixed_point_t maximum_steering_angle_radians;

    /**
     * Maximum forward velocity [meters per second]
     * Safe operating speed limit.
     * Typical F1/10th value: ~6.0 m/s
     */
    fixed_point_t maximum_velocity_meters_per_second;

    /**
     * Maximum acceleration (throttle) [meters per second squared]
     * Motor/traction limit for acceleration.
     * Typical F1/10th value: ~4.0 m/s²
     */
    fixed_point_t maximum_acceleration_meters_per_second_squared;

    /**
     * Maximum deceleration (braking) [meters per second squared]
     * Motor/friction limit for braking (stored as negative value).
     * Typical F1/10th value: ~-4.0 m/s²
     */
    fixed_point_t minimum_acceleration_meters_per_second_squared;

} VehicleParameters_t;

/*===========================================================================
 * MPC Solver Configuration
 *===========================================================================
 * Parameters that control the MPC optimization behavior.
 */

typedef struct
{
    /**
     * Prediction horizon: number of future time steps to consider.
     * Longer horizon = better planning but more computation.
     * Typical value: 10-20 steps
     */
    uint16_t prediction_horizon_steps;

    /**
     * Time step duration [seconds]
     * Time between consecutive prediction steps.
     * Typical value: 0.05-0.1 seconds (50-100 ms)
     */
    fixed_point_t time_step_seconds;

    /*
     * Cost function weights:
     * Higher weight = more penalty for deviation from reference
     */

    /** Weight for X position tracking error */
    fixed_point_t weight_position_x;

    /** Weight for Y position tracking error */
    fixed_point_t weight_position_y;

    /** Weight for heading angle tracking error */
    fixed_point_t weight_heading;

    /** Weight for velocity tracking error */
    fixed_point_t weight_velocity;

    /** Weight for steering angle magnitude (penalizes large steering) */
    fixed_point_t weight_steering_effort;

    /** Weight for acceleration magnitude (penalizes aggressive acceleration) */
    fixed_point_t weight_acceleration_effort;

    /** Weight for steering rate (penalizes jerky steering changes) */
    fixed_point_t weight_steering_rate;

    /** Weight for acceleration rate (penalizes jerky speed changes) */
    fixed_point_t weight_acceleration_rate;

    /*
     * Solver convergence parameters
     */

    /** Maximum QP solver iterations */
    uint16_t maximum_solver_iterations;

    /** Convergence tolerance for solver */
    fixed_point_t solver_convergence_tolerance;

} MpcConfiguration_t;

/*===========================================================================
 * Reference Trajectory Point
 *===========================================================================
 * A single point on the desired path the car should follow.
 * 
 * reference_position_x_meters, reference_position_y_meters: desired position
 * reference_heading_radians: desired heading angle at this point
 * reference_velocity_meters_per_second: desired speed at this point
 */

typedef struct
{
    /** Target X position [meters] */
    fixed_point_t reference_position_x_meters;

    /** Target Y position [meters] */
    fixed_point_t reference_position_y_meters;

    /** Target heading angle [radians] */
    fixed_point_t reference_heading_radians;

    /** Target velocity [meters per second] */
    fixed_point_t reference_velocity_meters_per_second;

} TrajectoryReferencePoint_t;

/*===========================================================================
 * MPC Solver Status
 *===========================================================================*/

typedef enum
{
    /** Optimal solution found successfully */
    MPC_STATUS_SUCCESS = 0,

    /** Solver reached maximum iterations (solution may still be usable) */
    MPC_STATUS_MAXIMUM_ITERATIONS_REACHED = 1,

    /** No feasible solution exists for given constraints */
    MPC_STATUS_INFEASIBLE = 2,

    /** Solver encountered an error */
    MPC_STATUS_ERROR = 3

} MpcSolverStatus_t;

/*===========================================================================
 * MPC Solver Result
 *===========================================================================
 * Complete output from the MPC solver.
 */

typedef struct
{
    /** Solver termination status */
    MpcSolverStatus_t solver_status;

    /** Optimal control input for current time step */
    ControlInput_t optimal_control;

    /** Number of solver iterations used */
    uint16_t iterations_used;

    /** Final cost function value */
    fixed_point_t final_cost;

} MpcSolverResult_t;

/*===========================================================================
 * Default Parameters for F1/10th Vehicle
 *===========================================================================
 * Pre-computed fixed-point constants for typical F1/10th configuration.
 */

/** F1/10th wheelbase: 0.32 meters (32 cm) */
#define F110_DEFAULT_WHEELBASE_METERS \
    (0.32f)

/** F1/10th max steering: 0.4189 radians (~24 degrees) */
#define F110_DEFAULT_MAXIMUM_STEERING_RADIANS \
    (0.4189f)

/** F1/10th max velocity: 6.0 meters per second */
#define F110_DEFAULT_MAXIMUM_VELOCITY_METERS_PER_SECOND \
    (6.0f)

/** F1/10th max acceleration: 4.0 meters per second squared */
#define F110_DEFAULT_MAXIMUM_ACCELERATION \
    (4.0f)

/** F1/10th max braking: -4.0 meters per second squared */
#define F110_DEFAULT_MINIMUM_ACCELERATION \
    (-4.0f)

/*===========================================================================
 * Default MPC Configuration
 *===========================================================================*/

/** Default prediction horizon: 10 steps */
#define MPC_DEFAULT_PREDICTION_HORIZON 10

/** Default time step: 0.1 seconds (100 ms) */
#define MPC_DEFAULT_TIME_STEP_SECONDS \
    (0.1f)

/** Default maximum solver iterations: 100 */
#define MPC_DEFAULT_MAXIMUM_ITERATIONS 1000

/** Default convergence tolerance: 0.001 */
#define MPC_DEFAULT_CONVERGENCE_TOLERANCE \
    (0.001f)

#endif /* MPC_TYPES_H */
