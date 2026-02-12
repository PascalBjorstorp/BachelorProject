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

    /** Commanded longitudinal velocity [meters per second] */
    fixed_point_t velocity_meters_per_second;

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
     * Minimum velocity [meters per second]
     * Typically 0 (no reverse).
     */
    fixed_point_t minimum_velocity_meters_per_second;

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

    /** Weight for velocity magnitude (penalizes large speed commands) */
    fixed_point_t weight_velocity_effort;

    /** Weight for steering rate (penalizes jerky steering changes) */
    fixed_point_t weight_steering_rate;

    /** Weight for velocity rate (penalizes jerky speed changes) */
    fixed_point_t weight_velocity_rate;

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

 
/**
 * Compile-time float-to-Q16.16 conversion macro.
 */
#define FP_CONST(x) ((fixed_point_t)((double)(x) * (1 << FIXED_POINT_FRACTIONAL_BITS)))

/**
 * Runtime double-to-Q16.16 conversion.
 */
#define DOUBLE_TO_FP(x) ((fixed_point_t)((x) * (1 << FIXED_POINT_FRACTIONAL_BITS)))

/**
 * Runtime Q16.16-to-double conversion.
 */
#define FP_TO_DOUBLE(x) ((double)(x) / (double)(1 << FIXED_POINT_FRACTIONAL_BITS))

/**
 * Runtime Q16.16-to-float conversion.
 */
#define FP_TO_FLOAT(x) ((float)(x) / (float)(1 << FIXED_POINT_FRACTIONAL_BITS))



/** F1/10th wheelbase: 0.32 meters (32 cm) — Q16.16 = 20972 */
#define F110_DEFAULT_WHEELBASE_METERS \
    FP_CONST(0.32)

/** F1/10th max steering: 0.4189 radians (~24 degrees) — Q16.16 = 27452 */
#define F110_DEFAULT_MAXIMUM_STEERING_RADIANS \
    ((fixed_point_t)27452)

/** F1/10th max velocity: 6.0 meters per second — Q16.16 = 393216 */
#define F110_DEFAULT_MAXIMUM_VELOCITY_METERS_PER_SECOND \
    FP_CONST(6.0)

/** F1/10th minimum velocity: 0 m/s (no reverse) — Q16.16 = 0 */
#define F110_DEFAULT_MINIMUM_VELOCITY_METERS_PER_SECOND \
    ((fixed_point_t)0)

/*===========================================================================
 * Default MPC Configuration
 *===========================================================================*/

/** Default prediction horizon: 10 steps */
#define MPC_DEFAULT_PREDICTION_HORIZON 10

/** Default time step: 0.05 seconds (50 ms) — Q16.16 = 3277
 *  Total lookahead = 10 × 0.05s = 0.5 seconds */
#define MPC_DEFAULT_TIME_STEP_SECONDS \
    ((fixed_point_t)3277)

/** Default maximum solver iterations */
#define MPC_DEFAULT_MAXIMUM_ITERATIONS 500

/** Default convergence tolerance: 0.001 — Q16.16 ~ 66 */
#define MPC_DEFAULT_CONVERGENCE_TOLERANCE \
    ((fixed_point_t)66)

#endif /* MPC_TYPES_H */
