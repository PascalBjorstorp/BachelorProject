/**
 * @file mpc_types.h
 * @brief Type Definitions for Model Predictive Control System
 *
 * Defines all data structures used in the MPC system:
 * - Vehicle state representation (dynamic bicycle model)
 * - Control inputs (steering + longitudinal force)
 * - Vehicle physical parameters (including tire dynamics)
 * - MPC configuration
 * - Trajectory references
 * - Solver results
 *
 * Dynamic Bicycle Model States: [x, y, psi, v_x, v_y, omega]
 * Control Inputs: [delta, F_x]
 *
 * All numerical values use Q16.16 fixed-point for FPGA compatibility.
 * Units: SI (meters, radians, seconds, Newtons)
 */

#ifndef MPC_TYPES_H
#define MPC_TYPES_H

#include "fp_math.h"
#include <stdint.h>

/*===========================================================================
 * Vehicle State (Dynamic Bicycle Model)
 *===========================================================================
 * Represents the current state of the vehicle in the world frame.
 * Uses the 6-state dynamic bicycle model with tire dynamics.
 * This is the INPUT to the MPC solver (from localization or simulator).
 *
 * State vector ordering: [x, y, psi, v_x, v_y, omega]
 */

typedef struct
{
    /** X position in world frame [meters] */
    fixed_point_t position_x_meters;

    /** Y position in world frame [meters] */
    fixed_point_t position_y_meters;

    /** Yaw angle (heading) relative to world X-axis [radians] */
    fixed_point_t heading_angle_radians;

    /** Longitudinal velocity in body frame [meters per second] */
    fixed_point_t longitudinal_velocity_meters_per_second;

    /** Lateral velocity in body frame [meters per second] */
    fixed_point_t lateral_velocity_meters_per_second;

    /** Yaw rate [radians per second] */
    fixed_point_t yaw_rate_radians_per_second;

} VehicleState_t;

/*===========================================================================
 * Control Input
 *===========================================================================
 * The control signals computed by the MPC solver.
 * Steering angle is sent to the servo; longitudinal force is converted
 * to a motor command (velocity/duty) by the low-level controller.
 *
 * Control vector ordering: [delta, F_x]
 */

typedef struct
{
    /** Front wheel steering angle [radians] */
    fixed_point_t steering_angle_radians;

    /** Longitudinal force applied at the rear axle [Newtons] */
    fixed_point_t longitudinal_force_newtons;

} ControlInput_t;

/*===========================================================================
 * Vehicle Physical Parameters
 *===========================================================================
 * Constants describing the physical properties of the F1/10th car,
 * including dynamic model parameters for tire force computation.
 */

typedef struct
{
    /**
     * Wheelbase: distance between front and rear axles [meters]
     * Equal to l_f + l_r. Typical F1/10th value: ~0.33 m
     */
    fixed_point_t wheelbase_meters;

    /**
     * Distance from center of gravity to front axle [meters]
     * Typical F1/10th value: 0.15875 m
     */
    fixed_point_t distance_cg_to_front_axle_meters;

    /**
     * Distance from center of gravity to rear axle [meters]
     * Typical F1/10th value: 0.17145 m
     */
    fixed_point_t distance_cg_to_rear_axle_meters;

    /**
     * Vehicle mass [kg]
     * Typical F1/10th value: 3.74 kg
     */
    fixed_point_t vehicle_mass_kg;

    /**
     * Yaw moment of inertia [kg*m^2]
     * Typical F1/10th value: 0.04712 kg*m^2
     */
    fixed_point_t yaw_moment_of_inertia_kgm2;

    /**
     * Front tire cornering stiffness [N/rad]
     * Linear tire model: F_yf = -C_Sf * alpha_f
     * Typical F1/10th value: 4.718
     */
    fixed_point_t front_cornering_stiffness;

    /**
     * Rear tire cornering stiffness [N/rad]
     * Linear tire model: F_yr = -C_Sr * alpha_r
     * Typical F1/10th value: 5.4562
     */
    fixed_point_t rear_cornering_stiffness;

    /**
     * Maximum steering angle magnitude [radians]
     * Physical limit of the steering servo.
     * Typical F1/10th value: ~0.42 rad (24 degrees)
     */
    fixed_point_t maximum_steering_angle_radians;

    /**
     * Maximum forward velocity [meters per second]
     * Safe operating speed limit for state clamping.
     * Typical F1/10th value: ~20.0 m/s
     */
    fixed_point_t maximum_velocity_meters_per_second;

    /**
     * Minimum velocity [meters per second]
     * Typically 0 (no reverse). Used for state clamping.
     */
    fixed_point_t minimum_velocity_meters_per_second;

    /**
     * Maximum longitudinal force [Newtons]
     * F_x_max = m * a_max. Typical: 3.74 * 9.51 = 35.6 N
     */
    fixed_point_t maximum_longitudinal_force_newtons;

    /**
     * Minimum longitudinal force (braking) [Newtons]
     * Negative value. F_x_min = -m * a_brake. Typical: -37.4 N
     */
    fixed_point_t minimum_longitudinal_force_newtons;

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

    /** Weight for longitudinal velocity tracking error */
    fixed_point_t weight_velocity;

    /** Weight for lateral velocity tracking error */
    fixed_point_t weight_lateral_velocity;

    /** Weight for yaw rate tracking error */
    fixed_point_t weight_yaw_rate;

    /** Weight for steering angle magnitude (penalizes large steering) */
    fixed_point_t weight_steering_effort;

    /** Weight for longitudinal force magnitude (penalizes large force) */
    fixed_point_t weight_force_effort;

    /** Weight for steering rate (penalizes jerky steering changes) */
    fixed_point_t weight_steering_rate;

    /** Weight for force rate (penalizes jerky force changes) */
    fixed_point_t weight_force_rate;

    /** Cross-call rate penalty scale factor.
     *
     * Scales the rate penalty between the current first control u[0]
     * and the previous MPC output u_prev. This accounts for the MPC
     * being called at a different rate than the prediction time step dt.
     *
     * Set to FP_ONE (1.0) when MPC call interval ≈ dt (e.g., offline test).
     * Set to dt_actual/dt_step (e.g., 0.1 for 5ms calls with 50ms dt).
     *
     * Without proper scaling, the cross-call rate penalty is too strong
     * at high call frequencies, causing ping-pong oscillation.
     */
    fixed_point_t cross_call_rate_scale;

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

    /** Target longitudinal velocity [meters per second] */
    fixed_point_t reference_velocity_meters_per_second;

    /** Target lateral velocity [meters per second] (typically 0) */
    fixed_point_t reference_lateral_velocity_meters_per_second;

    /** Target yaw rate [radians per second] */
    fixed_point_t reference_yaw_rate_radians_per_second;

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
 * Pre-computed fixed-point constants for F1/10th configuration.
 * Source: f1tenth_gym simulation (F110Env.f1tenth_vehicle_params)
 */

/*---------------------------------------------------------------------------
 * Core Kinematic Parameters (used by MPC)
 *---------------------------------------------------------------------------*/

/** F1/10th wheelbase: 0.3302 meters — distance between front and rear axles */
#define F110_DEFAULT_WHEELBASE_METERS \
    FP_CONST(0.3302)

/** F1/10th max steering: 0.4189 radians (~24 degrees) */
#define F110_DEFAULT_MAXIMUM_STEERING_RADIANS \
    FP_CONST(0.4189)

/** F1/10th max velocity: 20.0 meters per second (simulation limit) */
#define F110_DEFAULT_MAXIMUM_VELOCITY_METERS_PER_SECOND \
    FP_CONST(20.0)

/** F1/10th minimum velocity: 0 m/s (no reverse) */
#define F110_DEFAULT_MINIMUM_VELOCITY_METERS_PER_SECOND \
    ((fixed_point_t)0)

/** Vehicle width: 0.31 meters (for safety margin calculations) */
#define F110_VEHICLE_WIDTH_METERS \
    FP_CONST(0.31)

/** Vehicle length: 0.58 meters */
#define F110_VEHICLE_LENGTH_METERS \
    FP_CONST(0.53)

/** Distance from CG to front axle: 0.15875 meters */
#define F110_DIST_CG_TO_FRONT_AXLE_METERS \
    FP_CONST(0.15875)

/** Distance from CG to rear axle: 0.17145 meters */
#define F110_DIST_CG_TO_REAR_AXLE_METERS \
    FP_CONST(0.17145)

/** Vehicle mass: 3.74 kg */
#define F110_VEHICLE_MASS_KG \
    FP_CONST(3.314)

/** Yaw moment of inertia: 0.04712 kg·m² */
#define F110_YAW_INERTIA_KGM2 \
    FP_CONST(0.04712)

/** Center of gravity height: 0.074 meters */
#define F110_CG_HEIGHT_METERS \
    FP_CONST(0.074)

/** Wheel radius: 0.0508 meters */
#define F110_WHEEL_RADIUS_METERS \
    FP_CONST(0.0508)

/** Tire-road friction coefficient */
#define F110_FRICTION_COEFFICIENT \
    FP_CONST(1.0489)

/** Front cornering stiffness: 4.718 [1/rad] */
#define F110_FRONT_CORNERING_STIFFNESS \
    FP_CONST(4.718)

/** Rear cornering stiffness: 5.4562 [1/rad] (> front → slight understeer) */
#define F110_REAR_CORNERING_STIFFNESS \
    FP_CONST(5.4562)

/** Maximum longitudinal acceleration: 9.51 m/s² */
#define F110_MAX_ACCELERATION_MS2 \
    FP_CONST(9.51)

/** Maximum braking deceleration: 10.0 m/s² */
#define F110_MAX_DECELERATION_MS2 \
    FP_CONST(10.0)

/** Maximum longitudinal force: m * a_max = 3.74 * 9.51 ≈ 35.57 N */
#define F110_DEFAULT_MAX_LONGITUDINAL_FORCE_NEWTONS \
    FP_CONST(35.57)

/** Minimum longitudinal force (braking): -m * a_brake = -3.74 * 10.0 = -37.4 N */
#define F110_DEFAULT_MIN_LONGITUDINAL_FORCE_NEWTONS \
    FP_CONST(-37.4)

/** Maximum steering rate: 3.2 rad/s */
#define F110_MAX_STEERING_RATE_RADS \
    FP_CONST(3.2)

/*===========================================================================
 * Default MPC Configuration
 *===========================================================================*/

/** Default prediction horizon: 10 steps */
#define MPC_DEFAULT_PREDICTION_HORIZON 10

/** Default time step: 0.05 seconds (50 ms) — Q16.16 = 3277
 *  Total lookahead = 10 × 0.05s = 0.5 seconds
 *  The prediction dt is independent of the MPC call rate (200 Hz).
 *  Use cross_call_rate_scale to handle the frequency mismatch.
 */
#define MPC_DEFAULT_TIME_STEP_SECONDS \
    ((fixed_point_t)3277)

/** Default maximum solver iterations */
#define MPC_DEFAULT_MAXIMUM_ITERATIONS 2000

/** Default convergence tolerance: 0.001 — Q16.16 ~ 66 */
#define MPC_DEFAULT_CONVERGENCE_TOLERANCE \
    ((fixed_point_t)66)

#endif /* MPC_TYPES_H */
