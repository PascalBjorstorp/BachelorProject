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
 * Control Inputs: [delta, acceleration]
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
 * Frenet Frame State (Path-Relative Coordinates)
 *===========================================================================
 * Represents the vehicle state relative to a reference path.
 * Used by the MPC solver for path-following with wall constraints.
 *
 * State vector ordering: [e_y, e_psi, v_x, v_y, omega]
 *
 * Advantages over global XY:
 *   - Lateral error (e_y) directly maps to "distance from path"
 *   - Wall constraints become simple bounds on e_y
 *   - Heading and position tracking work together naturally
 */

typedef struct
{
    /** Lateral error: perpendicular distance from reference path [meters]
     *  Positive = left of path, Negative = right of path */
    fixed_point_t lateral_error_meters;

    /** Heading error: vehicle heading minus path tangent heading [radians] */
    fixed_point_t heading_error_radians;

    /** Longitudinal velocity in body frame [meters per second] */
    fixed_point_t longitudinal_velocity_meters_per_second;

    /** Lateral velocity in body frame [meters per second] */
    fixed_point_t lateral_velocity_meters_per_second;

    /** Yaw rate [radians per second] */
    fixed_point_t yaw_rate_radians_per_second;

} FrenetState_t;

/** Number of states in the Frenet vehicle model */
#define FRENET_STATE_DIMENSION 5

/*===========================================================================
 * Control Input
 ===========================================================================
 * The control signals computed by the MPC solver.
 * Steering angle is sent to the servo; motor torque is converted
 * to a motor command (current/duty) by the VESC controller.
 *
 * Control vector ordering: [delta, T_motor]
 */

typedef struct
{
    /** Front wheel steering angle [radians] */
    fixed_point_t steering_angle_radians;

    /** Motor torque [Newton-meters]
     *  Converted to wheel torque: T_wheel = T_motor / G_ratio
     *  Longitudinal force computed from wheel slip ratio.
     */
    fixed_point_t acceleration_meters_per_second_squared;  /* [m/s²] longitudinal acceleration command */

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
     * Height from center of gravity to ground [meters]
     */
    fixed_point_t height_cg_to_ground_meters;

    /**
     * Gravity acceleration [m/s²]
     */
    fixed_point_t gravity_acceleration_meters_per_second_squared;

    /**
     * Vehicle mass [kg]
     * Typical F1/10th value: 3.314 kg
     */
    fixed_point_t vehicle_mass_kg;

    /**
     * Yaw moment of inertia [kg*m^2]
     * Typical F1/10th value: 0.035 kg*m^2
     */
    fixed_point_t yaw_moment_of_inertia_kgm2;

    /**
     * Front tire cornering stiffness [1/rad]
     * Pure tire property, force = mu * C_Sf * alpha * F_z.
     * Typical F1/10th value: 3.053
     */
    fixed_point_t front_cornering_stiffness;

    /**
     * Rear tire cornering stiffness [1/rad]
     * Pure tire property, force = mu * C_Sr * alpha * F_z.
     * Typical F1/10th value: 5.282
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
     * Maximum motor torque [Newton-meters]
     * T_max = F_x_max * R_w * G_ratio. Typical: 35.57 * 0.0545 * 11.82 ≈ 22.9 N·m
     */
    fixed_point_t maximum_acceleration_meters_per_second_squared;  /* [m/s²] max accel */

    /**
     * Minimum motor torque (braking) [Newton-meters]
     * Negative value. T_min = F_x_min * R_w * G_ratio. Typical: -24.1 N·m
     */
    fixed_point_t minimum_acceleration_meters_per_second_squared;  /* [m/s²] max decel (negative) */

    /** 
     * Longitudinal acceleration [m/s²]
     */
    fixed_point_t longitudinal_acceleration_meters_per_second_squared;

    /**
     * Yaw heading rate [rad/s]
     */
    fixed_point_t omega;

    /*
     * Wheel / Drivetrain Parameters (7-state model)
     */

    /**
     * Wheel radius [meters]
     * Diameter 10.9 cm → R_w = 0.0545 m
     */
    fixed_point_t wheel_radius_meters;

    /**
     * Drivetrain inertia [kg·m²]
     * Combined inertia of wheels + drivetrain (4WD, single equivalent).
     * Typical F1/10th value: 2.223 kg·m²
     */
    fixed_point_t drivetrain_inertia_kgm2;

    /**
     * Longitudinal tire stiffness [Newtons]
     * In the linear slip model: F_x = C_x * κ
     * where κ = (R_w * ω_w - v_x) / max(|v_x|, ε)
     * Typical F1/10th value: 100 N
     */
    fixed_point_t longitudinal_tire_stiffness;

    /**
     * Gear ratio (motor_speed / wheel_speed) [-]
     * T_wheel = T_motor / G_ratio (speed reduction, torque multiplication).
     * F1/10th Velineon 3500kV + spur/pinion: 11.82
     */
    fixed_point_t gear_ratio;

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

    /** Weight for lateral error (e_y) tracking [Frenet] */
    fixed_point_t weight_lateral_error;

    /** Weight for heading error (e_psi) tracking [Frenet] */
    fixed_point_t weight_heading_error;

    /** Weight for longitudinal velocity tracking error */
    fixed_point_t weight_velocity;

    /** Weight for lateral velocity tracking error */
    fixed_point_t weight_lateral_velocity;

    /** Weight for yaw rate tracking error */
    fixed_point_t weight_yaw_rate;

    /** Weight for steering angle magnitude (penalizes large steering) */
    fixed_point_t weight_steering_effort;

    /** Weight for motor torque magnitude (penalizes large torque) */
    fixed_point_t weight_acceleration_effort;

    /** Weight for steering rate (penalizes jerky steering changes) */
    fixed_point_t weight_steering_rate;

    /** Weight for torque rate (penalizes jerky torque changes) */
    fixed_point_t weight_acceleration_rate;

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
 * Reference Trajectory Point (Frenet Frame)
 *===========================================================================
 * Reference point for MPC path-following in Frenet coordinates.
 * For pure path following, lateral_error and heading_error refs are 0.
 * Curvature and wall bounds are properties of the path at this point.
 */

typedef struct
{
    /** Reference lateral error [meters] (0 for path following) */
    fixed_point_t reference_lateral_error_meters;

    /** Reference heading error [radians] (0 for path following) */
    fixed_point_t reference_heading_error_radians;

    /** Target longitudinal velocity [meters per second] */
    fixed_point_t reference_velocity_meters_per_second;

    /** Target lateral velocity [meters per second] (typically 0) */
    fixed_point_t reference_lateral_velocity_meters_per_second;

    /** Target yaw rate [radians per second] */
    fixed_point_t reference_yaw_rate_radians_per_second;

    /** Path curvature at this point [radians per meter]
     *  Used for Frenet frame linearization: e_psi_dot = omega - kappa * v_x
     */
    fixed_point_t path_curvature_radians_per_meter;

    /** Maximum leftward deviation from path before hitting wall [meters]
     *  Always positive. The car must satisfy: e_y <= left_wall_bound */
    fixed_point_t left_wall_bound_meters;

    /** Maximum rightward deviation from path before hitting wall [meters]
     *  Always positive. The car must satisfy: e_y >= -right_wall_bound */
    fixed_point_t right_wall_bound_meters;

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
 * Source: Measured on real F1/10th car (f1tenth_parameters/vehicle_params.yaml)
 * Simulation defaults replaced with physical measurements where available.
 */

/*---------------------------------------------------------------------------
 * Core Kinematic Parameters (used by MPC)
 *---------------------------------------------------------------------------*/

/** F1/10th wheelbase: l_f + l_r = 0.166 + 0.16 = 0.326 m [CAD]
 *  Wheelbase test at 1.0 m/s gave 0.345 m (6.5% high due to understeer).
 *  CAD value 0.324 m is recommended (see report Section 5.6). */
#define F110_DEFAULT_WHEELBASE_METERS \
    FP_CONST(0.324)

/** F1/10th max steering: 0.4282 radians (~24.5 degrees) [TESTED] */
#define F110_DEFAULT_MAXIMUM_STEERING_RADIANS \
    FP_CONST(0.4282)

/** F1/10th max velocity: 20.0 meters per second (simulation limit)
 *  Real car measured 5.17 m/s (test) to ~10 m/s (higher cmd speed). */
#define F110_DEFAULT_MAXIMUM_VELOCITY_METERS_PER_SECOND \
    FP_CONST(20.0)

/** F1/10th minimum velocity: 0 m/s (no reverse) */
#define F110_DEFAULT_MINIMUM_VELOCITY_METERS_PER_SECOND \
    ((fixed_point_t)0)

/** Vehicle width: 0.273 meters [MEASURED] (widest point) */
#define F110_VEHICLE_WIDTH_METERS \
    FP_CONST(0.273)

/** Vehicle length: 0.51 meters [MEASURED] */
#define F110_VEHICLE_LENGTH_METERS \
    FP_CONST(0.51)

/** Distance from CG to front axle: 0.166 meters [CAD] */
#define F110_DIST_CG_TO_FRONT_AXLE_METERS \
    FP_CONST(0.166)

/** Distance from CG to rear axle: 0.16 meters [CAD] */
#define F110_DIST_CG_TO_REAR_AXLE_METERS \
    FP_CONST(0.16)

/** Vehicle mass: 3.314 kg [MEASURED] */
#define F110_VEHICLE_MASS_KG \
    FP_CONST(3.314)

/** Yaw moment of inertia: 0.035 kg·m² [CAD] */
#define F110_YAW_INERTIA_KGM2 \
    FP_CONST(0.035)

/** Center of gravity height: 0.0703 meters [CAD] */
#define F110_CG_HEIGHT_METERS \
    FP_CONST(0.0703)

/** Tire-road friction coefficient [TESTED] mu = 0.7463
 *  From test_friction.py (5 runs, 0.73-0.76 range). Surface-specific. */
#define F110_FRICTION_COEFFICIENT \
    FP_CONST(0.7463)

/** Front cornering stiffness [1/rad] — NEEDS RERUN
 *  Current test used only 1 steering angle (0.1 rad); C_alpha varied 28-80 N/rad.
 *  Keeping simulation default until rerun with full steering sweep.
 *  Conversion: C_Sf = C_alpha_f / (mu * F_zf)
 *    F_zf = m*g*l_r/L = 3.314*9.81*0.16/0.324 = 16.05 N
 *    If C_alpha_f = 49.9 → C_Sf = 49.9/(0.7463*16.05) = 4.17  */
#define F110_FRONT_CORNERING_STIFFNESS \
    FP_CONST(4.17)

/** Rear cornering stiffness [1/rad] — NEEDS RERUN
 *  Same limitation as front. Keeping approximate value.
 *  Conversion: C_Sr = C_alpha_r / (mu * F_zr)
 *    F_zr = m*g*l_f/L = 3.314*9.81*0.166/0.324 = 16.65 N
 *    If C_alpha_r = 54.89 → C_Sr = 54.89/(0.7463*16.65) = 4.42  */
#define F110_REAR_CORNERING_STIFFNESS \
    FP_CONST(4.42)

/** Maximum longitudinal acceleration: 8.0 m/s² [TESTED]
 *  Smoothed IMU value. Raw peak was ~14.5 (pitch-contaminated).
 *  Cross-check: mu*g = 0.79*9.81 = 7.7 m/s² (consistent). */
#define F110_MAX_ACCELERATION_MS2 \
    FP_CONST(8.0)

/** Maximum braking deceleration: 7.7 m/s² [TESTED]
 *  Conservative estimate: mu*g = 0.79*9.81.
 *  Raw IMU peak was ~18.7 (chassis pitch contaminates accelerometer). */
#define F110_MAX_DECELERATION_MS2 \
    FP_CONST(7.7)

/** Maximum longitudinal acceleration [m/s²]
 *  From vehicle_params.yaml: max_accel = 8.0 m/s² [TESTED]
 *  Bounded by mu*g = 0.746 * 9.81 = 7.32 m/s² (tire limit)
 *  Using tested IMU value (smoothed): 8.0 m/s² */
#define F110_DEFAULT_MAX_ACCELERATION \
    FP_CONST(8.0)

/** Minimum longitudinal acceleration (braking) [m/s²]
 *  From vehicle_params.yaml: max_decel = 7.7 m/s² [TESTED] */
#define F110_DEFAULT_MIN_ACCELERATION \
    FP_CONST(-7.7)

/** Wheel radius: 0.051 meters [MEASURED] (loaded effective radius) */
#define F110_WHEEL_RADIUS_METERS \
    FP_CONST(0.051)

/** Drivetrain inertia: 2.223 kg·m² [ESTIMATE/TODO]
 *  This value from simulation seems too high for a 3.3 kg car.
 *  Typical estimate: 0.01-0.05 kg·m².
 *  Needs verification from component inertias or system identification. */
#define F110_DRIVETRAIN_INERTIA_KGM2 \
    FP_CONST(2.223)

/** Longitudinal tire stiffness: F_x = C_x * κ [ESTIMATE]
 *  Cannot be reliably measured without lidar scan-matching odometry.
 *  YAML has 79.5 N from pitch-contaminated test. Using 80 N as approx. */
#define F110_LONGITUDINAL_TIRE_STIFFNESS \
    FP_CONST(80.0)

/** Maximum steering rate: 2.85 rad/s [TESTED]
 *  From test_steering_rate.py: yaw rate 10-90% rise time method.
 *  Includes vehicle dynamics delay (actual servo may be slightly faster). */
#define F110_MAX_STEERING_RATE_RADS \
    FP_CONST(2.85)

/** Default yaw rate: 0 rad/s */
#define F110_DEFAULT_YAW_RATE \
    FP_CONST(0.0)

/** Gravity acceleration: 9.81 m/s² */
#define F110_GRAVITY_ACCELERATION_MS2 \
    FP_CONST(9.81)

/** Center of gravity height: 0.0703 meters [CAD] */
#define F110_HEIGHT_METERS \
    FP_CONST(0.0703)

/** Longitudinal acceleration */
#define F110_LONGITUDINAL_ACCELERATION \
    FP_CONST(0)

/** Latteral acceleration */


/** Gear ratio */
#define F110_GEAR_RATIO \
    FP_CONST(11.82)    

/*===========================================================================
 * Default MPC Configuration
 *===========================================================================*/

/** Default prediction horizon: 20 steps */
#define MPC_DEFAULT_PREDICTION_HORIZON 20

/** Default time step: 0.05 seconds (50 ms) — Q16.16 = 3277
 *  Control rate = 200 Hz (5 ms per call).
 *  Prediction model uses 50ms steps: 10× the control step.
 *  Total lookahead = 20 × 0.05s = 1.0 seconds.
 *  The cross_call_rate_scale = 0.1 (5ms / 50ms).
 */
#define MPC_DEFAULT_TIME_STEP_SECONDS \
    ((fixed_point_t)3277)

/** Default maximum solver iterations.
 *  FPGA target uses a tighter cap for deterministic worst-case latency.
 *  With warm-start, the solver typically converges in 0-10 iterations.
 *  50 iterations provides margin while keeping latency bounded. */
#ifdef MPC_HLS_TARGET
#define MPC_DEFAULT_MAXIMUM_ITERATIONS 50
#else
#define MPC_DEFAULT_MAXIMUM_ITERATIONS 2000
#endif

/** Default convergence tolerance: 0.02 — Q16.16 ~ 1310 */
#define MPC_DEFAULT_CONVERGENCE_TOLERANCE \
    ((fixed_point_t)1310)

#endif /* MPC_TYPES_H */
