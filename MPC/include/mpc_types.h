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
 * Units: SI (meters, radians, seconds, Newtons)
 */

#ifndef MPC_TYPES_H
#define MPC_TYPES_H

#include "util_math.h"
#include <stdint.h>


#define FRENET_STATE_DIMENSION 5


/*===========================================================================
 * Vehicle State (Dynamic Bicycle Model)
 *===========================================================================
 */
typedef struct
{
    float pos_x;                /** X position in world frame [meters] */
    float pos_y;                /** Y position in world frame [meters] */
    float heading;              /** Yaw angle relative to world X-axis [radians] */
    float long_vel;             /** Longitudinal velocity in body frame [meters per second] */
    float lat_vel;              /** Lateral velocity in body frame [meters per second] */
    float yaw_rate;             /** Yaw rate [radians per second] */

} VehicleState_t;

/*===========================================================================
 * Frenet Frame State (Path-Relative Coordinates)
 *===========================================================================
 * Represents the vehicle state relative to a reference path.
 * State vector for frenet: [e_y, e_psi, v_x, v_y, omega]
 * */

typedef struct
{
    float flat_error;           /** Lateral error [meters] */
    float fhead_error;          /** Heading error [radians] */
    float flong_vel;            /** Longitudinal velocity [meters per second] */
    float flat_vel;             /** Lateral velocity [meters per second] */
    float fyaw_rate;            /** Yaw rate [radians per second] */

} FrenetState_t;

/*===========================================================================
 * Control Input
 ===========================================================================
 */
typedef struct
{   
    float steer_ang;           /** Front wheel steering angle [radians] */
    float long_acc;            /** Longitudinal acceleration command [m/s²] */

} ControlInput_t;

/*===========================================================================
 * Vehicle Physical Parameters
 *===========================================================================
 * Constants describing the physical properties of the F1/10th car,
 * including dynamic model parameters for tire force computation.
 */

typedef struct
{
    float wheelbase_meters;              /**Wheelbase [meters] */
    float distance_cg_to_front_axle;     /**Distance from center of gravity to front axle [meters]*/
    float distance_cg_to_rear_axle;      /**Distance from center of gravity to rear axle [meters] */
    float height_cg_to_ground;           /**Height from center of gravity to ground [meters]*/
    float gravity_acceleration;          /**Gravity acceleration [m/s²] */
    float vehicle_mass;                  /**Vehicle mass [kg]*/
    float yaw_moment_of_inertia;         /**Yaw moment of inertia [kg*m^2]*/
    float front_cornering_stiffness;     /**Front tire cornering stiffness [1/rad] */
    float rear_cornering_stiffness;      /**Rear tire cornering stiffness [1/rad] */
    float max_steering_angle;            /**Maximum steering angle magnitude [radians]*/    
    float max_velocity;                  /**Maximum forward velocity [meters per second]*/    
    float minvelocity;                   /**Minimum velocity [meters per second]*/
    float max_acceleration;              /** Maximum longitudinal acceleration [m/s²] */
    float min_acceleration;              /** Minimum longitudinal acceleration (braking) [m/s²] */

} VehicleParameters_t;

/*===========================================================================
 * MPC Solver Configuration
 *===========================================================================
 * Parameters that control the MPC optimization behavior.
 */

typedef struct
{
    
    uint16_t prediction_horizon_steps;  /**Prediction horizon: number of future time steps to consider */
    float time_step;                    /**Time step duration [seconds] */

    /*---------------------------------------------------------------------------
     Cost function weights: 
     ---------------------------------------------------------------------------*/

    
    float weight_lateral_error;         /** Weight for lateral error tracking [Frenet] */
    float weight_heading_error;         /** Weight for heading error tracking [Frenet] */
    float weight_velocity;              /** Weight for longitudinal velocity tracking error */
    float weight_lateral_velocity;      /** Weight for lateral velocity tracking error */
    float weight_yaw_rate;              /** Weight for yaw rate tracking error */
    float weight_steering_effort;       /** Weight for steering angle magnitude */
    float weight_acceleration_effort;   /** Weight for motor torque magnitude */
    float weight_steering_rate;         /** Weight for steering rate */
    float weight_acceleration_rate;     /** Weight for acceleration rate **/
    float cross_call_rate_scale;        /** Cross-call rate penalty scale factor.
                                        * Scales the rate penalty between the current first control u[0]
                                        * and the previous MPC output u_prev. */

    /*---------------------------------------------------------------------------
    Solver convergence parameters
    ---------------------------------------------------------------------------*/

    uint16_t max_solver_iterations;      /** Maximum QP solver iterations */
    float solver_convergence_tolerance;  /** Convergence tolerance for solver */

} MpcConfiguration_t;

/*===========================================================================
 * Reference Trajectory Point (Frenet Frame)
 *===========================================================================
*/

typedef struct
{
    float reference_lateral_error;       /** Reference lateral error [meters] */
    float reference_heading_error;       /** Reference heading error [radians] */
    float reference_velocity;            /** Target longitudinal velocity */    
    float reference_lateral_velocity;    /** Target lateral velocity [meters per second] */
    float reference_yaw_rate;            /** Target yaw rate [radians per second] */ 
    float path_curvature;                /** Path curvature at this point [radians per meter]
                                          *  Used for Frenet frame linearization: e_psi_dot = omega - kappa * v_x */
    float left_wall_bound;               /** Maximum leftward deviation from path before hitting wall [meters] */
    float right_wall_bound;              /** Maximum rightward deviation from path before hitting wall [meters]*/

} TrajectoryReferencePoint_t;

/*===========================================================================
 * MPC Solver Status
 *===========================================================================*/

typedef enum
{
    MPC_STATUS_SUCCESS = 0,                      /** Optimal solution found successfully */
    MPC_STATUS_MAXIMUM_ITERATIONS_REACHED = 1,   /** Solver reached maximum iterations */
    MPC_STATUS_INFEASIBLE = 2,                   /** No feasible solution exists for given constraints */
    MPC_STATUS_ERROR = 3                         /** Solver encountered an error */

} MpcSolverStatus_t;

/*===========================================================================
 * MPC Solver Result
 *===========================================================================
 * Complete output from the MPC solver.
 */

typedef struct
{
    MpcSolverStatus_t solver_status;            /** Solver termination status */
    ControlInput_t optimal_control;             /** Optimal control input for current time step */
    uint16_t iterations_used;                   /** Number of solver iterations used */
    float final_cost;                           /** Final cost function value */   
    float dual_residual;                        /** Final dual residual (ADMM convergence metric) */

} MpcSolverResult_t;

/*===========================================================================
 * Default Parameters for F1/10th Vehicle
 *===========================================================================
 * Pre-computed constants for F1/10th configuration.
 * Parameters are measured through testing and CAD schematics.
 */

/*---------------------------------------------------------------------------
 * Core Kinematic Parameters
 *---------------------------------------------------------------------------*/

/** F1/10th wheelbase: l_f + l_r = 0.166 + 0.16 = 0.326 m [CAD] */
#define F110_DEFAULT_WHEELBASE_METERS 0.324f

/** F1/10th max steering: 0.4189 radians (~24.0 degrees) [CALIBRATED] */
#define F110_DEFAULT_MAXIMUM_STEERING_RADIANS 0.4189f

/** F1/10th max velocity: 20.0 meters per second  */
#define F110_DEFAULT_MAXIMUM_VELOCITY_METERS_PER_SECOND 20.0f

/** F1/10th minimum velocity: 0 m/s (no reverse) */
#define F110_DEFAULT_MINIMUM_VELOCITY_METERS_PER_SECOND 0.0f

/** Distance from CG to front axle: 0.166 meters [CAD] */
#define F110_DIST_CG_TO_FRONT_AXLE_METERS 0.166f

/** Distance from CG to rear axle: 0.16 meters [CAD] */
#define F110_DIST_CG_TO_REAR_AXLE_METERS 0.16f

/** Vehicle mass: 3.314 kg [MEASURED] */
#define F110_VEHICLE_MASS_KG 3.314f

/** Yaw moment of inertia: 0.035 kg·m² [CAD] */
#define F110_YAW_INERTIA_KGM2 0.035f

/** Center of gravity height: 0.0703 meters [CAD] */
#define F110_CG_HEIGHT_METERS 0.0703f

/** Tire-road friction coefficient [TESTED] */
#define F110_FRICTION_COEFFICIENT 0.787f

/** Gravity acceleration: 9.81 m/s² */
#define F110_GRAVITY_ACCELERATION_MS2 9.81f

/** Maximum longitudinal acceleration bounded by mu*g  [m/s²] */
#define F110_DEFAULT_MAX_ACCELERATION       F110_FRICTION_COEFFICIENT * F110_GRAVITY_ACCELERATION_MS2

/** Minimum longitudinal acceleration (braking) [m/s²] [TESTED]*/
#define F110_DEFAULT_MIN_ACCELERATION       -F110_DEFAULT_MAX_ACCELERATION

/** Cornoring stiffness for front wheel */
#define VP_C_ALPHA_F 51.40f

/** Cornoring stiffness for rear wheel */
#define VP_C_ALPHA_R 43.10f

/** Normal force on front wheel */
#define VP_NORM_LOAD_F         F110_VEHICLE_MASS_KG * F110_GRAVITY_ACCELERATION_MS2 * F110_DIST_CG_TO_REAR_AXLE_METERS / F110_DEFAULT_WHEELBASE_METERS

/** Normal force on rear wheel */
#define VP_NORM_LOAD_R         F110_VEHICLE_MASS_KG * F110_GRAVITY_ACCELERATION_MS2 * F110_DIST_CG_TO_FRONT_AXLE_METERS / F110_DEFAULT_WHEELBASE_METERS

/** Peak force on front wheel D */
#define VP_D_FRONT        F110_FRICTION_COEFFICIENT * VP_NORM_LOAD_F

/** Peak force on rear wheel D */
#define VP_D_REAR        F110_FRICTION_COEFFICIENT * VP_NORM_LOAD_R

/** Shape factor C */
#define VP_C_SHAPE 1.9f

/** Inverse shape factor */
#define VP_INV_C_SHAPE 1/VP_C_SHAPE

/** Scalar to prevent at low speeds */
#define MIN_STIFF_SCALE 0.1f

/** Front cornering stiffness [1/rad] */
#define F110_FRONT_CORNERING_STIFFNESS      VP_C_ALPHA_F / (F110_FRICTION_COEFFICIENT*VP_D_FRONT)

/** Rear cornering stiffness [1/rad] */
#define F110_REAR_CORNERING_STIFFNESS   VP_C_ALPHA_R / (F110_FRICTION_COEFFICIENT*VP_D_REAR)

/** B factors for pacejka nonlinearity model */
#define VP_B_FRONT  F110_FRONT_CORNERING_STIFFNESS / VP_C_SHAPE
#define VP_B_REAR   F110_REAR_CORNERING_STIFFNESS / VP_C_SHAPE

/* C_shape * B aliases used in Jacobian terms (equal to C_alpha_S* by construction). */
#define VP_CB_FRONT F110_FRONT_CORNERING_STIFFNESS
#define VP_CB_REAR  F110_REAR_CORNERING_STIFFNESS

/* Precomputed minimum effective stiffness factors (mu*C_S*MIN_STIFF_SCALE) */
#define VP_MU_CSF_MIN           F110_FRICTION_COEFFICIENT * F110_FRONT_CORNERING_STIFFNESS * MIN_STIFF_SCALE
#define VP_MU_CSR_MIN           F110_FRICTION_COEFFICIENT * F110_REAR_CORNERING_STIFFNESS * MIN_STIFF_SCALE



/*=========================================================================== 
 * Internal Constants
 * ==========================================================================/

 /** Frenet state dimension (e_y, e_psi, vx, vy, omega) */
#define NX_FRENET 5

/** Augmented state dimension:
 *  [e_y, e_psi, vx, vy, omega, delta_actual, delta_rate_prev, accel_prev]
 *  States 0-5 form the "dense block" in the Riccati pass (6x6).
 *  States 6-7 are the "previous control" states (zero in A). */
#define NX_AUG 8

/** Index of delta_actual in the augmented state vector */
#define IDX_DELTA_ACTUAL 5

/** Index of delta_rate_prev in the augmented state vector */
#define IDX_DRATE_PREV 6

/** Index of accel_prev in the augmented state vector */
#define IDX_ACCEL_PREV 7

/** Dense block size in A matrix (Frenet + delta_actual) */
#define NX_DENSE 6

/** Control dimension (delta_rate, acceleration) */
#define NU 2

/** Maximum steering rate (rad/s) */
#define MAX_STEERING_RATE 2.849f

/** Big number for unconstrained states */
#define BIG_BOUND 100.0f

/** Minimum linearization velocity */
#define MIN_LINEARIZATION_VELOCITY 2.0f

/** A-row stability limit */
#define STABILITY_LIMIT 0.95f

/** Wall constraint margin  */
#define WALL_MARGIN_DEFAULT 0.855f

/** Wall constraints: only first few horizon steps for near-term safety.
 *  Override at runtime via WALL_END environment variable.
 *  WALL_STRIDE controls step spacing (1=every step, 2=every other, etc.).
 *  Override at runtime via WALL_STRIDE environment variable. */
#define WALL_CONSTRAINT_START  1
#define WALL_CONSTRAINT_STRIDE_DEFAULT 3
#define WALL_CONSTRAINT_END_DEFAULT 20    /* last horizon step to constrain (0=disable) */

/** Soft wall constraint stiffness (0 = hard box constraint).
 *  When > 0, wall constraints use a quadratic penalty instead of hard clipping:
 *    g(z) = (k/2) * max(0, z - ub)^2 + (k/2) * max(0, lb - z)^2
 *  The ADMM z-update uses the proximal operator, allowing controlled violation.
 *  Higher k = stiffer (500+ approaches hard). Lower k = more flexible.
 *  Recommended: 200-500 for tight corridors, 0 for wide tracks.
 *  Override at runtime via WALL_SOFT_K environment variable. */
#define WALL_SOFT_STIFFNESS_DEFAULT 657.0f

/** v_switch: above this velocity, max acceleration = a_max * v_switch / v.
 *  From f1tenth gym STDynamicsModel: v_switch = 7.319 m/s.
 *  Models constant-power regime: P = F*v = const → a_max(v) ∝ 1/v. */
#define V_SWITCH 7.319f

/** Maximum lateral acceleration for curvature-based velocity limiting [m/s²].
 *  v_max(κ) = √(a_lat_max / |κ|), capping reference velocities in corners.
 *  Physically correct value: mu*g = 0.787 * 9.81 = 7.72 m/s².
 *  Override at runtime via MPC_MAX_LAT_ACCEL environment variable. */
#define MPC_MAX_LAT_ACCEL_DEFAULT 7.3212f

/*===========================================================================
 * Default MPC Configuration
 *===========================================================================*/

/** Default prediction horizon: 20 steps */
#define MPC_PREDICTION_HORIZON 20

/** Default time step: 0.048 seconds (48 ms)
 *  Control rate = 200 Hz (5 ms per call).
 *  Prediction model uses 40ms steps: 8× the control step.
 *  Total lookahead = 20 × 0.04s = 0.8 seconds.
 *  The cross_call_rate_scale = 0.125 (5ms / 40ms).
 */
#define MPC_TIME_STEP_SECONDS 0.048f

/** Default maximum solver iterations.
 *  FPGA target uses a tighter cap for deterministic worst-case latency.
 *  With warm-start and optimized tolerance (5.0), the solver converges
 *  in 1-2 iterations on average (max observed: 5). 8 provides margin. */
#define MPC_MAXIMUM_ITERATIONS 20

/** Maximum number of waypoints in loaded trajectory */
#define TRAJECTORY_MAXIMUM_WAYPOINTS 1000

/** Maximum reference velocity [m/s] */
#define TRAJECTORY_MAXIMUM_VELOCITY 20.0f

/** Default convergence tolerance: 5.0 — optimized for warm-start MPC
 *  With warm-start + rho=15 persistence, tolerance=5.0 gives ~1.1 avg
 *  iterations at 200Hz with excellent tracking (avg lat 0.106m).
 *  Higher tolerance exploits warm-start quality — solution changes little
 *  between consecutive calls, so coarse convergence suffices. */
#define MPC_CONVERGENCE_TOLERANCE 5.0f

/**
 * Get the default MPC configuration (F1/10th tuned values).
 * Defined in mpc.c but needed by alternative MPC implementations.
 */
MpcConfiguration_t get_default_configuration(void);

#endif /* MPC_TYPES_H */
