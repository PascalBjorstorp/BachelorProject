/**
 * @file mpc_types.h
 * @brief Type definitions for the Model Predictive Control system.
 * @details Defines all data structures used by the MPC pipeline, including
 *          vehicle state, control inputs, physical parameters, solver
 *          configuration, trajectory references, and solver outputs.
 *          Dynamic Bicycle Model States: [x, y, psi, v_x, v_y, omega].
 *          Control Inputs: [delta, acceleration].
 *          Units: SI (meters, radians, seconds, Newtons).
 * @dependencies util_math.h, <stdint.h>
 */

#ifndef MPC_TYPES_H
#define MPC_TYPES_H

#include "util_math.h"
#include <stdint.h>


#define FRENET_STATE_DIMENSION 5

/*===========================================================================
 * Global MPC Constants
 *===========================================================================*/

#define MPC_TWO_PI (2.0 * M_PI)

/* Timing */
#define MPC_CONTROL_RATE_HZ 200.0f
#define MPC_CONTROL_DT_SECONDS (1.0f / MPC_CONTROL_RATE_HZ)
#define MPC_PREDICTION_DT_SECONDS 0.034f
#define MPC_DEFAULT_CROSS_CALL_RATE_SCALE (MPC_CONTROL_DT_SECONDS / MPC_PREDICTION_DT_SECONDS)

/* Default MPC weights */
#define MPC_WEIGHT_LAT_ERROR_DEFAULT 4779.0f
#define MPC_WEIGHT_HEADING_DEFAULT 762.129f
#define MPC_WEIGHT_VELOCITY_DEFAULT 138.6f
#define MPC_WEIGHT_LAT_VEL_DEFAULT 3.3f
#define MPC_WEIGHT_YAW_RATE_DEFAULT 3.0f
#define MPC_WEIGHT_STEER_EFFORT_DEFAULT 0.4f
#define MPC_WEIGHT_ACCEL_EFFORT_DEFAULT 0.0108f
#define MPC_WEIGHT_STEER_RATE_DEFAULT 0.3f
#define MPC_WEIGHT_ACCEL_RATE_DEFAULT 0.095f
#define MPC_WEIGHT_DELTA_ACTUAL_DEFAULT 0.5f
#define MPC_EMA_ALPHA_DEFAULT 0.7f
#define MPC_RICCATI_COST_FACTOR 2.0f

/* Model and solver constants */
#define MPC_STEERING_RATE_LIMIT 2.849f
#define MPC_STEERING_FEEDFORWARD_CLAMP_FACTOR 0.5f
#define MPC_BIG_BOUND 100.0f
#define MPC_MIN_LINEARIZATION_VELOCITY 2.0f
#define MPC_STABILITY_LIMIT 0.95f
#define MPC_WALL_MARGIN_DEFAULT 0.0085f
#define MPC_WALL_CONSTRAINT_START 1
#define MPC_WALL_CONSTRAINT_STRIDE_DEFAULT 2
#define MPC_WALL_CONSTRAINT_END_DEFAULT 10
#define MPC_V_SWITCH 7.319f
#define MPC_CONVERGENCE_TOLERANCE_DEFAULT 3.260281f
#define MPC_MIN_SLIP_VELOCITY 0.5f
#define MPC_WARMSTART_CURVATURE_RESET_THRESHOLD 0.5f
#define MPC_ADMM_RHO_DEFAULT 50.0f
#define MPC_ADMM_RHO_U_DEFAULT 12.0f
#define MPC_ADMM_ALPHA_DEFAULT 1.4f

/* Hardware/sim defaults */
#define MPC_MIN_TRAJECTORY_SPEED_MPS 1.0


/*===========================================================================
 * Vehicle State (Dynamic Bicycle Model)
 *===========================================================================
 */
typedef struct
{
    float pos_x;                /**< X position in world frame [meters]. */
    float pos_y;                /**< Y position in world frame [meters]. */
    float heading;              /**< Yaw angle relative to world X-axis [radians]. */
    float long_vel;             /**< Longitudinal velocity in body frame [meters per second]. */
    float lat_vel;              /**< Lateral velocity in body frame [meters per second]. */
    float yaw_rate;             /**< Yaw rate [radians per second]. */

} VehicleState_t;

/*===========================================================================
 * Frenet Frame State (Path-Relative Coordinates)
 *===========================================================================
 * Represents the vehicle state relative to a reference path.
 * State vector for frenet: [e_y, e_psi, v_x, v_y, omega]
 * */

typedef struct
{
    float flat_error;           /**< Lateral error [meters]. */
    float fhead_error;          /**< Heading error [radians]. */
    float flong_vel;            /**< Longitudinal velocity [meters per second]. */
    float flat_vel;             /**< Lateral velocity [meters per second]. */
    float fyaw_rate;            /**< Yaw rate [radians per second]. */

} FrenetState_t;

/*===========================================================================
 * Control Input
 ===========================================================================
 */
typedef struct
{   
    float steer_ang;            /**< Front wheel steering angle [radians]. */
    float long_acc;             /**< Longitudinal acceleration command [m/s^2]. */

} ControlInput_t;

/*===========================================================================
 * Vehicle Physical Parameters
 *===========================================================================
 * Constants describing the physical properties of the F1/10th car,
 * including dynamic model parameters for tire force computation.
 */

typedef struct
{
    float wheelbase_meters;              /**< Wheelbase [meters]. */
    float distance_cg_to_front_axle;     /**< Distance from center of gravity to front axle [meters]. */
    float distance_cg_to_rear_axle;      /**< Distance from center of gravity to rear axle [meters]. */
    float height_cg_to_ground;           /**< Height from center of gravity to ground [meters]. */
    float gravity_acceleration;          /**< Gravity acceleration [m/s^2]. */
    float vehicle_mass;                  /**< Vehicle mass [kg]. */
    float yaw_moment_of_inertia;         /**< Yaw moment of inertia [kg*m^2]. */
    float front_cornering_stiffness;     /**< Front tire cornering stiffness [1/rad]. */
    float rear_cornering_stiffness;      /**< Rear tire cornering stiffness [1/rad]. */
    float max_steering_angle;            /**< Maximum steering angle magnitude [radians]. */
    float max_velocity;                  /**< Maximum forward velocity [meters per second]. */
    float min_velocity;                  /**< Minimum velocity [meters per second]. */
    float max_acceleration;              /**< Maximum longitudinal acceleration [m/s^2]. */
    float min_acceleration;              /**< Minimum longitudinal acceleration (braking) [m/s^2]. */

} VehicleParameters_t;

/*===========================================================================
 * MPC Solver Configuration
 *===========================================================================
 * Parameters that control the MPC optimization behavior.
 */

typedef struct
{
    
    uint16_t prediction_horizon_steps;  /**< Prediction horizon: number of future time steps to consider. */
    float time_step;                    /**< Time step duration [seconds]. */

    /*---------------------------------------------------------------------------
     Cost function weights: 
     ---------------------------------------------------------------------------*/

    
    float weight_lateral_error;         /**< Weight for lateral error tracking [Frenet]. */
    float weight_heading_error;         /**< Weight for heading error tracking [Frenet]. */
    float weight_velocity;              /**< Weight for longitudinal velocity tracking error. */
    float weight_lateral_velocity;      /**< Weight for lateral velocity tracking error. */
    float weight_yaw_rate;              /**< Weight for yaw rate tracking error. */
    float weight_steering_effort;       /**< Weight for steering angle magnitude. */
    float weight_acceleration_effort;   /**< Weight for motor torque magnitude. */
    float weight_steering_rate;         /**< Weight for steering rate. */
    float weight_acceleration_rate;     /**< Weight for acceleration rate. */
    float weight_delta_actual;          /**< Weight for steering centering state. */
    float cross_call_rate_scale;        /**< Cross-call rate penalty scale factor.
                                         *   Scales the rate penalty between the current first control u[0]
                                         *   and the previous MPC output u_prev. */

    /*---------------------------------------------------------------------------
    Solver convergence parameters
    ---------------------------------------------------------------------------*/

    uint16_t max_solver_iterations;      /**< Maximum QP solver iterations. */
    float solver_convergence_tolerance;  /**< Convergence tolerance for solver. */

} MpcConfiguration_t;

/*===========================================================================
 * Reference Trajectory Point (Frenet Frame)
 *===========================================================================
*/

typedef struct
{
    float reference_lateral_error;       /**< Reference lateral error [meters]. */
    float reference_heading_error;       /**< Reference heading error [radians]. */
    float reference_velocity;            /**< Target longitudinal velocity [meters per second]. */
    float reference_lateral_velocity;    /**< Target lateral velocity [meters per second]. */
    float reference_yaw_rate;            /**< Target yaw rate [radians per second]. */
    float path_curvature;                /**< Path curvature at this point [radians per meter].
                                          *   Used for Frenet linearization: e_psi_dot = omega - kappa * v_x. */
    float left_wall_bound;               /**< Maximum leftward deviation from path before wall contact [meters]. */
    float right_wall_bound;              /**< Maximum rightward deviation from path before wall contact [meters]. */

} TrajectoryReferencePoint_t;

/*===========================================================================
 * MPC Solver Status
 *===========================================================================*/

typedef enum
{
    MPC_STATUS_SUCCESS = 0,                      /**< Optimal solution found successfully. */
    MPC_STATUS_MAXIMUM_ITERATIONS_REACHED = 1,   /**< Solver reached maximum iterations. */
    MPC_STATUS_INFEASIBLE = 2,                   /**< No feasible solution exists for given constraints. */
    MPC_STATUS_ERROR = 3                         /**< Solver encountered an error. */

} MpcSolverStatus_t;

/*===========================================================================
 * MPC Solver Result
 *===========================================================================
 * Complete output from the MPC solver.
 */

typedef struct
{
    MpcSolverStatus_t solver_status;            /**< Solver termination status. */
    ControlInput_t optimal_control;             /**< Optimal control input for current time step. */
    uint16_t iterations_used;                   /**< Number of solver iterations used. */
    float final_cost;                           /**< Final cost function value. */
    float dual_residual;                        /**< Final dual residual (ADMM convergence metric). */

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

/** Vehicle wheelbase */
#define F110_DEFAULT_WHEELBASE_METERS 0.324f

/** Maximum steering angle */
#define F110_DEFAULT_MAXIMUM_STEERING_RADIANS 0.4189f

/** Maximum forward velocity */
#define F110_DEFAULT_MAXIMUM_VELOCITY_METERS_PER_SECOND 20.0f

/** Minimum forward velocity */
#define F110_DEFAULT_MINIMUM_VELOCITY_METERS_PER_SECOND 0.0f

/** Distance from center of gravity to front axle */
#define VEHICLE_CG_TO_FRONT_AXLE_M 0.166f

/** Distance from center of gravity to rear axle */
#define VEHICLE_CG_TO_REAR_AXLE_M 0.16f

/** Vehicle mass */
#define F110_VEHICLE_MASS_KG 3.314f

/** Yaw moment of inertia */
#define F110_YAW_INERTIA_KGM2 0.035f

/** Center of gravity height */
#define F110_CG_HEIGHT_METERS 0.0703f

/** Tire-road friction coefficient */
#define F110_FRICTION_COEFFICIENT 0.745f

/** Gravity acceleration */
#define F110_GRAVITY_ACCELERATION_MS2 9.81f

/** Maximum longitudinal acceleration */
#define F110_DEFAULT_MAXIMUM_ACCELERATION_METERS_PER_SECOND2 (F110_FRICTION_COEFFICIENT * F110_GRAVITY_ACCELERATION_MS2)

/** Minimum longitudinal acceleration */
#define F110_DEFAULT_MINIMUM_ACCELERATION_METERS_PER_SECOND2 (-F110_DEFAULT_MAXIMUM_ACCELERATION_METERS_PER_SECOND2)

/** Cornoring stiffness for front wheel */
#define VP_C_ALPHA_F 51.40f

/** Cornoring stiffness for rear wheel */
#define VP_C_ALPHA_R 43.10f

/** Normal force on front wheel */
#define VP_NORM_LOAD_F (F110_VEHICLE_MASS_KG * F110_GRAVITY_ACCELERATION_MS2 * VEHICLE_CG_TO_REAR_AXLE_M / F110_DEFAULT_WHEELBASE_METERS)

/** Normal force on rear wheel */
#define VP_NORM_LOAD_R (F110_VEHICLE_MASS_KG * F110_GRAVITY_ACCELERATION_MS2 * VEHICLE_CG_TO_FRONT_AXLE_M / F110_DEFAULT_WHEELBASE_METERS)

/** Peak force on front wheel D */
#define VP_D_FRONT (F110_FRICTION_COEFFICIENT * VP_NORM_LOAD_F)

/** Peak force on rear wheel D */
#define VP_D_REAR (F110_FRICTION_COEFFICIENT * VP_NORM_LOAD_R)

/** Pacejka Magic Formula shape factor C, controlling the transition between
 *  the linear and saturation regions of the tire force curve. */
#define VP_C_SHAPE 1.9f

/** Minimum scale factor applied to tire cornering stiffness at low longitudinal
 *  velocities to prevent division-by-zero singularities in slip angle computation. */
#define MIN_STIFF_SCALE 0.1f

/** Front cornering stiffness [1/rad] */
#define F110_FRONT_CORNERING_STIFFNESS (VP_C_ALPHA_F / VP_D_FRONT)

/** Rear cornering stiffness [1/rad] */
#define F110_REAR_CORNERING_STIFFNESS (VP_C_ALPHA_R / VP_D_REAR)



/*=========================================================================== 
 * Internal Constants
 *===========================================================================*/

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

/*===========================================================================
 * Default MPC Configuration
 *===========================================================================*/

/** Default prediction horizon */
#define MPC_PREDICTION_HORIZON 10

/** Default prediction time step */
#define MPC_TIME_STEP_SECONDS MPC_PREDICTION_DT_SECONDS

/** Default maximum solver iterations */
#define MPC_MAXIMUM_ITERATIONS 20

/** Maximum number of waypoints in loaded trajectory */
#define TRAJECTORY_MAXIMUM_WAYPOINTS 1000

/** Maximum reference velocity */
#define TRAJECTORY_MAXIMUM_VELOCITY 20.0f

/** Default convergence tolerance */
#define MPC_CONVERGENCE_TOLERANCE MPC_CONVERGENCE_TOLERANCE_DEFAULT

/**
 * @brief Build an MpcConfiguration_t populated with F1/10th tuned defaults.
 * @details Default weights are overridden by environment variables at runtime
 *          (e.g. MPC_W_LAT_ERROR, MPC_W_HEADING). Declared here so alternative
 *          MPC implementations can reuse the same default configuration.
 * @return MpcConfiguration_t with default weights, horizon, and solver settings.
 */
MpcConfiguration_t get_default_configuration(void);

#endif /* MPC_TYPES_H */
