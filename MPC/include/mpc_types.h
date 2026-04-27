/**
 * @file mpc_types.h
 * @brief Type definitions and compile-time constants for the MPC system.
 * @details Defines all shared constants and data structures used by the MPC
 *          pipeline, including vehicle state, control inputs, physical
 *          parameters, solver configuration, trajectory references, and
 *          solver outputs.
 * @dependencies util_math.h, <stdint.h>
 */

#ifndef MPC_TYPES_H
#define MPC_TYPES_H

#include "util_math.h"
#include <stdint.h>

/*===========================================================================
 * Defines
 *===========================================================================*/

/* Global dimensions */
#define NX_GLOBAL 6                                      /* Global/body model state size used by nonlinear prediction and linearization. */
#define NX_FRENET 5                                      /* Frenet state size used by linearized MPC dynamics. */
#define NX_AUG 8                                         /* Augmented state size including steering state and prior controls. */
#define IDX_DELTA_ACTUAL 5                               /* Position of physical steering state in the augmented vector. */
#define IDX_DRATE_PREV 6                                 /* Position of previous steering-rate state in the augmented vector. */
#define IDX_ACCEL_PREV 7                                 /* Position of previous acceleration state in the augmented vector. */
#define IDX_SPARSE_B_FIRST_ROW 2                         /* First augmented-state row with non-zero dense B coupling in Riccati sparse products. */
#define NX_DENSE 6                                       /* Dense A-block width before sparse previous-control tail states. */
#define NU 2                                             /* Control vector width for steering-rate and longitudinal acceleration. */
#define RICCATI_MAX_NX  8                                /* Maximum Riccati state dimension (augmented Frenet model). */
#define RICCATI_MAX_NU  2                                /* Maximum Riccati control dimension (steering-rate and acceleration). */

/* Math and timing */
#define TWO_PI (2.0 * M_PI)                              /* Full-angle constant used for heading wrap operations. */
#define CONTROL_RATE_HZ 200.0f                           /* Controller update frequency used by cross-call scaling logic. */
#define CONTROL_DT_SECONDS (1.0f / CONTROL_RATE_HZ)      /* Controller sample period derived from control-rate definition. */
#define PREDICTION_DT_SECONDS 0.03f                     /* Nominal prediction-step duration used for model rollout defaults. */
#define CROSS_CALL_RATE_SCALE (CONTROL_DT_SECONDS / PREDICTION_DT_SECONDS) /* Normalizes first-step rate penalties across sample times. */

/* Default MPC objective weights */
#define WEIGHT_LAT_ERROR 8000.0f                        /* Penalizes lateral tracking deviation from the reference path. */
#define WEIGHT_HEADING 28.8f                           /* Penalizes heading misalignment relative to path tangent. */
#define WEIGHT_VELOCITY 15.0f                         /* Penalizes deviation from target longitudinal speed profile. */
#define WEIGHT_LAT_VEL 1.04f                             /* Penalizes lateral velocity to suppress side-slip growth. */
#define WEIGHT_YAW_RATE 1.5f                           /* Penalizes yaw-rate mismatch against reference curvature dynamics. */
#define WEIGHT_STEER_EFFORT 1.5f                       /* Penalizes steering-rate effort to limit aggressive steering actuation. */
#define WEIGHT_ACCEL_EFFORT 0.01f                      /* Penalizes longitudinal acceleration effort to smooth throttle/brake usage. */
#define WEIGHT_STEER_RATE 0.04f                         /* Penalizes steering-rate change to reduce steering jerk. */
#define WEIGHT_ACCEL_RATE 0.1f                          /* Penalizes acceleration change to reduce longitudinal jerk. */
#define WEIGHT_DELTA_ACTUAL 0.023f                        /* Penalizes steering-state bias away from curvature feedforward. */

/* Other swept MPC defaults */
#define MAX_ITERATIONS 100                               /* Default solver iteration budget per control update. */
#define WALL_MARGIN 0.14f                                 /* Safety offset subtracted from both wall boundaries. */
#define ADMM_RHO 16.0f                                   /* Primary ADMM penalty balancing feasibility and optimality progress. */
#define ADMM_RHO_U 24.0f                                 /* ADMM penalty applied to control-variable projection terms. */
#define CONVERGENCE_TOLERANCE 0.05f                      /* Residual threshold used to declare solver convergence. */
#ifndef PREDICTION_HORIZON
#define PREDICTION_HORIZON 20                            /* Default maximum number of prediction stages used by the controller. */
#endif
#define TIME_STEP_SECONDS 0.03f                         /* Default model integration period per horizon stage. */

/* Solver and model safeguards */
#define RICCATI_COST_FACTOR 2.0f                         /* Global scaling factor applied to stage and terminal costs. */
#define EMA_ALPHA 1.0f                                   /* Smoothing factor for optional Frenet-state exponential filtering. */
#define STEERING_RATE_LIMIT 2.849f                       /* Hard bound on steering-rate command in optimization. */
#define STEERING_FEEDFORWARD_CLAMP_FACTOR 0.5f           /* Limits feedforward steering around linearization operating point. */
#define BIG_BOUND 50.0f                                  /* Sentinel magnitude representing an effectively unconstrained bound. */
#define MIN_LINEARIZATION_VELOCITY 0.5f                  /* Lower velocity clamp aligned with slip-angle floor for low-speed recovery. */
#define STABILITY_LIMIT 0.95f                            /* Clamp on selected discrete self-coupling to preserve numerical stability. */
#define V_SWITCH 7.319f                                  /* Transition speed for constant-power acceleration limiting. */
#define MIN_SLIP_VELOCITY 0.5f                           /* Lower velocity clamp used in slip-angle calculations. */
#define WARMSTART_CURVATURE_RESET_THRESHOLD 0.5f         /* Curvature jump threshold that resets warm-start state history. */

/* Default MPC configuration values */
#define TRAJECTORY_MAXIMUM_WAYPOINTS 4000                /* Maximum trajectory samples accepted by MPC reference buffers. */
#define TRAJECTORY_MAXIMUM_VELOCITY 20.0f                /* Cap on reference velocity accepted from trajectory input. */
#define MIN_TRAJECTORY_SPEED_MPS 1.0f                    /* Lower bound used when trajectory speed is missing or too small. */

/* Hardware/sim calibration defaults */
#define TRAJECTORY_SPEED_GAIN 1.0f                       /* Global multiplier applied to reference speed profile. */
#define STEERING_TO_SERVO_GAIN -0.7284f                  /* Linear gain mapping steering angle to servo command domain. */
#define STEERING_TO_SERVO_OFFSET 0.55f                   /* Bias term for steering-angle to servo-command conversion. */
#define STEERING_CORRECTION_C2 0.589566f                 /* Quadratic term in empirical steering-command correction model. */
#define STEERING_CORRECTION_C1 0.918061f                 /* Linear term in empirical steering-command correction model. */
#define STEERING_CORRECTION_C0 0.001490f                 /* Constant term in empirical steering-command correction model. */

/* Vehicle parameter defaults and derived constants */
#define VP_MAX_STEERING_RAD 0.4189f                      /* Steering-angle saturation used by controller and model limits. */
#define VP_MAX_VELOCITY_MPS 20.0f                        /* Upper velocity limit used by vehicle model clamping logic. */
#define VP_MIN_VELOCITY_MPS 1.5f                         /* Lower velocity limit used by vehicle model clamping logic. */
#define VP_CG_TO_FRONT_AXLE_M 0.166f                     /* Center-of-gravity distance from front axle for load transfer. */
#define VP_CG_TO_REAR_AXLE_M 0.16f                       /* Center-of-gravity distance from rear axle for load transfer. */
#define VP_WHEELBASE_M (VP_CG_TO_FRONT_AXLE_M + VP_CG_TO_REAR_AXLE_M) /* Distance between front and rear axle centers. */
#define VP_MASS_KG 3.314f                                /* Vehicle mass used in dynamic equations and load calculations. */
#define VP_YAW_INERTIA_KGM2 0.035f                       /* Yaw inertia governing rotational response to tire moments. */
#define VP_CG_HEIGHT_M 0.0703f                           /* Center-of-gravity height driving longitudinal load transfer. */
#define VP_FRICTION_COEFF 0.72f                         /* Effective tire-road friction coefficient for force limits. */
#define GRAVITY_MPS2 9.81f                               /* Gravitational acceleration constant used in vehicle load equations. */
#define VP_MASS_TIMES_GRAVITY_N (VP_MASS_KG * GRAVITY_MPS2) /* Vehicle weight magnitude used by normal-load equations. */
#define VP_INV_MASS_1_PER_KG (1.0f / VP_MASS_KG)         /* Reciprocal vehicle mass used in acceleration-state Jacobians. */
#define VP_INV_YAW_INERTIA_1_PER_KGM2 (1.0f / VP_YAW_INERTIA_KGM2) /* Reciprocal yaw inertia used in yaw-rate Jacobians. */
#define VP_MAX_ACCEL_MPS2 (0.745f * GRAVITY_MPS2) /* Friction-limited forward acceleration derived from tire-road capacity. */
#define VP_MIN_ACCEL_MPS2 (-VP_MAX_ACCEL_MPS2)           /* Braking bound mirrored from the friction-limited acceleration envelope. */
#define VP_C_ALPHA_F 51.40f                              /* Front tire lateral-force scale used by the nonlinear tire model. */
#define VP_C_ALPHA_R 43.10f                              /* Rear tire lateral-force scale used by the nonlinear tire model. */
#define VP_NORM_LOAD_F (VP_MASS_TIMES_GRAVITY_N * VP_CG_TO_REAR_AXLE_M / VP_WHEELBASE_M) /* Front normal load from static axle-load distribution. */
#define VP_NORM_LOAD_R (VP_MASS_TIMES_GRAVITY_N * VP_CG_TO_FRONT_AXLE_M / VP_WHEELBASE_M) /* Rear normal load from static axle-load distribution. */
#define VP_D_FRONT (VP_FRICTION_COEFF * VP_NORM_LOAD_F) /* Front lateral force scale from friction and normal-load estimate. */
#define VP_D_REAR (VP_FRICTION_COEFF * VP_NORM_LOAD_R)  /* Rear lateral force scale from friction and normal-load estimate. */
#define VP_C_SHAPE 1.9f                                  /* Tire-model shape factor governing force saturation transition. */
#define VP_INV_C_SHAPE (1.0f / VP_C_SHAPE)               /* Reciprocal tire shape factor for local-slope calculations. */
#define MIN_STIFF_SCALE 0.1f                             /* Lower bound on stiffness scaling to avoid low-speed singularities. */
#define VP_FRONT_CORNERING_STIFFNESS (VP_C_ALPHA_F / VP_D_FRONT) /* Front normalized cornering stiffness used in slip-force mapping. */
#define VP_REAR_CORNERING_STIFFNESS (VP_C_ALPHA_R / VP_D_REAR)   /* Rear normalized cornering stiffness used in slip-force mapping. */

/*===========================================================================
 * Structs
 *===========================================================================*/

/**
 * @brief Vehicle state in world/body coordinates.
 * @details Holds the global pose and body-frame velocities used by nonlinear
 *          propagation, simulation updates, and conversion to Frenet errors.
 * @param pos_x X position in world frame [meters].
 * @param pos_y Y position in world frame [meters].
 * @param heading Yaw angle relative to world X-axis [radians].
 * @param long_vel Longitudinal velocity in body frame [meters per second].
 * @param lat_vel Lateral velocity in body frame [meters per second].
 * @param yaw_rate Yaw rate [radians per second].
 */
typedef struct
{
    float pos_x;                /* X position in world frame [meters]. */
    float pos_y;                /* Y position in world frame [meters]. */
    float heading;              /* Yaw angle relative to world X-axis [radians]. */
    float long_vel;             /* Longitudinal velocity in body frame [meters per second]. */
    float lat_vel;              /* Lateral velocity in body frame [meters per second]. */
    float yaw_rate;             /* Yaw rate [radians per second]. */
} VehicleState_t;

/**
 * @brief Frenet-frame state relative to the active reference path.
 * @details Stores path-relative tracking errors and body-frame dynamics in the
 *          coordinate system consumed by the MPC optimizer.
 * @param flat_error Lateral displacement from path centerline [meters].
 * @param fhead_error Heading error relative to path tangent [radians].
 * @param flong_vel Longitudinal velocity in vehicle body frame [meters per second].
 * @param flat_vel Lateral velocity in vehicle body frame [meters per second].
 * @param fyaw_rate Vehicle yaw rate [radians per second].
 */
typedef struct
{
    float flat_error;           /* Lateral error [meters]. */
    float fhead_error;          /* Heading error [radians]. */
    float flong_vel;            /* Longitudinal velocity [meters per second]. */
    float flat_vel;             /* Lateral velocity [meters per second]. */
    float fyaw_rate;            /* Yaw rate [radians per second]. */
} FrenetState_t;

/**
 * @brief Control command issued to the vehicle model/controller.
 * @details Represents the actuator-level command pair used throughout MPC
 *          rollouts and solver outputs.
 * @param steer_ang Front wheel steering angle command [radians].
 * @param long_acc Longitudinal acceleration command [m/s^2].
 */
typedef struct
{
    float steer_ang;            /* Front wheel steering angle [radians]. */
    float long_acc;             /* Longitudinal acceleration command [m/s^2]. */
} ControlInput_t;

/**
 * @brief One raceline waypoint sample used by ROS nodes when loading CSV tracks.
 * @details Stores geometric, kinematic, and corridor-bound data in double
 *          precision for interpolation and closest-point queries.
 * @param s_meters Arc length along the reference path [meters].
 * @param x_meters Global X position in map/world frame [meters].
 * @param y_meters Global Y position in map/world frame [meters].
 * @param heading_radians Path tangent heading at this waypoint [radians].
 * @param velocity_meters_per_second Reference speed at this waypoint [m/s].
 * @param curvature_radians_per_meter Path curvature at this waypoint [rad/m].
 * @param left_bound_meters Left corridor bound from centerline [meters].
 * @param right_bound_meters Right corridor bound from centerline [meters].
 * @param sin_heading Cached sin(heading_radians) for fast projections.
 * @param cos_heading Cached cos(heading_radians) for fast projections.
 */
typedef struct
{
    double s_meters;
    double x_meters;
    double y_meters;
    double heading_radians;
    double velocity_meters_per_second;
    double curvature_radians_per_meter;
    double left_bound_meters;
    double right_bound_meters;
    double sin_heading;
    double cos_heading;
} TrajectoryWaypoint_t;

/**
 * @brief Shared slip-angle intermediate terms for bicycle tire dynamics.
 * @details Caches safe longitudinal velocity terms, slip numerators/ratios,
 *          and resulting front/rear slip angles so prediction and
 *          linearization paths reuse the same heavy trigonometric state.
 * @param vx_safe Longitudinal velocity with low-speed floor [meters per second].
 * @param inv_vx_safe Reciprocal of vx_safe [seconds per meter].
 * @param front_num Front slip numerator (v_y + l_f*omega) [meters per second].
 * @param rear_num Rear slip numerator (v_y - l_r*omega) [meters per second].
 * @param front_ratio Front slip ratio front_num / vx_safe [dimensionless].
 * @param rear_ratio Rear slip ratio rear_num / vx_safe [dimensionless].
 * @param alpha_front Front slip angle [radians].
 * @param alpha_rear Rear slip angle [radians].
 */
typedef struct
{
    float vx_safe;
    float inv_vx_safe;
    float front_num;
    float rear_num;
    float front_ratio;
    float rear_ratio;
    float alpha_front;
    float alpha_rear;
} SlipTerms_t;

/**
  @brief Physical vehicle parameters used by the model and controller.
  @details This structure holds all physical parameters of the vehicle
  @param wheelbase_meters Distance between front and rear axle centers [meters].
  @param distance_cg_to_front_axle Distance from center of gravity to front axle [meters].
  @param distance_cg_to_rear_axle Distance from center of gravity to rear axle [meters].
  @param height_cg_to_ground Height from center of gravity to ground [meters].
  @param vehicle_mass Vehicle mass used in dynamic equations and load calculations [kg].
  @param yaw_moment_of_inertia Yaw moment of inertia governing rotational response to tire moments [kg*m^2].
  @param front_cornering_stiffness Front tire cornering stiffness [1/rad].
  @param rear_cornering_stiffness Rear tire cornering stiffness [1/rad].
  @param max_steering_angle Maximum steering angle magnitude [radians].
  @param max_velocity Maximum forward velocity [meters per second].
  @param min_velocity Minimum velocity [meters per second].
  @param max_acceleration Maximum longitudinal acceleration [m/s^2].
  @param min_acceleration Minimum longitudinal acceleration (braking) [m/s^2].
*/
typedef struct
{
    float wheelbase_meters;              /* Wheelbase [meters]. */
    float distance_cg_to_front_axle;     /* Distance from center of gravity to front axle [meters]. */
    float distance_cg_to_rear_axle;      /* Distance from center of gravity to rear axle [meters]. */
    float height_cg_to_ground;           /* Height from center of gravity to ground [meters]. */
    float vehicle_mass;                  /* Vehicle mass [kg]. */
    float yaw_moment_of_inertia;         /* Yaw moment of inertia [kg*m^2]. */
    float front_cornering_stiffness;     /* Front tire cornering stiffness [1/rad]. */
    float rear_cornering_stiffness;      /* Rear tire cornering stiffness [1/rad]. */
    float max_steering_angle;            /* Maximum steering angle magnitude [radians]. */
    float max_velocity;                  /* Maximum forward velocity [meters per second]. */
    float min_velocity;                  /* Minimum velocity [meters per second]. */
    float max_acceleration;              /* Maximum longitudinal acceleration [m/s^2]. */
    float min_acceleration;              /* Minimum longitudinal acceleration (braking) [m/s^2]. */
} VehicleParameters_t;

/**
 * @brief MPC solver configuration and objective policy parameters.
 * @details Collects horizon timing, objective weights, and solver stopping
 *          settings that define one complete MPC tuning profile.
 * @param prediction_horizon_steps Number of prediction stages in the horizon.
 * @param time_step Prediction stage duration [seconds].
 * @param weight_lateral_error Weight on lateral tracking error.
 * @param weight_heading_error Weight on heading tracking error.
 * @param weight_velocity Weight on longitudinal velocity tracking error.
 * @param weight_lateral_velocity Weight on lateral velocity tracking error.
 * @param weight_yaw_rate Weight on yaw-rate tracking error.
 * @param weight_steering_effort Weight on steering effort command magnitude.
 * @param weight_acceleration_effort Weight on acceleration effort command magnitude.
 * @param weight_steering_rate Weight on steering-rate change.
 * @param weight_acceleration_rate Weight on acceleration-rate change.
 * @param weight_delta_actual Weight on steering-state centering term.
 * @param cross_call_rate_scale Scale factor for first-step cross-call penalties.
 * @param wall_margin Effective lateral wall margin [meters] used in e_y constraints.
 * @param max_solver_iterations Maximum QP/ADMM iterations per solve.
 * @param solver_convergence_tolerance Residual tolerance for solve termination.
 */
typedef struct
{
    uint16_t prediction_horizon_steps;   /* Prediction horizon in time steps. */
    float time_step;                     /* Prediction step duration [seconds]. */
    float weight_lateral_error;          /* Weight for lateral error tracking. */
    float weight_heading_error;          /* Weight for heading error tracking. */
    float weight_velocity;               /* Weight for longitudinal velocity tracking error. */
    float weight_lateral_velocity;       /* Weight for lateral velocity tracking error. */
    float weight_yaw_rate;               /* Weight for yaw-rate tracking error. */
    float weight_steering_effort;        /* Weight for steering effort. */
    float weight_acceleration_effort;    /* Weight for acceleration effort. */
    float weight_steering_rate;          /* Weight for steering jerk/rate change. */
    float weight_acceleration_rate;      /* Weight for acceleration rate change. */
    float weight_delta_actual;           /* Weight for steering centering state. */
    float cross_call_rate_scale;         /* First-step cross-call rate penalty scale factor. */
    float wall_margin;                   /* Effective wall margin for lateral corridor bounds [m]. */

    uint16_t max_solver_iterations;      /* Maximum QP solver iterations. */
    float solver_convergence_tolerance;  /* Solver convergence tolerance. */
} MpcConfiguration_t;

/**
 * @brief One trajectory reference sample for a single horizon stage.
 * @details Defines the tracking targets and corridor bounds consumed by each
 *          MPC prediction step.
 * @param reference_lateral_error Target lateral error [meters].
 * @param reference_heading_error Target heading error [radians].
 * @param reference_velocity Target longitudinal velocity [meters per second].
 * @param reference_lateral_velocity Target lateral velocity [meters per second].
 * @param reference_yaw_rate Target yaw rate [radians per second].
 * @param path_curvature Reference path curvature [radians per meter].
 * @param left_wall_bound Maximum leftward allowed offset from centerline [meters].
 * @param right_wall_bound Maximum rightward allowed offset from centerline [meters].
 */
typedef struct
{
    float reference_lateral_error;       /* Reference lateral error [meters]. */
    float reference_heading_error;       /* Reference heading error [radians]. */
    float reference_velocity;            /* Target longitudinal velocity [meters per second]. */
    float reference_lateral_velocity;    /* Target lateral velocity [meters per second]. */
    float reference_yaw_rate;            /* Target yaw rate [radians per second]. */
    float path_curvature;                /* Path curvature [radians per meter]. */
    float left_wall_bound;               /* Maximum leftward bound from path centerline [meters]. */
    float right_wall_bound;              /* Maximum rightward bound from path centerline [meters]. */
} TrajectoryReferencePoint_t;

/** 
 * @brief Enumeration of possible MPC solver termination statuses.
 * @details Used to communicate solver outcomes to higher-level logic for
 * @param MPC_STATUS_SUCCESS Optimal solution found.
 * @param MPC_STATUS_MAXIMUM_ITERATIONS_REACHED Solver reached max iterations without convergence.
 * @param MPC_STATUS_INFEASIBLE No feasible solution exists for the given constraints.
*/
typedef enum
{
    MPC_STATUS_SUCCESS = 0,                    /* Optimal solution found. */
    MPC_STATUS_MAXIMUM_ITERATIONS_REACHED = 1, /* Solver reached max iterations. */
    MPC_STATUS_INFEASIBLE = 2,                 /* No feasible solution for constraints. */
    MPC_STATUS_ERROR = 3                       /* Solver error or invalid input. */
} MpcSolverStatus_t;

/**
 * @brief MPC solve output for one control cycle.
 * @details Captures solver termination metadata and the first control action
 *          applied to the plant for the current control tick.
 * @param solver_status Solver termination state.
 * @param optimal_control First-step optimal control command.
 * @param iterations_used Number of iterations consumed by the solver.
 * @param final_cost Final primal residual metric reported by the solver.
 * @param dual_residual Final dual residual from ADMM convergence checks.
 */
typedef struct
{
    MpcSolverStatus_t solver_status;  /* Solver termination status. */
    ControlInput_t optimal_control;   /* First-step optimal control command. */
    uint16_t iterations_used;         /* Number of solver iterations used. */
    float final_cost;                 /* Final solver primal residual metric. */
    float dual_residual;              /* Final dual residual (ADMM metric). */
} MpcSolverResult_t;


/*===========================================================================
 * Riccati-ADMM Solver Types
 *===========================================================================*/

/**
 * @brief Per-stage dynamics, cost, and box-constraint data for Riccati-ADMM.
 * @details Represents one prediction step used by the constrained LQR solve.
 *          Q and R are diagonal weights, N is an optional cross-cost term,
 *          and x/u bounds define box constraints for ADMM projection.
 */
typedef struct
{
    float A[RICCATI_MAX_NX][RICCATI_MAX_NX];
    float B[RICCATI_MAX_NX][RICCATI_MAX_NU];
    float d[NX_AUG]; 
    float Q_diag[RICCATI_MAX_NX];
    float q[RICCATI_MAX_NX];
    float R_diag[RICCATI_MAX_NU];
    float r[RICCATI_MAX_NU];
    float N[RICCATI_MAX_NX][RICCATI_MAX_NU];
    float x_lb[RICCATI_MAX_NX];
    float x_ub[RICCATI_MAX_NX];
    float u_lb[RICCATI_MAX_NU];
    float u_ub[RICCATI_MAX_NU];
} RiccatiStepData_t;

/**
 * @brief ADMM configuration parameters for the Riccati solver.
 * @details Controls augmented-Lagrangian penalties, convergence criteria,
 *          iteration budget, and optional adaptive-rho logic.
 */
typedef struct
{
    float rho;
    float rho_u;
    float tolerance;
    int max_iterations;
    int adaptive_rho;
} RiccatiAdmmConfig_t;

/**
 * @brief Termination status codes returned by Riccati-ADMM.
 * @details Distinguishes converged solves, iteration-limit exits, and hard
 *          input/runtime errors.
 */
typedef enum
{
    RICCATI_STATUS_OPTIMAL = 0,
    RICCATI_STATUS_MAX_ITERATIONS = 1,
    RICCATI_STATUS_ERROR = 2
} RiccatiStatus_t;

/**
 * @brief Output trajectories and convergence metrics from Riccati-ADMM.
 * @details Stores the solved state/control horizon together with residuals,
 *          iteration count, and final status.
 */
typedef struct
{
    float x[PREDICTION_HORIZON + 1][RICCATI_MAX_NX];
    float u[PREDICTION_HORIZON][RICCATI_MAX_NU];
    int iterations;
    float primal_residual;
    float dual_residual;
    RiccatiStatus_t status;
} RiccatiSolution_t;

/**
 * @brief Persistent ADMM warm-start buffers used across solve calls.
 * @details Retains projected variables, dual variables, and adapted penalty
 *          parameters so sequential MPC solves can converge faster.
 */
typedef struct
{
    float z_x[PREDICTION_HORIZON + 1][RICCATI_MAX_NX];
    float z_u[PREDICTION_HORIZON][RICCATI_MAX_NU];
    float y_x[PREDICTION_HORIZON + 1][RICCATI_MAX_NX];
    float y_u[PREDICTION_HORIZON][RICCATI_MAX_NU];
    float rho;
    float rho_u;
    int initialized;
} RiccatiAdmmState_t;

#endif /* MPC_TYPES_H */
