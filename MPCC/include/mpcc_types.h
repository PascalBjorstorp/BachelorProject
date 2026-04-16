/**
 * @file mpcc_types.h
 * @brief Type Definitions for Model Predictive Contouring Control
 *
 * 
 *   Global frame primary (7):  [s, vx, vy, omega, X, Y, psi]
 *
 *   s       — arc-length position on reference path [m]
 *   vx      — longitudinal velocity  [m/s]
 *   vy      — lateral velocity   [m/s]
 *   omega   — yaw rate [rad/s]
 *   X       — global X position  [m]
 *   Y       — global Y position  [m]
 *   psi     — heading angle (yaw) [rad]
 *
 * Controls (3): [delta, acceleration, virtual_progress]
 *   delta            — front wheel steering angle [rad]
 *   acceleration     — longitudinal acceleration  [m/s²]
 *   virtual_progress — virtual progress control [m/s]
 *
 * System dynamics:
 *   ds/dt      = (vx*cos(alpha) - vy*sin(alpha)) / (1 - n*kappa(s))
 *   dvx/dt     = (F_x - F_yf*sin(delta) + m*vy*omega) / m
 *   dvy/dt     = (F_yf*cos(delta) + F_yr - m*vx*omega) / m
 *   dX/dt      = vx*cos(psi) - vy*sin(psi)
 *   dY/dt      = vx*sin(psi) + vy*cos(psi)
 *   dpsi/dt    = omega
 *   domega/dt  = (l_f*F_yf*cos(delta) - l_r*F_yr) / I_z
 *   
 *
 * Cost:
 *   J = Sum_k [ q_ec*ec^2 + q_el*el^2 + q_vy*vy^2 + q_omega*omega^2
 *             - q_progress * v_theta_k                progress reward
 *             + u^T R u  +  du^T Rd du ]              control
 *             + terminal terms
 *
 * Units: SI (meters, radians, seconds, Newtons).
 */

#ifndef MPCC_TYPES_H
#define MPCC_TYPES_H

#include <math.h>
#include <float.h>
#include <stdint.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*===========================================================================
 * Vehicle State (for input conversion from ROS/simulator)
 *===========================================================================*/

typedef struct
{
    float pos_x;        /** X position in world frame [meters] */
    float pos_y;        /** Y position in world frame [meters] */
    float heading;      /** Yaw angle (heading) relative to world X-axis [radians] */
    float long_vel;     /** Longitudinal velocity in body frame [meters per second] */
    float lat_vel;      /** Lateral velocity in body frame [meters per second] */
    float yaw_rate;     /** Yaw rate [radians per second] */

} VehicleState_t;


/*===========================================================================
 * F1/10th Default Vehicle Parameters
 *===========================================================================*/

#define F110_DEFAULT_MAXIMUM_STEERING_RADIANS    0.4189f           /** F1/10th max steering: 0.4189 radians (~24.0 degrees) */
#define F110_DEFAULT_MAXIMUM_VELOCITY_METERS_PER_SECOND  20.0f     /** F1/10th max velocity: 20.0 meters per second */
#define F110_DEFAULT_MINIMUM_VELOCITY_METERS_PER_SECOND  0.0f      /** F1/10th min velocity: 0.0 meters per second */
#define F110_DIST_CG_TO_FRONT_AXLE_METERS    0.166f                /** Distance from CG to front axle: 0.166 meters [CAD] */
#define F110_DIST_CG_TO_REAR_AXLE_METERS     0.16f                /** Distance from CG to rear axle: 0.16 meters [CAD] */
#define F110_VEHICLE_MASS_KG                 3.314f               /** Vehicle mass: 3.314 kg [MEASURED] */
#define F110_YAW_INERTIA_KGM2                0.035f               /** Yaw moment of inertia: 0.035 kg·m² [CAD] */
#define F110_CG_HEIGHT_METERS                0.0703f              /** Center of gravity height: 0.0703 meters [CAD] */
#define F110_GRAVITY_ACCELERATION_MS2        9.81f                /** Gravity acceleration: 9.81 m/s² */
#define F110_FRICTION_COEFFICIENT            0.745f              /** Tire-road friction coefficient [TESTED] */
#define F110_NORMALIZED_CORNERING_STIFFNESS_FRONT  4.297f        /** Normalized front cornering stiffness [1/rad] */
#define F110_NORMALIZED_CORNERING_STIFFNESS_REAR   3.473f        /** Normalized rear cornering stiffness [1/rad] */

/*===========================================================================
 * MPCC Problem Dimensions
 *===========================================================================*/


#define MPCC_NX 7                   /** State space: [s, vx, vy, omega, X, Y, psi] */


#define MPCC_NU 3                   /** Number of controls: [delta, a_x, v_theta] */


/** State index constants for readability */
#define MPCC_IDX_S       0          /**< arc-length progress */
#define MPCC_IDX_VX      1          /**< longitudinal velocity */
#define MPCC_IDX_VY      2          /**< lateral velocity */
#define MPCC_IDX_OMEGA   3          /**< yaw rate */
#define MPCC_IDX_X       4          /**< global X position */
#define MPCC_IDX_Y       5          /**< global Y position */
#define MPCC_IDX_PSI     6          /**< global heading */

/** Control index constants */
#define MPCC_IDX_DELTA   0          /**< steering angle */
#define MPCC_IDX_AX      1          /**< longitudinal acceleration */
#define MPCC_IDX_VTHETA  2          /**< virtual progress speed ds/dt */


#define MPCC_MAX_HORIZON 20         /** Maximum prediction horizon steps.*/
#define MPCC_MAX_PATH_POINTS 500    /** Maximum number of reference path waypoints.*/
#define MPCC_MAX_OBSTACLES 10       /** Maximum number of obstacles that can be tracked simultaneously */

/*===========================================================================
 * MPCC ODE State
 *===========================================================================*/

typedef struct
{
    float s;        /** Virtual variable  */
    float vx;       /** Longitudinal velocity in body frame [m/s] */
    float vy;       /** Lateral velocity in body frame [m/s] */
    float omega;    /** Yaw rate [rad/s] */
    float X;        /** Global X position [m]. */
    float Y;        /** Global Y position [m].*/
    float psi;      /** Global heading angle [rad].*/

} MPCCState_t;

/*===========================================================================
 * MPCC Control
 *===========================================================================*/
/**
 * Vehicle controls augmented with virtual progress input [delta, a_x, v_theta].
 * Virtual progress rate v_theta = ds/dt.
 */

typedef struct
{
    float delta;        /** Front wheel steering angle [rad] */
    float a_x;          /** Longitudinal acceleration [m/s^2] */
    float v_theta;      /** Virtual progress speed [m/s] — controls ds/dt along path */

} MPCCControl_t;

/*===========================================================================
 * Reference Path Point
 *===========================================================================
 * A single waypoint on the arc-length parameterized reference path
 * (racing line or centerline).
 *
 * The MPCC uses kappa(s) for the Frenet dynamics and (x_ref, y_ref, phi_ref)
 * for computing the Frenet-Cartesian coupling.
 * Track bounds define the drivable corridor width at this point.
 */

typedef struct
{
    /** Reference X position [m] (in world frame) */
    float x_ref;

    /** Reference Y position [m] (in world frame) */
    float y_ref;

    /** Reference tangent angle [rad].
     *  phi_gamma(s) = direction the path is heading at this point. */
    float phi_ref;

    /** Reference curvature [1/m].
     *  kappa(s) = dphi/ds.  Positive = turning left.
     *  Critical for Frenet dynamics (appears in ds/dt and dalpha/dt). */
    float kappa_ref;

    /** Arc-length parameter at this waypoint [m].
     *  Monotonically increasing along the path. */
    float s_ref;

    /** Reference longitudinal velocity at this waypoint [m/s].
     *  From the raceline optimiser's velocity profile.
     *  Used as per-stage vx_ref in the QP cost. */
    float vx_ref;

    /** Maximum leftward deviation from centerline [m] (positive).
     *  Track constraint: n <= left_bound */
    float left_bound;

    /** Maximum rightward deviation from centerline [m] (positive).
     *  Track constraint: n >= -right_bound */
    float right_bound;

} MPCCPathPoint_t;

/*===========================================================================
 * Reference Path (Complete)
 *===========================================================================
 * Arc-length parameterized reference path.
 * Points must be ordered by increasing s_ref.
 */

typedef struct
{
    /** Array of path waypoints, ordered by arc-length */
    MPCCPathPoint_t points[MPCC_MAX_PATH_POINTS];

    /** Number of valid waypoints */
    uint16_t num_points;

    /** Total arc length of the path [m] */
    float total_length;

    /** Whether the path forms a closed loop */
    uint8_t is_closed;

} MPCCReferencePath_t;

/*===========================================================================
 * Obstacle (Ellipsoidal Representation)
 *===========================================================================
 * Each obstacle is modelled as an ellipse in the global (X, Y) frame.
 *
 * Ellipsoidal constraint (convex in Cartesian):
 *   (p - c)^T Sigma_inv (p - c) >= 1
 * where:
 *   p = [X, Y] (ego vehicle position from Cartesian states)
 *   c = [cx, cy] (obstacle center)
 *   Sigma = R(phi) * diag(a^2, b^2) * R(phi)^T  (oriented ellipse)
 *   a, b = semi-axes (already enlarged by ego radius)
 *
 * For SQP/ADMM linearization, this becomes a half-plane per iteration:
 *   grad_h(p_bar)^T * (p - p_bar) + h(p_bar) >= 0
 *
 * Based on Reiter et al.: ellipsoidal formulation is the fastest
 * and best-performing obstacle representation for NMPC.
 */

typedef struct
{
    /** Obstacle center X [m] (global frame) */
    float cx;

    /** Obstacle center Y [m] (global frame) */
    float cy;

    /** Semi-axis along primary direction [m] (includes ego radius) */
    float a;

    /** Semi-axis along secondary direction [m] (includes ego radius) */
    float b;

    /** Orientation of the ellipse [rad] (angle of primary axis) */
    float phi;

    /** Precomputed inverse shape matrix Sigma_inv (symmetric 2x2):
     *    Sigma_inv[0][0] = cos^2(phi)/a^2 + sin^2(phi)/b^2
     *    Sigma_inv[0][1] = Sigma_inv[1][0] = (1/a^2 - 1/b^2)*sin(phi)*cos(phi)
     *    Sigma_inv[1][1] = sin^2(phi)/a^2 + cos^2(phi)/b^2  */
    float Sigma_inv[2][2];

    /** Whether this obstacle slot is active */
    uint8_t active;

} MPCCObstacle_t;

/*===========================================================================
 * Obstacle Set
 *===========================================================================*/

typedef struct
{
    MPCCObstacle_t obstacles[MPCC_MAX_OBSTACLES];
    uint8_t num_active;
} MPCCObstacleSet_t;

/*===========================================================================
 * MPCC Configuration
 *===========================================================================
 * All tunable parameters for the Global Frame (7-state) MPCC controller.
 *
 * Weight tuning guide:
 *   - weight_contouring: Penalizes Cartesian contouring error ec
 *   - weight_lag:        Penalizes Cartesian lag error el
 *   - weight_progress:   Rewards ds/dt (forward progress along path)
 *   - weight_obstacle:   Penalty for violating obstacle constraints
 *   - Control rate weights: Higher = smoother, slower response
 *
 * ADMM tuning:
 *   - admm_rho: Penalty parameter. Too low -> slow convergence.
 *     Too high -> poor conditioning. Typical: 0.1 to 10.0
 *   - admm_max_iterations: 20-100 typical for real-time
 */

typedef struct
{
    /*--- Prediction horizon ---*/

    /** Number of prediction steps */
    uint16_t horizon_steps;

    /** Time step between prediction stages [s] */
    float dt;

    /*--- Linear tire model parameters ---*/

    /** Road-tire friction coefficient [-] */
    float mu;

    /** Front normalized cornering stiffness [1/rad] (dimensionless) */
    float C_Sf;

    /** Rear normalized cornering stiffness [1/rad] (dimensionless) */
    float C_Sr;

    /*--- Frenet tracking weights ---*/


    /** Contouring error weight (q_c).
     *  Penalizes e_c^2, the true perpendicular error from path at
     *  the virtual arc parameter s, computed in Cartesian:
     *    e_c = sin(phi(s))*(X - gamma_x(s)) - cos(phi(s))*(Y - gamma_y(s))
     *  Set to 0 to use the Frenet n approximation instead. */
    float weight_contouring;

    /** Lag error weight (q_l).
     *  Penalizes e_l^2, the tangential distance between the vehicle
     *  and the reference point at virtual parameter s:
     *    e_l = -cos(phi(s))*(X - gamma_x(s)) - sin(phi(s))*(Y - gamma_y(s))
     *  Keeps s from running too far ahead of the vehicle. */
    float weight_lag;

    /** Progress weight (q_s).
     *  Reward for forward progress ds/dt. Higher = more aggressive.
     *  Applied as linear cost: -q_s on s-component. */
    float weight_progress;

    /*--- State regularization weights ---*/

    /** Longitudinal velocity tracking weight.
     *  Optional: penalizes (vx - v_ref)^2. Set to 0 for pure progress. */
    float weight_vx;

    /** Velocity reference for vx tracking [m/s].
     *  Only used if weight_vx > 0. */
    float vx_ref;

    /** Lateral velocity penalty (anti-drift) */
    float weight_vy;

    /** Yaw rate penalty */
    float weight_omega;

    /*--- Control effort weights (R matrix diagonal) ---*/

    /** Steering angle effort penalty */
    float weight_delta;

    /** Longitudinal acceleration effort penalty */
    float weight_ax;

    /** Virtual progress speed effort penalty.
     *  Regularizes v_theta to prevent large spikes. */
    float weight_v_theta;

    /*--- Control rate weights (Rd matrix, smoothness) ---*/

    /** Steering rate penalty (d_delta = delta_k - delta_{k-1}) */
    float weight_delta_rate;

    /** Acceleration rate penalty */
    float weight_ax_rate;

    /** Virtual progress speed rate penalty */
    float weight_v_theta_rate;

    /*--- Terminal cost weights (stage N) ---*/

    /** Terminal contouring error penalty */
    float weight_contouring_terminal;

    /** Terminal lag error penalty */
    float weight_lag_terminal;


    /** Terminal progress reward (on s_N) */
    float weight_progress_terminal;

    /*--- Obstacle avoidance ---*/

    /** Obstacle constraint violation penalty (soft constraint).
     *  Used as a relaxation weight in the ADMM projection. */
    float weight_obstacle;

    /** Safety margin added to obstacle ellipses [m] */
    float obstacle_margin;

    /*--- ADMM solver parameters ---*/

    /** ADMM penalty parameter (rho) */
    float admm_rho;

    /** Maximum ADMM iterations */
    uint16_t admm_max_iterations;

    /** Convergence tolerance for primal and dual residuals */
    float admm_tolerance;

    /*--- Constraint bounds ---*/

    /** Maximum steering angle [rad] (symmetric: +/- delta_max) */
    float delta_max;

    /** Maximum longitudinal acceleration [m/s^2] */
    float ax_max;

    /** Minimum longitudinal acceleration (braking) [m/s^2] (negative) */
    float ax_min;

    /** Maximum longitudinal velocity [m/s] */
    float vx_max;

    /** Minimum longitudinal velocity [m/s] */
    float vx_min;

    /** Maximum virtual progress speed [m/s] */
    float v_theta_max;

    /** Minimum virtual progress speed [m/s] (typically 0) */
    float v_theta_min;

    /*--- Cross-call rate scaling ---*/

    /** Scale factor for rate penalties at step 0.
     *  Compensates for the mismatch between the control callback rate
     *  (~200 Hz) and the prediction dt (e.g. 35 ms).  Without this,
     *  rate penalties on the first prediction step are ~7x too strong
     *  because the real inter-call interval is much shorter than dt.
     *  Value: control_dt / prediction_dt  (e.g. 0.005/0.035 ≈ 0.143). */
    float cross_call_rate_scale;

} MPCCConfiguration_t;

/*===========================================================================
 * MPCC Solver Status
 *===========================================================================*/

typedef enum
{
    /** ADMM converged: primal and dual residuals below tolerance */
    MPCC_STATUS_SUCCESS = 0,

    /** Maximum ADMM iterations reached (solution may be usable) */
    MPCC_STATUS_MAX_ITERATIONS = 1,

    /** Problem is infeasible (constraints cannot be satisfied) */
    MPCC_STATUS_INFEASIBLE = 2,

    /** Solver encountered an error */
    MPCC_STATUS_ERROR = 3

} MPCCStatus_t;

/*===========================================================================
 * MPCC Solver Result
 *===========================================================================
 * Complete output from the MPCC controller.
 */

typedef struct
{
    MPCCStatus_t status;                /** Solver termination status */
    MPCCControl_t optimal_control;      /** Optimal control input for current time step.
                                         *  Apply optimal_control.delta and acceleration */
    uint16_t admm_iterations;           /** Number of ADMM iterations used */
    float primal_residual;              /** Final ADMM primal residual ||z - w|| */
    float dual_residual;                /** Final ADMM dual residual rho * ||w_new - w_old|| */
    float rho_final;                    /** Final adapted ADMM rho values used by this solve */
    float rho_u_final;                  /** Final adapted ADMM rho_u values used by this solve */
    uint16_t adaptive_rho_updates;      /** Number of adaptive rho updates during solve */
    uint32_t numeric_clip_count;        /** Number of numeric clipping events during solve */    
    float cost;                         /** Final cost function value */

   
    MPCCState_t predicted_states[MPCC_MAX_HORIZON + 1];     /** Predicted state trajectory (length = horizon + 1).
                                                             *  predicted_states[0] = current state.
                                                             *  Contains Cartesian views for visualization. */
    MPCCControl_t predicted_controls[MPCC_MAX_HORIZON];     /** Predicted control sequence (length = horizon) */

} MPCCResult_t;

/*===========================================================================
 * Linearized System Matrices (per prediction stage)
 *===========================================================================
 * Discrete-time linearization of the Lifted ODE:
 *
 *   z_{k+1} = A_k z_k + B_k u_k + d_k
 *
 * Where z = [s, n, alpha, vx, vy, omega, omega_w, X, Y, psi] (10 states)
 * and   u = [delta, T_motor] (2 controls)
 *
 * The A matrix has block structure:
 *   A = [ A_frenet(7x7)    0(7x3)       ]
 *       [ A_coupling(3x7)  A_cart(3x3)   ]
 *
 * Note: The Frenet dynamics depend on kappa(s), which couples s to
 * all Frenet states. The Cartesian ODE depends on vx, vy, omega, psi
 * but is independent of s, n, alpha in the state-space ODE form.
 */

typedef struct
{
    /** Discrete state transition matrix (NX x NX = 10x10) */
    float A[MPCC_NX][MPCC_NX];

    /** Discrete input matrix (NX x NU = 10x2) */
    float B[MPCC_NX][MPCC_NU];

    /** Affine term / linearization residual (NX = 10) */
    float d[MPCC_NX];

} MPCCLinearSystem_t;

/*===========================================================================
 * Stage Cost Matrices (per prediction stage, after linearization)
 *===========================================================================
 * Quadratic cost at each stage:
 *   l_k(z, u) = 0.5 z^T Q_k z + q_k^T z + 0.5 u^T R_k u + r_k^T u
 *
 * Q_k incorporates state regularization.
 * R_k incorporates control effort + control rate penalties.
 * q_k contains the linear progress reward (-q_s on s-component).
 * S_k is the cross term.
 */

typedef struct
{
    float Q[MPCC_NX][MPCC_NX];      /** Quadratic state cost (NX x NX = 7x7). Symmetric PSD. */
    float R[MPCC_NU][MPCC_NU];      /** Quadratic control cost (NU x NU = 3x3). Symmetric PD. */
    float S[MPCC_NU][MPCC_NX];      /** Cross term (NU x NX = 3x7). */
    float q[MPCC_NX];               /** Linear state cost (NX = 7).
                                     *  q[MPCC_IDX_S] = -weight_progress. */
    float r[MPCC_NU];               /** Linear control cost (NU = 3) */

} MPCCStageCost_t;

/*===========================================================================
 * Default MPCC Parameters
 *===========================================================================*/

/*--- Horizon (tuned via iterative sweep, Hardware mode) ---*/
#define MPCC_DEFAULT_HORIZON          10                            /** Horizon look ahead. */
#define MPCC_DEFAULT_DT               (0.02f)                       /** Time step between prediction stages [s]. */

/*--- Contouring tracking weights ---*/
#define MPCC_DEFAULT_WEIGHT_CONTOURING (1400.0f)                    /** Contouring error penalty. */
#define MPCC_DEFAULT_WEIGHT_LAG       (300.0f)                    /** Lag error penalty. */
#define MPCC_DEFAULT_WEIGHT_PROGRESS  (80.0f)                     /** Progress reward. */

/*--- State regularization ---*/
#define MPCC_DEFAULT_WEIGHT_VX        (1.5f)                        /** Longitudinal velocity tracking weight.*/
#define MPCC_DEFAULT_VX_REF           (4.0f)                        /** Reference velocity for longitudinal velocity tracking [m/s]. */
#define MPCC_DEFAULT_WEIGHT_VY        (0.4725f)                     /** Lateral velocity tracking weight. */
#define MPCC_DEFAULT_WEIGHT_OMEGA     (0.45f)                       /** Yaw rate tracking weight. */

/*--- Control effort ---*/
#define MPCC_DEFAULT_WEIGHT_DELTA     (20.0f)                       /** Steering angle effort penalty. */
#define MPCC_DEFAULT_WEIGHT_AX        (0.0495f)                     /** Longitudinal acceleration effort penalty. */
#define MPCC_DEFAULT_WEIGHT_V_THETA   (0.05f)                       /** Virtual progress speed effort penalty. */

/*--- Control rate (smoothness) ---*/
#define MPCC_DEFAULT_WEIGHT_DELTA_RATE    (0.345f)                    /** Steering rate penalty. */
#define MPCC_DEFAULT_WEIGHT_AX_RATE       (0.61f)                     /** Longitudinal acceleration rate penalty. */
#define MPCC_DEFAULT_WEIGHT_V_THETA_RATE  (0.156f)                    /** Virtual progress speed rate penalty. */

/*--- Cross-call rate scaling (control_dt / prediction_dt) ---*/
#define MPCC_CONTROL_RATE_HZ              (200.0f)                   /** Control callback rate in Hz. Used to compute cross-call rate scaling. */
#define MPCC_DEFAULT_CROSS_CALL_SCALE     (0.3f)                     /** Scale factor for control rate penalties at step 0. */

/*--- Terminal weights ---*/
#define MPCC_DEFAULT_WEIGHT_CONTOURING_TERMINAL     (4896.0f)     /** Terminal contouring error penalty. */
#define MPCC_DEFAULT_WEIGHT_LAG_TERMINAL            (800.0f)      /** Terminal lag error penalty. */
#define MPCC_DEFAULT_WEIGHT_PROGRESS_TERMINAL       (80.0f)      /** Terminal progress reward. */

/*--- Obstacle avoidance ---*/
#define MPCC_DEFAULT_WEIGHT_OBSTACLE  (1000.0f)                     /** Obstacle avoidance penalty. */
#define MPCC_DEFAULT_OBSTACLE_MARGIN  (0.1f)                        /** Minimum distance to obstacles [m]. */

/*--- ADMM solver (tuned via iterative sweep) ---*/
#define MPCC_DEFAULT_ADMM_RHO         (0.3f)                        /** ADMM penalty parameter. */
#define MPCC_DEFAULT_ADMM_MAX_ITER    300                           /** Maximum ADMM iterations. */
#define MPCC_DEFAULT_ADMM_TOLERANCE   (0.005f)                      /** ADMM convergence tolerance. */

/*--- Pacejka tire model ---*/
#define MPCC_DEFAULT_MU       F110_FRICTION_COEFFICIENT                  /** Friction coefficient for tire model. */
#define MPCC_DEFAULT_C_SF     F110_NORMALIZED_CORNERING_STIFFNESS_FRONT  /** Front normalized cornering stiffness [1/rad] */
#define MPCC_DEFAULT_C_SR     F110_NORMALIZED_CORNERING_STIFFNESS_REAR   /** Rear normalized cornering stiffness [1/rad] */

/*--- Acceleration bounds ---*/
#define MPCC_DEFAULT_AX_MAX           (8.0f)                        /** Maximum Acceleration bound */
#define MPCC_DEFAULT_AX_MIN           (-10.0f)                      /** Minimum Acceleration bound (negative = breaking)*/

/*--- Virtual progress speed bounds ---*/
#define MPCC_DEFAULT_V_THETA_MAX      (8.0f)                        /** Maximum virtual progress bound */
#define MPCC_DEFAULT_V_THETA_MIN      (0.0f)                        /** Minimum virtual progress bound */

#endif /* MPCC_TYPES_H */
