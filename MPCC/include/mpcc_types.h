/**
 * @file mpcc_types.h
 * @brief Type Definitions for Model Predictive Contouring Control
 *        — Lifted ODE Formulation (Frenet + Cartesian)
 *
 * Based on: Reiter et al., "Frenet-Cartesian Model Representations for
 *           Automotive Obstacle Avoidance within Nonlinear MPC" (2023).
 *
 * Lifted ODE state (10):
 *   Frenet primary (7):  [s, n, alpha, vx, vy, omega, omega_w]
 *   Cartesian redundant (3): [X, Y, psi]
 *
 *   s       — arc-length position on reference path [m]
 *   n       — lateral deviation from path [m] (+ = left)
 *   alpha   — heading error: vehicle heading - path tangent [rad]
 *   vx      — longitudinal velocity (body frame) [m/s]
 *   vy      — lateral velocity (body frame) [m/s]
 *   omega   — yaw rate [rad/s]
 *   omega_w — wheel angular velocity [rad/s]
 *
 * Controls (2): [delta, T_motor]
 *   delta   — front wheel steering angle [rad]
 *   T_motor — motor torque [N*m]
 *
 * Key advantages of Lifted ODE over standard MPCC:
 *   1. Track bounds are trivial box constraints on n
 *   3. Virtual progress control v_theta: ds/dt = v_theta decouples path
 *   4. Progress maximisation: reward ds/dt or terminal s_N
 *
 * Frenet dynamics:
 *   ds/dt      = (vx*cos(alpha) - vy*sin(alpha)) / (1 - n*kappa(s))
 *   dn/dt      = vx*sin(alpha) + vy*cos(alpha)
 *   dalpha/dt  = omega - kappa(s) * ds/dt
 *   dvx/dt     = (F_x - F_yf*sin(delta) + m*vy*omega) / m
 *   dvy/dt     = (F_yf*cos(delta) + F_yr - m*vx*omega) / m
 *   domega/dt  = (l_f*F_yf*cos(delta) - l_r*F_yr) / I_z
 *   domega_w/dt= (T_motor/G_ratio - F_x*R_w) / I_w
 *
 * Cost:
 *   J = Sum_k [ q_n*n^2 + q_alpha*alpha^2 + q_vy*vy^2 + q_omega*omega^2
 *             - q_s * ds/dt_k                         progress reward
 *             + u^T R u  +  du^T Rd du ]              control
 *       + terminal terms
 *
 * All numerical values use Q16.16 fixed-point for FPGA compatibility.
 * Units: SI (meters, radians, seconds, Newtons).
 */

#ifndef MPCC_TYPES_H
#define MPCC_TYPES_H

#include "fp_math_mpcc.h"
#include <stdint.h>

/*===========================================================================
 * Convenience float <-> fixed-point conversion
 *===========================================================================*/

/** Convert a float to Q16.16 fixed-point at runtime */
static inline fixed_point_t float_to_fp(float f)
{
    return (fixed_point_t)(f * (float)FP_ONE);
}

/** Convert Q16.16 fixed-point to float at runtime */
static inline float fp_to_float(fixed_point_t x)
{
    return (float)x / (float)FP_ONE;
}

/* MPCC is self-contained — no dependency on MPC package.
 * VehicleState_t and F110 constants are defined locally below. */

/*===========================================================================
 * Vehicle State (for input conversion from ROS/simulator)
 *===========================================================================*/

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
 * F1/10th Default Vehicle Parameters
 *===========================================================================*/

/** F1/10th max steering: 0.4189 radians (~24.0 degrees) */
#define F110_DEFAULT_MAXIMUM_STEERING_RADIANS    FP_CONST(0.4189)

/** F1/10th max velocity: 20.0 meters per second */
#define F110_DEFAULT_MAXIMUM_VELOCITY_METERS_PER_SECOND  FP_CONST(20.0)

/** F1/10th minimum velocity: 0 m/s (no reverse) */
#define F110_DEFAULT_MINIMUM_VELOCITY_METERS_PER_SECOND  FP_CONST(0.0)

/** Distance from CG to front axle: 0.166 meters [CAD] */
#define F110_DIST_CG_TO_FRONT_AXLE_METERS    FP_CONST(0.166)

/** Distance from CG to rear axle: 0.16 meters [CAD] */
#define F110_DIST_CG_TO_REAR_AXLE_METERS     FP_CONST(0.16)

/** Vehicle mass: 3.314 kg [MEASURED] */
#define F110_VEHICLE_MASS_KG                 FP_CONST(3.314)

/** Yaw moment of inertia: 0.035 kg·m² [CAD] */
#define F110_YAW_INERTIA_KGM2                FP_CONST(0.035)

/** Center of gravity height: 0.0703 meters [CAD] */
#define F110_CG_HEIGHT_METERS                FP_CONST(0.0703)

/** Gravity acceleration: 9.81 m/s² */
#define F110_GRAVITY_ACCELERATION_MS2        FP_CONST(9.81)

/** Tire-road friction coefficient [TESTED] */
#define F110_FRICTION_COEFFICIENT            FP_CONST(0.745)

/** Normalized front cornering stiffness [1/rad] */
#define F110_NORMALIZED_CORNERING_STIFFNESS_FRONT  FP_CONST(4.297)

/** Normalized rear cornering stiffness [1/rad] */
#define F110_NORMALIZED_CORNERING_STIFFNESS_REAR   FP_CONST(3.473)

/*===========================================================================
 * MPCC Problem Dimensions
 *===========================================================================*/

/** Number of Lifted ODE states:
 *  Frenet (6): [s, n, alpha, vx, vy, omega]
 *  Cartesian (3): [X, Y, psi] (redundant, for obstacle avoidance)
 *  Total: 10 states (but using 9 for now: s,n,alpha,vx,vy,omega + X,Y,psi = 9)
 *  NOTE: We use 9 states since omega_w (wheel angular velocity) is omitted. */
#define MPCC_NX 9

/** Number of controls: [delta, a_x, v_theta] */
#define MPCC_NU 3

/** Number of Frenet states (first 6 of MPCC state) */
#define MPCC_N_FRENET 6


/** State index constants for readability */
#define MPCC_IDX_S       0   /**< arc-length progress */
#define MPCC_IDX_N       1   /**< lateral deviation */
#define MPCC_IDX_ALPHA   2   /**< heading error */
#define MPCC_IDX_VX      3   /**< longitudinal velocity */
#define MPCC_IDX_VY      4   /**< lateral velocity */
#define MPCC_IDX_OMEGA   5   /**< yaw rate */
#define MPCC_IDX_X       6   /**< global X position (Cartesian) */
#define MPCC_IDX_Y       7   /**< global Y position (Cartesian) */
#define MPCC_IDX_PSI     8   /**< global heading (Cartesian) */

/** Control index constants */
#define MPCC_IDX_DELTA   0   /**< steering angle */
#define MPCC_IDX_AX      1   /**< longitudinal acceleration */
#define MPCC_IDX_VTHETA  2   /**< virtual progress speed ds/dt */

/** Maximum prediction horizon steps.*/
#define MPCC_MAX_HORIZON 20

/** Maximum number of reference path waypoints.*/

#define MPCC_MAX_PATH_POINTS 500

/** Maximum number of obstacles that can be tracked simultaneously */
#define MPCC_MAX_OBSTACLES 10

/*===========================================================================
 * MPCC ODE State
 *===========================================================================*/

typedef struct
{
    /*--- Frenet states ---*/

    /** Arc-length position along reference path [m].
     *  Evolves from Frenet kinematics: ds/dt = v_proj / (1 - n*kappa(s)).
     *  NOT a virtual variable — tied to actual vehicle position. */
    fixed_point_t s;

    /** Lateral deviation from reference path [m].
     *  Positive = left of path.  Track bounds: n_left <= n <= n_right. */
    fixed_point_t n;

    /** Heading error: vehicle heading - path tangent angle [rad].
     *  alpha = psi - phi_gamma(s).  Small when well-aligned. */
    fixed_point_t alpha;

    /** Longitudinal velocity in body frame [m/s] */
    fixed_point_t vx;

    /** Lateral velocity in body frame [m/s] */
    fixed_point_t vy;

    /** Yaw rate [rad/s] */
    fixed_point_t omega;

    /*--- Cartesian states (redundant, for obstacle avoidance) ---*/

    /** Global X position [m].
     *  Redundant — coupled to (s, n) via inverse Frenet transform:
     *    X = gamma_x(s) - n*sin(phi_gamma(s))
     *  Propagated by ODE: dX/dt = vx*cos(psi) - vy*sin(psi) */
    fixed_point_t X;

    /** Global Y position [m].
     *  Y = gamma_y(s) + n*cos(phi_gamma(s)) */
    fixed_point_t Y;

    /** Global heading angle [rad].
     *  psi = phi_gamma(s) + alpha.  Propagated by ODE: dpsi/dt = omega */
    fixed_point_t psi;

} MPCCState_t;

/*===========================================================================
 * MPCC Control
 *===========================================================================*/
/**
 * Vehicle controls augmented with virtual progress input.
 * The optimizer jointly determines physical controls (delta, a_x)
 * and the virtual progress rate v_theta = ds/dt.
 */

typedef struct
{
    /** Front wheel steering angle [rad] */
    fixed_point_t delta;

    /** Longitudinal acceleration [m/s^2] */
    fixed_point_t a_x;

    /** Virtual progress speed [m/s] — controls ds/dt along path */
    fixed_point_t v_theta;

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
    fixed_point_t x_ref;

    /** Reference Y position [m] (in world frame) */
    fixed_point_t y_ref;

    /** Reference tangent angle [rad].
     *  phi_gamma(s) = direction the path is heading at this point. */
    fixed_point_t phi_ref;

    /** Reference curvature [1/m].
     *  kappa(s) = dphi/ds.  Positive = turning left.
     *  Critical for Frenet dynamics (appears in ds/dt and dalpha/dt). */
    fixed_point_t kappa_ref;

    /** Arc-length parameter at this waypoint [m].
     *  Monotonically increasing along the path. */
    fixed_point_t s_ref;

    /** Reference longitudinal velocity at this waypoint [m/s].
     *  From the raceline optimiser's velocity profile.
     *  Used as per-stage vx_ref in the QP cost. */
    fixed_point_t vx_ref;

    /** Maximum leftward deviation from centerline [m] (positive).
     *  Track constraint: n <= left_bound */
    fixed_point_t left_bound;

    /** Maximum rightward deviation from centerline [m] (positive).
     *  Track constraint: n >= -right_bound */
    fixed_point_t right_bound;

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
    fixed_point_t total_length;

    /** Whether the path forms a closed loop (for s wrapping) */
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
    fixed_point_t cx;

    /** Obstacle center Y [m] (global frame) */
    fixed_point_t cy;

    /** Semi-axis along primary direction [m] (includes ego radius) */
    fixed_point_t a;

    /** Semi-axis along secondary direction [m] (includes ego radius) */
    fixed_point_t b;

    /** Orientation of the ellipse [rad] (angle of primary axis) */
    fixed_point_t phi;

    /** Precomputed inverse shape matrix Sigma_inv (symmetric 2x2):
     *    Sigma_inv[0][0] = cos^2(phi)/a^2 + sin^2(phi)/b^2
     *    Sigma_inv[0][1] = Sigma_inv[1][0] = (1/a^2 - 1/b^2)*sin(phi)*cos(phi)
     *    Sigma_inv[1][1] = sin^2(phi)/a^2 + cos^2(phi)/b^2  */
    fixed_point_t Sigma_inv[2][2];

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
 * All tunable parameters for the Lifted ODE MPCC controller.
 *
 * Weight tuning guide (Lifted ODE):
 *   - weight_n:        Penalizes lateral deviation (path following)
 *   - weight_alpha:    Penalizes heading error (alignment)
 *   - weight_progress: Rewards ds/dt (forward speed along path)
 *   - weight_obstacle: Penalty for violating obstacle constraints
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
    fixed_point_t dt;

    /*--- Linear tire model parameters ---*/

    /** Road-tire friction coefficient [-] */
    fixed_point_t mu;

    /** Front normalized cornering stiffness [1/rad] (dimensionless) */
    fixed_point_t C_Sf;

    /** Rear normalized cornering stiffness [1/rad] (dimensionless) */
    fixed_point_t C_Sr;

    /*--- Frenet tracking weights ---*/

    /** Lateral deviation weight (q_n).
     *  Penalizes n^2 — keeps car on the racing line. */
    fixed_point_t weight_n;

    /** Heading error weight (q_alpha).
     *  Penalizes alpha^2 — keeps car aligned with path tangent. */
    fixed_point_t weight_alpha;

    /** Progress weight (q_s).
     *  Reward for forward progress ds/dt. Higher = more aggressive.
     *  Applied as linear cost: -q_s on s-component. */
    fixed_point_t weight_progress;

    /*--- State regularization weights ---*/

    /** Longitudinal velocity tracking weight.
     *  Optional: penalizes (vx - v_ref)^2. Set to 0 for pure progress. */
    fixed_point_t weight_vx;

    /** Velocity reference for vx tracking [m/s].
     *  Only used if weight_vx > 0. */
    fixed_point_t vx_ref;

    /** Lateral velocity penalty (anti-drift) */
    fixed_point_t weight_vy;

    /** Yaw rate penalty */
    fixed_point_t weight_omega;

    /*--- Control effort weights (R matrix diagonal) ---*/

    /** Steering angle effort penalty */
    fixed_point_t weight_delta;

    /** Longitudinal acceleration effort penalty */
    fixed_point_t weight_ax;

    /** Virtual progress speed effort penalty.
     *  Regularizes v_theta to prevent large spikes. */
    fixed_point_t weight_v_theta;

    /*--- Control rate weights (Rd matrix, smoothness) ---*/

    /** Steering rate penalty (d_delta = delta_k - delta_{k-1}) */
    fixed_point_t weight_delta_rate;

    /** Acceleration rate penalty */
    fixed_point_t weight_ax_rate;

    /** Virtual progress speed rate penalty */
    fixed_point_t weight_v_theta_rate;

    /*--- Terminal cost weights (stage N) ---*/

    /** Terminal lateral deviation penalty */
    fixed_point_t weight_n_terminal;

    /** Terminal heading error penalty */
    fixed_point_t weight_alpha_terminal;

    /** Terminal progress reward (on s_N) */
    fixed_point_t weight_progress_terminal;

    /*--- Obstacle avoidance ---*/

    /** Obstacle constraint violation penalty (soft constraint).
     *  Used as a relaxation weight in the ADMM projection. */
    fixed_point_t weight_obstacle;

    /** Safety margin added to obstacle ellipses [m] */
    fixed_point_t obstacle_margin;

    /*--- ADMM solver parameters ---*/

    /** ADMM penalty parameter (rho) */
    fixed_point_t admm_rho;

    /** Maximum ADMM iterations */
    uint16_t admm_max_iterations;

    /** Convergence tolerance for primal and dual residuals */
    fixed_point_t admm_tolerance;

    /*--- Constraint bounds ---*/

    /** Maximum steering angle [rad] (symmetric: +/- delta_max) */
    fixed_point_t delta_max;

    /** Maximum longitudinal acceleration [m/s^2] */
    fixed_point_t ax_max;

    /** Minimum longitudinal acceleration (braking) [m/s^2] (negative) */
    fixed_point_t ax_min;

    /** Maximum longitudinal velocity [m/s] */
    fixed_point_t vx_max;

    /** Minimum longitudinal velocity [m/s] */
    fixed_point_t vx_min;

    /** Maximum lateral deviation [m].
     *  Default track half-width if per-stage bounds not set. */
    fixed_point_t n_max;

    /** Maximum virtual progress speed [m/s] */
    fixed_point_t v_theta_max;

    /** Minimum virtual progress speed [m/s] (typically 0) */
    fixed_point_t v_theta_min;

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
    /** Solver termination status */
    MPCCStatus_t status;

    /** Optimal control input for current time step.
     *  Apply optimal_control.delta and .T_motor to the vehicle. */
    MPCCControl_t optimal_control;

    /** Number of ADMM iterations used */
    uint16_t admm_iterations;

    /** Final ADMM primal residual ||z - w|| */
    fixed_point_t primal_residual;

    /** Final ADMM dual residual rho * ||w_new - w_old|| */
    fixed_point_t dual_residual;

    /** Final cost function value */
    fixed_point_t cost;

    /** Predicted state trajectory (length = horizon + 1).
     *  predicted_states[0] = current state.
     *  Contains both Frenet and Cartesian views for visualization. */
    MPCCState_t predicted_states[MPCC_MAX_HORIZON + 1];

    /** Predicted control sequence (length = horizon) */
    MPCCControl_t predicted_controls[MPCC_MAX_HORIZON];

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
    fixed_point_t A[MPCC_NX][MPCC_NX];

    /** Discrete input matrix (NX x NU = 10x2) */
    fixed_point_t B[MPCC_NX][MPCC_NU];

    /** Affine term / linearization residual (NX = 10) */
    fixed_point_t d[MPCC_NX];

} MPCCLinearSystem_t;

/*===========================================================================
 * Stage Cost Matrices (per prediction stage, after linearization)
 *===========================================================================
 * Quadratic cost at each stage:
 *   l_k(z, u) = 0.5 z^T Q_k z + q_k^T z + 0.5 u^T R_k u + r_k^T u
 *
 * Q_k incorporates Frenet penalties (on n, alpha) + state regularization.
 * R_k incorporates control effort + control rate penalties.
 * q_k contains the linear progress reward (-q_s on s-component).
 * S_k is the cross term (usually zero).
 */

typedef struct
{
    /** Quadratic state cost (NX x NX = 10x10). Symmetric PSD. */
    fixed_point_t Q[MPCC_NX][MPCC_NX];

    /** Quadratic control cost (NU x NU = 2x2). Symmetric PD. */
    fixed_point_t R[MPCC_NU][MPCC_NU];

    /** Cross term (NU x NX = 2x10). Usually zero. */
    fixed_point_t S[MPCC_NU][MPCC_NX];

    /** Linear state cost (NX = 10).
     *  q[MPCC_IDX_S] = -weight_progress (reward forward progress). */
    fixed_point_t q[MPCC_NX];

    /** Linear control cost (NU = 2) */
    fixed_point_t r[MPCC_NU];

} MPCCStageCost_t;

/*===========================================================================
 * Default MPCC Parameters
 *===========================================================================
 * Pre-computed fixed-point constants for default MPCC configuration.
 * Tuned for F1/10th autonomous racing at moderate speeds (~3-5 m/s).
 */

/*--- Horizon (tuned on aligned dynamics sweep) ---*/
#define MPCC_DEFAULT_HORIZON          10
#define MPCC_DEFAULT_DT               FP_CONST(0.0425)

/*--- Frenet tracking weights ---*/

/** Lateral deviation penalty (aligned-dynamics sweep best). */
#define MPCC_DEFAULT_WEIGHT_N         FP_CONST(50.0)

/** Heading error: penalizes deviation from path tangent angle. */
#define MPCC_DEFAULT_WEIGHT_ALPHA     FP_CONST(20.0)

/** Progress reward (reduced for stability on aligned dynamics). */
#define MPCC_DEFAULT_WEIGHT_PROGRESS  FP_CONST(2.0)

/*--- State regularization ---*/
#define MPCC_DEFAULT_WEIGHT_VX        FP_CONST(0.0)
#define MPCC_DEFAULT_VX_REF           FP_CONST(12.0)
#define MPCC_DEFAULT_WEIGHT_VY        FP_CONST(10.0)
#define MPCC_DEFAULT_WEIGHT_OMEGA     FP_CONST(0.1)

/*--- Control effort ---*/
#define MPCC_DEFAULT_WEIGHT_DELTA     FP_CONST(0.01)
#define MPCC_DEFAULT_WEIGHT_AX        FP_CONST(0.01)
#define MPCC_DEFAULT_WEIGHT_V_THETA   FP_CONST(0.5)

/*--- Control rate (smoothness) ---*/
#define MPCC_DEFAULT_WEIGHT_DELTA_RATE    FP_CONST(0.1)
#define MPCC_DEFAULT_WEIGHT_AX_RATE       FP_CONST(0.1)
#define MPCC_DEFAULT_WEIGHT_V_THETA_RATE  FP_CONST(0.1)

/*--- Terminal weights ---*/
#define MPCC_DEFAULT_WEIGHT_N_TERMINAL       FP_CONST(100.0)
#define MPCC_DEFAULT_WEIGHT_ALPHA_TERMINAL   FP_CONST(10.0)
#define MPCC_DEFAULT_WEIGHT_PROGRESS_TERMINAL FP_CONST(5.0)

/*--- Obstacle avoidance ---*/
#define MPCC_DEFAULT_WEIGHT_OBSTACLE  FP_CONST(1000.0)
#define MPCC_DEFAULT_OBSTACLE_MARGIN  FP_CONST(0.1)

/*--- ADMM solver (tuned via sweep) ---*/
#define MPCC_DEFAULT_ADMM_RHO         FP_CONST(1.218171)
#define MPCC_DEFAULT_ADMM_MAX_ITER    100
#define MPCC_DEFAULT_ADMM_TOLERANCE   FP_CONST(0.014462)

/*--- Track half-width (default if per-stage not set) ---*/
#define MPCC_DEFAULT_N_MAX            FP_CONST(0.5)

/*--- Linear tire model ---*/
/** Friction coefficient [-] — from test_friction.py */
#define MPCC_DEFAULT_MU               F110_FRICTION_COEFFICIENT
/** Front normalized cornering stiffness [1/rad] — typical F1/10th */
#define MPCC_DEFAULT_C_SF             F110_NORMALIZED_CORNERING_STIFFNESS_FRONT
/** Rear normalized cornering stiffness [1/rad] — typical F1/10th */
#define MPCC_DEFAULT_C_SR             F110_NORMALIZED_CORNERING_STIFFNESS_REAR

/*--- Acceleration bounds ---*/
#define MPCC_DEFAULT_AX_MAX           FP_CONST(7.0)
#define MPCC_DEFAULT_AX_MIN           FP_CONST(-10.0)

/*--- Virtual progress speed bounds (aligned-dynamics sweep) ---*/
#define MPCC_DEFAULT_V_THETA_MAX      FP_CONST(2.0)
#define MPCC_DEFAULT_V_THETA_MIN      FP_CONST(0.0)

#endif /* MPCC_TYPES_H */
