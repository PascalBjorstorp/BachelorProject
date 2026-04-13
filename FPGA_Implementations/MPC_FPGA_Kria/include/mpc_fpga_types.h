/**
 * @file mpc_fpga_types.h
 * @brief Types, constants, and shared structures for the FPGA MPC solver.
 * @details Defines dimensions, state indices, precomputed fixed-point
 *          constants, and all data structures exchanged across the
 *          vehicle-model, Riccati solver, and top-level integration units.
 * @dependencies fp_math_hls.h, mpc_fpga_constants.h
 */

#ifndef MPC_FPGA_TYPES_H
#define MPC_FPGA_TYPES_H

#ifndef MPC_HLS_TARGET
/* Build flag used by downstream modules to enable HLS-specific branches. */
#define MPC_HLS_TARGET
#endif

#include "fp_math_hls.h"
#include "mpc_fpga_constants.h"
#ifdef MPC_RUNTIME_TUNE
#include "mpc_runtime_tune.h"
#endif

/*===========================================================================
 * MPC Dimension Constants
 *===========================================================================*/

/** Frenet state dimension: [e_y, e_psi, vx, vy, omega] */
#define MPC_NX_FRENET   5

/** Augmented state: [e_y, e_psi, vx, vy, omega, delta_actual, delta_rate_prev, accel_prev] */
#define MPC_NX_AUG      8

/** Dense block size in A matrix (Frenet + delta_actual) */
#define MPC_NX_DENSE    6

/** Control dimension: [delta_rate, acceleration] */
#define MPC_NU          2

/** Fixed prediction horizon */
#define MPC_HORIZON     MPC_FPGA_HORIZON_STEPS

/** Horizon length used for loops that include terminal state k = N */
#define MPC_HORIZON_PLUS_ONE (MPC_HORIZON + 1)

/** Maximum ADMM iterations */
#define MPC_MAX_ADMM_ITER MPC_FPGA_MAX_ADMM_ITER

/*===========================================================================
 * HLS Resource Constraints
 *===========================================================================*/

/** Default multiplier budget for non-hot blocks.
 *  Override at compile time: -DMPC_HLS_MUL_LIMIT=N */
#ifndef MPC_HLS_MUL_LIMIT
#define MPC_HLS_MUL_LIMIT 4
#endif

/** Multiplier budget for Riccati hot path.
 *  Higher default to reduce cycle count in matrix-heavy backward/forward passes.
 *  Override at compile time: -DMPC_HLS_RICCATI_MUL_LIMIT=N */
#ifndef MPC_HLS_RICCATI_MUL_LIMIT
#define MPC_HLS_RICCATI_MUL_LIMIT 4
#endif

/** Vehicle model multiplier budget.
 *  Override at compile time: -DMPC_HLS_VEHICLE_MUL_LIMIT=N */
#ifndef MPC_HLS_VEHICLE_MUL_LIMIT
#define MPC_HLS_VEHICLE_MUL_LIMIT 8
#endif

/*===========================================================================
 * Augmented State Indices
 *===========================================================================*/

#define IDX_EY                  0   /* Lateral error */
#define IDX_EPSI                1   /* Heading error */
#define IDX_VX                  2   /* Longitudinal velocity */
#define IDX_VY                  3   /* Lateral velocity */
#define IDX_OMEGA               4   /* Yaw rate */
#define IDX_DELTA_ACT           5   /* Actual servo steering angle */
#define IDX_DELTA_RATE_PREV     6   /* Previous steering rate (for jerk penalty) */
#define IDX_ACCEL_PREV          7   /* Previous acceleration (for accel rate) */

/*===========================================================================
 * Trajectory
 *===========================================================================*/

#define MAX_TRAJECTORY_SIZE     64

/*===========================================================================
 * F1/10th Vehicle Parameters (compile-time constants)
 * Values can be found in f1tenth_parameters/vehicle_params.yaml
 *===========================================================================*/

#define VP_WHEELBASE            FP_QP_CONST(MPC_FPGA_WHEELBASE_M)     /* Distance between front and rear axles */
#define VP_LF                   FP_QP_CONST(MPC_FPGA_LF_M)            /* Distance from CG to front axle */
#define VP_LR                   FP_QP_CONST(MPC_FPGA_LR_M)            /* Distance from CG to rear axle */
#define VP_MASS                 FP_QP_CONST(MPC_FPGA_MASS_KG)         /* Vehicle mass (kg) */
#define VP_IZ                   FP_QP_CONST(MPC_FPGA_IZ_KGM2)         /* Yaw moment of inertia (kg*m^2) */
#define VP_CG_HEIGHT            FP_QP_CONST(MPC_FPGA_CG_HEIGHT_M)     /* Center of gravity height (m) */
#define VP_GRAVITY              FP_QP_CONST(MPC_FPGA_GRAVITY_MS2)     /* Gravitational acceleration (m/s^2) */
#define VP_MU                   FP_QP_CONST(MPC_FPGA_MU)              /* Tire friction coefficient */
#define VP_MAX_STEER            FP_QP_CONST(MPC_FPGA_MAX_STEER_RAD)   /* Maximum steering angle (rad) */
#define VP_MAX_VEL              FP_QP_CONST(MPC_FPGA_MAX_VEL_MPS)     /* Maximum velocity (m/s) */
#define VP_MIN_VEL              FP_QP_CONST(MPC_FPGA_MIN_VEL_MPS)     /* Minimum velocity (m/s) */
#define VP_MAX_ACCEL            fp_mul(VP_MU, VP_GRAVITY)  /* Maximum acceleration (m/s^2) = mu * g */
#define VP_MIN_ACCEL            (-(VP_MAX_ACCEL))           /* Minimum acceleration (m/s^2) */
#define VP_MAX_STEER_RATE       FP_QP_CONST(MPC_FPGA_MAX_STEER_RATE_RADPS)   /* Maximum steering rate (rad/s) */
#define VP_C_ALPHA_F_NRAD       FP_QP_CONST(MPC_FPGA_C_ALPHA_F_N_PER_RAD)     /* Front tire cornering stiffness (N/rad) */
#define VP_C_ALPHA_R_NRAD       FP_QP_CONST(MPC_FPGA_C_ALPHA_R_N_PER_RAD)     /* Rear tire cornering stiffness (N/rad) */

#define VP_INV_L                fp_div(FP_ONE, VP_WHEELBASE)         /* 1/L */
#define VP_MG                   fp_mul(VP_MASS, VP_GRAVITY)          /* mass * gravity */
#define VP_MG_LR                fp_mul(VP_MG, VP_LR)                 /* mass * gravity * l_r */
#define VP_MG_LF                fp_mul(VP_MG, VP_LF)                 /* mass * gravity * l_f */
#define VP_INV_MASS             fp_div(FP_ONE, VP_MASS)              /* 1/mass */
#define VP_INV_IZ               fp_div(FP_ONE, VP_IZ)                /* 1/I_z */

#define VP_C_SHAPE              FP_QP_CONST(MPC_FPGA_PACEJKA_C_SHAPE)       /* Pacejka shape factor C for lateral slip */
#define VP_INV_C_SHAPE          fp_div(FP_ONE, VP_C_SHAPE)
#define MIN_STIFF_SCALE         FP_QP_CONST(MPC_FPGA_MIN_STIFF_SCALE)       /* Minimum stiffness scale to prevent singularities */

/*
 * Static normal loads and Pacejka D terms per axle:
 * Fz_front = (m*g*lr)/L
 * Fz_rear  = (m*g*lf)/L
 * D_front  = mu * Fz_front
 * D_rear   = mu * Fz_rear
*/
#define VP_FZ_FRONT             fp_div(VP_MG_LR, VP_WHEELBASE)
#define VP_FZ_REAR              fp_div(VP_MG_LF, VP_WHEELBASE)
#define VP_D_FRONT              fp_mul(VP_MU, VP_FZ_FRONT)
#define VP_D_REAR               fp_mul(VP_MU, VP_FZ_REAR)

/*
 * Normalize absolute cornering stiffness (N/rad) into model C_S terms ([1/rad]):
 *   C_Sf = C_alpha_f / D_front
 *   C_Sr = C_alpha_r / D_rear
 *
 * This is the exact normalization used by the linear tire form:
 *   Fy = mu * C_S * Fz * alpha.
 */
#define VP_C_ALPHA_SF           fp_div(VP_C_ALPHA_F_NRAD, VP_D_FRONT)
#define VP_C_ALPHA_SR           fp_div(VP_C_ALPHA_R_NRAD, VP_D_REAR) 

/*
 * B factors for Pacejka nonlinearity: Fy = D * sin(C * atan(B * alpha)).
 * B appears inside atan(B*alpha), so it is required explicitly.
 */
#define VP_B_FRONT              fp_div(VP_C_ALPHA_SF, VP_C_SHAPE)
#define VP_B_REAR               fp_div(VP_C_ALPHA_SR, VP_C_SHAPE)

/* C_shape * B aliases used in Jacobian terms (equal to C_alpha_S* by construction). */
#define VP_CB_FRONT             (VP_C_ALPHA_SF)
#define VP_CB_REAR              (VP_C_ALPHA_SR)

/* Precomputed minimum effective stiffness factors (mu*C_S*MIN_STIFF_SCALE) */
#define VP_MU_CSF_MIN           fp_mul(fp_mul(VP_MU, VP_C_ALPHA_SF), MIN_STIFF_SCALE) /* mu*C_Sf*MIN_STIFF_SCALE */
#define VP_MU_CSR_MIN           fp_mul(fp_mul(VP_MU, VP_C_ALPHA_SR), MIN_STIFF_SCALE) /* mu*C_Sr*MIN_STIFF_SCALE */

/*===========================================================================
 * MPC Default Cost Weights (tuned for F1/10th)
 *===========================================================================*/

#ifdef MPC_RUNTIME_TUNE
#define MPC_DT              (mpc_rt_dt)                                 /* Runtime-overridable prediction dt */
#define MPC_W_LAT_ERROR     (mpc_rt_w_lat_error)
#define MPC_W_HEADING       (mpc_rt_w_heading)
#define MPC_W_VELOCITY      (mpc_rt_w_velocity)
#define MPC_W_LAT_VEL       (mpc_rt_w_lat_vel)
#define MPC_W_YAW_RATE      (mpc_rt_w_yaw_rate)
#define MPC_W_STEER_EFF     (mpc_rt_w_steer_eff)
#define MPC_W_ACCEL_EFF     (mpc_rt_w_accel_eff)
#define MPC_W_STEER_JERK    (mpc_rt_w_steer_jerk)
#define MPC_W_ACCEL_RATE    (mpc_rt_w_accel_rate)
#define MPC_W_DELTA_ACT     (mpc_rt_w_delta_act)
#else
#define MPC_DT              FP_QP_CONST(MPC_FPGA_PREDICTION_DT_S)      /* Prediction dt in Q16.16 */
#define MPC_W_LAT_ERROR     FP_QP_CONST(MPC_FPGA_W_LAT_ERROR)
#define MPC_W_HEADING       FP_QP_CONST(MPC_FPGA_W_HEADING)
#define MPC_W_VELOCITY      FP_QP_CONST(MPC_FPGA_W_VELOCITY)
#define MPC_W_LAT_VEL       FP_QP_CONST(MPC_FPGA_W_LAT_VEL)
#define MPC_W_YAW_RATE      FP_QP_CONST(MPC_FPGA_W_YAW_RATE)
#define MPC_W_STEER_EFF     FP_QP_CONST(MPC_FPGA_W_STEER_EFF)
#define MPC_W_ACCEL_EFF     FP_QP_CONST(MPC_FPGA_W_ACCEL_EFF)
#define MPC_W_STEER_JERK    FP_QP_CONST(MPC_FPGA_W_STEER_JERK)
#define MPC_W_ACCEL_RATE    FP_QP_CONST(MPC_FPGA_W_ACCEL_RATE)
#define MPC_W_DELTA_ACT     FP_QP_CONST(MPC_FPGA_W_DELTA_ACT)
#endif

/* Precomputed dt*inv_mass and dt*inv_Iz */
#define VP_DT_INV_MASS      fp_mul(MPC_DT, VP_INV_MASS)     /* dt * (1/mass) */
#define VP_DT_INV_IZ        fp_mul(MPC_DT, VP_INV_IZ)       /* dt * (1/I_z) */

/* Control period for cross-call rate scaling.
 * Default control loop is 200 Hz => 0.005 s. */
#define MPC_CONTROL_RATE_HZ     FP_QP_CONST(MPC_FPGA_CONTROL_RATE_HZ)
#define MPC_CONTROL_DT          fp_div(FP_ONE, MPC_CONTROL_RATE_HZ)

/* Cross-call rate scale = control_dt / prediction_dt. */
#define MPC_CROSS_CALL_SCALE    fp_div(MPC_CONTROL_DT, MPC_DT)

/* === Precomputed 2x weights for QP Hessian diagonal ===
 * All computed at compile time via integer arithmetic. */
#define MPC_Q2_LAT_ERROR    ((MPC_W_LAT_ERROR) << 1)   
#define MPC_Q2_HEADING      ((MPC_W_HEADING) << 1)   
#define MPC_Q2_VELOCITY     ((MPC_W_VELOCITY) << 1)    
#define MPC_Q2_LAT_VEL      ((MPC_W_LAT_VEL) << 1)  
#define MPC_Q2_YAW_RATE     ((MPC_W_YAW_RATE) << 1)    
#define MPC_Q2_DELTA_ACT    ((MPC_W_DELTA_ACT) << 1) 
#define MPC_Q2_STEER_JERK   ((MPC_W_STEER_JERK) << 1)    
#define MPC_Q2_ACCEL_RATE   ((MPC_W_ACCEL_RATE) << 1)    
#define MPC_R2_STEER        (((MPC_W_STEER_EFF) + (MPC_W_STEER_JERK)) << 1)
#define MPC_R2_ACCEL        (((MPC_W_ACCEL_EFF) + (MPC_W_ACCEL_RATE)) << 1) 
#define MPC_N2_STEER_JERK   (-((MPC_W_STEER_JERK) << 1))  
#define MPC_N2_ACCEL_RATE   (-((MPC_W_ACCEL_RATE) << 1))  

/* Cross-call scaled variants for step 0 (scale = control_dt / prediction_dt) */
#define MPC_W_STEER_JERK_CS fp_mul(MPC_W_STEER_JERK, MPC_CROSS_CALL_SCALE)
#define MPC_W_ACCEL_RATE_CS fp_mul(MPC_W_ACCEL_RATE, MPC_CROSS_CALL_SCALE)
#define MPC_Q2_JERK_CS      ((MPC_W_STEER_JERK_CS) << 1)
#define MPC_Q2_ARATE_CS     ((MPC_W_ACCEL_RATE_CS) << 1)
#define MPC_R2_STEER_CS     (((MPC_W_STEER_EFF) + (MPC_W_STEER_JERK_CS)) << 1)
#define MPC_R2_ACCEL_CS     (((MPC_W_ACCEL_EFF) + (MPC_W_ACCEL_RATE_CS)) << 1)

/*===========================================================================
 * Solver/Constraint Constants
 *===========================================================================*/

#define BIG_BOUND           FP_QP_CONST(MPC_FPGA_BIG_BOUND)
#ifdef MPC_RUNTIME_TUNE
#define MIN_LIN_VEL         (mpc_rt_min_lin_vel)
#define STABILITY_LIMIT_VAL (mpc_rt_stability_limit)
#define WALL_MARGIN         (mpc_rt_wall_margin)
#define WALL_START          (mpc_rt_wall_start)
#define WALL_STRIDE         (mpc_rt_wall_stride)
#define WALL_END            (mpc_rt_wall_end)
#else
#define MIN_LIN_VEL         FP_QP_CONST(MPC_FPGA_MIN_LIN_VEL_MPS)
#define STABILITY_LIMIT_VAL FP_QP_CONST(MPC_FPGA_STABILITY_LIMIT)
#define WALL_MARGIN         FP_QP_CONST(MPC_FPGA_WALL_MARGIN_M)
#define WALL_START          MPC_FPGA_WALL_START
#define WALL_STRIDE         MPC_FPGA_WALL_STRIDE
#define WALL_END            MPC_FPGA_WALL_END
#endif
#define V_SWITCH            FP_QP_CONST(MPC_FPGA_V_SWITCH_MPS)
#define BOUND_THRESHOLD     FP_QP_CONST(MPC_FPGA_BOUND_THRESHOLD)
#ifdef MPC_RUNTIME_TUNE
#define WP_ADVANCE_MAX      (mpc_rt_wp_advance_max)
#else
#define WP_ADVANCE_MAX      MPC_FPGA_WP_ADVANCE_MAX
#endif

/* ADMM default parameters */
#ifdef MPC_RUNTIME_TUNE
#define ADMM_RHO_DEFAULT    (mpc_rt_admm_rho)
#define ADMM_RHO_U_DEFAULT  (mpc_rt_admm_rho_u)
#define ADMM_TOL_DEFAULT    (mpc_rt_admm_tol)
#define ADMM_ALPHA_DEFAULT  (mpc_rt_admm_alpha)
#else
#define ADMM_RHO_DEFAULT    FP_QP_CONST(MPC_FPGA_ADMM_RHO)
#define ADMM_RHO_U_DEFAULT  FP_QP_CONST(MPC_FPGA_ADMM_RHO_U)
#define ADMM_TOL_DEFAULT    FP_QP_CONST(MPC_FPGA_ADMM_TOL)
#define ADMM_ALPHA_DEFAULT  FP_QP_CONST(MPC_FPGA_ADMM_ALPHA)
#endif

/*===========================================================================
 * Data Structures
 *===========================================================================*/

/** Waypoint stored in FPGA BRAM */
typedef struct {
    fp_QP_t x;
    fp_QP_t y;
    fp_QP_t psi;
    fp_QP_t vx;
    fp_QP_t kappa;
    fp_QP_t ax;
    fp_QP_t left_bound;
    fp_QP_t right_bound;
} MpcWaypoint_t;

/** Reference point for one MPC prediction step */
typedef struct {
    fp_QP_t velocity;
    fp_QP_t kappa;
    fp_QP_t yaw_rate;
    fp_QP_t left_bound;
    fp_QP_t right_bound;
} MpcRefPoint_t;

/** Per-step QP data for Riccati-ADMM */
typedef struct {
    fp_QP_t A[MPC_NX_AUG][MPC_NX_AUG];
    fp_QP_t B[MPC_NX_AUG][MPC_NU];
    fp_QP_t Q_diag[MPC_NX_AUG];
    fp_QP_t q[MPC_NX_AUG];
    fp_QP_t R_diag[MPC_NU];
    fp_QP_t r[MPC_NU];
    fp_QP_t N_cross[MPC_NX_AUG][MPC_NU];
    fp_QP_t x_lb[MPC_NX_AUG];
    fp_QP_t x_ub[MPC_NX_AUG];
    fp_QP_t u_lb[MPC_NU];
    fp_QP_t u_ub[MPC_NU];
} StepData_t;

/** MPC solver status */
typedef enum {
    MPC_STATUS_OPTIMAL  = 0,
    MPC_STATUS_MAX_ITER = 1,
    MPC_STATUS_ERROR    = 2
} MpcStatus_t;

/** ADMM warm-start state (persists between calls) */
typedef struct {
    fp_QP_t z_x[MPC_HORIZON + 1][MPC_NX_AUG];
    fp_QP_t z_u[MPC_HORIZON][MPC_NU];
    fp_QP_t y_x[MPC_HORIZON + 1][MPC_NX_AUG];
    fp_QP_t y_u[MPC_HORIZON][MPC_NU];
    fp_QP_t rho;    
    fp_QP_t rho_u;  
    int initialized;
} AdmmState_t;

/** Solver solution output */
typedef struct {
    fp_QP_t x[MPC_HORIZON + 1][MPC_NX_AUG];
    fp_QP_t u[MPC_HORIZON][MPC_NU];
    int iterations;
    fp_QP_t primal_residual;
    fp_QP_t dual_residual;
    MpcStatus_t status;
} MpcSolution_t;

/** ADMM configuration */
typedef struct {
    fp_QP_t rho;
    fp_QP_t rho_u;
    fp_QP_t tolerance;
    fp_QP_t alpha;
    int max_iterations;
    int adaptive_rho;
} AdmmConfig_t;

/** Persistent MPC state (between calls) */
typedef struct {
    fp_QP_t prev_steer_rate;
    fp_QP_t prev_accel;
    fp_QP_t prev_delta_cmd;
    fp_QP_t actual_steering;
    fp_QP_t prev_curvature;
    int prev_converged;
} MpcPersistState_t;

#endif /* MPC_FPGA_TYPES_H */
