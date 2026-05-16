/**
 * @file mpc_fpga_types.h
 * @brief Types, constants, and shared structures for the FPGA MPC solver.
 * @details Defines dimensions, state indices, precomputed fixed-point
 *          constants, and shared data structures used across the
 *          vehicle-model, Riccati solver, and top-level integration units.
 * @dependencies fp_math_hls.h, mpc_fpga_constants.h
 */

#ifndef MPC_FPGA_TYPES_H
#define MPC_FPGA_TYPES_H

/* Host/debug builds should honor runtime tuning env overrides for parity with
 * CPU simulation and faster diagnosis. Keep synthesis builds compile-time
 * fixed. */
#if !defined(MPC_RUNTIME_TUNE) && !defined(MPC_HLS_BUILD) && !defined(__SYNTHESIS__)
#define MPC_RUNTIME_TUNE
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
#define MPC_NX_FRENET 5

/** Augmented state:
 *  [e_y, e_psi, vx, vy, omega, delta_actual, delta_rate_prev, accel_prev]
 */
#define MPC_NX_AUG 8

/** Dense block size in A matrix (Frenet + delta_actual) */
#define MPC_NX_DENSE 6

/** Control dimension: [delta_rate, acceleration] */
#define MPC_NU 2

/** Number of ADMM-projected state channels */
#define MPC_NX_ADMM_STATE 2

/** Constrained-state slots stored in ADMM warm-start buffers */
#define IDX_ADMM_EY 0
#define IDX_ADMM_DELTA_ACT 1

/** Warm-start bound-jump threshold for invalidating persistent state */
#ifndef MPC_WS_BOUND_THRESH
#define MPC_WS_BOUND_THRESH FP_QP_CONST(0.05)
#endif

/** Fixed prediction horizon */
#define MPC_HORIZON MPC_FPGA_HORIZON_STEPS

/** Horizon length used for loops that include terminal state k = N */
#define MPC_HORIZON_PLUS_ONE (MPC_HORIZON + 1)

/** Maximum ADMM iterations */
#define MPC_MAX_ADMM_ITER MPC_FPGA_MAX_ADMM_ITER

/** Maximum ADMM passes including optional cold-start bootstrap pass */
#define MPC_MAX_ADMM_PASS_COUNT (MPC_MAX_ADMM_ITER + 1)

/*===========================================================================
 * HLS Resource Constraints
 *===========================================================================*/

/* ADMM state z/y update loop target II.
 * Override at compile time: -DMPC_HLS_STATE_ZY_II=N */
#ifndef MPC_HLS_STATE_ZY_II
#define MPC_HLS_STATE_ZY_II 2
#endif

/* Structural model signature used to invalidate persistent warm-start state
 * when the linearization/model basis changes. */
#ifndef MPC_MODEL_SIGNATURE
#define MPC_MODEL_SIGNATURE 1
#endif

/** ADMM control z/y update loop target II.
 *  Override at compile time: -DMPC_HLS_CTRL_ZY_II=N */
#ifndef MPC_HLS_CTRL_ZY_II
#define MPC_HLS_CTRL_ZY_II 2
#endif

/*===========================================================================
 * Augmented State Indices
 *===========================================================================*/

#define IDX_EY 0
#define IDX_EPSI 1
#define IDX_VX 2
#define IDX_VY 3
#define IDX_OMEGA 4
#define IDX_DELTA_ACT 5
#define IDX_DELTA_RATE_PREV 6
#define IDX_ACCEL_PREV 7

/*===========================================================================
 * Trajectory
 *===========================================================================*/

#define MAX_TRAJECTORY_SIZE 64

/*===========================================================================
 * F1/10th Vehicle Parameters
 *===========================================================================*/

#define VP_WHEELBASE  FP_QP_CONST(MPC_FPGA_WHEELBASE_M)
#define VP_LF         FP_QP_CONST(MPC_FPGA_LF_M)
#define VP_LR         FP_QP_CONST(MPC_FPGA_LR_M)
#define VP_MASS       FP_QP_CONST(MPC_FPGA_MASS_KG)
#define VP_IZ         FP_QP_CONST(MPC_FPGA_IZ_KGM2)
#define VP_CG_HEIGHT  FP_QP_CONST(MPC_FPGA_CG_HEIGHT_M)
#define VP_GRAVITY    FP_QP_CONST(MPC_FPGA_GRAVITY_MS2)
#define VP_MU         FP_QP_CONST(MPC_FPGA_MU)
#define VP_MAX_STEER  FP_QP_CONST(MPC_FPGA_MAX_STEER_RAD)
#define VP_MAX_VEL    FP_QP_CONST(MPC_FPGA_MAX_VEL_MPS)
#define VP_MIN_VEL    FP_QP_CONST(MPC_FPGA_MIN_VEL_MPS)
#define VP_MAX_ACCEL  (VP_MU * VP_GRAVITY)
#define VP_MIN_ACCEL  (-(VP_MAX_ACCEL))
#define VP_MAX_STEER_RATE FP_QP_CONST(MPC_FPGA_MAX_STEER_RATE_RADPS)

#define VP_C_ALPHA_F_NRAD FP_QP_CONST(MPC_FPGA_C_ALPHA_F_N_PER_RAD)
#define VP_C_ALPHA_R_NRAD FP_QP_CONST(MPC_FPGA_C_ALPHA_R_N_PER_RAD)

#define VP_INV_L     FP_QP_CONST(MPC_FPGA_INV_WHEELBASE)
#define VP_MG        (VP_MASS * VP_GRAVITY)
#define VP_MG_LR     (VP_MG * VP_LR)
#define VP_MG_LF     (VP_MG * VP_LF)
#define VP_INV_MASS  FP_QP_CONST(MPC_FPGA_INV_MASS)
#define VP_INV_IZ    FP_QP_CONST(MPC_FPGA_INV_IZ)

#define VP_C_SHAPE      FP_QP_CONST(MPC_FPGA_PACEJKA_C_SHAPE)
#define VP_INV_C_SHAPE  FP_QP_CONST(MPC_FPGA_INV_PACEJKA_C_SHAPE)
#define MIN_STIFF_SCALE FP_QP_CONST(MPC_FPGA_MIN_STIFF_SCALE)

#define VP_FZ_FRONT FP_QP_CONST(MPC_FPGA_FZ_FRONT_N)
#define VP_FZ_REAR  FP_QP_CONST(MPC_FPGA_FZ_REAR_N)
#define VP_D_FRONT  FP_QP_CONST(MPC_FPGA_D_FRONT_N)
#define VP_D_REAR   FP_QP_CONST(MPC_FPGA_D_REAR_N)

#define VP_FZ_LOAD_GAIN \
  FP_QP_CONST(MPC_FPGA_MASS_KG * MPC_FPGA_CG_HEIGHT_M * MPC_FPGA_INV_WHEELBASE)

#define VP_D_LOAD_GAIN \
  FP_QP_CONST(MPC_FPGA_MU * MPC_FPGA_MASS_KG * MPC_FPGA_CG_HEIGHT_M * \
              MPC_FPGA_INV_WHEELBASE)

#define VP_CMIN_FRONT_STATIC \
  FP_QP_CONST(MPC_FPGA_C_ALPHA_F_N_PER_RAD * MPC_FPGA_MIN_STIFF_SCALE)

#define VP_CMIN_REAR_STATIC \
  FP_QP_CONST(MPC_FPGA_C_ALPHA_R_N_PER_RAD * MPC_FPGA_MIN_STIFF_SCALE)

#define VP_CMIN_FRONT_LOAD_GAIN \
  FP_QP_CONST(MPC_FPGA_MASS_KG * MPC_FPGA_CG_HEIGHT_M * \
              MPC_FPGA_INV_WHEELBASE * MPC_FPGA_MU * \
              MPC_FPGA_C_ALPHA_SF_NORM * MPC_FPGA_MIN_STIFF_SCALE)

#define VP_CMIN_REAR_LOAD_GAIN \
  FP_QP_CONST(MPC_FPGA_MASS_KG * MPC_FPGA_CG_HEIGHT_M * \
              MPC_FPGA_INV_WHEELBASE * MPC_FPGA_MU * \
              MPC_FPGA_C_ALPHA_SR_NORM * MPC_FPGA_MIN_STIFF_SCALE)

#define VP_C_ALPHA_SF FP_QP_CONST(MPC_FPGA_C_ALPHA_SF_NORM)
#define VP_C_ALPHA_SR FP_QP_CONST(MPC_FPGA_C_ALPHA_SR_NORM)

#define VP_B_FRONT FP_QP_CONST(MPC_FPGA_B_FRONT)
#define VP_B_REAR  FP_QP_CONST(MPC_FPGA_B_REAR)

#define VP_CB_FRONT (VP_C_ALPHA_SF)
#define VP_CB_REAR  (VP_C_ALPHA_SR)

/*===========================================================================
 * MPC Default Cost Weights
 *===========================================================================*/

#ifdef MPC_RUNTIME_TUNE
#define MPC_DT             (mpc_rt_dt)
#define MPC_W_LAT_ERROR    (mpc_rt_w_lat_error)
#define MPC_W_HEADING      (mpc_rt_w_heading)
#define MPC_W_VELOCITY     (mpc_rt_w_velocity)
#define MPC_W_LAT_VEL      (mpc_rt_w_lat_vel)
#define MPC_W_YAW_RATE     (mpc_rt_w_yaw_rate)
#define MPC_W_STEER_EFF    (mpc_rt_w_steer_eff)
#define MPC_W_ACCEL_EFF    (mpc_rt_w_accel_eff)
#define MPC_W_STEER_JERK   (mpc_rt_w_steer_jerk)
#define MPC_W_ACCEL_RATE   (mpc_rt_w_accel_rate)
#define MPC_W_DELTA_ACT    (mpc_rt_w_delta_act)
#else
#define MPC_DT             FP_QP_CONST(MPC_FPGA_PREDICTION_DT_S)
#define MPC_W_LAT_ERROR    FP_QP_CONST(MPC_FPGA_W_LAT_ERROR)
#define MPC_W_HEADING      FP_QP_CONST(MPC_FPGA_W_HEADING)
#define MPC_W_VELOCITY     FP_QP_CONST(MPC_FPGA_W_VELOCITY)
#define MPC_W_LAT_VEL      FP_QP_CONST(MPC_FPGA_W_LAT_VEL)
#define MPC_W_YAW_RATE     FP_QP_CONST(MPC_FPGA_W_YAW_RATE)
#define MPC_W_STEER_EFF    FP_QP_CONST(MPC_FPGA_W_STEER_EFF)
#define MPC_W_ACCEL_EFF    FP_QP_CONST(MPC_FPGA_W_ACCEL_EFF)
#define MPC_W_STEER_JERK   FP_QP_CONST(MPC_FPGA_W_STEER_JERK)
#define MPC_W_ACCEL_RATE   FP_QP_CONST(MPC_FPGA_W_ACCEL_RATE)
#define MPC_W_DELTA_ACT    FP_QP_CONST(MPC_FPGA_W_DELTA_ACT)
#endif

#define VP_DT_INV_MASS     (MPC_DT * VP_INV_MASS)
#define NEG_VP_DT_INV_MASS (-(VP_DT_INV_MASS))
#define VP_DT_INV_IZ       (MPC_DT * VP_INV_IZ)

/* Control period for cross-call rate scaling */
#define MPC_CONTROL_RATE_HZ FP_QP_CONST(MPC_FPGA_CONTROL_RATE_HZ)

#ifdef MPC_RUNTIME_TUNE
#define MPC_CONTROL_DT       fp_div(FP_ONE, MPC_CONTROL_RATE_HZ)
#define MPC_CROSS_CALL_SCALE fp_div(MPC_CONTROL_DT, MPC_DT)
#else
#define MPC_CONTROL_DT       FP_QP_CONST(MPC_FPGA_CONTROL_DT_S)
#define MPC_CROSS_CALL_SCALE FP_QP_CONST(MPC_FPGA_CROSS_CALL_SCALE)
#endif

#define MPC_Q2_LAT_ERROR   ((MPC_W_LAT_ERROR) << 1)
#define MPC_Q2_HEADING     ((MPC_W_HEADING) << 1)
#define MPC_Q2_VELOCITY    ((MPC_W_VELOCITY) << 1)
#define MPC_Q2_LAT_VEL     ((MPC_W_LAT_VEL) << 1)
#define MPC_Q2_YAW_RATE    ((MPC_W_YAW_RATE) << 1)
#define MPC_Q2_DELTA_ACT   ((MPC_W_DELTA_ACT) << 1)
#define MPC_Q2_STEER_JERK  ((MPC_W_STEER_JERK) << 1)
#define MPC_Q2_ACCEL_RATE  ((MPC_W_ACCEL_RATE) << 1)
#define MPC_R2_STEER       (((MPC_W_STEER_EFF) + (MPC_W_STEER_JERK)) << 1)
#define MPC_R2_ACCEL       (((MPC_W_ACCEL_EFF) + (MPC_W_ACCEL_RATE)) << 1)
#define MPC_N2_STEER_JERK  (-((MPC_W_STEER_JERK) << 1))
#define MPC_N2_ACCEL_RATE  (-((MPC_W_ACCEL_RATE) << 1))

#ifdef MPC_RUNTIME_TUNE
#define MPC_W_STEER_JERK_CS (MPC_W_STEER_JERK * MPC_CROSS_CALL_SCALE)
#define MPC_W_ACCEL_RATE_CS (MPC_W_ACCEL_RATE * MPC_CROSS_CALL_SCALE)
#else
#define MPC_W_STEER_JERK_CS FP_QP_CONST(MPC_FPGA_W_STEER_JERK_CS)
#define MPC_W_ACCEL_RATE_CS FP_QP_CONST(MPC_FPGA_W_ACCEL_RATE_CS)
#endif

#define MPC_Q2_JERK_CS    ((MPC_W_STEER_JERK_CS) << 1)
#define MPC_Q2_ARATE_CS   ((MPC_W_ACCEL_RATE_CS) << 1)
#define MPC_R2_STEER_CS   (((MPC_W_STEER_EFF) + (MPC_W_STEER_JERK_CS)) << 1)
#define MPC_R2_ACCEL_CS   (((MPC_W_ACCEL_EFF) + (MPC_W_ACCEL_RATE_CS)) << 1)

/*===========================================================================
 * Solver / Constraint Constants
 *===========================================================================*/

#define BIG_BOUND           FP_QP_CONST(MPC_FPGA_BIG_BOUND)
#define MIN_LIN_VEL         FP_QP_CONST(MPC_FPGA_MIN_LIN_VEL_MPS)
#define STABILITY_LIMIT_VAL FP_QP_CONST(MPC_FPGA_STABILITY_LIMIT)
#define WALL_MARGIN         FP_QP_CONST(MPC_FPGA_WALL_MARGIN_M)
#define V_SWITCH            FP_QP_CONST(MPC_FPGA_V_SWITCH_MPS)
#define BOUND_THRESHOLD     FP_QP_CONST(MPC_FPGA_BOUND_THRESHOLD)
#define WALL_BIAS_CLEAR_M   FP_QP_CONST(MPC_FPGA_WALL_BIAS_CLEAR_M)
#define WALL_BIAS_MAX_M     FP_QP_CONST(MPC_FPGA_WALL_BIAS_MAX_M)
#define WALL_BOUND_WINDOW   MPC_FPGA_WALL_BOUND_WINDOW

#define ADMM_RHO_MIN FP_QP_CONST(1.0)
#define ADMM_RHO_MAX FP_QP_CONST(80.0)

#ifndef MPC_WS_CURVATURE_THRESH
#define MPC_WS_CURVATURE_THRESH FP_QP_CONST(0.25)
#endif

#ifdef MPC_RUNTIME_TUNE
#define ADMM_RHO_DEFAULT          (mpc_rt_admm_rho)
#define ADMM_RHO_U_DEFAULT        (mpc_rt_admm_rho_u)
#define ADMM_TOL_DEFAULT          (mpc_rt_admm_tol)
#define ADMM_MAX_ITER_DEFAULT     (mpc_rt_admm_max_iter)
#define ADMM_ADAPTIVE_RHO_DEFAULT (mpc_rt_adaptive_rho)
#else
#define ADMM_RHO_DEFAULT          FP_QP_CONST(MPC_FPGA_ADMM_RHO)
#define ADMM_RHO_U_DEFAULT        FP_QP_CONST(MPC_FPGA_ADMM_RHO_U)
#define ADMM_TOL_DEFAULT          FP_QP_CONST(MPC_FPGA_ADMM_TOL)
#define ADMM_MAX_ITER_DEFAULT     MPC_MAX_ADMM_ITER
#define ADMM_ADAPTIVE_RHO_DEFAULT MPC_HLS_ADAPTIVE_RHO
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

/** Reference point for one MPC prediction step.
 *  Eight fp_QP_t fields = 8 * 32 bits = 256 bits = 32 bytes.
 *  Keep the struct naturally 32-byte aligned for regular packed access.
 */
typedef struct alignas(32) {
  fp_QP_t reference_heading_error;
  fp_QP_t reference_lateral_error;
  fp_QP_t reference_velocity;
  fp_QP_t reference_lateral_velocity;
  fp_QP_t reference_yaw_rate;
  fp_QP_t path_curvature;
  fp_QP_t left_wall_bound;
  fp_QP_t right_wall_bound;
} MpcRefPoint_t;

/** Per-step dynamic QP data for Riccati-ADMM.
 *
 * Only stage-varying quantities live here:
 * - dense 6x6 dynamics block A
 * - sparse B terms used by the augmented model
 * - affine dynamics bias d for states 0..5
 * - linear cost terms q for states 0..5
 * - dynamic e_y box bounds
 * - dynamic accel upper bound
 *
 * All constant weights and constant bounds are reconstructed directly from
 * compile-time defines in the solver. This keeps the horizon memory compact,
 * avoids rewriting invariant policy fields every MPC call, and makes the
 * struct a regular 32-byte-multiple record for friendlier HLS packing.
 *
 * Size: 224 bytes = 7 x 32-byte lanes.
 */
typedef struct alignas(32) {
  fp_QP_t A[MPC_NX_DENSE][MPC_NX_DENSE];

  fp_QP_t d[MPC_NX_DENSE];
  fp_QP_t q[MPC_NX_DENSE];

  fp_QP_t B_delta_rate;
  fp_QP_t B_vx_accel;
  fp_QP_t B_vy_accel;
  fp_QP_t B_omega_accel;

  fp_QP_t ey_lb;
  fp_QP_t ey_ub;
  fp_QP_t accel_ub;
  fp_QP_t pad0;
} StepData_t;

static_assert((sizeof(StepData_t) % 32) == 0,
              "StepData_t must stay padded to a 32-byte multiple");

/** MPC solver status */
typedef enum {
  MPC_STATUS_OPTIMAL = 0,
  MPC_STATUS_MAX_ITER = 1
} MpcStatus_t;

/** ADMM warm-start state (persists between calls) */
typedef struct {
  fp_QP_t z_x[MPC_HORIZON_PLUS_ONE][MPC_NX_AUG];
  fp_QP_t z_u[MPC_HORIZON][MPC_NU];
  fp_QP_t y_x[MPC_HORIZON_PLUS_ONE][MPC_NX_AUG];
  fp_QP_t y_u[MPC_HORIZON][MPC_NU];
  fp_QP_t rho;
  fp_QP_t rho_u;
  int initialized;
} AdmmState_t;

/** Shared ADMM reset helpers used by top-level wrappers and solver wrapper. */
static inline void mpc_admm_reset_all_hls(AdmmState_t *admm_state) {
#pragma HLS INLINE
  if (!admm_state)
    return;

  for (int k = 0; k <= MPC_HORIZON; ++k) {
#pragma HLS PIPELINE II = 1
    for (int s = 0; s < MPC_NX_AUG; ++s) {
      admm_state->z_x[k][s] = FP_QP_CONST(0.0);
      admm_state->y_x[k][s] = FP_QP_CONST(0.0);
    }
  }

  for (int k = 0; k < MPC_HORIZON; ++k) {
#pragma HLS PIPELINE II = 1
    for (int a = 0; a < MPC_NU; ++a) {
      admm_state->z_u[k][a] = FP_QP_CONST(0.0);
      admm_state->y_u[k][a] = FP_QP_CONST(0.0);
    }
  }

  admm_state->rho = FP_QP_CONST(0.0);
  admm_state->rho_u = FP_QP_CONST(0.0);
  admm_state->initialized = 0;
}

static inline void mpc_admm_zero_duals_hls(AdmmState_t *admm_state) {
#pragma HLS INLINE
  if (!admm_state)
    return;

  for (int k = 0; k <= MPC_HORIZON; ++k) {
#pragma HLS PIPELINE II = 1
    for (int s = 0; s < MPC_NX_AUG; ++s) {
      admm_state->y_x[k][s] = FP_QP_CONST(0.0);
    }
  }

  for (int k = 0; k < MPC_HORIZON; ++k) {
#pragma HLS PIPELINE II = 1
    for (int a = 0; a < MPC_NU; ++a) {
      admm_state->y_u[k][a] = FP_QP_CONST(0.0);
    }
  }

  admm_state->rho = FP_QP_CONST(0.0);
  admm_state->rho_u = FP_QP_CONST(0.0);
  admm_state->initialized = 1;
}

/** Solver solution output */
typedef struct {
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
  int max_iterations;
  int adaptive_rho;
} AdmmConfig_t;

/** Persistent MPC state across calls */
/** Persistent MPC state (between calls) */
typedef struct {
  fp_QP_t prev_steer_rate;
  fp_QP_t prev_accel;
  fp_QP_t prev_delta_cmd;
  fp_QP_t actual_steering;

  fp_QP_t prev_curvature;
  fp_QP_t prev_ref_velocity;
  fp_QP_t prev_left_wall_bound;
  fp_QP_t prev_right_wall_bound;

  int prev_model_signature;
  int last_status;
  int last_iterations;
  int max_iter_streak;
} MpcPersistState_t;

/** Top-level persistent core state.
 *  Use this in the FPGA wrapper instead of separate globals for ADMM state,
 *  persistent actuator/model state, and initialization flag.
 */
typedef struct {
  AdmmState_t admm;
  MpcPersistState_t persist;
  int initialized;
} MpcCorePersistentState_t;

#endif /* MPC_FPGA_TYPES_H */
