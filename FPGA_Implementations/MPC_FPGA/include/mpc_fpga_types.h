/**
 * @file mpc_fpga_types.h
 * @brief Types, constants, and data structures for HLS MPC solver
 *
 * Defines all types for the FPGA MPC Riccati-ADMM implementation.
 * Uses Q16.16 fixed-point arithmetic (int32_t with 64-bit intermediates).
 * Targets Xilinx Zynq UltraScale+ ZU3EG (Ultra96-V2).
 *
 * HLS-compatible: no malloc, no stdio, no globals, no function pointers.
 */

#ifndef MPC_FPGA_TYPES_H
#define MPC_FPGA_TYPES_H

/* Always enable HLS target mode */
#ifndef MPC_HLS_TARGET
#define MPC_HLS_TARGET
#endif

/* Real-hardware vehicle model: full atan-based slip angles,
 * cos(δ)/sin(δ) force resolution, and Pacejka tire saturation.
 * No simulation-matching simplifications. */

#include <stdint.h>

/*===========================================================================
 * Fixed-Point Type and Constants (Q16.16)
 *===========================================================================*/

typedef int32_t fixed_point_t;

#define FP_FRAC_BITS    16
#define FP_ONE          (1 << FP_FRAC_BITS)       /* 65536 */
#define FP_TWO          (2 << FP_FRAC_BITS)       /* 131072 */
#define FP_HALF         (FP_ONE >> 1)              /* 32768 */
#define FP_PI           205887                     /* pi */
#define FP_PI_HALF      102943                     /* pi/2 */
#define FP_TWO_PI       411775                     /* 2*pi */

/** Compile-time float to Q16.16 conversion */
#define FP_CONST(x) ((fixed_point_t)(((double)(x) >= 0) ? \
                    ((double)(x) * FP_ONE + 0.5) : \
                    ((double)(x) * FP_ONE - 0.5)))

/** Runtime conversions */
#define DOUBLE_TO_FP(x) ((fixed_point_t)(((x) >= 0) ? \
                        ((x) * FP_ONE + 0.5) : ((x) * FP_ONE - 0.5)))
#define FP_TO_DOUBLE(x) ((double)(x) / (double)FP_ONE)
#define FP_TO_FLOAT(x)  ((float)(x) / (float)FP_ONE)

/*===========================================================================
 * MPC Dimension Constants
 *===========================================================================*/

/** Frenet state dimension: [e_y, e_psi, vx, vy, omega] */
#define MPC_NX_FRENET   5

/** Augmented state: [e_y, e_psi, vx, vy, omega, delta_actual, drate_prev, accel_prev] */
#define MPC_NX_AUG      8

/** Dense block size in A matrix (Frenet + delta_actual) */
#define MPC_NX_DENSE    6

/** Control dimension: [delta_rate, acceleration] */
#define MPC_NU          2

/** Fixed prediction horizon */
#define MPC_HORIZON     19

/** Maximum ADMM iterations (reduced from 20 for timing budget) */
#define MPC_MAX_ADMM_ITER 8

/*===========================================================================
 * HLS Resource Constraints
 *===========================================================================*/

/** Maximum multiplier instances in Riccati pass (trades latency for DSP area).
 *  Default 6: limits DSP usage in Riccati solver to fit ZU3EG (360 DSP).
 *  Vehicle model uses fp_mul_vm (separate DSP-pipelined, non-inline).
 *  Override at compile time: -DMPC_HLS_MUL_LIMIT=N */
#ifndef MPC_HLS_MUL_LIMIT
#define MPC_HLS_MUL_LIMIT 4
#endif

/*===========================================================================
 * Augmented State Indices
 *===========================================================================*/

#define IDX_EY          0
#define IDX_EPSI        1
#define IDX_VX          2
#define IDX_VY          3
#define IDX_OMEGA       4
#define IDX_DELTA_ACT   5   /* Actual servo steering angle */
#define IDX_DRATE_PREV  6   /* Previous steering rate (for jerk penalty) */
#define IDX_ACCEL_PREV  7   /* Previous acceleration (for accel rate) */

/*===========================================================================
 * Trajectory
 *===========================================================================*/

#define MAX_TRAJECTORY_SIZE 1024

/*===========================================================================
 * F1/10th Vehicle Parameters (compile-time constants)
 *===========================================================================*/

#define VP_WHEELBASE        FP_CONST(0.324)
#define VP_LF               FP_CONST(0.166)
#define VP_LR               FP_CONST(0.16)
#define VP_MASS             FP_CONST(3.314)
#define VP_IZ               FP_CONST(0.035)
#define VP_CG_HEIGHT        FP_CONST(0.0703)
#define VP_GRAVITY          FP_CONST(9.81)
#define VP_CSF              FP_CONST(2.804)
#define VP_CSR              FP_CONST(3.320)
#define VP_MU               FP_CONST(0.7463)
#define VP_MAX_STEER        FP_CONST(0.4282)
#define VP_MAX_VEL          FP_CONST(20.0)
#define VP_MIN_VEL          ((fixed_point_t)0)
#define VP_MAX_ACCEL        FP_CONST(8.0)
#define VP_MIN_ACCEL        FP_CONST(-7.7)
#define VP_MAX_STEER_RATE   FP_CONST(2.849)

/* Precomputed reciprocals for FPGA efficiency */
#define VP_INV_MASS         FP_CONST(0.301750)  /* 1/3.314 */
#define VP_INV_IZ           FP_CONST(28.571429) /* 1/0.035 */
#define VP_INV_L            FP_CONST(3.086420)  /* 1/0.324 */

/* Pacejka tire model constants (for real-hardware linearization) */
#define VP_C_SHAPE          FP_CONST(1.9)       /* Pacejka shape factor C */
#define VP_INV_C_SHAPE      FP_CONST(0.526316)  /* 1/1.9 */
#define VP_MIN_STIFF_SCALE  FP_CONST(0.1)       /* Floor for effective stiffness */

/* Precomputed Pacejka B parameters (saves 2 fp_mul per linearization call) */
#define VP_B_FRONT          FP_CONST(1.476)     /* C_Sf / C_shape = 2.804/1.9  */
#define VP_B_REAR           FP_CONST(1.747)     /* C_Sr / C_shape = 3.320/1.9  */
/* Precomputed mu*C_S products (saves 2 fp_mul per linearization) */
#define VP_MU_CSF           FP_CONST(2.092)     /* mu * C_Sf = 0.7463*2.804 */
#define VP_MU_CSR           FP_CONST(2.478)     /* mu * C_Sr = 0.7463*3.320 */

/*===========================================================================
 * MPC Default Cost Weights (tuned for F1/10th)
 *===========================================================================*/

#define MPC_DT              ((fixed_point_t)2621)   /* 0.04s in Q16.16 */

/* Precomputed dt*inv_mass and dt*inv_Iz (eliminates 9 fp_mul per linearization) */
#define VP_DT_INV_MASS      FP_CONST(0.012070)  /* dt * (1/mass) = 0.04 * 0.301750 */
#define VP_DT_INV_IZ        FP_CONST(1.142857)  /* dt * (1/I_z)  = 0.04 * 28.571429 */

#define MPC_W_LAT_ERROR     FP_CONST(340.0)
#define MPC_W_HEADING       FP_CONST(2000.0)     /* cl050 sweep best (was 1000) */
#define MPC_W_VELOCITY      FP_CONST(26.0)
#define MPC_W_LAT_VEL       FP_CONST(100.0)      /* cl050 sweep best (was 69) */
#define MPC_W_YAW_RATE      FP_CONST(22.0)
#define MPC_W_STEER_EFF     FP_CONST(0.15)
#define MPC_W_ACCEL_EFF     FP_CONST(0.01)
#define MPC_W_STEER_JERK    FP_CONST(0.3)
#define MPC_W_ACCEL_RATE    FP_CONST(0.1)        /* cl050 sweep best (was 0.116) */
#define MPC_W_DELTA_ACT     FP_CONST(0.795)      /* cl050 sweep best (was 0.53) */
#define MPC_CROSS_CALL_SCALE FP_CONST(0.125)

/* === Precomputed 2x weights for QP Hessian diagonal ===
 * Eliminates ~24 runtime fp_mul calls in mpc_compute_hls.
 * All computed at compile time via integer arithmetic. */
#define MPC_Q2_LAT_ERROR    ((MPC_W_LAT_ERROR) << 1)     /* 2*340 = 680 */
#define MPC_Q2_HEADING      ((MPC_W_HEADING) << 1)        /* 2*2000 = 4000 */
#define MPC_Q2_VELOCITY     ((MPC_W_VELOCITY) << 1)       /* 2*26 = 52 */
#define MPC_Q2_LAT_VEL      ((MPC_W_LAT_VEL) << 1)        /* 2*100 = 200 */
#define MPC_Q2_YAW_RATE     ((MPC_W_YAW_RATE) << 1)       /* 2*22 = 44 */
#define MPC_Q2_DELTA_ACT    ((MPC_W_DELTA_ACT) << 1)      /* 2*0.795 */
#define MPC_Q2_STEER_JERK   ((MPC_W_STEER_JERK) << 1)     /* 2*0.3 */
#define MPC_Q2_ACCEL_RATE   ((MPC_W_ACCEL_RATE) << 1)     /* 2*0.1 */
#define MPC_R2_STEER        (((MPC_W_STEER_EFF) + (MPC_W_STEER_JERK)) << 1)  /* 2*(0.15+0.3) */
#define MPC_R2_ACCEL        (((MPC_W_ACCEL_EFF) + (MPC_W_ACCEL_RATE)) << 1)   /* 2*(0.01+0.1) */
#define MPC_N2_STEER_JERK   (-((MPC_W_STEER_JERK) << 1))  /* -2*0.3 */
#define MPC_N2_ACCEL_RATE   (-((MPC_W_ACCEL_RATE) << 1))   /* -2*0.1 */
/* Cross-call scaled variants for step 0 (0.125 = >>3) */
#define MPC_Q2_JERK_CS      ((MPC_W_STEER_JERK >> 3) << 1)  /* 2*0.3*0.125 */
#define MPC_Q2_ARATE_CS     ((MPC_W_ACCEL_RATE >> 3) << 1)   /* 2*0.1*0.125 */
#define MPC_R2_STEER_CS     (((MPC_W_STEER_EFF) + (MPC_W_STEER_JERK >> 3)) << 1)
#define MPC_R2_ACCEL_CS     (((MPC_W_ACCEL_EFF) + (MPC_W_ACCEL_RATE >> 3)) << 1)

/*===========================================================================
 * Solver/Constraint Constants
 *===========================================================================*/

#define BIG_BOUND           FP_CONST(100.0)
#define MIN_LIN_VEL         FP_CONST(2.0)
#define STABILITY_LIMIT_VAL FP_CONST(0.95)
/* WALL_MARGIN = 0.36m: vehicle half-width=0.137m + body_safety=0.06m = 0.197m
 * effective body edge.  cl050 raceline has ~0.44m wall clearance in tightest
 * sections.  0.36 keeps constraints feasible while maintaining safety. */
#define WALL_MARGIN         FP_CONST(0.15)   /* updated from CPU sweep (was 0.4) */
#define WALL_START          1
#define WALL_STRIDE         1
#define WALL_END            18     /* updated from CPU sweep (was 10) */
#define V_SWITCH            FP_CONST(7.319)
#define BOUND_THRESHOLD     FP_CONST(100.0)
#define WP_ADVANCE_MAX      10   /* Max waypoint advance per horizon step */

/* ADMM default parameters */
#define ADMM_RHO_DEFAULT    FP_CONST(50.0)
#define ADMM_RHO_U_DEFAULT  FP_CONST(26.6)
#define ADMM_TOL_DEFAULT    FP_CONST(5.0)

/* ADMM steering rate quantization — disabled (ADMM_QUANTIZE_STEER was 0).
 * Kept as comment for reference.
 * #define ADMM_QUANTIZE_STEER         0
 * #define ADMM_STEER_HALF_THRESHOLD   FP_CONST(1.4245)  // VP_MAX_STEER_RATE / 2
 */

/* Over-relaxation parameter (alpha): typical range [1.5, 1.8]
 * Replaces x with alpha*x + (1-alpha)*z_old in z-update.
 * Literature shows 30-50% iteration reduction.
 *
 * DSP-optimised form: x_hat = x + (alpha-1)*(x - z_old)
 * uses 1 multiply instead of 2. */
#define ADMM_OVER_RELAX             FP_CONST(1.5)  /* cl050 sweep best (was 1.2) */
/* ADMM_OVER_RELAX_COMPLEMENT removed — was FP_CONST(-0.2), never used in HLS source.
 * Note: fpga_tune_weights.py also wrote this define; update that script if needed. */
#define ADMM_OVER_RELAX_MINUS1      32768           /* alpha - 1 = 0.5  in Q16.16 */

/*===========================================================================
 * Data Structures
 *===========================================================================*/

/** Waypoint stored in FPGA BRAM */
typedef struct {
    fixed_point_t x;
    fixed_point_t y;
    fixed_point_t psi;
    fixed_point_t vx;
    fixed_point_t kappa;
    fixed_point_t ax;
    fixed_point_t left_bound;
    fixed_point_t right_bound;
} MpcWaypoint_t;

/** Reference point for one MPC prediction step */
typedef struct {
    fixed_point_t velocity;
    fixed_point_t kappa;
    fixed_point_t left_bound;
    fixed_point_t right_bound;
} MpcRefPoint_t;

/** Per-step QP data for Riccati-ADMM */
typedef struct {
    fixed_point_t A[MPC_NX_AUG][MPC_NX_AUG];
    fixed_point_t B[MPC_NX_AUG][MPC_NU];
    fixed_point_t Q_diag[MPC_NX_AUG];
    fixed_point_t q[MPC_NX_AUG];
    fixed_point_t R_diag[MPC_NU];
    fixed_point_t r[MPC_NU];
    fixed_point_t N_cross[MPC_NX_AUG][MPC_NU];
    fixed_point_t x_lb[MPC_NX_AUG];
    fixed_point_t x_ub[MPC_NX_AUG];
    fixed_point_t u_lb[MPC_NU];
    fixed_point_t u_ub[MPC_NU];
} StepData_t;

/** MPC solver status */
typedef enum {
    MPC_STATUS_OPTIMAL  = 0,
    MPC_STATUS_MAX_ITER = 1,
    MPC_STATUS_ERROR    = 2
} MpcStatus_t;

/** ADMM warm-start state (persists between calls) */
typedef struct {
    fixed_point_t z_x[MPC_HORIZON + 1][MPC_NX_AUG];
    fixed_point_t z_u[MPC_HORIZON][MPC_NU];
    fixed_point_t y_x[MPC_HORIZON + 1][MPC_NX_AUG];
    fixed_point_t y_u[MPC_HORIZON][MPC_NU];
    fixed_point_t rho;      /* Persisted adapted rho (OPT-2) */
    fixed_point_t rho_u;    /* Persisted adapted rho_u (OPT-2) */
    int initialized;
} AdmmState_t;

/** Solver solution output */
typedef struct {
    fixed_point_t x[MPC_HORIZON + 1][MPC_NX_AUG];
    fixed_point_t u[MPC_HORIZON][MPC_NU];
    int iterations;
    fixed_point_t primal_residual;
    fixed_point_t dual_residual;
    MpcStatus_t status;
} MpcSolution_t;

/** ADMM configuration */
typedef struct {
    fixed_point_t rho;
    fixed_point_t rho_u;
    fixed_point_t tolerance;
    int max_iterations;
    int adaptive_rho;
} AdmmConfig_t;

/** Persistent MPC state (between calls) */
typedef struct {
    fixed_point_t prev_steer_rate;
    fixed_point_t prev_accel;
    fixed_point_t prev_delta_cmd;
    fixed_point_t actual_steering;
    fixed_point_t prev_curvature;
    int prev_converged;
} MpcPersistState_t;

#endif /* MPC_FPGA_TYPES_H */
