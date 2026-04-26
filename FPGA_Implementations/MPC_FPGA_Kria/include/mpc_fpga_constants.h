/**
 * @file mpc_fpga_constants.h
 * @brief Shared constants for the FPGA MPC production path.
 * @details Single source of truth for horizon sizing, DMA framing, numeric
 *          scales, vehicle constants, and solver defaults that must remain
 *          aligned across the FPGA implementation and host-side interfaces.
 * @dependencies None
 */

#ifndef MPC_FPGA_CONSTANTS_H
#define MPC_FPGA_CONSTANTS_H

/*===========================================================================
 * Fixed Horizon and DMA Framing
 *===========================================================================*/

#ifndef MPC_FPGA_HORIZON_STEPS
#define MPC_FPGA_HORIZON_STEPS        20
#endif
#define MPC_FPGA_STATE_BEATS          2
#define MPC_FPGA_STREAM_WORD_BYTES    16
#define MPC_FPGA_DMA_BEATS            (MPC_FPGA_STATE_BEATS + MPC_FPGA_HORIZON_STEPS)
#define MPC_FPGA_DMA_BYTES            (MPC_FPGA_DMA_BEATS * MPC_FPGA_STREAM_WORD_BYTES)

/*===========================================================================
 * Numeric Format Constants
 *===========================================================================*/

/* IO boundary: Fixed Q16.16 for protocol. */
#define MPC_FPGA_Q16_SCALE_I32        65536
#define MPC_FPGA_Q16_SCALE_F32        65536.0f
#define MPC_FPGA_Q16_SCALE_F64        65536.0

/* Internal Riccati domain: Parameterized by MPC_HLS_RICCATI_INT_BITS.
   Enables width/precision optimization sweeps independent of I/O format. */
#define MPC_FPGA_RICCATI_SCALE_I32    (1 << MPC_HLS_RICCATI_INT_BITS)
#define MPC_FPGA_RICCATI_SCALE_F32   ((float)(1 << MPC_HLS_RICCATI_INT_BITS))
#define MPC_FPGA_RICCATI_SCALE_F64   ((double)(1 << MPC_HLS_RICCATI_INT_BITS))

/*===========================================================================
 * Vehicle and Tire Constants (SI)
 *===========================================================================*/

#define MPC_FPGA_LF_M                 0.166f
#define MPC_FPGA_LR_M                 0.160f
#define MPC_FPGA_WHEELBASE_M          (MPC_FPGA_LF_M + MPC_FPGA_LR_M)
#define MPC_FPGA_MASS_KG              3.314f
#define MPC_FPGA_IZ_KGM2              0.035f
#define MPC_FPGA_CG_HEIGHT_M          0.0703f
#define MPC_FPGA_GRAVITY_MS2          9.81f
#define MPC_FPGA_MU                   0.745f

#define MPC_FPGA_MAX_STEER_RAD        0.4189f
#define MPC_FPGA_MAX_STEER_RATE_RADPS 2.849f
#define MPC_FPGA_MAX_VEL_MPS          20.0f
#define MPC_FPGA_MIN_VEL_MPS          0.0f

#define MPC_FPGA_C_ALPHA_F_N_PER_RAD  51.40f
#define MPC_FPGA_C_ALPHA_R_N_PER_RAD  43.10f

/*===========================================================================
 * Timing Constants
 *===========================================================================*/

#define MPC_FPGA_CONTROL_RATE_HZ      200.0f

/* Derived fixed timing constants used in synthesized builds.
 * Keep these as dependency-linked compile-time expressions.
 * The +Q16 half-LSB term counteracts truncation when converted to ap_fixed. */
#define MPC_FPGA_Q16_HALF_LSB         (0.5 / MPC_FPGA_Q16_SCALE_F64)
#define MPC_FPGA_CONTROL_DT_S         (1.0 / MPC_FPGA_CONTROL_RATE_HZ)
#define MPC_FPGA_CROSS_CALL_SCALE     ((MPC_FPGA_CONTROL_DT_S / MPC_FPGA_PREDICTION_DT_S) + MPC_FPGA_Q16_HALF_LSB)

/* Cross-call scaled default rate weights (control_dt / prediction_dt). */
#define MPC_FPGA_W_STEER_JERK_CS      ((MPC_FPGA_W_STEER_JERK * MPC_FPGA_CROSS_CALL_SCALE) + MPC_FPGA_Q16_HALF_LSB)
#define MPC_FPGA_W_ACCEL_RATE_CS      ((MPC_FPGA_W_ACCEL_RATE * MPC_FPGA_CROSS_CALL_SCALE) + MPC_FPGA_Q16_HALF_LSB)

/* Precomputed derived vehicle constants for fixed-point compile paths. */
#define MPC_FPGA_INV_WHEELBASE        (1.0 / MPC_FPGA_WHEELBASE_M)
#define MPC_FPGA_INV_MASS             (1.0 / MPC_FPGA_MASS_KG)
#define MPC_FPGA_INV_IZ               (1.0 / MPC_FPGA_IZ_KGM2)
#define MPC_FPGA_INV_PACEJKA_C_SHAPE  (1.0 / MPC_FPGA_PACEJKA_C_SHAPE)

#define MPC_FPGA_FZ_FRONT_N           ((MPC_FPGA_MASS_KG * MPC_FPGA_GRAVITY_MS2 * MPC_FPGA_LR_M) / MPC_FPGA_WHEELBASE_M)
#define MPC_FPGA_FZ_REAR_N            ((MPC_FPGA_MASS_KG * MPC_FPGA_GRAVITY_MS2 * MPC_FPGA_LF_M) / MPC_FPGA_WHEELBASE_M)
#define MPC_FPGA_D_FRONT_N            (MPC_FPGA_MU * MPC_FPGA_FZ_FRONT_N)
#define MPC_FPGA_D_REAR_N             (MPC_FPGA_MU * MPC_FPGA_FZ_REAR_N)

#define MPC_FPGA_C_ALPHA_SF_NORM      (MPC_FPGA_C_ALPHA_F_N_PER_RAD / MPC_FPGA_D_FRONT_N)
#define MPC_FPGA_C_ALPHA_SR_NORM      (MPC_FPGA_C_ALPHA_R_N_PER_RAD / MPC_FPGA_D_REAR_N)
#define MPC_FPGA_B_FRONT              (MPC_FPGA_C_ALPHA_SF_NORM * MPC_FPGA_INV_PACEJKA_C_SHAPE)
#define MPC_FPGA_B_REAR               (MPC_FPGA_C_ALPHA_SR_NORM * MPC_FPGA_INV_PACEJKA_C_SHAPE)

/*===========================================================================
 * Communication Runtime Defaults (Jetson/Kria)
 *===========================================================================*/

#define MPC_FPGA_RECEIVER_LOG_PERIOD_MSGS    100

#define MPC_FPGA_DMA_RESET_TIMEOUT_CYCLES    10000
#define MPC_FPGA_DMA_TRANSFER_TIMEOUT_CYCLES 100000
#define MPC_FPGA_MPC_DONE_TIMEOUT_CYCLES     200000
#define MPC_FPGA_OUTPUT_VALID_TIMEOUT_CYCLES 50000

#define MPC_FPGA_RAD_TO_DEG                  57.2957795f

#define MPC_FPGA_PUBLISHER_FORWARD_LOOKAHEAD 3
#define MPC_FPGA_PUBLISHER_ODOM_WATCHDOG_MS  500
#define MPC_FPGA_PUBLISHER_DEBUG_LOG_PERIOD  50

#define MPC_FPGA_BRIDGE_POLL_PERIOD_US        50
#define MPC_FPGA_BRIDGE_STATS_START_SPEED_MPS 0.5f

#define MPC_FPGA_SERVO_GAIN                 -0.7284f
#define MPC_FPGA_SERVO_OFFSET                0.55f
#define MPC_FPGA_STEER_CORRECTION_C2         0.589566f
#define MPC_FPGA_STEER_CORRECTION_C1         0.918061f
#define MPC_FPGA_STEER_CORRECTION_C0         0.001490f

/*===========================================================================
 * Solver Structure and Model Constants
 *===========================================================================*/

/** Maximum ADMM iterations. Fixed project-wide for stable timing/behavior comparisons. */
#define MPC_FPGA_MAX_ADMM_ITER        20
#define MPC_FPGA_PREDICTION_DT_S      0.03f
#define MPC_FPGA_PACEJKA_C_SHAPE      1.9f
#define MPC_FPGA_MIN_STIFF_SCALE      0.1f

/*===========================================================================
 * MPC Cost Weights (FPGA profile)
 *===========================================================================*/

#define MPC_FPGA_W_LAT_ERROR             16000.0f
#define MPC_FPGA_W_HEADING               248.864f
#define MPC_FPGA_W_VELOCITY              60.48f
#define MPC_FPGA_W_LAT_VEL               6.75f
#define MPC_FPGA_W_YAW_RATE              1.28f
#define MPC_FPGA_W_STEER_EFF             1.815f
#define MPC_FPGA_W_ACCEL_EFF             0.007276f
#define MPC_FPGA_W_STEER_JERK            0.05115f
#define MPC_FPGA_W_ACCEL_RATE            0.1387f
#define MPC_FPGA_W_DELTA_ACT             0.019f

/*===========================================================================
 * Solver and Constraint Limits (FPGA profile)
 *===========================================================================*/

#define MPC_FPGA_BIG_BOUND            50.0f
#define MPC_FPGA_MIN_LIN_VEL_MPS      0.5f
#define MPC_FPGA_STABILITY_LIMIT      0.95f
#define MPC_FPGA_WALL_MARGIN_M        0.14f
#define MPC_FPGA_WALL_BIAS_CLEAR_M    0.01f
#define MPC_FPGA_WALL_BIAS_MAX_M      0.30f
#define MPC_FPGA_WALL_BOUND_WINDOW    3
#define MPC_FPGA_V_SWITCH_MPS         7.319f
#define MPC_FPGA_BOUND_THRESHOLD      50.0f

#ifndef MPC_FPGA_ADMM_RHO
#define MPC_FPGA_ADMM_RHO                28.0f
#endif

#ifndef MPC_FPGA_ADMM_RHO_U
#define MPC_FPGA_ADMM_RHO_U              42.0f
#endif

#ifndef MPC_FPGA_ADMM_TOL
#define MPC_FPGA_ADMM_TOL                0.05f
#endif

#endif /* MPC_FPGA_CONSTANTS_H */
