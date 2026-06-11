/**
 * @file mpc_fpga_constants.h
 * @brief Shared constants for the FPGA MPC production path.
 * @details Single source of truth for horizon sizing, DMA framing, vehicle
 *          parameters, timing constants, solver defaults, and cost weights.
 *
 * This header should remain safe for both host and FPGA-side code.
 * Do not place HLS-internal numeric type configuration here.
 */

#ifndef MPC_FPGA_CONSTANTS_H
#define MPC_FPGA_CONSTANTS_H

/*===========================================================================
 * Fixed horizon and DMA framing
 *===========================================================================*/

#ifndef MPC_FPGA_HORIZON_STEPS
#define MPC_FPGA_HORIZON_STEPS 20
#endif

#define MPC_FPGA_HEADER_WORDS_32     8
#define MPC_FPGA_REF_WORDS_PER_STEP  8

/* Transport beat sizing */
#define MPC_FPGA_DMA_BEAT_BYTES_128  16

/* Logical payload size in 32-bit words */
#define MPC_FPGA_INPUT_WORDS_32 (MPC_FPGA_HEADER_WORDS_32 + (MPC_FPGA_HORIZON_STEPS * MPC_FPGA_REF_WORDS_PER_STEP))

/* Logical payload size expressed as 128-bit beats and bytes */
#define MPC_FPGA_DMA_BEATS_128 ((MPC_FPGA_INPUT_WORDS_32 + 3) / 4)

#define MPC_FPGA_DMA_BYTES (MPC_FPGA_INPUT_WORDS_32 * 4)

/*===========================================================================
 * IO numeric format constants
 *===========================================================================*/

/* Protocol numeric lanes are raw QP words: Q12.14.
 * Raw values are sign-extended 26-bit fixed-point carried in int32_t lanes.
 * The DMA lane count and packed payload layout are unchanged; only the
 * fixed-point interpretation of each lane changed from Q14.18 to Q12.14. */
#define MPC_FPGA_QP_WIDTH       26
#define MPC_FPGA_QP_INT_BITS    12
#define MPC_FPGA_QP_FRAC_BITS   (MPC_FPGA_QP_WIDTH - MPC_FPGA_QP_INT_BITS)
#define MPC_FPGA_QP_SCALE_I32   (1 << MPC_FPGA_QP_FRAC_BITS)
#define MPC_FPGA_QP_SCALE_F32   ((float)MPC_FPGA_QP_SCALE_I32)
#define MPC_FPGA_QP_SCALE_F64   ((double)MPC_FPGA_QP_SCALE_I32)

/*===========================================================================
 * Vehicle and tire constants (SI)
 *===========================================================================*/

#define MPC_FPGA_LF_M                 0.166f
#define MPC_FPGA_LR_M                 0.160f
#define MPC_FPGA_WHEELBASE_M          (MPC_FPGA_LF_M + MPC_FPGA_LR_M)
#define MPC_FPGA_MASS_KG              3.314f
#define MPC_FPGA_IZ_KGM2              0.035f
#define MPC_FPGA_CG_HEIGHT_M          0.0703f
#define MPC_FPGA_GRAVITY_MS2          9.81f
#define MPC_FPGA_MU                   0.72f

#define MPC_FPGA_MAX_STEER_RAD        0.39f
#define MPC_FPGA_MAX_STEER_RATE_RADPS 2.849f
#define MPC_FPGA_MAX_VEL_MPS          20.0f
#define MPC_FPGA_MIN_VEL_MPS          0.5f

#define MPC_FPGA_C_ALPHA_F_N_PER_RAD  51.40f
#define MPC_FPGA_C_ALPHA_R_N_PER_RAD  43.10f

/*===========================================================================
 * Timing constants
 *===========================================================================*/

#define MPC_FPGA_CONTROL_RATE_HZ      200.0f
#define MPC_FPGA_PREDICTION_DT_S      0.03f

#define MPC_FPGA_QP_HALF_LSB          (0.5 / MPC_FPGA_QP_SCALE_F64)
#define MPC_FPGA_CONTROL_DT_S         (1.0 / MPC_FPGA_CONTROL_RATE_HZ)
#define MPC_FPGA_CROSS_CALL_SCALE     ((MPC_FPGA_CONTROL_DT_S / MPC_FPGA_PREDICTION_DT_S) + MPC_FPGA_QP_HALF_LSB)

/*===========================================================================
 * Derived fixed vehicle constants
 *===========================================================================*/

#define MPC_FPGA_PACEJKA_C_SHAPE      1.9f
#define MPC_FPGA_MIN_STIFF_SCALE      0.1f

#define MPC_FPGA_INV_WHEELBASE        (1.0 / MPC_FPGA_WHEELBASE_M)
#define MPC_FPGA_INV_MASS             (1.0 / MPC_FPGA_MASS_KG)
#define MPC_FPGA_INV_IZ               (1.0 / MPC_FPGA_IZ_KGM2)
#define MPC_FPGA_INV_PACEJKA_C_SHAPE  (1.0 / MPC_FPGA_PACEJKA_C_SHAPE)

#define MPC_FPGA_FZ_FRONT_N ((MPC_FPGA_MASS_KG * MPC_FPGA_GRAVITY_MS2 * MPC_FPGA_LR_M) / MPC_FPGA_WHEELBASE_M)

#define MPC_FPGA_FZ_REAR_N ((MPC_FPGA_MASS_KG * MPC_FPGA_GRAVITY_MS2 * MPC_FPGA_LF_M) / MPC_FPGA_WHEELBASE_M)

#define MPC_FPGA_D_FRONT_N            (MPC_FPGA_MU * MPC_FPGA_FZ_FRONT_N)
#define MPC_FPGA_D_REAR_N             (MPC_FPGA_MU * MPC_FPGA_FZ_REAR_N)

#define MPC_FPGA_C_ALPHA_SF_NORM      (MPC_FPGA_C_ALPHA_F_N_PER_RAD / MPC_FPGA_D_FRONT_N)
#define MPC_FPGA_C_ALPHA_SR_NORM      (MPC_FPGA_C_ALPHA_R_N_PER_RAD / MPC_FPGA_D_REAR_N)
#define MPC_FPGA_B_FRONT              (MPC_FPGA_C_ALPHA_SF_NORM * MPC_FPGA_INV_PACEJKA_C_SHAPE)
#define MPC_FPGA_B_REAR               (MPC_FPGA_C_ALPHA_SR_NORM * MPC_FPGA_INV_PACEJKA_C_SHAPE)

/*===========================================================================
 * Cross-call scaled default rate weights
 *===========================================================================*/

#define MPC_FPGA_W_STEER_JERK_CS ((MPC_FPGA_W_STEER_JERK * MPC_FPGA_CROSS_CALL_SCALE) + MPC_FPGA_QP_HALF_LSB)

#define MPC_FPGA_W_ACCEL_RATE_CS ((MPC_FPGA_W_ACCEL_RATE * MPC_FPGA_CROSS_CALL_SCALE) + MPC_FPGA_QP_HALF_LSB)

/*===========================================================================
 * Communication/runtime defaults
 *===========================================================================*/

#define MPC_FPGA_RECEIVER_LOG_PERIOD_MSGS     100

#define MPC_FPGA_DMA_RESET_TIMEOUT_CYCLES     10000
#define MPC_FPGA_DMA_TRANSFER_TIMEOUT_CYCLES  100000
#define MPC_FPGA_MPC_DONE_TIMEOUT_CYCLES      200000
#define MPC_FPGA_OUTPUT_VALID_TIMEOUT_CYCLES  50000

#define MPC_FPGA_RAD_TO_DEG                   57.2957795f

#define MPC_FPGA_PUBLISHER_FORWARD_LOOKAHEAD  3
#define MPC_FPGA_PUBLISHER_ODOM_WATCHDOG_MS   500
#define MPC_FPGA_PUBLISHER_DEBUG_LOG_PERIOD   50

#define MPC_FPGA_BRIDGE_POLL_PERIOD_US        50
#define MPC_FPGA_BRIDGE_STATS_START_SPEED_MPS 0.5f

#define MPC_FPGA_SERVO_GAIN                   -0.7284f
#define MPC_FPGA_SERVO_OFFSET                  0.55f
#define MPC_FPGA_STEER_CORRECTION_C2           0.589566f
#define MPC_FPGA_STEER_CORRECTION_C1           0.918061f
#define MPC_FPGA_STEER_CORRECTION_C0           0.001490f

/*===========================================================================
 * Solver structure and defaults
 *===========================================================================*/

#define MPC_FPGA_MAX_ADMM_ITER        50

/* Q12.14 objective/ADMM-penalty scaling.
 * The whole QP objective (cost weights) and the ADMM penalty family (rho,
 * rho_u, adaptive rho bounds) are divided by this factor. Scaling the entire
 * objective by a constant does not change the QP minimizer or the ADMM
 * penalty proportions; it only shrinks the stored magnitudes so they fit the
 * Q12.14 (12 integer bits) range. Do NOT scale physical quantities (states,
 * controls, bounds, vehicle/tire constants, curvature) by this factor. */
#ifndef MPC_FPGA_NUMERIC_SCALE_DIV
#define MPC_FPGA_NUMERIC_SCALE_DIV 4.0f
#endif

#define MPC_FPGA_SCALED(x) ((x) / MPC_FPGA_NUMERIC_SCALE_DIV)

/* rho/rho_u carry the SAME /4 scaling as the objective weights so the
 * rho/weight ratio (which governs ADMM convergence + the bias of an
 * early-exit solution) is preserved from the original 7.0. A larger rho here
 * (e.g. the 77/11 that was tried) over-penalizes vs the /4-scaled weights and,
 * once the tolerance lets ADMM exit early, lands at a biased point that tracks
 * the CPU MPC poorly on harder states (frenet_parity dropped to 56% within
 * 1deg at rho=77/4 vs 94% at 7/4). Keep the original ratio. */
#ifndef MPC_FPGA_ADMM_RHO
#define MPC_FPGA_ADMM_RHO             (7.0f / MPC_FPGA_NUMERIC_SCALE_DIV)
#endif

#ifndef MPC_FPGA_ADMM_RHO_U
#define MPC_FPGA_ADMM_RHO_U           (7.0f / MPC_FPGA_NUMERIC_SCALE_DIV)
#endif

#ifndef MPC_FPGA_ADMM_RHO_MIN
#define MPC_FPGA_ADMM_RHO_MIN         (1.0f / MPC_FPGA_NUMERIC_SCALE_DIV)
#endif

#ifndef MPC_FPGA_ADMM_RHO_MAX
#define MPC_FPGA_ADMM_RHO_MAX         (80.0f / MPC_FPGA_NUMERIC_SCALE_DIV)
#endif

/* Absolute ADMM convergence tolerance. This is a BIT-WIDTH-DEPENDENT quantity:
 * the residual is computed in QP, so its roundoff floor scales with the QP LSB.
 * At Q14.18 the working value was 0.01; Q12.14 has a 16x coarser LSB, so the
 * solver could reach the optimum but never detect it (residual stuck above the
 * old 0.01), burning all MAX_ADMM_ITER iterations. 0.20 sits above the Q12.14
 * floor: replay over FPGA_UDP/MPC_10Laps/frenet drops mean iters from ~27 (54%
 * hitting max-iter) to ~1-1.5 (0% at max), and tracks the CPU MPC as well or
 * better (running past the fixed-point floor was drifting, not refining). Raise
 * further if you lower QP/family frac bits (higher LSB -> higher floor). */
#ifndef MPC_FPGA_ADMM_TOL
#define MPC_FPGA_ADMM_TOL             0.05f
#endif

/* Relative ADMM convergence tolerance. Replay showed the relative tolerance
 * (not max-iteration count) is the correct lever for residual stalls.
 * Joint abs+rel sweep (sequential scalar replay, 130k real-bag steps vs float
 * CPU): tightening BOTH from 0.20/0.10 -> 0.05/0.05 lifts within-1deg 87.6->90.0%
 * and RMS 0.82->0.70deg with ~zero added railing (0.02% non-converged, meanIt
 * 1.28->1.73). abs_tol is the additive floor (eps=abs+rel*norm), so rel alone
 * does nothing -- they must move together. The Q12.14 noise floor is ~0.03:
 * 0.03/0.03 peaks at 92.6% but 7% non-converge, and 0.02/0.02 DEGRADES (RMS
 * worse) -- below ~0.03 the solver can't reach the residual and rails. 0.05 is
 * the robust no-regret point. NOTE: validate on HW via live iteration_count --
 * the offline scalar replay's Frenet/input build differs from the synth top,
 * and tb_top_min is single-row-only (rails artificially over many rows). */
#ifndef MPC_FPGA_ADMM_REL_TOL
#define MPC_FPGA_ADMM_REL_TOL         0.05f
#endif

/*===========================================================================
 * MPC cost weights (scaled by MPC_FPGA_NUMERIC_SCALE_DIV for Q12.14)
 *===========================================================================*/

#define MPC_FPGA_W_LAT_ERROR          MPC_FPGA_SCALED(2000.0f)
#define MPC_FPGA_W_HEADING            MPC_FPGA_SCALED(50.0f)
#define MPC_FPGA_W_VELOCITY           MPC_FPGA_SCALED(250.0f)
#define MPC_FPGA_W_LAT_VEL            MPC_FPGA_SCALED(5.0f)
#define MPC_FPGA_W_YAW_RATE           MPC_FPGA_SCALED(1.5f)
#define MPC_FPGA_W_STEER_EFF          MPC_FPGA_SCALED(2.0f)
#define MPC_FPGA_W_ACCEL_EFF          MPC_FPGA_SCALED(0.5f)
#define MPC_FPGA_W_STEER_JERK         MPC_FPGA_SCALED(5.0f)
#define MPC_FPGA_W_ACCEL_RATE         MPC_FPGA_SCALED(5.0f)
#define MPC_FPGA_W_DELTA_ACT          MPC_FPGA_SCALED(1.0f)

/*===========================================================================
 * Solver and constraint limits
 *===========================================================================*/

#define MPC_FPGA_BIG_BOUND            50.0f
#define MPC_FPGA_MIN_LIN_VEL_MPS      0.5f
#define MPC_FPGA_STABILITY_LIMIT      0.95f
#define MPC_FPGA_WALL_MARGIN_M        0.2f
#define MPC_FPGA_WALL_BIAS_CLEAR_M    0.05f
#define MPC_FPGA_WALL_BIAS_MAX_M      0.2f
/* Min standoff of the tracked e_y reference from each corridor edge.
 * Mirrors CPU MPC_WALL_REF_CLEAR_M default; keep both in sync for parity. */
#define MPC_FPGA_WALL_REF_CLEAR_M     0.10f
#define MPC_FPGA_WALL_BOUND_WINDOW    3
#define MPC_FPGA_V_SWITCH_MPS         7.319f
#define MPC_FPGA_BOUND_THRESHOLD      50.0f

#endif /* MPC_FPGA_CONSTANTS_H */
