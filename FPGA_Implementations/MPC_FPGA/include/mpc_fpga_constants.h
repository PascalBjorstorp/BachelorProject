/**
 * @file mpc_fpga_constants.h
 * @brief Shared constants for the FPGA MPC production path.
 *
 * This header is the single source of truth for constants that must stay
 * consistent across FPGA implementation code and communication nodes.
 */

#ifndef MPC_FPGA_CONSTANTS_H
#define MPC_FPGA_CONSTANTS_H

/*===========================================================================
 * Fixed Horizon and DMA Framing
 *===========================================================================*/

#define MPC_FPGA_HORIZON_STEPS        10
#define MPC_FPGA_STATE_BEATS          2
#define MPC_FPGA_STREAM_WORD_BYTES    16
#define MPC_FPGA_DMA_BEATS            (MPC_FPGA_STATE_BEATS + MPC_FPGA_HORIZON_STEPS)
#define MPC_FPGA_DMA_BYTES            (MPC_FPGA_DMA_BEATS * MPC_FPGA_STREAM_WORD_BYTES)

/*===========================================================================
 * Vehicle and Tire Constants (SI)
 *===========================================================================*/

#define MPC_FPGA_WHEELBASE_M          0.324f
#define MPC_FPGA_LF_M                 0.166f
#define MPC_FPGA_LR_M                 0.160f
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

#endif /* MPC_FPGA_CONSTANTS_H */
