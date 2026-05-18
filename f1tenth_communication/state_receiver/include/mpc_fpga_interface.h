/**
 * @file mpc_fpga_interface.h
 * @brief OpenCL transport contract constants for MPC Riccati-ADMM.
 * @details Defines payload framing, control flags, and solver status values
 *          shared by host OpenCL launchers and FPGA kernel wrappers.
 * @dependencies <stdint.h>, mpc_fpga_constants.h
 *
 * OpenCL memory payload format (logical 32-bit words):
 *   Beat 0: [e_y | e_psi | vx | vy]
 *   Beat 1: [omega | steering | control_flags | prev_accel]
 *   Words 8..: repeating per-step groups (8 words per step in V2)
 *      [ref_ey[i] | ref_epsi[i] | ref_vx[i] | ref_vy[i] |
 *       ref_omega_ref[i] | ref_kappa[i] | ref_left[i] | ref_right[i]]
 *
 * Data format: state/reference numeric lanes are raw QP fixed-point words.
 * Control/metadata lanes (control_flags, status, iterations) are plain int32/uint32.
 */

#ifndef MPC_FPGA_INTERFACE_H
#define MPC_FPGA_INTERFACE_H

#include <stdint.h>
#include "mpc_fpga_constants.h"

/*===========================================================================
 * MPC Configuration
 *===========================================================================*/

#ifdef MPC_HORIZON
#if MPC_HORIZON != MPC_FPGA_HORIZON_STEPS
#error "MPC_HORIZON must match MPC_FPGA_HORIZON_STEPS"
#endif
#else
#define MPC_HORIZON             MPC_FPGA_HORIZON_STEPS      /* Prediction horizon (steps) */
#endif
#define MPC_FPGA_HEADER_WORDS   MPC_FPGA_HEADER_WORDS_32
#define MPC_FPGA_REF_WORDS      MPC_FPGA_REF_WORDS_PER_STEP
#define INPUT_BUFFER_WORDS_32   MPC_FPGA_INPUT_WORDS_32     /* Logical 32-bit payload words */
#define DMA_BUFFER_BYTES        MPC_FPGA_DMA_BYTES          /* Unpadded bytes in logical payload words */
#define DMA_BUFFER_BEATS        MPC_FPGA_DMA_BEATS          /* 128-bit beats (4 x 32-bit words) for logical payload */
#define INPUT_BUFFER_WORDS_512  ((INPUT_BUFFER_WORDS_32 + 15) / 16) /* Packed 512-bit transport words */
#define INPUT_BUFFER_WORDS_32_PAD (INPUT_BUFFER_WORDS_512 * 16) /* 32-bit words incl. 512-bit padding */
#define INPUT_BUFFER_BYTES_512  (INPUT_BUFFER_WORDS_512 * 64)   /* Input bytes required for 512-bit m_axi */

/*===========================================================================
 * Header word indices
 *===========================================================================*/

#define MPC_FPGA_WORD_EY              0
#define MPC_FPGA_WORD_EPSI            1
#define MPC_FPGA_WORD_VX              2
#define MPC_FPGA_WORD_VY              3
#define MPC_FPGA_WORD_OMEGA           4
#define MPC_FPGA_WORD_STEERING        5
#define MPC_FPGA_WORD_CONTROL_FLAGS   6
#define MPC_FPGA_WORD_PREV_ACCEL      7

/*===========================================================================
 * Control flags
 *===========================================================================*/

#define MPC_FPGA_CTRL_FLAGS_NONE            0u
#define MPC_FPGA_CTRL_FLAG_RESET_STATE      (1u << 0)
#define MPC_FPGA_CTRL_FLAG_FORCE_COLD_START (1u << 1)
#define MPC_FPGA_CTRL_FLAG_ZERO_DUALS       (1u << 2)
#define MPC_FPGA_CTRL_FLAG_DEBUG_ECHO_INPUTS (1u << 3)

/*===========================================================================
 * Status Codes
 *===========================================================================*/

#define MPC_FPGA_STATUS_OK              0   /* Optimal solution found */
#define MPC_FPGA_STATUS_MAX_ITER        1   /* Hit iteration limit */
#define MPC_FPGA_STATUS_ERROR           2   /* Solver error */
#define MPC_FPGA_STATUS_NO_TRAJECTORY   3   /* No valid trajectory */

#endif /* MPC_FPGA_INTERFACE_H */
