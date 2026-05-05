/**
 * @file mpc_fpga_interface.h
 * @brief OpenCL transport contract constants for MPC Riccati-ADMM.
 * @details Defines payload framing, horizon sizing, and solver status values
 *          shared by host OpenCL launchers and FPGA kernel wrappers.
 * @dependencies <stdint.h>, mpc_fpga_constants.h
 *
 * OpenCL memory payload format (logical 32-bit words):
 *   Beat 0: [e_y | e_psi | vx | vy]
 *   Beat 1: [omega | steering | horizon_length | prev_accel]
 *   Words 8..: repeating per-step groups (8 words per step in V2)
 *      [ref_ey[i] | ref_epsi[i] | ref_vx[i] | ref_vy[i] |
 *       ref_omega_ref[i] | ref_kappa[i] | ref_left[i] | ref_right[i]]
 *
 * Data format: state/reference numeric lanes are Q16.16 fixed-point (int32_t).
 * Control/metadata lanes (horizon_length, status, iterations) are plain int32/uint32.
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
 * Status Codes
 *===========================================================================*/

#define MPC_FPGA_STATUS_OK              0   /* Optimal solution found */
#define MPC_FPGA_STATUS_MAX_ITER        1   /* Hit iteration limit */
#define MPC_FPGA_STATUS_ERROR           2   /* Solver error */
#define MPC_FPGA_STATUS_NO_TRAJECTORY   3   /* No valid trajectory */

#endif /* MPC_FPGA_INTERFACE_H */
