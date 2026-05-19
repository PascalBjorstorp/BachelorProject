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
#define MPC_FPGA_CTRL_FLAG_DEBUG_READ_STATE  (1u << 4)
#define MPC_FPGA_CTRL_FLAG_DEBUG_READ_TRACE  (1u << 5)
#define MPC_FPGA_CTRL_FLAG_DEBUG_SOLVE_STATE (1u << 6)
#define MPC_FPGA_CTRL_FLAG_DEBUG_SOLVE_TRACE (1u << 7)
#define MPC_FPGA_CTRL_DEBUG_WORD_BASE_SHIFT  8u
#define MPC_FPGA_CTRL_DEBUG_WORD_BASE_MASK   (0xffffu << MPC_FPGA_CTRL_DEBUG_WORD_BASE_SHIFT)
#define MPC_FPGA_CTRL_DEBUG_WORD_BASE(base)  ((((uint32_t)(base)) << MPC_FPGA_CTRL_DEBUG_WORD_BASE_SHIFT) & MPC_FPGA_CTRL_DEBUG_WORD_BASE_MASK)
#define MPC_FPGA_CTRL_GET_DEBUG_WORD_BASE(flags) (((uint32_t)(flags) & MPC_FPGA_CTRL_DEBUG_WORD_BASE_MASK) >> MPC_FPGA_CTRL_DEBUG_WORD_BASE_SHIFT)

/*===========================================================================
 * Status Codes
 *===========================================================================*/

#define MPC_FPGA_STATUS_OK              0   /* Optimal solution found */
#define MPC_FPGA_STATUS_MAX_ITER        1   /* Hit iteration limit */
#define MPC_FPGA_STATUS_ERROR           2   /* Solver error */
#define MPC_FPGA_STATUS_NO_TRAJECTORY   3   /* No valid trajectory */

/*===========================================================================
 * Debug Dump Layout
 *===========================================================================*/

#define MPC_FPGA_DEBUG_MAGIC                0x4d504344u /* "MPCD" */
#define MPC_FPGA_DEBUG_VERSION              2u
#define MPC_FPGA_DEBUG_NX_AUG               8u
#define MPC_FPGA_DEBUG_NU                   2u
#define MPC_FPGA_DEBUG_STATE_WORDS          32u
#define MPC_FPGA_DEBUG_ZX_WORDS             (((MPC_FPGA_HORIZON_STEPS) + 1u) * MPC_FPGA_DEBUG_NX_AUG)
#define MPC_FPGA_DEBUG_ZU_WORDS             ((MPC_FPGA_HORIZON_STEPS) * MPC_FPGA_DEBUG_NU)
#define MPC_FPGA_DEBUG_YX_WORDS             MPC_FPGA_DEBUG_ZX_WORDS
#define MPC_FPGA_DEBUG_YU_WORDS             MPC_FPGA_DEBUG_ZU_WORDS
#define MPC_FPGA_DEBUG_SNAPSHOT_WORDS       (MPC_FPGA_DEBUG_STATE_WORDS + MPC_FPGA_DEBUG_ZX_WORDS + MPC_FPGA_DEBUG_ZU_WORDS + MPC_FPGA_DEBUG_YX_WORDS + MPC_FPGA_DEBUG_YU_WORDS)
#define MPC_FPGA_DEBUG_TRACE_MAX            256u
#define MPC_FPGA_DEBUG_TRACE_FIELDS         17u
#define MPC_FPGA_DEBUG_TRACE_HEADER_WORDS   4u
#define MPC_FPGA_DEBUG_TRACE_WORD_CAPACITY  (MPC_FPGA_DEBUG_TRACE_HEADER_WORDS + (MPC_FPGA_DEBUG_TRACE_MAX * MPC_FPGA_DEBUG_TRACE_FIELDS))
#define MPC_FPGA_DEBUG_SUMMARY_SLOTS        4u
#define MPC_FPGA_DEBUG_SUMMARY_FIELDS       15u
#define MPC_FPGA_DEBUG_SUMMARY_HEADER_WORDS 4u
#define MPC_FPGA_DEBUG_SUMMARY_WORD_CAPACITY (MPC_FPGA_DEBUG_SUMMARY_HEADER_WORDS + (MPC_FPGA_DEBUG_SUMMARY_SLOTS * MPC_FPGA_DEBUG_SUMMARY_FIELDS))
#define MPC_FPGA_DEBUG_HEADER_WORDS         24u
#define MPC_FPGA_DEBUG_RESULT_WORDS         4u
#define MPC_FPGA_DEBUG_INPUT_WORDS          INPUT_BUFFER_WORDS_32_PAD
#define MPC_FPGA_DEBUG_RESULT_OFFSET        MPC_FPGA_DEBUG_HEADER_WORDS
#define MPC_FPGA_DEBUG_INPUT_OFFSET         (MPC_FPGA_DEBUG_RESULT_OFFSET + MPC_FPGA_DEBUG_RESULT_WORDS)
#define MPC_FPGA_DEBUG_ENTRY_OFFSET         (MPC_FPGA_DEBUG_INPUT_OFFSET + MPC_FPGA_DEBUG_INPUT_WORDS)
#define MPC_FPGA_DEBUG_SOLVE_ENTRY_OFFSET   (MPC_FPGA_DEBUG_ENTRY_OFFSET + MPC_FPGA_DEBUG_SNAPSHOT_WORDS)
#define MPC_FPGA_DEBUG_POST_OFFSET          (MPC_FPGA_DEBUG_SOLVE_ENTRY_OFFSET + MPC_FPGA_DEBUG_SNAPSHOT_WORDS)
#define MPC_FPGA_DEBUG_TRACE_OFFSET         (MPC_FPGA_DEBUG_POST_OFFSET + MPC_FPGA_DEBUG_SNAPSHOT_WORDS)
#define MPC_FPGA_DEBUG_SUMMARY_OFFSET       (MPC_FPGA_DEBUG_TRACE_OFFSET + MPC_FPGA_DEBUG_TRACE_WORD_CAPACITY)
#define MPC_FPGA_DEBUG_TOTAL_WORDS          (MPC_FPGA_DEBUG_SUMMARY_OFFSET + MPC_FPGA_DEBUG_SUMMARY_WORD_CAPACITY)
#define MPC_FPGA_DEBUG_TOTAL_BEATS          ((MPC_FPGA_DEBUG_TOTAL_WORDS + 3u) / 4u)

#define MPC_FPGA_DEBUG_HDR_MAGIC              0
#define MPC_FPGA_DEBUG_HDR_VERSION            1
#define MPC_FPGA_DEBUG_HDR_CONTROL_FLAGS      2
#define MPC_FPGA_DEBUG_HDR_OUTPUT_STATUS      3
#define MPC_FPGA_DEBUG_HDR_OUTPUT_ITERS       4
#define MPC_FPGA_DEBUG_HDR_OUTPUT_STEER       5
#define MPC_FPGA_DEBUG_HDR_OUTPUT_ACCEL       6
#define MPC_FPGA_DEBUG_HDR_SOLVER_STATUS_RAW  7
#define MPC_FPGA_DEBUG_HDR_EY_WORD            8
#define MPC_FPGA_DEBUG_HDR_EPSI_WORD          9
#define MPC_FPGA_DEBUG_HDR_EPSI_NORM_WORD     10
#define MPC_FPGA_DEBUG_HDR_VX_WORD            11
#define MPC_FPGA_DEBUG_HDR_VY_WORD            12
#define MPC_FPGA_DEBUG_HDR_OMEGA_WORD         13
#define MPC_FPGA_DEBUG_HDR_STEERING_WORD      14
#define MPC_FPGA_DEBUG_HDR_PREV_ACCEL_WORD    15
#define MPC_FPGA_DEBUG_HDR_MEASURED_STEER_RATE_WORD 16
#define MPC_FPGA_DEBUG_HDR_TRACE_WORD_COUNT   17
#define MPC_FPGA_DEBUG_HDR_SUMMARY_WORD_COUNT 18
#define MPC_FPGA_DEBUG_HDR_VALID_INPUT_WORDS  19
#define MPC_FPGA_DEBUG_HDR_TOTAL_WORDS        20
#define MPC_FPGA_DEBUG_HDR_TRACE_FIELDS       21
#define MPC_FPGA_DEBUG_HDR_TRACE_MAX          22
#define MPC_FPGA_DEBUG_HDR_RESERVED           23

#endif /* MPC_FPGA_INTERFACE_H */
