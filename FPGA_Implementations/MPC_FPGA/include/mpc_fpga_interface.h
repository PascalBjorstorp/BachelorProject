/**
 * @file mpc_fpga_interface.h
 * @brief CPU <-> FPGA Interface for MPC Riccati-ADMM Solver
 *
 * AXI-Lite register map for the MPC FPGA IP core.
 *
 * No-preload dataflow model:
 *   - CPU prepares one horizon frame in memory
 *   - CPU writes buffer addresses + count to AXI-Lite control regs
 *   - FPGA reads frame via AXI master and computes in one transaction
 *
 * Data format: all values are Q16.16 fixed-point (int32_t).
 */

#ifndef MPC_FPGA_INTERFACE_H
#define MPC_FPGA_INTERFACE_H

#include <stdint.h>

/*===========================================================================
 * Memory Map — AXI-Lite Registers
 *===========================================================================
 * Base address set in Vivado Address Editor (e.g., 0xA0000000).
 *===========================================================================*/

#define MPC_FPGA_BASE_ADDR      0xA0000000

/* --- AXI-Lite Control Registers --- */
#define REG_AP_CTRL             0x000   /* bit0=start, bit1=done, bit2=idle */
#define REG_GIE                 0x004
#define REG_IER                 0x008
#define REG_ISR                 0x00C

/* --- Vehicle State (R/W) ---
 * state_x_fp is interpreted as e_y.
 * state_theta_fp is interpreted as e_psi. */
#define REG_ST_X                0x010
#define REG_ST_THETA            0x018
#define REG_ST_VX               0x020
#define REG_ST_VY               0x028
#define REG_ST_OMEGA            0x030
#define REG_ST_STEERING         0x038

/* --- Bulk reference pointers/count (Read/Write) --- */
#define REG_REF_VX_MEM_LO       0x040
#define REG_REF_VX_MEM_HI       0x044
#define REG_REF_KAPPA_MEM_LO    0x04C
#define REG_REF_KAPPA_MEM_HI    0x050
#define REG_REF_LEFT_MEM_LO     0x058
#define REG_REF_LEFT_MEM_HI     0x05C
#define REG_REF_RIGHT_MEM_LO    0x064
#define REG_REF_RIGHT_MEM_HI    0x068
#define REG_REF_COUNT           0x070

/* --- Output Registers (Read-Only) ---
 * Each ap_vld output takes 8 bytes: data@+0, ap_vld@+4, reserved@+8/+C
 */
#define REG_OUT_STEERING        0x078   /* Steering cmd Q16.16 */
#define REG_OUT_STEERING_VLD    0x07C   /* out_steering_fp_ap_vld */
#define REG_OUT_ACCEL           0x088   /* Accel cmd Q16.16    */
#define REG_OUT_ACCEL_VLD       0x08C   /* out_accel_fp_ap_vld */
#define REG_OUT_STATUS          0x098   /* 0=optimal, 1=max_iter, 2=error */
#define REG_OUT_STATUS_VLD      0x09C   /* out_status_ap_vld */
#define REG_OUT_ITERATIONS      0x0A8   /* ADMM iterations used */
#define REG_OUT_ITERATIONS_VLD  0x0AC   /* out_iterations_ap_vld */

/*===========================================================================
 * Status Codes
 *===========================================================================*/

#define MPC_FPGA_STATUS_OK              0
#define MPC_FPGA_STATUS_MAX_ITER        1
#define MPC_FPGA_STATUS_ERROR           2
#define MPC_FPGA_STATUS_NO_TRAJECTORY   3

/*===========================================================================
 * Reference Buffer Configuration
 *===========================================================================*/

#define MPC_FPGA_MAX_REF_POINTS         64

#endif /* MPC_FPGA_INTERFACE_H */
