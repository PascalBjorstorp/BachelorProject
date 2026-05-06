/**
 * @file mpc_fpga_interface.h
 * @brief OpenCL transport contract constants for MPC Riccati-ADMM.
 * @details Defines payload framing, horizon sizing, and solver status values
 *          shared by host OpenCL launchers and FPGA kernel wrappers.
 * @dependencies <stdint.h>, mpc_fpga_constants.h
 */

#ifndef MPC_FPGA_INTERFACE_H
#define MPC_FPGA_INTERFACE_H

#include <stdint.h>
#include "mpc_fpga_constants.h"

#ifdef MPC_HORIZON
#if MPC_HORIZON != MPC_FPGA_HORIZON_STEPS
#error "MPC_HORIZON must match MPC_FPGA_HORIZON_STEPS"
#endif
#else
#define MPC_HORIZON             MPC_FPGA_HORIZON_STEPS
#endif

#define MPC_FPGA_HEADER_WORDS   MPC_FPGA_HEADER_WORDS_32
#define MPC_FPGA_REF_WORDS      MPC_FPGA_REF_WORDS_PER_STEP
#define INPUT_BUFFER_WORDS_32   MPC_FPGA_INPUT_WORDS_32
#define DMA_BUFFER_BYTES        MPC_FPGA_DMA_BYTES
#define DMA_BUFFER_BEATS        MPC_FPGA_DMA_BEATS
#define INPUT_BUFFER_WORDS_512  ((INPUT_BUFFER_WORDS_32 + 15) / 16)
#define INPUT_BUFFER_WORDS_32_PAD (INPUT_BUFFER_WORDS_512 * 16)
#define INPUT_BUFFER_BYTES_512  (INPUT_BUFFER_WORDS_512 * 64)

#define MPC_FPGA_STATUS_OK              0
#define MPC_FPGA_STATUS_MAX_ITER        1
#define MPC_FPGA_STATUS_ERROR           2
#define MPC_FPGA_STATUS_NO_TRAJECTORY   3

#endif  // MPC_FPGA_INTERFACE_H

