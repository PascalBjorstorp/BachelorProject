/**
 * @file riccati_solver_hls.h
 * @brief Public interface for the HLS Riccati-ADMM solver core.
 * @details Declares the constrained LQR solve entry point used by the
 *          FPGA MPC pipeline. The implementation performs ADMM iterations
 *          with Riccati recursion for the primal update.
 * @dependencies mpc_fpga_types.h
 */

#ifndef RICCATI_SOLVER_HLS_H
#define RICCATI_SOLVER_HLS_H

#include "mpc_fpga_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Solve constrained LQR using Riccati-ADMM.
 * @param step_data Per-step dynamics, costs, and bounds array of length MPC_HORIZON.
 * @param terminal_q_diag Terminal diagonal cost vector of length MPC_NX_AUG.
 * @param terminal_q_linear Terminal linear cost vector of length MPC_NX_AUG.
 * @param x0 Initial augmented state vector of length MPC_NX_AUG.
 * @param config ADMM configuration pointer.
 * @param admm_state Warm-start and dual/primal history pointer.
 * @param solution Output solution trajectories and solver diagnostics pointer.
 * @return Solver status code.
 */
MpcStatus_t riccati_admm_solve_hls(
    const StepData_t step_data[MPC_HORIZON],
    const fixed_point_t terminal_q_diag[MPC_NX_AUG],
    const fixed_point_t terminal_q_linear[MPC_NX_AUG],
    const fixed_point_t x0[MPC_NX_AUG],
    const AdmmConfig_t *config,
    AdmmState_t *admm_state,
    MpcSolution_t *solution);

#ifdef __cplusplus
}
#endif

#endif /* RICCATI_SOLVER_HLS_H */
