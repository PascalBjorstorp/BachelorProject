/**
 * @file riccati_solver_hls.h
 * @brief Riccati-ADMM Solver Interface for HLS
 *
 * Constrained LQR solver using ADMM with Riccati recursion.
 */

#ifndef RICCATI_SOLVER_HLS_H
#define RICCATI_SOLVER_HLS_H

#include "mpc_fpga_types.h"

/**
 * Solve constrained LQR via Riccati-ADMM (HLS-synthesizable).
 *
 * @param step_data   Per-step dynamics and cost (array of MPC_HORIZON)
 * @param terminal_Q  Terminal diagonal cost (length MPC_NX_AUG)
 * @param terminal_q  Terminal linear cost (length MPC_NX_AUG)
 * @param x0          Initial state (length MPC_NX_AUG)
 * @param config      ADMM configuration
 * @param admm_state  Warm-start state
 * @param solution    Output: optimal trajectories and status
 * @return Solver status
 */
MpcStatus_t riccati_admm_solve_hls(
    const StepData_t step_data[MPC_HORIZON],
    const fixed_point_t terminal_Q[MPC_NX_AUG],
    const fixed_point_t terminal_q[MPC_NX_AUG],
    const fixed_point_t x0[MPC_NX_AUG],
    const AdmmConfig_t *config,
    AdmmState_t *admm_state,
    MpcSolution_t *solution);

#endif /* RICCATI_SOLVER_HLS_H */
