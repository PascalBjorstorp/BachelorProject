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

typedef struct {
    int iter;
    float primal_residual;
    float dual_residual;
    float state_primal_residual;
    float state_dual_residual;
    float ctrl_primal_residual;
    float ctrl_dual_residual;
    float rho;
    float rho_u;
    float u0_steer;
    float u0_accel;
    float z0_steer;
    float z0_accel;
    float y0_steer;
    float y0_accel;
    int scale_rho;
    int scale_rho_u;
} MpcHlsDebugIterSample_t;

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
/* Sparse-B horizon terms held OUTSIDE the BRAM-backed StepData_t. They sit on
 * the backward-pass M = B^T P recurrence; sourcing them from a far BRAM column
 * cost a 1.19 ns BRAM Tco + long route (the worst routed path). Kept as a
 * fully-partitioned register array threaded whole-array (safe HLS pattern). */
#define MPC_BSP_N 4
enum {
  MPC_BSP_DELTA_RATE  = 0,
  MPC_BSP_VX_ACCEL    = 1,
  MPC_BSP_VY_ACCEL    = 2,
  MPC_BSP_OMEGA_ACCEL = 3
};

MpcStatus_t riccati_admm_solve_hls(
    const StepData_t step_data[MPC_HORIZON],
    const fp_QP_t B_sparse[MPC_HORIZON][MPC_BSP_N],
    const fp_QP_t terminal_q_diag[MPC_NX_AUG],
    const fp_QP_t terminal_q_linear[MPC_NX_AUG],
    const fp_QP_t terminal_x_lb[MPC_NX_AUG],
    const fp_QP_t terminal_x_ub[MPC_NX_AUG],
    const fp_QP_t x0[MPC_NX_AUG],
    const AdmmConfig_t *config,
    AdmmState_t *admm_state,
    MpcSolution_t *solution);

int riccati_hls_debug_get_trace_count(void);
int riccati_hls_debug_get_trace_sample(int index, MpcHlsDebugIterSample_t *out);

#ifdef __cplusplus
}
#endif

#endif /* RICCATI_SOLVER_HLS_H */
