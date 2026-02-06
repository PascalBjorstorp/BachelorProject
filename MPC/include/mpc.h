/**
 * @file mpc.h
 * @brief Model Predictive Control Public API
 *
 * Platform-independent (no ROS dependencies).
 * All arithmetic uses Q16.16 fixed-point for FPGA compatibility.
 */

#ifndef MPC_H
#define MPC_H

#include "mpc_types.h"
#include "vehicle_model.h"
#include "fp_math.h"

/*===========================================================================
 * MPC Initialization
 *===========================================================================*/

/** Initialize MPC with default configuration */
void mpc_init(void);

/** Initialize MPC with custom configuration */
void mpc_init_config(const MpcConfig_t *config);

/*===========================================================================
 * Control Computation
 *===========================================================================*/

/**
 * Compute optimal control for current state.
 *
 * @param state      Current vehicle state
 * @param trajectory Reference trajectory (length = horizon)
 * @param result     Output: optimal control and solver status
 * @return Solver status
 */
SolverStatus_t mpc_compute(
    const VehicleState_t *state,
    const TrajectoryPoint_t *trajectory,
    MpcResult_t *result);

/*===========================================================================
 * Configuration Access
 *===========================================================================*/

/** Get current configuration */
MpcConfig_t mpc_get_config(void);

/** Update configuration */
void mpc_set_config(const MpcConfig_t *config);

/** Reset solver state (call after trajectory changes) */
void mpc_reset(void);

#endif /* MPC_H */
