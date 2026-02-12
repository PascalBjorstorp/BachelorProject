/**
 * @file mpc.h
 * @brief Model Predictive Control Public API
 *
 * Main interface for the F1/10th MPC system.
 * This header provides everything needed to use the MPC controller.
 *
 * Usage:
 *   1. Initialize: mpc_initialize() or mpc_initialize_with_config()
 *   2. Each control cycle:
 *      - Get current state from localization
 *      - Build reference trajectory
 *      - Call mpc_compute_optimal_control()
 *      - Apply returned control to vehicle
 *
 * Platform-independent (no ROS dependencies).
 * All arithmetic uses fixed-point for FPGA compatibility.
 */

#ifndef MPC_H
#define MPC_H

#include "mpc_types.h"
#include "vehicle_model.h"
#include "fp_math.h"

/*===========================================================================
 * MPC Initialization
 *===========================================================================*/

/**
 * Initialize MPC with default configuration.
 *
 * Default configuration:
 * - Prediction horizon: 10 steps
 * - Time step: 0.1 seconds
 * - Balanced cost weights
 * - 100 solver iterations max
 */
void mpc_initialize(void);

/**
 * Initialize MPC with custom configuration.
 *
 * @param configuration  Pointer to custom MPC configuration
 */
void mpc_initialize_with_configuration(
    const MpcConfiguration_t *configuration);

/*===========================================================================
 * Control Computation
 *===========================================================================*/

/**
 * Compute optimal control for current vehicle state.
 *
 * This is the main MPC function called each control cycle.
 *
 * @param current_vehicle_state  Current state from localization
 * @param reference_trajectory   Array of reference points (length = horizon)
 * @param result                 Output: optimal control and solver status
 * @return Solver status code
 */
MpcSolverStatus_t mpc_compute_optimal_control(
    const VehicleState_t *current_vehicle_state,
    const TrajectoryReferencePoint_t *reference_trajectory,
    MpcSolverResult_t *result);

/*===========================================================================
 * Configuration Access
 *===========================================================================*/

/**
 * Get current MPC configuration.
 *
 * @return Copy of current configuration structure
 */
MpcConfiguration_t mpc_get_configuration(void);

/**
 * Update MPC configuration.
 *
 * @param configuration  New configuration to apply
 */
void mpc_set_configuration(const MpcConfiguration_t *configuration);

/**
 * Reset MPC solver state.
 *
 * Clears any warm-start data. Call after:
 * - Significant trajectory changes
 * - Recovery from errors
 * - Mode transitions
 */
void mpc_reset(void);

#endif /* MPC_H */
