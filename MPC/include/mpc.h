/**
 * @file mpc.h
 * @brief Model Predictive Control Public API (Riccati-ADMM)
 *
 * Main interface for the F1/10th MPC system using Riccati-ADMM solver.
 * This is a non-condensed formulation where the Riccati recursion solves
 * the unconstrained LQR sub-problem in O(N) per ADMM iteration, and ADMM
 * handles box constraints on states (walls) and controls (actuator limits).
 *
 * Usage:
 *   1. Initialize: mpc_initialize() or mpc_initialize_with_configuration()
 *   2. Each control cycle:
 *      - Get current state from localization
 *      - Build reference trajectory
 *      - Call mpc_compute_optimal_control()
 *      - Apply returned control to vehicle (no post-processing needed)
 *
 * Platform-independent (no ROS dependencies).
 * All arithmetic uses Q16.16 fixed-point for FPGA compatibility.
 */

#ifndef MPC_H
#define MPC_H

#include "mpc_types.h"
#include "vehicle_model.h"
#include "util_math.h"

/*===========================================================================
 * MPC Initialization
 *===========================================================================*/

/**
 * Initialize MPC with default configuration.
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
 * Compute optimal control for current vehicle state (Frenet frame).
 *
 * This is the main MPC function called each control cycle.
 * The state is expressed in Frenet (path-relative) coordinates.
 * Output is the direct MPC solution — no bias, clamp, or post-processing.
 *
 * @param current_frenet_state   Current state in Frenet frame
 * @param reference_trajectory   Array of reference points (length >= horizon)
 * @param result                 Output: optimal control and solver status
 * @return Solver status code
 */
MpcSolverStatus_t mpc_compute_optimal_control(
    const FrenetState_t *current_frenet_state,
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
 * Reset MPC solver state (clears warm-start data).
 */
void mpc_reset(void);

/**
 * Feed back the actual (hardware-measured) previous control.
 *
 * When actuator dynamics (e.g. servo rate limits) cause the realized
 * control to differ from the MPC command, call this function BEFORE
 * the next mpc_compute_optimal_control() so the MPC's delta_prev /
 * accel_prev states match reality.  If not called, the MPC uses its
 * own previous command as delta_prev (correct when there is no
 * actuator lag).
 *
 * @param actual  The control actually applied to the plant.
 */
void mpc_set_actual_previous_control(const ControlInput_t *actual);

#endif /* MPC_H */
