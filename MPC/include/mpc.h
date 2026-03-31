/**
 * @file mpc.h
 * @brief Riccati-ADMM Model Predictive Control public API.
 * @details Main interface for the F1/10th MPC Riccati-ADMM implementation.
 * @dependencies mpc_types.h, vehicle_model.h, util_math.h
 */

#ifndef MPC_H
#define MPC_H

#include "mpc_types.h"
#include "vehicle_model.h"
#include "util_math.h"

/*===========================================================================
 * Configuration Helpers
 *===========================================================================*/

/**
 * @brief Read a float environment variable with fallback.
 * @param name Environment variable name to query.
 * @param default_val Fallback value when the variable is absent or invalid.
 * @return Parsed float value, or default_val when not available.
 */
float get_env_float(const char *name, float default_val);

/**
 * @brief Read an integer environment variable with fallback.
 * @param name Environment variable name to query.
 * @param default_val Fallback value when the variable is absent or invalid.
 * @return Parsed integer value, or default_val when not available.
 */
int get_env_int(const char *name, int default_val);

/**
 * @brief Build the default MPC configuration and apply environment overrides.
 * @return MpcConfiguration_t populated with runtime-ready default parameters.
 */
MpcConfiguration_t get_default_configuration(void);

/*===========================================================================
 * MPC Initialization
 *===========================================================================*/

/**
 * @brief Initialize the MPC system with default F1/10th configuration.
 * @return None.
 */
void mpc_initialize(void);

/**
 * @brief Initialize the MPC system with a caller-supplied configuration.
 * @param configuration Pointer to custom MPC configuration; NULL selects defaults.
 * @return None.
 */
void mpc_initialize_with_configuration(const MpcConfiguration_t *configuration);

/*===========================================================================
 * Control Computation
 *===========================================================================*/

/**
 * @brief Compute optimal control for current vehicle state (Frenet frame).
 * @param current_frenet_state Current state in Frenet frame.
 * @param reference_trajectory Array of reference points (length >= horizon).
 * @param result Output: optimal control and solver status.
 * @return Solver status code indicating convergence result.
 *         MPC_STATUS_SUCCESS when an optimal solution is found;
 *         MPC_STATUS_MAXIMUM_ITERATIONS_REACHED when iteration budget is exhausted
 *         but result->optimal_control still contains the best available control;
 *         MPC_STATUS_ERROR on NULL input or unrecoverable failure.
 */
MpcSolverStatus_t mpc_compute_optimal_control(
    const FrenetState_t *current_frenet_state,
    const TrajectoryReferencePoint_t *reference_trajectory,
    MpcSolverResult_t *result);

/*===========================================================================
 * Configuration Access
 *===========================================================================*/

/**
 * @brief Retrieve a copy of the active MPC configuration.
 * @return Copy of the current MpcConfiguration_t; all fields reflect the
 *         most recently applied configuration.
 */
MpcConfiguration_t mpc_get_configuration(void);

/**
 * @brief Replace the active MPC configuration.
 * @param configuration Pointer to new configuration; call is a no-op if NULL.
 * @return None.
 */
void mpc_set_configuration(const MpcConfiguration_t *configuration);

/**
 * @brief Clear solver warm-start state and reset previous-control memory.
 * @details Forces the next solve to start from a cold initial condition.
 *          Does not change the active configuration or vehicle parameters.
 * @return None.
 */
void mpc_reset(void);

/**
 * @brief Feed back the hardware-measured previous control for actuator lag compensation.
 * @details When servo rate limits cause the realized steering to differ from
 *          the MPC command, call this before mpc_compute_optimal_control() to
 *          synchronize the MPC's delta_actual state with physical reality.
 *          If not called, the MPC assumes its own previous command was applied.
 * @param actual Pointer to the control actually applied to the vehicle; no-op if NULL.
 * @return None.
 */
void mpc_set_actual_previous_control(const ControlInput_t *actual);

#endif /* MPC_H */
