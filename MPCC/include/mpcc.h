/**
 * @file mpcc.h
 * @brief Model Predictive Contouring Control — Public API
 *        (Lifted ODE: Frenet primary + Cartesian redundant)
 *
 * MPCC for F1/10th autonomous racing using the Lifted ODE formulation.
 * Unlike standard MPCC (which uses a virtual progress variable theta),
 * this formulation uses the Frenet frame as the primary ODE:
 *   - s (arc-length) evolves naturally from the vehicle dynamics
 *   - n (lateral deviation) gives trivial track boundary constraints
 *   - alpha (heading error) captures path alignment
 *   - Redundant Cartesian states (X, Y, psi) for obstacle avoidance
 *
 * Key differences from standard MPCC:
 *   - v_theta control: ds/dt = v_theta decouples progress from kinematics
 *   - Track bounds = box constraints on n (trivial in ADMM)
 *   - Obstacle avoidance in Cartesian frame (convex constraints)
 *   - Progress reward: maximize ds/dt or terminal s_N
 *   - Only 2 control inputs: [delta, a_x]
 *
 * Cost function (at each stage k):
 *   J_k = q_n * n^2  +  q_alpha * alpha^2    path following
 *       + q_vy * vy^2  +  q_omega * omega^2   state regularization
 *       - q_s * ds/dt_k                        progress reward
 *       + u^T R u  +  du^T Rd du              control effort/smoothness
 *
 * Obstacle avoidance (Cartesian states):
 *   ||[X, Y] - c_obs||_{Sigma_inv} >= 1       ellipsoidal constraint
 *
 * Solver: ADMM with Riccati recursion for the structured QP.
 *
 * Usage:
 *   1. mpcc_initialize()
 *   2. mpcc_set_reference_path(path)
 *   3. Each control cycle:
 *      a. Get vehicle state + Frenet conversion
 *      b. Build MPCCState_t (Frenet + Cartesian)
 *      c. mpcc_set_obstacles(...) if needed
 *      d. mpcc_compute_control(&state, &result)
 *      e. Apply result.optimal_control.delta and .a_x
 *
 * Platform-independent (no ROS dependencies).
 * All arithmetic uses Q16.16 fixed-point for FPGA compatibility.
 */

#ifndef MPCC_H
#define MPCC_H

#include "mpcc_types.h"

/*===========================================================================
 * MPCC Initialization
 *===========================================================================*/

/**
 * Initialize MPCC with default configuration.
 *
 * Sets default weights, ADMM parameters, and constraint bounds.
 * Initializes the vehicle model with F1/10th defaults.
 * Must be called before mpcc_compute_control().
 */
void mpcc_initialize(void);

/**
 * Initialize MPCC with custom configuration.
 * @param config  Pointer to custom MPCC configuration
 */
void mpcc_initialize_with_config(const MPCCConfiguration_t *config);

/*===========================================================================
 * Reference Path Management
 *===========================================================================*/

/**
 * Set the reference path for MPCC to follow.
 *
 * The path must be arc-length parameterized with monotonically
 * increasing s_ref values. For a closed track, set is_closed=1.
 * Calling this invalidates any warm-start data.
 *
 * @param path  Pointer to the reference path structure
 */
void mpcc_set_reference_path(const MPCCReferencePath_t *path);

/*===========================================================================
 * Obstacle Management
 *===========================================================================*/

/**
 * Set the current obstacle set for avoidance.
 *
 * Obstacles are expressed as ellipses in the global (X, Y) frame.
 * The Sigma_inv field must be precomputed before calling this.
 * Call mpcc_obstacle_compute_sigma_inv() to compute it.
 *
 * @param obstacles  Pointer to the obstacle set
 */
void mpcc_set_obstacles(const MPCCObstacleSet_t *obstacles);

/**
 * Clear all obstacles (disable obstacle avoidance).
 */
void mpcc_clear_obstacles(void);

/**
 * Precompute the inverse shape matrix for an obstacle.
 *
 * Computes Sigma_inv from (a, b, phi):
 *   Sigma_inv = R(-phi) * diag(1/a^2, 1/b^2) * R(-phi)^T
 *
 * @param obs  Obstacle to update (a, b, phi must be set). Sigma_inv is filled.
 */
void mpcc_obstacle_compute_sigma_inv(MPCCObstacle_t *obs);

/*===========================================================================
 * Control Computation
 *===========================================================================*/

/**
 * Compute optimal control for current vehicle state.
 *
 * Main MPCC function called each control cycle.
 * Internally:
 *   1. Warm-start from previous solution (shifted)
 *   2. Linearize Frenet + Cartesian dynamics at each stage
 *   3. Build stage costs (n/alpha penalties, progress reward)
 *   4. Set box constraints (track bounds on n, actuator limits)
 *   5. Linearize obstacle constraints (if any)
 *   6. Solve QP via ADMM + Riccati
 *   7. Extract first control input
 *
 * @param current_state  Current Lifted ODE state (Frenet + Cartesian)
 * @param result         Output: optimal control, trajectory, diagnostics
 * @return Solver status code
 */
MPCCStatus_t mpcc_compute_control(
    const MPCCState_t *current_state,
    MPCCResult_t *result);

/*===========================================================================
 * Configuration Access
 *===========================================================================*/

/** Get current MPCC configuration. */
MPCCConfiguration_t mpcc_get_configuration(void);

/** Update MPCC configuration at runtime. */
void mpcc_set_configuration(const MPCCConfiguration_t *config);

/**
 * Reset MPCC solver state.
 * Clears warm-start data and ADMM workspace.
 */
void mpcc_reset(void);

/*===========================================================================
 * Path Utility Functions
 *===========================================================================*/

/**
 * Interpolate the reference path at a given arc-length s.
 *
 * Linear interpolation between nearest waypoints.
 * Handles wrapping for closed paths (s modulo total_length).
 *
 * @param path    Pointer to the reference path
 * @param s       Arc-length parameter to interpolate at [m]
 * @param result  Output: interpolated path point
 */
void mpcc_path_interpolate(
    const MPCCReferencePath_t *path,
    float s,
    MPCCPathPoint_t *result);

/**
 * Find the closest s on the reference path to position (X, Y).
 *
 * Useful for initializing the Frenet state when the car starts.
 *
 * @param path  Reference path to search
 * @param X     Vehicle X position [m]
 * @param Y     Vehicle Y position [m]
 * @return Estimated s at the closest path point
 */
float mpcc_find_closest_s(
    const MPCCReferencePath_t *path,
    float X,
    float Y);

/*===========================================================================
 * Vehicle State → MPCC State Conversion
 *===========================================================================*/

/**
 * Convert VehicleState_t (Cartesian) to MPCCState_t (global-frame 7-state).
 *
 * Copies body-frame velocities and Cartesian pose directly.
 * Computes s via forward-biased closest-waypoint search on the reference path.
 *
 * Requires the reference path to be set.
 *
 * @param vehicle_state  Vehicle state from localization (Cartesian)
 * @param s_hint         Previous s estimate used to preserve path continuity
 * @return Global-frame MPCC state [s, vx, vy, omega, X, Y, psi]
 */
MPCCState_t mpcc_state_from_vehicle_state(
    const VehicleState_t *vehicle_state,
    float s_hint);

#endif /* MPCC_H */
