/**
 * @file mpc_fpga_interface.h
 * @brief CPU <-> FPGA Interface for MPC Controller
 *
 * This header defines the HLS top-level function signature and BRAM
 * layout for the MPC FPGA IP core on the Ultra96-V2.
 *
 * The CPU communicates with the FPGA via AXI-Lite registers:
 *   Startup:  CPU loads trajectory waypoints one at a time (mode=1)
 *             CPU finalizes trajectory (mode=2)
 *   Runtime:  CPU writes vehicle state + waypoint index (mode=0)
 *             FPGA computes optimal MPC control
 *             CPU reads steering + acceleration outputs
 *
 * The MPC uses a condensed QP formulation in Frenet (path-relative)
 * coordinates with a dynamic bicycle model and direct acceleration
 * input compatible with the VESC motor controller.
 *
 * Target: Xilinx Ultra96-V2 (Zynq UltraScale+ ZU3EG)
 *         100 MHz, 360 DSP48E2, ~70K LUT, ~63KB usable BRAM
 */

#ifndef MPC_FPGA_INTERFACE_H
#define MPC_FPGA_INTERFACE_H

#include <stdint.h>

/*===========================================================================
 * Trajectory Configuration
 *===========================================================================*/

/**
 * Maximum number of trajectory waypoints stored in FPGA BRAM.
 * Spielberg raceline has ~1000 waypoints; allocate extra for safety.
 * Each waypoint is 40 bytes → 1024 × 40 = 40 KB BRAM.
 */
#define MPC_FPGA_MAX_TRAJECTORY_SIZE 1024

/**
 * MPC prediction horizon (must match MPC_DEFAULT_PREDICTION_HORIZON).
 * N=20 with dt=50ms → 1.0 second lookahead.
 */
#define MPC_FPGA_HORIZON 20

/*===========================================================================
 * Waypoint Structure (stored in FPGA BRAM)
 *===========================================================================
 * Each waypoint contains the reference trajectory information needed
 * by the MPC for one prediction step: target state, curvature, and
 * wall distance bounds.
 *
 * Packed to 40 bytes (10 × int32_t) for efficient BRAM storage.
 */
typedef struct __attribute__((packed, aligned(8))) {
    int32_t ref_lateral_error_fp;      /**< Reference lateral error [m], Q16.16 (normally 0) */
    int32_t ref_heading_error_fp;      /**< Reference heading error [rad], Q16.16 (normally 0) */
    int32_t ref_velocity_fp;           /**< Target velocity [m/s], Q16.16 */
    int32_t ref_lateral_velocity_fp;   /**< Target lateral velocity [m/s], Q16.16 (normally 0) */
    int32_t ref_yaw_rate_fp;           /**< Target yaw rate [rad/s], Q16.16 */
    int32_t curvature_fp;              /**< Path curvature [1/m], Q16.16 */
    int32_t left_wall_fp;              /**< Left wall bound [m], Q16.16 */
    int32_t right_wall_fp;             /**< Right wall bound [m], Q16.16 */
    int32_t reserved;                  /**< Padding to 40 bytes */
} MpcFpgaWaypoint_t;

/*===========================================================================
 * Operation Modes
 *===========================================================================*/

/** Mode 0: Compute MPC control from current vehicle state */
#define MPC_FPGA_MODE_COMPUTE    0

/** Mode 1: Load one trajectory waypoint into BRAM at given index */
#define MPC_FPGA_MODE_LOAD_WP    1

/** Mode 2: Finalize trajectory (set size, mark as loaded) */
#define MPC_FPGA_MODE_FINALIZE   2

/** Mode 3: Reset MPC solver state (clear warm-start, etc.) */
#define MPC_FPGA_MODE_RESET      3

/*===========================================================================
 * Status Codes (output)
 *===========================================================================*/

#define MPC_FPGA_STATUS_OK                 0  /**< Optimal solution found */
#define MPC_FPGA_STATUS_MAX_ITER           1  /**< Hit max iterations (result usable) */
#define MPC_FPGA_STATUS_INFEASIBLE         2  /**< No feasible solution */
#define MPC_FPGA_STATUS_ERROR              3  /**< Solver error */
#define MPC_FPGA_STATUS_NO_TRAJECTORY     10  /**< Trajectory not loaded */
#define MPC_FPGA_STATUS_NOT_INITIALIZED   11  /**< MPC not initialized */

/*===========================================================================
 * HLS Top-Level Function
 *===========================================================================
 * All arguments are scalars for AXI-Lite register compatibility.
 * This function is synthesized by Vitis HLS into an IP core.
 *
 * Modes:
 *   mode=0: Compute steering + acceleration from Frenet vehicle state
 *   mode=1: Store one waypoint at wp_index
 *   mode=2: Finalize trajectory loading (set size)
 *   mode=3: Reset MPC solver state
 */
void mpc_fpga(
    /* Control */
    uint32_t mode,

    /* --- Waypoint loading (mode=1): one waypoint per call --- */
    uint32_t wp_index,
    int32_t  wp_ref_lat_error,       /**< Reference lateral error Q16.16 */
    int32_t  wp_ref_heading_error,   /**< Reference heading error Q16.16 */
    int32_t  wp_ref_velocity,        /**< Reference velocity Q16.16 */
    int32_t  wp_ref_lat_vel,         /**< Reference lateral velocity Q16.16 */
    int32_t  wp_ref_yaw_rate,        /**< Reference yaw rate Q16.16 */
    int32_t  wp_curvature,           /**< Path curvature Q16.16 */
    int32_t  wp_left_wall,           /**< Left wall bound Q16.16 */
    int32_t  wp_right_wall,          /**< Right wall bound Q16.16 */
    uint32_t wp_total,               /**< Total waypoints (mode=2 only) */

    /* --- Vehicle state in Frenet frame (mode=0) --- */
    int32_t  st_lateral_error,       /**< Lateral error e_y [m] Q16.16 */
    int32_t  st_heading_error,       /**< Heading error e_psi [rad] Q16.16 */
    int32_t  st_vx,                  /**< Longitudinal velocity [m/s] Q16.16 */
    int32_t  st_vy,                  /**< Lateral velocity [m/s] Q16.16 */
    int32_t  st_omega,               /**< Yaw rate [rad/s] Q16.16 */
    uint32_t st_wp_index,            /**< Closest waypoint index on trajectory */

    /* --- Outputs (mode=0) --- */
    int32_t  *out_steering,          /**< Optimal steering angle [rad] Q16.16 */
    int32_t  *out_acceleration,      /**< Optimal acceleration [m/s²] Q16.16 */
    uint32_t *out_status,            /**< Solver status code */
    uint32_t *out_iterations,        /**< QP iterations used */
    int32_t  *out_cost,              /**< Final cost Q16.16 */
    uint32_t *out_traj_loaded,       /**< Trajectory loaded flag */
    uint32_t *out_traj_size          /**< Number of loaded waypoints */
);

#endif /* MPC_FPGA_INTERFACE_H */
