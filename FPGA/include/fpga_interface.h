/**
 * @file fpga_interface.h
 * @brief CPU <-> FPGA Interface - Trajectory Stored in FPGA BRAM
 *
 * This version stores the entire trajectory in FPGA BRAM.
 * CPU only sends vehicle state + waypoint_index each cycle.
 *
 * Flow:
 *   Startup:  CPU → DMA → FPGA BRAM (full trajectory, once)
 *   Runtime:  CPU → AXI → FPGA (state + waypoint_idx only)
 *             FPGA looks up waypoints from internal BRAM
 */

#ifndef FPGA_INTERFACE_H
#define FPGA_INTERFACE_H

#include <stdint.h>

/*===========================================================================
 * Fixed-Point Format (Q16.16)
 *===========================================================================*/

#define FP_FRAC_BITS    16
#define FP_ONE          (1 << FP_FRAC_BITS)

/*===========================================================================
 * Trajectory Configuration
 *===========================================================================*/

// Maximum trajectory size (adjust based on your raceline)
// Spielberg has ~1000 waypoints, allocate more for safety
#define MAX_TRAJECTORY_SIZE  1024

/*===========================================================================
 * Waypoint Structure (stored in FPGA BRAM)
 *===========================================================================*/

typedef struct __attribute__((packed, aligned(32))) {
    int32_t x_fp;           // Position X [m], Q16.16
    int32_t y_fp;           // Position Y [m], Q16.16
    int32_t theta_fp;       // Heading [rad], Q16.16
    int32_t velocity_fp;    // Target velocity [m/s], Q16.16
    int32_t kappa_fp;       // Curvature [1/m], Q16.16
    int32_t reserved[3];    // Padding to 32 bytes
} FpgaWaypoint_t;

/*===========================================================================
 * Vehicle State Input (CPU -> FPGA, every cycle)
 * MINIMAL DATA - only 32 bytes per cycle!
 *===========================================================================*/

typedef struct __attribute__((packed, aligned(32))) {
    int32_t x_fp;               // Position X [m], Q16.16
    int32_t y_fp;               // Position Y [m], Q16.16
    int32_t theta_fp;           // Heading [rad], Q16.16
    int32_t velocity_fp;        // Current velocity [m/s], Q16.16
    uint32_t waypoint_index;    // Closest waypoint (from Jetson)
    uint32_t timestamp_ms;      // For latency tracking
    uint32_t sequence_number;   // Frame counter
    uint32_t reserved;          // Padding
} FpgaStateInput_t;

/*===========================================================================
 * Control Parameters (CPU -> FPGA, at startup/on change)
 *===========================================================================*/

typedef struct __attribute__((packed, aligned(64))) {
    // Pure Pursuit parameters
    int32_t min_lookahead_fp;   // Minimum lookahead [m], Q16.16
    int32_t max_lookahead_fp;   // Maximum lookahead [m], Q16.16
    int32_t lookahead_gain_fp;  // Velocity gain, Q16.16
    
    // Vehicle parameters
    int32_t wheelbase_fp;       // Wheelbase [m], Q16.16
    int32_t max_steering_fp;    // Max steering [rad], Q16.16
    int32_t max_velocity_fp;    // Max velocity [m/s], Q16.16
    
    // Trajectory info
    uint32_t trajectory_size;   // Number of waypoints loaded
    uint32_t lookahead_points;  // How many points ahead to consider
    
    uint32_t reserved[8];
} FpgaParams_t;

/*===========================================================================
 * Control Output (FPGA -> CPU)
 *===========================================================================*/

typedef struct __attribute__((packed, aligned(64))) {
    // Control outputs
    int32_t steering_angle_fp;      // Steering [rad], Q16.16
    int32_t velocity_fp;            // Velocity [m/s], Q16.16
    
    // Debug/monitoring
    int32_t cross_track_error_fp;   // CTE [m], Q16.16
    int32_t heading_error_fp;       // Heading error [rad], Q16.16
    int32_t lookahead_dist_fp;      // Actual lookahead used [m], Q16.16
    uint32_t target_waypoint_idx;   // Which waypoint was targeted
    
    // Status
    uint32_t status;                // 0=OK, non-zero=error
    uint32_t compute_cycles;        // FPGA cycles used
    
    // Echo back for sync
    uint32_t sequence_number;
    uint32_t timestamp_ms;
    
    uint32_t reserved[2];
} FpgaOutput_t;

/*===========================================================================
 * Memory Map for Ultra96 PL
 *===========================================================================*/

// Base address (set in Vivado Address Editor)
#define FPGA_BASE_ADDR          0xA0000000

// Memory regions
#define FPGA_CTRL_OFFSET        0x0000      // Control registers (start/done/idle)
#define FPGA_PARAMS_OFFSET      0x0100      // Parameters (64 bytes)
#define FPGA_STATE_OFFSET       0x0200      // State input (32 bytes)
#define FPGA_OUTPUT_OFFSET      0x0300      // Control output (64 bytes)
#define FPGA_TRAJ_OFFSET        0x1000      // Trajectory BRAM start

// Control registers
#define REG_START               0x00        // Write 1 to start
#define REG_DONE                0x04        // Read: 1 when complete
#define REG_IDLE                0x08        // Read: 1 when idle
#define REG_TRAJ_LOADED         0x0C        // Read: 1 when trajectory is loaded

/*===========================================================================
 * Status Codes
 *===========================================================================*/

#define STATUS_OK               0
#define STATUS_NO_TRAJECTORY    1
#define STATUS_INVALID_INDEX    2
#define STATUS_OVERFLOW         3

#endif /* FPGA_INTERFACE_H */
