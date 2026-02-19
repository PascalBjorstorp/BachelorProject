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
 * 6-state dynamic bicycle model: [x, y, ψ, v_x, v_y, ω]
 * 48 bytes per cycle
 *===========================================================================*/

typedef struct __attribute__((packed, aligned(16))) {
    int32_t x_fp;               // Position X [m], Q16.16
    int32_t y_fp;               // Position Y [m], Q16.16
    int32_t theta_fp;           // Heading ψ [rad], Q16.16
    int32_t velocity_fp;        // Longitudinal velocity v_x [m/s], Q16.16
    int32_t vy_fp;              // Lateral velocity v_y [m/s], Q16.16
    int32_t omega_fp;           // Yaw rate ω [rad/s], Q16.16
    uint32_t waypoint_index;    // Closest waypoint (from Jetson)
    uint32_t timestamp_ms;      // For latency tracking
    uint32_t sequence_number;   // Frame counter
    uint32_t reserved[3];       // Padding to 48 bytes
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
 * Memory Map for Ultra96 PL (v2 - scalar interface)
 *===========================================================================
 * All registers are on a single AXI-Lite CTRL bundle.
 * From HLS-generated xpure_pursuit_fpga_hw.h:
 *
 * Modes:
 *   0 = Compute steering from vehicle state
 *   1 = Load one waypoint into BRAM
 *   2 = Finalize trajectory (set total count, mark loaded)
 *===========================================================================*/

// Base address (set in Vivado Address Editor)
#define FPGA_BASE_ADDR          0xA0000000

// --- AXI-Lite Control Registers ---
#define REG_AP_CTRL             0x000  // bit0=start, bit1=done, bit2=idle
#define REG_GIE                 0x004  // Global Interrupt Enable
#define REG_IER                 0x008  // IP Interrupt Enable
#define REG_ISR                 0x00C  // IP Interrupt Status

// --- Mode Selection (R/W) ---
#define REG_MODE                0x010  // 0=compute, 1=load_wp, 2=finalize

// --- Waypoint Loading Registers (mode=1) ---
#define REG_WP_INDEX            0x018  // Waypoint index
#define REG_WP_X                0x020  // X position Q16.16
#define REG_WP_Y                0x028  // Y position Q16.16
#define REG_WP_THETA            0x030  // Heading Q16.16
#define REG_WP_VEL              0x038  // Velocity Q16.16
#define REG_WP_KAPPA            0x040  // Curvature Q16.16
#define REG_WP_TOTAL            0x048  // Total waypoints (mode=2)

// --- Vehicle State Registers (mode=0, R/W) ---
#define REG_ST_X                0x050  // Position X Q16.16
#define REG_ST_Y                0x058  // Position Y Q16.16
#define REG_ST_THETA            0x060  // Heading Q16.16
#define REG_ST_VEL              0x068  // Velocity Q16.16
#define REG_ST_WP_IDX           0x070  // Closest waypoint index

// --- Parameter Registers (mode=0, R/W) ---
#define REG_P_MIN_LA            0x078  // Min lookahead Q16.16
#define REG_P_MAX_LA            0x080  // Max lookahead Q16.16
#define REG_P_LA_GAIN           0x088  // Lookahead gain Q16.16
#define REG_P_WHEELBASE         0x090  // Wheelbase Q16.16
#define REG_P_MAX_STEER         0x098  // Max steering Q16.16
#define REG_P_MAX_VEL           0x0A0  // Max velocity Q16.16
#define REG_P_LA_POINTS         0x0A8  // Lookahead search points

// --- Output Registers (Read-Only) ---
#define REG_OUT_STEERING        0x0B0  // Steering angle Q16.16
#define REG_OUT_STEERING_VLD    0x0B4  // Steering valid flag
#define REG_OUT_VELOCITY        0x0C0  // Velocity Q16.16
#define REG_OUT_VELOCITY_VLD    0x0C4  // Velocity valid flag
#define REG_OUT_CTE             0x0D0  // Cross-track error Q16.16
#define REG_OUT_CTE_VLD         0x0D4  // CTE valid flag
#define REG_OUT_HEADING_ERR     0x0E0  // Heading error Q16.16
#define REG_OUT_HEADING_ERR_VLD 0x0E4  // Heading error valid flag
#define REG_OUT_LOOKAHEAD       0x0F0  // Actual lookahead Q16.16
#define REG_OUT_LOOKAHEAD_VLD   0x0F4  // Lookahead valid flag
#define REG_OUT_TARGET_WP       0x100  // Target waypoint index
#define REG_OUT_TARGET_WP_VLD   0x104  // Target WP valid flag
#define REG_OUT_STATUS          0x110  // Status code
#define REG_OUT_STATUS_VLD      0x114  // Status valid flag
#define REG_OUT_TRAJ_LOADED     0x120  // Trajectory loaded flag
#define REG_OUT_TRAJ_LOADED_VLD 0x124  // Traj loaded valid flag
#define REG_OUT_TRAJ_SIZE       0x130  // Trajectory size
#define REG_OUT_TRAJ_SIZE_VLD   0x134  // Traj size valid flag

/*===========================================================================
 * Status Codes
 *===========================================================================*/

#define STATUS_OK               0
#define STATUS_NO_TRAJECTORY    1
#define STATUS_INVALID_INDEX    2
#define STATUS_OVERFLOW         3

#endif /* FPGA_INTERFACE_H */
