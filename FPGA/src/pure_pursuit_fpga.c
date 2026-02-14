/**
 * @file pure_pursuit_fpga.c
 * @brief Pure Pursuit with BRAM-Stored Trajectory (Vitis HLS)
 *
 * This version stores the entire trajectory in FPGA BRAM.
 * Only receives vehicle state + waypoint_index each cycle.
 *
 * Interfaces:
 *   - AXI-Lite: Control registers, parameters, state input, output
 *   - BRAM: Internal storage for trajectory waypoints
 */

#include "fpga_interface.h"
#include "fp_math_hls.h"  /* Local FPGA version with HLS pragmas */

/*===========================================================================
 * Internal BRAM for Trajectory Storage
 *===========================================================================*/

// Trajectory stored in BRAM (persistent between calls)
static FpgaWaypoint_t trajectory_bram[MAX_TRAJECTORY_SIZE];
#pragma HLS BIND_STORAGE variable=trajectory_bram type=ram_2p impl=bram

// Trajectory metadata
static uint32_t trajectory_size = 0;
static uint32_t trajectory_loaded = 0;

/*===========================================================================
 * Trajectory Loading Function
 *===========================================================================
 * Called once at startup to load waypoints from CPU to FPGA BRAM.
 */

void load_trajectory(
    const FpgaWaypoint_t* waypoints,    // Array from CPU
    uint32_t num_waypoints              // Number of waypoints
)
{
    #pragma HLS INTERFACE s_axilite port=waypoints bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=num_waypoints bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=return bundle=CTRL
    
    // Clamp to max size
    uint32_t count = (num_waypoints > MAX_TRAJECTORY_SIZE) 
                     ? MAX_TRAJECTORY_SIZE : num_waypoints;
    
    // Copy to internal BRAM
    LOAD_LOOP: for (uint32_t i = 0; i < count; i++) {
        #pragma HLS PIPELINE II=1
        #pragma HLS LOOP_TRIPCOUNT min=100 max=1024
        trajectory_bram[i] = waypoints[i];
    }
    
    trajectory_size = count;
    trajectory_loaded = 1;
}

/*===========================================================================
 * Pure Pursuit Computation
 *===========================================================================
 * Main control function - called every cycle.
 * Uses trajectory from internal BRAM.
 */

void pure_pursuit_compute(
    const FpgaStateInput_t* state,    // Vehicle state + waypoint_index
    const FpgaParams_t* params,       // Control parameters
    FpgaOutput_t* output              // Steering + velocity output
)
{
    #pragma HLS INTERFACE s_axilite port=state bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=params bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=output bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=return bundle=CTRL
    
    // Initialize output
    output->steering_angle_fp = 0;
    output->velocity_fp = 0;
    output->cross_track_error_fp = 0;
    output->heading_error_fp = 0;
    output->lookahead_dist_fp = 0;
    output->target_waypoint_idx = 0;
    output->status = STATUS_OK;
    output->compute_cycles = 0;
    output->sequence_number = state->sequence_number;
    output->timestamp_ms = state->timestamp_ms;
    
    // Check trajectory is loaded
    if (!trajectory_loaded || trajectory_size == 0) {
        output->status = STATUS_NO_TRAJECTORY;
        return;
    }
    
    // Validate waypoint index
    uint32_t wp_idx = state->waypoint_index;
    if (wp_idx >= trajectory_size) {
        wp_idx = wp_idx % trajectory_size;  // Wrap around
    }
    
    // =========================================================
    // Step 1: Compute adaptive lookahead distance
    // L = L_min + K * |v|
    // =========================================================
    int32_t speed = fp_abs(state->velocity_fp);
    int32_t lookahead = fp_add(
        params->min_lookahead_fp,
        fp_mul(params->lookahead_gain_fp, speed)
    );
    lookahead = fp_clamp(lookahead, params->min_lookahead_fp, params->max_lookahead_fp);
    output->lookahead_dist_fp = lookahead;
    
    // =========================================================
    // Step 2: Find target waypoint at lookahead distance
    // Start from current waypoint_index, search forward
    // =========================================================
    int32_t lookahead_sq = fp_mul(lookahead, lookahead);
    
    int32_t cos_theta = fp_cos(state->theta_fp);
    int32_t sin_theta = fp_sin(state->theta_fp);
    
    uint32_t target_idx = wp_idx;
    uint32_t search_count = (params->lookahead_points < trajectory_size) 
                            ? params->lookahead_points : trajectory_size;
    
    SEARCH_LOOP: for (uint32_t i = 0; i < search_count; i++) {
        #pragma HLS PIPELINE II=1
        #pragma HLS LOOP_TRIPCOUNT min=5 max=20
        
        uint32_t idx = (wp_idx + i) % trajectory_size;
        
        // Vector from vehicle to waypoint
        int32_t dx = fp_sub(trajectory_bram[idx].x_fp, state->x_fp);
        int32_t dy = fp_sub(trajectory_bram[idx].y_fp, state->y_fp);
        
        // Squared distance
        int32_t dist_sq = fp_add(fp_mul(dx, dx), fp_mul(dy, dy));
        
        // Check if ahead of vehicle (forward projection)
        int32_t forward = fp_add(fp_mul(dx, cos_theta), fp_mul(dy, sin_theta));
        
        // Valid target: ahead of us AND at/beyond lookahead
        if (forward > 0 && dist_sq >= lookahead_sq) {
            target_idx = idx;
            break;
        }
        
        // Keep updating target to furthest ahead point
        if (forward > 0) {
            target_idx = idx;
        }
    }
    output->target_waypoint_idx = target_idx;
    
    // =========================================================
    // Step 3: Read target waypoint from BRAM
    // =========================================================
    FpgaWaypoint_t target = trajectory_bram[target_idx];
    FpgaWaypoint_t closest = trajectory_bram[wp_idx];
    
    // =========================================================
    // Step 4: Compute cross-track error (for monitoring)
    // =========================================================
    int32_t dx_close = fp_sub(state->x_fp, closest.x_fp);
    int32_t dy_close = fp_sub(state->y_fp, closest.y_fp);
    
    int32_t sin_path = fp_sin(closest.theta_fp);
    int32_t cos_path = fp_cos(closest.theta_fp);
    
    // CTE = -sin(path_heading) * dx + cos(path_heading) * dy
    int32_t cte = fp_add(
        fp_mul(-sin_path, dx_close),
        fp_mul(cos_path, dy_close)
    );
    output->cross_track_error_fp = cte;
    
    // =========================================================
    // Step 5: Compute heading error
    // =========================================================
    int32_t heading_error = fp_normalize_angle(
        fp_sub(target.theta_fp, state->theta_fp)
    );
    output->heading_error_fp = heading_error;
    
    // =========================================================
    // Step 6: Pure Pursuit steering calculation
    // =========================================================
    int32_t dx = fp_sub(target.x_fp, state->x_fp);
    int32_t dy = fp_sub(target.y_fp, state->y_fp);
    
    // Transform to vehicle frame
    // y_vehicle = -sin(θ) * dx + cos(θ) * dy
    int32_t y_vehicle = fp_add(
        fp_mul(-sin_theta, dx),
        fp_mul(cos_theta, dy)
    );
    
    // Squared distance to target
    int32_t L_sq = fp_add(fp_mul(dx, dx), fp_mul(dy, dy));
    
    int32_t steering = 0;
    if (L_sq > FP_CONST(0.01)) {
        // Pure Pursuit: κ = 2 * y / L²
        int32_t two_y = y_vehicle << 1;
        int32_t curvature = fp_div(two_y, L_sq);
        
        // Steering: δ = atan(κ * wheelbase)
        int32_t kappa_L = fp_mul(curvature, params->wheelbase_fp);
        steering = fp_atan(kappa_L);
        
        // Clamp to limits
        steering = fp_clamp(steering, 
                           fp_neg(params->max_steering_fp), 
                           params->max_steering_fp);
    }
    output->steering_angle_fp = steering;
    
    // =========================================================
    // Step 7: Velocity from target waypoint
    // =========================================================
    int32_t target_vel = fp_min(target.velocity_fp, params->max_velocity_fp);
    output->velocity_fp = target_vel;
}

/*===========================================================================
 * Top-Level Function (combines load + compute for HLS export)
 *===========================================================================*/

void pure_pursuit_fpga(
    // Mode select: 0 = compute, 1 = load trajectory
    uint32_t mode,
    
    // For trajectory loading (mode=1)
    const FpgaWaypoint_t* waypoints_in,
    uint32_t num_waypoints,
    
    // For computation (mode=0)
    const FpgaStateInput_t* state,
    const FpgaParams_t* params,
    FpgaOutput_t* output,
    
    // Status output
    uint32_t* traj_loaded,
    uint32_t* traj_size_out
)
{
    #pragma HLS INTERFACE s_axilite port=mode bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=waypoints_in bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=num_waypoints bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=state bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=params bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=output bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=traj_loaded bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=traj_size_out bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=return bundle=CTRL
    
    if (mode == 1) {
        // Load trajectory mode
        load_trajectory(waypoints_in, num_waypoints);
    } else {
        // Compute mode
        pure_pursuit_compute(state, params, output);
    }
    
    // Return status½
    *traj_loaded = trajectory_loaded;
    *traj_size_out = trajectory_size;
}
