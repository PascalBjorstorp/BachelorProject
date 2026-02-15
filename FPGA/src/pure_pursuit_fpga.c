/**
 * @file pure_pursuit_fpga.c
 * @brief Pure Pursuit with BRAM-Stored Trajectory (Vitis HLS)
 *
 * All arguments are scalars for RTL co-simulation compatibility.
 * The trajectory is stored in internal BRAM and loaded one waypoint
 * at a time via mode=1 calls.
 *
 * Modes:
 *   mode=0: Compute steering from current vehicle state
 *   mode=1: Load one waypoint into BRAM at given index
 *   mode=2: Finalize trajectory (set size, mark as loaded)
 *
 * Interfaces:
 *   - AXI-Lite: All scalar registers in one CTRL bundle
 *   - BRAM: Internal storage for trajectory waypoints
 */

#include "../include/fpga_interface.h"
#include "../include/fp_math_hls.h"

/*===========================================================================
 * Internal BRAM for Trajectory Storage
 *===========================================================================*/

static FpgaWaypoint_t trajectory_bram[MAX_TRAJECTORY_SIZE];

static uint32_t trajectory_size = 0;
static uint32_t trajectory_loaded = 0;

/*===========================================================================
 * Internal: Store one waypoint into BRAM
 *===========================================================================*/
static void store_waypoint(
    uint32_t index,
    int32_t x_fp, int32_t y_fp, int32_t theta_fp,
    int32_t velocity_fp, int32_t kappa_fp
)
{
#pragma HLS INLINE
    if (index < MAX_TRAJECTORY_SIZE) {
        trajectory_bram[index].x_fp        = x_fp;
        trajectory_bram[index].y_fp        = y_fp;
        trajectory_bram[index].theta_fp    = theta_fp;
        trajectory_bram[index].velocity_fp = velocity_fp;
        trajectory_bram[index].kappa_fp    = kappa_fp;
        trajectory_bram[index].reserved[0] = 0;
        trajectory_bram[index].reserved[1] = 0;
        trajectory_bram[index].reserved[2] = 0;
    }
}

/*===========================================================================
 * Internal: Pure Pursuit Computation (unchanged algorithm)
 *===========================================================================*/
static void compute_steering(
    /* Vehicle state */
    int32_t st_x, int32_t st_y, int32_t st_theta, int32_t st_vel,
    uint32_t st_wp_idx,
    /* Parameters */
    int32_t p_min_la, int32_t p_max_la, int32_t p_la_gain,
    int32_t p_wheelbase, int32_t p_max_steer, int32_t p_max_vel,
    uint32_t p_la_points,
    /* Outputs */
    int32_t* out_steering, int32_t* out_velocity,
    int32_t* out_cte, int32_t* out_heading_err,
    int32_t* out_lookahead, uint32_t* out_target_wp,
    uint32_t* out_status
)
{
#pragma HLS INLINE

    /* Default outputs */
    *out_steering    = 0;
    *out_velocity    = 0;
    *out_cte         = 0;
    *out_heading_err = 0;
    *out_lookahead   = 0;
    *out_target_wp   = 0;
    *out_status      = STATUS_OK;

    /* Check trajectory is loaded */
    if (!trajectory_loaded || trajectory_size == 0) {
        *out_status = STATUS_NO_TRAJECTORY;
        return;
    }

    /* Validate waypoint index */
    uint32_t wp_idx = st_wp_idx;
    if (wp_idx >= trajectory_size) {
        wp_idx = wp_idx % trajectory_size;
    }

    /* Step 1: Adaptive lookahead distance */
    int32_t speed = fp_abs(st_vel);
    int32_t lookahead = fp_add(p_min_la, fp_mul(p_la_gain, speed));
    lookahead = fp_clamp(lookahead, p_min_la, p_max_la);
    *out_lookahead = lookahead;

    /* Step 2: Find target waypoint at lookahead distance */
    int32_t lookahead_sq = fp_mul(lookahead, lookahead);
    int32_t cos_theta = fp_cos(st_theta);
    int32_t sin_theta = fp_sin(st_theta);

    uint32_t target_idx = wp_idx;
    uint32_t search_count = (p_la_points < trajectory_size)
                            ? p_la_points : trajectory_size;

    SEARCH_LOOP: for (uint32_t i = 0; i < search_count; i++) {
        #pragma HLS PIPELINE II=1
        #pragma HLS LOOP_TRIPCOUNT min=5 max=20

        uint32_t idx = (wp_idx + i) % trajectory_size;

        int32_t dx = fp_sub(trajectory_bram[idx].x_fp, st_x);
        int32_t dy = fp_sub(trajectory_bram[idx].y_fp, st_y);
        int32_t dist_sq = fp_add(fp_mul(dx, dx), fp_mul(dy, dy));
        int32_t forward = fp_add(fp_mul(dx, cos_theta), fp_mul(dy, sin_theta));

        if (forward > 0 && dist_sq >= lookahead_sq) {
            target_idx = idx;
            break;
        }
        if (forward > 0) {
            target_idx = idx;
        }
    }
    *out_target_wp = target_idx;

    /* Step 3: Read target and closest waypoints from BRAM */
    FpgaWaypoint_t target  = trajectory_bram[target_idx];
    FpgaWaypoint_t closest = trajectory_bram[wp_idx];

    /* Step 4: Cross-track error */
    int32_t dx_close = fp_sub(st_x, closest.x_fp);
    int32_t dy_close = fp_sub(st_y, closest.y_fp);
    int32_t sin_path = fp_sin(closest.theta_fp);
    int32_t cos_path = fp_cos(closest.theta_fp);
    *out_cte = fp_add(fp_mul(-sin_path, dx_close), fp_mul(cos_path, dy_close));

    /* Step 5: Heading error */
    *out_heading_err = fp_normalize_angle(fp_sub(target.theta_fp, st_theta));

    /* Step 6: Pure Pursuit steering */
    int32_t dx = fp_sub(target.x_fp, st_x);
    int32_t dy = fp_sub(target.y_fp, st_y);
    int32_t y_vehicle = fp_add(fp_mul(-sin_theta, dx), fp_mul(cos_theta, dy));
    int32_t L_sq = fp_add(fp_mul(dx, dx), fp_mul(dy, dy));

    int32_t steering = 0;
    if (L_sq > FP_CONST(0.01)) {
        int32_t two_y = y_vehicle << 1;
        int32_t curvature = fp_div(two_y, L_sq);
        int32_t kappa_L = fp_mul(curvature, p_wheelbase);
        steering = fp_atan(kappa_L);
        steering = fp_clamp(steering, fp_neg(p_max_steer), p_max_steer);
    }
    *out_steering = steering;

    /* Step 7: Velocity from target waypoint */
    *out_velocity = fp_min(target.velocity_fp, p_max_vel);
}

/*===========================================================================
 * Top-Level Function (HLS synthesis entry point)
 *
 * All arguments are scalars → compatible with RTL co-simulation.
 *
 * mode=0: Compute steering/velocity from vehicle state
 * mode=1: Store one waypoint at wp_index
 * mode=2: Finalize trajectory loading (set size)
 *===========================================================================*/

void pure_pursuit_fpga(
    /* Control */
    uint32_t mode,

    /* Waypoint loading (mode=1): one waypoint per call */
    uint32_t wp_index,
    int32_t wp_x, int32_t wp_y, int32_t wp_theta,
    int32_t wp_vel, int32_t wp_kappa,
    uint32_t wp_total,

    /* Vehicle state (mode=0) */
    int32_t st_x, int32_t st_y, int32_t st_theta, int32_t st_vel,
    uint32_t st_wp_idx,

    /* Parameters (mode=0) */
    int32_t p_min_la, int32_t p_max_la, int32_t p_la_gain,
    int32_t p_wheelbase, int32_t p_max_steer, int32_t p_max_vel,
    uint32_t p_la_points,

    /* Outputs */
    int32_t* out_steering, int32_t* out_velocity,
    int32_t* out_cte, int32_t* out_heading_err,
    int32_t* out_lookahead, uint32_t* out_target_wp,
    uint32_t* out_status,
    uint32_t* out_traj_loaded, uint32_t* out_traj_size
)
{
    /* AXI-Lite interface for all ports */
    #pragma HLS INTERFACE s_axilite port=mode           bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=wp_index       bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=wp_x           bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=wp_y           bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=wp_theta       bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=wp_vel         bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=wp_kappa       bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=wp_total       bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=st_x           bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=st_y           bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=st_theta       bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=st_vel         bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=st_wp_idx      bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=p_min_la       bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=p_max_la       bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=p_la_gain      bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=p_wheelbase    bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=p_max_steer    bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=p_max_vel      bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=p_la_points    bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=out_steering   bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=out_velocity   bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=out_cte        bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=out_heading_err bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=out_lookahead  bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=out_target_wp  bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=out_status     bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=out_traj_loaded bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=out_traj_size  bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=return         bundle=CTRL

    #pragma HLS BIND_STORAGE variable=trajectory_bram type=ram_2p impl=bram

    if (mode == 1) {
        /* Load one waypoint */
        store_waypoint(wp_index, wp_x, wp_y, wp_theta, wp_vel, wp_kappa);
    } else if (mode == 2) {
        /* Finalize: set trajectory size and mark as loaded */
        trajectory_size = (wp_total > MAX_TRAJECTORY_SIZE)
                          ? MAX_TRAJECTORY_SIZE : wp_total;
        trajectory_loaded = 1;
    } else {
        /* Compute steering */
        compute_steering(
            st_x, st_y, st_theta, st_vel, st_wp_idx,
            p_min_la, p_max_la, p_la_gain,
            p_wheelbase, p_max_steer, p_max_vel, p_la_points,
            out_steering, out_velocity,
            out_cte, out_heading_err,
            out_lookahead, out_target_wp, out_status
        );
    }

    /* Always return trajectory status */
    *out_traj_loaded = trajectory_loaded;
    *out_traj_size   = trajectory_size;
}
