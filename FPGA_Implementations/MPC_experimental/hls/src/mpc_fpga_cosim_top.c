/**
 * @file mpc_fpga_cosim_top.c
 * @brief Separate Translation Unit Top-Level for RTL Co-Simulation
 *
 * Unlike mpc_fpga_top.c (which #includes all sources as one TU for
 * synthesis optimization), this file only contains the HLS top-level
 * wrapper and BRAM management. The core MPC functions are compiled as
 * separate TUs so the Vitis HLS co-sim instrumenter can handle them.
 *
 * This file is used by run_cosim.tcl alongside the individual .c files.
 */

/* Must be defined before includes */
#ifndef MPC_HLS_TARGET
#define MPC_HLS_TARGET
#endif

#include "../../include/mpc.h"
#include "../../include/mpc_types.h"
#include "../../include/fp_math.h"
#include "../include/mpc_fpga_interface.h"

#include <string.h>

/*===========================================================================
 * Internal BRAM for Trajectory Storage
 *===========================================================================*/

static MpcFpgaWaypoint_t trajectory_bram[MPC_FPGA_MAX_TRAJECTORY_SIZE];
static uint32_t trajectory_size    = 0;
static uint32_t trajectory_loaded  = 0;
static uint32_t mpc_fpga_initialized = 0;

/*===========================================================================
 * Internal: Store one waypoint
 *===========================================================================*/
static void store_waypoint(
    uint32_t index,
    int32_t ref_lat_error, int32_t ref_heading_error,
    int32_t ref_velocity, int32_t ref_lat_vel, int32_t ref_yaw_rate,
    int32_t curvature, int32_t left_wall, int32_t right_wall)
{
#pragma HLS INLINE
    if (index < MPC_FPGA_MAX_TRAJECTORY_SIZE) {
        trajectory_bram[index].ref_lateral_error_fp    = ref_lat_error;
        trajectory_bram[index].ref_heading_error_fp    = ref_heading_error;
        trajectory_bram[index].ref_velocity_fp         = ref_velocity;
        trajectory_bram[index].ref_lateral_velocity_fp = ref_lat_vel;
        trajectory_bram[index].ref_yaw_rate_fp         = ref_yaw_rate;
        trajectory_bram[index].curvature_fp            = curvature;
        trajectory_bram[index].left_wall_fp            = left_wall;
        trajectory_bram[index].right_wall_fp           = right_wall;
        trajectory_bram[index].reserved                = 0;
    }
}

/*===========================================================================
 * Internal: Build reference from BRAM
 *===========================================================================*/
static void build_reference_from_bram(
    uint32_t wp_start_index,
    TrajectoryReferencePoint_t *ref_out,
    uint32_t horizon)
{
#pragma HLS INLINE off

    BUILD_REF_LOOP:
    for (uint32_t i = 0; i < horizon; i++) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=20 max=20
        uint32_t idx = (wp_start_index + i) % trajectory_size;

        ref_out[i].reference_lateral_error_meters           = trajectory_bram[idx].ref_lateral_error_fp;
        ref_out[i].reference_heading_error_radians          = trajectory_bram[idx].ref_heading_error_fp;
        ref_out[i].reference_velocity_meters_per_second     = trajectory_bram[idx].ref_velocity_fp;
        ref_out[i].reference_lateral_velocity_meters_per_second = trajectory_bram[idx].ref_lateral_velocity_fp;
        ref_out[i].reference_yaw_rate_radians_per_second    = trajectory_bram[idx].ref_yaw_rate_fp;
        ref_out[i].path_curvature_radians_per_meter         = trajectory_bram[idx].curvature_fp;
        ref_out[i].left_wall_bound_meters                   = trajectory_bram[idx].left_wall_fp;
        ref_out[i].right_wall_bound_meters                  = trajectory_bram[idx].right_wall_fp;
    }
}

/*===========================================================================
 * Internal: MPC compute
 *===========================================================================*/
static void compute_mpc_control(
    int32_t st_lat_error, int32_t st_heading_error,
    int32_t st_vx, int32_t st_vy, int32_t st_omega,
    uint32_t st_wp_index,
    int32_t *out_steering, int32_t *out_acceleration,
    uint32_t *out_status, uint32_t *out_iterations,
    int32_t *out_cost)
{
#pragma HLS INLINE

    *out_steering     = 0;
    *out_acceleration = 0;
    *out_status       = MPC_FPGA_STATUS_NO_TRAJECTORY;
    *out_iterations   = 0;
    *out_cost         = 0;

    if (!trajectory_loaded || trajectory_size == 0) {
        return;
    }

    if (!mpc_fpga_initialized) {
        mpc_initialize();
        mpc_fpga_initialized = 1;
    }

    FrenetState_t frenet_state;
    frenet_state.lateral_error_meters                   = st_lat_error;
    frenet_state.heading_error_radians                  = st_heading_error;
    frenet_state.longitudinal_velocity_meters_per_second = st_vx;
    frenet_state.lateral_velocity_meters_per_second     = st_vy;
    frenet_state.yaw_rate_radians_per_second            = st_omega;

    TrajectoryReferencePoint_t reference[MPC_FPGA_HORIZON];
    build_reference_from_bram(st_wp_index, reference, MPC_FPGA_HORIZON);

    MpcSolverResult_t result;
    MpcSolverStatus_t status = mpc_compute_optimal_control(
        &frenet_state, reference, &result);

    *out_steering     = result.optimal_control.steering_angle_radians;
    *out_acceleration = result.optimal_control.acceleration_meters_per_second_squared;
    *out_status       = (uint32_t)status;
    *out_iterations   = result.iterations_used;
    *out_cost         = result.final_cost;
}

/*===========================================================================
 * Top-Level Function
 *===========================================================================*/
void mpc_fpga(
    uint32_t mode,
    uint32_t wp_index,
    int32_t  wp_ref_lat_error, int32_t wp_ref_heading_error,
    int32_t  wp_ref_velocity, int32_t wp_ref_lat_vel, int32_t wp_ref_yaw_rate,
    int32_t  wp_curvature, int32_t wp_left_wall, int32_t wp_right_wall,
    uint32_t wp_total,
    int32_t  st_lateral_error, int32_t st_heading_error,
    int32_t  st_vx, int32_t st_vy, int32_t st_omega,
    uint32_t st_wp_index,
    int32_t  *out_steering, int32_t *out_acceleration,
    uint32_t *out_status, uint32_t *out_iterations, int32_t *out_cost,
    uint32_t *out_traj_loaded, uint32_t *out_traj_size)
{
#pragma HLS INTERFACE s_axilite port=mode              bundle=CTRL
#pragma HLS INTERFACE s_axilite port=wp_index          bundle=CTRL
#pragma HLS INTERFACE s_axilite port=wp_ref_lat_error  bundle=CTRL
#pragma HLS INTERFACE s_axilite port=wp_ref_heading_error bundle=CTRL
#pragma HLS INTERFACE s_axilite port=wp_ref_velocity   bundle=CTRL
#pragma HLS INTERFACE s_axilite port=wp_ref_lat_vel    bundle=CTRL
#pragma HLS INTERFACE s_axilite port=wp_ref_yaw_rate   bundle=CTRL
#pragma HLS INTERFACE s_axilite port=wp_curvature      bundle=CTRL
#pragma HLS INTERFACE s_axilite port=wp_left_wall      bundle=CTRL
#pragma HLS INTERFACE s_axilite port=wp_right_wall     bundle=CTRL
#pragma HLS INTERFACE s_axilite port=wp_total          bundle=CTRL
#pragma HLS INTERFACE s_axilite port=st_lateral_error  bundle=CTRL
#pragma HLS INTERFACE s_axilite port=st_heading_error  bundle=CTRL
#pragma HLS INTERFACE s_axilite port=st_vx             bundle=CTRL
#pragma HLS INTERFACE s_axilite port=st_vy             bundle=CTRL
#pragma HLS INTERFACE s_axilite port=st_omega          bundle=CTRL
#pragma HLS INTERFACE s_axilite port=st_wp_index       bundle=CTRL
#pragma HLS INTERFACE s_axilite port=out_steering      bundle=CTRL
#pragma HLS INTERFACE s_axilite port=out_acceleration  bundle=CTRL
#pragma HLS INTERFACE s_axilite port=out_status        bundle=CTRL
#pragma HLS INTERFACE s_axilite port=out_iterations    bundle=CTRL
#pragma HLS INTERFACE s_axilite port=out_cost          bundle=CTRL
#pragma HLS INTERFACE s_axilite port=out_traj_loaded   bundle=CTRL
#pragma HLS INTERFACE s_axilite port=out_traj_size     bundle=CTRL
#pragma HLS INTERFACE s_axilite port=return            bundle=CTRL

#pragma HLS BIND_STORAGE variable=trajectory_bram type=ram_2p impl=bram

    if (mode == MPC_FPGA_MODE_LOAD_WP) {
        store_waypoint(wp_index,
                       wp_ref_lat_error, wp_ref_heading_error,
                       wp_ref_velocity, wp_ref_lat_vel, wp_ref_yaw_rate,
                       wp_curvature, wp_left_wall, wp_right_wall);
    } else if (mode == MPC_FPGA_MODE_FINALIZE) {
        trajectory_size = (wp_total > MPC_FPGA_MAX_TRAJECTORY_SIZE)
                          ? MPC_FPGA_MAX_TRAJECTORY_SIZE : wp_total;
        trajectory_loaded = 1;
        if (!mpc_fpga_initialized) {
            mpc_initialize();
            mpc_fpga_initialized = 1;
        }
    } else if (mode == MPC_FPGA_MODE_RESET) {
        mpc_reset();
    } else {
        compute_mpc_control(
            st_lateral_error, st_heading_error,
            st_vx, st_vy, st_omega,
            st_wp_index,
            out_steering, out_acceleration,
            out_status, out_iterations, out_cost);
    }

    *out_traj_loaded = trajectory_loaded;
    *out_traj_size   = trajectory_size;
}
