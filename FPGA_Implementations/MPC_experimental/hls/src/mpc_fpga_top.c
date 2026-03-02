/**
 * @file mpc_fpga_top.c
 * @brief HLS Top-Level Wrapper for MPC FPGA IP Core
 *
 * This file is the single translation unit for Vitis HLS synthesis.
 * It #includes all MPC core source files so HLS can optimize across
 * function boundaries (inlining, scheduling, resource sharing).
 *
 * The top-level function `mpc_fpga` provides a scalar AXI-Lite interface
 * for CPU<->FPGA communication, with internal BRAM for trajectory storage.
 *
 * Build for HLS:
 *   vitis_hls -f run_hls.tcl
 *
 * Build for CPU test:
 *   gcc -DMPC_HLS_TARGET -I./include -I./hls/include \
 *       hls/src/mpc_fpga_top.c -o test_hls_top -lm
 */

/* Must be defined before includes to activate HLS pragmas */
#ifndef MPC_HLS_TARGET
#define MPC_HLS_TARGET
#endif

/*===========================================================================
 * Include all MPC core sources (single translation unit for HLS)
 *===========================================================================*/
#include "../../src/fp_math.c"
#include "../../src/vehicle_model.c"
#include "../../src/qp_solver.c"
#include "../../src/mpc.c"

/*===========================================================================
 * HLS Interface Header
 *===========================================================================*/
#include "../include/mpc_fpga_interface.h"

#include <string.h>

/*===========================================================================
 * Internal BRAM for Trajectory Storage
 *===========================================================================
 * The full raceline trajectory is stored in FPGA BRAM.
 * CPU loads waypoints one at a time via mode=1, then finalizes with mode=2.
 * During mode=0 (compute), the MPC reads N consecutive waypoints starting
 * from the closest waypoint index provided by the CPU.
 */

static MpcFpgaWaypoint_t trajectory_bram[MPC_FPGA_MAX_TRAJECTORY_SIZE];
static uint32_t trajectory_size    = 0;
static uint32_t trajectory_loaded  = 0;
static uint32_t mpc_fpga_initialized = 0;

/*===========================================================================
 * Internal: Store one waypoint into BRAM
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
 * Internal: Build MPC reference trajectory from BRAM
 *===========================================================================
 * Reads N consecutive waypoints starting from wp_start_index,
 * wrapping around the trajectory for closed-loop tracks.
 */
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
 * Internal: MPC Compute
 *===========================================================================*/
static void compute_mpc_control(
    /* Vehicle state (Frenet) */
    int32_t st_lat_error, int32_t st_heading_error,
    int32_t st_vx, int32_t st_vy, int32_t st_omega,
    uint32_t st_wp_index,
    /* Outputs */
    int32_t *out_steering, int32_t *out_acceleration,
    uint32_t *out_status, uint32_t *out_iterations,
    int32_t *out_cost)
{
#pragma HLS INLINE

    /* Default outputs */
    *out_steering     = 0;
    *out_acceleration = 0;
    *out_status       = MPC_FPGA_STATUS_NO_TRAJECTORY;
    *out_iterations   = 0;
    *out_cost         = 0;

    /* Check trajectory is loaded */
    if (!trajectory_loaded || trajectory_size == 0) {
        *out_status = MPC_FPGA_STATUS_NO_TRAJECTORY;
        return;
    }

    /* Ensure MPC is initialized */
    if (!mpc_fpga_initialized) {
        mpc_initialize();
        mpc_fpga_initialized = 1;
    }

    /* Build Frenet state */
    FrenetState_t frenet_state;
    frenet_state.lateral_error_meters                   = st_lat_error;
    frenet_state.heading_error_radians                  = st_heading_error;
    frenet_state.longitudinal_velocity_meters_per_second = st_vx;
    frenet_state.lateral_velocity_meters_per_second     = st_vy;
    frenet_state.yaw_rate_radians_per_second            = st_omega;

    /* Build reference trajectory from BRAM */
    TrajectoryReferencePoint_t reference[MPC_FPGA_HORIZON];
    build_reference_from_bram(st_wp_index, reference, MPC_FPGA_HORIZON);

    /* Run MPC solver */
    MpcSolverResult_t result;
    MpcSolverStatus_t status = mpc_compute_optimal_control(
        &frenet_state, reference, &result);

    /* Write outputs */
    *out_steering     = result.optimal_control.steering_angle_radians;
    *out_acceleration = result.optimal_control.acceleration_meters_per_second_squared;
    *out_status       = (uint32_t)status;
    *out_iterations   = result.iterations_used;
    *out_cost         = result.final_cost;
}

/*===========================================================================
 * Top-Level Function (HLS synthesis entry point)
 *===========================================================================
 * All arguments are scalars → compatible with AXI-Lite and RTL co-sim.
 *
 * mode=0: Compute MPC control from Frenet vehicle state
 * mode=1: Store one waypoint at wp_index
 * mode=2: Finalize trajectory loading (set size)
 * mode=3: Reset MPC solver state
 */
void mpc_fpga(
    /* Control */
    uint32_t mode,

    /* Waypoint loading (mode=1) */
    uint32_t wp_index,
    int32_t  wp_ref_lat_error,
    int32_t  wp_ref_heading_error,
    int32_t  wp_ref_velocity,
    int32_t  wp_ref_lat_vel,
    int32_t  wp_ref_yaw_rate,
    int32_t  wp_curvature,
    int32_t  wp_left_wall,
    int32_t  wp_right_wall,
    uint32_t wp_total,

    /* Vehicle state in Frenet frame (mode=0) */
    int32_t  st_lateral_error,
    int32_t  st_heading_error,
    int32_t  st_vx,
    int32_t  st_vy,
    int32_t  st_omega,
    uint32_t st_wp_index,

    /* Outputs (mode=0) */
    int32_t  *out_steering,
    int32_t  *out_acceleration,
    uint32_t *out_status,
    uint32_t *out_iterations,
    int32_t  *out_cost,
    uint32_t *out_traj_loaded,
    uint32_t *out_traj_size)
{
    /*-----------------------------------------------------------------------
     * AXI-Lite Interface: all ports on single CTRL bundle
     *-----------------------------------------------------------------------*/
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

    /*-----------------------------------------------------------------------
     * BRAM storage for trajectory
     *-----------------------------------------------------------------------*/
#pragma HLS BIND_STORAGE variable=trajectory_bram type=ram_2p impl=bram

    /*-----------------------------------------------------------------------
     * Mode dispatch
     *-----------------------------------------------------------------------*/
    if (mode == MPC_FPGA_MODE_LOAD_WP) {
        /* Mode 1: Load one waypoint into BRAM */
        store_waypoint(wp_index,
                       wp_ref_lat_error, wp_ref_heading_error,
                       wp_ref_velocity, wp_ref_lat_vel, wp_ref_yaw_rate,
                       wp_curvature, wp_left_wall, wp_right_wall);

    } else if (mode == MPC_FPGA_MODE_FINALIZE) {
        /* Mode 2: Finalize trajectory */
        trajectory_size = (wp_total > MPC_FPGA_MAX_TRAJECTORY_SIZE)
                          ? MPC_FPGA_MAX_TRAJECTORY_SIZE : wp_total;
        trajectory_loaded = 1;

        /* Initialize MPC on trajectory finalize */
        if (!mpc_fpga_initialized) {
            mpc_initialize();
            mpc_fpga_initialized = 1;
        }

    } else if (mode == MPC_FPGA_MODE_RESET) {
        /* Mode 3: Reset solver state */
        mpc_reset();

    } else {
        /* Mode 0: Compute MPC control */
        compute_mpc_control(
            st_lateral_error, st_heading_error, st_vx, st_vy, st_omega,
            st_wp_index,
            out_steering, out_acceleration,
            out_status, out_iterations, out_cost);
    }

    /* Always return trajectory status */
    *out_traj_loaded = trajectory_loaded;
    *out_traj_size   = trajectory_size;
}
