/**
 * @file mpc_fpga_top.c
 * @brief Top-Level HLS Function with AXI-Lite Interface
 *
 * This is the synthesizable top function for the MPC FPGA IP core.
 * It wraps the MPC Riccati-ADMM solver with an AXI-Lite register interface
 * for CPU communication on the Ultra96-V2 (Zynq UltraScale+ ZU3EG).
 *
 * Modes:
 *   0 = Compute: convert state to Frenet, run MPC, return control
 *   1 = Load waypoint: store one trajectory point in internal BRAM
 *   2 = Finalize: set total waypoint count
 *
 * All persistent state (trajectory, ADMM warm-start, previous control)
 * is stored in static local arrays inside this function, ensuring
 * it persists across HLS function calls.
 */

#include "../include/fp_math_hls.h"
#include "../include/mpc_fpga_types.h"
#include "../include/riccati_solver_hls.h"

/* Forward declarations */
extern void mpc_compute_hls(
    fixed_point_t state_ey,
    fixed_point_t state_epsi,
    fixed_point_t state_vx,
    fixed_point_t state_vy,
    fixed_point_t state_omega,
    const MpcRefPoint_t ref[MPC_HORIZON],
    MpcPersistState_t *persist,
    AdmmState_t *admm_state,
    fixed_point_t *out_steering,
    fixed_point_t *out_accel,
    int *out_status,
    int *out_iters);

/**
 * MPC FPGA Top-Level Function.
 *
 * All parameters are AXI-Lite registers on the 'ctrl' bundle.
 * The HLS tool generates the register map automatically.
 */
void mpc_fpga_top(
    /* Mode control */
    int mode,               /* 0=compute, 1=load_waypoint, 2=finalize */

    /* Waypoint loading (mode=1) */
    int wp_index,
    int wp_x_fp,
    int wp_y_fp,
    int wp_psi_fp,
    int wp_vx_fp,
    int wp_kappa_fp,
    int wp_ax_fp,
    int wp_left_bound_fp,
    int wp_right_bound_fp,
    int wp_total,           /* Used in mode=2 */

    /* Vehicle state input (mode=0) */
    int state_x_fp,
    int state_y_fp,
    int state_theta_fp,
    int state_vx_fp,
    int state_vy_fp,
    int state_omega_fp,
    int state_steering_fp,  /* Actual servo position */
    int state_wp_idx,       /* Closest waypoint index */

    /* Outputs (mode=0) */
    int *out_steering_fp,
    int *out_accel_fp,
    int *out_status,
    int *out_iterations)
{
    /* ===== AXI-Lite Interface Pragmas ===== */
#pragma HLS INTERFACE s_axilite port=return          bundle=ctrl
#pragma HLS INTERFACE s_axilite port=mode            bundle=ctrl
#pragma HLS INTERFACE s_axilite port=wp_index        bundle=ctrl
#pragma HLS INTERFACE s_axilite port=wp_x_fp         bundle=ctrl
#pragma HLS INTERFACE s_axilite port=wp_y_fp         bundle=ctrl
#pragma HLS INTERFACE s_axilite port=wp_psi_fp       bundle=ctrl
#pragma HLS INTERFACE s_axilite port=wp_vx_fp        bundle=ctrl
#pragma HLS INTERFACE s_axilite port=wp_kappa_fp     bundle=ctrl
#pragma HLS INTERFACE s_axilite port=wp_ax_fp        bundle=ctrl
#pragma HLS INTERFACE s_axilite port=wp_left_bound_fp  bundle=ctrl
#pragma HLS INTERFACE s_axilite port=wp_right_bound_fp bundle=ctrl
#pragma HLS INTERFACE s_axilite port=wp_total        bundle=ctrl
#pragma HLS INTERFACE s_axilite port=state_x_fp      bundle=ctrl
#pragma HLS INTERFACE s_axilite port=state_y_fp      bundle=ctrl
#pragma HLS INTERFACE s_axilite port=state_theta_fp  bundle=ctrl
#pragma HLS INTERFACE s_axilite port=state_vx_fp     bundle=ctrl
#pragma HLS INTERFACE s_axilite port=state_vy_fp     bundle=ctrl
#pragma HLS INTERFACE s_axilite port=state_omega_fp  bundle=ctrl
#pragma HLS INTERFACE s_axilite port=state_steering_fp bundle=ctrl
#pragma HLS INTERFACE s_axilite port=state_wp_idx    bundle=ctrl
#pragma HLS INTERFACE s_axilite port=out_steering_fp bundle=ctrl
#pragma HLS INTERFACE s_axilite port=out_accel_fp    bundle=ctrl
#pragma HLS INTERFACE s_axilite port=out_status      bundle=ctrl
#pragma HLS INTERFACE s_axilite port=out_iterations  bundle=ctrl

    /* Limit top-level multiplier instances: Frenet conversion and waypoint advance
     * only run once per MPC solve (<0.1% of runtime). Free DSP for Riccati solver. */
#pragma HLS ALLOCATION operation instances=mul limit=2

    /* ===== Static Persistent State (survives between calls) ===== */

    /* Trajectory waypoints in BRAM */
    static MpcWaypoint_t trajectory[MAX_TRAJECTORY_SIZE];
#pragma HLS BIND_STORAGE variable=trajectory type=ram_2p impl=bram
    static int trajectory_size = 0;
    static int trajectory_loaded = 0;

    /* ADMM warm-start state */
    static AdmmState_t admm_state;
#pragma HLS BIND_STORAGE variable=admm_state type=ram_2p impl=bram

    /* MPC persistent state */
    static MpcPersistState_t persist;
#pragma HLS BIND_STORAGE variable=persist type=register
    static int first_call = 1;

    /* ===== Initialize on first call ===== */
    if (first_call) {
        persist.prev_steer_rate = 0;
        persist.prev_accel = 0;
        persist.prev_delta_cmd = 0;
        persist.actual_steering = 0;
        persist.prev_curvature = 0;
        persist.prev_converged = 0;
        admm_state.initialized = 0;
        trajectory_size = 0;
        trajectory_loaded = 0;
        first_call = 0;
    }

    /* ===== Mode Switch ===== */

    if (mode == 1) {
        /* --- Mode 1: Load one waypoint into BRAM --- */
        if (wp_index >= 0 && wp_index < MAX_TRAJECTORY_SIZE) {
            trajectory[wp_index].x           = (fixed_point_t)wp_x_fp;
            trajectory[wp_index].y           = (fixed_point_t)wp_y_fp;
            trajectory[wp_index].psi         = (fixed_point_t)wp_psi_fp;
            trajectory[wp_index].vx          = (fixed_point_t)wp_vx_fp;
            trajectory[wp_index].kappa       = (fixed_point_t)wp_kappa_fp;
            trajectory[wp_index].ax          = (fixed_point_t)wp_ax_fp;
            trajectory[wp_index].left_bound  = (fixed_point_t)wp_left_bound_fp;
            trajectory[wp_index].right_bound = (fixed_point_t)wp_right_bound_fp;
        }
        /* Echo back zeros for output */
        *out_steering_fp = 0;
        *out_accel_fp    = 0;
        *out_status      = 0;
        *out_iterations  = 0;

    } else if (mode == 2) {
        /* --- Mode 2: Finalize trajectory --- */
        trajectory_size = wp_total;
        trajectory_loaded = (trajectory_size > 0) ? 1 : 0;
        /* Reset solver state for new trajectory */
        admm_state.initialized = 0;
        persist.prev_converged = 0;

        *out_steering_fp = 0;
        *out_accel_fp    = 0;
        *out_status      = trajectory_loaded ? 0 : 3;
        *out_iterations  = trajectory_size;

    } else {
        /* --- Mode 0: Compute MPC control --- */

        if (!trajectory_loaded || trajectory_size < MPC_HORIZON) {
            /* No valid trajectory */
            *out_steering_fp = 0;
            *out_accel_fp    = 0;
            *out_status      = 3;  /* NO_TRAJECTORY */
            *out_iterations  = 0;
            return;
        }

        /* Update actual steering from CPU measurement */
        persist.actual_steering = (fixed_point_t)state_steering_fp;

        /* ----- Frenet conversion -----
         * Transform global (x,y,theta) to Frenet (e_y, e_psi)
         * using the closest waypoint. */
        int wp_idx = state_wp_idx;
        if (wp_idx < 0) wp_idx = 0;
        if (wp_idx >= trajectory_size) wp_idx = trajectory_size - 1;

        MpcWaypoint_t closest_wp = trajectory[wp_idx];

        fixed_point_t dx = (fixed_point_t)state_x_fp - closest_wp.x;
        fixed_point_t dy = (fixed_point_t)state_y_fp - closest_wp.y;

        fixed_point_t cos_wpsi = fp_cos(closest_wp.psi);
        fixed_point_t sin_wpsi = fp_sin(closest_wp.psi);

        /* e_y = -dx*sin(psi_wp) + dy*cos(psi_wp) */
        fixed_point_t e_y = fp_add(
            fp_neg(fp_mul(dx, sin_wpsi)),
            fp_mul(dy, cos_wpsi));

        /* e_psi = theta - psi_wp, normalized */
        fixed_point_t e_psi = fp_normalize_angle(
            fp_sub((fixed_point_t)state_theta_fp, closest_wp.psi));

        fixed_point_t vx    = (fixed_point_t)state_vx_fp;
        fixed_point_t vy    = (fixed_point_t)state_vy_fp;
        fixed_point_t omega = (fixed_point_t)state_omega_fp;

        /* ----- Build reference trajectory -----
         * Look ahead MPC_HORIZON waypoints from current position.
         * Velocity-based spacing: at higher speeds, skip more waypoints
         * per prediction step so the horizon covers the correct distance.
         * wp_advance = round(vx * dt / wp_spacing) where dt=MPC_DT, spacing≈0.347m */
        MpcRefPoint_t ref[MPC_HORIZON];
#pragma HLS ARRAY_PARTITION variable=ref complete dim=0

        /* Compute waypoint advance based on reference velocity */
        int wp_advance = 1;
        {
            fixed_point_t ref_vx = trajectory[wp_idx].vx;
            if (ref_vx < FP_CONST(1.0)) ref_vx = FP_CONST(1.0);
            /* ds = vx * dt (prediction dt from MPC_DT) */
            fixed_point_t ds = fp_mul(ref_vx, MPC_DT);
            /* wp_advance = ds / 0.347 ≈ ds * 2.88 */
            fixed_point_t scaled = fp_mul(ds, FP_CONST(2.88));
            wp_advance = (int)(scaled >> FP_FRAC_BITS);
            if (wp_advance < 1) wp_advance = 1;
            if (wp_advance > WP_ADVANCE_MAX) wp_advance = WP_ADVANCE_MAX;  /* Safety cap */
        }

        int k;
        for (k = 0; k < MPC_HORIZON; k++) {
#pragma HLS PIPELINE II=1
            int idx = wp_idx + (k + 1) * wp_advance;
            if (idx >= trajectory_size) idx -= trajectory_size;
            ref[k].velocity    = trajectory[idx].vx;
            /* Clamp kappa to physical limits (prevents corrupted trajectory data
             * from destabilizing the solver: max ~1.5 rad/m for F1Tenth) */
            fixed_point_t kappa = trajectory[idx].kappa;
            if (kappa > FP_CONST(1.5))  kappa = FP_CONST(1.5);
            if (kappa < FP_CONST(-1.5)) kappa = FP_CONST(-1.5);
            ref[k].kappa       = kappa;
            ref[k].left_bound  = trajectory[idx].left_bound;
            ref[k].right_bound = trajectory[idx].right_bound;
        }

        /* ----- Run MPC solver ----- */
        fixed_point_t steer_out, accel_out;
        int status, iters;

        mpc_compute_hls(
            e_y, e_psi, vx, vy, omega,
            ref, &persist, &admm_state,
            &steer_out, &accel_out,
            &status, &iters);

        /* ----- Write outputs ----- */
        *out_steering_fp = (int)steer_out;
        *out_accel_fp    = (int)accel_out;
        *out_status      = status;
        *out_iterations  = iters;
    }
}
