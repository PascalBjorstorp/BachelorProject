/**
 * @file mpc_fpga_top.c
 * @brief Top-Level HLS Function with AXI-Lite Interface
 *
 * This is the synthesizable top function for the MPC FPGA IP core.
 * It wraps the MPC Riccati-ADMM solver with an AXI-Lite register interface
 * for CPU communication on the Ultra96-V2 (Zynq UltraScale+ ZU3EG).
 *
 * No-preload dataflow model:
 *   - CPU writes one horizon frame to memory
 *   - FPGA reads that frame via AXI master in one compute transaction
 *
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
**/
void mpc_fpga_top(
    /* Vehicle state input */
    int state_x_fp,
    int state_theta_fp,
    int state_vx_fp,
    int state_vy_fp,
    int state_omega_fp,
    int state_steering_fp,

    /* Bulk horizon references in external memory */
    const int *ref_vx_mem,
    const int *ref_kappa_mem,
    const int *ref_left_bound_mem,
    const int *ref_right_bound_mem,
    int ref_count,

    /* Outputs (mode=0) */
    int *out_steering_fp,
    int *out_accel_fp,
    int *out_status,
    int *out_iterations)
{
    /* ===== AXI-Lite Interface Pragmas ===== */
#pragma HLS INTERFACE s_axilite port=return          bundle=ctrl
#pragma HLS INTERFACE s_axilite port=state_x_fp      bundle=ctrl
#pragma HLS INTERFACE s_axilite port=state_theta_fp  bundle=ctrl
#pragma HLS INTERFACE s_axilite port=state_vx_fp     bundle=ctrl
#pragma HLS INTERFACE s_axilite port=state_vy_fp     bundle=ctrl
#pragma HLS INTERFACE s_axilite port=state_omega_fp  bundle=ctrl
#pragma HLS INTERFACE s_axilite port=state_steering_fp bundle=ctrl
#pragma HLS INTERFACE s_axilite port=ref_vx_mem      bundle=ctrl
#pragma HLS INTERFACE s_axilite port=ref_kappa_mem   bundle=ctrl
#pragma HLS INTERFACE s_axilite port=ref_left_bound_mem bundle=ctrl
#pragma HLS INTERFACE s_axilite port=ref_right_bound_mem bundle=ctrl
#pragma HLS INTERFACE s_axilite port=ref_count       bundle=ctrl
#pragma HLS INTERFACE s_axilite port=out_steering_fp bundle=ctrl
#pragma HLS INTERFACE s_axilite port=out_accel_fp    bundle=ctrl
#pragma HLS INTERFACE s_axilite port=out_status      bundle=ctrl
#pragma HLS INTERFACE s_axilite port=out_iterations  bundle=ctrl

#pragma HLS INTERFACE m_axi port=ref_vx_mem offset=slave bundle=gmem0 depth=1024
#pragma HLS INTERFACE m_axi port=ref_kappa_mem offset=slave bundle=gmem1 depth=1024
#pragma HLS INTERFACE m_axi port=ref_left_bound_mem offset=slave bundle=gmem2 depth=1024
#pragma HLS INTERFACE m_axi port=ref_right_bound_mem offset=slave bundle=gmem3 depth=1024

#pragma HLS ALLOCATION operation instances=mul limit=2

    /* ===== Static Persistent State (survives between calls) ===== */

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
        first_call = 0;
    }

    /* --- Compute MPC control from one bulk horizon frame --- */
    int horizon_size = ref_count;
    if (horizon_size <= 0) {
        *out_steering_fp = 0;
        *out_accel_fp    = 0;
        *out_status      = 3;  /* NO_TRAJECTORY */
        *out_iterations  = 0;
        return;
    }
    if (horizon_size > MAX_TRAJECTORY_SIZE) horizon_size = MAX_TRAJECTORY_SIZE;

    /* Update actual steering from CPU measurement */
    persist.actual_steering = (fixed_point_t)state_steering_fp;

    /* ARM precomputes Frenet errors and passes them in registers.
     * state_x_fp: e_y, state_theta_fp: e_psi */
    fixed_point_t e_y = (fixed_point_t)state_x_fp;
    fixed_point_t e_psi = fp_normalize_angle((fixed_point_t)state_theta_fp);

    fixed_point_t vx    = (fixed_point_t)state_vx_fp;
    fixed_point_t vy    = (fixed_point_t)state_vy_fp;
    fixed_point_t omega = (fixed_point_t)state_omega_fp;

    MpcRefPoint_t ref[MPC_HORIZON];
#pragma HLS ARRAY_PARTITION variable=ref complete dim=0

    int k;
    for (k = 0; k < MPC_HORIZON; k++) {
#pragma HLS PIPELINE II=1
        int idx = k;
        if (idx >= horizon_size) idx = horizon_size - 1;

        ref[k].velocity = (fixed_point_t)ref_vx_mem[idx];

        fixed_point_t kappa = (fixed_point_t)ref_kappa_mem[idx];
        if (kappa > FP_CONST(1.5))  kappa = FP_CONST(1.5);
        if (kappa < FP_CONST(-1.5)) kappa = FP_CONST(-1.5);
        ref[k].kappa = kappa;

        ref[k].left_bound  = (fixed_point_t)ref_left_bound_mem[idx];
        ref[k].right_bound = (fixed_point_t)ref_right_bound_mem[idx];
    }

    fixed_point_t steer_out, accel_out;
    int status, iters;

    mpc_compute_hls(
        e_y, e_psi, vx, vy, omega,
        ref, &persist, &admm_state,
        &steer_out, &accel_out,
        &status, &iters);

    *out_steering_fp = (int)steer_out;
    *out_accel_fp    = (int)accel_out;
    *out_status      = status;
    *out_iterations  = iters;
}
