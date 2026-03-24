/**
 * @file mpc_fpga_top.cpp
 * @brief MPC FPGA Top-Level with AXI-Stream Interface
 *
 * Stream Format (128-bit words):
 *   Beat 0: [e_y | e_psi | vx | vy]
 *   Beat 1: [omega | steering | horizon_length | reserved]
 *   Beat 2..N+1: [ref_vx[i] | ref_kappa[i] | ref_left[i] | ref_right[i]]
 *
 * Total: 2 + MPC_HORIZON beats = 21 beats for 19-point horizon
 */

#include "../include/fp_math_hls.h"
#include "../include/mpc_fpga_types.h"
#include "../include/riccati_solver_hls.h"

#ifdef MPC_HLS_BUILD
#include <hls_stream.h>
#include <ap_int.h>
#include <ap_axi_sdata.h>

typedef ap_uint<128> stream_word_t;
typedef hls::axis<stream_word_t, 0, 0, 0> axis_word_t;
#endif

/* Declare the MPC solver core (implemented in mpc_riccati_hls.c) */
extern "C" void mpc_compute_hls(
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
 * Shared compute core with persistent ADMM state across invocations.
 */
static void mpc_fpga_compute_core(
    fixed_point_t ey, fixed_point_t epsi,
    fixed_point_t vx, fixed_point_t vy, fixed_point_t omega,
    fixed_point_t steering,
    const MpcRefPoint_t ref[MPC_HORIZON],
    int *out_steering, int *out_accel, int *out_status, int *out_iters)
{
    static AdmmState_t admm_state;
    static MpcPersistState_t persist;
    static int initialized = 0;
#pragma HLS BIND_STORAGE variable=admm_state type=ram_2p impl=bram
#pragma HLS BIND_STORAGE variable=persist type=ram_1p impl=lutram

    if (!initialized) {
        persist.prev_steer_rate = 0;
        persist.prev_accel = 0;
        persist.prev_delta_cmd = 0;
        persist.actual_steering = 0;
        persist.prev_curvature = 0;
        persist.prev_converged = 0;
        admm_state.initialized = 0;
        initialized = 1;
    }
    persist.actual_steering = steering;

    fixed_point_t steer_out, accel_out;
    int status, iters;
    mpc_compute_hls(ey, epsi, vx, vy, omega, ref, &persist, &admm_state,
                    &steer_out, &accel_out, &status, &iters);

    *out_steering = (int)steer_out;
    *out_accel    = (int)accel_out;
    *out_status   = status;
    *out_iters    = iters;
}

#ifdef MPC_HLS_BUILD
static stream_word_t pack_word(fixed_point_t v0, fixed_point_t v1,
                               fixed_point_t v2, fixed_point_t v3) {
    stream_word_t w = 0;
    w.range(31, 0)   = (uint32_t)v0;
    w.range(63, 32)  = (uint32_t)v1;
    w.range(95, 64)  = (uint32_t)v2;
    w.range(127, 96) = (uint32_t)v3;
    return w;
}

/** AXI-Stream top-level entry point for synthesis. */
void mpc_fpga_top(
    hls::stream<axis_word_t>& input_stream,
    int *out_steering, int *out_accel, int *out_status, int *out_iters)
{
#pragma HLS INTERFACE axis port=input_stream
#pragma HLS INTERFACE s_axilite port=return bundle=ctrl
#pragma HLS INTERFACE s_axilite port=out_steering bundle=ctrl
#pragma HLS INTERFACE s_axilite port=out_accel bundle=ctrl
#pragma HLS INTERFACE s_axilite port=out_status bundle=ctrl
#pragma HLS INTERFACE s_axilite port=out_iters bundle=ctrl
#pragma HLS ALLOCATION operation instances=mul limit=2

    /* Beat 0: [e_y | e_psi | vx | vy] */
    stream_word_t d0 = input_stream.read().data;
    fixed_point_t ey   = (fixed_point_t)(int)d0.range(31, 0);
    fixed_point_t epsi = fp_normalize_angle((fixed_point_t)(int)d0.range(63, 32));
    fixed_point_t vx   = (fixed_point_t)(int)d0.range(95, 64);
    fixed_point_t vy   = (fixed_point_t)(int)d0.range(127, 96);

    /* Beat 1: [omega | steering | horizon_length | reserved] */
    stream_word_t d1 = input_stream.read().data;
    fixed_point_t omega    = (fixed_point_t)(int)d1.range(31, 0);
    fixed_point_t steering = (fixed_point_t)(int)d1.range(63, 32);
    int horizon_len        = (int)d1.range(95, 64);

    if (horizon_len <= 0) {
        *out_steering = 0; *out_accel = 0;
        *out_status = 3; *out_iters = 0;
        return;
    }

    /* Beats 2..N+1: Reference trajectory */
    MpcRefPoint_t ref[MPC_HORIZON];
#pragma HLS ARRAY_PARTITION variable=ref complete dim=0
    for (int k = 0; k < MPC_HORIZON; k++) {
#pragma HLS PIPELINE II=1
        stream_word_t dr = input_stream.read().data;
        ref[k].velocity = (fixed_point_t)(int)dr.range(31, 0);
        fixed_point_t kappa = (fixed_point_t)(int)dr.range(63, 32);
        ref[k].kappa = fp_clamp(kappa, FP_CONST(-1.5), FP_CONST(1.5));
        ref[k].left_bound  = (fixed_point_t)(int)dr.range(95, 64);
        ref[k].right_bound = (fixed_point_t)(int)dr.range(127, 96);
    }

    mpc_fpga_compute_core(ey, epsi, vx, vy, omega, steering, ref,
                          out_steering, out_accel, out_status, out_iters);
}
#endif

/* Scalar wrapper for testbench (not synthesized) */
#ifndef __SYNTHESIS__
extern "C" void mpc_fpga_top_scalar(
    int ey_fp, int epsi_fp, int vx_fp, int vy_fp, int omega_fp, int steering_fp,
    const int *ref_vx, const int *ref_kappa, const int *ref_left, const int *ref_right,
    int ref_count,
    int *out_steering, int *out_accel, int *out_status, int *out_iters)
{
    if (ref_count <= 0) {
        *out_steering = 0; *out_accel = 0;
        *out_status = 3; *out_iters = 0;
        return;
    }

#ifdef MPC_HLS_BUILD
    hls::stream<axis_word_t> stream;
    axis_word_t beat;

    beat.data = pack_word(ey_fp, epsi_fp, vx_fp, vy_fp);
    stream.write(beat);

    beat.data = pack_word(omega_fp, steering_fp, ref_count, 0);
    stream.write(beat);

    for (int k = 0; k < MPC_HORIZON; k++) {
        fixed_point_t kappa = fp_clamp(ref_kappa[k], FP_CONST(-1.5), FP_CONST(1.5));
        beat.data = pack_word(ref_vx[k], kappa, ref_left[k], ref_right[k]);
        stream.write(beat);
    }
    mpc_fpga_top(stream, out_steering, out_accel, out_status, out_iters);
#else
    MpcRefPoint_t ref[MPC_HORIZON];
    for (int k = 0; k < MPC_HORIZON; k++) {
        ref[k].velocity = ref_vx[k];
        ref[k].kappa = fp_clamp(ref_kappa[k], FP_CONST(-1.5), FP_CONST(1.5));
        ref[k].left_bound = ref_left[k];
        ref[k].right_bound = ref_right[k];
    }
    mpc_fpga_compute_core(ey_fp, epsi_fp, vx_fp, vy_fp, omega_fp, steering_fp,
                          ref, out_steering, out_accel, out_status, out_iters);
#endif
}
#endif
