/**
 * @file mpc_fpga_top.cpp
 * @brief FPGA top-level wrappers for the HLS MPC Riccati-ADMM solver.
 * @details Provides two front-ends to the same compute core:
 *          (1) AXI-stream interface for synthesis-time integration and
 *          (2) scalar testbench wrapper for software-driven validation.
 *          Both front-ends decode reference trajectory inputs, preserve
 *          solver state across calls, and expose steering/acceleration/status
 *          outputs from the shared MPC pipeline.
 * @dependencies fp_math_hls.h, mpc_fpga_types.h, riccati_solver_hls.h,
 *               hls_stream.h, ap_int.h, ap_axi_sdata.h
 *
 * Stream Format (128-bit words):
 *   Beat 0: [e_y | e_psi | vx | vy]
 *   Beat 1: [omega | steering | horizon_length | reserved]
 *   Beat 2..N+1: [ref_vx[i] | ref_kappa[i] | ref_left[i] | ref_right[i]]
 *
 * Total: 2 + MPC_HORIZON beats (MPC_FPGA_DMA_BEATS words)
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
 * @brief Execute one MPC solve call with persistent ADMM and actuator history.
 * @param ey Current lateral error state [Q-format fixed-point].
 * @param epsi Current heading error state [Q-format fixed-point radians].
 * @param vx Current longitudinal velocity [Q-format fixed-point meters/second].
 * @param vy Current lateral velocity [Q-format fixed-point meters/second].
 * @param omega Current yaw rate [Q-format fixed-point radians/second].
 * @param steering Measured steering angle used to update persistent steering state.
 * @param ref Reference trajectory array of MPC_HORIZON points.
 * @param out_steering Output steering command pointer.
 * @param out_accel Output acceleration command pointer.
 * @param out_status Output solver status pointer.
 * @param out_iters Output ADMM iteration count pointer.
 * @return None.
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

    if (!out_steering || !out_accel || !out_status || !out_iters) {
        return;
    }

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
/**
 * @brief Pack four 32-bit fixed-point values into one 128-bit stream beat.
 * @param v0 First lane value.
 * @param v1 Second lane value.
 * @param v2 Third lane value.
 * @param v3 Fourth lane value.
 * @return Packed 128-bit stream word.
 */
static stream_word_t pack_word(fixed_point_t v0, fixed_point_t v1,
                               fixed_point_t v2, fixed_point_t v3) {
    stream_word_t w = 0;
    w.range(31, 0)   = (uint32_t)v0;
    w.range(63, 32)  = (uint32_t)v1;
    w.range(95, 64)  = (uint32_t)v2;
    w.range(127, 96) = (uint32_t)v3;
    return w;
}

/**
 * @brief AXI-stream top-level entry point used for HLS synthesis.
 * @param input_stream AXI-stream carrying current state and reference beats.
 * @param out_steering Output steering command pointer.
 * @param out_accel Output acceleration command pointer.
 * @param out_status Output solver status pointer.
 * @param out_iters Output ADMM iteration count pointer.
 * @return None.
 */
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
#pragma HLS ALLOCATION operation instances=div limit=0
#pragma HLS ALLOCATION operation instances=mul limit=2

#ifndef MPC_HLS_BUILD
    if (!out_steering || !out_accel || !out_status || !out_iters) {
        return;
    }
#endif

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

    if (horizon_len <= 0 || horizon_len > MPC_HORIZON) {
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

/**
 * @brief Scalar testbench wrapper that mirrors the synthesized top-level behavior.
 * @param ey_fp Current lateral error [fixed-point].
 * @param epsi_fp Current heading error [fixed-point radians].
 * @param vx_fp Current longitudinal velocity [fixed-point meters/second].
 * @param vy_fp Current lateral velocity [fixed-point meters/second].
 * @param omega_fp Current yaw rate [fixed-point radians/second].
 * @param steering_fp Measured steering angle [fixed-point radians].
 * @param ref_vx Reference velocity array pointer.
 * @param ref_kappa Reference curvature array pointer.
 * @param ref_left Reference left wall bound array pointer.
 * @param ref_right Reference right wall bound array pointer.
 * @param ref_count Number of valid reference points provided by the caller.
 * @param out_steering Output steering command pointer.
 * @param out_accel Output acceleration command pointer.
 * @param out_status Output solver status pointer.
 * @param out_iters Output ADMM iteration count pointer.
 * @return None.
 */
#ifndef __SYNTHESIS__
extern "C" void mpc_fpga_top_scalar(
    int ey_fp, int epsi_fp, int vx_fp, int vy_fp, int omega_fp, int steering_fp,
    const int *ref_vx, const int *ref_kappa, const int *ref_left, const int *ref_right,
    int ref_count,
    int *out_steering, int *out_accel, int *out_status, int *out_iters)
{
    if (!out_steering || !out_accel || !out_status || !out_iters) {
        return;
    }

    if (!ref_vx || !ref_kappa || !ref_left || !ref_right ||
        ref_count <= 0 || ref_count > MPC_HORIZON) {
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
        int src_k = (k < ref_count) ? k : (ref_count - 1);
        fixed_point_t kappa = fp_clamp(ref_kappa[src_k], FP_CONST(-1.5), FP_CONST(1.5));
        beat.data = pack_word(ref_vx[src_k], kappa, ref_left[src_k], ref_right[src_k]);
        stream.write(beat);
    }
    mpc_fpga_top(stream, out_steering, out_accel, out_status, out_iters);
#else
    MpcRefPoint_t ref[MPC_HORIZON];
    for (int k = 0; k < MPC_HORIZON; k++) {
        int src_k = (k < ref_count) ? k : (ref_count - 1);
        ref[k].velocity = ref_vx[src_k];
        ref[k].kappa = fp_clamp(ref_kappa[src_k], FP_CONST(-1.5), FP_CONST(1.5));
        ref[k].left_bound = ref_left[src_k];
        ref[k].right_bound = ref_right[src_k];
    }
    mpc_fpga_compute_core(ey_fp, fp_normalize_angle(epsi_fp), vx_fp, vy_fp, omega_fp, steering_fp,
                          ref, out_steering, out_accel, out_status, out_iters);
#endif
}
#endif
