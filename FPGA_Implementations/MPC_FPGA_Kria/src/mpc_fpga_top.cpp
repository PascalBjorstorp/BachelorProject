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
#include "../include/mpc_riccati_hls.h"
#ifdef MPC_RUNTIME_TUNE
#include "../include/mpc_runtime_tune.h"
#endif

#ifdef MPC_HLS_BUILD
#include <hls_stream.h>
#include <ap_int.h>
#include <ap_axi_sdata.h>

typedef ap_uint<128> stream_word_t;
typedef hls::axis<stream_word_t, 0, 0, 0> axis_word_t;
#endif

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
    fp_QP_t ey, fp_QP_t epsi,
    fp_QP_t vx, fp_QP_t vy, fp_QP_t omega,
    fp_QP_t steering,
    const MpcRefPoint_t ref[MPC_HORIZON],
    int *out_steering, int *out_accel, int *out_status, int *out_iters)
{
    static AdmmState_t admm_state;
#pragma HLS ARRAY_PARTITION variable=admm_state.z_x complete dim=2
#pragma HLS ARRAY_PARTITION variable=admm_state.y_x complete dim=2
#pragma HLS ARRAY_PARTITION variable=admm_state.z_u complete dim=2
#pragma HLS ARRAY_PARTITION variable=admm_state.y_u complete dim=2
    static MpcPersistState_t persist;
    static int initialized = 0;
#ifdef MPC_RUNTIME_TUNE
#endif
#pragma HLS BIND_STORAGE variable=admm_state type=ram_2p impl=bram
#pragma HLS BIND_STORAGE variable=persist type=ram_1p impl=lutram

#ifdef MPC_RUNTIME_TUNE
    mpc_runtime_update_from_env();
#endif



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

    fp_QP_t steer_out, accel_out;
    int status, iters;
    mpc_compute_hls(ey, epsi, vx, vy, omega, ref, &persist, &admm_state,
                    &steer_out, &accel_out, &status, &iters);

    *out_steering = (int)fp_raw_from_io((fp_io_t)steer_out);
    *out_accel    = (int)fp_raw_from_io((fp_io_t)accel_out);
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
static stream_word_t pack_word(fp_QP_t v0, fp_QP_t v1,
                               fp_QP_t v2, fp_QP_t v3) {
    stream_word_t w = 0;
    w.range(31, 0)   = (uint32_t)((fp_stream_raw_t)fp_raw_from_QP(v0));
    w.range(63, 32)  = (uint32_t)((fp_stream_raw_t)fp_raw_from_QP(v1));
    w.range(95, 64)  = (uint32_t)((fp_stream_raw_t)fp_raw_from_QP(v2));
    w.range(127, 96) = (uint32_t)((fp_stream_raw_t)fp_raw_from_QP(v3));
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

    /* Beat 0: [e_y | e_psi | vx | vy] */
    stream_word_t d0 = input_stream.read().data;
    fp_io_t ey_io   = fp_io_from_raw((fp_stream_raw_t)(int)d0.range(31, 0));
    fp_io_t epsi_io = fp_io_from_raw((fp_stream_raw_t)(int)d0.range(63, 32));
    fp_io_t vx_io   = fp_io_from_raw((fp_stream_raw_t)(int)d0.range(95, 64));
    fp_io_t vy_io   = fp_io_from_raw((fp_stream_raw_t)(int)d0.range(127, 96));
    fp_QP_t ey      = (fp_QP_t)ey_io;
    fp_QP_t epsi    = fp_normalize_angle((fp_QP_t)epsi_io);
    fp_QP_t vx      = (fp_QP_t)vx_io;
    fp_QP_t vy      = (fp_QP_t)vy_io;

    /* Beat 1: [omega | steering | horizon_length | reserved] */
    stream_word_t d1 = input_stream.read().data;
    fp_io_t omega_io    = fp_io_from_raw((fp_stream_raw_t)(int)d1.range(31, 0));
    fp_io_t steering_io = fp_io_from_raw((fp_stream_raw_t)(int)d1.range(63, 32));
    fp_QP_t omega       = (fp_QP_t)omega_io;
    fp_QP_t steering    = (fp_QP_t)steering_io;
    int horizon_len        = (int)d1.range(95, 64);
    (void)horizon_len; /* Reserved protocol field (stream always carries MPC_HORIZON points). */

    /* Beats 2..N+1: Reference trajectory */
    MpcRefPoint_t ref[MPC_HORIZON];
#pragma HLS ARRAY_PARTITION variable=ref complete dim=0
    for (int k = 0; k < MPC_HORIZON; k++) {
#pragma HLS PIPELINE II=1
        stream_word_t dr = input_stream.read().data;
        fp_io_t ref_v_io = fp_io_from_raw((fp_stream_raw_t)(int)dr.range(31, 0));
        fp_io_t ref_k_io = fp_io_from_raw((fp_stream_raw_t)(int)dr.range(63, 32));
        ref[k].velocity = (fp_QP_t)ref_v_io;
        ref[k].kappa = (fp_QP_t)ref_k_io;
        ref[k].yaw_rate = fp_mul(ref[k].velocity, ref[k].kappa);
        ref[k].left_bound  = (fp_QP_t)fp_io_from_raw((fp_stream_raw_t)(int)dr.range(95, 64));
        ref[k].right_bound = (fp_QP_t)fp_io_from_raw((fp_stream_raw_t)(int)dr.range(127, 96));
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

    beat.data = pack_word(fp_QP_from_raw((fp_stream_raw_t)ey_fp),
                          fp_QP_from_raw((fp_stream_raw_t)epsi_fp),
                          fp_QP_from_raw((fp_stream_raw_t)vx_fp),
                          fp_QP_from_raw((fp_stream_raw_t)vy_fp));
    stream.write(beat);

    beat.data = pack_word(fp_QP_from_raw((fp_stream_raw_t)omega_fp),
                          fp_QP_from_raw((fp_stream_raw_t)steering_fp),
                          fp_QP_from_raw((fp_stream_raw_t)ref_count),
                          0);
    stream.write(beat);

    for (int k = 0; k < MPC_HORIZON; k++) {
        int src_k = (k < ref_count) ? k : (ref_count - 1);
        fp_QP_t kappa = fp_QP_from_raw((fp_stream_raw_t)ref_kappa[src_k]);
        beat.data = pack_word(fp_QP_from_raw((fp_stream_raw_t)ref_vx[src_k]),
                              kappa,
                              fp_QP_from_raw((fp_stream_raw_t)ref_left[src_k]),
                              fp_QP_from_raw((fp_stream_raw_t)ref_right[src_k]));
        stream.write(beat);
    }
    mpc_fpga_top(stream, out_steering, out_accel, out_status, out_iters);
#else
    MpcRefPoint_t ref[MPC_HORIZON];
    for (int k = 0; k < MPC_HORIZON; k++) {
        int src_k = (k < ref_count) ? k : (ref_count - 1);
        ref[k].velocity = fp_QP_from_raw((fp_stream_raw_t)ref_vx[src_k]);
        ref[k].kappa = fp_QP_from_raw((fp_stream_raw_t)ref_kappa[src_k]);
        ref[k].yaw_rate = fp_mul(ref[k].velocity, ref[k].kappa);
        ref[k].left_bound = fp_QP_from_raw((fp_stream_raw_t)ref_left[src_k]);
        ref[k].right_bound = fp_QP_from_raw((fp_stream_raw_t)ref_right[src_k]);
    }
    mpc_fpga_compute_core(fp_QP_from_raw((fp_stream_raw_t)ey_fp),
                          fp_normalize_angle(fp_QP_from_raw((fp_stream_raw_t)epsi_fp)),
                          fp_QP_from_raw((fp_stream_raw_t)vx_fp),
                          fp_QP_from_raw((fp_stream_raw_t)vy_fp),
                          fp_QP_from_raw((fp_stream_raw_t)omega_fp),
                          fp_QP_from_raw((fp_stream_raw_t)steering_fp),
                          ref, out_steering, out_accel, out_status, out_iters);
#endif
}
#endif
