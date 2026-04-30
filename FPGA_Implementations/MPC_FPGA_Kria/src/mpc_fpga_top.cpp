/**
 * @file mpc_fpga_top.cpp
 * @brief FPGA top-level wrappers for the HLS MPC Riccati-ADMM solver.
 * @details Provides two front-ends to the same compute core:
 *          (1) OpenCL RAM-buffer kernel interface for deployment and
 *          (2) scalar testbench wrapper for software-driven validation.
 *          Both front-ends decode reference trajectory inputs, preserve
 *          solver state across calls, and expose steering/acceleration/status
 *          outputs from the shared MPC pipeline.
 * @dependencies fp_math_hls.h, mpc_fpga_types.h, riccati_solver_hls.h
 *
 * Packed RAM-Word Format (logical 32-bit lane groups):
 *   Group 0: [e_y | e_psi | vx | vy]
 *   Group 1: [omega | steering | horizon_length | reserved]
 *   Group 2..N+1: [ref_vx[i] | ref_kappa[i] | ref_left[i] | ref_right[i]]
 *
 * Transport: packed in 512-bit OpenCL memory words.
 * Total payload: MPC_FPGA_DMA_BYTES bytes (3 x 512-bit words at current horizon).
 */

#include "../include/fp_math_hls.h"
#include "../include/mpc_fpga_types.h"
#include "../include/mpc_fpga_interface.h"
#include "../include/riccati_solver_hls.h"
#include "../include/mpc_riccati_hls.h"
#ifdef MPC_RUNTIME_TUNE
#include "../include/mpc_runtime_tune.h"
#endif

#ifdef MPC_HLS_BUILD
#include <ap_int.h>
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
    fp_QP_t steering, fp_QP_t prev_accel,
    const MpcRefPoint_t ref[MPC_HORIZON],
    int *out_steering, int *out_accel, int *out_status, int *out_iters)
{
    static AdmmState_t admm_state;
    static MpcPersistState_t persist;
    static int initialized = 0;

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
    /* Accept previous applied acceleration forwarded from compat layer */
    persist.prev_accel = prev_accel;

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
 * @brief OpenCL-friendly top-level that reads packed input words from RAM.
 * @param input_words512 Input buffer packed as 512-bit words.
 * @param output_words128 Output buffer packed as one 128-bit word:
 *        [steering_fp | accel_fp | status | iterations] in 32-bit lanes.
 * @return None.
 */
extern "C" void mpc_fpga_top_opencl(
    const ap_uint<512> *input_words512,
    ap_uint<128> *output_words128)
{
#pragma HLS INTERFACE m_axi port=input_words512 offset=slave bundle=gmem0 depth=INPUT_BUFFER_WORDS_512 max_widen_bitwidth=512
#pragma HLS INTERFACE m_axi port=output_words128 offset=slave bundle=gmem1 depth=1 max_widen_bitwidth=128
#pragma HLS INTERFACE s_axilite port=input_words512 bundle=control
#pragma HLS INTERFACE s_axilite port=output_words128 bundle=control
#pragma HLS INTERFACE s_axilite port=return bundle=control

    if (!output_words128) {
        return;
    }
    if (!input_words512) {
        ap_uint<128> packed_out = 0;
        packed_out.range(31, 0) = (ap_uint<32>)0;
        packed_out.range(63, 32) = (ap_uint<32>)0;
        packed_out.range(95, 64) = (ap_uint<32>)MPC_FPGA_STATUS_ERROR;
        packed_out.range(127, 96) = (ap_uint<32>)0;
        output_words128[0] = packed_out;
        return;
    }

    constexpr int kPacketWords = INPUT_BUFFER_WORDS_512;
    constexpr int kLaneWordsPerPacket = 16;
    constexpr int kLaneWords = INPUT_BUFFER_WORDS_512 * kLaneWordsPerPacket;

    ap_uint<32> lane_words[kLaneWords];
#pragma HLS ARRAY_PARTITION variable=lane_words complete dim=1

    for (int packet_idx = 0; packet_idx < kPacketWords; ++packet_idx) {
#pragma HLS PIPELINE II=1
        const ap_uint<512> packet = input_words512[packet_idx];
        ap_uint<32> packet_lanes[kLaneWordsPerPacket];
#pragma HLS ARRAY_PARTITION variable=packet_lanes complete dim=1

        packet_lanes[0]  = packet.range(31, 0);
        packet_lanes[1]  = packet.range(63, 32);
        packet_lanes[2]  = packet.range(95, 64);
        packet_lanes[3]  = packet.range(127, 96);
        packet_lanes[4]  = packet.range(159, 128);
        packet_lanes[5]  = packet.range(191, 160);
        packet_lanes[6]  = packet.range(223, 192);
        packet_lanes[7]  = packet.range(255, 224);
        packet_lanes[8]  = packet.range(287, 256);
        packet_lanes[9]  = packet.range(319, 288);
        packet_lanes[10] = packet.range(351, 320);
        packet_lanes[11] = packet.range(383, 352);
        packet_lanes[12] = packet.range(415, 384);
        packet_lanes[13] = packet.range(447, 416);
        packet_lanes[14] = packet.range(479, 448);
        packet_lanes[15] = packet.range(511, 480);

        for (int lane = 0; lane < kLaneWordsPerPacket; ++lane) {
#pragma HLS UNROLL
            const int word_index = (packet_idx * kLaneWordsPerPacket) + lane;
            if (word_index < INPUT_BUFFER_WORDS_32) {
                lane_words[word_index] = packet_lanes[lane];
            } else {
                lane_words[word_index] = 0;
            }
        }
    }

    auto read_word32 = [&](int idx) -> int {
#pragma HLS INLINE
        return (int)lane_words[idx];
    };

    fp_QP_t ey       = fp_QP_from_raw((fp_stream_raw_t)read_word32(0));
    fp_QP_t epsi     = fp_normalize_angle(fp_QP_from_raw((fp_stream_raw_t)read_word32(1)));
    fp_QP_t vx       = fp_QP_from_raw((fp_stream_raw_t)read_word32(2));
    fp_QP_t vy       = fp_QP_from_raw((fp_stream_raw_t)read_word32(3));
    fp_QP_t omega    = fp_QP_from_raw((fp_stream_raw_t)read_word32(4));
    fp_QP_t steering = fp_QP_from_raw((fp_stream_raw_t)read_word32(5));

    int horizon_len = read_word32(6);
    if (horizon_len < 1 || horizon_len > MPC_HORIZON) {
        ap_uint<128> packed_out = 0;
        packed_out.range(31, 0) = (ap_uint<32>)0;
        packed_out.range(63, 32) = (ap_uint<32>)0;
        packed_out.range(95, 64) = (ap_uint<32>)MPC_FPGA_STATUS_NO_TRAJECTORY;
        packed_out.range(127, 96) = (ap_uint<32>)0;
        output_words128[0] = packed_out;
        return;
    }

    MpcRefPoint_t ref[MPC_HORIZON];
    for (int k = 0; k < MPC_HORIZON; ++k) {
#pragma HLS PIPELINE II=1
        const int src_k = (k < horizon_len) ? k : (horizon_len - 1);
        const int base = 8 + (src_k * 4);
        ref[k].velocity = fp_QP_from_raw((fp_stream_raw_t)read_word32(base + 0));
        ref[k].kappa = fp_QP_from_raw((fp_stream_raw_t)read_word32(base + 1));
        ref[k].yaw_rate = fp_mul(ref[k].velocity, ref[k].kappa);
        ref[k].left_bound = fp_QP_from_raw((fp_stream_raw_t)read_word32(base + 2));
        ref[k].right_bound = fp_QP_from_raw((fp_stream_raw_t)read_word32(base + 3));
    }

    int out_steering = 0;
    int out_accel = 0;
    int out_status = 0;
    int out_iters = 0;
    mpc_fpga_compute_core(ey, epsi, vx, vy, omega, steering, FP_QP_CONST(0), ref,
                          &out_steering, &out_accel, &out_status, &out_iters);

    ap_uint<128> packed_out = 0;
    packed_out.range(31, 0) = (ap_uint<32>)((uint32_t)out_steering);
    packed_out.range(63, 32) = (ap_uint<32>)((uint32_t)out_accel);
    packed_out.range(95, 64) = (ap_uint<32>)((uint32_t)out_status);
    packed_out.range(127, 96) = (ap_uint<32>)((uint32_t)out_iters);
    output_words128[0] = packed_out;
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
    int32_t ey_fp, int32_t epsi_fp, int32_t vx_fp, int32_t vy_fp, int32_t omega_fp, int32_t steering_fp,
    const int32_t *ref_vx, const int32_t *ref_kappa, const int32_t *ref_left, const int32_t *ref_right,
    int ref_count,
    int32_t *out_steering, int32_t *out_accel, int32_t *out_status, int32_t *out_iters)
{
/* (no forward-declare) */
    if (!out_steering || !out_accel || !out_status || !out_iters) {
        if (out_steering) {
            *out_steering = 0;
        }
        if (out_accel) {
            *out_accel = 0;
        }
        if (out_status) {
            *out_status = MPC_FPGA_STATUS_ERROR;
        }
        if (out_iters) {
            *out_iters = 0;
        }
        return;
    }

    if (!ref_vx || !ref_kappa || !ref_left || !ref_right ||
        ref_count <= 0 || ref_count > MPC_HORIZON) {
        *out_steering = 0; *out_accel = 0;
        *out_status = MPC_FPGA_STATUS_NO_TRAJECTORY; *out_iters = 0;
        return;
    }

    MpcRefPoint_t ref[MPC_HORIZON];
    for (int k = 0; k < MPC_HORIZON; k++) {
        int src_k = (k < ref_count) ? k : (ref_count - 1);
        ref[k].velocity = fp_QP_from_raw((fp_stream_raw_t)ref_vx[src_k]);
        ref[k].kappa = fp_QP_from_raw((fp_stream_raw_t)ref_kappa[src_k]);
        ref[k].yaw_rate = fp_mul(ref[k].velocity, ref[k].kappa);
        ref[k].left_bound = fp_QP_from_raw((fp_stream_raw_t)ref_left[src_k]);
        ref[k].right_bound = fp_QP_from_raw((fp_stream_raw_t)ref_right[src_k]);
    }
    /* Default scalar wrapper: call compute core with zero previous-accel */
    mpc_fpga_compute_core(fp_QP_from_raw((fp_stream_raw_t)ey_fp),
                          fp_normalize_angle(fp_QP_from_raw((fp_stream_raw_t)epsi_fp)),
                          fp_QP_from_raw((fp_stream_raw_t)vx_fp),
                          fp_QP_from_raw((fp_stream_raw_t)vy_fp),
                          fp_QP_from_raw((fp_stream_raw_t)omega_fp),
                          fp_QP_from_raw((fp_stream_raw_t)steering_fp),
                          FP_QP_CONST(0),
                          ref, out_steering, out_accel, out_status, out_iters);
}
#endif

/* Extended scalar wrapper that accepts previous applied acceleration. */
#ifndef __SYNTHESIS__
extern "C" void mpc_fpga_top_scalar_with_prev_accel(
    int32_t ey_fp, int32_t epsi_fp, int32_t vx_fp, int32_t vy_fp, int32_t omega_fp, int32_t steering_fp, int32_t prev_accel_fp,
    const int32_t *ref_vx, const int32_t *ref_kappa, const int32_t *ref_left, const int32_t *ref_right,
    int ref_count,
    int32_t *out_steering, int32_t *out_accel, int32_t *out_status, int32_t *out_iters)
{
    if (!out_steering || !out_accel || !out_status || !out_iters) {
        if (out_steering) *out_steering = 0;
        if (out_accel) *out_accel = 0;
        if (out_status) *out_status = MPC_FPGA_STATUS_ERROR;
        if (out_iters) *out_iters = 0;
        return;
    }
    if (!ref_vx || !ref_kappa || !ref_left || !ref_right || ref_count <= 0 || ref_count > MPC_HORIZON) {
        *out_steering = 0; *out_accel = 0;
        *out_status = MPC_FPGA_STATUS_NO_TRAJECTORY; *out_iters = 0;
        return;
    }
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
                          fp_QP_from_raw((fp_stream_raw_t)prev_accel_fp),
                          ref, out_steering, out_accel, out_status, out_iters);
}
#endif
