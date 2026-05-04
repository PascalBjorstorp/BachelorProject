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
 *   Group 1: [omega | steering | horizon_length | prev_accel]
 *   Group 2..N+1(+): [ref_ey[i] | ref_vx[i] | ref_kappa[i] | ref_left[i] | ref_right[i]]
 *
 * Transport: packed in 512-bit OpenCL memory words.
 * Total payload: MPC_FPGA_DMA_BYTES logical bytes, padded to INPUT_BUFFER_BYTES_512.
 *
 * ─────────────────────────────────────────────────────────────────────
 * INTERFACE VERSIONING (reserved for future extensions)
 * ─────────────────────────────────────────────────────────────────────
 * Current interface (v1.0):
 *   - reference_heading_error: always zero (reserved, future nonzero tracking)
 *   - reference_lateral_velocity: always zero (reserved, future nonzero tracking)
 *   - control payload: steering [fp], accel [fp], status, iterations
 *
 * Future extensions to support (without payload redesign):
 *   - Nonzero heading_error reference for trajectory-following cost
 *   - Nonzero lateral_velocity reference for sideslip tracking
 *   - Extended control word (reset signal, mode flags, etc.)
 *
 * The transport and kernel support richer per-step reference channels.
 * Upstream publishers currently populate the geometric/path channels used by
 * the deployed path, while optional dynamic/error channels may still be zero
 * or locally derived in some host transport paths.
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

#include <string.h>

static AdmmState_t g_admm_state;
static MpcPersistState_t g_persist;
static int g_initialized = 0;

#ifndef __SYNTHESIS__
extern "C" void mpc_fpga_reset_persistent_state(void)
{
    memset(&g_admm_state, 0, sizeof(g_admm_state));
    memset(&g_persist, 0, sizeof(g_persist));
    g_initialized = 0;
}
#endif

/**
 * @brief Internal helper: decode reference trajectory into MpcRefPoint_t array.
 * @details Fills a reference array by unpacking raw fixed-point trajectory words.
 *          Currently supports zero heading error and lateral velocity references.
 *          This structure is prepared for future extension to nonzero references
 *          when upstream publishers provide richer trajectory information.
 * @param ref_ey Reference lateral error array (NULL → defaults to 0).
 * @param ref_vx Reference velocity array pointer.
 * @param ref_kappa Reference curvature array pointer.
 * @param ref_left Reference left wall bound array pointer.
 * @param ref_right Reference right wall bound array pointer.
 * @param ref_count Number of valid reference points.
 * @param out_ref Output reference trajectory array (length MPC_HORIZON).
 * @return None.
 */
static void fill_mpc_reference_trajectory(
    const int32_t *ref_ey,
    const int32_t *ref_epsi,
    const int32_t *ref_vx,
    const int32_t *ref_vy,
    const int32_t *ref_omega_ref,
    const int32_t *ref_kappa,
    const int32_t *ref_left,
    const int32_t *ref_right,
    int ref_count,
    MpcRefPoint_t out_ref[MPC_HORIZON])
{
#pragma HLS INLINE

    for (int k = 0; k < MPC_HORIZON; k++) {
#pragma HLS PIPELINE II=1
        int src_k = (k < ref_count) ? k : (ref_count - 1);
        /* Lateral error reference (optional in transport) */
        out_ref[k].reference_lateral_error = ref_ey
            ? fp_QP_from_raw((fp_stream_raw_t)ref_ey[src_k])
            : FP_QP_CONST(0.0);

        /* Heading error reference (reserved, may be nonzero in V2) */
        out_ref[k].reference_heading_error = ref_epsi
            ? fp_QP_from_raw((fp_stream_raw_t)ref_epsi[src_k])
            : FP_QP_CONST(0.0);

        /* Longitudinal velocity reference */
        out_ref[k].reference_velocity =
            fp_QP_from_raw((fp_stream_raw_t)ref_vx[src_k]);

        /* Lateral velocity reference (reserved in V2) */
        out_ref[k].reference_lateral_velocity = ref_vy
            ? fp_QP_from_raw((fp_stream_raw_t)ref_vy[src_k])
            : FP_QP_CONST(0.0);

        /* Path curvature and explicit yaw-rate reference */
        out_ref[k].path_curvature =
            fp_QP_from_raw((fp_stream_raw_t)ref_kappa[src_k]);
        out_ref[k].reference_yaw_rate = ref_omega_ref
            ? fp_QP_from_raw((fp_stream_raw_t)ref_omega_ref[src_k])
            : fp_mul(out_ref[k].reference_velocity, out_ref[k].path_curvature);

        /* Wall corridor bounds */
        out_ref[k].left_wall_bound =
            fp_QP_from_raw((fp_stream_raw_t)ref_left[src_k]);
        out_ref[k].right_wall_bound =
            fp_QP_from_raw((fp_stream_raw_t)ref_right[src_k]);
    }
}

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
#ifdef MPC_RUNTIME_TUNE
    mpc_runtime_init_once();
#endif

    if (!g_initialized) {
        g_persist.prev_steer_rate = 0;
        g_persist.prev_accel = 0;
        g_persist.prev_delta_cmd = 0;
        g_persist.actual_steering = 0;
        g_persist.prev_curvature = 0;
        g_persist.prev_converged = 0;
        g_persist.prev_ey = 0;
        g_persist.prev_epsi = 0;
        g_persist.prev_vx = 0;
        g_persist.prev_left_bound0 = 0;
        g_persist.prev_right_bound0 = 0;
        g_persist.prev_model_signature = MPC_MODEL_SIGNATURE;
        g_persist.cold_start_countdown = 0;
        g_admm_state.initialized = 0;
        g_initialized = 1;
    }
    /* Match CPU MPC: x0[delta_rate_prev] is seeded from measured steering-rate,
     * not from the previously optimized delta-rate command. */
    const fp_QP_t inv_control_dt = FP_QP_CONST(1.0 / MPC_FPGA_CONTROL_DT_S);
    fp_QP_t measured_steer_rate =
        fp_mul((steering - g_persist.actual_steering), inv_control_dt);
    measured_steer_rate = fp_clamp(measured_steer_rate,
                                   -VP_MAX_STEER_RATE,
                                   VP_MAX_STEER_RATE);
    g_persist.prev_steer_rate = measured_steer_rate;
    g_persist.actual_steering = steering;
    /* Accept previous applied acceleration forwarded from compat layer */
    g_persist.prev_accel = prev_accel;

    fp_QP_t steer_out, accel_out;
    int status, iters;
    mpc_compute_hls(ey, epsi, vx, vy, omega, ref, &g_persist, &g_admm_state,
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
    constexpr int kLaneWords = INPUT_BUFFER_WORDS_32_PAD;

    uint32_t lane_words[kLaneWords];
#pragma HLS ARRAY_PARTITION variable=lane_words cyclic factor=16 dim=1

    for (int i = 0; i < kLaneWords; ++i) {
#pragma HLS UNROLL
        lane_words[i] = 0u;
    }

    for (int packet_idx = 0; packet_idx < kPacketWords; ++packet_idx) {
#pragma HLS PIPELINE II=1
        const ap_uint<512> packet = input_words512[packet_idx];
        const int base_word = packet_idx * kLaneWordsPerPacket;

        lane_words[base_word + 0]  = (uint32_t)packet.range(31, 0);
        lane_words[base_word + 1]  = (uint32_t)packet.range(63, 32);
        lane_words[base_word + 2]  = (uint32_t)packet.range(95, 64);
        lane_words[base_word + 3]  = (uint32_t)packet.range(127, 96);
        lane_words[base_word + 4]  = (uint32_t)packet.range(159, 128);
        lane_words[base_word + 5]  = (uint32_t)packet.range(191, 160);
        lane_words[base_word + 6]  = (uint32_t)packet.range(223, 192);
        lane_words[base_word + 7]  = (uint32_t)packet.range(255, 224);
        lane_words[base_word + 8]  = (uint32_t)packet.range(287, 256);
        lane_words[base_word + 9]  = (uint32_t)packet.range(319, 288);
        lane_words[base_word + 10] = (uint32_t)packet.range(351, 320);
        lane_words[base_word + 11] = (uint32_t)packet.range(383, 352);
        lane_words[base_word + 12] = (uint32_t)packet.range(415, 384);
        lane_words[base_word + 13] = (uint32_t)packet.range(447, 416);
        lane_words[base_word + 14] = (uint32_t)packet.range(479, 448);
        lane_words[base_word + 15] = (uint32_t)packet.range(511, 480);
    }

    const int32_t w0 = (int32_t)lane_words[0];
    const int32_t w1 = (int32_t)lane_words[1];
    const int32_t w2 = (int32_t)lane_words[2];
    const int32_t w3 = (int32_t)lane_words[3];
    const int32_t w4 = (int32_t)lane_words[4];
    const int32_t w5 = (int32_t)lane_words[5];
    const int32_t w6 = (int32_t)lane_words[6];
    const int32_t w7 = (int32_t)lane_words[7];

    fp_QP_t ey         = fp_QP_from_raw((fp_stream_raw_t)w0);
    fp_QP_t epsi       = fp_normalize_angle(fp_QP_from_raw((fp_stream_raw_t)w1));
    fp_QP_t vx         = fp_QP_from_raw((fp_stream_raw_t)w2);
    fp_QP_t vy         = fp_QP_from_raw((fp_stream_raw_t)w3);
    fp_QP_t omega      = fp_QP_from_raw((fp_stream_raw_t)w4);
    fp_QP_t steering   = fp_QP_from_raw((fp_stream_raw_t)w5);
    fp_QP_t prev_accel = fp_QP_from_raw((fp_stream_raw_t)w7);

    int horizon_len = w6;
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
#if 1
    for (int k = 0; k < MPC_HORIZON; ++k) {
#pragma HLS PIPELINE II=1
        const int src_k = (k < horizon_len) ? k : (horizon_len - 1);
        const int base = 8 + (src_k * 8);
        const int32_t ref_ey_word        = (int32_t)lane_words[base + 0];
        const int32_t ref_epsi_word      = (int32_t)lane_words[base + 1];
        const int32_t ref_vx_word        = (int32_t)lane_words[base + 2];
        const int32_t ref_vy_word        = (int32_t)lane_words[base + 3];
        const int32_t ref_omega_word     = (int32_t)lane_words[base + 4];
        const int32_t ref_kappa_word     = (int32_t)lane_words[base + 5];
        const int32_t ref_left_word      = (int32_t)lane_words[base + 6];
        const int32_t ref_right_word     = (int32_t)lane_words[base + 7];

        ref[k].reference_lateral_error = fp_QP_from_raw((fp_stream_raw_t)ref_ey_word);
        ref[k].reference_heading_error = fp_QP_from_raw((fp_stream_raw_t)ref_epsi_word);
        ref[k].reference_velocity = fp_QP_from_raw((fp_stream_raw_t)ref_vx_word);
        ref[k].reference_lateral_velocity = fp_QP_from_raw((fp_stream_raw_t)ref_vy_word);
        ref[k].path_curvature = fp_QP_from_raw((fp_stream_raw_t)ref_kappa_word);
        ref[k].reference_yaw_rate = fp_QP_from_raw((fp_stream_raw_t)ref_omega_word);
        ref[k].left_wall_bound = fp_QP_from_raw((fp_stream_raw_t)ref_left_word);
        ref[k].right_wall_bound = fp_QP_from_raw((fp_stream_raw_t)ref_right_word);
    }
#endif

    int out_steering = 0;
    int out_accel = 0;
    int out_status = 0;
    int out_iters = 0;
    mpc_fpga_compute_core(ey, epsi, vx, vy, omega, steering, prev_accel, ref,
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
 * @param prev_accel_fp Previous applied acceleration forwarded from compat layer [fixed-point m/s^2].
 * @param ref_ey Reference lateral error array pointer (`NULL` defaults to 0).
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
extern "C" void mpc_fpga_top_scalar_with_prev_accel_and_ref_ey(
    int32_t ey_fp, int32_t epsi_fp, int32_t vx_fp, int32_t vy_fp, int32_t omega_fp, int32_t steering_fp, int32_t prev_accel_fp,
    const int32_t *ref_ey, const int32_t *ref_epsi, const int32_t *ref_vx, const int32_t *ref_vy, const int32_t *ref_omega_ref,
    const int32_t *ref_kappa, const int32_t *ref_left, const int32_t *ref_right,
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
    fill_mpc_reference_trajectory(
        ref_ey,
        ref_epsi,
        ref_vx,
        ref_vy,
        ref_omega_ref,
        ref_kappa,
        ref_left,
        ref_right,
        ref_count,
        ref);

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