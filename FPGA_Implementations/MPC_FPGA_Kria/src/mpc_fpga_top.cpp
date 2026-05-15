/**
 * @file mpc_fpga_top.cpp
 * @brief FPGA top-level wrappers for the HLS MPC Riccati-ADMM solver.
 * @details Provides:
 *          1) an OpenCL RAM-buffer kernel front-end, and
 *          2) a scalar software/testbench front-end,
 *          both feeding the same persistent MPC compute core.
 *
 * Packed input transport (32-bit logical lane words):
 *   Header words:
 *     0: e_y
 *     1: e_psi
 *     2: v_x
 *     3: v_y
 *     4: omega
 *     5: steering
 *     6: control_flags
 *     7: prev_accel
 *
 *   Per-step reference words (8 words per horizon step):
 *     0: reference_lateral_error
 *     1: reference_heading_error
 *     2: reference_velocity
 *     3: reference_lateral_velocity
 *     4: reference_yaw_rate
 *     5: path_curvature
 *     6: left_wall_bound
 *     7: right_wall_bound
 *
 * Packed output transport (128 bits / 4 x 32-bit words):
 *   0: steering command
 *   1: acceleration command
 *   2: solver status
 *   3: ADMM iterations
 */

#include "../include/fp_math_hls.h"
#include "../include/mpc_fpga_interface.h"
#include "../include/mpc_fpga_types.h"
#include "../include/mpc_riccati_hls.h"
#ifdef MPC_RUNTIME_TUNE
#include "../include/mpc_runtime_tune.h"
#endif

#ifdef MPC_HLS_BUILD
#include <ap_int.h>
#endif

#include <cstdint>

#ifndef MPC_FPGA_CTRL_FLAG_RESET_STATE
#define MPC_FPGA_CTRL_FLAG_RESET_STATE      (1u << 0)
#endif
#ifndef MPC_FPGA_CTRL_FLAG_FORCE_COLD_START
#define MPC_FPGA_CTRL_FLAG_FORCE_COLD_START (1u << 1)
#endif
#ifndef MPC_FPGA_CTRL_FLAG_ZERO_DUALS
#define MPC_FPGA_CTRL_FLAG_ZERO_DUALS       (1u << 2)
#endif

namespace {

constexpr int kInputHeaderWords = MPC_FPGA_HEADER_WORDS;
constexpr int kRefWordsPerStep = MPC_FPGA_REF_WORDS;
constexpr int kPackedWordsPer512 = 16;

constexpr int kHeaderWordEy = MPC_FPGA_WORD_EY;
constexpr int kHeaderWordEpsi = MPC_FPGA_WORD_EPSI;
constexpr int kHeaderWordVx = MPC_FPGA_WORD_VX;
constexpr int kHeaderWordVy = MPC_FPGA_WORD_VY;
constexpr int kHeaderWordOmega = MPC_FPGA_WORD_OMEGA;
constexpr int kHeaderWordSteering = MPC_FPGA_WORD_STEERING;
constexpr int kHeaderWordControlFlags = MPC_FPGA_WORD_CONTROL_FLAGS;
constexpr int kHeaderWordPrevAccel = MPC_FPGA_WORD_PREV_ACCEL;

constexpr int kOutputLaneSteering = 0;
constexpr int kOutputLaneAccel = 1;
constexpr int kOutputLaneStatus = 2;
constexpr int kOutputLaneIters = 3;

static MpcCorePersistentState_t g_core_state;

static void reset_admm_state_top_hls(AdmmState_t *admm_state) {
#pragma HLS INLINE off
  if (!admm_state)
    return;

  for (int k = 0; k <= MPC_HORIZON; ++k) {
#pragma HLS PIPELINE II = 1
    for (int s = 0; s < MPC_NX_AUG; ++s) {
      admm_state->z_x[k][s] = FP_QP_CONST(0.0);
      admm_state->y_x[k][s] = FP_QP_CONST(0.0);
    }
  }

  for (int k = 0; k < MPC_HORIZON; ++k) {
#pragma HLS PIPELINE II = 1
    for (int a = 0; a < MPC_NU; ++a) {
      admm_state->z_u[k][a] = FP_QP_CONST(0.0);
      admm_state->y_u[k][a] = FP_QP_CONST(0.0);
    }
  }

  admm_state->rho = FP_QP_CONST(0.0);
  admm_state->rho_u = FP_QP_CONST(0.0);
  admm_state->initialized = 0;
}

static void zero_admm_duals_top_hls(AdmmState_t *admm_state) {
#pragma HLS INLINE off
  if (!admm_state)
    return;

  for (int k = 0; k <= MPC_HORIZON; ++k) {
#pragma HLS PIPELINE II = 1
    for (int s = 0; s < MPC_NX_AUG; ++s) {
      admm_state->y_x[k][s] = FP_QP_CONST(0.0);
    }
  }

  for (int k = 0; k < MPC_HORIZON; ++k) {
#pragma HLS PIPELINE II = 1
    for (int a = 0; a < MPC_NU; ++a) {
      admm_state->y_u[k][a] = FP_QP_CONST(0.0);
    }
  }

  admm_state->rho = FP_QP_CONST(0.0);
  admm_state->rho_u = FP_QP_CONST(0.0);
  admm_state->initialized = 1;
}

static void reset_core_state_hls(MpcCorePersistentState_t *state) {
#pragma HLS INLINE off
  if (!state)
    return;

  reset_admm_state_top_hls(&state->admm);

  state->persist.prev_steer_rate = FP_QP_CONST(0.0);
  state->persist.prev_accel = FP_QP_CONST(0.0);
  state->persist.prev_delta_cmd = FP_QP_CONST(0.0);
  state->persist.actual_steering = FP_QP_CONST(0.0);

  state->persist.prev_curvature = FP_QP_CONST(0.0);
  state->persist.prev_ref_velocity = FP_QP_CONST(0.0);
  state->persist.prev_left_wall_bound = FP_QP_CONST(0.0);
  state->persist.prev_right_wall_bound = FP_QP_CONST(0.0);

  state->persist.last_primal_residual = FP_QP_CONST(0.0);
  state->persist.last_dual_residual = FP_QP_CONST(0.0);

  state->persist.prev_model_signature = 0;
  state->persist.last_status = (int)MPC_STATUS_ERROR;
  state->persist.last_iterations = 0;
  state->persist.max_iter_streak = 0;

  state->initialized = 0;
}

static fp_QP_t fp_qp_from_word(int32_t word) {
#pragma HLS INLINE
  return fp_QP_from_raw((fp_stream_raw_t)word);
}

static int32_t fp_word_from_qp(fp_QP_t value) {
#pragma HLS INLINE
  return (int32_t)fp_raw_from_QP(value);
}

static void fill_mpc_reference_trajectory_from_arrays(
    const int32_t *ref_ey,
    const int32_t *ref_epsi,
    const int32_t *ref_vx,
    const int32_t *ref_vy,
    const int32_t *ref_omega_ref,
    const int32_t *ref_kappa,
    const int32_t *ref_left,
    const int32_t *ref_right,
    int ref_count,
    MpcRefPoint_t out_ref[MPC_HORIZON]) {
#pragma HLS INLINE

  for (int k = 0; k < MPC_HORIZON; ++k) {
#pragma HLS PIPELINE II = 1
    const int src_k = (k < ref_count) ? k : (ref_count - 1);

    out_ref[k].reference_lateral_error =
        ref_ey ? fp_qp_from_word(ref_ey[src_k]) : FP_QP_CONST(0.0);

    out_ref[k].reference_heading_error =
        ref_epsi ? fp_qp_from_word(ref_epsi[src_k]) : FP_QP_CONST(0.0);

    out_ref[k].reference_velocity = fp_qp_from_word(ref_vx[src_k]);

    out_ref[k].reference_lateral_velocity =
        ref_vy ? fp_qp_from_word(ref_vy[src_k]) : FP_QP_CONST(0.0);

    out_ref[k].path_curvature = fp_qp_from_word(ref_kappa[src_k]);

    out_ref[k].reference_yaw_rate =
        ref_omega_ref
            ? fp_qp_from_word(ref_omega_ref[src_k])
            : fp_mul(out_ref[k].reference_velocity, out_ref[k].path_curvature);

    out_ref[k].left_wall_bound = fp_qp_from_word(ref_left[src_k]);
    out_ref[k].right_wall_bound = fp_qp_from_word(ref_right[src_k]);
  }
}

static void fill_mpc_reference_trajectory_from_lane_words(
    const uint32_t lane_words[INPUT_BUFFER_WORDS_32_PAD],
    MpcRefPoint_t out_ref[MPC_HORIZON]) {
#pragma HLS INLINE

  for (int k = 0; k < MPC_HORIZON; ++k) {
#pragma HLS PIPELINE II = 1
    const int base = kInputHeaderWords + (k * kRefWordsPerStep);

    out_ref[k].reference_lateral_error =
        fp_qp_from_word((int32_t)lane_words[base + 0]);
    out_ref[k].reference_heading_error =
        fp_qp_from_word((int32_t)lane_words[base + 1]);
    out_ref[k].reference_velocity =
        fp_qp_from_word((int32_t)lane_words[base + 2]);
    out_ref[k].reference_lateral_velocity =
        fp_qp_from_word((int32_t)lane_words[base + 3]);
    out_ref[k].reference_yaw_rate =
        fp_qp_from_word((int32_t)lane_words[base + 4]);
    out_ref[k].path_curvature =
        fp_qp_from_word((int32_t)lane_words[base + 5]);
    out_ref[k].left_wall_bound =
        fp_qp_from_word((int32_t)lane_words[base + 6]);
    out_ref[k].right_wall_bound =
        fp_qp_from_word((int32_t)lane_words[base + 7]);
  }
}

#ifdef MPC_HLS_BUILD
static void unpack_input_lane_words(
    const ap_uint<512> *input_words512,
    uint32_t lane_words[INPUT_BUFFER_WORDS_32_PAD]) {
#pragma HLS INLINE off

  for (int packet_idx = 0; packet_idx < INPUT_BUFFER_WORDS_512; ++packet_idx) {
#pragma HLS PIPELINE II = 1
    const ap_uint<512> packet = input_words512[packet_idx];
    const int base_word = packet_idx * kPackedWordsPer512;

    lane_words[base_word + 0] = (uint32_t)packet.range(31, 0);
    lane_words[base_word + 1] = (uint32_t)packet.range(63, 32);
    lane_words[base_word + 2] = (uint32_t)packet.range(95, 64);
    lane_words[base_word + 3] = (uint32_t)packet.range(127, 96);
    lane_words[base_word + 4] = (uint32_t)packet.range(159, 128);
    lane_words[base_word + 5] = (uint32_t)packet.range(191, 160);
    lane_words[base_word + 6] = (uint32_t)packet.range(223, 192);
    lane_words[base_word + 7] = (uint32_t)packet.range(255, 224);
    lane_words[base_word + 8] = (uint32_t)packet.range(287, 256);
    lane_words[base_word + 9] = (uint32_t)packet.range(319, 288);
    lane_words[base_word + 10] = (uint32_t)packet.range(351, 320);
    lane_words[base_word + 11] = (uint32_t)packet.range(383, 352);
    lane_words[base_word + 12] = (uint32_t)packet.range(415, 384);
    lane_words[base_word + 13] = (uint32_t)packet.range(447, 416);
    lane_words[base_word + 14] = (uint32_t)packet.range(479, 448);
    lane_words[base_word + 15] = (uint32_t)packet.range(511, 480);
  }
}

static ap_uint<128> pack_output_words(int32_t steering_word,
                                      int32_t accel_word,
                                      int32_t status_word,
                                      int32_t iter_word) {
#pragma HLS INLINE
  ap_uint<128> packed_out = 0;
  packed_out.range(32 * (kOutputLaneSteering + 1) - 1,
                   32 * kOutputLaneSteering) =
      (ap_uint<32>)((uint32_t)steering_word);
  packed_out.range(32 * (kOutputLaneAccel + 1) - 1,
                   32 * kOutputLaneAccel) =
      (ap_uint<32>)((uint32_t)accel_word);
  packed_out.range(32 * (kOutputLaneStatus + 1) - 1,
                   32 * kOutputLaneStatus) =
      (ap_uint<32>)((uint32_t)status_word);
  packed_out.range(32 * (kOutputLaneIters + 1) - 1,
                   32 * kOutputLaneIters) =
      (ap_uint<32>)((uint32_t)iter_word);
  return packed_out;
}
#endif

static void mpc_fpga_compute_core(fp_QP_t ey,
                                  fp_QP_t epsi,
                                  fp_QP_t vx,
                                  fp_QP_t vy,
                                  fp_QP_t omega,
                                  fp_QP_t steering,
                                  fp_QP_t prev_accel,
                                  uint32_t control_flags,
                                  const MpcRefPoint_t ref[MPC_HORIZON],
                                  int32_t *out_steering,
                                  int32_t *out_accel,
                                  int32_t *out_status,
                                  int32_t *out_iters) {
#ifdef MPC_RUNTIME_TUNE
  mpc_runtime_init_once();
#endif

  if (!out_steering || !out_accel || !out_status || !out_iters)
    return;

  if (control_flags & MPC_FPGA_CTRL_FLAG_RESET_STATE) {
    reset_core_state_hls(&g_core_state);
  }

  if (!g_core_state.initialized) {
    reset_core_state_hls(&g_core_state);

    /* Seed measured actuator state directly on first use/reset.
     * This avoids a fake initial steer-rate spike from (steering - 0) / dt. */
    g_core_state.persist.actual_steering = steering;
    g_core_state.persist.prev_delta_cmd = steering;
    g_core_state.persist.prev_accel = prev_accel;
    g_core_state.persist.prev_steer_rate = FP_QP_CONST(0.0);
    g_core_state.persist.prev_curvature = FP_QP_CONST(0.0);
    g_core_state.persist.prev_model_signature = MPC_MODEL_SIGNATURE;
    g_core_state.initialized = 1;
  } else {
    if (control_flags & MPC_FPGA_CTRL_FLAG_FORCE_COLD_START) {
      reset_admm_state_top_hls(&g_core_state.admm);
    } else if (control_flags & MPC_FPGA_CTRL_FLAG_ZERO_DUALS) {
      zero_admm_duals_top_hls(&g_core_state.admm);
    }
  }

  const fp_QP_t inv_control_dt = FP_QP_CONST(1.0 / MPC_FPGA_CONTROL_DT_S);
  fp_QP_t measured_steer_rate =
      fp_mul((steering - g_core_state.persist.actual_steering), inv_control_dt);
  measured_steer_rate =
      fp_clamp(measured_steer_rate, -VP_MAX_STEER_RATE, VP_MAX_STEER_RATE);

  g_core_state.persist.prev_steer_rate = measured_steer_rate;
  g_core_state.persist.actual_steering = steering;
  g_core_state.persist.prev_accel = prev_accel;

  fp_QP_t steer_out = FP_QP_CONST(0.0);
  fp_QP_t accel_out = FP_QP_CONST(0.0);
  int status = (int)MPC_STATUS_ERROR;
  int iters = 0;

  mpc_compute_hls(ey, epsi, vx, vy, omega, ref, &g_core_state.persist,
                  &g_core_state.admm, &steer_out, &accel_out, &status, &iters);

  *out_steering = fp_word_from_qp(steer_out);
  *out_accel = fp_word_from_qp(accel_out);
  *out_status = (int32_t)status;
  *out_iters = (int32_t)iters;
}

} // namespace

#ifndef __SYNTHESIS__
extern "C" void mpc_fpga_reset_persistent_state(void) {
  reset_core_state_hls(&g_core_state);
}
#endif

#ifdef MPC_HLS_BUILD
extern "C" void mpc_fpga_top_opencl(const ap_uint<512> *input_words512,
                                    ap_uint<128> *output_words128) {
#pragma HLS INTERFACE m_axi port = input_words512 offset = slave bundle = gmem0 depth = INPUT_BUFFER_WORDS_512 max_widen_bitwidth = 512
#pragma HLS INTERFACE m_axi port = output_words128 offset = slave bundle = gmem1 depth = 1 max_widen_bitwidth = 128
#pragma HLS INTERFACE s_axilite port = input_words512 bundle = control
#pragma HLS INTERFACE s_axilite port = output_words128 bundle = control
#pragma HLS INTERFACE s_axilite port = return bundle = control

  if (!output_words128)
    return;

  if (!input_words512) {
    output_words128[0] = pack_output_words(0, 0, MPC_FPGA_STATUS_ERROR, 0);
    return;
  }

  uint32_t lane_words[INPUT_BUFFER_WORDS_32_PAD];
#pragma HLS ARRAY_PARTITION variable = lane_words cyclic factor = 16 dim = 1

  unpack_input_lane_words(input_words512, lane_words);

  const int32_t ey_word = (int32_t)lane_words[kHeaderWordEy];
  const int32_t epsi_word = (int32_t)lane_words[kHeaderWordEpsi];
  const int32_t vx_word = (int32_t)lane_words[kHeaderWordVx];
  const int32_t vy_word = (int32_t)lane_words[kHeaderWordVy];
  const int32_t omega_word = (int32_t)lane_words[kHeaderWordOmega];
  const int32_t steering_word = (int32_t)lane_words[kHeaderWordSteering];
  const uint32_t control_flags =
      (uint32_t)lane_words[kHeaderWordControlFlags];
  const int32_t prev_accel_word = (int32_t)lane_words[kHeaderWordPrevAccel];

  MpcRefPoint_t ref[MPC_HORIZON];
  fill_mpc_reference_trajectory_from_lane_words(lane_words, ref);

  int32_t out_steering = 0;
  int32_t out_accel = 0;
  int32_t out_status = MPC_FPGA_STATUS_ERROR;
  int32_t out_iters = 0;

  mpc_fpga_compute_core(fp_qp_from_word(ey_word),
                        fp_normalize_angle(fp_qp_from_word(epsi_word)),
                        fp_qp_from_word(vx_word), fp_qp_from_word(vy_word),
                        fp_qp_from_word(omega_word),
                        fp_qp_from_word(steering_word),
                        fp_qp_from_word(prev_accel_word), control_flags, ref,
                        &out_steering, &out_accel, &out_status, &out_iters);

  output_words128[0] =
      pack_output_words(out_steering, out_accel, out_status, out_iters);
}
#endif

#ifndef __SYNTHESIS__
extern "C" void mpc_fpga_top_scalar_with_prev_accel_and_ref_ey(
    int32_t ey_fp,
    int32_t epsi_fp,
    int32_t vx_fp,
    int32_t vy_fp,
    int32_t omega_fp,
    int32_t steering_fp,
    int32_t prev_accel_fp,
    const int32_t *ref_ey,
    const int32_t *ref_epsi,
    const int32_t *ref_vx,
    const int32_t *ref_vy,
    const int32_t *ref_omega_ref,
    const int32_t *ref_kappa,
    const int32_t *ref_left,
    const int32_t *ref_right,
    int ref_count,
    int32_t *out_steering,
    int32_t *out_accel,
    int32_t *out_status,
    int32_t *out_iters) {
  if (!out_steering || !out_accel || !out_status || !out_iters) {
    if (out_steering)
      *out_steering = 0;
    if (out_accel)
      *out_accel = 0;
    if (out_status)
      *out_status = MPC_FPGA_STATUS_ERROR;
    if (out_iters)
      *out_iters = 0;
    return;
  }

  if (!ref_vx || !ref_kappa || !ref_left || !ref_right || ref_count <= 0 ||
      ref_count > MPC_HORIZON) {
    *out_steering = 0;
    *out_accel = 0;
    *out_status = MPC_FPGA_STATUS_NO_TRAJECTORY;
    *out_iters = 0;
    return;
  }

  MpcRefPoint_t ref[MPC_HORIZON];
  fill_mpc_reference_trajectory_from_arrays(
      ref_ey, ref_epsi, ref_vx, ref_vy, ref_omega_ref, ref_kappa, ref_left,
      ref_right, ref_count, ref);

  mpc_fpga_compute_core(fp_qp_from_word(ey_fp),
                        fp_normalize_angle(fp_qp_from_word(epsi_fp)),
                        fp_qp_from_word(vx_fp), fp_qp_from_word(vy_fp),
                        fp_qp_from_word(omega_fp),
                        fp_qp_from_word(steering_fp),
                        fp_qp_from_word(prev_accel_fp), 0u, ref, out_steering,
                        out_accel, out_status, out_iters);
}
#endif
