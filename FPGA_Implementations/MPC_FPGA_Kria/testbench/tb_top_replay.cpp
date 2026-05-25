/**
 * @file tb_top_replay.cpp
 * @brief Diagnostic replay harness for `mpc_fpga_top_opencl`.
 *
 * This testbench is intentionally heavier than a normal csim/cosim harness.
 * It exists to answer three separate questions with the same replay row:
 *   1) Did the packed 512-bit transport reach the kernel intact?
 *   2) Does RTL diverge from the C-model when both start from the same state?
 *   3) If the solver diverges, what did the persistent MPC/ADMM state look like?
 *
 * Opt-in debug controls:
 *   CLI:
 *     --debug-transport
 *     --debug-state
 *     --debug-trace
 *     --debug-dir <dir>
 *
 *   Environment:
 *   REPLAY_DEBUG_TRANSPORT=1  Sweep the entire input payload in 4-word windows
 *                             through the kernel's echo mode.
 *   REPLAY_DEBUG_STATE=1      Dump pre/post state snapshots for both paths.
 *   REPLAY_DEBUG_TRACE=1      Dump scalar ADMM iteration traces.
 *   REPLAY_DEBUG_DIR=<dir>    Output directory for the CSV artifacts.
 */

#include <ap_int.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <sys/stat.h>

#include "fp_types_hls.hpp"
#include "mpc_fpga_constants.h"
#include "mpc_fpga_interface.h"
#include "mpc_fpga_types.h"
#include "riccati_solver_hls.h"

extern "C" void mpc_fpga_top_opencl(const ap_uint<512> *input_words512,
                                    ap_uint<128> *output_words128);
extern "C" void mpc_fpga_top_opencl_debug(const ap_uint<512> *input_words512,
                                          ap_uint<128> *output_words128,
                                          ap_uint<128> *debug_words128);
extern "C" void mpc_fpga_top_scalar_with_prev_accel_and_ref_ey(
    int32_t ey_fp, int32_t epsi_fp, int32_t vx_fp, int32_t vy_fp,
    int32_t omega_fp, int32_t steering_fp, int32_t prev_accel_fp,
    const int32_t *ref_ey, const int32_t *ref_epsi, const int32_t *ref_vx,
    const int32_t *ref_vy, const int32_t *ref_omega_ref,
    const int32_t *ref_kappa, const int32_t *ref_left, const int32_t *ref_right,
    int ref_count, int32_t *out_steering, int32_t *out_accel,
    int32_t *out_status, int32_t *out_iters);
extern "C" void mpc_fpga_top_scalar_controlled_with_prev_accel_and_ref_ey(
    int32_t ey_fp, int32_t epsi_fp, int32_t vx_fp, int32_t vy_fp,
    int32_t omega_fp, int32_t steering_fp, int32_t prev_accel_fp,
    uint32_t control_flags,
    const int32_t *ref_ey, const int32_t *ref_epsi, const int32_t *ref_vx,
    const int32_t *ref_vy, const int32_t *ref_omega_ref,
    const int32_t *ref_kappa, const int32_t *ref_left, const int32_t *ref_right,
    int ref_count, int32_t *out_steering, int32_t *out_accel,
    int32_t *out_status, int32_t *out_iters);
extern "C" void mpc_fpga_reset_persistent_state(void);
extern "C" int mpc_fpga_debug_get_persistent_state(
    MpcCorePersistentState_t *out);
extern "C" int mpc_fpga_debug_set_persistent_state(
    const MpcCorePersistentState_t *in);

namespace {

constexpr int kCols = 234;
constexpr int kHorizon = MPC_FPGA_HORIZON_STEPS;
constexpr int kHeaderWords = MPC_FPGA_HEADER_WORDS_32;
constexpr int kRefWords = MPC_FPGA_REF_WORDS_PER_STEP;

constexpr int kRefEyCol = 12;
constexpr int kRefEpsiCol = kRefEyCol + kHorizon;
constexpr int kRefVxCol = 112;
constexpr int kRefVyCol = kRefVxCol + kHorizon;
constexpr int kRefOmegaCol = kRefVyCol + kHorizon;
constexpr int kRefKappaCol = kRefOmegaCol + kHorizon;
constexpr int kRefLeftCol = kRefKappaCol + kHorizon;
constexpr int kRefRightCol = kRefLeftCol + kHorizon;
constexpr int kEyCol = 232;
constexpr int kEpsiCol = 233;

struct DebugOutputs {
  bool transport = false;
  bool state = false;
  bool trace = false;
  std::string dir;
  std::FILE *transport_csv = nullptr;
  std::FILE *state_csv = nullptr;
  std::FILE *trace_csv = nullptr;
  std::FILE *rtl_header_csv = nullptr;
  std::FILE *rtl_input_csv = nullptr;
  std::FILE *rtl_state_csv = nullptr;
  std::FILE *rtl_admm_csv = nullptr;
  std::FILE *rtl_problem_csv = nullptr;
  std::FILE *rtl_step_data_csv = nullptr;
  std::FILE *rtl_model_csv = nullptr;
  std::FILE *rtl_bootstrap_csv = nullptr;
  std::FILE *rtl_trace_csv = nullptr;
  std::FILE *rtl_summary_csv = nullptr;
};

int split_row(char *line, long long *f) {
  int n = 0;
  char *save = nullptr;
  for (char *tok = strtok_r(line, ",\r\n", &save); tok && n < kCols;
       tok = strtok_r(nullptr, ",\r\n", &save)) {
    f[n++] = strtoll(tok, nullptr, 10);
  }
  return n;
}

bool env_flag_enabled(const char *name) {
  const char *value = std::getenv(name);
  if (!value || !*value) {
    return false;
  }
  return std::strcmp(value, "0") != 0 && std::strcmp(value, "false") != 0 &&
         std::strcmp(value, "FALSE") != 0;
}

std::string env_string_or_default(const char *name, const char *fallback) {
  const char *value = std::getenv(name);
  if (!value || !*value) {
    return std::string(fallback);
  }
  return std::string(value);
}

int32_t qp_raw_word(fp_QP_t value) {
  return (int32_t)fp_raw_from_QP(value);
}

void pack_lanes_to_words512(const int32_t lanes[INPUT_BUFFER_WORDS_32_PAD],
                            ap_uint<512> out[INPUT_BUFFER_WORDS_512]) {
  for (int p = 0; p < INPUT_BUFFER_WORDS_512; ++p) {
    for (int l = 0; l < 16; ++l) {
      out[p].range(32 * l + 31, 32 * l) =
          (ap_uint<32>)(uint32_t)lanes[p * 16 + l];
    }
  }
}

void unpack_words128(const ap_uint<128> &packed, int32_t out[4]) {
  out[0] = (int32_t)(uint32_t)packed.range(31, 0);
  out[1] = (int32_t)(uint32_t)packed.range(63, 32);
  out[2] = (int32_t)(uint32_t)packed.range(95, 64);
  out[3] = (int32_t)(uint32_t)packed.range(127, 96);
}

void init_debug_outputs(DebugOutputs &dbg) {
  dbg.transport = env_flag_enabled("REPLAY_DEBUG_TRANSPORT");
  dbg.state = env_flag_enabled("REPLAY_DEBUG_STATE");
  dbg.trace = env_flag_enabled("REPLAY_DEBUG_TRACE");
  if (!dbg.transport && !dbg.state && !dbg.trace) {
    return;
  }

  dbg.dir = env_string_or_default("REPLAY_DEBUG_DIR",
                                  "tools/output/mpc_replay_debug");
  (void)mkdir(dbg.dir.c_str(), 0755);

  if (dbg.transport) {
    char path[512];
    std::snprintf(path, sizeof(path), "%s/transport_windows.csv",
                  dbg.dir.c_str());
    dbg.transport_csv = std::fopen(path, "w");
    if (dbg.transport_csv) {
      std::fprintf(dbg.transport_csv,
                   "idx,base_word,word0_expected,word0_got,word1_expected,"
                   "word1_got,word2_expected,word2_got,word3_expected,"
                   "word3_got,ok\n");
      std::fflush(dbg.transport_csv);
    }
  }

  if (dbg.state || dbg.trace) {
    char path[512];
    std::snprintf(path, sizeof(path), "%s/state_snapshots.csv",
                  dbg.dir.c_str());
    dbg.state_csv = std::fopen(path, "w");
    if (dbg.state_csv) {
      std::fprintf(
          dbg.state_csv,
          "idx,path,phase,initialized,admm_initialized,rho_fp,rho_u_fp,"
          "prev_steer_rate_fp,prev_accel_fp,prev_delta_cmd_fp,"
          "actual_steering_fp,prev_curvature_fp,prev_ref_velocity_fp,"
          "prev_left_wall_bound_fp,prev_right_wall_bound_fp,"
          "prev_model_signature,last_status,last_iterations,max_iter_streak,"
          "z_x_0_0_fp,z_x_0_1_fp,z_x_last_0_fp,z_x_last_1_fp,"
          "z_u_0_0_fp,z_u_0_1_fp,y_x_0_0_fp,y_x_0_1_fp,"
          "y_x_last_0_fp,y_x_last_1_fp,y_u_0_0_fp,y_u_0_1_fp\n");
      std::fflush(dbg.state_csv);
    }

    std::snprintf(path, sizeof(path), "%s/rtl_header_words.csv", dbg.dir.c_str());
    dbg.rtl_header_csv = std::fopen(path, "w");
    if (dbg.rtl_header_csv) {
      std::fprintf(dbg.rtl_header_csv, "idx,word_idx,label,value\n");
      std::fflush(dbg.rtl_header_csv);
    }

    std::snprintf(path, sizeof(path), "%s/rtl_input_words.csv", dbg.dir.c_str());
    dbg.rtl_input_csv = std::fopen(path, "w");
    if (dbg.rtl_input_csv) {
      std::fprintf(dbg.rtl_input_csv,
                   "idx,word_idx,kind,step,lane,label,value\n");
      std::fflush(dbg.rtl_input_csv);
    }

    std::snprintf(path, sizeof(path), "%s/rtl_state_words.csv", dbg.dir.c_str());
    dbg.rtl_state_csv = std::fopen(path, "w");
    if (dbg.rtl_state_csv) {
      std::fprintf(dbg.rtl_state_csv,
                   "idx,snapshot,word_idx,label,value\n");
      std::fflush(dbg.rtl_state_csv);
    }

    std::snprintf(path, sizeof(path), "%s/rtl_admm_arrays.csv", dbg.dir.c_str());
    dbg.rtl_admm_csv = std::fopen(path, "w");
    if (dbg.rtl_admm_csv) {
      std::fprintf(dbg.rtl_admm_csv,
                   "idx,snapshot,array,step,lane,flat_idx,value\n");
      std::fflush(dbg.rtl_admm_csv);
    }

    std::snprintf(path, sizeof(path), "%s/rtl_problem_setup.csv", dbg.dir.c_str());
    dbg.rtl_problem_csv = std::fopen(path, "w");
    if (dbg.rtl_problem_csv) {
      std::fprintf(dbg.rtl_problem_csv,
                   "idx,section,step,row,col,lane,label,value\n");
      std::fflush(dbg.rtl_problem_csv);
    }

    std::snprintf(path, sizeof(path), "%s/rtl_step_data.csv", dbg.dir.c_str());
    dbg.rtl_step_data_csv = std::fopen(path, "w");
    if (dbg.rtl_step_data_csv) {
      std::fprintf(dbg.rtl_step_data_csv,
                   "idx,step,section,row,col,flat_idx,value\n");
      std::fflush(dbg.rtl_step_data_csv);
    }

    std::snprintf(path, sizeof(path), "%s/rtl_model_terms.csv", dbg.dir.c_str());
    dbg.rtl_model_csv = std::fopen(path, "w");
    if (dbg.rtl_model_csv) {
      std::fprintf(dbg.rtl_model_csv,
                   "idx,step,section,row,col,lane,flat_idx,label,value\n");
      std::fflush(dbg.rtl_model_csv);
    }

    std::snprintf(path, sizeof(path), "%s/rtl_bootstrap_solution.csv",
                  dbg.dir.c_str());
    dbg.rtl_bootstrap_csv = std::fopen(path, "w");
    if (dbg.rtl_bootstrap_csv) {
      std::fprintf(dbg.rtl_bootstrap_csv,
                   "idx,array,step,lane,flat_idx,value\n");
      std::fflush(dbg.rtl_bootstrap_csv);
    }
  }

  if (dbg.trace) {
    char path[512];
    std::snprintf(path, sizeof(path), "%s/scalar_trace.csv", dbg.dir.c_str());
    dbg.trace_csv = std::fopen(path, "w");
    if (dbg.trace_csv) {
      std::fprintf(
          dbg.trace_csv,
          "idx,iter,primal_residual,dual_residual,state_primal_residual,"
          "state_dual_residual,ctrl_primal_residual,ctrl_dual_residual,rho,"
          "rho_u,u0_steer,u0_accel,z0_steer,z0_accel,y0_steer,y0_accel,"
          "scale_rho,scale_rho_u\n");
      std::fflush(dbg.trace_csv);
    }

    std::snprintf(path, sizeof(path), "%s/rtl_trace.csv", dbg.dir.c_str());
    dbg.rtl_trace_csv = std::fopen(path, "w");
    if (dbg.rtl_trace_csv) {
      std::fprintf(
          dbg.rtl_trace_csv,
          "idx,slot,iter,primal_residual_fp,dual_residual_fp,"
          "state_primal_residual_fp,state_dual_residual_fp,"
          "ctrl_primal_residual_fp,ctrl_dual_residual_fp,rho_fp,rho_u_fp,"
          "u0_steer_fp,u0_accel_fp,z0_steer_fp,z0_accel_fp,y0_steer_fp,"
          "y0_accel_fp,scale_rho,scale_rho_u,trace_status,trace_iters\n");
      std::fflush(dbg.rtl_trace_csv);
    }

    std::snprintf(path, sizeof(path), "%s/rtl_summary.csv", dbg.dir.c_str());
    dbg.rtl_summary_csv = std::fopen(path, "w");
    if (dbg.rtl_summary_csv) {
      std::fprintf(
          dbg.rtl_summary_csv,
          "idx,slot,iter,primal_residual_fp,dual_residual_fp,"
          "state_primal_residual_fp,ctrl_primal_residual_fp,rho_fp,rho_u_fp,"
          "u0_steer_fp,u0_accel_fp,z0_steer_fp,z0_accel_fp,y0_steer_fp,"
          "y0_accel_fp,scale_rho,scale_rho_u,summary_status,summary_iters\n");
      std::fflush(dbg.rtl_summary_csv);
    }
  }
}

void close_debug_outputs(DebugOutputs &dbg) {
  if (dbg.transport_csv) {
    std::fclose(dbg.transport_csv);
  }
  if (dbg.state_csv) {
    std::fclose(dbg.state_csv);
  }
  if (dbg.trace_csv) {
    std::fclose(dbg.trace_csv);
  }
  if (dbg.rtl_header_csv) {
    std::fclose(dbg.rtl_header_csv);
  }
  if (dbg.rtl_input_csv) {
    std::fclose(dbg.rtl_input_csv);
  }
  if (dbg.rtl_state_csv) {
    std::fclose(dbg.rtl_state_csv);
  }
  if (dbg.rtl_admm_csv) {
    std::fclose(dbg.rtl_admm_csv);
  }
  if (dbg.rtl_problem_csv) {
    std::fclose(dbg.rtl_problem_csv);
  }
  if (dbg.rtl_step_data_csv) {
    std::fclose(dbg.rtl_step_data_csv);
  }
  if (dbg.rtl_model_csv) {
    std::fclose(dbg.rtl_model_csv);
  }
  if (dbg.rtl_bootstrap_csv) {
    std::fclose(dbg.rtl_bootstrap_csv);
  }
  if (dbg.rtl_trace_csv) {
    std::fclose(dbg.rtl_trace_csv);
  }
  if (dbg.rtl_summary_csv) {
    std::fclose(dbg.rtl_summary_csv);
  }
}

void log_state_snapshot(std::FILE *csv, uint64_t idx, const char *path_name,
                        const char *phase,
                        const MpcCorePersistentState_t &state) {
  if (!csv) {
    return;
  }

  std::fprintf(
      csv,
      "%llu,%s,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
      "%d,%d,%d,%d,%d,%d,%d,%d\n",
      (unsigned long long)idx, path_name, phase, state.initialized,
      state.admm.initialized, qp_raw_word(state.admm.rho),
      qp_raw_word(state.admm.rho_u), qp_raw_word(state.persist.prev_steer_rate),
      qp_raw_word(state.persist.prev_accel),
      qp_raw_word(state.persist.prev_delta_cmd),
      qp_raw_word(state.persist.actual_steering),
      qp_raw_word(state.persist.prev_curvature),
      qp_raw_word(state.persist.prev_ref_velocity),
      qp_raw_word(state.persist.prev_left_wall_bound),
      qp_raw_word(state.persist.prev_right_wall_bound),
      state.persist.prev_model_signature, state.persist.last_status,
      state.persist.last_iterations, state.persist.max_iter_streak,
      qp_raw_word(state.admm.z_x[0][0]), qp_raw_word(state.admm.z_x[0][1]),
      qp_raw_word(state.admm.z_x[MPC_HORIZON][0]),
      qp_raw_word(state.admm.z_x[MPC_HORIZON][1]),
      qp_raw_word(state.admm.z_u[0][0]), qp_raw_word(state.admm.z_u[0][1]),
      qp_raw_word(state.admm.y_x[0][0]), qp_raw_word(state.admm.y_x[0][1]),
      qp_raw_word(state.admm.y_x[MPC_HORIZON][0]),
      qp_raw_word(state.admm.y_x[MPC_HORIZON][1]),
      qp_raw_word(state.admm.y_u[0][0]), qp_raw_word(state.admm.y_u[0][1]));
  std::fflush(csv);
}

void log_scalar_trace(std::FILE *csv, uint64_t idx) {
  if (!csv) {
    return;
  }

  const int trace_count = riccati_hls_debug_get_trace_count();
  for (int i = 0; i < trace_count; ++i) {
    MpcHlsDebugIterSample_t sample{};
    if (riccati_hls_debug_get_trace_sample(i, &sample) != 0) {
      continue;
    }
    std::fprintf(
        csv,
        "%llu,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
        "%.9g,%.9g,%d,%d\n",
        (unsigned long long)idx, sample.iter,
        (double)sample.primal_residual, (double)sample.dual_residual,
        (double)sample.state_primal_residual,
        (double)sample.state_dual_residual,
        (double)sample.ctrl_primal_residual, (double)sample.ctrl_dual_residual,
        (double)sample.rho, (double)sample.rho_u, (double)sample.u0_steer,
        (double)sample.u0_accel, (double)sample.z0_steer,
        (double)sample.z0_accel, (double)sample.y0_steer,
        (double)sample.y0_accel, sample.scale_rho, sample.scale_rho_u);
  }
  std::fflush(csv);
}

bool run_transport_echo_check(DebugOutputs &dbg, uint64_t idx,
                              const int32_t lanes[INPUT_BUFFER_WORDS_32_PAD]) {
  if (!dbg.transport) {
    return true;
  }

  ap_uint<512> in512[INPUT_BUFFER_WORDS_512];
  ap_uint<128> out128[1];
  int32_t dbg_lanes[INPUT_BUFFER_WORDS_32_PAD];
  bool all_ok = true;

  for (int base = 0; base < INPUT_BUFFER_WORDS_32_PAD; base += 4) {
    std::memcpy(dbg_lanes, lanes, sizeof(dbg_lanes));
    dbg_lanes[MPC_FPGA_WORD_CONTROL_FLAGS] =
        (int32_t)(((uint32_t)lanes[MPC_FPGA_WORD_CONTROL_FLAGS]) |
                  MPC_FPGA_CTRL_FLAG_DEBUG_ECHO_INPUTS |
                  MPC_FPGA_CTRL_DEBUG_WORD_BASE(base));

    pack_lanes_to_words512(dbg_lanes, in512);
    out128[0] = 0;
    mpc_fpga_top_opencl(in512, out128);

    int32_t echoed[4];
    unpack_words128(out128[0], echoed);

    const int ok = (echoed[0] == dbg_lanes[base + 0] &&
                    echoed[1] == dbg_lanes[base + 1] &&
                    echoed[2] == dbg_lanes[base + 2] &&
                    echoed[3] == dbg_lanes[base + 3]);
    if (!ok) {
      all_ok = false;
    }

    if (dbg.transport_csv) {
      std::fprintf(dbg.transport_csv,
                   "%llu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                   (unsigned long long)idx, base, dbg_lanes[base + 0],
                   echoed[0], dbg_lanes[base + 1], echoed[1],
                   dbg_lanes[base + 2], echoed[2], dbg_lanes[base + 3],
                   echoed[3], ok);
    }

    if (!ok) {
      std::fprintf(stderr,
                   "idx %llu | TRANSPORT MISMATCH base=%d exp=[%d,%d,%d,%d] "
                   "got=[%d,%d,%d,%d]\n",
                   (unsigned long long)idx, base, dbg_lanes[base + 0],
                   dbg_lanes[base + 1], dbg_lanes[base + 2],
                   dbg_lanes[base + 3], echoed[0], echoed[1], echoed[2],
                   echoed[3]);
    }
  }

  if (dbg.transport_csv) {
    std::fflush(dbg.transport_csv);
  }

  if (all_ok) {
    std::fprintf(stderr,
                 "idx %llu | TRANSPORT echo OK across %d padded words\n",
                 (unsigned long long)idx, INPUT_BUFFER_WORDS_32_PAD);
  }
  return all_ok;
}

bool fetch_state_snapshot(MpcCorePersistentState_t &out) {
  return mpc_fpga_debug_get_persistent_state(&out) == 0;
}

bool restore_state_snapshot(const MpcCorePersistentState_t &in) {
  return mpc_fpga_debug_set_persistent_state(&in) == 0;
}

const char *rtl_state_word_label(int word_idx) {
  switch (word_idx) {
  case 0: return "initialized";
  case 1: return "admm_initialized";
  case 2: return "rho_fp";
  case 3: return "rho_u_fp";
  case 4: return "prev_steer_rate_fp";
  case 5: return "prev_accel_fp";
  case 6: return "prev_delta_cmd_fp";
  case 7: return "actual_steering_fp";
  case 8: return "prev_curvature_fp";
  case 9: return "prev_ref_velocity_fp";
  case 10: return "prev_left_wall_bound_fp";
  case 11: return "prev_right_wall_bound_fp";
  case 12: return "prev_model_signature";
  case 13: return "last_status";
  case 14: return "last_iterations";
  case 15: return "max_iter_streak";
  case 16: return "z_x_0_0_fp";
  case 17: return "z_x_0_1_fp";
  case 18: return "z_x_last_0_fp";
  case 19: return "z_x_last_1_fp";
  case 20: return "z_u_0_0_fp";
  case 21: return "z_u_0_1_fp";
  case 22: return "y_x_0_0_fp";
  case 23: return "y_x_0_1_fp";
  case 24: return "y_x_last_0_fp";
  case 25: return "y_x_last_1_fp";
  case 26: return "y_u_0_0_fp";
  case 27: return "y_u_0_1_fp";
  case 28: return "last_out_steer_fp";
  case 29: return "last_out_accel_fp";
  case 30: return "last_out_status";
  case 31: return "last_out_iters";
  default: return "unknown";
  }
}

const char *rtl_snapshot_name(int snapshot_idx) {
  switch (snapshot_idx) {
  case 0: return "entry";
  case 1: return "solve_entry";
  case 2: return "post";
  default: return "unknown";
  }
}

const char *rtl_header_word_label(int word_idx) {
  switch (word_idx) {
  case MPC_FPGA_DEBUG_HDR_MAGIC: return "magic";
  case MPC_FPGA_DEBUG_HDR_VERSION: return "version";
  case MPC_FPGA_DEBUG_HDR_CONTROL_FLAGS: return "control_flags";
  case MPC_FPGA_DEBUG_HDR_OUTPUT_STATUS: return "output_status";
  case MPC_FPGA_DEBUG_HDR_OUTPUT_ITERS: return "output_iters";
  case MPC_FPGA_DEBUG_HDR_OUTPUT_STEER: return "output_steer_fp";
  case MPC_FPGA_DEBUG_HDR_OUTPUT_ACCEL: return "output_accel_fp";
  case MPC_FPGA_DEBUG_HDR_SOLVER_STATUS_RAW: return "solver_status_raw";
  case MPC_FPGA_DEBUG_HDR_EY_WORD: return "ey_fp";
  case MPC_FPGA_DEBUG_HDR_EPSI_WORD: return "epsi_fp";
  case MPC_FPGA_DEBUG_HDR_EPSI_NORM_WORD: return "epsi_norm_fp";
  case MPC_FPGA_DEBUG_HDR_VX_WORD: return "vx_fp";
  case MPC_FPGA_DEBUG_HDR_VY_WORD: return "vy_fp";
  case MPC_FPGA_DEBUG_HDR_OMEGA_WORD: return "omega_fp";
  case MPC_FPGA_DEBUG_HDR_STEERING_WORD: return "steer_meas_fp";
  case MPC_FPGA_DEBUG_HDR_PREV_ACCEL_WORD: return "prev_accel_fp";
  case MPC_FPGA_DEBUG_HDR_MEASURED_STEER_RATE_WORD:
    return "measured_steer_rate_fp";
  case MPC_FPGA_DEBUG_HDR_TRACE_WORD_COUNT: return "trace_word_count";
  case MPC_FPGA_DEBUG_HDR_SUMMARY_WORD_COUNT: return "summary_word_count";
  case MPC_FPGA_DEBUG_HDR_VALID_INPUT_WORDS: return "valid_input_words";
  case MPC_FPGA_DEBUG_HDR_TOTAL_WORDS: return "total_debug_words";
  case MPC_FPGA_DEBUG_HDR_TRACE_FIELDS: return "trace_fields";
  case MPC_FPGA_DEBUG_HDR_TRACE_MAX: return "trace_max";
  case MPC_FPGA_DEBUG_HDR_CFG_WORDS: return "cfg_words";
  case MPC_FPGA_DEBUG_HDR_X0_WORDS: return "x0_words";
  case MPC_FPGA_DEBUG_HDR_TERM_WORDS: return "term_words";
  case MPC_FPGA_DEBUG_HDR_B_SPARSE_WORDS: return "b_sparse_words";
  case MPC_FPGA_DEBUG_HDR_STEP_WORDS_PER_STAGE: return "step_words_per_stage";
  case MPC_FPGA_DEBUG_HDR_STEP_WORDS: return "step_words";
  case MPC_FPGA_DEBUG_HDR_MODEL_WORDS_PER_STAGE: return "model_words_per_stage";
  case MPC_FPGA_DEBUG_HDR_MODEL_WORDS: return "model_words";
  case MPC_FPGA_DEBUG_HDR_BOOTSTRAP_SOL_X_WORDS: return "bootstrap_sol_x_words";
  case MPC_FPGA_DEBUG_HDR_BOOTSTRAP_SOL_U_WORDS: return "bootstrap_sol_u_words";
  case MPC_FPGA_DEBUG_HDR_SNAPSHOT_WORDS: return "snapshot_words";
  default: return "unknown";
  }
}

const char *input_header_label(int word_idx) {
  switch (word_idx) {
  case MPC_FPGA_WORD_EY: return "ey_fp";
  case MPC_FPGA_WORD_EPSI: return "epsi_fp";
  case MPC_FPGA_WORD_VX: return "vx_fp";
  case MPC_FPGA_WORD_VY: return "vy_fp";
  case MPC_FPGA_WORD_OMEGA: return "omega_fp";
  case MPC_FPGA_WORD_STEERING: return "steer_meas_fp";
  case MPC_FPGA_WORD_CONTROL_FLAGS: return "control_flags";
  case MPC_FPGA_WORD_PREV_ACCEL: return "prev_accel_fp";
  default: return "unknown";
  }
}

const char *ref_word_label(int lane) {
  switch (lane) {
  case 0: return "ref_ey_fp";
  case 1: return "ref_epsi_fp";
  case 2: return "ref_vx_fp";
  case 3: return "ref_vy_fp";
  case 4: return "ref_omega_ref_fp";
  case 5: return "ref_kappa_fp";
  case 6: return "ref_left_fp";
  case 7: return "ref_right_fp";
  default: return "unknown";
  }
}

void unpack_debug_words128(const ap_uint<128> in[MPC_FPGA_DEBUG_TOTAL_BEATS],
                           int32_t out[MPC_FPGA_DEBUG_TOTAL_WORDS]) {
  for (int beat = 0; beat < MPC_FPGA_DEBUG_TOTAL_BEATS; ++beat) {
    const int base = beat * 4;
    if (base + 0 < MPC_FPGA_DEBUG_TOTAL_WORDS) {
      out[base + 0] = (int32_t)(uint32_t)in[beat].range(31, 0);
    }
    if (base + 1 < MPC_FPGA_DEBUG_TOTAL_WORDS) {
      out[base + 1] = (int32_t)(uint32_t)in[beat].range(63, 32);
    }
    if (base + 2 < MPC_FPGA_DEBUG_TOTAL_WORDS) {
      out[base + 2] = (int32_t)(uint32_t)in[beat].range(95, 64);
    }
    if (base + 3 < MPC_FPGA_DEBUG_TOTAL_WORDS) {
      out[base + 3] = (int32_t)(uint32_t)in[beat].range(127, 96);
    }
  }
}

void dump_rtl_header_words(DebugOutputs &dbg, uint64_t idx,
                           const int32_t debug_words[MPC_FPGA_DEBUG_TOTAL_WORDS]) {
  if (!dbg.rtl_header_csv) {
    return;
  }
  for (int word_idx = 0; word_idx < MPC_FPGA_DEBUG_HEADER_WORDS; ++word_idx) {
    std::fprintf(dbg.rtl_header_csv, "%llu,%d,%s,%d\n",
                 (unsigned long long)idx, word_idx,
                 rtl_header_word_label(word_idx), debug_words[word_idx]);
  }
  std::fflush(dbg.rtl_header_csv);
}

void dump_rtl_input_words(DebugOutputs &dbg, uint64_t idx,
                          const int32_t debug_words[MPC_FPGA_DEBUG_TOTAL_WORDS]) {
  if (!dbg.rtl_input_csv) {
    return;
  }
  for (int word_idx = 0; word_idx < MPC_FPGA_DEBUG_INPUT_WORDS; ++word_idx) {
    const int32_t value = debug_words[MPC_FPGA_DEBUG_INPUT_OFFSET + word_idx];
    if (word_idx < MPC_FPGA_HEADER_WORDS_32) {
      std::fprintf(dbg.rtl_input_csv, "%llu,%d,header,-1,%d,%s,%d\n",
                   (unsigned long long)idx, word_idx, word_idx,
                   input_header_label(word_idx), value);
    } else if (word_idx < INPUT_BUFFER_WORDS_32) {
      const int ref_idx = word_idx - MPC_FPGA_HEADER_WORDS_32;
      const int step = ref_idx / MPC_FPGA_REF_WORDS_PER_STEP;
      const int lane = ref_idx % MPC_FPGA_REF_WORDS_PER_STEP;
      std::fprintf(dbg.rtl_input_csv, "%llu,%d,ref,%d,%d,%s,%d\n",
                   (unsigned long long)idx, word_idx, step, lane,
                   ref_word_label(lane), value);
    } else {
      std::fprintf(dbg.rtl_input_csv, "%llu,%d,padding,-1,-1,padding,%d\n",
                   (unsigned long long)idx, word_idx, value);
    }
  }
  std::fflush(dbg.rtl_input_csv);
}

void dump_rtl_snapshot(DebugOutputs &dbg, uint64_t idx, const char *snapshot,
                       const int32_t *snapshot_words) {
  if (dbg.rtl_state_csv) {
    for (int word_idx = 0; word_idx < MPC_FPGA_DEBUG_STATE_WORDS; ++word_idx) {
      std::fprintf(dbg.rtl_state_csv, "%llu,%s,%d,%s,%d\n",
                   (unsigned long long)idx, snapshot, word_idx,
                   rtl_state_word_label(word_idx), snapshot_words[word_idx]);
    }
    std::fflush(dbg.rtl_state_csv);
  }

  if (!dbg.rtl_admm_csv) {
    return;
  }

  const int32_t *zx = snapshot_words + MPC_FPGA_DEBUG_STATE_WORDS;
  const int32_t *zu = zx + MPC_FPGA_DEBUG_ZX_WORDS;
  const int32_t *yx = zu + MPC_FPGA_DEBUG_ZU_WORDS;
  const int32_t *yu = yx + MPC_FPGA_DEBUG_YX_WORDS;

  for (int flat = 0; flat < MPC_FPGA_DEBUG_ZX_WORDS; ++flat) {
    std::fprintf(dbg.rtl_admm_csv, "%llu,%s,z_x,%d,%d,%d,%d\n",
                 (unsigned long long)idx, snapshot, flat / MPC_FPGA_DEBUG_NX_AUG,
                 flat % MPC_FPGA_DEBUG_NX_AUG, flat, zx[flat]);
  }
  for (int flat = 0; flat < MPC_FPGA_DEBUG_ZU_WORDS; ++flat) {
    std::fprintf(dbg.rtl_admm_csv, "%llu,%s,z_u,%d,%d,%d,%d\n",
                 (unsigned long long)idx, snapshot, flat / MPC_FPGA_DEBUG_NU,
                 flat % MPC_FPGA_DEBUG_NU, flat, zu[flat]);
  }
  for (int flat = 0; flat < MPC_FPGA_DEBUG_YX_WORDS; ++flat) {
    std::fprintf(dbg.rtl_admm_csv, "%llu,%s,y_x,%d,%d,%d,%d\n",
                 (unsigned long long)idx, snapshot, flat / MPC_FPGA_DEBUG_NX_AUG,
                 flat % MPC_FPGA_DEBUG_NX_AUG, flat, yx[flat]);
  }
  for (int flat = 0; flat < MPC_FPGA_DEBUG_YU_WORDS; ++flat) {
    std::fprintf(dbg.rtl_admm_csv, "%llu,%s,y_u,%d,%d,%d,%d\n",
                 (unsigned long long)idx, snapshot, flat / MPC_FPGA_DEBUG_NU,
                 flat % MPC_FPGA_DEBUG_NU, flat, yu[flat]);
  }
  std::fflush(dbg.rtl_admm_csv);
}

void dump_rtl_trace(DebugOutputs &dbg, uint64_t idx,
                    const int32_t debug_words[MPC_FPGA_DEBUG_TOTAL_WORDS]) {
  if (!dbg.rtl_trace_csv) {
    return;
  }
  const int trace_word_count = debug_words[MPC_FPGA_DEBUG_HDR_TRACE_WORD_COUNT];
  if (trace_word_count < (int)MPC_FPGA_DEBUG_TRACE_HEADER_WORDS) {
    return;
  }
  const int *trace = &debug_words[MPC_FPGA_DEBUG_TRACE_OFFSET];
  const int sample_count = trace[0];
  const int fields_per_sample = trace[1];
  const int trace_status = trace[2];
  const int trace_iters = trace[3];
  if (sample_count <= 0 || fields_per_sample < 17) {
    return;
  }

  for (int sample_idx = 0; sample_idx < sample_count; ++sample_idx) {
    const int base = 4 + (sample_idx * fields_per_sample);
    if ((base + 16) >= trace_word_count) {
      break;
    }
    std::fprintf(
        dbg.rtl_trace_csv,
        "%llu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
        (unsigned long long)idx, sample_idx, trace[base + 0], trace[base + 1],
        trace[base + 2], trace[base + 3], trace[base + 4], trace[base + 5],
        trace[base + 6], trace[base + 7], trace[base + 8], trace[base + 9],
        trace[base + 10], trace[base + 11], trace[base + 12],
        trace[base + 13], trace[base + 14], trace[base + 15],
        trace[base + 16], trace_status, trace_iters);
  }
  std::fflush(dbg.rtl_trace_csv);
}

void dump_rtl_summary(DebugOutputs &dbg, uint64_t idx,
                      const int32_t debug_words[MPC_FPGA_DEBUG_TOTAL_WORDS]) {
  if (!dbg.rtl_summary_csv) {
    return;
  }
  const int summary_word_count =
      debug_words[MPC_FPGA_DEBUG_HDR_SUMMARY_WORD_COUNT];
  if (summary_word_count < (int)MPC_FPGA_DEBUG_SUMMARY_HEADER_WORDS) {
    return;
  }
  const int *summary = &debug_words[MPC_FPGA_DEBUG_SUMMARY_OFFSET];
  const int slot_count = summary[0];
  const int fields_per_slot = summary[1];
  const int summary_status = summary[2];
  const int summary_iters = summary[3];
  if (slot_count <= 0 || fields_per_slot < 15) {
    return;
  }

  for (int slot = 0; slot < slot_count; ++slot) {
    const int base = 4 + (slot * fields_per_slot);
    if ((base + 14) >= summary_word_count) {
      break;
    }
    std::fprintf(
        dbg.rtl_summary_csv,
        "%llu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
        (unsigned long long)idx, slot, summary[base + 0], summary[base + 1],
        summary[base + 2], summary[base + 3], summary[base + 4],
        summary[base + 5], summary[base + 6], summary[base + 7],
        summary[base + 8], summary[base + 9], summary[base + 10],
        summary[base + 11], summary[base + 12], summary[base + 13],
        summary[base + 14], summary_status, summary_iters);
  }
  std::fflush(dbg.rtl_summary_csv);
}

const char *x0_label(int lane) {
  switch (lane) {
  case 0: return "x0_ey_fp";
  case 1: return "x0_epsi_fp";
  case 2: return "x0_vx_fp";
  case 3: return "x0_vy_fp";
  case 4: return "x0_omega_fp";
  case 5: return "x0_delta_act_fp";
  case 6: return "x0_delta_rate_prev_fp";
  case 7: return "x0_accel_prev_fp";
  default: return "unknown";
  }
}

const char *cfg_label(int idx) {
  switch (idx) {
  case 0: return "cfg_rho_fp";
  case 1: return "cfg_rho_u_fp";
  case 2: return "cfg_tolerance_fp";
  case 3: return "cfg_max_iterations";
  case 4: return "cfg_adaptive_rho";
  default: return "unknown";
  }
}

const char *terminal_label_prefix(int lane) {
  switch (lane) {
  case 0: return "ey";
  case 1: return "epsi";
  case 2: return "vx";
  case 3: return "vy";
  case 4: return "omega";
  case 5: return "delta_act";
  case 6: return "delta_rate_prev";
  case 7: return "accel_prev";
  default: return "unknown";
  }
}

const char *b_sparse_label(int lane) {
  switch (lane) {
  case 0: return "delta_rate";
  case 1: return "vx_accel";
  case 2: return "vy_accel";
  case 3: return "omega_accel";
  default: return "unknown";
  }
}

const char *model_state_label(int lane) {
  switch (lane) {
  case 0: return "lin_ey_fp";
  case 1: return "lin_epsi_fp";
  case 2: return "lin_vx_fp";
  case 3: return "lin_vy_fp";
  case 4: return "lin_omega_fp";
  case 5: return "lin_delta_fp";
  case 6: return "uk0_fp";
  case 7: return "uk1_fp";
  case 8: return "lin_delta_k_fp";
  case 9: return "next_ey_fp";
  case 10: return "next_epsi_fp";
  case 11: return "next_vx_fp";
  case 12: return "next_vy_fp";
  case 13: return "next_omega_fp";
  case 14: return "next_delta_fp";
  default: return "unknown";
  }
}

const char *affine_row_field_label(int field) {
  switch (field) {
  case 0: return "A0_raw";
  case 1: return "A1_raw";
  case 2: return "A2_raw";
  case 3: return "A3_raw";
  case 4: return "A4_raw";
  case 5: return "B0_raw";
  case 6: return "B1_raw";
  case 7: return "a0_term";
  case 8: return "a1_term";
  case 9: return "a2_term";
  case 10: return "a3_term";
  case 11: return "a4_term";
  case 12: return "b0_term";
  case 13: return "b1_term";
  case 14: return "s01";
  case 15: return "s23";
  case 16: return "s45";
  case 17: return "s67";
  case 18: return "s0123";
  case 19: return "s4567";
  case 20: return "acc";
  case 21: return "d_row";
  case 22: return "b0_mul_lo";
  case 23: return "b0_mul_hi";
  case 24: return "b0_term_serial";
  case 25: return "b0_term_alt64";
  case 26: return "b0_term_fixed";
  case 27: return "b1_mul_lo";
  case 28: return "b1_mul_hi";
  case 29: return "b1_term_serial";
  case 30: return "b1_term_alt64";
  case 31: return "b1_term_fixed";
  case 32: return "d_row_serial";
  default: return "unknown";
  }
}

void dump_rtl_problem_setup(
    DebugOutputs &dbg, uint64_t idx,
    const int32_t debug_words[MPC_FPGA_DEBUG_TOTAL_WORDS]) {
  if (!dbg.rtl_problem_csv) {
    return;
  }

  for (int i = 0; i < MPC_FPGA_DEBUG_CFG_WORDS; ++i) {
    std::fprintf(dbg.rtl_problem_csv, "%llu,cfg,-1,-1,-1,%d,%s,%d\n",
                 (unsigned long long)idx, i, cfg_label(i),
                 debug_words[MPC_FPGA_DEBUG_CFG_OFFSET + i]);
  }
  for (int i = 0; i < MPC_FPGA_DEBUG_X0_WORDS; ++i) {
    std::fprintf(dbg.rtl_problem_csv, "%llu,x0,-1,-1,-1,%d,%s,%d\n",
                 (unsigned long long)idx, i, x0_label(i),
                 debug_words[MPC_FPGA_DEBUG_X0_OFFSET + i]);
  }

  const char *sections[] = {"term_q_diag", "term_q_linear", "term_x_lb",
                            "term_x_ub"};
  const int offsets[] = {MPC_FPGA_DEBUG_TERM_Q_DIAG_OFFSET,
                         MPC_FPGA_DEBUG_TERM_Q_LINEAR_OFFSET,
                         MPC_FPGA_DEBUG_TERM_X_LB_OFFSET,
                         MPC_FPGA_DEBUG_TERM_X_UB_OFFSET};
  for (int s = 0; s < 4; ++s) {
    for (int i = 0; i < MPC_FPGA_DEBUG_TERM_WORDS; ++i) {
      std::fprintf(dbg.rtl_problem_csv,
                   "%llu,%s,-1,-1,-1,%d,%s,%d\n",
                   (unsigned long long)idx, sections[s], i,
                   terminal_label_prefix(i), debug_words[offsets[s] + i]);
    }
  }

  for (int flat = 0; flat < MPC_FPGA_DEBUG_B_SPARSE_WORDS; ++flat) {
    const int step = flat / MPC_FPGA_DEBUG_BSP_N;
    const int lane = flat % MPC_FPGA_DEBUG_BSP_N;
    std::fprintf(dbg.rtl_problem_csv, "%llu,b_sparse,%d,-1,-1,%d,%s,%d\n",
                 (unsigned long long)idx, step, lane, b_sparse_label(lane),
                 debug_words[MPC_FPGA_DEBUG_B_SPARSE_OFFSET + flat]);
  }
  std::fflush(dbg.rtl_problem_csv);
}

void dump_rtl_step_data(DebugOutputs &dbg, uint64_t idx,
                        const int32_t debug_words[MPC_FPGA_DEBUG_TOTAL_WORDS]) {
  if (!dbg.rtl_step_data_csv) {
    return;
  }

  for (int step = 0; step < kHorizon; ++step) {
    const int step_base =
        MPC_FPGA_DEBUG_STEP_DATA_OFFSET + step * MPC_FPGA_DEBUG_STEP_WORDS_PER_STAGE;
    int flat = 0;
    for (int row = 0; row < 6; ++row) {
      for (int col = 0; col < 6; ++col, ++flat) {
        std::fprintf(dbg.rtl_step_data_csv, "%llu,%d,A,%d,%d,%d,%d\n",
                     (unsigned long long)idx, step, row, col, flat,
                     debug_words[step_base + flat]);
      }
    }
    for (int row = 0; row < 6; ++row, ++flat) {
      std::fprintf(dbg.rtl_step_data_csv, "%llu,%d,d,%d,-1,%d,%d\n",
                   (unsigned long long)idx, step, row, flat,
                   debug_words[step_base + flat]);
    }
    for (int row = 0; row < 6; ++row, ++flat) {
      std::fprintf(dbg.rtl_step_data_csv, "%llu,%d,q,%d,-1,%d,%d\n",
                   (unsigned long long)idx, step, row, flat,
                   debug_words[step_base + flat]);
    }
    std::fprintf(dbg.rtl_step_data_csv, "%llu,%d,ey_lb,-1,-1,%d,%d\n",
                 (unsigned long long)idx, step, flat,
                 debug_words[step_base + flat]);
    ++flat;
    std::fprintf(dbg.rtl_step_data_csv, "%llu,%d,ey_ub,-1,-1,%d,%d\n",
                 (unsigned long long)idx, step, flat,
                 debug_words[step_base + flat]);
    ++flat;
    std::fprintf(dbg.rtl_step_data_csv, "%llu,%d,accel_ub,-1,-1,%d,%d\n",
                 (unsigned long long)idx, step, flat,
                 debug_words[step_base + flat]);
  }
  std::fflush(dbg.rtl_step_data_csv);
}

void dump_rtl_model_terms(DebugOutputs &dbg, uint64_t idx,
                          const int32_t debug_words[MPC_FPGA_DEBUG_TOTAL_WORDS]) {
  if (!dbg.rtl_model_csv) {
    return;
  }

  for (int step = 0; step < kHorizon; ++step) {
    const int base =
        MPC_FPGA_DEBUG_MODEL_OFFSET + step * MPC_FPGA_DEBUG_MODEL_WORDS_PER_STAGE;
    int off = 0;
    for (int lane = 0; lane < 15; ++lane, ++off) {
      std::fprintf(dbg.rtl_model_csv,
                   "%llu,%d,state,-1,-1,%d,%d,%s,%d\n",
                   (unsigned long long)idx, step, lane, off,
                   model_state_label(lane), debug_words[base + off]);
    }
    for (int row = 0; row < 5; ++row) {
      for (int col = 0; col < 5; ++col, ++off) {
        std::fprintf(dbg.rtl_model_csv,
                     "%llu,%d,A_step,%d,%d,-1,%d,A_step,%d\n",
                     (unsigned long long)idx, step, row, col, off,
                     debug_words[base + off]);
      }
    }
    for (int row = 0; row < 5; ++row) {
      for (int col = 0; col < 2; ++col, ++off) {
        std::fprintf(dbg.rtl_model_csv,
                     "%llu,%d,B_step,%d,%d,-1,%d,B_step,%d\n",
                     (unsigned long long)idx, step, row, col, off,
                     debug_words[base + off]);
      }
    }
    for (int lane = 0; lane < 6; ++lane, ++off) {
      std::fprintf(dbg.rtl_model_csv,
                   "%llu,%d,xk_raw,-1,-1,%d,%d,xk_raw,%d\n",
                   (unsigned long long)idx, step, lane, off,
                   debug_words[base + off]);
    }
    for (int lane = 0; lane < 6; ++lane, ++off) {
      std::fprintf(dbg.rtl_model_csv,
                   "%llu,%d,xk1_raw,-1,-1,%d,%d,xk1_raw,%d\n",
                   (unsigned long long)idx, step, lane, off,
                   debug_words[base + off]);
    }
    for (int row = 0; row < 5; ++row) {
      for (int field = 0; field < 33; ++field, ++off) {
        std::fprintf(dbg.rtl_model_csv,
                     "%llu,%d,affine_row,%d,-1,%d,%d,%s,%d\n",
                     (unsigned long long)idx, step, row, field, off,
                     affine_row_field_label(field), debug_words[base + off]);
      }
    }
    std::fprintf(dbg.rtl_model_csv,
                 "%llu,%d,delta_row,-1,-1,0,%d,dt_uk0_term,%d\n",
                 (unsigned long long)idx, step, off, debug_words[base + off]);
    ++off;
    std::fprintf(dbg.rtl_model_csv,
                 "%llu,%d,delta_row,-1,-1,1,%d,acc_delta,%d\n",
                 (unsigned long long)idx, step, off, debug_words[base + off]);
    ++off;
    std::fprintf(dbg.rtl_model_csv,
                 "%llu,%d,delta_row,-1,-1,2,%d,d_delta,%d\n",
                 (unsigned long long)idx, step, off, debug_words[base + off]);
  }
  std::fflush(dbg.rtl_model_csv);
}

void dump_rtl_bootstrap_solution(
    DebugOutputs &dbg, uint64_t idx,
    const int32_t debug_words[MPC_FPGA_DEBUG_TOTAL_WORDS]) {
  if (!dbg.rtl_bootstrap_csv) {
    return;
  }

  for (int flat = 0; flat < MPC_FPGA_DEBUG_BOOTSTRAP_SOL_X_WORDS; ++flat) {
    std::fprintf(dbg.rtl_bootstrap_csv, "%llu,sol_x,%d,%d,%d,%d\n",
                 (unsigned long long)idx, flat / MPC_FPGA_DEBUG_NX_AUG,
                 flat % MPC_FPGA_DEBUG_NX_AUG, flat,
                 debug_words[MPC_FPGA_DEBUG_BOOTSTRAP_SOL_X_OFFSET + flat]);
  }
  for (int flat = 0; flat < MPC_FPGA_DEBUG_BOOTSTRAP_SOL_U_WORDS; ++flat) {
    std::fprintf(dbg.rtl_bootstrap_csv, "%llu,sol_u,%d,%d,%d,%d\n",
                 (unsigned long long)idx, flat / MPC_FPGA_DEBUG_NU,
                 flat % MPC_FPGA_DEBUG_NU, flat,
                 debug_words[MPC_FPGA_DEBUG_BOOTSTRAP_SOL_U_OFFSET + flat]);
  }
  std::fflush(dbg.rtl_bootstrap_csv);
}

void dump_rtl_debug_buffer(DebugOutputs &dbg, uint64_t idx,
                           const int32_t debug_words[MPC_FPGA_DEBUG_TOTAL_WORDS]) {
  dump_rtl_header_words(dbg, idx, debug_words);
  dump_rtl_input_words(dbg, idx, debug_words);
  dump_rtl_snapshot(dbg, idx, "entry",
                    &debug_words[MPC_FPGA_DEBUG_ENTRY_OFFSET]);
  dump_rtl_snapshot(dbg, idx, "solve_entry",
                    &debug_words[MPC_FPGA_DEBUG_SOLVE_ENTRY_OFFSET]);
  dump_rtl_snapshot(dbg, idx, "post",
                    &debug_words[MPC_FPGA_DEBUG_POST_OFFSET]);
  dump_rtl_trace(dbg, idx, debug_words);
  dump_rtl_summary(dbg, idx, debug_words);
  dump_rtl_problem_setup(dbg, idx, debug_words);
  dump_rtl_step_data(dbg, idx, debug_words);
  dump_rtl_model_terms(dbg, idx, debug_words);
  dump_rtl_bootstrap_solution(dbg, idx, debug_words);
}

}  // namespace

int main(int argc, char **argv) {
  const char *csv = (argc > 1) ? argv[1]
                               : std::getenv("REPLAY_CSV")
                                     ? std::getenv("REPLAY_CSV")
                                     : "state_replay.csv";
  const long max_rows =
      (argc > 2) ? std::atol(argv[2])
                 : std::getenv("REPLAY_ROWS")
                       ? std::atol(std::getenv("REPLAY_ROWS"))
                       : 1;

  int argi = 3;
  const char *out_path = nullptr;
  if (argc > argi && std::strncmp(argv[argi], "--", 2) != 0) {
    out_path = argv[argi++];
  } else {
    out_path = std::getenv("REPLAY_OUT") ? std::getenv("REPLAY_OUT") : nullptr;
  }

  while (argi < argc) {
    if (std::strcmp(argv[argi], "--debug-transport") == 0) {
      setenv("REPLAY_DEBUG_TRANSPORT", "1", 1);
      ++argi;
    } else if (std::strcmp(argv[argi], "--debug-state") == 0) {
      setenv("REPLAY_DEBUG_STATE", "1", 1);
      ++argi;
    } else if (std::strcmp(argv[argi], "--debug-trace") == 0) {
      setenv("REPLAY_DEBUG_TRACE", "1", 1);
      ++argi;
    } else if (std::strcmp(argv[argi], "--debug-dir") == 0 &&
               (argi + 1) < argc) {
      setenv("REPLAY_DEBUG_DIR", argv[argi + 1], 1);
      argi += 2;
    } else {
      std::fprintf(stderr, "unknown arg: %s\n", argv[argi]);
      return 2;
    }
  }

  std::FILE *in = std::fopen(csv, "r");
  if (!in) {
    std::perror("open input csv");
    return 3;
  }
  std::FILE *out = out_path ? std::fopen(out_path, "w") : stdout;
  if (!out) {
    std::perror("open out csv");
    std::fclose(in);
    return 4;
  }

  DebugOutputs dbg;
  init_debug_outputs(dbg);

  static char line[1 << 17];
  if (!std::fgets(line, sizeof(line), in)) {
    std::fprintf(stderr, "empty csv\n");
    close_debug_outputs(dbg);
    return 6;
  }

  std::fprintf(out,
               "idx,stamp_ns,compute_ok,status,iters,control_flags,out_steer_fp,"
               "out_accel_fp,ey_fp,epsi_fp,vx_fp,vy_fp,omega_fp,steer_meas_fp,"
               "prev_accel_in_fp,ref_vx0_fp,ref_kappa0_fp\n");

  ap_uint<512> in512[INPUT_BUFFER_WORDS_512];
  ap_uint<128> out128[1];
  ap_uint<128> debug128[MPC_FPGA_DEBUG_TOTAL_BEATS];
  int32_t debug_words[MPC_FPGA_DEBUG_TOTAL_WORDS];

  int32_t prev_accel_fp = 0;
  bool first_row = true;
  long emitted = 0;

  MpcCorePersistentState_t scalar_state{};
  mpc_fpga_reset_persistent_state();
  if (!fetch_state_snapshot(scalar_state)) {
    std::fprintf(stderr, "failed to snapshot initial persistent state\n");
    close_debug_outputs(dbg);
    return 7;
  }

  while (emitted < max_rows && std::fgets(line, sizeof(line), in)) {
    long long F[kCols] = {0};
    const int nf = split_row(line, F);
    if (nf < kCols) {
      std::fprintf(stderr, "skip malformed row (%d cols)\n", nf);
      continue;
    }
    if (F[11] < kHorizon) {
      std::fprintf(stderr, "skip idx %lld: horizon=%lld < %d\n", F[0], F[11],
                   kHorizon);
      continue;
    }

    const uint32_t control_flags =
        first_row ? MPC_FPGA_CTRL_FLAG_RESET_STATE : MPC_FPGA_CTRL_FLAGS_NONE;
    const int32_t prev_accel_in = first_row ? 0 : prev_accel_fp;

    int32_t lanes[INPUT_BUFFER_WORDS_32_PAD];
    std::memset(lanes, 0, sizeof(lanes));

    lanes[MPC_FPGA_WORD_EY] = (int32_t)F[kEyCol];
    lanes[MPC_FPGA_WORD_EPSI] = (int32_t)F[kEpsiCol];
    lanes[MPC_FPGA_WORD_VX] = (int32_t)F[7];
    lanes[MPC_FPGA_WORD_VY] = (int32_t)F[8];
    lanes[MPC_FPGA_WORD_OMEGA] = (int32_t)F[9];
    lanes[MPC_FPGA_WORD_STEERING] = (int32_t)F[10];
    lanes[MPC_FPGA_WORD_CONTROL_FLAGS] = (int32_t)control_flags;
    lanes[MPC_FPGA_WORD_PREV_ACCEL] = prev_accel_in;

    for (int i = 0; i < kHorizon; ++i) {
      const int base = kHeaderWords + i * kRefWords;
      lanes[base + 0] = (int32_t)F[kRefEyCol + i];
      lanes[base + 1] = (int32_t)F[kRefEpsiCol + i];
      lanes[base + 2] = (int32_t)F[kRefVxCol + i];
      lanes[base + 3] = (int32_t)F[kRefVyCol + i];
      lanes[base + 4] = (int32_t)F[kRefOmegaCol + i];
      lanes[base + 5] = (int32_t)F[kRefKappaCol + i];
      lanes[base + 6] = (int32_t)F[kRefLeftCol + i];
      lanes[base + 7] = (int32_t)F[kRefRightCol + i];
    }

    run_transport_echo_check(dbg, (uint64_t)F[0], lanes);
    pack_lanes_to_words512(lanes, in512);

    out128[0] = 0;
    std::memset(debug128, 0, sizeof(debug128));
    mpc_fpga_top_opencl_debug(in512, out128, debug128);
    unpack_debug_words128(debug128, debug_words);
    if (dbg.state || dbg.trace) {
      dump_rtl_debug_buffer(dbg, (uint64_t)F[0], debug_words);
    }

    const int32_t out_steer = (int32_t)(uint32_t)out128[0].range(31, 0);
    const int32_t out_accel = (int32_t)(uint32_t)out128[0].range(63, 32);
    const uint32_t status = (uint32_t)out128[0].range(95, 64);
    const uint32_t iters = (uint32_t)out128[0].range(127, 96);
    if (debug_words[MPC_FPGA_DEBUG_HDR_MAGIC] !=
        (int32_t)MPC_FPGA_DEBUG_MAGIC) {
      std::fprintf(stderr,
                   "idx %lld | bad debug magic: got=0x%x expected=0x%x\n",
                   F[0], (unsigned)debug_words[MPC_FPGA_DEBUG_HDR_MAGIC],
                   (unsigned)MPC_FPGA_DEBUG_MAGIC);
    }
    if (debug_words[MPC_FPGA_DEBUG_HDR_OUTPUT_STEER] != out_steer ||
        debug_words[MPC_FPGA_DEBUG_HDR_OUTPUT_ACCEL] != out_accel ||
        debug_words[MPC_FPGA_DEBUG_HDR_OUTPUT_STATUS] != (int32_t)status ||
        debug_words[MPC_FPGA_DEBUG_HDR_OUTPUT_ITERS] != (int32_t)iters) {
      std::fprintf(
          stderr,
          "idx %lld | RESULT/DEBUG HEADER MISMATCH beat=[%d,%d,%u,%u] "
          "header=[%d,%d,%d,%d]\n",
          F[0], out_steer, out_accel, status, iters,
          debug_words[MPC_FPGA_DEBUG_HDR_OUTPUT_STEER],
          debug_words[MPC_FPGA_DEBUG_HDR_OUTPUT_ACCEL],
          debug_words[MPC_FPGA_DEBUG_HDR_OUTPUT_STATUS],
          debug_words[MPC_FPGA_DEBUG_HDR_OUTPUT_ITERS]);
    }

    int32_t r_ey[kHorizon], r_epsi[kHorizon], r_vx[kHorizon], r_vy[kHorizon];
    int32_t r_om[kHorizon], r_ka[kHorizon], r_lf[kHorizon], r_rt[kHorizon];
    for (int i = 0; i < kHorizon; ++i) {
      r_ey[i] = (int32_t)F[kRefEyCol + i];
      r_epsi[i] = (int32_t)F[kRefEpsiCol + i];
      r_vx[i] = (int32_t)F[kRefVxCol + i];
      r_vy[i] = (int32_t)F[kRefVyCol + i];
      r_om[i] = (int32_t)F[kRefOmegaCol + i];
      r_ka[i] = (int32_t)F[kRefKappaCol + i];
      r_lf[i] = (int32_t)F[kRefLeftCol + i];
      r_rt[i] = (int32_t)F[kRefRightCol + i];
    }

    if (!restore_state_snapshot(scalar_state)) {
      std::fprintf(stderr, "idx %lld | failed to restore SCALAR state\n", F[0]);
      break;
    }
    if (dbg.state) {
      log_state_snapshot(dbg.state_csv, (uint64_t)F[0], "scalar",
                         "pre_scalar", scalar_state);
    }

    int32_t s_steer = 0;
    int32_t s_accel = 0;
    int32_t s_status = 0;
    int32_t s_iters = 0;
    mpc_fpga_top_scalar_controlled_with_prev_accel_and_ref_ey(
        (int32_t)F[kEyCol], (int32_t)F[kEpsiCol], (int32_t)F[7],
        (int32_t)F[8], (int32_t)F[9], (int32_t)F[10], prev_accel_in,
        control_flags, r_ey, r_epsi, r_vx, r_vy, r_om, r_ka, r_lf, r_rt,
        kHorizon, &s_steer, &s_accel, &s_status, &s_iters);

    if (!fetch_state_snapshot(scalar_state)) {
      std::fprintf(stderr, "idx %lld | failed to snapshot SCALAR state\n", F[0]);
      break;
    }
    if (dbg.state) {
      log_state_snapshot(dbg.state_csv, (uint64_t)F[0], "scalar",
                         "post_scalar", scalar_state);
    }
    if (dbg.trace) {
      log_scalar_trace(dbg.trace_csv, (uint64_t)F[0]);
    }

    const bool full_match = (out_steer == s_steer && out_accel == s_accel &&
                             (int32_t)status == s_status &&
                             (int32_t)iters == s_iters);
    std::fprintf(stderr,
                 "idx %lld | OPENCL steer=%d accel=%d it=%u st=%u | "
                 "SCALAR_ISO steer=%d accel=%d it=%d st=%d | %s\n",
                 F[0], out_steer, out_accel, iters, status, s_steer, s_accel,
                 s_iters, s_status, full_match ? "MATCH" : "*** DIVERGE ***");

    std::fprintf(out,
                 "%lld,%lld,1,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%lld,%lld\n",
                 F[0], F[3], status, iters, control_flags, out_steer,
                 out_accel, (int32_t)F[kEyCol], (int32_t)F[kEpsiCol],
                 (int32_t)F[7], (int32_t)F[8], (int32_t)F[9],
                 (int32_t)F[10], prev_accel_in, F[kRefVxCol], F[kRefKappaCol]);
    std::fflush(out);

    prev_accel_fp = out_accel;
    first_row = false;
    ++emitted;
  }

  std::fclose(in);
  if (out != stdout) {
    std::fclose(out);
  }
  close_debug_outputs(dbg);
  std::fprintf(stderr, "tb_top_replay: emitted %ld row(s)\n", emitted);
  return 0;
}
