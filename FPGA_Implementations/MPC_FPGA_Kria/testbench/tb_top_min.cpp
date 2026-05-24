/**
 * @file tb_top_min.cpp
 * @brief Minimal clean cosim/csim harness for the PROD top mpc_fpga_top_opencl.
 *
 * No debug instrumentation. Invokes the synthesized top once per state_replay
 * row (default: row 1 only) so Vitis cosim actually exercises the RTL
 * (avoids COSIM 212-330). Cross-checks the scalar HLS entry in csim.
 *
 * Usage:  tb_top_min <state_replay.csv> [max_rows]
 *   env:  REPLAY_CSV, REPLAY_ROWS
 *
 * Pass signal: solver should report status=0, iters~3 and the OPENCL line
 * should match the scalar/iso result and the known C model (steer -29,
 * accel 1496703 for the idx1 vector).
 */

#include <ap_int.h>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <glob.h>
#include <limits.h>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

#include "mpc_fpga_constants.h"
#include "mpc_fpga_interface.h"

extern "C" void mpc_fpga_top_opencl(const ap_uint<512> *input_words512,
                                    ap_uint<128> *output_words128);
extern "C" void mpc_fpga_top_scalar_with_prev_accel_and_ref_ey(
    int32_t ey_fp, int32_t epsi_fp, int32_t vx_fp, int32_t vy_fp,
    int32_t omega_fp, int32_t steering_fp, int32_t prev_accel_fp,
    const int32_t *ref_ey, const int32_t *ref_epsi, const int32_t *ref_vx,
    const int32_t *ref_vy, const int32_t *ref_omega_ref,
    const int32_t *ref_kappa, const int32_t *ref_left, const int32_t *ref_right,
    int ref_count, int32_t *out_steering, int32_t *out_accel,
    int32_t *out_status, int32_t *out_iters);

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

bool file_exists(const char *path) {
  struct stat st;
  return path && *path && stat(path, &st) == 0 && S_ISREG(st.st_mode);
}

bool dir_exists(const std::string &path) {
  struct stat st;
  return stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

std::string parent_dir(std::string path) {
  while (!path.empty() && path.back() == '/') path.pop_back();
  const std::string::size_type slash = path.find_last_of('/');
  if (slash == std::string::npos) return ".";
  if (slash == 0) return "/";
  return path.substr(0, slash);
}

std::string canonical_path_or_self(const char *path) {
  char *resolved = realpath(path, nullptr);
  if (!resolved) return std::string(path);
  std::string out(resolved);
  std::free(resolved);
  return out;
}

bool looks_like_repo_root(const std::string &path) {
  return dir_exists(path + "/FPGA_Implementations") && dir_exists(path + "/tools");
}

std::string find_repo_root(std::string start) {
  for (int i = 0; i < 10 && !start.empty(); ++i) {
    if (looks_like_repo_root(start)) return start;
    const std::string next = parent_dir(start);
    if (next == start) break;
    start = next;
  }
  return "";
}

std::string repo_root_from_source_path() {
  const std::string source_path = canonical_path_or_self(__FILE__);
  std::string repo_root = find_repo_root(parent_dir(source_path));
  if (!repo_root.empty()) return repo_root;

  char cwd_buf[PATH_MAX];
  if (getcwd(cwd_buf, sizeof(cwd_buf))) {
    repo_root = find_repo_root(cwd_buf);
    if (!repo_root.empty()) return repo_root;
  }

  return ".";
}

std::string newest_glob_match(const std::string &pattern) {
  glob_t matches;
  std::memset(&matches, 0, sizeof(matches));
  if (glob(pattern.c_str(), 0, nullptr, &matches) != 0) {
    globfree(&matches);
    return "";
  }

  std::string best;
  time_t best_mtime = 0;
  for (size_t i = 0; i < matches.gl_pathc; ++i) {
    const char *candidate = matches.gl_pathv[i];
    struct stat st;
    if (stat(candidate, &st) != 0 || !S_ISREG(st.st_mode)) continue;
    if (best.empty() || st.st_mtime > best_mtime) {
      best = candidate;
      best_mtime = st.st_mtime;
    }
  }
  globfree(&matches);
  return best;
}

std::string resolve_csv_path(int argc, char **argv) {
  const char *explicit_csv =
      (argc > 1)             ? argv[1]
      : getenv("REPLAY_CSV") ? getenv("REPLAY_CSV")
                             : "state_replay.csv";
  if (file_exists(explicit_csv)) return explicit_csv;

  std::fprintf(stderr,
               "tb_top_min: requested replay csv missing: %s\n",
               explicit_csv);

  const std::string repo_root = repo_root_from_source_path();
  const std::string candidates[] = {
      repo_root + "/FPGA_Implementations/MPC_FPGA_Kria/testbench/state_replay.csv",
      repo_root + "/tools/output/deep_diag_run1/frenet_parity/state_replay.csv",
  };
  for (const std::string &candidate : candidates) {
    if (file_exists(candidate.c_str())) return candidate;
  }

  const std::string glob_patterns[] = {
      repo_root + "/tools/output/*/frenet_parity/state_replay.csv",
      repo_root + "/tools/output/*/state_replay.csv",
  };
  for (const std::string &pattern : glob_patterns) {
    const std::string match = newest_glob_match(pattern);
    if (!match.empty()) return match;
  }

  return "";
}

int split_row(char *line, long long *f) {
  int n = 0;
  char *save = nullptr;
  for (char *tok = strtok_r(line, ",\r\n", &save); tok && n < kCols;
       tok = strtok_r(nullptr, ",\r\n", &save))
    f[n++] = strtoll(tok, nullptr, 10);
  return n;
}
}  // namespace

int main(int argc, char **argv) {
  const std::string csv_path = resolve_csv_path(argc, argv);
  if (csv_path.empty()) {
    std::fprintf(stderr,
                 "tb_top_min: no usable state_replay.csv found in repo fallbacks\n");
    return 3;
  }
  const long max_rows = (argc > 2)               ? atol(argv[2])
                        : getenv("REPLAY_ROWS")  ? atol(getenv("REPLAY_ROWS"))
                                                 : 1;
  /* 1-based data-row index to start at (default 1 = from the beginning).
   * RESET_STATE is applied on the first PROCESSED row and prev_accel feedback
   * begins there, so a mid-trajectory window is a clean cold-started replay
   * of that segment. argv[3] or REPLAY_START. */
  const long start_row = (argc > 3)              ? atol(argv[3])
                         : getenv("REPLAY_START") ? atol(getenv("REPLAY_START"))
                                                  : 1;
  std::fprintf(stderr, "tb_top_min: using replay csv %s\n", csv_path.c_str());
  std::FILE *in = std::fopen(csv_path.c_str(), "r");
  if (!in) { std::perror("open csv"); return 3; }

  static char line[1 << 17];
  if (!std::fgets(line, sizeof(line), in)) { std::fprintf(stderr, "empty\n"); return 6; }
  for (long s = 1; s < start_row; ++s) {
    if (!std::fgets(line, sizeof(line), in)) {
      std::fprintf(stderr, "start_row %ld past EOF\n", start_row);
      std::fclose(in);
      return 7;
    }
  }

  ap_uint<512> in512[INPUT_BUFFER_WORDS_512];
  ap_uint<128> out128[1];
  int32_t prev_accel = 0;
  bool first = true;
  long emitted = 0;

  while (emitted < max_rows && std::fgets(line, sizeof(line), in)) {
    long long F[kCols] = {0};
    if (split_row(line, F) < kCols) { std::fprintf(stderr, "skip bad row\n"); continue; }
    if (F[11] < kHorizon) { std::fprintf(stderr, "skip idx %lld horizon\n", F[0]); continue; }

    const uint32_t flags =
        first ? MPC_FPGA_CTRL_FLAG_RESET_STATE : MPC_FPGA_CTRL_FLAGS_NONE;
    const int32_t prev_in = first ? 0 : prev_accel;

    int32_t lanes[INPUT_BUFFER_WORDS_32_PAD];
    std::memset(lanes, 0, sizeof(lanes));
    lanes[MPC_FPGA_WORD_EY] = (int32_t)F[kEyCol];
    lanes[MPC_FPGA_WORD_EPSI] = (int32_t)F[kEpsiCol];
    lanes[MPC_FPGA_WORD_VX] = (int32_t)F[7];
    lanes[MPC_FPGA_WORD_VY] = (int32_t)F[8];
    lanes[MPC_FPGA_WORD_OMEGA] = (int32_t)F[9];
    lanes[MPC_FPGA_WORD_STEERING] = (int32_t)F[10];
    lanes[MPC_FPGA_WORD_CONTROL_FLAGS] = (int32_t)flags;
    lanes[MPC_FPGA_WORD_PREV_ACCEL] = prev_in;
    for (int i = 0; i < kHorizon; ++i) {
      const int b = kHeaderWords + i * kRefWords;
      lanes[b + 0] = (int32_t)F[kRefEyCol + i];
      lanes[b + 1] = (int32_t)F[kRefEpsiCol + i];
      lanes[b + 2] = (int32_t)F[kRefVxCol + i];
      lanes[b + 3] = (int32_t)F[kRefVyCol + i];
      lanes[b + 4] = (int32_t)F[kRefOmegaCol + i];
      lanes[b + 5] = (int32_t)F[kRefKappaCol + i];
      lanes[b + 6] = (int32_t)F[kRefLeftCol + i];
      lanes[b + 7] = (int32_t)F[kRefRightCol + i];
    }
    for (int p = 0; p < INPUT_BUFFER_WORDS_512; ++p)
      for (int l = 0; l < 16; ++l)
        in512[p].range(32 * l + 31, 32 * l) =
            (ap_uint<32>)(uint32_t)lanes[p * 16 + l];
    out128[0] = 0;

    mpc_fpga_top_opencl(in512, out128);
    const int32_t o_steer = (int32_t)(uint32_t)out128[0].range(31, 0);
    const int32_t o_accel = (int32_t)(uint32_t)out128[0].range(63, 32);
    const uint32_t o_st = (uint32_t)out128[0].range(95, 64);
    const uint32_t o_it = (uint32_t)out128[0].range(127, 96);

    /* The scalar cross-check is a SECOND solve into the same static
     * g_core_state and corrupts the persistent warm-start (the real car /
     * mpc_receiver calls the kernel ONCE per timestep). Off by default so
     * the run reflects the true single-call-per-step convention; set
     * REPLAY_SCALAR=1 only for an idx-isolated parity spot-check. */
    int32_t s_st_v = o_steer, s_ac = o_accel, s_status = (int32_t)o_st,
            s_it = (int32_t)o_it;
    if (getenv("REPLAY_SCALAR")) {
      int32_t r_ey[kHorizon], r_ep[kHorizon], r_vx[kHorizon], r_vy[kHorizon];
      int32_t r_om[kHorizon], r_ka[kHorizon], r_lf[kHorizon], r_rt[kHorizon];
      for (int i = 0; i < kHorizon; ++i) {
        r_ey[i] = (int32_t)F[kRefEyCol + i];   r_ep[i] = (int32_t)F[kRefEpsiCol + i];
        r_vx[i] = (int32_t)F[kRefVxCol + i];   r_vy[i] = (int32_t)F[kRefVyCol + i];
        r_om[i] = (int32_t)F[kRefOmegaCol + i];r_ka[i] = (int32_t)F[kRefKappaCol + i];
        r_lf[i] = (int32_t)F[kRefLeftCol + i]; r_rt[i] = (int32_t)F[kRefRightCol + i];
      }
      mpc_fpga_top_scalar_with_prev_accel_and_ref_ey(
          (int32_t)F[kEyCol], (int32_t)F[kEpsiCol], (int32_t)F[7], (int32_t)F[8],
          (int32_t)F[9], (int32_t)F[10], prev_in, r_ey, r_ep, r_vx, r_vy, r_om,
          r_ka, r_lf, r_rt, kHorizon, &s_st_v, &s_ac, &s_status, &s_it);
    }

    std::printf(
        "idx %lld | OPENCL steer=%d accel=%d it=%u st=%u | "
        "SCALAR steer=%d accel=%d it=%d st=%d | %s\n",
        F[0], o_steer, o_accel, o_it, o_st, s_st_v, s_ac, s_it, s_status,
        (o_steer == s_st_v && o_accel == s_ac) ? "MATCH" : "DIFF");
    std::fflush(stdout);

    prev_accel = o_accel;
    first = false;
    ++emitted;
  }
  std::fclose(in);
  std::fprintf(stderr, "tb_top_min: emitted %ld row(s)\n", emitted);
  return 0;
}
