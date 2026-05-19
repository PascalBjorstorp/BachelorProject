/**
 * @file tb_scalar_min.cpp
 * @brief Bisect harness: ONLY the scalar entry, ONE call per row, fed the RAW
 *        stored state_replay.csv directly (no OpenCL top, no float round-trip,
 *        no double-call). Isolates "OpenCL-top wrapper" vs "compute core".
 *
 * Compare convergence vs tb_top_min (which calls mpc_fpga_top_opencl):
 *   ~99% converge here -> bug is in mpc_fpga_top_opencl's wrapper layer.
 *   ~40% (same as opencl) -> bug is in mpc_fpga_compute_core under raw inputs.
 *
 * Usage: tb_scalar_min <state_replay.csv> [max_rows]
 */
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "mpc_fpga_constants.h"

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
constexpr int kH = MPC_FPGA_HORIZON_STEPS;
constexpr int kRefEy = 12, kRefEpsi = kRefEy + kH, kRefVx = 112,
              kRefVy = kRefVx + kH, kRefOm = kRefVy + kH, kRefKa = kRefOm + kH,
              kRefLf = kRefKa + kH, kRefRt = kRefLf + kH, kEy = 232, kEpsi = 233;
int split_row(char *l, long long *f) {
  int n = 0; char *s = nullptr;
  for (char *t = strtok_r(l, ",\r\n", &s); t && n < kCols;
       t = strtok_r(nullptr, ",\r\n", &s))
    f[n++] = strtoll(t, nullptr, 10);
  return n;
}
}  // namespace

int main(int argc, char **argv) {
  const char *csv = argc > 1 ? argv[1] : "state_replay.csv";
  const long maxr = argc > 2 ? atol(argv[2]) : 1;
  std::FILE *in = std::fopen(csv, "r");
  if (!in) { std::perror("open"); return 3; }
  static char line[1 << 17];
  if (!std::fgets(line, sizeof(line), in)) return 6;  // header

  int32_t prev_accel = 0;
  long emitted = 0;
  while (emitted < maxr && std::fgets(line, sizeof(line), in)) {
    long long F[kCols] = {0};
    if (split_row(line, F) < kCols) continue;
    if (F[11] < kH) continue;
    int32_t r_ey[kH], r_ep[kH], r_vx[kH], r_vy[kH], r_om[kH], r_ka[kH],
        r_lf[kH], r_rt[kH];
    for (int i = 0; i < kH; ++i) {
      r_ey[i] = (int32_t)F[kRefEy + i]; r_ep[i] = (int32_t)F[kRefEpsi + i];
      r_vx[i] = (int32_t)F[kRefVx + i]; r_vy[i] = (int32_t)F[kRefVy + i];
      r_om[i] = (int32_t)F[kRefOm + i]; r_ka[i] = (int32_t)F[kRefKa + i];
      r_lf[i] = (int32_t)F[kRefLf + i]; r_rt[i] = (int32_t)F[kRefRt + i];
    }
    int32_t st_v = 0, ac = 0, sttus = 0, it = 0;
    mpc_fpga_top_scalar_with_prev_accel_and_ref_ey(
        (int32_t)F[kEy], (int32_t)F[kEpsi], (int32_t)F[7], (int32_t)F[8],
        (int32_t)F[9], (int32_t)F[10], prev_accel, r_ey, r_ep, r_vx, r_vy,
        r_om, r_ka, r_lf, r_rt, kH, &st_v, &ac, &sttus, &it);
    std::printf("idx %lld | SCALAR steer=%d accel=%d it=%d st=%d\n", F[0],
                st_v, ac, it, sttus);
    prev_accel = ac;
    ++emitted;
  }
  std::fclose(in);
  std::fprintf(stderr, "tb_scalar_min: emitted %ld row(s)\n", emitted);
  return 0;
}
