#include "../include/fp_math_hls.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

template <typename RawT, typename FpT>
struct RecipStats {
  double median_lsb;
  double p95_lsb;
  double p99_lsb;
  double max_lsb;
  size_t samples;
};

template <typename RawT, typename FpT, typename FromRawFn, typename RecipFn>
static RecipStats<RawT, FpT>
measure_family(int frac_bits, FromRawFn from_raw, RecipFn recip_fn) {
  const RawT half_raw = (RawT)1 << (frac_bits - 1);
  const RawT one_raw = (RawT)1 << frac_bits;
  const double lsb = std::ldexp(1.0, -frac_bits);

  std::vector<double> errs;
  errs.reserve((size_t)(one_raw - half_raw) * 2u);

  for (RawT mag_raw = half_raw; mag_raw < one_raw; ++mag_raw) {
    for (int sign = -1; sign <= 1; sign += 2) {
      const RawT raw = (sign < 0) ? (RawT)(-mag_raw) : mag_raw;
      const FpT x = from_raw(raw);
      const FpT y = recip_fn(x);
      const double xd = (double)x;
      const double yd = (double)y;
      const double ref = 1.0 / xd;
      errs.push_back(std::abs(yd - ref) / lsb);
    }
  }

  std::sort(errs.begin(), errs.end());
  const auto pick = [&](double q) -> double {
    const size_t idx = (size_t)(q * (double)(errs.size() - 1));
    return errs[idx];
  };

  return {
      pick(0.50),
      pick(0.95),
      pick(0.99),
      errs.back(),
      errs.size(),
  };
}

int main() {
  const auto fn_stats =
      measure_family<fp_fn_raw_t, fp_FN_t>(FP_FN_FRAC_BITS, fp_FN_from_fn_raw,
                                           fp_recip_fn);
  const auto qp_stats =
      measure_family<fp_QP_raw_t, fp_QP_t>(FP_FRAC_BITS, fp_QP_from_qp_raw,
                                           fp_recip);

  std::printf("Family  Samples  Median_LSB  P95_LSB  P99_LSB  Max_LSB\n");
  std::printf("FN      %zu  %.6f  %.6f  %.6f  %.6f\n", fn_stats.samples,
              fn_stats.median_lsb, fn_stats.p95_lsb, fn_stats.p99_lsb,
              fn_stats.max_lsb);
  std::printf("QP      %zu  %.6f  %.6f  %.6f  %.6f\n", qp_stats.samples,
              qp_stats.median_lsb, qp_stats.p95_lsb, qp_stats.p99_lsb,
              qp_stats.max_lsb);
  return 0;
}
