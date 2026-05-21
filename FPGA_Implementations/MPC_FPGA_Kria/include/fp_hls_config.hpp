/**
 * @file fp_hls_config.hpp
 * @brief Merged HLS build controls: pragma ablation, width profiles, and probes.
 *
 * This file is intentionally multi-include. The profile/ablation section is
 * always available; the width-probe section is only emitted when the includer
 * defines `FP_HLS_CONFIG_INCLUDE_PROBE` before including this file.
 */

#ifndef FP_HLS_CONFIG_PROFILE_SECTION
#define FP_HLS_CONFIG_PROFILE_SECTION

/* Pragma ablation switches for the MPC FPGA build.
 *
 * The PIPELINE and UNROLL pragmas in src/ and include/fp_math_hls.h are
 * routed through these macros so a single dial flips a whole class of
 * directives without removing any pragma from the source tree.
 */
#define MPC_HLS_ABLATION_BASELINE    0
#define MPC_HLS_ABLATION_UNROLL_ONLY 1
#define MPC_HLS_ABLATION_SEQUENTIAL  2

#ifndef MPC_HLS_ABLATION_MODE
#define MPC_HLS_ABLATION_MODE MPC_HLS_ABLATION_BASELINE
#endif

#define MPC_HLS_STR_(x) #x
#define MPC_HLS_XSTR_(x) MPC_HLS_STR_(x)

#if MPC_HLS_ABLATION_MODE >= MPC_HLS_ABLATION_UNROLL_ONLY
#define MPC_HLS_PIPELINE(N) _Pragma("HLS PIPELINE off")
#else
#define MPC_HLS_PIPELINE(N) _Pragma(MPC_HLS_XSTR_(HLS PIPELINE II = N))
#endif

#if MPC_HLS_ABLATION_MODE >= MPC_HLS_ABLATION_SEQUENTIAL
#define MPC_HLS_UNROLL() _Pragma("HLS UNROLL factor=1")
#define MPC_HLS_UNROLL_FACTOR(N) _Pragma("HLS UNROLL factor=1")
#else
#define MPC_HLS_UNROLL() _Pragma("HLS UNROLL")
#define MPC_HLS_UNROLL_FACTOR(N) _Pragma(MPC_HLS_XSTR_(HLS UNROLL factor = N))
#endif

/* Width profile selector for non-store intermediate/product/sum families.
 *
 * QP transport/store remains fixed to the external 32-bit payload format.
 * Other stored families may optionally be widened with MPC_HLS_STORE_WIDTH_PAD.
 */
#define MPC_HLS_PROFILE_PLUS1 1
#define MPC_HLS_PROFILE_PLUS4 4
#define MPC_HLS_PROFILE_PLUS8 8
#define MPC_HLS_PROFILE_WORST (-1)

#define MPC_HLS_STORE_PAD_EXACT 0
#define MPC_HLS_STORE_PAD_PLUS1 1
#define MPC_HLS_STORE_PAD_PLUS4 4

#ifndef MPC_HLS_WIDTH_PROFILE
#define MPC_HLS_WIDTH_PROFILE MPC_HLS_PROFILE_PLUS1
#endif

#ifndef MPC_HLS_STORE_WIDTH_PAD
#define MPC_HLS_STORE_WIDTH_PAD MPC_HLS_STORE_PAD_EXACT
#endif

#endif /* FP_HLS_CONFIG_PROFILE_SECTION */

#ifdef FP_HLS_CONFIG_INCLUDE_PROBE
#ifndef FP_HLS_CONFIG_PROBE_SECTION
#define FP_HLS_CONFIG_PROBE_SECTION

#if defined(FP_WIDTH_PROBE) && !defined(__SYNTHESIS__)

#include "mpc_fpga_constants.h"
#include <cstdint>
#include <cstdio>
#include <cstdlib>

/* Algebraic worst-case product widths (sum of raw operand widths). */
#define FP_WP_AW_QP   (2 * MPC_HLS_QP_WIDTH)
#define FP_WP_AW_P_QP (MPC_HLS_P_WIDTH + MPC_HLS_QP_WIDTH)
#define FP_WP_AW_MGQP (MPC_HLS_MG_WIDTH + MPC_HLS_QP_WIDTH)
#define FP_WP_AW_MG_K (MPC_HLS_MG_WIDTH + MPC_HLS_K_WIDTH)
#define FP_WP_AW_K_QP (MPC_HLS_K_WIDTH + MPC_HLS_QP_WIDTH)
#define FP_WP_AW_FN   (2 * MPC_HLS_FN_WIDTH)

enum FpWidthProbeId {
  FP_WP_QP_MUL = 0,
  FP_WP_P_QP_MUL,
  FP_WP_MG_QP_MUL,
  FP_WP_MG_K_MUL,
  FP_WP_K_QP_MUL,
  FP_WP_FN_MUL,
  FP_WP_SUM2_QP_RAW,
  FP_WP_SUM4_QP_RAW,
  FP_WP_SUM8_QP_RAW,
  FP_WP_SUM6_QP,
  FP_WP_SUM6_QP_ACC,
  FP_WP_SUM2_P_RAW,
  FP_WP_SUM6_P_QP,
  FP_WP_SUM2_P_QP,
  FP_WP_SUM4_P_QP,
  FP_WP_SUM2_P_MIX,
  FP_WP_SUM4_P_MIX,
  FP_WP_SUM8_P_MIX,
  FP_WP_SUM8_P_MIX_PUP,
  FP_WP_SUM2_MG_RAW,
  FP_WP_SUM6_MG_QP,
  FP_WP_SUM2_MG_QP,
  FP_WP_SUM4_MG_QP,
  FP_WP_SUM2_QP_MG,
  FP_WP_SUM2_MG_K,
  FP_WP_SUM2_K_QP,
  FP_WP_SUM4_K_QP,
  FP_WP_SUM8_K_QP,
  FP_WP_QP_RECIP_SHIFT,
  FP_WP_FN_RECIP_SHIFT,
  FP_WP_QP_DET_MUL,
  FP_WP_QP_ITEM,
  FP_WP_P_QP_ITEM,
  FP_WP_P_MIX_ITEM,
  FP_WP_MG_QP_ITEM,
  FP_WP_K_QP_ITEM,
  FP_WP_QP_STORE,
  FP_WP_FN_STORE,
  FP_WP_P_STORE,
  FP_WP_MG_STORE,
  FP_WP_K_STORE,
  FP_WP_COUNT
};

struct FpWidthProbeSlot {
  const char *name;
  const char *kind;
  int max_width;
  long long max_abs;
  int algebraic_worst;
  int chosen_width;
  unsigned long long samples;
  int frac_bits;
  int int_bits_used;
  int min_trailing_zeros;
  unsigned long long min_nz_abs;
  unsigned long long nz_samples;
};

inline void fp_width_probe_print();

inline FpWidthProbeSlot *fp_width_probe_table() {
  static FpWidthProbeSlot t[FP_WP_COUNT] = {
    {"QP_MUL    fp_QP_mul_t",        "MUL", 0,0, FP_WP_AW_QP,   MPC_HLS_QP_WIDTH+MPC_HLS_QP_GUARD, 0},
    {"P_QP_MUL  fp_P_QP_mul_t",      "MUL", 0,0, FP_WP_AW_P_QP, MPC_HLS_P_WIDTH+MPC_HLS_P_QP_GUARD, 0},
    {"MG_QP_MUL fp_MG_QP_mul_t",     "MUL", 0,0, FP_WP_AW_MGQP, MPC_HLS_MG_WIDTH+MPC_HLS_MG_QP_GUARD, 0},
    {"MG_K_MUL  fp_MG_K_mul_t",      "MUL", 0,0, FP_WP_AW_MG_K, MPC_HLS_MG_WIDTH+MPC_HLS_MG_K_GUARD, 0},
    {"K_QP_MUL  fp_K_QP_mul_t",      "MUL", 0,0, FP_WP_AW_K_QP, MPC_HLS_K_WIDTH+MPC_HLS_K_QP_GUARD, 0},
    {"FN_MUL    fp_fn_accum_t",      "MUL", 0,0, FP_WP_AW_FN,   MPC_HLS_FN_WIDTH+MPC_HLS_FN_GUARD, 0},
    {"SUM2_QP_RAW",                  "SUM", 0,0, MPC_HLS_QP_WIDTH+2, MPC_HLS_SUM2_QP_RAW_WIDTH, 0},
    {"SUM4_QP_RAW",                  "SUM", 0,0, MPC_HLS_QP_WIDTH+3, MPC_HLS_SUM4_QP_RAW_WIDTH, 0},
    {"SUM8_QP_RAW",                  "SUM", 0,0, MPC_HLS_QP_WIDTH+4, MPC_HLS_SUM8_QP_RAW_WIDTH, 0},
    {"SUM6_QP_tree",                 "SUM", 0,0, FP_WP_AW_QP+3, MPC_HLS_SUM6_QP_MUL_WIDTH, 0},
    {"SUM6_QP_ACC (true)",           "SUM", 0,0, FP_WP_AW_QP+3, MPC_HLS_SUM6_QP_MUL_WIDTH, 0},
    {"SUM2_P_RAW",                   "SUM", 0,0, MPC_HLS_P_WIDTH+1, MPC_HLS_SUM2_P_RAW_WIDTH, 0},
    {"SUM6_P_QP",                    "SUM", 0,0, FP_WP_AW_P_QP+3, MPC_HLS_SUM6_P_QP_WIDTH, 0},
    {"SUM2_P_QP",                    "SUM", 0,0, FP_WP_AW_P_QP+1, MPC_HLS_SUM2_P_QP_WIDTH, 0},
    {"SUM4_P_QP",                    "SUM", 0,0, FP_WP_AW_P_QP+2, MPC_HLS_SUM4_P_QP_WIDTH, 0},
    {"SUM2_P_MIX",                   "SUM", 0,0, MPC_HLS_P_MIX_MUL_WIDTH+1, MPC_HLS_SUM2_P_MIX_WIDTH, 0},
    {"SUM4_P_MIX",                   "SUM", 0,0, MPC_HLS_P_MIX_MUL_WIDTH+2, MPC_HLS_SUM4_P_MIX_WIDTH, 0},
    {"SUM8_P_MIX",                   "SUM", 0,0, MPC_HLS_P_MIX_MUL_WIDTH+3, MPC_HLS_SUM8_P_MIX_WIDTH, 0},
    {"SUM8_P_MIX_pupdate",           "SUM", 0,0, MPC_HLS_P_MIX_MUL_WIDTH+3, MPC_HLS_SUM8_P_MIX_PUP_WIDTH, 0},
    {"SUM2_MG_RAW",                  "SUM", 0,0, MPC_HLS_MG_WIDTH+1, MPC_HLS_SUM2_MG_RAW_WIDTH, 0},
    {"SUM6_MG_QP",                   "SUM", 0,0, FP_WP_AW_MGQP+3, MPC_HLS_SUM6_MG_QP_WIDTH, 0},
    {"SUM2_MG_QP",                   "SUM", 0,0, FP_WP_AW_MGQP+1, MPC_HLS_SUM2_MG_QP_WIDTH, 0},
    {"SUM4_MG_QP",                   "SUM", 0,0, FP_WP_AW_MGQP+2, MPC_HLS_SUM4_MG_QP_WIDTH, 0},
    {"SUM2_QP_MG",                   "SUM", 0,0, FP_WP_AW_MGQP+1, MPC_HLS_SUM2_QP_MG_WIDTH, 0},
    {"SUM2_MG_K",                    "SUM", 0,0, FP_WP_AW_MG_K+1, MPC_HLS_SUM2_MG_K_WIDTH, 0},
    {"SUM2_K_QP",                    "SUM", 0,0, FP_WP_AW_K_QP+1, MPC_HLS_SUM2_K_QP_WIDTH, 0},
    {"SUM4_K_QP",                    "SUM", 0,0, FP_WP_AW_K_QP+2, MPC_HLS_SUM4_K_QP_WIDTH, 0},
    {"SUM8_K_QP",                    "SUM", 0,0, FP_WP_AW_K_QP+3, MPC_HLS_SUM8_K_QP_WIDTH, 0},
    {"QP_RECIP_SHIFT",               "SUM", 0,0, MPC_HLS_QP_WIDTH + MPC_HLS_QP_FRAC_BITS, MPC_HLS_QP_RECIP_SHIFT_WIDTH, 0},
    {"FN_RECIP_SHIFT",               "SUM", 0,0, MPC_HLS_FN_WIDTH + MPC_HLS_FN_FRAC_BITS, MPC_HLS_FN_RECIP_SHIFT_WIDTH, 0},
    {"QP_DET_MUL",                   "SUM", 0,0, (2 * MPC_HLS_QP_WIDTH) + 1, MPC_HLS_QP_DET_MUL_WIDTH, 0},
    {"QP_ITEM   single product",     "ITEM",0,0, FP_WP_AW_QP,   MPC_HLS_SUM6_QP_MUL_WIDTH, 0},
    {"P_QP_ITEM single product",     "ITEM",0,0, FP_WP_AW_P_QP, MPC_HLS_SUM6_P_QP_WIDTH, 0},
    {"P_MIX_ITEM single product",    "ITEM",0,0, MPC_HLS_P_MIX_MUL_WIDTH, MPC_HLS_P_MIX_ITEM_WIDTH, 0},
    {"MG_QP_ITEM single product",    "ITEM",0,0, FP_WP_AW_MGQP, MPC_HLS_SUM6_MG_QP_WIDTH, 0},
    {"K_QP_ITEM single product",     "ITEM",0,0, FP_WP_AW_K_QP, MPC_HLS_K_QP_ITEM_WIDTH, 0},
    {"QP_STORE  fp_QP_t",  "STORE",0,0, 0, MPC_HLS_QP_WIDTH, 0, MPC_HLS_QP_FRAC_BITS},
    {"FN_STORE  fp_FN_t",  "STORE",0,0, 0, MPC_HLS_FN_WIDTH, 0, MPC_HLS_FN_FRAC_BITS},
    {"P_STORE   fp_P_t",   "STORE",0,0, 0, MPC_HLS_P_WIDTH,  0, MPC_HLS_P_FRAC_BITS},
    {"MG_STORE  fp_MG_t",  "STORE",0,0, 0, MPC_HLS_MG_WIDTH, 0, MPC_HLS_MG_FRAC_BITS},
    {"K_STORE   fp_K_t",   "STORE",0,0, 0, MPC_HLS_K_WIDTH,  0, MPC_HLS_K_FRAC_BITS},
  };
  return t;
}

inline void fp_width_probe_register_once() {
  static bool registered = false;
  if (!registered) {
    registered = true;
    std::atexit(fp_width_probe_print);
  }
}

inline int fp_width_probe_signed_bits(__int128 v) {
  unsigned __int128 mag = (v < 0) ? (unsigned __int128)(~v) : (unsigned __int128)v;
  int bits = 0;
  while (mag) {
    ++bits;
    mag >>= 1;
  }
  return bits + 1;
}

inline void fp_width_probe_print() {
  FpWidthProbeSlot *t = fp_width_probe_table();
  const char *label = std::getenv("FP_WPROBE_LABEL");
  if (!label)
    label = "run";

  std::fprintf(stderr,
               "\n=============== FP WIDTH PROBE SUMMARY [%s] ===============\n",
               label);
  std::fprintf(stderr, "%-26s %-6s %8s %8s %8s %8s %14s\n",
               "site", "kind", "obs_bits", "worst", "chosen", "margin",
               "samples");
  for (int i = 0; i < FP_WP_COUNT; ++i) {
    int margin = t[i].chosen_width - t[i].max_width;
    std::fprintf(stderr, "%-26s %-6s %8d %8d %8d %8d %14llu\n",
                 t[i].name, t[i].kind, t[i].max_width,
                 t[i].algebraic_worst, t[i].chosen_width, margin,
                 t[i].samples);
  }
  std::fprintf(stderr,
               "obs_bits = max signed bits observed; worst = algebraic worst case "
               "(0 = data-bounded store);\nchosen = typedef width; margin = chosen - "
               "observed (>=1 required for safety).\n"
               "==================================================================\n\n");

  const char *csv = std::getenv("FP_WPROBE_CSV");
  if (csv) {
    std::FILE *f = std::fopen(csv, "a");
    if (f) {
      for (int i = 0; i < FP_WP_COUNT; ++i) {
        std::fprintf(f, "%s,%s,%s,%d,%lld,%d,%d,%llu,%d,%d,%d,%llu,%llu\n",
                     label, t[i].name, t[i].kind, t[i].max_width,
                     t[i].max_abs, t[i].algebraic_worst, t[i].chosen_width,
                     t[i].samples, t[i].frac_bits, t[i].int_bits_used,
                     t[i].min_trailing_zeros, t[i].min_nz_abs, t[i].nz_samples);
      }
      std::fclose(f);
    }
  }
}

inline void fp_width_probe_record(int id, __int128 wide_value) {
  fp_width_probe_register_once();
  FpWidthProbeSlot *t = fp_width_probe_table();
  int w = fp_width_probe_signed_bits(wide_value);
  if (w > t[id].max_width)
    t[id].max_width = w;
  long long a = (long long)(wide_value < 0 ? -wide_value : wide_value);
  if (a > t[id].max_abs)
    t[id].max_abs = a;
  ++t[id].samples;
}

inline void fp_width_probe_record_store(int id, long long raw, int frac) {
  fp_width_probe_register_once();
  FpWidthProbeSlot *t = fp_width_probe_table();
  t[id].frac_bits = frac;
  ++t[id].samples;
  int w = fp_width_probe_signed_bits((__int128)raw);
  if (w > t[id].max_width)
    t[id].max_width = w;
  unsigned long long a = (unsigned long long)(raw < 0 ? -raw : raw);
  if ((long long)a > t[id].max_abs)
    t[id].max_abs = (long long)a;
  if (a == 0ULL)
    return;
  unsigned long long ip = a >> frac;
  int ib = 1;
  while (ip) {
    ++ib;
    ip >>= 1;
  }
  if (ib > t[id].int_bits_used)
    t[id].int_bits_used = ib;
  int tz = 0;
  while (((a >> tz) & 1ULL) == 0ULL)
    ++tz;
  if (t[id].nz_samples == 0 || tz < t[id].min_trailing_zeros)
    t[id].min_trailing_zeros = tz;
  if (t[id].nz_samples == 0 || a < t[id].min_nz_abs)
    t[id].min_nz_abs = a;
  ++t[id].nz_samples;
}

#define FP_WPROBE(id, wide_value) fp_width_probe_record((id), (__int128)(wide_value))
#define FP_WPROBE_STORE(id, raw, frac) \
  fp_width_probe_record_store((id), (long long)(raw), (int)(frac))

#else

#define FP_WPROBE(id, wide_value) do { (void)(id); (void)(wide_value); } while (0)
#define FP_WPROBE_STORE(id, raw, frac) do { (void)(id); (void)(raw); (void)(frac); } while (0)

#endif /* defined(FP_WIDTH_PROBE) && !defined(__SYNTHESIS__) */

#endif /* FP_HLS_CONFIG_PROBE_SECTION */
#endif /* FP_HLS_CONFIG_INCLUDE_PROBE */
