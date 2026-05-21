/**
 * @file fp_width_probe.hpp
 * @brief Empirical bit-width / range probe for evidence-based fixed-point sizing.
 *
 * Purpose
 * -------
 * Every configurable width and guard constant in fp_types_hls.hpp is sized
 * BELOW the algebraic worst case on purpose. To justify each chosen width in
 * the report, this probe instruments the canonical chokepoints:
 *
 *   - product sites (one per fp_mul_*): the untruncated product magnitude,
 *     reconstructed from the raw operands -> justifies each *_GUARD.
 *   - sum/accumulator sites (the adder-tree helpers + true widest use)
 *     -> justifies each fp_sum*_* width.
 *   - store sites (one per family raw_from helper): the stored value
 *     magnitude -> justifies each family WIDTH / INT_BITS.
 *
 * For every site it tracks, across an entire replay run, the maximum signed
 * two's-complement bit-width and the maximum |value| observed, plus the
 * algebraic worst-case width and the currently chosen width. At exit it
 * prints a human table AND (if FP_WPROBE_CSV is set) appends one machine
 * row per site tagged with FP_WPROBE_LABEL, so a driver can aggregate the
 * same sites across multiple bags into one report.
 *
 * Inert unless FP_WIDTH_PROBE is defined; never active under HLS synthesis.
 *
 * Driver: tools/mpc_replay/run_width_report.sh (multi-bag) /
 *         tools/mpc_replay/run_width_probe.sh   (single quick run).
 */

#ifndef FP_WIDTH_PROBE_HPP
#define FP_WIDTH_PROBE_HPP

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
  /* ---- product (guard) sites ---- */
  FP_WP_QP_MUL = 0,     /* QP_raw*QP_raw   -> fp_QP_mul_t            */
  FP_WP_P_QP_MUL,       /* P_raw*QP_raw    -> fp_P_QP_mul_t         */
  FP_WP_MG_QP_MUL,      /* MG_raw*QP_raw   -> fp_MG_QP_mul_t        */
  FP_WP_MG_K_MUL,       /* MG_raw*K_raw    -> fp_MG_K_mul_t         */
  FP_WP_K_QP_MUL,       /* K_raw*QP_raw    -> fp_K_QP_mul_t         */
  FP_WP_FN_MUL,         /* FN*FN           -> fp_fn_accum_t         */
  /* ---- sum / accumulator sites ---- */
  FP_WP_SUM2_QP_RAW,
  FP_WP_SUM4_QP_RAW,
  FP_WP_SUM8_QP_RAW,
  FP_WP_SUM6_QP,        /* sum6_QP_raw tree only                   */
  FP_WP_SUM6_QP_ACC,    /* TRUE fp_sum6_QP_mul_t widest use         */
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
  /* ---- single cast-product (must also fit the sum type) ---- */
  FP_WP_QP_ITEM,
  FP_WP_P_QP_ITEM,
  FP_WP_P_MIX_ITEM,
  FP_WP_MG_QP_ITEM,
  FP_WP_K_QP_ITEM,
  /* ---- store (family WIDTH / INT_BITS) sites ---- */
  FP_WP_QP_STORE,       /* fp_QP_t  raw value  -> MPC_HLS_QP_WIDTH  */
  FP_WP_FN_STORE,       /* fp_FN_t                MPC_HLS_FN_WIDTH  */
  FP_WP_P_STORE,        /* fp_P_t                 MPC_HLS_P_WIDTH   */
  FP_WP_MG_STORE,       /* fp_MG_t                MPC_HLS_MG_WIDTH  */
  FP_WP_K_STORE,        /* fp_K_t                 MPC_HLS_K_WIDTH   */
  FP_WP_COUNT
};

struct FpWidthProbeSlot {
  const char *name;     /* report row label                       */
  const char *kind;     /* "MUL" | "SUM" | "ITEM" | "STORE"        */
  int max_width;        /* max signed two's-complement bits seen   */
  long long max_abs;    /* max |value| seen (raw LSBs)             */
  int algebraic_worst;  /* worst-case width (0 = data-bounded)     */
  int chosen_width;     /* width actually used by the typedef      */
  unsigned long long samples;
  /* ---- STORE-only family-width decomposition (0 for non-store) ---- */
  int frac_bits;            /* family fractional bits (resolution)  */
  int int_bits_used;        /* signed bits of the integer part used */
  int min_trailing_zeros;   /* min trailing-zero bits over nonzero  */
  unsigned long long min_nz_abs;  /* smallest nonzero |raw| seen    */
  unsigned long long nz_samples;  /* count of nonzero samples       */
};

inline void fp_width_probe_print();

inline FpWidthProbeSlot *fp_width_probe_table() {
  /* {name, kind, max_w, max_abs, algebraic_worst, chosen_width, samples} */
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

/* Minimal signed two's-complement bit-width that holds v. */
inline int fp_width_probe_signed_bits(__int128 v) {
  unsigned __int128 mag = (v < 0) ? (unsigned __int128)(~v) : (unsigned __int128)v;
  int bits = 0;
  while (mag) { ++bits; mag >>= 1; }
  return bits + 1; /* + sign bit */
}

inline void fp_width_probe_print() {
  FpWidthProbeSlot *t = fp_width_probe_table();
  const char *label = std::getenv("FP_WPROBE_LABEL");
  if (!label) label = "run";

  std::fprintf(stderr,
    "\n=============== FP WIDTH PROBE SUMMARY [%s] ===============\n", label);
  std::fprintf(stderr, "%-26s %-6s %8s %8s %8s %8s %14s\n",
    "site", "kind", "obs_bits", "worst", "chosen", "margin", "samples");
  for (int i = 0; i < FP_WP_COUNT; ++i) {
    int margin = t[i].chosen_width - t[i].max_width;       /* spare bits */
    std::fprintf(stderr, "%-26s %-6s %8d %8d %8d %8d %14llu\n",
      t[i].name, t[i].kind, t[i].max_width,
      t[i].algebraic_worst, t[i].chosen_width, margin, t[i].samples);
  }
  std::fprintf(stderr,
    "obs_bits = max signed bits observed; worst = algebraic worst case "
    "(0 = data-bounded store);\nchosen = typedef width; margin = chosen - "
    "observed (>=1 required for safety).\n"
    "==================================================================\n\n");

  /* Machine-readable append for multi-bag aggregation. */
  const char *csv = std::getenv("FP_WPROBE_CSV");
  if (csv) {
    std::FILE *f = std::fopen(csv, "a");
    if (f) {
      for (int i = 0; i < FP_WP_COUNT; ++i) {
        std::fprintf(f, "%s,%s,%s,%d,%lld,%d,%d,%llu,%d,%d,%d,%llu,%llu\n",
          label, t[i].name, t[i].kind, t[i].max_width, t[i].max_abs,
          t[i].algebraic_worst, t[i].chosen_width, t[i].samples,
          t[i].frac_bits, t[i].int_bits_used, t[i].min_trailing_zeros,
          t[i].min_nz_abs, t[i].nz_samples);
      }
      std::fclose(f);
    }
  }
}

inline void fp_width_probe_record(int id, __int128 wide_value) {
  fp_width_probe_register_once();
  FpWidthProbeSlot *t = fp_width_probe_table();
  int w = fp_width_probe_signed_bits(wide_value);
  if (w > t[id].max_width) t[id].max_width = w;
  long long a = (long long)(wide_value < 0 ? -wide_value : wide_value);
  if (a > t[id].max_abs) t[id].max_abs = a;
  ++t[id].samples;
}

/* STORE recorder: decomposes a stored fixed-point raw code into the
 * INTEGER bits actually needed (dynamic range, vs INT_BITS) and the
 * FRACTIONAL bits actually exercised (resolution, vs FRAC_BITS). The
 * deepest fractional bit ever set = frac - min_trailing_zeros: any
 * trailing-zero bits common to every nonzero sample are fractional
 * resolution the format provides but the data never uses. */
inline void fp_width_probe_record_store(int id, long long raw, int frac) {
  fp_width_probe_register_once();
  FpWidthProbeSlot *t = fp_width_probe_table();
  t[id].frac_bits = frac;
  ++t[id].samples;
  int w = fp_width_probe_signed_bits((__int128)raw);
  if (w > t[id].max_width) t[id].max_width = w;
  unsigned long long a =
      (unsigned long long)(raw < 0 ? -raw : raw);
  if ((long long)a > t[id].max_abs) t[id].max_abs = (long long)a;
  if (a == 0ULL) return; /* zero says nothing about range or resolution */
  unsigned long long ip = a >> frac;          /* integer-part magnitude */
  int ib = 1;                                 /* sign bit               */
  while (ip) { ++ib; ip >>= 1; }
  if (ib > t[id].int_bits_used) t[id].int_bits_used = ib;
  int tz = 0;
  while (((a >> tz) & 1ULL) == 0ULL) ++tz;     /* a != 0 here            */
  if (t[id].nz_samples == 0 || tz < t[id].min_trailing_zeros)
    t[id].min_trailing_zeros = tz;
  if (t[id].nz_samples == 0 || a < t[id].min_nz_abs)
    t[id].min_nz_abs = a;
  ++t[id].nz_samples;
}

#define FP_WPROBE(id, wide_value) fp_width_probe_record((id), (__int128)(wide_value))
#define FP_WPROBE_STORE(id, raw, frac)                                         \
  fp_width_probe_record_store((id), (long long)(raw), (int)(frac))
/* Per-input magnitude: records max bit-width across all 6/8 cast products. */
#define FP_WPROBE6(id, a0, a1, a2, a3, a4, a5)                                  \
  do {                                                                         \
    FP_WPROBE(id, (a0).to_int64());  FP_WPROBE(id, (a1).to_int64());            \
    FP_WPROBE(id, (a2).to_int64());  FP_WPROBE(id, (a3).to_int64());            \
    FP_WPROBE(id, (a4).to_int64());  FP_WPROBE(id, (a5).to_int64());            \
  } while (0)
#define FP_WPROBE8(id, a0, a1, a2, a3, a4, a5, a6, a7)                          \
  do {                                                                         \
    FP_WPROBE6(id, a0, a1, a2, a3, a4, a5);                                     \
    FP_WPROBE(id, (a6).to_int64());  FP_WPROBE(id, (a7).to_int64());            \
  } while (0)

#else /* probe disabled: zero cost, synthesis-safe */

#define FP_WPROBE(id, wide_value) ((void)0)
#define FP_WPROBE_STORE(id, raw, frac) ((void)0)
#define FP_WPROBE6(id, a0, a1, a2, a3, a4, a5) ((void)0)
#define FP_WPROBE8(id, a0, a1, a2, a3, a4, a5, a6, a7) ((void)0)

#endif

#endif /* FP_WIDTH_PROBE_HPP */
