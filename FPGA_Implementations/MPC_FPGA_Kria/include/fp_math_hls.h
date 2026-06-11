/**
 * @file fp_math_hls.h
 * @brief Fixed-point math helpers for HLS synthesis.
 */

#ifndef FP_MATH_HLS_H
#define FP_MATH_HLS_H

#include "fp_hls_config.hpp"
#include "fp_types_hls.hpp"
#include <climits>
#include <cstdint>

/* DSP multiply pipeline latency. MUST be >=2: the binding multiplies are
 * QP*QP and P*QP at 26x26, which need TWO cascaded DSP48E2s (both operands
 * exceed the 27x18 primitive). A 2-DSP cascade is ~4.17ns of logic and cannot
 * complete in one clock -- latency=1 synthesized at WNS -0.268 (failing path:
 * forward-pass mul_26s_26s, full DSP traversal in one cycle). QP is
 * protocol-pinned at 26-bit so no family-width change can avoid this; latency=2
 * is the floor. (F=14 + latency=2 closes at WNS +0.002.) */
#ifndef MPC_HLS_MUL_LATENCY
#define MPC_HLS_MUL_LATENCY 2
#endif

/* Per-multiply latency by operand width. A product fits ONE DSP48E2 (27x18)
 * when the smaller operand is <=18 bits and the larger <=27; a single DSP can
 * close at the lower latency, while a 2-DSP cascade (26x26 QP*QP, P*QP with
 * P>=21) cannot and must keep MPC_HLS_MUL_LATENCY. Deriving latency from the
 * family widths means a multiply auto-drops to lat1 the moment its operands
 * shrink under the 1-DSP line -- no per-site edits. With today's widths this
 * flips K*QP / QP*K / MG*K / K*MG (K=17<=18) to lat1; MG*QP/QP*MG upgrade once
 * MG<=18. latency=1 paths MUST be WNS-verified in synthesis. */
#ifndef MPC_HLS_MUL_LATENCY_1DSP
#define MPC_HLS_MUL_LATENCY_1DSP 1
#endif
#define MPC_HLS_MUL_MIN2(A, B) ((A) < (B) ? (A) : (B))
#define MPC_HLS_MUL_MAX2(A, B) ((A) > (B) ? (A) : (B))
#define MPC_HLS_MUL_LAT(WA, WB)                                                \
  ((MPC_HLS_MUL_MIN2((WA), (WB)) <= 18 && MPC_HLS_MUL_MAX2((WA), (WB)) <= 27)  \
       ? MPC_HLS_MUL_LATENCY_1DSP                                              \
       : MPC_HLS_MUL_LATENCY)

#define FP_FRAC_BITS (MPC_HLS_QP_FRAC_BITS)

/* atan LUT domain. The 1024-entry atan tables span [0, FP_ATAN_LUT_DOMAIN]
 * directly, so atan() is a pure LUT+lerp with NO reciprocal range-reduction.
 * Domain 8 (a power of two) is chosen so the index scale /FP_ATAN_LUT_DOMAIN
 * lowers to a single >>3 shift (zero multiply). Every atan argument here is
 * provably inside it: kinematic L*kappa <= 0.65; slip ratios < tan(15deg);
 * Pacejka B*alpha with B_FRONT=2.35/B_REAR=1.90 -> peaks at ~1.09, <=0.62
 * in-grip, and <=4.62 even in the premise-violating fully-sideways case
 * (1.7x margin under 8). Linear-interp error at 1024 entries over [0,8] is
 * ~5.0e-6 rad ~= 0.65 LSB of fp_FN_t (2^-17) -- below the table-entry
 * quantization floor, so accuracy is unchanged. If this constant changes,
 * regenerate the atan blocks of fp_trig_lut*_1024.h (generate_fn_luts.py)
 * with the same value. */
#define FP_ATAN_LUT_DOMAIN 8
/* Power-of-two domain => the index rescale is a bare shift, not a divide.
 * Shift by this instead of "/ FP_ATAN_LUT_DOMAIN" so the source states the
 * hardware fact directly. static_assert keeps the two in lockstep. */
#define FP_ATAN_LUT_DOMAIN_LOG2 3
static_assert((1 << FP_ATAN_LUT_DOMAIN_LOG2) == FP_ATAN_LUT_DOMAIN,
              "FP_ATAN_LUT_DOMAIN_LOG2 must be log2(FP_ATAN_LUT_DOMAIN)");

#define FP_QP_CONST(x) ((fp_QP_t)(x))

#define FP_ONE     FP_QP_CONST(1.0)
#define FP_TWO     FP_QP_CONST(2.0)
#define FP_HALF    FP_QP_CONST(0.5)
#define FP_PI      FP_QP_CONST(3.14159265358979323846)
#define FP_PI_HALF FP_QP_CONST(1.57079632679489661923)
#define FP_TWO_PI  FP_QP_CONST(6.28318530717958647693)

#define INV_FACT_2 FP_QP_CONST(0.5)
#define INV_FACT_3 FP_QP_CONST(0.16666666666666666)
#define INV_FACT_4 FP_QP_CONST(0.041666666666666664)
#define INV_FACT_5 FP_QP_CONST(0.008333333333333333)

#define FP_INVERT_2X2_DET_MIN_EXP 12
#define FP_INVERT_2X2_DIAG_FALLBACK_MIN_EXP 8

#define FP_FN_CONST(x) ((fp_FN_t)(x))
#define FP_FN_FRAC_BITS (MPC_HLS_FN_FRAC_BITS)
#define FP_FN_ONE FP_FN_CONST(1.0)
#define FP_FN_TWO FP_FN_CONST(2.0)
#define FP_FN_HALF FP_FN_CONST(0.5)
#define FP_FN_PI FP_FN_CONST(3.14159265358979323846)
#define FP_FN_PI_HALF FP_FN_CONST(1.57079632679489661923)
#define FP_FN_ZERO FP_FN_CONST(0.0)
#define FP_FN_TWO_PI FP_FN_CONST(6.28318530717958647693)

#define FP_TRIG_LUT_SIZE 1024
#define FP_TRIG_LUT_MASK (FP_TRIG_LUT_SIZE - 1)
/* Trig LUTs span [0, pi] with 1024 linear-interpolation segments. Angles are
 * normalized to [-pi, pi], folded with symmetry into [0, pi], and sine sign is
 * restored afterward. */
#define FP_TRIG_LUT_SCALE FP_QP_CONST(325.94932345220166780564)
/* FN trig LUT scale 1024/pi kept in raw integer form (F=12, decoupled from QP)
 * and used directly in the FN trig index multiply. Must match the identical
 * definition in fp_trig_lut_fn_1024.h. round(1024/pi * 2^12) = 1335088. */
#define FP_FN_TRIG_LUT_SCALE_RAW ((int32_t)1335088)
/* Reciprocal LUTs cover x_norm in [0.5, 1.0) using linear interpolation.
 * 256 segments were empirically best under this fixed-point lerp pipeline. */
#define FP_RECIP_LUT_BITS 8
#define FP_RECIP_LUT_SIZE (1 << FP_RECIP_LUT_BITS)

fp_QP_t fp_recip(fp_QP_t x);

/* Canonical multiply helpers */
fp_QP_t fp_mul_site(fp_QP_t a, fp_QP_t b, int site_id);

fp_QP_mul_t fp_mul_QP_raw(fp_QP_raw_t a, fp_QP_raw_t b);

/*-------------------------------------------------------------------------
 * Specialized Riccati-family raw multipliers
 *
 * These do not change solver behavior by themselves.
 * They are the arithmetic hooks needed for the staged Riccati-family rewrite.
 *------------------------------------------------------------------------*/

fp_P_QP_mul_t fp_mul_P_QP(fp_P_raw_t a, fp_QP_raw_t b);
fp_P_QP_mul_t fp_mul_QP_P(fp_QP_raw_t a, fp_P_raw_t b);

fp_MG_QP_mul_t fp_mul_MG_QP(fp_MG_raw_t a, fp_QP_raw_t b);
fp_MG_QP_mul_t fp_mul_QP_MG(fp_QP_raw_t a, fp_MG_raw_t b);

fp_MG_K_mul_t fp_mul_MG_K(fp_MG_raw_t a, fp_K_raw_t b);

fp_K_QP_mul_t fp_mul_K_QP(fp_K_raw_t a, fp_QP_raw_t b);

static inline fp_QP_t fp_abs(fp_QP_t a) {
#pragma HLS INLINE
  return (a < 0) ? fp_QP_t(-a) : a;
}

static inline fp_QP_t fp_max2(fp_QP_t a, fp_QP_t b) {
#pragma HLS INLINE
  return (a > b) ? a : b;
}

static inline fp_QP_t fp_clamp(fp_QP_t val, fp_QP_t lo, fp_QP_t hi) {
#pragma HLS INLINE
  if (val < lo)
    return lo;
  if (val > hi)
    return hi;
  return val;
}

static inline fp_QP_raw_t fp_qp_raw_from_neg_pow2(int exp) {
#pragma HLS INLINE
  const int shift = FP_FRAC_BITS - exp;
  if (shift <= 0)
    return (fp_QP_raw_t)1;
  return ((fp_QP_raw_t)1) << shift;
}

static inline fp_QP_t fp_qp_from_neg_pow2(int exp) {
#pragma HLS INLINE
  return fp_QP_from_qp_raw(fp_qp_raw_from_neg_pow2(exp));
}

static inline fp_QP_raw_t fp_sub_cast_qp_raw(fp_QP_raw_t a, fp_QP_raw_t b,
                                              int site_id) {
#pragma HLS INLINE
  fp_sum2_QP_raw_t diff = (fp_sum2_QP_raw_t)a - (fp_sum2_QP_raw_t)b;
  return cast_sum2_qp_raw_to_qp_site(diff, site_id);
}

static inline fp_QP_raw_t fp_add3_cast_qp_raw(fp_QP_raw_t a, fp_QP_raw_t b,
                                               fp_QP_raw_t c, int site_id) {
#pragma HLS INLINE
  fp_sum2_QP_raw_t sum_ab = (fp_sum2_QP_raw_t)a + (fp_sum2_QP_raw_t)b;
  FP_WPROBE(FP_WP_SUM2_QP_RAW, sum_ab.to_int64());
  fp_sum2_QP_raw_t sum_abc = sum_ab + (fp_sum2_QP_raw_t)c;
  return cast_sum2_qp_raw_to_qp_site(sum_abc, site_id);
}

template <typename OutT, typename InT>
static inline OutT fp_shift_right_cast(InT value, int shift) {
#pragma HLS INLINE
  return (OutT)(value >> shift);
}

static fp_sum6_P_QP_t sum6_P_QP_raw(fp_sum6_P_QP_t a0,
                                    fp_sum6_P_QP_t a1,
                                    fp_sum6_P_QP_t a2,
                                    fp_sum6_P_QP_t a3,
                                    fp_sum6_P_QP_t a4,
                                    fp_sum6_P_QP_t a5) {
#pragma HLS INLINE off
MPC_HLS_PIPELINE(1)
  FP_WPROBE(FP_WP_SUM6_P_QP,
            (__int128)a0.to_int64() + (__int128)a1.to_int64() +
                (__int128)a2.to_int64() + (__int128)a3.to_int64() +
                (__int128)a4.to_int64() + (__int128)a5.to_int64());
  FP_WPROBE6(FP_WP_P_QP_ITEM, a0, a1, a2, a3, a4, a5);
  fp_sum6_P_QP_t s01 = a0 + a1;
  fp_sum6_P_QP_t s23 = a2 + a3;
  fp_sum6_P_QP_t s45 = a4 + a5;
  fp_sum6_P_QP_t s0123 = s01 + s23;
  return s0123 + s45;
}

static fp_sum8_P_MIX_t sum8_P_MIX_raw(fp_P_mix_item_t a0,
                                      fp_P_mix_item_t a1,
                                      fp_P_mix_item_t a2,
                                      fp_P_mix_item_t a3,
                                      fp_P_mix_item_t a4,
                                      fp_P_mix_item_t a5,
                                      fp_P_mix_item_t a6,
                                      fp_P_mix_item_t a7) {
#pragma HLS INLINE off
MPC_HLS_PIPELINE(1)
  FP_WPROBE(FP_WP_SUM8_P_MIX,
            (__int128)a0.to_int64() + (__int128)a1.to_int64() +
                (__int128)a2.to_int64() + (__int128)a3.to_int64() +
                (__int128)a4.to_int64() + (__int128)a5.to_int64() +
                (__int128)a6.to_int64() + (__int128)a7.to_int64());
  FP_WPROBE8(FP_WP_P_MIX_ITEM, a0, a1, a2, a3, a4, a5, a6, a7);
  fp_sum2_P_MIX_t s01 = a0 + a1;
  fp_sum2_P_MIX_t s23 = a2 + a3;
  fp_sum2_P_MIX_t s45 = a4 + a5;
  fp_sum2_P_MIX_t s67 = a6 + a7;
  FP_WPROBE(FP_WP_SUM2_P_MIX, s01.to_int64());
  FP_WPROBE(FP_WP_SUM2_P_MIX, s23.to_int64());
  FP_WPROBE(FP_WP_SUM2_P_MIX, s45.to_int64());
  FP_WPROBE(FP_WP_SUM2_P_MIX, s67.to_int64());
  fp_sum4_P_MIX_t s0123 = s01 + s23;
  fp_sum4_P_MIX_t s4567 = s45 + s67;
  FP_WPROBE(FP_WP_SUM4_P_MIX, s0123.to_int64());
  FP_WPROBE(FP_WP_SUM4_P_MIX, s4567.to_int64());
  return (fp_sum8_P_MIX_t)(s0123 + s4567);
}

static fp_sum8_P_MIX_pup_t sum8_P_MIX_raw_pupdate(fp_P_mix_item_t a0,
                                                  fp_P_mix_item_t a1,
                                                  fp_P_mix_item_t a2,
                                                  fp_P_mix_item_t a3,
                                                  fp_P_mix_item_t a4,
                                                  fp_P_mix_item_t a5,
                                                  fp_P_mix_item_t a6,
                                                  fp_P_mix_item_t a7) {
#pragma HLS INLINE off
MPC_HLS_PIPELINE(1)
  /* LATENCY=1 is critical here, NOT optional. It forces HLS to register
   * the sum8 output before the downstream "+ q_aug" add and P-matrix
   * LUTRAM write. Removing it lets HLS fuse the last sum8 add stage
   * with the downstream LUTRAM data-input logic into one combinational
 * chain ~14 levels deep (9 CARRY8 + 5 LUTs), and WNS collapses to
 * -0.45ns across ~1000 endpoints. Confirmed via 2026-05-15 routed
 * report. Do not drop this pragma. */
#pragma HLS LATENCY min = 1 max = 1
  FP_WPROBE(FP_WP_SUM8_P_MIX_PUP,
            (__int128)a0.to_int64() + (__int128)a1.to_int64() +
                (__int128)a2.to_int64() + (__int128)a3.to_int64() +
                (__int128)a4.to_int64() + (__int128)a5.to_int64() +
                (__int128)a6.to_int64() + (__int128)a7.to_int64());
  FP_WPROBE8(FP_WP_P_MIX_ITEM, a0, a1, a2, a3, a4, a5, a6, a7);
  fp_sum2_P_MIX_t s01 = a0 + a1;
  fp_sum2_P_MIX_t s23 = a2 + a3;
  fp_sum2_P_MIX_t s45 = a4 + a5;
  fp_sum2_P_MIX_t s67 = a6 + a7;
  FP_WPROBE(FP_WP_SUM2_P_MIX, s01.to_int64());
  FP_WPROBE(FP_WP_SUM2_P_MIX, s23.to_int64());
  FP_WPROBE(FP_WP_SUM2_P_MIX, s45.to_int64());
  FP_WPROBE(FP_WP_SUM2_P_MIX, s67.to_int64());
  fp_sum4_P_MIX_t s0123 = s01 + s23;
  fp_sum4_P_MIX_t s4567 = s45 + s67;
  FP_WPROBE(FP_WP_SUM4_P_MIX, s0123.to_int64());
  FP_WPROBE(FP_WP_SUM4_P_MIX, s4567.to_int64());
  return (fp_sum8_P_MIX_pup_t)(s0123 + s4567);
}

static fp_sum6_QP_mul_t sum6_QP_raw(fp_sum6_QP_mul_t a0,
                                    fp_sum6_QP_mul_t a1,
                                    fp_sum6_QP_mul_t a2,
                                    fp_sum6_QP_mul_t a3,
                                    fp_sum6_QP_mul_t a4,
                                    fp_sum6_QP_mul_t a5) {
#pragma HLS INLINE off
MPC_HLS_PIPELINE(1)
  FP_WPROBE(FP_WP_SUM6_QP,
            (__int128)a0.to_int64() + (__int128)a1.to_int64() +
                (__int128)a2.to_int64() + (__int128)a3.to_int64() +
                (__int128)a4.to_int64() + (__int128)a5.to_int64());
  FP_WPROBE6(FP_WP_QP_ITEM, a0, a1, a2, a3, a4, a5);
  fp_sum6_QP_mul_t s01 = a0 + a1;
  fp_sum6_QP_mul_t s23 = a2 + a3;
  fp_sum6_QP_mul_t s45 = a4 + a5;
  fp_sum6_QP_mul_t s0123 = s01 + s23;
  return s0123 + s45;
}

static fp_sum6_MG_QP_t sum6_MG_QP_raw(fp_sum6_MG_QP_t a0,
                                      fp_sum6_MG_QP_t a1,
                                      fp_sum6_MG_QP_t a2,
                                      fp_sum6_MG_QP_t a3,
                                      fp_sum6_MG_QP_t a4,
                                      fp_sum6_MG_QP_t a5) {
#pragma HLS INLINE off
MPC_HLS_PIPELINE(1)
  FP_WPROBE(FP_WP_SUM6_MG_QP,
            (__int128)a0.to_int64() + (__int128)a1.to_int64() +
                (__int128)a2.to_int64() + (__int128)a3.to_int64() +
                (__int128)a4.to_int64() + (__int128)a5.to_int64());
  FP_WPROBE6(FP_WP_MG_QP_ITEM, a0, a1, a2, a3, a4, a5);
  fp_sum6_MG_QP_t s01 = a0 + a1;
  fp_sum6_MG_QP_t s23 = a2 + a3;
  fp_sum6_MG_QP_t s45 = a4 + a5;
  fp_sum6_MG_QP_t s0123 = s01 + s23;
  return s0123 + s45;
}

static fp_sum8_K_QP_t sum8_K_QP_raw(fp_K_qp_item_t a0,
                                    fp_K_qp_item_t a1,
                                    fp_K_qp_item_t a2,
                                    fp_K_qp_item_t a3,
                                    fp_K_qp_item_t a4,
                                    fp_K_qp_item_t a5,
                                    fp_K_qp_item_t a6,
                                    fp_K_qp_item_t a7) {
#pragma HLS INLINE off
MPC_HLS_PIPELINE(1)
  FP_WPROBE(FP_WP_SUM8_K_QP,
            (__int128)a0.to_int64() + (__int128)a1.to_int64() +
                (__int128)a2.to_int64() + (__int128)a3.to_int64() +
                (__int128)a4.to_int64() + (__int128)a5.to_int64() +
                (__int128)a6.to_int64() + (__int128)a7.to_int64());
  FP_WPROBE8(FP_WP_K_QP_ITEM, a0, a1, a2, a3, a4, a5, a6, a7);
  fp_sum2_K_QP_t s01 = a0 + a1;
  fp_sum2_K_QP_t s23 = a2 + a3;
  fp_sum2_K_QP_t s45 = a4 + a5;
  fp_sum2_K_QP_t s67 = a6 + a7;
  FP_WPROBE(FP_WP_SUM2_K_QP, s01.to_int64());
  FP_WPROBE(FP_WP_SUM2_K_QP, s23.to_int64());
  FP_WPROBE(FP_WP_SUM2_K_QP, s45.to_int64());
  FP_WPROBE(FP_WP_SUM2_K_QP, s67.to_int64());
  fp_sum4_K_QP_t s0123 = s01 + s23;
  fp_sum4_K_QP_t s4567 = s45 + s67;
  FP_WPROBE(FP_WP_SUM4_K_QP, s0123.to_int64());
  FP_WPROBE(FP_WP_SUM4_K_QP, s4567.to_int64());
  return (fp_sum8_K_QP_t)(s0123 + s4567);
}

static inline fp_QP_t fp_max_abs_state8(fp_QP_t x0, fp_QP_t x1, fp_QP_t x2,
                                        fp_QP_t x3, fp_QP_t x4, fp_QP_t x5,
                                        fp_QP_t x6, fp_QP_t x7) {
#pragma HLS INLINE
  fp_QP_t m0 = fp_max2(fp_abs(x0), fp_abs(x1));
  fp_QP_t m1 = fp_max2(fp_abs(x2), fp_abs(x3));
  fp_QP_t m2 = fp_max2(fp_abs(x4), fp_abs(x5));
  fp_QP_t m3 = fp_max2(fp_abs(x6), fp_abs(x7));
  fp_QP_t m4 = fp_max2(m0, m1);
  fp_QP_t m5 = fp_max2(m2, m3);
  return fp_max2(m4, m5);
}

static inline fp_QP_t fp_max_abs_ctrl2(fp_QP_t x0, fp_QP_t x1) {
#pragma HLS INLINE
  return fp_max2(fp_abs(x0), fp_abs(x1));
}

static inline fp_QP_t fp_max3_qp(fp_QP_t x0, fp_QP_t x1, fp_QP_t x2) {
#pragma HLS INLINE
  return fp_max2(fp_max2(x0, x1), x2);
}

fp_QP_t fp_normalize_angle(fp_QP_t angle);
fp_QP_t fp_atan_lut(fp_QP_t x);

/* FN family */
fp_FN_t fp_mul_fn(fp_FN_t a, fp_FN_t b);
/* Constant-operand variant: no DSP binding -> HLS shift-add in LUT. Use only
 * when one argument is a compile-time constant. */
fp_FN_t fp_mul_fn_const(fp_FN_t a, fp_FN_t b);

static inline fp_FN_t fp_abs_fn(fp_FN_t a) {
#pragma HLS INLINE
  return (a < 0) ? fp_FN_t(-a) : a;
}

void fp_trig_pair_fused_fn(fp_FN_t angle, fp_FN_t *sin_out, fp_FN_t *cos_out);
fp_FN_t fp_atan_lut_fn(fp_FN_t x);
fp_FN_t fp_recip_fn(fp_FN_t x);

int invert_2x2_qp_hls(fp_QP_raw_t S[2][2], fp_QP_raw_t Si[2][2]);

#endif
