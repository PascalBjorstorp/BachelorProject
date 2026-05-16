/**
 * @file fp_math_hls.h
 * @brief Fixed-point math helpers for HLS synthesis.
 */

#ifndef FP_MATH_HLS_H
#define FP_MATH_HLS_H

#include "fp_types_hls.hpp"
#include <climits>
#include <cstdint>

/* DSP multiply latency.
 *
 * FN_MUL_LATENCY was tried at 2 to shave one cycle per vehicle_model
 * multiply, but the resulting LUT bloat (~6500 LUT in tire alone, from
 * extra sign-extension/mux glue around each shallower-pipelined DSP)
 * pushed device LUT utilization from ~84% to 88% and spread the
 * placement of riccati_pass enough that the bucket A fanout (shared
 * sum6_P_QP_raw multiplier output from LOOP_522 → LOOP_580 sum6)
 * stopped meeting timing. Confirmed via 2026-05-15 routed report
 * (WNS -0.444 ns, 738 endpoints, all in LOOP_522/LOOP_580/PA matrix).
 * Keep FN at 3 unless device LUT pressure has headroom. */
#ifndef MPC_HLS_MUL_LATENCY
#define MPC_HLS_MUL_LATENCY 3
#endif

#define FP_FRAC_BITS (MPC_HLS_QP_FRAC_BITS)

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
#define FP_TRIG_LUT_SCALE FP_QP_CONST(162.9746617261)
#define FP_FN_TRIG_LUT_SCALE FP_FN_CONST(162.9746617261)

fp_QP_t fp_recip(fp_QP_t x);

/* Canonical multiply helpers */
fp_QP_t fp_mul_site(fp_QP_t a, fp_QP_t b, int site_id);
fp_QP_t fp_mul(fp_QP_t a, fp_QP_t b);
fp_QP_t fp_sq(fp_QP_t x);

fp_QP_mul_t fp_mul_QP_raw(fp_QP_raw_t a, fp_QP_raw_t b);
fp_acc_QP_mul_t fp_mul_QP_acc(fp_QP_raw_t a, fp_raw_acc_t b);
fp_acc_QP_mul_t fp_mul_acc_QP(fp_raw_acc_t a, fp_QP_raw_t b);

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

static inline fp_QP_t fp_div(fp_QP_t a, fp_QP_t b) {
#pragma HLS INLINE
  if (a == 0 || b == 0)
    return 0;
  return fp_mul(a, fp_recip(b));
}

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

static inline fp_raw_acc_t fp_raw_acc_from_neg_pow2(int exp) {
#pragma HLS INLINE
  const int shift = FP_FRAC_BITS - exp;
  if (shift <= 0)
    return (fp_raw_acc_t)1;
  return ((fp_raw_acc_t)1) << shift;
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
#pragma HLS PIPELINE II = 1
  fp_sum6_P_QP_t s01 = a0 + a1;
  fp_sum6_P_QP_t s23 = a2 + a3;
  fp_sum6_P_QP_t s45 = a4 + a5;
  fp_sum6_P_QP_t s0123 = s01 + s23;
  return s0123 + s45;
}

static fp_sum8_P_MIX_t sum8_P_MIX_raw(fp_sum8_P_MIX_t a0,
                                      fp_sum8_P_MIX_t a1,
                                      fp_sum8_P_MIX_t a2,
                                      fp_sum8_P_MIX_t a3,
                                      fp_sum8_P_MIX_t a4,
                                      fp_sum8_P_MIX_t a5,
                                      fp_sum8_P_MIX_t a6,
                                      fp_sum8_P_MIX_t a7) {
#pragma HLS INLINE off
#pragma HLS PIPELINE II = 1
  fp_sum8_P_MIX_t s01 = a0 + a1;
  fp_sum8_P_MIX_t s23 = a2 + a3;
  fp_sum8_P_MIX_t s45 = a4 + a5;
  fp_sum8_P_MIX_t s67 = a6 + a7;
  fp_sum8_P_MIX_t s0123 = s01 + s23;
  fp_sum8_P_MIX_t s4567 = s45 + s67;
  return s0123 + s4567;
}

static fp_sum8_P_MIX_t sum8_P_MIX_raw_pupdate(fp_sum8_P_MIX_t a0,
                                              fp_sum8_P_MIX_t a1,
                                              fp_sum8_P_MIX_t a2,
                                              fp_sum8_P_MIX_t a3,
                                              fp_sum8_P_MIX_t a4,
                                              fp_sum8_P_MIX_t a5,
                                              fp_sum8_P_MIX_t a6,
                                              fp_sum8_P_MIX_t a7) {
#pragma HLS INLINE off
#pragma HLS PIPELINE II = 1
  /* LATENCY=1 is critical here, NOT optional. It forces HLS to register
   * the sum8 output before the downstream "+ q_aug" add and P-matrix
   * LUTRAM write. Removing it lets HLS fuse the last sum8 add stage
   * with the downstream LUTRAM data-input logic into one combinational
   * chain ~14 levels deep (9 CARRY8 + 5 LUTs), and WNS collapses to
   * -0.45ns across ~1000 endpoints. Confirmed via 2026-05-15 routed
   * report. Do not drop this pragma. */
#pragma HLS LATENCY min = 1 max = 1
  fp_sum8_P_MIX_t s01 = a0 + a1;
  fp_sum8_P_MIX_t s23 = a2 + a3;
  fp_sum8_P_MIX_t s45 = a4 + a5;
  fp_sum8_P_MIX_t s67 = a6 + a7;
  fp_sum8_P_MIX_t s0123 = s01 + s23;
  fp_sum8_P_MIX_t s4567 = s45 + s67;
  return s0123 + s4567;
}

static fp_sum6_QP_mul_t sum6_QP_raw(fp_sum6_QP_mul_t a0,
                                    fp_sum6_QP_mul_t a1,
                                    fp_sum6_QP_mul_t a2,
                                    fp_sum6_QP_mul_t a3,
                                    fp_sum6_QP_mul_t a4,
                                    fp_sum6_QP_mul_t a5) {
#pragma HLS INLINE off
#pragma HLS PIPELINE II = 1
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
#pragma HLS PIPELINE II = 1
  fp_sum6_MG_QP_t s01 = a0 + a1;
  fp_sum6_MG_QP_t s23 = a2 + a3;
  fp_sum6_MG_QP_t s45 = a4 + a5;
  fp_sum6_MG_QP_t s0123 = s01 + s23;
  return s0123 + s45;
}

static fp_sum8_K_QP_t sum8_K_QP_raw(fp_sum8_K_QP_t a0,
                                    fp_sum8_K_QP_t a1,
                                    fp_sum8_K_QP_t a2,
                                    fp_sum8_K_QP_t a3,
                                    fp_sum8_K_QP_t a4,
                                    fp_sum8_K_QP_t a5,
                                    fp_sum8_K_QP_t a6,
                                    fp_sum8_K_QP_t a7) {
#pragma HLS INLINE off
#pragma HLS PIPELINE II = 1
  fp_sum8_K_QP_t s01 = a0 + a1;
  fp_sum8_K_QP_t s23 = a2 + a3;
  fp_sum8_K_QP_t s45 = a4 + a5;
  fp_sum8_K_QP_t s67 = a6 + a7;
  fp_sum8_K_QP_t s0123 = s01 + s23;
  fp_sum8_K_QP_t s4567 = s45 + s67;
  return s0123 + s4567;
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

/*-------------------------------------------------------------------------
 * Specialized shift-right + cast helpers
 *------------------------------------------------------------------------*/

static inline fp_P_raw_t fp_shift_right_cast_to_P(fp_P_QP_mul_t value,
                                                  int shift) {
#pragma HLS INLINE
  return (fp_P_raw_t)(value >> shift);
}

static inline fp_MG_raw_t fp_shift_right_cast_PQ_to_MG(fp_P_QP_mul_t value,
                                                       int shift) {
#pragma HLS INLINE
  return (fp_MG_raw_t)(value >> shift);
}

static inline fp_MG_raw_t fp_shift_right_cast_to_MG(fp_MG_QP_mul_t value,
                                                    int shift) {
#pragma HLS INLINE
  return (fp_MG_raw_t)(value >> shift);
}

static inline fp_S_raw_t fp_shift_right_cast_MGQ_to_S(fp_MG_QP_mul_t value,
                                                      int shift) {
#pragma HLS INLINE
  return (fp_S_raw_t)(value >> shift);
}

static inline fp_K_raw_t fp_shift_right_cast_to_K(fp_Si_MG_mul_t value,
                                                  int shift) {
#pragma HLS INLINE
  return (fp_K_raw_t)(value >> shift);
}

static inline fp_P_raw_t fp_shift_right_cast_MGK_to_P(fp_MG_K_mul_t value,
                                                      int shift) {
#pragma HLS INLINE
  return (fp_P_raw_t)(value >> shift);
}

static inline fp_QP_raw_t fp_shift_right_cast_KQ_to_qp(fp_K_QP_mul_t value,
                                                       int shift) {
#pragma HLS INLINE
  return (fp_QP_raw_t)(value >> shift);
}

fp_QP_t fp_normalize_angle(fp_QP_t angle);
fp_QP_t fp_sin(fp_QP_t angle);
fp_QP_t fp_cos(fp_QP_t angle);
void fp_trig_pair_fused(fp_QP_t angle, fp_QP_t *sin_out, fp_QP_t *cos_out);
fp_QP_t fp_atan_lut(fp_QP_t x);

/* FN family */
fp_FN_t fp_mul_fn(fp_FN_t a, fp_FN_t b);
fp_fn_accum_t fp_mul_fn_raw(fp_FN_t a, fp_FN_t b);

static inline fp_FN_t fp_abs_fn(fp_FN_t a) {
#pragma HLS INLINE
  return (a < 0) ? fp_FN_t(-a) : a;
}

fp_FN_t fp_sin_fn(fp_FN_t angle);
fp_FN_t fp_cos_fn(fp_FN_t angle);
void fp_trig_pair_fused_fn(fp_FN_t angle, fp_FN_t *sin_out, fp_FN_t *cos_out);
fp_FN_t fp_atan_lut_fn(fp_FN_t x);
fp_FN_t fp_recip_fn(fp_FN_t x);

int invert_2x2_qp_hls(fp_QP_raw_t S[2][2], fp_QP_raw_t Si[2][2]);

#endif
