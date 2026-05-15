/**
 * @file fp_math_hls.h
 * @brief Fixed-point math helpers for HLS synthesis.
 */

#ifndef FP_MATH_HLS_H
#define FP_MATH_HLS_H

#include "fp_types_hls.hpp"
#include <climits>
#include <cstdint>

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

/*-------------------------------------------------------------------------
 * Specialized shift-right + clip helpers
 *------------------------------------------------------------------------*/

static inline fp_P_raw_t fp_shift_right_clip_to_P(fp_P_QP_mul_t value,
                                                  int shift) {
#pragma HLS INLINE
  fp_P_QP_mul_t shifted = value >> shift;
  const int in_width = MPC_HLS_P_WIDTH + MPC_HLS_P_QP_GUARD;
  if (!fp_signed_fits_width<in_width, MPC_HLS_P_WIDTH>(shifted))
    return shifted[in_width - 1] ? fp_P_raw_min() : fp_P_raw_max();
  return (fp_P_raw_t)shifted;
}

static inline fp_MG_raw_t fp_shift_right_clip_PQ_to_MG(fp_P_QP_mul_t value,
                                                       int shift) {
#pragma HLS INLINE
  fp_P_QP_mul_t shifted = value >> shift;
  const int in_width = MPC_HLS_P_WIDTH + MPC_HLS_P_QP_GUARD;
  if (!fp_signed_fits_width<in_width, MPC_HLS_MG_WIDTH>(shifted))
    return shifted[in_width - 1] ? fp_MG_raw_min() : fp_MG_raw_max();
  return (fp_MG_raw_t)shifted;
}

static inline fp_MG_raw_t fp_shift_right_clip_to_MG(fp_MG_QP_mul_t value,
                                                    int shift) {
#pragma HLS INLINE
  fp_MG_QP_mul_t shifted = value >> shift;
  const int in_width = MPC_HLS_MG_WIDTH + MPC_HLS_MG_QP_GUARD;
  if (!fp_signed_fits_width<in_width, MPC_HLS_MG_WIDTH>(shifted))
    return shifted[in_width - 1] ? fp_MG_raw_min() : fp_MG_raw_max();
  return (fp_MG_raw_t)shifted;
}

static inline fp_S_raw_t fp_shift_right_clip_MGQ_to_S(fp_MG_QP_mul_t value,
                                                      int shift) {
#pragma HLS INLINE
  fp_MG_QP_mul_t shifted = value >> shift;
  const int in_width = MPC_HLS_MG_WIDTH + MPC_HLS_MG_QP_GUARD;
  if (!fp_signed_fits_width<in_width, MPC_HLS_S_WIDTH>(shifted))
    return shifted[in_width - 1] ? fp_S_raw_min() : fp_S_raw_max();
  return (fp_S_raw_t)shifted;
}

static inline fp_K_raw_t fp_shift_right_clip_to_K(fp_Si_MG_mul_t value,
                                                  int shift) {
#pragma HLS INLINE
  fp_Si_MG_mul_t shifted = value >> shift;
  const int in_width = MPC_HLS_SI_WIDTH + MPC_HLS_SI_MG_GUARD;
  if (!fp_signed_fits_width<in_width, MPC_HLS_K_WIDTH>(shifted))
    return shifted[in_width - 1] ? fp_K_raw_min() : fp_K_raw_max();
  return (fp_K_raw_t)shifted;
}

static inline fp_P_raw_t fp_shift_right_clip_MGK_to_P(fp_MG_K_mul_t value,
                                                      int shift) {
#pragma HLS INLINE
  fp_MG_K_mul_t shifted = value >> shift;
  const int in_width = MPC_HLS_MG_WIDTH + MPC_HLS_MG_K_GUARD;
  if (!fp_signed_fits_width<in_width, MPC_HLS_P_WIDTH>(shifted))
    return shifted[in_width - 1] ? fp_P_raw_min() : fp_P_raw_max();
  return (fp_P_raw_t)shifted;
}

static inline fp_QP_raw_t fp_shift_right_clip_KQ_to_qp(fp_K_QP_mul_t value,
                                                       int shift) {
#pragma HLS INLINE
  fp_K_QP_mul_t shifted = value >> shift;
  const int in_width = MPC_HLS_K_WIDTH + MPC_HLS_K_QP_GUARD;
  if (!fp_signed_fits_width<in_width, MPC_HLS_QP_WIDTH>(shifted))
    return shifted[in_width - 1] ? (fp_QP_raw_t)fp_qp_raw_min_acc()
                                 : (fp_QP_raw_t)fp_qp_raw_max_acc();
  return (fp_QP_raw_t)shifted;
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

int invert_2x2_hls(fp_raw_acc_t S[2][2], fp_raw_acc_t Si[2][2]);

#endif
