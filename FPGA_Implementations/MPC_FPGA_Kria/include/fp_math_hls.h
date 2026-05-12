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
#define MPC_HLS_MUL_LATENCY 4
#endif

#define FP_FRAC_BITS (MPC_HLS_QP_FRAC_BITS)

#define FP_IO_CONST(x) ((fp_io_t)(x))
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
fp_acc_mul_t fp_mul_acc_acc(fp_raw_acc_t a, fp_raw_acc_t b);

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

fp_QP_t fp_normalize_angle(fp_QP_t angle);
fp_QP_t fp_sin(fp_QP_t angle);
fp_QP_t fp_cos(fp_QP_t angle);
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
fp_FN_t fp_atan_lut_fn(fp_FN_t x);
fp_FN_t fp_recip_fn(fp_FN_t x);

#define FP_FN_CONST(x) ((fp_FN_t)(x))
#define FP_FN_FRAC_BITS (MPC_HLS_FN_FRAC_BITS)
#define FP_FN_ONE FP_FN_CONST(1.0)
#define FP_FN_TWO FP_FN_CONST(2.0)
#define FP_FN_HALF FP_FN_CONST(0.5)
#define FP_FN_PI FP_FN_CONST(3.14159265358979323846)
#define FP_FN_PI_HALF FP_FN_CONST(1.57079632679489661923)
#define FP_FN_ZERO FP_FN_CONST(0.0)
#define FP_FN_TWO_PI FP_FN_CONST(6.28318530717958647693)

int invert_2x2_hls(fp_raw_acc_t S[2][2], fp_raw_acc_t Si[2][2]);

#endif