/**
 * @file fp_math_hls.h
 * @brief Fixed-point math helpers for HLS synthesis.
 */

#ifndef FP_MATH_HLS_H
#define FP_MATH_HLS_H

#include "fp_types_hls.hpp"
#include <climits>
#include <cstdint>

/* Automatic bitwidth adjustment for countLeadingZeros normalization.
 * When casting ap_int<N> to unsigned long long for clz, the value occupies
 * lower N bits [N-1:0], leaving (64-N) unused high bits [63:N].
 * This requires adjustment when switching from __builtin_clzll to
 * countLeadingZeros.
 */
#define FP_RAW_ACC_WIDTH (MPC_HLS_RICCATI_WIDTH + MPC_HLS_RAW_ACC_GUARD_BITS)

/* Tunable DSP48E2 multiply pipeline depth.
 * MUST be 4 on XCK26 at 200MHz. Post-implementation analysis identified
 * the critical path as a 4-stage DSP PCOUT→PCIN accumulator cascade inside
 * the COMPUTE_P_SUM_TO 8-term add tree:
 *   (pa0+pa1)+(pa2+pa3)+(pa4+pa5)+(gk0+gk1)
 * = DSP_ALU→DSP_OUTPUT→DSP_ALU→DSP_OUTPUT→DSP_ALU→DSP_OUTPUT→DSP_ALU→OUTPUT
 * With latency=3, HLS does not register the P-output between cascade stages,
 * creating a ~4.4ns combinational chain → 5.464ns post-route (0.464ns over).
 * With latency=4 the extra FF at every DSP P-register breaks the cascade,
 * reducing each hop to a registered single stage and closing timing.
 * The INLINE off structural helpers address inter-function chains only;
 * intra-function DSP accumulator cascades require latency=4. */
#ifndef MPC_HLS_MUL_LATENCY
#define MPC_HLS_MUL_LATENCY 4
#endif

/* Fixed-point base constants */
#define FP_FRAC_BITS (MPC_HLS_RICCATI_WIDTH - MPC_HLS_RICCATI_INT_BITS)
#define FP_IO_CONST(x) ((fp_io_t)(x))
#define FP_QP_CONST(x) ((fp_QP_t)(x))
#define FP_ONE FP_QP_CONST(1.0)
#define FP_TWO FP_QP_CONST(2.0)
#define FP_HALF FP_QP_CONST(0.5)
#define FP_PI FP_QP_CONST(3.14159265358979323846)
#define FP_PI_HALF FP_QP_CONST(1.57079632679489661923)
#define FP_TWO_PI FP_QP_CONST(6.28318530717958647693)

#define INV_FACT_2 FP_QP_CONST(0.5)
#define INV_FACT_3 FP_QP_CONST(0.16666666666666666)
#define INV_FACT_4 FP_QP_CONST(0.041666666666666664)
#define INV_FACT_5 FP_QP_CONST(0.008333333333333333)

/* Width-aware algorithm thresholds expressed as real-value powers of two. */
#define FP_INVERT_2X2_DET_MIN_EXP 12
#define FP_INVERT_2X2_DIAG_FALLBACK_MIN_EXP 8

/* Forward declaration used by fp_div to keep slash out of hot call-sites. */
fp_QP_t fp_recip(fp_QP_t x);

/* Implemented out-of-line in fp_math_hls.cpp to enforce a pipelined DSP
   multiply. Note: When fp_io_t and fp_QP_t are identical (both Q32.16), the
   fp_QP_t overload serves both via implicit conversion. No separate fp_io_t
   overload needed. */

#if (MPC_HLS_RICCATI_WIDTH != MPC_HLS_IO_WIDTH) ||                             \
    (MPC_HLS_RICCATI_INT_BITS != MPC_HLS_IO_INT_BITS)
fp_io_t
fp_mul(fp_io_t a,
       fp_io_t b); /* Q16.16 I/O multiplication (when distinct from Riccati) */
#endif

fp_QP_t fp_mul(fp_QP_t a, fp_QP_t b);

/* fp_sq: squaring a*a with guaranteed DSP binding via INLINE off.
 * INLINE off ensures 'product' survives HLS SSA renaming. */
fp_QP_t fp_sq(fp_QP_t x);

fp_raw_mul_t fp_mul_qp_raw(
    fp_qp_raw_t a,
    fp_qp_raw_t b); /* QP-width raw multiplication with guarded result */
fp_raw_mul_t
fp_mul_qp_acc(fp_qp_raw_t a,
              fp_raw_acc_t b); /* Mixed QP/raw-accumulator multiplication */
fp_raw_mul_t
fp_mul_acc_qp(fp_raw_acc_t a,
              fp_qp_raw_t b); /* Mixed raw-accumulator/QP multiplication */
fp_raw_mul_t
fp_mul_raw_acc(fp_raw_acc_t a,
               fp_raw_acc_t b); /* Guarded raw-accumulator multiplication */

static inline fp_raw_acc_t fp_shift_right_clip_to_acc(fp_raw_mul_t value, int shift) {
#pragma HLS INLINE
  return fp_clip_mul_to_acc(value >> shift);
}

static inline fp_QP_t fp_div(fp_QP_t a, fp_QP_t b) {
  if (a == 0 || b == 0)
    return 0;
  return fp_mul(a, fp_recip(b));
}

static inline fp_QP_t fp_abs(fp_QP_t a) {
  if (a < 0) {
    fp_QP_t neg = -a;
    return neg;
  }
  return a;
}

static inline fp_QP_t fp_clamp(fp_QP_t val, fp_QP_t lo, fp_QP_t hi) {
  if (val < lo)
    return lo;
  if (val > hi)
    return hi;
  return val;
}

static inline fp_qp_raw_t fp_qp_raw_from_neg_pow2(int exp) {
#pragma HLS INLINE
  const int shift = FP_FRAC_BITS - exp;
  if (shift <= 0)
    return (fp_qp_raw_t)1;
  return ((fp_qp_raw_t)1) << shift;
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

int invert_2x2_hls(fp_raw_acc_t S[2][2], fp_raw_acc_t Si[2][2]);

#endif
