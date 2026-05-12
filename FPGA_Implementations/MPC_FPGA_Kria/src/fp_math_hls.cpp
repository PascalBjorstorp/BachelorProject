/**
 * @file fp_math_hls.cpp
 * @brief Q16.16 fixed-point math kernels for HLS synthesis.
 * @details Implements non-inline trigonometric and reciprocal kernels used by
 *          the FPGA MPC pipeline. Arithmetic is performed in fixed-point with
 *          deterministic iteration counts suited for synthesis scheduling.
 * @dependencies fp_math_hls.h
 */

#include "../include/fp_math_hls.h"
#include "../include/fp_trig_lut_1024.h"
#include <climits>
#include <cstdint>

/*----------------------------------------------------------------------------
 * Canonical exact-width raw multipliers
 *----------------------------------------------------------------------------*/

fp_QP_mul_t fp_mul_QP_raw(fp_QP_raw_t a, fp_QP_raw_t b) {
#pragma HLS INLINE
  fp_QP_mul_t product = (fp_QP_mul_t)a * (fp_QP_mul_t)b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
  return product;
}

fp_acc_QP_mul_t fp_mul_QP_acc(fp_QP_raw_t a, fp_raw_acc_t b) {
#pragma HLS INLINE
  fp_acc_QP_mul_t product = (fp_acc_QP_mul_t)a * (fp_acc_QP_mul_t)b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
  return product;
}

fp_acc_QP_mul_t fp_mul_acc_QP(fp_raw_acc_t a, fp_QP_raw_t b) {
#pragma HLS INLINE
  fp_acc_QP_mul_t product = (fp_acc_QP_mul_t)a * (fp_acc_QP_mul_t)b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
  return product;
}

fp_acc_mul_t fp_mul_acc_acc(fp_raw_acc_t a, fp_raw_acc_t b) {
#pragma HLS INLINE
  fp_acc_mul_t product = (fp_acc_mul_t)a * (fp_acc_mul_t)b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
  return product;
}

/*----------------------------------------------------------------------------
 * Base-family multiply helpers
 *----------------------------------------------------------------------------*/

fp_QP_t fp_mul(fp_QP_t a, fp_QP_t b) {
#pragma HLS INLINE
  fp_QP_mul_t product = fp_mul_QP_raw(fp_qp_raw_from_QP(a),
                                          fp_qp_raw_from_QP(b));
  fp_QP_raw_t product_q = fp_shift_right_clip_to_qp(product, FP_FRAC_BITS);
  return fp_QP_from_qp_raw(product_q);
}

/* fp_sq: dedicated squaring helper — INLINE off required.
 * Keeping fp_sq out-of-line preserves its own multiply context and DSP binding.
 */
fp_QP_t fp_sq(fp_QP_t x) {
#pragma HLS INLINE off
  fp_QP_mul_t product = fp_mul_QP_raw(fp_qp_raw_from_QP(x),
                                          fp_qp_raw_from_QP(x));
  fp_QP_raw_t product_q = fp_shift_right_clip_to_qp(product, FP_FRAC_BITS);
  return fp_QP_from_qp_raw(product_q);
}

/*--------------------------------------------------------------------------
 * Invert 2x2 Matrix
 *--------------------------------------------------------------------------*/

int invert_2x2_hls(fp_raw_acc_t S[2][2], fp_raw_acc_t Si[2][2]) {
#pragma HLS INLINE off
#pragma HLS ALLOCATION function instances = fp_recip limit = 1

  /* S entries are solver-acc values at base scale (already >> FP_FRAC_BITS). */
  fp_QP_t s00 = fp_QP_from_qp_raw((fp_QP_raw_t)fp_clip_raw_to_qp(S[0][0]));
  fp_QP_t s01 = fp_QP_from_qp_raw((fp_QP_raw_t)fp_clip_raw_to_qp(S[0][1]));
  fp_QP_t s10 = fp_QP_from_qp_raw((fp_QP_raw_t)fp_clip_raw_to_qp(S[1][0]));
  fp_QP_t s11 = fp_QP_from_qp_raw((fp_QP_raw_t)fp_clip_raw_to_qp(S[1][1]));

  fp_QP_t det = fp_mul(s00, s11) - fp_mul(s01, s10);

  fp_QP_t det_eps = fp_qp_from_neg_pow2(FP_INVERT_2X2_DET_MIN_EXP);
  if (det > -det_eps && det < det_eps) {
    return -1;
  }

  fp_QP_t inv_det = fp_recip(det);

  Si[0][0] = fp_raw_acc_from_qp(fp_mul(s11, inv_det));
  Si[0][1] = fp_raw_acc_from_qp(fp_mul(-s01, inv_det));
  Si[1][0] = fp_raw_acc_from_qp(fp_mul(-s10, inv_det));
  Si[1][1] = fp_raw_acc_from_qp(fp_mul(s00, inv_det));

  return 0;
}

/*===========================================================================
 * Normalize Angle to [-pi, pi]
 *===========================================================================*/

fp_QP_t fp_normalize_angle(fp_QP_t angle) {
#pragma HLS INLINE
  /* Bounded fp-domain wrap loop avoids int casting for phase reduction. */
  for (int i = 0; i < 2; i++) {
#pragma HLS UNROLL
    if (angle > FP_PI)
      angle = angle - FP_TWO_PI;
    if (angle < -FP_PI)
      angle = angle + FP_TWO_PI;
  }

  return angle;
}

/*===========================================================================
 * Reciprocal: 1/x (LUT-seeded Newton-Raphson, 1 iteration)
 *
 * 9-bit LUT covers x_norm in [0.5, 1.0). Entry i = floor(32768*1024/(512+i)),
 * representing 1/(0.5 + i/1024) in Q32.17 raw form (FP_FRAC_BITS=15).
 * Max LUT error: 0.195%. After 1 NR iteration: < 0.000381% < Q32.17 resolution
 * (0.00305%), so 1 iteration is sufficient vs 3 with est=1.5 seed.
 * '>=one_raw' (not '>') ensures x_norm is strictly < 1.0, keeping index in
 * [0,511]: x=1.0 maps to x_norm=0.5 via an extra shift, then denormalises
 * correctly.
 *===========================================================================*/
fp_QP_t fp_recip(fp_QP_t x) {
#pragma HLS INLINE off
#pragma HLS PIPELINE II = 21

  if (x == 0)
    return 0;

  bool neg = (x < 0);
  fp_QP_t abs_x = fp_abs(x);

  fp_QP_raw_t abs_raw_signed = fp_qp_raw_from_QP(abs_x);
  ap_uint<MPC_HLS_QP_WIDTH> abs_raw =
      (ap_uint<MPC_HLS_QP_WIDTH>)abs_raw_signed;

  const int one_bit = FP_FRAC_BITS;
  const int half_bit = FP_FRAC_BITS - 1;
  int shift = 0;

  int clz = (int)abs_raw.countLeadingZeros();
  int msb = (MPC_HLS_QP_WIDTH - 1) - clz;

  if (msb > one_bit) {
    shift = msb - one_bit;
  } else if (msb < half_bit) {
    shift = -(half_bit - msb);
  }

  if (shift >= 0) {
    ap_uint<MPC_HLS_QP_WIDTH> right_norm = abs_raw;
    for (int s = 1; s < MPC_HLS_QP_WIDTH - 1; s++) {
#pragma HLS UNROLL
      if (shift == s)
        right_norm = abs_raw >> s;
    }

    ap_uint<MPC_HLS_QP_WIDTH> one_raw =
        ((ap_uint<MPC_HLS_QP_WIDTH>)1) << FP_FRAC_BITS;

    if (right_norm >= one_raw && shift < (MPC_HLS_QP_WIDTH - 2)) {
      shift++;
    }
  }

  if (shift > (MPC_HLS_QP_WIDTH - 2))
    shift = (MPC_HLS_QP_WIDTH - 2);
  if (shift < -(MPC_HLS_QP_WIDTH - 2))
    shift = -(MPC_HLS_QP_WIDTH - 2);

  fp_QP_t x_norm = abs_x;
  for (int s = 1; s < MPC_HLS_QP_WIDTH - 1; s++) {
#pragma HLS UNROLL
    if (shift == s)
      x_norm = abs_x >> s;
    if (shift == -s)
      x_norm = abs_x << s;
  }

  /* LUT index for x_norm in [0.5, 1.0):
  *
  * raw(0.5) = 1 << (FP_FRAC_BITS - 1)
  * raw spacing across [0.5,1.0) for 1024 bins = 1 << (FP_FRAC_BITS - 11)
  *
  * Therefore the 10-bit bin index is the slice:
  *   [FP_FRAC_BITS - 2 : FP_FRAC_BITS - 11]
  *
  * recip_lut[] must contain 1024 entries regenerated for the active
  * FP_FRAC_BITS.
  */
#pragma HLS BIND_STORAGE variable = recip_lut type = rom_1p impl = bram
  ap_uint<MPC_HLS_QP_WIDTH> norm_raw_u = (ap_uint<MPC_HLS_QP_WIDTH>)fp_qp_raw_from_QP(x_norm);
  const int lut_hi = FP_FRAC_BITS - 2;
  const int lut_lo = FP_FRAC_BITS - 11;
  int lut_idx = (int)(norm_raw_u.range(lut_hi, lut_lo));
  if (lut_idx > 1023)
    lut_idx = 1023;

  fp_QP_t est = fp_QP_from_qp_raw((fp_QP_raw_t)recip_lut[lut_idx]);

  est = fp_mul(est, (FP_TWO - fp_mul(x_norm, est)));

  fp_QP_t est_denorm = est;
  for (int s = 1; s < MPC_HLS_QP_WIDTH - 1; s++) {
#pragma HLS UNROLL
    if (shift == s)
      est_denorm = est >> s;
    if (shift == -s)
      est_denorm = est << s;
  }

  if (neg)
    est_denorm = -est_denorm;
  return est_denorm;
}

/*===========================================================================
 * Sine/Cosine: 1024-segment LUT with linear interpolation
 *===========================================================================*/

fp_QP_t fp_sin(fp_QP_t angle) {
#pragma HLS INLINE off
#pragma HLS BIND_STORAGE variable = sin_lut type = rom_2p impl = bram
  fp_QP_t angle_u = fp_normalize_angle(angle);
  if (angle_u < 0)
    angle_u += FP_TWO_PI;

  fp_QP_t lut_pos = fp_mul(angle_u, FP_TRIG_LUT_SCALE);
  int idx = (int)lut_pos;
  if (idx >= FP_TRIG_LUT_SIZE)
    idx = 0;
  if (idx < 0)
    idx = 0;

  const int idx_next = (idx + 1) & FP_TRIG_LUT_MASK;
  fp_QP_raw_t idx_q_raw = ((fp_QP_raw_t)idx) << FP_FRAC_BITS;
  fp_QP_t frac = lut_pos - fp_QP_from_qp_raw(idx_q_raw);
  fp_QP_t v0 = sin_lut[idx];
  fp_QP_t v1 = sin_lut[idx_next];
  return v0 + fp_mul(frac, (v1 - v0));
}

fp_QP_t fp_cos(fp_QP_t angle) {
#pragma HLS INLINE off
#pragma HLS BIND_STORAGE variable = cos_lut type = rom_2p impl = bram
  fp_QP_t angle_u = fp_normalize_angle(angle);
  if (angle_u < 0)
    angle_u += FP_TWO_PI;

  fp_QP_t lut_pos = fp_mul(angle_u, FP_TRIG_LUT_SCALE);
  int idx = (int)lut_pos;
  if (idx >= FP_TRIG_LUT_SIZE)
    idx = 0;
  if (idx < 0)
    idx = 0;

  const int idx_next = (idx + 1) & FP_TRIG_LUT_MASK;
  fp_QP_raw_t idx_q_raw = ((fp_QP_raw_t)idx) << FP_FRAC_BITS;
  fp_QP_t frac = lut_pos - fp_QP_from_qp_raw(idx_q_raw);
  fp_QP_t v0 = cos_lut[idx];
  fp_QP_t v1 = cos_lut[idx_next];
  return v0 + fp_mul(frac, (v1 - v0));
}

/*===========================================================================
 * Unified atan: 1024-segment LUT with linear interpolation, full range.
 *
 * For |x| ≤ 1: LUT lookup of atan(|x|) directly.
 * For |x| > 1: atan(x) = π/2 − atan(1/x) via fp_recip + LUT.
 * Max error: < 0.00003% (LUT spacing π/4 / 1024 ≈ 7.7e-4 rad;
 *===========================================================================*/
fp_QP_t fp_atan_lut(fp_QP_t x) {
#pragma HLS INLINE off
#pragma HLS ALLOCATION function instances = fp_recip limit = 1
#pragma HLS BIND_STORAGE variable = atan_lut type = rom_2p impl = bram
  bool neg = (x < 0);
  fp_QP_t abs_x = fp_abs(x);
  bool over_one = (abs_x > FP_ONE);
  fp_QP_t y = over_one ? fp_recip(abs_x) : abs_x;
  fp_QP_t lut_pos = y << 10;
  int idx = (int)lut_pos;
  if (idx < 0)
    idx = 0;
  if (idx > 1023)
    idx = 1023;
  fp_QP_raw_t idx_q_raw = ((fp_QP_raw_t)idx) << FP_FRAC_BITS;
  fp_QP_t frac = lut_pos - fp_QP_from_qp_raw(idx_q_raw);
  fp_QP_t v0 = atan_lut[idx];
  fp_QP_t v1 = atan_lut[idx + 1];
  fp_QP_t atan_y = v0 + fp_mul(frac, v1 - v0);
  fp_QP_t result = over_one ? fp_QP_t(FP_PI_HALF - atan_y) : atan_y;
  return neg ? fp_QP_t(-result) : result;
}

/*===========================================================================
 * FN family (ap_fixed<26,7>) math implementations
 *
 * PRECISION NOTES:
 *   fp_FN_t = ap_fixed<26,9>: 9 integer bits (incl. sign), 17 fractional bits.
 *   FP_FN_FRAC_BITS = 17
 *===========================================================================*/

#include "../include/fp_trig_lut_fn_1024.h" /* FN-precision LUTs */

/* FN multiplication with DSP binding. FP_FN_FRAC_BITS = 17. */
fp_FN_t fp_mul_fn(fp_FN_t a, fp_FN_t b) {
#pragma HLS INLINE
  fp_fn_accum_t product = (fp_fn_accum_t)fp_fn_raw_from_FN(a) *
                            (fp_fn_accum_t)fp_fn_raw_from_FN(b);
#pragma HLS BIND_OP variable = product op = mul impl = dsp latency = MPC_HLS_MUL_LATENCY
  fp_fn_raw_t product_q = fp_shift_right_clip_to_fn(product, FP_FN_FRAC_BITS);
  return fp_FN_from_fn_raw(product_q);
}

/* FN raw (unshifted) multiplication — 48-bit product before >>FP_FN_FRAC_BITS.*/
fp_fn_accum_t fp_mul_fn_raw(fp_FN_t a, fp_FN_t b) {
#pragma HLS INLINE
  fp_fn_accum_t product = (fp_fn_accum_t)fp_fn_raw_from_FN(a) * (fp_fn_accum_t)fp_fn_raw_from_FN(b);
#pragma HLS BIND_OP variable = product op = mul impl = dsp latency = MPC_HLS_MUL_LATENCY
  return product;
}

/* FN sine — quadrant-reduction approach.
 * All vehicle model angles are in (-pi, pi). We reduce to [0, pi/2] via:
 *   sin(-x) = -sin(x),  sin(pi-x) = sin(x)
 * Then index into the 1024-entry [0, 2pi) LUT using integer arithmetic only.
 * This avoids the 2pi-wrap overflow that occurs when large wrapped angles
 * (near 2pi) are indexed using scaled integer multiply. */
fp_FN_t fp_sin_fn(fp_FN_t angle) {
#pragma HLS INLINE off
#pragma HLS BIND_STORAGE variable = sin_lut_fn type = rom_2p impl = bram
  /* Step 1: extract sign and work with |angle|. */
  bool neg = (angle < FP_FN_ZERO);
  fp_FN_t abs_angle = neg ? fp_FN_t(-angle) : angle;

  /* Step 2: reduce from [0, 2pi) to [0, pi] via period symmetry.
   * For angle > pi: sin(angle) = -sin(angle - pi), cos(angle) = -cos(angle-pi).
   */
  bool over_pi = (abs_angle > FP_FN_PI);
  fp_FN_t a = over_pi ? fp_FN_t(abs_angle - FP_FN_PI) : abs_angle;
  /* sin in [0, pi]: always >= 0, but negate result if over_pi. */
  bool negate = neg ^ over_pi;

  /* Step 3: compute LUT position using FN arithmetic, avoiding int64. */
  fp_fn_accum_t lut_scaled_raw = fp_mul_fn_raw(a, FP_FN_TRIG_LUT_SCALE);
  int32_t lut_pos_raw = (int32_t)(lut_scaled_raw >> FP_FN_FRAC_BITS);
  int idx = (int)(lut_pos_raw >> FP_FN_FRAC_BITS);
  if (idx >= FP_TRIG_LUT_SIZE)
    idx = FP_TRIG_LUT_SIZE - 1;
  if (idx < 0)
    idx = 0;

  const int idx_next = (idx + 1) & FP_TRIG_LUT_MASK;
  int32_t idx_raw = ((int32_t)idx) << FP_FN_FRAC_BITS;
  fp_FN_t frac = fp_FN_from_fn_raw((fp_fn_raw_t)(lut_pos_raw - idx_raw));
  fp_FN_t v0 = sin_lut_fn[idx];
  fp_FN_t v1 = sin_lut_fn[idx_next];
  fp_FN_t result = v0 + fp_mul_fn(frac, (v1 - v0));
  return negate ? fp_FN_t(-result) : result;
}

/* FN cosine — quadrant-reduction, same strategy as fp_sin_fn.
 * Uses: cos(-x) = cos(x),  cos(pi-x) = -cos(x).
 * Delegates to LUT via cos_lut_fn (which is just sin shifted by pi/2). */
fp_FN_t fp_cos_fn(fp_FN_t angle) {
#pragma HLS INLINE off
#pragma HLS BIND_STORAGE variable = cos_lut_fn type = rom_2p impl = bram
  /* cos(-x) = cos(x), work with |angle|. */
  fp_FN_t abs_angle = (angle < FP_FN_ZERO) ? fp_FN_t(-angle) : angle;

  /* cos(pi - a) = -cos(a): for a > pi, result negates. */
  bool over_pi = (abs_angle > FP_FN_PI);
  fp_FN_t a = over_pi ? fp_FN_t(abs_angle - FP_FN_PI) : abs_angle;

  fp_fn_accum_t lut_scaled_raw = fp_mul_fn_raw(a, FP_FN_TRIG_LUT_SCALE);
  int32_t lut_pos_raw = (int32_t)(lut_scaled_raw >> FP_FN_FRAC_BITS);
  int idx = (int)(lut_pos_raw >> FP_FN_FRAC_BITS);
  if (idx >= FP_TRIG_LUT_SIZE)
    idx = FP_TRIG_LUT_SIZE - 1;
  if (idx < 0)
    idx = 0;

  const int idx_next = (idx + 1) & FP_FN_TRIG_LUT_MASK;
  int32_t idx_raw = ((int32_t)idx) << FP_FN_FRAC_BITS;
  fp_FN_t frac = fp_FN_from_fn_raw((fp_fn_raw_t)(lut_pos_raw - idx_raw));
  fp_FN_t v0 = cos_lut_fn[idx];
  fp_FN_t v1 = cos_lut_fn[idx_next];
  fp_FN_t result = v0 + fp_mul_fn(frac, (v1 - v0));
  return over_pi ? fp_FN_t(-result) : result;
}

/* FN arctangent — purely FN-domain integer indexing.
 * y in [0,1): y_raw < 2^17 so y_raw>>7 in [0,1023] exactly.
 * frac_raw = (y_raw<<10) - (idx<<17) stays in int32 (<2^17). */
fp_FN_t fp_atan_lut_fn(fp_FN_t x) {
#pragma HLS INLINE off
#pragma HLS ALLOCATION function instances = fp_recip_fn limit = 1
#pragma HLS BIND_STORAGE variable = atan_lut_fn type = rom_2p impl = bram
  bool neg = (x < FP_FN_ZERO);
  fp_FN_t abs_x = neg ? fp_FN_t(-x) : x;
  bool over_one = (abs_x > FP_FN_ONE);
  fp_FN_t y = over_one ? fp_recip_fn(abs_x) : abs_x;

  /* y in [0,1): y_raw in [0, 2^17). idx = floor(y*1024) = y_raw >> 7. */
  int32_t y_raw = (int32_t)fp_fn_raw_from_FN(y);
  int idx = y_raw >> 7;
  if (idx < 0)
    idx = 0;
  if (idx > 1023)
    idx = 1023;

  /* frac_raw in FN units: (y*1024 - idx) * 2^17 = (y_raw<<10) - (idx<<17). */
  int32_t frac_raw_i = (y_raw << 10) - (idx << 17);
  fp_FN_t frac = fp_FN_from_fn_raw((fp_fn_raw_t)frac_raw_i);
  fp_FN_t v0 = atan_lut_fn[idx];
  fp_FN_t v1 = atan_lut_fn[idx + 1];
  fp_FN_t atan_y = v0 + fp_mul_fn(frac, v1 - v0);
  fp_FN_t result = over_one ? fp_FN_t(FP_FN_PI_HALF - atan_y) : atan_y;
  return neg ? fp_FN_t(-result) : result;
}

/* FN reciprocal — exact QP structure with FN types */
fp_FN_t fp_recip_fn(fp_FN_t x) {
#pragma HLS INLINE off
#pragma HLS PIPELINE II = 21
#pragma HLS BIND_STORAGE variable = recip_lut_fn type = rom_1p impl = bram
  if (x == 0)
    return 0;

  bool neg = (x < 0);
  fp_FN_t abs_x = neg ? fp_FN_t(-x) : x;

  /* Width-aware bit-domain normalization without variable-trip loops. */
  fp_fn_raw_t abs_raw_signed = fp_fn_raw_from_FN(abs_x);
  ap_uint<MPC_HLS_FN_WIDTH> abs_raw = (ap_uint<MPC_HLS_FN_WIDTH>)abs_raw_signed;

  const int one_bit = MPC_HLS_FN_WIDTH - MPC_HLS_FN_INT_BITS; /* 17 */
  const int half_bit = one_bit - 1;                           /* 16 */
  int shift = 0;

  int clz = (int)abs_raw.countLeadingZeros();
  int msb = (MPC_HLS_FN_WIDTH - 1) - clz;

  if (msb > one_bit) {
    shift = msb - one_bit;
  } else if (msb < half_bit) {
    shift = -(half_bit - msb);
  }

  if (shift >= 0) {
    ap_uint<MPC_HLS_FN_WIDTH> right_norm = abs_raw;
    for (int s = 1; s < MPC_HLS_FN_WIDTH - 1; s++) {
#pragma HLS UNROLL
      if (shift == s)
        right_norm = abs_raw >> s;
    }
    ap_uint<MPC_HLS_FN_WIDTH> one_raw = ((ap_uint<MPC_HLS_FN_WIDTH>)1)
                                        << one_bit;
    if (right_norm >= one_raw && shift < (MPC_HLS_FN_WIDTH - 2)) {
      shift++;
    }
  }

  if (shift > (MPC_HLS_FN_WIDTH - 2))
    shift = (MPC_HLS_FN_WIDTH - 2);
  if (shift < -(MPC_HLS_FN_WIDTH - 2))
    shift = -(MPC_HLS_FN_WIDTH - 2);

  fp_FN_t x_norm = abs_x;
  for (int s = 1; s < MPC_HLS_FN_WIDTH - 1; s++) {
#pragma HLS UNROLL
    if (shift == s)
      x_norm = abs_x >> s;
    if (shift == -s)
      x_norm = abs_x << s;
  }

  /* LUT index for x_norm in [0.5, 1.0):
  *
  * FP_FN_FRAC_BITS = 17
  * raw(0.5) = 1 << 16
  * raw spacing across [0.5,1.0) for 512 bins = 1 << 7
  *
  * Therefore the 9-bit bin index is the slice:
  *   [FP_FN_FRAC_BITS - 2 : FP_FN_FRAC_BITS - 10]
  * = [15:7]
  */
  ap_uint<MPC_HLS_FN_WIDTH> norm_raw_u =
      (ap_uint<MPC_HLS_FN_WIDTH>)fp_fn_raw_from_FN(x_norm);

  const int lut_hi = FP_FN_FRAC_BITS - 2;
  const int lut_lo = FP_FN_FRAC_BITS - 10;
  int lut_idx = (int)(norm_raw_u.range(lut_hi, lut_lo));

  if (lut_idx > 511)
    lut_idx = 511;

  /* Seed is a Q24.17 raw integer — load directly as FN raw bits. */
  fp_FN_t est = fp_FN_from_fn_raw((fp_fn_raw_t)recip_lut_fn[lut_idx]);

  /* 1 NR iteration: est*(2 - x_norm*est). */
  est = fp_mul_fn(est, (FP_FN_TWO - fp_mul_fn(x_norm, est)));

  fp_FN_t est_denorm = est;
  for (int s = 1; s < MPC_HLS_FN_WIDTH - 1; s++) {
#pragma HLS UNROLL
    if (shift == s)
      est_denorm = est >> s;
    if (shift == -s)
      est_denorm = est << s;
  }

  return neg ? fp_FN_t(-est_denorm) : est_denorm;
}
