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

/* Overloaded fp_mul for QP type (internal Riccati precision).
    When fp_io_t and fp_QP_t are the same type (both Q32.16), this overload
    serves both; no separate fp_io_t overload is defined (see conditional
   above). */
fp_QP_t fp_mul(fp_QP_t a, fp_QP_t b) {
#pragma HLS INLINE
  fp_raw_mul_t product =
      (fp_raw_mul_t)fp_qp_raw_from_QP(a) * (fp_raw_mul_t)fp_qp_raw_from_QP(b);
#pragma HLS BIND_OP variable = product op = mul impl = dsp latency =           \
    MPC_HLS_MUL_LATENCY
  fp_raw_acc_t product_q = fp_shift_right_clip_to_acc(product, FP_FRAC_BITS);
  return fp_QP_from_qp_raw((fp_qp_raw_t)product_q);
}

/* fp_sq: dedicated squaring helper — INLINE off required.
 * When fp_mul is inlined for self-product (a*a), HLS SSA renames 'product'
 * (both operands identical), losing the BIND_OP DSP target and falling back
 * to a 30-bit LUT multiplier (~3.5ns). INLINE off gives fp_sq its own
 * synthesis context where 'product' is always preserved. */
fp_QP_t fp_sq(fp_QP_t x) {
#pragma HLS INLINE off
  fp_raw_mul_t product =
      (fp_raw_mul_t)fp_qp_raw_from_QP(x) * (fp_raw_mul_t)fp_qp_raw_from_QP(x);
#pragma HLS BIND_OP variable = product op = mul impl = dsp latency = MPC_HLS_MUL_LATENCY
  fp_raw_acc_t product_q = fp_shift_right_clip_to_acc(product, FP_FRAC_BITS);
  return fp_QP_from_qp_raw((fp_qp_raw_t)product_q);
}

fp_raw_mul_t fp_mul_qp_raw(fp_qp_raw_t a, fp_qp_raw_t b) {
#pragma HLS INLINE
  fp_raw_mul_t product = (fp_raw_mul_t)a * (fp_raw_mul_t)b;
#pragma HLS BIND_OP variable = product op = mul impl = dsp latency = MPC_HLS_MUL_LATENCY
  return product;
}

fp_raw_mul_t fp_mul_qp_acc(fp_qp_raw_t a, fp_raw_acc_t b) {
#pragma HLS INLINE
  fp_raw_mul_t product = (fp_raw_mul_t)a * (fp_raw_mul_t)b;
#pragma HLS BIND_OP variable = product op = mul impl = dsp latency = MPC_HLS_MUL_LATENCY
  return product;
}

fp_raw_mul_t fp_mul_acc_qp(fp_raw_acc_t a, fp_qp_raw_t b) {
#pragma HLS INLINE
  fp_raw_mul_t product = (fp_raw_mul_t)a * (fp_raw_mul_t)b;
#pragma HLS BIND_OP variable = product op = mul impl = dsp latency = MPC_HLS_MUL_LATENCY
  return product;
}

fp_raw_mul_t fp_mul_raw_acc(fp_raw_acc_t a, fp_raw_acc_t b) {
#pragma HLS INLINE
  fp_raw_mul_t product = (fp_raw_mul_t)a * (fp_raw_mul_t)b;
#pragma HLS BIND_OP variable = product op = mul impl = dsp latency = MPC_HLS_MUL_LATENCY
  return product;
}

int invert_2x2_hls(fp_raw_acc_t S[2][2], fp_raw_acc_t Si[2][2]) {
#pragma HLS INLINE off
#pragma HLS ALLOCATION function instances=fp_recip limit=1
  /* S entries are raw-acc values at QP scale (>> FP_FRAC_BITS already applied
   * by caller). */
  fp_QP_t s00 = fp_QP_from_qp_raw((fp_qp_raw_t)fp_clip_raw_to_qp(S[0][0]));
  fp_QP_t s01 = fp_QP_from_qp_raw((fp_qp_raw_t)fp_clip_raw_to_qp(S[0][1]));
  fp_QP_t s10 = fp_QP_from_qp_raw((fp_qp_raw_t)fp_clip_raw_to_qp(S[1][0]));
  fp_QP_t s11 = fp_QP_from_qp_raw((fp_qp_raw_t)fp_clip_raw_to_qp(S[1][1]));

  /* Determinant in QP space. */
  fp_QP_t det = fp_mul(s00, s11) - fp_mul(s01, s10);

  /* Singular threshold of 2^-12 in real units, mapped to the active QP width. */
  fp_QP_t det_eps = fp_qp_from_neg_pow2(FP_INVERT_2X2_DET_MIN_EXP);
  if (det > -det_eps && det < det_eps) {
    return -1;
  }

  /* Newton-Raphson reciprocal in the active QP format. */
  fp_QP_t inv_det = fp_recip(det);

  /* Cramer's rule: Si = adj(S) * inv_det. */
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

/* fp_QP_t is ap_fixed<32,17>: FP_FRAC_BITS=15, 1.0 raw=32768, 2.0 raw=65536.
 * Entry i = floor(32768*1024/(512+i)) = raw Q32.17 value of 1/(0.5+i/1024). */
static const int32_t recip_lut[512] = {
    65536, 65408, 65280, 65154, 65027, 64902, 64776, 64652,
    64527, 64403, 64280, 64157, 64035, 63913, 63791, 63670,
    63550, 63429, 63310, 63191, 63072, 62953, 62836, 62718,
    62601, 62484, 62368, 62253, 62137, 62022, 61908, 61794,
    61680, 61567, 61455, 61342, 61230, 61119, 61008, 60897,
    60787, 60677, 60567, 60458, 60349, 60241, 60133, 60025,
    59918, 59811, 59705, 59599, 59493, 59388, 59283, 59178,
    59074, 58970, 58867, 58764, 58661, 58559, 58457, 58355,
    58254, 58153, 58052, 57952, 57852, 57752, 57653, 57554,
    57456, 57358, 57260, 57162, 57065, 56968, 56871, 56775,
    56679, 56584, 56488, 56394, 56299, 56205, 56111, 56017,
    55924, 55831, 55738, 55645, 55553, 55461, 55370, 55279,
    55188, 55097, 55007, 54917, 54827, 54738, 54648, 54560,
    54471, 54383, 54295, 54207, 54120, 54032, 53946, 53859,
    53773, 53687, 53601, 53515, 53430, 53345, 53261, 53176,
    53092, 53008, 52924, 52841, 52758, 52675, 52593, 52510,
    52428, 52347, 52265, 52184, 52103, 52022, 51941, 51861,
    51781, 51701, 51622, 51542, 51463, 51385, 51306, 51228,
    51150, 51072, 50994, 50917, 50840, 50763, 50686, 50610,
    50533, 50457, 50382, 50306, 50231, 50156, 50081, 50006,
    49932, 49857, 49784, 49710, 49636, 49563, 49490, 49417,
    49344, 49272, 49200, 49128, 49056, 48984, 48913, 48841,
    48770, 48700, 48629, 48559, 48489, 48419, 48349, 48279,
    48210, 48141, 48072, 48003, 47934, 47866, 47798, 47730,
    47662, 47594, 47527, 47460, 47393, 47326, 47259, 47193,
    47127, 47060, 46995, 46929, 46863, 46798, 46733, 46668,
    46603, 46538, 46474, 46410, 46345, 46281, 46218, 46154,
    46091, 46028, 45964, 45902, 45839, 45776, 45714, 45652,
    45590, 45528, 45466, 45405, 45343, 45282, 45221, 45160,
    45100, 45039, 44979, 44918, 44858, 44798, 44739, 44679,
    44620, 44560, 44501, 44442, 44384, 44325, 44267, 44208,
    44150, 44092, 44034, 43976, 43919, 43862, 43804, 43747,
    43690, 43633, 43577, 43520, 43464, 43408, 43351, 43296,
    43240, 43184, 43129, 43073, 43018, 42963, 42908, 42853,
    42799, 42744, 42690, 42635, 42581, 42527, 42473, 42420,
    42366, 42313, 42259, 42206, 42153, 42100, 42048, 41995,
    41943, 41890, 41838, 41786, 41734, 41682, 41630, 41579,
    41527, 41476, 41425, 41374, 41323, 41272, 41221, 41171,
    41120, 41070, 41020, 40970, 40920, 40870, 40820, 40770,
    40721, 40672, 40622, 40573, 40524, 40475, 40427, 40378,
    40329, 40281, 40233, 40184, 40136, 40088, 40041, 39993,
    39945, 39898, 39850, 39803, 39756, 39709, 39662, 39615,
    39568, 39522, 39475, 39429, 39383, 39336, 39290, 39244,
    39199, 39153, 39107, 39062, 39016, 38971, 38926, 38881,
    38836, 38791, 38746, 38701, 38657, 38612, 38568, 38524,
    38479, 38435, 38391, 38347, 38304, 38260, 38216, 38173,
    38130, 38086, 38043, 38000, 37957, 37914, 37871, 37829,
    37786, 37744, 37701, 37659, 37617, 37574, 37532, 37490,
    37449, 37407, 37365, 37324, 37282, 37241, 37200, 37158,
    37117, 37076, 37035, 36994, 36954, 36913, 36873, 36832,
    36792, 36751, 36711, 36671, 36631, 36591, 36551, 36511,
    36472, 36432, 36393, 36353, 36314, 36275, 36235, 36196,
    36157, 36118, 36080, 36041, 36002, 35964, 35925, 35887,
    35848, 35810, 35772, 35734, 35696, 35658, 35620, 35582,
    35544, 35507, 35469, 35432, 35394, 35357, 35320, 35283,
    35246, 35209, 35172, 35135, 35098, 35062, 35025, 34988,
    34952, 34916, 34879, 34843, 34807, 34771, 34735, 34699,
    34663, 34627, 34592, 34556, 34521, 34485, 34450, 34414,
    34379, 34344, 34309, 34274, 34239, 34204, 34169, 34134,
    34100, 34065, 34030, 33996, 33961, 33927, 33893, 33859,
    33825, 33790, 33756, 33723, 33689, 33655, 33621, 33588,
    33554, 33520, 33487, 33454, 33420, 33387, 33354, 33321,
    33288, 33255, 33222, 33189, 33156, 33123, 33091, 33058,
    33026, 32993, 32961, 32928, 32896, 32864, 32832, 32800,
};

fp_QP_t fp_recip(fp_QP_t x) {
#pragma HLS INLINE off
#pragma HLS PIPELINE II=21
  if (x == 0)
    return 0;

  bool neg = (x < 0);
  fp_QP_t abs_x = fp_abs(x);

  /* Width-aware bit-domain normalization without variable-trip loops. */
  fp_qp_raw_t abs_raw_signed = fp_qp_raw_from_QP(abs_x);
  ap_uint<MPC_HLS_RICCATI_WIDTH> abs_raw =
      (ap_uint<MPC_HLS_RICCATI_WIDTH>)abs_raw_signed;

  const int one_bit = FP_FRAC_BITS;
  const int half_bit = FP_FRAC_BITS - 1;
  int shift = 0;

  int clz = (int)abs_raw.countLeadingZeros();
  int msb = (MPC_HLS_RICCATI_WIDTH - 1) - clz;

  if (msb > one_bit) {
    shift = msb - one_bit;
  } else if (msb < half_bit) {
    shift = -(half_bit - msb);
  }

  if (shift >= 0) {
    ap_uint<MPC_HLS_RICCATI_WIDTH> right_norm = abs_raw;
    for (int s = 1; s < MPC_HLS_RICCATI_WIDTH - 1; s++) {
#pragma HLS UNROLL
      if (shift == s)
        right_norm = abs_raw >> s;
    }
    ap_uint<MPC_HLS_RICCATI_WIDTH> one_raw = ((ap_uint<MPC_HLS_RICCATI_WIDTH>)1)
                                             << FP_FRAC_BITS;
    if (right_norm >= one_raw && shift < (MPC_HLS_RICCATI_WIDTH - 2)) {
      shift++;
    }
  }

  if (shift > (MPC_HLS_RICCATI_WIDTH - 2))
    shift = (MPC_HLS_RICCATI_WIDTH - 2);
  if (shift < -(MPC_HLS_RICCATI_WIDTH - 2))
    shift = -(MPC_HLS_RICCATI_WIDTH - 2);

  fp_QP_t x_norm = abs_x;
  for (int s = 1; s < MPC_HLS_RICCATI_WIDTH - 1; s++) {
#pragma HLS UNROLL
    if (shift == s)
      x_norm = abs_x >> s;
    if (shift == -s)
      x_norm = abs_x << s;
  }

  /* LUT lookup: x_norm in [0.5,1.0) → bits [13:5] of raw Q32.17 → index [0,511].
   * FP_FRAC_BITS=15: 0.5 raw=16384=0x4000 (bit13 set), 1.0 raw=32768=0x8000. */
#pragma HLS BIND_STORAGE variable=recip_lut type=rom_1p impl=bram
  ap_uint<MPC_HLS_RICCATI_WIDTH> norm_raw_u =
      (ap_uint<MPC_HLS_RICCATI_WIDTH>)fp_qp_raw_from_QP(x_norm);
  int lut_idx = (int)(norm_raw_u.range(13, 5));
  fp_QP_t est = fp_QP_from_qp_raw((fp_qp_raw_t)recip_lut[lut_idx]);

  /* 1 NR iteration: est*(2 - x_norm*est). Sufficient after 9-bit LUT seed. */
  est = fp_mul(est, (FP_TWO - fp_mul(x_norm, est)));

  /* Undo normalization: if x = x_norm * 2^shift, then 1/x = (1/x_norm) *
   * 2^-shift. */
  fp_QP_t est_denorm = est;
  for (int s = 1; s < MPC_HLS_RICCATI_WIDTH - 1; s++) {
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
#pragma HLS BIND_STORAGE variable=sin_lut type=rom_2p impl=bram
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
  fp_qp_raw_t idx_q_raw = ((fp_qp_raw_t)idx) << FP_FRAC_BITS;
  fp_QP_t frac = lut_pos - fp_QP_from_qp_raw(idx_q_raw);
  fp_QP_t v0 = sin_lut[idx];
  fp_QP_t v1 = sin_lut[idx_next];
  return v0 + fp_mul(frac, (v1 - v0));
}

fp_QP_t fp_cos(fp_QP_t angle) {
#pragma HLS INLINE off
#pragma HLS BIND_STORAGE variable=cos_lut type=rom_2p impl=bram
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
  fp_qp_raw_t idx_q_raw = ((fp_qp_raw_t)idx) << FP_FRAC_BITS;
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
 * Max error: < 0.00003% (LUT spacing π/4 / 1024 ≈ 7.7e-4 rad; linear
 * interpolation reduces residual to < 1 ulp at Q32.17 resolution).
 *===========================================================================*/
fp_QP_t fp_atan_lut(fp_QP_t x) {
#pragma HLS INLINE off
#pragma HLS ALLOCATION function instances=fp_recip limit=1
#pragma HLS BIND_STORAGE variable=atan_lut type=rom_2p impl=bram
    bool neg = (x < 0);
    fp_QP_t abs_x = fp_abs(x);
    bool over_one = (abs_x > FP_ONE);
    fp_QP_t y = over_one ? fp_recip(abs_x) : abs_x;
    fp_QP_t lut_pos = y << 10;
    int idx = (int)lut_pos;
    if (idx < 0) idx = 0;
    if (idx > 1023) idx = 1023;
    fp_qp_raw_t idx_q_raw = ((fp_qp_raw_t)idx) << FP_FRAC_BITS;
    fp_QP_t frac = lut_pos - fp_QP_from_qp_raw(idx_q_raw);
    fp_QP_t v0 = atan_lut[idx];
    fp_QP_t v1 = atan_lut[idx + 1];
    fp_QP_t atan_y = v0 + fp_mul(frac, v1 - v0);
    fp_QP_t result = over_one ? fp_QP_t(FP_PI_HALF - atan_y) : atan_y;
    return neg ? fp_QP_t(-result) : result;
}
