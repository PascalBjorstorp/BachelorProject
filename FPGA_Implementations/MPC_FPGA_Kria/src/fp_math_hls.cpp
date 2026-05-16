/**
 * @file fp_math_hls.cpp
 * @brief Fixed-point math kernels for HLS synthesis.
 * @details Implements non-inline trigonometric and reciprocal kernels used by
 *          the FPGA MPC pipeline. Arithmetic is performed in fixed-point with
 *          deterministic iteration counts suited for synthesis scheduling.
 * @dependencies fp_math_hls.h
 */

#include "../include/fp_math_hls.h"
#include "../include/fp_trig_lut_1024.h"
#include <climits>
#include <cstdint>
#include <cstdio>

#ifdef CAST_AUDIT
static unsigned long long g_fp_cast_audit_counts[FP_CAST_AUDIT_COUNT] = {0};
static unsigned long long g_fp_cast_sum2_site_counts[FP_CAST_SITE_COUNT] = {0};
static unsigned long long g_fp_cast_mulqp_site_counts[FP_CAST_SITE_COUNT] = {0};

static const char *k_fp_cast_audit_names[FP_CAST_AUDIT_COUNT] = {
    "cast_sum2_qp_raw_to_qp",
    "fp_shift_right_cast_to_qp(QP_mul)"};

static const char *k_fp_cast_site_names[FP_CAST_SITE_COUNT] = {
    "UNKNOWN",
    "SUM2:add_cast_QP_raw",
    "SUM2:riccati.sub_cast_qp_raw",
    "SUM2:riccati.add3_cast_qp_raw",
    "MULQP:fp_math.fp_mul_QP_raw_q",
    "MULQP:fp_math.fp_mul",
    "MULQP:fp_math.fp_sq",
    "MULQP:mpc_riccati.A0*x0",
    "MULQP:mpc_riccati.A1*x1",
    "MULQP:mpc_riccati.A2*x2",
    "MULQP:mpc_riccati.A3*x3",
    "MULQP:mpc_riccati.A4*x4",
    "MULQP:mpc_riccati.B0*delta_k",
    "MULQP:mpc_riccati.B1*uk1",
    "MULQP:mpc_riccati.DT*uk0",
    "MULQP:riccati.rho*(z-y) term Pn e_y",
    "MULQP:riccati.rho*(z-y) term Pn delta",
    "MULQP:riccati.rho*(z-y) term Qk e_y",
    "MULQP:riccati.rho*(z-y) term Qk delta",
    "MULQP:riccati.rho_u*(z-y) ctrl",
    "MULQP:top.ref_vx*kappa",
    "MULQP:top.steer_rate",
    "MULQP:mr.0.5*(lb+ub)",
    "MULQP:mr.atan(wb*kappa)",
    "MULQP:mr.delta+dt*uk0",
    "MULQP:mr.q_lat*ey_ref",
    "MULQP:mr.q_hdg*epsi_ref",
    "MULQP:mr.q_vel*vx_ref",
    "MULQP:mr.q_lat_vel*vy_ref",
    "MULQP:mr.q_yaw*omega_ref",
    "MULQP:mr.q_delta*dff",
    "MULQP:mr.v_blend_0.7",
    "MULQP:mr.v_blend_0.3",
    "MULQP:mr.v_switch/v",
    "MULQP:mr.uub=amax*scale",
    "MULQP:mr.term_q_ey",
    "MULQP:mr.term_q_hdg",
    "MULQP:mr.term_q_vel",
    "MULQP:mr.term_q_lat_vel",
    "MULQP:mr.term_q_yaw",
    "MULQP:mr.term_q_delta",
    "MULQP:mr.persist_steer",
    "MULQP:rs.dual_state_dd",
    "MULQP:rs.dual_state_absl",
    "MULQP:rs.dual_ctrl_dd",
    "MULQP:rs.dual_ctrl_absl",
    "MULQP:rs.inv.det00",
    "MULQP:rs.inv.det01",
    "MULQP:rs.inv.si00",
    "MULQP:rs.inv.si01",
    "MULQP:rs.inv.si10",
    "MULQP:rs.inv.si11",
    "MULQP:rs.eps_primal_rel",
    "MULQP:rs.eps_dual_rel",
    "MULQP:rs.adapt_state_dual",
    "MULQP:rs.adapt_state_primal",
    "MULQP:rs.adapt_ctrl_dual",
    "MULQP:rs.adapt_ctrl_primal"};

extern "C" void fp_cast_audit_bump(int id) {
  if (id >= 0 && id < FP_CAST_AUDIT_COUNT)
    g_fp_cast_audit_counts[id]++;
}

extern "C" void fp_cast_audit_bump_sum2_site(int site_id) {
  if (site_id >= 0 && site_id < FP_CAST_SITE_COUNT)
    g_fp_cast_sum2_site_counts[site_id]++;
}

extern "C" void fp_cast_audit_bump_mulqp_site(int site_id) {
  if (site_id >= 0 && site_id < FP_CAST_SITE_COUNT)
    g_fp_cast_mulqp_site_counts[site_id]++;
}

extern "C" void fp_cast_audit_reset(void) {
  for (int i = 0; i < FP_CAST_AUDIT_COUNT; ++i)
    g_fp_cast_audit_counts[i] = 0;
  for (int i = 0; i < FP_CAST_SITE_COUNT; ++i) {
    g_fp_cast_sum2_site_counts[i] = 0;
    g_fp_cast_mulqp_site_counts[i] = 0;
  }
}

extern "C" unsigned long long fp_cast_audit_get_count(int id) {
  if (id < 0 || id >= FP_CAST_AUDIT_COUNT)
    return 0;
  return g_fp_cast_audit_counts[id];
}

extern "C" const char *fp_cast_audit_get_name(int id) {
  if (id < 0 || id >= FP_CAST_AUDIT_COUNT)
    return "invalid";
  return k_fp_cast_audit_names[id];
}

extern "C" void fp_cast_audit_print_summary(void) {
  std::printf("\n=== CAST_AUDIT Summary ===\n");
  for (int i = 0; i < FP_CAST_AUDIT_COUNT; ++i) {
    std::printf("  %-38s : %llu\n", k_fp_cast_audit_names[i],
                g_fp_cast_audit_counts[i]);
  }

  std::printf("\n=== CAST_AUDIT Site Hotspots ===\n");
  for (int i = 0; i < FP_CAST_SITE_COUNT; ++i) {
    const unsigned long long sum2 = g_fp_cast_sum2_site_counts[i];
    const unsigned long long mul = g_fp_cast_mulqp_site_counts[i];
    const unsigned long long total = sum2 + mul;
    if (total == 0)
      continue;
    std::printf("  %-36s total=%llu sum2=%llu mulqp=%llu\n",
                k_fp_cast_site_names[i], total, sum2, mul);
  }
}
#else
extern "C" void fp_cast_audit_reset(void) {}
extern "C" unsigned long long fp_cast_audit_get_count(int id) {
  (void)id;
  return 0;
}
extern "C" const char *fp_cast_audit_get_name(int id) {
  (void)id;
  return "disabled";
}
extern "C" void fp_cast_audit_print_summary(void) {}
#endif

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


typedef ap_int<(MPC_HLS_QP_WIDTH + MPC_HLS_QP_FRAC_BITS)> fp_qp_recip_shift_t;
typedef ap_int<(MPC_HLS_FN_WIDTH + MPC_HLS_FN_FRAC_BITS)> fp_fn_recip_shift_t;

static inline fp_QP_raw_t fp_mul_QP_raw_q(fp_QP_raw_t a, fp_QP_raw_t b) {
#pragma HLS INLINE
  fp_QP_mul_t product = (fp_QP_mul_t)a * (fp_QP_mul_t)b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
  return fp_shift_right_cast_to_qp_site(product, FP_FRAC_BITS,
                                        FP_CAST_SITE_MUL_FP_MUL_QP_RAW_Q);
}

static inline fp_QP_raw_t fp_shift_qp_raw_sel(fp_QP_raw_t value, int shift) {
#pragma HLS INLINE
  fp_QP_raw_t shifted = value;
  for (int s = 1; s < MPC_HLS_QP_WIDTH - 1; ++s) {
#pragma HLS UNROLL
    if (shift == s)
      shifted = (fp_QP_raw_t)(value >> s);
    if (shift == -s)
      shifted = (fp_QP_raw_t)(value << s);
  }
  return shifted;
}

static inline fp_QP_raw_t fp_shift_qp_raw_cast_sel(fp_QP_raw_t value, int shift) {
#pragma HLS INLINE
  fp_qp_recip_shift_t shifted = (fp_qp_recip_shift_t)value;
  for (int s = 1; s < MPC_HLS_QP_WIDTH - 1; ++s) {
#pragma HLS UNROLL
    if (shift == s)
      shifted = (fp_qp_recip_shift_t)(value >> s);
    if (shift == -s)
      shifted = ((fp_qp_recip_shift_t)value) << s;
  }
  return (fp_QP_raw_t)shifted;
}

static inline fp_QP_raw_t fp_lerp_qp_raw(fp_QP_raw_t v0_raw,
                                         fp_QP_raw_t v1_raw,
                                         fp_QP_raw_t frac_raw) {
#pragma HLS INLINE
  const fp_QP_raw_t dv_raw = (fp_QP_raw_t)(v1_raw - v0_raw);
  const fp_QP_raw_t interp_raw = fp_mul_QP_raw_q(frac_raw, dv_raw);
  return (fp_QP_raw_t)(v0_raw + interp_raw);
}

static inline fp_fn_raw_t fp_mul_fn_raw_q(fp_fn_raw_t a, fp_fn_raw_t b) {
#pragma HLS INLINE
  fp_fn_accum_t product = (fp_fn_accum_t)a * (fp_fn_accum_t)b;
#pragma HLS BIND_OP variable = product op = mul impl = dsp latency = MPC_HLS_MUL_LATENCY
  return (fp_fn_raw_t)(product >> FP_FN_FRAC_BITS);
}

static inline fp_fn_raw_t fp_shift_fn_raw_sel(fp_fn_raw_t value, int shift) {
#pragma HLS INLINE
  fp_fn_raw_t shifted = value;
  for (int s = 1; s < MPC_HLS_FN_WIDTH - 1; ++s) {
#pragma HLS UNROLL
    if (shift == s)
      shifted = (fp_fn_raw_t)(value >> s);
    if (shift == -s)
      shifted = (fp_fn_raw_t)(value << s);
  }
  return shifted;
}

static inline fp_fn_raw_t fp_shift_fn_raw_cast_sel(fp_fn_raw_t value, int shift) {
#pragma HLS INLINE
  fp_fn_recip_shift_t shifted = (fp_fn_recip_shift_t)value;
  for (int s = 1; s < MPC_HLS_FN_WIDTH - 1; ++s) {
#pragma HLS UNROLL
    if (shift == s)
      shifted = (fp_fn_recip_shift_t)(value >> s);
    if (shift == -s)
      shifted = ((fp_fn_recip_shift_t)value) << s;
  }
  return (fp_fn_raw_t)shifted;
}

static inline fp_fn_raw_t fp_lerp_fn_raw(fp_fn_raw_t v0_raw,
                                         fp_fn_raw_t v1_raw,
                                         fp_fn_raw_t frac_raw) {
#pragma HLS INLINE
  const fp_fn_raw_t dv_raw = (fp_fn_raw_t)(v1_raw - v0_raw);
  const fp_fn_raw_t interp_raw = fp_mul_fn_raw_q(frac_raw, dv_raw);
  return (fp_fn_raw_t)(v0_raw + interp_raw);
}

/*----------------------------------------------------------------------------
 * Specialized Riccati-family exact-width raw multipliers
 *----------------------------------------------------------------------------*/

fp_P_QP_mul_t fp_mul_P_QP(fp_P_raw_t a, fp_QP_raw_t b) {
#pragma HLS INLINE
  fp_P_QP_mul_t product = (fp_P_QP_mul_t)a * (fp_P_QP_mul_t)b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
  return product;
}

fp_P_QP_mul_t fp_mul_QP_P(fp_QP_raw_t a, fp_P_raw_t b) {
#pragma HLS INLINE
  fp_P_QP_mul_t product = (fp_P_QP_mul_t)a * (fp_P_QP_mul_t)b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
  return product;
}

fp_MG_QP_mul_t fp_mul_MG_QP(fp_MG_raw_t a, fp_QP_raw_t b) {
#pragma HLS INLINE
  fp_MG_QP_mul_t product = (fp_MG_QP_mul_t)a * (fp_MG_QP_mul_t)b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
  return product;
}

fp_MG_QP_mul_t fp_mul_QP_MG(fp_QP_raw_t a, fp_MG_raw_t b) {
#pragma HLS INLINE
  fp_MG_QP_mul_t product = (fp_MG_QP_mul_t)a * (fp_MG_QP_mul_t)b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
  return product;
}

fp_Si_MG_mul_t fp_mul_Si_MG(fp_Si_raw_t a, fp_MG_raw_t b) {
#pragma HLS INLINE
  fp_Si_MG_mul_t product = (fp_Si_MG_mul_t)a * (fp_Si_MG_mul_t)b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
  return product;
}

fp_Si_MG_mul_t fp_mul_MG_Si(fp_MG_raw_t a, fp_Si_raw_t b) {
#pragma HLS INLINE
  fp_Si_MG_mul_t product = (fp_Si_MG_mul_t)a * (fp_Si_MG_mul_t)b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
  return product;
}

fp_MG_K_mul_t fp_mul_MG_K(fp_MG_raw_t a, fp_K_raw_t b) {
#pragma HLS INLINE
  fp_MG_K_mul_t product = (fp_MG_K_mul_t)a * (fp_MG_K_mul_t)b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
  return product;
}

fp_MG_K_mul_t fp_mul_K_MG(fp_K_raw_t a, fp_MG_raw_t b) {
#pragma HLS INLINE
  fp_MG_K_mul_t product = (fp_MG_K_mul_t)a * (fp_MG_K_mul_t)b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
  return product;
}

fp_K_QP_mul_t fp_mul_K_QP(fp_K_raw_t a, fp_QP_raw_t b) {
#pragma HLS INLINE
  fp_K_QP_mul_t product = (fp_K_QP_mul_t)a * (fp_K_QP_mul_t)b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
  return product;
}

fp_K_QP_mul_t fp_mul_QP_K(fp_QP_raw_t a, fp_K_raw_t b) {
#pragma HLS INLINE
  fp_K_QP_mul_t product = (fp_K_QP_mul_t)a * (fp_K_QP_mul_t)b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
  return product;
}

/*----------------------------------------------------------------------------
 * Base-family multiply helpers
 *----------------------------------------------------------------------------*/

fp_QP_t fp_mul_site(fp_QP_t a, fp_QP_t b, int site_id) {
#pragma HLS INLINE
  fp_QP_mul_t product = fp_mul_QP_raw(fp_qp_raw_from_QP(a),
                                      fp_qp_raw_from_QP(b));
  fp_QP_raw_t product_q = fp_shift_right_cast_to_qp_site(
      product, FP_FRAC_BITS, site_id);
  return fp_QP_from_qp_raw(product_q);
}

fp_QP_t fp_mul(fp_QP_t a, fp_QP_t b) {
#pragma HLS INLINE
  return fp_mul_site(a, b, FP_CAST_SITE_MUL_FP_MUL);
}

/* fp_sq: dedicated squaring helper — INLINE off required.
 * Keeping fp_sq out-of-line preserves its own multiply context and DSP binding.
 */
fp_QP_t fp_sq(fp_QP_t x) {
#pragma HLS INLINE off
  fp_QP_mul_t product = fp_mul_QP_raw(fp_qp_raw_from_QP(x),
                                      fp_qp_raw_from_QP(x));
  fp_QP_raw_t product_q =
      fp_shift_right_cast_to_qp_site(product, FP_FRAC_BITS, FP_CAST_SITE_MUL_FP_SQ);
  return fp_QP_from_qp_raw(product_q);
}

/*--------------------------------------------------------------------------
 * Invert 2x2 QP Matrix
 *--------------------------------------------------------------------------*/

static inline ap_uint<MPC_HLS_QP_WIDTH - 1> abs_qp_raw_u(fp_QP_raw_t v) {
#pragma HLS INLINE
  return (v < 0) ? (ap_uint<MPC_HLS_QP_WIDTH - 1>)(-v)
                 : (ap_uint<MPC_HLS_QP_WIDTH - 1>)v;
}

int invert_2x2_qp_hls(fp_QP_raw_t S[2][2], fp_QP_raw_t Si[2][2]) {
#pragma HLS INLINE off
#pragma HLS ALLOCATION function instances = fp_recip limit = 1
  ap_uint<MPC_HLS_QP_WIDTH - 1> max_abs = abs_qp_raw_u(S[0][0]);
  {
    ap_uint<MPC_HLS_QP_WIDTH - 1> t = abs_qp_raw_u(S[0][1]);
    if (t > max_abs)
      max_abs = t;
  }
  {
    ap_uint<MPC_HLS_QP_WIDTH - 1> t = abs_qp_raw_u(S[1][0]);
    if (t > max_abs)
      max_abs = t;
  }
  {
    ap_uint<MPC_HLS_QP_WIDTH - 1> t = abs_qp_raw_u(S[1][1]);
    if (t > max_abs)
      max_abs = t;
  }

  const int target_magnitude_bit = FP_FRAC_BITS + 6;
  int scale_shift = 0;
  for (int b = MPC_HLS_QP_WIDTH - 2; b > target_magnitude_bit; --b) {
#pragma HLS UNROLL
    if (max_abs[b]) {
      scale_shift = b - target_magnitude_bit;
      break;
    }
  }

  const fp_QP_raw_t s00_sc = (fp_QP_raw_t)(S[0][0] >> scale_shift);
  const fp_QP_raw_t s01_sc = (fp_QP_raw_t)(S[0][1] >> scale_shift);
  const fp_QP_raw_t s10_sc = (fp_QP_raw_t)(S[1][0] >> scale_shift);
  const fp_QP_raw_t s11_sc = (fp_QP_raw_t)(S[1][1] >> scale_shift);

  const fp_QP_mul_t p00_raw = fp_mul_QP_raw(s00_sc, s11_sc);
  const fp_QP_mul_t p01_raw = fp_mul_QP_raw(s01_sc, s10_sc);
  const ap_int<(2 * MPC_HLS_QP_WIDTH + 1)> det_mul_raw =
      (ap_int<(2 * MPC_HLS_QP_WIDTH + 1)>)p00_raw -
      (ap_int<(2 * MPC_HLS_QP_WIDTH + 1)>)p01_raw;
  const ap_int<(2 * MPC_HLS_QP_WIDTH + 1)> det_shifted_raw =
      det_mul_raw >> FP_FRAC_BITS;
  const fp_QP_raw_t det_raw = (fp_QP_raw_t)det_shifted_raw;

  const fp_QP_raw_t det_eps_raw =
      fp_qp_raw_from_neg_pow2(FP_INVERT_2X2_DET_MIN_EXP);
  if (det_raw > -det_eps_raw && det_raw < det_eps_raw)
    return -1;

  const fp_QP_t inv_det = fp_recip(fp_QP_from_qp_raw(det_raw));
  const fp_QP_raw_t inv_det_raw = fp_qp_raw_from_QP(inv_det);

  const fp_QP_raw_t si00_sc =
      (fp_QP_raw_t)(fp_mul_QP_raw(s11_sc, inv_det_raw) >> FP_FRAC_BITS);
  const fp_QP_raw_t si01_sc = (fp_QP_raw_t)(fp_mul_QP_raw((fp_QP_raw_t)(-s01_sc),
                                                           inv_det_raw) >>
                                            FP_FRAC_BITS);
  const fp_QP_raw_t si10_sc = (fp_QP_raw_t)(fp_mul_QP_raw((fp_QP_raw_t)(-s10_sc),
                                                           inv_det_raw) >>
                                            FP_FRAC_BITS);
  const fp_QP_raw_t si11_sc =
      (fp_QP_raw_t)(fp_mul_QP_raw(s00_sc, inv_det_raw) >> FP_FRAC_BITS);

  Si[0][0] = (fp_QP_raw_t)(si00_sc >> scale_shift);
  Si[0][1] = (fp_QP_raw_t)(si01_sc >> scale_shift);
  Si[1][0] = (fp_QP_raw_t)(si10_sc >> scale_shift);
  Si[1][1] = (fp_QP_raw_t)(si11_sc >> scale_shift);
  return 0;
}

/*===========================================================================
 * Normalize Angle to [-pi, pi]
 *===========================================================================*/

fp_QP_t fp_normalize_angle(fp_QP_t angle) {
#pragma HLS INLINE
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
 * Reciprocal: 1/x
 *===========================================================================*/
fp_QP_t fp_recip(fp_QP_t x) {
#pragma HLS INLINE off
  /* II=21 left intentionally. Tried II=16, but combined with the FN-side
   * changes it contributed to LUT pressure that broke WNS. Revisit only
   * after measuring this function's resource cost in isolation. */
#pragma HLS PIPELINE II = 21

  if (x == 0)
    return 0;

  const bool neg = (x < 0);
  const fp_QP_raw_t x_raw = fp_qp_raw_from_QP(x);
  const fp_QP_raw_t abs_raw_signed = neg ? (fp_QP_raw_t)(-x_raw) : x_raw;
  const ap_uint<MPC_HLS_QP_WIDTH> abs_raw = (ap_uint<MPC_HLS_QP_WIDTH>)abs_raw_signed;

  const int one_bit = FP_FRAC_BITS;
  const int half_bit = FP_FRAC_BITS - 1;
  int shift = 0;

  const int clz = (int)abs_raw.countLeadingZeros();
  const int msb = (MPC_HLS_QP_WIDTH - 1) - clz;

  if (msb > one_bit) {
    shift = msb - one_bit;
  } else if (msb < half_bit) {
    shift = -(half_bit - msb);
  }

  if (shift >= 0) {
    ap_uint<MPC_HLS_QP_WIDTH> right_norm = abs_raw;
    for (int s = 1; s < MPC_HLS_QP_WIDTH - 1; ++s) {
#pragma HLS UNROLL
      if (shift == s)
        right_norm = abs_raw >> s;
    }

    const ap_uint<MPC_HLS_QP_WIDTH> one_raw =
        ((ap_uint<MPC_HLS_QP_WIDTH>)1) << FP_FRAC_BITS;
    if (right_norm >= one_raw && shift < (MPC_HLS_QP_WIDTH - 2)) {
      shift++;
    }
  }

  if (shift > (MPC_HLS_QP_WIDTH - 2))
    shift = (MPC_HLS_QP_WIDTH - 2);
  if (shift < -(MPC_HLS_QP_WIDTH - 2))
    shift = -(MPC_HLS_QP_WIDTH - 2);

  const fp_QP_raw_t x_norm_raw = fp_shift_qp_raw_sel(abs_raw_signed, shift);

#pragma HLS BIND_STORAGE variable = recip_lut type = rom_1p impl = bram
  const ap_uint<MPC_HLS_QP_WIDTH> norm_raw_u = (ap_uint<MPC_HLS_QP_WIDTH>)x_norm_raw;
  const int lut_hi = FP_FRAC_BITS - 2;
  const int lut_lo = FP_FRAC_BITS - 11;
  int lut_idx = (int)(norm_raw_u.range(lut_hi, lut_lo));
  if (lut_idx > 1023)
    lut_idx = 1023;

  fp_QP_raw_t est_raw = (fp_QP_raw_t)recip_lut[lut_idx];
  const fp_QP_raw_t xe_raw = fp_mul_QP_raw_q(x_norm_raw, est_raw);
  const fp_QP_raw_t corr_raw = (fp_QP_raw_t)(fp_qp_raw_from_QP(FP_TWO) - xe_raw);
  est_raw = fp_mul_QP_raw_q(est_raw, corr_raw);

  const fp_QP_raw_t est_denorm_raw = fp_shift_qp_raw_cast_sel(est_raw, shift);
  return neg ? fp_QP_from_qp_raw((fp_QP_raw_t)(-est_denorm_raw))
             : fp_QP_from_qp_raw(est_denorm_raw);
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

  const fp_QP_raw_t lut_pos_raw =
      fp_mul_QP_raw_q(fp_qp_raw_from_QP(angle_u), fp_qp_raw_from_QP(FP_TRIG_LUT_SCALE));
  int idx = (int)(lut_pos_raw >> FP_FRAC_BITS);
  if (idx >= FP_TRIG_LUT_SIZE)
    idx = 0;
  if (idx < 0)
    idx = 0;

  const int idx_next = (idx + 1) & FP_TRIG_LUT_MASK;
  const fp_QP_raw_t frac_raw = lut_pos_raw - (((fp_QP_raw_t)idx) << FP_FRAC_BITS);
  const fp_QP_raw_t v0_raw = fp_qp_raw_from_QP(sin_lut[idx]);
  const fp_QP_raw_t v1_raw = fp_qp_raw_from_QP(sin_lut[idx_next]);
  return fp_QP_from_qp_raw(fp_lerp_qp_raw(v0_raw, v1_raw, frac_raw));
}

fp_QP_t fp_cos(fp_QP_t angle) {
#pragma HLS INLINE off
#pragma HLS BIND_STORAGE variable = cos_lut type = rom_2p impl = bram
  fp_QP_t angle_u = fp_normalize_angle(angle);
  if (angle_u < 0)
    angle_u += FP_TWO_PI;

  const fp_QP_raw_t lut_pos_raw =
      fp_mul_QP_raw_q(fp_qp_raw_from_QP(angle_u), fp_qp_raw_from_QP(FP_TRIG_LUT_SCALE));
  int idx = (int)(lut_pos_raw >> FP_FRAC_BITS);
  if (idx >= FP_TRIG_LUT_SIZE)
    idx = 0;
  if (idx < 0)
    idx = 0;

  const int idx_next = (idx + 1) & FP_TRIG_LUT_MASK;
  const fp_QP_raw_t frac_raw = lut_pos_raw - (((fp_QP_raw_t)idx) << FP_FRAC_BITS);
  const fp_QP_raw_t v0_raw = fp_qp_raw_from_QP(cos_lut[idx]);
  const fp_QP_raw_t v1_raw = fp_qp_raw_from_QP(cos_lut[idx_next]);
  return fp_QP_from_qp_raw(fp_lerp_qp_raw(v0_raw, v1_raw, frac_raw));
}

void fp_trig_pair_fused(fp_QP_t angle, fp_QP_t *sin_out, fp_QP_t *cos_out) {
#pragma HLS INLINE off
#pragma HLS BIND_STORAGE variable = sin_lut type = rom_2p impl = bram
#pragma HLS BIND_STORAGE variable = cos_lut type = rom_2p impl = bram
  fp_QP_t angle_u = fp_normalize_angle(angle);
  if (angle_u < 0)
    angle_u += FP_TWO_PI;

  const fp_QP_raw_t lut_pos_raw =
      fp_mul_QP_raw_q(fp_qp_raw_from_QP(angle_u), fp_qp_raw_from_QP(FP_TRIG_LUT_SCALE));
  int idx = (int)(lut_pos_raw >> FP_FRAC_BITS);
  if (idx >= FP_TRIG_LUT_SIZE)
    idx = 0;
  if (idx < 0)
    idx = 0;

  const int idx_next = (idx + 1) & FP_TRIG_LUT_MASK;
  const fp_QP_raw_t frac_raw = lut_pos_raw - (((fp_QP_raw_t)idx) << FP_FRAC_BITS);

  const fp_QP_raw_t sin_v0_raw = fp_qp_raw_from_QP(sin_lut[idx]);
  const fp_QP_raw_t sin_v1_raw = fp_qp_raw_from_QP(sin_lut[idx_next]);
  *sin_out = fp_QP_from_qp_raw(fp_lerp_qp_raw(sin_v0_raw, sin_v1_raw, frac_raw));

  const fp_QP_raw_t cos_v0_raw = fp_qp_raw_from_QP(cos_lut[idx]);
  const fp_QP_raw_t cos_v1_raw = fp_qp_raw_from_QP(cos_lut[idx_next]);
  *cos_out = fp_QP_from_qp_raw(fp_lerp_qp_raw(cos_v0_raw, cos_v1_raw, frac_raw));
}

fp_QP_t fp_atan_lut(fp_QP_t x) {
#pragma HLS INLINE off
#pragma HLS ALLOCATION function instances = fp_recip limit = 1
#pragma HLS BIND_STORAGE variable = atan_lut type = rom_2p impl = bram
  const bool neg = (x < 0);
  const fp_QP_t abs_x = fp_abs(x);
  const bool over_one = (abs_x > FP_ONE);
  const fp_QP_raw_t y_raw = fp_qp_raw_from_QP(over_one ? fp_recip(abs_x) : abs_x);
  const fp_QP_mul_t lut_pos_wide = ((fp_QP_mul_t)y_raw) << 10;
  int idx = (int)(lut_pos_wide >> FP_FRAC_BITS);
  if (idx < 0)
    idx = 0;
  if (idx > 1023)
    idx = 1023;
  const fp_QP_raw_t frac_raw =
      (fp_QP_raw_t)(lut_pos_wide - (((fp_QP_mul_t)idx) << FP_FRAC_BITS));
  const fp_QP_raw_t v0_raw = fp_qp_raw_from_QP(atan_lut[idx]);
  const fp_QP_raw_t v1_raw = fp_qp_raw_from_QP(atan_lut[idx + 1]);
  const fp_QP_raw_t atan_y_raw = fp_lerp_qp_raw(v0_raw, v1_raw, frac_raw);
  fp_QP_raw_t result_raw = atan_y_raw;
  if (over_one) {
    result_raw = (fp_QP_raw_t)(fp_qp_raw_from_QP(FP_PI_HALF) - atan_y_raw);
  }
  return neg ? fp_QP_from_qp_raw((fp_QP_raw_t)(-result_raw))
             : fp_QP_from_qp_raw(result_raw);
}

/*===========================================================================
 * FN family
 *===========================================================================*/

#include "../include/fp_trig_lut_fn_1024.h"

fp_FN_t fp_mul_fn(fp_FN_t a, fp_FN_t b) {
#pragma HLS INLINE
  fp_fn_accum_t product = (fp_fn_accum_t)fp_fn_raw_from_FN(a) *
                          (fp_fn_accum_t)fp_fn_raw_from_FN(b);
#pragma HLS BIND_OP variable = product op = mul impl = dsp latency = MPC_HLS_MUL_LATENCY
  fp_fn_raw_t product_q = (fp_fn_raw_t)(product >> FP_FN_FRAC_BITS);
  return fp_FN_from_fn_raw(product_q);
}

fp_fn_accum_t fp_mul_fn_raw(fp_FN_t a, fp_FN_t b) {
#pragma HLS INLINE
  fp_fn_accum_t product = (fp_fn_accum_t)fp_fn_raw_from_FN(a) *
                          (fp_fn_accum_t)fp_fn_raw_from_FN(b);
#pragma HLS BIND_OP variable = product op = mul impl = dsp latency = MPC_HLS_MUL_LATENCY
  return product;
}

fp_FN_t fp_sin_fn(fp_FN_t angle) {
#pragma HLS INLINE off
#pragma HLS BIND_STORAGE variable = sin_lut_fn type = rom_2p impl = bram
  const bool neg = (angle < FP_FN_ZERO);
  const fp_FN_t abs_angle = neg ? fp_FN_t(-angle) : angle;

  const bool over_pi = (abs_angle > FP_FN_PI);
  const fp_FN_t a = over_pi ? fp_FN_t(abs_angle - FP_FN_PI) : abs_angle;
  const bool negate = neg ^ over_pi;

  const fp_fn_accum_t lut_scaled_raw = fp_mul_fn_raw(a, FP_FN_TRIG_LUT_SCALE);
  int32_t lut_pos_raw = (int32_t)(lut_scaled_raw >> FP_FN_FRAC_BITS);
  int idx = (int)(lut_pos_raw >> FP_FN_FRAC_BITS);
  if (idx >= FP_TRIG_LUT_SIZE)
    idx = FP_TRIG_LUT_SIZE - 1;
  if (idx < 0)
    idx = 0;

  const int idx_next = (idx + 1) & FP_TRIG_LUT_MASK;
  const fp_fn_raw_t frac_raw =
      (fp_fn_raw_t)(lut_pos_raw - (((int32_t)idx) << FP_FN_FRAC_BITS));
  const fp_fn_raw_t v0_raw = fp_fn_raw_from_FN(sin_lut_fn[idx]);
  const fp_fn_raw_t v1_raw = fp_fn_raw_from_FN(sin_lut_fn[idx_next]);
  const fp_fn_raw_t result_raw = fp_lerp_fn_raw(v0_raw, v1_raw, frac_raw);
  return negate ? fp_FN_from_fn_raw((fp_fn_raw_t)(-result_raw))
                : fp_FN_from_fn_raw(result_raw);
}

fp_FN_t fp_cos_fn(fp_FN_t angle) {
#pragma HLS INLINE off
#pragma HLS BIND_STORAGE variable = cos_lut_fn type = rom_2p impl = bram
  const fp_FN_t abs_angle = (angle < FP_FN_ZERO) ? fp_FN_t(-angle) : angle;

  const bool over_pi = (abs_angle > FP_FN_PI);
  const fp_FN_t a = over_pi ? fp_FN_t(abs_angle - FP_FN_PI) : abs_angle;

  const fp_fn_accum_t lut_scaled_raw = fp_mul_fn_raw(a, FP_FN_TRIG_LUT_SCALE);
  int32_t lut_pos_raw = (int32_t)(lut_scaled_raw >> FP_FN_FRAC_BITS);
  int idx = (int)(lut_pos_raw >> FP_FN_FRAC_BITS);
  if (idx >= FP_TRIG_LUT_SIZE)
    idx = FP_TRIG_LUT_SIZE - 1;
  if (idx < 0)
    idx = 0;

  const int idx_next = (idx + 1) & FP_TRIG_LUT_MASK;
  const fp_fn_raw_t frac_raw =
      (fp_fn_raw_t)(lut_pos_raw - (((int32_t)idx) << FP_FN_FRAC_BITS));
  const fp_fn_raw_t v0_raw = fp_fn_raw_from_FN(cos_lut_fn[idx]);
  const fp_fn_raw_t v1_raw = fp_fn_raw_from_FN(cos_lut_fn[idx_next]);
  const fp_fn_raw_t result_raw = fp_lerp_fn_raw(v0_raw, v1_raw, frac_raw);
  return over_pi ? fp_FN_from_fn_raw((fp_fn_raw_t)(-result_raw))
                 : fp_FN_from_fn_raw(result_raw);
}

void fp_trig_pair_fused_fn(fp_FN_t angle, fp_FN_t *sin_out, fp_FN_t *cos_out) {
#pragma HLS INLINE off
#pragma HLS BIND_STORAGE variable = sin_lut_fn type = rom_2p impl = bram
#pragma HLS BIND_STORAGE variable = cos_lut_fn type = rom_2p impl = bram
  const bool neg = (angle < FP_FN_ZERO);
  const fp_FN_t abs_angle = neg ? fp_FN_t(-angle) : angle;
  const bool over_pi = (abs_angle > FP_FN_PI);
  const fp_FN_t a = over_pi ? fp_FN_t(abs_angle - FP_FN_PI) : abs_angle;

  const fp_fn_accum_t lut_scaled_raw = fp_mul_fn_raw(a, FP_FN_TRIG_LUT_SCALE);
  int32_t lut_pos_raw = (int32_t)(lut_scaled_raw >> FP_FN_FRAC_BITS);
  int idx = (int)(lut_pos_raw >> FP_FN_FRAC_BITS);
  if (idx >= FP_TRIG_LUT_SIZE)
    idx = FP_TRIG_LUT_SIZE - 1;
  if (idx < 0)
    idx = 0;

  const int idx_next = (idx + 1) & FP_TRIG_LUT_MASK;
  const fp_fn_raw_t frac_raw =
      (fp_fn_raw_t)(lut_pos_raw - (((int32_t)idx) << FP_FN_FRAC_BITS));

  const bool sin_negate = neg ^ over_pi;
  const fp_fn_raw_t sin_v0_raw = fp_fn_raw_from_FN(sin_lut_fn[idx]);
  const fp_fn_raw_t sin_v1_raw = fp_fn_raw_from_FN(sin_lut_fn[idx_next]);
  const fp_fn_raw_t sin_raw = fp_lerp_fn_raw(sin_v0_raw, sin_v1_raw, frac_raw);
  *sin_out = sin_negate ? fp_FN_from_fn_raw((fp_fn_raw_t)(-sin_raw))
                        : fp_FN_from_fn_raw(sin_raw);

  const fp_fn_raw_t cos_v0_raw = fp_fn_raw_from_FN(cos_lut_fn[idx]);
  const fp_fn_raw_t cos_v1_raw = fp_fn_raw_from_FN(cos_lut_fn[idx_next]);
  const fp_fn_raw_t cos_raw = fp_lerp_fn_raw(cos_v0_raw, cos_v1_raw, frac_raw);
  *cos_out = over_pi ? fp_FN_from_fn_raw((fp_fn_raw_t)(-cos_raw))
                     : fp_FN_from_fn_raw(cos_raw);
}

fp_FN_t fp_atan_lut_fn(fp_FN_t x) {
#pragma HLS INLINE off
#pragma HLS ALLOCATION function instances = fp_recip_fn limit = 1
#pragma HLS BIND_STORAGE variable = atan_lut_fn type = rom_2p impl = bram
  const bool neg = (x < FP_FN_ZERO);
  const fp_FN_t abs_x = neg ? fp_FN_t(-x) : x;
  const bool over_one = (abs_x > FP_FN_ONE);
  const fp_fn_raw_t y_raw = fp_fn_raw_from_FN(over_one ? fp_recip_fn(abs_x) : abs_x);

  int idx = y_raw >> 7;
  if (idx < 0)
    idx = 0;
  if (idx > 1023)
    idx = 1023;

  const fp_fn_raw_t frac_raw =
      (fp_fn_raw_t)(((int32_t)y_raw << 10) - (((int32_t)idx) << FP_FN_FRAC_BITS));
  const fp_fn_raw_t v0_raw = fp_fn_raw_from_FN(atan_lut_fn[idx]);
  const fp_fn_raw_t v1_raw = fp_fn_raw_from_FN(atan_lut_fn[idx + 1]);
  const fp_fn_raw_t atan_y_raw = fp_lerp_fn_raw(v0_raw, v1_raw, frac_raw);
  fp_fn_raw_t result_raw = atan_y_raw;
  if (over_one) {
    result_raw = (fp_fn_raw_t)(fp_fn_raw_from_FN(FP_FN_PI_HALF) - atan_y_raw);
  }
  return neg ? fp_FN_from_fn_raw((fp_fn_raw_t)(-result_raw))
             : fp_FN_from_fn_raw(result_raw);
}

fp_FN_t fp_recip_fn(fp_FN_t x) {
#pragma HLS INLINE off
  /* II=21 left intentionally. Same reason as fp_recip (QP) above:
   * tried II=16 alongside FN_MUL_LATENCY=2 and the LUT cost grew tire
   * by ~6500 LUT, breaking WNS in riccati_pass via placement spread. */
#pragma HLS PIPELINE II = 21
#pragma HLS BIND_STORAGE variable = recip_lut_fn type = rom_1p impl = bram
  if (x == 0)
    return 0;

  const bool neg = (x < 0);
  const fp_fn_raw_t x_raw = fp_fn_raw_from_FN(x);
  const fp_fn_raw_t abs_raw_signed = neg ? (fp_fn_raw_t)(-x_raw) : x_raw;
  const ap_uint<MPC_HLS_FN_WIDTH> abs_raw = (ap_uint<MPC_HLS_FN_WIDTH>)abs_raw_signed;

  const int one_bit = FP_FN_FRAC_BITS;
  const int half_bit = FP_FN_FRAC_BITS - 1;
  int shift = 0;

  const int clz = (int)abs_raw.countLeadingZeros();
  const int msb = (MPC_HLS_FN_WIDTH - 1) - clz;

  if (msb > one_bit) {
    shift = msb - one_bit;
  } else if (msb < half_bit) {
    shift = -(half_bit - msb);
  }

  if (shift >= 0) {
    ap_uint<MPC_HLS_FN_WIDTH> right_norm = abs_raw;
    for (int s = 1; s < MPC_HLS_FN_WIDTH - 1; ++s) {
#pragma HLS UNROLL
      if (shift == s)
        right_norm = abs_raw >> s;
    }
    const ap_uint<MPC_HLS_FN_WIDTH> one_raw =
        ((ap_uint<MPC_HLS_FN_WIDTH>)1) << FP_FN_FRAC_BITS;
    if (right_norm >= one_raw && shift < (MPC_HLS_FN_WIDTH - 2)) {
      shift++;
    }
  }

  if (shift > (MPC_HLS_FN_WIDTH - 2))
    shift = (MPC_HLS_FN_WIDTH - 2);
  if (shift < -(MPC_HLS_FN_WIDTH - 2))
    shift = -(MPC_HLS_FN_WIDTH - 2);

  const fp_fn_raw_t x_norm_raw = fp_shift_fn_raw_sel(abs_raw_signed, shift);
  const ap_uint<MPC_HLS_FN_WIDTH> norm_raw_u = (ap_uint<MPC_HLS_FN_WIDTH>)x_norm_raw;
  const int lut_hi = FP_FN_FRAC_BITS - 2;
  const int lut_lo = FP_FN_FRAC_BITS - 10;
  int lut_idx = (int)(norm_raw_u.range(lut_hi, lut_lo));
  if (lut_idx > 511)
    lut_idx = 511;

  fp_fn_raw_t est_raw = (fp_fn_raw_t)recip_lut_fn[lut_idx];
  const fp_fn_raw_t xe_raw = fp_mul_fn_raw_q(x_norm_raw, est_raw);
  const fp_fn_raw_t corr_raw = (fp_fn_raw_t)(fp_fn_raw_from_FN(FP_FN_TWO) - xe_raw);
  est_raw = fp_mul_fn_raw_q(est_raw, corr_raw);

  const fp_fn_raw_t est_denorm_raw = fp_shift_fn_raw_cast_sel(est_raw, shift);
  return neg ? fp_FN_from_fn_raw((fp_fn_raw_t)(-est_denorm_raw))
             : fp_FN_from_fn_raw(est_denorm_raw);
}
