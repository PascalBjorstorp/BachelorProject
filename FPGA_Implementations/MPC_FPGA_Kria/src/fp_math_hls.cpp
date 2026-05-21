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
  FP_WPROBE(FP_WP_QP_MUL, (__int128)a.to_int64() * (__int128)b.to_int64());
  fp_QP_mul_t product = (fp_QP_mul_t)a * (fp_QP_mul_t)b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp // latency = MPC_HLS_MUL_LATENCY
  return product;
}

static inline fp_QP_raw_t fp_mul_QP_raw_q(fp_QP_raw_t a, fp_QP_raw_t b) {
#pragma HLS INLINE
  fp_QP_mul_t product = (fp_QP_mul_t)a * (fp_QP_mul_t)b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp // latency = MPC_HLS_MUL_LATENCY
  return fp_shift_right_cast_to_qp_site(product, FP_FRAC_BITS,
                                        FP_CAST_SITE_MUL_FP_MUL_QP_RAW_Q);
}

static inline fp_QP_raw_t fp_shift_qp_raw_sel(fp_QP_raw_t value, int shift) {
#pragma HLS INLINE
  fp_QP_raw_t shifted = value;
  for (int s = 1; s < MPC_HLS_QP_WIDTH - 1; ++s) {
MPC_HLS_UNROLL()
    if (shift == s)
      shifted = (fp_QP_raw_t)(value >> s);
    if (shift == -s)
      shifted = (fp_QP_raw_t)(value << s);
  }
  return shifted;
}

static inline fp_QP_raw_t fp_shift_qp_raw_cast_sel(fp_QP_raw_t value, int shift) {
#pragma HLS INLINE
  fp_QP_recip_shift_t shifted = (fp_QP_recip_shift_t)value;
  for (int s = 1; s < MPC_HLS_QP_WIDTH - 1; ++s) {
MPC_HLS_UNROLL()
    if (shift == s)
      shifted = (fp_QP_recip_shift_t)(value >> s);
    if (shift == -s)
      shifted = ((fp_QP_recip_shift_t)value) << s;
  }
  FP_WPROBE(FP_WP_QP_RECIP_SHIFT, shifted.to_int64());
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
  FP_WPROBE(FP_WP_FN_MUL, (__int128)a.to_int64() * (__int128)b.to_int64());
  fp_fn_accum_t product = (fp_fn_accum_t)a * (fp_fn_accum_t)b;
#pragma HLS BIND_OP variable = product op = mul impl = dsp // latency = MPC_HLS_MUL_LATENCY
  return (fp_fn_raw_t)(product >> FP_FN_FRAC_BITS);
}

static inline fp_fn_raw_t fp_shift_fn_raw_sel(fp_fn_raw_t value, int shift) {
#pragma HLS INLINE
  fp_fn_raw_t shifted = value;
  for (int s = 1; s < MPC_HLS_FN_WIDTH - 1; ++s) {
MPC_HLS_UNROLL()
    if (shift == s)
      shifted = (fp_fn_raw_t)(value >> s);
    if (shift == -s)
      shifted = (fp_fn_raw_t)(value << s);
  }
  return shifted;
}

static inline fp_fn_raw_t fp_shift_fn_raw_cast_sel(fp_fn_raw_t value, int shift) {
#pragma HLS INLINE
  fp_FN_recip_shift_t shifted = (fp_FN_recip_shift_t)value;
  for (int s = 1; s < MPC_HLS_FN_WIDTH - 1; ++s) {
MPC_HLS_UNROLL()
    if (shift == s)
      shifted = (fp_FN_recip_shift_t)(value >> s);
    if (shift == -s)
      shifted = ((fp_FN_recip_shift_t)value) << s;
  }
  FP_WPROBE(FP_WP_FN_RECIP_SHIFT, shifted.to_int64());
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
  FP_WPROBE(FP_WP_P_QP_MUL, (__int128)a.to_int64() * (__int128)b.to_int64());
  fp_P_QP_mul_t product = (fp_P_QP_mul_t)a * (fp_P_QP_mul_t)b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp // latency = MPC_HLS_MUL_LATENCY
  return product;
}

fp_P_QP_mul_t fp_mul_QP_P(fp_QP_raw_t a, fp_P_raw_t b) {
#pragma HLS INLINE
  FP_WPROBE(FP_WP_P_QP_MUL, (__int128)a.to_int64() * (__int128)b.to_int64());
  fp_P_QP_mul_t product = (fp_P_QP_mul_t)a * (fp_P_QP_mul_t)b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp // latency = MPC_HLS_MUL_LATENCY
  return product;
}

fp_MG_QP_mul_t fp_mul_MG_QP(fp_MG_raw_t a, fp_QP_raw_t b) {
#pragma HLS INLINE
  FP_WPROBE(FP_WP_MG_QP_MUL, (__int128)a.to_int64() * (__int128)b.to_int64());
  fp_MG_QP_mul_t product = (fp_MG_QP_mul_t)a * (fp_MG_QP_mul_t)b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp // latency = MPC_HLS_MUL_LATENCY
  return product;
}

fp_MG_QP_mul_t fp_mul_QP_MG(fp_QP_raw_t a, fp_MG_raw_t b) {
#pragma HLS INLINE
  FP_WPROBE(FP_WP_MG_QP_MUL, (__int128)a.to_int64() * (__int128)b.to_int64());
  fp_MG_QP_mul_t product = (fp_MG_QP_mul_t)a * (fp_MG_QP_mul_t)b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp // latency = MPC_HLS_MUL_LATENCY
  return product;
}

fp_MG_K_mul_t fp_mul_MG_K(fp_MG_raw_t a, fp_K_raw_t b) {
#pragma HLS INLINE
  FP_WPROBE(FP_WP_MG_K_MUL, (__int128)a.to_int64() * (__int128)b.to_int64());
  fp_MG_K_mul_t product = (fp_MG_K_mul_t)a * (fp_MG_K_mul_t)b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp // latency = MPC_HLS_MUL_LATENCY
  return product;
}

fp_MG_K_mul_t fp_mul_K_MG(fp_K_raw_t a, fp_MG_raw_t b) {
#pragma HLS INLINE
  fp_MG_K_mul_t product = (fp_MG_K_mul_t)a * (fp_MG_K_mul_t)b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp // latency = MPC_HLS_MUL_LATENCY
  return product;
}

fp_K_QP_mul_t fp_mul_K_QP(fp_K_raw_t a, fp_QP_raw_t b) {
#pragma HLS INLINE
  FP_WPROBE(FP_WP_K_QP_MUL, (__int128)a.to_int64() * (__int128)b.to_int64());
  fp_K_QP_mul_t product = (fp_K_QP_mul_t)a * (fp_K_QP_mul_t)b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp // latency = MPC_HLS_MUL_LATENCY
  return product;
}

fp_K_QP_mul_t fp_mul_QP_K(fp_QP_raw_t a, fp_K_raw_t b) {
#pragma HLS INLINE
  fp_K_QP_mul_t product = (fp_K_QP_mul_t)a * (fp_K_QP_mul_t)b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp // latency = MPC_HLS_MUL_LATENCY
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
//#pragma HLS ALLOCATION function instances = fp_recip limit = 1
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
MPC_HLS_UNROLL()
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
  FP_WPROBE(FP_WP_QP_DET_MUL,
            (__int128)p00_raw.to_int64() - (__int128)p01_raw.to_int64());
  const fp_QP_det_mul_t det_mul_raw =
      (fp_QP_det_mul_t)p00_raw - (fp_QP_det_mul_t)p01_raw;
  const fp_QP_raw_t det_raw = (fp_QP_raw_t)(det_mul_raw >> FP_FRAC_BITS);

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
MPC_HLS_UNROLL()
    if (angle > FP_PI)
      angle = angle - FP_TWO_PI;
    if (angle < -FP_PI)
      angle = angle + FP_TWO_PI;
  }

  return angle;
}

static inline fp_FN_t fp_normalize_angle_fn(fp_FN_t angle) {
#pragma HLS INLINE
  for (int i = 0; i < 2; i++) {
MPC_HLS_UNROLL()
    if (angle > FP_FN_PI)
      angle = angle - FP_FN_TWO_PI;
    if (angle < -FP_FN_PI)
      angle = angle + FP_FN_TWO_PI;
  }

  return angle;
}

static inline int32_t fp_fn_trig_lut_pos_raw(fp_FN_t angle) {
#pragma HLS INLINE
  typedef ap_int<(MPC_HLS_FN_WIDTH + 27)> fp_fn_lut_mul_t;
  const fp_fn_lut_mul_t lut_scaled_raw =
      (fp_fn_lut_mul_t)fp_fn_raw_from_FN(angle) *
      (fp_fn_lut_mul_t)FP_FN_TRIG_LUT_SCALE_RAW;
#pragma HLS BIND_OP variable = lut_scaled_raw op = mul impl = dsp // latency = MPC_HLS_MUL_LATENCY
  return (int32_t)(lut_scaled_raw >> FP_FN_FRAC_BITS);
}

/*===========================================================================
 * Reciprocal: 1/x
 *===========================================================================*/
fp_QP_t fp_recip(fp_QP_t x) {
#pragma HLS INLINE off
  /* Feed-forward LUT+lerp; the win was LATENCY (NR removed: 13->8 cyc).
   * II is intentionally HIGH: measured II=1 changed zero caller cycles
   * (+4.3k LUT) -- callers never issue independent back-to-back reciprocals
   * (setup recip is hidden in the 97-cyc single-instance frenet engine;
   * backward recips are data-dependent links in the P-recurrence). High II
   * minimizes pipeline-register area since throughput is never exploited. */
MPC_HLS_PIPELINE(1)

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
MPC_HLS_UNROLL()
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

/* rom_2p: the lerp reads recip_lut[idx] AND recip_lut[idx+1] in the same
 * cycle -- a single-port ROM cannot serve two reads (HLS 200-882). Dual-port
 * BRAM is the same silicon (BRAM is natively 2-port), so this is free and is
 * the correct binding for an interpolated table -- atan_lut already does it. */
#pragma HLS BIND_STORAGE variable = recip_lut type = rom_2p impl = bram
  const ap_uint<MPC_HLS_QP_WIDTH> norm_raw_u = (ap_uint<MPC_HLS_QP_WIDTH>)x_norm_raw;
  const int lut_hi = FP_FRAC_BITS - 2;
  const int lut_lo = FP_FRAC_BITS - (FP_RECIP_LUT_BITS + 1);
  int lut_idx = (int)(norm_raw_u.range(lut_hi, lut_lo));
  if (lut_idx < 0)
    lut_idx = 0;
  if (lut_idx > (FP_RECIP_LUT_SIZE - 1))
    lut_idx = FP_RECIP_LUT_SIZE - 1;

  /* 1/x_norm by accurate LUT + linear interpolation, NO Newton-Raphson.
   * recip_lut[i] = 2^F / x_norm at x_norm=(L+i)/(2L), L=FP_RECIP_LUT_SIZE
   * (+1 guard for the lerp neighbour). Sub-grid weight = the lut_lo low
   * mantissa bits. 256 segments were measured to beat larger tables after
   * table-value quantization and integer lerp truncation are included. */
  const int frac = (int)(norm_raw_u.range(lut_lo - 1, 0));
  const fp_QP_raw_t v0 = (fp_QP_raw_t)recip_lut[lut_idx];
  const fp_QP_raw_t v1 = (fp_QP_raw_t)recip_lut[lut_idx + 1];
  const fp_QP_raw_t est_raw =
      (fp_QP_raw_t)(v0 + ((((long long)(v1 - v0)) * (long long)frac) >> lut_lo));

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
  const fp_QP_t angle_n = fp_normalize_angle(angle);
  const bool neg = (angle_n < 0);
  const fp_QP_t angle_u = neg ? fp_QP_t(-angle_n) : angle_n;

  const fp_QP_raw_t lut_pos_raw =
      fp_mul_QP_raw_q(fp_qp_raw_from_QP(angle_u), fp_qp_raw_from_QP(FP_TRIG_LUT_SCALE));
  int idx = (int)(lut_pos_raw >> FP_FRAC_BITS);
  if (idx >= FP_TRIG_LUT_SIZE)
    idx = FP_TRIG_LUT_SIZE - 1;
  if (idx < 0)
    idx = 0;

  const int idx_next = idx + 1;
  const fp_QP_raw_t frac_raw = lut_pos_raw - (((fp_QP_raw_t)idx) << FP_FRAC_BITS);
  const fp_QP_raw_t v0_raw = fp_qp_raw_from_QP(sin_lut[idx]);
  const fp_QP_raw_t v1_raw = fp_qp_raw_from_QP(sin_lut[idx_next]);
  const fp_QP_raw_t sin_raw = fp_lerp_qp_raw(v0_raw, v1_raw, frac_raw);
  return neg ? fp_QP_from_qp_raw((fp_QP_raw_t)(-sin_raw))
             : fp_QP_from_qp_raw(sin_raw);
}

fp_QP_t fp_cos(fp_QP_t angle) {
#pragma HLS INLINE off
#pragma HLS BIND_STORAGE variable = cos_lut type = rom_2p impl = bram
  const fp_QP_t angle_n = fp_normalize_angle(angle);
  const fp_QP_t angle_u = (angle_n < 0) ? fp_QP_t(-angle_n) : angle_n;

  const fp_QP_raw_t lut_pos_raw =
      fp_mul_QP_raw_q(fp_qp_raw_from_QP(angle_u), fp_qp_raw_from_QP(FP_TRIG_LUT_SCALE));
  int idx = (int)(lut_pos_raw >> FP_FRAC_BITS);
  if (idx >= FP_TRIG_LUT_SIZE)
    idx = FP_TRIG_LUT_SIZE - 1;
  if (idx < 0)
    idx = 0;

  const int idx_next = idx + 1;
  const fp_QP_raw_t frac_raw = lut_pos_raw - (((fp_QP_raw_t)idx) << FP_FRAC_BITS);
  const fp_QP_raw_t v0_raw = fp_qp_raw_from_QP(cos_lut[idx]);
  const fp_QP_raw_t v1_raw = fp_qp_raw_from_QP(cos_lut[idx_next]);
  return fp_QP_from_qp_raw(fp_lerp_qp_raw(v0_raw, v1_raw, frac_raw));
}

void fp_trig_pair_fused(fp_QP_t angle, fp_QP_t *sin_out, fp_QP_t *cos_out) {
#pragma HLS INLINE off
#pragma HLS BIND_STORAGE variable = sin_lut type = rom_2p impl = bram
#pragma HLS BIND_STORAGE variable = cos_lut type = rom_2p impl = bram
  const fp_QP_t angle_n = fp_normalize_angle(angle);
  const bool neg = (angle_n < 0);
  const fp_QP_t angle_u = neg ? fp_QP_t(-angle_n) : angle_n;

  const fp_QP_raw_t lut_pos_raw =
      fp_mul_QP_raw_q(fp_qp_raw_from_QP(angle_u), fp_qp_raw_from_QP(FP_TRIG_LUT_SCALE));
  int idx = (int)(lut_pos_raw >> FP_FRAC_BITS);
  if (idx >= FP_TRIG_LUT_SIZE)
    idx = FP_TRIG_LUT_SIZE - 1;
  if (idx < 0)
    idx = 0;

  const int idx_next = idx + 1;
  const fp_QP_raw_t frac_raw = lut_pos_raw - (((fp_QP_raw_t)idx) << FP_FRAC_BITS);

  const fp_QP_raw_t sin_v0_raw = fp_qp_raw_from_QP(sin_lut[idx]);
  const fp_QP_raw_t sin_v1_raw = fp_qp_raw_from_QP(sin_lut[idx_next]);
  const fp_QP_raw_t sin_raw = fp_lerp_qp_raw(sin_v0_raw, sin_v1_raw, frac_raw);
  *sin_out = neg ? fp_QP_from_qp_raw((fp_QP_raw_t)(-sin_raw))
                 : fp_QP_from_qp_raw(sin_raw);

  const fp_QP_raw_t cos_v0_raw = fp_qp_raw_from_QP(cos_lut[idx]);
  const fp_QP_raw_t cos_v1_raw = fp_qp_raw_from_QP(cos_lut[idx_next]);
  *cos_out = fp_QP_from_qp_raw(fp_lerp_qp_raw(cos_v0_raw, cos_v1_raw, frac_raw));
}

fp_QP_t fp_atan_lut(fp_QP_t x) {
#pragma HLS INLINE off
#pragma HLS BIND_STORAGE variable = atan_lut type = rom_2p impl = bram
  /* Direct [0, FP_ATAN_LUT_DOMAIN] LUT+lerp. No reciprocal range-reduction:
   * every argument here is provably < FP_ATAN_LUT_DOMAIN (see fp_math_hls.h).
   * |x| beyond the domain saturates at idx=1023 -> atan(domain) ~= pi/2 dir
   * (graceful, never hit under the proven bounds). atan is odd -> sign folded.
   * The constant /FP_ATAN_LUT_DOMAIN lowers to a multiply+shift, NOT a divider
   * or fp_recip, so this function no longer instantiates a reciprocal. */
  const bool neg = (x < 0);
  const fp_QP_t abs_x = fp_abs(x);
  const fp_QP_raw_t y_raw = fp_qp_raw_from_QP(abs_x);
  const fp_QP_mul_t lut_pos_wide =
      (((fp_QP_mul_t)y_raw) << 10) >> FP_ATAN_LUT_DOMAIN_LOG2;
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
  return neg ? fp_QP_from_qp_raw((fp_QP_raw_t)(-atan_y_raw))
             : fp_QP_from_qp_raw(atan_y_raw);
}

/*===========================================================================
 * FN family
 *===========================================================================*/

#include "../include/fp_trig_lut_fn_1024.h"

fp_FN_t fp_mul_fn(fp_FN_t a, fp_FN_t b) {
#pragma HLS INLINE
  FP_WPROBE(FP_WP_FN_MUL,
            (__int128)fp_fn_raw_from_FN(a).to_int64() *
                (__int128)fp_fn_raw_from_FN(b).to_int64());
  fp_fn_accum_t product = (fp_fn_accum_t)fp_fn_raw_from_FN(a) *
                          (fp_fn_accum_t)fp_fn_raw_from_FN(b);
#pragma HLS BIND_OP variable = product op = mul impl = dsp // latency = MPC_HLS_MUL_LATENCY
  fp_fn_raw_t product_q = (fp_fn_raw_t)(product >> FP_FN_FRAC_BITS);
  return fp_FN_from_fn_raw(product_q);
}

fp_fn_accum_t fp_mul_fn_raw(fp_FN_t a, fp_FN_t b) {
#pragma HLS INLINE
  FP_WPROBE(FP_WP_FN_MUL,
            (__int128)fp_fn_raw_from_FN(a).to_int64() *
                (__int128)fp_fn_raw_from_FN(b).to_int64());
  fp_fn_accum_t product = (fp_fn_accum_t)fp_fn_raw_from_FN(a) *
                          (fp_fn_accum_t)fp_fn_raw_from_FN(b);
#pragma HLS BIND_OP variable = product op = mul impl = dsp // latency = MPC_HLS_MUL_LATENCY
  return product;
}

fp_FN_t fp_sin_fn(fp_FN_t angle) {
#pragma HLS INLINE off
#pragma HLS BIND_STORAGE variable = sin_lut_fn type = rom_2p impl = bram
  const fp_FN_t angle_n = fp_normalize_angle_fn(angle);
  const bool neg = (angle_n < FP_FN_ZERO);
  const fp_FN_t angle_u = neg ? fp_FN_t(-angle_n) : angle_n;

  const int32_t lut_pos_raw = fp_fn_trig_lut_pos_raw(angle_u);
  int idx = (int)(lut_pos_raw >> FP_FN_FRAC_BITS);
  if (idx >= FP_TRIG_LUT_SIZE)
    idx = FP_TRIG_LUT_SIZE - 1;
  if (idx < 0)
    idx = 0;

  const int idx_next = idx + 1;
  const fp_fn_raw_t frac_raw =
      (fp_fn_raw_t)(lut_pos_raw - (((int32_t)idx) << FP_FN_FRAC_BITS));
  const fp_fn_raw_t v0_raw = fp_fn_raw_from_FN(sin_lut_fn[idx]);
  const fp_fn_raw_t v1_raw = fp_fn_raw_from_FN(sin_lut_fn[idx_next]);
  const fp_fn_raw_t sin_raw = fp_lerp_fn_raw(v0_raw, v1_raw, frac_raw);
  return neg ? fp_FN_from_fn_raw((fp_fn_raw_t)(-sin_raw))
             : fp_FN_from_fn_raw(sin_raw);
}

fp_FN_t fp_cos_fn(fp_FN_t angle) {
#pragma HLS INLINE off
#pragma HLS BIND_STORAGE variable = cos_lut_fn type = rom_2p impl = bram
  const fp_FN_t angle_n = fp_normalize_angle_fn(angle);
  const fp_FN_t angle_u = (angle_n < FP_FN_ZERO) ? fp_FN_t(-angle_n) : angle_n;

  const int32_t lut_pos_raw = fp_fn_trig_lut_pos_raw(angle_u);
  int idx = (int)(lut_pos_raw >> FP_FN_FRAC_BITS);
  if (idx >= FP_TRIG_LUT_SIZE)
    idx = FP_TRIG_LUT_SIZE - 1;
  if (idx < 0)
    idx = 0;

  const int idx_next = idx + 1;
  const fp_fn_raw_t frac_raw =
      (fp_fn_raw_t)(lut_pos_raw - (((int32_t)idx) << FP_FN_FRAC_BITS));
  const fp_fn_raw_t v0_raw = fp_fn_raw_from_FN(cos_lut_fn[idx]);
  const fp_fn_raw_t v1_raw = fp_fn_raw_from_FN(cos_lut_fn[idx_next]);
  return fp_FN_from_fn_raw(fp_lerp_fn_raw(v0_raw, v1_raw, frac_raw));
}

void fp_trig_pair_fused_fn(fp_FN_t angle, fp_FN_t *sin_out, fp_FN_t *cos_out) {
#pragma HLS INLINE off
#pragma HLS BIND_STORAGE variable = sin_lut_fn type = rom_2p impl = bram
#pragma HLS BIND_STORAGE variable = cos_lut_fn type = rom_2p impl = bram
  const fp_FN_t angle_n = fp_normalize_angle_fn(angle);
  const bool neg = (angle_n < FP_FN_ZERO);
  const fp_FN_t angle_u = neg ? fp_FN_t(-angle_n) : angle_n;

  const int32_t lut_pos_raw = fp_fn_trig_lut_pos_raw(angle_u);
  int idx = (int)(lut_pos_raw >> FP_FN_FRAC_BITS);
  if (idx >= FP_TRIG_LUT_SIZE)
    idx = FP_TRIG_LUT_SIZE - 1;
  if (idx < 0)
    idx = 0;

  const int idx_next = idx + 1;
  const fp_fn_raw_t frac_raw =
      (fp_fn_raw_t)(lut_pos_raw - (((int32_t)idx) << FP_FN_FRAC_BITS));

  const fp_fn_raw_t sin_v0_raw = fp_fn_raw_from_FN(sin_lut_fn[idx]);
  const fp_fn_raw_t sin_v1_raw = fp_fn_raw_from_FN(sin_lut_fn[idx_next]);
  const fp_fn_raw_t sin_raw = fp_lerp_fn_raw(sin_v0_raw, sin_v1_raw, frac_raw);
  *sin_out = neg ? fp_FN_from_fn_raw((fp_fn_raw_t)(-sin_raw))
                 : fp_FN_from_fn_raw(sin_raw);

  const fp_fn_raw_t cos_v0_raw = fp_fn_raw_from_FN(cos_lut_fn[idx]);
  const fp_fn_raw_t cos_v1_raw = fp_fn_raw_from_FN(cos_lut_fn[idx_next]);
  *cos_out = fp_FN_from_fn_raw(fp_lerp_fn_raw(cos_v0_raw, cos_v1_raw, frac_raw));
}

fp_FN_t fp_atan_lut_fn(fp_FN_t x) {
#pragma HLS INLINE off
#pragma HLS BIND_STORAGE variable = atan_lut_fn type = rom_2p impl = bram
  /* Direct [0, FP_ATAN_LUT_DOMAIN] LUT+lerp; no reciprocal range-reduction.
   * Arguments here (slip ratios, Pacejka B*alpha) are provably inside the
   * domain (see fp_math_hls.h). |x| beyond it saturates at idx=1023. The
   * constant /FP_ATAN_LUT_DOMAIN is a multiply+shift, so this function no
   * longer instantiates fp_recip_fn -- freeing that contended unit from the
   * tire path. 64-bit scaled intermediate: y_raw<<10 can exceed int32. */
  const bool neg = (x < FP_FN_ZERO);
  const fp_FN_t abs_x = neg ? fp_FN_t(-x) : x;
  const fp_fn_raw_t y_raw = fp_fn_raw_from_FN(abs_x);

  const long long scaled =
      (((long long)y_raw) << 10) >> FP_ATAN_LUT_DOMAIN_LOG2;
  int idx = (int)(scaled >> FP_FN_FRAC_BITS);
  if (idx < 0)
    idx = 0;
  if (idx > 1023)
    idx = 1023;

  const fp_fn_raw_t frac_raw =
      (fp_fn_raw_t)(scaled - (((long long)idx) << FP_FN_FRAC_BITS));
  const fp_fn_raw_t v0_raw = fp_fn_raw_from_FN(atan_lut_fn[idx]);
  const fp_fn_raw_t v1_raw = fp_fn_raw_from_FN(atan_lut_fn[idx + 1]);
  const fp_fn_raw_t atan_y_raw = fp_lerp_fn_raw(v0_raw, v1_raw, frac_raw);
  return neg ? fp_FN_from_fn_raw((fp_fn_raw_t)(-atan_y_raw))
             : fp_FN_from_fn_raw(atan_y_raw);
}

fp_FN_t fp_recip_fn(fp_FN_t x) {
#pragma HLS INLINE off
  /* Same as fp_recip (QP): II kept high on purpose. The ~7 tire reciprocals
   * look independent but the tire block's critical path is slip->atan->
   * Pacejka force, not recip throughput -- compute_frenet_tire_hls stayed
   * 70 cyc at both II=21 and II=2. Low II only added LUT. */
MPC_HLS_PIPELINE(1)
/* rom_2p: lerp reads recip_lut_fn[idx] and [idx+1] same cycle (see fp_recip
 * QP note). Single-port ROM triggers HLS 200-882; dual-port BRAM is free. */
#pragma HLS BIND_STORAGE variable = recip_lut_fn type = rom_2p impl = bram
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
MPC_HLS_UNROLL()
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
  const int lut_lo = FP_FN_FRAC_BITS - (FP_RECIP_LUT_BITS + 1);
  int lut_idx = (int)(norm_raw_u.range(lut_hi, lut_lo));
  if (lut_idx < 0)
    lut_idx = 0;
  if (lut_idx > (FP_RECIP_LUT_SIZE - 1))
    lut_idx = FP_RECIP_LUT_SIZE - 1;

  /* 1/x_norm by accurate LUT + linear interpolation, NO Newton-Raphson.
   * recip_lut_fn[i] = 2^F / x_norm at x_norm=(L+i)/(2L), L=FP_RECIP_LUT_SIZE.
   * 256 segments were measured to beat larger tables after table-value
   * quantization and integer lerp truncation are included. */
  const int frac = (int)(norm_raw_u.range(lut_lo - 1, 0));
  const fp_fn_raw_t v0 = (fp_fn_raw_t)recip_lut_fn[lut_idx];
  const fp_fn_raw_t v1 = (fp_fn_raw_t)recip_lut_fn[lut_idx + 1];
  const fp_fn_raw_t est_raw =
      (fp_fn_raw_t)(v0 + ((((long long)(v1 - v0)) * (long long)frac) >> lut_lo));

  const fp_fn_raw_t est_denorm_raw = fp_shift_fn_raw_cast_sel(est_raw, shift);
  return neg ? fp_FN_from_fn_raw((fp_fn_raw_t)(-est_denorm_raw))
             : fp_FN_from_fn_raw(est_denorm_raw);
}
