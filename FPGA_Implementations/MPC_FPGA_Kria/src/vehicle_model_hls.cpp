/**
 * @file vehicle_model_hls.cpp
 * @brief Frenet-Frame Vehicle Model Linearization for HLS
 * @details Computes the 5x5 Frenet A matrix and 5x2 B matrix used by the
 *          fixed-horizon FPGA MPC controller. The model includes load
 *          transfer, slip-angle Jacobians, and Pacejka-inspired effective
 *          stiffness scaling with fixed-point arithmetic.
 * @dependencies fp_math_hls.h, mpc_fpga_types.h
 */

#include "../include/fp_math_hls.h"
#include "../include/mpc_fpga_types.h"

/* FN-local vehicle constants for the vehicle model hot path. */
#define FN_MPC_DT FP_FN_CONST(MPC_FPGA_PREDICTION_DT_S)
#define VP_FN_LF FP_FN_CONST(MPC_FPGA_LF_M)
#define VP_FN_LR FP_FN_CONST(MPC_FPGA_LR_M)
#define VP_FN_MASS FP_FN_CONST(MPC_FPGA_MASS_KG)
#define VP_FN_INV_MASS FP_FN_CONST(MPC_FPGA_INV_MASS)
#define VP_FN_INV_IZ FP_FN_CONST(MPC_FPGA_INV_IZ)

/* DT-scaled inverses used in A/B matrix assembly (matching baseline QP
 * VP_DT_INV_*) */
#define VP_FN_DT_INV_MASS FP_FN_CONST(MPC_FPGA_PREDICTION_DT_S *MPC_FPGA_INV_MASS)
#define VP_FN_DT_INV_IZ FP_FN_CONST(MPC_FPGA_PREDICTION_DT_S *MPC_FPGA_INV_IZ)
#define VP_FN_NEG_DT_INV_MASS FP_FN_CONST(-MPC_FPGA_PREDICTION_DT_S *MPC_FPGA_INV_MASS)
#define VP_FN_C_SHAPE FP_FN_CONST(MPC_FPGA_PACEJKA_C_SHAPE)
#define VP_FN_FZ_FRONT FP_FN_CONST(MPC_FPGA_FZ_FRONT_N)
#define VP_FN_FZ_REAR FP_FN_CONST(MPC_FPGA_FZ_REAR_N)
#define VP_FN_D_FRONT FP_FN_CONST(MPC_FPGA_D_FRONT_N)
#define VP_FN_D_REAR FP_FN_CONST(MPC_FPGA_D_REAR_N)
#define VP_FN_FZ_LOAD_GAIN FP_FN_CONST(MPC_FPGA_MASS_KG * MPC_FPGA_CG_HEIGHT_M * MPC_FPGA_INV_WHEELBASE)
#define VP_FN_D_LOAD_GAIN                                                      \
  FP_FN_CONST(MPC_FPGA_MU * MPC_FPGA_MASS_KG * MPC_FPGA_CG_HEIGHT_M *          \
              MPC_FPGA_INV_WHEELBASE)
#define VP_FN_CMIN_FRONT_STATIC                                                \
  FP_FN_CONST(MPC_FPGA_C_ALPHA_F_N_PER_RAD *MPC_FPGA_MIN_STIFF_SCALE)
#define VP_FN_CMIN_REAR_STATIC                                                 \
  FP_FN_CONST(MPC_FPGA_C_ALPHA_R_N_PER_RAD *MPC_FPGA_MIN_STIFF_SCALE)
#define VP_FN_CMIN_FRONT_LOAD_GAIN                                             \
  FP_FN_CONST(MPC_FPGA_MASS_KG * MPC_FPGA_CG_HEIGHT_M *                        \
              MPC_FPGA_INV_WHEELBASE * MPC_FPGA_MU *                           \
              MPC_FPGA_C_ALPHA_SF_NORM * MPC_FPGA_MIN_STIFF_SCALE)
#define VP_FN_CMIN_REAR_LOAD_GAIN                                              \
  FP_FN_CONST(MPC_FPGA_MASS_KG * MPC_FPGA_CG_HEIGHT_M *                        \
              MPC_FPGA_INV_WHEELBASE * MPC_FPGA_MU *                           \
              MPC_FPGA_C_ALPHA_SR_NORM * MPC_FPGA_MIN_STIFF_SCALE)
#define VP_FN_CB_FRONT FP_FN_CONST(MPC_FPGA_C_ALPHA_SF_NORM)
#define VP_FN_CB_REAR FP_FN_CONST(MPC_FPGA_C_ALPHA_SR_NORM)

static fp_FN_t fp_pacejka_inner_arg(fp_FN_t ba) {
#pragma HLS INLINE
  /* FN-precision arctangent and multiply. */
  fp_FN_t atan_ba_fn = fp_atan_lut_fn(ba);
  return fp_mul_fn(VP_FN_C_SHAPE, atan_ba_fn);
}

/* Breaks the 2-deep multiply chain:  cos_inner * inv_denom → D_cb * result.
 * With INLINE off, HLS creates a module boundary with registered I/Os,
 * preventing the combinational DSP-to-DSP path that caused -0.14ns
 * timing violation in compute_frenet_AB_hls. */
static fp_FN_t fp_pacejka_ceff(fp_FN_t cos_inner, fp_FN_t inv_denom,
                               fp_FN_t D_cb) {
#pragma HLS INLINE off
  fp_FN_t cos_over_denom = fp_mul_fn(cos_inner, inv_denom);
  return fp_mul_fn(D_cb, cos_over_denom);
}

static void fp_trig_pair(fp_FN_t angle, fp_FN_t *sin_val, fp_FN_t *cos_val) {
#pragma HLS INLINE off
#pragma HLS PIPELINE II = 1
  *sin_val = fp_sin_fn(angle);
  *cos_val = fp_cos_fn(angle);
}

/* Step 1 of the Frenet rows 0/1 critical path split.
 * Returns 1/denom with INLINE off so HLS registers the output
 * before the multiply fan-out that follows. */
static fp_FN_t fp_frenet_recip(fp_FN_t kappa, fp_FN_t ey) {
#pragma HLS INLINE
  fp_FN_t denom = FP_FN_ONE - fp_mul_fn(kappa, ey);
  if (fp_abs_fn(denom) < FP_FN_CONST(1e-3)) {
    denom = (denom >= 0) ? FP_FN_CONST(1e-3) : FP_FN_CONST(-1e-3);
  }
  return fp_recip_fn(denom);
}

/* Step 2: receives registered inv_denom, computes all A rows 0/1 entries.
 * Separating this from fp_frenet_recip() ensures HLS schedules the
 * multiply fan-out (inv_denom2, A10, A11, A12) in states AFTER fp_recip_fn
 * completes, preventing the -0.12ns combinational path. */
static void fp_frenet_rows01_fn(fp_FN_t sin_epsi, fp_FN_t cos_epsi, fp_FN_t vx,
                                fp_FN_t vy, fp_FN_t kappa, fp_FN_t ey,
                                fp_FN_t inv_denom, fp_FN_t reference_velocity,
                                fp_FN_t *A00, fp_FN_t *A01, fp_FN_t *A02,
                                fp_FN_t *A03, fp_FN_t *A10, fp_FN_t *A11,
                                fp_FN_t *A12) {
#pragma HLS INLINE off
  (void)vx;
  (void)ey;
  fp_FN_t dt_cp = fp_mul_fn(FN_MPC_DT, cos_epsi);
  fp_FN_t kappa_dt = fp_mul_fn(kappa, FN_MPC_DT);
  fp_FN_t kappa2 = fp_mul_fn(kappa, kappa);
  fp_FN_t vx_cp = fp_mul_fn(reference_velocity, cos_epsi);
  fp_FN_t vy_sp = fp_mul_fn(vy, sin_epsi);
  *A00 = FP_FN_ONE;
  *A01 = fp_mul_fn(FN_MPC_DT, vx_cp - vy_sp);
  *A02 = fp_mul_fn(FN_MPC_DT, sin_epsi);
  *A03 = dt_cp;
  /* With MUL_LATENCY=4, every fp_mul() has an extra FF stage — the
   * inv_denom → inv_denom2 → A10 chain is automatically broken. */
  fp_FN_t inv_denom2 = fp_mul_fn(inv_denom, inv_denom);
  fp_FN_t k2_vx = fp_mul_fn(kappa2, reference_velocity);
  fp_FN_t a10_pre = fp_mul_fn(dt_cp, k2_vx);
  *A10 = fp_mul_fn(-a10_pre, inv_denom2);
  fp_FN_t kdt_vx = fp_mul_fn(kappa_dt, reference_velocity);
  fp_FN_t a11_pre = fp_mul_fn(kdt_vx, sin_epsi);
  *A11 = FP_FN_ONE + fp_mul_fn(a11_pre, inv_denom);
  fp_FN_t a12_pre = fp_mul_fn(kappa_dt, cos_epsi);
  *A12 = fp_mul_fn(-a12_pre, inv_denom);
}

static void fp_slip_terms(fp_FN_t vy, fp_FN_t omega, fp_FN_t inv_vx,
                          fp_FN_t *lf_omega, fp_FN_t *lr_omega,
                          fp_FN_t *front_num, fp_FN_t *rear_num,
                          fp_FN_t *front_ratio, fp_FN_t *rear_ratio) {
#pragma HLS INLINE off
#pragma HLS PIPELINE II = 1
  fp_FN_t lf_omega_local = fp_mul_fn(VP_FN_LF, omega);
  fp_FN_t lr_omega_local = fp_mul_fn(VP_FN_LR, omega);
  fp_FN_t front_num_local = vy + lf_omega_local;
  fp_FN_t rear_num_local = vy - lr_omega_local;

  *lf_omega = lf_omega_local;
  *lr_omega = lr_omega_local;
  *front_num = front_num_local;
  *rear_num = rear_num_local;
  *front_ratio = fp_mul_fn(front_num_local, inv_vx);
  *rear_ratio = fp_mul_fn(rear_num_local, inv_vx);
}

/* FN variants of the force-jacobian helpers that operate in FN precision. */
static void fp_front_force_jacobians_fn(fp_FN_t C_eff_f, fp_FN_t front_num,
                                        fp_FN_t vx_safe, fp_FN_t inv_D_f,
                                        fp_FN_t *dFyf_dvx, fp_FN_t *dFyf_dvy,
                                        fp_FN_t *dFyf_dom) {
#pragma HLS INLINE off
  fp_FN_t vx_inv = fp_mul_fn(vx_safe, inv_D_f);
  fp_FN_t daf_dvx = fp_mul_fn(front_num, inv_D_f);
  fp_FN_t daf_dvy = -vx_inv;
  fp_FN_t lf_vx = fp_mul_fn(VP_FN_LF, vx_safe);
  fp_FN_t daf_dom = -fp_mul_fn(lf_vx, inv_D_f);
  *dFyf_dvx = fp_mul_fn(C_eff_f, daf_dvx);
  *dFyf_dvy = fp_mul_fn(C_eff_f, daf_dvy);
  *dFyf_dom = fp_mul_fn(C_eff_f, daf_dom);
}

static void fp_rear_force_jacobians_fn(fp_FN_t C_eff_r, fp_FN_t rear_num,
                                       fp_FN_t vx_safe, fp_FN_t inv_D_r,
                                       fp_FN_t *dFyr_dvx, fp_FN_t *dFyr_dvy,
                                       fp_FN_t *dFyr_dom) {
#pragma HLS INLINE off
  fp_FN_t vx_inv = fp_mul_fn(vx_safe, inv_D_r);
  fp_FN_t dar_dvx = fp_mul_fn(rear_num, inv_D_r);
  fp_FN_t dar_dvy = -vx_inv;
  fp_FN_t dar_dom = fp_mul_fn(VP_FN_LR, vx_inv);
  *dFyr_dvx = fp_mul_fn(C_eff_r, dar_dvx);
  *dFyr_dvy = fp_mul_fn(C_eff_r, dar_dvy);
  *dFyr_dom = fp_mul_fn(C_eff_r, dar_dom);
}

static void fp_rollout_from_forces_fn(
    fp_FN_t ey, fp_FN_t epsi, fp_FN_t sin_epsi, fp_FN_t cos_epsi,
    fp_FN_t vx_safe, fp_FN_t v_frenet, fp_FN_t vy, fp_FN_t omega, fp_FN_t kappa,
    fp_FN_t dt, fp_FN_t Fx, fp_FN_t F_yf, fp_FN_t F_yr, fp_FN_t sin_delta,
    fp_FN_t cos_delta, fp_FN_t next_state_fn[MPC_NX_FRENET]) {
#pragma HLS INLINE
  fp_FN_t ey_denom = FP_FN_ONE - fp_mul_fn(kappa, ey);
  if (fp_abs_fn(ey_denom) < FP_FN_CONST(1e-3)) {
    ey_denom = (ey_denom >= 0) ? FP_FN_CONST(1e-3) : FP_FN_CONST(-1e-3);
  }
  fp_FN_t inv_ey_denom = fp_recip_fn(ey_denom);

  fp_FN_t vx_sin_epsi = fp_mul_fn(v_frenet, sin_epsi);
  fp_FN_t vy_cos_epsi = fp_mul_fn(vy, cos_epsi);
  fp_FN_t vx_cos_epsi = fp_mul_fn(v_frenet, cos_epsi);
  fp_FN_t kappa_vx_cos = fp_mul_fn(kappa, vx_cos_epsi);
  fp_FN_t kappa_vx_cos_inv = fp_mul_fn(kappa_vx_cos, inv_ey_denom);
  fp_FN_t fx_sin_delta = fp_mul_fn(F_yf, sin_delta);
  fp_FN_t fyf_cos = fp_mul_fn(F_yf, cos_delta);
  fp_FN_t dvy_term = fyf_cos + F_yr;
  fp_FN_t vx_omega = fp_mul_fn(vx_safe, omega);
  fp_FN_t vy_omega = fp_mul_fn(vy, omega);
  fp_FN_t domega_lf = fp_mul_fn(VP_FN_LF, fyf_cos);
  fp_FN_t domega_lr = fp_mul_fn(VP_FN_LR, F_yr);
  fp_FN_t dvx_dt = fp_mul_fn((Fx - fx_sin_delta), VP_FN_INV_MASS) + vy_omega;
  fp_FN_t dvy_dt = fp_mul_fn(dvy_term, VP_FN_INV_MASS) - vx_omega;
  fp_FN_t domega_dt = fp_mul_fn((domega_lf - domega_lr), VP_FN_INV_IZ);
  fp_FN_t e_y_step_vx = fp_mul_fn(dt, vx_sin_epsi);
  fp_FN_t e_y_step_vy = fp_mul_fn(dt, vy_cos_epsi);
  fp_FN_t e_psi_step_omega = fp_mul_fn(dt, omega);
  fp_FN_t e_psi_step_corr = fp_mul_fn(dt, kappa_vx_cos_inv);
  fp_FN_t dvx_step = fp_mul_fn(dt, dvx_dt);
  fp_FN_t dvy_step = fp_mul_fn(dt, dvy_dt);
  fp_FN_t domega_step = fp_mul_fn(dt, domega_dt);

  next_state_fn[0] = (ey + e_y_step_vx + e_y_step_vy);
  next_state_fn[1] = (epsi + e_psi_step_omega - e_psi_step_corr);
  next_state_fn[2] = (vx_safe + dvx_step);
  if (next_state_fn[2] < MIN_LIN_VEL)
    next_state_fn[2] = MIN_LIN_VEL;
  next_state_fn[3] = (vy + dvy_step);
  next_state_fn[4] = (omega + domega_step);
}

/* Compute Frenet-frame linearized dynamics (split into two sequential
 * sub-functions to halve the FSM state count and reduce the HLS critical path).
 */
/* -----------------------------------------------------------------------
 * Tire physics intermediate results, passed from compute_frenet_tire_hls
 * to compute_frenet_AB_hls.  Using a plain struct avoids pointer-to-array
 * address arithmetic that would add extra FSM states.
 * ----------------------------------------------------------------------- */
struct FpTireResults {
  /* Internal tire results stored in FN precision to match vehicle-model
   * bit widths and reduce QP<->FN conversions. */
  fp_FN_t C_eff_f, C_eff_f_raw, C_eff_r, C_min_f, C_min_r;
  fp_FN_t F_yf, F_yr;
  fp_FN_t dFyf_dvx, dFyf_dvy, dFyf_dom;
  fp_FN_t dFyr_dvx, dFyr_dvy, dFyr_dom;
  fp_FN_t sin_delta, cos_delta;
  fp_FN_t alpha_f_op, alpha_r_op;
  fp_FN_t inv_vx;
  fp_FN_t D_transfer;
  fp_FN_t Fz_transfer;
  fp_FN_t vx_safe;
};

/* -----------------------------------------------------------------------
 * Half 1 — Tire physics and Jacobians (~40 FSM states).
 * Inputs:  raw vehicle state + control.
 * Outputs: FpTireResults struct (all tire forces and their Jacobians).
 * Splitting here removes 10 sub-module handshake state groups from the
 * original compute_frenet_AB_hls FSM, cutting the next-state logic from
 * 458 LUT → ~220 LUT and the critical path from 4.684ns → ~3.5ns.
 * ----------------------------------------------------------------------- */
static void compute_frenet_tire_hls(fp_QP_t vx, fp_QP_t vy, fp_QP_t omega,
                                    fp_QP_t delta, fp_QP_t a_cmd,
                                    FpTireResults &tr) {
#pragma HLS INLINE off
#pragma HLS ALLOCATION function instances = fp_atan_lut_fn limit = 2
#pragma HLS ALLOCATION function instances = fp_recip_fn limit = 1
#pragma HLS ALLOCATION function instances = fp_trig_pair limit = 1
  tr.vx_safe = fp_FN_from_QP((vx > MIN_LIN_VEL) ? vx : MIN_LIN_VEL);
  tr.inv_vx = fp_recip_fn(tr.vx_safe);

  fp_FN_t vy_fn = fp_FN_from_QP(vy);
  fp_FN_t omega_fn = fp_FN_from_QP(omega);
  fp_FN_t delta_fn = fp_FN_from_QP(delta);
  fp_FN_t a_cmd_fn = fp_FN_from_QP(a_cmd);

  /* LUT-based trig in FN (accurate for all delta values, matches CPU model). */
  fp_trig_pair(delta_fn, &tr.sin_delta, &tr.cos_delta);

  /* Load transfer in FN. */
  tr.Fz_transfer = fp_mul_fn(a_cmd_fn, VP_FN_FZ_LOAD_GAIN);
  tr.D_transfer = fp_mul_fn(a_cmd_fn, VP_FN_D_LOAD_GAIN);
  tr.C_min_f =
      VP_FN_CMIN_FRONT_STATIC - fp_mul_fn(a_cmd_fn, VP_FN_CMIN_FRONT_LOAD_GAIN);
  tr.C_min_r =
      VP_FN_CMIN_REAR_STATIC + fp_mul_fn(a_cmd_fn, VP_FN_CMIN_REAR_LOAD_GAIN);

  fp_FN_t D_pac_f = VP_FN_D_FRONT - tr.D_transfer;
  fp_FN_t D_pac_r = VP_FN_D_REAR + tr.D_transfer;

  /* Slip terms in FN. */
  fp_FN_t lf_omega, lr_omega, front_num, rear_num;
  fp_FN_t front_ratio, rear_ratio;
  fp_slip_terms(vy_fn, omega_fn, tr.inv_vx, &lf_omega, &lr_omega, &front_num,
                &rear_num, &front_ratio, &rear_ratio);

  fp_FN_t vx2 = fp_mul_fn(tr.vx_safe, tr.vx_safe);
  fp_FN_t vy2 = fp_mul_fn(vy_fn, vy_fn);
  fp_FN_t lf_omega2 = fp_mul_fn(lf_omega, lf_omega);
  fp_FN_t lr_omega2 = fp_mul_fn(lr_omega, lr_omega);
  fp_FN_t front_cross = fp_mul_fn(vy_fn, lf_omega);
  fp_FN_t rear_cross = fp_mul_fn(vy_fn, lr_omega);
  fp_FN_t front_num2 = vy2 + (front_cross + front_cross) + lf_omega2;
  fp_FN_t rear_num2 = vy2 - (rear_cross + rear_cross) + lr_omega2;
  fp_FN_t D_f_fn = vx2 + front_num2;
  fp_FN_t D_r_fn = vx2 + rear_num2;

  /* Slip angles and Pacejka in FN. */
  tr.alpha_f_op = delta_fn - fp_atan_lut_fn(front_ratio);
  tr.alpha_r_op = -fp_atan_lut_fn(rear_ratio);

  fp_FN_t Ba_f_fn = fp_mul_fn(FP_FN_CONST(MPC_FPGA_B_FRONT), tr.alpha_f_op);
  fp_FN_t inner_f_fn = fp_pacejka_inner_arg(Ba_f_fn);
  fp_FN_t sin_inner_f, cos_inner_f;
  fp_trig_pair(inner_f_fn, &sin_inner_f, &cos_inner_f);
  fp_FN_t inv_denom_f_fn = fp_recip_fn(FP_FN_ONE + fp_mul_fn(Ba_f_fn, Ba_f_fn));
  fp_FN_t D_pac_f_cb_fn = fp_mul_fn(VP_FN_D_FRONT, VP_FN_CB_FRONT) +
                          (-fp_mul_fn(tr.D_transfer, VP_FN_CB_FRONT));
  tr.C_eff_f_raw = fp_pacejka_ceff(cos_inner_f, inv_denom_f_fn, D_pac_f_cb_fn);
  tr.F_yf = fp_mul_fn(D_pac_f, sin_inner_f);
  tr.C_eff_f = (tr.C_eff_f_raw > tr.C_min_f) ? tr.C_eff_f_raw : tr.C_min_f;

  fp_FN_t Ba_r_fn = fp_mul_fn(FP_FN_CONST(MPC_FPGA_B_REAR), tr.alpha_r_op);
  fp_FN_t inner_r_fn = fp_pacejka_inner_arg(Ba_r_fn);
  fp_FN_t sin_inner_r, cos_inner_r;
  fp_trig_pair(inner_r_fn, &sin_inner_r, &cos_inner_r);
  fp_FN_t inv_denom_r_fn = fp_recip_fn(FP_FN_ONE + fp_mul_fn(Ba_r_fn, Ba_r_fn));
  fp_FN_t D_pac_r_cb_fn = fp_mul_fn(VP_FN_D_REAR, VP_FN_CB_REAR) +
                          fp_mul_fn(tr.D_transfer, VP_FN_CB_REAR);
  tr.C_eff_r = fp_pacejka_ceff(cos_inner_r, inv_denom_r_fn, D_pac_r_cb_fn);
  tr.F_yr = fp_mul_fn(D_pac_r, sin_inner_r);
  tr.C_eff_r = (tr.C_eff_r > tr.C_min_r) ? tr.C_eff_r : tr.C_min_r;

  /* Force Jacobians in FN. */
  fp_FN_t dFyf_dvx_fn, dFyf_dvy_fn, dFyf_dom_fn;
  fp_FN_t dFyr_dvx_fn, dFyr_dvy_fn, dFyr_dom_fn;
  fp_FN_t inv_D_f_fn = fp_recip_fn(D_f_fn);
  fp_FN_t inv_D_r_fn = fp_recip_fn(D_r_fn);

  fp_front_force_jacobians_fn(tr.C_eff_f, front_num, tr.vx_safe, inv_D_f_fn,
                              &dFyf_dvx_fn, &dFyf_dvy_fn, &dFyf_dom_fn);
  fp_rear_force_jacobians_fn(tr.C_eff_r, rear_num, tr.vx_safe, inv_D_r_fn,
                             &dFyr_dvx_fn, &dFyr_dvy_fn, &dFyr_dom_fn);

  if (fp_QP_from_FN(tr.vx_safe) <= MIN_LIN_VEL) {
    dFyf_dvx_fn = FP_FN_ZERO;
    dFyr_dvx_fn = FP_FN_ZERO;
  }
  tr.dFyf_dvx = dFyf_dvx_fn;
  tr.dFyf_dvy = dFyf_dvy_fn;
  tr.dFyf_dom = dFyf_dom_fn;
  tr.dFyr_dvx = dFyr_dvx_fn;
  tr.dFyr_dvy = dFyr_dvy_fn;
  tr.dFyr_dom = dFyr_dom_fn;
}

/* -----------------------------------------------------------------------
 * Half 2 — A/B matrix assembly (~38 FSM states).
 * Inputs:  geometry (ey, epsi, kappa), FpTireResults struct.
 * Outputs: A_fr, B_fr, next_state arrays.
 * ----------------------------------------------------------------------- */

/* Helpers extracted from compute_frenet_AB_hls to create clear HLS
 * module boundaries and shorten combinational paths. Placed at file
 * scope (not inside the main function) so HLS will synthesize them as
 * separate modules when `#pragma HLS INLINE off` is present. */
struct StageSharedProducts {
  fp_FN_t dFyf_dvx_sin;
  fp_FN_t dFyf_dvy_sin;
  fp_FN_t dFyf_dom_sin;
  fp_FN_t vx_damping;
  fp_FN_t vy_damping;
  fp_FN_t om_damping;
  fp_FN_t neg_vx_damping;
  fp_FN_t neg_vy_damping;
  fp_FN_t neg_om_damping;
  fp_FN_t dFyf_dvx_cos;
  fp_FN_t dFyf_dvy_cos;
  fp_FN_t dFyf_dom_cos;
  fp_FN_t mass_omega;
  fp_FN_t mass_vx;
  fp_FN_t neg_mass_omega;
  fp_FN_t neg_mass_vx;
  fp_FN_t lf_dFyf_dvx_cos;
  fp_FN_t lf_dFyf_dvy_cos;
  fp_FN_t lf_dFyf_dom_cos;
  fp_FN_t lr_dFyr_dvx;
  fp_FN_t lr_dFyr_dvy;
  fp_FN_t lr_dFyr_dom;
  fp_FN_t neg_lr_dFyr_dvx;
  fp_FN_t neg_lr_dFyr_dvy;
  fp_FN_t neg_lr_dFyr_dom;
};

static inline void compute_stage_shared_products(const FpTireResults &tr,
                                                 fp_FN_t vx,
                                                 fp_FN_t omega,
                                                 StageSharedProducts &out) {
#pragma HLS INLINE off
  fp_FN_t dFyf_dvx_sin_fn = fp_mul_fn(tr.dFyf_dvx, tr.sin_delta);
  fp_FN_t dFyf_dvy_sin_fn = fp_mul_fn(tr.dFyf_dvy, tr.sin_delta);
  fp_FN_t dFyf_dom_sin_fn = fp_mul_fn(tr.dFyf_dom, tr.sin_delta);
  out.dFyf_dvx_sin = dFyf_dvx_sin_fn;
  out.dFyf_dvy_sin = dFyf_dvy_sin_fn;
  out.dFyf_dom_sin = dFyf_dom_sin_fn;
  out.vx_damping = fp_mul_fn(out.dFyf_dvx_sin, VP_FN_DT_INV_MASS);
  out.vy_damping = fp_mul_fn(out.dFyf_dvy_sin, VP_FN_DT_INV_MASS);
  out.om_damping = fp_mul_fn(out.dFyf_dom_sin, VP_FN_DT_INV_MASS);
  out.neg_vx_damping = -out.vx_damping;
  out.neg_vy_damping = -out.vy_damping;
  out.neg_om_damping = -out.om_damping;
  fp_FN_t dFyf_dvx_cos_fn = fp_mul_fn(tr.dFyf_dvx, tr.cos_delta);
  fp_FN_t dFyf_dvy_cos_fn = fp_mul_fn(tr.dFyf_dvy, tr.cos_delta);
  fp_FN_t dFyf_dom_cos_fn = fp_mul_fn(tr.dFyf_dom, tr.cos_delta);
  out.dFyf_dvx_cos = dFyf_dvx_cos_fn;
  out.dFyf_dvy_cos = dFyf_dvy_cos_fn;
  out.dFyf_dom_cos = dFyf_dom_cos_fn;
  out.mass_omega = fp_mul_fn(VP_FN_MASS, omega);
  out.mass_vx = fp_mul_fn(VP_FN_MASS, vx);
  out.neg_mass_omega = -out.mass_omega;
  out.neg_mass_vx = -out.mass_vx;

  out.lf_dFyf_dvx_cos = fp_mul_fn(VP_FN_LF, out.dFyf_dvx_cos);
  out.lf_dFyf_dvy_cos = fp_mul_fn(VP_FN_LF, out.dFyf_dvy_cos);
  out.lf_dFyf_dom_cos = fp_mul_fn(VP_FN_LF, out.dFyf_dom_cos);
  out.lr_dFyr_dvx = fp_mul_fn(VP_FN_LR, tr.dFyr_dvx);
  out.lr_dFyr_dvy = fp_mul_fn(VP_FN_LR, tr.dFyr_dvy);
  out.lr_dFyr_dom = fp_mul_fn(VP_FN_LR, tr.dFyr_dom);
  out.neg_lr_dFyr_dvx = -out.lr_dFyr_dvx;
  out.neg_lr_dFyr_dvy = -out.lr_dFyr_dvy;
  out.neg_lr_dFyr_dom = -out.lr_dFyr_dom;
}

static inline void compute_B_steering(const FpTireResults &tr,
                                      fp_FN_t lf_dt_over_iz,
                                      fp_FN_t B_fr[MPC_NX_FRENET][MPC_NU]) {
#pragma HLS INLINE off
  bool use_front_raw = (tr.C_eff_f_raw > tr.C_min_f);
  fp_FN_t dFyf_dd_sin_raw_fn = fp_mul_fn(tr.C_eff_f_raw, tr.sin_delta);
  fp_FN_t dFyf_dd_sin_min_fn = fp_mul_fn(tr.C_min_f, tr.sin_delta);
  fp_FN_t Fyf_cos_fn = fp_mul_fn(tr.F_yf, tr.cos_delta);
  fp_FN_t dFyf_dd_cos_raw_fn = fp_mul_fn(tr.C_eff_f_raw, tr.cos_delta);
  fp_FN_t dFyf_dd_cos_min_fn = fp_mul_fn(tr.C_min_f, tr.cos_delta);
  fp_FN_t Fyf_sin_fn = fp_mul_fn(tr.F_yf, tr.sin_delta);
  fp_FN_t Fyf_cos_dt = fp_mul_fn(Fyf_cos_fn, VP_FN_NEG_DT_INV_MASS);
  fp_FN_t Fyf_sin_dt = fp_mul_fn(Fyf_sin_fn, VP_FN_DT_INV_MASS);

  fp_FN_t B20_raw =
      fp_mul_fn(dFyf_dd_sin_raw_fn + Fyf_cos_fn, VP_FN_NEG_DT_INV_MASS);
  fp_FN_t B20_min =
      fp_mul_fn(dFyf_dd_sin_min_fn + Fyf_cos_fn, VP_FN_NEG_DT_INV_MASS);
  B_fr[2][0] = use_front_raw ? B20_raw : B20_min;

  fp_FN_t B30_raw =
      fp_mul_fn(dFyf_dd_cos_raw_fn, VP_FN_DT_INV_MASS) + (-Fyf_sin_dt);
  fp_FN_t B30_min =
      fp_mul_fn(dFyf_dd_cos_min_fn, VP_FN_DT_INV_MASS) + (-Fyf_sin_dt);
  B_fr[3][0] = use_front_raw ? B30_raw : B30_min;

  fp_FN_t B40_raw =
      fp_mul_fn(dFyf_dd_cos_raw_fn - Fyf_sin_fn, lf_dt_over_iz);
  fp_FN_t B40_min =
      fp_mul_fn(dFyf_dd_cos_min_fn - Fyf_sin_fn, lf_dt_over_iz);
  B_fr[4][0] = use_front_raw ? B40_raw : B40_min;

  B_fr[2][1] = FN_MPC_DT;
}

void compute_frenet_AB_hls(fp_QP_t ey, fp_QP_t epsi, fp_QP_t vx, fp_QP_t vy,
                           fp_QP_t omega, fp_QP_t delta, fp_QP_t a_cmd,
                           fp_QP_t kappa, fp_QP_t reference_velocity,
                           fp_QP_t A_fr[MPC_NX_FRENET][MPC_NX_FRENET],
                           fp_QP_t B_fr[MPC_NX_FRENET][MPC_NU],
                           fp_QP_t next_state[MPC_NX_FRENET]) {
#pragma HLS INLINE off
#pragma HLS ALLOCATION function instances = fp_recip_fn limit = 1
  /* ---- Half 1: tire physics ---- */
  FpTireResults tr;
#pragma HLS AGGREGATE variable = tr compact = bit
  compute_frenet_tire_hls(vx, vy, omega, delta, a_cmd, tr);

  fp_FN_t A_fn[MPC_NX_FRENET][MPC_NX_FRENET];
  fp_FN_t B_fn[MPC_NX_FRENET][MPC_NU];
  fp_FN_t next_state_fn[MPC_NX_FRENET];

  /* ---- Half 2: matrix assembly ---- */

  int i, j;
  for (i = 0; i < MPC_NX_FRENET; i++) {
#pragma HLS UNROLL
    for (j = 0; j < MPC_NX_FRENET; j++) {
#pragma HLS UNROLL
      A_fn[i][j] = FP_FN_ZERO;
    }
    B_fn[i][0] = FP_FN_ZERO;
    B_fn[i][1] = FP_FN_ZERO;
  }

  /* Frenet geometry rows */
  {
    fp_FN_t sin_epsi_fn, cos_epsi_fn;
    fp_trig_pair(fp_FN_from_QP(epsi), &sin_epsi_fn, &cos_epsi_fn);
    fp_FN_t inv_denom =
        fp_frenet_recip(fp_FN_from_QP(kappa), fp_FN_from_QP(ey));
    fp_frenet_rows01_fn(
        sin_epsi_fn, cos_epsi_fn, fp_FN_from_QP(vx), fp_FN_from_QP(vy),
        fp_FN_from_QP(kappa), fp_FN_from_QP(ey), inv_denom,
        fp_FN_from_QP(reference_velocity), &A_fn[0][0], &A_fn[0][1],
        &A_fn[0][2], &A_fn[0][3], &A_fn[1][0], &A_fn[1][1], &A_fn[1][2]);
    A_fn[1][4] = FP_FN_CONST(MPC_FPGA_PREDICTION_DT_S);

    /* Reuse exactly the same trig pair for rollout to avoid duplicate LUT
     * reads. */
    fp_FN_t Fx = fp_mul_fn(VP_FN_MASS, fp_FN_from_QP(a_cmd));
    fp_rollout_from_forces_fn(
        fp_FN_from_QP(ey), fp_FN_from_QP(epsi), sin_epsi_fn, cos_epsi_fn,
        tr.vx_safe, fp_FN_from_QP(reference_velocity), fp_FN_from_QP(vy),
        fp_FN_from_QP(omega), fp_FN_from_QP(kappa), FN_MPC_DT, Fx,
        tr.F_yf, tr.F_yr, tr.sin_delta, tr.cos_delta, next_state_fn);
  }

  /* Stage shared products (extracted to helper to reduce combinational depth)
   */
  StageSharedProducts s;
  compute_stage_shared_products(tr, fp_FN_from_QP(vx),
                                fp_FN_from_QP(omega), s);

  fp_FN_t dFyf_dvx_sin = s.dFyf_dvx_sin;
  fp_FN_t dFyf_dvy_sin = s.dFyf_dvy_sin;
  fp_FN_t dFyf_dom_sin = s.dFyf_dom_sin;
  fp_FN_t vx_damping = s.vx_damping;
  fp_FN_t vy_damping = s.vy_damping;
  fp_FN_t om_damping = s.om_damping;
  fp_FN_t neg_vx_damping = s.neg_vx_damping;
  fp_FN_t neg_vy_damping = s.neg_vy_damping;
  fp_FN_t neg_om_damping = s.neg_om_damping;

  fp_FN_t dFyf_dvx_cos = s.dFyf_dvx_cos;
  fp_FN_t dFyf_dvy_cos = s.dFyf_dvy_cos;
  fp_FN_t dFyf_dom_cos = s.dFyf_dom_cos;
  fp_FN_t mass_omega = s.mass_omega;
  fp_FN_t mass_vx = s.mass_vx;
  fp_FN_t neg_mass_omega = s.neg_mass_omega;
  fp_FN_t neg_mass_vx = s.neg_mass_vx;

  fp_FN_t lf_dFyf_dvx_cos = s.lf_dFyf_dvx_cos;
  fp_FN_t lf_dFyf_dvy_cos = s.lf_dFyf_dvy_cos;
  fp_FN_t lf_dFyf_dom_cos = s.lf_dFyf_dom_cos;
  fp_FN_t lr_dFyr_dvx = s.lr_dFyr_dvx;
  fp_FN_t lr_dFyr_dvy = s.lr_dFyr_dvy;
  fp_FN_t lr_dFyr_dom = s.lr_dFyr_dom;
  fp_FN_t neg_lr_dFyr_dvx = s.neg_lr_dFyr_dvx;
  fp_FN_t neg_lr_dFyr_dvy = s.neg_lr_dFyr_dvy;
  fp_FN_t neg_lr_dFyr_dom = s.neg_lr_dFyr_dom;
  fp_FN_t lf_dt_over_iz = fp_mul_fn(VP_FN_LF, VP_FN_DT_INV_IZ);
  fp_FN_t neg_lr_dt_over_iz = fp_mul_fn(-VP_FN_LR, VP_FN_DT_INV_IZ);

  /* Row 2: vx dynamics */
  A_fn[2][2] = FP_FN_ONE + neg_vx_damping;
  A_fn[2][3] = fp_mul_fn(FN_MPC_DT, fp_FN_from_QP(omega)) + neg_vy_damping;
  A_fn[2][4] = fp_mul_fn(FN_MPC_DT, fp_FN_from_QP(vy)) + neg_om_damping;

  /* Row 3: vy dynamics (uses DT-scaled inv_mass, matching baseline QP). */
  A_fn[3][2] =
      fp_mul_fn(dFyf_dvx_cos + tr.dFyr_dvx + neg_mass_omega, VP_FN_DT_INV_MASS);
  A_fn[3][3] =
      FP_FN_ONE + fp_mul_fn(dFyf_dvy_cos + tr.dFyr_dvy, VP_FN_DT_INV_MASS);
  A_fn[3][4] =
      fp_mul_fn(dFyf_dom_cos + tr.dFyr_dom + neg_mass_vx, VP_FN_DT_INV_MASS);

  /* Row 4: omega dynamics.*/
  A_fn[4][2] = fp_mul_fn(lf_dFyf_dvx_cos + neg_lr_dFyr_dvx, VP_FN_DT_INV_IZ);
  A_fn[4][3] = fp_mul_fn(lf_dFyf_dvy_cos + neg_lr_dFyr_dvy, VP_FN_DT_INV_IZ);
  A_fn[4][4] =
      FP_FN_ONE + fp_mul_fn(lf_dFyf_dom_cos + neg_lr_dFyr_dom, VP_FN_DT_INV_IZ);

  /* B matrix — steering (moved into helper) */
  compute_B_steering(tr, lf_dt_over_iz, B_fn);

  /* B matrix — acceleration / load transfer */
  {
    fp_FN_t F_zf = VP_FN_FZ_FRONT - tr.Fz_transfer;
    fp_FN_t F_zr = VP_FN_FZ_REAR + tr.Fz_transfer;
    fp_FN_t inv_Fzf = fp_recip_fn(F_zf);
    fp_FN_t inv_Fzr = fp_recip_fn(F_zr);
    fp_FN_t C_Sf_norm_raw = fp_mul_fn(tr.C_eff_f_raw, inv_Fzf);
    fp_FN_t C_Sf_norm_min = fp_mul_fn(tr.C_min_f, inv_Fzf);
    fp_FN_t C_Sr_norm = fp_mul_fn(tr.C_eff_r, inv_Fzr);

    fp_FN_t dFzf_da = -VP_FN_FZ_LOAD_GAIN;
    fp_FN_t dFzr_da = VP_FN_FZ_LOAD_GAIN;

    bool use_front_raw = (tr.C_eff_f_raw > tr.C_min_f);
    fp_FN_t dFyf_da_raw = fp_mul_fn(fp_mul_fn(C_Sf_norm_raw, tr.alpha_f_op), dFzf_da);
    fp_FN_t dFyf_da_min = fp_mul_fn(fp_mul_fn(C_Sf_norm_min, tr.alpha_f_op), dFzf_da);
    fp_FN_t dFyr_da = fp_mul_fn(fp_mul_fn(C_Sr_norm, tr.alpha_r_op), dFzr_da);
    fp_FN_t dFyf_da_raw_cos = fp_mul_fn(dFyf_da_raw, tr.cos_delta);
    fp_FN_t dFyf_da_min_cos = fp_mul_fn(dFyf_da_min, tr.cos_delta);

    fp_FN_t B31_raw = fp_mul_fn(dFyf_da_raw_cos + dFyr_da, VP_FN_DT_INV_MASS);
    fp_FN_t B31_min = fp_mul_fn(dFyf_da_min_cos + dFyr_da, VP_FN_DT_INV_MASS);
    B_fn[3][1] = use_front_raw ? B31_raw : B31_min;

    fp_FN_t lr_dFyr_da_dt_iz = fp_mul_fn(dFyr_da, neg_lr_dt_over_iz);
    fp_FN_t B41_raw = fp_mul_fn(dFyf_da_raw_cos, lf_dt_over_iz) + lr_dFyr_da_dt_iz;
    fp_FN_t B41_min = fp_mul_fn(dFyf_da_min_cos, lf_dt_over_iz) + lr_dFyr_da_dt_iz;
    B_fn[4][1] = use_front_raw ? B41_raw : B41_min;
  }

  for (i = 0; i < MPC_NX_FRENET; i++) {
#pragma HLS UNROLL
    for (j = 0; j < MPC_NX_FRENET; j++) {
#pragma HLS UNROLL
      A_fr[i][j] = fp_QP_from_FN(A_fn[i][j]);
    }
    for (j = 0; j < MPC_NU; j++) {
#pragma HLS UNROLL
      B_fr[i][j] = fp_QP_from_FN(B_fn[i][j]);
    }
    next_state[i] = fp_QP_from_FN(next_state_fn[i]);
  }
}

void compute_frenet_AB_and_next_hls(fp_QP_t ey, fp_QP_t epsi, fp_QP_t vx,
                                    fp_QP_t vy, fp_QP_t omega, fp_QP_t delta,
                                    fp_QP_t a_cmd, fp_QP_t kappa,
                                    fp_QP_t reference_velocity,
                                    fp_QP_t A_fr[MPC_NX_FRENET][MPC_NX_FRENET],
                                    fp_QP_t B_fr[MPC_NX_FRENET][MPC_NU],
                                    fp_QP_t next_state[MPC_NX_FRENET]) {
#pragma HLS INLINE off
  /*
   * NOTE: This wrapper intentionally exists as a separate function
   * and is marked `#pragma HLS INLINE off` to create a clear HLS
   * function/module boundary. The split is deliberate:
   * - It gives the HLS tool an explicit module to synthesize (see HLS logs
   *   where `compute_frenet_AB_and_next_hls` is implemented as a module).
   * - Preventing inlining keeps the half-1 (tire model) and half-2
   *   (A/B assembly + rollout) grouped under a defined RTL module,
   *   enabling finer control of FSM/state replication and resource
   *   partitioning during synthesis.
   * - It preserves the public API used by other code (e.g. the Riccati
   *   HLS flow) and avoids changing many call-sites.
   *
   * Removing this wrapper could force compute_frenet_AB_hls to be
   * inlined everywhere, which changes the HLS module hierarchy and may
   * negatively affect critical-path, FSM states, or replication.
   */
  compute_frenet_AB_hls(ey, epsi, vx, vy, omega, delta, a_cmd, kappa,
                        reference_velocity, A_fr, B_fr, next_state);
}

/**
 * Saturate control to physical vehicle limits.
 *
 * @param steer_in   Raw steering angle [rad]
 * @param accel_in   Raw acceleration [m/s^2]
 * @param steer_out  Clamped steering
 * @param accel_out  Clamped acceleration
 * @return None.
 */
void saturate_control_hls(fp_QP_t steer_in, fp_QP_t accel_in,
                          fp_QP_t *steer_out, fp_QP_t *accel_out) {
#pragma HLS INLINE
  *steer_out = fp_clamp(steer_in, -VP_MAX_STEER, VP_MAX_STEER);
  *accel_out = fp_clamp(accel_in, VP_MIN_ACCEL, VP_MAX_ACCEL);
}
