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

static fp_QP_t fp_pacejka_inner_arg(fp_QP_t ba)
{
#pragma HLS INLINE off
    fp_QP_t atan_ba = fp_atan_tire_approx(ba);
    return fp_mul(VP_C_SHAPE, atan_ba);
}

/* Breaks the 2-deep multiply chain:  cos_inner * inv_denom → D_cb * result.
 * With INLINE off, HLS creates a module boundary with registered I/Os,
 * preventing the combinational DSP-to-DSP path that caused -0.14ns
 * timing violation in compute_frenet_AB_hls. */
static fp_QP_t fp_pacejka_ceff(
    fp_QP_t cos_inner, fp_QP_t inv_denom, fp_QP_t D_cb)
{
#pragma HLS INLINE off
    fp_QP_t cos_over_denom = fp_mul(cos_inner, inv_denom);
    return fp_mul(D_cb, cos_over_denom);
}

static void fp_trig_pair(
    fp_QP_t angle,
    fp_QP_t *sin_val,
    fp_QP_t *cos_val)
{
#pragma HLS INLINE off
#pragma HLS PIPELINE II=1
    fp_QP_t angle_reg = angle;
    *sin_val = fp_sin(angle_reg);
    *cos_val = fp_cos(angle_reg);
}

/* Step 1 of the Frenet rows 0/1 critical path split.
 * Returns 1/denom with INLINE off so HLS registers the output
 * before the multiply fan-out that follows. */
static fp_QP_t fp_frenet_recip(
    fp_QP_t epsi, fp_QP_t kappa, fp_QP_t ey)
{
#pragma HLS INLINE off
    fp_QP_t sp, cp;
    fp_trig_pair(epsi, &sp, &cp);
    fp_QP_t denom = FP_ONE - fp_mul(kappa, ey);
    if (fp_abs(denom) < FP_QP_CONST(1e-3)) {
        denom = (denom >= 0) ? FP_QP_CONST(1e-3) : FP_QP_CONST(-1e-3);
    }
    return fp_recip(denom);
}

/* Step 2: receives registered inv_denom, computes all A rows 0/1 entries.
 * Separating this from fp_frenet_recip() ensures HLS schedules the
 * multiply fan-out (inv_denom2, A10, A11, A12) in states AFTER fp_recip
 * completes, preventing the -0.12ns combinational path. */
static void fp_frenet_rows01(
    fp_QP_t epsi, fp_QP_t vx, fp_QP_t vy,
    fp_QP_t kappa, fp_QP_t ey, fp_QP_t inv_denom,
    fp_QP_t *A00, fp_QP_t *A01, fp_QP_t *A02, fp_QP_t *A03,
    fp_QP_t *A10, fp_QP_t *A11, fp_QP_t *A12)
{
#pragma HLS INLINE off
    fp_QP_t sp, cp;
    fp_trig_pair(epsi, &sp, &cp);
    fp_QP_t dt_cp    = fp_mul(MPC_DT, cp);
    fp_QP_t kappa_dt = fp_mul(kappa, MPC_DT);
    fp_QP_t kappa2   = fp_mul(kappa, kappa);
    fp_QP_t vx_cp    = fp_mul(vx, cp);
    fp_QP_t vy_sp    = fp_mul(vy, sp);
    *A00 = FP_ONE;
    *A01 = fp_mul(MPC_DT, vx_cp + (-vy_sp));
    *A02 = fp_mul(MPC_DT, sp);
    *A03 = dt_cp;
    /* With MUL_LATENCY=4, every fp_mul() has an extra FF stage — the
     * inv_denom → inv_denom2 → A10 chain is automatically broken. */
    fp_QP_t inv_denom2 = fp_mul(inv_denom, inv_denom);
    fp_QP_t k2_vx    = fp_mul(kappa2, vx);
    fp_QP_t a10_pre  = fp_mul(dt_cp, k2_vx);
    *A10 = fp_mul(-a10_pre, inv_denom2);
    fp_QP_t kdt_vx   = fp_mul(kappa_dt, vx);
    fp_QP_t a11_pre  = fp_mul(kdt_vx, sp);
    *A11 = FP_ONE + fp_mul(a11_pre, inv_denom);
    fp_QP_t a12_pre  = fp_mul(kappa_dt, cp);
    *A12 = fp_mul(-a12_pre, inv_denom);
}

static void fp_slip_terms(
    fp_QP_t vy,
    fp_QP_t omega,
    fp_QP_t inv_vx,
    fp_QP_t *lf_omega,
    fp_QP_t *lr_omega,
    fp_QP_t *front_num,
    fp_QP_t *rear_num,
    fp_QP_t *front_ratio,
    fp_QP_t *rear_ratio)
{
#pragma HLS PIPELINE II=1
    fp_QP_t lf_omega_local = fp_mul(VP_LF, omega);
    fp_QP_t lr_omega_local = fp_mul(VP_LR, omega);
    fp_QP_t front_num_local = vy + lf_omega_local;
    fp_QP_t rear_num_local = vy + (-lr_omega_local);

    *lf_omega = lf_omega_local;
    *lr_omega = lr_omega_local;
    *front_num = front_num_local;
    *rear_num = rear_num_local;
    *front_ratio = fp_mul(front_num_local, inv_vx);
    *rear_ratio = fp_mul(rear_num_local, inv_vx);
}

static void fp_front_force_jacobians(
    fp_QP_t C_eff_f, fp_QP_t front_num, fp_QP_t vx_safe, fp_QP_t inv_D_f,
    fp_QP_t *dFyf_dvx, fp_QP_t *dFyf_dvy, fp_QP_t *dFyf_dom)
{
#pragma HLS INLINE off
    fp_QP_t vx_inv = fp_mul(vx_safe, inv_D_f);
    fp_QP_t daf_dvx = fp_mul(front_num, inv_D_f);
    fp_QP_t daf_dvy = -vx_inv;
    fp_QP_t lf_vx = fp_mul(VP_LF, vx_safe);
    fp_QP_t daf_dom = -fp_mul(lf_vx, inv_D_f);
    *dFyf_dvx = fp_mul(C_eff_f, daf_dvx);
    *dFyf_dvy = fp_mul(C_eff_f, daf_dvy);
    *dFyf_dom = fp_mul(C_eff_f, daf_dom);
}

static void fp_rear_force_jacobians(
    fp_QP_t C_eff_r, fp_QP_t rear_num, fp_QP_t vx_safe, fp_QP_t inv_D_r,
    fp_QP_t *dFyr_dvx, fp_QP_t *dFyr_dvy, fp_QP_t *dFyr_dom)
{
#pragma HLS INLINE off
    fp_QP_t vx_inv = fp_mul(vx_safe, inv_D_r);
    fp_QP_t dar_dvx = fp_mul(rear_num, inv_D_r);
    fp_QP_t dar_dvy = -vx_inv;
    fp_QP_t dar_dom = fp_mul(VP_LR, vx_inv);
    *dFyr_dvx = fp_mul(C_eff_r, dar_dvx);
    *dFyr_dvy = fp_mul(C_eff_r, dar_dvy);
    *dFyr_dom = fp_mul(C_eff_r, dar_dom);
}

static void fp_rollout_from_forces_hls(
    fp_QP_t ey, fp_QP_t epsi,
    fp_QP_t vx_safe, fp_QP_t vy, fp_QP_t omega,
    fp_QP_t kappa, fp_QP_t dt,
    fp_QP_t Fx, fp_QP_t F_yf, fp_QP_t F_yr,
    fp_QP_t sin_delta, fp_QP_t cos_delta,
    fp_QP_t next_state[MPC_NX_FRENET])
{
#pragma HLS INLINE
    fp_QP_t ey_denom = FP_ONE - fp_mul(kappa, ey);
    if (fp_abs(ey_denom) < FP_QP_CONST(1e-3)) {
        ey_denom = (ey_denom >= 0) ? FP_QP_CONST(1e-3) : FP_QP_CONST(-1e-3);
    }

    fp_QP_t sin_epsi = fp_sin(epsi);
    fp_QP_t cos_epsi = fp_cos(epsi);
    fp_QP_t inv_ey_denom = fp_recip(ey_denom);

    fp_QP_t vx_sin_epsi = fp_mul(vx_safe, sin_epsi);
    fp_QP_t vy_cos_epsi = fp_mul(vy, cos_epsi);
    fp_QP_t vx_cos_epsi = fp_mul(vx_safe, cos_epsi);
    fp_QP_t kappa_vx_cos = fp_mul(kappa, vx_cos_epsi);
    fp_QP_t kappa_vx_cos_inv = fp_mul(kappa_vx_cos, inv_ey_denom);
    fp_QP_t fx_sin_delta = fp_mul(F_yf, sin_delta);
    fp_QP_t fyf_cos = fp_mul(F_yf, cos_delta);
    fp_QP_t dvy_term = fyf_cos + F_yr;
    fp_QP_t vx_omega = fp_mul(vx_safe, omega);
    fp_QP_t vy_omega = fp_mul(vy, omega);
    fp_QP_t domega_lf = fp_mul(VP_LF, fyf_cos);
    fp_QP_t domega_lr = fp_mul(VP_LR, F_yr);
    fp_QP_t dvx_dt = fp_mul((Fx - fx_sin_delta), VP_INV_MASS) + vy_omega;
    fp_QP_t dvy_dt = fp_mul(dvy_term, VP_INV_MASS) - vx_omega;
    fp_QP_t domega_dt = fp_mul((domega_lf - domega_lr), VP_INV_IZ);
    fp_QP_t e_y_step_vx = fp_mul(dt, vx_sin_epsi);
    fp_QP_t e_y_step_vy = fp_mul(dt, vy_cos_epsi);
    fp_QP_t e_psi_step_omega = fp_mul(dt, omega);
    fp_QP_t e_psi_step_corr = fp_mul(dt, kappa_vx_cos_inv);
    fp_QP_t dvx_step = fp_mul(dt, dvx_dt);
    fp_QP_t dvy_step = fp_mul(dt, dvy_dt);
    fp_QP_t domega_step = fp_mul(dt, domega_dt);

    next_state[0] = ey + e_y_step_vx + e_y_step_vy;
    next_state[1] = epsi + e_psi_step_omega + (-e_psi_step_corr);
    next_state[2] = vx_safe + dvx_step;
    if (next_state[2] < MIN_LIN_VEL) next_state[2] = MIN_LIN_VEL;
    next_state[3] = vy + dvy_step;
    next_state[4] = omega + domega_step;
}

void predict_frenet_next_hls(
    fp_QP_t ey, fp_QP_t epsi,
    fp_QP_t vx, fp_QP_t vy, fp_QP_t omega,
    fp_QP_t delta, fp_QP_t a_cmd,
    fp_QP_t kappa, fp_QP_t dt,
    fp_QP_t next_state[MPC_NX_FRENET]);

/* Compute Frenet-frame linearized dynamics (split into two sequential sub-functions
 * to halve the FSM state count and reduce the HLS critical path). */
/* -----------------------------------------------------------------------
 * Tire physics intermediate results, passed from compute_frenet_tire_hls
 * to compute_frenet_AB_hls.  Using a plain struct avoids pointer-to-array
 * address arithmetic that would add extra FSM states.
 * ----------------------------------------------------------------------- */
struct FpTireResults {
    fp_QP_t C_eff_f, C_eff_f_raw, C_eff_r, C_min_f, C_min_r;
    fp_QP_t F_yf, F_yr;
    fp_QP_t dFyf_dvx, dFyf_dvy, dFyf_dom;
    fp_QP_t dFyr_dvx, dFyr_dvy, dFyr_dom;
    fp_QP_t sin_delta, cos_delta;
    fp_QP_t alpha_f_op, alpha_r_op;
    fp_QP_t inv_vx;
    fp_QP_t D_transfer;
    fp_QP_t Fz_transfer;
    fp_QP_t vx_safe;
};

/* -----------------------------------------------------------------------
 * Half 1 — Tire physics and Jacobians (~40 FSM states).
 * Inputs:  raw vehicle state + control.
 * Outputs: FpTireResults struct (all tire forces and their Jacobians).
 * Splitting here removes 10 sub-module handshake state groups from the
 * original compute_frenet_AB_hls FSM, cutting the next-state logic from
 * 458 LUT → ~220 LUT and the critical path from 4.684ns → ~3.5ns.
 * ----------------------------------------------------------------------- */
static void compute_frenet_tire_hls(
    fp_QP_t vx, fp_QP_t vy, fp_QP_t omega,
    fp_QP_t delta, fp_QP_t a_cmd,
    FpTireResults &tr)
{
#pragma HLS INLINE off
    tr.vx_safe = (vx > MIN_LIN_VEL) ? vx : MIN_LIN_VEL;
    tr.inv_vx  = fp_recip(tr.vx_safe);

    /* Polynomial trig (no transcendental paths in this critical function). */
    fp_QP_t d2 = fp_mul(delta, delta);
    fp_QP_t d3 = fp_mul(d2,    delta);
    fp_QP_t d4 = fp_mul(d2,    d2);
    fp_QP_t d5 = fp_mul(d4,    delta);
    tr.cos_delta = FP_ONE - fp_mul(d2, INV_FACT_2) + fp_mul(d4, INV_FACT_4);
    tr.sin_delta = delta  - fp_mul(d3, INV_FACT_3) + fp_mul(d5, INV_FACT_5);

    /* Load transfer */
    tr.Fz_transfer = fp_mul(a_cmd, VP_FZ_LOAD_GAIN);
    tr.D_transfer  = fp_mul(a_cmd, VP_D_LOAD_GAIN);
    fp_QP_t Cmin_transfer_f = fp_mul(a_cmd, VP_CMIN_FRONT_LOAD_GAIN);
    fp_QP_t Cmin_transfer_r = fp_mul(a_cmd, VP_CMIN_REAR_LOAD_GAIN);

    fp_QP_t D_pac_f = VP_D_FRONT + (-tr.D_transfer);
    fp_QP_t D_pac_r = VP_D_REAR  + tr.D_transfer;
    tr.C_min_f = VP_CMIN_FRONT_STATIC - Cmin_transfer_f;
    tr.C_min_r = VP_CMIN_REAR_STATIC  + Cmin_transfer_r;

    /* Slip terms */
    fp_QP_t lf_omega, lr_omega, front_num, rear_num;
    fp_QP_t front_ratio, rear_ratio;
    fp_slip_terms(vy, omega, tr.inv_vx,
                  &lf_omega, &lr_omega,
                  &front_num, &rear_num,
                  &front_ratio, &rear_ratio);

    fp_QP_t vx2        = fp_mul(tr.vx_safe, tr.vx_safe);
    fp_QP_t vy2        = fp_mul(vy, vy);
    fp_QP_t lf_omega2  = fp_mul(lf_omega, lf_omega);
    fp_QP_t lr_omega2  = fp_mul(lr_omega, lr_omega);
    fp_QP_t front_cross = fp_mul(vy, lf_omega);
    fp_QP_t rear_cross  = fp_mul(vy, lr_omega);
    fp_QP_t front_num2  = vy2 + (front_cross + front_cross) + lf_omega2;
    fp_QP_t rear_num2   = vy2 + (rear_cross  + rear_cross)  + lr_omega2;
    fp_QP_t D_f = vx2 + front_num2;
    fp_QP_t D_r = vx2 + rear_num2;
    fp_QP_t inv_D_f = fp_recip(D_f);
    fp_QP_t inv_D_r = fp_recip(D_r);

    /* Slip angles */
    tr.alpha_f_op = delta - fp_atan_tire_approx(front_ratio);
    fp_QP_t rear_atan = fp_atan_tire_approx(rear_ratio);
    tr.alpha_r_op = -rear_atan;

    /* Front Pacejka */
    fp_QP_t Ba_f        = fp_mul(VP_B_FRONT, tr.alpha_f_op);
    fp_QP_t inner_f     = fp_pacejka_inner_arg(Ba_f);
    fp_QP_t sin_inner_f, cos_inner_f;
    fp_trig_pair(inner_f, &sin_inner_f, &cos_inner_f);
    fp_QP_t ba_f2       = fp_sq(Ba_f);
    fp_QP_t inv_denom_f = fp_recip(FP_ONE + ba_f2);
    fp_QP_t D_pac_f_cb  = fp_mul(VP_D_FRONT, VP_CB_FRONT)
                        + (-fp_mul(tr.D_transfer, VP_CB_FRONT));
    tr.C_eff_f_raw      = fp_pacejka_ceff(cos_inner_f, inv_denom_f, D_pac_f_cb);
    tr.F_yf             = fp_mul(D_pac_f, sin_inner_f);

    /* Rear Pacejka */
    fp_QP_t Ba_r        = fp_mul(VP_B_REAR, tr.alpha_r_op);
    fp_QP_t inner_r     = fp_pacejka_inner_arg(Ba_r);
    fp_QP_t sin_inner_r, cos_inner_r;
    fp_trig_pair(inner_r, &sin_inner_r, &cos_inner_r);
    tr.F_yr             = fp_mul(D_pac_r, sin_inner_r);
    fp_QP_t ba_r2       = fp_sq(Ba_r);
    fp_QP_t inv_denom_r = fp_recip(FP_ONE + ba_r2);
    fp_QP_t D_pac_r_cb  = fp_mul(VP_D_REAR,  VP_CB_REAR)
                        + fp_mul(tr.D_transfer, VP_CB_REAR);
    tr.C_eff_r          = fp_pacejka_ceff(cos_inner_r, inv_denom_r, D_pac_r_cb);
    tr.C_eff_r          = (tr.C_eff_r > tr.C_min_r) ? tr.C_eff_r : tr.C_min_r;

    /* Force Jacobians */
    tr.C_eff_f = (tr.C_eff_f_raw > tr.C_min_f) ? tr.C_eff_f_raw : tr.C_min_f;
    fp_front_force_jacobians(tr.C_eff_f, front_num, tr.vx_safe, inv_D_f,
                             &tr.dFyf_dvx, &tr.dFyf_dvy, &tr.dFyf_dom);
    fp_rear_force_jacobians( tr.C_eff_r, rear_num,  tr.vx_safe, inv_D_r,
                             &tr.dFyr_dvx, &tr.dFyr_dvy, &tr.dFyr_dom);
}

/* -----------------------------------------------------------------------
 * Half 2 — A/B matrix assembly (~38 FSM states).
 * Inputs:  geometry (ey, epsi, kappa), FpTireResults struct.
 * Outputs: A_fr, B_fr, next_state arrays.
 * ----------------------------------------------------------------------- */
void compute_frenet_AB_hls(
    fp_QP_t ey, fp_QP_t epsi,
    fp_QP_t vx, fp_QP_t vy, fp_QP_t omega,
    fp_QP_t delta, fp_QP_t a_cmd,
    fp_QP_t kappa,
    fp_QP_t A_fr[MPC_NX_FRENET][MPC_NX_FRENET],
    fp_QP_t B_fr[MPC_NX_FRENET][MPC_NU],
    fp_QP_t next_state[MPC_NX_FRENET])
{
#pragma HLS INLINE off

    /* ---- Half 1: tire physics ---- */
    FpTireResults tr;
#pragma HLS AGGREGATE variable=tr compact=bit
    compute_frenet_tire_hls(vx, vy, omega, delta, a_cmd, tr);

    /* ---- Half 2: matrix assembly ---- */

    int i, j;
    for (i = 0; i < MPC_NX_FRENET; i++) {
#pragma HLS UNROLL
        for (j = 0; j < MPC_NX_FRENET; j++) {
#pragma HLS UNROLL
            A_fr[i][j] = 0;
        }
        B_fr[i][0] = 0;
        B_fr[i][1] = 0;
    }

    /* Frenet geometry rows */
    {
        fp_QP_t inv_denom = fp_frenet_recip(epsi, kappa, ey);
        fp_frenet_rows01(
            epsi, vx, vy, kappa, ey, inv_denom,
            &A_fr[0][0], &A_fr[0][1], &A_fr[0][2], &A_fr[0][3],
            &A_fr[1][0], &A_fr[1][1], &A_fr[1][2]);
        A_fr[1][4] = MPC_DT;
    }

    /* Stage shared products */
    fp_QP_t dFyf_dvx_sin = fp_mul(tr.dFyf_dvx, tr.sin_delta);
    fp_QP_t dFyf_dvy_sin = fp_mul(tr.dFyf_dvy, tr.sin_delta);
    fp_QP_t dFyf_dom_sin = fp_mul(tr.dFyf_dom, tr.sin_delta);
    fp_QP_t vx_damping   = fp_mul(dFyf_dvx_sin, VP_DT_INV_MASS);
    fp_QP_t vy_damping   = fp_mul(dFyf_dvy_sin, VP_INV_MASS);
    fp_QP_t om_damping   = fp_mul(dFyf_dom_sin, VP_INV_MASS);
    fp_QP_t neg_vx_damping = -vx_damping;
    fp_QP_t neg_vy_damping = -vy_damping;
    fp_QP_t neg_om_damping = -om_damping;

    fp_QP_t dFyf_dvx_cos = fp_mul(tr.dFyf_dvx, tr.cos_delta);
    fp_QP_t dFyf_dvy_cos = fp_mul(tr.dFyf_dvy, tr.cos_delta);
    fp_QP_t dFyf_dom_cos = fp_mul(tr.dFyf_dom, tr.cos_delta);
    fp_QP_t mass_omega   = fp_mul(VP_MASS, omega);
    fp_QP_t mass_vx      = fp_mul(VP_MASS, vx);
    fp_QP_t neg_mass_omega = -mass_omega;
    fp_QP_t neg_mass_vx    = -mass_vx;

    fp_QP_t lf_dFyf_dvx_cos = fp_mul(VP_LF, dFyf_dvx_cos);
    fp_QP_t lf_dFyf_dvy_cos = fp_mul(VP_LF, dFyf_dvy_cos);
    fp_QP_t lf_dFyf_dom_cos = fp_mul(VP_LF, dFyf_dom_cos);
    fp_QP_t lr_dFyr_dvx     = fp_mul(VP_LR, tr.dFyr_dvx);
    fp_QP_t lr_dFyr_dvy     = fp_mul(VP_LR, tr.dFyr_dvy);
    fp_QP_t lr_dFyr_dom     = fp_mul(VP_LR, tr.dFyr_dom);
    fp_QP_t neg_lr_dFyr_dvx = -lr_dFyr_dvx;
    fp_QP_t neg_lr_dFyr_dvy = -lr_dFyr_dvy;
    fp_QP_t neg_lr_dFyr_dom = -lr_dFyr_dom;

    /* Row 2: vx dynamics */
    A_fr[2][2] = FP_ONE + neg_vx_damping;
    A_fr[2][3] = fp_mul(MPC_DT, omega + neg_vy_damping);
    A_fr[2][4] = fp_mul(MPC_DT, vy    + neg_om_damping);

    /* Row 3: vy dynamics */
    A_fr[3][2] = fp_mul((fp_QP_t)(dFyf_dvx_cos + tr.dFyr_dvx + neg_mass_omega), VP_DT_INV_MASS);
    A_fr[3][3] = FP_ONE + fp_mul((fp_QP_t)(dFyf_dvy_cos + tr.dFyr_dvy), VP_DT_INV_MASS);
    A_fr[3][4] = fp_mul((fp_QP_t)(dFyf_dom_cos + tr.dFyr_dom + neg_mass_vx),  VP_DT_INV_MASS);

    /* Row 4: omega dynamics */
    A_fr[4][2] = fp_mul((fp_QP_t)(lf_dFyf_dvx_cos + neg_lr_dFyr_dvx), VP_DT_INV_IZ);
    A_fr[4][3] = fp_mul((fp_QP_t)(lf_dFyf_dvy_cos + neg_lr_dFyr_dvy), VP_DT_INV_IZ);
    A_fr[4][4] = FP_ONE + fp_mul((fp_QP_t)(lf_dFyf_dom_cos + neg_lr_dFyr_dom), VP_DT_INV_IZ);

    /* B matrix — steering */
    bool use_front_raw = (tr.C_eff_f_raw > tr.C_min_f);
    fp_QP_t dFyf_dd_sin_raw = fp_mul(tr.C_eff_f_raw, tr.sin_delta);
    fp_QP_t dFyf_dd_sin_min = fp_mul(tr.C_min_f,     tr.sin_delta);
    fp_QP_t Fyf_cos         = fp_mul(tr.F_yf, tr.cos_delta);
    fp_QP_t dFyf_dd_cos_raw = fp_mul(tr.C_eff_f_raw, tr.cos_delta);
    fp_QP_t dFyf_dd_cos_min = fp_mul(tr.C_min_f,     tr.cos_delta);
    fp_QP_t Fyf_sin         = fp_mul(tr.F_yf, tr.sin_delta);
    fp_QP_t Fyf_cos_dt      = fp_mul(Fyf_cos, NEG_VP_DT_INV_MASS);
    fp_QP_t Fyf_sin_dt      = fp_mul(Fyf_sin, VP_DT_INV_MASS);
    fp_QP_t lf_dt_over_iz      = fp_mul(VP_LF,         VP_DT_INV_IZ);
    fp_QP_t neg_lr_dt_over_iz  = fp_mul((fp_QP_t)(-VP_LR), VP_DT_INV_IZ);
    fp_QP_t Fyf_sin_lf_dt_iz   = fp_mul(Fyf_sin, lf_dt_over_iz);

    fp_QP_t B20_raw = fp_mul(dFyf_dd_sin_raw, NEG_VP_DT_INV_MASS) + Fyf_cos_dt;
    fp_QP_t B20_min = fp_mul(dFyf_dd_sin_min, NEG_VP_DT_INV_MASS) + Fyf_cos_dt;
    B_fr[2][0] = use_front_raw ? B20_raw : B20_min;

    fp_QP_t B30_raw = fp_mul(dFyf_dd_cos_raw, VP_DT_INV_MASS) + (-Fyf_sin_dt);
    fp_QP_t B30_min = fp_mul(dFyf_dd_cos_min, VP_DT_INV_MASS) + (-Fyf_sin_dt);
    B_fr[3][0] = use_front_raw ? B30_raw : B30_min;

    fp_QP_t B40_raw = fp_mul(dFyf_dd_cos_raw, lf_dt_over_iz) + (-Fyf_sin_lf_dt_iz);
    fp_QP_t B40_min = fp_mul(dFyf_dd_cos_min, lf_dt_over_iz) + (-Fyf_sin_lf_dt_iz);
    B_fr[4][0] = use_front_raw ? B40_raw : B40_min;

    B_fr[2][1] = MPC_DT;

    /* B matrix — acceleration / load transfer */
    {
        fp_QP_t F_zf = VP_FZ_FRONT - tr.Fz_transfer;
        fp_QP_t F_zr = VP_FZ_REAR  + tr.Fz_transfer;
        if (F_zf < FP_QP_CONST(1.0)) F_zf = FP_QP_CONST(1.0);
        if (F_zr < FP_QP_CONST(1.0)) F_zr = FP_QP_CONST(1.0);

        fp_QP_t inv_Fzf        = fp_recip(F_zf);
        fp_QP_t inv_Fzr        = fp_recip(F_zr);
        fp_QP_t C_Sf_norm_raw  = fp_mul(tr.C_eff_f_raw, inv_Fzf);
        fp_QP_t C_Sf_norm_min  = fp_mul(tr.C_min_f,     inv_Fzf);
        fp_QP_t C_Sr_norm      = fp_mul(tr.C_eff_r,     inv_Fzr);

        fp_QP_t dFzf_da = -VP_FZ_LOAD_GAIN;
        fp_QP_t dFzr_da =  VP_FZ_LOAD_GAIN;

        bool use_front_norm_raw = (C_Sf_norm_raw > C_Sf_norm_min);
        fp_QP_t dFyf_da_raw = fp_mul(fp_mul(C_Sf_norm_raw, tr.alpha_f_op), dFzf_da);
        fp_QP_t dFyf_da_min = fp_mul(fp_mul(C_Sf_norm_min, tr.alpha_f_op), dFzf_da);
        fp_QP_t dFyr_da     = fp_mul(fp_mul(C_Sr_norm,     tr.alpha_r_op), dFzr_da);
        fp_QP_t dFyf_da_raw_cos = fp_mul(dFyf_da_raw, tr.cos_delta);
        fp_QP_t dFyf_da_min_cos = fp_mul(dFyf_da_min, tr.cos_delta);

        fp_QP_t B31_raw = fp_mul((fp_QP_t)(dFyf_da_raw_cos + dFyr_da), VP_DT_INV_MASS);
        fp_QP_t B31_min = fp_mul((fp_QP_t)(dFyf_da_min_cos + dFyr_da), VP_DT_INV_MASS);
        B_fr[3][1] = use_front_norm_raw ? B31_raw : B31_min;

        fp_QP_t lr_dFyr_da_dt_iz = fp_mul(dFyr_da, neg_lr_dt_over_iz);
        fp_QP_t B41_raw = fp_mul(dFyf_da_raw_cos, lf_dt_over_iz) + lr_dFyr_da_dt_iz;
        fp_QP_t B41_min = fp_mul(dFyf_da_min_cos, lf_dt_over_iz) + lr_dFyr_da_dt_iz;
        B_fr[4][1] = use_front_norm_raw ? B41_raw : B41_min;
    }

    /* Nonlinear rollout */
    fp_QP_t Fx = fp_mul(VP_MASS, a_cmd);
    fp_rollout_from_forces_hls(
        ey, epsi,
        tr.vx_safe, vy, omega,
        kappa, MPC_DT,
        Fx, tr.F_yf, tr.F_yr,
        tr.sin_delta, tr.cos_delta,
        next_state);
}

void compute_frenet_AB_and_next_hls(
    fp_QP_t ey, fp_QP_t epsi,
    fp_QP_t vx, fp_QP_t vy, fp_QP_t omega,
    fp_QP_t delta, fp_QP_t a_cmd,
    fp_QP_t kappa,
    fp_QP_t A_fr[MPC_NX_FRENET][MPC_NX_FRENET],
    fp_QP_t B_fr[MPC_NX_FRENET][MPC_NU],
    fp_QP_t next_state[MPC_NX_FRENET])
{
#pragma HLS INLINE off
    compute_frenet_AB_hls(
        ey, epsi,
        vx, vy, omega,
        delta, a_cmd,
        kappa,
        A_fr, B_fr,
        next_state);
}
/**
 * @brief Predict one Frenet-step forward using the same plant terms as the linearization.
 * @param ey Current lateral error.
 * @param epsi Current heading error.
 * @param vx Current longitudinal velocity.
 * @param vy Current lateral velocity.
 * @param omega Current yaw rate.
 * @param delta Current steering angle.
 * @param a_cmd Current longitudinal acceleration.
 * @param kappa Path curvature at the current point.
 * @param dt Discretization step.
 * @param next_state Output next-state array [e_y, e_psi, vx, vy, omega].
 * @return None.
 */
void predict_frenet_next_hls(
    fp_QP_t ey, fp_QP_t epsi,
    fp_QP_t vx, fp_QP_t vy, fp_QP_t omega,
    fp_QP_t delta, fp_QP_t a_cmd,
    fp_QP_t kappa, fp_QP_t dt,
    fp_QP_t next_state[MPC_NX_FRENET])
{
#pragma HLS INLINE off

    fp_QP_t vx_safe = (vx > MIN_LIN_VEL) ? vx : MIN_LIN_VEL;
    fp_QP_t inv_vx = fp_recip(vx_safe);

    fp_QP_t cos_delta, sin_delta;
    fp_QP_t abs_delta = fp_abs(delta);
    if (abs_delta < FP_QP_CONST(0.25)) {
        fp_QP_t d2 = fp_mul(delta, delta);
        cos_delta = FP_ONE - (d2 >> 1);
        fp_QP_t d3 = fp_mul(d2, delta);
        sin_delta = delta - fp_mul(d3, INV_FACT_3);
    } else {
        cos_delta = fp_cos(delta);
        sin_delta = fp_sin(delta);
    }

    fp_QP_t Fx = fp_mul(VP_MASS, a_cmd);
    fp_QP_t D_transfer = fp_mul(a_cmd, VP_D_LOAD_GAIN);
    fp_QP_t D_pac_f = VP_D_FRONT + (-D_transfer);
    fp_QP_t D_pac_r = VP_D_REAR + D_transfer;

    fp_QP_t lf_omega, lr_omega, front_num, rear_num;
    fp_QP_t front_ratio, rear_ratio;
    fp_slip_terms(vy, omega, inv_vx,
                  &lf_omega, &lr_omega,
                  &front_num, &rear_num,
                  &front_ratio, &rear_ratio);
    fp_QP_t alpha_f_op = delta - fp_atan_tire_approx(front_ratio);
    fp_QP_t rear_atan = fp_atan_tire_approx(rear_ratio);
    fp_QP_t alpha_r_op = -rear_atan;

    fp_QP_t Ba_f = fp_mul(VP_B_FRONT, alpha_f_op);
    fp_QP_t inner_f = fp_pacejka_inner_arg(Ba_f);
    fp_QP_t sin_inner_f, cos_inner_f_unused;
    fp_trig_pair(inner_f, &sin_inner_f, &cos_inner_f_unused);
    fp_QP_t F_yf = fp_mul(D_pac_f, sin_inner_f);

    /* Rear tire — Pacejka effective stiffness */
    fp_QP_t Ba_r = fp_mul(VP_B_REAR, alpha_r_op);
    fp_QP_t inner_r = fp_pacejka_inner_arg(Ba_r);
    fp_QP_t sin_inner_r, cos_inner_r;
    fp_trig_pair(inner_r, &sin_inner_r, &cos_inner_r);
    fp_QP_t F_yr = fp_mul(D_pac_r, sin_inner_r);
    fp_QP_t ba_r2 = fp_sq(Ba_r);
    fp_QP_t inv_denom_r = fp_recip(FP_ONE + ba_r2);

    fp_rollout_from_forces_hls(
        ey, epsi,
        vx_safe, vy, omega,
        kappa, dt,
        Fx, F_yf, F_yr,
        sin_delta, cos_delta,
        next_state);
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
void saturate_control_hls(
    fp_QP_t steer_in, fp_QP_t accel_in,
    fp_QP_t *steer_out, fp_QP_t *accel_out)
{
#pragma HLS INLINE
    *steer_out = fp_clamp(steer_in, -VP_MAX_STEER, VP_MAX_STEER);
    *accel_out = fp_clamp(accel_in, VP_MIN_ACCEL, VP_MAX_ACCEL);
}
