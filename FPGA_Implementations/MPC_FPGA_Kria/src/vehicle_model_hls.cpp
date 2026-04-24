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

static fp_QP_t fp_rear_pacejka_ratio_eff(fp_QP_t rear_ratio)
{
#pragma HLS INLINE off
    fp_QP_t rear_ratio2 = fp_mul(rear_ratio, rear_ratio);
    fp_QP_t rear_ratio3 = fp_mul(rear_ratio2, rear_ratio);
    fp_QP_t rear_atan_corr = fp_mul(rear_ratio3, FP_QP_CONST(0.33333333));
    return (-rear_ratio) + rear_atan_corr;
}

static fp_QP_t fp_rear_pacejka_b_scale(fp_QP_t rear_ratio_eff)
{
#pragma HLS INLINE off
    return fp_mul(VP_B_REAR, rear_ratio_eff);
}

static fp_QP_t fp_pacejka_inner_arg(fp_QP_t ba)
{
#pragma HLS INLINE off
    fp_QP_t atan_ba = fp_atan_tire_approx(ba);
    return fp_mul(VP_C_SHAPE, atan_ba);
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

void predict_frenet_next_hls(
    fp_QP_t ey, fp_QP_t epsi,
    fp_QP_t vx, fp_QP_t vy, fp_QP_t omega,
    fp_QP_t delta, fp_QP_t a_cmd,
    fp_QP_t kappa, fp_QP_t dt,
    fp_QP_t next_state[MPC_NX_FRENET]);

/**
 * @brief Compute Frenet-frame linearized dynamics matrices.
 * @param vx Operating-point longitudinal velocity (expected >= MIN_LIN_VEL).
 * @param vy Operating-point lateral velocity.
 * @param omega Operating-point yaw rate.
 * @param delta Operating-point steering angle.
 * @param a_cmd Operating-point longitudinal acceleration.
 * @param kappa Path curvature at the linearization point.
 * @param dt Discretization step.
 * @param A_fr Output 5x5 Frenet state transition matrix.
 * @param B_fr Output 5x2 input matrix.
 * @return None.
 */
void compute_frenet_AB_hls(
    fp_QP_t ey, fp_QP_t epsi,
    fp_QP_t vx, fp_QP_t vy, fp_QP_t omega,
    fp_QP_t delta, fp_QP_t a_cmd,
    fp_QP_t kappa,
    fp_QP_t A_fr[MPC_NX_FRENET][MPC_NX_FRENET],
    fp_QP_t B_fr[MPC_NX_FRENET][MPC_NU],
    fp_QP_t next_state[MPC_NX_FRENET])
{
#pragma HLS INLINE

    /* The MPC setup clamps vx before this timing-critical linearization call. */
    fp_QP_t vx_safe = vx;
    fp_QP_t inv_vx = fp_recip(vx_safe);

    /* Always-on polynomial trig approximation to avoid synthesizing
     * transcendental trig paths in this timing-critical kernel. */
    fp_QP_t cos_delta, sin_delta;
    fp_QP_t d2 = fp_mul(delta, delta);
    fp_QP_t d3 = fp_mul(d2, delta);
    fp_QP_t d4 = fp_mul(d2, d2);
    fp_QP_t d5 = fp_mul(d4, delta);
    cos_delta = FP_ONE - fp_mul(d2, INV_FACT_2)
              + fp_mul(d4, INV_FACT_4);
    sin_delta = delta - fp_mul(d3, INV_FACT_3)
              + fp_mul(d5, INV_FACT_5);

    /* Load transfer as one multiply from acceleration. Pacejka D uses the
     * same transfer scaled by mu, computed independently to avoid F_z -> D
     * serial multiplier chains in the inlined setup loop. */
    fp_QP_t Fz_transfer = fp_mul(a_cmd, VP_FZ_LOAD_GAIN);
    fp_QP_t D_transfer = fp_mul(a_cmd, VP_D_LOAD_GAIN);
    fp_QP_t Cmin_transfer_f = fp_mul(a_cmd, VP_CMIN_FRONT_LOAD_GAIN);
    fp_QP_t Cmin_transfer_r = fp_mul(a_cmd, VP_CMIN_REAR_LOAD_GAIN);

    fp_QP_t F_zf = VP_FZ_FRONT - Fz_transfer;
    fp_QP_t F_zr = VP_FZ_REAR + Fz_transfer;
    fp_QP_t D_pac_f = VP_D_FRONT + (-D_transfer);
    fp_QP_t D_pac_r = VP_D_REAR + D_transfer;
    fp_QP_t C_min_f = VP_CMIN_FRONT_STATIC - Cmin_transfer_f;
    fp_QP_t C_min_r = VP_CMIN_REAR_STATIC + Cmin_transfer_r;

    /* ================================================================
     * Slip angle Jacobians
     *
     * alpha_f = delta - atan_approx((vy + lf*omega) / vx)
     * alpha_r = -atan_approx((vy - lr*omega) / vx)
     *
     * d(atan_approx(n/d))/dx ~= (d * dn/dx - n * dd/dx) / (d^2 + n^2)
     * ================================================================ */
    fp_QP_t lf_omega = fp_mul(VP_LF, omega);
    fp_QP_t lr_omega = fp_mul(VP_LR, omega);
    fp_QP_t front_num = vy + lf_omega;
    fp_QP_t rear_num  = vy + (-lr_omega);

    fp_QP_t vx2 = fp_mul(vx_safe, vx_safe);
    fp_QP_t vy2 = fp_mul(vy, vy);
    fp_QP_t lf_omega2 = fp_mul(lf_omega, lf_omega);
    fp_QP_t lr_omega2 = fp_mul(lr_omega, lr_omega);
    fp_QP_t front_cross = fp_mul(vy, lf_omega);
    fp_QP_t rear_cross = fp_mul(vy, lr_omega);
    fp_QP_t front_num2 = vy2 + (front_cross + front_cross) + lf_omega2;
    fp_QP_t rear_num2  = vy2 + (rear_cross + rear_cross) + lr_omega2;

    fp_QP_t D_f = vx2 + front_num2;
    fp_QP_t D_r = vx2 + rear_num2;

    fp_QP_t inv_D_f = fp_recip(D_f);

    fp_QP_t inv_D_r = fp_recip(D_r);

    /* ================================================================
     * Pacejka-like tire saturation for effective cornering stiffness
     *
     * F_y = D * sin(C * atan_approx(B * alpha))
     * C_eff = dF_y/dalpha evaluated at operating-point slip angle
     *
     * Also computes F_yf at operating point (needed for B matrix).
     * ================================================================ */

    /* Slip angles at operating point */
    fp_QP_t front_ratio = fp_mul(front_num, inv_vx);
    fp_QP_t rear_ratio  = fp_mul(rear_num, inv_vx);
    fp_QP_t alpha_f_op = delta - fp_atan_tire_approx(front_ratio);
    fp_QP_t rear_atan = fp_atan_tire_approx(rear_ratio);
    fp_QP_t alpha_r_op = -rear_atan;

    /* Front tire — Pacejka effective stiffness (B_f precomputed) */
    fp_QP_t Ba_f = fp_mul(VP_B_FRONT, alpha_f_op);
    fp_QP_t inner_f = fp_pacejka_inner_arg(Ba_f);
    fp_QP_t cos_inner_f = fp_cos(inner_f);
    fp_QP_t ba_f2 = fp_mul(Ba_f, Ba_f);
    fp_QP_t inv_denom_f = fp_recip(FP_ONE + ba_f2);
    fp_QP_t D_pac_f_cb_base = fp_mul(VP_D_FRONT, VP_CB_FRONT);
    fp_QP_t D_pac_f_cb_delta = fp_mul(D_transfer, VP_CB_FRONT);
    fp_QP_t D_pac_f_cb = D_pac_f_cb_base + (-D_pac_f_cb_delta);
    fp_QP_t cos_over_denom_f = fp_mul(cos_inner_f, inv_denom_f);
    fp_QP_t C_eff_f_raw = fp_mul(D_pac_f_cb, cos_over_denom_f);

    /* F_yf at operating point (for B matrix terms). */
    fp_QP_t sin_inner_f = fp_sin(inner_f);
    fp_QP_t F_yf = fp_mul(D_pac_f, sin_inner_f);

    /* Rear tire — Pacejka effective stiffness */
    fp_QP_t rear_ratio_eff = fp_rear_pacejka_ratio_eff(rear_ratio);
    fp_QP_t Ba_r = fp_rear_pacejka_b_scale(rear_ratio_eff);
    fp_QP_t inner_r = fp_pacejka_inner_arg(Ba_r);
    fp_QP_t cos_inner_r = fp_cos(inner_r);
    fp_QP_t sin_inner_r = fp_sin(inner_r);
    fp_QP_t F_yr = fp_mul(D_pac_r, sin_inner_r);
    fp_QP_t ba_r2 = fp_mul(Ba_r, Ba_r);
    fp_QP_t inv_denom_r = fp_recip(FP_ONE + ba_r2);

    fp_QP_t D_pac_r_cb_base = fp_mul(VP_D_REAR, VP_CB_REAR);
    fp_QP_t D_pac_r_cb_delta = fp_mul(D_transfer, VP_CB_REAR);
    fp_QP_t D_pac_r_cb = D_pac_r_cb_base + D_pac_r_cb_delta;
    fp_QP_t cos_over_denom_r = fp_mul(cos_inner_r, inv_denom_r);
    fp_QP_t C_eff_r = fp_mul(D_pac_r_cb, cos_over_denom_r);
    C_eff_r = (C_eff_r > C_min_r) ? C_eff_r : C_min_r;

    /* ================================================================
     * Tire force Jacobians w.r.t. body states
     * ================================================================ */
    fp_QP_t C_eff_f = (C_eff_f_raw > C_min_f) ? C_eff_f_raw : C_min_f;
    fp_QP_t dFyf_dvx, dFyf_dvy, dFyf_dom;
    fp_front_force_jacobians(C_eff_f, front_num, vx_safe, inv_D_f,
                             &dFyf_dvx, &dFyf_dvy, &dFyf_dom);
    fp_QP_t dFyr_dvx, dFyr_dvy, dFyr_dom;
    fp_rear_force_jacobians(C_eff_r, rear_num, vx_safe, inv_D_r,
                            &dFyr_dvx, &dFyr_dvy, &dFyr_dom);

    /* ================================================================
     * Initialize A to zero, then fill non-zero entries
     * ================================================================ */
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

    /* --- Rows 0/1: exact operating-point Frenet Jacobian --- */
    {
        fp_QP_t cp = fp_cos(epsi);
        fp_QP_t sp = fp_sin(epsi);
        fp_QP_t denom = FP_ONE - fp_mul(kappa, ey);
        fp_QP_t dt_cp = fp_mul(MPC_DT, cp);
        fp_QP_t kappa_dt = fp_mul(kappa, MPC_DT);
        fp_QP_t kappa2 = fp_mul(kappa, kappa);

        if (fp_abs(denom) < FP_QP_CONST(1e-3)) {
            denom = (denom >= 0) ? FP_QP_CONST(1e-3) : FP_QP_CONST(-1e-3);
        }
        fp_QP_t inv_denom = fp_recip(denom);
        fp_QP_t inv_denom2 = fp_mul(inv_denom, inv_denom);
        fp_QP_t vx_cp = fp_mul(vx, cp);
        fp_QP_t vy_sp = fp_mul(vy, sp);

        A_fr[0][0] = FP_ONE;
        A_fr[0][1] = fp_mul(MPC_DT, vx_cp + (-vy_sp));
        A_fr[0][2] = fp_mul(MPC_DT, sp);
        A_fr[0][3] = dt_cp;

        fp_QP_t k2_vx = fp_mul(kappa2, vx);
        fp_QP_t a10_pre = fp_mul(dt_cp, k2_vx);
        A_fr[1][0] = fp_mul(-a10_pre, inv_denom2);

        fp_QP_t kdt_vx = fp_mul(kappa_dt, vx);
        fp_QP_t a11_pre = fp_mul(kdt_vx, sp);
        A_fr[1][1] = FP_ONE + fp_mul(a11_pre, inv_denom);

        fp_QP_t a12_pre = fp_mul(kappa_dt, cp);
        A_fr[1][2] = fp_mul(-a12_pre, inv_denom);
        A_fr[1][4] = MPC_DT;
    }

    /* Stage shared products to reduce arithmetic depth in row assembly. */
    fp_QP_t dFyf_dvx_sin = fp_mul(dFyf_dvx, sin_delta);
    fp_QP_t dFyf_dvy_sin = fp_mul(dFyf_dvy, sin_delta);
    fp_QP_t dFyf_dom_sin = fp_mul(dFyf_dom, sin_delta);
    fp_QP_t vx_damping = fp_mul(dFyf_dvx_sin, VP_DT_INV_MASS);
    fp_QP_t vy_damping = fp_mul(dFyf_dvy_sin, VP_INV_MASS);
    fp_QP_t om_damping = fp_mul(dFyf_dom_sin, VP_INV_MASS);
    fp_QP_t neg_vx_damping = -vx_damping;
    fp_QP_t neg_vy_damping = -vy_damping;
    fp_QP_t neg_om_damping = -om_damping;

    fp_QP_t dFyf_dvx_cos = fp_mul(dFyf_dvx, cos_delta);
    fp_QP_t dFyf_dvy_cos = fp_mul(dFyf_dvy, cos_delta);
    fp_QP_t dFyf_dom_cos = fp_mul(dFyf_dom, cos_delta);
    fp_QP_t mass_omega = fp_mul(VP_MASS, omega);
    fp_QP_t mass_vx = fp_mul(VP_MASS, vx);
    fp_QP_t neg_mass_omega = -mass_omega;
    fp_QP_t neg_mass_vx = -mass_vx;

    fp_QP_t lf_dFyf_dvx_cos = fp_mul(VP_LF, dFyf_dvx_cos);
    fp_QP_t lf_dFyf_dvy_cos = fp_mul(VP_LF, dFyf_dvy_cos);
    fp_QP_t lf_dFyf_dom_cos = fp_mul(VP_LF, dFyf_dom_cos);
    fp_QP_t lr_dFyr_dvx = fp_mul(VP_LR, dFyr_dvx);
    fp_QP_t lr_dFyr_dvy = fp_mul(VP_LR, dFyr_dvy);
    fp_QP_t lr_dFyr_dom = fp_mul(VP_LR, dFyr_dom);
    fp_QP_t neg_lr_dFyr_dvx = -lr_dFyr_dvx;
    fp_QP_t neg_lr_dFyr_dvy = -lr_dFyr_dvy;
    fp_QP_t neg_lr_dFyr_dom = -lr_dFyr_dom;

    /* --- Row 2: vx dynamics (full model with cos/sin delta) ---
     * dvx/dt = (Fx - Fyf*sin(δ) + m*vy*ω) / m
     * A[2][2] = 1 + (-dFyf_dvx * sin(δ)) * dt/m */
    A_fr[2][2] = FP_ONE + neg_vx_damping;
    /* A[2][3] = dt * (-dFyf_dvy * sin(δ) / m + ω) */
    A_fr[2][3] = fp_mul(MPC_DT, omega + neg_vy_damping);
    /* A[2][4] = dt * (-dFyf_dom * sin(δ) / m + vy) */
    A_fr[2][4] = fp_mul(MPC_DT, vy + neg_om_damping);

    /* --- Row 3: vy dynamics (full model) ---
     * dvy/dt = (Fyf*cos(δ) + Fyr - m*vx*ω) / m */
    A_fr[3][2] = fp_mul(dFyf_dvx_cos + dFyr_dvx + neg_mass_omega, VP_DT_INV_MASS);
    A_fr[3][3] = FP_ONE + fp_mul(dFyf_dvy_cos + dFyr_dvy, VP_DT_INV_MASS);
    A_fr[3][4] = fp_mul(dFyf_dom_cos + dFyr_dom + neg_mass_vx, VP_DT_INV_MASS);

    /* --- Row 4: omega dynamics (full model) ---
     * dω/dt = (lf*Fyf*cos(δ) - lr*Fyr) / Iz */
    A_fr[4][2] = fp_mul(lf_dFyf_dvx_cos + neg_lr_dFyr_dvx, VP_DT_INV_IZ);
    A_fr[4][3] = fp_mul(lf_dFyf_dvy_cos + neg_lr_dFyr_dvy, VP_DT_INV_IZ);
    A_fr[4][4] = FP_ONE + fp_mul(lf_dFyf_dom_cos + neg_lr_dFyr_dom, VP_DT_INV_IZ);

    /* ================================================================
     * B matrix: steering and acceleration effects
     * Full model with cos(δ)/sin(δ) force resolution
     * ================================================================ */

    /* Precompute branch-specific B-matrix terms so the select happens after
     * the expensive multiplies, not before them. */
    bool use_front_raw = (C_eff_f_raw > C_min_f);
    fp_QP_t dFyf_dd_sin_raw = fp_mul(C_eff_f_raw, sin_delta);
    fp_QP_t dFyf_dd_sin_min = fp_mul(C_min_f, sin_delta);
    fp_QP_t Fyf_cos     = fp_mul(F_yf, cos_delta);
    fp_QP_t dFyf_dd_cos_raw = fp_mul(C_eff_f_raw, cos_delta);
    fp_QP_t dFyf_dd_cos_min = fp_mul(C_min_f, cos_delta);
    fp_QP_t Fyf_sin     = fp_mul(F_yf, sin_delta);
    fp_QP_t Fyf_cos_dt = fp_mul(Fyf_cos, NEG_VP_DT_INV_MASS);
    fp_QP_t Fyf_sin_dt = fp_mul(Fyf_sin, VP_DT_INV_MASS);
    fp_QP_t lf_dt_over_iz = fp_mul(VP_LF, VP_DT_INV_IZ);
    fp_QP_t Fyf_sin_lf_dt_iz = fp_mul(Fyf_sin, lf_dt_over_iz);

    /* B[2][0]: d(dvx/dt)/dδ = (dFyf_dd*sin(δ) + Fyf*cos(δ)) * dt/m with sign folded into NEG_VP_DT_INV_MASS */
    fp_QP_t B20_raw = fp_mul(dFyf_dd_sin_raw, NEG_VP_DT_INV_MASS) + Fyf_cos_dt;
    fp_QP_t B20_min = fp_mul(dFyf_dd_sin_min, NEG_VP_DT_INV_MASS) + Fyf_cos_dt;
    B_fr[2][0] = use_front_raw ? B20_raw : B20_min;

    /* B[3][0]: d(dvy/dt)/dδ = (dFyf_dd*cos(δ) - Fyf*sin(δ)) * dt/m */
    fp_QP_t B30_raw = fp_mul(dFyf_dd_cos_raw, VP_DT_INV_MASS) + (-Fyf_sin_dt);
    fp_QP_t B30_min = fp_mul(dFyf_dd_cos_min, VP_DT_INV_MASS) + (-Fyf_sin_dt);
    B_fr[3][0] = use_front_raw ? B30_raw : B30_min;

    /* B[4][0]: d(dω/dt)/dδ = lf*(dFyf_dd*cos(δ) - Fyf*sin(δ)) * dt/Iz */
    fp_QP_t B40_raw = fp_mul(dFyf_dd_cos_raw, lf_dt_over_iz) + (-Fyf_sin_lf_dt_iz);
    fp_QP_t B40_min = fp_mul(dFyf_dd_cos_min, lf_dt_over_iz) + (-Fyf_sin_lf_dt_iz);
    B_fr[4][0] = use_front_raw ? B40_raw : B40_min;

    /* B[2][1] = dt (acceleration → vx directly) */
    B_fr[2][1] = MPC_DT;

    /* B[3][1], B[4][1]: acceleration affects lateral dynamics through load transfer. */
    {
        fp_QP_t inv_Fzf = fp_recip(F_zf);
        fp_QP_t inv_Fzr = fp_recip(F_zr);
        fp_QP_t C_Sf_norm_raw = fp_mul(C_eff_f_raw, inv_Fzf);
        fp_QP_t C_Sf_norm_min = fp_mul(C_min_f, inv_Fzf);
        fp_QP_t C_Sr_norm = fp_mul(C_eff_r, inv_Fzr);

        fp_QP_t dFzf_da = -VP_FZ_LOAD_GAIN;
        fp_QP_t dFzr_da = VP_FZ_LOAD_GAIN;

        bool use_front_norm_raw = (C_Sf_norm_raw > C_Sf_norm_min);
        fp_QP_t dFyf_da_raw = fp_mul(fp_mul(C_Sf_norm_raw, alpha_f_op), dFzf_da);
        fp_QP_t dFyf_da_min = fp_mul(fp_mul(C_Sf_norm_min, alpha_f_op), dFzf_da);
        fp_QP_t dFyr_da = fp_mul(fp_mul(C_Sr_norm, alpha_r_op), dFzr_da);

        fp_QP_t B31_raw = fp_mul(
            fp_mul((fp_mul(dFyf_da_raw, cos_delta) + dFyr_da), VP_INV_MASS),
            MPC_DT);
        fp_QP_t B31_min = fp_mul(
            fp_mul((fp_mul(dFyf_da_min, cos_delta) + dFyr_da), VP_INV_MASS),
            MPC_DT);
        B_fr[3][1] = use_front_norm_raw ? B31_raw : B31_min;

        fp_QP_t B41_raw = fp_mul(
            fp_mul(
                (fp_mul(fp_mul(VP_LF, dFyf_da_raw), cos_delta) + (-fp_mul(VP_LR, dFyr_da))),
                VP_INV_IZ),
            MPC_DT);
        fp_QP_t B41_min = fp_mul(
            fp_mul(
                (fp_mul(fp_mul(VP_LF, dFyf_da_min), cos_delta) + (-fp_mul(VP_LR, dFyr_da))),
                VP_INV_IZ),
            MPC_DT);
        B_fr[4][1] = use_front_norm_raw ? B41_raw : B41_min;
    }

    /* B[0:1][*] = 0: steering/accel don't directly change e_y or e_psi */

    /* Nonlinear rollout for affine-term consistency. This shares the tire,
     * load-transfer, and delta trig terms already needed for linearization. */
    fp_QP_t ey_denom = FP_ONE + (-fp_mul(kappa, ey));
    if (fp_abs(ey_denom) < FP_QP_CONST(1e-3)) {
        ey_denom = (ey_denom >= 0) ? FP_QP_CONST(1e-3) : FP_QP_CONST(-1e-3);
    }

    fp_QP_t sin_epsi = fp_sin(epsi);
    fp_QP_t cos_epsi = fp_cos(epsi);
    fp_QP_t inv_ey_denom = fp_recip(ey_denom);
    fp_QP_t Fx = fp_mul(VP_MASS, a_cmd);

    fp_QP_t vx_sin_epsi = fp_mul(vx_safe, sin_epsi);
    fp_QP_t vy_cos_epsi = fp_mul(vy, cos_epsi);
    fp_QP_t vx_cos_epsi = fp_mul(vx_safe, cos_epsi);
    fp_QP_t kappa_vx_cos = fp_mul(kappa, vx_cos_epsi);
    fp_QP_t kappa_vx_cos_inv = fp_mul(kappa_vx_cos, inv_ey_denom);
    fp_QP_t fx_sin_delta = fp_mul(F_yf, sin_delta);
    fp_QP_t vx_omega = fp_mul(vx_safe, omega);
    fp_QP_t fyf_cos = fp_mul(F_yf, cos_delta);
    fp_QP_t dvy_term = fyf_cos + F_yr;
    fp_QP_t domega_lf = fp_mul(VP_LF, fyf_cos);
    fp_QP_t domega_lr = fp_mul(VP_LR, F_yr);
    fp_QP_t dvx_dt = (Fx + (-fx_sin_delta)) * VP_INV_MASS + fp_mul(vy, omega);
    fp_QP_t dvy_dt = dvy_term * VP_INV_MASS + (-vx_omega);
    fp_QP_t domega_step = (domega_lf + (-domega_lr)) * VP_DT_INV_IZ;
    fp_QP_t e_y_step_vx = fp_mul(MPC_DT, vx_sin_epsi);
    fp_QP_t e_y_step_vy = fp_mul(MPC_DT, vy_cos_epsi);
    fp_QP_t e_psi_step_omega = fp_mul(MPC_DT, omega);
    fp_QP_t e_psi_step_corr = fp_mul(MPC_DT, kappa_vx_cos_inv);
    fp_QP_t dvx_step = fp_mul(MPC_DT, dvx_dt);
    fp_QP_t dvy_step = fp_mul(MPC_DT, dvy_dt);

    next_state[0] = ey + e_y_step_vx + e_y_step_vy;
    next_state[1] = epsi + e_psi_step_omega + (-e_psi_step_corr);
    next_state[2] = vx_safe + dvx_step;
    if (next_state[2] < MIN_LIN_VEL) next_state[2] = MIN_LIN_VEL;
    next_state[3] = vy + dvy_step;
    next_state[4] = omega + domega_step;
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
#pragma HLS INLINE
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

    fp_QP_t lf_omega = fp_mul(VP_LF, omega);
    fp_QP_t lr_omega = fp_mul(VP_LR, omega);
    fp_QP_t front_num = vy + lf_omega;
    fp_QP_t rear_num  = vy + (-lr_omega);
    fp_QP_t front_ratio = fp_mul(front_num, inv_vx);
    fp_QP_t rear_ratio  = fp_mul(rear_num, inv_vx);
    fp_QP_t alpha_f_op = delta - fp_atan_tire_approx(front_ratio);
    fp_QP_t rear_atan = fp_atan_tire_approx(rear_ratio);
    fp_QP_t alpha_r_op = -rear_atan;

    fp_QP_t Ba_f = fp_mul(VP_B_FRONT, alpha_f_op);
    fp_QP_t inner_f = fp_pacejka_inner_arg(Ba_f);
    fp_QP_t sin_inner_f = fp_sin(inner_f);
    fp_QP_t F_yf = fp_mul(D_pac_f, sin_inner_f);

    /* Rear tire — Pacejka effective stiffness */
    fp_QP_t rear_ratio_eff = fp_rear_pacejka_ratio_eff(rear_ratio);
    fp_QP_t Ba_r = fp_rear_pacejka_b_scale(rear_ratio_eff);
    fp_QP_t inner_r = fp_pacejka_inner_arg(Ba_r);
    fp_QP_t cos_inner_r = fp_cos(inner_r);
    fp_QP_t sin_inner_r = fp_sin(inner_r);
    fp_QP_t F_yr = fp_mul(D_pac_r, sin_inner_r);
    fp_QP_t ba_r2 = fp_mul(Ba_r, Ba_r);
    fp_QP_t inv_denom_r = fp_recip(FP_ONE + ba_r2);

    fp_QP_t ey_denom = FP_ONE - fp_mul(kappa, ey);
    fp_QP_t ey_denom_abs = fp_abs(ey_denom);
    if (ey_denom_abs < FP_QP_CONST(1e-3)) {
        ey_denom = (ey_denom >= 0) ? FP_QP_CONST(1e-3) : FP_QP_CONST(-1e-3);
    }

    fp_QP_t sin_epsi = fp_sin(epsi);
    fp_QP_t cos_epsi = fp_cos(epsi);
    fp_QP_t inv_ey_denom = fp_recip(ey_denom);

    fp_QP_t vx_sin_epsi = fp_mul(vx, sin_epsi);
    fp_QP_t vy_cos_epsi = fp_mul(vy, cos_epsi);
    fp_QP_t vx_cos_epsi = fp_mul(vx, cos_epsi);
    fp_QP_t kappa_vx_cos = fp_mul(kappa, vx_cos_epsi);
    fp_QP_t kappa_vx_cos_inv = fp_mul(kappa_vx_cos, inv_ey_denom);
    fp_QP_t fx_sin_delta = fp_mul(F_yf, sin_delta);
    fp_QP_t vx_omega = fp_mul(vx, omega);
    fp_QP_t fyf_cos = fp_mul(F_yf, cos_delta);
    fp_QP_t dvy_term = fyf_cos + F_yr;
    fp_QP_t domega_lf = fp_mul(VP_LF, fyf_cos);
    fp_QP_t domega_lr = fp_mul(VP_LR, F_yr);
    fp_QP_t dvx_dt = (Fx - fx_sin_delta) * VP_INV_MASS + vx_omega;
    fp_QP_t dvy_dt = dvy_term * VP_INV_MASS - vx_omega;
    fp_QP_t domega_step = (domega_lf - domega_lr) * VP_DT_INV_IZ;
    fp_QP_t e_y_step_vx = fp_mul(dt, vx_sin_epsi);
    fp_QP_t e_y_step_vy = fp_mul(dt, vy_cos_epsi);
    fp_QP_t e_psi_step_omega = fp_mul(dt, omega);
    fp_QP_t e_psi_step_corr = fp_mul(dt, kappa_vx_cos_inv);
    fp_QP_t dvx_step = fp_mul(dt, dvx_dt);
    fp_QP_t dvy_step = fp_mul(dt, dvy_dt);

    next_state[0] = ey + e_y_step_vx + e_y_step_vy;
    next_state[1] = epsi + e_psi_step_omega + (-e_psi_step_corr);
    next_state[2] = vx + dvx_step;
    if (next_state[2] < MIN_LIN_VEL) next_state[2] = MIN_LIN_VEL;
    next_state[3] = vy + dvy_step;
    next_state[4] = omega + domega_step;
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
