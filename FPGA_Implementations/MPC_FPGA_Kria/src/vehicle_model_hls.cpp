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

/* HLS-tuned multiply wrapper used in model hot paths to preserve
 * predictable operator binding and latency. */
static fp_QP_t fp_mul_vm(fp_QP_t a, fp_QP_t b) {
#pragma HLS INLINE off
#pragma HLS LATENCY min=MPC_HLS_VM_MUL_LATENCY max=MPC_HLS_VM_MUL_LATENCY
    fp_QP_t product = a * b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp latency=MPC_HLS_VM_MUL_LATENCY
    return product;
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
    fp_QP_t kappa, fp_QP_t dt,
    fp_QP_t A_fr[MPC_NX_FRENET][MPC_NX_FRENET],
    fp_QP_t B_fr[MPC_NX_FRENET][MPC_NU])
{
/* Keep this function as a separate scheduled block with bounded multiplier use. */
#pragma HLS INLINE off
#pragma HLS ALLOCATION operation instances=mul limit=MPC_HLS_VEHICLE_MUL_LIMIT
#pragma HLS ALLOCATION function instances=fp_mul_vm limit=MPC_HLS_VEHICLE_MUL_LIMIT
#pragma HLS ALLOCATION function instances=fp_recip limit=4

    /* Velocity floor for numerical stability (shared with controller tuning). */
    fp_QP_t vx_safe = (vx > MIN_LIN_VEL) ? vx : MIN_LIN_VEL;
    fp_QP_t inv_vx = fp_recip(vx_safe);

    /* Hybrid steering trig:
     * - fast Taylor path for small steering angles
     * - exact shared kernels outside the small-angle region
     */
    fp_QP_t cos_delta, sin_delta;
    fp_QP_t abs_delta = fp_abs(delta);
    if (abs_delta < FP_QP_CONST(0.25)) {
        fp_QP_t d2 = fp_mul_vm(delta, delta);
        cos_delta = FP_ONE - (d2 >> 1);
        fp_QP_t d3 = fp_mul_vm(d2, delta);
        sin_delta = delta - fp_mul_vm(d3, FP_QP_CONST(0.16666667));
    } else {
        cos_delta = fp_cos(delta);
        sin_delta = fp_sin(delta);
    }

    /* Longitudinal force for load transfer: Fx = m * a_cmd */
    fp_QP_t Fx = fp_mul_vm(VP_MASS, a_cmd);

    /* Normal forces with load transfer (constant gravity terms precomputed). */
    fp_QP_t F_zf = fp_mul_vm(
        VP_MG_LR - fp_mul_vm(Fx, VP_CG_HEIGHT), VP_INV_L);
    fp_QP_t F_zr = fp_mul_vm(
        VP_MG_LF + fp_mul_vm(Fx, VP_CG_HEIGHT), VP_INV_L);

    /* ================================================================
     * Slip angle Jacobians
     *
     * alpha_f = delta - atan((vy + lf*omega) / vx)
     * alpha_r = -atan((vy - lr*omega) / vx)
     *
     * d(atan(n/d))/dx = (d * dn/dx - n * dd/dx) / (d^2 + n^2)
     * ================================================================ */
    fp_QP_t front_num = vy + fp_mul_vm(VP_LF, omega);
    fp_QP_t rear_num  = vy - fp_mul_vm(VP_LR, omega);

    fp_QP_t vx2 = fp_mul_vm(vx_safe, vx_safe);
    fp_QP_t front_num2 = fp_mul_vm(front_num, front_num);
    fp_QP_t rear_num2  = fp_mul_vm(rear_num, rear_num);

    fp_QP_t D_f = vx2 + front_num2;
    fp_QP_t D_r = vx2 + rear_num2;
    if (D_f == 0) D_f = FP_ONE;
    if (D_r == 0) D_r = FP_ONE;

    fp_QP_t inv_D_f = fp_recip(D_f);
    fp_QP_t inv_D_r = fp_recip(D_r);

    fp_QP_t daf_dvx = fp_mul_vm(front_num, inv_D_f);
    fp_QP_t daf_dvy = -fp_mul_vm(vx_safe, inv_D_f);
    fp_QP_t daf_dom = -fp_mul_vm(fp_mul_vm(VP_LF, vx_safe), inv_D_f);

    fp_QP_t dar_dvx = fp_mul_vm(rear_num, inv_D_r);
    fp_QP_t dar_dvy = -fp_mul_vm(vx_safe, inv_D_r);
    fp_QP_t dar_dom = fp_mul_vm(fp_mul_vm(VP_LR, vx_safe), inv_D_r);

    /* ================================================================
     * Pacejka-like tire saturation for effective cornering stiffness
     *
     * F_y = D * sin(C * atan(B * alpha))
     * C_eff = dF_y/dalpha evaluated at operating-point slip angle
     *
     * Also computes F_yf at operating point (needed for B matrix).
     * ================================================================ */

    /* Slip angles at operating point */
    fp_QP_t front_ratio = fp_mul_vm(front_num, inv_vx);
    fp_QP_t rear_ratio  = fp_mul_vm(rear_num, inv_vx);
    fp_QP_t alpha_f_op = delta - fp_atan(front_ratio);
    fp_QP_t alpha_r_op = -fp_atan(rear_ratio);

    /* Front tire — Pacejka effective stiffness (B_f precomputed) */
    fp_QP_t D_pac_f = fp_mul_vm(VP_MU, F_zf);
    fp_QP_t Ba_f = fp_mul_vm(VP_B_FRONT, alpha_f_op);
    fp_QP_t atan_ba_f = (fp_abs(Ba_f) < FP_QP_CONST(2.0))
                            ? fp_atan_tire_approx(Ba_f)
                            : fp_atan(Ba_f);
    fp_QP_t inner_f = fp_mul_vm(VP_C_SHAPE, atan_ba_f);
    fp_QP_t cos_inner_f = fp_cos(inner_f);
    fp_QP_t ba_f2 = fp_mul_vm(Ba_f, Ba_f);
    fp_QP_t inv_denom_f = fp_recip(FP_ONE + ba_f2);

    fp_QP_t C_eff_f = fp_mul_vm(
        fp_mul_vm(D_pac_f, VP_CB_FRONT),
        fp_mul_vm(cos_inner_f, inv_denom_f));
    fp_QP_t C_min_f = fp_mul_vm(F_zf, VP_MU_CSF_MIN);
    C_eff_f = (C_eff_f > C_min_f) ? C_eff_f : C_min_f;

    /* F_yf at operating point (for B matrix terms). */
    fp_QP_t sin_inner_f = fp_sin(inner_f);
    fp_QP_t F_yf = fp_mul_vm(D_pac_f, sin_inner_f);

    /* Rear tire — Pacejka effective stiffness */
    fp_QP_t D_pac_r = fp_mul_vm(VP_MU, F_zr);
    fp_QP_t Ba_r = fp_mul_vm(VP_B_REAR, alpha_r_op);
    fp_QP_t atan_ba_r = (fp_abs(Ba_r) < FP_QP_CONST(2.0))
                            ? fp_atan_tire_approx(Ba_r)
                            : fp_atan(Ba_r);
    fp_QP_t inner_r = fp_mul_vm(VP_C_SHAPE, atan_ba_r);
    fp_QP_t cos_inner_r = fp_cos(inner_r);
    fp_QP_t ba_r2 = fp_mul_vm(Ba_r, Ba_r);
    fp_QP_t inv_denom_r = fp_recip(FP_ONE + ba_r2);

    fp_QP_t C_eff_r = fp_mul_vm(
        fp_mul_vm(D_pac_r, VP_CB_REAR),
        fp_mul_vm(cos_inner_r, inv_denom_r));
    fp_QP_t C_min_r = fp_mul_vm(F_zr, VP_MU_CSR_MIN);
    C_eff_r = (C_eff_r > C_min_r) ? C_eff_r : C_min_r;

    /* ================================================================
     * Tire force Jacobians w.r.t. body states
     * ================================================================ */
    fp_QP_t dFyf_dvx = fp_mul_vm(C_eff_f, daf_dvx);
    fp_QP_t dFyf_dvy = fp_mul_vm(C_eff_f, daf_dvy);
    fp_QP_t dFyf_dom = fp_mul_vm(C_eff_f, daf_dom);
    fp_QP_t dFyf_dd  = C_eff_f;  /* dFyf/d(delta) = C_eff_f */

    fp_QP_t dFyr_dvx = fp_mul_vm(C_eff_r, dar_dvx);
    fp_QP_t dFyr_dvy = fp_mul_vm(C_eff_r, dar_dvy);
    fp_QP_t dFyr_dom = fp_mul_vm(C_eff_r, dar_dom);

    /* ================================================================
     * Initialize A to zero, then fill non-zero entries
     * ================================================================ */
    int i, j;
    for (i = 0; i < MPC_NX_FRENET; i++) {
#pragma HLS PIPELINE II=1
        for (j = 0; j < MPC_NX_FRENET; j++) {
            A_fr[i][j] = 0;
        }
        B_fr[i][0] = 0;
        B_fr[i][1] = 0;
    }

    /* --- Rows 0/1: exact operating-point Frenet Jacobian --- */
    {
        fp_QP_t cp = fp_cos(epsi);
        fp_QP_t sp = fp_sin(epsi);
        fp_QP_t denom = FP_ONE - fp_mul_vm(kappa, ey);
        if (fp_abs(denom) < FP_QP_CONST(1e-3)) {
            denom = (denom >= 0) ? FP_QP_CONST(1e-3) : FP_QP_CONST(-1e-3);
        }
        fp_QP_t inv_denom = fp_recip(denom);
        fp_QP_t inv_denom2 = fp_mul_vm(inv_denom, inv_denom);

        A_fr[0][0] = FP_ONE;
        A_fr[0][1] = fp_mul_vm(dt, fp_mul_vm(vx, cp) - fp_mul_vm(vy, sp));
        A_fr[0][2] = fp_mul_vm(dt, sp);
        A_fr[0][3] = fp_mul_vm(dt, cp);

        A_fr[1][0] = -fp_mul_vm(
            fp_mul_vm(fp_mul_vm(fp_mul_vm(dt, fp_mul_vm(kappa, kappa)), vx), cp),
            inv_denom2);
        A_fr[1][1] = FP_ONE + fp_mul_vm(
            fp_mul_vm(fp_mul_vm(fp_mul_vm(dt, kappa), vx), sp),
            inv_denom);
        A_fr[1][2] = -fp_mul_vm(fp_mul_vm(fp_mul_vm(dt, kappa), cp), inv_denom);
        A_fr[1][4] = dt;
    }

    /* Stage shared products to reduce arithmetic depth in row assembly. */
    fp_QP_t dFyf_dvx_sin = fp_mul_vm(dFyf_dvx, sin_delta);
    fp_QP_t dFyf_dvy_sin = fp_mul_vm(dFyf_dvy, sin_delta);
    fp_QP_t dFyf_dom_sin = fp_mul_vm(dFyf_dom, sin_delta);
    fp_QP_t vx_damping = fp_mul_vm(dFyf_dvx_sin, VP_DT_INV_MASS);
    fp_QP_t vy_damping = fp_mul_vm(dFyf_dvy_sin, VP_INV_MASS);
    fp_QP_t om_damping = fp_mul_vm(dFyf_dom_sin, VP_INV_MASS);

    fp_QP_t dFyf_dvx_cos = fp_mul_vm(dFyf_dvx, cos_delta);
    fp_QP_t dFyf_dvy_cos = fp_mul_vm(dFyf_dvy, cos_delta);
    fp_QP_t dFyf_dom_cos = fp_mul_vm(dFyf_dom, cos_delta);
    fp_QP_t mass_omega = fp_mul_vm(VP_MASS, omega);
    fp_QP_t mass_vx = fp_mul_vm(VP_MASS, vx);

    fp_QP_t lf_dFyf_dvx_cos = fp_mul_vm(VP_LF, dFyf_dvx_cos);
    fp_QP_t lf_dFyf_dvy_cos = fp_mul_vm(VP_LF, dFyf_dvy_cos);
    fp_QP_t lf_dFyf_dom_cos = fp_mul_vm(VP_LF, dFyf_dom_cos);
    fp_QP_t lr_dFyr_dvx = fp_mul_vm(VP_LR, dFyr_dvx);
    fp_QP_t lr_dFyr_dvy = fp_mul_vm(VP_LR, dFyr_dvy);
    fp_QP_t lr_dFyr_dom = fp_mul_vm(VP_LR, dFyr_dom);

    /* --- Row 2: vx dynamics (full model with cos/sin delta) ---
     * dvx/dt = (Fx - Fyf*sin(δ) + m*vy*ω) / m
     * A[2][2] = 1 + (-dFyf_dvx * sin(δ)) * dt/m */
    A_fr[2][2] = FP_ONE - vx_damping;
    /* A[2][3] = dt * (-dFyf_dvy * sin(δ) / m + ω) */
    A_fr[2][3] = fp_mul_vm(dt, omega - vy_damping);
    /* A[2][4] = dt * (-dFyf_dom * sin(δ) / m + vy) */
    A_fr[2][4] = fp_mul_vm(dt, vy - om_damping);

    /* --- Row 3: vy dynamics (full model) ---
     * dvy/dt = (Fyf*cos(δ) + Fyr - m*vx*ω) / m */
    A_fr[3][2] = fp_mul_vm(dFyf_dvx_cos + dFyr_dvx - mass_omega, VP_DT_INV_MASS);
    A_fr[3][3] = FP_ONE + fp_mul_vm(dFyf_dvy_cos + dFyr_dvy, VP_DT_INV_MASS);
    A_fr[3][4] = fp_mul_vm(dFyf_dom_cos + dFyr_dom - mass_vx, VP_DT_INV_MASS);

    /* --- Row 4: omega dynamics (full model) ---
     * dω/dt = (lf*Fyf*cos(δ) - lr*Fyr) / Iz */
    A_fr[4][2] = fp_mul_vm(lf_dFyf_dvx_cos - lr_dFyr_dvx, VP_DT_INV_IZ);
    A_fr[4][3] = fp_mul_vm(lf_dFyf_dvy_cos - lr_dFyr_dvy, VP_DT_INV_IZ);
    A_fr[4][4] = FP_ONE + fp_mul_vm(lf_dFyf_dom_cos - lr_dFyr_dom, VP_DT_INV_IZ);

    /* ================================================================
     * B matrix: steering and acceleration effects
     * Full model with cos(δ)/sin(δ) force resolution
     * ================================================================ */

    /* Precompute common subexpressions for B-matrix steering column */
    fp_QP_t dFyf_dd_sin = fp_mul_vm(dFyf_dd, sin_delta);
    fp_QP_t Fyf_cos     = fp_mul_vm(F_yf, cos_delta);
    fp_QP_t dFyf_dd_cos = fp_mul_vm(dFyf_dd, cos_delta);
    fp_QP_t Fyf_sin     = fp_mul_vm(F_yf, sin_delta);

    /* B[2][0]: d(dvx/dt)/dδ = (-dFyf_dd*sin(δ) - Fyf*cos(δ)) * dt/m */
    B_fr[2][0] = -fp_mul_vm(dFyf_dd_sin + Fyf_cos, VP_DT_INV_MASS);

    /* B[3][0]: d(dvy/dt)/dδ = (dFyf_dd*cos(δ) - Fyf*sin(δ)) * dt/m */
    B_fr[3][0] = fp_mul_vm(dFyf_dd_cos - Fyf_sin, VP_DT_INV_MASS);

    /* B[4][0]: d(dω/dt)/dδ = lf*(dFyf_dd*cos(δ) - Fyf*sin(δ)) * dt/Iz */
    B_fr[4][0] = fp_mul_vm(fp_mul_vm(VP_LF, dFyf_dd_cos - Fyf_sin), VP_DT_INV_IZ);

    /* B[2][1] = dt (acceleration → vx directly) */
    B_fr[2][1] = dt;

    /* B[3][1], B[4][1]: acceleration affects lateral dynamics through load transfer. */
    {
        fp_QP_t inv_Fzf = fp_recip(F_zf);
        fp_QP_t inv_Fzr = fp_recip(F_zr);
        fp_QP_t C_Sf_norm = fp_mul_vm(C_eff_f, inv_Fzf);
        fp_QP_t C_Sr_norm = fp_mul_vm(C_eff_r, inv_Fzr);

        fp_QP_t dFzf_da = -fp_mul_vm(fp_mul_vm(VP_MASS, VP_CG_HEIGHT), VP_INV_L);
        fp_QP_t dFzr_da = -dFzf_da;

        fp_QP_t dFyf_da = fp_mul_vm(fp_mul_vm(C_Sf_norm, alpha_f_op), dFzf_da);
        fp_QP_t dFyr_da = fp_mul_vm(fp_mul_vm(C_Sr_norm, alpha_r_op), dFzr_da);

        B_fr[3][1] = fp_mul_vm(
            fp_mul_vm((fp_mul_vm(dFyf_da, cos_delta) + dFyr_da), VP_INV_MASS),
            dt);

        B_fr[4][1] = fp_mul_vm(
            fp_mul_vm(
                (fp_mul_vm(fp_mul_vm(VP_LF, dFyf_da), cos_delta) - fp_mul_vm(VP_LR, dFyr_da)),
                VP_INV_IZ),
            dt);
    }

    /* B[0:1][*] = 0: steering/accel don't directly change e_y or e_psi */
}

void compute_frenet_AB_and_next_hls(
    fp_QP_t ey, fp_QP_t epsi,
    fp_QP_t vx, fp_QP_t vy, fp_QP_t omega,
    fp_QP_t delta, fp_QP_t a_cmd,
    fp_QP_t kappa, fp_QP_t dt,
    fp_QP_t A_fr[MPC_NX_FRENET][MPC_NX_FRENET],
    fp_QP_t B_fr[MPC_NX_FRENET][MPC_NU],
    fp_QP_t next_state[MPC_NX_FRENET])
{
#pragma HLS INLINE off
    compute_frenet_AB_hls(
        ey, epsi,
        vx, vy, omega,
        delta, a_cmd,
        kappa, dt,
        A_fr, B_fr);

    /* Use a nonlinear one-step rollout for affine-term consistency in d_k. */
    predict_frenet_next_hls(
        ey, epsi,
        vx, vy, omega,
        delta, a_cmd,
        kappa, dt,
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
#pragma HLS ALLOCATION operation instances=mul limit=MPC_HLS_VEHICLE_MUL_LIMIT
#pragma HLS ALLOCATION function instances=fp_mul_vm limit=MPC_HLS_VEHICLE_MUL_LIMIT
#pragma HLS ALLOCATION function instances=fp_recip limit=4

    fp_QP_t vx_safe = (vx > MIN_LIN_VEL) ? vx : MIN_LIN_VEL;
    fp_QP_t inv_vx = fp_recip(vx_safe);

    fp_QP_t cos_delta, sin_delta;
    fp_QP_t abs_delta = fp_abs(delta);
    if (abs_delta < FP_QP_CONST(0.25)) {
        fp_QP_t d2 = fp_mul_vm(delta, delta);
        cos_delta = FP_ONE - (d2 >> 1);
        fp_QP_t d3 = fp_mul_vm(d2, delta);
        sin_delta = delta - fp_mul_vm(d3, FP_QP_CONST(0.16666667));
    } else {
        cos_delta = fp_cos(delta);
        sin_delta = fp_sin(delta);
    }

    fp_QP_t Fx = fp_mul_vm(VP_MASS, a_cmd);
    fp_QP_t F_zf = fp_mul_vm(VP_MG_LR - fp_mul_vm(Fx, VP_CG_HEIGHT), VP_INV_L);
    fp_QP_t F_zr = fp_mul_vm(VP_MG_LF + fp_mul_vm(Fx, VP_CG_HEIGHT), VP_INV_L);

    fp_QP_t front_num = vy + fp_mul_vm(VP_LF, omega);
    fp_QP_t rear_num  = vy - fp_mul_vm(VP_LR, omega);
    fp_QP_t front_ratio = fp_mul_vm(front_num, inv_vx);
    fp_QP_t rear_ratio  = fp_mul_vm(rear_num, inv_vx);
    fp_QP_t alpha_f_op = delta - fp_atan(front_ratio);
    fp_QP_t alpha_r_op = -fp_atan(rear_ratio);

    fp_QP_t D_pac_f = fp_mul_vm(VP_MU, F_zf);
    fp_QP_t Ba_f = fp_mul_vm(VP_B_FRONT, alpha_f_op);
    fp_QP_t atan_ba_f = (fp_abs(Ba_f) < FP_QP_CONST(2.0))
                            ? fp_atan_tire_approx(Ba_f)
                            : fp_atan(Ba_f);
    fp_QP_t inner_f = fp_mul_vm(VP_C_SHAPE, atan_ba_f);
    fp_QP_t sin_inner_f = fp_sin(inner_f);
    fp_QP_t F_yf = fp_mul_vm(D_pac_f, sin_inner_f);

    fp_QP_t D_pac_r = fp_mul_vm(VP_MU, F_zr);
    fp_QP_t Ba_r = fp_mul_vm(VP_B_REAR, alpha_r_op);
    fp_QP_t atan_ba_r = (fp_abs(Ba_r) < FP_QP_CONST(2.0))
                            ? fp_atan_tire_approx(Ba_r)
                            : fp_atan(Ba_r);
    fp_QP_t inner_r = fp_mul_vm(VP_C_SHAPE, atan_ba_r);
    fp_QP_t sin_inner_r = fp_sin(inner_r);
    fp_QP_t F_yr = fp_mul_vm(D_pac_r, sin_inner_r);

    fp_QP_t ey_denom = FP_ONE - fp_mul_vm(kappa, ey);
    fp_QP_t ey_denom_abs = fp_abs(ey_denom);
    if (ey_denom_abs < FP_QP_CONST(1e-3)) {
        ey_denom = (ey_denom >= 0) ? FP_QP_CONST(1e-3) : FP_QP_CONST(-1e-3);
    }

    fp_QP_t sin_epsi = fp_sin(epsi);
    fp_QP_t cos_epsi = fp_cos(epsi);
    fp_QP_t inv_ey_denom = fp_recip(ey_denom);

    fp_QP_t e_y_dot = fp_mul_vm(vx, sin_epsi) + fp_mul_vm(vy, cos_epsi);
    fp_QP_t e_psi_dot = omega - fp_mul_vm(fp_mul_vm(kappa, fp_mul_vm(vx, cos_epsi)), inv_ey_denom);
    fp_QP_t dvx_dt = (Fx - fp_mul_vm(F_yf, sin_delta)) * VP_INV_MASS + fp_mul_vm(vy, omega);
    fp_QP_t dvy_dt = (fp_mul_vm(F_yf, cos_delta) + F_yr) * VP_INV_MASS - fp_mul_vm(vx, omega);
    fp_QP_t domega_dt = (fp_mul_vm(VP_LF, fp_mul_vm(F_yf, cos_delta)) - fp_mul_vm(VP_LR, F_yr)) * VP_INV_IZ;

    next_state[0] = ey + fp_mul_vm(dt, e_y_dot);
    next_state[1] = epsi + fp_mul_vm(dt, e_psi_dot);
    next_state[2] = vx + fp_mul_vm(dt, dvx_dt);
    if (next_state[2] < MIN_LIN_VEL) next_state[2] = MIN_LIN_VEL;
    next_state[3] = vy + fp_mul_vm(dt, dvy_dt);
    next_state[4] = omega + fp_mul_vm(dt, domega_dt);
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
