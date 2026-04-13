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
#pragma HLS LATENCY min=4 max=4
    fp_QP_t product = a * b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp latency=4
    return product;
}

static fp_QP_t fp_atan_tire_vm(fp_QP_t x) {
#pragma HLS INLINE off
#pragma HLS PIPELINE II=1
    return fp_atan_tire_approx(x);
}

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
    fp_QP_t vx, fp_QP_t vy, fp_QP_t omega,
    fp_QP_t delta, fp_QP_t a_cmd,
    fp_QP_t kappa, fp_QP_t dt,
    fp_QP_t A_fr[MPC_NX_FRENET][MPC_NX_FRENET],
    fp_QP_t B_fr[MPC_NX_FRENET][MPC_NU])
{
/* Keep this function as a separate scheduled block with bounded multiplier use. */
#pragma HLS INLINE off
#pragma HLS ALLOCATION operation instances=mul limit=MPC_HLS_VEHICLE_MUL_LIMIT
#pragma HLS ALLOCATION function instances=fp_mul_vm limit=2

    /* Velocity floor for numerical stability */
    fp_QP_t vx_safe = (vx > FP_QP_CONST(0.5)) ? vx : FP_QP_CONST(0.5);
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
    fp_QP_t inner_f = fp_mul_vm(VP_C_SHAPE, fp_atan(Ba_f));
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
    fp_QP_t inner_r = fp_mul_vm(VP_C_SHAPE, fp_atan(Ba_r));
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

    /* --- Row 0: e_y dynamics ---
     * e_y[k+1] = e_y[k] + dt*(vx*e_psi + vy) */
    A_fr[0][0] = FP_ONE;
    A_fr[0][1] = fp_mul_vm(dt, vx);
    A_fr[0][3] = dt;

    /* --- Row 1: e_psi dynamics ---
     * e_psi[k+1] = e_psi[k] + dt*(omega - kappa*vx) */
    A_fr[1][1] = FP_ONE;
    A_fr[1][2] = -fp_mul_vm(dt, kappa);
    A_fr[1][4] = dt;

    /* --- Row 2: vx dynamics (full model with cos/sin delta) ---
     * dvx/dt = (Fx - Fyf*sin(δ) + m*vy*ω) / m
     * A[2][2] = 1 + (-dFyf_dvx * sin(δ)) * dt/m */
    A_fr[2][2] = FP_ONE
        - fp_mul_vm(fp_mul_vm(dFyf_dvx, sin_delta), VP_DT_INV_MASS);
    /* A[2][3] = dt * (-dFyf_dvy * sin(δ) / m + ω) */
    A_fr[2][3] = fp_mul_vm(dt,
        omega - fp_mul_vm(fp_mul_vm(dFyf_dvy, sin_delta), VP_INV_MASS));
    /* A[2][4] = dt * (-dFyf_dom * sin(δ) / m + vy) */
    A_fr[2][4] = fp_mul_vm(dt,
        vy - fp_mul_vm(fp_mul_vm(dFyf_dom, sin_delta), VP_INV_MASS));

    /* --- Row 3: vy dynamics (full model) ---
     * dvy/dt = (Fyf*cos(δ) + Fyr - m*vx*ω) / m */
    A_fr[3][2] = fp_mul_vm(
        fp_mul_vm(dFyf_dvx, cos_delta) + dFyr_dvx - fp_mul_vm(VP_MASS, omega),
        VP_DT_INV_MASS);
    A_fr[3][3] = FP_ONE + fp_mul_vm(
        fp_mul_vm(dFyf_dvy, cos_delta) + dFyr_dvy,
        VP_DT_INV_MASS);
    A_fr[3][4] = fp_mul_vm(
        fp_mul_vm(dFyf_dom, cos_delta) + dFyr_dom - fp_mul_vm(VP_MASS, vx),
        VP_DT_INV_MASS);

    /* --- Row 4: omega dynamics (full model) ---
     * dω/dt = (lf*Fyf*cos(δ) - lr*Fyr) / Iz */
    A_fr[4][2] = fp_mul_vm(
        fp_mul_vm(VP_LF, fp_mul_vm(dFyf_dvx, cos_delta))
        - fp_mul_vm(VP_LR, dFyr_dvx),
        VP_DT_INV_IZ);
    A_fr[4][3] = fp_mul_vm(
        fp_mul_vm(VP_LF, fp_mul_vm(dFyf_dvy, cos_delta))
        - fp_mul_vm(VP_LR, dFyr_dvy),
        VP_DT_INV_IZ);
    A_fr[4][4] = FP_ONE + fp_mul_vm(
        fp_mul_vm(VP_LF, fp_mul_vm(dFyf_dom, cos_delta))
        - fp_mul_vm(VP_LR, dFyr_dom),
        VP_DT_INV_IZ);

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

    /* B[0:1][*] = 0: steering/accel don't directly change e_y or e_psi */
    /* B[3][1] = 0, B[4][1] = 0: accel doesn't directly change vy/omega */
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
