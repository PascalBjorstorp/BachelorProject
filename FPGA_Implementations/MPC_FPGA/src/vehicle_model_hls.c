/**
 * @file vehicle_model_hls.c
 * @brief Frenet-Frame Vehicle Model Linearization for HLS
 *
 * Computes the 5x5 Frenet A matrix and 5x2 B matrix for the MPC.
 * Uses full real-hardware physics:
 *   - atan-based slip angle Jacobians (not small-angle approximation)
 *   - Pacejka-like tire saturation for effective cornering stiffness
 *   - cos(δ)/sin(δ) force resolution in dynamics
 * All vehicle parameters are compile-time constants.
 *
 * Frenet state: [e_y, e_psi, v_x, v_y, omega]
 * Control: [delta (steering angle), acceleration]
 */

#include "../include/fp_math_hls.h"
#include "../include/mpc_fpga_types.h"

/**
 * Compute Frenet-frame linearization matrices (real-hardware model).
 *
 * Combines global linearization rows 3-5 (body dynamics) with
 * Frenet kinematic rows 0-1, using compile-time vehicle parameters.
 * Uses full atan-based slip angles and Pacejka tire saturation.
 *
 * @param vx     Longitudinal velocity (>= MIN_LIN_VEL expected)
 * @param vy     Lateral velocity
 * @param omega  Yaw rate
 * @param delta  Operating steering angle (feedforward)
 * @param a_cmd  Operating acceleration (for load transfer)
 * @param kappa  Path curvature at this point
 * @param dt     Time step
 * @param A_fr   Output: 5x5 Frenet state transition matrix
 * @param B_fr   Output: 5x2 Frenet input matrix
 */
/**
 * Vehicle-model multiply with guaranteed 3-cycle pipelined DSP.
 * Non-inline to prevent HLS from chaining DSP blocks combinationally.
 * This breaks 20+ ns paths (4-chained DSP48E2) that cause WNS=-10ns.
 */
static fixed_point_t fp_mul_vm(fixed_point_t a, fixed_point_t b) {
#pragma HLS INLINE off
#pragma HLS LATENCY min=3 max=3
    int64_t product = (int64_t)a * (int64_t)b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp latency=3
    return (fixed_point_t)(product >> FP_FRAC_BITS);
}

void compute_frenet_AB_hls(
    fixed_point_t vx, fixed_point_t vy, fixed_point_t omega,
    fixed_point_t delta, fixed_point_t a_cmd,
    fixed_point_t kappa, fixed_point_t dt,
    fixed_point_t A_fr[MPC_NX_FRENET][MPC_NX_FRENET],
    fixed_point_t B_fr[MPC_NX_FRENET][MPC_NU])
{
/* Un-inline vehicle model: keeps multiplier resources separate from riccati_pass.
 * limit=2 serializes most multiplies — only 0.4% of total MPC runtime, so latency
 * increase is negligible. Frees ~40 DSP for the Riccati backward pass bottleneck.
 * fp_mul_vm limit=1: all Pacejka multiplies share one 4-cycle pipelined DSP unit.
 * mul limit=2: allows 2 concurrent raw multiplier operations (enough for fp_recip). */
#pragma HLS INLINE off
#pragma HLS ALLOCATION operation instances=mul limit=2
#pragma HLS ALLOCATION function instances=fp_mul_vm limit=1

    /* Velocity floor for numerical stability */
    fixed_point_t vx_safe = (vx > FP_CONST(0.5)) ? vx : FP_CONST(0.5);
    fixed_point_t inv_vx = fp_recip(vx_safe);

    /* Trig of steering angle — Taylor approx for small |δ| < 0.4 rad:
     *   sin(x) ≈ x - x³/6    (error < 0.03% at δ=0.4)
     *   cos(x) ≈ 1 - x²/2    (error < 0.12% at δ=0.4)
     * Saves ~20 fp_mul vs full polynomial sin/cos computation. */
    fixed_point_t cos_delta, sin_delta;
    if (delta == 0) {
        cos_delta = FP_ONE;
        sin_delta = 0;
    } else {
        fixed_point_t d2 = fp_mul_vm(delta, delta);       /* δ² */
        cos_delta = FP_ONE - (d2 >> 1);                /* 1 - δ²/2 */
        fixed_point_t d3 = fp_mul_vm(d2, delta);           /* δ³ */
        sin_delta = delta - fp_mul_vm(d3, FP_CONST(0.16666667)); /* δ - δ³/6 */
    }

    /* Longitudinal force for load transfer: Fx = m * a_cmd */
    fixed_point_t Fx = fp_mul_vm(VP_MASS, a_cmd);

    /* Normal forces with load transfer */
    fixed_point_t mg = fp_mul_vm(VP_MASS, VP_GRAVITY);
    fixed_point_t F_zf = fp_mul_vm(
        fp_sub(fp_mul_vm(mg, VP_LR), fp_mul_vm(Fx, VP_CG_HEIGHT)), VP_INV_L);
    fixed_point_t F_zr = fp_mul_vm(
        fp_add(fp_mul_vm(mg, VP_LF), fp_mul_vm(Fx, VP_CG_HEIGHT)), VP_INV_L);

    /* ================================================================
     * Full atan-based slip angle Jacobians
     *
     * alpha_f = delta - atan((vy + lf*omega) / vx)
     * alpha_r = -atan((vy - lr*omega) / vx)
     *
     * d(atan(n/d))/dx = (d * dn/dx - n * dd/dx) / (d^2 + n^2)
     * ================================================================ */
    fixed_point_t front_num = fp_add(vy, fp_mul_vm(VP_LF, omega));
    fixed_point_t rear_num  = fp_sub(vy, fp_mul_vm(VP_LR, omega));

    fixed_point_t vx2 = fp_mul_vm(vx_safe, vx_safe);
    fixed_point_t front_num2 = fp_mul_vm(front_num, front_num);
    fixed_point_t rear_num2  = fp_mul_vm(rear_num, rear_num);

    fixed_point_t D_f = fp_add(vx2, front_num2);
    fixed_point_t D_r = fp_add(vx2, rear_num2);
    if (D_f == 0) D_f = FP_ONE;
    if (D_r == 0) D_r = FP_ONE;

    /* Use reciprocal instead of division for FPGA efficiency */
    fixed_point_t inv_D_f = fp_recip(D_f);
    fixed_point_t inv_D_r = fp_recip(D_r);

    /* dα_f/dvx = front_num / (vx^2 + front_num^2) */
    fixed_point_t daf_dvx = fp_mul_vm(front_num, inv_D_f);
    /* dα_f/dvy = -vx / (vx^2 + front_num^2) */
    fixed_point_t daf_dvy = fp_neg(fp_mul_vm(vx_safe, inv_D_f));
    /* dα_f/dω  = -lf * vx / (vx^2 + front_num^2) */
    fixed_point_t daf_dom = fp_neg(fp_mul_vm(fp_mul_vm(VP_LF, vx_safe), inv_D_f));

    /* dα_r/dvx = rear_num / (vx^2 + rear_num^2) */
    fixed_point_t dar_dvx = fp_mul_vm(rear_num, inv_D_r);
    /* dα_r/dvy = -vx / (vx^2 + rear_num^2) */
    fixed_point_t dar_dvy = fp_neg(fp_mul_vm(vx_safe, inv_D_r));
    /* dα_r/dω  = lr * vx / (vx^2 + rear_num^2) */
    fixed_point_t dar_dom = fp_mul_vm(fp_mul_vm(VP_LR, vx_safe), inv_D_r);

    /* ================================================================
     * Pacejka-like tire saturation for effective cornering stiffness
     *
     * F_y = D * sin(C * atan(B * alpha))
     * C_eff = dF_y/dalpha evaluated at operating-point slip angle
     *
     * Also computes F_yf at operating point (needed for B matrix).
     * ================================================================ */

    /* Slip angles at operating point (full atan model) */
    fixed_point_t front_ratio = fp_mul_vm(front_num, inv_vx);
    fixed_point_t rear_ratio  = fp_mul_vm(rear_num, inv_vx);

    fixed_point_t alpha_f_op = fp_sub(delta, fp_atan(front_ratio));
    fixed_point_t alpha_r_op = fp_neg(fp_atan(rear_ratio));

    /* Front tire — Pacejka effective stiffness (B_f precomputed) */
    fixed_point_t B_f = VP_B_FRONT;
    fixed_point_t D_pac_f = fp_mul_vm(VP_MU, F_zf);
    fixed_point_t Ba_f = fp_mul_vm(B_f, alpha_f_op);
    fixed_point_t inner_f = fp_mul_vm(VP_C_SHAPE, fp_atan(Ba_f));
    fixed_point_t cos_inner_f = fp_cos(inner_f);
    fixed_point_t denom_f = fp_add(FP_ONE, fp_mul_vm(Ba_f, Ba_f));
    fixed_point_t inv_denom_f = fp_recip(denom_f);

    /* C_eff_f = D * C * B * cos(C*atan(B*α)) / (1 + (B*α)^2) */
    fixed_point_t C_eff_f = fp_mul_vm(
        fp_mul_vm(D_pac_f, VP_CB_FRONT),
        fp_mul_vm(cos_inner_f, inv_denom_f));
    fixed_point_t C_min_f = fp_mul_vm(F_zf, VP_MU_CSF_MIN);
    C_eff_f = (C_eff_f > C_min_f) ? C_eff_f : C_min_f;

    /* F_yf at operating point (for B matrix cos/sin terms) */
    fixed_point_t F_yf = fp_mul_vm(D_pac_f, fp_sin(inner_f));

    /* Rear tire — Pacejka effective stiffness (B_r precomputed) */
    fixed_point_t B_r = VP_B_REAR;
    fixed_point_t D_pac_r = fp_mul_vm(VP_MU, F_zr);
    fixed_point_t Ba_r = fp_mul_vm(B_r, alpha_r_op);
    fixed_point_t inner_r = fp_mul_vm(VP_C_SHAPE, fp_atan(Ba_r));
    fixed_point_t cos_inner_r = fp_cos(inner_r);
    fixed_point_t denom_r = fp_add(FP_ONE, fp_mul_vm(Ba_r, Ba_r));
    fixed_point_t inv_denom_r = fp_recip(denom_r);

    fixed_point_t C_eff_r = fp_mul_vm(
        fp_mul_vm(D_pac_r, VP_CB_REAR),
        fp_mul_vm(cos_inner_r, inv_denom_r));
    fixed_point_t C_min_r = fp_mul_vm(F_zr, VP_MU_CSR_MIN);
    C_eff_r = (C_eff_r > C_min_r) ? C_eff_r : C_min_r;

    /* ================================================================
     * Tire force Jacobians w.r.t. body states
     * ================================================================ */
    fixed_point_t dFyf_dvx = fp_mul_vm(C_eff_f, daf_dvx);
    fixed_point_t dFyf_dvy = fp_mul_vm(C_eff_f, daf_dvy);
    fixed_point_t dFyf_dom = fp_mul_vm(C_eff_f, daf_dom);
    fixed_point_t dFyf_dd  = C_eff_f;  /* dFyf/d(delta) = C_eff_f */

    fixed_point_t dFyr_dvx = fp_mul_vm(C_eff_r, dar_dvx);
    fixed_point_t dFyr_dvy = fp_mul_vm(C_eff_r, dar_dvy);
    fixed_point_t dFyr_dom = fp_mul_vm(C_eff_r, dar_dom);

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

    /* --- Row 0: e_y dynamics ---
     * e_y[k+1] = e_y[k] + dt*(vx*e_psi + vy) */
    A_fr[0][0] = FP_ONE;
    A_fr[0][1] = fp_mul_vm(dt, vx_safe);
    A_fr[0][3] = dt;

    /* --- Row 1: e_psi dynamics ---
     * e_psi[k+1] = e_psi[k] + dt*(omega - kappa*vx) */
    A_fr[1][1] = FP_ONE;
    A_fr[1][2] = fp_neg(fp_mul_vm(dt, kappa));
    A_fr[1][4] = dt;

    /* --- Row 2: vx dynamics (full model with cos/sin delta) ---
     * dvx/dt = (Fx - Fyf*sin(δ) + m*vy*ω) / m
     * A[2][2] = 1 + (-dFyf_dvx * sin(δ)) * dt/m */
    A_fr[2][2] = fp_sub(FP_ONE,
        fp_mul_vm(fp_mul_vm(dFyf_dvx, sin_delta), VP_DT_INV_MASS));
    /* A[2][3] = dt * (-dFyf_dvy * sin(δ) / m + ω) */
    A_fr[2][3] = fp_mul_vm(dt,
        fp_sub(omega, fp_mul_vm(fp_mul_vm(dFyf_dvy, sin_delta), VP_INV_MASS)));
    /* A[2][4] = dt * (-dFyf_dom * sin(δ) / m + vy) */
    A_fr[2][4] = fp_mul_vm(dt,
        fp_sub(vy, fp_mul_vm(fp_mul_vm(dFyf_dom, sin_delta), VP_INV_MASS)));

    /* --- Row 3: vy dynamics (full model) ---
     * dvy/dt = (Fyf*cos(δ) + Fyr - m*vx*ω) / m */
    A_fr[3][2] = fp_mul_vm(
        fp_sub(fp_add(fp_mul_vm(dFyf_dvx, cos_delta), dFyr_dvx),
               fp_mul_vm(VP_MASS, omega)),
        VP_DT_INV_MASS);
    A_fr[3][3] = fp_add(FP_ONE, fp_mul_vm(
        fp_add(fp_mul_vm(dFyf_dvy, cos_delta), dFyr_dvy),
        VP_DT_INV_MASS));
    A_fr[3][4] = fp_mul_vm(
        fp_sub(fp_add(fp_mul_vm(dFyf_dom, cos_delta), dFyr_dom),
               fp_mul_vm(VP_MASS, vx_safe)),
        VP_DT_INV_MASS);

    /* --- Row 4: omega dynamics (full model) ---
     * dω/dt = (lf*Fyf*cos(δ) - lr*Fyr) / Iz */
    A_fr[4][2] = fp_mul_vm(
        fp_sub(fp_mul_vm(VP_LF, fp_mul_vm(dFyf_dvx, cos_delta)),
               fp_mul_vm(VP_LR, dFyr_dvx)),
        VP_DT_INV_IZ);
    A_fr[4][3] = fp_mul_vm(
        fp_sub(fp_mul_vm(VP_LF, fp_mul_vm(dFyf_dvy, cos_delta)),
               fp_mul_vm(VP_LR, dFyr_dvy)),
        VP_DT_INV_IZ);
    A_fr[4][4] = fp_add(FP_ONE, fp_mul_vm(
        fp_sub(fp_mul_vm(VP_LF, fp_mul_vm(dFyf_dom, cos_delta)),
               fp_mul_vm(VP_LR, dFyr_dom)),
        VP_DT_INV_IZ));

    /* ================================================================
     * B matrix: steering and acceleration effects
     * Full model with cos(δ)/sin(δ) force resolution
     * ================================================================ */

    /* Precompute common subexpressions for B-matrix steering column */
    fixed_point_t dFyf_dd_sin = fp_mul_vm(dFyf_dd, sin_delta);
    fixed_point_t Fyf_cos     = fp_mul_vm(F_yf, cos_delta);
    fixed_point_t dFyf_dd_cos = fp_mul_vm(dFyf_dd, cos_delta);
    fixed_point_t Fyf_sin     = fp_mul_vm(F_yf, sin_delta);

    /* B[2][0]: d(dvx/dt)/dδ = (-dFyf_dd*sin(δ) - Fyf*cos(δ)) * dt/m */
    B_fr[2][0] = fp_neg(fp_mul_vm(fp_add(dFyf_dd_sin, Fyf_cos), VP_DT_INV_MASS));

    /* B[3][0]: d(dvy/dt)/dδ = (dFyf_dd*cos(δ) - Fyf*sin(δ)) * dt/m */
    B_fr[3][0] = fp_mul_vm(fp_sub(dFyf_dd_cos, Fyf_sin), VP_DT_INV_MASS);

    /* B[4][0]: d(dω/dt)/dδ = lf*(dFyf_dd*cos(δ) - Fyf*sin(δ)) * dt/Iz */
    B_fr[4][0] = fp_mul_vm(fp_mul_vm(VP_LF, fp_sub(dFyf_dd_cos, Fyf_sin)), VP_DT_INV_IZ);

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
 */
void saturate_control_hls(
    fixed_point_t steer_in, fixed_point_t accel_in,
    fixed_point_t *steer_out, fixed_point_t *accel_out)
{
#pragma HLS INLINE
    *steer_out = fp_clamp(steer_in, fp_neg(VP_MAX_STEER), VP_MAX_STEER);
    *accel_out = fp_clamp(accel_in, VP_MIN_ACCEL, VP_MAX_ACCEL);
}
