/**
 * @file vehicle_model_hls.c
 * @brief Frenet-Frame Vehicle Model Linearization for HLS
 *
 * Computes the 5x5 Frenet A matrix and 5x2 B matrix for the MPC.
 * Uses sim-matching equations (linear tire model, no Pacejka).
 * All vehicle parameters are compile-time constants.
 *
 * Frenet state: [e_y, e_psi, v_x, v_y, omega]
 * Control: [delta (steering angle), acceleration]
 */

#include "../include/fp_math_hls.h"
#include "../include/mpc_fpga_types.h"

/**
 * Compute Frenet-frame linearization matrices.
 *
 * Combines global linearization rows 3-5 (body dynamics) with
 * Frenet kinematic rows 0-1, using compile-time vehicle parameters.
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
void compute_frenet_AB_hls(
    fixed_point_t vx, fixed_point_t vy, fixed_point_t omega,
    fixed_point_t delta, fixed_point_t a_cmd,
    fixed_point_t kappa, fixed_point_t dt,
    fixed_point_t A_fr[MPC_NX_FRENET][MPC_NX_FRENET],
    fixed_point_t B_fr[MPC_NX_FRENET][MPC_NU])
{
#pragma HLS INLINE

    /* Velocity floor for numerical stability */
    fixed_point_t vx_safe = (vx > FP_CONST(0.5)) ? vx : FP_CONST(0.5);
    fixed_point_t inv_vx = fp_recip(vx_safe);
    fixed_point_t inv_vx2 = fp_mul(inv_vx, inv_vx);

    /* Longitudinal force for load transfer: Fx = m * a_cmd */
    fixed_point_t Fx = fp_mul(VP_MASS, a_cmd);

    /* Normal forces */
    fixed_point_t mg = fp_mul(VP_MASS, VP_GRAVITY);
    fixed_point_t F_zf = fp_mul(
        fp_sub(fp_mul(mg, VP_LR), fp_mul(Fx, VP_CG_HEIGHT)), VP_INV_L);
    fixed_point_t F_zr = fp_mul(
        fp_add(fp_mul(mg, VP_LF), fp_mul(Fx, VP_CG_HEIGHT)), VP_INV_L);

    /* Effective cornering stiffness: C_eff = mu * C_S * F_z */
    fixed_point_t C_eff_f = fp_mul(VP_MU, fp_mul(VP_CSF, F_zf));
    fixed_point_t C_eff_r = fp_mul(VP_MU, fp_mul(VP_CSR, F_zr));

    /* Slip angle Jacobians (sim-matching: linear, no atan) */
    fixed_point_t front_num = fp_add(vy, fp_mul(VP_LF, omega));
    fixed_point_t rear_num  = fp_sub(vy, fp_mul(VP_LR, omega));

    fixed_point_t daf_dvx = fp_mul(front_num, inv_vx2);
    fixed_point_t daf_dvy = fp_neg(inv_vx);
    fixed_point_t daf_dom = fp_neg(fp_mul(VP_LF, inv_vx));

    fixed_point_t dar_dvx = fp_mul(rear_num, inv_vx2);
    fixed_point_t dar_dvy = fp_neg(inv_vx);
    fixed_point_t dar_dom = fp_mul(VP_LR, inv_vx);

    /* Tire force Jacobians */
    fixed_point_t dFyf_dvx = fp_mul(C_eff_f, daf_dvx);
    fixed_point_t dFyf_dvy = fp_mul(C_eff_f, daf_dvy);
    fixed_point_t dFyf_dom = fp_mul(C_eff_f, daf_dom);
    fixed_point_t dFyr_dvx = fp_mul(C_eff_r, dar_dvx);
    fixed_point_t dFyr_dvy = fp_mul(C_eff_r, dar_dvy);
    fixed_point_t dFyr_dom = fp_mul(C_eff_r, dar_dom);
    fixed_point_t dFyf_dd  = C_eff_f;  /* dFyf/d(delta) = C_eff_f */

    /* ============================================================
     * Initialize A to zero, then fill non-zero entries
     * ============================================================ */
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
    A_fr[0][1] = fp_mul(dt, vx_safe);
    A_fr[0][3] = dt;

    /* --- Row 1: e_psi dynamics ---
     * e_psi[k+1] = e_psi[k] + dt*(omega - kappa*vx) */
    A_fr[1][1] = FP_ONE;
    A_fr[1][2] = fp_neg(fp_mul(dt, kappa));
    A_fr[1][4] = dt;

    /* --- Row 2: vx dynamics (global row 3) ---
     * dvx/dt = a_cmd + vy*omega  (sim matching)
     * A[2][2] = 1 + dt*0 = 1 (dFx/dvx = 0 for accel model) */
    A_fr[2][2] = FP_ONE;
    A_fr[2][3] = fp_mul(dt, omega);
    A_fr[2][4] = fp_mul(dt, vy);

    /* --- Row 3: vy dynamics (global row 4) ---
     * dvy/dt = (Fyf + Fyr)/m - vx*omega */
    A_fr[3][2] = fp_mul(dt, fp_mul(
        fp_sub(fp_add(dFyf_dvx, dFyr_dvx), fp_mul(VP_MASS, omega)),
        VP_INV_MASS));
    A_fr[3][3] = fp_add(FP_ONE, fp_mul(dt, fp_mul(
        fp_add(dFyf_dvy, dFyr_dvy), VP_INV_MASS)));
    A_fr[3][4] = fp_mul(dt, fp_mul(
        fp_sub(fp_add(dFyf_dom, dFyr_dom), fp_mul(VP_MASS, vx_safe)),
        VP_INV_MASS));

    /* --- Row 4: omega dynamics (global row 5) ---
     * domega/dt = (lf*Fyf - lr*Fyr) / Iz */
    A_fr[4][2] = fp_mul(dt, fp_mul(
        fp_sub(fp_mul(VP_LF, dFyf_dvx), fp_mul(VP_LR, dFyr_dvx)),
        VP_INV_IZ));
    A_fr[4][3] = fp_mul(dt, fp_mul(
        fp_sub(fp_mul(VP_LF, dFyf_dvy), fp_mul(VP_LR, dFyr_dvy)),
        VP_INV_IZ));
    A_fr[4][4] = fp_add(FP_ONE, fp_mul(dt, fp_mul(
        fp_sub(fp_mul(VP_LF, dFyf_dom), fp_mul(VP_LR, dFyr_dom)),
        VP_INV_IZ)));

    /* ============================================================
     * B matrix: steering and acceleration effects
     * ============================================================ */

    /* B[3][0] = dt * C_eff_f / mass (steering -> vy) */
    B_fr[3][0] = fp_mul(dt, fp_mul(dFyf_dd, VP_INV_MASS));

    /* B[4][0] = dt * lf * C_eff_f / Iz (steering -> omega) */
    B_fr[4][0] = fp_mul(dt, fp_mul(fp_mul(VP_LF, dFyf_dd), VP_INV_IZ));

    /* B[2][1] = dt (acceleration -> vx directly) */
    B_fr[2][1] = dt;

    /* B[0:1][*] = 0: steering/accel don't directly change e_y or e_psi */
    /* B[3][1] = 0, B[4][1] = 0: accel doesn't directly change vy/omega */

    (void)delta;  /* Unused in sim-matching model for B (no cos/sin on delta) */
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
