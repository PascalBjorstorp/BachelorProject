/**
 * @file mpcc_vehicle_model.c
 * @brief MPCC Vehicle Dynamics Linearization — Implementation
 */

#include "mpcc_vehicle_model.h"
#include <string.h>
#include <stdint.h>

/*===========================================================================
 * Dynamics Linearization (Pacejka Tires)
 *===========================================================================*/

/**
 * Linearize the dynamics (Pacejka tire model).
 *
 * State [7]: [s, vx, vy, omega, X, Y, psi]
 * Control [3]: [delta, a_x, v_theta]
 *
 *   Virtual progress (row 0):
 *     ds/dt     = v_theta                           (virtual progress control)
 *
 *   Vehicle dynamics with Pacejka tires (rows 1-3):
 *     dvx/dt    = (-F_yf*sin(d) + F_xf) / m + vy*omega
 *     dvy/dt    = (F_yf*cos(d) + F_yr) / m - vx*omega
 *     domega/dt = (l_f*F_yf*cos(d) - l_r*F_yr) / I_z
 *
 *     Pacejka tire forces:
 *       F_y = D * sin(C * atan(B * alpha))
 *       D = mu * F_z,  B = C_S / C_shape,  C_shape = 1.9
 *       alpha_f = delta - atan((vy + l_f*omega) / vx)
 *       alpha_r = -atan((vy - l_r*omega) / vx)
 *
 *   Cartesian kinematics (rows 4-6):
 *     dX/dt   = vx*cos(psi) - vy*sin(psi)
 *     dY/dt   = vx*sin(psi) + vy*cos(psi)
 *     dpsi/dt = omega
 *
 * Discretization: Forward Euler  A_d = I + dt*A_c,  B_d = dt*B_c
 * Affine term: d = f(z_bar, u_bar) - A_d*z_bar - B_d*u_bar
 */
void mpcc_linearize_dynamics(
    const MPCCState_t *state,
    const MPCCControl_t *control,
    fixed_point_t kappa,
    fixed_point_t dt,
    const MPCCConfiguration_t *cfg,
    MPCCLinearSystem_t *sys)
{
    memset(sys, 0, sizeof(*sys));

    /* --- Trigonometric values --- */
    fixed_point_t cos_psi = fp_cos(state->psi);
    fixed_point_t sin_psi = fp_sin(state->psi);

    /* --- Vehicle parameters --- */
    fixed_point_t m   = F110_VEHICLE_MASS_KG;
    fixed_point_t l_f = F110_DIST_CG_TO_FRONT_AXLE_METERS;
    fixed_point_t l_r = F110_DIST_CG_TO_REAR_AXLE_METERS;
    fixed_point_t I_z = F110_YAW_INERTIA_KGM2;
    fixed_point_t C_f = cfg->C_Sf;
    fixed_point_t C_r = cfg->C_Sr;

    /* --- Normal loads and Pacejka parameters ---
     *   F_zf = m*g*l_r / (l_f+l_r)   F_zr = m*g*l_f / (l_f+l_r)
     *   D_pac = mu * F_z   (peak lateral force)
     *   B = C_S / C_shape  (Pacejka stiffness factor)
     */
    fixed_point_t g_acc = F110_GRAVITY_ACCELERATION_MS2;
    fixed_point_t L = fp_add(l_f, l_r);
    fixed_point_t F_zf = fp_div(fp_mul(fp_mul(m, g_acc), l_r), L);
    fixed_point_t F_zr = fp_div(fp_mul(fp_mul(m, g_acc), l_f), L);
    fixed_point_t mu = cfg->mu;

    /* Pacejka shape factor and B factors.
     * Standard Pacejka: F_y = D * sin(C * atan(B * alpha))
     * At small alpha: F_y ≈ D * C * B * alpha
     * To match the linear model (F_y = C_S * alpha) at zero slip:
     *   D * C * B = C_S  →  B = C_S / (D * C) = C_S / (mu * F_z * C_shape)
     */
    fixed_point_t C_shape = FP_CONST(1.9);
    fixed_point_t D_pac_f = fp_mul(mu, F_zf);        /* peak front force */
    fixed_point_t D_pac_r = fp_mul(mu, F_zr);        /* peak rear force */
    fixed_point_t B_f = fp_div(C_f, fp_mul(D_pac_f, C_shape));  /* C_Sf / (D_f * C) */
    fixed_point_t B_r = fp_div(C_r, fp_mul(D_pac_r, C_shape));  /* C_Sr / (D_r * C) */

    /* Minimum stiffness clamp (10% of linear C_S) */
    fixed_point_t C_min_f = fp_mul(C_f, FP_CONST(0.1));
    fixed_point_t C_min_r = fp_mul(C_r, FP_CONST(0.1));



    /*=====================================================================================
     * Tire forces and Jacobians (Pacejka model with atan slip angles)
     *=====================================================================================*/
    /* --- Tire forces and Jacobians (rows 1-3) ---
     * Use minimum vx of 0.5 m/s for slip angle calculations.
     * Lower thresholds cause Riccati instability at high speeds due to
     * large tire Jacobian eigenvalues in corners. */
    fixed_point_t vx_abs = fp_abs(state->vx);
    fixed_point_t vx_safe = (vx_abs < FP_CONST(0.5)) ? FP_CONST(0.5) : vx_abs;
    fixed_point_t inv_vx = fp_div(FP_ONE, vx_safe);
    fixed_point_t cos_delta = fp_cos(control->delta);
    fixed_point_t sin_delta = fp_sin(control->delta);

    fixed_point_t vy_plus_lf_w  = fp_add(state->vy, fp_mul(l_f, state->omega));
    fixed_point_t vy_minus_lr_w = fp_sub(state->vy, fp_mul(l_r, state->omega));

    /* Slip angles at operating point (atan model like MPC/MPC_FPGA) */
    fixed_point_t front_ratio = fp_mul(vy_plus_lf_w, inv_vx);
    fixed_point_t rear_ratio  = fp_mul(vy_minus_lr_w, inv_vx);
    fixed_point_t alpha_f = fp_sub(control->delta, fp_atan(front_ratio));
    fixed_point_t alpha_r = fp_sub(0, fp_atan(rear_ratio));

    /* ================================================================
     * Pacejka tire model: F_y = D * sin(C * atan(B * alpha))
     *
     *   C_eff = dF_y/dalpha at operating-point slip angle:
     *   C_eff = D * C * B * cos(C*atan(B*a)) / (1 + (B*a)^2)
     *
     *   F_yf, F_yr at operating point (for B matrix and affine term).
     * ================================================================ */

    /* Front tire — Pacejka effective stiffness */
    fixed_point_t Ba_f = fp_mul(B_f, alpha_f);
    fixed_point_t inner_f = fp_mul(C_shape, fp_atan(Ba_f));
    fixed_point_t inner_f2 = fp_mul(inner_f, inner_f);
    fixed_point_t cos_inner_f = fp_sub(FP_ONE, (inner_f2 >> 1));   /* cos(x) ≈ 1 - x²/2 */
    fixed_point_t ba_f2 = fp_mul(Ba_f, Ba_f);
    fixed_point_t inv_denom_f = fp_div(FP_ONE, fp_add(FP_ONE, ba_f2));
    fixed_point_t C_eff_f = fp_mul(fp_mul(D_pac_f, fp_mul(C_shape, B_f)),
                                   fp_mul(cos_inner_f, inv_denom_f));
    C_eff_f = (C_eff_f > C_min_f) ? C_eff_f : C_min_f;

    /* F_yf at operating point: sin(x) ≈ x - x³/6 */
    fixed_point_t inner_f3 = fp_mul(fp_mul(inner_f, inner_f), inner_f);
    fixed_point_t sin_inner_f = fp_sub(inner_f, fp_mul(inner_f3, FP_CONST(0.16666667)));
    fixed_point_t F_yf = fp_mul(D_pac_f, sin_inner_f);

    /* Rear tire — Pacejka effective stiffness */
    fixed_point_t Ba_r = fp_mul(B_r, alpha_r);
    fixed_point_t inner_r = fp_mul(C_shape, fp_atan(Ba_r));
    fixed_point_t inner_r2 = fp_mul(inner_r, inner_r);
    fixed_point_t cos_inner_r = fp_sub(FP_ONE, (inner_r2 >> 1));
    fixed_point_t ba_r2 = fp_mul(Ba_r, Ba_r);
    fixed_point_t inv_denom_r = fp_div(FP_ONE, fp_add(FP_ONE, ba_r2));
    fixed_point_t C_eff_r = fp_mul(fp_mul(D_pac_r, fp_mul(C_shape, B_r)),
                                   fp_mul(cos_inner_r, inv_denom_r));
    C_eff_r = (C_eff_r > C_min_r) ? C_eff_r : C_min_r;

    /* F_yr at operating point */
    fixed_point_t inner_r3 = fp_mul(fp_mul(inner_r, inner_r), inner_r);
    fixed_point_t sin_inner_r = fp_sub(inner_r, fp_mul(inner_r3, FP_CONST(0.16666667)));
    fixed_point_t F_yr = fp_mul(D_pac_r, sin_inner_r);

    /* Precomputed denominators */
    fixed_point_t inv_m     = fp_div(FP_ONE, m);
    fixed_point_t inv_Iz    = fp_div(FP_ONE, I_z);
    fixed_point_t vx2 = fp_mul(vx_safe, vx_safe);
    fixed_point_t front_num2 = fp_mul(vy_plus_lf_w, vy_plus_lf_w);
    fixed_point_t rear_num2  = fp_mul(vy_minus_lr_w, vy_minus_lr_w);
    fixed_point_t D_f = fp_add(vx2, front_num2);
    fixed_point_t D_r = fp_add(vx2, rear_num2);
    if (D_f == 0) D_f = FP_ONE;
    if (D_r == 0) D_r = FP_ONE;
    fixed_point_t inv_D_f = fp_div(FP_ONE, D_f);
    fixed_point_t inv_D_r = fp_div(FP_ONE, D_r);

    fixed_point_t daf_dvx = fp_mul(vy_plus_lf_w, inv_D_f);
    fixed_point_t daf_dvy = fp_sub(0, fp_mul(vx_safe, inv_D_f));
    fixed_point_t daf_dom = fp_sub(0, fp_mul(fp_mul(l_f, vx_safe), inv_D_f));
    fixed_point_t dar_dvx = fp_mul(vy_minus_lr_w, inv_D_r);
    fixed_point_t dar_dvy = fp_sub(0, fp_mul(vx_safe, inv_D_r));
    fixed_point_t dar_dom = fp_mul(fp_mul(l_r, vx_safe), inv_D_r);

    fixed_point_t dFyf_dvx = fp_mul(C_eff_f, daf_dvx);
    fixed_point_t dFyf_dvy = fp_mul(C_eff_f, daf_dvy);
    fixed_point_t dFyf_dom = fp_mul(C_eff_f, daf_dom);
    fixed_point_t dFyf_dd = C_eff_f;
    fixed_point_t dFyr_dvx = fp_mul(C_eff_r, dar_dvx);
    fixed_point_t dFyr_dvy = fp_mul(C_eff_r, dar_dvy);
    fixed_point_t dFyr_dom = fp_mul(C_eff_r, dar_dom);

    /* Jacobians aligned with MPC/MPC_FPGA full model */
    fixed_point_t dvx_dvx = fp_mul(fp_sub(0, fp_mul(dFyf_dvx, sin_delta)), inv_m);
    fixed_point_t dvx_dvy = fp_add(fp_mul(fp_sub(0, fp_mul(dFyf_dvy, sin_delta)), inv_m),
                                   state->omega);
    fixed_point_t dvx_domega = fp_add(fp_mul(fp_sub(0, fp_mul(dFyf_dom, sin_delta)), inv_m),
                                      state->vy);

    fixed_point_t dvy_dvx = fp_mul(
        fp_sub(fp_add(fp_mul(dFyf_dvx, cos_delta), dFyr_dvx), fp_mul(m, state->omega)),
        inv_m);
    fixed_point_t dvy_dvy = fp_mul(fp_add(fp_mul(dFyf_dvy, cos_delta), dFyr_dvy), inv_m);
    fixed_point_t dvy_domega = fp_mul(
        fp_sub(fp_add(fp_mul(dFyf_dom, cos_delta), dFyr_dom), fp_mul(m, vx_safe)),
        inv_m);

    fixed_point_t domega_dvx = fp_mul(
        fp_sub(fp_mul(l_f, fp_mul(dFyf_dvx, cos_delta)), fp_mul(l_r, dFyr_dvx)),
        inv_Iz);
    fixed_point_t domega_dvy = fp_mul(
        fp_sub(fp_mul(l_f, fp_mul(dFyf_dvy, cos_delta)), fp_mul(l_r, dFyr_dvy)),
        inv_Iz);
    fixed_point_t domega_domega = fp_mul(
        fp_sub(fp_mul(l_f, fp_mul(dFyf_dom, cos_delta)), fp_mul(l_r, dFyr_dom)),
        inv_Iz);



    /* === Build discrete-time A = I + dt * A_c === */
    for (int i = 0; i < MPCC_NX; i++)
        sys->A[i][i] = FP_ONE;

    /* Row 0 (s): ds/dt = v_theta (control-driven, no state derivatives) */

    /* Row 1 (vx): full tire model */
    sys->A[1][1] = fp_add(sys->A[1][1], fp_mul(dt, dvx_dvx));
    sys->A[1][2] = fp_add(sys->A[1][2], fp_mul(dt, dvx_dvy));
    sys->A[1][3] = fp_add(sys->A[1][3], fp_mul(dt, dvx_domega));

    /* Row 2 (vy): full tire model */
    sys->A[2][1] = fp_add(sys->A[2][1], fp_mul(dt, dvy_dvx));
    sys->A[2][2] = fp_add(sys->A[2][2], fp_mul(dt, dvy_dvy));
    sys->A[2][3] = fp_add(sys->A[2][3], fp_mul(dt, dvy_domega));

    /* Row 3 (omega): full tire model */
    sys->A[3][1] = fp_add(sys->A[3][1], fp_mul(dt, domega_dvx));
    sys->A[3][2] = fp_add(sys->A[3][2], fp_mul(dt, domega_dvy));
    sys->A[3][3] = fp_add(sys->A[3][3], fp_mul(dt, domega_domega));

    /* Row 4 (X): dX/dt = vx*cos(psi) - vy*sin(psi) */
    sys->A[4][1] = fp_add(sys->A[4][1], fp_mul(dt, cos_psi));
    sys->A[4][2] = fp_add(sys->A[4][2], fp_mul(dt, fp_sub(0, sin_psi)));
    sys->A[4][6] = fp_add(sys->A[4][6], fp_mul(dt,
        fp_sub(0, fp_add(fp_mul(state->vx, sin_psi),
                         fp_mul(state->vy, cos_psi)))));

    /* Row 5 (Y): dY/dt = vx*sin(psi) + vy*cos(psi) */
    sys->A[5][1] = fp_add(sys->A[5][1], fp_mul(dt, sin_psi));
    sys->A[5][2] = fp_add(sys->A[5][2], fp_mul(dt, cos_psi));
    sys->A[5][6] = fp_add(sys->A[5][6], fp_mul(dt,
        fp_sub(fp_mul(state->vx, cos_psi),
               fp_mul(state->vy, sin_psi))));

    /* Row 6 (psi): dpsi/dt = omega */
    sys->A[6][3] = fp_add(sys->A[6][3], dt);

    /* === Build B = dt * B_c === */
    /* Row 0: ds/dt = v_theta */
    sys->B[0][MPCC_IDX_VTHETA] = dt;

    /* Row 1: dvx/dt = a_x */
    sys->B[1][MPCC_IDX_AX] = dt;

    /* Steering couplings (full model) */
    {
        fixed_point_t dFyf_dd_sin = fp_mul(dFyf_dd, sin_delta);
        fixed_point_t Fyf_cos = fp_mul(F_yf, cos_delta);
        fixed_point_t dFyf_dd_cos = fp_mul(dFyf_dd, cos_delta);
        fixed_point_t Fyf_sin = fp_mul(F_yf, sin_delta);
        sys->B[1][MPCC_IDX_DELTA] = fp_mul(dt,
            fp_mul(fp_sub(fp_sub(0, dFyf_dd_sin), Fyf_cos), inv_m));
        sys->B[2][MPCC_IDX_DELTA] = fp_mul(dt,
            fp_mul(fp_sub(dFyf_dd_cos, Fyf_sin), inv_m));
        sys->B[3][MPCC_IDX_DELTA] = fp_mul(dt,
            fp_mul(fp_mul(l_f, fp_sub(dFyf_dd_cos, Fyf_sin)), inv_Iz));
    }

    /* === Affine term d = f(z_bar, u_bar) - A*z_bar - B*u_bar === */
    fixed_point_t z_next[MPCC_NX];

    /* kinematics (Forward Euler) */

    z_next[0] = fp_add(state->s, fp_mul(dt, control->v_theta));     /* ds/dt = v_theta (virtual progress control) */

    /* Vehicle dynamics with Pacejka tires (Forward Euler) */
    fixed_point_t dvx_dt = fp_add(
        fp_mul(fp_sub(0, fp_mul(F_yf, sin_delta)), inv_m),
        fp_add(control->a_x, fp_mul(state->vy, state->omega)));
    fixed_point_t dvy_dt = fp_mul(
        fp_sub(fp_add(fp_mul(F_yf, cos_delta), F_yr), fp_mul(m, fp_mul(vx_safe, state->omega))),
        inv_m);
    fixed_point_t domega_dt = fp_mul(
        fp_sub(fp_mul(l_f, fp_mul(F_yf, cos_delta)), fp_mul(l_r, F_yr)), inv_Iz);

    z_next[1] = fp_add(state->vx, fp_mul(dt, dvx_dt));
    z_next[2] = fp_add(state->vy, fp_mul(dt, dvy_dt));
    z_next[3] = fp_add(state->omega, fp_mul(dt, domega_dt));

    /* Cartesian kinematics (Forward Euler) */
    z_next[4] = fp_add(state->X, fp_mul(dt,
        fp_sub(fp_mul(state->vx, cos_psi), fp_mul(state->vy, sin_psi))));
    z_next[5] = fp_add(state->Y, fp_mul(dt,
        fp_add(fp_mul(state->vx, sin_psi), fp_mul(state->vy, cos_psi))));
    z_next[6] = fp_add(state->psi, fp_mul(dt, state->omega));

    /* z_bar, u_bar as arrays (7-state global frame) */
    fixed_point_t z_bar[MPCC_NX] = {
        state->s,
        state->vx, state->vy, state->omega,
        state->X, state->Y, state->psi
    };
    fixed_point_t u_bar[MPCC_NU] = { control->delta, control->a_x, control->v_theta };

    /* d = f(z_bar, u_bar) - A*z_bar - B*u_bar */
    for (int i = 0; i < MPCC_NX; i++)
    {
        int64_t Az_i = 0;
        for (int j = 0; j < MPCC_NX; j++)
            Az_i += (int64_t)sys->A[i][j] * z_bar[j];
        fixed_point_t Az = (fixed_point_t)(Az_i >> FP_FRAC_BITS);

        int64_t Bu_i = 0;
        for (int j = 0; j < MPCC_NU; j++)
            Bu_i += (int64_t)sys->B[i][j] * u_bar[j];
        fixed_point_t Bu = (fixed_point_t)(Bu_i >> FP_FRAC_BITS);

        sys->d[i] = fp_sub(z_next[i], fp_add(Az, Bu));
    }
}
