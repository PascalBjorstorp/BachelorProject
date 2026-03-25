/**
 * @file mpcc_vehicle_model.c
 * @brief MPCC Vehicle Dynamics Linearization — Implementation
 *
 * Lifted ODE: 9-state Frenet + linear tire + Cartesian model.
 * Discretized via Forward Euler.
 * All arithmetic Q16.16 fixed-point for FPGA compatibility.
 */

#include "mpcc_vehicle_model.h"
#include <string.h>
#include <stdint.h>

/*===========================================================================
 * Dynamics Linearization (Lifted ODE — Linear Tires)
 *===========================================================================*/

/**
 * Linearize the Lifted ODE dynamics (Frenet + linear tire + Cartesian).
 *
 * State [9]: [s, n, alpha, vx, vy, omega, X, Y, psi]
 * Control [3]: [delta, a_x, v_theta]
 *
 *   Frenet kinematics (rows 0-2):
 *     ds/dt     = v_theta                           (virtual progress control)
 *     dn/dt     = vx*sin(a) + vy*cos(a)
 *     dalpha/dt = omega - kappa * v_theta
 *
 *   Vehicle dynamics with linear tires (rows 3-5):
 *     dvx/dt    = a_x                           (direct control)
 *     dvy/dt    = (F_yf + F_yr) / m - vx*omega
 *     domega/dt = (l_f*F_yf - l_r*F_yr) / I_z
 *
 *     Linear tire forces (with friction and normal load):
 *       F_zf = m*g*l_r/(l_f+l_r),  F_zr = m*g*l_f/(l_f+l_r)
 *       alpha_f = delta - (vy + l_f*omega) / vx
 *       alpha_r = -(vy - l_r*omega) / vx
 *       F_yf = mu * C_Sf * F_zf * alpha_f
 *       F_yr = mu * C_Sr * F_zr * alpha_r
 *
 *   Cartesian kinematics (rows 6-8):
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
    fixed_point_t cos_a   = fp_cos(state->alpha);
    fixed_point_t sin_a   = fp_sin(state->alpha);
    fixed_point_t cos_psi = fp_cos(state->psi);
    fixed_point_t sin_psi = fp_sin(state->psi);

    /* --- Vehicle parameters --- */
    fixed_point_t m   = F110_VEHICLE_MASS_KG;
    fixed_point_t l_f = F110_DIST_CG_TO_FRONT_AXLE_METERS;
    fixed_point_t l_r = F110_DIST_CG_TO_REAR_AXLE_METERS;
    fixed_point_t I_z = F110_YAW_INERTIA_KGM2;
    fixed_point_t C_f = cfg->C_Sf;
    fixed_point_t C_r = cfg->C_Sr;

    /* --- Compute effective cornering stiffness ---
     *   F_zf = m*g*l_r / (l_f+l_r)   F_zr = m*g*l_f / (l_f+l_r)
     *   C_f_eff = mu * C_Sf * F_zf    C_r_eff = mu * C_Sr * F_zr
     */
    fixed_point_t g_acc = F110_GRAVITY_ACCELERATION_MS2;
    fixed_point_t L = fp_add(l_f, l_r);
    fixed_point_t F_zf = fp_div(fp_mul(fp_mul(m, g_acc), l_r), L);
    fixed_point_t F_zr = fp_div(fp_mul(fp_mul(m, g_acc), l_f), L);
    fixed_point_t mu = cfg->mu;
    C_f = fp_mul(mu, fp_mul(C_f, F_zf));   /* effective [N/rad] */
    C_r = fp_mul(mu, fp_mul(C_r, F_zr));   /* effective [N/rad] */

    /* --- Frenet kinematics Jacobians (rows 0-2) --- */
    /* With v_theta control: ds/dt = v_theta (virtual progress),
     * so s-row has no state derivatives — only B[0][VTHETA] = dt.
     *
     * dn/dt = vx*sin(alpha) + vy*cos(alpha) (unchanged)
     * dalpha/dt = omega - kappa * v_theta (only omega dependency) */
    fixed_point_t v_proj = fp_sub(fp_mul(state->vx, cos_a),
                                  fp_mul(state->vy, sin_a));
    fixed_point_t v_perp = fp_add(fp_mul(state->vx, sin_a),
                                  fp_mul(state->vy, cos_a));

    fixed_point_t dn_dalpha = v_proj;
    fixed_point_t dn_dvx    = sin_a;
    fixed_point_t dn_dvy    = cos_a;

    /* --- Tire forces and Jacobians (rows 3-5), aligned with MPC/MPC_FPGA --- */
    fixed_point_t vx_safe = state->vx;
    if (fp_abs(vx_safe) < FP_CONST(0.5)) vx_safe = FP_CONST(0.5);
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

    /* Forces at operating point */
    fixed_point_t F_yf = fp_mul(C_f, alpha_f);
    fixed_point_t F_yr = fp_mul(C_r, alpha_r);

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

    fixed_point_t dFyf_dvx = fp_mul(C_f, daf_dvx);
    fixed_point_t dFyf_dvy = fp_mul(C_f, daf_dvy);
    fixed_point_t dFyf_dom = fp_mul(C_f, daf_dom);
    fixed_point_t dFyf_dd = C_f;
    fixed_point_t dFyr_dvx = fp_mul(C_r, dar_dvx);
    fixed_point_t dFyr_dvy = fp_mul(C_r, dar_dvy);
    fixed_point_t dFyr_dom = fp_mul(C_r, dar_dom);

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

    /* Row 1 (n) */
    sys->A[1][2] = fp_add(sys->A[1][2], fp_mul(dt, dn_dalpha));
    sys->A[1][3] = fp_add(sys->A[1][3], fp_mul(dt, dn_dvx));
    sys->A[1][4] = fp_add(sys->A[1][4], fp_mul(dt, dn_dvy));

    /* Row 2 (alpha): dalpha/dt = omega - kappa*v_theta */
    sys->A[2][5] = fp_add(sys->A[2][5], dt); /* da/domega = 1 */

    /* Row 3 (vx): full model */
    sys->A[3][3] = fp_add(sys->A[3][3], fp_mul(dt, dvx_dvx));
    sys->A[3][4] = fp_add(sys->A[3][4], fp_mul(dt, dvx_dvy));
    sys->A[3][5] = fp_add(sys->A[3][5], fp_mul(dt, dvx_domega));

    /* Row 4 (vy): linear tire */
    sys->A[4][3] = fp_add(sys->A[4][3], fp_mul(dt, dvy_dvx));
    sys->A[4][4] = fp_add(sys->A[4][4], fp_mul(dt, dvy_dvy));
    sys->A[4][5] = fp_add(sys->A[4][5], fp_mul(dt, dvy_domega));

    /* Row 5 (omega): linear tire */
    sys->A[5][3] = fp_add(sys->A[5][3], fp_mul(dt, domega_dvx));
    sys->A[5][4] = fp_add(sys->A[5][4], fp_mul(dt, domega_dvy));
    sys->A[5][5] = fp_add(sys->A[5][5], fp_mul(dt, domega_domega));

    /* Row 6 (X): dX/dt = vx*cos(psi) - vy*sin(psi) */
    sys->A[6][3] = fp_add(sys->A[6][3], fp_mul(dt, cos_psi));
    sys->A[6][4] = fp_add(sys->A[6][4], fp_mul(dt, fp_sub(0, sin_psi)));
    sys->A[6][8] = fp_add(sys->A[6][8], fp_mul(dt,
        fp_sub(0, fp_add(fp_mul(state->vx, sin_psi),
                         fp_mul(state->vy, cos_psi)))));

    /* Row 7 (Y): dY/dt = vx*sin(psi) + vy*cos(psi) */
    sys->A[7][3] = fp_add(sys->A[7][3], fp_mul(dt, sin_psi));
    sys->A[7][4] = fp_add(sys->A[7][4], fp_mul(dt, cos_psi));
    sys->A[7][8] = fp_add(sys->A[7][8], fp_mul(dt,
        fp_sub(fp_mul(state->vx, cos_psi),
               fp_mul(state->vy, sin_psi))));

    /* Row 8 (psi): dpsi/dt = omega */
    sys->A[8][5] = fp_add(sys->A[8][5], dt);

    /* === Build B = dt * B_c === */
    /* Row 0: ds/dt = v_theta */
    sys->B[0][MPCC_IDX_VTHETA] = dt;

    /* Row 2: dalpha/dv_theta = -kappa */
    sys->B[2][MPCC_IDX_VTHETA] = fp_mul(dt, fp_sub(0, kappa));

    /* Row 3: dvx/dt = a_x */
    sys->B[3][MPCC_IDX_AX] = dt;

    /* Steering couplings (full model) */
    {
        fixed_point_t dFyf_dd_sin = fp_mul(dFyf_dd, sin_delta);
        fixed_point_t Fyf_cos = fp_mul(F_yf, cos_delta);
        fixed_point_t dFyf_dd_cos = fp_mul(dFyf_dd, cos_delta);
        fixed_point_t Fyf_sin = fp_mul(F_yf, sin_delta);
        sys->B[3][MPCC_IDX_DELTA] = fp_mul(dt,
            fp_mul(fp_sub(fp_sub(0, dFyf_dd_sin), Fyf_cos), inv_m));
        sys->B[4][MPCC_IDX_DELTA] = fp_mul(dt,
            fp_mul(fp_sub(dFyf_dd_cos, Fyf_sin), inv_m));
        sys->B[5][MPCC_IDX_DELTA] = fp_mul(dt,
            fp_mul(fp_mul(l_f, fp_sub(dFyf_dd_cos, Fyf_sin)), inv_Iz));
    }

    /* === Affine term d = f(z_bar, u_bar) - A*z_bar - B*u_bar === */
    fixed_point_t z_next[MPCC_NX];

    /* Frenet kinematics (Forward Euler) */
    fixed_point_t ndot = v_perp;
    fixed_point_t adot = fp_sub(state->omega, fp_mul(kappa, control->v_theta));

    z_next[0] = fp_add(state->s, fp_mul(dt, control->v_theta));
    z_next[1] = fp_add(state->n, fp_mul(dt, ndot));
    z_next[2] = fp_add(state->alpha, fp_mul(dt, adot));

    /* Vehicle dynamics with linear tires (Forward Euler) */
    fixed_point_t dvx_dt = fp_add(
        fp_mul(fp_sub(0, fp_mul(F_yf, sin_delta)), inv_m),
        fp_add(control->a_x, fp_mul(state->vy, state->omega)));
    fixed_point_t dvy_dt = fp_mul(
        fp_sub(fp_add(fp_mul(F_yf, cos_delta), F_yr), fp_mul(m, fp_mul(vx_safe, state->omega))),
        inv_m);
    fixed_point_t domega_dt = fp_mul(
        fp_sub(fp_mul(l_f, fp_mul(F_yf, cos_delta)), fp_mul(l_r, F_yr)), inv_Iz);

    z_next[3] = fp_add(state->vx, fp_mul(dt, dvx_dt));
    z_next[4] = fp_add(state->vy, fp_mul(dt, dvy_dt));
    z_next[5] = fp_add(state->omega, fp_mul(dt, domega_dt));

    /* Cartesian kinematics (Forward Euler) */
    z_next[6] = fp_add(state->X, fp_mul(dt,
        fp_sub(fp_mul(state->vx, cos_psi), fp_mul(state->vy, sin_psi))));
    z_next[7] = fp_add(state->Y, fp_mul(dt,
        fp_add(fp_mul(state->vx, sin_psi), fp_mul(state->vy, cos_psi))));
    z_next[8] = fp_add(state->psi, fp_mul(dt, state->omega));

    /* z_bar, u_bar as arrays (9-state Lifted ODE) */
    fixed_point_t z_bar[MPCC_NX] = {
        state->s, state->n, state->alpha,
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
