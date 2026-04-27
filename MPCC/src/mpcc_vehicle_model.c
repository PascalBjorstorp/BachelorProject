/**
 * @file mpcc_vehicle_model.c
 * @brief MPCC Vehicle Dynamics Linearization — Implementation
 */

#include "mpcc_vehicle_model.h"
#include <string.h>
#include <stdint.h>
#include <math.h>

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
 *       D = mu * F_z,  B = C_S / C_shape
 *       C_shape = 1.9
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
    float dt,
    const MPCCConfiguration_t *cfg,
    MPCCLinearSystem_t *sys)
{
    memset(sys, 0, sizeof(*sys));

    /* --- Trigonometric values --- */
    float cos_psi = cosf(state->psi);
    float sin_psi = sinf(state->psi);

    /* --- Vehicle parameters --- */
    float m   = F110_VEHICLE_MASS_KG;
    float l_f = F110_DIST_CG_TO_FRONT_AXLE_METERS;
    float l_r = F110_DIST_CG_TO_REAR_AXLE_METERS;
    float I_z = F110_YAW_INERTIA_KGM2;
    float C_f = cfg->C_Sf;
    float C_r = cfg->C_Sr;

    /* --- Normal loads and Pacejka parameters ---
     *   F_zf = m*g*l_r / (l_f+l_r)   F_zr = m*g*l_f / (l_f+l_r)
     *   D_pac = mu * F_z   (peak lateral force)
     *   B = C_S / C_shape  (Pacejka stiffness factor)
     */
    float g_acc = F110_GRAVITY_ACCELERATION_MS2;
    float L = l_f + l_r;
    float F_zf = (m * g_acc * l_r) / L;
    float F_zr = (m * g_acc * l_f) / L;
    float mu = cfg->mu;

    /* Pacejka shape factor and B factors.
     * Standard Pacejka: F_y = D * sin(C * atan(B * alpha))
     * At small alpha: F_y ≈ D * C * B * alpha
     * Gym linear model: F_y = mu * C_S * F_z * alpha
     * Matching: D * C * B = mu * C_S * F_z
     *   (mu * F_z) * C_shape * B = mu * C_S * F_z
     *   B = C_S / C_shape
     */
    float C_shape = 1.9f;
    float D_pac_f = mu * F_zf;        /* peak front force */
    float D_pac_r = mu * F_zr;        /* peak rear force */
    float B_f = C_f / C_shape;        /* was C_f / (D_pac_f * C_shape) — 12x too small */
    float B_r = C_r / C_shape;        /* was C_r / (D_pac_r * C_shape) — 12x too small */

    /* Minimum stiffness clamp (10% of linear C_S) */
    float C_min_f = C_f * 0.1f;
    float C_min_r = C_r * 0.1f;


    /*=====================================================================================
     * Tire forces and Jacobians (Pacejka model with atan slip angles)
     *=====================================================================================*/
    /* --- Tire forces and Jacobians (rows 1-3) ---
     * Minimum vx for slip angle calculations. With the corrected Pacejka
     * B factors (B = C_S/C_shape), effective tire stiffness is ~12x larger
     * than old (wrong) formula. The continuous-time [vy, omega] eigenvalues
    * scale as ~C_eff / (vx * I_z). At vx=0.5 they reach -55 and -145,
    * causing Forward Euler (A_d = I + dt*A_c) discrete eigenvalues of
    * -1.76 and -6.23 — far outside the unit circle (unstable).
    * vx_safe=2.0 gives stable discrete eigenvalues [0.40, -0.90]. */
    float vx_abs = fabsf(state->vx);
    float vx_safe = (vx_abs < 0.5f) ? 0.5f : vx_abs;
    float inv_vx = 1.0f / vx_safe;
    float cos_delta = cosf(control->delta);
    float sin_delta = sinf(control->delta);

    float vy_plus_lf_w  = state->vy + l_f * state->omega;
    float vy_minus_lr_w = state->vy - l_r * state->omega;

    /* Slip angles at operating point */
    float front_ratio = vy_plus_lf_w * inv_vx;
    float rear_ratio  = vy_minus_lr_w * inv_vx;
    float alpha_f = control->delta - atanf(front_ratio);
    float alpha_r = -atanf(rear_ratio);

    /* ================================================================
     * Pacejka tire model: F_y = D * sin(C * atan(B * alpha))
     *
     *   C_eff = dF_y/dalpha at operating-point slip angle:
     *   C_eff = D * C * B * cos(C*atan(B*a)) / (1 + (B*a)^2)
     *
     *   F_yf, F_yr at operating point (for B matrix and affine term).
     * ================================================================ */

    /* Front tire — Pacejka effective stiffness */
    float Ba_f = B_f * alpha_f;
    float inner_f = C_shape * atanf(Ba_f);
    float cos_inner_f = cosf(inner_f);
    float ba_f2 = Ba_f * Ba_f;
    float inv_denom_f = 1.0f / (1.0f + ba_f2);
    float C_eff_f = (D_pac_f * C_shape * B_f) * (cos_inner_f * inv_denom_f);
    C_eff_f = (C_eff_f > C_min_f) ? C_eff_f : C_min_f;

    /* F_yf at operating point */
    float sin_inner_f = sinf(inner_f);
    float F_yf = D_pac_f * sin_inner_f;

    /* Rear tire — Pacejka effective stiffness */
    float Ba_r = B_r * alpha_r;
    float inner_r = C_shape * atanf(Ba_r);
    float cos_inner_r = cosf(inner_r);
    float ba_r2 = Ba_r * Ba_r;
    float inv_denom_r = 1.0f / (1.0f + ba_r2);
    float C_eff_r = (D_pac_r * C_shape * B_r) * (cos_inner_r * inv_denom_r);
    C_eff_r = (C_eff_r > C_min_r) ? C_eff_r : C_min_r;

    /* F_yr at operating point */
    float sin_inner_r = sinf(inner_r);
    float F_yr = D_pac_r * sin_inner_r;
    
    /* Precomputed denominators */
    float inv_m     = 1.0f / m;
    float inv_Iz    = 1.0f / I_z;
    float vx2 = vx_safe * vx_safe;
    float front_num2 = vy_plus_lf_w * vy_plus_lf_w;
    float rear_num2  = vy_minus_lr_w * vy_minus_lr_w;
    float D_f = vx2 + front_num2;
    float D_r = vx2 + rear_num2;
    if (D_f == 0) D_f = 1.0f;
    if (D_r == 0) D_r = 1.0f;
    float inv_D_f = 1.0f / D_f;
    float inv_D_r = 1.0f / D_r;

    float daf_dvx = (vx_abs >= 0.5f) ? (vy_plus_lf_w  * inv_D_f) : 0.0f;
    float daf_dvy = -vx_safe * inv_D_f;
    float daf_dom = -(l_f * vx_safe) * inv_D_f;
    float dar_dvx = (vx_abs >= 0.5f) ? (vy_minus_lr_w * inv_D_r) : 0.0f;
    float dar_dvy = -vx_safe * inv_D_r;
    float dar_dom = (l_r * vx_safe) * inv_D_r;

    float dFyf_dvx = C_eff_f * daf_dvx;
    float dFyf_dvy = C_eff_f * daf_dvy;
    float dFyf_dom = C_eff_f * daf_dom;
    float dFyf_dd = C_eff_f;
    float dFyr_dvx = C_eff_r * dar_dvx;
    float dFyr_dvy = C_eff_r * dar_dvy;
    float dFyr_dom = C_eff_r * dar_dom;

    /* Jacobians aligned with MPC/MPC_FPGA full model */
    float dvx_dvx = (0 - dFyf_dvx * sin_delta) * inv_m;
    float dvx_dvy = ((0 - dFyf_dvy * sin_delta) * inv_m) + state->omega;
    float dvx_domega = ((0 - dFyf_dom * sin_delta) * inv_m) + state->vy;

    float dvy_dvx = ((dFyf_dvx * cos_delta + dFyr_dvx - m * state->omega) * inv_m);
    float dvy_dvy = ((dFyf_dvy * cos_delta + dFyr_dvy) * inv_m);
    float dvy_domega = ((dFyf_dom * cos_delta + dFyr_dom - m * vx_safe) * inv_m);

    float domega_dvx = ((l_f * (dFyf_dvx * cos_delta) - l_r * dFyr_dvx) * inv_Iz);
    float domega_dvy = ((l_f * (dFyf_dvy * cos_delta) - l_r * dFyr_dvy) * inv_Iz);
    float domega_domega = ((l_f * (dFyf_dom * cos_delta) - l_r * dFyr_dom) * inv_Iz);



    /* === Build discrete-time A = I + dt * A_c === */
    for (int i = 0; i < MPCC_NX; i++)
        sys->A[i][i] = 1.0f;

    /* Row 0 (s): ds/dt = v_theta (control-driven, no state derivatives) */

    /* Row 1 (vx): full tire model */
    sys->A[1][1] = sys->A[1][1] + (dt * dvx_dvx);
    sys->A[1][2] = sys->A[1][2] + (dt * dvx_dvy);
    sys->A[1][3] = sys->A[1][3] + (dt * dvx_domega);

    /* Row 2 (vy): full tire model */
    sys->A[2][1] = sys->A[2][1] + (dt * dvy_dvx);
    sys->A[2][2] = sys->A[2][2] + (dt * dvy_dvy);
    sys->A[2][3] = sys->A[2][3] + (dt * dvy_domega);

    /* Row 3 (omega): full tire model */
    sys->A[3][1] = sys->A[3][1] + (dt * domega_dvx);
    sys->A[3][2] = sys->A[3][2] + (dt * domega_dvy);
    sys->A[3][3] = sys->A[3][3] + (dt * domega_domega);

    /* Row 4 (X): dX/dt = vx*cos(psi) - vy*sin(psi) */
    sys->A[4][1] = sys->A[4][1] + (dt * cos_psi);
    sys->A[4][2] = sys->A[4][2] + (dt * -sin_psi);
    sys->A[4][6] = sys->A[4][6] + (dt * -(state->vx * sin_psi + state->vy * cos_psi));

    /* Row 5 (Y): dY/dt = vx*sin(psi) + vy*cos(psi) */
    sys->A[5][1] = sys->A[5][1] + (dt * sin_psi);
    sys->A[5][2] = sys->A[5][2] + (dt * cos_psi);
    sys->A[5][6] = sys->A[5][6] + (dt * (state->vx * cos_psi - state->vy * sin_psi));

    /* Row 6 (psi): dpsi/dt = omega */
    sys->A[6][3] = sys->A[6][3] + dt;

    /* === Build B = dt * B_c === */
    /* Row 0: ds/dt = v_theta */
    sys->B[0][MPCC_IDX_VTHETA] = dt;

    /* Row 1: dvx/dt = a_x */
    sys->B[1][MPCC_IDX_AX] = dt;

    /* Rows 2-3: load transfer effect of a_x on lateral and yaw dynamics.
     * Under longitudinal acceleration, normal loads shift front-to-rear:
     *   dF_zf/d(a_x) = -m * h_cg / L  (front unloads)
     *   dF_zr/d(a_x) = +m * h_cg / L  (rear loads)
     * This changes peak lateral force D_pac = mu * F_z, which changes
     * F_yf and F_yr: dF_y/d(a_x) = (F_y / D_pac) * mu * dF_z/d(a_x) */
    {
        float h_cg    = F110_CG_HEIGHT_METERS;
        float dFzf_dax = -(m * h_cg) / L;
        float dFzr_dax = -dFzf_dax;
        float dFyf_dax = (F_yf / D_pac_f) * (mu * dFzf_dax);
        float dFyr_dax = (F_yr / D_pac_r) * (mu * dFzr_dax);
        sys->B[2][MPCC_IDX_AX] = dt * ((dFyf_dax * cos_delta + dFyr_dax) * inv_m);
        sys->B[3][MPCC_IDX_AX] = dt * ((l_f * dFyf_dax * cos_delta
                                       - l_r * dFyr_dax) * inv_Iz);
    }

    /* Steering couplings (full model) */
    {
        float dFyf_dd_sin = dFyf_dd * sin_delta;
        float Fyf_cos = F_yf * cos_delta;
        float dFyf_dd_cos = dFyf_dd * cos_delta;
        float Fyf_sin = F_yf * sin_delta;
        sys->B[1][MPCC_IDX_DELTA] = dt * (((-dFyf_dd_sin) - Fyf_cos) * inv_m);
        sys->B[2][MPCC_IDX_DELTA] = dt * ((dFyf_dd_cos - Fyf_sin) * inv_m);
        sys->B[3][MPCC_IDX_DELTA] = dt * ((l_f * (dFyf_dd_cos - Fyf_sin)) * inv_Iz);
    }

    /* === Affine term d = f(z_bar, u_bar) - A*z_bar - B*u_bar === */
    float z_next[MPCC_NX];

    /* kinematics (Forward Euler) */

    z_next[0] = state->s + (dt * control->v_theta);     /* ds/dt = v_theta (virtual progress control) */

    /* Vehicle dynamics with Pacejka tires (Forward Euler)
     * Use actual state->vx in forward model for accurate affine term,
     * even though Jacobians use vx_safe for numerical stability. */
    float vx_actual = (vx_abs > 0.01f) ? state->vx : 0.01f;
    float dvx_dt = (((-F_yf * sin_delta) * inv_m) + (control->a_x + (state->vy * state->omega)));
    float dvy_dt = (((F_yf * cos_delta) + F_yr) - (m * (vx_actual * state->omega))) * inv_m;
    float domega_dt = (((l_f * (F_yf * cos_delta)) - (l_r * F_yr)) * inv_Iz);

    z_next[1] = state->vx + (dt * dvx_dt);
    z_next[2] = state->vy + (dt * dvy_dt);
    z_next[3] = state->omega + (dt * domega_dt);

    /* Cartesian kinematics (Forward Euler) */
    z_next[4] = state->X + (dt * ((state->vx * cos_psi) - (state->vy * sin_psi)));
    z_next[5] = state->Y + (dt * ((state->vx * sin_psi) + (state->vy * cos_psi)));
    z_next[6] = state->psi + (dt * state->omega);

    /* z_bar, u_bar as arrays (7-state global frame) */
    float z_bar[MPCC_NX] = {
        state->s,
        state->vx, state->vy, state->omega,
        state->X, state->Y, state->psi
    };
    float u_bar[MPCC_NU] = { control->delta, control->a_x, control->v_theta };

    /* d = f(z_bar, u_bar) - A*z_bar - B*u_bar */
    for (int i = 0; i < MPCC_NX; i++)
    {
        float Az_i = 0.0f;
        for (int j = 0; j < MPCC_NX; j++)
            Az_i += (float)sys->A[i][j] * z_bar[j];
        float Az = Az_i;

        float Bu_i = 0.0f;
        for (int j = 0; j < MPCC_NU; j++)
            Bu_i += (float)sys->B[i][j] * u_bar[j];
        float Bu = Bu_i;

        sys->d[i] = z_next[i] - Az - Bu;
    }
}
