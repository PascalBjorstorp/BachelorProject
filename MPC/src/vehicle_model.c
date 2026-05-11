/**
 * @file vehicle_model.c
 * @brief Dynamic nonlinear bicycle model implementation.
 * @dependencies vehicle_model.h, <stdio.h>, <string.h>, <stdlib.h>
 */

#include "vehicle_model.h"

/*===========================================================================
 * Shared Helper Math
 *===========================================================================*/

void vehicle_model_compute_slip_terms(
    float vx,
    float vy,
    float omega,
    float delta,
    SlipTerms_t *slip_terms)
{
    /*
     * Bicycle slip-angle definitions:
     *   alpha_f = delta - atan((v_y + l_f*omega)/v_x)
     *   alpha_r =       -atan((v_y - l_r*omega)/v_x)
     * A minimum longitudinal velocity floor keeps the ratio well-conditioned.
     */
    slip_terms->vx_safe = (vx > MIN_SLIP_VELOCITY) ? vx : MIN_SLIP_VELOCITY;
    slip_terms->inv_vx_safe = util_recip(slip_terms->vx_safe);

    slip_terms->front_num = vy + VP_CG_TO_FRONT_AXLE_M * omega;
    slip_terms->rear_num = vy - VP_CG_TO_REAR_AXLE_M * omega;

    slip_terms->front_ratio = slip_terms->front_num * slip_terms->inv_vx_safe;
    slip_terms->rear_ratio = slip_terms->rear_num * slip_terms->inv_vx_safe;

    slip_terms->alpha_front = delta - atanf(slip_terms->front_ratio);
    slip_terms->alpha_rear = -atanf(slip_terms->rear_ratio);
}

void vehicle_model_compute_normal_loads(
    float longitudinal_force,
    float *front_normal_load,
    float *rear_normal_load)
{
    /*
     *  Quasi-static longitudinal load transfer:
     *  acceleration/braking shifts load by (F_x * h / L) between front/rear axles.
     */
    *front_normal_load = util_div(
        VP_MASS_TIMES_GRAVITY_N * VP_CG_TO_REAR_AXLE_M - longitudinal_force * VP_CG_HEIGHT_M,
        VP_WHEELBASE_M);

    *rear_normal_load = util_div(
        VP_MASS_TIMES_GRAVITY_N * VP_CG_TO_FRONT_AXLE_M + longitudinal_force * VP_CG_HEIGHT_M,
        VP_WHEELBASE_M);
}

void vehicle_model_compute_effective_lateral_stiffness(
    uint8_t use_front_axle,
    float normal_load,
    float slip_angle,
    float *effective_stiffness,
    float *lateral_force)
{
    float cornering_stiffness =
        use_front_axle ? VP_FRONT_CORNERING_STIFFNESS : VP_REAR_CORNERING_STIFFNESS;
    float linear_stiffness = VP_FRICTION_COEFF * cornering_stiffness * normal_load;
    float peak_force = VP_FRICTION_COEFF * normal_load;
    float B_term = cornering_stiffness * VP_INV_C_SHAPE;
    float B_alpha = B_term * slip_angle;
    float inner_angle = VP_C_SHAPE * atanf(B_alpha);
    float cos_inner = cosf(inner_angle);
    float inv_denom = util_recip(1.0f + B_alpha * B_alpha);

    /*
     * Local linearization slope for a Pacejka-like tire law, with a floor to
     * prevent near-saturated operating points from collapsing Jacobian gains.
     */
    float effective_slope = peak_force * VP_C_SHAPE * B_term * cos_inner * inv_denom;
    float minimum_slope = linear_stiffness * MIN_STIFF_SCALE;
    *effective_stiffness =
        (effective_slope > minimum_slope) ? effective_slope : minimum_slope;

    if (lateral_force != NULL)
    {
        *lateral_force = peak_force * sinf(inner_angle);
    }
}

/*===========================================================================
 * Control Saturation
 *===========================================================================*/

ControlInput_t vehicle_model_saturate_control(
    const ControlInput_t *raw_control)
{
    ControlInput_t saturated_control;

    /* Clamp steering angle to physical limits */
    saturated_control.steer_ang = util_clamp(
        raw_control->steer_ang,
        -VP_MAX_STEERING_RAD,
        VP_MAX_STEERING_RAD);

    /* Clamp acceleration to [min, max] */
    saturated_control.long_acc = util_clamp(
        raw_control->long_acc,
        VP_MIN_ACCEL_MPS2,
        VP_MAX_ACCEL_MPS2);

    return saturated_control;
}

/*===========================================================================
 * Single-Step State Prediction
 *===========================================================================*/

VehicleState_t vehicle_model_predict_next_state(
    const VehicleState_t *current_state,
    const ControlInput_t *control_input,
    float time_step)
{
    VehicleState_t next_state;

    // Saturate control inputs to ensure predictions respect physical limits.
    ControlInput_t saturated_control = vehicle_model_saturate_control(control_input);

    // Extract current state variables for readability
    float psi = current_state->heading;
    float vx  = current_state->long_vel;
    float vy  = current_state->lat_vel;
    float omega = current_state->yaw_rate;

    // Extract control inputs
    float delta   = saturated_control.steer_ang;
    float a_cmd  = saturated_control.long_acc;

    // Precompute steering trigonometric terms for force resolution
    float cos_delta = cosf(delta);
    float sin_delta = sinf(delta);

    /*
     * Compute longitudinal force from acceleration command
     *
     * F_x = m * a_cmd
     *
     * The MPC outputs an acceleration command directly.
     * The longitudinal force is simply mass × acceleration.
     */
    float Fx = VP_MASS_KG * a_cmd;

    /*
     * Compute tire slip angles
     *
     * Minimum velocity floor to prevent division by zero at standstill.
     * Below this speed, the dynamic model degenerates — slip angles are
     * undefined when v_x ≈ 0.
     */
    SlipTerms_t slip_terms;
    vehicle_model_compute_slip_terms(vx, vy, omega, delta, &slip_terms);

    /*
     * Compute normal forces (load transfer under acceleration)
     *
     * Static weight distribution plus longitudinal load transfer:
     *   F_zf = (m * g * l_r - F_x * h) / (l_f + l_r)
     *   F_zr = (m * g * l_f + F_x * h) / (l_f + l_r)
     */
    float F_zf;
    float F_zr;
    vehicle_model_compute_normal_loads(Fx, &F_zf, &F_zr);

    /*
     * Compute lateral tire forces (linear tire model with normal force)
     *
     * The cornering stiffness is scaled by friction coefficient and
     * normal force to capture load transfer and surface grip effects:
     *
     *   F_yf = mu * C_Sf * alpha_f * F_zf
     *   F_yr = mu * C_Sr * alpha_r * F_zr
     *
     * mu        — tire-road friction coefficient (dimensionless)
     * C_Sf/C_Sr — pure tire cornering stiffness [1/rad]
     * F_zf/F_zr — normal forces [N]
     */
    float F_yf = VP_FRICTION_COEFF * VP_FRONT_CORNERING_STIFFNESS * slip_terms.alpha_front * F_zf;
    float F_yr = VP_FRICTION_COEFF * VP_REAR_CORNERING_STIFFNESS * slip_terms.alpha_rear * F_zr;

    /* Full model: cos(δ)/sin(δ) force resolution for real-world accuracy */
    /* dv_x/dt = (F_x - F_yf * sin(delta) + m * v_y * omega) / m */
    float dvx_dt = (Fx - F_yf * sin_delta) * VP_INV_MASS_1_PER_KG + vy * omega;

    /* dv_y/dt = (F_yf * cos(delta) + F_yr - m * v_x * omega) / m */
    float dvy_dt = (F_yf * cos_delta + F_yr) * VP_INV_MASS_1_PER_KG - vx * omega;

    /* domega/dt = (l_f * F_yf * cos(delta) - l_r * F_yr) / I_z */
    float domega_dt =
        (VP_CG_TO_FRONT_AXLE_M * F_yf * cos_delta - VP_CG_TO_REAR_AXLE_M * F_yr) *
        VP_INV_YAW_INERTIA_1_PER_KGM2;

    /*
     * Analytical pose integration on SE(2) with constant body twist over dt.
     * Falls back to straight-line integration when yaw rate is near zero.
     */
    {
        float psi_end = psi + time_step * omega;

        if (fabsf(omega) < 1e-6f)
        {
            float cos_psi = cosf(psi);
            float sin_psi = sinf(psi);
            next_state.pos_x = current_state->pos_x + time_step * (vx * cos_psi - vy * sin_psi);
            next_state.pos_y = current_state->pos_y + time_step * (vx * sin_psi + vy * cos_psi);
        }
        else
        {
            float inv_omega = util_recip(omega);
            float sin_psi_start = sinf(psi);
            float cos_psi_start = cosf(psi);
            float sin_psi_end = sinf(psi_end);
            float cos_psi_end = cosf(psi_end);

            next_state.pos_x = current_state->pos_x +
                (vx * (sin_psi_end - sin_psi_start) + vy * (cos_psi_end - cos_psi_start)) * inv_omega;
            next_state.pos_y = current_state->pos_y +
                (vx * (cos_psi_start - cos_psi_end) + vy * (sin_psi_end - sin_psi_start)) * inv_omega;
        }

        next_state.heading = psi_end;
    }

    // Forward Euler integration for body-dynamic states
    next_state.long_vel = vx + time_step * dvx_dt;
    next_state.lat_vel = vy + time_step * dvy_dt;
    next_state.yaw_rate = omega + time_step * domega_dt;

    // Apply state constraints and normalization

    /* Clamp longitudinal velocity to [min, max] */
    next_state.long_vel = util_clamp(
        next_state.long_vel,
        VP_MIN_VELOCITY_MPS,
        VP_MAX_VELOCITY_MPS);

    /* Normalize heading to principal angle domain. */
    next_state.heading = util_normalize_angle(next_state.heading);

    return next_state;
}

/*===========================================================================
 * Multi-Step Trajectory Prediction
 *===========================================================================*/

void vehicle_model_predict_trajectory(
    const VehicleState_t *initial_state,
    const ControlInput_t *control_sequence,
    float time_step,
    uint16_t step_count,
    VehicleState_t *predicted_trajectory)
{
    /* First element of trajectory is the initial state */
    predicted_trajectory[0] = *initial_state;

    /* Predict each subsequent state */
    for (uint16_t step_index = 0; step_index < step_count; step_index++)
    {
        predicted_trajectory[step_index + 1] = vehicle_model_predict_next_state(
            &predicted_trajectory[step_index],
            &control_sequence[step_index],
            time_step);
    }
}

/*===========================================================================
 * Frenet Frame Linearization (Direct Computation)
 *===========================================================================
 *
 * Computes the 5×5 discrete state-space matrices for the Frenet frame
 * directly, without building the full 6×6 global matrices first.
 *
 * Frenet state: [e_y, e_psi, v_x, v_y, omega]
 *
 * Rows 0-1: Frenet kinematic relations (path-relative)
 *   e_y_dot   ≈ v_x * e_psi + v_y           (linearized at e_psi=0)
 *   e_psi_dot ≈ omega - kappa * v_x          (linearized at e_y=0, e_psi=0)
 *
 * Rows 2-4: Body-frame dynamics (identical to global model rows 3-5)
 *   v_x_dot, v_y_dot, omega_dot
 *   Computed inline — avoids the 6×6 global A/B allocation and the
 *   wasted work on global rows 0-2 (X, Y, heading kinematics).
 */
void vehicle_model_compute_frenet_linearization(
    const FrenetState_t *frenet_state,
    const ControlInput_t *operating_control,
    float time_step,
    float path_curvature,
    float reference_velocity,
    float state_matrix_A[NX_FRENET][NX_FRENET],
    float input_matrix_B[NX_FRENET][2])
{

    // Extract Frenet operating point variables
    float vx    = frenet_state->flong_vel;
    float vy    = frenet_state->flat_vel;
    float omega = frenet_state->fyaw_rate;
    float delta = operating_control->steer_ang;

    /* Steering trig values at operating point (accuracy-first path). */
    float cos_delta = cosf(delta);
    float sin_delta = sinf(delta);

    /* Shared slip-angle kinematics used by prediction and linearization. */
    SlipTerms_t slip_terms;
    vehicle_model_compute_slip_terms(vx, vy, omega, delta, &slip_terms);

    /*
     * Longitudinal force (for load transfer only).
     * Fx = m * a_cmd — independent of state, so dFx/dvx = 0.
     */
    float Fx = VP_MASS_KG * operating_control->long_acc;
    const float dFx_dvx = 0.0f;

    // Compute slip angle intermediates for Jacobian
    float front_num = slip_terms.front_num;
    float rear_num = slip_terms.rear_num;

    /*
     * Analytic slip-angle derivatives from atan(n/d):
     *   d(atan(n/d))/dx = (d*dn/dx - n*dd/dx) / (d^2 + n^2)
     * Using closed-form derivatives keeps the Jacobian stable and low-noise.
     */
    float front_num2 = front_num * front_num;
    float rear_num2 = rear_num * rear_num;
    float vx2 = slip_terms.vx_safe * slip_terms.vx_safe;

    float D_f = vx2 + front_num2;
    float D_r = vx2 + rear_num2;
    if (D_f == 0) D_f = 1.0f;
    if (D_r == 0) D_r = 1.0f;

    float inv_D_f = util_recip(D_f);
    float inv_D_r = util_recip(D_r);

    float daf_dvx = front_num * inv_D_f;
    float daf_dvy = -slip_terms.vx_safe * inv_D_f;
    float daf_domega = -(VP_CG_TO_FRONT_AXLE_M * slip_terms.vx_safe) * inv_D_f;

    float dar_dvx = rear_num * inv_D_r;
    float dar_dvy = -slip_terms.vx_safe * inv_D_r;
    float dar_domega = VP_CG_TO_REAR_AXLE_M * slip_terms.vx_safe * inv_D_r;

    /* Below the slip-velocity floor, alpha depends on vx_safe (constant),
     * so d(alpha)/d(vx) must be zero. */
    if (vx <= MIN_SLIP_VELOCITY) {
        daf_dvx = 0.0f;
        dar_dvx = 0.0f;
    }

    /*
     * Normal forces with load transfer
     *   F_zf = (m*g*l_r - F_x*h) / L
     *   F_zr = (m*g*l_f + F_x*h) / L
     */
    float F_zf;
    float F_zr;
    vehicle_model_compute_normal_loads(Fx, &F_zf, &F_zr);

    /*
     * Pacejka-like local linearization at the operating point:
     * Keep nonlinear saturation behavior, then convert to effective
     * cornering stiffness dF_y/dalpha for Jacobian entries.
     */
    float C_Sf_Fzf;
    float C_Sr_Fzr;
    float F_yf;
    vehicle_model_compute_effective_lateral_stiffness(
        1u,
        F_zf,
        slip_terms.alpha_front,
        &C_Sf_Fzf,
        &F_yf);
    vehicle_model_compute_effective_lateral_stiffness(
        0u,
        F_zr,
        slip_terms.alpha_rear,
        &C_Sr_Fzr,
        NULL);

    /*
     * Tire force Jacobians w.r.t. body states
     */
    float dFyf_dvx = C_Sf_Fzf * daf_dvx;
    float dFyf_dvy = C_Sf_Fzf * daf_dvy;
    float dFyf_domega = C_Sf_Fzf * daf_domega;
    float dFyf_ddelta = C_Sf_Fzf;

    float dFyr_dvx = C_Sr_Fzr * dar_dvx;
    float dFyr_dvy = C_Sr_Fzr * dar_dvy;
    float dFyr_domega = C_Sr_Fzr * dar_domega;

    /* Initialize Frenet matrices to zero. */
    memset(state_matrix_A, 0, sizeof(float) * NX_FRENET * NX_FRENET);
    memset(input_matrix_B, 0, sizeof(float) * NX_FRENET * 2);

    /*
     * Row 0: e_y dynamics (Frenet kinematics)
     *   Continuous: e_y_dot = v_x * sin(e_psi) + v_y * cos(e_psi)
     *   Linearized (e_psi ≈ 0): e_y_dot ≈ v_ref * e_psi + v_y
     *     (using v_ref instead of vx to maintain lookahead visibility when vx=0)
     *   Discrete: e_y[k+1] = e_y[k] + dt * (v_ref * e_psi[k] + v_y[k])
     *
     *   A[0][0] = 1                (identity)
     *   A[0][1] = dt * v_ref       (heading error drives lateral drift, using v_ref)
     *   A[0][2] = 0                (∂/∂v_x = sin(e_psi) ≈ 0 at e_psi=0)
     *   A[0][3] = dt               (lateral velocity directly changes e_y)
     *   A[0][4] = 0                (no direct omega coupling)
     */
    {
        float epsi = frenet_state->fhead_error;
        float ey = frenet_state->flat_error;
        float cp = cosf(epsi);
        float sp = sinf(epsi);
        float denom = 1.0f - path_curvature * ey;
        if (fabsf(denom) < 1e-3f)
            denom = (denom >= 0.0f) ? 1e-3f : -1e-3f;
        float inv_denom = util_recip(denom);
        float inv_denom2 = inv_denom * inv_denom;

        state_matrix_A[0][0] = 1.0f;
        state_matrix_A[0][1] = time_step * (reference_velocity * cp - vy * sp);
        state_matrix_A[0][2] = time_step * sp;
        state_matrix_A[0][3] = time_step * cp;

        /*
         * Row 1: e_psi dynamics (Frenet kinematics)
         *   Continuous: e_psi_dot = omega - kappa * v_ref * cos(e_psi) / (1 - kappa * e_y)
         *   Linearized (e_y ≈ 0, e_psi ≈ 0): e_psi_dot ≈ omega - kappa * v_ref
         *     (using v_ref instead of vx to maintain lookahead visibility when vx=0)
         *   Discrete: e_psi[k+1] = e_psi[k] + dt * (omega[k] - kappa * v_ref)
         *
         *   A[1][1] = 1                (identity)
         *   A[1][2] = -dt * kappa * v_ref / v_ref_clamp  (speed effect, using v_ref)
         *   A[1][4] = dt               (yaw rate directly changes heading error)
         */

        state_matrix_A[1][0] = -time_step * (path_curvature * path_curvature) * reference_velocity * cp * inv_denom2;
        state_matrix_A[1][1] = 1.0f + time_step * path_curvature * reference_velocity * sp * inv_denom;
        state_matrix_A[1][2] = -time_step * path_curvature * cp * inv_denom;
        state_matrix_A[1][4] = time_step;
    }

    /* Row 1 already assigned in the exact Frenet Jacobian block above. */

    /*
     * Rows 2-4: Body-frame dynamics (computed directly — avoids the
     * 6×6 global matrix allocation and discarded rows 0-2).
     *
     * These are the Jacobians of [v_x_dot, v_y_dot, omega_dot] w.r.t.
     * body states [v_x, v_y, omega], discretized via Forward Euler.
     */

    /* Full model with cos(δ)/sin(δ) force resolution */

    /* Row 2: dvx/dt = (Fx - Fyf*sin(δ) + m*vy*ω) / m */
    state_matrix_A[2][2] = 1.0f + time_step * ((dFx_dvx - dFyf_dvx * sin_delta) * VP_INV_MASS_1_PER_KG);
    state_matrix_A[2][3] = time_step * (-dFyf_dvy * sin_delta * VP_INV_MASS_1_PER_KG + omega);
    state_matrix_A[2][4] = time_step * (-dFyf_domega * sin_delta * VP_INV_MASS_1_PER_KG + vy);

    /* Row 3: dvy/dt = (Fyf*cos(δ) + Fyr - m*vx*ω) / m */
    state_matrix_A[3][2] = time_step * ((dFyf_dvx * cos_delta + dFyr_dvx - VP_MASS_KG * omega) * VP_INV_MASS_1_PER_KG);
    state_matrix_A[3][3] = 1.0f + time_step * ((dFyf_dvy * cos_delta + dFyr_dvy) * VP_INV_MASS_1_PER_KG);
    state_matrix_A[3][4] = time_step * ((dFyf_domega * cos_delta + dFyr_domega - VP_MASS_KG * vx) * VP_INV_MASS_1_PER_KG);

    /* Row 4: dω/dt = (lf*Fyf*cos(δ) - lr*Fyr) / Iz */
    state_matrix_A[4][2] = time_step * ((VP_CG_TO_FRONT_AXLE_M * dFyf_dvx * cos_delta - VP_CG_TO_REAR_AXLE_M * dFyr_dvx) * VP_INV_YAW_INERTIA_1_PER_KGM2);
    state_matrix_A[4][3] = time_step * ((VP_CG_TO_FRONT_AXLE_M * dFyf_dvy * cos_delta - VP_CG_TO_REAR_AXLE_M * dFyr_dvy) * VP_INV_YAW_INERTIA_1_PER_KGM2);
    state_matrix_A[4][4] = 1.0f + time_step * ((VP_CG_TO_FRONT_AXLE_M * dFyf_domega * cos_delta - VP_CG_TO_REAR_AXLE_M * dFyr_domega) * VP_INV_YAW_INERTIA_1_PER_KGM2);

    /* B matrix — steering column with cos(δ)/sin(δ) force resolution */
    float dFyf_dd_sin = dFyf_ddelta * sin_delta;
    float Fyf_cos = F_yf * cos_delta;
    float dFyf_dd_cos = dFyf_ddelta * cos_delta;
    float Fyf_sin = F_yf * sin_delta;

    /* B[2][0]: d(dvx/dt)/dδ = (-dFyf_dd*sin(δ) - Fyf*cos(δ)) / m */
    input_matrix_B[2][0] = time_step * ((-dFyf_dd_sin - Fyf_cos) * VP_INV_MASS_1_PER_KG);

    /* B[3][0]: d(dvy/dt)/dδ = (dFyf_dd*cos(δ) - Fyf*sin(δ)) / m */
    input_matrix_B[3][0] = time_step * ((dFyf_dd_cos - Fyf_sin) * VP_INV_MASS_1_PER_KG);

    /* B[4][0]: d(dω/dt)/dδ = lf*(dFyf_dd*cos(δ) - Fyf*sin(δ)) / Iz */
    input_matrix_B[4][0] = time_step * (VP_CG_TO_FRONT_AXLE_M * (dFyf_dd_cos - Fyf_sin) * VP_INV_YAW_INERTIA_1_PER_KGM2);

    /* B[2][1]: acceleration → vx directly (dt * 1) */
    input_matrix_B[2][1] = time_step;

    /* B[3][1], B[4][1]: acceleration affects lateral dynamics via load transfer. */
    {
        float inv_Fzf = util_recip(F_zf);
        float inv_Fzr = util_recip(F_zr);
        float C_Sf_norm = C_Sf_Fzf * inv_Fzf;
        float C_Sr_norm = C_Sr_Fzr * inv_Fzr;
        float dFzf_da = -(VP_MASS_KG * VP_CG_HEIGHT_M) / VP_WHEELBASE_M;
        float dFzr_da = -dFzf_da;
        float dFyf_da = C_Sf_norm * slip_terms.alpha_front * dFzf_da;
        float dFyr_da = C_Sr_norm * slip_terms.alpha_rear * dFzr_da;

        input_matrix_B[3][1] = time_step * ((dFyf_da * cos_delta + dFyr_da) * VP_INV_MASS_1_PER_KG);
        input_matrix_B[4][1] = time_step * ((VP_CG_TO_FRONT_AXLE_M * dFyf_da * cos_delta
                                            - VP_CG_TO_REAR_AXLE_M * dFyr_da)
                                           * VP_INV_YAW_INERTIA_1_PER_KGM2);
    }

    /*
     * B rows 0-1 are all zero:
     * Steering and torque don't directly change e_y or e_psi.
     * Their effect propagates through omega (row 4) and v_y (row 3),
     * which then affect e_y and e_psi through the A matrix coupling.
     */
    /* input_matrix_B[0][0..1] = 0 (already zeroed) */
    /* input_matrix_B[1][0..1] = 0 (already zeroed) */
}
