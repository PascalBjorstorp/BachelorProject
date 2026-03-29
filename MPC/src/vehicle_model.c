/**
 * @file vehicle_model.c
 * @brief Dynamic nonlinear bicycle model implementation.
 * @details Implements the dynamic bicycle model with direct acceleration
 *          input for F1/10th vehicle dynamics. The MPC commands an
 *          acceleration that the VESC motor controller enforces, so
 *          F_x = m * a_cmd directly.
 *
 * The linearization uses a Pacejka-like tire model for nonlinear
 * tire force saturation (lateral), while the forward prediction
 * uses a linear lateral tire model.
 *
 * State vector (6): [x, y, psi, v_x, v_y, omega]
 * Control vector (2): [delta, a_cmd]
 *
 * Model Equations:
 *   dx/dt        = v_x * cos(psi) - v_y * sin(psi)
 *   dy/dt        = v_x * sin(psi) + v_y * cos(psi)
 *   dpsi/dt      = omega
 *   dv_x/dt      = (F_x - F_yf * sin(delta) + m * v_y * omega) / m
 *   dv_y/dt      = (F_yf * cos(delta) + F_yr - m * v_x * omega) / m
 *   domega/dt    = (l_f * F_yf * cos(delta) - l_r * F_yr) / I_z
 *
 * Longitudinal force from commanded acceleration:
 *   F_x = m * a_cmd
 *
 * Tire model (linear, with friction and normal force scaling):
 *   alpha_f = delta - atan((v_y + l_f * omega) / v_x)
 *   alpha_r = -atan((v_y - l_r * omega) / v_x)
 *   F_zf = (m*g*l_r - F_x*h) / L    (front normal force)
 *   F_zr = (m*g*l_f + F_x*h) / L    (rear normal force)
 *   F_yf = mu * C_Sf * alpha_f * F_zf (front lateral tire force)
 *   F_yr = mu * C_Sr * alpha_r * F_zr (rear lateral tire force)
 *
 * The linearization (compute_linearization) uses a Pacejka-like model
 * for the effective tire stiffness (dFy/dalpha), providing realistic
 * tire force saturation at high slip angles.
 * @dependencies vehicle_model.h, <stdio.h>, <string.h>, <stdlib.h>
 */

#include "vehicle_model.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/*===========================================================================
 * Module State (Vehicle Parameters)
 *===========================================================================*/

/* Current vehicle parameters (initialized by vehicle_model_initialize). */
static VehicleParameters_t stored_vehicle_parameters;

/* Canonical default vehicle parameters used for initialization. */
static const VehicleParameters_t default_vehicle_parameters = {
    .wheelbase_meters = F110_DEFAULT_WHEELBASE_METERS,
    .distance_cg_to_front_axle = VEHICLE_CG_TO_FRONT_AXLE_M,
    .distance_cg_to_rear_axle = VEHICLE_CG_TO_REAR_AXLE_M,
    .vehicle_mass = F110_VEHICLE_MASS_KG,
    .yaw_moment_of_inertia = F110_YAW_INERTIA_KGM2,
    .front_cornering_stiffness = F110_FRONT_CORNERING_STIFFNESS,
    .rear_cornering_stiffness = F110_REAR_CORNERING_STIFFNESS,
    .max_steering_angle = F110_DEFAULT_MAXIMUM_STEERING_RADIANS,
    .max_velocity = F110_DEFAULT_MAXIMUM_VELOCITY_METERS_PER_SECOND,
    .min_velocity = F110_DEFAULT_MINIMUM_VELOCITY_METERS_PER_SECOND,
    .max_acceleration = F110_DEFAULT_MAXIMUM_ACCELERATION_METERS_PER_SECOND2,
    .min_acceleration = F110_DEFAULT_MINIMUM_ACCELERATION_METERS_PER_SECOND2,
    .height_cg_to_ground = F110_CG_HEIGHT_METERS,
    .gravity_acceleration = F110_GRAVITY_ACCELERATION_MS2,
};

/* Boolean flag (0 = uninitialized, 1 = ready) indicating whether
 * vehicle_model_initialize() has been called at least once. */
static uint8_t model_is_initialized = 0;

/* Cached reciprocals of constant parameters for repeated linearizations. */
static float cached_inv_mass = 0;   /* 1 / vehicle_mass_kg */
static float cached_inv_Iz   = 0;   /* 1 / yaw_moment_of_inertia */
static float cached_inv_L_wb = 0;   /* 1 / wheelbase */

/*===========================================================================
 * Initialization Functions
 *===========================================================================*/

/* Recompute cached reciprocals after any parameter change. */
static void recompute_cached_reciprocals(void)
{
    cached_inv_mass = util_recip(stored_vehicle_parameters.vehicle_mass);
    cached_inv_Iz   = util_recip(stored_vehicle_parameters.yaw_moment_of_inertia);
    cached_inv_L_wb = util_recip(stored_vehicle_parameters.wheelbase_meters);
}

void vehicle_model_initialize(void)
{
    stored_vehicle_parameters = default_vehicle_parameters;

    /* Optional scaling of cornering stiffness for the prediction model. */
    {
        const char *tire_env = getenv("MPC_TIRE_SCALE");
        if (tire_env) {
            float scale = (float)(atof(tire_env));
            stored_vehicle_parameters.front_cornering_stiffness =
                stored_vehicle_parameters.front_cornering_stiffness * scale;
            stored_vehicle_parameters.rear_cornering_stiffness =
                stored_vehicle_parameters.rear_cornering_stiffness * scale;
        }
    }

    recompute_cached_reciprocals();
    model_is_initialized = 1;
}

void vehicle_model_initialize_with_parameters(
    const VehicleParameters_t *parameters)
{
    stored_vehicle_parameters = *parameters;
    recompute_cached_reciprocals();
    model_is_initialized = 1;
}

VehicleParameters_t vehicle_model_get_parameters(void)
{
    return stored_vehicle_parameters;
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
        -stored_vehicle_parameters.max_steering_angle,
        stored_vehicle_parameters.max_steering_angle);

    /* Clamp acceleration to [min, max] */
    saturated_control.long_acc = util_clamp(
        raw_control->long_acc,
        stored_vehicle_parameters.min_acceleration,
        stored_vehicle_parameters.max_acceleration);

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

    /* Apply control saturation */
    ControlInput_t saturated_control = vehicle_model_saturate_control(control_input);

    /*
     * Extract current state variables
     */
    float psi = current_state->heading;
    float vx  = current_state->long_vel;
    float vy  = current_state->lat_vel;
    float omega = current_state->yaw_rate;

    /*
     * Extract control inputs
     */
    float delta   = saturated_control.steer_ang;
    float a_cmd  = saturated_control.long_acc;

    /*
     * Extract vehicle parameters
     */
    float lf   = stored_vehicle_parameters.distance_cg_to_front_axle;
    float lr   = stored_vehicle_parameters.distance_cg_to_rear_axle;
    float mass = stored_vehicle_parameters.vehicle_mass;
    float Iz   = stored_vehicle_parameters.yaw_moment_of_inertia;
    float C_Sf = stored_vehicle_parameters.front_cornering_stiffness;
    float C_Sr = stored_vehicle_parameters.rear_cornering_stiffness;

    /*
     * Compute trigonometric values
     */
    float cos_psi   = cosf(psi);
    float sin_psi   = sinf(psi);
    float cos_delta = cosf(delta);
    float sin_delta = sinf(delta);

    /*
     * Compute longitudinal force from acceleration command
     *
     * F_x = m * a_cmd
     *
     * The MPC outputs an acceleration command directly.
     * The longitudinal force is simply mass × acceleration.
     * This replaces the slip-ratio based model (F_x = C_x * kappa)
     * since the VESC accepts acceleration commands.
     */
    float Fx = mass * a_cmd;

    /*
     * Compute tire slip angles
     *
     * Minimum velocity floor to prevent division by zero at standstill.
     * Below this speed, the dynamic model degenerates — slip angles are
     * undefined when v_x ≈ 0.
     */
    float min_vx = MPC_MIN_SLIP_VELOCITY;
    float vx_safe = (vx > min_vx) ? vx : min_vx;

    /* Front slip angle: alpha_f = delta - atan((v_y + l_f * omega) / v_x) */
    float front_numerator = vy + lf * omega;
    float rear_numerator = vy - lr * omega;
    float front_ratio = util_div(front_numerator, vx_safe);
    float alpha_f = delta - atanf(front_ratio);
    float rear_ratio = util_div(rear_numerator, vx_safe);
    float alpha_r = -atanf(rear_ratio);

    /*
     * Compute normal forces (load transfer under acceleration)
     *
     * Static weight distribution plus longitudinal load transfer:
     *   F_zf = (m * g * l_r - F_x * h) / (l_f + l_r)
     *   F_zr = (m * g * l_f + F_x * h) / (l_f + l_r)
     */
    float g   = stored_vehicle_parameters.gravity_acceleration;
    float h   = stored_vehicle_parameters.height_cg_to_ground;
    float L   = stored_vehicle_parameters.wheelbase_meters;
    float mg  = mass * g;

    float F_zf = util_div(mg * lr - Fx * h, L);
    float F_zr = util_div(mg * lf + Fx * h, L);

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
    const float mu = F110_FRICTION_COEFFICIENT;
    float F_yf = mu * C_Sf * alpha_f * F_zf;
    float F_yr = mu * C_Sr * alpha_r * F_zr;

    /*
     * Compute state derivatives
    */

    /* dx/dt = v_x * cos(psi) - v_y * sin(psi) */
    float dx_dt = vx * cos_psi - vy * sin_psi;

    /* dy/dt = v_x * sin(psi) + v_y * cos(psi) */
    float dy_dt = vx * sin_psi + vy * cos_psi;

    /* dpsi/dt = omega */
    float dpsi_dt = omega;

    /* Full model: cos(δ)/sin(δ) force resolution for real-world accuracy */
    /* dv_x/dt = (F_x - F_yf * sin(delta) + m * v_y * omega) / m */
    float dvx_dt = util_div(Fx - F_yf * sin_delta + mass * vy * omega, mass);

    /* dv_y/dt = (F_yf * cos(delta) + F_yr - m * v_x * omega) / m */
    float dvy_dt = util_div(F_yf * cos_delta + F_yr - mass * vx * omega, mass);

    /* domega/dt = (l_f * F_yf * cos(delta) - l_r * F_yr) / I_z */
    float domega_dt = util_div(lf * F_yf * cos_delta - lr * F_yr, Iz);

    /*
     * Forward Euler integration: state[k+1] = state[k] + dt * derivative
     */
    next_state.pos_x = current_state->pos_x + time_step * dx_dt;
    next_state.pos_y = current_state->pos_y + time_step * dy_dt;
    next_state.heading = current_state->heading + time_step * dpsi_dt;
    next_state.long_vel = vx + time_step * dvx_dt;
    next_state.lat_vel = vy + time_step * dvy_dt;
    next_state.yaw_rate = omega + time_step * domega_dt;

    /*
     * Apply state constraints
     */

    /* Clamp longitudinal velocity to [min, max] */
    next_state.long_vel = util_clamp(
        next_state.long_vel,
        stored_vehicle_parameters.min_velocity,
        stored_vehicle_parameters.max_velocity);

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
 * Model Linearization
 *===========================================================================*/

void vehicle_model_compute_linearization(
    const VehicleState_t *operating_state,
    const ControlInput_t *operating_control,
    float time_step,
    float state_matrix_A[6][6],
    float input_matrix_B[6][2])
{
    /*
     * Extract operating point variables
     */
    float psi     = operating_state->heading;
    float vx      = operating_state->long_vel;
    float vy      = operating_state->lat_vel;
    float omega   = operating_state->yaw_rate;
    float delta   = operating_control->steer_ang;

    /*
     * Extract vehicle parameters
     */
    float lf   = stored_vehicle_parameters.distance_cg_to_front_axle;
    float lr   = stored_vehicle_parameters.distance_cg_to_rear_axle;
    float mass = stored_vehicle_parameters.vehicle_mass;
    float C_Sf = stored_vehicle_parameters.front_cornering_stiffness;
    float C_Sr = stored_vehicle_parameters.rear_cornering_stiffness;

    /*
     * Compute trigonometric values at operating point.
     * Short-circuit at zero: MPC always linearizes at δ=0, ψ=0 (Frenet),
     * saving 4 Taylor-series evaluations (~40 multiplies).
     */
    float cos_psi, sin_psi;
    if (psi == 0) { cos_psi = 1.0f; sin_psi = 0; }
    else { cos_psi = cosf(psi); sin_psi = sinf(psi); }

    float cos_delta, sin_delta;
    if (delta == 0) { cos_delta = 1.0f; sin_delta = 0; }
    else { cos_delta = cosf(delta); sin_delta = sinf(delta); }

    /*
     * Minimum velocity floor for linearization stability
     */
    float min_vx = MPC_MIN_SLIP_VELOCITY;
    float vx_safe = (vx > min_vx) ? vx : min_vx;
    /* Cache reciprocal for repeated slip-angle terms. */
    float inv_vx_safe = util_recip(vx_safe);

    /*
     * Compute slip ratio and longitudinal force at operating point
     *
     *   kappa = computed from tire model
     *   F_x = C_x * kappa
     *
     * Derivatives w.r.t. states (for v_x > epsilon):
     *   dFx/dvx    = 0 (Fx = m * a_cmd, independent of state)
     */
    /* With acceleration model: Fx = m * a_cmd
     * Fx does NOT depend on state, so all Jacobians are zero:
     *   dFx/dvx = 0
     * The Fx value is only needed for load transfer. */
    float Fx = mass * operating_control->long_acc;
    float dFx_dvx = 0;

    /*
     * Compute slip angle intermediates for Jacobian
     */
    float front_num = vy + lf * omega;
    float rear_num  = vy - lr * omega;

    /* Full atan model: d(atan(n/d))/dx = (d·dn/dx - n·dd/dx) / (d² + n²) */
    float front_num2 = front_num * front_num;
    float rear_num2  = rear_num * rear_num;

    float vx2 = vx_safe * vx_safe;
    float D_f = vx2 + front_num2;
    float D_r = vx2 + rear_num2;

    if (D_f == 0) D_f = 1.0f;
    if (D_r == 0) D_r = 1.0f;

    /* Cache reciprocal denominators used in Jacobian terms. */
    float inv_D_f = util_recip(D_f);
    float inv_D_r = util_recip(D_r);

    float daf_dvx    = front_num * inv_D_f;
    float daf_dvy    = -vx_safe * inv_D_f;
    float daf_domega = -(lf * vx_safe) * inv_D_f;

    float dar_dvx    = rear_num * inv_D_r;
    float dar_dvy    = -vx_safe * inv_D_r;
    float dar_domega = lr * vx_safe * inv_D_r;

    /*
     * Compute normal forces at operating point
     *
     *   F_zf = (m*g*l_r - F_x*h) / L
     *   F_zr = (m*g*l_f + F_x*h) / L
     */
    float g_acc = stored_vehicle_parameters.gravity_acceleration;
    float h_cg  = stored_vehicle_parameters.height_cg_to_ground;
    float mg = mass * g_acc;

    float F_zf = (mg * lr - Fx * h_cg) * cached_inv_L_wb;
    float F_zr = (mg * lf + Fx * h_cg) * cached_inv_L_wb;

    /*
     * Lateral tire force model for linearization.
     *
     * Full model: Pacejka-like saturation curve
     *   F_y = D · sin(C · atan(B · α))
     *   C_eff = dF_y/dα at operating-point slip angle
     */

    /* Compute front and rear slip angles at operating point */
    float front_ratio_lin = front_num * inv_vx_safe;
    float rear_ratio_lin  = rear_num * inv_vx_safe;

    const float mu = F110_FRICTION_COEFFICIENT;

    /* Linear stiffness (slope at α=0): mu * C_Sf * F_zf */
    float C_Sf_Fzf_linear = mu * C_Sf * F_zf;
    float C_Sr_Fzr_linear = mu * C_Sr * F_zr;

    /* Full Pacejka-like model for real-world accuracy */
    float alpha_f_op = delta - atanf(front_ratio_lin);
    float alpha_r_op = -atanf(rear_ratio_lin);

    const float C_shape = VP_C_SHAPE;
    const float inv_C_shape = util_recip(C_shape);
    const float min_stiffness_scale = MIN_STIFF_SCALE;

    float B_f = C_Sf * inv_C_shape;
    float B_r = C_Sr * inv_C_shape;

    float C_Sf_Fzf;
    float F_yf;
    {
        float D_pac_f = mu * F_zf;
        float Ba_f = B_f * alpha_f_op;
        float inner_f = C_shape * atanf(Ba_f);
        float cos_inner_f = cosf(inner_f);
        float denom_f_pac = 1.0f + Ba_f * Ba_f;
        float inv_denom_f_pac = util_recip(denom_f_pac);
        float C_eff_f = D_pac_f * C_shape * B_f * cos_inner_f * inv_denom_f_pac;
        float C_min_f = C_Sf_Fzf_linear * min_stiffness_scale;
        C_Sf_Fzf = (C_eff_f > C_min_f) ? C_eff_f : C_min_f;
        F_yf = D_pac_f * sinf(inner_f);
    }

    float C_Sr_Fzr;
    {
        float D_pac_r = mu * F_zr;
        float Ba_r = B_r * alpha_r_op;
        float inner_r = C_shape * atanf(Ba_r);
        float cos_inner_r = cosf(inner_r);
        float denom_r_pac = 1.0f + Ba_r * Ba_r;
        float inv_denom_r_pac = util_recip(denom_r_pac);
        float C_eff_r = D_pac_r * C_shape * B_r * cos_inner_r * inv_denom_r_pac;
        float C_min_r = C_Sr_Fzr_linear * min_stiffness_scale;
        C_Sr_Fzr = (C_eff_r > C_min_r) ? C_eff_r : C_min_r;
    }

    float dFyf_dvx    = C_Sf_Fzf * daf_dvx;
    float dFyf_dvy    = C_Sf_Fzf * daf_dvy;
    float dFyf_domega = C_Sf_Fzf * daf_domega;
    float dFyf_ddelta = C_Sf_Fzf;

    float dFyr_dvx    = C_Sr_Fzr * dar_dvx;
    float dFyr_dvy    = C_Sr_Fzr * dar_dvy;
    float dFyr_domega = C_Sr_Fzr * dar_domega;

    /*
     * Initialize A matrix as identity (6×6)
     * A_discrete = I + dt * A_continuous
     */
    for (int row = 0; row < 6; row++)
    {
        for (int col = 0; col < 6; col++)
        {
            state_matrix_A[row][col] = (row == col) ? 1.0f : 0;
        }
    }

    /*
     * Continuous-time A matrix (∂f/∂state) — only non-zero entries
     *
     * Rows 0-2: position/heading kinematics
     * Row 3: longitudinal velocity (dv_x/dt)
     * Row 4: lateral velocity (dv_y/dt)
     * Row 5: yaw rate (domega/dt)
     */

    /* Row 0: position X derivatives */
    state_matrix_A[0][2] += time_step * (-vx * sin_psi - vy * cos_psi);
    state_matrix_A[0][3] += time_step * cos_psi;
    state_matrix_A[0][4] -= time_step * sin_psi;

    /* Row 1: position Y derivatives */
    state_matrix_A[1][2] += time_step * (vx * cos_psi - vy * sin_psi);
    state_matrix_A[1][3] += time_step * sin_psi;
    state_matrix_A[1][4] += time_step * cos_psi;

    /* Row 2: heading derivative */
    state_matrix_A[2][5] += time_step;

    /* Row 3: longitudinal velocity derivatives
     * dvx/dt = (Fx - Fyf*sin(d) + m*vy*w) / m
     * Now Fx = m * a_cmd (direct acceleration input).
     */
    float inv_m = cached_inv_mass;

    /* Full model with cos(δ)/sin(δ) force resolution */
    /* A[3][3]: dFx/dvx/m + (-dFyf_dvx * sin(d)) / m */
    state_matrix_A[3][3] += time_step * ((dFx_dvx - dFyf_dvx * sin_delta) * inv_m);
    state_matrix_A[3][4] += time_step * (-dFyf_dvy * sin_delta * inv_m + omega);
    state_matrix_A[3][5] += time_step * (-dFyf_domega * sin_delta * inv_m + vy);

    state_matrix_A[4][3] += time_step * ((dFyf_dvx * cos_delta + dFyr_dvx - mass * omega) * inv_m);
    state_matrix_A[4][4] += time_step * ((dFyf_dvy * cos_delta + dFyr_dvy) * inv_m);
    state_matrix_A[4][5] += time_step * ((dFyf_domega * cos_delta + dFyr_domega - mass * vx) * inv_m);

    /* Row 5: yaw rate derivatives */
    float inv_Iz = cached_inv_Iz;

    state_matrix_A[5][3] += time_step * ((lf * dFyf_dvx * cos_delta - lr * dFyr_dvx) * inv_Iz);
    state_matrix_A[5][4] += time_step * ((lf * dFyf_dvy * cos_delta - lr * dFyr_dvy) * inv_Iz);
    state_matrix_A[5][5] += time_step * ((lf * dFyf_domega * cos_delta - lr * dFyr_domega) * inv_Iz);

    /*
     * Initialize B matrix as zeros (6×2) and add continuous terms × dt
     */
    for (int row = 0; row < 6; row++)
    {
        for (int col = 0; col < 2; col++)
        {
            input_matrix_B[row][col] = 0;
        }
    }

    /* Full model: B with cos(δ)/sin(δ) force resolution */
    /* B[3][0]: d(dvx/dt)/d(delta) × dt */
    float dFyf_dd_sin = dFyf_ddelta * sin_delta;
    float Fyf_cos = F_yf * cos_delta;
    input_matrix_B[3][0] = time_step * ((-dFyf_dd_sin - Fyf_cos) * inv_m);

    /* B[4][0]: d(dvy/dt)/d(delta) × dt */
    float dFyf_dd_cos = dFyf_ddelta * cos_delta;
    float Fyf_sin = F_yf * sin_delta;
    input_matrix_B[4][0] = time_step * ((dFyf_dd_cos - Fyf_sin) * inv_m);

    /* B[5][0]: d(domega/dt)/d(delta) × dt */
    input_matrix_B[5][0] = time_step * (lf * (dFyf_dd_cos - Fyf_sin) * inv_Iz);

    /* B[3][1]: d(dvx/dt)/d(a_cmd) × dt = dt */
    input_matrix_B[3][1] = time_step;
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
    float state_matrix_A[FRENET_STATE_DIMENSION][FRENET_STATE_DIMENSION],
    float input_matrix_B[FRENET_STATE_DIMENSION][2])
{

    /*
     * Extract operating point (body-frame states only — Frenet position
     * and heading error don't affect body-frame dynamics).
     */
    float vx    = frenet_state->flong_vel;
    float vy    = frenet_state->flat_vel;
    float omega = frenet_state->fyaw_rate;
    float delta = operating_control->steer_ang;

    /*
     * Vehicle parameters
     */
    float lf   = stored_vehicle_parameters.distance_cg_to_front_axle;
    float lr   = stored_vehicle_parameters.distance_cg_to_rear_axle;
    float mass = stored_vehicle_parameters.vehicle_mass;
    float C_Sf = stored_vehicle_parameters.front_cornering_stiffness;
    float C_Sr = stored_vehicle_parameters.rear_cornering_stiffness;

    /*
     * Steering trig — short-circuit at zero (MPC always linearizes at δ=0
     * in Frenet, saving 2 Taylor-series evaluations).
     */
    float cos_delta, sin_delta;
    if (delta == 0) { 
        cos_delta = 1.0f; 
        sin_delta = 0; 
    } else { 
        cos_delta = cosf(delta); 
        sin_delta = sinf(delta); 
    }

    /*
     * Velocity floor for linearization stability
     */
    float min_vx = MPC_MIN_SLIP_VELOCITY;
    float vx_safe = (vx > min_vx) ? vx : min_vx;
    float inv_vx_safe = util_recip(vx_safe);

    /*
     * Longitudinal force (for load transfer only).
     * Fx = m * a_cmd — independent of state, so dFx/dvx = 0.
     */
    float Fx = mass * operating_control->long_acc;
    float dFx_dvx = 0;

    /*
     * Slip angle intermediates for Jacobians
     */
    float front_num = vy + lf * omega;
    float rear_num = vy - lr * omega;

    /* Full atan model: d(atan(n/d))/dx = (d·dn/dx - n·dd/dx) / (d² + n²) */
    float front_num2 = front_num * front_num;
    float rear_num2 = rear_num * rear_num;
    float vx2 = vx_safe * vx_safe;

    float D_f = vx2 + front_num2;
    float D_r = vx2 + rear_num2;
    if (D_f == 0) D_f = 1.0f;
    if (D_r == 0) D_r = 1.0f;

    float inv_D_f = util_recip(D_f);
    float inv_D_r = util_recip(D_r);

    float daf_dvx = front_num * inv_D_f;
    float daf_dvy = -vx_safe * inv_D_f;
    float daf_domega = -(lf * vx_safe) * inv_D_f;

    float dar_dvx = rear_num * inv_D_r;
    float dar_dvy = -vx_safe * inv_D_r;
    float dar_domega = lr * vx_safe * inv_D_r;

    /*
     * Normal forces with load transfer
     *   F_zf = (m*g*l_r - F_x*h) / L
     *   F_zr = (m*g*l_f + F_x*h) / L
     */
    float g_acc = stored_vehicle_parameters.gravity_acceleration;
    float h_cg  = stored_vehicle_parameters.height_cg_to_ground;
    float mg = mass * g_acc;

    float F_zf = (mg * lr - Fx * h_cg) * cached_inv_L_wb;
    float F_zr = (mg * lf + Fx * h_cg) * cached_inv_L_wb;

    /*
     * Tire force model for linearization
     */
    float front_ratio_lin = front_num * inv_vx_safe;
    float rear_ratio_lin = rear_num * inv_vx_safe;

    const float mu = F110_FRICTION_COEFFICIENT;

    float C_Sf_Fzf_linear = mu * C_Sf * F_zf;
    float C_Sr_Fzr_linear = mu * C_Sr * F_zr;

    /* Full Pacejka-like tire model for real-world accuracy */
    float alpha_f_op = delta - atanf(front_ratio_lin);
    float alpha_r_op = -atanf(rear_ratio_lin);

    const float C_shape = VP_C_SHAPE;
    const float inv_C_shape = util_recip(C_shape);
    const float min_stiffness_scale = MIN_STIFF_SCALE;

    float B_f = C_Sf * inv_C_shape;
    float B_r = C_Sr * inv_C_shape;

    float C_Sf_Fzf;
    float F_yf;
    {
        float D_pac_f = mu * F_zf;
        float Ba_f = B_f * alpha_f_op;
        float inner_f = C_shape * atanf(Ba_f);
        float cos_inner_f = cosf(inner_f);
        float denom_f_pac = 1.0f + Ba_f * Ba_f;
        float inv_denom_f_pac = util_recip(denom_f_pac);
        float C_eff_f = D_pac_f * C_shape * B_f * cos_inner_f * inv_denom_f_pac;
        float C_min_f = C_Sf_Fzf_linear * min_stiffness_scale;
        C_Sf_Fzf = (C_eff_f > C_min_f) ? C_eff_f : C_min_f;
        F_yf = D_pac_f * sinf(inner_f);
    }

    float C_Sr_Fzr;
    {
        float D_pac_r = mu * F_zr;
        float Ba_r = B_r * alpha_r_op;
        float inner_r = C_shape * atanf(Ba_r);
        float cos_inner_r = cosf(inner_r);
        float denom_r_pac = 1.0f + Ba_r * Ba_r;
        float inv_denom_r_pac = util_recip(denom_r_pac);
        float C_eff_r = D_pac_r * C_shape * B_r * cos_inner_r * inv_denom_r_pac;
        float C_min_r = C_Sr_Fzr_linear * min_stiffness_scale;
        C_Sr_Fzr = (C_eff_r > C_min_r) ? C_eff_r : C_min_r;
    }

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

    /*
     * Cached reciprocals for mass and inertia
     */
    float inv_m  = cached_inv_mass;
    float inv_Iz = cached_inv_Iz;

    /*
    * Initialize Frenet matrices to zero.
     */
    for (int i = 0; i < FRENET_STATE_DIMENSION; i++)
    {
        for (int j = 0; j < FRENET_STATE_DIMENSION; j++)
        {
            state_matrix_A[i][j] = 0;
        }
        input_matrix_B[i][0] = 0;
        input_matrix_B[i][1] = 0;
    }

    /*
     * Row 0: e_y dynamics (Frenet kinematics)
     *   Continuous: e_y_dot = v_x * sin(e_psi) + v_y * cos(e_psi)
     *   Linearized (e_psi ≈ 0): e_y_dot ≈ v_x * e_psi + v_y
     *   Discrete: e_y[k+1] = e_y[k] + dt * (v_x * e_psi[k] + v_y[k])
     *
     *   A[0][0] = 1         (identity)
     *   A[0][1] = dt * v_x  (heading error drives lateral drift)
     *   A[0][2] = 0         (∂/∂v_x = sin(e_psi) ≈ 0 at e_psi=0)
     *   A[0][3] = dt        (lateral velocity directly changes e_y)
     *   A[0][4] = 0         (no direct omega coupling)
     */
    state_matrix_A[0][0] = 1.0f;
    state_matrix_A[0][1] = time_step * vx;
    state_matrix_A[0][3] = time_step;

    /*
     * Row 1: e_psi dynamics (Frenet kinematics)
     *   Continuous: e_psi_dot = omega - kappa * v_x * cos(e_psi) / (1 - kappa * e_y)
     *   Linearized (e_y ≈ 0, e_psi ≈ 0): e_psi_dot ≈ omega - kappa * v_x
     *   Discrete: e_psi[k+1] = e_psi[k] + dt * (omega[k] - kappa * v_x[k])
     *
     *   A[1][1] = 1                (identity)
     *   A[1][2] = -dt * kappa      (speed along curved path changes heading error)
     *   A[1][4] = dt               (yaw rate directly changes heading error)
     */
    state_matrix_A[1][1] = 1.0f;
    state_matrix_A[1][2] = -time_step * path_curvature;
    state_matrix_A[1][4] = time_step;

    /*
     * Rows 2-4: Body-frame dynamics (computed directly — avoids the
     * 6×6 global matrix allocation and discarded rows 0-2).
     *
     * These are the Jacobians of [v_x_dot, v_y_dot, omega_dot] w.r.t.
     * body states [v_x, v_y, omega], discretized via Forward Euler.
     */
    /* Full model with cos(δ)/sin(δ) force resolution */

    /* Row 2: dvx/dt = (Fx - Fyf*sin(δ) + m*vy*ω) / m */
    state_matrix_A[2][2] = 1.0f + time_step * ((dFx_dvx - dFyf_dvx * sin_delta) * inv_m);
    state_matrix_A[2][3] = time_step * (-dFyf_dvy * sin_delta * inv_m + omega);
    state_matrix_A[2][4] = time_step * (-dFyf_domega * sin_delta * inv_m + vy);

    /* Row 3: dvy/dt = (Fyf*cos(δ) + Fyr - m*vx*ω) / m */
    state_matrix_A[3][2] = time_step * ((dFyf_dvx * cos_delta + dFyr_dvx - mass * omega) * inv_m);
    state_matrix_A[3][3] = 1.0f + time_step * ((dFyf_dvy * cos_delta + dFyr_dvy) * inv_m);
    state_matrix_A[3][4] = time_step * ((dFyf_domega * cos_delta + dFyr_domega - mass * vx) * inv_m);

    /* Row 4: dω/dt = (lf*Fyf*cos(δ) - lr*Fyr) / Iz */
    state_matrix_A[4][2] = time_step * ((lf * dFyf_dvx * cos_delta - lr * dFyr_dvx) * inv_Iz);
    state_matrix_A[4][3] = time_step * ((lf * dFyf_dvy * cos_delta - lr * dFyr_dvy) * inv_Iz);
    state_matrix_A[4][4] = 1.0f + time_step * ((lf * dFyf_domega * cos_delta - lr * dFyr_domega) * inv_Iz);

    /* B matrix — steering column with cos(δ)/sin(δ) force resolution */
    float dFyf_dd_sin = dFyf_ddelta * sin_delta;
    float Fyf_cos = F_yf * cos_delta;
    float dFyf_dd_cos = dFyf_ddelta * cos_delta;
    float Fyf_sin = F_yf * sin_delta;

    /* B[2][0]: d(dvx/dt)/dδ = (-dFyf_dd*sin(δ) - Fyf*cos(δ)) / m */
    input_matrix_B[2][0] = time_step * ((-dFyf_dd_sin - Fyf_cos) * inv_m);

    /* B[3][0]: d(dvy/dt)/dδ = (dFyf_dd*cos(δ) - Fyf*sin(δ)) / m */
    input_matrix_B[3][0] = time_step * ((dFyf_dd_cos - Fyf_sin) * inv_m);

    /* B[4][0]: d(dω/dt)/dδ = lf*(dFyf_dd*cos(δ) - Fyf*sin(δ)) / Iz */
    input_matrix_B[4][0] = time_step * (lf * (dFyf_dd_cos - Fyf_sin) * inv_Iz);

    /* B[2][1]: acceleration → vx directly (dt * 1) */
    input_matrix_B[2][1] = time_step;

    /*
     * B rows 0-1 are all zero:
     * Steering and torque don't directly change e_y or e_psi.
     * Their effect propagates through omega (row 4) and v_y (row 3),
     * which then affect e_y and e_psi through the A matrix coupling.
     */
    /* input_matrix_B[0][0..1] = 0 (already zeroed) */
    /* input_matrix_B[1][0..1] = 0 (already zeroed) */
}
