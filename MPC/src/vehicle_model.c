/**
 * @file vehicle_model.c
 * @brief Dynamic Nonlinear Bicycle Model Implementation
 *
 * Implements the dynamic bicycle model with linear tire forces and wheel
 * dynamics for F1/10th vehicle dynamics. All calculations use fixed-point
 * arithmetic for FPGA compatibility.
 *
 * State vector (7): [x, y, psi, v_x, v_y, omega, omega_w]
 * Control vector (2): [delta, T_motor]
 *
 * Model Equations:
 *   dx/dt        = v_x * cos(psi) - v_y * sin(psi)
 *   dy/dt        = v_x * sin(psi) + v_y * cos(psi)
 *   dpsi/dt      = omega
 *   dv_x/dt      = (F_x - F_yf * sin(delta) + m * v_y * omega) / m
 *   dv_y/dt      = (F_yf * cos(delta) + F_yr - m * v_x * omega) / m
 *   domega/dt    = (l_f * F_yf * cos(delta) - l_r * F_yr) / I_z
 *   domega_w/dt  = (T_motor / G_ratio - F_x * R_w) / I_w
 *
 * Longitudinal force via slip ratio:
 *   kappa = (R_w * omega_w - v_x) / max(|v_x|, epsilon)
 *   F_x = C_x * kappa
 *
 * Tire model (linear, with normal force scaling):
 *   alpha_f = delta - atan((v_y + l_f * omega) / v_x)
 *   alpha_r = -atan((v_y - l_r * omega) / v_x)
 *   F_zf = (m*g*l_r - F_x*h) / L    (front normal force)
 *   F_zr = (m*g*l_f + F_x*h) / L    (rear normal force)
 *   F_yf = C_Sf * alpha_f * F_zf     (front lateral tire force)
 *   F_yr = C_Sr * alpha_r * F_zr     (rear lateral tire force)
 */

#include "vehicle_model.h"
#include "fp_math.h"
#include <stdio.h>

/*===========================================================================
 * Module State (Vehicle Parameters)
 *===========================================================================*/

/** Current vehicle parameters (initialized by vehicle_model_initialize) */
static VehicleParameters_t stored_vehicle_parameters;

/** Flag indicating if model has been initialized */
static uint8_t model_is_initialized = 0;

/*===========================================================================
 * Initialization Functions
 *===========================================================================*/

void vehicle_model_initialize(void)
{
    stored_vehicle_parameters.wheelbase_meters =
        F110_DEFAULT_WHEELBASE_METERS;

    stored_vehicle_parameters.distance_cg_to_front_axle_meters =
        F110_DIST_CG_TO_FRONT_AXLE_METERS;

    stored_vehicle_parameters.distance_cg_to_rear_axle_meters =
        F110_DIST_CG_TO_REAR_AXLE_METERS;

    stored_vehicle_parameters.vehicle_mass_kg =
        F110_VEHICLE_MASS_KG;

    stored_vehicle_parameters.yaw_moment_of_inertia_kgm2 =
        F110_YAW_INERTIA_KGM2;

    stored_vehicle_parameters.front_cornering_stiffness =
        F110_FRONT_CORNERING_STIFFNESS;

    stored_vehicle_parameters.rear_cornering_stiffness =
        F110_REAR_CORNERING_STIFFNESS;

    stored_vehicle_parameters.maximum_steering_angle_radians =
        F110_DEFAULT_MAXIMUM_STEERING_RADIANS;

    stored_vehicle_parameters.maximum_velocity_meters_per_second =
        F110_DEFAULT_MAXIMUM_VELOCITY_METERS_PER_SECOND;

    stored_vehicle_parameters.minimum_velocity_meters_per_second =
        F110_DEFAULT_MINIMUM_VELOCITY_METERS_PER_SECOND;

    stored_vehicle_parameters.maximum_motor_torque_newton_meters =
        F110_DEFAULT_MAX_MOTOR_TORQUE_NM;

    stored_vehicle_parameters.minimum_motor_torque_newton_meters =
        F110_DEFAULT_MIN_MOTOR_TORQUE_NM;

    stored_vehicle_parameters.omega = 
        F110_DEFAULT_YAW_RATE;

    stored_vehicle_parameters.height_cg_to_ground_meters =
        F110_CG_HEIGHT_METERS;

    stored_vehicle_parameters.gravity_acceleration_meters_per_second_squared =
        F110_GRAVITY_ACCELERATION_MS2;

    /* Wheel / Drivetrain parameters (7-state model) */
    stored_vehicle_parameters.wheel_radius_meters =
        F110_WHEEL_RADIUS_METERS;

    stored_vehicle_parameters.drivetrain_inertia_kgm2 =
        F110_DRIVETRAIN_INERTIA_KGM2;

    stored_vehicle_parameters.longitudinal_tire_stiffness =
        F110_LONGITUDINAL_TIRE_STIFFNESS;

    stored_vehicle_parameters.gear_ratio =
        F110_GEAR_RATIO;

    model_is_initialized = 1;
}

void vehicle_model_initialize_with_parameters(
    const VehicleParameters_t *parameters)
{
    stored_vehicle_parameters = *parameters;
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
    saturated_control.steering_angle_radians = fp_clamp(
        raw_control->steering_angle_radians,
        fp_neg(stored_vehicle_parameters.maximum_steering_angle_radians),
        stored_vehicle_parameters.maximum_steering_angle_radians);

    /* Clamp motor torque to [min_torque, max_torque] */
    saturated_control.motor_torque_newton_meters = fp_clamp(
        raw_control->motor_torque_newton_meters,
        stored_vehicle_parameters.minimum_motor_torque_newton_meters,
        stored_vehicle_parameters.maximum_motor_torque_newton_meters);

    return saturated_control;
}

/*===========================================================================
 * Single-Step State Prediction
 *===========================================================================*/

VehicleState_t vehicle_model_predict_next_state(
    const VehicleState_t *current_state,
    const ControlInput_t *control_input,
    fixed_point_t time_step)
{
    VehicleState_t next_state;

    /* Apply control saturation */
    ControlInput_t saturated_control = vehicle_model_saturate_control(control_input);

    /*
     * Extract current state variables
     */
    fixed_point_t psi = current_state->heading_angle_radians;
    fixed_point_t vx  = current_state->longitudinal_velocity_meters_per_second;
    fixed_point_t vy  = current_state->lateral_velocity_meters_per_second;
    fixed_point_t omega = current_state->yaw_rate_radians_per_second;
    fixed_point_t omega_w = current_state->wheel_speed_radians_per_second;

    /*
     * Extract control inputs
     */
    fixed_point_t delta   = saturated_control.steering_angle_radians;
    fixed_point_t T_motor = saturated_control.motor_torque_newton_meters;

    /*
     * Extract vehicle parameters
     */
    fixed_point_t lf   = stored_vehicle_parameters.distance_cg_to_front_axle_meters;
    fixed_point_t lr   = stored_vehicle_parameters.distance_cg_to_rear_axle_meters;
    fixed_point_t mass = stored_vehicle_parameters.vehicle_mass_kg;
    fixed_point_t Iz   = stored_vehicle_parameters.yaw_moment_of_inertia_kgm2;
    fixed_point_t C_Sf = stored_vehicle_parameters.front_cornering_stiffness;
    fixed_point_t C_Sr = stored_vehicle_parameters.rear_cornering_stiffness;

    /* Wheel / drivetrain parameters */
    fixed_point_t Rw      = stored_vehicle_parameters.wheel_radius_meters;
    fixed_point_t Iw      = stored_vehicle_parameters.drivetrain_inertia_kgm2;
    fixed_point_t Cx      = stored_vehicle_parameters.longitudinal_tire_stiffness;
    fixed_point_t G_ratio = stored_vehicle_parameters.gear_ratio;

    /*
     * Compute trigonometric values
     */
    fixed_point_t cos_psi   = fp_cos(psi);
    fixed_point_t sin_psi   = fp_sin(psi);
    fixed_point_t cos_delta = fp_cos(delta);
    fixed_point_t sin_delta = fp_sin(delta);

    /*
     * Compute longitudinal force from wheel slip ratio
     *
     * Slip ratio: kappa = (R_w * omega_w - v_x) / max(|v_x|, epsilon)
     * Longitudinal force: F_x = C_x * kappa
     *
     * Positive kappa = traction (wheel spinning faster than ground speed)
     * Negative kappa = braking (wheel spinning slower than ground speed)
     */
    fixed_point_t slip_vx_safe = (vx > FP_CONST(0.5)) ? vx : FP_CONST(0.5);
    fixed_point_t kappa = fp_div(
        fp_sub(fp_mul(Rw, omega_w), vx),
        slip_vx_safe);
    fixed_point_t Fx = fp_mul(Cx, kappa);

    /*
     * Compute tire slip angles
     *
     * Minimum velocity floor to prevent division by zero at standstill.
     * Below this speed, the dynamic model degenerates — slip angles are
     * undefined when v_x ≈ 0.
     */
    fixed_point_t min_vx = FP_CONST(0.5);
    fixed_point_t vx_safe = (vx > min_vx) ? vx : min_vx;

    /* Front slip angle: alpha_f = delta - atan((v_y + l_f * omega) / v_x) */
    fixed_point_t front_numerator = fp_add(vy, fp_mul(lf, omega));
    fixed_point_t front_ratio = fp_div(front_numerator, vx_safe);
    fixed_point_t alpha_f = fp_sub(delta, fp_atan(front_ratio));

    /* Rear slip angle: alpha_r = -atan((v_y - l_r * omega) / v_x) */
    fixed_point_t rear_numerator = fp_sub(vy, fp_mul(lr, omega));
    fixed_point_t rear_ratio = fp_div(rear_numerator, vx_safe);
    fixed_point_t alpha_r = fp_neg(fp_atan(rear_ratio));

    /*
     * Compute normal forces (load transfer under acceleration)
     *
     * Static weight distribution plus longitudinal load transfer:
     *   F_zf = (m * g * l_r - F_x * h) / (l_f + l_r)
     *   F_zr = (m * g * l_f + F_x * h) / (l_f + l_r)
     *
     * Under acceleration (F_x > 0): front unloads, rear loads up.
     * Under braking   (F_x < 0): front loads up, rear unloads.
     */
    fixed_point_t g   = stored_vehicle_parameters.gravity_acceleration_meters_per_second_squared;
    fixed_point_t h   = stored_vehicle_parameters.height_cg_to_ground_meters;
    fixed_point_t L   = stored_vehicle_parameters.wheelbase_meters;
    fixed_point_t mg  = fp_mul(mass, g);

    fixed_point_t F_zf = fp_div(
        fp_sub(fp_mul(mg, lr), fp_mul(Fx, h)), L);
    fixed_point_t F_zr = fp_div(
        fp_add(fp_mul(mg, lf), fp_mul(Fx, h)), L);

    /*
     * Compute tire contact patch velocities
     *
     * Total velocity magnitude at each tire:
     *   V_tf = sqrt((v_y + l_f * omega)^2 + v_x^2)   [front]
     *   V_tr = sqrt((v_y - l_r * omega)^2 + v_x^2)   [rear]
     */
    fixed_point_t V_tf = fp_sqrt(fp_add(
        fp_mul(front_numerator, front_numerator),
        fp_mul(vx_safe, vx_safe)));
    fixed_point_t V_tr = fp_sqrt(fp_add(
        fp_mul(rear_numerator, rear_numerator),
        fp_mul(vx_safe, vx_safe)));

    /*
     * Compute wheel-frame longitudinal velocities
     *
     * Projection of tire velocity onto wheel heading direction:
     *   v_omega_xf = V_tf * cos(alpha_f)
     *   v_omega_xr = V_tr * cos(alpha_r)
     *
     * These represent the longitudinal speed the tire "sees" and are needed
     * for longitudinal slip ratio computation: kappa = (R*omega_w - v_omega_x) / v_omega_x
     */
    fixed_point_t cos_alpha_f = fp_cos(alpha_f);
    fixed_point_t cos_alpha_r = fp_cos(alpha_r);
    fixed_point_t v_omega_xf = fp_mul(V_tf, cos_alpha_f);
    fixed_point_t v_omega_xr = fp_mul(V_tr, cos_alpha_r);

    /* NOTE: V_tf, V_tr, v_omega_xf, v_omega_xr are computed
     * but not yet used in the tire force model. They will be integrated
     * when the combined slip / Pacejka model is implemented.
     * Suppress unused warnings: */
    (void)V_tf; (void)V_tr;
    (void)v_omega_xf; (void)v_omega_xr;

    /*
     * Compute lateral tire forces (linear tire model with normal force)
     *
     * The cornering stiffness is scaled by normal force to capture
     * load transfer effects (braking loads the front, accelerating
     * loads the rear):
     *
     *   F_yf = C_Sf * alpha_f * F_zf
     *   F_yr = C_Sr * alpha_r * F_zr
     *
     * C_Sf/C_Sr here have units [1/rad] (normalized cornering stiffness),
     * and F_zf/F_zr [N] gives the tire force in [N].
     */
    fixed_point_t F_yf = fp_mul(fp_mul(C_Sf, alpha_f), F_zf);
    fixed_point_t F_yr = fp_mul(fp_mul(C_Sr, alpha_r), F_zr);

    /*
     * Compute state derivatives
     */

    /* dx/dt = v_x * cos(psi) - v_y * sin(psi) */
    fixed_point_t dx_dt = fp_sub(
        fp_mul(vx, cos_psi),
        fp_mul(vy, sin_psi));

    /* dy/dt = v_x * sin(psi) + v_y * cos(psi) */
    fixed_point_t dy_dt = fp_add(
        fp_mul(vx, sin_psi),
        fp_mul(vy, cos_psi));

    /* dpsi/dt = omega */
    fixed_point_t dpsi_dt = omega;

    /* dv_x/dt = (F_x - F_yf * sin(delta) + m * v_y * omega) / m */
    fixed_point_t dvx_dt = fp_div(
        fp_add(
            fp_sub(Fx, fp_mul(F_yf, sin_delta)),
            fp_mul(mass, fp_mul(vy, omega))),
        mass);

    /* dv_y/dt = (F_yf * cos(delta) + F_yr - m * v_x * omega) / m */
    fixed_point_t dvy_dt = fp_div(
        fp_sub(
            fp_add(fp_mul(F_yf, cos_delta), F_yr),
            fp_mul(mass, fp_mul(vx, omega))),
        mass);

    /* domega/dt = (l_f * F_yf * cos(delta) - l_r * F_yr) / I_z */
    fixed_point_t domega_dt = fp_div(
        fp_sub(
            fp_mul(lf, fp_mul(F_yf, cos_delta)),
            fp_mul(lr, F_yr)),
        Iz);

    /* domega_w/dt = (T_motor * G_ratio - F_x * R_w) / I_w
     * Wheel dynamics: motor torque amplified by gear ratio drives wheel,
     * tire longitudinal force resists. G > 1 means the gearbox trades
     * motor speed for wheel torque: T_wheel = T_motor × G. */
    fixed_point_t T_wheel = fp_mul(T_motor, G_ratio);
    fixed_point_t domega_w_dt = fp_div(
        fp_sub(T_wheel, fp_mul(Fx, Rw)),
        Iw);

    /*
     * Forward Euler integration: state[k+1] = state[k] + dt * derivative
     */
    next_state.position_x_meters = fp_add(
        current_state->position_x_meters,
        fp_mul(time_step, dx_dt));

    next_state.position_y_meters = fp_add(
        current_state->position_y_meters,
        fp_mul(time_step, dy_dt));

    next_state.heading_angle_radians = fp_add(
        current_state->heading_angle_radians,
        fp_mul(time_step, dpsi_dt));

    next_state.longitudinal_velocity_meters_per_second = fp_add(
        vx, fp_mul(time_step, dvx_dt));

    next_state.lateral_velocity_meters_per_second = fp_add(
        vy, fp_mul(time_step, dvy_dt));

    next_state.yaw_rate_radians_per_second = fp_add(
        omega, fp_mul(time_step, domega_dt));

    next_state.wheel_speed_radians_per_second = fp_add(
        omega_w, fp_mul(time_step, domega_w_dt));

    /*
     * Apply state constraints
     */

    /* Clamp longitudinal velocity to [min, max] */
    next_state.longitudinal_velocity_meters_per_second = fp_clamp(
        next_state.longitudinal_velocity_meters_per_second,
        stored_vehicle_parameters.minimum_velocity_meters_per_second,
        stored_vehicle_parameters.maximum_velocity_meters_per_second);

    /* Clamp wheel speed to non-negative (no reverse) */
    if (next_state.wheel_speed_radians_per_second < 0)
    {
        next_state.wheel_speed_radians_per_second = 0;
    }

    /* Normalize heading angle to [-pi, +pi] */
    while (next_state.heading_angle_radians > FP_PI)
    {
        next_state.heading_angle_radians = fp_sub(
            next_state.heading_angle_radians,
            FP_TWO_PI);
    }
    while (next_state.heading_angle_radians < -FP_PI)
    {
        next_state.heading_angle_radians = fp_add(
            next_state.heading_angle_radians,
            FP_TWO_PI);
    }

    return next_state;
}

/*===========================================================================
 * Multi-Step Trajectory Prediction
 *===========================================================================*/

void vehicle_model_predict_trajectory(
    const VehicleState_t *initial_state,
    const ControlInput_t *control_sequence,
    fixed_point_t time_step,
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
    fixed_point_t time_step,
    fixed_point_t state_matrix_A[7][7],
    fixed_point_t input_matrix_B[7][2])
{
    /*
     * Extract operating point variables
     */
    fixed_point_t psi     = operating_state->heading_angle_radians;
    fixed_point_t vx      = operating_state->longitudinal_velocity_meters_per_second;
    fixed_point_t vy      = operating_state->lateral_velocity_meters_per_second;
    fixed_point_t omega   = operating_state->yaw_rate_radians_per_second;
    fixed_point_t omega_w = operating_state->wheel_speed_radians_per_second;
    fixed_point_t delta   = operating_control->steering_angle_radians;

    /*
     * Extract vehicle parameters
     */
    fixed_point_t lf   = stored_vehicle_parameters.distance_cg_to_front_axle_meters;
    fixed_point_t lr   = stored_vehicle_parameters.distance_cg_to_rear_axle_meters;
    fixed_point_t mass = stored_vehicle_parameters.vehicle_mass_kg;
    fixed_point_t Iz   = stored_vehicle_parameters.yaw_moment_of_inertia_kgm2;
    fixed_point_t C_Sf = stored_vehicle_parameters.front_cornering_stiffness;
    fixed_point_t C_Sr = stored_vehicle_parameters.rear_cornering_stiffness;
    fixed_point_t Rw   = stored_vehicle_parameters.wheel_radius_meters;
    fixed_point_t Iw   = stored_vehicle_parameters.drivetrain_inertia_kgm2;
    fixed_point_t Cx   = stored_vehicle_parameters.longitudinal_tire_stiffness;
    fixed_point_t G_ratio = stored_vehicle_parameters.gear_ratio;

    /*
     * Compute trigonometric values at operating point
     */
    fixed_point_t cos_psi   = fp_cos(psi);
    fixed_point_t sin_psi   = fp_sin(psi);
    fixed_point_t cos_delta = fp_cos(delta);
    fixed_point_t sin_delta = fp_sin(delta);

    /*
     * Minimum velocity floor for linearization stability
     */
    fixed_point_t min_vx = FP_CONST(0.5);
    fixed_point_t vx_safe = (vx > min_vx) ? vx : min_vx;

    /*
     * Compute slip ratio and longitudinal force at operating point
     *
     *   kappa = (R_w * omega_w - v_x) / v_x
     *   F_x = C_x * kappa
     *
     * Derivatives w.r.t. states (for v_x > epsilon):
     *   dFx/dvx    = -C_x * R_w * omega_w / vx^2
     *   dFx/domega_w = C_x * R_w / vx
     */
    fixed_point_t kappa = fp_div(
        fp_sub(fp_mul(Rw, omega_w), vx_safe), vx_safe);
    fixed_point_t Fx = fp_mul(Cx, kappa);

    fixed_point_t vx2 = fp_mul(vx_safe, vx_safe);
    fixed_point_t dFx_dvx = fp_neg(fp_div(fp_mul(Cx, fp_mul(Rw, omega_w)), vx2));
    fixed_point_t dFx_domega_w = fp_div(fp_mul(Cx, Rw), vx_safe);

    /*
     * Compute slip angle intermediates for Jacobian
     */
    fixed_point_t front_num = fp_add(vy, fp_mul(lf, omega));
    fixed_point_t rear_num  = fp_sub(vy, fp_mul(lr, omega));

    fixed_point_t front_num2 = fp_mul(front_num, front_num);
    fixed_point_t rear_num2  = fp_mul(rear_num, rear_num);

    fixed_point_t D_f = fp_add(vx2, front_num2);
    fixed_point_t D_r = fp_add(vx2, rear_num2);

    if (D_f == 0) D_f = FP_ONE;
    if (D_r == 0) D_r = FP_ONE;

    /*
     * Partial derivatives of slip angles w.r.t. states
     */
    fixed_point_t daf_dvx    = fp_div(front_num, D_f);
    fixed_point_t daf_dvy    = fp_neg(fp_div(vx_safe, D_f));
    fixed_point_t daf_domega = fp_neg(fp_div(fp_mul(lf, vx_safe), D_f));

    fixed_point_t dar_dvx    = fp_div(rear_num, D_r);
    fixed_point_t dar_dvy    = fp_neg(fp_div(vx_safe, D_r));
    fixed_point_t dar_domega = fp_div(fp_mul(lr, vx_safe), D_r);

    /*
     * Compute normal forces at operating point
     *
     *   F_zf = (m*g*l_r - F_x*h) / L
     *   F_zr = (m*g*l_f + F_x*h) / L
     */
    fixed_point_t g_acc = stored_vehicle_parameters.gravity_acceleration_meters_per_second_squared;
    fixed_point_t h_cg  = stored_vehicle_parameters.height_cg_to_ground_meters;
    fixed_point_t L_wb  = stored_vehicle_parameters.wheelbase_meters;
    fixed_point_t mg    = fp_mul(mass, g_acc);

    fixed_point_t F_zf = fp_div(
        fp_sub(fp_mul(mg, lr), fp_mul(Fx, h_cg)), L_wb);
    fixed_point_t F_zr = fp_div(
        fp_add(fp_mul(mg, lf), fp_mul(Fx, h_cg)), L_wb);

    /*
     * Lateral tire force derivatives w.r.t. states
     *
     * F_yf = C_Sf * alpha_f * F_zf  →  dF_yf/dx = C_Sf * F_zf * d(alpha_f)/dx
     * F_yr = C_Sr * alpha_r * F_zr  →  dF_yr/dx = C_Sr * F_zr * d(alpha_r)/dx
     *
     * (Neglecting cross-coupling dF_zf/dvx × alpha_f, which is second-order small)
     */
    fixed_point_t C_Sf_Fzf = fp_mul(C_Sf, F_zf);
    fixed_point_t C_Sr_Fzr = fp_mul(C_Sr, F_zr);

    fixed_point_t dFyf_dvx    = fp_mul(C_Sf_Fzf, daf_dvx);
    fixed_point_t dFyf_dvy    = fp_mul(C_Sf_Fzf, daf_dvy);
    fixed_point_t dFyf_domega = fp_mul(C_Sf_Fzf, daf_domega);
    fixed_point_t dFyf_ddelta = C_Sf_Fzf;

    fixed_point_t dFyr_dvx    = fp_mul(C_Sr_Fzr, dar_dvx);
    fixed_point_t dFyr_dvy    = fp_mul(C_Sr_Fzr, dar_dvy);
    fixed_point_t dFyr_domega = fp_mul(C_Sr_Fzr, dar_domega);

    /*
     * Compute current lateral tire forces for B matrix delta column
     */
    fixed_point_t front_ratio = fp_div(front_num, vx_safe);
    fixed_point_t alpha_f = fp_sub(delta, fp_atan(front_ratio));
    fixed_point_t F_yf = fp_mul(fp_mul(C_Sf, alpha_f), F_zf);

    /*
     * Initialize A matrix as identity (7×7)
     * A_discrete = I + dt * A_continuous
     */
    for (int row = 0; row < 7; row++)
    {
        for (int col = 0; col < 7; col++)
        {
            state_matrix_A[row][col] = (row == col) ? FP_ONE : 0;
        }
    }

    /*
     * Continuous-time A matrix (∂f/∂state) — only non-zero entries
     *
     * Rows 0-5: same as 6-state model, PLUS:
     *   Row 3 gains:  A[3][3] += dFx/dvx / m
     *                 A[3][6]  = dFx/domega_w / m
     *
     * Row 6 (domega_w/dt = (T_motor/G - Fx*Rw) / Iw):
     *   A[6][3] = -Rw * dFx/dvx / Iw
     *   A[6][6] = -Rw * dFx/domega_w / Iw
     */

    /* Row 0: position X derivatives */
    state_matrix_A[0][2] = fp_add(state_matrix_A[0][2], fp_mul(time_step,
        fp_sub(fp_neg(fp_mul(vx, sin_psi)), fp_mul(vy, cos_psi))));
    state_matrix_A[0][3] = fp_add(state_matrix_A[0][3], fp_mul(time_step, cos_psi));
    state_matrix_A[0][4] = fp_add(state_matrix_A[0][4], fp_mul(time_step, fp_neg(sin_psi)));

    /* Row 1: position Y derivatives */
    state_matrix_A[1][2] = fp_add(state_matrix_A[1][2], fp_mul(time_step,
        fp_sub(fp_mul(vx, cos_psi), fp_mul(vy, sin_psi))));
    state_matrix_A[1][3] = fp_add(state_matrix_A[1][3], fp_mul(time_step, sin_psi));
    state_matrix_A[1][4] = fp_add(state_matrix_A[1][4], fp_mul(time_step, cos_psi));

    /* Row 2: heading derivative */
    state_matrix_A[2][5] = fp_add(state_matrix_A[2][5], fp_mul(time_step, FP_ONE));

    /* Row 3: longitudinal velocity derivatives
     * dvx/dt = (Fx - Fyf*sin(d) + m*vy*w) / m
     * Now Fx depends on vx and omega_w through slip ratio.
     */
    fixed_point_t inv_m = fp_div(FP_ONE, mass);

    /* A[3][3]: dFx/dvx/m + (-dFyf_dvx * sin(d)) / m */
    state_matrix_A[3][3] = fp_add(state_matrix_A[3][3], fp_mul(time_step,
        fp_add(fp_mul(dFx_dvx, inv_m),
               fp_mul(fp_neg(fp_mul(dFyf_dvx, sin_delta)), inv_m))));

    state_matrix_A[3][4] = fp_add(state_matrix_A[3][4], fp_mul(time_step,
        fp_add(fp_mul(fp_neg(fp_mul(dFyf_dvy, sin_delta)), inv_m), omega)));

    state_matrix_A[3][5] = fp_add(state_matrix_A[3][5], fp_mul(time_step,
        fp_add(fp_mul(fp_neg(fp_mul(dFyf_domega, sin_delta)), inv_m), vy)));

    /* A[3][6]: dFx/domega_w / m  (NEW: Fx depends on wheel speed) */
    state_matrix_A[3][6] = fp_mul(time_step, fp_mul(dFx_domega_w, inv_m));

    /* Row 4: lateral velocity derivatives (unchanged — Fx doesn't enter directly) */
    state_matrix_A[4][3] = fp_add(state_matrix_A[4][3], fp_mul(time_step,
        fp_mul(fp_add(fp_mul(dFyf_dvx, cos_delta),
                       fp_sub(dFyr_dvx, fp_mul(mass, omega))), inv_m)));

    state_matrix_A[4][4] = fp_add(state_matrix_A[4][4], fp_mul(time_step,
        fp_mul(fp_add(fp_mul(dFyf_dvy, cos_delta), dFyr_dvy), inv_m)));

    state_matrix_A[4][5] = fp_add(state_matrix_A[4][5], fp_mul(time_step,
        fp_mul(fp_sub(fp_add(fp_mul(dFyf_domega, cos_delta), dFyr_domega),
                       fp_mul(mass, vx)), inv_m)));

    /* Row 5: yaw rate derivatives */
    fixed_point_t inv_Iz = fp_div(FP_ONE, Iz);

    state_matrix_A[5][3] = fp_add(state_matrix_A[5][3], fp_mul(time_step,
        fp_mul(fp_sub(fp_mul(lf, fp_mul(dFyf_dvx, cos_delta)),
                       fp_mul(lr, dFyr_dvx)), inv_Iz)));

    state_matrix_A[5][4] = fp_add(state_matrix_A[5][4], fp_mul(time_step,
        fp_mul(fp_sub(fp_mul(lf, fp_mul(dFyf_dvy, cos_delta)),
                       fp_mul(lr, dFyr_dvy)), inv_Iz)));

    state_matrix_A[5][5] = fp_add(state_matrix_A[5][5], fp_mul(time_step,
        fp_mul(fp_sub(fp_mul(lf, fp_mul(dFyf_domega, cos_delta)),
                       fp_mul(lr, dFyr_domega)), inv_Iz)));

    /* Row 6: wheel dynamics
     * domega_w/dt = (T_motor/G - Fx*Rw) / Iw
     *
     * dFx/dvx and dFx/domega_w propagate through:
     *   A[6][3] = -Rw * dFx/dvx / Iw
     *   A[6][6] = -Rw * dFx/domega_w / Iw
     */
    fixed_point_t inv_Iw = fp_div(FP_ONE, Iw);

    state_matrix_A[6][3] = fp_mul(time_step,
        fp_mul(fp_neg(fp_mul(Rw, dFx_dvx)), inv_Iw));

    state_matrix_A[6][6] = fp_add(state_matrix_A[6][6], fp_mul(time_step,
        fp_mul(fp_neg(fp_mul(Rw, dFx_domega_w)), inv_Iw)));

    /*
     * Initialize B matrix as zeros (7×2) and add continuous terms × dt
     *
     * Column 0: control = delta (steering angle)
     *   B[3][0] = (-dFyf_ddelta * sin(d) - F_yf * cos(d)) / m
     *   B[4][0] = (dFyf_ddelta * cos(d) - F_yf * sin(d)) / m
     *   B[5][0] = lf * (dFyf_ddelta * cos(d) - F_yf * sin(d)) / Iz
     *
     * Column 1: control = T_motor (motor torque)
     *   T_motor enters ONLY the wheel dynamics equation:
     *   B[6][1] = 1 / (Iw * G_ratio)
     *
     *   The effect on body states (vx, vy, omega) propagates
     *   through the A matrix coupling: T → ω_w → κ → Fx → vx.
     */
    for (int row = 0; row < 7; row++)
    {
        for (int col = 0; col < 2; col++)
        {
            input_matrix_B[row][col] = 0;
        }
    }

    /* B[3][0]: d(dvx/dt)/d(delta) × dt */
    fixed_point_t dFyf_dd_sin = fp_mul(dFyf_ddelta, sin_delta);
    fixed_point_t Fyf_cos     = fp_mul(F_yf, cos_delta);
    input_matrix_B[3][0] = fp_mul(time_step,
        fp_mul(fp_sub(fp_neg(dFyf_dd_sin), Fyf_cos), inv_m));

    /* B[4][0]: d(dvy/dt)/d(delta) × dt */
    fixed_point_t dFyf_dd_cos = fp_mul(dFyf_ddelta, cos_delta);
    fixed_point_t Fyf_sin     = fp_mul(F_yf, sin_delta);
    input_matrix_B[4][0] = fp_mul(time_step,
        fp_mul(fp_sub(dFyf_dd_cos, Fyf_sin), inv_m));

    /* B[5][0]: d(domega/dt)/d(delta) × dt */
    input_matrix_B[5][0] = fp_mul(time_step,
        fp_mul(fp_mul(lf, fp_sub(dFyf_dd_cos, Fyf_sin)), inv_Iz));

    /* B[6][1]: d(domega_w/dt)/d(T_motor) × dt = G_ratio × dt / Iw
     * Gearbox amplifies motor torque by G at the wheel. */
    input_matrix_B[6][1] = fp_mul(time_step, fp_mul(G_ratio, inv_Iw));

    /* B[3][1]: Approximate direct torque→velocity coupling.
     * Physically, T_motor affects vx through: T → ωw (B[6][1]) → κ → Fx → vx (A[3][6]).
     * This 2-step propagation is captured by A^k·B in the condensed QP, but the
     * gradient is weak and causes poor QP conditioning / chattering.
     *
     * Adding B[3][1] ≈ A[3][6] × B[6][1] gives the QP a direct gradient from
     * torque to velocity, dramatically improving convergence.
     * Equivalent to: dt² × (dFx/dωw) / (m × Iw × G)
     */
    input_matrix_B[3][1] = fp_mul(state_matrix_A[3][6], input_matrix_B[6][1]);
}
