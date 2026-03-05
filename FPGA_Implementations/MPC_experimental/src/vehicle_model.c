/**
 * @file vehicle_model.c
 * @brief Dynamic Nonlinear Bicycle Model Implementation
 *
 * Implements the dynamic bicycle model with direct acceleration input
 * for F1/10th vehicle dynamics. The MPC commands an acceleration that
 * the VESC motor controller enforces, so F_x = m * a_cmd directly.
 *
 * The linearization uses a Pacejka-like tire model for nonlinear
 * tire force saturation (lateral), while the forward prediction
 * uses a linear lateral tire model.
 * All calculations use fixed-point arithmetic for FPGA compatibility.
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
 */

#include "vehicle_model.h"
#include "fp_math.h"
#ifndef MPC_HLS_TARGET
#include <stdio.h>
#endif
#include <string.h>

/* When MPC_SIM_MATCHING is defined, the vehicle model uses simplified
 * equations that exactly match the f1tenth_gym simulator:
 *   - No cos(δ)/sin(δ) in force resolution (gym doesn't have them)
 *   - Linear tire model (no Pacejka saturation)
 *   - Small-angle slip angles (no atan)
 * This improves sim prediction accuracy at the cost of real-world fidelity.
 * Undefine for real-car deployment.  */
#define MPC_SIM_MATCHING

/*===========================================================================
 * Module State (Vehicle Parameters)
 *===========================================================================*/

/** Current vehicle parameters (initialized by vehicle_model_initialize) */
static VehicleParameters_t stored_vehicle_parameters;

/** Flag indicating if model has been initialized */
static uint8_t model_is_initialized = 0;

/** Cached reciprocals of constant parameters (avoid fp_div every linearization call).
 *  Precomputed during initialization — saves 3 divisions per linearization. */
static fixed_point_t cached_inv_mass = 0;   /* 1 / vehicle_mass_kg */
static fixed_point_t cached_inv_Iz   = 0;   /* 1 / yaw_moment_of_inertia */
static fixed_point_t cached_inv_Rw   = 0;   /* 1 / wheel_radius */
static fixed_point_t cached_inv_L_wb = 0;   /* 1 / wheelbase */

/*===========================================================================
 * Initialization Functions
 *===========================================================================*/

/** Recompute cached reciprocals after any parameter change. */
static void recompute_cached_reciprocals(void)
{
    cached_inv_mass = fp_recip(stored_vehicle_parameters.vehicle_mass_kg);
    cached_inv_Iz   = fp_recip(stored_vehicle_parameters.yaw_moment_of_inertia_kgm2);
    cached_inv_Rw   = fp_recip(stored_vehicle_parameters.wheel_radius_meters);
    cached_inv_L_wb = fp_recip(stored_vehicle_parameters.wheelbase_meters);
}

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

    stored_vehicle_parameters.maximum_acceleration_meters_per_second_squared =
        F110_DEFAULT_MAX_ACCELERATION;

    stored_vehicle_parameters.minimum_acceleration_meters_per_second_squared =
        F110_DEFAULT_MIN_ACCELERATION;

    stored_vehicle_parameters.omega = 
        F110_DEFAULT_YAW_RATE;

    stored_vehicle_parameters.height_cg_to_ground_meters =
        F110_CG_HEIGHT_METERS;

    stored_vehicle_parameters.gravity_acceleration_meters_per_second_squared =
        F110_GRAVITY_ACCELERATION_MS2;

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
#ifdef MPC_HLS_TARGET
#pragma HLS INLINE
#endif
    ControlInput_t saturated_control;

    /* Clamp steering angle to physical limits */
    saturated_control.steering_angle_radians = fp_clamp(
        raw_control->steering_angle_radians,
        fp_neg(stored_vehicle_parameters.maximum_steering_angle_radians),
        stored_vehicle_parameters.maximum_steering_angle_radians);

    /* Clamp acceleration to [min, max] */
    saturated_control.acceleration_meters_per_second_squared = fp_clamp(
        raw_control->acceleration_meters_per_second_squared,
        stored_vehicle_parameters.minimum_acceleration_meters_per_second_squared,
        stored_vehicle_parameters.maximum_acceleration_meters_per_second_squared);

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

    /*
     * Extract control inputs
     */
    fixed_point_t delta   = saturated_control.steering_angle_radians;
    fixed_point_t a_cmd  = saturated_control.acceleration_meters_per_second_squared;

    /*
     * Extract vehicle parameters
     */
    fixed_point_t lf   = stored_vehicle_parameters.distance_cg_to_front_axle_meters;
    fixed_point_t lr   = stored_vehicle_parameters.distance_cg_to_rear_axle_meters;
    fixed_point_t mass = stored_vehicle_parameters.vehicle_mass_kg;
    fixed_point_t Iz   = stored_vehicle_parameters.yaw_moment_of_inertia_kgm2;
    fixed_point_t C_Sf = stored_vehicle_parameters.front_cornering_stiffness;
    fixed_point_t C_Sr = stored_vehicle_parameters.rear_cornering_stiffness;

    /* Wheel parameters */
    fixed_point_t Rw      = stored_vehicle_parameters.wheel_radius_meters;

    /*
     * Compute trigonometric values
     */
    fixed_point_t cos_psi   = fp_cos(psi);
    fixed_point_t sin_psi   = fp_sin(psi);
    fixed_point_t cos_delta = fp_cos(delta);
    fixed_point_t sin_delta = fp_sin(delta);

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
    fixed_point_t Fx = fp_mul(mass, a_cmd);

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
    fixed_point_t rear_numerator = fp_sub(vy, fp_mul(lr, omega));
#ifdef MPC_SIM_MATCHING
    /* Sim-matching: small-angle slip (gym uses α ≈ linear, no atan) */
    fixed_point_t front_ratio = fp_div(front_numerator, vx_safe);
    fixed_point_t alpha_f = fp_sub(delta, front_ratio);
    fixed_point_t rear_ratio = fp_div(rear_numerator, vx_safe);
    fixed_point_t alpha_r = fp_neg(rear_ratio);
#else
    fixed_point_t front_ratio = fp_div(front_numerator, vx_safe);
    fixed_point_t alpha_f = fp_sub(delta, fp_atan(front_ratio));
    fixed_point_t rear_ratio = fp_div(rear_numerator, vx_safe);
    fixed_point_t alpha_r = fp_neg(fp_atan(rear_ratio));
#endif

    /*
     * Compute normal forces (load transfer under acceleration)
     *
     * Static weight distribution plus longitudinal load transfer:
     *   F_zf = (m * g * l_r - F_x * h) / (l_f + l_r)
     *   F_zr = (m * g * l_f + F_x * h) / (l_f + l_r)
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
    const fixed_point_t mu = F110_FRICTION_COEFFICIENT;
    fixed_point_t F_yf = fp_mul(mu, fp_mul(fp_mul(C_Sf, alpha_f), F_zf));
    fixed_point_t F_yr = fp_mul(mu, fp_mul(fp_mul(C_Sr, alpha_r), F_zr));

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

#ifdef MPC_SIM_MATCHING
    /* Sim-matching: no cos(δ)/sin(δ) in force resolution (matches gym) */
    /* dv_x/dt = a_cmd + v_y * omega  (no -F_yf*sin(δ)/m term) */
    fixed_point_t dvx_dt = fp_add(a_cmd, fp_mul(vy, omega));

    /* dv_y/dt = (F_yf + F_yr) / m - v_x * omega  (no cos(δ) on F_yf) */
    fixed_point_t dvy_dt = fp_sub(
        fp_mul(fp_add(F_yf, F_yr), cached_inv_mass),
        fp_mul(vx, omega));

    /* domega/dt = (l_f * F_yf - l_r * F_yr) / I_z  (no cos(δ)) */
    fixed_point_t domega_dt = fp_mul(
        fp_sub(fp_mul(lf, F_yf), fp_mul(lr, F_yr)),
        cached_inv_Iz);
#else
    /* Full model: cos(δ)/sin(δ) force resolution for real-world accuracy */
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
#endif

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

    /*
     * Apply state constraints
     */

    /* Clamp longitudinal velocity to [min, max] */
    next_state.longitudinal_velocity_meters_per_second = fp_clamp(
        next_state.longitudinal_velocity_meters_per_second,
        stored_vehicle_parameters.minimum_velocity_meters_per_second,
        stored_vehicle_parameters.maximum_velocity_meters_per_second);

    /* Normalize heading angle to [-pi, +pi] */
    for (int _i = 0; _i < 10 && next_state.heading_angle_radians > FP_PI; _i++)
    {
        next_state.heading_angle_radians = fp_sub(
            next_state.heading_angle_radians,
            FP_TWO_PI);
    }
    for (int _i = 0; _i < 10 && next_state.heading_angle_radians < -FP_PI; _i++)
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
    fixed_point_t state_matrix_A[6][6],
    fixed_point_t input_matrix_B[6][2])
{
#ifdef MPC_HLS_TARGET
#pragma HLS INLINE
#endif
    /*
     * Extract operating point variables
     */
    fixed_point_t psi     = operating_state->heading_angle_radians;
    fixed_point_t vx      = operating_state->longitudinal_velocity_meters_per_second;
    fixed_point_t vy      = operating_state->lateral_velocity_meters_per_second;
    fixed_point_t omega   = operating_state->yaw_rate_radians_per_second;
    fixed_point_t delta   = operating_control->steering_angle_radians;

    /*
     * Extract vehicle parameters
     */
    fixed_point_t lf   = stored_vehicle_parameters.distance_cg_to_front_axle_meters;
    fixed_point_t lr   = stored_vehicle_parameters.distance_cg_to_rear_axle_meters;
    fixed_point_t mass = stored_vehicle_parameters.vehicle_mass_kg;
    fixed_point_t C_Sf = stored_vehicle_parameters.front_cornering_stiffness;
    fixed_point_t C_Sr = stored_vehicle_parameters.rear_cornering_stiffness;

    /*
     * Compute trigonometric values at operating point.
     * Short-circuit at zero: MPC always linearizes at δ=0, ψ=0 (Frenet),
     * saving 4 Taylor-series evaluations (~40 multiplies).
     */
    fixed_point_t cos_psi, sin_psi;
    if (psi == 0) { cos_psi = FP_ONE; sin_psi = 0; }
    else { cos_psi = fp_cos(psi); sin_psi = fp_sin(psi); }

    fixed_point_t cos_delta, sin_delta;
    if (delta == 0) { cos_delta = FP_ONE; sin_delta = 0; }
    else { cos_delta = fp_cos(delta); sin_delta = fp_sin(delta); }

    /*
     * Minimum velocity floor for linearization stability
     */
    fixed_point_t min_vx = FP_CONST(0.5);
    fixed_point_t vx_safe = (vx > min_vx) ? vx : min_vx;
    /* Precompute reciprocal: replaces ~10 fp_div by vx_safe with fp_mul */
    fixed_point_t inv_vx_safe = fp_recip(vx_safe);

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
    fixed_point_t Fx = fp_mul(mass, operating_control->acceleration_meters_per_second_squared);
    fixed_point_t dFx_dvx = 0;

    /*
     * Compute slip angle intermediates for Jacobian
     */
    fixed_point_t front_num = fp_add(vy, fp_mul(lf, omega));
    fixed_point_t rear_num  = fp_sub(vy, fp_mul(lr, omega));

#ifdef MPC_SIM_MATCHING
    /* Sim-matching: small-angle slip angle Jacobians
     * α_f = δ - (vy + lf·ω)/vx,  α_r = -(vy - lr·ω)/vx
     * dα_f/dvx = (vy + lf·ω)/vx², dα_f/dvy = -1/vx, dα_f/dω = -lf/vx
     * dα_r/dvx = (vy - lr·ω)/vx², dα_r/dvy = -1/vx, dα_r/dω = lr/vx  */
    fixed_point_t inv_vx2 = fp_mul(inv_vx_safe, inv_vx_safe);

    fixed_point_t daf_dvx    = fp_mul(front_num, inv_vx2);
    fixed_point_t daf_dvy    = fp_neg(inv_vx_safe);
    fixed_point_t daf_domega = fp_neg(fp_mul(lf, inv_vx_safe));

    fixed_point_t dar_dvx    = fp_mul(rear_num, inv_vx2);
    fixed_point_t dar_dvy    = fp_neg(inv_vx_safe);
    fixed_point_t dar_domega = fp_mul(lr, inv_vx_safe);
#else
    /* Full atan model: d(atan(n/d))/dx = (d·dn/dx - n·dd/dx) / (d² + n²) */
    fixed_point_t front_num2 = fp_mul(front_num, front_num);
    fixed_point_t rear_num2  = fp_mul(rear_num, rear_num);

    fixed_point_t vx2 = fp_mul(vx_safe, vx_safe);
    fixed_point_t D_f = fp_add(vx2, front_num2);
    fixed_point_t D_r = fp_add(vx2, rear_num2);

    if (D_f == 0) D_f = FP_ONE;
    if (D_r == 0) D_r = FP_ONE;

    /* Precompute reciprocals: 2 fp_recip replaces 6 fp_div */
    fixed_point_t inv_D_f = fp_recip(D_f);
    fixed_point_t inv_D_r = fp_recip(D_r);

    fixed_point_t daf_dvx    = fp_mul(front_num, inv_D_f);
    fixed_point_t daf_dvy    = fp_neg(fp_mul(vx_safe, inv_D_f));
    fixed_point_t daf_domega = fp_neg(fp_mul(fp_mul(lf, vx_safe), inv_D_f));

    fixed_point_t dar_dvx    = fp_mul(rear_num, inv_D_r);
    fixed_point_t dar_dvy    = fp_neg(fp_mul(vx_safe, inv_D_r));
    fixed_point_t dar_domega = fp_mul(fp_mul(lr, vx_safe), inv_D_r);
#endif

    /*
     * Compute normal forces at operating point
     *
     *   F_zf = (m*g*l_r - F_x*h) / L
     *   F_zr = (m*g*l_f + F_x*h) / L
     */
    fixed_point_t g_acc = stored_vehicle_parameters.gravity_acceleration_meters_per_second_squared;
    fixed_point_t h_cg  = stored_vehicle_parameters.height_cg_to_ground_meters;
    fixed_point_t mg    = fp_mul(mass, g_acc);

    fixed_point_t F_zf = fp_mul(
        fp_sub(fp_mul(mg, lr), fp_mul(Fx, h_cg)), cached_inv_L_wb);
    fixed_point_t F_zr = fp_mul(
        fp_add(fp_mul(mg, lf), fp_mul(Fx, h_cg)), cached_inv_L_wb);

    /*
     * Lateral tire force model for linearization.
     *
     * MPC_SIM_MATCHING: Pure linear model (matches gym exactly)
     *   C_eff = mu * C_S * F_z  (constant stiffness)
     *
     * Full model: Pacejka-like saturation curve
     *   F_y = D · sin(C · atan(B · α))
     *   C_eff = dF_y/dα at operating-point slip angle
     */

    /* Compute front and rear slip angles at operating point */
    fixed_point_t front_ratio_lin = fp_mul(front_num, inv_vx_safe);
    fixed_point_t rear_ratio_lin  = fp_mul(rear_num, inv_vx_safe);

    const fixed_point_t mu = F110_FRICTION_COEFFICIENT;

    /* Linear stiffness (slope at α=0): mu * C_Sf * F_zf */
    fixed_point_t C_Sf_Fzf_linear = fp_mul(mu, fp_mul(C_Sf, F_zf));
    fixed_point_t C_Sr_Fzr_linear = fp_mul(mu, fp_mul(C_Sr, F_zr));

#ifdef MPC_SIM_MATCHING
    /* Sim-matching: pure linear tire model (no Pacejka saturation)
     * This exactly matches the gym's F_y = mu * C_S * alpha * F_z */
    fixed_point_t C_Sf_Fzf = C_Sf_Fzf_linear;
    fixed_point_t C_Sr_Fzr = C_Sr_Fzr_linear;

    /* For B-matrix: F_yf at operating point (needed for full model but not sim) */
    fixed_point_t alpha_f_op = fp_sub(delta, front_ratio_lin);
    fixed_point_t F_yf = fp_mul(C_Sf_Fzf, alpha_f_op);
    (void)rear_ratio_lin; /* Not needed for sim-matching F_yr in B matrix */
#else
    /* Full Pacejka-like model for real-world accuracy */
    fixed_point_t alpha_f_op = fp_sub(delta, fp_atan(front_ratio_lin));
    fixed_point_t alpha_r_op = fp_neg(fp_atan(rear_ratio_lin));

    const fixed_point_t C_shape = FP_CONST(1.9);
    const fixed_point_t inv_C_shape = fp_recip(C_shape);
    const fixed_point_t min_stiffness_scale = FP_CONST(0.1);

    fixed_point_t B_f = fp_mul(C_Sf, inv_C_shape);
    fixed_point_t B_r = fp_mul(C_Sr, inv_C_shape);

    fixed_point_t C_Sf_Fzf;
    fixed_point_t F_yf;
    {
        fixed_point_t D_pac_f = fp_mul(mu, F_zf);
        fixed_point_t Ba_f = fp_mul(B_f, alpha_f_op);
        fixed_point_t inner_f = fp_mul(C_shape, fp_atan(Ba_f));
        fixed_point_t cos_inner_f = fp_cos(inner_f);
        fixed_point_t denom_f_pac = fp_add(FP_ONE, fp_mul(Ba_f, Ba_f));
        fixed_point_t inv_denom_f_pac = fp_recip(denom_f_pac);
        fixed_point_t C_eff_f = fp_mul(fp_mul(fp_mul(D_pac_f, C_shape), B_f),
                                       fp_mul(cos_inner_f, inv_denom_f_pac));
        fixed_point_t C_min_f = fp_mul(C_Sf_Fzf_linear, min_stiffness_scale);
        C_Sf_Fzf = (C_eff_f > C_min_f) ? C_eff_f : C_min_f;
        F_yf = fp_mul(D_pac_f, fp_sin(inner_f));
    }

    fixed_point_t C_Sr_Fzr;
    {
        fixed_point_t D_pac_r = fp_mul(mu, F_zr);
        fixed_point_t Ba_r = fp_mul(B_r, alpha_r_op);
        fixed_point_t inner_r = fp_mul(C_shape, fp_atan(Ba_r));
        fixed_point_t cos_inner_r = fp_cos(inner_r);
        fixed_point_t denom_r_pac = fp_add(FP_ONE, fp_mul(Ba_r, Ba_r));
        fixed_point_t inv_denom_r_pac = fp_recip(denom_r_pac);
        fixed_point_t C_eff_r = fp_mul(fp_mul(fp_mul(D_pac_r, C_shape), B_r),
                                       fp_mul(cos_inner_r, inv_denom_r_pac));
        fixed_point_t C_min_r = fp_mul(C_Sr_Fzr_linear, min_stiffness_scale);
        C_Sr_Fzr = (C_eff_r > C_min_r) ? C_eff_r : C_min_r;
    }
#endif

#ifdef MPC_DEBUG_PRINT
    printf("[MPC-DBG] C_Sf_Fzf=%.3f C_Sr_Fzr=%.3f (linear: %.3f, %.3f) F_zf=%.3f F_zr=%.3f alpha_f=%.4f\n",
           FP_TO_DOUBLE(C_Sf_Fzf), FP_TO_DOUBLE(C_Sr_Fzr),
           FP_TO_DOUBLE(C_Sf_Fzf_linear), FP_TO_DOUBLE(C_Sr_Fzr_linear),
           FP_TO_DOUBLE(F_zf), FP_TO_DOUBLE(F_zr),
           FP_TO_DOUBLE(alpha_f));
#endif

    fixed_point_t dFyf_dvx    = fp_mul(C_Sf_Fzf, daf_dvx);
    fixed_point_t dFyf_dvy    = fp_mul(C_Sf_Fzf, daf_dvy);
    fixed_point_t dFyf_domega = fp_mul(C_Sf_Fzf, daf_domega);
    fixed_point_t dFyf_ddelta = C_Sf_Fzf;

    fixed_point_t dFyr_dvx    = fp_mul(C_Sr_Fzr, dar_dvx);
    fixed_point_t dFyr_dvy    = fp_mul(C_Sr_Fzr, dar_dvy);
    fixed_point_t dFyr_domega = fp_mul(C_Sr_Fzr, dar_domega);

    /*
     * Initialize A matrix as identity (6×6)
     * A_discrete = I + dt * A_continuous
     */
    for (int row = 0; row < 6; row++)
    {
        for (int col = 0; col < 6; col++)
        {
            state_matrix_A[row][col] = (row == col) ? FP_ONE : 0;
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
     * Now Fx = m * a_cmd (direct acceleration input).
     */
    fixed_point_t inv_m = cached_inv_mass;

#ifdef MPC_SIM_MATCHING
    /* Sim-matching: no sin(δ)/cos(δ) force resolution.
     * dvx/dt = a_cmd + vy·ω  → d/dvx = 0, d/dvy = ω, d/dω = vy
     * dvy/dt = (F_yf + F_yr)/m - vx·ω
     * dω/dt  = (lf·F_yf - lr·F_yr)/Iz
     * No cos/sin on F_yf — matches gym's equations exactly. */

    /* Row 3: dvx/dt = a_cmd + vy*omega (no tire coupling) */
    /* A[3][3] += dt * 0 = 0  (Fx = m*a_cmd, dFx/dvx = 0) */
    state_matrix_A[3][4] = fp_add(state_matrix_A[3][4], fp_mul(time_step, omega));
    state_matrix_A[3][5] = fp_add(state_matrix_A[3][5], fp_mul(time_step, vy));

    /* Row 4: dvy/dt = (F_yf + F_yr)/m - vx*omega */
    state_matrix_A[4][3] = fp_add(state_matrix_A[4][3], fp_mul(time_step,
        fp_mul(fp_add(dFyf_dvx, fp_sub(dFyr_dvx, fp_mul(mass, omega))), inv_m)));
    state_matrix_A[4][4] = fp_add(state_matrix_A[4][4], fp_mul(time_step,
        fp_mul(fp_add(dFyf_dvy, dFyr_dvy), inv_m)));
    state_matrix_A[4][5] = fp_add(state_matrix_A[4][5], fp_mul(time_step,
        fp_mul(fp_sub(fp_add(dFyf_domega, dFyr_domega),
                       fp_mul(mass, vx)), inv_m)));

    /* Row 5: dω/dt = (lf*F_yf - lr*F_yr)/Iz */
    fixed_point_t inv_Iz = cached_inv_Iz;

    state_matrix_A[5][3] = fp_add(state_matrix_A[5][3], fp_mul(time_step,
        fp_mul(fp_sub(fp_mul(lf, dFyf_dvx), fp_mul(lr, dFyr_dvx)), inv_Iz)));
    state_matrix_A[5][4] = fp_add(state_matrix_A[5][4], fp_mul(time_step,
        fp_mul(fp_sub(fp_mul(lf, dFyf_dvy), fp_mul(lr, dFyr_dvy)), inv_Iz)));
    state_matrix_A[5][5] = fp_add(state_matrix_A[5][5], fp_mul(time_step,
        fp_mul(fp_sub(fp_mul(lf, dFyf_domega), fp_mul(lr, dFyr_domega)), inv_Iz)));
#else
    /* Full model with cos(δ)/sin(δ) force resolution */
    /* A[3][3]: dFx/dvx/m + (-dFyf_dvx * sin(d)) / m */
    state_matrix_A[3][3] = fp_add(state_matrix_A[3][3], fp_mul(time_step,
        fp_add(fp_mul(dFx_dvx, inv_m),
               fp_mul(fp_neg(fp_mul(dFyf_dvx, sin_delta)), inv_m))));

    state_matrix_A[3][4] = fp_add(state_matrix_A[3][4], fp_mul(time_step,
        fp_add(fp_mul(fp_neg(fp_mul(dFyf_dvy, sin_delta)), inv_m), omega)));

    state_matrix_A[3][5] = fp_add(state_matrix_A[3][5], fp_mul(time_step,
        fp_add(fp_mul(fp_neg(fp_mul(dFyf_domega, sin_delta)), inv_m), vy)));

    state_matrix_A[4][3] = fp_add(state_matrix_A[4][3], fp_mul(time_step,
        fp_mul(fp_add(fp_mul(dFyf_dvx, cos_delta),
                       fp_sub(dFyr_dvx, fp_mul(mass, omega))), inv_m)));

    state_matrix_A[4][4] = fp_add(state_matrix_A[4][4], fp_mul(time_step,
        fp_mul(fp_add(fp_mul(dFyf_dvy, cos_delta), dFyr_dvy), inv_m)));

    state_matrix_A[4][5] = fp_add(state_matrix_A[4][5], fp_mul(time_step,
        fp_mul(fp_sub(fp_add(fp_mul(dFyf_domega, cos_delta), dFyr_domega),
                       fp_mul(mass, vx)), inv_m)));

    /* Row 5: yaw rate derivatives */
    fixed_point_t inv_Iz = cached_inv_Iz;

    state_matrix_A[5][3] = fp_add(state_matrix_A[5][3], fp_mul(time_step,
        fp_mul(fp_sub(fp_mul(lf, fp_mul(dFyf_dvx, cos_delta)),
                       fp_mul(lr, dFyr_dvx)), inv_Iz)));

    state_matrix_A[5][4] = fp_add(state_matrix_A[5][4], fp_mul(time_step,
        fp_mul(fp_sub(fp_mul(lf, fp_mul(dFyf_dvy, cos_delta)),
                       fp_mul(lr, dFyr_dvy)), inv_Iz)));

    state_matrix_A[5][5] = fp_add(state_matrix_A[5][5], fp_mul(time_step,
        fp_mul(fp_sub(fp_mul(lf, fp_mul(dFyf_domega, cos_delta)),
                       fp_mul(lr, dFyr_domega)), inv_Iz)));
#endif

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

#ifdef MPC_SIM_MATCHING
    /* Sim-matching: no cos(δ)/sin(δ) on steering column.
     * dvy/d(delta)  = dFyf_ddelta / m
     * dω/d(delta)   = lf * dFyf_ddelta / Iz
     * dvx/d(delta)  = 0  (no sin(δ) term in gym) */
    /* B[3][0] = 0 (no longitudinal coupling from steering in gym model) */
    input_matrix_B[4][0] = fp_mul(time_step, fp_mul(dFyf_ddelta, inv_m));
    input_matrix_B[5][0] = fp_mul(time_step, fp_mul(fp_mul(lf, dFyf_ddelta), inv_Iz));
#else
    /* Full model: B with cos(δ)/sin(δ) force resolution */
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
#endif

    /* B[3][1]: d(dvx/dt)/d(a_cmd) × dt = dt */
    input_matrix_B[3][1] = time_step;
}

/*===========================================================================
 * Frenet Frame Linearization
 *===========================================================================
 *
 * Computes the 6×6 discrete state-space matrices for the Frenet frame.
 *
 * Frenet state: [e_y, e_psi, v_x, v_y, omega]
 *
 * Rows 0-1: Frenet kinematic relations (path-relative)
 *   e_y_dot   ≈ v_x * e_psi + v_y           (linearized at e_psi=0)
 *   e_psi_dot ≈ omega - kappa * v_x          (linearized at e_y=0, e_psi=0)
 *
 * Rows 2-4: Body-frame dynamics (identical to global model rows 3-5)
 *   v_x_dot, v_y_dot, omega_dot
 *
 * Strategy: Call the global linearization and extract body-frame rows,
 * then build the Frenet kinematic rows using path curvature.
 */
void vehicle_model_compute_frenet_linearization(
    const FrenetState_t *frenet_state,
    const ControlInput_t *operating_control,
    fixed_point_t time_step,
    fixed_point_t path_curvature,
    fixed_point_t state_matrix_A[FRENET_STATE_DIMENSION][FRENET_STATE_DIMENSION],
    fixed_point_t input_matrix_B[FRENET_STATE_DIMENSION][2])
{
#ifdef MPC_HLS_TARGET
#pragma HLS INLINE
#endif
    /*
     * Step 1: Create a global VehicleState_t for the existing linearization.
     * Global position and heading don't affect the body-frame Jacobians,
     * so we set them to zero. Only body-frame states matter.
     */
    VehicleState_t global_state;
    global_state.position_x_meters = 0;
    global_state.position_y_meters = 0;
    global_state.heading_angle_radians = 0;
    global_state.longitudinal_velocity_meters_per_second =
        frenet_state->longitudinal_velocity_meters_per_second;
    global_state.lateral_velocity_meters_per_second =
        frenet_state->lateral_velocity_meters_per_second;
    global_state.yaw_rate_radians_per_second =
        frenet_state->yaw_rate_radians_per_second;

    /*
     * Step 2: Get the full 6×6 global linearization
     */
    fixed_point_t A_global[6][6];
    fixed_point_t B_global[6][2];
    vehicle_model_compute_linearization(
        &global_state, operating_control, time_step,
        A_global, B_global);

    /*
     * Step 3: Initialize Frenet matrices to zero
     * Explicit loops instead of memset for HLS-friendly unrolling.
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
     * Step 4: Copy body-frame dynamics (global rows 3-5, cols 3-5)
     * into Frenet rows 2-4, cols 2-4.
     *
     * These rows represent v_x, v_y, omega dynamics which
     * are identical in global and Frenet frames (body-frame forces
     * don't depend on global position or path-relative position).
     *
     * Body dynamics cols 0-2 (dependency on e_y, e_psi) are zero
     * because body-frame forces don't depend on Frenet position.
     */
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            state_matrix_A[i + 2][j + 2] = A_global[i + 3][j + 3];
        }
        input_matrix_B[i + 2][0] = B_global[i + 3][0];
        input_matrix_B[i + 2][1] = B_global[i + 3][1];
    }

    /*
     * Step 5: Build Frenet kinematic rows (discrete-time Forward Euler)
     *
     * Row 0: e_y dynamics
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
    fixed_point_t v_x = frenet_state->longitudinal_velocity_meters_per_second;

    state_matrix_A[0][0] = FP_ONE;
    state_matrix_A[0][1] = fp_mul(time_step, v_x);
    /* A[0][2..5] = 0, except: */
    state_matrix_A[0][3] = time_step;

    /*
     * Row 1: e_psi dynamics
     *   Continuous: e_psi_dot = omega - kappa * v_x * cos(e_psi) / (1 - kappa * e_y)
     *   Linearized (e_y ≈ 0, e_psi ≈ 0): e_psi_dot ≈ omega - kappa * v_x
     *   Discrete: e_psi[k+1] = e_psi[k] + dt * (omega[k] - kappa * v_x[k])
     *
     *   A[1][0] = 0                (∂/∂e_y ≈ kappa² * v_x ≈ 0 for small kappa)
     *   A[1][1] = 1                (identity)
     *   A[1][2] = -dt * kappa      (speed along curved path changes heading error)
     *   A[1][3] = 0                (no v_y coupling)
     *   A[1][4] = dt               (yaw rate directly changes heading error)
     */
    state_matrix_A[1][1] = FP_ONE;
    state_matrix_A[1][2] = fp_neg(fp_mul(time_step, path_curvature));
    state_matrix_A[1][4] = time_step;

    /*
     * B rows 0-1 are all zero:
     * Steering and torque don't directly change e_y or e_psi.
     * Their effect propagates through omega (row 4) and v_y (row 3),
     * which then affect e_y and e_psi through the A matrix coupling.
     */
    /* input_matrix_B[0][0..1] = 0 (already zeroed) */
    /* input_matrix_B[1][0..1] = 0 (already zeroed) */
}
