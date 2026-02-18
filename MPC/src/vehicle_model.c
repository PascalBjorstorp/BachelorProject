/**
 * @file vehicle_model.c
 * @brief Dynamic Nonlinear Bicycle Model Implementation
 *
 * Implements the dynamic bicycle model with linear tire forces for
 * F1/10th vehicle dynamics. All calculations use fixed-point arithmetic
 * for FPGA compatibility.
 *
 * State vector (6): [x, y, psi, v_x, v_y, omega]
 * Control vector (2): [delta, F_x]
 *
 * Model Equations:
 *   dx/dt      = v_x * cos(psi) - v_y * sin(psi)
 *   dy/dt      = v_x * sin(psi) + v_y * cos(psi)
 *   dpsi/dt    = omega
 *   dv_x/dt    = (F_x - F_yf * sin(delta) + m * v_y * omega) / m
 *   dv_y/dt    = (F_yf * cos(delta) + F_yr - m * v_x * omega) / m
 *   domega/dt  = (l_f * F_yf * cos(delta) - l_r * F_yr) / I_z
 *
 * Tire model (linear):
 *   alpha_f = delta - atan((v_y + l_f * omega) / v_x)
 *   alpha_r = -atan((v_y - l_r * omega) / v_x)
 *   F_yf = -C_Sf * alpha_f
 *   F_yr = -C_Sr * alpha_r
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

    stored_vehicle_parameters.maximum_longitudinal_force_newtons =
        F110_DEFAULT_MAX_LONGITUDINAL_FORCE_NEWTONS;

    stored_vehicle_parameters.minimum_longitudinal_force_newtons =
        F110_DEFAULT_MIN_LONGITUDINAL_FORCE_NEWTONS;

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

    /* Clamp longitudinal force to [min_force, max_force] */
    saturated_control.longitudinal_force_newtons = fp_clamp(
        raw_control->longitudinal_force_newtons,
        stored_vehicle_parameters.minimum_longitudinal_force_newtons,
        stored_vehicle_parameters.maximum_longitudinal_force_newtons);

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
    fixed_point_t delta = saturated_control.steering_angle_radians;
    fixed_point_t Fx    = saturated_control.longitudinal_force_newtons;

    /*
     * Extract vehicle parameters
     */
    fixed_point_t lf   = stored_vehicle_parameters.distance_cg_to_front_axle_meters;
    fixed_point_t lr   = stored_vehicle_parameters.distance_cg_to_rear_axle_meters;
    fixed_point_t mass = stored_vehicle_parameters.vehicle_mass_kg;
    fixed_point_t Iz   = stored_vehicle_parameters.yaw_moment_of_inertia_kgm2;
    fixed_point_t C_Sf = stored_vehicle_parameters.front_cornering_stiffness;
    fixed_point_t C_Sr = stored_vehicle_parameters.rear_cornering_stiffness;

    /*
     * Compute trigonometric values
     */
    fixed_point_t cos_psi   = fp_cos(psi);
    fixed_point_t sin_psi   = fp_sin(psi);
    fixed_point_t cos_delta = fp_cos(delta);
    fixed_point_t sin_delta = fp_sin(delta);

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
     * Compute lateral tire forces (linear tire model)
     *
     * With slip angle defined as alpha = delta - atan(vel_ratio), positive
     * alpha means the wheel is pointed more leftward than the velocity vector.
     * The tire generates a lateral force IN the direction of the slip:
     *
     *   F_yf = C_Sf * alpha_f   (positive alpha → positive lateral force)
     *   F_yr = C_Sr * alpha_r
     *
     * This matches the Pacejka linearization: F_y = B*C*D * alpha ≈ C_alpha * alpha
     */
    fixed_point_t F_yf = fp_mul(C_Sf, alpha_f);
    fixed_point_t F_yr = fp_mul(C_Sr, alpha_r);

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
    fixed_point_t state_matrix_A[6][6],
    fixed_point_t input_matrix_B[6][2])
{
    /*
     * Extract operating point variables
     */
    fixed_point_t psi   = operating_state->heading_angle_radians;
    fixed_point_t vx    = operating_state->longitudinal_velocity_meters_per_second;
    fixed_point_t vy    = operating_state->lateral_velocity_meters_per_second;
    fixed_point_t omega = operating_state->yaw_rate_radians_per_second;
    fixed_point_t delta = operating_control->steering_angle_radians;

    /*
     * Extract vehicle parameters
     */
    fixed_point_t lf   = stored_vehicle_parameters.distance_cg_to_front_axle_meters;
    fixed_point_t lr   = stored_vehicle_parameters.distance_cg_to_rear_axle_meters;
    fixed_point_t mass = stored_vehicle_parameters.vehicle_mass_kg;
    fixed_point_t Iz   = stored_vehicle_parameters.yaw_moment_of_inertia_kgm2;
    fixed_point_t C_Sf = stored_vehicle_parameters.front_cornering_stiffness;
    fixed_point_t C_Sr = stored_vehicle_parameters.rear_cornering_stiffness;

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
     * Compute slip angle intermediates for Jacobian
     *
     * Front: u_f = (v_y + l_f * omega) / v_x
     *        alpha_f = delta - atan(u_f)
     *        D_f = v_x^2 + (v_y + l_f * omega)^2       (denominator for atan derivative)
     *
     * Rear:  u_r = (v_y - l_r * omega) / v_x
     *        alpha_r = -atan(u_r)
     *        D_r = v_x^2 + (v_y - l_r * omega)^2
     */
    fixed_point_t front_num = fp_add(vy, fp_mul(lf, omega));
    fixed_point_t rear_num  = fp_sub(vy, fp_mul(lr, omega));

    fixed_point_t vx2 = fp_mul(vx_safe, vx_safe);
    fixed_point_t front_num2 = fp_mul(front_num, front_num);
    fixed_point_t rear_num2  = fp_mul(rear_num, rear_num);

    fixed_point_t D_f = fp_add(vx2, front_num2);
    fixed_point_t D_r = fp_add(vx2, rear_num2);

    /* Prevent division by zero */
    if (D_f == 0) D_f = FP_ONE;
    if (D_r == 0) D_r = FP_ONE;

    /*
     * Partial derivatives of slip angles w.r.t. states
     *
     * d(alpha_f)/d(v_x) = front_num / D_f           (positive: atan deriv × chain rule)
     * d(alpha_f)/d(v_y) = -v_x / D_f
     * d(alpha_f)/d(omega) = -l_f * v_x / D_f
     * d(alpha_f)/d(delta) = 1
     *
     * d(alpha_r)/d(v_x) = rear_num / D_r
     * d(alpha_r)/d(v_y) = -v_x / D_r
     * d(alpha_r)/d(omega) = l_r * v_x / D_r
     */
    fixed_point_t daf_dvx    = fp_div(front_num, D_f);
    fixed_point_t daf_dvy    = fp_neg(fp_div(vx_safe, D_f));
    fixed_point_t daf_domega = fp_neg(fp_div(fp_mul(lf, vx_safe), D_f));

    fixed_point_t dar_dvx    = fp_div(rear_num, D_r);
    fixed_point_t dar_dvy    = fp_neg(fp_div(vx_safe, D_r));
    fixed_point_t dar_domega = fp_div(fp_mul(lr, vx_safe), D_r);

    /*
     * Partial derivatives of tire forces w.r.t. states
     *
     * With F_yf = C_Sf * alpha_f:  dF_yf/dx = C_Sf * d(alpha_f)/dx
     * With F_yr = C_Sr * alpha_r:  dF_yr/dx = C_Sr * d(alpha_r)/dx
     */
    fixed_point_t dFyf_dvx    = fp_mul(C_Sf, daf_dvx);
    fixed_point_t dFyf_dvy    = fp_mul(C_Sf, daf_dvy);
    fixed_point_t dFyf_domega = fp_mul(C_Sf, daf_domega);
    fixed_point_t dFyf_ddelta = C_Sf;    /* d(alpha_f)/d(delta) = 1 */

    fixed_point_t dFyr_dvx    = fp_mul(C_Sr, dar_dvx);
    fixed_point_t dFyr_dvy    = fp_mul(C_Sr, dar_dvy);
    fixed_point_t dFyr_domega = fp_mul(C_Sr, dar_domega);

    /*
     * Compute current tire forces for mixed-derivative terms
     */
    fixed_point_t front_ratio = fp_div(front_num, vx_safe);
    fixed_point_t rear_ratio  = fp_div(rear_num, vx_safe);
    fixed_point_t alpha_f = fp_sub(delta, fp_atan(front_ratio));
    fixed_point_t alpha_r = fp_neg(fp_atan(rear_ratio));
    fixed_point_t F_yf = fp_mul(C_Sf, alpha_f);
    /* F_yr not needed for A matrix computation, only its derivatives */
    (void)alpha_r;  /* suppress unused warning - intermediate only */

    /*
     * Initialize A matrix as identity (6x6)
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
     * Row 0 (dx/dt = vx*cos(psi) - vy*sin(psi)):
     *   A[0][2] = -vx*sin(psi) - vy*cos(psi)
     *   A[0][3] = cos(psi)
     *   A[0][4] = -sin(psi)
     *
     * Row 1 (dy/dt = vx*sin(psi) + vy*cos(psi)):
     *   A[1][2] = vx*cos(psi) - vy*sin(psi)
     *   A[1][3] = sin(psi)
     *   A[1][4] = cos(psi)
     *
     * Row 2 (dpsi/dt = omega):
     *   A[2][5] = 1
     *
     * Row 3 (dvx/dt = (Fx - Fyf*sin(d) + m*vy*w) / m):
     *   A[3][3] = (-dFyf_dvx * sin(d)) / m
     *   A[3][4] = (-dFyf_dvy * sin(d) + m*w) / m  =  -dFyf_dvy*sin(d)/m + w
     *   A[3][5] = (-dFyf_domega * sin(d) + m*vy) / m  =  -dFyf_domega*sin(d)/m + vy
     *
     * Row 4 (dvy/dt = (Fyf*cos(d) + Fyr - m*vx*w) / m):
     *   A[4][3] = (dFyf_dvx*cos(d) + dFyr_dvx - m*w) / m
     *   A[4][4] = (dFyf_dvy*cos(d) + dFyr_dvy) / m
     *   A[4][5] = (dFyf_domega*cos(d) + dFyr_domega - m*vx) / m
     *
     * Row 5 (domega/dt = (lf*Fyf*cos(d) - lr*Fyr) / Iz):
     *   A[5][3] = (lf*dFyf_dvx*cos(d) - lr*dFyr_dvx) / Iz
     *   A[5][4] = (lf*dFyf_dvy*cos(d) - lr*dFyr_dvy) / Iz
     *   A[5][5] = (lf*dFyf_domega*cos(d) - lr*dFyr_domega) / Iz
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

    /* Row 3: longitudinal velocity derivatives */
    fixed_point_t inv_m = fp_div(FP_ONE, mass);

    state_matrix_A[3][3] = fp_add(state_matrix_A[3][3], fp_mul(time_step,
        fp_mul(fp_neg(fp_mul(dFyf_dvx, sin_delta)), inv_m)));

    state_matrix_A[3][4] = fp_add(state_matrix_A[3][4], fp_mul(time_step,
        fp_add(fp_mul(fp_neg(fp_mul(dFyf_dvy, sin_delta)), inv_m), omega)));

    state_matrix_A[3][5] = fp_add(state_matrix_A[3][5], fp_mul(time_step,
        fp_add(fp_mul(fp_neg(fp_mul(dFyf_domega, sin_delta)), inv_m), vy)));

    /* Row 4: lateral velocity derivatives */
    state_matrix_A[4][3] = fp_add(state_matrix_A[4][3], fp_mul(time_step,
        fp_mul(fp_add(fp_mul(dFyf_dvx, cos_delta), fp_sub(dFyr_dvx, fp_mul(mass, omega))), inv_m)));

    /* Simplify: (dFyf_dvy*cos_d + dFyr_dvy) / m  — no mass*omega term here */
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

    /*
     * Initialize B matrix as zeros (6x2) and add continuous terms × dt
     *
     * Column 0: control = delta (steering angle)
     *   df4/d(delta) = (-dFyf_ddelta * sin(d) - F_yf * cos(d)) / m
     *   df5/d(delta) = (dFyf_ddelta * cos(d) - F_yf * sin(d)) / m
     *   df6/d(delta) = (lf * (dFyf_ddelta * cos(d) - F_yf * sin(d))) / Iz
     *
     * Column 1: control = F_x (longitudinal force)
     *   df4/d(F_x) = 1 / m
     */
    for (int row = 0; row < 6; row++)
    {
        for (int col = 0; col < 2; col++)
        {
            input_matrix_B[row][col] = 0;
        }
    }

    /* B[3][0]: d(dvx/dt)/d(delta) × dt */
    /* = dt * (-dFyf_ddelta*sin(d) - F_yf*cos(d)) / m */
    fixed_point_t dFyf_dd_sin = fp_mul(dFyf_ddelta, sin_delta);
    fixed_point_t Fyf_cos     = fp_mul(F_yf, cos_delta);
    input_matrix_B[3][0] = fp_mul(time_step,
        fp_mul(fp_sub(fp_neg(dFyf_dd_sin), Fyf_cos), inv_m));

    /* B[4][0]: d(dvy/dt)/d(delta) × dt */
    /* = dt * (dFyf_ddelta*cos(d) - F_yf*sin(d)) / m */
    fixed_point_t dFyf_dd_cos = fp_mul(dFyf_ddelta, cos_delta);
    fixed_point_t Fyf_sin     = fp_mul(F_yf, sin_delta);
    input_matrix_B[4][0] = fp_mul(time_step,
        fp_mul(fp_sub(dFyf_dd_cos, Fyf_sin), inv_m));

    /* B[5][0]: d(domega/dt)/d(delta) × dt */
    /* = dt * lf * (dFyf_ddelta*cos(d) - F_yf*sin(d)) / Iz */
    input_matrix_B[5][0] = fp_mul(time_step,
        fp_mul(fp_mul(lf, fp_sub(dFyf_dd_cos, Fyf_sin)), inv_Iz));

    /* B[3][1]: d(dvx/dt)/d(F_x) × dt = dt / m */
    input_matrix_B[3][1] = fp_mul(time_step, inv_m);
}
