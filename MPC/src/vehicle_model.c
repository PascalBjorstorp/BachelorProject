/**
 * @file vehicle_model.c
 * @brief Kinematic Bicycle Model Implementation
 *
 * Implements the kinematic bicycle model for F1/10th vehicle dynamics.
 * All calculations use fixed-point arithmetic for FPGA compatibility.
 *
 * Model Equations (velocity is a direct control input):
 *   dx/dt = v_cmd × cos(ψ)
 *   dy/dt = v_cmd × sin(ψ)
 *   dψ/dt = (v_cmd / L) × tan(δ)
 *   v     = v_cmd
 *
 * Where:
 *   (x, y) = position, ψ = heading
 *   δ = steering angle, v_cmd = commanded velocity, L = wheelbase
 */

#include "vehicle_model.h"
#include "fp_math.h"

/*===========================================================================
 * Module State
 *===========================================================================*/

static VehicleParams_t g_params;
static uint8_t g_initialized = 0;

/*===========================================================================
 * Initialization
 *===========================================================================*/

void vehicle_model_init(void)
{
    g_params.wheelbase = F110_WHEELBASE;
    g_params.max_steer = F110_MAX_STEER;
    g_params.max_steer_vel = F110_MAX_STEER_VEL;
    g_params.max_vel = F110_MAX_VEL;
    g_params.min_vel = F110_MIN_VEL;
    g_params.max_accel = F110_MAX_ACCEL;
    g_initialized = 1;
}

void vehicle_model_init_params(const VehicleParams_t *params)
{
    g_params = *params;
    g_initialized = 1;
}

VehicleParams_t vehicle_model_get_params(void)
{
    return g_params;
}

/*===========================================================================
 * Control Saturation
 *===========================================================================*/

ControlInput_t vehicle_model_saturate(const ControlInput_t *raw)
{
    ControlInput_t sat;

    sat.steer = fp_clamp(
        raw->steer,
        fp_neg(g_params.max_steer),
        g_params.max_steer);

    sat.vel = fp_clamp(
        raw->vel,
        g_params.min_vel,
        g_params.max_vel);

    return sat;
}

/*===========================================================================
 * State Prediction (Single Step)
 *===========================================================================*/

VehicleState_t vehicle_model_predict(
    const VehicleState_t *state,
    const ControlInput_t *control,
    fixed_point_t dt)
{
    VehicleState_t next;
    ControlInput_t sat = vehicle_model_saturate(control);

    /* Trig values */
    fixed_point_t cos_h = fp_cos(state->heading);
    fixed_point_t sin_h = fp_sin(state->heading);
    fixed_point_t tan_s = fp_tan(sat.steer);

    /* Derivatives */
    fixed_point_t dx = fp_mul(state->vel, cos_h);
    fixed_point_t dy = fp_mul(state->vel, sin_h);
    fixed_point_t v_over_L = fp_div(state->vel, g_params.wheelbase);
    fixed_point_t dheading = fp_mul(v_over_L, tan_s);

    /* Euler integration */
    next.x = fp_add(state->x, fp_mul(dt, dx));
    next.y = fp_add(state->y, fp_mul(dt, dy));
    next.heading = fp_add(state->heading, fp_mul(dt, dheading));
    next.vel = sat.vel;  /* Direct velocity control */

    /* Clamp velocity */
    next.vel = fp_clamp(next.vel, 0, g_params.max_vel);

    /* Normalize heading to [-π, +π] */
    while (next.heading > FP_PI)
        next.heading = fp_sub(next.heading, FP_TWO_PI);
    while (next.heading < -FP_PI)
        next.heading = fp_add(next.heading, FP_TWO_PI);

    return next;
}

/*===========================================================================
 * Trajectory Prediction (Multiple Steps)
 *===========================================================================*/

void vehicle_model_predict_traj(
    const VehicleState_t *initial,
    const ControlInput_t *controls,
    fixed_point_t dt,
    uint16_t steps,
    VehicleState_t *trajectory)
{
    trajectory[0] = *initial;
    for (uint16_t k = 0; k < steps; k++)
    {
        trajectory[k + 1] = vehicle_model_predict(&trajectory[k], &controls[k], dt);
    }
}

/*===========================================================================
 * Model Linearization
 *===========================================================================*/

void vehicle_model_linearize(
    const VehicleState_t *state,
    const ControlInput_t *control,
    fixed_point_t dt,
    fixed_point_t A[4][4],
    fixed_point_t B[4][2])
{
    fixed_point_t cos_h = fp_cos(state->heading);
    fixed_point_t sin_h = fp_sin(state->heading);
    fixed_point_t tan_s = fp_tan(control->steer);
    fixed_point_t cos_s = fp_cos(control->steer);
    fixed_point_t cos_s_sq = fp_mul(cos_s, cos_s);

    /* Initialize A as identity for x,y,heading; 0 for velocity row */
    for (int r = 0; r < 4; r++)
    {
        for (int c = 0; c < 4; c++)
        {
            if (r == c && r < 3)
                A[r][c] = FP_ONE;
            else
                A[r][c] = 0;
        }
    }

    /* Add continuous-time Jacobian terms × dt */
    fixed_point_t v_sin = fp_mul(state->vel, sin_h);
    fixed_point_t v_cos = fp_mul(state->vel, cos_h);

    A[0][2] = fp_mul(dt, fp_neg(v_sin));  /* ∂x/∂heading */
    A[0][3] = fp_mul(dt, cos_h);                   /* ∂x/∂v */
    A[1][2] = fp_mul(dt, v_cos);                   /* ∂y/∂heading */
    A[1][3] = fp_mul(dt, sin_h);                   /* ∂y/∂v */

    fixed_point_t tan_over_L = fp_div(tan_s, g_params.wheelbase);
    A[2][3] = fp_mul(dt, tan_over_L);              /* ∂heading/∂v */

    /* Initialize B as zeros */
    for (int r = 0; r < 4; r++)
    {
        for (int c = 0; c < 2; c++)
            B[r][c] = 0;
    }

    /* B[2][0] = dt × (v / (L × cos²(steer))) */
    fixed_point_t L_cos_sq = fp_mul(g_params.wheelbase, cos_s_sq);
    fixed_point_t v_over_Lcs = fp_div(state->vel, L_cos_sq);
    B[2][0] = fp_mul(dt, v_over_Lcs);

    /* B[3][1] = 1 (velocity directly equals command) */
    B[3][1] = FP_ONE;
}
