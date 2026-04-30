#ifndef MPC_CPU_COMPAT_H
#define MPC_CPU_COMPAT_H

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "mpc_fpga_types.h"
#include "mpc_fpga_interface.h"
#include "fp_math_hls.h"

#ifdef MPC_RUNTIME_TUNE
#include "mpc_runtime_tune.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif
extern void mpc_fpga_top_scalar(
    int32_t ey_fp, int32_t epsi_fp,
    int32_t vx_fp, int32_t vy_fp, int32_t omega_fp,
    int32_t steering_fp,
    const int32_t *ref_vx,
    const int32_t *ref_kappa,
    const int32_t *ref_left,
    const int32_t *ref_right,
    int ref_count,
    int32_t *out_steering, int32_t *out_accel,
    int32_t *out_status, int32_t *out_iters);
// Extended scalar wrapper that accepts previous applied acceleration (m/s^2)
extern void mpc_fpga_top_scalar_with_prev_accel(
    int32_t ey_fp, int32_t epsi_fp,
    int32_t vx_fp, int32_t vy_fp, int32_t omega_fp, int32_t steering_fp, int32_t prev_accel_fp,
    const int32_t *ref_vx, const int32_t *ref_kappa, const int32_t *ref_left, const int32_t *ref_right,
    int ref_count,
    int32_t *out_steering, int32_t *out_accel,
    int32_t *out_status, int32_t *out_iters);
#ifdef __cplusplus
}
#endif

#ifndef TIME_STEP_SECONDS
#define TIME_STEP_SECONDS 0.03
#endif

#ifndef PREDICTION_HORIZON
#define PREDICTION_HORIZON MPC_HORIZON
#endif

typedef struct {
    float flat_error;
    float fhead_error;
    float flong_vel;
    float flat_vel;
    float fyaw_rate;
} FrenetState_t;

typedef struct {
    float pos_x;
    float pos_y;
    float heading;
    float long_vel;
    float lat_vel;
    float yaw_rate;
} VehicleState_t;

typedef struct {
    float steer_ang;
    float long_acc;
} ControlInput_t;

typedef struct {
    float reference_lateral_error;
    float reference_heading_error;
    float reference_velocity;
    float reference_lateral_velocity;
    float reference_yaw_rate;
    float path_curvature;
    float left_wall_bound;
    float right_wall_bound;
} TrajectoryReferencePoint_t;

typedef struct {
    uint16_t prediction_horizon_steps;
    float time_step;
    float cross_call_rate_scale;
    float weight_lateral_error;
    float weight_heading_error;
    float weight_velocity;
    float weight_lateral_velocity;
    float weight_yaw_rate;
    float weight_steering_effort;
    float weight_acceleration_effort;
    float weight_steering_rate;
    float weight_acceleration_rate;
    float wall_margin;
    int max_solver_iterations;
    float solver_convergence_tolerance;
} MpcConfiguration_t;

typedef struct {
    ControlInput_t optimal_control;
    int iterations_used;
} MpcSolverResult_t;

typedef enum {
    MPC_SOLVER_STATUS_SUCCESS = 0,
    MPC_SOLVER_STATUS_MAXIMUM_ITERATIONS_REACHED = 1,
    MPC_SOLVER_STATUS_ERROR = 2
} MpcSolverStatus_t;

static MpcConfiguration_t g_mpc_cpu_compat_cfg = {
    .prediction_horizon_steps = (uint16_t)PREDICTION_HORIZON,
    .time_step = (float)TIME_STEP_SECONDS,
    .cross_call_rate_scale = 1.0f,
    .weight_lateral_error = 200.0f,
    .weight_heading_error = 28.8f,
    .weight_velocity = 30.0f,
    .weight_lateral_velocity = 1.04f,
    .weight_yaw_rate = 1.5f,
    .weight_steering_effort = 1.5f,
    .weight_acceleration_effort = 0.01f,
    .weight_steering_rate = 0.04f,
    .weight_acceleration_rate = 0.10f,
    .wall_margin = 0.197f,
    .max_solver_iterations = 100,
    .solver_convergence_tolerance = 0.05f,
};
static float g_mpc_cpu_compat_actual_steering = 0.0f;
static float g_mpc_cpu_compat_prev_accel = 0.0f;

static inline void mpc_cpu_compat_set_env_double(const char *name, double value)
{
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "%.17g", value);
    setenv(name, buffer, 1);
}

static inline void mpc_cpu_compat_set_env_int(const char *name, int value)
{
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%d", value);
    setenv(name, buffer, 1);
}

static inline MpcConfiguration_t mpc_cpu_compat_default_config(void)
{
    MpcConfiguration_t cfg;
    cfg.prediction_horizon_steps = (uint16_t)PREDICTION_HORIZON;
    cfg.time_step = (float)TIME_STEP_SECONDS;
    cfg.cross_call_rate_scale = 1.0f;
    cfg.weight_lateral_error = 200.0f;
    cfg.weight_heading_error = 28.8f;
    cfg.weight_velocity = 30.0f;
    cfg.weight_lateral_velocity = 1.04f;
    cfg.weight_yaw_rate = 1.5f;
    cfg.weight_steering_effort = 1.5f;
    cfg.weight_acceleration_effort = 0.01f;
    cfg.weight_steering_rate = 0.04f;
    cfg.weight_acceleration_rate = 0.10f;
    cfg.wall_margin = 0.197f;
    cfg.max_solver_iterations = 100;
    cfg.solver_convergence_tolerance = 0.05f;
    return cfg;
}

static inline void mpc_initialize(void) {}
static inline void mpc_reset(void)
{
    g_mpc_cpu_compat_cfg = mpc_cpu_compat_default_config();
    g_mpc_cpu_compat_actual_steering = 0.0f;
}

static inline MpcConfiguration_t mpc_get_configuration(void)
{
    return g_mpc_cpu_compat_cfg;
}

static inline void mpc_set_configuration(const MpcConfiguration_t *cfg)
{
    if (!cfg) {
        return;
    }

    g_mpc_cpu_compat_cfg = *cfg;

    mpc_cpu_compat_set_env_int("HORIZON", (int)cfg->prediction_horizon_steps);
    mpc_cpu_compat_set_env_double("PRED_DT", (double)cfg->time_step);
    mpc_cpu_compat_set_env_double("Q_LAT", (double)cfg->weight_lateral_error);
    mpc_cpu_compat_set_env_double("Q_HDG", (double)cfg->weight_heading_error);
    mpc_cpu_compat_set_env_double("Q_VEL", (double)cfg->weight_velocity);
    mpc_cpu_compat_set_env_double("Q_LAT_VEL", (double)cfg->weight_lateral_velocity);
    mpc_cpu_compat_set_env_double("Q_YAW", (double)cfg->weight_yaw_rate);
    mpc_cpu_compat_set_env_double("R_STEER", (double)cfg->weight_steering_effort);
    mpc_cpu_compat_set_env_double("R_ACCEL", (double)cfg->weight_acceleration_effort);
    mpc_cpu_compat_set_env_double("W_JERK", (double)cfg->weight_steering_rate);
    mpc_cpu_compat_set_env_double("W_ACCEL_RATE", (double)cfg->weight_acceleration_rate);
    mpc_cpu_compat_set_env_double("WALL_MARGIN", (double)cfg->wall_margin);
    mpc_cpu_compat_set_env_int("MAX_ITER", (int)cfg->max_solver_iterations);
    /* Do not overwrite TOL here; let constants and sweep scripts control TOL.
     * mpc_runtime_update_from_env() will still read the environment if the
     * sweep sets it explicitly. */
    /* Do not override RHO/RHO_U here.
     * Keep parity with CPU path where ADMM penalties are read directly from
     * environment (or solver defaults) at solve time. */
}

static inline void mpc_set_actual_previous_control(const ControlInput_t *ctrl)
{
    if (ctrl) {
        g_mpc_cpu_compat_actual_steering = ctrl->steer_ang;
        g_mpc_cpu_compat_prev_accel = ctrl->long_acc;
    }
}

static inline MpcSolverStatus_t mpc_compute_optimal_control(
    const FrenetState_t *state,
    const TrajectoryReferencePoint_t ref[PREDICTION_HORIZON],
    MpcSolverResult_t *result)
{
    if (!state || !ref || !result) {
        return MPC_SOLVER_STATUS_ERROR;
    }

    int horizon = PREDICTION_HORIZON;
    if (horizon < 1) {
        return MPC_SOLVER_STATUS_ERROR;
    }

    int32_t ref_vx_fp[MPC_HORIZON];
    int32_t ref_kappa_fp[MPC_HORIZON];
    int32_t ref_left_fp[MPC_HORIZON];
    int32_t ref_right_fp[MPC_HORIZON];

    for (int i = 0; i < horizon; i++) {
        ref_vx_fp[i] = DOUBLE_TO_FP((double)ref[i].reference_velocity);
        ref_kappa_fp[i] = DOUBLE_TO_FP((double)ref[i].path_curvature);
        ref_left_fp[i] = DOUBLE_TO_FP((double)ref[i].left_wall_bound);
        ref_right_fp[i] = DOUBLE_TO_FP((double)ref[i].right_wall_bound);
    }

    int32_t out_steering = 0;
    int32_t out_accel = 0;
    int32_t out_status = 0;
    int32_t out_iters = 0;

    /* Forward previous applied acceleration to FPGA scalar wrapper for parity */
    mpc_fpga_top_scalar_with_prev_accel(
        DOUBLE_TO_FP((double)state->flat_error),
        DOUBLE_TO_FP((double)state->fhead_error),
        DOUBLE_TO_FP((double)state->flong_vel),
        DOUBLE_TO_FP((double)state->flat_vel),
        DOUBLE_TO_FP((double)state->fyaw_rate),
        DOUBLE_TO_FP((double)g_mpc_cpu_compat_actual_steering),
        DOUBLE_TO_FP((double)g_mpc_cpu_compat_prev_accel),
        ref_vx_fp, ref_kappa_fp, ref_left_fp, ref_right_fp,
        horizon,
        &out_steering, &out_accel, &out_status, &out_iters);

    result->optimal_control.steer_ang = (float)FP_TO_DOUBLE(out_steering);
    result->optimal_control.long_acc = (float)FP_TO_DOUBLE(out_accel);
    result->iterations_used = (int)out_iters;

    if (out_status == MPC_FPGA_STATUS_OK) {
        return MPC_SOLVER_STATUS_SUCCESS;
    }
    if (out_status == MPC_FPGA_STATUS_MAX_ITER) {
        return MPC_SOLVER_STATUS_MAXIMUM_ITERATIONS_REACHED;
    }
    return MPC_SOLVER_STATUS_ERROR;
}

#endif
