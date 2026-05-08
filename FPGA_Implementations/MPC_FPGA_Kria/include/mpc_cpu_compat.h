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

extern void mpc_fpga_top_scalar_with_prev_accel_and_ref_ey(
    int32_t ey_fp, int32_t epsi_fp,
    int32_t vx_fp, int32_t vy_fp, int32_t omega_fp, int32_t steering_fp, int32_t prev_accel_fp,
    const int32_t *ref_ey, const int32_t *ref_epsi, const int32_t *ref_vx, const int32_t *ref_vy, const int32_t *ref_omega_ref,
    const int32_t *ref_kappa, const int32_t *ref_left, const int32_t *ref_right,
    int ref_count,
    int32_t *out_steering, int32_t *out_accel,
    int32_t *out_status, int32_t *out_iters);

extern void mpc_fpga_reset_persistent_state(void);
#ifdef __cplusplus
}
#endif

#ifndef TIME_STEP_SECONDS
#define TIME_STEP_SECONDS 0.03
#endif

#ifndef PREDICTION_HORIZON
#define PREDICTION_HORIZON MPC_HORIZON
#endif

#define CPU_COMPAT_FLOAT_TO_FP16(x)((int32_t)((double)(x) * MPC_FPGA_Q16_SCALE_F64))
#define CPU_COMPAT_FP16_TO_FLOAT(x)((float)((double)(x) / MPC_FPGA_Q16_SCALE_F64))
#define CPU_COMPAT_DOUBLE_TO_FP16(x)((int32_t)((double)(x) * MPC_FPGA_Q16_SCALE_F64))
#define CPU_COMPAT_FP16_TO_DOUBLE(x)((double)(x) / MPC_FPGA_Q16_SCALE_F64)

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
    float weight_delta_actual;
    float wall_margin;
    int max_solver_iterations;
    float solver_convergence_tolerance;
} MpcConfiguration_t;

typedef struct {
    ControlInput_t optimal_control;
    int iterations_used;
    float final_cost;      /* Unsupported in FPGA scalar API (set to 0). */
    float dual_residual;   /* Unsupported in FPGA scalar API (set to 0). */
    int solver_status;
} MpcSolverResult_t;

typedef enum {
    MPC_SOLVER_STATUS_SUCCESS = 0,
    MPC_SOLVER_STATUS_MAXIMUM_ITERATIONS_REACHED = 1,
    MPC_SOLVER_STATUS_ERROR = 2
} MpcSolverStatus_t;

static MpcConfiguration_t g_mpc_cpu_compat_cfg = {
    .time_step = (float)TIME_STEP_SECONDS,
    .cross_call_rate_scale = (float)(MPC_FPGA_CONTROL_DT_S / MPC_FPGA_PREDICTION_DT_S),
    .weight_lateral_error = (float)MPC_FPGA_W_LAT_ERROR,
    .weight_heading_error = (float)MPC_FPGA_W_HEADING,
    .weight_velocity = (float)MPC_FPGA_W_VELOCITY,
    .weight_lateral_velocity = (float)MPC_FPGA_W_LAT_VEL,
    .weight_yaw_rate = (float)MPC_FPGA_W_YAW_RATE,
    .weight_steering_effort = (float)MPC_FPGA_W_STEER_EFF,
    .weight_acceleration_effort = (float)MPC_FPGA_W_ACCEL_EFF,
    .weight_steering_rate = (float)MPC_FPGA_W_STEER_JERK,
    .weight_acceleration_rate = (float)MPC_FPGA_W_ACCEL_RATE,
    .weight_delta_actual = (float)MPC_FPGA_W_DELTA_ACT,
    .wall_margin = (float)MPC_FPGA_WALL_MARGIN_M,
    .max_solver_iterations = MPC_FPGA_MAX_ADMM_ITER,
    .solver_convergence_tolerance = (float)MPC_FPGA_ADMM_TOL,
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
    cfg.time_step = (float)TIME_STEP_SECONDS;
    cfg.cross_call_rate_scale = (float)(MPC_FPGA_CONTROL_DT_S / MPC_FPGA_PREDICTION_DT_S);
    cfg.weight_lateral_error = (float)MPC_FPGA_W_LAT_ERROR;
    cfg.weight_heading_error = (float)MPC_FPGA_W_HEADING;
    cfg.weight_velocity = (float)MPC_FPGA_W_VELOCITY;
    cfg.weight_lateral_velocity = (float)MPC_FPGA_W_LAT_VEL;
    cfg.weight_yaw_rate = (float)MPC_FPGA_W_YAW_RATE;
    cfg.weight_steering_effort = (float)MPC_FPGA_W_STEER_EFF;
    cfg.weight_acceleration_effort = (float)MPC_FPGA_W_ACCEL_EFF;
    cfg.weight_steering_rate = (float)MPC_FPGA_W_STEER_JERK;
    cfg.weight_acceleration_rate = (float)MPC_FPGA_W_ACCEL_RATE;
    cfg.weight_delta_actual = (float)MPC_FPGA_W_DELTA_ACT;
    cfg.wall_margin = (float)MPC_FPGA_WALL_MARGIN_M;
    cfg.max_solver_iterations = MPC_FPGA_MAX_ADMM_ITER;
    cfg.solver_convergence_tolerance = (float)MPC_FPGA_ADMM_TOL;
    return cfg;
}

static inline void mpc_initialize(void)
{
    g_mpc_cpu_compat_cfg = mpc_cpu_compat_default_config();
    g_mpc_cpu_compat_actual_steering = 0.0f;
    g_mpc_cpu_compat_prev_accel = 0.0f;
#ifdef __cplusplus
    mpc_fpga_reset_persistent_state();
#endif
}
static inline void mpc_reset(void)
{
    g_mpc_cpu_compat_cfg = mpc_cpu_compat_default_config();
    g_mpc_cpu_compat_actual_steering = 0.0f;
    g_mpc_cpu_compat_prev_accel = 0.0f;
#ifdef __cplusplus
    mpc_fpga_reset_persistent_state();
#endif
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
    mpc_cpu_compat_set_env_double("W_DELTA_ACT", (double)cfg->weight_delta_actual);
    mpc_cpu_compat_set_env_double("WALL_MARGIN", (double)cfg->wall_margin);
    mpc_cpu_compat_set_env_int("MAX_ITER", (int)cfg->max_solver_iterations);
    mpc_cpu_compat_set_env_double("TOL", (double)cfg->solver_convergence_tolerance);
    /* Do not override RHO/RHO_U here.
     * Keep parity with CPU path where ADMM penalties are read directly from
     * environment (or solver defaults) at solve time. */
#ifdef MPC_RUNTIME_TUNE
    mpc_runtime_reload_from_env();
#endif
    
    /* Note: Runtime horizon control disabled. Horizon is compile-time only.
     * All internal arrays are fixed-size for HLS synthesis. */  
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

    int32_t ref_ey_fp[MPC_HORIZON];
    int32_t ref_epsi_fp[MPC_HORIZON];
    int32_t ref_vx_fp[MPC_HORIZON];
    int32_t ref_vy_fp[MPC_HORIZON];
    int32_t ref_omega_ref_fp[MPC_HORIZON];
    int32_t ref_kappa_fp[MPC_HORIZON];
    int32_t ref_left_fp[MPC_HORIZON];
    int32_t ref_right_fp[MPC_HORIZON];

    for (int i = 0; i < MPC_HORIZON; i++) {
        ref_ey_fp[i] = CPU_COMPAT_DOUBLE_TO_FP16(ref[i].reference_lateral_error);
        ref_epsi_fp[i] = CPU_COMPAT_DOUBLE_TO_FP16(ref[i].reference_heading_error);
        ref_vx_fp[i] = CPU_COMPAT_DOUBLE_TO_FP16(ref[i].reference_velocity);
        ref_vy_fp[i] = CPU_COMPAT_DOUBLE_TO_FP16(ref[i].reference_lateral_velocity);
        ref_omega_ref_fp[i] = CPU_COMPAT_DOUBLE_TO_FP16(ref[i].reference_yaw_rate);
        ref_kappa_fp[i] = CPU_COMPAT_DOUBLE_TO_FP16(ref[i].path_curvature);
        ref_left_fp[i] = CPU_COMPAT_DOUBLE_TO_FP16(ref[i].left_wall_bound);
        ref_right_fp[i] = CPU_COMPAT_DOUBLE_TO_FP16(ref[i].right_wall_bound);
    }

    int32_t out_steering = 0;
    int32_t out_accel = 0;
    int32_t out_status = 0;
    int32_t out_iters = 0;

    /* Forward previous applied acceleration to FPGA scalar wrapper for parity */
    mpc_fpga_top_scalar_with_prev_accel_and_ref_ey(
    CPU_COMPAT_DOUBLE_TO_FP16(state->flat_error),
    CPU_COMPAT_DOUBLE_TO_FP16(state->fhead_error),
    CPU_COMPAT_DOUBLE_TO_FP16(state->flong_vel),
    CPU_COMPAT_DOUBLE_TO_FP16(state->flat_vel),
    CPU_COMPAT_DOUBLE_TO_FP16(state->fyaw_rate),
    CPU_COMPAT_DOUBLE_TO_FP16(g_mpc_cpu_compat_actual_steering),
    CPU_COMPAT_DOUBLE_TO_FP16(g_mpc_cpu_compat_prev_accel),
    ref_ey_fp, ref_epsi_fp, ref_vx_fp, ref_vy_fp, ref_omega_ref_fp,
    /* ref_kappa */ ref_kappa_fp, /* ref_left */ ref_left_fp, /* ref_right */ ref_right_fp,
    MPC_HORIZON,
    &out_steering, &out_accel, &out_status, &out_iters);

    result->optimal_control.steer_ang = CPU_COMPAT_FP16_TO_FLOAT(out_steering);
    result->optimal_control.long_acc = CPU_COMPAT_FP16_TO_FLOAT(out_accel);
    result->iterations_used = (int)out_iters;
    result->final_cost = 0.0f;     /* Unsupported by current scalar wrapper */
    result->dual_residual = 0.0f;  /* Unsupported by current scalar wrapper */
    result->solver_status = (int)out_status;

    if (out_status == MPC_FPGA_STATUS_OK) {
        return MPC_SOLVER_STATUS_SUCCESS;
    }
    if (out_status == MPC_FPGA_STATUS_MAX_ITER) {
        return MPC_SOLVER_STATUS_MAXIMUM_ITERATIONS_REACHED;
    }
    return MPC_SOLVER_STATUS_ERROR;
}

#endif
