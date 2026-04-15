/**
 * @file mpc_runtime_tune.h
 * @brief Runtime-overridable MPC tuning parameters for host-side sweeps.
 */

#ifndef MPC_RUNTIME_TUNE_H
#define MPC_RUNTIME_TUNE_H

#include "fp_math_hls.h"
#include "fp_types_hls.hpp"

#ifdef __cplusplus
extern "C" {
#endif

extern fp_QP_t mpc_rt_dt;
extern fp_QP_t mpc_rt_horizon;
extern fp_QP_t mpc_rt_w_lat_error;
extern fp_QP_t mpc_rt_w_heading;
extern fp_QP_t mpc_rt_w_velocity;
extern fp_QP_t mpc_rt_w_lat_vel;
extern fp_QP_t mpc_rt_w_yaw_rate;
extern fp_QP_t mpc_rt_w_steer_eff;
extern fp_QP_t mpc_rt_w_accel_eff;
extern fp_QP_t mpc_rt_w_steer_jerk;
extern fp_QP_t mpc_rt_w_accel_rate;
extern fp_QP_t mpc_rt_w_delta_act;

extern fp_QP_t mpc_rt_min_lin_vel;
extern fp_QP_t mpc_rt_stability_limit;
extern fp_QP_t mpc_rt_wall_margin;
extern int mpc_rt_wp_advance_max;

extern fp_QP_t mpc_rt_admm_rho;
extern fp_QP_t mpc_rt_admm_rho_u;
extern fp_QP_t mpc_rt_admm_tol;

void mpc_runtime_update_from_env(void);

#ifdef __cplusplus
}
#endif

#endif
