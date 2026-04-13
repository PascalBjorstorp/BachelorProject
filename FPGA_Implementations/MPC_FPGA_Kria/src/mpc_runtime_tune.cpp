/**
 * @file mpc_runtime_tune.cpp
 * @brief Runtime tuning parameter store for host-only sweep binaries.
 */

#include "../include/mpc_runtime_tune.h"
#include "../include/mpc_fpga_constants.h"

#include <errno.h>
#include <math.h>
#include <stdlib.h>

fp_QP_t mpc_rt_dt = ((fp_QP_t)(MPC_FPGA_PREDICTION_DT_S));
fp_QP_t mpc_rt_w_lat_error = ((fp_QP_t)(MPC_FPGA_W_LAT_ERROR));
fp_QP_t mpc_rt_w_heading = ((fp_QP_t)(MPC_FPGA_W_HEADING));
fp_QP_t mpc_rt_w_velocity = ((fp_QP_t)(MPC_FPGA_W_VELOCITY));
fp_QP_t mpc_rt_w_lat_vel = ((fp_QP_t)(MPC_FPGA_W_LAT_VEL));
fp_QP_t mpc_rt_w_yaw_rate = ((fp_QP_t)(MPC_FPGA_W_YAW_RATE));
fp_QP_t mpc_rt_w_steer_eff = ((fp_QP_t)(MPC_FPGA_W_STEER_EFF));
fp_QP_t mpc_rt_w_accel_eff = ((fp_QP_t)(MPC_FPGA_W_ACCEL_EFF));
fp_QP_t mpc_rt_w_steer_jerk = ((fp_QP_t)(MPC_FPGA_W_STEER_JERK));
fp_QP_t mpc_rt_w_accel_rate = ((fp_QP_t)(MPC_FPGA_W_ACCEL_RATE));
fp_QP_t mpc_rt_w_delta_act = ((fp_QP_t)(MPC_FPGA_W_DELTA_ACT));

fp_QP_t mpc_rt_min_lin_vel = ((fp_QP_t)(MPC_FPGA_MIN_LIN_VEL_MPS));
fp_QP_t mpc_rt_stability_limit = ((fp_QP_t)(MPC_FPGA_STABILITY_LIMIT));
fp_QP_t mpc_rt_wall_margin = ((fp_QP_t)(MPC_FPGA_WALL_MARGIN_M));
int mpc_rt_wall_start = MPC_FPGA_WALL_START;
int mpc_rt_wall_stride = MPC_FPGA_WALL_STRIDE;
int mpc_rt_wall_end = MPC_FPGA_WALL_END;
int mpc_rt_wp_advance_max = MPC_FPGA_WP_ADVANCE_MAX;

fp_QP_t mpc_rt_admm_rho = ((fp_QP_t)(MPC_FPGA_ADMM_RHO));
fp_QP_t mpc_rt_admm_rho_u = ((fp_QP_t)(MPC_FPGA_ADMM_RHO_U));
fp_QP_t mpc_rt_admm_tol = ((fp_QP_t)(MPC_FPGA_ADMM_TOL));
int mpc_rt_max_admm_iter = MPC_FPGA_MAX_ADMM_ITER;

static int read_env_double(const char *name, double *out)
{
    const char *raw = getenv(name);
    if (!raw || !raw[0]) {
        return 0;
    }

    errno = 0;
    char *end = NULL;
    double parsed = strtod(raw, &end);
    if (end == raw || errno != 0 || !isfinite(parsed)) {
        return 0;
    }

    *out = parsed;
    return 1;
}

static int read_env_int(const char *name, int *out)
{
    const char *raw = getenv(name);
    if (!raw || !raw[0]) {
        return 0;
    }

    errno = 0;
    char *end = NULL;
    long parsed = strtol(raw, &end, 10);
    if (end == raw || errno != 0) {
        return 0;
    }

    *out = (int)parsed;
    return 1;
}

void mpc_runtime_update_from_env(void)
{
    double dv = 0.0;
    int iv = 0;

    if (read_env_double("PRED_DT", &dv) || read_env_double("dt", &dv)) {
        mpc_rt_dt = (fp_QP_t)dv;
    }
    if (read_env_double("Q_LAT", &dv)) mpc_rt_w_lat_error = (fp_QP_t)dv;
    if (read_env_double("Q_HDG", &dv)) mpc_rt_w_heading = (fp_QP_t)dv;
    if (read_env_double("Q_VEL", &dv)) mpc_rt_w_velocity = (fp_QP_t)dv;
    if (read_env_double("Q_LAT_VEL", &dv)) mpc_rt_w_lat_vel = (fp_QP_t)dv;
    if (read_env_double("Q_YAW", &dv)) mpc_rt_w_yaw_rate = (fp_QP_t)dv;
    if (read_env_double("R_STEER", &dv)) mpc_rt_w_steer_eff = (fp_QP_t)dv;
    if (read_env_double("R_ACCEL", &dv)) mpc_rt_w_accel_eff = (fp_QP_t)dv;
    if (read_env_double("W_JERK", &dv)) mpc_rt_w_steer_jerk = (fp_QP_t)dv;
    if (read_env_double("W_ACCEL_RATE", &dv)) mpc_rt_w_accel_rate = (fp_QP_t)dv;
    if (read_env_double("W_DELTA_ACT", &dv)) mpc_rt_w_delta_act = (fp_QP_t)dv;

    if (read_env_double("MIN_LIN_VEL", &dv)) mpc_rt_min_lin_vel = (fp_QP_t)dv;
    if (read_env_double("STABILITY_LIMIT", &dv)) mpc_rt_stability_limit = (fp_QP_t)dv;
    if (read_env_double("WALL_MARGIN", &dv)) mpc_rt_wall_margin = (fp_QP_t)dv;

    if (read_env_int("WALL_START", &iv) && iv >= 0) mpc_rt_wall_start = iv;
    if (read_env_int("WALL_STRIDE", &iv) && iv > 0) mpc_rt_wall_stride = iv;
    if (read_env_int("WALL_END", &iv) && iv >= 0) mpc_rt_wall_end = iv;
    if (read_env_int("HORIZON", &iv) && iv >= 1) mpc_rt_wall_end = iv;
    if (read_env_int("WP_ADVANCE_MAX", &iv) && iv > 0) mpc_rt_wp_advance_max = iv;

    if (read_env_double("RHO", &dv) || read_env_double("ADMM_RHO", &dv)) {
        mpc_rt_admm_rho = (fp_QP_t)dv;
    }
    if (read_env_double("RHO_U", &dv) || read_env_double("ADMM_RHO_U", &dv)) {
        mpc_rt_admm_rho_u = (fp_QP_t)dv;
    }
    if (read_env_double("TOL", &dv) || read_env_double("ADMM_TOL", &dv)) {
        mpc_rt_admm_tol = (fp_QP_t)dv;
    }

    if (read_env_int("MAX_ITER", &iv) || read_env_int("MAX_ADMM_ITER", &iv)) {
        if (iv < 1) iv = 1;
        if (iv > MPC_FPGA_MAX_ADMM_ITER) iv = MPC_FPGA_MAX_ADMM_ITER;
        mpc_rt_max_admm_iter = iv;
    }
}
