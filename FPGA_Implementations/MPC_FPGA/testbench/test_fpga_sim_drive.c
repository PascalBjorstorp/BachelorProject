/**
 * @file test_fpga_sim_drive.c
 * @brief Closed-loop MPC simulation test for the FPGA HLS implementation
 *
 * Tests the FPGA top-level function (mpc_fpga_top) in a 60-second
 * closed-loop simulation using the nonlinear single-track vehicle model
 * (matching f1tenth_gym). Runs at 200Hz (5ms dt) with the Spielberg
 * raceline.
 *
 * Compile (standalone GCC, no Vitis needed):
 *   cd FPGA_Implementations/MPC_FPGA
 *   gcc -D_GNU_SOURCE -O2 -std=c99 -Wall -Wno-unknown-pragmas -Iinclude \
 *       testbench/test_fpga_sim_drive.c src/riccati_solver_hls.c \
 *       src/mpc_riccati_hls.c src/mpc_fpga_top.c src/vehicle_model_hls.c \
 *       src/fp_math_hls.c -o test_fpga_sim -lm
 *   ./test_fpga_sim
 */

#define _USE_MATH_DEFINES
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "mpc_fpga_types.h"
#include "fp_math_hls.h"

/* External: FPGA top-level function */
extern void mpc_fpga_top(
    int mode, int wp_index,
    int wp_x_fp, int wp_y_fp, int wp_psi_fp,
    int wp_vx_fp, int wp_kappa_fp, int wp_ax_fp,
    int wp_left_bound_fp, int wp_right_bound_fp, int wp_total,
    int state_x_fp, int state_y_fp, int state_theta_fp,
    int state_vx_fp, int state_vy_fp, int state_omega_fp,
    int state_steering_fp, int state_wp_idx,
    int *out_steering_fp, int *out_accel_fp,
    int *out_status, int *out_iterations);

/*===========================================================================
 * Configuration
 *===========================================================================*/

#define SIM_DT            0.005   /* 5ms simulation step */
#define SIM_DURATION      60.0    /* seconds */
#define SIM_STEPS         ((int)(SIM_DURATION / SIM_DT))
#define MAX_WAYPOINTS     2000
#define MAX_STEERING      0.4282  /* rad */
#define MAX_VELOCITY      20.0    /* m/s */
#define PHYSICAL_MAX_ACCEL 8.0    /* m/s^2 */
#define MIN_SPEED_FOR_MPC 0.5     /* m/s */
#define MPC_CALL_INTERVAL 1       /* Call MPC every sim step = 5ms = 200Hz (matches CPU test) */

/*===========================================================================
 * Vehicle Model — Single Track (ST) dynamic model matching f1tenth_gym
 *
 * State: [X, Y, delta, V, psi, psi_dot, beta] (7 states)
 * Matches f1tenth_gym single_track.py exactly.
 * Vehicle params from measured data (sim.yaml / vehicle_params.yaml).
 *===========================================================================*/

typedef struct {
    double x, y, theta;
    double vx, vy, omega;
} SimState_t;

/* Vehicle parameters matching gym config */
static const double ST_mu       = 0.7463;
static const double ST_mass     = 3.314;
static const double ST_Iz       = 0.035;
static const double ST_C_Sf     = 2.804;
static const double ST_C_Sr     = 3.320;
static const double ST_lf       = 0.166;
static const double ST_lr       = 0.16;
static const double ST_h_cg     = 0.0703;
static const double ST_g_acc    = 9.81;
static const double ST_sv_max   = 2.8492;  /* max steering velocity (rad/s) */
static const double ST_s_max    = 0.4282;  /* max steering angle (rad) */
static const double ST_v_switch = 7.319;
static const double ST_v_min    = 0.0;
static const double ST_v_max    = 20.0;
static const double ST_lwb      = 0.166 + 0.16;  /* lf + lr */

/* Persistent ST state variables (survive across propagate_vehicle calls) */
static double st_delta       = 0.0;
static double st_V           = 0.0;
static double st_psi_dot     = 0.0;
static double st_beta        = 0.0;
static int    st_initialized = 0;

static double st_wrap_angle(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

/* ST dynamics RHS function matching f1tenth_gym single_track.py exactly.
 * Kinematic mode (V < 0.5): same as CommonRoad.
 * Dynamic mode (V >= 0.5): full nonlinear model with:
 *   - atan2-based slip angles (not small-angle approx)
 *   - cos(δ)/sin(δ) front force resolution
 *   - Full body-frame dynamics for V_DOT, BETA_DOT */
#define ST_DYNAMICS(X, Y, DELTA, V, PSI, PSI_DOT_VAL, BETA, SV, ACCL, \
                   dX, dY, dDELTA, dV, dPSI, dPSI_DOT, dBETA) \
do { \
    if ((V) < 0.5) { \
        /* Kinematic mode */ \
        double beta_hat = atan(tan(DELTA) * ST_lr / ST_lwb); \
        double beta_dot = (1.0 / (1.0 + pow(tan(DELTA) * ST_lr / ST_lwb, 2.0))) \
                        * (ST_lr / (ST_lwb * pow(cos(DELTA), 2.0))) * (SV); \
        (dX) = (V) * cos((PSI) + beta_hat); \
        (dY) = (V) * sin((PSI) + beta_hat); \
        (dDELTA) = (SV); \
        (dV) = (ACCL); \
        (dPSI) = (V) * cos(beta_hat) * tan(DELTA) / ST_lwb; \
        (dPSI_DOT) = (1.0 / ST_lwb) * ( \
            (ACCL) * cos(BETA) * tan(DELTA) \
            - (V) * sin(BETA) * tan(DELTA) * beta_dot \
            + ((V) * cos(BETA) * (SV)) / pow(cos(DELTA), 2.0)); \
        (dBETA) = beta_dot; \
    } else { \
        /* Dynamic ST mode — full nonlinear (matching gym single_track.py) */ \
        double vx_ = (V) * cos(BETA); \
        double vy_ = (V) * sin(BETA); \
        double vx_safe_ = (vx_ > 0.5) ? vx_ : 0.5; \
        /* Normal forces with longitudinal load transfer */ \
        double Fzf_ = ST_mass * (ST_g_acc * ST_lr - (ACCL) * ST_h_cg) / ST_lwb; \
        double Fzr_ = ST_mass * (ST_g_acc * ST_lf + (ACCL) * ST_h_cg) / ST_lwb; \
        /* atan2-based slip angles (not small-angle approx) */ \
        double alpha_f_ = (DELTA) - atan2(vy_ + ST_lf * (PSI_DOT_VAL), vx_safe_); \
        double alpha_r_ = -atan2(vy_ - ST_lr * (PSI_DOT_VAL), vx_safe_); \
        /* Lateral tire forces: F_y = mu * C_S * alpha * F_z */ \
        double Fyf_ = ST_mu * ST_C_Sf * alpha_f_ * Fzf_; \
        double Fyr_ = ST_mu * ST_C_Sr * alpha_r_ * Fzr_; \
        /* Longitudinal force from acceleration command */ \
        double Fx_ = ST_mass * (ACCL); \
        double cos_delta_ = cos(DELTA); \
        double sin_delta_ = sin(DELTA); \
        /* Body-frame dynamics with cos(δ)/sin(δ) force resolution */ \
        double dvx_dt_ = (Fx_ - Fyf_ * sin_delta_ \
                          + ST_mass * vy_ * (PSI_DOT_VAL)) / ST_mass; \
        double dvy_dt_ = (Fyf_ * cos_delta_ + Fyr_ \
                          - ST_mass * vx_ * (PSI_DOT_VAL)) / ST_mass; \
        /* Convert body-frame accelerations to (V, β) derivatives */ \
        double V_safe_ = ((V) > 0.001) ? (V) : 0.001; \
        double V_sq_ = (V) * (V); \
        if (V_sq_ < 0.001) V_sq_ = 0.001; \
        (dX) = (V) * cos((PSI) + (BETA)); \
        (dY) = (V) * sin((PSI) + (BETA)); \
        (dDELTA) = (SV); \
        (dV) = (vx_ * dvx_dt_ + vy_ * dvy_dt_) / V_safe_; \
        (dPSI) = (PSI_DOT_VAL); \
        (dPSI_DOT) = (ST_lf * Fyf_ * cos_delta_ - ST_lr * Fyr_) / ST_Iz; \
        (dBETA) = (vx_ * dvy_dt_ - vy_ * dvx_dt_) / V_sq_; \
    } \
} while(0)

static SimState_t propagate_vehicle(SimState_t s, double desired_steer, double accel, double dt)
{
    /* Initialize persistent ST state on first call */
    if (!st_initialized) {
        st_delta   = desired_steer;
        st_V       = s.vx;
        st_psi_dot = 0.0;
        st_beta    = 0.0;
        st_initialized = 1;
    }

    /* steering_constraint: bang-bang controller for steering velocity */
    double steer_vel;
    {
        double diff = desired_steer - st_delta;
        if (diff > 0)      steer_vel = ST_sv_max;
        else if (diff < 0) steer_vel = -ST_sv_max;
        else               steer_vel = 0.0;
        /* Clip so we don't overshoot the target */
        if (fabs(diff) < ST_sv_max * dt)
            steer_vel = diff / dt;
        /* Enforce steering angle limits */
        if (st_delta >= ST_s_max && steer_vel > 0) steer_vel = 0.0;
        if (st_delta <= -ST_s_max && steer_vel < 0) steer_vel = 0.0;
    }

    /* accl_constraints: v_switch power limit */
    double accl = accel;
    if (st_V > ST_v_switch) {
        double a_max_eff = PHYSICAL_MAX_ACCEL * ST_v_switch / st_V;
        if (accl > a_max_eff) accl = a_max_eff;
    }
    if (accl > PHYSICAL_MAX_ACCEL) accl = PHYSICAL_MAX_ACCEL;
    if (accl < -PHYSICAL_MAX_ACCEL) accl = -PHYSICAL_MAX_ACCEL;
    /* Velocity bounds */
    if (st_V <= ST_v_min && accl < 0) accl = 0.0;
    if (st_V >= ST_v_max && accl > 0) accl = 0.0;

    /* RK4 integration (matching gym's integrator) */
    double k1[7], k2[7], k3[7], k4[7];
    double s0[7] = {s.x, s.y, st_delta, st_V, s.theta, st_psi_dot, st_beta};

    /* k1 */
    ST_DYNAMICS(s0[0], s0[1], s0[2], s0[3], s0[4], s0[5], s0[6],
                steer_vel, accl,
                k1[0], k1[1], k1[2], k1[3], k1[4], k1[5], k1[6]);

    /* k2 */
    {
        double s1[7];
        for (int i = 0; i < 7; i++) s1[i] = s0[i] + 0.5 * dt * k1[i];
        ST_DYNAMICS(s1[0], s1[1], s1[2], s1[3], s1[4], s1[5], s1[6],
                    steer_vel, accl,
                    k2[0], k2[1], k2[2], k2[3], k2[4], k2[5], k2[6]);
    }

    /* k3 */
    {
        double s2[7];
        for (int i = 0; i < 7; i++) s2[i] = s0[i] + 0.5 * dt * k2[i];
        ST_DYNAMICS(s2[0], s2[1], s2[2], s2[3], s2[4], s2[5], s2[6],
                    steer_vel, accl,
                    k3[0], k3[1], k3[2], k3[3], k3[4], k3[5], k3[6]);
    }

    /* k4 */
    {
        double s3[7];
        for (int i = 0; i < 7; i++) s3[i] = s0[i] + dt * k3[i];
        ST_DYNAMICS(s3[0], s3[1], s3[2], s3[3], s3[4], s3[5], s3[6],
                    steer_vel, accl,
                    k4[0], k4[1], k4[2], k4[3], k4[4], k4[5], k4[6]);
    }

    /* RK4 update */
    double sn[7];
    for (int i = 0; i < 7; i++)
        sn[i] = s0[i] + (dt / 6.0) * (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);

    /* Clamp steering angle */
    if (sn[2] > ST_s_max) sn[2] = ST_s_max;
    if (sn[2] < -ST_s_max) sn[2] = -ST_s_max;
    /* Clamp velocity */
    if (sn[3] < ST_v_min) sn[3] = ST_v_min;
    if (sn[3] > ST_v_max) sn[3] = ST_v_max;
    /* Normalize heading */
    sn[4] = st_wrap_angle(sn[4]);

    /* Update persistent ST state */
    st_delta   = sn[2];
    st_V       = sn[3];
    st_psi_dot = sn[5];
    st_beta    = sn[6];

    /* Convert ST state to SimState_t */
    SimState_t ns;
    ns.x     = sn[0];
    ns.y     = sn[1];
    ns.theta = sn[4];
    ns.vx    = sn[3] * cos(sn[6]);  /* V * cos(beta) */
    ns.vy    = sn[3] * sin(sn[6]);  /* V * sin(beta) */
    ns.omega = sn[5];

    return ns;
}

/*===========================================================================
 * Raceline Data
 *===========================================================================*/

typedef struct {
    double s, x, y, psi, kappa, vx, ax;
    double left_bound, right_bound;
} Waypoint_t;

static Waypoint_t raceline[MAX_WAYPOINTS];
static int raceline_count = 0;

static int load_raceline(void)
{
    const char *paths[] = {
        "../../f1tenth_planning/trajectories/Spielberg_raceline.csv",
        "../../../f1tenth_planning/trajectories/Spielberg_raceline.csv",
        "../../../../f1tenth_planning/trajectories/Spielberg_raceline.csv",
        "f1tenth_planning/trajectories/Spielberg_raceline.csv",
        NULL
    };
    FILE *f = NULL;
    for (int i = 0; paths[i]; i++) {
        f = fopen(paths[i], "r");
        if (f) { printf("[LOAD] %s\n", paths[i]); break; }
    }
    if (!f) { fprintf(stderr, "ERROR: Cannot open Spielberg_raceline.csv\n"); return 0; }

    char buf[512];
    while (fgets(buf, sizeof(buf), f)) {
        if (buf[0] == '#' || buf[0] == '\n') continue;
        if (raceline_count >= MAX_WAYPOINTS) break;
        Waypoint_t *wp = &raceline[raceline_count];
        int n = sscanf(buf, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                       &wp->s, &wp->x, &wp->y, &wp->psi,
                       &wp->kappa, &wp->vx, &wp->ax,
                       &wp->left_bound, &wp->right_bound);
        if (n >= 9) raceline_count++;
        else if (n >= 7) {
            wp->left_bound = 5.0;
            wp->right_bound = 5.0;
            raceline_count++;
        }
    }
    fclose(f);
    printf("[LOAD] %d waypoints (v: %.1f-%.1f m/s)\n",
           raceline_count, raceline[0].vx, raceline[raceline_count/2].vx);
    return raceline_count > 0;
}

/*===========================================================================
 * Helpers
 *===========================================================================*/

static double wrap_angle(double a)
{
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

/* Use macros from mpc_fpga_types.h: DOUBLE_TO_FP, FP_TO_DOUBLE */

static int last_closest = 0;

static int find_closest_waypoint(double px, double py, double heading)
{
    if (raceline_count == 0) return 0;
    int window = raceline_count / 4;
    if (window < 20) window = 20;
    int best = last_closest;
    double best_dist = 1e18;
    double dir_x = cos(heading), dir_y = sin(heading);

    for (int off = -window; off <= window; off++) {
        int idx = (last_closest + off) % raceline_count;
        if (idx < 0) idx += raceline_count;
        double dx = raceline[idx].x - px;
        double dy = raceline[idx].y - py;
        double d2 = dx * dx + dy * dy;
        double dot = dx * dir_x + dy * dir_y;
        if (dot < -0.5 && d2 > 0.25) continue;
        if (d2 < best_dist) { best_dist = d2; best = idx; }
    }
    last_closest = best;
    return best;
}

/*===========================================================================
 * Main Simulation
 *===========================================================================*/

static int tests_passed = 0, tests_failed = 0;

static void check(const char *name, int cond)
{
    if (cond) { tests_passed++; printf("  [PASS] %s\n", name); }
    else      { tests_failed++; printf("  [FAIL] %s\n", name); }
}

int main(void)
{
    printf("=== FPGA MPC Sim-Drive Test (Riccati-ADMM HLS, %.0fs at dt=%.3fs) ===\n\n",
           SIM_DURATION, SIM_DT);

    if (!load_raceline()) return 1;

    /* ===== Phase 1: Load trajectory into FPGA ===== */
    printf("  Loading %d waypoints into FPGA...\n", raceline_count);

    int dummy_steer, dummy_accel, dummy_status, dummy_iters;
    for (int i = 0; i < raceline_count; i++) {
        mpc_fpga_top(
            1, i,
            DOUBLE_TO_FP(raceline[i].x),
            DOUBLE_TO_FP(raceline[i].y),
            DOUBLE_TO_FP(raceline[i].psi),
            DOUBLE_TO_FP(raceline[i].vx),
            DOUBLE_TO_FP(raceline[i].kappa),
            DOUBLE_TO_FP(raceline[i].ax),
            DOUBLE_TO_FP(raceline[i].left_bound),
            DOUBLE_TO_FP(raceline[i].right_bound),
            0,
            0, 0, 0, 0, 0, 0, 0, 0,
            &dummy_steer, &dummy_accel, &dummy_status, &dummy_iters);
    }

    /* Finalize trajectory */
    mpc_fpga_top(
        2, 0, 0, 0, 0, 0, 0, 0, 0, 0, raceline_count,
        0, 0, 0, 0, 0, 0, 0, 0,
        &dummy_steer, &dummy_accel, &dummy_status, &dummy_iters);
    printf("  Finalized: status=%d, count=%d\n\n", dummy_status, dummy_iters);

    /* ===== Phase 2: Run simulation ===== */
    SimState_t state;
    state.x     = raceline[0].x;
    state.y     = raceline[0].y;
    state.theta = raceline[0].psi;
    state.vx    = 0.0;
    state.vy    = 0.0;
    state.omega = 0.0;

    /* Tracking metrics */
    double max_lat_err = 0, sum_lat_err = 0;
    double max_hdg_err = 0, sum_hdg_err = 0;
    double max_vel_err = 0, sum_vel_err = 0;
    int wall_collisions = 0;
    int solver_ok = 0, solver_calls = 0;
    double actual_steer = 0.0;
    double prev_steer = 0.0;
    int steer_reversals = 0;
    double max_steer_change = 0;
    double time_above_5ms = 0.0;
    double max_vx = 0.0;
    long total_iterations = 0;
    int max_iters_single = 0;

    /* Timing */
    struct timespec t0, t1;
    double total_solve_us = 0.0;
    double max_solve_us = 0.0;

    /* Current MPC output (held between MPC calls) */
    double cmd_steer = 0.0;
    double cmd_accel = 0.0;

    printf("  Step | Time  | vx    | v_ref | e_y   | e_psi | steer  | accel | iter | wp  | wall?\n");
    printf("  -----|-------|-------|-------|-------|-------|--------|-------|------|-----|------\n");

    for (int step = 0; step < SIM_STEPS; step++) {
        double t = step * SIM_DT;

        if (state.vx > max_vx) max_vx = state.vx;

        int closest = find_closest_waypoint(state.x, state.y, state.theta);

        /* Frenet error (for metrics only — FPGA computes its own) */
        double e_y = -(state.x - raceline[closest].x) * sin(raceline[closest].psi)
                     + (state.y - raceline[closest].y) * cos(raceline[closest].psi);
        double e_psi = wrap_angle(state.theta - raceline[closest].psi);

        /* Wall collision check — wall hit = crash */
        double left_wall = raceline[closest].left_bound;
        double right_wall = raceline[closest].right_bound;
        int wall_hit = 0;
        if (e_y > left_wall)  { wall_hit = 1;  wall_collisions++; }
        if (e_y < -right_wall){ wall_hit = -1; wall_collisions++; }
        if (wall_hit && state.vx > 1.0) {
            printf("\n  !!! WALL CRASH: e_y = %.3f m (bound: %.3f) at step %d (t=%.2fs, wp=%d, v=%.1f) !!!\n",
                   e_y, wall_hit > 0 ? left_wall : right_wall, step, t, closest, state.vx);
            break;
        }

        /* Call MPC at the configured interval (or at low speed for startup) */
        if (step % MPC_CALL_INTERVAL == 0) {
            if (fabs(state.vx) < MIN_SPEED_FOR_MPC) {
                /* Low-speed guard */
                cmd_steer = -0.5 * e_psi;
                if (cmd_steer > 0.2) cmd_steer = 0.2;
                if (cmd_steer < -0.2) cmd_steer = -0.2;
                cmd_accel = PHYSICAL_MAX_ACCEL;
                solver_ok++;
            } else {
                int out_steer_fp, out_accel_fp, out_status, out_iters;

                clock_gettime(CLOCK_MONOTONIC_RAW, &t0);

                mpc_fpga_top(
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    DOUBLE_TO_FP(state.x),
                    DOUBLE_TO_FP(state.y),
                    DOUBLE_TO_FP(state.theta),
                    DOUBLE_TO_FP(state.vx),
                    DOUBLE_TO_FP(state.vy),
                    DOUBLE_TO_FP(state.omega),
                    DOUBLE_TO_FP(actual_steer),
                    closest,
                    &out_steer_fp, &out_accel_fp,
                    &out_status, &out_iters);

                clock_gettime(CLOCK_MONOTONIC_RAW, &t1);
                double solve_us = (t1.tv_sec - t0.tv_sec) * 1e6
                                + (t1.tv_nsec - t0.tv_nsec) / 1e3;
                total_solve_us += solve_us;
                if (solve_us > max_solve_us) max_solve_us = solve_us;

                cmd_steer = FP_TO_DOUBLE(out_steer_fp);
                cmd_accel = FP_TO_DOUBLE(out_accel_fp);
                total_iterations += out_iters;
                if (out_iters > max_iters_single) max_iters_single = out_iters;

                if (out_status == 0 || out_status == 1)
                    solver_ok++;
            }
            solver_calls++;
        }

        /* Physical saturation */
        double steer = cmd_steer;
        double accel = cmd_accel;
        if (steer > MAX_STEERING) steer = MAX_STEERING;
        if (steer < -MAX_STEERING) steer = -MAX_STEERING;
        if (accel > PHYSICAL_MAX_ACCEL) accel = PHYSICAL_MAX_ACCEL;
        if (accel < -PHYSICAL_MAX_ACCEL) accel = -PHYSICAL_MAX_ACCEL;

        /* ST model handles servo dynamics internally via steering velocity.
         * actual_steer tracks the desired MPC command for the ST model.
         * We feed the REALIZED st_delta (from previous step) to MPC. */
        actual_steer = steer;

        /* Metrics */
        if (fabs(e_y) > max_lat_err) max_lat_err = fabs(e_y);
        sum_lat_err += fabs(e_y);
        if (fabs(e_psi) > max_hdg_err) max_hdg_err = fabs(e_psi);
        sum_hdg_err += fabs(e_psi);
        double vel_err = fabs(state.vx - raceline[closest].vx);
        if (vel_err > max_vel_err) max_vel_err = vel_err;
        sum_vel_err += vel_err;
        if (state.vx > 5.0) time_above_5ms += SIM_DT;

        double steer_change = actual_steer - prev_steer;
        if (fabs(steer_change) > fabs(max_steer_change)) max_steer_change = steer_change;
        if (step > 0 && actual_steer * prev_steer < 0 && fabs(steer_change) > 0.1)
            steer_reversals++;
        prev_steer = actual_steer;

        /* Print schedule */
        int print_row = (step < 40) || (step % 200 == 0) || wall_hit || (fabs(e_y) > 0.8);
        if (print_row) {
            printf("  %4d | %5.2f | %5.2f | %5.2f | %+.3f | %+.3f | %+.4f | %+.2f | %4d | %3d | %s\n",
                   step, t, state.vx, raceline[closest].vx, e_y, e_psi, actual_steer, accel,
                   (step % MPC_CALL_INTERVAL == 0) ? max_iters_single : -1,
                   closest,
                   wall_hit > 0 ? "LEFT!" : (wall_hit < 0 ? "RIGHT!" : ""));
        }

        /* Propagate vehicle (ST model with RK4) */
        state = propagate_vehicle(state, actual_steer, accel, SIM_DT);

        /* Update actual_steer from ST model's realized delta state */
        actual_steer = st_delta;

        /* Early termination on extreme deviation (safety net) */
        if (fabs(e_y) > 5.0) {
            printf("\n  !!! EXTREME: e_y = %.2f m at step %d (t=%.2fs, wp=%d) !!!\n",
                   e_y, step, t, closest);
            break;
        }
    }

    /* ===== Summary ===== */
    double avg_lat = sum_lat_err / SIM_STEPS;
    double avg_hdg = sum_hdg_err / SIM_STEPS;
    double avg_vel = sum_vel_err / SIM_STEPS;
    double avg_iters = (solver_calls > 0) ? (double)total_iterations / solver_calls : 0;
    double avg_solve = (solver_calls > 0) ? total_solve_us / solver_calls : 0;

    printf("\n  === Results (FPGA HLS Riccati-ADMM, %.0f seconds) ===\n", SIM_DURATION);
    printf("  Solver calls:       %d (success: %d, %.1f%%)\n",
           solver_calls, solver_ok, 100.0 * solver_ok / (solver_calls > 0 ? solver_calls : 1));
    printf("  Max velocity:       %.2f m/s\n", max_vx);
    printf("  Max lateral error:  %.3f m\n", max_lat_err);
    printf("  Avg lateral error:  %.3f m\n", avg_lat);
    printf("  Max heading error:  %.4f rad (%.1f deg)\n", max_hdg_err, max_hdg_err * 180 / M_PI);
    printf("  Avg heading error:  %.4f rad (%.1f deg)\n", avg_hdg, avg_hdg * 180 / M_PI);
    printf("  Max velocity error: %.2f m/s\n", max_vel_err);
    printf("  Avg velocity error: %.2f m/s\n", avg_vel);
    printf("  Max steer change:   %.4f rad/step\n", max_steer_change);
    printf("  Steer reversals:    %d\n", steer_reversals);
    printf("  Wall collisions:    %d\n", wall_collisions);
    printf("  Time above 5 m/s:   %.1f / %.1f s (%.0f%%)\n",
           time_above_5ms, SIM_DURATION, 100 * time_above_5ms / SIM_DURATION);
    printf("\n  --- Solver Performance ---\n");
    printf("  Total iterations:   %ld\n", total_iterations);
    printf("  Avg iterations/call: %.1f\n", avg_iters);
    printf("  Max iterations:     %d\n", max_iters_single);
    printf("  Avg solve time:     %.1f us (on this PC, not FPGA)\n", avg_solve);
    printf("  Max solve time:     %.1f us\n", max_solve_us);
    printf("  Est. FPGA time/call: %.1f us (avg_iters * 112us)\n", avg_iters * 112.0);
    printf("  Est. FPGA max time:  %.1f us (%d iters * 112us)\n",
           max_iters_single * 112.0, max_iters_single);
    printf("\n");

    /* Pass/fail criteria */
    check("No wall collisions", wall_collisions == 0);
    check("Max lateral error < 1.5 m", max_lat_err < 1.5);
    check("Avg lateral error < 0.5 m", avg_lat < 0.5);
    check("Avg heading error < 0.3 rad (17 deg)", avg_hdg < 0.3);
    check("Solver mostly succeeds (>80%)", solver_ok > solver_calls * 80 / 100);
    check("Reaches driving speed (>5 m/s for >50% of time)",
          time_above_5ms > SIM_DURATION * 0.5);

    printf("\n=== RESULTS: %d passed, %d failed ===\n", tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
