/**
 * @file test_dynamic_raceline.c
 * @brief Dynamic raceline shift + wall tightening MPC stress test
 *
 * Variant of test_sim_drive.c that simulates overtaking scenarios:
 *   - Raceline shifts laterally mid-race (smooth ramps)
 *   - Wall constraints tighten at specific waypoints (nearby car)
 *   - Runs multiple horizon configurations via WALL_END
 *
 * Configurations are controlled via environment variables:
 *   WALL_END    — last horizon step with wall constraint (default 5)
 *   NO_SHIFT    — if set to "1", disable raceline shifts (baseline)
 *
 * Compile:
 *   cd MPC
 *   gcc -D_GNU_SOURCE -O3 -std=c99 -Wall -ffast-math \
 *       -Wno-unused-variable -Wno-unused-but-set-variable \
 *       -Iinclude test/test_dynamic_raceline.c src/mpc_riccati.c \
 *       src/riccati_solver.c src/vehicle_model.c src/fp_math.c \
 *       -o test_dynamic -lm
 *
 * Run:
 *   WALL_END=5  ./test_dynamic
 *   WALL_END=20 ./test_dynamic
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

#include "mpc.h"
#include "mpc_types.h"
#include "vehicle_model.h"
#include "riccati_solver.h"

/* Portability: some systems (e.g. Windows) don't define CLOCK_MONOTONIC_RAW.
 * Provide a safe fallback to CLOCK_MONOTONIC when RAW is unavailable. */
#ifndef CLOCK_MONOTONIC_RAW
#ifdef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC_RAW CLOCK_MONOTONIC
#endif
#endif

/*===========================================================================
 * Configuration
 *===========================================================================*/

#define SIM_DT_DEFAULT    0.005   /* 5ms (200 Hz) */
#define SIM_DURATION      60.0   /* seconds */
#define MPC_HORIZON       20
#define MPC_REF_ENTRIES   20
#define MAX_WAYPOINTS     2000
#define MAX_STEERING      0.4189  /* rad (calibrated with polynomial servo correction) */
#define MAX_VELOCITY      20.0    /* m/s */
#define PHYSICAL_MAX_ACCEL 7.31  /* m/s² */
#define MIN_SPEED_FOR_MPC 0.5    /* m/s */
#define MPC_CALL_INTERVAL 1

/* Minimum wall distance after all modifications (safety clamp) */
#define MIN_WALL_DISTANCE 0.15   /* meters */

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
        "f1tenth_planning/trajectories/Spielberg_raceline.csv",
        "../f1tenth_planning/trajectories/Spielberg_raceline.csv",
        "../../../../f1tenth_planning/trajectories/Spielberg_raceline.csv",
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
        if (n == 9) {
            raceline_count++;
        } else if (n > 0) {
            fprintf(stderr, "ERROR: Raceline rows must include wall bounds (9 columns).\n");
            fclose(f);
            return 0;
        }
    }
    fclose(f);
    printf("[LOAD] %d waypoints (v: %.1f-%.1f m/s)\n",
           raceline_count,
           raceline[0].vx, raceline[raceline_count/2].vx);
    return raceline_count > 0;
}

/*===========================================================================
 * Dynamic Raceline Offset
 *===========================================================================
 * Returns lateral offset in meters at the given simulation time.
 * Positive = shift left, Negative = shift right.
 *
 * Schedule:
 *  t=10-15s: ramp left  to +0.3m  (approaching car, overtake on right)
 *  t=15-25s: hold +0.3m
 *  t=25-30s: ramp back to center
 *  t=40-45s: ramp right to -0.3m  (overtaking on left)
 *  t=45-50s: hold -0.3m
 *  t=50-55s: ramp back to center
 */
static double shift_magnitude = 0.3;  /* set via SHIFT_MAG env var */

static double get_raceline_offset(double time)
{
    double m = shift_magnitude;
    if (time >= 10.0 && time < 15.0)
        return  m * (time - 10.0) / 5.0;   /* ramp up left */
    if (time >= 15.0 && time < 25.0)
        return  m;                           /* hold left */
    if (time >= 25.0 && time < 30.0)
        return  m * (30.0 - time) / 5.0;    /* ramp down */
    if (time >= 40.0 && time < 45.0)
        return -m * (time - 40.0) / 5.0;    /* ramp up right */
    if (time >= 45.0 && time < 50.0)
        return -m;                           /* hold right */
    if (time >= 50.0 && time < 55.0)
        return -m * (55.0 - time) / 5.0;    /* ramp down */
    return 0.0;                                /* centered */
}

/* Returns a human-readable label for the current offset phase */
static const char *get_offset_phase(double time)
{
    if (time >= 10.0 && time < 15.0) return "RAMP LEFT";
    if (time >= 15.0 && time < 25.0) return "HOLD LEFT (+0.3m)";
    if (time >= 25.0 && time < 30.0) return "RAMP CENTER";
    if (time >= 40.0 && time < 45.0) return "RAMP RIGHT";
    if (time >= 45.0 && time < 50.0) return "HOLD RIGHT (-0.3m)";
    if (time >= 50.0 && time < 55.0) return "RAMP CENTER";
    return "CENTERED";
}

/*===========================================================================
 * Obstacle / Wall-Tightening Function
 *===========================================================================
 * Simulates a nearby car at certain waypoint ranges:
 *   wp 200-300: reduce RIGHT wall by 0.4m (car on the right)
 *   wp 700-800: reduce LEFT  wall by 0.3m (car on the left)
 *
 * Returns: adjustment (always >= 0) to subtract from the wall bound.
 */
static double obstacle_right_wall_reduction(int wp_idx)
{
    if (wp_idx >= 200 && wp_idx <= 300) return 0.4;
    return 0.0;
}

static double obstacle_left_wall_reduction(int wp_idx)
{
    if (wp_idx >= 700 && wp_idx <= 800) return 0.3;
    return 0.0;
}

/*===========================================================================
 * Helpers
 *===========================================================================*/

static double wrap_angle(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

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

static FrenetState_t vehicle_to_frenet(const VehicleState_t *v, int wp)
{
    FrenetState_t f;
    double px  = (double)(v->pos_x);
    double py  = (double)(v->pos_y);
    double psi = (double)(v->heading);
    double dx  = px - raceline[wp].x;
    double dy  = py - raceline[wp].y;
    double path_psi = raceline[wp].psi;
    double lat_err  = -dx * sin(path_psi) + dy * cos(path_psi);
    double hdg_err  = wrap_angle(psi - path_psi);
    f.flat_error                      = (float)(lat_err);
    f.fhead_error                     = (float)(hdg_err);
    f.flong_vel   = v->long_vel;
    f.flat_vel        = v->lat_vel;
    f.fyaw_rate               = v->yaw_rate;
    return f;
}

/*===========================================================================
 * Build reference with dynamic offset + obstacle wall tightening
 *===========================================================================*/

static void build_reference_dynamic(int closest, double actual_vx,
                                    double lateral_offset,
                                    TrajectoryReferencePoint_t *ref)
{
    double wp_spacing = 0.347;
    double v_cur = fabs(raceline[closest].vx);
    if (v_cur < 1.0) v_cur = 1.0;
    const double mpc_prediction_dt = 0.05;
    double ds_per_step = v_cur * mpc_prediction_dt;
    int wp_advance = (int)(ds_per_step / wp_spacing + 0.5);
    if (wp_advance < 1) wp_advance = 1;

    for (int step = 0; step < MPC_REF_ENTRIES; step++) {
        int base = (closest + step * wp_advance) % raceline_count;
        int wp   = (closest + (step + 1) * wp_advance) % raceline_count;

        /* Reference lateral error = the offset (shift the target line) */
        ref[step].reference_lateral_error    = (float)(lateral_offset);
        ref[step].reference_heading_error   = 0;
        ref[step].reference_velocity =
            (float)(raceline[base].vx);
        ref[step].reference_lateral_velocity = 0;

        double kappa = raceline[wp].kappa;
        double v_wp  = raceline[base].vx;
        ref[step].reference_yaw_rate =
            (float)(kappa * v_wp);
        ref[step].path_curvature =
            (float)(raceline[wp].kappa);

        /* Start from CSV wall bounds */
        double lw = raceline[wp].left_bound;
        double rw = raceline[wp].right_bound;

        /* 1) Adjust for raceline lateral offset.
         *    Positive offset = shifted left → left wall shrinks, right grows */
        lw -= lateral_offset;
        rw += lateral_offset;

        /* 2) Apply obstacle tightening at specific waypoints */
        lw -= obstacle_left_wall_reduction(wp);
        rw -= obstacle_right_wall_reduction(wp);

        /* 3) Clamp to minimum */
        if (lw < MIN_WALL_DISTANCE) lw = MIN_WALL_DISTANCE;
        if (rw < MIN_WALL_DISTANCE) rw = MIN_WALL_DISTANCE;

        ref[step].left_wall_bound  = (float)(lw);
        ref[step].right_wall_bound = (float)(rw);
    }
}

/*===========================================================================
 * Main Simulation
 *===========================================================================*/

int main(void)
{
    /* ---- Read configuration from environment ---- */
    const char *dt_env    = getenv("SIM_DT");
    const double SIM_DT   = dt_env ? atof(dt_env) : SIM_DT_DEFAULT;
    const int SIM_STEPS   = (int)(SIM_DURATION / SIM_DT);
    const double mpc_prediction_dt = 0.05;
    const double cross_scale = SIM_DT / mpc_prediction_dt;

    const char *wall_end_env  = getenv("WALL_END");
    int cfg_wall_end    = wall_end_env  ? atoi(wall_end_env)  : 5;

    const char *no_shift_env = getenv("NO_SHIFT");
    int shift_enabled = !(no_shift_env && atoi(no_shift_env));

    const char *shift_mag_env = getenv("SHIFT_MAG");
    if (shift_mag_env) shift_magnitude = atof(shift_mag_env);

    printf("============================================================\n");
    printf("  Dynamic Raceline + Wall Tightening MPC Test\n");
    printf("============================================================\n");
    printf("  Duration:    %.0fs at dt=%.4fs (%d steps, %.0fHz)\n",
           SIM_DURATION, SIM_DT, SIM_STEPS, 1.0 / SIM_DT);
    printf("  WALL_END:    %d\n", cfg_wall_end);
    printf("  Shift magnitude: %.2fm\n", shift_magnitude);
    printf("  Raceline shift: %s\n", shift_enabled ? "ENABLED" : "DISABLED");
    printf("  Obstacle zones: wp 200-300 (right -0.4m), wp 700-800 (left -0.3m)\n");
    printf("============================================================\n\n");

    if (!load_raceline()) return 1;

    /* ---- Initialize MPC ---- */
    mpc_initialize();
    mpc_reset();

    MpcConfiguration_t cfg = mpc_get_configuration();
    cfg.prediction_horizon_steps = MPC_HORIZON;
    cfg.cross_call_rate_scale = (float)(cross_scale);

    /* Tuned weights (same as test_sim_drive defaults, overridable) */
    const char *env;
    cfg.weight_lateral_error       = (float)((env = getenv("Q_LAT"))       ? atof(env) : 125.0);
    cfg.weight_heading_error       = (float)((env = getenv("Q_HDG"))       ? atof(env) : 300.0);
    cfg.weight_velocity            = (float)((env = getenv("Q_VEL"))       ? atof(env) : 30.0);
    cfg.weight_lateral_velocity    = (float)((env = getenv("Q_LAT_VEL"))   ? atof(env) : 60.0);
    cfg.weight_yaw_rate            = (float)((env = getenv("Q_YAW"))       ? atof(env) : 20.0);
    cfg.weight_steering_effort     = (float)((env = getenv("R_STEER"))     ? atof(env) : 0.35);
    cfg.weight_acceleration_effort = (float)((env = getenv("R_ACCEL"))     ? atof(env) : 0.01);
    cfg.weight_steering_rate       = (float)((env = getenv("W_JERK"))      ? atof(env) : 0.5);
    cfg.weight_acceleration_rate   = (float)((env = getenv("W_ACCEL_RATE"))? atof(env) : 0.01);
    mpc_set_configuration(&cfg);

    printf("  Weights: Q_lat=%.1f Q_hdg=%.1f Q_vel=%.1f R_steer=%.2f R_accel=%.2f\n",
           (double)(cfg.weight_lateral_error),
           (double)(cfg.weight_heading_error),
           (double)(cfg.weight_velocity),
           (double)(cfg.weight_steering_effort),
           (double)(cfg.weight_acceleration_effort));

    /* ---- Spawn at raceline[0] ---- */
    VehicleState_t state;
    state.pos_x                       = (float)(raceline[0].x);
    state.pos_y                       = (float)(raceline[0].y);
    state.heading                    = (float)(raceline[0].psi);
    state.long_vel  = 0;
    state.lat_vel       = 0;
    state.yaw_rate              = 0;

    /* ---- Tracking metrics ---- */
    double max_lat_err = 0, sum_lat_err = 0;
    double max_hdg_err = 0, sum_hdg_err = 0;
    double max_vel_err = 0, sum_vel_err = 0;
    int    wall_collisions = 0;
    int    solver_ok = 0, solver_calls = 0;
    int    convergence_failures = 0;
    double prev_steer = 0;
    double actual_steer = 0;
    int    steer_reversals = 0;
    double max_steer_change = 0;
    double time_above_5ms = 0;
    double max_vx = 0;
    long   total_iterations = 0;
    int    max_iter_single = 0;
    double total_solve_us = 0.0, max_solve_us = 0.0;
    struct timespec ts0, ts1;

    double cmd_steer = 0.0;
    double cmd_accel = 0.0;
    int    completed_steps = 0;

    /* Track phase transitions for event logging */
    const char *prev_phase = "CENTERED";

    printf("\n  Step | Time  | vx    | e_y   | e_psi | steer  | accel | iter | wp  | offset | phase\n");
    printf("  -----|-------|-------|-------|-------|--------|-------|------|-----|--------|------\n");

    for (int step = 0; step < SIM_STEPS; step++) {
        double t  = step * SIM_DT;
        double px  = (double)(state.pos_x);
        double py  = (double)(state.pos_y);
        double psi = (double)(state.heading);
        double vx  = (double)(state.long_vel);

        if (vx > max_vx) max_vx = vx;

        int closest = find_closest_waypoint(px, py, psi);

        FrenetState_t frenet = vehicle_to_frenet(&state, closest);
        double e_y   = (double)(frenet.flat_error);
        double e_psi = (double)(frenet.fhead_error);

        /* Compute current lateral offset */
        double lateral_offset = shift_enabled ? get_raceline_offset(t) : 0.0;
        const char *phase = shift_enabled ? get_offset_phase(t) : "DISABLED";

        /* Log phase transitions */
        if (strcmp(phase, prev_phase) != 0) {
            printf("\n  >>> [t=%.2f] Phase change: %s -> %s (offset=%.3fm)\n\n",
                   t, prev_phase, phase, lateral_offset);
            prev_phase = phase;
        }

        /* Wall collision check (use ORIGINAL wall bounds for collision,
         * since that's where the physical walls actually are) */
        double left_wall  = raceline[closest].left_bound;
        double right_wall = raceline[closest].right_bound;
        int wall_hit = 0;
        if (e_y > left_wall)   { wall_hit =  1; wall_collisions++; }
        if (e_y < -right_wall) { wall_hit = -1; wall_collisions++; }
        if (wall_hit && vx > 1.0) {
            printf("\n  !!! WALL CRASH: e_y=%.3f (bound: %.3f) step=%d t=%.2fs wp=%d v=%.1f !!!\n",
                   e_y, wall_hit > 0 ? left_wall : right_wall,
                   step, t, closest, vx);
            break;
        }

        /* ---- MPC control ---- */
        double steer    = cmd_steer;
        double accel_cmd = cmd_accel;
        int    iter = 0;

        /* Feed realized steering to MPC */
        {
            ControlInput_t actual_ctrl;
            actual_ctrl.steer_ang = (float)(actual_steer);
            actual_ctrl.long_acc = (float)(cmd_accel);
            mpc_set_actual_previous_control(&actual_ctrl);
        }

        if (step % MPC_CALL_INTERVAL == 0) {
            TrajectoryReferencePoint_t ref[MPC_REF_ENTRIES];
            build_reference_dynamic(closest, vx, lateral_offset, ref);

            if (fabs(vx) < MIN_SPEED_FOR_MPC) {
                double hdg_err = wrap_angle(psi - raceline[closest].psi);
                steer = 0.5 * hdg_err;
                if (steer >  0.2) steer =  0.2;
                if (steer < -0.2) steer = -0.2;
                accel_cmd = PHYSICAL_MAX_ACCEL;
                solver_ok++;
            } else {
                MpcSolverResult_t result;
                clock_gettime(CLOCK_MONOTONIC_RAW, &ts0);
                MpcSolverStatus_t status =
                    mpc_compute_optimal_control(&frenet, ref, &result);
                clock_gettime(CLOCK_MONOTONIC_RAW, &ts1);

                double solve_us = (ts1.tv_sec  - ts0.tv_sec) * 1e6
                                + (ts1.tv_nsec - ts0.tv_nsec) / 1e3;
                total_solve_us += solve_us;
                if (solve_us > max_solve_us) max_solve_us = solve_us;

                steer     = (double)(result.optimal_control.steer_ang);
                accel_cmd = (double)(
                    result.optimal_control.long_acc);
                iter = result.iterations_used;
                total_iterations += iter;
                if (iter > max_iter_single) max_iter_single = iter;

                if (status == MPC_STATUS_SUCCESS ||
                    status == MPC_STATUS_MAXIMUM_ITERATIONS_REACHED)
                    solver_ok++;
                if (status == MPC_STATUS_INFEASIBLE ||
                    status == MPC_STATUS_ERROR)
                    convergence_failures++;
            }
            solver_calls++;
            cmd_steer = steer;
            cmd_accel = accel_cmd;
        }

        /* Physical saturation */
        if (steer >  MAX_STEERING) steer =  MAX_STEERING;
        if (steer < -MAX_STEERING) steer = -MAX_STEERING;
        if (accel_cmd >  PHYSICAL_MAX_ACCEL) accel_cmd =  PHYSICAL_MAX_ACCEL;
        if (accel_cmd < -PHYSICAL_MAX_ACCEL) accel_cmd = -PHYSICAL_MAX_ACCEL;

        actual_steer = steer;

        /* ---- Metrics ---- */
        /* For lateral error metric, measure error relative to the SHIFTED
         * raceline (which is the MPC target) */
        double e_y_from_target = e_y - lateral_offset;
        if (fabs(e_y_from_target) > max_lat_err) max_lat_err = fabs(e_y_from_target);
        sum_lat_err += fabs(e_y_from_target);
        if (fabs(e_psi) > max_hdg_err) max_hdg_err = fabs(e_psi);
        sum_hdg_err += fabs(e_psi);
        double vel_err = fabs(vx - raceline[closest].vx);
        if (vel_err > max_vel_err) max_vel_err = vel_err;
        sum_vel_err += vel_err;
        if (vx > 5.0) time_above_5ms += SIM_DT;

        double steer_change = actual_steer - prev_steer;
        if (fabs(steer_change) > fabs(max_steer_change))
            max_steer_change = steer_change;
        if (step > 0 && actual_steer * prev_steer < 0 &&
            fabs(steer_change) > 0.1)
            steer_reversals++;
        prev_steer = actual_steer;

        /* Print: first 2s, then every 100 steps, phase transitions,
         * obstacles, or trouble */
        int in_obstacle_zone = (closest >= 200 && closest <= 300) ||
                               (closest >= 700 && closest <= 800);
        int print_row = (step < 40) || (step % 100 == 0) || wall_hit ||
                        (fabs(e_y_from_target) > 0.6) || in_obstacle_zone;
        /* Thin out obstacle zone printing to every 10th step */
        if (in_obstacle_zone && step % 10 != 0 && step >= 40)
            print_row = 0;

        if (print_row) {
            printf("  %4d | %5.2f | %5.2f | %+.3f | %+.3f | %+.4f | %+.2f | %4d | %3d | %+.3f | %s%s\n",
                   step, t, vx, e_y, e_psi, steer, accel_cmd, iter, closest,
                   lateral_offset, phase,
                   in_obstacle_zone ? " [OBS]" : "");
        }

        completed_steps = step + 1;

        /* ---- Vehicle propagation (ST model with RK4) ---- */
        {
            static const double mu = 0.745, mass = 3.314, Iz = 0.035;
            static const double C_Sf = 4.297, C_Sr = 3.473;
            static const double lf = 0.166, lr = 0.16, h_cg = 0.0703;
            static const double g_acc = 9.81;
            static const double sv_max = 2.8492;
            static const double s_max  = 0.4189;
            static const double v_switch = 7.319;
            static const double v_min = 0.0, v_max = 20.0;
            static const double lwb = 0.166 + 0.16;

            static double st_delta = 0.0;
            static double st_V = 0.0;
            static double st_psi_dot = 0.0;
            static double st_beta = 0.0;
            static int    st_initialized = 0;

            if (!st_initialized) {
                st_delta = actual_steer;
                st_V     = vx;
                st_psi_dot = 0.0;
                st_beta    = 0.0;
                st_initialized = 1;
            }

            /* Steering constraint */
            double steer_vel;
            {
                double diff = actual_steer - st_delta;
                if (diff > 0) steer_vel = sv_max;
                else if (diff < 0) steer_vel = -sv_max;
                else steer_vel = 0.0;
                if (fabs(diff) < sv_max * SIM_DT)
                    steer_vel = diff / SIM_DT;
                if (st_delta >=  s_max && steer_vel > 0) steer_vel = 0.0;
                if (st_delta <= -s_max && steer_vel < 0) steer_vel = 0.0;
            }

            /* Acceleration constraint */
            double accl = accel_cmd;
            if (st_V > v_switch) {
                double a_max_eff = PHYSICAL_MAX_ACCEL * v_switch / st_V;
                if (accl > a_max_eff) accl = a_max_eff;
            }
            if (accl >  PHYSICAL_MAX_ACCEL) accl =  PHYSICAL_MAX_ACCEL;
            if (accl < -PHYSICAL_MAX_ACCEL) accl = -PHYSICAL_MAX_ACCEL;
            if (st_V <= v_min && accl < 0) accl = 0.0;
            if (st_V >= v_max && accl > 0) accl = 0.0;

            /* ST dynamics macro (identical to test_sim_drive.c) */
            #define ST_DYNAMICS(X, Y, DELTA, V, PSI, PSI_DOT_VAL, BETA, SV, ACCL, \
                               dX, dY, dDELTA, dV, dPSI, dPSI_DOT, dBETA) \
            do { \
                if ((V) < 0.5) { \
                    double beta_hat = atan(tan(DELTA) * lr / lwb); \
                    double beta_dot = (1.0 / (1.0 + pow(tan(DELTA) * lr / lwb, 2.0))) \
                                    * (lr / (lwb * pow(cos(DELTA), 2.0))) * (SV); \
                    (dX) = (V) * cos((PSI) + beta_hat); \
                    (dY) = (V) * sin((PSI) + beta_hat); \
                    (dDELTA) = (SV); \
                    (dV) = (ACCL); \
                    (dPSI) = (V) * cos(beta_hat) * tan(DELTA) / lwb; \
                    (dPSI_DOT) = (1.0 / lwb) * ( \
                        (ACCL) * cos(BETA) * tan(DELTA) \
                        - (V) * sin(BETA) * tan(DELTA) * beta_dot \
                        + ((V) * cos(BETA) * (SV)) / pow(cos(DELTA), 2.0)); \
                    (dBETA) = beta_dot; \
                } else { \
                    double vx_ = (V) * cos(BETA); \
                    double vy_ = (V) * sin(BETA); \
                    double vx_safe_ = (vx_ > 0.5) ? vx_ : 0.5; \
                    double Fzf_ = mass * (g_acc * lr - (ACCL) * h_cg) / lwb; \
                    double Fzr_ = mass * (g_acc * lf + (ACCL) * h_cg) / lwb; \
                    double alpha_f_ = (DELTA) - atan2(vy_ + lf * (PSI_DOT_VAL), vx_safe_); \
                    double alpha_r_ = -atan2(vy_ - lr * (PSI_DOT_VAL), vx_safe_); \
                    double Fyf_ = mu * C_Sf * alpha_f_ * Fzf_; \
                    double Fyr_ = mu * C_Sr * alpha_r_ * Fzr_; \
                    double Fx_  = mass * (ACCL); \
                    double cos_delta_ = cos(DELTA); \
                    double sin_delta_ = sin(DELTA); \
                    double dvx_dt_ = (Fx_ - Fyf_ * sin_delta_ \
                                      + mass * vy_ * (PSI_DOT_VAL)) / mass; \
                    double dvy_dt_ = (Fyf_ * cos_delta_ + Fyr_ \
                                      - mass * vx_ * (PSI_DOT_VAL)) / mass; \
                    double V_safe_ = ((V) > 0.001) ? (V) : 0.001; \
                    double V_sq_   = (V) * (V); \
                    if (V_sq_ < 0.001) V_sq_ = 0.001; \
                    (dX) = (V) * cos((PSI) + (BETA)); \
                    (dY) = (V) * sin((PSI) + (BETA)); \
                    (dDELTA) = (SV); \
                    (dV) = (vx_ * dvx_dt_ + vy_ * dvy_dt_) / V_safe_; \
                    (dPSI) = (PSI_DOT_VAL); \
                    (dPSI_DOT) = (lf * Fyf_ * cos_delta_ - lr * Fyr_) / Iz; \
                    (dBETA) = (vx_ * dvy_dt_ - vy_ * dvx_dt_) / V_sq_; \
                } \
            } while(0)

            /* RK4 integration */
            double k1[7], k2[7], k3[7], k4[7];
            double s0[7] = {px, py, st_delta, st_V, psi, st_psi_dot, st_beta};

            ST_DYNAMICS(s0[0], s0[1], s0[2], s0[3], s0[4], s0[5], s0[6],
                        steer_vel, accl,
                        k1[0], k1[1], k1[2], k1[3], k1[4], k1[5], k1[6]);
            {
                double s1[7];
                for (int i = 0; i < 7; i++) s1[i] = s0[i] + 0.5 * SIM_DT * k1[i];
                ST_DYNAMICS(s1[0], s1[1], s1[2], s1[3], s1[4], s1[5], s1[6],
                            steer_vel, accl,
                            k2[0], k2[1], k2[2], k2[3], k2[4], k2[5], k2[6]);
            }
            {
                double s2[7];
                for (int i = 0; i < 7; i++) s2[i] = s0[i] + 0.5 * SIM_DT * k2[i];
                ST_DYNAMICS(s2[0], s2[1], s2[2], s2[3], s2[4], s2[5], s2[6],
                            steer_vel, accl,
                            k3[0], k3[1], k3[2], k3[3], k3[4], k3[5], k3[6]);
            }
            {
                double s3[7];
                for (int i = 0; i < 7; i++) s3[i] = s0[i] + SIM_DT * k3[i];
                ST_DYNAMICS(s3[0], s3[1], s3[2], s3[3], s3[4], s3[5], s3[6],
                            steer_vel, accl,
                            k4[0], k4[1], k4[2], k4[3], k4[4], k4[5], k4[6]);
            }

            double sn[7];
            for (int i = 0; i < 7; i++)
                sn[i] = s0[i] + (SIM_DT / 6.0) *
                         (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);

            if (sn[2] >  s_max) sn[2] =  s_max;
            if (sn[2] < -s_max) sn[2] = -s_max;
            if (sn[3] < v_min)  sn[3] = v_min;
            if (sn[3] > v_max)  sn[3] = v_max;
            sn[4] = wrap_angle(sn[4]);

            st_delta   = sn[2];
            st_V       = sn[3];
            st_psi_dot = sn[5];
            st_beta    = sn[6];

            state.pos_x =
                (float)(sn[0]);
            state.pos_y =
                (float)(sn[1]);
            state.heading =
                (float)(sn[4]);
            state.long_vel =
                (float)(sn[3] * cos(sn[6]));
            state.lat_vel =
                (float)(sn[3] * sin(sn[6]));
            state.yaw_rate =
                (float)(sn[5]);

            actual_steer = st_delta;

            #undef ST_DYNAMICS
        }

        /* Early termination */
        if (fabs(e_y) > 3.0) {
            printf("\n  !!! CRASH: e_y=%.2f at step %d (t=%.2fs wp=%d) !!!\n",
                   e_y, step, t, closest);
            break;
        }
    }

    /* ================================================================
     * Summary Report
     * ================================================================ */
    int actual_steps = completed_steps > 0 ? completed_steps : 1;
    double avg_lat = sum_lat_err / actual_steps;
    double avg_hdg = sum_hdg_err / actual_steps;
    double avg_vel = sum_vel_err / actual_steps;
    double avg_iters = (solver_calls > 0) ?
                       (double)total_iterations / solver_calls : 0;
    double avg_solve = (solver_calls > 0) ?
                       total_solve_us / solver_calls : 0;

    printf("\n");
    printf("============================================================\n");
        printf("  Results: WALL_END=%d  Shift=%s\n",
            cfg_wall_end,
           shift_enabled ? "ON" : "OFF");
    printf("============================================================\n");
    printf("  Completed:          %d / %d steps (%.1fs / %.1fs)\n",
           completed_steps, SIM_STEPS,
           completed_steps * SIM_DT, SIM_DURATION);
    printf("  Solver success:     %d / %d (%.1f%%)\n",
           solver_ok, solver_calls,
           100.0 * solver_ok / (solver_calls > 0 ? solver_calls : 1));
    printf("  Convergence fails:  %d\n", convergence_failures);
    printf("  Wall collisions:    %d\n", wall_collisions);
    printf("  Max velocity:       %.2f m/s\n", max_vx);
    printf("  Max lateral error:  %.3f m  (relative to shifted raceline)\n",
           max_lat_err);
    printf("  Avg lateral error:  %.3f m\n", avg_lat);
    printf("  Max heading error:  %.4f rad (%.1f deg)\n",
           max_hdg_err, max_hdg_err * 180.0 / M_PI);
    printf("  Avg heading error:  %.4f rad\n", avg_hdg);
    printf("  Max velocity error: %.2f m/s\n", max_vel_err);
    printf("  Avg velocity error: %.2f m/s\n", avg_vel);
    printf("  Max steer change:   %.4f rad/step\n", max_steer_change);
    printf("  Steer reversals:    %d\n", steer_reversals);
    printf("  Time above 5 m/s:   %.1f / %.1fs (%.0f%%)\n",
           time_above_5ms, SIM_DURATION,
           100.0 * time_above_5ms / SIM_DURATION);
    printf("\n  --- Solver Performance ---\n");
    printf("  Total iterations:    %ld\n", total_iterations);
    printf("  Avg iterations/call: %.1f\n", avg_iters);
    printf("  Max iterations:      %d\n", max_iter_single);
    printf("  Avg solve time:      %.1f us\n", avg_solve);
    printf("  Max solve time:      %.1f us\n", max_solve_us);
    printf("  Total solve time:    %.1f ms\n", total_solve_us / 1000.0);

    /* Machine-readable CSV line */
        printf("\nCSV_DYNAMIC,%d,%s,%d,%d,%.4f,%.4f,%.4f,%.4f,%.2f,%.1f,%d,%.1f,%.1f,%d\n",
            cfg_wall_end,
           shift_enabled ? "ON" : "OFF",
           wall_collisions, convergence_failures,
           max_lat_err, avg_lat, max_hdg_err, avg_hdg,
           max_vx, avg_iters, max_iter_single,
           avg_solve, max_solve_us, completed_steps);

    printf("\n============================================================\n");
    int crashed = (completed_steps < SIM_STEPS);
    if (crashed)
        printf("  STATUS: CRASHED at step %d (%.2fs)\n",
               completed_steps, completed_steps * SIM_DT);
    else if (wall_collisions > 0)
        printf("  STATUS: COMPLETED WITH %d WALL COLLISIONS\n",
               wall_collisions);
    else
        printf("  STATUS: COMPLETED SUCCESSFULLY — no crashes, no wall hits\n");
    printf("============================================================\n");

    return crashed ? 2 : (wall_collisions > 0 ? 1 : 0);
}
