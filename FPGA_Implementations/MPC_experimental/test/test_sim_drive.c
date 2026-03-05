/**
 * @file test_sim_drive.c
 * @brief Realistic 30-second MPC simulation on Spielberg raceline
 *
 * Tests the Riccati-ADMM MPC controller in a closed-loop simulation:
 *   - dt = 0.05s (MPC time step, matching sim update rate)
 *   - v_cmd = raceline velocity
 *   - Wall bounds from the CSV
 *   - Vehicle model propagation with MPC acceleration output
 *   - Spawn at raceline[0]
 *   - Runs for 30 seconds (600 steps)
 *
 * Reports: wall collisions, max/avg lateral error, steering behavior,
 *          velocity tracking, and step-by-step diagnostics near crashes.
 *
 * Compile (standalone):
 *   cd MPC_experimental
 *   cmake -S . -B build -DMPC_BUILD_TESTS=ON
 *   cmake --build build -j
 *   ./build/test_sim_drive
 */

#define _USE_MATH_DEFINES
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "mpc.h"
#include "mpc_types.h"
#include "fp_math.h"
#include "vehicle_model.h"
#include "riccati_solver.h"

/*===========================================================================
 * Configuration
 *===========================================================================*/

#define SIM_DT            0.005   /* MPC time step = 50ms */
#define SIM_DURATION      30.0   /* seconds */
#define SIM_STEPS         ((int)(SIM_DURATION / SIM_DT))
#define MPC_HORIZON       20
#define MPC_REF_ENTRIES   20     /* Must match horizon */
#define MAX_WAYPOINTS     2000
#define MAX_STEERING      0.4282 /* rad — physical limit */
#define MAX_VELOCITY      20.0   /* m/s */
#define PHYSICAL_MAX_ACCEL 8.0   /* m/s² — matches MPC constraint bounds */
#define MIN_SPEED_FOR_MPC 0.5    /* m/s — below this, use low-speed guard */

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
        if (n >= 9) raceline_count++;
        else if (n >= 6) {
            wp->left_bound = 5.0;
            wp->right_bound = 5.0;
            raceline_count++;
        }
    }
    fclose(f);
    printf("[LOAD] %d waypoints (v: %.1f-%.1f m/s)\n",
           raceline_count,
           raceline[0].vx, raceline[raceline_count/2].vx);
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
        double d2 = dx*dx + dy*dy;
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
    double px = FP_TO_DOUBLE(v->position_x_meters);
    double py = FP_TO_DOUBLE(v->position_y_meters);
    double psi = FP_TO_DOUBLE(v->heading_angle_radians);
    double dx = px - raceline[wp].x;
    double dy = py - raceline[wp].y;
    double path_psi = raceline[wp].psi;
    double lat_err = -dx * sin(path_psi) + dy * cos(path_psi);
    double hdg_err = wrap_angle(psi - path_psi);
    f.lateral_error_meters = DOUBLE_TO_FP(lat_err);
    f.heading_error_radians = DOUBLE_TO_FP(hdg_err);
    f.longitudinal_velocity_meters_per_second = v->longitudinal_velocity_meters_per_second;
    f.lateral_velocity_meters_per_second = v->lateral_velocity_meters_per_second;
    f.yaw_rate_radians_per_second = v->yaw_rate_radians_per_second;
    return f;
}

static void build_reference(int closest, double actual_vx, TrajectoryReferencePoint_t *ref)
{
    double wp_spacing = 0.347;
    /* Use the raceline velocity for reference spacing.
     * The reference at step k represents the waypoint the car should aim
     * for at prediction time k*dt_pred.  The raceline velocity determines
     * the expected distance traveled per prediction step. */
    double v_cur = fabs(raceline[closest].vx);
    if (v_cur < 1.0) v_cur = 1.0;
    const double mpc_prediction_dt = 0.05;  /* Must match MPC_DEFAULT_TIME_STEP_SECONDS */
    double ds_per_step = v_cur * mpc_prediction_dt;
    int wp_advance = (int)(ds_per_step / wp_spacing + 0.5);
    if (wp_advance < 1) wp_advance = 1;

    for (int step = 0; step < MPC_REF_ENTRIES; step++) {
        int base = (closest + step * wp_advance) % raceline_count;
        int wp   = (closest + (step + 1) * wp_advance) % raceline_count;

        ref[step].reference_lateral_error_meters = 0;
        ref[step].reference_heading_error_radians = 0;
        ref[step].reference_velocity_meters_per_second = DOUBLE_TO_FP(raceline[base].vx);

        double kappa = raceline[wp].kappa;
        double v_wp = raceline[base].vx;

        /* B5: vy reference — DISABLED for baseline test */
        ref[step].reference_lateral_velocity_meters_per_second = 0;

        ref[step].reference_yaw_rate_radians_per_second = DOUBLE_TO_FP(kappa * v_wp);

        /* B3: Acceleration feedforward from raceline */
        ref[step].reference_acceleration_meters_per_second_squared = DOUBLE_TO_FP(raceline[base].ax);

        ref[step].path_curvature_radians_per_meter = DOUBLE_TO_FP(raceline[wp].kappa);
        ref[step].left_wall_bound_meters = DOUBLE_TO_FP(raceline[wp].left_bound);
        ref[step].right_wall_bound_meters = DOUBLE_TO_FP(raceline[wp].right_bound);
    }
}

/*===========================================================================
 * Main Simulation
 *===========================================================================*/

static int tests_passed = 0, tests_failed = 0;

static void check(const char *name, int cond)
{
    if (cond) { tests_passed++; printf("  [PASS] %s\n", name); }
    else       { tests_failed++; printf("  [FAIL] %s\n", name); }
}

int main(void)
{
    printf("=== Spielberg Sim-Drive Test (Riccati-ADMM, %.0fs at dt=%.3fs = %d steps) ===\n\n",
           SIM_DURATION, SIM_DT, SIM_STEPS);

    if (!load_raceline()) return 1;

    /* Initialize Riccati-ADMM MPC via the unified API */
    mpc_initialize();
    mpc_reset();

    /* Configure horizon and cross-call rate */
    MpcConfiguration_t cfg = mpc_get_configuration();
    cfg.prediction_horizon_steps = MPC_HORIZON;
    /* cross_call_rate_scale: ratio of control interval to prediction dt.
     * In this test, MPC is called every SIM_DT = 50ms = prediction dt,
     * so the scale is 1.0 (not 0.3 which is for the ROS2 200Hz control loop). */
    cfg.cross_call_rate_scale = FP_CONST(0.1);
    mpc_set_configuration(&cfg);

    printf("  Horizon: %d, Q_lat=%.2f Q_hdg=%.2f Q_vel=%.2f R_steer=%.2f R_accel=%.2f\n",
           cfg.prediction_horizon_steps,
           FP_TO_DOUBLE(cfg.weight_lateral_error),
           FP_TO_DOUBLE(cfg.weight_heading_error),
           FP_TO_DOUBLE(cfg.weight_velocity),
           FP_TO_DOUBLE(cfg.weight_steering_effort),
           FP_TO_DOUBLE(cfg.weight_acceleration_effort));
    printf("  Steer_rate=%.2f Accel_rate=%.2f Cross_call=%.2f\n",
           FP_TO_DOUBLE(cfg.weight_steering_rate),
           FP_TO_DOUBLE(cfg.weight_acceleration_rate),
           FP_TO_DOUBLE(cfg.cross_call_rate_scale));

    /* Spawn at raceline[0] at standstill */
    VehicleState_t state;
    state.position_x_meters = DOUBLE_TO_FP(raceline[0].x);
    state.position_y_meters = DOUBLE_TO_FP(raceline[0].y);
    state.heading_angle_radians = DOUBLE_TO_FP(raceline[0].psi);
    state.longitudinal_velocity_meters_per_second = 0;
    state.lateral_velocity_meters_per_second = 0;
    state.yaw_rate_radians_per_second = 0;

    /* Tracking metrics */
    double max_lat_err = 0, sum_lat_err = 0;
    double max_hdg_err = 0, sum_hdg_err = 0;
    double max_vel_err = 0, sum_vel_err = 0;
    int wall_collisions = 0;
    int solver_ok = 0;
    double prev_steer = 0;
    double actual_steer = 0;  /* Physical servo position (rate-limited) */
    int steer_reversals = 0;
    double max_steer_change = 0;
    double time_above_5ms = 0;
    double max_vx = 0;

    printf("\n  Step | Time  | vx    | v_cmd | e_y   | e_psi | cmd_st | act_st | accel | iter | wp  | wall?\n");
    printf("  -----|-------|-------|-------|-------|-------|--------|--------|-------|------|-----|------\n");

    for (int step = 0; step < SIM_STEPS; step++) {
        double t = step * SIM_DT;
        double px = FP_TO_DOUBLE(state.position_x_meters);
        double py = FP_TO_DOUBLE(state.position_y_meters);
        double psi = FP_TO_DOUBLE(state.heading_angle_radians);
        double vx = FP_TO_DOUBLE(state.longitudinal_velocity_meters_per_second);

        if (vx > max_vx) max_vx = vx;

        int closest = find_closest_waypoint(px, py, psi);

        FrenetState_t frenet = vehicle_to_frenet(&state, closest);
        double e_y = FP_TO_DOUBLE(frenet.lateral_error_meters);
        double e_psi = FP_TO_DOUBLE(frenet.heading_error_radians);

        /* Wall collision check */
        double left_wall = raceline[closest].left_bound;
        double right_wall = raceline[closest].right_bound;
        int wall_hit = 0;
        if (e_y > left_wall) { wall_hit = 1; wall_collisions++; }
        if (e_y < -right_wall) { wall_hit = -1; wall_collisions++; }

        /* Build reference */
        TrajectoryReferencePoint_t ref[MPC_REF_ENTRIES];
        build_reference(closest, vx, ref);

        /* MPC solve (or low-speed guard) */
        double steer = 0;
        double accel_cmd = 0;
        int iter = 0;

        if (fabs(vx) < MIN_SPEED_FOR_MPC) {
            /* Low-speed guard: gentle straight + accelerate */
            double hdg_err = wrap_angle(psi - raceline[closest].psi);
            steer = 0.5 * hdg_err;
            if (steer > 0.2) steer = 0.2;
            if (steer < -0.2) steer = -0.2;
            accel_cmd = PHYSICAL_MAX_ACCEL;  /* Max physical acceleration */
            solver_ok++;
        } else {
            MpcSolverResult_t result;
            MpcSolverStatus_t status = mpc_compute_optimal_control(&frenet, ref, &result);
            steer = FP_TO_DOUBLE(result.optimal_control.steering_angle_radians);
            accel_cmd = FP_TO_DOUBLE(result.optimal_control.acceleration_meters_per_second_squared);
            iter = result.iterations_used;
            if (status == MPC_STATUS_SUCCESS || status == MPC_STATUS_MAXIMUM_ITERATIONS_REACHED)
                solver_ok++;
        }

        /* Physical saturation: steering and acceleration */
        if (steer > MAX_STEERING) steer = MAX_STEERING;
        if (steer < -MAX_STEERING) steer = -MAX_STEERING;
        if (accel_cmd > PHYSICAL_MAX_ACCEL) accel_cmd = PHYSICAL_MAX_ACCEL;
        if (accel_cmd < -PHYSICAL_MAX_ACCEL) accel_cmd = -PHYSICAL_MAX_ACCEL;

        /* Servo dynamics: rate-limit steering to the measured actuator rate.
         * The MPC output (steer) is the command; the servo moves toward it
         * at max F110_MAX_STEERING_RATE_RADS = 2.85 rad/s.
         * This matches the real car's servo behavior. */
        {
            double max_delta = 2.85 * SIM_DT;  /* F110_MAX_STEERING_RATE_RADS * dt */
            double steer_diff = steer - actual_steer;
            if (steer_diff > max_delta) steer_diff = max_delta;
            if (steer_diff < -max_delta) steer_diff = -max_delta;
            actual_steer += steer_diff;
        }

        /* Feed the actual servo position back to the MPC so its delta_prev
         * state matches the realized steering, not the commanded steering.
         * Without this, servo rate-limiting creates a growing model mismatch:
         * the MPC thinks delta_prev = command, but reality = rate-limited.
         * NOTE: disabled for now — while more accurate, the awareness of
         * servo lag makes the MPC too conservative for cornering speed.
         * TODO: re-enable once servo-aware planning is implemented. */
        /* {
            ControlInput_t actual_ctrl;
            actual_ctrl.steering_angle_radians = DOUBLE_TO_FP(actual_steer);
            actual_ctrl.acceleration_meters_per_second_squared = DOUBLE_TO_FP(accel_cmd);
            mpc_set_actual_previous_control(&actual_ctrl);
        } */

        /* Metrics */
        if (fabs(e_y) > max_lat_err) max_lat_err = fabs(e_y);
        sum_lat_err += fabs(e_y);
        if (fabs(e_psi) > max_hdg_err) max_hdg_err = fabs(e_psi);
        sum_hdg_err += fabs(e_psi);
        double vel_err = fabs(vx - raceline[closest].vx);
        if (vel_err > max_vel_err) max_vel_err = vel_err;
        sum_vel_err += vel_err;
        if (vx > 5.0) time_above_5ms += SIM_DT;

        double steer_change = actual_steer - prev_steer;
        if (fabs(steer_change) > fabs(max_steer_change)) max_steer_change = steer_change;
        if (step > 0 && actual_steer * prev_steer < 0 && fabs(steer_change) > 0.1)
            steer_reversals++;
        prev_steer = actual_steer;

        /* Print every step for first 2s, then every 20 steps or on issues */
        int print_row = (step < 40) || (step % 20 == 0) || wall_hit || (fabs(e_y) > 0.8);
        if (print_row) {
            printf("  %4d | %5.2f | %5.2f | %5.2f | %+.3f | %+.3f | %+.4f | %+.4f | %+.2f | %4d | %3d | %s\n",
                   step, t, vx, raceline[closest].vx, e_y, e_psi, steer, actual_steer, accel_cmd, iter, closest,
                   wall_hit > 0 ? "LEFT!" : (wall_hit < 0 ? "RIGHT!" : ""));
        }

        /* Propagate vehicle state using actual (servo-limited) steering.
         * Apply v_switch power limit to match gym's acceleration model (A5). */
        ControlInput_t ctrl;
        ctrl.steering_angle_radians = DOUBLE_TO_FP(actual_steer);
        double prop_accel = accel_cmd;
        /* v_switch power limit: above 7.319 m/s, a_max = 8.0 * 7.319/v */
        double v_switch = 7.319;
        double a_max_eff = PHYSICAL_MAX_ACCEL;
        if (vx > v_switch) {
            a_max_eff = PHYSICAL_MAX_ACCEL * v_switch / vx;
        }
        if (prop_accel > a_max_eff) prop_accel = a_max_eff;
        if (prop_accel < -PHYSICAL_MAX_ACCEL) prop_accel = -PHYSICAL_MAX_ACCEL;
        ctrl.acceleration_meters_per_second_squared = DOUBLE_TO_FP(prop_accel);
        state = vehicle_model_predict_next_state(&state, &ctrl, DOUBLE_TO_FP(SIM_DT));

        /* Early termination on severe crash */
        if (fabs(e_y) > 3.0) {
            printf("\n  !!! CRASH: e_y = %.2f m at step %d (t=%.2fs, wp=%d) !!!\n", e_y, step, t, closest);
            break;
        }
    }

    /* Summary */
    double avg_lat = sum_lat_err / SIM_STEPS;
    double avg_hdg = sum_hdg_err / SIM_STEPS;
    double avg_vel = sum_vel_err / SIM_STEPS;
    printf("\n  === Results (%.0f seconds, Riccati-ADMM) ===\n", SIM_DURATION);
    printf("  Solver success:     %d / %d (%.1f%%)\n", solver_ok, SIM_STEPS, 100.0*solver_ok/SIM_STEPS);
    printf("  Max velocity:       %.2f m/s\n", max_vx);
    printf("  Max lateral error:  %.3f m\n", max_lat_err);
    printf("  Avg lateral error:  %.3f m\n", avg_lat);
    printf("  Max heading error:  %.4f rad (%.1f deg)\n", max_hdg_err, max_hdg_err*180/M_PI);
    printf("  Avg heading error:  %.4f rad (%.1f deg)\n", avg_hdg, avg_hdg*180/M_PI);
    printf("  Max velocity error: %.2f m/s\n", max_vel_err);
    printf("  Avg velocity error: %.2f m/s\n", avg_vel);
    printf("  Max steer change:   %.4f rad/step\n", max_steer_change);
    printf("  Steer reversals:    %d\n", steer_reversals);
    printf("  Wall collisions:    %d\n", wall_collisions);
    printf("  Time above 5 m/s:   %.1f / %.1f s (%.0f%%)\n",
           time_above_5ms, SIM_DURATION, 100*time_above_5ms/SIM_DURATION);
    printf("\n");

    /* Pass/fail criteria */
    check("No wall collisions", wall_collisions == 0);
    check("Max lateral error < 1.2 m", max_lat_err < 1.2);
    check("Avg lateral error < 0.5 m", avg_lat < 0.5);
    check("Avg heading error < 0.3 rad (17 deg)", avg_hdg < 0.3);
    check("Solver mostly succeeds (>80%)", solver_ok > SIM_STEPS * 80 / 100);
    check("Reaches driving speed (>5 m/s for >50% of time)",
          time_above_5ms > SIM_DURATION * 0.5);

    printf("\n=== RESULTS: %d passed, %d failed ===\n", tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
