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
#include <time.h>

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

#define SIM_DT_DEFAULT    0.005   /* Simulation time step = 5ms (200Hz) */
#define SIM_DURATION      60.0   /* seconds */
#define MPC_HORIZON       20
#define MPC_REF_ENTRIES   20     /* Must match horizon */
#define MAX_WAYPOINTS     2000
#define MAX_STEERING      0.4282 /* rad — physical limit */
#define MAX_VELOCITY      20.0   /* m/s */
#define PHYSICAL_MAX_ACCEL 8.0   /* m/s² — matches MPC constraint bounds */
#define MIN_SPEED_FOR_MPC 0.5    /* m/s — below this, use low-speed guard */
#define MPC_CALL_INTERVAL 1      /* Call MPC every sim step */

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

        /* vy reference: zero */
        ref[step].reference_lateral_velocity_meters_per_second = 0;

        ref[step].reference_yaw_rate_radians_per_second = DOUBLE_TO_FP(kappa * v_wp);

        /* Acceleration feedforward: populated but currently unused by solver */
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
    /* Runtime-configurable timestep (env SIM_DT, default 0.005 = 200Hz) */
    const char *dt_env = getenv("SIM_DT");
    const double SIM_DT = dt_env ? atof(dt_env) : SIM_DT_DEFAULT;
    const int SIM_STEPS = (int)(SIM_DURATION / SIM_DT);
    const double mpc_prediction_dt = 0.05;
    const double cross_scale = SIM_DT / mpc_prediction_dt;

    printf("=== Spielberg Sim-Drive Test (Riccati-ADMM, %.0fs at dt=%.4fs = %d steps, %.0fHz) ===\n\n",
           SIM_DURATION, SIM_DT, SIM_STEPS, 1.0/SIM_DT);

    if (!load_raceline()) return 1;

    /* Initialize Riccati-ADMM MPC via the unified API */
    mpc_initialize();
    mpc_reset();

    /* Configure horizon and weights */
    MpcConfiguration_t cfg = mpc_get_configuration();
    cfg.prediction_horizon_steps = MPC_HORIZON;
    /* cross_call_rate_scale: ratio of control interval to prediction dt */
    cfg.cross_call_rate_scale = FP_CONST(cross_scale);
    /* Tuned weights — overridable via environment variables for tuning script.
     * See tune_weights.py for automated grid search. */
    const char *env;
    cfg.weight_lateral_error          = FP_CONST((env = getenv("Q_LAT"))       ? atof(env) : 125.0);
    cfg.weight_heading_error          = FP_CONST((env = getenv("Q_HDG"))       ? atof(env) : 300.0);
    cfg.weight_velocity               = FP_CONST((env = getenv("Q_VEL"))       ? atof(env) : 30.0);
    cfg.weight_lateral_velocity       = FP_CONST((env = getenv("Q_LAT_VEL"))   ? atof(env) : 60.0);
    cfg.weight_yaw_rate               = FP_CONST((env = getenv("Q_YAW"))       ? atof(env) : 20.0);
    cfg.weight_steering_effort        = FP_CONST((env = getenv("R_STEER"))     ? atof(env) : 0.35);
    cfg.weight_acceleration_effort    = FP_CONST((env = getenv("R_ACCEL"))     ? atof(env) : 0.01);
    cfg.weight_steering_rate          = FP_CONST((env = getenv("W_JERK"))      ? atof(env) : 0.5);
    cfg.weight_acceleration_rate      = FP_CONST((env = getenv("W_ACCEL_RATE"))? atof(env) : 0.01);
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
    int solver_ok = 0, solver_calls = 0;
    double prev_steer = 0;
    double actual_steer = 0;  /* Physical servo position (rate-limited) */
    int steer_reversals = 0;
    double max_steer_change = 0;
    double time_above_5ms = 0;
    double max_vx = 0;
    long total_iterations = 0;
    int max_iter_single = 0;
    double total_solve_us = 0.0, max_solve_us = 0.0;
    struct timespec ts0, ts1;

    /* Held MPC commands (zero-order hold between 20Hz MPC calls) */
    double cmd_steer = 0.0;
    double cmd_accel = 0.0;

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

        /* Wall collision check — wall hit = crash */
        double left_wall = raceline[closest].left_bound;
        double right_wall = raceline[closest].right_bound;
        int wall_hit = 0;
        if (e_y > left_wall)  { wall_hit = 1;  wall_collisions++; }
        if (e_y < -right_wall){ wall_hit = -1; wall_collisions++; }
        if (wall_hit && vx > 1.0) {
            printf("\n  !!! WALL CRASH: e_y = %.3f m (bound: %.3f) at step %d (t=%.2fs, wp=%d, v=%.1f) !!!\n",
                   e_y, wall_hit > 0 ? left_wall : right_wall, step, t, closest, vx);
            break;
        }

        /* Call MPC at 200Hz (every step) */
        double steer = cmd_steer;
        double accel_cmd = cmd_accel;
        int iter = 0;

        /* Feed the realized ST servo position to MPC (from previous propagation).
         * actual_steer is st_delta from end of previous step's propagation. */
        {
            ControlInput_t actual_ctrl;
            /* Use previously realized steering (st_delta), not desired */
            actual_ctrl.steering_angle_radians = DOUBLE_TO_FP(actual_steer);
            actual_ctrl.acceleration_meters_per_second_squared = DOUBLE_TO_FP(cmd_accel);
            mpc_set_actual_previous_control(&actual_ctrl);
        }

        if (step % MPC_CALL_INTERVAL == 0) {
            /* Build reference */
            TrajectoryReferencePoint_t ref[MPC_REF_ENTRIES];
            build_reference(closest, vx, ref);

            if (fabs(vx) < MIN_SPEED_FOR_MPC) {
                /* Low-speed guard: gentle straight + accelerate */
                double hdg_err = wrap_angle(psi - raceline[closest].psi);
                steer = 0.5 * hdg_err;
                if (steer > 0.2) steer = 0.2;
                if (steer < -0.2) steer = -0.2;
                accel_cmd = PHYSICAL_MAX_ACCEL;
                solver_ok++;
            } else {
                MpcSolverResult_t result;
                clock_gettime(CLOCK_MONOTONIC_RAW, &ts0);
                MpcSolverStatus_t status = mpc_compute_optimal_control(&frenet, ref, &result);
                clock_gettime(CLOCK_MONOTONIC_RAW, &ts1);
                double solve_us = (ts1.tv_sec - ts0.tv_sec) * 1e6
                                + (ts1.tv_nsec - ts0.tv_nsec) / 1e3;
                total_solve_us += solve_us;
                if (solve_us > max_solve_us) max_solve_us = solve_us;
                steer = FP_TO_DOUBLE(result.optimal_control.steering_angle_radians);
                accel_cmd = FP_TO_DOUBLE(result.optimal_control.acceleration_meters_per_second_squared);
                iter = result.iterations_used;
                total_iterations += iter;
                if (iter > max_iter_single) max_iter_single = iter;
                if (status == MPC_STATUS_SUCCESS || status == MPC_STATUS_MAXIMUM_ITERATIONS_REACHED)
                    solver_ok++;
            }
            solver_calls++;
            cmd_steer = steer;
            cmd_accel = accel_cmd;
        }

        /* Physical saturation: steering and acceleration */
        if (steer > MAX_STEERING) steer = MAX_STEERING;
        if (steer < -MAX_STEERING) steer = -MAX_STEERING;
        if (accel_cmd > PHYSICAL_MAX_ACCEL) accel_cmd = PHYSICAL_MAX_ACCEL;
        if (accel_cmd < -PHYSICAL_MAX_ACCEL) accel_cmd = -PHYSICAL_MAX_ACCEL;

        /* ST model handles servo dynamics internally via steering velocity.
         * actual_steer tracks the desired MPC command for the ST model.
         * We feed the REALIZED st_delta (from previous step) to MPC. */
        actual_steer = steer;

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

        /* Propagate vehicle state using gym-matching ST model with RK4.
         * State: [X, Y, delta, V, psi, psi_dot, beta] (7 states)
         * Matches f1tenth_gym single_track.py exactly.
         * Vehicle params from measured data (sim.yaml / vehicle_params.yaml). */
        {
            /* Vehicle parameters matching gym config */
            static const double mu = 0.7463, mass = 3.314, Iz = 0.035;
            static const double C_Sf = 2.804, C_Sr = 3.320;
            static const double lf = 0.166, lr = 0.16, h_cg = 0.0703;
            static const double g_acc = 9.81;
            static const double sv_max = 2.8492;  /* max steering velocity */
            static const double s_max = 0.4282;   /* max steering angle */
            static const double v_switch = 7.319;
            static const double v_min = 0.0, v_max = 20.0;
            static const double lwb = lf + lr;

            /* Persistent ST state variables (first call initializes from vehicle state) */
            static double st_delta = 0.0;
            static double st_V = 0.0;
            static double st_psi_dot = 0.0;
            static double st_beta = 0.0;
            static int st_initialized = 0;

            if (!st_initialized) {
                st_delta = actual_steer;
                st_V = vx;
                st_psi_dot = 0.0;
                st_beta = 0.0;
                st_initialized = 1;
            }

            /* Compute control inputs matching gym's constraint functions */
            /* steering_constraint: convert desired angle to steering velocity */
            double steer_vel;
            {
                double diff = actual_steer - st_delta;
                if (diff > 0) steer_vel = sv_max;
                else if (diff < 0) steer_vel = -sv_max;
                else steer_vel = 0.0;
                /* Clip so we don't overshoot the target */
                if (fabs(diff) < sv_max * SIM_DT)
                    steer_vel = diff / SIM_DT;
                /* Enforce steering angle limits */
                if (st_delta >= s_max && steer_vel > 0) steer_vel = 0.0;
                if (st_delta <= -s_max && steer_vel < 0) steer_vel = 0.0;
            }

            /* accl_constraints: v_switch power limit */
            double accl = accel_cmd;
            if (st_V > v_switch) {
                double a_max_eff = PHYSICAL_MAX_ACCEL * v_switch / st_V;
                if (accl > a_max_eff) accl = a_max_eff;
            }
            if (accl > PHYSICAL_MAX_ACCEL) accl = PHYSICAL_MAX_ACCEL;
            if (accl < -PHYSICAL_MAX_ACCEL) accl = -PHYSICAL_MAX_ACCEL;
            /* Velocity bounds */
            if (st_V <= v_min && accl < 0) accl = 0.0;
            if (st_V >= v_max && accl > 0) accl = 0.0;

            /* ST dynamics RHS function (matching single_track.py exactly) */
            #define ST_DYNAMICS(X, Y, DELTA, V, PSI, PSI_DOT_VAL, BETA, SV, ACCL, \
                               dX, dY, dDELTA, dV, dPSI, dPSI_DOT, dBETA) \
            do { \
                if ((V) < 0.5) { \
                    /* Kinematic mode */ \
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
                    /* Dynamic ST mode */ \
                    double glr_ = g_acc * lr - (ACCL) * h_cg; \
                    double glf_ = g_acc * lf + (ACCL) * h_cg; \
                    (dX) = (V) * cos((PSI) + (BETA)); \
                    (dY) = (V) * sin((PSI) + (BETA)); \
                    (dDELTA) = (SV); \
                    (dV) = (ACCL); \
                    (dPSI) = (PSI_DOT_VAL); \
                    (dPSI_DOT) = (mu * mass / (Iz * lwb)) * ( \
                        lf * C_Sf * glr_ * (DELTA) \
                        + (lr * C_Sr * glf_ - lf * C_Sf * glr_) * (BETA) \
                        - (lf*lf * C_Sf * glr_ + lr*lr * C_Sr * glf_) * (PSI_DOT_VAL) / (V)); \
                    (dBETA) = (mu / ((V) * lwb)) * ( \
                        C_Sf * glr_ * (DELTA) \
                        - (C_Sr * glf_ + C_Sf * glr_) * (BETA) \
                        + (C_Sr * glf_ * lr - C_Sf * glr_ * lf) * (PSI_DOT_VAL) / (V)) \
                        - (PSI_DOT_VAL); \
                } \
            } while(0)

            /* RK4 integration (matching gym's integrator) */
            double k1[7], k2[7], k3[7], k4[7];
            double s0[7] = {px, py, st_delta, st_V, psi, st_psi_dot, st_beta};

            /* k1 */
            ST_DYNAMICS(s0[0], s0[1], s0[2], s0[3], s0[4], s0[5], s0[6],
                        steer_vel, accl,
                        k1[0], k1[1], k1[2], k1[3], k1[4], k1[5], k1[6]);

            /* k2 */
            {
                double s1[7];
                for (int i = 0; i < 7; i++) s1[i] = s0[i] + 0.5 * SIM_DT * k1[i];
                ST_DYNAMICS(s1[0], s1[1], s1[2], s1[3], s1[4], s1[5], s1[6],
                            steer_vel, accl,
                            k2[0], k2[1], k2[2], k2[3], k2[4], k2[5], k2[6]);
            }

            /* k3 */
            {
                double s2[7];
                for (int i = 0; i < 7; i++) s2[i] = s0[i] + 0.5 * SIM_DT * k2[i];
                ST_DYNAMICS(s2[0], s2[1], s2[2], s2[3], s2[4], s2[5], s2[6],
                            steer_vel, accl,
                            k3[0], k3[1], k3[2], k3[3], k3[4], k3[5], k3[6]);
            }

            /* k4 */
            {
                double s3[7];
                for (int i = 0; i < 7; i++) s3[i] = s0[i] + SIM_DT * k3[i];
                ST_DYNAMICS(s3[0], s3[1], s3[2], s3[3], s3[4], s3[5], s3[6],
                            steer_vel, accl,
                            k4[0], k4[1], k4[2], k4[3], k4[4], k4[5], k4[6]);
            }

            /* RK4 update */
            double sn[7];
            for (int i = 0; i < 7; i++)
                sn[i] = s0[i] + (SIM_DT / 6.0) * (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);

            /* Clamp steering angle */
            if (sn[2] > s_max) sn[2] = s_max;
            if (sn[2] < -s_max) sn[2] = -s_max;
            /* Clamp velocity */
            if (sn[3] < v_min) sn[3] = v_min;
            if (sn[3] > v_max) sn[3] = v_max;
            /* Normalize heading */
            sn[4] = wrap_angle(sn[4]);

            /* Update persistent ST state */
            st_delta = sn[2];
            st_V = sn[3];
            st_psi_dot = sn[5];
            st_beta = sn[6];

            /* Convert ST state to MPC vehicle state */
            state.position_x_meters = DOUBLE_TO_FP(sn[0]);
            state.position_y_meters = DOUBLE_TO_FP(sn[1]);
            state.heading_angle_radians = DOUBLE_TO_FP(sn[4]);
            state.longitudinal_velocity_meters_per_second = DOUBLE_TO_FP(sn[3] * cos(sn[6]));
            state.lateral_velocity_meters_per_second = DOUBLE_TO_FP(sn[3] * sin(sn[6]));
            state.yaw_rate_radians_per_second = DOUBLE_TO_FP(sn[5]);

            /* Also update actual_steer from the ST state (delta is now a state) */
            actual_steer = st_delta;

            #undef ST_DYNAMICS
        }

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
    printf("\n  === Results (%.0f seconds, Riccati-ADMM, 200Hz MPC) ===\n", SIM_DURATION);
    printf("  Solver success:     %d / %d (%.1f%%)\n", solver_ok, solver_calls,
           100.0*solver_ok/(solver_calls > 0 ? solver_calls : 1));
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
    printf("\n  --- Solver Performance ---\n");
    int mpc_calls = solver_calls;
    double avg_iters = (mpc_calls > 0) ? (double)total_iterations / mpc_calls : 0;
    double avg_solve = (mpc_calls > 0) ? total_solve_us / mpc_calls : 0;
    printf("  Total iterations:   %ld\n", total_iterations);
    printf("  Avg iterations/call: %.1f\n", avg_iters);
    printf("  Max iterations:     %d\n", max_iter_single);
    printf("  Avg solve time:     %.1f us\n", avg_solve);
    printf("  Max solve time:     %.1f us\n", max_solve_us);
    printf("  Total solve time:   %.1f ms\n", total_solve_us / 1000.0);
    printf("\n");

    /* Pass/fail criteria */
    check("No wall collisions", wall_collisions == 0);
    check("Max lateral error < 1.2 m", max_lat_err < 1.2);
    check("Avg lateral error < 0.5 m", avg_lat < 0.5);
    check("Avg heading error < 0.3 rad (17 deg)", avg_hdg < 0.3);
    check("Solver mostly succeeds (>80%)", solver_ok > solver_calls * 80 / 100);
    check("Reaches driving speed (>5 m/s for >50% of time)",
          time_above_5ms > SIM_DURATION * 0.5);

    printf("\n=== RESULTS: %d passed, %d failed ===\n", tests_passed, tests_failed);

    /* Machine-readable CSV summary line for tuning scripts */
    if (getenv("MPC_TUNING_CSV")) {
        printf("CSV,%d,%d,%.4f,%.4f,%.4f,%.4f,%.2f,%.1f,%.1f,%d,%.1f,%.4f,%.4f,%.1f\n",
               tests_passed, tests_failed,
               max_lat_err, avg_lat, max_hdg_err, avg_hdg,
               max_vx, avg_solve, max_solve_us,
               wall_collisions, time_above_5ms,
               max_vel_err, avg_vel, avg_iters);
    }
    return tests_failed > 0 ? 1 : 0;
}
