/*******************************************************************************
 * test_sim_drive.c — Realistic closed-loop MPCC simulation on raceline
 *
 * Tests the MPCC controller (Global Frame, ADMM+Riccati) in closed-loop:
 *   - Gym-matching nonlinear single-track vehicle model (RK4)
 *   - Raceline CSV loading with track bounds
 *   - Environment-variable overrides for all MPCC weights (for tuning scripts)
 *   - Machine-readable CSV output for automated tuning (MPCC_TUNING_CSV=1)
 *
 * Build (standalone, from MPCC/):
 *   gcc -D_GNU_SOURCE -O3 -std=c99 -Wall -ffast-math \
 *       -Wno-unused-variable -Wno-unused-but-set-variable \
 *       -Iinclude -I../MPC/include \
 *       test/test_sim_drive.c \
 *       src/mpcc.c src/mpcc_vehicle_model.c src/qp_solver_mpcc.c \
 *       -o test_sim_drive -lm
 ******************************************************************************/

#define _GNU_SOURCE
#define _USE_MATH_DEFINES
#include <stdio.h>
#include <stdlib.h>

#include <string.h>
#include <math.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "mpcc_types.h"
#include "mpcc.h"

/* Portability: CLOCK_MONOTONIC_RAW fallback */
#ifndef CLOCK_MONOTONIC_RAW
#ifdef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC_RAW CLOCK_MONOTONIC
#endif
#endif

/*===========================================================================
 * Configuration
 *===========================================================================*/

#define SIM_DT_DEFAULT    0.005f   /* 200 Hz physics */
#define MPCC_DT_DEFAULT   0.050f   /* 20 Hz MPCC (matches ROS2 node timer) */
#define SIM_DURATION      30.0f    /* seconds */
#define MAX_WAYPOINTS     2000
#define MAX_STEERING      0.4189f  /* rad */
#define MAX_VELOCITY      20.0f    /* m/s */
#define PHYSICAL_MAX_ACCEL 7.31f   /* m/s² */

#define VEHICLE_HALF_WIDTH        0.155f  /* meters (F1/10th body half-width) */
#define DEFAULT_BODY_SAFETY_MARGIN 0.06f
#define STEER_BUFFER_SIZE         2

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
    const char *env_path = getenv("RACELINE_PATH");
    if (env_path) {
        FILE *f = fopen(env_path, "r");
        if (f) {
            if (getenv("VERBOSE")) printf("[LOAD] %s (from RACELINE_PATH)\n", env_path);
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
                    /* Legacy 7-column format without track bounds */
                    wp->left_bound = 5.0;
                    wp->right_bound = 5.0;
                    raceline_count++;
                }
            }
            fclose(f);
            printf("  Loaded %d waypoints\n", raceline_count);
            return 1;
        }
        fprintf(stderr, "ERROR: Cannot open RACELINE_PATH=%s\n", env_path);
        return 0;
    }
    const char *paths[] = {
        "../../f1tenth_planning/trajectories/my_track_raceline.csv",
        "../../../f1tenth_planning/trajectories/my_track_raceline.csv",
        "f1tenth_planning/trajectories/my_track_raceline.csv",
        "../f1tenth_planning/trajectories/my_track_raceline.csv",
        "../../../../f1tenth_planning/trajectories/my_track_raceline.csv",
        NULL
    };
    FILE *f = NULL;
    for (int i = 0; paths[i]; i++) {
        f = fopen(paths[i], "r");
        if (f) { printf("[LOAD] %s\n", paths[i]); break; }
    }
    if (!f) { fprintf(stderr, "ERROR: Cannot open raceline CSV\n"); return 0; }

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
            /* Legacy 7-column format without track bounds */
            wp->left_bound = 5.0;
            wp->right_bound = 5.0;
            raceline_count++;
        }
    }
    fclose(f);
    printf("[LOAD] %d waypoints\n", raceline_count);
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
        double hdg_diff = heading - raceline[idx].psi;
        while (hdg_diff >  M_PI) hdg_diff -= 2*M_PI;
        while (hdg_diff < -M_PI) hdg_diff += 2*M_PI;
        if (fabs(hdg_diff) > 1.57 && d2 < 4.0) continue;
        if (d2 < best_dist) { best_dist = d2; best = idx; }
    }
    last_closest = best;
    return best;
}

/*===========================================================================
 * Build MPCC reference path from loaded raceline
 *===========================================================================*/

static MPCCReferencePath_t g_ref_path;

static int build_reference_path(void)
{
    if (raceline_count < 2) return 0;

    g_ref_path.num_points = 0;
    for (int i = 0; i < raceline_count && i < MPCC_MAX_PATH_POINTS; i++) {
        MPCCPathPoint_t *pt = &g_ref_path.points[i];
        pt->s_ref     = (float)raceline[i].s;
        pt->x_ref     = (float)raceline[i].x;
        pt->y_ref     = (float)raceline[i].y;
        pt->phi_ref   = (float)raceline[i].psi;
        pt->kappa_ref = (float)raceline[i].kappa;
        pt->vx_ref    = (float)raceline[i].vx;

        /* Subtract car half-width so n bounds keep body inside track */
        float lb = (float)raceline[i].left_bound  - VEHICLE_HALF_WIDTH;
        float rb = (float)raceline[i].right_bound - VEHICLE_HALF_WIDTH;
        if (lb < 0.05f) lb = 0.05f;
        if (rb < 0.05f) rb = 0.05f;
        pt->left_bound  = lb;
        pt->right_bound = rb;

        g_ref_path.num_points++;
    }

    g_ref_path.total_length = g_ref_path.points[g_ref_path.num_points - 1].s_ref;
    g_ref_path.is_closed = 1;

    printf("[MPCC] Built reference path: %d points, length %.1f m\n",
           g_ref_path.num_points, g_ref_path.total_length);
    return 1;
}

/*===========================================================================
 * Env-var helper
 *===========================================================================*/

static double env_double(const char *name, double fallback)
{
    const char *v = getenv(name);
    return v ? atof(v) : fallback;
}

static int env_int(const char *name, int fallback)
{
    const char *v = getenv(name);
    return v ? atoi(v) : fallback;
}

/*===========================================================================
 * Test pass/fail
 *===========================================================================*/

static int tests_passed = 0, tests_failed = 0;

static void check(const char *label, int cond)
{
    if (cond) {
        tests_passed++;
        printf("  \033[0;32m[PASS]\033[0m %s\n", label);
    } else {
        tests_failed++;
        printf("  \033[0;31m[FAIL]\033[0m %s\n", label);
    }
}

/*===========================================================================
 * Main
 *===========================================================================*/

int main(void)
{
    const double SIM_DT = env_double("SIM_DT", SIM_DT_DEFAULT);
    const double MPCC_DT = env_double("MPCC_DT", MPCC_DT_DEFAULT);
    const int MPCC_CALL_INTERVAL = (int)(MPCC_DT / SIM_DT + 0.5);
    const int SIM_STEPS = (int)(SIM_DURATION / SIM_DT);
    const int verbose = getenv("VERBOSE") != NULL;
    const double body_safety_margin = env_double("BODY_SAFETY_MARGIN", DEFAULT_BODY_SAFETY_MARGIN);

    printf("=== MPCC Sim-Drive (Lifted ODE, ADMM+Riccati, %.0fs at dt=%.4fs) ===\n",
           SIM_DURATION, SIM_DT);
    printf("    MPCC rate: %.0fHz (every %d sim steps)\n",
           1.0/MPCC_DT, MPCC_CALL_INTERVAL);

    if (!load_raceline()) return 1;
    if (!build_reference_path()) return 1;

    /* ── Build MPCC configuration from env vars ────────────────────────── */
    MPCCConfiguration_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    /* Horizon (tuned via sweep) */
    cfg.horizon_steps     = env_int("HORIZON", 7);
    cfg.dt                = env_double("DT", 0.02f);

    /* Contouring tracking */
    cfg.weight_contouring = env_double("Q_CONTOURING", 2000.0f);
    cfg.weight_lag        = env_double("Q_LAG", 136.282905f);
    cfg.weight_progress   = env_double("Q_PROGRESS", 9.04816f);

    /* State regularization */
    cfg.weight_vx         = env_double("Q_VX", 1.5f);
    cfg.vx_ref            = env_double("VX_REF", 4.034304f);
    cfg.weight_vy         = env_double("Q_VY", 1.5435f);
    cfg.weight_omega      = env_double("Q_OMEGA", 0.805f);

    /* Control effort */
    cfg.weight_delta      = env_double("R_DELTA", 13.75f);
    cfg.weight_ax         = env_double("R_AX", 0.054864f);
    cfg.weight_v_theta    = env_double("R_VTHETA", 1.1232f);

    /* Control rate */
    cfg.weight_delta_rate   = env_double("W_DELTA_RATE", 0.65f);
    cfg.weight_ax_rate      = env_double("W_AX_RATE", 0.610926f);
    cfg.weight_v_theta_rate = env_double("W_VTHETA_RATE", 0.126f);

    /* Terminal */
    cfg.weight_contouring_terminal = env_double("Q_CONTOURING_TERM", 493.7625f);
    cfg.weight_lag_terminal     = env_double("Q_LAG_TERM", 1072.17f);
    cfg.weight_progress_terminal = env_double("Q_PROGRESS_TERM", 5.564503f);

    /* Obstacle */
    cfg.weight_obstacle   = env_double("W_OBSTACLE", 1000.0f);
    cfg.obstacle_margin   = env_double("OBSTACLE_MARGIN", 0.1f);

    /* ADMM solver (tuned via sweep) */
    cfg.admm_rho            = env_double("ADMM_RHO", 0.9f);
    cfg.admm_max_iterations = env_int("ADMM_MAX_ITER", 100);
    cfg.admm_tolerance      = env_double("ADMM_TOL", 0.011449f);

    /* Constraint bounds */
    cfg.delta_max = env_double("DELTA_MAX", 0.4189f);
    cfg.ax_max    = env_double("AX_MAX", 7.0f);
    cfg.ax_min    = env_double("AX_MIN", -10.0f);
    cfg.vx_max    = env_double("VX_MAX", 20.0f);
    cfg.vx_min    = env_double("VX_MIN", 0.0f);
    cfg.v_theta_max = env_double("V_THETA_MAX", 6.24f);
    cfg.v_theta_min = env_double("V_THETA_MIN", 0.0f);

    /* Tire parameters */
    cfg.mu   = env_double("MU", 0.745f);
    cfg.C_Sf = env_double("C_SF", 4.297f);
    cfg.C_Sr = env_double("C_SR", 3.473f);

    if (verbose) {
        printf("  Config: N=%d dt=%.3f Q_c=%.1f Q_l=%.1f Q_prog=%.1f\n",
               cfg.horizon_steps, cfg.dt,
               cfg.weight_contouring, cfg.weight_lag,
               cfg.weight_progress);
        printf("  Q_vx=%.1f vx_ref=%.1f R_delta=%.2f R_ax=%.3f R_vt=%.2f\n",
               cfg.weight_vx, cfg.vx_ref,
               cfg.weight_delta, cfg.weight_ax,
               cfg.weight_v_theta);
        printf("  ADMM: rho=%.2f max_iter=%d tol=%.4f\n",
               cfg.admm_rho, cfg.admm_max_iterations,
               cfg.admm_tolerance);
    }

    /* Initialize MPCC with custom config */
    mpcc_initialize_with_config(&cfg);
    mpcc_set_reference_path(&g_ref_path);

    /* ── Vehicle model parameters (gym-matching) ───────────────────────── */
    static const double mu = 0.745, mass = 3.314, Iz = 0.035;
    static const double C_Sf = 4.297, C_Sr = 3.473;
    static const double lf = 0.166, lr = 0.16, h_cg = 0.0703;
    static const double g_acc = 9.81;
    static const double sv_max = 2.8492;
    static const double s_max = 0.4189;
    static const double v_switch = 7.319;
    static const double v_min = 0.0, v_max = 20.0;
    static const double lwb = 0.166 + 0.16;

    /* ── Spawn at raceline[0] from standstill ─────────────────────────────── */
    /* Standstill startup validates that MPCC can launch cleanly from 0 m/s. */
    VehicleState_t state;
    state.pos_x = (raceline[0].x);
    state.pos_y = (raceline[0].y);
    state.heading = (raceline[0].psi);
    state.long_vel = 0;
    state.lat_vel = 0;
    state.yaw_rate = 0;

    /* Tracking metrics */
    double max_lat_err = 0, sum_lat_err = 0;
    double max_hdg_err = 0, sum_hdg_err = 0;
    double max_vel_err = 0, sum_vel_err = 0;
    double sum_vx = 0;
    int wall_collisions = 0;
    int prev_wall_hit = 0;  /* edge detection: only count entering a wall */
    int numerical_failures = 0;
    int solver_ok = 0, solver_calls = 0;
    double prev_steer = 0;
    double actual_steer = 0;
    double steer_buffer[STEER_BUFFER_SIZE];
    int steer_buf_idx = 0;
    for (int i = 0; i < STEER_BUFFER_SIZE; i++) steer_buffer[i] = 0.0;
    int steer_reversals = 0;
    double max_steer_change = 0;
    double time_above_2ms = 0;
    double time_above_5ms = 0;
    double max_vx = 0;

    long total_iterations = 0;
    int max_iter_single = 0;
    double total_solve_us = 0.0, max_solve_us = 0.0;
    long total_adaptive_updates = 0;
    uint64_t total_clip_events = 0;
    double sum_rho = 0.0, sum_rho_u = 0.0;
    struct timespec ts0, ts1;

    /* Held MPCC commands (zero-order hold between MPCC calls) */
    double cmd_steer = 0.0;
    double cmd_accel = 0.0;
    float current_s = 0;

    /* ST model persistent state */
    double st_delta = 0.0;
    double st_V = 0.0;
    double st_psi_dot = 0.0;
    double st_beta = 0.0;
    int st_initialized = 0;

    int stepped = 0;

    if (verbose) {
        printf("\n  Step | Time  | vx    | v_cmd | e_y   | e_psi | cmd_st | act_st | accel | st | it\n");
        printf("  -----|-------|-------|-------|-------|-------|--------|--------|-------|----|---\n");
    }

    for (int step = 0; step < SIM_STEPS; step++) {
        double t = step * SIM_DT;
        double px = (double)(state.pos_x);
        double py = (double)(state.pos_y);
        double psi = (double)(state.heading);
        double vx = (double)(state.long_vel);

        if (!isfinite(px) || !isfinite(py) || !isfinite(psi) || !isfinite(vx)) {
            numerical_failures++;
            printf("\n  !!! NUMERICAL FAILURE at step=%d t=%.2f !!!\n", step, t);
            stepped = step;
            break;
        }

        if (vx > max_vx) max_vx = vx;

        int closest = find_closest_waypoint(px, py, psi);

        /* Compute Frenet error for metrics */
        double dx = px - raceline[closest].x;
        double dy = py - raceline[closest].y;
        double path_psi = raceline[closest].psi;
        double e_y = -dx * sin(path_psi) + dy * cos(path_psi);
        double e_psi = wrap_angle(psi - path_psi);

        /* Wall collision check (body-edge) — abort on first hit */
        double left_wall  = raceline[closest].left_bound;
        double right_wall = raceline[closest].right_bound;
        int wall_hit = 0;
        if (e_y > (left_wall - VEHICLE_HALF_WIDTH - body_safety_margin))   { wall_hit = 1; }
        if (e_y < -(right_wall - VEHICLE_HALF_WIDTH - body_safety_margin)) { wall_hit = -1; }
        if (wall_hit && !prev_wall_hit) {
            wall_collisions++;
            if (verbose) {
                printf("\n  !!! WALL COLLISION: e_y=%.3f (bound:%.3f) step=%d t=%.2f wp=%d v=%.1f — aborting !!!\n",
                       e_y, wall_hit > 0 ? left_wall : right_wall, step, t, closest, vx);
            }
            stepped = step;
            break;  /* single hit = failure, stop sim */
        }
        prev_wall_hit = wall_hit;

        /* Call MPCC at configured rate */
        double steer = cmd_steer;
        double accel_cmd = cmd_accel;
        int iter = 0;
        int status_val = -1;

        if (step % MPCC_CALL_INTERVAL == 0) {
            /* Convert vehicle state -> MPCC state */
            MPCCState_t mpcc_state = mpcc_state_from_vehicle_state(&state, current_s);
            current_s = mpcc_state.s;

            /* Solve */
            MPCCResult_t result;
            clock_gettime(CLOCK_MONOTONIC_RAW, &ts0);
            MPCCStatus_t status = mpcc_compute_control(&mpcc_state, &result);
            clock_gettime(CLOCK_MONOTONIC_RAW, &ts1);

            double solve_us = (ts1.tv_sec - ts0.tv_sec) * 1e6
                            + (ts1.tv_nsec - ts0.tv_nsec) / 1e3;
            total_solve_us += solve_us;
            if (solve_us > max_solve_us) max_solve_us = solve_us;

            steer = (double)result.optimal_control.delta;
            accel_cmd = (double)result.optimal_control.a_x;
            iter = result.admm_iterations;
            status_val = (int)status;

            total_iterations += iter;
            if (iter > max_iter_single) max_iter_single = iter;
            if (status == MPCC_STATUS_SUCCESS || status == MPCC_STATUS_MAX_ITERATIONS)
                solver_ok++;
            solver_calls++;

            total_adaptive_updates += result.adaptive_rho_updates;
            total_clip_events += result.numeric_clip_count;
            sum_rho += (double)(result.rho_final);
            sum_rho_u += (double)(result.rho_u_final);

            cmd_steer = steer;
            cmd_accel = accel_cmd;
        }

        /* Physical saturation */
        if (steer > MAX_STEERING)  steer = MAX_STEERING;
        if (steer < -MAX_STEERING) steer = -MAX_STEERING;
        if (accel_cmd > PHYSICAL_MAX_ACCEL)  accel_cmd = PHYSICAL_MAX_ACCEL;
        if (accel_cmd < -PHYSICAL_MAX_ACCEL) accel_cmd = -PHYSICAL_MAX_ACCEL;

        /* Steer buffer delay (matching gym) */
        actual_steer = steer_buffer[steer_buf_idx];
        steer_buffer[steer_buf_idx] = steer;
        steer_buf_idx = (steer_buf_idx + 1) % STEER_BUFFER_SIZE;

        /* Metrics */
        if (fabs(e_y) > max_lat_err) max_lat_err = fabs(e_y);
        sum_lat_err += fabs(e_y);
        if (fabs(e_psi) > max_hdg_err) max_hdg_err = fabs(e_psi);
        sum_hdg_err += fabs(e_psi);
        double vel_err = fabs(vx - raceline[closest].vx);
        if (vel_err > max_vel_err) max_vel_err = vel_err;
        sum_vel_err += vel_err;
        sum_vx += vx;
        if (vx > 2.0) time_above_2ms += SIM_DT;
        if (vx > 5.0) time_above_5ms += SIM_DT;

        double steer_change = actual_steer - prev_steer;
        if (fabs(steer_change) > fabs(max_steer_change)) max_steer_change = steer_change;
        if (step > 0 && actual_steer * prev_steer < 0 && fabs(steer_change) > 0.1)
            steer_reversals++;
        prev_steer = actual_steer;

        if (verbose) {
            int print_row = (step < 40) || (step % 200 == 0) || wall_hit || (fabs(e_y) > 0.8);
            if (print_row) {
                printf("  %4d | %5.2f | %5.2f | %5.2f | %+.3f | %+.3f | %+.4f | %+.4f | %+.2f | %2d | %3d\n",
                       step, t, vx, raceline[closest].vx, e_y, e_psi,
                       steer, actual_steer, accel_cmd, status_val, iter);
            }
        }

        /* ── Propagate vehicle: gym-matching ST model with RK4 ─────────── */
        {
            if (!st_initialized) {
                st_delta = actual_steer;
                st_V = vx;
                st_psi_dot = 0.0;
                st_beta = 0.0;
                st_initialized = 1;
            }

            /* pid_steer: bang-bang matching gym */
            double steer_vel;
            {
                double diff = actual_steer - st_delta;
                if (fabs(diff) > 1e-4)
                    steer_vel = (diff > 0 ? 1.0 : -1.0) * sv_max;
                else
                    steer_vel = 0.0;
                if (st_delta >= s_max && steer_vel > 0) steer_vel = 0.0;
                if (st_delta <= -s_max && steer_vel < 0) steer_vel = 0.0;
                if (steer_vel < -sv_max) steer_vel = -sv_max;
                if (steer_vel >  sv_max) steer_vel =  sv_max;
            }

            /* accl_constraints */
            double accl = accel_cmd;
            if (st_V > v_switch) {
                double a_max_eff = PHYSICAL_MAX_ACCEL * v_switch / st_V;
                if (accl > a_max_eff) accl = a_max_eff;
            }
            if (accl > PHYSICAL_MAX_ACCEL)  accl = PHYSICAL_MAX_ACCEL;
            if (accl < -PHYSICAL_MAX_ACCEL) accl = -PHYSICAL_MAX_ACCEL;
            if (st_V <= v_min && accl < 0) accl = 0.0;
            if (st_V >= v_max && accl > 0) accl = 0.0;

            /* ST dynamics (linear tires, matching gym single_track.py) */
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
                    double Fx_ = mass * (ACCL); \
                    double cos_delta_ = cos(DELTA); \
                    double sin_delta_ = sin(DELTA); \
                    double dvx_dt_ = (Fx_ - Fyf_ * sin_delta_ \
                                      + mass * vy_ * (PSI_DOT_VAL)) / mass; \
                    double dvy_dt_ = (Fyf_ * cos_delta_ + Fyr_ \
                                      - mass * vx_ * (PSI_DOT_VAL)) / mass; \
                    double V_safe_ = ((V) > 0.001) ? (V) : 0.001; \
                    double V_sq_ = (V) * (V); \
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

            double true_px = (double)(state.pos_x);
            double true_py = (double)(state.pos_y);
            double true_psi = (double)(state.heading);
            double s0[7] = {true_px, true_py, st_delta, st_V, true_psi, st_psi_dot, st_beta};

            double k1[7], k2[7], k3[7], k4[7];

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
                sn[i] = s0[i] + (SIM_DT / 6.0) * (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);

            if (sn[2] > s_max)  sn[2] = s_max;
            if (sn[2] < -s_max) sn[2] = -s_max;
            if (sn[3] < v_min)  sn[3] = v_min;
            if (sn[3] > v_max)  sn[3] = v_max;
            sn[4] = wrap_angle(sn[4]);

            st_delta = sn[2];
            st_V = sn[3];
            st_psi_dot = sn[5];
            st_beta = sn[6];

            state.pos_x = (sn[0]);
            state.pos_y = (sn[1]);
            state.heading = (sn[4]);
            state.long_vel = (sn[3] * cos(sn[6]));
            state.lat_vel = (sn[3] * sin(sn[6]));
            state.yaw_rate = (sn[5]);

            actual_steer = st_delta;

            #undef ST_DYNAMICS
        }

        stepped = step;

    }

    /* ── Summary ───────────────────────────────────────────────────────── */
    int total_stepped = stepped + 1;
    double avg_lat = sum_lat_err / total_stepped;
    double avg_hdg = sum_hdg_err / total_stepped;
    double avg_vel_err = sum_vel_err / total_stepped;
    double avg_speed = sum_vx / total_stepped;

    printf("\n  === Results (%.0fs, MPCC Lifted ODE, ADMM+Riccati) ===\n", SIM_DURATION);
    printf("  Solver success:     %d / %d (%.1f%%)\n", solver_ok, solver_calls,
           100.0*solver_ok/(solver_calls > 0 ? solver_calls : 1));
    printf("  Avg speed:          %.2f m/s\n", avg_speed);
    printf("  Max velocity:       %.2f m/s\n", max_vx);
    printf("  Max lateral error:  %.3f m\n", max_lat_err);
    printf("  Avg lateral error:  %.3f m\n", avg_lat);
    printf("  Max heading error:  %.4f rad (%.1f deg)\n", max_hdg_err, max_hdg_err*180/M_PI);
    printf("  Avg heading error:  %.4f rad (%.1f deg)\n", avg_hdg, avg_hdg*180/M_PI);
    printf("  Max velocity error: %.2f m/s\n", max_vel_err);
    printf("  Avg velocity error: %.2f m/s\n", avg_vel_err);
    printf("  Max steer change:   %.4f rad/step\n", max_steer_change);
    printf("  Steer reversals:    %d\n", steer_reversals);
    printf("  Wall collisions:    %d\n", wall_collisions);
    printf("  Time above 2 m/s:   %.1f / %.1f s (%.0f%%)\n",
           time_above_2ms, SIM_DURATION, 100*time_above_2ms/SIM_DURATION);
    printf("  Time above 5 m/s:   %.1f / %.1f s (%.0f%%)\n",
           time_above_5ms, SIM_DURATION, 100*time_above_5ms/SIM_DURATION);
    printf("\n  --- Solver Performance ---\n");
    double avg_iters = (solver_calls > 0) ? (double)total_iterations / solver_calls : 0;
    double avg_solve = (solver_calls > 0) ? total_solve_us / solver_calls : 0;
    double avg_adapt_updates = (solver_calls > 0)
                             ? (double)total_adaptive_updates / solver_calls : 0.0;
    double avg_clip_events = (solver_calls > 0)
                           ? (double)total_clip_events / solver_calls : 0.0;
    double avg_rho = (solver_calls > 0) ? sum_rho / solver_calls : 0.0;
    double avg_rho_u = (solver_calls > 0) ? sum_rho_u / solver_calls : 0.0;
    printf("  Total iterations:   %ld\n", total_iterations);
    printf("  Avg iterations/call: %.1f\n", avg_iters);
    printf("  Max iterations:     %d\n", max_iter_single);
    printf("  Avg solve time:     %.1f us\n", avg_solve);
    printf("  Max solve time:     %.1f us\n", max_solve_us);
    printf("  Total solve time:   %.1f ms\n", total_solve_us / 1000.0);
    printf("  Avg rho/rho_u:      %.3f / %.3f\n", avg_rho, avg_rho_u);
    printf("  Avg rho updates:    %.2f per solve\n", avg_adapt_updates);
    printf("  Avg clip events:    %.2f per solve\n", avg_clip_events);
    printf("\n");

    /* Pass/fail criteria: standstill launch + numerical safety + speed */
    check("No numerical failures", numerical_failures == 0);
    check("Runs full simulation horizon", total_stepped == SIM_STEPS);
    check("Solver mostly succeeds (>50%)", solver_ok > solver_calls * 50 / 100);
    check("Launches from standstill (>1 m/s reached)", max_vx > 1.0);
    check("Sustains motion (>2 m/s for >5% of time)",
          time_above_2ms > SIM_DURATION * 0.05);
    check("High top speed (>5 m/s)", max_vx > 5.0);

    printf("\n=== RESULTS: %d passed, %d failed ===\n", tests_passed, tests_failed);

    /* Machine-readable CSV for tuning scripts */
    if (getenv("MPCC_TUNING_CSV")) {
         printf("CSV,%d,%d,%.4f,%.4f,%.4f,%.4f,%.2f,%.1f,%.1f,%d,%.1f,%.4f,%.4f,%.1f,%.3f,%.3f,%.2f,%.2f,%.4f\n",
               tests_passed, tests_failed,
               max_lat_err, avg_lat, max_hdg_err, avg_hdg,
               max_vx, avg_solve, max_solve_us,
               wall_collisions, time_above_5ms,
               max_vel_err, avg_vel_err, avg_iters,
               avg_rho, avg_rho_u, avg_adapt_updates, avg_clip_events,
               avg_speed);
    }
    return tests_failed > 0 ? 1 : 0;
}
