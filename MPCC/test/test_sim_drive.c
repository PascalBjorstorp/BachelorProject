/*******************************************************************************
 * test_sim_drive.c — Realistic closed-loop MPCC simulation on raceline
 *
 * Tests the MPCC controller (Global Frame, OSQP or ADMM/Riccati) in closed-loop:
 *   - Gym-matching nonlinear single-track vehicle model (RK4)
 *   - Raceline CSV loading with track bounds
 *   - Environment-variable overrides for all MPCC weights (for tuning scripts)
 *   - Optional per-step trace CSV export (MPCC_TRACE_CSV_PATH=/path/to/file.csv)
 *   - Machine-readable CSV output for automated tuning (MPCC_TUNING_CSV=1)
 *
 * Build (ADMM solver, from MPCC/):
 *   gcc -O3 -std=c99 -Wall -ffast-math \
 *       -Wno-unused-variable -Wno-unused-but-set-variable \
 *       -Iinclude -I../MPC/include \
 *       test/test_sim_drive.c \
 *       src/mpcc.c src/mpcc_vehicle_model.c src/qp_solver_mpcc.c \
 *       -o test_sim_drive -lm
 *
 * Build (OSQP solver, from MPCC/):
 *   gcc -DUSE_OSQP -O3 -std=c99 -Wall \
 *       -ffast-math -Wno-unused-variable -Wno-unused-but-set-variable \
 *       -Iinclude -I/opt/ros/jazzy/include \
 *       test/test_sim_drive.c \
 *       src/mpcc.c src/mpcc_vehicle_model.c src/qp_solver_osqp.c \
 *       -o test_sim_drive_osqp -L/opt/ros/jazzy/lib -losqp -lm \
 *       -Wl,-rpath,/opt/ros/jazzy/lib
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

#ifdef USE_OSQP
#define MPCC_SOLVER_NAME "OSQP"
#else
#define MPCC_SOLVER_NAME "ADMM+Riccati"
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
#define MPCC_DT_DEFAULT   (1.0f / MPCC_CONTROL_RATE_HZ) /* 200 Hz nominal MPCC solve rate */
#define SIM_DURATION      120.0f   /* seconds — 60s too short for slow/long tracks */
#define MAX_WAYPOINTS     2000
#define MAX_STEERING      0.4189f  /* rad */
#define MAX_VELOCITY      20.0f    /* m/s */
#define PHYSICAL_MAX_ACCEL 7.31f   /* m/s² */

#define VEHICLE_HALF_WIDTH        0.155f  /* meters (F1/10th body half-width) */
#define DEFAULT_BODY_SAFETY_MARGIN 0.01f
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
        "../../f1tenth_planning/trajectories/my_track_centerline_smooth.csv",
        "../../../f1tenth_planning/trajectories/my_track_centerline_smooth.csv",
        "f1tenth_planning/trajectories/my_track_centerline_smooth.csv",
        "../f1tenth_planning/trajectories/my_track_centerline_smooth.csv",
        "../../../../f1tenth_planning/trajectories/my_track_centerline_smooth.csv",
        "../../f1tenth_planning/trajectories/my_track_centerline_verified.csv",
        "../../../f1tenth_planning/trajectories/my_track_centerline_verified.csv",
        "f1tenth_planning/trajectories/my_track_centerline_verified.csv",
        "../f1tenth_planning/trajectories/my_track_centerline_verified.csv",
        "../../../../f1tenth_planning/trajectories/my_track_centerline_verified.csv",
        "../../f1tenth_planning/trajectories/my_track_centerline.csv",
        "../../../f1tenth_planning/trajectories/my_track_centerline.csv",
        "f1tenth_planning/trajectories/my_track_centerline.csv",
        "../f1tenth_planning/trajectories/my_track_centerline.csv",
        "../../../../f1tenth_planning/trajectories/my_track_centerline.csv",
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

static double wrap_s_delta(double delta, double track_length)
{
    if (track_length <= 0.0) return delta;

    double half_length = 0.5 * track_length;
    while (delta > half_length) delta -= track_length;
    while (delta < -half_length) delta += track_length;
    return delta;
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

    /* body_safety_margin is also applied in collision detection;
     * subtract it from QP bounds too so the solver respects the same corridor. */
    const char *body_margin_env = getenv("BODY_SAFETY_MARGIN");
    const double body_margin = body_margin_env ? atof(body_margin_env)
                                               : DEFAULT_BODY_SAFETY_MARGIN;

    g_ref_path.num_points = 0;
    int invalid_corridor_points = 0;
    float min_left_effective = 1.0e9f;
    float min_right_effective = 1.0e9f;
    for (int i = 0; i < raceline_count && i < MPCC_MAX_PATH_POINTS; i++) {
        MPCCPathPoint_t *pt = &g_ref_path.points[i];
        pt->s_ref     = (float)raceline[i].s;
        pt->x_ref     = (float)raceline[i].x;
        pt->y_ref     = (float)raceline[i].y;
        pt->phi_ref   = (float)raceline[i].psi;
        pt->kappa_ref = (float)raceline[i].kappa;
        pt->vx_ref    = (float)raceline[i].vx;

        /* Keep the exact post-body corridor where possible; only clip truly
         * impossible negative widths to zero so the solver does not get more
         * space than the collision checker allows. */
        float lb_raw = (float)(raceline[i].left_bound  - VEHICLE_HALF_WIDTH - body_margin);
        float rb_raw = (float)(raceline[i].right_bound - VEHICLE_HALF_WIDTH - body_margin);
        float lb = lb_raw;
        float rb = rb_raw;
        if (lb_raw < min_left_effective) min_left_effective = lb_raw;
        if (rb_raw < min_right_effective) min_right_effective = rb_raw;
        int body_infeasible = (lb_raw <= 0.0f || rb_raw <= 0.0f);
        if (body_infeasible) invalid_corridor_points++;
        if ((lb + rb) < 0.0f) {
            float lower = -lb;
            float upper = rb;
            float center = 0.5f * (lower + upper);
            lb = -center;
            rb = center;
        }
        pt->left_bound  = lb;
        pt->right_bound = rb;

        /* Local robustness: if corridor is body-infeasible, slow down and expand margin */
        if (body_infeasible) {
            pt->vx_ref = fminf(pt->vx_ref, 0.5f); // Slow to 0.5 m/s at infeasible points
            // Optionally, expand bounds slightly for extra margin (soft, not hard)
            pt->left_bound  += 0.02f; // Add 2cm margin left
            pt->right_bound += 0.02f; // Add 2cm margin right
        }

        g_ref_path.num_points++;
    }

    g_ref_path.total_length = g_ref_path.points[g_ref_path.num_points - 1].s_ref;
    g_ref_path.is_closed = 1;

    printf("[MPCC] Built reference path: %d points, length %.1f m\n",
           g_ref_path.num_points, g_ref_path.total_length);
    printf("[MPCC] Wall corridor: CSV bounds minus body %.3f m and safety %.3f m "
           "(min effective left/right %.3f / %.3f m)\n",
           (double)VEHICLE_HALF_WIDTH,
           body_margin,
           (double)min_left_effective,
           (double)min_right_effective);
    if (invalid_corridor_points > 0) {
        printf("[MPCC] Warning: %d path points are narrower than vehicle width + safety margin (min effective left/right %.3f / %.3f m)\n",
               invalid_corridor_points,
               (double)min_left_effective,
               (double)min_right_effective);
    }
    return 1;
}

static double compute_predicted_min_wall_slack(
    const MPCCResult_t *result,
    int horizon_steps)
{
    double min_slack = 1.0e9;

    for (int k = 1; k <= horizon_steps; k++) {
        MPCCPathPoint_t path_pt;
        mpcc_path_interpolate(&g_ref_path, result->predicted_states[k].s, &path_pt);

        double dx = (double)result->predicted_states[k].X - (double)path_pt.x_ref;
        double dy = (double)result->predicted_states[k].Y - (double)path_pt.y_ref;
        double sin_phi = sin((double)path_pt.phi_ref);
        double cos_phi = cos((double)path_pt.phi_ref);
        double e_c = (sin_phi * dx) - (cos_phi * dy);
        double left_slack = e_c + (double)path_pt.left_bound;
        double right_slack = (double)path_pt.right_bound - e_c;
        double stage_slack = (left_slack < right_slack) ? left_slack : right_slack;

        if (stage_slack < min_slack)
            min_slack = stage_slack;
    }

    return min_slack;
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
    const double SIM_DURATION_S = env_double("SIM_DURATION", SIM_DURATION);
    const double SIM_DT = env_double("SIM_DT", SIM_DT_DEFAULT);
    const double MPCC_DT = env_double("MPCC_DT", MPCC_DT_DEFAULT);
    const double ODOM_DT = env_double("ODOM_DT", SIM_DT);
    const double POSE_DT = env_double("POSE_DT", MPCC_DT);
    const double CONTROL_DT = (POSE_DT > ODOM_DT) ? POSE_DT : ODOM_DT;
    int MPCC_CALL_INTERVAL = (int)(CONTROL_DT / SIM_DT + 0.5);
    int ODOM_UPDATE_INTERVAL = (int)(ODOM_DT / SIM_DT + 0.5);
    int POSE_UPDATE_INTERVAL = (int)(POSE_DT / SIM_DT + 0.5);
    const int SIM_STEPS = (int)(SIM_DURATION_S / SIM_DT);
    const int verbose = getenv("VERBOSE") != NULL;
    const double body_safety_margin = env_double("BODY_SAFETY_MARGIN", DEFAULT_BODY_SAFETY_MARGIN);
    const char *trace_csv_path = getenv("MPCC_TRACE_CSV_PATH");
    const char *cross_call_scale_env = getenv("CROSS_CALL_SCALE");
    FILE *trace_csv = NULL;

    if (MPCC_CALL_INTERVAL < 1) MPCC_CALL_INTERVAL = 1;
    if (ODOM_UPDATE_INTERVAL < 1) ODOM_UPDATE_INTERVAL = 1;
    if (POSE_UPDATE_INTERVAL < 1) POSE_UPDATE_INTERVAL = 1;

    printf("=== MPCC Sim-Drive (Lifted ODE, %s, %.0fs at dt=%.4fs) ===\n",
           MPCC_SOLVER_NAME, SIM_DURATION_S, SIM_DT);
    printf("    MPCC rate: %.0fHz (every %d sim steps)\n",
        1.0 / CONTROL_DT, MPCC_CALL_INTERVAL);
        printf("    Sensor cadence: odom %.0fHz (every %d), pose %.0fHz (every %d)\n",
            1.0 / ODOM_DT, ODOM_UPDATE_INTERVAL,
            1.0 / POSE_DT, POSE_UPDATE_INTERVAL);

    if (!load_raceline()) return 1;
    if (!build_reference_path()) return 1;

    /* ── Build MPCC configuration from env vars ────────────────────────── */
    MPCCConfiguration_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    /* Horizon */
    cfg.horizon_steps     = env_int("HORIZON", MPCC_DEFAULT_HORIZON);
    cfg.dt                = env_double("DT", 0.03f);

    /* April 21 winning configuration from the latest safe sweep. */
    cfg.weight_contouring = env_double("Q_CONTOURING", 960.0f);
    cfg.weight_lag        = env_double("Q_LAG", 200.0f);
    cfg.weight_wall_clearance = env_double("Q_WALL_CLEARANCE", MPCC_DEFAULT_WEIGHT_WALL_CLEARANCE);
    cfg.wall_clearance_margin = env_double("WALL_CLEARANCE_MARGIN", MPCC_DEFAULT_WALL_CLEARANCE_MARGIN);
    cfg.weight_progress   = env_double("Q_PROGRESS", 15.6f);
    cfg.track_safety_buffer = env_double("MPCC_TRACK_BUFFER", MPCC_DEFAULT_TRACK_SAFETY_BUFFER);

    /* State regularization */
    cfg.weight_vx         = env_double("Q_VX", 30.0f);
    cfg.vx_ref            = env_double("VX_REF", 4.0f);
    cfg.use_raceline_vx_ref = (uint8_t)env_int("MPCC_USE_RACELINE_VX_REF", 0);
    cfg.use_raceline_vx_limit = (uint8_t)env_int("MPCC_USE_RACELINE_VX_LIMIT", 0);
    cfg.raceline_vx_limit_scale = env_double("MPCC_RACELINE_VX_LIMIT_SCALE", 1.0f);
    cfg.weight_vy         = env_double("Q_VY", 0.5f);
    cfg.weight_omega      = env_double("Q_OMEGA", 1.5f);

    /* Control effort */
    cfg.weight_delta      = env_double("R_DELTA", 100.0f);
    cfg.weight_ax         = env_double("R_AX", 0.05225f);
    cfg.weight_v_theta    = env_double("R_VTHETA", 0.1f);

    /* Control rate */
    cfg.weight_delta_rate   = env_double("W_DELTA_RATE", 5.0f);
    cfg.weight_ax_rate      = env_double("W_AX_RATE", 0.488f);
    cfg.weight_v_theta_rate = env_double("W_VTHETA_RATE", 0.1105f);

    /* Terminal MUST be >= running (Riccati requires positive semi-definite cost-to-go) */
    cfg.weight_contouring_terminal = env_double("Q_CONTOURING_TERM", 4800.0f);
    cfg.weight_lag_terminal     = env_double("Q_LAG_TERM", 800.0f);
    cfg.weight_progress_terminal = env_double("Q_PROGRESS_TERM", 41.4f);

    /* Obstacle */
    cfg.weight_obstacle   = env_double("W_OBSTACLE", 1000.0f);
    cfg.obstacle_margin   = env_double("OBSTACLE_MARGIN", 0.1f);

    /* ADMM config is only used by the ADMM/Riccati build. OSQP settings are
     * read directly by qp_solver_osqp.c from OSQP_* environment variables. */
    cfg.admm_rho            = env_double("ADMM_RHO", 5.0f);
    cfg.admm_max_iterations = env_int("ADMM_MAX_ITER", 300);
    cfg.admm_tolerance      = env_double("ADMM_TOL", 0.02f);
    cfg.admm_rho_u          = env_double("ADMM_RHO_U", 0.0f);
    cfg.admm_adaptive_rho   = (uint8_t)env_int("ADMM_ADAPTIVE_RHO", 1);
    cfg.admm_alpha_relax    = env_double("ADMM_ALPHA_RELAX", 1.6f);
    cfg.accept_max_iterations = (uint8_t)env_int("MPCC_ACCEPT_MAX_ITER", MPCC_DEFAULT_ACCEPT_MAX_ITERATIONS);
    cfg.max_iter_primal_tolerance = env_double("MPCC_MAX_ITER_PRIMAL_TOL", MPCC_DEFAULT_MAX_ITER_PRIMAL_TOL);
    cfg.max_iter_dual_tolerance = env_double("MPCC_MAX_ITER_DUAL_TOL", MPCC_DEFAULT_MAX_ITER_DUAL_TOL);
    cfg.max_iter_track_violation_tolerance = env_double("MPCC_MAX_ITER_TRACK_TOL", MPCC_DEFAULT_MAX_ITER_TRACK_VIOLATION_TOL);

    /* Constraint bounds */
    cfg.delta_max = env_double("DELTA_MAX", F110_DEFAULT_MAXIMUM_STEERING_RADIANS);
    cfg.ax_max    = env_double("AX_MAX", 7.0f);
    cfg.ax_min    = env_double("AX_MIN", -10.0f);
    cfg.vx_max    = env_double("VX_MAX", 20.0f);
    cfg.vx_min    = env_double("VX_MIN", 0.0f);
    cfg.v_theta_max = env_double("V_THETA_MAX", 10.0f);
    cfg.v_theta_min = env_double("V_THETA_MIN", 0.0f);

    /* Tire parameters */
    cfg.mu   = env_double("MU", 0.745f);
    cfg.C_Sf = env_double("C_SF", 4.297f);
    cfg.C_Sr = env_double("C_SR", 3.473f);

    /* Cross-call rate scaling */
    if (cross_call_scale_env != NULL) {
        cfg.cross_call_rate_scale = (float)atof(cross_call_scale_env);
    } else if (cfg.dt > 0.0f) {
        cfg.cross_call_rate_scale = (float)(CONTROL_DT / (double)cfg.dt);
    } else {
        cfg.cross_call_rate_scale = MPCC_DEFAULT_CROSS_CALL_SCALE;
    }

    if (verbose) {
        printf("  Config: N=%d dt=%.3f Q_c=%.1f Q_l=%.1f Q_prog=%.1f\n",
               cfg.horizon_steps, cfg.dt,
               cfg.weight_contouring, cfg.weight_lag,
               cfg.weight_progress);
         printf("  Q_wall=%.1f wall_margin=%.3f track_buffer=%.3f\n",
             cfg.weight_wall_clearance, cfg.wall_clearance_margin,
             cfg.track_safety_buffer);
        printf("  Q_vx=%.1f vx_ref=%.1f use_csv_vx_ref=%u use_csv_vx_limit=%u R_delta=%.2f R_ax=%.3f R_vt=%.2f\n",
               cfg.weight_vx, cfg.vx_ref,
               (unsigned)cfg.use_raceline_vx_ref,
               (unsigned)cfg.use_raceline_vx_limit,
               cfg.weight_delta, cfg.weight_ax,
               cfg.weight_v_theta);
#ifdef USE_OSQP
        printf("  Solver: OSQP (OSQP_* environment settings)\n");
#else
        printf("  Solver: ADMM rho=%.2f max_iter=%d tol=%.4f\n",
               cfg.admm_rho, cfg.admm_max_iterations,
               cfg.admm_tolerance);
#endif
         printf("  control_dt=%.4f cross_call=%.4f\n",
             CONTROL_DT, cfg.cross_call_rate_scale);
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
    int consecutive_failures = 0;  /* for proportional fallback */
    float current_s = 0;
    int odom_update_count = 0;
    int last_solve_odom_update_count = -1;
    int s_backward_jumps = 0;
    int s_large_corrections = 0;
    int s_prediction_regressions = 0;
    double max_s_estimator_jump = 0.0;
    double max_predicted_s_span = 0.0;
    double last_pred_min_wall_slack = 0.0;
    int last_pred_slack_valid = 0;

    /* Lap tracking */
    int    lap_count     = 0;
    double lap_start_time = 0.0;
    double best_lap_time  = 9999.0;
    double first_lap_time = 0.0;
    float  prev_s         = 0.0f;
    float  track_length   = g_ref_path.total_length;
    double lap_distance_traveled = 0.0;
    double lap_prev_x = (double)state.pos_x;
    double lap_prev_y = (double)state.pos_y;

    /* ST model persistent state */
    double st_delta = 0.0;
    double st_V = 0.0;
    double st_psi_dot = 0.0;
    double st_beta = 0.0;
    int st_initialized = 0;

    int stepped = 0;

    if (trace_csv_path && trace_csv_path[0] != '\0') {
        trace_csv = fopen(trace_csv_path, "w");
        if (!trace_csv) {
            fprintf(stderr, "ERROR: Cannot open MPCC_TRACE_CSV_PATH=%s for writing\n", trace_csv_path);
            return 1;
        }
        fprintf(trace_csv,
            "step,time_s,x_m,y_m,psi_rad,vx_mps,s_m,closest_wp,ref_s_m,e_y_m,e_c_m,e_psi_rad,"
                "left_bound_m,right_bound_m,effective_left_bound_m,effective_right_bound_m,"
                "body_safety_margin_m,cmd_steer_rad,act_steer_rad,accel_cmd_mps2,"
                "status,iterations,pred_min_wall_slack_m,wall_hit,lap_count,"
                "dx_exec_pred,dy_exec_pred,dpsi_exec_pred,ds_exec_pred\n");
    }

    if (verbose) {
        printf("\n  Step | Time  | vx    | v_cmd | e_y   | e_psi | cmd_st | act_st | accel | st | it\n");
        printf("  -----|-------|-------|-------|-------|-------|--------|--------|-------|----|---\n");
    }

    for (int step = 0; step < SIM_STEPS; step++) {
        double t = step * SIM_DT;
        double s_debug_now = current_s;
        double s_debug_pred1 = current_s;
        double s_debug_predN = current_s;
        double pred_min_wall_slack = last_pred_min_wall_slack;
        int pred_slack_valid = last_pred_slack_valid;
        int s_debug_valid = 0;
        double px = (double)(state.pos_x);
        double py = (double)(state.pos_y);
        double psi = (double)(state.heading);
        double vx = (double)(state.long_vel);
        int odom_tick = (step % ODOM_UPDATE_INTERVAL) == 0;
        int pose_tick = (step % POSE_UPDATE_INTERVAL) == 0;

        /* Track physical distance traveled for robust lap validation.
         * This prevents counting "laps" from s-estimator wrap when the car is
         * stationary or not actually driving the track. */
        {
            double dx_lap = px - lap_prev_x;
            double dy_lap = py - lap_prev_y;
            lap_distance_traveled += sqrt((dx_lap * dx_lap) + (dy_lap * dy_lap));
            lap_prev_x = px;
            lap_prev_y = py;
        }

        // Executed state minus previous predicted state (for model/actuation mismatch diagnosis)
        static double prev_pred_X = 0.0, prev_pred_Y = 0.0, prev_pred_psi = 0.0, prev_pred_s = 0.0;
        double dx_exec_pred = 0.0, dy_exec_pred = 0.0, dpsi_exec_pred = 0.0, ds_exec_pred = 0.0;
        if (step > 0) {
            dx_exec_pred = px - prev_pred_X;
            dy_exec_pred = py - prev_pred_Y;
            dpsi_exec_pred = wrap_angle(psi - prev_pred_psi);
            ds_exec_pred = s_debug_now - prev_pred_s;
        }

        if (odom_tick)
            odom_update_count++;

        if (!isfinite(px) || !isfinite(py) || !isfinite(psi) || !isfinite(vx)) {
            numerical_failures++;
            printf("\n  !!! NUMERICAL FAILURE at step=%d t=%.2f !!!\n", step, t);
            stepped = step;
            break;
        }

        if (vx > max_vx) max_vx = vx;

        int closest = find_closest_waypoint(px, py, psi);
        MPCCPathPoint_t metrics_path_pt;
        mpcc_path_interpolate(&g_ref_path, s_debug_now, &metrics_path_pt);

        /* Compute Frenet error for metrics */
        double dx = px - (double)metrics_path_pt.x_ref;
        double dy = py - (double)metrics_path_pt.y_ref;
        double path_psi = (double)metrics_path_pt.phi_ref;
        double e_y = -dx * sin(path_psi) + dy * cos(path_psi);
        double e_c = -e_y;  /* MPCC core uses positive = right of reference. */
        double e_psi = wrap_angle(psi - path_psi);

        /* Wall collision check (body-edge) — abort on first hit.
         * Use the same contouring-error sign convention as the MPCC core:
         * e_c > 0 lies to the right of the reference, e_c < 0 to the left. */
        double effective_left_bound = (double)metrics_path_pt.left_bound;
        double effective_right_bound = (double)metrics_path_pt.right_bound;
        double left_wall  = effective_left_bound + VEHICLE_HALF_WIDTH + body_safety_margin;
        double right_wall = effective_right_bound + VEHICLE_HALF_WIDTH + body_safety_margin;
        int wall_hit = 0;
        if (e_c > effective_right_bound) { wall_hit = 1; }
        if (e_c < -effective_left_bound) { wall_hit = -1; }
        if (wall_hit && !prev_wall_hit) {
            wall_collisions++;
            if (trace_csv) {
                fprintf(trace_csv,
                    "%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%.6f,%d,%d,%.6f,%.6f,%.6f,%.6f\n",
                    step, t, px, py, psi, vx,
                    s_debug_now, closest, raceline[closest].s,
                    e_y, e_c, e_psi,
                    left_wall, right_wall,
                    effective_left_bound, effective_right_bound,
                    body_safety_margin,
                    cmd_steer, actual_steer, cmd_accel,
                    -1, 0,
                    pred_slack_valid ? pred_min_wall_slack : 0.0,
                    wall_hit, lap_count,
                    dx_exec_pred, dy_exec_pred, dpsi_exec_pred, ds_exec_pred);
                fflush(trace_csv);
            }
            if (verbose) {
                double bound = (wall_hit > 0)
                             ? effective_right_bound
                             : -effective_left_bound;
                printf("\n  !!! WALL COLLISION: e_c=%.3f (bound:%.3f) step=%d t=%.2f wp=%d v=%.1f — aborting !!!\n",
                       e_c, bound, step, t, closest, vx);
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

        if (pose_tick && odom_update_count != last_solve_odom_update_count) {
            /* Convert vehicle state -> MPCC state */
            MPCCState_t mpcc_state = mpcc_state_from_vehicle_state(&state, current_s);
            {
                double s_jump = wrap_s_delta((double)mpcc_state.s - (double)current_s,
                                             (double)track_length);
                double s_jump_abs = fabs(s_jump);

                if (s_jump < -0.05)
                    s_backward_jumps++;
                if (s_jump_abs > 0.75)
                    s_large_corrections++;
                if (s_jump_abs > max_s_estimator_jump)
                    max_s_estimator_jump = s_jump_abs;
            }
            current_s = mpcc_state.s;
            s_debug_now = current_s;

            /* Lap detection: s wrapped past track_length (min lap time guard) */
            double min_lap_distance = 0.8 * (double)track_length;
            if (track_length > 1e-3f
                && lap_distance_traveled > min_lap_distance
                && current_s < prev_s - track_length * 0.5f
                && prev_s > track_length * 0.5f
                && (t - lap_start_time) > 2.0) {
                double lap_time = t - lap_start_time;
                lap_count++;
                if (lap_count == 1) first_lap_time = lap_time;
                if (lap_time < best_lap_time) best_lap_time = lap_time;
                lap_start_time = t;
                lap_distance_traveled = 0.0;
                if (verbose) {
                    printf("  >>> LAP %d completed: %.3f s (best: %.3f s) <<<\n",
                           lap_count, lap_time, best_lap_time);
                }
            }
            prev_s = current_s;

            /* Solve */
            MPCCResult_t result;
            clock_gettime(CLOCK_MONOTONIC_RAW, &ts0);
            MPCCStatus_t status = mpcc_compute_control(&mpcc_state, &result);
            clock_gettime(CLOCK_MONOTONIC_RAW, &ts1);
            last_solve_odom_update_count = odom_update_count;

            if (cfg.horizon_steps > 0) {
                double ds1 = wrap_s_delta((double)result.predicted_states[1].s
                                          - (double)result.predicted_states[0].s,
                                          (double)track_length);
                double dsN = wrap_s_delta((double)result.predicted_states[cfg.horizon_steps].s
                                          - (double)result.predicted_states[0].s,
                                          (double)track_length);

                s_debug_pred1 = result.predicted_states[1].s;
                s_debug_predN = result.predicted_states[cfg.horizon_steps].s;
                s_debug_valid = 1;

                if (ds1 <= 0.0 || dsN <= ds1)
                    s_prediction_regressions++;
                if (dsN > max_predicted_s_span)
                    max_predicted_s_span = dsN;

                pred_min_wall_slack = compute_predicted_min_wall_slack(&result, cfg.horizon_steps);
                pred_slack_valid = 1;
                last_pred_min_wall_slack = pred_min_wall_slack;
                last_pred_slack_valid = pred_slack_valid;

                // Save predicted state for next step's executed-minus-predicted diagnostic
                prev_pred_X = result.predicted_states[1].X;
                prev_pred_Y = result.predicted_states[1].Y;
                prev_pred_psi = result.predicted_states[1].psi;
                prev_pred_s = result.predicted_states[1].s;
            }

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

            /* Track only hard solver failures for proportional fallback.
             * MAX_ITERATIONS still returns a usable control in this MPCC,
             * so treating it as a failure can override the optimizer with a
             * saturated feedforward steer right when the car is cornering. */
            if (status == MPCC_STATUS_INFEASIBLE ||
                status == MPCC_STATUS_ERROR) {
                consecutive_failures++;
            } else {
                consecutive_failures = 0;
            }

            /* Feedforward-only fallback: if the solver hard-fails 3+ times in a row,
             * replace steering with kappa-based feedforward + proportional
             * corrections for heading error and lateral error. */
            if (consecutive_failures >= 3) {
                int la_wp = (closest + 8) % raceline_count;
                double kappa_la = raceline[la_wp].kappa;
                double L = 0.326;
                /* Feedforward from path curvature */
                double steer_ff = atan(L * kappa_la);
                /* Proportional feedback on heading error and lateral error */
                double k_psi = 0.6;   /* heading gain */
                double k_ey  = 0.2;   /* lateral gain */
                steer = steer_ff - k_psi * e_psi - k_ey * e_y;
                if (steer >  MAX_STEERING) steer =  MAX_STEERING;
                if (steer < -MAX_STEERING) steer = -MAX_STEERING;
                /* Keep solver's accel_cmd as-is */
                status_val = 2;
            }

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

        if (trace_csv) {
            fprintf(trace_csv,
                "%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%.6f,%d,%d,%.6f,%.6f,%.6f,%.6f\n",
                step, t, px, py, psi, vx,
                s_debug_now, closest, raceline[closest].s,
                e_y, e_c, e_psi,
                left_wall, right_wall,
                effective_left_bound, effective_right_bound,
                body_safety_margin,
                steer, actual_steer, accel_cmd,
                status_val, iter,
                pred_slack_valid ? pred_min_wall_slack : 0.0,
                wall_hit, lap_count,
                dx_exec_pred, dy_exec_pred, dpsi_exec_pred, ds_exec_pred);
        }

        if (verbose) {
            int print_row = (step < 40) || (step % 10 == 0) || wall_hit || (fabs(e_y) > 0.8);
            if (print_row) {
                printf("  %4d | %5.2f | %5.2f | %5.2f | %+.3f | %+.3f | %+.4f | %+.4f | %+.2f | %2d | %3d\n",
                       step, t, vx, raceline[closest].vx, e_y, e_psi,
                       steer, actual_steer, accel_cmd, status_val, iter);
                if (s_debug_valid) {
                    printf("       SDBG | s=%6.2f | p1=%6.2f | pN=%6.2f | ds1=%+.3f | dsN=%+.3f\n",
                           s_debug_now,
                           s_debug_pred1,
                           s_debug_predN,
                           wrap_s_delta(s_debug_pred1 - s_debug_now, (double)track_length),
                           wrap_s_delta(s_debug_predN - s_debug_now, (double)track_length));
                }
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

    printf("\n  === Results (%.0fs, MPCC Lifted ODE, %s) ===\n",
           SIM_DURATION_S, MPCC_SOLVER_NAME);
    printf("  Solver success:     %d / %d (%.1f%%)\n", solver_ok, solver_calls,
           100.0*solver_ok/(solver_calls > 0 ? solver_calls : 1));
    printf("  Avg speed:          %.2f m/s\n", avg_speed);
    printf("  Laps completed:     %d\n", lap_count);
    if (lap_count > 0) {
        printf("  Best lap time:      %.3f s\n", best_lap_time);
        printf("  First lap time:     %.3f s\n", first_lap_time);
    }
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
    printf("  S backward jumps:   %d\n", s_backward_jumps);
    printf("  Large s corrections:%d\n", s_large_corrections);
    printf("  Max s jump:         %.3f m\n", max_s_estimator_jump);
    printf("  S regressions:      %d\n", s_prediction_regressions);
    printf("  Max predicted ds:   %.3f m\n", max_predicted_s_span);
    printf("  Time above 2 m/s:   %.1f / %.1f s (%.0f%%)\n",
           time_above_2ms, SIM_DURATION_S, 100*time_above_2ms/SIM_DURATION_S);
    printf("  Time above 5 m/s:   %.1f / %.1f s (%.0f%%)\n",
           time_above_5ms, SIM_DURATION_S, 100*time_above_5ms/SIM_DURATION_S);
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

        /* Pass/fail criteria: numerical safety + usable lap-driving */
    check("No numerical failures", numerical_failures == 0);
    check("Runs full simulation horizon", total_stepped == SIM_STEPS);
    check("Solver mostly succeeds (>50%)", solver_ok > solver_calls * 50 / 100);
        check("Completes at least one lap", lap_count > 0);
    check("Sustains motion (>2 m/s for >5% of time)",
          time_above_2ms > SIM_DURATION_S * 0.05);
    check("High top speed (>5 m/s)", max_vx > 5.0);

    printf("\n=== RESULTS: %d passed, %d failed ===\n", tests_passed, tests_failed);

    /* Machine-readable CSV for tuning scripts */
    if (getenv("MPCC_TUNING_CSV")) {
           printf("CSV,%d,%d,%.4f,%.4f,%.4f,%.4f,%.2f,%.1f,%.1f,%d,%.1f,%.4f,%.4f,%.1f,%.3f,%.3f,%.2f,%.2f,%.4f,%d,%.4f,%d,%d,%.4f,%d,%.4f\n",
               tests_passed, tests_failed,
               max_lat_err, avg_lat, max_hdg_err, avg_hdg,
               max_vx, avg_solve, max_solve_us,
               wall_collisions, time_above_5ms,
               max_vel_err, avg_vel_err, avg_iters,
               avg_rho, avg_rho_u, avg_adapt_updates, avg_clip_events,
               avg_speed,
               lap_count, best_lap_time,
               s_backward_jumps,
               s_large_corrections,
               max_s_estimator_jump,
               s_prediction_regressions,
               max_predicted_s_span);
    }
    if (trace_csv) {
        fclose(trace_csv);
        printf("  Trace CSV:           %s\n", trace_csv_path);
    }
    return tests_failed > 0 ? 1 : 0;
}
