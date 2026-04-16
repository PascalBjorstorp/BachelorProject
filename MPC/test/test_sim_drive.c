/**
 * @file test_sim_drive.c
 * @brief Realistic MPC simulation on my_track raceline
 * @details Runs closed-loop simulation of the Riccati-ADMM controller with
 *          configurable physics/control rates and optional realistic effects
 *          (noise, delay, nonlinear tire saturation, drivetrain drag).
 *
 * Tests the Riccati-ADMM MPC controller in a closed-loop simulation:
 *   - SIM_DT = sim physics step (default 5ms = 200Hz, configurable via env)
 *   - MPC_DT = MPC control interval (default 5ms = 200Hz, configurable via env)
 *   - Higher sim frequencies (e.g. SIM_DT=0.001 = 1000Hz) with MPC at 200Hz
 *     better approximate continuous dynamics (real-world behavior)
 *   - v_cmd = raceline velocity
 *   - Wall bounds from the CSV
 *   - Nonlinear single-track vehicle model (matching f1tenth_gym)
 *   - Spawn at raceline[0]
 *   - Runs for 60 seconds
 *
 * Reports: wall collisions, max/avg lateral error, steering behavior,
 *          velocity tracking, and step-by-step diagnostics near crashes.
 *
 * Compile (standalone):
 *   cd MPC
 *   gcc -D_GNU_SOURCE -O3 -std=c99 -Wall -ffast-math \
 *       -Wno-unused-variable -Wno-unused-but-set-variable \
 *       -Iinclude test/test_sim_drive.c src/mpc.c \
 *       src/riccati_solver.c src/vehicle_model.c src/util_math.c \
 *       -o test_sim_drive -lm
 * @dependencies mpc.h, mpc_types.h, vehicle_model.h,
 *               <stdio.h>, <stdlib.h>, <string.h>, <math.h>, <time.h>
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

/*===========================================================================
 * Configuration
 *===========================================================================*/

/* Default simulation physics integration step; a finer step improves
 * continuous-time accuracy but increases total compute time. */
#define SIM_DT_DEFAULT    0.005

/* Default MPC re-computation interval; should match the hardware control rate
 * for faithful simulation of the deployed system. */
#define MPC_DT_DEFAULT    0.005
#define SIM_DURATION_DEFAULT 100.0  /* seconds */
#define MAX_WAYPOINTS     2000
#define MAX_STEERING      0.4189 /* rad — calibrated limit (with polynomial servo correction) */
#define MAX_VELOCITY      20.0   /* m/s */
#define PHYSICAL_MAX_ACCEL 7.31  /* m/s² — bounded by mu*g */

/* Trajectory pre-processing (matching gym_bridge ROS2 node exactly) */
#define TRAJECTORY_MAX_VELOCITY   20.0
#define VEHICLE_HALF_WIDTH        0.137   /* meters — for body-edge collision */
#define DEFAULT_BODY_SAFETY_MARGIN 0.06   /* extra margin: gym bitmap is stricter */
#define STEER_BUFFER_SIZE         2       /* matching gym steer_buffer_size */

/*===========================================================================
 * Realistic Simulation Enhancements (enabled with REALISTIC_SIM=1 env var)
 *
 * When enabled, the plant model adds real-world effects NOT in the gym:
 *   1. Rolling resistance (F_roll = 2.79 N) + drivetrain efficiency (57.5%)
 *   2. Pacejka tire saturation (replaces linear Fy = mu*Cs*alpha*Fz)
 *   3. 1-step MPC computation delay (control computed at t applied at t+dt)
 *   4. Sensor noise (position ±2cm, heading ±1.5°, velocity ±0.1 m/s)
 * These values come from measured vehicle parameters in vehicle_params.yaml.
 *===========================================================================*/
#define ROLLING_RESISTANCE_N      2.79    /* Measured: vehicle_params.yaml L176 */
#define PACEJKA_C_SHAPE           1.9     /* Shape factor for Pacejka tire */
/* Noise std-devs matching real sensor characteristics.
 * Note: VESC ACCEL_TO_CURRENT mode compensates for rolling resistance and
 * drivetrain efficiency, so a_max=7.31 respects measured traction limits.
 * Rolling resistance is modeled as speed-dependent aerodynamic+friction drag
 * that the VESC cannot fully compensate at high speed. */
#define NOISE_POS_M               0.01    /* AMCL position noise (m) */
#define NOISE_HDG_RAD             0.009   /* AMCL heading noise (~0.5 deg) */
#define NOISE_VX_MS               0.05    /* ERPM velocity noise (m/s) */
#define NOISE_VY_MS               0.05    /* Lateral vel (usually set to 0) */
#define NOISE_OMEGA_RAD           0.05    /* IMU yaw rate noise (rad/s) */

/*===========================================================================
 * Raceline Data
 *===========================================================================*/

typedef struct {
    double s, x, y, psi, kappa, vx, ax;
    double left_bound, right_bound;
} Waypoint_t;

typedef struct {
    double s;
    double lateral_error;
    double heading_error;
} PathProjection_t;

static Waypoint_t raceline[MAX_WAYPOINTS];
static int raceline_count = 0;
static double g_mpc_prediction_dt = TIME_STEP_SECONDS;  /* Set from PRED_DT env or default */
static double g_track_length_m = 0.0;

/* Read a double-valued environment override, falling back when unset/invalid. */
static double get_env_double(const char *name, double fallback)
{
    const char *env = getenv(name);
    if (env == NULL || env[0] == '\0') {
        return fallback;
    }
    return atof(env);
}

/* Load raceline waypoints from an environment path or known fallback paths.
 * Side effect: fills global raceline buffer and track-length metadata.
 * Returns non-zero when at least one waypoint is parsed successfully. */
static int load_raceline(void)
{
    raceline_count = 0;

    /* Allow env var override for raceline path (tuning with different tracks) */
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
                if (n >= 9) {
                    raceline_count++;
                } else if (n >= 7) {
                    /* Older racelines do not include wall bounds. */
                    wp->left_bound = 5.0;
                    wp->right_bound = 5.0;
                    raceline_count++;
                }
            }
            fclose(f);

            /* Keep behavior aligned with default loader path. */
            for (int i = 0; i < raceline_count; i++) {
                double sv = raceline[i].vx * TRAJECTORY_SPEED_GAIN;
                if (sv > TRAJECTORY_MAX_VELOCITY) sv = TRAJECTORY_MAX_VELOCITY;
                if (sv < 0.0) sv = 0.0;
                raceline[i].vx = sv;
            }

            if (raceline_count >= 2) {
                g_track_length_m = raceline[raceline_count - 1].s - raceline[0].s;
                if (g_track_length_m < 1e-3) g_track_length_m = 0.0;
            }

            printf("  Loaded %d waypoints\n", raceline_count);
            return raceline_count > 0;
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
    if (!f) { fprintf(stderr, "ERROR: Cannot open my_track_raceline.csv\n"); return 0; }

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

    /* === Trajectory speed gain === */
    for (int i = 0; i < raceline_count; i++) {
        double sv = raceline[i].vx * TRAJECTORY_SPEED_GAIN;
        if (sv > TRAJECTORY_MAX_VELOCITY) sv = TRAJECTORY_MAX_VELOCITY;
        if (sv < 0.0) sv = 0.0;
        raceline[i].vx = sv;
    }

    printf("[LOAD] %d waypoints (speed_gain=%.2f, v: %.1f-%.1f m/s)\n",
           raceline_count, TRAJECTORY_SPEED_GAIN,
           raceline[0].vx, raceline[raceline_count/2].vx);

    if (raceline_count >= 2) {
        g_track_length_m = raceline[raceline_count - 1].s - raceline[0].s;
        if (g_track_length_m < 1e-3) g_track_length_m = 0.0;
    }
    return raceline_count > 0;
}

/*===========================================================================
 * Helpers
 *===========================================================================*/

/* Wrap heading angle to the principal interval [-pi, pi].
 * Parameter: a is heading angle in radians.
 * Returns wrapped angle in radians. */
static double wrap_angle(double a)
{
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

static int last_closest = 0;

/* Find nearest raceline waypoint to the vehicle pose.
 * Parameters: px/py in meters and heading in radians.
 * Returns nearest waypoint index and updates cached index for next query. */
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
        /* Reject waypoints with opposite heading (prevents closure wrap) */
        double hdg_diff = heading - raceline[idx].psi;
        while (hdg_diff >  M_PI) hdg_diff -= 2*M_PI;
        while (hdg_diff < -M_PI) hdg_diff += 2*M_PI;
        if (fabs(hdg_diff) > 1.57 && d2 < 4.0) continue;
        if (d2 < best_dist) { best_dist = d2; best = idx; }
    }
    last_closest = best;
    return best;
}

/* Convert world-frame vehicle state to Frenet tracking state at waypoint wp.
 * Parameters: v points to vehicle state, wp is waypoint index.
 * Returns FrenetState_t with lateral/heading error and body velocities. */
/* Wrap arc-length coordinate for closed-loop track interpolation.
 * Parameter: s is arc length in meters.
 * Returns wrapped arc length in meters. */
static double wrap_track_s(double s)
{
    if (g_track_length_m <= 1e-6 || raceline_count <= 1) return s;
    double s0 = raceline[0].s;
    while (s < s0) s += g_track_length_m;
    while (s >= s0 + g_track_length_m) s -= g_track_length_m;
    return s;
}

/* Project a pose to the local raceline segment used by the hardware node.
 * Parameters: px/py/psi are world pose values, closest_index is the active
 * nearest waypoint.
 * Returns interpolated Frenet errors together with projected track arc length. */
static PathProjection_t project_pose_to_raceline(
    double px, double py, double psi, int closest_index)
{
    PathProjection_t out = {0};

    if (raceline_count <= 0) return out;

    int idx0 = closest_index;
    if (idx0 < 0) idx0 = 0;
    if (idx0 >= raceline_count) idx0 = raceline_count - 1;
    int idx1 = (idx0 + 1) % raceline_count;

    double ax = raceline[idx0].x;
    double ay = raceline[idx0].y;
    double bx = raceline[idx1].x;
    double by = raceline[idx1].y;

    double abx = bx - ax;
    double aby = by - ay;
    double apx = px - ax;
    double apy = py - ay;
    double ab_len2 = abx * abx + aby * aby;
    double t = 0.0;
    if (ab_len2 > 1e-12)
        t = (apx * abx + apy * aby) / ab_len2;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;

    double path_x = ax + t * abx;
    double path_y = ay + t * aby;

    double h0 = raceline[idx0].psi;
    double h1 = raceline[idx1].psi;
    double dh = h1 - h0;
    while (dh > M_PI) dh -= TWO_PI;
    while (dh < -M_PI) dh += TWO_PI;
    double path_psi = wrap_angle(h0 + t * dh);

    double dx = px - path_x;
    double dy = py - path_y;
    out.lateral_error = -dx * sin(path_psi) + dy * cos(path_psi);
    out.heading_error = wrap_angle(psi - path_psi);

    double s0 = raceline[idx0].s;
    double s1 = raceline[idx1].s;
    if (idx0 == raceline_count - 1 && g_track_length_m > 1e-6)
        s1 += g_track_length_m;
    out.s = s0 + t * (s1 - s0);
    out.s = wrap_track_s(out.s);
    return out;
}

/* Convert world-frame vehicle state to Frenet tracking state at waypoint wp.
 * Parameters: v points to vehicle state, wp is waypoint index.
 * Returns FrenetState_t with lateral/heading error and body velocities. */
static FrenetState_t vehicle_to_frenet(const VehicleState_t *v, int wp)
{
    FrenetState_t f;
    PathProjection_t proj = project_pose_to_raceline(
        (double)(v->pos_x),
        (double)(v->pos_y),
        (double)(v->heading),
        wp);
    f.flat_error = (float)(proj.lateral_error);
    f.fhead_error = (float)(proj.heading_error);
    f.flong_vel = v->long_vel;
    f.flat_vel = v->lat_vel;
    f.fyaw_rate = v->yaw_rate;
    return f;
}

/* Interpolate raceline waypoint values at requested arc length.
 * Parameter: s_query is arc length in meters.
 * Returns interpolated waypoint sample. */
static Waypoint_t sample_raceline_by_s(double s_query)
{
    if (raceline_count <= 0) {
        Waypoint_t empty = {0};
        return empty;
    }
    if (raceline_count == 1 || g_track_length_m <= 1e-6) {
        return raceline[0];
    }

    const double s = wrap_track_s(s_query);
    for (int i = 0; i < raceline_count - 1; i++) {
        const Waypoint_t *w0 = &raceline[i];
        const Waypoint_t *w1 = &raceline[i + 1];
        if (s >= w0->s && s <= w1->s) {
            const double denom = w1->s - w0->s;
            const double t = (denom > 1e-9) ? ((s - w0->s) / denom) : 0.0;
            Waypoint_t out = *w0;
            out.s = s;
            out.x = w0->x + (w1->x - w0->x) * t;
            out.y = w0->y + (w1->y - w0->y) * t;
            out.psi = w0->psi + (w1->psi - w0->psi) * t;
            out.kappa = w0->kappa + (w1->kappa - w0->kappa) * t;
            out.vx = w0->vx + (w1->vx - w0->vx) * t;
            out.ax = w0->ax + (w1->ax - w0->ax) * t;
            out.left_bound = w0->left_bound + (w1->left_bound - w0->left_bound) * t;
            out.right_bound = w0->right_bound + (w1->right_bound - w0->right_bound) * t;
            return out;
        }
    }

    const Waypoint_t *w0 = &raceline[raceline_count - 1];
    const Waypoint_t *w1 = &raceline[0];
    const double s1 = w1->s + g_track_length_m;
    double s_adj = s;
    if (s_adj < w0->s) s_adj += g_track_length_m;
    const double denom = s1 - w0->s;
    const double t = (denom > 1e-9) ? ((s_adj - w0->s) / denom) : 0.0;
    Waypoint_t out = *w0;
    out.s = s;
    out.x = w0->x + (w1->x - w0->x) * t;
    out.y = w0->y + (w1->y - w0->y) * t;
    out.psi = w0->psi + (w1->psi - w0->psi) * t;
    out.kappa = w0->kappa + (w1->kappa - w0->kappa) * t;
    out.vx = w0->vx + (w1->vx - w0->vx) * t;
    out.ax = w0->ax + (w1->ax - w0->ax) * t;
    out.left_bound = w0->left_bound + (w1->left_bound - w0->left_bound) * t;
    out.right_bound = w0->right_bound + (w1->right_bound - w0->right_bound) * t;
    return out;
}

/* Build horizon reference trajectory for the MPC from raceline samples.
 * Parameters: closest waypoint index, horizon steps, output ref array.
 * Side effect: writes MPC reference entries into ref[0..horizon_steps-1]. */
static void build_reference(int closest, int horizon_steps, TrajectoryReferencePoint_t *ref)
{
    if (ref == NULL) return;
    if (horizon_steps < 1) return;
    if (horizon_steps > PREDICTION_HORIZON) horizon_steps = PREDICTION_HORIZON;

    double s_query = raceline[closest].s;
    double step_velocity = fabs(raceline[closest].vx);
    if (step_velocity < 1.0) step_velocity = 1.0;

    /* Curvature-based velocity limiting.
     * Caps reference velocity to what the tires can support at each curvature.
     * v_max_curve = sqrt(a_lat_max / |kappa|)   — lateral grip limit
     * Active in realistic mode only. */
    static int use_vlimit = -1;
    static double a_lat_max = 7.3078;  /* m/s² — mu*g = 0.745 * 9.81 */
    if (use_vlimit < 0) {
        use_vlimit = (getenv("REALISTIC_SIM") && atoi(getenv("REALISTIC_SIM")))
                  || (getenv("REALISTIC_TIRES") && atoi(getenv("REALISTIC_TIRES")))
                  || (getenv("REALISTIC_DRIVE") && atoi(getenv("REALISTIC_DRIVE")))
                  || (getenv("REALISTIC_DELAY") && atoi(getenv("REALISTIC_DELAY")))
                  || (getenv("REALISTIC_NOISE") && atoi(getenv("REALISTIC_NOISE")));
        const char *al = getenv("MAX_LAT_ACCEL");
        if (al) a_lat_max = atof(al);
    }

    for (int step = 0; step < horizon_steps; step++) {
        s_query += step_velocity * g_mpc_prediction_dt;
        Waypoint_t wp = sample_raceline_by_s(s_query);

        ref[step].reference_lateral_error = 0;
        ref[step].reference_heading_error = 0;

        double v_ref = wp.vx;

        /* Velocity limiting: curvature-based (realistic mode only). */
        if (use_vlimit) {
            double abs_kappa = fabs(wp.kappa);
            if (abs_kappa > 0.01) {
                double v_curv = sqrt(a_lat_max / abs_kappa);
                if (v_ref > v_curv) v_ref = v_curv;
            }
        }

        if (v_ref < 1.0) v_ref = 1.0;
        if (v_ref > TRAJECTORY_MAX_VELOCITY) v_ref = TRAJECTORY_MAX_VELOCITY;
        step_velocity = v_ref;

        ref[step].reference_velocity = (float)(v_ref);

        /* Clamp kappa to physical limits */
        double kappa = wp.kappa;
        if (kappa > 1.5) kappa = 1.5;
        if (kappa < -1.5) kappa = -1.5;

        /* vy reference: zero */
        ref[step].reference_lateral_velocity = 0;

        ref[step].reference_yaw_rate = (float)(kappa * v_ref);


        ref[step].path_curvature = (float)(kappa);
        ref[step].left_wall_bound = (float)(wp.left_bound);
        ref[step].right_wall_bound = (float)(wp.right_bound);
    }
}

/*===========================================================================
 * Main Simulation
 *===========================================================================*/

static int tests_passed = 0, tests_failed = 0;

/* Record test assertion result and print pass/fail status line.
 * Parameters: name is assertion label, cond is boolean predicate.
 * Side effect: increments global pass/fail counters. */
static void check(const char *name, int cond)
{
    if (cond) { tests_passed++; printf("  [PASS] %s\n", name); }
    else       { tests_failed++; printf("  [FAIL] %s\n", name); }
}

/* Box-Muller Gaussian random number generator (for sensor noise) */
/* Generate one approximately normal-distributed random sample.
 * Returns N(0,1) sample using Box-Muller transform. */
static double randn(void)
{
    double u1 = ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);
    double u2 = ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

int main(void)
{
    /* Runtime-configurable timesteps:
     *   SIM_DT  = physics simulation timestep (env SIM_DT, default 5ms = 200Hz)
     *   MPC_DT  = MPC control interval (env MPC_DT, default 5ms = 200Hz)
     * The MPC is called every MPC_DT/SIM_DT simulation steps.
     * Higher SIM_DT frequencies (e.g. 1000Hz) better approximate continuous
     * dynamics while keeping MPC at a fixed rate (e.g. 200Hz). */
    const char *dt_env = getenv("SIM_DT");
    const double sim_dt_raw = dt_env ? atof(dt_env) : SIM_DT_DEFAULT;
    const double SIM_DT = (sim_dt_raw > 1e-6) ? sim_dt_raw : SIM_DT_DEFAULT;

    const char *mpc_dt_env = getenv("MPC_DT");
    const double mpc_dt_raw = mpc_dt_env ? atof(mpc_dt_env) : MPC_DT_DEFAULT;
    const double MPC_DT = (mpc_dt_raw > 1e-6) ? mpc_dt_raw : MPC_DT_DEFAULT;

    int mpc_call_interval = (int)(MPC_DT / SIM_DT + 0.5);
    if (mpc_call_interval < 1) mpc_call_interval = 1;
    const int MPC_CALL_INTERVAL = mpc_call_interval;
    const char *sim_duration_env = getenv("SIM_DURATION");
    const double sim_duration_raw = sim_duration_env ? atof(sim_duration_env) : SIM_DURATION_DEFAULT;
    const double SIM_DURATION = (sim_duration_raw > 1e-3) ? sim_duration_raw : SIM_DURATION_DEFAULT;
    const int SIM_STEPS = (int)(SIM_DURATION / SIM_DT);

    const char *pred_dt_env = getenv("PRED_DT");
    const double pred_dt_raw = pred_dt_env ? atof(pred_dt_env) : TIME_STEP_SECONDS;
    g_mpc_prediction_dt = (pred_dt_raw > 1e-6) ? pred_dt_raw : TIME_STEP_SECONDS;
    const double cross_scale = MPC_DT / g_mpc_prediction_dt;

    /* Body safety margin: extra buffer beyond VEHICLE_HALF_WIDTH for wall checks.
     * Default 0.06m matches gym bitmap strictness.  Set to 0 for pure
     * body-edge collision (more accurate for real hardware). */
    const char *bsm_env = getenv("BODY_SAFETY_MARGIN");
    const double body_safety_margin = bsm_env ? atof(bsm_env) : DEFAULT_BODY_SAFETY_MARGIN;
    const double start_offset_lat = get_env_double("START_OFFSET_LAT", 0.0);
    const double start_offset_x = get_env_double("START_OFFSET_X", 0.0);
    const double start_offset_y = get_env_double("START_OFFSET_Y", 0.0);
    const double start_heading_offset = get_env_double("START_HEADING_OFFSET", 0.0);
    const double start_speed = get_env_double("START_SPEED", 0.0);

    const int verbose = getenv("VERBOSE") != NULL;

    printf("=== MPC Sim-Drive Test (Riccati-ADMM, %.0fs at dt=%.4fs = %d steps, %.0fHz) ===\n",
           SIM_DURATION, SIM_DT, SIM_STEPS, 1.0/SIM_DT);
    printf("    MPC rate: %.0fHz (every %d sim steps)\n",
           1.0/MPC_DT, MPC_CALL_INTERVAL);

    /* Realistic simulation mode: enable real-world effects.
     * REALISTIC_SIM=1  → all features
     * Individual toggles: REALISTIC_TIRES=1, REALISTIC_DRIVE=1,
     *                     REALISTIC_DELAY=1, REALISTIC_NOISE=1  */
    const char *realistic_env = getenv("REALISTIC_SIM");
    const int realistic_all = realistic_env ? atoi(realistic_env) : 0;
    const int realistic_tires = realistic_all || (getenv("REALISTIC_TIRES") && atoi(getenv("REALISTIC_TIRES")));
    const int realistic_drive = realistic_all || (getenv("REALISTIC_DRIVE") && atoi(getenv("REALISTIC_DRIVE")));
    const int realistic_delay = realistic_all || (getenv("REALISTIC_DELAY") && atoi(getenv("REALISTIC_DELAY")));
    const int realistic_noise = realistic_all || (getenv("REALISTIC_NOISE") && atoi(getenv("REALISTIC_NOISE")));
    const int realistic_mode = realistic_tires || realistic_drive || realistic_delay || realistic_noise;
    if (realistic_mode) {
        const char *seed_env = getenv("SIM_SEED");
        unsigned int sim_seed = seed_env ? (unsigned int)strtoul(seed_env, NULL, 10) : 42u;
        srand(sim_seed);  /* Deterministic by default; override with SIM_SEED for multi-trial robustness. */
        printf("    REALISTIC MODE:");
        if (realistic_drive) printf(" [drag: F_roll=%.1fN]", ROLLING_RESISTANCE_N);
        if (realistic_tires) printf(" [Pacejka tires: C=%.1f]", PACEJKA_C_SHAPE);
        if (realistic_delay) printf(" [1-step delay]");
        if (realistic_noise) printf(" [sensor noise]");
        printf(" [seed=%u]", sim_seed);
        printf("\n");
    }
    if (body_safety_margin != DEFAULT_BODY_SAFETY_MARGIN)
        printf("    BODY_SAFETY_MARGIN: %.3fm (default: %.3fm)\n",
               body_safety_margin, DEFAULT_BODY_SAFETY_MARGIN);
    if (fabs(start_offset_lat) > 1e-9 || fabs(start_offset_x) > 1e-9 || fabs(start_offset_y) > 1e-9 ||
        fabs(start_heading_offset) > 1e-9 || fabs(start_speed) > 1e-9) {
        printf("    START override: lat=%+.3fm dx=%+.3fm dy=%+.3fm hdg=%+.3frad v0=%.2fm/s\n",
               start_offset_lat, start_offset_x, start_offset_y, start_heading_offset, start_speed);
    }
    printf("\n");

    if (!load_raceline()) return 1;

    int start_index = 0;
    const char *start_index_env = getenv("START_INDEX");
    if (start_index_env) start_index = atoi(start_index_env);
    if (start_index < 0) start_index = 0;
    if (start_index >= raceline_count) start_index = raceline_count - 1;

    /* Initialize Riccati-ADMM MPC via the unified API */
    mpc_initialize();
    mpc_reset();

    /* Configure horizon and weights. Horizon must respect compile-time limits. */
    MpcConfiguration_t cfg = mpc_get_configuration();
    int horizon = PREDICTION_HORIZON;
    if (getenv("HORIZON")) horizon = atoi(getenv("HORIZON"));
    if (horizon < 1) horizon = 1;
    if (horizon > PREDICTION_HORIZON) horizon = PREDICTION_HORIZON;
    cfg.prediction_horizon_steps = (uint16_t)horizon;
    /* Prediction time step: propagate PRED_DT to solver's dynamics model */
    cfg.time_step = (float)(g_mpc_prediction_dt);
    /* cross_call_rate_scale: ratio of control interval to prediction dt */
    cfg.cross_call_rate_scale = (float)(cross_scale);
    /* Tuned weights — overridable via environment variables for tuning script. */
    const char *env;
    cfg.weight_lateral_error          = (float)((env = getenv("Q_LAT"))       ? atof(env) : 340.0);
    cfg.weight_heading_error          = (float)((env = getenv("Q_HDG"))       ? atof(env) : 1000.0);
    cfg.weight_velocity               = (float)((env = getenv("Q_VEL"))       ? atof(env) : 26.0);
    cfg.weight_lateral_velocity       = (float)((env = getenv("Q_LAT_VEL"))   ? atof(env) : 69.0);
    cfg.weight_yaw_rate               = (float)((env = getenv("Q_YAW"))       ? atof(env) : 22.0);
    cfg.weight_steering_effort        = (float)((env = getenv("R_STEER"))     ? atof(env) : 0.15);
    cfg.weight_acceleration_effort    = (float)((env = getenv("R_ACCEL"))     ? atof(env) : 0.01);
    cfg.weight_steering_rate          = (float)((env = getenv("W_JERK"))      ? atof(env) : 0.3);
    cfg.weight_acceleration_rate      = (float)((env = getenv("W_ACCEL_RATE"))? atof(env) : 0.1);
    cfg.max_solver_iterations       = (env = getenv("MAX_ITER")) ? atoi(env) : 20;
    cfg.solver_convergence_tolerance    = (float)((env = getenv("TOL")) ? atof(env) : 5.0);
    mpc_set_configuration(&cfg);

    if (verbose) {
        printf("  Horizon: %d, Q_lat=%.2f Q_hdg=%.2f Q_vel=%.2f R_steer=%.2f R_accel=%.2f\n",
               cfg.prediction_horizon_steps,
               (double)(cfg.weight_lateral_error),
               (double)(cfg.weight_heading_error),
               (double)(cfg.weight_velocity),
               (double)(cfg.weight_steering_effort),
               (double)(cfg.weight_acceleration_effort));
        printf("  Steer_rate=%.2f Accel_rate=%.2f Cross_call=%.2f\n",
               (double)(cfg.weight_steering_rate),
               (double)(cfg.weight_acceleration_rate),
               (double)(cfg.cross_call_rate_scale));
    }

    /* Spawn at raceline[0], optionally shifted laterally and with a custom initial speed. */
    const double start_normal = raceline[0].psi + M_PI / 2.0;
    VehicleState_t state;
    state.pos_x = (float)(raceline[0].x + start_offset_lat * cos(start_normal) + start_offset_x);
    state.pos_y = (float)(raceline[0].y + start_offset_lat * sin(start_normal) + start_offset_y);
    state.heading = (float)(wrap_angle(raceline[0].psi + start_heading_offset));
    state.long_vel = (float)(start_speed);
    state.lat_vel = 0;
    state.yaw_rate = 0;
    last_closest = start_index;

    if (verbose || fabs(start_offset_lat) > 1e-6 || fabs(start_heading_offset) > 1e-6 || start_speed > 1e-6) {
        printf("    Start state: idx=%d, lat_offset=%+.3fm, hdg_offset=%+.3frad, speed=%.2fm/s\n",
               start_index, start_offset_lat, start_heading_offset, start_speed);
    }

    /* Tracking metrics */
    double max_lat_err = 0, sum_lat_err = 0;
    double max_hdg_err = 0, sum_hdg_err = 0;
    double max_vel_err = 0, sum_vel_err = 0;
    int wall_collisions = 0;
    int solver_ok = 0, solver_calls = 0;
    double prev_steer = 0;
    double actual_steer = 0;  /* Physical servo position (rate-limited) */
    /* Steer buffer: matching gym's steer_buffer_size=2 delay FIFO */
    double steer_buffer[STEER_BUFFER_SIZE];
    int steer_buf_idx = 0;
    for (int i = 0; i < STEER_BUFFER_SIZE; i++) steer_buffer[i] = 0.0;
    int steer_reversals = 0;
    double max_steer_change = 0;
    double time_above_5ms = 0;
    double max_vx = 0;
    double sum_vx = 0;
    double progress_m = 0.0;
    double avg_progress_mps = 0.0;
    double total_lap_time = 0.0;
    double last_lap_cross_time = 0.0;
    double unwrapped_s = 0.0;
    double last_projected_s = 0.0;
    double start_projected_s = 0.0;
    double next_lap_marker_s = 0.0;
    int completed_laps = 0;
    int progress_initialized = 0;

    /* True (noise-free) state for wall checks and metrics.
     * Sensor noise should only affect MPC input, not ground-truth collision. */
    VehicleState_t true_state = state;  /* Starts same as initial state */
    long total_iterations = 0;
    int max_iter_single = 0;
    double total_solve_us = 0.0, max_solve_us = 0.0;
    clock_t ts0_clk, ts1_clk;

    /* Held MPC commands (zero-order hold between 20Hz MPC calls) */
    double cmd_steer = 0.0;
    double cmd_accel = 0.0;
    int steps_executed = 0;

    /* MPC computation delay buffer (realistic mode only):
     * Control computed from state at time t is applied at time t+dt. */
    double delayed_steer = 0.0;
    double delayed_accel = 0.0;

    if (verbose) {
        printf("\n  Step | Time  | vx    | v_cmd | e_y   | e_psi | cmd_st | act_st | accel | iter | wp  | wall?\n");
        printf("  -----|-------|-------|-------|-------|-------|--------|--------|-------|------|-----|------\n");
    }

    for (int step = 0; step < SIM_STEPS; step++) {
        steps_executed = step + 1;
        double t = step * SIM_DT;
        double px = (double)(state.pos_x);
        double py = (double)(state.pos_y);
        double psi = (double)(state.heading);
        double vx = (double)(state.long_vel);

        int closest = find_closest_waypoint(px, py, psi);

        FrenetState_t frenet = vehicle_to_frenet(&state, closest);

        /* EMA (exponential moving average) filter on Frenet state for MPC.
         * Smooths sensor noise in realistic mode. Wall check uses unfiltered. */
        static FrenetState_t frenet_filt;
        static int ema_initialized = 0;
        FrenetState_t frenet_for_mpc;
        if (realistic_noise && !ema_initialized) {
            frenet_filt = frenet;
            ema_initialized = 1;
        }
        if (realistic_noise) {
            const double ema_alpha = 0.35;  /* 0.35 = 65% filtered */
            frenet_filt.flat_error = (float)(
                ema_alpha * (double)(frenet.flat_error)
                + (1.0 - ema_alpha) * (double)(frenet_filt.flat_error));
            frenet_filt.fhead_error = (float)(
                ema_alpha * (double)(frenet.fhead_error)
                + (1.0 - ema_alpha) * (double)(frenet_filt.fhead_error));
            frenet_filt.flong_vel = (float)(
                ema_alpha * (double)(frenet.flong_vel)
                + (1.0 - ema_alpha) * (double)(frenet_filt.flong_vel));
            frenet_filt.flat_vel = (float)(
                ema_alpha * (double)(frenet.flat_vel)
                + (1.0 - ema_alpha) * (double)(frenet_filt.flat_vel));
            frenet_filt.fyaw_rate = (float)(
                ema_alpha * (double)(frenet.fyaw_rate)
                + (1.0 - ema_alpha) * (double)(frenet_filt.fyaw_rate));
            frenet_for_mpc = frenet_filt;
        } else {
            frenet_for_mpc = frenet;
        }

        /* Wall check and metrics use TRUE state (no sensor noise) */
        double true_px = (double)(true_state.pos_x);
        double true_py = (double)(true_state.pos_y);
        double true_psi = (double)(true_state.heading);
        int true_closest = realistic_noise ? find_closest_waypoint(true_px, true_py, true_psi) : closest;
        FrenetState_t true_frenet = realistic_noise ? vehicle_to_frenet(&true_state, true_closest) : frenet;
        /* Ground-truth speed magnitude for robust metrics (frame/sign independent). */
        const double speed_mps = hypot((double)(true_state.long_vel), (double)(true_state.lat_vel));
        if (speed_mps > max_vx) max_vx = speed_mps;
        sum_vx += speed_mps;
        double e_y = (double)(true_frenet.flat_error);
        double e_psi = (double)(true_frenet.fhead_error);
        const double current_progress_s = raceline[true_closest].s;

        if (!progress_initialized) {
            start_projected_s = current_progress_s;
            last_projected_s = current_progress_s;
            unwrapped_s = current_progress_s;
            next_lap_marker_s = start_projected_s + g_track_length_m;
            last_lap_cross_time = 0.0;
            progress_initialized = 1;
        } else {
            double ds = current_progress_s - last_projected_s;
            if (g_track_length_m > 1e-6) {
                if (ds < -0.5 * g_track_length_m) ds += g_track_length_m;
                else if (ds > 0.5 * g_track_length_m) ds -= g_track_length_m;
            }
            unwrapped_s += ds;
            last_projected_s = current_progress_s;
        }
        progress_m = unwrapped_s - start_projected_s;
        while (g_track_length_m > 1e-6 && unwrapped_s >= next_lap_marker_s) {
            completed_laps++;
            total_lap_time += (t - last_lap_cross_time);
            last_lap_cross_time = t;
            next_lap_marker_s += g_track_length_m;
        }

        /* Wall collision check — body-edge, using TRUE state */
        double left_wall = raceline[true_closest].left_bound;
        double right_wall = raceline[true_closest].right_bound;
        int wall_hit = 0;
        if (e_y > (left_wall - VEHICLE_HALF_WIDTH - body_safety_margin))  { wall_hit = 1;  wall_collisions++; }
        if (e_y < -(right_wall - VEHICLE_HALF_WIDTH - body_safety_margin)){ wall_hit = -1; wall_collisions++; }
        if (wall_hit) {
            printf("\n  !!! WALL CRASH: e_y = %.3f m (bound: %.3f) at step %d (t=%.2fs, wp=%d, v=%.1f) !!!\n",
                   e_y, wall_hit > 0 ? left_wall : right_wall, step, t, closest, speed_mps);
            break;
        }

        /* Recompute MPC at the configured control interval. */
        double steer = cmd_steer;
        double accel_cmd = cmd_accel;
        int iter = 0;

        /* Feed the realized ST servo position to MPC (from previous propagation).
         * actual_steer is st_delta from end of previous step's propagation. */
        {
            ControlInput_t actual_ctrl;
            /* Use previously realized steering (st_delta), not desired */
            actual_ctrl.steer_ang = (float)(actual_steer);
            actual_ctrl.long_acc = (float)(cmd_accel);
            mpc_set_actual_previous_control(&actual_ctrl);
        }

        if (step % MPC_CALL_INTERVAL == 0) {
            /* Build reference */
            TrajectoryReferencePoint_t ref[PREDICTION_HORIZON];
            build_reference(closest, cfg.prediction_horizon_steps, ref);

            /* Always call MPC — no low-speed guard */
            MpcSolverResult_t result;
            ts0_clk = clock();
            MpcSolverStatus_t status = mpc_compute_optimal_control(&frenet_for_mpc, ref, &result);
            ts1_clk = clock();
            double solve_us = 0.0;
            if (ts0_clk != (clock_t)-1 && ts1_clk != (clock_t)-1) {
                solve_us = (double)(ts1_clk - ts0_clk) * 1e6 / CLOCKS_PER_SEC;
            }
            total_solve_us += solve_us;
            if (solve_us > max_solve_us) max_solve_us = solve_us;
            steer = (double)(result.optimal_control.steer_ang);
            accel_cmd = (double)(result.optimal_control.long_acc);
            iter = result.iterations_used;
            total_iterations += iter;
            if (iter > max_iter_single) max_iter_single = iter;
            if (status == MPC_STATUS_SUCCESS || status == MPC_STATUS_MAXIMUM_ITERATIONS_REACHED)
                solver_ok++;
            solver_calls++;
            cmd_steer = steer;
            cmd_accel = accel_cmd;
        }

        /* Realistic mode: 1-step MPC computation delay.
         * Control computed this step is applied next step. */
        if (realistic_delay) {
            double apply_steer = delayed_steer;
            double apply_accel = delayed_accel;
            delayed_steer = steer;
            delayed_accel = accel_cmd;
            steer = apply_steer;
            accel_cmd = apply_accel;
        }

        /* Physical saturation: steering and acceleration */
        if (steer > MAX_STEERING) steer = MAX_STEERING;
        if (steer < -MAX_STEERING) steer = -MAX_STEERING;
        if (accel_cmd > PHYSICAL_MAX_ACCEL) accel_cmd = PHYSICAL_MAX_ACCEL;
        if (accel_cmd < -PHYSICAL_MAX_ACCEL) accel_cmd = -PHYSICAL_MAX_ACCEL;

        /* Steer buffer: FIFO delay matching gym's steer_buffer_size.
         * Push new command into buffer, pop delayed command as actual_steer. */
        actual_steer = steer_buffer[steer_buf_idx];
        steer_buffer[steer_buf_idx] = steer;
        steer_buf_idx = (steer_buf_idx + 1) % STEER_BUFFER_SIZE;

        /* Metrics */
        if (fabs(e_y) > max_lat_err) max_lat_err = fabs(e_y);
        sum_lat_err += fabs(e_y);
        if (fabs(e_psi) > max_hdg_err) max_hdg_err = fabs(e_psi);
        sum_hdg_err += fabs(e_psi);
        double vel_err = fabs(speed_mps - raceline[closest].vx);
        if (vel_err > max_vel_err) max_vel_err = vel_err;
        sum_vel_err += vel_err;
        if (speed_mps > 5.0) time_above_5ms += SIM_DT;

        double steer_change = actual_steer - prev_steer;
        if (fabs(steer_change) > fabs(max_steer_change)) max_steer_change = steer_change;
        if (step > 0 && actual_steer * prev_steer < 0 && fabs(steer_change) > 0.1)
            steer_reversals++;
        prev_steer = actual_steer;

        /* Print every step for first 2s, then every 20 steps or on issues */
        if (verbose) {
            int print_row = (step < 40) || (step % 20 == 0) || wall_hit || (fabs(e_y) > 0.8);
            if (print_row) {
                printf("  %4d | %5.2f | %5.2f | %5.2f | %+.3f | %+.3f | %+.4f | %+.4f | %+.2f | %4d | %3d | %s\n",
                       step, t, speed_mps, raceline[closest].vx, e_y, e_psi, steer, actual_steer, accel_cmd, iter, closest,
                       wall_hit > 0 ? "LEFT!" : (wall_hit < 0 ? "RIGHT!" : ""));
            }
        }

        /* Propagate vehicle state using gym-matching ST model with RK4.
         * State: [X, Y, delta, V, psi, psi_dot, beta] (7 states)
         * Matches f1tenth_gym single_track.py exactly:
         *   - Kinematic mode (V < 0.5 m/s)
         *   - Full nonlinear dynamic mode (V >= 0.5 m/s) with:
         *     * atan2-based slip angles (not small-angle approx)
         *     * cos(δ)/sin(δ) front force resolution
         *     * Body-frame dynamics for V_DOT and BETA_DOT
         * Vehicle params from measured data (sim.yaml / vehicle_params.yaml). */
        {
            /* Vehicle parameters matching gym config */
            const double mu = 0.745, mass = 3.314, Iz = 0.035;
            const double C_Sf = 4.297, C_Sr = 3.473;
            const double lf = 0.166, lr = 0.16, h_cg = 0.0703;
            const double g_acc = 9.81;
            const double sv_max = 2.8492;  /* max steering velocity */
            const double s_max = 0.4189;   /* max steering angle (calibrated) */
            const double v_switch = 7.319;
            const double v_min = 0.0, v_max = 20.0;
            const double lwb = 0.326;  /* lf + lr */

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

            /* pid_steer: bang-bang controller matching gym's pid_steer() exactly.
             * gym: sv = sign(steer_diff) * max_sv  if |diff| > 1e-4, else 0.
             * NO overshoot clipping — gym relies on angle limits only. */
            double steer_vel;
            {
                double diff = actual_steer - st_delta;
                if (fabs(diff) > 1e-4)
                    steer_vel = (diff > 0 ? 1.0 : -1.0) * sv_max;
                else
                    steer_vel = 0.0;
                /* steering_constraint: enforce angle limits (matching gym) */
                if (st_delta >= s_max && steer_vel > 0) steer_vel = 0.0;
                if (st_delta <= -s_max && steer_vel < 0) steer_vel = 0.0;
                if (steer_vel < -sv_max) steer_vel = -sv_max;
                if (steer_vel >  sv_max) steer_vel =  sv_max;
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

            /* ST dynamics RHS function matching gym's single_track.py exactly.
             * Kinematic mode (V < 0.5): same as CommonRoad.
             * Dynamic mode (V >= 0.5): full nonlinear model with:
             *   - atan2-based slip angles
             *   - cos(δ)/sin(δ) front force resolution
             *   - Full body-frame dynamics for V_DOT, BETA_DOT */
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
                    /* Dynamic ST mode — full nonlinear (matching gym single_track.py) */ \
                    double vx_ = (V) * cos(BETA); \
                    double vy_ = (V) * sin(BETA); \
                    double vx_safe_ = (vx_ > 0.5) ? vx_ : 0.5; \
                    /* Normal forces with longitudinal load transfer */ \
                    double Fzf_ = mass * (g_acc * lr - (ACCL) * h_cg) / lwb; \
                    double Fzr_ = mass * (g_acc * lf + (ACCL) * h_cg) / lwb; \
                    /* atan2-based slip angles (not small-angle approx) */ \
                    double alpha_f_ = (DELTA) - atan2(vy_ + lf * (PSI_DOT_VAL), vx_safe_); \
                    double alpha_r_ = -atan2(vy_ - lr * (PSI_DOT_VAL), vx_safe_); \
                    /* Lateral tire forces */ \
                    double Fyf_, Fyr_; \
                    if (realistic_tires) { \
                        /* Pacejka magic formula: Fy = D*sin(C*atan(B*alpha)) */ \
                        double B_f = C_Sf / PACEJKA_C_SHAPE; \
                        double B_r = C_Sr / PACEJKA_C_SHAPE; \
                        double D_f = mu * Fzf_; \
                        double D_r = mu * Fzr_; \
                        Fyf_ = D_f * sin(PACEJKA_C_SHAPE * atan(B_f * alpha_f_)); \
                        Fyr_ = D_r * sin(PACEJKA_C_SHAPE * atan(B_r * alpha_r_)); \
                    } else { \
                        /* Linear tire model: Fy = mu * C_S * alpha * Fz */ \
                        Fyf_ = mu * C_Sf * alpha_f_ * Fzf_; \
                        Fyr_ = mu * C_Sr * alpha_r_ * Fzr_; \
                    } \
                    /* Longitudinal force */ \
                    double Fx_raw_ = mass * (ACCL); \
                    double Fx_; \
                    if (realistic_drive) { \
                        /* Rolling resistance as constant drag force. \
                         * Note: VESC ACCEL_TO_CURRENT compensates for drivetrain \
                         * efficiency, so we do NOT apply eta (would double-count). */ \
                        Fx_ = Fx_raw_ - ROLLING_RESISTANCE_N; \
                    } else { \
                        Fx_ = Fx_raw_; \
                    } \
                    double cos_delta_ = cos(DELTA); \
                    double sin_delta_ = sin(DELTA); \
                    /* Body-frame dynamics with cos(δ)/sin(δ) force resolution */ \
                    double dvx_dt_ = (Fx_ - Fyf_ * sin_delta_ \
                                      + mass * vy_ * (PSI_DOT_VAL)) / mass; \
                    double dvy_dt_ = (Fyf_ * cos_delta_ + Fyr_ \
                                      - mass * vx_ * (PSI_DOT_VAL)) / mass; \
                    /* Convert body-frame accelerations to (V, β) derivatives */ \
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

            /* RK4 integration (matching gym's integrator) */
            double k1[7], k2[7], k3[7], k4[7];
            /* Use TRUE state for dynamics (not noisy measurement) */
            double true_px_dyn = (double)(true_state.pos_x);
            double true_py_dyn = (double)(true_state.pos_y);
            double true_psi_dyn = (double)(true_state.heading);
            double s0[7] = {true_px_dyn, true_py_dyn, st_delta, st_V, true_psi_dyn, st_psi_dot, st_beta};

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
            if (realistic_noise) {
                /* True state: noise-free for wall checks and metrics */
                true_state.pos_x = (float)(sn[0]);
                true_state.pos_y = (float)(sn[1]);
                true_state.heading = (float)(sn[4]);
                true_state.long_vel = (float)(sn[3] * cos(sn[6]));
                true_state.lat_vel = (float)(sn[3] * sin(sn[6]));
                true_state.yaw_rate = (float)(sn[5]);
                /* Noisy state: what the MPC controller sees */
                state.pos_x = (float)(sn[0] + NOISE_POS_M * randn());
                state.pos_y = (float)(sn[1] + NOISE_POS_M * randn());
                state.heading = (float)(sn[4] + NOISE_HDG_RAD * randn());
                state.long_vel = (float)(
                    sn[3] * cos(sn[6]) + NOISE_VX_MS * randn());
                state.lat_vel = (float)(
                    sn[3] * sin(sn[6]) + NOISE_VY_MS * randn());
                state.yaw_rate = (float)(
                    sn[5] + NOISE_OMEGA_RAD * randn());
            } else {
                state.pos_x = (float)(sn[0]);
                state.pos_y = (float)(sn[1]);
                state.heading = (float)(sn[4]);
                state.long_vel = (float)(sn[3] * cos(sn[6]));
                state.lat_vel = (float)(sn[3] * sin(sn[6]));
                state.yaw_rate = (float)(sn[5]);
                true_state = state;
            }

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
    const int metric_steps = (steps_executed > 0) ? steps_executed : 1;
    const double simulated_time = metric_steps * SIM_DT;
    double avg_lat = sum_lat_err / metric_steps;
    double avg_hdg = sum_hdg_err / metric_steps;
    double avg_vel = sum_vel_err / metric_steps;
    double avg_vx = sum_vx / metric_steps;
    avg_progress_mps = progress_m / simulated_time;
    double avg_lap_time = (completed_laps > 0) ? (total_lap_time / completed_laps) : 0.0;
    printf("\n  === Results (%.1f seconds, Riccati-ADMM, %.0fHz MPC) ===\n",
           simulated_time, 1.0 / MPC_DT);
    printf("  Solver success:     %d / %d (%.1f%%)\n", solver_ok, solver_calls,
           100.0*solver_ok/(solver_calls > 0 ? solver_calls : 1));
    printf("  Max velocity:       %.2f m/s\n", max_vx);
    printf("  Avg velocity:       %.2f m/s\n", avg_vx);
    printf("  Max lateral error:  %.3f m\n", max_lat_err);
    printf("  Avg lateral error:  %.3f m\n", avg_lat);
    printf("  Max heading error:  %.4f rad (%.1f deg)\n", max_hdg_err, max_hdg_err*180/M_PI);
    printf("  Avg heading error:  %.4f rad (%.1f deg)\n", avg_hdg, avg_hdg*180/M_PI);
    printf("  Max velocity error: %.2f m/s\n", max_vel_err);
    printf("  Avg velocity error: %.2f m/s\n", avg_vel);
    printf("  Track progress:     %.2f m (%.2f m/s)\n", progress_m, avg_progress_mps);
    printf("  Completed laps:     %d\n", completed_laps);
    if (completed_laps > 0)
        printf("  Avg lap time:       %.3f s\n", avg_lap_time);
    printf("  Max steer change:   %.4f rad/step\n", max_steer_change);
    printf("  Steer reversals:    %d\n", steer_reversals);
    printf("  Track progress:     %.2f m (%.2f m/s avg)\n", progress_m, avg_progress_mps);
    printf("  Completed laps:     %d\n", completed_laps);
    if (completed_laps > 0) {
        printf("  Avg lap time:       %.2f s\n", avg_lap_time);
    }
    printf("  Wall collisions:    %d\n", wall_collisions);
    printf("  Time above 5 m/s:   %.1f / %.1f s (%.0f%%)\n",
           time_above_5ms, simulated_time,
           (simulated_time > 0.0) ? (100*time_above_5ms/simulated_time) : 0.0);
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

    /* Pass/fail criteria — relaxed speed threshold in realistic mode since
     * rolling resistance, Pacejka saturation, delay, and noise all reduce
     * achievable speed compared to the ideal model. */
    double speed_threshold = realistic_mode ? 0.30 : 0.50;
    check("No wall collisions", wall_collisions == 0);
    check("Max lateral error < 1.2 m", max_lat_err < 1.2);
    check("Avg lateral error < 0.5 m", avg_lat < 0.5);
    check("Avg heading error < 0.3 rad (17 deg)", avg_hdg < 0.3);
    check("Solver mostly succeeds (>80%)", solver_ok > solver_calls * 80 / 100);
    if (realistic_mode) {
        char speed_msg[128];
        snprintf(speed_msg, sizeof(speed_msg),
                 "Reaches driving speed (>5 m/s for >%.0f%% of time, realistic)",
                 speed_threshold * 100);
        check(speed_msg, time_above_5ms > simulated_time * speed_threshold);
    } else {
        check("Reaches driving speed (>5 m/s for >50% of time)",
              time_above_5ms > simulated_time * 0.5);
    }

    printf("\n=== RESULTS: %d passed, %d failed ===\n", tests_passed, tests_failed);

    /* Machine-readable CSV summary line for tuning scripts */
    if (getenv("MPC_TUNING_CSV")) {
         printf("CSV,%d,%d,%.4f,%.4f,%.4f,%.4f,%.2f,%.1f,%.1f,%d,%.1f,%.4f,%.4f,%.1f,%.4f,%.4f,%.4f,%d,%.4f,%.4f,%d\n",
            tests_passed, tests_failed,
            max_lat_err, avg_lat, max_hdg_err, avg_hdg,
            max_vx, avg_solve, max_solve_us,
            wall_collisions, time_above_5ms,
            max_vel_err, avg_vel, avg_iters, avg_vx,
            progress_m, avg_progress_mps, completed_laps,
            avg_lap_time, fabs(max_steer_change), steer_reversals);
    }
    return tests_failed > 0 ? 1 : 0;
}
