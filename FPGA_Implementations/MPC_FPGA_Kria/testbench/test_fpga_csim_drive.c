/**
 * @file test_fpga_csim_drive.c
 * @brief FPGA C-sim: realistic MPC closed-loop on hardware map
 * @details Runs closed-loop simulation of the Riccati-ADMM controller with
 *          configurable physics/control rates and optional realistic effects
 *          (noise, nonlinear tire saturation, drivetrain drag).
 *          This is a full port of MPC/test/test_sim_drive.c with the MPC
 *          boundary redirected through the FPGA scalar wrapper.
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
 *   - Runs for 100 seconds
 *
 * Reports: wall collisions, max/avg lateral error, steering behavior,
 *          velocity tracking, and step-by-step diagnostics near crashes.
 *
 * @dependencies mpc_cpu_compat.h, <stdio.h>, <stdlib.h>, <string.h>, <math.h>, <time.h>
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

#ifndef TWO_PI
#define TWO_PI (2.0 * M_PI)
#endif

#include "mpc_cpu_compat.h"

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
#define MAX_STEERING      0.39 /* rad — calibrated limit (with polynomial servo correction) */
#define MAX_VELOCITY      20.0   /* m/s */
#define PHYSICAL_MAX_ACCEL 7.31  /* m/s² — bounded by mu*g */

/* Trajectory pre-processing (matching gym_bridge ROS2 node exactly) */
#define TRAJECTORY_MAX_VELOCITY   20.0
#define MIN_TRAJECTORY_SPEED_MPS  0.5
#define TRAJECTORY_SPEED_GAIN     1.0
#define VEHICLE_HALF_WIDTH        0.137   /* meters — for body-edge collision */
#define DEFAULT_BODY_SAFETY_MARGIN 0.06   /* extra margin: gym bitmap is stricter */

/*===========================================================================
 * Hardware-matched plant enhancements
 *
 * This simulator is intentionally locked to the hardware-like plant:
 *   1. Rolling resistance / drivetrain drag
 *   2. Pacejka tire saturation
 *   3. Sensor noise on the MPC-observed state
 * Idealized toggles are intentionally removed so tuning always targets the
 * real-car-like failure modes.
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

/* Optional local-raceline emulation (matches MPC hardware node behavior more closely). */
static Waypoint_t local_line[MAX_WAYPOINTS];
static int local_count = 0;
static int local_last_closest = 0;
typedef struct {
    unsigned long seq;
    long long ros_time_ns;
    long file_offset;
    int waypoint_count;
    double s_global;
    int has_s_global;
} LocalReplaySnapshot_t;
static FILE *g_local_replay_file = NULL;
static LocalReplaySnapshot_t *g_local_replay_snapshots = NULL;
static int g_local_replay_snapshot_count = 0;
static int g_local_replay_active_snapshot = -1;
static int g_local_replay_enabled = 0;
static long long g_local_replay_start_ns = 0;
static int g_local_replay_progress_mode = 0;

/* Read a double-valued environment override, falling back when unset/invalid. */
static double get_env_double(const char *name, double fallback)
{
    const char *env = getenv(name);
    if (env == NULL || env[0] == '\0') {
        return fallback;
    }
    char *end = NULL;
    double value = strtod(env, &end);
    if (end == env || (end != NULL && *end != '\0')) {
        return fallback;
    }
    return value;
}

static void free_local_raceline_replay(void)
{
    if (g_local_replay_file) {
        fclose(g_local_replay_file);
        g_local_replay_file = NULL;
    }
    free(g_local_replay_snapshots);
    g_local_replay_snapshots = NULL;
    g_local_replay_snapshot_count = 0;
    g_local_replay_active_snapshot = -1;
    g_local_replay_enabled = 0;
    g_local_replay_start_ns = 0;
    g_local_replay_progress_mode = 0;
}

static int load_local_replay_snapshot(int snapshot_index)
{
    if (!g_local_replay_file || snapshot_index < 0 || snapshot_index >= g_local_replay_snapshot_count)
        return 0;

    const LocalReplaySnapshot_t *snap = &g_local_replay_snapshots[snapshot_index];
    if (fseek(g_local_replay_file, snap->file_offset, SEEK_SET) != 0)
        return 0;

    char line[512];
    int count = 0;
    while (count < snap->waypoint_count && fgets(line, sizeof(line), g_local_replay_file)) {
        unsigned long seq = 0;
        long long ros_ns = 0;
        int waypoint_index = 0;
        int waypoint_count = 0;
        Waypoint_t wp = {0};
        if (sscanf(line,
                   "%lu,%lld,%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                   &seq, &ros_ns, &waypoint_index, &waypoint_count,
                   &wp.s, &wp.x, &wp.y, &wp.psi, &wp.kappa, &wp.vx,
                   &wp.left_bound, &wp.right_bound) != 12) {
            break;
        }
        wp.ax = 0.0;
        if (waypoint_index >= 0 && waypoint_index < MAX_WAYPOINTS) {
            local_line[waypoint_index] = wp;
        }
        count++;
    }
    local_count = snap->waypoint_count;
    local_last_closest = 0;
    g_local_replay_active_snapshot = snapshot_index;
    return (count == snap->waypoint_count);
}

static int init_local_raceline_replay(const char *filepath, long long start_ros_time_ns)
{
    if (!filepath || !filepath[0]) return 0;

    FILE *file = fopen(filepath, "r");
    if (!file) return 0;

    free_local_raceline_replay();
    g_local_replay_file = file;
    g_local_replay_start_ns = start_ros_time_ns;

    char line[512];
    if (!fgets(line, sizeof(line), g_local_replay_file)) {
        free_local_raceline_replay();
        return 0;
    }

    int capacity = 256;
    g_local_replay_snapshots = (LocalReplaySnapshot_t *)malloc((size_t)capacity * sizeof(LocalReplaySnapshot_t));
    if (!g_local_replay_snapshots) {
        free_local_raceline_replay();
        return 0;
    }

    unsigned long current_seq = 0;
    int have_seq = 0;
    while (1) {
        long offset = ftell(g_local_replay_file);
        if (!fgets(line, sizeof(line), g_local_replay_file)) break;

        unsigned long seq = 0;
        long long ros_ns = 0;
        int waypoint_index = 0;
        int waypoint_count = 0;
        if (sscanf(line, "%lu,%lld,%d,%d,", &seq, &ros_ns, &waypoint_index, &waypoint_count) != 4)
            continue;

        if (!have_seq || seq != current_seq) {
            if (g_local_replay_snapshot_count >= capacity) {
                capacity *= 2;
                LocalReplaySnapshot_t *grown = (LocalReplaySnapshot_t *)realloc(
                    g_local_replay_snapshots, (size_t)capacity * sizeof(LocalReplaySnapshot_t));
                if (!grown) {
                    free_local_raceline_replay();
                    return 0;
                }
                g_local_replay_snapshots = grown;
            }
            LocalReplaySnapshot_t *snap = &g_local_replay_snapshots[g_local_replay_snapshot_count++];
            snap->seq = seq;
            snap->ros_time_ns = ros_ns;
            snap->file_offset = offset;
            snap->waypoint_count = waypoint_count;
            snap->s_global = 0.0;
            snap->has_s_global = 0;
            current_seq = seq;
            have_seq = 1;
        }
    }

    if (g_local_replay_snapshot_count <= 0) {
        free_local_raceline_replay();
        return 0;
    }

    g_local_replay_enabled = 1;
    if (g_local_replay_start_ns == 0)
        g_local_replay_start_ns = g_local_replay_snapshots[0].ros_time_ns;

    int initial_snapshot = 0;
    while (initial_snapshot + 1 < g_local_replay_snapshot_count &&
           g_local_replay_snapshots[initial_snapshot + 1].ros_time_ns <= g_local_replay_start_ns) {
        initial_snapshot++;
    }
    return load_local_replay_snapshot(initial_snapshot);
}

static int load_local_raceline_replay_index(const char *filepath)
{
    if (!filepath || !filepath[0] || !g_local_replay_snapshots || g_local_replay_snapshot_count <= 0)
        return 0;

    FILE *file = fopen(filepath, "r");
    if (!file) return 0;

    char line[256];
    if (!fgets(line, sizeof(line), file)) {
        fclose(file);
        return 0;
    }

    int loaded = 0;
    while (fgets(line, sizeof(line), file)) {
        unsigned long seq = 0;
        double s_global = 0.0;
        long long pose_ns = 0;
        if (sscanf(line, "%lu,%lf,%lld", &seq, &s_global, &pose_ns) < 2)
            continue;
        for (int i = 0; i < g_local_replay_snapshot_count; i++) {
            if (g_local_replay_snapshots[i].seq == seq) {
                g_local_replay_snapshots[i].s_global = s_global;
                g_local_replay_snapshots[i].has_s_global = 1;
                loaded++;
                break;
            }
        }
    }
    fclose(file);
    return loaded > 0;
}

static void maybe_update_local_raceline_replay(double sim_time_s)
{
    if (!g_local_replay_enabled || g_local_replay_snapshot_count <= 0) return;

    long long target_ns = g_local_replay_start_ns + (long long)llround(sim_time_s * 1e9);
    int next_snapshot = g_local_replay_active_snapshot;
    if (next_snapshot < 0) next_snapshot = 0;
    while (next_snapshot + 1 < g_local_replay_snapshot_count &&
           g_local_replay_snapshots[next_snapshot + 1].ros_time_ns <= target_ns) {
        next_snapshot++;
    }
    if (next_snapshot != g_local_replay_active_snapshot) {
        load_local_replay_snapshot(next_snapshot);
    }
}

static void maybe_update_local_raceline_replay_progress(double s_global)
{
    if (!g_local_replay_enabled || g_local_replay_snapshot_count <= 0) return;

    int best_index = g_local_replay_active_snapshot;
    double best_dist = 1e18;
    for (int i = 0; i < g_local_replay_snapshot_count; i++) {
        if (!g_local_replay_snapshots[i].has_s_global) continue;
        double ds = fabs(g_local_replay_snapshots[i].s_global - s_global);
        if (g_track_length_m > 1e-6) {
            if (ds > 0.5 * g_track_length_m) ds = g_track_length_m - ds;
        }
        if (ds < best_dist) {
            best_dist = ds;
            best_index = i;
        }
    }
    if (best_index >= 0 && best_index != g_local_replay_active_snapshot) {
        load_local_replay_snapshot(best_index);
    }
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
        "trajectories/my_track_raceline.csv",
        "../trajectories/my_track_raceline.csv",
        "../../MPC/trajectories/my_track_raceline.csv",
        "../../f1tenth_planning/trajectories/my_track_raceline.csv",
        "../../../f1tenth_planning/trajectories/my_track_raceline.csv",
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

/* Recompute s/psi/kappa from x/y geometry, similar to the hardware node's
 * /local_raceline processing. This helps stress-test MPC robustness against
 * discretization/noise in the provided path rather than trusting CSV kappa. */
static void recompute_raceline_geometry(int diff_window, int recompute_s)
{
    if (raceline_count < 2) return;
    if (diff_window < 1) diff_window = 1;

    /* Recompute s as cumulative arc-length (optional). */
    if (recompute_s) {
        double cumulative_s = 0.0;
        double prev_x = raceline[0].x;
        double prev_y = raceline[0].y;
        raceline[0].s = 0.0;
        for (int i = 1; i < raceline_count; i++) {
            const double x = raceline[i].x;
            const double y = raceline[i].y;
            cumulative_s += hypot(x - prev_x, y - prev_y);
            raceline[i].s = cumulative_s;
            prev_x = x;
            prev_y = y;
        }
        g_track_length_m = raceline[raceline_count - 1].s - raceline[0].s;
        if (g_track_length_m < 1e-3) g_track_length_m = 0.0;
    }

    /* Heading from windowed finite differences. */
    for (int i = 0; i < raceline_count; i++) {
        int i_prev = (i - diff_window >= 0) ? (i - diff_window) : 0;
        int i_next = (i + diff_window < raceline_count) ? (i + diff_window) : (raceline_count - 1);
        const double dx = raceline[i_next].x - raceline[i_prev].x;
        const double dy = raceline[i_next].y - raceline[i_prev].y;
        double heading = 0.0;
        if ((dx * dx + dy * dy) > 1e-12) {
            heading = atan2(dy, dx);
        } else if (i > 0) {
            heading = raceline[i - 1].psi;
        }
        raceline[i].psi = heading;
    }

    /* Curvature from dpsi/ds using same window. */
    if (raceline_count >= 3) {
        for (int i = 1; i + 1 < raceline_count; i++) {
            int i_prev = (i - diff_window >= 0) ? (i - diff_window) : 0;
            int i_next = (i + diff_window < raceline_count) ? (i + diff_window) : (raceline_count - 1);
            double dpsi = wrap_angle(raceline[i_next].psi - raceline[i_prev].psi);
            const double ds = raceline[i_next].s - raceline[i_prev].s;
            const double ds_safe = (ds > 1e-6) ? ds : 1e-6;
            raceline[i].kappa = dpsi / ds_safe;
        }
        raceline[0].kappa = raceline[1].kappa;
        raceline[raceline_count - 1].kappa = raceline[raceline_count - 2].kappa;
    }
}

/* Build an open local segment starting ahead of the vehicle on the global raceline.
 * This emulates the /local_raceline behavior on hardware (segment may start ahead). */
static void build_local_segment(int global_closest, int start_offset_points, int segment_points,
                                int heading_window, int curvature_window)
{
    if (raceline_count < 2) { local_count = 0; return; }
    if (segment_points < 2) segment_points = 2;
    if (segment_points > MAX_WAYPOINTS) segment_points = MAX_WAYPOINTS;
    if (heading_window < 1) heading_window = 1;
    if (curvature_window < 1) curvature_window = 1;

    const int start = (global_closest + start_offset_points) % raceline_count;
    local_count = 0;
    double cumulative_s = 0.0;
    double prev_x = 0.0, prev_y = 0.0;

    for (int i = 0; i < segment_points; i++) {
        const int gi = (start + i) % raceline_count;
        Waypoint_t *dst = &local_line[i];
        const Waypoint_t *src = &raceline[gi];
        dst->x = src->x;
        dst->y = src->y;
        dst->vx = src->vx;
        dst->ax = src->ax;
        dst->left_bound = src->left_bound;
        dst->right_bound = src->right_bound;

        if (i == 0) {
            cumulative_s = 0.0;
        } else {
            cumulative_s += hypot(dst->x - prev_x, dst->y - prev_y);
        }
        dst->s = cumulative_s;
        dst->psi = 0.0;
        dst->kappa = 0.0;

        prev_x = dst->x;
        prev_y = dst->y;
        local_count++;
    }

    /* Heading (local window) */
    for (int i = 0; i < local_count; i++) {
        int i_prev = (i - heading_window >= 0) ? (i - heading_window) : 0;
        int i_next = (i + heading_window < local_count) ? (i + heading_window) : (local_count - 1);
        const double dx = local_line[i_next].x - local_line[i_prev].x;
        const double dy = local_line[i_next].y - local_line[i_prev].y;
        double heading = 0.0;
        if ((dx * dx + dy * dy) > 1e-12) heading = atan2(dy, dx);
        else if (i > 0) heading = local_line[i - 1].psi;
        local_line[i].psi = heading;
    }

    /* Curvature (larger window) */
    if (local_count >= 3) {
        for (int i = 1; i + 1 < local_count; i++) {
            int i_prev = (i - curvature_window >= 0) ? (i - curvature_window) : 0;
            int i_next = (i + curvature_window < local_count) ? (i + curvature_window) : (local_count - 1);
            double dpsi = wrap_angle(local_line[i_next].psi - local_line[i_prev].psi);
            const double ds = local_line[i_next].s - local_line[i_prev].s;
            const double ds_safe = (ds > 1e-6) ? ds : 1e-6;
            local_line[i].kappa = dpsi / ds_safe;
        }
        local_line[0].kappa = local_line[1].kappa;
        local_line[local_count - 1].kappa = local_line[local_count - 2].kappa;
    }
}

static int find_closest_waypoint_local(double px, double py, double heading)
{
    if (local_count <= 0) return 0;
    const int search_end = (local_count < 60) ? local_count : 60;
    int search_start = local_last_closest;
    if (search_start < 0) search_start = 0;
    if (search_start >= search_end) search_start = search_end - 1;

    const int search_window = 50;
    const int back_window = 5;
    int best = search_start;
    double best_score = 1e18;
    const double dir_x = cos(heading), dir_y = sin(heading);

    for (int off = -back_window; off < search_window; off++) {
        int i = search_start + off;
        if (i < 0) i = 0;
        if (i >= search_end) i = search_end - 1;
        const double dx = local_line[i].x - px;
        const double dy = local_line[i].y - py;
        const double d2 = dx*dx + dy*dy;
        const double dot = dx*dir_x + dy*dir_y;
        const double score = d2 + ((dot < 0.0) ? 25.0 : 0.0);
        if (score < best_score) { best_score = score; best = i; }
    }
    local_last_closest = best;
    return best;
}

static PathProjection_t project_pose_to_local_segment(double px, double py, double psi, int closest_index)
{
    PathProjection_t out = {0};
    if (local_count <= 0) return out;

    int idx0 = closest_index;
    if (idx0 < 0) idx0 = 0;
    if (idx0 >= local_count) idx0 = local_count - 1;
    int idx1 = idx0 + 1;
    if (idx1 >= local_count) idx1 = local_count - 1;

    const double ax = local_line[idx0].x;
    const double ay = local_line[idx0].y;
    const double bx = local_line[idx1].x;
    const double by = local_line[idx1].y;

    const double abx = bx - ax;
    const double aby = by - ay;
    const double apx = px - ax;
    const double apy = py - ay;
    const double ab_len2 = abx * abx + aby * aby;
    double t = 0.0;
    if (ab_len2 > 1e-12) t = (apx * abx + apy * aby) / ab_len2;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;

    const double path_x = ax + t * abx;
    const double path_y = ay + t * aby;

    /* Heading interpolation with wrap */
    double h0 = local_line[idx0].psi;
    double h1 = local_line[idx1].psi;
    double dh = h1 - h0;
    while (dh > M_PI) dh -= 2.0 * M_PI;
    while (dh < -M_PI) dh += 2.0 * M_PI;
    const double path_hdg = wrap_angle(h0 + dh * t);

    const double dx = px - path_x;
    const double dy = py - path_y;
    const double cos_h = cos(path_hdg);
    const double sin_h = sin(path_hdg);
    out.lateral_error = -dx * sin_h + dy * cos_h;

    double hdg_err = psi - path_hdg;
    while (hdg_err > M_PI) hdg_err -= 2.0 * M_PI;
    while (hdg_err < -M_PI) hdg_err += 2.0 * M_PI;
    out.heading_error = hdg_err;

    out.s = local_line[idx0].s + t * (local_line[idx1].s - local_line[idx0].s);
    return out;
}

static Waypoint_t sample_local_by_s(double s_query)
{
    if (local_count <= 0)
        return (Waypoint_t){0};

    if (local_count == 1)
    {
        Waypoint_t w = local_line[0];
        w.s = s_query;
        return w;
    }

    if (s_query <= local_line[0].s)
    {
        Waypoint_t w = local_line[0];
        w.s = s_query;
        return w;
    }
    if (s_query >= local_line[local_count - 1].s)
    {
        Waypoint_t w = local_line[local_count - 1];
        w.s = s_query;
        return w;
    }

    for (int i = 0; i < local_count - 1; i++)
    {
        const Waypoint_t *w0 = &local_line[i];
        const Waypoint_t *w1 = &local_line[i + 1];
        if (s_query >= w0->s && s_query <= w1->s)
        {
            const double denom = w1->s - w0->s;
            const double u = (denom > 1e-9) ? ((s_query - w0->s) / denom) : 0.0;

            Waypoint_t out = *w0;
            out.s = s_query;
            out.x = w0->x + (w1->x - w0->x) * u;
            out.y = w0->y + (w1->y - w0->y) * u;
            {
                double dh = w1->psi - w0->psi;
                while (dh > M_PI) dh -= 2.0 * M_PI;
                while (dh < -M_PI) dh += 2.0 * M_PI;
                out.psi = wrap_angle(w0->psi + dh * u);
            }
            out.kappa = w0->kappa + (w1->kappa - w0->kappa) * u;
            out.vx = w0->vx + (w1->vx - w0->vx) * u;
            out.ax = w0->ax + (w1->ax - w0->ax) * u;
            out.left_bound = w0->left_bound + (w1->left_bound - w0->left_bound) * u;
            out.right_bound = w0->right_bound + (w1->right_bound - w0->right_bound) * u;
            return out;
        }
    }

    Waypoint_t w = local_line[local_count - 1];
    w.s = s_query;
    return w;
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

static FrenetState_t vehicle_to_frenet_local(const VehicleState_t *v, int local_wp)
{
    FrenetState_t f;
    PathProjection_t proj = project_pose_to_local_segment(
        (double)(v->pos_x),
        (double)(v->pos_y),
        (double)(v->heading),
        local_wp);
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
            {
                double dpsi = w1->psi - w0->psi;
                while (dpsi > M_PI) dpsi -= TWO_PI;
                while (dpsi < -M_PI) dpsi += TWO_PI;
                out.psi = wrap_angle(w0->psi + dpsi * t);
            }
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
    {
        double dpsi = w1->psi - w0->psi;
        while (dpsi > M_PI) dpsi -= TWO_PI;
        while (dpsi < -M_PI) dpsi += TWO_PI;
        out.psi = wrap_angle(w0->psi + dpsi * t);
    }
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

    for (int step = 0; step < horizon_steps; step++) {
        /* Keep ref[0] aligned with the current stage; advance s for the next stage at loop end. */
        Waypoint_t wp = sample_raceline_by_s(s_query);

        ref[step].reference_lateral_error = 0;
        ref[step].reference_heading_error = 0;

        double v_ref = wp.vx;

        if (v_ref < 1.0) v_ref = 1.0;
        if (v_ref > TRAJECTORY_MAX_VELOCITY) v_ref = TRAJECTORY_MAX_VELOCITY;
        step_velocity = v_ref;
        s_query += step_velocity * g_mpc_prediction_dt;

        ref[step].reference_velocity = (float)(v_ref);

        /* vy reference: zero */
        ref[step].reference_lateral_velocity = 0;

        ref[step].reference_yaw_rate = (float)(wp.kappa * v_ref);


        ref[step].path_curvature = (float)(wp.kappa);
        ref[step].left_wall_bound = (float)(wp.left_bound);
        ref[step].right_wall_bound = (float)(wp.right_bound);
    }
}

static void build_reference_local(int horizon_steps, TrajectoryReferencePoint_t *ref)
{
    if (ref == NULL) return;
    if (horizon_steps < 1) return;
    if (horizon_steps > PREDICTION_HORIZON) horizon_steps = PREDICTION_HORIZON;
    if (local_count <= 0) return;

    double v_ref_base = local_line[0].vx;
    if (v_ref_base <= 0.0) v_ref_base = MIN_TRAJECTORY_SPEED_MPS;

    for (int step = 0; step < horizon_steps; step++) {
        double target_s = v_ref_base * g_mpc_prediction_dt * (double)step;
        Waypoint_t wp = sample_local_by_s(target_s);

        ref[step].reference_lateral_error = 0;
        ref[step].reference_heading_error = 0;
        ref[step].reference_velocity = (float)wp.vx;
        ref[step].reference_lateral_velocity = 0;
        ref[step].path_curvature = (float)wp.kappa;
        ref[step].reference_yaw_rate = (float)(wp.kappa * wp.vx);
        ref[step].left_wall_bound = (float)wp.left_bound;
        ref[step].right_wall_bound = (float)wp.right_bound;
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
    const double mpc_ctrl_dt = (mpc_dt_raw > 1e-6) ? mpc_dt_raw : MPC_DT_DEFAULT;
    const double sim_duration_raw = get_env_double("SIM_DURATION", SIM_DURATION_DEFAULT);
    const double SIM_DURATION = (sim_duration_raw > 1e-3) ? sim_duration_raw : SIM_DURATION_DEFAULT;

    int mpc_call_interval = (int)(mpc_ctrl_dt / SIM_DT + 0.5);
    if (mpc_call_interval < 1) mpc_call_interval = 1;
    const int MPC_CALL_INTERVAL = mpc_call_interval;
    const int SIM_STEPS = (int)(SIM_DURATION / SIM_DT);

    const char *pred_dt_env = getenv("PRED_DT");
    const double pred_dt_raw = pred_dt_env ? atof(pred_dt_env) : TIME_STEP_SECONDS;
    g_mpc_prediction_dt = (pred_dt_raw > 1e-6) ? pred_dt_raw : TIME_STEP_SECONDS;
    const double cross_scale = mpc_ctrl_dt / g_mpc_prediction_dt;

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
    const double start_lat_speed = get_env_double("START_LAT_SPEED", 0.0);
    const double start_yaw_rate = get_env_double("START_YAW_RATE", 0.0);
    const double start_steer = get_env_double("START_STEER", 0.0);

    const int verbose = getenv("VERBOSE") != NULL;

    /* Always emulate hardware local-raceline behavior in tuning runs. */
    const int local_raceline_sim = 1;
    const int local_start_offset_points = (getenv("LOCAL_START_OFFSET_POINTS") ? atoi(getenv("LOCAL_START_OFFSET_POINTS")) : 0);
    const int local_segment_points = (getenv("LOCAL_SEGMENT_POINTS") ? atoi(getenv("LOCAL_SEGMENT_POINTS")) : 80);
    const int local_heading_window = (getenv("LOCAL_HEADING_WINDOW") ? atoi(getenv("LOCAL_HEADING_WINDOW")) : 1);
    const int local_curvature_window = (getenv("LOCAL_CURVATURE_WINDOW") ? atoi(getenv("LOCAL_CURVATURE_WINDOW")) : 3);
    const char *local_replay_log_path = getenv("REPLAY_LOCAL_RACELINE_LOG");
    const char *local_replay_mode = getenv("REPLAY_LOCAL_RACELINE_MODE");
    const char *local_replay_index_path = getenv("REPLAY_LOCAL_RACELINE_INDEX");
    const long long local_replay_start_ns =
        (getenv("REPLAY_LOCAL_RACELINE_START_NS") && getenv("REPLAY_LOCAL_RACELINE_START_NS")[0])
        ? strtoll(getenv("REPLAY_LOCAL_RACELINE_START_NS"), NULL, 10)
        : 0LL;

    printf("=== FPGA C-Sim Drive Test (Riccati-ADMM, %.0fs at dt=%.4fs = %d steps, %.0fHz) ===\n",
            SIM_DURATION, SIM_DT, SIM_STEPS, 1.0/SIM_DT);
    printf("    MPC rate: %.0fHz (every %d sim steps)\n",
           1.0/mpc_ctrl_dt, MPC_CALL_INTERVAL);

    /* Always enable hardware-like tires, actuation/drag, and observation noise. */
    const int realistic_tires = 1;
    const int realistic_drive = 1;
    const int realistic_noise = 1;
    const int realistic_mode = 1;
    {
        const char *seed_env = getenv("SIM_SEED");
        unsigned int sim_seed = seed_env ? (unsigned int)strtoul(seed_env, NULL, 10) : 42u;
        srand(sim_seed);  /* Deterministic by default; override with SIM_SEED for multi-trial robustness. */
        printf("    HARDWARE MODE:");
        printf(" [drag: F_roll=%.1fN]", ROLLING_RESISTANCE_N);
        printf(" [Pacejka tires: C=%.1f]", PACEJKA_C_SHAPE);
        printf(" [sensor noise]");
        printf(" [seed=%u]", sim_seed);
        printf("\n");
    }

    /* Extra longitudinal realism knobs (helps match accel-to-current behavior):
     *  - DRAG_C*: coastdown drag in accel domain (m/s^2): a_drag(v)=c0+c1*|v|+c2*v^2
     *  - ACCEL_TAU_*: 1st-order accel tracking lag (seconds), separate for drive/brake
     *  - ACCEL_GAIN_*: accel tracking gain (unitless), separate for drive/brake */
    const double drag_c0 = get_env_double("DRAG_C0", 0.0);
    const double drag_c1 = get_env_double("DRAG_C1", 0.05);
    const double drag_c2 = get_env_double("DRAG_C2", 0.04);
    const double accel_tau_pos = get_env_double("ACCEL_TAU_POS", 0.05);
    const double accel_tau_neg = get_env_double("ACCEL_TAU_NEG", 0.12);
    const double accel_gain_pos = get_env_double("ACCEL_GAIN_POS", 0.5);
    const double accel_gain_neg = get_env_double("ACCEL_GAIN_NEG", 1.0);
    const double sim_mu = get_env_double("SIM_MU", 0.6652002785524997);
    const double sim_mu_front = get_env_double("SIM_MU_FRONT", 0.6745101974282083);
    const double sim_mu_rear = get_env_double("SIM_MU_REAR", 0.6565520426481404);
    const double sim_mass = get_env_double("SIM_MASS", 3.314);
    const double sim_Iz = get_env_double("SIM_IZ", 0.035);
    const double sim_C_Sf = get_env_double("SIM_C_SF", 4.78281642069513);
    const double sim_C_Sr = get_env_double("SIM_C_SR", 2.73123678240426);
    const double sim_C_Sf_high_slip = get_env_double("SIM_C_SF_HIGH_SLIP", 2.4199490875105907);
    const double sim_C_Sr_high_slip = get_env_double("SIM_C_SR_HIGH_SLIP", 2.73123678240426);
    const double sim_lf = get_env_double("SIM_LF", 0.166);
    const double sim_lr = get_env_double("SIM_LR", 0.16);
    const double sim_h_cg = get_env_double("SIM_H_CG", 0.0703);
    const double sim_steer_rate_max = get_env_double("SIM_STEER_RATE_MAX", 2.8492);
    const double sim_steer_angle_max = get_env_double("SIM_STEER_ANGLE_MAX", MAX_STEERING);
    const double sim_steer_gain = get_env_double("SIM_STEER_GAIN", 1.0085301459687404);
    const double sim_steer_gain_high_slip = get_env_double("SIM_STEER_GAIN_HIGH_SLIP", 0.6541720766809247);
    const double sim_v_switch = get_env_double("SIM_V_SWITCH", 7.319);
    const double sim_v_min = get_env_double("SIM_V_MIN", 0.5);
    const double sim_v_max = get_env_double("SIM_V_MAX", 20.0);
    const double sim_roll_res_n = get_env_double("SIM_ROLL_RES_N", ROLLING_RESISTANCE_N);
    const double sim_pacejka_c = get_env_double("SIM_PACEJKA_C", 1.6041121492252324);
    const double sim_pacejka_c_front = get_env_double("SIM_PACEJKA_C_FRONT", 1.8031639754063644);
    const double sim_pacejka_c_rear = get_env_double("SIM_PACEJKA_C_REAR", 1.7681655069132207);
    const double sim_slip_blend_start = get_env_double("SIM_SLIP_BLEND_START", 0.1643527788471148);
    const double sim_slip_blend_end = get_env_double("SIM_SLIP_BLEND_END", 0.5319307735091576);
    const double sim_slip_blend_start_front = get_env_double("SIM_SLIP_BLEND_START_FRONT", 0.1643527788471148);
    const double sim_slip_blend_end_front = get_env_double("SIM_SLIP_BLEND_END_FRONT", 0.5319307735091576);
    const double sim_slip_blend_start_rear = get_env_double("SIM_SLIP_BLEND_START_REAR", 0.2502122916247753);
    const double sim_slip_blend_end_rear = get_env_double("SIM_SLIP_BLEND_END_REAR", 0.47793678552502167);
    const double sim_combined_slip_gain = get_env_double("SIM_COMBINED_SLIP_GAIN", 0.10359393575265835);
    const double sim_front_peak_drop = get_env_double("SIM_FRONT_PEAK_DROP", 0.11804981810838257);
    const double sim_front_peak_drop_start = get_env_double("SIM_FRONT_PEAK_DROP_START", 0.13813810031946996);
    const double sim_front_peak_drop_end = get_env_double("SIM_FRONT_PEAK_DROP_END", 0.48938120479012814);
    const double sim_front_peak_drop_pow = get_env_double("SIM_FRONT_PEAK_DROP_POW", 1.0);
    const double sim_front_combined_gain = get_env_double("SIM_FRONT_COMBINED_GAIN", 0.13366870620631957);
    const double sim_front_peak_floor = get_env_double("SIM_FRONT_PEAK_FLOOR", 0.2708096984131235);
    const double sim_noise_pos_m = get_env_double("SIM_NOISE_POS_M", NOISE_POS_M);
    const double sim_noise_hdg_rad = get_env_double("SIM_NOISE_HDG_RAD", NOISE_HDG_RAD);
    const double sim_noise_vx_ms = get_env_double("SIM_NOISE_VX_MS", NOISE_VX_MS);
    const double sim_noise_vy_ms = get_env_double("SIM_NOISE_VY_MS", NOISE_VY_MS);
    const double sim_noise_omega_rad = get_env_double("SIM_NOISE_OMEGA_RAD", NOISE_OMEGA_RAD);
    int realistic_actuation = (getenv("REALISTIC_ACTUATION") && atoi(getenv("REALISTIC_ACTUATION")));
    if (!realistic_actuation) {
        if (fabs(drag_c0) > 1e-9 || fabs(drag_c1) > 1e-9 || fabs(drag_c2) > 1e-9 ||
            accel_tau_pos > 1e-6 || accel_tau_neg > 1e-6 ||
            fabs(accel_gain_pos - 1.0) > 1e-9 || fabs(accel_gain_neg - 1.0) > 1e-9) {
            realistic_actuation = 1;
        }
    }
    if (realistic_actuation && verbose) {
        printf("    Actuation model: DRAG_C0=%.3f DRAG_C1=%.3f DRAG_C2=%.3f ACCEL_TAU_POS=%.3f ACCEL_TAU_NEG=%.3f ACCEL_GAIN_POS=%.3f ACCEL_GAIN_NEG=%.3f\n",
               drag_c0, drag_c1, drag_c2, accel_tau_pos, accel_tau_neg, accel_gain_pos, accel_gain_neg);
    }
    if (verbose) {
        printf("    Plant model: MU=%.4f FRONT_MU=%.4f REAR_MU=%.4f MASS=%.3f IZ=%.4f C_SF=%.3f->%.3f C_SR=%.3f->%.3f LF=%.3f LR=%.3f H_CG=%.4f\n",
               sim_mu, sim_mu_front, sim_mu_rear, sim_mass, sim_Iz, sim_C_Sf, sim_C_Sf_high_slip, sim_C_Sr, sim_C_Sr_high_slip, sim_lf, sim_lr, sim_h_cg);
        printf("                 STEER_MAX=%.4f STEER_RATE_MAX=%.4f STEER_GAIN=%.4f->%.4f V_SWITCH=%.3f V_RANGE=[%.2f, %.2f] ROLL_RES=%.3f PACEJKA_C=%.3f F/R=[%.3f, %.3f] COMBINED=%.3f FRONT_COMBINED=%.3f\n",
               sim_steer_angle_max, sim_steer_rate_max, sim_steer_gain, sim_steer_gain_high_slip, sim_v_switch, sim_v_min, sim_v_max, sim_roll_res_n, sim_pacejka_c, sim_pacejka_c_front, sim_pacejka_c_rear, sim_combined_slip_gain, sim_front_combined_gain);
        printf("                 SLIP_BLEND_FRONT=[%.3f, %.3f] REAR=[%.3f, %.3f] FRONT_DROP=[%.3f, %.3f] DROP=%.3f POW=%.3f FLOOR=%.3f\n",
               sim_slip_blend_start_front, sim_slip_blend_end_front, sim_slip_blend_start_rear, sim_slip_blend_end_rear,
               sim_front_peak_drop_start, sim_front_peak_drop_end, sim_front_peak_drop, sim_front_peak_drop_pow, sim_front_peak_floor);
        printf("                 NOISE pos=%.4f hdg=%.4f vx=%.4f vy=%.4f omega=%.4f\n",
               sim_noise_pos_m, sim_noise_hdg_rad, sim_noise_vx_ms, sim_noise_vy_ms, sim_noise_omega_rad);
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

    if (local_replay_log_path && local_replay_log_path[0]) {
        if (!init_local_raceline_replay(local_replay_log_path, local_replay_start_ns)) {
            fprintf(stderr, "[SIM] Failed to initialize REPLAY_LOCAL_RACELINE_LOG=%s\n", local_replay_log_path);
            return 1;
        }
        g_local_replay_progress_mode = (local_replay_mode && strcmp(local_replay_mode, "progress") == 0) ? 1 : 0;
        if (g_local_replay_progress_mode) {
            if (!load_local_raceline_replay_index(local_replay_index_path)) {
                fprintf(stderr, "[SIM] Failed to initialize REPLAY_LOCAL_RACELINE_INDEX=%s\n",
                        local_replay_index_path ? local_replay_index_path : "(null)");
                free_local_raceline_replay();
                return 1;
            }
        }
        if (verbose) {
            printf("    Local replay: %s (snapshots=%d, mode=%s, start_ns=%lld)\n",
                   local_replay_log_path, g_local_replay_snapshot_count,
                   g_local_replay_progress_mode ? "progress" : "time",
                   (long long)g_local_replay_start_ns);
        }
    }

    /* Optional: emulate hardware /local_raceline processing by recomputing
     * heading/curvature from geometry (and optionally s). */
    if (getenv("RECOMPUTE_GEOMETRY") && atoi(getenv("RECOMPUTE_GEOMETRY"))) {
        const int w = (getenv("GEOM_WINDOW") ? atoi(getenv("GEOM_WINDOW")) : 3);
        const int recompute_s = (getenv("RECOMPUTE_S") && atoi(getenv("RECOMPUTE_S")));
        recompute_raceline_geometry(w, recompute_s);
        if (verbose) {
            printf("    Geometry recompute: window=%d recompute_s=%d\n", w, recompute_s);
        }
    }

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
    cfg.weight_lateral_error          = (float)((env = getenv("Q_LAT"))       ? atof(env) : 200.0);
    cfg.weight_heading_error          = (float)((env = getenv("Q_HDG"))       ? atof(env) : 28.8);
    cfg.weight_velocity               = (float)((env = getenv("Q_VEL"))       ? atof(env) : 30.0);
    cfg.weight_lateral_velocity       = (float)((env = getenv("Q_LAT_VEL"))   ? atof(env) : 1.04);
    cfg.weight_yaw_rate               = (float)((env = getenv("Q_YAW"))       ? atof(env) : 1.5);
    cfg.weight_steering_effort        = (float)((env = getenv("R_STEER"))     ? atof(env) : 1.5);
    cfg.weight_acceleration_effort    = (float)((env = getenv("R_ACCEL"))     ? atof(env) : 0.01);
    cfg.weight_steering_rate          = (float)((env = getenv("W_JERK"))      ? atof(env) : 0.04);
    cfg.weight_acceleration_rate      = (float)((env = getenv("W_ACCEL_RATE"))? atof(env) : 0.10);
    {
        const double footprint_margin = VEHICLE_HALF_WIDTH + body_safety_margin;
        const double margin_env = get_env_double("WALL_MARGIN", footprint_margin);
        const double effective_margin = fmax(footprint_margin, margin_env);
        cfg.wall_margin = (float)effective_margin;
    }
    cfg.max_solver_iterations       = (env = getenv("MAX_ITER")) ? atoi(env) : 100;
    cfg.solver_convergence_tolerance    = (float)((env = getenv("TOL")) ? atof(env) : 0.05);
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

    /* Spawn at selected raceline waypoint, optionally shifted laterally and with a custom initial speed. */
    const Waypoint_t *start_wp = &raceline[start_index];
    const double start_normal = start_wp->psi + M_PI / 2.0;
    VehicleState_t state;
    state.pos_x = (float)(start_wp->x + start_offset_lat * cos(start_normal) + start_offset_x);
    state.pos_y = (float)(start_wp->y + start_offset_lat * sin(start_normal) + start_offset_y);
    state.heading = (float)(wrap_angle(start_wp->psi + start_heading_offset));
    state.long_vel = (float)(start_speed);
    state.lat_vel = (float)(start_lat_speed);
    state.yaw_rate = (float)(start_yaw_rate);
    last_closest = start_index;
    local_last_closest = 0;

    if (verbose || fabs(start_offset_lat) > 1e-6 || fabs(start_heading_offset) > 1e-6 ||
        start_speed > 1e-6 || fabs(start_lat_speed) > 1e-6 || fabs(start_yaw_rate) > 1e-6 ||
        fabs(start_steer) > 1e-6) {
        printf("    Start state: idx=%d, lat_offset=%+.3fm, hdg_offset=%+.3frad, vx=%.2fm/s vy=%.2fm/s yaw_rate=%.2frad/s steer=%.3frad\n",
               start_index, start_offset_lat, start_heading_offset, start_speed, start_lat_speed, start_yaw_rate, start_steer);
    }

    FILE *sim_trace_file = NULL;
    const char *sim_trace_path = getenv("SIM_TRACE_LOG");
    if (sim_trace_path && sim_trace_path[0]) {
        sim_trace_file = fopen(sim_trace_path, "w");
        if (!sim_trace_file) {
            perror("[SIM] fopen(SIM_TRACE_LOG)");
            return 1;
        }
        fprintf(sim_trace_file,
                "sim_time_s,step,closest_wp,closest_local,true_path_s,completed_laps,"
                "pos_x,pos_y,heading,e_y,e_psi,vx,vy,omega,"
                "v_ref0,kappa0,cmd_steer,cmd_accel,actual_steer,solver_iter,solver_status,wall_hit\n");
        fflush(sim_trace_file);
    }

    /* Tracking metrics */
    double max_lat_err = 0, sum_lat_err = 0;
    double max_hdg_err = 0, sum_hdg_err = 0;
    double max_vel_err = 0, sum_vel_err = 0;
    int wall_collisions = 0;
    int solver_ok = 0, solver_calls = 0;
    int solver_optimal = 0, solver_max_iter = 0;
    double prev_steer = start_steer;
    double actual_steer = start_steer;  /* Steering target fed into actuator dynamics */
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
    double accel_applied = 0.0;  /* 1st-order longitudinal actuator state (optional) */
    int steps_executed = 0;
    int last_solver_status = -1;

    if (verbose) {
        if (local_raceline_sim) {
            printf("\n  Step | Time  | vx    | v_ref | e_y   | e_psi | cmd_st | act_st | accel | iter | wp_g | wp_l | wall?\n");
            printf("  -----|-------|-------|-------|-------|-------|--------|--------|-------|------|------|------|------\n");
        } else {
            printf("\n  Step | Time  | vx    | v_ref | e_y   | e_psi | cmd_st | act_st | accel | iter | wp_g | wall?\n");
            printf("  -----|-------|-------|-------|-------|-------|--------|--------|-------|------|------|------\n");
        }
    }

    for (int step = 0; step < SIM_STEPS; step++) {
        steps_executed = step + 1;
        double t = step * SIM_DT;
        double px = (double)(state.pos_x);
        double py = (double)(state.pos_y);
        double psi = (double)(state.heading);
        double vx = (double)(state.long_vel);

        int closest_global = find_closest_waypoint(px, py, psi);

        FrenetState_t frenet = vehicle_to_frenet(&state, closest_global);
        PathProjection_t state_proj = project_pose_to_raceline(px, py, psi, closest_global);

        /* MPC input can optionally use a local open segment (hardware-like). */
        int closest_local = 0;
        FrenetState_t frenet_for_mpc = frenet;
        if (local_raceline_sim)
        {
            if (g_local_replay_enabled) {
                if (g_local_replay_progress_mode)
                    maybe_update_local_raceline_replay_progress(state_proj.s);
                else
                    maybe_update_local_raceline_replay(t);
            } else {
                build_local_segment(closest_global, local_start_offset_points, local_segment_points,
                                    local_heading_window, local_curvature_window);
            }
            closest_local = find_closest_waypoint_local(px, py, psi);
            frenet_for_mpc = vehicle_to_frenet_local(&state, closest_local);
        }

        /* Wall check and metrics use TRUE state (no sensor noise) */
        double true_px = (double)(true_state.pos_x);
        double true_py = (double)(true_state.pos_y);
        double true_psi = (double)(true_state.heading);
        int true_closest = realistic_noise ? find_closest_waypoint(true_px, true_py, true_psi) : closest_global;
        FrenetState_t true_frenet = realistic_noise ? vehicle_to_frenet(&true_state, true_closest) : frenet;
        int true_local_closest = 0;
        FrenetState_t true_local_frenet = {0};
        int have_true_local_frenet = 0;
        PathProjection_t true_local_proj = {0};
        if (local_raceline_sim && local_count > 1) {
            true_local_closest = find_closest_waypoint_local(true_px, true_py, true_psi);
            true_local_frenet = vehicle_to_frenet_local(&true_state, true_local_closest);
            true_local_proj = project_pose_to_local_segment(true_px, true_py, true_psi, true_local_closest);
            have_true_local_frenet = 1;
        }
        /* Ground-truth speed magnitude for robust metrics (frame/sign independent). */
        const double speed_mps = hypot((double)(true_state.long_vel), (double)(true_state.lat_vel));
        if (speed_mps > max_vx) max_vx = speed_mps;
        sum_vx += speed_mps;
        double e_y = have_true_local_frenet ? (double)(true_local_frenet.flat_error) : (double)(true_frenet.flat_error);
        double e_psi = have_true_local_frenet ? (double)(true_local_frenet.fhead_error) : (double)(true_frenet.fhead_error);
        PathProjection_t true_proj = project_pose_to_raceline(true_px, true_py, true_psi, true_closest);

        if (!progress_initialized) {
            start_projected_s = true_proj.s;
            last_projected_s = true_proj.s;
            unwrapped_s = true_proj.s;
            next_lap_marker_s = start_projected_s + g_track_length_m;
            last_lap_cross_time = 0.0;
            progress_initialized = 1;
        } else {
            double ds = true_proj.s - last_projected_s;
            if (g_track_length_m > 1e-6) {
                if (ds < -0.5 * g_track_length_m) ds += g_track_length_m;
                else if (ds > 0.5 * g_track_length_m) ds -= g_track_length_m;
            }
            unwrapped_s += ds;
            last_projected_s = true_proj.s;
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
        if (local_raceline_sim && local_count > 1) {
            Waypoint_t local_wall_wp = sample_local_by_s(true_local_proj.s);
            left_wall = local_wall_wp.left_bound;
            right_wall = local_wall_wp.right_bound;
        }
        const double effective_wall_margin = (double)(cfg.wall_margin);
        int wall_hit = 0;
        if (e_y > (left_wall - effective_wall_margin))  { wall_hit = 1;  wall_collisions++; }
        if (e_y < -(right_wall - effective_wall_margin)){ wall_hit = -1; wall_collisions++; }
        if (wall_hit) {
            printf("\n  !!! WALL CRASH: e_y = %.3f m (bound: %.3f) at step %d (t=%.2fs, wp=%d, v=%.1f) !!!\n",
                   e_y, wall_hit > 0 ? left_wall : right_wall, step, t, true_closest, speed_mps);
            break;
        }

        /* Recompute MPC at the configured control interval. */
        double steer = cmd_steer;
        double accel_cmd = cmd_accel;
        int iter = 0;
        double v_ref_print = raceline[closest_global].vx;
        double kappa_ref_print = raceline[closest_global].kappa;

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
            if (local_raceline_sim) {
                build_reference_local(cfg.prediction_horizon_steps, ref);
            } else {
                build_reference(closest_global, cfg.prediction_horizon_steps, ref);
            }
            v_ref_print = (double)ref[0].reference_velocity;
            kappa_ref_print = (double)ref[0].path_curvature;

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
            if (status == MPC_SOLVER_STATUS_SUCCESS) {
                solver_ok++;
                solver_optimal++;
            } else if (status == MPC_SOLVER_STATUS_MAXIMUM_ITERATIONS_REACHED) {
                solver_ok++;
                solver_max_iter++;
            }
            last_solver_status = (int)status;
            solver_calls++;

            cmd_steer = steer;
            cmd_accel = accel_cmd;
        }

        /* Physical saturation: steering and acceleration */
        if (steer > sim_steer_angle_max) steer = sim_steer_angle_max;
        if (steer < -sim_steer_angle_max) steer = -sim_steer_angle_max;
        if (accel_cmd > PHYSICAL_MAX_ACCEL) accel_cmd = PHYSICAL_MAX_ACCEL;
        if (accel_cmd < -PHYSICAL_MAX_ACCEL) accel_cmd = -PHYSICAL_MAX_ACCEL;

        /* Use command directly as actuator target; actuator lag is modeled by
         * the ST steering-rate dynamics below (sv_max constrained). */
        actual_steer = steer;

        /* Metrics */
        if (fabs(e_y) > max_lat_err) max_lat_err = fabs(e_y);
        sum_lat_err += fabs(e_y);
        if (fabs(e_psi) > max_hdg_err) max_hdg_err = fabs(e_psi);
        sum_hdg_err += fabs(e_psi);
        double vel_err = fabs(speed_mps - raceline[true_closest].vx);
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
                if (local_raceline_sim) {
                    printf("  %4d | %5.2f | %5.2f | %5.2f | %+.3f | %+.3f | %+.4f | %+.4f | %+.2f | %4d | %4d | %4d | %s\n",
                           step, t, speed_mps, v_ref_print, e_y, e_psi, steer, actual_steer, accel_cmd, iter,
                           true_closest, closest_local,
                           wall_hit > 0 ? "LEFT!" : (wall_hit < 0 ? "RIGHT!" : ""));
                } else {
                    printf("  %4d | %5.2f | %5.2f | %5.2f | %+.3f | %+.3f | %+.4f | %+.4f | %+.2f | %4d | %4d | %s\n",
                           step, t, speed_mps, v_ref_print, e_y, e_psi, steer, actual_steer, accel_cmd, iter,
                           true_closest,
                           wall_hit > 0 ? "LEFT!" : (wall_hit < 0 ? "RIGHT!" : ""));
                }
            }
        }

        if (sim_trace_file) {
            fprintf(sim_trace_file,
                    "%.6f,%d,%d,%d,%.6f,%d,"
                    "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                    "%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%d\n",
                    t, step, true_closest, closest_local, true_proj.s, completed_laps,
                    true_px, true_py, true_psi, e_y, e_psi,
                    (double)(true_state.long_vel), (double)(true_state.lat_vel), (double)(true_state.yaw_rate),
                    v_ref_print, kappa_ref_print, steer, accel_cmd, actual_steer,
                    iter, last_solver_status, wall_hit);
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
            const double mu = sim_mu, mass = sim_mass, Iz = sim_Iz;
            const double lf = sim_lf, lr = sim_lr, h_cg = sim_h_cg;
            const double g_acc = 9.81;
            const double sv_max = sim_steer_rate_max;
            const double s_max = sim_steer_angle_max;
            const double v_switch = sim_v_switch;
            const double v_min = sim_v_min, v_max = sim_v_max;
            const double lwb = lf + lr;

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

            /* Longitudinal actuation: optional first-order tracking + asymmetry
             * between drive/brake (matches accel-to-current "feel" better than
             * directly applying MPC a_cmd). */
            double accl = accel_cmd;
            if (realistic_actuation) {
                const double gain = (accl >= 0.0) ? accel_gain_pos : accel_gain_neg;
                double a_target = accl * gain;
                const double tau = (a_target >= 0.0) ? accel_tau_pos : accel_tau_neg;
                if (tau > 1e-6) {
                    double alpha = SIM_DT / tau;
                    if (alpha > 1.0) alpha = 1.0;
                    accel_applied += alpha * (a_target - accel_applied);
                } else {
                    accel_applied = a_target;
                }
                accl = accel_applied;
            }

            /* accl_constraints: v_switch power limit */
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
                    double delta_eff = sim_steer_gain * (DELTA); \
                    double sv_eff = sim_steer_gain * (SV); \
                    double beta_hat = atan(tan(delta_eff) * lr / lwb); \
                    double beta_dot = (1.0 / (1.0 + pow(tan(delta_eff) * lr / lwb, 2.0))) \
                                    * (lr / (lwb * pow(cos(delta_eff), 2.0))) * (sv_eff); \
                    (dX) = (V) * cos((PSI) + beta_hat); \
                    (dY) = (V) * sin((PSI) + beta_hat); \
                    (dDELTA) = (SV); \
                    (dV) = (ACCL); \
                    (dPSI) = (V) * cos(beta_hat) * tan(delta_eff) / lwb; \
                    (dPSI_DOT) = (1.0 / lwb) * ( \
                        (ACCL) * cos(BETA) * tan(delta_eff) \
                        - (V) * sin(BETA) * tan(delta_eff) * beta_dot \
                        + ((V) * cos(BETA) * (sv_eff)) / pow(cos(delta_eff), 2.0)); \
                    (dBETA) = beta_dot; \
                } else { \
                    /* Dynamic ST mode — full nonlinear (matching gym single_track.py) */ \
                    double vx_ = (V) * cos(BETA); \
                    double vy_ = (V) * sin(BETA); \
                    double vx_safe_ = (vx_ > 0.5) ? vx_ : 0.5; \
                    /* Normal forces with longitudinal load transfer */ \
                    double Fzf_ = mass * (g_acc * lr - (ACCL) * h_cg) / lwb; \
                    double Fzr_ = mass * (g_acc * lf + (ACCL) * h_cg) / lwb; \
                    /* Provisional slip angles for slip-dependent steering compliance. */ \
                    double alpha_f_raw_ = sim_steer_gain * (DELTA) - atan2(vy_ + lf * (PSI_DOT_VAL), vx_safe_); \
                    double alpha_r_raw_ = -atan2(vy_ - lr * (PSI_DOT_VAL), vx_safe_); \
                    double slip_front_raw_ = fabs(alpha_f_raw_); \
                    double steer_blend_; \
                    if (slip_front_raw_ <= sim_slip_blend_start_front) steer_blend_ = 0.0; \
                    else if (slip_front_raw_ >= sim_slip_blend_end_front) steer_blend_ = 1.0; \
                    else steer_blend_ = (slip_front_raw_ - sim_slip_blend_start_front) / (sim_slip_blend_end_front - sim_slip_blend_start_front); \
                    steer_blend_ = steer_blend_ * steer_blend_ * (3.0 - 2.0 * steer_blend_); \
                    double steer_gain_eff_ = sim_steer_gain + (sim_steer_gain_high_slip - sim_steer_gain) * steer_blend_; \
                    double delta_eff = steer_gain_eff_ * (DELTA); \
                    /* atan2-based slip angles (not small-angle approx) */ \
                    double alpha_f_ = delta_eff - atan2(vy_ + lf * (PSI_DOT_VAL), vx_safe_); \
                    double alpha_r_ = -atan2(vy_ - lr * (PSI_DOT_VAL), vx_safe_); \
                    double slip_front_ = fabs(alpha_f_); \
                    double slip_rear_ = fabs(alpha_r_); \
                    double slip_blend_front_; \
                    if (slip_front_ <= sim_slip_blend_start_front) slip_blend_front_ = 0.0; \
                    else if (slip_front_ >= sim_slip_blend_end_front) slip_blend_front_ = 1.0; \
                    else slip_blend_front_ = (slip_front_ - sim_slip_blend_start_front) / (sim_slip_blend_end_front - sim_slip_blend_start_front); \
                    slip_blend_front_ = slip_blend_front_ * slip_blend_front_ * (3.0 - 2.0 * slip_blend_front_); \
                    double slip_blend_rear_; \
                    if (slip_rear_ <= sim_slip_blend_start_rear) slip_blend_rear_ = 0.0; \
                    else if (slip_rear_ >= sim_slip_blend_end_rear) slip_blend_rear_ = 1.0; \
                    else slip_blend_rear_ = (slip_rear_ - sim_slip_blend_start_rear) / (sim_slip_blend_end_rear - sim_slip_blend_start_rear); \
                    slip_blend_rear_ = slip_blend_rear_ * slip_blend_rear_ * (3.0 - 2.0 * slip_blend_rear_); \
                    double C_Sf_eff_ = sim_C_Sf + (sim_C_Sf_high_slip - sim_C_Sf) * slip_blend_front_; \
                    double C_Sr_eff_ = sim_C_Sr + (sim_C_Sr_high_slip - sim_C_Sr) * slip_blend_rear_; \
                    double combined_scale_ = 1.0; \
                    double accel_usage_ = 0.0; \
                    if (sim_combined_slip_gain > 1e-9) { \
                        accel_usage_ = fabs(ACCL) / PHYSICAL_MAX_ACCEL; \
                        if (accel_usage_ > 1.0) accel_usage_ = 1.0; \
                        combined_scale_ = 1.0 - sim_combined_slip_gain * accel_usage_; \
                        if (combined_scale_ < 0.30) combined_scale_ = 0.30; \
                    } \
                    double front_drop_blend_; \
                    if (slip_front_ <= sim_front_peak_drop_start) front_drop_blend_ = 0.0; \
                    else if (slip_front_ >= sim_front_peak_drop_end) front_drop_blend_ = 1.0; \
                    else front_drop_blend_ = (slip_front_ - sim_front_peak_drop_start) / (sim_front_peak_drop_end - sim_front_peak_drop_start); \
                    front_drop_blend_ = front_drop_blend_ * front_drop_blend_ * (3.0 - 2.0 * front_drop_blend_); \
                    if (sim_front_peak_drop_pow > 1.0) front_drop_blend_ = pow(front_drop_blend_, sim_front_peak_drop_pow); \
                    double front_peak_scale_ = 1.0 - sim_front_peak_drop * front_drop_blend_; \
                    if (front_peak_scale_ < sim_front_peak_floor) front_peak_scale_ = sim_front_peak_floor; \
                    double front_combined_scale_ = 1.0 - sim_front_combined_gain * accel_usage_ * front_drop_blend_; \
                    if (front_combined_scale_ < sim_front_peak_floor) front_combined_scale_ = sim_front_peak_floor; \
                    front_peak_scale_ *= front_combined_scale_; \
                    /* Lateral tire forces */ \
                    double Fyf_, Fyr_; \
                    if (realistic_tires) { \
                        /* Pacejka magic formula: Fy = D*sin(C*atan(B*alpha)) */ \
                        double B_f = C_Sf_eff_ / sim_pacejka_c_front; \
                        double B_r = C_Sr_eff_ / sim_pacejka_c_rear; \
                        double D_f = sim_mu_front * Fzf_ * front_peak_scale_; \
                        double D_r = sim_mu_rear * Fzr_; \
                        Fyf_ = combined_scale_ * D_f * sin(sim_pacejka_c_front * atan(B_f * alpha_f_)); \
                        Fyr_ = combined_scale_ * D_r * sin(sim_pacejka_c_rear * atan(B_r * alpha_r_)); \
                    } else { \
                        /* Linear tire model: Fy = mu * C_S * alpha * Fz */ \
                        Fyf_ = combined_scale_ * sim_mu_front * front_peak_scale_ * C_Sf_eff_ * alpha_f_ * Fzf_; \
                        Fyr_ = combined_scale_ * sim_mu_rear * C_Sr_eff_ * alpha_r_ * Fzr_; \
                    } \
                    /* Longitudinal force */ \
                    double Fx_raw_ = mass * (ACCL); \
                    double Fx_ = Fx_raw_; \
                    if (realistic_drive) { \
                        /* Rolling resistance as constant drag force. \
                         * Note: VESC ACCEL_TO_CURRENT compensates for drivetrain \
                         * efficiency, so we do NOT apply eta (would double-count). */ \
                        Fx_ -= sim_roll_res_n; \
                    } \
                    if (realistic_actuation) { \
                        /* Coastdown drag in acceleration domain: a_drag(v)=c0+c1*|v|+c2*v^2 */ \
                        const double v_abs_ = fabs(vx_); \
                        double a_drag_ = drag_c0 + drag_c1 * v_abs_ + drag_c2 * v_abs_ * v_abs_; \
                        if (a_drag_ > 0.0) { \
                            Fx_ -= mass * a_drag_; \
                        } \
                    } \
                    double cos_delta_ = cos(delta_eff); \
                    double sin_delta_ = sin(delta_eff); \
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
                state.pos_x = (float)(sn[0] + sim_noise_pos_m * randn());
                state.pos_y = (float)(sn[1] + sim_noise_pos_m * randn());
                state.heading = (float)(sn[4] + sim_noise_hdg_rad * randn());
                state.long_vel = (float)(
                    sn[3] * cos(sn[6]) + sim_noise_vx_ms * randn());
                state.lat_vel = (float)(
                    sn[3] * sin(sn[6]) + sim_noise_vy_ms * randn());
                state.yaw_rate = (float)(
                    sn[5] + sim_noise_omega_rad * randn());
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
            printf("\n  !!! CRASH: e_y = %.2f m at step %d (t=%.2fs, wp_g=%d) !!!\n", e_y, step, t, true_closest);
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
    const double solver_feasible_pct = 100.0 * solver_ok / (solver_calls > 0 ? solver_calls : 1);
    const double solver_optimal_pct = 100.0 * solver_optimal / (solver_calls > 0 ? solver_calls : 1);
    const double solver_max_iter_pct = 100.0 * solver_max_iter / (solver_calls > 0 ? solver_calls : 1);
    printf("\n  === Results (%.1f seconds, Riccati-ADMM, %.0fHz MPC) ===\n",
           simulated_time, 1.0 / mpc_ctrl_dt);
    printf("  Solver feasible:    %d / %d (%.1f%%)\n", solver_ok, solver_calls, solver_feasible_pct);
    printf("  Solver optimal:     %d / %d (%.1f%%)\n", solver_optimal, solver_calls, solver_optimal_pct);
    printf("  Solver max-iter:    %d / %d (%.1f%%)\n", solver_max_iter, solver_calls, solver_max_iter_pct);
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
    printf("  Wall collisions:    %d\n", wall_collisions);
    printf("  Time above 5 m/s:   %.1f / %.1f s (%.0f%%)\n",
           time_above_5ms, simulated_time,
           (simulated_time > 0.0) ? (100*time_above_5ms/simulated_time) : 0.0);
    printf("\n  --- Solver Performance ---\n");
    int mpc_calls = solver_calls;
    double avg_iters = (mpc_calls > 0) ? (double)total_iterations / mpc_calls : 0;
    double avg_solve = (mpc_calls > 0) ? total_solve_us / mpc_calls : 0;
    double solver_optimal_rate = (mpc_calls > 0) ? ((double)solver_optimal / mpc_calls) : 0.0;
    double solver_max_iter_rate = (mpc_calls > 0) ? ((double)solver_max_iter / mpc_calls) : 0.0;
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
    int speed_check_pass = 1;
    double ref_peak_speed = 0.0;
    double ref_avg_speed = 0.0;
    for (int i = 0; i < raceline_count; i++) {
        if (raceline[i].vx > ref_peak_speed)
            ref_peak_speed = raceline[i].vx;
        ref_avg_speed += raceline[i].vx;
    }
    if (raceline_count > 0)
        ref_avg_speed /= raceline_count;
    if (realistic_mode) {
        char speed_msg[128];
        snprintf(speed_msg, sizeof(speed_msg),
                 "Reaches driving speed (>5 m/s for >%.0f%% of time, realistic)",
                 speed_threshold * 100);
        speed_check_pass = (time_above_5ms > simulated_time * speed_threshold);
        check(speed_msg, speed_check_pass);
    } else {
        if (ref_avg_speed < 5.0) {
            char speed_msg[160];
            const double min_avg_ratio = 0.60;
            const double min_avg_speed = ref_avg_speed * min_avg_ratio;
            snprintf(speed_msg, sizeof(speed_msg),
                     "Tracks low-speed map (avg speed > %.0f%% of avg ref %.2f m/s)",
                     min_avg_ratio * 100.0, ref_avg_speed);
            speed_check_pass = (avg_vx > min_avg_speed);
            check(speed_msg, speed_check_pass);
        } else {
            speed_check_pass = (time_above_5ms > simulated_time * 0.5);
            check("Reaches driving speed (>5 m/s for >50% of time)", speed_check_pass);
        }
    }

    int failed_non_speed = tests_failed - (speed_check_pass ? 0 : 1);
    if (failed_non_speed < 0) failed_non_speed = 0;

    printf("\n=== RESULTS: %d passed, %d failed ===\n", tests_passed, tests_failed);

    /* Machine-readable CSV summary line for tuning scripts */
    if (getenv("MPC_TUNING_CSV")) {
        printf("CSV,%d,%d,%.4f,%.4f,%.4f,%.4f,%.2f,%.1f,%.1f,%d,%.1f,%.4f,%.4f,%.1f,%.4f,%.4f,%.4f,%d,%.4f,%.4f,%d,%d,%d,%.6f,%.6f\n",
            tests_passed, tests_failed,
            max_lat_err, avg_lat, max_hdg_err, avg_hdg,
            max_vx, avg_solve, max_solve_us,
            wall_collisions, time_above_5ms,
            max_vel_err, avg_vel, avg_iters, avg_vx,
            progress_m, avg_progress_mps, completed_laps,
            avg_lap_time, fabs(max_steer_change), steer_reversals,
            speed_check_pass, failed_non_speed,
            solver_optimal_rate, solver_max_iter_rate);
    }
    if (sim_trace_file) fclose(sim_trace_file);
    free_local_raceline_replay();
    return tests_failed > 0 ? 1 : 0;
}
