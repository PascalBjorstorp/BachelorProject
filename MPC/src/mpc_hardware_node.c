/**
 * @file mpc_hardware_node.c
 * @brief MPC Riccati-ADMM ROS2 Node for F1/10th Real Hardware
 * @details Production ROS2 node for the F1TENTH car running on Jetson Xavier
 *          NX. Uses the same MPC core library as simulation but optimized for
 *          real-time embedded execution and hardware feedback integration.
 *
 * Production ROS2 node for the F1TENTH car running on Jetson Xavier NX.
 * Uses the same MPC core library (Riccati-ADMM solver) as the simulation
 * node, but stripped of simulation-specific code and optimized for
 * real-time 200 Hz execution on embedded hardware.
 *
 * Architecture (EKF-driven):
 *   - Servo feedback callback: stores actual steering angle from VESC
 *   - Odometry callback: stores latest velocity state (vx/vy/omega) from VESC odom
 *   - EKF pose callback: receives map-frame pose, runs MPC, publishes result
 *
 * Topics:
 *   Subscribe: /sensors/servo_position_command (std_msgs/Float64) — servo fb [QoS(10)]
 *   Subscribe: /local_raceline         (nav_msgs/Path)         — lateral planner reference [QoS(10)]
 *   Subscribe: /ekf_pose               (geometry_msgs/PoseWithCovarianceStamped) — EKF pose [QoS(10)]
 *   Subscribe: /ego_racecar/odom       (nav_msgs/Odometry)     — velocity feedback [QoS(10)]
 *   Publish:   /drive                  (ackermann_msgs/AckermannDriveStamped) — mux [QoS(10)]
 *
 * @dependencies mpc.h, mpc_types.h, util_math.h, vehicle_model.h,
 *               rclc, rcl, nav_msgs, ackermann_msgs, std_msgs, geometry_msgs,
 *               <sched.h>, <sys/stat.h>
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <signal.h>
#include <sched.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <limits.h>
#include <stdint.h>
/* ROS2 C Client Library Headers */
#include "rcl/rcl.h"
#include "rcl/error_handling.h"
#include "rcl/timer.h"
#include "rcl/time.h"
#include "rclc/rclc.h"
#include "rclc/executor.h"
#include "rosidl_runtime_c/string_functions.h"
#include "rcutils/allocator.h"

/* ROS2 Message Types */
#include "nav_msgs/msg/odometry.h"
#include "nav_msgs/msg/path.h"
#include "ackermann_msgs/msg/ackermann_drive_stamped.h"
#include "std_msgs/msg/float64.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.h"

/* MPC Core Library Headers (Platform-Independent) */
#include "mpc.h"
#include "mpc_types.h"
#include "util_math.h"
#include "vehicle_model.h"

/*===========================================================================
 * Configuration Constants
 *===========================================================================*/

/** Configurable topic names */
static const char *g_drive_topic = "/drive";
static const char *g_servo_topic = "/sensors/servo_position_command";
static const char *g_odom_topic = "/ego_racecar/odom";
static const char *g_ekf_pose_topic = "/ekf_pose";
static const char *g_local_raceline_topic = "/local_raceline";
static int g_local_raceline_received = 0;
static int g_local_raceline_wait_logged = 0;
static int g_local_raceline_speed_warned = 0;
static int g_local_raceline_wall_warned = 0;
static const double FALLBACK_WALL_BOUND_M = 1.5;

/** Enable verbose logging (disabled by default for real-time performance) */
static int g_verbose = 1;

/** Set to 1 once the first EKF pose message has been received. */
static int g_ekf_pose_received = 0;

/** Safety watchdog timeout [seconds] */
static double g_watchdog_timeout_sec = 0.2;
/** Low-speed braking inhibit threshold [m/s]. <=0 disables. */
static double g_low_speed_brake_inhibit_vx = 0.7;
/** Minimum allowed acceleration when braking is inhibited [m/s^2]. */
static double g_low_speed_min_accel = 0.0;
static struct timespec g_last_servo_time;
/* Last published drive command (fallback uses these instead of forcing stop). */
static float g_last_cmd_steer = 0.0f;
static float g_last_cmd_speed = 0.0f;
static float g_last_cmd_accel = 0.0f;

/*===========================================================================
 * VESC Servo Conversion Parameters
 *===========================================================================
 * The VESC converts steering angle to servo position:
 *   servo_val = steering_to_servo_gain * corrected_angle + steering_to_servo_offset
 * where corrected_angle = c2·|δ|² + c1·|δ| + c0  (sign-preserved)
 *
 * Forward (angle → servo): applied in ackermann_to_vesc
 * Inverse (servo → angle): solve quadratic to recover δ from corrected_angle
 *   corrected = (servo_val - offset) / gain
 *   δ = sign(corrected) * (-c1 + sqrt(c1² - 4·c2·(c0 - |corrected|))) / (2·c2)
 */

/*===========================================================================
 * Servo Dynamics Tracking
 *===========================================================================
 * The MPC needs to know the actual servo position (delta_actual) to correctly
 * compute steering commands via the integrator: delta_cmd = delta_actual + dt * delta_dot.
 *
 * On the real car, the VESC publishes /sensors/servo_position_command which
 * gives the commanded servo value (0-1). We convert it back to steering angle.
 * If servo feedback is not available, fall back to rate-limited tracking.
 */
static double global_actual_steering_angle = 0.0;
static int g_use_steering_feedback = 0;

/*===========================================================================
 * Global State Variables (pre-allocated, no dynamic allocation in hot path)
 *===========================================================================*/

static TrajectoryWaypoint_t global_trajectory[TRAJECTORY_MAXIMUM_WAYPOINTS];
static int global_trajectory_count = 0;
static VehicleState_t global_vehicle_state = {0};
static FrenetState_t global_frenet_state = {0};
static ControlInput_t global_control_command = {0};
static int g_ekf_state_received = 0;
static volatile sig_atomic_t global_shutdown_requested = 0;
static rcl_context_t *global_ros2_context = NULL;

static rcl_publisher_t global_control_publisher;

static nav_msgs__msg__Odometry global_odometry_message_buffer;
static std_msgs__msg__Float64 global_servo_message_buffer;
static ackermann_msgs__msg__AckermannDriveStamped global_drive_message_buffer;
static geometry_msgs__msg__PoseWithCovarianceStamped global_ekf_pose_buffer;
static nav_msgs__msg__Path global_local_raceline_buffer;

static TrajectoryReferencePoint_t global_reference_trajectory[PREDICTION_HORIZON];

/** Latest EKF pose-derived state used by MPC cycle. */
static double g_latest_pos_x = 0.0;
static double g_latest_pos_y = 0.0;
static double g_latest_heading = 0.0;
static double g_latest_vx = 0.0;
static double g_latest_vy = 0.0;
static double g_latest_omega = 0.0;
static struct timespec g_last_ekf_time = {0, 0};
static int g_odometry_received = 0;
static int g_odom_wait_logged = 0;
static int g_last_control_time_valid = 0;
static struct timespec g_last_control_time = {0, 0};

/** Rolling solve-time instrumentation (always active, prints every 500 cycles) */
static double g_solve_time_sum_us = 0.0;
static double g_solve_time_max_us = 0.0;
static unsigned long g_solve_cycle_count = 0;
#define SOLVE_STATS_PRINT_INTERVAL 500

/** Optional solver telemetry logging for post-drive analysis. */
static FILE *g_solver_log_file = NULL;
static unsigned long g_solver_log_counter = 0;
static int g_solver_log_stride = 1;

/**
 * @brief Ensure all parent directories exist for a given file path.
 * @param filepath Target file path.
 * @return None.
 */
static void ensure_parent_directories(const char *filepath)
{
    if (filepath == NULL) return;

    char path_buf[PATH_MAX];
    size_t n = strlen(filepath);
    if (n == 0 || n >= sizeof(path_buf)) return;

    memcpy(path_buf, filepath, n + 1);

    char *last_slash = strrchr(path_buf, '/');
    if (last_slash == NULL) return;  /* No directory component */

    *last_slash = '\0';
    if (path_buf[0] == '\0') return;

    char tmp[PATH_MAX];
    size_t len = strlen(path_buf);
    if (len >= sizeof(tmp)) return;

    memcpy(tmp, path_buf, len + 1);

    for (char *p = tmp + 1; *p; p++) {
        if (*p == '/') {
            *p = '\0';
            if (mkdir(tmp, 0775) != 0 && errno != EEXIST) {
                return;
            }
            *p = '/';
        }
    }

    if (mkdir(tmp, 0775) != 0 && errno != EEXIST) {
        return;
    }
}

/** Number of executor handles: 4 subscriptions (odom, servo, ekf_pose, local_raceline). */
#define EXECUTOR_NUM_HANDLES 4

/*===========================================================================
 * Signal Handler for Graceful Shutdown
 *===========================================================================*/

/**
 * @brief Handle process termination signals and request ROS shutdown.
 * @param sig Received POSIX signal number.
 * @return None.
 */
static void signal_handler(int sig)
{
    (void)sig;
    global_shutdown_requested = 1;
    if (global_ros2_context != NULL && rcl_context_is_valid(global_ros2_context))
    {
        rcl_ret_t rc __attribute__((unused)) = rcl_shutdown(global_ros2_context);
    }
}

/*===========================================================================
 * Real-Time Setup (Jetson Xavier NX)
 *===========================================================================*/

/**
 * @brief Configure real-time scheduler policy and CPU affinity when available.
 * @return None.
 */
static void setup_realtime_scheduling(void)
{
    /* Try SCHED_FIFO for real-time priority */
    struct sched_param param;
    param.sched_priority = 49;  /* Below kernel threads (50+), above normal */

    if (sched_setscheduler(0, SCHED_FIFO, &param) == 0)
    {
        printf("[RT] SCHED_FIFO enabled (priority=%d)\n", param.sched_priority);
    }
    else
    {
        printf("[RT] SCHED_FIFO not available (errno=%d: %s) — running as normal priority\n",
               errno, strerror(errno));
        printf("[RT] Run as root or set CAP_SYS_NICE for real-time scheduling\n");
    }

    /* Pin to a specific CPU core to reduce cache thrashing */
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(5, &cpuset);  /* Jetson Xavier NX core 5 (big core) */

    if (sched_setaffinity(0, sizeof(cpuset), &cpuset) == 0)
    {
        printf("[RT] Pinned to CPU core 5\n");
    }
    else
    {
        /* Try core 3 as fallback (works on 4-core systems) */
        CPU_ZERO(&cpuset);
        CPU_SET(3, &cpuset);
        if (sched_setaffinity(0, sizeof(cpuset), &cpuset) == 0)
        {
            printf("[RT] Pinned to CPU core 3 (fallback)\n");
        }
        else
        {
            printf("[RT] CPU affinity not set (errno=%d) — using default\n", errno);
        }
    }
}

/*===========================================================================
 * Trajectory Metrics
 *===========================================================================*/

/** Average spacing of latest local raceline waypoints. */
static double g_avg_waypoint_spacing = 0.05;
/** Last closest index used for local raceline projection (stabilizes closest-point selection). */
static int g_last_local_closest_index = 0;

/**
 * @brief Wrap heading difference into [-pi, pi].
 * @param angle Angle in radians.
 * @return Wrapped angle in radians.
 */
static double wrap_angle_pi(double angle)
{
    while (angle > M_PI) angle -= TWO_PI;
    while (angle < -M_PI) angle += TWO_PI;
    return angle;
}

/*===========================================================================
 * Reference Trajectory Builder
 *===========================================================================*/

/**
 * @brief Find the closest forward-relevant waypoint in the latest local raceline.
 *
 * The lateral planner often publishes a segment that starts ahead of the car
 * (see PATH_START_OFFSET_POINTS). Using index 0 as the closest point can
 * create large artificial tracking errors (e_y/e_psi) → MPC brakes hard and
 * appears "stuck" on real hardware (especially in accel-to-current mode).
 *
 * @param position_x Vehicle x-position in map frame (m).
 * @param position_y Vehicle y-position in map frame (m).
 * @param vehicle_heading Vehicle heading in map frame (rad).
 * @return Closest waypoint index (0..global_trajectory_count-1).
 */
static int find_closest_waypoint_local(double position_x, double position_y, double vehicle_heading)
{
    if (global_trajectory_count <= 0)
    {
        return 0;
    }

    /* Local raceline is an open segment that typically starts near the vehicle.
     * Restrict closest-point search to an early window to avoid index "jumps"
     * to far-ahead points when the vehicle deviates. */
    const int max_search_points = 60;
    const int search_end =
        (global_trajectory_count < max_search_points) ? global_trajectory_count : max_search_points;

    int search_start = g_last_local_closest_index;
    if (search_start < 0) search_start = 0;
    if (search_start >= search_end) search_start = search_end - 1;

    const int search_window = 50;
    const int back_window = 5;

    int best_index = search_start;
    double best_score = 1e18;
    const double veh_dx = cos(vehicle_heading);
    const double veh_dy = sin(vehicle_heading);

    for (int offset = -back_window; offset < search_window; offset++)
    {
        int i = search_start + offset;
        if (i < 0) i = 0;
        if (i >= search_end) i = search_end - 1;

        const double dx = global_trajectory[i].x_meters - position_x;
        const double dy = global_trajectory[i].y_meters - position_y;
        const double dist2 = dx * dx + dy * dy;

        /* Penalize points behind the vehicle. */
        const double dot = dx * veh_dx + dy * veh_dy;
        const double score = dist2 + ((dot < 0.0) ? 25.0 : 0.0); /* +5m equiv penalty */

        if (score < best_score)
        {
            best_score = score;
            best_index = i;
        }
    }

    g_last_local_closest_index = best_index;
    return best_index;
}

/**
 * @brief Interpolate local raceline waypoint at an arbitrary arc-length.
 *
 * Unlike the simulator's global raceline, the local raceline segment is not
 * assumed to wrap (open interval). Query is clamped to [s_first, s_last].
 *
 * @param s_query Arc-length coordinate in meters.
 * @param out Destination waypoint pointer.
 * @return None.
 */
static void sample_waypoint_by_s_local(double s_query, TrajectoryWaypoint_t *out)
{
    if (out == NULL || global_trajectory_count <= 0)
    {
        return;
    }

    if (global_trajectory_count == 1)
    {
        *out = global_trajectory[0];
        out->s_meters = s_query;
        return;
    }

    const double s_first = global_trajectory[0].s_meters;
    const double s_last = global_trajectory[global_trajectory_count - 1].s_meters;
    double s = s_query;
    if (s <= s_first)
    {
        *out = global_trajectory[0];
        out->s_meters = s;
        return;
    }
    if (s >= s_last)
    {
        *out = global_trajectory[global_trajectory_count - 1];
        out->s_meters = s;
        return;
    }

    for (int i = 0; i < global_trajectory_count - 1; i++)
    {
        TrajectoryWaypoint_t *w0 = &global_trajectory[i];
        TrajectoryWaypoint_t *w1 = &global_trajectory[i + 1];
        if (s >= w0->s_meters && s <= w1->s_meters)
        {
            const double denom = w1->s_meters - w0->s_meters;
            const double t = (denom > 1e-9) ? ((s - w0->s_meters) / denom) : 0.0;

            *out = *w0;
            out->s_meters = s;
            out->x_meters = w0->x_meters + (w1->x_meters - w0->x_meters) * t;
            out->y_meters = w0->y_meters + (w1->y_meters - w0->y_meters) * t;
            {
                double dh = w1->heading_radians - w0->heading_radians;
                while (dh > M_PI) dh -= TWO_PI;
                while (dh < -M_PI) dh += TWO_PI;
                out->heading_radians = w0->heading_radians + t * dh;
            }
            out->curvature_radians_per_meter =
                w0->curvature_radians_per_meter +
                (w1->curvature_radians_per_meter - w0->curvature_radians_per_meter) * t;
            out->velocity_meters_per_second =
                w0->velocity_meters_per_second +
                (w1->velocity_meters_per_second - w0->velocity_meters_per_second) * t;
            out->left_bound_meters = w0->left_bound_meters + (w1->left_bound_meters - w0->left_bound_meters) * t;
            out->right_bound_meters = w0->right_bound_meters + (w1->right_bound_meters - w0->right_bound_meters) * t;
            return;
        }
    }

    /* Fallback (should not hit if s is within bounds). */
    *out = global_trajectory[global_trajectory_count - 1];
    out->s_meters = s;
}

/**
 * @brief Build MPC reference from the latest /local_raceline (s-interpolated).
 *
 * The lateral planner publishes a local (vehicle-anchored) path segment that
 * already represents the intended horizon ahead. In this mode we should not
 * resample by arc-length using the global raceline model; instead, take the
 * provided discrete points directly (clamped to the last point if the horizon
 * exceeds the message length).
 *
 * @return None.
 */
static void build_reference_from_local_raceline(int closest_index)
{
    if (global_trajectory_count <= 0)
    {
        return;
    }

    if (closest_index < 0) closest_index = 0;
    if (closest_index >= global_trajectory_count) closest_index = global_trajectory_count - 1;

    const MpcConfiguration_t cfg = mpc_get_configuration();
    const double pred_dt = (cfg.time_step > 0.0f) ? (double)cfg.time_step : (double)TIME_STEP_SECONDS;
    double s_query = global_trajectory[closest_index].s_meters;
    double step_velocity = global_trajectory[closest_index].velocity_meters_per_second;
    /* If the vehicle is significantly slower than the reference (e.g. after an
     * avoidance/stop event), advancing the query by v_ref*dt can jump far ahead
     * on the local segment and make the MPC "give up". Use a blended speed so
     * the horizon stays locally relevant during recovery. */
    {
        const double v_meas = fabs(g_latest_vx);
        if (isfinite(v_meas) && v_meas > 0.0)
        {
            const double v_ref0 = step_velocity;
            const double v_blend = 0.6 * v_meas + 0.4 * v_ref0;
            if (v_blend > 0.0) step_velocity = v_blend;
        }
    }

    for (int step = 0; step < PREDICTION_HORIZON; step++)
    {
        s_query += step_velocity * pred_dt;
        TrajectoryWaypoint_t wp = {0};
        sample_waypoint_by_s_local(s_query, &wp);
        const double traj_vel = wp.velocity_meters_per_second;
        step_velocity = traj_vel;

        global_reference_trajectory[step].reference_lateral_error = 0;
        global_reference_trajectory[step].reference_heading_error = 0;
        global_reference_trajectory[step].path_curvature = wp.curvature_radians_per_meter;
        global_reference_trajectory[step].left_wall_bound = wp.left_bound_meters;
        global_reference_trajectory[step].right_wall_bound = wp.right_bound_meters;
        global_reference_trajectory[step].reference_velocity = traj_vel;
        global_reference_trajectory[step].reference_lateral_velocity = 0;
        global_reference_trajectory[step].reference_yaw_rate =
            wp.curvature_radians_per_meter * traj_vel;
    }
}

/*===========================================================================
 * Helper Functions
 *===========================================================================*/

static double get_env_double_or_default(const char *name, double default_val)
{
    const char *env = getenv(name);
    if (env == NULL || env[0] == '\0') return default_val;
    char *end = NULL;
    double v = strtod(env, &end);
    if (end == env) return default_val;
    return v;
}

/**
 * @brief Clamp reference velocity during large-error recovery.
 *
 * Real hardware startup often begins with large heading/lateral errors and low speed.
 * If we command a high v_ref in that state, the MPC can saturate accel/steer, overshoot,
 * and get "stuck" in a limit cycle. This helper optionally caps v_ref across the
 * horizon while preserving the path curvature feedforward (yaw_rate = κ * v_ref).
 *
 * Enable via:
 *  - MPC_RECOVERY_EPSI_RAD (default 0.45)
 *  - MPC_RECOVERY_EY_M     (default 0.20)
 *  - MPC_RECOVERY_VREF_MAX (default 1.50)
 * Set thresholds <=0 to disable.
 */
static void maybe_apply_recovery_reference_cap(double ey, double epsi)
{
    const double epsi_th = get_env_double_or_default("MPC_RECOVERY_EPSI_RAD", 0.45);
    const double ey_th   = get_env_double_or_default("MPC_RECOVERY_EY_M", 0.20);
    const double v_cap   = get_env_double_or_default("MPC_RECOVERY_VREF_MAX", 1.50);

    if (!(v_cap > 0.0)) return;
    if (!(epsi_th > 0.0) && !(ey_th > 0.0)) return;

    const int trig =
        ((epsi_th > 0.0) && (fabs(epsi) > epsi_th)) ||
        ((ey_th > 0.0) && (fabs(ey) > ey_th));

    if (!trig) return;

    for (int k = 0; k < PREDICTION_HORIZON; k++)
    {
        if (global_reference_trajectory[k].reference_velocity > (float)v_cap)
        {
            global_reference_trajectory[k].reference_velocity = (float)v_cap;
            global_reference_trajectory[k].reference_yaw_rate =
                global_reference_trajectory[k].path_curvature * (float)v_cap;
        }
    }
}

/**
 * @brief Convert quaternion orientation to yaw angle.
 * @param qx Quaternion x component.
 * @param qy Quaternion y component.
 * @param qz Quaternion z component.
 * @param qw Quaternion w component.
 * @return Yaw angle in radians.
 */
static double quaternion_to_yaw_angle(double qx, double qy, double qz, double qw){
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    return atan2(siny_cosp, cosy_cosp);
}

/**
 * @brief Pre-allocate storage for a ROSIDL string.
 * @param str ROSIDL string object to initialize.
 * @param capacity Buffer capacity in bytes.
 * @return 1 on success, 0 on failure.
 */
static int preallocate_rosidl_string(rosidl_runtime_c__String *str, size_t capacity){
    if (str == NULL || capacity <= 1) return 0;
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    char *data = (char *)allocator.allocate(capacity, allocator.state);
    if (data == NULL) return 0;
    data[0] = '\0';
    str->data = data;
    str->size = 0;
    str->capacity = capacity;
    return 1;
}

/**
 * @brief Copy text into a pre-allocated ROSIDL string with truncation safety.
 * @param str Destination ROSIDL string.
 * @param value Source C string value.
 * @return None.
 */
static void set_rosidl_string(rosidl_runtime_c__String *str, const char *value){
    if (str == NULL || str->data == NULL || value == NULL) return;
    size_t length = strlen(value);
    if (length >= str->capacity) length = str->capacity - 1;
    memcpy(str->data, value, length);
    str->data[length] = '\0';
    str->size = length;
}

/**
 * @brief Compute elapsed wall-clock time between two timespec samples.
 * @param a Start timestamp.
 * @param b End timestamp.
 * @return Elapsed time in seconds.
 */
static double timespec_diff_sec(struct timespec *a, struct timespec *b){
    return (double)(b->tv_sec - a->tv_sec) +
           (double)(b->tv_nsec - a->tv_nsec) / 1e9;
}

/*===========================================================================
 * Frenet State Conversion
 *===========================================================================*/

/**
 * @brief Convert map-frame vehicle state into Frenet tracking state.
 * @param car_x Vehicle x-position in map frame (m).
 * @param car_y Vehicle y-position in map frame (m).
 * @param car_heading Vehicle heading in map frame (rad).
 * @param closest_index Closest trajectory waypoint index.
 * @param frenet_out Destination Frenet state pointer.
 * @return None.
 */
static void convert_to_frenet_state(
    double car_x, double car_y, double car_heading,
    int closest_index,
    FrenetState_t *frenet_out)
{
    int idx0 = closest_index;
    int idx1 = idx0 + 1;
    if (global_trajectory_count < 2)
    {
        return;
    }
    if (idx0 >= global_trajectory_count - 1)
    {
        idx0 = global_trajectory_count - 2;
    }
    if (idx0 < 0) idx0 = 0;
    idx1 = idx0 + 1;

    double ax = global_trajectory[idx0].x_meters;
    double ay = global_trajectory[idx0].y_meters;
    double bx = global_trajectory[idx1].x_meters;
    double by = global_trajectory[idx1].y_meters;

    /* Project car position onto segment A->B, parameter t in [0,1] */
    double abx = bx - ax, aby = by - ay;
    double apx = car_x - ax, apy = car_y - ay;
    double ab_len2 = abx * abx + aby * aby;
    double t = 0.0;
    if (ab_len2 > 1e-12)
        t = (apx * abx + apy * aby) / ab_len2;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;

    /* Interpolated path point */
    double path_x = ax + t * abx;
    double path_y = ay + t * aby;

    /* Interpolated heading (with angle wrapping) */
    double h0 = global_trajectory[idx0].heading_radians;
    double h1 = global_trajectory[idx1].heading_radians;
    double dh = h1 - h0;
    while (dh > M_PI) dh -= TWO_PI;
    while (dh < -M_PI) dh += TWO_PI;
    double path_heading = h0 + t * dh;

    /* Signed lateral error (positive = left of path) using exact trig of
     * interpolated heading. */
    double sin_h = sin(path_heading);
    double cos_h = cos(path_heading);
    double dx = car_x - path_x;
    double dy = car_y - path_y;
    double lateral_error = -dx * sin_h + dy * cos_h;

    double heading_error = car_heading - path_heading;
    while (heading_error > M_PI) heading_error -= TWO_PI;
    while (heading_error < -M_PI) heading_error += TWO_PI;

    frenet_out->flat_error = lateral_error;
    frenet_out->fhead_error = heading_error;
    frenet_out->flong_vel =
        global_vehicle_state.long_vel;
    frenet_out->flat_vel =
        global_vehicle_state.lat_vel;
    frenet_out->fyaw_rate =
        global_vehicle_state.yaw_rate;
}

/* 8-state augmented model verification note:
 * The FrenetState_t provides x0[0..4] = {e_y, e_psi, vx, vy, omega}.
 * The remaining augmented states are managed by the solver internally:
 *   x0[5] = delta_actual    — set via mpc_set_actual_previous_control()
 *   x0[6] = delta_rate_prev — stored by the solver after each solve
 *   x0[7] = accel_prev      — stored by the solver after each solve
 * The hardware node feeds delta_actual from servo feedback (or rate-limited
 * tracking) after each solve, which the solver uses on the NEXT call.
 */

/*===========================================================================
 * ROS2 Callback: Local Raceline Path Subscription
 *===========================================================================*/

/**
 * @brief Convert local raceline path into MPC trajectory waypoints.
 * @param message_in Pointer to nav_msgs/Path message.
 * @return None.
 */
void local_raceline_callback(const void *message_in)
{
    if (message_in == NULL) return;

    const nav_msgs__msg__Path *msg =
        (const nav_msgs__msg__Path *)message_in;

    if (msg->poses.size < 2)
    {
        return;
    }

    size_t waypoint_count = msg->poses.size;
    if (waypoint_count > TRAJECTORY_MAXIMUM_WAYPOINTS)
    {
        waypoint_count = TRAJECTORY_MAXIMUM_WAYPOINTS;
    }

    double cumulative_s = 0.0;
    double prev_x = 0.0;
    double prev_y = 0.0;
    size_t missing_speed_count = 0;
    size_t missing_wall_count = 0;

    for (size_t i = 0; i < waypoint_count; i++)
    {
        const geometry_msgs__msg__PoseStamped *pose = &msg->poses.data[i];
        TrajectoryWaypoint_t *wp = &global_trajectory[i];

        const double x = pose->pose.position.x;
        const double y = pose->pose.position.y;

        if (i > 0)
        {
            cumulative_s += hypot(x - prev_x, y - prev_y);
        }

        wp->s_meters = cumulative_s;
        wp->x_meters = x;
        wp->y_meters = y;

        double v_ref = fabs(pose->pose.position.z);
        if (!isfinite(v_ref) || v_ref < MIN_TRAJECTORY_SPEED_MPS)
        {
            v_ref = MIN_TRAJECTORY_SPEED_MPS;
            missing_speed_count++;
        }
        if (v_ref > TRAJECTORY_MAXIMUM_VELOCITY)
        {
            v_ref = TRAJECTORY_MAXIMUM_VELOCITY;
        }
        wp->velocity_meters_per_second = v_ref;

        double left_bound = pose->pose.orientation.x;
        double right_bound = pose->pose.orientation.y;
        if (!isfinite(left_bound) || left_bound <= 0.0)
        {
            left_bound = FALLBACK_WALL_BOUND_M;
            missing_wall_count++;
        }
        if (!isfinite(right_bound) || right_bound <= 0.0)
        {
            right_bound = FALLBACK_WALL_BOUND_M;
            missing_wall_count++;
        }
        wp->left_bound_meters = (float)left_bound;
        wp->right_bound_meters = (float)right_bound;
        /* Heading is computed from geometry after loading all points. */
        wp->heading_radians = 0.0;
        wp->curvature_radians_per_meter = 0.0;

        prev_x = x;
        prev_y = y;
    }

    /* Compute heading from finite differences of x/y to avoid relying on
     * PoseStamped.orientation (which may be unset or noisy on some stacks).
     *
     * NOTE: Keep heading local (small window) because it directly affects e_psi.
     * Compute curvature with a larger window below to avoid κ spikes from
     * dense/noisy points. */
    const size_t heading_window = 1;   /* points on each side (local heading) */
    const size_t curvature_window = 3; /* points on each side (smooth κ) */
    for (size_t i = 0; i < waypoint_count; i++)
    {
        size_t i_prev = (i > heading_window) ? (i - heading_window) : 0;
        size_t i_next = (i + heading_window < waypoint_count) ? (i + heading_window) : (waypoint_count - 1);

        const double dx =
            global_trajectory[i_next].x_meters - global_trajectory[i_prev].x_meters;
        const double dy =
            global_trajectory[i_next].y_meters - global_trajectory[i_prev].y_meters;

        double heading = 0.0;
        if ((dx * dx + dy * dy) > 1e-12)
        {
            heading = atan2(dy, dx);
        }
        else if (i > 0)
        {
            heading = global_trajectory[i - 1].heading_radians;
        }

        global_trajectory[i].heading_radians = heading;
    }

    if (waypoint_count >= 3)
    {
        /* Clamp κ to the maximum curvature implied by steering limits. This is
         * a pragmatic guardrail for local-planner paths that may contain
         * discontinuities (piecewise linear segments) or overly aggressive
         * cornering. Without this, κ can exceed what δ_max can realize and the
         * MPC can choose to stop rather than accept tracking error. */
        const double kappa_max =
            (VP_WHEELBASE_M > 1e-6f)
                ? (tan((double)VP_MAX_STEERING_RAD) / (double)VP_WHEELBASE_M)
                : 10.0;

        for (size_t i = 1; i + 1 < waypoint_count; i++)
        {
            const size_t i_prev = (i > curvature_window) ? (i - curvature_window) : 0;
            const size_t i_next = (i + curvature_window < waypoint_count) ? (i + curvature_window) : (waypoint_count - 1);

            const double dpsi = wrap_angle_pi(
                global_trajectory[i_next].heading_radians -
                global_trajectory[i_prev].heading_radians);
            const double ds = global_trajectory[i_next].s_meters - global_trajectory[i_prev].s_meters;
            const double ds_safe = (ds > 1e-6) ? ds : 1e-6;
            double kappa = dpsi / ds_safe;
            if (kappa > kappa_max) kappa = kappa_max;
            if (kappa < -kappa_max) kappa = -kappa_max;
            global_trajectory[i].curvature_radians_per_meter = kappa;
        }

        global_trajectory[0].curvature_radians_per_meter =
            global_trajectory[1].curvature_radians_per_meter;
        global_trajectory[waypoint_count - 1].curvature_radians_per_meter =
            global_trajectory[waypoint_count - 2].curvature_radians_per_meter;
    }

    global_trajectory_count = (int)waypoint_count;
    g_avg_waypoint_spacing = (waypoint_count > 1) ? (cumulative_s / (double)(waypoint_count - 1)) : 0.05;
    if (g_avg_waypoint_spacing < 0.01) g_avg_waypoint_spacing = 0.01;

    /* Keep closest-index seed across updates; clamp will be applied in the search. */

    g_local_raceline_received = 1;
    g_local_raceline_wait_logged = 0;

    if (!g_local_raceline_speed_warned &&
        missing_speed_count > (size_t)((double)waypoint_count * 0.90))
    {
        printf("[MPC] WARNING: /local_raceline speed missing (z≈0). If using lateral planner, subscribe to /local_raceline (not /local_raceline_viz).\n");
        g_local_raceline_speed_warned = 1;
    }

    if (!g_local_raceline_wall_warned &&
        missing_wall_count > (size_t)((double)waypoint_count * 2.0 * 0.90))
    {
        printf("[MPC] WARNING: /local_raceline wall distances missing (orientation.x/y≈0). Using fallback wall bounds.\n");
        g_local_raceline_wall_warned = 1;
    }

    if (g_verbose)
    {
        printf("[MPC] Local raceline updated: %zu waypoints, length=%.2f m\n",
               waypoint_count, cumulative_s);
    }
}

static void run_mpc_control_cycle(void);

/*===========================================================================
 * ROS2 Callback: Odometry Subscription (velocity feedback)
 *===========================================================================*/

void odometry_subscription_callback(const void *message_in)
{
    if (message_in == NULL) return;

    const nav_msgs__msg__Odometry *odom =
        (const nav_msgs__msg__Odometry *)message_in;

    g_latest_vx = odom->twist.twist.linear.x;
    g_latest_vy = odom->twist.twist.linear.y;
    g_latest_omega = odom->twist.twist.angular.z;

    global_vehicle_state.long_vel = g_latest_vx;
    global_vehicle_state.lat_vel = g_latest_vy;
    global_vehicle_state.yaw_rate = g_latest_omega;

    g_odometry_received = 1;
    g_odom_wait_logged = 0;
}

/*===========================================================================
 * ROS2 Callback: Servo Feedback Subscription
 *===========================================================================*/

/**
 * @brief Process servo feedback and estimate actual steering angle.
 * @param message_in Pointer to std_msgs/Float64 message.
 * @return None.
 */

void servo_feedback_callback(const void *message_in)
{
    if (message_in == NULL) return;

    const std_msgs__msg__Float64 *msg =
        (const std_msgs__msg__Float64 *)message_in;

    /* Convert servo position (0-1) back to steering angle.
     *
     * Forward path (in ackermann_to_vesc):
     *   corrected = c2·|δ|² + c1·|δ| + c0   (sign preserved)
     *   servo_val = gain * corrected + offset
     *
     * Inverse: recover δ from servo_val:
     *   corrected = (servo_val - offset) / gain
     *   solve c2·t² + c1·t + (c0 - |corrected|) = 0 for t
     *   δ = sign(corrected) * t
     */
    double servo_val = msg->data;
    if (STEERING_TO_SERVO_GAIN != 0.0f)
    {
        double corrected = (servo_val - STEERING_TO_SERVO_OFFSET)
                         / STEERING_TO_SERVO_GAIN;
        double abs_corr = fabs(corrected);

        /* Invert the polynomial: t = (-c1 + sqrt(c1² + 4·c2·(abs_corr - c0))) / (2·c2) */
        if (STEERING_CORRECTION_C2 != 0.0f)
        {
            double disc = STEERING_CORRECTION_C1 * STEERING_CORRECTION_C1
                        - 4.0 * STEERING_CORRECTION_C2
                              * (STEERING_CORRECTION_C0 - abs_corr);
            if (disc >= 0.0)
            {
                double t = (-STEERING_CORRECTION_C1 + sqrt(disc))
                         / (2.0 * STEERING_CORRECTION_C2);
                global_actual_steering_angle = copysign(t, corrected);
            }
            else
            {
                /* Fallback: linear inverse (discriminant < 0 should not happen) */
                global_actual_steering_angle = corrected;
            }
        }
        else
        {
            /* No polynomial correction (c2=0) — pure linear */
            global_actual_steering_angle = corrected;
        }
    }

    g_use_steering_feedback = 1;
    clock_gettime(CLOCK_MONOTONIC, &g_last_servo_time);

    if (g_verbose)
    {
        printf("[MPC] Servo feedback: servo_val=%.3f -> delta=%.4f rad\n",
               servo_val, global_actual_steering_angle);
    }
}

/*===========================================================================
 * ROS2 Callback: EKF Pose + MPC Computation
 *===========================================================================
 * Receives the map-frame pose from the EKF and runs the MPC solver.
 * MPC only executes when a new EKF pose message arrives (event-driven).
 * The reference path is in map frame, so Frenet errors are only meaningful
 * when position comes from here rather than from raw wheel odometry.
 * Default topic: /ekf_pose.
 *===========================================================================*/

/**
 * @brief Process map-frame pose updates, run MPC, and publish drive commands.
 * @param message_in Pointer to geometry_msgs/PoseWithCovarianceStamped message.
 * @return None.
 */
void ekf_pose_callback(const void *message_in)
{
    if (message_in == NULL) return;

    const geometry_msgs__msg__PoseWithCovarianceStamped *msg =
        (const geometry_msgs__msg__PoseWithCovarianceStamped *)message_in;

    double pos_x   = msg->pose.pose.position.x;
    double pos_y   = msg->pose.pose.position.y;
    double heading = quaternion_to_yaw_angle(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    g_last_ekf_time = now;

    g_latest_pos_x   = pos_x;
    g_latest_pos_y   = pos_y;
    g_latest_heading = heading;
    g_ekf_state_received = 1;

    if (!g_ekf_pose_received) {
        printf("[MPC] EKF pose received — using map-frame position\n");
        g_ekf_pose_received = 1;
    }

    if (!g_last_control_time_valid)
    {
        g_last_control_time = now;
        g_last_control_time_valid = 1;
        return;
    }

    {
        const MpcConfiguration_t cfg = mpc_get_configuration();
        const double control_dt =
            (cfg.time_step > 0.0f) ? (double)cfg.time_step : (double)TIME_STEP_SECONDS;
        const double elapsed = timespec_diff_sec(&g_last_control_time, &now);
        if (elapsed + 1e-9 < control_dt)
        {
            return;
        }
    }
    g_last_control_time = now;

    run_mpc_control_cycle();
}

static void run_mpc_control_cycle(void)
{
    double pos_x = g_latest_pos_x;
    double pos_y = g_latest_pos_y;
    double heading = g_latest_heading;

    /* Don't run MPC until at least one EKF-derived state sample is available. */
    if (!g_ekf_state_received)
    {
        return;
    }

    if (!g_odometry_received)
    {
        if (!g_odom_wait_logged)
        {
            printf("[MPC] Waiting for odometry on %s before enabling control\n",
                   g_odom_topic);
            g_odom_wait_logged = 1;
        }
        return;
    }

    if (!g_local_raceline_received || global_trajectory_count < 2)
    {
        if (!g_local_raceline_wait_logged)
        {
            printf("[MPC] Waiting for local raceline on %s before enabling control\n",
                   g_local_raceline_topic);
            g_local_raceline_wait_logged = 1;
        }
        return;
    }

    /* Safety watchdog: check if EKF pose stream is stale. */
    {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        double elapsed = timespec_diff_sec(&g_last_ekf_time, &now);

        if (elapsed > g_watchdog_timeout_sec)
        {
            if (g_verbose)
            {
                printf("[MPC] WATCHDOG: EKF pose stale (%.1fms > %.1fms), continuing\n",
                       elapsed * 1000.0, g_watchdog_timeout_sec * 1000.0);
            }
        }
    }

    if (g_verbose)
    {
        printf("[MPC] State: x=%.2f y=%.2f th=%.2f vx=%.2f vy=%.2f w=%.2f\n",
               pos_x, pos_y, heading, g_latest_vx, g_latest_vy, g_latest_omega);
    }

    if (global_trajectory_count > 1)
    {
        int closest = 0;
        double ey = 0.0, epsi = 0.0;
        double vref0 = 0.0, kappa0 = 0.0;
        double left_wall0 = 0.0, right_wall0 = 0.0;

        closest = find_closest_waypoint_local(pos_x, pos_y, heading);
        build_reference_from_local_raceline(closest);

        convert_to_frenet_state(pos_x, pos_y, heading, closest, &global_frenet_state);

        ey = global_frenet_state.flat_error;
        epsi = global_frenet_state.fhead_error;

        /* Startup / recovery shaping: cap v_ref when errors are large to avoid
         * saturating accel/steer and overshooting the local raceline. */
        maybe_apply_recovery_reference_cap(ey, epsi);

        vref0 = global_reference_trajectory[0].reference_velocity;
        kappa0 = global_reference_trajectory[0].path_curvature;
        left_wall0 = global_reference_trajectory[0].left_wall_bound;
        right_wall0 = global_reference_trajectory[0].right_wall_bound;

        if (g_verbose)
        {
            printf("[MPC] Frenet: e_y=%.3f e_psi=%.3f v_ref=%.2f kappa=%.3f\n",
                   ey, epsi, vref0, kappa0);
        }

        /* ===== Run MPC — output used DIRECTLY, no post-processing ===== */
        MpcSolverResult_t mpc_result;
        MpcSolverStatus_t mpc_status;
        struct timespec t0, t1;
        clock_gettime(CLOCK_MONOTONIC, &t0);
        mpc_status = mpc_compute_optimal_control(
            &global_frenet_state,
            global_reference_trajectory,
            &mpc_result);
        clock_gettime(CLOCK_MONOTONIC, &t1);
        double solve_us = (t1.tv_sec - t0.tv_sec) * 1e6 +
                          (t1.tv_nsec - t0.tv_nsec) / 1e3;
        double primal_res = mpc_result.final_cost;
        double dual_res = mpc_result.dual_residual;

        /* Rolling solve-time statistics (always active, lightweight) */
        g_solve_time_sum_us += solve_us;
        if (solve_us > g_solve_time_max_us) g_solve_time_max_us = solve_us;
        g_solve_cycle_count++;
        if (g_solve_cycle_count >= SOLVE_STATS_PRINT_INTERVAL)
        {
            double avg_us = g_solve_time_sum_us / (double)g_solve_cycle_count;
            printf("[MPC] Solve stats (%lu cycles): avg=%.1f us, max=%.1f us (budget=%.0f us)\n",
                   g_solve_cycle_count, avg_us, g_solve_time_max_us,
                 1e6 / CONTROL_RATE_HZ);
            g_solve_time_sum_us = 0.0;
            g_solve_time_max_us = 0.0;
            g_solve_cycle_count = 0;
        }

        if (mpc_status == MPC_STATUS_SUCCESS ||
            mpc_status == MPC_STATUS_MAXIMUM_ITERATIONS_REACHED)
        {
            double steer =
                mpc_result.optimal_control.steer_ang;
            int servo_feedback_fresh = 0;

            if (g_use_steering_feedback)
            {
                struct timespec now;
                clock_gettime(CLOCK_MONOTONIC, &now);
                double servo_age = timespec_diff_sec(&g_last_servo_time, &now);
                if (servo_age <= 0.05)
                    servo_feedback_fresh = 1;
            }

            /* Pass MPC output directly. */
            global_control_command.steer_ang = mpc_result.optimal_control.steer_ang;
            global_control_command.long_acc = mpc_result.optimal_control.long_acc;

            /* Real hardware cannot reverse out of heading errors. A common failure mode
             * is "brake to zero and get stuck" when the MPC decides stopping is the
             * cheapest way to reduce lateral/heading error. Prevent full braking at
             * very low speed so the vehicle keeps creeping forward and can steer back
             * onto the local raceline. */
            if (g_low_speed_brake_inhibit_vx > 0.0)
            {
                const double vx_abs = fabs(g_latest_vx);
                if (isfinite(vx_abs) && vx_abs < g_low_speed_brake_inhibit_vx)
                {
                    if (global_control_command.long_acc < (float)g_low_speed_min_accel)
                        global_control_command.long_acc = (float)g_low_speed_min_accel;
                }
            }

            /* Update servo tracking.
             * If steering feedback is available from VESC, it's already set by
             * the servo callback. Otherwise, simulate servo dynamics with rate limit. */
            if (!servo_feedback_fresh)
            {
                double max_delta = STEERING_RATE_LIMIT * CONTROL_DT_SECONDS;
                double steer_diff = steer - global_actual_steering_angle;
                if (steer_diff > max_delta) steer_diff = max_delta;
                if (steer_diff < -max_delta) steer_diff = -max_delta;
                global_actual_steering_angle += steer_diff;
            }

            /* Feed actual servo position back to MPC */
            {
                ControlInput_t actual_ctrl;
                actual_ctrl.steer_ang = global_actual_steering_angle;
                /* Use the applied longitudinal command (after any safety clamps),
                 * not the raw solver output. This keeps the augmented a_prev
                 * state consistent with the true plant input. */
                actual_ctrl.long_acc = global_control_command.long_acc;
                mpc_set_actual_previous_control(&actual_ctrl);
            }

            if (g_verbose && g_solver_log_file == NULL)
            {
                double accel =
                    mpc_result.optimal_control.long_acc;
                printf("[MPC] Control: steer=%.4f accel=%.2f (status=%d iter=%d pr=%.3e dr=%.3e solve=%.1fus)\n",
                       steer, accel, mpc_status, mpc_result.iterations_used, primal_res, dual_res, solve_us);
            }
            else if (g_solver_log_file != NULL)
            {
                const MpcConfiguration_t cfg = mpc_get_configuration();
                const double pred_dt = (cfg.time_step > 0.0f)
                    ? (double)cfg.time_step : (double)TIME_STEP_SECONDS;
                const double applied_accel = (double)global_control_command.long_acc;
                double cmd_speed = g_latest_vx + (applied_accel * pred_dt);
                if (cmd_speed < (double)VP_MIN_VELOCITY_MPS)
                    cmd_speed = (double)VP_MIN_VELOCITY_MPS;
                if (cmd_speed > (double)TRAJECTORY_MAXIMUM_VELOCITY)
                    cmd_speed = (double)TRAJECTORY_MAXIMUM_VELOCITY;
                /* Published v_cmd (drive.speed) includes the "never faster than raceline"
                 * clamp applied right before publishing. */
                double v_cmd_pub = cmd_speed;
                if (isfinite(vref0) && vref0 > 0.0)
                {
                    if (v_cmd_pub > vref0) v_cmd_pub = vref0;
                }

                struct timespec ts_now;
                clock_gettime(CLOCK_REALTIME, &ts_now);
                long long unix_time_ns =
                    ((long long)ts_now.tv_sec * 1000000000LL) + (long long)ts_now.tv_nsec;

                fprintf(g_solver_log_file,
                        "%lld,%.1f,%d,%d,%.6e,%.6e,%d,"
                        "%.4f,%.4f,%.4f,"
                        "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                        "%.4f,%.4f,%.4f,%.4f,%.4f,"
                        "%.4f,%.4f,%d\n",
                        unix_time_ns,
                        solve_us,
                        (int)mpc_status,
                        (int)mpc_result.iterations_used,
                        primal_res,
                        dual_res,
                        closest,
                        (float)g_latest_pos_x,
                        (float)g_latest_pos_y,
                        (float)g_latest_heading,
                        ey,
                        epsi,
                        g_latest_vx,
                        g_latest_vy,
                        g_latest_omega,
                        vref0,
                        kappa0,
                        left_wall0,
                        right_wall0,
                        mpc_result.optimal_control.steer_ang,
                        global_control_command.long_acc,
                        cmd_speed,
                        v_cmd_pub,
                        global_actual_steering_angle,
                        g_use_steering_feedback);

                g_solver_log_counter++;
                if ((g_solver_log_counter % 20UL) == 0UL)
                {
                    fflush(g_solver_log_file);
                }
            }
        }
        else
        {
            global_control_command.steer_ang = 0.0f;
            global_control_command.long_acc = VP_MIN_ACCEL_MPS2;
            fprintf(stderr,
                    "[MPC] WARNING: Solver status=%d, publishing braking fallback\n",
                    (int)mpc_status);
        }
    }
    else
    {
        global_control_command.steer_ang = 0.0f;
        global_control_command.long_acc = VP_MIN_ACCEL_MPS2;
    }

    /* Publish drive command */
    {
        global_drive_message_buffer.drive.steering_angle =
            global_control_command.steer_ang;

        /* Convert acceleration command to velocity target over one prediction step.
         * This keeps the velocity command consistent with the MPC integration model. */
        {
            double a_cmd =
                global_control_command.long_acc;
            const MpcConfiguration_t cfg = mpc_get_configuration();
            const double pred_dt = (cfg.time_step > 0.0f) ? (double)cfg.time_step : (double)TIME_STEP_SECONDS;

            if (a_cmd > (double)VP_MAX_ACCEL_MPS2) a_cmd = (double)VP_MAX_ACCEL_MPS2;
            if (a_cmd < (double)VP_MIN_ACCEL_MPS2) a_cmd = (double)VP_MIN_ACCEL_MPS2;

            /* Integrate over control time step used by MPC. */
            double v_cmd = g_latest_vx + a_cmd * pred_dt;
            if (v_cmd < (double)VP_MIN_VELOCITY_MPS) v_cmd = (double)VP_MIN_VELOCITY_MPS;
            if (v_cmd > TRAJECTORY_MAXIMUM_VELOCITY) v_cmd = TRAJECTORY_MAXIMUM_VELOCITY;

            /* Hardware is in VEL_TO_ERPM mode by default (see `vesc.yaml`), so the
             * low-level stack primarily obeys `drive.speed`. If we allow `v_cmd`
             * to exceed the reference profile, the car can enter corners too
             * fast even when the MPC outputs braking (because the speed setpoint
             * only decreases gradually by a_cmd*dt). Clamp the published speed
             * to the current reference to enforce "never faster than raceline". */
            if (global_trajectory_count > 1)
            {
                const double v_ref0 = (double)global_reference_trajectory[0].reference_velocity;
                if (isfinite(v_ref0) && v_ref0 > 0.0)
                {
                    if (v_cmd > v_ref0) v_cmd = v_ref0;
                }
            }

            global_drive_message_buffer.drive.speed = (float)v_cmd;
            global_drive_message_buffer.drive.acceleration = (float)a_cmd;
        }

        /* Cache last published values for fallback paths. */
        g_last_cmd_steer = global_drive_message_buffer.drive.steering_angle;
        g_last_cmd_speed = global_drive_message_buffer.drive.speed;
        g_last_cmd_accel = global_drive_message_buffer.drive.acceleration;

        rcl_ret_t pub_rc __attribute__((unused)) =
            rcl_publish(&global_control_publisher, &global_drive_message_buffer, NULL);
    }
}

/*===========================================================================
 * Main Entry Point
 *===========================================================================*/

/**
 * @brief Initialize MPC/ROS2 resources and run the hardware control node.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return Process exit code.
 */
int main(int argc, char *argv[])
{
    printf("============================================================\n");
    printf("  MPC Riccati-ADMM ROS2 Node for F1/10th Hardware\n");
    printf("  Target: Jetson Xavier NX (EKF-driven)\n");
    printf("  8-state augmented Frenet model\n");
    printf("  [e_y, e_psi, vx, vy, omega, delta_actual, drate_prev, accel_prev]\n");
    printf("  Controls: [delta_rate, a_x]\n");
    printf("  Solver: Riccati backward/forward pass inside ADMM loop\n");
    printf("============================================================\n");

    /* Install signal handlers for graceful shutdown */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    rcl_ret_t rc;

    /* Set up real-time scheduling before anything else */
    setup_realtime_scheduling();

    /* Initialize MPC controller — uses Riccati-ADMM internally */
    mpc_initialize();

    /* Runtime parameters from environment */
    {
        /* SECURITY: Environment variables are taken as-is. Caller is
         * responsible for providing valid numeric values; strtof/strtol/atof
         * return zero on invalid input and may silently fall back. */
        const char *env_val;
        if ((env_val = getenv("MPC_ODOM_TOPIC")) != NULL)
            g_odom_topic = env_val;
        if ((env_val = getenv("MPC_DRIVE_TOPIC")) != NULL)
            g_drive_topic = env_val;
        if ((env_val = getenv("MPC_SERVO_TOPIC")) != NULL)
            g_servo_topic = env_val;
        if ((env_val = getenv("MPC_VERBOSE")) != NULL)
            g_verbose = atoi(env_val);
        if ((env_val = getenv("MPC_WATCHDOG_TIMEOUT")) != NULL)
        {
            double timeout = atof(env_val);
            if (timeout > 0.0 && timeout <= 5.0) g_watchdog_timeout_sec = timeout;
        }
        if ((env_val = getenv("MPC_LOW_SPEED_BRAKE_INHIBIT_VX")) != NULL)
        {
            double v = atof(env_val);
            if (v >= 0.0 && v <= 5.0) g_low_speed_brake_inhibit_vx = v;
        }
        if ((env_val = getenv("MPC_LOW_SPEED_MIN_ACCEL")) != NULL)
        {
            double a = atof(env_val);
            if (a >= -VP_MAX_ACCEL_MPS2 && a <= VP_MAX_ACCEL_MPS2) g_low_speed_min_accel = a;
        }
        if ((env_val = getenv("MPC_EKF_TOPIC")) != NULL)
            g_ekf_pose_topic = env_val;
        if ((env_val = getenv("MPC_LOCAL_RACELINE_TOPIC")) != NULL)
            g_local_raceline_topic = env_val;
        if ((env_val = getenv("MPC_USE_LOCAL_RACELINE")) != NULL)
        {
            printf("[MPC] WARNING: MPC_USE_LOCAL_RACELINE ignored (hardware node always uses local raceline)\n");
        }
    }

    {
        /* SECURITY: Environment variable is trusted deployment input.
         * Log path must come from controlled local configuration sources. */
        const char *log_path = getenv("MPC_SOLVER_LOG");
        char default_log_path[256];

        /* Always log every control callback unless code is changed. */
        g_solver_log_stride = 1;

        if (log_path == NULL || log_path[0] == '\0')
        {
            time_t now = time(NULL);
            struct tm tm_now;
            localtime_r(&now, &tm_now);
            strftime(default_log_path, sizeof(default_log_path),
                     "log/mpc_solver_%Y%m%d_%H%M%S.csv", &tm_now);
            log_path = default_log_path;
        }

        ensure_parent_directories(log_path);

        g_solver_log_file = fopen(log_path, "w");
        if (g_solver_log_file == NULL)
        {
            fprintf(stderr, "[MPC] WARNING: Failed to open solver log file %s\n", log_path);
        }
        else
        {
            fprintf(g_solver_log_file,
                    "unix_time_ns,solve_us,status,iterations,primal_residual,dual_residual,closest_wp,"
                    "pose_x,pose_y,yaw,"
                    "e_y,e_psi,vx,vy,omega,v_ref0,kappa0,left_wall0,right_wall0,"
                    "cmd_steer,cmd_accel,cmd_speed,v_cmd,actual_steer,use_steering_feedback\n");
            fflush(g_solver_log_file);
            printf("[MPC] Solver telemetry log: %s (every control callback)\n", log_path);
        }
    }

    {
        MpcConfiguration_t cfg = mpc_get_configuration();
        printf("[MPC] Controller initialized (horizon=%d, dt=%.0fms)\n",
               (int)cfg.prediction_horizon_steps, (double)cfg.time_step * 1000.0);
    }

    printf("[MPC] Control mode: EKF-driven (MPC runs on each /ekf_pose message)\n");
    printf("[MPC] Topics: ekf_pose=%s, drive=%s\n", g_ekf_pose_topic, g_drive_topic);
    printf("[MPC] Servo feedback: %s (gain=%.4f, offset=%.4f)\n",
            g_servo_topic, STEERING_TO_SERVO_GAIN, STEERING_TO_SERVO_OFFSET);
    printf("[MPC] Steering correction: c2=%.6f, c1=%.6f, c0=%.6f\n",
            STEERING_CORRECTION_C2, STEERING_CORRECTION_C1, STEERING_CORRECTION_C0);
    printf("[MPC] Watchdog timeout: %.0f ms\n", g_watchdog_timeout_sec * 1000.0);
    printf("[MPC] EKF pose topic: %s\n", g_ekf_pose_topic);
    printf("[MPC] Odometry topic: %s\n", g_odom_topic);
    printf("[MPC] Local raceline mode: enabled \n");
    printf("[MPC] Local raceline topic: %s\n", g_local_raceline_topic);
    printf("[MPC] Verbose=%d\n", g_verbose);

    {
        MpcConfiguration_t cfg = mpc_get_configuration();
        unsigned int effective_max_iter = cfg.max_solver_iterations;
        double effective_tol = (double)cfg.solver_convergence_tolerance;
        double effective_rho = ADMM_RHO;
        double effective_rho_u = ADMM_RHO_U;
        const char *env_val = getenv("MAX_ITER");
        if (env_val != NULL && env_val[0] != '\0') {
            int parsed = atoi(env_val);
            if (parsed > 0) effective_max_iter = (unsigned int)parsed;
        }
        env_val = getenv("TOL");
        if (env_val != NULL && env_val[0] != '\0') {
            double parsed = atof(env_val);
            if (parsed > 0.0) effective_tol = parsed;
        }
        env_val = getenv("RHO");
        if (env_val != NULL && env_val[0] != '\0') {
            double parsed = atof(env_val);
            if (parsed > 0.0) effective_rho = parsed;
        }
        env_val = getenv("RHO_U");
        if (env_val != NULL && env_val[0] != '\0') {
            double parsed = atof(env_val);
            if (parsed > 0.0) effective_rho_u = parsed;
        }
        printf("[MPC] max_iter=%u, tol=%.6g\n",
               effective_max_iter,
               effective_tol);
        printf("[MPC] rho=%.6g, rho_u=%.6g\n",
               effective_rho,
               effective_rho_u);
        printf("[MPC] Weights: lat=%.1f heading=%.1f vel=%.1f steer_rate=%.2f steer_effort=%.4f\n",
               cfg.weight_lateral_error,
               cfg.weight_heading_error,
               cfg.weight_velocity,
               cfg.weight_steering_rate,
               cfg.weight_steering_effort);
    }

    printf("[MPC] Reference source: %s (nav_msgs/Path)\n", g_local_raceline_topic);
    printf("[MPC] Waiting for first local raceline message before running control\n");

    /* Initialize ROS2 */
    rcl_context_t ctx = rcl_get_zero_initialized_context();
    rcl_init_options_t init_opts = rcl_get_zero_initialized_init_options();

    rc = rcl_init_options_init(&init_opts, rcl_get_default_allocator());
    if (rc != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: init_options: %s\n", rcl_get_error_string().str);
        return 1;
    }

    rc = rcl_init(argc, (const char *const *)argv, &init_opts, &ctx);
    if (rc != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: rcl_init: %s\n", rcl_get_error_string().str);
        return 1;
    }
    printf("[ROS2] Context initialized\n");
    global_ros2_context = &ctx;

    /* Create node */
    rcl_node_t node = rcl_get_zero_initialized_node();
    rcl_node_options_t node_opts = rcl_node_get_default_options();

    rc = rcl_node_init(&node, "mpc_hardware_node", "", &ctx, &node_opts);
    if (rc != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: node_init: %s\n", rcl_get_error_string().str);
        return 1;
    }
    printf("[ROS2] Node 'mpc_hardware_node' created\n");

    /* QoS: Reliable, KeepLast(10) — matches VESC driver QoS(10) */
    rmw_qos_profile_t qos_reliable_10 = rmw_qos_profile_default;
    qos_reliable_10.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    qos_reliable_10.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    qos_reliable_10.depth = 10;

    /* ===== Subscription 1: Odometry (nav_msgs/Odometry) ===== */
    rcl_subscription_t odom_sub = rcl_get_zero_initialized_subscription();
    rcl_subscription_options_t odom_sub_opts = rcl_subscription_get_default_options();
    odom_sub_opts.qos = qos_reliable_10;

    rc = rcl_subscription_init(&odom_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        g_odom_topic, &odom_sub_opts);
    if (rc != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: odom subscription: %s\n", rcl_get_error_string().str);
        return 1;
    }
    printf("[ROS2] Subscribed to %s (Reliable, KeepLast(10))\n", g_odom_topic);

    /* ===== Subscription 2: /sensors/servo_position_command (std_msgs/Float64) ===== */
    rcl_subscription_t servo_sub = rcl_get_zero_initialized_subscription();
    rcl_subscription_options_t servo_sub_opts = rcl_subscription_get_default_options();
    servo_sub_opts.qos = qos_reliable_10;

    rc = rcl_subscription_init(&servo_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
        g_servo_topic, &servo_sub_opts);
    if (rc != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: servo subscription: %s\n", rcl_get_error_string().str);
        return 1;
    }
    printf("[ROS2] Subscribed to %s (Reliable, KeepLast(10))\n", g_servo_topic);

    /* ===== Subscription 3: EKF pose (geometry_msgs/PoseWithCovarianceStamped) ===== */
    rmw_qos_profile_t qos_ekf_pose = rmw_qos_profile_default;
    qos_ekf_pose.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    qos_ekf_pose.history     = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    qos_ekf_pose.depth       = 10;

    rcl_subscription_t ekf_pose_sub = rcl_get_zero_initialized_subscription();
    rcl_subscription_options_t ekf_pose_sub_opts = rcl_subscription_get_default_options();
    ekf_pose_sub_opts.qos = qos_ekf_pose;

    rc = rcl_subscription_init(&ekf_pose_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseWithCovarianceStamped),
        g_ekf_pose_topic, &ekf_pose_sub_opts);
    if (rc != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: ekf_pose subscription: %s\n", rcl_get_error_string().str);
        return 1;
    }
    printf("[ROS2] Subscribed to %s (Reliable, KeepLast(10))\n", g_ekf_pose_topic);

    /* ===== Subscription 4: Local raceline path (nav_msgs/Path) ===== */
    rcl_subscription_t local_raceline_sub = rcl_get_zero_initialized_subscription();
    rcl_subscription_options_t local_raceline_sub_opts = rcl_subscription_get_default_options();
    local_raceline_sub_opts.qos = qos_reliable_10;

    rc = rcl_subscription_init(&local_raceline_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Path),
        g_local_raceline_topic, &local_raceline_sub_opts);
    if (rc != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: local raceline subscription: %s\n", rcl_get_error_string().str);
        return 1;
    }
    printf("[ROS2] Subscribed to %s (Reliable, KeepLast(10))\n", g_local_raceline_topic);

    /* ===== Publisher: /drive (ackermann_msgs/AckermannDriveStamped) ===== */
    rcl_publisher_options_t pub_opts = rcl_publisher_get_default_options();
    pub_opts.qos = qos_reliable_10;

    global_control_publisher = rcl_get_zero_initialized_publisher();
    rc = rcl_publisher_init(&global_control_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(ackermann_msgs, msg, AckermannDriveStamped),
        g_drive_topic, &pub_opts);
    if (rc != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: drive publisher: %s\n", rcl_get_error_string().str);
        return 1;
    }
    printf("[ROS2] Publishing to %s (Reliable, KeepLast(10))\n", g_drive_topic);

    /* Initialize message buffers (pre-allocate all strings) */
    nav_msgs__msg__Odometry__init(&global_odometry_message_buffer);
    if (!preallocate_rosidl_string(&global_odometry_message_buffer.header.frame_id, 64) ||
        !preallocate_rosidl_string(&global_odometry_message_buffer.child_frame_id, 64))
    {
        fprintf(stderr, "[ROS2] ERROR: odom string alloc\n");
        return 1;
    }
    std_msgs__msg__Float64__init(&global_servo_message_buffer);
    geometry_msgs__msg__PoseWithCovarianceStamped__init(&global_ekf_pose_buffer);
    nav_msgs__msg__Path__init(&global_local_raceline_buffer);

    ackermann_msgs__msg__AckermannDriveStamped__init(&global_drive_message_buffer);
    if (!preallocate_rosidl_string(&global_drive_message_buffer.header.frame_id, 64))
    {
        fprintf(stderr, "[ROS2] ERROR: drive header string alloc\n");
        return 1;
    }
    set_rosidl_string(&global_drive_message_buffer.header.frame_id, "base_link");

    /* Executor: 4 subscriptions (odom, servo, ekf_pose, local_raceline) */
    rcl_allocator_t alloc = rcl_get_default_allocator();
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();

    rc = rclc_executor_init(&executor, &ctx, EXECUTOR_NUM_HANDLES, &alloc);
    if (rc != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: executor init: %s\n", rcl_get_error_string().str);
        return 1;
    }

    /* Add odometry subscription */
    rc = rclc_executor_add_subscription(&executor, &odom_sub,
        &global_odometry_message_buffer, &odometry_subscription_callback, ON_NEW_DATA);
    if (rc != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: add odom sub: %s\n", rcl_get_error_string().str);
        return 1;
    }

    /* Add servo feedback subscription */
    rc = rclc_executor_add_subscription(&executor, &servo_sub,
        &global_servo_message_buffer, &servo_feedback_callback, ON_NEW_DATA);
    if (rc != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: add servo sub: %s\n", rcl_get_error_string().str);
        return 1;
    }

    /* Add EKF pose subscription — this drives MPC execution */
    rc = rclc_executor_add_subscription(&executor, &ekf_pose_sub,
        &global_ekf_pose_buffer, &ekf_pose_callback, ON_NEW_DATA);
    if (rc != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: add ekf_pose sub failed — MPC cannot run without EKF pose\n");
        return 1;
    }

    /* Add local raceline subscription */
    rc = rclc_executor_add_subscription(&executor, &local_raceline_sub,
        &global_local_raceline_buffer, &local_raceline_callback, ON_NEW_DATA);
    if (rc != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: add local raceline sub failed\n");
        return 1;
    }

    printf("[ROS2] Executor ready (4 subs, MPC driven by %s)\n", g_ekf_pose_topic);
        printf("\n[MPC] Spinning... (waiting for EKF pose on %s, odom on %s, and local raceline on %s)\n\n",
            g_ekf_pose_topic, g_odom_topic, g_local_raceline_topic);

    rclc_executor_spin(&executor);

    /* Cleanup */
    printf("\n[ROS2] Shutting down...\n");
    if (g_solver_log_file != NULL)
    {
        fflush(g_solver_log_file);
        fclose(g_solver_log_file);
        g_solver_log_file = NULL;
    }

    nav_msgs__msg__Odometry__fini(&global_odometry_message_buffer);
    std_msgs__msg__Float64__fini(&global_servo_message_buffer);
    ackermann_msgs__msg__AckermannDriveStamped__fini(&global_drive_message_buffer);
    nav_msgs__msg__Path__fini(&global_local_raceline_buffer);
    geometry_msgs__msg__PoseWithCovarianceStamped__fini(&global_ekf_pose_buffer);

    rcl_ret_t cleanup_rc;
    cleanup_rc = rcl_subscription_fini(&odom_sub, &node); (void)cleanup_rc;
    cleanup_rc = rcl_subscription_fini(&servo_sub, &node); (void)cleanup_rc;
    cleanup_rc = rcl_subscription_fini(&ekf_pose_sub, &node); (void)cleanup_rc;
    cleanup_rc = rcl_subscription_fini(&local_raceline_sub, &node); (void)cleanup_rc;
    cleanup_rc = rcl_publisher_fini(&global_control_publisher, &node); (void)cleanup_rc;
    cleanup_rc = rcl_node_fini(&node); (void)cleanup_rc;
    cleanup_rc = rcl_context_fini(&ctx); (void)cleanup_rc;

    printf("[ROS2] Cleanup complete\n");
    return 0;
}
