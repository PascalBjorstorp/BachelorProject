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
 *   - Odometry callback: stores latest velocity state (fast, non-blocking)
 *   - Servo feedback callback: stores actual steering angle from VESC
 *   - EKF pose callback: caches latest pose and signals MPC thread
 *   - MPC control thread: runs once per newest EKF pose, updates latest command
 *   - Drive publisher thread: republishes latest MPC command at fixed rate
 *   - Safety: no command published until both odom and EKF pose are received
 *
 * Topics:
 *   Subscribe: /ego_racecar/odom       (nav_msgs/Odometry)     — VESC odometry [QoS(10)]
 *   Subscribe: /sensors/servo_position_command (std_msgs/Float64) — servo fb [QoS(10)]
 *   Subscribe: /ekf_pose               (geometry_msgs/PoseWithCovarianceStamped) — EKF pose [QoS(10)]
 *   Subscribe: /local_raceline         (nav_msgs/Path)         — lateral planner reference [QoS(10)]
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
#include <pthread.h>
/* ROS2 C Client Library Headers */
#include "rcl/rcl.h"
#include "rcl/error_handling.h"
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
static const char *g_odom_topic = "/ego_racecar/odom";
static const char *g_drive_topic = "/drive";
static const char *g_servo_topic = "/sensors/servo_position_command";
static const char *g_ekf_pose_topic = "/ekf_pose";
static const char *g_local_raceline_topic = "/local_raceline";
static int g_local_raceline_received = 0;
static int g_local_raceline_wait_logged = 0;
static int g_local_raceline_wall_warned = 0;
static const double FALLBACK_WALL_BOUND_M = 1.5;

/** Published speed is capped to raceline speed plus this margin [m/s]. */
static double g_raceline_speed_margin_mps = 0.5;

/** Enable verbose logging (disabled by default for real-time performance) */
static int g_verbose = 0;

/** Set to 1 once the first EKF pose message has been received. */
static int g_ekf_pose_received = 0;

/** Safety watchdog timeout [seconds] */
static double g_watchdog_timeout_sec = 0.2;
static double g_drive_republish_period_ms = 5.0;
static int g_drive_command_ready = 0;
/* Last published drive command (fallback uses these instead of forcing stop). */
static float g_last_cmd_steer = 0.0f;
static float g_last_cmd_speed = 0.0f;
static float g_last_cmd_accel = 0.0f;

/* Standstill brake-override fallback:
 * If the car is essentially stopped but MPC commands full braking, override
 * acceleration to gently pull forward (keeps MPC steering). */
static int g_standstill_brake_override_active = 0;

static double apply_standstill_brake_override(double vx, double accel_cmd, int allow_log)
{
    const double standstill_vx_thresh = 0.15; /* m/s */
    const double accel_override = 0.5;        /* m/s^2 */
    const double brake_eps = 0.05;            /* m/s^2 */

    const int standstill = isfinite(vx) && fabs(vx) < standstill_vx_thresh;
    const int max_brake_cmd = isfinite(accel_cmd) && (accel_cmd <= ((double)VP_MIN_ACCEL_MPS2 + brake_eps));

    if (standstill && max_brake_cmd) {
        if (!g_standstill_brake_override_active && allow_log && g_verbose) {
            printf("[MPC] Standstill brake override active: vx=%.3f a_cmd=%.2f -> %.2f\n",
                   vx, accel_cmd, accel_override);
        }
        g_standstill_brake_override_active = 1;
        return accel_override;
    }

    if (g_standstill_brake_override_active) {
        if (allow_log && g_verbose) {
            printf("[MPC] Standstill brake override cleared: vx=%.3f a_cmd=%.2f\n",
                   vx, accel_cmd);
        }
        g_standstill_brake_override_active = 0;
    }
    return accel_cmd;
}

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
static int global_odometry_received_flag = 0;
static volatile sig_atomic_t global_shutdown_requested = 0;
static rcl_context_t *global_ros2_context = NULL;

static rcl_publisher_t global_control_publisher;
static rcl_publisher_t g_timing_solve_us_publisher;
static rcl_publisher_t g_timing_control_gap_ms_publisher;
static rcl_publisher_t g_timing_ekf_to_control_ms_publisher;
static rcl_publisher_t g_timing_output_gap_ms_publisher;
static rcl_publisher_t g_timing_drive_age_ms_publisher;
static rcl_publisher_t g_timing_pose_seq_publisher;
static rcl_publisher_t g_timing_skipped_poses_publisher;
static rcl_publisher_t g_timing_solver_enter_seq_publisher;
static int g_timing_publishers_ready = 0;
static pthread_mutex_t g_timing_publish_mutex = PTHREAD_MUTEX_INITIALIZER;

static nav_msgs__msg__Odometry global_odometry_message_buffer;
static std_msgs__msg__Float64 global_servo_message_buffer;
static ackermann_msgs__msg__AckermannDriveStamped global_drive_message_buffer;
static geometry_msgs__msg__PoseWithCovarianceStamped global_ekf_pose_buffer;
static nav_msgs__msg__Path global_local_raceline_buffer;

static TrajectoryReferencePoint_t global_reference_trajectory[PREDICTION_HORIZON];

/** Cached latest odometry velocity values. */
static double g_latest_vx = 0.0;
static double g_latest_vy = 0.0;
static double g_latest_omega = 0.0;

/** Watchdog: timestamp of last odometry received (CLOCK_MONOTONIC — uses VDSO
 *  fast path on aarch64, ~3ns vs ~50ns syscall for CLOCK_MONOTONIC_RAW) */
static struct timespec g_last_odom_time = {0, 0};

/** Rolling solve-time and iteration instrumentation (always active) */
static double g_solve_time_sum_us = 0.0;
static double g_solve_time_min_us = 1e9;
static double g_solve_time_max_us = 0.0;
static uint16_t g_iter_count_min = 0xFFFF;
static uint16_t g_iter_count_max = 0;
static double g_iter_count_sum = 0.0;
static unsigned long g_stats_cycle_count = 0;
static unsigned long g_stats_optimal_count = 0;
static unsigned long g_stats_max_iter_count = 0;
static struct timespec g_last_terminal_print_time = {0, 0};
#define TERMINAL_STATS_PRINT_INTERVAL_SECONDS 5.0

/** Optional solver telemetry logging for post-drive analysis. */
static FILE *g_solver_log_file = NULL;
static FILE *g_solver_meta_file = NULL;
static FILE *g_local_raceline_log_file = NULL;
static FILE *g_stats_csv_file = NULL;
static unsigned long g_solver_log_counter = 0;
static int g_solver_log_stride = 1;
static int g_solver_csv_logging_enabled = 0;
static int g_log_local_raceline_snapshots = 0;
static double g_last_servo_raw = 0.0;
static long long g_last_odom_ros_time_ns = 0;
static unsigned long g_local_raceline_update_seq = 0;
static long long g_local_raceline_ros_time_ns = 0;
static struct timespec g_last_mpc_output_time = {0, 0};
static int g_last_mpc_output_valid = 0;
static unsigned long g_mpc_output_seq = 0;
static struct timespec g_last_control_start_time = {0, 0};
static int g_last_control_start_valid = 0;

typedef struct
{
    int32_t stamp_sec;
    uint32_t stamp_nanosec;
    struct timespec received_mono_time;
    double pos_x;
    double pos_y;
    double qx;
    double qy;
    double qz;
    double qw;
    double covariance[36];
} PoseSnapshot_t;

static PoseSnapshot_t g_latest_pose_snapshot;
static unsigned long g_latest_pose_seq = 0;
static int g_latest_pose_valid = 0;
static pthread_mutex_t g_pose_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_pose_cond = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t g_drive_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t g_state_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t g_trajectory_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_t g_control_thread;
static pthread_t g_drive_thread;
static int g_control_thread_started = 0;
static int g_drive_thread_started = 0;

typedef struct
{
    double path_x_m;
    double path_y_m;
    double path_heading_rad;
    double path_s_m;
    double segment_t;
    int segment_index;
} PathProjection_t;

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

static double timespec_diff_sec(const struct timespec *a, const struct timespec *b);

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

/**
 * @brief Print a minimal fatal-signal reason before the process disappears.
 */
static void fatal_signal_handler(int sig)
{
    const char *name = "UNKNOWN";
    size_t name_len = 7U;
    ssize_t ignored;

    switch (sig)
    {
        case SIGSEGV: name = "SIGSEGV"; name_len = 7U; break;
        case SIGABRT: name = "SIGABRT"; name_len = 7U; break;
        case SIGFPE:  name = "SIGFPE";  name_len = 6U; break;
        case SIGILL:  name = "SIGILL";  name_len = 6U; break;
        case SIGBUS:  name = "SIGBUS";  name_len = 6U; break;
        default: break;
    }

    ignored = write(STDERR_FILENO,
                    "\n[MPC] FATAL: mpc_hardware_node received ",
                    sizeof("\n[MPC] FATAL: mpc_hardware_node received ") - 1U);
    (void)ignored;
    ignored = write(STDERR_FILENO, name, name_len);
    (void)ignored;
    ignored = write(STDERR_FILENO,
                    " and is exiting\n",
                    sizeof(" and is exiting\n") - 1U);
    (void)ignored;
    _Exit(128 + sig);
}

/**
 * @brief Publish /drive and log if ROS refuses the publish call.
 */
static void publish_drive_command_or_warn(const char *context)
{
    static unsigned long publish_error_count = 0;
    rcl_ret_t pub_rc =
        rcl_publish(&global_control_publisher, &global_drive_message_buffer, NULL);

    if (pub_rc != RCL_RET_OK)
    {
        publish_error_count++;
        if (publish_error_count <= 5UL || (publish_error_count % 20UL) == 0UL)
        {
            fprintf(stderr,
                    "[ROS2] ERROR: /drive publish failed in %s: rc=%d, error=%s\n",
                    context,
                    (int)pub_rc,
                    rcl_get_error_string().str);
        }
        rcl_reset_error();
    }
}

static void publish_float64_metric(rcl_publisher_t *publisher, double value, const char *context)
{
    static unsigned long metric_publish_error_count = 0;
    if (!g_timing_publishers_ready || publisher == NULL)
    {
        return;
    }

    std_msgs__msg__Float64 msg;
    msg.data = value;

    pthread_mutex_lock(&g_timing_publish_mutex);
    rcl_ret_t pub_rc = rcl_publish(publisher, &msg, NULL);
    pthread_mutex_unlock(&g_timing_publish_mutex);

    if (pub_rc != RCL_RET_OK)
    {
        metric_publish_error_count++;
        if (metric_publish_error_count <= 5UL ||
            (metric_publish_error_count % 100UL) == 0UL)
        {
            fprintf(stderr,
                    "[ROS2] WARNING: timing metric publish failed in %s: rc=%d, error=%s\n",
                    context,
                    (int)pub_rc,
                    rcl_get_error_string().str);
        }
        rcl_reset_error();
    }
}

static int init_float64_publisher(
    rcl_publisher_t *publisher,
    rcl_node_t *node,
    const char *topic,
    const rcl_publisher_options_t *options)
{
    if (publisher == NULL || node == NULL || topic == NULL || options == NULL)
    {
        return 0;
    }

    *publisher = rcl_get_zero_initialized_publisher();
    rcl_ret_t rc = rcl_publisher_init(
        publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
        topic,
        options);

    if (rc != RCL_RET_OK)
    {
        fprintf(stderr,
                "[ROS2] ERROR: timing publisher %s: %s\n",
                topic,
                rcl_get_error_string().str);
        rcl_reset_error();
        return 0;
    }

    printf("[ROS2] Publishing timing metric %s (BestEffort, KeepLast(1))\n", topic);
    return 1;
}

static void *drive_publisher_thread_main(void *arg)
{
    (void)arg;

    const long period_ns = (long)(g_drive_republish_period_ms * 1000000.0);
    struct timespec sleep_time;
    sleep_time.tv_sec = period_ns / 1000000000L;
    sleep_time.tv_nsec = period_ns % 1000000000L;

    while (!global_shutdown_requested)
    {
        nanosleep(&sleep_time, NULL);

        double drive_age_ms = 0.0;
        int publish_drive_age = 0;

        pthread_mutex_lock(&g_drive_mutex);
        if (g_drive_command_ready)
        {
            struct timespec now_rt;
            clock_gettime(CLOCK_REALTIME, &now_rt);
            global_drive_message_buffer.header.stamp.sec = (int32_t)now_rt.tv_sec;
            global_drive_message_buffer.header.stamp.nanosec = (uint32_t)now_rt.tv_nsec;

            if (g_last_mpc_output_valid)
            {
                struct timespec now_mono;
                clock_gettime(CLOCK_MONOTONIC, &now_mono);
                drive_age_ms = timespec_diff_sec(&g_last_mpc_output_time, &now_mono) * 1000.0;
                publish_drive_age = 1;
            }
            publish_drive_command_or_warn("drive_publisher_thread");
        }
        pthread_mutex_unlock(&g_drive_mutex);

        if (publish_drive_age)
        {
            publish_float64_metric(&g_timing_drive_age_ms_publisher,
                                   drive_age_ms,
                                   "drive_age_ms");
        }
    }

    return NULL;
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
 * Local Raceline Reference
 *===========================================================================*/

/** Length of the latest local raceline segment. */
static double g_local_raceline_length_meters = 0.0;

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

/**
 * @brief Build MPC reference directly from latest local raceline topic points.
 * Computes arc-length lookahead using reference velocity (not actual velocity)
 * so the horizon is never "blind" when the car momentarily stops.
 * @return None.
 */
static void build_reference_from_local_raceline(void)
{
    if (global_trajectory_count <= 0)
    {
        return;
    }

    /* Get MPC parameters for time step */
    const MpcConfiguration_t cfg = mpc_get_configuration();
    const double dt = (cfg.time_step > 0.0f) ? (double)cfg.time_step : (double)TIME_STEP_SECONDS;
    
    /* Use v_ref at current position (step 0) to compute lookahead arc-length.
     * This ensures the horizon is always computed from a valid reference speed,
     * not from actual velocity which may momentarily be zero. */
    double v_ref_base = (double)global_trajectory[0].velocity_meters_per_second;
    if (v_ref_base <= 0.0) v_ref_base = MIN_TRAJECTORY_SPEED_MPS;
    
    /* For each horizon step, compute the arc-length position and fill reference trajectory.
     * Interpolate reference fields along arc length so the CPU node can be
     * tested against the smoother horizon construction used elsewhere. */
    for (int step = 0; step < PREDICTION_HORIZON; step++)
    {
        double target_s = v_ref_base * dt * (double)step;
        int idx = global_trajectory_count - 1;
        for (int i = 0; i < global_trajectory_count - 1; i++)
        {
            if (global_trajectory[i].s_meters <= target_s &&
                global_trajectory[i + 1].s_meters > target_s)
            {
                idx = i;
                break;
            }
        }

        const TrajectoryWaypoint_t *a = &global_trajectory[idx];
        const TrajectoryWaypoint_t *b = a;
        double t = 0.0;
        if (idx + 1 < global_trajectory_count)
        {
            b = &global_trajectory[idx + 1];
            const double ds = b->s_meters - a->s_meters;
            if (ds > 1e-9)
            {
                t = (target_s - a->s_meters) / ds;
                if (t < 0.0) t = 0.0;
                if (t > 1.0) t = 1.0;
            }
        }

        const double traj_vel =
            a->velocity_meters_per_second +
            t * (b->velocity_meters_per_second - a->velocity_meters_per_second);
        const double traj_kappa =
            a->curvature_radians_per_meter +
            t * (b->curvature_radians_per_meter - a->curvature_radians_per_meter);
        const double traj_left_bound =
            a->left_bound_meters +
            t * (b->left_bound_meters - a->left_bound_meters);
        const double traj_right_bound =
            a->right_bound_meters +
            t * (b->right_bound_meters - a->right_bound_meters);

        global_reference_trajectory[step].reference_lateral_error = 0;
        global_reference_trajectory[step].reference_heading_error = 0;
        global_reference_trajectory[step].path_curvature = traj_kappa;
        global_reference_trajectory[step].left_wall_bound = traj_left_bound;
        global_reference_trajectory[step].right_wall_bound = traj_right_bound;
        global_reference_trajectory[step].reference_velocity = traj_vel;
        global_reference_trajectory[step].reference_lateral_velocity = 0;
        global_reference_trajectory[step].reference_yaw_rate =
            traj_kappa * traj_vel;
    }
}

/*===========================================================================
 * Helper Functions
 *===========================================================================*/

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
static double timespec_diff_sec(const struct timespec *a, const struct timespec *b){
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
static void project_to_path_segment(
    double car_x, double car_y, double car_heading,
    int closest_index,
    PathProjection_t *projection_out,
    FrenetState_t *frenet_out)
{
    int idx0 = closest_index;
    if (global_trajectory_count < 2)
    {
        return;
    }
    if (idx0 < 0) idx0 = 0;
    if (idx0 >= global_trajectory_count - 1)
    {
        idx0 = global_trajectory_count - 2;
    }
    int idx1 = idx0 + 1;

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

    /* Signed lateral error (positive = left of path). */
    double sin_h = sin(path_heading);
    double cos_h = cos(path_heading);
    double dx = car_x - path_x;
    double dy = car_y - path_y;
    double lateral_error = -dx * sin_h + dy * cos_h;

    double heading_error = car_heading - path_heading;
    while (heading_error > M_PI) heading_error -= TWO_PI;
    while (heading_error < -M_PI) heading_error += TWO_PI;

    if (projection_out != NULL)
    {
        projection_out->path_x_m = path_x;
        projection_out->path_y_m = path_y;
        projection_out->path_heading_rad = path_heading;
        projection_out->path_s_m =
            global_trajectory[idx0].s_meters +
            t * (global_trajectory[idx1].s_meters - global_trajectory[idx0].s_meters);
        projection_out->segment_t = t;
        projection_out->segment_index = idx0;
    }

    if (frenet_out != NULL)
    {
        frenet_out->flat_error = lateral_error;
        frenet_out->fhead_error = heading_error;
        frenet_out->flong_vel =
            global_vehicle_state.long_vel;
        frenet_out->flat_vel =
            global_vehicle_state.lat_vel;
        frenet_out->fyaw_rate =
            global_vehicle_state.yaw_rate;
    }
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
    const long long local_raceline_ros_time_ns =
        (long long)msg->header.stamp.sec * 1000000000LL +
        (long long)msg->header.stamp.nanosec;

    if (msg->poses.size < 2)
    {
        return;
    }

    size_t waypoint_count = msg->poses.size;
    if (waypoint_count > TRAJECTORY_MAXIMUM_WAYPOINTS)
    {
        waypoint_count = TRAJECTORY_MAXIMUM_WAYPOINTS;
    }

    pthread_mutex_lock(&g_trajectory_mutex);

    double cumulative_s = 0.0;
    double prev_x = 0.0;
    double prev_y = 0.0;
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
        wp->left_bound_meters = left_bound;
        wp->right_bound_meters = right_bound;
        wp->heading_radians = 0.0;
        wp->curvature_radians_per_meter = 0.0;

        prev_x = x;
        prev_y = y;
    }

    for (size_t i = 0; i < waypoint_count; i++)
    {
        size_t i_prev = (i == 0) ? 0 : (i - 1);
        size_t i_next = (i + 1 < waypoint_count) ? (i + 1) : (waypoint_count - 1);

        const double dx = global_trajectory[i_next].x_meters - global_trajectory[i_prev].x_meters;
        const double dy = global_trajectory[i_next].y_meters - global_trajectory[i_prev].y_meters;

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
        for (size_t i = 1; i + 1 < waypoint_count; i++)
        {
            const double dpsi = wrap_angle_pi(
                global_trajectory[i + 1].heading_radians -
                global_trajectory[i - 1].heading_radians);
            const double ds = global_trajectory[i + 1].s_meters - global_trajectory[i - 1].s_meters;
            const double ds_safe = (ds > 1e-6) ? ds : 1e-6;
            double kappa = dpsi / ds_safe;
            // if (kappa > kappa_max) kappa = kappa_max;
            // if (kappa < -kappa_max) kappa = -kappa_max;
            global_trajectory[i].curvature_radians_per_meter = kappa;
        }

        global_trajectory[0].curvature_radians_per_meter =
            global_trajectory[1].curvature_radians_per_meter;
        global_trajectory[waypoint_count - 1].curvature_radians_per_meter =
            global_trajectory[waypoint_count - 2].curvature_radians_per_meter;
    }

    global_trajectory_count = (int)waypoint_count;
    g_local_raceline_length_meters =
        (waypoint_count > 1)
        ? (global_trajectory[waypoint_count - 1].s_meters - global_trajectory[0].s_meters)
        : 0.0;
    g_local_raceline_ros_time_ns = local_raceline_ros_time_ns;
    g_local_raceline_update_seq++;

    g_local_raceline_received = 1;
    g_local_raceline_wait_logged = 0;

    if (g_local_raceline_log_file != NULL)
    {
        for (size_t i = 0; i < waypoint_count; i++)
        {
            const TrajectoryWaypoint_t *wp = &global_trajectory[i];
            fprintf(g_local_raceline_log_file,
                    "%lu,%lld,%zu,%zu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                    g_local_raceline_update_seq,
                    g_local_raceline_ros_time_ns,
                    i,
                    waypoint_count,
                    wp->s_meters,
                    wp->x_meters,
                    wp->y_meters,
                    wp->heading_radians,
                    wp->curvature_radians_per_meter,
                    wp->velocity_meters_per_second,
                    wp->left_bound_meters,
                    wp->right_bound_meters);
        }
        fflush(g_local_raceline_log_file);
    }

    if (!g_local_raceline_wall_warned &&
        missing_wall_count > (size_t)((double)waypoint_count * 2.0 * 0.90))
    {
        printf("[MPC] WARNING: /local_raceline wall distances missing (orientation.x/y). Using fallback wall bounds.\n");
        g_local_raceline_wall_warned = 1;
    }

    if (g_verbose)
    {
        printf("[MPC] Local raceline updated: %zu waypoints, length=%.2f m\n",
               waypoint_count, g_local_raceline_length_meters);
    }

    pthread_mutex_unlock(&g_trajectory_mutex);
}

/*===========================================================================
 * ROS2 Callback: Odometry Subscription (non-blocking, just stores state)
 *===========================================================================*/

/**
 * @brief Process odometry messages and update cached vehicle dynamics.
 * @param message_in Pointer to nav_msgs/Odometry message.
 * @return None.
 */

void odometry_subscription_callback(const void *message_in)
{
    if (message_in == NULL) return;

    const nav_msgs__msg__Odometry *odom =
        (const nav_msgs__msg__Odometry *)message_in;

    double vx = odom->twist.twist.linear.x;
    double vy = odom->twist.twist.linear.y;
    double omega = odom->twist.twist.angular.z;

    pthread_mutex_lock(&g_state_mutex);
    /* Store only the velocity state from odometry. EKF pose drives position,
     * heading, and the control callback. */
    global_vehicle_state.long_vel = vx;
    global_vehicle_state.lat_vel = vy;
    global_vehicle_state.yaw_rate = omega;

    g_latest_vx = vx;
    g_latest_vy = vy;
    g_latest_omega = omega;
    g_last_odom_ros_time_ns =
        (long long)odom->header.stamp.sec * 1000000000LL +
        (long long)odom->header.stamp.nanosec;

    /* Update watchdog timestamp */
    clock_gettime(CLOCK_MONOTONIC, &g_last_odom_time);

    global_odometry_received_flag = 1;
    pthread_mutex_unlock(&g_state_mutex);
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
    pthread_mutex_lock(&g_state_mutex);
    g_last_servo_raw = servo_val;
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
    const double actual_steer = global_actual_steering_angle;
    pthread_mutex_unlock(&g_state_mutex);

    if (g_verbose)
    {
        printf("[MPC] Servo feedback: servo_val=%.3f -> delta=%.4f rad\n",
               servo_val, actual_steer);
    }
}

/*===========================================================================
 * MPC Control Step
 *===========================================================================
 * Runs once for one copied EKF pose. EKF callback never calls this directly;
 * the control thread does, after being signaled that a new pose arrived.
 *===========================================================================*/
static void run_mpc_for_pose(
    const PoseSnapshot_t *pose,
    unsigned long pose_seq,
    unsigned long skipped_poses)
{
    if (pose == NULL) return;

    struct timespec control_start_time;
    clock_gettime(CLOCK_MONOTONIC, &control_start_time);

    double ekf_to_control_ms =
        timespec_diff_sec(&pose->received_mono_time, &control_start_time) * 1000.0;
    double control_gap_ms = 0.0;
    if (g_last_control_start_valid)
    {
        control_gap_ms =
            timespec_diff_sec(&g_last_control_start_time, &control_start_time) * 1000.0;
    }
    g_last_control_start_time = control_start_time;
    g_last_control_start_valid = 1;

    publish_float64_metric(&g_timing_pose_seq_publisher,
                           (double)pose_seq,
                           "pose_seq");
    publish_float64_metric(&g_timing_skipped_poses_publisher,
                           (double)skipped_poses,
                           "skipped_poses");
    publish_float64_metric(&g_timing_ekf_to_control_ms_publisher,
                           ekf_to_control_ms,
                           "ekf_to_control_ms");
    publish_float64_metric(&g_timing_control_gap_ms_publisher,
                           control_gap_ms,
                           "control_gap_ms");

    const int32_t drive_stamp_sec = pose->stamp_sec;
    const uint32_t drive_stamp_nanosec = pose->stamp_nanosec;
    double pos_x   = pose->pos_x;
    double pos_y   = pose->pos_y;
    double heading = quaternion_to_yaw_angle(
        pose->qx,
        pose->qy,
        pose->qz,
        pose->qw);
    const long long pose_ros_time_ns =
        (long long)pose->stamp_sec * 1000000000LL +
        (long long)pose->stamp_nanosec;
    const double pose_cov_x = pose->covariance[0];
    const double pose_cov_y = pose->covariance[7];
    const double pose_cov_yaw = pose->covariance[35];

    /* Local variables for MPC computation and logging */
    int closest = 0;
    double ey = 0.0, epsi = 0.0;
    double vref0 = 0.0, kappa0 = 0.0;
    double left_wall0 = 0.0, right_wall0 = 0.0;
    double ref_yaw_rate0 = 0.0;
    double path_x = 0.0, path_y = 0.0, path_heading = 0.0, path_s = 0.0, path_t = 0.0;
    int path_segment_idx = 0;
    double local_wp0_x = 0.0, local_wp0_y = 0.0, local_wp0_heading = 0.0, local_wp0_s = 0.0;
    double odom_age_ms = 0.0;
    double publish_speed_cmd = g_last_cmd_speed;
    double publish_accel_cmd = g_last_cmd_accel;
    PathProjection_t projection = {0};
    int trajectory_count_snapshot = 0;
    double local_raceline_length_snapshot = 0.0;
    unsigned long local_raceline_seq_snapshot = 0;
    long long local_raceline_ros_time_snapshot = 0;
    int odom_received_snapshot = 0;
    double latest_vx_snapshot = 0.0;
    double latest_vy_snapshot = 0.0;
    double latest_omega_snapshot = 0.0;
    struct timespec last_odom_time_snapshot = {0, 0};
    long long last_odom_ros_time_snapshot = 0;
    double actual_steer_snapshot = 0.0;
    double last_servo_raw_snapshot = 0.0;
    int use_steering_feedback_snapshot = 0;

    pthread_mutex_lock(&g_state_mutex);
    odom_received_snapshot = global_odometry_received_flag;
    latest_vx_snapshot = g_latest_vx;
    latest_vy_snapshot = g_latest_vy;
    latest_omega_snapshot = g_latest_omega;
    last_odom_time_snapshot = g_last_odom_time;
    last_odom_ros_time_snapshot = g_last_odom_ros_time_ns;
    actual_steer_snapshot = global_actual_steering_angle;
    last_servo_raw_snapshot = g_last_servo_raw;
    use_steering_feedback_snapshot = g_use_steering_feedback;
    pthread_mutex_unlock(&g_state_mutex);

    /* Don't run MPC until odometry (velocity) has been received */
    if (!odom_received_snapshot)
    {
        return;
    }

    pthread_mutex_lock(&g_trajectory_mutex);
    if (!g_local_raceline_received || global_trajectory_count < 2)
    {
        if (!g_local_raceline_wait_logged)
        {
            printf("[MPC] Waiting for local raceline on %s before enabling control\n",
                   g_local_raceline_topic);
            g_local_raceline_wait_logged = 1;
        }
        pthread_mutex_unlock(&g_trajectory_mutex);
        return;
    }

    /* Safety watchdog: check if odometry velocity is stale */
    {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        double elapsed = timespec_diff_sec(&last_odom_time_snapshot, &now);
        odom_age_ms = elapsed * 1000.0;

        if (elapsed > g_watchdog_timeout_sec)
        {
            if (g_verbose)
            {
                printf("[MPC] WATCHDOG: Odometry stale (%.1fms > %.1fms), continuing\n",
                       elapsed * 1000.0, g_watchdog_timeout_sec * 1000.0);
            }
        }
    }

    if (g_verbose)
    {
        printf("[MPC] State: x=%.2f y=%.2f th=%.2f vx=%.2f vy=%.2f w=%.2f\n",
               pos_x, pos_y, heading, latest_vx_snapshot, latest_vy_snapshot, latest_omega_snapshot);
    }
    if (global_trajectory_count > 1)
    {
        /* The lateral planner republishes /local_raceline every cycle anchored
         * at the car, so segment 0 IS the current reference origin. A global
         * nearest-segment search adds per-cycle cost and can snap to an
         * offset/curved-back segment; project from index 0 instead. */
        closest = 0;
        build_reference_from_local_raceline();

        project_to_path_segment(pos_x, pos_y, heading, closest, &projection, &global_frenet_state);
        global_frenet_state.flong_vel = latest_vx_snapshot;
        global_frenet_state.flat_vel = latest_vy_snapshot;
        global_frenet_state.fyaw_rate = latest_omega_snapshot;

        ey = global_frenet_state.flat_error;
        epsi = global_frenet_state.fhead_error;
        vref0 = global_reference_trajectory[0].reference_velocity;
        kappa0 = global_reference_trajectory[0].path_curvature;
        ref_yaw_rate0 = global_reference_trajectory[0].reference_yaw_rate;
        left_wall0 = global_reference_trajectory[0].left_wall_bound;
        right_wall0 = global_reference_trajectory[0].right_wall_bound;
        path_x = projection.path_x_m;
        path_y = projection.path_y_m;
        path_heading = projection.path_heading_rad;
        path_s = projection.path_s_m;
        path_t = projection.segment_t;
        path_segment_idx = projection.segment_index;
        local_wp0_x = global_trajectory[0].x_meters;
        local_wp0_y = global_trajectory[0].y_meters;
        local_wp0_heading = global_trajectory[0].heading_radians;
        local_wp0_s = global_trajectory[0].s_meters;
        trajectory_count_snapshot = global_trajectory_count;
        local_raceline_length_snapshot = g_local_raceline_length_meters;
        local_raceline_seq_snapshot = g_local_raceline_update_seq;
        local_raceline_ros_time_snapshot = g_local_raceline_ros_time_ns;

        if (g_verbose)
        {
            printf("[MPC] Frenet: e_y=%.3f e_psi=%.3f v_ref=%.2f kappa=%.3f\n",
                   ey, epsi, vref0, kappa0);
        }
    }
    else
    {
        if (g_verbose)
        {
            printf("[MPC] ERROR: No trajectory loaded, publishing last command\n");
        }
        pthread_mutex_lock(&g_drive_mutex);
        global_drive_message_buffer.header.stamp.sec = drive_stamp_sec;
        global_drive_message_buffer.header.stamp.nanosec = drive_stamp_nanosec;
        global_drive_message_buffer.drive.steering_angle = g_last_cmd_steer;
        global_drive_message_buffer.drive.speed = g_last_cmd_speed;
        global_drive_message_buffer.drive.acceleration = g_last_cmd_accel;
        g_drive_command_ready = 1;
        publish_drive_command_or_warn("no_trajectory_fallback");
        pthread_mutex_unlock(&g_drive_mutex);
        pthread_mutex_unlock(&g_trajectory_mutex);
        return;
    }
    pthread_mutex_unlock(&g_trajectory_mutex);

    /* ===== Run MPC — output used DIRECTLY, no post-processing ===== */
    MpcSolverResult_t mpc_result;
    MpcSolverStatus_t mpc_status;
    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    publish_float64_metric(&g_timing_solver_enter_seq_publisher,
                           (double)pose_seq,
                           "solver_enter_seq");
    mpc_status = mpc_compute_optimal_control(
        &global_frenet_state,
        global_reference_trajectory,
        &mpc_result);
    clock_gettime(CLOCK_MONOTONIC, &t1);
    double solve_us = (t1.tv_sec - t0.tv_sec) * 1e6 +
                      (t1.tv_nsec - t0.tv_nsec) / 1e3;
    publish_float64_metric(&g_timing_solve_us_publisher,
                           solve_us,
                           "solve_us");
    double primal_res = mpc_result.final_cost;
    double dual_res = mpc_result.dual_residual;

    /* Collect rolling statistics (always active, lightweight) */
    uint16_t iterations_used = mpc_result.iterations_used;
    MpcConfiguration_t cfg = mpc_get_configuration();
    uint16_t max_iter = cfg.max_solver_iterations;

    /* Track solve time statistics */
    g_solve_time_sum_us += solve_us;
    if (solve_us < g_solve_time_min_us) g_solve_time_min_us = solve_us;
    if (solve_us > g_solve_time_max_us) g_solve_time_max_us = solve_us;

    /* Track iteration statistics */
    g_iter_count_sum += iterations_used;
    if (iterations_used < g_iter_count_min) g_iter_count_min = iterations_used;
    if (iterations_used > g_iter_count_max) g_iter_count_max = iterations_used;

    /* Count optimal solutions and max-iter cases */
    if (mpc_status == MPC_STATUS_SUCCESS) g_stats_optimal_count++;
    if (iterations_used >= max_iter) g_stats_max_iter_count++;

    g_stats_cycle_count++;

    /* Write to stats CSV file */
    if (g_stats_csv_file != NULL)
    {
        fprintf(g_stats_csv_file, "%lu,%u,%.1f\n", g_stats_cycle_count, iterations_used, solve_us);
    }

    /* Print terminal stats every 5 seconds */
    {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        double elapsed_sec = (now.tv_sec - g_last_terminal_print_time.tv_sec) +
                             (now.tv_nsec - g_last_terminal_print_time.tv_nsec) / 1e9;

        if (elapsed_sec >= TERMINAL_STATS_PRINT_INTERVAL_SECONDS)
        {
            if (g_stats_cycle_count > 0)
            {
                double avg_iter = g_iter_count_sum / (double)g_stats_cycle_count;
                double avg_time = g_solve_time_sum_us / (double)g_stats_cycle_count;
                double optimal_pct = (g_stats_optimal_count * 100.0) / (double)g_stats_cycle_count;
                double max_iter_pct = (g_stats_max_iter_count * 100.0) / (double)g_stats_cycle_count;

                printf("[MPC] Stats (last %.1fs, %lu calls):\n", elapsed_sec, g_stats_cycle_count);
                printf("  Iterations: min=%u, avg=%.1f, max=%u\n",
                       g_iter_count_min, avg_iter, g_iter_count_max);
                printf("  Solve time: min=%.1f us, avg=%.1f us, max=%.1f us\n",
                       g_solve_time_min_us, avg_time, g_solve_time_max_us);
                printf("  Optimal: %.1f%%, Max iter: %.1f%%\n",
                       optimal_pct, max_iter_pct);
                if (g_stats_csv_file != NULL)
                {
                    fflush(g_stats_csv_file);
                }

                /* Reset stats */
                g_solve_time_sum_us = 0.0;
                g_solve_time_min_us = 1e9;
                g_solve_time_max_us = 0.0;
                g_iter_count_min = 0xFFFF;
                g_iter_count_max = 0;
                g_iter_count_sum = 0.0;
                g_stats_cycle_count = 0;
                g_stats_optimal_count = 0;
                g_stats_max_iter_count = 0;
                g_last_terminal_print_time = now;
            }
        }
    }

    if (mpc_status == MPC_STATUS_SUCCESS ||
        mpc_status == MPC_STATUS_MAXIMUM_ITERATIONS_REACHED)
    {
        double steer =
            mpc_result.optimal_control.steer_ang;

        /* Pass MPC output directly — no clamping, no bias, no softening. */
        global_control_command.steer_ang =
            mpc_result.optimal_control.steer_ang;
        global_control_command.long_acc =
            mpc_result.optimal_control.long_acc;

        /* Fallback: if at standstill and MPC commands max braking, override
         * acceleration to +0.5 m/s^2 while keeping MPC steering. */
        {
            const double vx = latest_vx_snapshot;
            const double a_cmd = (double)global_control_command.long_acc;
            global_control_command.long_acc =
                (float)apply_standstill_brake_override(vx, a_cmd, 1);
        }

        /* Update servo tracking.
         * If steering feedback is available from VESC, it's already set by
         * the servo callback. Otherwise, simulate servo dynamics with rate limit. */
        pthread_mutex_lock(&g_state_mutex);
        actual_steer_snapshot = global_actual_steering_angle;
        use_steering_feedback_snapshot = g_use_steering_feedback;
        if (!use_steering_feedback_snapshot)
        {
            double max_delta = STEERING_RATE_LIMIT * CONTROL_DT_SECONDS;
            double steer_diff = steer - actual_steer_snapshot;
            if (steer_diff > max_delta) steer_diff = max_delta;
            if (steer_diff < -max_delta) steer_diff = -max_delta;
            actual_steer_snapshot += steer_diff;
            global_actual_steering_angle = actual_steer_snapshot;
        }
        last_servo_raw_snapshot = g_last_servo_raw;
        pthread_mutex_unlock(&g_state_mutex);

        /* Feed actual servo position back to MPC */
        {
            ControlInput_t actual_ctrl;
            actual_ctrl.steer_ang =
                actual_steer_snapshot;
            actual_ctrl.long_acc =
                global_control_command.long_acc;
            mpc_set_actual_previous_control(&actual_ctrl);
        }

        if (g_verbose && g_solver_log_file == NULL)
        {
            double accel =
                mpc_result.optimal_control.long_acc;
            printf("[MPC] Control: steer=%.4f accel=%.2f (status=%d iter=%d pr=%.3e dr=%.3e solve=%.1fus)\n",
                   steer, accel, mpc_status, mpc_result.iterations_used,
                   primal_res, dual_res, solve_us);
        }
    }
    else
    {
        if (g_verbose)
        {
            printf("[MPC] WARNING: Solver status=%d, holding last command\n", mpc_status);
        }
    }

    /* Publish drive command */
    double output_gap_ms = 0.0;
    {
        pthread_mutex_lock(&g_drive_mutex);
        global_drive_message_buffer.header.stamp.sec = drive_stamp_sec;
        global_drive_message_buffer.header.stamp.nanosec = drive_stamp_nanosec;
        global_drive_message_buffer.drive.steering_angle =
            global_control_command.steer_ang;

        /* Convert acceleration command to velocity target over one prediction step.
         * This keeps the velocity command consistent with the MPC integration model. */
        {
            double a_cmd =
                global_control_command.long_acc;
            const MpcConfiguration_t cfg = mpc_get_configuration();
            const double pred_dt = (cfg.time_step > 0.0f) ? (double)cfg.time_step : (double)TIME_STEP_SECONDS;

            /* Apply the same standstill brake-override in all publish paths
             * (including solver hold-last-command fallback). */
            a_cmd = apply_standstill_brake_override(latest_vx_snapshot, a_cmd, 0);

            /* Integrate over the prediction time step used by MPC. */
            double v_cmd = latest_vx_snapshot + a_cmd * pred_dt;
            if (v_cmd < (double)VP_MIN_VELOCITY_MPS) v_cmd = (double)VP_MIN_VELOCITY_MPS;
            if (v_cmd > TRAJECTORY_MAXIMUM_VELOCITY) v_cmd = TRAJECTORY_MAXIMUM_VELOCITY;
            if (vref0 > 0.0)
            {
                if (isfinite(vref0))
                {
                    const double v_cap = vref0 + g_raceline_speed_margin_mps;
                    if (v_cmd > v_cap) v_cmd = v_cap;
                }
            }

            global_drive_message_buffer.drive.speed = (float)v_cmd;
            global_drive_message_buffer.drive.acceleration = (float)a_cmd;
            publish_speed_cmd = v_cmd;
            publish_accel_cmd = a_cmd;
        }

        /* Cache last published values for fallback paths. */
        g_last_cmd_steer = global_drive_message_buffer.drive.steering_angle;
        g_last_cmd_speed = global_drive_message_buffer.drive.speed;
        g_last_cmd_accel = global_drive_message_buffer.drive.acceleration;
        g_drive_command_ready = 1;

        {
            struct timespec now_mono;
            clock_gettime(CLOCK_MONOTONIC, &now_mono);
            if (g_last_mpc_output_valid)
            {
                output_gap_ms =
                    timespec_diff_sec(&g_last_mpc_output_time, &now_mono) * 1000.0;
            }
            g_last_mpc_output_time = now_mono;
            g_last_mpc_output_valid = 1;
            g_mpc_output_seq++;
        }

        publish_drive_command_or_warn("mpc_control_thread");
        pthread_mutex_unlock(&g_drive_mutex);
    }

    publish_float64_metric(&g_timing_output_gap_ms_publisher,
                           output_gap_ms,
                           "output_gap_ms");

    if (output_gap_ms > 50.0)
    {
        fprintf(stderr,
                "[MPC] WARNING: MPC output gap %.1f ms (pose_seq=%lu skipped=%lu solve=%.1f us status=%d iter=%u output_seq=%lu)\n",
                output_gap_ms,
                pose_seq,
                skipped_poses,
                solve_us,
                (int)mpc_status,
                (unsigned int)mpc_result.iterations_used,
                g_mpc_output_seq);
    }

    /* Optional per-cycle solver telemetry (CSV) for post-drive analysis. */
    if (g_solver_log_file != NULL)
    {
        g_solver_log_counter++;
        if ((g_solver_log_counter % (unsigned long)g_solver_log_stride) == 0)
        {
            struct timespec now_rt;
            clock_gettime(CLOCK_REALTIME, &now_rt);
            long long unix_time_ns =
                (long long)now_rt.tv_sec * 1000000000LL + (long long)now_rt.tv_nsec;

            double cmd_steer = mpc_result.optimal_control.steer_ang;
            double cmd_accel = mpc_result.optimal_control.long_acc;

            fprintf(g_solver_log_file,
                    "%lld,%lld,%lu,%lu,%.3f,%.3f,%.3f,%.3f,%d,%u,%.9f,%.9f,%d,%lu,%lld,%d,%.6f,%.3f,"
                    "%.6f,%.6f,%.6f,%.9f,%.9f,%.9f,"
                    "%.6f,%.6f,%.6f,%.6f,%.6f,"
                    "%.6f,%.6f,%.6f,%.6f,%.6f,%d,"
                    "%.6f,%.6f,%.6f,%.6f,"
                    "%.6f,%.6f,%.6f,%.6f,%.6f,"
                    "%.6f,%.6f,%.6f,%.6f,"
                    "%.6f,%.3f,%d,%lld\n",
                    pose_ros_time_ns, unix_time_ns, pose_seq, skipped_poses,
                    ekf_to_control_ms, control_gap_ms, output_gap_ms,
                    solve_us, (int)mpc_status, mpc_result.iterations_used,
                    primal_res, dual_res, closest, local_raceline_seq_snapshot, local_raceline_ros_time_snapshot,
                    trajectory_count_snapshot, local_raceline_length_snapshot, odom_age_ms,
                    pos_x, pos_y, heading, pose_cov_x, pose_cov_y, pose_cov_yaw,
                    ey, epsi, latest_vx_snapshot, latest_vy_snapshot, latest_omega_snapshot,
                    path_x, path_y, path_heading, path_s, path_t, path_segment_idx,
                    vref0, kappa0, ref_yaw_rate0, left_wall0, right_wall0,
                    local_wp0_x, local_wp0_y, local_wp0_heading, local_wp0_s,
                    cmd_steer, cmd_accel, publish_speed_cmd, publish_accel_cmd,
                    actual_steer_snapshot, last_servo_raw_snapshot,
                    use_steering_feedback_snapshot, last_odom_ros_time_snapshot);

            if ((g_solver_log_counter % 200UL) == 0UL)
            {
                fflush(g_solver_log_file);
            }
        }
    }
}

/*===========================================================================
 * ROS2 Callback: EKF Pose Subscription
 *===========================================================================
 * Fast path only: copy latest pose, bump sequence, signal control thread.
 *===========================================================================*/
void ekf_pose_callback(const void *message_in)
{
    if (message_in == NULL) return;

    const geometry_msgs__msg__PoseWithCovarianceStamped *msg =
        (const geometry_msgs__msg__PoseWithCovarianceStamped *)message_in;

    PoseSnapshot_t snapshot;
    clock_gettime(CLOCK_MONOTONIC, &snapshot.received_mono_time);
    snapshot.stamp_sec = msg->header.stamp.sec;
    snapshot.stamp_nanosec = msg->header.stamp.nanosec;
    snapshot.pos_x = msg->pose.pose.position.x;
    snapshot.pos_y = msg->pose.pose.position.y;
    snapshot.qx = msg->pose.pose.orientation.x;
    snapshot.qy = msg->pose.pose.orientation.y;
    snapshot.qz = msg->pose.pose.orientation.z;
    snapshot.qw = msg->pose.pose.orientation.w;
    memcpy(snapshot.covariance, msg->pose.covariance, sizeof(snapshot.covariance));

    pthread_mutex_lock(&g_pose_mutex);
    g_latest_pose_snapshot = snapshot;
    g_latest_pose_valid = 1;
    g_latest_pose_seq++;
    pthread_cond_signal(&g_pose_cond);
    pthread_mutex_unlock(&g_pose_mutex);

    if (!g_ekf_pose_received) {
        printf("[MPC] EKF pose received — starting MPC control\n");
        g_ekf_pose_received = 1;
    }
}

static void *mpc_control_thread_main(void *arg)
{
    (void)arg;
    unsigned long processed_seq = 0;

    while (!global_shutdown_requested)
    {
        PoseSnapshot_t pose;
        unsigned long seq = 0;

        pthread_mutex_lock(&g_pose_mutex);
        while (!global_shutdown_requested &&
               (!g_latest_pose_valid || g_latest_pose_seq == processed_seq))
        {
            pthread_cond_wait(&g_pose_cond, &g_pose_mutex);
        }
        if (global_shutdown_requested)
        {
            pthread_mutex_unlock(&g_pose_mutex);
            break;
        }
        pose = g_latest_pose_snapshot;
        seq = g_latest_pose_seq;
        pthread_mutex_unlock(&g_pose_mutex);

        unsigned long skipped_poses =
            (seq > processed_seq + 1UL) ? (seq - processed_seq - 1UL) : 0UL;
        processed_seq = seq;
        run_mpc_for_pose(&pose, seq, skipped_poses);
    }

    return NULL;
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
    signal(SIGSEGV, fatal_signal_handler);
    signal(SIGABRT, fatal_signal_handler);
    signal(SIGFPE, fatal_signal_handler);
    signal(SIGILL, fatal_signal_handler);
    signal(SIGBUS, fatal_signal_handler);

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
        if ((env_val = getenv("MPC_EKF_TOPIC")) != NULL)
            g_ekf_pose_topic = env_val;
        if ((env_val = getenv("MPC_LOCAL_RACELINE_TOPIC")) != NULL)
            g_local_raceline_topic = env_val;
        if ((env_val = getenv("MPC_RACELINE_SPEED_MARGIN")) != NULL)
        {
            double margin = atof(env_val);
            if (margin >= 0.0 && margin <= 5.0) g_raceline_speed_margin_mps = margin;
        }
        if ((env_val = getenv("MPC_DRIVE_REPUBLISH_PERIOD_MS")) != NULL)
        {
            double period_ms = atof(env_val);
            if (period_ms >= 1.0 && period_ms <= 100.0) g_drive_republish_period_ms = period_ms;
        }
        if ((env_val = getenv("MPC_SOLVER_CSV_LOG")) != NULL)
            g_solver_csv_logging_enabled = atoi(env_val) != 0;
        if ((env_val = getenv("MPC_LOG_LOCAL_RACELINE_SNAPSHOTS")) != NULL)
            g_log_local_raceline_snapshots = atoi(env_val) != 0;
    }

    if (g_solver_csv_logging_enabled)
    {
        /* Solver CSV logging is opt-in only. Per-cycle file I/O can stall
         * the real-time control thread on Jetson storage. */
        time_t now = time(NULL);
        char timestamp[64];
        struct tm *tm_now = localtime(&now);
        strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", tm_now);

        char log_path[PATH_MAX];
        snprintf(log_path, sizeof(log_path), "log/solver_%s.csv", timestamp);

        /* Use configured stride (default 1) already present in g_solver_log_stride */

        ensure_parent_directories(log_path);

        g_solver_log_file = fopen(log_path, "w");
        if (g_solver_log_file == NULL)
        {
            fprintf(stderr, "[MPC] WARNING: Failed to open solver log file %s\n", log_path);
        }
        else
        {
            fprintf(g_solver_log_file,
                    "pose_ros_time_ns,unix_time_ns,pose_seq,skipped_poses,ekf_to_control_ms,control_gap_ms,output_gap_ms,"
                    "solve_us,status,iterations,primal_residual,dual_residual,"
                    "closest_wp,local_raceline_seq,local_raceline_ros_time_ns,local_raceline_count,local_raceline_length_m,odom_age_ms,"
                    "pos_x,pos_y,heading,pose_cov_x,pose_cov_y,pose_cov_yaw,"
                    "e_y,e_psi,vx,vy,omega,"
                    "path_x,path_y,path_heading,path_s,path_t,path_segment_idx,"
                    "v_ref0,kappa0,ref_yaw_rate0,left_wall0,right_wall0,"
                    "local_wp0_x,local_wp0_y,local_wp0_heading,local_wp0_s,"
                    "cmd_steer,cmd_accel,publish_speed_cmd,publish_accel_cmd,"
                    "actual_steer,servo_raw,use_steering_feedback,odom_ros_time_ns\n");
            fflush(g_solver_log_file);
            printf("[MPC] Solver telemetry log: %s (stride=%d)\n", log_path, g_solver_log_stride);

            /* Heavy local raceline snapshots are opt-in only; writing all
             * waypoints from the callback can stall real-time publishing. */
            if (g_log_local_raceline_snapshots)
            {
                char local_raceline_path[PATH_MAX];
                snprintf(local_raceline_path, sizeof(local_raceline_path), "log/local_raceline_%s.csv", timestamp);
                ensure_parent_directories(local_raceline_path);
                g_local_raceline_log_file = fopen(local_raceline_path, "w");
                if (g_local_raceline_log_file != NULL)
                {
                    fprintf(g_local_raceline_log_file,
                            "local_raceline_seq,local_raceline_ros_time_ns,waypoint_index,waypoint_count,"
                            "s_m,x_m,y_m,heading_rad,kappa_radpm,v_ref_mps,left_bound_m,right_bound_m\n");
                    fflush(g_local_raceline_log_file);
                    printf("[MPC] Local raceline snapshot log: %s\n", local_raceline_path);
                }
                else
                {
                    fprintf(stderr, "[MPC] WARNING: Failed to open local raceline log file %s\n",
                            local_raceline_path);
                }
            }

            /* Write metadata file next to the solver log */
            {
                char meta_path[PATH_MAX];
                int meta_len = snprintf(meta_path, sizeof(meta_path), "%s.meta.txt", log_path);
                if (meta_len > 0 && (size_t)meta_len < sizeof(meta_path))
                {
                    MpcConfiguration_t cfg = mpc_get_configuration();
                    g_solver_meta_file = fopen(meta_path, "w");
                    if (g_solver_meta_file != NULL)
                    {
                        fprintf(g_solver_meta_file, "log_path=%s\n", log_path);
                        fprintf(g_solver_meta_file, "local_raceline_log_path=%s\n",
                                g_log_local_raceline_snapshots ? "log/local_raceline_<timestamp>.csv" : "disabled");
                        fprintf(g_solver_meta_file, "control_rate_hz=%.3f\n", (double)CONTROL_RATE_HZ);
                        fprintf(g_solver_meta_file, "control_dt_s=%.6f\n", (double)CONTROL_DT_SECONDS);
                        fprintf(g_solver_meta_file, "prediction_horizon=%d\n", PREDICTION_HORIZON);
                        fprintf(g_solver_meta_file, "prediction_dt_s=%.6f\n", (double)cfg.time_step);
                        fprintf(g_solver_meta_file, "weight_lat=%.9g\n", (double)cfg.weight_lateral_error);
                        fprintf(g_solver_meta_file, "weight_heading=%.9g\n", (double)cfg.weight_heading_error);
                        fprintf(g_solver_meta_file, "weight_velocity=%.9g\n", (double)cfg.weight_velocity);
                        fprintf(g_solver_meta_file, "weight_lat_vel=%.9g\n", (double)cfg.weight_lateral_velocity);
                        fprintf(g_solver_meta_file, "weight_yaw_rate=%.9g\n", (double)cfg.weight_yaw_rate);
                        fprintf(g_solver_meta_file, "weight_steer_effort=%.9g\n", (double)cfg.weight_steering_effort);
                        fprintf(g_solver_meta_file, "weight_accel_effort=%.9g\n", (double)cfg.weight_acceleration_effort);
                        fprintf(g_solver_meta_file, "weight_steer_rate=%.9g\n", (double)cfg.weight_steering_rate);
                        fprintf(g_solver_meta_file, "weight_accel_rate=%.9g\n", (double)cfg.weight_acceleration_rate);
                        fprintf(g_solver_meta_file, "weight_delta_actual=%.9g\n", (double)cfg.weight_delta_actual);
                        fprintf(g_solver_meta_file, "solver_max_iter=%u\n", (unsigned int)cfg.max_solver_iterations);
                        fprintf(g_solver_meta_file, "solver_tol=%.9g\n", (double)cfg.solver_convergence_tolerance);
                        fprintf(g_solver_meta_file, "vehicle_mass_kg=%.9g\n", (double)VP_MASS_KG);
                        fprintf(g_solver_meta_file, "vehicle_iz_kgm2=%.9g\n", (double)VP_YAW_INERTIA_KGM2);
                        fprintf(g_solver_meta_file, "vehicle_lf_m=%.9g\n", (double)VP_CG_TO_FRONT_AXLE_M);
                        fprintf(g_solver_meta_file, "vehicle_lr_m=%.9g\n", (double)VP_CG_TO_REAR_AXLE_M);
                        fprintf(g_solver_meta_file, "vehicle_hcg_m=%.9g\n", (double)VP_CG_HEIGHT_M);
                        fprintf(g_solver_meta_file, "vehicle_steer_max_rad=%.9g\n", (double)VP_MAX_STEERING_RAD);
                        fprintf(g_solver_meta_file, "vehicle_min_speed_mps=%.9g\n", (double)VP_MIN_VELOCITY_MPS);
                        fprintf(g_solver_meta_file, "vehicle_max_speed_mps=%.9g\n", (double)VP_MAX_VELOCITY_MPS);
                        fprintf(g_solver_meta_file, "servo_gain=%.9g\n", (double)STEERING_TO_SERVO_GAIN);
                        fprintf(g_solver_meta_file, "servo_offset=%.9g\n", (double)STEERING_TO_SERVO_OFFSET);
                        fprintf(g_solver_meta_file, "steering_rate_limit=%.9g\n", (double)STEERING_RATE_LIMIT);
                        fprintf(g_solver_meta_file, "raceline_speed_margin_mps=%.9g\n", g_raceline_speed_margin_mps);
                        fprintf(g_solver_meta_file, "use_solver_log_stride=%d\n", g_solver_log_stride);
                        fflush(g_solver_meta_file);
                    }
                }
            }

            /* Also open stats CSV file for simple idx/iter/solve_time logging */
            {
                char stats_csv_path[PATH_MAX];
                int stats_len = snprintf(stats_csv_path, sizeof(stats_csv_path), "%s.stats.csv", log_path);
                if (stats_len > 0 && (size_t)stats_len < sizeof(stats_csv_path))
                {
                    g_stats_csv_file = fopen(stats_csv_path, "w");
                    if (g_stats_csv_file != NULL)
                    {
                        fprintf(g_stats_csv_file, "idx,iterations,solve_time_us\n");
                        fflush(g_stats_csv_file);
                        printf("[MPC] Stats CSV log: %s\n", stats_csv_path);
                    }
                }
            }
        }
    }
    else
    {
        printf("[MPC] Solver CSV logging disabled (set MPC_SOLVER_CSV_LOG=1 to enable)\n");
    }

    {
        MpcConfiguration_t cfg = mpc_get_configuration();
        printf("[MPC] Controller initialized (horizon=%d, dt=%.0fms)\n",
               PREDICTION_HORIZON, (double)cfg.time_step * 1000.0);
    }

    printf("[MPC] Control mode: EKF-driven solver, fixed-rate /drive republish\n");
    printf("[MPC] Topics: odom=%s, drive=%s\n", g_odom_topic, g_drive_topic);
    printf("[MPC] Servo feedback: %s (gain=%.4f, offset=%.4f)\n",
            g_servo_topic, STEERING_TO_SERVO_GAIN, STEERING_TO_SERVO_OFFSET);
    printf("[MPC] Steering correction: c2=%.6f, c1=%.6f, c0=%.6f\n",
            STEERING_CORRECTION_C2, STEERING_CORRECTION_C1, STEERING_CORRECTION_C0);
    printf("[MPC] Watchdog timeout: %.0f ms\n", g_watchdog_timeout_sec * 1000.0);
    printf("[MPC] EKF pose topic: %s\n", g_ekf_pose_topic);
    printf("[MPC] Local raceline topic: %s\n", g_local_raceline_topic);
    printf("[MPC] Raceline speed margin: %.2f m/s\n", g_raceline_speed_margin_mps);
    printf("[MPC] Drive republish period: %.1f ms\n", g_drive_republish_period_ms);
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

    /* Initialize terminal output timer */
    clock_gettime(CLOCK_MONOTONIC, &g_last_terminal_print_time);

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

    /* ===== Subscription 1: /ego_racecar/odom (nav_msgs/Odometry) ===== */
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

    /* Bag-visible timing telemetry. BestEffort/KeepLast(1) avoids blocking control. */
    rmw_qos_profile_t qos_timing = rmw_qos_profile_default;
    qos_timing.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    qos_timing.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    qos_timing.depth = 1;

    rcl_publisher_options_t timing_pub_opts = rcl_publisher_get_default_options();
    timing_pub_opts.qos = qos_timing;

    if (!init_float64_publisher(&g_timing_solve_us_publisher, &node,
                                "/mpc/timing/solve_us", &timing_pub_opts) ||
        !init_float64_publisher(&g_timing_control_gap_ms_publisher, &node,
                                "/mpc/timing/control_gap_ms", &timing_pub_opts) ||
        !init_float64_publisher(&g_timing_ekf_to_control_ms_publisher, &node,
                                "/mpc/timing/ekf_to_control_ms", &timing_pub_opts) ||
        !init_float64_publisher(&g_timing_output_gap_ms_publisher, &node,
                                "/mpc/timing/output_gap_ms", &timing_pub_opts) ||
        !init_float64_publisher(&g_timing_drive_age_ms_publisher, &node,
                                "/mpc/timing/drive_age_ms", &timing_pub_opts) ||
        !init_float64_publisher(&g_timing_pose_seq_publisher, &node,
                                "/mpc/timing/pose_seq", &timing_pub_opts) ||
        !init_float64_publisher(&g_timing_skipped_poses_publisher, &node,
                                "/mpc/timing/skipped_poses", &timing_pub_opts) ||
        !init_float64_publisher(&g_timing_solver_enter_seq_publisher, &node,
                                "/mpc/timing/solver_enter_seq", &timing_pub_opts))
    {
        return 1;
    }
    g_timing_publishers_ready = 1;

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

    /* Executor: subscriptions only. Control and /drive publish run in pthreads. */
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

    if (pthread_create(&g_drive_thread, NULL, drive_publisher_thread_main, NULL) == 0)
    {
        g_drive_thread_started = 1;
    }
    else
    {
        fprintf(stderr, "[MPC] ERROR: failed to start drive publisher thread\n");
        return 1;
    }

    if (pthread_create(&g_control_thread, NULL, mpc_control_thread_main, NULL) == 0)
    {
        g_control_thread_started = 1;
    }
    else
    {
        fprintf(stderr, "[MPC] ERROR: failed to start MPC control thread\n");
        return 1;
    }

    printf("[ROS2] Executor ready (4 subs, MPC thread signaled by %s)\n", g_ekf_pose_topic);
    printf("\n[MPC] Spinning... (waiting for EKF pose on %s, odometry on %s, and local raceline on %s)\n\n",
           g_ekf_pose_topic, g_odom_topic, g_local_raceline_topic);

    rclc_executor_spin(&executor);

    /* Cleanup */
    printf("\n[ROS2] Shutting down...\n");
    global_shutdown_requested = 1;
    pthread_cond_broadcast(&g_pose_cond);
    if (g_control_thread_started)
    {
        pthread_join(g_control_thread, NULL);
    }
    if (g_drive_thread_started)
    {
        pthread_join(g_drive_thread, NULL);
    }
    g_timing_publishers_ready = 0;
    if (g_solver_log_file != NULL)
    {
        fflush(g_solver_log_file);
        fclose(g_solver_log_file);
        g_solver_log_file = NULL;
    }
    if (g_solver_meta_file != NULL)
    {
        fflush(g_solver_meta_file);
        fclose(g_solver_meta_file);
        g_solver_meta_file = NULL;
    }
    if (g_local_raceline_log_file != NULL)
    {
        fflush(g_local_raceline_log_file);
        fclose(g_local_raceline_log_file);
        g_local_raceline_log_file = NULL;
    }
    rclc_executor_fini(&executor);
    nav_msgs__msg__Odometry__fini(&global_odometry_message_buffer);
    std_msgs__msg__Float64__fini(&global_servo_message_buffer);
    ackermann_msgs__msg__AckermannDriveStamped__fini(&global_drive_message_buffer);
    nav_msgs__msg__Path__fini(&global_local_raceline_buffer);
    rcl_ret_t cleanup_rc;
    cleanup_rc = rcl_subscription_fini(&odom_sub, &node); (void)cleanup_rc;
    cleanup_rc = rcl_subscription_fini(&servo_sub, &node); (void)cleanup_rc;
    cleanup_rc = rcl_subscription_fini(&ekf_pose_sub, &node); (void)cleanup_rc;
    cleanup_rc = rcl_subscription_fini(&local_raceline_sub, &node); (void)cleanup_rc;
    geometry_msgs__msg__PoseWithCovarianceStamped__fini(&global_ekf_pose_buffer);
    cleanup_rc = rcl_publisher_fini(&g_timing_solve_us_publisher, &node); (void)cleanup_rc;
    cleanup_rc = rcl_publisher_fini(&g_timing_control_gap_ms_publisher, &node); (void)cleanup_rc;
    cleanup_rc = rcl_publisher_fini(&g_timing_ekf_to_control_ms_publisher, &node); (void)cleanup_rc;
    cleanup_rc = rcl_publisher_fini(&g_timing_output_gap_ms_publisher, &node); (void)cleanup_rc;
    cleanup_rc = rcl_publisher_fini(&g_timing_drive_age_ms_publisher, &node); (void)cleanup_rc;
    cleanup_rc = rcl_publisher_fini(&g_timing_pose_seq_publisher, &node); (void)cleanup_rc;
    cleanup_rc = rcl_publisher_fini(&g_timing_skipped_poses_publisher, &node); (void)cleanup_rc;
    cleanup_rc = rcl_publisher_fini(&g_timing_solver_enter_seq_publisher, &node); (void)cleanup_rc;
    cleanup_rc = rcl_publisher_fini(&global_control_publisher, &node); (void)cleanup_rc;
    cleanup_rc = rcl_node_fini(&node); (void)cleanup_rc;
    cleanup_rc = rcl_context_fini(&ctx); (void)cleanup_rc;

    printf("[ROS2] Cleanup complete\n");
    return 0;
}
