/**
 * @file mpc_hardware_node.c
 * @brief MPC Riccati-ADMM ROS2 Node for F1/10th Real Hardware
 *
 * Production ROS2 node for the F1TENTH car running on Jetson Xavier NX.
 * Uses the same MPC core library (Riccati-ADMM solver) as the simulation
 * node, but stripped of simulation-specific code and optimized for
 * real-time 200 Hz execution on embedded hardware.
 *
 * Architecture (timer-driven):
 *   - Odometry callback: stores latest state (fast, non-blocking)
 *   - Servo feedback callback: stores actual steering angle from VESC
 *   - IMU callback: stores filtered yaw rate for higher-quality feedback
 *   - 200 Hz wall timer: runs MPC computation and publishes the result
 *   - Safety watchdog: zeros command if odometry is stale (>200ms)
 *
 * Topics:
 *   Subscribe: /ego_racecar/odom       (nav_msgs/Odometry)     — VESC odometry [QoS(10)]
 *   Subscribe: /sensors/servo_position_command (std_msgs/Float64) — servo fb [QoS(10)]
 *   Subscribe: /imu/filtered_angular_velocity (std_msgs/Float64)  — filtered yaw [QoS(10)]
 *   Publish:   /drive                  (ackermann_msgs/AckermannDriveStamped) — mux [QoS(10)]
 *
 * IMPORTANT: This node is a transparent bridge between hardware and the
 * MPC solver. Nothing in this file may alter, clamp, bias, or post-process
 * the control output returned by mpc_compute_optimal_control(). All tuning
 * and constraint handling lives inside the solver.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE  /* For CPU_SET, sched_setscheduler */
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
#include "ackermann_msgs/msg/ackermann_drive_stamped.h"
#include "std_msgs/msg/float64.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.h"

/* MPC Core Library Headers (Platform-Independent) */
#include "mpc.h"
#include "mpc_types.h"
#include "fp_math.h"
#include "vehicle_model.h"

/*===========================================================================
 * Configuration Constants
 *===========================================================================*/

/** Number of MPC prediction steps (20 steps x 0.04s = 0.8 second lookahead) */
#define MPC_PREDICTION_HORIZON_STEPS 20

/** Time step between predictions [seconds] — must match MPC_DEFAULT_TIME_STEP_SECONDS */
#define MPC_TIME_STEP_SECONDS 0.04

/** Maximum number of waypoints in loaded trajectory */
#define TRAJECTORY_MAXIMUM_WAYPOINTS 1000

/** Maximum reference velocity [m/s] */
#define TRAJECTORY_MAXIMUM_VELOCITY 20.0

/** Speed gain applied to trajectory velocities (1.0 = full optimal racing speed) */
static double g_speed_gain = 1.0;

/** Configurable topic names */
static const char *g_odom_topic = "/ego_racecar/odom";
static const char *g_drive_topic = "/drive";
static const char *g_servo_topic = "/sensors/servo_position_command";
static const char *g_imu_topic = "/imu/filtered_angular_velocity";
static const char *g_amcl_pose_topic = "/ekf_pose";
static const char *g_trajectory_file = NULL;

/** Enable verbose logging (disabled by default for real-time performance) */
static int g_verbose = 0;

/** Set to 1 once the first /amcl_pose message has been received.
 *  When enabled, position/heading come from the map frame (AMCL)
 *  rather than the odom frame.  The trajectory CSV is in map frame,
 *  so Frenet errors are only correct when this flag is set. */
static int g_amcl_received = 0;

/** Timer-driven control rate [Hz] */
static double g_control_rate_hz = 200.0;

/** Safety watchdog timeout [seconds] */
static double g_watchdog_timeout_sec = 0.2;

/*===========================================================================
 * VESC Servo Conversion Parameters
 *===========================================================================
 * The VESC converts steering angle to servo position:
 *   servo_val = steering_to_servo_gain * delta + steering_to_servo_offset
 * Inversion:
 *   delta = (servo_val - steering_to_servo_offset) / steering_to_servo_gain
 */
static double g_steering_to_servo_gain = -0.794;
static double g_steering_to_servo_offset = 0.55;

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
#define SERVO_RATE_LIMIT  2.849  /* rad/s — matches f1tenth servo sv_max */
static double global_actual_steering_angle = 0.0;
static int g_use_steering_feedback = 0;

/*===========================================================================
 * Trajectory Waypoint (loaded from CSV, stored as double)
 *===========================================================================*/

typedef struct
{
    double x_meters;
    double y_meters;
    double heading_radians;
    double velocity_meters_per_second;
    double curvature_radians_per_meter;
    double left_bound_meters;
    double right_bound_meters;
    double sin_heading;  /* Precomputed sin(heading_radians) — avoids trig in hot path */
    double cos_heading;  /* Precomputed cos(heading_radians) — avoids trig in hot path */
} TrajectoryWaypoint_t;

/*===========================================================================
 * Global State Variables (pre-allocated, no dynamic allocation in hot path)
 *===========================================================================*/

static TrajectoryWaypoint_t global_trajectory[TRAJECTORY_MAXIMUM_WAYPOINTS];
static int global_trajectory_count = 0;
static int global_last_closest_index = 0;
static VehicleState_t global_vehicle_state = {0};
static FrenetState_t global_frenet_state = {0};
static ControlInput_t global_control_command = {0};
static int global_odometry_received_flag = 0;
static volatile sig_atomic_t global_shutdown_requested = 0;
static rcl_context_t *global_ros2_context = NULL;

static rcl_publisher_t global_control_publisher;

static nav_msgs__msg__Odometry global_odometry_message_buffer;
static std_msgs__msg__Float64 global_servo_message_buffer;
static std_msgs__msg__Float64 global_imu_message_buffer;
static ackermann_msgs__msg__AckermannDriveStamped global_drive_message_buffer;
static geometry_msgs__msg__PoseWithCovarianceStamped global_amcl_pose_buffer;

static TrajectoryReferencePoint_t global_reference_trajectory[MPC_PREDICTION_HORIZON_STEPS];

/** Cached latest odometry values for the timer callback (stored in odom callback) */
static double g_latest_pos_x = 0.0;
static double g_latest_pos_y = 0.0;
static double g_latest_heading = 0.0;
static double g_latest_vx = 0.0;
static double g_latest_vy = 0.0;
static double g_latest_omega = 0.0;

/** Filtered IMU yaw rate (updated by IMU callback) */
static double g_imu_yaw_rate = 0.0;
static int g_imu_received = 0;

/** Watchdog: timestamp of last odometry received (CLOCK_MONOTONIC — uses VDSO
 *  fast path on aarch64, ~3ns vs ~50ns syscall for CLOCK_MONOTONIC_RAW) */
static struct timespec g_last_odom_time = {0, 0};

/** Rolling solve-time instrumentation (always active, prints every 500 cycles) */
static double g_solve_time_sum_us = 0.0;
static double g_solve_time_max_us = 0.0;
static unsigned long g_solve_cycle_count = 0;
#define SOLVE_STATS_PRINT_INTERVAL 500

/** Computed control dt from g_control_rate_hz (set in main) */
static double g_control_dt = 0.005;

/** Number of executor handles: 4 subscriptions + 1 timer */
#define EXECUTOR_NUM_HANDLES 5

/*===========================================================================
 * Signal Handler for Graceful Shutdown
 *===========================================================================*/

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
 * @brief Attempt to set real-time scheduling and CPU affinity.
 *
 * Tries SCHED_FIFO with priority 49 (below kernel threads).
 * Falls back silently if not running as root.
 * Pins to CPU core 5 on Jetson (6-core Denver/A57).
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
 * Trajectory Loading (CSV from f1tenth_planning)
 *===========================================================================*/

/** Average waypoint spacing, computed from loaded trajectory. */
static double g_avg_waypoint_spacing = 0.346;

/**
 * @brief Load trajectory from CSV file.
 *
 * CSV format (TUM compatible):
 *   # s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2[,left_bound,right_bound]
 */
static int load_trajectory_from_csv(const char *file_path)
{
    FILE *csv_file = fopen(file_path, "r");
    if (csv_file == NULL)
    {
        fprintf(stderr, "[MPC] ERROR: Cannot open trajectory file: %s\n", file_path);
        return 0;
    }

    char line_buffer[512];
    global_trajectory_count = 0;

    while (fgets(line_buffer, sizeof(line_buffer), csv_file) != NULL)
    {
        if (line_buffer[0] == '#' || line_buffer[0] == '\n' || line_buffer[0] == '\r')
        {
            continue;
        }

        if (global_trajectory_count >= TRAJECTORY_MAXIMUM_WAYPOINTS)
        {
            printf("[MPC] WARNING: Trajectory truncated at %d waypoints\n",
                   TRAJECTORY_MAXIMUM_WAYPOINTS);
            break;
        }

        double s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2;
        double left_bound = 5.0, right_bound = 5.0;
        int fields_read = sscanf(line_buffer, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                                 &s_m, &x_m, &y_m, &psi_rad,
                                 &kappa_radpm, &vx_mps, &ax_mps2,
                                 &left_bound, &right_bound);

        if (fields_read >= 6)
        {
            TrajectoryWaypoint_t *wp = &global_trajectory[global_trajectory_count];
            wp->x_meters = x_m;
            wp->y_meters = y_m;
            wp->heading_radians = psi_rad;
            wp->curvature_radians_per_meter = kappa_radpm;
            wp->left_bound_meters = (fields_read >= 9) ? left_bound : 5.0;
            wp->right_bound_meters = (fields_read >= 9) ? right_bound : 5.0;

            double scaled_velocity = vx_mps * g_speed_gain;
            if (scaled_velocity > TRAJECTORY_MAXIMUM_VELOCITY)
                scaled_velocity = TRAJECTORY_MAXIMUM_VELOCITY;
            if (scaled_velocity < 0.0)
                scaled_velocity = 0.0;
            wp->velocity_meters_per_second = scaled_velocity;

            global_trajectory_count++;
        }
    }

    fclose(csv_file);

    if (global_trajectory_count == 0)
    {
        fprintf(stderr, "[MPC] ERROR: No waypoints loaded from %s\n", file_path);
        return 0;
    }

    /* Precompute sin/cos for each waypoint heading (immutable after load).
     * Eliminates sin()/cos() calls from the 200 Hz Frenet conversion hot path
     * by interpolating precomputed values instead. */
    for (int i = 0; i < global_trajectory_count; i++)
    {
        global_trajectory[i].sin_heading = sin(global_trajectory[i].heading_radians);
        global_trajectory[i].cos_heading = cos(global_trajectory[i].heading_radians);
    }

    printf("[MPC] Loaded %d waypoints from %s\n", global_trajectory_count, file_path);
    printf("[MPC] Speed gain: %.2f, max velocity: %.1f m/s\n",
           g_speed_gain, TRAJECTORY_MAXIMUM_VELOCITY);

    /* Compute average waypoint spacing from loaded data */
    if (global_trajectory_count >= 2)
    {
        double total_spacing = 0.0;
        for (int i = 0; i < global_trajectory_count - 1; i++)
        {
            double dx = global_trajectory[i + 1].x_meters - global_trajectory[i].x_meters;
            double dy = global_trajectory[i + 1].y_meters - global_trajectory[i].y_meters;
            total_spacing += sqrt(dx * dx + dy * dy);
        }
        g_avg_waypoint_spacing = total_spacing / (global_trajectory_count - 1);
        if (g_avg_waypoint_spacing < 0.01) g_avg_waypoint_spacing = 0.01; /* safety floor */
        printf("[MPC] Average waypoint spacing: %.4f m\n", g_avg_waypoint_spacing);
    }

    return 1;
}

/*===========================================================================
 * Waypoint Search
 *===========================================================================*/

/**
 * @brief Find the closest trajectory waypoint to a position.
 *
 * Forward-biased local search from the last known position.
 * Uses a wide search window (200 ahead, 20 behind) to handle
 * high-speed driving and track loop-around, then caches the result
 * for efficient subsequent lookups.
 */
static int find_closest_waypoint(double position_x, double position_y, double vehicle_heading)
{
    if (global_trajectory_count == 0)
        return 0;

    int search_start = global_last_closest_index;
    int search_forward = 200;
    int search_backward = 20;
    int best_index = search_start;
    double best_score = 1e18;

    /* Hoist trig out of loop (loop-invariant) */
    double veh_dx = cos(vehicle_heading);
    double veh_dy = sin(vehicle_heading);

    for (int offset = -search_backward; offset < search_forward; offset++)
    {
        /* Branch-free-ish wrap: conditional subtract is ~1 cycle on ARM
         * vs ~10+ cycles for integer division (modulo).  Safe because
         * search window (220) < trajectory count for any real track. */
        int idx = search_start + offset;
        if (idx >= global_trajectory_count) idx -= global_trajectory_count;
        if (idx < 0) idx += global_trajectory_count;

        double dx = global_trajectory[idx].x_meters - position_x;
        double dy = global_trajectory[idx].y_meters - position_y;
        double dist = dx * dx + dy * dy;

        /* Penalize points behind the vehicle */
        double dot = dx * veh_dx + dy * veh_dy;
        double score = dist + ((dot < 0.0) ? 2.0 : 0.0);

        if (score < best_score)
        {
            best_score = score;
            best_index = idx;
        }
    }

    global_last_closest_index = best_index;
    return best_index;
}

/*===========================================================================
 * Reference Trajectory Builder
 *===========================================================================*/

/**
 * @brief Build MPC reference trajectory from loaded waypoints (Frenet frame).
 *
 * Frenet references: lateral and heading error are zero (follow the path).
 * Each prediction step maps to the waypoint at the expected travel distance.
 */
static void build_reference_from_trajectory(int closest_index)
{
    const double mpc_dt = MPC_TIME_STEP_SECONDS;
    const double avg_waypoint_spacing = g_avg_waypoint_spacing;

    for (int step = 0; step < MPC_PREDICTION_HORIZON_STEPS; step++)
    {
        int base_waypoint_index = closest_index + step;
        if (base_waypoint_index >= global_trajectory_count)
            base_waypoint_index -= global_trajectory_count;
        double ref_velocity = global_trajectory[base_waypoint_index].velocity_meters_per_second;

        if (ref_velocity < 3.0) ref_velocity = 3.0;
        if (ref_velocity > TRAJECTORY_MAXIMUM_VELOCITY) ref_velocity = TRAJECTORY_MAXIMUM_VELOCITY;

        double expected_distance = ref_velocity * mpc_dt * (step + 1);
        int waypoints_ahead = (int)(expected_distance / avg_waypoint_spacing);
        if (waypoints_ahead < step + 1) waypoints_ahead = step + 1;

        int waypoint_index = closest_index + waypoints_ahead;
        if (waypoint_index >= global_trajectory_count)
            waypoint_index -= global_trajectory_count;
        TrajectoryWaypoint_t *wp = &global_trajectory[waypoint_index];

        global_reference_trajectory[step].reference_lateral_error_meters = 0;
        global_reference_trajectory[step].reference_heading_error_radians = 0;

        global_reference_trajectory[step].path_curvature_radians_per_meter =
            DOUBLE_TO_FP(wp->curvature_radians_per_meter);
        global_reference_trajectory[step].left_wall_bound_meters =
            DOUBLE_TO_FP(wp->left_bound_meters);
        global_reference_trajectory[step].right_wall_bound_meters =
            DOUBLE_TO_FP(wp->right_bound_meters);

        double traj_vel = wp->velocity_meters_per_second;
        if (traj_vel < 0.0) traj_vel = 0.0;
        global_reference_trajectory[step].reference_velocity_meters_per_second =
            DOUBLE_TO_FP(traj_vel);

        /* Yaw rate reference = kappa * v_ref (steady-state cornering) — fused in single pass */
        double omega_ref = wp->curvature_radians_per_meter * traj_vel;
        global_reference_trajectory[step].reference_yaw_rate_radians_per_second =
            DOUBLE_TO_FP(omega_ref);
        global_reference_trajectory[step].reference_lateral_velocity_meters_per_second = 0;
    }
}

/*===========================================================================
 * Helper Functions
 *===========================================================================*/

static double quaternion_to_yaw_angle(double qx, double qy, double qz, double qw)
{
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    return atan2(siny_cosp, cosy_cosp);
}

static int preallocate_rosidl_string(rosidl_runtime_c__String *str, size_t capacity)
{
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

static void set_rosidl_string(rosidl_runtime_c__String *str, const char *value)
{
    if (str == NULL || str->data == NULL || value == NULL) return;
    size_t length = strlen(value);
    if (length >= str->capacity) length = str->capacity - 1;
    memcpy(str->data, value, length);
    str->data[length] = '\0';
    str->size = length;
}

/** Get elapsed time in seconds between two timespec values */
static double timespec_diff_sec(struct timespec *a, struct timespec *b)
{
    return (double)(b->tv_sec - a->tv_sec) +
           (double)(b->tv_nsec - a->tv_nsec) / 1e9;
}

/*===========================================================================
 * Frenet State Conversion
 *===========================================================================*/

/**
 * @brief Convert global vehicle state to Frenet (path-relative) state.
 *
 * Projects the car position onto the segment between closest and closest+1
 * waypoints, then interpolates path position and heading at the projection
 * point. Provides smooth Frenet state feedback to the MPC.
 */
static void convert_to_frenet_state(
    double car_x, double car_y, double car_heading,
    int closest_index,
    FrenetState_t *frenet_out)
{
    int idx0 = closest_index;
    int idx1 = (closest_index + 1) % global_trajectory_count;

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
    while (dh > 3.14159265) dh -= 2.0 * 3.14159265;
    while (dh < -3.14159265) dh += 2.0 * 3.14159265;
    double path_heading = h0 + t * dh;

    /* Signed lateral error (positive = left of path).
     * Use linearly-interpolated precomputed sin/cos instead of calling
     * sin()/cos() on the interpolated heading.  Accurate for adjacent
     * waypoints with small heading difference (typical: <0.05 rad). */
    double sin_h = global_trajectory[idx0].sin_heading
                 + t * (global_trajectory[idx1].sin_heading
                      - global_trajectory[idx0].sin_heading);
    double cos_h = global_trajectory[idx0].cos_heading
                 + t * (global_trajectory[idx1].cos_heading
                      - global_trajectory[idx0].cos_heading);
    double dx = car_x - path_x;
    double dy = car_y - path_y;
    double lateral_error = -dx * sin_h + dy * cos_h;

    double heading_error = car_heading - path_heading;
    while (heading_error > 3.14159265) heading_error -= 2.0 * 3.14159265;
    while (heading_error < -3.14159265) heading_error += 2.0 * 3.14159265;

    frenet_out->lateral_error_meters = DOUBLE_TO_FP(lateral_error);
    frenet_out->heading_error_radians = DOUBLE_TO_FP(heading_error);
    frenet_out->longitudinal_velocity_meters_per_second =
        global_vehicle_state.longitudinal_velocity_meters_per_second;
    frenet_out->lateral_velocity_meters_per_second =
        global_vehicle_state.lateral_velocity_meters_per_second;
    frenet_out->yaw_rate_radians_per_second =
        global_vehicle_state.yaw_rate_radians_per_second;
}

/* 8-state augmented model verification note:
 * The FrenetState_t provides x0[0..4] = {e_y, e_psi, vx, vy, omega}.
 * The remaining augmented states are managed by the solver internally:
 *   x0[5] = delta_actual    — set via mpc_set_actual_previous_control()
 *   x0[6] = delta_rate_prev — stored by the solver after each solve
 *   x0[7] = accel_prev      — stored by the solver after each solve
 * The hardware node feeds delta_actual from servo feedback (or rate-limited
 * tracking) after each solve, which the solver uses on the NEXT call.
 * This is correct: the 5-state Frenet + 3 internal augmented states are
 * all properly populated before each solve.
 */

/*===========================================================================
 * ROS2 Callback: Odometry Subscription (non-blocking, just stores state)
 *===========================================================================*/

void odometry_subscription_callback(const void *message_in)
{
    if (message_in == NULL) return;

    const nav_msgs__msg__Odometry *odom =
        (const nav_msgs__msg__Odometry *)message_in;

    double pos_x = odom->pose.pose.position.x;
    double pos_y = odom->pose.pose.position.y;
    double heading = quaternion_to_yaw_angle(
        odom->pose.pose.orientation.x,
        odom->pose.pose.orientation.y,
        odom->pose.pose.orientation.z,
        odom->pose.pose.orientation.w);
    double vx = odom->twist.twist.linear.x;
    double vy = odom->twist.twist.linear.y;
    double omega = odom->twist.twist.angular.z;

    /* Store into vehicle state (fixed-point for MPC) — only store fields
     * actually consumed by the solver (vx, vy, omega).  Position/heading
     * are used as doubles for Frenet conversion, so skip the FP conversion. */
    global_vehicle_state.longitudinal_velocity_meters_per_second = DOUBLE_TO_FP(vx);
    global_vehicle_state.lateral_velocity_meters_per_second = DOUBLE_TO_FP(vy);

    /* Use IMU yaw rate if available (higher quality than odom twist) */
    if (g_imu_received)
    {
        global_vehicle_state.yaw_rate_radians_per_second = DOUBLE_TO_FP(g_imu_yaw_rate);
    }
    else
    {
        global_vehicle_state.yaw_rate_radians_per_second = DOUBLE_TO_FP(omega);
    }

    /* Cache velocity for the timer callback.
     * Position/heading are only taken from odom when AMCL is not available.
     * When AMCL is running the amcl_pose_callback keeps them updated in the
     * map frame, which is the same frame as the trajectory CSV. */
    if (!g_amcl_received) {
        g_latest_pos_x = pos_x;
        g_latest_pos_y = pos_y;
        g_latest_heading = heading;
    }
    g_latest_vx = vx;
    g_latest_vy = vy;
    g_latest_omega = omega;

    /* Update watchdog timestamp */
    clock_gettime(CLOCK_MONOTONIC, &g_last_odom_time);

    global_odometry_received_flag = 1;
}

/*===========================================================================
 * ROS2 Callback: Servo Feedback Subscription
 *===========================================================================*/

void servo_feedback_callback(const void *message_in)
{
    if (message_in == NULL) return;

    const std_msgs__msg__Float64 *msg =
        (const std_msgs__msg__Float64 *)message_in;

    /* Convert servo position (0-1) back to steering angle using VESC formula:
     * servo_val = gain * delta + offset
     * => delta = (servo_val - offset) / gain */
    double servo_val = msg->data;
    if (g_steering_to_servo_gain != 0.0)
    {
        global_actual_steering_angle =
            (servo_val - g_steering_to_servo_offset) / g_steering_to_servo_gain;
    }

    g_use_steering_feedback = 1;

    if (g_verbose)
    {
        printf("[MPC] Servo feedback: servo_val=%.3f -> delta=%.4f rad\n",
               servo_val, global_actual_steering_angle);
    }
}

/*===========================================================================
 * ROS2 Callback: IMU Filtered Angular Velocity
 *===========================================================================*/

void imu_callback(const void *message_in)
{
    if (message_in == NULL) return;

    const std_msgs__msg__Float64 *msg =
        (const std_msgs__msg__Float64 *)message_in;

    g_imu_yaw_rate = msg->data;
    g_imu_received = 1;
}

/*===========================================================================
 * ROS2 Callback: Map-Frame Pose (EKF or AMCL)
 *===========================================================================
 * Overrides the odom-frame position from odometry_subscription_callback.
 * The trajectory CSV is in map frame, so Frenet errors are only meaningful
 * when position comes from here rather than from raw wheel odometry.
 * Default source: /ekf_pose (EKF fuses odom + AMCL for smooth updates).
 *===========================================================================*/
void amcl_pose_callback(const void *message_in)
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

    g_latest_pos_x   = pos_x;
    g_latest_pos_y   = pos_y;
    g_latest_heading = heading;

    if (!g_amcl_received) {
        printf("[MPC] Map-frame pose received — switching to map-frame position\n");
        g_amcl_received = 1;
    }
}

/*===========================================================================
 * Timer Callback: MPC Computation + Publish (200 Hz)
 *===========================================================================*/

void mpc_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)timer;
    (void)last_call_time;

    /* Safety watchdog: check if odometry is stale */
    if (global_odometry_received_flag)
    {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        double elapsed = timespec_diff_sec(&g_last_odom_time, &now);

        if (elapsed > g_watchdog_timeout_sec)
        {
            /* Odometry stale — zero the command for safety */
            global_drive_message_buffer.drive.steering_angle = 0.0f;
            global_drive_message_buffer.drive.speed = 0.0f;
            global_drive_message_buffer.drive.acceleration = 0.0f;

            rcl_ret_t pub_rc __attribute__((unused)) =
                rcl_publish(&global_control_publisher, &global_drive_message_buffer, NULL);

            if (g_verbose)
            {
                printf("[MPC] WATCHDOG: Odometry stale (%.1fms > %.1fms), zeroing command\n",
                       elapsed * 1000.0, g_watchdog_timeout_sec * 1000.0);
            }
            return;
        }
    }
    else
    {
        /* No odometry received yet — don't publish anything */
        return;
    }

    /* Use cached latest odometry values */
    double pos_x = g_latest_pos_x;
    double pos_y = g_latest_pos_y;
    double heading = g_latest_heading;

    if (g_verbose)
    {
        printf("[MPC] State: x=%.2f y=%.2f th=%.2f vx=%.2f vy=%.2f w=%.2f\n",
               pos_x, pos_y, heading, g_latest_vx, g_latest_vy, g_latest_omega);
    }

    if (global_trajectory_count > 0)
    {
        int closest = find_closest_waypoint(pos_x, pos_y, heading);
        build_reference_from_trajectory(closest);
        convert_to_frenet_state(pos_x, pos_y, heading, closest, &global_frenet_state);

        if (g_verbose)
        {
            double ey  = FP_TO_DOUBLE(global_frenet_state.lateral_error_meters);
            double epsi = FP_TO_DOUBLE(global_frenet_state.heading_error_radians);
            double vref = FP_TO_DOUBLE(global_reference_trajectory[0].reference_velocity_meters_per_second);
            double kappa = FP_TO_DOUBLE(global_reference_trajectory[0].path_curvature_radians_per_meter);
            printf("[MPC] Frenet: e_y=%.3f e_psi=%.3f v_ref=%.2f kappa=%.3f\n",
                   ey, epsi, vref, kappa);
        }
    }
    else
    {
        /* Fallback: straight line at low speed */
        fixed_point_t target_velocity = DOUBLE_TO_FP(1.0);

        global_frenet_state.lateral_error_meters = 0;
        global_frenet_state.heading_error_radians = 0;
        global_frenet_state.longitudinal_velocity_meters_per_second =
            global_vehicle_state.longitudinal_velocity_meters_per_second;
        global_frenet_state.lateral_velocity_meters_per_second =
            global_vehicle_state.lateral_velocity_meters_per_second;
        global_frenet_state.yaw_rate_radians_per_second =
            global_vehicle_state.yaw_rate_radians_per_second;

        for (int step = 0; step < MPC_PREDICTION_HORIZON_STEPS; step++)
        {
            global_reference_trajectory[step].reference_lateral_error_meters = 0;
            global_reference_trajectory[step].reference_heading_error_radians = 0;
            global_reference_trajectory[step].path_curvature_radians_per_meter = 0;
            global_reference_trajectory[step].left_wall_bound_meters = DOUBLE_TO_FP(5.0);
            global_reference_trajectory[step].right_wall_bound_meters = DOUBLE_TO_FP(5.0);
            global_reference_trajectory[step].reference_velocity_meters_per_second = target_velocity;
            global_reference_trajectory[step].reference_lateral_velocity_meters_per_second = 0;
            global_reference_trajectory[step].reference_yaw_rate_radians_per_second = 0;
            global_reference_trajectory[step].reference_acceleration_meters_per_second_squared = 0;
        }
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

    /* Rolling solve-time statistics (always active, lightweight) */
    g_solve_time_sum_us += solve_us;
    if (solve_us > g_solve_time_max_us) g_solve_time_max_us = solve_us;
    g_solve_cycle_count++;
    if (g_solve_cycle_count >= SOLVE_STATS_PRINT_INTERVAL)
    {
        double avg_us = g_solve_time_sum_us / (double)g_solve_cycle_count;
        printf("[MPC] Solve stats (%lu cycles): avg=%.1f us, max=%.1f us (budget=%.0f us)\n",
               g_solve_cycle_count, avg_us, g_solve_time_max_us,
               1e6 / g_control_rate_hz);
        g_solve_time_sum_us = 0.0;
        g_solve_time_max_us = 0.0;
        g_solve_cycle_count = 0;
    }

    if (mpc_status == MPC_STATUS_SUCCESS ||
        mpc_status == MPC_STATUS_MAXIMUM_ITERATIONS_REACHED)
    {
        double steer = FP_TO_DOUBLE(
            mpc_result.optimal_control.steering_angle_radians);

        /* Pass MPC output directly — no clamping, no bias, no softening. */
        global_control_command.steering_angle_radians =
            mpc_result.optimal_control.steering_angle_radians;
        global_control_command.acceleration_meters_per_second_squared =
            mpc_result.optimal_control.acceleration_meters_per_second_squared;

        /* Update servo tracking.
         * If steering feedback is available from VESC, it's already set by
         * the servo callback. Otherwise, simulate servo dynamics with rate limit. */
        if (!g_use_steering_feedback)
        {
            double max_delta = SERVO_RATE_LIMIT * g_control_dt;
            double steer_diff = steer - global_actual_steering_angle;
            if (steer_diff > max_delta) steer_diff = max_delta;
            if (steer_diff < -max_delta) steer_diff = -max_delta;
            global_actual_steering_angle += steer_diff;
        }

        /* Feed actual servo position back to MPC */
        {
            ControlInput_t actual_ctrl;
            actual_ctrl.steering_angle_radians =
                DOUBLE_TO_FP(global_actual_steering_angle);
            actual_ctrl.acceleration_meters_per_second_squared =
                mpc_result.optimal_control.acceleration_meters_per_second_squared;
            mpc_set_actual_previous_control(&actual_ctrl);
        }

        if (g_verbose)
        {
            double accel = FP_TO_DOUBLE(
                mpc_result.optimal_control.acceleration_meters_per_second_squared);
            printf("[MPC] Control: steer=%.4f accel=%.2f (status=%d iter=%d solve=%.1fus)\n",
                   steer, accel, mpc_status, mpc_result.iterations_used, solve_us);
        }
    }
    else
    {
        if (g_verbose)
        {
            printf("[MPC] WARNING: Solver status=%d, keeping previous command\n", mpc_status);
        }
    }

    /* Publish drive command */
    {
        global_drive_message_buffer.drive.steering_angle = FP_TO_FLOAT(
            global_control_command.steering_angle_radians);

        /* VEL_TO_ERPM mode: the ackermann_to_vesc node converts drive.speed
         * to ERPM and the VESC's internal PID handles velocity tracking.
         *
         * drive.speed is computed by integrating the MPC's acceleration command
         * over ONE MPC PREDICTION STEP (MPC_TIME_STEP_SECONDS = 0.05 s),
         * NOT the control timer period (g_control_dt = 0.005 s at 200 Hz).
         *
         * WHY MPC_TIME_STEP_SECONDS and not g_control_dt:
         * At 200 Hz (dt=0.005s), even a 5 m/s^2 command produces only a
         * 0.025 m/s increment per cycle.  ERPM = 4550 * 0.025 = ~114 ERPM.
         * A sensorless BLDC motor needs sufficient back-EMF (hundreds of RPM)
         * for the rotor-position observer to lock onto.  Below that threshold
         * the motor strains, current-limits, and stalls, then MPC re-issues
         * the same tiny target => repeated start-stop.  The slow_start ramp
         * in ackermann_to_vesc never fires because commanded_vel never jumps
         * from <1 m/s to >1 m/s in a single step.
         *
         * drive.acceleration is still set so that systems configured with
         * non-zero accel_to_current_gain / accel_to_brake_gain use the MPC's
         * direct torque command.  Set those gains to 8.935 A/(m/s^2) in
         * vesc.yaml to switch to ACCEL_TO_CURRENT mode.
         */
        {
            double a_cmd = FP_TO_DOUBLE(
                global_control_command.acceleration_meters_per_second_squared);

            /* Integrate MPC acceleration over one MPC prediction step.
             * See the comment block above for why MPC_TIME_STEP_SECONDS (0.04s)
             * is used here rather than g_control_dt. */
            double v_cmd = g_latest_vx + a_cmd * MPC_TIME_STEP_SECONDS;
            if (v_cmd < 0.0) v_cmd = 0.0;
            if (v_cmd > TRAJECTORY_MAXIMUM_VELOCITY) v_cmd = TRAJECTORY_MAXIMUM_VELOCITY;

            global_drive_message_buffer.drive.speed = (float)v_cmd;
            global_drive_message_buffer.drive.acceleration = (float)a_cmd;
        }

        rcl_ret_t pub_rc __attribute__((unused)) =
            rcl_publish(&global_control_publisher, &global_drive_message_buffer, NULL);
    }
}

/*===========================================================================
 * Main Entry Point
 *===========================================================================*/

int main(int argc, char *argv[])
{
    printf("============================================================\n");
    printf("  MPC Riccati-ADMM ROS2 Node for F1/10th Hardware\n");
    printf("  Target: Jetson Xavier NX @ 200 Hz (timer-driven)\n");
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
        const char *env_val;
        if ((env_val = getenv("MPC_SPEED_GAIN")) != NULL)
        {
            double sg = atof(env_val);
            if (sg > 0.0 && sg <= 2.0) g_speed_gain = sg;
        }
        if ((env_val = getenv("MPC_ODOM_TOPIC")) != NULL)
            g_odom_topic = env_val;
        if ((env_val = getenv("MPC_DRIVE_TOPIC")) != NULL)
            g_drive_topic = env_val;
        if ((env_val = getenv("MPC_SERVO_TOPIC")) != NULL)
            g_servo_topic = env_val;
        if ((env_val = getenv("MPC_IMU_TOPIC")) != NULL)
            g_imu_topic = env_val;
        if ((env_val = getenv("MPC_VERBOSE")) != NULL)
            g_verbose = atoi(env_val);
        if ((env_val = getenv("MPC_CONTROL_RATE")) != NULL)
        {
            double rate = atof(env_val);
            if (rate >= 10.0 && rate <= 1000.0) g_control_rate_hz = rate;
        }
        if ((env_val = getenv("MPC_WATCHDOG_TIMEOUT")) != NULL)
        {
            double timeout = atof(env_val);
            if (timeout > 0.0 && timeout <= 5.0) g_watchdog_timeout_sec = timeout;
        }
        if ((env_val = getenv("MPC_SERVO_GAIN")) != NULL)
        {
            double gain = atof(env_val);
            if (gain != 0.0) g_steering_to_servo_gain = gain;
        }
        if ((env_val = getenv("MPC_SERVO_OFFSET")) != NULL)
        {
            g_steering_to_servo_offset = atof(env_val);
        }
        if ((env_val = getenv("MPC_AMCL_TOPIC")) != NULL)
            g_amcl_pose_topic = env_val;
    }

    printf("[MPC] Controller initialized (horizon=%d, dt=%.0fms)\n",
           MPC_PREDICTION_HORIZON_STEPS, MPC_TIME_STEP_SECONDS * 1000.0);

    /* Compute CONTROL_DT from the configured control rate */
    g_control_dt = 1.0 / g_control_rate_hz;

    printf("[MPC] Control rate: %.0f Hz (timer-driven, dt=%.4fs)\n", g_control_rate_hz, g_control_dt);
    printf("[MPC] Topics: odom=%s, drive=%s\n", g_odom_topic, g_drive_topic);
    printf("[MPC] Servo feedback: %s (gain=%.3f, offset=%.3f)\n",
           g_servo_topic, g_steering_to_servo_gain, g_steering_to_servo_offset);
    printf("[MPC] IMU yaw rate: %s\n", g_imu_topic);
    printf("[MPC] Watchdog timeout: %.0f ms\n", g_watchdog_timeout_sec * 1000.0);
    printf("[MPC] Map-frame pose topic: %s\n", g_amcl_pose_topic);
    printf("[MPC] Verbose=%d\n", g_verbose);

    {
        MpcConfiguration_t cfg = mpc_get_configuration();
        printf("[MPC] max_iter=%u, tol=%d\n",
               cfg.maximum_solver_iterations,
               (int)cfg.solver_convergence_tolerance);
        printf("[MPC] Weights: lat=%.1f heading=%.1f vel=%.1f steer_rate=%.2f steer_effort=%.4f\n",
               FP_TO_DOUBLE(cfg.weight_lateral_error),
               FP_TO_DOUBLE(cfg.weight_heading_error),
               FP_TO_DOUBLE(cfg.weight_velocity),
               FP_TO_DOUBLE(cfg.weight_steering_rate),
               FP_TO_DOUBLE(cfg.weight_steering_effort));
    }

    /* Load trajectory — search order:
     *   1. Command-line argument: ./mpc_hardware_node /path/to/raceline.csv
     *   2. Environment variable:  MPC_TRAJECTORY_FILE=/path/to/raceline.csv
     *   3. Default search paths (relative to cwd and common install locations)
     */
    if (argc >= 2)
    {
        g_trajectory_file = argv[1];
    }
    else
    {
        const char *env_val = getenv("MPC_TRAJECTORY_FILE");
        if (env_val != NULL)
            g_trajectory_file = env_val;
    }

    /* If no explicit path given, try common relative/absolute locations */
    if (g_trajectory_file == NULL || strlen(g_trajectory_file) == 0)
    {
        static const char *search_paths[] = {
            "my_track_raceline.csv",
            "trajectories/my_track_raceline.csv",
            "f1tenth_planning/trajectories/my_track_raceline.csv",
            "../f1tenth_planning/trajectories/my_track_raceline.csv",
            "../trajectories/my_track_raceline.csv",
            NULL
        };
        for (int i = 0; search_paths[i] != NULL; i++)
        {
            FILE *test = fopen(search_paths[i], "r");
            if (test != NULL)
            {
                fclose(test);
                g_trajectory_file = search_paths[i];
                printf("[MPC] Auto-found trajectory: %s\n", g_trajectory_file);
                break;
            }
        }
    }

    if (g_trajectory_file != NULL && strlen(g_trajectory_file) > 0)
    {
        if (load_trajectory_from_csv(g_trajectory_file))
            printf("[MPC] Trajectory loaded successfully\n");
        else
            printf("[MPC] WARNING: Failed to load trajectory, using straight-line fallback\n");
    }
    else
    {
        printf("[MPC] WARNING: No trajectory file specified. Use arg or MPC_TRAJECTORY_FILE env.\n");
        printf("[MPC] Searched: my_track_raceline.csv, trajectories/, f1tenth_planning/trajectories/, ../f1tenth_planning/trajectories/\n");
        printf("[MPC] Using straight-line fallback.\n");
    }

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

    /* ===== Subscription 3: /imu/filtered_angular_velocity (std_msgs/Float64) ===== */
    rcl_subscription_t imu_sub = rcl_get_zero_initialized_subscription();
    rcl_subscription_options_t imu_sub_opts = rcl_subscription_get_default_options();
    imu_sub_opts.qos = qos_reliable_10;

    rc = rcl_subscription_init(&imu_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
        g_imu_topic, &imu_sub_opts);
    if (rc != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: imu subscription: %s\n", rcl_get_error_string().str);
        return 1;
    }
    printf("[ROS2] Subscribed to %s (Reliable, KeepLast(10))\n", g_imu_topic);

    /* ===== Subscription 4: Map-frame pose (geometry_msgs/PoseWithCovarianceStamped) ==
     * Default: /ekf_pose from the EKF localization node (fuses odom + AMCL).
     * Override with MPC_AMCL_TOPIC env var (e.g. /amcl_pose for raw AMCL).
     * Reliable QoS to match the EKF publisher. */
    rmw_qos_profile_t qos_amcl = rmw_qos_profile_default;
    qos_amcl.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    qos_amcl.history     = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    qos_amcl.depth       = 10;

    rcl_subscription_t amcl_sub = rcl_get_zero_initialized_subscription();
    rcl_subscription_options_t amcl_sub_opts = rcl_subscription_get_default_options();
    amcl_sub_opts.qos = qos_amcl;

    rc = rcl_subscription_init(&amcl_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseWithCovarianceStamped),
        g_amcl_pose_topic, &amcl_sub_opts);
    if (rc != RCL_RET_OK)
    {
        /* Non-fatal: robot may not have localization running */
        fprintf(stderr, "[ROS2] WARNING: map-frame pose subscription failed (%s) — using odom\n",
                rcl_get_error_string().str);
        rcl_reset_error();
    }
    else
    {
        printf("[ROS2] Subscribed to %s (Reliable, KeepLast(10))\n", g_amcl_pose_topic);
    }

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

    /* ===== Timer: 200 Hz MPC computation ===== */
    rcl_clock_t steady_clock;
    rcl_allocator_t alloc = rcl_get_default_allocator();
    rc = rcl_steady_clock_init(&steady_clock, &alloc);
    if (rc != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: clock init: %s\n", rcl_get_error_string().str);
        return 1;
    }

    rcl_timer_t mpc_timer = rcl_get_zero_initialized_timer();
    int64_t timer_period_ns = (int64_t)(1e9 / g_control_rate_hz);  /* e.g., 5ms for 200Hz */

    /* Use rcl_timer_init (available on all Humble builds).
     * rcl_timer_init2 (adds autostart param) was only added in later patches
     * and is missing on some Jetson Humble installations. */
    rc = rcl_timer_init(
        &mpc_timer, &steady_clock, &ctx,
        timer_period_ns, &mpc_timer_callback,
        alloc);
    if (rc != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: timer init: %s\n", rcl_get_error_string().str);
        return 1;
    }
    printf("[ROS2] Timer created: %.0f Hz (%ld ns period)\n",
           g_control_rate_hz, (long)timer_period_ns);

    /* Initialize message buffers (pre-allocate all strings) */
    nav_msgs__msg__Odometry__init(&global_odometry_message_buffer);
    if (!preallocate_rosidl_string(&global_odometry_message_buffer.header.frame_id, 64) ||
        !preallocate_rosidl_string(&global_odometry_message_buffer.child_frame_id, 64))
    {
        fprintf(stderr, "[ROS2] ERROR: odom string alloc\n");
        return 1;
    }

    std_msgs__msg__Float64__init(&global_servo_message_buffer);
    std_msgs__msg__Float64__init(&global_imu_message_buffer);
    geometry_msgs__msg__PoseWithCovarianceStamped__init(&global_amcl_pose_buffer);

    ackermann_msgs__msg__AckermannDriveStamped__init(&global_drive_message_buffer);
    if (!preallocate_rosidl_string(&global_drive_message_buffer.header.frame_id, 64))
    {
        fprintf(stderr, "[ROS2] ERROR: drive header string alloc\n");
        return 1;
    }
    set_rosidl_string(&global_drive_message_buffer.header.frame_id, "base_link");

    /* Executor: 3 subscriptions + 1 timer */
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

    /* Add IMU subscription */
    rc = rclc_executor_add_subscription(&executor, &imu_sub,
        &global_imu_message_buffer, &imu_callback, ON_NEW_DATA);
    if (rc != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: add imu sub: %s\n", rcl_get_error_string().str);
        return 1;
    }

    /* Add AMCL pose subscription (non-fatal if not available) */
    rc = rclc_executor_add_subscription(&executor, &amcl_sub,
        &global_amcl_pose_buffer, &amcl_pose_callback, ON_NEW_DATA);
    if (rc != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] WARNING: add amcl_pose sub failed — Frenet will use odom frame\n");
        rcl_reset_error();
    }

    /* Add MPC timer */
    rc = rclc_executor_add_timer(&executor, &mpc_timer);
    if (rc != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: add timer: %s\n", rcl_get_error_string().str);
        return 1;
    }

    printf("[ROS2] Executor ready (4 subs + 1 timer)\n");  /* odom, servo, imu, amcl */
    printf("\n[MPC] Spinning... (waiting for odometry on %s)\n\n", g_odom_topic);

    rclc_executor_spin(&executor);

    /* Cleanup */
    printf("\n[ROS2] Shutting down...\n");
    rclc_executor_fini(&executor);
    nav_msgs__msg__Odometry__fini(&global_odometry_message_buffer);
    std_msgs__msg__Float64__fini(&global_servo_message_buffer);
    std_msgs__msg__Float64__fini(&global_imu_message_buffer);
    ackermann_msgs__msg__AckermannDriveStamped__fini(&global_drive_message_buffer);
    rcl_ret_t cleanup_rc;
    cleanup_rc = rcl_timer_fini(&mpc_timer); (void)cleanup_rc;
    cleanup_rc = rcl_subscription_fini(&odom_sub, &node); (void)cleanup_rc;
    cleanup_rc = rcl_subscription_fini(&servo_sub, &node); (void)cleanup_rc;
    cleanup_rc = rcl_subscription_fini(&imu_sub, &node); (void)cleanup_rc;
    cleanup_rc = rcl_subscription_fini(&amcl_sub, &node); (void)cleanup_rc;
    geometry_msgs__msg__PoseWithCovarianceStamped__fini(&global_amcl_pose_buffer);
    cleanup_rc = rcl_publisher_fini(&global_control_publisher, &node); (void)cleanup_rc;
    cleanup_rc = rcl_steady_clock_fini(&steady_clock); (void)cleanup_rc;
    cleanup_rc = rcl_node_fini(&node); (void)cleanup_rc;
    cleanup_rc = rcl_context_fini(&ctx); (void)cleanup_rc;

    printf("[ROS2] Cleanup complete\n");
    return 0;
}
