#define _POSIX_C_SOURCE 199309L

/*******************************************************************************
 * mpcc_hardware_node.c -- ROS2 Node for Lifted ODE MPCC on Real Hardware
 *
 * Frenet primary (s, n, alpha) + Cartesian redundant (X, Y, psi) formulation.
 * Subscribes to hardware odometry/localization and publishes Ackermann commands.
 *
 * State [9]: s, n, alpha, vx, vy, omega, X, Y, psi
 * Control [3]: delta, a_x, v_theta
 ******************************************************************************/

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <nav_msgs/msg/path.h>
#include <ackermann_msgs/msg/ackermann_drive_stamped.h>
#include <geometry_msgs/msg/pose_array.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.h>
#include <std_msgs/msg/float64.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include <rcutils/allocator.h>

#include "mpcc_types.h"
#include "mpcc.h"

/* -------------------------------------------------------------------------- */
/* Runtime Configuration                                                       */
/* -------------------------------------------------------------------------- */

#define MPCC_DEFAULT_CONTROL_PERIOD_MS 50U
#define MPCC_EXECUTOR_HANDLES 5

static const char *g_odom_topic = "/ego_racecar/odom";
static const char *g_pose_topic = "/ekf_pose";
static const char *g_imu_topic = "/imu/filtered_angular_velocity";
static const char *g_servo_topic = "/sensors/servo_position_command";
static const char *g_drive_topic = "/drive";

static const char *g_trajectory_file = NULL;

static double g_watchdog_timeout_sec = 0.5;
static int g_verbose = 0;

/* Solver-derived values used to map MPCC acceleration to a velocity command. */
static double g_solver_dt_sec = 0.05;
static double g_vx_max_mps = 8.0;
static double g_vx_min_cmd = 1.0;  /* Minimum velocity command [m/s] */

/* Hardware safety: clamp acceleration to prevent violent braking */
static double g_ax_min_hardware = -3.0;

/* -------------------------------------------------------------------------- */
/* VESC Servo Conversion Parameters                                            */
/* -------------------------------------------------------------------------- */
/* Forward (angle -> servo): servo_val = gain * corrected + offset
 * where corrected = c2*|d|^2 + c1*|d| + c0 (sign-preserved)
 * Inverse: solve quadratic to recover d from servo_val */
#define STEERING_TO_SERVO_GAIN   (-0.7284)
#define STEERING_TO_SERVO_OFFSET 0.55
#define STEERING_CORRECTION_C2   0.589566
#define STEERING_CORRECTION_C1   0.918061
#define STEERING_CORRECTION_C0   0.001490
#define STEERING_RATE_LIMIT      2.849

/* -------------------------------------------------------------------------- */
/* Global State                                                                */
/* -------------------------------------------------------------------------- */

static VehicleState_t g_vehicle_state;
static fixed_point_t g_current_s = 0;
static MPCCReferencePath_t g_reference_path;

static int g_have_reference = 0;
static int g_have_odom = 0;
static int g_have_pose = 0;
static int g_using_map_pose = 0;

static double g_latest_vx_mps = 0.0;
static double g_latest_vy_mps = 0.0;
static double g_latest_omega = 0.0;
static struct timespec g_last_odom_time = {0, 0};
static uint32_t g_solve_count = 0;

/* Measured control loop timing for cross-call scaling */
static struct timespec g_prev_solve_time = {0, 0};
static double g_control_dt_filtered = 0.005;  /* Initial guess: 200 Hz */

/* Servo feedback tracking */
static double g_actual_steering_angle = 0.0;
static int g_use_steering_feedback = 0;

/* Previous control command (kept on solver failure) */
static float g_prev_delta_cmd = 0.0f;
static float g_prev_speed_cmd = 0.0f;
static float g_prev_ax_cmd = 0.0f;

/* ROS entities */
static rcl_subscription_t g_odom_sub;
static rcl_subscription_t g_pose_sub;
static rcl_subscription_t g_imu_sub;
static rcl_subscription_t g_servo_sub;

static rcl_publisher_t g_drive_pub;
static rcl_publisher_t g_predicted_path_pub;
static rcl_publisher_t g_raceline_pub;

/* Message buffers */
static nav_msgs__msg__Odometry g_odom_msg;
static geometry_msgs__msg__PoseWithCovarianceStamped g_pose_msg;
static std_msgs__msg__Float64 g_imu_msg;
static std_msgs__msg__Float64 g_servo_msg;
static ackermann_msgs__msg__AckermannDriveStamped g_drive_msg;

/* -------------------------------------------------------------------------- */
/* Helpers                                                                     */
/* -------------------------------------------------------------------------- */

static double quat_to_yaw(double qx, double qy, double qz, double qw)
{
    const double siny = 2.0 * (qw * qz + qx * qy);
    const double cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
    return atan2(siny, cosy);
}

static double timespec_diff_sec(const struct timespec *start,
                                const struct timespec *end)
{
    return (double)(end->tv_sec - start->tv_sec)
         + (double)(end->tv_nsec - start->tv_nsec) * 1e-9;
}

/* -------------------------------------------------------------------------- */
/* Trajectory Loading                                                          */
/* -------------------------------------------------------------------------- */

static int load_trajectory_csv(const char *file_path, MPCCReferencePath_t *path)
{
    FILE *file = fopen(file_path, "r");
    if (file == NULL)
    {
        fprintf(stderr, "[MPCC] ERROR: cannot open trajectory file: %s\n", file_path);
        return 0;
    }

    char line[512];
    path->num_points = 0;

    while (fgets(line, sizeof(line), file) != NULL)
    {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r')
        {
            continue;
        }

        if (path->num_points >= MPCC_MAX_PATH_POINTS)
        {
            printf("[MPCC] WARNING: trajectory truncated at %d points\n",
                   MPCC_MAX_PATH_POINTS);
            break;
        }

        double s, x, y, psi, kappa, vx, ax;
        double d_left = 0.5;
        double d_right = 0.5;

        const int parsed = sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                                  &s, &x, &y, &psi, &kappa, &vx, &ax,
                                  &d_left, &d_right);

        if (parsed < 5)
        {
            continue;
        }

        MPCCPathPoint_t *pt = &path->points[path->num_points];
        pt->s_ref = float_to_fp((float)s);
        pt->x_ref = float_to_fp((float)x);
        pt->y_ref = float_to_fp((float)y);
        pt->phi_ref = float_to_fp((float)psi);
        pt->kappa_ref = float_to_fp((float)kappa);
        pt->vx_ref = (parsed >= 6) ? float_to_fp((float)vx) : FP_CONST(3.0);

        {
            const float car_half_width = 0.155f;
            if (parsed >= 9)
            {
                float left_bound = (float)d_left - car_half_width;
                float right_bound = (float)d_right - car_half_width;
                if (left_bound < 0.05f) left_bound = 0.05f;
                if (right_bound < 0.05f) right_bound = 0.05f;
                pt->left_bound = float_to_fp(left_bound);
                pt->right_bound = float_to_fp(right_bound);
            }
            else
            {
                pt->left_bound = FP_CONST(0.5);
                pt->right_bound = FP_CONST(0.5);
            }
        }

        (void)ax;
        path->num_points++;
    }

    fclose(file);

    if (path->num_points < 2)
    {
        fprintf(stderr, "[MPCC] ERROR: only %d points loaded from %s\n",
                path->num_points, file_path);
        return 0;
    }

    path->total_length = path->points[path->num_points - 1].s_ref;
    path->is_closed = 1;

    printf("[MPCC] Loaded %d points from %s (track length %.1f m)\n",
           path->num_points, file_path, fp_to_float(path->total_length));

    return 1;
}

static void publish_raceline(const MPCCReferencePath_t *path)
{
    nav_msgs__msg__Path path_msg;
    nav_msgs__msg__Path__init(&path_msg);

    rcutils_allocator_t alloc = rcutils_get_default_allocator();

    char *frame = (char *)alloc.allocate(8, alloc.state);
    if (frame != NULL)
    {
        memcpy(frame, "map", 4);
        path_msg.header.frame_id.data = frame;
        path_msg.header.frame_id.size = 3;
        path_msg.header.frame_id.capacity = 8;
    }

    const uint16_t n = path->num_points;
    geometry_msgs__msg__PoseStamped *poses =
        (geometry_msgs__msg__PoseStamped *)alloc.allocate(
            (size_t)n * sizeof(geometry_msgs__msg__PoseStamped), alloc.state);

    if (poses == NULL)
    {
        fprintf(stderr, "[MPCC] WARNING: failed to allocate raceline poses\n");
        if (frame != NULL)
        {
            alloc.deallocate(frame, alloc.state);
        }
        return;
    }

    for (uint16_t i = 0; i < n; ++i)
    {
        geometry_msgs__msg__PoseStamped__init(&poses[i]);

        char *pose_frame = (char *)alloc.allocate(8, alloc.state);
        if (pose_frame != NULL)
        {
            memcpy(pose_frame, "map", 4);
            poses[i].header.frame_id.data = pose_frame;
            poses[i].header.frame_id.size = 3;
            poses[i].header.frame_id.capacity = 8;
        }

        const float x = fp_to_float(path->points[i].x_ref);
        const float y = fp_to_float(path->points[i].y_ref);
        const float psi = fp_to_float(path->points[i].phi_ref);

        poses[i].pose.position.x = x;
        poses[i].pose.position.y = y;
        poses[i].pose.position.z = 0.0;
        poses[i].pose.orientation.x = 0.0;
        poses[i].pose.orientation.y = 0.0;
        poses[i].pose.orientation.z = sin(0.5f * psi);
        poses[i].pose.orientation.w = cos(0.5f * psi);
    }

    path_msg.poses.data = poses;
    path_msg.poses.size = n;
    path_msg.poses.capacity = n;

    {
        const rcl_ret_t rc = rcl_publish(&g_raceline_pub, &path_msg, NULL);
        if (rc != RCL_RET_OK)
        {
            fprintf(stderr, "[MPCC] WARNING: failed to publish /mpcc/raceline: %s\n",
                    rcl_get_error_string().str);
            rcl_reset_error();
        }
    }

    for (uint16_t i = 0; i < n; ++i)
    {
        if (poses[i].header.frame_id.data != NULL)
        {
            alloc.deallocate(poses[i].header.frame_id.data, alloc.state);
        }
    }
    alloc.deallocate(poses, alloc.state);

    if (frame != NULL)
    {
        alloc.deallocate(frame, alloc.state);
    }
}

/* -------------------------------------------------------------------------- */
/* ROS Callbacks                                                               */
/* -------------------------------------------------------------------------- */

static void odom_callback(const void *msg_in)
{
    if (msg_in == NULL)
    {
        return;
    }

    const nav_msgs__msg__Odometry *msg = (const nav_msgs__msg__Odometry *)msg_in;

    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;
    const double heading = quat_to_yaw(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    const double vx = msg->twist.twist.linear.x;
    const double vy = msg->twist.twist.linear.y;
    const double omega = msg->twist.twist.angular.z;

    if (!g_using_map_pose)
    {
        g_vehicle_state.pos_x = float_to_fp((float)x);
        g_vehicle_state.pos_y = float_to_fp((float)y);
        g_vehicle_state.heading = float_to_fp((float)heading);
        g_have_pose = 1;
    }

    g_vehicle_state.long_vel = float_to_fp((float)vx);
    g_vehicle_state.lat_vel = float_to_fp((float)vy);
    g_vehicle_state.yaw_rate = float_to_fp((float)omega);

    g_latest_vx_mps = vx;
    g_latest_vy_mps = vy;
    g_latest_omega = omega;
    g_have_odom = 1;

    clock_gettime(CLOCK_MONOTONIC, &g_last_odom_time);
}

static void pose_callback(const void *msg_in)
{
    if (msg_in == NULL)
    {
        return;
    }

    const geometry_msgs__msg__PoseWithCovarianceStamped *msg =
        (const geometry_msgs__msg__PoseWithCovarianceStamped *)msg_in;

    const double heading = quat_to_yaw(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    g_vehicle_state.pos_x = float_to_fp((float)msg->pose.pose.position.x);
    g_vehicle_state.pos_y = float_to_fp((float)msg->pose.pose.position.y);
    g_vehicle_state.heading = float_to_fp((float)heading);

    if (!g_using_map_pose)
    {
        printf("[MPCC] Map-frame pose received on %s; switching to map pose\n",
               g_pose_topic);
        g_using_map_pose = 1;
    }

    g_have_pose = 1;

    /* ------- EKF-driven control: run solver on every new pose ------- */

    if (!g_have_reference || !g_have_odom)
    {
        return;
    }

    /* Watchdog: check for stale odometry */
    {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);

        if ((g_last_odom_time.tv_sec != 0 || g_last_odom_time.tv_nsec != 0)
            && timespec_diff_sec(&g_last_odom_time, &now) > g_watchdog_timeout_sec)
        {
            if (g_verbose)
            {
                printf("[MPCC] WATCHDOG: stale odometry, publishing zero command\n");
            }
            g_drive_msg.drive.steering_angle = 0.0f;
            g_drive_msg.drive.speed = 0.0f;
            g_drive_msg.drive.acceleration = 0.0f;
            { rcl_ret_t rc_ = rcl_publish(&g_drive_pub, &g_drive_msg, NULL); (void)rc_; }
            return;
        }
    }

    MPCCState_t mpcc_state = mpcc_state_from_vehicle_state(&g_vehicle_state, g_current_s);
    g_current_s = mpcc_state.s;

    /* Measure actual control loop dt and update cross-call rate scale */
    {
        struct timespec solve_now;
        clock_gettime(CLOCK_MONOTONIC, &solve_now);
        if (g_prev_solve_time.tv_sec != 0 || g_prev_solve_time.tv_nsec != 0)
        {
            double dt_actual = timespec_diff_sec(&g_prev_solve_time, &solve_now);
            if (dt_actual > 0.0005 && dt_actual < 0.5)  /* sanity: 2 Hz–2 kHz */
            {
                g_control_dt_filtered = 0.9 * g_control_dt_filtered + 0.1 * dt_actual;
                double prediction_dt = (double)g_solver_dt_sec;
                if (prediction_dt > 0.0)
                {
                    MPCCConfiguration_t cfg = mpcc_get_configuration();
                    cfg.cross_call_rate_scale = float_to_fp(
                        (float)(g_control_dt_filtered / prediction_dt));
                    mpcc_set_configuration(&cfg);
                }
            }
        }
        g_prev_solve_time = solve_now;
    }

    MPCCResult_t result;
    const MPCCStatus_t status = mpcc_compute_control(&mpcc_state, &result);

    g_solve_count++;

    if (status != MPCC_STATUS_SUCCESS && status != MPCC_STATUS_MAX_ITERATIONS)
    {
        if (g_verbose || g_solve_count <= 20 || (g_solve_count % 10U) == 0U)
        {
            fprintf(stderr,
                    "[MPCC %3u] solver failed (status=%d), keeping previous command (d=%.3f v=%.2f)\n",
                    g_solve_count, (int)status, g_prev_delta_cmd, g_prev_speed_cmd);
        }
        /* Keep the previous command instead of commanding zero */
        g_drive_msg.drive.steering_angle = g_prev_delta_cmd;
        g_drive_msg.drive.speed = g_prev_speed_cmd;
        g_drive_msg.drive.acceleration = g_prev_ax_cmd;
        { rcl_ret_t rc_ = rcl_publish(&g_drive_pub, &g_drive_msg, NULL); (void)rc_; }
        return;
    }

    float a_x_cmd = fp_to_float(result.optimal_control.a_x);
    const float delta_cmd = fp_to_float(result.optimal_control.delta);
    const float v_theta_cmd = fp_to_float(result.optimal_control.v_theta);

    /* Clamp acceleration for hardware safety */
    if (a_x_cmd < (float)g_ax_min_hardware)
    {
        a_x_cmd = (float)g_ax_min_hardware;
    }

    double v_cmd = g_latest_vx_mps + (double)a_x_cmd * g_solver_dt_sec;
    if (v_cmd < 0.0)
    {
        v_cmd = 0.0;
    }
    /* Apply minimum velocity floor only when the solver wants to accelerate,
     * so the car can still brake/stop when the solver commands a_x < 0. */
    if (a_x_cmd >= 0.0f && v_cmd < g_vx_min_cmd)
    {
        v_cmd = g_vx_min_cmd;
    }
    if (v_cmd > g_vx_max_mps)
    {
        v_cmd = g_vx_max_mps;
    }

    /* Store for next iteration (keep-prev on failure) */
    g_prev_delta_cmd = delta_cmd;
    g_prev_speed_cmd = (float)v_cmd;
    g_prev_ax_cmd = a_x_cmd;

    g_drive_msg.header.stamp.sec = 0;
    g_drive_msg.header.stamp.nanosec = 0;
    g_drive_msg.drive.steering_angle = delta_cmd;
    g_drive_msg.drive.steering_angle_velocity = 0.0f;
    g_drive_msg.drive.speed = (float)v_cmd;
    g_drive_msg.drive.acceleration = a_x_cmd;
    g_drive_msg.drive.jerk = 0.0f;

    {
        const rcl_ret_t rc = rcl_publish(&g_drive_pub, &g_drive_msg, NULL);
        if (rc != RCL_RET_OK)
        {
            fprintf(stderr, "[MPCC] WARNING: failed to publish drive command: %s\n",
                    rcl_get_error_string().str);
            rcl_reset_error();
        }
    }

    if (g_solve_count <= 20 || (g_solve_count % 10U) == 0U)
    {
        fprintf(stderr,
                "[MPCC %3u] s=%.2f x=%.2f y=%.2f psi=%.2f vx=%.2f | d=%.4f ax=%.3f vt=%.3f vcmd=%.2f | st=%d it=%u\n",
                g_solve_count,
                fp_to_float(mpcc_state.s),
                fp_to_float(mpcc_state.X),
                fp_to_float(mpcc_state.Y),
                fp_to_float(mpcc_state.psi),
                fp_to_float(mpcc_state.vx),
                delta_cmd,
                a_x_cmd,
                v_theta_cmd,
                (float)v_cmd,
                (int)status,
                result.admm_iterations);
        fflush(stderr);
    }
}

/* -------------------------------------------------------------------------- */
/* IMU & Servo Callbacks                                                       */
/* -------------------------------------------------------------------------- */

static void imu_callback(const void *msg_in)
{
    if (msg_in == NULL)
    {
        return;
    }

    const std_msgs__msg__Float64 *msg = (const std_msgs__msg__Float64 *)msg_in;
    g_vehicle_state.yaw_rate = float_to_fp((float)msg->data);
}

static void servo_callback(const void *msg_in)
{
    if (msg_in == NULL)
    {
        return;
    }

    const std_msgs__msg__Float64 *msg = (const std_msgs__msg__Float64 *)msg_in;
    const double servo_val = msg->data;

    /* Invert the forward mapping: servo_val = GAIN * corrected_angle + OFFSET
     * => corrected_angle = (servo_val - OFFSET) / GAIN */
    const double corrected = (servo_val - STEERING_TO_SERVO_OFFSET) / STEERING_TO_SERVO_GAIN;

    /* Invert the polynomial correction: corrected = C2*|d|^2 + C1*|d| + C0
     * Solve quadratic: C2*a^2 + C1*a + (C0 - |corrected|) = 0
     * where a = |d|  */
    const double abs_corrected = fabs(corrected);
    const double discriminant =
        STEERING_CORRECTION_C1 * STEERING_CORRECTION_C1
        - 4.0 * STEERING_CORRECTION_C2 * (STEERING_CORRECTION_C0 - abs_corrected);

    if (discriminant >= 0.0)
    {
        const double abs_delta =
            (-STEERING_CORRECTION_C1 + sqrt(discriminant)) / (2.0 * STEERING_CORRECTION_C2);
        g_actual_steering_angle = (corrected >= 0.0) ? abs_delta : -abs_delta;
        g_use_steering_feedback = 1;
    }
}

/* -------------------------------------------------------------------------- */
/* Runtime Setup                                                               */
/* -------------------------------------------------------------------------- */

static void read_runtime_environment(void)
{
    const char *value;

    if ((value = getenv("MPCC_ODOM_TOPIC")) != NULL && value[0] != '\0')
    {
        g_odom_topic = value;
    }
    if ((value = getenv("MPCC_DRIVE_TOPIC")) != NULL && value[0] != '\0')
    {
        g_drive_topic = value;
    }
    if ((value = getenv("MPCC_IMU_TOPIC")) != NULL && value[0] != '\0')
    {
        g_imu_topic = value;
    }
    if ((value = getenv("MPCC_POSE_TOPIC")) != NULL && value[0] != '\0')
    {
        g_pose_topic = value;
    }
    else if ((value = getenv("MPCC_AMCL_TOPIC")) != NULL && value[0] != '\0')
    {
        /* Backward-compatible alias for map-frame pose topic. */
        g_pose_topic = value;
    }

    if ((value = getenv("MPCC_SERVO_TOPIC")) != NULL && value[0] != '\0')
    {
        g_servo_topic = value;
    }

    if ((value = getenv("MPCC_AX_MIN_HARDWARE")) != NULL)
    {
        const double val = atof(value);
        if (val < 0.0 && val >= -20.0)
        {
            g_ax_min_hardware = val;
        }
    }

    if ((value = getenv("MPCC_WATCHDOG_TIMEOUT")) != NULL)
    {
        const double timeout = atof(value);
        if (timeout > 0.0 && timeout <= 5.0)
        {
            g_watchdog_timeout_sec = timeout;
        }
    }

    if ((value = getenv("MPCC_VERBOSE")) != NULL)
    {
        g_verbose = atoi(value);
    }

    if ((value = getenv("MPCC_TRAJECTORY_FILE")) != NULL && value[0] != '\0')
    {
        g_trajectory_file = value;
    }

    if ((value = getenv("MPCC_VX_MIN_CMD")) != NULL)
    {
        const double val = atof(value);
        if (val >= 0.0 && val <= 5.0)
        {
            g_vx_min_cmd = val;
        }
    }
}

static void configure_mpcc_from_environment(void)
{
    mpcc_initialize();
    MPCCConfiguration_t cfg = mpcc_get_configuration();

    const char *v;
    if ((v = getenv("Q_N")) != NULL) cfg.weight_contouring = float_to_fp((float)atof(v));
    if ((v = getenv("Q_CONTOURING")) != NULL) cfg.weight_contouring = float_to_fp((float)atof(v));
    if ((v = getenv("Q_LAG")) != NULL) cfg.weight_lag = float_to_fp((float)atof(v));
    if ((v = getenv("Q_ALPHA")) != NULL) cfg.weight_lag = float_to_fp((float)atof(v));
    if ((v = getenv("Q_PROGRESS")) != NULL) cfg.weight_progress = float_to_fp((float)atof(v));
    if ((v = getenv("Q_VX")) != NULL) cfg.weight_vx = float_to_fp((float)atof(v));
    if ((v = getenv("VX_REF")) != NULL) cfg.vx_ref = float_to_fp((float)atof(v));
    if ((v = getenv("Q_VY")) != NULL) cfg.weight_vy = float_to_fp((float)atof(v));
    if ((v = getenv("Q_OMEGA")) != NULL) cfg.weight_omega = float_to_fp((float)atof(v));
    if ((v = getenv("R_DELTA")) != NULL) cfg.weight_delta = float_to_fp((float)atof(v));
    if ((v = getenv("R_AX")) != NULL) cfg.weight_ax = float_to_fp((float)atof(v));
    if ((v = getenv("R_VTHETA")) != NULL) cfg.weight_v_theta = float_to_fp((float)atof(v));
    if ((v = getenv("W_DELTA_RATE")) != NULL) cfg.weight_delta_rate = float_to_fp((float)atof(v));
    if ((v = getenv("W_AX_RATE")) != NULL) cfg.weight_ax_rate = float_to_fp((float)atof(v));
    if ((v = getenv("W_VTHETA_RATE")) != NULL) cfg.weight_v_theta_rate = float_to_fp((float)atof(v));
    if ((v = getenv("Q_N_TERM")) != NULL) cfg.weight_contouring_terminal = float_to_fp((float)atof(v));
    if ((v = getenv("Q_CONTOURING_TERM")) != NULL) cfg.weight_contouring_terminal = float_to_fp((float)atof(v));
    if ((v = getenv("Q_LAG_TERM")) != NULL) cfg.weight_lag_terminal = float_to_fp((float)atof(v));
    if ((v = getenv("Q_ALPHA_TERM")) != NULL) cfg.weight_lag_terminal = float_to_fp((float)atof(v));
    if ((v = getenv("Q_PROGRESS_TERM")) != NULL) cfg.weight_progress_terminal = float_to_fp((float)atof(v));
    if ((v = getenv("ADMM_RHO")) != NULL) cfg.admm_rho = float_to_fp((float)atof(v));
    if ((v = getenv("ADMM_MAX_ITER")) != NULL) cfg.admm_max_iterations = (uint16_t)atoi(v);
    if ((v = getenv("ADMM_TOL")) != NULL) cfg.admm_tolerance = float_to_fp((float)atof(v));
    if ((v = getenv("HORIZON")) != NULL) cfg.horizon_steps = (uint16_t)atoi(v);
    if ((v = getenv("DT")) != NULL) cfg.dt = float_to_fp((float)atof(v));
    if ((v = getenv("V_THETA_MAX")) != NULL) cfg.v_theta_max = float_to_fp((float)atof(v));
    if ((v = getenv("V_THETA_MIN")) != NULL) cfg.v_theta_min = float_to_fp((float)atof(v));
    if ((v = getenv("MU")) != NULL) cfg.mu = float_to_fp((float)atof(v));
    if ((v = getenv("C_SF")) != NULL) cfg.C_Sf = float_to_fp((float)atof(v));
    if ((v = getenv("C_SR")) != NULL) cfg.C_Sr = float_to_fp((float)atof(v));
    if ((v = getenv("AX_MAX")) != NULL) cfg.ax_max = float_to_fp((float)atof(v));
    if ((v = getenv("AX_MIN")) != NULL) cfg.ax_min = float_to_fp((float)atof(v));

    if ((v = getenv("MPCC_CROSS_CALL_SCALE")) != NULL)
        cfg.cross_call_rate_scale = float_to_fp((float)atof(v));

    mpcc_set_configuration(&cfg);

    /* If the user hasn't explicitly set the cross-call scale, auto-compute
     * from control rate and the (possibly overridden) prediction dt. */
    if (getenv("MPCC_CROSS_CALL_SCALE") == NULL)
    {
        float dt = fp_to_float(cfg.dt);
        if (dt > 0.0f)
            cfg.cross_call_rate_scale = float_to_fp(
                (float)(1.0 / (MPCC_CONTROL_RATE_HZ * (double)dt)));
        mpcc_set_configuration(&cfg);
    }

    g_solver_dt_sec = fp_to_float(cfg.dt);
    if (g_solver_dt_sec <= 0.0)
    {
        g_solver_dt_sec = 0.05;
    }

    g_vx_max_mps = fp_to_float(cfg.vx_max);
    if (g_vx_max_mps <= 0.0)
    {
        g_vx_max_mps = 8.0;
    }

    printf("[MPCC] Config: N=%d dt=%.3f Q_c=%.1f Q_l=%.1f Q_prog=%.1f R_delta=%.2f ax_min_hw=%.1f cross_call=%.4f\n",
           cfg.horizon_steps,
           fp_to_float(cfg.dt),
           fp_to_float(cfg.weight_contouring),
           fp_to_float(cfg.weight_lag),
           fp_to_float(cfg.weight_progress),
           fp_to_float(cfg.weight_delta),
           g_ax_min_hardware,
           fp_to_float(cfg.cross_call_rate_scale));
}

static const char *autodetect_trajectory_file(void)
{
    static const char *candidates[] = {
        "hardware_raceline.csv",
        "my_track_raceline.csv",
        "f1tenth_planning/trajectories/hardware_raceline.csv",
        "f1tenth_planning/trajectories/my_track_raceline.csv",
        "../f1tenth_planning/trajectories/hardware_raceline.csv",
        "../f1tenth_planning/trajectories/my_track_raceline.csv",
        "/ros2_ws/src/f1tenth_planning/trajectories/hardware_raceline.csv",
        "/ros2_ws/src/f1tenth_planning/trajectories/my_track_raceline.csv",
        NULL
    };

    for (int i = 0; candidates[i] != NULL; ++i)
    {
        FILE *probe = fopen(candidates[i], "r");
        if (probe != NULL)
        {
            fclose(probe);
            return candidates[i];
        }
    }

    return NULL;
}

/* -------------------------------------------------------------------------- */
/* Main                                                                        */
/* -------------------------------------------------------------------------- */

int main(int argc, const char *argv[])
{
    read_runtime_environment();
    configure_mpcc_from_environment();

    g_current_s = 0;

    if (argc >= 2 && argv[1] != NULL && argv[1][0] != '\0')
    {
        g_trajectory_file = argv[1];
    }
    if (g_trajectory_file == NULL)
    {
        g_trajectory_file = autodetect_trajectory_file();
    }

    if (g_trajectory_file == NULL)
    {
        fprintf(stderr,
                "[MPCC] FATAL: no trajectory file provided (arg or MPCC_TRAJECTORY_FILE)\n");
        return 1;
    }

    if (!load_trajectory_csv(g_trajectory_file, &g_reference_path))
    {
        fprintf(stderr, "[MPCC] FATAL: could not load reference path\n");
        return 1;
    }

    mpcc_set_reference_path(&g_reference_path);
    g_have_reference = 1;

    printf("[MPCC] Hardware topics: odom=%s pose=%s imu=%s servo=%s drive=%s\n",
           g_odom_topic, g_pose_topic, g_imu_topic, g_servo_topic, g_drive_topic);
    printf("[MPCC] EKF-driven mode | watchdog: %.0f ms | ax_min_hw: %.1f | trajectory: %s\n",
           g_watchdog_timeout_sec * 1000.0, g_ax_min_hardware, g_trajectory_file);

    rcl_allocator_t allocator = rcl_get_default_allocator();

    rclc_support_t support;
    if (rclc_support_init(&support, argc, argv, &allocator) != RCL_RET_OK)
    {
        fprintf(stderr, "[MPCC] FATAL: rclc_support_init failed\n");
        return 1;
    }

    rcl_node_t node;
    if (rclc_node_init_default(&node, "mpcc_hardware_node", "", &support) != RCL_RET_OK)
    {
        fprintf(stderr, "[MPCC] FATAL: rclc_node_init_default failed\n");
        return 1;
    }

    /* Raceline publisher with transient-local durability for rviz. */
    {
        rcl_publisher_options_t pub_opts = rcl_publisher_get_default_options();
        pub_opts.qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
        pub_opts.qos.depth = 1;
        pub_opts.qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;

        const rcl_ret_t rc = rcl_publisher_init(
            &g_raceline_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Path),
            "/mpcc/raceline",
            &pub_opts);
        if (rc != RCL_RET_OK)
        {
            fprintf(stderr, "[MPCC] WARNING: failed to init /mpcc/raceline: %s\n",
                    rcl_get_error_string().str);
            rcl_reset_error();
        }
        else
        {
            publish_raceline(&g_reference_path);
            printf("[MPCC] Published raceline on /mpcc/raceline\n");
        }
    }

    if (rclc_subscription_init_default(
            &g_odom_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
            g_odom_topic) != RCL_RET_OK)
    {
        fprintf(stderr, "[MPCC] FATAL: odom subscription init failed\n");
        return 1;
    }

    if (rclc_subscription_init_default(
            &g_pose_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseWithCovarianceStamped),
            g_pose_topic) != RCL_RET_OK)
    {
        fprintf(stderr, "[MPCC] FATAL: pose subscription init failed\n");
        return 1;
    }

    if (rclc_subscription_init_default(
            &g_imu_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
            g_imu_topic) != RCL_RET_OK)
    {
        fprintf(stderr, "[MPCC] FATAL: imu subscription init failed\n");
        return 1;
    }

    int g_servo_sub_ok = 0;
    if (rclc_subscription_init_default(
            &g_servo_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
            g_servo_topic) != RCL_RET_OK)
    {
        fprintf(stderr, "[MPCC] WARNING: servo subscription init failed (steering feedback disabled)\n");
        rcl_reset_error();
    }
    else
    {
        g_servo_sub_ok = 1;
    }

    if (rclc_publisher_init_default(
            &g_drive_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(ackermann_msgs, msg, AckermannDriveStamped),
            g_drive_topic) != RCL_RET_OK)
    {
        fprintf(stderr, "[MPCC] FATAL: drive publisher init failed\n");
        return 1;
    }

    if (rclc_publisher_init_default(
            &g_predicted_path_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseArray),
            "/mpcc/predicted_path") != RCL_RET_OK)
    {
        fprintf(stderr, "[MPCC] WARNING: predicted path publisher init failed\n");
        rcl_reset_error();
    }

    /* Initialize message buffers used by the executor. */
    nav_msgs__msg__Odometry__init(&g_odom_msg);
    geometry_msgs__msg__PoseWithCovarianceStamped__init(&g_pose_msg);
    std_msgs__msg__Float64__init(&g_imu_msg);
    std_msgs__msg__Float64__init(&g_servo_msg);
    ackermann_msgs__msg__AckermannDriveStamped__init(&g_drive_msg);

    {
        rcutils_allocator_t alloc = rcutils_get_default_allocator();

        char *drive_frame = (char *)alloc.allocate(16, alloc.state);
        if (drive_frame != NULL)
        {
            memcpy(drive_frame, "base_link", 10);
            g_drive_msg.header.frame_id.data = drive_frame;
            g_drive_msg.header.frame_id.size = 9;
            g_drive_msg.header.frame_id.capacity = 16;
        }

        char *odom_header = (char *)alloc.allocate(64, alloc.state);
        if (odom_header != NULL)
        {
            odom_header[0] = '\0';
            g_odom_msg.header.frame_id.data = odom_header;
            g_odom_msg.header.frame_id.size = 0;
            g_odom_msg.header.frame_id.capacity = 64;
        }

        char *odom_child = (char *)alloc.allocate(64, alloc.state);
        if (odom_child != NULL)
        {
            odom_child[0] = '\0';
            g_odom_msg.child_frame_id.data = odom_child;
            g_odom_msg.child_frame_id.size = 0;
            g_odom_msg.child_frame_id.capacity = 64;
        }

        char *pose_header = (char *)alloc.allocate(64, alloc.state);
        if (pose_header != NULL)
        {
            pose_header[0] = '\0';
            g_pose_msg.header.frame_id.data = pose_header;
            g_pose_msg.header.frame_id.size = 0;
            g_pose_msg.header.frame_id.capacity = 64;
        }
    }

    rclc_executor_t executor;
    if (rclc_executor_init(&executor,
                           &support.context,
                           MPCC_EXECUTOR_HANDLES,
                           &allocator) != RCL_RET_OK)
    {
        fprintf(stderr, "[MPCC] FATAL: executor init failed\n");
        return 1;
    }

    if (rclc_executor_add_subscription(
            &executor,
            &g_odom_sub,
            &g_odom_msg,
            &odom_callback,
            ON_NEW_DATA) != RCL_RET_OK)
    {
        fprintf(stderr, "[MPCC] FATAL: add odom subscription failed\n");
        return 1;
    }

    if (rclc_executor_add_subscription(
            &executor,
            &g_pose_sub,
            &g_pose_msg,
            &pose_callback,
            ON_NEW_DATA) != RCL_RET_OK)
    {
        fprintf(stderr, "[MPCC] FATAL: add pose subscription failed\n");
        return 1;
    }

    if (rclc_executor_add_subscription(
            &executor,
            &g_imu_sub,
            &g_imu_msg,
            &imu_callback,
            ON_NEW_DATA) != RCL_RET_OK)
    {
        fprintf(stderr, "[MPCC] FATAL: add imu subscription failed\n");
        return 1;
    }

    if (g_servo_sub_ok)
    {
        if (rclc_executor_add_subscription(
                &executor,
                &g_servo_sub,
                &g_servo_msg,
                &servo_callback,
                ON_NEW_DATA) != RCL_RET_OK)
        {
            fprintf(stderr, "[MPCC] WARNING: add servo subscription failed (steering feedback disabled)\n");
            rcl_reset_error();
            g_servo_sub_ok = 0;
        }
    }

    printf("[MPCC] Spinning hardware node...\n");
    rclc_executor_spin(&executor);

    /* Cleanup */
    {
        rcl_ret_t rc_cleanup;
        rc_cleanup = rcl_subscription_fini(&g_odom_sub, &node);
        rc_cleanup = rcl_subscription_fini(&g_pose_sub, &node);
        rc_cleanup = rcl_subscription_fini(&g_imu_sub, &node);
        rc_cleanup = rcl_subscription_fini(&g_servo_sub, &node);
        rc_cleanup = rcl_publisher_fini(&g_drive_pub, &node);
        rc_cleanup = rcl_publisher_fini(&g_predicted_path_pub, &node);
        rc_cleanup = rcl_publisher_fini(&g_raceline_pub, &node);
        (void)rc_cleanup;
    }

    rclc_executor_fini(&executor);
    {
        rcl_ret_t rc_cleanup = rcl_node_fini(&node);
        (void)rc_cleanup;
    }
    (void)rclc_support_fini(&support);

    return 0;
}
