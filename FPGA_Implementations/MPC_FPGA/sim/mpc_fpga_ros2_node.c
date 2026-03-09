/**
 * @file mpc_fpga_ros2_node.c
 * @brief FPGA MPC ROS2 Node for F1/10th Simulator Integration
 *
 * Implements ROS2 node using rclc (C client library) for Jazzy.
 * Subscribes to odometry, runs the FPGA MPC solver (C simulation of HLS),
 * publishes control commands.
 *
 * This node calls the FPGA top-level function mpc_fpga_top() which uses
 * a mode-based interface:
 *   Mode 0: Compute MPC control from vehicle state
 *   Mode 1: Load one waypoint into internal BRAM
 *   Mode 2: Finalize trajectory (set total count)
 *
 * IMPORTANT: This node is a transparent bridge between the simulator and the
 * FPGA MPC solver. Nothing in this file may alter, clamp, bias, or
 * post-process the control output returned by mpc_fpga_top().
 *
 * Topics:
 *   Subscribe: /ego_racecar/ground_truth (nav_msgs/Odometry)
 *              /ego_racecar/collision     (std_msgs/Bool)
 *   Publish:   /drive          (ackermann_msgs/AckermannDriveStamped)
 *              /mpc/reference_path   (nav_msgs/Path)
 *              /mpc/trajectory_path  (nav_msgs/Path)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <signal.h>

/* ROS2 C Client Library Headers */
#include "rcl/rcl.h"
#include "rcl/error_handling.h"
#include "rclc/rclc.h"
#include "rclc/executor.h"
#include "rosidl_runtime_c/string_functions.h"
#include "rcutils/allocator.h"

/* ROS2 Message Types */
#include "nav_msgs/msg/odometry.h"
#include "nav_msgs/msg/path.h"
#include "ackermann_msgs/msg/ackermann_drive_stamped.h"
#include "geometry_msgs/msg/pose_stamped.h"
#include "std_msgs/msg/bool.h"

/* FPGA MPC Headers */
#include "mpc_fpga_types.h"
#include "fp_math_hls.h"

/* Forward declaration of the FPGA top-level function */
extern void mpc_fpga_top(
    int mode, int wp_index,
    int wp_x_fp, int wp_y_fp, int wp_psi_fp,
    int wp_vx_fp, int wp_kappa_fp, int wp_ax_fp,
    int wp_left_bound_fp, int wp_right_bound_fp, int wp_total,
    int state_x_fp, int state_y_fp, int state_theta_fp,
    int state_vx_fp, int state_vy_fp, int state_omega_fp,
    int state_steering_fp, int state_wp_idx,
    int *out_steering_fp, int *out_accel_fp,
    int *out_status, int *out_iterations);

/*===========================================================================
 * Configuration Constants
 *===========================================================================*/

/** Odometry callback divider (run MPC every N callbacks) */
#define ODOMETRY_CALLBACK_DIVIDER_DEFAULT 1
static int g_odom_divider = ODOMETRY_CALLBACK_DIVIDER_DEFAULT;

/** Maximum number of waypoints in loaded trajectory */
#define TRAJECTORY_MAXIMUM_WAYPOINTS 1024

/** Maximum reference velocity [m/s] */
#define TRAJECTORY_MAXIMUM_VELOCITY 20.0

/** Speed gain applied to trajectory velocities */
#define TRAJECTORY_SPEED_GAIN 1.0

/** MPC prediction horizon (must match MPC_HORIZON in mpc_fpga_types.h) */
#define MPC_PREDICTION_HORIZON_STEPS 20

/** MPC time step */
#define MPC_TIME_STEP_SECONDS 0.04f

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
    double acceleration_meters_per_second_squared;
    double left_bound_meters;
    double right_bound_meters;
} TrajectoryWaypoint_t;

/*===========================================================================
 * Global State Variables
 *===========================================================================*/

static TrajectoryWaypoint_t global_trajectory[TRAJECTORY_MAXIMUM_WAYPOINTS];
static int global_trajectory_count = 0;
static int global_last_closest_index = 0;
static int global_odometry_received_flag = 0;
static int global_odometry_callback_counter = 0;
static volatile int global_collision_detected = 0;
static rcl_context_t *global_ros2_context = NULL;

/* Current vehicle state (doubles) */
static double global_pos_x = 0.0;
static double global_pos_y = 0.0;
static double global_heading = 0.0;
static double global_vx = 0.0;
static double global_vy = 0.0;
static double global_omega = 0.0;

/* Servo dynamics tracking.
 * Since f1tenth gym doesn't publish actual servo angle, we simulate it
 * locally using the same rate limit (sv_max = 2.849 rad/s). */
static double global_actual_steering_angle = 0.0;
#define SERVO_RATE_LIMIT  2.849  /* rad/s */
#define CONTROL_DT        0.005  /* 200 Hz control rate */

/* Last MPC output (held between calls) */
static double global_cmd_steer = 0.0;
static double global_cmd_accel = 0.0;

static rcl_publisher_t global_control_publisher;
static rcl_publisher_t global_reference_path_publisher;
static rcl_publisher_t global_trajectory_path_publisher;

static nav_msgs__msg__Odometry global_odometry_message_buffer;
static std_msgs__msg__Bool global_collision_message_buffer;
static ackermann_msgs__msg__AckermannDriveStamped global_drive_message_buffer;
static nav_msgs__msg__Path global_reference_path_message;
static nav_msgs__msg__Path global_trajectory_path_message;

/*===========================================================================
 * Trajectory Loading (CSV from f1tenth_planning)
 *===========================================================================*/

/**
 * @brief Load trajectory from CSV file and upload to FPGA via mode 1/2 calls.
 *
 * CSV format (TUM compatible):
 *   # s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2[,left_bound,right_bound]
 */
static int load_trajectory_from_csv(const char *file_path)
{
    FILE *csv_file = fopen(file_path, "r");
    if (csv_file == NULL)
    {
        fprintf(stderr, "[MPC-FPGA] ERROR: Cannot open trajectory file: %s\n", file_path);
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
            printf("[MPC-FPGA] WARNING: Trajectory truncated at %d waypoints\n",
                   TRAJECTORY_MAXIMUM_WAYPOINTS);
            break;
        }

        double s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2;
        double left_bound = 5.0, right_bound = 5.0;
        int fields_read = sscanf(line_buffer, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                                 &s_m, &x_m, &y_m, &psi_rad,
                                 &kappa_radpm, &vx_mps, &ax_mps2,
                                 &left_bound, &right_bound);

        if (fields_read >= 7)
        {
            TrajectoryWaypoint_t *wp = &global_trajectory[global_trajectory_count];
            wp->x_meters = x_m;
            wp->y_meters = y_m;
            wp->heading_radians = psi_rad;
            wp->curvature_radians_per_meter = kappa_radpm;
            wp->acceleration_meters_per_second_squared = ax_mps2;
            wp->left_bound_meters = (fields_read >= 9) ? left_bound : 5.0;
            wp->right_bound_meters = (fields_read >= 9) ? right_bound : 5.0;

            double scaled_velocity = vx_mps * TRAJECTORY_SPEED_GAIN;
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
        fprintf(stderr, "[MPC-FPGA] ERROR: No waypoints loaded from %s\n", file_path);
        return 0;
    }

    printf("[MPC-FPGA] Loaded %d waypoints from %s\n", global_trajectory_count, file_path);
    printf("[MPC-FPGA] Speed gain: %.2f, max velocity: %.1f m/s\n",
           TRAJECTORY_SPEED_GAIN, TRAJECTORY_MAXIMUM_VELOCITY);

    /* Upload trajectory to FPGA via mode 1 calls */
    printf("[MPC-FPGA] Uploading %d waypoints to FPGA BRAM...\n", global_trajectory_count);

    int dummy_steer, dummy_accel, dummy_status, dummy_iters;
    for (int i = 0; i < global_trajectory_count; i++)
    {
        TrajectoryWaypoint_t *wp = &global_trajectory[i];
        mpc_fpga_top(
            1, i,  /* mode=1 (load waypoint), index=i */
            DOUBLE_TO_FP(wp->x_meters),
            DOUBLE_TO_FP(wp->y_meters),
            DOUBLE_TO_FP(wp->heading_radians),
            DOUBLE_TO_FP(wp->velocity_meters_per_second),
            DOUBLE_TO_FP(wp->curvature_radians_per_meter),
            DOUBLE_TO_FP(wp->acceleration_meters_per_second_squared),
            DOUBLE_TO_FP(wp->left_bound_meters),
            DOUBLE_TO_FP(wp->right_bound_meters),
            0,  /* wp_total unused in mode 1 */
            0, 0, 0, 0, 0, 0, 0, 0,  /* state unused in mode 1 */
            &dummy_steer, &dummy_accel, &dummy_status, &dummy_iters);
    }

    /* Finalize via mode 2 */
    mpc_fpga_top(
        2, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        global_trajectory_count,  /* wp_total */
        0, 0, 0, 0, 0, 0, 0, 0,
        &dummy_steer, &dummy_accel, &dummy_status, &dummy_iters);

    printf("[MPC-FPGA] Trajectory finalized: status=%d, count=%d\n",
           dummy_status, dummy_iters);

    return 1;
}

/*===========================================================================
 * Waypoint Search
 *===========================================================================*/

/**
 * @brief Find the closest trajectory waypoint to a position.
 *
 * Forward-biased local search from the last known position.
 */
static int find_closest_waypoint(double position_x, double position_y, double vehicle_heading)
{
    if (global_trajectory_count == 0)
        return 0;

    int search_window = global_trajectory_count / 4;
    if (search_window < 50) search_window = 50;
    int best_index = global_last_closest_index;
    double best_dist = 1e18;

    double dir_x = cos(vehicle_heading);
    double dir_y = sin(vehicle_heading);

    for (int offset = -search_window; offset <= search_window; offset++)
    {
        int idx = (global_last_closest_index + offset) % global_trajectory_count;
        if (idx < 0) idx += global_trajectory_count;

        double dx = global_trajectory[idx].x_meters - position_x;
        double dy = global_trajectory[idx].y_meters - position_y;
        double dist = dx * dx + dy * dy;

        /* Skip points that are behind the vehicle AND far away */
        double dot = dx * dir_x + dy * dir_y;
        if (dot < -0.5 && dist > 0.25) continue;

        if (dist < best_dist)
        {
            best_dist = dist;
            best_index = idx;
        }
    }

    global_last_closest_index = best_index;
    return best_index;
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

static void yaw_to_quaternion(double yaw, geometry_msgs__msg__Quaternion *q)
{
    if (q == NULL) return;
    double half = 0.5 * yaw;
    q->x = 0.0;
    q->y = 0.0;
    q->z = sin(half);
    q->w = cos(half);
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

/*===========================================================================
 * ROS2 Callback: Odometry Subscription
 *===========================================================================*/

void odometry_subscription_callback(const void *message_in)
{
    if (message_in == NULL)
    {
        fprintf(stderr, "[MPC-FPGA] ERROR: Odometry callback received NULL message\n");
        return;
    }

    const nav_msgs__msg__Odometry *odom =
        (const nav_msgs__msg__Odometry *)message_in;

    global_pos_x = odom->pose.pose.position.x;
    global_pos_y = odom->pose.pose.position.y;
    global_heading = quaternion_to_yaw_angle(
        odom->pose.pose.orientation.x,
        odom->pose.pose.orientation.y,
        odom->pose.pose.orientation.z,
        odom->pose.pose.orientation.w);
    global_vx = odom->twist.twist.linear.x;
    global_vy = odom->twist.twist.linear.y;
    global_omega = odom->twist.twist.angular.z;

    global_odometry_received_flag = 1;

    /* Run MPC at configured rate */
    if ((global_odometry_callback_counter % g_odom_divider) == 0)
    {
        printf("[MPC-FPGA] State: x=%.2f y=%.2f th=%.2f vx=%.2f vy=%.2f w=%.2f\n",
               global_pos_x, global_pos_y, global_heading,
               global_vx, global_vy, global_omega);

        if (global_trajectory_count > 0)
        {
            int closest = find_closest_waypoint(global_pos_x, global_pos_y, global_heading);

            /* Debug: Frenet error (computed host-side for logging only;
             * the FPGA does its own Frenet conversion internally) */
            {
                double dx = global_pos_x - global_trajectory[closest].x_meters;
                double dy = global_pos_y - global_trajectory[closest].y_meters;
                double wp_psi = global_trajectory[closest].heading_radians;
                double e_y = -dx * sin(wp_psi) + dy * cos(wp_psi);
                double e_psi = global_heading - wp_psi;
                while (e_psi > 3.14159265) e_psi -= 2.0 * 3.14159265;
                while (e_psi < -3.14159265) e_psi += 2.0 * 3.14159265;
                double vref = global_trajectory[closest].velocity_meters_per_second;
                double kappa = global_trajectory[closest].curvature_radians_per_meter;
                double lw = global_trajectory[closest].left_bound_meters;
                double rw = global_trajectory[closest].right_bound_meters;
                printf("[MPC-FPGA] Frenet: e_y=%.3f e_psi=%.3f v_ref=%.2f kappa=%.3f walls=[%.2f,%.2f]\n",
                       e_y, e_psi, vref, kappa, lw, rw);
            }

            /* Publish reference path visualization */
            global_reference_path_message.header.stamp = odom->header.stamp;
            global_reference_path_message.poses.size = MPC_PREDICTION_HORIZON_STEPS;
            {
                const double mpc_dt_viz = (double)MPC_TIME_STEP_SECONDS;
                const double avg_spacing_viz = 0.346;
                for (int step = 0; step < MPC_PREDICTION_HORIZON_STEPS; step++)
                {
                    geometry_msgs__msg__PoseStamped *pose =
                        &global_reference_path_message.poses.data[step];
                    int base_idx = (closest + step) % global_trajectory_count;
                    double rv = global_trajectory[base_idx].velocity_meters_per_second;
                    if (rv < 3.0) rv = 3.0;
                    if (rv > TRAJECTORY_MAXIMUM_VELOCITY) rv = TRAJECTORY_MAXIMUM_VELOCITY;
                    int wpa = (int)(rv * mpc_dt_viz * (step + 1) / avg_spacing_viz);
                    if (wpa < step + 1) wpa = step + 1;
                    int wp_idx = (closest + wpa) % global_trajectory_count;
                    pose->pose.position.x = global_trajectory[wp_idx].x_meters;
                    pose->pose.position.y = global_trajectory[wp_idx].y_meters;
                    pose->pose.position.z = 0.0;
                    yaw_to_quaternion(global_trajectory[wp_idx].heading_radians,
                                      &pose->pose.orientation);
                }
            }
            rcl_publish(&global_reference_path_publisher, &global_reference_path_message, NULL);

            /* Publish full trajectory visualization */
            global_trajectory_path_message.header.stamp = odom->header.stamp;
            rcl_publish(&global_trajectory_path_publisher, &global_trajectory_path_message, NULL);

            /* ===== Call FPGA MPC (mode 0: compute) — always, no guards ===== */
            int out_steer_fp, out_accel_fp, out_status, out_iters;

            struct timespec t0, t1;
            clock_gettime(CLOCK_MONOTONIC, &t0);

            mpc_fpga_top(
                0,  /* mode=0 (compute) */
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  /* waypoint params unused in mode 0 */
                DOUBLE_TO_FP(global_pos_x),
                DOUBLE_TO_FP(global_pos_y),
                DOUBLE_TO_FP(global_heading),
                DOUBLE_TO_FP(global_vx),
                DOUBLE_TO_FP(global_vy),
                DOUBLE_TO_FP(global_omega),
                DOUBLE_TO_FP(global_actual_steering_angle),
                closest,  /* closest waypoint index */
                &out_steer_fp, &out_accel_fp,
                &out_status, &out_iters);

            clock_gettime(CLOCK_MONOTONIC, &t1);
            double solve_us = (t1.tv_sec - t0.tv_sec) * 1e6 +
                              (t1.tv_nsec - t0.tv_nsec) / 1e3;

            double steer = FP_TO_DOUBLE(out_steer_fp);
            double accel = FP_TO_DOUBLE(out_accel_fp);

            /* Always use MPC output — no fallback, no override */
            global_cmd_steer = steer;
            global_cmd_accel = accel;

            /* Simulate servo dynamics to track the actual steering angle.
             * The f1tenth gym rate-limits steering at sv_max = 2.849 rad/s.
             * We replicate this locally to feed δ_actual back to the MPC. */
            {
                double max_delta = SERVO_RATE_LIMIT * CONTROL_DT;
                double steer_diff = steer - global_actual_steering_angle;
                if (steer_diff > max_delta) steer_diff = max_delta;
                if (steer_diff < -max_delta) steer_diff = -max_delta;
                global_actual_steering_angle += steer_diff;
            }

            printf("[MPC-FPGA] Control: steer=%.4f accel=%.2f (status=%d iter=%d solve=%.1fus)\n",
                   steer, accel, out_status, out_iters, solve_us);
            fflush(stdout);
        }
        else
        {
            /* No trajectory: stop */
            global_cmd_steer = 0.0;
            global_cmd_accel = 0.0;
        }
    }

    /* Publish drive command every callback */
    if (global_odometry_received_flag)
    {
        global_drive_message_buffer.drive.steering_angle = (float)global_cmd_steer;

        /* gym_bridge with control_input=['accl','steering_angle']
         * interprets drive.speed as acceleration command. */
        global_drive_message_buffer.drive.speed = (float)global_cmd_accel;

        rcl_publish(&global_control_publisher, &global_drive_message_buffer, NULL);
    }

    global_odometry_callback_counter++;
}

/*===========================================================================
 * ROS2 Callback: Collision Subscription
 *===========================================================================*/

void collision_subscription_callback(const void *message_in)
{
    if (message_in == NULL) return;

    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)message_in;

    if (!msg->data || global_collision_detected) return;

    global_collision_detected = 1;
    fprintf(stderr, "[MPC-FPGA] COLLISION detected. Shutting down.\n");

    if (global_ros2_context != NULL && rcl_context_is_valid(global_ros2_context))
    {
        rcl_ret_t rc = rcl_shutdown(global_ros2_context);
        if (rc != RCL_RET_OK)
        {
            fprintf(stderr, "[ROS2] WARNING: rcl_shutdown failed: %s\n",
                    rcl_get_error_string().str);
            rcl_reset_error();
            raise(SIGINT);
        }
    }
    else
    {
        raise(SIGINT);
    }
}

/*===========================================================================
 * Main Entry Point
 *===========================================================================*/

int main(int argc, char *argv[])
{
    printf("============================================================\n");
    printf("  FPGA MPC (Riccati-ADMM HLS) ROS2 Node for F1/10th Sim\n");
    printf("  C-simulation of FPGA top-level function\n");
    printf("  Mode-based interface: mpc_fpga_top()\n");
    printf("  State: [e_y, e_psi, vx, vy, omega, delta_actual, drate_prev, accel_prev]\n");
    printf("  Controls: [delta_rate, a_x] -> output: [steering, accel]\n");
    printf("============================================================\n");
    printf("  Prediction horizon: %d steps (%.1f ms each)\n",
           MPC_PREDICTION_HORIZON_STEPS,
           MPC_TIME_STEP_SECONDS * 1000.0f);
    printf("  Trajectory speed gain: %.2f, max velocity: %.1f m/s\n",
           TRAJECTORY_SPEED_GAIN, TRAJECTORY_MAXIMUM_VELOCITY);
    printf("  Max waypoints: %d (FPGA BRAM limit)\n", TRAJECTORY_MAXIMUM_WAYPOINTS);
    printf("------------------------------------------------------------\n");
    printf("  Subscribe: /ego_racecar/ground_truth (nav_msgs/Odometry)\n");
    printf("  Subscribe: /ego_racecar/collision     (std_msgs/Bool)\n");
    printf("  Publish:   /drive (ackermann_msgs/AckermannDriveStamped)\n");
    printf("============================================================\n\n");

    rcl_ret_t rc;

    /* Runtime parameters from environment */
    {
        const char *env_val;
        if ((env_val = getenv("MPC_RATE_DIVIDER")) != NULL)
        {
            int div = atoi(env_val);
            if (div >= 1 && div <= 100) g_odom_divider = div;
        }
    }

    printf("[MPC-FPGA] Control rate: ~%d Hz (odom_divider=%d)\n",
           200 / g_odom_divider, g_odom_divider);

    /* Load trajectory (this also uploads to FPGA via mode 1/2) */
    const char *trajectory_file = NULL;
    if (argc >= 2)
    {
        trajectory_file = argv[1];
    }
    else
    {
        trajectory_file = "/ros2_ws/src/f1tenth_planning/trajectories/Spielberg_raceline.csv";
    }

    if (load_trajectory_from_csv(trajectory_file))
    {
        printf("[MPC-FPGA] Trajectory loaded and uploaded successfully\n");
    }
    else
    {
        fprintf(stderr, "[MPC-FPGA] ERROR: Failed to load trajectory\n");
        return 1;
    }

    /* Build full trajectory path message for RViz */
    nav_msgs__msg__Path__init(&global_trajectory_path_message);
    if (!preallocate_rosidl_string(&global_trajectory_path_message.header.frame_id, 64))
    {
        fprintf(stderr, "[ROS2] ERROR: Failed to pre-allocate trajectory path frame_id\n");
        return 1;
    }
    set_rosidl_string(&global_trajectory_path_message.header.frame_id, "map");

    if (global_trajectory_count > 0)
    {
        if (!geometry_msgs__msg__PoseStamped__Sequence__init(
                &global_trajectory_path_message.poses,
                global_trajectory_count))
        {
            fprintf(stderr, "[ROS2] ERROR: Failed to allocate trajectory path poses\n");
            return 1;
        }
        for (int i = 0; i < global_trajectory_count; i++)
        {
            geometry_msgs__msg__PoseStamped *pose =
                &global_trajectory_path_message.poses.data[i];
            pose->pose.position.x = global_trajectory[i].x_meters;
            pose->pose.position.y = global_trajectory[i].y_meters;
            pose->pose.position.z = 0.0;
            yaw_to_quaternion(global_trajectory[i].heading_radians,
                              &pose->pose.orientation);
        }
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

    rc = rcl_node_init(&node, "mpc_fpga_node", "", &ctx, &node_opts);
    if (rc != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: node_init: %s\n", rcl_get_error_string().str);
        return 1;
    }
    printf("[ROS2] Node 'mpc_fpga_node' created\n");

    /* Subscriptions */
    rcl_subscription_t odom_sub = rcl_get_zero_initialized_subscription();
    rcl_subscription_options_t sub_opts = rcl_subscription_get_default_options();

    rc = rcl_subscription_init(&odom_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/ego_racecar/ground_truth", &sub_opts);
    if (rc != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: odom subscription: %s\n", rcl_get_error_string().str);
        return 1;
    }
    printf("[ROS2] Subscribed to /ego_racecar/ground_truth\n");

    rcl_subscription_t collision_sub = rcl_get_zero_initialized_subscription();
    rc = rcl_subscription_init(&collision_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/ego_racecar/collision", &sub_opts);
    if (rc != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: collision subscription: %s\n", rcl_get_error_string().str);
        return 1;
    }
    printf("[ROS2] Subscribed to /ego_racecar/collision\n");

    /* Publishers */
    rcl_publisher_options_t pub_opts = rcl_publisher_get_default_options();

    global_control_publisher = rcl_get_zero_initialized_publisher();
    rc = rcl_publisher_init(&global_control_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(ackermann_msgs, msg, AckermannDriveStamped),
        "/drive", &pub_opts);
    if (rc != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: drive publisher: %s\n", rcl_get_error_string().str);
        return 1;
    }
    printf("[ROS2] Publishing to /drive\n");

    global_reference_path_publisher = rcl_get_zero_initialized_publisher();
    rc = rcl_publisher_init(&global_reference_path_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Path),
        "/mpc/reference_path", &pub_opts);
    if (rc != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: ref path publisher: %s\n", rcl_get_error_string().str);
        return 1;
    }
    printf("[ROS2] Publishing to /mpc/reference_path\n");

    global_trajectory_path_publisher = rcl_get_zero_initialized_publisher();
    rc = rcl_publisher_init(&global_trajectory_path_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Path),
        "/mpc/trajectory_path", &pub_opts);
    if (rc != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: trajectory path publisher: %s\n", rcl_get_error_string().str);
        return 1;
    }
    printf("[ROS2] Publishing to /mpc/trajectory_path\n");

    /* Initialize message buffers */
    nav_msgs__msg__Odometry__init(&global_odometry_message_buffer);
    if (!preallocate_rosidl_string(&global_odometry_message_buffer.header.frame_id, 64) ||
        !preallocate_rosidl_string(&global_odometry_message_buffer.child_frame_id, 64))
    {
        fprintf(stderr, "[ROS2] ERROR: odom string alloc\n");
        return 1;
    }

    std_msgs__msg__Bool__init(&global_collision_message_buffer);

    ackermann_msgs__msg__AckermannDriveStamped__init(&global_drive_message_buffer);
    if (!preallocate_rosidl_string(&global_drive_message_buffer.header.frame_id, 64))
    {
        fprintf(stderr, "[ROS2] ERROR: drive header string alloc\n");
        return 1;
    }

    nav_msgs__msg__Path__init(&global_reference_path_message);
    if (!preallocate_rosidl_string(&global_reference_path_message.header.frame_id, 64))
    {
        fprintf(stderr, "[ROS2] ERROR: ref path string alloc\n");
        return 1;
    }
    set_rosidl_string(&global_reference_path_message.header.frame_id, "map");

    if (!geometry_msgs__msg__PoseStamped__Sequence__init(
            &global_reference_path_message.poses,
            MPC_PREDICTION_HORIZON_STEPS))
    {
        fprintf(stderr, "[ROS2] ERROR: ref path poses alloc\n");
        return 1;
    }

    /* Executor */
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    rcl_allocator_t alloc = rcl_get_default_allocator();

    rc = rclc_executor_init(&executor, &ctx, 2, &alloc);
    if (rc != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: executor init: %s\n", rcl_get_error_string().str);
        return 1;
    }

    rc = rclc_executor_add_subscription(&executor, &odom_sub,
        &global_odometry_message_buffer, &odometry_subscription_callback, ON_NEW_DATA);
    if (rc != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: add odom sub: %s\n", rcl_get_error_string().str);
        return 1;
    }

    rc = rclc_executor_add_subscription(&executor, &collision_sub,
        &global_collision_message_buffer, &collision_subscription_callback, ON_NEW_DATA);
    if (rc != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: add collision sub: %s\n", rcl_get_error_string().str);
        return 1;
    }

    printf("[ROS2] Executor ready\n");
    printf("\n[MPC-FPGA] Spinning... (waiting for odometry messages)\n\n");

    rclc_executor_spin(&executor);

    /* Cleanup */
    printf("\n[ROS2] Shutting down...\n");
    rclc_executor_fini(&executor);
    nav_msgs__msg__Odometry__fini(&global_odometry_message_buffer);
    std_msgs__msg__Bool__fini(&global_collision_message_buffer);
    ackermann_msgs__msg__AckermannDriveStamped__fini(&global_drive_message_buffer);
    nav_msgs__msg__Path__fini(&global_reference_path_message);
    nav_msgs__msg__Path__fini(&global_trajectory_path_message);
    rcl_ret_t cleanup_rc;
    cleanup_rc = rcl_subscription_fini(&odom_sub, &node); (void)cleanup_rc;
    cleanup_rc = rcl_subscription_fini(&collision_sub, &node); (void)cleanup_rc;
    cleanup_rc = rcl_publisher_fini(&global_control_publisher, &node); (void)cleanup_rc;
    cleanup_rc = rcl_publisher_fini(&global_reference_path_publisher, &node); (void)cleanup_rc;
    cleanup_rc = rcl_publisher_fini(&global_trajectory_path_publisher, &node); (void)cleanup_rc;
    cleanup_rc = rcl_node_fini(&node); (void)cleanup_rc;
    cleanup_rc = rcl_context_fini(&ctx); (void)cleanup_rc;

    printf("[ROS2] Cleanup complete\n");
    return 0;
}
