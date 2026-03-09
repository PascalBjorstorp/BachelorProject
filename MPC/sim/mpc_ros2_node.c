/**
 * @file mpc_ros2_node.c
 * @brief MPC Riccati-ADMM ROS2 Node for F1/10th Simulator Integration
 *
 * Implements ROS2 node using rclc (C client library) for Jazzy.
 * Subscribes to odometry, runs the Riccati-ADMM MPC solver,
 * publishes control commands.
 *
 * IMPORTANT: This node is a transparent bridge between the simulator and the
 * MPC solver.  Nothing in this file may alter, clamp, bias, or post-process
 * the control output returned by mpc_compute_optimal_control().  All tuning
 * and constraint handling lives inside the solver.
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

/* MPC Core Library Headers (Platform-Independent) */
#include "mpc.h"
#include "mpc_types.h"
#include "fp_math.h"
#include "vehicle_model.h"

/*===========================================================================
 * Configuration Constants
 *===========================================================================*/

/** Number of MPC prediction steps (20 steps x 0.05s = 1.0 second lookahead) */
#define MPC_PREDICTION_HORIZON_STEPS 20

/** Time step between predictions [seconds] — must match MPC_DEFAULT_TIME_STEP_SECONDS */
#define MPC_TIME_STEP_SECONDS 0.05f

/** Odometry callback divider (run MPC every N callbacks, default 1 = ~250 Hz) */
#define ODOMETRY_CALLBACK_DIVIDER_DEFAULT 1
static int g_odom_divider = ODOMETRY_CALLBACK_DIVIDER_DEFAULT;

/** Maximum number of waypoints in loaded trajectory */
#define TRAJECTORY_MAXIMUM_WAYPOINTS 2000

/** Maximum reference velocity [m/s] */
#define TRAJECTORY_MAXIMUM_VELOCITY 20.0

/** Speed gain applied to trajectory velocities (1.0 = full optimal racing speed) */
#define TRAJECTORY_SPEED_GAIN 1.0

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
} TrajectoryWaypoint_t;

/*===========================================================================
 * Global State Variables
 *===========================================================================*/

static TrajectoryWaypoint_t global_trajectory[TRAJECTORY_MAXIMUM_WAYPOINTS];
static int global_trajectory_count = 0;
static int global_last_closest_index = 0;
static VehicleState_t global_vehicle_state = {0};
static FrenetState_t global_frenet_state = {0};
static ControlInput_t global_control_command = {0};
static int global_odometry_received_flag = 0;
static int global_odometry_callback_counter = 0;
static volatile int global_collision_detected = 0;
static rcl_context_t *global_ros2_context = NULL;

/* Servo dynamics tracking for 8-state MPC.
 * The MPC needs to know the actual servo position (δ_actual) to correctly
 * compute steering commands via the integrator: δ_cmd = δ_actual + dt * δ̇.
 * Since the f1tenth gym doesn't publish the actual servo angle, we simulate
 * it locally using the same rate limit (sv_max = 2.849 rad/s). */
static double global_actual_steering_angle = 0.0;
#define SERVO_RATE_LIMIT  2.849  /* rad/s — matches f1tenth gym sv_max */
#define CONTROL_DT        0.005  /* 200 Hz control rate */

static rcl_publisher_t global_control_publisher;
static rcl_publisher_t global_reference_path_publisher;
static rcl_publisher_t global_trajectory_path_publisher;

static nav_msgs__msg__Odometry global_odometry_message_buffer;
static std_msgs__msg__Bool global_collision_message_buffer;
static ackermann_msgs__msg__AckermannDriveStamped global_drive_message_buffer;
static nav_msgs__msg__Path global_reference_path_message;
static nav_msgs__msg__Path global_trajectory_path_message;

static TrajectoryReferencePoint_t global_reference_trajectory[MPC_PREDICTION_HORIZON_STEPS];

/*===========================================================================
 * Trajectory Loading (CSV from f1tenth_planning)
 *===========================================================================*/

/**
 * @brief Load trajectory from CSV file.
 *
 * CSV format (TUM compatible):
 *   # s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2[,left_bound,right_bound]
 *
 * Velocities are scaled by TRAJECTORY_SPEED_GAIN and clamped
 * to TRAJECTORY_MAXIMUM_VELOCITY.
 *
 * @param file_path  Path to the CSV trajectory file
 * @return 1 on success, 0 on failure
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

            double scaled_velocity = vx_mps * TRAJECTORY_SPEED_GAIN;
            if (scaled_velocity > TRAJECTORY_MAXIMUM_VELOCITY)
            {
                scaled_velocity = TRAJECTORY_MAXIMUM_VELOCITY;
            }
            if (scaled_velocity < 0.0)
            {
                scaled_velocity = 0.0;
            }
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

    printf("[MPC] Loaded %d waypoints from %s\n", global_trajectory_count, file_path);
    printf("[MPC] Speed gain: %.2f, max velocity: %.1f m/s\n",
           TRAJECTORY_SPEED_GAIN, TRAJECTORY_MAXIMUM_VELOCITY);

    printf("[MPC] Sample velocities: wp[0]=%.2f, wp[100]=%.2f, wp[500]=%.2f m/s\n",
           global_trajectory[0].velocity_meters_per_second,
           global_trajectory[100 < global_trajectory_count ? 100 : global_trajectory_count - 1].velocity_meters_per_second,
           global_trajectory[500 < global_trajectory_count ? 500 : global_trajectory_count - 1].velocity_meters_per_second);

    return 1;
}

/*===========================================================================
 * Waypoint Search
 *===========================================================================*/

/**
 * @brief Find the closest trajectory waypoint to a position.
 *
 * Forward-biased local search from the last known position.
 * Searches a window of 50 waypoints ahead and 3 behind to prevent
 * backward jumps that cause Frenet frame discontinuities.
 * Among equidistant candidates, prefers the one ahead of the vehicle.
 */
static int find_closest_waypoint(double position_x, double position_y, double vehicle_heading)
{
    if (global_trajectory_count == 0)
    {
        return 0;
    }

    int search_start = global_last_closest_index;
    int search_window = 50;
    int best_index = search_start;
    double best_score = 1e18;

    for (int offset = -3; offset < search_window; offset++)
    {
        int idx = (search_start + offset) % global_trajectory_count;
        if (idx < 0) idx += global_trajectory_count;

        double dx = global_trajectory[idx].x_meters - position_x;
        double dy = global_trajectory[idx].y_meters - position_y;
        double dist = dx * dx + dy * dy;

        /* Penalize points behind the vehicle */
        double veh_dx = cos(vehicle_heading);
        double veh_dy = sin(vehicle_heading);
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
 * Each prediction step maps to the waypoint at the expected travel distance
 * based on the reference velocity.
 */
static void build_reference_from_trajectory(int closest_index)
{
    const double mpc_dt = (double)MPC_TIME_STEP_SECONDS;
    const double avg_waypoint_spacing = 0.346;  /* meters — from Spielberg trajectory */

    for (int step = 0; step < MPC_PREDICTION_HORIZON_STEPS; step++)
    {
        int base_waypoint_index = (closest_index + step) % global_trajectory_count;
        double ref_velocity = global_trajectory[base_waypoint_index].velocity_meters_per_second;

        if (ref_velocity < 3.0) ref_velocity = 3.0;
        if (ref_velocity > TRAJECTORY_MAXIMUM_VELOCITY) ref_velocity = TRAJECTORY_MAXIMUM_VELOCITY;

        double expected_distance = ref_velocity * mpc_dt * (step + 1);
        int waypoints_ahead = (int)(expected_distance / avg_waypoint_spacing);
        if (waypoints_ahead < step + 1) waypoints_ahead = step + 1;

        int waypoint_index = (closest_index + waypoints_ahead) % global_trajectory_count;
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

        global_reference_trajectory[step].reference_lateral_velocity_meters_per_second = 0;
        global_reference_trajectory[step].reference_yaw_rate_radians_per_second = 0;
    }

    /* Second pass: yaw rate reference = kappa * v_ref (steady-state cornering) */
    for (int step = 0; step < MPC_PREDICTION_HORIZON_STEPS; step++)
    {
        double kappa = FP_TO_DOUBLE(global_reference_trajectory[step].path_curvature_radians_per_meter);
        double v_ref = FP_TO_DOUBLE(global_reference_trajectory[step].reference_velocity_meters_per_second);
        double omega_ref = kappa * v_ref;
        global_reference_trajectory[step].reference_yaw_rate_radians_per_second =
            DOUBLE_TO_FP(omega_ref);

        /* Lateral velocity reference: zero (not used) */
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
 * Frenet State Conversion
 *===========================================================================*/

/**
 * @brief Convert global vehicle state to Frenet (path-relative) state.
 *
 * Projects the car position onto the segment between closest and closest+1
 * waypoints, then interpolates path position and heading at the projection
 * point. This eliminates discontinuous jumps when the closest waypoint index
 * changes, providing smooth Frenet state feedback to the MPC.
 *
 * e_y   = signed perpendicular distance (positive = left of path)
 * e_psi = heading error (vehicle heading - interpolated path tangent)
 * v_x, v_y, omega copied from body-frame state unchanged.
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

    /* Project car position onto segment A→B, parameter t ∈ [0,1] */
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

    /* Signed lateral error (positive = left of path) */
    double dx = car_x - path_x;
    double dy = car_y - path_y;
    double lateral_error = -dx * sin(path_heading) + dy * cos(path_heading);

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

/*===========================================================================
 * ROS2 Callback: Odometry Subscription
 *===========================================================================*/

void odometry_subscription_callback(const void *message_in)
{
    if (message_in == NULL)
    {
        fprintf(stderr, "[MPC] ERROR: Odometry callback received NULL message\n");
        return;
    }

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

    global_vehicle_state.position_x_meters = DOUBLE_TO_FP(pos_x);
    global_vehicle_state.position_y_meters = DOUBLE_TO_FP(pos_y);
    global_vehicle_state.heading_angle_radians = DOUBLE_TO_FP(heading);
    global_vehicle_state.longitudinal_velocity_meters_per_second = DOUBLE_TO_FP(vx);
    global_vehicle_state.lateral_velocity_meters_per_second = DOUBLE_TO_FP(vy);
    global_vehicle_state.yaw_rate_radians_per_second = DOUBLE_TO_FP(omega);

    global_odometry_received_flag = 1;

    /* Run MPC at reduced rate */
    if ((global_odometry_callback_counter % g_odom_divider) == 0)
    {
        printf("[MPC] State: x=%.2f y=%.2f th=%.2f vx=%.2f vy=%.2f w=%.2f\n",
               pos_x, pos_y, heading, vx, vy, omega);

        if (global_trajectory_count > 0)
        {
            int closest = find_closest_waypoint(pos_x, pos_y, heading);
            build_reference_from_trajectory(closest);
            convert_to_frenet_state(pos_x, pos_y, heading, closest, &global_frenet_state);

            /* Debug: Frenet state + reference */
            {
                double ey  = FP_TO_DOUBLE(global_frenet_state.lateral_error_meters);
                double epsi = FP_TO_DOUBLE(global_frenet_state.heading_error_radians);
                double vref = FP_TO_DOUBLE(global_reference_trajectory[0].reference_velocity_meters_per_second);
                double kappa = FP_TO_DOUBLE(global_reference_trajectory[0].path_curvature_radians_per_meter);
                double lw = FP_TO_DOUBLE(global_reference_trajectory[0].left_wall_bound_meters);
                double rw = FP_TO_DOUBLE(global_reference_trajectory[0].right_wall_bound_meters);
                printf("[MPC] Frenet: e_y=%.3f e_psi=%.3f v_ref=%.2f kappa=%.3f walls=[%.2f,%.2f]\n",
                       ey, epsi, vref, kappa, lw, rw);
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

        /* ===== Run MPC — always, output used DIRECTLY, no guards ===== */
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

        {
            double steer = FP_TO_DOUBLE(
                mpc_result.optimal_control.steering_angle_radians);
            double accel = FP_TO_DOUBLE(
                mpc_result.optimal_control.acceleration_meters_per_second_squared);

            /* Always use MPC output — no fallback, no override */
            global_control_command.steering_angle_radians =
                mpc_result.optimal_control.steering_angle_radians;
            global_control_command.acceleration_meters_per_second_squared =
                mpc_result.optimal_control.acceleration_meters_per_second_squared;

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

            /* Feed actual servo position and acceleration back to MPC.
             * This updates actual_steering_angle (for x0[5]) and prev accel. */
            {
                ControlInput_t actual_ctrl;
                actual_ctrl.steering_angle_radians =
                    DOUBLE_TO_FP(global_actual_steering_angle);
                actual_ctrl.acceleration_meters_per_second_squared =
                    mpc_result.optimal_control.acceleration_meters_per_second_squared;
                mpc_set_actual_previous_control(&actual_ctrl);
            }

            printf("[MPC] Control: steer=%.4f accel=%.2f (status=%d iter=%d solve=%.1fus)\n",
                   steer, accel, mpc_status, mpc_result.iterations_used, solve_us);
            fflush(stdout);
        }
    }

    /* Publish drive command every callback */
    if (global_odometry_received_flag)
    {
        global_drive_message_buffer.drive.steering_angle = FP_TO_FLOAT(
            global_control_command.steering_angle_radians);

        /* gym_bridge with control_input=['accl','steering_angle']
         * interprets drive.speed as acceleration command. */
        global_drive_message_buffer.drive.speed = FP_TO_FLOAT(
            global_control_command.acceleration_meters_per_second_squared);

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
    fprintf(stderr, "[MPC] COLLISION detected. Shutting down.\n");

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
    printf("  MPC Riccati-ADMM ROS2 Node for F1/10th Simulator\n");
    printf("  8-state augmented Frenet model\n");
    printf("  [e_y, e_psi, vx, vy, omega, delta_actual, drate_prev, accel_prev]\n");
    printf("  Controls: [delta_rate, a_x]\n");
    printf("  Solver: Riccati backward/forward pass inside ADMM loop\n");
    printf("============================================================\n");
    printf("  Prediction horizon: %d steps (%.1f ms each)\n",
           MPC_PREDICTION_HORIZON_STEPS,
           MPC_TIME_STEP_SECONDS * 1000.0f);
    printf("  Trajectory speed gain: %.2f, max velocity: %.1f m/s\n",
           TRAJECTORY_SPEED_GAIN, TRAJECTORY_MAXIMUM_VELOCITY);
    printf("------------------------------------------------------------\n");
    printf("  Subscribe: /ego_racecar/ground_truth (nav_msgs/Odometry)\n");
    printf("  Subscribe: /ego_racecar/collision     (std_msgs/Bool)\n");
    printf("  Publish:   /drive (ackermann_msgs/AckermannDriveStamped)\n");
    printf("============================================================\n\n");

    rcl_ret_t rc;

    /* Initialize MPC controller — uses Riccati-ADMM internally */
    mpc_initialize();

    /* Runtime parameters from environment (no rebuild needed) */
    {
        const char *env_val;
        if ((env_val = getenv("MPC_RATE_DIVIDER")) != NULL)
        {
            int div = atoi(env_val);
            if (div >= 1 && div <= 100) g_odom_divider = div;
        }
    }

    printf("[MPC] Controller initialized (horizon=%d, dt=%.0fms)\n",
           MPC_PREDICTION_HORIZON_STEPS, MPC_TIME_STEP_SECONDS * 1000.0f);
    printf("[MPC] Control rate: ~%d Hz (odom_divider=%d)\n",
           200 / g_odom_divider, g_odom_divider);

    {
        MpcConfiguration_t cfg = mpc_get_configuration();
        printf("[MPC] max_iter=%u, tol=%d\n",
               cfg.maximum_solver_iterations,
               (int)cfg.solver_convergence_tolerance);
        printf("[MPC] Weights: lat=%.2f heading=%.2f vel=%.2f steer_rate=%.2f steer_effort=%.4f\n",
               FP_TO_DOUBLE(cfg.weight_lateral_error),
               FP_TO_DOUBLE(cfg.weight_heading_error),
               FP_TO_DOUBLE(cfg.weight_velocity),
               FP_TO_DOUBLE(cfg.weight_steering_rate),
               FP_TO_DOUBLE(cfg.weight_steering_effort));
        printf("[MPC] cross_call_scale=%.2f\n",
               FP_TO_DOUBLE(cfg.cross_call_rate_scale));
    }

    /* Load trajectory */
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
        printf("[MPC] Trajectory loaded successfully\n");
    }
    else
    {
        printf("[MPC] WARNING: No trajectory loaded, using straight-line fallback\n");
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

    rc = rcl_node_init(&node, "mpc_riccati_node", "", &ctx, &node_opts);
    if (rc != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: node_init: %s\n", rcl_get_error_string().str);
        return 1;
    }
    printf("[ROS2] Node 'mpc_riccati_node' created\n");

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
    printf("\n[MPC] Spinning... (waiting for odometry messages)\n\n");

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
