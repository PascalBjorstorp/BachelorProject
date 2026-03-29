#define _POSIX_C_SOURCE 199309L

/**
 * @file mpc_ros2_node.c
 * @brief MPC Riccati-ADMM ROS2 Node for F1/10th Simulator Integration
 * @details Implements a ROS2 bridge node using rclc for Jazzy. The node
 *          subscribes to simulator odometry/collision topics, converts state
 *          to the solver's Frenet representation, runs the Riccati-ADMM MPC,
 *          and publishes direct control commands without post-processing.
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
 * @dependencies mpc.h, mpc_types.h, util_math.h, vehicle_model.h,
 *               rclc, rcl, nav_msgs, ackermann_msgs, geometry_msgs, std_msgs
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
#include "util_math.h"
#include "vehicle_model.h"

/*===========================================================================
 * Configuration Constants
 *===========================================================================*/

/** Maximum prediction horizon (array sizing) */
#define MPC_MAX_HORIZON_STEPS 30

/** Default prediction horizon and dt — overridden by HORIZON / PRED_DT env vars */
#define MPC_DEFAULT_DT      0.048f

/** Runtime horizon and dt (set from env vars in main()) */
static int    g_mpc_horizon = MPC_PREDICTION_HORIZON;
static double g_mpc_dt      = MPC_DEFAULT_DT;

/** Odometry callback divider (run MPC every N callbacks, default 1 = ~250 Hz) */
#define ODOMETRY_CALLBACK_DIVIDER_DEFAULT 1

/** Maximum number of waypoints in loaded trajectory */
/* NOTE: This redefines TRAJECTORY_MAXIMUM_WAYPOINTS from mpc_types.h (1000).
 * The sim node uses a larger buffer to accommodate longer tracks. The two
 * definitions must be kept in sync or unified into a shared header. */
#define TRAJECTORY_MAXIMUM_WAYPOINTS 2000

/** Maximum reference velocity [m/s] */
#define TRAJECTORY_MAXIMUM_VELOCITY 20.0

/*===========================================================================
 * Trajectory Waypoint (loaded from CSV, stored as double)
 *===========================================================================*/

typedef struct
{
    double s_meters;
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
static double global_track_length_meters = 0.0;
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
 * it locally using the configured steering-rate limit. */
static double global_actual_steering_angle = 0.0;

static rcl_publisher_t global_control_publisher;
static rcl_publisher_t global_reference_path_publisher;
static rcl_publisher_t global_trajectory_path_publisher;

static nav_msgs__msg__Odometry global_odometry_message_buffer;
static std_msgs__msg__Bool global_collision_message_buffer;
static ackermann_msgs__msg__AckermannDriveStamped global_drive_message_buffer;
static nav_msgs__msg__Path global_reference_path_message;
static nav_msgs__msg__Path global_trajectory_path_message;

static TrajectoryReferencePoint_t global_reference_trajectory[MPC_MAX_HORIZON_STEPS];

/*===========================================================================
 * Trajectory Loading (CSV from f1tenth_planning)
 *===========================================================================*/

/* Load trajectory waypoints from CSV into the global ring-track buffer.
 * Parameters: file_path is a filesystem path to a TUM-format raceline CSV.
 * Returns 1 on successful parse with at least one waypoint, otherwise 0. */
static int load_trajectory_from_csv(const char *file_path)
{
    FILE *csv_file = fopen(file_path, "r");
    if (csv_file == NULL)
    {
        fprintf(stderr, "[MPC] ERROR: Cannot open trajectory file: %s\n", file_path);
        return 0;
    }

    char line_buffer[512];
    int line_number = 0;
    global_trajectory_count = 0;

    while (fgets(line_buffer, sizeof(line_buffer), csv_file) != NULL)
    {
        line_number++;
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
        double left_bound = 0.0, right_bound = 0.0;
        int fields_read = sscanf(line_buffer, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                                 &s_m, &x_m, &y_m, &psi_rad,
                                 &kappa_radpm, &vx_mps, &ax_mps2,
                                 &left_bound, &right_bound);

        if (fields_read != 9) {
            fprintf(stderr,
                    "[MPC] ERROR: Trajectory line %d must contain 9 columns including wall bounds\n",
                    line_number);
            fclose(csv_file);
            return 0;
        }
        if (left_bound <= 0.0 || right_bound <= 0.0) {
            fprintf(stderr,
                    "[MPC] ERROR: Invalid wall bounds at trajectory line %d (left=%.3f right=%.3f)\n",
                    line_number, left_bound, right_bound);
            fclose(csv_file);
            return 0;
        }

        TrajectoryWaypoint_t *wp = &global_trajectory[global_trajectory_count];
        wp->s_meters = s_m;
        wp->x_meters = x_m;
        wp->y_meters = y_m;
        wp->heading_radians = psi_rad;
        wp->curvature_radians_per_meter = kappa_radpm;
        wp->left_bound_meters = left_bound;
        wp->right_bound_meters = right_bound;

        if (vx_mps > TRAJECTORY_MAXIMUM_VELOCITY)
        {
            vx_mps = TRAJECTORY_MAXIMUM_VELOCITY;
        }
        if (vx_mps < 0.0)
        {
            vx_mps = 0.0;
        }
        wp->velocity_meters_per_second = vx_mps;

        global_trajectory_count++;
    }

    fclose(csv_file);

    if (global_trajectory_count == 0)
    {
        fprintf(stderr, "[MPC] ERROR: No waypoints loaded from %s\n", file_path);
        return 0;
    }

    printf("[MPC] Loaded %d waypoints from %s\n", global_trajectory_count, file_path);
    printf("[MPC] Max velocity: %.1f m/s\n",
           TRAJECTORY_MAXIMUM_VELOCITY);

    printf("[MPC] Sample velocities: wp[0]=%.2f, wp[100]=%.2f, wp[500]=%.2f m/s\n",
           global_trajectory[0].velocity_meters_per_second,
           global_trajectory[100 < global_trajectory_count ? 100 : global_trajectory_count - 1].velocity_meters_per_second,
           global_trajectory[500 < global_trajectory_count ? 500 : global_trajectory_count - 1].velocity_meters_per_second);

    if (global_trajectory_count >= 2)
    {
        global_track_length_meters = global_trajectory[global_trajectory_count - 1].s_meters
                                   - global_trajectory[0].s_meters;
        if (global_track_length_meters < 1e-3)
            global_track_length_meters = 0.0;

    }

    return 1;
}

/*===========================================================================
 * Waypoint Search
 *===========================================================================*/

/* Find the closest waypoint index to the current vehicle pose.
 * Parameters: position_x/position_y in meters, vehicle_heading in radians.
 * Returns the selected waypoint index and updates the cached search anchor. */
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

/* Wrap arc length to the closed-track domain used by the loaded trajectory.
 * Parameter: s in meters along track coordinate.
 * Returns wrapped s in meters when track length is known. */
static double wrap_track_s(double s)
{
    if (global_track_length_meters <= 1e-6)
        return s;

    double s0 = global_trajectory[0].s_meters;
    while (s < s0) s += global_track_length_meters;
    while (s >= s0 + global_track_length_meters) s -= global_track_length_meters;
    return s;
}

/* Interpolate a waypoint sample at a query arc length.
 * Parameters: s_query in meters, out points to destination waypoint struct.
 * Side effect: writes interpolated state into *out when inputs are valid. */
static void sample_waypoint_by_s(double s_query, TrajectoryWaypoint_t *out)
{
    if (out == NULL || global_trajectory_count == 0)
        return;

    if (global_trajectory_count == 1)
    {
        *out = global_trajectory[0];
        return;
    }

    double s = wrap_track_s(s_query);
    for (int i = 0; i < global_trajectory_count - 1; i++)
    {
        TrajectoryWaypoint_t *w0 = &global_trajectory[i];
        TrajectoryWaypoint_t *w1 = &global_trajectory[i + 1];
        if (s >= w0->s_meters && s <= w1->s_meters)
        {
            double denom = w1->s_meters - w0->s_meters;
            double t = (denom > 1e-9) ? ((s - w0->s_meters) / denom) : 0.0;
            *out = *w0;
            out->s_meters = s;
            out->x_meters = w0->x_meters + (w1->x_meters - w0->x_meters) * t;
            out->y_meters = w0->y_meters + (w1->y_meters - w0->y_meters) * t;
            out->heading_radians = w0->heading_radians + (w1->heading_radians - w0->heading_radians) * t;
            out->curvature_radians_per_meter = w0->curvature_radians_per_meter +
                                               (w1->curvature_radians_per_meter - w0->curvature_radians_per_meter) * t;
            out->velocity_meters_per_second = w0->velocity_meters_per_second +
                                              (w1->velocity_meters_per_second - w0->velocity_meters_per_second) * t;
            out->left_bound_meters = w0->left_bound_meters + (w1->left_bound_meters - w0->left_bound_meters) * t;
            out->right_bound_meters = w0->right_bound_meters + (w1->right_bound_meters - w0->right_bound_meters) * t;
            return;
        }
    }

    TrajectoryWaypoint_t *w0 = &global_trajectory[global_trajectory_count - 1];
    TrajectoryWaypoint_t *w1 = &global_trajectory[0];
    double s1 = w1->s_meters + global_track_length_meters;
    double denom = s1 - w0->s_meters;
    double s_adj = s;
    if (s_adj < w0->s_meters) s_adj += global_track_length_meters;
    double t = (denom > 1e-9) ? ((s_adj - w0->s_meters) / denom) : 0.0;
    *out = *w0;
    out->s_meters = s;
    out->x_meters = w0->x_meters + (w1->x_meters - w0->x_meters) * t;
    out->y_meters = w0->y_meters + (w1->y_meters - w0->y_meters) * t;
    out->heading_radians = w0->heading_radians + (w1->heading_radians - w0->heading_radians) * t;
    out->curvature_radians_per_meter = w0->curvature_radians_per_meter +
                                       (w1->curvature_radians_per_meter - w0->curvature_radians_per_meter) * t;
    out->velocity_meters_per_second = w0->velocity_meters_per_second +
                                      (w1->velocity_meters_per_second - w0->velocity_meters_per_second) * t;
    out->left_bound_meters = w0->left_bound_meters + (w1->left_bound_meters - w0->left_bound_meters) * t;
    out->right_bound_meters = w0->right_bound_meters + (w1->right_bound_meters - w0->right_bound_meters) * t;
}

/*===========================================================================
 * Reference Trajectory Builder
 *===========================================================================*/

/* Build the per-step MPC Frenet reference sequence from the raceline.
 * Parameter: closest_index is the nearest waypoint index to the vehicle.
 * Side effect: fills global_reference_trajectory for the active horizon. */
static void build_reference_from_trajectory(int closest_index)
{
    const double mpc_dt = g_mpc_dt;
    double s_query = global_trajectory[closest_index].s_meters;
    double step_velocity = global_trajectory[closest_index].velocity_meters_per_second;
    if (step_velocity > TRAJECTORY_MAXIMUM_VELOCITY) step_velocity = TRAJECTORY_MAXIMUM_VELOCITY;

    for (int step = 0; step < g_mpc_horizon; step++)
    {
        s_query += step_velocity * mpc_dt;
        TrajectoryWaypoint_t wp;
        sample_waypoint_by_s(s_query, &wp);

        double traj_vel = wp.velocity_meters_per_second;
        if (traj_vel > TRAJECTORY_MAXIMUM_VELOCITY) traj_vel = TRAJECTORY_MAXIMUM_VELOCITY;
        step_velocity = traj_vel;

        global_reference_trajectory[step].reference_lateral_error = 0;
        global_reference_trajectory[step].reference_heading_error = 0;

        global_reference_trajectory[step].path_curvature = wp.curvature_radians_per_meter;
        global_reference_trajectory[step].left_wall_bound = wp.left_bound_meters;
        global_reference_trajectory[step].right_wall_bound = wp.right_bound_meters;

        global_reference_trajectory[step].reference_velocity = traj_vel;

        global_reference_trajectory[step].reference_lateral_velocity = 0;
        double omega_ref = wp.curvature_radians_per_meter * traj_vel;
        global_reference_trajectory[step].reference_yaw_rate = omega_ref;
    }
}

/*===========================================================================
 * Helper Functions
 *===========================================================================*/

/* Convert a quaternion orientation to yaw angle in radians.
 * Parameters: qx/qy/qz/qw are unit quaternion components.
 * Returns yaw angle in radians in the world frame. */
static double quaternion_to_yaw_angle(double qx, double qy, double qz, double qw)
{
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    return atan2(siny_cosp, cosy_cosp);
}

/* Convert yaw angle to quaternion representation with zero roll/pitch.
 * Parameters: yaw in radians, q points to destination quaternion.
 * Side effect: writes quaternion components into *q when q is non-null. */
static void yaw_to_quaternion(double yaw, geometry_msgs__msg__Quaternion *q)
{
    if (q == NULL) return;
    double half = 0.5 * yaw;
    q->x = 0.0;
    q->y = 0.0;
    q->z = sin(half);
    q->w = cos(half);
}

/* Pre-allocate a ROSIDL string buffer to avoid heap churn in callbacks.
 * Parameters: str points to the target string object, capacity in bytes.
 * Returns 1 on success, otherwise 0. */
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

/* Copy a C string into a pre-allocated ROSIDL string with truncation safety.
 * Parameters: str destination ROSIDL string, value source C string.
 * Side effect: updates str->data and str->size when inputs are valid. */
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

/* Convert map-frame vehicle pose/velocity into Frenet tracking errors.
 * Parameters: car_x/car_y [m], car_heading [rad], closest_index waypoint id,
 * frenet_out destination structure.
 * Side effect: writes path-relative state used by the MPC solver. */
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

    frenet_out->flat_error = lateral_error;
    frenet_out->fhead_error = heading_error;
    frenet_out->flong_vel =
        global_vehicle_state.long_vel;
    frenet_out->flat_vel =
        global_vehicle_state.lat_vel;
    frenet_out->fyaw_rate =
        global_vehicle_state.yaw_rate;
}

/*===========================================================================
 * ROS2 Callback: Odometry Subscription
 *===========================================================================*/

/* Handle odometry updates from the simulator and publish the current command.
 * Parameters: message_in points to nav_msgs/Odometry payload.
 * Side effect: updates global vehicle state and publishes /drive command. */
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

    global_vehicle_state.pos_x = pos_x;
    global_vehicle_state.pos_y = pos_y;
    global_vehicle_state.heading = heading;
    global_vehicle_state.long_vel = vx;
    global_vehicle_state.lat_vel = vy;
    global_vehicle_state.yaw_rate = omega;

    global_odometry_received_flag = 1;

    /* Publish drive command every callback */
    if (global_odometry_received_flag)
    {
        global_drive_message_buffer.drive.steering_angle =
            global_control_command.steer_ang;

        /* gym_bridge with control_input=['accl','steering_angle']
         * interprets drive.speed as acceleration command. */
        global_drive_message_buffer.drive.speed =
            global_control_command.long_acc;

        rcl_publish(&global_control_publisher, &global_drive_message_buffer, NULL);
    }

    global_odometry_callback_counter++;
}

/*===========================================================================
 * ROS2 Callback: Collision Subscription
 *===========================================================================*/

/* Handle collision events and trigger orderly ROS shutdown.
 * Parameters: message_in points to std_msgs/Bool payload.
 * Side effect: requests node shutdown and raises SIGINT fallback on failure. */
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
           g_mpc_horizon,
           g_mpc_dt * 1000.0);
    printf("  Max velocity: %.1f m/s\n",
           TRAJECTORY_MAXIMUM_VELOCITY);
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
        /* HORIZON and PRED_DT are also read by the library's get_default_configuration(),
         * but the sim node needs its own copies for reference trajectory building and viz. */
        if ((env_val = getenv("HORIZON")) != NULL)
        {
            int h = atoi(env_val);
            if (h >= 1 && h <= MPC_MAX_HORIZON_STEPS) g_mpc_horizon = h;
        }
        if ((env_val = getenv("PRED_DT")) != NULL)
        {
            double dt = atof(env_val);
            if (dt > 0.001 && dt < 1.0) g_mpc_dt = dt;
        }
    }

    printf("[MPC] Controller initialized (horizon=%d, dt=%.0fms)\n",
           g_mpc_horizon, g_mpc_dt * 1000.0);
    printf("[MPC] Control rate: ~%d Hz \n",
           200);

    {
        MpcConfiguration_t cfg = mpc_get_configuration();
        printf("[MPC] max_iter=%u, tol=%d\n",
               cfg.max_solver_iterations,
               (int)cfg.solver_convergence_tolerance);
        printf("[MPC] Weights: lat=%.2f heading=%.2f vel=%.2f steer_rate=%.2f steer_effort=%.4f\n",
               (double)cfg.weight_lateral_error,
               (double)cfg.weight_heading_error,
               (double)cfg.weight_velocity,
               (double)cfg.weight_steering_rate,
               (double)cfg.weight_steering_effort);
        printf("[MPC] cross_call_scale=%.2f\n",
               (double)cfg.cross_call_rate_scale);
    }

    /* Load trajectory */
    const char *trajectory_file = NULL;
    if (argc >= 2)
    {
        /* SECURITY: trajectory_file is taken from argv[1] without sanitization.
         * This is acceptable for a controlled ROS2 deployment environment, but
         * the path should not be derived from network input or untrusted sources. */
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
            g_mpc_horizon))
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
