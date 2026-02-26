/**
 * @file mpc_ros2_node.c
 * @brief MPC ROS2 Node for F1/10th Simulator Integration
 *
 * Implements ROS2 node using rclc (C client library) for Jazzy.
 * Subscribes to odometry, runs MPC solver, publishes control commands.
 *
 * Topics:
 *   Subscribe: /ego_racecar/odom (nav_msgs/Odometry)
 *   Publish:   /drive (ackermann_msgs/AckermannDriveStamped)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

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

/* MPC Core Library Headers (Platform-Independent) */
#include "mpc.h"
#include "mpc_types.h"
#include "fp_math.h"
#include "vehicle_model.h"

/*===========================================================================
 * Configuration Constants
 *===========================================================================*/

/** Number of MPC prediction steps (40 steps × 0.05s = 2.0 second lookahead) */
#define MPC_PREDICTION_HORIZON_STEPS 40

/** Time step between predictions (seconds) - MUST match MPC_DEFAULT_TIME_STEP_SECONDS in mpc_types.h
 *  50 ms for prediction lookahead. MPC is called at ~250 Hz (divider=1) but
 *  predicts 2.0s ahead with 40 steps. cross_call_rate_scale handles the mismatch. */
#define MPC_TIME_STEP_SECONDS 0.05f

/** Maximum allowed steering angle (radians, ~24.5 degrees) [TESTED] */
#define MAXIMUM_STEERING_ANGLE_RADIANS 0.4282f

/** Maximum allowed velocity (m/s) */
#define MAXIMUM_VELOCITY_METERS_PER_SECOND 20.0f

/** Wheel radius [m] — Traxxas Slash 4x4 VXL [MEASURED: 0.051 m loaded] */
#define WHEEL_RADIUS_METERS 0.051

/** Convert vehicle velocity to matching wheel speed (zero slip ratio) */
#define VX_TO_WHEEL_SPEED(vx) ((vx) / WHEEL_RADIUS_METERS)

/** Odometry callback divider (run MPC every N callbacks)
 *
 * Set to 1 to run MPC at every odometry callback (~250 Hz).
 * The within-horizon rate penalty (between steps k and k+1) provides
 * smoothing regardless of call frequency. The cross-call rate penalty
 * (u[0] vs u_prev) is proportionally weaker at higher frequencies
 * since each call has smaller u differences, which is acceptable.
 */
#define ODOMETRY_CALLBACK_DIVIDER_DEFAULT 1
static int g_odom_divider = ODOMETRY_CALLBACK_DIVIDER_DEFAULT;

/** Maximum number of waypoints in loaded trajectory */
#define TRAJECTORY_MAXIMUM_WAYPOINTS 2000

/** Maximum reference velocity [m/s] (clamp trajectory velocities) */
#define TRAJECTORY_MAXIMUM_VELOCITY 20.0

/** Speed gain applied to trajectory velocities (1.0 = full optimal racing speed) */
#define TRAJECTORY_SPEED_GAIN 1.0

/** Maximum longitudinal acceleration for reference velocity ramp [m/s²]
 *  Raised from 2.6 to 5.0 to match physical capability (measured 6-9.5 m/s²).
 *  The MPC's internal torque constraints (±22.9 N·m) provide the real limit. */
#define MAX_REFERENCE_ACCELERATION 5.0

/*===========================================================================
 * Trajectory Waypoint (loaded from CSV, stored as double)
 *===========================================================================*/

/**
 * Single waypoint from the trajectory CSV file.
 * Stored in double precision (only converted to fixed-point for MPC).
 */
typedef struct
{
    double x_meters;
    double y_meters;
    double heading_radians;
    double velocity_meters_per_second;
    double curvature_radians_per_meter;  /**< Trajectory curvature (kappa) */
    double left_bound_meters;            /**< Distance to left/outer wall [m] */
    double right_bound_meters;           /**< Distance to right/inner wall [m] */
} TrajectoryWaypoint_t;

/*===========================================================================
 * Global State Variables
 *===========================================================================*/

/** Runtime-tunable parameters (initialized from environment variables) */
static double g_min_speed_for_mpc = 0.5;    /**< Min speed before MPC engages (m/s) */
static double g_max_speed = 20.0;           /**< Global speed cap (m/s) */
static double g_speed_ramp = 1.0;           /**< Max speed increase per MPC step (m/s) */
static double g_feedforward_gain = 0.0;     /**< Curvature feedforward gain (0=off, 1=full). OFF: MPC Frenet model handles curvature via A matrix */
static double g_max_lateral_accel = 7.0;    /**< Max lateral accel for curvature speed limit (m/s²), measured on real car */
static double g_curvature_lookahead = 150;  /**< Waypoints ahead to scan for curvature (for speed limit) */

/** Loaded trajectory waypoints */
static TrajectoryWaypoint_t global_trajectory[TRAJECTORY_MAXIMUM_WAYPOINTS];

/** Number of loaded waypoints */
static int global_trajectory_count = 0;

/** Index of last closest waypoint (for efficient search) */
static int global_last_closest_index = 0;

/** Current vehicle state from odometry */
static VehicleState_t global_vehicle_state = {0};

/** Current Frenet state (path-relative, for MPC) */
static FrenetState_t global_frenet_state = {0};

/** Current control command to publish */
static ControlInput_t global_control_command = {0};

/** Flag: have we received at least one odometry message? */
static int global_odometry_received_flag = 0;

/** Counter for odometry callbacks (used for rate limiting MPC) */
static int global_odometry_callback_counter = 0;

/** ROS2 publisher handle for control commands */
static rcl_publisher_t global_control_publisher;

/** ROS2 publisher handle for MPC reference path visualization */
static rcl_publisher_t global_reference_path_publisher;

/** ROS2 publisher handle for full trajectory visualization */
static rcl_publisher_t global_trajectory_path_publisher;

/** Buffer for incoming odometry message */
static nav_msgs__msg__Odometry global_odometry_message_buffer;

/** Buffer for outgoing drive command message */
static ackermann_msgs__msg__AckermannDriveStamped global_drive_message_buffer;

/** Buffer for outgoing reference path message */
static nav_msgs__msg__Path global_reference_path_message;

/** Buffer for full trajectory path message */
static nav_msgs__msg__Path global_trajectory_path_message;

/** Reference trajectory for MPC */
static TrajectoryReferencePoint_t global_reference_trajectory[MPC_PREDICTION_HORIZON_STEPS];

/*===========================================================================
 * Trajectory Loading (CSV from f1tenth_planning)
 *===========================================================================*/

/**
 * @brief Load trajectory from CSV file
 *
 * CSV format (TUM compatible):
 *   # s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2
 *
 * Velocities are scaled by TRAJECTORY_SPEED_GAIN and clamped
 * to TRAJECTORY_MAXIMUM_VELOCITY.
 *
 * @param file_path Path to the CSV trajectory file
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
        /* Skip comment/header lines */
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

        /* Parse: s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2[, left_bound, right_bound] */
        double s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2;
        double left_bound = 5.0, right_bound = 5.0;  /* Default: 5m if not in CSV */
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

            /* Scale and clamp velocity (trajectory is for full-scale car) */
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
    
    /* Debug: Print some sample velocities */
    printf("[MPC] Sample velocities: wp[0]=%.2f, wp[100]=%.2f, wp[500]=%.2f m/s\n",
           global_trajectory[0].velocity_meters_per_second,
           global_trajectory[100].velocity_meters_per_second,
           global_trajectory[500 < global_trajectory_count ? 500 : global_trajectory_count-1].velocity_meters_per_second);

    return 1;
}

/**
 * @brief Find the closest trajectory waypoint to a position (considering heading)
 *
 * Uses a windowed search around the last closest index for efficiency.
 * The trajectory is assumed to be a closed loop.
 * Waypoints behind the vehicle (based on heading) are skipped to prevent
 * tracking problems when the car faces the wrong direction.
 *
 * @param position_x Vehicle X position [meters]
 * @param position_y Vehicle Y position [meters]
 * @param vehicle_heading Vehicle heading angle [radians]
 * @return Index of the closest waypoint ahead of the vehicle
 */
static int find_closest_waypoint(double position_x, double position_y, double vehicle_heading)
{
    if (global_trajectory_count == 0)
    {
        return 0;
    }

    /* Always search the FULL trajectory to avoid getting stuck at wrong index.
     * For ~300 waypoints this is negligible at 250 Hz. */
    int best_index = global_last_closest_index;
    double best_distance_squared = 1e18;

    for (int i = 0; i < global_trajectory_count; i++)
    {
        double dx = global_trajectory[i].x_meters - position_x;
        double dy = global_trajectory[i].y_meters - position_y;
        double distance_squared = dx * dx + dy * dy;

        if (distance_squared < best_distance_squared)
        {
            best_distance_squared = distance_squared;
            best_index = i;
        }
    }

    /* Among points at similar distance (within 1m), prefer the one
     * whose trajectory heading best matches vehicle heading.
     * This handles corners where multiple waypoints are equidistant. */
    double best_dist = sqrt(best_distance_squared);
    double best_heading_score = 1e18;
    int refined_index = best_index;

    for (int offset = -5; offset <= 15; offset++)
    {
        int idx = (best_index + offset) % global_trajectory_count;
        if (idx < 0) idx += global_trajectory_count;

        double dx = global_trajectory[idx].x_meters - position_x;
        double dy = global_trajectory[idx].y_meters - position_y;
        double dist = sqrt(dx * dx + dy * dy);

        /* Only consider points within 1m of the closest */
        if (dist > best_dist + 1.0) continue;

        /* Check if this waypoint is roughly ahead (dot product with vehicle dir) */
        double veh_dx = cos(vehicle_heading);
        double veh_dy = sin(vehicle_heading);
        double dot = dx * veh_dx + dy * veh_dy;

        /* Score: distance + penalty for being behind */
        double score = dist + ((dot < 0.0) ? 2.0 : 0.0);

        if (score < best_heading_score)
        {
            best_heading_score = score;
            refined_index = idx;
        }
    }

    global_last_closest_index = refined_index;
    return refined_index;
}

/**
 * @brief Build MPC reference trajectory from loaded waypoints (Frenet frame)
 *
 * In the Frenet frame, the reference lateral error and heading error are
 * always zero (we want to follow the path). The important outputs are:
 *   - path curvature (for linearization)
 *   - wall bounds (for constraints)
 *   - velocity / yaw rate / wheel speed references
 *
 * For each MPC prediction step, finds the waypoint at the expected
 * travel distance based on the REFERENCE velocity (not current velocity).
 *
 * @param closest_index Index of the closest waypoint
 * @param current_velocity Current vehicle velocity (m/s) - used for reference velocity ramp
 */
static void build_reference_from_trajectory(int closest_index, double current_velocity)
{
    /* MPC time step in seconds */
    const double mpc_dt = (double)MPC_TIME_STEP_SECONDS;

    /* Average waypoint spacing (assumed constant from trajectory file) */
    const double avg_waypoint_spacing = 0.346;  /* meters - from Spielberg trajectory */

    for (int step = 0; step < MPC_PREDICTION_HORIZON_STEPS; step++)
    {
        /* Use the reference trajectory velocity for lookahead distance computation.
         * NOTE: velocity in global_trajectory is already scaled during CSV loading. */
        int base_waypoint_index = (closest_index + step) % global_trajectory_count;
        double ref_velocity = global_trajectory[base_waypoint_index].velocity_meters_per_second;

        /* Minimum lookahead velocity - ensures we see corners coming even when slow.
         * With 10 steps x 0.05s x 3.0 m/s = 1.5 meter minimum lookahead */
        if (ref_velocity < 3.0) ref_velocity = 3.0;

        /* Clamp to max velocity */
        if (ref_velocity > TRAJECTORY_MAXIMUM_VELOCITY)
        {
            ref_velocity = TRAJECTORY_MAXIMUM_VELOCITY;
        }

        /* Distance we expect to travel from closest point to this prediction step */
        double expected_distance = ref_velocity * mpc_dt * (step + 1);

        /* How many waypoints ahead corresponds to this distance? */
        int waypoints_ahead = (int)(expected_distance / avg_waypoint_spacing);
        if (waypoints_ahead < step + 1) waypoints_ahead = step + 1;

        /* Wrap around for closed loop */
        int waypoint_index = (closest_index + waypoints_ahead) % global_trajectory_count;

        TrajectoryWaypoint_t *wp = &global_trajectory[waypoint_index];

        /* === Frenet references: track-following means zero lateral/heading error === */
        global_reference_trajectory[step].reference_lateral_error_meters = 0;
        global_reference_trajectory[step].reference_heading_error_radians = 0;

        /* === Path geometry: curvature and wall bounds === */
        global_reference_trajectory[step].path_curvature_radians_per_meter =
            DOUBLE_TO_FP(wp->curvature_radians_per_meter);
        global_reference_trajectory[step].left_wall_bound_meters =
            DOUBLE_TO_FP(wp->left_bound_meters);
        global_reference_trajectory[step].right_wall_bound_meters =
            DOUBLE_TO_FP(wp->right_bound_meters);

        /* === Velocity reference (capped to achievable ramp) === */
        double traj_vel = global_trajectory[base_waypoint_index].velocity_meters_per_second;
        double max_achievable = current_velocity + MAX_REFERENCE_ACCELERATION * mpc_dt * (step + 1);
        if (traj_vel > max_achievable) traj_vel = max_achievable;
        if (traj_vel < 0.0) traj_vel = 0.0;
        global_reference_trajectory[step].reference_velocity_meters_per_second =
            DOUBLE_TO_FP(traj_vel);

        /* Lateral velocity reference: 0 (no sideslip desired) */
        global_reference_trajectory[step].reference_lateral_velocity_meters_per_second = 0;

        /* Yaw rate reference: will be computed in second pass from heading differences */
        global_reference_trajectory[step].reference_yaw_rate_radians_per_second = 0;

        /* Wheel speed reference: match capped velocity for zero slip ratio */
        double ref_ww = (traj_vel > 0.01) ? VX_TO_WHEEL_SPEED(traj_vel) : 0.0;
        global_reference_trajectory[step].reference_wheel_speed_radians_per_second =
            DOUBLE_TO_FP(ref_ww);
    }

    /* Second pass: yaw rate reference = κ · v_ref (steady-state cornering).
     * Previously set to 0 to avoid aggressive curve-entry steering caused by
     * inflated B[4][0] in the old linear tire model. With the Pacejka model
     * now providing correct (reduced) B-matrix gains at high slip angles,
     * setting the physically correct ω_ref = κ · v improves steady-state
     * curve tracking without causing yaw cascade.
     * The small weight_yaw_rate (0.05) keeps this a gentle guide, not a
     * hard target — heading error still drives the primary cornering. */
    for (int step = 0; step < MPC_PREDICTION_HORIZON_STEPS; step++)
    {
        double kappa = FP_TO_DOUBLE(global_reference_trajectory[step].path_curvature_radians_per_meter);
        double v_ref = FP_TO_DOUBLE(global_reference_trajectory[step].reference_velocity_meters_per_second);
        double omega_ref = kappa * v_ref;
        global_reference_trajectory[step].reference_yaw_rate_radians_per_second =
            DOUBLE_TO_FP(omega_ref);
    }
}

/*===========================================================================
 * Helper Functions
 *===========================================================================*/

/**
 * @brief Convert quaternion orientation to yaw angle
 *
 * Extracts yaw (rotation about Z axis) from quaternion using:
 *   yaw = atan2(2*(w*z + x*y), 1 - 2*(y² + z²))
 *
 * @param quaternion_x Quaternion X component
 * @param quaternion_y Quaternion Y component
 * @param quaternion_z Quaternion Z component
 * @param quaternion_w Quaternion W component
 * @return Yaw angle in radians
 */
static double quaternion_to_yaw_angle(double quaternion_x,
                                      double quaternion_y,
                                      double quaternion_z,
                                      double quaternion_w)
{
    double sine_yaw_times_cosine_pitch = 2.0 * (quaternion_w * quaternion_z +
                                                quaternion_x * quaternion_y);
    double cosine_yaw_times_cosine_pitch = 1.0 - 2.0 * (quaternion_y * quaternion_y +
                                                        quaternion_z * quaternion_z);
    return atan2(sine_yaw_times_cosine_pitch, cosine_yaw_times_cosine_pitch);
}

/**
 * @brief Convert yaw angle to quaternion (Z-rotation)
 *
 * @param yaw_radians Yaw angle in radians
 * @param quaternion Output quaternion message
 */
static void yaw_to_quaternion(double yaw_radians, geometry_msgs__msg__Quaternion *quaternion)
{
    if (quaternion == NULL)
    {
        return;
    }

    double half_yaw = 0.5 * yaw_radians;
    quaternion->x = 0.0;
    quaternion->y = 0.0;
    quaternion->z = sin(half_yaw);
    quaternion->w = cos(half_yaw);
}

/**
 * @brief Saturate control commands to safe limits
 *
 * @param steering_angle_radians Steering angle to saturate (modified in-place)
 * @param velocity_mps Velocity to saturate (modified in-place)
 */
/* Maximum steering rate per MPC step (rad).  Tunable via MPC_STEER_RATE_LIMIT.
 * This is a SAFETY-ONLY clamp — the MPC's internal weight_steering_rate
 * (0.5) is the primary steering-rate controller.  The external clamp must
 * be wide enough to never override the MPC plan, otherwise the two rate-
 * limiting systems fight each other, causing oscillation at straight→curve
 * transitions.  Default 0.40 rad/step ≈ full lock-to-lock in ~1 step. */
static double g_steer_rate_limit = 0.40;
static double previous_steering_command = 0.0;

static void saturate_control_commands(double *steering_angle_radians,
                                      double *velocity_mps)
{
    /* Steering rate clamp: safety-only — should rarely activate.
     * The MPC internal rate penalty (weight_steering_rate) handles
     * smooth steering transitions.  This clamp only catches extreme
     * solver failures or transients (e.g., low-speed→MPC handover). */
    double delta_steer = *steering_angle_radians - previous_steering_command;
    if (delta_steer > g_steer_rate_limit)
    {
        *steering_angle_radians = previous_steering_command + g_steer_rate_limit;
    }
    else if (delta_steer < -g_steer_rate_limit)
    {
        *steering_angle_radians = previous_steering_command - g_steer_rate_limit;
    }
    previous_steering_command = *steering_angle_radians;

    /* Steering magnitude clamp */
    if (*steering_angle_radians > MAXIMUM_STEERING_ANGLE_RADIANS)
    {
        *steering_angle_radians = MAXIMUM_STEERING_ANGLE_RADIANS;
    }
    if (*steering_angle_radians < -MAXIMUM_STEERING_ANGLE_RADIANS)
    {
        *steering_angle_radians = -MAXIMUM_STEERING_ANGLE_RADIANS;
    }
    if (*velocity_mps > MAXIMUM_VELOCITY_METERS_PER_SECOND)
    {
        *velocity_mps = MAXIMUM_VELOCITY_METERS_PER_SECOND;
    }
    if (*velocity_mps < 0.0)
    {
        *velocity_mps = 0.0;
    }
}

/**
 * @brief Pre-allocate a rosidl string with a fixed capacity (static memory model)
 *
 * @param str Pointer to rosidl string
 * @param capacity Desired capacity (must be > 1)
 * @return 1 on success, 0 on failure
 */
static int preallocate_rosidl_string(rosidl_runtime_c__String *str, size_t capacity)
{
    if (str == NULL || capacity <= 1)
    {
        return 0;
    }

    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    char *data = (char *)allocator.allocate(capacity, allocator.state);
    if (data == NULL)
    {
        return 0;
    }

    data[0] = '\0';
    str->data = data;
    str->size = 0;
    str->capacity = capacity;
    return 1;
}

/**
 * @brief Set a rosidl string value without reallocation
 *
 * @param str Pointer to rosidl string (must be pre-allocated)
 * @param value Null-terminated string value to set
 */
static void set_rosidl_string(rosidl_runtime_c__String *str, const char *value)
{
    if (str == NULL || str->data == NULL || value == NULL)
    {
        return;
    }

    size_t length = strlen(value);
    if (length >= str->capacity)
    {
        length = str->capacity - 1;
    }

    memcpy(str->data, value, length);
    str->data[length] = '\0';
    str->size = length;
}

/*===========================================================================
 * Frenet State Conversion
 *===========================================================================*/

/**
 * @brief Convert global vehicle state to Frenet (path-relative) state
 *
 * Computes the Frenet state [e_y, e_psi, v_x, v_y, omega, omega_w] from
 * the global vehicle pose and the closest trajectory waypoint.
 *
 * e_y (lateral error) = signed perpendicular distance from car to path
 *   Positive = car is LEFT of path
 *   Negative = car is RIGHT of path
 *
 * e_psi (heading error) = vehicle heading - path tangent heading
 *   Positive = vehicle is turned left relative to path
 *
 * v_x, v_y, omega, omega_w are unchanged (already in body frame).
 *
 * @param car_x         Vehicle X position [meters]
 * @param car_y         Vehicle Y position [meters]
 * @param car_heading   Vehicle heading [radians]
 * @param closest_index Index of closest trajectory waypoint
 * @param frenet_out    Output: Frenet state
 */
static void convert_to_frenet_state(
    double car_x, double car_y, double car_heading,
    int closest_index,
    FrenetState_t *frenet_out)
{
    /* Get path point and heading at closest waypoint */
    double path_x = global_trajectory[closest_index].x_meters;
    double path_y = global_trajectory[closest_index].y_meters;
    double path_heading = global_trajectory[closest_index].heading_radians;

    /* Lateral error: signed perpendicular distance from car to path
     * Using the path normal direction:
     *   normal = (-sin(path_heading), cos(path_heading))
     *   e_y = (car - path) · normal
     *   Positive = left of path, Negative = right of path */
    double dx = car_x - path_x;
    double dy = car_y - path_y;
    double lateral_error = -dx * sin(path_heading) + dy * cos(path_heading);

    /* Heading error: vehicle heading minus path heading, wrapped to [-pi, pi] */
    double heading_error = car_heading - path_heading;
    while (heading_error > 3.14159265) heading_error -= 2.0 * 3.14159265;
    while (heading_error < -3.14159265) heading_error += 2.0 * 3.14159265;

    /* Fill Frenet state */
    frenet_out->lateral_error_meters = DOUBLE_TO_FP(lateral_error);
    frenet_out->heading_error_radians = DOUBLE_TO_FP(heading_error);

    /* Body-frame states: copy directly from global vehicle state */
    frenet_out->longitudinal_velocity_meters_per_second =
        global_vehicle_state.longitudinal_velocity_meters_per_second;
    frenet_out->lateral_velocity_meters_per_second =
        global_vehicle_state.lateral_velocity_meters_per_second;
    frenet_out->yaw_rate_radians_per_second =
        global_vehicle_state.yaw_rate_radians_per_second;
    frenet_out->wheel_speed_radians_per_second =
        global_vehicle_state.wheel_speed_radians_per_second;
}

/*===========================================================================
 * ROS2 Callback: Odometry Subscription
 *===========================================================================*/

/**
 * @brief Callback for odometry messages
 *
 * Extracts vehicle state from odometry message and runs MPC solver
 * at a reduced rate (every ODOMETRY_CALLBACK_DIVIDER callbacks).
 *
 * @param message_in Pointer to incoming nav_msgs/Odometry message
 */
void odometry_subscription_callback(const void *message_in)
{
    if (message_in == NULL)
    {
        fprintf(stderr, "[MPC] ERROR: Odometry callback received NULL message\n");
        return;
    }

    const nav_msgs__msg__Odometry *odometry_message =
        (const nav_msgs__msg__Odometry *)message_in;

    /* Extract position from odometry */
    double position_x_meters = odometry_message->pose.pose.position.x;
    double position_y_meters = odometry_message->pose.pose.position.y;

    /* Extract yaw angle from quaternion orientation */
    double heading_angle_radians = quaternion_to_yaw_angle(
        odometry_message->pose.pose.orientation.x,
        odometry_message->pose.pose.orientation.y,
        odometry_message->pose.pose.orientation.z,
        odometry_message->pose.pose.orientation.w);

    /* Extract velocity */
    double velocity_x_meters_per_second = odometry_message->twist.twist.linear.x;
    double velocity_y_meters_per_second = odometry_message->twist.twist.linear.y;
    double yaw_rate_radians_per_second = odometry_message->twist.twist.angular.z;

    /* Update global vehicle state (convert to fixed-point)
     * For dynamic model: v_x and v_y are body-frame velocities.
     * The odometry twist is already in body frame. */
    global_vehicle_state.position_x_meters = DOUBLE_TO_FP(position_x_meters);
    global_vehicle_state.position_y_meters = DOUBLE_TO_FP(position_y_meters);
    global_vehicle_state.heading_angle_radians = DOUBLE_TO_FP(heading_angle_radians);
    global_vehicle_state.longitudinal_velocity_meters_per_second = DOUBLE_TO_FP(velocity_x_meters_per_second);
    global_vehicle_state.lateral_velocity_meters_per_second = DOUBLE_TO_FP(velocity_y_meters_per_second);
    global_vehicle_state.yaw_rate_radians_per_second = DOUBLE_TO_FP(yaw_rate_radians_per_second);

    /* Wheel speed: compute from longitudinal velocity assuming zero slip ratio.
     * In practice, the VESC maintains wheel speed close to commanded speed,
     * so ωw ≈ vx / Rw is a reasonable estimate when no direct wheel speed sensor is available. */
    double wheel_speed_estimate = (velocity_x_meters_per_second > 0.01)
        ? VX_TO_WHEEL_SPEED(velocity_x_meters_per_second) : 0.0;
    global_vehicle_state.wheel_speed_radians_per_second = DOUBLE_TO_FP(wheel_speed_estimate);

    global_odometry_received_flag = 1;

    /* Run MPC solver at reduced rate */
    if ((global_odometry_callback_counter % g_odom_divider) == 0)
    {
        printf("[MPC] State: x=%.2f m, y=%.2f m, θ=%.2f rad, vx=%.2f m/s, vy=%.2f m/s, ω=%.2f rad/s, ωw=%.1f rad/s\n",
               position_x_meters, position_y_meters, heading_angle_radians,
               velocity_x_meters_per_second, velocity_y_meters_per_second, yaw_rate_radians_per_second,
               wheel_speed_estimate);

        /*
         * Build reference trajectory from loaded waypoints.
         * Find closest waypoint (considering heading), then extract N-step reference with lookahead.
         */
        if (global_trajectory_count > 0)
        {
            int closest_index = find_closest_waypoint(position_x_meters, position_y_meters, heading_angle_radians);
            /* Compute velocity magnitude for waypoint lookup and reference building */
            double velocity_magnitude = sqrt(velocity_x_meters_per_second * velocity_x_meters_per_second +
                                             velocity_y_meters_per_second * velocity_y_meters_per_second);
            build_reference_from_trajectory(closest_index, velocity_magnitude);

            /* Convert global pose to Frenet state for MPC */
            convert_to_frenet_state(position_x_meters, position_y_meters,
                                    heading_angle_radians, closest_index,
                                    &global_frenet_state);

            /* Debug: Print Frenet state + reference for diagnostics */
            printf("[MPC] Car: x=%.2f, y=%.2f, θ=%.3f, v=%.2f | idx=%d\n",
                   position_x_meters, position_y_meters, heading_angle_radians, velocity_magnitude,
                   closest_index);
            {
                double e_y = FP_TO_DOUBLE(global_frenet_state.lateral_error_meters);
                double e_psi = FP_TO_DOUBLE(global_frenet_state.heading_error_radians);
                double ref_v = FP_TO_DOUBLE(global_reference_trajectory[0].reference_velocity_meters_per_second);
                double kappa = FP_TO_DOUBLE(global_reference_trajectory[0].path_curvature_radians_per_meter);
                double lw = FP_TO_DOUBLE(global_reference_trajectory[0].left_wall_bound_meters);
                double rw = FP_TO_DOUBLE(global_reference_trajectory[0].right_wall_bound_meters);
                printf("[MPC] Frenet: e_y=%.3f m, e_psi=%.3f rad, v_ref=%.2f | kappa=%.3f, walls=[%.2f, %.2f]\n",
                       e_y, e_psi, ref_v, kappa, lw, rw);
            }

            /* Publish reference path for visualization */
            global_reference_path_message.header.stamp = odometry_message->header.stamp;
            global_reference_path_message.poses.size = MPC_PREDICTION_HORIZON_STEPS;

            /* Visualization: reconstruct reference path from trajectory waypoints.
             * Since Frenet references don't store global XY, recompute waypoint
             * indices using the same lookahead logic as build_reference_from_trajectory. */
            {
                const double mpc_dt_viz = (double)MPC_TIME_STEP_SECONDS;
                const double avg_spacing_viz = 0.346;
                for (int step = 0; step < MPC_PREDICTION_HORIZON_STEPS; step++)
                {
                    geometry_msgs__msg__PoseStamped *pose =
                        &global_reference_path_message.poses.data[step];

                    int base_idx = (closest_index + step) % global_trajectory_count;
                    double rv = global_trajectory[base_idx].velocity_meters_per_second;
                    if (rv < 3.0) rv = 3.0;
                    if (rv > TRAJECTORY_MAXIMUM_VELOCITY) rv = TRAJECTORY_MAXIMUM_VELOCITY;
                    int wpa = (int)(rv * mpc_dt_viz * (step + 1) / avg_spacing_viz);
                    if (wpa < step + 1) wpa = step + 1;
                    int wp_idx = (closest_index + wpa) % global_trajectory_count;

                    pose->pose.position.x = global_trajectory[wp_idx].x_meters;
                    pose->pose.position.y = global_trajectory[wp_idx].y_meters;
                    pose->pose.position.z = 0.0;

                    yaw_to_quaternion(global_trajectory[wp_idx].heading_radians,
                                      &pose->pose.orientation);
                }
            }

            rcl_ret_t path_publish_result = rcl_publish(
                &global_reference_path_publisher,
                &global_reference_path_message,
                NULL);
            if (path_publish_result != RCL_RET_OK)
            {
                fprintf(stderr, "[MPC] WARNING: Failed to publish reference path: %s\n",
                        rcl_get_error_string().str);
            }

            /* Publish full trajectory path occasionally for RViz */
            if ((global_odometry_callback_counter % g_odom_divider) == 0)
            {
                global_trajectory_path_message.header.stamp = odometry_message->header.stamp;
                rcl_ret_t trajectory_publish_result = rcl_publish(
                    &global_trajectory_path_publisher,
                    &global_trajectory_path_message,
                    NULL);
                if (trajectory_publish_result != RCL_RET_OK)
                {
                    fprintf(stderr, "[MPC] WARNING: Failed to publish trajectory path: %s\n",
                            rcl_get_error_string().str);
                }
            }
        }
        else
        {
            /* Fallback: drive straight at low speed if no trajectory loaded.
             * In Frenet frame: zero lateral/heading error, straight path (kappa=0),
             * wide wall bounds, and low velocity reference. */
            fixed_point_t target_velocity = DOUBLE_TO_FP(1.0);

            /* Set Frenet state to zero errors for fallback */
            global_frenet_state.lateral_error_meters = 0;
            global_frenet_state.heading_error_radians = 0;
            global_frenet_state.longitudinal_velocity_meters_per_second =
                global_vehicle_state.longitudinal_velocity_meters_per_second;
            global_frenet_state.lateral_velocity_meters_per_second =
                global_vehicle_state.lateral_velocity_meters_per_second;
            global_frenet_state.yaw_rate_radians_per_second =
                global_vehicle_state.yaw_rate_radians_per_second;
            global_frenet_state.wheel_speed_radians_per_second =
                global_vehicle_state.wheel_speed_radians_per_second;

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
                global_reference_trajectory[step].reference_wheel_speed_radians_per_second =
                    DOUBLE_TO_FP(VX_TO_WHEEL_SPEED(1.0));
            }
        }

        /* Run MPC to compute optimal control */
        MpcSolverResult_t mpc_result;
        MpcSolverStatus_t mpc_status;
        
        /* LOW-SPEED GUARD: At very low speed (< 0.5 m/s), the dynamic model
         * linearization is degenerate (vx appears in denominator of tire
         * slip angle). The QP produces garbage steering that accumulates
         * heading errors.  Instead: steer straight and accelerate. */
        double current_vx = FP_TO_DOUBLE(
            global_vehicle_state.longitudinal_velocity_meters_per_second);
        const double MIN_SPEED_FOR_MPC = g_min_speed_for_mpc;
        
        if (fabs(current_vx) < MIN_SPEED_FOR_MPC)
        {
            /* Bypass MPC: drive straight, accelerate to trajectory velocity */
            double trajectory_ref_velocity =
                global_trajectory[global_last_closest_index].velocity_meters_per_second;
            double velocity_command_mps = trajectory_ref_velocity;
            double v_cmd_ceiling = fabs(current_vx) + g_speed_ramp;
            if (velocity_command_mps > v_cmd_ceiling)
                velocity_command_mps = v_cmd_ceiling;
            if (velocity_command_mps > g_max_speed)
                velocity_command_mps = g_max_speed;
            if (velocity_command_mps < 0.0) velocity_command_mps = 0.0;
            
            /* Steer toward trajectory heading (simple proportional) */
            double traj_heading = global_trajectory[global_last_closest_index].heading_radians;
            double heading_error = traj_heading - heading_angle_radians;
            /* Wrap to ±π */
            while (heading_error > 3.14159) heading_error -= 2.0 * 3.14159;
            while (heading_error < -3.14159) heading_error += 2.0 * 3.14159;
            double low_speed_steer = 0.5 * heading_error; /* proportional gain */
            if (low_speed_steer > 0.2) low_speed_steer = 0.2;
            if (low_speed_steer < -0.2) low_speed_steer = -0.2;
            
            /* Update the rate-limiter's previous steering so MPC transition
             * is smooth (no jump from 0 to MPC's first output) */
            previous_steering_command = low_speed_steer;

            global_control_command.steering_angle_radians =
                DOUBLE_TO_FP(low_speed_steer);
            global_control_command.motor_torque_newton_meters =
                DOUBLE_TO_FP(velocity_command_mps);
            
            printf("[MPC] LOW-SPEED: steer=%.3f rad, v_cmd=%.2f m/s (vx=%.2f, θ_err=%.3f)\n",
                   low_speed_steer, velocity_command_mps, current_vx, heading_error);
            fflush(stdout);
        }
        else
        {
        struct timespec mpc_t0, mpc_t1;
        clock_gettime(CLOCK_MONOTONIC, &mpc_t0);
        mpc_status = mpc_compute_optimal_control(
            &global_frenet_state,
            global_reference_trajectory,
            &mpc_result);
        clock_gettime(CLOCK_MONOTONIC, &mpc_t1);
        double mpc_solve_us = (mpc_t1.tv_sec - mpc_t0.tv_sec) * 1e6 +
                              (mpc_t1.tv_nsec - mpc_t0.tv_nsec) / 1e3;

        if (mpc_status == MPC_STATUS_SUCCESS ||
            mpc_status == MPC_STATUS_MAXIMUM_ITERATIONS_REACHED)
        {
            /* Extract steering control from MPC result */
            double steering_command_radians = FP_TO_DOUBLE(
                mpc_result.optimal_control.steering_angle_radians);
            
            /* Extract motor torque from MPC result (for logging).
             * The 6-state Frenet MPC optimizes [steering, motor_torque].
             */
            double torque_command_nm = FP_TO_DOUBLE(
                mpc_result.optimal_control.motor_torque_newton_meters);
            
            /* Velocity command for the simulator:
             * The sim uses a PID speed controller, so we send the trajectory
             * reference velocity directly. The MPC's torque channel is used
             * internally for prediction, not as the speed command.
             *
             * For the REAL CAR with VESC, the torque → velocity conversion
             * should be used instead:
             *   wheel_accel = T_motor * G / Iw
             *   delta_vx = Rw * wheel_accel * dt
             *   v_cmd = current_vx + delta_vx
             */
            /* current_vx already declared above for low-speed guard */
            
            /* Speed command for the simulator:
             * Use the trajectory reference velocity, with a progressive ramp
             * to prevent sending 20 m/s from standstill. The sim has its own
             * PID speed controller with acceleration limits, so we can be
             * more aggressive than the MPC internal ramp. */
            double trajectory_ref_velocity =
                global_trajectory[global_last_closest_index].velocity_meters_per_second;

            /* Curvature-based speed limit with proper braking profile.
             * For each waypoint ahead, compute the max cornering speed:
             *   v_corner = sqrt(a_lat_max / |kappa|)
             * Then back-calculate the max speed NOW using kinematics:
             *   v_now = sqrt(v_corner² + 2 * a_brake * distance)
             * This means: go fast on straights, brake smoothly into corners. */
            {
                const double a_brake = 5.0; /* braking deceleration (m/s²) */
                const double wp_spacing = 0.346; /* approximate waypoint spacing (m) */
                double v_limit = 1e6; /* start with no limit */
                int lookahead = (int)g_curvature_lookahead;
                for (int la = 0; la <= lookahead; la++)
                {
                    int wp_idx = (global_last_closest_index + la) % global_trajectory_count;
                    double k = fabs(global_trajectory[wp_idx].curvature_radians_per_meter);
                    if (k < 0.001) k = 0.001; /* prevent div-by-zero */
                    double v_corner = sqrt(g_max_lateral_accel / k);
                    double dist = la * wp_spacing;
                    /* v_now² = v_corner² + 2*a_brake*dist */
                    double v_now = sqrt(v_corner * v_corner + 2.0 * a_brake * dist);
                    if (v_now < v_limit) v_limit = v_now;
                }
                if (trajectory_ref_velocity > v_limit)
                    trajectory_ref_velocity = v_limit;
            }

            double velocity_command_mps = trajectory_ref_velocity;
            double v_cmd_ceiling = current_vx + g_speed_ramp;
            if (velocity_command_mps > v_cmd_ceiling)
                velocity_command_mps = v_cmd_ceiling;
            if (velocity_command_mps > g_max_speed)
                velocity_command_mps = g_max_speed;
            if (velocity_command_mps < 0.0) velocity_command_mps = 0.0;
            
            double distance_from_trajectory = fabs(
                FP_TO_DOUBLE(global_frenet_state.lateral_error_meters));

            /* Curvature feedforward steering: pre-steer for the current
             * trajectory curvature so MPC only handles corrections.
             * δ_ff = atan(L * κ) where L=0.3302m is the wheelbase. */
            double current_kappa = global_trajectory[global_last_closest_index].curvature_radians_per_meter;
            double steer_feedforward = g_feedforward_gain * atan(0.3302 * current_kappa);
            steering_command_radians += steer_feedforward;

            /* ESC removed — Pacejka nonlinear tire model in the
             * linearization naturally reduces B-matrix gain at high
             * slip angles, preventing the yaw-rate cascade without
             * blocking necessary curve-following steering. */

            /* Apply safety saturation */
            saturate_control_commands(&steering_command_radians, &velocity_command_mps);

            /* Store control command:
             * steering_angle_radians: direct MPC output
             * motor_torque_newton_meters: stores velocity command for sim/VESC */
            global_control_command.steering_angle_radians =
                DOUBLE_TO_FP(steering_command_radians);
            global_control_command.motor_torque_newton_meters =
                DOUBLE_TO_FP(velocity_command_mps);

            printf("[MPC] Control: steer=%.4f rad (raw=%.4f, ff=%.3f), torque=%.2f Nm, v_cmd=%.2f m/s (status=%d, iter=%d, dist=%.2f, solve=%.1fus)\n",
                   steering_command_radians, FP_TO_DOUBLE(mpc_result.optimal_control.steering_angle_radians), steer_feedforward, torque_command_nm, velocity_command_mps,
                   mpc_status, mpc_result.iterations_used, distance_from_trajectory, mpc_solve_us);
            fflush(stdout);
        }
        else
        {
            printf("[MPC] WARNING: Solver status=%d, keeping previous command\n", mpc_status);
        }
        } /* end of else (speed >= MIN_SPEED_FOR_MPC) */
    }

    /* Publish drive command every callback (not just when MPC runs) */
    if (global_odometry_received_flag)
    {
        global_drive_message_buffer.drive.steering_angle = FP_TO_FLOAT(
            global_control_command.steering_angle_radians);

        /* The MPC solve block converted torque→velocity and stored
         * the velocity command in motor_torque_newton_meters field. */
        float speed_cmd = FP_TO_FLOAT(
            global_control_command.motor_torque_newton_meters);
        if (speed_cmd < 0.0f) speed_cmd = 0.0f;
        global_drive_message_buffer.drive.speed = speed_cmd;

        rcl_ret_t publish_result = rcl_publish(
            &global_control_publisher,
            &global_drive_message_buffer,
            NULL);
        if (publish_result != RCL_RET_OK)
        {
            fprintf(stderr, "[MPC] ERROR: Failed to publish drive command: %s\n",
                    rcl_get_error_string().str);
        }
    }

    global_odometry_callback_counter++;
}

/*===========================================================================
 * Main Entry Point
 *===========================================================================*/

int main(int argc, char *argv[])
{
    printf("============================================================\n");
    printf("  MPC ROS2 Node for F1/10th Simulator (Jazzy)\n");
    printf("  6-state Frenet model [e_y, e_ψ, vx, vy, ω, ωw]\n");
    printf("  Controls: [δ, T_motor]\n");
    printf("============================================================\n");
    printf("  Prediction horizon: %d steps (%.1f ms each)\n",
           MPC_PREDICTION_HORIZON_STEPS,
           MPC_TIME_STEP_SECONDS * 1000.0f);
    printf("  Max steering: %.2f rad (%.1f°)\n",
           MAXIMUM_STEERING_ANGLE_RADIANS,
           MAXIMUM_STEERING_ANGLE_RADIANS * 180.0f / 3.14159f);
    printf("  Max velocity: %.1f m/s\n",
           MAXIMUM_VELOCITY_METERS_PER_SECOND);
    printf("  Trajectory speed gain: %.2f, max velocity: %.1f m/s\n",
           TRAJECTORY_SPEED_GAIN, TRAJECTORY_MAXIMUM_VELOCITY);
    printf("------------------------------------------------------------\n");
    printf("  Subscribe: /ego_racecar/odom (nav_msgs/Odometry)\n");
    printf("  Publish:   /drive (ackermann_msgs/AckermannDriveStamped)\n");
    printf("============================================================\n\n");

    rcl_ret_t return_code;

    /* Initialize MPC controller (includes vehicle model initialization) */
    mpc_initialize();

    /* Configure MPC for simulation: ODOMETRY_CALLBACK_DIVIDER=1 runs
     * MPC at ~250 Hz (every odom callback). */
    {
        MpcConfiguration_t cfg = mpc_get_configuration();
        cfg.cross_call_rate_scale = FP_CONST(0.3);    /* Allow inter-call steering changes */
        mpc_set_configuration(&cfg);
    }

    /* Load runtime parameters from environment variables (no rebuild needed).
     * Example: MPC_W_LAT_ERROR=2.0 MPC_W_HEADING=5.0 ros2 launch ... */
    {
        const char *env_val;
        if ((env_val = getenv("MPC_MIN_SPEED")) != NULL)
            g_min_speed_for_mpc = atof(env_val);
        if ((env_val = getenv("MPC_MAX_SPEED")) != NULL)
            g_max_speed = atof(env_val);
        if ((env_val = getenv("MPC_SPEED_RAMP")) != NULL)
            g_speed_ramp = atof(env_val);
        if ((env_val = getenv("MPC_STEER_RATE_LIMIT")) != NULL)
            g_steer_rate_limit = atof(env_val);
        if ((env_val = getenv("MPC_FEEDFORWARD")) != NULL)
            g_feedforward_gain = atof(env_val);
        if ((env_val = getenv("MPC_MAX_LAT_ACCEL")) != NULL)
            g_max_lateral_accel = atof(env_val);
        if ((env_val = getenv("MPC_CURV_LOOKAHEAD")) != NULL)
            g_curvature_lookahead = atof(env_val);
        if ((env_val = getenv("MPC_RATE_DIVIDER")) != NULL) {
            int div = atoi(env_val);
            if (div >= 1 && div <= 100) g_odom_divider = div;
        }
    }

    printf("[MPC] MPC controller initialized (horizon=%d steps, dt=%.0f ms)\n",
           MPC_PREDICTION_HORIZON_STEPS, MPC_TIME_STEP_SECONDS * 1000.0f);
    printf("[MPC] Control rate: ~%d Hz (odom_divider=%d)\n",
           200 / g_odom_divider, g_odom_divider);

        MpcConfiguration_t mpc_config = mpc_get_configuration();
        printf("[MPC] Solver max iterations: %u, tolerance (raw): %d\n",
            mpc_config.maximum_solver_iterations,
            (int)mpc_config.solver_convergence_tolerance);
        printf("[MPC] Weights: lat_error=%.2f, heading_error=%.2f, velocity=%.2f, steer_rate=%.2f, steer_effort=%.4f\n",
            FP_TO_DOUBLE(mpc_config.weight_lateral_error),
            FP_TO_DOUBLE(mpc_config.weight_heading_error),
            FP_TO_DOUBLE(mpc_config.weight_velocity),
            FP_TO_DOUBLE(mpc_config.weight_steering_rate),
            FP_TO_DOUBLE(mpc_config.weight_steering_effort));
        printf("[MPC] cross_call_scale=%.2f\n",
            FP_TO_DOUBLE(mpc_config.cross_call_rate_scale));
        printf("[MPC] Frenet MPC: min_speed=%.1f m/s\n",
            g_min_speed_for_mpc);
        printf("[MPC] Speed: max=%.1f m/s, ramp=%.1f m/s per step\n",
            g_max_speed, g_speed_ramp);
        printf("[MPC] Steer rate limit: %.2f rad/step (%.1f rad/s at 20Hz)\n",
            g_steer_rate_limit, g_steer_rate_limit * 20.0);
        printf("[MPC] Feedforward: gain=%.2f, max_lat_accel=%.1f m/s², curv_lookahead=%.0f wp\n",
            g_feedforward_gain, g_max_lateral_accel, g_curvature_lookahead);

    /* Load trajectory from CSV file */
    const char *trajectory_file = NULL;
    if (argc >= 2)
    {
        trajectory_file = argv[1];
    }
    else
    {
        /* Default path - absolute path to trajectory file */
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

    /* Build full trajectory path message for visualization */
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

    /* Initialize ROS2 Context */
    rcl_context_t ros2_context = rcl_get_zero_initialized_context();
    rcl_init_options_t initialization_options = rcl_get_zero_initialized_init_options();

    return_code = rcl_init_options_init(&initialization_options, rcl_get_default_allocator());
    if (return_code != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: Failed to initialize options: %s\n",
                rcl_get_error_string().str);
        return 1;
    }

    return_code = rcl_init(argc, (const char *const *)argv, &initialization_options, &ros2_context);
    if (return_code != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: Failed to initialize ROS2: %s\n",
                rcl_get_error_string().str);
        return 1;
    }
    printf("[ROS2] Context initialized\n");

    /* Create ROS2 Node */
    rcl_node_t ros2_node = rcl_get_zero_initialized_node();
    rcl_node_options_t node_options = rcl_node_get_default_options();

    return_code = rcl_node_init(&ros2_node, "mpc_node", "", &ros2_context, &node_options);
    if (return_code != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: Failed to create node: %s\n",
                rcl_get_error_string().str);
        return 1;
    }
    printf("[ROS2] Node 'mpc_node' created\n");

    /* Create Odometry Subscription */
    rcl_subscription_t odometry_subscription = rcl_get_zero_initialized_subscription();
    const rosidl_message_type_support_t *odometry_type_support =
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry);
    rcl_subscription_options_t subscription_options = rcl_subscription_get_default_options();

    return_code = rcl_subscription_init(
        &odometry_subscription,
        &ros2_node,
        odometry_type_support,
        "/ego_racecar/odom",
        &subscription_options);
    if (return_code != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: Failed to create odometry subscription: %s\n",
                rcl_get_error_string().str);
        return 1;
    }
    printf("[ROS2] Subscribed to /ego_racecar/odom\n");

    /* Create Control Command Publisher */
    global_control_publisher = rcl_get_zero_initialized_publisher();
    const rosidl_message_type_support_t *drive_type_support =
        ROSIDL_GET_MSG_TYPE_SUPPORT(ackermann_msgs, msg, AckermannDriveStamped);
    rcl_publisher_options_t publisher_options = rcl_publisher_get_default_options();

    return_code = rcl_publisher_init(
        &global_control_publisher,
        &ros2_node,
        drive_type_support,
        "/drive",
        &publisher_options);
    if (return_code != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: Failed to create control publisher: %s\n",
                rcl_get_error_string().str);
        return 1;
    }
    printf("[ROS2] Publishing to /drive\n");

    /* Create Reference Path Publisher */
    global_reference_path_publisher = rcl_get_zero_initialized_publisher();
    const rosidl_message_type_support_t *path_type_support =
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Path);

    return_code = rcl_publisher_init(
        &global_reference_path_publisher,
        &ros2_node,
        path_type_support,
        "/mpc/reference_path",
        &publisher_options);
    if (return_code != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: Failed to create reference path publisher: %s\n",
                rcl_get_error_string().str);
        return 1;
    }
    printf("[ROS2] Publishing to /mpc/reference_path\n");

    /* Create Full Trajectory Publisher */
    global_trajectory_path_publisher = rcl_get_zero_initialized_publisher();
    return_code = rcl_publisher_init(
        &global_trajectory_path_publisher,
        &ros2_node,
        path_type_support,
        "/mpc/trajectory_path",
        &publisher_options);
    if (return_code != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: Failed to create trajectory path publisher: %s\n",
                rcl_get_error_string().str);
        return 1;
    }
    printf("[ROS2] Publishing to /mpc/trajectory_path\n");

    /*
     * Initialize odometry message buffer.
     * rclc uses static memory - string fields must be pre-allocated
     * with enough capacity for incoming frame_id strings.
     */
    nav_msgs__msg__Odometry__init(&global_odometry_message_buffer);
    if (!preallocate_rosidl_string(&global_odometry_message_buffer.header.frame_id, 64) ||
        !preallocate_rosidl_string(&global_odometry_message_buffer.child_frame_id, 64))
    {
        fprintf(stderr, "[ROS2] ERROR: Failed to pre-allocate odometry string fields\n");
        return 1;
    }

    /* Initialize drive message buffer (outgoing) */
    ackermann_msgs__msg__AckermannDriveStamped__init(&global_drive_message_buffer);
    if (!preallocate_rosidl_string(&global_drive_message_buffer.header.frame_id, 64))
    {
        fprintf(stderr, "[ROS2] ERROR: Failed to pre-allocate drive header frame_id\n");
        return 1;
    }

    /* Initialize reference path message buffer (outgoing visualization) */
    nav_msgs__msg__Path__init(&global_reference_path_message);
    if (!preallocate_rosidl_string(&global_reference_path_message.header.frame_id, 64))
    {
        fprintf(stderr, "[ROS2] ERROR: Failed to pre-allocate path header frame_id\n");
        return 1;
    }
    set_rosidl_string(&global_reference_path_message.header.frame_id, "map");

    if (!geometry_msgs__msg__PoseStamped__Sequence__init(
            &global_reference_path_message.poses,
            MPC_PREDICTION_HORIZON_STEPS))
    {
        fprintf(stderr, "[ROS2] ERROR: Failed to allocate reference path poses\n");
        return 1;
    }

    /* Create Executor */
    rclc_executor_t ros2_executor = rclc_executor_get_zero_initialized_executor();
    rcl_allocator_t memory_allocator = rcl_get_default_allocator();

    return_code = rclc_executor_init(&ros2_executor, &ros2_context, 1, &memory_allocator);
    if (return_code != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: Failed to initialize executor: %s\n",
                rcl_get_error_string().str);
        return 1;
    }

    /* Add Subscription to Executor */
    return_code = rclc_executor_add_subscription(
        &ros2_executor,
        &odometry_subscription,
        &global_odometry_message_buffer,
        &odometry_subscription_callback,
        ON_NEW_DATA);
    if (return_code != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: Failed to add subscription to executor: %s\n",
                rcl_get_error_string().str);
        return 1;
    }

    printf("[ROS2] Executor ready\n");
    printf("\n[MPC] Spinning... (waiting for odometry messages)\n\n");

    /* Spin: Process callbacks indefinitely */
    rclc_executor_spin(&ros2_executor);

    /* Cleanup (reached on SIGINT) */
    printf("\n[ROS2] Shutting down...\n");
    rclc_executor_fini(&ros2_executor);
    nav_msgs__msg__Odometry__fini(&global_odometry_message_buffer);
    ackermann_msgs__msg__AckermannDriveStamped__fini(&global_drive_message_buffer);
    nav_msgs__msg__Path__fini(&global_reference_path_message);
    nav_msgs__msg__Path__fini(&global_trajectory_path_message);
    rcl_ret_t rc;  /* Suppress unused-result warnings for cleanup */
    rc = rcl_subscription_fini(&odometry_subscription, &ros2_node); (void)rc;
    rc = rcl_publisher_fini(&global_control_publisher, &ros2_node); (void)rc;
    rc = rcl_publisher_fini(&global_reference_path_publisher, &ros2_node); (void)rc;
    rc = rcl_publisher_fini(&global_trajectory_path_publisher, &ros2_node); (void)rc;
    rc = rcl_node_fini(&ros2_node); (void)rc;
    rc = rcl_context_fini(&ros2_context); (void)rc;

    printf("[ROS2] Cleanup complete\n");
    return 0;
}
