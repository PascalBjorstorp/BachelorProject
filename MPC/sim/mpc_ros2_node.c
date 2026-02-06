/**
 * @file mpc_ros2_node.c
 * @brief MPC ROS2 Node for F1/10th Simulator Integration
 *
 * Implements ROS2 node using rclc (C client library) for Jazzy.
 * Subscribes to odometry, runs MPC solver, publishes control commands.
 *
 * Topics:
 *   Subscribe: /ego_racecar/odom (nav_msgs/Odometry)
 *   Publish:   /cmd_vel (geometry_msgs/Twist)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ROS2 C Client Library Headers */
#include "rcl/rcl.h"
#include "rcl/error_handling.h"
#include "rclc/rclc.h"
#include "rclc/executor.h"

/* ROS2 Message Types */
#include "nav_msgs/msg/odometry.h"
#include "geometry_msgs/msg/twist.h"

/* MPC Core Library Headers (Platform-Independent) */
#include "mpc.h"
#include "mpc_types.h"
#include "fixed_point.h"
#include "vehicle_model.h"

/*===========================================================================
 * Configuration Constants
 *===========================================================================*/

/** Number of MPC prediction steps */
#define MPC_PREDICTION_HORIZON_STEPS 10

/** Time step between predictions (seconds) */
#define MPC_TIME_STEP_SECONDS 0.05f

/** Maximum allowed steering angle (radians, ~23 degrees) */
#define MAXIMUM_STEERING_ANGLE_RADIANS 0.4f

/** Maximum allowed acceleration (m/s²) */
#define MAXIMUM_ACCELERATION_METERS_PER_SECOND_SQUARED 5.0f

/** Odometry callback divider (run MPC every N callbacks) */
#define ODOMETRY_CALLBACK_DIVIDER 10

/*===========================================================================
 * Global State Variables
 *===========================================================================*/

/** Current vehicle state from odometry */
static VehicleState_t global_vehicle_state = {0};

/** Current control command to publish */
static ControlInput_t global_control_command = {0};

/** Flag: have we received at least one odometry message? */
static int global_odometry_received_flag = 0;

/** Counter for odometry callbacks (used for rate limiting MPC) */
static int global_odometry_callback_counter = 0;

/** ROS2 publisher handle for control commands */
static rcl_publisher_t global_control_publisher;

/** Buffer for incoming odometry message */
static nav_msgs__msg__Odometry global_odometry_message_buffer;

/** Reference trajectory for MPC (simple straight-line reference for now) */
static TrajectoryReferencePoint_t global_reference_trajectory[MPC_PREDICTION_HORIZON_STEPS];

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
 * @brief Saturate control commands to safe limits
 *
 * @param steering_angle_radians Steering angle to saturate (modified in-place)
 * @param acceleration Acceleration to saturate (modified in-place)
 */
static void saturate_control_commands(double *steering_angle_radians,
                                      double *acceleration_meters_per_second_squared)
{
    if (*steering_angle_radians > MAXIMUM_STEERING_ANGLE_RADIANS)
    {
        *steering_angle_radians = MAXIMUM_STEERING_ANGLE_RADIANS;
    }
    if (*steering_angle_radians < -MAXIMUM_STEERING_ANGLE_RADIANS)
    {
        *steering_angle_radians = -MAXIMUM_STEERING_ANGLE_RADIANS;
    }
    if (*acceleration_meters_per_second_squared > MAXIMUM_ACCELERATION_METERS_PER_SECOND_SQUARED)
    {
        *acceleration_meters_per_second_squared = MAXIMUM_ACCELERATION_METERS_PER_SECOND_SQUARED;
    }
    if (*acceleration_meters_per_second_squared < -MAXIMUM_ACCELERATION_METERS_PER_SECOND_SQUARED)
    {
        *acceleration_meters_per_second_squared = -MAXIMUM_ACCELERATION_METERS_PER_SECOND_SQUARED;
    }
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
    (void)yaw_rate_radians_per_second; /* Unused for now */

    /* Compute total velocity magnitude */
    double velocity_magnitude = sqrt(velocity_x_meters_per_second * velocity_x_meters_per_second +
                                     velocity_y_meters_per_second * velocity_y_meters_per_second);

    /* Update global vehicle state (convert to fixed-point) */
    global_vehicle_state.position_x_meters = fixed_point_from_float((float)position_x_meters);
    global_vehicle_state.position_y_meters = fixed_point_from_float((float)position_y_meters);
    global_vehicle_state.heading_angle_radians = fixed_point_from_float((float)heading_angle_radians);
    global_vehicle_state.velocity_meters_per_second = fixed_point_from_float((float)velocity_magnitude);

    global_odometry_received_flag = 1;

    /* Run MPC solver at reduced rate */
    if ((global_odometry_callback_counter % ODOMETRY_CALLBACK_DIVIDER) == 0)
    {
        printf("[MPC] State: x=%.2f m, y=%.2f m, θ=%.2f rad, v=%.2f m/s\n",
               position_x_meters, position_y_meters, heading_angle_radians, velocity_magnitude);

        /*
         * Build reference trajectory (simple: maintain current heading, target velocity)
         * In a full implementation, this would come from a path planner.
         */
        fixed_point_t target_velocity = fixed_point_from_float(2.0f); /* 2 m/s target */
        fixed_point_t time_step = fixed_point_from_float(MPC_TIME_STEP_SECONDS);

        for (int step = 0; step < MPC_PREDICTION_HORIZON_STEPS; step++)
        {
            /* Project position forward along current heading */
            fixed_point_t time_ahead = fixed_point_multiply(
                time_step,
                fixed_point_from_float((float)(step + 1)));

            fixed_point_t distance_ahead = fixed_point_multiply(target_velocity, time_ahead);

            global_reference_trajectory[step].reference_position_x_meters = fixed_point_add(
                global_vehicle_state.position_x_meters,
                fixed_point_multiply(distance_ahead,
                                     fixed_point_cosine(global_vehicle_state.heading_angle_radians)));

            global_reference_trajectory[step].reference_position_y_meters = fixed_point_add(
                global_vehicle_state.position_y_meters,
                fixed_point_multiply(distance_ahead,
                                     fixed_point_sine(global_vehicle_state.heading_angle_radians)));

            global_reference_trajectory[step].reference_heading_radians =
                global_vehicle_state.heading_angle_radians;

            global_reference_trajectory[step].reference_velocity_meters_per_second = target_velocity;
        }

        /* Run MPC to compute optimal control */
        MpcSolverResult_t mpc_result;
        MpcSolverStatus_t mpc_status = mpc_compute_optimal_control(
            &global_vehicle_state,
            global_reference_trajectory,
            &mpc_result);

        if (mpc_status == MPC_STATUS_SUCCESS ||
            mpc_status == MPC_STATUS_MAXIMUM_ITERATIONS_REACHED)
        {
            /* Extract control from MPC result */
            double steering_command_radians = fixed_point_to_float(
                mpc_result.optimal_control.steering_angle_radians);
            double acceleration_command = fixed_point_to_float(
                mpc_result.optimal_control.acceleration_meters_per_second_squared);

            /* Apply safety saturation */
            saturate_control_commands(&steering_command_radians, &acceleration_command);

            /* Store in global control command (convert to fixed-point) */
            global_control_command.steering_angle_radians =
                fixed_point_from_float((float)steering_command_radians);
            global_control_command.acceleration_meters_per_second_squared =
                fixed_point_from_float((float)acceleration_command);

            printf("[MPC] Control: steering=%.4f rad, accel=%.4f m/s² (status=%d, iter=%d)\n",
                   steering_command_radians, acceleration_command,
                   mpc_status, mpc_result.iterations_used);
            fflush(stdout);
        }
        else
        {
            printf("[MPC] WARNING: Solver status=%d, keeping previous command\n", mpc_status);
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
    printf("============================================================\n");
    printf("  Prediction horizon: %d steps (%.1f ms each)\n",
           MPC_PREDICTION_HORIZON_STEPS,
           MPC_TIME_STEP_SECONDS * 1000.0f);
    printf("  Max steering: %.2f rad (%.1f°)\n",
           MAXIMUM_STEERING_ANGLE_RADIANS,
           MAXIMUM_STEERING_ANGLE_RADIANS * 180.0f / 3.14159f);
    printf("  Max acceleration: %.1f m/s²\n",
           MAXIMUM_ACCELERATION_METERS_PER_SECOND_SQUARED);
    printf("------------------------------------------------------------\n");
    printf("  Subscribe: /ego_racecar/odom (nav_msgs/Odometry)\n");
    printf("  Publish:   /cmd_vel (geometry_msgs/Twist)\n");
    printf("============================================================\n\n");

    rcl_ret_t return_code;

    /* Initialize MPC controller (includes vehicle model initialization) */
    mpc_initialize();
    printf("[MPC] MPC controller initialized (horizon=%d steps, dt=%.0f ms)\n",
           MPC_PREDICTION_HORIZON_STEPS, MPC_TIME_STEP_SECONDS * 1000.0f);

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
    const rosidl_message_type_support_t *twist_type_support =
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist);
    rcl_publisher_options_t publisher_options = rcl_publisher_get_default_options();

    return_code = rcl_publisher_init(
        &global_control_publisher,
        &ros2_node,
        twist_type_support,
        "/cmd_vel",
        &publisher_options);
    if (return_code != RCL_RET_OK)
    {
        fprintf(stderr, "[ROS2] ERROR: Failed to create control publisher: %s\n",
                rcl_get_error_string().str);
        return 1;
    }
    printf("[ROS2] Publishing to /cmd_vel\n");

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
    rcl_subscription_fini(&odometry_subscription, &ros2_node);
    rcl_publisher_fini(&global_control_publisher, &ros2_node);
    rcl_node_fini(&ros2_node);
    rcl_context_fini(&ros2_context);

    printf("[ROS2] Cleanup complete\n");
    return 0;
}
