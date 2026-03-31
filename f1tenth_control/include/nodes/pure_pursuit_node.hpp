#ifndef F1TENTH_CONTROL_PURE_PURSUIT_NODE_HPP_
#define F1TENTH_CONTROL_PURE_PURSUIT_NODE_HPP_

/**
 * @file pure_pursuit_node.hpp
 * @brief ROS2 node wrapper for the Pure Pursuit path-following controller.
 * @details Fuses odometry (velocity) with an external pose estimate (EKF).
 *          Applies command-side rate limiting on steering and acceleration.
 *          Supports online trajectory updates from a local planner topic.
 *          Soft-start ramp is applied after trajectory load.
 * @dependencies pure_pursuit.hpp, rclcpp, nav_msgs, ackermann_msgs, geometry_msgs, std_msgs
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include "algorithms/pure_pursuit.hpp"
#include <memory>
#include <mutex>
#include <algorithm>
#include <cmath>

namespace f1tenth_control {

/**
 * @brief ROS2 Node for Pure Pursuit path following
 * 
 * This node:
 * - Loads a pre-computed racing line trajectory from CSV
 * - Subscribes to /ekf_pose for vehicle state
 * - Publishes drive commands to /drive
 * - Supports dynamic parameter reconfiguration
 * 
 * Topics:
 *   Subscriptions:
 *     - /ekf_pose (geometry_msgs/PoseWithCovarianceStamped): Vehicle pose
 *     - /pp_enable (std_msgs/Bool): Enable/disable controller
 *   
 *   Publications:
 *     - /drive (ackermann_msgs/AckermannDriveStamped): Control commands
 * 
 * @param trajectory_file Path to CSV trajectory file
 * @param min_lookahead Minimum lookahead distance [m]
 * @param max_lookahead Maximum lookahead distance [m]
 * @param lookahead_gain Velocity-proportional lookahead gain
 * 
 */
class PurePursuitNode : public rclcpp::Node {
public:
    /**
     * @brief Construct a Pure Pursuit node instance with specified options.
     * @param options ROS2 node options controlling parameters and execution behavior.
     * @return None.
     */
    explicit PurePursuitNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // Controller
    std::unique_ptr<PurePursuit> controller_;   // Pure Pursuit controller instance
    PurePursuitConfig config_;                  // Active controller configuration parameters
    std::mutex controller_mutex_;               // Protects access to controller and config for thread safety
    
    // State
    VehicleState current_state_;    // Current vehicle state (pose, velocity, etc.)
    std::mutex state_mutex_;        // Protects access to current_state_ for thread safety
    bool enabled_{true};            // Whether the controller is currently enabled
    bool trajectory_loaded_{false}; // Whether a trajectory has been successfully loaded into the controller
    
    // ROS2 Communication
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;                         // Subscription for odometry messages
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;   // Subscription for pose estimate messages
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;                           // Subscription for enable/disable commands                  
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_raceline_sub_;                   // Subscription for local raceline updates
    
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;    // Publisher for drive commands
    
    // Soft start
    rclcpp::Time soft_start_time_;          // Timestamp when soft start was initiated
    bool soft_start_initialized_{false};    // Whether the soft start timer has been initialized
    double soft_start_distance_traveled_{0.0};  // Distance tracked during soft-start phase [m]
    Point2D last_position_{};               // Last pose sample used for soft-start distance integration
    double last_cmd_steering_{0.0};         // Last commanded steering angle for rate limiting
    double last_cmd_speed_{0.0};            // Last commanded speed for rate limiting
    bool cmd_history_initialized_{false};   // Whether the command history has been initialized for rate limiting
    rclcpp::Time last_cmd_time_;            // Timestamp of the last published command for rate limiting
    
    // Parameters
    std::string trajectory_file_;           // Path to trajectory CSV file
    std::string pose_topic_{"/ekf_pose"};   // Topic for pose estimate messages
    bool pose_received_{false};             // Whether a valid pose estimate has been received
    bool odom_received_{false};             // Whether a valid odometry message has been received
    rclcpp::Time last_pose_time_;           // Timestamp of the last received pose message
    rclcpp::Time last_odom_time_;           // Timestamp of the last received odometry message
    double pose_timeout_s_{0.1};            // Timeout for considering pose data stale [s]
    double odom_timeout_s_{0.2};            // Timeout for considering odometry data stale [s]
    double max_speed_{2.0};                 // [m/s] Maximum commanded speed
    double max_steering_rate_{2.8};         // [rad/s] Maximum rate of change for steering angle
    double max_accel_cmd_{3.0};             // [m/s^2] Maximum acceleration command for speed ramping
    double max_decel_cmd_{5.0};             // [m/s^2] Maximum deceleration command for speed ramping
    
    // Parameter handling
    /**
     * @brief Declare ROS parameters for the Pure Pursuit node.
     * @return None.
     */
    void declareParameters();

    /**
     * @brief Load parameters from the ROS parameter server into internal config.
     * @return None.
     */
    void loadParameters();

    /**
    * @brief Callback for dynamic parameter updates.
    * @param parameters Vector of parameters that were updated.
    * @return Result indicating whether the parameter update was successful.
    */
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter>& parameters
    );
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;  // Handle for parameter callback registration
    
    // Callbacks

    /**
     * @brief Callback for odometry messages.
     * @param msg Shared pointer to the received Odometry message.
     * @return None.
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Callback for pose estimate messages.
     * @param msg Shared pointer to the received PoseWithCovarianceStamped message.
     * @return None.
     */
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    /**
     * @brief Callback for enable/disable commands.
     * @param msg Shared pointer to the received Bool message indicating whether to enable or disable the controller.
     * @return None.
     */
    void enableCallback(const std_msgs::msg::Bool::SharedPtr msg);

    /**
     * @brief Callback for local raceline updates.
     * @param msg Shared pointer to the received Path message representing the updated local raceline trajectory.
     * @return None.
     */
    void localRacelineCallback(const nav_msgs::msg::Path::SharedPtr msg);

    /**
     * @brief Main control loop callback.
     * This function is called periodically 
     * by a timer and executes one cycle of the 
     * Pure Pursuit control logic, including state checks, 
     * controller evaluation, command generation, and publishing.
     * @return None.
     */
    void controlLoop();
    
    // Publishing
    /**
     * @brief Publish drive command based on computed steering and speed.
     * This function constructs an AckermannDriveStamped message from the given steering angle and speed, applies any necessary rate limiting or soft start logic, and publishes it to the /drive topic.
     * @param steering Desired steering angle in radians.
     * @param speed Desired speed in meters per second.
     * @return None.
     */
    void publishDriveCommand(double steering, double speed);

    // Helpers

    /**
     * @brief Load trajectory from CSV file into the Pure Pursuit controller.
     * This function reads a trajectory from the specified CSV file, parses it into the expected format, and loads it into the Pure Pursuit controller instance. It also updates the trajectory_loaded_ flag based on whether the loading was successful.
     * @return True if the trajectory was successfully loaded and parsed, false otherwise.
     */
    bool loadTrajectory();
};

}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_PURE_PURSUIT_NODE_HPP_
