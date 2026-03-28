#ifndef F1TENTH_CONTROL_PURE_PURSUIT_NODE_HPP_
#define F1TENTH_CONTROL_PURE_PURSUIT_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/bool.hpp>

#include "f1tenth_control/algorithms/pure_pursuit.hpp"
#include <memory>
#include <mutex>

namespace f1tenth_control {

/**
 * @brief ROS2 Node for Pure Pursuit path following
 * 
 * This node:
 * - Loads a pre-computed racing line trajectory from CSV
 * - Subscribes to /odom for vehicle state
 * - Publishes drive commands to /drive
 * - Optionally publishes visualization markers
 * - Supports dynamic parameter reconfiguration
 * 
 * Topics:
 *   Subscriptions:
 *     - /odom (nav_msgs/Odometry): Vehicle velocity
 *     - /pp_enable (std_msgs/Bool): Enable/disable controller
 *   
 *   Publications:
 *     - /drive (ackermann_msgs/AckermannDriveStamped): Control commands
 *     - /pp_viz (visualization_msgs/MarkerArray): Lookahead point visualization
 * 
 * Parameters:
 *     - trajectory_file: Path to CSV trajectory file
 *     - min_lookahead: Minimum lookahead distance [m]
 *     - max_lookahead: Maximum lookahead distance [m]
 *     - lookahead_gain: Velocity-proportional lookahead gain
 *     - publish_visualization: Whether to publish debug markers
 */
class PurePursuitNode : public rclcpp::Node {
public:
    /**
     * Inputs:
     * - options: ROS2 node options controlling parameters and execution behavior.
     *
     * Purpose:
     * - Construct ROS2 wrapper around Pure Pursuit controller and initialize runtime
     *   subscriptions, publishers, timers, and parameter interfaces.
     *
     * Outputs:
     * - Creates an operational node ready to receive state and publish drive commands.
     */
    explicit PurePursuitNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // Controller
    std::unique_ptr<PurePursuit> controller_;
    PurePursuitConfig config_;
    std::mutex controller_mutex_;
    
    // State
    VehicleState current_state_;
    std::mutex state_mutex_;
    bool enabled_{true};
    bool trajectory_loaded_{false};
    
    // ROS2 Communication
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_raceline_sub_;
    
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
    
    // Soft start
    rclcpp::Time soft_start_time_;
    bool soft_start_initialized_{false};
    double last_cmd_steering_{0.0};
    double last_cmd_speed_{0.0};
    bool cmd_history_initialized_{false};
    rclcpp::Time last_cmd_time_;
    
    // Parameters
    std::string trajectory_file_;
    std::string pose_topic_{"/ekf_pose"};
    bool publish_visualization_{true};
    bool pose_received_{false};
    bool odom_received_{false};
    rclcpp::Time last_pose_time_;
    rclcpp::Time last_odom_time_;
    double pose_timeout_s_{0.1};
    double odom_timeout_s_{0.2};
    double max_speed_{2.0};
    double max_steering_rate_{2.8};
    double max_accel_cmd_{3.0};
    double max_decel_cmd_{5.0};
    
    /**
     * Inputs:
     * - None.
     *
     * Purpose:
     * - Declare ROS parameters and defaults used by Pure Pursuit node.
     *
     * Outputs:
     * - Registers parameter keys for startup and runtime overrides.
     */
    void declareParameters();

    /**
     * Inputs:
     * - None.
     *
     * Purpose:
     * - Load active ROS parameter values into node/controller configuration.
     *
     * Outputs:
     * - Updates config_ and node-local runtime parameter fields.
     */
    void loadParameters();

    /**
     * Inputs:
     * - parameters: Proposed parameter updates.
     *
     * Purpose:
     * - Validate and apply runtime parameter changes.
     *
     * Outputs:
     * - Returns parameter-set result indicating acceptance/rejection.
     */
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter>& parameters
    );
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    // Callbacks

    /**
     * Inputs:
     * - msg: Odometry message.
     *
     * Purpose:
     * - Update velocity and fallback pose state from odometry stream.
     *
     * Outputs:
     * - Mutates current_state_ and odometry freshness timestamps.
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * Inputs:
     * - msg: Pose estimate message.
     *
     * Purpose:
     * - Update high-confidence pose used by trajectory tracking controller.
     *
     * Outputs:
     * - Mutates current_state_.pose and pose freshness timestamps.
     */
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    /**
     * Inputs:
     * - msg: Enable/disable command.
     *
     * Purpose:
     * - Gate controller output based on external supervisory signal.
     *
     * Outputs:
     * - Updates enabled_ state for control loop behavior.
     */
    void enableCallback(const std_msgs::msg::Bool::SharedPtr msg);

    /**
     * Inputs:
     * - msg: Path message representing local raceline.
     *
     * Purpose:
     * - Refresh controller trajectory from online local planner updates.
     *
     * Outputs:
     * - Updates controller trajectory and trajectory_loaded_ state.
     */
    void localRacelineCallback(const nav_msgs::msg::Path::SharedPtr msg);

    /**
     * Inputs:
     * - None (timer/event driven).
     *
     * Purpose:
     * - Execute one control-cycle evaluation and command publication.
     *
     * Outputs:
     * - Publishes drive commands and optional visualization side effects.
     */
    void controlLoop();
    
    // Publishing

    /**
     * Inputs:
     * - steering: Commanded steering angle.
     * - speed: Commanded longitudinal speed.
     *
     * Purpose:
     * - Convert controller outputs into Ackermann ROS message.
     *
     * Outputs:
     * - Publishes AckermannDriveStamped drive command.
     */
    void publishDriveCommand(double steering, double speed);

    /**
     * Inputs:
     * - output: Pure Pursuit diagnostic output containing target/lookahead info.
     *
     * Purpose:
     * - Publish marker visualization for controller target/debug interpretation.
     *
     * Outputs:
     * - Publishes MarkerArray on visualization topic.
     */
    void publishLookaheadMarker(const PurePursuitOutput& output);
    
    // Helpers

    /**
     * Inputs:
     * - None.
     *
     * Purpose:
     * - Load trajectory from configured trajectory file path.
     *
     * Outputs:
     * - Returns true when controller trajectory is successfully loaded.
     */
    bool loadTrajectory();
};

}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_PURE_PURSUIT_NODE_HPP_
