#ifndef F1TENTH_CONTROL_PURE_PURSUIT_NODE_HPP_
#define F1TENTH_CONTROL_PURE_PURSUIT_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
    explicit PurePursuitNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // Controller
    std::unique_ptr<PurePursuit> controller_;
    PurePursuitConfig config_;
    
    // State
    VehicleState current_state_;
    std::mutex state_mutex_;
    bool enabled_{true};
    bool trajectory_loaded_{false};
    
    // ROS2 Communication
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_raceline_sub_;
    
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
    
    // Soft start
    rclcpp::Time soft_start_time_;
    bool soft_start_initialized_{false};
    
    // Parameters
    std::string trajectory_file_;
    std::string pose_topic_{"/ekf_pose"};
    bool publish_visualization_{true};
    bool pose_received_{false};
    rclcpp::Time last_pose_time_;
    double pose_timeout_s_{0.1};
    
    void declareParameters();
    void loadParameters();
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter>& parameters
    );
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    // Callbacks
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void enableCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void localRacelineCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void controlLoop();
    
    // Publishing
    void publishDriveCommand(double steering, double speed);
    void publishLookaheadMarker(const PurePursuitOutput& output);
    
    // Helpers
    bool loadTrajectory();
};

}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_PURE_PURSUIT_NODE_HPP_
