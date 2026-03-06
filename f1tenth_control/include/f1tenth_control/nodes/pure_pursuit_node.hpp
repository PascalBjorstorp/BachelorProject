#ifndef F1TENTH_CONTROL_PURE_PURSUIT_NODE_HPP_
#define F1TENTH_CONTROL_PURE_PURSUIT_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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
 *     - /odom (nav_msgs/Odometry): Vehicle pose and velocity
 *     - /pp_enable (std_msgs/Bool): Enable/disable controller
 *   
 *   Publications:
 *     - /drive (ackermann_msgs/AckermannDriveStamped): Control commands
 *     - /pp_viz (visualization_msgs/MarkerArray): Debug visualization
 *     - /pp_path (nav_msgs/Path): Loaded trajectory for visualization
 * 
 * Parameters:
 *     - trajectory_file: Path to CSV trajectory file
 *     - min_lookahead: Minimum lookahead distance [m]
 *     - max_lookahead: Maximum lookahead distance [m]
 *     - lookahead_gain: Velocity-proportional lookahead gain
 *     - max_speed: Maximum commanded speed [m/s]
 *     - min_speed: Minimum commanded speed [m/s]
 *     - speed_gain: Multiplier for trajectory target speeds
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
    
    // Performance tracking
    struct Metrics {
        double total_distance{0.0};
        double lap_count{0.0};
        rclcpp::Time start_time;
        double last_x{0.0};
        double last_y{0.0};
        double start_x{0.0};
        double start_y{0.0};
        bool initialized{false};
        double max_cross_track_error{0.0};
        double sum_cross_track_error{0.0};
        int update_count{0};
    };
    Metrics metrics_;
    
    // ROS2 Communication
    // TF for map-frame pose (keeps waypoint and vehicle coordinates consistent)
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_raceline_sub_;
    
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // Parameters
    std::string trajectory_file_;
    std::string map_frame_{"map"};
    std::string base_frame_{"ego_racecar/base_link"};
    bool publish_visualization_{true};
    double control_rate_{50.0};  // Hz
    
    void declareParameters();
    void loadParameters();
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter>& parameters
    );
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    // Callbacks
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void enableCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void localRacelineCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void controlLoop();
    bool updatePoseFromTF();  // Updates pose in map frame; falls back to /odom
    
    // Publishing
    void publishDriveCommand(double steering, double speed);
    void publishVisualization(const PurePursuitOutput& output);
    void publishTrajectoryPath();
    
    // Helpers
    bool loadTrajectory();
    void updateMetrics(const PurePursuitOutput& output);
    void printMetricsSummary();
};

}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_PURE_PURSUIT_NODE_HPP_
