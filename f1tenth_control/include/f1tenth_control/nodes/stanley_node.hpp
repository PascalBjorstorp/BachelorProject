#ifndef F1TENTH_CONTROL_STANLEY_NODE_HPP_
#define F1TENTH_CONTROL_STANLEY_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/bool.hpp>

#include "f1tenth_control/algorithms/stanley.hpp"
#include <mutex>

namespace f1tenth_control {

/**
 * @brief ROS2 node for Stanley path-following controller
 */
class StanleyNode : public rclcpp::Node {
public:
    explicit StanleyNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
private:
    // Parameters
    void declareParameters();
    void loadParameters();
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter>& parameters);
    
    // Callbacks
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void enableCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void localRacelineCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void controlLoop();
    
    // Visualization
    void publishVisualization(const StanleyOutput& output);
    void publishTrajectoryPath();
    
    // Metrics
    void updateMetrics(const StanleyOutput& output);
    void checkLapCompletion(size_t current_idx);
    
    // Controller
    std::unique_ptr<Stanley> controller_;
    StanleyConfig config_;
    
    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_raceline_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // Parameter callback handle
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    // State
    VehicleState current_state_;
    std::mutex state_mutex_;
    bool state_received_{false};
    bool enabled_{true};
    
    // Metrics
    double total_cte_{0.0};
    double max_cte_{0.0};
    size_t cte_count_{0};
    size_t last_lap_idx_{0};
    double lap_start_time_{0.0};
    int lap_count_{0};
    bool crossed_start_{false};
    
    // Config
    std::string trajectory_file_;
    bool publish_visualization_{true};
    double control_rate_{200.0};
};

}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_STANLEY_NODE_HPP_
