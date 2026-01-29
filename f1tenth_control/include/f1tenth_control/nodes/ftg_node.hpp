#ifndef F1TENTH_CONTROL_FTG_NODE_HPP_
#define F1TENTH_CONTROL_FTG_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/bool.hpp>

#include "f1tenth_control/algorithms/follow_the_gap.hpp"
#include <memory>
#include <mutex>

namespace f1tenth_control {

/**
 * @brief ROS2 Node wrapper for Follow The Gap algorithm
 * 
 * This node:
 * - Subscribes to /scan (LiDAR) and /odom (odometry)
 * - Publishes drive commands to /drive
 * - Optionally publishes visualization markers
 * - Supports dynamic parameter reconfiguration
 */
class FTGNode : public rclcpp::Node {
public:
    explicit FTGNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // Algorithm
    std::unique_ptr<FollowTheGap> ftg_;
    FTGConfig config_;
    
    // State
    VehicleState current_state_;
    std::mutex state_mutex_;
    bool enabled_{true};
    
    // ROS2 Communication
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
    
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
    
    // Parameters
    void declareParameters();
    void loadParameters();
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter>& parameters
    );
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    // Callbacks
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void enableCallback(const std_msgs::msg::Bool::SharedPtr msg);
    
    // Publishing
    void publishDriveCommand(const DriveCommand& cmd);
    void publishVisualization(const FTGOutput& output, const ProcessedScan& scan);
    
    // Helpers
    visualization_msgs::msg::Marker createGapMarker(
        const Gap& gap, 
        const ProcessedScan& scan,
        int id, 
        bool selected
    );
    visualization_msgs::msg::Marker createClosestPointMarker(
        const ProcessedScan& scan,
        size_t idx,
        int marker_id
    );
};

}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_FTG_NODE_HPP_
