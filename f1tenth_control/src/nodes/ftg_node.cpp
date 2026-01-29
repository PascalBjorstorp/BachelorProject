#include "f1tenth_control/nodes/ftg_node.hpp"
#include <tf2/utils.h>
#include <chrono>

namespace f1tenth_control {

FTGNode::FTGNode(const rclcpp::NodeOptions& options)
    : Node("ftg_node", options)
{
    // Declare and load parameters
    declareParameters();
    loadParameters();
    
    // Create algorithm instance
    ftg_ = std::make_unique<FollowTheGap>(config_);
    
    // Setup parameter callback for dynamic reconfiguration
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&FTGNode::parametersCallback, this, std::placeholders::_1)
    );
    
    // QoS profiles
    auto sensor_qos = rclcpp::SensorDataQoS();
    auto reliable_qos = rclcpp::QoS(10);
    
    // Subscribers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", sensor_qos,
        std::bind(&FTGNode::scanCallback, this, std::placeholders::_1)
    );
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", sensor_qos,
        std::bind(&FTGNode::odomCallback, this, std::placeholders::_1)
    );
    
    enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "ftg/enable", reliable_qos,
        std::bind(&FTGNode::enableCallback, this, std::placeholders::_1)
    );
    
    // Publishers
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        "drive", reliable_qos
    );
    
    viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "ftg/visualization", reliable_qos
    );
    
    RCLCPP_INFO(this->get_logger(), "FTG Node initialized");
    RCLCPP_INFO(this->get_logger(), "  Subscribing to: scan, odom");
    RCLCPP_INFO(this->get_logger(), "  Publishing to: drive, ftg/visualization");
}

void FTGNode::declareParameters() {
    // Vehicle parameters
    this->declare_parameter("wheelbase", 0.324);
    this->declare_parameter("car_width", 0.30);
    
    // Speed control
    this->declare_parameter("max_speed", 4.0);
    this->declare_parameter("min_speed", 1.0);
    this->declare_parameter("speed_range_factor", 0.5);
    this->declare_parameter("nominal_gap_width", 1.0);
    
    // Steering control
    this->declare_parameter("max_steering_angle", 0.4);
    this->declare_parameter("steering_gain", 1.0);
    
    // Gap selection
    this->declare_parameter("prefer_straight", true);
    this->declare_parameter("straight_weight", 0.3);
    
    // Safety
    this->declare_parameter("emergency_brake_distance", 0.3);
    this->declare_parameter("slowdown_distance", 1.5);
    
    // LiDAR processing
    this->declare_parameter("lidar.range_min", 0.1);
    this->declare_parameter("lidar.range_max", 10.0);
    this->declare_parameter("lidar.angle_min", -1.57);  // -90 degrees
    this->declare_parameter("lidar.angle_max", 1.57);   // +90 degrees
    this->declare_parameter("lidar.apply_median_filter", true);
    this->declare_parameter("lidar.median_window_size", 3);
    this->declare_parameter("lidar.disparity_threshold", 0.3);
    this->declare_parameter("lidar.gap_threshold", 3.0);
    this->declare_parameter("lidar.min_gap_width", 0.3);
    this->declare_parameter("lidar.bubble_radius", 0.2);
    this->declare_parameter("lidar.apply_bubble", true);
    
    // Mapping mode
    this->declare_parameter("mapping_mode", false);
    this->declare_parameter("mapping_sample_rate", 10.0);
}

void FTGNode::loadParameters() {
    // Vehicle parameters
    config_.wheelbase = this->get_parameter("wheelbase").as_double();
    config_.car_width = this->get_parameter("car_width").as_double();
    
    // Speed control
    config_.max_speed = this->get_parameter("max_speed").as_double();
    config_.min_speed = this->get_parameter("min_speed").as_double();
    config_.speed_range_factor = this->get_parameter("speed_range_factor").as_double();
    config_.nominal_gap_width = this->get_parameter("nominal_gap_width").as_double();
    
    // Steering control
    config_.max_steering_angle = this->get_parameter("max_steering_angle").as_double();
    config_.steering_gain = this->get_parameter("steering_gain").as_double();
    
    // Gap selection
    config_.prefer_straight = this->get_parameter("prefer_straight").as_bool();
    config_.straight_weight = this->get_parameter("straight_weight").as_double();
    
    // Safety
    config_.emergency_brake_distance = this->get_parameter("emergency_brake_distance").as_double();
    config_.slowdown_distance = this->get_parameter("slowdown_distance").as_double();
    
    // LiDAR processing
    config_.lidar_config.range_min = this->get_parameter("lidar.range_min").as_double();
    config_.lidar_config.range_max = this->get_parameter("lidar.range_max").as_double();
    config_.lidar_config.angle_min = this->get_parameter("lidar.angle_min").as_double();
    config_.lidar_config.angle_max = this->get_parameter("lidar.angle_max").as_double();
    config_.lidar_config.apply_median_filter = this->get_parameter("lidar.apply_median_filter").as_bool();
    config_.lidar_config.median_window_size = this->get_parameter("lidar.median_window_size").as_int();
    config_.lidar_config.disparity_threshold = this->get_parameter("lidar.disparity_threshold").as_double();
    config_.lidar_config.gap_threshold = this->get_parameter("lidar.gap_threshold").as_double();
    config_.lidar_config.min_gap_width = this->get_parameter("lidar.min_gap_width").as_double();
    config_.lidar_config.bubble_radius = this->get_parameter("lidar.bubble_radius").as_double();
    config_.lidar_config.apply_bubble = this->get_parameter("lidar.apply_bubble").as_bool();
    
    // Mapping mode
    config_.mapping_mode = this->get_parameter("mapping_mode").as_bool();
    config_.mapping_sample_rate = this->get_parameter("mapping_sample_rate").as_double();
}

rcl_interfaces::msg::SetParametersResult FTGNode::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters
) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    for (const auto& param : parameters) {
        RCLCPP_INFO(this->get_logger(), "Parameter '%s' changed", param.get_name().c_str());
    }
    
    // Reload all parameters and update algorithm
    loadParameters();
    if (ftg_) {
        ftg_->setConfig(config_);
    }
    
    return result;
}

void FTGNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!enabled_) {
        return;
    }
    
    // Get current pose from odometry
    Pose2D current_pose;
    double timestamp;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_pose = current_state_.pose;
        timestamp = this->now().seconds();
    }
    
    // Run FTG algorithm
    FTGOutput output = ftg_->compute(
        msg->ranges,
        msg->angle_min,
        msg->angle_max,
        msg->angle_increment,
        current_pose,
        timestamp
    );
    
    // Publish drive command
    publishDriveCommand(output.command);
    
    // Publish visualization if anyone is listening
    if (viz_pub_->get_subscription_count() > 0) {
        ProcessedScan scan = ftg_->getLidarProcessor().processScan(
            msg->ranges, msg->angle_min, msg->angle_max, msg->angle_increment
        );
        publishVisualization(output, scan);
    }
    
    // Log if emergency stop
    if (output.emergency_stop) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Emergency stop! Closest obstacle at %.2f m", output.closest_point_dist);
    }
}

void FTGNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // Extract pose
    current_state_.pose.x = msg->pose.pose.position.x;
    current_state_.pose.y = msg->pose.pose.position.y;
    
    // Extract yaw from quaternion
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    current_state_.pose.theta = tf2::getYaw(q);
    
    // Extract velocities
    current_state_.velocity = msg->twist.twist.linear.x;
    current_state_.angular_velocity = msg->twist.twist.angular.z;
}

void FTGNode::enableCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    enabled_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "FTG %s", enabled_ ? "enabled" : "disabled");
    
    if (!enabled_) {
        // Publish zero command when disabled
        publishDriveCommand(DriveCommand(0.0, 0.0));
    }
}

void FTGNode::publishDriveCommand(const DriveCommand& cmd) {
    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.header.stamp = this->now();
    drive_msg.header.frame_id = "base_link";
    drive_msg.drive.speed = cmd.speed;
    drive_msg.drive.steering_angle = cmd.steering_angle;
    
    drive_pub_->publish(drive_msg);
}

void FTGNode::publishVisualization(const FTGOutput& output, const ProcessedScan& scan) {
    visualization_msgs::msg::MarkerArray marker_array;
    
    int id = 0;
    
    // Visualize all gaps
    for (size_t i = 0; i < output.all_gaps.size(); ++i) {
        // A gap is selected if its indices match the selected gap
        bool is_selected =
            (output.all_gaps[i].start_idx == output.selected_gap.start_idx &&
             output.all_gaps[i].end_idx == output.selected_gap.end_idx);
        marker_array.markers.push_back(
            createGapMarker(output.all_gaps[i], scan, id++, is_selected)
        );
    }
    
    // Visualize closest point (use next available ID to avoid conflict)
    marker_array.markers.push_back(
        createClosestPointMarker(scan, output.closest_point_idx, id++)
    );
    
    viz_pub_->publish(marker_array);
}

visualization_msgs::msg::Marker FTGNode::createGapMarker(
    const Gap& gap,
    const ProcessedScan& scan,
    int id,
    bool selected
) {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = this->now();
    marker.header.frame_id = "laser";
    marker.ns = "gaps";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Color: green for selected, blue for others
    if (selected) {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.scale.x = 0.05;  // Thicker line for selected
    } else {
        marker.color.r = 0.0;
        marker.color.g = 0.5;
        marker.color.b = 1.0;
        marker.color.a = 0.5;
        marker.scale.x = 0.02;
    }
    
    // Create arc showing the gap
    for (size_t i = gap.start_idx; i <= gap.end_idx; ++i) {
        geometry_msgs::msg::Point p;
        double range = scan.filtered_ranges[i];
        double angle = scan.angles[i];
        p.x = range * std::cos(angle);
        p.y = range * std::sin(angle);
        p.z = 0.0;
        marker.points.push_back(p);
    }
    
    marker.lifetime = rclcpp::Duration::from_seconds(0.1);
    
    return marker;
}

visualization_msgs::msg::Marker FTGNode::createClosestPointMarker(
    const ProcessedScan& scan,
    size_t idx,
    int marker_id
) {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = this->now();
    marker.header.frame_id = "laser";
    marker.ns = "closest_point";
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    double range = scan.filtered_ranges[idx];
    double angle = scan.angles[idx];
    
    marker.pose.position.x = range * std::cos(angle);
    marker.pose.position.y = range * std::sin(angle);
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;
    
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    
    marker.lifetime = rclcpp::Duration::from_seconds(0.1);
    
    return marker;
}

}  // namespace f1tenth_control

// Main function
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<f1tenth_control::FTGNode>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
