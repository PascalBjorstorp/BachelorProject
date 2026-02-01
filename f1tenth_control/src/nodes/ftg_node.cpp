#include "f1tenth_control/nodes/ftg_node.hpp"
#include <tf2/utils.h>

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
    this->declare_parameter("car_width", 0.50);
    
    // Speed control (reference FTG formula)
    this->declare_parameter("max_speed", 6.0);
    this->declare_parameter("min_speed", 2.0);
    this->declare_parameter("speed_full_range", 9.0);
    this->declare_parameter("steer_slowdown_gain", 0.7);
    
    // Steering control
    this->declare_parameter("max_steering_angle", 0.4);
    this->declare_parameter("steering_gain", 0.8);  // Reduced for stability
    this->declare_parameter("max_steering_delta", 0.05);  // Reduced for smoother steering
    this->declare_parameter("target_angle_smoothing", 0.3);  // EMA smoothing factor
    this->declare_parameter("gap_center_weight", 0.3);  // Blend between deepest (0) and center (1)
    
    // Safety
    this->declare_parameter("emergency_brake_distance", 0.3);
    
    // FTG-specific LiDAR processing parameters
    this->declare_parameter("disparity_threshold", 0.5);
    this->declare_parameter("gap_threshold", 0.8);
    this->declare_parameter("min_gap_width", 0.15);
    this->declare_parameter("bubble_radius", 0.25);
    this->declare_parameter("apply_bubble", true);
    
    // Generic LiDAR preprocessing parameters
    this->declare_parameter("lidar.range_min", 0.1);
    this->declare_parameter("lidar.range_max", 12.0);
    this->declare_parameter("lidar.angle_min", -1.57);  // -90 degrees
    this->declare_parameter("lidar.angle_max", 1.57);   // +90 degrees
    this->declare_parameter("lidar.apply_median_filter", true);
    this->declare_parameter("lidar.median_window_size", 3);
    
    // Mapping mode
    this->declare_parameter("mapping_mode", false);
    this->declare_parameter("mapping_sample_rate", 10.0);
}

void FTGNode::loadParameters() {
    // Vehicle parameters
    config_.wheelbase = this->get_parameter("wheelbase").as_double();
    config_.car_width = this->get_parameter("car_width").as_double();
    
    // Speed control (reference FTG formula)
    config_.max_speed = this->get_parameter("max_speed").as_double();
    config_.min_speed = this->get_parameter("min_speed").as_double();
    config_.speed_full_range = this->get_parameter("speed_full_range").as_double();
    config_.steer_slowdown_gain = this->get_parameter("steer_slowdown_gain").as_double();
    
    // Steering control
    config_.max_steering_angle = this->get_parameter("max_steering_angle").as_double();
    config_.steering_gain = this->get_parameter("steering_gain").as_double();
    config_.max_steering_delta = this->get_parameter("max_steering_delta").as_double();
    config_.target_angle_smoothing = this->get_parameter("target_angle_smoothing").as_double();
    config_.gap_center_weight = this->get_parameter("gap_center_weight").as_double();
    
    // Safety
    config_.emergency_brake_distance = this->get_parameter("emergency_brake_distance").as_double();
    
    // FTG-specific LiDAR processing parameters (now in FTGConfig)
    config_.disparity_threshold = this->get_parameter("disparity_threshold").as_double();
    config_.gap_threshold = this->get_parameter("gap_threshold").as_double();
    config_.min_gap_width = this->get_parameter("min_gap_width").as_double();
    config_.bubble_radius = this->get_parameter("bubble_radius").as_double();
    config_.apply_bubble = this->get_parameter("apply_bubble").as_bool();
    
    // Generic LiDAR preprocessing config
    config_.lidar_config.range_min = this->get_parameter("lidar.range_min").as_double();
    config_.lidar_config.range_max = this->get_parameter("lidar.range_max").as_double();
    config_.lidar_config.angle_min = this->get_parameter("lidar.angle_min").as_double();
    config_.lidar_config.angle_max = this->get_parameter("lidar.angle_max").as_double();
    config_.lidar_config.apply_median_filter = this->get_parameter("lidar.apply_median_filter").as_bool();
    config_.lidar_config.median_window_size = this->get_parameter("lidar.median_window_size").as_int();
    
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
    
    // Steering smoothing is now handled internally by the FTG algorithm
    // via rate limiting (max_steering_delta)
    DriveCommand smoothed_cmd = output.command;
    prev_steering_ = output.command.steering_angle;
    
    // Recovery mode logic: detect stuck state and try to escape
    DriveCommand final_cmd = smoothed_cmd;
    
    if (output.emergency_stop && output.all_gaps.empty()) {
        // We're stuck with no gaps - increment stuck counter
        stuck_counter_++;
        
        if (stuck_counter_ >= STUCK_THRESHOLD && !in_recovery_mode_) {
            // Enter recovery mode
            in_recovery_mode_ = true;
            recovery_counter_ = 0;
            metrics_.recovery_events++;  // Track recovery event
            // Choose turn direction based on which side has more space
            // Use the deepest_idx angle to determine direction
            ProcessedScan scan = ftg_->getLidarProcessor().processScan(
                msg->ranges, msg->angle_min, msg->angle_max, msg->angle_increment
            );
            if (!scan.angles.empty() && output.closest_point_idx < scan.angles.size()) {
                // Turn away from the closest obstacle
                double closest_angle = scan.angles[output.closest_point_idx];
                recovery_steer_direction_ = (closest_angle > 0) ? -1.0 : 1.0;
            }
            RCLCPP_WARN(this->get_logger(), 
                "RECOVERY MODE: Stuck for %d cycles, backing up and turning %s",
                stuck_counter_, recovery_steer_direction_ > 0 ? "RIGHT" : "LEFT");
        }
    } else if (!output.emergency_stop && !output.all_gaps.empty()) {
        // Normal driving - reset stuck counter
        stuck_counter_ = 0;
        // Only exit recovery after minimum backup duration (40 cycles ~= 0.8s of backing)
        constexpr int MIN_RECOVERY_BACKUP = 40;
        if (in_recovery_mode_ && recovery_counter_ >= MIN_RECOVERY_BACKUP) {
            RCLCPP_INFO(this->get_logger(), "RECOVERY MODE: Exited after %d cycles - found gaps!", recovery_counter_);
            in_recovery_mode_ = false;
            recovery_counter_ = 0;
        }
    }
    
    // Apply recovery behavior
    if (in_recovery_mode_) {
        recovery_counter_++;
        
        // Recovery: reverse slowly with steering to turn away from obstacle
        double reverse_speed = -0.5;  // Slow reverse
        double recovery_steer = recovery_steer_direction_ * config_.max_steering_angle;
        
        final_cmd = DriveCommand(reverse_speed, recovery_steer);
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "RECOVERY: counter=%d/%d, cmd=(speed=%.2f, steer=%.2f)",
            recovery_counter_, RECOVERY_DURATION, reverse_speed, recovery_steer);
        
        // Exit recovery after duration (even if still stuck, will re-enter if needed)
        if (recovery_counter_ >= RECOVERY_DURATION) {
            RCLCPP_WARN(this->get_logger(), "RECOVERY MODE: Timeout - trying normal mode");
            in_recovery_mode_ = false;
            stuck_counter_ = 0;
            recovery_counter_ = 0;
        }
    }
    
    // Publish drive command (either normal or recovery)
    publishDriveCommand(final_cmd);
    
    // Update performance metrics
    updatePerformanceMetrics(output, final_cmd);
    
    // Publish visualization if anyone is listening
    if (viz_pub_->get_subscription_count() > 0) {
        ProcessedScan scan = ftg_->getLidarProcessor().processScan(
            msg->ranges, msg->angle_min, msg->angle_max, msg->angle_increment
        );
        publishVisualization(output, scan);
    }
    
    // Performance logging (throttled to reduce overhead)
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "FTG: gaps=%zu, cmd=(%.2f, %.2f), closest=%.2fm",
        output.all_gaps.size(), final_cmd.speed, final_cmd.steering_angle,
        output.closest_point_dist);
    
    // Periodic performance report (every 30 seconds)
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 30000,
        "PERF: dist=%.1fm, avg_spd=%.2fm/s, laps=%d",
        metrics_.total_distance, metrics_.average_speed, metrics_.lap_count);
    
    // Log if emergency stop (and not in recovery)
    if (output.emergency_stop && !in_recovery_mode_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Emergency stop! Closest: %.2fm, stuck_count=%d", 
            output.closest_point_dist, stuck_counter_);
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

void FTGNode::updatePerformanceMetrics(const FTGOutput& output, const DriveCommand& cmd) {
    if (!metrics_initialized_) {
        metrics_start_time_ = this->now();
        metrics_.start_x = current_state_.pose.x;
        metrics_.start_y = current_state_.pose.y;
        metrics_.last_x = current_state_.pose.x;
        metrics_.last_y = current_state_.pose.y;
        metrics_initialized_ = true;
        return;
    }
    
    // Calculate distance traveled
    double dx = current_state_.pose.x - metrics_.last_x;
    double dy = current_state_.pose.y - metrics_.last_y;
    double dist = std::sqrt(dx * dx + dy * dy);
    metrics_.total_distance += dist;
    metrics_.last_x = current_state_.pose.x;
    metrics_.last_y = current_state_.pose.y;
    
    // Update time
    metrics_.total_time = (this->now() - metrics_start_time_).seconds();
    
    // Calculate average speed
    if (metrics_.total_time > 0.0) {
        metrics_.average_speed = metrics_.total_distance / metrics_.total_time;
    }
    
    // Track steering history for variance calculation
    metrics_.steering_history.push_back(cmd.steering_angle);
    if (metrics_.steering_history.size() > METRIC_HISTORY_SIZE) {
        metrics_.steering_history.pop_front();
    }
    
    // Track speed history
    metrics_.speed_history.push_back(cmd.speed);
    if (metrics_.speed_history.size() > METRIC_HISTORY_SIZE) {
        metrics_.speed_history.pop_front();
    }
    
    // Update min obstacle distance
    if (output.closest_point_dist < metrics_.min_obstacle_dist) {
        metrics_.min_obstacle_dist = output.closest_point_dist;
    }
    
    // Check for crash
    if (output.closest_point_dist < CRASH_THRESHOLD) {
        metrics_.crashed = true;
    }
    
    // Count emergency stops
    if (output.emergency_stop) {
        metrics_.emergency_stops++;
    }
    
    // Calculate steering variance
    metrics_.steering_variance = calculateSteeringVariance();
    
    // Check for lap completion using a state machine approach:
    // 1. Must leave the start zone first (was_near_start becomes false)
    // 2. Then re-enter it to count a lap
    // 3. Must have traveled at least 50m since last lap
    double dist_to_start = std::sqrt(
        std::pow(current_state_.pose.x - metrics_.start_x, 2) +
        std::pow(current_state_.pose.y - metrics_.start_y, 2)
    );
    
    constexpr double START_ZONE_RADIUS = 3.0;  // meters
    constexpr double MIN_LAP_DISTANCE = 50.0;  // minimum distance for a valid lap
    
    bool is_near_start = dist_to_start < START_ZONE_RADIUS;
    double distance_since_last_lap = metrics_.total_distance - metrics_.last_lap_distance;
    
    if (is_near_start && !metrics_.was_near_start && distance_since_last_lap > MIN_LAP_DISTANCE) {
        // Transitioning INTO start zone after being away, and traveled enough distance
        metrics_.lap_count++;
        metrics_.lap_time = metrics_.total_time;
        metrics_.last_lap_distance = metrics_.total_distance;
        RCLCPP_INFO(this->get_logger(), 
            "=== LAP %d COMPLETED === Time: %.1fs, Lap Distance: %.1fm, Avg Speed: %.2f m/s",
            metrics_.lap_count, metrics_.lap_time, distance_since_last_lap, metrics_.average_speed);
    }
    
    // Update start zone state
    metrics_.was_near_start = is_near_start;
}

double FTGNode::calculateSteeringVariance() const {
    if (metrics_.steering_history.size() < 2) {
        return 0.0;
    }
    
    // Calculate mean
    double sum = 0.0;
    for (double s : metrics_.steering_history) {
        sum += s;
    }
    double mean = sum / metrics_.steering_history.size();
    
    // Calculate variance
    double variance = 0.0;
    for (double s : metrics_.steering_history) {
        variance += (s - mean) * (s - mean);
    }
    variance /= metrics_.steering_history.size();
    
    return variance;
}

void FTGNode::printPerformanceSummary() {
    RCLCPP_INFO(this->get_logger(),
        "\n========== PERFORMANCE SUMMARY ==========\n"
        "Total Time: %.1f seconds\n"
        "Total Distance: %.1f meters\n"
        "Average Speed: %.2f m/s\n"
        "Steering Variance: %.4f (lower is smoother)\n"
        "Min Obstacle Distance: %.2f m\n"
        "Emergency Stops: %d\n"
        "Recovery Events: %d\n"
        "Laps Completed: %d\n"
        "Crashed: %s\n"
        "=========================================",
        metrics_.total_time,
        metrics_.total_distance,
        metrics_.average_speed,
        metrics_.steering_variance,
        metrics_.min_obstacle_dist,
        metrics_.emergency_stops,
        metrics_.recovery_events,
        metrics_.lap_count,
        metrics_.crashed ? "YES" : "NO"
    );
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
