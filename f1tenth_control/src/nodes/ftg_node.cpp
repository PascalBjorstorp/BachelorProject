#include "nodes/ftg_node.hpp"


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
    param_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&FTGNode::parametersCallback, this, std::placeholders::_1)
    );
    
    // QoS profiles
    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    auto reliable_qos = rclcpp::QoS(10);
    
    // Subscribers
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", sensor_qos,
        std::bind(&FTGNode::scanCallback, this, std::placeholders::_1)
    );
    
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", sensor_qos,
        std::bind(&FTGNode::odomCallback, this, std::placeholders::_1)
    );
    
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
        "ftg/enable", reliable_qos,
        std::bind(&FTGNode::enableCallback, this, std::placeholders::_1)
    );
    
    // Publishers
    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        "drive", reliable_qos
    );
    
    RCLCPP_INFO(get_logger(), "FTG Node initialized");
    RCLCPP_INFO(get_logger(), "  Subscribing to: scan, odom");
    RCLCPP_INFO(get_logger(), "  Publishing to: drive");
}

void FTGNode::declareParameters() {
    // Vehicle parameters
    declare_parameter("wheelbase", 0.3302);
    declare_parameter("car_width", 0.273);

    // Speed control
    declare_parameter("max_speed", 2.0);
    declare_parameter("min_speed", 1.0);
    declare_parameter("speed_full_range", 4.0);
    declare_parameter("steer_slowdown_gain", 0.5);

    // Steering control
    declare_parameter("max_steering", 0.4189);
    declare_parameter("steering_gain", 1.0);
    declare_parameter("max_steering_rate", 2.8492);
    declare_parameter("target_ema_alpha", 0.35);

    // Weighted free-space scoring
    declare_parameter("heading_weight", 1.0);
    declare_parameter("score_power", 2.0);
    declare_parameter("clearance_cone_scale", 1.5);
    declare_parameter("min_score_range", 0.3);

    // Safety
    declare_parameter("emergency_brake_distance", 0.15);

    // LiDAR processing
    declare_parameter("disparity_threshold", 0.5);
    declare_parameter("wall_margin", 0.15);
    declare_parameter("gap_threshold", 0.5);
    declare_parameter("min_gap_width", 0.15);

    // Generic LiDAR preprocessing
    declare_parameter("lidar.range_min", 0.1);
    declare_parameter("lidar.range_max", 10.0);
    declare_parameter("lidar.angle_min", -1.5708);
    declare_parameter("lidar.angle_max", 1.5708);
    declare_parameter("lidar.apply_median_filter", true);
    declare_parameter("lidar.median_window_size", 3);
}

void FTGNode::loadParameters() {
    // Vehicle parameters
    config_.wheelbase = get_parameter("wheelbase").as_double();
    config_.car_width = get_parameter("car_width").as_double();

    // Speed control
    config_.max_speed = get_parameter("max_speed").as_double();
    config_.min_speed = get_parameter("min_speed").as_double();
    config_.speed_full_range = get_parameter("speed_full_range").as_double();
    config_.steer_slowdown_gain = get_parameter("steer_slowdown_gain").as_double();

    // Steering control
    config_.max_steering = get_parameter("max_steering").as_double();
    config_.steering_gain = get_parameter("steering_gain").as_double();
    config_.max_steering_rate = get_parameter("max_steering_rate").as_double();
    config_.target_ema_alpha = get_parameter("target_ema_alpha").as_double();

    // Weighted free-space scoring
    config_.heading_weight = get_parameter("heading_weight").as_double();
    config_.score_power = get_parameter("score_power").as_double();
    config_.clearance_cone_scale = get_parameter("clearance_cone_scale").as_double();
    config_.min_score_range = get_parameter("min_score_range").as_double();

    // Safety
    config_.emergency_brake_distance = get_parameter("emergency_brake_distance").as_double();

    // LiDAR processing
    config_.disparity_threshold = get_parameter("disparity_threshold").as_double();
    config_.wall_margin = get_parameter("wall_margin").as_double();
    config_.gap_threshold = get_parameter("gap_threshold").as_double();
    config_.min_gap_width = get_parameter("min_gap_width").as_double();

    // Generic LiDAR preprocessing config
    config_.lidar_config.range_min = get_parameter("lidar.range_min").as_double();
    config_.lidar_config.range_max = get_parameter("lidar.range_max").as_double();
    config_.lidar_config.angle_min = get_parameter("lidar.angle_min").as_double();
    config_.lidar_config.angle_max = get_parameter("lidar.angle_max").as_double();
    config_.lidar_config.apply_median_filter = get_parameter("lidar.apply_median_filter").as_bool();
    config_.lidar_config.median_window_size = get_parameter("lidar.median_window_size").as_int();
}

rcl_interfaces::msg::SetParametersResult FTGNode::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    for (const auto& param : parameters) {
        RCLCPP_INFO(get_logger(), "Parameter '%s' changed", param.get_name().c_str());
    }
    
    // Reload all parameters and update algorithm
    loadParameters();
    if (ftg_) {
        ftg_->setConfig(config_);
    }
    
    return result;
}

void FTGNode::scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
    if (!enabled_) {
        return;
    }
    
    // Run FTG algorithm
    FTGOutput output = ftg_->compute(
        msg->ranges,
        msg->angle_min,
        msg->angle_max,
        msg->angle_increment
    );
    
    // Steering smoothing is now handled internally by the FTG algorithm
    // via rate limiting (max_steering_rate)
    last_steering_ = output.command.steering_angle;
    
    // Publish drive command
    publishDriveCommand(output.command);
    
    // Update performance metrics
    updatePerformanceMetrics(output, output.command);
    
    // Performance logging (throttled to reduce overhead)
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
        "FTG: gaps=%zu, cmd=(%.2f, %.2f), closest=%.2fm",
        output.all_gaps.size(), output.command.speed, output.command.steering_angle,
        output.closest_point_dist);
    
    // Periodic performance report (every 30 seconds)
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 30000,
        "PERF: dist=%.1fm, avg_spd=%.2fm/s, laps=%d",
        metrics_.total_distance, metrics_.average_speed, metrics_.lap_count);
    
    // Log if emergency stop (and not in recovery)
    if (output.emergency_stop && !in_recovery_mode_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Emergency stop! Closest: %.2fm, stuck_count=%d", 
            output.closest_point_dist, stuck_counter_);
    }
}

void FTGNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
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

void FTGNode::enableCallback(const std_msgs::msg::Bool::ConstSharedPtr msg) {
    enabled_ = msg->data;
    RCLCPP_INFO(get_logger(), "FTG %s", enabled_ ? "enabled" : "disabled");
    
    if (!enabled_) {
        // Publish zero command when disabled
        publishDriveCommand(DriveCommand(0.0, 0.0));
    }
}

void FTGNode::publishDriveCommand(const DriveCommand& cmd) {
    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.header.stamp = now();
    drive_msg.header.frame_id = "base_link";
    drive_msg.drive.speed = cmd.speed;
    drive_msg.drive.steering_angle = cmd.steering_angle;
    
    drive_pub_->publish(drive_msg);
}

void FTGNode::updatePerformanceMetrics(const FTGOutput& output, const DriveCommand& cmd) {
    if (!metrics_initialized_) {
        metrics_start_time_ = now();
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
    metrics_.total_time = (now() - metrics_start_time_).seconds();
    
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
        RCLCPP_INFO(get_logger(), 
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
    RCLCPP_INFO(get_logger(),
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

// Register as composable node for zero-copy intra-process communication
RCLCPP_COMPONENTS_REGISTER_NODE(f1tenth_control::FTGNode)

// Main function
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<f1tenth_control::FTGNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    
    return 0;
}
