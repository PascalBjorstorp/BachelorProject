#include "f1tenth_control/nodes/stanley_node.hpp"
#include <tf2/utils.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace f1tenth_control {

StanleyNode::StanleyNode(const rclcpp::NodeOptions& options)
    : Node("stanley_node", options)
{
    RCLCPP_INFO(get_logger(), "Initializing Stanley Controller Node");
    
    // Declare and load parameters
    declareParameters();
    loadParameters();
    
    // Initialize controller
    controller_ = std::make_unique<Stanley>(config_);
    
    // Initialize lap timing
    lap_start_time_ = now().seconds();
    
    // Load trajectory
    if (!trajectory_file_.empty()) {
        if (controller_->loadTrajectory(trajectory_file_)) {
            RCLCPP_INFO(get_logger(), "Loaded trajectory with %zu waypoints (%.1f m)",
                        controller_->getTrajectory().size(),
                        controller_->getTrajectoryLength());
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to load trajectory: %s", trajectory_file_.c_str());
        }
    }
    
    // Setup publishers/subscribers
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&StanleyNode::odomCallback, this, std::placeholders::_1)
    );
    
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/stanley_enable", 10,
        std::bind(&StanleyNode::enableCallback, this, std::placeholders::_1)
    );
    
    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        "/drive", 10
    );
    
    if (publish_visualization_) {
        viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/stanley_viz", 10);
        path_pub_ = create_publisher<nav_msgs::msg::Path>(
            "/stanley_path", rclcpp::QoS(1).transient_local()
        );
        publishTrajectoryPath();
    }
    
    // Setup control timer
    auto period = std::chrono::duration<double>(1.0 / control_rate_);
    control_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&StanleyNode::controlLoop, this)
    );
    
    // Setup parameter callback
    param_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&StanleyNode::parametersCallback, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(get_logger(), "Stanley Controller Node initialized");
    RCLCPP_INFO(get_logger(), "  Trajectory: %s (%zu points)", 
                trajectory_file_.c_str(), controller_->getTrajectory().size());
    RCLCPP_INFO(get_logger(), "  k_e: %.2f, k_h: %.2f, k_s: %.2f",
                config_.k_e, config_.k_h, config_.k_s);
    RCLCPP_INFO(get_logger(), "  Speed: %.1f - %.1f m/s (gain: %.2f)",
                config_.min_speed, config_.max_speed, config_.speed_gain);
    RCLCPP_INFO(get_logger(), "  Feedforward: %s (gain: %.2f)",
                config_.use_feedforward ? "ON" : "OFF", config_.feedforward_gain);
}

void StanleyNode::declareParameters() {
    // Trajectory
    declare_parameter("trajectory_file", "");
    
    // Stanley gains
    declare_parameter("k_e", 2.5);           // Cross-track error gain
    declare_parameter("k_h", 1.0);           // Heading error gain
    declare_parameter("k_s", 1.0);           // Softening constant
    declare_parameter("k_d", 0.1);           // Damping gain (suppresses oscillation)
    
    // Feedforward
    declare_parameter("use_feedforward", true);
    declare_parameter("feedforward_gain", 1.0);
    
    // Speed
    declare_parameter("max_speed", 15.0);
    declare_parameter("min_speed", 1.0);
    declare_parameter("speed_gain", 1.0);
    
    // Steering
    declare_parameter("max_steering", 0.4189);
    declare_parameter("max_steering_rate", 1.5);  // [rad/s] Tighter rate limit to reduce oscillation
    
    // Vehicle
    declare_parameter("wheelbase", 0.3302);
    
    // Stability
    declare_parameter("curvature_speed_factor", 0.8);
    
    // Misc
    declare_parameter("publish_visualization", true);
    declare_parameter("control_rate", 50.0);
}

void StanleyNode::loadParameters() {
    trajectory_file_ = get_parameter("trajectory_file").as_string();
    
    config_.k_e = get_parameter("k_e").as_double();
    config_.k_h = get_parameter("k_h").as_double();
    config_.k_s = get_parameter("k_s").as_double();
    config_.k_d = get_parameter("k_d").as_double();
    
    config_.use_feedforward = get_parameter("use_feedforward").as_bool();
    config_.feedforward_gain = get_parameter("feedforward_gain").as_double();
    
    config_.max_speed = get_parameter("max_speed").as_double();
    config_.min_speed = get_parameter("min_speed").as_double();
    config_.speed_gain = get_parameter("speed_gain").as_double();
    
    config_.max_steering = get_parameter("max_steering").as_double();
    config_.max_steering_rate = get_parameter("max_steering_rate").as_double();
    config_.wheelbase = get_parameter("wheelbase").as_double();
    config_.curvature_speed_factor = get_parameter("curvature_speed_factor").as_double();
    
    publish_visualization_ = get_parameter("publish_visualization").as_bool();
    control_rate_ = get_parameter("control_rate").as_double();
    config_.control_rate = control_rate_;  // Pass control rate to algorithm for rate limiting
}

rcl_interfaces::msg::SetParametersResult StanleyNode::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    for (const auto& param : parameters) {
        if (param.get_name() == "k_e") {
            config_.k_e = param.as_double();
        } else if (param.get_name() == "k_h") {
            config_.k_h = param.as_double();
        } else if (param.get_name() == "k_s") {
            config_.k_s = param.as_double();
        } else if (param.get_name() == "use_feedforward") {
            config_.use_feedforward = param.as_bool();
        } else if (param.get_name() == "feedforward_gain") {
            config_.feedforward_gain = param.as_double();
        } else if (param.get_name() == "max_speed") {
            config_.max_speed = param.as_double();
        } else if (param.get_name() == "min_speed") {
            config_.min_speed = param.as_double();
        } else if (param.get_name() == "speed_gain") {
            config_.speed_gain = param.as_double();
        } else if (param.get_name() == "curvature_speed_factor") {
            config_.curvature_speed_factor = param.as_double();
        }
    }
    
    if (controller_) {
        controller_->setConfig(config_);
        RCLCPP_INFO(get_logger(), "Parameters updated: k_e=%.2f, k_h=%.2f, k_s=%.2f, max_speed=%.1f",
                    config_.k_e, config_.k_h, config_.k_s, config_.max_speed);
    }
    
    return result;
}

void StanleyNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
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
    
    current_state_.velocity = msg->twist.twist.linear.x;
    current_state_.angular_velocity = msg->twist.twist.angular.z;
    
    state_received_ = true;
}

void StanleyNode::enableCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    enabled_ = msg->data;
    RCLCPP_INFO(get_logger(), "Stanley controller %s", enabled_ ? "ENABLED" : "DISABLED");
}

void StanleyNode::controlLoop() {
    if (!state_received_ || !controller_->hasTrajectory()) {
        return;
    }
    
    // Copy state under lock
    VehicleState state;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state = current_state_;
    }
    
    // Compute control
    auto output = controller_->compute(state);
    
    if (!output.valid) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Invalid controller output");
        return;
    }
    
    // Compact status output every 5 seconds
    static int debug_counter = 0;
    if (++debug_counter >= 1000) {  // At 200 Hz, print every 5 seconds
        debug_counter = 0;
        double avg_cte = (cte_count_ > 0) ? total_cte_ / cte_count_ : 0.0;
        RCLCPP_INFO(get_logger(), 
                    "CTE: %.2fm (avg: %.2fm) | Speed: %.1f m/s | Lap: %d",
                    output.cross_track_error, avg_cte, output.target_speed, lap_count_);
    }
    
    // Publish drive command
    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.header.stamp = now();
    drive_msg.header.frame_id = "base_link";
    
    if (enabled_) {
        drive_msg.drive.steering_angle = output.steering_angle;
        drive_msg.drive.speed = output.target_speed;
    } else {
        drive_msg.drive.steering_angle = 0.0;
        drive_msg.drive.speed = 0.0;
    }
    
    drive_pub_->publish(drive_msg);
    
    // Update metrics
    updateMetrics(output);
    checkLapCompletion(output.closest_idx);
    
    // Publish visualization
    if (publish_visualization_) {
        publishVisualization(output);
    }
}

void StanleyNode::publishVisualization(const StanleyOutput& output) {
    visualization_msgs::msg::MarkerArray markers;
    
    // Marker for closest point on trajectory
    visualization_msgs::msg::Marker closest_marker;
    closest_marker.header.frame_id = "map";
    closest_marker.header.stamp = now();
    closest_marker.ns = "stanley";
    closest_marker.id = 0;
    closest_marker.type = visualization_msgs::msg::Marker::SPHERE;
    closest_marker.action = visualization_msgs::msg::Marker::ADD;
    
    const auto& trajectory = controller_->getTrajectory();
    if (output.closest_idx < trajectory.size()) {
        closest_marker.pose.position.x = trajectory[output.closest_idx].x;
        closest_marker.pose.position.y = trajectory[output.closest_idx].y;
        closest_marker.pose.position.z = 0.1;
    }
    closest_marker.scale.x = 0.2;
    closest_marker.scale.y = 0.2;
    closest_marker.scale.z = 0.2;
    closest_marker.color.r = 0.0;
    closest_marker.color.g = 1.0;
    closest_marker.color.b = 0.0;
    closest_marker.color.a = 1.0;
    markers.markers.push_back(closest_marker);
    
    // Text marker for debug info
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "map";
    text_marker.header.stamp = now();
    text_marker.ns = "stanley";
    text_marker.id = 1;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = current_state_.pose.x;
    text_marker.pose.position.y = current_state_.pose.y + 1.0;
    text_marker.pose.position.z = 1.0;
    text_marker.scale.z = 0.3;
    
    char buf[256];
    snprintf(buf, sizeof(buf), 
             "CTE: %.3fm  HE: %.1f°\nFF: %.3f  Steer: %.1f°\nSpeed: %.1f m/s",
             output.cross_track_error,
             output.heading_error * 180.0 / M_PI,
             output.feedforward_steering,
             output.steering_angle * 180.0 / M_PI,
             output.target_speed);
    text_marker.text = buf;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    markers.markers.push_back(text_marker);
    
    viz_pub_->publish(markers);
}

void StanleyNode::publishTrajectoryPath() {
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = now();
    
    for (const auto& pt : controller_->getTrajectory()) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = pt.x;
        pose.pose.position.y = pt.y;
        pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, pt.heading);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        
        path_msg.poses.push_back(pose);
    }
    
    path_pub_->publish(path_msg);
}

void StanleyNode::updateMetrics(const StanleyOutput& output) {
    double abs_cte = std::abs(output.cross_track_error);
    total_cte_ += abs_cte;
    max_cte_ = std::max(max_cte_, abs_cte);
    cte_count_++;
}

void StanleyNode::checkLapCompletion(size_t current_idx) {
    const auto& trajectory = controller_->getTrajectory();
    if (trajectory.empty()) return;
    
    size_t n = trajectory.size();
    size_t start_region = n / 20;  // First 5% of track
    
    // Detect crossing the start/finish
    bool in_start_region = current_idx < start_region;
    bool was_near_end = last_lap_idx_ > (n - start_region);
    
    if (in_start_region && was_near_end && !crossed_start_) {
        crossed_start_ = true;
        lap_count_++;
        
        double current_time = now().seconds();
        double lap_time = current_time - lap_start_time_;
        double avg_cte = (cte_count_ > 0) ? total_cte_ / cte_count_ : 0.0;
        
        RCLCPP_INFO(get_logger(), 
                    "Lap %d complete! Time: %.2fs, Avg CTE: %.3fm, Max CTE: %.3fm",
                    lap_count_, lap_time, avg_cte, max_cte_);
        
        // Reset for next lap
        lap_start_time_ = current_time;
        total_cte_ = 0.0;
        max_cte_ = 0.0;
        cte_count_ = 0;
    }
    
    if (!in_start_region) {
        crossed_start_ = false;
    }
    
    last_lap_idx_ = current_idx;
}

}  // namespace f1tenth_control

// Component registration
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(f1tenth_control::StanleyNode)

// Main entry point
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<f1tenth_control::StanleyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
