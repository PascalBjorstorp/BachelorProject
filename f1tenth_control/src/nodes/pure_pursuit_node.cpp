#include "f1tenth_control/nodes/pure_pursuit_node.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace f1tenth_control {

PurePursuitNode::PurePursuitNode(const rclcpp::NodeOptions& options)
    : Node("pure_pursuit_node", options)
{
    RCLCPP_INFO(get_logger(), "Initializing Pure Pursuit Node");
    
    // Declare and load parameters
    declareParameters();
    loadParameters();
    
    // Create controller
    controller_ = std::make_unique<PurePursuit>(config_);
    
    // Load trajectory
    if (!loadTrajectory()) {
        RCLCPP_ERROR(get_logger(), "Failed to load trajectory from: %s", trajectory_file_.c_str());
        RCLCPP_WARN(get_logger(), "Controller will be disabled until trajectory is loaded");
    }
    
    // Setup subscribers
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&PurePursuitNode::odomCallback, this, std::placeholders::_1)
    );
    
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/pp_enable", 10,
        std::bind(&PurePursuitNode::enableCallback, this, std::placeholders::_1)
    );
    
    // Local raceline from lateral planner (overrides loaded trajectory when received)
    local_raceline_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/local_raceline", 10,
        std::bind(&PurePursuitNode::localRacelineCallback, this, std::placeholders::_1)
    );
    
    // Setup publishers
    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        "/drive", 10
    );
    
    if (publish_visualization_) {
        viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/pp_viz", 10
        );
        path_pub_ = create_publisher<nav_msgs::msg::Path>(
            "/pp_path", rclcpp::QoS(1).transient_local()
        );
        
        // Publish trajectory as path for visualization
        publishTrajectoryPath();
    }
    
    // Setup control timer
    auto period = std::chrono::duration<double>(1.0 / control_rate_);
    control_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&PurePursuitNode::controlLoop, this)
    );
    
    // Setup parameter callback
    param_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&PurePursuitNode::parametersCallback, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(get_logger(), "Pure Pursuit Node initialized");
    RCLCPP_INFO(get_logger(), "  Trajectory: %s (%zu points)", 
                trajectory_file_.c_str(), controller_->getTrajectory().size());
    RCLCPP_INFO(get_logger(), "  Lookahead: %.2f - %.2f m (gain: %.2f)",
                config_.min_lookahead, config_.max_lookahead, config_.lookahead_gain);
    RCLCPP_INFO(get_logger(), "  Speed: %.1f - %.1f m/s (gain: %.2f)",
                config_.min_speed, config_.max_speed, config_.speed_gain);
}

void PurePursuitNode::declareParameters() {
    // Trajectory
    declare_parameter("trajectory_file", "");
    
    // Lookahead
    declare_parameter("min_lookahead", 0.5);
    declare_parameter("max_lookahead", 2.5);
    declare_parameter("lookahead_gain", 0.15);
    
    // Speed
    declare_parameter("max_speed", 8.0);
    declare_parameter("min_speed", 1.0);
    declare_parameter("speed_gain", 0.8);
    
    // Steering
    declare_parameter("max_steering", 0.4189);
    declare_parameter("max_steering_rate", 2.0);
    
    // Vehicle
    declare_parameter("wheelbase", 0.3302);
    
    // Stability
    declare_parameter("curvature_speed_factor", 0.5);
    
    // Misc
    declare_parameter("publish_visualization", true);
    declare_parameter("control_rate", 50.0);
}

void PurePursuitNode::loadParameters() {
    trajectory_file_ = get_parameter("trajectory_file").as_string();
    
    config_.min_lookahead = get_parameter("min_lookahead").as_double();
    config_.max_lookahead = get_parameter("max_lookahead").as_double();
    config_.lookahead_gain = get_parameter("lookahead_gain").as_double();
    
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

rcl_interfaces::msg::SetParametersResult PurePursuitNode::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    for (const auto& param : parameters) {
        if (param.get_name() == "min_lookahead") {
            config_.min_lookahead = param.as_double();
        } else if (param.get_name() == "max_lookahead") {
            config_.max_lookahead = param.as_double();
        } else if (param.get_name() == "lookahead_gain") {
            config_.lookahead_gain = param.as_double();
        } else if (param.get_name() == "max_speed") {
            config_.max_speed = param.as_double();
        } else if (param.get_name() == "min_speed") {
            config_.min_speed = param.as_double();
        } else if (param.get_name() == "speed_gain") {
            config_.speed_gain = param.as_double();
        } else if (param.get_name() == "max_steering") {
            config_.max_steering = param.as_double();
        }
    }
    
    if (controller_) {
        controller_->setConfig(config_);
    }
    
    return result;
}

bool PurePursuitNode::loadTrajectory() {
    if (trajectory_file_.empty()) {
        return false;
    }
    
    if (controller_->loadTrajectory(trajectory_file_)) {
        trajectory_loaded_ = true;
        RCLCPP_INFO(get_logger(), "Loaded trajectory with %zu waypoints (%.1f m)",
                    controller_->getTrajectory().size(),
                    controller_->getTrajectoryLength());
        return true;
    }
    
    return false;
}

void PurePursuitNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
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
}

void PurePursuitNode::enableCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    enabled_ = msg->data;
    if (enabled_) {
        RCLCPP_INFO(get_logger(), "Pure Pursuit ENABLED");
    } else {
        RCLCPP_INFO(get_logger(), "Pure Pursuit DISABLED");
        // Stop the car
        publishDriveCommand(0.0, 0.0);
    }
}

void PurePursuitNode::localRacelineCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.empty()) {
        return;
    }

    std::vector<TrajectoryPoint> new_traj;
    new_traj.reserve(msg->poses.size());

    double cumulative_s = 0.0;
    for (size_t i = 0; i < msg->poses.size(); ++i) {
        const auto& pose = msg->poses[i];
        TrajectoryPoint tp;
        tp.x = pose.pose.position.x;
        tp.y = pose.pose.position.y;
        // Velocity encoded in z by the lateral planner
        tp.velocity = pose.pose.position.z;
        // Heading from quaternion (yaw only)
        double siny = 2.0 * (pose.pose.orientation.w * pose.pose.orientation.z +
                              pose.pose.orientation.x * pose.pose.orientation.y);
        double cosy = 1.0 - 2.0 * (pose.pose.orientation.y * pose.pose.orientation.y +
                                     pose.pose.orientation.z * pose.pose.orientation.z);
        tp.heading = std::atan2(siny, cosy);

        // Compute arc length from consecutive points
        if (i > 0) {
            double dx = tp.x - new_traj.back().x;
            double dy = tp.y - new_traj.back().y;
            cumulative_s += std::sqrt(dx * dx + dy * dy);
        }
        tp.arc_length = cumulative_s;

        // Curvature from finite differences (computed after all points added)
        tp.curvature = 0.0;
        new_traj.push_back(tp);
    }

    // Compute curvature from heading differences
    for (size_t i = 1; i + 1 < new_traj.size(); ++i) {
        double ds = new_traj[i + 1].arc_length - new_traj[i - 1].arc_length;
        if (ds > 1e-6) {
            double dtheta = new_traj[i + 1].heading - new_traj[i - 1].heading;
            // Normalize to [-pi, pi]
            while (dtheta > M_PI) dtheta -= 2.0 * M_PI;
            while (dtheta < -M_PI) dtheta += 2.0 * M_PI;
            new_traj[i].curvature = dtheta / ds;
        }
    }

    controller_->setTrajectory(new_traj);
    trajectory_loaded_ = true;
}

void PurePursuitNode::controlLoop() {
    if (!enabled_ || !trajectory_loaded_) {
        return;
    }
    
    VehicleState state;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state = current_state_;
    }
    
    // Compute control
    PurePursuitOutput output = controller_->compute(state);
    
    if (output.valid) {
        publishDriveCommand(output.steering_angle, output.target_speed);
        
        if (publish_visualization_) {
            publishVisualization(output);
        }
        
        updateMetrics(output);
    } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                            "Invalid Pure Pursuit output");
    }
}

void PurePursuitNode::publishDriveCommand(double steering, double speed) {
    auto msg = ackermann_msgs::msg::AckermannDriveStamped();
    msg.header.stamp = now();
    msg.header.frame_id = "base_link";
    msg.drive.steering_angle = steering;
    msg.drive.speed = speed;
    drive_pub_->publish(msg);
}

void PurePursuitNode::publishVisualization(const PurePursuitOutput& output) {
    visualization_msgs::msg::MarkerArray markers;
    
    // Marker 1: Lookahead target point
    visualization_msgs::msg::Marker target_marker;
    target_marker.header.stamp = now();
    target_marker.header.frame_id = "map";
    target_marker.ns = "pure_pursuit";
    target_marker.id = 0;
    target_marker.type = visualization_msgs::msg::Marker::SPHERE;
    target_marker.action = visualization_msgs::msg::Marker::ADD;
    target_marker.pose.position.x = output.target_point.x;
    target_marker.pose.position.y = output.target_point.y;
    target_marker.pose.position.z = 0.1;
    target_marker.scale.x = 0.3;
    target_marker.scale.y = 0.3;
    target_marker.scale.z = 0.3;
    target_marker.color.r = 0.0;
    target_marker.color.g = 1.0;
    target_marker.color.b = 0.0;
    target_marker.color.a = 0.8;
    markers.markers.push_back(target_marker);
    
    // Marker 2: Line from car to target
    VehicleState state;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state = current_state_;
    }
    
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.stamp = now();
    line_marker.header.frame_id = "map";
    line_marker.ns = "pure_pursuit";
    line_marker.id = 1;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.scale.x = 0.05;
    line_marker.color.r = 0.0;
    line_marker.color.g = 0.8;
    line_marker.color.b = 0.8;
    line_marker.color.a = 0.6;
    
    geometry_msgs::msg::Point p1, p2;
    p1.x = state.pose.x;
    p1.y = state.pose.y;
    p1.z = 0.1;
    p2.x = output.target_point.x;
    p2.y = output.target_point.y;
    p2.z = 0.1;
    line_marker.points.push_back(p1);
    line_marker.points.push_back(p2);
    markers.markers.push_back(line_marker);
    
    // Marker 3: Cross-track error indicator
    const auto& traj = controller_->getTrajectory();
    if (output.closest_idx < traj.size()) {
        visualization_msgs::msg::Marker cte_marker;
        cte_marker.header.stamp = now();
        cte_marker.header.frame_id = "map";
        cte_marker.ns = "pure_pursuit";
        cte_marker.id = 2;
        cte_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        cte_marker.action = visualization_msgs::msg::Marker::ADD;
        cte_marker.scale.x = 0.03;
        
        // Color based on error magnitude
        double err_normalized = std::min(std::abs(output.cross_track_error) / 0.5, 1.0);
        cte_marker.color.r = err_normalized;
        cte_marker.color.g = 1.0 - err_normalized;
        cte_marker.color.b = 0.0;
        cte_marker.color.a = 0.8;
        
        p1.x = state.pose.x;
        p1.y = state.pose.y;
        p2.x = traj[output.closest_idx].x;
        p2.y = traj[output.closest_idx].y;
        cte_marker.points.push_back(p1);
        cte_marker.points.push_back(p2);
        markers.markers.push_back(cte_marker);
    }
    
    viz_pub_->publish(markers);
}

void PurePursuitNode::publishTrajectoryPath() {
    if (!controller_->hasTrajectory()) return;
    
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = now();
    path_msg.header.frame_id = "map";
    
    for (const auto& pt : controller_->getTrajectory()) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = pt.x;
        pose.pose.position.y = pt.y;
        pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, pt.heading);
        pose.pose.orientation = tf2::toMsg(q);
        
        path_msg.poses.push_back(pose);
    }
    
    path_pub_->publish(path_msg);
}

void PurePursuitNode::updateMetrics(const PurePursuitOutput& output) {
    VehicleState state;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state = current_state_;
    }
    
    if (!metrics_.initialized) {
        metrics_.start_time = now();
        metrics_.start_x = state.pose.x;
        metrics_.start_y = state.pose.y;
        metrics_.last_x = state.pose.x;
        metrics_.last_y = state.pose.y;
        metrics_.initialized = true;
        return;
    }
    
    // Update distance
    double dx = state.pose.x - metrics_.last_x;
    double dy = state.pose.y - metrics_.last_y;
    double dist = std::hypot(dx, dy);
    metrics_.total_distance += dist;
    metrics_.last_x = state.pose.x;
    metrics_.last_y = state.pose.y;
    
    // Track cross-track error
    double cte = std::abs(output.cross_track_error);
    metrics_.max_cross_track_error = std::max(metrics_.max_cross_track_error, cte);
    metrics_.sum_cross_track_error += cte;
    metrics_.update_count++;
    
    // Check for lap completion (near start position)
    double dist_to_start = std::hypot(state.pose.x - metrics_.start_x, 
                                       state.pose.y - metrics_.start_y);
    static bool was_far = false;
    if (dist_to_start > 5.0) {
        was_far = true;
    }
    if (was_far && dist_to_start < 2.0) {
        was_far = false;
        metrics_.lap_count += 1.0;
        double elapsed = (now() - metrics_.start_time).seconds();
        RCLCPP_INFO(get_logger(), 
                    "Lap %.0f complete! Time: %.2fs, Avg CTE: %.3fm, Max CTE: %.3fm",
                    metrics_.lap_count, elapsed,
                    metrics_.sum_cross_track_error / metrics_.update_count,
                    metrics_.max_cross_track_error);
    }
}

void PurePursuitNode::printMetricsSummary() {
    if (!metrics_.initialized) return;
    
    double elapsed = (now() - metrics_.start_time).seconds();
    double avg_cte = metrics_.update_count > 0 
                     ? metrics_.sum_cross_track_error / metrics_.update_count : 0.0;
    
    RCLCPP_INFO(get_logger(), "=== Pure Pursuit Performance ===");
    RCLCPP_INFO(get_logger(), "  Total distance: %.1f m", metrics_.total_distance);
    RCLCPP_INFO(get_logger(), "  Elapsed time:   %.1f s", elapsed);
    RCLCPP_INFO(get_logger(), "  Average speed:  %.2f m/s", metrics_.total_distance / elapsed);
    RCLCPP_INFO(get_logger(), "  Laps:           %.0f", metrics_.lap_count);
    RCLCPP_INFO(get_logger(), "  Avg CTE:        %.3f m", avg_cte);
    RCLCPP_INFO(get_logger(), "  Max CTE:        %.3f m", metrics_.max_cross_track_error);
}

}  // namespace f1tenth_control

// Component registration for composable nodes
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(f1tenth_control::PurePursuitNode)

// Main entry point for standalone executable
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<f1tenth_control::PurePursuitNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
