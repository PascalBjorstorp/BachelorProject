#include "f1tenth_control/nodes/pure_pursuit_node.hpp"
#include <algorithm>
#include <cmath>

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
        "/odom", rclcpp::SensorDataQoS(),
        std::bind(&PurePursuitNode::odomCallback, this, std::placeholders::_1)
    );

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic_, rclcpp::SensorDataQoS(),
        std::bind(&PurePursuitNode::poseCallback, this, std::placeholders::_1)
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
    }
    
    // Setup parameter callback
    param_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&PurePursuitNode::parametersCallback, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(get_logger(), "Pure Pursuit Node initialized");
    RCLCPP_INFO(get_logger(), "  Trajectory: %s (%zu points)", 
                trajectory_file_.c_str(), controller_->getTrajectory().size());
    RCLCPP_INFO(get_logger(), "  Lookahead: %.2f - %.2f m (gain: %.2f)",
                config_.min_lookahead, config_.max_lookahead, config_.lookahead_gain);
    RCLCPP_INFO(get_logger(), "  Max speed cap: %.2f m/s", max_speed_);
    RCLCPP_INFO(get_logger(), "  Lookahead adapt: cte_weight=%.2f cte_gain=%.3f curvature_gain=%.3f",
                config_.cte_lookahead_weight, config_.cte_lookahead_gain, config_.curvature_lookahead_gain);
    RCLCPP_INFO(get_logger(), "  Speed adapt: curvature_factor=%.3f floor_ratio=%.2f",
                config_.curvature_speed_factor, config_.curvature_speed_floor_ratio);
}

void PurePursuitNode::declareParameters() {
    // Trajectory
    declare_parameter("trajectory_file", "");
    
    // Lookahead
    declare_parameter("min_lookahead", 0.5);
    declare_parameter("max_lookahead", 2.5);
    declare_parameter("lookahead_gain", 0.15);
    declare_parameter("max_speed", 2.0);
    declare_parameter("cte_lookahead_weight", 1.0);
    declare_parameter("cte_lookahead_gain", 0.03);
    declare_parameter("curvature_lookahead_gain", 0.0);
    declare_parameter("curvature_speed_factor", 0.20);
    declare_parameter("curvature_speed_floor_ratio", 0.85);
    
    // Steering
    declare_parameter("max_steering", 0.4189);
    
    // Vehicle
    declare_parameter("wheelbase", 0.3302);
    
    // Misc
    declare_parameter("publish_visualization", true);
    declare_parameter("pose_topic", std::string("/ekf_pose"));
    declare_parameter("pose_timeout_s", 0.1);
}

void PurePursuitNode::loadParameters() {
    trajectory_file_ = get_parameter("trajectory_file").as_string();
    
    config_.min_lookahead = get_parameter("min_lookahead").as_double();
    config_.max_lookahead = get_parameter("max_lookahead").as_double();
    config_.lookahead_gain = get_parameter("lookahead_gain").as_double();
    max_speed_ = get_parameter("max_speed").as_double();
    config_.cte_lookahead_weight = get_parameter("cte_lookahead_weight").as_double();
    config_.cte_lookahead_gain = get_parameter("cte_lookahead_gain").as_double();
    config_.curvature_lookahead_gain = get_parameter("curvature_lookahead_gain").as_double();
    config_.curvature_speed_factor = get_parameter("curvature_speed_factor").as_double();
    config_.curvature_speed_floor_ratio = std::clamp(
        get_parameter("curvature_speed_floor_ratio").as_double(), 0.0, 1.0);
    
    config_.max_steering = get_parameter("max_steering").as_double();
    config_.wheelbase = get_parameter("wheelbase").as_double();
    
    publish_visualization_ = get_parameter("publish_visualization").as_bool();
    pose_topic_ = get_parameter("pose_topic").as_string();
    pose_timeout_s_ = get_parameter("pose_timeout_s").as_double();
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
            max_speed_ = param.as_double();
        } else if (param.get_name() == "cte_lookahead_weight") {
            config_.cte_lookahead_weight = param.as_double();
        } else if (param.get_name() == "cte_lookahead_gain") {
            config_.cte_lookahead_gain = param.as_double();
        } else if (param.get_name() == "curvature_lookahead_gain") {
            config_.curvature_lookahead_gain = param.as_double();
        } else if (param.get_name() == "curvature_speed_factor") {
            config_.curvature_speed_factor = param.as_double();
        } else if (param.get_name() == "curvature_speed_floor_ratio") {
            config_.curvature_speed_floor_ratio = std::clamp(param.as_double(), 0.0, 1.0);
        } else if (param.get_name() == "max_steering") {
            config_.max_steering = param.as_double();
        } else if (param.get_name() == "pose_timeout_s") {
            pose_timeout_s_ = param.as_double();
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
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state_.velocity = msg->twist.twist.linear.x;
        current_state_.angular_velocity = msg->twist.twist.angular.z;
    }
}

void PurePursuitNode::poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state_.pose.x = msg->pose.pose.position.x;
        current_state_.pose.y = msg->pose.pose.position.y;
        const double qx = msg->pose.pose.orientation.x;
        const double qy = msg->pose.pose.orientation.y;
        const double qz = msg->pose.pose.orientation.z;
        const double qw = msg->pose.pose.orientation.w;
        current_state_.pose.theta = std::atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz));
        pose_received_ = true;
        last_pose_time_ = now();
    }

    // Run control on every pose update (typically /ekf_pose).
    controlLoop();
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

    if (!pose_received_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "No pose received on %s yet", pose_topic_.c_str());
        publishDriveCommand(0.0, 0.0);
        return;
    }

    const double pose_age = (now() - last_pose_time_).seconds();
    if (pose_age > pose_timeout_s_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Pose timeout %.3fs > %.3fs; issuing stop for fail-safe",
                             pose_age, pose_timeout_s_);
        publishDriveCommand(0.0, 0.0);
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
        // Soft start: cap speed to 1.0 m/s for the first 2 seconds
        if (!soft_start_initialized_) {
            soft_start_time_ = now();
            soft_start_initialized_ = true;
            RCLCPP_INFO(get_logger(), "Soft start: capping speed to 1.0 m/s for 2 seconds");
        }
        double elapsed = (now() - soft_start_time_).seconds();
        if (elapsed < 2.0) {
            output.target_speed = std::min(output.target_speed, 1.0);
        }

        output.target_speed = std::clamp(output.target_speed, 0.0, max_speed_);
        
        publishDriveCommand(output.steering_angle, output.target_speed);
        
        if (publish_visualization_) {
            publishLookaheadMarker(output);
        }
    } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                            "Invalid Pure Pursuit output");
        publishDriveCommand(0.0, 0.0);
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

void PurePursuitNode::publishLookaheadMarker(const PurePursuitOutput& output) {
    visualization_msgs::msg::MarkerArray markers;
    
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
    
    viz_pub_->publish(markers);
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
