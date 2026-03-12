#include "f1tenth_control/nodes/pure_pursuit_node.hpp"
#include <tf2/utils.h>
#include <tf2/exceptions.h>
#include <cmath>

namespace f1tenth_control {

PurePursuitNode::PurePursuitNode(const rclcpp::NodeOptions& options)
    : Node("pure_pursuit_node", options)
{
    RCLCPP_INFO(get_logger(), "Initializing Pure Pursuit Node");
    
    // Declare and load parameters
    declareParameters();
    loadParameters();

    // TF listener for map-frame pose
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
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
    
    // Vehicle
    declare_parameter("wheelbase", 0.3302);
    
    // Misc
    declare_parameter("publish_visualization", true);
    declare_parameter("map_frame",  std::string("map"));
    declare_parameter("base_frame", std::string("ego_racecar/base_link"));
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
    config_.wheelbase = get_parameter("wheelbase").as_double();
    
    publish_visualization_ = get_parameter("publish_visualization").as_bool();
    map_frame_  = get_parameter("map_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
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
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state_.velocity = msg->twist.twist.linear.x;
        current_state_.angular_velocity = msg->twist.twist.angular.z;
    }
    
    // Run control on every odom update
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
    
    // Update pose from TF (map frame)
    updatePoseFromTF();

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
        
        publishDriveCommand(output.steering_angle, output.target_speed);
        
        if (publish_visualization_) {
            publishLookaheadMarker(output);
        }
    } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                            "Invalid Pure Pursuit output");
    }
}

bool PurePursuitNode::updatePoseFromTF() {
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_->lookupTransform(
            map_frame_, base_frame_,
            tf2::TimePointZero,
            tf2::durationFromSec(0.02));
    } catch (const tf2::TransformException & ex) {
        RCLCPP_DEBUG(get_logger(), "TF lookup failed: %s", ex.what());
        return false;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    current_state_.pose.x = tf.transform.translation.x;
    current_state_.pose.y = tf.transform.translation.y;
    current_state_.pose.theta = tf2::getYaw(
        tf2::Quaternion(
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w));
    return true;
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
