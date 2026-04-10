#include "gpu_amcl_cpp/core/odom_node.hpp"
#include "gpu_amcl_cpp/helpers/math_utils.hpp"

#include <cmath>

namespace gpu_amcl_cpp {

OdomNode::OdomNode(const rclcpp::NodeOptions& options)
    : Node("odom_fused", options) { // Node name = "odom_fused"

    declare_all_parameters();       // Register all ROS params
    load_parameters();              // Read values into member vars

    // Publisher: outputs PoseWithCovarianceStamped
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        output_topic_, rclcpp::QoS(10));       // From param "output_topic" → "/odom_pose" and Queue depth of 10

    // Subscriber: receives wheel odom
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS(),
        std::bind(&OdomNode::odom_callback, this, std::placeholders::_1));  // From param "odom_topic" → "/ego_racecar/odom"
                                                                            // Best-effort, keep only latest
    // Log startup info
    RCLCPP_INFO(get_logger(),
                "Odom relay node started — '%s' -> '%s' (cov: xy=%.5f, theta=%.5f)",
                odom_topic_.c_str(), output_topic_.c_str(),
                base_cov_xy_, base_cov_theta_);
}

// ─── Parameters ─────────────────────────────────────────────────────
void OdomNode::declare_all_parameters() {
    declare_parameter<std::string>("odom_topic", "/ego_racecar/odom");
    declare_parameter<std::string>("output_topic", "/odom_pose");
    declare_parameter<std::string>("frame_id", "ego_racecar/odom");

    declare_parameter<double>("base_cov_xy", base_cov_xy_);
    declare_parameter<double>("base_cov_theta", base_cov_theta_);
}

void OdomNode::load_parameters() {
    odom_topic_     = get_parameter("odom_topic").as_string();
    output_topic_   = get_parameter("output_topic").as_string();
    frame_id_       = get_parameter("frame_id").as_string();
    base_cov_xy_    = get_parameter("base_cov_xy").as_double();
    base_cov_theta_ = get_parameter("base_cov_theta").as_double();
}

// ─── Odom callback ──────────────────────────────────────────────────
void OdomNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Extract x, y from Odometry message
    odom_x_     = msg->pose.pose.position.x;
    odom_y_     = msg->pose.pose.position.y;
    // Convert quaternion to yaw using helper function
    odom_theta_ = math_utils::quaternion_to_yaw(msg->pose.pose.orientation);

    // Immediately publish — preserves original timestamp
    publish_pose(rclcpp::Time(msg->header.stamp));
}

// ─── Publish pose ───────────────────────────────────────────────────
void OdomNode::publish_pose(const rclcpp::Time& stamp) {
    auto out = geometry_msgs::msg::PoseWithCovarianceStamped();
    
    // Header
    out.header.stamp    = stamp;        // Preserve original odom timestamp
    out.header.frame_id = frame_id_;    // From param → "ego_racecar/odom"
    // Pose (position + orientation)
    out.pose.pose.position.x = odom_x_;
    out.pose.pose.position.y = odom_y_;
    out.pose.pose.position.z = 0.0;     // Always 0 (planar robot)
    out.pose.pose.orientation = math_utils::yaw_to_quaternion(odom_theta_);

    auto& cov = out.pose.covariance;
    std::fill(cov.begin(), cov.end(), 0.0); // Zero everything first

    cov[0]  = base_cov_xy_;                 // xx variance (meters²)
    cov[7]  = base_cov_xy_;                 // yy variance (meters²)
    cov[35] = base_cov_theta_;              // yaw-yaw variance (radians²)

    pose_pub_->publish(out);
}

}  // namespace gpu_amcl_cpp
