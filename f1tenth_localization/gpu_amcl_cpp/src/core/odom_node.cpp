#include "gpu_amcl_cpp/core/odom_node.hpp"
#include "gpu_amcl_cpp/helpers/math_utils.hpp"

#include <cmath>

namespace gpu_amcl_cpp {

OdomNode::OdomNode(const rclcpp::NodeOptions& options)
    : Node("odom_fused", options) {
    declare_all_parameters();
    load_parameters();

    // ── Publisher ──────────────────────────────────────────────────
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        output_topic_, rclcpp::QoS(10));

    // ── Subscriber (odom already contains VESC-fused IMU heading) ─
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS(),
        std::bind(&OdomNode::odom_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "Odom relay node started — '%s' -> '%s' (cov: xy=%.5f, theta=%.5f)",
                odom_topic_.c_str(), output_topic_.c_str(),
                base_cov_xy_, base_cov_theta_);
}

// ─── Parameters ─────────────────────────────────────────────────────
void OdomNode::declare_all_parameters() {
    declare_parameter<std::string>("odom_topic", "/ego_racecar/odom");
    declare_parameter<std::string>("output_topic", "/odom_pose");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<std::string>("child_frame_id", "ego_racecar/base_link");

    declare_parameter<double>("base_cov_xy", 0.01);
    declare_parameter<double>("base_cov_theta", 0.02);
}

void OdomNode::load_parameters() {
    odom_topic_     = get_parameter("odom_topic").as_string();
    output_topic_   = get_parameter("output_topic").as_string();
    frame_id_       = get_parameter("frame_id").as_string();
    child_frame_id_ = get_parameter("child_frame_id").as_string();
    base_cov_xy_    = get_parameter("base_cov_xy").as_double();
    base_cov_theta_ = get_parameter("base_cov_theta").as_double();
}

// ─── Odom callback ──────────────────────────────────────────────────
void OdomNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    odom_x_     = msg->pose.pose.position.x;
    odom_y_     = msg->pose.pose.position.y;
    odom_theta_ = math_utils::quaternion_to_yaw(msg->pose.pose.orientation);
    odom_init_  = true;

    // Publish as PoseWithCovarianceStamped with configurable covariance.
    publish_pose(rclcpp::Time(msg->header.stamp));
}

// ─── Publish pose ───────────────────────────────────────────────────
void OdomNode::publish_pose(const rclcpp::Time& stamp) {
    auto out = geometry_msgs::msg::PoseWithCovarianceStamped();
    out.header.stamp    = stamp;
    out.header.frame_id = frame_id_;

    out.pose.pose.position.x = odom_x_;
    out.pose.pose.position.y = odom_y_;
    out.pose.pose.position.z = 0.0;
    out.pose.pose.orientation = math_utils::yaw_to_quaternion(odom_theta_);

    auto& cov = out.pose.covariance;
    std::fill(cov.begin(), cov.end(), 0.0);
    cov[0]  = base_cov_xy_;     // xx
    cov[7]  = base_cov_xy_;     // yy
    cov[35] = base_cov_theta_;  // yaw-yaw

    pose_pub_->publish(out);
}

}  // namespace gpu_amcl_cpp
