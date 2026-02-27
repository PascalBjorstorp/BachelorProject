#include "gpu_amcl_cpp/core/odom_node.hpp"
#include "gpu_amcl_cpp/helpers/math_utils.hpp"

#include <chrono>
#include <cmath>
#include <numeric>

using namespace std::chrono_literals;

namespace gpu_amcl_cpp {

OdomNode::OdomNode(const rclcpp::NodeOptions& options)
    : Node("odom_fused", options) {
    declare_all_parameters();
    load_parameters();

    // ── Publisher ──────────────────────────────────────────────────
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        output_topic_, rclcpp::QoS(10));

    // ── Subscribers ───────────────────────────────────────────────
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS(),
        std::bind(&OdomNode::odom_callback, this, std::placeholders::_1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::SensorDataQoS(),
        std::bind(&OdomNode::imu_callback, this, std::placeholders::_1));

    // No publish timer — event-driven publishing from odom_callback (§10.2).

    RCLCPP_INFO(get_logger(),
                "Odom fusion node started — event-driven publishing on '%s'",
                output_topic_.c_str());
}

// ─── Parameters ─────────────────────────────────────────────────────
void OdomNode::declare_all_parameters() {
    declare_parameter<std::string>("odom_topic", "/ego_racecar/odom");
    declare_parameter<std::string>("imu_topic", "/sensors/imu/raw");
    declare_parameter<std::string>("output_topic", "/odom_pose");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<std::string>("child_frame_id", "ego_racecar/base_link");

    declare_parameter<double>("publish_rate", 200.0);
    declare_parameter<double>("imu_weight", 0.8);
    declare_parameter<double>("drift_threshold", 0.15);
    declare_parameter<int>("drift_window", 10);
    declare_parameter<double>("base_cov_xy", 0.01);
    declare_parameter<double>("base_cov_theta", 0.02);
    declare_parameter<double>("drift_cov_multiplier", 5.0);
}

void OdomNode::load_parameters() {
    odom_topic_           = get_parameter("odom_topic").as_string();
    imu_topic_            = get_parameter("imu_topic").as_string();
    output_topic_         = get_parameter("output_topic").as_string();
    frame_id_             = get_parameter("frame_id").as_string();
    child_frame_id_       = get_parameter("child_frame_id").as_string();
    publish_rate_         = get_parameter("publish_rate").as_double();
    imu_weight_           = get_parameter("imu_weight").as_double();
    drift_threshold_      = get_parameter("drift_threshold").as_double();
    drift_window_         = get_parameter("drift_window").as_int();
    base_cov_xy_          = get_parameter("base_cov_xy").as_double();
    base_cov_theta_       = get_parameter("base_cov_theta").as_double();
    drift_cov_multiplier_ = get_parameter("drift_cov_multiplier").as_double();
}

// ─── Odom callback ──────────────────────────────────────────────────
void OdomNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    double x     = msg->pose.pose.position.x;
    double y     = msg->pose.pose.position.y;
    double theta = math_utils::quaternion_to_yaw(msg->pose.pose.orientation);

    if (!odom_init_) {
        odom_x_     = x;
        odom_y_     = y;
        odom_theta_ = theta;
        odom_stamp_ = rclcpp::Time(msg->header.stamp);
        fused_pose_ = {x, y, theta};
        odom_init_  = true;
        return;
    }

    double dt = (rclcpp::Time(msg->header.stamp) - odom_stamp_).seconds();
    if (dt <= 0.0) return;

    // Compute odom-derived yaw rate.
    double dtheta_odom = math_utils::angle_diff(theta, odom_theta_);
    double odom_yaw_rate = dtheta_odom / dt;

    // ── IMU fusion for yaw ────────────────────────────────────────
    // Strategy: trust odom by default; when drift (slip) is detected,
    // shift trust toward IMU.  imu_weight_ is the *maximum* IMU weight
    // applied at full drift confidence.
    double fused_dtheta;
    if (imu_init_) {
        // ── Drift detection ───────────────────────────────────────
        double disc = std::abs(imu_gyro_z_ - odom_yaw_rate);
        drift_history_.push_back(disc);
        if (static_cast<int>(drift_history_.size()) > drift_window_) {
            drift_history_.pop_front();
        }
        double mean_disc = std::accumulate(
            drift_history_.begin(), drift_history_.end(), 0.0)
            / drift_history_.size();
        drift_confidence_ = std::clamp(
            mean_disc / drift_threshold_, 0.0, 1.0);

        // Adaptive blending: no drift → pure odom, full drift → max IMU
        double adaptive_imu_weight = drift_confidence_ * imu_weight_;
        double imu_dtheta = imu_gyro_z_ * dt;
        fused_dtheta = adaptive_imu_weight * imu_dtheta
                     + (1.0 - adaptive_imu_weight) * dtheta_odom;
    } else {
        fused_dtheta = dtheta_odom;
    }

    // ── Position update (use odom x/y + fused yaw) ───────────────
    double dx = x - odom_x_;
    double dy = y - odom_y_;

    // Use odom translation magnitude but fused heading.
    double trans = std::sqrt(dx * dx + dy * dy);
    double bearing = std::atan2(dy, dx);
    double rel_bearing = math_utils::angle_diff(bearing, odom_theta_);

    fused_pose_[0] += trans * std::cos(fused_pose_[2] + rel_bearing);
    fused_pose_[1] += trans * std::sin(fused_pose_[2] + rel_bearing);
    fused_pose_[2]  = math_utils::normalize_angle(
        fused_pose_[2] + fused_dtheta);

    odom_x_     = x;
    odom_y_     = y;
    odom_theta_ = theta;
    odom_stamp_ = rclcpp::Time(msg->header.stamp);

    // Publish immediately with original odom timestamp (§10.2 + §10.6).
    publish_fused_pose(rclcpp::Time(msg->header.stamp));
}

// ─── IMU callback ───────────────────────────────────────────────────
void OdomNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    imu_gyro_z_ = msg->angular_velocity.z;
    imu_stamp_  = rclcpp::Time(msg->header.stamp);

    if (!imu_init_) {
        imu_init_ = true;
        RCLCPP_INFO(get_logger(), "IMU data received — fusion enabled.");
    }
}

// ─── Publish fused pose (called from odom_callback, which holds state_mutex_) ──
void OdomNode::publish_fused_pose(const rclcpp::Time& stamp) {
    if (!odom_init_) return;

    // Caller already holds state_mutex_ — read directly.
    Eigen::Vector3d pose = fused_pose_;
    double drift_conf    = drift_confidence_;

    auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    msg.header.stamp    = stamp;   // original odom timestamp (§10.6)
    msg.header.frame_id = frame_id_;

    msg.pose.pose.position.x = pose[0];
    msg.pose.pose.position.y = pose[1];
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation = math_utils::yaw_to_quaternion(pose[2]);

    // Covariance: inflate when drift is detected.
    double scale = 1.0 + drift_conf * (drift_cov_multiplier_ - 1.0);
    auto& cov = msg.pose.covariance;
    std::fill(cov.begin(), cov.end(), 0.0);
    cov[0]  = base_cov_xy_    * scale;  // xx
    cov[7]  = base_cov_xy_    * scale;  // yy
    cov[35] = base_cov_theta_ * scale;  // yaw-yaw

    pose_pub_->publish(msg);
}

}  // namespace gpu_amcl_cpp
