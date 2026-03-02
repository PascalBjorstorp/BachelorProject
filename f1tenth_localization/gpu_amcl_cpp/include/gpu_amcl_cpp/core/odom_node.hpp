#pragma once

#include "gpu_amcl_cpp/helpers/math_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <mutex>

namespace gpu_amcl_cpp {

/**
 * @brief Odometry relay node — converts wheel odometry (which already
 *        includes VESC-fused IMU heading) to PoseWithCovarianceStamped
 *        with configurable covariance.
 *
 * Event-driven: publishes one PoseWithCovarianceStamped for every
 * incoming odom message.  Output is consumed by the EKF node.
 */
class OdomNode : public rclcpp::Node {
public:
    explicit OdomNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // ── Callbacks ──────────────────────────────────────────────────
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // ── Helpers ────────────────────────────────────────────────────
    void declare_all_parameters();
    void load_parameters();
    void publish_pose(const rclcpp::Time& stamp);

    // ── ROS I/O ────────────────────────────────────────────────────
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr  odom_sub_;
    rclcpp::Publisher<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;

    // ── State ──────────────────────────────────────────────────────
    bool   odom_init_ = false;
    double odom_x_    = 0.0, odom_y_    = 0.0, odom_theta_ = 0.0;

    // Parameters
    double base_cov_xy_    = 0.01;
    double base_cov_theta_ = 0.02;

    std::string odom_topic_;
    std::string output_topic_;
    std::string frame_id_;
    std::string child_frame_id_;

    std::mutex state_mutex_;
};

}  // namespace gpu_amcl_cpp
