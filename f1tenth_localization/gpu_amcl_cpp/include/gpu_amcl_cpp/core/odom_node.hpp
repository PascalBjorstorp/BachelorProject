#pragma once

#include "gpu_amcl_cpp/helpers/math_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <Eigen/Core>
#include <mutex>
#include <deque>

namespace gpu_amcl_cpp {

/**
 * @brief Odometry node — fuses wheel odometry with IMU for a
 *        high-rate, drift-corrected position estimate.
 *
 * Publishes at a configurable rate (target ≈ 200 Hz).
 * Output is consumed by both AMCL (for motion deltas) and the EKF
 * (as prediction source).
 *
 * Drift detection: compares IMU-measured yaw rate against odom-derived
 * yaw rate.  When they diverge beyond a threshold the covariance is
 * inflated, signalling the EKF to trust AMCL more.
 */
class OdomNode : public rclcpp::Node {
public:
    explicit OdomNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // ── Callbacks ──────────────────────────────────────────────────
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    // ── Helpers ────────────────────────────────────────────────────
    void declare_all_parameters();
    void load_parameters();
    void publish_fused_pose(const rclcpp::Time& stamp);

    // ── ROS I/O ────────────────────────────────────────────────────
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr  odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr    imu_sub_;
    rclcpp::Publisher<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;

    // ── State ──────────────────────────────────────────────────────
    // Latest odom
    bool   odom_init_ = false;
    double odom_x_    = 0.0, odom_y_    = 0.0, odom_theta_ = 0.0;
    rclcpp::Time odom_stamp_;

    // IMU integration
    bool   imu_init_ = false;
    double imu_yaw_  = 0.0;
    double imu_gyro_z_ = 0.0;   ///< latest angular velocity.
    rclcpp::Time imu_stamp_;

    // Fused pose
    Eigen::Vector3d fused_pose_ = Eigen::Vector3d::Zero();  ///< (x, y, θ)

    // Drift detection (sliding window of yaw-rate discrepancy)
    std::deque<double> drift_history_;
    double drift_confidence_ = 0.0;  ///< 0 = no drift, 1 = full drift.

    // Parameters
    double imu_weight_             = 0.8;  ///< blend of IMU vs odom yaw
    double publish_rate_           = 200.0;
    double drift_threshold_        = 0.15;  ///< rad/s discrepancy
    int    drift_window_           = 10;
    double base_cov_xy_            = 0.01;
    double base_cov_theta_         = 0.02;
    double drift_cov_multiplier_   = 5.0;

    std::string odom_topic_;
    std::string imu_topic_;
    std::string output_topic_;
    std::string frame_id_;
    std::string child_frame_id_;

    std::mutex state_mutex_;
};

}  // namespace gpu_amcl_cpp
