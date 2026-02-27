#pragma once

#include "gpu_amcl_cpp/helpers/math_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <mutex>

namespace gpu_amcl_cpp {

/**
 * @brief Extended Kalman Filter node — fuses AMCL and odom-based
 *        pose estimates into a single localisation output.
 *
 * Prediction uses the odom model; correction uses AMCL.
 * Broadcasts the map → odom TF.
 *
 * State vector: [x, y, θ]  (SE(2) pose in the map frame).
 */
class EkfNode : public rclcpp::Node {
public:
    explicit EkfNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // ── Callbacks ──────────────────────────────────────────────────
    void amcl_callback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void odom_callback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    // ── EKF steps ──────────────────────────────────────────────────
    /// Prediction from odom delta.  Updates state_ and P_.
    void predict(const Eigen::Vector3d& odom_delta,
                 const Eigen::Matrix3d& Q);

    /// Correction from AMCL measurement.  Updates state_ and P_.
    void correct(const Eigen::Vector3d& z,
                 const Eigen::Matrix3d& R);

    // ── Helpers ────────────────────────────────────────────────────
    void declare_all_parameters();
    void load_parameters();
    void broadcast_tf(const rclcpp::Time& stamp);
    void publish_and_broadcast();

    // ── ROS I/O ────────────────────────────────────────────────────
    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr odom_sub_;
    rclcpp::Publisher<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ── EKF state ──────────────────────────────────────────────────
    Eigen::Vector3d state_ = Eigen::Vector3d::Zero();   ///< [x, y, θ]
    Eigen::Matrix3d P_     = Eigen::Matrix3d::Identity(); ///< covariance

    bool initialized_ = false;

    // Previous odom for delta computation
    Eigen::Vector3d prev_odom_ = Eigen::Vector3d::Zero();
    bool odom_received_ = false;

    // ── Parameters ─────────────────────────────────────────────────
    double transform_tolerance_ = 0.1;

    // Process noise scaling (multiplied onto odom covariance)
    double process_noise_scale_ = 1.0;

    std::string amcl_topic_;
    std::string odom_topic_;
    std::string output_topic_;
    std::string global_frame_;
    std::string odom_frame_;
    std::string base_frame_;

    std::mutex state_mutex_;
};

}  // namespace gpu_amcl_cpp
