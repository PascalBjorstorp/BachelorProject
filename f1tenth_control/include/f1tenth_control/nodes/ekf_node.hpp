#ifndef F1TENTH_CONTROL_EKF_NODE_HPP_
#define F1TENTH_CONTROL_EKF_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "f1tenth_control/state_estimation/ekf.hpp"

namespace f1tenth_control {

/**
 * @brief ROS2 node for EKF-based state estimation
 * 
 * Subscribes to:
 *   - /odom (wheel odometry from VESC)
 *   - /imu (IMU data from VESC)
 *   - /mcl_pose (optional: pose from Monte Carlo Localization)
 * 
 * Publishes:
 *   - /ekf_odom (fused odometry)
 *   - TF: odom -> base_link
 */
class EKFNode : public rclcpp::Node {
public:
    explicit EKFNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
private:
    void declareParameters();
    void loadParameters();
    
    // Callbacks
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void mclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void predictCallback();
    
    // Publishing
    void publishOdometry();
    void publishTF();
    
    // EKF
    std::unique_ptr<ExtendedKalmanFilter> ekf_;
    ExtendedKalmanFilter::Config ekf_config_;
    
    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr mcl_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_pub_;
    rclcpp::TimerBase::SharedPtr predict_timer_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Configuration
    std::string odom_frame_{"odom"};
    std::string base_frame_{"base_link"};
    bool publish_tf_{true};
    double predict_rate_{200.0};  // Hz
    
    // State
    rclcpp::Time last_odom_time_;
    rclcpp::Time last_imu_time_;
    rclcpp::Time last_predict_time_;
    bool initialized_{false};
    
    // For velocity calculation from wheel odometry
    double last_odom_x_{0.0};
    double last_odom_y_{0.0};
    double last_odom_theta_{0.0};
};

}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_EKF_NODE_HPP_
