// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#ifndef VESC_ACKERMANN__VESC_TO_ODOM_HPP_
#define VESC_ACKERMANN__VESC_TO_ODOM_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

namespace vesc_ackermann
{

using nav_msgs::msg::Odometry;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;

class VescToOdom : public rclcpp::Node
{
public:
  explicit VescToOdom(const rclcpp::NodeOptions & options);

private:
  // ROS parameters
  std::string odom_frame_;
  std::string base_frame_;
  /** State message does not report servo position, so use the command instead */
  bool use_servo_cmd_;
  bool use_imu_;
  // conversion gain and offset
  double speed_to_erpm_gain_, speed_to_erpm_offset_;
  double steering_to_servo_gain_, steering_to_servo_offset_;
  double steering_correction_c2_, steering_correction_c1_, steering_correction_c0_;
  double wheelbase_;
  bool publish_tf_;
  std::string integration_method_;  ///< Integration method: "euler", "trapezoidal", "analytical"

  // Odometry covariance parameters
  double odom_x_covariance_;    ///< x position covariance
  double odom_y_covariance_;    ///< y position covariance
  double odom_yaw_covariance_;  ///< yaw covariance

  // odometry state
  double x_, y_, yaw_;
  Float64::SharedPtr last_servo_cmd_;  ///< Last servo position commanded value
  VescStateStamped::SharedPtr last_state_;  ///< Last received state message
  sensor_msgs::msg::Imu::SharedPtr last_imu_;  ///< Last received IMU message

  // IMU yaw alignment (yaw = imu_yaw + imu_yaw_offset_)
  double imu_yaw_offset_;  ///< Offset to keep IMU yaw aligned with odom yaw state
  bool imu_initialized_;   ///< Flag to check if IMU has been initialized

  // Adaptive IMU takeover (for lateral slip/glide handling)
  bool adaptive_imu_takeover_;         ///< If true, switch to IMU yaw only during detected slip
  double slip_min_speed_;              ///< Minimum speed (m/s) before slip detection is active
  double slip_lateral_accel_threshold_;  ///< |a_y| threshold (m/s^2) for slip detection
  double slip_yaw_rate_diff_threshold_;  ///< |omega_imu - omega_model| threshold (rad/s)
  double slip_hysteresis_factor_;      ///< Exit threshold scale in (0, 1)
  bool imu_takeover_active_;           ///< Current adaptive takeover state
  std::string takeover_reason_;        ///< Reason label for current/last takeover state
  bool using_imu_last_step_;           ///< Whether previous callback used IMU yaw path

  // Optional IMU lateral-velocity correction during slip takeover
  bool use_imu_lateral_velocity_;    ///< If true, estimate body-frame v_y from IMU during slip
  double imu_lateral_accel_alpha_;   ///< Low-pass alpha for IMU lateral acceleration
  double imu_lateral_velocity_decay_;  ///< Exponential decay rate for v_y estimate (1/s)
  double imu_lateral_velocity_max_;  ///< Saturation limit for estimated body-frame v_y (m/s)

  double filtered_lateral_accel_;      ///< Filtered IMU lateral acceleration (m/s^2)
  bool lateral_accel_filter_initialized_;  ///< Filter init flag
  double imu_lateral_velocity_;        ///< Estimated body-frame lateral velocity (m/s)

  // Low-pass filter for IMU angular velocity
  double imu_angular_velocity_alpha_;  ///< Low-pass filter coefficient (0-1)
  double filtered_angular_velocity_;   ///< Filtered angular velocity value
  bool angular_velocity_filter_initialized_;  ///< Flag to check if filter has been initialized

  // ROS services
  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<Float64>::SharedPtr filtered_angular_velocity_pub_;
  rclcpp::Subscription<VescStateStamped>::SharedPtr vesc_state_sub_;
  rclcpp::Subscription<Float64>::SharedPtr servo_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub_;

  // ROS callbacks
  void vescStateCallback(const VescStateStamped::SharedPtr state);
  void servoCmdCallback(const Float64::SharedPtr servo);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu);
};

}  // namespace vesc_ackermann

#endif  // VESC_ACKERMANN__VESC_TO_ODOM_HPP_
