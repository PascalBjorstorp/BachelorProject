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

#include <cstddef>
#include <deque>
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
  static double normalizeAngle(double angle);

  // Frames and conversion
  std::string odom_frame_{"ego_racecar/odom"};
  std::string base_frame_{"ego_racecar/base_link"};
  double speed_to_erpm_gain_{0.0};
  double speed_to_erpm_offset_{0.0};
  double speed_deadband_{0.15};
  double max_dt_sec_{1.0};

  // Steering model parameters
  double steering_to_servo_gain_{0.0};
  double steering_to_servo_offset_{0.0};
  double steering_correction_c2_{0.0};
  double steering_correction_c1_{1.0};
  double steering_correction_c0_{0.0};
  double wheelbase_{0.33};

  // Dynamic bicycle model parameters
  bool use_dynamic_bicycle_model_{true};
  double vehicle_mass_{3.314};
  double vehicle_Iz_{0.035};
  double l_f_{0.166};
  double l_r_{0.16};
  double c_alpha_f_{51.4};
  double c_alpha_r_{43.1};
  double pacejka_shape_factor_{1.9};
  double dynamic_model_min_speed_{1.0};

  // 2D pose covariance (x/y/yaw)
  double odom_x_covariance_{0.2};
  double odom_y_covariance_{0.2};
  double odom_yaw_covariance_{0.2};

  // Slip-aware covariance scaling for x/y
  double slip_xy_covariance_scale_{6.0};

  // Slip detection thresholds (m/s^2)
  double slip_accel_enter_{1.8};
  double slip_accel_exit_{1.0};
  std::string slip_indicator_source_{"yaw_residual"};
  double slip_lateral_velocity_enter_{0.7};
  double slip_lateral_velocity_exit_{0.35};
  bool slip_active_{false};
  double slip_min_speed_{1.2};

  // Slip indicator filtering and dwell hysteresis
  double slip_indicator_alpha_{0.2};
  double filtered_slip_indicator_{0.0};
  bool slip_indicator_initialized_{false};
  bool slip_use_lateral_accel_{false};
  double slip_accel_clip_{6.0};
  double slip_yaw_rate_weight_{3.0};
  double slip_enter_hold_sec_{0.10};
  double slip_exit_hold_sec_{0.20};
  double slip_enter_timer_{0.0};
  double slip_exit_timer_{0.0};

  // IMU filtering
  double imu_angular_velocity_alpha_{0.45};  // fallback EMA if Butterworth is disabled
  bool imu_use_butterworth_filter_{true};
  double imu_butterworth_gyro_cutoff_hz_{18.0};
  double imu_butterworth_lateral_accel_cutoff_hz_{12.0};
  double imu_yaw_base_weight_{0.8};
  double filtered_angular_velocity_{0.0};
  double filtered_linear_accel_x_{0.0};
  double filtered_linear_accel_y_{0.0};
  double debiased_angular_velocity_z_raw_{0.0};
  double debiased_linear_accel_x_raw_{0.0};
  double debiased_linear_accel_y_raw_{0.0};
  bool angular_velocity_filter_initialized_{false};
  bool imu_sample_time_initialized_{false};
  double last_imu_sample_time_sec_{0.0};

  // 2nd-order filter states (Direct Form I)
  bool gyro_biquad_initialized_{false};
  double gyro_biquad_x1_{0.0};
  double gyro_biquad_x2_{0.0};
  double gyro_biquad_y1_{0.0};
  double gyro_biquad_y2_{0.0};
  bool lateral_accel_y_biquad_initialized_{false};
  double lateral_accel_y_biquad_x1_{0.0};
  double lateral_accel_y_biquad_x2_{0.0};
  double lateral_accel_y_biquad_y1_{0.0};
  double lateral_accel_y_biquad_y2_{0.0};
  bool longitudinal_accel_x_biquad_initialized_{false};
  double longitudinal_accel_x_biquad_x1_{0.0};
  double longitudinal_accel_x_biquad_x2_{0.0};
  double longitudinal_accel_x_biquad_y1_{0.0};
  double longitudinal_accel_x_biquad_y2_{0.0};

  // Gyro bias adaptation
  double gyro_bias_{0.0};
  double gyro_bias_alpha_{0.02};

  // Startup IMU bias calibration (use only linear x/y and angular z)
  bool imu_startup_calibration_enabled_{true};
  double imu_startup_calibration_duration_sec_{5.0};
  bool imu_startup_hold_odom_during_calibration_{false};
  bool imu_startup_calibration_started_{false};
  bool imu_startup_calibration_done_{false};
  double imu_startup_calibration_start_sec_{0.0};
  std::size_t imu_startup_calibration_sample_count_{0};
  double imu_startup_linear_accel_x_sum_{0.0};
  double imu_startup_linear_accel_y_sum_{0.0};
  double imu_startup_angular_velocity_z_sum_{0.0};
  double imu_startup_linear_accel_x_bias_{0.0};
  double imu_startup_linear_accel_y_bias_{0.0};
  double imu_startup_angular_velocity_z_bias_{0.0};

  // Lateral-velocity estimation from IMU
  double imu_lateral_accel_alpha_{0.35};
  double filtered_lateral_accel_{0.0};
  bool lateral_accel_filter_initialized_{false};
  double imu_lateral_velocity_{0.0};
  double imu_lateral_velocity_decay_{1.5};
  double imu_lateral_velocity_max_{3.0};

  // Slip-angle handling
  double beta_max_rad_{0.8};
  double kinematic_beta_ratio_{0.5};

  // State
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  double yaw_rate_state_{0.0};
  VescStateStamped::SharedPtr last_state_;
  sensor_msgs::msg::Imu::SharedPtr last_imu_;
  Float64::SharedPtr last_servo_cmd_;

  struct ImuSyncSample
  {
    rclcpp::Time stamp;
    double filtered_angular_velocity{0.0};
    double filtered_linear_accel_x{0.0};
    double filtered_linear_accel_y{0.0};
    double raw_angular_velocity{0.0};
    double raw_linear_accel_x{0.0};
    double raw_linear_accel_y{0.0};
  };
  std::deque<ImuSyncSample> imu_history_;
  int imu_history_max_samples_{400};
  double imu_sync_tolerance_sec_{0.03};
  double imu_timeout_sec_{0.10};
  double servo_timeout_sec_{0.10};

  // Last receive timestamps (node clock) for stale-signal detection
  rclcpp::Time last_imu_receive_time_;
  rclcpp::Time last_servo_receive_time_;
  bool imu_receive_time_initialized_{false};
  bool servo_receive_time_initialized_{false};

  // ROS I/O
  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<Float64>::SharedPtr filtered_angular_velocity_pub_;
  rclcpp::Subscription<VescStateStamped>::SharedPtr vesc_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<Float64>::SharedPtr servo_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub_;

  // Callbacks
  void vescStateCallback(const VescStateStamped::SharedPtr state);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu);
  void servoCmdCallback(const Float64::SharedPtr servo);
};

}  // namespace vesc_ackermann

#endif  // VESC_ACKERMANN__VESC_TO_ODOM_HPP_
