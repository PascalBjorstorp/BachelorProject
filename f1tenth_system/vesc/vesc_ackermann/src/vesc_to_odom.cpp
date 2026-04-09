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

#include "vesc_ackermann/vesc_to_odom.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

namespace vesc_ackermann
{

using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::Odometry;
using std::placeholders::_1;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;

namespace
{
constexpr double kEpsilon = 1e-9;
}  // namespace

VescToOdom::VescToOdom(const rclcpp::NodeOptions & options)
: Node("vesc_to_odom_node", options)
{
  // Frame and conversion parameters
  odom_frame_ = declare_parameter("odom_frame", odom_frame_);
  base_frame_ = declare_parameter("base_frame", base_frame_);
  speed_to_erpm_gain_ = declare_parameter("speed_to_erpm_gain", speed_to_erpm_gain_);
  speed_to_erpm_offset_ = declare_parameter("speed_to_erpm_offset", speed_to_erpm_offset_);
  speed_deadband_ = declare_parameter("speed_deadband", speed_deadband_);
  max_dt_sec_ = declare_parameter("max_dt_sec", max_dt_sec_);

  // Steering model parameters
  steering_to_servo_gain_ = declare_parameter("steering_angle_to_servo_gain", steering_to_servo_gain_);
  steering_to_servo_offset_ = declare_parameter("steering_angle_to_servo_offset", steering_to_servo_offset_);
  steering_correction_c2_ = declare_parameter("steering_correction_c2", steering_correction_c2_);
  steering_correction_c1_ = declare_parameter("steering_correction_c1", steering_correction_c1_);
  steering_correction_c0_ = declare_parameter("steering_correction_c0", steering_correction_c0_);
  wheelbase_ = declare_parameter("wheelbase", wheelbase_);

  // Base covariance parameters
  odom_x_covariance_ = declare_parameter("odom_x_covariance", odom_x_covariance_);
  odom_y_covariance_ = declare_parameter("odom_y_covariance", odom_y_covariance_);
  odom_yaw_covariance_ = declare_parameter("odom_yaw_covariance", odom_yaw_covariance_);

  // Slip-aware covariance inflation
  slip_xy_covariance_scale_ =
    declare_parameter("slip_xy_covariance_scale", slip_xy_covariance_scale_);
  slip_yaw_covariance_scale_ =
    declare_parameter("slip_yaw_covariance_scale", slip_yaw_covariance_scale_);

  // Slip detection thresholds
  slip_accel_enter_ = declare_parameter("slip_accel_enter", slip_accel_enter_);
  slip_accel_exit_ = declare_parameter("slip_accel_exit", slip_accel_exit_);

  // IMU filter + bias parameters
  imu_angular_velocity_alpha_ =
    declare_parameter("imu_angular_velocity_alpha", imu_angular_velocity_alpha_);
  gyro_bias_alpha_ = declare_parameter("gyro_bias_alpha", gyro_bias_alpha_);

  // Lateral velocity estimator parameters
  imu_lateral_accel_alpha_ =
    declare_parameter("imu_lateral_accel_alpha", imu_lateral_accel_alpha_);
  imu_lateral_velocity_decay_ =
    declare_parameter("imu_lateral_velocity_decay", imu_lateral_velocity_decay_);
  imu_lateral_velocity_max_ =
    declare_parameter("imu_lateral_velocity_max", imu_lateral_velocity_max_);

  // Slip-angle parameters
  beta_max_rad_ = declare_parameter("beta_max_rad", beta_max_rad_);
  kinematic_beta_ratio_ = declare_parameter("kinematic_beta_ratio", kinematic_beta_ratio_);

  if (slip_accel_enter_ <= slip_accel_exit_) {
    RCLCPP_WARN(
      get_logger(),
      "slip_accel_enter (%.3f) <= slip_accel_exit (%.3f). Adjusting exit threshold.",
      slip_accel_enter_, slip_accel_exit_);
    slip_accel_exit_ = 0.5 * slip_accel_enter_;
  }

  if (std::fabs(speed_to_erpm_gain_) < kEpsilon) {
    RCLCPP_WARN(
      get_logger(),
      "speed_to_erpm_gain is %.6f. Odometry cannot convert ERPM to speed until this is set.",
      speed_to_erpm_gain_);
  }

  odom_pub_ = create_publisher<Odometry>("ego_racecar/odom", 10);
  filtered_angular_velocity_pub_ = create_publisher<Float64>("imu/filtered_angular_velocity", 10);
  tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  vesc_state_sub_ = create_subscription<VescStateStamped>(
    "sensors/core", 10, std::bind(&VescToOdom::vescStateCallback, this, _1));
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "sensors/imu/raw", 10, std::bind(&VescToOdom::imuCallback, this, _1));
  servo_sub_ = create_subscription<Float64>(
    "sensors/servo_position_command", 10, std::bind(&VescToOdom::servoCmdCallback, this, _1));

  RCLCPP_INFO(
    get_logger(),
    "Slip-aware odom enabled: odom frame '%s', base frame '%s'.",
    odom_frame_.c_str(), base_frame_.c_str());
}

double VescToOdom::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

void VescToOdom::vescStateCallback(const VescStateStamped::SharedPtr state)
{
  if (!last_imu_) {
    RCLCPP_INFO_ONCE(get_logger(), "Waiting for IMU to compute odometry.");
    return;
  }

  if (!last_state_) {
    last_state_ = state;
    return;
  }

  const double dt_sec =
    (rclcpp::Time(state->header.stamp) - rclcpp::Time(last_state_->header.stamp)).seconds();
  if (dt_sec <= 0.0 || dt_sec > max_dt_sec_) {
    RCLCPP_WARN(
      get_logger(),
      "Skipping odom update due to invalid dt=%.6f (max_dt_sec=%.3f).",
      dt_sec, max_dt_sec_);
    last_state_ = state;
    return;
  }

  if (std::fabs(speed_to_erpm_gain_) < kEpsilon) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "speed_to_erpm_gain is zero. Cannot compute odometry speed.");
    last_state_ = state;
    return;
  }

  // Wheel speed from ERPM calibration.
  double current_speed = (state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
  if (std::fabs(current_speed) < speed_deadband_) {
    current_speed = 0.0;
  }

  // Steering-derived model yaw rate.
  const bool has_servo = static_cast<bool>(last_servo_cmd_);
  double steering_angle = 0.0;
  if (has_servo && std::fabs(steering_to_servo_gain_) > kEpsilon) {
    const double raw_steering =
      (last_servo_cmd_->data - steering_to_servo_offset_) / steering_to_servo_gain_;
    const double abs_raw = std::fabs(raw_steering);
    const double corrected_abs =
      steering_correction_c2_ * abs_raw * abs_raw +
      steering_correction_c1_ * abs_raw +
      steering_correction_c0_;
    steering_angle = std::copysign(corrected_abs, raw_steering);
  }

  double model_yaw_rate = 0.0;
  if (has_servo && std::fabs(wheelbase_) > kEpsilon) {
    model_yaw_rate = current_speed * std::tan(steering_angle) / wheelbase_;
  }

  // IMU yaw rate with online bias correction.
  const double imu_yaw_rate_raw = filtered_angular_velocity_;

  // Slip indicator compares measured and model lateral acceleration.
  const double lateral_accel_measured = last_imu_->linear_acceleration.y;
  const double lateral_accel_model = current_speed * model_yaw_rate;
  const double slip_indicator = std::fabs(lateral_accel_measured - lateral_accel_model);

  if (!slip_active_ && slip_indicator > slip_accel_enter_) {
    slip_active_ = true;
    RCLCPP_INFO(
      get_logger(),
      "Slip mode ON (indicator=%.3f, enter=%.3f).",
      slip_indicator, slip_accel_enter_);
  } else if (slip_active_ && slip_indicator < slip_accel_exit_) {
    slip_active_ = false;
    RCLCPP_INFO(
      get_logger(),
      "Slip mode OFF (indicator=%.3f, exit=%.3f).",
      slip_indicator, slip_accel_exit_);
  }

  double slip_weight = std::clamp(
    (slip_indicator - slip_accel_exit_) / (slip_accel_enter_ - slip_accel_exit_),
    0.0,
    1.0);
  if (!has_servo) {
    slip_weight = 1.0;
  }
  if (std::fabs(current_speed) < 0.3) {
    slip_weight = 0.0;
  }

  if (has_servo && slip_weight < 0.2 && std::fabs(current_speed) > 0.5) {
    const double yaw_rate_error = imu_yaw_rate_raw - model_yaw_rate;
    gyro_bias_ = (1.0 - gyro_bias_alpha_) * gyro_bias_ + gyro_bias_alpha_ * yaw_rate_error;
  }

  const double imu_yaw_rate = imu_yaw_rate_raw - gyro_bias_;
  const double current_yaw_rate =
    (1.0 - slip_weight) * model_yaw_rate + slip_weight * imu_yaw_rate;

  // Lateral velocity estimate (high-slip helper) from IMU lateral acceleration residual.
  const double lateral_accel_residual = lateral_accel_measured - current_speed * current_yaw_rate;
  if (!lateral_accel_filter_initialized_) {
    filtered_lateral_accel_ = lateral_accel_residual;
    lateral_accel_filter_initialized_ = true;
  } else {
    filtered_lateral_accel_ =
      imu_lateral_accel_alpha_ * lateral_accel_residual +
      (1.0 - imu_lateral_accel_alpha_) * filtered_lateral_accel_;
  }

  imu_lateral_velocity_ += filtered_lateral_accel_ * dt_sec;
  const double decay = std::max(0.0, 1.0 - imu_lateral_velocity_decay_ * dt_sec);
  imu_lateral_velocity_ *= decay;
  if (std::fabs(current_speed) < 0.2) {
    imu_lateral_velocity_ = 0.0;
  }
  imu_lateral_velocity_ = std::clamp(
    imu_lateral_velocity_, -imu_lateral_velocity_max_, imu_lateral_velocity_max_);

  // Blend kinematic and IMU slip-angle estimates.
  double beta_kinematic = 0.0;
  if (has_servo) {
    beta_kinematic = std::atan(kinematic_beta_ratio_ * std::tan(steering_angle));
  }
  const double beta_imu = std::atan2(imu_lateral_velocity_, std::max(0.4, std::fabs(current_speed)));
  const double beta = std::clamp(
    (1.0 - slip_weight) * beta_kinematic + slip_weight * beta_imu,
    -beta_max_rad_, beta_max_rad_);

  // Integrate pose in the velocity direction (yaw + beta).
  yaw_ = normalizeAngle(yaw_ + current_yaw_rate * dt_sec);
  const double heading = yaw_ + beta;
  x_ += current_speed * std::cos(heading) * dt_sec;
  y_ += current_speed * std::sin(heading) * dt_sec;

  last_state_ = state;

  auto odom = std::make_unique<Odometry>();
  odom->header.frame_id = odom_frame_;
  odom->header.stamp = state->header.stamp;
  odom->child_frame_id = base_frame_;

  odom->pose.pose.position.x = x_;
  odom->pose.pose.position.y = y_;
  odom->pose.pose.position.z = 0.0;
  odom->pose.pose.orientation.x = 0.0;
  odom->pose.pose.orientation.y = 0.0;
  odom->pose.pose.orientation.z = std::sin(yaw_ / 2.0);
  odom->pose.pose.orientation.w = std::cos(yaw_ / 2.0);

  const double xy_cov = odom_x_covariance_ *
    (1.0 + slip_weight * (slip_xy_covariance_scale_ - 1.0));
  const double yaw_cov = odom_yaw_covariance_ *
    (1.0 + slip_weight * (slip_yaw_covariance_scale_ - 1.0));

  odom->pose.covariance[0] = xy_cov;
  odom->pose.covariance[7] = odom_y_covariance_ *
    (1.0 + slip_weight * (slip_xy_covariance_scale_ - 1.0));
  odom->pose.covariance[35] = yaw_cov;

  odom->twist.twist.linear.x = current_speed;
  odom->twist.twist.linear.y = imu_lateral_velocity_;
  odom->twist.twist.angular.z = current_yaw_rate;

  TransformStamped tf;
  tf.header.frame_id = odom_frame_;
  tf.child_frame_id = base_frame_;
  tf.header.stamp = state->header.stamp;
  tf.transform.translation.x = x_;
  tf.transform.translation.y = y_;
  tf.transform.translation.z = 0.0;
  tf.transform.rotation = odom->pose.pose.orientation;

  if (rclcpp::ok()) {
    tf_pub_->sendTransform(tf);
    odom_pub_->publish(std::move(odom));
  }
}

void VescToOdom::imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu)
{
  last_imu_ = imu;

  const double raw_angular_velocity = imu->angular_velocity.z;
  if (!angular_velocity_filter_initialized_) {
    filtered_angular_velocity_ = raw_angular_velocity;
    angular_velocity_filter_initialized_ = true;
  } else {
    filtered_angular_velocity_ =
      imu_angular_velocity_alpha_ * raw_angular_velocity +
      (1.0 - imu_angular_velocity_alpha_) * filtered_angular_velocity_;
  }

  auto filtered_msg = std::make_unique<Float64>();
  filtered_msg->data = filtered_angular_velocity_;
  filtered_angular_velocity_pub_->publish(std::move(filtered_msg));
}

void VescToOdom::servoCmdCallback(const Float64::SharedPtr servo)
{
  last_servo_cmd_ = servo;
}

}  // namespace vesc_ackermann

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::VescToOdom)
