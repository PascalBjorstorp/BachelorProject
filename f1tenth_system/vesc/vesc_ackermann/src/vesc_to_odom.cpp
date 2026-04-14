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
#include <limits>
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

  // Dynamic bicycle model parameters
  use_dynamic_bicycle_model_ =
    declare_parameter("use_dynamic_bicycle_model", use_dynamic_bicycle_model_);
  vehicle_mass_ = declare_parameter("vehicle_mass", vehicle_mass_);
  vehicle_Iz_ = declare_parameter("vehicle_Iz", vehicle_Iz_);
  l_f_ = declare_parameter("l_f", l_f_);
  l_r_ = declare_parameter("l_r", l_r_);
  c_alpha_f_ = declare_parameter("c_alpha_f", c_alpha_f_);
  c_alpha_r_ = declare_parameter("c_alpha_r", c_alpha_r_);
  dynamic_model_min_speed_ =
    declare_parameter("dynamic_model_min_speed", dynamic_model_min_speed_);

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
  slip_indicator_source_ = declare_parameter("slip_indicator_source", slip_indicator_source_);
  slip_lateral_velocity_enter_ =
    declare_parameter("slip_lateral_velocity_enter", slip_lateral_velocity_enter_);
  slip_lateral_velocity_exit_ =
    declare_parameter("slip_lateral_velocity_exit", slip_lateral_velocity_exit_);
  slip_min_speed_ = declare_parameter("slip_min_speed", slip_min_speed_);
  slip_indicator_alpha_ = declare_parameter("slip_indicator_alpha", slip_indicator_alpha_);
  slip_use_lateral_accel_ =
    declare_parameter("slip_use_lateral_accel", slip_use_lateral_accel_);
  slip_accel_clip_ = declare_parameter("slip_accel_clip", slip_accel_clip_);
  slip_yaw_rate_weight_ =
    declare_parameter("slip_yaw_rate_weight", slip_yaw_rate_weight_);
  slip_enter_hold_sec_ = declare_parameter("slip_enter_hold_sec", slip_enter_hold_sec_);
  slip_exit_hold_sec_ = declare_parameter("slip_exit_hold_sec", slip_exit_hold_sec_);

  // IMU filter + bias parameters
  imu_angular_velocity_alpha_ =
    declare_parameter("imu_angular_velocity_alpha", imu_angular_velocity_alpha_);
  imu_use_butterworth_filter_ =
    declare_parameter("imu_use_butterworth_filter", imu_use_butterworth_filter_);
  imu_butterworth_gyro_cutoff_hz_ =
    declare_parameter("imu_butterworth_gyro_cutoff_hz", imu_butterworth_gyro_cutoff_hz_);
  imu_butterworth_lateral_accel_cutoff_hz_ =
    declare_parameter(
    "imu_butterworth_lateral_accel_cutoff_hz", imu_butterworth_lateral_accel_cutoff_hz_);
  imu_yaw_base_weight_ =
    declare_parameter("imu_yaw_base_weight", imu_yaw_base_weight_);
  gyro_bias_alpha_ = declare_parameter("gyro_bias_alpha", gyro_bias_alpha_);
  imu_startup_calibration_enabled_ =
    declare_parameter("imu_startup_calibration_enabled", imu_startup_calibration_enabled_);
  imu_startup_calibration_duration_sec_ =
    declare_parameter("imu_startup_calibration_duration_sec", imu_startup_calibration_duration_sec_);
  imu_startup_hold_odom_during_calibration_ =
    declare_parameter(
    "imu_startup_hold_odom_during_calibration",
    imu_startup_hold_odom_during_calibration_);
  imu_history_max_samples_ =
    declare_parameter("imu_history_max_samples", imu_history_max_samples_);
  imu_sync_tolerance_sec_ =
    declare_parameter("imu_sync_tolerance_sec", imu_sync_tolerance_sec_);
  imu_timeout_sec_ = declare_parameter("imu_timeout_sec", imu_timeout_sec_);
  servo_timeout_sec_ = declare_parameter("servo_timeout_sec", servo_timeout_sec_);

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

  if (
    slip_indicator_source_ != "yaw_residual" &&
    slip_indicator_source_ != "lateral_accel" &&
    slip_indicator_source_ != "lateral_velocity_error")
  {
    RCLCPP_WARN(
      get_logger(),
      "Invalid slip_indicator_source '%s'. Using 'yaw_residual'.",
      slip_indicator_source_.c_str());
    slip_indicator_source_ = "yaw_residual";
  }

  if (slip_lateral_velocity_enter_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "slip_lateral_velocity_enter %.3f <= 0. Using 0.7 m/s.",
      slip_lateral_velocity_enter_);
    slip_lateral_velocity_enter_ = 0.7;
  }

  if (slip_lateral_velocity_exit_ < 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "slip_lateral_velocity_exit %.3f < 0. Using 0.35 m/s.",
      slip_lateral_velocity_exit_);
    slip_lateral_velocity_exit_ = 0.35;
  }

  if (slip_lateral_velocity_enter_ <= slip_lateral_velocity_exit_) {
    RCLCPP_WARN(
      get_logger(),
      "slip_lateral_velocity_enter (%.3f) <= slip_lateral_velocity_exit (%.3f). Adjusting exit threshold.",
      slip_lateral_velocity_enter_, slip_lateral_velocity_exit_);
    slip_lateral_velocity_exit_ = 0.5 * slip_lateral_velocity_enter_;
  }

  if (slip_indicator_alpha_ < 0.0 || slip_indicator_alpha_ > 1.0) {
    RCLCPP_WARN(
      get_logger(),
      "slip_indicator_alpha %.3f out of [0,1]. Using 0.2.",
      slip_indicator_alpha_);
    slip_indicator_alpha_ = 0.2;
  }

  if (slip_accel_clip_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "slip_accel_clip %.3f <= 0. Using 6.0.",
      slip_accel_clip_);
    slip_accel_clip_ = 6.0;
  }

  if (slip_yaw_rate_weight_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "slip_yaw_rate_weight %.3f <= 0. Using 3.0.",
      slip_yaw_rate_weight_);
    slip_yaw_rate_weight_ = 3.0;
  }

  if (imu_yaw_base_weight_ < 0.0 || imu_yaw_base_weight_ > 1.0) {
    RCLCPP_WARN(
      get_logger(),
      "imu_yaw_base_weight %.3f out of [0,1]. Clamping.",
      imu_yaw_base_weight_);
    imu_yaw_base_weight_ = std::clamp(imu_yaw_base_weight_, 0.0, 1.0);
  }

  if (imu_history_max_samples_ < 5) {
    RCLCPP_WARN(
      get_logger(),
      "imu_history_max_samples %d is too small. Using 400.",
      imu_history_max_samples_);
    imu_history_max_samples_ = 400;
  }

  if (imu_sync_tolerance_sec_ < 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "imu_sync_tolerance_sec %.3f < 0. Using 0.03.",
      imu_sync_tolerance_sec_);
    imu_sync_tolerance_sec_ = 0.03;
  }

  if (imu_timeout_sec_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "imu_timeout_sec %.3f <= 0. Using 0.10.",
      imu_timeout_sec_);
    imu_timeout_sec_ = 0.10;
  }

  if (servo_timeout_sec_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "servo_timeout_sec %.3f <= 0. Using 0.10.",
      servo_timeout_sec_);
    servo_timeout_sec_ = 0.10;
  }

  if (slip_enter_hold_sec_ < 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "slip_enter_hold_sec %.3f < 0. Using 0.10.",
      slip_enter_hold_sec_);
    slip_enter_hold_sec_ = 0.10;
  }

  if (slip_exit_hold_sec_ < 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "slip_exit_hold_sec %.3f < 0. Using 0.20.",
      slip_exit_hold_sec_);
    slip_exit_hold_sec_ = 0.20;
  }

  if (std::fabs(speed_to_erpm_gain_) < kEpsilon) {
    RCLCPP_WARN(
      get_logger(),
      "speed_to_erpm_gain is %.6f. Odometry cannot convert ERPM to speed until this is set.",
      speed_to_erpm_gain_);
  }

  if (imu_startup_calibration_enabled_ && imu_startup_calibration_duration_sec_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "imu_startup_calibration_duration_sec %.3f <= 0. Disabling startup IMU calibration.",
      imu_startup_calibration_duration_sec_);
    imu_startup_calibration_enabled_ = false;
  }

  if (imu_use_butterworth_filter_) {
    if (imu_butterworth_gyro_cutoff_hz_ <= 0.0 ||
      imu_butterworth_lateral_accel_cutoff_hz_ <= 0.0)
    {
      RCLCPP_WARN(
        get_logger(),
        "Butterworth cutoffs must be positive (gyro=%.3f, lat_accel=%.3f). Falling back to EMA.",
        imu_butterworth_gyro_cutoff_hz_,
        imu_butterworth_lateral_accel_cutoff_hz_);
      imu_use_butterworth_filter_ = false;
    } else {
      RCLCPP_INFO(
        get_logger(),
        "IMU 2nd-order Butterworth enabled (gyro cutoff=%.2f Hz, linear-y cutoff=%.2f Hz).",
        imu_butterworth_gyro_cutoff_hz_,
        imu_butterworth_lateral_accel_cutoff_hz_);
    }
  }

  if (use_dynamic_bicycle_model_ &&
    (vehicle_mass_ <= 0.0 || vehicle_Iz_ <= 0.0 || l_f_ <= 0.0 || l_r_ <= 0.0 ||
    c_alpha_f_ <= 0.0 || c_alpha_r_ <= 0.0))
  {
    RCLCPP_WARN(
      get_logger(),
      "Dynamic bicycle model parameters invalid. Falling back to kinematic yaw model.");
    use_dynamic_bicycle_model_ = false;
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

  const rclcpp::Time current_time = now();
  const double imu_age_sec =
    imu_receive_time_initialized_ ? (current_time - last_imu_receive_time_).seconds() :
    std::numeric_limits<double>::infinity();
  const bool imu_fresh = imu_age_sec <= imu_timeout_sec_;

  const bool servo_message_seen = servo_receive_time_initialized_ && static_cast<bool>(last_servo_cmd_);
  const double servo_age_sec =
    servo_message_seen ? (current_time - last_servo_receive_time_).seconds() :
    std::numeric_limits<double>::infinity();
  const bool servo_fresh = servo_message_seen && servo_age_sec <= servo_timeout_sec_;

  if (!imu_fresh) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "IMU data is stale (age %.4f s > timeout %.4f s). Falling back from IMU yaw fusion.",
      imu_age_sec, imu_timeout_sec_);
  }

  if (servo_message_seen && !servo_fresh) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Servo command is stale (age %.4f s > timeout %.4f s). Ignoring steering model for this update.",
      servo_age_sec, servo_timeout_sec_);
  }

  if (!imu_fresh && !servo_fresh) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Skipping odom update: both IMU and servo are stale.");
    last_state_ = state;
    return;
  }

  // Wheel speed from ERPM calibration.
  double current_speed = (state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
  if (std::fabs(current_speed) < speed_deadband_) {
    current_speed = 0.0;
  }

  if (
    imu_startup_calibration_enabled_ &&
    imu_startup_hold_odom_during_calibration_ &&
    !imu_startup_calibration_done_)
  {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "IMU startup calibration in progress. Holding odom output for %.2f seconds.",
      imu_startup_calibration_duration_sec_);
    last_state_ = state;
    return;
  }

  // Steering-derived model yaw rate.
  const bool has_servo = servo_fresh;
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

  double model_lateral_velocity = imu_lateral_velocity_;
  if (use_dynamic_bicycle_model_ && has_servo && std::fabs(current_speed) > dynamic_model_min_speed_) {
    const double safe_vx =
      std::copysign(std::max(std::fabs(current_speed), dynamic_model_min_speed_), current_speed);
    const double alpha_f = steering_angle - (model_lateral_velocity + l_f_ * yaw_rate_state_) / safe_vx;
    const double alpha_r = -(model_lateral_velocity - l_r_ * yaw_rate_state_) / safe_vx;

    const double f_yf = c_alpha_f_ * alpha_f;
    const double f_yr = c_alpha_r_ * alpha_r;

    const double v_y_dot = (f_yf + f_yr) / vehicle_mass_ - safe_vx * yaw_rate_state_;
    const double yaw_dot = (l_f_ * f_yf - l_r_ * f_yr) / vehicle_Iz_;

    model_lateral_velocity += v_y_dot * dt_sec;
    model_yaw_rate = yaw_rate_state_ + yaw_dot * dt_sec;
  }

  // Time-align IMU and VESC updates by using the nearest IMU sample to the VESC stamp.
  double imu_yaw_rate_raw = filtered_angular_velocity_;
  double lateral_accel_measured = filtered_linear_accel_y_;
  double lateral_accel_measured_for_slip = debiased_linear_accel_y_raw_;
  double yaw_rate_measured_for_slip = debiased_angular_velocity_z_raw_;

  if (imu_fresh && !imu_history_.empty()) {
    const rclcpp::Time state_stamp(state->header.stamp);
    const ImuSyncSample * nearest_sample = nullptr;
    double nearest_abs_dt_sec = std::numeric_limits<double>::infinity();

    for (const auto & sample : imu_history_) {
      const double abs_dt_sec = std::fabs((state_stamp - sample.stamp).seconds());
      if (abs_dt_sec < nearest_abs_dt_sec) {
        nearest_abs_dt_sec = abs_dt_sec;
        nearest_sample = &sample;
      }
    }

    if (nearest_sample != nullptr) {
      imu_yaw_rate_raw = nearest_sample->filtered_angular_velocity;
      lateral_accel_measured = nearest_sample->filtered_linear_accel_y;
      lateral_accel_measured_for_slip = nearest_sample->raw_linear_accel_y;
      yaw_rate_measured_for_slip = nearest_sample->raw_angular_velocity;

      if (nearest_abs_dt_sec > imu_sync_tolerance_sec_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "IMU/VESC timestamp offset is %.4f s (tolerance %.4f s).",
          nearest_abs_dt_sec, imu_sync_tolerance_sec_);
      }
    }
  }

  if (!imu_fresh) {
    // If IMU is stale, use model-consistent channels to avoid stale residual spikes.
    imu_yaw_rate_raw = model_yaw_rate + gyro_bias_;
    lateral_accel_measured = current_speed * model_yaw_rate;
    lateral_accel_measured_for_slip = lateral_accel_measured;
    yaw_rate_measured_for_slip = model_yaw_rate + gyro_bias_;
  }

  // Slip indicator uses fast de-biased IMU channels to keep slip response quick.
  const double lateral_accel_model = current_speed * model_yaw_rate;
  const double lateral_accel_residual_abs =
    std::fabs(lateral_accel_measured_for_slip - lateral_accel_model);
  const double yaw_rate_residual_abs =
    std::fabs((yaw_rate_measured_for_slip - gyro_bias_) - model_yaw_rate);
  const double lateral_velocity_error_abs = std::fabs(imu_lateral_velocity_ - model_lateral_velocity);
  const double yaw_indicator = slip_yaw_rate_weight_ * yaw_rate_residual_abs;
  const double lateral_accel_indicator = std::min(lateral_accel_residual_abs, slip_accel_clip_);

  double slip_indicator_raw = yaw_indicator;
  double slip_enter_threshold = slip_accel_enter_;
  double slip_exit_threshold = slip_accel_exit_;

  if (slip_indicator_source_ == "lateral_accel") {
    slip_indicator_raw = lateral_accel_indicator;
  } else if (slip_indicator_source_ == "lateral_velocity_error") {
    slip_indicator_raw = lateral_velocity_error_abs;
    slip_enter_threshold = slip_lateral_velocity_enter_;
    slip_exit_threshold = slip_lateral_velocity_exit_;
  } else if (slip_use_lateral_accel_) {
    slip_indicator_raw = std::max(yaw_indicator, lateral_accel_indicator);
  }

  if (!slip_indicator_initialized_) {
    filtered_slip_indicator_ = slip_indicator_raw;
    slip_indicator_initialized_ = true;
  } else {
    filtered_slip_indicator_ =
      slip_indicator_alpha_ * slip_indicator_raw +
      (1.0 - slip_indicator_alpha_) * filtered_slip_indicator_;
  }

  if (std::fabs(current_speed) < slip_min_speed_) {
    slip_enter_timer_ = 0.0;
    slip_exit_timer_ = 0.0;
    slip_active_ = false;
  } else if (!slip_active_) {
    if (filtered_slip_indicator_ > slip_enter_threshold) {
      slip_enter_timer_ += dt_sec;
    } else {
      slip_enter_timer_ = 0.0;
    }

    if (slip_enter_timer_ >= slip_enter_hold_sec_) {
      slip_active_ = true;
      slip_enter_timer_ = 0.0;
      slip_exit_timer_ = 0.0;
      RCLCPP_INFO(
        get_logger(),
        "Slip mode ON (raw=%.3f, filtered=%.3f, enter=%.3f).",
        slip_indicator_raw, filtered_slip_indicator_, slip_enter_threshold);
    }
  } else {
    if (filtered_slip_indicator_ < slip_exit_threshold) {
      slip_exit_timer_ += dt_sec;
    } else {
      slip_exit_timer_ = 0.0;
    }

    if (slip_exit_timer_ >= slip_exit_hold_sec_) {
      slip_active_ = false;
      slip_enter_timer_ = 0.0;
      slip_exit_timer_ = 0.0;
      RCLCPP_INFO(
        get_logger(),
        "Slip mode OFF (raw=%.3f, filtered=%.3f, exit=%.3f).",
        slip_indicator_raw, filtered_slip_indicator_, slip_exit_threshold);
    }
  }

  const double slip_threshold_delta =
    std::max(slip_enter_threshold - slip_exit_threshold, kEpsilon);
  double slip_weight = std::clamp(
    (filtered_slip_indicator_ - slip_exit_threshold) / slip_threshold_delta,
    0.0,
    1.0);
  if (!has_servo) {
    slip_weight = 1.0;
  }
  if (std::fabs(current_speed) < slip_min_speed_) {
    slip_weight = 0.0;
  }

  if (imu_fresh && has_servo && slip_weight < 0.2 && std::fabs(current_speed) > 0.5) {
    const double yaw_rate_error = imu_yaw_rate_raw - model_yaw_rate;
    gyro_bias_ = (1.0 - gyro_bias_alpha_) * gyro_bias_ + gyro_bias_alpha_ * yaw_rate_error;
  }

  const double imu_yaw_rate = imu_yaw_rate_raw - gyro_bias_;
  const double imu_yaw_weight = imu_fresh ?
    (imu_yaw_base_weight_ + (1.0 - imu_yaw_base_weight_) * slip_weight) :
    0.0;
  const double current_yaw_rate =
    (1.0 - imu_yaw_weight) * model_yaw_rate + imu_yaw_weight * imu_yaw_rate;

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

  double imu_lateral_velocity_estimate = imu_lateral_velocity_;
  imu_lateral_velocity_estimate += filtered_lateral_accel_ * dt_sec;
  const double decay = std::max(0.0, 1.0 - imu_lateral_velocity_decay_ * dt_sec);
  imu_lateral_velocity_estimate *= decay;
  if (std::fabs(current_speed) < 0.2) {
    imu_lateral_velocity_estimate = 0.0;
  }
  imu_lateral_velocity_estimate = std::clamp(
    imu_lateral_velocity_estimate, -imu_lateral_velocity_max_, imu_lateral_velocity_max_);

  // Use dynamic bicycle lateral state in low-slip and IMU estimate in high-slip.
  if (use_dynamic_bicycle_model_ && has_servo && std::fabs(current_speed) > dynamic_model_min_speed_) {
    imu_lateral_velocity_ =
      (1.0 - slip_weight) * model_lateral_velocity + slip_weight * imu_lateral_velocity_estimate;
  } else {
    imu_lateral_velocity_ = imu_lateral_velocity_estimate;
  }

  // Blend kinematic and IMU slip-angle estimates.
  double beta_kinematic = 0.0;
  if (has_servo) {
    beta_kinematic = std::atan(kinematic_beta_ratio_ * std::tan(steering_angle));
  }
  const double beta_imu = std::atan2(imu_lateral_velocity_, std::max(0.4, std::fabs(current_speed)));
  const double beta = std::clamp(
    (1.0 - slip_weight) * beta_kinematic + slip_weight * beta_imu,
    -beta_max_rad_, beta_max_rad_);

  // Integrate pose with midpoint heading to reduce turn-rate integration error.
  const double yaw_mid = normalizeAngle(yaw_ + 0.5 * current_yaw_rate * dt_sec);
  yaw_ = normalizeAngle(yaw_ + current_yaw_rate * dt_sec);
  yaw_rate_state_ = current_yaw_rate;
  const double heading = yaw_mid + beta;
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

  const double xy_cov = odom_x_covariance_ * (1.0 + slip_weight * (slip_xy_covariance_scale_ - 1.0));
  const double yaw_cov = odom_yaw_covariance_ * (1.0 + slip_weight * (slip_yaw_covariance_scale_ - 1.0));

  std::fill(odom->pose.covariance.begin(), odom->pose.covariance.end(), 0.0);
  constexpr double kUnsupportedPoseCov = 1e3;
  odom->pose.covariance[0] = xy_cov;
  odom->pose.covariance[7] = odom_y_covariance_ *
    (1.0 + slip_weight * (slip_xy_covariance_scale_ - 1.0));
  odom->pose.covariance[14] = kUnsupportedPoseCov;
  odom->pose.covariance[21] = kUnsupportedPoseCov;
  odom->pose.covariance[28] = kUnsupportedPoseCov;
  odom->pose.covariance[35] = yaw_cov;

  std::fill(odom->twist.covariance.begin(), odom->twist.covariance.end(), 0.0);
  constexpr double kUnsupportedTwistCov = 1e3;
  const double vx_cov = std::max(0.02, xy_cov);
  const double vy_cov = std::max(0.05, xy_cov);
  const double wz_cov = std::max(0.05, yaw_cov);
  odom->twist.covariance[0] = vx_cov;
  odom->twist.covariance[7] = vy_cov;
  odom->twist.covariance[14] = kUnsupportedTwistCov;
  odom->twist.covariance[21] = kUnsupportedTwistCov;
  odom->twist.covariance[28] = kUnsupportedTwistCov;
  odom->twist.covariance[35] = wz_cov;

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
  last_imu_receive_time_ = now();
  imu_receive_time_initialized_ = true;

  const bool has_valid_stamp =
    (imu->header.stamp.sec != 0) || (imu->header.stamp.nanosec != 0);
  const rclcpp::Time sample_stamp = has_valid_stamp ? rclcpp::Time(imu->header.stamp) : now();
  const double sample_time_sec = sample_stamp.seconds();

  if (imu_startup_calibration_enabled_ && !imu_startup_calibration_done_) {
    if (!imu_startup_calibration_started_) {
      imu_startup_calibration_started_ = true;
      imu_startup_calibration_start_sec_ = sample_time_sec;
      imu_startup_calibration_sample_count_ = 0;
      imu_startup_linear_accel_x_sum_ = 0.0;
      imu_startup_linear_accel_y_sum_ = 0.0;
      imu_startup_angular_velocity_z_sum_ = 0.0;
      RCLCPP_INFO(
        get_logger(),
        "Starting IMU startup calibration for %.2f seconds. Keep the car stationary.",
        imu_startup_calibration_duration_sec_);
    }

    imu_startup_linear_accel_x_sum_ += imu->linear_acceleration.x;
    imu_startup_linear_accel_y_sum_ += imu->linear_acceleration.y;
    imu_startup_angular_velocity_z_sum_ += imu->angular_velocity.z;
    ++imu_startup_calibration_sample_count_;

    const double elapsed_sec = sample_time_sec - imu_startup_calibration_start_sec_;
    if (elapsed_sec >= imu_startup_calibration_duration_sec_ &&
      imu_startup_calibration_sample_count_ > 0U)
    {
      const double inv_samples =
        1.0 / static_cast<double>(std::max<std::size_t>(imu_startup_calibration_sample_count_, 1U));
      imu_startup_linear_accel_x_bias_ = imu_startup_linear_accel_x_sum_ * inv_samples;
      imu_startup_linear_accel_y_bias_ = imu_startup_linear_accel_y_sum_ * inv_samples;
      imu_startup_angular_velocity_z_bias_ = imu_startup_angular_velocity_z_sum_ * inv_samples;
      imu_startup_calibration_done_ = true;

      RCLCPP_INFO(
        get_logger(),
        "IMU startup calibration done (%zu samples): accel_bias_x=%.6f, accel_bias_y=%.6f, gyro_bias_z=%.6f",
        imu_startup_calibration_sample_count_,
        imu_startup_linear_accel_x_bias_,
        imu_startup_linear_accel_y_bias_,
        imu_startup_angular_velocity_z_bias_);
    }
  }

  double imu_dt_sec = 0.0;
  if (imu_sample_time_initialized_) {
    imu_dt_sec = sample_time_sec - last_imu_sample_time_sec_;
    if (imu_dt_sec <= 0.0 || imu_dt_sec > max_dt_sec_) {
      imu_dt_sec = 0.0;
    }
  }
  imu_sample_time_initialized_ = true;
  last_imu_sample_time_sec_ = sample_time_sec;

  double current_gyro_z_bias = imu_startup_angular_velocity_z_bias_;
  double current_linear_accel_y_bias = imu_startup_linear_accel_y_bias_;
  if (imu_startup_calibration_enabled_ && !imu_startup_calibration_done_ &&
    imu_startup_calibration_sample_count_ > 0U)
  {
    current_gyro_z_bias =
      imu_startup_angular_velocity_z_sum_ /
      static_cast<double>(imu_startup_calibration_sample_count_);
    current_linear_accel_y_bias =
      imu_startup_linear_accel_y_sum_ /
      static_cast<double>(imu_startup_calibration_sample_count_);
  }

  const double raw_angular_velocity = imu->angular_velocity.z - current_gyro_z_bias;
  const double raw_linear_accel_y = imu->linear_acceleration.y - current_linear_accel_y_bias;
  debiased_angular_velocity_z_raw_ = raw_angular_velocity;
  debiased_linear_accel_y_raw_ = raw_linear_accel_y;

  auto apply_butterworth_lowpass =
    [&](double input,
    double cutoff_hz,
    bool & filter_initialized,
    double & x1,
    double & x2,
    double & y1,
    double & y2,
    double dt_sec) -> double
    {
      if (!filter_initialized) {
        filter_initialized = true;
        x1 = input;
        x2 = input;
        y1 = input;
        y2 = input;
        return input;
      }

      if (dt_sec <= 0.0 || cutoff_hz <= 0.0) {
        x2 = x1;
        x1 = input;
        y2 = y1;
        y1 = input;
        return input;
      }

      const double sample_rate_hz = 1.0 / dt_sec;
      const double max_cutoff_hz = 0.45 * sample_rate_hz;
      if (max_cutoff_hz <= 0.0) {
        x2 = x1;
        x1 = input;
        y2 = y1;
        y1 = input;
        return input;
      }

      const double limited_cutoff_hz = std::min(cutoff_hz, max_cutoff_hz);
      const double q = std::sqrt(0.5);  // Butterworth Q = 1/sqrt(2)
      const double omega = 2.0 * M_PI * limited_cutoff_hz / sample_rate_hz;
      const double sin_omega = std::sin(omega);
      const double cos_omega = std::cos(omega);
      const double alpha = sin_omega / (2.0 * q);

      double b0 = (1.0 - cos_omega) * 0.5;
      double b1 = 1.0 - cos_omega;
      double b2 = (1.0 - cos_omega) * 0.5;
      double a0 = 1.0 + alpha;
      double a1 = -2.0 * cos_omega;
      double a2 = 1.0 - alpha;

      const double a0_inv = 1.0 / a0;
      b0 *= a0_inv;
      b1 *= a0_inv;
      b2 *= a0_inv;
      a1 *= a0_inv;
      a2 *= a0_inv;

      const double output =
        b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;

      x2 = x1;
      x1 = input;
      y2 = y1;
      y1 = output;
      return output;
    };

  if (imu_use_butterworth_filter_) {
    filtered_angular_velocity_ = apply_butterworth_lowpass(
      raw_angular_velocity,
      imu_butterworth_gyro_cutoff_hz_,
      gyro_biquad_initialized_,
      gyro_biquad_x1_,
      gyro_biquad_x2_,
      gyro_biquad_y1_,
      gyro_biquad_y2_,
      imu_dt_sec);

    filtered_linear_accel_y_ = apply_butterworth_lowpass(
      raw_linear_accel_y,
      imu_butterworth_lateral_accel_cutoff_hz_,
      lateral_accel_y_biquad_initialized_,
      lateral_accel_y_biquad_x1_,
      lateral_accel_y_biquad_x2_,
      lateral_accel_y_biquad_y1_,
      lateral_accel_y_biquad_y2_,
      imu_dt_sec);

    angular_velocity_filter_initialized_ = true;
  } else {
    if (!angular_velocity_filter_initialized_) {
      filtered_angular_velocity_ = raw_angular_velocity;
      angular_velocity_filter_initialized_ = true;
    } else {
      filtered_angular_velocity_ =
        imu_angular_velocity_alpha_ * raw_angular_velocity +
        (1.0 - imu_angular_velocity_alpha_) * filtered_angular_velocity_;
    }
    filtered_linear_accel_y_ = raw_linear_accel_y;
  }

  ImuSyncSample sync_sample;
  sync_sample.stamp = sample_stamp;
  sync_sample.filtered_angular_velocity = filtered_angular_velocity_;
  sync_sample.filtered_linear_accel_y = filtered_linear_accel_y_;
  sync_sample.raw_angular_velocity = debiased_angular_velocity_z_raw_;
  sync_sample.raw_linear_accel_y = debiased_linear_accel_y_raw_;
  imu_history_.push_back(sync_sample);
  while (static_cast<int>(imu_history_.size()) > imu_history_max_samples_) {
    imu_history_.pop_front();
  }

  auto filtered_msg = std::make_unique<Float64>();
  filtered_msg->data = filtered_angular_velocity_;
  filtered_angular_velocity_pub_->publish(std::move(filtered_msg));
}

void VescToOdom::servoCmdCallback(const Float64::SharedPtr servo)
{
  last_servo_cmd_ = servo;
  last_servo_receive_time_ = now();
  servo_receive_time_initialized_ = true;
}

}  // namespace vesc_ackermann

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::VescToOdom)
