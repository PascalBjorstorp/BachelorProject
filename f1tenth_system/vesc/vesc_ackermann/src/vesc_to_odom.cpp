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

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

namespace vesc_ackermann
{

using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::Odometry;
using std::placeholders::_1;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;

VescToOdom::VescToOdom(const rclcpp::NodeOptions & options)
: Node("vesc_to_odom_node", options),
  odom_frame_("ego_racecar/odom"),
  base_frame_("base_link"),
  use_servo_cmd_(true),
  use_imu_(false),
  publish_tf_(false),
  integration_method_("euler"),
  x_(0.0),
  y_(0.0),
  yaw_(0.0),
  initial_imu_yaw_(0.0),
  imu_initialized_(false),
  imu_angular_velocity_alpha_(0.3),
  filtered_angular_velocity_(0.0),
  angular_velocity_filter_initialized_(false)
{
  // get ROS parameters
  odom_frame_ = declare_parameter("odom_frame", odom_frame_);
  base_frame_ = declare_parameter("base_frame", base_frame_);
  use_servo_cmd_ = declare_parameter("use_servo_cmd_to_calc_angular_velocity", use_servo_cmd_);
  use_imu_ = declare_parameter("use_imu", use_imu_);
  integration_method_ = declare_parameter("integration_method", integration_method_);
  imu_angular_velocity_alpha_ = declare_parameter("imu_angular_velocity_alpha",
      imu_angular_velocity_alpha_);

  // Odometry covariance parameters
  odom_x_covariance_ = declare_parameter("odom_x_covariance", 0.2);
  odom_y_covariance_ = declare_parameter("odom_y_covariance", 0.2);
  odom_yaw_covariance_ = declare_parameter("odom_yaw_covariance", 0.4);

  declare_parameter<double>("speed_to_erpm_gain", 0.0);
  declare_parameter<double>("speed_to_erpm_offset", 0.0);

  speed_to_erpm_gain_ = get_parameter("speed_to_erpm_gain").get_value<double>();
  speed_to_erpm_offset_ = get_parameter("speed_to_erpm_offset").get_value<double>();

  if (use_servo_cmd_) {
    declare_parameter<double>("steering_angle_to_servo_gain", 0.0);
    declare_parameter<double>("steering_angle_to_servo_offset", 0.0);
    declare_parameter<double>("wheelbase", 0.0);

    steering_to_servo_gain_ = get_parameter("steering_angle_to_servo_gain").get_value<double>();
    steering_to_servo_offset_ = get_parameter("steering_angle_to_servo_offset").get_value<double>();
    wheelbase_ = get_parameter("wheelbase").get_value<double>();
  }

  publish_tf_ = declare_parameter("publish_tf", publish_tf_);

  // Parameter validation
  if (use_imu_ && use_servo_cmd_) {
    RCLCPP_WARN(get_logger(),
      "Both use_imu and use_servo_cmd are true. IMU will be used for angular velocity and yaw.");
  }

  // Validate integration method
  if (integration_method_ != "euler" && integration_method_ != "trapezoidal" &&
    integration_method_ != "analytical")
  {
    RCLCPP_WARN(get_logger(),
      "Invalid integration_method '%s'. Using 'euler' as default. "
      "Valid options: 'euler', 'trapezoidal', 'analytical'",
      integration_method_.c_str());
    integration_method_ = "euler";
  }

  RCLCPP_INFO(get_logger(), "Using '%s' integration method for odometry",
              integration_method_.c_str());

  // create odom publisher
  odom_pub_ = create_publisher<Odometry>("ego_racecar/odom", 10);

  // create filtered angular velocity publisher
  if (use_imu_) {
    filtered_angular_velocity_pub_ = create_publisher<Float64>("imu/filtered_angular_velocity", 10);
  }

  // create tf broadcaster
  if (publish_tf_) {
    tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  // subscribe to vesc state and. optionally, servo command
  vesc_state_sub_ = create_subscription<VescStateStamped>(
    "sensors/core", 10, std::bind(&VescToOdom::vescStateCallback, this, _1));

  if (use_servo_cmd_) {
    servo_sub_ = create_subscription<Float64>(
      "sensors/servo_position_command", 10, std::bind(&VescToOdom::servoCmdCallback, this, _1));
  }

  if (use_imu_) {
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "sensors/imu/raw", 10, std::bind(&VescToOdom::imuCallback, this, _1));
  }
}


void VescToOdom::vescStateCallback(const VescStateStamped::SharedPtr state)
{
  // check that we have a last servo command if we are depending on it for angular velocity
  if (use_servo_cmd_ && !last_servo_cmd_) {
    RCLCPP_INFO_ONCE(this->get_logger(),
      "Waiting for servo command message to calculate angular velocity.");
    return;
  }

  // check that we have IMU data if we are using it
  if (use_imu_ && !last_imu_) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for IMU message to calculate angular velocity.");
    return;
  }

  // convert to engineering units
  // Linear model: speed = (ERPM - offset) / gain
  double erpm = state->state.speed;
  double current_speed = (erpm - speed_to_erpm_offset_) / speed_to_erpm_gain_;
  if (std::fabs(current_speed) < 0.05) {
    current_speed = 0.0;
  }

  double current_angular_velocity = 0.0;

  if (use_imu_) {
    // Use filtered IMU yaw rate (angular velocity around z-axis)
    current_angular_velocity = filtered_angular_velocity_;
  } else if (use_servo_cmd_) {
    // Calculate from steering angle
    double current_steering_angle =
      (last_servo_cmd_->data - steering_to_servo_offset_) / steering_to_servo_gain_;
    current_angular_velocity = current_speed * tan(current_steering_angle) / wheelbase_;
  }

  // use current state as last state if this is our first time here
  if (!last_state_) {
    last_state_ = state;
  }

  // calc elapsed time
  auto dt = rclcpp::Time(state->header.stamp) - rclcpp::Time(last_state_->header.stamp);

  // Check for abnormal dt (e.g., node restart, message dropout)
  const double MAX_DT = 1.0;  // 1 second
  if (dt.seconds() > MAX_DT || dt.seconds() < 0) {
    RCLCPP_WARN(get_logger(),
      "Abnormal dt detected: %.6f seconds. Skipping odometry update. "
      "Current stamp: %d.%09d, Last stamp: %d.%09d",
      dt.seconds(),
      state->header.stamp.sec, state->header.stamp.nanosec,
      last_state_->header.stamp.sec, last_state_->header.stamp.nanosec);
    last_state_ = state;
    return;
  }

  // Update yaw first (needed for position integration)
  double yaw_start = yaw_;
  double yaw_end = yaw_;

  if (use_imu_) {
    // Extract yaw from IMU quaternion
    tf2::Quaternion q(
      last_imu_->orientation.x,
      last_imu_->orientation.y,
      last_imu_->orientation.z,
      last_imu_->orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, current_imu_yaw;
    m.getRPY(roll, pitch, current_imu_yaw);

    // Initialize IMU yaw offset on first IMU data
    if (!imu_initialized_) {
      initial_imu_yaw_ = current_imu_yaw;
      imu_initialized_ = true;
      RCLCPP_INFO(get_logger(), "IMU initialized with yaw offset: %.3f rad", initial_imu_yaw_);
    }

    // Apply offset to make initial yaw = 0
    yaw_end = current_imu_yaw - initial_imu_yaw_;
    yaw_ = yaw_end;
  } else {
    yaw_end = yaw_ + current_angular_velocity * dt.seconds();
    yaw_ = yaw_end;
  }

  // Propagate odometry using selected integration method
  if (integration_method_ == "trapezoidal") {
    // Trapezoidal integration: average velocity at start and end of dt
    const double cos_start = cos(yaw_start);
    const double sin_start = sin(yaw_start);
    const double cos_end = cos(yaw_end);
    const double sin_end = sin(yaw_end);

    const double x_dot_start = current_speed * cos_start;
    const double y_dot_start = current_speed * sin_start;
    const double x_dot_end = current_speed * cos_end;
    const double y_dot_end = current_speed * sin_end;

    // Use average of start and end velocities
    const double dt_sec = dt.seconds();
    x_ += 0.5 * (x_dot_start + x_dot_end) * dt_sec;
    y_ += 0.5 * (y_dot_start + y_dot_end) * dt_sec;

  } else if (integration_method_ == "analytical") {
    // Analytical solution for Ackermann kinematics (circular arc)
    const double delta_yaw = yaw_end - yaw_start;

    const double cos_start = cos(yaw_start);
    const double sin_start = sin(yaw_start);

    if (std::fabs(delta_yaw) < 1e-6) {
      // Nearly straight motion: use simple forward integration
      const double dt_sec = dt.seconds();
      x_ += current_speed * cos_start * dt_sec;
      y_ += current_speed * sin_start * dt_sec;
    } else {
      // Circular arc motion: exact solution for constant curvature
      // Use actual delta_yaw to calculate turning radius (more accurate
      // than using angular velocity)
      const double actual_angular_velocity = delta_yaw / dt.seconds();
      const double turning_radius = current_speed / actual_angular_velocity;

      const double sin_delta = sin(delta_yaw);
      const double cos_delta = cos(delta_yaw);

      // Calculate displacement in vehicle frame (arc geometry)
      const double dx_vehicle = turning_radius * sin_delta;
      const double dy_vehicle = turning_radius * (1.0 - cos_delta);

      // Transform to global frame
      x_ += dx_vehicle * cos_start - dy_vehicle * sin_start;
      y_ += dx_vehicle * sin_start + dy_vehicle * cos_start;
    }

  } else {
    // Default: Euler integration (first-order)
    const double cos_start = cos(yaw_start);
    const double sin_start = sin(yaw_start);
    const double dt_sec = dt.seconds();
    x_ += current_speed * cos_start * dt_sec;
    y_ += current_speed * sin_start * dt_sec;
  }

  // save state for next time
  last_state_ = state;

  // publish odometry message
  Odometry odom;
  odom.header.frame_id = odom_frame_;
  odom.header.stamp = state->header.stamp;
  odom.child_frame_id = base_frame_;

  // Position
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = sin(yaw_ / 2.0);
  odom.pose.pose.orientation.w = cos(yaw_ / 2.0);

  // Position uncertainty - configurable via parameters
  odom.pose.covariance[0] = odom_x_covariance_;     // x
  odom.pose.covariance[7] = odom_y_covariance_;     // y
  odom.pose.covariance[35] = odom_yaw_covariance_;  // yaw

  // Velocity ("in the coordinate frame given by the child_frame_id")
  odom.twist.twist.linear.x = current_speed;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = current_angular_velocity;

  // Velocity uncertainty
  /** @todo Think about velocity uncertainty */

  if (publish_tf_) {
    TransformStamped tf;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.header.stamp = state->header.stamp;
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom.pose.pose.orientation;

    if (rclcpp::ok()) {
      tf_pub_->sendTransform(tf);
    }
  }

  if (rclcpp::ok()) {
    odom_pub_->publish(odom);
  }
}

void VescToOdom::servoCmdCallback(const Float64::SharedPtr servo)
{
  last_servo_cmd_ = servo;
}

void VescToOdom::imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu)
{
  last_imu_ = imu;

  // Apply low-pass filter to angular velocity
  double raw_angular_velocity = imu->angular_velocity.z;

  if (!angular_velocity_filter_initialized_) {
    // Initialize filter with first value
    filtered_angular_velocity_ = raw_angular_velocity;
    angular_velocity_filter_initialized_ = true;
  } else {
    // Apply exponential moving average (low-pass filter)
    // filtered = alpha * new + (1 - alpha) * old
    filtered_angular_velocity_ = imu_angular_velocity_alpha_ * raw_angular_velocity +
      (1.0 - imu_angular_velocity_alpha_) * filtered_angular_velocity_;
  }

  // Publish filtered angular velocity
  Float64 filtered_msg;
  filtered_msg.data = filtered_angular_velocity_;
  filtered_angular_velocity_pub_->publish(filtered_msg);
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::VescToOdom)
