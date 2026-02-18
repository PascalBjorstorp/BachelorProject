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

#include "vesc_ackermann/ackermann_to_vesc.hpp"

#include <cmath>
#include <sstream>
#include <string>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>

namespace vesc_ackermann
{

using ackermann_msgs::msg::AckermannDriveStamped;
using nav_msgs::msg::Odometry;
using std::placeholders::_1;
using std_msgs::msg::Float64;

AckermannToVesc::AckermannToVesc(const rclcpp::NodeOptions & options)
: Node("ackermann_to_vesc_node", options)
{
  // Declare parameters with defaults
  declare_parameter("speed_to_erpm_gain", 0.0);
  declare_parameter("speed_to_erpm_offset", 0.0);
  declare_parameter("steering_angle_to_servo_gain", 0.0);
  declare_parameter("steering_angle_to_servo_offset", 0.0);

  // Braking parameters
  declare_parameter("brake_deadzone", 0.1);
  declare_parameter("speed_to_braking_gain", 0.0);
  declare_parameter("speed_to_braking_center", 0.0);
  declare_parameter("speed_to_braking_max", 20000.0);
  declare_parameter("speed_to_braking_min", 0.0);

  // Acceleration control parameters
  declare_parameter("accel_to_current_gain", 0.0);
  declare_parameter("accel_to_brake_gain", 0.0);

  // Slow-start parameters (for sensorless motors that need low-speed
  // rotation to detect rotor position before full ERPM can be commanded)
  declare_parameter("slow_start_threshold", 1.0);  // m/s
  declare_parameter("slow_start_increment", 0.4);   // m/s

  // Get conversion parameters
  speed_to_erpm_gain_ = get_parameter("speed_to_erpm_gain").get_value<double>();
  speed_to_erpm_offset_ = get_parameter("speed_to_erpm_offset").get_value<double>();
  steering_to_servo_gain_ = get_parameter("steering_angle_to_servo_gain").get_value<double>();
  steering_to_servo_offset_ = get_parameter("steering_angle_to_servo_offset").get_value<double>();

  // Braking parameters
  brake_deadzone_ = get_parameter("brake_deadzone").get_value<double>();
  speed_to_braking_gain_ = get_parameter("speed_to_braking_gain").get_value<double>();
  speed_to_braking_center_ = get_parameter("speed_to_braking_center").get_value<double>();
  speed_to_braking_max_ = get_parameter("speed_to_braking_max").get_value<double>();
  speed_to_braking_min_ = get_parameter("speed_to_braking_min").get_value<double>();

  // Acceleration parameters
  accel_to_current_gain_ = get_parameter("accel_to_current_gain").get_value<double>();
  accel_to_brake_gain_ = get_parameter("accel_to_brake_gain").get_value<double>();

  // Slow-start parameters
  slow_start_threshold_ = get_parameter("slow_start_threshold").get_value<double>();
  slow_start_increment_ = get_parameter("slow_start_increment").get_value<double>();

  // Initialize state
  current_vel_ = 0.0;
  operation_mode_ = VEL_TO_ERPM;

  // Create publishers
  erpm_pub_ = create_publisher<Float64>("commands/motor/speed", 10);
  servo_pub_ = create_publisher<Float64>("commands/servo/position", 10);
  brake_pub_ = create_publisher<Float64>("commands/motor/brake", 10);
  current_pub_ = create_publisher<Float64>("commands/motor/current", 10);

  // Subscribe to ackermann commands
  ackermann_sub_ = create_subscription<AckermannDriveStamped>(
    "ackermann_cmd", 10, std::bind(&AckermannToVesc::ackermannCmdCallback, this, _1));

  // Subscribe to odometry for velocity feedback
  odom_sub_ = create_subscription<Odometry>(
    "ego_racecar/odom", 10, std::bind(&AckermannToVesc::odomCallback, this, _1));
}

void AckermannToVesc::ackermannCmdCallback(const AckermannDriveStamped::SharedPtr cmd)
{
  // Initialize messages
  Float64 servo_msg;
  Float64 brake_msg;
  Float64 current_msg;
  Float64 erpm_msg;
  bool publish_brake = false;
  bool publish_erpm = false;

  // Zero-initialize
  brake_msg.data = 0.0;
  current_msg.data = 0.0;
  erpm_msg.data = 0.0;

  // Calculate steering angle (servo)
  servo_msg.data = steering_to_servo_gain_ * cmd->drive.steering_angle + steering_to_servo_offset_;

  // Case 1: Acceleration-to-current mode (if gains are set)
  if (accel_to_current_gain_ != 0 && accel_to_brake_gain_ != 0) {
    if (cmd->drive.acceleration != 0) {
      // Direct acceleration command
      operation_mode_ = ACCEL_TO_CURRENT;
      if (cmd->drive.acceleration < 0) {
        brake_msg.data = accel_to_brake_gain_ * std::abs(cmd->drive.acceleration);
        publish_brake = true;
      } else {
        current_msg.data = accel_to_current_gain_ * cmd->drive.acceleration;
        publish_erpm = true;
      }
    } else if (cmd->drive.speed != 0 || operation_mode_ == VEL_TO_CURRENT) {
      // Velocity-to-current mode with feedback
      operation_mode_ = VEL_TO_CURRENT;
      double commanded_vel = cmd->drive.speed;
      double acceleration = 10.0 * (commanded_vel - current_vel_);
      if (acceleration > 0) {
        current_msg.data = acceleration * accel_to_current_gain_;
        publish_erpm = true;
      } else {
        brake_msg.data = std::abs(acceleration) * accel_to_brake_gain_;
        publish_brake = true;
      }
    }
  } else {
    // Case 2: Velocity-to-ERPM mode (default, when accel gains are 0)
    // Always command ERPM and let the VESC internal PID handle both
    // acceleration and deceleration. Only use explicit braking for
    // direction changes (forward<->reverse).
    operation_mode_ = VEL_TO_ERPM;
    double commanded_vel = cmd->drive.speed;

    // Check for direction change (forward vs reverse)
    bool direction_change = (current_vel_ > 0.1 && commanded_vel < -0.1) ||
                            (current_vel_ < -0.1 && commanded_vel > 0.1);

    if (direction_change) {
      // Direction reversal: brake to stop first
      brake_msg.data = speed_to_braking_max_;
      publish_brake = true;
    } else if (commanded_vel >= 0) {
      // Forward direction
      if (current_vel_ < slow_start_threshold_ && commanded_vel > slow_start_threshold_) {
        // Slow start to get rotor position (sensorless motor)
        erpm_msg.data = speed_to_erpm_gain_ * (current_vel_ + slow_start_increment_) + speed_to_erpm_offset_;
      } else {
        // Direct ERPM command — VESC PID handles accel and decel
        erpm_msg.data = speed_to_erpm_gain_ * commanded_vel + speed_to_erpm_offset_;
      }
      publish_erpm = true;
    } else {
      // Reverse direction
      if (current_vel_ == 0 && commanded_vel < -slow_start_increment_) {
        // Slow start for reverse (sensorless motor)
        erpm_msg.data = speed_to_erpm_gain_ * -slow_start_increment_ + speed_to_erpm_offset_;
      } else {
        // Direct ERPM command — VESC PID handles accel and decel
        erpm_msg.data = speed_to_erpm_gain_ * commanded_vel + speed_to_erpm_offset_;
      }
      publish_erpm = true;
    }
  }

  // Publish commands
  if (rclcpp::ok()) {
    if (publish_brake && brake_msg.data != 0.0) {
      brake_pub_->publish(brake_msg);
    } else if (publish_erpm) {
      if (operation_mode_ == ACCEL_TO_CURRENT || operation_mode_ == VEL_TO_CURRENT) {
        if (current_msg.data != 0.0) {
          current_pub_->publish(current_msg);
        }
      } else if (operation_mode_ == VEL_TO_ERPM) {
        if (erpm_msg.data != 0.0) {
          erpm_pub_->publish(erpm_msg);
        }
      }
    }
    servo_pub_->publish(servo_msg);
  }
}

void AckermannToVesc::odomCallback(const Odometry::SharedPtr odom_msg)
{
  current_vel_ = odom_msg->twist.twist.linear.x;
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::AckermannToVesc)
