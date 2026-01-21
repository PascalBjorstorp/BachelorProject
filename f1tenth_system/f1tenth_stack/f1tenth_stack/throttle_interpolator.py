# Copyright (c) 2020 Hongrui Zheng
# Modified for enhanced f1tenth_system
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class ThrottleInterpolator(Node):
    """
    Smooths throttle and servo commands to limit acceleration and jerk.

    This node takes 'unsmoothed' commands and publishes smoothed versions
    that respect maximum acceleration and servo speed limits.
    """

    def __init__(self):
        super().__init__('throttle_interpolator')

        # Declare parameters
        self.declare_parameter('rpm_input_topic', 'commands/motor/unsmoothed_speed')
        self.declare_parameter('rpm_output_topic', 'commands/motor/speed')
        self.declare_parameter('servo_input_topic', 'commands/servo/unsmoothed_position')
        self.declare_parameter('servo_output_topic', 'commands/servo/position')
        self.declare_parameter('max_acceleration', 2.5)  # m/s^2
        self.declare_parameter('speed_max', 100000.0)
        self.declare_parameter('speed_min', -100000.0)
        self.declare_parameter('throttle_smoother_rate', 75.0)  # Hz
        self.declare_parameter('speed_to_erpm_gain', 4450.0)
        self.declare_parameter('max_servo_speed', 3.2)  # rad/s
        self.declare_parameter('steering_angle_to_servo_gain', -0.915)
        self.declare_parameter('servo_smoother_rate', 75.0)  # Hz
        self.declare_parameter('servo_max', 0.82)
        self.declare_parameter('servo_min', 0.0)
        self.declare_parameter('steering_angle_to_servo_offset', 0.468)

        # Get parameters
        self.rpm_input_topic = self.get_parameter('rpm_input_topic').value
        self.rpm_output_topic = self.get_parameter('rpm_output_topic').value
        self.servo_input_topic = self.get_parameter('servo_input_topic').value
        self.servo_output_topic = self.get_parameter('servo_output_topic').value
        self.max_acceleration = self.get_parameter('max_acceleration').value
        self.max_rpm = self.get_parameter('speed_max').value
        self.min_rpm = self.get_parameter('speed_min').value
        self.throttle_smoother_rate = self.get_parameter('throttle_smoother_rate').value
        self.speed_to_erpm_gain = self.get_parameter('speed_to_erpm_gain').value
        self.max_servo_speed = self.get_parameter('max_servo_speed').value
        self.steering_angle_to_servo_gain = self.get_parameter(
            'steering_angle_to_servo_gain').value
        self.servo_smoother_rate = self.get_parameter('servo_smoother_rate').value
        self.max_servo = self.get_parameter('servo_max').value
        self.min_servo = self.get_parameter('servo_min').value
        self.last_servo = self.get_parameter('steering_angle_to_servo_offset').value

        # State variables
        self.last_rpm = 0.0
        self.desired_rpm = self.last_rpm
        self.desired_servo_position = self.last_servo

        # Publishers
        self.rpm_output = self.create_publisher(Float64, self.rpm_output_topic, 1)
        self.servo_output = self.create_publisher(Float64, self.servo_output_topic, 1)

        # Subscribers
        self.rpm_sub = self.create_subscription(
            Float64,
            self.rpm_input_topic,
            self._process_throttle_command,
            1)
        self.servo_sub = self.create_subscription(
            Float64,
            self.servo_input_topic,
            self._process_servo_command,
            1)

        # Calculate max deltas per update
        self.max_delta_servo = abs(
            self.steering_angle_to_servo_gain * self.max_servo_speed / self.servo_smoother_rate
        )
        self.max_delta_rpm = abs(
            self.speed_to_erpm_gain * self.max_acceleration / self.throttle_smoother_rate
        )

        # Timers for smooth output
        self.servo_timer = self.create_timer(
            1.0 / self.servo_smoother_rate,
            self._publish_servo_command
        )
        self.rpm_timer = self.create_timer(
            1.0 / self.throttle_smoother_rate,
            self._publish_throttle_command
        )

        self.get_logger().info(
            f'Throttle interpolator started. '
            f'Max acceleration: {self.max_acceleration} m/s^2, '
            f'Max servo speed: {self.max_servo_speed} rad/s'
        )

    def _publish_throttle_command(self):
        """Publish smoothed throttle command."""
        desired_delta = self.desired_rpm - self.last_rpm
        clipped_delta = max(min(desired_delta, self.max_delta_rpm), -self.max_delta_rpm)
        smoothed_rpm = self.last_rpm + clipped_delta
        self.last_rpm = smoothed_rpm

        rpm_msg = Float64()
        rpm_msg.data = float(smoothed_rpm)
        self.rpm_output.publish(rpm_msg)

    def _process_throttle_command(self, msg):
        """Process incoming throttle command."""
        input_rpm = msg.data
        # Sanity clipping
        input_rpm = min(max(input_rpm, self.min_rpm), self.max_rpm)
        self.desired_rpm = input_rpm

    def _publish_servo_command(self):
        """Publish smoothed servo command."""
        desired_delta = self.desired_servo_position - self.last_servo
        clipped_delta = max(min(desired_delta, self.max_delta_servo), -self.max_delta_servo)
        smoothed_servo = self.last_servo + clipped_delta
        self.last_servo = smoothed_servo

        servo_msg = Float64()
        servo_msg.data = float(smoothed_servo)
        self.servo_output.publish(servo_msg)

    def _process_servo_command(self, msg):
        """Process incoming servo command."""
        input_servo = msg.data
        # Sanity clipping
        input_servo = min(max(input_servo, self.min_servo), self.max_servo)
        self.desired_servo_position = input_servo


def main(args=None):
    rclpy.init(args=args)
    node = ThrottleInterpolator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
