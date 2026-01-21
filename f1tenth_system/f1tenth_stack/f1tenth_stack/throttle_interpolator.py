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
        self.declare_parameter('smoother_rate', 75.0)  # Hz - single rate for both
        self.declare_parameter('speed_to_erpm_gain', 4450.0)
        self.declare_parameter('max_servo_speed', 3.2)  # rad/s
        self.declare_parameter('steering_angle_to_servo_gain', -0.915)
        self.declare_parameter('servo_max', 0.82)
        self.declare_parameter('servo_min', 0.0)
        self.declare_parameter('steering_angle_to_servo_offset', 0.468)

        # Get parameters
        rpm_input_topic = self.get_parameter('rpm_input_topic').value
        rpm_output_topic = self.get_parameter('rpm_output_topic').value
        servo_input_topic = self.get_parameter('servo_input_topic').value
        servo_output_topic = self.get_parameter('servo_output_topic').value
        max_acceleration = self.get_parameter('max_acceleration').value
        self.max_rpm = self.get_parameter('speed_max').value
        self.min_rpm = self.get_parameter('speed_min').value
        smoother_rate = self.get_parameter('smoother_rate').value
        speed_to_erpm_gain = self.get_parameter('speed_to_erpm_gain').value
        max_servo_speed = self.get_parameter('max_servo_speed').value
        steering_angle_to_servo_gain = self.get_parameter(
            'steering_angle_to_servo_gain').value
        self.max_servo = self.get_parameter('servo_max').value
        self.min_servo = self.get_parameter('servo_min').value
        self.last_servo = self.get_parameter('steering_angle_to_servo_offset').value

        # State variables
        self.last_rpm = 0.0
        self.desired_rpm = self.last_rpm
        self.desired_servo_position = self.last_servo

        # Pre-allocate messages for reuse (avoid allocation in callbacks)
        self._rpm_msg = Float64()
        self._servo_msg = Float64()

        # Publishers with best_effort QoS for real-time performance
        self.rpm_output = self.create_publisher(Float64, rpm_output_topic, 1)
        self.servo_output = self.create_publisher(Float64, servo_output_topic, 1)

        # Subscribers
        self.rpm_sub = self.create_subscription(
            Float64,
            rpm_input_topic,
            self._process_throttle_command,
            1)
        self.servo_sub = self.create_subscription(
            Float64,
            servo_input_topic,
            self._process_servo_command,
            1)

        # Calculate max deltas per update (computed once at init)
        self.max_delta_servo = abs(
            steering_angle_to_servo_gain * max_servo_speed / smoother_rate
        )
        self.max_delta_rpm = abs(
            speed_to_erpm_gain * max_acceleration / smoother_rate
        )

        # Timer for smooth output
        self._timer = self.create_timer(1.0 / smoother_rate, self._publish_commands)

        self.get_logger().info(
            f'Throttle interpolator started at {smoother_rate} Hz. '
            f'Max accel: {max_acceleration} m/s^2, Max servo: {max_servo_speed} rad/s'
        )

    def _publish_commands(self):
        """Publish both smoothed commands."""
        # Process throttle
        desired_delta = self.desired_rpm - self.last_rpm
        if desired_delta > self.max_delta_rpm:
            clipped_delta = self.max_delta_rpm
        elif desired_delta < -self.max_delta_rpm:
            clipped_delta = -self.max_delta_rpm
        else:
            clipped_delta = desired_delta
        self.last_rpm += clipped_delta

        self._rpm_msg.data = self.last_rpm
        self.rpm_output.publish(self._rpm_msg)

        # Process servo
        desired_delta = self.desired_servo_position - self.last_servo
        if desired_delta > self.max_delta_servo:
            clipped_delta = self.max_delta_servo
        elif desired_delta < -self.max_delta_servo:
            clipped_delta = -self.max_delta_servo
        else:
            clipped_delta = desired_delta
        self.last_servo += clipped_delta

        self._servo_msg.data = self.last_servo
        self.servo_output.publish(self._servo_msg)

    def _process_throttle_command(self, msg):
        """Process incoming throttle command."""
        input_rpm = msg.data
        # Sanity clipping using branch-free clamping
        if input_rpm > self.max_rpm:
            self.desired_rpm = self.max_rpm
        elif input_rpm < self.min_rpm:
            self.desired_rpm = self.min_rpm
        else:
            self.desired_rpm = input_rpm

    def _process_servo_command(self, msg):
        """Process incoming servo command."""
        input_servo = msg.data
        # Sanity clipping
        if input_servo > self.max_servo:
            self.desired_servo_position = self.max_servo
        elif input_servo < self.min_servo:
            self.desired_servo_position = self.min_servo
        else:
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
