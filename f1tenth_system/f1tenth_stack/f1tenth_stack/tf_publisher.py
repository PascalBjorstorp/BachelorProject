# Copyright (c) 2020 Hongrui Zheng
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


class FramePublisher(Node):
    """
    Publishes static transforms for the F1TENTH car.

    This node broadcasts transforms from base_link to sensor frames
    like laser and IMU. Uses StaticTransformBroadcaster for efficiency
    since these transforms don't change at runtime.
    """

    def __init__(self):
        super().__init__('f1tenth_tf_publisher')

        # Declare parameters for transform positions
        self.declare_parameter('laser_x', 0.27)
        self.declare_parameter('laser_y', 0.0)
        self.declare_parameter('laser_z', 0.11)
        self.declare_parameter('imu_x', 0.0)
        self.declare_parameter('imu_y', 0.0)
        self.declare_parameter('imu_z', 0.0)

        # Get parameters
        laser_x = self.get_parameter('laser_x').value
        laser_y = self.get_parameter('laser_y').value
        laser_z = self.get_parameter('laser_z').value
        imu_x = self.get_parameter('imu_x').value
        imu_y = self.get_parameter('imu_y').value
        imu_z = self.get_parameter('imu_z').value

        # Use StaticTransformBroadcaster - publishes once with latched QoS
        self.br = StaticTransformBroadcaster(self)

        # Build transforms once
        transforms = []

        # base_link -> laser transform
        t_laser = TransformStamped()
        t_laser.header.stamp = self.get_clock().now().to_msg()
        t_laser.header.frame_id = 'base_link'
        t_laser.child_frame_id = 'laser'
        t_laser.transform.translation.x = laser_x
        t_laser.transform.translation.y = laser_y
        t_laser.transform.translation.z = laser_z
        t_laser.transform.rotation.x = 0.0
        t_laser.transform.rotation.y = 0.0
        t_laser.transform.rotation.z = 0.0
        t_laser.transform.rotation.w = 1.0
        transforms.append(t_laser)

        # base_link -> imu transform
        t_imu = TransformStamped()
        t_imu.header.stamp = self.get_clock().now().to_msg()
        t_imu.header.frame_id = 'base_link'
        t_imu.child_frame_id = 'imu'
        t_imu.transform.translation.x = imu_x
        t_imu.transform.translation.y = imu_y
        t_imu.transform.translation.z = imu_z
        t_imu.transform.rotation.x = 0.0
        t_imu.transform.rotation.y = 0.0
        t_imu.transform.rotation.z = 0.0
        t_imu.transform.rotation.w = 1.0
        transforms.append(t_imu)

        # Publish all transforms once (latched, subscribers get them on connect)
        self.br.sendTransform(transforms)

        self.get_logger().info(
            f'Static TF published. Laser at ({laser_x}, {laser_y}, {laser_z})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
