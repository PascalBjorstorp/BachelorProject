# MIT License
# Copyright (c) 2020 Hongrui Zheng

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class FramePublisher(Node):
    """
    Publishes static transforms for the F1TENTH car.
    
    This node broadcasts transforms from base_link to sensor frames
    like laser and IMU.
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
        self.declare_parameter('publish_rate', 100.0)  # Hz
        
        # Get parameters
        self.laser_x = self.get_parameter('laser_x').value
        self.laser_y = self.get_parameter('laser_y').value
        self.laser_z = self.get_parameter('laser_z').value
        self.imu_x = self.get_parameter('imu_x').value
        self.imu_y = self.get_parameter('imu_y').value
        self.imu_z = self.get_parameter('imu_z').value
        publish_rate = self.get_parameter('publish_rate').value

        # Transform broadcaster
        self.br = TransformBroadcaster(self)
        
        # Timer for publishing transforms
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        
        self.get_logger().info(
            f'TF publisher started. Laser at ({self.laser_x}, {self.laser_y}, {self.laser_z})'
        )

    def timer_callback(self):
        """Publish all transforms."""
        now = self.get_clock().now().to_msg()
        
        # base_link -> laser transform
        t_laser = TransformStamped()
        t_laser.header.stamp = now
        t_laser.header.frame_id = 'base_link'
        t_laser.child_frame_id = 'laser'
        t_laser.transform.translation.x = self.laser_x
        t_laser.transform.translation.y = self.laser_y
        t_laser.transform.translation.z = self.laser_z
        t_laser.transform.rotation.x = 0.0
        t_laser.transform.rotation.y = 0.0
        t_laser.transform.rotation.z = 0.0
        t_laser.transform.rotation.w = 1.0
        self.br.sendTransform(t_laser)
        
        # base_link -> imu transform
        t_imu = TransformStamped()
        t_imu.header.stamp = now
        t_imu.header.frame_id = 'base_link'
        t_imu.child_frame_id = 'imu'
        t_imu.transform.translation.x = self.imu_x
        t_imu.transform.translation.y = self.imu_y
        t_imu.transform.translation.z = self.imu_z
        t_imu.transform.rotation.x = 0.0
        t_imu.transform.rotation.y = 0.0
        t_imu.transform.rotation.z = 0.0
        t_imu.transform.rotation.w = 1.0
        self.br.sendTransform(t_imu)


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
