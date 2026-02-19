#!/usr/bin/env python3
"""
Odometry TF Publisher

Publishes odom -> base_link transform from odometry messages.
This allows AMCL to work with bags that only have ground truth TF (map -> base_link).

Usage:
    ros2 run f1tenth_localization odom_tf_publisher.py
    ros2 run f1tenth_localization odom_tf_publisher.py --ros-args -p odom_topic:=/ego_racecar/odom
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTfPublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_publisher')
        
        # Parameters
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('odom_frame', 'ego_racecar/odom')
        self.declare_parameter('base_frame', 'ego_racecar/base_link')
        
        odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )
        
        self.get_logger().info(f'Publishing TF: {self.odom_frame} -> {self.base_frame}')
        self.get_logger().info(f'Subscribed to: {odom_topic}')
        
    def odom_callback(self, msg: Odometry):
        """Publish odom -> base_link transform from odometry message."""
        t = TransformStamped()
        
        # Use the odometry timestamp
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        # Copy position from odometry
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        
        # Copy orientation from odometry
        t.transform.rotation = msg.pose.pose.orientation
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        pass  # Handle external shutdown gracefully
    finally:
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
