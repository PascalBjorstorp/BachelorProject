#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

class TfPosePublisher(Node):
    def __init__(self):
        super().__init__('tf_pose_publisher')

        # TF buffer + listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher for pose in world frame
        self.pose_pub = self.create_publisher(PoseStamped, 'ego_pose_world', 10)

        # Timer to query TF at fixed rate
        self.timer = self.create_timer(0.005, self.timer_callback)  # 20 Hz

    def timer_callback(self):
        try:
            # Transform from world -> ego_racecar/base_link at current time
            t = self.tf_buffer.lookup_transform(
                'world',                # target frame
                'ego_racecar/base_link',# source frame
                rclpy.time.Time()       # latest available
            )

            pose = PoseStamped()
            pose.header = t.header
            pose.pose.position.x = t.transform.translation.x
            pose.pose.position.y = t.transform.translation.y
            pose.pose.position.z = t.transform.translation.z
            pose.pose.orientation = t.transform.rotation

            self.pose_pub.publish(pose)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Could not transform world -> ego_racecar/base_link: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TfPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()