#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster, LookupException, ConnectivityException, ExtrapolationException
import math

class TfPosePublisher(Node):
    def __init__(self):
        super().__init__('tf_pose_publisher')

        self.target_frame = 'world'
        self.source_frames = ['ego_racecar/base_link', 'base_link']

        # TF buffer + listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Keep map and world aligned for bagging/analysis pipelines that expect map.
        self.publish_static_map_to_world()

        # Publisher for pose in world frame
        self.pose_pub = self.create_publisher(PoseStamped, 'ego_pose_world', 10)

        # Timer to query TF at fixed rate
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

    def publish_static_map_to_world(self):
        yaw = 4.71
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = 'map'
        static_tf.child_frame_id = 'world'
        static_tf.transform.translation.x = -0.0133
        static_tf.transform.translation.y = -0.815
        static_tf.transform.translation.z = -0.00227
        static_tf.transform.rotation.x = 0.0
        static_tf.transform.rotation.y = 0.0
        static_tf.transform.rotation.z = math.sin(yaw / 2.0)
        static_tf.transform.rotation.w = math.cos(yaw / 2.0)

        self.static_broadcaster.sendTransform(static_tf)
        self.get_logger().info('Published static transform map -> world: x=-0.0133, y=-0.815, z=-0.00227, yaw=3.14 rad')

    def timer_callback(self):
        try:
            t = None
            for source_frame in self.source_frames:
                if self.tf_buffer.can_transform(self.target_frame, source_frame, rclpy.time.Time()):
                    t = self.tf_buffer.lookup_transform(
                        self.target_frame,
                        source_frame,
                        rclpy.time.Time()
                    )
                    break

            if t is None:
                self.get_logger().warn(
                    f'No TF available for {self.target_frame} <- any({", ".join(self.source_frames)})',
                    throttle_duration_sec=1.0,
                )
                return

            pose = PoseStamped()
            pose.header = t.header
            pose.pose.position.x = t.transform.translation.x
            pose.pose.position.y = t.transform.translation.y
            pose.pose.position.z = t.transform.translation.z
            pose.pose.orientation = t.transform.rotation

            self.pose_pub.publish(pose)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f'Could not transform {self.target_frame} -> base frame: {e}',
                throttle_duration_sec=1.0,
            )

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