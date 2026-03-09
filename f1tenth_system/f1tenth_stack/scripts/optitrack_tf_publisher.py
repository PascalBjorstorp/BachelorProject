#!/usr/bin/env python3
"""
OptiTrack Ground Truth Publisher for F1TENTH Localization Benchmarking.

Subscribes to the VRPN/OptiTrack pose topic (``PoseStamped`` in ``world``
frame) and publishes:

1. **Ground truth odometry** on ``/ego_racecar/ground_truth``
   (``nav_msgs/Odometry`` in ``map`` frame) — compatible with the existing
   ``localization_benchmark.py`` node.

2. **TF transform** ``world → map`` (static, from calibrated offset) so both
   coordinate systems are connected for RViz visualisation.

The node converts OptiTrack poses from the ``world`` frame to the ``map``
frame using user-supplied calibration offsets, and estimates linear/angular
velocity via finite differences.

Calibration
-----------
Place the car at a known position.  Read the OptiTrack pose and the
``map → base_link`` TF.  The offset is::

    offset_x   = optitrack_x  (with map position at origin)
    offset_y   = optitrack_y
    offset_yaw = -(car_yaw_in_map)

These offsets describe the position & orientation of the ``map`` origin
expressed in the ``world`` frame.

Usage
-----
::

    python3 optitrack_tf_publisher.py

    python3 optitrack_tf_publisher.py --ros-args \\
        -p vrpn_topic:=/vrpn_mocap/car_pos/pose \\
        -p offset_x:=1.291 \\
        -p offset_y:=-1.316 \\
        -p offset_yaw:=-0.108
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


def quaternion_to_yaw(q):
    """Extract yaw from a quaternion with attributes x, y, z, w."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw):
    """Return (x, y, z, w) for a pure-yaw rotation."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def transform_world_to_map(wx, wy, wyaw, off_x, off_y, off_yaw):
    """
    Convert a pose in the ``world`` frame to the ``map`` frame.

    The calibration offset (off_x, off_y, off_yaw) describes where the
    ``map`` origin sits inside the ``world`` frame.

    T_map_point = inv(T_world_map) * T_world_point
    """
    # Translate then rotate by -off_yaw
    dx = wx - off_x
    dy = wy - off_y
    cos_neg = math.cos(-off_yaw)
    sin_neg = math.sin(-off_yaw)
    mx = cos_neg * dx - sin_neg * dy
    my = sin_neg * dx + cos_neg * dy
    myaw = wyaw - off_yaw
    return mx, my, myaw


class OptiTrackGroundTruth(Node):
    def __init__(self):
        super().__init__('optitrack_ground_truth')

        # ── Parameters ──
        self.declare_parameter('vrpn_topic', '/vrpn_mocap/car_pos/pose')
        self.declare_parameter('ground_truth_topic', '/ego_racecar/ground_truth')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'ego_racecar/base_link')

        # Calibrated world → map offset (position of map origin in world frame)
        self.declare_parameter('offset_x', 1.291)
        self.declare_parameter('offset_y', -1.316)
        self.declare_parameter('offset_yaw', -0.108)

        self.vrpn_topic = self.get_parameter('vrpn_topic').value
        self.gt_topic = self.get_parameter('ground_truth_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.off_x = self.get_parameter('offset_x').value
        self.off_y = self.get_parameter('offset_y').value
        self.off_yaw = self.get_parameter('offset_yaw').value

        # ── Publishers ──
        self.gt_pub = self.create_publisher(Odometry, self.gt_topic, 10)

        # Static TF: world → map (so both trees are visible in RViz)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_world_to_map()

        # ── Subscriber ──
        vrpn_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.create_subscription(
            PoseStamped, self.vrpn_topic, self._vrpn_cb, vrpn_qos
        )

        # ── Velocity estimation state ──
        self._prev_x = None
        self._prev_y = None
        self._prev_yaw = None
        self._prev_stamp = None

        self.get_logger().info(
            f'OptiTrack ground truth publisher started\n'
            f'  VRPN topic  : {self.vrpn_topic}\n'
            f'  GT topic    : {self.gt_topic}\n'
            f'  Offset      : x={self.off_x:.3f}  y={self.off_y:.3f}  '
            f'yaw={self.off_yaw:.3f} rad\n'
            f'  Static TF   : world → {self.map_frame}'
        )

    # ──────────────────────────────────────────────
    def _publish_static_world_to_map(self):
        """Broadcast the calibrated world → map static transform."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.map_frame
        t.transform.translation.x = self.off_x
        t.transform.translation.y = self.off_y
        t.transform.translation.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(self.off_yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.static_tf_broadcaster.sendTransform(t)

    # ──────────────────────────────────────────────
    def _vrpn_cb(self, msg: PoseStamped):
        """Transform OptiTrack pose to map frame and publish as Odometry."""
        # ── Pose in world frame ──
        wx = msg.pose.position.x
        wy = msg.pose.position.y
        wyaw = quaternion_to_yaw(msg.pose.orientation)

        # ── Convert to map frame ──
        mx, my, myaw = transform_world_to_map(
            wx, wy, wyaw, self.off_x, self.off_y, self.off_yaw
        )

        # ── Estimate velocity via finite differences ──
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        vx = vy = omega = 0.0
        if self._prev_stamp is not None:
            dt = stamp_sec - self._prev_stamp
            if dt > 1e-6:
                # World-frame velocity components
                dx = mx - self._prev_x
                dy = my - self._prev_y

                # Body-frame velocity (project onto heading)
                cos_yaw = math.cos(myaw)
                sin_yaw = math.sin(myaw)
                vx = cos_yaw * dx / dt + sin_yaw * dy / dt
                vy = -sin_yaw * dx / dt + cos_yaw * dy / dt

                # Angular velocity
                dyaw = myaw - self._prev_yaw
                # Wrap to [-π, π]
                dyaw = (dyaw + math.pi) % (2.0 * math.pi) - math.pi
                omega = dyaw / dt

        self._prev_x = mx
        self._prev_y = my
        self._prev_yaw = myaw
        self._prev_stamp = stamp_sec

        # ── Build Odometry message ──
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = mx
        odom.pose.pose.position.y = my
        odom.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(myaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # OptiTrack is very precise — small covariance
        odom.pose.covariance[0] = 0.001   # x
        odom.pose.covariance[7] = 0.001   # y
        odom.pose.covariance[35] = 0.001  # yaw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega

        self.gt_pub.publish(odom)


def main():
    rclpy.init()
    node = OptiTrackGroundTruth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
