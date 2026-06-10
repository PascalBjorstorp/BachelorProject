#!/usr/bin/env python3
"""Publish simulated odometry with OptiTrack-calibrated local drift.

The node takes ground-truth simulator odometry and publishes a dead-reckoned
odom stream whose local SE(2) increments are perturbed to match measured
short-horizon physical-vehicle odometry drift.
"""

import math
import random
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(q: Quaternion) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    half = 0.5 * yaw
    q.w = math.cos(half)
    q.z = math.sin(half)
    return q


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def pose_delta_body(
    prev: Tuple[float, float, float],
    current: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    dx_world = current[0] - prev[0]
    dy_world = current[1] - prev[1]
    c = math.cos(prev[2])
    s = math.sin(prev[2])
    dx_body = c * dx_world + s * dy_world
    dy_body = -s * dx_world + c * dy_world
    return dx_body, dy_body, wrap_angle(current[2] - prev[2])


class CalibratedDriftOdom(Node):
    def __init__(self) -> None:
        super().__init__("calibrated_drift_odom")

        self.declare_parameter("groundtruth_topic", "/ego_racecar/ground_truth")
        self.declare_parameter("odom_topic", "/ego_racecar/odom")
        self.declare_parameter("odom_frame", "ego_racecar/odom")
        self.declare_parameter("base_frame", "ego_racecar/base_link")
        self.declare_parameter("publish_tf", True)

        self.declare_parameter("reference_interval_s", 0.025)
        self.declare_parameter("position_p95_m", 0.036)
        self.declare_parameter("yaw_median_deg", 0.24)
        self.declare_parameter("yaw_p95_deg", 1.8)
        self.declare_parameter("yaw_tail_probability", 0.10)
        self.declare_parameter("covariance_scale", 1.0)
        self.declare_parameter("random_seed", 23)

        self.groundtruth_topic = self.get_parameter("groundtruth_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.reference_interval_s = max(
            float(self.get_parameter("reference_interval_s").value), 1.0e-6)
        self.position_p95_m = max(float(self.get_parameter("position_p95_m").value), 0.0)
        self.yaw_median_rad = math.radians(
            max(float(self.get_parameter("yaw_median_deg").value), 0.0))
        self.yaw_p95_rad = math.radians(
            max(float(self.get_parameter("yaw_p95_deg").value), 0.0))
        self.yaw_tail_probability = min(
            max(float(self.get_parameter("yaw_tail_probability").value), 0.0), 0.95)
        self.covariance_scale = max(
            float(self.get_parameter("covariance_scale").value), 0.0)
        self.rng = random.Random(int(self.get_parameter("random_seed").value))

        # For a 2D isotropic Gaussian, radial p95 = sigma * sqrt(-2 ln(0.05)).
        self.position_sigma_ref = (
            self.position_p95_m / math.sqrt(-2.0 * math.log(0.05))
            if self.position_p95_m > 0.0 else 0.0
        )
        # The measured yaw table has a heavier tail than a single Gaussian.
        # Use the median for the normal case and a small heavy-tail component
        # whose median absolute value is approximately the p95 target.
        self.yaw_base_sigma_ref = (
            self.yaw_median_rad / 0.67448975 if self.yaw_median_rad > 0.0 else 0.0
        )
        self.yaw_tail_sigma_ref = (
            self.yaw_p95_rad / 0.67448975 if self.yaw_p95_rad > 0.0 else 0.0
        )

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster: Optional[TransformBroadcaster] = (
            TransformBroadcaster(self) if self.publish_tf else None
        )
        self.gt_sub = self.create_subscription(
            Odometry, self.groundtruth_topic, self._on_groundtruth, 10)

        self.prev_gt_pose: Optional[Tuple[float, float, float]] = None
        self.prev_stamp_ns: Optional[int] = None
        self.odom_pose: Optional[Tuple[float, float, float]] = None

        self.get_logger().info(
            f"Calibrated drift odom: {self.groundtruth_topic} -> {self.odom_topic}, "
            f"ref {self.reference_interval_s:.3f} s, pos p95 {self.position_p95_m:.3f} m, "
            f"yaw median/p95 {math.degrees(self.yaw_median_rad):.2f}/"
            f"{math.degrees(self.yaw_p95_rad):.2f} deg, "
            f"cov scale {self.covariance_scale:.1f}"
        )

    def _stamp_ns(self, msg: Odometry) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def _sample_yaw_noise(self, scale: float) -> float:
        if self.yaw_tail_probability > 0.0 and self.rng.random() < self.yaw_tail_probability:
            sigma = self.yaw_tail_sigma_ref
        else:
            sigma = self.yaw_base_sigma_ref
        return self.rng.gauss(0.0, sigma * scale)

    def _on_groundtruth(self, msg: Odometry) -> None:
        gt_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_from_quaternion(msg.pose.pose.orientation),
        )
        stamp_ns = self._stamp_ns(msg)

        if self.prev_gt_pose is None or self.odom_pose is None or self.prev_stamp_ns is None:
            self.prev_gt_pose = gt_pose
            self.prev_stamp_ns = stamp_ns
            self.odom_pose = gt_pose
            self._publish(msg)
            return

        dt_s = max((stamp_ns - self.prev_stamp_ns) * 1.0e-9, 0.0)
        scale = math.sqrt(dt_s / self.reference_interval_s) if dt_s > 0.0 else 0.0

        dx_body, dy_body, dtheta = pose_delta_body(self.prev_gt_pose, gt_pose)
        dx_body += self.rng.gauss(0.0, self.position_sigma_ref * scale)
        dy_body += self.rng.gauss(0.0, self.position_sigma_ref * scale)
        dtheta = wrap_angle(dtheta + self._sample_yaw_noise(scale))

        x, y, yaw = self.odom_pose
        c = math.cos(yaw)
        s = math.sin(yaw)
        x += c * dx_body - s * dy_body
        y += s * dx_body + c * dy_body
        yaw = wrap_angle(yaw + dtheta)
        self.odom_pose = (x, y, yaw)

        self.prev_gt_pose = gt_pose
        self.prev_stamp_ns = stamp_ns
        self._publish(msg)

    def _publish(self, gt_msg: Odometry) -> None:
        assert self.odom_pose is not None
        x, y, yaw = self.odom_pose

        out = Odometry()
        out.header.stamp = gt_msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.orientation = quaternion_from_yaw(yaw)
        out.twist = gt_msg.twist

        cov_xy = self.covariance_scale * (0.5 * self.position_p95_m) ** 2
        cov_yaw = self.covariance_scale * (0.5 * self.yaw_p95_rad) ** 2
        out.pose.covariance[0] = cov_xy
        out.pose.covariance[7] = cov_xy
        out.pose.covariance[35] = cov_yaw
        out.twist.covariance[0] = cov_xy
        out.twist.covariance[7] = cov_xy
        out.twist.covariance[35] = cov_yaw

        self.odom_pub.publish(out)

        if self.tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header = out.header
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = x
            tf_msg.transform.translation.y = y
            tf_msg.transform.rotation = out.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)


def main() -> None:
    rclpy.init()
    node = CalibratedDriftOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
