#!/usr/bin/env python3
"""
Lateral Planner Node — Opponent avoidance via raceline modification.

Subscribes to:
  /scan_obstacles  — obstacle-only beams (from scan splitter)
  /odom            — vehicle odometry (for current speed)
  TF map→base_link — robot pose in map frame

Loads:
  Global raceline CSV (same format as Pure Pursuit / Stanley)

Publishes:
  /local_raceline     — nav_msgs/Path with modified trajectory segment
  /opponent_marker    — visualization_msgs/MarkerArray for RViz

The node detects the opponent from obstacle beams, computes a smooth
lateral shift to avoid it, and publishes the modified raceline. The
downstream controllers (Pure Pursuit, Stanley, MPC) track the modified
path without needing obstacle awareness.

Algorithm:
  1. Cluster obstacle beams → find opponent centroid and width
  2. Project opponent onto nearest raceline waypoint
  3. Decide passing side (more clearance)
  4. Apply cosine-blend lateral offset to a speed-scaled window
  5. Publish shifted waypoints as /local_raceline
"""

import math
import csv
import os
import numpy as np
from dataclasses import dataclass, field
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool, ColorRGBA
import tf2_ros


# ════════════════════════════════════════════════════════════════════════
#  Data types
# ════════════════════════════════════════════════════════════════════════

@dataclass
class Waypoint:
    """Single raceline waypoint matching the CSV columns."""
    s: float = 0.0       # arc length [m]
    x: float = 0.0       # [m]
    y: float = 0.0       # [m]
    psi: float = 0.0     # heading [rad]
    kappa: float = 0.0   # curvature [1/m]
    vx: float = 0.0      # velocity [m/s]
    ax: float = 0.0      # acceleration [m/s²]


@dataclass
class OpponentState:
    """Detected opponent position and geometry."""
    x: float = 0.0       # centroid in map frame [m]
    y: float = 0.0
    width: float = 0.3   # estimated width [m]
    detected: bool = False


# ════════════════════════════════════════════════════════════════════════
#  Lateral Planner Node
# ════════════════════════════════════════════════════════════════════════

class LateralPlannerNode(Node):
    """Generates locally shifted racelines to avoid a detected opponent."""

    def __init__(self):
        super().__init__("lateral_planner_node")

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter("trajectory_file", "")
        self.declare_parameter("safety_margin_m", 0.3)
        self.declare_parameter("min_window_m", 3.0)
        self.declare_parameter("window_time_s", 0.8)
        self.declare_parameter("max_lateral_shift_m", 0.8)
        self.declare_parameter("min_replan_dist_m", 1.0)
        self.declare_parameter("publish_rate_hz", 40.0)
        self.declare_parameter("enabled", True)

        # Frame IDs
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("laser_frame", "ego_racecar/laser")

        # Topics
        self.declare_parameter("obstacles_topic", "/scan_obstacles")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("raceline_topic", "/local_raceline")
        self.declare_parameter("enable_topic", "/lateral_planner_enable")

        self.safety_margin = self.get_parameter("safety_margin_m").value
        self.min_window = self.get_parameter("min_window_m").value
        self.window_time = self.get_parameter("window_time_s").value
        self.max_lateral_shift = self.get_parameter("max_lateral_shift_m").value
        self.min_replan_dist = self.get_parameter("min_replan_dist_m").value
        self.map_frame = self.get_parameter("map_frame").value
        self.laser_frame = self.get_parameter("laser_frame").value
        self.enabled = self.get_parameter("enabled").value

        # ── Load global raceline ────────────────────────────────────
        traj_file = self.get_parameter("trajectory_file").value
        self.waypoints: list[Waypoint] = []
        self.wp_xy: Optional[np.ndarray] = None  # (N, 2) for fast lookup
        if traj_file and os.path.exists(traj_file):
            self._load_trajectory(traj_file)
        else:
            self.get_logger().warn(f"No trajectory file: '{traj_file}'")

        # ── State ───────────────────────────────────────────────────
        self.opponent = OpponentState()
        self.current_speed = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # ── TF ──────────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Subscribers ─────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        obstacles_topic = self.get_parameter("obstacles_topic").value
        self.obstacle_sub = self.create_subscription(
            LaserScan, obstacles_topic, self._obstacle_callback, sensor_qos
        )

        odom_topic = self.get_parameter("odom_topic").value
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self._odom_callback, 10
        )

        enable_topic = self.get_parameter("enable_topic").value
        self.enable_sub = self.create_subscription(
            Bool, enable_topic, self._enable_callback, 10
        )

        # ── Publishers ──────────────────────────────────────────────
        raceline_topic = self.get_parameter("raceline_topic").value
        self.raceline_pub = self.create_publisher(Path, raceline_topic, 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/opponent_marker", 10
        )

        # ── Main loop timer ─────────────────────────────────────────
        rate = self.get_parameter("publish_rate_hz").value
        period = 1.0 / rate
        self.timer = self.create_timer(period, self._plan_loop)

        self.get_logger().info("Lateral Planner Node initialized")
        self.get_logger().info(f"  Raceline: {len(self.waypoints)} waypoints")
        self.get_logger().info(f"  Safety margin: {self.safety_margin} m")
        self.get_logger().info(f"  Max shift: {self.max_lateral_shift} m")

    # ────────────────────────────────────────────────────────────────
    #  Trajectory loading
    # ────────────────────────────────────────────────────────────────

    def _load_trajectory(self, path: str):
        """Load raceline CSV: s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2."""
        self.waypoints.clear()
        with open(path, "r") as f:
            reader = csv.reader(f)
            for row in reader:
                if not row or row[0].startswith("#"):
                    continue
                vals = [float(v) for v in row]
                if len(vals) >= 7:
                    self.waypoints.append(Waypoint(
                        s=vals[0], x=vals[1], y=vals[2],
                        psi=vals[3], kappa=vals[4],
                        vx=vals[5], ax=vals[6],
                    ))
        if self.waypoints:
            self.wp_xy = np.array(
                [[wp.x, wp.y] for wp in self.waypoints], dtype=np.float64
            )
            self.get_logger().info(
                f"Loaded {len(self.waypoints)} waypoints "
                f"({self.waypoints[-1].s:.1f} m)"
            )

    # ────────────────────────────────────────────────────────────────
    #  Callbacks
    # ────────────────────────────────────────────────────────────────

    def _odom_callback(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = math.sqrt(vx * vx + vy * vy)

    def _enable_callback(self, msg: Bool):
        self.enabled = msg.data
        self.get_logger().info(f"Lateral planner {'enabled' if self.enabled else 'disabled'}")

    def _obstacle_callback(self, scan: LaserScan):
        """Extract opponent centroid and width from obstacle-only scan."""
        # Get laser pose in map frame
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.laser_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.02),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        lx = tf.transform.translation.x
        ly = tf.transform.translation.y
        q = tf.transform.rotation
        laser_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        # Collect valid obstacle beam endpoints
        ranges = np.array(scan.ranges, dtype=np.float32)
        n = len(ranges)
        valid = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)

        if not np.any(valid):
            self.opponent.detected = False
            return

        angles = scan.angle_min + np.arange(n, dtype=np.float32) * scan.angle_increment
        world_angles = angles[valid] + laser_yaw
        r = ranges[valid]

        xs = lx + r * np.cos(world_angles)
        ys = ly + r * np.sin(world_angles)

        # Compute centroid
        cx, cy = float(np.mean(xs)), float(np.mean(ys))

        # Estimate width from angular extent × mean range
        angular_extent = float(world_angles[-1] - world_angles[0]) if len(world_angles) > 1 else 0.0
        mean_range = float(np.mean(r))
        width = abs(angular_extent) * mean_range
        width = max(width, 0.15)  # minimum physical width

        self.opponent = OpponentState(
            x=cx, y=cy, width=width, detected=True
        )

    # ────────────────────────────────────────────────────────────────
    #  Main planning loop
    # ────────────────────────────────────────────────────────────────

    def _plan_loop(self):
        """Compute and publish the (possibly shifted) local raceline."""
        if not self.enabled or not self.waypoints:
            return

        # Update robot pose from TF
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, "ego_racecar/base_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.02),
            )
            self.robot_x = tf.transform.translation.x
            self.robot_y = tf.transform.translation.y
            q = tf.transform.rotation
            self.robot_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        if not self.opponent.detected:
            # No opponent — publish unmodified raceline segment ahead
            self._publish_raceline_segment()
            self._publish_opponent_marker(False)
            return

        # ── Opponent detected: compute avoidance ────────────────────
        opp = self.opponent

        # Distance from robot to opponent
        dx = opp.x - self.robot_x
        dy = opp.y - self.robot_y
        dist_to_opp = math.sqrt(dx * dx + dy * dy)

        # Too close for replanning — don't shift (emergency handled by controller)
        if dist_to_opp < self.min_replan_dist:
            self._publish_raceline_segment()
            self._publish_opponent_marker(True)
            return

        # Project opponent onto raceline
        opp_idx = self._closest_waypoint(opp.x, opp.y)
        opp_wp = self.waypoints[opp_idx]

        # Compute lateral offset of opponent from raceline
        # Positive = left of heading, negative = right
        dx_opp = opp.x - opp_wp.x
        dy_opp = opp.y - opp_wp.y
        normal_angle = opp_wp.psi + math.pi / 2
        opp_lateral = dx_opp * math.cos(normal_angle) + dy_opp * math.sin(normal_angle)

        # Decide passing side: pass on opposite side from where opponent is
        # (i.e., if opponent is left of raceline, pass on the right)
        pass_direction = -1.0 if opp_lateral > 0 else 1.0

        # Shift magnitude: clear the opponent + safety margin
        shift_magnitude = opp.width / 2.0 + self.safety_margin + abs(opp_lateral)
        shift_magnitude = min(shift_magnitude, self.max_lateral_shift)
        d_max = pass_direction * shift_magnitude

        # Window length: scale with speed
        half_window = max(self.min_window, self.current_speed * self.window_time)

        # Find waypoint indices for the window
        s_opp = opp_wp.s
        s_start = s_opp - half_window
        s_end = s_opp + half_window

        # Apply cosine-blend lateral offset
        shifted_waypoints = self._apply_lateral_shift(s_start, s_end, d_max)

        # Publish
        self._publish_path(shifted_waypoints)
        self._publish_opponent_marker(True)

    # ────────────────────────────────────────────────────────────────
    #  Lateral shift
    # ────────────────────────────────────────────────────────────────

    def _apply_lateral_shift(
        self, s_start: float, s_end: float, d_max: float
    ) -> list[Waypoint]:
        """
        Shift raceline waypoints within [s_start, s_end] by a cosine-blended
        lateral offset, returning the modified waypoints for a segment around
        the robot.
        """
        n = len(self.waypoints)
        total_s = self.waypoints[-1].s
        window_len = s_end - s_start

        # Determine segment to publish (generous range around robot)
        robot_idx = self._closest_waypoint(self.robot_x, self.robot_y)
        segment_behind = 5      # waypoints behind robot
        segment_ahead = 80      # waypoints ahead
        idx_start = max(0, robot_idx - segment_behind)
        idx_end = min(n, robot_idx + segment_ahead)

        result = []
        for i in range(idx_start, idx_end):
            wp = self.waypoints[i]
            s = wp.s

            # Compute offset using cosine blend
            offset = 0.0
            if window_len > 0:
                # Handle wraparound for closed tracks
                s_rel = s - s_start
                if s_rel < 0:
                    s_rel += total_s
                if 0 <= s_rel <= window_len:
                    offset = d_max * 0.5 * (1.0 - math.cos(2.0 * math.pi * s_rel / window_len))

            # Shift perpendicular to heading
            normal_angle = wp.psi + math.pi / 2
            new_x = wp.x + offset * math.cos(normal_angle)
            new_y = wp.y + offset * math.sin(normal_angle)

            # Recompute heading from shifted positions (numerical derivative)
            # For simplicity in the skeleton, keep original heading.
            # TODO: Recompute heading and curvature analytically from offset profile.
            new_psi = wp.psi
            new_kappa = wp.kappa

            # Optionally reduce velocity in shifted section
            speed_scale = 1.0 - 0.2 * abs(offset / self.max_lateral_shift) if self.max_lateral_shift > 0 else 1.0
            new_vx = wp.vx * speed_scale

            result.append(Waypoint(
                s=wp.s, x=new_x, y=new_y,
                psi=new_psi, kappa=new_kappa,
                vx=new_vx, ax=wp.ax,
            ))

        return result

    # ────────────────────────────────────────────────────────────────
    #  Publishing
    # ────────────────────────────────────────────────────────────────

    def _publish_raceline_segment(self):
        """Publish unmodified raceline segment around the robot."""
        if not self.waypoints:
            return
        n = len(self.waypoints)
        robot_idx = self._closest_waypoint(self.robot_x, self.robot_y)
        idx_start = max(0, robot_idx - 5)
        idx_end = min(n, robot_idx + 80)
        segment = self.waypoints[idx_start:idx_end]
        self._publish_path(segment)

    def _publish_path(self, waypoints: list[Waypoint]):
        """Publish waypoints as a nav_msgs/Path."""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.map_frame

        for wp in waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = wp.x
            pose.pose.position.y = wp.y
            pose.pose.position.z = wp.vx  # encode velocity in z (pragmatic hack)
            # Encode heading as quaternion
            pose.pose.orientation.z = math.sin(wp.psi / 2.0)
            pose.pose.orientation.w = math.cos(wp.psi / 2.0)
            path.poses.append(pose)

        self.raceline_pub.publish(path)

    def _publish_opponent_marker(self, detected: bool):
        """Publish opponent visualization marker for RViz."""
        markers = MarkerArray()
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.map_frame
        marker.ns = "opponent"
        marker.id = 0
        marker.type = Marker.CYLINDER

        if detected:
            marker.action = Marker.ADD
            marker.pose.position.x = self.opponent.x
            marker.pose.position.y = self.opponent.y
            marker.pose.position.z = 0.1
            marker.scale.x = self.opponent.width
            marker.scale.y = self.opponent.width
            marker.scale.z = 0.2
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
        else:
            marker.action = Marker.DELETE

        markers.markers.append(marker)
        self.marker_pub.publish(markers)

    # ────────────────────────────────────────────────────────────────
    #  Helpers
    # ────────────────────────────────────────────────────────────────

    def _closest_waypoint(self, x: float, y: float) -> int:
        """Find the index of the closest raceline waypoint to (x, y)."""
        if self.wp_xy is None:
            return 0
        dists = (self.wp_xy[:, 0] - x) ** 2 + (self.wp_xy[:, 1] - y) ** 2
        return int(np.argmin(dists))


def main(args=None):
    rclpy.init(args=args)
    node = LateralPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
