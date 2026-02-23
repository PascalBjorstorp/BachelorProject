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

Algorithm:
  1. Cluster obstacle beams → find opponent centroid and width
  2. Project opponent onto nearest raceline waypoint
  3. Decide passing side (more clearance)
  4. Apply cosine-blend lateral offset to a speed-scaled window
  5. Publish shifted waypoints as /local_raceline

Performance: All hot-path computation is vectorized with NumPy.
"""

import math
import csv
import os
import numpy as np
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
        self.declare_parameter("stitch_points", 3)
        self.declare_parameter("segment_behind", 0)
        self.declare_parameter("segment_ahead", 40)
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
        self.stitch_points = self.get_parameter("stitch_points").value
        self.segment_behind = self.get_parameter("segment_behind").value
        self.segment_ahead = self.get_parameter("segment_ahead").value
        self.map_frame = self.get_parameter("map_frame").value
        self.laser_frame = self.get_parameter("laser_frame").value
        self.enabled = self.get_parameter("enabled").value

        # ── Load global raceline (precompute numpy arrays) ──────────
        traj_file = self.get_parameter("trajectory_file").value
        self.n_wp = 0
        # Columnar numpy arrays for vectorized computation
        self.wp_s: Optional[np.ndarray] = None   # (N,)
        self.wp_x: Optional[np.ndarray] = None   # (N,)
        self.wp_y: Optional[np.ndarray] = None   # (N,)
        self.wp_psi: Optional[np.ndarray] = None  # (N,)
        self.wp_kappa: Optional[np.ndarray] = None
        self.wp_vx: Optional[np.ndarray] = None
        self.wp_ax: Optional[np.ndarray] = None
        if traj_file and os.path.exists(traj_file):
            self._load_trajectory(traj_file)
        else:
            self.get_logger().warn(f"No trajectory file: '{traj_file}'")

        # ── State ───────────────────────────────────────────────────
        self.opp_detected = False
        self.opp_x = 0.0
        self.opp_y = 0.0
        self.opp_width = 0.35
        self.current_speed = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        # Cached indices for local search (avoids O(N) every frame)
        self._last_robot_idx = 0
        self._last_opp_idx = 0
        # Avoidance state machine: lock pass direction once decided
        self._avoidance_active = False
        self._locked_pass_direction = 0.0  # +1 = pass left, -1 = pass right
        self._locked_opp_s = 0.0  # arc-length position of opponent when locked
        # Temporal smoothing: ramp d_max up/down instead of jumping
        self._smooth_d_max = 0.0  # current (smoothed) shift magnitude
        self._blend_rate = 0.10   # lerp factor per frame (0=frozen, 1=instant)

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
        self.get_logger().info(f"  Raceline: {self.n_wp} waypoints")
        self.get_logger().info(f"  Safety margin: {self.safety_margin} m")
        self.get_logger().info(f"  Max shift: {self.max_lateral_shift} m")
        self.get_logger().info(f"  Rate: {rate} Hz, segment: {self.segment_ahead} ahead")

    # ────────────────────────────────────────────────────────────────
    #  Trajectory loading
    # ────────────────────────────────────────────────────────────────

    def _load_trajectory(self, path: str):
        """Load raceline CSV into columnar numpy arrays."""
        rows = []
        with open(path, "r") as f:
            reader = csv.reader(f)
            for row in reader:
                if not row or row[0].startswith("#"):
                    continue
                vals = [float(v) for v in row]
                if len(vals) >= 7:
                    rows.append(vals[:7])

        if not rows:
            return

        data = np.array(rows, dtype=np.float64)  # (N, 7)
        self.n_wp = len(data)
        self.wp_s = data[:, 0]
        self.wp_x = data[:, 1]
        self.wp_y = data[:, 2]
        self.wp_psi = data[:, 3]
        self.wp_kappa = data[:, 4]
        self.wp_vx = data[:, 5]
        self.wp_ax = data[:, 6]

        self.get_logger().info(
            f"Loaded {self.n_wp} waypoints ({self.wp_s[-1]:.1f} m)"
        )

    # ────────────────────────────────────────────────────────────────
    #  Fast closest-waypoint (local search with fallback)
    # ────────────────────────────────────────────────────────────────

    def _closest_waypoint_local(self, x: float, y: float, hint: int) -> int:
        """
        Find closest waypoint starting from `hint`, searching locally first.
        Falls back to full search if local minimum is far from actual minimum.
        Typically O(1) if the car moves incrementally between frames.
        """
        n = self.n_wp
        # Local search: check ±30 around hint
        lo = max(0, hint - 30)
        hi = min(n, hint + 30)
        dx = self.wp_x[lo:hi] - x
        dy = self.wp_y[lo:hi] - y
        dists_local = dx * dx + dy * dy
        local_best = lo + int(np.argmin(dists_local))
        local_dist = dists_local[local_best - lo]

        # If the local best is at the boundary, do a full search
        if local_best == lo or local_best == hi - 1:
            dx_all = self.wp_x - x
            dy_all = self.wp_y - y
            return int(np.argmin(dx_all * dx_all + dy_all * dy_all))

        return local_best

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

        ranges = np.array(scan.ranges, dtype=np.float32)
        valid = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)

        if not np.any(valid):
            self.opp_detected = False
            return

        n = len(ranges)
        angles = scan.angle_min + np.arange(n, dtype=np.float32) * scan.angle_increment
        world_angles = angles[valid] + laser_yaw
        r = ranges[valid]

        xs = lx + r * np.cos(world_angles)
        ys = ly + r * np.sin(world_angles)

        self.opp_x = float(np.mean(xs))
        self.opp_y = float(np.mean(ys))

        angular_extent = float(world_angles[-1] - world_angles[0]) if len(world_angles) > 1 else 0.0
        mean_range = float(np.mean(r))
        self.opp_width = max(abs(angular_extent) * mean_range, 0.15)
        self.opp_detected = True

    # ────────────────────────────────────────────────────────────────
    #  Main planning loop (vectorized)
    # ────────────────────────────────────────────────────────────────

    def _plan_loop(self):
        if not self.enabled or self.n_wp == 0:
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

        # Find robot's arc-length position on raceline
        robot_idx = self._closest_waypoint_local(
            self.robot_x, self.robot_y, self._last_robot_idx
        )
        self._last_robot_idx = robot_idx
        robot_s = self.wp_s[robot_idx]
        total_s = self.wp_s[-1]

        # ── Decide target d_max (0 when no avoidance needed) ────────
        target_d_max = 0.0
        opp_marker_visible = False

        if self.opp_detected:
            dx = self.opp_x - self.robot_x
            dy = self.opp_y - self.robot_y
            dist_to_opp = math.sqrt(dx * dx + dy * dy)
            opp_marker_visible = True

            if dist_to_opp >= self.min_replan_dist:
                # Project opponent onto raceline
                opp_idx = self._closest_waypoint_local(
                    self.opp_x, self.opp_y, self._last_opp_idx
                )
                self._last_opp_idx = opp_idx
                opp_s = self.wp_s[opp_idx]

                # Check if opponent is BEHIND the car (already passed)
                s_diff = opp_s - robot_s
                if s_diff < 0:
                    s_diff += total_s
                opp_ahead = s_diff <= total_s * 0.5

                if opp_ahead:
                    # Compute lateral offset of opponent from raceline
                    opp_psi = self.wp_psi[opp_idx]
                    opp_wp_x = self.wp_x[opp_idx]
                    opp_wp_y = self.wp_y[opp_idx]
                    normal_angle = opp_psi + math.pi / 2.0
                    cn = math.cos(normal_angle)
                    sn = math.sin(normal_angle)
                    opp_lateral = (
                        (self.opp_x - opp_wp_x) * cn +
                        (self.opp_y - opp_wp_y) * sn
                    )

                    # Lock pass direction on first detection
                    if not self._avoidance_active:
                        self._locked_pass_direction = (
                            -1.0 if opp_lateral > 0 else 1.0
                        )
                        self._avoidance_active = True
                        side = "right" if self._locked_pass_direction < 0 else "left"
                        self.get_logger().info(
                            f"Avoidance started: passing {side} "
                            f"(opp lat={opp_lateral:.2f}m, "
                            f"dist={dist_to_opp:.1f}m)"
                        )

                    # Update window center to track opponent
                    self._locked_opp_s = opp_s

                    shift_mag = min(
                        self.opp_width / 2.0 + self.safety_margin +
                        abs(opp_lateral),
                        self.max_lateral_shift,
                    )
                    target_d_max = self._locked_pass_direction * shift_mag
                else:
                    # Opponent is behind us
                    if self._avoidance_active:
                        self.get_logger().info("Avoidance ending: opponent passed")
                    self._avoidance_active = False
            else:
                # Too close — ramp down
                self._avoidance_active = False
        else:
            # No opponent
            if self._avoidance_active:
                self.get_logger().info("Avoidance ending: opponent lost")
            self._avoidance_active = False

        # ── Temporal smoothing of d_max ─────────────────────────────
        self._smooth_d_max += self._blend_rate * (target_d_max - self._smooth_d_max)
        if abs(self._smooth_d_max) < 0.005:
            self._smooth_d_max = 0.0

        # ── Publish raceline (shifted or unmodified) ────────────────
        if abs(self._smooth_d_max) < 0.001:
            # No shift — publish original raceline unmodified
            self._publish_segment_np(
                self.wp_x, self.wp_y, self.wp_psi, self.wp_vx,
            )
        else:
            # ── Avoidance offsets (asymmetric cosine blend) ─────────
            opp_s = self._locked_opp_s
            half_window = max(
                self.min_window, self.current_speed * self.window_time
            )
            ramp_up_raw = opp_s - robot_s
            if ramp_up_raw < 0:
                ramp_up_raw += total_s
            ramp_up_len = max(ramp_up_raw, self.min_window)
            ramp_down_len = half_window
            s_anchor = opp_s - ramp_up_len

            # Forward arc-length from anchor (wrapped)
            s_rel = (self.wp_s - s_anchor) % total_s

            # Ramp-up: 0 → d_max
            in_ramp_up = s_rel <= ramp_up_len
            up_progress = np.clip(s_rel / ramp_up_len, 0.0, 1.0)
            blend_up = 0.5 * (1.0 - np.cos(np.pi * up_progress))
            up_offsets = self._smooth_d_max * blend_up

            # Ramp-down: d_max → 0
            s_past_peak = s_rel - ramp_up_len
            in_ramp_down = (s_past_peak > 0) & (s_past_peak <= ramp_down_len)
            down_progress = np.clip(s_past_peak / ramp_down_len, 0.0, 1.0)
            down_offsets = self._smooth_d_max * 0.5 * (1.0 + np.cos(np.pi * down_progress))

            offsets = np.where(in_ramp_up, up_offsets, 0.0)
            offsets = np.where(in_ramp_down, down_offsets, offsets)

            # Reduce shift in high-curvature sections (corners)
            curvature_scale = 1.0 / (1.0 + 5.0 * np.abs(self.wp_kappa))
            offsets *= curvature_scale

            # Apply offsets perpendicular to raceline
            full_x = self.wp_x.copy()
            full_y = self.wp_y.copy()
            normal_angles = self.wp_psi + (np.pi / 2.0)
            full_x += offsets * np.cos(normal_angles)
            full_y += offsets * np.sin(normal_angles)

            self._publish_segment_np(full_x, full_y, self.wp_psi, self.wp_vx)

        self._publish_opponent_marker(opp_marker_visible)

    # ────────────────────────────────────────────────────────────────
    #  Publishing (vectorized)
    # ────────────────────────────────────────────────────────────────

    def _publish_segment_np(
        self,
        xs: np.ndarray,
        ys: np.ndarray,
        psis: np.ndarray,
        vxs: np.ndarray,
    ):
        """Build and publish Path from numpy arrays."""
        path = Path()
        now_stamp = self.get_clock().now().to_msg()
        path.header.stamp = now_stamp
        path.header.frame_id = self.map_frame

        # Pre-compute quaternion components
        half_psi = psis * 0.5
        qz = np.sin(half_psi)
        qw = np.cos(half_psi)

        # Build all poses
        n = len(xs)
        poses = []
        for i in range(n):
            pose = PoseStamped()
            pose.header.stamp = now_stamp
            pose.header.frame_id = self.map_frame
            pose.pose.position.x = float(xs[i])
            pose.pose.position.y = float(ys[i])
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = float(vxs[i])  # velocity encoding
            pose.pose.orientation.z = float(qz[i])
            pose.pose.orientation.w = float(qw[i])
            poses.append(pose)

        path.poses = poses
        self.raceline_pub.publish(path)

    def _publish_opponent_marker(self, detected: bool):
        markers = MarkerArray()
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.map_frame
        marker.ns = "opponent"
        marker.id = 0
        marker.type = Marker.CYLINDER

        if detected:
            marker.action = Marker.ADD
            marker.pose.position.x = self.opp_x
            marker.pose.position.y = self.opp_y
            marker.pose.position.z = 0.1
            marker.scale.x = self.opp_width
            marker.scale.y = self.opp_width
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
        """Full O(N) fallback search."""
        if self.wp_x is None:
            return 0
        dx = self.wp_x - x
        dy = self.wp_y - y
        return int(np.argmin(dx * dx + dy * dy))


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
