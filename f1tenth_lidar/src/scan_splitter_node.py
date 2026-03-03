#!/usr/bin/env python3
"""
Scan Splitter Node — Classifies LiDAR beams as wall or obstacle.

Subscribes to raw /scan and the static occupancy grid map, then for each
beam checks whether the endpoint lies near a known wall (using a precomputed
distance field). Beams that hit something NOT in the map are classified as
obstacle beams (likely the opponent car).

Publishes two LaserScan topics:
  /scan_walls     — obstacle beams replaced with inf (for AMCL)
  /scan_obstacles — wall beams replaced with inf (for lateral planner)

Requires:
  - A static map served on /map (nav2_map_server or equivalent)
  - Robot pose available via TF (map → base_link / laser frame)
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import tf2_ros
from scipy.ndimage import distance_transform_edt


class ScanSplitterNode(Node):
    """Splits raw LiDAR scan into wall beams and obstacle beams."""

    def __init__(self):
        super().__init__("scan_splitter_node")

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter("enable_splitting", True)
        self.declare_parameter("obstacle_threshold_m", 0.3)
        self.declare_parameter("min_cluster_size", 3)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("walls_topic", "/scan_walls")
        self.declare_parameter("obstacles_topic", "/scan_obstacles")
        self.declare_parameter("robot_frame", "ego_racecar/base_link")
        self.declare_parameter("laser_frame", "ego_racecar/laser")
        self.declare_parameter("map_frame", "map")

        # Handle enable_splitting — may arrive as string from launch
        _es = self.get_parameter("enable_splitting").value
        if isinstance(_es, str):
            self.enable_splitting = _es.lower() in ('true', '1', 'yes')
        else:
            self.enable_splitting = bool(_es)

        self.obstacle_threshold = self.get_parameter("obstacle_threshold_m").value
        self.min_cluster_size = self.get_parameter("min_cluster_size").value
        scan_topic = self.get_parameter("scan_topic").value
        walls_topic = self.get_parameter("walls_topic").value
        obstacles_topic = self.get_parameter("obstacles_topic").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.laser_frame = self.get_parameter("laser_frame").value
        self.map_frame = self.get_parameter("map_frame").value

        # ── Map state ───────────────────────────────────────────────────
        self.map_ready = False
        self.distance_field = None  # 2D array, meters to nearest wall
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_resolution = 0.05
        self.map_width = 0
        self.map_height = 0

        # ── TF ──────────────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Subscribers ─────────────────────────────────────────────────
        # Map: latched / transient local
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self._map_callback, map_qos
        )

        # Scan: best-effort sensor QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._scan_callback, sensor_qos
        )

        # ── Publishers (RELIABLE so RViz / AMCL can subscribe) ────────
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        self.walls_pub = self.create_publisher(LaserScan, walls_topic, pub_qos)
        self.obstacles_pub = self.create_publisher(LaserScan, obstacles_topic, pub_qos)

        self.get_logger().info("Scan Splitter Node initialized")
        self.get_logger().info(f"  Splitting enabled: {self.enable_splitting}")
        self.get_logger().info(f"  Input:  {scan_topic}")
        self.get_logger().info(f"  Walls:  {walls_topic}")
        self.get_logger().info(f"  Obstacles: {obstacles_topic}")
        self.get_logger().info(f"  Threshold: {self.obstacle_threshold} m")

    # ────────────────────────────────────────────────────────────────────
    #  Map handling
    # ────────────────────────────────────────────────────────────────────

    def _map_callback(self, msg: OccupancyGrid):
        """Receive the occupancy grid and precompute the distance field."""
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

        # Convert occupancy grid to binary: occupied (>=50) = wall
        grid = np.array(msg.data, dtype=np.int8).reshape(
            (self.map_height, self.map_width)
        )
        free_mask = (grid >= 0) & (grid < 50)  # free cells

        # Distance transform: for each free cell, distance (in cells) to
        # the nearest occupied cell.  Multiply by resolution → meters.
        dist_cells = distance_transform_edt(free_mask)
        self.distance_field = dist_cells * self.map_resolution

        self.map_ready = True
        self.get_logger().info(
            f"Map received: {self.map_width}×{self.map_height} "
            f"(res={self.map_resolution} m)"
        )

    # ────────────────────────────────────────────────────────────────────
    #  Scan processing
    # ────────────────────────────────────────────────────────────────────

    def _scan_callback(self, scan: LaserScan):
        """Classify each beam as wall or obstacle and publish split scans."""
        # ── Passthrough mode: publish /scan as /scan_walls unchanged ──
        if not self.enable_splitting:
            self.walls_pub.publish(scan)
            # Publish all-inf on /scan_obstacles so downstream nodes
            # see no obstacles.
            empty = self._copy_scan_header(scan)
            empty.ranges = [float('inf')] * len(scan.ranges)
            self.obstacles_pub.publish(empty)
            return

        if not self.map_ready:
            # No map yet — pass through raw scan as walls, empty obstacles
            self.walls_pub.publish(scan)
            return

        # Get laser pose in map frame
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.laser_frame,
                rclpy.time.Time(),  # latest available
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            # TF not available — pass through
            self.walls_pub.publish(scan)
            return

        laser_x = tf.transform.translation.x
        laser_y = tf.transform.translation.y
        # Quaternion → yaw
        q = tf.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        laser_yaw = math.atan2(siny_cosp, cosy_cosp)

        ranges = np.array(scan.ranges, dtype=np.float32)
        n_beams = len(ranges)
        angles = scan.angle_min + np.arange(n_beams, dtype=np.float32) * scan.angle_increment

        # Beam endpoints in map frame
        world_angles = angles + laser_yaw
        endpoints_x = laser_x + ranges * np.cos(world_angles)
        endpoints_y = laser_y + ranges * np.sin(world_angles)

        # Convert to map pixel coordinates
        px = ((endpoints_x - self.map_origin_x) / self.map_resolution).astype(np.int32)
        py = ((endpoints_y - self.map_origin_y) / self.map_resolution).astype(np.int32)

        # Clamp to map bounds
        px = np.clip(px, 0, self.map_width - 1)
        py = np.clip(py, 0, self.map_height - 1)

        # Look up distance to nearest wall at each beam endpoint
        dist_to_wall = self.distance_field[py, px]

        # Valid beams (not inf/nan/out-of-range)
        valid = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)

        # A beam is an obstacle candidate if it's valid AND its endpoint is
        # far from any known wall (i.e. it hit something not in the map).
        is_obstacle_candidate = valid & (dist_to_wall > self.obstacle_threshold)

        # Cluster filtering: require min_cluster_size adjacent obstacle beams
        is_obstacle = self._filter_clusters(is_obstacle_candidate)

        # ── Build output scans ──────────────────────────────────────────
        inf = float("inf")

        # Walls scan: obstacle beams replaced with inf
        wall_ranges = ranges.copy()
        wall_ranges[is_obstacle] = inf

        # Obstacles scan: wall beams replaced with inf
        obstacle_ranges = np.full(n_beams, inf, dtype=np.float32)
        obstacle_ranges[is_obstacle] = ranges[is_obstacle]

        walls_msg = self._copy_scan_header(scan)
        walls_msg.ranges = wall_ranges.tolist()

        obstacles_msg = self._copy_scan_header(scan)
        obstacles_msg.ranges = obstacle_ranges.tolist()

        self.walls_pub.publish(walls_msg)
        self.obstacles_pub.publish(obstacles_msg)

    # ────────────────────────────────────────────────────────────────────
    #  Helpers
    # ────────────────────────────────────────────────────────────────────

    def _filter_clusters(self, mask: np.ndarray) -> np.ndarray:
        """Keep only obstacle segments with >= min_cluster_size adjacent True values."""
        if self.min_cluster_size <= 1:
            return mask

        result = np.zeros_like(mask)
        n = len(mask)
        i = 0
        while i < n:
            if mask[i]:
                # Start of a run
                j = i
                while j < n and mask[j]:
                    j += 1
                run_len = j - i
                if run_len >= self.min_cluster_size:
                    result[i:j] = True
                i = j
            else:
                i += 1
        return result

    def _copy_scan_header(self, scan: LaserScan) -> LaserScan:
        """Create a new LaserScan with the same header/metadata as the input."""
        msg = LaserScan()
        msg.header = scan.header
        msg.angle_min = scan.angle_min
        msg.angle_max = scan.angle_max
        msg.angle_increment = scan.angle_increment
        msg.time_increment = scan.time_increment
        msg.scan_time = scan.scan_time
        msg.range_min = scan.range_min
        msg.range_max = scan.range_max
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = ScanSplitterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
