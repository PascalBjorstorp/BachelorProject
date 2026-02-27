#!/usr/bin/env python3
"""
Localization Benchmark Script

Run alongside the simulation to:
1. Log ground-truth odom (/ego_racecar/odom) vs EKF estimate (/ekf_pose)
2. Measure scan-to-EKF latency (time from /scan_walls to next /ekf_pose)
3. Record AMCL processing time (published on /amcl_timing)
4. Save all data to a timestamped CSV for analysis

Usage:
    ros2 run f1tenth_localization localization_benchmark
    # or just:
    python3 localization_benchmark.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

import csv
import os
import math
import time
from datetime import datetime
from collections import deque


def quaternion_to_yaw(q):
    """Extract yaw from a quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class LocalizationBenchmark(Node):
    def __init__(self):
        super().__init__('localization_benchmark')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('output_dir', os.path.expanduser(
            '~/Documents/BachelorProject/f1tenth_localization/Benchmark'))
        self.declare_parameter('log_rate', 200.0)  # Hz, match EKF rate
        self.declare_parameter('ground_truth_topic', '/ego_racecar/odom')
        self.declare_parameter('ekf_topic', '/ekf_pose')
        self.declare_parameter('amcl_topic', '/amcl_pose')
        self.declare_parameter('scan_topic', '/scan_walls')
        self.declare_parameter('amcl_timing_topic', '/amcl_timing')

        output_dir = self.get_parameter('output_dir').value
        log_rate = self.get_parameter('log_rate').value
        gt_topic = self.get_parameter('ground_truth_topic').value
        ekf_topic = self.get_parameter('ekf_topic').value
        amcl_topic = self.get_parameter('amcl_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        timing_topic = self.get_parameter('amcl_timing_topic').value

        os.makedirs(output_dir, exist_ok=True)

        # ── CSV file ────────────────────────────────────────────────
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(output_dir,
                                     f'localization_benchmark_{timestamp}.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'time_s',
            # Ground truth
            'gt_x', 'gt_y', 'gt_theta',
            'gt_vx', 'gt_vy', 'gt_omega',
            # EKF estimate
            'ekf_x', 'ekf_y', 'ekf_theta',
            'ekf_cov_xx', 'ekf_cov_yy', 'ekf_cov_tt',
            # AMCL estimate
            'amcl_x', 'amcl_y', 'amcl_theta',
            'amcl_cov_xx', 'amcl_cov_yy', 'amcl_cov_tt',
            # Errors
            'error_x', 'error_y', 'error_theta', 'error_euclidean',
            # Timing
            'amcl_processing_ms',
            'scan_to_amcl_ms',
            'scan_to_ekf_ms',
        ])

        # ── State ───────────────────────────────────────────────────
        self.gt_pose = None   # (x, y, theta, vx, vy, omega, stamp)
        self.ekf_pose = None  # (x, y, theta, cov_xx, cov_yy, cov_tt, stamp)
        self.amcl_pose = None  # (x, y, theta, cov_xx, cov_yy, cov_tt, stamp)
        self.last_scan_stamp = None  # wall-clock time of last scan received
        self.last_ekf_stamp = None
        self.scan_to_amcl_ms = float('nan')
        self.scan_to_ekf_ms = float('nan')
        self.amcl_proc_ms = float('nan')
        self.start_time = time.monotonic()

        # Track AMCL-triggered EKF updates:
        # When an AMCL pose arrives, record the scan time for that correction.
        # The next EKF update incorporates that correction.
        self._amcl_correction_scan_stamp = None  # scan time of pending correction
        self._waiting_for_corrected_ekf = False

        # Rolling stats
        self.error_history = deque(maxlen=1000)
        self.amcl_timing_history = deque(maxlen=500)
        self.scan_amcl_history = deque(maxlen=500)
        self.scan_ekf_history = deque(maxlen=500)
        self.log_count = 0

        # ── QoS ─────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        # ── Subscribers ─────────────────────────────────────────────
        self.create_subscription(
            Odometry, gt_topic, self.gt_callback, sensor_qos)
        self.create_subscription(
            PoseWithCovarianceStamped, ekf_topic,
            self.ekf_callback, reliable_qos)
        self.create_subscription(
            PoseWithCovarianceStamped, amcl_topic,
            self.amcl_callback, reliable_qos)
        self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, sensor_qos)
        self.create_subscription(
            Float64, timing_topic, self.timing_callback, reliable_qos)

        # ── Log timer ───────────────────────────────────────────────
        period = 1.0 / log_rate
        self.create_timer(period, self.log_timer_callback)

        # Summary timer (every 5 seconds)
        self.create_timer(5.0, self.summary_callback)

        self.get_logger().info(
            f'Localization benchmark started — saving to {self.csv_path}')

    # ── Callbacks ───────────────────────────────────────────────────
    def gt_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.gt_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            quaternion_to_yaw(q),
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.angular.z,
            self._wall_time(),
        )

    def ekf_callback(self, msg: PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation
        cov = msg.pose.covariance
        now = self._wall_time()

        self.ekf_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            quaternion_to_yaw(q),
            cov[0], cov[7], cov[35],
            now,
        )

        # Scan→EKF latency: only for EKF updates triggered by an AMCL correction.
        # This is the FULL pipeline: scan → AMCL PF → publish → EKF correct → publish.
        if self._waiting_for_corrected_ekf and self._amcl_correction_scan_stamp is not None:
            self.scan_to_ekf_ms = (now - self._amcl_correction_scan_stamp) * 1000.0
            self.scan_ekf_history.append(self.scan_to_ekf_ms)
            self._waiting_for_corrected_ekf = False

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation
        cov = msg.pose.covariance
        now = self._wall_time()

        self.amcl_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            quaternion_to_yaw(q),
            cov[0], cov[7], cov[35],
            now,
        )

        # Scan-to-AMCL latency: time from last scan to AMCL pose arrival.
        # This measures the actual pipeline delay: scan → PF → publish → DDS.
        if self.last_scan_stamp is not None:
            self.scan_to_amcl_ms = (now - self.last_scan_stamp) * 1000.0
            self.scan_amcl_history.append(self.scan_to_amcl_ms)

        # Flag that the next EKF update incorporates this AMCL correction.
        # Store the scan wall-time so EKF can measure full pipeline latency.
        self._amcl_correction_scan_stamp = self.last_scan_stamp
        self._waiting_for_corrected_ekf = True

    def scan_callback(self, msg: LaserScan):
        self.last_scan_stamp = self._wall_time()

    def timing_callback(self, msg: Float64):
        """Receive AMCL processing time in milliseconds."""
        self.amcl_proc_ms = msg.data
        self.amcl_timing_history.append(msg.data)

    # ── Periodic logging ────────────────────────────────────────────
    def log_timer_callback(self):
        if self.gt_pose is None or self.ekf_pose is None:
            return

        t = self._wall_time()
        gt = self.gt_pose
        ekf = self.ekf_pose

        # Compute errors
        error_x = ekf[0] - gt[0]
        error_y = ekf[1] - gt[1]
        error_theta = self._angle_diff(ekf[2], gt[2])
        error_euclid = math.sqrt(error_x ** 2 + error_y ** 2)

        self.error_history.append(error_euclid)

        # AMCL pose (may be None if not received yet)
        amcl = self.amcl_pose or (float('nan'),) * 7

        self.csv_writer.writerow([
            f'{t:.4f}',
            f'{gt[0]:.6f}', f'{gt[1]:.6f}', f'{gt[2]:.6f}',
            f'{gt[3]:.4f}', f'{gt[4]:.4f}', f'{gt[5]:.4f}',
            f'{ekf[0]:.6f}', f'{ekf[1]:.6f}', f'{ekf[2]:.6f}',
            f'{ekf[3]:.8f}', f'{ekf[4]:.8f}', f'{ekf[5]:.8f}',
            f'{amcl[0]:.6f}' if not math.isnan(amcl[0]) else '',
            f'{amcl[1]:.6f}' if not math.isnan(amcl[0]) else '',
            f'{amcl[2]:.6f}' if not math.isnan(amcl[0]) else '',
            f'{amcl[3]:.8f}' if not math.isnan(amcl[0]) else '',
            f'{amcl[4]:.8f}' if not math.isnan(amcl[0]) else '',
            f'{amcl[5]:.8f}' if not math.isnan(amcl[0]) else '',
            f'{error_x:.6f}', f'{error_y:.6f}', f'{error_theta:.6f}',
            f'{error_euclid:.6f}',
            f'{self.amcl_proc_ms:.3f}'
            if not math.isnan(self.amcl_proc_ms) else '',
            f'{self.scan_to_amcl_ms:.3f}'
            if not math.isnan(self.scan_to_amcl_ms) else '',
            f'{self.scan_to_ekf_ms:.3f}'
            if not math.isnan(self.scan_to_ekf_ms) else '',
        ])

        self.log_count += 1

    # ── Periodic summary ────────────────────────────────────────────
    def summary_callback(self):
        if len(self.error_history) == 0:
            self.get_logger().info('Waiting for data...')
            return

        errors = list(self.error_history)
        mean_err = sum(errors) / len(errors)
        max_err = max(errors)
        rms_err = math.sqrt(sum(e * e for e in errors) / len(errors))

        lines = [
            f'--- Benchmark Summary ({self.log_count} samples) ---',
            f'  Position error: mean={mean_err:.4f}m, '
            f'max={max_err:.4f}m, RMS={rms_err:.4f}m',
        ]

        if len(self.amcl_timing_history) > 0:
            t = list(self.amcl_timing_history)
            lines.append(
                f'  AMCL processing: mean={sum(t)/len(t):.2f}ms, '
                f'max={max(t):.2f}ms, min={min(t):.2f}ms')

        if len(self.scan_amcl_history) > 0:
            s = list(self.scan_amcl_history)
            lines.append(
                f'  Scan→AMCL latency: mean={sum(s)/len(s):.2f}ms, '
                f'max={max(s):.2f}ms, min={min(s):.2f}ms')

        if len(self.scan_ekf_history) > 0:
            e = list(self.scan_ekf_history)
            lines.append(
                f'  Scan→EKF  latency: mean={sum(e)/len(e):.2f}ms, '
                f'max={max(e):.2f}ms, min={min(e):.2f}ms')

        for line in lines:
            self.get_logger().info(line)

    # ── Helpers ─────────────────────────────────────────────────────
    def _wall_time(self):
        return time.monotonic() - self.start_time

    @staticmethod
    def _angle_diff(a, b):
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d

    def destroy_node(self):
        self.csv_file.close()
        self.get_logger().info(
            f'Benchmark saved: {self.csv_path} '
            f'({self.log_count} samples)')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationBenchmark()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
