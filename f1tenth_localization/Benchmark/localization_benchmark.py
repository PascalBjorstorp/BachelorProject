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
        self.declare_parameter('ground_truth_topic', '/ego_racecar/ground_truth')
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
            # EKF errors (vs ground truth)
            'error_x', 'error_y', 'error_theta', 'error_euclidean',
            # AMCL errors (vs ground truth)
            'amcl_error_x', 'amcl_error_y', 'amcl_error_theta',
            'amcl_error_euclidean',
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
        self.error_x_history = deque(maxlen=1000)
        self.error_y_history = deque(maxlen=1000)
        self.error_theta_history = deque(maxlen=1000)
        self.amcl_error_history = deque(maxlen=1000)   # AMCL euclidean
        self.amcl_error_x_history = deque(maxlen=1000)
        self.amcl_error_y_history = deque(maxlen=1000)
        self.amcl_timing_history = deque(maxlen=500)
        self.scan_amcl_history = deque(maxlen=500)
        self.scan_ekf_history = deque(maxlen=500)
        self.log_count = 0

        # Early vs late bias (first 5 seconds vs rest)
        self._early_errors_x = []   # collected during t < 5s
        self._early_errors_y = []
        self._late_errors_x = []    # collected during t >= 5s
        self._late_errors_y = []
        self._early_amcl_errors_x = []
        self._early_amcl_errors_y = []
        self._late_amcl_errors_x = []
        self._late_amcl_errors_y = []
        self._early_cutoff = 5.0    # seconds

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

        # Compute EKF errors
        error_x = ekf[0] - gt[0]
        error_y = ekf[1] - gt[1]
        error_theta = self._angle_diff(ekf[2], gt[2])
        error_euclid = math.sqrt(error_x ** 2 + error_y ** 2)

        self.error_history.append(error_euclid)
        self.error_x_history.append(error_x)
        self.error_y_history.append(error_y)
        self.error_theta_history.append(error_theta)

        # Early vs late bias tracking (EKF)
        if t < self._early_cutoff:
            self._early_errors_x.append(error_x)
            self._early_errors_y.append(error_y)
        else:
            self._late_errors_x.append(error_x)
            self._late_errors_y.append(error_y)

        # AMCL pose (may be None if not received yet)
        amcl = self.amcl_pose or (float('nan'),) * 7

        # Compute AMCL errors
        if not math.isnan(amcl[0]):
            amcl_err_x = amcl[0] - gt[0]
            amcl_err_y = amcl[1] - gt[1]
            amcl_err_theta = self._angle_diff(amcl[2], gt[2])
            amcl_err_euclid = math.sqrt(amcl_err_x ** 2 + amcl_err_y ** 2)
            self.amcl_error_history.append(amcl_err_euclid)
            self.amcl_error_x_history.append(amcl_err_x)
            self.amcl_error_y_history.append(amcl_err_y)
            # Early vs late (AMCL)
            if t < self._early_cutoff:
                self._early_amcl_errors_x.append(amcl_err_x)
                self._early_amcl_errors_y.append(amcl_err_y)
            else:
                self._late_amcl_errors_x.append(amcl_err_x)
                self._late_amcl_errors_y.append(amcl_err_y)
        else:
            amcl_err_x = float('nan')
            amcl_err_y = float('nan')
            amcl_err_theta = float('nan')
            amcl_err_euclid = float('nan')

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
            f'{amcl_err_x:.6f}' if not math.isnan(amcl_err_x) else '',
            f'{amcl_err_y:.6f}' if not math.isnan(amcl_err_y) else '',
            f'{amcl_err_theta:.6f}' if not math.isnan(amcl_err_theta) else '',
            f'{amcl_err_euclid:.6f}' if not math.isnan(amcl_err_euclid) else '',
            f'{self.amcl_proc_ms:.3f}'
            if not math.isnan(self.amcl_proc_ms) else '',
            f'{self.scan_to_amcl_ms:.3f}'
            if not math.isnan(self.scan_to_amcl_ms) else '',
            f'{self.scan_to_ekf_ms:.3f}'
            if not math.isnan(self.scan_to_ekf_ms) else '',
        ])

        self.log_count += 1

    # ── Periodic summary ────────────────────────────────────────────
    @staticmethod
    def _stats(data):
        """Return (mean, std, min, max) for a list of numbers."""
        n = len(data)
        if n == 0:
            return (float('nan'),) * 4
        m = sum(data) / n
        var = sum((x - m) ** 2 for x in data) / n
        return m, math.sqrt(var), min(data), max(data)

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
            f'  EKF euclidean: mean={mean_err:.4f}m, '
            f'max={max_err:.4f}m, RMS={rms_err:.4f}m',
        ]

        # ── Per-axis EKF bias ────────────────────────────────────
        if len(self.error_x_history) > 0:
            bx_m, bx_s, _, _ = self._stats(list(self.error_x_history))
            by_m, by_s, _, _ = self._stats(list(self.error_y_history))
            bt_m, bt_s, _, _ = self._stats(list(self.error_theta_history))
            bias_mag = math.sqrt(bx_m ** 2 + by_m ** 2)
            lines.append(
                f'  EKF bias: X={bx_m:+.4f}±{bx_s:.4f}m  '
                f'Y={by_m:+.4f}±{by_s:.4f}m  '
                f'θ={math.degrees(bt_m):+.2f}±{math.degrees(bt_s):.2f}°  '
                f'|bias|={bias_mag:.4f}m')

        # ── Per-axis AMCL bias ───────────────────────────────────
        if len(self.amcl_error_x_history) > 0:
            ax_m, ax_s, _, _ = self._stats(list(self.amcl_error_x_history))
            ay_m, ay_s, _, _ = self._stats(list(self.amcl_error_y_history))
            amcl_errs = list(self.amcl_error_history)
            amcl_rms = math.sqrt(
                sum(e * e for e in amcl_errs) / len(amcl_errs))
            abias_mag = math.sqrt(ax_m ** 2 + ay_m ** 2)
            lines.append(
                f'  AMCL bias: X={ax_m:+.4f}±{ax_s:.4f}m  '
                f'Y={ay_m:+.4f}±{ay_s:.4f}m  '
                f'|bias|={abias_mag:.4f}m  RMS={amcl_rms:.4f}m')

        # ── Early vs Late bias (convergence check) ──────────────
        if len(self._late_errors_x) > 10:
            ex_m, _, _, _ = self._stats(self._early_errors_x)
            ey_m, _, _, _ = self._stats(self._early_errors_y)
            lx_m, lx_s, _, _ = self._stats(self._late_errors_x)
            ly_m, ly_s, _, _ = self._stats(self._late_errors_y)
            lines.append(
                f'  EKF early(<{self._early_cutoff:.0f}s): '
                f'X={ex_m:+.4f}  Y={ey_m:+.4f}  '
                f'|bias|={math.sqrt(ex_m**2+ey_m**2):.4f}m')
            lines.append(
                f'  EKF late(≥{self._early_cutoff:.0f}s): '
                f'X={lx_m:+.4f}±{lx_s:.4f}  Y={ly_m:+.4f}±{ly_s:.4f}  '
                f'|bias|={math.sqrt(lx_m**2+ly_m**2):.4f}m')

        if len(self._late_amcl_errors_x) > 10:
            eax_m, _, _, _ = self._stats(self._early_amcl_errors_x)
            eay_m, _, _, _ = self._stats(self._early_amcl_errors_y)
            lax_m, lax_s, _, _ = self._stats(self._late_amcl_errors_x)
            lay_m, lay_s, _, _ = self._stats(self._late_amcl_errors_y)
            lines.append(
                f'  AMCL early(<{self._early_cutoff:.0f}s): '
                f'X={eax_m:+.4f}  Y={eay_m:+.4f}  '
                f'|bias|={math.sqrt(eax_m**2+eay_m**2):.4f}m')
            lines.append(
                f'  AMCL late(≥{self._early_cutoff:.0f}s): '
                f'X={lax_m:+.4f}±{lax_s:.4f}  Y={lay_m:+.4f}±{lay_s:.4f}  '
                f'|bias|={math.sqrt(lax_m**2+lay_m**2):.4f}m')

        # ── Timing ──────────────────────────────────────────────
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
