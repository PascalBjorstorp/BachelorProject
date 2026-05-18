#!/usr/bin/env python3
"""Log ground truth aligned to each /ekf_pose update and stop benchmark runs."""

import argparse
import csv
from collections import deque
import json
import math
import os
import sys
from typing import Deque, List, Optional, Tuple

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


class RacelineProgress:
    def __init__(self, path: str) -> None:
        self.points: List[Tuple[float, float, float]] = []
        with open(path, newline='') as handle:
            for raw in handle:
                line = raw.strip()
                if not line or line.startswith('#'):
                    continue
                parts = [p.strip() for p in line.split(',')]
                if len(parts) < 3:
                    continue
                try:
                    s = float(parts[0])
                    x = float(parts[1])
                    y = float(parts[2])
                except ValueError:
                    continue
                self.points.append((s, x, y))

        if len(self.points) < 2:
            raise RuntimeError(f'Need at least two raceline points: {path}')

        self.length = self.points[-1][0]
        if self.length <= 0.0:
            raise RuntimeError(f'Invalid raceline length in {path}')

    def nearest_s(self, x: float, y: float) -> float:
        best_s = 0.0
        best_d2 = float('inf')
        for s, px, py in self.points:
            dx = x - px
            dy = y - py
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_s = s
        return best_s


class SimBenchmarkLogger(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('sim_benchmark_logger')
        self.args = args
        self.progress = RacelineProgress(args.trajectory_file)
        self.gt_buffer: Deque[Odometry] = deque(maxlen=args.gt_buffer_size)
        self.latest_amcl: Optional[PoseWithCovarianceStamped] = None
        self.collision = False
        self.prev_s: Optional[float] = None
        self.laps = 0
        self.rows = 0
        self.done_reason: Optional[str] = None
        self.start_time = self.get_clock().now()

        os.makedirs(args.output_dir, exist_ok=True)
        self.csv_path = os.path.join(args.output_dir, args.csv_name)
        self.status_path = os.path.join(args.output_dir, args.status_name)
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'wall_time_ns',
            'ekf_stamp_ns',
            'gt_stamp_ns',
            'amcl_stamp_ns',
            'localizer',
            'lap_count',
            'progress_s',
            'progress_ratio',
            'ekf_x',
            'ekf_y',
            'ekf_yaw',
            'ekf_cov_x',
            'ekf_cov_y',
            'ekf_cov_yaw',
            'gt_x',
            'gt_y',
            'gt_yaw',
            'gt_vx',
            'gt_vy',
            'gt_wz',
            'amcl_x',
            'amcl_y',
            'amcl_yaw',
            'err_x',
            'err_y',
            'err_xy',
            'err_yaw',
            'collision',
        ])
        self.csv_file.flush()

        self.create_subscription(Odometry, args.groundtruth_topic, self.gt_callback, 50)
        self.create_subscription(
            PoseWithCovarianceStamped, args.ekf_topic, self.ekf_callback, 50)
        self.create_subscription(
            PoseWithCovarianceStamped, args.amcl_topic, self.amcl_callback, 50)
        self.create_subscription(Bool, args.collision_topic, self.collision_callback, 10)
        self.create_timer(0.5, self.timeout_check)

        self.get_logger().info(
            f'Logging {args.groundtruth_topic} at each {args.ekf_topic} update to {self.csv_path}')

    def gt_callback(self, msg: Odometry) -> None:
        self.gt_buffer.append(msg)

    def amcl_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self.latest_amcl = msg

    def collision_callback(self, msg: Bool) -> None:
        self.collision = bool(msg.data)
        if self.collision:
            self.finish('collision')

    def timeout_check(self) -> None:
        if self.done_reason:
            rclpy.shutdown()
            return
        if self.args.max_duration_sec <= 0.0:
            return
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed >= self.args.max_duration_sec:
            self.finish('timeout')

    def nearest_gt(self, ekf_stamp_ns: int) -> Optional[Odometry]:
        if not self.gt_buffer:
            return None
        if ekf_stamp_ns <= 0:
            return self.gt_buffer[-1]
        return min(
            self.gt_buffer,
            key=lambda msg: abs(stamp_ns(msg.header.stamp) - ekf_stamp_ns))

    def update_laps(self, progress_s: float) -> None:
        if self.prev_s is not None:
            length = self.progress.length
            if self.prev_s > 0.75 * length and progress_s < 0.25 * length:
                self.laps += 1
                self.get_logger().info(f'Lap {self.laps}/{self.args.max_laps}')
        self.prev_s = progress_s

    def ekf_callback(self, msg: PoseWithCovarianceStamped) -> None:
        if self.done_reason:
            return

        ekf_ns = stamp_ns(msg.header.stamp)
        gt = self.nearest_gt(ekf_ns)
        if gt is None:
            return

        gt_ns = stamp_ns(gt.header.stamp)
        gt_x = gt.pose.pose.position.x
        gt_y = gt.pose.pose.position.y
        gt_yaw = yaw_from_quaternion(gt.pose.pose.orientation)

        ekf_x = msg.pose.pose.position.x
        ekf_y = msg.pose.pose.position.y
        ekf_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        progress_s = self.progress.nearest_s(gt_x, gt_y)
        self.update_laps(progress_s)
        progress_ratio = progress_s / self.progress.length

        amcl_ns = 0
        amcl_x = math.nan
        amcl_y = math.nan
        amcl_yaw = math.nan
        if self.latest_amcl is not None:
            amcl_ns = stamp_ns(self.latest_amcl.header.stamp)
            amcl_x = self.latest_amcl.pose.pose.position.x
            amcl_y = self.latest_amcl.pose.pose.position.y
            amcl_yaw = yaw_from_quaternion(self.latest_amcl.pose.pose.orientation)

        err_x = ekf_x - gt_x
        err_y = ekf_y - gt_y
        err_xy = math.hypot(err_x, err_y)
        err_yaw = angle_diff(ekf_yaw, gt_yaw)

        self.writer.writerow([
            self.get_clock().now().nanoseconds,
            ekf_ns,
            gt_ns,
            amcl_ns,
            self.args.localizer,
            self.laps,
            f'{progress_s:.9f}',
            f'{progress_ratio:.9f}',
            f'{ekf_x:.9f}',
            f'{ekf_y:.9f}',
            f'{ekf_yaw:.9f}',
            f'{msg.pose.covariance[0]:.9g}',
            f'{msg.pose.covariance[7]:.9g}',
            f'{msg.pose.covariance[35]:.9g}',
            f'{gt_x:.9f}',
            f'{gt_y:.9f}',
            f'{gt_yaw:.9f}',
            f'{gt.twist.twist.linear.x:.9f}',
            f'{gt.twist.twist.linear.y:.9f}',
            f'{gt.twist.twist.angular.z:.9f}',
            f'{amcl_x:.9f}',
            f'{amcl_y:.9f}',
            f'{amcl_yaw:.9f}',
            f'{err_x:.9f}',
            f'{err_y:.9f}',
            f'{err_xy:.9f}',
            f'{err_yaw:.9f}',
            int(self.collision),
        ])
        self.rows += 1
        if self.rows % self.args.flush_every == 0:
            self.csv_file.flush()

        if self.args.max_laps > 0 and self.laps >= self.args.max_laps:
            self.finish('laps_complete')

    def finish(self, reason: str) -> None:
        if self.done_reason:
            return
        self.done_reason = reason
        self.csv_file.flush()
        summary = {
            'localizer': self.args.localizer,
            'reason': reason,
            'laps': self.laps,
            'rows': self.rows,
            'csv_path': self.csv_path,
            'trajectory_file': self.args.trajectory_file,
            'track_length_m': self.progress.length,
        }
        with open(self.status_path, 'w') as handle:
            json.dump(summary, handle, indent=2)
            handle.write('\n')
        self.get_logger().info(f'Benchmark stopping: {reason}')

    def close(self) -> None:
        if self.done_reason is None:
            self.finish('shutdown')
        self.csv_file.close()


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--localizer', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--trajectory-file', required=True)
    parser.add_argument('--max-laps', type=int, default=10)
    parser.add_argument('--max-duration-sec', type=float, default=0.0)
    parser.add_argument('--groundtruth-topic', default='/ego_racecar/ground_truth')
    parser.add_argument('--ekf-topic', default='/ekf_pose')
    parser.add_argument('--amcl-topic', default='/amcl_pose')
    parser.add_argument('--collision-topic', default='/ego_racecar/collision')
    parser.add_argument('--csv-name', default='groundtruth_at_ekf.csv')
    parser.add_argument('--status-name', default='run_status.json')
    parser.add_argument('--flush-every', type=int, default=50)
    parser.add_argument('--gt-buffer-size', type=int, default=2000)
    args, _ = parser.parse_known_args(argv)
    return args


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    rclpy.init()
    node = SimBenchmarkLogger(args)
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
