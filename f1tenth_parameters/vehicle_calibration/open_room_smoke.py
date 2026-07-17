#!/usr/bin/env python3
"""Exercise both calibration ROS graphs in the purpose-built open room."""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float64, String
from vesc_msgs.msg import VescStateStamped


ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[1]


class Probe(Node):
    def __init__(self, kind: str) -> None:
        super().__init__(f"{kind}_open_room_smoke_probe")
        self.kind = kind
        self.scan_times: list[float] = []
        self.scan_lengths: list[int] = []
        self.odom: list[tuple[float, float, float, float]] = []
        self.ground_truth: list[tuple[float, float, float, float]] = []
        self.imu_count = 0
        self.core_count = 0
        self.servo_echo: list[float] = []
        self.selected_servo: list[float] = []
        self.selected_speed: list[float] = []
        self.selected_brake: list[float] = []
        self.create_subscription(LaserScan, "/scan", self._scan, 10)
        self.create_subscription(Odometry, "/ego_racecar/odom", self._odom, 10)
        self.create_subscription(Odometry, "/ego_racecar/ground_truth", self._ground_truth, 10)
        self.create_subscription(Imu, "/sensors/imu/raw", self._imu, 10)
        self.create_subscription(VescStateStamped, "/sensors/core", self._core, 10)
        self.create_subscription(Float64, "/sensors/servo_position_command", self._echo, 10)
        self.drive = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        if kind == "steering":
            self.create_subscription(Float64, "/steering_calibration/servo_selected", self._selected_servo, 10)
            self.raw_servo = self.create_publisher(Float64, "/steering_calibration/servo_raw", 10)
        else:
            self.create_subscription(Float64, "/erpm_calibration/motor_selected_speed", self._selected_speed, 10)
            self.create_subscription(Float64, "/erpm_calibration/motor_selected_brake", self._selected_brake, 10)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.mode = self.create_publisher(String, "/erpm_calibration/motor_mode", qos)
            self.raw_speed = self.create_publisher(Float64, "/erpm_calibration/motor_raw_speed", 10)
            self.raw_brake = self.create_publisher(Float64, "/erpm_calibration/motor_raw_brake", 10)

    def _scan(self, msg: LaserScan) -> None:
        self.scan_times.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1.0e-9)
        self.scan_lengths.append(len(msg.ranges))

    def _odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom.append((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.twist.twist.linear.x, yaw))

    def _ground_truth(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.ground_truth.append((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.twist.twist.linear.x, yaw))

    def _imu(self, _msg: Imu) -> None:
        self.imu_count += 1

    def _core(self, _msg: VescStateStamped) -> None:
        self.core_count += 1

    def _echo(self, msg: Float64) -> None:
        self.servo_echo.append(msg.data)

    def _selected_servo(self, msg: Float64) -> None:
        self.selected_servo.append(msg.data)

    def _selected_speed(self, msg: Float64) -> None:
        self.selected_speed.append(msg.data)

    def _selected_brake(self, msg: Float64) -> None:
        self.selected_brake.append(msg.data)

    def spin_for(self, seconds: float, action: Any) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            action()
            rclpy.spin_once(self, timeout_sec=0.01)

    def exercise(self) -> None:
        if self.kind == "steering":
            def move() -> None:
                drive = AckermannDriveStamped(); drive.drive.speed = 0.40; self.drive.publish(drive)
                servo = Float64(); servo.data = 0.45; self.raw_servo.publish(servo)
            self.spin_for(2.5, move)
            def steering_stop() -> None:
                self.drive.publish(AckermannDriveStamped())
                servo = Float64(); servo.data = 0.55; self.raw_servo.publish(servo)
            self.spin_for(1.0, steering_stop)
        else:
            speed_mode = String(); speed_mode.data = "raw_erpm"; self.mode.publish(speed_mode)
            def move() -> None:
                drive = AckermannDriveStamped(); drive.drive.steering_angle = 0.15; self.drive.publish(drive)
                value = Float64(); value.data = 1200.0; self.raw_speed.publish(value)
            self.spin_for(2.5, move)
            brake_mode = String(); brake_mode.data = "raw_brake"; self.mode.publish(brake_mode)
            def stop() -> None:
                drive = AckermannDriveStamped(); drive.drive.steering_angle = 0.15; self.drive.publish(drive)
                value = Float64(); value.data = 1.0; self.raw_brake.publish(value)
            self.spin_for(0.8, stop)
            neutral = String(); neutral.data = "neutral"; self.mode.publish(neutral)
            self.spin_for(0.3, lambda: None)

    def summary(self) -> dict[str, Any]:
        intervals = [b - a for a, b in zip(self.scan_times, self.scan_times[1:]) if b > a]
        scan_rate = 1.0 / (sum(intervals) / len(intervals)) if intervals else None
        maximum_position = max((max(abs(x), abs(y)) for x, y, _, _ in self.ground_truth), default=None)
        maximum_odom_position = max((max(abs(x), abs(y)) for x, y, _, _ in self.odom), default=None)
        heading_change = (
            math.atan2(
                math.sin(self.ground_truth[-1][3] - self.ground_truth[0][3]),
                math.cos(self.ground_truth[-1][3] - self.ground_truth[0][3]),
            ) if len(self.ground_truth) >= 2 else None
        )
        result: dict[str, Any] = {
            "map": "calibration_room_map (14 x 14 m; central 12 x 12 m clear; diagonal lane)",
            "scan_messages": len(self.scan_times),
            "scan_lengths": sorted(set(self.scan_lengths)),
            "scan_rate_hz": scan_rate,
            "odom_messages": len(self.odom),
            "ground_truth_messages": len(self.ground_truth),
            "imu_messages": self.imu_count,
            "core_messages": self.core_count,
            "servo_echo_messages": len(self.servo_echo),
            "maximum_abs_position_m": maximum_position,
            "maximum_abs_odom_position_m": maximum_odom_position,
            "ground_truth_start_xy": list(self.ground_truth[0][:2]) if self.ground_truth else None,
            "ground_truth_end_xy": list(self.ground_truth[-1][:2]) if self.ground_truth else None,
            "odom_start_xy": list(self.odom[0][:2]) if self.odom else None,
            "odom_end_xy": list(self.odom[-1][:2]) if self.odom else None,
            "ground_truth_heading_change_rad": heading_change,
        }
        if self.kind == "steering":
            result["selected_servo_median"] = sorted(self.selected_servo)[len(self.selected_servo) // 2] if self.selected_servo else None
        else:
            result["selected_speed_peak_erpm"] = max(self.selected_speed, default=None)
            result["selected_brake_peak_a"] = max(self.selected_brake, default=None)
        failures = []
        if sorted(set(self.scan_lengths)) != [1080]: failures.append("scan does not contain exactly 1080 beams")
        if scan_rate is None or not 35.0 <= scan_rate <= 45.0: failures.append("scan rate is outside 35–45 Hz")
        if min(len(self.odom), self.imu_count, self.core_count) < 20: failures.append("odom/IMU/VESC streams are incomplete")
        if maximum_position is None or maximum_position >= 5.0: failures.append("smoke manoeuvre left the central safety region")
        if heading_change is None or abs(heading_change) < 0.02: failures.append("intentional steering did not produce observable yaw")
        result["failures"] = failures
        result["status"] = "pass" if not failures else "fail"
        return result


def _command(kind: str) -> list[str]:
    if kind == "steering":
        return [
            sys.executable, str(REPO / "f1tenth_parameters/steering/launch/calibration_stack.py"),
            "--config", str(REPO / "f1tenth_parameters/steering/config/steering_calibration.yaml"),
            "--raw-min", "0.10", "--raw-max", "0.94", "--simulation",
        ]
    return [
        sys.executable, str(REPO / "f1tenth_parameters/ERPM/launch/calibration_stack.py"),
        "--config", str(REPO / "f1tenth_parameters/ERPM/config/erpm_calibration.yaml"), "--simulation",
    ]


def run_one(kind: str, output: Path) -> dict[str, Any]:
    log_path = output / f"{kind}_open_room_ros.log"
    with log_path.open("w", encoding="utf-8") as log:
        process = subprocess.Popen(_command(kind), cwd=REPO, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True, text=True)
        try:
            time.sleep(8.0)  # includes the production odometry startup bias epoch
            rclpy.init()
            node = Probe(kind)
            try:
                node.exercise()
                result = node.summary()
            finally:
                node.destroy_node()
                rclpy.shutdown()
        finally:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3.0)
    text = log_path.read_text(encoding="utf-8", errors="replace")
    if "calibration_room_map" not in text:
        result["failures"].append("launch log does not prove the open calibration map")
    if "collision detected" in text.lower():
        result["failures"].append("simulated car collided")
    if "vesc_driver::VescDriver" in text:
        result["failures"].append("hardware VESC driver was started in simulation")
    result["status"] = "pass" if not result["failures"] else "fail"
    result["log"] = str(log_path)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--kind", choices=("all", "steering", "erpm"), default="all")
    parser.add_argument("--output", type=Path, default=ROOT / "simulation_smoke")
    args = parser.parse_args()
    output = args.output.resolve(); output.mkdir(parents=True, exist_ok=True)
    kinds = ("steering", "erpm") if args.kind == "all" else (args.kind,)
    report = {kind: run_one(kind, output) for kind in kinds}
    report["status"] = "pass" if all(item["status"] == "pass" for item in report.values()) else "fail"
    path = output / "open_room_smoke_report.json"
    path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["status"] == "pass" else 2


if __name__ == "__main__":
    raise SystemExit(main())
