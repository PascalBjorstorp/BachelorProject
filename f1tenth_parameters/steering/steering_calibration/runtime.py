"""ROS runtime primitives for raw-servo, bag-first steering calibration.

The calibration measurements deliberately bypass the existing steering-angle
mapping.  During all calibration manoeuvres AckermannToVesc still supplies the
motor command, but its servo output is remapped into a selector input.  This
node publishes the selected raw servo value to the other selector input.
"""
from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float64, String

from .events import EventPublisher


@dataclass
class Latest:
    imu_gz: float = math.nan
    imu_ax: float = math.nan
    imu_ay: float = math.nan
    odom_vx: float = math.nan
    odom_wz: float = math.nan
    battery_v: float = math.nan
    erpm: float = math.nan
    servo_echo: float = math.nan
    servo_selected: float = math.nan
    servo_bus: float = math.nan
    scan_count: int = 0
    seen: set[str] = field(default_factory=set)


class CalibrationNode(Node):
    """Experiment control only; all fit-quality decisions remain offline."""

    def __init__(self, name: str, config: dict[str, Any]) -> None:
        super().__init__(name)
        self.cfg = config
        self.latest = Latest()
        self._samples: dict[str, list[float]] = {}
        self._window_active = False
        self.event = EventPublisher(self, config["session"]["event_topic"])

        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 20)
        self.raw_servo_pub = self.create_publisher(Float64, "/steering_calibration/servo_raw", 20)
        mode_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mode_pub = self.create_publisher(String, "/steering_calibration/servo_mode", mode_qos)

        # These queues are used only for runtime interlocks and state scheduling.
        # MCAP separately captures every native topic message.
        self.create_subscription(Imu, "/sensors/imu/raw", self._imu_cb, 400)
        self.create_subscription(Odometry, "/ego_racecar/odom", self._odom_cb, 400)
        self.create_subscription(Float64, "/sensors/servo_position_command", self._servo_echo_cb, 200)
        # Redundant steering command observations: selected selector output and
        # the actual VESC command bus are both recorded and checked in Stage 0.
        self.create_subscription(Float64, "/steering_calibration/servo_selected", self._servo_selected_cb, 200)
        self.create_subscription(Float64, "/commands/servo/position", self._servo_bus_cb, 200)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, 50)
        try:
            from vesc_msgs.msg import VescStateStamped
            self.create_subscription(VescStateStamped, "/sensors/core", self._vesc_cb, 200)
        except Exception as exc:
            self.get_logger().error(f"Cannot import vesc_msgs/VescStateStamped: {exc}")

    # ---- sensor callbacks -------------------------------------------------
    def _imu_cb(self, msg: Imu) -> None:
        self.latest.imu_gz = float(msg.angular_velocity.z)
        self.latest.imu_ax = float(msg.linear_acceleration.x)
        self.latest.imu_ay = float(msg.linear_acceleration.y)
        self.latest.seen.add("imu")
        self._record_window(imu_gz=self.latest.imu_gz, imu_ax=self.latest.imu_ax, imu_ay=self.latest.imu_ay)

    def _odom_cb(self, msg: Odometry) -> None:
        self.latest.odom_vx = float(msg.twist.twist.linear.x)
        self.latest.odom_wz = float(msg.twist.twist.angular.z)
        self.latest.seen.add("odom")
        self._record_window(odom_vx=self.latest.odom_vx, odom_wz=self.latest.odom_wz)

    def _servo_echo_cb(self, msg: Float64) -> None:
        self.latest.servo_echo = float(msg.data)
        self.latest.seen.add("servo_echo")
        self._record_window(servo_echo=self.latest.servo_echo)

    def _servo_selected_cb(self, msg: Float64) -> None:
        self.latest.servo_selected = float(msg.data)
        self.latest.seen.add("servo_selected")
        self._record_window(servo_selected=self.latest.servo_selected)

    def _servo_bus_cb(self, msg: Float64) -> None:
        self.latest.servo_bus = float(msg.data)
        self.latest.seen.add("servo_bus")
        self._record_window(servo_bus=self.latest.servo_bus)

    def _scan_cb(self, _: LaserScan) -> None:
        self.latest.scan_count += 1
        self.latest.seen.add("scan")

    def _vesc_cb(self, msg: Any) -> None:
        self.latest.battery_v = float(msg.state.voltage_input)
        self.latest.erpm = float(msg.state.speed)
        self.latest.seen.add("vesc")
        self._record_window(battery_v=self.latest.battery_v, erpm=self.latest.erpm)

    # ---- windows ----------------------------------------------------------
    def begin_window(self, *fields: str) -> None:
        self._samples = {field: [] for field in fields}
        self._window_active = True

    def end_window(self) -> dict[str, float]:
        self._window_active = False
        result: dict[str, float] = {}
        for key, values in self._samples.items():
            array = np.asarray(values, dtype=float)
            result[f"{key}_mean"] = float(np.nanmean(array)) if array.size else math.nan
            result[f"{key}_std"] = float(np.nanstd(array)) if array.size else math.nan
            result[f"{key}_count"] = int(array.size)
        return result

    def _record_window(self, **values: float) -> None:
        if not self._window_active:
            return
        for key, value in values.items():
            if key in self._samples:
                self._samples[key].append(float(value))

    # ---- actuator / drive commands ---------------------------------------
    def set_steering_mode(self, mode: str) -> None:
        if mode not in {"raw", "ackermann"}:
            raise ValueError(f"unknown steering selector mode: {mode}")
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        self.event.emit("steering_selector_mode", mode=mode)

    def drive(self, speed_mps: float) -> None:
        """Publish motor-speed demand through the normal drive/mux/Ackermann path.

        Steering angle is deliberately zero: the selector ignores Ackermann
        servo output while in raw mode. It remains available for later deployed
        mapping verification when the selector is switched to ackermann mode.
        """
        message = AckermannDriveStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.drive.speed = float(speed_mps)
        message.drive.steering_angle = 0.0
        self.drive_pub.publish(message)

    def raw_servo(self, servo_position: float) -> None:
        msg = Float64()
        msg.data = float(servo_position)
        self.raw_servo_pub.publish(msg)

    def command(self, speed_mps: float, raw_servo: float | None) -> None:
        self.drive(speed_mps)
        if raw_servo is not None:
            self.raw_servo(raw_servo)

    def neutral_drive(self, centre_raw_servo: float | None = None) -> None:
        self.command(0.0, centre_raw_servo)
        self.spin(0.05)
        self.command(0.0, centre_raw_servo)

    def spin(self, timeout_s: float) -> None:
        """Drain ready callbacks during the requested control interval.

        Odom arrives at 200 Hz. This runtime needs only current values for
        startup/safety scheduling; MCAP remains the authoritative raw record.
        """
        deadline = time.monotonic() + max(0.0, timeout_s)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.0005)

    # ---- readiness and safety --------------------------------------------
    def wait_for(self, required: set[str], timeout_s: float = 12.0) -> None:
        start = time.monotonic()
        while time.monotonic() - start < timeout_s:
            self.spin(0.05)
            if required.issubset(self.latest.seen):
                return
        missing = sorted(required - self.latest.seen)
        raise RuntimeError("Missing required streams: " + ", ".join(missing))

    def check_safety(self, motion_expected: bool) -> None:
        limits = self.cfg["session"]
        battery_min = limits.get("battery_min_v")
        if battery_min is not None and "vesc" in self.latest.seen:
            if self.latest.battery_v < float(battery_min):
                raise RuntimeError(f"Battery below limit: {self.latest.battery_v:.2f} V < {float(battery_min):.2f} V")
        if motion_expected and "odom" in self.latest.seen:
            if abs(self.latest.odom_vx) > float(limits["hard_speed_limit_mps"]):
                raise RuntimeError(
                    f"ERPM odometry exceeded hard speed limit: {self.latest.odom_vx:.2f} m/s"
                )
        hard_erpm = float(limits.get("hard_erpm_limit", math.inf))
        if motion_expected and "vesc" in self.latest.seen:
            if abs(self.latest.erpm) > hard_erpm:
                raise RuntimeError(
                    f"VESC ERPM exceeded independent hard safety limit: {self.latest.erpm:.1f} > {hard_erpm:.1f}"
                )

    # ---- timed state holds ------------------------------------------------
    def hold(
        self,
        *,
        speed_mps: float,
        raw_servo: float | None,
        duration_s: float,
        phase: str,
        segment_id: str,
        capture: bool,
        centre_raw_servo: float | None = None,
        begin_window_fields: tuple[str, ...] = (),
        trial_id: str | None = None,
        target_abs_yaw_change_rad: float | None = None,
        minimum_duration_s: float = 0.0,
        maximum_duration_s: float | None = None,
        **event_metadata: Any,
    ) -> dict[str, float]:
        hz = float(self.cfg["session"]["command_publish_hz"])
        period = 1.0 / hz
        phase_payload = {"capture": capture, "speed_mps": speed_mps, "raw_servo_target": raw_servo}
        target_yaw = (
            float(target_abs_yaw_change_rad)
            if target_abs_yaw_change_rad is not None else math.nan
        )
        yaw_target_enabled = math.isfinite(target_yaw) and target_yaw > 0.0
        maximum_s = float(
            maximum_duration_s
            if maximum_duration_s is not None
            else (1.5 * duration_s if yaw_target_enabled else duration_s)
        )
        maximum_s = max(float(duration_s), maximum_s) if yaw_target_enabled else float(duration_s)
        minimum_s = max(0.0, min(float(minimum_duration_s), maximum_s))
        phase_payload.update({
            "target_abs_yaw_change_rad": target_yaw if yaw_target_enabled else None,
            "minimum_duration_s": minimum_s,
            "maximum_duration_s": maximum_s,
        })
        phase_payload.update(event_metadata)
        self.event.emit("phase_start", phase=phase, segment_id=segment_id, trial_id=trial_id,
                        **phase_payload)
        if begin_window_fields:
            self.begin_window(*begin_window_fields)
        start = time.monotonic()
        previous = start
        deadline = start + maximum_s
        integrated_abs_yaw = 0.0
        try:
            while time.monotonic() < deadline:
                self.command(speed_mps, raw_servo)
                self.spin(period)
                self.check_safety(motion_expected=abs(speed_mps) > 1e-3)
                now = time.monotonic()
                dt_s = max(0.0, now - previous)
                previous = now
                if math.isfinite(self.latest.imu_gz):
                    integrated_abs_yaw += abs(float(self.latest.imu_gz)) * dt_s
                if (
                    yaw_target_enabled and now - start >= minimum_s
                    and integrated_abs_yaw >= target_yaw
                ):
                    break
        except Exception as exc:
            self.event.emit("safety_abort", phase=phase, segment_id=segment_id,
                            trial_id=trial_id, reason=repr(exc))
            self.neutral_drive(centre_raw_servo)
            raise
        summary = self.end_window() if begin_window_fields else {}
        elapsed_s = float(time.monotonic() - start)
        summary.update({
            "hold_elapsed_s": elapsed_s,
            "integrated_abs_imu_yaw_rad": float(integrated_abs_yaw),
            "target_abs_yaw_change_rad": target_yaw if yaw_target_enabled else math.nan,
            "yaw_target_reached": bool(
                yaw_target_enabled and integrated_abs_yaw >= target_yaw
            ),
        })
        end_payload = {"capture": capture, **summary}
        end_payload.update(event_metadata)
        self.event.emit("phase_end", phase=phase, segment_id=segment_id, trial_id=trial_id, **end_payload)
        return summary

    def establish_speed(
        self,
        *,
        speed_mps: float,
        raw_servo: float,
        centre_raw_servo: float,
        segment_id: str,
        trial_id: str,
    ) -> dict[str, float | bool]:
        """Exclude startup and wait for an operational steady-speed window."""
        cfg = self.cfg["motion_startup"]
        hz = float(self.cfg["session"]["command_publish_hz"])
        period = 1.0 / hz
        start = time.monotonic()
        startup_end = start + float(cfg["minimum_startup_s"])
        deadline = start + float(cfg["stability_timeout_s"])
        window_s = float(cfg["stability_window_s"])
        history: deque[tuple[float, float, float]] = deque()
        self.event.emit("motion_startup_begin", segment_id=segment_id, trial_id=trial_id,
                        speed_mps=speed_mps, raw_servo_target=raw_servo,
                        minimum_startup_s=float(cfg["minimum_startup_s"]))
        startup_event_sent = False
        while time.monotonic() < deadline:
            self.command(speed_mps, raw_servo)
            self.spin(period)
            self.check_safety(motion_expected=abs(speed_mps) > 1e-3)
            now = time.monotonic()
            if now >= startup_end and not startup_event_sent:
                self.event.emit("motion_startup_excluded_end", segment_id=segment_id, trial_id=trial_id)
                startup_event_sent = True
            if now < startup_end or not (math.isfinite(self.latest.odom_vx) and math.isfinite(self.latest.imu_ax)):
                continue
            history.append((now, self.latest.odom_vx, self.latest.imu_ax))
            while history and now - history[0][0] > window_s:
                history.popleft()
            # Evaluate once a full window of post-startup time has elapsed, over the
            # most recent <=window_s of samples. The old check required the oldest
            # buffered sample to be >= window_s old, but the trim above removes
            # anything older than window_s, so with discrete sampling that condition
            # almost never fired and the gate timed out even on a clean steady pass.
            if now - startup_end < window_s or len(history) < 3:
                continue
            speeds = np.asarray([x[1] for x in history], dtype=float)
            ax = np.asarray([x[2] for x in history], dtype=float)
            speed_median = float(np.median(speeds))
            speed_std = float(np.std(speeds))
            ax_median = float(np.median(ax))
            stable = (
                abs(speed_median - speed_mps) <= float(cfg["max_speed_error_mps"])
                and speed_std <= float(cfg["max_speed_std_mps"])
                and abs(ax_median) <= float(cfg["max_abs_longitudinal_accel_mps2"])
            )
            if stable:
                result = {"stable": True, "elapsed_s": float(now - start),
                          "odom_speed_median_mps": speed_median, "odom_speed_std_mps": speed_std,
                          "imu_ax_median_mps2": ax_median, "samples": int(len(history))}
                self.event.emit("motion_stable", segment_id=segment_id, trial_id=trial_id, **result)
                return result
        speeds = np.asarray([x[1] for x in history], dtype=float)
        ax = np.asarray([x[2] for x in history], dtype=float)
        result = {"stable": False, "elapsed_s": float(time.monotonic() - start),
                  "odom_speed_median_mps": float(np.median(speeds)) if len(speeds) else math.nan,
                  "odom_speed_std_mps": float(np.std(speeds)) if len(speeds) else math.nan,
                  "imu_ax_median_mps2": float(np.median(ax)) if len(ax) else math.nan,
                  "samples": int(len(history))}
        self.event.emit("motion_stability_timeout", segment_id=segment_id, trial_id=trial_id, **result)
        self.neutral_drive(centre_raw_servo)
        return result
