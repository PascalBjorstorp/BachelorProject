#!/usr/bin/env python3
"""Live 3D IMU visualizer for raw IMU data.

This tool subscribes to a sensor_msgs/msg/Imu topic and renders:
- linear acceleration vector (m/s^2)
- angular velocity vector (rad/s)

Example:
  python3 f1tenth_system/vesc/vesc_ackermann/scripts/imu_3d_visualizer.py \
      --topic /sensors/imu/raw
"""

from __future__ import annotations

import argparse
from collections import deque
import signal
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from matplotlib.lines import Line2D
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Imu


@dataclass
class ImuVectors:
    linear: np.ndarray
    angular: np.ndarray
    orientation_dir: np.ndarray
    stamp_s: float
    seq: int


class ImuSubscriber(Node):
    """Consumes IMU samples and keeps a filtered latest vector state."""

    def __init__(self, topic: str, alpha: float, history_size: int) -> None:
        super().__init__("imu_3d_visualizer")
        self.alpha = alpha
        self.latest: Optional[ImuVectors] = None
        self.filtered_linear = np.zeros(3, dtype=float)
        self.filtered_angular = np.zeros(3, dtype=float)
        self.linear_peak = 1.0
        self.angular_peak = 1.0
        self.sample_count = 0
        self.t_hist = deque(maxlen=history_size)
        self.ax_hist = deque(maxlen=history_size)
        self.ay_hist = deque(maxlen=history_size)
        self.az_hist = deque(maxlen=history_size)
        self.gx_hist = deque(maxlen=history_size)
        self.gy_hist = deque(maxlen=history_size)
        self.gz_hist = deque(maxlen=history_size)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.create_subscription(Imu, topic, self._on_imu, qos)
        self.get_logger().info(f"Listening to IMU topic: {topic}")

    @staticmethod
    def _quat_rotate_x_axis(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
        """Rotate the unit X axis by quaternion q to get a heading direction."""
        norm = float(np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw))
        if norm <= 1e-9:
            return np.array([1.0, 0.0, 0.0], dtype=float)

        x = qx / norm
        y = qy / norm
        z = qz / norm
        w = qw / norm

        r00 = 1.0 - 2.0 * (y * y + z * z)
        r10 = 2.0 * (x * y + z * w)
        r20 = 2.0 * (x * z - y * w)
        return np.array([r00, r10, r20], dtype=float)

    def _on_imu(self, msg: Imu) -> None:
        linear = np.array(
            [
                float(msg.linear_acceleration.x),
                float(msg.linear_acceleration.y),
                float(msg.linear_acceleration.z),
            ],
            dtype=float,
        )
        angular = np.array(
            [
                float(msg.angular_velocity.x),
                float(msg.angular_velocity.y),
                float(msg.angular_velocity.z),
            ],
            dtype=float,
        )
        orientation_dir = self._quat_rotate_x_axis(
            float(msg.orientation.x),
            float(msg.orientation.y),
            float(msg.orientation.z),
            float(msg.orientation.w),
        )

        if self.latest is None:
            self.filtered_linear = linear
            self.filtered_angular = angular
        else:
            self.filtered_linear = self.alpha * linear + (1.0 - self.alpha) * self.filtered_linear
            self.filtered_angular = self.alpha * angular + (1.0 - self.alpha) * self.filtered_angular

        stamp_s = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        self.latest = ImuVectors(
            linear=self.filtered_linear.copy(),
            angular=self.filtered_angular.copy(),
            orientation_dir=orientation_dir,
            stamp_s=stamp_s,
            seq=self.sample_count,
        )
        self.sample_count += 1
        self.linear_peak = max(self.linear_peak, float(np.linalg.norm(self.filtered_linear)))
        self.angular_peak = max(self.angular_peak, float(np.linalg.norm(self.filtered_angular)))

        self.t_hist.append(stamp_s)
        self.ax_hist.append(float(self.filtered_linear[0]))
        self.ay_hist.append(float(self.filtered_linear[1]))
        self.az_hist.append(float(self.filtered_linear[2]))
        self.gx_hist.append(float(self.filtered_angular[0]))
        self.gy_hist.append(float(self.filtered_angular[1]))
        self.gz_hist.append(float(self.filtered_angular[2]))

    def history_snapshot(self) -> Optional[Tuple[np.ndarray, ...]]:
        if not self.t_hist:
            return None
        return (
            np.asarray(self.t_hist, dtype=float),
            np.asarray(self.ax_hist, dtype=float),
            np.asarray(self.ay_hist, dtype=float),
            np.asarray(self.az_hist, dtype=float),
            np.asarray(self.gx_hist, dtype=float),
            np.asarray(self.gy_hist, dtype=float),
            np.asarray(self.gz_hist, dtype=float),
        )


class ImuFigure:
    """Four-panel figure for separated vector views plus time history."""

    def __init__(self, min_axis_limit: float, max_axis_limit: float) -> None:
        self.min_axis_limit = min_axis_limit
        self.max_axis_limit = max_axis_limit

        self.figure = plt.figure(figsize=(15, 10))
        self.ax_linear = self.figure.add_subplot(221, projection="3d")
        self.ax_angular = self.figure.add_subplot(222, projection="3d")
        self.ax_orientation = self.figure.add_subplot(223, projection="3d")
        self.ax_hist_panel = self.figure.add_subplot(224)

        self._configure_vector_axis(
            self.ax_linear,
            "Linear Acceleration [m/s^2]",
            "tab:red",
            "Combined linear",
        )
        self._configure_vector_axis(
            self.ax_angular,
            "Angular Velocity [rad/s]",
            "tab:blue",
            "Combined angular",
        )
        self._configure_vector_axis(
            self.ax_orientation,
            "Orientation Direction",
            "tab:green",
            "Combined orientation",
        )

        self.ax_hist_panel.set_title("Recent IMU Component History")
        self.ax_hist_panel.set_xlabel("Time In Buffer [s]")
        self.ax_hist_panel.set_ylabel("Linear Acc [m/s^2]")
        self.ax_hist_panel.grid(True, alpha=0.3)
        self.ax_hist_gyro = self.ax_hist_panel.twinx()
        self.ax_hist_gyro.set_ylabel("Angular Vel [rad/s]")

        self.ax_line, = self.ax_hist_panel.plot([], [], color="tab:red", linewidth=1.6, label="ax")
        self.ay_line, = self.ax_hist_panel.plot([], [], color="tab:orange", linewidth=1.6, label="ay")
        self.az_line, = self.ax_hist_panel.plot([], [], color="tab:pink", linewidth=1.6, label="az")
        self.gx_line, = self.ax_hist_gyro.plot([], [], color="tab:blue", linewidth=1.6, label="gx")
        self.gy_line, = self.ax_hist_gyro.plot([], [], color="tab:cyan", linewidth=1.6, label="gy")
        self.gz_line, = self.ax_hist_gyro.plot([], [], color="tab:green", linewidth=1.6, label="gz")

        hist_handles = [self.ax_line, self.ay_line, self.az_line, self.gx_line, self.gy_line, self.gz_line]
        self.ax_hist_panel.legend(handles=hist_handles, loc="upper left", ncol=2, fontsize=9)

        self.linear_quiver = None
        self.angular_quiver = None
        self.orientation_quiver = None
        self.linear_components = []
        self.angular_components = []
        self.orientation_components = []

        self.status_text = self.figure.text(0.01, 0.98, "Waiting for IMU data...", ha="left", va="top")
        plt.tight_layout(rect=[0.0, 0.0, 1.0, 0.95])

    def _configure_vector_axis(self, ax, title: str, vector_color: str, vector_label: str) -> None:
        ax.set_title(title)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_box_aspect((1.0, 1.0, 1.0))

        legend_items = [
            Line2D([0], [0], color=vector_color, lw=3, label=vector_label),
            Line2D([0], [0], color="tab:red", lw=2, linestyle="--", label="X component"),
            Line2D([0], [0], color="tab:orange", lw=2, linestyle="--", label="Y component"),
            Line2D([0], [0], color="tab:purple", lw=2, linestyle="--", label="Z component"),
        ]
        ax.legend(handles=legend_items, loc="upper right", fontsize=8)

    def _set_panel_limits(self, ax, limit: float) -> None:
        clamped_limit = max(0.5, min(limit, self.max_axis_limit))
        ax.set_xlim(-clamped_limit, clamped_limit)
        ax.set_ylim(-clamped_limit, clamped_limit)
        ax.set_zlim(-clamped_limit, clamped_limit)

    def _remove_if_exists(self, artist) -> None:
        if artist is None:
            return
        try:
            artist.remove()
        except ValueError:
            pass

    def _remove_artists(self, artists) -> None:
        for artist in artists:
            self._remove_if_exists(artist)

    def _draw_component_lines(self, ax, vector: np.ndarray):
        lines = []
        lines.append(ax.plot([0.0, vector[0]], [0.0, 0.0], [0.0, 0.0], color="tab:red", linestyle="--", linewidth=2.0)[0])
        lines.append(
            ax.plot([0.0, 0.0], [0.0, vector[1]], [0.0, 0.0], color="tab:orange", linestyle="--", linewidth=2.0)[0]
        )
        lines.append(
            ax.plot([0.0, 0.0], [0.0, 0.0], [0.0, vector[2]], color="tab:purple", linestyle="--", linewidth=2.0)[0]
        )
        return lines

    def _update_history(self, history: Optional[Tuple[np.ndarray, ...]]) -> None:
        if history is None:
            return

        t_hist, ax_hist, ay_hist, az_hist, gx_hist, gy_hist, gz_hist = history
        if t_hist.size == 0:
            return

        t_rel = t_hist - t_hist[0]
        self.ax_line.set_data(t_rel, ax_hist)
        self.ay_line.set_data(t_rel, ay_hist)
        self.az_line.set_data(t_rel, az_hist)
        self.gx_line.set_data(t_rel, gx_hist)
        self.gy_line.set_data(t_rel, gy_hist)
        self.gz_line.set_data(t_rel, gz_hist)

        x_max = max(float(t_rel[-1]), 1e-3)
        self.ax_hist_panel.set_xlim(0.0, x_max)

        acc_vals = np.concatenate((ax_hist, ay_hist, az_hist))
        acc_min = float(np.min(acc_vals))
        acc_max = float(np.max(acc_vals))
        acc_margin = 0.1 * (acc_max - acc_min) if acc_max > acc_min else 1.0
        self.ax_hist_panel.set_ylim(acc_min - acc_margin, acc_max + acc_margin)

        gyro_vals = np.concatenate((gx_hist, gy_hist, gz_hist))
        gyro_min = float(np.min(gyro_vals))
        gyro_max = float(np.max(gyro_vals))
        gyro_margin = 0.1 * (gyro_max - gyro_min) if gyro_max > gyro_min else 0.5
        self.ax_hist_gyro.set_ylim(gyro_min - gyro_margin, gyro_max + gyro_margin)

    def update(
        self,
        sample: Optional[ImuVectors],
        linear_peak: float,
        angular_peak: float,
        history: Optional[Tuple[np.ndarray, ...]],
    ) -> None:
        self._update_history(history)

        if sample is None:
            self.status_text.set_text("Waiting for IMU data...")
            self.figure.canvas.draw_idle()
            return

        linear_limit = max(self.min_axis_limit, 1.2 * linear_peak)
        angular_limit = max(1.0, 1.2 * angular_peak)
        orientation_limit = 1.5

        self._set_panel_limits(self.ax_linear, linear_limit)
        self._set_panel_limits(self.ax_angular, angular_limit)
        self._set_panel_limits(self.ax_orientation, orientation_limit)

        self._remove_if_exists(self.linear_quiver)
        self._remove_if_exists(self.angular_quiver)
        self._remove_if_exists(self.orientation_quiver)
        self._remove_artists(self.linear_components)
        self._remove_artists(self.angular_components)
        self._remove_artists(self.orientation_components)

        self.linear_components = self._draw_component_lines(self.ax_linear, sample.linear)
        self.angular_components = self._draw_component_lines(self.ax_angular, sample.angular)

        orient_norm = max(float(np.linalg.norm(sample.orientation_dir)), 1e-9)
        orient_unit = sample.orientation_dir / orient_norm
        orient_vector = orient_unit * 1.1
        self.orientation_components = self._draw_component_lines(self.ax_orientation, orient_vector)

        self.linear_quiver = self.ax_linear.quiver(
            0.0,
            0.0,
            0.0,
            sample.linear[0],
            sample.linear[1],
            sample.linear[2],
            color="tab:red",
            linewidth=2.8,
            arrow_length_ratio=0.14,
        )
        self.angular_quiver = self.ax_angular.quiver(
            0.0,
            0.0,
            0.0,
            sample.angular[0],
            sample.angular[1],
            sample.angular[2],
            color="tab:blue",
            linewidth=2.8,
            arrow_length_ratio=0.14,
        )
        self.orientation_quiver = self.ax_orientation.quiver(
            0.0,
            0.0,
            0.0,
            orient_vector[0],
            orient_vector[1],
            orient_vector[2],
            color="tab:green",
            linewidth=2.8,
            arrow_length_ratio=0.14,
        )

        lin_mag = float(np.linalg.norm(sample.linear))
        ang_mag = float(np.linalg.norm(sample.angular))
        self.status_text.set_text(
            (
                f"t={sample.stamp_s:.3f}s  sample={sample.seq}"
                f"   linear=({sample.linear[0]: .3f},{sample.linear[1]: .3f},{sample.linear[2]: .3f}) |a|={lin_mag:.3f}"
                f"   angular=({sample.angular[0]: .3f},{sample.angular[1]: .3f},{sample.angular[2]: .3f}) |w|={ang_mag:.3f}"
            )
        )
        self.figure.canvas.draw_idle()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Live 3D visualizer for raw IMU topic")
    parser.add_argument("--topic", default="/sensors/imu/raw", help="IMU topic (sensor_msgs/msg/Imu)")
    parser.add_argument(
        "--alpha",
        type=float,
        default=0.35,
        help="Exponential smoothing factor in [0,1] (higher = less smoothing)",
    )
    parser.add_argument("--fps", type=float, default=20.0, help="Target plot refresh rate")
    parser.add_argument(
        "--history-size",
        type=int,
        default=1200,
        help="Number of recent IMU samples kept for the history panel",
    )
    parser.add_argument("--min-axis-limit", type=float, default=4.0, help="Minimum axis range")
    parser.add_argument("--max-axis-limit", type=float, default=30.0, help="Maximum axis range")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not (0.0 <= args.alpha <= 1.0):
        raise ValueError("--alpha must be in [0, 1]")
    if args.fps <= 0.0:
        raise ValueError("--fps must be > 0")
    if args.history_size <= 0:
        raise ValueError("--history-size must be > 0")

    rclpy.init()
    node = ImuSubscriber(args.topic, args.alpha, args.history_size)
    viz = ImuFigure(min_axis_limit=args.min_axis_limit, max_axis_limit=args.max_axis_limit)

    should_stop = False

    def stop_handler(*_unused) -> None:
        nonlocal should_stop
        should_stop = True

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    print("IMU visualizer started. Close the plot window or Ctrl+C to stop.")
    print(f"Topic: {args.topic}")

    frame_period = 1.0 / args.fps
    next_draw = time.monotonic()

    try:
        while rclpy.ok() and not should_stop and plt.fignum_exists(viz.figure.number):
            rclpy.spin_once(node, timeout_sec=0.02)

            now = time.monotonic()
            if now >= next_draw:
                viz.update(node.latest, node.linear_peak, node.angular_peak, node.history_snapshot())
                plt.pause(0.001)
                next_draw = now + frame_period
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())