#!/usr/bin/env python3
"""Exclusive raw-servo selector with auditable redundant outputs.

Only this node publishes the real ``/commands/servo/position`` topic. It also
publishes the exact selected value and selector state on calibration topics so
that every command is recorded at four points:

raw request -> selected value -> VESC command bus -> VESC command echo.
"""
from __future__ import annotations

import argparse
import json
from typing import Iterable

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, String


class ServoSelector(Node):
    def __init__(self, raw_min: float, raw_max: float) -> None:
        super().__init__("servo_selector")
        self.declare_parameter("raw_min", float(raw_min))
        self.declare_parameter("raw_max", float(raw_max))
        self.raw_min = float(self.get_parameter("raw_min").value)
        self.raw_max = float(self.get_parameter("raw_max").value)
        if not self.raw_min < self.raw_max:
            raise ValueError("raw_min must be lower than raw_max")
        self.mode = "raw"
        self.last_source = "startup"
        self.last_requested: float | None = None
        self.last_selected: float | None = None
        self.add_on_set_parameters_callback(self._on_set_parameters)

        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.output = self.create_publisher(Float64, "/commands/servo/position", 100)
        self.selected_pub = self.create_publisher(Float64, "/steering_calibration/servo_selected", 100)
        self.status_pub = self.create_publisher(String, "/steering_calibration/servo_selector_status", status_qos)
        self.create_subscription(String, "/steering_calibration/servo_mode", self._mode_cb, status_qos)
        self.create_subscription(Float64, "/steering_calibration/servo_raw", self._raw_cb, 100)
        self.create_subscription(Float64, "/steering_calibration/servo_from_ackermann", self._ackermann_cb, 100)
        self._publish_status("startup")
        self.get_logger().info(f"Selector active in raw mode; range=[{self.raw_min:.6f}, {self.raw_max:.6f}]")

    def _on_set_parameters(self, parameters: Iterable[Parameter]) -> SetParametersResult:
        proposed_low, proposed_high = self.raw_min, self.raw_max
        for parameter in parameters:
            if parameter.name == "raw_min":
                proposed_low = float(parameter.value)
            if parameter.name == "raw_max":
                proposed_high = float(parameter.value)
        if not proposed_low < proposed_high:
            return SetParametersResult(successful=False, reason="raw_min must be lower than raw_max")
        self.raw_min, self.raw_max = proposed_low, proposed_high
        self._publish_status("limits_updated")
        return SetParametersResult(successful=True, reason="")

    def _mode_cb(self, msg: String) -> None:
        requested = msg.data.strip().lower()
        if requested not in {"raw", "ackermann"}:
            self.get_logger().error(f"Ignoring unknown selector mode: {requested!r}")
            return
        self.mode = requested
        self._publish_status("mode_updated")

    def _publish_status(self, reason: str) -> None:
        record = {
            "reason": reason,
            "node_time_ns": int(self.get_clock().now().nanoseconds),
            "mode": self.mode,
            "raw_min": self.raw_min,
            "raw_max": self.raw_max,
            "last_source": self.last_source,
            "last_requested": self.last_requested,
            "last_selected": self.last_selected,
        }
        message = String()
        message.data = json.dumps(record, sort_keys=True)
        self.status_pub.publish(message)

    def _publish(self, value: float, source: str) -> None:
        requested = float(value)
        selected = min(self.raw_max, max(self.raw_min, requested))
        self.last_source = source
        self.last_requested = requested
        self.last_selected = selected
        if selected != requested:
            self.get_logger().warn(
                f"{source} servo command clipped by active selector range: {requested:.6f} -> {selected:.6f}"
            )
        message = Float64()
        message.data = selected
        # Redundant publication of the *same* selected value makes a downstream
        # clamp or command-path discrepancy visible in the MCAP bag.
        self.selected_pub.publish(message)
        self.output.publish(message)
        self._publish_status("command")

    def _raw_cb(self, msg: Float64) -> None:
        if self.mode == "raw":
            self._publish(msg.data, "raw")

    def _ackermann_cb(self, msg: Float64) -> None:
        if self.mode == "ackermann":
            self._publish(msg.data, "ackermann")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw-min", type=float, required=True)
    parser.add_argument("--raw-max", type=float, required=True)
    args = parser.parse_args()
    if not args.raw_min < args.raw_max:
        raise SystemExit("raw-min must be lower than raw-max")
    rclpy.init()
    node = ServoSelector(args.raw_min, args.raw_max)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
