#!/usr/bin/env python3
"""Exclusive motor command selector for ERPM/current calibration.

AckermannToVesc is remapped into `motor_from_ackermann/*`; this node is the
only publisher of the real VESC motor command topics.  Raw ERPM, current and
brake tests therefore cannot race the normal controller.  The selector mirrors
its chosen output and emits JSON status for MCAP audit.
"""
from __future__ import annotations
import json
from typing import Iterable
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, String

MODES = {"ackermann", "raw_erpm", "raw_current", "raw_brake", "neutral"}

class MotorSelector(Node):
    def __init__(self) -> None:
        super().__init__('motor_selector')
        self.mode = 'neutral'
        self.last_source = 'startup'
        self.last_values = {'speed': 0.0, 'current': 0.0, 'brake': 0.0}
        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_speed = self.create_publisher(Float64, '/commands/motor/speed', 100)
        self.pub_current = self.create_publisher(Float64, '/commands/motor/current', 100)
        self.pub_brake = self.create_publisher(Float64, '/commands/motor/brake', 100)
        self.sel_speed = self.create_publisher(Float64, '/erpm_calibration/motor_selected_speed', 100)
        self.sel_current = self.create_publisher(Float64, '/erpm_calibration/motor_selected_current', 100)
        self.sel_brake = self.create_publisher(Float64, '/erpm_calibration/motor_selected_brake', 100)
        self.status = self.create_publisher(String, '/erpm_calibration/motor_selector_status', status_qos)
        self.create_subscription(String, '/erpm_calibration/motor_mode', self._mode, status_qos)
        self.create_subscription(Float64, '/erpm_calibration/motor_raw_speed', self._raw_speed, 100)
        self.create_subscription(Float64, '/erpm_calibration/motor_raw_current', self._raw_current, 100)
        self.create_subscription(Float64, '/erpm_calibration/motor_raw_brake', self._raw_brake, 100)
        self.create_subscription(Float64, '/erpm_calibration/motor_from_ackermann/speed', self._ack_speed, 100)
        self.create_subscription(Float64, '/erpm_calibration/motor_from_ackermann/current', self._ack_current, 100)
        self.create_subscription(Float64, '/erpm_calibration/motor_from_ackermann/brake', self._ack_brake, 100)
        self._publish_status('startup')

    def _publish_status(self, reason: str) -> None:
        msg = String()
        msg.data = json.dumps({
            'reason': reason, 'mode': self.mode, 'last_source': self.last_source,
            'last_speed_erpm': self.last_values['speed'],
            'last_current_a': self.last_values['current'],
            'last_brake_a': self.last_values['brake'],
            'node_time_ns': int(self.get_clock().now().nanoseconds),
        }, sort_keys=True)
        self.status.publish(msg)

    @staticmethod
    def _msg(value: float) -> Float64:
        out = Float64(); out.data = float(value); return out

    def _publish(self, *, speed: float | None = None, current: float | None = None,
                 brake: float | None = None, source: str) -> None:
        # Explicitly zero competing command modalities before emitting the active
        # one. The active command is published last so VESC mode selection is
        # deterministic in the recorded sequence.
        if speed is not None:
            self.pub_current.publish(self._msg(0.0)); self.sel_current.publish(self._msg(0.0))
            self.pub_brake.publish(self._msg(0.0)); self.sel_brake.publish(self._msg(0.0))
            self.pub_speed.publish(self._msg(speed)); self.sel_speed.publish(self._msg(speed))
            self.last_values.update(speed=float(speed), current=0.0, brake=0.0)
        elif current is not None:
            self.pub_speed.publish(self._msg(0.0)); self.sel_speed.publish(self._msg(0.0))
            self.pub_brake.publish(self._msg(0.0)); self.sel_brake.publish(self._msg(0.0))
            self.pub_current.publish(self._msg(current)); self.sel_current.publish(self._msg(current))
            self.last_values.update(speed=0.0, current=float(current), brake=0.0)
        elif brake is not None:
            self.pub_speed.publish(self._msg(0.0)); self.sel_speed.publish(self._msg(0.0))
            self.pub_current.publish(self._msg(0.0)); self.sel_current.publish(self._msg(0.0))
            self.pub_brake.publish(self._msg(brake)); self.sel_brake.publish(self._msg(brake))
            self.last_values.update(speed=0.0, current=0.0, brake=float(brake))
        else:
            self.pub_speed.publish(self._msg(0.0)); self.sel_speed.publish(self._msg(0.0))
            self.pub_current.publish(self._msg(0.0)); self.sel_current.publish(self._msg(0.0))
            self.pub_brake.publish(self._msg(0.0)); self.sel_brake.publish(self._msg(0.0))
            self.last_values.update(speed=0.0, current=0.0, brake=0.0)
        self.last_source = source
        self._publish_status('command')

    def _mode(self, msg: String) -> None:
        requested = msg.data.strip().lower()
        if requested not in MODES:
            self.get_logger().error(f'Ignoring unknown motor selector mode: {requested!r}')
            return
        self.mode = requested
        if self.mode == 'neutral': self._publish(source='neutral')
        self._publish_status('mode_updated')

    def _raw_speed(self, msg: Float64) -> None:
        if self.mode == 'raw_erpm': self._publish(speed=msg.data, source='raw_erpm')
    def _raw_current(self, msg: Float64) -> None:
        if self.mode == 'raw_current': self._publish(current=msg.data, source='raw_current')
    def _raw_brake(self, msg: Float64) -> None:
        if self.mode == 'raw_brake': self._publish(brake=msg.data, source='raw_brake')
    def _ack_speed(self, msg: Float64) -> None:
        if self.mode == 'ackermann': self._publish(speed=msg.data, source='ackermann_speed')
    def _ack_current(self, msg: Float64) -> None:
        if self.mode == 'ackermann': self._publish(current=msg.data, source='ackermann_current')
    def _ack_brake(self, msg: Float64) -> None:
        if self.mode == 'ackermann': self._publish(brake=msg.data, source='ackermann_brake')

def main() -> int:
    rclpy.init(); node = MotorSelector()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
    return 0
if __name__ == '__main__': raise SystemExit(main())
