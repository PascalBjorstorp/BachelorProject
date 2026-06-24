#!/usr/bin/env python3
"""Shadow ACCEL_TO_CURRENT mapper for full-envelope candidate verification.

The production ``AckermannToVesc`` node remains responsible for steering.  Its
motor outputs are remapped away during Stage 12.  This node consumes the same
Ackermann acceleration request and the candidate odometry estimate, then
publishes the *selected* drive/brake current through the calibration selector.

Two modes are supported:

``scalar``
    Existing low-slip A/(m/s²) gains plus drag feed-forward.

``traction_surface``
    Inverts the empirically fitted full-envelope net-acceleration surface

      a_net(I, v) = c1 I + c2 I² + c3 I v + c4 I v²,

    separately for drive and brake.  It is a bounded, causal reference
    implementation used only in the reversible candidate verification launch.
    The production C++ port must reproduce this exact mapping before a
    permanent configuration is installed.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import rclpy
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64

from traction_surface import invert_monotone_envelope



class CandidateAccelMap(Node):
    def __init__(self, patch_path: Path) -> None:
        super().__init__('candidate_accel_map')
        data = yaml.safe_load(patch_path.read_text(encoding='utf-8')) or {}
        self.p = dict(data.get('ackermann_to_vesc_node', {}).get('ros__parameters', {}))

        self.model = str(self.p.get('acceleration_command_model', 'scalar')).strip().lower()
        if self.model not in {'scalar', 'traction_surface'}:
            raise ValueError(f'unsupported acceleration_command_model: {self.model!r}')

        self.drive_gain = float(self.p.get('accel_to_current_gain', 0.0))
        self.brake_gain = float(self.p.get('accel_to_brake_gain', 0.0))
        self.deadzone = max(0.0, float(self.p.get('accel_deadzone', 0.02)))
        # Below this speed a=0 commands zero current so the car can come to rest;
        # above it, a=0 means HOLD speed and must apply the drag-compensating
        # current instead of coasting.
        self.hold_speed_min = max(0.0, float(self.p.get('hold_speed_min_mps', 0.12)))
        self.drag_c0 = max(0.0, float(self.p.get('accel_drag_coulomb', 0.0)))
        self.drag_c1 = max(0.0, float(self.p.get('accel_drag_viscous', 0.0)))
        self.drag_c2 = max(0.0, float(self.p.get('accel_drag_quadratic', 0.0)))
        self.max_drive = max(0.0, float(self.p.get('max_drive_current', 0.0)))
        self.max_brake = max(0.0, float(self.p.get('max_brake_current', 0.0)))

        self.drive_surface = self._surface('drive')
        self.brake_surface = self._surface('brake')
        self.vx = 0.0

        self.current_pub = self.create_publisher(Float64, '/erpm_calibration/motor_from_ackermann/current', 100)
        self.brake_pub = self.create_publisher(Float64, '/erpm_calibration/motor_from_ackermann/brake', 100)
        self.debug = {
            name: self.create_publisher(Float64, f'/erpm_calibration/candidate_accel/{name}', 100)
            for name in ('target_accel', 'target_net_accel', 'speed_used', 'drive_current_cmd', 'brake_current_cmd')
        }
        self.create_subscription(AckermannDriveStamped, '/ackermann_cmd', self._cmd, 100)
        self.create_subscription(Odometry, '/erpm_calibration/candidate_odom', self._odom, 100)

    def _surface(self, polarity: str) -> tuple[float, float, float, float]:
        return tuple(float(self.p.get(f'accel_surface_{polarity}_{c}', 0.0)) for c in ('c1', 'c2', 'c3', 'c4'))

    @staticmethod
    def _float(value: float) -> Float64:
        msg = Float64()
        msg.data = float(value)
        return msg

    def _debug(self, name: str, value: float) -> None:
        self.debug[name].publish(self._float(value))

    def _odom(self, msg: Odometry) -> None:
        value = float(msg.twist.twist.linear.x)
        if math.isfinite(value):
            self.vx = value

    def _drag(self, speed: float) -> float:
        v = abs(speed)
        return self.drag_c0 + self.drag_c1 * v + self.drag_c2 * v * v

    def _invert_surface(self, target: float, speed: float, coeff: tuple[float, float, float, float], limit: float) -> float:
        return invert_monotone_envelope(target, speed, coeff, limit)

    def _drive_current(self, accel_cmd: float, speed: float) -> tuple[float, float]:
        # To achieve a positive ground acceleration command, the motor needs to
        # overcome both the requested acceleration and measured coast-down drag.
        target = max(0.0, accel_cmd + self._drag(speed))
        if self.model == 'traction_surface':
            current = self._invert_surface(target, speed, self.drive_surface, self.max_drive)
        else:
            current = self.drive_gain * target
        return min(max(current, 0.0), self.max_drive), target

    def _brake_current(self, accel_cmd: float, speed: float) -> tuple[float, float]:
        # Natural drag already decelerates the car.  Explicit brake force is the
        # remaining required deceleration magnitude.
        target = max(0.0, abs(accel_cmd) - self._drag(speed))
        if self.model == 'traction_surface':
            current = self._invert_surface(target, speed, self.brake_surface, self.max_brake)
        else:
            current = self.brake_gain * target
        return min(max(current, 0.0), self.max_brake), target

    def _cmd(self, msg: AckermannDriveStamped) -> None:
        accel = float(msg.drive.acceleration)
        speed = float(self.vx)
        if not math.isfinite(accel):
            accel = 0.0
        if not math.isfinite(speed):
            speed = 0.0

        drive = brake = target = 0.0
        if accel > self.deadzone:
            drive, target = self._drive_current(accel, speed)
            self.current_pub.publish(self._float(drive))
        elif accel < -self.deadzone:
            brake, target = self._brake_current(accel, speed)
            self.brake_pub.publish(self._float(brake))
        elif abs(speed) > self.hold_speed_min:
            # a == 0 means HOLD speed: apply the drag-compensating current
            # (= drive current for a net acceleration of 0) so the car does not
            # coast down. This is the fix for "commanding a=0 slowly coasts".
            drive, target = self._drive_current(0.0, speed)
            self.current_pub.publish(self._float(drive))
        else:
            # Near rest, command zero so the vehicle can actually stop.
            self.current_pub.publish(self._float(0.0))

        self._debug('target_accel', accel)
        self._debug('target_net_accel', target)
        self._debug('speed_used', speed)
        self._debug('drive_current_cmd', drive)
        self._debug('brake_current_cmd', brake)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--candidate-patch', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = CandidateAccelMap(args.candidate_patch.resolve())
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
