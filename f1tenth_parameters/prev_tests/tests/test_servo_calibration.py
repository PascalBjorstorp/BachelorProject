#!/usr/bin/env python3
"""
Servo Calibration Test for F1/10th Car

Measures the actual wheel angle at multiple commanded steering angles to
characterise the servo linkage nonlinearity. Outputs polynomial correction
coefficients that can be used in ackermann_to_vesc to compensate.

The Ackermann steering linkage is inherently nonlinear: at small deflections
the servo produces more wheel angle per unit displacement than at large
deflections (or vice versa). A single linear gain cannot map all commanded
angles accurately. This test:

  1. Drives circles at low speed (0.5 m/s) at several steering angles
  2. Measures the actual turning radius from IMU yaw rate (R = v / |ω|)
  3. Back-computes the actual wheel angle: δ_actual = atan(L / R)
  4. Fits a polynomial: δ_corrected = c2·δ² + c1·δ + c0  (sign-preserving)
  5. Prints correction coefficients for vesc.yaml

The correction maps commanded angle → pre-compensated angle such that the
physical result matches the command. Applied in ackermann_to_vesc as:
  abs_corrected = c2*|δ|² + c1*|δ| + c0
  servo_pos = gain * copysign(abs_corrected, δ) + offset

Usage:
    python3 test_servo_calibration.py
    python3 test_servo_calibration.py --angles 0.10,0.20,0.30,0.40
    python3 test_servo_calibration.py --speed 0.75 --laps 3
"""

import argparse
import time

import numpy as np
import rclpy

import sys as _sys, os as _os
_sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..'))
from common import (
    TestNode, fit_circle, steering_angle_from_radius,
    radius_from_steering_angle, radius_from_imu,
    DEFAULT_WHEELBASE, DEFAULT_SERVO_GAIN, DEFAULT_MAX_STEER
)


class ServoCalibrationNode(TestNode):

    def __init__(self, args):
        columns = [
            'odom_x', 'odom_y', 'odom_yaw',
            'odom_vx', 'odom_omega',
            'imu_ay', 'imu_gz',
            'cmd_speed', 'cmd_steering',
            'phase'
        ]

        super().__init__(
            'servo_calibration_test',
            'servo_calibration_test',
            columns,
            max_speed=args.speed * 1.5,
            max_time=0.0  # no time limit — driven by angle list
        )

        self.angles = args.angles
        self.speed = args.speed
        self.num_laps = args.laps
        self.wheelbase = args.wheelbase
        self.direction = args.direction

        # Direction sign
        if DEFAULT_SERVO_GAIN < 0:
            self.dir_sign = -1.0 if args.direction == 'left' else 1.0
        else:
            self.dir_sign = 1.0 if args.direction == 'left' else -1.0

        self.results = []  # list of (angle_cmd, angle_actual, R_imu, R_odom)

    def run_circle(self, steer_magnitude: float):
        """Drive a circle at the given steering magnitude and measure radius."""
        steer = self.dir_sign * steer_magnitude

        predicted_r = radius_from_steering_angle(steer_magnitude, self.wheelbase)
        circumference = 2.0 * np.pi * predicted_r
        total_distance = circumference * self.num_laps
        record_time = total_distance / self.speed + 5.0

        self.get_logger().info(
            f"  δ_cmd = {steer_magnitude:.3f} rad ({np.degrees(steer_magnitude):.1f}°), "
            f"R_pred = {predicted_r:.2f}m, "
            f"time ≈ {record_time:.0f}s")

        # Accelerate straight first
        self.send_command(self.speed, 0.0)
        self.spin_for(1.0)

        # Start turning
        self.send_command(self.speed, steer)

        # Settle into circle
        self.spin_for(2.0)

        # Collect data
        xs, ys = [], []
        gzs, vxs = [], []

        start = time.monotonic()
        while time.monotonic() - start < record_time:
            rclpy.spin_once(self, timeout_sec=0.02)

            if not self.safety.check():
                self.get_logger().error(f"Safety abort: {self.safety.abort_reason}")
                self.stop_car()
                return None

            self.send_command(self.speed, steer)

            xs.append(self.odom_x)
            ys.append(self.odom_y)
            gzs.append(self.imu_gz)
            vxs.append(self.odom_vx)

            self.recorder.record(
                odom_x=self.odom_x,
                odom_y=self.odom_y,
                odom_yaw=self.odom_yaw,
                odom_vx=self.odom_vx,
                odom_omega=self.odom_omega,
                imu_ay=self.imu_ay,
                imu_gz=self.imu_gz,
                cmd_speed=self.speed,
                cmd_steering=steer,
                phase=f'circle_d{steer_magnitude:.3f}'
            )

        if len(xs) < 50:
            self.get_logger().warn("  Not enough data")
            return None

        # Trim first/last 10% to remove transients
        n = len(xs)
        trim = int(n * 0.1)
        xs = np.array(xs[trim:n - trim])
        ys = np.array(ys[trim:n - trim])
        gzs = np.array(gzs[trim:n - trim])
        vxs = np.array(vxs[trim:n - trim])

        # Circle fit
        cx, cy, R_odom, residual = fit_circle(xs, ys)

        # IMU-based radius
        mask = (np.abs(vxs) > 0.05) & (np.abs(gzs) > 0.03)
        if mask.sum() < 20:
            self.get_logger().warn("  Not enough high-quality IMU samples")
            return None
        R_imu = np.median(np.abs(vxs[mask]) / np.abs(gzs[mask]))

        delta_actual = np.arctan(self.wheelbase / R_imu)
        error_pct = (delta_actual / steer_magnitude - 1.0) * 100

        self.get_logger().info(
            f"  Result: R_imu={R_imu:.3f}m, R_odom={R_odom:.3f}m, "
            f"δ_actual={delta_actual:.4f} rad ({np.degrees(delta_actual):.2f}°), "
            f"error={error_pct:+.1f}%")

        return {
            'angle_cmd': steer_magnitude,
            'angle_actual': delta_actual,
            'R_imu': R_imu,
            'R_odom': R_odom,
            'error_pct': error_pct,
        }

    def run_test(self):
        """Run servo calibration across all angles."""
        if not self.wait_for_sensors():
            return False

        self.get_logger().info("=" * 60)
        self.get_logger().info("SERVO CALIBRATION TEST")
        self.get_logger().info(f"Speed:     {self.speed:.2f} m/s")
        self.get_logger().info(f"Direction: {self.direction}")
        self.get_logger().info(f"Angles:    {[f'{a:.3f}' for a in self.angles]}")
        self.get_logger().info(f"Laps:      {self.num_laps}")
        self.get_logger().info(f"Wheelbase: {self.wheelbase:.4f}m")
        self.get_logger().info(f"Gain:      {DEFAULT_SERVO_GAIN}")
        self.get_logger().info("=" * 60)

        self.countdown(5)
        self.recorder.start()
        self.safety.start()
        self.test_running = True

        for angle in self.angles:
            if not self.test_running:
                break

            self.get_logger().info(f"\n--- Angle {np.degrees(angle):.1f}° ---")
            result = self.run_circle(angle)
            if result is not None:
                self.results.append(result)

            self.stop_car()
            if angle != self.angles[-1] and self.test_running:
                self.get_logger().info(
                    "\n  >>> Reposition the car if needed.")
                input("  >>> Press ENTER for next angle...")
                for _ in range(20):
                    rclpy.spin_once(self, timeout_sec=0.005)

        self.stop_car()
        time.sleep(0.5)
        self.stop_car()

        self.recorder.save()
        self.analyze()
        return True

    def analyze(self):
        """Fit polynomial correction and print results."""
        if len(self.results) < 2:
            self.get_logger().warn("Need at least 2 angle measurements")
            return

        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("SERVO CALIBRATION RESULTS")
        self.get_logger().info("=" * 60)

        # --- Table ---
        self.get_logger().info(
            f"\n  {'δ_cmd':>8s} {'δ_actual':>9s} {'Error':>7s} {'R_imu':>7s}")
        self.get_logger().info(f"  {'-'*35}")
        for r in self.results:
            self.get_logger().info(
                f"  {np.degrees(r['angle_cmd']):7.2f}° "
                f"{np.degrees(r['angle_actual']):8.2f}° "
                f"{r['error_pct']:+6.1f}% "
                f"{r['R_imu']:7.3f}m")

        # --- Polynomial fit ---
        # We want: for each δ_cmd, compute δ_corrected such that the
        # physical linkage then produces exactly δ_cmd.
        #
        # Measured: δ_cmd → δ_actual (what the servo actually produced)
        # We need the inverse: δ_target → δ_cmd that produces δ_target.
        # i.e., given we WANT δ_actual = δ_target, what should we command?
        #
        # From measurements: actual = f(cmd), where f is the physical response.
        # We want: cmd = f_inv(target).
        # Fit: cmd = p(actual) — the inverse mapping from actual to command.
        # Then at runtime: corrected_cmd = p(desired_angle).

        cmds = np.array([r['angle_cmd'] for r in self.results])
        actuals = np.array([r['angle_actual'] for r in self.results])

        # Add origin constraint: 0 → 0
        cmds_ext = np.concatenate([[0.0], cmds])
        actuals_ext = np.concatenate([[0.0], actuals])

        # Fit inverse: cmd = c2 * actual^2 + c1 * actual + c0
        # This gives us: "to get actual angle X, command p(X)"
        # At runtime: corrected = p(desired_angle)
        if len(self.results) >= 3:
            coeffs = np.polyfit(actuals_ext, cmds_ext, 2)
        else:
            coeffs = np.polyfit(actuals_ext, cmds_ext, 1)
            coeffs = np.concatenate([[0.0], coeffs])

        c2, c1, c0 = coeffs

        self.get_logger().info(f"\n  Polynomial correction (inverse mapping):")
        self.get_logger().info(f"    δ_corrected = {c2:.6f}·|δ|² + {c1:.6f}·|δ| + {c0:.6f}")

        # Verify fit
        self.get_logger().info(f"\n  Verification:")
        self.get_logger().info(
            f"  {'δ_target':>8s} {'δ_corrected':>12s} {'predicted δ_actual':>18s} {'residual':>9s}")
        self.get_logger().info(f"  {'-'*50}")

        max_residual = 0
        for r in self.results:
            target = r['angle_cmd']
            # What we'd command after correction
            corrected = c2 * target**2 + c1 * target + c0
            # What the physical servo would produce (using measured f(cmd))
            # approximate: actual ≈ target (since corrected ≈ what's needed)
            # More precisely: interpolate from measured data
            predicted_actual = np.interp(corrected, cmds, actuals)
            residual_deg = np.degrees(abs(predicted_actual - target))
            max_residual = max(max_residual, residual_deg)
            self.get_logger().info(
                f"  {np.degrees(target):7.2f}° "
                f"{np.degrees(corrected):11.2f}° "
                f"{np.degrees(predicted_actual):17.2f}° "
                f"{residual_deg:8.2f}°")

        self.get_logger().info(f"\n  Max residual: {max_residual:.2f}°")

        # --- Output for vesc.yaml ---
        self.get_logger().info(f"\n  === Configuration for vesc.yaml ===")
        self.get_logger().info(f"  steering_angle_to_servo_gain: {DEFAULT_SERVO_GAIN}")
        self.get_logger().info(f"  steering_correction_c2: {c2:.6f}")
        self.get_logger().info(f"  steering_correction_c1: {c1:.6f}")
        self.get_logger().info(f"  steering_correction_c0: {c0:.6f}")

        # --- Linear gain comparison ---
        # Compute what a simple linear gain would give
        # Best linear fit: actual = k * cmd → gain_ideal = -1/k
        if len(actuals) >= 2:
            k_linear = np.polyfit(cmds, actuals, 1)[0]
            gain_linear = DEFAULT_SERVO_GAIN / k_linear
            self.get_logger().info(f"\n  For comparison — best linear gain: {gain_linear:.4f}")
            self.get_logger().info(f"  (current: {DEFAULT_SERVO_GAIN})")

        self.get_logger().info("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description='F1/10th Servo Calibration Test')
    parser.add_argument(
        '--angles', type=str, default='0.10,0.15,0.20,0.25,0.30,0.35,0.40',
        help='Comma-separated steering angles in radians (default: 0.10,...,0.40)')
    parser.add_argument(
        '--speed', type=float, default=0.5,
        help='Circle speed in m/s (default: 0.5 — low to minimize tire slip)')
    parser.add_argument(
        '--laps', type=int, default=3,
        help='Laps per angle (default: 3)')
    parser.add_argument(
        '--wheelbase', type=float, default=DEFAULT_WHEELBASE,
        help=f'Wheelbase in meters (default: {DEFAULT_WHEELBASE})')
    parser.add_argument(
        '--direction', choices=['left', 'right'], default='left',
        help='Circle direction (default: left)')
    args = parser.parse_args()

    args.angles = sorted([float(a) for a in args.angles.split(',')])

    # Validate angles against max
    for a in args.angles:
        if a > DEFAULT_MAX_STEER:
            print(f"WARNING: angle {a:.3f} rad exceeds max steering "
                  f"{DEFAULT_MAX_STEER:.3f} rad — skipping")
            args.angles.remove(a)

    if len(args.angles) < 2:
        print("ERROR: Need at least 2 valid steering angles")
        return

    rclpy.init()
    node = ServoCalibrationNode(args)
    try:
        node.run_test()
    except KeyboardInterrupt:
        node.stop_car()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
