#!/usr/bin/env python3
"""
Steering Angle Gain Calibration via Half-Circle Test

Calibrates steering_angle_to_servo_gain by driving a half-circle at max
steering and measuring the straight-line distance from start to end.

VALIDATION STATUS:
    Tested and validated at 1 m/s.  At this speed tire slip is negligible
    and the kinematic bicycle model matches well.  Higher speeds have not
    been validated and may introduce slip errors.

Theory (kinematic bicycle model):
    beta  = arctan(0.5 * tan(delta))
    R     = L / (2 * sin(beta))
    D     = 2 * R   (diameter = distance from start to end of half-circle)

With default parameters (L=0.324m, delta=0.34rad):
    beta  = 0.1717 rad
    R     = 0.949 m
    D     = 1.898 m

Procedure:
    1. Lay a tape measure on the floor (≥2.5m)
    2. Place car at the 0 mark with the rear axle aligned
    3. Script drives at low speed with max steering
    4. Car traces a half-circle and crosses back over the tape
    5. Note where the rear axle crosses the tape → that's your measured distance
    6. Enter the measured distance when prompted
    7. Script computes the corrected steering_angle_to_servo_gain

Usage:
    python3 test_steering_gain.py [--speed 1.0] [--runs 3] [--direction left]
"""

import argparse
import time

import numpy as np
import rclpy

from common import (
    TestNode,
    DEFAULT_SERVO_GAIN, DEFAULT_SERVO_OFFSET,
    DEFAULT_SERVO_MIN, DEFAULT_SERVO_MAX,
    DEFAULT_WHEELBASE, DEFAULT_MAX_STEER,
    steering_angle_from_radius, radius_from_steering_angle,
)


def compute_half_circle_diameter(steering_angle: float, wheelbase: float) -> float:
    """
    Compute the expected half-circle diameter using the Ackermann model.

        beta = arctan(0.5 * tan(delta))
        R    = L / (2 * sin(beta))
        D    = 2 * R

    Args:
        steering_angle: Front-wheel steering angle (radians, positive)
        wheelbase: Distance between axles (meters)

    Returns:
        Expected straight-line distance from start to end of half-circle.
    """
    beta = np.arctan(0.5 * np.tan(steering_angle))
    R = wheelbase / (2.0 * np.sin(beta))
    return 2.0 * R


class SteeringGainNode(TestNode):

    def __init__(self, args):
        columns = [
            'odom_x', 'odom_y', 'odom_yaw',
            'odom_vx', 'cmd_speed', 'cmd_steering', 'phase',
        ]

        super().__init__(
            'steering_gain_test',
            'steering_gain',
            columns,
            max_speed=args.speed * 1.5,
            max_time=120.0,
        )

        self.test_speed = args.speed
        self.num_runs = args.runs
        self.wheelbase = args.wheelbase
        self.direction_name = args.direction.upper()  # 'LEFT' or 'RIGHT'

        # Direction sign for the steering angle command.
        # With negative gain: negative angle = left, positive angle = right.
        # With positive gain: the opposite.
        if DEFAULT_SERVO_GAIN < 0:
            self.direction = -1.0 if args.direction == 'left' else 1.0
        else:
            self.direction = 1.0 if args.direction == 'left' else -1.0

        # Max steering angle from current calibration
        self.max_steer = DEFAULT_MAX_STEER
        self.expected_diameter = compute_half_circle_diameter(
            self.max_steer, self.wheelbase)

        self.run_results = []

    def run_half_circle(self, run_idx: int):
        """Drive a single half-circle and collect the user's measurement."""
        self.get_logger().info(f"\n--- Run {run_idx + 1}/{self.num_runs} ---")
        self.get_logger().info(
            f"Steering angle: {np.degrees(self.max_steer):.1f}° "
            f"({self.direction_name.lower()})")
        self.get_logger().info(
            f"Expected half-circle diameter: {self.expected_diameter:.3f}m "
            f"({self.expected_diameter * 39.37:.1f} inches)")
        self.get_logger().info(
            "Place car at 0-mark, rear axle on the tape. "
            "Press Enter when ready...")
        input()

        self.countdown(3)

        steer_cmd = self.direction * self.max_steer
        expected_servo = DEFAULT_SERVO_GAIN * steer_cmd + DEFAULT_SERVO_OFFSET
        self.get_logger().info(
            f"  DEBUG: SERVO_GAIN={DEFAULT_SERVO_GAIN}, direction_name={self.direction_name}, "
            f"direction={self.direction}, max_steer={self.max_steer:.4f}, "
            f"steer_cmd={steer_cmd:.4f} rad, expected_servo={expected_servo:.4f}")
        start_x = self.odom_x
        start_y = self.odom_y
        start_yaw = self.odom_yaw

        # Estimate required driving time (half-circle arc length / speed)
        R = self.expected_diameter / 2.0
        arc_length = np.pi * R
        est_time = arc_length / self.test_speed
        max_drive_time = est_time * 3.0  # generous timeout

        self.get_logger().info(
            f"  Driving ~{arc_length:.2f}m arc, est. {est_time:.1f}s. "
            f"Press Ctrl+C if the car overshoots.")

        drive_start = time.monotonic()

        while True:
            self.send_command(self.test_speed, steer_cmd)
            rclpy.spin_once(self, timeout_sec=0.02)

            elapsed = time.monotonic() - drive_start

            if not self.safety.check():
                self.get_logger().error(
                    f"Safety abort: {self.safety.abort_reason}")
                self.stop_car()
                return None

            self.recorder.record(
                odom_x=self.odom_x,
                odom_y=self.odom_y,
                odom_yaw=self.odom_yaw,
                odom_vx=self.odom_vx,
                cmd_speed=self.test_speed,
                cmd_steering=steer_cmd,
                phase=f'run_{run_idx}',
            )

            # Detect half-circle completion: heading changed by ~180°
            # (use odom yaw — good enough at low speed for rough detection)
            yaw_diff = abs(self.odom_yaw - start_yaw)
            # Normalize to [0, 2*pi]
            yaw_diff = yaw_diff % (2 * np.pi)
            if yaw_diff > np.pi:
                yaw_diff = 2 * np.pi - yaw_diff

            if elapsed > 2.0 and yaw_diff > np.radians(170):
                self.get_logger().info("  Half-circle detected (≈180° heading change).")
                break

            if elapsed > max_drive_time:
                self.get_logger().warn(
                    "  Max drive time reached — stopping. May not have "
                    "completed the half-circle.")
                break

        # Stop the car
        self.stop_car()
        time.sleep(0.5)
        self.stop_car()

        # Odom-based straight-line distance (just for reference)
        dx = self.odom_x - start_x
        dy = self.odom_y - start_y
        odom_diameter = np.sqrt(dx**2 + dy**2)

        self.get_logger().info(f"  Odom straight-line distance: {odom_diameter:.3f}m "
                               f"(for reference only)")
        self.get_logger().info(f"  Expected diameter: {self.expected_diameter:.3f}m")
        self.get_logger().info("")

        measured_str = input(
            f"  Enter MEASURED distance on tape (meters), "
            f"or press Enter for {self.expected_diameter:.3f}m: ")

        if measured_str.strip():
            measured = float(measured_str)
        else:
            measured = self.expected_diameter

        return {
            'odom_diameter': odom_diameter,
            'measured_diameter': measured,
            'expected_diameter': self.expected_diameter,
        }

    def run_test(self):
        """Execute the full steering gain test."""
        if not self.wait_for_odom():
            return False

        direction_str = self.direction_name

        self.get_logger().info("=" * 60)
        self.get_logger().info("STEERING ANGLE GAIN — HALF-CIRCLE TEST")
        self.get_logger().info(f"Speed: {self.test_speed:.1f} m/s")
        self.get_logger().info(f"Direction: {direction_str}")
        self.get_logger().info(f"Max steering angle: {np.degrees(self.max_steer):.1f}°")
        self.get_logger().info(f"Wheelbase: {self.wheelbase:.3f}m")
        self.get_logger().info(f"Expected half-circle diameter: "
                               f"{self.expected_diameter:.3f}m "
                               f"({self.expected_diameter * 39.37:.1f} inches)")
        self.get_logger().info(f"Runs: {self.num_runs}")
        self.get_logger().info("")
        self.get_logger().info("Setup:")
        self.get_logger().info(
            "  1. Lay a tape measure on the floor (≥2.5m)")
        self.get_logger().info(
            f"  2. Place car at 0-mark, rear axle aligned")
        self.get_logger().info(
            f"  3. Tape extends in the {direction_str} turn direction")
        self.get_logger().info(
            "  4. Car will drive a half-circle at max steering")
        self.get_logger().info(
            "  5. Note where the rear axle crosses back over the tape")
        self.get_logger().info("=" * 60)

        self.recorder.start()
        self.safety.start()
        self.test_running = True

        for i in range(self.num_runs):
            if not self.test_running:
                break
            result = self.run_half_circle(i)
            if result is not None:
                self.run_results.append(result)

        self.stop_car()
        self.recorder.save()
        self.analyze()
        return True

    def analyze(self):
        """Compute corrected steering_angle_to_servo_gain."""
        if not self.run_results:
            self.get_logger().warn("No valid runs to analyze.")
            return

        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("ANALYSIS RESULTS")
        self.get_logger().info("=" * 60)

        corrections = []

        for i, r in enumerate(self.run_results):
            expected = r['expected_diameter']
            measured = r['measured_diameter']
            odom = r['odom_diameter']

            # Compute actual steering angle from measured diameter
            R_actual = measured / 2.0
            delta_actual = steering_angle_from_radius(R_actual, self.wheelbase)

            # Correction: new_gain = old_gain * (delta_cmd / delta_actual)
            correction = self.max_steer / delta_actual
            corrections.append(correction)

            self.get_logger().info(
                f"  Run {i+1}: expected={expected:.3f}m, "
                f"measured={measured:.3f}m, odom={odom:.3f}m, "
                f"δ_actual={np.degrees(delta_actual):.2f}°, "
                f"correction={correction:.4f}")

        avg_correction = np.mean(corrections)
        new_gain = DEFAULT_SERVO_GAIN * avg_correction

        # Recompute max steering with new gain
        new_max_steer_min = abs(
            (DEFAULT_SERVO_MIN - DEFAULT_SERVO_OFFSET) / new_gain)
        new_max_steer_max = abs(
            (DEFAULT_SERVO_MAX - DEFAULT_SERVO_OFFSET) / new_gain)
        new_max_steer = min(new_max_steer_min, new_max_steer_max)

        self.get_logger().info("")
        self.get_logger().info(f"  Average correction factor: {avg_correction:.4f}")
        self.get_logger().info(
            f"  Old steering_angle_to_servo_gain: {DEFAULT_SERVO_GAIN:.4f}")
        self.get_logger().info(
            f"  New steering_angle_to_servo_gain: {new_gain:.4f}")
        self.get_logger().info(
            f"  Max steering angle with new gain: "
            f"{np.degrees(new_max_steer):.2f}° ({new_max_steer:.4f} rad)")
        self.get_logger().info("")

        if abs(avg_correction - 1.0) < 0.03:
            self.get_logger().info(
                "  Gain looks good! (< 3% correction)")
        else:
            if measured > expected:
                direction = "increase (make more negative)"
            else:
                direction = "decrease (make less negative)"
            self.get_logger().info(
                f"  → {direction} the gain.")
            self.get_logger().info("  Update vesc.yaml with:")
            self.get_logger().info(
                f"    steering_angle_to_servo_gain: {new_gain:.4f}")

        self.get_logger().info("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description='F1/10th Steering Gain — Half-Circle Test')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='Driving speed in m/s (default: 1.0)')
    parser.add_argument('--runs', type=int, default=3,
                        help='Number of half-circle runs (default: 3)')
    parser.add_argument('--direction', type=str, default='left',
                        choices=['left', 'right'],
                        help='Turn direction (default: left)')
    parser.add_argument('--wheelbase', type=float, default=DEFAULT_WHEELBASE,
                        help=f'Wheelbase in meters (default: {DEFAULT_WHEELBASE})')
    args = parser.parse_args()

    rclpy.init()
    node = SteeringGainNode(args)

    try:
        node.run_test()
    finally:
        node.stop_car()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
