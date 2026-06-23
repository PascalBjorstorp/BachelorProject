#!/usr/bin/env python3
"""
Rolling Resistance Coast-Down Test for F1/10th Car

Measures the rolling resistance coefficient (c_roll) by performing a
coast-down test: accelerate to a moderate speed, then release the throttle
and let the car decelerate naturally under rolling resistance alone.

THEORY:
    When the motor current is zero, the only decelerating forces are:

        F_drag = F_roll + F_aero
               = c_roll * m * g + 0.5 * rho * Cd * A * v^2

    At F1/10th scale and v < 4 m/s, aerodynamic drag is negligible
    (< 0.03 N vs ~2.7 N rolling resistance), so:

        m * a_x ≈ -c_roll * m * g
        c_roll  ≈ -a_x / g

    The deceleration is measured from wheel odometry (ERPM), which is
    accurate during a free-rolling coast (no slip).

PROCEDURE:
    1. Calibrate IMU bias while stationary
    2. Accelerate to coast_speed (default 3.0 m/s)
    3. Release throttle by commanding motor current = 0
    4. Record (time, speed, imu_ax) until stopped
    5. Fit c_roll from the coast-down deceleration

NOTE: This test bypasses ackermann_to_vesc during the coast phase and
      publishes COMM_SET_CURRENT(0) directly to the VESC driver. This is
      necessary because ackermann_to_vesc's speed controller actively
      brakes (COMM_SET_CURRENT_BRAKE) when commanded speed ≈ 0 while the
      car is still moving. Zero motor current lets the motor freewheel.

Usage:
    python3 test_rolling_resistance.py [--coast-speed 3.0] [--runs 5]
"""

import argparse
import time

import numpy as np
import rclpy
from std_msgs.msg import Float64

import sys as _sys, os as _os  # noqa: E402
_sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..'))
from common import TestNode, DEFAULT_ERPM_GAIN


class RollingResistanceNode(TestNode):

    def __init__(self, args):
        columns = [
            'odom_vx', 'imu_ax', 'imu_ay', 'imu_az',
            'motor_rpm', 'motor_current', 'battery_voltage',
            'cmd_speed', 'phase'
        ]

        super().__init__(
            'rolling_resistance_test',
            'rolling_resistance',
            columns,
            max_speed=args.coast_speed * 1.3,
            max_time=0,
            max_distance=args.geofence
        )

        self.coast_speed = args.coast_speed
        self.mass = args.mass

        # Direct VESC motor current publisher — bypasses ackermann_to_vesc
        # so we can command zero current (true freewheel) instead of the
        # speed controller's active braking.
        self.current_pub = self.create_publisher(
            Float64, 'commands/motor/current', 10)

        # Data collected during coast phase
        self.coast_data = []

    def run_test(self):
        """Execute the rolling resistance coast-down test."""
        if not self.wait_for_sensors():
            return False

        self.get_logger().info("=" * 60)
        self.get_logger().info("ROLLING RESISTANCE COAST-DOWN TEST")
        self.get_logger().info(f"Coast speed: {self.coast_speed:.1f} m/s")
        self.get_logger().info(f"Vehicle mass: {self.mass:.3f} kg")
        self.get_logger().info("=" * 60)

        # ---- IMU Bias Calibration ----
        self.calibrate_imu_bias(duration=2.0)
        self.get_logger().info(f"IMU ax bias: {self.imu_bias_ax:.4f} m/s²")

        self.countdown(3)
        self.recorder.start()
        self.safety.start()
        self.test_running = True

        # ---- Phase 1: Accelerate to coast speed ----
        self.get_logger().info("\n--- Phase 1: ACCELERATE ---")
        phase_start = time.monotonic()

        while time.monotonic() - phase_start < 8.0:
            rclpy.spin_once(self, timeout_sec=0.005)
            if not self.safety.check():
                self.get_logger().error(f"Safety abort: {self.safety.abort_reason}")
                self.stop_car()
                self.test_running = False
                break

            self.send_command(self.coast_speed, 0.0)

            ax_corr = self.imu_ax - self.imu_bias_ax

            self.recorder.record(
                odom_vx=self.odom_vx, imu_ax=ax_corr,
                imu_ay=self.imu_ay, imu_az=self.imu_az,
                motor_rpm=self.motor_rpm, motor_current=self.motor_current,
                battery_voltage=self.battery_voltage,
                cmd_speed=self.coast_speed, phase='acceleration'
            )

            if abs(self.odom_vx) >= self.coast_speed * 0.95:
                self.get_logger().info(
                    f"  Reached target speed ({self.odom_vx:.2f} m/s)")
                break

        if not self.test_running:
            self.recorder.save()
            return False

        # Brief hold at coast speed to stabilize
        hold_start = time.monotonic()
        while time.monotonic() - hold_start < 0.5:
            rclpy.spin_once(self, timeout_sec=0.005)
            self.send_command(self.coast_speed, 0.0)

        # ---- Phase 2: Coast (release throttle) ----
        self.get_logger().info("\n--- Phase 2: COAST-DOWN ---")
        self.get_logger().info("  Releasing throttle (motor current = 0)")

        coast_start = time.monotonic()
        v_at_coast_start = abs(self.odom_vx)
        self.get_logger().info(f"  Speed at coast start: {v_at_coast_start:.2f} m/s")

        while abs(self.odom_vx) > 0.15 and time.monotonic() - coast_start < 30.0:
            rclpy.spin_once(self, timeout_sec=0.005)

            # Publish zero motor current directly to the VESC driver,
            # bypassing ackermann_to_vesc which would actively brake.
            # COMM_SET_CURRENT(0) lets the motor freewheel.
            current_msg = Float64()
            current_msg.data = 0.0
            self.current_pub.publish(current_msg)

            t_coast = time.monotonic() - coast_start
            ax_corr = self.imu_ax - self.imu_bias_ax

            self.coast_data.append({
                't': t_coast,
                'v': abs(self.odom_vx),
                'ax': ax_corr,
                'current': self.motor_current,
            })

            self.recorder.record(
                odom_vx=self.odom_vx, imu_ax=ax_corr,
                imu_ay=self.imu_ay, imu_az=self.imu_az,
                motor_rpm=self.motor_rpm, motor_current=self.motor_current,
                battery_voltage=self.battery_voltage,
                cmd_speed=0.0, phase='coast'
            )

        coast_duration = time.monotonic() - coast_start
        self.get_logger().info(f"  Coast duration: {coast_duration:.2f} s")

        self.stop_car()
        time.sleep(0.5)
        self.stop_car()

        self.recorder.save()
        self.analyze(v_at_coast_start)
        return True

    def analyze(self, v_start):
        """Analyze coast-down data to extract rolling resistance."""
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("ANALYSIS RESULTS")
        self.get_logger().info("=" * 60)

        if len(self.coast_data) < 20:
            self.get_logger().warn("Not enough coast data for analysis")
            return

        t_arr = np.array([d['t'] for d in self.coast_data])
        v_arr = np.array([d['v'] for d in self.coast_data])
        ax_arr = np.array([d['ax'] for d in self.coast_data])
        current_arr = np.array([d['current'] for d in self.coast_data])

        # Method 1: From v(t) slope — fit linear v(t) = v0 - a*t
        # Rolling resistance gives constant deceleration, so v(t) is linear
        # Use only the middle portion where speed is well above noise floor
        v_high = v_arr > 0.5  # above noise floor
        if np.sum(v_high) > 10:
            t_fit = t_arr[v_high]
            v_fit = v_arr[v_high]
            # Linear fit: v = p[0]*t + p[1]
            p = np.polyfit(t_fit, v_fit, 1)
            a_coast_vfit = abs(p[0])  # deceleration magnitude (should be positive)

            c_roll_vfit = a_coast_vfit / 9.81
            self.get_logger().info(f"\n  Method 1: v(t) linear fit")
            self.get_logger().info(f"    Deceleration: {a_coast_vfit:.3f} m/s²")
            self.get_logger().info(f"    c_roll = {c_roll_vfit:.4f}")
            self.get_logger().info(f"    F_roll = {c_roll_vfit * self.mass * 9.81:.2f} N")
        else:
            c_roll_vfit = None
            self.get_logger().warn("  Method 1: Not enough high-speed coast data")

        # Method 2: From IMU ax during coast
        if np.sum(v_high) > 10:
            ax_coast = ax_arr[v_high]
            a_coast_imu = abs(np.mean(ax_coast))
            c_roll_imu = a_coast_imu / 9.81
            self.get_logger().info(f"\n  Method 2: IMU acceleration mean")
            self.get_logger().info(f"    Deceleration: {a_coast_imu:.3f} m/s²")
            self.get_logger().info(f"    c_roll = {c_roll_imu:.4f}")
        else:
            c_roll_imu = None

        # Check motor current during coast (should be ~0 for true coasting)
        coast_current = current_arr[v_high] if np.sum(v_high) > 0 else current_arr
        mean_current = np.mean(np.abs(coast_current))
        self.get_logger().info(f"\n  Motor current during coast: {mean_current:.2f} A "
                               f"(should be near 0 for true coasting)")
        if mean_current > 1.0:
            self.get_logger().warn("  WARNING: Significant motor current during coast — "
                                   "VESC may be actively braking!")
            self.get_logger().warn("  The c_roll estimate may be too high.")

        # Best estimate: prefer v(t) fit (less sensitive to IMU bias)
        if c_roll_vfit is not None:
            best_c_roll = c_roll_vfit
            method = "v(t) linear fit"
        elif c_roll_imu is not None:
            best_c_roll = c_roll_imu
            method = "IMU mean"
        else:
            self.get_logger().error("Could not estimate c_roll")
            return

        self.get_logger().info(f"\n--- Parameters for Trajectory Optimizer (mintime) ---")
        self.get_logger().info(f"  c_roll: {best_c_roll:.4f} (from {method})")
        self.get_logger().info(f"  F_roll: {best_c_roll * self.mass * 9.81:.2f} N")
        self.get_logger().info(f"  (Sanity: typical RC car c_roll = 0.01 – 0.15)")

        # Auto-save to vehicle_params.yaml
        from common import update_vehicle_params
        update_vehicle_params({
            'c_roll': round(float(best_c_roll), 4),
        }, status='TESTED', logger=self.get_logger())

        self.get_logger().info("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description='F1/10th Rolling Resistance Coast-Down Test')
    parser.add_argument('--coast-speed', type=float, default=3.0,
                        help='Speed to coast down from (m/s, default: 3.0)')
    parser.add_argument('--mass', type=float, default=3.314,
                        help='Vehicle mass in kg (default: 3.314)')
    parser.add_argument('--geofence', type=float, default=35.0,
                        help='Max distance from start before abort (m, default: 35.0)')
    parser.add_argument('--runs', type=int, default=5,
                        help='Number of complete test runs (default: 5)')
    args = parser.parse_args()

    rclpy.init()
    for run_idx in range(args.runs):
        if args.runs > 1:
            print(f"\n{'='*60}")
            print(f"RUN {run_idx + 1}/{args.runs}")
            print(f"{'='*60}\n")
        node = RollingResistanceNode(args)
        try:
            node.run_test()
        finally:
            node.stop_car()
            node.destroy_node()
        if run_idx < args.runs - 1:
            print("\n  >>> Reposition the car for the next run.")
            input("  >>> Press ENTER when ready...")

    rclpy.shutdown()
    print("\nAll runs complete.")


if __name__ == '__main__':
    main()
