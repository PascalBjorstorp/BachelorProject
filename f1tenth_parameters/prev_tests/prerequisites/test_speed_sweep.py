#!/usr/bin/env python3
"""
Speed Calibration Sweep for F1/10th Car

Drives the car at multiple speeds (1-10 m/s in steps of 1 by default),
measuring actual distance traveled at each speed with a tape measure.
Then fits a linear model: ERPM = gain * v (through origin).

Procedure:
1. For each speed: drive a straight line, measure actual distance
2. Script records motor RPM and odom during each run
3. After all runs, fits a linear model
4. Outputs new speed_to_erpm_gain

Usage:
    python3 test_speed_sweep.py [--distance 5.0] [--min-speed 1] [--max-speed 10] [--step 1]
"""

import argparse
import csv
import os
import time

import numpy as np
import rclpy

import sys as _sys, os as _os  # noqa: E402
_sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..'))
from common import (
    TestNode, DEFAULT_ERPM_GAIN, DEFAULT_ERPM_OFFSET,
    DATA_DIR
)


class SpeedSweepNode(TestNode):

    def __init__(self, args):
        columns = [
            'odom_x', 'odom_y', 'odom_vx',
            'motor_rpm', 'imu_ax', 'cmd_speed', 'phase'
        ]

        speeds = np.arange(args.min_speed, args.max_speed + 0.01, args.step)
        max_speed = float(speeds[-1]) * 1.5

        super().__init__(
            'speed_sweep_test',
            'speed_sweep',
            columns,
            max_speed=max_speed,
            max_time=1800.0,  # 30 minutes for the full sweep
        )

        self.target_distance = args.distance
        self.speeds = [float(s) for s in speeds]
        self.run_results = []

    def run_single(self, speed: float, idx: int, total: int):
        """Drive at a given speed and collect measurement."""
        self.get_logger().info(f"\n--- Speed {idx+1}/{total}: {speed:.1f} m/s ---")
        self.get_logger().info(
            f"Will drive at {speed:.1f} m/s for ~{self.target_distance:.1f}m")
        self.get_logger().info("Place car at start mark. Press Enter when ready...")
        input()

        self.countdown(3)

        # Calibrate IMU accelerometer bias while stationary
        imu_bias_samples = []
        for _ in range(50):  # ~1s at 50Hz
            rclpy.spin_once(self, timeout_sec=0.02)
            imu_bias_samples.append(self.imu_ax)
        imu_ax_bias = np.mean(imu_bias_samples) if imu_bias_samples else 0.0
        self.get_logger().info(f"  IMU ax bias: {imu_ax_bias:+.3f} m/s²")

        # Flush odom and take position reference
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.005)

        phase_start = time.monotonic()
        odom_distance = 0.0
        prev_x = self.odom_x
        prev_y = self.odom_y
        speed_samples = []
        rpm_samples = []

        self.send_command(speed, 0.0)

        while odom_distance < self.target_distance * 1.3:
            rclpy.spin_once(self, timeout_sec=0.02)

            if not self.safety.check():
                self.get_logger().error(f"Safety abort: {self.safety.abort_reason}")
                self.stop_car()
                return None

            # Accumulate distance
            dx = self.odom_x - prev_x
            dy = self.odom_y - prev_y
            odom_distance += np.sqrt(dx**2 + dy**2)
            prev_x = self.odom_x
            prev_y = self.odom_y

            # Record when at stable speed (after 1s acceleration)
            elapsed = time.monotonic() - phase_start
            if elapsed > 1.0 and abs(self.odom_vx) > speed * 0.5:
                speed_samples.append(self.odom_vx)
                rpm_samples.append(self.motor_rpm)

            self.recorder.record(
                odom_x=self.odom_x,
                odom_y=self.odom_y,
                odom_vx=self.odom_vx,
                motor_rpm=self.motor_rpm,
                imu_ax=self.imu_ax,
                cmd_speed=speed,
                phase=f'speed_{speed:.1f}'
            )

            self.send_command(speed, 0.0)

            if odom_distance >= self.target_distance:
                break

        # Stop and accumulate braking distance using IMU integration
        # The ERPM-based odom misses tire slide distance. The IMU accelerometer
        # measures actual body deceleration even when wheels lock/slip.
        elapsed_time = time.monotonic() - phase_start
        self.stop_car()

        self.get_logger().info("  Braking (IMU-assisted)...")

        # Capture velocity at brake start from the last odom reading
        brake_velocity = abs(self.odom_vx)  # Initial velocity when braking starts
        imu_velocity = brake_velocity
        imu_brake_distance = 0.0
        odom_brake_distance = 0.0

        brake_start = time.monotonic()
        last_t = brake_start

        while time.monotonic() - brake_start < 5.0:
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.monotonic()
            dt = now - last_t
            last_t = now

            # Odom-based braking distance (misses tire slide)
            dx = self.odom_x - prev_x
            dy = self.odom_y - prev_y
            odom_brake_distance += np.sqrt(dx**2 + dy**2)
            prev_x = self.odom_x
            prev_y = self.odom_y

            # IMU-based braking distance (captures tire slide)
            # imu_ax is forward acceleration in body frame (negative when braking)
            # Subtract bias measured while stationary
            corrected_ax = self.imu_ax - imu_ax_bias
            imu_velocity += corrected_ax * dt
            imu_velocity = max(imu_velocity, 0.0)  # Can't go negative
            imu_brake_distance += imu_velocity * dt

            # Record braking data
            self.recorder.record(
                odom_x=self.odom_x,
                odom_y=self.odom_y,
                odom_vx=self.odom_vx,
                motor_rpm=self.motor_rpm,
                imu_ax=self.imu_ax,
                cmd_speed=0.0,
                phase=f'brake_{speed:.1f}'
            )

            self.stop_car()

            # Stop when both odom and IMU say car has stopped
            odom_stopped = abs(self.odom_vx) < 0.05
            imu_stopped = imu_velocity < 0.05
            if odom_stopped and imu_stopped and (now - brake_start) > 0.3:
                break

        # Use the LARGER of odom or IMU braking distance
        # (IMU captures slide, odom captures normal deceleration)
        brake_distance = max(odom_brake_distance, imu_brake_distance)
        odom_distance += brake_distance

        slide_distance = imu_brake_distance - odom_brake_distance
        brake_time = time.monotonic() - brake_start
        if slide_distance > 0.01:
            self.get_logger().info(
                f"  Tire slide detected: ~{slide_distance:.3f}m "
                f"(odom_brake={odom_brake_distance:.3f}m, imu_brake={imu_brake_distance:.3f}m, "
                f"brake_t={brake_time:.2f}s, v0={brake_velocity:.2f}m/s)")

        total_time = time.monotonic() - phase_start

        # Compute what ERPM was commanded for this speed
        erpm_cmd = DEFAULT_ERPM_GAIN * speed + DEFAULT_ERPM_OFFSET

        avg_speed = np.mean(speed_samples) if speed_samples else 0.0
        avg_rpm = np.mean(rpm_samples) if rpm_samples else 0.0
        rpm_achieved_pct = 100.0 * avg_rpm / erpm_cmd if erpm_cmd != 0 else 0

        self.get_logger().info(f"  Drive time: {elapsed_time:.2f}s, total: {total_time:.2f}s")
        self.get_logger().info(f"  Odom distance (inc. braking): {odom_distance:.3f}m")
        self.get_logger().info(f"  Brake: odom={odom_brake_distance:.3f}m, imu={imu_brake_distance:.3f}m, slide={slide_distance:.3f}m")
        self.get_logger().info(f"  Avg reported speed: {avg_speed:.3f} m/s")
        self.get_logger().info(f"  ERPM commanded: {erpm_cmd:.0f}, achieved: {avg_rpm:.0f} ({rpm_achieved_pct:.0f}%)")
        self.get_logger().info("")

        actual_str = input(
            f"  Enter actual distance (start to stop, meters), "
            f"or press Enter for {self.target_distance:.1f}m: ")

        if actual_str.strip():
            actual_distance = float(actual_str)
        else:
            actual_distance = self.target_distance

        actual_speed = actual_distance / total_time

        self.get_logger().info(f"  Actual speed: {actual_speed:.3f} m/s")

        return {
            'cmd_speed': speed,
            'erpm_cmd': erpm_cmd,
            'odom_distance': odom_distance,
            'actual_distance': actual_distance,
            'elapsed_time': elapsed_time,
            'total_time': total_time,
            'avg_reported_speed': avg_speed,
            'actual_speed': actual_speed,
            'avg_rpm': avg_rpm,
            'rpm_achieved_pct': rpm_achieved_pct,
            'odom_brake_distance': odom_brake_distance,
            'imu_brake_distance': imu_brake_distance,
            'slide_distance': slide_distance,
            'imu_ax_bias': imu_ax_bias,
            'brake_v0': brake_velocity,
        }

    def run_test(self):
        """Execute the full speed sweep."""
        if not self.wait_for_sensors(require_vesc=True):
            return False

        self.get_logger().info("=" * 60)
        self.get_logger().info("SPEED CALIBRATION SWEEP")
        self.get_logger().info(f"Speeds: {[f'{s:.0f}' for s in self.speeds]} m/s")
        self.get_logger().info(f"Distance per run: {self.target_distance:.1f}m")
        self.get_logger().info(f"Total runs: {len(self.speeds)}")
        self.get_logger().info("")
        self.get_logger().info("For each speed:")
        self.get_logger().info("  1. Place car at start mark")
        self.get_logger().info("  2. Car drives forward, then stops")
        self.get_logger().info("  3. Measure distance from start to where it stopped")
        self.get_logger().info("=" * 60)

        self.recorder.start()
        self.safety.start()
        self.test_running = True
        total = len(self.speeds)

        for i, speed in enumerate(self.speeds):
            if not self.test_running:
                break
            result = self.run_single(speed, i, total)
            if result is not None:
                self.run_results.append(result)

        self.stop_car()
        self.recorder.save()
        self.analyze()
        return True

    def analyze(self):
        """Fit the best ERPM model to all data points."""
        if len(self.run_results) < 2:
            self.get_logger().warn("Need at least 2 speed points for fitting")
            return

        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("SPEED SWEEP RESULTS")
        self.get_logger().info("=" * 60)

        # Print all data points
        self.get_logger().info(
            f"{'cmd':>6} {'odom_d':>7} {'tape_d':>7} "
            f"{'v_odom':>7} {'v_tape':>7} {'RPM':>7} {'ERPM%':>6} {'slide':>6}")
        for r in self.run_results:
            self.get_logger().info(
                f"{r['cmd_speed']:6.1f} {r['odom_distance']:7.3f} "
                f"{r['actual_distance']:7.3f} {r['avg_reported_speed']:7.3f} "
                f"{r['actual_speed']:7.3f} {r['avg_rpm']:7.0f} "
                f"{r['rpm_achieved_pct']:5.0f}% {r['slide_distance']:6.3f}")

        # Save summary CSV
        summary_path = os.path.join(
            DATA_DIR, f"speed_sweep_summary_{self.recorder.test_name}.csv")
        try:
            with open(summary_path, 'w', newline='') as f:
                if self.run_results:
                    writer = csv.DictWriter(f, fieldnames=self.run_results[0].keys())
                    writer.writeheader()
                    writer.writerows(self.run_results)
            self.get_logger().info(f"\nSummary saved to {summary_path}")
        except Exception as e:
            self.get_logger().warn(f"Could not save summary: {e}")

        # ── Fit ERPM model ──
        # We have: avg_rpm (ERPM from VESC) and actual_speed (from tape measure)
        # Fit: ERPM = a*v² + b*v (no offset, passes through origin)
        actual_speeds = np.array([r['actual_speed'] for r in self.run_results])
        rpms = np.array([r['avg_rpm'] for r in self.run_results])

        if np.any(rpms == 0) or np.any(actual_speeds == 0):
            self.get_logger().warn("Some runs have zero RPM or speed — check data quality")
            # Filter out bad data
            mask = (rpms != 0) & (actual_speeds != 0)
            actual_speeds_fit = actual_speeds[mask]
            rpms_fit = rpms[mask]
        else:
            actual_speeds_fit = actual_speeds
            rpms_fit = rpms

        # ── Odom-to-tape ratio method (always works, no RPM needed) ──
        odom_dists = np.array([r['odom_distance'] for r in self.run_results])
        tape_dists = np.array([r['actual_distance'] for r in self.run_results])
        mask_valid = tape_dists > 0
        if np.any(mask_valid):
            speed_ratios = odom_dists[mask_valid] / tape_dists[mask_valid]
            avg_ratio = np.mean(speed_ratios)
            corrected_erpm_gain = DEFAULT_ERPM_GAIN * avg_ratio
            self.get_logger().info(f"\n── Odom-to-Tape Ratio Method ──")
            self.get_logger().info(f"  Speed correction factor: {avg_ratio:.4f} ({(avg_ratio-1)*100:.1f}%)")
            self.get_logger().info(f"  Current speed_to_erpm_gain: {DEFAULT_ERPM_GAIN:.1f}")
            self.get_logger().info(f"  Corrected speed_to_erpm_gain: {corrected_erpm_gain:.1f}")
            for i, r in enumerate(self.run_results):
                if tape_dists[i] > 0:
                    self.get_logger().info(
                        f"    v={r['cmd_speed']:.0f}: odom={r['odom_distance']:.3f}m, "
                        f"tape={r['actual_distance']:.3f}m, "
                        f"ratio={r['odom_distance']/r['actual_distance']:.4f}")

        if len(actual_speeds_fit) < 2:
            self.get_logger().warn("Not enough valid RPM data points — using odom-to-tape ratio only")
            return

        # ── Model 1: Linear (gain only, no offset) ──
        # ERPM = b*v → b = sum(ERPM*v) / sum(v²)
        linear_gain = np.sum(rpms_fit * actual_speeds_fit) / np.sum(actual_speeds_fit**2)
        linear_erpm = linear_gain * actual_speeds_fit
        linear_rmse = np.sqrt(np.mean((rpms_fit - linear_erpm)**2))

        # ── Model 2: Linear with offset ──
        # ERPM = b*v + c
        A_lo = np.column_stack([actual_speeds_fit, np.ones_like(actual_speeds_fit)])
        result_lo = np.linalg.lstsq(A_lo, rpms_fit, rcond=None)
        lo_gain = result_lo[0][0]
        lo_offset = result_lo[0][1]
        lo_erpm = lo_gain * actual_speeds_fit + lo_offset
        lo_rmse = np.sqrt(np.mean((rpms_fit - lo_erpm)**2))

        self.get_logger().info("")
        self.get_logger().info("── Model Comparison ──")
        self.get_logger().info(f"  Linear (through origin, RECOMMENDED):")
        self.get_logger().info(f"    ERPM = {linear_gain:.1f} * v")
        self.get_logger().info(f"    RMSE: {linear_rmse:.1f} ERPM")
        self.get_logger().info(f"  Linear (with offset):")
        self.get_logger().info(f"    ERPM = {lo_gain:.1f} * v + {lo_offset:.1f}")
        self.get_logger().info(f"    RMSE: {lo_rmse:.1f} ERPM  (warning: offset breaks stop logic!)")

        # Verify at each speed
        self.get_logger().info("")
        self.get_logger().info("── Linear Model Verification ──")
        self.get_logger().info(f"{'v_actual':>8} {'RPM_meas':>10} {'RPM_model':>10} {'Error%':>8}")
        for v, rpm in zip(actual_speeds, rpms):
            model_rpm = linear_gain * v
            err_pct = 100.0 * (model_rpm - rpm) / rpm if rpm != 0 else 0
            self.get_logger().info(f"{v:8.2f} {rpm:10.0f} {model_rpm:10.0f} {err_pct:8.2f}%")

        self.get_logger().info("")
        self.get_logger().info("── Update vesc.yaml with: ──")
        self.get_logger().info(f"  speed_to_erpm_gain: {linear_gain:.1f}")
        self.get_logger().info(f"  speed_to_erpm_offset: 0.0")
        self.get_logger().info("")
        self.get_logger().info("── Update common.py with: ──")
        self.get_logger().info(f"  DEFAULT_ERPM_GAIN = {linear_gain:.1f}")
        self.get_logger().info(f"  DEFAULT_ERPM_OFFSET = 0.0")
        self.get_logger().info("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description='F1/10th Speed Calibration Sweep')
    parser.add_argument('--distance', type=float, default=5.0,
                        help='Distance per run (meters, default: 5.0)')
    parser.add_argument('--min-speed', type=float, default=1.0,
                        help='Minimum speed (m/s, default: 1.0)')
    parser.add_argument('--max-speed', type=float, default=10.0,
                        help='Maximum speed (m/s, default: 10.0)')
    parser.add_argument('--step', type=float, default=1.0,
                        help='Speed step (m/s, default: 1.0)')
    args = parser.parse_args()

    rclpy.init()
    node = SpeedSweepNode(args)

    try:
        node.run_test()
    finally:
        node.stop_car()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
