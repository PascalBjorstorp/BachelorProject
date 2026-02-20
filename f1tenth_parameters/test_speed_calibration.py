#!/usr/bin/env python3
"""
Speed Calibration Test for F1/10th Car

Verifies the speed_to_erpm_gain by driving the car over a known distance
at constant speed and comparing reported velocity with actual elapsed time.

Since the VESC uses back-EMF to estimate motor RPM (sensorless), and the
odometry velocity is derived from that same RPM via the gain we're calibrating,
we need an external reference: a measured distance on the ground.

Procedure:
1. Place the car at the start of a measured straight (default 5m)
2. Mark the start and end positions clearly
3. Run the test — car accelerates to target speed, holds it, then stops
4. When prompted, enter the actual distance traveled (measured by tape/markings)
5. Script computes corrected speed_to_erpm_gain

Usage:
    python3 test_speed_calibration.py [--distance 5.0] [--speed 2.0] [--runs 3]
"""

import argparse
import time

import numpy as np
import rclpy

from common import (
    TestNode, DEFAULT_ERPM_GAIN, DEFAULT_ERPM_OFFSET
)


class SpeedCalibrationNode(TestNode):
    
    def __init__(self, args):
        columns = [
            'odom_x', 'odom_y', 'odom_vx',
            'motor_rpm', 'cmd_speed', 'phase'
        ]
        
        super().__init__(
            'speed_calibration_test',
            'speed_calibration',
            columns,
            max_speed=args.speed * 1.5,
            max_time=60.0
        )
        
        self.target_speed = args.speed
        self.target_distance = args.distance
        self.num_runs = args.runs
        
        # Accumulated distances per run
        self.run_results = []
    
    def run_single(self, run_idx: int):
        """Execute a single calibration run."""
        self.get_logger().info(f"\n--- Run {run_idx + 1}/{self.num_runs} ---")
        self.get_logger().info(
            f"Will drive at {self.target_speed:.1f} m/s for ~{self.target_distance:.1f}m")
        self.get_logger().info("Place car at start mark. Press Enter when ready...")
        input()
        
        # Reset position reference
        start_x = self.odom_x
        start_y = self.odom_y
        
        self.countdown(3)
        
        # Accelerate and drive
        phase_start = time.monotonic()
        odom_distance = 0.0
        
        # Flush any stale odom messages before taking the position reference
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.01)
        
        prev_x = self.odom_x
        prev_y = self.odom_y
        speed_samples = []
        rpm_samples = []
        
        self.send_command(self.target_speed, 0.0)
        
        while odom_distance < self.target_distance * 1.3:  # overshoot margin
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
            if elapsed > 1.0 and abs(self.odom_vx) > self.target_speed * 0.5:
                speed_samples.append(self.odom_vx)
                rpm_samples.append(self.motor_rpm)
            
            self.recorder.record(
                odom_x=self.odom_x,
                odom_y=self.odom_y,
                odom_vx=self.odom_vx,
                motor_rpm=self.motor_rpm,
                cmd_speed=self.target_speed,
                phase=f'run_{run_idx}'
            )
            
            # Send command continuously
            self.send_command(self.target_speed, 0.0)
            
            # Check if we've traveled far enough
            if odom_distance >= self.target_distance:
                break
        
        # Stop commanding speed
        elapsed_time = time.monotonic() - phase_start
        self.stop_car()
        
        # Keep accumulating odom distance while braking (car coasts to a stop)
        self.get_logger().info("  Braking... accumulating coast distance")
        brake_start = time.monotonic()
        while time.monotonic() - brake_start < 3.0:  # max 3s braking window
            rclpy.spin_once(self, timeout_sec=0.02)
            dx = self.odom_x - prev_x
            dy = self.odom_y - prev_y
            step = np.sqrt(dx**2 + dy**2)
            odom_distance += step
            prev_x = self.odom_x
            prev_y = self.odom_y
            self.stop_car()  # keep commanding zero
            
            # Stop once the car has essentially halted
            if abs(self.odom_vx) < 0.05 and (time.monotonic() - brake_start) > 0.5:
                break
        
        total_time = time.monotonic() - phase_start
        
        # Compute odom-reported distance
        total_dx = self.odom_x - start_x
        total_dy = self.odom_y - start_y
        odom_straight_dist = np.sqrt(total_dx**2 + total_dy**2)
        
        avg_reported_speed = np.mean(speed_samples) if speed_samples else 0.0
        avg_rpm = np.mean(rpm_samples) if rpm_samples else 0.0
        
        self.get_logger().info(f"Run complete!")
        self.get_logger().info(f"  Drive time: {elapsed_time:.2f}s, total (inc. braking): {total_time:.2f}s")
        self.get_logger().info(f"  Odom accumulated distance (inc. braking): {odom_distance:.3f}m")
        self.get_logger().info(f"  Odom straight-line distance: {odom_straight_dist:.3f}m")
        self.get_logger().info(f"  Average reported speed (driving only): {avg_reported_speed:.3f} m/s")
        self.get_logger().info(f"  Average motor RPM: {avg_rpm:.1f}")
        
        # Ask for actual measured distance
        self.get_logger().info("")
        actual_dist_str = input(
            f"  Enter actual distance traveled (start to stop, meters), "
            f"or press Enter for {self.target_distance}m: ")
        
        if actual_dist_str.strip():
            actual_distance = float(actual_dist_str)
        else:
            actual_distance = self.target_distance
        
        return {
            'elapsed_time': elapsed_time,
            'total_time': total_time,
            'odom_distance': odom_distance,
            'actual_distance': actual_distance,
            'avg_reported_speed': avg_reported_speed,
            'avg_rpm': avg_rpm,
            'actual_avg_speed': actual_distance / total_time,
        }
    
    def run_test(self):
        """Execute the full speed calibration test."""
        if not self.wait_for_odom():
            return False
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("SPEED CALIBRATION TEST")
        self.get_logger().info(f"Target speed: {self.target_speed:.1f} m/s")
        self.get_logger().info(f"Target distance: {self.target_distance:.1f}m")
        self.get_logger().info(f"Number of runs: {self.num_runs}")
        self.get_logger().info("")
        self.get_logger().info("Setup:")
        self.get_logger().info(f"  1. Mark a straight line of ~{self.target_distance:.0f}m on the ground")
        self.get_logger().info(f"  2. Place the car at the start mark")
        self.get_logger().info(f"  3. Ensure the path is clear and straight")
        self.get_logger().info("=" * 60)
        
        self.recorder.start()
        self.safety.start()
        self.test_running = True
        
        for i in range(self.num_runs):
            result = self.run_single(i)
            if result is not None:
                self.run_results.append(result)
        
        self.stop_car()
        self.recorder.save()
        self.analyze()
        return True
    
    def analyze(self):
        """Analyze results and compute corrected gain."""
        if not self.run_results:
            self.get_logger().warn("No valid runs to analyze")
            return
        
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("ANALYSIS RESULTS")
        self.get_logger().info("=" * 60)
        
        # For each run, compute the correction factor
        # correction = odom_distance / actual_distance
        # new_gain = old_gain * correction  (if odom reads high, gain too low)
        corrections = []
        
        for i, r in enumerate(self.run_results):
            correction = r['odom_distance'] / r['actual_distance']
            corrections.append(correction)
            
            self.get_logger().info(
                f"  Run {i+1}: odom={r['odom_distance']:.3f}m, "
                f"actual={r['actual_distance']:.3f}m, "
                f"correction={correction:.4f}, "
                f"v_reported={r['avg_reported_speed']:.3f} m/s, "
                f"v_actual={r['actual_avg_speed']:.3f} m/s")
        
        avg_correction = np.mean(corrections)
        new_gain = DEFAULT_ERPM_GAIN * avg_correction
        
        self.get_logger().info("")
        self.get_logger().info(f"  Average correction factor: {avg_correction:.4f}")
        self.get_logger().info(f"  Old speed_to_erpm_gain: {DEFAULT_ERPM_GAIN:.1f}")
        self.get_logger().info(f"  New speed_to_erpm_gain: {new_gain:.1f}")
        self.get_logger().info("")
        
        if abs(avg_correction - 1.0) < 0.02:
            self.get_logger().info("  Calibration looks good! (< 2% error)")
        else:
            self.get_logger().info("  Update vesc.yaml with:")
            self.get_logger().info(f"    speed_to_erpm_gain: {new_gain:.1f}")
        
        self.get_logger().info("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description='F1/10th Speed Calibration Test')
    parser.add_argument('--distance', type=float, default=5.0,
                        help='Distance to drive (meters, default: 5.0)')
    parser.add_argument('--speed', type=float, default=2.0,
                        help='Target speed (m/s, default: 2.0)')
    parser.add_argument('--runs', type=int, default=3,
                        help='Number of calibration runs (default: 3)')
    args = parser.parse_args()
    
    rclpy.init()
    node = SpeedCalibrationNode(args)
    
    try:
        node.run_test()
    finally:
        node.stop_car()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
