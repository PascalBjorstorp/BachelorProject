#!/usr/bin/env python3
"""
Steering Calibration Test for F1/10th Car

Drives slowly forward while sweeping through servo positions to measure
the actual turning radius at each position. This validates/calibrates the
steering_angle_to_servo_gain and steering_angle_to_servo_offset parameters.

Procedure:
1. Car drives forward at low constant speed
2. Servo is swept from center to max-left, then center to max-right
3. At each position, holds for a few seconds while recording trajectory
4. Fits circles to trajectory segments to extract turning radii
5. Computes actual steering angle = atan(wheelbase / radius)
6. Fits linear model: servo = gain * angle + offset

Usage:
    python3 test_steering_calibration.py [--speed 1.0] [--hold-time 3.0] [--steps 7]
"""

import argparse
import time

import numpy as np
import rclpy

from common import (
    TestNode, DataRecorder, fit_circle, steering_angle_from_radius,
    DEFAULT_SERVO_MIN, DEFAULT_SERVO_MAX, DEFAULT_SERVO_OFFSET,
    DEFAULT_SERVO_GAIN, DEFAULT_WHEELBASE
)


class SteeringCalibrationNode(TestNode):
    
    def __init__(self, args):
        # Compute test duration estimate
        num_steps = args.steps * 2 + 1  # both sides + center
        max_time = num_steps * (args.hold_time + 1.0) + 30.0  # margin
        
        columns = [
            'cmd_servo', 'cmd_speed', 'cmd_steering_angle',
            'odom_x', 'odom_y', 'odom_yaw',
            'odom_vx', 'odom_omega',
            'imu_gz', 'phase'
        ]
        
        super().__init__(
            'steering_calibration_test',
            'steering_calibration',
            columns,
            max_speed=args.speed * 1.5,
            max_time=max_time
        )
        
        self.test_speed = args.speed
        self.hold_time = args.hold_time
        self.num_steps = args.steps
        self.wheelbase = args.wheelbase
        
        # Generate servo positions to test
        # From center toward servo_min (positive steering for negative gain)
        # and from center toward servo_max
        servo_center = DEFAULT_SERVO_OFFSET
        servo_range_left = np.linspace(
            servo_center, DEFAULT_SERVO_MIN + 0.02, args.steps + 1)
        servo_range_right = np.linspace(
            servo_center, DEFAULT_SERVO_MAX - 0.02, args.steps + 1)
        
        # Combine: center, left sweep, back to center, right sweep
        self.servo_positions = np.concatenate([
            [servo_center],           # start at center (straight)
            servo_range_left[1:],     # sweep left
            [servo_center],           # back to center
            servo_range_right[1:],    # sweep right
            [servo_center],           # end at center
        ])
        
        self.segment_data = []  # List of (servo_pos, x_array, y_array)
    
    def run_test(self):
        """Execute the steering calibration test."""
        if not self.wait_for_sensors():
            return False
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("STEERING CALIBRATION TEST")
        self.get_logger().info(f"Speed: {self.test_speed:.1f} m/s")
        self.get_logger().info(f"Hold time per position: {self.hold_time:.1f}s")
        self.get_logger().info(f"Number of positions: {len(self.servo_positions)}")
        self.get_logger().info(f"Estimated duration: {len(self.servo_positions) * (self.hold_time + 1.0):.0f}s")
        self.get_logger().info("Ensure ~3m of open space around the car.")
        self.get_logger().info("=" * 60)
        
        self.countdown(5)
        self.recorder.start()
        self.safety.start()
        self.test_running = True
        
        for i, servo_pos in enumerate(self.servo_positions):
            if not self.test_running:
                break
            
            # Compute expected steering angle from current calibration
            expected_angle = (servo_pos - DEFAULT_SERVO_OFFSET) / DEFAULT_SERVO_GAIN
            
            self.get_logger().info(
                f"Position {i+1}/{len(self.servo_positions)}: "
                f"servo={servo_pos:.3f}, expected_angle={np.degrees(expected_angle):.1f}°")
            
            # Command the steering position
            self.send_command(self.test_speed, expected_angle)
            
            # Allow 1 second for servo to reach position
            if not self.spin_for(1.0):
                break
            
            # Record data for hold_time seconds
            segment_x = []
            segment_y = []
            phase = f"servo_{servo_pos:.3f}"
            
            start_hold = time.monotonic()
            while time.monotonic() - start_hold < self.hold_time:
                rclpy.spin_once(self, timeout_sec=0.02)
                
                if not self.safety.check():
                    self.get_logger().error(f"Safety abort: {self.safety.abort_reason}")
                    self.stop_car()
                    self.test_running = False
                    break
                
                self.recorder.record(
                    cmd_servo=servo_pos,
                    cmd_speed=self.test_speed,
                    cmd_steering_angle=expected_angle,
                    odom_x=self.odom_x,
                    odom_y=self.odom_y,
                    odom_yaw=self.odom_yaw,
                    odom_vx=self.odom_vx,
                    odom_omega=self.odom_omega,
                    imu_gz=self.imu_gz,
                    phase=phase
                )
                
                segment_x.append(self.odom_x)
                segment_y.append(self.odom_y)
            
            if len(segment_x) > 10:
                self.segment_data.append((
                    servo_pos, 
                    np.array(segment_x), 
                    np.array(segment_y)
                ))
        
        # Stop the car
        self.stop_car()
        time.sleep(0.5)
        self.stop_car()
        
        # Save data
        csv_file = self.recorder.save()
        
        # Analyze results
        self.analyze()
        
        return True
    
    def analyze(self):
        """Analyze recorded data to extract steering calibration."""
        if len(self.segment_data) < 3:
            self.get_logger().warn("Not enough data segments for analysis")
            return
        
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("ANALYSIS RESULTS")
        self.get_logger().info("=" * 60)
        
        servo_values = []
        measured_angles = []
        
        for servo_pos, x, y in self.segment_data:
            # Skip near-center positions (radius too large to fit reliably)
            expected_angle = (servo_pos - DEFAULT_SERVO_OFFSET) / DEFAULT_SERVO_GAIN
            if abs(expected_angle) < 0.05:  # < ~3 degrees
                self.get_logger().info(
                    f"  servo={servo_pos:.3f}: near-center, skipping circle fit")
                continue
            
            # Check if we have enough trajectory variation
            dx = x[-1] - x[0]
            dy = y[-1] - y[0]
            path_length = np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2))
            
            if path_length < 0.3:  # Need at least 30cm of path
                self.get_logger().info(
                    f"  servo={servo_pos:.3f}: path too short ({path_length:.2f}m), skipping")
                continue
            
            # Fit circle
            cx, cy, radius, residual = fit_circle(x, y)
            
            if residual > 0.2 * radius:  # Poor fit
                self.get_logger().warn(
                    f"  servo={servo_pos:.3f}: poor circle fit "
                    f"(R={radius:.3f}m, residual={residual:.4f}m), skipping")
                continue
            
            # Compute actual steering angle from radius
            actual_angle = steering_angle_from_radius(radius, self.wheelbase)
            
            # Determine sign from the trajectory curvature
            # Cross product of consecutive segments gives sign
            if len(x) > 2:
                cross = np.mean(np.diff(x[:-1]) * np.diff(y[1:]) - 
                               np.diff(y[:-1]) * np.diff(x[1:]))
                if cross < 0:
                    actual_angle = -actual_angle
            
            servo_values.append(servo_pos)
            measured_angles.append(actual_angle)
            
            self.get_logger().info(
                f"  servo={servo_pos:.3f}: R={radius:.3f}m, "
                f"δ_measured={np.degrees(actual_angle):.2f}°, "
                f"δ_expected={np.degrees(expected_angle):.2f}°, "
                f"fit_residual={residual:.4f}m")
        
        if len(servo_values) < 3:
            self.get_logger().warn("Not enough valid measurements for calibration fit")
            return
        
        # Fit linear model: servo = gain * angle + offset
        servo_arr = np.array(servo_values)
        angle_arr = np.array(measured_angles)
        
        # Linear regression
        A = np.column_stack([angle_arr, np.ones_like(angle_arr)])
        result = np.linalg.lstsq(A, servo_arr, rcond=None)
        new_gain = result[0][0]
        new_offset = result[0][1]
        
        # Compute max steering angle with new calibration
        max_angle_left = (DEFAULT_SERVO_MIN - new_offset) / new_gain
        max_angle_right = (DEFAULT_SERVO_MAX - new_offset) / new_gain
        max_steer = max(abs(max_angle_left), abs(max_angle_right))
        
        self.get_logger().info("")
        self.get_logger().info("--- Calibration Results ---")
        self.get_logger().info(f"  Old gain:   {DEFAULT_SERVO_GAIN:.4f}")
        self.get_logger().info(f"  New gain:   {new_gain:.4f}")
        self.get_logger().info(f"  Old offset: {DEFAULT_SERVO_OFFSET:.4f}")
        self.get_logger().info(f"  New offset: {new_offset:.4f}")
        self.get_logger().info(f"  Max steering angle: {np.degrees(max_steer):.2f}° ({max_steer:.4f} rad)")
        self.get_logger().info("")
        self.get_logger().info("Update vesc.yaml with:")
        self.get_logger().info(f"  steering_angle_to_servo_gain: {new_gain:.4f}")
        self.get_logger().info(f"  steering_angle_to_servo_offset: {new_offset:.4f}")
        self.get_logger().info("")
        self.get_logger().info("Update mpc_types.h with:")
        self.get_logger().info(f"  #define MAX_STEER  FP_FROM_FLOAT({max_steer:.4f}f)")
        self.get_logger().info("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description='F1/10th Steering Calibration Test')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='Forward speed during test (m/s, default: 1.0)')
    parser.add_argument('--hold-time', type=float, default=3.0,
                        help='Hold time at each servo position (s, default: 3.0)')
    parser.add_argument('--steps', type=int, default=5,
                        help='Number of steps per side (default: 5)')
    parser.add_argument('--wheelbase', type=float, default=DEFAULT_WHEELBASE,
                        help=f'Known wheelbase in meters (default: {DEFAULT_WHEELBASE})')
    args = parser.parse_args()
    
    rclpy.init()
    node = SteeringCalibrationNode(args)
    
    try:
        node.run_test()
    finally:
        node.stop_car()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
