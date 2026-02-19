#!/usr/bin/env python3
"""
Circle Test for F1/10th Car

Drives circles at fixed steering angle and constant speed to extract:
- Effective wheelbase (from turning radius at low speed)
- Validation of steering calibration
- Understeer gradient (from radius change with speed)
- Cornering stiffness ratio C_f/C_r (from understeer gradient)

At low speeds, the kinematic model applies:
    R_kinematic = L / tan(δ)

At higher speeds, the actual radius increases due to tire slip:
    R_actual = R_kinematic + K_us * v²
    where K_us = understeer gradient = m/(2*L) * (l_r/C_f - l_f/C_r)

Procedure:
1. Drive circles at low speed (1-2 m/s) → extract R_kinematic → verify wheelbase
2. Drive circles at increasing speeds → measure R_actual(v)
3. Fit R_actual vs v² → extract understeer gradient
4. From understeer gradient + known mass/geometry → cornering stiffness ratio

Usage:
    python3 test_circle.py [--steering 0.3] [--speeds 1.5,2.0,3.0] [--laps 2]
"""

import argparse
import time

import numpy as np
import rclpy

from common import (
    TestNode, fit_circle, steering_angle_from_radius,
    radius_from_steering_angle, DEFAULT_WHEELBASE
)


class CircleTestNode(TestNode):
    
    def __init__(self, args):
        columns = [
            'odom_x', 'odom_y', 'odom_yaw',
            'odom_vx', 'odom_omega',
            'imu_ay', 'imu_gz',
            'cmd_speed', 'cmd_steering',
            'speed_setpoint', 'phase'
        ]
        
        max_speed = max(args.speeds) * 1.3
        max_time = len(args.speeds) * args.laps * 20.0 + 60.0  # rough estimate
        
        super().__init__(
            'circle_test',
            'circle_test',
            columns,
            max_speed=max_speed,
            max_time=max_time
        )
        
        self.steering_angle = args.steering
        self.speeds = args.speeds
        self.num_laps = args.laps
        self.wheelbase = args.wheelbase
        self.direction = args.direction
        
        # Results: list of (speed, radius, residual, avg_ay, avg_omega)
        self.circle_results = []
    
    def run_circle_at_speed(self, target_speed: float):
        """Drive circles at a given speed and measure the radius."""
        steer = self.steering_angle if self.direction == 'left' else -self.steering_angle
        
        # Predict kinematic radius for lap estimation
        predicted_radius = radius_from_steering_angle(self.steering_angle, self.wheelbase)
        circumference = 2.0 * np.pi * predicted_radius
        total_distance = circumference * self.num_laps
        estimated_time = total_distance / target_speed
        
        self.get_logger().info(
            f"  Speed: {target_speed:.1f} m/s, "
            f"Predicted radius: {predicted_radius:.2f}m, "
            f"Estimated time: {estimated_time:.1f}s")
        
        # Accelerate to target speed with straight steering first
        self.send_command(target_speed, 0.0)
        self.spin_for(1.0)
        
        # Start turning
        self.send_command(target_speed, steer)
        
        # Allow 2 seconds for the car to settle into the circle
        self.spin_for(2.0)
        
        # Record data for the required number of laps
        segment_x = []
        segment_y = []
        ay_samples = []
        omega_samples = []
        speed_samples = []
        
        record_time = estimated_time + 5.0  # extra margin
        start = time.monotonic()
        
        while time.monotonic() - start < record_time:
            rclpy.spin_once(self, timeout_sec=0.02)
            
            if not self.safety.check():
                self.get_logger().error(f"Safety abort: {self.safety.abort_reason}")
                self.stop_car()
                return None
            
            self.send_command(target_speed, steer)
            
            segment_x.append(self.odom_x)
            segment_y.append(self.odom_y)
            ay_samples.append(self.imu_ay)
            omega_samples.append(self.imu_gz)
            speed_samples.append(self.odom_vx)
            
            self.recorder.record(
                odom_x=self.odom_x,
                odom_y=self.odom_y,
                odom_yaw=self.odom_yaw,
                odom_vx=self.odom_vx,
                odom_omega=self.odom_omega,
                imu_ay=self.imu_ay,
                imu_gz=self.imu_gz,
                cmd_speed=target_speed,
                cmd_steering=steer,
                speed_setpoint=target_speed,
                phase=f'circle_v{target_speed:.1f}'
            )
        
        if len(segment_x) < 20:
            self.get_logger().warn(f"  Not enough data points ({len(segment_x)})")
            return None
        
        # Fit circle
        x = np.array(segment_x)
        y = np.array(segment_y)
        cx, cy, radius, residual = fit_circle(x, y)
        
        avg_ay = np.mean(np.abs(ay_samples))
        avg_omega = np.mean(np.abs(omega_samples))
        avg_speed = np.mean(speed_samples)
        
        self.get_logger().info(
            f"  Result: R={radius:.3f}m (residual={residual:.4f}m), "
            f"avg_speed={avg_speed:.2f} m/s, "
            f"avg_ay={avg_ay:.3f} m/s², "
            f"avg_omega={np.degrees(avg_omega):.2f} °/s")
        
        return {
            'speed_cmd': target_speed,
            'speed_actual': avg_speed,
            'radius': radius,
            'residual': residual,
            'avg_ay': avg_ay,
            'avg_omega': avg_omega,
        }
    
    def run_test(self):
        """Execute the circle test at multiple speeds."""
        if not self.wait_for_sensors():
            return False
        
        predicted_r = radius_from_steering_angle(self.steering_angle, self.wheelbase)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("CIRCLE TEST")
        self.get_logger().info(f"Steering angle: {np.degrees(self.steering_angle):.1f}°")
        self.get_logger().info(f"Direction: {self.direction}")
        self.get_logger().info(f"Speeds: {[f'{s:.1f}' for s in self.speeds]} m/s")
        self.get_logger().info(f"Laps per speed: {self.num_laps}")
        self.get_logger().info(f"Predicted kinematic radius: {predicted_r:.2f}m")
        self.get_logger().info(f"Required space: ~{2*predicted_r + 1:.1f}m x {2*predicted_r + 1:.1f}m")
        self.get_logger().info("=" * 60)
        
        self.countdown(5)
        self.recorder.start()
        self.safety.start()
        self.test_running = True
        
        for speed in self.speeds:
            if not self.test_running:
                break
            
            self.get_logger().info(f"\n--- Circle at {speed:.1f} m/s ---")
            result = self.run_circle_at_speed(speed)
            
            if result is not None:
                self.circle_results.append(result)
            
            # Brief pause between speeds
            self.stop_car()
            self.spin_for(2.0)
        
        self.stop_car()
        time.sleep(0.5)
        self.stop_car()
        
        self.recorder.save()
        self.analyze()
        return True
    
    def analyze(self):
        """Analyze circle test results."""
        if not self.circle_results:
            self.get_logger().warn("No valid circle data to analyze")
            return
        
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("ANALYSIS RESULTS")
        self.get_logger().info("=" * 60)
        
        # ---- 1. Wheelbase from lowest-speed circle ----
        lowest = min(self.circle_results, key=lambda r: r['speed_actual'])
        
        # At low speed, R ≈ L / tan(δ), so L ≈ R * tan(δ)
        measured_wheelbase = lowest['radius'] * np.tan(self.steering_angle)
        measured_angle = steering_angle_from_radius(
            lowest['radius'], self.wheelbase)
        
        self.get_logger().info(f"\n1. Wheelbase Estimate (from v={lowest['speed_actual']:.2f} m/s):")
        self.get_logger().info(f"   Measured radius: {lowest['radius']:.3f}m")
        self.get_logger().info(f"   Commanded steering: {np.degrees(self.steering_angle):.2f}°")
        self.get_logger().info(f"   Current wheelbase: {self.wheelbase:.4f}m")
        self.get_logger().info(f"   Measured wheelbase: {measured_wheelbase:.4f}m")
        self.get_logger().info(f"   (Or: current L implies actual δ = {np.degrees(measured_angle):.2f}°)")
        
        # ---- 2. Understeer gradient (if multiple speeds) ----
        if len(self.circle_results) >= 3:
            speeds = np.array([r['speed_actual'] for r in self.circle_results])
            radii = np.array([r['radius'] for r in self.circle_results])
            
            # R_actual = R_kinematic + K_us * v²
            # Fit: R = a + b * v²
            v_sq = speeds**2
            A = np.column_stack([np.ones_like(v_sq), v_sq])
            result = np.linalg.lstsq(A, radii, rcond=None)
            R_kinematic_fit = result[0][0]
            K_us = result[0][1]
            
            self.get_logger().info(f"\n2. Understeer Gradient (from {len(self.circle_results)} speeds):")
            self.get_logger().info(f"   R_kinematic (fit): {R_kinematic_fit:.3f}m")
            self.get_logger().info(f"   Understeer gradient K_us: {K_us:.6f} m/(m/s)²")
            
            if K_us > 0:
                self.get_logger().info(f"   Car is UNDERSTEERING (typical for 4WD)")
            elif K_us < 0:
                self.get_logger().info(f"   Car is OVERSTEERING (rare, be careful at high speed!)")
            else:
                self.get_logger().info(f"   Car is NEUTRAL (unlikely)")
            
            # Cornering stiffness info
            # K_us = m/(2L) * (l_r/C_f - l_f/C_r)
            # We can't separate C_f and C_r from K_us alone, but we can give the ratio
            # if we assume l_f and l_r (50/50 split: l_f = l_r = L/2)
            self.get_logger().info(f"\n   Assuming 50/50 weight distribution (l_f = l_r = L/2):")
            self.get_logger().info(f"   K_us = m/(2L) * (L/2) * (1/C_f - 1/C_r)")
            self.get_logger().info(f"   Note: Need data at higher speeds for reliable cornering stiffness")
        else:
            self.get_logger().info(f"\n2. Need >= 3 speed points for understeer gradient analysis")
        
        # ---- 3. Friction estimate from lateral acceleration ----
        fastest = max(self.circle_results, key=lambda r: r['speed_actual'])
        self.get_logger().info(f"\n3. Lateral Acceleration at Max Speed:")
        self.get_logger().info(
            f"   v={fastest['speed_actual']:.2f} m/s: "
            f"a_y={fastest['avg_ay']:.3f} m/s² "
            f"(≈ {fastest['avg_ay']/9.81:.3f} g)")
        self.get_logger().info(f"   v²/R = {fastest['speed_actual']**2/fastest['radius']:.3f} m/s² (kinematic)")
        
        # ---- Summary for MPC ----
        self.get_logger().info(f"\n--- Parameters for MPC ---")
        self.get_logger().info(f"  Wheelbase: {measured_wheelbase:.4f}m")
        self.get_logger().info(f"  Max steer: {np.degrees(self.steering_angle):.2f}° (commanded)")
        self.get_logger().info(f"  Update mpc_types.h:")
        self.get_logger().info(f"    #define WHEELBASE  FP_FROM_FLOAT({measured_wheelbase:.4f}f)")
        self.get_logger().info("=" * 60)


def main():
    parser = argparse.ArgumentParser(description='F1/10th Circle Test')
    parser.add_argument('--steering', type=float, default=0.3,
                        help='Steering angle in radians (default: 0.3 ≈ 17°)')
    parser.add_argument('--speeds', type=str, default='1.0,1.5,2.0,2.5,3.0',
                        help='Comma-separated speeds in m/s (default: 1.0,1.5,2.0,2.5,3.0)')
    parser.add_argument('--laps', type=int, default=2,
                        help='Number of laps per speed point (default: 2)')
    parser.add_argument('--wheelbase', type=float, default=DEFAULT_WHEELBASE,
                        help=f'Expected wheelbase in meters (default: {DEFAULT_WHEELBASE})')
    parser.add_argument('--direction', choices=['left', 'right'], default='left',
                        help='Circle direction (default: left)')
    args = parser.parse_args()
    
    # Parse speeds
    args.speeds = [float(s) for s in args.speeds.split(',')]
    
    rclpy.init()
    node = CircleTestNode(args)
    
    try:
        node.run_test()
    finally:
        node.stop_car()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
