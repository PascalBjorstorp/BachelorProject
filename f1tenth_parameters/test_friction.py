#!/usr/bin/env python3
"""
Friction Limit Test for F1/10th Car

Measures the maximum tire grip (friction coefficient μ) by driving circles
at increasing speed until the tires begin to saturate.

The friction coefficient is:
    μ = a_y_max / g

where a_y_max is the maximum sustained lateral acceleration before the car
begins to slide (understeer or oversteer).

Detection methods:
1. IMU lateral acceleration reaches a plateau
2. Actual yaw rate diverges from kinematic prediction
3. Odom-reported radius increases significantly from kinematic radius

CAUTION: This test intentionally approaches the limits of grip!
Start at low speed and increase gradually. Keep the joystick ready.

Usage:
    python3 test_friction.py [--steering 0.3] [--max-speed 4.0] [--speed-step 0.5]
"""

import argparse
import time

import numpy as np
import rclpy

from common import (
    TestNode, fit_circle, radius_from_steering_angle, DEFAULT_WHEELBASE
)

GRAVITY = 9.81  # m/s²


class FrictionTestNode(TestNode):
    
    def __init__(self, args):
        columns = [
            'odom_x', 'odom_y', 'odom_yaw',
            'odom_vx', 'odom_omega',
            'imu_ax', 'imu_ay', 'imu_gz',
            'cmd_speed', 'cmd_steering',
            'speed_setpoint', 'phase'
        ]
        
        num_speeds = int((args.max_speed - args.min_speed) / args.speed_step) + 1
        max_time = num_speeds * 15.0 + 60.0
        
        super().__init__(
            'friction_test',
            'friction_test',
            columns,
            max_speed=args.max_speed * 1.3,
            max_time=max_time
        )
        
        self.steering_angle = args.steering
        self.min_speed = args.min_speed
        self.max_speed_cmd = args.max_speed
        self.speed_step = args.speed_step
        self.wheelbase = args.wheelbase
        self.circle_time = args.circle_time
        
        self.speed_results = []
    
    def run_at_speed(self, target_speed: float):
        """Drive circle at target speed, record lateral acceleration."""
        steer = self.steering_angle  # always turn left for consistency
        
        self.get_logger().info(f"  Speed: {target_speed:.1f} m/s")
        
        # Drive at speed with steering
        self.send_command(target_speed, steer)
        
        # Allow settle time
        self.spin_for(2.0)
        
        # Record data
        ay_samples = []
        ax_samples = []
        omega_samples = []
        speed_samples = []
        x_samples = []
        y_samples = []
        
        start = time.monotonic()
        while time.monotonic() - start < self.circle_time:
            rclpy.spin_once(self, timeout_sec=0.02)
            
            if not self.safety.check():
                self.get_logger().error(f"Safety abort: {self.safety.abort_reason}")
                self.stop_car()
                return None
            
            self.send_command(target_speed, steer)
            
            ay_samples.append(self.imu_ay)
            ax_samples.append(self.imu_ax)
            omega_samples.append(self.imu_gz)
            speed_samples.append(self.odom_vx)
            x_samples.append(self.odom_x)
            y_samples.append(self.odom_y)
            
            self.recorder.record(
                odom_x=self.odom_x,
                odom_y=self.odom_y,
                odom_yaw=self.odom_yaw,
                odom_vx=self.odom_vx,
                odom_omega=self.odom_omega,
                imu_ax=self.imu_ax,
                imu_ay=self.imu_ay,
                imu_gz=self.imu_gz,
                cmd_speed=target_speed,
                cmd_steering=steer,
                speed_setpoint=target_speed,
                phase=f'friction_v{target_speed:.1f}'
            )
        
        if len(ay_samples) < 10:
            return None
        
        avg_ay = np.mean(np.abs(ay_samples))
        avg_speed = np.mean(speed_samples)
        avg_omega = np.mean(np.abs(omega_samples))
        
        # Kinematic prediction
        r_kinematic = radius_from_steering_angle(self.steering_angle, self.wheelbase)
        omega_kinematic = avg_speed / r_kinematic
        ay_kinematic = avg_speed**2 / r_kinematic
        
        # Fit actual radius
        x = np.array(x_samples)
        y = np.array(y_samples)
        if len(x) > 10:
            _, _, r_actual, _ = fit_circle(x, y)
        else:
            r_actual = r_kinematic
        
        # Slip indicator: ratio of actual to kinematic yaw rate
        slip_ratio = avg_omega / omega_kinematic if omega_kinematic > 0.01 else 1.0
        
        result = {
            'speed_cmd': target_speed,
            'speed_actual': avg_speed,
            'ay_imu': avg_ay,
            'ay_kinematic': ay_kinematic,
            'omega_actual': avg_omega,
            'omega_kinematic': omega_kinematic,
            'r_actual': r_actual,
            'r_kinematic': r_kinematic,
            'slip_ratio': slip_ratio,
            'mu_estimate': avg_ay / GRAVITY,
        }
        
        self.get_logger().info(
            f"    v={avg_speed:.2f} m/s, "
            f"a_y={avg_ay:.3f} m/s² ({avg_ay/GRAVITY:.3f}g), "
            f"ω={np.degrees(avg_omega):.1f}°/s "
            f"(kinematic: {np.degrees(omega_kinematic):.1f}°/s), "
            f"slip_ratio={slip_ratio:.3f}")
        
        return result
    
    def run_test(self):
        """Execute friction limit test with increasing speed."""
        if not self.wait_for_sensors():
            return False
        
        r_kinematic = radius_from_steering_angle(self.steering_angle, self.wheelbase)
        speeds = np.arange(self.min_speed, self.max_speed_cmd + 0.01, self.speed_step)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("FRICTION LIMIT TEST")
        self.get_logger().info(f"Steering angle: {np.degrees(self.steering_angle):.1f}°")
        self.get_logger().info(f"Speed range: {self.min_speed:.1f} to {self.max_speed_cmd:.1f} m/s "
                              f"(step {self.speed_step:.1f})")
        self.get_logger().info(f"Kinematic radius: {r_kinematic:.2f}m")
        self.get_logger().info(f"Required space: ~{2*r_kinematic + 2:.1f}m x {2*r_kinematic + 2:.1f}m")
        self.get_logger().info("")
        self.get_logger().info("CAUTION: Car will approach grip limits!")
        self.get_logger().info("Keep joystick ready. Test will auto-abort if")
        self.get_logger().info("slip ratio drops significantly (tire saturation).")
        self.get_logger().info("=" * 60)
        
        self.countdown(5)
        self.recorder.start()
        self.safety.start()
        self.test_running = True
        
        for speed in speeds:
            if not self.test_running:
                break
            
            result = self.run_at_speed(speed)
            
            if result is not None:
                self.speed_results.append(result)
                
                # Auto-abort if significant slip detected
                if result['slip_ratio'] < 0.7:
                    self.get_logger().warn(
                        f"  Significant tire slip detected (ratio={result['slip_ratio']:.2f}). "
                        f"Stopping test for safety.")
                    break
        
        self.stop_car()
        time.sleep(0.5)
        self.stop_car()
        
        self.recorder.save()
        self.analyze()
        return True
    
    def analyze(self):
        """Analyze friction test results."""
        if not self.speed_results:
            self.get_logger().warn("No valid data to analyze")
            return
        
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("ANALYSIS RESULTS")
        self.get_logger().info("=" * 60)
        
        speeds = np.array([r['speed_actual'] for r in self.speed_results])
        ay_values = np.array([r['ay_imu'] for r in self.speed_results])
        slip_ratios = np.array([r['slip_ratio'] for r in self.speed_results])
        
        # Maximum lateral acceleration
        max_ay = np.max(ay_values)
        mu = max_ay / GRAVITY
        
        self.get_logger().info(f"\n1. FRICTION COEFFICIENT:")
        self.get_logger().info(f"   Max lateral acceleration: {max_ay:.3f} m/s² ({mu:.3f} g)")
        self.get_logger().info(f"   Friction coefficient μ ≈ {mu:.3f}")
        self.get_logger().info(f"   At speed: {speeds[np.argmax(ay_values)]:.2f} m/s")
        
        # Check if we reached saturation
        if np.min(slip_ratios) < 0.85:
            self.get_logger().info(f"   Tire saturation detected (min slip ratio: {np.min(slip_ratios):.3f})")
        else:
            self.get_logger().info(f"   Tires did NOT fully saturate at tested speeds.")
            self.get_logger().info(f"   Consider testing at higher speeds for true μ.")
        
        # Print table
        self.get_logger().info(f"\n2. SPEED vs LATERAL ACCELERATION:")
        self.get_logger().info(f"   {'v (m/s)':>8} {'a_y (m/s²)':>12} {'a_y (g)':>10} {'slip_ratio':>12}")
        for r in self.speed_results:
            self.get_logger().info(
                f"   {r['speed_actual']:8.2f} {r['ay_imu']:12.3f} {r['ay_imu']/GRAVITY:10.3f} {r['slip_ratio']:12.3f}")
        
        # ---- Summary for MPC ----
        self.get_logger().info(f"\n--- Parameters for MPC ---")
        self.get_logger().info(f"  Friction coefficient μ: {mu:.3f}")
        self.get_logger().info(f"  Max safe lateral accel: {max_ay * 0.8:.2f} m/s² (80% margin)")
        self.get_logger().info(f"  This limits max cornering speed at radius R:")
        self.get_logger().info(f"    v_max = sqrt(μ * g * R)")
        
        r_kin = radius_from_steering_angle(self.steering_angle, self.wheelbase)
        v_max_corner = np.sqrt(mu * GRAVITY * r_kin)
        self.get_logger().info(f"  At δ={np.degrees(self.steering_angle):.0f}° (R={r_kin:.2f}m): v_max = {v_max_corner:.2f} m/s")
        self.get_logger().info("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description='F1/10th Friction Limit Test')
    parser.add_argument('--steering', type=float, default=0.3,
                        help='Steering angle in radians (default: 0.3 ≈ 17°)')
    parser.add_argument('--min-speed', type=float, default=1.0,
                        help='Starting speed (m/s, default: 1.0)')
    parser.add_argument('--max-speed', type=float, default=4.0,
                        help='Maximum speed to test (m/s, default: 4.0)')
    parser.add_argument('--speed-step', type=float, default=0.5,
                        help='Speed increment (m/s, default: 0.5)')
    parser.add_argument('--circle-time', type=float, default=8.0,
                        help='Recording time per speed point (s, default: 8.0)')
    parser.add_argument('--wheelbase', type=float, default=DEFAULT_WHEELBASE,
                        help=f'Wheelbase in meters (default: {DEFAULT_WHEELBASE})')
    args = parser.parse_args()
    
    rclpy.init()
    node = FrictionTestNode(args)
    
    try:
        node.run_test()
    finally:
        node.stop_car()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
