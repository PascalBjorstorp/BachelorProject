#!/usr/bin/env python3
"""
Steering Rate Test for F1/10th Car

Measures the effective steering actuator speed (servo rate limit) by
commanding step changes in steering angle and observing the yaw rate response.

The throttle interpolator in the f1tenth stack limits servo rate to
max_servo_speed (default: 3.2 rad/s for the servo position value, NOT the
steering angle). The actual steering angle rate depends on the servo gain:
    steering_rate = max_servo_speed / abs(steering_angle_to_servo_gain)

This test directly measures the effective steering angle rate by:
1. Driving straight at constant speed
2. Commanding a step change from 0 to max steering
3. Recording the yaw rate response from IMU
4. Measuring the time for yaw rate to reach steady state
5. Computing effective steering rate from the response

Usage:
    python3 test_steering_rate.py [--speed 1.5] [--steering 0.3] [--repeats 5]
"""

import argparse
import time

import numpy as np
import rclpy

from common import TestNode, DEFAULT_SERVO_GAIN


class SteeringRateNode(TestNode):
    
    def __init__(self, args):
        columns = [
            'odom_vx', 'odom_omega',
            'imu_gz', 'cmd_steering',
            'phase'
        ]
        
        super().__init__(
            'steering_rate_test',
            'steering_rate',
            columns,
            max_speed=args.speed * 1.5,
            max_time=args.repeats * 15.0 + 30.0
        )
        
        self.test_speed = args.speed
        self.steering_angle = args.steering
        self.num_repeats = args.repeats
        
        self.step_results = []
    
    def run_step(self, step_idx: int, direction: float):
        """Run a single steering step test."""
        steer_target = direction * self.steering_angle
        dir_name = "LEFT" if direction > 0 else "RIGHT"
        
        self.get_logger().info(f"  Step {step_idx + 1}: {dir_name} to {np.degrees(steer_target):.1f}°")
        
        # Drive straight for 1 second to stabilize
        self.send_command(self.test_speed, 0.0)
        self.spin_for(1.5)
        
        # Record the step response
        times = []
        yaw_rates = []
        
        # Command step change
        step_time = time.monotonic()
        self.send_command(self.test_speed, steer_target)
        
        # Record for 3 seconds (should be well past settling)
        # Use self.imu_gz (from /sensors/imu/raw) NOT self.odom_omega
        # because odom angular velocity is low-pass filtered (EMA alpha=0.3)
        while time.monotonic() - step_time < 3.0:
            rclpy.spin_once(self, timeout_sec=0.005)
            
            if not self.safety.check():
                self.get_logger().error(f"Safety abort: {self.safety.abort_reason}")
                self.stop_car()
                return None
            
            self.send_command(self.test_speed, steer_target)
            
            t = time.monotonic() - step_time
            times.append(t)
            yaw_rates.append(self.imu_gz)
            
            self.recorder.record(
                odom_vx=self.odom_vx,
                odom_omega=self.odom_omega,
                imu_gz=self.imu_gz,
                cmd_steering=steer_target,
                phase=f'step_{step_idx}_{dir_name}'
            )
        
        if len(times) < 10:
            return None
        
        t = np.array(times)
        omega = np.array(yaw_rates)
        
        # Find steady-state yaw rate (last 1 second average)
        mask_ss = t > 2.0
        if np.sum(mask_ss) < 5:
            mask_ss = t > t[-1] * 0.7
        
        omega_ss = np.mean(omega[mask_ss])
        
        # Find rise time: time from 10% to 90% of steady-state
        omega_10 = 0.1 * omega_ss
        omega_90 = 0.9 * omega_ss
        
        if abs(omega_ss) > 0.01:
            # Find first crossing of 10% and 90%
            if omega_ss > 0:
                idx_10 = np.argmax(omega >= omega_10) if np.any(omega >= omega_10) else 0
                idx_90 = np.argmax(omega >= omega_90) if np.any(omega >= omega_90) else len(t) - 1
            else:
                idx_10 = np.argmax(omega <= omega_10) if np.any(omega <= omega_10) else 0
                idx_90 = np.argmax(omega <= omega_90) if np.any(omega <= omega_90) else len(t) - 1
            
            rise_time = t[idx_90] - t[idx_10] if idx_90 > idx_10 else 0.0
        else:
            rise_time = 0.0
        
        # Effective steering rate = Δδ / rise_time
        if rise_time > 0.001:
            effective_rate = abs(steer_target) * 0.8 / rise_time  # 0.8 = (90%-10%)
        else:
            effective_rate = float('inf')
        
        # Return to straight briefly
        self.send_command(self.test_speed, 0.0)
        self.spin_for(0.5)
        
        return {
            'direction': dir_name,
            'target_angle': steer_target,
            'omega_ss': omega_ss,
            'rise_time': rise_time,
            'effective_rate': effective_rate,
        }
    
    def run_test(self):
        """Execute the steering rate test."""
        if not self.wait_for_sensors():
            return False
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("STEERING RATE TEST")
        self.get_logger().info(f"Speed: {self.test_speed:.1f} m/s")
        self.get_logger().info(f"Step steering angle: {np.degrees(self.steering_angle):.1f}°")
        self.get_logger().info(f"Number of repeats: {self.num_repeats}")
        self.get_logger().info("Ensure ~3m of open space.")
        self.get_logger().info("=" * 60)
        
        self.countdown(5)
        self.recorder.start()
        self.safety.start()
        self.test_running = True
        
        for i in range(self.num_repeats):
            if not self.test_running:
                break
            
            # Alternate directions
            direction = 1.0 if i % 2 == 0 else -1.0
            result = self.run_step(i, direction)
            
            if result is not None:
                self.step_results.append(result)
        
        self.stop_car()
        time.sleep(0.5)
        self.stop_car()
        
        self.recorder.save()
        self.analyze()
        return True
    
    def analyze(self):
        """Analyze steering rate results."""
        if not self.step_results:
            self.get_logger().warn("No valid step responses to analyze")
            return
        
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("ANALYSIS RESULTS")
        self.get_logger().info("=" * 60)
        
        rise_times = []
        effective_rates = []
        
        for r in self.step_results:
            self.get_logger().info(
                f"  {r['direction']}: δ={np.degrees(r['target_angle']):.1f}°, "
                f"ω_ss={np.degrees(r['omega_ss']):.1f}°/s, "
                f"rise_time={r['rise_time']*1000:.1f}ms, "
                f"effective_rate={np.degrees(r['effective_rate']):.1f}°/s")
            
            if r['rise_time'] > 0.001:
                rise_times.append(r['rise_time'])
                effective_rates.append(r['effective_rate'])
        
        if rise_times:
            avg_rise = np.mean(rise_times)
            avg_rate = np.mean(effective_rates)
            
            self.get_logger().info(f"\n  Average rise time (10-90%): {avg_rise*1000:.1f}ms")
            self.get_logger().info(f"  Average effective steering rate: {np.degrees(avg_rate):.1f}°/s ({avg_rate:.2f} rad/s)")
            self.get_logger().info(f"  Note: This is the physical servo rate limit (no throttle_interpolator active).")
            
            self.get_logger().info(f"\n--- Parameters for MPC ---")
            self.get_logger().info(f"  Max steering rate: {avg_rate:.2f} rad/s")
            self.get_logger().info(f"  (Current MPC does not limit steering rate directly,")
            self.get_logger().info(f"   but this affects achievable rate penalty weight)")
        
        self.get_logger().info("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description='F1/10th Steering Rate Test')
    parser.add_argument('--speed', type=float, default=1.5,
                        help='Forward speed during test (m/s, default: 1.5)')
    parser.add_argument('--steering', type=float, default=0.3,
                        help='Step steering angle in radians (default: 0.3 ≈ 17°)')
    parser.add_argument('--repeats', type=int, default=6,
                        help='Number of step repeats (default: 6, alternating L/R)')
    args = parser.parse_args()
    
    rclpy.init()
    node = SteeringRateNode(args)
    
    try:
        node.run_test()
    finally:
        node.stop_car()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
