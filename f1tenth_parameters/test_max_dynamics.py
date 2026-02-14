#!/usr/bin/env python3
"""
Maximum Dynamics Test for F1/10th Car

Measures maximum velocity, acceleration, and deceleration.

CAUTION: This test drives at maximum speed! Ensure you have at least 5m of
clear, straight space and keep the joystick ready for override.

Procedure:
1. Full throttle from standstill → record v(t) → max acceleration and velocity
2. Full brake from max speed → record v(t) → max deceleration

Usage:
    python3 test_max_dynamics.py [--max-speed 3.0] [--accel-time 5.0]
"""

import argparse
import time

import numpy as np
import rclpy

from common import TestNode


class MaxDynamicsNode(TestNode):
    
    def __init__(self, args):
        columns = [
            'odom_x', 'odom_y', 'odom_vx',
            'imu_ax', 'imu_ay',
            'motor_rpm', 'cmd_speed', 'phase'
        ]
        
        super().__init__(
            'max_dynamics_test',
            'max_dynamics',
            columns,
            max_speed=args.max_speed * 1.3,
            max_time=args.accel_time * 3 + 30.0
        )
        
        self.max_speed = args.max_speed
        self.accel_time = args.accel_time
        
        # Results
        self.accel_data = {'time': [], 'speed': [], 'imu_ax': []}
        self.decel_data = {'time': [], 'speed': [], 'imu_ax': []}
    
    def run_test(self):
        """Execute the max dynamics test."""
        if not self.wait_for_sensors():
            return False
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("MAXIMUM DYNAMICS TEST")
        self.get_logger().info(f"Max speed: {self.max_speed:.1f} m/s")
        self.get_logger().info(f"Acceleration test duration: {self.accel_time:.1f}s")
        self.get_logger().info("")
        self.get_logger().info("WARNING: Car will drive at full speed!")
        self.get_logger().info("Ensure >= 5m of clear, straight space ahead.")
        self.get_logger().info("Keep joystick ready for emergency override.")
        self.get_logger().info("=" * 60)
        
        self.countdown(5)
        self.recorder.start()
        self.safety.start()
        self.test_running = True
        
        # ---- Phase 1: Full acceleration ----
        self.get_logger().info("\n--- Phase 1: ACCELERATION ---")
        self.get_logger().info(f"Commanding {self.max_speed:.1f} m/s...")
        self.get_logger().info("NOTE: ackermann_to_vesc has slow-start behavior when v<1 m/s.")
        self.get_logger().info("      Initial acceleration may be limited by the driver.")
        
        phase_start = time.monotonic()
        
        while time.monotonic() - phase_start < self.accel_time:
            rclpy.spin_once(self, timeout_sec=0.01)
            
            if not self.safety.check():
                self.get_logger().error(f"Safety abort: {self.safety.abort_reason}")
                self.stop_car()
                self.test_running = False
                break
            
            self.send_command(self.max_speed, 0.0)
            
            t = time.monotonic() - phase_start
            self.accel_data['time'].append(t)
            self.accel_data['speed'].append(self.odom_vx)
            self.accel_data['imu_ax'].append(self.imu_ax)
            
            self.recorder.record(
                odom_x=self.odom_x,
                odom_y=self.odom_y,
                odom_vx=self.odom_vx,
                imu_ax=self.imu_ax,
                imu_ay=self.imu_ay,
                motor_rpm=self.motor_rpm,
                cmd_speed=self.max_speed,
                phase='acceleration'
            )
        
        if not self.test_running:
            self.recorder.save()
            return False
        
        # Record peak speed
        peak_speed = max(self.accel_data['speed']) if self.accel_data['speed'] else 0
        self.get_logger().info(f"Peak speed reached: {peak_speed:.2f} m/s")
        
        # Brief cruise at peak
        self.spin_for(0.5)
        
        # ---- Phase 2: Full deceleration ----
        self.get_logger().info("\n--- Phase 2: DECELERATION ---")
        self.get_logger().info("Commanding 0 m/s (full brake)...")
        
        phase_start = time.monotonic()
        
        while abs(self.odom_vx) > 0.1 and time.monotonic() - phase_start < 10.0:
            rclpy.spin_once(self, timeout_sec=0.01)
            
            self.send_command(0.0, 0.0)
            
            t = time.monotonic() - phase_start
            self.decel_data['time'].append(t)
            self.decel_data['speed'].append(self.odom_vx)
            self.decel_data['imu_ax'].append(self.imu_ax)
            
            self.recorder.record(
                odom_x=self.odom_x,
                odom_y=self.odom_y,
                odom_vx=self.odom_vx,
                imu_ax=self.imu_ax,
                imu_ay=self.imu_ay,
                motor_rpm=self.motor_rpm,
                cmd_speed=0.0,
                phase='deceleration'
            )
        
        stop_time = time.monotonic() - phase_start
        self.get_logger().info(f"Stopped in {stop_time:.2f}s")
        
        self.stop_car()
        time.sleep(0.5)
        self.stop_car()
        
        self.recorder.save()
        self.analyze()
        return True
    
    def analyze(self):
        """Analyze max dynamics results."""
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("ANALYSIS RESULTS")
        self.get_logger().info("=" * 60)
        
        # ---- Acceleration Analysis ----
        if self.accel_data['time']:
            t = np.array(self.accel_data['time'])
            v = np.array(self.accel_data['speed'])
            ax = np.array(self.accel_data['imu_ax'])
            
            max_velocity = np.max(v)
            
            # Find max acceleration: dv/dt
            if len(v) > 5:
                # Smooth velocity with simple moving average
                kernel = 5
                v_smooth = np.convolve(v, np.ones(kernel)/kernel, mode='valid')
                t_smooth = np.convolve(t, np.ones(kernel)/kernel, mode='valid')
                
                if len(v_smooth) > 1:
                    dv_dt = np.diff(v_smooth) / np.diff(t_smooth)
                    max_accel_from_v = np.max(dv_dt)
                else:
                    max_accel_from_v = 0.0
            else:
                max_accel_from_v = (v[-1] - v[0]) / (t[-1] - t[0]) if len(v) > 1 else 0.0
            
            max_accel_from_imu = np.max(ax)
            
            # Time to reach 90% of max speed
            target_90 = max_velocity * 0.9
            idx_90 = np.argmax(v >= target_90) if np.any(v >= target_90) else len(v) - 1
            time_to_90 = t[idx_90]
            
            self.get_logger().info(f"\n1. ACCELERATION:")
            self.get_logger().info(f"   Max velocity: {max_velocity:.2f} m/s")
            self.get_logger().info(f"   Max acceleration (from dv/dt): {max_accel_from_v:.2f} m/s²")
            self.get_logger().info(f"   Max acceleration (from IMU): {max_accel_from_imu:.2f} m/s²")
            self.get_logger().info(f"   Time to 90% max speed: {time_to_90:.2f}s")
        
        # ---- Deceleration Analysis ----
        if self.decel_data['time']:
            t = np.array(self.decel_data['time'])
            v = np.array(self.decel_data['speed'])
            ax = np.array(self.decel_data['imu_ax'])
            
            if len(v) > 5:
                kernel = 5
                v_smooth = np.convolve(v, np.ones(kernel)/kernel, mode='valid')
                t_smooth = np.convolve(t, np.ones(kernel)/kernel, mode='valid')
                
                if len(v_smooth) > 1:
                    dv_dt = np.diff(v_smooth) / np.diff(t_smooth)
                    max_decel_from_v = abs(np.min(dv_dt))
                else:
                    max_decel_from_v = 0.0
            else:
                max_decel_from_v = abs((v[-1] - v[0]) / (t[-1] - t[0])) if len(v) > 1 else 0.0
            
            max_decel_from_imu = abs(np.min(ax))
            stop_time = t[-1]
            
            self.get_logger().info(f"\n2. DECELERATION:")
            self.get_logger().info(f"   Max deceleration (from dv/dt): {max_decel_from_v:.2f} m/s²")
            self.get_logger().info(f"   Max deceleration (from IMU): {max_decel_from_imu:.2f} m/s²")
            self.get_logger().info(f"   Time to stop: {stop_time:.2f}s")
        
        # ---- Summary for MPC ----
        self.get_logger().info(f"\n--- Parameters for MPC ---")
        if self.accel_data['speed']:
            max_v = np.max(self.accel_data['speed'])
            self.get_logger().info(f"  Max velocity: {max_v:.2f} m/s")
            self.get_logger().info(f"    #define MAX_VELOCITY  FP_FROM_FLOAT({max_v:.2f}f)")
        self.get_logger().info("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description='F1/10th Maximum Dynamics Test')
    parser.add_argument('--max-speed', type=float, default=3.0,
                        help='Max speed to command (m/s, default: 3.0)')
    parser.add_argument('--accel-time', type=float, default=5.0,
                        help='Duration of acceleration phase (s, default: 5.0)')
    args = parser.parse_args()
    
    rclpy.init()
    node = MaxDynamicsNode(args)
    
    try:
        node.run_test()
    finally:
        node.stop_car()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
