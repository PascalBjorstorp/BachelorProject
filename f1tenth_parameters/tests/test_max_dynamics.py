#!/usr/bin/env python3
"""
Maximum Dynamics Test for F1/10th Car

Measures maximum velocity, acceleration, and deceleration.

NOTE ON ODOMETRY LIMITATIONS:
    The VESC derives speed from motor ERPM (back-EMF).  During braking the
    wheels may lock or slip, causing ERPM to drop much faster than the actual
    vehicle speed.  This means:
      - Odom-reported speed drops to zero before the car physically stops
      - dv/dt from odom may OVERESTIMATE peak deceleration (abrupt drop)
      - Braking distance from odom will UNDERESTIMATE actual distance
    The IMU longitudinal accelerometer (imu_ax) measures the true body
    deceleration regardless of wheel state.  This script uses IMU-based
    measurements alongside odom for comparison.  The IMU values are more
    trustworthy during braking; the odom values are fine during steady
    acceleration (wheels not slipping).

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

import sys as _sys, os as _os  # noqa: E402
_sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..'))
from common import TestNode, ImuVelocityEstimator


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
        
        # IMU velocity estimator for braking phase
        self.imu_vel = ImuVelocityEstimator()
        
        # Results
        self.accel_data = {'time': [], 'speed': [], 'imu_ax': []}
        self.decel_data = {'time': [], 'speed': [], 'speed_imu': [], 'imu_ax': []}
    
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
        
        # ---- IMU Bias Calibration (while stationary during countdown) ----
        self.calibrate_imu_bias(duration=2.0)
        imu_bias = self.imu_vel.calibrate_bias([self.imu_bias_ax])
        cal_duration = 2.0
        self.get_logger().info(
            f"IMU ax bias: {imu_bias:.4f} m/s² "
            f"(from stationary calibration over {cal_duration}s)")
        
        self.countdown(3)
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
            rclpy.spin_once(self, timeout_sec=0.005)
            
            if not self.safety.check():
                self.get_logger().error(f"Safety abort: {self.safety.abort_reason}")
                self.stop_car()
                self.test_running = False
                break
            
            self.send_command(self.max_speed, 0.0)
            imu_ax_corr = self.imu_ax - imu_bias
            
            t = time.monotonic() - phase_start
            self.accel_data['time'].append(t)
            self.accel_data['speed'].append(self.odom_vx)
            self.accel_data['imu_ax'].append(imu_ax_corr)
            
            self.recorder.record(
                odom_x=self.odom_x,
                odom_y=self.odom_y,
                odom_vx=self.odom_vx,
                imu_ax=imu_ax_corr,
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
        self.get_logger().info("NOTE: Odom speed may drop to 0 before the car stops (wheel slip).")
        self.get_logger().info("      IMU-integrated velocity is used for accurate deceleration.")
        
        # Calibrate IMU bias from the stationary calibration at start
        # Reset with current speed as initial velocity
        self.imu_vel.reset(initial_velocity=abs(self.odom_vx))
        
        phase_start = time.monotonic()
        last_t = phase_start
        
        while (abs(self.odom_vx) > 0.1 or self.imu_vel.velocity > 0.1) and \
              time.monotonic() - phase_start < 10.0:
            rclpy.spin_once(self, timeout_sec=0.005)
            
            now = time.monotonic()
            dt = now - last_t
            last_t = now
            
            self.send_command(0.0, 0.0)
            imu_ax_corr = self.imu_ax - imu_bias
            
            # Update IMU-based velocity
            imu_v = self.imu_vel.update(self.imu_ax, dt)
            
            t = now - phase_start
            self.decel_data['time'].append(t)
            self.decel_data['speed'].append(self.odom_vx)
            self.decel_data['speed_imu'].append(imu_v)
            self.decel_data['imu_ax'].append(imu_ax_corr)
            
            self.recorder.record(
                odom_x=self.odom_x,
                odom_y=self.odom_y,
                odom_vx=self.odom_vx,
                imu_ax=imu_ax_corr,
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
            
            # ---- Check if velocity actually plateaued ----
            # Look at the last 20% of the acceleration phase
            n_tail = max(int(len(v) * 0.2), 5)
            v_tail = v[-n_tail:]
            t_tail = t[-n_tail:]
            reached_plateau = True
            plateau_note = ""
            
            if len(v_tail) > 2 and (t_tail[-1] - t_tail[0]) > 0.1:
                # Linear fit on last 20% of data
                slope = np.polyfit(t_tail, v_tail, 1)[0]
                # If speed is still rising at > 0.3 m/s², it hasn't plateaued
                if slope > 0.3:
                    reached_plateau = False
                    plateau_note = (
                        f"   ⚠ Speed was still increasing at {slope:.2f} m/s² "
                        f"when the test ended!\n"
                        f"     The car likely did NOT reach true max velocity.\n"
                        f"     Increase --accel-time (currently {self.accel_time:.1f}s) "
                        f"or --max-speed (currently {self.max_speed:.1f} m/s).\n"
                        f"     Also ensure enough straight runway (>= 5m at high speed)."
                    )
            
            # Also check if commanded speed was the bottleneck
            cmd_limited = False
            if max_velocity > 0 and max_velocity >= self.max_speed * 0.95:
                cmd_limited = True
            
            self.get_logger().info(f"\n1. ACCELERATION:")
            self.get_logger().info(f"   Commanded speed: {self.max_speed:.2f} m/s")
            self.get_logger().info(f"   Max velocity reached: {max_velocity:.2f} m/s")
            self.get_logger().info(f"   Max acceleration (from dv/dt): {max_accel_from_v:.2f} m/s²")
            self.get_logger().info(f"   Max acceleration (from IMU):   {max_accel_from_imu:.2f} m/s²")
            self.get_logger().info(f"   Time to 90% max speed: {time_to_90:.2f}s")
            
            if not reached_plateau:
                self.get_logger().warn(plateau_note)
            elif cmd_limited:
                self.get_logger().info(
                    f"   Reached commanded speed limit ({self.max_speed:.1f} m/s). "
                    f"Increase --max-speed to find true max velocity."
                )
            else:
                self.get_logger().info(
                    f"   Velocity plateaued — this is the true max at this command level."
                )
        
        # ---- Deceleration Analysis ----
        if self.decel_data['time']:
            t = np.array(self.decel_data['time'])
            v = np.array(self.decel_data['speed'])
            v_imu = np.array(self.decel_data['speed_imu'])
            ax = np.array(self.decel_data['imu_ax'])
            
            # Odom-based deceleration
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
            stop_time_odom = t[np.argmax(np.abs(v) < 0.1)] if np.any(np.abs(v) < 0.1) else t[-1]
            stop_time_imu = t[np.argmax(v_imu < 0.1)] if np.any(v_imu < 0.1) else t[-1]
            
            # IMU-based deceleration (from dv_imu/dt)
            if len(v_imu) > 5:
                kernel = 5
                vi_smooth = np.convolve(v_imu, np.ones(kernel)/kernel, mode='valid')
                ti_smooth = np.convolve(t, np.ones(kernel)/kernel, mode='valid')
                if len(vi_smooth) > 1:
                    dv_dt_imu = np.diff(vi_smooth) / np.diff(ti_smooth)
                    max_decel_imu_integrated = abs(np.min(dv_dt_imu))
                else:
                    max_decel_imu_integrated = 0.0
            else:
                max_decel_imu_integrated = 0.0
            
            self.get_logger().info(f"\n2. DECELERATION:")
            self.get_logger().info(f"   Max deceleration (odom dv/dt):    {max_decel_from_v:.2f} m/s²")
            self.get_logger().info(f"   Max deceleration (IMU ax):        {max_decel_from_imu:.2f} m/s²")
            self.get_logger().info(f"   Max deceleration (IMU integrated): {max_decel_imu_integrated:.2f} m/s²")
            self.get_logger().info(f"   Time to stop (odom):  {stop_time_odom:.2f}s")
            self.get_logger().info(f"   Time to stop (IMU):   {stop_time_imu:.2f}s")
            
            if abs(stop_time_odom - stop_time_imu) > 0.3:
                self.get_logger().warn(
                    f"   ⚠ Odom stops {abs(stop_time_odom - stop_time_imu):.2f}s before "
                    f"IMU — tire slip during braking. "
                    f"Trust the IMU-based values.")
        
        # ---- Summary for MPC ----
        self.get_logger().info(f"\n--- Parameters for MPC ---")
        if self.accel_data['speed']:
            max_v = np.max(self.accel_data['speed'])
            self.get_logger().info(f"  Max velocity: {max_v:.2f} m/s")
            self.get_logger().info(f"    #define MAX_VELOCITY  FP_FROM_FLOAT({max_v:.2f}f)")
        self.get_logger().info(f"\n  NOTE: Use IMU-based deceleration for MPC constraints.")
        self.get_logger().info(f"  The odom-based value is unreliable during braking due to wheel slip.")

        # Auto-save to vehicle_params.yaml
        params = {}
        if self.accel_data['speed']:
            params['max_velocity'] = float(np.max(self.accel_data['speed']))
            params['max_accel'] = float(np.max(np.abs(self.accel_data['imu_ax'])))
        if self.decel_data['imu_ax']:
            params['max_decel'] = float(np.max(np.abs(self.decel_data['imu_ax'])))
        if params:
            from common import update_vehicle_params
            update_vehicle_params(params, status='TESTED', logger=self.get_logger())
        self.get_logger().info("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description='F1/10th Maximum Dynamics Test')
    parser.add_argument('--max-speed', type=float, default=2.5,
                        help='Max speed to command (m/s, default: 2.5)')
    parser.add_argument('--accel-time', type=float, default=5.0,
                        help='Duration of acceleration phase (s, default: 5.0)')
    parser.add_argument('--runs', type=int, default=5,
                        help='Number of complete test runs (default: 5)')
    args = parser.parse_args()
    
    rclpy.init()
    for run_idx in range(args.runs):
        if args.runs > 1:
            print(f"\n{'='*60}")
            print(f"RUN {run_idx + 1}/{args.runs}")
            print(f"{'='*60}\n")
        node = MaxDynamicsNode(args)
        try:
            node.run_test()
        finally:
            node.stop_car()
            node.destroy_node()
        if run_idx < args.runs - 1:
            print("\nCooling down for 5s before next run...")
            time.sleep(5)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
