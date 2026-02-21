#!/usr/bin/env python3
"""
Wheelbase Verification Test for F1/10th Car

Drives circles at fixed steering angle and constant speed to extract the
effective wheelbase from the turning radius at low speed.

At low speeds the kinematic (Ackermann) model applies well:
    R_kinematic = L / tan(δ)
So measuring R and knowing δ gives L.

At higher speeds tire slip makes the odom-based radius unreliable.
The ERPM-based odometry does not account for lateral tire slip or wheel
lock-up, so the trajectory integrated from odom drifts.  As a cross-check
the script also computes a slip-independent radius from the IMU yaw rate:
    R_imu = v / |ω_imu|
Large discrepancies between R_odom and R_imu indicate significant tire slip.

NOTE ON ODOMETRY LIMITATIONS:
    The VESC derives speed from motor back-EMF (ERPM). During braking the
    wheels may lock / slip → ERPM drops faster than actual vehicle speed.
    During high-speed cornering the driven wheels may spin faster than
    ground speed. Both effects corrupt the odom-based position and speed.
    The wheelbase estimate is most reliable at LOW speed (≤ 2 m/s) where
    tire slip is minimal.

Procedure:
1. Drive circles at low speed (1-2 m/s) → extract R_kinematic → verify wheelbase
2. Optionally drive at higher speeds → compare R_odom vs R_imu for slip detection

Usage:
    python3 test_wheelbase.py [--steering 0.3] [--speeds 1.0,1.5,2.0] [--laps 2]
"""

import argparse
import time

import numpy as np
import rclpy

import sys as _sys, os as _os  # noqa: E402
_sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..'))
from common import (
    TestNode, fit_circle, steering_angle_from_radius,
    radius_from_steering_angle, radius_from_imu,
    update_vehicle_params,
    DEFAULT_WHEELBASE, DEFAULT_SERVO_GAIN
)


class WheelbaseTestNode(TestNode):
    
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
            'wheelbase_test',
            'wheelbase_test',
            columns,
            max_speed=max_speed,
            max_time=max_time
        )
        
        self.steering_angle = args.steering
        self.speeds = args.speeds
        self.num_laps = args.laps
        self.wheelbase = args.wheelbase
        self.direction = args.direction

        # Direction sign: with negative gain, negative angle = left
        if DEFAULT_SERVO_GAIN < 0:
            self.dir_sign = -1.0 if args.direction == 'left' else 1.0
        else:
            self.dir_sign = 1.0 if args.direction == 'left' else -1.0
        
        # Results: list of (speed, radius, residual, avg_ay, avg_omega)
        self.circle_results = []
    
    def run_circle_at_speed(self, target_speed: float):
        """Drive circles at a given speed and measure the radius."""
        steer = self.dir_sign * self.steering_angle
        
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
        
        # IMU-based radius (slip-independent cross-check)
        r_imu = radius_from_imu(avg_speed, avg_omega)
        slip_pct = 0.0
        if r_imu != float('inf') and radius > 0:
            slip_pct = abs(radius - r_imu) / radius * 100.0
        
        self.get_logger().info(
            f"  Result: R_odom={radius:.3f}m (residual={residual:.4f}m), "
            f"R_imu={r_imu:.3f}m, "
            f"slip={slip_pct:.1f}%, "
            f"avg_speed={avg_speed:.2f} m/s")
        
        if slip_pct > 10:
            self.get_logger().warn(
                f"  ⚠ Large odom/IMU radius discrepancy ({slip_pct:.0f}%) — "
                f"tire slip likely. Wheelbase estimate unreliable at this speed.")
        
        return {
            'speed_cmd': target_speed,
            'speed_actual': avg_speed,
            'radius': radius,
            'radius_imu': r_imu,
            'residual': residual,
            'avg_ay': avg_ay,
            'avg_omega': avg_omega,
            'slip_pct': slip_pct,
        }
    
    def run_test(self):
        """Execute the circle test at multiple speeds."""
        if not self.wait_for_sensors():
            return False
        
        predicted_r = radius_from_steering_angle(self.steering_angle, self.wheelbase)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("WHEELBASE VERIFICATION TEST")
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
        
        # At low speed, Ackermann: R = L / (2*sin(arctan(0.5*tan(δ))))
        # So: L = R * 2*sin(arctan(0.5*tan(δ)))
        beta = np.arctan(0.5 * np.tan(self.steering_angle))
        measured_wheelbase = lowest['radius'] * 2.0 * np.sin(beta)
        measured_angle = steering_angle_from_radius(
            lowest['radius'], self.wheelbase)
        
        # Also compute from IMU-based radius
        r_imu = lowest.get('radius_imu', lowest['radius'])
        wheelbase_imu = r_imu * 2.0 * np.sin(beta)
        
        self.get_logger().info(f"\n1. Wheelbase Estimate (from v={lowest['speed_actual']:.2f} m/s):")
        self.get_logger().info(f"   R_odom (circle fit): {lowest['radius']:.3f}m")
        self.get_logger().info(f"   R_imu  (v/ω):       {r_imu:.3f}m")
        self.get_logger().info(f"   Commanded steering: {np.degrees(self.steering_angle):.2f}°")
        self.get_logger().info(f"   Current wheelbase:  {self.wheelbase:.4f}m")
        self.get_logger().info(f"   Wheelbase (odom):   {measured_wheelbase:.4f}m")
        self.get_logger().info(f"   Wheelbase (IMU):    {wheelbase_imu:.4f}m")
        self.get_logger().info(f"   (current L implies actual δ = {np.degrees(measured_angle):.2f}°)")
        
        if lowest.get('slip_pct', 0) > 10:
            self.get_logger().warn(
                f"   ⚠ Odom/IMU radius mismatch > 10% even at lowest speed. "
                f"Check servo offset and steering calibration.")
        
        # ---- 2. Odom vs IMU radius comparison across speeds ----
        if len(self.circle_results) >= 2:
            self.get_logger().info(f"\n2. Radius Comparison (odom vs IMU):")
            self.get_logger().info(f"   {'v (m/s)':>8} {'R_odom':>8} {'R_imu':>8} {'slip%':>7}")
            for r in self.circle_results:
                ri = r.get('radius_imu', r['radius'])
                sp = r.get('slip_pct', 0)
                self.get_logger().info(
                    f"   {r['speed_actual']:8.2f} {r['radius']:8.3f} {ri:8.3f} {sp:7.1f}")
            self.get_logger().info(
                f"   NOTE: Large slip% at higher speeds is expected (tire slip).")
            self.get_logger().info(
                f"   The odom radius grows because ERPM-based position integration ")
            self.get_logger().info(
                f"   does not account for lateral tire slip during cornering.")
        
        # ---- 3. Friction estimate from lateral acceleration ----
        fastest = max(self.circle_results, key=lambda r: r['speed_actual'])
        self.get_logger().info(f"\n3. Lateral Acceleration at Max Speed:")
        self.get_logger().info(
            f"   v={fastest['speed_actual']:.2f} m/s: "
            f"a_y={fastest['avg_ay']:.3f} m/s² "
            f"(≈ {fastest['avg_ay']/9.81:.3f} g)")
        self.get_logger().info(f"   v²/R = {fastest['speed_actual']**2/fastest['radius']:.3f} m/s² (kinematic)")
        
        # ---- Summary for MPC ----
        # Prefer the IMU-based wheelbase at the lowest speed
        best_wheelbase = wheelbase_imu if lowest.get('slip_pct', 0) < 15 else measured_wheelbase
        self.get_logger().info(f"\n--- Parameters for MPC ---")
        self.get_logger().info(f"  Wheelbase: {best_wheelbase:.4f}m")
        self.get_logger().info(f"  Max steer: {np.degrees(self.steering_angle):.2f}° (commanded)")
        self.get_logger().info(f"  Update mpc_types.h:")
        self.get_logger().info(f"    #define WHEELBASE  FP_FROM_FLOAT({best_wheelbase:.4f}f)")

        # Auto-save to vehicle_params.yaml
        update_vehicle_params({
            'wheelbase': best_wheelbase,
        }, status='TESTED', logger=self.get_logger())
        self.get_logger().info("=" * 60)


def main():
    parser = argparse.ArgumentParser(description='F1/10th Wheelbase Verification Test')
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
    node = WheelbaseTestNode(args)
    
    try:
        node.run_test()
    finally:
        node.stop_car()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
