#!/usr/bin/env python3
"""
Cornering Stiffness Test for F1/10th Car

Identifies front and rear tire cornering stiffness (C_alpha_f, C_alpha_r)
by driving steady-state circles at increasing speeds with a fixed steering angle.

THEORY (Dynamic Bicycle Model in steady-state):
    In steady-state cornering (constant speed, constant yaw rate), all
    time derivatives are zero. The lateral force balance gives:

        m * v² / R = F_yf + F_yr                    (centripetal force)
        F_yf * l_f = F_yr * l_r                      (yaw moment = 0)

    Solving:
        F_yf = m * a_y * l_r / L    (front lateral force)
        F_yr = m * a_y * l_f / L    (rear lateral force)

    where a_y = v * omega (centripetal acceleration from IMU).

    Slip angles (small angle approximation):
        alpha_f = delta - (v_y + l_f * omega) / v_x  ≈ delta - l_f * omega / v_x
        alpha_r = -(v_y - l_r * omega) / v_x         ≈ l_r * omega / v_x

    (At low-to-moderate speeds v_y is small compared to v_x, so we neglect it.
     For a more accurate estimate, v_y could be derived from the dynamic model.)

    Cornering stiffness (linear tire model):
        C_alpha_f = F_yf / alpha_f    (N/rad)
        C_alpha_r = F_yr / alpha_r    (N/rad)

    These values are valid in the LINEAR region of the tire (low slip angles).
    At higher speeds the tires saturate and C_alpha drops — use the low-speed
    points for the initial estimate.

PROCEDURE:
    1. Drive circles at fixed steering angle, increasing speed
    2. At each speed, wait for steady state (constant omega)
    3. Record: v_x (odom), omega (IMU gz), a_y (IMU ay), delta (commanded)
    4. Compute slip angles and lateral forces → C_alpha

CAUTION: Higher speeds approach the grip limit. Start slow and increase gradually.

Usage:
    python3 test_cornering_stiffness.py [--steering 0.3] [--max-speed 3.5]
"""

import argparse
import time

import numpy as np
import rclpy

import sys as _sys, os as _os  # noqa: E402
_sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..'))
from common import (
    TestNode, radius_from_imu, radius_from_steering_angle,
    DEFAULT_WHEELBASE
)

GRAVITY = 9.81


class CorneringStiffnessNode(TestNode):

    def __init__(self, args):
        columns = [
            'odom_vx', 'imu_ay', 'imu_gz', 'imu_ax',
            'motor_current', 'cmd_speed', 'cmd_steering', 'phase'
        ]

        # Auto-calculate geofence if not specified (0 = auto)
        geofence = args.geofence
        if geofence <= 0:
            r_kin = radius_from_steering_angle(args.steering, args.wheelbase)
            # At moderate speeds tires may slip → radius grows
            r_max_est = r_kin * 2.0  # moderate estimate for this test's lower speeds
            geofence = 2.0 * r_max_est + 1.5  # extra margin
        
        super().__init__(
            'cornering_stiffness_test',
            'cornering_stiffness',
            columns,
            max_speed=args.max_speed * 1.3,
            max_time=args.settle_time * 20 + 60.0,
            max_distance=geofence
        )

        self.steering_angle = args.steering
        self.direction = 1.0 if args.direction == 'left' else -1.0
        self.min_speed = args.min_speed
        self.max_speed = args.max_speed
        self.speed_step = args.speed_step
        self.settle_time = args.settle_time
        self.record_time = args.record_time

        # Vehicle parameters
        self.mass = args.mass
        self.wheelbase = args.wheelbase
        self.l_f = args.l_f
        self.l_r = args.l_r

        # Results per speed point
        self.speed_results = []

    def run_test(self):
        """Execute the cornering stiffness test."""
        if not self.wait_for_sensors():
            return False

        effective_steer = self.direction * self.steering_angle
        speeds = np.arange(self.min_speed, self.max_speed + 0.01, self.speed_step)
        r_kin = radius_from_steering_angle(self.steering_angle, self.wheelbase)

        self.get_logger().info("=" * 60)
        self.get_logger().info("CORNERING STIFFNESS TEST")
        self.get_logger().info(f"Steering: {np.degrees(self.steering_angle):.1f}° "
                               f"({'left' if self.direction > 0 else 'right'})")
        self.get_logger().info(f"Speed range: {self.min_speed:.1f} - {self.max_speed:.1f} m/s "
                               f"(step {self.speed_step:.1f})")
        self.get_logger().info(f"Kinematic radius: {r_kin:.2f}m")
        if self.safety.max_distance > 0:
            min_geofence = 2.0 * r_kin + 0.2
            if self.safety.max_distance < min_geofence:
                self.get_logger().warn(
                    f"Geofence ({self.safety.max_distance:.2f}m) may be too tight for this circle. "
                    f"Recommended >= {min_geofence:.2f}m (2R + margin).")
            else:
                self.get_logger().info(
                    f"Geofence check: {self.safety.max_distance:.2f}m (recommended >= {min_geofence:.2f}m)")
        self.get_logger().info(f"Vehicle: m={self.mass:.3f}kg, "
                               f"l_f={self.l_f:.4f}m, l_r={self.l_r:.4f}m")
        self.get_logger().info("=" * 60)

        self.calibrate_imu_bias(duration=1.5)

        self.countdown(5)
        self.recorder.start()
        self.safety.set_origin(self.odom_x, self.odom_y)
        self.safety.start()
        self.test_running = True

        for speed in speeds:
            if not self.test_running:
                break

            self.get_logger().info(f"\n--- Speed: {speed:.1f} m/s ---")

            # Settle: drive for settle_time to reach steady state
            self.get_logger().info(f"Settling for {self.settle_time:.1f}s...")
            settle_start = time.monotonic()
            while time.monotonic() - settle_start < self.settle_time:
                rclpy.spin_once(self, timeout_sec=0.005)
                if not self.safety.check():
                    self.get_logger().error(f"Safety abort: {self.safety.abort_reason}")
                    self.stop_car()
                    self.test_running = False
                    break
                self.send_command(speed, effective_steer)
                imu_ax_corr = self.imu_ax - self.imu_bias_ax
                imu_ay_corr = self.imu_ay - self.imu_bias_ay
                imu_gz_corr = self.imu_gz - self.imu_bias_gz
                self.recorder.record(
                    odom_vx=self.odom_vx, imu_ay=imu_ay_corr,
                    imu_gz=imu_gz_corr, imu_ax=imu_ax_corr,
                    motor_current=self.motor_current,
                    cmd_speed=speed, cmd_steering=effective_steer,
                    phase='settle'
                )

            if not self.test_running:
                break

            # Record steady-state data
            self.get_logger().info(f"Recording for {self.record_time:.1f}s...")
            vx_samples = []
            ay_samples = []
            gz_samples = []
            consistency_samples = []

            record_start = time.monotonic()
            while time.monotonic() - record_start < self.record_time:
                rclpy.spin_once(self, timeout_sec=0.005)
                if not self.safety.check():
                    self.get_logger().error(f"Safety abort: {self.safety.abort_reason}")
                    self.stop_car()
                    self.test_running = False
                    break

                self.send_command(speed, effective_steer)

                imu_ax_corr = self.imu_ax - self.imu_bias_ax
                imu_ay_corr = self.imu_ay - self.imu_bias_ay
                imu_gz_corr = self.imu_gz - self.imu_bias_gz

                vx_samples.append(self.odom_vx)
                ay_samples.append(imu_ay_corr)
                gz_samples.append(imu_gz_corr)
                consistency_samples.append(abs(abs(imu_ay_corr) - abs(self.odom_vx * imu_gz_corr)))

                self.recorder.record(
                    odom_vx=self.odom_vx, imu_ay=imu_ay_corr,
                    imu_gz=imu_gz_corr, imu_ax=imu_ax_corr,
                    motor_current=self.motor_current,
                    cmd_speed=speed, cmd_steering=effective_steer,
                    phase='record'
                )

            if not self.test_running:
                break

            # Process this speed point
            if len(vx_samples) > 10:
                vx_arr = np.array(vx_samples)
                ay_arr = np.array(ay_samples)
                gz_arr = np.array(gz_samples)
                consistency_arr = np.array(consistency_samples)

                steady_mask = consistency_arr < 0.8
                n_total = len(vx_arr)
                if np.sum(steady_mask) >= 20:
                    vx_used = vx_arr[steady_mask]
                    ay_used = ay_arr[steady_mask]
                    gz_used = gz_arr[steady_mask]
                    consistency_used = consistency_arr[steady_mask]
                else:
                    vx_used = vx_arr
                    ay_used = ay_arr
                    gz_used = gz_arr
                    consistency_used = consistency_arr

                vx_avg = np.mean(vx_used)
                vx_std = np.std(vx_used)
                ay_avg = np.mean(ay_used)
                ay_std = np.std(ay_used)
                gz_avg = np.mean(gz_used)
                gz_std = np.std(gz_used)
                consistency_rmse = float(np.sqrt(np.mean(consistency_used**2)))
                omega = abs(gz_avg)
                n_samples = len(vx_used)

                # Slip angles (small-angle approximation, v_y ≈ 0)
                if vx_avg > 0.3 and omega > 0.01:
                    alpha_f = self.steering_angle - self.l_f * omega / vx_avg
                    alpha_r = self.l_r * omega / vx_avg

                    # Propagated uncertainty in slip angles
                    # alpha_f = delta - l_f * omega / v_x
                    # sigma_alpha_f = sqrt((l_f/v_x * sigma_omega)^2 + (l_f*omega/v_x^2 * sigma_vx)^2)
                    sigma_af = np.sqrt(
                        (self.l_f / vx_avg * gz_std) ** 2 +
                        (self.l_f * omega / vx_avg**2 * vx_std) ** 2)
                    sigma_ar = np.sqrt(
                        (self.l_r / vx_avg * gz_std) ** 2 +
                        (self.l_r * omega / vx_avg**2 * vx_std) ** 2)

                    # Lateral forces from force balance
                    a_y = abs(ay_avg)
                    F_yf = self.mass * a_y * self.l_r / self.wheelbase
                    F_yr = self.mass * a_y * self.l_f / self.wheelbase
                    sigma_Fyf = self.mass * ay_std * self.l_r / self.wheelbase
                    sigma_Fyr = self.mass * ay_std * self.l_f / self.wheelbase

                    # Cornering stiffness
                    C_af = F_yf / alpha_f if abs(alpha_f) > 0.001 else float('nan')
                    C_ar = F_yr / alpha_r if abs(alpha_r) > 0.001 else float('nan')

                    # Propagated uncertainty in C_alpha: sigma_C/C = sqrt((sigma_F/F)^2 + (sigma_a/a)^2)
                    if abs(alpha_f) > 0.001 and not np.isnan(C_af):
                        sigma_Caf = abs(C_af) * np.sqrt(
                            (sigma_Fyf / F_yf) ** 2 + (sigma_af / abs(alpha_f)) ** 2)
                    else:
                        sigma_Caf = float('nan')
                    if abs(alpha_r) > 0.001 and not np.isnan(C_ar):
                        sigma_Car = abs(C_ar) * np.sqrt(
                            (sigma_Fyr / F_yr) ** 2 + (sigma_ar / abs(alpha_r)) ** 2)
                    else:
                        sigma_Car = float('nan')

                    # Radius from IMU
                    r_imu = vx_avg / omega if omega > 0.01 else float('inf')

                    result = {
                        'speed': vx_avg,
                        'speed_cmd': speed,
                        'speed_std': vx_std,
                        'omega': omega,
                        'omega_std': gz_std,
                        'ay': a_y,
                        'ay_std': ay_std,
                        'alpha_f': alpha_f,
                        'alpha_r': alpha_r,
                        'sigma_alpha_f': sigma_af,
                        'sigma_alpha_r': sigma_ar,
                        'F_yf': F_yf,
                        'F_yr': F_yr,
                        'sigma_Fyf': sigma_Fyf,
                        'sigma_Fyr': sigma_Fyr,
                        'C_af': C_af,
                        'C_ar': C_ar,
                        'sigma_Caf': sigma_Caf,
                        'sigma_Car': sigma_Car,
                        'r_imu': r_imu,
                        'consistency_rmse': consistency_rmse,
                        'n_samples_total': n_total,
                        'n_samples': n_samples,
                    }
                    self.speed_results.append(result)

                    self.get_logger().info(
                        f"  v={vx_avg:.2f} m/s, ω={omega:.3f} rad/s, "
                        f"a_y={a_y:.3f} m/s², R={r_imu:.2f}m")
                    self.get_logger().info(
                        f"  α_f={np.degrees(alpha_f):.2f}°, "
                        f"α_r={np.degrees(alpha_r):.2f}°, "
                        f"Cα_f={C_af:.1f} N/rad, Cα_r={C_ar:.1f} N/rad")
                else:
                    self.get_logger().warn(
                        f"  Speed ({vx_avg:.2f}) or yaw rate ({omega:.3f}) too low, skipping")

        self.stop_car()
        time.sleep(0.5)
        self.stop_car()

        self.recorder.save()
        self.analyze()
        return True

    def analyze(self):
        """Analyze cornering stiffness results."""
        if not self.speed_results:
            self.get_logger().warn("No valid speed points to analyze")
            return

        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("ANALYSIS RESULTS")
        self.get_logger().info("=" * 60)

        # Table
        self.get_logger().info(
            f"\n{'v(m/s)':>7} {'ω(r/s)':>7} {'ay(m/s²)':>9} "
            f"{'αf(°)':>7} {'αr(°)':>7} {'Fyf(N)':>7} {'Fyr(N)':>7} "
            f"{'Cαf':>8} {'Cαr':>8} {'R(m)':>6}")

        for r in self.speed_results:
            self.get_logger().info(
                f"{r['speed']:7.2f} {r['omega']:7.3f} {r['ay']:9.3f} "
                f"{np.degrees(r['alpha_f']):7.2f} {np.degrees(r['alpha_r']):7.2f} "
                f"{r['F_yf']:7.2f} {r['F_yr']:7.2f} "
                f"{r['C_af']:8.1f} {r['C_ar']:8.1f} {r['r_imu']:6.2f}")

        # Use the linear region (low speed points) for the best estimate
        # Filter: only points where slip angles are small (< ~5°)
        linear_points = [r for r in self.speed_results
                         if abs(r['alpha_f']) < np.radians(5)
                         and abs(r['alpha_r']) < np.radians(5)
                         and r.get('consistency_rmse', 0.0) < 1.0
                         and not np.isnan(r['C_af'])
                         and not np.isnan(r['C_ar'])]

        if linear_points:
            C_af_values = np.array([r['C_af'] for r in linear_points], dtype=float)
            C_ar_values = np.array([r['C_ar'] for r in linear_points], dtype=float)
            sigma_Caf = np.array([r['sigma_Caf'] for r in linear_points], dtype=float)
            sigma_Car = np.array([r['sigma_Car'] for r in linear_points], dtype=float)

            valid_f = np.isfinite(C_af_values) & np.isfinite(sigma_Caf) & (sigma_Caf > 1e-9)
            valid_r = np.isfinite(C_ar_values) & np.isfinite(sigma_Car) & (sigma_Car > 1e-9)

            if np.any(valid_f):
                w_f = 1.0 / (sigma_Caf[valid_f] ** 2)
                C_af_best = float(np.sum(w_f * C_af_values[valid_f]) / np.sum(w_f))
            else:
                C_af_best = float(np.mean(C_af_values))

            if np.any(valid_r):
                w_r = 1.0 / (sigma_Car[valid_r] ** 2)
                C_ar_best = float(np.sum(w_r * C_ar_values[valid_r]) / np.sum(w_r))
            else:
                C_ar_best = float(np.mean(C_ar_values))

            self.get_logger().info(f"\n1. CORNERING STIFFNESS (linear region, α < 5°):")
            self.get_logger().info(f"   C_alpha_f = {C_af_best:.1f} N/rad "
                                   f"({len(linear_points)} points)")
            self.get_logger().info(f"   C_alpha_r = {C_ar_best:.1f} N/rad")

            # Check for tire saturation at higher speeds
            all_C_af = [r['C_af'] for r in self.speed_results if not np.isnan(r['C_af'])]
            if len(all_C_af) > 2:
                if min(all_C_af) < C_af_best * 0.7:
                    self.get_logger().info(
                        f"   Tire saturation detected: C_alpha drops at higher speeds")
                else:
                    self.get_logger().info(
                        f"   No significant tire saturation in tested speed range")
        else:
            self.get_logger().warn("No points in linear region (α < 5°)")
            self.get_logger().warn("Try lower speeds or a larger steering angle")

            # Fall back to all points
            C_af_values = [r['C_af'] for r in self.speed_results if not np.isnan(r['C_af'])]
            C_ar_values = [r['C_ar'] for r in self.speed_results if not np.isnan(r['C_ar'])]
            if C_af_values:
                C_af_best = np.mean(C_af_values)
                C_ar_best = np.mean(C_ar_values)
                self.get_logger().info(
                    f"\n   Using all points: C_alpha_f ≈ {C_af_best:.1f}, "
                    f"C_alpha_r ≈ {C_ar_best:.1f} N/rad")
            else:
                C_af_best = 0
                C_ar_best = 0

        # Understeer gradient
        if C_af_best > 0 and C_ar_best > 0:
            K_us = (self.mass * GRAVITY / self.wheelbase) * \
                   (self.l_f / C_af_best - self.l_r / C_ar_best)

            self.get_logger().info(f"\n2. UNDERSTEER GRADIENT:")
            self.get_logger().info(f"   K_us = {K_us:.4f} rad/(m/s²)")
            if K_us > 0:
                self.get_logger().info(f"   → Vehicle UNDERSTEERS (stable, safe)")
            elif K_us < 0:
                self.get_logger().info(f"   → Vehicle OVERSTEERS (unstable at limit!)")
            else:
                self.get_logger().info(f"   → Neutral steer")

        # Summary
        self.get_logger().info(f"\n--- Parameters for MPC ---")
        if C_af_best > 0:
            self.get_logger().info(f"  C_alpha_f: {C_af_best:.1f} N/rad (front)")
            self.get_logger().info(f"  C_alpha_r: {C_ar_best:.1f} N/rad (rear)")
        self.get_logger().info(f"\n  NOTE: These values assume v_y ≈ 0 (small-angle approx).")
        self.get_logger().info(f"  Valid in the linear tire region (slip angles < ~5°).")
        self.get_logger().info(f"  For higher accuracy, fit a Pacejka tire model to the full dataset.")

        # Save summary CSV with per-speed-point results
        import csv as csv_mod
        summary_path = self.recorder.filename.replace('.csv', '_summary.csv')
        with open(summary_path, 'w', newline='') as f:
            writer = csv_mod.DictWriter(f, fieldnames=[
                'speed_cmd', 'speed', 'speed_std', 'omega', 'omega_std',
                'ay', 'ay_std',
                'alpha_f_rad', 'alpha_r_rad', 'alpha_f_deg', 'alpha_r_deg',
                'sigma_alpha_f', 'sigma_alpha_r',
                'F_yf', 'F_yr', 'sigma_Fyf', 'sigma_Fyr',
                'C_alpha_f', 'C_alpha_r',
                'sigma_C_alpha_f', 'sigma_C_alpha_r',
                'r_imu', 'consistency_rmse', 'n_samples_total', 'n_samples'])
            writer.writeheader()
            for r in self.speed_results:
                writer.writerow({
                    'speed_cmd': r['speed_cmd'],
                    'speed': r['speed'],
                    'speed_std': r['speed_std'],
                    'omega': r['omega'],
                    'omega_std': r['omega_std'],
                    'ay': r['ay'],
                    'ay_std': r['ay_std'],
                    'alpha_f_rad': r['alpha_f'],
                    'alpha_r_rad': r['alpha_r'],
                    'alpha_f_deg': np.degrees(r['alpha_f']),
                    'alpha_r_deg': np.degrees(r['alpha_r']),
                    'sigma_alpha_f': r['sigma_alpha_f'],
                    'sigma_alpha_r': r['sigma_alpha_r'],
                    'F_yf': r['F_yf'],
                    'F_yr': r['F_yr'],
                    'sigma_Fyf': r['sigma_Fyf'],
                    'sigma_Fyr': r['sigma_Fyr'],
                    'C_alpha_f': r['C_af'],
                    'C_alpha_r': r['C_ar'],
                    'sigma_C_alpha_f': r['sigma_Caf'],
                    'sigma_C_alpha_r': r['sigma_Car'],
                    'r_imu': r['r_imu'],
                    'consistency_rmse': r.get('consistency_rmse', float('nan')),
                    'n_samples_total': r.get('n_samples_total', r['n_samples']),
                    'n_samples': r['n_samples'],
                })
        self.get_logger().info(f"Summary saved to {summary_path}")

        # Auto-save to vehicle_params.yaml
        from common import update_vehicle_params
        params = {}
        if C_af_best > 0:
            params['C_alpha_f'] = float(C_af_best)
            params['C_alpha_r'] = float(C_ar_best)
        if params:
            update_vehicle_params(params, status='TESTED', logger=self.get_logger())
        self.get_logger().info("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description='F1/10th Cornering Stiffness Test')
    parser.add_argument('--steering', type=float, default=0.1,
                        help='Steering angle in radians (default: 0.1 ≈ 5.7°, keep small for linear tire region)')
    parser.add_argument('--min-speed', type=float, default=1.5,
                        help='Starting speed (m/s, default: 1.5, below this forces are too small to measure)')
    parser.add_argument('--max-speed', type=float, default=3.0,
                        help='Maximum speed (m/s, default: 3.0)')
    parser.add_argument('--speed-step', type=float, default=0.5,
                        help='Speed increment (m/s, default: 0.5)')
    parser.add_argument('--settle-time', type=float, default=8.0,
                        help='Time to reach steady state per speed (s, default: 8.0)')
    parser.add_argument('--record-time', type=float, default=15.0,
                        help='Steady-state recording time per speed (s, default: 15.0, ~1 full circle at R=3.2m)')
    parser.add_argument('--direction', choices=['left', 'right'], default='left',
                        help='Circle direction (default: left)')
    parser.add_argument('--mass', type=float, default=3.314,
                        help='Vehicle mass in kg (default: 3.314)')
    parser.add_argument('--wheelbase', type=float, default=DEFAULT_WHEELBASE,
                        help=f'Wheelbase in m (default: {DEFAULT_WHEELBASE})')
    parser.add_argument('--l-f', type=float, default=0.166,
                        help='Front axle to CG distance in m (default: 0.166)')
    parser.add_argument('--l-r', type=float, default=0.16,
                        help='Rear axle to CG distance in m (default: 0.16)')
    parser.add_argument('--runs', type=int, default=5,
                        help='Number of complete test runs (default: 5)')
    parser.add_argument('--geofence', type=float, default=0.0,
                        help='Max distance from start before abort in m (default: 0=auto-calculate from steering, circle path needs ~2R)')
    args = parser.parse_args()

    rclpy.init()
    for run_idx in range(args.runs):
        if args.runs > 1:
            print(f"\n{'='*60}")
            print(f"RUN {run_idx + 1}/{args.runs}")
            print(f"{'='*60}\n")
        node = CorneringStiffnessNode(args)
        try:
            node.run_test()
        finally:
            node.stop_car()
            node.destroy_node()
        if run_idx < args.runs - 1:
            print("\n  >>> Reposition the car for the next run.")
            input("  >>> Press ENTER when ready...")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
