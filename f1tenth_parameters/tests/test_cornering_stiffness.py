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
            'odom_vx', 'odom_vy', 'imu_ay', 'imu_gz', 'imu_ax',
            'v_lidar_vx', 'v_lidar_vy',
            'motor_current', 'cmd_speed', 'cmd_steering', 'phase'
        ]

        # Auto-calculate geofence from the SMALLEST steering angle (largest circle)
        geofence = args.geofence
        if geofence <= 0:
            min_steer = min(args.steering_angles)
            r_kin = radius_from_steering_angle(min_steer, args.wheelbase)
            r_max_est = r_kin * 2.0
            geofence = 2.0 * r_max_est + 1.5
        
        super().__init__(
            'cornering_stiffness_test',
            'cornering_stiffness',
            columns,
            max_speed=args.max_speed * 1.3,
            max_time=0,  # No timeout — test manages its own timing
            max_distance=geofence
        )

        self.steering_angles = args.steering_angles
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

        speeds = np.arange(self.min_speed, self.max_speed + 0.01, self.speed_step)

        self.get_logger().info("=" * 60)
        self.get_logger().info("CORNERING STIFFNESS TEST")
        self.get_logger().info(f"Steering angles: {[f'{np.degrees(s):.1f}°' for s in self.steering_angles]}")
        if len(self.steering_angles) < 3:
            self.get_logger().warn(
                f"⚠ Only {len(self.steering_angles)} steering angle(s) specified! "
                f"A single angle produces unreliable C_alpha estimates because "
                f"the slip-angle range is too narrow. Use the default sweep of "
                f"5 angles [0.08, 0.12, 0.16, 0.20, 0.24] rad for reliable results."
            )
        self.get_logger().info(f"Direction: {'left' if self.direction > 0 else 'right'}")
        self.get_logger().info(f"Speed range: {self.min_speed:.1f} - {self.max_speed:.1f} m/s "
                               f"(step {self.speed_step:.1f})")
        for steer in self.steering_angles:
            r_kin = radius_from_steering_angle(steer, self.wheelbase)
            self.get_logger().info(
                f"  δ={np.degrees(steer):.1f}° → kinematic R={r_kin:.2f}m")
        if self.safety.max_distance > 0:
            min_steer = min(self.steering_angles)
            r_max = radius_from_steering_angle(min_steer, self.wheelbase)
            min_geofence = 2.0 * r_max + 0.2
            if self.safety.max_distance < min_geofence:
                self.get_logger().warn(
                    f"Geofence ({self.safety.max_distance:.2f}m) may be too tight. "
                    f"Recommended >= {min_geofence:.2f}m for δ={np.degrees(min_steer):.1f}°.")
            else:
                self.get_logger().info(
                    f"Geofence: {self.safety.max_distance:.2f}m (min recommended: {min_geofence:.2f}m)")
        self.get_logger().info(f"Vehicle: m={self.mass:.3f}kg, "
                               f"l_f={self.l_f:.4f}m, l_r={self.l_r:.4f}m")
        self.get_logger().info("=" * 60)

        self.calibrate_imu_bias(duration=1.5)

        self.countdown(5)
        self.recorder.start()
        self.safety.set_origin(self.odom_x, self.odom_y)
        self.safety.start()
        self.test_running = True

        for steer in self.steering_angles:
            if not self.test_running:
                break

            effective_steer = self.direction * steer

            for speed in speeds:
                if not self.test_running:
                    break

                self.get_logger().info(
                    f"\n--- δ={np.degrees(steer):.1f}°, v={speed:.1f} m/s ---")

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
                        odom_vx=self.odom_vx, odom_vy=self.odom_vy,
                        imu_ay=imu_ay_corr,
                        imu_gz=imu_gz_corr, imu_ax=imu_ax_corr,
                        v_lidar_vx=self.lidar_vx,
                        v_lidar_vy=self.lidar_vy,
                        motor_current=self.motor_current,
                        cmd_speed=speed, cmd_steering=effective_steer,
                        phase='settle'
                    )

                if not self.test_running:
                    break

                # Record steady-state data
                self.get_logger().info(f"Recording for {self.record_time:.1f}s...")
                vx_samples = []
                vy_odom_samples = []
                ay_samples = []
                gz_samples = []
                vy_lidar_samples = []
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
                    vy_odom_samples.append(self.odom_vy)
                    ay_samples.append(imu_ay_corr)
                    gz_samples.append(imu_gz_corr)
                    vy_lidar_samples.append(self.lidar_vy)
                    consistency_samples.append(abs(abs(imu_ay_corr) - abs(self.odom_vx * imu_gz_corr)))

                    self.recorder.record(
                        odom_vx=self.odom_vx, odom_vy=self.odom_vy,
                        imu_ay=imu_ay_corr,
                        imu_gz=imu_gz_corr, imu_ax=imu_ax_corr,
                        v_lidar_vx=self.lidar_vx,
                        v_lidar_vy=self.lidar_vy,
                        motor_current=self.motor_current,
                        cmd_speed=speed, cmd_steering=effective_steer,
                        phase='record'
                    )

                if not self.test_running:
                    break

                # Process this speed point
                if len(vx_samples) > 10:
                    vx_arr = np.array(vx_samples)
                    vy_odom_arr = np.array(vy_odom_samples)
                    ay_arr = np.array(ay_samples)
                    gz_arr = np.array(gz_samples)
                    vy_lidar_arr = np.array(vy_lidar_samples)
                    consistency_arr = np.array(consistency_samples)

                    steady_mask = consistency_arr < 0.8
                    n_total = len(vx_arr)
                    if np.sum(steady_mask) >= 20:
                        vx_used = vx_arr[steady_mask]
                        vy_odom_used = vy_odom_arr[steady_mask]
                        ay_used = ay_arr[steady_mask]
                        gz_used = gz_arr[steady_mask]
                        vy_lidar_used = vy_lidar_arr[steady_mask]
                        consistency_used = consistency_arr[steady_mask]
                    else:
                        vx_used = vx_arr
                        vy_odom_used = vy_odom_arr
                        ay_used = ay_arr
                        gz_used = gz_arr
                        vy_lidar_used = vy_lidar_arr
                        consistency_used = consistency_arr

                    vx_avg = np.mean(vx_used)
                    vx_std = np.std(vx_used)
                    vy_odom_avg = np.mean(vy_odom_used)
                    vy_odom_std = np.std(vy_odom_used)
                    ay_avg = np.mean(ay_used)
                    ay_std = np.std(ay_used)
                    gz_avg = np.mean(gz_used)
                    gz_std = np.std(gz_used)
                    vy_lidar_avg = np.mean(vy_lidar_used)
                    vy_lidar_std = np.std(vy_lidar_used)
                    consistency_rmse = float(np.sqrt(np.mean(consistency_used**2)))
                    omega = abs(gz_avg)
                    n_samples = len(vx_used)

                    # Slip angles WITH sideslip from v_y measurement
                    # β = v_y / v_x  (body sideslip angle)
                    # α_f = δ − β − l_f·ω/v_x
                    # α_r = l_r·ω/v_x − β
                    # Without v_y, F_y/α is a kinematic identity (∝ v²)
                    # that doesn't reflect actual tire stiffness.
                    if vx_avg > 0.3 and omega > 0.01:
                        # Auto-detect v_y source:
                        # - Sim: odom_vy has variance (ground truth)
                        # - Real hw: odom_vy ≡ 0 (VESC), use LiDAR v_y
                        if vy_odom_std > 1e-6:
                            vy_for_slip = vy_odom_avg
                            vy_for_slip_std = vy_odom_std
                            vy_source = "odom"
                        else:
                            vy_for_slip = vy_lidar_avg
                            vy_for_slip_std = vy_lidar_std
                            vy_source = "LiDAR"

                        beta = vy_for_slip / vx_avg if abs(vx_avg) > 0.1 else 0.0
                        alpha_f = steer - beta - self.l_f * omega / vx_avg
                        alpha_r = self.l_r * omega / vx_avg - beta

                        # LiDAR-based sideslip for comparison
                        beta_lidar = vy_lidar_avg / vx_avg if abs(vx_avg) > 0.1 else 0.0

                        # Also compute the naive (no-sideslip) values for comparison
                        alpha_f_naive = steer - self.l_f * omega / vx_avg
                        alpha_r_naive = self.l_r * omega / vx_avg

                        # Propagated uncertainty in slip angles
                        sigma_beta = abs(vy_for_slip_std / vx_avg) if abs(vx_avg) > 0.1 else 0.0
                        sigma_af = np.sqrt(
                            sigma_beta ** 2 +
                            (self.l_f / vx_avg * gz_std) ** 2 +
                            (self.l_f * omega / vx_avg**2 * vx_std) ** 2)
                        sigma_ar = np.sqrt(
                            sigma_beta ** 2 +
                            (self.l_r / vx_avg * gz_std) ** 2 +
                            (self.l_r * omega / vx_avg**2 * vx_std) ** 2)

                        # Lateral forces from force balance
                        a_y = abs(ay_avg)
                        F_yf = self.mass * a_y * self.l_r / self.wheelbase
                        F_yr = self.mass * a_y * self.l_f / self.wheelbase
                        sigma_Fyf = self.mass * ay_std * self.l_r / self.wheelbase
                        sigma_Fyr = self.mass * ay_std * self.l_f / self.wheelbase

                        # Cornering stiffness (with sideslip correction)
                        C_af = F_yf / alpha_f if abs(alpha_f) > 0.001 else float('nan')
                        C_ar = F_yr / alpha_r if abs(alpha_r) > 0.001 else float('nan')

                        # Naive (no sideslip) for comparison
                        C_af_naive = F_yf / alpha_f_naive if abs(alpha_f_naive) > 0.001 else float('nan')
                        C_ar_naive = F_yr / alpha_r_naive if abs(alpha_r_naive) > 0.001 else float('nan')

                        # Propagated uncertainty
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
                            'steering_angle': steer,
                            'speed_std': vx_std,
                            'omega': omega,
                            'omega_std': gz_std,
                            'ay': a_y,
                            'ay_std': ay_std,
                            'vy_odom': vy_odom_avg,
                            'vy_odom_std': vy_odom_std,
                            'vy_lidar': vy_lidar_avg,
                            'vy_lidar_std': vy_lidar_std,
                            'beta': beta,
                            'beta_lidar': beta_lidar,
                            'alpha_f': alpha_f,
                            'alpha_r': alpha_r,
                            'alpha_f_naive': alpha_f_naive,
                            'alpha_r_naive': alpha_r_naive,
                            'sigma_alpha_f': sigma_af,
                            'sigma_alpha_r': sigma_ar,
                            'F_yf': F_yf,
                            'F_yr': F_yr,
                            'sigma_Fyf': sigma_Fyf,
                            'sigma_Fyr': sigma_Fyr,
                            'C_af': C_af,
                            'C_ar': C_ar,
                            'C_af_naive': C_af_naive,
                            'C_ar_naive': C_ar_naive,
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
                            f"a_y={a_y:.3f} m/s², R={r_imu:.2f}m, v_y src={vy_source}")
                        self.get_logger().info(
                            f"  β={np.degrees(beta):.2f}° (v_y={vy_for_slip:.4f}), "
                            f"β_lidar={np.degrees(beta_lidar):.2f}° (v_y={vy_lidar_avg:.4f})")
                        self.get_logger().info(
                            f"  α_f={np.degrees(alpha_f):.2f}° (naive: {np.degrees(alpha_f_naive):.2f}°), "
                            f"α_r={np.degrees(alpha_r):.2f}° (naive: {np.degrees(alpha_r_naive):.2f}°)")
                        self.get_logger().info(
                            f"  Cα_f={C_af:.1f} N/rad (naive: {C_af_naive:.1f}), "
                            f"Cα_r={C_ar:.1f} N/rad (naive: {C_ar_naive:.1f})")
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
        """Analyze cornering stiffness results.
        
        Uses LiDAR-measured lateral velocity (v_y) to compute body sideslip,
        which breaks the kinematic degeneracy that plagues naive steady-state
        circle analysis. Without v_y, C_alpha = F_y/alpha is always proportional
        to v^2 (a kinematic identity, not a tire property).
        """
        if not self.speed_results:
            self.get_logger().warn("No valid speed points to analyze")
            return

        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("ANALYSIS RESULTS (with LiDAR sideslip correction)")
        self.get_logger().info("=" * 60)

        # Table header
        self.get_logger().info(
            f"\n{'v(m/s)':>7} {'δ(°)':>6} {'β(°)':>6} {'ω(r/s)':>7} {'ay(m/s²)':>9} "
            f"{'αf(°)':>7} {'αr(°)':>7} {'Fyf(N)':>7} {'Fyr(N)':>7} "
            f"{'Cαf':>7} {'Cαr':>7} {'Cαf_n':>7} {'Cαr_n':>7}")

        for r in self.speed_results:
            self.get_logger().info(
                f"{r['speed']:7.2f} {np.degrees(r['steering_angle']):6.1f} "
                f"{np.degrees(r['beta']):6.2f} {r['omega']:7.3f} {r['ay']:9.3f} "
                f"{np.degrees(r['alpha_f']):7.2f} {np.degrees(r['alpha_r']):7.2f} "
                f"{r['F_yf']:7.2f} {r['F_yr']:7.2f} "
                f"{r['C_af']:7.1f} {r['C_ar']:7.1f} "
                f"{r['C_af_naive']:7.1f} {r['C_ar_naive']:7.1f}")

        self.get_logger().info(
            f"\n  (Cαf_n / Cαr_n = naive values without sideslip, for comparison)")

        # --- Method 1: Weighted average of per-point C_alpha ---
        valid_points = [r for r in self.speed_results
                        if not np.isnan(r['C_af']) and not np.isnan(r['C_ar'])
                        and abs(r['alpha_f']) > 0.005 and abs(r['alpha_r']) > 0.005]

        if valid_points:
            C_af_values = np.array([r['C_af'] for r in valid_points])
            C_ar_values = np.array([r['C_ar'] for r in valid_points])
            sigma_Caf = np.array([r['sigma_Caf'] for r in valid_points])
            sigma_Car = np.array([r['sigma_Car'] for r in valid_points])

            # Weighted average
            valid_f = np.isfinite(sigma_Caf) & (sigma_Caf > 1e-9)
            valid_r = np.isfinite(sigma_Car) & (sigma_Car > 1e-9)

            if np.any(valid_f):
                w_f = 1.0 / (sigma_Caf[valid_f] ** 2)
                C_af_avg = float(np.sum(w_f * C_af_values[valid_f]) / np.sum(w_f))
                C_af_std = float(np.std(C_af_values[valid_f]))
            else:
                C_af_avg = float(np.mean(C_af_values))
                C_af_std = float(np.std(C_af_values))

            if np.any(valid_r):
                w_r = 1.0 / (sigma_Car[valid_r] ** 2)
                C_ar_avg = float(np.sum(w_r * C_ar_values[valid_r]) / np.sum(w_r))
                C_ar_std = float(np.std(C_ar_values[valid_r]))
            else:
                C_ar_avg = float(np.mean(C_ar_values))
                C_ar_std = float(np.std(C_ar_values))

            self.get_logger().info(f"\n1. CORNERING STIFFNESS (weighted average, {len(valid_points)} points):")
            self.get_logger().info(f"   C_alpha_f = {C_af_avg:.1f} ± {C_af_std:.1f} N/rad")
            self.get_logger().info(f"   C_alpha_r = {C_ar_avg:.1f} ± {C_ar_std:.1f} N/rad")

            # --- Method 2: Linear regression F_y vs alpha ---
            alpha_f_arr = np.array([r['alpha_f'] for r in valid_points])
            alpha_r_arr = np.array([r['alpha_r'] for r in valid_points])
            F_yf_arr = np.array([r['F_yf'] for r in valid_points])
            F_yr_arr = np.array([r['F_yr'] for r in valid_points])

            # Regression through origin: F_y = C_alpha * alpha
            C_af_reg = float(np.sum(alpha_f_arr * F_yf_arr) / np.sum(alpha_f_arr**2))
            C_ar_reg = float(np.sum(alpha_r_arr * F_yr_arr) / np.sum(alpha_r_arr**2))

            # R² for regression
            F_yf_pred = C_af_reg * alpha_f_arr
            F_yr_pred = C_ar_reg * alpha_r_arr
            ss_res_f = np.sum((F_yf_arr - F_yf_pred)**2)
            ss_tot_f = np.sum((F_yf_arr - np.mean(F_yf_arr))**2)
            ss_res_r = np.sum((F_yr_arr - F_yr_pred)**2)
            ss_tot_r = np.sum((F_yr_arr - np.mean(F_yr_arr))**2)
            R2_f = 1 - ss_res_f / ss_tot_f if ss_tot_f > 0 else 0
            R2_r = 1 - ss_res_r / ss_tot_r if ss_tot_r > 0 else 0

            self.get_logger().info(f"\n2. CORNERING STIFFNESS (linear regression F_y = C_α·α):")
            self.get_logger().info(f"   C_alpha_f = {C_af_reg:.1f} N/rad  (R² = {R2_f:.4f})")
            self.get_logger().info(f"   C_alpha_r = {C_ar_reg:.1f} N/rad  (R² = {R2_r:.4f})")

            # Check if naive values show the v² artifact
            C_af_naive_vals = [r['C_af_naive'] for r in valid_points if not np.isnan(r['C_af_naive'])]
            C_ar_naive_vals = [r['C_ar_naive'] for r in valid_points if not np.isnan(r['C_ar_naive'])]
            if len(C_ar_naive_vals) > 2:
                cv_naive = np.std(C_ar_naive_vals) / np.mean(C_ar_naive_vals)
                cv_odom = np.std(C_ar_values) / np.mean(C_ar_values)
                self.get_logger().info(
                    f"\n   Naive CoV(C_αr) = {cv_naive:.2f} vs odom v_y CoV = {cv_odom:.2f}")
                if cv_naive > 0.3:
                    self.get_logger().info(
                        f"   Naive method shows v² artifact (CoV > 0.3), odom v_y corrects it")

            # Use regression as the best estimate
            C_af_best = C_af_reg
            C_ar_best = C_ar_reg

        else:
            self.get_logger().warn("No valid data points for analysis")
            C_af_best = 0
            C_ar_best = 0

        # Understeer gradient
        if C_af_best > 0 and C_ar_best > 0:
            K_us = (self.mass * GRAVITY / self.wheelbase) * \
                   (self.l_f / C_af_best - self.l_r / C_ar_best)

            self.get_logger().info(f"\n3. UNDERSTEER GRADIENT:")
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
        self.get_logger().info(
            f"\n  NOTE: Slip angles computed WITH measured sideslip (v_y).")
        self.get_logger().info(
            f"  This overcomes the v² kinematic degeneracy of the naive method.")

        # Save summary CSV with per-speed-point results
        import csv as csv_mod
        summary_path = self.recorder.filename.replace('.csv', '_summary.csv')
        with open(summary_path, 'w', newline='') as f:
            writer = csv_mod.DictWriter(f, fieldnames=[
                'speed_cmd', 'steering_angle_rad', 'speed', 'speed_std',
                'omega', 'omega_std', 'ay', 'ay_std',
                'vy_odom', 'vy_odom_std', 'vy_lidar', 'vy_lidar_std',
                'beta_rad', 'beta_lidar_rad',
                'alpha_f_rad', 'alpha_r_rad', 'alpha_f_deg', 'alpha_r_deg',
                'alpha_f_naive_rad', 'alpha_r_naive_rad',
                'sigma_alpha_f', 'sigma_alpha_r',
                'F_yf', 'F_yr', 'sigma_Fyf', 'sigma_Fyr',
                'C_alpha_f', 'C_alpha_r',
                'C_alpha_f_naive', 'C_alpha_r_naive',
                'sigma_C_alpha_f', 'sigma_C_alpha_r',
                'r_imu', 'consistency_rmse', 'n_samples_total', 'n_samples'])
            writer.writeheader()
            for r in self.speed_results:
                writer.writerow({
                    'speed_cmd': r['speed_cmd'],
                    'steering_angle_rad': r['steering_angle'],
                    'speed': r['speed'],
                    'speed_std': r['speed_std'],
                    'omega': r['omega'],
                    'omega_std': r['omega_std'],
                    'ay': r['ay'],
                    'ay_std': r['ay_std'],
                    'vy_odom': r['vy_odom'],
                    'vy_odom_std': r['vy_odom_std'],
                    'vy_lidar': r['vy_lidar'],
                    'vy_lidar_std': r['vy_lidar_std'],
                    'beta_rad': r['beta'],
                    'beta_lidar_rad': r['beta_lidar'],
                    'alpha_f_rad': r['alpha_f'],
                    'alpha_r_rad': r['alpha_r'],
                    'alpha_f_deg': np.degrees(r['alpha_f']),
                    'alpha_r_deg': np.degrees(r['alpha_r']),
                    'alpha_f_naive_rad': r['alpha_f_naive'],
                    'alpha_r_naive_rad': r['alpha_r_naive'],
                    'sigma_alpha_f': r['sigma_alpha_f'],
                    'sigma_alpha_r': r['sigma_alpha_r'],
                    'F_yf': r['F_yf'],
                    'F_yr': r['F_yr'],
                    'sigma_Fyf': r['sigma_Fyf'],
                    'sigma_Fyr': r['sigma_Fyr'],
                    'C_alpha_f': r['C_af'],
                    'C_alpha_r': r['C_ar'],
                    'C_alpha_f_naive': r['C_af_naive'],
                    'C_alpha_r_naive': r['C_ar_naive'],
                    'sigma_C_alpha_f': r['sigma_Caf'],
                    'sigma_C_alpha_r': r['sigma_Car'],
                    'r_imu': r['r_imu'],
                    'consistency_rmse': r.get('consistency_rmse', float('nan')),
                    'n_samples_total': r.get('n_samples_total', r['n_samples']),
                    'n_samples': r['n_samples'],
                })
        self.get_logger().info(f"Summary saved to {summary_path}")

        # Auto-save to vehicle_params.yaml
        # Only save if multiple steering angles were tested (single-angle
        # data is unreliable — C_alpha varies with speed when the slip-angle
        # range is too narrow).
        from common import update_vehicle_params
        params = {}
        n_angles_tested = len(set(
            r.get('steering_angle', 0) for r in self.speed_results
        )) if self.speed_results else len(self.steering_angles)
        # Fall back to checking args if steering_angle not in results
        if n_angles_tested <= 1:
            n_angles_tested = len(self.steering_angles)

        if n_angles_tested < 3:
            self.get_logger().warn(
                f"  ⚠ Only {n_angles_tested} steering angle(s) tested — "
                f"NOT saving C_alpha to YAML (unreliable with narrow "
                f"slip-angle range). Re-run with default angle sweep "
                f"[0.08, 0.12, 0.16, 0.20, 0.24] for valid results.")
        elif C_af_best > 0:
            params['C_alpha_f'] = float(C_af_best)
            params['C_alpha_r'] = float(C_ar_best)
        if params:
            update_vehicle_params(params, status='TESTED', logger=self.get_logger())
        self.get_logger().info("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description='F1/10th Cornering Stiffness Test')
    parser.add_argument('--steering', '--steering-angles', type=float, nargs='+',
                        default=[0.08, 0.12, 0.16, 0.20, 0.24],
                        dest='steering_angles',
                        help='Steering angles in radians (default: 0.08 0.12 0.16 0.20 0.24, '
                             'i.e. ~4.6° to ~13.8°, sweeps through different slip angles)')
    parser.add_argument('--min-speed', type=float, default=1.5,
                        help='Starting speed (m/s, default: 1.5)')
    parser.add_argument('--max-speed', type=float, default=2.5,
                        help='Maximum speed (m/s, default: 2.5, keep moderate to stay in linear tire region)')
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
