#!/usr/bin/env python3
"""
Longitudinal Tire Stiffness Test for F1/10th Car

Identifies the longitudinal tire stiffness (C_x) by measuring the relationship
between wheel slip ratio and longitudinal force during acceleration and braking.

THEORY:
    Longitudinal tire force (linear region):
        F_x = C_x * kappa

    where kappa is the slip ratio:
        Driving:  kappa = (v_wheel - v_body) / max(v_wheel, v_body)
        Braking:  kappa = (v_wheel - v_body) / max(|v_wheel|, |v_body|)

    kappa > 0 → driving (traction), kappa < 0 → braking

    v_wheel comes from ERPM (speed_to_erpm_gain), which is what the VESC
    measures directly. v_body comes from IMU integration, which represents
    the actual vehicle speed independent of wheel state.

    F_x = m * a_x (from IMU), so:
        C_x = m * a_x / kappa

    In the linear region (small kappa), C_x is approximately constant.
    At higher slip the tires saturate (peak friction, then sliding).

PROCEDURE:
    1. Calibrate IMU bias while stationary
    2. Accelerate from standstill → record (v_wheel, v_imu, a_x, current)
    3. Cruise at steady speed → reset IMU anchor
    4. Brake to standstill → record (v_wheel, v_imu, a_x, current)
    5. Compute slip ratio and C_x in the linear region

NOTE: The Traxxas Slash 4x4 is all-wheel drive, so all four wheels contribute.
      The effective C_x here is the combined longitudinal stiffness.

Usage:
    python3 test_longitudinal_stiffness.py [--max-speed 3.0]
"""

import argparse
import time

import numpy as np
import rclpy

import sys as _sys, os as _os  # noqa: E402
_sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..'))
from common import TestNode, ImuVelocityEstimator, DEFAULT_ERPM_GAIN


class LongitudinalStiffnessNode(TestNode):

    def __init__(self, args):
        columns = [
            'odom_vx', 'imu_ax', 'imu_ay', 'imu_az',
            'motor_rpm', 'motor_current',
            'v_imu', 'slip_ratio',
            'cmd_speed', 'phase'
        ]

        super().__init__(
            'longitudinal_stiffness_test',
            'longitudinal_stiffness',
            columns,
            max_speed=args.max_speed * 1.3,
            max_time=0,
            max_distance=args.geofence
        )

        self.max_speed = args.max_speed
        self.accel_time = args.accel_time
        self.cruise_time = args.cruise_time
        self.mass = args.mass
        self.erpm_gain = args.erpm_gain

        self.imu_vel = ImuVelocityEstimator()

        # Collected data per phase
        self.accel_data = []
        self.cruise_data = []
        self.decel_data = []

    def pitch_corrected_ax(self, ax_biased, az_raw, is_braking=True):
        """
        Correct longitudinal acceleration for pitch-induced gravity projection.

        When the car pitches during braking/acceleration, gravity projects onto
        the longitudinal axis:
            During braking (nose down): measured deceleration is too large
            During acceleration (nose up): measured acceleration is too large

        Correction uses the vertical accelerometer:
            |θ_pitch| = arccos(clamp(az / g_static, -1, 1))
            gravity_x = g_static * sin(|θ_pitch|)

        Returns (ax_corrected, pitch_angle_degrees).
        """
        g_ref = abs(self.az_static) if hasattr(self, 'az_static') else 9.81
        cos_pitch = np.clip(az_raw / g_ref, -1.0, 1.0)
        pitch_mag = float(np.arccos(cos_pitch))
        gravity_x = g_ref * np.sin(pitch_mag)

        if is_braking:
            # Nose down: gravity makes measured ax more negative → add correction
            ax_corrected = ax_biased + gravity_x
        else:
            # Nose up: gravity makes measured ax more positive → subtract correction
            ax_corrected = ax_biased - gravity_x

        return ax_corrected, np.degrees(pitch_mag)

    def run_test(self):
        """Execute the longitudinal stiffness test."""
        if not self.wait_for_sensors():
            return False

        self.get_logger().info("=" * 60)
        self.get_logger().info("LONGITUDINAL TIRE STIFFNESS TEST")
        self.get_logger().info(f"Max speed: {self.max_speed:.1f} m/s")
        self.get_logger().info(f"Cruise time: {self.cruise_time:.1f} s")
        self.get_logger().info(f"Vehicle mass: {self.mass:.3f} kg")
        self.get_logger().info("=" * 60)

        # ---- IMU Bias Calibration ----
        self.calibrate_imu_bias(duration=2.0)
        imu_bias = self.imu_vel.calibrate_bias([self.imu_bias_ax])
        self.az_static = self.imu_bias_az  # gravity reference for pitch compensation
        self.get_logger().info(f"Using IMU ax bias: {imu_bias:.4f} m/s²")
        self.get_logger().info(f"Static az (gravity ref): {self.az_static:.4f} m/s²")

        self.countdown(3)
        self.recorder.start()
        self.safety.start()
        self.test_running = True

        # ---- Phase 1: Acceleration ----
        self.get_logger().info("\n--- Phase 1: ACCELERATION ---")
        self.imu_vel.reset(initial_velocity=0.0)
        phase_start = time.monotonic()
        last_t = phase_start

        reached_target = False
        reached_target_time = None
        while time.monotonic() - phase_start < self.accel_time:
            rclpy.spin_once(self, timeout_sec=0.005)
            if not self.safety.check():
                self.get_logger().error(f"Safety abort: {self.safety.abort_reason}")
                self.stop_car()
                self.test_running = False
                break

            now = time.monotonic()
            dt = now - last_t
            last_t = now

            self.send_command(self.max_speed, 0.0)

            # Pitch-corrected acceleration (nose-up during accel → subtract correction)
            ax_corr = self.imu_ax - imu_bias
            ax_pitch, pitch_deg = self.pitch_corrected_ax(
                ax_corr, self.imu_az, is_braking=False)

            v_imu = self.imu_vel.update(self.imu_ax, dt)
            v_wheel = self.odom_vx  # ERPM-based

            # Slip ratio (driving: wheel faster than body for traction)
            v_max = max(abs(v_wheel), abs(v_imu), 0.1)
            kappa = (v_wheel - v_imu) / v_max

            self.accel_data.append({
                't': now - phase_start,
                'v_wheel': v_wheel,
                'v_imu': v_imu,
                'ax': ax_pitch,
                'ax_raw': ax_corr,
                'pitch_deg': pitch_deg,
                'kappa': kappa,
                'current': self.motor_current,
            })

            self.recorder.record(
                odom_vx=v_wheel, imu_ax=ax_pitch, imu_ay=self.imu_ay,
                imu_az=self.imu_az,
                motor_rpm=self.motor_rpm, motor_current=self.motor_current,
                v_imu=v_imu, slip_ratio=kappa,
                cmd_speed=self.max_speed, phase='acceleration'
            )

            # Stop accel phase once target speed is reached
            if abs(v_wheel) >= self.max_speed * 0.95 and not reached_target:
                reached_target = True
                reached_target_time = now
                self.get_logger().info(
                    f"  Reached target speed ({v_wheel:.2f} m/s) "
                    f"in {now - phase_start:.2f}s")

            # End accel phase 0.5s after reaching target
            if reached_target and (now - reached_target_time) > 0.5:
                break

        if not self.test_running:
            self.recorder.save()
            return False

        # ---- Phase 2: Cruise at steady state ----
        self.get_logger().info(f"\n--- Phase 2: CRUISE ({self.cruise_time:.1f}s) ---")
        cruise_start = time.monotonic()
        last_t = cruise_start

        while time.monotonic() - cruise_start < self.cruise_time:
            rclpy.spin_once(self, timeout_sec=0.005)
            if not self.safety.check():
                self.get_logger().error(f"Safety abort during cruise: {self.safety.abort_reason}")
                self.stop_car()
                self.test_running = False
                break

            now = time.monotonic()
            dt = now - last_t
            last_t = now

            self.send_command(self.max_speed, 0.0)

            v_imu = self.imu_vel.update(self.imu_ax, dt)
            v_wheel = self.odom_vx
            ax_corr = self.imu_ax - imu_bias

            v_max = max(abs(v_wheel), abs(v_imu), 0.1)
            kappa = (v_wheel - v_imu) / v_max

            self.cruise_data.append({
                't': now - cruise_start,
                'v_wheel': v_wheel,
                'v_imu': v_imu,
                'ax': ax_corr,
                'az': self.imu_az,
                'kappa': kappa,
                'current': self.motor_current,
            })

            self.recorder.record(
                odom_vx=v_wheel, imu_ax=ax_corr, imu_ay=self.imu_ay,
                imu_az=self.imu_az,
                motor_rpm=self.motor_rpm, motor_current=self.motor_current,
                v_imu=v_imu, slip_ratio=kappa,
                cmd_speed=self.max_speed, phase='cruise'
            )

        if not self.test_running:
            self.recorder.save()
            return False

        # Report cruise stats and compute refined IMU bias from cruise
        cruise_bias = imu_bias  # fallback
        if self.cruise_data:
            cruise_kappa = np.array([d['kappa'] for d in self.cruise_data])
            cruise_vw = np.array([d['v_wheel'] for d in self.cruise_data])
            cruise_vi = np.array([d['v_imu'] for d in self.cruise_data])
            # During cruise, true ax ≈ 0 → raw imu_ax ≈ bias
            # ax_corr was stored as (raw - standstill_bias), so raw = ax_corr + standstill_bias
            cruise_ax_corr = np.array([d['ax'] for d in self.cruise_data])
            cruise_raw_ax = cruise_ax_corr + imu_bias
            cruise_bias = float(np.mean(cruise_raw_ax))

            self.get_logger().info(
                f"  Cruise: v_wheel={np.mean(cruise_vw):.2f}±{np.std(cruise_vw):.3f}, "
                f"v_imu={np.mean(cruise_vi):.2f}±{np.std(cruise_vi):.3f}, "
                f"kappa={np.mean(cruise_kappa):.4f}±{np.std(cruise_kappa):.4f}")
            self.get_logger().info(
                f"  IMU bias: standstill={imu_bias:.4f}, cruise-refined={cruise_bias:.4f}, "
                f"delta={cruise_bias - imu_bias:.4f} m/s²")

        # Use cruise-refined bias for braking phase
        braking_bias = cruise_bias

        # Anchor IMU to current odom speed (known-good at steady state)
        # Also update IMU estimator bias with cruise-refined value
        self.imu_vel.reset(initial_velocity=abs(self.odom_vx), bias=braking_bias)
        self.get_logger().info(f"  IMU velocity anchored to {abs(self.odom_vx):.2f} m/s")

        # ---- Phase 3: Braking ----
        self.get_logger().info("\n--- Phase 3: BRAKING (with pitch compensation) ---")
        self.get_logger().info(f"  Using cruise-refined IMU bias: {braking_bias:.4f} m/s²")
        phase_start = time.monotonic()
        last_t = phase_start

        # Track pitch-corrected velocity separately
        v_imu_pc = abs(self.odom_vx)  # start from anchored value

        while (abs(self.odom_vx) > 0.1 or self.imu_vel.velocity > 0.1) and \
              time.monotonic() - phase_start < 10.0:
            rclpy.spin_once(self, timeout_sec=0.005)

            now = time.monotonic()
            dt = now - last_t
            last_t = now

            self.send_command(0.0, 0.0)

            # Raw bias-corrected acceleration
            ax_corr = self.imu_ax - braking_bias

            # Pitch-corrected acceleration (nose-down during braking)
            ax_pitch, pitch_deg = self.pitch_corrected_ax(
                ax_corr, self.imu_az, is_braking=True)

            # Integrate pitch-corrected acceleration for body velocity
            v_imu_pc += ax_pitch * dt
            v_imu_pc = max(v_imu_pc, 0.0)

            # Also keep the uncorrected v_imu for comparison
            v_imu = self.imu_vel.update(self.imu_ax, dt)

            v_wheel = self.odom_vx

            # Slip ratio using pitch-corrected body velocity
            v_max = max(abs(v_wheel), abs(v_imu_pc), 0.1)
            kappa = (v_wheel - v_imu_pc) / v_max

            self.decel_data.append({
                't': now - phase_start,
                'v_wheel': v_wheel,
                'v_imu': v_imu_pc,
                'v_imu_raw': v_imu,
                'ax': ax_pitch,
                'ax_raw': ax_corr,
                'pitch_deg': pitch_deg,
                'kappa': kappa,
                'current': self.motor_current,
            })

            self.recorder.record(
                odom_vx=v_wheel, imu_ax=ax_pitch, imu_ay=self.imu_ay,
                imu_az=self.imu_az,
                motor_rpm=self.motor_rpm, motor_current=self.motor_current,
                v_imu=v_imu_pc, slip_ratio=kappa,
                cmd_speed=0.0, phase='braking'
            )

        self.stop_car()
        time.sleep(0.5)
        self.stop_car()

        self.recorder.save()
        self.analyze()
        return True

    def analyze(self):
        """Analyze longitudinal stiffness results with pitch compensation."""
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("ANALYSIS RESULTS (pitch-compensated)")
        self.get_logger().info("=" * 60)

        # Cruise phase summary
        if self.cruise_data:
            cruise_kappa = np.array([d['kappa'] for d in self.cruise_data])
            cruise_vw = np.array([d['v_wheel'] for d in self.cruise_data])
            cruise_vi = np.array([d['v_imu'] for d in self.cruise_data])
            self.get_logger().info(f"\nCRUISE (steady-state baseline):")
            self.get_logger().info(f"  Samples: {len(self.cruise_data)}")
            self.get_logger().info(f"  v_wheel: {np.mean(cruise_vw):.2f} ± {np.std(cruise_vw):.3f} m/s")
            self.get_logger().info(f"  v_imu:   {np.mean(cruise_vi):.2f} ± {np.std(cruise_vi):.3f} m/s")
            self.get_logger().info(f"  kappa:   {np.mean(cruise_kappa):.4f} ± {np.std(cruise_kappa):.4f}")

        for phase_name, data in [('ACCELERATION', self.accel_data),
                                  ('BRAKING', self.decel_data)]:
            if not data:
                continue

            kappa_arr = np.array([d['kappa'] for d in data])
            ax_arr = np.array([d['ax'] for d in data])
            vw_arr = np.array([d['v_wheel'] for d in data])
            vi_arr = np.array([d['v_imu'] for d in data])
            F_x = self.mass * ax_arr  # Longitudinal force (pitch-corrected)

            self.get_logger().info(f"\n{phase_name}:")
            self.get_logger().info(f"  Samples: {len(data)}")
            self.get_logger().info(f"  Speed range (wheel): {np.min(vw_arr):.2f} - {np.max(vw_arr):.2f} m/s")
            self.get_logger().info(f"  Speed range (IMU):   {np.min(vi_arr):.2f} - {np.max(vi_arr):.2f} m/s")
            self.get_logger().info(f"  Slip ratio range: {np.min(kappa_arr):.4f} to {np.max(kappa_arr):.4f}")
            self.get_logger().info(f"  Max |F_x|: {np.max(np.abs(F_x)):.2f} N")

            # Show pitch correction stats if available
            if 'pitch_deg' in data[0]:
                pitch_arr = np.array([d['pitch_deg'] for d in data])
                self.get_logger().info(f"  Dynamic pitch: {np.mean(pitch_arr):.1f}° "
                                       f"(max {np.max(pitch_arr):.1f}°)")
            if 'ax_raw' in data[0]:
                ax_raw_arr = np.array([d['ax_raw'] for d in data])
                Fx_raw = self.mass * ax_raw_arr
                self.get_logger().info(f"  Max |F_x| (uncorrected): {np.max(np.abs(Fx_raw)):.2f} N")

            # Use EARLY braking window for best accuracy (first 150ms)
            if phase_name == 'BRAKING':
                t_arr = np.array([d['t'] for d in data])
                early_mask = (t_arr >= 0.02) & (t_arr <= 0.15)
                early_lin = early_mask & (np.abs(kappa_arr) > 0.005) & (np.abs(kappa_arr) < 0.25)
                early_kappa = kappa_arr[early_lin]
                early_Fx = F_x[early_lin]

                if len(early_kappa) > 5:
                    C_x_early = np.sum(early_Fx * early_kappa) / np.sum(early_kappa ** 2)
                    p = np.polyfit(early_kappa, early_Fx, 1)
                    ss_res = np.sum((early_Fx - C_x_early * early_kappa)**2)
                    ss_tot = np.sum((early_Fx - np.mean(early_Fx))**2)
                    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else 0

                    self.get_logger().info(
                        f"  Early braking window (0.02-0.15s): {len(early_kappa)} pts")
                    self.get_logger().info(f"  C_x (early, origin-fit) ≈ {C_x_early:.1f} N/slip, R²={r2:.3f}")
                    self.get_logger().info(f"  C_x (polyfit) = slope {p[0]:.1f}, intercept {p[1]:.1f} N")

            # Full linear region analysis
            linear_mask = (np.abs(kappa_arr) > 0.005) & (np.abs(kappa_arr) < 0.15)
            linear_kappa = kappa_arr[linear_mask]
            linear_Fx = F_x[linear_mask]

            if len(linear_kappa) > 5:
                C_x = np.sum(linear_Fx * linear_kappa) / np.sum(linear_kappa ** 2)
                self.get_logger().info(f"  Full linear region (|κ| in [0.005, 0.15]): {len(linear_kappa)} points")
                self.get_logger().info(f"  C_x ≈ {C_x:.1f} N/unit-slip")
                p = np.polyfit(linear_kappa, linear_Fx, 1)
                self.get_logger().info(f"  (Polyfit: slope={p[0]:.1f}, intercept={p[1]:.2f} N)")
            else:
                self.get_logger().warn(f"  Not enough points in linear region ({len(linear_kappa)})")
                C_x = 0

        # Best estimate: use early braking window if available, else combined
        best_Cx = 0

        # Try early braking first (most reliable)
        if self.decel_data:
            t_arr = np.array([d['t'] for d in self.decel_data])
            kappa_arr = np.array([d['kappa'] for d in self.decel_data])
            ax_arr = np.array([d['ax'] for d in self.decel_data])
            early_mask = (t_arr >= 0.02) & (t_arr <= 0.15)
            early_lin = early_mask & (np.abs(kappa_arr) > 0.005) & (np.abs(kappa_arr) < 0.25)
            ek = kappa_arr[early_lin]
            ef = self.mass * ax_arr[early_lin]
            if len(ek) > 5:
                best_Cx = float(np.sum(ef * ek) / np.sum(ek ** 2))

        # Fallback: combined estimate
        if best_Cx == 0:
            combined_kappa = []
            combined_Fx = []
            for data in [self.accel_data, self.decel_data]:
                for d in data:
                    if 0.005 < abs(d['kappa']) < 0.15:
                        combined_kappa.append(d['kappa'])
                        combined_Fx.append(self.mass * d['ax'])
            if len(combined_kappa) > 10:
                kk = np.array(combined_kappa)
                ff = np.array(combined_Fx)
                best_Cx = float(np.sum(ff * kk) / np.sum(kk ** 2))

        self.get_logger().info(f"\n--- Parameters for MPC ---")
        if best_Cx != 0:
            self.get_logger().info(f"  C_x: {abs(best_Cx):.1f} N/unit-slip (combined, pitch-corrected)")
            self.get_logger().info(f"  C_x per tire: ~{abs(best_Cx)/4:.1f} N/unit-slip")
        self.get_logger().info(f"\n  NOTE: This is the COMBINED longitudinal stiffness")
        self.get_logger().info(f"  (all 4 wheels, AWD). For per-tire: divide by ~4.")
        self.get_logger().info(f"  Pitch compensation applied using vertical accelerometer.")

        # Auto-save to vehicle_params.yaml
        from common import update_vehicle_params
        if best_Cx != 0:
            update_vehicle_params({
                'C_x': float(abs(best_Cx)),
            }, status='TESTED', logger=self.get_logger())
        self.get_logger().info("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description='F1/10th Longitudinal Tire Stiffness Test')
    parser.add_argument('--max-speed', type=float, default=2.5,
                        help='Max speed to command (m/s, default: 2.5)')
    parser.add_argument('--accel-time', type=float, default=5.0,
                        help='Acceleration phase max duration (s, default: 5.0)')
    parser.add_argument('--cruise-time', type=float, default=2.0,
                        help='Steady-state cruise duration before braking (s, default: 2.0)')
    parser.add_argument('--mass', type=float, default=3.314,
                        help='Vehicle mass in kg (default: 3.314)')
    parser.add_argument('--erpm-gain', type=float, default=DEFAULT_ERPM_GAIN,
                        help=f'ERPM gain (default: {DEFAULT_ERPM_GAIN})')
    parser.add_argument('--geofence', type=float, default=10.0,
                        help='Max distance from start before abort (m, default: 10.0)')
    parser.add_argument('--runs', type=int, default=5,
                        help='Number of complete test runs (default: 5)')
    args = parser.parse_args()

    rclpy.init()
    for run_idx in range(args.runs):
        if args.runs > 1:
            print(f"\n{'='*60}")
            print(f"RUN {run_idx + 1}/{args.runs}")
            print(f"{'='*60}\n")
        node = LongitudinalStiffnessNode(args)
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
