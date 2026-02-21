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

from common import TestNode, ImuVelocityEstimator, DEFAULT_ERPM_GAIN


class LongitudinalStiffnessNode(TestNode):

    def __init__(self, args):
        columns = [
            'odom_vx', 'imu_ax', 'imu_ay',
            'motor_rpm', 'motor_current',
            'v_imu', 'slip_ratio',
            'cmd_speed', 'phase'
        ]

        super().__init__(
            'longitudinal_stiffness_test',
            'longitudinal_stiffness',
            columns,
            max_speed=args.max_speed * 1.3,
            max_time=60.0
        )

        self.max_speed = args.max_speed
        self.accel_time = args.accel_time
        self.mass = args.mass
        self.erpm_gain = args.erpm_gain

        self.imu_vel = ImuVelocityEstimator()

        # Collected data per phase
        self.accel_data = []
        self.decel_data = []

    def run_test(self):
        """Execute the longitudinal stiffness test."""
        if not self.wait_for_sensors():
            return False

        self.get_logger().info("=" * 60)
        self.get_logger().info("LONGITUDINAL TIRE STIFFNESS TEST")
        self.get_logger().info(f"Max speed: {self.max_speed:.1f} m/s")
        self.get_logger().info(f"Vehicle mass: {self.mass:.3f} kg")
        self.get_logger().info("=" * 60)

        # ---- IMU Bias Calibration ----
        self.get_logger().info("\nCalibrating IMU bias (keep the car still)...")
        bias_samples = []
        cal_start = time.monotonic()
        while time.monotonic() - cal_start < 2.0:
            rclpy.spin_once(self, timeout_sec=0.005)
            bias_samples.append(self.imu_ax)

        imu_bias = self.imu_vel.calibrate_bias(bias_samples)
        self.get_logger().info(f"IMU ax bias: {imu_bias:.4f} m/s² ({len(bias_samples)} samples)")

        self.countdown(3)
        self.recorder.start()
        self.safety.start()
        self.test_running = True

        # ---- Phase 1: Acceleration ----
        self.get_logger().info("\n--- Phase 1: ACCELERATION ---")
        self.imu_vel.reset(initial_velocity=0.0)
        phase_start = time.monotonic()
        last_t = phase_start

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

            v_imu = self.imu_vel.update(self.imu_ax, dt)
            v_wheel = self.odom_vx  # ERPM-based

            # Slip ratio (driving: wheel faster than body for traction)
            v_max = max(abs(v_wheel), abs(v_imu), 0.1)
            kappa = (v_wheel - v_imu) / v_max

            self.accel_data.append({
                't': now - phase_start,
                'v_wheel': v_wheel,
                'v_imu': v_imu,
                'ax': self.imu_ax - imu_bias,
                'kappa': kappa,
                'current': self.motor_current,
            })

            self.recorder.record(
                odom_vx=v_wheel, imu_ax=self.imu_ax, imu_ay=self.imu_ay,
                motor_rpm=self.motor_rpm, motor_current=self.motor_current,
                v_imu=v_imu, slip_ratio=kappa,
                cmd_speed=self.max_speed, phase='acceleration'
            )

        if not self.test_running:
            self.recorder.save()
            return False

        # ---- Brief cruise to stabilize ----
        self.get_logger().info("Cruising at steady speed...")
        self.spin_for(1.0)

        # Anchor IMU to current odom speed (known-good regime)
        self.imu_vel.reset(initial_velocity=abs(self.odom_vx))

        # ---- Phase 2: Braking ----
        self.get_logger().info("\n--- Phase 2: BRAKING ---")
        phase_start = time.monotonic()
        last_t = phase_start

        while (abs(self.odom_vx) > 0.1 or self.imu_vel.velocity > 0.1) and \
              time.monotonic() - phase_start < 10.0:
            rclpy.spin_once(self, timeout_sec=0.005)

            now = time.monotonic()
            dt = now - last_t
            last_t = now

            self.send_command(0.0, 0.0)

            v_imu = self.imu_vel.update(self.imu_ax, dt)
            v_wheel = self.odom_vx

            v_max = max(abs(v_wheel), abs(v_imu), 0.1)
            kappa = (v_wheel - v_imu) / v_max

            self.decel_data.append({
                't': now - phase_start,
                'v_wheel': v_wheel,
                'v_imu': v_imu,
                'ax': self.imu_ax - imu_bias,
                'kappa': kappa,
                'current': self.motor_current,
            })

            self.recorder.record(
                odom_vx=v_wheel, imu_ax=self.imu_ax, imu_ay=self.imu_ay,
                motor_rpm=self.motor_rpm, motor_current=self.motor_current,
                v_imu=v_imu, slip_ratio=kappa,
                cmd_speed=0.0, phase='braking'
            )

        self.stop_car()
        time.sleep(0.5)
        self.stop_car()

        self.recorder.save()
        self.analyze()
        return True

    def analyze(self):
        """Analyze longitudinal stiffness results."""
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("ANALYSIS RESULTS")
        self.get_logger().info("=" * 60)

        for phase_name, data in [('ACCELERATION', self.accel_data),
                                  ('BRAKING', self.decel_data)]:
            if not data:
                continue

            kappa_arr = np.array([d['kappa'] for d in data])
            ax_arr = np.array([d['ax'] for d in data])
            vw_arr = np.array([d['v_wheel'] for d in data])
            vi_arr = np.array([d['v_imu'] for d in data])
            F_x = self.mass * ax_arr  # Longitudinal force

            self.get_logger().info(f"\n{phase_name}:")
            self.get_logger().info(f"  Samples: {len(data)}")
            self.get_logger().info(f"  Speed range (wheel): {np.min(vw_arr):.2f} - {np.max(vw_arr):.2f} m/s")
            self.get_logger().info(f"  Speed range (IMU):   {np.min(vi_arr):.2f} - {np.max(vi_arr):.2f} m/s")
            self.get_logger().info(f"  Slip ratio range: {np.min(kappa_arr):.4f} to {np.max(kappa_arr):.4f}")
            self.get_logger().info(f"  Max |F_x|: {np.max(np.abs(F_x)):.2f} N")

            # Find linear region: |kappa| < 0.1 and |kappa| > 0.005 (exclude near-zero)
            linear_mask = (np.abs(kappa_arr) > 0.005) & (np.abs(kappa_arr) < 0.15)
            linear_kappa = kappa_arr[linear_mask]
            linear_Fx = F_x[linear_mask]

            if len(linear_kappa) > 5:
                # Linear fit: F_x = C_x * kappa (force through origin)
                # C_x = sum(F_x * kappa) / sum(kappa^2)
                C_x = np.sum(linear_Fx * linear_kappa) / np.sum(linear_kappa ** 2)

                self.get_logger().info(f"  Linear region (|κ| in [0.005, 0.15]): {len(linear_kappa)} points")
                self.get_logger().info(f"  C_x ≈ {C_x:.1f} N/unit-slip")

                # Also compute via least-squares polyfit as a check
                p = np.polyfit(linear_kappa, linear_Fx, 1)
                self.get_logger().info(f"  (Polyfit: slope={p[0]:.1f}, intercept={p[1]:.2f} N)")
            else:
                self.get_logger().warn(f"  Not enough points in linear region ({len(linear_kappa)})")
                self.get_logger().warn(f"  Try different speed settings or check IMU calibration")
                C_x = 0

        # Combined estimate
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
            C_x_combined = np.sum(ff * kk) / np.sum(kk ** 2)

            self.get_logger().info(f"\nCOMBINED ESTIMATE:")
            self.get_logger().info(f"  C_x = {C_x_combined:.1f} N/unit-slip "
                                   f"({len(combined_kappa)} points)")
        else:
            C_x_combined = 0

        self.get_logger().info(f"\n--- Parameters for MPC ---")
        if C_x_combined > 0:
            self.get_logger().info(f"  C_x: {C_x_combined:.1f} N/unit-slip (combined)")
        self.get_logger().info(f"\n  NOTE: This is the COMBINED longitudinal stiffness")
        self.get_logger().info(f"  (all 4 wheels, AWD). For per-tire: divide by ~4.")
        self.get_logger().info(f"  Accuracy depends on IMU bias calibration quality.")
        self.get_logger().info(f"  The slip ratio uses IMU-integrated velocity as ground truth.")

        # Auto-save to vehicle_params.yaml
        from common import update_vehicle_params
        if C_x_combined > 0:
            update_vehicle_params({
                'C_x': float(C_x_combined),
            }, status='TESTED', logger=self.get_logger())
        self.get_logger().info("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description='F1/10th Longitudinal Tire Stiffness Test')
    parser.add_argument('--max-speed', type=float, default=3.0,
                        help='Max speed to command (m/s, default: 3.0)')
    parser.add_argument('--accel-time', type=float, default=5.0,
                        help='Acceleration phase duration (s, default: 5.0)')
    parser.add_argument('--mass', type=float, default=3.314,
                        help='Vehicle mass in kg (default: 3.314)')
    parser.add_argument('--erpm-gain', type=float, default=DEFAULT_ERPM_GAIN,
                        help=f'ERPM gain (default: {DEFAULT_ERPM_GAIN})')
    parser.add_argument('--runs', type=int, default=1,
                        help='Number of complete test runs (default: 1)')
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
            print("\nCooling down for 5s before next run...")
            time.sleep(5)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
