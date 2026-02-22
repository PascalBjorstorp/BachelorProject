#!/usr/bin/env python3
"""
Motor Torque Mapping Test for F1/10th Car

Maps the relationship between motor current and wheel force by measuring
acceleration at different commanded speeds (and therefore different
motor current levels).

THEORY:
    Two methods for torque measurement:

    Method 1 — Current-based (motor model):
        T_motor = Kt * I_motor                    (motor shaft torque)
        F_wheel = T_motor * gear_ratio * eta / r_eff  (force at ground)
        where Kt ≈ 0.00273 Nm/A for the Traxxas 3351R (Kv=3500)

    Method 2 — Acceleration-based (direct measurement):
        F_net = m * a_x_imu                       (net force at ground)
        T_wheel_net = F_net * r_eff               (net wheel torque)
        This includes all losses (drivetrain friction, rolling resistance).

    By plotting motor current vs net force, we get the effective
    current-to-force map including all drivetrain losses. The slope gives
    an effective Kt_eff = Kt * gear_ratio * eta / r_eff.

PROCEDURE:
    1. Calibrate IMU bias while stationary
    2. Accelerate at multiple speed targets → record (current, a_x) at each
    3. For each speed level, compute average F = m*a_x and average I
    4. Fit the I → F relationship
    5. Also measure max braking torque (regen current)

NOTE: ackermann_to_vesc has a slow-start limiter when v < 1 m/s. The
initial acceleration phase may not reflect true max torque.

Usage:
    python3 test_motor_torque.py [--speeds 1.0,2.0,3.0,4.0]
"""

import argparse
import time

import numpy as np
import rclpy

import sys as _sys, os as _os  # noqa: E402
_sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..'))
from common import TestNode, ImuVelocityEstimator


class MotorTorqueNode(TestNode):

    def __init__(self, args):
        columns = [
            'odom_vx', 'imu_ax', 'imu_ay',
            'motor_rpm', 'motor_current', 'input_current',
            'battery_voltage',
            'cmd_speed', 'phase'
        ]

        super().__init__(
            'motor_torque_test',
            'motor_torque',
            columns,
            max_speed=max(args.speeds) * 1.3,
            max_time=len(args.speeds) * 15 + 60.0
        )

        self.speeds = args.speeds
        self.accel_time = args.accel_time
        self.mass = args.mass
        self.Kt = args.Kt
        self.r_tire = args.r_tire

        self.imu_vel = ImuVelocityEstimator()

        # Results
        self.accel_results = []
        self.brake_results = []
        self.max_drive_current = 0.0
        self.max_brake_current = 0.0

    def run_test(self):
        """Execute the motor torque mapping test."""
        if not self.wait_for_sensors():
            return False

        self.get_logger().info("=" * 60)
        self.get_logger().info("MOTOR TORQUE MAPPING TEST")
        self.get_logger().info(f"Speed targets: {self.speeds}")
        self.get_logger().info(f"Vehicle mass: {self.mass:.3f} kg")
        self.get_logger().info(f"Motor Kt: {self.Kt:.5f} Nm/A")
        self.get_logger().info("=" * 60)

        # ---- IMU Bias Calibration ----
        self.calibrate_imu_bias(duration=2.0)
        imu_bias = self.imu_vel.calibrate_bias([self.imu_bias_ax])
        self.get_logger().info(f"Using IMU ax bias: {imu_bias:.4f} m/s²")

        self.countdown(3)
        self.recorder.start()
        self.safety.start()
        self.test_running = True

        for speed_target in self.speeds:
            if not self.test_running:
                break

            self.get_logger().info(f"\n--- Speed target: {speed_target:.1f} m/s ---")

            # ---- Acceleration phase ----
            self.get_logger().info("Accelerating...")
            ax_samples = []
            current_samples = []
            rpm_samples = []
            vx_samples = []

            phase_start = time.monotonic()
            while time.monotonic() - phase_start < self.accel_time:
                rclpy.spin_once(self, timeout_sec=0.005)
                if not self.safety.check():
                    self.get_logger().error(f"Safety abort: {self.safety.abort_reason}")
                    self.stop_car()
                    self.test_running = False
                    break

                self.send_command(speed_target, 0.0)

                ax = self.imu_ax - imu_bias
                ax_samples.append(ax)
                current_samples.append(self.motor_current)
                rpm_samples.append(self.motor_rpm)
                vx_samples.append(self.odom_vx)

                self.recorder.record(
                    odom_vx=self.odom_vx, imu_ax=ax,
                    imu_ay=self.imu_ay, motor_rpm=self.motor_rpm,
                    motor_current=self.motor_current,
                    input_current=self.input_current,
                    battery_voltage=self.battery_voltage,
                    cmd_speed=speed_target, phase='acceleration'
                )

            if not self.test_running:
                break

            # Analyze this acceleration run
            if len(ax_samples) > 10:
                # Use samples where speed was actively increasing
                # (filter out the cruising portion at the end)
                vx_arr = np.array(vx_samples)
                ax_arr = np.array(ax_samples)
                current_arr = np.array(current_samples)

                # Only use samples where a_x > 0.3 (car is actually accelerating)
                accel_mask = ax_arr > 0.3
                if np.any(accel_mask):
                    avg_ax = np.mean(ax_arr[accel_mask])
                    std_ax = np.std(ax_arr[accel_mask])
                    avg_current = np.mean(current_arr[accel_mask])
                    std_current = np.std(current_arr[accel_mask])
                    avg_vx = np.mean(vx_arr[accel_mask])
                    peak_current = np.max(current_arr)
                    F_net = self.mass * avg_ax
                    F_net_std = self.mass * std_ax
                    n_accel = int(np.sum(accel_mask))

                    result = {
                        'speed_target': speed_target,
                        'avg_speed': avg_vx,
                        'avg_ax': avg_ax,
                        'ax_std': std_ax,
                        'avg_current': avg_current,
                        'current_std': std_current,
                        'peak_current': peak_current,
                        'F_net': F_net,
                        'F_net_std': F_net_std,
                        'n_samples': n_accel,
                    }
                    self.accel_results.append(result)
                    self.max_drive_current = max(self.max_drive_current, peak_current)

                    self.get_logger().info(
                        f"  avg a_x={avg_ax:.2f} m/s², "
                        f"avg I={avg_current:.1f}A, "
                        f"peak I={peak_current:.1f}A, "
                        f"F_net={F_net:.2f}N")

            # ---- Brief cruise ----
            self.spin_for(0.5)

            # ---- Braking phase ----
            self.get_logger().info("Braking...")
            brake_ax = []
            brake_current = []
            brake_start = time.monotonic()

            while abs(self.odom_vx) > 0.1 and time.monotonic() - brake_start < 5.0:
                rclpy.spin_once(self, timeout_sec=0.005)
                self.send_command(0.0, 0.0)

                ax = self.imu_ax - imu_bias
                brake_ax.append(ax)
                brake_current.append(self.motor_current)

                self.recorder.record(
                    odom_vx=self.odom_vx, imu_ax=ax,
                    imu_ay=self.imu_ay, motor_rpm=self.motor_rpm,
                    motor_current=self.motor_current,
                    input_current=self.input_current,
                    battery_voltage=self.battery_voltage,
                    cmd_speed=0.0, phase='braking'
                )

            if brake_ax:
                min_current = np.min(brake_current)
                avg_brake_ax = np.mean(brake_ax)
                avg_brake_current = np.mean(brake_current)
                self.max_brake_current = min(self.max_brake_current, min_current)

                self.brake_results.append({
                    'from_speed': speed_target,
                    'avg_ax': avg_brake_ax,
                    'avg_current': avg_brake_current,
                    'peak_regen_current': min_current,
                    'F_brake': self.mass * abs(avg_brake_ax),
                })

                self.get_logger().info(
                    f"  avg brake a_x={avg_brake_ax:.2f} m/s², "
                    f"avg I={avg_brake_current:.1f}A, "
                    f"peak regen I={min_current:.1f}A")

            # Wait for full stop
            self.spin_for(1.0)

        self.stop_car()
        time.sleep(0.5)
        self.stop_car()

        self.recorder.save()
        self.analyze()
        return True

    def analyze(self):
        """Analyze motor torque results."""
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("ANALYSIS RESULTS")
        self.get_logger().info("=" * 60)

        # ---- Acceleration torque map ----
        if self.accel_results:
            self.get_logger().info(f"\n1. ACCELERATION TORQUE MAP:")
            self.get_logger().info(
                f"  {'v_cmd':>6} {'v_avg':>6} {'a_x':>6} {'I_avg':>6} "
                f"{'I_peak':>7} {'F_net':>6} {'T_motor':>8}")

            currents = []
            forces = []

            for r in self.accel_results:
                T_motor = self.Kt * r['avg_current']
                self.get_logger().info(
                    f"  {r['speed_target']:6.1f} {r['avg_speed']:6.2f} "
                    f"{r['avg_ax']:6.2f} {r['avg_current']:6.1f} "
                    f"{r['peak_current']:7.1f} {r['F_net']:6.2f} {T_motor:8.4f}")
                currents.append(r['avg_current'])
                forces.append(r['F_net'])

            # Fit I → F_net relationship
            if len(currents) > 1:
                currents = np.array(currents)
                forces = np.array(forces)
                p = np.polyfit(currents, forces, 1)
                Kt_eff = p[0]
                F_loss = p[1]  # Force at zero current (rolling resistance + drag)

                self.get_logger().info(f"\n   Linear fit: F_net = {Kt_eff:.4f} * I + ({F_loss:.2f})")
                self.get_logger().info(f"   Effective Kt_eff = {Kt_eff:.4f} N/A")
                self.get_logger().info(f"   (This is Kt × gear_ratio × efficiency / r_tire)")
                self.get_logger().info(f"   Rolling resistance + losses ≈ {abs(F_loss):.2f} N")

                if self.r_tire > 0:
                    T_wheel_per_A = Kt_eff * self.r_tire
                    self.get_logger().info(f"   Wheel torque per amp: {T_wheel_per_A:.5f} Nm/A")
            else:
                Kt_eff = 0

            # Max drive force
            max_F = max(r['F_net'] for r in self.accel_results)
            max_I = max(r['peak_current'] for r in self.accel_results)
            self.get_logger().info(f"\n   Max drive force: {max_F:.2f} N "
                                   f"(at peak I = {max_I:.1f}A)")
            if self.r_tire > 0:
                self.get_logger().info(f"   Max drive torque at wheels: "
                                       f"{max_F * self.r_tire:.4f} Nm")

        # ---- Braking torque ----
        if self.brake_results:
            self.get_logger().info(f"\n2. BRAKING TORQUE:")
            for r in self.brake_results:
                self.get_logger().info(
                    f"  From {r['from_speed']:.1f} m/s: "
                    f"avg a_x={r['avg_ax']:.2f} m/s², "
                    f"F_brake={r['F_brake']:.2f} N, "
                    f"regen I={r['peak_regen_current']:.1f}A")

            max_brake_F = max(r['F_brake'] for r in self.brake_results)
            self.get_logger().info(f"\n   Max braking force: {max_brake_F:.2f} N")
            if self.r_tire > 0:
                self.get_logger().info(f"   Max braking torque at wheels: "
                                       f"{max_brake_F * self.r_tire:.4f} Nm")

        # ---- Summary ----
        self.get_logger().info(f"\n--- Parameters for MPC ---")
        if self.accel_results:
            max_drive_F = max(r['F_net'] for r in self.accel_results)
            self.get_logger().info(f"  Max drive force: {max_drive_F:.2f} N")
            if self.r_tire > 0:
                T_drive = max_drive_F * self.r_tire
                T_brake = max(r['F_brake'] for r in self.brake_results) * self.r_tire \
                    if self.brake_results else 0
                self.get_logger().info(f"  Max drive torque (at wheels): {T_drive:.4f} Nm")
                self.get_logger().info(f"  Max brake torque (at wheels): {T_brake:.4f} Nm")
        self.get_logger().info(f"  Max drive current: {self.max_drive_current:.1f} A")
        self.get_logger().info(f"  Max regen current: {self.max_brake_current:.1f} A")
        self.get_logger().info(f"\n  NOTE: These are NET forces at the ground (F=ma).")
        self.get_logger().info(f"  They include all drivetrain losses and rolling resistance.")
        self.get_logger().info(f"  For motor-level torque, multiply by r_tire/gear_ratio.")

        # Save summary CSV with per-speed-point results
        import csv as csv_mod
        summary_path = self.recorder.filename.replace('.csv', '_summary.csv')
        with open(summary_path, 'w', newline='') as f:
            writer = csv_mod.DictWriter(f, fieldnames=[
                'phase', 'speed_target', 'avg_speed', 'avg_ax', 'ax_std',
                'avg_current', 'current_std', 'peak_current',
                'F_net', 'F_net_std', 'T_motor_Kt', 'n_samples'])
            writer.writeheader()
            for r in self.accel_results:
                writer.writerow({
                    'phase': 'acceleration',
                    'speed_target': r['speed_target'],
                    'avg_speed': r['avg_speed'],
                    'avg_ax': r['avg_ax'],
                    'ax_std': r.get('ax_std', ''),
                    'avg_current': r['avg_current'],
                    'current_std': r.get('current_std', ''),
                    'peak_current': r['peak_current'],
                    'F_net': r['F_net'],
                    'F_net_std': r.get('F_net_std', ''),
                    'T_motor_Kt': self.Kt * r['avg_current'],
                    'n_samples': r.get('n_samples', ''),
                })
            for r in self.brake_results:
                writer.writerow({
                    'phase': 'braking',
                    'speed_target': r['from_speed'],
                    'avg_speed': '',
                    'avg_ax': r['avg_ax'],
                    'ax_std': '',
                    'avg_current': r['avg_current'],
                    'current_std': '',
                    'peak_current': r['peak_regen_current'],
                    'F_net': r['F_brake'],
                    'F_net_std': '',
                    'T_motor_Kt': self.Kt * r['avg_current'],
                    'n_samples': '',
                })
        self.get_logger().info(f"Summary saved to {summary_path}")

        # Auto-save to vehicle_params.yaml
        from common import update_vehicle_params
        params = {}
        if self.accel_results:
            max_drive_F = max(r['F_net'] for r in self.accel_results)
            if self.r_tire > 0:
                params['max_motor_torque'] = float(max_drive_F * self.r_tire)
            params['max_motor_current'] = float(self.max_drive_current)
        if self.brake_results:
            max_brake_F = max(r['F_brake'] for r in self.brake_results)
            if self.r_tire > 0:
                params['max_brake_torque'] = float(max_brake_F * self.r_tire)
            params['max_brake_current'] = float(self.max_brake_current)
        if params:
            update_vehicle_params(params, status='TESTED', logger=self.get_logger())
        self.get_logger().info("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description='F1/10th Motor Torque Mapping Test')
    parser.add_argument('--speeds', type=str, default='1.5,2.0,2.5,3.0',
                        help='Comma-separated speed targets (m/s)')
    parser.add_argument('--accel-time', type=float, default=4.0,
                        help='Acceleration time per speed (s, default: 4.0)')
    parser.add_argument('--mass', type=float, default=3.314,
                        help='Vehicle mass in kg (default: 3.314)')
    parser.add_argument('--Kt', type=float, default=0.00273,
                        help='Motor torque constant Nm/A (default: 0.00273 for 3351R)')
    parser.add_argument('--r-tire', type=float, default=0.05,
                        help='Effective tire radius in m (default: 0.05, UPDATE THIS)')
    parser.add_argument('--runs', type=int, default=5,
                        help='Number of complete test runs (default: 5)')
    args = parser.parse_args()

    # Parse speeds
    args.speeds = [float(s) for s in args.speeds.split(',')]

    rclpy.init()
    for run_idx in range(args.runs):
        if args.runs > 1:
            print(f"\n{'='*60}")
            print(f"RUN {run_idx + 1}/{args.runs}")
            print(f"{'='*60}\n")
        node = MotorTorqueNode(args)
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
