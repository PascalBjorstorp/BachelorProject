#!/usr/bin/env python3
"""
Turn-In Transient Test for F1/10th Car

Purpose:
    Collect the transient near-limit corner-entry data that steady-state circle
    tests do not capture. This is the missing dataset for fitting simulator RMSE
    in the high-error turn regime.

Protocol:
    1. Drive straight at constant speed to settle.
    2. Apply a steering step and hold it.
    3. Optionally command a speed drop during the hold to emulate
       braking / speed bleed while cornering.
    4. Recover back to zero steering.

The recorded data is intended to fit:
    - transient yaw-rate build-up
    - lateral acceleration build-up
    - LiDAR-estimated sideslip / lateral velocity
    - understeer / washout at higher speeds
    - combined longitudinal + lateral behavior (when speed_drop > 0)

Recommended first runs:
    python3 tests/test_turn_in_transient.py \\
        --speeds 2.0,2.5,3.0,3.5 \\
        --steering 0.30 \\
        --directions both \\
        --repeats 2 \\
        --turn-time 2.5

Optional combined-slip run:
    python3 tests/test_turn_in_transient.py \\
        --speeds 2.5,3.0,3.5 \\
        --steering 0.30 \\
        --speed-drop 0.6 \\
        --drop-after 0.35
"""

import argparse
import math
import time

import numpy as np
import rclpy

import sys as _sys, os as _os  # noqa: E402
_sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..'))
from common import TestNode, radius_from_steering_angle, DEFAULT_WHEELBASE


def _parse_csv_floats(text: str) -> list[float]:
    values = []
    for part in str(text).split(','):
        part = part.strip()
        if not part:
            continue
        values.append(float(part))
    return values


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _simulate_transient_footprint(
    speed: float,
    steer: float,
    settle_time: float,
    turn_time: float,
    recover_time: float,
    wheelbase: float,
    dt: float = 0.01,
) -> dict:
    x = 0.0
    y = 0.0
    yaw = 0.0
    xs = [x]
    ys = [y]

    for duration, delta in (
        (settle_time, 0.0),
        (turn_time, steer),
        (recover_time, 0.0),
    ):
        steps = max(1, int(math.ceil(duration / dt)))
        step_dt = duration / steps
        yaw_rate = speed * math.tan(delta) / wheelbase if abs(delta) > 1e-9 else 0.0
        for _ in range(steps):
            x += speed * math.cos(yaw) * step_dt
            y += speed * math.sin(yaw) * step_dt
            yaw += yaw_rate * step_dt
            xs.append(x)
            ys.append(y)

    xs = np.array(xs, dtype=float)
    ys = np.array(ys, dtype=float)
    radius = np.sqrt(xs * xs + ys * ys)
    return {
        'span_x': float(np.max(xs) - np.min(xs)),
        'span_y': float(np.max(ys) - np.min(ys)),
        'max_radius': float(np.max(radius)),
    }


class TurnInTransientNode(TestNode):

    def __init__(self, args):
        columns = [
            'run_id',
            'repeat_idx', 'direction_sign',
            'phase', 'phase_elapsed_s',
            'speed_setpoint', 'steer_step_setpoint', 'steering_trim', 'steer_setpoint',
            'cmd_speed', 'cmd_steering',
            'odom_x', 'odom_y', 'odom_yaw',
            'odom_vx', 'odom_vy', 'odom_omega',
            'odom_vx_corr', 'odom_vy_corr', 'odom_omega_corr',
            'imu_ax_raw', 'imu_ay_raw', 'imu_az_raw',
            'imu_gx_raw', 'imu_gy_raw', 'imu_gz_raw',
            'imu_ax_corr', 'imu_ay_corr', 'imu_az_corr', 'imu_gz_corr',
            'lidar_vx_raw',
            'lidar_vx', 'lidar_vy', 'lidar_omega',
            'lidar_vx_corr', 'lidar_vy_corr', 'lidar_omega_corr',
            'beta_lidar_rad',
            'motor_rpm', 'motor_current', 'input_current',
            'battery_voltage', 'temp_fet', 'temp_motor'
        ]

        max_speed = max(args.speeds) + max(0.5, args.speed_drop) + 0.5
        max_time = 0.0

        self.room_length = float(args.room_length)
        self.room_width = float(args.room_width)
        self.room_margin = float(args.room_margin)

        worst_span_x = 0.0
        worst_span_y = 0.0
        worst_radius = 0.0
        for speed in args.speeds:
            for steer in args.steering:
                footprint = _simulate_transient_footprint(
                    speed=speed,
                    steer=abs(steer),
                    settle_time=args.settle_time,
                    turn_time=args.turn_time,
                    recover_time=args.recover_time,
                    wheelbase=args.wheelbase,
                )
                worst_span_x = max(worst_span_x, footprint['span_x'])
                worst_span_y = max(worst_span_y, footprint['span_y'])
                worst_radius = max(worst_radius, footprint['max_radius'])

        fits_room = (
            (worst_span_x <= self.room_length - 2.0 * self.room_margin and
             worst_span_y <= self.room_width - 2.0 * self.room_margin) or
            (worst_span_y <= self.room_length - 2.0 * self.room_margin and
             worst_span_x <= self.room_width - 2.0 * self.room_margin)
        )
        if not fits_room:
            raise ValueError(
                "Configured transient footprint does not fit the declared room. "
                f"Estimated span {worst_span_x:.2f}m x {worst_span_y:.2f}m, "
                f"room usable area {(self.room_length - 2.0 * self.room_margin):.2f}m x "
                f"{(self.room_width - 2.0 * self.room_margin):.2f}m."
            )

        geofence = args.geofence
        if geofence <= 0.0:
            geofence = worst_radius + self.room_margin
        geofence = min(geofence, min(self.room_length, self.room_width) * 0.5 - 0.1)

        super().__init__(
            'turn_in_transient_test',
            'turn_in_transient',
            columns,
            max_speed=max_speed,
            max_time=max_time,
            max_distance=geofence
        )

        self.speeds = list(args.speeds)
        self.steering_list = list(args.steering)
        self.directions = list(args.directions)
        self.repeats = int(args.repeats)
        self.settle_time = float(args.settle_time)
        self.turn_time = float(args.turn_time)
        self.recover_time = float(args.recover_time)
        self.speed_drop = float(args.speed_drop)
        self.drop_after = float(args.drop_after)
        self.geofence_input = float(args.geofence)
        self.wheelbase = float(args.wheelbase)
        self.straight_check_speed = float(args.straight_check_speed)
        self.straight_check_time = float(args.straight_check_time)
        self.straight_check_max_passes = int(args.straight_check_max_passes)
        self.straight_trim_limit = float(args.straight_trim_limit)
        self.straight_vy_limit = float(args.straight_vy_limit)
        self.straight_gz_limit = float(args.straight_gz_limit)
        self.straight_beta_limit = float(args.straight_beta_limit)
        self.straight_yaw_delta_limit = float(args.straight_yaw_delta_limit)
        self.pre_run_pause = float(args.pre_run_pause)
        self.between_run_pause = float(args.between_run_pause)
        self.results = []
        self.steering_trim = 0.0
        self.odom_bias_vx = 0.0
        self.odom_bias_vy = 0.0
        self.odom_bias_omega = 0.0
        self.lidar_bias_vx = 0.0
        self.lidar_bias_vy = 0.0
        self.lidar_bias_omega = 0.0
        self.footprint_span_x = worst_span_x
        self.footprint_span_y = worst_span_y
        self.footprint_max_radius = worst_radius
        self.straightness_reports = []
        self.lidar_zero_reliable = True
        self.straightness_degraded = False

    def _corrected_lidar(self) -> tuple[float, float, float]:
        return (
            self.lidar_vx - self.lidar_bias_vx,
            self.lidar_vy - self.lidar_bias_vy,
            self.lidar_omega - self.lidar_bias_omega,
        )

    def _record_now(
        self,
        *,
        run_id: int,
        repeat_idx: int,
        direction_sign: float,
        phase: str,
        phase_elapsed_s: float,
        speed_setpoint: float,
        steer_step_setpoint: float,
        steer_setpoint: float,
    ):
        imu_ax_corr = self.imu_ax - self.imu_bias_ax
        imu_ay_corr = self.imu_ay - self.imu_bias_ay
        imu_az_corr = self.imu_az - self.imu_bias_az
        imu_gz_corr = self.imu_gz - self.imu_bias_gz
        lidar_vx_corr, lidar_vy_corr, lidar_omega_corr = self._corrected_lidar()
        odom_vx_corr = self.odom_vx - self.odom_bias_vx
        odom_vy_corr = self.odom_vy - self.odom_bias_vy
        odom_omega_corr = self.odom_omega - self.odom_bias_omega
        beta_lidar = math.atan2(lidar_vy_corr, max(abs(lidar_vx_corr), 1e-6))
        self.recorder.record(
            run_id=run_id,
            repeat_idx=repeat_idx,
            direction_sign=direction_sign,
            phase=phase,
            phase_elapsed_s=phase_elapsed_s,
            speed_setpoint=speed_setpoint,
            steer_step_setpoint=steer_step_setpoint,
            steering_trim=self.steering_trim,
            steer_setpoint=steer_setpoint,
            cmd_speed=self.cmd_speed,
            cmd_steering=self.cmd_steering,
            odom_x=self.odom_x, odom_y=self.odom_y, odom_yaw=self.odom_yaw,
            odom_vx=self.odom_vx, odom_vy=self.odom_vy, odom_omega=self.odom_omega,
            odom_vx_corr=odom_vx_corr, odom_vy_corr=odom_vy_corr, odom_omega_corr=odom_omega_corr,
            imu_ax_raw=self.imu_ax, imu_ay_raw=self.imu_ay, imu_az_raw=self.imu_az,
            imu_gx_raw=self.imu_gx, imu_gy_raw=self.imu_gy, imu_gz_raw=self.imu_gz,
            imu_ax_corr=imu_ax_corr, imu_ay_corr=imu_ay_corr, imu_az_corr=imu_az_corr, imu_gz_corr=imu_gz_corr,
            lidar_vx_raw=self.lidar_vx_raw,
            lidar_vx=self.lidar_vx, lidar_vy=self.lidar_vy, lidar_omega=self.lidar_omega,
            lidar_vx_corr=lidar_vx_corr, lidar_vy_corr=lidar_vy_corr, lidar_omega_corr=lidar_omega_corr,
            beta_lidar_rad=beta_lidar,
            motor_rpm=self.motor_rpm, motor_current=self.motor_current, input_current=self.input_current,
            battery_voltage=self.battery_voltage, temp_fet=self.temp_fet, temp_motor=self.temp_motor
        )

    def _calibrate_stationary_biases(self, duration: float = 2.0, settle_time: float = 0.5):
        self.get_logger().info(
            f"Calibrating stationary sensor biases for {duration:.1f}s (car must be still)...")
        end_settle = time.monotonic() + settle_time
        while time.monotonic() < end_settle:
            self.send_command(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)

        imu_ax_samples = []
        imu_ay_samples = []
        imu_az_samples = []
        imu_gz_samples = []
        odom_vx_samples = []
        odom_vy_samples = []
        odom_omega_samples = []
        lidar_vx_samples = []
        lidar_vy_samples = []
        lidar_omega_samples = []

        start = time.monotonic()
        while time.monotonic() - start < duration:
            self.send_command(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.005)
            if not (self.imu_received and self.odom_received and self.lidar_received):
                continue
            if (
                abs(self.odom_vx) > 0.10 or
                abs(self.odom_vy) > 0.10 or
                abs(self.imu_gz) > 0.12 or
                abs(self.motor_current) > 4.0
            ):
                continue
            imu_ax_samples.append(self.imu_ax)
            imu_ay_samples.append(self.imu_ay)
            imu_az_samples.append(self.imu_az)
            imu_gz_samples.append(self.imu_gz)
            odom_vx_samples.append(self.odom_vx)
            odom_vy_samples.append(self.odom_vy)
            odom_omega_samples.append(self.odom_omega)
            lidar_vx_samples.append(self.lidar_vx)
            lidar_vy_samples.append(self.lidar_vy)
            lidar_omega_samples.append(self.lidar_omega)
            self._record_now(
                run_id=0,
                repeat_idx=0,
                direction_sign=0.0,
                phase='stationary_bias',
                phase_elapsed_s=time.monotonic() - start,
                speed_setpoint=0.0,
                steer_step_setpoint=0.0,
                steer_setpoint=0.0,
            )

        if len(imu_ax_samples) < 20:
            raise RuntimeError("Not enough stationary samples to calibrate sensor biases")

        self.imu_bias_ax = float(np.median(imu_ax_samples))
        self.imu_bias_ay = float(np.median(imu_ay_samples))
        self.imu_bias_az = float(np.median(imu_az_samples))
        self.imu_bias_gz = float(np.median(imu_gz_samples))
        self.odom_bias_vx = float(np.median(odom_vx_samples))
        self.odom_bias_vy = float(np.median(odom_vy_samples))
        self.odom_bias_omega = float(np.median(odom_omega_samples))
        self.lidar_bias_vx = float(np.median(lidar_vx_samples))
        self.lidar_bias_vy = float(np.median(lidar_vy_samples))
        self.lidar_bias_omega = float(np.median(lidar_omega_samples))

        lidar_vx_std = float(np.std(lidar_vx_samples))
        lidar_vy_std = float(np.std(lidar_vy_samples))
        lidar_omega_std = float(np.std(lidar_omega_samples))
        self.lidar_zero_reliable = (
            abs(self.lidar_bias_vx) <= 0.10 and
            abs(self.lidar_bias_vy) <= 0.10 and
            abs(self.lidar_bias_omega) <= 0.10 and
            lidar_vx_std <= 0.08 and
            lidar_vy_std <= 0.08 and
            lidar_omega_std <= 0.08
        )

        self.get_logger().info(
            f"IMU bias: ax={self.imu_bias_ax:+.4f}, ay={self.imu_bias_ay:+.4f}, "
            f"az={self.imu_bias_az:+.4f}, gz={self.imu_bias_gz:+.4f}")
        self.get_logger().info(
            f"Odom zero: vx={self.odom_bias_vx:+.4f}, vy={self.odom_bias_vy:+.4f}, "
            f"omega={self.odom_bias_omega:+.4f}")
        self.get_logger().info(
            f"LiDAR zero: vx={self.lidar_bias_vx:+.4f}, vy={self.lidar_bias_vy:+.4f}, "
            f"omega={self.lidar_bias_omega:+.4f}")
        if not self.lidar_zero_reliable:
            self.get_logger().warn(
                "LiDAR stationary zero looks unreliable; straightness trim will rely on yaw response "
                "more than LiDAR lateral velocity for this run."
            )

    def _run_phase(
        self,
        *,
        duration: float,
        run_id: int,
        repeat_idx: int,
        direction_sign: float,
        phase: str,
        speed_cmd_fn,
        steer_step_fn,
        collect_metrics: bool = False,
    ):
        samples = []
        start = time.monotonic()
        while time.monotonic() - start < duration:
            rclpy.spin_once(self, timeout_sec=0.005)
            if not self.safety.check():
                self.get_logger().error(f"Safety abort: {self.safety.abort_reason}")
                self.stop_car()
                return False, samples

            elapsed = time.monotonic() - start
            speed_cmd = float(speed_cmd_fn(elapsed))
            steer_step = float(steer_step_fn(elapsed))
            steer_cmd = float(np.clip(
                self.steering_trim + steer_step,
                -self.straight_trim_limit - max(abs(v) for v in self.steering_list),
                self.straight_trim_limit + max(abs(v) for v in self.steering_list),
            ))
            self.send_command(speed_cmd, steer_cmd)
            self._record_now(
                run_id=run_id,
                repeat_idx=repeat_idx,
                direction_sign=direction_sign,
                phase=phase,
                phase_elapsed_s=elapsed,
                speed_setpoint=speed_cmd,
                steer_step_setpoint=steer_step,
                steer_setpoint=steer_cmd,
            )
            if collect_metrics:
                lidar_vx_corr, lidar_vy_corr, _ = self._corrected_lidar()
                samples.append({
                    'elapsed': elapsed,
                    'odom_yaw': self.odom_yaw,
                    'odom_vx_corr': self.odom_vx - self.odom_bias_vx,
                    'imu_ay_corr': self.imu_ay - self.imu_bias_ay,
                    'imu_gz_corr': self.imu_gz - self.imu_bias_gz,
                    'lidar_vy_corr': lidar_vy_corr,
                    'beta_lidar': math.atan2(lidar_vy_corr, max(abs(lidar_vx_corr), 1e-6)),
                })
        return True, samples

    def _run_straightness_check(self) -> bool:
        self.get_logger().info("Checking straight-line trim from measured velocities...")
        check_speed = min(self.straight_check_speed, max(self.speeds))
        hard_gz_abort = math.radians(20.0)
        best_report = None
        for pass_idx in range(self.straight_check_max_passes):
            ok, samples = self._run_phase(
                duration=self.straight_check_time,
                run_id=0,
                repeat_idx=0,
                direction_sign=0.0,
                phase=f'straight_check_{pass_idx + 1}',
                speed_cmd_fn=lambda _t, v=check_speed: v,
                steer_step_fn=lambda _t: 0.0,
                collect_metrics=True,
            )
            if not ok or not samples:
                return False

            mean_vx = float(np.mean([s['odom_vx_corr'] for s in samples]))
            mean_gz = float(np.mean([s['imu_gz_corr'] for s in samples]))
            mean_vy = float(np.mean([s['lidar_vy_corr'] for s in samples]))
            mean_beta = float(np.mean([s['beta_lidar'] for s in samples]))
            yaw_delta = _wrap_angle(samples[-1]['odom_yaw'] - samples[0]['odom_yaw'])

            correction = -math.atan2(self.wheelbase * mean_gz, max(abs(mean_vx), 0.5))
            yaw_ok = abs(mean_gz) <= self.straight_gz_limit and abs(yaw_delta) <= self.straight_yaw_delta_limit
            lidar_ok = (
                abs(mean_vy) <= self.straight_vy_limit and
                abs(mean_beta) <= self.straight_beta_limit
            ) if self.lidar_zero_reliable else True
            trim_ok = yaw_ok and lidar_ok

            report = {
                'pass_idx': pass_idx + 1,
                'mean_vx': mean_vx,
                'mean_gz': mean_gz,
                'mean_vy': mean_vy,
                'mean_beta': mean_beta,
                'yaw_delta': yaw_delta,
                'steering_trim': self.steering_trim,
                'yaw_ok': yaw_ok,
                'lidar_ok': lidar_ok,
                'trim_ok': trim_ok,
                'lidar_zero_reliable': self.lidar_zero_reliable,
            }
            best_report = report

            if trim_ok:
                self.straightness_reports.append(report)
                self.get_logger().info(
                    f"Straight pass {pass_idx + 1}: trim={self.steering_trim:+.4f} rad, "
                    f"mean_vx={mean_vx:.3f}, mean_vy={mean_vy:+.3f}, "
                    f"mean_gz={math.degrees(mean_gz):+.2f} deg/s, "
                    f"beta={math.degrees(mean_beta):+.2f} deg, yaw_delta={math.degrees(yaw_delta):+.2f} deg "
                    f"-> straight enough")
                return True

            new_trim = float(np.clip(
                self.steering_trim + correction,
                -self.straight_trim_limit,
                self.straight_trim_limit,
            ))
            trim_saturated = abs(new_trim - self.steering_trim) < 1e-4 and abs(correction) > 1e-4
            self.steering_trim = new_trim
            report['steering_trim'] = self.steering_trim

            self.straightness_reports.append(report)
            self.get_logger().info(
                f"Straight pass {pass_idx + 1}: trim={self.steering_trim:+.4f} rad, "
                f"mean_vx={mean_vx:.3f}, mean_vy={mean_vy:+.3f}, "
                f"mean_gz={math.degrees(mean_gz):+.2f} deg/s, "
                f"beta={math.degrees(mean_beta):+.2f} deg, yaw_delta={math.degrees(yaw_delta):+.2f} deg")

            self.stop_car()
            self.spin_for(0.4)
            if abs(mean_gz) > hard_gz_abort and trim_saturated:
                self.get_logger().error(
                    "Straight trim saturated while yaw error stayed very large; aborting."
                )
                return False

        self.straightness_degraded = True
        if best_report is not None:
            self.get_logger().warn(
                "Straight-line trim did not fully meet thresholds after iterative correction. "
                "Continuing with best available trim and marking the dataset as degraded."
            )
        return True

    def _run_single(self, run_id: int, repeat_idx: int, speed: float, steer: float, direction: float) -> bool:
        signed_steer = direction * steer
        dir_name = 'left' if direction > 0.0 else 'right'
        self.get_logger().info(
            f"Run {run_id}: v={speed:.2f} m/s, δ_step={math.degrees(signed_steer):.1f}°, "
            f"dir={dir_name}, trim={math.degrees(self.steering_trim):+.2f}°")

        ok, _ = self._run_phase(
            duration=self.settle_time,
            run_id=run_id,
            repeat_idx=repeat_idx,
            direction_sign=direction,
            phase='settle',
            speed_cmd_fn=lambda _t: speed,
            steer_step_fn=lambda _t: 0.0,
        )
        if not ok:
            return False

        turn_samples = []
        ok, turn_metrics = self._run_phase(
            duration=self.turn_time,
            run_id=run_id,
            repeat_idx=repeat_idx,
            direction_sign=direction,
            phase='turn',
            speed_cmd_fn=lambda t: max(0.0, speed - self.speed_drop) if self.speed_drop > 1e-6 and t >= self.drop_after else speed,
            steer_step_fn=lambda _t, s=signed_steer: s,
            collect_metrics=True,
        )
        if not ok:
            return False
        for sample in turn_metrics:
            turn_samples.append((
                sample['elapsed'],
                sample['odom_vx_corr'],
                0.0,
                sample['imu_ay_corr'],
                sample['imu_gz_corr'],
                0.0,
                sample['lidar_vy_corr'],
                sample['beta_lidar'],
            ))

        ok, _ = self._run_phase(
            duration=self.recover_time,
            run_id=run_id,
            repeat_idx=repeat_idx,
            direction_sign=direction,
            phase='recover',
            speed_cmd_fn=lambda _t: speed,
            steer_step_fn=lambda _t: 0.0,
        )
        if not ok:
            return False

        if turn_samples:
            arr = np.array(turn_samples, dtype=float)
            peak_beta = float(np.max(np.abs(arr[:, 7])))
            peak_ay = float(np.max(np.abs(arr[:, 3])))
            peak_gz = float(np.max(np.abs(arr[:, 4])))
            peak_vy = float(np.max(np.abs(arr[:, 6])))
            self.results.append({
                'run_id': run_id,
                'speed_cmd': speed,
                'steer_cmd': signed_steer,
                'peak_beta_deg': math.degrees(peak_beta),
                'peak_ay': peak_ay,
                'peak_gz_deg_s': math.degrees(peak_gz),
                'peak_lidar_vy': peak_vy,
            })
        return True

    def _pause_for_reposition(self, duration: float, label: str) -> bool:
        if duration <= 1e-6:
            return True
        self.get_logger().info(
            f"{label}: holding stopped for {duration:.1f}s so you can reposition the car.")
        self.stop_car()
        end_time = time.monotonic() + duration
        next_log = time.monotonic()
        while time.monotonic() < end_time:
            now = time.monotonic()
            if now >= next_log:
                remaining = end_time - now
                self.get_logger().info(f"  {label}: {remaining:.1f}s remaining")
                next_log = now + 1.0
            rclpy.spin_once(self, timeout_sec=0.05)
            self.send_command(0.0, 0.0)
            if not self.safety.check():
                self.get_logger().error(f"Safety abort: {self.safety.abort_reason}")
                self.stop_car()
                return False
        self.stop_car()
        return True

    def run_test(self):
        if not self.wait_for_sensors(require_vesc=True, require_lidar=True):
            return False

        self.get_logger().info("=" * 60)
        self.get_logger().info("TURN-IN TRANSIENT TEST")
        self.get_logger().info(f"Speeds: {', '.join(f'{v:.2f}' for v in self.speeds)} m/s")
        self.get_logger().info(f"Steering: {', '.join(f'{math.degrees(v):.1f}°' for v in self.steering_list)}")
        self.get_logger().info(f"Directions: {', '.join('left' if d > 0 else 'right' for d in self.directions)}")
        self.get_logger().info(
            f"Timings: settle={self.settle_time:.2f}s turn={self.turn_time:.2f}s recover={self.recover_time:.2f}s")
        if self.speed_drop > 1e-6:
            self.get_logger().info(
                f"Combined-slip mode: speed drop {self.speed_drop:.2f} m/s after {self.drop_after:.2f}s")
        self.get_logger().info(
            f"Room envelope: {self.room_length:.1f}m x {self.room_width:.1f}m "
            f"(margin {self.room_margin:.2f}m)")
        self.get_logger().info(
            f"Estimated worst-case transient span: {self.footprint_span_x:.2f}m x "
            f"{self.footprint_span_y:.2f}m, max radius from start {self.footprint_max_radius:.2f}m")
        self.get_logger().info(f"Geofence: {self.safety.max_distance:.2f} m")
        self.get_logger().info("=" * 60)

        self.recorder.start()
        self.safety.set_origin(self.odom_x, self.odom_y)
        self.safety.start()
        self.test_running = True
        self._calibrate_stationary_biases(duration=2.0)
        if not self._run_straightness_check():
            self.test_running = False
            self.stop_car()
            csv_path = self.recorder.save()
            self._write_metadata(csv_path)
            return False
        if not self._pause_for_reposition(self.pre_run_pause, "Pre-run pause"):
            self.test_running = False
            csv_path = self.recorder.save()
            self._write_metadata(csv_path)
            return False
        self.countdown(5)

        run_id = 0
        for repeat in range(self.repeats):
            for speed in self.speeds:
                for steer in self.steering_list:
                    for direction in self.directions:
                        if not self.test_running:
                            break
                        run_id += 1
                        ok = self._run_single(run_id, repeat + 1, speed, steer, direction)
                        if not ok:
                            self.test_running = False
                            break
                        if not self._pause_for_reposition(self.between_run_pause, f"Between run {run_id} and next run"):
                            self.test_running = False
                            break
                if not self.test_running:
                    break
            if not self.test_running:
                break

        self.stop_car()
        time.sleep(0.5)
        self.stop_car()
        csv_path = self.recorder.save()
        self._write_metadata(csv_path)
        self.analyze()
        return self.test_running

    def _write_metadata(self, csv_path: str):
        meta_path = f"{csv_path}.meta.txt"
        with open(meta_path, 'w', encoding='utf-8') as f:
            f.write("turn_in_transient metadata\n")
            f.write(f"speeds_mps={','.join(f'{v:.6f}' for v in self.speeds)}\n")
            f.write(f"steering_steps_rad={','.join(f'{v:.6f}' for v in self.steering_list)}\n")
            f.write(f"directions={','.join('left' if d > 0 else 'right' for d in self.directions)}\n")
            f.write(f"repeats={self.repeats}\n")
            f.write(f"settle_time_s={self.settle_time:.6f}\n")
            f.write(f"turn_time_s={self.turn_time:.6f}\n")
            f.write(f"recover_time_s={self.recover_time:.6f}\n")
            f.write(f"speed_drop_mps={self.speed_drop:.6f}\n")
            f.write(f"drop_after_s={self.drop_after:.6f}\n")
            f.write(f"wheelbase_m={self.wheelbase:.6f}\n")
            f.write(f"geofence_input_m={self.geofence_input:.6f}\n")
            f.write(f"room_length_m={self.room_length}\n")
            f.write(f"room_width_m={self.room_width}\n")
            f.write(f"room_margin_m={self.room_margin}\n")
            f.write(f"estimated_span_x_m={self.footprint_span_x:.6f}\n")
            f.write(f"estimated_span_y_m={self.footprint_span_y:.6f}\n")
            f.write(f"estimated_max_radius_m={self.footprint_max_radius:.6f}\n")
            f.write(f"geofence_m={self.safety.max_distance:.6f}\n")
            f.write(f"straight_check_speed_mps={self.straight_check_speed:.6f}\n")
            f.write(f"straight_check_time_s={self.straight_check_time:.6f}\n")
            f.write(f"straight_check_max_passes={self.straight_check_max_passes}\n")
            f.write(f"straight_trim_limit_rad={self.straight_trim_limit:.6f}\n")
            f.write(f"straight_vy_limit_mps={self.straight_vy_limit:.6f}\n")
            f.write(f"straight_gz_limit_radps={self.straight_gz_limit:.6f}\n")
            f.write(f"straight_beta_limit_rad={self.straight_beta_limit:.6f}\n")
            f.write(f"straight_yaw_delta_limit_rad={self.straight_yaw_delta_limit:.6f}\n")
            f.write(f"pre_run_pause_s={self.pre_run_pause:.6f}\n")
            f.write(f"between_run_pause_s={self.between_run_pause:.6f}\n")
            f.write(f"lidar_zero_reliable={int(self.lidar_zero_reliable)}\n")
            f.write(f"straightness_degraded={int(self.straightness_degraded)}\n")
            f.write(f"imu_bias_ax={self.imu_bias_ax:.9f}\n")
            f.write(f"imu_bias_ay={self.imu_bias_ay:.9f}\n")
            f.write(f"imu_bias_az={self.imu_bias_az:.9f}\n")
            f.write(f"imu_bias_gz={self.imu_bias_gz:.9f}\n")
            f.write(f"odom_bias_vx={self.odom_bias_vx:.9f}\n")
            f.write(f"odom_bias_vy={self.odom_bias_vy:.9f}\n")
            f.write(f"odom_bias_omega={self.odom_bias_omega:.9f}\n")
            f.write(f"lidar_bias_vx={self.lidar_bias_vx:.9f}\n")
            f.write(f"lidar_bias_vy={self.lidar_bias_vy:.9f}\n")
            f.write(f"lidar_bias_omega={self.lidar_bias_omega:.9f}\n")
            f.write(f"steering_trim_rad={self.steering_trim:.9f}\n")
            for report in self.straightness_reports:
                prefix = f"straight_pass_{report['pass_idx']}"
                f.write(f"{prefix}_mean_vx={report['mean_vx']:.9f}\n")
                f.write(f"{prefix}_mean_vy={report['mean_vy']:.9f}\n")
                f.write(f"{prefix}_mean_gz={report['mean_gz']:.9f}\n")
                f.write(f"{prefix}_mean_beta={report['mean_beta']:.9f}\n")
                f.write(f"{prefix}_yaw_delta={report['yaw_delta']:.9f}\n")
                f.write(f"{prefix}_steering_trim={report['steering_trim']:.9f}\n")

    def analyze(self):
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("ANALYSIS RESULTS")
        self.get_logger().info("=" * 60)
        if not self.results:
            self.get_logger().warn("No valid turn-in runs recorded")
            return
        for item in self.results:
            self.get_logger().info(
                f"run={item['run_id']:02d} v={item['speed_cmd']:.2f} "
                f"δ={math.degrees(item['steer_cmd']):+.1f}° "
                f"peak_beta={item['peak_beta_deg']:.1f}° "
                f"peak_ay={item['peak_ay']:.2f} m/s² "
                f"peak_gz={item['peak_gz_deg_s']:.1f}°/s "
                f"peak_vy={item['peak_lidar_vy']:.2f} m/s")
        peak_beta = np.array([abs(v['peak_beta_deg']) for v in self.results])
        peak_ay = np.array([abs(v['peak_ay']) for v in self.results])
        self.get_logger().info(
            f"\nSummary: peak_beta mean={np.mean(peak_beta):.2f}° max={np.max(peak_beta):.2f}°")
        self.get_logger().info(
            f"         peak_ay   mean={np.mean(peak_ay):.2f} m/s² max={np.max(peak_ay):.2f} m/s²")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--speeds', type=str, default='2.0,2.5,3.0,3.5',
                        help='Comma-separated speed setpoints [m/s].')
    parser.add_argument('--steering', type=float, nargs='+', default=[0.30],
                        help='Steering step magnitudes [rad].')
    parser.add_argument('--directions', type=str, default='both',
                        choices=['left', 'right', 'both'],
                        help='Which turn directions to test.')
    parser.add_argument('--repeats', type=int, default=2)
    parser.add_argument('--settle-time', type=float, default=1.5)
    parser.add_argument('--turn-time', type=float, default=2.5)
    parser.add_argument('--recover-time', type=float, default=1.0)
    parser.add_argument('--speed-drop', type=float, default=0.0,
                        help='Optional speed setpoint reduction during the turn [m/s].')
    parser.add_argument('--drop-after', type=float, default=0.35,
                        help='Delay before applying speed_drop [s].')
    parser.add_argument('--wheelbase', type=float, default=DEFAULT_WHEELBASE)
    parser.add_argument('--geofence', type=float, default=0.0,
                        help='Max distance from start before abort. 0 = auto.')
    parser.add_argument('--room-length', type=float, default=17.0,
                        help='Available room length [m] for footprint validation.')
    parser.add_argument('--room-width', type=float, default=15.0,
                        help='Available room width [m] for footprint validation.')
    parser.add_argument('--room-margin', type=float, default=0.75,
                        help='Wall clearance margin [m] used in footprint validation.')
    parser.add_argument('--straight-check-speed', type=float, default=1.5,
                        help='Speed used for pre-test straight-line verification [m/s].')
    parser.add_argument('--straight-check-time', type=float, default=1.0,
                        help='Duration of each straight-line verification pass [s].')
    parser.add_argument('--straight-check-max-passes', type=int, default=5,
                        help='Maximum number of iterative straight-trim passes before continuing with best trim.')
    parser.add_argument('--straight-trim-limit', type=float, default=0.06,
                        help='Maximum steering trim correction magnitude [rad].')
    parser.add_argument('--straight-vy-limit', type=float, default=0.10,
                        help='Allowed mean lateral velocity during straight check [m/s].')
    parser.add_argument('--straight-gz-limit', type=float, default=0.18,
                        help='Allowed mean yaw rate during straight check [rad/s].')
    parser.add_argument('--straight-beta-limit', type=float, default=0.05,
                        help='Allowed mean sideslip during straight check [rad].')
    parser.add_argument('--straight-yaw-delta-limit', type=float, default=0.08,
                        help='Allowed yaw change over a straight-check pass [rad].')
    parser.add_argument('--pre-run-pause', type=float, default=5.0,
                        help='Stopped pause before the first measured run [s].')
    parser.add_argument('--between-run-pause', type=float, default=6.0,
                        help='Stopped pause between completed runs [s].')
    args = parser.parse_args()

    args.speeds = _parse_csv_floats(args.speeds)
    if args.directions == 'left':
        args.directions = [1.0]
    elif args.directions == 'right':
        args.directions = [-1.0]
    else:
        args.directions = [1.0, -1.0]

    rclpy.init()
    node = TurnInTransientNode(args)
    try:
        success = node.run_test()
    finally:
        node.stop_car()
        node.destroy_node()
        rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
