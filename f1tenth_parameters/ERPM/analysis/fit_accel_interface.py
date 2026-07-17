#!/usr/bin/env python3
"""Validate ACCEL_TO_CURRENT routing *and* realised ground acceleration.

This is intentionally not just a topic-echo audit.  It uses the same robust
LiDAR-window pulse slope as the current model and checks whether the candidate
drag feed-forward/gain turns an acceleration request into the requested vehicle
acceleration on a fresh speed/command grid.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

from common import accepted_capture_windows, analysis_dir, dump_yaml, expected_grid_coverage, load_yaml, motion_windows, stage_tables
from fit_current_model import _drag, _mean_accel, _median_speed, _phase_window


def _metric(values: np.ndarray) -> dict[str, float | int]:
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return {
        'rmse': float(np.sqrt(np.mean(values ** 2))) if len(values) else math.inf,
        'bias': float(np.mean(values)) if len(values) else math.nan,
        'n': int(len(values)),
    }


def _expected(command: float, speed: float, current: dict, drag: dict) -> tuple[float, float, str]:
    """Return expected selected output and realised acceleration under C++ logic."""
    deadzone = float(current.get('candidate_accel_deadzone_mps2', 0.02))
    loss = float(_drag(np.asarray([speed]), drag)[0])
    if command > deadzone:
        return float(current['candidate_accel_to_current_gain']) * (command + loss), command, 'drive'
    if command < -deadzone:
        brake_decel = max(abs(command) - loss, 0.0)
        # If requested braking is weaker than natural drag, the C++ node emits
        # zero brake current; realised acceleration remains coast-down drag.
        return float(current['candidate_accel_to_brake_gain']) * brake_decel, -max(abs(command), loss), 'brake'
    # Zero acceleration is an explicit speed-hold command above the stop gate.
    return float(current['candidate_accel_to_current_gain']) * loss, 0.0, 'hold'


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--stage-directory', default='10_accel_to_current_interface')
    parser.add_argument('--validation', action='store_true')
    parser.add_argument('--output-name', default='accel_to_current_interface_report.yaml')
    args = parser.parse_args()
    session = args.session.resolve()
    cfg = load_yaml(session / 'calibration_config_snapshot.yaml')
    out = analysis_dir(session)
    current = yaml.safe_load((out / 'current_acceleration_report.yaml').read_text(encoding='utf-8')) or {}
    drag = yaml.safe_load((out / 'coastdown_drag_report.yaml').read_text(encoding='utf-8')) or {}
    tables = stage_tables(session, args.stage_directory)
    windows = accepted_capture_windows(tables['events'], 'accel_to_current_pulse')
    lidar = motion_windows(tables)
    rows: list[dict[str, object]] = []
    for _, window in windows.iterrows():
        start, end = int(window.start_ns), int(window.end_ns)
        command = float(window.get('acceleration_command_mps2', math.nan))
        initial_speed = float(window.get('initial_speed_mps', math.nan))
        acceleration, median_speed, samples = _mean_accel(lidar, start, end, cfg)
        baseline = _phase_window(tables['events'], str(window.trial_id), 'accel_interface_baseline')
        recovery = _phase_window(tables['events'], str(window.trial_id), 'accel_interface_recovery')
        baseline_speed = _median_speed(lidar, baseline)
        recovery_speed = _median_speed(lidar, recovery)
        selected_current = tables['motor_selected_current'][(tables['motor_selected_current'].bag_ns >= start) & (tables['motor_selected_current'].bag_ns <= end)]
        selected_brake = tables['motor_selected_brake'][(tables['motor_selected_brake'].bag_ns >= start) & (tables['motor_selected_brake'].bag_ns <= end)]
        vesc = tables['vesc'][(tables['vesc'].bag_ns >= start) & (tables['vesc'].bag_ns <= end)]
        imu = tables['imu'][(tables['imu'].bag_ns >= start) & (tables['imu'].bag_ns <= end)]
        expected_output, expected_accel, route = _expected(command, median_speed, current, drag) if math.isfinite(command) and math.isfinite(median_speed) else (math.nan, math.nan, 'unknown')
        observed_output = (
            float(np.nanmedian(selected_brake.value.to_numpy(float))) if route == 'brake' and len(selected_brake)
            else float(np.nanmedian(selected_current.value.to_numpy(float))) if len(selected_current)
            else math.nan
        )
        yaw = float(np.nanmedian(imu.gz.to_numpy(float))) if len(imu) else math.nan
        yaw_ok = not len(imu) or (math.isfinite(yaw) and abs(yaw) <= float(cfg['analysis']['max_straight_yaw_rate_rad_s']))
        measurement_valid = bool(
            math.isfinite(observed_output) and math.isfinite(expected_output)
            and math.isfinite(acceleration) and math.isfinite(expected_accel)
            and math.isfinite(baseline_speed) and math.isfinite(recovery_speed)
            and yaw_ok
        )
        rows.append({
            'trial_id': str(window.trial_id), 'condition_id': str(window.condition_id),
            'initial_speed_mps': initial_speed, 'acceleration_command_mps2': command,
            'route': route, 'selected_current_a': float(np.nanmedian(selected_current.value.to_numpy(float))) if len(selected_current) else math.nan,
            'selected_brake_a': float(np.nanmedian(selected_brake.value.to_numpy(float))) if len(selected_brake) else math.nan,
            'motor_current_a': float(np.nanmedian(vesc.motor_current.to_numpy(float))) if len(vesc) else math.nan,
            'expected_interface_output_a': expected_output, 'observed_interface_output_a': observed_output,
            'interface_residual_a': observed_output - expected_output,
            'expected_ground_accel_mps2': expected_accel, 'observed_ground_accel_mps2': acceleration,
            'ground_accel_residual_mps2': acceleration - expected_accel,
            'vx_lidar_mps': median_speed, 'baseline_vx_lidar_mps': baseline_speed,
            'recovery_vx_lidar_mps': recovery_speed, 'pulse_window_count': samples,
            'imu_yaw_rate_median_rad_s': yaw, 'measurement_valid': measurement_valid,
            'measurement_method': 'robust_window_velocity_slope_with_pre_and_post_reference_windows',
        })
    table = pd.DataFrame(rows)
    suffix = '_validation' if args.validation else ''
    table.to_parquet(out / f'accel_to_current_interface{suffix}_trials.parquet', index=False)
    usable = table[table.measurement_valid.astype(bool)].copy() if not table.empty else table
    spec = cfg['accel_to_current_interface']
    speeds = spec.get('validation_initial_speeds_mps', []) if args.validation else spec['initial_speeds_mps']
    commands = spec.get('validation_acceleration_commands_mps2', []) if args.validation else spec['acceleration_commands_mps2']
    repetitions = int(spec.get('validation_repetitions', spec['repetitions'])) if args.validation else int(spec['repetitions'])
    grid = [(float(v), float(a)) for v in speeds for a in commands]
    coverage = expected_grid_coverage(
        usable, fields=['initial_speed_mps', 'acceleration_command_mps2'], expected_grid=grid,
        expected_repetitions=repetitions,
        tolerances={'initial_speed_mps': 1e-6, 'acceleration_command_mps2': 1e-6},
    )
    coverage.to_parquet(out / f'accel_to_current_interface{suffix}_coverage.parquet', index=False)
    routing = _metric(usable.interface_residual_a.to_numpy(float) if not usable.empty else np.empty(0))
    physical = _metric(usable.ground_accel_residual_mps2.to_numpy(float) if not usable.empty else np.empty(0))
    policy = cfg.get('analysis', {}).get('accel_interface', {})
    gates = cfg['analysis']['gates']
    failures: list[str] = []
    if not bool(len(coverage)) or not bool(coverage.coverage_ok.all()):
        failures.append('ACCEL_TO_CURRENT grid coverage incomplete')
    if routing['rmse'] > float(gates['max_accel_interface_current_rmse_a']):
        failures.append('selected current/brake routing RMSE exceeds gate')
    if physical['rmse'] > float(policy.get('max_ground_accel_rmse_mps2', 0.75)):
        failures.append('realised ground-acceleration RMSE exceeds gate')
    if not math.isfinite(float(physical['bias'])) or abs(float(physical['bias'])) > float(policy.get('max_ground_accel_bias_mps2', 0.35)):
        failures.append('realised ground-acceleration bias exceeds gate')
    report = {
        'validation_capture': bool(args.validation),
        'routing_output_rmse_a': routing['rmse'], 'routing_output_bias_a': routing['bias'],
        'physical_ground_accel_rmse_mps2': physical['rmse'], 'physical_ground_accel_bias_mps2': physical['bias'],
        'trials': int(len(usable)), 'coverage_ok': bool(len(coverage)) and bool(coverage.coverage_ok.all()),
        'accepted_for_candidate': not failures if not args.validation else None,
        'accepted_for_validation': not failures if args.validation else None,
        'gate_max_routing_output_rmse_a': float(gates['max_accel_interface_current_rmse_a']),
        'gate_max_ground_accel_rmse_mps2': float(policy.get('max_ground_accel_rmse_mps2', 0.75)),
        'gate_max_ground_accel_bias_mps2': float(policy.get('max_ground_accel_bias_mps2', 0.35)),
        'coverage': coverage.to_dict(orient='records'),
        'failures': failures,
        'note': 'Positive acceleration includes drag feed-forward; negative commands use explicit brake only beyond natural coast-down drag. Ground acceleration is measured from LiDAR windows, not odometry.',
    }
    dump_yaml(out / args.output_name, report)
    print(json.dumps(report, indent=2))
    if failures:
        raise SystemExit('ACCEL_TO_CURRENT interface rejected: ' + '; '.join(failures))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
