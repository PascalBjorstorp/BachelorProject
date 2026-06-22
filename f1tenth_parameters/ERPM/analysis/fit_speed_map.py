#!/usr/bin/env python3
"""Fit raw ERPM command and measured-drivetrain maps against LiDAR ground speed.

Two different quantities are intentionally retained:

* ``selected ERPM -> ground speed`` identifies the command map used by
  ``VEL_TO_ERPM`` (``speed_to_erpm_gain`` / ``offset``).
* ``measured VESC ERPM -> ground speed`` identifies the physical drivetrain
  relation used by ERPM-derived odometry and exposes speed-loop tracking error.

Using VESC telemetry as an exact experimental bucket is avoided.  Coverage is
anchored to the recorded nominal command condition; selected and measured ERPM
remain measured values inside each condition.
"""
from __future__ import annotations
import argparse, json, math
from pathlib import Path
import numpy as np
import pandas as pd
from common import analysis_dir, accepted_capture_windows, coverage, dump_yaml, load_yaml, robust_linear, session_original_values, stage_tables, straight_filter, summarize_windows


def _summary(session: Path, stage: str, phase: str, cfg: dict) -> pd.DataFrame:
    tables = stage_tables(session, stage)
    windows = accepted_capture_windows(tables['events'], phase)
    return straight_filter(summarize_windows(windows, tables, cfg), cfg)


def _condition_coverage(table: pd.DataFrame, keys: list[str], reps: int) -> pd.DataFrame:
    if table.empty:
        return pd.DataFrame(columns=keys + ['accepted_trials', 'expected_trials', 'coverage_ok'])
    return coverage(table, keys, reps)


def _finite_column(table: pd.DataFrame, col: str, label: str) -> np.ndarray:
    if col not in table:
        raise SystemExit(f'{label} has no {col!r} column')
    values = table[col].to_numpy(dtype=float)
    if not np.isfinite(values).all():
        raise SystemExit(f'{label} has non-finite {col} values')
    return values


def _pipeline_summary(session: Path, cfg: dict) -> tuple[pd.DataFrame, pd.DataFrame]:
    pipeline = _summary(session, '05_vel_to_erpm_pipeline_audit', 'vel_to_erpm_pipeline_audit', cfg)
    reps = max(int(cfg['vel_to_erpm_pipeline_audit']['repetitions']), int(cfg['analysis']['gates']['min_training_repetitions']))
    cov = _condition_coverage(pipeline, ['speed_command_mps'], reps)
    return pipeline, cov


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('session', type=Path)
    args = p.parse_args()
    session = args.session.resolve()
    cfg = load_yaml(session / 'calibration_config_snapshot.yaml')
    out = analysis_dir(session)
    gates = cfg['analysis']['gates']

    train = _summary(session, '03_raw_erpm_map_training', 'raw_erpm_training', cfg)
    hold = _summary(session, '04_raw_erpm_map_holdout', 'raw_erpm_holdout', cfg)
    training_expected = max(int(cfg['raw_erpm_map_training']['repetitions']), int(cfg['analysis']['gates']['min_training_repetitions']))
    holdout_expected = max(int(cfg['raw_erpm_map_holdout']['repetitions']), int(cfg['analysis']['gates']['min_holdout_repetitions']))
    train_cov = _condition_coverage(train, ['nominal_speed_mps'], training_expected)
    hold_cov = _condition_coverage(hold, ['nominal_speed_mps'], holdout_expected)
    train_cov.to_parquet(out / 'erpm_map_training_coverage.parquet', index=False)
    hold_cov.to_parquet(out / 'erpm_map_holdout_coverage.parquet', index=False)
    if train.empty:
        raise SystemExit('No accepted, straight, LiDAR-valid raw-ERPM training captures')
    if hold.empty or not bool(train_cov.coverage_ok.all()) or not bool(hold_cov.coverage_ok.all()):
        raise SystemExit('ERPM map coverage incomplete; inspect *_coverage.parquet')

    # ``raw_erpm_target`` is emitted with every phase.  The selector mirror is
    # a redundant independent witness of the command actually delivered to the
    # VESC command topic; use it for the fitted VEL_TO_ERPM command map.
    for table, label in ((train, 'training'), (hold, 'holdout')):
        if 'raw_erpm_target' not in table:
            raise SystemExit(f'{label} raw-ERPM captures are missing raw_erpm_target event metadata')
        table['commanded_erpm'] = table['selected_speed_erpm'].where(
            np.isfinite(table['selected_speed_erpm']), table['raw_erpm_target']
        )
        table['command_delivery_error_erpm'] = table['commanded_erpm'] - table['raw_erpm_target']
        _finite_column(table, 'commanded_erpm', label)
        _finite_column(table, 'erpm_measured', label)
        _finite_column(table, 'vx_lidar_mps', label)

    max_delivery_error = float(gates['max_raw_erpm_delivery_error_erpm'])
    train_delivery = float(np.nanmax(np.abs(train.command_delivery_error_erpm.to_numpy(dtype=float))))
    hold_delivery = float(np.nanmax(np.abs(hold.command_delivery_error_erpm.to_numpy(dtype=float))))
    if max(train_delivery, hold_delivery) > max_delivery_error:
        raise SystemExit('raw-ERPM selector delivery error exceeds gate; inspect selected command mirrors')

    command_fit = robust_linear(train.commanded_erpm.to_numpy(), train.vx_lidar_mps.to_numpy())
    drivetrain_fit = robust_linear(train.erpm_measured.to_numpy(), train.vx_lidar_mps.to_numpy())
    command_a, command_b = command_fit['slope'], command_fit['intercept']
    if not math.isfinite(command_a) or command_a <= 0:
        raise SystemExit(f'Invalid selected-ERPM command map slope: {command_a}')

    train = train.copy(); hold = hold.copy()
    train['vx_pred_command_mps'] = command_a * train.commanded_erpm + command_b
    hold['vx_pred_command_mps'] = command_a * hold.commanded_erpm + command_b
    train['command_map_residual_mps'] = train.vx_lidar_mps - train.vx_pred_command_mps
    hold['command_map_residual_mps'] = hold.vx_lidar_mps - hold.vx_pred_command_mps
    train['vx_pred_measured_erpm_mps'] = drivetrain_fit['slope'] * train.erpm_measured + drivetrain_fit['intercept']
    hold['vx_pred_measured_erpm_mps'] = drivetrain_fit['slope'] * hold.erpm_measured + drivetrain_fit['intercept']
    train['drivetrain_residual_mps'] = train.vx_lidar_mps - train.vx_pred_measured_erpm_mps
    hold['drivetrain_residual_mps'] = hold.vx_lidar_mps - hold.vx_pred_measured_erpm_mps

    nominal = train.groupby('nominal_speed_mps', as_index=False).agg(
        accepted_trials=('trial_id', 'count'),
        raw_erpm_target_median=('raw_erpm_target', 'median'),
        selected_erpm_median=('commanded_erpm', 'median'),
        selected_erpm_std=('commanded_erpm', 'std'),
        measured_erpm_median=('erpm_measured', 'median'),
        measured_erpm_std=('erpm_measured', 'std'),
        vx_lidar_median=('vx_lidar_mps', 'median'),
        vx_lidar_std=('vx_lidar_mps', 'std'),
        command_residual_std_mps=('command_map_residual_mps', 'std'),
        drivetrain_residual_std_mps=('drivetrain_residual_mps', 'std'),
    )
    nominal.to_parquet(out / 'erpm_map_nominal_condition_summary.parquet', index=False)

    train_rmse = float(np.sqrt(np.mean(train.command_map_residual_mps ** 2)))
    hold_rmse = float(np.sqrt(np.mean(hold.command_map_residual_mps ** 2)))
    hold_bias = float(np.mean(hold.command_map_residual_mps))

    # Candidate command map: selected/commanded ERPM = gain * requested speed + offset.
    speed_to_erpm_gain = float(1.0 / command_a)
    speed_to_erpm_offset = float(-command_b / command_a)

    # Odom uses VESC measurement, not the command target.  Its candidate scale is
    # therefore estimated from measured ERPM using the candidate command-map
    # conventions, then independently evaluated on hold-out plateaus.
    original = session_original_values(session)
    current_odom_scale = float(original.get('odom_speed_scale', 1.0) or 1.0)
    candidate_wheel_speed_train = (train.erpm_measured.to_numpy(dtype=float) - speed_to_erpm_offset) / speed_to_erpm_gain
    valid_train = np.isfinite(candidate_wheel_speed_train) & (np.abs(candidate_wheel_speed_train) > 0.08)
    candidate_odom_scale = float(np.median(train.vx_lidar_mps.to_numpy(dtype=float)[valid_train] / candidate_wheel_speed_train[valid_train])) if valid_train.any() else math.nan
    candidate_wheel_speed_hold = (hold.erpm_measured.to_numpy(dtype=float) - speed_to_erpm_offset) / speed_to_erpm_gain
    odom_pred = candidate_odom_scale * candidate_wheel_speed_hold if math.isfinite(candidate_odom_scale) else np.full(len(hold), math.nan)
    odom_hold_rmse = float(np.sqrt(np.nanmean((hold.vx_lidar_mps.to_numpy(dtype=float) - odom_pred) ** 2))) if np.isfinite(odom_pred).any() else math.inf
    current_odom_ratio = train.vx_lidar_mps.to_numpy(dtype=float) / train.odom_vx_mps.to_numpy(dtype=float)
    valid_ratio = np.isfinite(current_odom_ratio) & (np.abs(train.odom_vx_mps.to_numpy(dtype=float)) > 0.08)
    observed_odom_multiplier = float(np.median(current_odom_ratio[valid_ratio])) if valid_ratio.any() else math.nan

    # Raw low-speed trials determine hardware launch threshold; VEL_TO_ERPM
    # pipeline trials determine the currently configured controller's low-speed
    # behaviour and the candidate slow-start recommendations.
    launch = _summary(session, '02_low_speed_launch', 'low_speed_launch', cfg)
    stable_raw = launch[launch.vx_lidar_mps >= float(cfg['low_speed_launch']['minimum_lidar_speed_mps'])].copy()
    if stable_raw.empty:
        min_stable_raw_speed = math.nan; min_stable_raw_erpm = math.nan
    else:
        low = stable_raw.sort_values('erpm_measured').iloc[0]
        min_stable_raw_speed, min_stable_raw_erpm = float(low.vx_lidar_mps), float(low.erpm_measured)

    pipeline, pipeline_cov = _pipeline_summary(session, cfg)
    pipeline_cov.to_parquet(out / 'vel_to_erpm_pipeline_coverage.parquet', index=False)
    pipeline.to_parquet(out / 'vel_to_erpm_pipeline_audit_trials.parquet', index=False)
    if pipeline.empty or not bool(pipeline_cov.coverage_ok.all()):
        raise SystemExit('VEL_TO_ERPM pipeline audit coverage incomplete')
    pipeline_stable = pipeline[pipeline.vx_lidar_mps >= float(cfg['low_speed_launch']['minimum_lidar_speed_mps'])].copy()
    if pipeline_stable.empty:
        min_pipeline_speed = math.nan
    else:
        min_pipeline_speed = float(pipeline_stable.sort_values('speed_command_mps').iloc[0].speed_command_mps)

    stationary = stage_tables(session, '01_longitudinal_observability')
    stationary_w = accepted_capture_windows(stationary['events'], 'stationary_observability')
    stationary_s = summarize_windows(stationary_w, stationary, cfg)
    stationary_noise = float(stationary_s.vx_lidar_std_mps.median()) if len(stationary_s) else math.nan
    odom_deadband = max(float(cfg['analysis']['desired_odom_deadband_mps']), 3.0 * stationary_noise) if math.isfinite(stationary_noise) else float(cfg['analysis']['desired_odom_deadband_mps'])

    accepted = bool(
        train_rmse <= float(gates['max_speed_map_training_rmse_mps']) and
        hold_rmse <= float(gates['max_speed_map_holdout_rmse_mps']) and
        abs(hold_bias) <= float(gates['max_speed_map_holdout_bias_mps']) and
        odom_hold_rmse <= float(gates['max_odom_holdout_rmse_mps']) and
        math.isfinite(min_pipeline_speed)
    )
    report = {
        'candidate_command_model': 'vx_lidar_mps = a_command * selected_ERPM + b_command',
        'a_command_mps_per_erpm': command_a,
        'b_command_mps': command_b,
        'command_training_fit': command_fit,
        'drivetrain_measurement_model': 'vx_lidar_mps = a_measured * VESC_ERPM_measured + b_measured',
        'a_measured_mps_per_erpm': drivetrain_fit['slope'],
        'b_measured_mps': drivetrain_fit['intercept'],
        'measured_erpm_training_fit': drivetrain_fit,
        'max_selector_delivery_error_training_erpm': train_delivery,
        'max_selector_delivery_error_holdout_erpm': hold_delivery,
        'selector_delivery_gate_erpm': max_delivery_error,
        'training_rmse_mps': train_rmse,
        'holdout_rmse_mps': hold_rmse,
        'holdout_bias_mps': hold_bias,
        'candidate_speed_to_erpm_gain': speed_to_erpm_gain,
        'candidate_speed_to_erpm_offset': speed_to_erpm_offset,
        'current_odom_speed_scale': current_odom_scale,
        'observed_odom_scale_multiplier_relative_to_current_config': observed_odom_multiplier,
        'candidate_odom_speed_scale': candidate_odom_scale,
        'candidate_odom_holdout_rmse_mps': odom_hold_rmse,
        'minimum_stable_raw_speed_mps': min_stable_raw_speed,
        'minimum_stable_raw_erpm': min_stable_raw_erpm,
        'minimum_stable_vel_to_erpm_command_mps': min_pipeline_speed,
        'candidate_slow_start_threshold_mps': min_pipeline_speed,
        'candidate_slow_start_increment_mps': min_pipeline_speed,
        'candidate_stop_speed_deadzone_mps': float(cfg['analysis']['desired_stop_speed_deadband_mps']),
        'candidate_odom_speed_deadband_mps': odom_deadband,
        'stationary_lidar_velocity_std_mps': stationary_noise,
        'accepted_for_candidate': accepted,
        'gates': dict(gates),
        'notes': [
            'speed_to_erpm_gain/offset use selected command ERPM because AckermannToVesc emits an ERPM command.',
            'Measured VESC ERPM is retained separately for the physical drivetrain/odometry relation.',
            'Slow-start recommendations are derived from the VEL_TO_ERPM pipeline audit, not raw-ERPM bypass trials.',
        ],
    }
    train.to_parquet(out / 'erpm_map_training_trials.parquet', index=False)
    hold.to_parquet(out / 'erpm_map_holdout_trials.parquet', index=False)
    dump_yaml(out / 'erpm_speed_map_report.yaml', report)
    print(json.dumps(report, indent=2, default=str))
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
