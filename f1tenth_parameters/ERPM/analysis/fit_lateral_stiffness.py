#!/usr/bin/env python3
"""Fit effective front/rear tyre cornering stiffness from quasi-steady arcs.

This implements the quasi-steady bicycle-model approach used in the supplied
thesis, with two important improvements for this vehicle: LiDAR-window body
velocity supplies a direct beta estimate, and every fit is performed once per
accepted manoeuvre rather than once per highly-correlated scan registration.

The resulting Cf/Cr values are *effective* low-slip values for the measured
tyres, load, battery and surface.  They are not universal tyre constants.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from common import accepted_capture_windows, analysis_dir, dump_yaml, expected_grid_coverage, load_yaml, motion_windows, stage_tables


def _median(frame: pd.DataFrame, column: str) -> float:
    if frame.empty or column not in frame:
        return math.nan
    values = frame[column].to_numpy(float)
    return float(np.nanmedian(values)) if np.isfinite(values).any() else math.nan


def _std(frame: pd.DataFrame, column: str) -> float:
    if frame.empty or column not in frame:
        return math.nan
    values = frame[column].to_numpy(float)
    return float(np.nanstd(values)) if np.isfinite(values).any() else math.nan


def _stationary_imu_diagnostic(session: Path) -> dict[str, float]:
    """Report the observability epoch without carrying it into later stacks.

    The VESC/odom bringup recalibrates IMU bias at every launch.  Reusing one
    stationary value collected during the earlier longitudinal preflight would
    create a stale correction in the steering/lateral stages.  These numbers
    therefore remain explicit diagnostics only.
    """
    tables = stage_tables(session, '01_longitudinal_observability')
    windows = accepted_capture_windows(tables['events'], 'stationary_observability')
    if windows.empty or tables['imu'].empty:
        return {'ax_mps2': 0.0, 'ay_mps2': 0.0, 'gz_rad_s': 0.0, 'available': False}
    values: dict[str, list[float]] = {'ax': [], 'ay': [], 'gz': []}
    for _, window in windows.iterrows():
        part = tables['imu'][(tables['imu'].bag_ns >= window.start_ns) & (tables['imu'].bag_ns <= window.end_ns)]
        for short, column in (('ax', 'ax'), ('ay', 'ay'), ('gz', 'gz')):
            value = _median(part, column)
            if math.isfinite(value):
                values[short].append(value)
    return {
        'ax_mps2': float(np.median(values['ax'])) if values['ax'] else 0.0,
        'ay_mps2': float(np.median(values['ay'])) if values['ay'] else 0.0,
        'gz_rad_s': float(np.median(values['gz'])) if values['gz'] else 0.0,
        'available': bool(values['ax'] or values['ay'] or values['gz']),
    }


def _physical(session: Path) -> dict[str, Any]:
    path = session / 'analysis' / 'physical_vehicle_parameters.yaml'
    if not path.is_file():
        raise ValueError('physical_metrology has not produced physical_vehicle_parameters.yaml')
    import yaml
    value = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    if not isinstance(value, dict) or not bool(value.get('accepted_for_lateral_identification', False)):
        raise ValueError('physical metrology is incomplete; fill the session physical_measurements.yaml sheet')
    for key in ('mass_kg', 'wheelbase_m', 'cg_to_front_axle_lf_m', 'cg_to_rear_axle_lr_m'):
        try:
            number = float(value[key])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f'physical metrology has no finite {key}') from exc
        if not math.isfinite(number) or number <= 0.0:
            raise ValueError(f'physical metrology has invalid {key}')
    cg = value.get('cg_in_base_link', {})
    if not isinstance(cg, dict):
        raise ValueError('physical metrology has no cg_in_base_link mapping')
    for axis in ('x_m', 'y_m'):
        try:
            number = float(cg[axis])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f'physical metrology has no finite cg_in_base_link.{axis}') from exc
        if not math.isfinite(number):
            raise ValueError(f'physical metrology has invalid cg_in_base_link.{axis}')
    imu = value.get('imu_to_base', {})
    if not isinstance(imu, dict):
        raise ValueError('physical metrology has no imu_to_base mapping')
    for axis in ('x_m', 'y_m', 'yaw_rad'):
        try:
            number = float(imu[axis])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f'physical metrology has no finite imu_to_base.{axis}') from exc
        if not math.isfinite(number):
            raise ValueError(f'physical metrology has invalid imu_to_base.{axis}')
    return value


def _expected_grid(cfg: dict[str, Any], *, validation: bool) -> list[tuple[float, float]]:
    spec = cfg['lateral_stiffness']
    speeds = spec.get('validation_speeds_mps', []) if validation else spec.get('speeds_mps', [])
    angles = spec.get('validation_steering_angles_rad', []) if validation else spec.get('steering_angles_rad', [])
    signs = spec.get('validation_signs', spec.get('signs', [-1.0, 1.0])) if validation else spec.get('signs', [-1.0, 1.0])
    return [(float(speed), math.copysign(float(angle), float(sign))) for speed in speeds for angle in angles for sign in signs]


def _selected_wheel_model(session: Path) -> dict[str, Any]:
    """Return the frozen ERPM-to-wheel-speed observation used by the arc tests."""
    patch_path = session / 'analysis' / 'selected_odometry_candidate_patch.yaml'
    if patch_path.is_file():
        params = load_yaml(patch_path).get('vesc_to_odom_node', {}).get('ros__parameters', {})
        kind = str(params.get('odom_wheel_model', 'linear'))
        model = {
            'kind': kind,
            'linear_mps_per_erpm': float(params.get('odom_erpm_to_speed_linear', 0.0)),
            'quadratic_mps_per_erpm2': float(params.get('odom_erpm_to_speed_quadratic', 0.0)),
            'erpm_knots': list(map(float, params.get('odom_erpm_lut_erpm', []))),
            'speed_knots_mps': list(map(float, params.get('odom_erpm_lut_speed_mps', []))),
            'source': 'selected_odometry_candidate_patch.yaml',
        }
        if kind == 'lut':
            if len(model['erpm_knots']) >= 2 and len(model['erpm_knots']) == len(model['speed_knots_mps']):
                return model
        elif abs(float(model['linear_mps_per_erpm'])) > 0.0:
            return model

    # Compatibility path for a session made before odometry model selection.
    report_path = session / 'analysis' / 'erpm_speed_map_report.yaml'
    if report_path.is_file():
        report = load_yaml(report_path)
        gain = float(report.get('candidate_speed_to_erpm_gain', 0.0))
        scale = float(report.get('candidate_odom_speed_scale', 1.0))
        if math.isfinite(gain) and gain > 0.0 and math.isfinite(scale):
            return {
                'kind': 'linear',
                'linear_mps_per_erpm': scale / gain,
                'quadratic_mps_per_erpm2': 0.0,
                'erpm_knots': [],
                'speed_knots_mps': [],
                'source': 'erpm_speed_map_report.yaml',
            }
    raise ValueError('no frozen ERPM-to-wheel-speed model is available for cornering-slip identification')


def _wheel_speed(erpm: float, model: dict[str, Any]) -> float:
    if not math.isfinite(erpm):
        return math.nan
    if str(model.get('kind')) == 'lut':
        x = np.asarray(model.get('erpm_knots', []), dtype=float)
        y = np.asarray(model.get('speed_knots_mps', []), dtype=float)
        if len(x) < 2 or len(x) != len(y):
            return math.nan
        sign = math.copysign(1.0, erpm)
        magnitude = abs(erpm)
        if magnitude <= float(x[-1]):
            return sign * float(np.interp(magnitude, x, y))
        slope = float((y[-1] - y[-2]) / max(x[-1] - x[-2], 1e-12))
        return sign * float(y[-1] + slope * (magnitude - x[-1]))
    linear = float(model.get('linear_mps_per_erpm', 0.0))
    quadratic = float(model.get('quadratic_mps_per_erpm2', 0.0))
    return linear * erpm + quadratic * erpm * abs(erpm)


def collect_lateral_trials(session: Path, stage: str, cfg: dict[str, Any], physical: dict[str, Any],
                           *, steering_model_scale: float = 1.0) -> pd.DataFrame:
    tables = stage_tables(session, stage)
    windows = accepted_capture_windows(tables['events'], 'lateral_quasi_steady')
    lidar_source = motion_windows(tables)
    policy = cfg.get('analysis', {}).get('lateral_stiffness', {})
    imu_diagnostic = _stationary_imu_diagnostic(session)
    mass = float(physical['mass_kg'])
    wheelbase = float(physical['wheelbase_m'])
    lf = float(physical['cg_to_front_axle_lf_m'])
    lr = float(physical['cg_to_rear_axle_lr_m'])
    cg_reference = physical['cg_in_base_link']
    cg_x = float(cg_reference['x_m'])
    cg_y = float(cg_reference['y_m'])
    imu_reference = physical['imu_to_base']
    imu_x = float(imu_reference['x_m'])
    imu_y = float(imu_reference['y_m'])
    imu_yaw = float(imu_reference['yaw_rad'])
    wheel_model = _selected_wheel_model(session)
    rows: list[dict[str, Any]] = []
    for _, window in windows.iterrows():
        start, end = int(window.start_ns), int(window.end_ns)
        lidar_all = lidar_source[(lidar_source.bag_ns >= start) & (lidar_source.bag_ns <= end)].copy()
        lidar = lidar_all[lidar_all.valid.astype(bool)].copy() if not lidar_all.empty and 'valid' in lidar_all else lidar_all.iloc[0:0].copy()
        imu = tables['imu'][(tables['imu'].bag_ns >= start) & (tables['imu'].bag_ns <= end)].copy()
        vesc = tables['vesc'][(tables['vesc'].bag_ns >= start) & (tables['vesc'].bag_ns <= end)].copy()
        odom = tables['odom'][(tables['odom'].bag_ns >= start) & (tables['odom'].bag_ns <= end)].copy()
        delta_command = float(window.get('steering_angle_rad', window.get('steering_angle_target_rad', math.nan)))
        delta = float(steering_model_scale) * delta_command if math.isfinite(delta_command) else math.nan
        speed_command = float(window.get('speed_command_mps', math.nan))
        vx_base, vy_base = _median(lidar, 'vx'), _median(lidar, 'vy')
        vx_std = _std(lidar, 'vx')
        raw_yaw = _median(imu, 'gz')
        raw_ay = _median(imu, 'ay')
        raw_ax = _median(imu, 'ax')
        # Use the current stack's IMU output as produced by bringup.  The
        # stationary diagnostic above is deliberately not subtracted here.
        yaw = raw_yaw if math.isfinite(raw_yaw) else math.nan
        # Rotate the horizontal accelerometer components into base_link before
        # moving their reference point to the CG.  This keeps a small IMU yaw
        # mounting error from looking like a tyre-force asymmetry.
        if math.isfinite(raw_ax) and math.isfinite(raw_ay):
            cos_imu, sin_imu = math.cos(imu_yaw), math.sin(imu_yaw)
            ax = cos_imu * raw_ax - sin_imu * raw_ay
            ay = sin_imu * raw_ax + cos_imu * raw_ay
        else:
            ax = ay = math.nan
        yaw_std = _std(imu, 'gz')
        ay_std = _std(imu, 'ay')
        erpm = _median(vesc, 'erpm')
        runtime_odom_vx = _median(odom, 'vx')
        wheel_speed = _wheel_speed(erpm, wheel_model)
        # LiDAR motion has already been transformed to base_link.  The bicycle
        # equations, however, are written at the CG.  A rigid-body shift is
        # material on this short wheelbase: v_CG = v_base + r x p_CG/base.
        vx = vx_base - yaw * cg_y if math.isfinite(vx_base) and math.isfinite(yaw) else math.nan
        vy = vy_base + yaw * cg_x if math.isfinite(vy_base) and math.isfinite(yaw) else math.nan
        # For a steady turn, a_CG = a_IMU - r^2 p_CG/IMU (angular acceleration
        # is gated out by the quasi-steady windows). This uses the measured IMU
        # transform rather than assuming the IMU origin equals base_link.
        cg_from_imu_x, cg_from_imu_y = cg_x - imu_x, cg_y - imu_y
        ay_cg = ay - yaw * yaw * cg_from_imu_y if math.isfinite(ay) and math.isfinite(yaw) else math.nan
        ax_cg = ax - yaw * yaw * cg_from_imu_x if math.isfinite(ax) and math.isfinite(yaw) else math.nan
        valid_fraction = float(len(lidar) / max(1, len(lidar_all)))
        beta_lidar = math.atan2(vy, vx) if math.isfinite(vx) and math.isfinite(vy) and vx > 0.0 else math.nan
        beta_quasi_steady = math.atan2(-ax_cg / yaw, vx) if math.isfinite(ax_cg) and math.isfinite(yaw) and abs(yaw) > float(policy.get('min_abs_yaw_rate_rad_s', 0.08)) and vx > 0.0 else math.nan
        beta_gap = beta_lidar - beta_quasi_steady if math.isfinite(beta_lidar) and math.isfinite(beta_quasi_steady) else math.nan
        ay_kinematic = vx * yaw if math.isfinite(vx) and math.isfinite(yaw) else math.nan
        ay_gap = ay_cg - ay_kinematic if math.isfinite(ay_cg) and math.isfinite(ay_kinematic) else math.nan
        cos_delta = math.cos(delta) if math.isfinite(delta) else math.nan
        fy_front_base = lr * mass * ay_cg / wheelbase if math.isfinite(ay_cg) else math.nan
        fy_front = fy_front_base / cos_delta if math.isfinite(fy_front_base) and math.isfinite(cos_delta) and abs(cos_delta) > 0.1 else math.nan
        fy_rear = lf * mass * ay_cg / wheelbase if math.isfinite(ay_cg) else math.nan
        front_kinematic_term = -beta_lidar - lf * yaw / vx if all(math.isfinite(value) for value in (beta_lidar, yaw, vx)) and vx > 0.0 else math.nan
        alpha_front = delta + front_kinematic_term if all(math.isfinite(value) for value in (delta, front_kinematic_term)) else math.nan
        alpha_rear = -beta_lidar + lr * yaw / vx if all(math.isfinite(value) for value in (beta_lidar, yaw, vx)) and vx > 0.0 else math.nan
        reasons: list[str] = []
        if len(lidar) < int(policy.get('min_valid_windows_per_trial', 6)):
            reasons.append('too_few_valid_lidar_windows')
        if valid_fraction < float(policy.get('min_lidar_window_valid_fraction', 0.70)):
            reasons.append('low_lidar_window_valid_fraction')
        if not math.isfinite(vx) or vx < float(policy.get('min_speed_mps', 0.45)):
            reasons.append('insufficient_forward_speed')
        if math.isfinite(vx_std) and vx_std > float(policy.get('max_speed_std_mps', 0.12)):
            reasons.append('speed_not_quasi_steady')
        if not math.isfinite(yaw) or abs(yaw) < float(policy.get('min_abs_yaw_rate_rad_s', 0.08)):
            reasons.append('yaw_rate_too_small_for_lateral_identification')
        if math.isfinite(yaw_std) and yaw_std > float(policy.get('max_yaw_rate_std_rad_s', 0.10)):
            reasons.append('yaw_rate_not_quasi_steady')
        if math.isfinite(ay_std) and ay_std > float(policy.get('max_lateral_accel_std_mps2', 0.35)):
            reasons.append('lateral_acceleration_not_quasi_steady')
        if not math.isfinite(ay_cg) or abs(ay_cg) < float(policy.get('min_abs_lateral_accel_mps2', 0.20)):
            reasons.append('lateral_acceleration_too_small')
        if math.isfinite(ay_gap) and abs(ay_gap) > float(policy.get('max_imu_kinematic_ay_disagreement_mps2', 0.45)):
            reasons.append('imu_and_vx_yaw_lateral_accel_disagree')
        if math.isfinite(beta_gap) and abs(beta_gap) > float(policy.get('max_lidar_quasi_steady_beta_disagreement_rad', 0.12)):
            reasons.append('lidar_and_quasi_steady_beta_disagree')
        if not all(math.isfinite(value) for value in (alpha_front, alpha_rear, fy_front, fy_rear)):
            reasons.append('nonfinite_slip_or_force')
        else:
            min_slip = float(policy.get('min_abs_slip_angle_rad', 0.003))
            max_slip = float(policy.get('max_abs_slip_angle_rad', 0.16))
            if not min_slip <= abs(alpha_front) <= max_slip or not min_slip <= abs(alpha_rear) <= max_slip:
                reasons.append('slip_angle_outside_identifiable_linear_region')
            if alpha_front * fy_front <= 0.0 or alpha_rear * fy_rear <= 0.0:
                reasons.append('slip_force_sign_inconsistent')
        if math.isfinite(delta_command) and math.isfinite(yaw) and delta_command * yaw <= 0.0:
            reasons.append('turn_direction_inconsistent_with_command')
        turn_slip_reasons = list(reasons)
        min_turn_speed = float(policy.get('turn_slip_min_speed_mps', policy.get('min_speed_mps', 0.45)))
        if not math.isfinite(erpm):
            turn_slip_reasons.append('missing_erpm_for_turn_slip')
        if not math.isfinite(wheel_speed) or wheel_speed <= min_turn_speed:
            turn_slip_reasons.append('wheel_speed_too_small_for_turn_slip')
        turn_regressor = (
            abs(wheel_speed) * abs(yaw)
            if math.isfinite(wheel_speed) and math.isfinite(yaw) else math.nan
        )
        slip_fraction = (
            1.0 - vx / wheel_speed
            if math.isfinite(vx) and math.isfinite(wheel_speed) and abs(wheel_speed) > 1e-9 else math.nan
        )
        if not math.isfinite(turn_regressor) or not math.isfinite(slip_fraction):
            turn_slip_reasons.append('nonfinite_turn_slip_measurement')
        rows.append({
            'trial_id': str(window.trial_id), 'condition_id': str(window.condition_id),
            'speed_command_mps': speed_command,
            # The command angle is the nominal-grid key. The separately stored
            # model angle is the candidate passed to the dynamic bicycle model.
            'steering_angle_rad': delta_command,
            'steering_angle_model_rad': delta,
            'steering_model_scale_used': float(steering_model_scale),
            'vx_base_link_mps': vx_base, 'vy_base_link_mps': vy_base,
            'vx_lidar_mps': vx, 'vy_lidar_mps': vy, 'vx_lidar_std_mps': vx_std,
            'lidar_valid_fraction': valid_fraction, 'valid_lidar_windows': int(len(lidar)),
            'imu_yaw_rate_rad_s': yaw, 'imu_yaw_rate_std_rad_s': yaw_std,
            'erpm_measured': erpm,
            'v_wheel_frozen_mps': wheel_speed,
            'runtime_odom_vx_mps': runtime_odom_vx,
            'turn_slip_lateral_accel_regressor_mps2': turn_regressor,
            'turn_slip_fraction': slip_fraction,
            'turn_slip_measurement_valid': not turn_slip_reasons,
            'turn_slip_rejection_reasons': ';'.join(dict.fromkeys(turn_slip_reasons)),
            'wheel_speed_model_source': str(wheel_model.get('source', 'unknown')),
            'wheel_speed_model_kind': str(wheel_model.get('kind', 'unknown')),
            'imu_ax_raw_mps2': raw_ax, 'imu_ay_raw_mps2': raw_ay,
            'imu_ax_mps2': ax, 'imu_ay_mps2': ay,
            'imu_to_base_yaw_rad': imu_yaw,
            'cg_ax_mps2': ax_cg, 'cg_ay_mps2': ay_cg,
            'imu_ay_std_mps2': ay_std,
            'beta_lidar_rad': beta_lidar, 'beta_quasi_steady_rad': beta_quasi_steady,
            'beta_lidar_minus_quasi_steady_rad': beta_gap,
            'ay_kinematic_vx_times_r_mps2': ay_kinematic, 'imu_minus_kinematic_ay_mps2': ay_gap,
            'front_kinematic_slip_term_rad': front_kinematic_term,
            'alpha_front_rad': alpha_front, 'alpha_rear_rad': alpha_rear,
            'front_lateral_force_base_N': fy_front_base,
            'fy_front_N': fy_front, 'fy_rear_N': fy_rear,
            'measurement_valid': not reasons, 'rejection_reasons': ';'.join(reasons),
            'stationary_imu_diagnostic_available': bool(imu_diagnostic['available']),
            'stationary_imu_correction_applied': False,
        })
    return pd.DataFrame(rows)


def _speed_metrics(truth: np.ndarray, prediction: np.ndarray) -> dict[str, float | int]:
    truth = np.asarray(truth, dtype=float)
    prediction = np.asarray(prediction, dtype=float)
    keep = np.isfinite(truth) & np.isfinite(prediction)
    truth, prediction = truth[keep], prediction[keep]
    if not len(truth):
        return {'n': 0, 'rmse_mps': math.inf, 'bias_mps': math.nan, 'p95_abs_error_mps': math.inf}
    residual = prediction - truth
    return {
        'n': int(len(residual)),
        'rmse_mps': float(np.sqrt(np.mean(residual ** 2))),
        'bias_mps': float(np.mean(residual)),
        'p95_abs_error_mps': float(np.quantile(np.abs(residual), 0.95)),
    }


def apply_turn_slip(v_wheel: np.ndarray, a_lat: np.ndarray, coefficient: float,
                    clip_fraction: float) -> np.ndarray:
    fraction = np.clip(float(coefficient) * np.asarray(a_lat, dtype=float), 0.0, float(clip_fraction))
    return np.asarray(v_wheel, dtype=float) * (1.0 - fraction)


def fit_cornering_longitudinal_slip(frame: pd.DataFrame, *, clip_fraction: float,
                                    min_coefficient: float, min_improvement_fraction: float,
                                    bootstrap_resamples: int, seed: int) -> dict[str, Any]:
    """Fit one causal cornering correction from independent accepted trials."""
    if frame.empty or 'turn_slip_measurement_valid' not in frame:
        raise ValueError('cornering-slip fit has no trial evidence')
    usable = frame[frame.turn_slip_measurement_valid.astype(bool)].copy()
    if len(usable) < 8 or usable.trial_id.nunique() < 8:
        raise ValueError('cornering-slip fit requires at least eight independent usable manoeuvres')
    a_lat = usable.turn_slip_lateral_accel_regressor_mps2.to_numpy(float)
    slip = usable.turn_slip_fraction.to_numpy(float)
    denominator = float(np.dot(a_lat, a_lat))
    coefficient = float(np.dot(a_lat, slip) / max(denominator, 1e-12))
    predicted_slip = coefficient * a_lat
    slip_denom = float(np.sum((slip - np.mean(slip)) ** 2))
    r2 = float(1.0 - np.sum((slip - predicted_slip) ** 2) / slip_denom) if slip_denom > 1e-12 else math.nan
    truth = usable.vx_lidar_mps.to_numpy(float)
    wheel = usable.v_wheel_frozen_mps.to_numpy(float)
    baseline = _speed_metrics(truth, wheel)
    fitted_prediction = apply_turn_slip(wheel, a_lat, coefficient, clip_fraction)
    fitted = _speed_metrics(truth, fitted_prediction)
    improvement = (
        (float(baseline['rmse_mps']) - float(fitted['rmse_mps'])) / max(float(baseline['rmse_mps']), 1e-12)
        if math.isfinite(float(baseline['rmse_mps'])) and math.isfinite(float(fitted['rmse_mps'])) else -math.inf
    )
    groups = [part for _, part in usable.groupby('trial_id', sort=False)]
    rng = np.random.default_rng(seed)
    bootstrap: list[float] = []
    for _ in range(max(0, int(bootstrap_resamples))):
        sample = pd.concat(
            [groups[index] for index in rng.integers(0, len(groups), size=len(groups))],
            ignore_index=True,
        )
        ba = sample.turn_slip_lateral_accel_regressor_mps2.to_numpy(float)
        bs = sample.turn_slip_fraction.to_numpy(float)
        bdenom = float(np.dot(ba, ba))
        if bdenom > 1e-12:
            value = float(np.dot(ba, bs) / bdenom)
            if math.isfinite(value):
                bootstrap.append(value)
    interval = [float(x) for x in np.quantile(bootstrap, [0.025, 0.975])] if bootstrap else []
    evidence_supports_correction = bool(
        math.isfinite(coefficient)
        and coefficient >= float(min_coefficient)
        and improvement >= float(min_improvement_fraction)
        and len(interval) == 2 and interval[0] > 0.0
    )
    selected_coefficient = coefficient if evidence_supports_correction else 0.0
    selected_prediction = apply_turn_slip(wheel, a_lat, selected_coefficient, clip_fraction)
    return {
        'model': 'v_ground = v_wheel * (1 - clip(c1 * |v_wheel|*|yaw_rate|, 0, clip_fraction))',
        'fit_measurement_unit': 'one aggregate per independent accepted arc trial',
        'usable_independent_trials': int(usable.trial_id.nunique()),
        'fitted_coefficient_per_mps2': coefficient,
        'selected_coefficient_per_mps2': selected_coefficient,
        'coefficient_bootstrap_95pct_per_mps2': interval,
        'bootstrap_valid_resamples': int(len(bootstrap)),
        'clip_fraction': float(clip_fraction),
        'train_slip_fraction_r2': r2,
        'uncorrected_training': baseline,
        'fitted_training': fitted,
        'selected_training': _speed_metrics(truth, selected_prediction),
        'fitted_rmse_improvement_fraction': improvement,
        'correction_active': evidence_supports_correction,
        'accepted_for_candidate': True,
        'selection_reason': (
            'Positive trial-bootstrap evidence and material training RMSE reduction support the causal correction.'
            if evidence_supports_correction else
            'No statistically supported material cornering over-read was found; the frozen candidate is exactly zero.'
        ),
        'straight_line_preserved': True,
    }


def add_turn_slip_predictions(frame: pd.DataFrame, candidate: dict[str, Any]) -> pd.DataFrame:
    result = frame.copy()
    if result.empty:
        return result
    coefficient = float(candidate.get('selected_coefficient_per_mps2', 0.0))
    clip_fraction = float(candidate.get('clip_fraction', 0.25))
    result['turn_slip_frozen_prediction_mps'] = apply_turn_slip(
        result.v_wheel_frozen_mps.to_numpy(float),
        result.turn_slip_lateral_accel_regressor_mps2.to_numpy(float),
        coefficient, clip_fraction,
    )
    result['turn_slip_uncorrected_error_mps'] = result.v_wheel_frozen_mps - result.vx_lidar_mps
    result['turn_slip_frozen_error_mps'] = result.turn_slip_frozen_prediction_mps - result.vx_lidar_mps
    return result


def _metrics(actual: np.ndarray, prediction: np.ndarray) -> dict[str, float | int]:
    residual = np.asarray(actual, dtype=float) - np.asarray(prediction, dtype=float)
    residual = residual[np.isfinite(residual)]
    if not len(residual):
        return {'rmse_N': math.inf, 'bias_N': math.nan, 'normalized_rmse': math.inf, 'r2': math.nan, 'n': 0}
    y = np.asarray(actual, dtype=float)
    y = y[np.isfinite(y)]
    rms = float(np.sqrt(np.mean(y ** 2))) if len(y) else math.nan
    denom = float(np.sum((y - np.mean(y)) ** 2)) if len(y) else math.nan
    return {
        'rmse_N': float(np.sqrt(np.mean(residual ** 2))), 'bias_N': float(np.mean(residual)),
        'normalized_rmse': float(np.sqrt(np.mean(residual ** 2)) / max(rms, 1e-9)),
        'r2': float(1.0 - np.sum(residual ** 2) / denom) if math.isfinite(denom) and denom > 1e-12 else math.nan,
        'n': int(len(residual)),
    }


def bounded_tyre_prediction(alpha: np.ndarray, stiffness: float, shape_factor: float) -> np.ndarray:
    """Exact bounded tyre equation used by ``vesc_to_odom`` at runtime."""
    alpha = np.asarray(alpha, dtype=float)
    shape = float(shape_factor)
    if not math.isfinite(shape) or shape <= 0.0:
        return np.full_like(alpha, np.nan)
    # D=C_alpha, B=1/shape, so the zero-slip slope remains C_alpha.
    return float(stiffness) * np.sin(shape * np.arctan(alpha / shape))


def fit_tyre(alpha: np.ndarray, force: np.ndarray) -> dict[str, Any]:
    alpha = np.asarray(alpha, dtype=float)
    force = np.asarray(force, dtype=float)
    keep = np.isfinite(alpha) & np.isfinite(force)
    alpha, force = alpha[keep], force[keep]
    if len(alpha) < 8:
        raise ValueError('at least eight usable manoeuvres are required for each tyre fit')
    stiffness = float(np.dot(alpha, force) / max(np.dot(alpha, alpha), 1e-12))
    linear_prediction = stiffness * alpha
    X = np.column_stack([alpha, alpha * np.abs(alpha)])
    nonlinear_coeff, *_ = np.linalg.lstsq(X, force, rcond=None)
    nonlinear_prediction = X @ nonlinear_coeff
    return {
        'linear': {
            'cornering_stiffness_N_per_rad': stiffness,
            'model_form': 'Fy = C_alpha * alpha',
            'metrics': _metrics(force, linear_prediction),
        },
        'nonlinear': {
            'cornering_stiffness_N_per_rad': float(nonlinear_coeff[0]),
            'quadratic_saturation_N_per_rad2': float(nonlinear_coeff[1]),
            'model_form': 'Fy = C_alpha*alpha + q*alpha*abs(alpha)',
            'metrics': _metrics(force, nonlinear_prediction),
        },
        'design_condition_number': float(np.linalg.cond(X)),
    }


def fit_front_tyre_and_steering_scale(frame: pd.DataFrame) -> dict[str, Any]:
    """Jointly fit front stiffness and the dynamic-model steering scale.

    The static LiDAR map provides a well-observed command-to-*effective*
    steering angle.  The bicycle dynamics still needs a scale between that map
    and the front-tyre input.  With balanced speed/steering arcs the front
    equation is

        F_yf = C_f (s delta_cmd + k),  k = -beta - l_f r / v_x.

    This exposes both ``C_f`` and ``s`` instead of silently retaining a stale
    hand-tuned ``steering_model_scale``.  The small ``cos(delta)`` front-force
    projection is iterated self-consistently after each scale update.
    """
    required = ('steering_angle_rad', 'front_kinematic_slip_term_rad', 'front_lateral_force_base_N')
    if any(column not in frame for column in required):
        raise ValueError('front stiffness/scale fit is missing steering or force columns')
    command = frame.steering_angle_rad.to_numpy(float)
    kinematic = frame.front_kinematic_slip_term_rad.to_numpy(float)
    force_base = frame.front_lateral_force_base_N.to_numpy(float)
    keep = np.isfinite(command) & np.isfinite(kinematic) & np.isfinite(force_base)
    command, kinematic, force_base = command[keep], kinematic[keep], force_base[keep]
    if len(command) < 8:
        raise ValueError('at least eight usable manoeuvres are required for the front stiffness/scale fit')
    scale = 1.0
    coeff = np.array([math.nan, math.nan], dtype=float)
    force = np.full(len(command), math.nan)
    for iteration in range(12):
        cos_delta = np.cos(scale * command)
        valid = np.abs(cos_delta) > 0.1
        if int(valid.sum()) < 8:
            raise ValueError('front steering-scale iteration leaves too few valid force samples')
        force = force_base[valid] / cos_delta[valid]
        X = np.column_stack([command[valid], kinematic[valid]])
        coeff, *_ = np.linalg.lstsq(X, force, rcond=None)
        command_coefficient, stiffness = (float(value) for value in coeff)
        if not math.isfinite(stiffness) or abs(stiffness) <= 1e-10:
            raise ValueError('front steering-scale fit has zero/non-finite stiffness coefficient')
        next_scale = command_coefficient / stiffness
        if not math.isfinite(next_scale):
            raise ValueError('front steering-scale fit has no finite scale')
        if abs(next_scale - scale) < 1e-7:
            scale = next_scale
            break
        scale = next_scale
    cos_delta = np.cos(scale * command)
    valid = np.abs(cos_delta) > 0.1
    command, kinematic, force_base, cos_delta = command[valid], kinematic[valid], force_base[valid], cos_delta[valid]
    force = force_base / cos_delta
    X = np.column_stack([command, kinematic])
    coeff, *_ = np.linalg.lstsq(X, force, rcond=None)
    command_coefficient, stiffness = (float(value) for value in coeff)
    if not math.isfinite(stiffness) or abs(stiffness) <= 1e-10:
        raise ValueError('front steering-scale final fit has invalid stiffness')
    scale = command_coefficient / stiffness
    alpha = scale * command + kinematic
    linear_prediction = stiffness * alpha
    nonlinear_X = np.column_stack([alpha, alpha * np.abs(alpha)])
    nonlinear_coeff, *_ = np.linalg.lstsq(nonlinear_X, force, rcond=None)
    nonlinear_prediction = nonlinear_X @ nonlinear_coeff
    return {
        'linear': {
            'cornering_stiffness_N_per_rad': stiffness,
            'model_form': 'Fy = C_alpha * (steering_model_scale*delta_command - beta - l_f*r/v_x)',
            'metrics': _metrics(force, linear_prediction),
        },
        'nonlinear': {
            'cornering_stiffness_N_per_rad': float(nonlinear_coeff[0]),
            'quadratic_saturation_N_per_rad2': float(nonlinear_coeff[1]),
            'model_form': 'Fy = C_alpha*alpha + q*alpha*abs(alpha), with frozen fitted steering-model scale',
            'metrics': _metrics(force, nonlinear_prediction),
        },
        'steering_model_scale_candidate': float(scale),
        'scale_fit_iterations': int(iteration + 1),
        'design_condition_number': float(np.linalg.cond(X)),
    }


def apply_front_steering_scale(frame: pd.DataFrame, scale: float) -> pd.DataFrame:
    """Make trial artefacts represent the front candidate actually validated."""
    result = frame.copy()
    if result.empty:
        return result
    command = result.steering_angle_rad.to_numpy(float)
    term = result.front_kinematic_slip_term_rad.to_numpy(float)
    force_base = result.front_lateral_force_base_N.to_numpy(float)
    delta = float(scale) * command
    cos_delta = np.cos(delta)
    result['steering_angle_model_rad'] = delta
    result['steering_model_scale_used'] = float(scale)
    result['alpha_front_rad'] = delta + term
    result['fy_front_N'] = np.divide(
        force_base, cos_delta, out=np.full(len(result), np.nan), where=np.abs(cos_delta) > 0.1,
    )
    return result


def bootstrap_stiffness(frame: pd.DataFrame, *, front: bool, resamples: int, seed: int) -> list[float]:
    if frame.trial_id.nunique() < 4 or resamples <= 0:
        return []
    groups = [part for _, part in frame.groupby('trial_id', sort=False)]
    rng = np.random.default_rng(seed)
    values: list[float] = []
    alpha_column, force_column = ('alpha_front_rad', 'fy_front_N') if front else ('alpha_rear_rad', 'fy_rear_N')
    for _ in range(resamples):
        sample = pd.concat([groups[index] for index in rng.integers(0, len(groups), size=len(groups))], ignore_index=True)
        alpha = sample[alpha_column].to_numpy(float)
        force = sample[force_column].to_numpy(float)
        denominator = float(np.dot(alpha, alpha))
        if denominator > 1e-12:
            value = float(np.dot(alpha, force) / denominator)
            if math.isfinite(value):
                values.append(value)
    return [float(x) for x in np.quantile(np.asarray(values), [0.025, 0.975])] if values else []


def bootstrap_front_scale(frame: pd.DataFrame, *, resamples: int, seed: int) -> dict[str, list[float]]:
    """Trial-resampled uncertainty for the coupled front stiffness/scale fit."""
    if frame.trial_id.nunique() < 4 or resamples <= 0:
        return {'front_cornering_stiffness_N_per_rad': [], 'steering_model_scale': []}
    groups = [part for _, part in frame.groupby('trial_id', sort=False)]
    rng = np.random.default_rng(seed)
    stiffness_values: list[float] = []
    scale_values: list[float] = []
    for _ in range(resamples):
        sample = pd.concat([groups[index] for index in rng.integers(0, len(groups), size=len(groups))], ignore_index=True)
        try:
            fitted = fit_front_tyre_and_steering_scale(sample)
            stiffness = float(fitted['linear']['cornering_stiffness_N_per_rad'])
            scale = float(fitted['steering_model_scale_candidate'])
        except (KeyError, TypeError, ValueError, np.linalg.LinAlgError):
            continue
        if math.isfinite(stiffness):
            stiffness_values.append(stiffness)
        if math.isfinite(scale):
            scale_values.append(scale)
    return {
        'front_cornering_stiffness_N_per_rad': [float(x) for x in np.quantile(stiffness_values, [0.025, 0.975])] if stiffness_values else [],
        'steering_model_scale': [float(x) for x in np.quantile(scale_values, [0.025, 0.975])] if scale_values else [],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    cfg = load_yaml(session / 'calibration_config_snapshot.yaml')
    out = analysis_dir(session)
    physical = _physical(session)
    stationary_imu_diagnostic = _stationary_imu_diagnostic(session)
    # Start with a neutral scale only to form the raw quasi-steady quality
    # table. B then jointly identifies the front stiffness and model scale and
    # rewrites the trial artefact with the actual candidate used for C.
    trials = collect_lateral_trials(session, '12_quasi_steady_lateral_training', cfg, physical)
    usable = trials[trials.measurement_valid.astype(bool)].copy() if not trials.empty else trials
    spec = cfg['lateral_stiffness']
    coverage = expected_grid_coverage(
        usable, fields=['speed_command_mps', 'steering_angle_rad'], expected_grid=_expected_grid(cfg, validation=False),
        expected_repetitions=int(spec['repetitions']),
        tolerances={'speed_command_mps': 1e-6, 'steering_angle_rad': 1e-6},
    )
    coverage.to_parquet(out / 'lateral_stiffness_training_coverage.parquet', index=False)
    failures: list[str] = []
    if usable.empty or not bool(coverage.coverage_ok.all()):
        failures.append('quasi-steady lateral training coverage incomplete')
    try:
        front = fit_front_tyre_and_steering_scale(usable)
        steering_model_scale = float(front['steering_model_scale_candidate'])
        trials = apply_front_steering_scale(trials, steering_model_scale)
        usable = trials[trials.measurement_valid.astype(bool)].copy() if not trials.empty else trials
        # Refit on the candidate-scaled data retained in the report so all
        # values/plots exactly match the patch sent to the dynamic model.
        front = fit_front_tyre_and_steering_scale(usable)
        steering_model_scale = float(front['steering_model_scale_candidate'])
        trials = apply_front_steering_scale(trials, steering_model_scale)
        usable = trials[trials.measurement_valid.astype(bool)].copy() if not trials.empty else trials
        rear = fit_tyre(usable.alpha_rear_rad.to_numpy(float), usable.fy_rear_N.to_numpy(float))
    except (AttributeError, ValueError, np.linalg.LinAlgError) as exc:
        failures.append(str(exc))
        steering_model_scale = math.nan
        front = {'linear': {'cornering_stiffness_N_per_rad': math.nan, 'metrics': {}},
                 'nonlinear': {'metrics': {}}, 'design_condition_number': math.inf,
                 'steering_model_scale_candidate': math.nan}
        rear = {'linear': {'cornering_stiffness_N_per_rad': math.nan, 'metrics': {}},
                'nonlinear': {'metrics': {}}, 'design_condition_number': math.inf}
    policy = cfg.get('analysis', {}).get('lateral_stiffness', {})
    runtime_shape = float(spec.get('runtime_pacejka_shape_factor', 1.9))
    for fit, alpha_column, force_column in (
        (front, 'alpha_front_rad', 'fy_front_N'),
        (rear, 'alpha_rear_rad', 'fy_rear_N'),
    ):
        stiffness = float(fit.get('linear', {}).get('cornering_stiffness_N_per_rad', math.nan))
        alpha = usable[alpha_column].to_numpy(float) if not usable.empty else np.empty(0)
        force = usable[force_column].to_numpy(float) if not usable.empty else np.empty(0)
        fit['runtime_bounded'] = {
            'cornering_stiffness_N_per_rad': stiffness,
            'pacejka_shape_factor': runtime_shape,
            'model_form': 'Fy = C_alpha*sin(shape*atan(alpha/shape))',
            'metrics': _metrics(force, bounded_tyre_prediction(alpha, stiffness, runtime_shape)),
            'shape_factor_identified': False,
        }
    statistics_policy = cfg.get('analysis', {}).get('statistics', {})
    try:
        turn_slip = fit_cornering_longitudinal_slip(
            trials,
            clip_fraction=float(policy.get('turn_slip_clip_fraction', 0.25)),
            min_coefficient=float(policy.get('turn_slip_min_coefficient_per_mps2', 0.002)),
            min_improvement_fraction=float(policy.get('turn_slip_min_training_rmse_improvement_fraction', 0.05)),
            bootstrap_resamples=int(statistics_policy.get('bootstrap_resamples', 1000)),
            seed=int(statistics_policy.get('bootstrap_seed', 20260717)) + 31,
        )
        trials = add_turn_slip_predictions(trials, turn_slip)
    except (AttributeError, KeyError, TypeError, ValueError, np.linalg.LinAlgError) as exc:
        failures.append(str(exc))
        turn_slip = {
            'selected_coefficient_per_mps2': 0.0,
            'clip_fraction': float(policy.get('turn_slip_clip_fraction', 0.25)),
            'correction_active': False,
            'accepted_for_candidate': False,
            'failure': str(exc),
        }
    trials.to_parquet(out / 'lateral_stiffness_training_trials.parquet', index=False)
    for axle, fit in (('front', front), ('rear', rear)):
        stiffness = float(fit['linear'].get('cornering_stiffness_N_per_rad', math.nan))
        r2 = float(fit['linear'].get('metrics', {}).get('r2', math.nan))
        if not math.isfinite(stiffness) or stiffness <= 0.0:
            failures.append(f'{axle} linear cornering stiffness is not positive')
        if not math.isfinite(r2) or r2 < float(policy.get('min_training_linear_r2', 0.20)):
            failures.append(f'{axle} linear tyre fit R² is below the training gate')
        if float(fit.get('design_condition_number', math.inf)) > float(policy.get('max_design_condition_number', 2e4)):
            failures.append(f'{axle} tyre-fit design matrix is poorly conditioned')
    if not math.isfinite(steering_model_scale):
        failures.append('front steering-model scale is not finite')
    elif not float(policy.get('min_steering_model_scale', 0.5)) <= steering_model_scale <= float(policy.get('max_steering_model_scale', 1.5)):
        failures.append(
            f'steering-model scale {steering_model_scale:.4f} is outside the configured identifiable range '
            f'[{float(policy.get("min_steering_model_scale", 0.5)):.3f}, '
            f'{float(policy.get("max_steering_model_scale", 1.5)):.3f}]'
        )
    cf = float(front['linear'].get('cornering_stiffness_N_per_rad', math.nan))
    cr = float(rear['linear'].get('cornering_stiffness_N_per_rad', math.nan))
    mass, wheelbase = float(physical['mass_kg']), float(physical['wheelbase_m'])
    lf, lr = float(physical['cg_to_front_axle_lf_m']), float(physical['cg_to_rear_axle_lr_m'])
    understeer = mass / wheelbase * (lr / cf - lf / cr) if all(math.isfinite(x) and x > 0.0 for x in (cf, cr)) else math.nan
    boot_count = int(policy.get('bootstrap_resamples', 200))
    front_bootstrap = bootstrap_front_scale(usable, resamples=boot_count, seed=20260713)
    report = {
        'model_scope': (
            'Effective low-slip, quasi-steady bicycle-model cornering stiffness and dynamic-model steering scale '
            'for the measured tyre/surface/load state; not universal component constants.'
        ),
        'physical_parameters': {
            'mass_kg': mass, 'wheelbase_m': wheelbase, 'lf_m': lf, 'lr_m': lr,
        },
        'stationary_imu_diagnostic': stationary_imu_diagnostic,
        'front_tyre': front, 'rear_tyre': rear,
        'runtime_lateral_tyre_model': {
            'form': 'bounded Pacejka form with the fitted small-angle C_alpha',
            'pacejka_shape_factor': runtime_shape,
            'shape_factor_identified': False,
            'note': 'The safe low-slip campaign validates adequacy over its arc envelope; it does not identify a peak-force shape.',
        },
        'steering_model_scale_candidate': steering_model_scale,
        'steering_model_scale_scope': (
            'Scale applied after the LiDAR-derived static steering map inside vesc_to_odom_node; '
            'identified jointly with front stiffness and frozen for the independent lateral hold-out.'
        ),
        'linear_understeer_gradient_rad_per_mps2': understeer,
        'bootstrap_linear_cornering_stiffness_95pct_N_per_rad': {
            'front': front_bootstrap['front_cornering_stiffness_N_per_rad'],
            'rear': bootstrap_stiffness(usable, front=False, resamples=boot_count, seed=20260714),
        },
        'bootstrap_steering_model_scale_95pct': front_bootstrap['steering_model_scale'],
        'cornering_longitudinal_slip': turn_slip,
        'coverage_ok': bool(len(coverage)) and bool(coverage.coverage_ok.all()),
        'accepted_for_candidate': not failures,
        'failures': failures,
        'validation_note': 'The next stage uses distinct speeds and steering angles and evaluates these frozen linear/nonlinear candidates without refitting.',
    }
    dump_yaml(out / 'lateral_stiffness_training_report.yaml', report)
    print(json.dumps(report, indent=2))
    if failures:
        raise SystemExit('lateral tyre-stiffness training rejected: ' + '; '.join(failures))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
