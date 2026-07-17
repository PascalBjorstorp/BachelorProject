#!/usr/bin/env python3
"""Identify full-envelope current/traction/slip behaviour.

A single ``accel_to_current_gain`` is fitted only from low-slip, low-current
pulses.  The full speed/current grid is then used to test whether that scalar
remains adequate.  When it does not, the script emits a speed/current traction
surface and rejects automatic deployment of the scalar-only candidate.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd
from common import (
    accepted_capture_windows,
    analysis_dir,
    dump_yaml,
    eval_origin_quadratic,
    expected_grid_coverage,
    load_yaml,
    motion_windows,
    stage_tables,
)


def _drag(v: np.ndarray, report: dict) -> np.ndarray:
    return (
        float(report['accel_drag_coulomb_mps2'])
        + float(report['accel_drag_viscous_per_s']) * np.abs(v)
        + float(report['accel_drag_quadratic_per_m']) * v * v
    )


def _mean_accel(lidar: pd.DataFrame, start: int, end: int, cfg: dict) -> tuple[float, float, int]:
    """Estimate pulse acceleration from robust LiDAR *window* motion samples.

    The old derivative required eight overlapping scan-pair rows and therefore
    failed at the low-speed end of a 40 Hz sensor.  The new estimator fits one
    slope to independently quality-gated multi-registration windows after
    removing command-edge transients.  It is deliberately one measurement per
    pulse, not dozens of falsely independent scan-pair accelerations.
    """
    frame = lidar[(lidar.bag_ns >= start) & (lidar.bag_ns <= end)].copy()
    if frame.empty or 'valid' not in frame:
        return math.nan, math.nan, 0
    frame = frame[frame.valid.astype(bool)].sort_values('bag_ns')
    policy = cfg['analysis'].get('current_pulse', {})
    minimum = int(policy.get('min_window_samples', 4))
    if len(frame) < minimum:
        return math.nan, math.nan, len(frame)
    lo = int(len(frame) * float(policy.get('trim_fraction', 0.15)))
    hi = len(frame) - lo
    frame = frame.iloc[lo:hi]
    if len(frame) < minimum:
        return math.nan, math.nan, len(frame)
    t = frame.bag_ns.to_numpy(dtype=float) * 1e-9
    v = frame.vx.to_numpy(dtype=float)
    if float(t[-1] - t[0]) < float(policy.get('min_effective_pulse_s', 0.40)):
        return math.nan, math.nan, len(frame)
    slope, _ = np.polyfit(t - np.mean(t), v, 1)
    return float(slope), float(np.nanmedian(v)), len(frame)


def _phase_window(events: pd.DataFrame, trial_id: str, phase: str) -> tuple[int, int] | None:
    starts = events[(events.event.astype(str) == 'phase_start') &
                    (events.phase.astype(str) == phase) &
                    (events.trial_id.astype(str) == trial_id)]
    if starts.empty:
        return None
    start = starts.sort_values('bag_ns').iloc[-1]
    ends = events[(events.event.astype(str) == 'phase_end') &
                  (events.phase.astype(str) == phase) &
                  (events.trial_id.astype(str) == trial_id) &
                  (events.bag_ns > start.bag_ns)]
    if ends.empty:
        return None
    return int(start.bag_ns), int(ends.sort_values('bag_ns').iloc[0].bag_ns)


def _median_speed(lidar: pd.DataFrame, window: tuple[int, int] | None) -> float:
    if window is None:
        return math.nan
    frame = lidar[(lidar.bag_ns >= window[0]) & (lidar.bag_ns <= window[1])].copy()
    if frame.empty or 'valid' not in frame:
        return math.nan
    frame = frame[frame.valid.astype(bool)]
    return float(np.nanmedian(frame.vx.to_numpy(dtype=float))) if len(frame) else math.nan


def _pulse_summary(session: Path, stage: str, phases: list[str], cfg: dict,
                   drag_report: dict, static_report: dict) -> pd.DataFrame:
    tables = stage_tables(session, stage)
    windows = accepted_capture_windows(tables['events'], phases)
    lidar = motion_windows(tables)
    direct = static_report['measured_erpm_odometry_map']['direct_erpm_to_ground_speed_candidate']
    speed_floor = float(cfg['analysis']['slip']['speed_floor_mps'])
    rows: list[dict[str, object]] = []
    for _, window in windows.iterrows():
        accel, speed, n = _mean_accel(lidar, int(window.start_ns), int(window.end_ns), cfg)
        vesc = tables['vesc'][(tables['vesc'].bag_ns >= window.start_ns) & (tables['vesc'].bag_ns <= window.end_ns)]
        imu = tables['imu'][(tables['imu'].bag_ns >= window.start_ns) & (tables['imu'].bag_ns <= window.end_ns)]
        selected_current = tables['motor_selected_current'][(tables['motor_selected_current'].bag_ns >= window.start_ns) & (tables['motor_selected_current'].bag_ns <= window.end_ns)]
        selected_brake = tables['motor_selected_brake'][(tables['motor_selected_brake'].bag_ns >= window.start_ns) & (tables['motor_selected_brake'].bag_ns <= window.end_ns)]
        baseline = _phase_window(tables['events'], str(window.trial_id), 'current_pulse_baseline')
        recovery = _phase_window(tables['events'], str(window.trial_id), f'raw_{str(window.get("polarity", ""))}_current_recovery')
        baseline_speed = _median_speed(lidar, baseline)
        recovery_speed = _median_speed(lidar, recovery)
        if (not math.isfinite(accel) or not math.isfinite(speed) or n < int(cfg['analysis'].get('current_pulse', {}).get('min_window_samples', 4))
                or not math.isfinite(baseline_speed) or not math.isfinite(recovery_speed)
                or abs(speed) < float(cfg['analysis'].get('min_lidar_speed_mps', 0.3))):
            continue
        yaw = float(np.nanmedian(imu.gz)) if len(imu) else math.nan
        if math.isfinite(yaw) and abs(yaw) > float(cfg['analysis']['max_straight_yaw_rate_rad_s']):
            continue
        erpm = float(np.nanmedian(vesc.erpm)) if len(vesc) else math.nan
        motor_current = float(np.nanmedian(vesc.motor_current)) if len(vesc) else math.nan
        input_current = float(np.nanmedian(vesc.input_current)) if len(vesc) else math.nan
        wheel_equiv = float(eval_origin_quadratic(
            np.asarray([erpm]),
            float(direct['gain_mps_per_erpm']),
            float(direct['quadratic_mps_per_erpm2']),
        )[0]) if math.isfinite(erpm) else math.nan
        slip_ratio = (wheel_equiv - speed) / max(abs(speed), speed_floor) if math.isfinite(wheel_equiv) else math.nan
        polarity = str(window.get('polarity', ''))
        command = float(window.get('current_command_a', math.nan))
        fraction = float(window.get('current_fraction', math.nan))
        initial_speed = float(window.get('initial_speed_mps', math.nan))
        selected_drive = float(np.nanmedian(selected_current.value.to_numpy(float))) if len(selected_current) else math.nan
        selected_braking = float(np.nanmedian(selected_brake.value.to_numpy(float))) if len(selected_brake) else math.nan
        selected_actuation = selected_drive if polarity == 'drive' else selected_braking if polarity == 'brake' else math.nan
        selection_tolerance = float(cfg.get('preflight', {}).get('raw_motor_tolerance_current_a', 0.25))
        if (
            not math.isfinite(selected_actuation)
            or not math.isfinite(command)
            or abs(selected_actuation - command) > selection_tolerance
        ):
            # Do not turn a selector-routing fault into a false drivetrain
            # calibration. Missing usable rows will explicitly fail coverage.
            continue
        net_accel = accel + float(_drag(np.asarray([speed]), drag_report)[0]) if polarity == 'drive' else -accel - float(_drag(np.asarray([speed]), drag_report)[0])
        rows.append({
            'trial_id': str(window.trial_id),
            'condition_id': str(window.condition_id),
            'polarity': polarity,
            'current_command_a': command,
            'selected_drive_current_a': selected_drive,
            'selected_brake_current_a': selected_braking,
            'selected_actuation_a': selected_actuation,
            'selected_actuation_minus_command_a': selected_actuation - command,
            'current_fraction': fraction,
            'initial_speed_mps': initial_speed,
            'motor_current_a': motor_current,
            'input_current_a': input_current,
            'erpm_measured': erpm,
            'wheel_equivalent_speed_mps': wheel_equiv,
            'longitudinal_slip_ratio': slip_ratio,
            'ax_lidar_mps2': accel,
            'vx_lidar_mps': speed,
            'drag_mps2': float(_drag(np.asarray([speed]), drag_report)[0]),
            'net_accel_mps2': net_accel,
            'sample_count': n,
            'baseline_vx_lidar_mps': baseline_speed,
            'recovery_vx_lidar_mps': recovery_speed,
            'pulse_delta_v_mps': recovery_speed - baseline_speed,
            'measurement_method': 'robust_window_velocity_slope_over_trimmed_current_pulse',
        })
    return pd.DataFrame(rows)


def _grid(cfg: dict, section: str, polarity: str) -> list[tuple[object, ...]]:
    """Expected current conditions, keyed by nominal schedule not telemetry."""
    spec = cfg[section]
    entries = spec.get(f'{polarity}_conditions')
    if entries is None:
        speeds = spec[f'{polarity}_initial_speeds_mps']
        fractions = spec[f'{polarity}_current_fractions']
        entries = [{'initial_speed_mps': speed, 'current_fractions': fractions} for speed in speeds]
    return [
        (polarity, float(entry['initial_speed_mps']), float(fraction))
        for entry in entries
        for fraction in entry['current_fractions']
    ]


def _coverage(frame: pd.DataFrame, cfg: dict, section: str, polarity: str) -> pd.DataFrame:
    subset = frame[frame.polarity.astype(str) == polarity].copy() if not frame.empty else frame
    return expected_grid_coverage(
        subset,
        fields=['polarity', 'initial_speed_mps', 'current_fraction'],
        expected_grid=_grid(cfg, section, polarity),
        expected_repetitions=int(cfg[section]['repetitions']),
        tolerances={'initial_speed_mps': 1e-6, 'current_fraction': 1e-6},
    )


def _linear_fit(frame: pd.DataFrame, polarity: str, cfg: dict) -> dict:
    part = frame[frame.polarity.astype(str) == polarity].copy()
    if part.empty:
        raise ValueError(f'no {polarity} current trials')
    # ACCEL_TO_CURRENT publishes into the selector, so the deployable gain must
    # be fitted against the *selected command* rather than VESC motor-current
    # feedback. The latter is retained as a drivetrain diagnostic but can be
    # scaled/limited by the controller and would make the runtime gain wrong.
    current = part.selected_actuation_a.to_numpy(dtype=float)
    y = part.net_accel_mps2.to_numpy(dtype=float)
    slip_limit = float(cfg['analysis']['slip']['high_slip_ratio_threshold'])
    low = (part.current_fraction.to_numpy(dtype=float) <= 0.33) & (np.abs(part.longitudinal_slip_ratio.to_numpy(dtype=float)) <= slip_limit)
    if int(low.sum()) < 5:
        low = np.isfinite(current) & np.isfinite(y)
    x = current[low]
    yy = y[low]
    slope = float(np.dot(x, yy) / max(np.dot(x, x), 1e-12))
    pred = slope * current
    residual = y - pred
    denom = float(np.sum((y - np.mean(y)) ** 2))
    r2 = float(1.0 - np.sum(residual ** 2) / denom) if denom > 1e-12 else math.nan
    return {
        'accel_per_amp': slope,
        'gain_a_per_mps2': 1.0 / slope if slope > 1e-9 else math.inf,
        'training_low_slip_samples': int(low.sum()),
        'full_training_rmse_mps2': float(np.sqrt(np.nanmean(residual ** 2))),
        'full_training_bias_mps2': float(np.nanmean(residual)),
        'full_training_r2': r2,
    }


def _surface_fit(frame: pd.DataFrame, polarity: str) -> dict:
    part = frame[frame.polarity.astype(str) == polarity].copy()
    if part.empty:
        raise ValueError(f'no {polarity} current trials')
    current = part.selected_actuation_a.to_numpy(dtype=float)
    speed = np.maximum(part.vx_lidar_mps.to_numpy(dtype=float), 0.0)
    y = part.net_accel_mps2.to_numpy(dtype=float)
    mask = np.isfinite(current) & np.isfinite(speed) & np.isfinite(y)
    current, speed, y = current[mask], speed[mask], y[mask]
    if len(y) < 8:
        raise ValueError(f'insufficient {polarity} rows for high-demand surface')
    # Through-origin in current: at I=0, net drive/brake acceleration is zero.
    # Terms allow saturation/efficiency change with current and speed without
    # pretending that a static ERPM quadratic describes transient tyre slip.
    X = np.column_stack([current, current * current, current * speed, current * speed * speed])
    coeff, *_ = np.linalg.lstsq(X, y, rcond=None)
    pred = X @ coeff
    residual = y - pred
    denom = float(np.sum((y - np.mean(y)) ** 2))
    r2 = float(1.0 - np.sum(residual ** 2) / denom) if denom > 1e-12 else math.nan
    return {
        'model_form': 'a_net = c1*I + c2*I^2 + c3*I*v + c4*I*v^2',
        'c1_accel_per_amp': float(coeff[0]),
        'c2_accel_per_amp2': float(coeff[1]),
        'c3_accel_per_amp_mps': float(coeff[2]),
        'c4_accel_per_amp_mps2': float(coeff[3]),
        'training_rmse_mps2': float(np.sqrt(np.mean(residual ** 2))),
        'training_bias_mps2': float(np.mean(residual)),
        'training_r2': r2,
        'n': int(len(y)),
    }


def _surface_predict(frame: pd.DataFrame, polarity: str, fit: dict) -> np.ndarray:
    current = frame.selected_actuation_a.to_numpy(dtype=float)
    speed = np.maximum(frame.vx_lidar_mps.to_numpy(dtype=float), 0.0)
    return (
        float(fit['c1_accel_per_amp']) * current
        + float(fit['c2_accel_per_amp2']) * current * current
        + float(fit['c3_accel_per_amp_mps']) * current * speed
        + float(fit['c4_accel_per_amp_mps2']) * current * speed * speed
    )


def _metrics(actual: np.ndarray, pred: np.ndarray) -> dict:
    r = np.asarray(actual, dtype=float) - np.asarray(pred, dtype=float)
    r = r[np.isfinite(r)]
    if not len(r):
        return {'rmse_mps2': math.inf, 'bias_mps2': math.nan, 'n': 0}
    return {'rmse_mps2': float(np.sqrt(np.mean(r ** 2))), 'bias_mps2': float(np.mean(r)), 'n': int(len(r))}


def _slip_summary(frame: pd.DataFrame, cfg: dict, polarity: str) -> dict:
    part = frame[frame.polarity.astype(str) == polarity].copy()
    threshold = float(cfg['analysis']['slip']['high_slip_ratio_threshold'])
    values = np.abs(part.longitudinal_slip_ratio.to_numpy(dtype=float)) if not part.empty else np.empty(0)
    values = values[np.isfinite(values)]
    return {
        'n': int(len(values)),
        'median_abs_slip_ratio': float(np.median(values)) if len(values) else math.nan,
        'p95_abs_slip_ratio': float(np.quantile(values, 0.95)) if len(values) else math.nan,
        'high_slip_fraction': float(np.mean(values > threshold)) if len(values) else math.nan,
        'high_slip_threshold': threshold,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    cfg = load_yaml(session / 'calibration_config_snapshot.yaml')
    out = analysis_dir(session)
    import yaml
    drag = yaml.safe_load((out / 'coastdown_drag_report.yaml').read_text(encoding='utf-8'))
    static = yaml.safe_load((out / 'erpm_speed_map_report.yaml').read_text(encoding='utf-8'))
    training_candidate_path = out / 'current_acceleration_training_report.yaml'
    training_candidate = (
        yaml.safe_load(training_candidate_path.read_text(encoding='utf-8')) or {}
        if training_candidate_path.exists() else {}
    )

    train = _pulse_summary(session, '08_raw_current_training', ['raw_drive_current_pulse', 'raw_brake_current_pulse'], cfg, drag, static)
    hold = _pulse_summary(session, '09_raw_current_holdout', ['raw_drive_current_pulse', 'raw_brake_current_pulse'], cfg, drag, static)
    train.to_parquet(out / 'current_model_training_trials.parquet', index=False)
    hold.to_parquet(out / 'current_model_holdout_trials.parquet', index=False)

    coverage_reports: dict[str, dict[str, bool]] = {}
    for polarity in ('drive', 'brake'):
        train_cov = _coverage(train, cfg, 'raw_current_training', polarity)
        hold_cov = _coverage(hold, cfg, 'raw_current_holdout', polarity)
        train_cov.to_parquet(out / f'current_{polarity}_training_coverage.parquet', index=False)
        hold_cov.to_parquet(out / f'current_{polarity}_holdout_coverage.parquet', index=False)
        coverage_reports[polarity] = {
            'training_ok': bool(len(train_cov)) and bool(train_cov.coverage_ok.all()),
            'holdout_ok': bool(len(hold_cov)) and bool(hold_cov.coverage_ok.all()),
        }

    linear: dict[str, dict] = {}
    surface: dict[str, dict] = {}
    holdout: dict[str, dict] = {}
    slip: dict[str, dict] = {}
    nonlinear_required: dict[str, bool] = {}
    for polarity in ('drive', 'brake'):
        linear[polarity] = _linear_fit(train, polarity, cfg)
        surface[polarity] = _surface_fit(train, polarity)
        h = hold[hold.polarity.astype(str) == polarity].copy()
        if h.empty:
            linear_metrics = {'rmse_mps2': math.inf, 'bias_mps2': math.nan, 'n': 0}
            surface_metrics = {'rmse_mps2': math.inf, 'bias_mps2': math.nan, 'n': 0}
        else:
            current = h.selected_actuation_a.to_numpy(dtype=float)
            linear_metrics = _metrics(h.net_accel_mps2.to_numpy(dtype=float), float(linear[polarity]['accel_per_amp']) * current)
            surface_metrics = _metrics(h.net_accel_mps2.to_numpy(dtype=float), _surface_predict(h, polarity, surface[polarity]))
        holdout[polarity] = {'linear_scalar': linear_metrics, 'surface': surface_metrics}
        slip[polarity] = _slip_summary(hold, cfg, polarity)
        improve = float(cfg['analysis']['slip']['nonlinear_model_min_holdout_improvement_fraction'])
        surface_better = surface_metrics['rmse_mps2'] <= linear_metrics['rmse_mps2'] * (1.0 - improve)
        high_slip = math.isfinite(slip[polarity]['high_slip_fraction']) and slip[polarity]['high_slip_fraction'] > float(cfg['analysis']['slip']['max_high_demand_slip_fraction_for_linear_candidate'])
        nonlinear_required[polarity] = bool(surface_better or high_slip)

    gates = cfg['analysis']['gates']
    surface_ok = all(
        coverage_reports[p]['training_ok'] and coverage_reports[p]['holdout_ok']
        and surface[p]['training_r2'] >= float(gates['min_current_model_r2'])
        and holdout[p]['surface']['rmse_mps2'] <= float(gates['max_current_model_holdout_rmse_mps2'])
        and abs(float(holdout[p]['surface']['bias_mps2'])) <= float(gates['max_current_model_holdout_bias_mps2'])
        for p in ('drive', 'brake')
    )
    scalar_adequate = not any(nonlinear_required.values())
    accel_noise = float(np.nanstd(train.ax_lidar_mps2.to_numpy(dtype=float))) if not train.empty else math.nan
    candidate_deadzone = max(0.02, min(0.20, 0.25 * accel_noise)) if math.isfinite(accel_noise) else 0.05

    traction = {
        'model_scope': 'Full-envelope empirical longitudinal net-acceleration surface. Use this for MPC/traction modelling when the scalar ACCEL_TO_CURRENT map is rejected.',
        'drive': surface['drive'],
        'brake': surface['brake'],
        'holdout': holdout,
        'slip': slip,
        'coverage': coverage_reports,
        'scalar_accel_to_current_adequate_over_envelope': scalar_adequate,
        'requires_nonlinear_longitudinal_model': not scalar_adequate,
    }
    dump_yaml(out / 'longitudinal_traction_surface.yaml', traction)

    report = {
        'low_slip_scalar_fit': linear,
        'full_envelope_surface_fit': surface,
        'holdout': holdout,
        'slip': slip,
        'coverage': coverage_reports,
        'candidate_accel_to_current_gain': float(linear['drive']['gain_a_per_mps2']),
        'candidate_accel_to_brake_gain': float(linear['brake']['gain_a_per_mps2']),
        'candidate_gain_bootstrap': training_candidate.get('candidate_gain_bootstrap', {}),
        'candidate_accel_deadzone_mps2': candidate_deadzone,
        'scalar_accel_to_current_adequate_over_envelope': scalar_adequate,
        'requires_nonlinear_longitudinal_model': not scalar_adequate,
        'accepted_for_candidate': bool(surface_ok and scalar_adequate),
        'gates': dict(gates),
        'notes': [
            'The scalar gain is fitted from low-slip selected-current command data, matching the ACCEL_TO_CURRENT runtime input. VESC motor current is retained as a diagnostic, not substituted into the deployable command gain.',
            'If requires_nonlinear_longitudinal_model=true, automatic deployment is rejected. The high-demand traction surface remains the evidence needed for a subsequent MPC/actuation-model upgrade.',
        ],
    }
    dump_yaml(out / 'current_acceleration_report.yaml', report)
    if not scalar_adequate:
        dump_yaml(out / 'nonlinear_longitudinal_actuation_model_request.yaml', {
            'required': True,
            'reason': 'independent current hold-out materially prefers the speed/current surface or shows excessive high-demand slip',
            'polarity_requires_nonlinear_model': nonlinear_required,
            'evidence': {
                'scalar_and_surface_holdout': holdout,
                'slip': slip,
                'fitted_surface': surface,
            },
            'next_action': (
                'Implement a bounded speed-dependent current/acceleration model in the racing actuation path, '
                'then redo current_training and current_holdout before continuing.'
            ),
        })
    print(json.dumps(report, indent=2, default=str))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
