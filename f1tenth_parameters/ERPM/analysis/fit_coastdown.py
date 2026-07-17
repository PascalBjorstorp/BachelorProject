#!/usr/bin/env python3
"""Fit zero-current drag from robust LiDAR-window velocity trajectories.

The previous implementation differentiated overlapping ICP registrations.  That
turns correlated LiDAR noise into acceleration noise and lets one short trial
look like many independent measurements.  This fitter instead integrates the
drag ODE over every accepted trial and optimises the directly measured velocity
trajectory, as recommended for longitudinal identification in the supplied
reference thesis.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
from scipy.integrate import solve_ivp
from scipy.optimize import least_squares

from common import accepted_capture_windows, analysis_dir, dump_yaml, expected_numeric_coverage, load_yaml, motion_windows, stage_tables


def _sub(frame: pd.DataFrame, start: int, end: int) -> pd.DataFrame:
    return frame[(frame.bag_ns >= start) & (frame.bag_ns <= end)].copy() if not frame.empty else frame.copy()


def collect_coastdown_trials(session: Path, stage: str, cfg: dict[str, Any]) -> tuple[pd.DataFrame, pd.DataFrame]:
    """Return one row per usable trajectory and its robust LiDAR window samples."""
    tables = stage_tables(session, stage)
    windows = accepted_capture_windows(tables['events'], 'coastdown')
    lidar_source = motion_windows(tables)
    policy = cfg.get('analysis', {}).get('coastdown', {})
    minimum_windows = int(policy.get('min_window_samples', 6))
    trim_fraction = float(policy.get('trim_fraction', 0.05))
    rows: list[dict[str, Any]] = []
    samples: list[dict[str, Any]] = []
    for _, window in windows.iterrows():
        start, end = int(window.start_ns), int(window.end_ns)
        lidar_all = _sub(lidar_source, start, end)
        lidar = lidar_all[lidar_all.valid.astype(bool)].sort_values('bag_ns').copy() if not lidar_all.empty and 'valid' in lidar_all else lidar_all.iloc[0:0].copy()
        vesc = _sub(tables['vesc'], start, end)
        imu = _sub(tables['imu'], start, end)
        if len(lidar):
            finite = np.isfinite(lidar.vx.to_numpy(float)) & (lidar.vx.to_numpy(float) >= float(cfg['analysis']['min_lidar_speed_mps']))
            lidar = lidar.loc[finite].copy()
        trim = int(math.floor(len(lidar) * trim_fraction))
        if trim and len(lidar) > 2 * trim:
            lidar = lidar.iloc[trim:-trim].copy()
        current_ok = bool(len(vesc)) and abs(float(np.nanmedian(vesc.motor_current.to_numpy(float)))) <= float(cfg['coastdown']['max_abs_motor_current_a'])
        yaw = float(np.nanmedian(imu.gz.to_numpy(float))) if len(imu) else math.nan
        straight_ok = not len(imu) or (math.isfinite(yaw) and abs(yaw) <= float(cfg['analysis']['max_straight_yaw_rate_rad_s']))
        valid = bool(current_ok and straight_ok and len(lidar) >= minimum_windows)
        initial = float(window.get('initial_speed_mps', math.nan))
        row = {
            'trial_id': str(window.trial_id),
            'condition_id': str(window.condition_id),
            'initial_speed_mps': initial,
            'measurement_valid': valid,
            'valid_window_count': int(len(lidar)),
            'raw_window_count': int(len(lidar_all)),
            'zero_current_ok': current_ok,
            'straight_ok': straight_ok,
            'imu_yaw_rate_median_rad_s': yaw,
        }
        rows.append(row)
        if not valid:
            continue
        t = (lidar.bag_ns.to_numpy(float) - start) * 1.0e-9
        v = lidar.vx.to_numpy(float)
        for tt, vv in zip(t, v):
            samples.append({
                'trial_id': str(window.trial_id), 'condition_id': str(window.condition_id),
                'initial_speed_mps': initial, 't_s': float(tt), 'vx_mps': float(vv),
            })
    return pd.DataFrame(rows), pd.DataFrame(samples)


def _trajectory_prediction(params: np.ndarray, t: np.ndarray, v0: float) -> np.ndarray:
    """Integrate dv/dt=-(c0+c1*v+c2*v²), preserving measured trial start."""
    if len(t) < 2 or not np.isfinite(v0):
        return np.full(len(t), math.nan)
    unique, index = np.unique(t, return_index=True)
    if len(unique) < 2:
        return np.full(len(t), v0)
    def rhs(_time: float, value: np.ndarray) -> list[float]:
        speed = max(0.0, float(value[0]))
        return [-(float(params[0]) + float(params[1]) * speed + float(params[2]) * speed * speed)]
    try:
        result = solve_ivp(rhs, (float(unique[0]), float(unique[-1])), [float(v0)], t_eval=unique,
                           rtol=2e-5, atol=2e-6, max_step=0.04)
    except Exception:
        return np.full(len(t), math.nan)
    if not result.success or len(result.y) == 0:
        return np.full(len(t), math.nan)
    predicted_unique = result.y[0]
    # `t` is ordered, but use interpolation so this helper is safe for any
    # caller and does not accidentally change the sample count/weighting.
    return np.interp(t, unique, predicted_unique)


def _trial_groups(samples: pd.DataFrame) -> list[pd.DataFrame]:
    return [part.sort_values('t_s').copy() for _, part in samples.groupby('trial_id', sort=False)] if not samples.empty else []


def fit_drag(samples: pd.DataFrame, *, initial: np.ndarray | None = None) -> tuple[np.ndarray, dict[str, float]]:
    groups = _trial_groups(samples)
    if len(groups) < 2:
        raise ValueError('at least two accepted coast-down trajectories are required')
    def residual(params: np.ndarray) -> np.ndarray:
        values: list[np.ndarray] = []
        for group in groups:
            t = group.t_s.to_numpy(float)
            observed = group.vx_mps.to_numpy(float)
            prediction = _trajectory_prediction(params, t, float(observed[0]))
            delta = prediction - observed
            values.append(np.where(np.isfinite(delta), delta, 1e3))
        return np.concatenate(values)
    start = np.asarray(initial if initial is not None else [0.35, 0.10, 0.01], dtype=float)
    result = least_squares(residual, x0=np.clip(start, 0.0, 15.0), bounds=(np.zeros(3), np.full(3, 15.0)),
                           loss='soft_l1', f_scale=0.05, max_nfev=1000)
    residuals = residual(result.x)
    observed = samples.vx_mps.to_numpy(float)
    denom = float(np.sum((observed - np.mean(observed)) ** 2))
    info = {
        'trajectory_rmse_mps': float(np.sqrt(np.mean(residuals ** 2))),
        'trajectory_bias_mps': float(np.mean(residuals)),
        'trajectory_r2': float(1.0 - np.sum(residuals ** 2) / denom) if denom > 1e-12 else math.nan,
        'n_velocity_windows': int(len(observed)),
        'n_trajectories': int(len(groups)),
        'optimizer_success': bool(result.success),
        'jacobian_condition_number': float(np.linalg.cond(result.jac)) if result.jac.size else math.inf,
    }
    return result.x, info


def bootstrap_drag(samples: pd.DataFrame, fit: np.ndarray, cfg: dict[str, Any]) -> dict[str, list[float] | int]:
    policy = cfg.get('analysis', {}).get('coastdown', {})
    count = int(policy.get('bootstrap_resamples', 100))
    groups = _trial_groups(samples)
    if len(groups) < 3 or count <= 0:
        return {'resamples_requested': count, 'resamples_valid': 0, 'c0_95pct': [], 'c1_95pct': [], 'c2_95pct': []}
    rng = np.random.default_rng(int(policy.get('bootstrap_seed', 20260713)))
    estimates: list[np.ndarray] = []
    for _ in range(count):
        selected = [groups[index] for index in rng.integers(0, len(groups), size=len(groups))]
        sample = pd.concat(selected, ignore_index=True)
        try:
            estimate, _ = fit_drag(sample, initial=fit)
        except (ValueError, ArithmeticError):
            continue
        if np.all(np.isfinite(estimate)):
            estimates.append(estimate)
    if not estimates:
        return {'resamples_requested': count, 'resamples_valid': 0, 'c0_95pct': [], 'c1_95pct': [], 'c2_95pct': []}
    values = np.asarray(estimates, dtype=float)
    return {
        'resamples_requested': count,
        'resamples_valid': int(len(values)),
        'c0_95pct': [float(x) for x in np.quantile(values[:, 0], [0.025, 0.975])],
        'c1_95pct': [float(x) for x in np.quantile(values[:, 1], [0.025, 0.975])],
        'c2_95pct': [float(x) for x in np.quantile(values[:, 2], [0.025, 0.975])],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    cfg = load_yaml(session / 'calibration_config_snapshot.yaml')
    out = analysis_dir(session)
    trials, samples = collect_coastdown_trials(session, '07_coastdown', cfg)
    trials.to_parquet(out / 'coastdown_trials.parquet', index=False)
    samples.to_parquet(out / 'coastdown_samples.parquet', index=False)
    usable = trials[trials.measurement_valid.astype(bool)].copy() if not trials.empty else trials
    coverage = expected_numeric_coverage(usable, 'initial_speed_mps', cfg['coastdown']['initial_speeds_mps'], int(cfg['coastdown']['repetitions']))
    coverage.to_parquet(out / 'coastdown_coverage.parquet', index=False)
    if usable.empty or not bool(coverage.coverage_ok.all()):
        raise SystemExit('coast-down coverage incomplete; inspect coastdown_coverage.parquet')
    try:
        coefficients, metrics = fit_drag(samples)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    bootstrap = bootstrap_drag(samples, coefficients, cfg)
    gates = cfg['analysis']['gates']
    policy = cfg.get('analysis', {}).get('coastdown', {})
    accepted = bool(
        metrics['optimizer_success']
        and math.isfinite(metrics['trajectory_r2'])
        and metrics['trajectory_r2'] >= float(gates['min_coastdown_r2'])
        and metrics['trajectory_rmse_mps'] <= float(policy.get('max_training_velocity_rmse_mps', 0.15))
        and metrics['jacobian_condition_number'] <= float(policy.get('max_jacobian_condition_number', 5e4))
    )
    report = {
        'model': 'dv/dt = -(c0 + c1*|v| + c2*v^2), fitted by integrating each robust LiDAR-window velocity trajectory',
        'accel_drag_coulomb_mps2': float(coefficients[0]),
        'accel_drag_viscous_per_s': float(coefficients[1]),
        'accel_drag_quadratic_per_m': float(coefficients[2]),
        **metrics,
        'bootstrap': bootstrap,
        'coverage_ok': bool(coverage.coverage_ok.all()),
        'accepted_for_candidate': accepted,
        'gate_min_r2': float(gates['min_coastdown_r2']),
        'gate_max_training_velocity_rmse_mps': float(policy.get('max_training_velocity_rmse_mps', 0.15)),
        'gate_max_jacobian_condition_number': float(policy.get('max_jacobian_condition_number', 5e4)),
    }
    dump_yaml(out / 'coastdown_drag_report.yaml', report)
    print(json.dumps(report, indent=2))
    if not accepted:
        raise SystemExit('coast-down drag candidate rejected; inspect coastdown_drag_report.yaml')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
