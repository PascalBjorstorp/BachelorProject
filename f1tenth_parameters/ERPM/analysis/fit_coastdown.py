#!/usr/bin/env python3
"""Identify full-envelope longitudinal drag from raw-current-zero coast-downs."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.optimize import least_squares
from scipy.signal import savgol_filter

from common import accepted_capture_windows, analysis_dir, dump_yaml, expected_numeric_coverage, load_yaml, stage_tables


def _derivative(t: np.ndarray, v: np.ndarray) -> np.ndarray:
    if len(v) < 7:
        return np.gradient(v, t)
    dt = float(np.median(np.diff(t)))
    window = max(5, int(round(0.20 / max(dt, 1e-3))))
    if window % 2 == 0:
        window += 1
    if window >= len(v):
        window = len(v) - 1 if len(v) % 2 == 0 else len(v)
    smooth = savgol_filter(v, window, 2, mode='interp') if window >= 5 else v
    return np.gradient(smooth, t)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    cfg = load_yaml(session / 'calibration_config_snapshot.yaml')
    out = analysis_dir(session)
    tables = stage_tables(session, '07_coastdown')
    windows = accepted_capture_windows(tables['events'], 'coastdown')

    trial_rows: list[dict[str, object]] = []
    samples: list[dict[str, float | str]] = []
    for _, w in windows.iterrows():
        lidar = tables['lidar_velocity'][(tables['lidar_velocity'].bag_ns >= w.start_ns) & (tables['lidar_velocity'].bag_ns <= w.end_ns)].copy()
        if lidar.empty or 'valid' not in lidar:
            continue
        lidar = lidar[lidar.valid.astype(bool)].sort_values('bag_ns')
        if len(lidar) < 10:
            continue
        t = (lidar.bag_ns.to_numpy(dtype=float) - float(w.start_ns)) * 1e-9
        v = lidar.vx.to_numpy(dtype=float)
        ax = _derivative(t, v)
        vesc = tables['vesc'][(tables['vesc'].bag_ns >= w.start_ns) & (tables['vesc'].bag_ns <= w.end_ns)]
        imu = tables['imu'][(tables['imu'].bag_ns >= w.start_ns) & (tables['imu'].bag_ns <= w.end_ns)]
        current_ok = len(vesc) and abs(float(np.nanmedian(vesc.motor_current))) <= float(cfg['coastdown']['max_abs_motor_current_a'])
        straight_ok = not len(imu) or abs(float(np.nanmedian(imu.gz))) <= float(cfg['analysis']['max_straight_yaw_rate_rad_s'])
        initial = float(w.get('initial_speed_mps', math.nan))
        keep = np.isfinite(v) & np.isfinite(ax) & (v >= float(cfg['analysis']['min_lidar_speed_mps']))
        valid = bool(current_ok and straight_ok and keep.sum() >= 8)
        trial_rows.append({
            'trial_id': str(w.trial_id),
            'condition_id': str(w.condition_id),
            'initial_speed_mps': initial,
            'measurement_valid': valid,
            'sample_count': int(keep.sum()),
        })
        if not valid:
            continue
        for tt, vv, aa in zip(t[keep], v[keep], ax[keep]):
            samples.append({'trial_id': str(w.trial_id), 't_s': float(tt), 'vx_mps': float(vv), 'ax_mps2': float(aa), 'initial_speed_mps': initial})

    trial_table = pd.DataFrame(trial_rows)
    trial_table.to_parquet(out / 'coastdown_trials.parquet', index=False)
    usable = trial_table[trial_table.measurement_valid.astype(bool)].copy() if not trial_table.empty else trial_table
    cov = expected_numeric_coverage(
        usable, 'initial_speed_mps', cfg['coastdown']['initial_speeds_mps'], int(cfg['coastdown']['repetitions'])
    )
    cov.to_parquet(out / 'coastdown_coverage.parquet', index=False)
    if usable.empty or not bool(cov.coverage_ok.all()):
        raise SystemExit('coast-down coverage incomplete; inspect coastdown_coverage.parquet')

    data = pd.DataFrame(samples)
    if len(data) < 50:
        raise SystemExit('insufficient accepted coast-down LiDAR samples')
    x = data.vx_mps.to_numpy(dtype=float)
    y = -data.ax_mps2.to_numpy(dtype=float)

    # d(v) = c0 + c1|v| + c2 v²; constrained non-negative for forward coast-down.
    def residual(params: np.ndarray) -> np.ndarray:
        return params[0] + params[1] * np.abs(x) + params[2] * x * x - y

    result = least_squares(
        residual, x0=np.array([0.4, 0.1, 0.0]),
        bounds=(np.zeros(3), np.array([15.0, 15.0, 15.0])),
        loss='soft_l1', f_scale=0.18, max_nfev=2000,
    )
    pred = result.x[0] + result.x[1] * np.abs(x) + result.x[2] * x * x
    rmse = float(np.sqrt(np.mean((pred - y) ** 2)))
    denom = float(np.sum((y - np.mean(y)) ** 2))
    r2 = float(1 - np.sum((y - pred) ** 2) / denom) if denom > 1e-12 else math.nan
    data['drag_pred_mps2'] = pred
    data['residual_mps2'] = y - pred
    data.to_parquet(out / 'coastdown_samples.parquet', index=False)

    report = {
        'model': 'dv/dt = -(c0 + c1*|v| + c2*v^2) under measured near-zero motor current',
        'accel_drag_coulomb_mps2': float(result.x[0]),
        'accel_drag_viscous_per_s': float(result.x[1]),
        'accel_drag_quadratic_per_m': float(result.x[2]),
        'rmse_mps2': rmse,
        'r2': r2,
        'samples': int(len(data)),
        'coverage_ok': bool(cov.coverage_ok.all()),
        'accepted_for_candidate': bool(cov.coverage_ok.all() and r2 >= float(cfg['analysis']['gates']['min_coastdown_r2'])),
        'gate_min_r2': float(cfg['analysis']['gates']['min_coastdown_r2']),
    }
    dump_yaml(out / 'coastdown_drag_report.yaml', report)
    print(json.dumps(report, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
