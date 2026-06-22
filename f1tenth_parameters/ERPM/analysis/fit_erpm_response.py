#!/usr/bin/env python3
"""Fit full-envelope command-to-ERPM and command-to-ground-speed dynamics."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.optimize import least_squares
from scipy.signal import savgol_filter

from common import accepted_capture_windows, analysis_dir, dump_yaml, expected_grid_coverage, load_yaml, stage_tables


def crossing(t: np.ndarray, y: np.ndarray, level: float, hold: float = 0.05) -> float | None:
    for i in range(len(t)):
        if y[i] < level:
            continue
        j = np.searchsorted(t, t[i] + hold)
        if j > i and np.all(y[i:j] >= level):
            return float(t[i])
    return None


def metrics(t: np.ndarray, y: np.ndarray) -> dict:
    finite = np.isfinite(t) & np.isfinite(y)
    t, y = t[finite], y[finite]
    if len(t) < 12:
        return {'valid': False, 'reason': 'too_few_samples'}
    base_mask = (t >= -1.0) & (t <= -0.05)
    base = np.median(y[base_mask]) if np.any(base_mask) else y[0]
    final = np.median(y[t >= max(0.0, t.max() - 0.60)])
    amplitude = final - base
    if abs(amplitude) < 1e-9:
        return {'valid': False, 'reason': 'zero_amplitude', 'initial': float(base), 'final': float(final)}
    post = t >= 0.0
    tp = t[post]
    norm = (y[post] - base) / amplitude
    t10, t50, t90 = crossing(tp, norm, 0.10), crossing(tp, norm, 0.50), crossing(tp, norm, 0.90)
    peak = None
    if len(tp) >= 7:
        dt = float(np.median(np.diff(tp)))
        window = max(5, int(round(0.075 / max(dt, 1e-3))))
        if window % 2 == 0:
            window += 1
        if window < len(tp):
            peak = float(np.nanmax(np.abs(np.gradient(savgol_filter(y[post], window, 2, mode='interp'), tp))))
    try:
        tau0 = max(0.02, (t90 - t10) / 2.197 if t10 is not None and t90 is not None else 0.2)
        delay0 = max(0.0, (t10 or 0.0) - 0.1 * tau0)
        def model(params: np.ndarray) -> np.ndarray:
            delay, tau, gain = params
            elapsed = np.maximum(tp - delay, 0.0)
            return gain * (1.0 - np.exp(-elapsed / tau))
        result = least_squares(
            lambda params: model(params) - norm,
            x0=[delay0, tau0, 1.0],
            bounds=([0.0, 0.005, 0.2], [min(1.5, tp.max() * 0.75), max(0.2, tp.max() * 2.0), 2.0]),
            loss='soft_l1',
        )
        fopdt = {
            'fopdt_delay_s': float(result.x[0]),
            'fopdt_tau_s': float(result.x[1]),
            'fopdt_gain': float(result.x[2]),
            'fopdt_rmse_normalized': float(np.sqrt(np.mean((model(result.x) - norm) ** 2))),
            'fopdt_fit_ok': bool(result.success),
        }
    except Exception:
        fopdt = {'fopdt_delay_s': None, 'fopdt_tau_s': None, 'fopdt_gain': None, 'fopdt_rmse_normalized': None, 'fopdt_fit_ok': False}
    return {
        'valid': True, 'initial': float(base), 'final': float(final), 'amplitude': float(amplitude),
        'delay_10pct_s': t10, 'delay_50pct_s': t50,
        'rise_10_90_s': None if t10 is None or t90 is None else float(t90 - t10),
        'peak_rate_per_s': peak, **fopdt,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    cfg = load_yaml(session / 'calibration_config_snapshot.yaml')
    out = analysis_dir(session)
    tables = stage_tables(session, '06_raw_erpm_response')
    windows = accepted_capture_windows(tables['events'], 'erpm_step_response')
    rows: list[dict[str, object]] = []
    for _, window in windows.iterrows():
        start, end = int(window.start_ns), int(window.end_ns)
        vesc = tables['vesc'][(tables['vesc'].bag_ns >= start - 1_000_000_000) & (tables['vesc'].bag_ns <= end)].copy()
        lidar = tables['lidar_velocity'][(tables['lidar_velocity'].bag_ns >= start - 1_000_000_000) & (tables['lidar_velocity'].bag_ns <= end)].copy()
        selected = tables['motor_selected_speed'][(tables['motor_selected_speed'].bag_ns >= start - 1_000_000_000) & (tables['motor_selected_speed'].bag_ns <= end)].copy()
        if not lidar.empty and 'valid' in lidar:
            lidar = lidar[lidar.valid.astype(bool)]
        te = (vesc.bag_ns.to_numpy(dtype=float) - start) * 1e-9
        tl = (lidar.bag_ns.to_numpy(dtype=float) - start) * 1e-9
        ts = (selected.bag_ns.to_numpy(dtype=float) - start) * 1e-9
        erpm_metrics = metrics(te, vesc.erpm.to_numpy(dtype=float)) if len(vesc) else {'valid': False, 'reason': 'no_vesc'}
        speed_metrics = metrics(tl, lidar.vx.to_numpy(dtype=float)) if len(lidar) else {'valid': False, 'reason': 'no_lidar'}
        selector_metrics = metrics(ts, selected.value.to_numpy(dtype=float)) if len(selected) else {'valid': False, 'reason': 'no_selector'}
        rows.append({
            'trial_id': str(window.trial_id), 'condition_id': str(window.condition_id),
            'baseline_speed_mps': float(window.get('baseline_speed_mps', math.nan)),
            'target_speed_mps': float(window.get('target_speed_mps', math.nan)),
            'baseline_erpm': float(window.get('baseline_erpm', math.nan)),
            'target_erpm': float(window.get('target_erpm', math.nan)),
            'erpm_metrics_json': json.dumps(erpm_metrics),
            'ground_speed_metrics_json': json.dumps(speed_metrics),
            'selector_metrics_json': json.dumps(selector_metrics),
            'erpm_delay_10pct_s': erpm_metrics.get('delay_10pct_s'),
            'ground_speed_delay_10pct_s': speed_metrics.get('delay_10pct_s'),
            'ground_speed_tau_s': speed_metrics.get('fopdt_tau_s'),
        })
    table = pd.DataFrame(rows)
    table.to_parquet(out / 'erpm_response_trials.parquet', index=False)
    spec = cfg['raw_erpm_response']
    grid = [(float(base), float(target)) for base, target in spec['steps_mps']]
    usable = table.dropna(subset=['erpm_delay_10pct_s', 'ground_speed_delay_10pct_s']) if not table.empty else table
    cov = expected_grid_coverage(
        usable,
        fields=['baseline_speed_mps', 'target_speed_mps'],
        expected_grid=grid,
        expected_repetitions=int(spec['repetitions']),
        tolerances={'baseline_speed_mps': 1e-6, 'target_speed_mps': 1e-6},
    )
    cov.to_parquet(out / 'erpm_response_coverage.parquet', index=False)
    report = {
        'trials': int(len(table)),
        'valid_timing_trials': int(len(usable)),
        'coverage_ok': bool(len(cov)) and bool(cov.coverage_ok.all()),
        'median_command_to_erpm_delay_s': float(usable.erpm_delay_10pct_s.median()) if len(usable) else math.inf,
        'median_command_to_ground_speed_delay_s': float(usable.ground_speed_delay_10pct_s.median()) if len(usable) else math.inf,
        'accepted_for_candidate': bool(
            len(usable) > 0 and bool(cov.coverage_ok.all())
            and usable.erpm_delay_10pct_s.median() <= float(cfg['analysis']['gates']['max_command_to_erpm_delay_s'])
            and usable.ground_speed_delay_10pct_s.median() <= float(cfg['analysis']['gates']['max_command_to_ground_speed_delay_s'])
        ),
    }
    dump_yaml(out / 'erpm_response_report.yaml', report)
    print(json.dumps(report, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
