#!/usr/bin/env python3
"""Validate the frozen coast-down drag model on distinct initial speeds."""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np

from common import analysis_dir, dump_yaml, expected_numeric_coverage, load_yaml
from fit_coastdown import _trajectory_prediction, collect_coastdown_trials


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    cfg = load_yaml(session / 'calibration_config_snapshot.yaml')
    out = analysis_dir(session)
    training = load_yaml(out / 'coastdown_drag_report.yaml')
    params = np.asarray([
        training.get('accel_drag_coulomb_mps2', math.nan),
        training.get('accel_drag_viscous_per_s', math.nan),
        training.get('accel_drag_quadratic_per_m', math.nan),
    ], dtype=float)
    trials, samples = collect_coastdown_trials(session, '07a_coastdown_validation', cfg)
    trials.to_parquet(out / 'coastdown_validation_trials.parquet', index=False)
    usable = trials[trials.measurement_valid.astype(bool)].copy() if not trials.empty else trials
    spec = cfg['coastdown']
    coverage = expected_numeric_coverage(
        usable, 'initial_speed_mps', spec['validation_initial_speeds_mps'], int(spec['validation_repetitions'])
    )
    coverage.to_parquet(out / 'coastdown_validation_coverage.parquet', index=False)
    rows: list[dict[str, object]] = []
    residuals: list[float] = []
    for trial_id, group in samples.groupby('trial_id', sort=False) if not samples.empty else []:
        group = group.sort_values('t_s').copy()
        observed = group.vx_mps.to_numpy(float)
        prediction = _trajectory_prediction(params, group.t_s.to_numpy(float), float(observed[0])) if np.all(np.isfinite(params)) else np.full(len(group), math.nan)
        residual = observed - prediction
        group['vx_model_mps'] = prediction
        group['velocity_residual_mps'] = residual
        rows.extend(group.to_dict(orient='records'))
        residuals.extend(residual[np.isfinite(residual)].tolist())
    import pandas as pd
    sample_table = pd.DataFrame(rows)
    sample_table.to_parquet(out / 'coastdown_validation_samples.parquet', index=False)
    values = np.asarray(residuals, dtype=float)
    rmse = float(np.sqrt(np.mean(values ** 2))) if len(values) else math.inf
    bias = float(np.mean(values)) if len(values) else math.nan
    gates = cfg['analysis']['gates']
    policy = cfg.get('analysis', {}).get('coastdown', {})
    failures: list[str] = []
    if not bool(len(coverage)) or not bool(coverage.coverage_ok.all()):
        failures.append('coast-down hold-out coverage incomplete')
    if not np.all(np.isfinite(params)):
        failures.append('training drag parameters are not finite')
    if rmse > float(policy.get('max_validation_velocity_rmse_mps', 0.18)):
        failures.append('hold-out velocity trajectory RMSE exceeds gate')
    if not math.isfinite(bias) or abs(bias) > float(policy.get('max_validation_velocity_bias_mps', 0.10)):
        failures.append('hold-out velocity trajectory bias exceeds gate')
    report = {
        'status': 'pass' if not failures else 'fail',
        'accepted_for_validation': not failures,
        'frozen_training_parameters': {
            'accel_drag_coulomb_mps2': float(params[0]) if math.isfinite(params[0]) else None,
            'accel_drag_viscous_per_s': float(params[1]) if math.isfinite(params[1]) else None,
            'accel_drag_quadratic_per_m': float(params[2]) if math.isfinite(params[2]) else None,
        },
        'holdout_velocity_rmse_mps': rmse,
        'holdout_velocity_bias_mps': bias,
        'holdout_velocity_windows': int(len(values)),
        'coverage': coverage.to_dict(orient='records'),
        'gate_max_validation_velocity_rmse_mps': float(policy.get('max_validation_velocity_rmse_mps', 0.18)),
        'gate_max_validation_velocity_bias_mps': float(policy.get('max_validation_velocity_bias_mps', 0.10)),
        'failures': failures,
        'note': 'The hold-out evaluates the frozen training coefficients by trajectory integration; it does not refit drag.',
    }
    dump_yaml(out / 'coastdown_validation_report.yaml', report)
    print(report)
    if failures:
        raise SystemExit('coast-down hold-out rejected: ' + '; '.join(failures))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
