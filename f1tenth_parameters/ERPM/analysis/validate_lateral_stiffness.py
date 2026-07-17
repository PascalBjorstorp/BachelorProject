#!/usr/bin/env python3
"""Validate frozen lateral tyre models on independent quasi-steady arcs."""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
import yaml

from common import analysis_dir, dump_yaml, expected_grid_coverage, load_yaml
from fit_lateral_stiffness import (
    _expected_grid,
    _metrics,
    _physical,
    _speed_metrics,
    add_turn_slip_predictions,
    bounded_tyre_prediction,
    collect_lateral_trials,
)


def _prediction(alpha: np.ndarray, model: dict) -> np.ndarray:
    alpha = np.asarray(alpha, dtype=float)
    c = float(model.get('cornering_stiffness_N_per_rad', math.nan))
    q = float(model.get('quadratic_saturation_N_per_rad2', 0.0))
    return c * alpha + q * alpha * np.abs(alpha)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    cfg = load_yaml(session / 'calibration_config_snapshot.yaml')
    out = analysis_dir(session)
    training = yaml.safe_load((out / 'lateral_stiffness_training_report.yaml').read_text(encoding='utf-8')) or {}
    physical = _physical(session)
    try:
        steering_model_scale = float(training['steering_model_scale_candidate'])
    except (KeyError, TypeError, ValueError) as exc:
        raise SystemExit('lateral training report has no finite steering-model scale candidate') from exc
    if not math.isfinite(steering_model_scale) or steering_model_scale <= 0.0:
        raise SystemExit('lateral training report has an invalid steering-model scale candidate')
    # The runner has rebuilt this same scale/Cf/Cr candidate before C. The
    # analysis independently tests the frozen values on new arc cells.
    trials = collect_lateral_trials(
        session, '12a_quasi_steady_lateral_validation', cfg, physical,
        steering_model_scale=steering_model_scale,
    )
    turn_slip_candidate = training.get('cornering_longitudinal_slip', {})
    if not isinstance(turn_slip_candidate, dict) or not bool(turn_slip_candidate.get('accepted_for_candidate', False)):
        raise SystemExit('lateral training report has no frozen cornering longitudinal-slip candidate')
    trials = add_turn_slip_predictions(trials, turn_slip_candidate)
    trials.to_parquet(out / 'lateral_stiffness_validation_trials.parquet', index=False)
    usable = trials[trials.measurement_valid.astype(bool)].copy() if not trials.empty else trials
    spec = cfg['lateral_stiffness']
    coverage = expected_grid_coverage(
        usable, fields=['speed_command_mps', 'steering_angle_rad'], expected_grid=_expected_grid(cfg, validation=True),
        expected_repetitions=int(spec['validation_repetitions']),
        tolerances={'speed_command_mps': 1e-6, 'steering_angle_rad': 1e-6},
    )
    coverage.to_parquet(out / 'lateral_stiffness_validation_coverage.parquet', index=False)
    comparison: dict[str, dict] = {}
    failures: list[str] = []
    policy = cfg.get('analysis', {}).get('lateral_stiffness', {})
    nonlinear_required = False
    for axle, alpha_column, force_column, key in (
        ('front', 'alpha_front_rad', 'fy_front_N', 'front_tyre'),
        ('rear', 'alpha_rear_rad', 'fy_rear_N', 'rear_tyre'),
    ):
        alpha = usable[alpha_column].to_numpy(float) if not usable.empty else np.empty(0)
        force = usable[force_column].to_numpy(float) if not usable.empty else np.empty(0)
        fit = training.get(key, {}) if isinstance(training, dict) else {}
        linear = fit.get('linear', {}) if isinstance(fit, dict) else {}
        nonlinear = fit.get('nonlinear', {}) if isinstance(fit, dict) else {}
        runtime = fit.get('runtime_bounded', {}) if isinstance(fit, dict) else {}
        linear_metrics = _metrics(force, _prediction(alpha, linear))
        nonlinear_metrics = _metrics(force, _prediction(alpha, nonlinear))
        runtime_metrics = _metrics(
            force,
            bounded_tyre_prediction(
                alpha,
                float(runtime.get('cornering_stiffness_N_per_rad', math.nan)),
                float(runtime.get('pacejka_shape_factor', math.nan)),
            ),
        )
        improvement = (
            (float(runtime_metrics['rmse_N']) - float(nonlinear_metrics['rmse_N'])) / max(float(runtime_metrics['rmse_N']), 1e-9)
            if math.isfinite(float(runtime_metrics['rmse_N'])) and math.isfinite(float(nonlinear_metrics['rmse_N'])) else -math.inf
        )
        material = bool(improvement >= float(policy.get('nonlinear_holdout_improvement_fraction', 0.15)))
        nonlinear_required = nonlinear_required or material
        comparison[axle] = {
            'frozen_linear_reference': linear_metrics,
            'frozen_runtime_bounded': runtime_metrics,
            'frozen_nonlinear_diagnostic': nonlinear_metrics,
            'nonlinear_holdout_rmse_improvement_over_runtime_fraction': improvement,
            'nonlinear_materially_better': material,
        }
        if int(runtime_metrics['n']) < int(policy.get('min_validation_trials', 8)):
            failures.append(f'{axle} validation has too few usable manoeuvres')
        if float(runtime_metrics['normalized_rmse']) > float(policy.get('max_validation_runtime_normalized_force_rmse', 0.55)):
            failures.append(f'{axle} frozen runtime tyre model force RMSE exceeds gate')
    if not bool(len(coverage)) or not bool(coverage.coverage_ok.all()):
        failures.append('lateral hold-out coverage incomplete')
    if nonlinear_required:
        failures.append('independent hold-out shows a materially better nonlinear tyre model; implement and validate that model before deployment')
    turn_usable = (
        trials[trials.turn_slip_measurement_valid.astype(bool)].copy()
        if not trials.empty and 'turn_slip_measurement_valid' in trials else trials.iloc[0:0].copy()
    )
    if turn_usable.empty:
        turn_truth = np.empty(0)
        turn_baseline_prediction = np.empty(0)
        turn_frozen_prediction = np.empty(0)
        turn_runtime_prediction = np.empty(0)
    else:
        turn_truth = turn_usable.vx_lidar_mps.to_numpy(float)
        turn_baseline_prediction = turn_usable.v_wheel_frozen_mps.to_numpy(float)
        turn_frozen_prediction = turn_usable.turn_slip_frozen_prediction_mps.to_numpy(float)
        turn_runtime_prediction = turn_usable.runtime_odom_vx_mps.to_numpy(float)
    turn_baseline = _speed_metrics(turn_truth, turn_baseline_prediction)
    turn_frozen = _speed_metrics(turn_truth, turn_frozen_prediction)
    runtime_against_truth = _speed_metrics(turn_truth, turn_runtime_prediction)
    runtime_against_frozen = _speed_metrics(turn_frozen_prediction, turn_runtime_prediction)
    turn_active = bool(turn_slip_candidate.get('correction_active', False))
    gates = cfg.get('analysis', {}).get('gates', {})
    min_turn_trials = int(policy.get('turn_slip_min_validation_trials', policy.get('min_validation_trials', 8)))
    if int(turn_frozen['n']) < min_turn_trials:
        failures.append('cornering longitudinal-slip validation has too few usable independent manoeuvres')
    if float(turn_frozen['rmse_mps']) > float(gates.get('max_turn_slip_holdout_rmse_mps', 0.12)):
        failures.append('frozen cornering longitudinal-speed model exceeds the hold-out RMSE gate')
    if abs(float(turn_frozen['bias_mps'])) > float(gates.get('max_turn_slip_holdout_bias_mps', 0.06)):
        failures.append('frozen cornering longitudinal-speed model exceeds the hold-out bias gate')
    if turn_active and float(turn_frozen['rmse_mps']) >= float(turn_baseline['rmse_mps']):
        failures.append('frozen cornering-slip correction does not improve independent hold-out RMSE')
    if int(runtime_against_frozen['n']) < min_turn_trials:
        failures.append('runtime odometry is missing from cornering-slip hold-out manoeuvres')
    elif float(runtime_against_frozen['rmse_mps']) > float(policy.get('turn_slip_max_runtime_equation_rmse_mps', 0.04)):
        failures.append('runtime odometry does not match the frozen cornering-slip equation')
    turn_slip_validation = {
        'frozen_candidate': turn_slip_candidate,
        'usable_independent_trials': int(turn_usable.trial_id.nunique()) if 'trial_id' in turn_usable else int(len(turn_usable)),
        'uncorrected_holdout': turn_baseline,
        'frozen_candidate_holdout': turn_frozen,
        'runtime_odom_against_lidar': runtime_against_truth,
        'runtime_odom_against_frozen_equation': runtime_against_frozen,
        'correction_active': turn_active,
        'straight_line_preserved': True,
    }
    report = {
        'status': 'pass' if not failures else 'fail',
        'accepted_for_validation': not failures,
        'comparison': comparison,
        'frozen_steering_model_scale': steering_model_scale,
        'coverage': coverage.to_dict(orient='records'),
        'requires_nonlinear_tyre_model': nonlinear_required,
        'cornering_longitudinal_slip_validation': turn_slip_validation,
        'failures': failures,
        'scope': 'Validation evaluates frozen steering scale, tyre fits and causal turn-slip correction only. It does not refit any coefficient on hold-out data, and it checks the rebuilt runtime odometry against the same frozen speed equation.',
    }
    dump_yaml(out / 'lateral_stiffness_validation_report.yaml', report)
    if nonlinear_required:
        dump_yaml(out / 'lateral_nonlinear_model_request.yaml', {
            'required': True,
            'reason': 'Hold-out data materially prefer Fy=C*alpha+q*alpha*abs(alpha) over the deployed bounded effective-stiffness model.',
            'evidence': comparison,
            'next_action': 'Port a bounded nonlinear lateral tyre model into the MPC/model interface, then redo lateral_stiffness_training and lateral_stiffness_validation.',
        })
    print(report)
    if failures:
        raise SystemExit('lateral stiffness hold-out rejected: ' + '; '.join(failures))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
