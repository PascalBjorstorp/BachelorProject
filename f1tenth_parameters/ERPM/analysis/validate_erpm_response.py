#!/usr/bin/env python3
"""Validate ERPM/ground-speed response timing on distinct speed steps."""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import yaml

from common import analysis_dir, dump_yaml, load_yaml


def _finite(mapping: dict, key: str) -> float:
    try:
        value = float(mapping.get(key, math.nan))
    except (TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    cfg = load_yaml(session / 'calibration_config_snapshot.yaml')
    out = analysis_dir(session)
    training = load_yaml(out / 'erpm_response_report.yaml')
    validation = load_yaml(out / 'erpm_response_validation_observed_report.yaml')
    gates = cfg['analysis']['gates']
    failures: list[str] = []
    if not bool(validation.get('coverage_ok', False)):
        failures.append('response hold-out coverage incomplete')
    for key, limit_key in (
        ('median_command_to_erpm_delay_s', 'max_command_to_erpm_delay_s'),
        ('median_command_to_ground_speed_delay_s', 'max_command_to_ground_speed_delay_s'),
    ):
        value = _finite(validation, key)
        if not math.isfinite(value) or value > float(gates[limit_key]):
            failures.append(f'{key}={value!r} exceeds gate {float(gates[limit_key]):.3f}')
    train_tau = _finite(training, 'median_ground_speed_fopdt_tau_s')
    hold_tau = _finite(validation, 'median_ground_speed_fopdt_tau_s')
    ratio = max(train_tau, hold_tau) / max(min(train_tau, hold_tau), 1e-9) if math.isfinite(train_tau) and math.isfinite(hold_tau) else math.inf
    max_ratio = float(cfg.get('analysis', {}).get('response', {}).get('max_validation_training_tau_ratio', 3.0))
    if ratio > max_ratio:
        failures.append(f'training/hold-out ground-speed FOPDT tau ratio {ratio:.3f} exceeds {max_ratio:.3f}')
    report = {
        'status': 'pass' if not failures else 'fail',
        'accepted_for_validation': not failures,
        'training': training,
        'holdout': validation,
        'ground_speed_fopdt_tau_ratio': ratio if math.isfinite(ratio) else None,
        'failures': failures,
    }
    dump_yaml(out / 'erpm_response_validation_report.yaml', report)
    print(report)
    if failures:
        raise SystemExit('ERPM response hold-out rejected: ' + '; '.join(failures))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
