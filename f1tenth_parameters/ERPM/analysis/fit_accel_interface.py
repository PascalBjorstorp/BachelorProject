#!/usr/bin/env python3
"""Audit ACCEL_TO_CURRENT routing over the complete speed/acceleration grid."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd

from common import accepted_capture_windows, analysis_dir, dump_yaml, expected_grid_coverage, load_yaml, stage_tables


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    cfg = load_yaml(session / 'calibration_config_snapshot.yaml')
    out = analysis_dir(session)
    tables = stage_tables(session, '10_accel_to_current_interface')
    import yaml
    current = yaml.safe_load((out / 'current_acceleration_report.yaml').read_text(encoding='utf-8'))
    windows = accepted_capture_windows(tables['events'], 'accel_to_current_pulse')
    rows: list[dict[str, object]] = []
    for _, window in windows.iterrows():
        vesc = tables['vesc'][(tables['vesc'].bag_ns >= window.start_ns) & (tables['vesc'].bag_ns <= window.end_ns)]
        selected_current = tables['motor_selected_current'][(tables['motor_selected_current'].bag_ns >= window.start_ns) & (tables['motor_selected_current'].bag_ns <= window.end_ns)]
        selected_brake = tables['motor_selected_brake'][(tables['motor_selected_brake'].bag_ns >= window.start_ns) & (tables['motor_selected_brake'].bag_ns <= window.end_ns)]
        command = float(window.get('acceleration_command_mps2', math.nan))
        initial_speed = float(window.get('initial_speed_mps', math.nan))
        selected_c = float(np.nanmedian(selected_current.value)) if len(selected_current) else math.nan
        selected_b = float(np.nanmedian(selected_brake.value)) if len(selected_brake) else math.nan
        actual_motor = float(np.nanmedian(vesc.motor_current)) if len(vesc) else math.nan
        if command >= 0:
            expected = float(current['candidate_accel_to_current_gain']) * command
            observed = selected_c
        else:
            expected = float(current['candidate_accel_to_brake_gain']) * abs(command)
            observed = selected_b
        rows.append({
            'trial_id': str(window.trial_id), 'condition_id': str(window.condition_id),
            'initial_speed_mps': initial_speed,
            'acceleration_command_mps2': command,
            'selected_current_a': selected_c, 'selected_brake_a': selected_b,
            'motor_current_a': actual_motor,
            'expected_interface_output_a': expected,
            'interface_residual_a': observed - expected,
            'measurement_valid': bool(math.isfinite(observed) and math.isfinite(expected)),
        })
    table = pd.DataFrame(rows)
    table.to_parquet(out / 'accel_to_current_interface_trials.parquet', index=False)
    usable = table[table.measurement_valid.astype(bool)].copy() if not table.empty else table
    spec = cfg['accel_to_current_interface']
    grid = [(float(v), float(a)) for v in spec['initial_speeds_mps'] for a in spec['acceleration_commands_mps2']]
    coverage = expected_grid_coverage(
        usable,
        fields=['initial_speed_mps', 'acceleration_command_mps2'],
        expected_grid=grid,
        expected_repetitions=int(spec['repetitions']),
        tolerances={'initial_speed_mps': 1e-6, 'acceleration_command_mps2': 1e-6},
    )
    coverage.to_parquet(out / 'accel_to_current_interface_coverage.parquet', index=False)
    residual = usable.interface_residual_a.to_numpy(dtype=float) if not usable.empty else np.array([])
    rmse = float(np.sqrt(np.mean(residual ** 2))) if len(residual) else math.inf
    bias = float(np.mean(residual)) if len(residual) else math.nan
    report = {
        'interface_output_rmse_a': rmse,
        'interface_output_bias_a': bias,
        'trials': int(len(usable)),
        'coverage_ok': bool(len(coverage)) and bool(coverage.coverage_ok.all()),
        'accepted_for_candidate': bool(
            len(coverage) and bool(coverage.coverage_ok.all())
            and rmse <= float(cfg['analysis']['gates']['max_accel_interface_current_rmse_a'])
        ),
        'gate_max_interface_output_rmse_a': float(cfg['analysis']['gates']['max_accel_interface_current_rmse_a']),
        'note': 'This stage audits ACCEL_TO_CURRENT routing across the full envelope. High-demand physical adequacy is judged by Stage 8/9 traction/slip hold-outs and Stage 12 candidate verification.',
    }
    dump_yaml(out / 'accel_to_current_interface_report.yaml', report)
    print(json.dumps(report, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
