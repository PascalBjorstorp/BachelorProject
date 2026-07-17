#!/usr/bin/env python3
"""Identify high-demand longitudinal slip onset, peak and recovery.

Stages 8/9 already identify a speed/current acceleration surface.  This script
uses the complete high-current pulse and post-release recovery captures to
measure whether the wheel-equivalent speed and LiDAR ground speed diverge in a
way that a memoryless scalar ACCEL_TO_CURRENT gain cannot represent.

The output is intentionally diagnostic and conservative: a high peak slip or
slow slip onset/recovery rejects automatic deployment of the scalar candidate;
it does not claim an electrical safety limit.
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
    stage_tables,
)


def _high_grid(cfg: dict, section: str, polarity: str) -> list[tuple[object, ...]]:
    spec = cfg[section]
    threshold = float(cfg['traction_transients']['minimum_current_fraction_for_transient_metrics'])
    entries = spec.get(f'{polarity}_conditions')
    if entries is None:
        speeds = spec[f'{polarity}_initial_speeds_mps']
        fractions = spec[f'{polarity}_current_fractions']
        entries = [{'initial_speed_mps': speed, 'current_fractions': fractions} for speed in speeds]
    return [
        (polarity, float(entry['initial_speed_mps']), float(fraction))
        for entry in entries
        for fraction in entry['current_fractions']
        if float(fraction) >= threshold
    ]


def _window_by_trial(windows: pd.DataFrame) -> dict[str, pd.Series]:
    return {str(row.trial_id): row for _, row in windows.iterrows()}


def _metric_for_pulse(
    lidar: pd.DataFrame,
    vesc: pd.DataFrame,
    pulse: pd.Series,
    recovery: pd.Series | None,
    *,
    direct_static: dict,
    speed_floor: float,
    slip_threshold: float,
) -> dict[str, object] | None:
    start_ns = int(pulse.start_ns)
    pulse_end_ns = int(pulse.end_ns)
    end_ns = int(recovery.end_ns) if recovery is not None else pulse_end_ns
    li = lidar[(lidar.bag_ns >= start_ns) & (lidar.bag_ns <= end_ns)].copy()
    li = li[li.valid.astype(bool)] if not li.empty and 'valid' in li else li
    if len(li) < 8:
        return None
    li = li.sort_values('bag_ns')
    times = li.bag_ns.to_numpy(dtype=np.int64)
    vx = li.vx.to_numpy(dtype=float)
    vesc_sorted = vesc.sort_values('bag_ns') if not vesc.empty else vesc
    if vesc_sorted.empty or 'erpm' not in vesc_sorted:
        return None
    valid_vesc = vesc_sorted[['bag_ns', 'erpm']].dropna()
    if len(valid_vesc) < 2:
        return None
    erpm = np.interp(
        times.astype(float),
        valid_vesc.bag_ns.to_numpy(dtype=float),
        valid_vesc.erpm.to_numpy(dtype=float),
        left=np.nan, right=np.nan,
    )
    wheel = eval_origin_quadratic(
        erpm,
        float(direct_static['gain_mps_per_erpm']),
        float(direct_static['quadratic_mps_per_erpm2']),
    )
    slip = (wheel - vx) / np.maximum(np.abs(vx), float(speed_floor))
    pulse_mask = (times >= start_ns) & (times <= pulse_end_ns) & np.isfinite(slip)
    if int(pulse_mask.sum()) < 5:
        return None
    pulse_slip = slip[pulse_mask]
    n_base = max(3, int(math.ceil(0.20 * len(pulse_slip))))
    baseline = float(np.median(pulse_slip[:n_base]))
    departure = np.abs(pulse_slip - baseline)
    exceeds = np.flatnonzero(departure >= float(slip_threshold))
    pulse_time = (times[pulse_mask].astype(float) - float(start_ns)) * 1e-9
    onset_s = float(pulse_time[exceeds[0]]) if len(exceeds) else math.nan
    peak_idx = int(np.argmax(departure))
    peak_abs = float(departure[peak_idx])
    peak_signed = float(pulse_slip[peak_idx] - baseline)
    peak_time_s = float(pulse_time[peak_idx])

    recovery_delay_s = math.nan
    if recovery is not None:
        rec_start = int(recovery.start_ns)
        rec_mask = (times >= rec_start) & (times <= int(recovery.end_ns)) & np.isfinite(slip)
        if int(rec_mask.sum()) >= 3:
            rec_departure = np.abs(slip[rec_mask] - baseline)
            rec_time = (times[rec_mask].astype(float) - float(rec_start)) * 1e-9
            recovered = np.flatnonzero(rec_departure <= float(slip_threshold))
            if len(recovered):
                recovery_delay_s = float(rec_time[recovered[0]])

    return {
        'trial_id': str(pulse.trial_id),
        'condition_id': str(pulse.condition_id),
        'polarity': str(pulse.get('polarity', '')),
        'initial_speed_mps': float(pulse.get('initial_speed_mps', math.nan)),
        'current_fraction': float(pulse.get('current_fraction', math.nan)),
        'pulse_current_command_a': float(pulse.get('current_command_a', math.nan)),
        'pulse_duration_s': float(pulse.get('pulse_duration_s', math.nan)),
        'has_recovery_capture': recovery is not None,
        'baseline_slip_ratio': baseline,
        'peak_signed_slip_delta': peak_signed,
        'peak_abs_slip_delta': peak_abs,
        'slip_onset_delay_s': onset_s,
        'slip_peak_time_s': peak_time_s,
        'slip_recovery_delay_s': recovery_delay_s,
        'lidar_pair_count': int(len(li)),
    }


def _stage_metrics(rows: pd.DataFrame, cfg: dict, section: str) -> dict:
    threshold = float(cfg['traction_transients']['minimum_current_fraction_for_transient_metrics'])
    required_reps = int(cfg['traction_transients']['min_high_demand_transient_repetitions'])
    details: dict[str, dict] = {}
    coverage_ok = True
    for polarity in ('drive', 'brake'):
        part = rows[rows.polarity.astype(str) == polarity].copy() if not rows.empty else rows
        grid = _high_grid(cfg, section, polarity)
        coverage = expected_grid_coverage(
            part,
            fields=['polarity', 'initial_speed_mps', 'current_fraction'],
            expected_grid=grid,
            expected_repetitions=required_reps if section == 'raw_current_training' else int(cfg[section]['repetitions']),
            tolerances={'initial_speed_mps': 1e-6, 'current_fraction': 1e-6},
        )
        details[polarity] = {
            'coverage': coverage.to_dict(orient='records'),
            'coverage_ok': bool(len(coverage)) and bool(coverage.coverage_ok.all()),
            'median_peak_abs_slip_ratio': float(part.peak_abs_slip_delta.median()) if not part.empty else math.nan,
            'p95_peak_abs_slip_ratio': float(part.peak_abs_slip_delta.quantile(0.95)) if not part.empty else math.nan,
            'median_slip_onset_delay_s': float(part.slip_onset_delay_s.median()) if not part.empty else math.nan,
            'median_slip_recovery_delay_s': float(part.slip_recovery_delay_s.median()) if not part.empty else math.nan,
            'n': int(len(part)),
        }
        coverage_ok = coverage_ok and details[polarity]['coverage_ok']
    return {'minimum_current_fraction': threshold, 'coverage_ok': coverage_ok, 'by_polarity': details}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    cfg = load_yaml(session / 'calibration_config_snapshot.yaml')
    out = analysis_dir(session)
    import yaml
    static = yaml.safe_load((out / 'erpm_speed_map_report.yaml').read_text(encoding='utf-8'))
    direct = static['measured_erpm_odometry_map']['direct_erpm_to_ground_speed_candidate']
    threshold = float(cfg['analysis']['slip']['high_slip_ratio_threshold'])
    speed_floor = float(cfg['analysis']['slip']['speed_floor_mps'])

    all_rows: list[dict[str, object]] = []
    per_stage: dict[str, pd.DataFrame] = {}
    for stage, section in (
        ('08_raw_current_training', 'raw_current_training'),
        ('09_raw_current_holdout', 'raw_current_holdout'),
    ):
        tables = stage_tables(session, stage)
        pulses = accepted_capture_windows(tables['events'], ['raw_drive_current_pulse', 'raw_brake_current_pulse'])
        recoveries = accepted_capture_windows(tables['events'], ['raw_drive_current_recovery', 'raw_brake_current_recovery'])
        recovery_by_trial = _window_by_trial(recoveries)
        rows: list[dict[str, object]] = []
        for _, pulse in pulses.iterrows():
            fraction = float(pulse.get('current_fraction', math.nan))
            if not math.isfinite(fraction) or fraction < float(cfg['traction_transients']['minimum_current_fraction_for_transient_metrics']):
                continue
            metric = _metric_for_pulse(
                tables['lidar_velocity'], tables['vesc'], pulse,
                recovery_by_trial.get(str(pulse.trial_id)),
                direct_static=direct, speed_floor=speed_floor, slip_threshold=threshold,
            )
            if metric is not None:
                metric['source_stage'] = stage
                rows.append(metric)
                all_rows.append(metric)
        frame = pd.DataFrame(rows)
        frame.to_parquet(out / f'{stage}_traction_transients.parquet', index=False)
        per_stage[section] = frame

    training = _stage_metrics(per_stage['raw_current_training'], cfg, 'raw_current_training')
    holdout = _stage_metrics(per_stage['raw_current_holdout'], cfg, 'raw_current_holdout')
    dynamic_cfg = cfg['traction_transients']
    hold_metrics = holdout['by_polarity']
    scalar_dynamic_ok = bool(
        training['coverage_ok'] and holdout['coverage_ok']
        and all(
            math.isfinite(hold_metrics[p]['p95_peak_abs_slip_ratio'])
            and hold_metrics[p]['p95_peak_abs_slip_ratio'] <= float(dynamic_cfg['max_peak_slip_ratio_for_scalar_candidate'])
            and math.isfinite(hold_metrics[p]['median_slip_onset_delay_s'])
            and hold_metrics[p]['median_slip_onset_delay_s'] <= float(dynamic_cfg['max_median_slip_onset_delay_s_for_scalar_candidate'])
            for p in ('drive', 'brake')
        )
    )
    report = {
        'definition': 'slip ratio is (static-ERPM wheel-equivalent speed - LiDAR ground speed) / max(|LiDAR speed|, speed_floor). Peak values are measured relative to the pulse-start baseline.',
        'training': training,
        'holdout': holdout,
        'scalar_dynamic_traction_adequate_over_envelope': scalar_dynamic_ok,
        'requires_dynamic_longitudinal_slip_model': not scalar_dynamic_ok,
        'gates': {
            'max_peak_slip_ratio_for_scalar_candidate': float(dynamic_cfg['max_peak_slip_ratio_for_scalar_candidate']),
            'max_median_slip_onset_delay_s_for_scalar_candidate': float(dynamic_cfg['max_median_slip_onset_delay_s_for_scalar_candidate']),
        },
        'notes': [
            'These metrics characterize traction/slip dynamics; they are not electrical or thermal safe-current limits.',
            'A failed dynamic traction gate rejects automatic scalar candidate deployment, even when a memoryless current/acceleration surface has acceptable RMSE.',
        ],
    }
    pd.DataFrame(all_rows).to_parquet(out / 'traction_transient_trials.parquet', index=False)
    dump_yaml(out / 'traction_transient_report.yaml', report)
    if not scalar_dynamic_ok:
        dump_yaml(out / 'dynamic_longitudinal_slip_model_request.yaml', {
            'required': True,
            'reason': 'independent traction transients reject a memoryless scalar ACCEL_TO_CURRENT model over the tested envelope',
            'training': training,
            'holdout': holdout,
            'gates': report['gates'],
            'next_action': (
                'Implement a bounded dynamic longitudinal slip/traction state in the actuation and prediction path, '
                'then redo current_training and current_holdout before continuing.'
            ),
        })
    print(json.dumps(report, indent=2, default=str))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
