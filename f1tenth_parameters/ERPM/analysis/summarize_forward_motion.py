#!/usr/bin/env python3
"""Assemble one operator-facing summary for forward-motion calibration outputs."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd

from common import analysis_dir, dump_yaml, load_yaml, read_table

G_MPS2 = 9.81


def _finite_or_none(value: object) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _best_row(frame: pd.DataFrame, *, value_column: str, descending: bool) -> dict[str, object] | None:
    if frame.empty or value_column not in frame:
        return None
    series = pd.to_numeric(frame[value_column], errors='coerce')
    valid = frame.loc[np.isfinite(series.to_numpy(dtype=float))].copy()
    if valid.empty:
        return None
    row = valid.sort_values(value_column, ascending=not descending).iloc[0]
    return {
        'trial_id': str(row.get('trial_id', '')),
        'condition_id': str(row.get('condition_id', '')),
        value_column: float(row[value_column]),
        'ax_lidar_mps2': _finite_or_none(row.get('ax_lidar_mps2')),
        'net_accel_mps2': _finite_or_none(row.get('net_accel_mps2')),
        'vx_lidar_mps': _finite_or_none(row.get('vx_lidar_mps')),
        'initial_speed_mps': _finite_or_none(row.get('initial_speed_mps')),
        'current_fraction': _finite_or_none(row.get('current_fraction')),
        'current_command_a': _finite_or_none(row.get('current_command_a')),
        'motor_current_a': _finite_or_none(row.get('motor_current_a')),
        'longitudinal_slip_ratio': _finite_or_none(row.get('longitudinal_slip_ratio')),
    }


def _combine_trials(out: Path) -> pd.DataFrame:
    parts = []
    for name, section in (
        ('current_model_training_trials.parquet', 'training'),
        ('current_model_holdout_trials.parquet', 'holdout'),
    ):
        frame = read_table(out / name)
        if frame.empty:
            continue
        tagged = frame.copy()
        tagged['source_partition'] = section
        parts.append(tagged)
    return pd.concat(parts, ignore_index=True) if parts else pd.DataFrame()


def _polarity_subset(frame: pd.DataFrame, polarity: str) -> pd.DataFrame:
    if frame.empty or 'polarity' not in frame:
        return frame.iloc[0:0].copy() if not frame.empty else pd.DataFrame()
    return frame[frame.polarity.astype(str) == polarity].copy()


def _ground_extrema(frame: pd.DataFrame, polarity: str) -> dict[str, object]:
    part = _polarity_subset(frame, polarity)
    if part.empty:
        return {}
    if polarity == 'drive':
        ground = pd.to_numeric(part['ax_lidar_mps2'], errors='coerce')
        best = part.loc[np.isfinite(ground.to_numpy(dtype=float))].copy()
        if best.empty:
            return {}
        row = best.iloc[int(np.nanargmax(ground.loc[best.index].to_numpy(dtype=float)))]
        accel = float(row['ax_lidar_mps2'])
    else:
        ground = -pd.to_numeric(part['ax_lidar_mps2'], errors='coerce')
        best = part.loc[np.isfinite(ground.to_numpy(dtype=float))].copy()
        if best.empty:
            return {}
        values = -pd.to_numeric(best['ax_lidar_mps2'], errors='coerce').to_numpy(dtype=float)
        row = best.iloc[int(np.nanargmax(values))]
        accel = float(-row['ax_lidar_mps2'])
    return {
        'trial_id': str(row.get('trial_id', '')),
        'condition_id': str(row.get('condition_id', '')),
        'ground_accel_mps2': accel,
        'mu_estimate': accel / G_MPS2,
        'vx_lidar_mps': _finite_or_none(row.get('vx_lidar_mps')),
        'initial_speed_mps': _finite_or_none(row.get('initial_speed_mps')),
        'current_fraction': _finite_or_none(row.get('current_fraction')),
        'current_command_a': _finite_or_none(row.get('current_command_a')),
        'motor_current_a': _finite_or_none(row.get('motor_current_a')),
        'longitudinal_slip_ratio': _finite_or_none(row.get('longitudinal_slip_ratio')),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    cfg = load_yaml(session / 'calibration_config_snapshot.yaml')
    out = analysis_dir(session)

    speed = load_yaml(out / 'erpm_speed_map_report.yaml')
    odom = load_yaml(out / 'odometry_model_selection_report.yaml')
    odom_patch = load_yaml(out / 'selected_odometry_candidate_patch.yaml')
    drag = load_yaml(out / 'coastdown_drag_report.yaml')
    current = load_yaml(out / 'current_acceleration_report.yaml')
    transient = load_yaml(out / 'traction_transient_report.yaml')
    response = load_yaml(out / 'erpm_response_report.yaml')
    interface = load_yaml(out / 'accel_to_current_interface_report.yaml')
    candidate = load_yaml(out / 'longitudinal_candidate_summary.yaml')
    ack_patch = odom_patch.get('ackermann_to_vesc_node', {}).get('ros__parameters', {})
    odom_patch_params = odom_patch.get('vesc_to_odom_node', {}).get('ros__parameters', {})

    trials = _combine_trials(out)
    drive = _polarity_subset(trials, 'drive')
    brake = _polarity_subset(trials, 'brake')

    drive_ground = _ground_extrema(trials, 'drive')
    brake_ground = _ground_extrema(trials, 'brake')
    drive_net = _best_row(drive, value_column='net_accel_mps2', descending=True) or {}
    brake_net = _best_row(brake, value_column='net_accel_mps2', descending=True) or {}

    report = {
        'session': str(session),
        'campaign_contract': {
            'operator_entrypoint': 'python3 erpm_calibration.py --workspace <workspace>',
            'one_script_operator_workflow': True,
            'automatic_internal_checkpoints': [
                {
                    'after_stage': '05_vel_to_erpm_pipeline_audit',
                    'purpose': 'fit the interim speed/odometry map and use it to improve later setup-speed establishment and runtime odometry',
                },
                {
                    'after_stage': '09_raw_current_holdout',
                    'purpose': 'fit drag and bootstrap acceleration/current terms before Stage 10 ACCEL_TO_CURRENT routing audit',
                },
            ],
        },
        'configured_operating_envelope': {
            'maximum_test_speed_mps': float(cfg['operating_envelope']['maximum_test_speed_mps']),
            'maximum_test_accel_mps2': float(cfg['operating_envelope']['maximum_test_accel_mps2']),
            'maximum_test_brake_mps2': float(cfg['operating_envelope']['maximum_test_brake_mps2']),
            'approved_drive_test_current_a': float(cfg['operating_envelope']['approved_drive_test_current_a']),
            'approved_brake_test_current_a': float(cfg['operating_envelope']['approved_brake_test_current_a']),
            'straight_usable_length_m': float(cfg['site']['straight_usable_length_m']),
        },
        'speed_command_model': {
            'selected_model': str(ack_patch.get('speed_command_model', odom.get('command_map_selected', speed['command_map']['selected_model']))),
            'candidate_speed_to_erpm_gain': float(ack_patch.get('speed_to_erpm_gain', speed['candidate_speed_to_erpm_gain'])),
            'candidate_speed_to_erpm_quadratic': float(ack_patch.get('speed_to_erpm_quadratic', speed.get('candidate_speed_to_erpm_quadratic', 0.0))),
            'candidate_speed_to_erpm_offset': 0.0,
            'candidate_speed_command_lut_speed_mps': ack_patch.get('speed_command_lut_speed_mps'),
            'candidate_speed_command_lut_erpm': ack_patch.get('speed_command_lut_erpm'),
            'candidate_slow_start_threshold_mps': float(speed['candidate_slow_start_threshold_mps']),
            'candidate_slow_start_increment_mps': float(speed['candidate_slow_start_increment_mps']),
            'candidate_stop_speed_deadzone_mps': float(speed['candidate_stop_speed_deadzone_mps']),
            'accepted_for_candidate': bool(speed.get('accepted_for_candidate')),
        },
        'wheel_speed_and_odometry': {
            'selected_odometry_family': str(odom.get('selected_family', '')),
            'selected_wheel_observation': str(odom.get('selected_wheel_observation', '')),
            'command_map_selected': str(odom.get('command_map_selected', '')),
            'candidate_odom_speed_scale': _finite_or_none(odom_patch_params.get('odom_speed_scale', speed.get('candidate_odom_speed_scale'))),
            'candidate_odom_speed_deadband_mps': float(odom_patch_params.get('speed_deadband', speed['candidate_odom_speed_deadband_mps'])),
            'accepted_for_shadow_deployment_verification': bool(odom.get('accepted_for_shadow_deployment_verification')),
        },
        'acceleration_and_drag_model': {
            'selected_acceleration_command_model': str(candidate.get('acceleration_model_selection', {}).get('selected_model', 'scalar')),
            'candidate_accel_to_current_gain': float(current['candidate_accel_to_current_gain']),
            'candidate_accel_to_brake_gain': float(current['candidate_accel_to_brake_gain']),
            'candidate_accel_deadzone_mps2': float(current['candidate_accel_deadzone_mps2']),
            'accel_drag_coulomb_mps2': float(drag['accel_drag_coulomb_mps2']),
            'accel_drag_viscous_per_s': float(drag['accel_drag_viscous_per_s']),
            'accel_drag_quadratic_per_m': float(drag['accel_drag_quadratic_per_m']),
            'scalar_accel_to_current_adequate_over_envelope': bool(current.get('scalar_accel_to_current_adequate_over_envelope')),
            'requires_nonlinear_longitudinal_model': bool(current.get('requires_nonlinear_longitudinal_model')),
            'requires_dynamic_longitudinal_slip_model': bool(transient.get('requires_dynamic_longitudinal_slip_model')),
        },
        'observed_straight_line_envelope': {
            'maximum_observed_drive_ground_accel': drive_ground,
            'maximum_observed_brake_ground_decel': brake_ground,
            'maximum_observed_drive_net_accel': drive_net,
            'maximum_observed_brake_net_decel': brake_net,
            'note': (
                'Ground acceleration/deceleration are the actual LiDAR-derived straight-line values. '
                'Net acceleration adds back fitted drag and is useful for current-model inversion. '
                'Mu estimates are maximum observed straight-line values during accepted Stage 8/9 runs, not universal tyre limits.'
            ),
        },
        'response_and_interface': {
            'median_command_to_erpm_delay_s': _finite_or_none(response.get('median_command_to_erpm_delay_s')),
            'median_command_to_ground_speed_delay_s': _finite_or_none(response.get('median_command_to_ground_speed_delay_s')),
            'interface_output_rmse_a': _finite_or_none(interface.get('interface_output_rmse_a')),
            'interface_output_bias_a': _finite_or_none(interface.get('interface_output_bias_a')),
        },
        'slip_and_traction_holdout': {
            'drive': current.get('slip', {}).get('drive', {}),
            'brake': current.get('slip', {}).get('brake', {}),
            'transient_training': transient.get('training', {}),
            'transient_holdout': transient.get('holdout', {}),
        },
        'candidate_readiness': {
            'accepted_for_temporary_candidate_verification': bool(candidate.get('accepted_for_temporary_candidate_verification')),
            'full_stack_upgrade_required': bool(candidate.get('required_full_stack_upgrade', {}).get('full_stack_upgrade_required')),
            'temporary_candidate_summary': 'analysis/longitudinal_candidate_summary.yaml',
        },
    }
    dump_yaml(out / 'forward_motion_summary.yaml', report)
    print(json.dumps(report, indent=2, default=str))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
