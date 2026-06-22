#!/usr/bin/env python3
"""Assemble a reversible, full-envelope longitudinal candidate.

This stage never assumes that the low-slip scalar current map remains valid at
high demand.  It carries both the scalar calibration and, when justified by
held-out current data, an invertible speed/current traction surface into the
Stage 12 shadow verification launch.  Stage 12 is therefore evidence about the
actual candidate command path, not merely an offline curve fit.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import yaml

from common import analysis_dir, dump_yaml, load_yaml


def read(path: Path) -> dict:
    value = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    if not isinstance(value, dict):
        raise ValueError(f'expected mapping: {path}')
    return value


def merge(target: dict, patch: dict) -> dict:
    for key, value in patch.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            merge(target[key], value)
        else:
            target[key] = value
    return target


def _surface_ready(current: dict, gates: dict) -> tuple[bool, list[str]]:
    faults: list[str] = []
    surface = current.get('full_envelope_surface_fit', {})
    holdout = current.get('holdout', {})
    coverage = current.get('coverage', {})
    for polarity in ('drive', 'brake'):
        fit = surface.get(polarity, {})
        result = holdout.get(polarity, {}).get('surface', {})
        cov = coverage.get(polarity, {})
        if not bool(cov.get('training_ok')) or not bool(cov.get('holdout_ok')):
            faults.append(f'{polarity}: incomplete current/speed condition coverage')
        if not math.isfinite(float(fit.get('training_r2', math.nan))) or float(fit['training_r2']) < float(gates['min_current_model_r2']):
            faults.append(f'{polarity}: traction-surface training R² below gate')
        if not math.isfinite(float(result.get('rmse_mps2', math.nan))) or float(result['rmse_mps2']) > float(gates['max_current_model_holdout_rmse_mps2']):
            faults.append(f'{polarity}: traction-surface hold-out RMSE above gate')
        if not math.isfinite(float(result.get('bias_mps2', math.nan))) or abs(float(result['bias_mps2'])) > float(gates['max_current_model_holdout_bias_mps2']):
            faults.append(f'{polarity}: traction-surface hold-out bias above gate')
    return not faults, faults


def _surface_patch(surface: dict, polarity: str) -> dict:
    p = surface.get(polarity, {})
    keys = {
        'c1': 'c1_accel_per_amp',
        'c2': 'c2_accel_per_amp2',
        'c3': 'c3_accel_per_amp_mps',
        'c4': 'c4_accel_per_amp_mps2',
    }
    return {f'accel_surface_{polarity}_{dst}': float(p[src]) for dst, src in keys.items()}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('session', type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    out = analysis_dir(session)
    cfg = load_yaml(session / 'calibration_config_snapshot.yaml')
    gates = cfg['analysis']['gates']

    speed = read(out / 'erpm_speed_map_report.yaml')
    odom = read(out / 'odometry_model_selection_report.yaml')
    odom_patch = read(out / 'selected_odometry_candidate_patch.yaml')
    drag = read(out / 'coastdown_drag_report.yaml')
    current = read(out / 'current_acceleration_report.yaml')
    transient = read(out / 'traction_transient_report.yaml')
    response = read(out / 'erpm_response_report.yaml')
    interface = read(out / 'accel_to_current_interface_report.yaml')
    capture = read(out / 'capture_completeness_report.yaml')

    surface_ready, surface_faults = _surface_ready(current, gates)
    requires_surface = bool(current.get('requires_nonlinear_longitudinal_model'))
    accel_model = 'traction_surface' if requires_surface else 'scalar'

    # The current surface is allowed into temporary Stage 12 verification even
    # when slip transients say a memoryless scalar model is inadequate.  The
    # actual candidate pulse evidence decides whether the chosen model survives;
    # rejecting it before the reversible test would conceal the high-demand
    # failure mode the campaign was built to expose.
    accepted = all([
        bool(capture.get('ok')),
        bool(odom.get('accepted_for_shadow_deployment_verification')),
        bool(drag.get('accepted_for_candidate')),
        bool(surface_ready),
        bool(response.get('accepted_for_candidate')),
        bool(interface.get('accepted_for_candidate')),
    ])

    patch = merge({}, odom_patch)
    ack = patch.setdefault('ackermann_to_vesc_node', {}).setdefault('ros__parameters', {})
    odom_node = patch.setdefault('vesc_to_odom_node', {}).setdefault('ros__parameters', {})
    env = cfg['operating_envelope']

    ack.update({
        'acceleration_command_model': accel_model,
        'accel_to_current_gain': float(current['candidate_accel_to_current_gain']),
        'accel_to_brake_gain': float(current['candidate_accel_to_brake_gain']),
        'accel_deadzone': float(current['candidate_accel_deadzone_mps2']),
        'accel_drag_coulomb': float(drag['accel_drag_coulomb_mps2']),
        'accel_drag_viscous': float(drag['accel_drag_viscous_per_s']),
        'accel_drag_quadratic': float(drag['accel_drag_quadratic_per_m']),
        'max_drive_current': float(env['approved_drive_test_current_a']),
        'max_brake_current': float(env['approved_brake_test_current_a']),
        'slow_start_threshold': float(speed['candidate_slow_start_threshold_mps']),
        'slow_start_increment': float(speed['candidate_slow_start_increment_mps']),
        'stop_speed_deadzone': float(speed['candidate_stop_speed_deadzone_mps']),
        'speed_to_braking_max': float(env['approved_brake_test_current_a']),
        'speed_to_erpm_offset': 0.0,
    })
    ack.update(_surface_patch(current['full_envelope_surface_fit'], 'drive'))
    ack.update(_surface_patch(current['full_envelope_surface_fit'], 'brake'))
    odom_node['speed_deadband'] = float(speed['candidate_odom_speed_deadband_mps'])

    selected_model = str(odom.get('selected_family', ''))
    selected_wheel = str(odom.get('selected_wheel_observation', ''))
    selected_command = str(odom.get('command_map_selected', ''))
    required_upgrade = {
        'full_stack_upgrade_required': True,
        'reason': (
            'The current production stack couples desired-speed-to-ERPM command conversion and measured-ERPM-to-odometry conversion. '
            'The selected candidate uses independent zero-intercept maps, plus an acceleration-aware shadow odometry estimator. '
            'A C++ port matching the shadow output and, when selected, the traction-surface current inversion is required before permanent installation.'
        ),
        'selected_odometry_model_family': selected_model,
        'selected_wheel_observation': selected_wheel,
        'selected_command_map': selected_command,
        'selected_acceleration_command_model': accel_model,
        'shadow_references': [
            'full_stack/adaptive_odom_shadow.py',
            'full_stack/candidate_command_map.py',
            'full_stack/candidate_accel_map.py',
        ],
        'production_port_contract': 'full_stack/PRODUCTION_PORT_CONTRACT.md',
    }
    report = {
        'accepted_for_temporary_candidate_verification': bool(accepted),
        'candidate_patch': patch,
        'required_full_stack_upgrade': required_upgrade,
        'acceleration_model_selection': {
            'selected_model': accel_model,
            'traction_surface_ready': bool(surface_ready),
            'traction_surface_faults': surface_faults,
            'scalar_model_adequate_over_envelope': bool(current.get('scalar_accel_to_current_adequate_over_envelope')),
            'dynamic_slip_requires_additional_state_model': bool(transient.get('requires_dynamic_longitudinal_slip_model')),
            'note': (
                'The reversible Stage 12 candidate launch tests the selected scalar or traction-surface mapper. '
                'A failed Stage 12 high-demand gate means a memoryless map is insufficient and the candidate is rejected; '
                'it does not get silently collapsed back to a low-slip scalar gain.'
            ),
        },
        'input_reports': {
            'capture': 'capture_completeness_report.yaml',
            'speed': 'erpm_speed_map_report.yaml',
            'odometry_model_selection': 'odometry_model_selection_report.yaml',
            'drag': 'coastdown_drag_report.yaml',
            'current': 'current_acceleration_report.yaml',
            'traction_transient': 'traction_transient_report.yaml',
            'response': 'erpm_response_report.yaml',
            'interface': 'accel_to_current_interface_report.yaml',
        },
        'not_automatically_installed': True,
        'zero_intercept_contract': 'speed_command_to_erpm(0) == 0; no fitted global ERPM offset is allowed.',
    }
    dump_yaml(out / 'candidate_vesc_patch.yaml', patch)
    dump_yaml(out / 'required_full_stack_upgrade.yaml', required_upgrade)
    dump_yaml(out / 'longitudinal_candidate_summary.yaml', report)
    print(json.dumps(report, indent=2, default=str))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
