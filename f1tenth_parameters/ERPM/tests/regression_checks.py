#!/usr/bin/env python3
"""Cheap, hardware-free guards for the model-selection campaign wiring."""
from __future__ import annotations
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'analysis'))
from common import coverage, expected_grid_coverage, expected_numeric_coverage


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    cfg = yaml.safe_load((ROOT / 'config' / 'erpm_calibration.yaml').read_text())
    topics = yaml.safe_load((ROOT / 'config' / 'topics.yaml').read_text())

    required = {
        'command_audit', 'raw_erpm', 'ackermann_vel', 'raw_current',
        'ackermann_accel', 'candidate_ackermann_vel', 'candidate_ackermann_accel',
        'post_calibration_steering',
    }
    require(required.issubset(topics['required']), 'missing required topic group')
    require(topics['recording']['record_all_topics'] is False, 'fast campaign must use targeted topic recording')
    require(topics['recording'].get('compression_mode') == 'file', 'MCAP recording should use file compression')
    require(topics['recording'].get('compression_format') == 'zstd', 'MCAP recording should use zstd compression')
    require('/tf' in topics['redundancy_topics'] and '/parameter_events' in topics['redundancy_topics'],
            'targeted bagging must still retain tf and parameter events')
    candidate_topics = set(topics['required']['candidate_ackermann_vel'])
    candidate_accel_topics = set(topics['required']['candidate_ackermann_accel'])
    require('/erpm_calibration/candidate_odom' in candidate_topics, 'candidate odom must be recorded')
    require('/erpm_calibration/candidate_odom/wheel_weight' in candidate_topics, 'candidate fusion debug must be recorded')
    require('/erpm_calibration/candidate_accel/target_net_accel' in candidate_accel_topics, 'candidate traction-map debug must be recorded')
    require('/erpm_calibration/candidate_accel/drive_current_cmd' in candidate_accel_topics, 'candidate drive-current command must be recorded')
    require('/erpm_calibration/candidate_accel/brake_current_cmd' in candidate_accel_topics, 'candidate brake-current command must be recorded')
    require(float(cfg['profiles']['vel_to_erpm']['speed_to_erpm_offset']) == 0.0, 'ERPM command zero intercept violated')
    require(max(float(f) for c in cfg['raw_current_training']['drive_conditions'] for f in c['current_fractions']) >= 0.75, 'drive-current grid no longer reaches meaningful high demand')
    require(max(float(f) for c in cfg['raw_current_training']['brake_conditions'] for f in c['current_fractions']) >= 0.75, 'brake-current grid no longer reaches meaningful high demand')
    require(int(cfg['session']['cooling_pause_every_accepted_trials']) == 0, 'fast campaign should not inject scheduled cooling pauses')
    require(float(cfg['session']['min_free_disk_gb']) <= 12.0, 'disk preflight was not reduced for targeted recording')

    # Echo/telemetry jitter cannot create nominal-condition buckets.
    jittered = pd.DataFrame({
        'nominal_speed_mps': [0.5] * 5 + [0.7] * 5,
        'erpm_measured': [2274.7, 2275.1, 2274.3, 2275.5, 2274.9, 3181.1, 3180.7, 3181.3, 3180.2, 3181.0],
    })
    cov = coverage(jittered, ['nominal_speed_mps'], 5)
    require(bool(cov.coverage_ok.all()), 'nominal coverage broke under ERPM echo jitter')

    # Final candidate coverage must retain missing configured conditions.
    candidate = pd.DataFrame({'speed_command_mps': [0.28, 0.2800002, 0.46, 0.4599998, 0.64, 0.6400001]})
    candidate_cov = expected_numeric_coverage(candidate, 'speed_command_mps', [0.28, 0.46, 0.64, 0.82], 3)
    require(not bool(candidate_cov.coverage_ok.all()), 'partial candidate verification must fail coverage')
    require(int(candidate_cov.loc[np.isclose(candidate_cov.speed_command_mps, 0.82), 'accepted_usable_trials'].iloc[0]) == 0,
            'missing configured condition vanished from coverage')
    # Pointwise cross-axis verification must count accepted trials, not samples
    # within one accepted run.
    cross_axis = pd.DataFrame({
        'trial_id': ['A'] * 5 + ['B'] * 5,
        'steering_angle_rad': [0.04] * 10,
        'speed_command_mps': [1.5] * 10,
    })
    cross_cov = expected_grid_coverage(
        cross_axis,
        fields=['steering_angle_rad', 'speed_command_mps'],
        expected_grid=[(0.04, 1.5)],
        expected_repetitions=3,
        tolerances={'steering_angle_rad': 1e-6, 'speed_command_mps': 1e-6},
        unique_by=['trial_id'],
    )
    require(int(cross_cov['accepted_usable_trials'].iloc[0]) == 2,
            'cross-axis coverage must count accepted trials, not point samples')
    require(not bool(cross_cov.coverage_ok.iloc[0]),
            'two accepted steering runs must not satisfy a three-repetition hold-out')

    session = (ROOT / 'erpm_calibration' / 'session.py').read_text()
    transaction = (ROOT / 'erpm_calibration' / 'config_transaction.py').read_text()
    selector = (ROOT / 'analysis' / 'fit_odom_model_selection.py').read_text()
    verify = (ROOT / 'analysis' / 'verify_candidate.py').read_text()
    export = (ROOT / 'analysis' / 'export_bag.py').read_text()
    common = (ROOT / 'analysis' / 'common.py').read_text()
    launch = (ROOT / 'launch' / 'calibration_stack.py').read_text()
    stage = (ROOT / 'erpm_calibration' / 'stages.py').read_text()
    bagging = (ROOT / 'erpm_calibration' / 'bagging.py').read_text()
    runtime = (ROOT / 'erpm_calibration' / 'runtime.py').read_text()
    motor_selector = (ROOT / 'erpm_calibration' / 'motor_selector.py').read_text()

    require('self._verify_site_envelope()' in session, 'full-envelope site preflight missing')
    require('run_preflight_only' in session, 'preflight-only dry run helper missing')
    require("self.transaction.apply_profile('vel_to_erpm')" in session, 'VEL profile transaction missing')
    require("self.transaction.apply_profile('vel_to_erpm_interim'" in session, 'interim velocity checkpoint profile missing')
    require("self.transaction.apply_profile('accel_to_current_interim'" in session, 'interim accel checkpoint profile missing')
    require('_run_velocity_checkpoint' in session and '_run_forward_motion_checkpoint' in session,
            'automatic stage-order checkpoints missing')
    require("vel_to_erpm_candidate_cross_axis" in session, 'Stage 13 must re-launch velocity candidate profile')
    require('shutil.copy2(self.backup,self.config_path)' in transaction, 'byte-exact restoration missing')
    require('raw_drive_current_pulse' in selector and 'raw_brake_current_pulse' in selector,
            'dynamic selector must use actual Stage 8/9 phase names')
    require('command_map_and_wheel_observation_are_separate' in selector,
            'command and odom maps must remain decoupled')
    require('adaptive_wheel' in selector and 'fused_adaptive' in selector,
            'adaptive odometry candidate families missing')
    require('expected_grid_coverage' in verify, 'candidate verification must enforce configured coverage')
    require('max_cross_axis_odom_bias_mps' in verify and 'max_cross_axis_odom_p95_abs_error_mps' in verify,
            'cross-axis speed validation must gate bias and p95')
    require('candidate_odom_acceleration_gate_ok' in verify, 'candidate speed derivative must be validated')
    require("unique_by=['trial_id']" in verify, 'cross-axis coverage must deduplicate point samples by accepted trial')
    require('candidate_accel_debug' in export and 'candidate_accel_debug' in common,
            'candidate acceleration shadow debug must be exported into derived analysis tables')
    require('adaptive_odom_shadow.py' in launch and 'candidate_command_map.py' in launch and 'candidate_accel_map.py' in launch,
            'live velocity/acceleration shadow validation graph missing')
    require('recorded_topics' in bagging and 'record_all_topics' in bagging and '--compression-mode' in bagging,
            'bagger must support targeted topic recording')
    require('approved_drive_test_current_a' in stage and 'approved_brake_test_current_a' in stage,
            'current grid must use approved full-envelope limits')
    require("selector_mode='ackermann_speed'" in runtime and "selector_mode='ackermann_accel'" in runtime,
            'Ackermann speed and accel selector modes must stay separate')
    require("self.mode in {'ackermann', 'ackermann_speed'}" in motor_selector,
            'Ackermann speed mode must accept speed output')
    require("self.mode in {'ackermann', 'ackermann_accel'}" in motor_selector,
            'Ackermann accel mode must accept current/brake output')
    require('zero current/brake during speed mode can cancel the speed controller' in motor_selector,
            'selector must document that active speed mode may not publish real zero current/brake commands')
    require('capture_speed_gate' in stage and '_capture_speed_ok' in stage,
            'Ackermann speed captures must fail automatically if requested speed was not reached')
    cli = (ROOT / 'erpm_calibration' / 'cli.py').read_text()
    require('--preflight-only' in cli, 'CLI must expose a no-drive preflight mode')
    run_analysis = (ROOT / 'analysis' / 'run_analysis.py').read_text()
    require('summarize_forward_motion.py' in run_analysis, 'forward-motion synthesis report must be part of offline analysis')
    print('ERPM model-selection regression checks passed')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
