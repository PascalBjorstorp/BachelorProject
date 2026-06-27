#!/usr/bin/env python3
"""Static contract checks for the full-envelope ERPM model-selection suite."""
from pathlib import Path
import math
import yaml

ROOT = Path(__file__).resolve().parents[1]
cfg = yaml.safe_load((ROOT / 'config' / 'erpm_calibration.yaml').read_text())
readme = (ROOT / 'README.md').read_text()
operator_card = (ROOT / 'docs' / 'OPERATOR_CARD.md').read_text()

assert '| 13 |' in readme
assert 'full-envelope' in readme.lower()
assert int(cfg['raw_erpm_map_training']['repetitions']) == 3
assert int(cfg['raw_erpm_map_holdout']['repetitions']) == 2
assert int(cfg['raw_current_training']['repetitions']) == 3
assert int(cfg['raw_current_holdout']['repetitions']) == 2
assert bool(cfg['site']['require_full_envelope_track'])
assert float(cfg['site']['straight_usable_length_m']) >= float(cfg['site']['minimum_full_envelope_length_m'])
assert float(cfg['site']['straight_usable_length_m']) == 20.0
min_test_speed = float(cfg['operating_envelope']['minimum_test_speed_mps'])
assert min_test_speed == 0.5
assert float(cfg['operating_envelope']['maximum_test_speed_mps']) >= 9.0
for section, key in (
    ('observability', 'straight_probe_speeds_mps'),
    ('low_speed_launch', 'nominal_speeds_mps'),
    ('raw_erpm_map_training', 'nominal_speeds_mps'),
    ('raw_erpm_map_holdout', 'nominal_speeds_mps'),
    ('vel_to_erpm_pipeline_audit', 'speed_commands_mps'),
    ('coastdown', 'initial_speeds_mps'),
    ('accel_to_current_interface', 'initial_speeds_mps'),
    ('candidate_verification', 'velocity_holdout_commands_mps'),
    ('candidate_verification', 'acceleration_initial_speeds_mps'),
):
    assert min(float(v) for v in cfg[section][key]) >= min_test_speed, (section, key)
for pair in cfg['raw_erpm_response']['steps_mps']:
    assert min(float(v) for v in pair) >= min_test_speed, pair
assert float(cfg['low_speed_launch']['minimum_lidar_speed_mps']) >= min_test_speed
assert max(float(f) for c in cfg['raw_current_training']['drive_conditions'] for f in c['current_fractions']) >= 0.75
assert max(float(f) for c in cfg['raw_current_training']['brake_conditions'] for f in c['current_fractions']) >= 0.75
assert float(cfg['operating_envelope']['approved_drive_test_current_a']) >= 0.0
assert float(cfg['operating_envelope']['approved_brake_test_current_a']) >= 0.0
assert float(cfg['profiles']['vel_to_erpm']['speed_to_erpm_offset']) == 0.0
assert 'max_attempts_per_condition' not in cfg.get('session', {})
assert int(cfg['session']['cooling_pause_every_accepted_trials']) == 0
assert float(cfg['session']['min_free_disk_gb']) <= 12.0
assert 'approved_drive_test_current_a' in operator_card
assert 'approved_brake_test_current_a' in operator_card

# Every explicit current condition must have enough configured duration to
# produce a LiDAR-identifiable transient. The runner must not shorten pulses
# using a software acceleration cap.
env = cfg['operating_envelope']
def _base_duration(f):
    spec = env['high_demand_pulse_duration_s']
    return float(spec['low_fraction'] if f <= 0.25 else spec['medium_fraction'] if f <= 0.55 else spec['high_fraction'])
for section in ('raw_current_training', 'raw_current_holdout'):
    for polarity in ('drive', 'brake'):
        for condition in cfg[section][f'{polarity}_conditions']:
            assert float(condition['initial_speed_mps']) >= min_test_speed
            for fraction in condition['current_fractions']:
                assert _base_duration(float(fraction)) >= float(env['dynamic_capture_min_s'])
cross = cfg['cross_axis_validation']
assert bool(cross['enabled'])
assert len(cross['conditions']) >= 30
wheelbase = float(cross['wheelbase_m'])
lat_limit = float(cross['max_expected_lateral_accel_mps2'])
assert lat_limit == 7.5
turn_duration = float(cross.get('pre_turn_hold_s', 0.0)) + float(cross['capture_s'])
for condition in cross['conditions']:
    speed = float(condition['speed_mps'])
    steer = abs(float(condition['steering_angle_rad']))
    ay = speed * speed * math.tan(steer) / wheelbase
    radius = speed * speed / ay
    yaw = ay / speed * turn_duration
    forward = radius * math.sin(yaw)
    lateral = radius * (1.0 - math.cos(yaw))
    assert speed >= min_test_speed
    assert speed <= float(env['maximum_test_speed_mps'])
    assert ay <= lat_limit + 0.03, (condition, ay)
    assert abs(ay - float(condition['expected_lateral_accel_mps2'])) <= 0.08, (condition, ay)
    assert forward <= 6.0 and lateral <= 1.6, (condition, forward, lateral)
steering = cfg['post_calibration_steering_dynamics']
assert bool(steering['enabled'])
assert len(steering['conditions']) >= 12
assert int(steering['repetitions']) == 1
assert float(steering['capture_s']) <= 0.45
wheelbase = float(steering['wheelbase_m'])
lat_limit = float(steering['max_expected_lateral_accel_mps2'])
assert lat_limit == 7.5
turn_duration = float(steering.get('pre_turn_hold_s', 0.0)) + float(steering['capture_s'])
for condition in steering['conditions']:
    speed = float(condition['speed_mps'])
    steer = abs(float(condition['steering_angle_rad']))
    ay = speed * speed * math.tan(steer) / wheelbase
    radius = speed * speed / ay
    yaw = ay / speed * turn_duration
    forward = radius * math.sin(yaw)
    lateral = radius * (1.0 - math.cos(yaw))
    assert speed >= min_test_speed
    assert speed <= float(env['maximum_test_speed_mps'])
    assert ay <= lat_limit + 0.03, (condition, ay)
    assert abs(ay - float(condition['expected_lateral_accel_mps2'])) <= 0.08, (condition, ay)
    assert forward <= 5.5 and lateral <= 1.3, (condition, forward, lateral)
assert (ROOT / 'full_stack' / 'candidate_accel_map.py').is_file()
assert (ROOT / 'docs' / 'SOURCE_PARAMETER_AUDIT.md').is_file()
print('ERPM full-envelope file-contract checks passed')
