#!/usr/bin/env python3
"""Static contract checks for the full-envelope ERPM model-selection suite."""
from pathlib import Path
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
assert float(cfg['operating_envelope']['maximum_test_speed_mps']) > 2.5
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

# Every explicit current condition must have enough conservative duration to
# produce a LiDAR-identifiable transient before reaching the declared envelope.
env = cfg['operating_envelope']
def _base_duration(f):
    spec = env['high_demand_pulse_duration_s']
    return float(spec['low_fraction'] if f <= 0.25 else spec['medium_fraction'] if f <= 0.55 else spec['high_fraction'])
def _safe_duration(f, v, polarity):
    if polarity == 'drive':
        headroom = float(env['maximum_test_speed_mps']) - float(env['speed_guard_margin_mps']) - float(v)
        accel = float(env['maximum_test_accel_mps2']) * float(f)
    else:
        headroom = float(v) - float(env['brake_guard_margin_mps'])
        accel = float(env['maximum_test_brake_mps2']) * float(f)
    return min(_base_duration(float(f)), headroom / max(accel, 1e-9))
for section in ('raw_current_training', 'raw_current_holdout'):
    for polarity in ('drive', 'brake'):
        for condition in cfg[section][f'{polarity}_conditions']:
            for fraction in condition['current_fractions']:
                assert _safe_duration(float(fraction), float(condition['initial_speed_mps']), polarity) >= float(env['dynamic_capture_min_s'])
assert (ROOT / 'full_stack' / 'candidate_accel_map.py').is_file()
assert (ROOT / 'docs' / 'SOURCE_PARAMETER_AUDIT.md').is_file()
print('ERPM full-envelope file-contract checks passed')
