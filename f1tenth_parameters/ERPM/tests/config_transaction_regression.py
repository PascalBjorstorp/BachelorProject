#!/usr/bin/env python3
"""Isolated regression test for reversible ERPM VESC configuration transaction."""
from __future__ import annotations

import sys
import tempfile
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from erpm_calibration.config_transaction import VescModeTransaction


def main() -> int:
    with tempfile.TemporaryDirectory(prefix='erpm_tx_test_') as temp:
        workspace = Path(temp)
        cfg_path = workspace / 'f1tenth_system/f1tenth_stack/config/vesc.yaml'
        cfg_path.parent.mkdir(parents=True)
        original = '''/**:
  ros__parameters:
    speed_to_erpm_gain: 4550.0
    speed_to_erpm_offset: 0.0
    servo_min: 0.190
    servo_max: 0.910
vesc_to_odom_node:
  ros__parameters:
    odom_speed_scale: 1.03
    speed_deadband: 0.15
ackermann_to_vesc_node:
  ros__parameters:
    operation_mode: ACCEL_TO_CURRENT
    accel_to_current_gain: 5.82
    accel_to_brake_gain: 7.38
    accel_deadzone: 0.02
    accel_drag_coulomb: 1.60
    accel_drag_viscous: 0.0
    accel_drag_quadratic: 0.0
    slow_start_threshold: 1.0
    slow_start_increment: 0.4
    stop_speed_deadzone: 0.05
    speed_to_braking_max: 20000.0
'''
        cfg_path.write_text(original, encoding='utf-8')

        session = workspace / 'session'
        session.mkdir()
        profiles = {
            'accel_to_current_bootstrap': {
                'fallback_accel_to_current_gain': 5.0,
                'fallback_accel_to_brake_gain': 6.0,
            }
        }
        tx = VescModeTransaction(
            calibration_root=workspace / 'f1tenth_parameters/ERPM',
            session_dir=session,
            workspace=workspace,
            config_relpath='f1tenth_system/f1tenth_stack/config/vesc.yaml',
            build_command=[sys.executable, '-c', 'raise SystemExit(0)'],
            profiles=profiles,
        )

        tx.begin()
        tx.apply_profile('vel_to_erpm')
        doc = yaml.safe_load(cfg_path.read_text())
        ack = doc['ackermann_to_vesc_node']['ros__parameters']
        assert ack['accel_to_current_gain'] == 0.0 and ack['accel_to_brake_gain'] == 0.0

        candidate = {
            'speed_to_erpm_gain': 5000.0,
            'speed_to_erpm_offset': 12.0,
            'odom_speed_scale': 0.98,
            'accel_to_current_gain': 6.1,
            'accel_to_brake_gain': 7.1,
            'accel_deadzone': 0.03,
            'accel_drag_coulomb': 1.4,
            'accel_drag_viscous': 0.1,
            'accel_drag_quadratic': 0.01,
            'slow_start_threshold': 0.18,
            'slow_start_increment': 0.18,
            'stop_speed_deadzone': 0.05,
            'speed_to_braking_max': 14.2,
        }

        tx.apply_profile('vel_to_erpm_interim', candidate_patch=candidate)
        doc = yaml.safe_load(cfg_path.read_text())
        assert doc['/**']['ros__parameters']['speed_to_erpm_gain'] == 5000.0
        assert doc['/**']['ros__parameters']['speed_to_erpm_offset'] == 0.0
        assert doc['vesc_to_odom_node']['ros__parameters']['odom_speed_scale'] == 0.98

        tx.apply_profile('vel_to_erpm_candidate', candidate_patch=candidate)
        doc = yaml.safe_load(cfg_path.read_text())
        assert doc['/**']['ros__parameters']['speed_to_erpm_gain'] == 5000.0
        assert doc['/**']['ros__parameters']['speed_to_erpm_offset'] == 0.0
        assert doc['vesc_to_odom_node']['ros__parameters']['odom_speed_scale'] == 0.98

        tx.apply_profile('accel_to_current_interim', candidate_patch=candidate)
        doc = yaml.safe_load(cfg_path.read_text())
        ack = doc['ackermann_to_vesc_node']['ros__parameters']
        assert ack['accel_to_current_gain'] == 6.1 and ack['accel_to_brake_gain'] == 7.1

        tx.apply_profile('accel_to_current_candidate', candidate_patch=candidate)
        doc = yaml.safe_load(cfg_path.read_text())
        ack = doc['ackermann_to_vesc_node']['ros__parameters']
        assert ack['accel_to_current_gain'] == 6.1 and ack['accel_to_brake_gain'] == 7.1

        tx.restore(build=True)
        assert cfg_path.read_text(encoding='utf-8') == original
        assert not (workspace / '.ERPM_CALIBRATION_RECOVERY.json').exists()

    print('ERPM configuration transaction regression passed')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
