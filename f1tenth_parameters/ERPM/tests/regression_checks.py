#!/usr/bin/env python3
"""Pure-Python regression checks for ERPM calibration file contracts.

These checks target failure modes that are cheap to catch before hardware: all
mandatory topic groups, nominal-condition grouping, explicit command-vs-measured
ERPM treatment, raw-current command audit coverage, and the VESC profile
transaction contract.
"""
from __future__ import annotations
import sys
from pathlib import Path
import numpy as np
import pandas as pd
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'analysis'))
from common import coverage, robust_linear


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    cfg=yaml.safe_load((ROOT/'config'/'erpm_calibration.yaml').read_text())
    topics=yaml.safe_load((ROOT/'config'/'topics.yaml').read_text())
    required={'command_audit','raw_erpm','ackermann_vel','raw_current','ackermann_accel'}
    require(required.issubset(topics['required']), 'missing required topic group')
    require(topics['recording']['record_all_topics'] is True, 'must record all topics')
    require(topics['recording']['include_hidden_topics'] is True, 'must record hidden topics')
    require({'/erpm_calibration/motor_raw_current','/erpm_calibration/motor_raw_brake'}.issubset(topics['required']['command_audit']), 'Stage 0 must verify direct current/brake command evidence')
    require(cfg['hardware']['lidar_ip_address']=='192.168.10.10', 'ERPM LiDAR IP must match steering campaign')
    require(abs(float(cfg['hardware']['laser_to_base_x_m'])-0.265)<1e-12, 'ERPM LiDAR x transform drifted from steering campaign')
    require(abs(float(cfg['hardware']['laser_to_base_z_m'])-0.05)<1e-12, 'ERPM LiDAR z transform drifted from steering campaign')
    require(float(cfg['analysis']['gates']['max_raw_erpm_delivery_error_erpm'])>0, 'missing raw selector delivery gate')

    # Exact telemetry values must never become nominal experimental buckets.
    jittered=pd.DataFrame({'nominal_speed_mps':[0.5]*5+[0.7]*5,
                           'erpm_measured':[2274.7,2275.1,2274.3,2275.5,2274.9,3181.1,3180.7,3181.3,3180.2,3181.0]})
    cov=coverage(jittered,['nominal_speed_mps'],5)
    require(bool(cov.coverage_ok.all()), 'nominal condition coverage broke under ERPM echo jitter')
    fit=robust_linear(np.array([1000,2000,3000,4000.]),np.array([.2,.4,.6,.8]))
    require(abs(fit['slope']-0.0002)<1e-9 and abs(fit['intercept'])<1e-9,'speed-map linear fit wrong')

    session=(ROOT/'erpm_calibration'/'session.py').read_text()
    transaction=(ROOT/'erpm_calibration'/'config_transaction.py').read_text()
    speed_fit=(ROOT/'analysis'/'fit_speed_map.py').read_text()
    stage=(ROOT/'erpm_calibration'/'stages.py').read_text()
    require("self.transaction.apply_profile('vel_to_erpm')" in session,'VEL profile transaction missing')
    require("self.transaction.apply_profile('accel_to_current_bootstrap')" in session,'ACCEL bootstrap transaction missing')
    require("candidate_velocity_verification" in session and "candidate_accel_verification" in session,'temporary candidate verification missing')
    require("shutil.copy2(self.backup,self.config_path)" in transaction,'byte-exact restoration missing')
    require("selected_ERPM" in speed_fit and "VESC_ERPM_measured" in speed_fit,'speed-map must distinguish command from measured drivetrain ERPM')
    require("raw_current" in stage and "raw_brake" in stage,'stage command audit must exercise raw current and brake routing')
    print('ERPM regression checks passed')
    return 0
if __name__=='__main__':
    raise SystemExit(main())
