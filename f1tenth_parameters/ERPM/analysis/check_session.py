#!/usr/bin/env python3
"""Fail offline analysis when the stage-targeted campaign has incomplete capture evidence."""
from __future__ import annotations
import argparse, json
from pathlib import Path
import yaml
from common import STAGES, analysis_dir, dump_yaml, load_yaml

def main()->int:
    p=argparse.ArgumentParser(); p.add_argument('session',type=Path); a=p.parse_args(); session=a.session.resolve(); cfg=load_yaml(session/'calibration_config_snapshot.yaml'); policy=load_yaml(session/'recording_policy_snapshot.yaml'); manifest=load_yaml(session/'session_manifest.yaml'); report={'session':str(session),'stage_checks':{},'ok':True}
    groups={'00_command_chain_audit':'command_audit','01_longitudinal_observability':'raw_erpm','02_low_speed_launch':'raw_erpm','03_raw_erpm_map_training':'raw_erpm','04_raw_erpm_map_holdout':'raw_erpm','05_vel_to_erpm_pipeline_audit':'ackermann_vel','06_raw_erpm_response':'raw_erpm','07_coastdown':'raw_current','08_raw_current_training':'raw_current','09_raw_current_holdout':'raw_current','10_accel_to_current_interface':'ackermann_accel'}
    for stage in STAGES:
        item=manifest.get('stages',{}).get(stage,{}); counts_path=session/stage/'bag_topic_counts.yaml'; counts=yaml.safe_load(counts_path.read_text(encoding='utf-8')) if counts_path.is_file() else {}
        ok=item.get('status')=='completed' and bool(counts.get('ok'))
        report['stage_checks'][stage]={'manifest_status':item.get('status'),'topic_group':groups[stage],'bag_verification_ok':counts.get('ok'),'missing_or_empty_required_topics':counts.get('missing_or_empty_required_topics',[]),'ok':ok}
        report['ok'] &= ok
    dump_yaml(analysis_dir(session)/'capture_completeness_report.yaml',report); print(json.dumps(report,indent=2)); return 0 if report['ok'] else 2
if __name__=='__main__': raise SystemExit(main())
