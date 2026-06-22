#!/usr/bin/env python3
"""Assemble reviewed longitudinal candidate parameters without installing them."""
from __future__ import annotations
import argparse,json,math
from pathlib import Path
import yaml
from common import analysis_dir,dump_yaml,load_yaml

def read(p:Path)->dict: return yaml.safe_load(p.read_text(encoding='utf-8'))
def main()->int:
    p=argparse.ArgumentParser(); p.add_argument('session',type=Path); a=p.parse_args(); session=a.session.resolve(); out=analysis_dir(session); cfg=load_yaml(session/'calibration_config_snapshot.yaml')
    speed=read(out/'erpm_speed_map_report.yaml'); drag=read(out/'coastdown_drag_report.yaml'); current=read(out/'current_acceleration_report.yaml'); response=read(out/'erpm_response_report.yaml'); interface=read(out/'accel_to_current_interface_report.yaml'); capture=read(out/'capture_completeness_report.yaml')
    accepted=all([bool(capture['ok']),bool(speed['accepted_for_candidate']),bool(drag['accepted_for_candidate']),bool(current['accepted_for_candidate']),bool(response['accepted_for_candidate']),bool(interface['accepted_for_candidate'])])
    stop_brake=float(current['candidate_accel_to_brake_gain'])*float(cfg['analysis']['desired_vel_stop_decel_mps2'])
    patch={
      '/**':{'ros__parameters':{'speed_to_erpm_gain':float(speed['candidate_speed_to_erpm_gain']),'speed_to_erpm_offset':float(speed['candidate_speed_to_erpm_offset'])}},
      'vesc_to_odom_node':{'ros__parameters':{'odom_speed_scale':float(speed['candidate_odom_speed_scale']),'speed_deadband':float(speed['candidate_odom_speed_deadband_mps'])}},
      'ackermann_to_vesc_node':{'ros__parameters':{
        'accel_to_current_gain':float(current['candidate_accel_to_current_gain']),'accel_to_brake_gain':float(current['candidate_accel_to_brake_gain']),'accel_deadzone':float(current['candidate_accel_deadzone_mps2']),
        'accel_drag_coulomb':float(drag['accel_drag_coulomb_mps2']),'accel_drag_viscous':float(drag['accel_drag_viscous_per_s']),'accel_drag_quadratic':float(drag['accel_drag_quadratic_per_m']),
        'slow_start_threshold':float(speed['candidate_slow_start_threshold_mps']),'slow_start_increment':float(speed['candidate_slow_start_increment_mps']),'stop_speed_deadzone':float(speed['candidate_stop_speed_deadzone_mps']),'speed_to_braking_max':float(stop_brake),
      }}}
    report={'accepted_for_temporary_candidate_verification':accepted,'candidate_patch':patch,'input_reports':{'capture':'capture_completeness_report.yaml','speed':'erpm_speed_map_report.yaml','drag':'coastdown_drag_report.yaml','current':'current_acceleration_report.yaml','response':'erpm_response_report.yaml','interface':'accel_to_current_interface_report.yaml'},'not_automatically_installed':True,'safety_parameters_not_identified':['max_drive_current','max_brake_current','max_regen_input_current','speed_min','speed_max','current_min','current_max','brake_min','brake_max'],'rationale':'Electrical and thermal safety limits require component/battery specifications and thermal qualification; the campaign logs them and reports required currents but does not claim to identify safe maxima.'}
    dump_yaml(out/'candidate_vesc_patch.yaml',patch); dump_yaml(out/'longitudinal_candidate_summary.yaml',report); print(json.dumps(report,indent=2,default=str)); return 0
if __name__=='__main__': raise SystemExit(main())
