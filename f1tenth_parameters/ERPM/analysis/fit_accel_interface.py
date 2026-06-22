#!/usr/bin/env python3
"""Audit ACCEL_TO_CURRENT command mapping using the temporary bootstrap profile."""
from __future__ import annotations
import argparse, json, math
from pathlib import Path
import numpy as np
import pandas as pd
from common import accepted_capture_windows, analysis_dir, dump_yaml, load_yaml, stage_tables


def main()->int:
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('session',type=Path); a=p.parse_args(); session=a.session.resolve(); cfg=load_yaml(session/'calibration_config_snapshot.yaml'); out=analysis_dir(session); tables=stage_tables(session,'10_accel_to_current_interface')
    import yaml
    current=yaml.safe_load((out/'current_acceleration_report.yaml').read_text(encoding='utf-8')); drag=yaml.safe_load((out/'coastdown_drag_report.yaml').read_text(encoding='utf-8'))
    windows=accepted_capture_windows(tables['events'],'accel_to_current_pulse'); rows=[]
    for _,w in windows.iterrows():
        ve=tables['vesc'][(tables['vesc'].bag_ns>=w.start_ns)&(tables['vesc'].bag_ns<=w.end_ns)]
        selc=tables['motor_selected_current'][(tables['motor_selected_current'].bag_ns>=w.start_ns)&(tables['motor_selected_current'].bag_ns<=w.end_ns)]
        selb=tables['motor_selected_brake'][(tables['motor_selected_brake'].bag_ns>=w.start_ns)&(tables['motor_selected_brake'].bag_ns<=w.end_ns)]
        cmd=float(w.get('acceleration_command_mps2',math.nan)); measured=float(np.nanmedian(ve.motor_current)) if len(ve) else math.nan
        selected_c=float(np.nanmedian(selc.value)) if len(selc) else math.nan; selected_b=float(np.nanmedian(selb.value)) if len(selb) else math.nan
        # This stage validates interface command delivery. Ground acceleration is
        # already identified from direct-current captures and held out separately.
        if cmd>=0:
            expected=float(current['candidate_accel_to_current_gain'])*cmd
            observed=selected_c
        else:
            expected=float(current['candidate_accel_to_brake_gain'])*abs(cmd)
            observed=selected_b
        rows.append({'trial_id':w.trial_id,'condition_id':w.condition_id,'acceleration_command_mps2':cmd,'selected_current_a':selected_c,'selected_brake_a':selected_b,'motor_current_a':measured,'expected_interface_output_a':expected,'interface_residual_a':observed-expected})
    table=pd.DataFrame(rows); table.to_parquet(out/'accel_to_current_interface_trials.parquet',index=False)
    rmse=float(np.sqrt(np.nanmean(table.interface_residual_a.to_numpy(dtype=float)**2))) if len(table) else math.inf
    report={'interface_output_rmse_a':rmse,'trials':int(len(table)),'accepted_for_candidate':bool(rmse<=float(cfg['analysis']['gates']['max_accel_interface_current_rmse_a'])),'gate_max_interface_output_rmse_a':float(cfg['analysis']['gates']['max_accel_interface_current_rmse_a']),'note':'Bootstrap gains are the original/fallback settings. This stage confirms command routing and reports mismatch; candidate gains are validated later through the held-out direct-current model and optional temporary deployment verification.'}
    dump_yaml(out/'accel_to_current_interface_report.yaml',report); print(json.dumps(report,indent=2)); return 0
if __name__=='__main__': raise SystemExit(main())
