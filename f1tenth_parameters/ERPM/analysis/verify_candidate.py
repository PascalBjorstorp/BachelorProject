#!/usr/bin/env python3
"""Evaluate the temporary candidate deployment against independent hold-outs."""
from __future__ import annotations
import argparse, json, math
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.signal import savgol_filter
from common import accepted_capture_windows, analysis_dir, dump_yaml, load_yaml, stage_tables, summarize_windows, straight_filter


def _accel(frame: pd.DataFrame, start: int, end: int) -> float:
    if frame.empty or 'valid' not in frame: return math.nan
    f=frame[(frame.bag_ns>=start)&(frame.bag_ns<=end)&frame.valid.astype(bool)].sort_values('bag_ns')
    if len(f)<8: return math.nan
    lo,hi=int(.2*len(f)),int(.8*len(f)); f=f.iloc[lo:hi]
    t=f.bag_ns.to_numpy(float)*1e-9; v=f.vx.to_numpy(float)
    if len(v)<5: return math.nan
    dt=float(np.median(np.diff(t))); w=max(5,int(round(.2/max(dt,1e-3)))); w+=1-w%2
    if w>=len(v): w=len(v)-1 if len(v)%2==0 else len(v)
    return float(np.nanmedian(np.gradient(savgol_filter(v,w,2,mode='interp') if w>=5 else v,t)))

def main()->int:
    p=argparse.ArgumentParser(); p.add_argument('session',type=Path); a=p.parse_args(); session=a.session.resolve(); cfg=load_yaml(session/'calibration_config_snapshot.yaml'); out=analysis_dir(session)
    vel=stage_tables(session,'11_candidate_velocity_verification'); vw=accepted_capture_windows(vel['events'],'candidate_velocity_verification'); vs=straight_filter(summarize_windows(vw,vel,cfg),cfg)
    if len(vs):
        e=vs.vx_lidar_mps.to_numpy(float)-vs.speed_command_mps.to_numpy(float); vel_rmse=float(np.sqrt(np.mean(e**2))); vel_bias=float(np.mean(e))
    else: vel_rmse=math.inf; vel_bias=math.nan
    vs.to_parquet(out/'candidate_velocity_verification_trials.parquet',index=False)
    acc=stage_tables(session,'12_candidate_accel_verification'); aw=accepted_capture_windows(acc['events'],'candidate_accel_verification'); rows=[]
    for _,w in aw.iterrows(): rows.append({'trial_id':w.trial_id,'condition_id':w.condition_id,'acceleration_command_mps2':float(w.acceleration_command_mps2),'measured_lidar_accel_mps2':_accel(acc['lidar_velocity'],int(w.start_ns),int(w.end_ns))})
    at=pd.DataFrame(rows); at.to_parquet(out/'candidate_accel_verification_trials.parquet',index=False)
    if len(at):
        e=at.measured_lidar_accel_mps2.to_numpy(float)-at.acceleration_command_mps2.to_numpy(float); acc_rmse=float(np.sqrt(np.nanmean(e**2))); acc_bias=float(np.nanmean(e))
    else: acc_rmse=math.inf; acc_bias=math.nan
    report={'candidate_velocity_rmse_mps':vel_rmse,'candidate_velocity_bias_mps':vel_bias,'candidate_acceleration_rmse_mps2':acc_rmse,'candidate_acceleration_bias_mps2':acc_bias,'accepted_for_permanent_review':bool(vel_rmse<=float(cfg['analysis']['gates']['max_speed_map_holdout_rmse_mps']) and acc_rmse<=float(cfg['analysis']['gates']['max_current_model_holdout_rmse_mps2'])),'note':'The candidate was deployed only inside the reversible transaction. This report does not modify the normal VESC configuration.'}
    dump_yaml(out/'candidate_deployment_verification_report.yaml',report); print(json.dumps(report,indent=2)); return 0
if __name__=='__main__': raise SystemExit(main())
