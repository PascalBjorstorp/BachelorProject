#!/usr/bin/env python3
"""Verify the selected shadow/full-stack odometry candidate on complete hold-outs."""
from __future__ import annotations
import argparse, json, math
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.signal import savgol_filter
from common import accepted_capture_windows, analysis_dir, dump_yaml, expected_grid_coverage, expected_numeric_coverage, load_yaml, stage_tables, summarize_windows, straight_filter


def _metrics(measured: np.ndarray, predicted: np.ndarray) -> dict:
    m=np.isfinite(measured)&np.isfinite(predicted)
    if not np.any(m): return {'n':0,'rmse':math.inf,'bias':math.nan,'p95':math.inf}
    r=predicted[m]-measured[m]
    return {'n':int(m.sum()),'rmse':float(np.sqrt(np.mean(r*r))),'bias':float(np.mean(r)),'p95':float(np.quantile(np.abs(r),.95))}


def _accel(frame:pd.DataFrame,start:int,end:int,column:str)->float:
    f=frame[(frame.bag_ns>=start)&(frame.bag_ns<=end)].dropna(subset=[column]).sort_values('bag_ns')
    if len(f)<8:return math.nan
    lo,hi=int(.2*len(f)),int(.8*len(f)); f=f.iloc[lo:hi]
    if len(f)<5:return math.nan
    t=f.bag_ns.to_numpy(float)*1e-9; v=f[column].to_numpy(float); dt=float(np.median(np.diff(t)))
    window=max(5,int(round(.18/max(dt,1e-3)))); window += (window%2==0)
    if window>=len(v):window=len(v)-1 if len(v)%2==0 else len(v)
    smooth=savgol_filter(v,window,2,mode='interp') if window>=5 else v
    return float(np.nanmedian(np.gradient(smooth,t)))


def _candidate_pointwise(tables:dict, windows:pd.DataFrame)->pd.DataFrame:
    out=[]; cand=tables.get('candidate_odom',pd.DataFrame()); lidar=tables['lidar_velocity']; imu=tables['imu']
    if cand.empty or lidar.empty:return pd.DataFrame()
    c=cand[['bag_ns','vx']].dropna().sort_values('bag_ns')
    if len(c)<2:return pd.DataFrame()
    for _,w in windows.iterrows():
        li=lidar[(lidar.bag_ns>=int(w.start_ns))&(lidar.bag_ns<=int(w.end_ns))].copy()
        if 'valid' in li:li=li[li.valid.astype(bool)]
        if li.empty:continue
        tt=li.bag_ns.to_numpy(float)
        li['candidate_vx_mps']=np.interp(tt,c.bag_ns.to_numpy(float),c.vx.to_numpy(float),left=np.nan,right=np.nan)
        if not imu.empty:
            f=imu[['bag_ns','ax']].dropna().sort_values('bag_ns')
            li['imu_ax']=np.interp(tt,f.bag_ns.to_numpy(float),f.ax.to_numpy(float),left=np.nan,right=np.nan) if len(f)>=2 else math.nan
        else:li['imu_ax']=math.nan
        li['trial_id']=str(w.trial_id);li['condition_id']=str(w.condition_id)
        for key in ['speed_command_mps','acceleration_command_mps2','initial_speed_mps','steering_angle_rad']:
            if key in w:li[key]=w[key]
        out.append(li)
    return pd.concat(out,ignore_index=True) if out else pd.DataFrame()


def _window_candidate_summary(tables:dict, windows:pd.DataFrame,cfg:dict)->pd.DataFrame:
    base=summarize_windows(windows,tables,cfg)
    cand=tables.get('candidate_odom',pd.DataFrame())
    values=[]
    for _,w in windows.iterrows():
        f=cand[(cand.bag_ns>=int(w.start_ns))&(cand.bag_ns<=int(w.end_ns))] if not cand.empty else cand
        values.append(float(np.nanmedian(f.vx.to_numpy(float))) if not f.empty else math.nan)
    if len(base)==len(values):base['candidate_odom_vx_mps']=values
    return straight_filter(base,cfg)


def main()->int:
    p=argparse.ArgumentParser();p.add_argument('session',type=Path);a=p.parse_args();session=a.session.resolve();cfg=load_yaml(session/'calibration_config_snapshot.yaml');out=analysis_dir(session);g=cfg['analysis']['gates'];cv=cfg['candidate_verification'];sel=cfg['analysis']['odometry_model_selection']

    # Stage 11 plateaus: command map and candidate odom both face independent levels.
    vel=stage_tables(session,'11_candidate_velocity_verification');vw=accepted_capture_windows(vel['events'],'candidate_velocity_verification');vt=_window_candidate_summary(vel,vw,cfg)
    vcov=expected_numeric_coverage(vt,'speed_command_mps',cv['velocity_holdout_commands_mps'],int(cv['velocity_repetitions']));vcov.to_parquet(out/'candidate_velocity_verification_coverage.parquet',index=False);vt.to_parquet(out/'candidate_velocity_verification_trials.parquet',index=False)
    vel_cmd=_metrics(vt.get('vx_lidar_mps',pd.Series(dtype=float)).to_numpy(float),vt.get('speed_command_mps',pd.Series(dtype=float)).to_numpy(float))
    vel_odom=_metrics(vt.get('vx_lidar_mps',pd.Series(dtype=float)).to_numpy(float),vt.get('candidate_odom_vx_mps',pd.Series(dtype=float)).to_numpy(float))
    vel_cov_ok=bool(len(vcov)) and bool(vcov.coverage_ok.all())

    # Stage 12 dynamics: coverage by configured initial-speed/acceleration cell;
    # candidate speed is scored pointwise against LiDAR through the whole pulse.
    acc=stage_tables(session,'12_candidate_accel_verification');aw=accepted_capture_windows(acc['events'],'candidate_accel_verification');point=_candidate_pointwise(acc,aw)
    point.to_parquet(out/'candidate_dynamic_speed_samples.parquet',index=False)
    rows=[]
    for _,w in aw.iterrows():
        rows.append({'trial_id':str(w.trial_id),'condition_id':str(w.condition_id),'initial_speed_mps':float(w.get('initial_speed_mps',math.nan)),'acceleration_command_mps2':float(w.get('acceleration_command_mps2',math.nan)),'lidar_accel_mps2':_accel(acc['lidar_velocity'],int(w.start_ns),int(w.end_ns),'vx'),'candidate_odom_accel_mps2':_accel(acc.get('candidate_odom',pd.DataFrame()),int(w.start_ns),int(w.end_ns),'vx')})
    atrials=pd.DataFrame(rows)
    atrials['measurement_valid']=np.isfinite(atrials.lidar_accel_mps2) if not atrials.empty else []
    atrials.to_parquet(out/'candidate_accel_verification_trials.parquet',index=False)
    usable=atrials[atrials.measurement_valid.astype(bool)].copy() if not atrials.empty else atrials
    grid=[(float(v),float(q)) for v in cv['acceleration_initial_speeds_mps'] for q in cv['acceleration_holdout_commands_mps2']]
    acov=expected_grid_coverage(usable,fields=['initial_speed_mps','acceleration_command_mps2'],expected_grid=grid,expected_repetitions=int(cv['acceleration_repetitions']),tolerances={'initial_speed_mps':1e-6,'acceleration_command_mps2':1e-6});acov.to_parquet(out/'candidate_accel_verification_coverage.parquet',index=False)
    accel_command=_metrics(usable.get('lidar_accel_mps2',pd.Series(dtype=float)).to_numpy(float),usable.get('acceleration_command_mps2',pd.Series(dtype=float)).to_numpy(float))
    accel_odom=_metrics(usable.get('lidar_accel_mps2',pd.Series(dtype=float)).to_numpy(float),usable.get('candidate_odom_accel_mps2',pd.Series(dtype=float)).to_numpy(float))
    # Hold-speed: commanding a=0 must actually hold speed (drag feed-forward),
    # not coast. Residual ground acceleration at a=0 is gated near zero.
    hold=usable[usable.acceleration_command_mps2.abs()<1e-6] if not usable.empty else usable
    hold_speed=_metrics(hold.get('lidar_accel_mps2',pd.Series(dtype=float)).to_numpy(float),np.zeros(len(hold)))
    hold_ok=bool(hold_speed['n']>0 and hold_speed['rmse']<=float(g['max_hold_speed_accel_mps2']))
    dyn_odom=_metrics(point.get('vx',pd.Series(dtype=float)).to_numpy(float),point.get('candidate_vx_mps',pd.Series(dtype=float)).to_numpy(float))
    acoast,ahigh=map(float,sel['acceleration_regimes_mps2']); drive=point[point.imu_ax>=ahigh] if not point.empty else point; brake=point[point.imu_ax<=-ahigh] if not point.empty else point
    dyn_drive=_metrics(drive.get('vx',pd.Series(dtype=float)).to_numpy(float),drive.get('candidate_vx_mps',pd.Series(dtype=float)).to_numpy(float));dyn_brake=_metrics(brake.get('vx',pd.Series(dtype=float)).to_numpy(float),brake.get('candidate_vx_mps',pd.Series(dtype=float)).to_numpy(float))
    vel_ok=(vel_cmd['rmse']<=float(g['max_speed_map_holdout_rmse_mps']) and abs(vel_cmd['bias'])<=float(g['max_speed_map_holdout_bias_mps']) and vel_odom['rmse']<=float(g['max_odom_holdout_rmse_mps']) and abs(vel_odom['bias'])<=float(g['max_odom_holdout_bias_mps']) and vel_odom['p95']<=float(g['max_odom_holdout_p95_abs_error_mps']))
    dyn_ok=(dyn_odom['rmse']<=float(g['max_odom_holdout_rmse_mps']) and abs(dyn_odom['bias'])<=float(g['max_odom_holdout_bias_mps']) and dyn_drive['rmse']<=float(g['max_high_accel_odom_rmse_mps']) and abs(dyn_drive['bias'])<=float(g['max_high_accel_odom_bias_mps']) and dyn_brake['rmse']<=float(g['max_high_brake_odom_rmse_mps']) and abs(dyn_brake['bias'])<=float(g['max_high_brake_odom_bias_mps']))
    acc_ok=(accel_command['rmse']<=float(g['max_current_model_holdout_rmse_mps2']) and abs(accel_command['bias'])<=float(g['max_candidate_acceleration_bias_mps2']))
    # Stage 13: independent non-zero-steering check. It validates the selected
    # speed estimator in the combined operating state but is never used to fit it.
    cross=stage_tables(session,'13_candidate_cross_axis_verification'); cw=accepted_capture_windows(cross['events'],'candidate_cross_axis_verification'); cpoints=_candidate_pointwise(cross,cw)
    cpoints.to_parquet(out/'candidate_cross_axis_speed_samples.parquet',index=False)
    sx=cfg['cross_axis_validation']; cgrid=[(float(angle),float(speed)) for angle in sx['steering_angle_rad'] for speed in sx['speed_commands_mps']]
    ccov=expected_grid_coverage(cpoints,fields=['steering_angle_rad','speed_command_mps'],expected_grid=cgrid,expected_repetitions=int(sx['repetitions']),tolerances={'steering_angle_rad':1e-6,'speed_command_mps':1e-6},unique_by=['trial_id']) if bool(sx.get('enabled',False)) else pd.DataFrame()
    if not ccov.empty: ccov.to_parquet(out/'candidate_cross_axis_coverage.parquet',index=False)
    cross_metric=_metrics(cpoints.get('vx',pd.Series(dtype=float)).to_numpy(float),cpoints.get('candidate_vx_mps',pd.Series(dtype=float)).to_numpy(float))
    cross_cov_ok=(not bool(sx.get('enabled',False))) or (bool(len(ccov)) and bool(ccov.coverage_ok.all()))
    cross_ok=(not bool(sx.get('enabled',False))) or (
        cross_cov_ok
        and cross_metric['rmse']<=float(g['max_cross_axis_odom_rmse_mps'])
        and abs(cross_metric['bias'])<=float(g['max_cross_axis_odom_bias_mps'])
        and cross_metric['p95']<=float(g['max_cross_axis_odom_p95_abs_error_mps'])
    )
    odom_accel_ok=(
        accel_odom['rmse']<=float(g['max_candidate_odom_acceleration_rmse_mps2'])
        and abs(accel_odom['bias'])<=float(g['max_candidate_odom_acceleration_bias_mps2'])
    )
    accepted=bool(vel_cov_ok and bool(acov.coverage_ok.all()) and vel_ok and dyn_ok and acc_ok and odom_accel_ok and cross_ok and hold_ok)
    report={'candidate_velocity_command':vel_cmd,'candidate_velocity_odom':vel_odom,'candidate_velocity_coverage_ok':vel_cov_ok,'candidate_dynamic_odom':dyn_odom,'candidate_dynamic_high_drive':dyn_drive,'candidate_dynamic_high_brake':dyn_brake,'candidate_accel_command':accel_command,'candidate_accel_odom_derivative':accel_odom,'candidate_hold_speed_residual_accel':hold_speed,'candidate_acceleration_coverage_ok':bool(len(acov)) and bool(acov.coverage_ok.all()),'candidate_cross_axis_odom':cross_metric,'candidate_cross_axis_coverage_ok':cross_cov_ok,'velocity_gates_ok':vel_ok,'dynamic_odom_gates_ok':dyn_ok,'accel_interface_gates_ok':acc_ok,'candidate_odom_acceleration_gate_ok':odom_accel_ok,'cross_axis_gate_ok':cross_ok,'hold_speed_gate_ok':hold_ok,'accepted_for_permanent_review':accepted,'note':'Candidate shadow odometry is compared directly with LiDAR velocity during settled, high-demand, hold-speed (a=0) and non-zero-steering candidate hold-outs. Coverage, command tracking, RMSE, signed bias, high-drive/high-brake error and a=0 speed-hold all gate acceptance.'}
    dump_yaml(out/'candidate_deployment_verification_report.yaml',report);print(json.dumps(report,indent=2));return 0
if __name__=='__main__':raise SystemExit(main())
