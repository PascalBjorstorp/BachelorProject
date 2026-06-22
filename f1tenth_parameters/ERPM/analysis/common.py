"""Shared offline analysis utilities for ERPM calibration."""
from __future__ import annotations
import json, math
from pathlib import Path
from typing import Iterable
import numpy as np
import pandas as pd
import yaml

ROOT=Path(__file__).resolve().parents[1]

STAGES=[
 '00_command_chain_audit','01_longitudinal_observability','02_low_speed_launch','03_raw_erpm_map_training','04_raw_erpm_map_holdout','05_vel_to_erpm_pipeline_audit','06_raw_erpm_response','07_coastdown','08_raw_current_training','09_raw_current_holdout','10_accel_to_current_interface'
]

def load_yaml(path:Path)->dict:
    value=yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    if not isinstance(value,dict): raise ValueError(f'expected mapping: {path}')
    return value

def dump_yaml(path:Path,value:dict)->None:
    path.parent.mkdir(parents=True,exist_ok=True); path.write_text(yaml.safe_dump(value,sort_keys=False),encoding='utf-8')

def analysis_dir(session:Path)->Path:
    p=session/'analysis'; p.mkdir(exist_ok=True); return p

def stage_dir(session:Path,name:str)->Path: return session/name

def read_table(path:Path)->pd.DataFrame:
    return pd.read_parquet(path) if path.is_file() else pd.DataFrame()

def stage_tables(session:Path,name:str)->dict[str,pd.DataFrame]:
    d=stage_dir(session,name)/'derived'
    return {key:read_table(d/f'{key}.parquet') for key in ['events','imu','vesc','odom','motor_command','motor_raw_speed','motor_raw_current','motor_raw_brake','motor_selected_speed','motor_selected_current','motor_selected_brake','drive','ackermann','lidar_velocity','topic_index']}

def accepted_trial_ids(events:pd.DataFrame)->set[str]:
    if events.empty or not {'event','trial_id'}.issubset(events.columns): return set()
    d=events[events.event.astype(str)=='trial_decision'].copy()
    if d.empty: return set()
    d=d.sort_values('bag_ns').groupby('trial_id',as_index=False).tail(1)
    return {str(x) for x in d.loc[d.decision.astype(str)=='accepted','trial_id'].dropna()}

def accepted_capture_windows(events:pd.DataFrame, phases:str|Iterable[str]|None=None)->pd.DataFrame:
    if events.empty: return pd.DataFrame()
    accepted=accepted_trial_ids(events)
    starts=events[(events.event.astype(str)=='phase_start') & (events.get('capture',False)==True)].copy()
    ends=events[(events.event.astype(str)=='phase_end') & (events.get('capture',False)==True)].copy()
    if phases is not None:
        wanted={phases} if isinstance(phases,str) else set(phases); starts=starts[starts.phase.isin(wanted)]; ends=ends[ends.phase.isin(wanted)]
    rows=[]
    for _,s in starts.iterrows():
        tid=str(s.get('trial_id',''))
        if tid not in accepted: continue
        e=ends[(ends.trial_id.astype(str)==tid) & (ends.phase.astype(str)==str(s.get('phase',''))) & (ends.bag_ns>s.bag_ns)]
        if e.empty: continue
        end=e.sort_values('bag_ns').iloc[0]
        row=s.to_dict(); row['start_ns']=int(s.bag_ns); row['end_ns']=int(end.bag_ns); row['duration_s']=(row['end_ns']-row['start_ns'])*1e-9
        rows.append(row)
    return pd.DataFrame(rows)

def _sub(frame:pd.DataFrame,start:int,end:int)->pd.DataFrame:
    if frame.empty or 'bag_ns' not in frame: return frame.iloc[0:0].copy()
    return frame[(frame.bag_ns>=start)&(frame.bag_ns<=end)].copy()

def _median(frame:pd.DataFrame,column:str)->float:
    if frame.empty or column not in frame: return math.nan
    a=frame[column].to_numpy(dtype=float); return float(np.nanmedian(a)) if np.isfinite(a).any() else math.nan

def _std(frame:pd.DataFrame,column:str)->float:
    if frame.empty or column not in frame: return math.nan
    a=frame[column].to_numpy(dtype=float); return float(np.nanstd(a)) if np.isfinite(a).any() else math.nan

def summarize_windows(windows:pd.DataFrame,tables:dict[str,pd.DataFrame],cfg:dict)->pd.DataFrame:
    rows=[]
    for _,w in windows.iterrows():
        start,end=int(w.start_ns),int(w.end_ns); li=_sub(tables['lidar_velocity'],start,end); im=_sub(tables['imu'],start,end); ve=_sub(tables['vesc'],start,end); od=_sub(tables['odom'],start,end)
        selected_speed=_sub(tables['motor_selected_speed'],start,end); selected_current=_sub(tables['motor_selected_current'],start,end); selected_brake=_sub(tables['motor_selected_brake'],start,end)
        if not li.empty and 'valid' in li: li=li[li.valid.astype(bool)]
        row={k:v for k,v in w.to_dict().items() if k not in {'bag_ns','header_ns','type'}}
        row.update({
          'vx_lidar_mps':_median(li,'vx'),'vx_lidar_std_mps':_std(li,'vx'),'lidar_pairs':int(len(li)),'lidar_valid_fraction':float(len(li))/max(1,len(_sub(tables['lidar_velocity'],start,end))),
          'erpm_measured':_median(ve,'erpm'),'erpm_std':_std(ve,'erpm'),'motor_current_a':_median(ve,'motor_current'),'input_current_a':_median(ve,'input_current'),'battery_voltage_v':_median(ve,'battery_voltage'),'motor_temp_c':_median(ve,'temp_motor'),'fet_temp_c':_median(ve,'temp_fet'),
          'odom_vx_mps':_median(od,'vx'),'odom_vx_std_mps':_std(od,'vx'),'imu_ax_mps2':_median(im,'ax'),'imu_ay_mps2':_median(im,'ay'),'imu_gz_rad_s':_median(im,'gz'),
          'selected_speed_erpm':_median(selected_speed,'value'),'selected_speed_erpm_std':_std(selected_speed,'value'),
          'selected_current_a':_median(selected_current,'value'),'selected_current_a_std':_std(selected_current,'value'),
          'selected_brake_a':_median(selected_brake,'value'),'selected_brake_a_std':_std(selected_brake,'value'),
        })
        rows.append(row)
    return pd.DataFrame(rows)

def straight_filter(summary:pd.DataFrame,cfg:dict)->pd.DataFrame:
    if summary.empty: return summary
    a=cfg['analysis']; valid=(summary.lidar_valid_fraction>=float(a['gates']['min_valid_lidar_fraction'])) & np.isfinite(summary.vx_lidar_mps) & (np.abs(summary.imu_gz_rad_s)<=float(a['max_straight_yaw_rate_rad_s'])) & (np.abs(summary.imu_ay_mps2)<=float(a['max_abs_lateral_accel_mps2']))
    return summary[valid].copy()

def coverage(summary:pd.DataFrame, keys:list[str], expected:int)->pd.DataFrame:
    if summary.empty: return pd.DataFrame(columns=keys+['accepted_trials','expected_trials','coverage_ok'])
    g=summary.groupby(keys,dropna=False).size().reset_index(name='accepted_trials'); g['expected_trials']=int(expected); g['coverage_ok']=g.accepted_trials>=int(expected); return g

def robust_linear(x:np.ndarray,y:np.ndarray,*,through_origin:bool=False)->dict:
    x=np.asarray(x,dtype=float); y=np.asarray(y,dtype=float); m=np.isfinite(x)&np.isfinite(y); x,y=x[m],y[m]
    if len(x)<3: raise ValueError('too few finite samples for linear fit')
    if through_origin:
        slope=float(np.dot(x,y)/max(np.dot(x,x),1e-12)); intercept=0.0
    else:
        X=np.column_stack([x,np.ones(len(x))]); slope,intercept=np.linalg.lstsq(X,y,rcond=None)[0]; slope=float(slope); intercept=float(intercept)
    pred=slope*x+intercept; rmse=float(np.sqrt(np.mean((pred-y)**2))); denom=float(np.sum((y-np.mean(y))**2)); r2=float(1-np.sum((y-pred)**2)/denom) if denom>1e-12 else math.nan
    return {'slope':slope,'intercept':intercept,'rmse':rmse,'r2':r2,'n':int(len(x))}

def interpolate(frame:pd.DataFrame,t_ns:np.ndarray,column:str)->np.ndarray:
    if frame.empty or 'bag_ns' not in frame or column not in frame: return np.full(len(t_ns),np.nan)
    f=frame[['bag_ns',column]].dropna().sort_values('bag_ns');
    if f.empty: return np.full(len(t_ns),np.nan)
    return np.interp(t_ns.astype(float),f.bag_ns.to_numpy(dtype=float),f[column].to_numpy(dtype=float),left=np.nan,right=np.nan)

def session_original_values(session:Path)->dict:
    p=session/'vesc_config_transaction'/'profile_history.json'
    if not p.is_file(): return {}
    try: return json.loads(p.read_text(encoding='utf-8')).get('original_values',{})
    except Exception: return {}
