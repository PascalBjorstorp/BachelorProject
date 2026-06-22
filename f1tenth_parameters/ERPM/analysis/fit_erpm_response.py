#!/usr/bin/env python3
"""Fit command-to-ERPM and command-to-ground-speed dynamic response metrics."""
from __future__ import annotations
import argparse, json, math
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.optimize import least_squares
from scipy.signal import savgol_filter
from common import accepted_capture_windows, analysis_dir, dump_yaml, interpolate, load_yaml, stage_tables


def crossing(t,y,level,hold=0.05):
    for i in range(len(t)):
        if y[i] < level: continue
        j=np.searchsorted(t,t[i]+hold)
        if j>i and np.all(y[i:j]>=level): return float(t[i])
    return None

def metrics(t,y,unit):
    finite=np.isfinite(t)&np.isfinite(y); t,y=t[finite],y[finite]
    if len(t)<12: return {'valid':False,'reason':'too_few_samples'}
    base=np.median(y[(t>=-1.0)&(t<=-0.05)]) if np.any((t>=-1.0)&(t<=-0.05)) else y[0]
    final=np.median(y[t>=max(0,t.max()-0.75)])
    amp=final-base
    if abs(amp)<1e-9: return {'valid':False,'reason':'zero_amplitude','initial':float(base),'final':float(final)}
    post=t>=0; tp=t[post]; n=(y[post]-base)/amp
    t10,t50,t90=crossing(tp,n,.10),crossing(tp,n,.50),crossing(tp,n,.90)
    dt=float(np.median(np.diff(tp))) if len(tp)>1 else math.nan; peak=None
    if math.isfinite(dt) and len(tp)>=7:
        w=max(5,int(round(.075/dt))); w+=1-w%2
        if w<len(tp): peak=float(np.nanmax(np.abs(np.gradient(savgol_filter(y[post],w,2,mode='interp'),tp))))
    # FOPDT normalized response.
    try:
        d0=max(0,(t10 or 0)-0.1*((t90-t10)/2.197 if t10 is not None and t90 is not None else .1)); tau0=max(.02,(t90-t10)/2.197 if t10 is not None and t90 is not None else .2)
        def model(p):
            d,tau,k=p; elapsed=np.maximum(tp-d,0); return k*(1-np.exp(-elapsed/tau))
        r=least_squares(lambda p:model(p)-n,x0=[d0,tau0,1.0],bounds=([0,.005,.2],[min(1.5,tp.max()*.75),max(.2,tp.max()*2),2.]),loss='soft_l1')
        f={'fopdt_delay_s':float(r.x[0]),'fopdt_tau_s':float(r.x[1]),'fopdt_gain':float(r.x[2]),'fopdt_rmse_normalized':float(np.sqrt(np.mean((model(r.x)-n)**2))),'fopdt_fit_ok':bool(r.success)}
    except Exception: f={'fopdt_delay_s':None,'fopdt_tau_s':None,'fopdt_gain':None,'fopdt_rmse_normalized':None,'fopdt_fit_ok':False}
    return {'valid':True,'initial':float(base),'final':float(final),'amplitude':float(amp),'delay_10pct_s':t10,'delay_50pct_s':t50,'rise_10_90_s':None if t10 is None or t90 is None else float(t90-t10),'peak_rate_per_s':peak,**f}

def main()->int:
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('session',type=Path); a=p.parse_args(); session=a.session.resolve(); cfg=load_yaml(session/'calibration_config_snapshot.yaml'); out=analysis_dir(session); tables=stage_tables(session,'06_raw_erpm_response')
    windows=accepted_capture_windows(tables['events'],'erpm_step_response'); rows=[]
    for _,w in windows.iterrows():
        start,end=int(w.start_ns),int(w.end_ns); t0=start
        ve=tables['vesc'][(tables['vesc'].bag_ns>=start-1_000_000_000)&(tables['vesc'].bag_ns<=end)].copy(); li=tables['lidar_velocity'][(tables['lidar_velocity'].bag_ns>=start-1_000_000_000)&(tables['lidar_velocity'].bag_ns<=end)].copy(); sel=tables['motor_selected_speed'][(tables['motor_selected_speed'].bag_ns>=start-1_000_000_000)&(tables['motor_selected_speed'].bag_ns<=end)].copy()
        if not li.empty and 'valid' in li: li=li[li.valid.astype(bool)]
        te=(ve.bag_ns.to_numpy(dtype=float)-t0)*1e-9; tl=(li.bag_ns.to_numpy(dtype=float)-t0)*1e-9; ts=(sel.bag_ns.to_numpy(dtype=float)-t0)*1e-9
        erpm_m=metrics(te,ve.erpm.to_numpy(dtype=float),'ERPM') if len(ve) else {'valid':False,'reason':'no_vesc'}; speed_m=metrics(tl,li.vx.to_numpy(dtype=float),'m/s') if len(li) else {'valid':False,'reason':'no_lidar'}; selector_m=metrics(ts,sel.value.to_numpy(dtype=float),'ERPM') if len(sel) else {'valid':False,'reason':'no_selector'}
        rows.append({'trial_id':w.trial_id,'condition_id':w.condition_id,'baseline_erpm':w.get('baseline_erpm'),'target_erpm':w.get('target_erpm'),'erpm_metrics_json':json.dumps(erpm_m),'ground_speed_metrics_json':json.dumps(speed_m),'selector_metrics_json':json.dumps(selector_m),'erpm_delay_10pct_s':erpm_m.get('delay_10pct_s'),'ground_speed_delay_10pct_s':speed_m.get('delay_10pct_s'),'ground_speed_tau_s':speed_m.get('fopdt_tau_s')})
    table=pd.DataFrame(rows); table.to_parquet(out/'erpm_response_trials.parquet',index=False)
    valid=table.dropna(subset=['erpm_delay_10pct_s','ground_speed_delay_10pct_s']) if not table.empty else table
    report={'trials':int(len(table)),'valid_timing_trials':int(len(valid)),'median_command_to_erpm_delay_s':float(valid.erpm_delay_10pct_s.median()) if len(valid) else math.inf,'median_command_to_ground_speed_delay_s':float(valid.ground_speed_delay_10pct_s.median()) if len(valid) else math.inf,'accepted_for_candidate':bool(len(valid)>0 and valid.erpm_delay_10pct_s.median()<=float(cfg['analysis']['gates']['max_command_to_erpm_delay_s']) and valid.ground_speed_delay_10pct_s.median()<=float(cfg['analysis']['gates']['max_command_to_ground_speed_delay_s']))}
    dump_yaml(out/'erpm_response_report.yaml',report); print(json.dumps(report,indent=2)); return 0
if __name__=='__main__': raise SystemExit(main())
