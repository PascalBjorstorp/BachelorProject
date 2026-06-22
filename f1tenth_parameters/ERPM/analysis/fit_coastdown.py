#!/usr/bin/env python3
"""Identify longitudinal drag from raw-current-zero coast-down captures."""
from __future__ import annotations
import argparse, json, math
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.optimize import least_squares
from scipy.signal import savgol_filter
from common import accepted_capture_windows, analysis_dir, dump_yaml, load_yaml, stage_tables


def _derivative(t: np.ndarray, v: np.ndarray) -> np.ndarray:
    if len(v) < 7:
        return np.gradient(v, t)
    dt = float(np.median(np.diff(t)))
    n = max(5, int(round(0.25 / max(dt, 1e-3))))
    if n % 2 == 0: n += 1
    if n >= len(v): n = len(v) - 1 if len(v) % 2 == 0 else len(v)
    smooth = savgol_filter(v, n, 2, mode='interp') if n >= 5 else v
    return np.gradient(smooth, t)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('session', type=Path); a = p.parse_args()
    session=a.session.resolve(); cfg=load_yaml(session/'calibration_config_snapshot.yaml'); out=analysis_dir(session); tables=stage_tables(session,'07_coastdown')
    windows=accepted_capture_windows(tables['events'],'coastdown'); lidar=tables['lidar_velocity']; vesc=tables['vesc']; imu=tables['imu']
    rows=[]
    for _,w in windows.iterrows():
        f=lidar[(lidar.bag_ns>=w.start_ns)&(lidar.bag_ns<=w.end_ns)].copy()
        if f.empty or 'valid' not in f: continue
        f=f[f.valid.astype(bool)].sort_values('bag_ns');
        if len(f)<10: continue
        t=(f.bag_ns.to_numpy(dtype=float)-float(w.start_ns))*1e-9; v=f.vx.to_numpy(dtype=float); acc=_derivative(t,v)
        cur=vesc[(vesc.bag_ns>=w.start_ns)&(vesc.bag_ns<=w.end_ns)]
        gz=imu[(imu.bag_ns>=w.start_ns)&(imu.bag_ns<=w.end_ns)]
        if len(cur) and abs(float(np.nanmedian(cur.motor_current)))>float(cfg['coastdown']['max_abs_motor_current_a']): continue
        if len(gz) and abs(float(np.nanmedian(gz.gz)))>float(cfg['analysis']['max_straight_yaw_rate_rad_s']): continue
        keep=np.isfinite(v)&np.isfinite(acc)&(v>=float(cfg['analysis']['min_lidar_speed_mps']))
        for tt,vv,aa in zip(t[keep],v[keep],acc[keep]): rows.append({'trial_id':w.trial_id,'t_s':tt,'vx_mps':vv,'ax_mps2':aa})
    data=pd.DataFrame(rows)
    if len(data)<30: raise SystemExit('insufficient accepted coast-down LiDAR samples')
    x=data.vx_mps.to_numpy(dtype=float); y=-data.ax_mps2.to_numpy(dtype=float)
    # Non-negative drag polynomial: d(v)=c0+c1*v+c2*v².
    def resid(p): return p[0]+p[1]*x+p[2]*x*x-y
    result=least_squares(resid,x0=np.array([0.4,0.1,0.0]),bounds=(np.zeros(3),np.array([10.0,10.0,10.0])),loss='soft_l1',f_scale=0.15,max_nfev=1000)
    pred=result.x[0]+result.x[1]*x+result.x[2]*x*x; rmse=float(np.sqrt(np.mean((pred-y)**2))); denom=float(np.sum((y-np.mean(y))**2)); r2=float(1-np.sum((y-pred)**2)/denom) if denom>1e-12 else math.nan
    data['drag_pred_mps2']=pred; data['residual_mps2']=y-pred; data.to_parquet(out/'coastdown_samples.parquet',index=False)
    report={'model':'dv/dt = -(c0 + c1*|v| + c2*v^2) under measured near-zero motor current','accel_drag_coulomb_mps2':float(result.x[0]),'accel_drag_viscous_per_s':float(result.x[1]),'accel_drag_quadratic_per_m':float(result.x[2]),'rmse_mps2':rmse,'r2':r2,'samples':int(len(data)),'accepted_for_candidate':bool(r2>=float(cfg['analysis']['gates']['min_coastdown_r2'])),'gate_min_r2':float(cfg['analysis']['gates']['min_coastdown_r2'])}
    dump_yaml(out/'coastdown_drag_report.yaml',report); print(json.dumps(report,indent=2)); return 0
if __name__=='__main__': raise SystemExit(main())
