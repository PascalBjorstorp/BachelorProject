#!/usr/bin/env python3
"""Identify drive/brake current-to-ground-acceleration gains with hold-out gates."""
from __future__ import annotations
import argparse, json, math
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.signal import savgol_filter
from common import accepted_capture_windows, analysis_dir, dump_yaml, load_yaml, robust_linear, stage_tables


def _drag(v: np.ndarray, report: dict) -> np.ndarray:
    return float(report['accel_drag_coulomb_mps2']) + float(report['accel_drag_viscous_per_s']) * np.abs(v) + float(report['accel_drag_quadratic_per_m']) * v * v


def _mean_accel(lidar: pd.DataFrame, start: int, end: int) -> tuple[float, float, int]:
    f = lidar[(lidar.bag_ns >= start) & (lidar.bag_ns <= end)].copy()
    if f.empty or 'valid' not in f: return math.nan, math.nan, 0
    f = f[f.valid.astype(bool)].sort_values('bag_ns')
    if len(f) < 8: return math.nan, math.nan, len(f)
    # Remove entry/exit edges of the pulse. The central 60% is more robust to
    # selector/VESC transitions and produces an average ground acceleration.
    lo = int(len(f) * 0.20); hi = max(lo + 3, int(len(f) * 0.80)); f = f.iloc[lo:hi]
    t = f.bag_ns.to_numpy(dtype=float) * 1e-9; v = f.vx.to_numpy(dtype=float)
    if len(v) < 5: return math.nan, math.nan, len(v)
    dt = float(np.median(np.diff(t))); w = max(5, int(round(0.20 / max(dt, 1e-3))))
    if w % 2 == 0: w += 1
    if w >= len(v): w = len(v) - 1 if len(v) % 2 == 0 else len(v)
    smooth = savgol_filter(v, w, 2, mode='interp') if w >= 5 else v
    a = np.gradient(smooth, t)
    return float(np.nanmedian(a)), float(np.nanmedian(v)), len(v)


def _pulse_summary(session: Path, stage: str, phases: list[str], cfg: dict, drag: dict) -> pd.DataFrame:
    tables = stage_tables(session, stage); windows = accepted_capture_windows(tables['events'], phases); rows=[]
    for _,w in windows.iterrows():
        accel, speed, n = _mean_accel(tables['lidar_velocity'], int(w.start_ns), int(w.end_ns))
        vesc=tables['vesc'][(tables['vesc'].bag_ns>=w.start_ns)&(tables['vesc'].bag_ns<=w.end_ns)]
        imu=tables['imu'][(tables['imu'].bag_ns>=w.start_ns)&(tables['imu'].bag_ns<=w.end_ns)]
        if not math.isfinite(accel) or not math.isfinite(speed) or n<5: continue
        yaw=float(np.nanmedian(imu.gz)) if len(imu) else math.nan
        if math.isfinite(yaw) and abs(yaw)>float(cfg['analysis']['max_straight_yaw_rate_rad_s']): continue
        actual_motor=float(np.nanmedian(vesc.motor_current)) if len(vesc) else math.nan
        actual_input=float(np.nanmedian(vesc.input_current)) if len(vesc) else math.nan
        command=float(w.get('current_command_a', math.nan)); polarity=str(w.get('polarity',''))
        row={'trial_id':w.trial_id,'condition_id':w.condition_id,'polarity':polarity,'current_command_a':command,'motor_current_a':actual_motor,'input_current_a':actual_input,'ax_lidar_mps2':accel,'vx_lidar_mps':speed,'drag_mps2':float(_drag(np.asarray([speed]),drag)[0]),'sample_count':n}
        rows.append(row)
    return pd.DataFrame(rows)


def _fit(df: pd.DataFrame, *, polarity: str) -> dict:
    part=df[df.polarity==polarity].copy()
    if len(part)<4: raise ValueError(f'insufficient {polarity} current samples')
    if polarity=='drive':
        x=part.motor_current_a.to_numpy(dtype=float); y=(part.ax_lidar_mps2+part.drag_mps2).to_numpy(dtype=float)
        x=np.maximum(x,0.0)
    else:
        # Brake command is a clearer independent variable than negative motor
        # current telemetry because firmware may report sign/convention changes.
        x=part.current_command_a.to_numpy(dtype=float); y=(-part.ax_lidar_mps2-part.drag_mps2).to_numpy(dtype=float)
    fit=robust_linear(x,y,through_origin=True); fit['polarity']=polarity; fit['accel_per_amp']=fit['slope']; fit['gain_a_per_mps2']=1.0/fit['slope'] if fit['slope']>1e-9 else math.inf
    part['pred_net_accel_mps2']=fit['slope']*x; part['net_accel_mps2']=y; part['residual_mps2']=y-part.pred_net_accel_mps2
    return fit,part


def main()->int:
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('session',type=Path); a=p.parse_args(); session=a.session.resolve(); cfg=load_yaml(session/'calibration_config_snapshot.yaml'); out=analysis_dir(session)
    import yaml
    drag=yaml.safe_load((out/'coastdown_drag_report.yaml').read_text(encoding='utf-8'))
    train=_pulse_summary(session,'08_raw_current_training',['raw_drive_current_pulse','raw_brake_current_pulse'],cfg,drag)
    hold=_pulse_summary(session,'09_raw_current_holdout',['raw_drive_current_pulse','raw_brake_current_pulse'],cfg,drag)
    drive_fit,drive_rows=_fit(train,polarity='drive'); brake_fit,brake_rows=_fit(train,polarity='brake')
    train_rows=pd.concat([drive_rows,brake_rows],ignore_index=True); train_rows.to_parquet(out/'current_model_training_trials.parquet',index=False); hold.to_parquet(out/'current_model_holdout_trials.parquet',index=False)
    hold_metrics={}
    for polarity,fit in [('drive',drive_fit),('brake',brake_fit)]:
        q=hold[hold.polarity==polarity].copy()
        if q.empty: hold_metrics[polarity]={'rmse_mps2':math.inf,'n':0}; continue
        x=np.maximum(q.motor_current_a.to_numpy(dtype=float),0.0) if polarity=='drive' else q.current_command_a.to_numpy(dtype=float)
        y=(q.ax_lidar_mps2+q.drag_mps2).to_numpy(dtype=float) if polarity=='drive' else (-q.ax_lidar_mps2-q.drag_mps2).to_numpy(dtype=float)
        residual=y-float(fit['slope'])*x; hold_metrics[polarity]={'rmse_mps2':float(np.sqrt(np.mean(residual**2))),'bias_mps2':float(np.mean(residual)),'n':int(len(q))}
    gate=cfg['analysis']['gates']; hold_ok=all(v['rmse_mps2']<=float(gate['max_current_model_holdout_rmse_mps2']) for v in hold_metrics.values())
    # accel_deadzone is a command interface hysteresis guard, not a physical
    # friction parameter. Derive it from the noise floor of accepted current data.
    accel_noise=float(np.nanstd(train.ax_lidar_mps2.to_numpy(dtype=float)))
    candidate_deadzone=max(0.02,min(0.20,0.25*accel_noise))
    report={'drive_fit':drive_fit,'brake_fit':brake_fit,'holdout':hold_metrics,'candidate_accel_to_current_gain':float(drive_fit['gain_a_per_mps2']),'candidate_accel_to_brake_gain':float(brake_fit['gain_a_per_mps2']),'candidate_accel_deadzone_mps2':candidate_deadzone,'accepted_for_candidate':bool(drive_fit['r2']>=float(gate['min_current_model_r2']) and brake_fit['r2']>=float(gate['min_current_model_r2']) and hold_ok),'gates':dict(gate)}
    dump_yaml(out/'current_acceleration_report.yaml',report); print(json.dumps(report,indent=2,default=str)); return 0
if __name__=='__main__': raise SystemExit(main())
