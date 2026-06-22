"""Operator-driven, bag-first longitudinal calibration stages.

Each physical attempt is in a stage MCAP bag and explicitly ACCEPTed, REDOne,
SKIPped or ABORTed. Runtime uses odometry only for safety and startup gating;
all speed, odometry and drivetrain parameter fits occur offline against LiDAR
scan matching.
"""
from __future__ import annotations
import math, time
from pathlib import Path
from typing import Any, Callable
import numpy as np
from .config import dump_json
from .runtime import CalibrationNode, start_node, finish_node
from .ui import banner, checklist, pause_for_reposition, require_ready, review_trial, warn

WINDOW = ('imu_ax','imu_ay','imu_gz','odom_vx','odom_vy','erpm','motor_current_a','input_current_a','battery_v','motor_temp_c','fet_temp_c','selected_speed_erpm','selected_current_a','selected_brake_a')

class TrialCounter:
    def __init__(self,cfg:dict[str,Any]): self.cfg=cfg; self.accepted=0
    def count(self,node:CalibrationNode)->None:
        self.accepted+=1; every=int(self.cfg['session']['cooling_pause_every_accepted_trials'])
        if every>0 and self.accepted%every==0:
            seconds=float(self.cfg['session']['cooling_pause_s'])
            node.event.emit('cooling_pause_start',accepted_trials=self.accepted,duration_s=seconds)
            print(f'\nCooling pause: {seconds:.0f} s after {self.accepted} accepted motion trials.')
            node.neutral(); node.spin(seconds)
            node.event.emit('cooling_pause_end',accepted_trials=self.accepted)

def _id(prefix:str,rep:int)->str: return f'{prefix}__rep_{rep:02d}'
def _raw_erpm(gain:float,offset:float,speed:float)->float: return float(gain*speed+offset)

def _decision(node:CalibrationNode,*,stage:str,condition_id:str,trial_id:str,attempt:int,auto_ok:bool,summary:dict[str,Any])->str:
    d=review_trial(label=f'{stage}: {condition_id}, attempt {attempt}',automatic_ok=auto_ok,automatic_summary=summary)
    node.event.emit('trial_decision',stage=stage,condition_id=condition_id,trial_id=trial_id,attempt=attempt,decision=d,accepted=d=='accepted',automatic_ok=auto_ok)
    return d

def _straight_ok(node:CalibrationNode,cfg:dict[str,Any])->bool:
    p=cfg['preflight']; return math.isfinite(node.latest.imu_gz) and math.isfinite(node.latest.imu_ay) and abs(node.latest.imu_gz)<=float(p['max_straight_yaw_rate_rad_s']) and abs(node.latest.imu_ay)<=float(p['max_straight_lateral_accel_mps2'])

def _run_raw_erpm_plateau(node:CalibrationNode,*,cfg:dict[str,Any],counter:TrialCounter,stage:str,condition_id:str,raw_erpm:float,nominal_speed:float,capture_s:float,phase:str)->list[dict[str,Any]]:
    records=[]; attempt=1
    while True:
        trial=_id(condition_id,attempt); pause_for_reposition(f'Position car at the straight-line start. {condition_id}; raw target={raw_erpm:.1f} ERPM.\nAttempt {attempt}; REDO has no limit.')
        node.event.emit('trial_start',stage=stage,condition_id=condition_id,trial_id=trial,attempt=attempt,nominal_speed_mps=nominal_speed,raw_erpm_target=raw_erpm)
        startup=node.establish_raw_erpm(target_erpm=raw_erpm,segment_id=condition_id,trial_id=trial)
        summary=None
        if startup.get('stable'):
            summary=node.hold(kind='raw_erpm',target=raw_erpm,duration_s=capture_s,phase=phase,segment_id=condition_id,trial_id=trial,capture=True,window_fields=WINDOW,nominal_speed_mps=nominal_speed,raw_erpm_target=raw_erpm)
        straight=_straight_ok(node,cfg) if summary is not None else False
        node.neutral(); auto=bool(startup.get('stable')) and summary is not None and straight
        decision=_decision(node,stage=stage,condition_id=condition_id,trial_id=trial,attempt=attempt,auto_ok=auto,summary={'startup':startup,'straight_runtime_gate':straight,'capture':summary or {}})
        rec={'trial_id':trial,'attempt':attempt,'decision':decision,'startup':startup,'capture':summary,'raw_erpm_target':raw_erpm,'nominal_speed_mps':nominal_speed}; records.append(rec)
        if decision=='accepted': counter.count(node); return records
        if decision=='skipped': return records
        attempt+=1

def _run_ackermann_plateau(node:CalibrationNode,*,cfg:dict[str,Any],counter:TrialCounter,stage:str,condition_id:str,speed:float,capture_s:float,phase:str)->list[dict[str,Any]]:
    records=[]; attempt=1
    while True:
        trial=_id(condition_id,attempt); pause_for_reposition(f'Position car at the straight-line start. {condition_id}; speed command={speed:.3f} m/s.\nAttempt {attempt}; REDO has no limit.')
        node.event.emit('trial_start',stage=stage,condition_id=condition_id,trial_id=trial,attempt=attempt,speed_command_mps=speed)
        startup=node.establish_ackermann_speed(speed_mps=speed,segment_id=condition_id,trial_id=trial)
        summary=None
        if startup.get('stable'): summary=node.hold(kind='ackermann_speed',target=speed,duration_s=capture_s,phase=phase,segment_id=condition_id,trial_id=trial,capture=True,window_fields=WINDOW,speed_command_mps=speed)
        straight=_straight_ok(node,cfg) if summary is not None else False; node.neutral(); auto=bool(startup.get('stable')) and summary is not None and straight
        decision=_decision(node,stage=stage,condition_id=condition_id,trial_id=trial,attempt=attempt,auto_ok=auto,summary={'startup':startup,'straight_runtime_gate':straight,'capture':summary or {}})
        rec={'trial_id':trial,'attempt':attempt,'decision':decision,'startup':startup,'capture':summary,'speed_command_mps':speed}; records.append(rec)
        if decision=='accepted': counter.count(node); return records
        if decision=='skipped': return records
        attempt+=1

def command_chain_audit(cfg:dict[str,Any],stage_dir:Path)->dict[str,Any]:
    banner('STAGE 0 — MOTOR COMMAND-CHAIN AUDIT','Car must be lifted; wheels may rotate briefly at low ERPM.')
    checklist(['Car is securely on a stand.','No MPC/MPCC/planner/teleoperation/normal bringup is running.','Emergency stop is immediately reachable.'])
    require_ready(); node=start_node('erpm_command_chain_audit',cfg,{'imu','odom','vesc','scan','selected_speed','selected_current','selected_brake'})
    p=cfg['preflight']; delta=float(p['min_command_change_erpm']); results=[]
    try:
        for label,target in [('zero',0.0),('positive',delta),('zero_return',0.0)]:
            summary=node.hold(kind='raw_erpm',target=target,duration_s=0.6,phase='command_chain_audit',segment_id=label,trial_id=f'audit_{label}',capture=True,window_fields=WINDOW,raw_erpm_target=target)
            results.append({'label':label,'command_kind':'raw_erpm','target':target,**summary})
        current_delta=float(p['min_command_change_current_a'])
        for label,kind,target in [('current_zero','raw_current',0.0),('current_positive','raw_current',current_delta),('brake_zero','raw_brake',0.0),('brake_positive','raw_brake',current_delta),('neutral','neutral',0.0)]:
            summary=node.hold(kind=kind,target=target,duration_s=0.35,phase='command_chain_audit',segment_id=label,trial_id=f'audit_{label}',capture=True,window_fields=WINDOW)
            results.append({'label':label,'command_kind':kind,'target':target,**summary})
        pos=next(x for x in results if x['label']=='positive'); erpm_err=abs(float(pos['selected_speed_erpm_mean'])-delta)
        cur=next(x for x in results if x['label']=='current_positive'); current_err=abs(float(cur['selected_current_a_mean'])-current_delta)
        brk=next(x for x in results if x['label']=='brake_positive'); brake_err=abs(float(brk['selected_brake_a_mean'])-current_delta)
        if erpm_err>float(p['raw_motor_tolerance_erpm']): raise RuntimeError(f'motor selector raw ERPM error {erpm_err:.2f} exceeds tolerance')
        if max(current_err,brake_err)>float(p['raw_motor_tolerance_current_a']): raise RuntimeError(f'motor selector raw current/brake error exceeds tolerance: current={current_err:.2f}, brake={brake_err:.2f} A')
        result={'status':'pass','selected_erpm_error':erpm_err,'selected_current_error_a':current_err,'selected_brake_error_a':brake_err,'samples':results}; dump_json(stage_dir/'runtime_result.json',result); print(f'PASS — selector errors: ERPM={erpm_err:.2f}, current={current_err:.2f} A, brake={brake_err:.2f} A'); return result
    finally: finish_node(node)

def longitudinal_observability(cfg:dict[str,Any],stage_dir:Path,gain:float,offset:float,counter:TrialCounter)->dict[str,Any]:
    banner('STAGE 1 — LONGITUDINAL SENSOR OBSERVABILITY','Stationary noise floor plus straight LiDAR/IMU observability checks.')
    require_ready('Place car stationary on the floor. Type READY to capture stationary data, or ABORT'); node=start_node('erpm_observability',cfg,{'imu','odom','vesc','scan','selected_speed','selected_current','selected_brake'})
    try:
        stationary=node.hold(kind='neutral',target=0.0,duration_s=float(cfg['observability']['stationary_duration_s']),phase='stationary_observability',segment_id='stationary',trial_id='stationary',capture=True,window_fields=WINDOW)
        probes=[]
        for speed in cfg['observability']['straight_probe_speeds_mps']:
            for rep in range(1,int(cfg['observability']['straight_probe_repetitions'])+1):
                cid=f'observability_speed_{float(speed):.3f}_rep_{rep:02d}'; probes += _run_raw_erpm_plateau(node,cfg=cfg,counter=counter,stage='01_longitudinal_observability',condition_id=cid,raw_erpm=_raw_erpm(gain,offset,float(speed)),nominal_speed=float(speed),capture_s=float(cfg['observability']['straight_probe_capture_s']),phase='straight_observability')
        result={'stationary':stationary,'probes':probes}; dump_json(stage_dir/'runtime_result.json',result); return result
    finally: finish_node(node)

def low_speed_launch(cfg:dict[str,Any],stage_dir:Path,gain:float,offset:float,counter:TrialCounter)->dict[str,Any]:
    banner('STAGE 2 — LOW-SPEED LAUNCH / DEAD-BAND','Find the smallest repeatable ground-motion command and launch delay.')
    node=start_node('erpm_low_speed_launch',cfg,{'imu','odom','vesc','scan','selected_speed','selected_current','selected_brake'}); rec=[]
    try:
        s=cfg['low_speed_launch']
        for speed in s['nominal_speeds_mps']:
            for rep in range(1,int(s['repetitions'])+1):
                cid=f'launch_speed_{float(speed):.3f}_rep_{rep:02d}'; rec+=_run_raw_erpm_plateau(node,cfg=cfg,counter=counter,stage='02_low_speed_launch',condition_id=cid,raw_erpm=_raw_erpm(gain,offset,float(speed)),nominal_speed=float(speed),capture_s=float(s['capture_s']),phase='low_speed_launch')
        result={'records':rec}; dump_json(stage_dir/'runtime_result.json',result); return result
    finally: finish_node(node)

def raw_erpm_map(cfg:dict[str,Any],stage_dir:Path,gain:float,offset:float,counter:TrialCounter,*,holdout:bool)->dict[str,Any]:
    num='04' if holdout else '03'; title='HOLD-OUT' if holdout else 'TRAINING'
    banner(f'STAGE {num} — RAW ERPM-TO-GROUND-SPEED {title}', 'Raw ERPM is the input; LiDAR velocity is the identification reference.')
    node=start_node('erpm_map_holdout' if holdout else 'erpm_map_training',cfg,{'imu','odom','vesc','scan','selected_speed','selected_current','selected_brake'}); records=[]
    try:
        spec=cfg['raw_erpm_map_holdout' if holdout else 'raw_erpm_map_training']; stage=f'{num}_raw_erpm_map_{"holdout" if holdout else "training"}'; phase='raw_erpm_holdout' if holdout else 'raw_erpm_training'
        # Alternating order reduces battery/temperature drift confounding command level.
        levels=list(map(float,spec['nominal_speeds_mps'])); ordered=[]
        for i in range((len(levels)+1)//2):
            ordered.append(levels[i]); j=len(levels)-1-i
            if j!=i: ordered.append(levels[j])
        for speed in ordered:
            for rep in range(1,int(spec['repetitions'])+1):
                cid=f'{"holdout" if holdout else "training"}_speed_{speed:.3f}_rep_{rep:02d}'; records+=_run_raw_erpm_plateau(node,cfg=cfg,counter=counter,stage=stage,condition_id=cid,raw_erpm=_raw_erpm(gain,offset,speed),nominal_speed=speed,capture_s=float(spec['capture_s']),phase=phase)
        result={'holdout':holdout,'records':records}; dump_json(stage_dir/'runtime_result.json',result); return result
    finally: finish_node(node)

def vel_to_erpm_pipeline_audit(cfg:dict[str,Any],stage_dir:Path,counter:TrialCounter)->dict[str,Any]:
    banner('STAGE 5 — EXISTING VEL_TO_ERPM PIPELINE AUDIT','Audits current desired-speed → ERPM → ground-speed behaviour before candidate deployment.')
    node=start_node('erpm_vel_pipeline',cfg,{'imu','odom','vesc','scan','selected_speed','selected_current','selected_brake'}); rec=[]
    try:
        s=cfg['vel_to_erpm_pipeline_audit']
        for speed in s['speed_commands_mps']:
            for rep in range(1,int(s['repetitions'])+1): rec+=_run_ackermann_plateau(node,cfg=cfg,counter=counter,stage='05_vel_to_erpm_pipeline_audit',condition_id=f'vel_command_{float(speed):.3f}_rep_{rep:02d}',speed=float(speed),capture_s=float(s['capture_s']),phase='vel_to_erpm_pipeline')
        result={'records':rec}; dump_json(stage_dir/'runtime_result.json',result); return result
    finally: finish_node(node)

def raw_erpm_response(cfg:dict[str,Any],stage_dir:Path,gain:float,offset:float,counter:TrialCounter)->dict[str,Any]:
    banner('STAGE 6 — ERPM / GROUND-SPEED STEP RESPONSE','Identifies command delivery, VESC ERPM tracking, and effective vehicle-speed delay/rate.')
    node=start_node('erpm_step_response',cfg,{'imu','odom','vesc','scan','selected_speed','selected_current','selected_brake'}); rec=[]
    try:
        s=cfg['raw_erpm_response']
        for baseline,target in s['steps_mps']:
            base=_raw_erpm(gain,offset,float(baseline)); step=_raw_erpm(gain,offset,float(target))
            for rep in range(1,int(s['repetitions'])+1):
                cid=f'erpm_step_{float(baseline):.3f}_to_{float(target):.3f}_rep_{rep:02d}'; attempt=1
                while True:
                    trial=_id(cid,attempt); pause_for_reposition(f'Position car at straight start for {cid}. Attempt {attempt}.')
                    node.event.emit('trial_start',stage='06_raw_erpm_response',condition_id=cid,trial_id=trial,attempt=attempt,baseline_erpm=base,target_erpm=step,baseline_speed_mps=float(baseline),target_speed_mps=float(target))
                    startup=node.establish_raw_erpm(target_erpm=base,segment_id=cid,trial_id=trial); summary=None
                    if startup.get('stable'):
                        node.hold(kind='raw_erpm',target=base,duration_s=float(s['pre_step_hold_s']),phase='erpm_step_baseline',segment_id=cid,trial_id=trial,capture=False,baseline_erpm=base,target_erpm=step)
                        summary=node.hold(kind='raw_erpm',target=step,duration_s=float(s['response_capture_s']),phase='erpm_step_response',segment_id=cid,trial_id=trial,capture=True,window_fields=WINDOW,baseline_erpm=base,target_erpm=step,baseline_speed_mps=float(baseline),target_speed_mps=float(target))
                    straight=_straight_ok(node,cfg) if summary else False; node.neutral(); auto=bool(startup.get('stable')) and summary is not None and straight
                    d=_decision(node,stage='06_raw_erpm_response',condition_id=cid,trial_id=trial,attempt=attempt,auto_ok=auto,summary={'startup':startup,'straight_runtime_gate':straight,'capture':summary or {}})
                    rec.append({'trial_id':trial,'condition_id':cid,'decision':d,'startup':startup,'capture':summary})
                    if d=='accepted': counter.count(node); break
                    if d=='skipped': break
                    attempt+=1
        result={'records':rec}; dump_json(stage_dir/'runtime_result.json',result); return result
    finally: finish_node(node)

def coastdown(cfg:dict[str,Any],stage_dir:Path,gain:float,offset:float,counter:TrialCounter)->dict[str,Any]:
    banner('STAGE 7 — COAST-DOWN / DRAG','Raw-current zero is commanded after a steady ERPM plateau; this is not a VEL_TO_ERPM stop command.')
    node=start_node('erpm_coastdown',cfg,{'imu','odom','vesc','scan','selected_speed','selected_current','selected_brake'}); rec=[]
    try:
        s=cfg['coastdown']
        for speed in s['initial_speeds_mps']:
            erpm=_raw_erpm(gain,offset,float(speed))
            for rep in range(1,int(s['repetitions'])+1):
                cid=f'coastdown_speed_{float(speed):.3f}_rep_{rep:02d}'; attempt=1
                while True:
                    trial=_id(cid,attempt); pause_for_reposition(f'Position car at straight start for {cid}. Attempt {attempt}.')
                    node.event.emit('trial_start',stage='07_coastdown',condition_id=cid,trial_id=trial,attempt=attempt,initial_speed_mps=float(speed),initial_erpm=erpm)
                    startup=node.establish_raw_erpm(target_erpm=erpm,segment_id=cid,trial_id=trial); summary=None
                    if startup.get('stable'):
                        node.hold(kind='raw_erpm',target=erpm,duration_s=float(s['pre_coast_hold_s']),phase='coastdown_baseline',segment_id=cid,trial_id=trial,capture=False,initial_erpm=erpm)
                        summary=node.hold(kind='raw_current',target=0.0,duration_s=float(s['coast_capture_s']),phase='coastdown',segment_id=cid,trial_id=trial,capture=True,window_fields=WINDOW,initial_erpm=erpm,initial_speed_mps=float(speed))
                    current_ok=summary is not None and abs(float(summary.get('motor_current_a_mean',math.inf)))<=float(s['max_abs_motor_current_a'])
                    straight=_straight_ok(node,cfg) if summary else False; node.neutral(); auto=bool(startup.get('stable')) and current_ok and straight
                    d=_decision(node,stage='07_coastdown',condition_id=cid,trial_id=trial,attempt=attempt,auto_ok=auto,summary={'startup':startup,'current_near_zero':current_ok,'straight_runtime_gate':straight,'capture':summary or {}})
                    rec.append({'trial_id':trial,'condition_id':cid,'decision':d,'startup':startup,'capture':summary})
                    if d=='accepted': counter.count(node); break
                    if d=='skipped': break
                    attempt+=1
        result={'records':rec}; dump_json(stage_dir/'runtime_result.json',result); return result
    finally: finish_node(node)

def _current_pulses(cfg:dict[str,Any],stage_dir:Path,gain:float,offset:float,counter:TrialCounter,*,holdout:bool)->dict[str,Any]:
    num='09' if holdout else '08'; banner(f'STAGE {num} — RAW CURRENT {"HOLD-OUT" if holdout else "TRAINING"}','Direct VESC current/brake pulses identify the drivetrain before fitting ACCEL_TO_CURRENT gains.')
    node=start_node('erpm_current_holdout' if holdout else 'erpm_current_training',cfg,{'imu','odom','vesc','scan','selected_speed','selected_current','selected_brake'}); rec=[]
    try:
        s=cfg['raw_current_holdout' if holdout else 'raw_current_training']
        for polarity,field,start_speed,kind in [('drive','drive_currents_a','initial_speed_mps_drive','raw_current'),('brake','brake_currents_a','initial_speed_mps_brake','raw_brake')]:
            launch_erpm=_raw_erpm(gain,offset,float(s[start_speed]))
            for amp in s[field]:
                for rep in range(1,int(s['repetitions'])+1):
                    cid=f'{polarity}_current_{float(amp):.2f}_rep_{rep:02d}'; attempt=1
                    while True:
                        trial=_id(cid,attempt); pause_for_reposition(f'Position car at straight start for {cid}. Attempt {attempt}.')
                        node.event.emit('trial_start',stage=f'{num}_raw_current_{"holdout" if holdout else "training"}',condition_id=cid,trial_id=trial,attempt=attempt,polarity=polarity,current_command_a=float(amp),initial_speed_mps=float(s[start_speed]),initial_erpm=launch_erpm)
                        startup=node.establish_raw_erpm(target_erpm=launch_erpm,segment_id=cid,trial_id=trial); summary=None
                        if startup.get('stable'):
                            node.hold(kind='raw_erpm',target=launch_erpm,duration_s=float(s['pre_pulse_hold_s']),phase='current_pulse_baseline',segment_id=cid,trial_id=trial,capture=False,polarity=polarity,current_command_a=float(amp))
                            summary=node.hold(kind=kind,target=float(amp),duration_s=float(s['pulse_capture_s']),phase=f'raw_{polarity}_current_pulse',segment_id=cid,trial_id=trial,capture=True,window_fields=WINDOW,polarity=polarity,current_command_a=float(amp),initial_speed_mps=float(s[start_speed]))
                        straight=_straight_ok(node,cfg) if summary else False; node.neutral(); auto=bool(startup.get('stable')) and summary is not None and straight
                        d=_decision(node,stage=f'{num}_raw_current_{"holdout" if holdout else "training"}',condition_id=cid,trial_id=trial,attempt=attempt,auto_ok=auto,summary={'startup':startup,'straight_runtime_gate':straight,'capture':summary or {}})
                        rec.append({'trial_id':trial,'condition_id':cid,'decision':d,'startup':startup,'capture':summary,'polarity':polarity})
                        if d=='accepted': counter.count(node); break
                        if d=='skipped': break
                        attempt+=1
        result={'holdout':holdout,'records':rec}; dump_json(stage_dir/'runtime_result.json',result); return result
    finally: finish_node(node)

def accel_to_current_interface(cfg:dict[str,Any],stage_dir:Path,gain:float,offset:float,counter:TrialCounter)->dict[str,Any]:
    banner('STAGE 10 — ACCEL_TO_CURRENT INTERFACE AUDIT','Temporary bootstrap gains are active. This audits acceleration command → selected current → vehicle response; it does not install candidates.')
    node=start_node('erpm_accel_interface',cfg,{'imu','odom','vesc','scan','selected_speed','selected_current','selected_brake'}); rec=[]
    try:
        s=cfg['accel_to_current_interface']; start_erpm=_raw_erpm(gain,offset,float(s['initial_speed_mps']))
        for accel in s['acceleration_commands_mps2']:
            for rep in range(1,int(s['repetitions'])+1):
                cid=f'accel_command_{float(accel):+.2f}_rep_{rep:02d}'; attempt=1
                while True:
                    trial=_id(cid,attempt); pause_for_reposition(f'Position car at straight start for {cid}. Attempt {attempt}.')
                    node.event.emit('trial_start',stage='10_accel_to_current_interface',condition_id=cid,trial_id=trial,attempt=attempt,acceleration_command_mps2=float(accel),initial_erpm=start_erpm)
                    startup=node.establish_raw_erpm(target_erpm=start_erpm,segment_id=cid,trial_id=trial); summary=None
                    if startup.get('stable'):
                        node.hold(kind='raw_erpm',target=start_erpm,duration_s=float(s['pre_pulse_hold_s']),phase='accel_interface_baseline',segment_id=cid,trial_id=trial,capture=False,acceleration_command_mps2=float(accel))
                        summary=node.hold(kind='ackermann_accel',target=float(accel),speed_hint=float(s['initial_speed_mps']),duration_s=float(s['pulse_capture_s']),phase='accel_to_current_pulse',segment_id=cid,trial_id=trial,capture=True,window_fields=WINDOW,acceleration_command_mps2=float(accel))
                    straight=_straight_ok(node,cfg) if summary else False; node.neutral(); auto=bool(startup.get('stable')) and summary is not None and straight
                    d=_decision(node,stage='10_accel_to_current_interface',condition_id=cid,trial_id=trial,attempt=attempt,auto_ok=auto,summary={'startup':startup,'straight_runtime_gate':straight,'capture':summary or {}})
                    rec.append({'trial_id':trial,'condition_id':cid,'decision':d,'startup':startup,'capture':summary})
                    if d=='accepted': counter.count(node); break
                    if d=='skipped': break
                    attempt+=1
        result={'records':rec}; dump_json(stage_dir/'runtime_result.json',result); return result
    finally: finish_node(node)

def run_stage(stage_name:str,cfg:dict[str,Any],stage_dir:Path,gain:float,offset:float,counter:TrialCounter)->dict[str,Any]:
    table={
      '00_command_chain_audit':lambda:command_chain_audit(cfg,stage_dir),
      '01_longitudinal_observability':lambda:longitudinal_observability(cfg,stage_dir,gain,offset,counter),
      '02_low_speed_launch':lambda:low_speed_launch(cfg,stage_dir,gain,offset,counter),
      '03_raw_erpm_map_training':lambda:raw_erpm_map(cfg,stage_dir,gain,offset,counter,holdout=False),
      '04_raw_erpm_map_holdout':lambda:raw_erpm_map(cfg,stage_dir,gain,offset,counter,holdout=True),
      '05_vel_to_erpm_pipeline_audit':lambda:vel_to_erpm_pipeline_audit(cfg,stage_dir,counter),
      '06_raw_erpm_response':lambda:raw_erpm_response(cfg,stage_dir,gain,offset,counter),
      '07_coastdown':lambda:coastdown(cfg,stage_dir,gain,offset,counter),
      '08_raw_current_training':lambda:_current_pulses(cfg,stage_dir,gain,offset,counter,holdout=False),
      '09_raw_current_holdout':lambda:_current_pulses(cfg,stage_dir,gain,offset,counter,holdout=True),
      '10_accel_to_current_interface':lambda:accel_to_current_interface(cfg,stage_dir,gain,offset,counter),
    }
    return table[stage_name]()

# Candidate verification stages are deliberately separate from collection.
# They are run only after strict offline analysis accepts a temporary candidate.
def candidate_velocity_verification(cfg: dict[str, Any], stage_dir: Path, counter: TrialCounter) -> dict[str, Any]:
    banner('STAGE 11 — TEMPORARY CANDIDATE VEL_TO_ERPM VERIFICATION', 'The candidate is active only in the reversible transaction. It will be restored after this session.')
    node=start_node('erpm_candidate_velocity_verification',cfg,{'imu','odom','vesc','scan','selected_speed','selected_current','selected_brake'}); rec=[]
    try:
        s=cfg['candidate_verification']
        for speed in s['velocity_holdout_commands_mps']:
            for rep in range(1,int(s['repetitions'])+1):
                rec+=_run_ackermann_plateau(node,cfg=cfg,counter=counter,stage='11_candidate_velocity_verification',condition_id=f'candidate_vel_{float(speed):.3f}_rep_{rep:02d}',speed=float(speed),capture_s=float(s['capture_s']),phase='candidate_velocity_verification')
        result={'records':rec}; dump_json(stage_dir/'runtime_result.json',result); return result
    finally: finish_node(node)

def candidate_accel_verification(cfg: dict[str, Any], stage_dir: Path, gain: float, offset: float, counter: TrialCounter) -> dict[str, Any]:
    banner('STAGE 12 — TEMPORARY CANDIDATE ACCEL_TO_CURRENT VERIFICATION', 'The candidate current/drag model is active only in the reversible transaction.')
    node=start_node('erpm_candidate_accel_verification',cfg,{'imu','odom','vesc','scan','selected_speed','selected_current','selected_brake'}); rec=[]
    try:
        s=cfg['candidate_verification']; start_erpm=_raw_erpm(gain,offset,0.45)
        for accel in s['acceleration_holdout_commands_mps2']:
            for rep in range(1,int(s['repetitions'])+1):
                cid=f'candidate_accel_{float(accel):+.2f}_rep_{rep:02d}'; attempt=1
                while True:
                    trial=_id(cid,attempt); pause_for_reposition(f'Position car at straight start for {cid}. Attempt {attempt}.')
                    node.event.emit('trial_start',stage='12_candidate_accel_verification',condition_id=cid,trial_id=trial,attempt=attempt,acceleration_command_mps2=float(accel),initial_erpm=start_erpm)
                    startup=node.establish_raw_erpm(target_erpm=start_erpm,segment_id=cid,trial_id=trial); summary=None
                    if startup.get('stable'):
                        node.hold(kind='raw_erpm',target=start_erpm,duration_s=1.5,phase='candidate_accel_baseline',segment_id=cid,trial_id=trial,capture=False,acceleration_command_mps2=float(accel))
                        summary=node.hold(kind='ackermann_accel',target=float(accel),speed_hint=0.45,duration_s=float(s['capture_s']),phase='candidate_accel_verification',segment_id=cid,trial_id=trial,capture=True,window_fields=WINDOW,acceleration_command_mps2=float(accel))
                    straight=_straight_ok(node,cfg) if summary else False; node.neutral(); auto=bool(startup.get('stable')) and summary is not None and straight
                    d=_decision(node,stage='12_candidate_accel_verification',condition_id=cid,trial_id=trial,attempt=attempt,auto_ok=auto,summary={'startup':startup,'straight_runtime_gate':straight,'capture':summary or {}})
                    rec.append({'trial_id':trial,'condition_id':cid,'decision':d,'startup':startup,'capture':summary})
                    if d=='accepted': counter.count(node); break
                    if d=='skipped': break
                    attempt+=1
        result={'records':rec}; dump_json(stage_dir/'runtime_result.json',result); return result
    finally: finish_node(node)
