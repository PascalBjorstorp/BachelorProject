#!/usr/bin/env python3
"""Run strict offline analysis for a complete ERPM longitudinal session."""
from __future__ import annotations
import argparse, subprocess, sys
from pathlib import Path
from common import STAGES
ROOT=Path(__file__).resolve().parents[1]
def call(args:list[str])->None:
    print('>', ' '.join(map(str,args))); r=subprocess.run(args,check=False)
    if r.returncode: raise SystemExit(r.returncode)
def main()->int:
    p=argparse.ArgumentParser(); p.add_argument('session',type=Path); a=p.parse_args(); session=a.session.resolve()
    call([sys.executable,str(ROOT/'analysis'/'check_session.py'),str(session)])
    for stage in STAGES:
        bag=session/stage/'bag'
        call([sys.executable,str(ROOT/'analysis'/'export_bag.py'),str(bag)])
        # All motion analysis has access to raw scans. Stage 0 does not need ICP.
        if stage!='00_command_chain_audit': call([sys.executable,str(ROOT/'analysis'/'estimate_lidar_motion.py'),str(bag),'--config',str(session/'calibration_config_snapshot.yaml')])
    for name in ['fit_speed_map.py','fit_coastdown.py','fit_current_model.py','fit_traction_transients.py','fit_erpm_response.py','fit_accel_interface.py','fit_odom_model_selection.py','assemble_candidate.py','summarize_forward_motion.py']:
        call([sys.executable,str(ROOT/'analysis'/name),str(session)])
    return 0
if __name__=='__main__': raise SystemExit(main())
