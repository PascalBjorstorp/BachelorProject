from __future__ import annotations
import argparse,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
from .config import load_yaml
from .config_transaction import ConfigTransactionError,VescModeTransaction
from .session import SessionRunner

def main()->int:
    p=argparse.ArgumentParser(description='Run full bag-first ERPM/longitudinal calibration.')
    p.add_argument('--config',type=Path,default=ROOT/'config'/'erpm_calibration.yaml'); p.add_argument('--runs-dir',type=Path); p.add_argument('--resume',type=Path); p.add_argument('--workspace',type=Path); p.add_argument('--recover',action='store_true'); p.add_argument('--preflight-only',action='store_true')
    a=p.parse_args(); cfg=load_yaml(a.config.resolve())
    try:
        if a.recover and a.preflight_only:
            raise ConfigTransactionError('--recover and --preflight-only are mutually exclusive')
        if a.recover:
            VescModeTransaction.recover(calibration_root=ROOT,workspace=a.workspace,config_relpath=cfg['workspace']['vesc_config_relpath'],build_command=list(cfg['workspace']['colcon_build_command'])); return 0
        runner = SessionRunner(ROOT,a.config.resolve(),a.runs_dir,a.resume,a.workspace)
        if a.preflight_only:
            runner.run_preflight_only(); return 0
        runner.run(); return 0
    except KeyboardInterrupt: return 130
    except (ConfigTransactionError,Exception) as exc:
        print(f'ERPM CALIBRATION FAILED: {exc}',file=sys.stderr); return 2
