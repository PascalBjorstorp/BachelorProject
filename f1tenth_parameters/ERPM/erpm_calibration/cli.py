from __future__ import annotations
import argparse, sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
from .config import load_yaml
from .config_transaction import ConfigTransactionError, VescModeTransaction
from .session import SessionRunner


def _recover(cfg: dict, workspace: Path | None) -> None:
    VescModeTransaction.recover(calibration_root=ROOT, workspace=workspace,
                                config_relpath=cfg['workspace']['vesc_config_relpath'],
                                build_command=list(cfg['workspace']['colcon_build_command']))


def _common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument('--config', type=Path, default=ROOT / 'config' / 'erpm_calibration.yaml')
    parser.add_argument('--workspace', type=Path)
    parser.add_argument('--recover', action='store_true',
                        help='Restore a temporary VESC config left active by a crash/power loss, then rebuild.')


def main_config_calibration() -> int:
    """Tier 1: config-deployable calibration (Stages 0-10 + fit). No C++ edits."""
    p = argparse.ArgumentParser(description='ERPM Tier-1 config-deployable calibration: gathers data and emits '
                                            'analysis/config_only_vesc_patch.yaml (apply to vesc.yaml, no C++ edit).')
    _common_args(p)
    p.add_argument('--runs-dir', type=Path)
    p.add_argument('--resume', type=Path)
    p.add_argument('--preflight-only', action='store_true')
    a = p.parse_args()
    cfg = load_yaml(a.config.resolve())
    try:
        if a.recover:
            _recover(cfg, a.workspace); return 0
        runner = SessionRunner(ROOT, a.config.resolve(), a.runs_dir, a.resume, a.workspace)
        if a.preflight_only:
            runner.run_preflight_only(); return 0
        runner.run_identification(); return 0
    except KeyboardInterrupt:
        return 130
    except (ConfigTransactionError, Exception) as exc:
        print(f'ERPM TIER-1 CALIBRATION FAILED: {exc}', file=sys.stderr); return 2


def main_candidate_port() -> int:
    """Tier 2: candidate shadow verification on a Tier-1 session (needs C++ port)."""
    p = argparse.ArgumentParser(description='ERPM Tier-2 candidate shadow verification: validates the quadratic/LUT '
                                            'command map, fused odometry, traction surface and turn-slip correction. '
                                            'These require the C++ port (PRODUCTION_PORT_CONTRACT.md) to deploy.')
    _common_args(p)
    p.add_argument('--session', type=Path, required=True,
                   help='An existing Tier-1 identification session directory (runs/<session-id>).')
    a = p.parse_args()
    cfg = load_yaml(a.config.resolve())
    try:
        if a.recover:
            _recover(cfg, a.workspace); return 0
        runner = SessionRunner(ROOT, a.config.resolve(), None, a.session, a.workspace)
        runner.run_candidate_port(); return 0
    except KeyboardInterrupt:
        return 130
    except (ConfigTransactionError, Exception) as exc:
        print(f'ERPM TIER-2 CANDIDATE VERIFICATION FAILED: {exc}', file=sys.stderr); return 2
