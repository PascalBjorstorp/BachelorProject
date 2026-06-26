#!/usr/bin/env python3
"""Run the complete operator-guided, bag-first steering calibration session."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from steering_calibration.config_transaction import ConfigTransactionError, VescConfigTransaction
from steering_calibration.session import SessionRunner


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config" / "steering_calibration.yaml")
    parser.add_argument("--runs-dir", type=Path, default=None)
    parser.add_argument("--resume", type=Path, default=None,
                        help="Resume an interrupted session; completed stages are skipped.")
    parser.add_argument("--accept-partial-stage", action="append", default=[],
                        help=(
                            "With --resume, mark an existing failed/interrupted stage bag as completed "
                            "after verifying required topics, then continue with later stages. "
                            "May be repeated."
                        ))
    parser.add_argument("--workspace", type=Path, default=None,
                        help="BachelorProject colcon workspace root. Auto-detected by default.")
    parser.add_argument("--recover", action="store_true",
                        help="Restore a temporary VESC configuration left active by a crash or power loss, then rebuild.")
    args = parser.parse_args()
    try:
        if args.recover:
            VescConfigTransaction.recover(steering_root=ROOT, workspace=args.workspace)
            return 0
        SessionRunner(ROOT, args.config.resolve(), args.runs_dir, args.resume, args.workspace,
                      accept_partial_stages=args.accept_partial_stage).run()
        return 0
    except KeyboardInterrupt:
        return 130
    except ConfigTransactionError as exc:
        print(f"CONFIGURATION SAFETY ERROR: {exc}", file=sys.stderr)
        return 2
    except Exception as exc:
        print(f"STEERING CALIBRATION FAILED: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
