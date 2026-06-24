#!/usr/bin/env python3
"""Tier 2 — candidate shadow verification (requires the C++ port to deploy).

Runs on an existing Tier-1 session (``--session runs/<session-id>``). It
relaunches the dedicated graph with the candidate maps as shadow nodes and runs
Stages 11-13 to validate the features the current production parameters cannot
express: the quadratic/LUT command map, the fused IMU/wheel odometry estimator,
the full-envelope traction surface, the a=0 hold-speed gate, and the cornering
turn-slip correction.

Nothing is installed permanently. A passing candidate is "eligible for review";
permanent deployment requires the C++ port in full_stack/PRODUCTION_PORT_CONTRACT.md
and a re-run of these stages with the C++ node. The original VESC configuration
is backed up byte-for-byte and restored automatically.
"""
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from erpm_calibration.cli import main_candidate_port
if __name__ == '__main__':
    raise SystemExit(main_candidate_port())
