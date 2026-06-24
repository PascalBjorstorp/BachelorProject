#!/usr/bin/env python3
"""Tier 1 — config-deployable ERPM/longitudinal calibration.

Runs Stages 0-10 (observability, launch, ERPM map, response, coast-down,
current) and the offline fit, then emits analysis/config_only_vesc_patch.yaml.
Every parameter it produces is already supported by the production C++
(ackermann_to_vesc / vesc_to_odom), so you apply it to vesc.yaml with NO code
edit: ERPM/odom gain, odom scale, accel/brake current gains, the coast-down
drag feed-forward (makes a=0 hold speed instead of coasting) and current clamps.

The original VESC configuration is backed up byte-for-byte and restored
automatically. For the quadratic/LUT command map, fused odometry, traction
surface and turn-slip correction (which need the C++ port), run
erpm_candidate_port.py on the same session afterwards.
"""
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from erpm_calibration.cli import main_config_calibration
if __name__ == '__main__':
    raise SystemExit(main_config_calibration())
