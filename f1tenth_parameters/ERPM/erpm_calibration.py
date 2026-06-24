#!/usr/bin/env python3
"""Deprecated single entry point — the campaign is now split into two scripts.

The ERPM campaign is partitioned at the C++-edit boundary:

  * erpm_config_calibration.py  (Tier 1) — gathers data and emits parameters
    that the production C++ already supports; apply to vesc.yaml with NO code
    edit (ERPM/odom gain, odom scale, accel/brake gains, drag feed-forward that
    holds speed at a=0, current clamps).

  * erpm_candidate_port.py      (Tier 2) — validates, in shadow, the features
    that require the C++ port (quadratic/LUT command map, fused odometry,
    traction surface, turn-slip correction). Run it on a Tier-1 session.
"""
import sys

_MESSAGE = (
    "erpm_calibration.py is deprecated. Use:\n"
    "  python3 erpm_config_calibration.py --workspace ~/BachelorProject     # Tier 1 (config-only, no C++)\n"
    "  python3 erpm_candidate_port.py --session runs/<id> --workspace ~/BachelorProject  # Tier 2 (needs C++ port)\n"
)

if __name__ == '__main__':
    print(_MESSAGE, file=sys.stderr)
    raise SystemExit(2)
