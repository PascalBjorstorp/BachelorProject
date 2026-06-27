#!/usr/bin/env python3
"""Checks for the proportional straight-line steering assist (odometry-heading hold)."""
from __future__ import annotations
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from erpm_calibration.straight_assist import StraightAssist, from_config  # noqa: E402


def check_anchors_then_corrects_drift():
    """First moving sample anchors 'straight'; a later heading drift produces a
    proportional counter-steer in the correcting direction."""
    sa = StraightAssist(max_trim_rate_rad_s=1.0, max_trim_step_rad=1.0)
    assert sa.step(heading=1.000, speed=2.0, dt=0.01) == 0.0      # anchor here
    # Drift +0.1 rad (left): expect a negative trim (steer back toward ref).
    trim = sa.step(heading=1.100, speed=2.0, dt=0.01)
    assert trim < 0.0, trim
    # Bigger drift -> bigger (more negative) correction, until the bound.
    trim2 = sa.step(heading=1.200, speed=2.0, dt=0.01)
    assert trim2 < trim, (trim, trim2)
    # Opposite drift flips the sign.
    sa.reset()
    sa.step(heading=1.000, speed=2.0, dt=0.01)
    assert sa.step(heading=0.900, speed=2.0, dt=0.01) > 0.0


def check_deadband_ignores_tiny_drift():
    sa = StraightAssist(deadband_rad=0.0087, max_trim_rate_rad_s=1.0, max_trim_step_rad=1.0)
    sa.step(heading=0.0, speed=2.0, dt=0.01)                      # anchor
    assert sa.step(heading=0.005, speed=2.0, dt=0.01) == 0.0      # below deadband
    assert sa.step(heading=0.05, speed=2.0, dt=0.01) != 0.0       # above deadband


def check_clamped():
    sa = StraightAssist(max_trim_rad=0.07, max_trim_rate_rad_s=10.0, max_trim_step_rad=10.0)
    sa.step(heading=0.0, speed=2.0, dt=0.01)                      # anchor
    assert abs(sa.step(heading=3.0, speed=2.0, dt=0.01)) <= 0.07 + 1e-9


def check_slew_limited():
    sa = StraightAssist(max_trim_rate_rad_s=0.08, max_trim_step_rad=0.0015)
    sa.step(heading=0.0, speed=2.0, dt=0.01)
    first = sa.step(heading=1.0, speed=2.0, dt=0.01)
    second = sa.step(heading=1.0, speed=2.0, dt=0.01)
    assert abs(first) <= 0.0008 + 1e-12, first
    assert abs(second - first) <= 0.0008 + 1e-12, (first, second)
    assert second < first < 0.0, (first, second)


def check_sign_flip():
    a = StraightAssist(steer_sign=1.0, max_trim_rate_rad_s=1.0, max_trim_step_rad=1.0)
    b = StraightAssist(steer_sign=-1.0, max_trim_rate_rad_s=1.0, max_trim_step_rad=1.0)
    a.step(heading=0.0, speed=2.0, dt=0.01); b.step(heading=0.0, speed=2.0, dt=0.01)
    ta = a.step(heading=0.1, speed=2.0, dt=0.01)
    tb = b.step(heading=0.1, speed=2.0, dt=0.01)
    assert ta == -tb and ta != 0.0, (ta, tb)


def check_reanchors_at_rest():
    sa = StraightAssist(max_trim_rate_rad_s=1.0, max_trim_step_rad=1.0)
    sa.step(heading=0.0, speed=2.0, dt=0.01)                      # anchor at 0.0
    assert sa.step(heading=0.5, speed=0.0, dt=0.01) == 0.0        # stopped -> reset
    assert sa.yaw_ref is None
    assert sa.step(heading=0.5, speed=2.0, dt=0.01) == 0.0        # re-anchor at 0.5
    assert sa.step(heading=0.6, speed=2.0, dt=0.01) < 0.0


def check_disabled_passthrough():
    sa = from_config({"straight_assist": {"enabled": False}})
    sa2 = StraightAssist()
    sa2.step(heading=0.0, speed=2.0, dt=0.01)
    assert sa.step(heading=0.5, speed=2.0, dt=0.01) == 0.0


def check_from_config_defaults():
    sa = from_config({})
    assert sa.enabled and sa.max_trim_rad > 0 and sa.kp_heading > 0 and sa.max_trim_rate_rad_s > 0 and sa.steer_sign == 1.0


def check_legacy_config_aliases():
    sa = from_config({"straight_assist": {"ki_heading_rad_per_rad": 0.3, "max_rate_rad_s": 0.2}})
    assert abs(sa.kp_heading - 0.3) < 1e-12
    assert abs(sa.max_trim_rate_rad_s - 0.2) < 1e-12


def main():
    check_anchors_then_corrects_drift()
    check_deadband_ignores_tiny_drift()
    check_clamped()
    check_slew_limited()
    check_sign_flip()
    check_reanchors_at_rest()
    check_disabled_passthrough()
    check_from_config_defaults()
    check_legacy_config_aliases()
    print("straight-assist checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
