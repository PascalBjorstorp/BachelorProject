#!/usr/bin/env python3
"""Checks for the closed-loop straight-line steering assist."""
from __future__ import annotations
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from erpm_calibration.straight_assist import StraightAssist, from_config  # noqa: E402


def check_reduces_drift_vs_open_loop():
    """With the default (gentle) gains, a steering centre offset that would curve
    the car open-loop is largely cancelled: heading drift is far smaller and the
    trim builds a steady counter-steer (the slack/offset compensation)."""
    offset, plant, dt, n = 0.05, 6.0, 0.01, 500   # v/L~6, 5 s run
    heading_open = 0.0
    for _ in range(n):
        heading_open += (plant * offset) * dt      # trim=0, drift grows unbounded
    sa = StraightAssist()                            # default on-car gains
    heading_closed, trim, last_yaw = 0.0, 0.0, 0.0
    for _ in range(n):
        last_yaw = plant * (offset + trim)           # effective steer = offset + trim
        heading_closed += last_yaw * dt
        trim = sa.step(yaw_rate=last_yaw, lateral_velocity=0.0, speed=2.0, dt=dt)
    assert abs(heading_closed) < 0.3 * abs(heading_open), (heading_closed, heading_open)
    assert trim < -0.005, f"trim did not build a counter-steer: {trim}"
    assert abs(last_yaw) < abs(plant * offset), "yaw not reduced from open-loop drift"


def check_clamped_and_rate_limited():
    sa = StraightAssist(max_trim_rad=0.07, max_rate_rad_s=0.6)
    # Huge yaw should saturate at max_trim, but only after rate-limited steps.
    first = sa.step(yaw_rate=5.0, lateral_velocity=0.0, speed=2.0, dt=0.01)
    assert abs(first) <= 0.6 * 0.01 + 1e-9, first  # rate limited on first step
    for _ in range(1000):
        t = sa.step(yaw_rate=5.0, lateral_velocity=0.0, speed=2.0, dt=0.01)
    assert abs(t) <= 0.07 + 1e-9 and t <= -0.07 + 1e-9, t  # saturated at the bound


def check_zero_at_rest_no_windup():
    sa = StraightAssist()
    for _ in range(100):
        t = sa.step(yaw_rate=0.3, lateral_velocity=0.1, speed=0.0, dt=0.01)  # not moving
        assert t == 0.0
    assert sa.heading == 0.0 and sa.last_trim == 0.0


def check_disabled_passthrough():
    sa = from_config({"straight_assist": {"enabled": False}})
    assert sa.step(yaw_rate=0.5, lateral_velocity=0.2, speed=2.0, dt=0.01) == 0.0


def check_from_config_defaults():
    sa = from_config({})
    assert sa.enabled and sa.max_trim_rad > 0 and sa.kp_heading > 0


def main():
    check_reduces_drift_vs_open_loop()
    check_clamped_and_rate_limited()
    check_zero_at_rest_no_windup()
    check_disabled_passthrough()
    check_from_config_defaults()
    print("straight-assist checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
