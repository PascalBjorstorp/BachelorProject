#!/usr/bin/env python3
"""Synthetic checks for the cornering longitudinal-slip fit.

Generates steady-arc windows with a known slip coefficient and verifies the fit
recovers it, that the held-out corrected speed beats the uncorrected wheel speed,
and that the correction leaves straight-line motion (a_lat=0) untouched.
"""
from __future__ import annotations
import math, sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "analysis"))
from fit_turn_slip import apply_turn_slip, fit_turn_slip_coefficient, _condition_split  # noqa: E402


def _make(c_true=0.03, gain_mps_per_erpm=1/4550, seed=0):
    rng = np.random.default_rng(seed)
    speeds = [1.2, 2.2]; angles = [0.08, 0.16, 0.24]; L = 0.324
    a_lat, slip, v_wheel, v_ground, cells = [], [], [], [], []
    for v in speeds:
        for d in angles:
            kappa = math.tan(d) / L
            al = v * v * kappa                     # lateral accel of the arc
            for _ in range(3):                      # repetitions
                vg = v + rng.normal(0, 0.02)        # true ground speed
                s = c_true * al                     # true slip fraction
                vw = vg / (1.0 - s)                 # wheel over-reads
                # small measurement noise on the window-averaged ground speed
                vg_meas = vg + rng.normal(0, 0.03)
                a_lat.append(vw * (v * kappa))       # regressor uses v_wheel*|yaw|, yaw=v*kappa
                v_wheel.append(vw); v_ground.append(vg_meas)
                slip.append(1.0 - vg_meas / vw)
                cells.append(f"{d}|{v}")
    return (np.array(a_lat), np.array(slip), np.array(v_wheel), np.array(v_ground), cells, c_true)


def check_recovers_coefficient():
    a, s, vw, vg, cells, c_true = _make()
    c1, r2 = fit_turn_slip_coefficient(a, s)
    # a_lat regressor uses v*kappa for yaw (=a_lat/v_wheel), so recovered c1 ~ c_true
    assert c1 > 0, c1
    assert r2 > 0.8, r2
    assert abs(c1 - c_true) < 0.5 * c_true, (c1, c_true)


def check_holdout_improves_and_straight_preserved():
    a, s, vw, vg, cells, _ = _make()
    tr, ho = _condition_split(cells)
    cells = np.array(cells)
    m_tr = np.isin(cells, list(tr)); m_ho = np.isin(cells, list(ho))
    c1, _ = fit_turn_slip_coefficient(a[m_tr], s[m_tr])
    corrected = apply_turn_slip(vw[m_ho], a[m_ho], c1, 0.25)
    base_rmse = float(np.sqrt(np.mean((vw[m_ho] - vg[m_ho]) ** 2)))
    corr_rmse = float(np.sqrt(np.mean((corrected - vg[m_ho]) ** 2)))
    assert corr_rmse < base_rmse, (corr_rmse, base_rmse)
    # straight line: a_lat = 0 -> correction is identity
    straight = apply_turn_slip(np.array([3.0]), np.array([0.0]), c1, 0.25)
    assert abs(straight[0] - 3.0) < 1e-12, straight


def check_split_disjoint():
    cells = [f"c{i}" for i in range(10)]
    tr, ho = _condition_split(cells)
    assert tr.isdisjoint(ho) and (tr | ho) == set(cells) and len(ho) > 0


def main():
    check_recovers_coefficient()
    check_holdout_improves_and_straight_preserved()
    check_split_disjoint()
    print("turn-slip fit checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
