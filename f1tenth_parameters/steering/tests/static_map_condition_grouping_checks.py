#!/usr/bin/env python3
"""Regression tests for nominal-condition static-map grouping.

Run from package root:
    python3 tests/static_map_condition_grouping_checks.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "analysis"))
import fit_static_map as fsm  # noqa: E402


def _samples() -> pd.DataFrame:
    rows = []
    # Same commanded conditions; deliberately perturb the mean echo per repeated
    # trial enough that exact-float grouping would split every trial apart.
    for side, base_echo, base_delta in (("low_raw", 0.45, -0.10), ("high_raw", 0.65, 0.10)):
        for approach, hysteresis in (("outward", 0.004), ("inward", -0.004)):
            for rep in range(4):
                rows.append({
                    "side": side,
                    "fraction": 0.20,
                    "approach": approach,
                    "raw_servo_target": base_echo,
                    "raw_servo_echo": base_echo + (rep - 1.5) * 0.00017,
                    "delta_eq_rad": base_delta + hysteresis + (rep - 1.5) * 0.0005,
                })
    return fsm.add_nominal_condition_keys(pd.DataFrame(rows))


def check_nominal_grouping() -> None:
    data = _samples()
    repeatability = fsm._repeatability_summary(data)
    assert len(repeatability) == 4, repeatability
    assert set(repeatability.repeat_count.tolist()) == {4}
    assert repeatability.repeatability_std_rad.notna().all()

    hysteresis = fsm._hysteresis_summary(repeatability)
    assert len(hysteresis) == 2, hysteresis
    assert hysteresis.hysteresis_delta_rad.notna().all()
    assert np.allclose(np.abs(hysteresis.hysteresis_delta_rad), 0.008, atol=1e-9)

    conditions = fsm._condition_summary(data)
    assert len(conditions) == 2, conditions
    x, y = fsm.map_interpolate(conditions, centre_servo=0.55)
    assert len(x) == 3 and len(y) == 3
    assert np.all(np.diff(x) > 0.0)


def check_coverage_gate() -> None:
    data = _samples()
    _, failures = fsm._coverage_report(
        data,
        fractions=[0.20],
        approaches=fsm.TRAINING_APPROACHES,
        required_count=4,
        label="training",
    )
    assert not failures, failures

    incomplete = data[~((data.side == "low_raw") & (data.approach == "inward"))].copy()
    _, failures = fsm._coverage_report(
        incomplete,
        fractions=[0.20],
        approaches=fsm.TRAINING_APPROACHES,
        required_count=4,
        label="training",
    )
    assert len(failures) == 1 and "low_raw" in failures[0] and "inward" in failures[0]


def check_source_and_config_contract() -> None:
    source = (ROOT / "analysis" / "fit_static_map.py").read_text(encoding="utf-8")
    assert 'groupby(["raw_servo_echo"' not in source
    assert 'index=["raw_servo_echo"' not in source
    assert "condition_approach_key" in source
    assert "static_map_condition_coverage.parquet" in source

    cfg = yaml.safe_load((ROOT / "config" / "steering_calibration.yaml").read_text(encoding="utf-8"))
    assert "max_center_servo_spread" not in cfg["centre_trim"]
    assert "measurements_per_side" not in cfg["endstops"]
    assert int(cfg["response"]["repetitions"]) == 5

    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    assert "5 repetitions per speed / side / step size" in readme


def main() -> int:
    check_nominal_grouping()
    check_coverage_gate()
    check_source_and_config_contract()
    print("static-map nominal-condition grouping checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
