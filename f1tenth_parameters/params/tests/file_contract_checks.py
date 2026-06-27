#!/usr/bin/env python3
from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from params_identification.config import load_yaml
from params_identification.design import (
    G,
    combined_slip_conditions,
    design_summary,
    steady_lateral_conditions,
    steering_transient_conditions,
    validate_design,
)


def assert_close(actual: float, expected: float, tol: float = 1e-9) -> None:
    assert abs(actual - expected) <= tol, f"{actual} != {expected}"


cfg = load_yaml(ROOT / "config" / "params_identification.yaml")
topics = load_yaml(ROOT / "config" / "topics.yaml")

errors = validate_design(cfg)
assert not errors, errors

summary = design_summary(cfg)
assert summary["steady_lateral_core_captures"] == 32
assert summary["steering_transient_captures"] == 36
assert summary["combined_slip_captures"] == 48
assert summary["limit_lap_total"] == 24

steady = cfg["steady_lateral"]
assert steady["radii_m"] == [2.5, 3.0]
assert steady["directions"] == ["left", "right"]
assert steady["ay_ladder_g_initial"] == [0.15, 0.30, 0.45, 0.60]
assert_close(float(steady["steady_capture_s"]), 1.2)
assert int(steady["repetitions"]) == 2
assert bool(steady["operator_escalates_near_limit"])
for condition in steady_lateral_conditions(cfg):
    expected_speed = math.sqrt(condition["target_ay_g"] * G * condition["radius_m"])
    assert_close(condition["target_speed_mps"], expected_speed)
    assert condition["target_speed_mps"] <= 4.3

transients = cfg["steering_transients"]
assert transients["speed_fractions_of_vmax"] == [0.50, 0.75, 1.00]
assert transients["lateral_response_targets_g"] == [0.18, 0.32]
assert_close(float(transients["capture_s"]), 0.55)
assert int(transients["repetitions"]) == 3
max_post_stable = max(c["post_stable_required_m"] for c in steering_transient_conditions(cfg))
assert max_post_stable <= float(cfg["site"]["post_stable_corridor_available_m"])
assert 8.6 <= max_post_stable <= 8.7

combined = cfg["combined_slip"]
assert combined["lateral_utilisation"] == [0.35, 0.60, 0.80]
assert combined["longitudinal_utilisation"] == [-0.60, -0.30, 0.30, 0.60]
assert len(combined_slip_conditions(cfg)) == 48
assert bool(combined["pure_lateral_parameters_fixed_during_fit"])

assert topics["recording"]["compression_mode"] == "file"
assert topics["recording"]["compression_format"] == "zstd"
for group in ["stationary_physical", "lateral_motion", "combined_slip", "limit_laps"]:
    required = topics["required"][group]
    assert "/sensors/imu/raw" in required
    assert "/scan" in required
    assert "/tf_static" in required
    assert "/params_identification/event" in required

print("params identification file-contract checks passed")
