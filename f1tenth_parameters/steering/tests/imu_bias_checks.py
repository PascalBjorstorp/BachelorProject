#!/usr/bin/env python3
"""Checks for the stationary IMU-bias model evaluation and clamping."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "analysis"))

from imu_bias import ImuBias  # noqa: E402


def check_constant_model() -> None:
    bias = ImuBias(gz_intercept=0.01, gz_slope_per_ns=0.0, model="constant")
    assert bias.gz_at(123.0) == 0.01
    assert np.allclose(bias.gz_at(np.array([0.0, 1e9, 5e9])), 0.01)


def check_linear_drift_and_clamp() -> None:
    # 0.002 rad/s over 100 s, anchored at t=0.
    span_ns = 100e9
    slope = 0.002 / span_ns
    bias = ImuBias(
        gz_intercept=0.01, gz_slope_per_ns=slope, t_ref_ns=0.0,
        t_min_ns=0.0, t_max_ns=span_ns, model="linear_drift", epochs=[{"t_ns": 0.0}, {"t_ns": span_ns}],
    )
    assert abs(bias.gz_at(0.0) - 0.01) < 1e-12
    assert abs(bias.gz_at(span_ns) - 0.012) < 1e-12
    assert abs(bias.gz_at(50e9) - 0.011) < 1e-12
    # Outside the bracket the bias is held constant, never extrapolated.
    assert abs(bias.gz_at(-10e9) - 0.01) < 1e-12
    assert abs(bias.gz_at(200e9) - 0.012) < 1e-12
    # Vectorised evaluation matches scalar.
    arr = bias.gz_at(np.array([-10e9, 0.0, 50e9, span_ns, 200e9]))
    assert np.allclose(arr, [0.01, 0.01, 0.011, 0.012, 0.012])
    summary = bias.to_dict()
    assert abs(summary["gz_drift_over_session_rad_s"] - 0.002) < 1e-12


def check_zero_model() -> None:
    bias = ImuBias()
    assert bias.used_fallback_zero is True
    assert bias.gz_at(9.0) == 0.0


def main() -> int:
    check_constant_model()
    check_linear_drift_and_clamp()
    check_zero_model()
    print("imu bias model checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
