#!/usr/bin/env python3
"""Checks for the stationary IMU-bias model evaluation and clamping."""
from __future__ import annotations

import sys
import tempfile
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "analysis"))

from imu_bias import (  # noqa: E402
    GROUND_EARLY_STAGE,
    STATIONARY_PHASE,
    STATIONARY_STAGE,
    ImuBias,
    estimate_imu_bias,
)


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


def _parquet_available() -> bool:
    try:
        import pyarrow  # noqa: F401
        return True
    except Exception:
        try:
            import fastparquet  # noqa: F401
            return True
        except Exception:
            return False


def _write_imu(path: Path, *, t0_ns: float, n: int, gz: float, std: float) -> None:
    rng = np.random.default_rng(0)
    bag_ns = t0_ns + np.arange(n) * 1e7  # 100 Hz
    frame = pd.DataFrame({
        "bag_ns": bag_ns,
        "gz": gz + rng.normal(0.0, std, size=n),
        "ay": rng.normal(0.0, std, size=n),
        "ax": rng.normal(0.0, std, size=n),
    })
    path.parent.mkdir(parents=True, exist_ok=True)
    frame.to_parquet(path)


def _write_stage3_events(path: Path, *, start_ns: float, end_ns: float) -> None:
    trial = "observability_stationary__attempt_01"
    rows = [
        {"event": "phase_start", "phase": STATIONARY_PHASE, "trial_id": trial, "bag_ns": start_ns},
        {"event": "phase_end", "phase": STATIONARY_PHASE, "trial_id": trial, "bag_ns": end_ns},
        {"event": "trial_decision", "phase": STATIONARY_PHASE, "trial_id": trial,
         "bag_ns": end_ns + 1, "decision": "accepted"},
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    pd.DataFrame(rows).to_parquet(path)


def check_estimate_uses_on_ground_epochs() -> None:
    """Both on-ground epochs feed the bias, and a clear drift yields linear_drift."""
    with tempfile.TemporaryDirectory() as tmp:
        session = Path(tmp)
        early_t0 = 1_000_000_000.0
        # Early on-ground epoch (Stage 1b): whole imu.parquet is one stationary hold.
        _write_imu(session / GROUND_EARLY_STAGE / "derived" / "imu.parquet",
                   t0_ns=early_t0, n=300, gz=0.010, std=2e-4)
        # Late on-ground epoch (Stage 3): trimmed inside an accepted stationary window.
        late_start = early_t0 + 100e9
        late_end = late_start + 4e9
        _write_imu(session / STATIONARY_STAGE / "derived" / "imu.parquet",
                   t0_ns=late_start, n=400, gz=0.013, std=2e-4)
        _write_stage3_events(session / STATIONARY_STAGE / "derived" / "events.parquet",
                             start_ns=late_start, end_ns=late_end)

        bias = estimate_imu_bias(session)
        assert bias.used_fallback_zero is False
        assert bias.model == "linear_drift", bias.model
        assert len(bias.epochs) == 2
        assert {e["stage"] for e in bias.epochs} == {GROUND_EARLY_STAGE, STATIONARY_STAGE}
        # Bias tracks each epoch's moment, clamped at the brackets.
        assert abs(bias.gz_at(early_t0) - 0.010) < 1e-3
        assert abs(bias.gz_at(late_start) - 0.013) < 1e-3


def check_estimate_single_on_ground_epoch_is_constant() -> None:
    """One on-ground epoch (Stage 1b only) gives a non-fallback constant bias."""
    with tempfile.TemporaryDirectory() as tmp:
        session = Path(tmp)
        _write_imu(session / GROUND_EARLY_STAGE / "derived" / "imu.parquet",
                   t0_ns=1_000_000_000.0, n=300, gz=0.008, std=1e-4)
        bias = estimate_imu_bias(session)
        assert bias.used_fallback_zero is False
        assert bias.model == "constant", bias.model
        assert abs(bias.gz_intercept - 0.008) < 1e-3


def main() -> int:
    check_constant_model()
    check_linear_drift_and_clamp()
    check_zero_model()
    if _parquet_available():
        check_estimate_uses_on_ground_epochs()
        check_estimate_single_on_ground_epoch_is_constant()
    else:
        print("SKIP: estimate_imu_bias session checks (no parquet engine installed)")
    print("imu bias model checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
