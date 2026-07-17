#!/usr/bin/env python3
"""Fit the scalar current candidate from training pulses only."""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
import yaml

from common import analysis_dir, dump_yaml, load_yaml
from fit_current_model import _coverage, _linear_fit, _pulse_summary, _surface_fit


def _bootstrap_current_gain(frame, polarity: str, cfg: dict, *,
                            resamples: int = 1000, seed: int = 20260717) -> dict:
    """Trial-resampled interval for one deployable acceleration-current gain."""
    part = frame[frame.polarity.astype(str) == polarity].copy()
    gains: list[float] = []
    if len(part) >= 4 and resamples > 0:
        rng = np.random.default_rng(seed)
        for _ in range(resamples):
            sample = part.iloc[rng.integers(0, len(part), len(part))].copy()
            try:
                gain = float(_linear_fit(sample, polarity, cfg)["gain_a_per_mps2"])
            except (KeyError, ValueError):
                continue
            if math.isfinite(gain) and gain > 0.0:
                gains.append(gain)
    return {
        "method": "nonparametric bootstrap over independent accepted trials",
        "requested_resamples": int(resamples),
        "valid_resamples": int(len(gains)),
        "gain_a_per_mps2_95pct": (
            [float(value) for value in np.quantile(gains, [0.025, 0.975])] if gains else []
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    cfg = load_yaml(session / "calibration_config_snapshot.yaml")
    out = analysis_dir(session)
    drag = yaml.safe_load((out / "coastdown_drag_report.yaml").read_text(encoding="utf-8"))
    static = yaml.safe_load((out / "erpm_speed_map_report.yaml").read_text(encoding="utf-8"))
    train = _pulse_summary(session, "08_raw_current_training", ["raw_drive_current_pulse", "raw_brake_current_pulse"], cfg, drag, static)
    train.to_parquet(out / "current_model_training_trials.parquet", index=False)
    coverage: dict[str, bool] = {}
    failures: list[str] = []
    for polarity in ("drive", "brake"):
        cov = _coverage(train, cfg, "raw_current_training", polarity)
        cov.to_parquet(out / f"current_{polarity}_training_coverage.parquet", index=False)
        coverage[polarity] = bool(len(cov)) and bool(cov.coverage_ok.all())
        if not coverage[polarity]:
            failures.append(f"{polarity} current training coverage incomplete")
    linear: dict[str, dict] = {}
    surface: dict[str, dict] = {}
    try:
        for polarity in ("drive", "brake"):
            linear[polarity] = _linear_fit(train, polarity, cfg)
            surface[polarity] = _surface_fit(train, polarity)
    except (ValueError, KeyError) as exc:
        failures.append(str(exc))
    gains = {
        "drive": float(linear.get("drive", {}).get("gain_a_per_mps2", math.nan)),
        "brake": float(linear.get("brake", {}).get("gain_a_per_mps2", math.nan)),
    }
    statistics_cfg = cfg.get("analysis", {}).get("statistics", {})
    gain_bootstrap = {
        polarity: _bootstrap_current_gain(
            train,
            polarity,
            cfg,
            resamples=int(statistics_cfg.get("bootstrap_resamples", 1000)),
            seed=int(statistics_cfg.get("bootstrap_seed", 20260717)) + index,
        )
        for index, polarity in enumerate(("drive", "brake"))
    }
    if not all(math.isfinite(value) and value > 0.0 for value in gains.values()):
        failures.append("training current fit did not produce positive finite gains")
    noise = float(np.nanstd(train.ax_lidar_mps2.to_numpy(float))) if not train.empty else math.nan
    deadzone = max(0.02, min(0.20, 0.25 * noise)) if math.isfinite(noise) else 0.05
    report = {
        "training_only": True,
        "low_slip_scalar_fit": linear,
        "full_envelope_surface_fit": surface,
        "coverage": coverage,
        "candidate_accel_to_current_gain": gains["drive"],
        "candidate_accel_to_brake_gain": gains["brake"],
        "candidate_gain_bootstrap": gain_bootstrap,
        "candidate_accel_deadzone_mps2": deadzone,
        "accepted_for_update": not failures,
        "failures": failures,
        "validation_note": "The next hold-out stage evaluates these applied gains without refitting them.",
    }
    dump_yaml(out / "current_acceleration_training_report.yaml", report)
    print(report)
    if failures:
        raise SystemExit("current training candidate rejected: " + "; ".join(failures))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
