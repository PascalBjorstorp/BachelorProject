#!/usr/bin/env python3
"""Validate a fitted effective-steering response on distinct raw-step data."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import yaml


def _number(mapping: dict, key: str) -> float:
    try:
        value = float(mapping.get(key, math.nan))
    except (TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    cfg = yaml.safe_load((session / "calibration_config_snapshot.yaml").read_text(encoding="utf-8")) or {}
    analysis_cfg = cfg.get("analysis", {}).get("response", {})
    analysis = session / "analysis"
    training = json.loads((analysis / "command_to_effective_steering_response_summary.json").read_text(encoding="utf-8"))
    validation = json.loads((analysis / "validation_command_to_effective_steering_response_summary.json").read_text(encoding="utf-8"))
    failures: list[str] = []
    required = int(analysis_cfg.get("min_validation_segments", 1))
    valid = int(validation.get("segments_with_valid_effective_response", 0))
    if valid < required:
        failures.append(f"valid effective-response segments {valid} < required {required}")
    for key, cap_key in (
        ("median_effective_settling_5pct_s", "max_validation_settling_s"),
        ("median_fopdt_rmse_normalized", "max_validation_fopdt_rmse_normalized"),
    ):
        cap = analysis_cfg.get(cap_key)
        value = _number(validation, key)
        if cap is not None and (not math.isfinite(value) or value > float(cap)):
            failures.append(f"{key}={value!r} exceeds {float(cap):.3f}")
    max_ratio = float(analysis_cfg.get("max_validation_training_time_constant_ratio", 3.0))
    training_tau = _number(training, "median_fopdt_tau_s")
    validation_tau = _number(validation, "median_fopdt_tau_s")
    tau_ratio = max(training_tau, validation_tau) / max(min(training_tau, validation_tau), 1e-9) if math.isfinite(training_tau) and math.isfinite(validation_tau) else math.inf
    if tau_ratio > max_ratio:
        failures.append(f"training/validation FOPDT tau ratio {tau_ratio:.3f} exceeds {max_ratio:.3f}")
    report = {
        "status": "pass" if not failures else "fail",
        "accepted_for_validation": not failures,
        "training_summary": training,
        "validation_summary": validation,
        "fopdt_tau_ratio": tau_ratio if math.isfinite(tau_ratio) else None,
        "failures": failures,
        "scope": "effective vehicle steering response; this validates a control-oriented response model, not a physical servo-shaft model.",
    }
    (analysis / "steering_response_validation_report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    if failures:
        raise SystemExit("steering response hold-out rejected: " + "; ".join(failures))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
