#!/usr/bin/env python3
"""Fit and gate the ground IMU bias before steering identification continues."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from imu_bias import estimate_imu_bias


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    bias = estimate_imu_bias(session)
    report = bias.to_dict()
    report["accepted_for_update"] = bool(
        not report.get("used_fallback_zero", True)
        and int(report.get("stationary_samples", 0)) >= 20
        and bool(report.get("epochs"))
    )
    report["status"] = "pass" if report["accepted_for_update"] else "fail"
    if not report["accepted_for_update"]:
        report["failure"] = "ground stationary IMU epoch is missing or too short"
    output = session / "analysis"
    output.mkdir(parents=True, exist_ok=True)
    (output / "imu_bias.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    if not report["accepted_for_update"]:
        raise SystemExit(report["failure"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
