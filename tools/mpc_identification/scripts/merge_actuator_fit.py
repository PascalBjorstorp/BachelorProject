#!/usr/bin/env python3
"""Apply only defensible actuator-fit fields to a plant parameter JSON.

The servo feedback topic in the hardware node can be command-domain feedback rather
than physical wheel-angle measurement.  By default this script therefore copies
transport delay, gain and first-order time constant only; it deliberately keeps
`steer_rate_max` at the known simulator/physical default.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", type=Path, required=True)
    parser.add_argument("--actuator-fit", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--use-rate-limit", action="store_true",
                        help="Also use fitted steer_rate_max. Off by default because feedback may be a command echo.")
    args = parser.parse_args()

    base = json.loads(args.base.read_text())
    fit = json.loads(args.actuator_fit.read_text())
    applied: dict[str, float] = {}
    for key, lo, hi in [
        ("steer_delay_s", 0.0, 0.15),
        ("steer_gain", 0.70, 1.30),
        ("steer_tau_s", 0.0, 0.20),
    ]:
        value = float(fit[key])
        if not lo <= value <= hi:
            raise ValueError(f"{key}={value} is outside expected bounds [{lo}, {hi}]")
        base[key] = value
        applied[key] = value

    if args.use_rate_limit:
        value = float(fit["steer_rate_max"])
        if not 0.5 <= value <= 8.0:
            raise ValueError(f"steer_rate_max={value} is outside expected bounds")
        base["steer_rate_max"] = value
        applied["steer_rate_max"] = value

    metadata = base.get("metadata", {})
    metadata["actuator_fit_merge"] = {
        "source": str(args.actuator_fit),
        "applied": applied,
        "not_applied": ["steer_bias_rad", "steer_rate_max"] if not args.use_rate_limit else ["steer_bias_rad"],
        "warning": (
            "The bag's servo feedback topic may be command-domain feedback. "
            "Rate limit and bias are intentionally not copied by default."
        ),
    }
    base["metadata"] = metadata
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(base, indent=2) + "\n")
    print(json.dumps(metadata["actuator_fit_merge"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
