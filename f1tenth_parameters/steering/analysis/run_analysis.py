#!/usr/bin/env python3
"""Run the complete offline steering-calibration analysis pipeline."""
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FULL_STAGES = [
    "00_command_chain_audit", "01_zero_curvature_centre", "03_sensor_observability",
    "04_static_map_training", "05_static_map_holdout", "06_command_to_curvature_response",
]
ALL_STAGES = ["02_physical_endstops", *FULL_STAGES]


def run(command: list[str]) -> None:
    print("\n$ " + " ".join(command))
    subprocess.run(command, check=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    for stage in ALL_STAGES:
        bag = session / stage / "bag"
        if (bag / "metadata.yaml").exists():
            run([sys.executable, str(ROOT / "analysis" / "export_bag.py"), str(bag)])
    for stage in FULL_STAGES:
        bag = session / stage / "bag"
        if (bag / "metadata.yaml").exists() and (session / stage / "derived" / "scan_index.parquet").exists():
            run([sys.executable, str(ROOT / "analysis" / "estimate_lidar_motion.py"), str(bag),
                 "--config", str(session / "calibration_config_snapshot.yaml")])
    # Do not fit or summarize a session with incomplete required streams.
    run([sys.executable, str(ROOT / "analysis" / "check_session.py"), str(session), "--strict"])
    run([sys.executable, str(ROOT / "analysis" / "assess_icp_quality.py"), str(session)])
    run([sys.executable, str(ROOT / "analysis" / "fit_centre.py"), str(session)])
    run([sys.executable, str(ROOT / "analysis" / "fit_static_map.py"), str(session),
         "--config", str(session / "calibration_config_snapshot.yaml")])
    run([sys.executable, str(ROOT / "analysis" / "fit_response.py"), str(session)])
    run([sys.executable, str(ROOT / "analysis" / "assemble_steering_summary.py"), str(session)])
    print("\nOffline analysis complete. Review session/analysis before applying any parameters.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
