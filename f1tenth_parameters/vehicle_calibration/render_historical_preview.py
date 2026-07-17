#!/usr/bin/env python3
"""Render the unified GUI plots from an existing standalone steering session.

This is deliberately non-destructive: only small derived tables and runtime
summaries are copied, while MCAP bags stay in their original session.
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from calibration_suite.runner import STAGE_BY_KEY  # noqa: E402
from calibration_suite.stage_report import _plot_stage  # noqa: E402


PLOT_STAGES = (
    "steering_command_audit",
    "steering_observability",
    "steering_centre",
    "steering_endstops",
    "steering_static_training",
    "steering_static_holdout",
    "steering_response",
)

ANALYSIS_FILES = (
    "centre_trim_points.parquet",
    "centre_trim_offline.json",
    "static_map_training_segments.parquet",
    "static_map_holdout_segments.parquet",
    "command_to_effective_steering_response_metrics.parquet",
    "command_to_effective_steering_response_summary.json",
)


def _copy_if_present(source: Path, destination: Path, *, overwrite: bool = False) -> bool:
    if not source.is_file() or (destination.exists() and not overwrite):
        return destination.is_file()
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    return True


def prepare(source: Path, output: Path) -> None:
    for name in ANALYSIS_FILES:
        _copy_if_present(source / "analysis" / name, output / "analysis" / name)
    for directory in (
        "00_command_chain_audit", "01_zero_curvature_centre", "02_physical_endstops",
        "03_sensor_observability", "04_static_map_training", "05_static_map_holdout",
        "06_command_to_curvature_response",
    ):
        _copy_if_present(
            source / directory / "runtime_result.json",
            output / directory / "runtime_result.json",
        )
        for name in (
            "events.parquet", "imu.parquet", "odom.parquet", "servo_echo.parquet",
            "servo_selected.parquet", "scan_index.parquet",
        ):
            _copy_if_present(
                source / directory / "derived" / name,
                output / directory / "derived" / name,
            )
    # Use the old result only as a fallback. A new non-mutating ICP replay in
    # this directory supersedes these two files when available.
    for directory in (
        "01_zero_curvature_centre", "03_sensor_observability", "04_static_map_training",
        "05_static_map_holdout", "06_command_to_curvature_response",
    ):
        _copy_if_present(
            source / directory / "derived" / "lidar_velocity.parquet",
            output / directory / "derived" / "lidar_velocity.parquet",
        )
    legacy_map = source / "analysis" / "lidar_static_map.json"
    candidate_map = output / "analysis" / "candidate_static_steering_map.json"
    if legacy_map.is_file():
        payload = json.loads(legacy_map.read_text(encoding="utf-8"))
        candidate_map.parent.mkdir(parents=True, exist_ok=True)
        candidate_map.write_text(json.dumps({
            "raw_servo": payload.get("raw_servo", []),
            "delta_eq_rad": payload.get("delta_eq_rad", []),
            "plot_label": "archived candidate (not revalidated)",
            "source": str(legacy_map),
        }, indent=2) + "\n", encoding="utf-8")


def render(source: Path, output: Path) -> dict[str, object]:
    plots = output / "plots" / "stages"
    records: list[dict[str, str]] = []
    for key in PLOT_STAGES:
        stage = STAGE_BY_KEY[key]
        try:
            relative = _plot_stage(output, stage, plots)
            status = "generated" if relative else "not_available"
            records.append({"stage": key, "status": status, "plot": relative or ""})
        except Exception as exc:
            records.append({"stage": key, "status": "error", "plot": "", "reason": repr(exc)})
    manifest: dict[str, object] = {
        "source_session": str(source),
        "output": str(output),
        "scope": "real archived steering data; no synthetic fixtures",
        "plots": records,
        "unavailable_from_archived_campaign": [
            "physical_metrology", "steering_centre_validation", "steering_response_validation",
            "all longitudinal, selected-odometry and lateral-stiffness stages",
        ],
    }
    (output / "preview_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source_session", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--prepare-only", action="store_true")
    args = parser.parse_args()
    source = args.source_session.expanduser().resolve()
    output = args.output.expanduser().resolve()
    if not (source / "analysis").is_dir():
        raise SystemExit(f"not a steering analysis session: {source}")
    output.mkdir(parents=True, exist_ok=True)
    prepare(source, output)
    if args.prepare_only:
        print(output)
        return 0
    print(json.dumps(render(source, output), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
