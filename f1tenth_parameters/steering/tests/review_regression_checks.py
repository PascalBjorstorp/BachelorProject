#!/usr/bin/env python3
"""Static regression checks for the four review findings.

Run from the steering package root:
    python3 tests/review_regression_checks.py

These checks intentionally avoid ROS hardware. They protect source contracts
that previously allowed an undefined runtime variable, launch/analysis geometry
drift, report-only holdout validation, a no-op centre-search quality gate,
and exact-float static-map grouping.
"""
from __future__ import annotations

import ast
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


def check_stage_source() -> None:
    source = (ROOT / "steering_calibration" / "stages.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    names = {node.id for node in ast.walk(tree) if isinstance(node, ast.Name)}
    assert "max_error" not in names, "stale undefined Stage-0 variable present"
    assert "max_command_path_error" in source
    assert "candidate_yaw_gate_passed" in source
    assert "centre-search refinement gate failed" in source
    assert "centre confirmation gate failed" in source
    assert "hold_metadata" in source
    assert '"speed_mps", "raw_servo", "duration_s", "phase", "segment_id"' in source
    assert "static_map_settle_echo_mismatch" in source
    assert "raw_servo_echo_mismatch" in source
    assert "_settle_raw_servo" in source
    assert "settle_echo_max_error" in source


def check_geometry_contract() -> None:
    cfg = yaml.safe_load((ROOT / "config" / "steering_calibration.yaml").read_text(encoding="utf-8"))
    hardware = cfg["hardware"]
    for key in (
        "lidar_ip_address",
        "laser_to_base_x_m",
        "laser_to_base_y_m",
        "laser_to_base_z_m",
        "laser_to_base_yaw_rad",
        "base_frame_id",
        "laser_frame_id",
        "imu_frame_id",
    ):
        assert key in hardware, f"missing hardware key: {key}"
    launch = (ROOT / "launch" / "calibration_stack.py").read_text(encoding="utf-8")
    session = (ROOT / "steering_calibration" / "session.py").read_text(encoding="utf-8")
    motion = (ROOT / "analysis" / "estimate_lidar_motion.py").read_text(encoding="utf-8")
    assert "--config" in launch and "_hardware_from_config" in launch
    assert "calibration_config_snapshot.yaml" in session
    assert '"0.265"' not in launch, "hard-coded LiDAR x transform remains in launch"
    assert "R_base_laser @ result.R @ R_laser_base" in motion


def check_static_map_gate() -> None:
    cfg = yaml.safe_load((ROOT / "config" / "steering_calibration.yaml").read_text(encoding="utf-8"))
    quality = cfg["analysis"]["map"]
    for key in (
        "min_training_points",
        "min_holdout_points",
        "max_holdout_rmse_rad",
        "max_abs_holdout_bias_rad",
        "max_hysteresis_median_abs_rad",
        "max_repeatability_median_std_rad",
    ):
        assert key in quality, f"missing static-map gate: {key}"
    source = (ROOT / "analysis" / "fit_static_map.py").read_text(encoding="utf-8")
    assert "accepted_for_deployment" in source
    assert "static-map hold-out validation failed" in source
    assert 'groupby(["raw_servo_echo"' not in source
    assert 'index=["raw_servo_echo"' not in source
    assert "static_map_condition_coverage.parquet" in source
    runner = (ROOT / "analysis" / "run_analysis.py").read_text(encoding="utf-8")
    assert '"--strict"' in runner
    assert "summarize_simulation_seeds.py" in runner


def main() -> int:
    check_stage_source()
    check_geometry_contract()
    check_static_map_gate()
    cfg = yaml.safe_load((ROOT / "config" / "steering_calibration.yaml").read_text(encoding="utf-8"))
    assert "steering_settle_timeout_s" in cfg["static_map"]
    assert "steering_settle_final_window_s" in cfg["static_map"]
    assert "max_center_servo_spread" not in cfg["centre_trim"]
    assert "measurements_per_side" not in cfg["endstops"]
    assert int(cfg["response"]["repetitions"]) == 5
    outputs = (ROOT / "docs" / "STEERING_PARAMETER_OUTPUTS.md").read_text(encoding="utf-8")
    assert "steering_simulation_seed_report.json" in outputs
    print("review regression checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
