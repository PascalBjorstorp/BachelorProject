#!/usr/bin/env python3
"""Run GPU AMCL particle-injection benchmark and call MATLAB plots."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shlex
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Sequence


BENCHMARK_NAME = "AMCL_benchmark"
DEFAULT_INJECTION_RATIOS = (0.0, 0.05, 0.10, 0.15, 0.20)


def ratio_slug(ratio: float) -> str:
    return f"inj_{int(round(100.0 * ratio)):02d}pct"


def ratio_label(ratio: float) -> str:
    return f"{int(round(100.0 * ratio))}%"


def parse_ratios(text: str) -> List[float]:
    ratios: List[float] = []
    for token in text.split(","):
        token = token.strip()
        if not token:
            continue
        value = float(token)
        if value > 1.0:
            value *= 0.01
        if value < 0.0 or value > 1.0:
            raise ValueError(f"Injection ratio must be in [0, 1] or [0, 100] percent: {token}")
        ratios.append(value)
    if not ratios:
        raise ValueError("At least one injection ratio is required")
    return ratios


def read_status(path: Path) -> Dict[str, object]:
    if not path.exists():
        return {}
    try:
        with path.open() as handle:
            return json.load(handle)
    except (OSError, json.JSONDecodeError):
        return {}


def write_manifest(path: Path, rows: Sequence[Dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "condition",
        "label",
        "injection_ratio",
        "injection_percent",
        "run",
        "run_dir",
        "csv_path",
        "status_path",
        "log_path",
        "returncode",
        "status_reason",
        "status_laps",
    ]
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def build_command(args: argparse.Namespace, ratio: float, run_dir: Path) -> List[str]:
    benchmark_script = Path(args.benchmark_script).resolve()
    process_timeout = args.process_timeout_sec
    if process_timeout <= 0.0 and args.max_duration_sec > 0.0:
        process_timeout = args.max_duration_sec + 120.0

    cmd = [
        sys.executable,
        str(benchmark_script),
        "--output-root", str(run_dir),
        "--laps", str(args.laps),
        "--max-duration-sec", str(args.max_duration_sec),
        "--process-timeout-sec", str(process_timeout),
        "--map-file", str(args.map_file),
        "--trajectory-file", str(args.trajectory_file),
        "--initial-pose-x", str(args.initial_pose_x),
        "--initial-pose-y", str(args.initial_pose_y),
        "--sim-odom-source", args.sim_odom_source,
        "--sim-drive-input-mode", args.sim_drive_input_mode,
        "--mpc-raceline-speed-margin", str(args.mpc_raceline_speed_margin),
        "--cloud-publish-rate", str(args.cloud_publish_rate),
        "--amcl-num-particles", str(args.particles),
        "--amcl-min-particles", str(args.particles),
        "--amcl-max-particles", str(args.particles),
        "--amcl-max-beams", str(args.max_beams),
        "--amcl-likelihood-scale", str(args.likelihood_scale),
        "--amcl-cluster-publish-min-weight", str(args.cluster_publish_min_weight),
        "--amcl-max-scan-age", str(args.max_scan_age),
        "--amcl-enable-recovery-injection",
        "--amcl-recovery-injection-ratio", f"{ratio:.9g}",
        "--no-debug-pre-resample-particles",
    ]

    if args.initial_pose_yaw is not None:
        cmd.extend(["--initial-pose-yaw", str(args.initial_pose_yaw)])
    cmd.append("--headless" if args.headless else "--no-headless")
    cmd.append("--global-localization" if args.global_localization else "--no-global-localization")
    if args.no_realistic_plant:
        cmd.append("--no-realistic-plant")
    if not args.sim_drive_uses_acceleration_field:
        cmd.append("--no-sim-drive-uses-acceleration-field")
    if args.avoidance_enabled:
        cmd.append("--avoidance-enabled")

    for extra in args.extra_benchmark_arg:
        cmd.append(extra)
    return cmd


def load_workspace_env(repo_root: Path) -> Dict[str, str]:
    env = os.environ.copy()
    setup_path = repo_root / "install" / "setup.bash"
    if not setup_path.exists():
        return env

    command = f"source {shlex.quote(str(setup_path))} && env -0"
    result = subprocess.run(
        ["bash", "-lc", command],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False)
    if result.returncode != 0:
        print(f"[warn] failed to source {setup_path}; using current environment")
        return env

    loaded: Dict[str, str] = {}
    for item in result.stdout.split(b"\0"):
        if not item or b"=" not in item:
            continue
        key, value = item.split(b"=", 1)
        loaded[key.decode()] = value.decode()
    loaded.setdefault("PYTHONUNBUFFERED", "1")
    loaded.setdefault("RCUTILS_LOGGING_BUFFERED_STREAM", "1")
    return loaded


def expected_paths(output_root: Path, ratio: float, run_index: int) -> Dict[str, Path]:
    run_dir = output_root / ratio_slug(ratio) / f"run_{run_index:02d}"
    benchmark_dir = run_dir / BENCHMARK_NAME
    return {
        "run_dir": run_dir,
        "csv_path": benchmark_dir / f"{BENCHMARK_NAME}.csv",
        "status_path": benchmark_dir / f"{BENCHMARK_NAME}_status.json",
        "log_path": run_dir / "benchmark_stdout.log",
    }


def row_for_run(output_root: Path,
                ratio: float,
                run_index: int,
                returncode: object = "") -> Dict[str, object]:
    paths = expected_paths(output_root, ratio, run_index)
    status = read_status(paths["status_path"])
    return {
        "condition": ratio_slug(ratio),
        "label": ratio_label(ratio),
        "injection_ratio": ratio,
        "injection_percent": 100.0 * ratio,
        "run": run_index,
        "run_dir": str(paths["run_dir"]),
        "csv_path": str(paths["csv_path"]),
        "status_path": str(paths["status_path"]),
        "log_path": str(paths["log_path"]),
        "returncode": returncode,
        "status_reason": status.get("reason", ""),
        "status_laps": status.get("laps", ""),
    }


def run_benchmark_case(args: argparse.Namespace,
                       ratio: float,
                       run_index: int,
                       manifest_rows: List[Dict[str, object]],
                       manifest_path: Path) -> None:
    paths = expected_paths(args.output_root, ratio, run_index)
    run_dir = paths["run_dir"]
    status_path = paths["status_path"]

    if args.resume and status_path.exists():
        row = row_for_run(args.output_root, ratio, run_index, 0)
        manifest_rows.append(row)
        write_manifest(manifest_path, manifest_rows)
        print(f"skip {ratio_label(ratio)} run {run_index:02d}: existing status")
        return

    if run_dir.exists():
        shutil.rmtree(run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)

    cmd = build_command(args, ratio, run_dir)
    print(f"\n=== injection {ratio_label(ratio)} run {run_index:02d}/{args.runs} ===")
    print(" ".join(cmd))
    if args.dry_run:
        manifest_rows.append(row_for_run(args.output_root, ratio, run_index, "dry_run"))
        write_manifest(manifest_path, manifest_rows)
        return

    with paths["log_path"].open("w") as log:
        log.write(" ".join(cmd) + "\n\n")
        log.flush()
        env = dict(args.ros_env)
        env.setdefault("ROS_LOG_DIR", str(run_dir / "ros_logs"))
        completed = subprocess.run(
            cmd,
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
            text=True,
            check=False)

    row = row_for_run(args.output_root, ratio, run_index, completed.returncode)
    manifest_rows.append(row)
    write_manifest(manifest_path, manifest_rows)

    if args.delete_bags:
        bag_dir = paths["run_dir"] / BENCHMARK_NAME / BENCHMARK_NAME
        if bag_dir.exists():
            shutil.rmtree(bag_dir)

    print(
        f"done {ratio_label(ratio)} run {run_index:02d}: "
        f"code={completed.returncode} "
        f"reason={row['status_reason']} laps={row['status_laps']}")
    if completed.returncode != 0 and args.stop_on_failure:
        raise RuntimeError(
            f"{ratio_label(ratio)} run {run_index:02d} failed with code "
            f"{completed.returncode}")


def matlab_quote(text: str) -> str:
    return "'" + text.replace("'", "''") + "'"


def run_matlab_plots(args: argparse.Namespace) -> None:
    matlab_dir = Path(args.matlab_plot_script).resolve().parent
    function_name = Path(args.matlab_plot_script).stem
    expr = (
        f"addpath({matlab_quote(str(matlab_dir))}); "
        f"{function_name}("
        f"{matlab_quote(str(args.output_root))}, "
        f"{matlab_quote(str(args.figure_output_dir))}, "
        "false);"
    )
    print("\n=== MATLAB plots ===")
    print(f"{args.matlab_command} -batch {expr}")
    subprocess.run([args.matlab_command, "-batch", expr], check=True)


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    repo_root = script_dir.parents[1]
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    default_output = (
        repo_root / "f1tenth_localization" / "Benchmark" / "Matlab" /
        "sim_benchmark" / f"gpu_amcl_injection_{timestamp}"
    )

    parser = argparse.ArgumentParser(
        description="Run GPU AMCL recovery particle-injection benchmark.")
    parser.add_argument(
        "--benchmark-script",
        type=Path,
        default=script_dir / "run_sim_gpu_amcl_benchmark.py")
    parser.add_argument(
        "--matlab-plot-script",
        type=Path,
        default=script_dir / "Matlab" / "plot_sim_gpu_amcl_injection_test.m")
    parser.add_argument("--output-root", type=Path, default=default_output)
    parser.add_argument(
        "--figure-output-dir",
        type=Path,
        default=repo_root / "ReportMaterial" / "TestFigures")
    parser.add_argument("--runs", type=int, default=5)
    parser.add_argument("--laps", type=int, default=5)
    parser.add_argument(
        "--ratios",
        default=",".join(str(r) for r in DEFAULT_INJECTION_RATIOS),
        help="Comma-separated recovery injection ratios. Values above 1 are treated as percent.")
    parser.add_argument("--max-duration-sec", type=float, default=0.0)
    parser.add_argument("--process-timeout-sec", type=float, default=0.0)
    parser.add_argument("--particles", type=int, default=1000)
    parser.add_argument("--max-beams", type=int, default=270)
    parser.add_argument("--likelihood-scale", type=float, default=4.0)
    parser.add_argument("--cluster-publish-min-weight", type=float, default=0.60)
    parser.add_argument("--max-scan-age", type=float, default=0.12)
    parser.add_argument("--cloud-publish-rate", type=float, default=0.0)
    parser.add_argument(
        "--map-file",
        type=Path,
        default=repo_root / "f1tenth_planning" / "maps" / "my_track_map.yaml")
    parser.add_argument(
        "--trajectory-file",
        type=Path,
        default=repo_root / "f1tenth_planning" / "trajectories" / "my_track_raceline.csv")
    parser.add_argument("--initial-pose-x", type=float, default=0.5)
    parser.add_argument("--initial-pose-y", type=float, default=0.2)
    parser.add_argument("--initial-pose-yaw", type=float, default=None)
    parser.add_argument("--global-localization", action=argparse.BooleanOptionalAction,
                        default=False)
    parser.add_argument("--headless", action=argparse.BooleanOptionalAction,
                        default=True)
    parser.add_argument("--no-realistic-plant", action="store_true")
    parser.add_argument(
        "--sim-odom-source",
        choices=("vesc", "ground_truth", "calibrated_drift"),
        default="vesc",
        help=(
            "vesc uses simulated VESC/IMU sensors and vesc_to_odom; "
            "ground_truth uses old pose odom; calibrated_drift uses the "
            "OptiTrack-calibrated drift model."))
    parser.add_argument(
        "--sim-drive-input-mode",
        choices=("vesc", "ackermann"),
        default="vesc",
        help="vesc routes /ackermann_cmd through ackermann_to_vesc; ackermann feeds gym directly.")
    parser.add_argument("--sim-drive-uses-acceleration-field",
                        action=argparse.BooleanOptionalAction,
                        default=True)
    parser.add_argument("--mpc-raceline-speed-margin", type=float, default=0.0,
                        help="Extra speed above raceline v_ref passed to MPC.")
    parser.add_argument("--avoidance-enabled", action="store_true")
    parser.add_argument("--alternate-order", action=argparse.BooleanOptionalAction,
                        default=True)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--stop-on-failure", action="store_true")
    parser.add_argument("--plot-only", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--delete-bags", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--skip-matlab", action="store_true")
    parser.add_argument("--matlab-command", default="matlab")
    parser.add_argument("--extra-benchmark-arg", action="append", default=[])
    return parser.parse_args(argv)


def main(argv: Sequence[str]) -> int:
    args = parse_args(argv)
    args.output_root = args.output_root.resolve()
    args.figure_output_dir = args.figure_output_dir.resolve()
    args.map_file = args.map_file.resolve()
    args.trajectory_file = args.trajectory_file.resolve()
    args.ros_env = load_workspace_env(Path(__file__).resolve().parents[2])
    args.injection_ratios = parse_ratios(args.ratios)

    args.output_root.mkdir(parents=True, exist_ok=True)
    manifest_path = args.output_root / "particle_injection_manifest.csv"
    manifest_rows: List[Dict[str, object]] = []

    print(f"Output root: {args.output_root}")
    print(f"Figures    : {args.figure_output_dir}")
    print(f"Runs       : {args.runs}")
    print(f"Laps/run   : {args.laps}")
    print("Injection : " + ", ".join(ratio_label(ratio) for ratio in args.injection_ratios))

    if not args.plot_only:
        for run_index in range(1, args.runs + 1):
            ratios = list(args.injection_ratios)
            if args.alternate_order and run_index % 2 == 0:
                ratios.reverse()
            for ratio in ratios:
                run_benchmark_case(args, ratio, run_index, manifest_rows, manifest_path)
    elif not manifest_path.exists():
        for ratio in args.injection_ratios:
            for run_index in range(1, args.runs + 1):
                manifest_rows.append(row_for_run(args.output_root, ratio, run_index, ""))
        write_manifest(manifest_path, manifest_rows)

    if args.dry_run or args.skip_matlab:
        print(f"Manifest: {manifest_path}")
        return 0

    try:
        run_matlab_plots(args)
    except FileNotFoundError:
        print("MATLAB not found. Run plotting later with --plot-only or install MATLAB CLI.")
        return 1

    print(f"Manifest: {manifest_path}")
    print(f"Figures : {args.figure_output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
