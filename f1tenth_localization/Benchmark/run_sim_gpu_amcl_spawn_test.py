#!/usr/bin/env python3
"""Run GPU AMCL global-localization benchmark from many raceline spawn poses."""

import argparse
import csv
import json
import math
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


BENCHMARK_NAME = "AMCL_benchmark"


def angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def finite_float(value: str) -> Optional[float]:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def median(values: Sequence[float]) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def percentile(values: Sequence[float], pct: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    idx = int(round((pct / 100.0) * (len(ordered) - 1)))
    idx = max(0, min(idx, len(ordered) - 1))
    return ordered[idx]


def map_yaml_for(path: str) -> str:
    if path.lower().endswith(".pgm"):
        yaml_path = os.path.splitext(path)[0] + ".yaml"
        if os.path.exists(yaml_path):
            return yaml_path
        raise FileNotFoundError(
            f"Nav2 map_server needs YAML; no sibling YAML found for {path}")
    return path


def load_raceline(path: str) -> List[Dict[str, float]]:
    points: List[Dict[str, float]] = []
    with open(path, newline="") as handle:
        for raw in handle:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            parts = [part.strip() for part in line.split(",")]
            if len(parts) < 4:
                continue
            try:
                point = {
                    "s": float(parts[0]),
                    "x": float(parts[1]),
                    "y": float(parts[2]),
                    "yaw": float(parts[3]),
                    "d_left": float(parts[7]) if len(parts) > 7 else 0.0,
                    "d_right": float(parts[8]) if len(parts) > 8 else 0.0,
                }
            except ValueError:
                continue
            points.append(point)

    if not points:
        raise RuntimeError(f"No usable raceline points in {path}")
    return points


def make_spawn_poses(points: List[Dict[str, float]],
                     spawn_count: int,
                     lateral_pattern: Sequence[float],
                     max_lateral_offset: float,
                     wall_margin: float) -> List[Dict[str, float]]:
    if spawn_count <= 0:
        raise ValueError("spawn_count must be > 0")
    if not lateral_pattern:
        lateral_pattern = [0.0]

    poses: List[Dict[str, float]] = []
    n = len(points)
    for i in range(spawn_count):
        idx = int(round((i * n) / spawn_count)) % n
        point = points[idx]
        fraction = lateral_pattern[i % len(lateral_pattern)]

        left_available = max(0.0, min(point["d_left"], max_lateral_offset) - wall_margin)
        right_available = max(0.0, min(point["d_right"], max_lateral_offset) - wall_margin)
        if fraction >= 0.0:
            lateral = fraction * left_available
        else:
            lateral = fraction * right_available

        yaw = point["yaw"]
        nx_left = -math.sin(yaw)
        ny_left = math.cos(yaw)
        x = point["x"] + lateral * nx_left
        y = point["y"] + lateral * ny_left
        poses.append({
            "spawn_index": i,
            "raceline_index": idx,
            "s_m": point["s"],
            "x": x,
            "y": y,
            "yaw": yaw,
            "lateral_offset_m": lateral,
            "lateral_fraction": fraction,
        })
    return poses


def read_status(path: Path) -> Dict[str, object]:
    if not path.exists():
        return {}
    try:
        with path.open() as handle:
            return json.load(handle)
    except (OSError, json.JSONDecodeError):
        return {}


def summarize_csv(csv_path: Path,
                  tail_rows: int,
                  xy_threshold_m: float,
                  yaw_threshold_rad: float,
                  min_valid_fraction: float) -> Dict[str, object]:
    if not csv_path.exists():
        return {
            "success": False,
            "reason": "missing_csv",
            "rows": 0,
            "valid_rows": 0,
            "valid_fraction": 0.0,
        }

    rows = []
    with csv_path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            rows.append(row)

    amcl_xy_errors: List[float] = []
    amcl_yaw_errors: List[float] = []
    ekf_xy_errors: List[float] = []
    for row in rows:
        gt_x = finite_float(row.get("gt_x", ""))
        gt_y = finite_float(row.get("gt_y", ""))
        gt_yaw = finite_float(row.get("gt_yaw", ""))
        amcl_x = finite_float(row.get("amcl_x", ""))
        amcl_y = finite_float(row.get("amcl_y", ""))
        amcl_yaw = finite_float(row.get("amcl_yaw", ""))
        ekf_err = finite_float(row.get("err_xy", ""))
        if None in (gt_x, gt_y, gt_yaw, amcl_x, amcl_y, amcl_yaw):
            continue
        amcl_xy_errors.append(math.hypot(amcl_x - gt_x, amcl_y - gt_y))
        amcl_yaw_errors.append(abs(angle_diff(amcl_yaw, gt_yaw)))
        if ekf_err is not None:
            ekf_xy_errors.append(ekf_err)

    valid_rows = len(amcl_xy_errors)
    valid_fraction = valid_rows / len(rows) if rows else 0.0
    tail_xy = amcl_xy_errors[-tail_rows:] if tail_rows > 0 else amcl_xy_errors
    tail_yaw = amcl_yaw_errors[-tail_rows:] if tail_rows > 0 else amcl_yaw_errors
    tail_ekf = ekf_xy_errors[-tail_rows:] if tail_rows > 0 else ekf_xy_errors

    tail_median_xy = median(tail_xy)
    tail_median_yaw = median(tail_yaw)
    success = (
        valid_fraction >= min_valid_fraction and
        math.isfinite(tail_median_xy) and
        math.isfinite(tail_median_yaw) and
        tail_median_xy <= xy_threshold_m and
        tail_median_yaw <= yaw_threshold_rad
    )

    reason = "ok" if success else "localization_error"
    if valid_fraction < min_valid_fraction:
        reason = "too_few_valid_amcl_rows"

    return {
        "success": success,
        "reason": reason,
        "rows": len(rows),
        "valid_rows": valid_rows,
        "valid_fraction": valid_fraction,
        "amcl_final_err_xy_m": amcl_xy_errors[-1] if amcl_xy_errors else math.nan,
        "amcl_tail_median_err_xy_m": tail_median_xy,
        "amcl_tail_p95_err_xy_m": percentile(tail_xy, 95.0),
        "amcl_tail_median_abs_yaw_rad": tail_median_yaw,
        "amcl_tail_p95_abs_yaw_rad": percentile(tail_yaw, 95.0),
        "ekf_tail_median_err_xy_m": median(tail_ekf),
        "ekf_tail_p95_err_xy_m": percentile(tail_ekf, 95.0),
    }


def write_csv(path: Path, rows: List[Dict[str, object]]) -> None:
    if not rows:
        return
    keys: List[str] = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def result_row_failed(row: Dict[str, object]) -> bool:
    try:
        returncode = int(float(str(row.get("returncode", "1"))))
    except ValueError:
        returncode = 1
    success = str(row.get("success", "")).strip().lower() == "true"
    return returncode != 0 or not success


def run_spawn(args: argparse.Namespace,
              pose: Dict[str, float],
              map_file: str,
              trajectory_file: str,
              benchmark_script: str) -> Tuple[int, Path]:
    spawn_dir = Path(args.output_root) / f"spawn_{int(pose['spawn_index']):03d}"
    spawn_dir.mkdir(parents=True, exist_ok=True)
    log_path = spawn_dir / "benchmark_stdout.log"
    process_timeout = args.process_timeout_sec
    if process_timeout <= 0.0 and args.max_duration_sec > 0.0:
        process_timeout = args.max_duration_sec + 90.0

    cmd = [
        sys.executable,
        benchmark_script,
        "--output-root", str(spawn_dir),
        "--laps", str(args.laps),
        "--max-duration-sec", str(args.max_duration_sec),
        "--process-timeout-sec", str(process_timeout),
        "--map-file", map_file,
        "--trajectory-file", trajectory_file,
        "--initial-pose-x", f"{pose['x']:.9f}",
        "--initial-pose-y", f"{pose['y']:.9f}",
        "--initial-pose-yaw", f"{pose['yaw']:.9f}",
        "--sim-odom-source", args.sim_odom_source,
        "--sim-drive-input-mode", args.sim_drive_input_mode,
        "--mpc-raceline-speed-margin", str(args.mpc_raceline_speed_margin),
        "--global-localization",
        "--amcl-num-particles", str(args.amcl_num_particles),
        "--amcl-min-particles", str(args.amcl_min_particles),
        "--amcl-max-particles", str(args.amcl_max_particles),
        "--amcl-max-beams", str(args.amcl_max_beams),
        "--amcl-update-min-d", str(args.amcl_update_min_d),
        "--amcl-update-min-a", str(args.amcl_update_min_a),
        "--amcl-likelihood-scale", str(args.amcl_likelihood_scale),
        "--amcl-alpha1", str(args.amcl_alpha1),
        "--amcl-alpha2", str(args.amcl_alpha2),
        "--amcl-alpha3", str(args.amcl_alpha3),
        "--amcl-alpha4", str(args.amcl_alpha4),
        "--amcl-z-hit", str(args.amcl_z_hit),
        "--amcl-z-rand", str(args.amcl_z_rand),
        "--amcl-sigma-hit", str(args.amcl_sigma_hit),
        "--amcl-resample-threshold", str(args.amcl_resample_threshold),
        "--amcl-cluster-publish-min-weight", str(args.amcl_cluster_publish_min_weight),
        "--ekf-process-noise-scale", str(args.ekf_process_noise_scale),
        "--no-debug-pre-resample-particles",
    ]
    cmd.append(
        "--sim-drive-uses-acceleration-field"
        if args.sim_drive_uses_acceleration_field
        else "--no-sim-drive-uses-acceleration-field")
    cmd.append("--realistic-plant" if args.realistic_plant else "--no-realistic-plant")
    if args.extra_benchmark_arg:
        cmd.extend(args.extra_benchmark_arg)

    with log_path.open("w") as log:
        log.write(" ".join(cmd) + "\n\n")
        log.flush()
        completed = subprocess.run(
            cmd,
            stdout=log,
            stderr=subprocess.STDOUT,
            text=True,
            check=False)
    return completed.returncode, spawn_dir


def parse_lateral_pattern(text: str) -> List[float]:
    values = []
    for token in text.split(","):
        token = token.strip()
        if not token:
            continue
        values.append(float(token))
    return values


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[2]
    default_output = (
        repo_root / "f1tenth_localization" / "Benchmark" / "Matlab" /
        "sim_benchmark" / "gpu_amcl_spawn_test")
    parser = argparse.ArgumentParser(
        description="Run existing GPU AMCL benchmark from many raceline spawn poses.")
    parser.add_argument(
        "--benchmark-script",
        default=str(Path(__file__).with_name("run_sim_gpu_amcl_benchmark.py")))
    parser.add_argument("--output-root", default=str(default_output))
    parser.add_argument(
        "--map-file",
        default=str(repo_root / "f1tenth_planning" / "maps" / "my_track_map.yaml"),
        help="Map YAML or PGM. PGM is resolved to sibling YAML for map_server.")
    parser.add_argument(
        "--trajectory-file",
        default=str(repo_root / "f1tenth_localization" / "Benchmark" /
                    "Matlab" / "sim_benchmark" / "my_track_raceline_vcap_4p0.csv"))
    parser.add_argument("--spawn-count", type=int, default=50)
    parser.add_argument("--max-duration-sec", type=float, default=12.0)
    parser.add_argument("--process-timeout-sec", type=float, default=0.0)
    parser.add_argument("--laps", type=int, default=0)
    parser.add_argument("--xy-threshold-m", type=float, default=0.50)
    parser.add_argument("--yaw-threshold-rad", type=float, default=0.50)
    parser.add_argument("--min-valid-fraction", type=float, default=0.20)
    parser.add_argument("--tail-rows", type=int, default=200)
    parser.add_argument("--max-lateral-offset-m", type=float, default=1.0)
    parser.add_argument("--wall-margin-m", type=float, default=0.20)
    parser.add_argument("--sim-odom-source",
                        choices=("vesc", "ground_truth"),
                        default="vesc",
                        help="Forwarded to benchmark; vesc uses simulated VESC/IMU odometry.")
    parser.add_argument("--sim-drive-input-mode",
                        choices=("vesc", "ackermann"),
                        default="vesc",
                        help="Forwarded to benchmark; vesc routes MPC acceleration through current/brake.")
    parser.add_argument("--sim-drive-uses-acceleration-field",
                        action=argparse.BooleanOptionalAction,
                        default=True)
    parser.add_argument("--realistic-plant",
                        action=argparse.BooleanOptionalAction,
                        default=True)
    parser.add_argument("--mpc-raceline-speed-margin", type=float, default=0.0)
    parser.add_argument("--amcl-num-particles", type=int, default=1000)
    parser.add_argument("--amcl-min-particles", type=int, default=1000)
    parser.add_argument("--amcl-max-particles", type=int, default=1000)
    parser.add_argument("--amcl-max-beams", type=int, default=270)
    parser.add_argument("--amcl-update-min-d", type=float, default=0.05)
    parser.add_argument("--amcl-update-min-a", type=float, default=0.05)
    parser.add_argument("--amcl-likelihood-scale", type=float, default=0.75)
    parser.add_argument("--amcl-alpha1", type=float, default=0.4)
    parser.add_argument("--amcl-alpha2", type=float, default=0.4)
    parser.add_argument("--amcl-alpha3", type=float, default=0.2)
    parser.add_argument("--amcl-alpha4", type=float, default=0.2)
    parser.add_argument("--amcl-z-hit", type=float, default=0.95)
    parser.add_argument("--amcl-z-rand", type=float, default=0.05)
    parser.add_argument("--amcl-sigma-hit", type=float, default=0.10)
    parser.add_argument("--amcl-resample-threshold", type=float, default=0.3)
    parser.add_argument("--amcl-cluster-publish-min-weight", type=float, default=0.60)
    parser.add_argument("--ekf-process-noise-scale", type=float, default=0.1)
    parser.add_argument(
        "--lateral-pattern",
        default="-0.75,-0.35,0.0,0.35,0.75",
        help="Comma-separated side fractions; positive is left, negative is right.")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--resume", action="store_true",
                        help="Keep existing spawn_results.csv rows and skip those spawn indices.")
    parser.add_argument("--stop-on-failure", action="store_true")
    parser.add_argument("--delete-bags", action="store_true",
                        help="Delete nested recorded bag directories after parsing CSV.")
    parser.add_argument("--extra-benchmark-arg", action="append",
                        help="Extra argument forwarded to run_sim_gpu_amcl_benchmark.py.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    map_file = map_yaml_for(args.map_file)
    trajectory_file = os.path.abspath(args.trajectory_file)
    benchmark_script = os.path.abspath(args.benchmark_script)
    output_root = Path(args.output_root)
    output_root.mkdir(parents=True, exist_ok=True)

    points = load_raceline(trajectory_file)
    poses = make_spawn_poses(
        points=points,
        spawn_count=args.spawn_count,
        lateral_pattern=parse_lateral_pattern(args.lateral_pattern),
        max_lateral_offset=args.max_lateral_offset_m,
        wall_margin=args.wall_margin_m)

    manifest_path = output_root / "spawn_manifest.json"
    with manifest_path.open("w") as handle:
        json.dump({
            "map_file": map_file,
            "trajectory_file": trajectory_file,
            "benchmark_script": benchmark_script,
            "spawn_count": len(poses),
            "sim_odom_source": args.sim_odom_source,
            "sim_drive_input_mode": args.sim_drive_input_mode,
            "sim_drive_uses_acceleration_field": args.sim_drive_uses_acceleration_field,
            "realistic_plant": args.realistic_plant,
            "mpc_raceline_speed_margin": args.mpc_raceline_speed_margin,
            "poses": poses,
        }, handle, indent=2)
        handle.write("\n")

    print(f"Spawn count: {len(poses)}")
    print(f"Output: {output_root}")
    print(f"Manifest: {manifest_path}")
    if args.dry_run:
        print("Dry run only.")
        return 0

    results_path = output_root / "spawn_results.csv"
    rows: List[Dict[str, object]] = []
    completed_indices = set()
    if args.resume and results_path.exists():
        with results_path.open(newline="") as handle:
            for row in csv.DictReader(handle):
                rows.append(dict(row))
                spawn_index = finite_float(row.get("spawn_index", ""))
                if spawn_index is not None:
                    completed_indices.add(int(spawn_index))
        print(f"Resume: loaded {len(rows)} existing result rows.")

    failures = sum(1 for row in rows if result_row_failed(row))
    for pose in poses:
        index = int(pose["spawn_index"])
        if index in completed_indices:
            print(f"\n=== spawn {index + 1}/{len(poses)} already done; skipping ===")
            continue
        print(
            f"\n=== spawn {index + 1}/{len(poses)} "
            f"x={pose['x']:.2f} y={pose['y']:.2f} yaw={pose['yaw']:.2f} ===")
        returncode, spawn_dir = run_spawn(
            args, pose, map_file, trajectory_file, benchmark_script)
        csv_path = spawn_dir / BENCHMARK_NAME / f"{BENCHMARK_NAME}.csv"
        status_path = spawn_dir / BENCHMARK_NAME / f"{BENCHMARK_NAME}_status.json"
        metrics = summarize_csv(
            csv_path,
            tail_rows=args.tail_rows,
            xy_threshold_m=args.xy_threshold_m,
            yaw_threshold_rad=args.yaw_threshold_rad,
            min_valid_fraction=args.min_valid_fraction)
        status = read_status(status_path)

        row: Dict[str, object] = {
            **pose,
            "returncode": returncode,
            "status_reason": status.get("reason", ""),
            "status_laps": status.get("laps", ""),
            "csv_path": str(csv_path),
            "log_path": str(spawn_dir / "benchmark_stdout.log"),
            **metrics,
        }
        rows.append(row)
        write_csv(results_path, rows)

        if args.delete_bags:
            bag_dir = spawn_dir / BENCHMARK_NAME / BENCHMARK_NAME
            if bag_dir.exists():
                shutil.rmtree(bag_dir)

        print(
            f"success={metrics['success']} reason={metrics['reason']} "
            f"amcl_med={metrics.get('amcl_tail_median_err_xy_m', math.nan):.3f}m "
            f"yaw_med={metrics.get('amcl_tail_median_abs_yaw_rad', math.nan):.3f}rad")

        if returncode != 0 or not metrics["success"]:
            failures += 1
            if args.stop_on_failure:
                break

    passed = len(rows) - failures
    print(f"\nSummary: passed={passed}/{len(rows)} failed={failures}")
    print(f"Results: {results_path}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
