#!/usr/bin/env python3
"""Run GPU AMCL fixed-particle vs KLD epsilon sweep in simulation."""

from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import statistics
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Sequence


def percentile(values: Sequence[float], q: float) -> float:
    finite = sorted(v for v in values if math.isfinite(v))
    if not finite:
        return math.nan
    if len(finite) == 1:
        return finite[0]
    pos = (len(finite) - 1) * q
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return finite[lo]
    w = pos - lo
    return finite[lo] * (1.0 - w) + finite[hi] * w


def mean_or_nan(values: Sequence[float]) -> float:
    finite = [v for v in values if math.isfinite(v)]
    return statistics.fmean(finite) if finite else math.nan


def median_or_nan(values: Sequence[float]) -> float:
    finite = [v for v in values if math.isfinite(v)]
    return statistics.median(finite) if finite else math.nan


def float_field(row: Dict[str, str], name: str) -> float:
    try:
        value = float(row.get(name, ""))
    except (TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def parse_epsilons(text: str) -> List[float]:
    values = []
    for token in text.split(","):
        token = token.strip()
        if token:
            values.append(float(token))
    if not values:
        raise ValueError("At least one epsilon value is required")
    return values


def condition_name(epsilon: float | None) -> str:
    if epsilon is None:
        return "fixed"
    return f"kld_eps_{str(epsilon).replace('.', 'p')}"


def condition_label(epsilon: float | None) -> str:
    if epsilon is None:
        return "Fixed"
    return f"KLD eps {epsilon:g}"


def latest_pipeline_csv(run_root: Path) -> Path | None:
    matches = sorted(run_root.glob("AMCL_benchmark/pipeline/pipeline_latency_*.csv"))
    if matches:
        return matches[-1]
    matches = sorted(run_root.glob("AMCL_benchmark/pipeline/Pipeline_*.csv"))
    return matches[-1] if matches else None


def benchmark_csv(run_root: Path) -> Path:
    return run_root / "AMCL_benchmark" / "AMCL_benchmark.csv"


def status_json(run_root: Path) -> Path:
    return run_root / "AMCL_benchmark" / "AMCL_benchmark_status.json"


def load_pipeline_csv(path: Path, skip_first_sec: float) -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    with path.open(newline="") as handle:
        raw_rows = list(csv.DictReader(handle))
    if not raw_rows:
        return rows

    first_wall = float_field(raw_rows[0], "wall_time_ns")
    for raw in raw_rows:
        wall_ns = float_field(raw, "wall_time_ns")
        t_rel = (wall_ns - first_wall) * 1e-9 if math.isfinite(wall_ns) and math.isfinite(first_wall) else math.nan
        if math.isfinite(t_rel) and t_rel < skip_first_sec:
            continue
        scan_to_amcl = float_field(raw, "scan_to_amcl_ms")
        if not math.isfinite(scan_to_amcl):
            continue
        rows.append({
            "t_rel_s": t_rel,
            "scan_to_amcl_ms": scan_to_amcl,
            "amcl_particle_count": float_field(raw, "amcl_particle_count"),
            "amcl_predict_ms": float_field(raw, "amcl_predict_ms"),
            "amcl_sensor_model_ms": float_field(raw, "amcl_sensor_model_ms"),
            "amcl_normalize_ms": float_field(raw, "amcl_normalize_ms"),
            "amcl_scan_confidence_ms": float_field(raw, "amcl_scan_confidence_ms"),
            "amcl_update_weights_total_ms": float_field(raw, "amcl_update_weights_total_ms"),
            "amcl_cluster_estimate_ms": float_field(raw, "amcl_cluster_estimate_ms"),
            "amcl_resample_ms": float_field(raw, "amcl_resample_ms"),
            "amcl_kld_target_ms": float_field(raw, "amcl_kld_target_ms"),
            "amcl_full_compute_ms": float_field(raw, "amcl_full_compute_ms"),
            "cpu_to_gpu_scan_ms": float_field(raw, "cpu_to_gpu_scan_ms"),
            "gpu_to_cpu_particles_ms": float_field(raw, "gpu_to_cpu_particles_ms"),
            "gpu_to_cpu_weights_ms": float_field(raw, "gpu_to_cpu_weights_ms"),
            "cpu_gpu_transfer_total_ms": float_field(raw, "cpu_gpu_transfer_total_ms"),
            "amcl_cluster_weight": float_field(raw, "amcl_cluster_weight"),
            "amcl_raycast_setup_ms": float_field(raw, "amcl_raycast_setup_ms"),
            "amcl_raycast_score_ms": float_field(raw, "amcl_raycast_score_ms"),
            "amcl_raycast_correction_ms": float_field(raw, "amcl_raycast_correction_ms"),
        })
    return rows


def load_benchmark_errors(path: Path, skip_first_sec: float) -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    with path.open(newline="") as handle:
        raw_rows = list(csv.DictReader(handle))
    if not raw_rows:
        return rows

    first_wall = float_field(raw_rows[0], "wall_time_ns")
    for raw in raw_rows:
        wall_ns = float_field(raw, "wall_time_ns")
        t_rel = (wall_ns - first_wall) * 1e-9 if math.isfinite(wall_ns) and math.isfinite(first_wall) else math.nan
        if math.isfinite(t_rel) and t_rel < skip_first_sec:
            continue

        gt_x = float_field(raw, "gt_x")
        gt_y = float_field(raw, "gt_y")
        gt_yaw = float_field(raw, "gt_yaw")
        amcl_x = float_field(raw, "amcl_x")
        amcl_y = float_field(raw, "amcl_y")
        amcl_yaw = float_field(raw, "amcl_yaw")
        if not all(math.isfinite(v) for v in (gt_x, gt_y, gt_yaw, amcl_x, amcl_y, amcl_yaw)):
            continue
        rows.append({
            "t_rel_s": t_rel,
            "amcl_error_xy_m": math.hypot(amcl_x - gt_x, amcl_y - gt_y),
            "amcl_abs_yaw_error_rad": abs(angle_diff(amcl_yaw, gt_yaw)),
            "kld_pre_particles": float_field(raw, "kld_pre_particles"),
            "kld_occupied_bins": float_field(raw, "kld_occupied_bins"),
            "kld_target_unclamped": float_field(raw, "kld_target_unclamped"),
            "kld_target_clamped": float_field(raw, "kld_target_clamped"),
            "kld_epsilon": float_field(raw, "kld_epsilon"),
            "kld_z": float_field(raw, "kld_z"),
            "kld_bin_x": float_field(raw, "kld_bin_x"),
            "kld_bin_y": float_field(raw, "kld_bin_y"),
            "kld_bin_theta": float_field(raw, "kld_bin_theta"),
            "kld_sequence": float_field(raw, "kld_sequence"),
        })
    return rows


def run_duration(rows: Sequence[Dict[str, float]]) -> float:
    times = [r["t_rel_s"] for r in rows if math.isfinite(r["t_rel_s"])]
    if len(times) < 2:
        return math.nan
    return max(times) - min(times)


def summarize(condition: str,
              label: str,
              run_name: str,
              rows: Sequence[Dict[str, float]],
              error_rows: Sequence[Dict[str, float]],
              max_particles: int,
              status: Dict[str, object],
              duration_override_s: float | None = None) -> Dict[str, object]:
    latency = [r["scan_to_amcl_ms"] for r in rows if math.isfinite(r["scan_to_amcl_ms"])]
    particles = [
        r["amcl_particle_count"] for r in rows
        if math.isfinite(r["amcl_particle_count"]) and r["amcl_particle_count"] >= 0
    ]
    stage_series = {
        name: [
            r[name] for r in rows
            if math.isfinite(r.get(name, math.nan)) and r[name] >= 0.0
        ]
        for name in (
            "amcl_predict_ms",
            "amcl_sensor_model_ms",
            "amcl_normalize_ms",
            "amcl_scan_confidence_ms",
            "amcl_update_weights_total_ms",
            "amcl_cluster_estimate_ms",
            "amcl_resample_ms",
            "amcl_kld_target_ms",
            "amcl_full_compute_ms",
            "cpu_to_gpu_scan_ms",
            "gpu_to_cpu_particles_ms",
            "gpu_to_cpu_weights_ms",
            "cpu_gpu_transfer_total_ms",
            "amcl_cluster_weight",
            "amcl_raycast_setup_ms",
            "amcl_raycast_score_ms",
            "amcl_raycast_correction_ms",
        )
    }
    xy_errors = [r["amcl_error_xy_m"] for r in error_rows if math.isfinite(r["amcl_error_xy_m"])]
    yaw_errors = [
        r["amcl_abs_yaw_error_rad"] for r in error_rows
        if math.isfinite(r["amcl_abs_yaw_error_rad"])
    ]
    kld_bins = [r["kld_occupied_bins"] for r in error_rows if math.isfinite(r.get("kld_occupied_bins", math.nan))]
    kld_target_unclamped = [
        r["kld_target_unclamped"] for r in error_rows
        if math.isfinite(r.get("kld_target_unclamped", math.nan))
    ]
    kld_target_clamped = [
        r["kld_target_clamped"] for r in error_rows
        if math.isfinite(r.get("kld_target_clamped", math.nan))
    ]

    at_max = [p >= 0.95 * max_particles for p in particles]
    max_fraction = sum(at_max) / len(at_max) if at_max else math.nan
    duration_s = duration_override_s if duration_override_s is not None else run_duration(rows)
    time_at_max_s = duration_s * max_fraction if math.isfinite(duration_s) and math.isfinite(max_fraction) else math.nan

    out = {
        "condition": condition,
        "label": label,
        "run": run_name,
        "status_reason": status.get("reason", ""),
        "status_laps": status.get("laps", ""),
        "n_latency_samples": len(latency),
        "n_error_samples": len(xy_errors),
        "duration_s": duration_s,
        "scan_to_amcl_mean_ms": mean_or_nan(latency),
        "scan_to_amcl_median_ms": median_or_nan(latency),
        "scan_to_amcl_p95_ms": percentile(latency, 0.95),
        "scan_to_amcl_max_ms": max(latency) if latency else math.nan,
        "amcl_position_error_median_m": median_or_nan(xy_errors),
        "amcl_position_error_p95_m": percentile(xy_errors, 0.95),
        "amcl_yaw_error_median_rad": median_or_nan(yaw_errors),
        "amcl_yaw_error_p95_rad": percentile(yaw_errors, 0.95),
        "particle_mean": mean_or_nan(particles),
        "particle_median": median_or_nan(particles),
        "particle_p95": percentile(particles, 0.95),
        "particle_min": min(particles) if particles else math.nan,
        "particle_max": max(particles) if particles else math.nan,
        "time_at_or_near_max_particles_s": time_at_max_s,
        "fraction_at_or_near_max_particles": max_fraction,
        "kld_occupied_bins_median": median_or_nan(kld_bins),
        "kld_occupied_bins_p95": percentile(kld_bins, 0.95),
        "kld_target_unclamped_median": median_or_nan(kld_target_unclamped),
        "kld_target_unclamped_p95": percentile(kld_target_unclamped, 0.95),
        "kld_target_clamped_median": median_or_nan(kld_target_clamped),
        "kld_target_clamped_p95": percentile(kld_target_clamped, 0.95),
    }
    for name, values in stage_series.items():
        out[f"{name}_median"] = median_or_nan(values)
        out[f"{name}_p95"] = percentile(values, 0.95)
    return out


def write_csv(path: Path, rows: Sequence[Dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
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


def run_root_for(output_root: Path, condition: str, run_index: int) -> Path:
    return output_root / condition / f"run_{run_index:02d}"


def build_command(args: argparse.Namespace, epsilon: float | None, run_index: int) -> List[str]:
    benchmark_script = Path(__file__).with_name("run_sim_gpu_amcl_benchmark.py")
    condition = condition_name(epsilon)
    run_output_root = run_root_for(args.output_root, condition, run_index)

    cmd = [
        sys.executable,
        str(benchmark_script),
        "--output-root", str(run_output_root),
        "--laps", str(args.laps),
        "--max-duration-sec", str(args.max_duration_sec),
        "--process-timeout-sec", str(args.process_timeout_sec),
        "--map-file", str(args.map_file),
        "--trajectory-file", str(args.trajectory_file),
        "--headless",
        "--realistic-plant",
        "--sim-odom-source", args.sim_odom_source,
        "--sim-drive-input-mode", args.sim_drive_input_mode,
        "--sim-drive-uses-acceleration-field",
        "--mpc-raceline-speed-margin", str(args.mpc_raceline_speed_margin),
        "--cloud-publish-rate", str(args.cloud_publish_rate),
        "--amcl-num-particles", str(args.max_particles),
        "--amcl-max-particles", str(args.max_particles),
        "--amcl-max-beams", str(args.max_beams),
        "--amcl-update-min-d", str(args.update_min_d),
        "--amcl-update-min-a", str(args.update_min_a),
        "--amcl-likelihood-scale", str(args.likelihood_scale),
        "--amcl-alpha1", str(args.amcl_alpha1),
        "--amcl-alpha2", str(args.amcl_alpha2),
        "--amcl-alpha3", str(args.amcl_alpha3),
        "--amcl-alpha4", str(args.amcl_alpha4),
        "--amcl-z-hit", str(args.amcl_z_hit),
        "--amcl-z-rand", str(args.amcl_z_rand),
        "--amcl-sigma-hit", str(args.amcl_sigma_hit),
        "--amcl-resample-threshold", str(args.amcl_resample_threshold),
        "--amcl-kld-z", str(args.kld_z),
        "--amcl-kld-bin-x", str(args.kld_bin_x),
        "--amcl-kld-bin-y", str(args.kld_bin_y),
        "--amcl-kld-bin-theta", str(args.kld_bin_theta),
        "--amcl-cluster-publish-min-weight", str(args.cluster_publish_min_weight),
        "--ekf-process-noise-scale", str(args.ekf_process_noise_scale),
        "--no-debug-pre-resample-particles",
    ]

    if args.use_cluster_estimate:
        cmd.append("--amcl-use-cluster-estimate")
    else:
        cmd.append("--no-amcl-use-cluster-estimate")

    if epsilon is None:
        cmd.extend([
            "--amcl-min-particles", str(args.max_particles),
        ])
    else:
        cmd.extend([
            "--amcl-min-particles", str(args.kld_min_particles),
            "--amcl-kld-epsilon", str(epsilon),
            "--amcl-use-kld",
        ])

    for extra in args.extra_launch_arg:
        cmd.extend(["--extra-launch-arg", extra])
    return cmd


def delete_bag(run_root: Path) -> None:
    bag_dir = run_root / "AMCL_benchmark" / "AMCL_benchmark"
    if bag_dir.exists():
        shutil.rmtree(bag_dir)


def run_cases(args: argparse.Namespace, epsilons: Sequence[float]) -> None:
    conditions: List[float | None] = [None, *epsilons]
    for run_index in range(1, args.runs + 1):
        order = list(conditions)
        if args.alternate_order and run_index % 2 == 0:
            order.reverse()
        for epsilon in order:
            condition = condition_name(epsilon)
            cmd = build_command(args, epsilon, run_index)
            print(f"\n=== {condition} run {run_index:02d}/{args.runs} ===")
            print(" ".join(cmd))
            if args.dry_run:
                continue
            result = subprocess.run(cmd, check=False)
            if args.delete_bags:
                delete_bag(run_root_for(args.output_root, condition, run_index))
            if result.returncode != 0:
                raise RuntimeError(
                    f"{condition} run {run_index:02d} failed with code {result.returncode}")


def read_status(path: Path) -> Dict[str, object]:
    if not path.exists():
        return {}
    try:
        with path.open() as handle:
            return json.load(handle)
    except (OSError, json.JSONDecodeError):
        return {}


def collect_results(args: argparse.Namespace, epsilons: Sequence[float]) -> None:
    run_rows: List[Dict[str, object]] = []
    sample_rows: List[Dict[str, object]] = []

    for epsilon in [None, *epsilons]:
        condition = condition_name(epsilon)
        label = condition_label(epsilon)
        aggregate_pipeline: List[Dict[str, float]] = []
        aggregate_errors: List[Dict[str, float]] = []
        aggregate_duration_s = 0.0
        aggregate_status = {"reason": "aggregate", "laps": ""}

        for run_index in range(1, args.runs + 1):
            run_root = run_root_for(args.output_root, condition, run_index)
            pipeline_path = latest_pipeline_csv(run_root)
            if pipeline_path is None:
                print(f"[warn] missing pipeline CSV: {run_root}")
                continue

            pipeline_rows = load_pipeline_csv(pipeline_path, args.skip_first_sec)
            error_path = benchmark_csv(run_root)
            error_rows = load_benchmark_errors(error_path, args.skip_first_sec) if error_path.exists() else []
            status = read_status(status_json(run_root))

            run_rows.append(summarize(
                condition, label, f"run_{run_index:02d}",
                pipeline_rows, error_rows, args.max_particles, status))
            aggregate_pipeline.extend(pipeline_rows)
            aggregate_errors.extend(error_rows)
            duration_s = run_duration(pipeline_rows)
            if math.isfinite(duration_s):
                aggregate_duration_s += duration_s

            for row in pipeline_rows:
                sample_rows.append({
                    "condition": condition,
                    "label": label,
                    "run": f"run_{run_index:02d}",
                    **row,
                })

        run_rows.append(summarize(
            condition, label, "aggregate", aggregate_pipeline, aggregate_errors,
            args.max_particles, aggregate_status, aggregate_duration_s))

    aggregate_rows = [row for row in run_rows if row["run"] == "aggregate"]
    write_csv(args.output_root / "KLD_Epsilon_Run_Summary.csv", run_rows)
    write_csv(args.output_root / "KLD_Epsilon_Aggregate_Summary.csv", aggregate_rows)
    write_csv(args.output_root / "KLD_Epsilon_Latency_Samples.csv", sample_rows)


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    repo_root = script_dir.parents[1]
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    default_output_root = (
        repo_root / "f1tenth_localization" / "Benchmark" / "Matlab" /
        "sim_benchmark" / f"gpu_amcl_kld_epsilon_sweep_{timestamp}"
    )

    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", type=Path, default=default_output_root)
    parser.add_argument("--map-file", type=Path,
                        default=repo_root / "f1tenth_planning" / "maps" / "my_track_map.yaml")
    parser.add_argument("--trajectory-file", type=Path,
                        default=repo_root / "f1tenth_localization" / "Benchmark" /
                        "Matlab" / "sim_benchmark" / "my_track_raceline_vcap_4p0.csv")
    parser.add_argument("--epsilons", default="0.1,0.25,0.35,0.5,0.7")
    parser.add_argument("--runs", type=int, default=5)
    parser.add_argument("--laps", type=int, default=5)
    parser.add_argument("--max-duration-sec", type=float, default=0.0)
    parser.add_argument("--process-timeout-sec", type=float, default=180.0)
    parser.add_argument("--max-particles", type=int, default=2000)
    parser.add_argument("--kld-min-particles", type=int, default=600)
    parser.add_argument("--max-beams", type=int, default=270)
    parser.add_argument("--kld-z", type=float, default=1.96)
    parser.add_argument("--kld-bin-x", type=float, default=0.5)
    parser.add_argument("--kld-bin-y", type=float, default=0.5)
    parser.add_argument("--kld-bin-theta", type=float, default=0.1)
    parser.add_argument("--update-min-d", type=float, default=0.04)
    parser.add_argument("--update-min-a", type=float, default=0.035)
    parser.add_argument("--likelihood-scale", type=float, default=4.0)
    parser.add_argument("--amcl-alpha1", type=float, default=0.1)
    parser.add_argument("--amcl-alpha2", type=float, default=0.2)
    parser.add_argument("--amcl-alpha3", type=float, default=0.2)
    parser.add_argument("--amcl-alpha4", type=float, default=0.25)
    parser.add_argument("--amcl-z-hit", type=float, default=0.90)
    parser.add_argument("--amcl-z-rand", type=float, default=0.10)
    parser.add_argument("--amcl-sigma-hit", type=float, default=0.05)
    parser.add_argument("--amcl-resample-threshold", type=float, default=0.5)
    parser.add_argument("--ekf-process-noise-scale", type=float, default=0.2)
    parser.add_argument("--cloud-publish-rate", type=float, default=0.0)
    parser.add_argument("--cluster-publish-min-weight", type=float, default=0.60)
    parser.add_argument("--use-cluster-estimate", action=argparse.BooleanOptionalAction,
                        default=True)
    parser.add_argument("--sim-odom-source", choices=("vesc", "ground_truth"), default="vesc")
    parser.add_argument("--sim-drive-input-mode", choices=("vesc", "ackermann"), default="vesc")
    parser.add_argument("--mpc-raceline-speed-margin", type=float, default=0.0)
    parser.add_argument("--skip-first-sec", type=float, default=5.0)
    parser.add_argument("--alternate-order", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--delete-bags", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--plot-only", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--extra-launch-arg", action="append", default=[])
    return parser.parse_args(argv)


def main(argv: Sequence[str]) -> int:
    args = parse_args(argv)
    args.output_root = args.output_root.resolve()
    args.map_file = args.map_file.resolve()
    args.trajectory_file = args.trajectory_file.resolve()
    epsilons = parse_epsilons(args.epsilons)

    print(f"Output root: {args.output_root}")
    print(f"Map        : {args.map_file}")
    print(f"Trajectory : {args.trajectory_file}")
    print(f"Conditions : Fixed + eps {', '.join(f'{v:g}' for v in epsilons)}")
    print(f"Runs/laps  : {args.runs} x {args.laps}")
    print(f"Particles  : max={args.max_particles}, kld_min={args.kld_min_particles}")
    print(f"Sim setup  : odom={args.sim_odom_source}, drive={args.sim_drive_input_mode}")

    if not args.plot_only:
        run_cases(args, epsilons)
    if args.dry_run:
        return 0
    collect_results(args, epsilons)

    print(f"Run summary      : {args.output_root / 'KLD_Epsilon_Run_Summary.csv'}")
    print(f"Aggregate summary: {args.output_root / 'KLD_Epsilon_Aggregate_Summary.csv'}")
    print(f"Latency samples  : {args.output_root / 'KLD_Epsilon_Latency_Samples.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
