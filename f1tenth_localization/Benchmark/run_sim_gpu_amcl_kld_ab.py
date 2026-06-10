#!/usr/bin/env python3
"""Run paired GPU AMCL fixed-particle vs KLD-adaptive benchmark.

This wrapper keeps all launch settings identical except the adaptive particle
settings. It then reads the pipeline monitor CSVs and writes comparison figures
and summary CSVs.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import statistics
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Sequence


CONDITIONS = ("fixed", "kld")
CONDITION_LABELS = {
    "fixed": "Fixed particles",
    "kld": "KLD adaptive",
}


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
    weight = pos - lo
    return finite[lo] * (1.0 - weight) + finite[hi] * weight


def mean_or_nan(values: Sequence[float]) -> float:
    finite = [v for v in values if math.isfinite(v)]
    return statistics.fmean(finite) if finite else math.nan


def median_or_nan(values: Sequence[float]) -> float:
    finite = [v for v in values if math.isfinite(v)]
    return statistics.median(finite) if finite else math.nan


def float_field(row: Dict[str, str], name: str) -> float:
    raw = row.get(name, "")
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def latest_pipeline_csv(run_root: Path) -> Path | None:
    matches = sorted(run_root.glob("AMCL_benchmark/pipeline/pipeline_latency_*.csv"))
    if matches:
        return matches[-1]
    matches = sorted(run_root.glob("AMCL_benchmark/pipeline/Pipeline_*.csv"))
    return matches[-1] if matches else None


def benchmark_csv(run_root: Path) -> Path:
    return run_root / "AMCL_benchmark" / "AMCL_benchmark.csv"


def angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def load_pipeline_csv(path: Path, skip_first_sec: float) -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        raw_rows = list(reader)

    if not raw_rows:
        return rows

    first_wall = float_field(raw_rows[0], "wall_time_ns")
    for raw in raw_rows:
        wall_ns = float_field(raw, "wall_time_ns")
        if math.isfinite(wall_ns) and math.isfinite(first_wall):
            t_rel = (wall_ns - first_wall) * 1e-9
            if t_rel < skip_first_sec:
                continue
        else:
            t_rel = math.nan

        scan_to_amcl = float_field(raw, "scan_to_amcl_ms")
        particle_count = float_field(raw, "amcl_particle_count")
        if not math.isfinite(scan_to_amcl):
            continue
        rows.append({
            "t_rel_s": t_rel,
            "scan_to_amcl_ms": scan_to_amcl,
            "amcl_particle_count": particle_count,
        })
    return rows


def load_benchmark_errors(path: Path, skip_first_sec: float) -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        raw_rows = list(reader)

    if not raw_rows:
        return rows

    first_wall = float_field(raw_rows[0], "wall_time_ns")
    for raw in raw_rows:
        wall_ns = float_field(raw, "wall_time_ns")
        if math.isfinite(wall_ns) and math.isfinite(first_wall):
            t_rel = (wall_ns - first_wall) * 1e-9
            if t_rel < skip_first_sec:
                continue
        else:
            t_rel = math.nan

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
        })
    return rows


def summarize_rows(condition: str,
                   run_name: str,
                   rows: Sequence[Dict[str, float]],
                   error_rows: Sequence[Dict[str, float]],
                   fixed_particles: int) -> Dict[str, object]:
    latency = [r["scan_to_amcl_ms"] for r in rows]
    particles = [
        r["amcl_particle_count"]
        for r in rows
        if math.isfinite(r["amcl_particle_count"])
    ]
    xy_errors = [r["amcl_error_xy_m"] for r in error_rows]
    yaw_errors = [r["amcl_abs_yaw_error_rad"] for r in error_rows]
    max_fraction = math.nan
    if particles:
        at_max = sum(1 for p in particles if p >= 0.95 * fixed_particles)
        max_fraction = at_max / len(particles)
    return {
        "condition": condition,
        "label": CONDITION_LABELS[condition],
        "run": run_name,
        "n_samples": len(latency),
        "n_error_samples": len(xy_errors),
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
        "fraction_at_or_near_max": max_fraction,
    }


def write_csv(path: Path, rows: Sequence[Dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    fieldnames = list(rows[0].keys())
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def condition_run_dir(output_root: Path, condition: str, run_index: int) -> Path:
    return output_root / condition / f"run_{run_index:02d}"


def build_command(args: argparse.Namespace,
                  condition: str,
                  run_index: int) -> List[str]:
    benchmark_script = Path(__file__).with_name("run_sim_gpu_amcl_benchmark.py")
    run_output_root = condition_run_dir(args.output_root, condition, run_index)

    cmd = [
        sys.executable,
        str(benchmark_script),
        "--output-root", str(run_output_root),
        "--laps", str(args.laps),
        "--max-duration-sec", str(args.max_duration_sec),
        "--process-timeout-sec", str(args.process_timeout_sec),
        "--headless",
        "--cloud-publish-rate", str(args.cloud_publish_rate),
        "--amcl-num-particles", str(args.fixed_particles),
        "--amcl-max-particles", str(args.fixed_particles),
        "--amcl-max-beams", str(args.max_beams),
        "--amcl-update-min-d", str(args.update_min_d),
        "--amcl-update-min-a", str(args.update_min_a),
        "--amcl-likelihood-scale", str(args.likelihood_scale),
        "--amcl-kld-epsilon", str(args.kld_epsilon),
        "--amcl-kld-z", str(args.kld_z),
        "--amcl-kld-bin-x", str(args.kld_bin_x),
        "--amcl-kld-bin-y", str(args.kld_bin_y),
        "--amcl-kld-bin-theta", str(args.kld_bin_theta),
        "--amcl-cluster-publish-min-weight", str(args.cluster_publish_min_weight),
        "--no-debug-pre-resample-particles",
        "--no-amcl-use-cluster-estimate" if args.disable_cluster_estimate else "--amcl-use-cluster-estimate",
    ]

    if args.global_localization:
        cmd.append("--global-localization")
    else:
        cmd.append("--no-global-localization")

    if condition == "fixed":
        cmd.extend([
            "--amcl-min-particles", str(args.fixed_particles),
        ])
    elif condition == "kld":
        cmd.extend([
            "--amcl-min-particles", str(args.kld_min_particles),
            "--amcl-use-kld",
        ])
    else:
        raise ValueError(f"Unknown condition: {condition}")

    for extra in args.extra_launch_arg:
        cmd.extend(["--extra-launch-arg", extra])
    return cmd


def run_cases(args: argparse.Namespace) -> None:
    for run_index in range(1, args.runs + 1):
        order = list(CONDITIONS)
        if args.alternate_order and run_index % 2 == 0:
            order.reverse()
        for condition in order:
            cmd = build_command(args, condition, run_index)
            print(f"\n=== {condition} run {run_index:02d}/{args.runs} ===")
            print(" ".join(cmd))
            if args.dry_run:
                continue
            result = subprocess.run(cmd, check=False)
            if result.returncode != 0:
                raise RuntimeError(
                    f"{condition} run {run_index:02d} failed with code {result.returncode}")


def collect_results(args: argparse.Namespace) -> tuple[
    List[Dict[str, object]],
    Dict[str, List[Dict[str, float]]],
    Dict[str, List[Dict[str, float]]],
]:
    run_summaries: List[Dict[str, object]] = []
    all_rows: Dict[str, List[Dict[str, float]]] = {condition: [] for condition in CONDITIONS}
    all_error_rows: Dict[str, List[Dict[str, float]]] = {condition: [] for condition in CONDITIONS}

    for condition in CONDITIONS:
        for run_index in range(1, args.runs + 1):
            run_root = condition_run_dir(args.output_root, condition, run_index)
            csv_path = latest_pipeline_csv(run_root)
            if csv_path is None:
                print(f"[warn] Missing pipeline CSV: {run_root}")
                continue
            rows = load_pipeline_csv(csv_path, args.skip_first_sec)
            error_path = benchmark_csv(run_root)
            error_rows: List[Dict[str, float]] = []
            if error_path.exists():
                error_rows = load_benchmark_errors(error_path, args.skip_first_sec)
            else:
                print(f"[warn] Missing benchmark CSV: {error_path}")
            all_rows[condition].extend(rows)
            all_error_rows[condition].extend(error_rows)
            run_summaries.append(summarize_rows(
                condition, f"run_{run_index:02d}", rows, error_rows, args.fixed_particles))

    aggregate_rows = [
        summarize_rows(
            condition,
            "aggregate",
            all_rows[condition],
            all_error_rows[condition],
            args.fixed_particles)
        for condition in CONDITIONS
    ]
    write_csv(args.output_root / "KLD_AB_Run_Summary.csv", run_summaries)
    write_csv(args.output_root / "KLD_AB_Aggregate_Summary.csv", aggregate_rows)
    missing_conditions = [
        CONDITION_LABELS[condition]
        for condition in CONDITIONS
        if not all_rows[condition]
    ]
    if missing_conditions:
        missing = ", ".join(missing_conditions)
        raise RuntimeError(
            "No scan-to-AMCL samples found for: "
            f"{missing}. The run produced pipeline CSVs without data. "
            "Check that /amcl_pose and /amcl_particle_count are published.")
    return aggregate_rows, all_rows, all_error_rows


def finite_values(rows: Iterable[Dict[str, float]], key: str) -> List[float]:
    return [r[key] for r in rows if math.isfinite(r[key])]


def save_plots(args: argparse.Namespace,
               aggregate_rows: Sequence[Dict[str, object]],
               all_rows: Dict[str, List[Dict[str, float]]],
               all_error_rows: Dict[str, List[Dict[str, float]]]) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[warn] matplotlib missing; CSV summaries written, plots skipped")
        return

    plots_dir = args.figure_output_dir
    plots_dir.mkdir(parents=True, exist_ok=True)

    labels = [CONDITION_LABELS[c] for c in CONDITIONS]
    latency_data = [finite_values(all_rows[c], "scan_to_amcl_ms") for c in CONDITIONS]

    fig, ax = plt.subplots(figsize=(6.4, 4.2))
    ax.boxplot(latency_data, labels=labels, showfliers=False)
    ax.set_ylabel("scan-to-AMCL latency [ms]")
    ax.set_title("Fixed particles vs KLD adaptive")
    ax.grid(True, axis="y", alpha=0.35)
    fig.tight_layout()
    fig.savefig(plots_dir / "KLD_AB_ScanToAmcl_Boxplot.png", dpi=220)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(6.6, 4.4))
    colors = {"fixed": "#1f77b4", "kld": "#ff7f0e"}
    for condition in CONDITIONS:
        particles = finite_values(all_rows[condition], "amcl_particle_count")
        latency = [
            r["scan_to_amcl_ms"]
            for r in all_rows[condition]
            if math.isfinite(r["scan_to_amcl_ms"]) and math.isfinite(r["amcl_particle_count"])
        ]
        particle_for_latency = [
            r["amcl_particle_count"]
            for r in all_rows[condition]
            if math.isfinite(r["scan_to_amcl_ms"]) and math.isfinite(r["amcl_particle_count"])
        ]
        if not particles:
            continue
        ax.scatter(particle_for_latency, latency, s=7, alpha=0.25,
                   color=colors[condition], label=CONDITION_LABELS[condition])
    ax.set_xlabel("active particles")
    ax.set_ylabel("scan-to-AMCL latency [ms]")
    ax.set_title("Latency vs active particle count")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(plots_dir / "KLD_AB_Latency_vs_Particles.png", dpi=220)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9.2, 2.8))
    table_rows = []
    for row in aggregate_rows:
        table_rows.append([
            row["label"],
            f'{row["scan_to_amcl_median_ms"]:.3f}',
            f'{row["scan_to_amcl_p95_ms"]:.3f}',
            f'{100.0 * row["amcl_position_error_median_m"]:.2f}',
            f'{100.0 * row["amcl_position_error_p95_m"]:.2f}',
            f'{math.degrees(row["amcl_yaw_error_median_rad"]):.2f}',
            f'{math.degrees(row["amcl_yaw_error_p95_rad"]):.2f}',
        ])
    ax.axis("off")
    table = ax.table(
        cellText=table_rows,
        colLabels=[
            "case",
            "median ms",
            "p95 ms",
            "median pos cm",
            "p95 pos cm",
            "median yaw deg",
            "p95 yaw deg",
        ],
        loc="center",
    )
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1, 1.25)
    ax.set_title("KLD A/B aggregate summary")
    fig.tight_layout()
    fig.savefig(plots_dir / "KLD_AB_Summary_Table.png", dpi=220)
    plt.close(fig)


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    repo_root = script_dir.parents[1]
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    default_output_root = (
        repo_root / "f1tenth_localization" / "Benchmark" / "Matlab" /
        "sim_benchmark" / f"gpu_amcl_kld_ab_{timestamp}"
    )

    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", type=Path, default=default_output_root)
    parser.add_argument(
        "--figure-output-dir",
        type=Path,
        default=repo_root / "ReportMaterial" / "TestFigures",
        help="Directory for generated figure PNGs.")
    parser.add_argument("--runs", type=int, default=5)
    parser.add_argument("--laps", type=int, default=3)
    parser.add_argument("--max-duration-sec", type=float, default=0.0)
    parser.add_argument("--process-timeout-sec", type=float, default=0.0)
    parser.add_argument("--fixed-particles", type=int, default=2000)
    parser.add_argument("--kld-min-particles", type=int, default=600)
    parser.add_argument("--max-beams", type=int, default=270)
    parser.add_argument("--kld-epsilon", type=float, default=0.035)
    parser.add_argument("--kld-z", type=float, default=1.96)
    parser.add_argument("--kld-bin-x", type=float, default=0.5)
    parser.add_argument("--kld-bin-y", type=float, default=0.5)
    parser.add_argument("--kld-bin-theta", type=float, default=0.1)
    parser.add_argument("--update-min-d", type=float, default=0.04)
    parser.add_argument("--update-min-a", type=float, default=0.035)
    parser.add_argument("--likelihood-scale", type=float, default=4.0)
    parser.add_argument("--cloud-publish-rate", type=float, default=0.0)
    parser.add_argument("--cluster-publish-min-weight", type=float, default=0.0,
                        help="Forwarded to AMCL; 0.0 avoids hiding valid timing samples.")
    parser.add_argument("--skip-first-sec", type=float, default=5.0)
    parser.add_argument("--disable-cluster-estimate", action="store_true",
                        help="Disable cluster estimate to isolate KLD overhead more strongly.")
    parser.add_argument("--global-localization", action=argparse.BooleanOptionalAction,
                        default=False,
                        help="Use global initialization; default is known initial pose.")
    parser.add_argument("--alternate-order", action=argparse.BooleanOptionalAction,
                        default=True)
    parser.add_argument("--plot-only", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--extra-launch-arg", action="append", default=[],
                        help="Extra launch arg forwarded to run_sim_gpu_amcl_benchmark.py.")
    return parser.parse_args(argv)


def main(argv: Sequence[str]) -> int:
    args = parse_args(argv)
    args.output_root = args.output_root.resolve()
    args.figure_output_dir = args.figure_output_dir.resolve()
    print(f"Output root: {args.output_root}")
    print(f"Figures    : {args.figure_output_dir}")
    print(f"Runs       : {args.runs}")
    print(f"KLD params : min={args.kld_min_particles}, eps={args.kld_epsilon}, z={args.kld_z}")

    if not args.plot_only:
        run_cases(args)
    if args.dry_run:
        return 0

    aggregate_rows, all_rows, all_error_rows = collect_results(args)
    save_plots(args, aggregate_rows, all_rows, all_error_rows)

    print(f"Run summary      : {args.output_root / 'KLD_AB_Run_Summary.csv'}")
    print(f"Aggregate summary: {args.output_root / 'KLD_AB_Aggregate_Summary.csv'}")
    print(f"Figures          : {args.figure_output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
