#!/usr/bin/env python3
"""Create report figures for GPU AMCL global-initialization spawn tests."""

from __future__ import annotations

import argparse
import csv
import math
import os
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image


XY_THRESHOLD_M = 0.20
YAW_THRESHOLD_RAD = math.radians(5.0)
STABLE_WINDOW_S = 1.0
WINDOW_TOLERANCE_S = 0.01


def finite_float(value: object) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return math.nan
    return out if math.isfinite(out) else math.nan


def angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def median(values: Sequence[float]) -> float:
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return math.nan
    mid = len(values) // 2
    if len(values) % 2:
        return values[mid]
    return 0.5 * (values[mid - 1] + values[mid])


def percentile(values: Sequence[float], pct: float) -> float:
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return math.nan
    idx = int(round((pct / 100.0) * (len(values) - 1)))
    return values[max(0, min(idx, len(values) - 1))]


def load_csv(path: Path) -> List[Dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: Sequence[Dict[str, object]]) -> None:
    keys: List[str] = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def load_raceline(path: Path) -> List[Dict[str, float]]:
    points: List[Dict[str, float]] = []
    with path.open(newline="") as handle:
        for raw in handle:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) < 4:
                continue
            try:
                points.append({
                    "s": float(parts[0]),
                    "x": float(parts[1]),
                    "y": float(parts[2]),
                    "yaw": float(parts[3]),
                })
            except ValueError:
                continue
    if not points:
        raise RuntimeError(f"No raceline points in {path}")
    return points


def parse_map_yaml(path: Path) -> Tuple[Path, float, Tuple[float, float, float]]:
    image_path: Optional[Path] = None
    resolution = math.nan
    origin = (math.nan, math.nan, 0.0)
    with path.open() as handle:
        for raw in handle:
            line = raw.strip()
            if not line or line.startswith("#") or ":" not in line:
                continue
            key, value = line.split(":", 1)
            key = key.strip()
            value = value.strip()
            if key == "image":
                image_path = (path.parent / value).resolve()
            elif key == "resolution":
                resolution = float(value)
            elif key == "origin":
                nums = value.strip("[]").split(",")
                origin = tuple(float(v.strip()) for v in nums[:3])  # type: ignore[assignment]
    if image_path is None or not math.isfinite(resolution):
        raise RuntimeError(f"Invalid map yaml: {path}")
    return image_path, resolution, origin


def load_map_image(map_yaml: Path) -> Tuple[np.ndarray, Tuple[float, float, float, float]]:
    image_path, resolution, origin = parse_map_yaml(map_yaml)
    img = np.array(Image.open(image_path).convert("L"))
    img = np.flipud(img)
    height, width = img.shape
    extent = (
        origin[0],
        origin[0] + width * resolution,
        origin[1],
        origin[1] + height * resolution,
    )
    return img, extent


def read_spawn_samples(csv_path: Path) -> List[Tuple[float, float, bool, float, float]]:
    samples: List[Tuple[float, float, bool, float, float]] = []
    with csv_path.open(newline="") as handle:
        for row in csv.DictReader(handle):
            t = finite_float(row.get("wall_time_ns")) / 1e9
            progress = finite_float(row.get("progress_s"))
            gt_x = finite_float(row.get("gt_x"))
            gt_y = finite_float(row.get("gt_y"))
            gt_yaw = finite_float(row.get("gt_yaw"))
            amcl_x = finite_float(row.get("amcl_x"))
            amcl_y = finite_float(row.get("amcl_y"))
            amcl_yaw = finite_float(row.get("amcl_yaw"))
            valid = all(math.isfinite(v) for v in (t, gt_x, gt_y, gt_yaw, amcl_x, amcl_y, amcl_yaw))
            err_xy = math.hypot(amcl_x - gt_x, amcl_y - gt_y) if valid else math.nan
            err_yaw = abs(angle_diff(amcl_yaw, gt_yaw)) if valid else math.nan
            samples.append((t, progress, valid, err_xy, err_yaw))
    return samples


def distance_delta(progress: float, start_progress: float, track_length: float) -> float:
    if not (math.isfinite(progress) and math.isfinite(start_progress)):
        return math.nan
    delta = progress - start_progress
    if delta < 0.0:
        delta += track_length
    return delta


def first_converged_index(
    samples: Sequence[Tuple[float, float, bool, float, float]],
    xy_threshold_m: float,
    yaw_threshold_rad: float,
    stable_window_s: float,
    tolerance_s: float,
) -> Optional[int]:
    for i, sample in enumerate(samples):
        start_t = sample[0]
        window = [s for s in samples[i:] if s[0] <= start_t + stable_window_s]
        if not window or window[-1][0] < start_t + stable_window_s - tolerance_s:
            continue
        ok = [
            s[2] and s[3] < xy_threshold_m and s[4] < yaw_threshold_rad
            for s in window
        ]
        if ok and all(ok):
            return i
    return None


def summarize_run(
    root: Path,
    xy_threshold_m: float,
    yaw_threshold_rad: float,
    stable_window_s: float,
    tolerance_s: float,
    track_length_m: float,
) -> List[Dict[str, object]]:
    results_path = root / "spawn_results.csv"
    rows = load_csv(results_path)
    out_rows: List[Dict[str, object]] = []

    for row in rows:
        samples = read_spawn_samples(Path(row["csv_path"]))
        t0 = samples[0][0] if samples else math.nan
        p0 = samples[0][1] if samples else math.nan
        first_idx = next((i for i, sample in enumerate(samples) if sample[2]), None)
        conv_idx = first_converged_index(
            samples, xy_threshold_m, yaw_threshold_rad, stable_window_s, tolerance_s)

        time_to_first = samples[first_idx][0] - t0 if first_idx is not None else math.nan
        time_to_conv = samples[conv_idx][0] - t0 if conv_idx is not None else math.nan
        dist_to_conv = (
            distance_delta(samples[conv_idx][1], p0, track_length_m)
            if conv_idx is not None else math.nan
        )

        merged: Dict[str, object] = dict(row)
        merged.update({
            "time_to_first_pose_s": time_to_first,
            "time_to_converged_s": time_to_conv,
            "distance_to_converged_m": dist_to_conv,
            "converged_stable_1s": conv_idx is not None,
            "xy_threshold_m": xy_threshold_m,
            "yaw_threshold_rad": yaw_threshold_rad,
            "stable_window_s": stable_window_s,
            "window_duration_tolerance_s": tolerance_s,
        })
        out_rows.append(merged)

    write_csv(root / "convergence_summary.csv", out_rows)
    return out_rows


def set_report_style() -> None:
    plt.rcParams.update({
        "figure.dpi": 140,
        "savefig.dpi": 300,
        "font.size": 10,
        "axes.titlesize": 12,
        "axes.labelsize": 10,
        "axes.grid": True,
        "grid.alpha": 0.22,
        "grid.linewidth": 0.6,
        "legend.frameon": False,
        "axes.spines.top": False,
        "axes.spines.right": False,
    })


def save_figure(fig: plt.Figure, output_dir: Path, name: str) -> None:
    fig.tight_layout()
    fig.savefig(output_dir / f"{name}.png", bbox_inches="tight")
    fig.savefig(output_dir / f"{name}.pdf", bbox_inches="tight")
    plt.close(fig)


def bool_value(value: object) -> bool:
    return str(value).strip().lower() in {"true", "1", "yes"}


def float_column(rows: Sequence[Dict[str, object]], key: str) -> np.ndarray:
    return np.array([finite_float(row.get(key)) for row in rows], dtype=float)


def plot_success_map(
    rows: Sequence[Dict[str, object]],
    raceline: Sequence[Dict[str, float]],
    map_yaml: Path,
    output_dir: Path,
) -> None:
    img, extent = load_map_image(map_yaml)
    x = float_column(rows, "x")
    y = float_column(rows, "y")
    conv_t = float_column(rows, "time_to_converged_s")
    success = np.array([bool_value(row.get("converged_stable_1s")) for row in rows])
    tail_success = np.array([bool_value(row.get("success")) for row in rows])
    crash_fail = np.array([
        (not ok) and str(row.get("status_reason", "")).strip().lower() == "collision"
        for ok, row in zip(success, rows)
    ])
    other_fail = (~success) & (~crash_fail)

    fig, ax = plt.subplots(figsize=(6.2, 7.0))
    ax.imshow(img, cmap="gray", extent=extent, origin="lower", alpha=0.72)
    ax.plot([p["x"] for p in raceline], [p["y"] for p in raceline],
            color="#111827", linewidth=1.2, label="raceline")

    if np.any(success):
        sizes = 18 + 76 * np.nan_to_num(conv_t[success], nan=0.0) / max(1.0, np.nanmax(conv_t[success]))
        sc = ax.scatter(x[success], y[success], c=conv_t[success], s=sizes,
                        cmap="viridis", edgecolors="white", linewidths=0.6,
                        label="stable convergence")
        cbar = fig.colorbar(sc, ax=ax, shrink=0.82, pad=0.02)
        cbar.set_label("time to converged [s]")

    if np.any(crash_fail):
        ax.scatter(x[crash_fail], y[crash_fail], marker="X", s=58, c="#d62728",
                   edgecolors="white", linewidths=0.8, label="crash before convergence")
    if np.any(other_fail):
        ax.scatter(x[other_fail], y[other_fail], marker="o", s=44, c="none",
                   edgecolors="#d62728", linewidths=1.2, label="no stable convergence")

    ax.set_title(f"Global Initialization Results ({len(rows)} Spawn Poses)")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.legend(loc="lower center", bbox_to_anchor=(0.5, -0.16), ncol=2)
    save_figure(fig, output_dir, "test2_spawn_success_map")


def plot_cdf(rows: Sequence[Dict[str, object]], output_dir: Path) -> None:
    conv_t = sorted(v for v in float_column(rows, "time_to_converged_s") if math.isfinite(v))
    first_t = sorted(v for v in float_column(rows, "time_to_first_pose_s") if math.isfinite(v))
    n = len(rows)

    fig, ax = plt.subplots(figsize=(6.6, 4.2))
    if conv_t:
        y = np.arange(1, len(conv_t) + 1) / n
        ax.step(conv_t, y, where="post", color="#1f77b4", linewidth=2.0,
                label=f"converged ({len(conv_t)}/{n})")
    if first_t:
        y_first = np.arange(1, len(first_t) + 1) / n
        ax.step(first_t, y_first, where="post", color="#2ca02c", linewidth=1.8,
                label=f"first pose ({len(first_t)}/{n})")
    ax.set_title("Global Initialization Timing CDF")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("fraction of all spawn poses")
    ax.set_ylim(0.0, 1.03)
    ax.legend(loc="lower right")
    save_figure(fig, output_dir, "test2_convergence_cdf")


def plot_error_by_progress(rows: Sequence[Dict[str, object]], output_dir: Path) -> None:
    s = float_column(rows, "s_m")
    med_xy = float_column(rows, "amcl_tail_median_err_xy_m")
    p95_xy = float_column(rows, "amcl_tail_p95_err_xy_m")
    conv_t = float_column(rows, "time_to_converged_s")
    tail_success = np.array([bool_value(row.get("success")) for row in rows])

    fig, axes = plt.subplots(2, 1, figsize=(7.2, 5.8), sharex=True)
    axes[0].plot(s, med_xy, color="#1f77b4", linewidth=1.7, label="tail median")
    axes[0].plot(s, p95_xy, color="#ff7f0e", linewidth=1.2, label="tail p95")
    axes[0].axhline(XY_THRESHOLD_M, color="#d62728", linestyle="--", linewidth=1.1,
                    label="0.20 m threshold")
    axes[0].scatter(s[~tail_success], med_xy[~tail_success], marker="X",
                    c="#d62728", s=54, zorder=5, label="tail fail")
    axes[0].set_ylabel("AMCL position error [m]")
    axes[0].set_title("Post-Convergence Error By Spawn Location")
    axes[0].legend(loc="upper right", ncol=2)

    finite_conv = np.isfinite(conv_t)
    axes[1].scatter(s[finite_conv], conv_t[finite_conv],
                    c=conv_t[np.isfinite(conv_t)], cmap="viridis",
                    edgecolors="white", linewidths=0.4, s=34)
    crash_fail = np.array([
        (not bool_value(row.get("converged_stable_1s"))) and
        str(row.get("status_reason", "")).strip().lower() == "collision"
        for row in rows
    ])
    other_fail = (~finite_conv) & (~crash_fail)
    axes[1].scatter(s[crash_fail], np.zeros(np.count_nonzero(crash_fail)),
                    marker="X", c="#d62728", s=42, label="crash before convergence")
    if np.any(other_fail):
        axes[1].scatter(s[other_fail], np.zeros(np.count_nonzero(other_fail)),
                        marker="o", facecolors="none", edgecolors="#d62728",
                        linewidths=1.0, s=34, label="no stable convergence")
    if np.any(finite_conv):
        axes[1].set_ylim(-0.08, np.nanmax(conv_t[finite_conv]) * 1.15)
    axes[1].set_xlabel("raceline distance s [m]")
    axes[1].set_ylabel("time to converged [s]")
    axes[1].legend(loc="upper right")
    save_figure(fig, output_dir, "test2_error_by_spawn_progress")


def representative_indices(rows: Sequence[Dict[str, object]]) -> List[int]:
    conv_pairs = [
        (idx, finite_float(row.get("time_to_converged_s")))
        for idx, row in enumerate(rows)
        if math.isfinite(finite_float(row.get("time_to_converged_s")))
    ]
    selected: List[int] = []
    if conv_pairs:
        ordered = sorted(conv_pairs, key=lambda p: p[1])
        selected.extend([ordered[0][0], ordered[len(ordered) // 2][0], ordered[-1][0]])
    fail_idx = next((
        idx for idx, row in enumerate(rows)
        if not bool_value(row.get("converged_stable_1s"))
    ), None)
    if fail_idx is not None:
        selected.append(fail_idx)
    unique: List[int] = []
    for idx in selected:
        if idx not in unique:
            unique.append(idx)
    return unique[:4]


def plot_timeline(rows: Sequence[Dict[str, object]], output_dir: Path) -> None:
    indices = representative_indices(rows)
    if not indices:
        return

    fig, ax = plt.subplots(figsize=(7.2, 4.4))
    colors = ["#1f77b4", "#2ca02c", "#9467bd", "#d62728"]
    for color, idx in zip(colors, indices):
        row = rows[idx]
        samples = read_spawn_samples(Path(str(row["csv_path"])))
        if not samples:
            continue
        t0 = samples[0][0]
        t = np.array([s[0] - t0 for s in samples], dtype=float)
        err = np.array([s[3] for s in samples], dtype=float)
        label = f"spawn {row['spawn_index']}"
        ax.plot(t, err, color=color, linewidth=1.35, label=label)
        first = finite_float(row.get("time_to_first_pose_s"))
        conv = finite_float(row.get("time_to_converged_s"))
        if math.isfinite(first):
            ax.axvline(first, color=color, linestyle=":", linewidth=0.9, alpha=0.75)
        if math.isfinite(conv):
            ax.axvline(conv, color=color, linestyle="--", linewidth=1.0, alpha=0.85)

    ax.axhline(XY_THRESHOLD_M, color="#111827", linestyle="--", linewidth=1.0,
               label="0.20 m threshold")
    ax.set_title("Representative Global Initialization Timelines")
    ax.set_xlabel("time from run start [s]")
    ax.set_ylabel("AMCL position error [m]")
    ax.set_ylim(bottom=0.0)
    ax.legend(loc="upper right", ncol=2)
    save_figure(fig, output_dir, "test2_representative_timelines")


def plot_summary_bars(rows: Sequence[Dict[str, object]], output_dir: Path) -> None:
    n = len(rows)
    tail_pass = sum(bool_value(row.get("success")) for row in rows)
    stable_pass = sum(bool_value(row.get("converged_stable_1s")) for row in rows)
    first_pose = sum(math.isfinite(finite_float(row.get("time_to_first_pose_s"))) for row in rows)

    fig, ax = plt.subplots(figsize=(5.8, 3.8))
    labels = ["first pose", "stable converged", "tail pass"]
    values = [first_pose / n, stable_pass / n, tail_pass / n]
    counts = [first_pose, stable_pass, tail_pass]
    bars = ax.bar(labels, values, color=["#2ca02c", "#1f77b4", "#6b7280"], width=0.62)
    for bar, count in zip(bars, counts):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.025,
                f"{count}/{n}", ha="center", va="bottom", fontweight="bold")
    ax.set_ylim(0.0, 1.12)
    ax.set_ylabel("fraction of spawn poses")
    ax.set_title("Global Initialization Success Summary")
    save_figure(fig, output_dir, "test2_success_summary")


def write_text_summary(rows: Sequence[Dict[str, object]], output_dir: Path) -> None:
    n = len(rows)
    tail_pass = sum(bool_value(row.get("success")) for row in rows)
    stable_pass = sum(bool_value(row.get("converged_stable_1s")) for row in rows)
    first_times = [finite_float(row.get("time_to_first_pose_s")) for row in rows]
    conv_times = [finite_float(row.get("time_to_converged_s")) for row in rows]
    conv_dist = [finite_float(row.get("distance_to_converged_m")) for row in rows]
    med_xy = [finite_float(row.get("amcl_tail_median_err_xy_m")) for row in rows]
    p95_xy = [finite_float(row.get("amcl_tail_p95_err_xy_m")) for row in rows]

    lines = [
        "GPU AMCL global initialization spawn test",
        f"spawn poses: {n}",
        f"tail-median pass: {tail_pass}/{n}",
        f"stable 1 s convergence: {stable_pass}/{n}",
        f"time to first pose median/p95/max [s]: {median(first_times):.3f} / {percentile(first_times, 95):.3f} / {max(v for v in first_times if math.isfinite(v)):.3f}",
        f"time to converged median/p95/max [s]: {median(conv_times):.3f} / {percentile(conv_times, 95):.3f} / {max(v for v in conv_times if math.isfinite(v)):.3f}",
        f"distance to converged median/p95/max [m]: {median(conv_dist):.3f} / {percentile(conv_dist, 95):.3f} / {max(v for v in conv_dist if math.isfinite(v)):.3f}",
        f"tail median xy error median/p95 [m]: {median(med_xy):.3f} / {percentile(med_xy, 95):.3f}",
        f"tail p95 xy error median/p95 [m]: {median(p95_xy):.3f} / {percentile(p95_xy, 95):.3f}",
        "",
        "Non-converged or tail-fail spawns:",
    ]
    for row in rows:
        if not bool_value(row.get("converged_stable_1s")) or not bool_value(row.get("success")):
            lines.append(
                f"spawn {row.get('spawn_index')}: s={finite_float(row.get('s_m')):.2f} m, "
                f"stable={row.get('converged_stable_1s')}, tail_success={row.get('success')}, "
                f"tail_med_xy={finite_float(row.get('amcl_tail_median_err_xy_m')):.3f} m"
            )
    (output_dir / "test2_summary.txt").write_text("\n".join(lines) + "\n")


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[2]
    default_root = (
        repo_root / "f1tenth_localization" / "Benchmark" / "Matlab" /
        "sim_benchmark" / "gpu_amcl_spawn_test_test2_20260519_150_centerline_cloud_off"
    )
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-root", type=Path, default=default_root)
    parser.add_argument("--map-yaml", type=Path,
                        default=repo_root / "f1tenth_planning" / "maps" / "my_track_map.yaml")
    parser.add_argument("--raceline", type=Path,
                        default=repo_root / "f1tenth_planning" / "trajectories" / "my_track_raceline.csv")
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--xy-threshold-m", type=float, default=XY_THRESHOLD_M)
    parser.add_argument("--yaw-threshold-deg", type=float, default=5.0)
    parser.add_argument("--stable-window-s", type=float, default=STABLE_WINDOW_S)
    parser.add_argument("--window-tolerance-s", type=float, default=WINDOW_TOLERANCE_S)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output_dir = args.output_dir or (args.run_root / "report_figures")
    output_dir.mkdir(parents=True, exist_ok=True)

    set_report_style()
    raceline = load_raceline(args.raceline)
    track_length = max(point["s"] for point in raceline)
    rows = summarize_run(
        root=args.run_root,
        xy_threshold_m=args.xy_threshold_m,
        yaw_threshold_rad=math.radians(args.yaw_threshold_deg),
        stable_window_s=args.stable_window_s,
        tolerance_s=args.window_tolerance_s,
        track_length_m=track_length,
    )

    plot_success_map(rows, raceline, args.map_yaml, output_dir)
    plot_cdf(rows, output_dir)
    plot_error_by_progress(rows, output_dir)
    plot_timeline(rows, output_dir)
    plot_summary_bars(rows, output_dir)
    write_text_summary(rows, output_dir)

    print(f"Wrote convergence summary: {args.run_root / 'convergence_summary.csv'}")
    print(f"Wrote figures: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
