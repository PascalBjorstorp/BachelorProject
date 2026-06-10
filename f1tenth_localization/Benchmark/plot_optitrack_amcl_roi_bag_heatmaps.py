#!/usr/bin/env python3
"""Plot one OptiTrack AMCL ROI heat map per real benchmark bag.

The input samples are produced by analyze_real_vs_sim_amcl_equivalence.py.
Each particle count corresponds to one real OptiTrack AMCL bag in the
benchmark data set.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Dict, Sequence, Tuple

import numpy as np
import pandas as pd

from analyze_real_vs_sim_amcl_equivalence import (
    filter_optitrack,
    first_available_tf,
    interp_yaw,
    lap_segments,
    qmul,
    quat_to_rot,
    read_map_world_tf,
    read_pose_series,
    roi_mask,
    start_calibrate,
    wrap_angle,
    yaw_from_quat,
)

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError as exc:  # pragma: no cover
    raise SystemExit("matplotlib is required for the heat-map plots") from exc


def finite_roi_max_y(values: np.ndarray, roi_max_y: float, padding: float) -> float:
    if math.isfinite(roi_max_y):
        return roi_max_y
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return 1.0
    return float(np.max(finite) + padding)


def discover_bag_names(real_root: Path) -> Dict[int, str]:
    bag_names: Dict[int, str] = {}
    for particle_dir in sorted(real_root.glob("ParticleCount*")):
        if not particle_dir.is_dir():
            continue
        try:
            particles = int(particle_dir.name.removeprefix("ParticleCount"))
        except ValueError:
            continue
        bags = sorted(p for p in particle_dir.glob("OptitrackBenchmark_*") if p.is_dir())
        if bags:
            bag_names[particles] = bags[0].name
    return bag_names


def parse_simple_map_yaml(path: Path) -> Tuple[Path, float, Tuple[float, float, float]]:
    image_path: Path | None = None
    resolution: float | None = None
    origin: Tuple[float, float, float] | None = None
    for raw_line in path.read_text().splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = [part.strip() for part in line.split(":", 1)]
        if key == "image":
            image_path = (path.parent / value).resolve()
        elif key == "resolution":
            resolution = float(value)
        elif key == "origin":
            origin_values = value.strip("[]").split(",")
            origin = tuple(float(v.strip()) for v in origin_values)  # type: ignore[assignment]
    if image_path is None or resolution is None or origin is None:
        raise ValueError(f"Could not parse map yaml: {path}")
    return image_path, resolution, origin


def load_map_background(map_yaml: Path | None) -> Tuple[np.ndarray, Tuple[float, float, float, float]] | None:
    if map_yaml is None or not map_yaml.exists():
        return None
    image_path, resolution, origin = parse_simple_map_yaml(map_yaml)
    image = plt.imread(image_path)
    if image.ndim == 3:
        image = image[:, :, 0]
    image = np.flipud(image)
    height, width = image.shape[:2]
    extent = (
        origin[0],
        origin[0] + width * resolution,
        origin[1],
        origin[1] + height * resolution,
    )
    return image, extent


def load_real_bag_roi_samples(
    real_root: Path,
    roi: Tuple[float, float, float, float],
    min_speed_mps: float,
    pose_topic: str,
) -> pd.DataFrame:
    fallback_tf = first_available_tf(real_root)
    rows = []
    for particles, bag_name in sorted(discover_bag_names(real_root).items()):
        bag_dir = real_root / f"ParticleCount{particles}" / bag_name
        tf = read_map_world_tf(bag_dir)
        transform_source = "bag /tf_static"
        if tf is None:
            tf = fallback_tf
            transform_source = "fallback /tf_static"
        trans, q_tf = tf
        rot = quat_to_rot(q_tf)

        t_opti, opti_raw, q_opti_raw = read_pose_series(bag_dir, "/vrpn_mocap/Car2/pose")
        t_pose, pose_pos, q_pose = read_pose_series(bag_dir, pose_topic)
        t_opti, opti_raw, q_opti_raw = filter_optitrack(t_opti, opti_raw, q_opti_raw)

        gt_pos_raw = (rot @ opti_raw.T).T + trans
        gt_yaw_raw = np.array([yaw_from_quat(qmul(q_tf, q)) for q in q_opti_raw])
        pose_yaw_all = np.array([yaw_from_quat(q) for q in q_pose])

        t0 = max(float(np.min(t_opti)), float(np.min(t_pose)))
        t1 = min(float(np.max(t_opti)), float(np.max(t_pose)))
        time_mask = (t_opti >= t0) & (t_opti <= t1)
        t = t_opti[time_mask]
        gt_xy_raw = gt_pos_raw[time_mask, :2].copy()
        gt_yaw_raw = gt_yaw_raw[time_mask].copy()
        pose_xy = np.column_stack([
            np.interp(t, t_pose, pose_pos[:, 0]),
            np.interp(t, t_pose, pose_pos[:, 1]),
        ])
        pose_yaw = interp_yaw(t_pose, pose_yaw_all, t)

        # Error is start-calibrated to match the equivalence analysis, but the
        # heat-map location stays in the raw map-frame OptiTrack coordinates.
        gt_xy_error, gt_yaw_error, _ = start_calibrate(
            t, gt_xy_raw.copy(), gt_yaw_raw.copy(), pose_xy, pose_yaw)
        position_error_cm = 100.0 * np.linalg.norm(pose_xy - gt_xy_error, axis=1)
        yaw_error_deg = np.degrees(wrap_angle(pose_yaw - gt_yaw_error))
        speed = np.r_[
            np.nan,
            np.linalg.norm(np.diff(gt_xy_raw, axis=0), axis=1) /
            np.maximum(np.diff(t), np.finfo(float).eps),
        ]

        segments = lap_segments(t, gt_xy_raw)
        if not segments:
            segments = [(0, t.size - 1)]

        for lap_idx, (start, end) in enumerate(segments, start=1):
            idx = np.arange(start, end + 1)
            mask = (
                roi_mask(gt_xy_raw[idx, 0], gt_xy_raw[idx, 1], roi)
                & (speed[idx] >= min_speed_mps)
                & np.isfinite(position_error_cm[idx])
                & np.isfinite(yaw_error_deg[idx])
            )
            for sample_idx in idx[mask]:
                rows.append({
                    "particles": int(particles),
                    "bag": bag_name,
                    "unit": f"real_lap_{lap_idx:02d}",
                    "x_m": float(gt_xy_raw[sample_idx, 0]),
                    "y_m": float(gt_xy_raw[sample_idx, 1]),
                    "position_error_cm": float(position_error_cm[sample_idx]),
                    "yaw_error_deg": float(yaw_error_deg[sample_idx]),
                    "pose_topic": pose_topic,
                    "transform_source": transform_source,
                })

    return pd.DataFrame(rows)


def bin_median_grid(
    x: np.ndarray,
    y: np.ndarray,
    values: np.ndarray,
    x_edges: np.ndarray,
    y_edges: np.ndarray,
    min_bin_samples: int,
) -> Tuple[np.ndarray, np.ndarray]:
    valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(values)
    x_bin = np.digitize(x[valid], x_edges) - 1
    y_bin = np.digitize(y[valid], y_edges) - 1
    in_range = (
        (x_bin >= 0) & (x_bin < x_edges.size - 1) &
        (y_bin >= 0) & (y_bin < y_edges.size - 1)
    )
    x_bin = x_bin[in_range]
    y_bin = y_bin[in_range]
    vals = values[valid][in_range]

    grid = np.full((y_edges.size - 1, x_edges.size - 1), np.nan, dtype=float)
    counts = np.zeros_like(grid, dtype=int)
    for row in range(grid.shape[0]):
        for col in range(grid.shape[1]):
            mask = (x_bin == col) & (y_bin == row)
            counts[row, col] = int(np.count_nonzero(mask))
            if counts[row, col] >= min_bin_samples:
                grid[row, col] = float(np.median(vals[mask]))
    return grid, counts


def draw_map_background(ax: plt.Axes, background: Tuple[np.ndarray, Tuple[float, float, float, float]] | None) -> None:
    if background is None:
        return
    image, extent = background
    ax.imshow(
        image,
        extent=extent,
        origin="lower",
        cmap="gray",
        vmin=0,
        vmax=1,
        alpha=0.35,
        interpolation="nearest",
        zorder=0,
    )


def plot_particle_heatmap(
    rows: pd.DataFrame,
    particles: int,
    bag_name: str,
    output_dir: Path,
    x_edges: np.ndarray,
    y_edges: np.ndarray,
    view_extent: Tuple[float, float, float, float],
    background: Tuple[np.ndarray, Tuple[float, float, float, float]] | None,
    position_vmax_cm: float,
    yaw_vmax_deg: float,
    min_bin_samples: int,
    pose_topic: str,
) -> Dict[str, float | int | str]:
    x = rows["x_m"].to_numpy(dtype=float)
    y = rows["y_m"].to_numpy(dtype=float)
    position = rows["position_error_cm"].to_numpy(dtype=float)
    yaw_abs = np.abs(rows["yaw_error_deg"].to_numpy(dtype=float))
    position_grid, count_grid = bin_median_grid(x, y, position, x_edges, y_edges, min_bin_samples)
    yaw_grid, _ = bin_median_grid(x, y, yaw_abs, x_edges, y_edges, min_bin_samples)

    pos_masked = np.ma.masked_invalid(position_grid)
    yaw_masked = np.ma.masked_invalid(yaw_grid)

    fig, axes = plt.subplots(1, 2, figsize=(13.5, 5.3), constrained_layout=True)
    specs = [
        (axes[0], pos_masked, "Median position error [cm]", position_vmax_cm, "magma"),
        (axes[1], yaw_masked, "Median |yaw error| [deg]", yaw_vmax_deg, "viridis"),
    ]
    for ax, grid, label, vmax, cmap_name in specs:
        draw_map_background(ax, background)
        mesh = ax.pcolormesh(
            x_edges,
            y_edges,
            grid,
            shading="auto",
            cmap=cmap_name,
            vmin=0.0,
            vmax=vmax,
            alpha=0.88,
            zorder=1,
        )
        ax.plot(x, y, ".", color="black", markersize=0.7, alpha=0.18, zorder=2)
        roi_rect = plt.Rectangle(
            (float(x_edges[0]), float(y_edges[0])),
            float(x_edges[-1] - x_edges[0]),
            float(y_edges[-1] - y_edges[0]),
            fill=False,
            edgecolor="#1f77b4",
            linewidth=1.2,
            linestyle="--",
            alpha=0.75,
            zorder=3,
        )
        ax.add_patch(roi_rect)
        ax.set_xlim(view_extent[0], view_extent[1])
        ax.set_ylim(view_extent[2], view_extent[3])
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, color="white", alpha=0.25, linewidth=0.5)
        ax.set_xlabel("map x [m]")
        ax.set_ylabel("map y [m]")
        cbar = fig.colorbar(mesh, ax=ax, shrink=0.9)
        cbar.set_label(label)

    median_position = float(np.median(position))
    p95_position = float(np.percentile(position, 95))
    median_yaw = float(np.median(yaw_abs))
    p95_yaw = float(np.percentile(yaw_abs, 95))
    transform_source = str(rows["transform_source"].iloc[0]) if "transform_source" in rows else "unknown TF"
    pose_label = pose_topic.strip("/").replace("_", " ")
    fig.suptitle(
        f"ParticleCount{particles} {bag_name} {pose_label} ROI heat map\n"
        f"N={len(rows)} samples, pos median={median_position:.1f} cm, "
        f"pos p95={p95_position:.1f} cm, yaw median={median_yaw:.2f} deg, "
        f"TF={transform_source}",
        fontsize=11,
    )

    pose_stem = pose_topic.strip("/").replace("/", "_")
    output_path = output_dir / f"ParticleCount{particles:04d}_{bag_name}_{pose_stem}_roi_heatmap.png"
    fig.savefig(output_path, dpi=300)
    plt.close(fig)

    valid_bins = int(np.count_nonzero(np.isfinite(position_grid)))
    return {
        "particles": int(particles),
        "bag": bag_name,
        "samples": int(len(rows)),
        "valid_position_bins": valid_bins,
        "median_position_cm": median_position,
        "p95_position_cm": p95_position,
        "median_abs_yaw_deg": median_yaw,
        "p95_abs_yaw_deg": p95_yaw,
        "pose_topic": pose_topic,
        "transform_source": transform_source,
        "heatmap_path": str(output_path),
    }


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[2]
    default_samples = (
        repo_root / "f1tenth_localization" / "Benchmark" / "Matlab" /
        "sim_benchmark" / "optitrack_map_roi_particle_match_5laps" /
        "Equivalence_ROI_Samples.csv"
    )
    default_real_root = (
        repo_root / "f1tenth_localization" / "Benchmark" / "bags" /
        "OptitrackBags" / "AMCL"
    )
    default_map_yaml = (
        repo_root / "f1tenth_localization" / "Benchmark" / "Matlab" /
        "plots" / "BagMapAndRaceline" / "OptitrackBenchmark_20260430_120324" /
        "bag_map.yaml"
    )
    default_output = (
        repo_root / "f1tenth_localization" / "Benchmark" / "Matlab" /
        "plots" / "OptiTrackAMCL" / "ROI_Bag_Heatmaps"
    )

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--samples-csv",
        type=Path,
        default=default_samples,
        help="Deprecated fallback input. Heat maps are generated from bags by default.")
    parser.add_argument("--real-root", type=Path, default=default_real_root)
    parser.add_argument("--map-yaml", type=Path, default=default_map_yaml)
    parser.add_argument("--output-dir", type=Path, default=default_output)
    parser.add_argument("--roi-min-x", type=float, default=0.0)
    parser.add_argument("--roi-max-x", type=float, default=8.0)
    parser.add_argument("--roi-min-y", type=float, default=-1.4)
    parser.add_argument("--roi-max-y", type=float, default=math.inf)
    parser.add_argument("--roi-y-padding", type=float, default=0.25)
    parser.add_argument("--bin-size-m", type=float, default=0.20)
    parser.add_argument("--min-bin-samples", type=int, default=2)
    parser.add_argument("--min-speed-mps", type=float, default=0.2)
    parser.add_argument("--pose-topic", default="/amcl_pose")
    parser.add_argument("--position-vmax-cm", type=float, default=30.0)
    parser.add_argument("--yaw-vmax-deg", type=float, default=8.0)
    parser.add_argument(
        "--view",
        choices=("full-map", "roi"),
        default="full-map",
        help="Use full map axes while showing only ROI data, or crop the axes to the ROI.",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    roi = (args.roi_min_x, args.roi_max_x, args.roi_min_y, args.roi_max_y)
    real = load_real_bag_roi_samples(args.real_root, roi, args.min_speed_mps, args.pose_topic)
    if real.empty:
        raise ValueError("No real OptiTrack ROI samples found")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    background = load_map_background(args.map_yaml)
    if args.view == "full-map" and background is not None:
        view_extent = background[1]
    else:
        roi_view_max_y = args.roi_max_y
        if not math.isfinite(roi_view_max_y):
            roi_view_max_y = finite_roi_max_y(
                real["y_m"].to_numpy(dtype=float), args.roi_max_y, args.roi_y_padding)
        view_extent = (
            float(args.roi_min_x),
            float(args.roi_max_x),
            float(args.roi_min_y),
            float(roi_view_max_y),
        )
    roi_max_y = args.roi_max_y
    if not math.isfinite(roi_max_y):
        roi_max_y = view_extent[3] if args.view == "full-map" else finite_roi_max_y(
            real["y_m"].to_numpy(dtype=float), args.roi_max_y, args.roi_y_padding)
    x_edges = np.arange(args.roi_min_x, args.roi_max_x + args.bin_size_m, args.bin_size_m)
    y_edges = np.arange(args.roi_min_y, roi_max_y + args.bin_size_m, args.bin_size_m)

    sample_path = args.output_dir / "OptiTrack_AMCL_ROI_Bag_Heatmap_Samples.csv"
    real.to_csv(sample_path, index=False)
    rows = []
    for particles in sorted(real["particles"].unique()):
        particle_rows = real[real["particles"] == particles].copy()
        bag_name = str(particle_rows["bag"].iloc[0])
        rows.append(plot_particle_heatmap(
            particle_rows,
            int(particles),
            bag_name,
            args.output_dir,
            x_edges,
            y_edges,
            view_extent,
            background,
            args.position_vmax_cm,
            args.yaw_vmax_deg,
            args.min_bin_samples,
            args.pose_topic,
        ))

    summary = pd.DataFrame(rows)
    summary_path = args.output_dir / "OptiTrack_AMCL_ROI_Bag_Heatmap_Summary.csv"
    summary.to_csv(summary_path, index=False)
    print(f"Wrote {len(summary)} heat maps to {args.output_dir}")
    print(f"Samples: {sample_path}")
    print(f"Summary: {summary_path}")
    print(summary[[
        "particles", "bag", "pose_topic", "transform_source", "samples", "valid_position_bins",
        "median_position_cm", "p95_position_cm",
        "median_abs_yaw_deg", "p95_abs_yaw_deg",
    ]].to_string(index=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
