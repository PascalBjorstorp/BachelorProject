#!/usr/bin/env python3
"""Plot the OptiTrack failure map region used for real/simulation AMCL matching."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, Tuple

import numpy as np
import pandas as pd

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
except ImportError as exc:  # pragma: no cover
    raise SystemExit("matplotlib is required for plotting") from exc


CATEGORIES: Dict[str, Tuple[str, str]] = {
    "OptiTrack stall": ("#2f77d1", "o"),
    "Yaw rate > 3 rad/s": ("#d62728", "^"),
    "Sudden yaw jump": ("#ff7f0e", "v"),
    "Sideways/backwards motion": ("#7f3fbf", "s"),
    "Position jump": ("#1a9850", "d"),
}


def parse_map_yaml(path: Path) -> Tuple[Path, float, Tuple[float, float, float]]:
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
            values = value.strip("[]").split(",")
            origin = tuple(float(v.strip()) for v in values)  # type: ignore[assignment]
    if image_path is None or resolution is None or origin is None:
        raise ValueError(f"Could not parse map yaml: {path}")
    return image_path, resolution, origin


def load_map(map_yaml: Path) -> Tuple[np.ndarray, Tuple[float, float, float, float]]:
    image_path, resolution, origin = parse_map_yaml(map_yaml)
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


def draw_map(ax: plt.Axes, image: np.ndarray, extent: Tuple[float, float, float, float]) -> None:
    ax.imshow(image, extent=extent, origin="lower", cmap="gray", vmin=0, vmax=1, alpha=0.95)


def plot_failures(
    ax: plt.Axes,
    issues: pd.DataFrame,
    mask: np.ndarray,
    alpha: float,
    size: float,
    include_labels: bool,
) -> None:
    frame = issues[mask]
    for category, (color, marker) in CATEGORIES.items():
        selected = frame["category"] == category
        if not selected.any():
            continue
        label = category if include_labels else None
        ax.scatter(
            frame.loc[selected, "x_m"],
            frame.loc[selected, "y_m"],
            s=size,
            c=color,
            marker=marker,
            alpha=alpha,
            linewidths=0.35,
            edgecolors="white",
            label=label,
        )


def main() -> int:
    plt.rcParams.update({
        "font.size": 16,
        "axes.labelsize": 17,
        "xtick.labelsize": 14,
        "ytick.labelsize": 14,
        "legend.fontsize": 14,
    })
    repo_root = Path(__file__).resolve().parents[2]
    failure_csv = (
        repo_root / "f1tenth_localization" / "Benchmark" / "Matlab" / "plots" /
        "OptiTrackFailureMap" / "OptiTrack_Failure_Map_Samples.csv"
    )
    map_yaml = (
        repo_root / "f1tenth_localization" / "Benchmark" / "Matlab" / "plots" /
        "BagMapAndRaceline" / "OptitrackBenchmark_20260430_120324" / "bag_map.yaml"
    )
    output_dir = (
        repo_root / "f1tenth_localization" / "Benchmark" / "Matlab" / "plots" /
        "OptiTrackFailureMap"
    )
    report_path = (
        repo_root / "Report" / "Sections" / "Localization" / "AMCL" / "Images" /
        "Test" / "optitrack_failure_roi.png"
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    report_path.parent.mkdir(parents=True, exist_ok=True)

    issues = pd.read_csv(failure_csv)
    rois = [
        (0.6, 8.0, -1.0, 1.5),
        (1.0, 6.0, -5.0, -3.0),
    ]
    roi = np.zeros(len(issues), dtype=bool)
    for min_x, max_x, min_y, max_y in rois:
        roi |= (
            (issues["x_m"].to_numpy() >= min_x) &
            (issues["x_m"].to_numpy() <= max_x) &
            (issues["y_m"].to_numpy() >= min_y) &
            (issues["y_m"].to_numpy() <= max_y)
        )
    image, extent = load_map(map_yaml)
    fig, axes = plt.subplots(1, 2, figsize=(14.5, 5.6))

    draw_map(axes[0], image, extent)
    plot_failures(axes[0], issues, ~roi, alpha=0.22, size=15, include_labels=True)
    plot_failures(axes[0], issues, roi, alpha=0.95, size=27, include_labels=False)
    for min_x, max_x, min_y, max_y in rois:
        axes[0].add_patch(Rectangle(
            (min_x, min_y),
            max_x - min_x,
            max_y - min_y,
            fill=False,
            edgecolor="#1f78b4",
            linewidth=2.2,
            linestyle="--",
        ))
    axes[0].set_xlim(extent[0], extent[1])
    axes[0].set_ylim(extent[2], extent[3])

    draw_map(axes[1], image, extent)
    plot_failures(axes[1], issues, roi, alpha=0.95, size=34, include_labels=False)
    for min_x, max_x, min_y, max_y in rois:
        axes[1].add_patch(Rectangle(
            (min_x, min_y),
            max_x - min_x,
            max_y - min_y,
            fill=False,
            edgecolor="#1f78b4",
            linewidth=2.2,
            linestyle="--",
        ))
    axes[1].set_xlim(-0.25, 8.25)
    axes[1].set_ylim(-5.35, min(1.75, extent[3]))

    for ax in axes:
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, color="0.2", alpha=0.18, linewidth=0.7)
        ax.set_xlabel("map x [m]")
        ax.set_ylabel("map y [m]")

    handles, labels = axes[0].get_legend_handles_labels()
    fig.subplots_adjust(left=0.065, right=0.985, top=0.82, bottom=0.12, wspace=0.22)
    fig.legend(
        handles,
        labels,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.985),
        ncol=len(labels),
        frameon=True,
        markerscale=2.0,
        scatterpoints=1,
        handletextpad=0.4,
        columnspacing=0.8,
    )

    summary = issues[roi].groupby("category").size().to_dict()
    print(f"ROI failure rows: {int(roi.sum())} / {len(issues)}")
    print(summary)

    output_path = output_dir / "OptiTrack_Failure_Map_ROI.png"
    fig.savefig(output_path, dpi=220)
    fig.savefig(report_path, dpi=220)
    print(output_path)
    print(report_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
