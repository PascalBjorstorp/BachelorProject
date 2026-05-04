#!/usr/bin/env python3
"""Render raceline geometry and clearance diagnostics for MPCC debugging."""

from __future__ import annotations

import argparse
import ast
import csv
import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.transforms import Affine2D
from PIL import Image


DEFAULT_VEHICLE_HALF_WIDTH_M = 0.155
DEFAULT_SAFETY_MARGIN_M = 0.010
DEFAULT_WARN_CLEARANCE_M = 0.050
DEFAULT_HIGHLIGHT_COUNT = 12


@dataclass
class Waypoint:
    index: int
    s: float
    x: float
    y: float
    psi: float
    kappa: float
    vx: float
    ax: float
    left: float
    right: float


@dataclass
class MapOverlay:
    yaml_path: Path
    image_path: Path
    image_data: np.ndarray
    width_m: float
    height_m: float
    resolution: float
    origin_x: float
    origin_y: float
    yaw_rad: float


@dataclass
class SimTraceSample:
    step: int
    time_s: float
    x: float
    y: float
    s: float
    wall_hit: int


def load_raceline(csv_path: Path) -> list[Waypoint]:
    points: list[Waypoint] = []
    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.reader(handle)
        for row in reader:
            if not row:
                continue
            if row[0].strip().startswith("#"):
                continue

            values = [value.strip() for value in row]
            if len(values) < 7:
                continue

            left = float(values[7]) if len(values) >= 8 and values[7] else float("nan")
            right = float(values[8]) if len(values) >= 9 and values[8] else float("nan")
            points.append(
                Waypoint(
                    index=len(points),
                    s=float(values[0]),
                    x=float(values[1]),
                    y=float(values[2]),
                    psi=float(values[3]),
                    kappa=float(values[4]),
                    vx=float(values[5]),
                    ax=float(values[6]),
                    left=left,
                    right=right,
                )
            )

    if not points:
        raise ValueError(f"No trajectory samples found in {csv_path}")
    return points


def offset_point(wp: Waypoint, offset_m: float, direction: str) -> tuple[float, float]:
    nx = -math.sin(wp.psi)
    ny = math.cos(wp.psi)
    signed_offset = offset_m if direction == "left" else -offset_m
    return wp.x + signed_offset * nx, wp.y + signed_offset * ny


def build_edge(points: list[Waypoint], offsets: list[float], direction: str) -> tuple[list[float], list[float]]:
    xs: list[float] = []
    ys: list[float] = []
    for wp, offset in zip(points, offsets):
        px, py = offset_point(wp, offset, direction)
        xs.append(px)
        ys.append(py)
    return xs, ys


def load_map_overlay(yaml_path: Path) -> MapOverlay:
    values: dict[str, str] = {}
    with yaml_path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.split("#", 1)[0].strip()
            if not line or ":" not in line:
                continue
            key, value = line.split(":", 1)
            values[key.strip()] = value.strip()

    try:
        image_value = values["image"].strip("\"'")
        resolution = float(values["resolution"])
        origin = ast.literal_eval(values["origin"])
        negate = int(values.get("negate", "0"))
    except KeyError as exc:
        raise ValueError(f"Missing required field in map YAML {yaml_path}: {exc.args[0]}") from exc

    if not isinstance(origin, (list, tuple)) or len(origin) < 2:
        raise ValueError(f"Map origin must be a 2D or 3D list in {yaml_path}")

    image_path = Path(image_value)
    if not image_path.is_absolute():
        image_path = (yaml_path.parent / image_path).resolve()
    if not image_path.exists():
        raise FileNotFoundError(f"Map image not found: {image_path}")

    image_data = np.asarray(Image.open(image_path))
    if image_data.ndim == 3:
        image_data = image_data[..., 0]
    if negate:
        image_data = 255 - image_data

    flipped = image_data[::-1]
    height_px, width_px = flipped.shape[:2]
    return MapOverlay(
        yaml_path=yaml_path,
        image_path=image_path,
        image_data=flipped,
        width_m=width_px * resolution,
        height_m=height_px * resolution,
        resolution=resolution,
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
        yaw_rad=float(origin[2]) if len(origin) >= 3 else 0.0,
    )


def load_sim_trace(csv_path: Path) -> list[SimTraceSample]:
    samples: list[SimTraceSample] = []
    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        required = {"step", "time_s", "x_m", "y_m", "s_m", "wall_hit"}
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise ValueError(
                f"Simulation trace {csv_path} is missing required columns: {sorted(required)}"
            )

        for row in reader:
            if not row:
                continue
            samples.append(
                SimTraceSample(
                    step=int(row["step"]),
                    time_s=float(row["time_s"]),
                    x=float(row["x_m"]),
                    y=float(row["y_m"]),
                    s=float(row["s_m"]),
                    wall_hit=int(row["wall_hit"]),
                )
            )

    if not samples:
        raise ValueError(f"No samples found in simulation trace {csv_path}")
    return samples


def parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    project_dir = script_dir.parent.parent
    default_raceline = project_dir / "f1tenth_planning" / "trajectories" / "my_track_centerline_smooth.csv"

    parser = argparse.ArgumentParser(description="Visualize raceline geometry and effective clearances.")
    parser.add_argument(
        "--raceline",
        default=str(default_raceline),
        help="Path to a 9-column raceline CSV.",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Output PNG path. Defaults to <raceline>_mpcc_diagnostics.png.",
    )
    parser.add_argument(
        "--map-yaml",
        default=None,
        help="Optional ROS occupancy map YAML to overlay under the XY plot.",
    )
    parser.add_argument(
        "--sim-trace",
        default=None,
        help="Optional simulator trace CSV exported by test_sim_drive.c.",
    )
    parser.add_argument(
        "--vehicle-half-width",
        type=float,
        default=DEFAULT_VEHICLE_HALF_WIDTH_M,
        help="Vehicle half-width in meters.",
    )
    parser.add_argument(
        "--safety-margin",
        type=float,
        default=DEFAULT_SAFETY_MARGIN_M,
        help="Extra lateral safety margin in meters.",
    )
    parser.add_argument(
        "--warn-clearance",
        type=float,
        default=DEFAULT_WARN_CLEARANCE_M,
        help="Clearance threshold to flag as narrow after subtracting body and margin.",
    )
    parser.add_argument(
        "--highlight-index",
        type=int,
        action="append",
        default=[],
        help="Waypoint index to highlight. Can be passed multiple times.",
    )
    parser.add_argument(
        "--highlight-count",
        type=int,
        default=DEFAULT_HIGHLIGHT_COUNT,
        help="How many of the worst-clearance points to label automatically.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    raceline_path = Path(args.raceline).expanduser().resolve()
    if not raceline_path.exists():
        raise FileNotFoundError(f"Raceline not found: {raceline_path}")

    output_path = (
        Path(args.output).expanduser().resolve()
        if args.output
        else raceline_path.with_name(f"{raceline_path.stem}_mpcc_diagnostics.png")
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)

    points = load_raceline(raceline_path)
    has_bounds = all(math.isfinite(wp.left) and math.isfinite(wp.right) for wp in points)
    if not has_bounds:
        raise ValueError("This visualizer requires 9-column raceline CSVs with left/right bounds.")

    map_overlay: MapOverlay | None = None
    if args.map_yaml:
        map_yaml_path = Path(args.map_yaml).expanduser().resolve()
        if not map_yaml_path.exists():
            raise FileNotFoundError(f"Map YAML not found: {map_yaml_path}")
        map_overlay = load_map_overlay(map_yaml_path)

    sim_trace_path: Path | None = None
    sim_trace: list[SimTraceSample] | None = None
    if args.sim_trace:
        sim_trace_path = Path(args.sim_trace).expanduser().resolve()
        if not sim_trace_path.exists():
            raise FileNotFoundError(f"Simulation trace not found: {sim_trace_path}")
        sim_trace = load_sim_trace(sim_trace_path)

    body_margin = args.vehicle_half_width + args.safety_margin
    raw_left = [wp.left for wp in points]
    raw_right = [wp.right for wp in points]
    eff_left = [wp.left - body_margin for wp in points]
    eff_right = [wp.right - body_margin for wp in points]
    clipped_left = [max(value, 0.0) for value in eff_left]
    clipped_right = [max(value, 0.0) for value in eff_right]

    center_x = [wp.x for wp in points]
    center_y = [wp.y for wp in points]
    s_vals = [wp.s for wp in points]

    raw_left_x, raw_left_y = build_edge(points, raw_left, "left")
    raw_right_x, raw_right_y = build_edge(points, raw_right, "right")
    safe_left_x, safe_left_y = build_edge(points, clipped_left, "left")
    safe_right_x, safe_right_y = build_edge(points, clipped_right, "right")

    invalid_indices = [i for i, (left, right) in enumerate(zip(eff_left, eff_right)) if left <= 0.0 or right <= 0.0]
    narrow_indices = [
        i for i, (left, right) in enumerate(zip(eff_left, eff_right))
        if i not in invalid_indices and min(left, right) < args.warn_clearance
    ]

    ranked = sorted(
        range(len(points)),
        key=lambda i: min(eff_left[i], eff_right[i]),
    )
    auto_labels = ranked[: max(args.highlight_count, 0)]
    highlight_indices = sorted(set(auto_labels + [i for i in args.highlight_index if 0 <= i < len(points)]))

    fig = plt.figure(figsize=(16, 9), constrained_layout=True)
    grid = fig.add_gridspec(2, 2, width_ratios=[1.6, 1.0], height_ratios=[1.0, 0.65])
    ax_xy = fig.add_subplot(grid[:, 0])
    ax_clear = fig.add_subplot(grid[0, 1])
    ax_text = fig.add_subplot(grid[1, 1])

    if map_overlay is not None:
        transform = Affine2D().rotate(map_overlay.yaw_rad).translate(map_overlay.origin_x, map_overlay.origin_y)
        ax_xy.imshow(
            map_overlay.image_data,
            cmap="gray",
            extent=(0.0, map_overlay.width_m, 0.0, map_overlay.height_m),
            origin="lower",
            interpolation="nearest",
            alpha=0.72,
            transform=transform + ax_xy.transData,
            zorder=0,
        )

    ax_xy.plot(raw_left_x, raw_left_y, color="#111827", linewidth=1.6, label="raw left wall")
    ax_xy.plot(raw_right_x, raw_right_y, color="#111827", linewidth=1.6, label="raw right wall")
    ax_xy.plot(center_x, center_y, color="#2563eb", linewidth=1.5, label="raceline")
    ax_xy.plot(safe_left_x, safe_left_y, color="#f59e0b", linewidth=1.0, linestyle="--", label="vehicle-safe left")
    ax_xy.plot(safe_right_x, safe_right_y, color="#10b981", linewidth=1.0, linestyle="--", label="vehicle-safe right")

    if sim_trace is not None:
        trace_x = [sample.x for sample in sim_trace]
        trace_y = [sample.y for sample in sim_trace]
        ax_xy.plot(trace_x, trace_y, color="#be123c", linewidth=1.5, alpha=0.92, label="sim trace", zorder=4)

        collision_samples = [sample for sample in sim_trace if sample.wall_hit != 0]
        if collision_samples:
            first_collision = collision_samples[0]
            ax_xy.scatter(
                [first_collision.x],
                [first_collision.y],
                s=64,
                marker="x",
                c="#be123c",
                linewidths=2.0,
                label="sim wall hit",
                zorder=7,
            )

    if narrow_indices:
        ax_xy.scatter(
            [center_x[i] for i in narrow_indices],
            [center_y[i] for i in narrow_indices],
            s=18,
            c="#f59e0b",
            alpha=0.9,
            label=f"narrow < {args.warn_clearance:.3f} m",
            zorder=5,
        )
    if invalid_indices:
        ax_xy.scatter(
            [center_x[i] for i in invalid_indices],
            [center_y[i] for i in invalid_indices],
            s=30,
            c="#dc2626",
            alpha=0.95,
            label="invalid <= 0 clearance",
            zorder=6,
        )

    for idx in highlight_indices:
        ax_xy.annotate(
            str(idx),
            (center_x[idx], center_y[idx]),
            textcoords="offset points",
            xytext=(4, 4),
            fontsize=7,
            color="#7c3aed",
            bbox={"boxstyle": "round,pad=0.15", "fc": "white", "ec": "#7c3aed", "lw": 0.5, "alpha": 0.85},
        )

    ax_xy.set_title(
        "Raceline, walls, and vehicle-safe corridor"
        if map_overlay is None
        else "Raceline, walls, vehicle-safe corridor, and map overlay"
    )
    ax_xy.set_aspect("equal", adjustable="box")
    ax_xy.grid(alpha=0.18)
    ax_xy.legend(loc="best", fontsize=9)

    ax_clear.plot(s_vals, raw_left, color="#6b7280", linewidth=1.1, label="raw left")
    ax_clear.plot(s_vals, raw_right, color="#9ca3af", linewidth=1.1, label="raw right")
    ax_clear.plot(s_vals, eff_left, color="#f59e0b", linewidth=1.4, label="effective left")
    ax_clear.plot(s_vals, eff_right, color="#10b981", linewidth=1.4, label="effective right")
    ax_clear.axhline(0.0, color="#dc2626", linewidth=1.0, linestyle="--")
    ax_clear.axhline(args.warn_clearance, color="#f59e0b", linewidth=1.0, linestyle=":")
    if invalid_indices:
        ax_clear.scatter([s_vals[i] for i in invalid_indices], [min(eff_left[i], eff_right[i]) for i in invalid_indices], c="#dc2626", s=18, zorder=5)
    if narrow_indices:
        ax_clear.scatter([s_vals[i] for i in narrow_indices], [min(eff_left[i], eff_right[i]) for i in narrow_indices], c="#f59e0b", s=14, zorder=4)
    if sim_trace is not None:
        collision_samples = [sample for sample in sim_trace if sample.wall_hit != 0]
        if collision_samples:
            first_collision = collision_samples[0]
            ax_clear.scatter([first_collision.s], [0.0], c="#be123c", s=56, marker="x", zorder=6)
    ax_clear.set_title("Raw and effective wall clearance")
    ax_clear.set_xlabel("s [m]")
    ax_clear.set_ylabel("clearance [m]")
    ax_clear.grid(alpha=0.25)
    ax_clear.legend(loc="best", fontsize=8)

    worst_points = ranked[: min(8, len(ranked))]
    summary_lines = [
        f"Raceline: {raceline_path.name}",
        f"Waypoints: {len(points)}",
        f"Track length: {points[-1].s:.3f} m",
        f"Map overlay: {map_overlay.yaml_path.name}" if map_overlay is not None else "Map overlay: none",
        f"Sim trace: {sim_trace_path.name}" if sim_trace_path is not None else "Sim trace: none",
        (
            f"Map origin/res: ({map_overlay.origin_x:.2f}, {map_overlay.origin_y:.2f}) / {map_overlay.resolution:.3f} m"
            if map_overlay is not None
            else "Map origin/res: n/a"
        ),
        f"Trace samples: {len(sim_trace)}" if sim_trace is not None else "Trace samples: n/a",
        f"Vehicle half-width: {args.vehicle_half_width:.3f} m",
        f"Safety margin: {args.safety_margin:.3f} m",
        f"Effective body+margin: {body_margin:.3f} m",
        f"Invalid points: {len(invalid_indices)}",
        f"Narrow points (< {args.warn_clearance:.3f} m): {len(narrow_indices)}",
        "",
        "Worst effective clearances:",
    ]
    for idx in worst_points:
        summary_lines.append(
            f"#{idx:4d}  s={points[idx].s:6.3f}  effL={eff_left[idx]:6.3f}  effR={eff_right[idx]:6.3f}"
        )

    ax_text.axis("off")
    ax_text.set_title("Summary")
    ax_text.text(
        0.0,
        1.0,
        "\n".join(summary_lines),
        ha="left",
        va="top",
        family="monospace",
        fontsize=10,
    )

    fig.suptitle("MPCC Raceline Diagnostics", fontsize=15)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)

    print(f"Saved diagnostics plot to: {output_path}")
    if map_overlay is not None:
        print(f"Map overlay: {map_overlay.yaml_path} -> {map_overlay.image_path}")
    if sim_trace_path is not None:
        print(f"Simulation trace: {sim_trace_path}")
    print(f"Invalid points: {len(invalid_indices)}")
    print(f"Narrow points (< {args.warn_clearance:.3f} m): {len(narrow_indices)}")
    if worst_points:
        worst = worst_points[0]
        print(
            "Worst point: "
            f"index={worst} s={points[worst].s:.3f} "
            f"eff_left={eff_left[worst]:.3f} eff_right={eff_right[worst]:.3f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
