#!/usr/bin/env python3
"""Generate a MPCC-compatible centerline CSV from a 9-column raceline CSV.

The input CSV convention is:
  s,x,y,psi,kappa,vx,ax,d_left,d_right

For each waypoint, the provisional center point is the midpoint between the
left and right wall rays:
  center = p + 0.5 * (d_left - d_right) * left_normal

If --map is supplied, the script ray-casts fresh wall distances from the map
for the generated centerline. Otherwise it writes the geometric half-width
estimate from the source CSV.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from compute_wall_distances import compute_wall_distances, load_map  # noqa: E402
from optimize_trajectory import (  # noqa: E402
    compute_centerline_thinned_loop,
    detect_winding_direction,
    load_map as load_planning_map,
    measure_track_widths,
)


@dataclass
class Waypoint:
    s: float
    x: float
    y: float
    psi: float
    kappa: float
    vx: float
    ax: float
    left: float
    right: float


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def load_raceline(path: Path) -> list[Waypoint]:
    points: list[Waypoint] = []
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.reader(handle)
        for row in reader:
            if not row or row[0].strip().startswith("#"):
                continue
            if len(row) < 9:
                continue
            values = [float(value) for value in row[:9]]
            points.append(Waypoint(*values))
    if len(points) < 4:
        raise ValueError(f"Need at least 4 waypoints in {path}")
    return points


def has_duplicate_endpoint(points: list[Waypoint]) -> bool:
    first = points[0]
    last = points[-1]
    return math.hypot(first.x - last.x, first.y - last.y) < 1.0e-4


def compute_center_xy(points: list[Waypoint]) -> tuple[np.ndarray, np.ndarray]:
    n = len(points)
    xy = np.zeros((n, 2), dtype=float)
    half_width = np.zeros(n, dtype=float)

    for i, point in enumerate(points):
        left_normal = np.array([-math.sin(point.psi), math.cos(point.psi)])
        lateral_offset = 0.5 * (point.left - point.right)
        xy[i] = np.array([point.x, point.y]) + lateral_offset * left_normal
        half_width[i] = 0.5 * (point.left + point.right)

    return xy, half_width


def closed_geometry(xy_unique: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    next_xy = np.roll(xy_unique, -1, axis=0)
    prev_xy = np.roll(xy_unique, 1, axis=0)

    seg = next_xy - xy_unique
    seg_len = np.linalg.norm(seg, axis=1)
    total_length = float(np.sum(seg_len))
    if total_length <= 1.0e-6:
        raise ValueError("Generated centerline has near-zero length")

    s = np.zeros(len(xy_unique), dtype=float)
    s[1:] = np.cumsum(seg_len[:-1])

    tangent = next_xy - prev_xy
    psi = np.unwrap(np.arctan2(tangent[:, 1], tangent[:, 0]))

    prev_s = np.roll(s, 1)
    next_s = np.roll(s, -1)
    ds_prev = s - prev_s
    ds_next = next_s - s
    ds_prev[0] += total_length
    ds_next[-1] += total_length
    ds_total = ds_prev + ds_next

    psi_prev = np.roll(psi, 1)
    psi_next = np.roll(psi, -1)
    dpsi = np.array([wrap_angle(a - b) for a, b in zip(psi_next, psi_prev)])
    kappa = np.divide(dpsi, ds_total, out=np.zeros_like(dpsi), where=ds_total > 1.0e-9)
    psi = np.array([wrap_angle(value) for value in psi])

    return s, psi, kappa


def speed_profile(
    kappa: np.ndarray,
    min_speed: float,
    max_speed: float,
    lateral_acc_limit: float,
    smooth_passes: int,
) -> np.ndarray:
    abs_kappa = np.maximum(np.abs(kappa), 1.0e-6)
    vx = np.sqrt(lateral_acc_limit / abs_kappa)
    vx = np.clip(vx, min_speed, max_speed)

    for _ in range(max(0, smooth_passes)):
        vx = 0.25 * np.roll(vx, 1) + 0.5 * vx + 0.25 * np.roll(vx, -1)
        vx = np.clip(vx, min_speed, max_speed)

    return vx


def build_output_rows(
    xy_unique: np.ndarray,
    half_width_unique: np.ndarray,
    min_speed: float,
    max_speed: float,
    lateral_acc_limit: float,
    smooth_speed_passes: int,
) -> list[dict[str, float]]:
    s, psi, kappa = closed_geometry(xy_unique)
    vx = speed_profile(kappa, min_speed, max_speed, lateral_acc_limit, smooth_speed_passes)

    rows: list[dict[str, float]] = []
    for i in range(len(xy_unique)):
        rows.append(
            {
                "s": float(s[i]),
                "x": float(xy_unique[i, 0]),
                "y": float(xy_unique[i, 1]),
                "psi": float(psi[i]),
                "kappa": float(kappa[i]),
                "vx": float(vx[i]),
                "ax": 0.0,
                "left": float(half_width_unique[i]),
                "right": float(half_width_unique[i]),
            }
        )

    total_length = float(s[-1] + np.linalg.norm(xy_unique[0] - xy_unique[-1]))
    first = dict(rows[0])
    first["s"] = total_length
    rows.append(first)
    return rows


def apply_widths(rows: list[dict[str, float]], left: np.ndarray, right: np.ndarray) -> None:
    n_unique = len(rows) - 1
    if len(left) != n_unique or len(right) != n_unique:
        raise ValueError(
            f"Width arrays must have {n_unique} samples, got left={len(left)} right={len(right)}"
        )
    for i in range(n_unique):
        rows[i]["left"] = float(left[i])
        rows[i]["right"] = float(right[i])
    rows[-1]["left"] = rows[0]["left"]
    rows[-1]["right"] = rows[0]["right"]


def wall_threshold_from_map(map_img: np.ndarray, occupied_thresh: float) -> int:
    wall_thresh = int(255 * (1.0 - occupied_thresh))
    unique_vals, counts = np.unique(map_img, return_counts=True)
    grey_mask = (unique_vals > wall_thresh) & (unique_vals < 240)
    grey_pct = counts[grey_mask].sum() / map_img.size
    if grey_pct > 0.10:
        white_mask = unique_vals >= 240
        if white_mask.any():
            wall_thresh = int(unique_vals[white_mask].min()) - 5
    return wall_thresh


def build_gvd_centerline(
    map_path: Path,
    output_path: Path,
    centerline_points: int,
    direction: str,
    max_distance: float,
    min_speed: float,
    max_speed: float,
    lateral_acc_limit: float,
    smooth_speed_passes: int,
) -> list[dict[str, float]]:
    map_img, resolution, origin, occupied_thresh = load_planning_map(str(map_path))
    wall_thresh = wall_threshold_from_map(map_img, occupied_thresh)
    gvd_debug_path = output_path.with_name(f"{output_path.stem}_gvd.png")

    centerline, _ = compute_centerline_thinned_loop(
        map_img,
        resolution,
        origin,
        wall_thresh=wall_thresh,
        num_points=centerline_points,
        gvd_debug_path=str(gvd_debug_path),
    )

    winding = detect_winding_direction(centerline)
    requested_direction = winding if direction == "auto" else direction
    if requested_direction != winding:
        centerline = centerline[::-1].copy()

    w_right, w_left = measure_track_widths(
        centerline,
        map_img,
        resolution,
        origin,
        max_dist=max_distance,
        wall_thresh=wall_thresh,
    )

    rows = build_output_rows(
        centerline,
        0.5 * (np.asarray(w_left) + np.asarray(w_right)),
        min_speed=min_speed,
        max_speed=max_speed,
        lateral_acc_limit=lateral_acc_limit,
        smooth_speed_passes=smooth_speed_passes,
    )
    apply_widths(rows, np.asarray(w_left), np.asarray(w_right))

    print(f"  GVD direction: detected={winding}, output={requested_direction}")
    print(f"  GVD debug: {gvd_debug_path}")
    return rows


def refresh_wall_distances(rows: list[dict[str, float]], map_path: Path, max_distance: float) -> None:
    is_wall, resolution, origin_x, origin_y = load_map(str(map_path))
    map_height, map_width = is_wall.shape
    trajectory = [
        {
            "s": row["s"],
            "x": row["x"],
            "y": row["y"],
            "psi": row["psi"],
            "kappa": row["kappa"],
            "vx": row["vx"],
            "ax": row["ax"],
        }
        for row in rows
    ]
    distances = compute_wall_distances(
        trajectory,
        is_wall,
        origin_x,
        origin_y,
        resolution,
        map_height,
        map_width,
        max_distance=max_distance,
    )
    for row, (left, right) in zip(rows, distances):
        row["left"] = float(left)
        row["right"] = float(right)


def save_rows(path: Path, rows: list[dict[str, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        handle.write("# s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2,d_left_m,d_right_m\n")
        for row in rows:
            handle.write(
                f"{row['s']:.6f},{row['x']:.6f},{row['y']:.6f},"
                f"{row['psi']:.6f},{row['kappa']:.6f},{row['vx']:.6f},"
                f"{row['ax']:.6f},{row['left']:.6f},{row['right']:.6f}\n"
            )


def print_stats(rows: list[dict[str, float]], vehicle_half_width: float, safety_margin: float) -> None:
    left = np.array([row["left"] for row in rows], dtype=float)
    right = np.array([row["right"] for row in rows], dtype=float)
    effective_left = left - vehicle_half_width - safety_margin
    effective_right = right - vehicle_half_width - safety_margin
    print(f"  Points: {len(rows)}")
    print(f"  Length: {rows[-1]['s']:.3f} m")
    print(f"  Raw left/right min: {left.min():.3f} / {right.min():.3f} m")
    print(
        "  Effective left/right min: "
        f"{effective_left.min():.3f} / {effective_right.min():.3f} m"
    )
    print(
        "  Narrow effective points (<0.05 m): "
        f"{int(np.sum((effective_left < 0.05) | (effective_right < 0.05)))}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--method",
        choices=("gvd", "midpoint"),
        default="gvd",
        help="gvd extracts the map centerline; midpoint shifts an existing raceline between its CSV walls.",
    )
    parser.add_argument("--input", type=Path, help="Input 9-column raceline CSV for --method midpoint")
    parser.add_argument("--output", required=True, type=Path, help="Output centerline CSV")
    parser.add_argument("--map", type=Path, default=None, help="Map YAML for GVD extraction or fresh wall ray-cast")
    parser.add_argument("--centerline-points", type=int, default=500)
    parser.add_argument("--direction", choices=("auto", "cw", "ccw"), default="cw")
    parser.add_argument("--max-distance", type=float, default=5.0, help="Maximum ray-cast distance [m]")
    parser.add_argument("--min-speed", type=float, default=1.5, help="Minimum generated vx_ref [m/s]")
    parser.add_argument("--max-speed", type=float, default=3.0, help="Maximum generated vx_ref [m/s]")
    parser.add_argument("--lateral-acc-limit", type=float, default=3.0, help="Conservative lateral acceleration cap [m/s^2]")
    parser.add_argument("--smooth-speed-passes", type=int, default=4, help="Circular smoothing passes for vx_ref")
    parser.add_argument("--vehicle-half-width", type=float, default=0.155)
    parser.add_argument("--safety-margin", type=float, default=0.010)
    args = parser.parse_args()

    if args.method == "gvd":
        if args.map is None:
            raise ValueError("--map is required for --method gvd")
        rows = build_gvd_centerline(
            args.map,
            args.output,
            centerline_points=args.centerline_points,
            direction=args.direction,
            max_distance=args.max_distance,
            min_speed=args.min_speed,
            max_speed=args.max_speed,
            lateral_acc_limit=args.lateral_acc_limit,
            smooth_speed_passes=args.smooth_speed_passes,
        )
    else:
        if args.input is None:
            raise ValueError("--input is required for --method midpoint")
        points = load_raceline(args.input)
        duplicate_endpoint = has_duplicate_endpoint(points)
        working_points = points[:-1] if duplicate_endpoint else points

        xy, half_width = compute_center_xy(working_points)
        rows = build_output_rows(
            xy,
            half_width,
            min_speed=args.min_speed,
            max_speed=args.max_speed,
            lateral_acc_limit=args.lateral_acc_limit,
            smooth_speed_passes=args.smooth_speed_passes,
        )

        if args.map is not None:
            refresh_wall_distances(rows, args.map, args.max_distance)

    save_rows(args.output, rows)
    print(f"Generated centerline: {args.output}")
    print_stats(rows, args.vehicle_half_width, args.safety_margin)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
