#!/usr/bin/env python3
"""Smooth and resample a closed MPCC centerline CSV.

The script keeps the input loop shape, fits a periodic spline to x/y, samples
uniformly in arc length, recomputes psi/kappa from spline derivatives, and
optionally ray-casts fresh wall distances from a map YAML.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
from scipy.interpolate import CubicSpline, interp1d, splprep, splev


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from compute_wall_distances import compute_wall_distances, load_map  # noqa: E402


COLS = ("s", "x", "y", "psi", "kappa", "vx", "ax", "left", "right")


def load_rows(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.reader(handle)
        for row in reader:
            if not row or row[0].strip().startswith("#"):
                continue
            values = [float(value) for value in row[:9]]
            rows.append(dict(zip(COLS, values)))
    if len(rows) < 8:
        raise ValueError(f"Need at least 8 points in {path}")
    return rows


def has_duplicate_endpoint(rows: list[dict[str, float]]) -> bool:
    first = rows[0]
    last = rows[-1]
    return math.hypot(first["x"] - last["x"], first["y"] - last["y"]) < 1.0e-5


def cumulative_closed_s(xy: np.ndarray) -> tuple[np.ndarray, float]:
    seg = np.roll(xy, -1, axis=0) - xy
    ds = np.linalg.norm(seg, axis=1)
    total = float(ds.sum())
    s = np.zeros(len(xy), dtype=float)
    s[1:] = np.cumsum(ds[:-1])
    return s, total


def fit_smooth_loop(xy: np.ndarray, smooth: float, samples: int) -> tuple[np.ndarray, np.ndarray]:
    _, total_input = cumulative_closed_s(xy)
    # splprep's smoothing factor is in squared map units. Scale by point count
    # so command-line values remain intuitive across resolutions.
    tck, _ = splprep([xy[:, 0], xy[:, 1]], s=smooth * len(xy), per=True, k=3)

    dense_u = np.linspace(0.0, 1.0, max(4000, samples * 12), endpoint=False)
    dense = np.column_stack(splev(dense_u, tck, der=0))
    dense_closed = np.vstack([dense, dense[0]])
    dense_seg = np.linalg.norm(np.diff(dense_closed, axis=0), axis=1)
    dense_s = np.zeros(len(dense_closed), dtype=float)
    dense_s[1:] = np.cumsum(dense_seg)
    total = float(dense_s[-1])

    target_s = np.linspace(0.0, total, samples, endpoint=False)
    u_of_s = interp1d(dense_s, np.r_[dense_u, 1.0], kind="linear")
    u = np.asarray(u_of_s(target_s), dtype=float)

    xy_new = np.column_stack(splev(u, tck, der=0))
    dx, dy = splev(u, tck, der=1)
    ddx, ddy = splev(u, tck, der=2)
    dx = np.asarray(dx, dtype=float)
    dy = np.asarray(dy, dtype=float)
    ddx = np.asarray(ddx, dtype=float)
    ddy = np.asarray(ddy, dtype=float)

    psi = np.arctan2(dy, dx)
    denom = np.maximum((dx * dx + dy * dy) ** 1.5, 1.0e-9)
    kappa = ((dx * ddy) - (dy * ddx)) / denom

    # Keep the output length close to the original so progress comparisons stay
    # meaningful. Rescale s, not geometry.
    s_new = target_s * (total_input / total)
    out = np.column_stack([s_new, xy_new[:, 0], xy_new[:, 1], psi, kappa])
    return out, tck


def interpolate_profile(rows: list[dict[str, float]], s_new: np.ndarray, total: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    unique = rows[:-1] if has_duplicate_endpoint(rows) else rows
    s_old = np.array([row["s"] for row in unique], dtype=float)
    vx_old = np.array([row["vx"] for row in unique], dtype=float)
    left_old = np.array([row["left"] for row in unique], dtype=float)
    right_old = np.array([row["right"] for row in unique], dtype=float)

    s_ext = np.r_[s_old, total]
    vx_ext = np.r_[vx_old, vx_old[0]]
    left_ext = np.r_[left_old, left_old[0]]
    right_ext = np.r_[right_old, right_old[0]]
    return (
        np.interp(s_new, s_ext, vx_ext),
        np.interp(s_new, s_ext, left_ext),
        np.interp(s_new, s_ext, right_ext),
    )


def refresh_widths(rows: list[dict[str, float]], map_path: Path, max_distance: float) -> None:
    is_wall, resolution, origin_x, origin_y = load_map(str(map_path))
    h, w = is_wall.shape
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
        h,
        w,
        max_distance=max_distance,
    )
    for row, (left, right) in zip(rows, distances):
        row["left"] = float(left)
        row["right"] = float(right)


def _periodic_running_median(values: np.ndarray, window: int) -> np.ndarray:
    if window <= 1:
        return values.copy()
    if window % 2 == 0:
        raise ValueError("width clamp window must be odd")

    half = window // 2
    out = np.empty_like(values)
    count = len(values)
    for idx in range(count):
        sample = [values[(idx + off) % count] for off in range(-half, half + 1)]
        out[idx] = float(np.median(sample))
    return out


def suppress_width_spikes(
    rows: list[dict[str, float]],
    window: int,
    spike_margin: float | None,
    max_side_width: float | None,
) -> None:
    if window <= 1 or not rows:
        return

    body = rows[:-1] if len(rows) > 1 else rows
    left = np.array([row["left"] for row in body], dtype=float)
    right = np.array([row["right"] for row in body], dtype=float)

    left_med = _periodic_running_median(left, window)
    right_med = _periodic_running_median(right, window)

    if spike_margin is not None:
        left = np.minimum(left, left_med + spike_margin)
        right = np.minimum(right, right_med + spike_margin)

    if max_side_width is not None:
        left = np.minimum(left, max_side_width)
        right = np.minimum(right, max_side_width)

    # Blend once with neighbors after clipping so corridor changes stay smooth.
    left = 0.2 * np.roll(left, 1) + 0.6 * left + 0.2 * np.roll(left, -1)
    right = 0.2 * np.roll(right, 1) + 0.6 * right + 0.2 * np.roll(right, -1)

    for row, left_value, right_value in zip(body, left, right):
        row["left"] = float(left_value)
        row["right"] = float(right_value)

    if len(rows) > len(body):
        rows[-1]["left"] = float(rows[0]["left"])
        rows[-1]["right"] = float(rows[0]["right"])


def render_overlay_png(rows: list[dict[str, float]], map_path: Path, output_path: Path) -> None:
    is_wall, resolution, origin_x, origin_y = load_map(str(map_path))
    height, width = is_wall.shape

    canvas = np.empty((height, width, 3), dtype=np.uint8)
    canvas[:, :] = (245, 245, 245)
    canvas[is_wall] = (35, 35, 35)

    image = Image.fromarray(canvas, mode="RGB")
    draw = ImageDraw.Draw(image)

    points = []
    for row in rows[:-1]:
        col = int(round((row["x"] - origin_x) / resolution))
        py = int(round(height - 1 - ((row["y"] - origin_y) / resolution)))
        points.append((col, py))

    if len(points) < 2:
        raise ValueError("Need at least two points to render an overlay PNG")

    draw.line(points + [points[0]], fill=(20, 80, 235), width=4)
    marker_indices = (0, len(points) // 4, len(points) // 2, (3 * len(points)) // 4)
    for idx in marker_indices:
        x, y = points[idx]
        radius = 5 if idx else 6
        draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=(40, 180, 40))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(output_path)


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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--map", type=Path)
    parser.add_argument("--samples", type=int, default=500)
    parser.add_argument("--smooth", type=float, default=0.004)
    parser.add_argument("--max-distance", type=float, default=5.0)
    parser.add_argument("--min-speed", type=float, default=1.2)
    parser.add_argument("--max-speed", type=float, default=3.0)
    parser.add_argument("--lateral-acc-limit", type=float, default=3.0)
    parser.add_argument("--width-clamp-window", type=int, default=0)
    parser.add_argument("--width-spike-margin", type=float)
    parser.add_argument("--max-side-width", type=float)
    args = parser.parse_args()

    source_rows = load_rows(args.input)
    unique_rows = source_rows[:-1] if has_duplicate_endpoint(source_rows) else source_rows
    xy = np.array([[row["x"], row["y"]] for row in unique_rows], dtype=float)
    s_old, total = cumulative_closed_s(xy)

    smooth_data, _ = fit_smooth_loop(xy, args.smooth, args.samples)
    s_new = smooth_data[:, 0]
    vx_interp, left_interp, right_interp = interpolate_profile(source_rows, s_new, total)

    kappa_abs = np.maximum(np.abs(smooth_data[:, 4]), 1.0e-6)
    vx_curve = np.sqrt(args.lateral_acc_limit / kappa_abs)
    vx = np.minimum(vx_interp, vx_curve)
    vx = np.clip(vx, args.min_speed, args.max_speed)
    for _ in range(8):
        vx = 0.25 * np.roll(vx, 1) + 0.5 * vx + 0.25 * np.roll(vx, -1)
        vx = np.clip(vx, args.min_speed, args.max_speed)

    rows: list[dict[str, float]] = []
    for i in range(args.samples):
        rows.append(
            {
                "s": float(smooth_data[i, 0]),
                "x": float(smooth_data[i, 1]),
                "y": float(smooth_data[i, 2]),
                "psi": float(math.atan2(math.sin(smooth_data[i, 3]), math.cos(smooth_data[i, 3]))),
                "kappa": float(smooth_data[i, 4]),
                "vx": float(vx[i]),
                "ax": 0.0,
                "left": float(left_interp[i]),
                "right": float(right_interp[i]),
            }
        )
    endpoint = dict(rows[0])
    endpoint["s"] = total
    rows.append(endpoint)

    if args.map is not None:
        refresh_widths(rows, args.map, args.max_distance)
        suppress_width_spikes(
            rows,
            args.width_clamp_window,
            args.width_spike_margin,
            args.max_side_width,
        )

    save_rows(args.output, rows)

    png_path = None
    if args.map is not None:
        png_path = args.output.with_suffix(".png")
        render_overlay_png(rows, args.map, png_path)

    kappas = np.array([abs(row["kappa"]) for row in rows[:-1]], dtype=float)
    left = np.array([row["left"] for row in rows[:-1]], dtype=float)
    right = np.array([row["right"] for row in rows[:-1]], dtype=float)
    print(f"Smoothed centerline: {args.output}")
    if png_path is not None:
        print(f"  overlay png={png_path}")
    print(f"  points={len(rows)} length={rows[-1]['s']:.3f} m")
    print(f"  max|kappa|={kappas.max():.3f} p95|kappa|={np.quantile(kappas, 0.95):.3f}")
    print(f"  min left/right={left.min():.3f}/{right.min():.3f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
