#!/usr/bin/env python3
"""Create a scan-matching point cloud from the MCAP occupancy-grid map."""
from __future__ import annotations

import argparse
from pathlib import Path
import numpy as np
from scipy.ndimage import binary_erosion
import matplotlib.pyplot as plt


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("map_npz", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--occupied-threshold", type=int, default=50)
    parser.add_argument("--stride", type=int, default=1, help="Keep every Nth wall point after boundary extraction.")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    d = np.load(args.map_npz, allow_pickle=False)
    grid = d["grid"]
    resolution = float(d["resolution"])
    origin_x, origin_y, origin_yaw = float(d["origin_x"]), float(d["origin_y"]), float(d["origin_yaw"])
    occupied = grid >= args.occupied_threshold
    # Use occupied-cell boundaries. This avoids filling a wall interior with
    # correspondence targets and is more stable than matching all occupied cells.
    boundary = occupied & ~binary_erosion(occupied, structure=np.ones((3, 3), dtype=bool), border_value=0)
    rows, cols = np.nonzero(boundary)
    x_local = (cols.astype(float) + 0.5) * resolution
    y_local = (rows.astype(float) + 0.5) * resolution
    c, s = np.cos(origin_yaw), np.sin(origin_yaw)
    x = origin_x + c * x_local - s * y_local
    y = origin_y + s * x_local + c * y_local
    points = np.column_stack([x, y])
    if args.stride > 1:
        points = points[::args.stride]
    np.savez_compressed(args.output / "icp_map_points.npz", points=points.astype(np.float32), resolution=resolution)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.scatter(points[:, 0], points[:, 1], s=0.2)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("map x [m]")
    ax.set_ylabel("map y [m]")
    ax.set_title(f"ICP map: {len(points):,} occupied-boundary points")
    fig.tight_layout()
    fig.savefig(args.output / "icp_map_preview.png", dpi=200)
    plt.close(fig)
    print(f"Wrote {len(points):,} ICP map points to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
