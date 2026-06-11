#!/usr/bin/env python3
"""Create a baseline tracking map SVG from a state_replay.csv.

Usage:
  python3 plot_baseline_tracking_from_state.py --state-csv PATH --out-dir DIR
"""
import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from mpl_toolkits.axes_grid1 import make_axes_locatable

plt.rcParams.update({
    "font.size": 18,
    "axes.titlesize": 20,
    "axes.labelsize": 18,
    "xtick.labelsize": 16,
    "ytick.labelsize": 16,
    "legend.fontsize": 16,
})


def colored_line(ax, x, y, values, *, cmap="viridis", vmin=None, vmax=None, linewidth=5.0):
    x = np.asarray(x)
    y = np.asarray(y)
    values = np.asarray(values)
    points = np.column_stack([x, y]).reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    seg_values = 0.5 * (values[:-1] + values[1:])
    lc = LineCollection(segments, cmap=cmap, linewidth=linewidth)
    lc.set_array(seg_values)
    if vmin is not None and vmax is not None:
        lc.set_clim(vmin, vmax)
    ax.add_collection(lc)
    pad = 0.5
    ax.set_xlim(float(np.nanmin(x)) - pad, float(np.nanmax(x)) + pad)
    ax.set_ylim(float(np.nanmin(y)) - pad, float(np.nanmax(y)) + pad)
    return lc


def add_matched_colorbar(fig, ax, mappable, label):
    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="4%", pad=0.10)
    cbar = fig.colorbar(mappable, cax=cax)
    cbar.set_label(label)
    return cbar


def load_state_csv(path: Path):
    with path.open() as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"empty CSV: {path}")
    # Support multiple possible field names
    def get_field(r, keys):
        for k in keys:
            if k in r and r[k] != "":
                return r[k]
        return "0"

    # Fixed-point scale used in state_replay exports
    scale = 262144.0

    # Prefer already-scaled `x`/`y` if present; otherwise convert fixed-point `x_fp`/`y_fp`.
    pos_x = []
    pos_y = []
    for r in rows:
        if r.get("x"):
            pos_x.append(float(r["x"]))
        else:
            pos_x.append(int(r.get("x_fp", 0)) / scale)
        if r.get("y"):
            pos_y.append(float(r["y"]))
        else:
            pos_y.append(int(r.get("y_fp", 0)) / scale)
    pos_x = np.array(pos_x, dtype=float)
    pos_y = np.array(pos_y, dtype=float)

    def int_field(name, default=0):
        return np.array([int(get_field(r, [name]) or default) for r in rows], dtype=np.int64)

    e_y = np.abs(int_field("e_y_fp").astype(np.float64) / scale) if "e_y_fp" in rows[0] else np.zeros_like(pos_x)
    e_psi = np.abs(int_field("e_psi_fp").astype(np.float64) / scale) if "e_psi_fp" in rows[0] else np.zeros_like(pos_x)

    if "velocity_fp" in rows[0]:
        vel_fp = int_field("velocity_fp")
        ref_vx0_fp = int_field("ref_vx_fp_0") if "ref_vx_fp_0" in rows[0] else np.zeros_like(vel_fp)
        e_vx = (vel_fp.astype(np.float64) - ref_vx0_fp.astype(np.float64)) / scale
    else:
        e_vx = np.zeros_like(pos_x)

    return {"pos_x": pos_x, "pos_y": pos_y, "e_y": e_y, "e_psi": e_psi, "e_vx": e_vx}


def drop_warmup(data):
    mask = (data["pos_x"] ** 2 + data["pos_y"] ** 2) > 1e-4
    return {k: v[mask] for k, v in data.items()}


def detect_laps(pos_x, pos_y, min_radius=2.0, min_lap_length=25.0):
    n = len(pos_x)
    if n < 2:
        return [(0, n)]
    start_x, start_y = pos_x[0], pos_y[0]
    dist_from_start = np.sqrt((pos_x - start_x) ** 2 + (pos_y - start_y) ** 2)
    seg_len = np.sqrt(np.diff(pos_x) ** 2 + np.diff(pos_y) ** 2)
    cum = np.concatenate(([0.0], np.cumsum(seg_len)))

    lap_starts = [0]
    last_path = 0.0
    in_start_zone = True
    for i in range(1, n):
        inside = dist_from_start[i] < min_radius
        if inside and not in_start_zone and (cum[i] - last_path) > min_lap_length:
            lap_starts.append(i)
            last_path = cum[i]
        in_start_zone = inside

    lap_starts.append(n)
    return [(lap_starts[i], lap_starts[i + 1]) for i in range(len(lap_starts) - 1)]


def subsample_path_by_spacing(xy, min_spacing):
    if xy.shape[0] == 0:
        return np.asarray([], dtype=np.int64)
    keep = [0]
    last = xy[0]
    min_sq = float(min_spacing) * float(min_spacing)
    for i in range(1, len(xy)):
        dx = xy[i, 0] - last[0]
        dy = xy[i, 1] - last[1]
        if dx * dx + dy * dy >= min_sq:
            keep.append(i)
            last = xy[i]
    return np.asarray(keep, dtype=np.int64)


def concat_laps(data, lap_slices):
    return {k: np.concatenate([v[s:e] for s, e in lap_slices]) for k, v in data.items()}


def average_along_reference_path(all_data, ref_xy):
    try:
        from scipy.spatial import cKDTree
        tree = cKDTree(ref_xy)
        _, nearest = tree.query(np.column_stack([all_data["pos_x"], all_data["pos_y"]]))
    except Exception:
        data_xy = np.column_stack([all_data["pos_x"], all_data["pos_y"]])
        nearest = np.empty(len(data_xy), dtype=np.int64)
        chunk = 1000
        for i in range(0, len(data_xy), chunk):
            end = min(i + chunk, len(data_xy))
            d2 = ((data_xy[i:end, None, :] - ref_xy[None, :, :]) ** 2).sum(axis=2)
            nearest[i:end] = d2.argmin(axis=1)

    n_ref = len(ref_xy)
    counts = np.bincount(nearest, minlength=n_ref).astype(np.float64)
    safe_counts = np.where(counts > 0, counts, 1.0)
    out = {"pos_x": ref_xy[:, 0].copy(), "pos_y": ref_xy[:, 1].copy()}
    for field, values in all_data.items():
        if field in ("pos_x", "pos_y"):
            continue
        sums = np.bincount(nearest, weights=values, minlength=n_ref)
        out[field] = np.where(counts > 0, sums / safe_counts, np.nan)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--state-csv", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--cell-size", type=float, default=0.1,
                    help="Min spacing (m) when subsampling reference path")
    ap.add_argument("--max-lap", type=int, default=11,
                    help="Discard laps strictly after this (default 11)")
    args = ap.parse_args()

    state_csv = Path(args.state_csv)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    data = load_state_csv(state_csv)
    data = drop_warmup(data)

    laps = detect_laps(data["pos_x"], data["pos_y"])
    kept = [laps[i - 1] for i in range(2, args.max_lap + 1) if i <= len(laps)]
    if kept:
        ref_s, ref_e = kept[0]
        ref_xy_full = np.column_stack([data["pos_x"][ref_s:ref_e], data["pos_y"][ref_s:ref_e]])
        keep_idx = subsample_path_by_spacing(ref_xy_full, args.cell_size)
        ref_xy = ref_xy_full[keep_idx]
        sel = concat_laps(data, kept)
        avg = average_along_reference_path(sel, ref_xy)
    else:
        ref_xy_full = np.column_stack([data["pos_x"], data["pos_y"]])
        keep_idx = subsample_path_by_spacing(ref_xy_full, args.cell_size)
        ref_xy = ref_xy_full[keep_idx]
        avg = average_along_reference_path(data, ref_xy)

    x = avg["pos_x"]
    y = avg["pos_y"]
    e_y = avg.get("e_y", np.full_like(x, np.nan))
    e_psi = avg.get("e_psi", np.full_like(x, np.nan))
    e_vx = avg.get("e_vx", np.full_like(x, np.nan))

    fig, axes = plt.subplots(1, 3, figsize=(24, 9), sharex=True, sharey=True)

    # Panel 1: |e_y| lateral error
    ax = axes[0]
    vmin = 0.0
    vmax = float(np.percentile(e_y[np.isfinite(e_y)], 99)) if np.any(np.isfinite(e_y)) else float(np.nanmax(e_y))
    lc1 = colored_line(ax, x, y, e_y, cmap="viridis", vmin=vmin, vmax=vmax, linewidth=6.0)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal")
    add_matched_colorbar(fig, ax, lc1, "|e_y| [m]")

    # Panel 2: |e_psi| heading error
    ax = axes[1]
    vmin2 = 0.0
    vmax2 = float(np.percentile(e_psi[np.isfinite(e_psi)], 99)) if np.any(np.isfinite(e_psi)) else float(np.nanmax(e_psi))
    lc2 = colored_line(ax, x, y, e_psi, cmap="viridis", vmin=vmin2, vmax=vmax2, linewidth=6.0)
    ax.set_xlabel("x [m]")
    ax.set_aspect("equal")
    add_matched_colorbar(fig, ax, lc2, "|e_psi| [rad]")

    # Panel 3: e_vx velocity error (signed)
    ax = axes[2]
    m = np.nanmax(np.abs(e_vx)) if np.any(np.isfinite(e_vx)) else 1.0
    lc3 = colored_line(ax, x, y, e_vx, cmap="RdBu_r", vmin=-m, vmax=m, linewidth=6.0)
    ax.set_xlabel("x [m]")
    ax.set_aspect("equal")
    add_matched_colorbar(fig, ax, lc3, "e_vx [m/s]")

    out_path = out_dir / "baseline_tracking_maps.svg"
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)
    print(f"Wrote: {out_path}")


if __name__ == "__main__":
    main()
