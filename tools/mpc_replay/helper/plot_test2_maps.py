#!/usr/bin/env python3
"""
Test 2 spatial maps from the cost-replay CSVs.

Inputs: baseline + low-velocity-weight CSVs from
        tools/mpc_replay/run_test2_low_velocity_weight.sh

Lap selection (--lap):
  N (integer)   plot a single race lap; default 2 (first lap after warmup).
                Lap 1 is the warm-up lap and is dropped.
  average       use the first kept lap as a reference path, snap every point
                from every kept race lap to its nearest reference point, and
                average the per-field values per reference point. The result
                is a single continuous lap whose color reflects the average
                behavior across all 10 race laps.

The plots are drawn as connected polylines (LineCollection) so the path
looks continuous even at small --cell-size values.

Outputs (in <out-dir>/plots/):
    map1_v_h_end.svg         baseline vs low-vel side-by-side, colored by
                           planned end-of-horizon velocity (shared colormap)
    map2_cost_components.svg 2x3 grid: per-term cost on the averaged track
                           (low-vel run; one panel per term)
    map4_out_accel.svg       baseline vs low-vel side-by-side, colored by
                           commanded acceleration
"""

import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection

plt.rcParams.update({
    "font.size": 18,
    "axes.titlesize": 20,
    "axes.labelsize": 18,
    "xtick.labelsize": 16,
    "ytick.labelsize": 16,
    "legend.fontsize": 16,
})

COST_TERMS = [
    "J_lat", "J_heading", "J_vel", "J_lat_vel", "J_yaw_rate",
    "J_delta_actual", "J_drate_prev", "J_accel_prev",
    "J_steer_in", "J_accel_in",
]


def load_csv(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"empty CSV: {path}")
    return rows


def to_arrays(rows):
    skip = {"idx", "stamp_ns", "status", "iters"}
    return {k: np.array([float(r[k]) for r in rows])
            for k in rows[0].keys() if k not in skip}


def drop_warmup(data):
    mask = (data["pos_x"] ** 2 + data["pos_y"] ** 2) > 1e-4
    return {k: v[mask] for k, v in data.items()}


def detect_laps(pos_x, pos_y, min_radius=2.0, min_lap_length=25.0):
    """Detect lap boundaries by tracking returns near the start position.

    A lap ends when we re-enter `min_radius` of the start AND have travelled
    at least `min_lap_length` along the path since the previous boundary.
    `min_lap_length` must exceed any incidental passes near the start (a
    full race lap on this track is ~37 m, so 25 m comfortably suppresses
    startup wiggle while accepting all true laps).
    Returns a list of (start_idx, end_idx) half-open slices.
    """
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


def slice_lap(data, lap_slice):
    s, e = lap_slice
    return {k: v[s:e] for k, v in data.items()}


def concat_laps(data, lap_slices):
    return {k: np.concatenate([v[s:e] for s, e in lap_slices])
            for k, v in data.items()}


def subsample_path_by_spacing(xy, min_spacing):
    """Greedy walk along the path keeping points at least `min_spacing` apart.

    Preserves trajectory order so the result is still a connected polyline.
    """
    keep = [0]
    last = xy[0]
    min_sq = min_spacing * min_spacing
    for i in range(1, len(xy)):
        dx = xy[i, 0] - last[0]
        dy = xy[i, 1] - last[1]
        if dx * dx + dy * dy >= min_sq:
            keep.append(i)
            last = xy[i]
    return np.asarray(keep, dtype=np.int64)


def average_along_reference_path(all_data, ref_xy):
    """Snap every (pos_x, pos_y) in all_data to its nearest reference point and
    average each field per reference point.

    Returns a dict whose pos_x/pos_y are exactly the reference path so the
    result can be plotted as a connected line.
    """
    try:
        from scipy.spatial import cKDTree
        tree = cKDTree(ref_xy)
        _, nearest = tree.query(np.column_stack([all_data["pos_x"],
                                                 all_data["pos_y"]]))
    except ImportError:
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


def colored_line(ax, x, y, values, *, cmap, vmin, vmax, linewidth=5.0):
    """Draw (x, y) as a connected polyline whose per-segment color encodes `values`."""
    x = np.asarray(x)
    y = np.asarray(y)
    values = np.asarray(values)
    points = np.column_stack([x, y]).reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    seg_values = 0.5 * (values[:-1] + values[1:])
    lc = LineCollection(segments, cmap=cmap, linewidth=linewidth)
    lc.set_array(seg_values)
    lc.set_clim(vmin, vmax)
    ax.add_collection(lc)
    pad = 0.5
    ax.set_xlim(float(np.nanmin(x)) - pad, float(np.nanmax(x)) + pad)
    ax.set_ylim(float(np.nanmin(y)) - pad, float(np.nanmax(y)) + pad)
    return lc


def plot_v_h_end_map(low_vel, base, out_path):
    fig, axes = plt.subplots(1, 2, figsize=(18, 9), sharex=True, sharey=True)
    vmin = float(min(np.nanmin(base["v_h_end"]), np.nanmin(low_vel["v_h_end"])))
    vmax = float(max(np.nanmax(base["v_h_end"]), np.nanmax(low_vel["v_h_end"])))
    for ax, data, title in [(axes[0], base, "Baseline weights"),
                            (axes[1], low_vel, "Low velocity weight")]:
        lc = colored_line(ax, data["pos_x"], data["pos_y"], data["v_h_end"],
                          cmap="RdYlGn", vmin=vmin, vmax=vmax)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_title(title)
        ax.set_aspect("equal")
        plt.colorbar(lc, ax=ax, label="planned v at horizon end [m/s]")
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close(fig)


# Cost terms to display in the per-term spatial grid.
COMPONENT_GRID_TERMS = [
    ("J_lat",       "Lateral tracking"),
    ("J_heading",   "Heading tracking"),
    ("J_lat_vel",   "Lateral velocity tracking"),
    ("J_yaw_rate",  "Yaw-rate tracking"),
    ("J_accel_in",  "Accel command effort"),
    ("J_steer_in",  "Steering command effort"),
]


def plot_cost_components_grid(low_vel, out_path):
    fig, axes = plt.subplots(2, 3, figsize=(20, 12), sharex=True, sharey=True)
    axes = axes.flatten()
    for ax, (term, descr) in zip(axes, COMPONENT_GRID_TERMS):
        values = low_vel[term]
        valid = np.isfinite(values)
        if valid.any():
            vmin = max(0.0, float(np.percentile(values[valid], 1)))
            vmax = float(np.percentile(values[valid], 99))
        else:
            vmin, vmax = 0.0, 1.0
        if vmax <= vmin:
            vmax = vmin + 1e-6
        lc = colored_line(ax, low_vel["pos_x"], low_vel["pos_y"], values,
                          cmap="viridis", vmin=vmin, vmax=vmax)
        ax.set_title(f"{descr}")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_aspect("equal")
        plt.colorbar(lc, ax=ax)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_out_accel_map(low_vel, base, out_path):
    fig, axes = plt.subplots(1, 2, figsize=(18, 9), sharex=True, sharey=True)
    a_lim = 7.5
    for ax, data, title in [(axes[0], base, "Baseline weights"),
                            (axes[1], low_vel, "Low velocity weight")]:
        lc = colored_line(ax, data["pos_x"], data["pos_y"], data["out_accel"],
                          cmap="RdBu_r", vmin=-a_lim, vmax=a_lim)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_title(title)
        ax.set_aspect("equal")
        plt.colorbar(lc, ax=ax, label="commanded a [m/s^2]")
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close(fig)


def parse_lap_arg(value):
    if value.lower() in ("average", "avg", "all", "mean"):
        return "average"
    try:
        return int(value)
    except ValueError:
        raise argparse.ArgumentTypeError(
            f"--lap must be an integer or 'average', got: {value}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", required=True,
                    help="Test 2 output dir from run_test2_low_velocity_weight.sh")
    ap.add_argument("--plot-dir", default=None,
                    help="Override plot output dir (default: <out-dir>/plots)")
    ap.add_argument("--lap", type=parse_lap_arg, default=2,
                    help="Integer lap (default 2), or 'average' to average across kept race laps")
    ap.add_argument("--max-lap", type=int, default=11,
                    help="Discard laps strictly after this (default 11 = 1 warmup + 10 race)")
    ap.add_argument("--cell-size", type=float, default=0.1,
                    help="Min spacing (meters) between reference points in 'average' mode "
                         "(default 0.1). Smaller = finer line.")
    ap.add_argument("--list-laps", action="store_true",
                    help="Print detected lap boundaries and exit")
    args = ap.parse_args()

    out_dir = Path(args.out_dir)
    plot_dir = Path(args.plot_dir) if args.plot_dir else out_dir / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)

    base = drop_warmup(to_arrays(load_csv(out_dir / "cost_baseline.csv")))
    low_vel = drop_warmup(to_arrays(load_csv(out_dir / "cost_low_velocity_weight.csv")))

    laps = detect_laps(base["pos_x"], base["pos_y"])
    print(f"Detected {len(laps)} laps (post-warmup rows):")
    for i, (s, e) in enumerate(laps, start=1):
        kept = " [keep]" if 2 <= i <= args.max_lap else " [drop]"
        print(f"  lap {i:>2}: rows [{s:>6}, {e:>6})  length={e-s}{kept}")

    if args.list_laps:
        return

    if args.lap == "average":
        kept = [laps[i - 1] for i in range(2, args.max_lap + 1) if i <= len(laps)]
        if not kept:
            raise SystemExit("no laps in keep range")

        # Reference path = first kept lap's trajectory, sub-sampled by --cell-size.
        ref_s, ref_e = kept[0]
        ref_xy_full = np.column_stack([base["pos_x"][ref_s:ref_e],
                                       base["pos_y"][ref_s:ref_e]])
        keep_idx = subsample_path_by_spacing(ref_xy_full, args.cell_size)
        ref_xy = ref_xy_full[keep_idx]

        base_sel = average_along_reference_path(concat_laps(base, kept), ref_xy)
        low_vel_sel = average_along_reference_path(concat_laps(low_vel, kept), ref_xy)
        rows_in = sum(e - s for s, e in kept)
        print(f"Averaging across laps 2..{2 + len(kept) - 1}  "
              f"({rows_in} rows -> {len(ref_xy)} reference points, "
              f"min spacing {args.cell_size} m)")
    else:
        if args.lap < 1 or args.lap > len(laps):
            raise SystemExit(f"--lap {args.lap} out of range (detected {len(laps)} laps)")
        if args.lap == 1:
            print("WARNING: lap 1 is the warmup lap (usually discarded)")
        if args.lap > args.max_lap:
            print(f"WARNING: lap {args.lap} > --max-lap {args.max_lap}")
        sel = laps[args.lap - 1]
        base_sel = slice_lap(base, sel)
        low_vel_sel = slice_lap(low_vel, sel)
        print(f"Selected lap {args.lap}: {sel[1]-sel[0]} rows")

    plot_v_h_end_map(low_vel_sel, base_sel, plot_dir / "map1_v_h_end.svg")
    plot_cost_components_grid(low_vel_sel, plot_dir / "map2_cost_components.svg")
    plot_out_accel_map(low_vel_sel, base_sel, plot_dir / "map4_out_accel.svg")

    print(f"Wrote plots to {plot_dir}")


if __name__ == "__main__":
    main()
