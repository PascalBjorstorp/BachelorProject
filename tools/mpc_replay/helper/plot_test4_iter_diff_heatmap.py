#!/usr/bin/env python3
"""
Test 4 mismatch plot from the cost-replay CSVs.

Same connected-polyline style as plot_test2_maps.py:
the warm baseline run defines an ordered reference path, both runs are
snapped and averaged onto that path, and the figures are drawn as
LineCollection curves instead of point clouds.

Outputs:
        test4_iter_diff_heatmap.svg     cold - warm along the full shared path
        test4_iter_diff_heatmap_zoom.svg cold - warm zoomed to selected window
    test4_iter_per_run_heatmaps.svg warm vs cold mean iterations along the
                                  shared reference path
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


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"empty CSV: {path}")
    return {
        "pos_x": np.array([float(r["pos_x"]) for r in rows], dtype=float),
        "pos_y": np.array([float(r["pos_y"]) for r in rows], dtype=float),
        "iterations": np.array([float(r["iterations"]) for r in rows], dtype=float),
    }


def drop_warmup(data):
    mask = (data["pos_x"] ** 2 + data["pos_y"] ** 2) > 1e-4
    return {k: v[mask] for k, v in data.items()}


def subsample_path_by_spacing(xy, min_spacing):
    """Greedy walk along the path keeping points at least `min_spacing` apart."""
    if xy.shape[0] == 0:
        return np.asarray([], dtype=np.int64)
    keep = [0]
    last = xy[0]
    min_sq = float(min_spacing) * float(min_spacing)
    for i in range(1, xy.shape[0]):
        dx = xy[i, 0] - last[0]
        dy = xy[i, 1] - last[1]
        if dx * dx + dy * dy >= min_sq:
            keep.append(i)
            last = xy[i]
    return np.asarray(keep, dtype=np.int64)


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


def slice_lap(data, lap_slice):
    s, e = lap_slice
    return {k: v[s:e] for k, v in data.items()}


def concat_laps(data, lap_slices):
    return {k: np.concatenate([v[s:e] for s, e in lap_slices]) for k, v in data.items()}


def average_along_reference_path(all_data, ref_xy):
    """Snap all points to the nearest reference path point and average fields."""
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
    counts = np.bincount(nearest, minlength=n_ref).astype(np.int64)
    out = {"pos_x": ref_xy[:, 0].copy(), "pos_y": ref_xy[:, 1].copy()}
    for field, values in all_data.items():
        if field in ("pos_x", "pos_y"):
            continue
        sums = np.bincount(nearest, weights=values, minlength=n_ref)
        out[field] = np.where(counts > 0, sums / np.maximum(counts, 1), np.nan)
    return out, counts


def colored_line(ax, x, y, values, *, cmap, vmin, vmax, linewidth=5.0):
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


def crop_xy_values(xy, values, xlim=None, ylim=None):
    if xlim is None or ylim is None:
        return xy, values
    mask = (
        (xy[:, 0] >= xlim[0]) & (xy[:, 0] <= xlim[1]) &
        (xy[:, 1] >= ylim[0]) & (xy[:, 1] <= ylim[1])
    )
    return xy[mask], values[mask]


def plot_diff_heatmap(ref_xy, diff, out_path,
                      inset_xy=None, inset_diff=None,
                      inset_xlim=None, inset_ylim=None):
    fig, ax = plt.subplots(figsize=(11, 9))
    vmax = float(np.percentile(np.abs(diff), 95)) if len(diff) else 1.0
    vmax = max(vmax, 0.5)
    lc = colored_line(ax, ref_xy[:, 0], ref_xy[:, 1], diff,
                      cmap="RdBu_r", vmin=-vmax, vmax=vmax, linewidth=6.0)
    cb = plt.colorbar(lc, ax=ax)
    cb.set_label("mean iterations: cold − warm")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal")
    # Keep main axis as full-track auto-bounds from colored_line().

    # Optional inset view (bottom-right) to show a zoomed section while
    # preserving full-track context in the same figure.
    if (
        inset_xy is not None and inset_diff is not None and len(inset_diff) > 1 and
        inset_xlim is not None and inset_ylim is not None
    ):
        # Keep inset near the edge and wide for readability; only reduce
        # height so it is shorter rather than uniformly smaller.
        inset_ax = ax.inset_axes([0.60, 0.01, 0.45, 0.4])
        # Use a local color scale for the inset so small ranges remain visible.
        inset_vmax = float(np.percentile(np.abs(inset_diff), 95))
        inset_vmax = max(inset_vmax, 1e-6)
        inset_lc = colored_line(inset_ax, inset_xy[:, 0], inset_xy[:, 1], inset_diff,
                                cmap="RdBu_r", vmin=-inset_vmax, vmax=inset_vmax,
                                linewidth=6.0)
        inset_ax.set_xlim(*inset_xlim)
        inset_ax.set_ylim(*inset_ylim)
        inset_ax.set_aspect("equal")
        inset_ax.set_xticks([])
        inset_ax.set_yticks([])
        # Place the inset colorbar outside the zoom box (to the right), so it
        # never overlays the zoomed data.
        inset_cax = inset_ax.inset_axes([1.04, 0.00, 0.05, 1.0])
        inset_cb = fig.colorbar(inset_lc, cax=inset_cax)
        inset_cb.ax.tick_params(labelsize=9)
        # Avoid indicate_inset_zoom overlay on the main axis; it adds an extra
        # rectangle/connector pair that can look like duplicate overlays.

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_per_run_heatmaps(ref_xy, warm_m, cold_m, out_path, xlim=None, ylim=None):
    fig, axes = plt.subplots(1, 2, figsize=(18, 9), sharex=True, sharey=True)
    combined = np.concatenate([warm_m, cold_m]) if len(warm_m) or len(cold_m) else np.array([])
    vmax = float(np.percentile(combined, 95)) if len(combined) else 1.0
    for ax, m, title in [(axes[0], warm_m, "Warm-start (baseline)"),
                         (axes[1], cold_m, "Cold-start")]:
        lc = colored_line(ax, ref_xy[:, 0], ref_xy[:, 1], m,
                          cmap="viridis", vmin=0, vmax=vmax, linewidth=6.0)
        ax.set_title(title)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_aspect("equal")
        if xlim is not None:
            ax.set_xlim(*xlim)
        if ylim is not None:
            ax.set_ylim(*ylim)
        plt.colorbar(lc, ax=ax, label="mean iterations per solve")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--baseline-csv", required=True,
                    help="iters CSV from warm-start bag (with pos_x, pos_y)")
    ap.add_argument("--cold-csv", required=True,
                    help="iters CSV from cold-start bag (with pos_x, pos_y)")
    ap.add_argument("--out-dir", required=True, help="Output directory for plots")
    ap.add_argument("--cell-size", type=float, default=0.5,
                    help="Min spacing in meters when subsampling the reference path")
    ap.add_argument("--max-lap", type=int, default=11,
                    help="Discard laps strictly after this (default 11 = 1 warmup + 10 race)")
    ap.add_argument("--min-cell-count", type=int, default=3,
                    help="Drop reference points with fewer than this many samples per run")
    ap.add_argument("--x-min", type=float, default=5.0,
                    help="Zoom window lower x bound (default 5.0)")
    ap.add_argument("--x-max", type=float, default=7.0,
                    help="Zoom window upper x bound (default 7.0)")
    ap.add_argument("--y-min", type=float, default=-9.0,
                    help="Zoom window lower y bound (default -9.0)")
    ap.add_argument("--y-max", type=float, default=-6.0,
                    help="Zoom window upper y bound (default -6.0)")
    args = ap.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    warm = drop_warmup(load(args.baseline_csv))
    cold = drop_warmup(load(args.cold_csv))

    # Detect laps and use the first kept lap as the reference path, then
    # average all kept laps onto that reference (same behavior as plot_test2_maps.py).
    laps = detect_laps(warm["pos_x"], warm["pos_y"])
    kept = [laps[i - 1] for i in range(2, args.max_lap + 1) if i <= len(laps)]
    if kept:
        ref_s, ref_e = kept[0]
        ref_xy_full = np.column_stack([warm["pos_x"][ref_s:ref_e], warm["pos_y"][ref_s:ref_e]])
        keep_idx = subsample_path_by_spacing(ref_xy_full, args.cell_size)
        ref_xy = ref_xy_full[keep_idx]

        warm_sel = concat_laps(warm, kept)
        cold_sel = concat_laps(cold, kept)
        warm_avg, warm_counts = average_along_reference_path(warm_sel, ref_xy)
        cold_avg, cold_counts = average_along_reference_path(cold_sel, ref_xy)
        print(f"Averaging across laps 2..{2 + len(kept) - 1} ({sum(e - s for s,e in kept)} rows -> {len(ref_xy)} reference points)")
    else:
        # fallback to single-run reference if no laps detected
        ref_xy_full = np.column_stack([warm["pos_x"], warm["pos_y"]])
        keep_idx = subsample_path_by_spacing(ref_xy_full, args.cell_size)
        ref_xy = ref_xy_full[keep_idx]
        warm_avg, warm_counts = average_along_reference_path(warm, ref_xy)
        cold_avg, cold_counts = average_along_reference_path(cold, ref_xy)

    valid = (
        np.isfinite(warm_avg["iterations"]) &
        np.isfinite(cold_avg["iterations"]) &
        (warm_counts >= args.min_cell_count) &
        (cold_counts >= args.min_cell_count)
    )

    ref_xy_plot = ref_xy[valid]
    warm_m = warm_avg["iterations"][valid]
    cold_m = cold_avg["iterations"][valid]
    diff = cold_m - warm_m

    xlim = (args.x_min, args.x_max)
    ylim = (args.y_min, args.y_max)
    zoom_mask = (
        (ref_xy_plot[:, 0] >= args.x_min) & (ref_xy_plot[:, 0] <= args.x_max) &
        (ref_xy_plot[:, 1] >= args.y_min) & (ref_xy_plot[:, 1] <= args.y_max)
    )
    ref_xy_zoom = ref_xy_plot[zoom_mask]
    diff_zoom = diff[zoom_mask]

    print(f"shared points (full): {len(diff)} (cell_size={args.cell_size}, min_cell_count={args.min_cell_count})")
    print(f"shared points (zoom): {len(diff_zoom)} in x=[{args.x_min}, {args.x_max}], y=[{args.y_min}, {args.y_max}]")
    if len(diff):
        print(f"diff stats (full): min={diff.min():.2f}  median={np.median(diff):.2f}  mean={diff.mean():.2f}  max={diff.max():.2f}")
    if len(diff_zoom):
        print(f"diff stats (zoom): min={diff_zoom.min():.2f}  median={np.median(diff_zoom):.2f}  mean={diff_zoom.mean():.2f}  max={diff_zoom.max():.2f}")

    # Full-track diff heatmap with zoom inset in the bottom-right corner.
    # Keep the main view fully zoomed out; xlim/ylim apply only to the inset.
    plot_diff_heatmap(ref_xy_plot, diff,
                      out_dir / "test4_iter_diff_heatmap.svg",
                      inset_xy=ref_xy_zoom, inset_diff=diff_zoom,
                      inset_xlim=xlim, inset_ylim=ylim)
    # Zoomed diff heatmap for the selected section.
    plot_diff_heatmap(ref_xy_zoom, diff_zoom,
                      out_dir / "test4_iter_diff_heatmap_zoom.svg")
    # Per-run panel stays full-track for context.
    plot_per_run_heatmaps(ref_xy_plot, warm_m, cold_m, out_dir / "test4_iter_per_run_heatmaps.svg")

    print(f"Wrote heatmaps to {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
