#!/usr/bin/env python3
"""
Test 3: augmented-state ablation comparison.

Inputs: per-run CSVs produced by export_ablation_csv.py. Each CSV is tagged
with a label (e.g. "Baseline", "SteerOff", "AccelOff", "AccelOffSteerOff").
The first --run is treated as the reference: its first kept race lap defines
the polyline that every working run is snapped to so per-track metric
averages line up.

Runs are auto-classified:
  - "working":  detect_laps finds >= --min-laps full race laps.
                We average each metric along the reference path across the
                kept race laps (default laps 2..11).
  - "crashed":  fewer laps detected. We keep the raw recorded path so the
                trajectory overlay can show *where* the run died.

Outputs (in <out-dir>):
    test3_trajectory_overlay.svg  all runs' driven paths overlaid; crashed
                                runs end abruptly
    test3_iter_boxplot.svg        boxplot of per-solve iteration counts for
                                every run, including crashed ones up to
                                the crash point
    test3_steering_smoothness.svg working runs side-by-side on the track,
                                colored by |Δsteer| (jitter)
    test3_speed_track.svg         working runs side-by-side, colored by
                                commanded speed
    test3_iter_track.svg          working runs side-by-side, colored by
                                per-solve iteration count
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

DEFAULT_STEERING_RATE_LIMIT_RAD_S = 2.849


# ---------- I/O ----------

def load(path):
    rows = list(csv.DictReader(open(path)))
    return {
        "stamp_ns":   np.array([int(r["stamp_ns"])     for r in rows], dtype=np.int64),
        "pos_x":      np.array([float(r["pos_x"])      for r in rows]),
        "pos_y":      np.array([float(r["pos_y"])      for r in rows]),
        "iterations": np.array([float(r["iterations"]) for r in rows]),
        "steer":      np.array([float(r["steer"])      for r in rows]),
        "speed":      np.array([float(r["speed"])      for r in rows]),
    }


def add_steer_rate(data, steering_rate_limit=DEFAULT_STEERING_RATE_LIMIT_RAD_S):
    """Add steering-rate fields.

    The bag stores /drive.steering_angle, i.e. the commanded steering angle.
    Differentiating that command by bag row timestamps gives a command jump
    rate, not the physical actuator rate. Reconstruct the actuator-limited
    steering angle first, using the same rate limit as the MPC feedback path.
    """
    stamp_s = data["stamp_ns"].astype(np.float64) * 1e-9
    dt = np.diff(stamp_s, prepend=stamp_s[0])
    # First row dt is 0; use the median dt as a safe fallback.
    nominal_dt = float(np.median(dt[dt > 0])) if (dt > 0).any() else 0.005
    dt = np.where(dt > 0, dt, nominal_dt)
    command_dsteer = np.diff(data["steer"], prepend=data["steer"][0])
    data["command_steer_rate"] = command_dsteer / dt
    data["abs_command_steer_rate"] = np.abs(data["command_steer_rate"])

    actual_steer = np.empty_like(data["steer"])
    if len(actual_steer):
        actual_steer[0] = data["steer"][0]
        for i in range(1, len(actual_steer)):
            max_delta = steering_rate_limit * dt[i]
            steer_diff = data["steer"][i] - actual_steer[i - 1]
            actual_steer[i] = actual_steer[i - 1] + np.clip(
                steer_diff, -max_delta, max_delta)
    data["actual_steer_est"] = actual_steer
    actual_dsteer = np.diff(actual_steer, prepend=actual_steer[0])
    data["steer_rate"] = actual_dsteer / dt
    data["abs_steer_rate"] = np.abs(data["steer_rate"])
    return data


def add_lateral_dev(data, ref_xy):
    """Add lateral_dev (m) = distance from each pose to the nearest reference point."""
    pts = np.column_stack([data["pos_x"], data["pos_y"]])
    try:
        from scipy.spatial import cKDTree
        tree = cKDTree(ref_xy)
        dists, _ = tree.query(pts)
    except ImportError:
        dists = np.empty(len(pts))
        chunk = 1000
        for i in range(0, len(pts), chunk):
            end = min(i + chunk, len(pts))
            d2 = ((pts[i:end, None, :] - ref_xy[None, :, :]) ** 2).sum(axis=2)
            dists[i:end] = np.sqrt(d2.min(axis=1))
    data["lateral_dev"] = dists
    return data


def drop_warmup(data):
    mask = (data["pos_x"] ** 2 + data["pos_y"] ** 2) > 1e-4
    return {k: v[mask] for k, v in data.items()}


# ---------- Lap detection (same scheme as plot_test2_maps.py) ----------

def detect_laps(pos_x, pos_y, min_radius=2.0, min_lap_length=25.0):
    n = len(pos_x)
    if n < 2:
        return [(0, n)]
    start_x, start_y = pos_x[0], pos_y[0]
    dist = np.sqrt((pos_x - start_x) ** 2 + (pos_y - start_y) ** 2)
    seg = np.sqrt(np.diff(pos_x) ** 2 + np.diff(pos_y) ** 2)
    cum = np.concatenate(([0.0], np.cumsum(seg)))
    starts = [0]
    last_path = 0.0
    in_zone = True
    for i in range(1, n):
        inside = dist[i] < min_radius
        if inside and not in_zone and (cum[i] - last_path) > min_lap_length:
            starts.append(i)
            last_path = cum[i]
        in_zone = inside
    starts.append(n)
    return [(starts[i], starts[i + 1]) for i in range(len(starts) - 1)]


def slice_lap(data, lap_slice):
    s, e = lap_slice
    return {k: v[s:e] for k, v in data.items()}


def concat_laps(data, lap_slices):
    return {k: np.concatenate([v[s:e] for s, e in lap_slices])
            for k, v in data.items()}


# ---------- Reference-path averaging ----------

def subsample_path_by_spacing(xy, min_spacing):
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


def average_along_reference_path(data, ref_xy):
    try:
        from scipy.spatial import cKDTree
        tree = cKDTree(ref_xy)
        _, nearest = tree.query(np.column_stack([data["pos_x"], data["pos_y"]]))
    except ImportError:
        d_xy = np.column_stack([data["pos_x"], data["pos_y"]])
        nearest = np.empty(len(d_xy), dtype=np.int64)
        chunk = 1000
        for i in range(0, len(d_xy), chunk):
            end = min(i + chunk, len(d_xy))
            d2 = ((d_xy[i:end, None, :] - ref_xy[None, :, :]) ** 2).sum(axis=2)
            nearest[i:end] = d2.argmin(axis=1)
    n_ref = len(ref_xy)
    counts = np.bincount(nearest, minlength=n_ref).astype(np.float64)
    safe = np.where(counts > 0, counts, 1.0)
    out = {"pos_x": ref_xy[:, 0].copy(), "pos_y": ref_xy[:, 1].copy()}
    for field, values in data.items():
        if field in ("pos_x", "pos_y"):
            continue
        sums = np.bincount(nearest, weights=values, minlength=n_ref)
        out[field] = np.where(counts > 0, sums / safe, np.nan)
    return out


def colored_line(ax, x, y, values, *, cmap, vmin, vmax, linewidth=5.0):
    x = np.asarray(x); y = np.asarray(y); values = np.asarray(values)
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


# ---------- Per-run preparation ----------

class Run:
    """One ablation condition: raw data + classification + (if working) averaged."""
    def __init__(self, label, data_raw, laps, kept_slices, status):
        self.label = label
        self.data_raw = data_raw          # post-warmup, all messages
        self.laps = laps                  # all detected laps
        self.kept_slices = kept_slices    # laps used for averaging (may be empty)
        self.status = status              # "working" or "crashed"
        self.averaged = None              # set by snap_to_reference()

    def snap_to_reference(self, ref_xy, steering_rate_limit):
        if self.status != "working":
            return
        concat = concat_laps(self.data_raw, self.kept_slices)
        # annotate per-sample derived signals before averaging
        add_steer_rate(concat, steering_rate_limit)
        add_lateral_dev(concat, ref_xy)
        self.averaged = average_along_reference_path(concat, ref_xy)
        # keep the un-averaged kept-lap data around for histograms / time-series
        self.kept_raw = concat


def prepare_run(label, csv_path, max_lap, min_laps, steering_rate_limit):
    raw = drop_warmup(load(csv_path))
    add_steer_rate(raw, steering_rate_limit)  # always available, including for crashed runs
    laps = detect_laps(raw["pos_x"], raw["pos_y"])
    kept_idx = [i for i in range(2, max_lap + 1) if i <= len(laps)]
    kept_slices = [laps[i - 1] for i in kept_idx]
    n_race = max(0, len(laps) - 1)  # excluding warmup
    status = "working" if n_race >= min_laps else "crashed"
    return Run(label, raw, laps, kept_slices, status)


# ---------- Plots ----------

PALETTE = {
    "Baseline":         "#1f77b4",   # blue
    "SteerOff":         "#2ca02c",   # green
    "AccelOff":         "#ff7f0e",   # orange
    "AccelOffSteerOff": "#d62728",   # red
}
DEFAULT_COLOR = "#7f7f7f"


def color_for(label):
    return PALETTE.get(label, DEFAULT_COLOR)


def plot_trajectory_overlay(runs, out_path):
    fig, ax = plt.subplots(figsize=(11, 9))
    for r in runs:
        if r.status == "working" and r.kept_slices:
            # plot first kept lap (clean single-lap trace) for legibility
            s, e = r.kept_slices[0]
            x, y = r.data_raw["pos_x"][s:e], r.data_raw["pos_y"][s:e]
            ax.plot(x, y, "-", color=color_for(r.label), linewidth=1.5,
                    label=f"{r.label} (1 lap shown, {len(r.kept_slices)} laps kept)")
        else:
            # crashed: show full driven path until the bag ended
            x, y = r.data_raw["pos_x"], r.data_raw["pos_y"]
            ax.plot(x, y, "-", color=color_for(r.label), linewidth=1.5,
                    label=f"{r.label} (CRASHED, ~{len(r.laps)-1} laps before stop)")
            if len(x):
                ax.plot([x[-1]], [y[-1]], "X", color=color_for(r.label),
                        markersize=14, markeredgecolor="black",
                        markeredgewidth=1.2, zorder=5)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal")
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_iter_boxplot(runs, out_path):
    fig, ax = plt.subplots(figsize=(10, 7))
    data = [r.data_raw["iterations"] for r in runs]
    labels = []
    for r in runs:
        tag = "" if r.status == "working" else "\n(crashed)"
        labels.append(f"{r.label}{tag}")
    bp = ax.boxplot(data, patch_artist=True, showfliers=True, widths=0.5)
    ax.set_xticklabels(labels)
    for patch, r in zip(bp["boxes"], runs):
        patch.set_facecolor(color_for(r.label))
        patch.set_alpha(0.7)
    for median in bp["medians"]:
        median.set_color("black"); median.set_linewidth(1.5)
    ax.set_ylabel("ADMM iterations per solve")
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_steer_rate_histogram(runs, out_path):
    """Overlay log-y histograms of |steer rate| for each run.

    The shape is the diagnostic: a tall narrow peak at small values + long
    tail = "many tiny adjustments, occasional large bursts"; a single broad
    peak at moderate values = "constant moderate twitching".
    """
    fig, ax = plt.subplots(figsize=(11, 7))
    # shared bins so distributions are directly comparable
    all_vals = np.concatenate([r.data_raw["abs_steer_rate"] for r in runs])
    vmax = float(np.nanpercentile(all_vals, 99.9))
    vmax = max(vmax, 0.1)
    bins = np.linspace(0.0, vmax, 80)
    for r in runs:
        v = r.data_raw["abs_steer_rate"]
        ax.hist(v, bins=bins, histtype="step", linewidth=2.0,
                color=color_for(r.label),
                label=f"{r.label} (n={len(v)})")
    ax.set_yscale("log")
    ax.set_xlabel("|estimated actuator steering rate| [rad/s]")
    ax.set_ylabel("count (log)")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_crash_forensics(crashed_runs, ref_xy, out_path, window_s=10.0):
    """For each crashed run, plot the last `window_s` seconds of every metric
    we have so the failure signature is obvious. Markers: t=0 = last sample
    in the bag (presumed crash moment)."""
    if not crashed_runs:
        return
    rows_def = [
        ("speed",           "commanded speed [m/s]"),
        ("steer",           "commanded steering [rad]"),
        ("abs_steer_rate",  "|estimated actuator steering rate| [rad/s]"),
        ("iterations",      "iterations per solve"),
        ("lateral_dev",     "lateral dev. from baseline path [m]"),
    ]
    n_col = len(crashed_runs)
    n_row = len(rows_def)
    fig, axes = plt.subplots(n_row, n_col,
                             figsize=(7.5 * n_col, 2.6 * n_row),
                             sharex="col")
    if n_col == 1:
        axes = axes.reshape(-1, 1)

    for col, r in enumerate(crashed_runs):
        # crashed runs aren't snapped; add lateral_dev on the raw data
        add_lateral_dev(r.data_raw, ref_xy)
        stamp_s = r.data_raw["stamp_ns"].astype(np.float64) * 1e-9
        end_t = stamp_s[-1]
        mask = stamp_s >= (end_t - window_s)
        t = stamp_s[mask] - end_t

        for row, (field, ylabel) in enumerate(rows_def):
            ax = axes[row, col]
            ax.plot(t, r.data_raw[field][mask], "-",
                    color=color_for(r.label), linewidth=1.0)
            ax.axvline(0.0, color="red", linestyle="--", linewidth=1.2,
                       label="bag end (crash)" if row == 0 else None)
            ax.grid(alpha=0.3)
            if row == 0:
                ax.set_title(f"{r.label} — final {window_s:.0f} s")
                ax.legend(loc="upper left", fontsize=8)
            if col == 0:
                ax.set_ylabel(ylabel)
            if row == n_row - 1:
                ax.set_xlabel("time before bag end [s]")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_crash_trajectory_zoom(crashed_runs, ref_xy, out_path, window_s=10.0):
    """Zoom trajectory of the final `window_s` seconds for each crashed run,
    colored by time-before-crash, with the baseline reference path as a
    light backdrop for spatial context."""
    if not crashed_runs:
        return
    n = len(crashed_runs)
    fig, axes = plt.subplots(1, n, figsize=(8.5 * n, 8), squeeze=False)
    axes = axes[0]
    for ax, r in zip(axes, crashed_runs):
        ax.plot(ref_xy[:, 0], ref_xy[:, 1], "-", color="#bbbbbb",
                linewidth=1.0, label="baseline lap 2 (reference)")
        stamp_s = r.data_raw["stamp_ns"].astype(np.float64) * 1e-9
        end_t = stamp_s[-1]
        mask = stamp_s >= (end_t - window_s)
        x = r.data_raw["pos_x"][mask]
        y = r.data_raw["pos_y"][mask]
        t_rel = stamp_s[mask] - end_t
        sc = ax.scatter(x, y, c=t_rel, s=18, cmap="plasma",
                        vmin=float(t_rel.min()), vmax=0.0)
        if len(x):
            ax.plot([x[-1]], [y[-1]], "X", color="black", markersize=18,
                    markeredgewidth=2, label="bag end (crash)")
        ax.set_title(f"{r.label} — final {window_s:.0f} s")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_aspect("equal")
        plt.colorbar(sc, ax=ax, label="time before bag end [s]")
        ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_steering_trace(working_runs, out_path):
    """Time series of commanded steering angle for the first kept lap of each run.
    Shows the zigzag-vs-burst pattern directly."""
    fig, ax = plt.subplots(figsize=(13, 6))
    for r in working_runs:
        s, e = r.kept_slices[0]
        t = (r.data_raw["stamp_ns"][s:e] - r.data_raw["stamp_ns"][s]).astype(np.float64) * 1e-9
        ax.plot(t, r.data_raw["steer"][s:e], "-", color=color_for(r.label),
                linewidth=0.9, label=r.label)
    ax.set_xlabel("time within lap [s]")
    ax.set_ylabel("commanded steering angle [rad]")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_metric_side_by_side(working_runs, field, label, cmap, out_path,
                              symmetric=False):
    fig, axes = plt.subplots(1, len(working_runs),
                             figsize=(9 * len(working_runs), 9),
                             sharex=True, sharey=True)
    if len(working_runs) == 1:
        axes = [axes]
    # shared color range across all working runs
    all_vals = np.concatenate([r.averaged[field][np.isfinite(r.averaged[field])]
                                for r in working_runs])
    if symmetric:
        vmax = float(np.nanpercentile(np.abs(all_vals), 99))
        vmax = max(vmax, 1e-6)
        vmin = -vmax
    else:
        vmin = float(np.nanpercentile(all_vals, 1))
        vmax = float(np.nanpercentile(all_vals, 99))
        if vmax <= vmin: vmax = vmin + 1e-6
    for ax, r in zip(axes, working_runs):
        d = r.averaged
        lc = colored_line(ax, d["pos_x"], d["pos_y"], d[field],
                          cmap=cmap, vmin=vmin, vmax=vmax)
        ax.set_title(r.label)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_aspect("equal")
        plt.colorbar(lc, ax=ax, label=label)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


# ---------- Main ----------

def parse_run_arg(s):
    if "=" not in s:
        raise argparse.ArgumentTypeError(f"--run must be LABEL=PATH, got: {s}")
    label, path = s.split("=", 1)
    if not label or not path:
        raise argparse.ArgumentTypeError(f"--run must be LABEL=PATH, got: {s}")
    return (label, path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True, action="append", type=parse_run_arg,
                    help="LABEL=CSV_PATH (repeat for each condition; first one "
                         "becomes the reference path)")
    ap.add_argument("--out-dir", required=True, help="Output directory for plots")
    ap.add_argument("--max-lap", type=int, default=11,
                    help="Discard laps strictly after this (default 11)")
    ap.add_argument("--min-laps", type=int, default=5,
                    help="Minimum race laps for 'working' classification (default 5)")
    ap.add_argument("--cell-size", type=float, default=0.1,
                    help="Min spacing (m) between reference points (default 0.1)")
    ap.add_argument("--steer-rate-limit", type=float,
                    default=DEFAULT_STEERING_RATE_LIMIT_RAD_S,
                    help="Actuator steering-rate limit used when reconstructing "
                         "physical steering rate from /drive commands "
                         f"(default {DEFAULT_STEERING_RATE_LIMIT_RAD_S} rad/s)")
    args = ap.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    runs = [prepare_run(label, path, args.max_lap, args.min_laps,
                        args.steer_rate_limit)
            for label, path in args.run]
    for r in runs:
        race_laps = max(0, len(r.laps) - 1)
        kept = len(r.kept_slices)
        print(f"  {r.label:>20}: detected {len(r.laps)} laps "
              f"({race_laps} race) -> keep {kept} -> {r.status}")

    working = [r for r in runs if r.status == "working"]
    if not working:
        raise SystemExit("No working runs to use as reference path")

    # Build reference path from first working run's first kept lap.
    ref_run = working[0]
    rs, re_ = ref_run.kept_slices[0]
    ref_xy_full = np.column_stack([ref_run.data_raw["pos_x"][rs:re_],
                                   ref_run.data_raw["pos_y"][rs:re_]])
    ref_xy = ref_xy_full[subsample_path_by_spacing(ref_xy_full, args.cell_size)]
    print(f"  reference path: {len(ref_xy)} points from {ref_run.label} lap 2 "
          f"(min spacing {args.cell_size} m)")

    for r in working:
        r.snap_to_reference(ref_xy, args.steer_rate_limit)

    # Per-run summary that quantifies the reconstructed actuator-rate pattern.
    print(f"\n  steering pattern summary (estimated actuator |steer rate| in rad/s, "
          f"limit={args.steer_rate_limit:.3f}):")
    print(f"  {'run':>18} {'median':>8} {'mean':>8} {'p90':>8} {'p99':>8} "
          f"{'%<0.5':>6} {'%>limit':>8} {'raw_cmd_p99':>11} {'lat_dev_p95[m]':>15}")
    for r in runs:
        v = r.data_raw["abs_steer_rate"]
        p_small = (v < 0.5).mean() * 100.0
        p_large = (v > args.steer_rate_limit + 1e-6).mean() * 100.0
        raw_cmd_p99 = float(np.nanpercentile(r.data_raw["abs_command_steer_rate"], 99))
        lat_p95 = (float(np.nanpercentile(r.averaged["lateral_dev"], 95))
                   if r.averaged is not None else float("nan"))
        print(f"  {r.label:>18} {np.median(v):>8.3f} {np.mean(v):>8.3f} "
              f"{np.percentile(v,90):>8.3f} {np.percentile(v,99):>8.3f} "
              f"{p_small:>5.1f}% {p_large:>7.1f}% {raw_cmd_p99:>11.3f} "
              f"{lat_p95:>15.3f}")

    plot_trajectory_overlay(runs, out_dir / "test3_trajectory_overlay.svg")
    plot_iter_boxplot(runs, out_dir / "test3_iter_boxplot.svg")
    plot_steer_rate_histogram(runs, out_dir / "test3_steer_rate_histogram.svg")
    crashed = [r for r in runs if r.status == "crashed"]
    if crashed:
        plot_crash_forensics(crashed, ref_xy,
                             out_dir / "test3_crash_forensics.svg")
        plot_crash_trajectory_zoom(crashed, ref_xy,
                                   out_dir / "test3_crash_trajectory_zoom.svg")
    if len(working) >= 1:
        plot_steering_trace(working, out_dir / "test3_steering_trace.svg")
        plot_metric_side_by_side(working, "abs_steer_rate",
                                 "|estimated actuator steering rate| [rad/s]", "viridis",
                                 out_dir / "test3_steering_smoothness.svg")
        plot_metric_side_by_side(working, "speed",
                                 "commanded speed [m/s]", "viridis",
                                 out_dir / "test3_speed_track.svg")
        plot_metric_side_by_side(working, "iterations",
                                 "iterations per solve", "viridis",
                                 out_dir / "test3_iter_track.svg")
        plot_metric_side_by_side(working, "lateral_dev",
                                 "lateral deviation from baseline path [m]",
                                 "magma",
                                 out_dir / "test3_tracking_error_track.svg")

    print(f"\nWrote plots to {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
