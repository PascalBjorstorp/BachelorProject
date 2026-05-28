#!/usr/bin/env python3
"""
Test 1: CPU baseline vs FPGA UDP (the chosen FPGA transport).

The methodology mirrors the FPGA timing section in the thesis:
  * Keep only the cleaned race-lap window (laps 2..MAX_LAP) for the CPU
    baseline, using the same lap detection as Test 2/3.
  * Treat the FPGA UDP stats CSV as-is (the same recording already targets
    the race-lap window; lap filtering would require joining the stats CSV
    back to bag pose, which is out of scope here).
  * For each implementation, model the per-solve compute time as an affine
    function of the ADMM iteration count:
            t = t_setup + N_iter * t_iter
    Group rows by iteration count, take the median per group, drop groups
    with fewer than --min-iter-group samples, fit with numpy.polyfit, and
    report R^2.

Inputs:
  --baseline-csv     CSV from export_baseline_timing_csv.py
                     (cols: iterations, solve_us, pos_x, pos_y, ...)
  --fpga-udp-stats   the *.stats.csv from FPGA_UDP/ActualOutput/
                     (cols: iterations, status, solve_time_us,
                            total_call_us, opencl_overhead_us, ...)

Outputs (in <out-dir>):
  test1_iterations_boxplot.png            iteration count: CPU vs FPGA UDP
  test1_solve_us_boxplot.png              kernel solve time: CPU vs FPGA UDP
  test1_solve_us_histogram.png            overlaid solve-time histogram
  test1_per_iter_affine_fit.png           per-iteration affine fit in time
  test1_per_iter_affine_fit_cycles.png    same fit normalized to clock cycles
                                          (uses --cpu-freq-mhz, --fpga-freq-mhz)
"""

import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


# ---------- I/O ----------

def load_columns(path, fields):
    rows = list(csv.DictReader(open(path)))
    return {f: np.array([float(r[f]) for r in rows]) for f in fields}


def drop_warmup(data):
    mask = (data["pos_x"] ** 2 + data["pos_y"] ** 2) > 1e-4
    return {k: v[mask] for k, v in data.items()}


# ---------- Lap detection (same scheme as plot_test2_maps.py) ----------

def detect_laps(pos_x, pos_y, min_radius=2.0, min_lap_length=25.0):
    n = len(pos_x)
    if n < 2:
        return [(0, n)]
    sx, sy = pos_x[0], pos_y[0]
    dist = np.sqrt((pos_x - sx) ** 2 + (pos_y - sy) ** 2)
    seg = np.sqrt(np.diff(pos_x) ** 2 + np.diff(pos_y) ** 2)
    cum = np.concatenate(([0.0], np.cumsum(seg)))
    starts = [0]; last = 0.0; in_zone = True
    for i in range(1, n):
        inside = dist[i] < min_radius
        if inside and not in_zone and (cum[i] - last) > min_lap_length:
            starts.append(i); last = cum[i]
        in_zone = inside
    starts.append(n)
    return [(starts[i], starts[i + 1]) for i in range(len(starts) - 1)]


def keep_race_laps(data, max_lap):
    laps = detect_laps(data["pos_x"], data["pos_y"])
    kept = [laps[i - 1] for i in range(2, max_lap + 1) if i <= len(laps)]
    if not kept:
        raise SystemExit(f"no laps in range 2..{max_lap} (detected {len(laps)})")
    out = {k: np.concatenate([v[s:e] for s, e in kept]) for k, v in data.items()}
    return out, kept, laps


# ---------- Stats helpers ----------

def summary(label, x):
    return (f"  {label:>30}  n={len(x):>6}  "
            f"median={np.median(x):8.2f}  mean={x.mean():8.2f}  "
            f"p90={np.percentile(x, 90):8.2f}  p99={np.percentile(x, 99):8.2f}  "
            f"max={x.max():8.2f}")


def affine_fit_per_iter(iters, times, min_group_n=20):
    """Group `times` by integer iteration count, take the median per group,
    drop groups with fewer than min_group_n samples, and fit an affine
    model y = a + b*x. Returns (a, b, r2, fit_x, fit_y, group_counts)."""
    iters_int = np.round(iters).astype(np.int64)
    unique = np.unique(iters_int)
    medians, xs, counts = [], [], []
    for u in unique:
        sel = (iters_int == u)
        n = int(sel.sum())
        if n < min_group_n:
            continue
        medians.append(float(np.median(times[sel])))
        xs.append(int(u))
        counts.append(n)
    xs = np.asarray(xs, dtype=np.float64)
    ys = np.asarray(medians, dtype=np.float64)
    counts = np.asarray(counts, dtype=np.int64)
    if len(xs) < 2:
        return None
    # least squares fit y = a + b*x
    b, a = np.polyfit(xs, ys, 1)
    y_pred = a + b * xs
    ss_res = float(np.sum((ys - y_pred) ** 2))
    ss_tot = float(np.sum((ys - ys.mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    return {"a": float(a), "b": float(b), "r2": float(r2),
            "x": xs, "y": ys, "counts": counts}


# ---------- Plots ----------

def plot_box(series, labels, ylabel, out_path, log_y=False, colors=None):
    fig, ax = plt.subplots(figsize=(max(7, 2.5 * len(labels)), 7))
    bp = ax.boxplot(series, tick_labels=labels, patch_artist=True,
                    showfliers=True, widths=0.5)
    palette = colors or ["#7eb6ff", "#ffb47e"]
    for patch, c in zip(bp["boxes"], palette):
        patch.set_facecolor(c); patch.set_alpha(0.7)
    for m in bp["medians"]:
        m.set_color("black"); m.set_linewidth(1.5)
    ax.set_ylabel(ylabel)
    ax.grid(axis="y", alpha=0.3)
    if log_y:
        ax.set_yscale("log")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_hist(series, labels, xlabel, out_path, log_y=True, bins=80, colors=None):
    fig, ax = plt.subplots(figsize=(11, 7))
    vmax = float(np.nanpercentile(np.concatenate(series), 99.5))
    vmax = max(vmax, 1e-3)
    edges = np.linspace(0.0, vmax, bins)
    palette = colors or ["#1f77b4", "#ff7f0e"]
    for v, lab, c in zip(series, labels, palette):
        ax.hist(v, bins=edges, histtype="step", linewidth=2.0, color=c,
                label=f"{lab} (n={len(v)})")
    ax.set_xlabel(xlabel)
    ax.set_ylabel("count")
    if log_y: ax.set_yscale("log")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=10)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_affine_fits(fits, labels, out_path, colors=None):
    """Scatter the per-iter-count median together with the fitted line for
    each implementation, on the same axes."""
    fig, ax = plt.subplots(figsize=(11, 7))
    palette = colors or ["#1f77b4", "#ff7f0e"]
    x_max = max(int(f["x"].max()) for f in fits if f is not None)
    for fit, lab, c in zip(fits, labels, palette):
        if fit is None:
            continue
        ax.scatter(fit["x"], fit["y"], s=40, color=c, edgecolors="black",
                   linewidths=0.5,
                   label=f"{lab} medians (n_groups={len(fit['x'])})")
        xx = np.linspace(0, max(x_max, int(fit["x"].max())) + 1, 50)
        ax.plot(xx, fit["a"] + fit["b"] * xx, "-", color=c, linewidth=2.0,
                label=f"{lab} fit: t = {fit['a']:.1f} + {fit['b']:.2f}·N  "
                      f"(R² = {fit['r2']:.4f})")
    ax.set_xlabel("ADMM iterations per solve")
    ax.set_ylabel("solve time [μs]")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


# ---------- Main ----------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--baseline-csv", required=True)
    ap.add_argument("--fpga-udp-stats", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--max-lap", type=int, default=11,
                    help="Keep CPU race laps 2..MAX_LAP (default 11)")
    ap.add_argument("--min-iter-group", type=int, default=20,
                    help="Drop iteration-count groups with fewer samples "
                         "than this when fitting (default 20, matches "
                         "the FPGA section's methodology)")
    ap.add_argument("--cpu-freq-mhz", type=float, default=1700.0,
                    help="CPU clock frequency [MHz] for the cycle-count "
                         "normalization (default 1700)")
    ap.add_argument("--fpga-freq-mhz", type=float, default=200.0,
                    help="FPGA kernel clock frequency [MHz] for the "
                         "cycle-count normalization (default 200)")
    args = ap.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # ---- CPU baseline: load, drop warmup, keep race laps 2..MAX_LAP ----
    base_full = load_columns(args.baseline_csv,
                             ["iterations", "solve_us", "pos_x", "pos_y"])
    base_full = drop_warmup(base_full)
    base, kept, all_laps = keep_race_laps(base_full, args.max_lap)
    print(f"CPU baseline: detected {len(all_laps)} laps, "
          f"kept {len(kept)} race laps -> {len(base['iterations'])} samples")

    # ---- FPGA UDP stats: load, drop failed solves ----
    # Use kernel_us (the pure FPGA device event) for the affine fit; the
    # `solve_time_us` column has different definitions in the ROS2 and UDP
    # receiver programs and isn't a fair comparison target.
    fpga_fields = ["iterations", "status", "kernel_us", "solve_time_us",
                   "total_call_us", "opencl_overhead_us"]
    fud_all = load_columns(args.fpga_udp_stats, fpga_fields)
    ok = fud_all["status"] == 0
    fud = {k: v[ok] for k, v in fud_all.items()}
    n_dropped = int((~ok).sum())
    print(f"FPGA UDP: {len(fud['iterations'])} valid solves "
          f"(dropped {n_dropped} with status != 0)")

    print()
    print("=== ITERATIONS ===")
    print(summary("CPU baseline (laps 2..N)", base["iterations"]))
    print(summary("FPGA UDP",                 fud["iterations"]))
    print()
    print("=== SOLVE TIME [μs] ===")
    print(summary("CPU /solve_us",            base["solve_us"]))
    print(summary("FPGA UDP kernel_us (device event)", fud["kernel_us"]))
    print(summary("FPGA UDP solve_time_us (host bookkeeping)", fud["solve_time_us"]))
    print(summary("FPGA UDP total_call_us",   fud["total_call_us"]))

    # ---- Affine per-iteration fit (matches FPGA section methodology) ----
    print()
    print(f"=== PER-ITERATION AFFINE FIT  (groups with n >= {args.min_iter_group}) ===")
    cpu_fit  = affine_fit_per_iter(base["iterations"], base["solve_us"],
                                   args.min_iter_group)
    fpga_fit = affine_fit_per_iter(fud["iterations"], fud["kernel_us"],
                                   args.min_iter_group)
    if cpu_fit is None:
        print("  CPU: insufficient groups for fit"); cpu_fit = None
    else:
        print(f"  CPU baseline   :  t = {cpu_fit['a']:7.3f} + {cpu_fit['b']:7.3f}·N  "
              f"R² = {cpu_fit['r2']:.4f}  ({len(cpu_fit['x'])} groups)")
    if fpga_fit is None:
        print("  FPGA UDP: insufficient groups for fit")
    else:
        print(f"  FPGA UDP kernel:  t = {fpga_fit['a']:7.3f} + {fpga_fit['b']:7.3f}·N  "
              f"R² = {fpga_fit['r2']:.4f}  ({len(fpga_fit['x'])} groups)")
    if cpu_fit and fpga_fit:
        print()
        print(f"  setup-time ratio (FPGA / CPU): {fpga_fit['a'] / max(cpu_fit['a'], 1e-9):.2f}×")
        print(f"  per-iter   ratio (FPGA / CPU): {fpga_fit['b'] / max(cpu_fit['b'], 1e-9):.2f}×")

        # ---- Cycle-count normalization (scale time × clock frequency) ----
        # cycles = t[μs] × f[MHz]   since μs × MHz = 1 cycle
        print()
        print(f"=== CYCLE-COUNT NORMALIZATION  "
              f"(CPU @ {args.cpu_freq_mhz:.0f} MHz, FPGA @ {args.fpga_freq_mhz:.0f} MHz) ===")
        cpu_setup_cyc  = cpu_fit['a']  * args.cpu_freq_mhz
        cpu_iter_cyc   = cpu_fit['b']  * args.cpu_freq_mhz
        fpga_setup_cyc = fpga_fit['a'] * args.fpga_freq_mhz
        fpga_iter_cyc  = fpga_fit['b'] * args.fpga_freq_mhz
        print(f"  CPU baseline   :  N_cyc = {cpu_setup_cyc:8.0f} + {cpu_iter_cyc:7.1f}·N  cycles")
        print(f"  FPGA UDP kernel:  N_cyc = {fpga_setup_cyc:8.0f} + {fpga_iter_cyc:7.1f}·N  cycles")
        print()
        print(f"  setup-cycles ratio (CPU / FPGA): "
              f"{cpu_setup_cyc / max(fpga_setup_cyc, 1e-9):.2f}×")
        print(f"  per-iter cycles ratio (CPU / FPGA): "
              f"{cpu_iter_cyc / max(fpga_iter_cyc, 1e-9):.2f}×")
        # Per-iteration time the FPGA would have if it ran at CPU frequency:
        equiv_us = fpga_iter_cyc / args.cpu_freq_mhz
        print(f"  FPGA per-iter if it ran at CPU clock: {equiv_us:.3f} μs "
              f"(vs CPU {cpu_fit['b']:.3f} μs/iter)")

    # ---- Plots ----
    LABELS2 = ["CPU baseline", "FPGA UDP"]
    plot_box([base["iterations"], fud["iterations"]],
             LABELS2, "ADMM iterations per solve",
             out_dir / "test1_iterations_boxplot.png")
    plot_box([base["solve_us"], fud["kernel_us"]],
             LABELS2, "kernel solve time [μs]  (FPGA: device event kernel_us)",
             out_dir / "test1_solve_us_boxplot.png", log_y=True)
    plot_hist([base["solve_us"], fud["kernel_us"]],
              LABELS2, "kernel solve time [μs]",
              out_dir / "test1_solve_us_histogram.png", log_y=True)
    plot_affine_fits([cpu_fit, fpga_fit],
                     ["CPU baseline", "FPGA UDP kernel"],
                     out_dir / "test1_per_iter_affine_fit.png")

    # Cycle-count version of the affine-fit plot.
    if cpu_fit and fpga_fit:
        cpu_cyc_fit = {
            "a": cpu_fit["a"]  * args.cpu_freq_mhz,
            "b": cpu_fit["b"]  * args.cpu_freq_mhz,
            "r2": cpu_fit["r2"],
            "x": cpu_fit["x"],
            "y": cpu_fit["y"] * args.cpu_freq_mhz,
            "counts": cpu_fit["counts"],
        }
        fpga_cyc_fit = {
            "a": fpga_fit["a"] * args.fpga_freq_mhz,
            "b": fpga_fit["b"] * args.fpga_freq_mhz,
            "r2": fpga_fit["r2"],
            "x": fpga_fit["x"],
            "y": fpga_fit["y"] * args.fpga_freq_mhz,
            "counts": fpga_fit["counts"],
        }
        # Render with the same plotter; tweak label suffix for cycles.
        fig, ax = plt.subplots(figsize=(11, 7))
        palette = ["#1f77b4", "#ff7f0e"]
        x_max = max(int(cpu_cyc_fit["x"].max()), int(fpga_cyc_fit["x"].max()))
        for fit, lab, c, f_mhz in [
            (cpu_cyc_fit,  f"CPU baseline @ {args.cpu_freq_mhz:.0f} MHz",
             palette[0], args.cpu_freq_mhz),
            (fpga_cyc_fit, f"FPGA UDP kernel @ {args.fpga_freq_mhz:.0f} MHz",
             palette[1], args.fpga_freq_mhz),
        ]:
            ax.scatter(fit["x"], fit["y"], s=40, color=c,
                       edgecolors="black", linewidths=0.5,
                       label=f"{lab} medians")
            xx = np.linspace(0, x_max + 1, 50)
            ax.plot(xx, fit["a"] + fit["b"] * xx, "-", color=c, linewidth=2.0,
                    label=f"fit: N = {fit['a']:.0f} + {fit['b']:.1f}·N_iter  "
                          f"(R² = {fit['r2']:.4f})")
        ax.set_xlabel("ADMM iterations per solve")
        ax.set_ylabel("solve time [clock cycles]")
        ax.grid(alpha=0.3)
        ax.legend(loc="best", fontsize=9)
        fig.tight_layout()
        fig.savefig(out_dir / "test1_per_iter_affine_fit_cycles.png", dpi=150)
        plt.close(fig)

    print(f"\nWrote plots to {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
