#!/usr/bin/env python3
"""
Test 4: warm-start vs cold-start convergence comparison.

Reads per-solve iteration counts (one per row, column 'iterations') from two
CSVs produced by export_iteration_count_csv.py and renders a boxplot showing
the iteration-count distribution for each condition.
"""

import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update({
    "font.size": 18,
    "axes.titlesize": 20,
    "axes.labelsize": 18,
    "xtick.labelsize": 16,
    "ytick.labelsize": 16,
    "legend.fontsize": 16,
})


def load_iters(path):
    with open(path) as f:
        return np.asarray([float(r["iterations"]) for r in csv.DictReader(f)])


def summarize(label, x):
    return (f"{label:>16}: n={len(x):>6}  "
            f"median={np.median(x):5.1f}  mean={np.mean(x):5.2f}  "
            f"p90={np.percentile(x, 90):5.1f}  max={int(np.max(x)):>3d}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--baseline-csv", required=True,
                    help="Iteration counts CSV from the warm-start (baseline) bag")
    ap.add_argument("--cold-csv", required=True,
                    help="Iteration counts CSV from the cold-start bag")
    ap.add_argument("--out-dir", required=True, help="Output directory for plots")
    args = ap.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    base = load_iters(args.baseline_csv)
    cold = load_iters(args.cold_csv)
    print(summarize("baseline (warm)", base))
    print(summarize("cold-start", cold))

    fig, ax = plt.subplots(figsize=(8, 7))
    bp = ax.boxplot([base, cold],
                    labels=["Warm-start\n(baseline)", "Cold-start"],
                    patch_artist=True,
                    showfliers=True,
                    showmeans=True,
                    meanprops={"marker": "D",
                               "markerfacecolor": "white",
                               "markeredgecolor": "black",
                               "markersize": 7},
                    widths=0.5)
    for patch, color in zip(bp["boxes"], ["#7eb6ff", "#ff7e7e"]):
        patch.set_facecolor(color)
    for median in bp["medians"]:
        median.set_color("black")
        median.set_linewidth(1.5)
    ax.set_ylabel("ADMM iterations per solve")
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    out_path = out_dir / "test4_iterations_boxplot.svg"
    fig.savefig(out_path, dpi=150)
    plt.close(fig)

    print(f"Wrote: {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
