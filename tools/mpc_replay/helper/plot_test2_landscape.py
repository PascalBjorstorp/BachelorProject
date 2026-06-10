#!/usr/bin/env python3
"""
Test 2 cost landscape: visualize how the optimal plan + cost shift as the
velocity-tracking weight is varied at a single snapshot.

Inputs:  CSV from cost_landscape_at_pose (one row per swept weight).
Outputs: three PNGs in <out-dir>/plots/:
  map3_landscape_v_trajectory.png  planned velocity at horizon checkpoints
                                   vs swept w_vel
  map3_landscape_cost_components.png  per-term cost (under original weights)
                                      vs swept w_vel
  map3_landscape_J_vs_v.png        cost components vs the achieved v_h_end
                                   — the actual "landscape"
"""

import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

COST_TERMS = [
    "J_lat", "J_heading", "J_vel", "J_lat_vel", "J_yaw_rate",
    "J_delta_actual", "J_drate_prev", "J_accel_prev",
    "J_steer_in", "J_accel_in",
]


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"empty CSV: {path}")
    arr = {k: np.array([float(r[k]) for r in rows]) for k in rows[0].keys()
           if k not in {"idx", "status", "iters"}}
    return arr


def plot_v_trajectory(d, out_path, snapshot_idx):
    fig, ax = plt.subplots(figsize=(10, 7))
    ax.plot(d["w_vel_used"], d["v_now"],   "-o", label="v at k=0",     markersize=3)
    ax.plot(d["w_vel_used"], d["v_h5"],    "-o", label="v at k=5",     markersize=3)
    ax.plot(d["w_vel_used"], d["v_h10"],   "-o", label="v at k=10",    markersize=3)
    ax.plot(d["w_vel_used"], d["v_h_end"], "-o", label="v at horizon end", markersize=3)
    ax.set_xscale("log")
    ax.set_xlabel("MPC_W_VELOCITY (swept, log)")
    ax.set_ylabel("planned velocity [m/s]")
    ax.set_title(f"Map 3 (snapshot idx={snapshot_idx}): planned velocity profile "
                 f"as velocity-tracking weight is swept\n"
                 "low w_vel collapses the planned velocity toward zero")
    ax.axhline(0.0, color="k", linewidth=0.5)
    ax.grid(alpha=0.3)
    ax.legend()
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_cost_components(d, out_path, snapshot_idx):
    fig, ax = plt.subplots(figsize=(11, 7))
    for name in COST_TERMS:
        ax.plot(d["w_vel_used"], d[name], "-o", label=name, markersize=3)
    ax.plot(d["w_vel_used"], d["J_total"], "-k", label="J_total", linewidth=2)
    ax.set_xscale("log")
    ax.set_yscale("symlog", linthresh=1.0)
    ax.set_xlabel("MPC_W_VELOCITY (swept, log)")
    ax.set_ylabel("cost contribution under ORIGINAL weights (symlog)")
    ax.set_title(f"Map 3 (snapshot idx={snapshot_idx}): cost decomposition vs swept w_vel\n"
                 "evaluated under the baseline weight set so curves are comparable")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_J_vs_v(d, out_path, snapshot_idx):
    fig, ax = plt.subplots(figsize=(11, 7))
    order = np.argsort(d["v_h_end"])
    v = d["v_h_end"][order]
    for name in COST_TERMS:
        ax.plot(v, d[name][order], "-o", label=name, markersize=3)
    ax.plot(v, d["J_total"][order], "-k", label="J_total", linewidth=2)
    ax.set_xlabel("achieved v at horizon end [m/s] (parameterized by w_vel)")
    ax.set_ylabel("cost contribution under ORIGINAL weights (symlog)")
    ax.set_yscale("symlog", linthresh=1.0)
    ax.set_title(f"Map 3 landscape (snapshot idx={snapshot_idx}): J(v_h_end) traced out by sweeping w_vel\n"
                 "a local minimum of J_total at low v under the baseline objective\n"
                 "would justify the controller's 'stop' choice")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--landscape-csv", required=True,
                    help="CSV produced by cost_landscape_at_pose")
    ap.add_argument("--out-dir", default=None,
                    help="Where to put plots (default: alongside CSV)")
    args = ap.parse_args()

    csv_path = Path(args.landscape_csv)
    out_dir = Path(args.out_dir) if args.out_dir else csv_path.parent / "plots"
    out_dir.mkdir(parents=True, exist_ok=True)

    d = load(csv_path)
    snapshot_idx = "?" if "idx" not in d else int(d["idx"][0]) if hasattr(d.get("idx"), "__len__") else "?"

    # idx is the snapshot identifier; re-load to get the integer
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        first = next(reader)
        snapshot_idx = first["idx"]

    plot_v_trajectory(d, out_dir / "map3_landscape_v_trajectory.png", snapshot_idx)
    plot_cost_components(d, out_dir / "map3_landscape_cost_components.png", snapshot_idx)
    plot_J_vs_v(d, out_dir / "map3_landscape_J_vs_v.png", snapshot_idx)

    print(f"Wrote landscape plots to {out_dir}")


if __name__ == "__main__":
    main()
