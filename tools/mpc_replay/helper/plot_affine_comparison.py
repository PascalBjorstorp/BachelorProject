#!/usr/bin/env python3
"""
Visualize the effect of the d_affine bias term by running the MPC at one
snapshot twice -- once with MPC_AFFINE_SCALE=1.0 (baseline) and once with
MPC_AFFINE_SCALE=0.0 -- and overlaying both predicted horizons.

At a corner this makes the planning difference visible:
  - WITH affine: predicted horizon follows the curved reference path.
  - WITHOUT affine: predicted horizon misses the curvature because the
    linear-only model `x[k+1] = A x[k] + B u[k]` cannot represent the
    "natural curved drift" the nonlinear vehicle has at that operating point.

Snapshot picker prefers high-curvature / large-heading-change rows so the
divergence is geometrically obvious.

Outputs (in <out-dir>):
    affine_track.svg        track view with both predicted horizons overlaid
    affine_tracking.svg     per-state-component (ey, epsi, vx, steering, accel)
                                                 comparison across the horizon, with-affine vs without

Usage:
  python3 plot_affine_comparison.py \
      --state-csv path/to/state_replay.csv \
      [--idx N] [--seed S] --out-dir path/to/plots
"""

import argparse
import csv
import math
import os
import random
import subprocess
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

QP_SCALE = 262144.0
HORIZON = 20
WITH_COLOR = "#1f77b4"      # blue
WITHOUT_COLOR = "#d62728"   # red
REF_COLOR = "#2ca02c"       # green for reference path
TRACK_COLOR = "#cfcfcf"
CONS_COLOR = "#8bcddd"
WALL_MARGIN_M = 0.20
TRACK_X_MIN = 5.0
TRACK_X_MAX = 9.0
TRACK_Y_MIN = -5.7
TRACK_Y_MAX = -1.0


def fp_to_float(v):
    return int(v) / QP_SCALE


def load_state_rows(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def pick_corner_snapshot(rows, seed, past_steps=10,
                         kappa_thresh=0.08, heading_thresh=0.18,
                         min_vel=1.5):
    rng = random.Random(seed)
    candidates = []
    for i, row in enumerate(rows):
        if i < past_steps + 2 or i > len(rows) - 4:
            continue
        try:
            vx = fp_to_float(row["velocity_fp"])
            kappa = abs(fp_to_float(row["ref_kappa_fp_5"]))
            heading_span = abs(fp_to_float(row["ref_psi_fp_19"]) -
                               fp_to_float(row["ref_psi_fp_0"]))
        except (KeyError, ValueError):
            continue
        if vx > min_vel and (kappa > kappa_thresh or heading_span > heading_thresh):
            candidates.append((i, kappa, heading_span))
    if not candidates:
        # fallback: any row with non-trivial heading change
        for i, row in enumerate(rows):
            if i < past_steps + 2 or i > len(rows) - 4:
                continue
            try:
                vx = fp_to_float(row["velocity_fp"])
                heading_span = abs(fp_to_float(row["ref_psi_fp_19"]) -
                                   fp_to_float(row["ref_psi_fp_0"]))
            except (KeyError, ValueError):
                continue
            if vx > 0.8 and heading_span > 0.05:
                candidates.append((i, 0.0, heading_span))
    if not candidates:
        raise SystemExit("No suitable corner snapshot found in state CSV.")
    row_idx, _, _ = candidates[rng.randrange(len(candidates))]
    return row_idx, int(rows[row_idx]["idx"])


def run_plan_export(helper_bin, state_csv, replay_idx, out_csv, affine_scale):
    env = os.environ.copy()
    env["MPC_AFFINE_SCALE"] = str(affine_scale)
    proc = subprocess.run(
        [str(helper_bin), str(state_csv), str(replay_idx), str(out_csv)],
        env=env, capture_output=True, text=True,
    )
    if proc.returncode != 0:
        raise RuntimeError(
            f"export_mpc_plan_snapshot failed (affine={affine_scale}):\n"
            f"stderr:\n{proc.stderr[-2000:]}"
        )


def read_plan_snapshot(path):
    meta = {}
    with open(path) as f:
        line = f.readline()
        while line and line.strip():
            k, v = line.strip().split(",", 1)
            meta[k] = v
            line = f.readline()
        rows = list(csv.DictReader(f))
    return meta, rows


def build_corridor_xy(plan_rows):
    ref_x = np.array([float(r["ref_x"]) for r in plan_rows[:-1]])
    ref_y = np.array([float(r["ref_y"]) for r in plan_rows[:-1]])
    ref_psi = np.array([float(r["ref_psi"]) for r in plan_rows[:-1]])
    left = np.array([float(r["left_bound"]) for r in plan_rows[:-1]])
    right = np.array([float(r["right_bound"]) for r in plan_rows[:-1]])
    nx = -np.sin(ref_psi); ny = np.cos(ref_psi)
    left_xy = np.column_stack([ref_x + nx * left, ref_y + ny * left])
    right_xy = np.column_stack([ref_x - nx * right, ref_y - ny * right])
    return left_xy, right_xy


def build_extended_corridor_from_replay(state_rows, row_idx, span_rows):
    """Build a long track corridor by sampling the centerline + bounds from
    state_replay.csv rows in a window around the selected snapshot.
    The polyline will be much longer than the prediction horizon; the
    bounding-box clipping in the plot will trim it visually."""
    j0 = max(0, row_idx - span_rows)
    j1 = min(len(state_rows), row_idx + span_rows + 1)
    rows = state_rows[j0:j1]
    cx = np.array([fp_to_float(r["ref_x_fp_0"]) for r in rows])
    cy = np.array([fp_to_float(r["ref_y_fp_0"]) for r in rows])
    psi = np.array([fp_to_float(r["ref_psi_fp_0"]) for r in rows])
    left = np.array([fp_to_float(r["ref_left_bound_fp_0"]) for r in rows])
    right = np.array([fp_to_float(r["ref_right_bound_fp_0"]) for r in rows])
    nx = -np.sin(psi); ny = np.cos(psi)
    left_xy  = np.column_stack([cx + nx * left,  cy + ny * left])
    right_xy = np.column_stack([cx - nx * right, cy - ny * right])
    return left_xy, right_xy


def build_past_vehicle_path(state_rows, row_idx, past_steps):
    j0 = max(0, row_idx - past_steps)
    sl = state_rows[j0:row_idx + 1]
    x = np.array([fp_to_float(r["x_fp"]) for r in sl])
    y = np.array([fp_to_float(r["y_fp"]) for r in sl])
    return x, y


def build_past_reference_horizon(state_rows, row_idx, past_steps):
    j0 = max(0, row_idx - past_steps)
    sl = state_rows[j0:row_idx + 1]
    rx = np.array([fp_to_float(r["ref_x_fp_0"]) for r in sl])
    ry = np.array([fp_to_float(r["ref_y_fp_0"]) for r in sl])
    return rx, ry


def plan_xy(plan_rows):
    return (np.array([float(r["pred_x"]) for r in plan_rows]),
            np.array([float(r["pred_y"]) for r in plan_rows]))


def make_track_plot(plan_with, plan_without, out_path, meta_with,
                    state_rows, row_idx, past_steps, extended_span):
    # forward (horizon-step) reference + corridor from the plan snapshots
    ref_x = np.array([float(r["ref_x"]) for r in plan_with[:-1]])
    ref_y = np.array([float(r["ref_y"]) for r in plan_with[:-1]])
    px_w, py_w = plan_xy(plan_with)
    px_n, py_n = plan_xy(plan_without)
    cur_x = px_w[0]; cur_y = py_w[0]
    pred_psi0 = float(plan_with[0]["pred_psi"])

    # extended corridor (much longer than just the horizon)
    ext_left_xy, ext_right_xy = build_extended_corridor_from_replay(
        state_rows, row_idx, extended_span)
    past_x, past_y = build_past_vehicle_path(state_rows, row_idx, past_steps)
    past_ref_x, past_ref_y = build_past_reference_horizon(state_rows, row_idx, past_steps)

    # axis bounds: compute from interesting features only, then add padding.
    # The extended corridor gets visually clipped to these bounds.
    pad = 1.0
    interesting_x = np.concatenate([past_x, ref_x, px_w, px_n])
    interesting_y = np.concatenate([past_y, ref_y, py_w, py_n])
    x_lim = (TRACK_X_MIN, TRACK_X_MAX)
    y_lim = (TRACK_Y_MIN, TRACK_Y_MAX)

    fig, ax = plt.subplots(figsize=(12, 9))

    # Extended corridor (long polylines clipped naturally by axis bounds)
    ax.plot(ext_left_xy[:, 0],  ext_left_xy[:, 1],  "--", color=TRACK_COLOR, lw=1.4)
    ax.plot(ext_right_xy[:, 0], ext_right_xy[:, 1], "--", color=TRACK_COLOR, lw=1.4)
    ax.fill(np.r_[ext_left_xy[:, 0], ext_right_xy[::-1, 0]],
            np.r_[ext_left_xy[:, 1], ext_right_xy[::-1, 1]],
            color="#ececec", alpha=0.6, label="Track corridor")

    # Past vehicle path + past reference (rear-facing)
    ax.plot(past_x,     past_y,     "-",  color="#2b6f9e", lw=2.4, label="Past vehicle path")
    ax.plot(past_ref_x, past_ref_y, "--", color="#6f8fb7", lw=1.6, alpha=0.95,
            label="Past reference")

    # Forward reference + the two predicted horizons
    ax.plot(ref_x, ref_y, "-o", color=REF_COLOR, lw=2.0, ms=4,
            label="Reference horizon")
    ax.plot(px_w, py_w, "-o", color=WITH_COLOR, lw=2.5, ms=5,
            label="Predicted horizon — with d_k")
    ax.plot(px_n, py_n, "-o", color=WITHOUT_COLOR, lw=2.5, ms=5,
            label="Predicted horizon — without d_k")
    ax.scatter([cur_x], [cur_y], s=110, color="black", zorder=10,
               label="Current vehicle state k")
    arrow_len = 0.4
    ax.arrow(cur_x, cur_y,
             arrow_len * math.cos(pred_psi0),
             arrow_len * math.sin(pred_psi0),
             width=0.02, head_width=0.12, head_length=0.14,
             color="black", length_includes_head=True, zorder=10)
    ax.annotate(f"k+{HORIZON}", xy=(px_w[-1], py_w[-1]),
                xytext=(8, 8), textcoords="offset points",
                color=WITH_COLOR, fontsize=11, fontweight="bold")
    ax.annotate(f"k+{HORIZON}", xy=(px_n[-1], py_n[-1]),
                xytext=(8, -14), textcoords="offset points",
                color=WITHOUT_COLOR, fontsize=11, fontweight="bold")

    ax.set_xlim(*x_lim)
    ax.set_ylim(*y_lim)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(alpha=0.3)
    iters = meta_with.get("iterations", "?")
    ax.legend(loc="best", fontsize=10, framealpha=0.95)
    fig.tight_layout()
    fig.savefig(out_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def make_tracking_plot(plan_with, plan_without, out_path, meta_with):
    H = HORIZON
    stage_state = np.arange(H + 1)
    stage_ctrl = np.arange(H)

    def col(rows, key, n):
        return np.array([float(rows[k][key]) for k in range(n)])

    fig, axes = plt.subplots(5, 1, figsize=(11, 13), sharex=True)

    # Lateral error
    axes[0].plot(stage_state, col(plan_with, "plan_ey", H + 1),
                 "-o", color=WITH_COLOR, lw=2.2, ms=4, label="WITH d_affine")
    axes[0].plot(stage_state, col(plan_without, "plan_ey", H + 1),
                 "-o", color=WITHOUT_COLOR, lw=2.2, ms=4, label="WITHOUT d_affine")
    axes[0].axhline(0.0, color="grey", ls="--", lw=1.0)
    axes[0].set_ylabel("e_y [m]")
    axes[0].grid(alpha=0.3)
    axes[0].legend(loc="best", fontsize=10)

    # Heading error
    axes[1].plot(stage_state, col(plan_with, "plan_epsi", H + 1),
                 "-o", color=WITH_COLOR, lw=2.2, ms=4)
    axes[1].plot(stage_state, col(plan_without, "plan_epsi", H + 1),
                 "-o", color=WITHOUT_COLOR, lw=2.2, ms=4)
    axes[1].axhline(0.0, color="grey", ls="--", lw=1.0)
    axes[1].set_ylabel("e_psi [rad]")
    axes[1].grid(alpha=0.3)

    # v_x
    axes[2].plot(stage_state, col(plan_with, "plan_vx", H + 1),
                 "-o", color=WITH_COLOR, lw=2.2, ms=4)
    axes[2].plot(stage_state, col(plan_without, "plan_vx", H + 1),
                 "-o", color=WITHOUT_COLOR, lw=2.2, ms=4)
    ref_vx = col(plan_with, "ref_vx", H)
    axes[2].plot(stage_ctrl, ref_vx, "--", color=REF_COLOR, lw=1.5, label="ref")
    axes[2].set_ylabel("v_x [m/s]")
    axes[2].grid(alpha=0.3)
    axes[2].legend(loc="best", fontsize=9)

    # commanded steering (plan_delta_actual at next stage = applied output)
    steer_w = np.array([float(plan_with[k + 1]["plan_delta_actual"]) for k in range(H)])
    steer_n = np.array([float(plan_without[k + 1]["plan_delta_actual"]) for k in range(H)])
    axes[3].step(stage_ctrl, steer_w, where="post", color=WITH_COLOR, lw=2.4)
    axes[3].step(stage_ctrl, steer_n, where="post", color=WITHOUT_COLOR, lw=2.4)
    axes[3].set_ylabel("steering [rad]")
    axes[3].grid(alpha=0.3)

    # commanded acceleration
    accel_w = np.array([float(plan_with[k]["u_accel"]) for k in range(H)])
    accel_n = np.array([float(plan_without[k]["u_accel"]) for k in range(H)])
    axes[4].step(stage_ctrl, accel_w, where="post", color=WITH_COLOR, lw=2.4)
    axes[4].step(stage_ctrl, accel_n, where="post", color=WITHOUT_COLOR, lw=2.4)
    axes[4].set_ylabel("u_accel [m/s²]")
    axes[4].set_xlabel("horizon stage")
    axes[4].grid(alpha=0.3)

    fig.suptitle(f"Planned state + control over horizon, idx={meta_with.get('selected_idx', '?')}",
                 fontsize=12)
    fig.tight_layout()
    fig.savefig(out_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def ensure_export_binary(repo_root):
    helper_src = repo_root / "tools/mpc_replay/helper/export_mpc_plan_snapshot.c"
    helper_bin = repo_root / "tools/mpc_replay/helper/export_mpc_plan_snapshot"
    deps = [helper_src,
            repo_root / "MPC/src/util_math.c",
            repo_root / "MPC/src/vehicle_model.c",
            repo_root / "MPC/src/riccati_solver.c",
            repo_root / "MPC/src/mpc.c"]
    if not helper_bin.exists() or any(d.stat().st_mtime > helper_bin.stat().st_mtime for d in deps):
        cmd = ["gcc", "-O3",
               f"-I{repo_root}/MPC/include",
               f"-I{repo_root}/FPGA_Implementations/MPC_FPGA_Kria/include",
               str(helper_src),
               str(repo_root / "MPC/src/util_math.c"),
               str(repo_root / "MPC/src/vehicle_model.c"),
               str(repo_root / "MPC/src/riccati_solver.c"),
               str(repo_root / "MPC/src/mpc.c"),
               "-lm", "-o", str(helper_bin)]
        subprocess.run(cmd, check=True, cwd=str(repo_root))
    return helper_bin


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--state-csv", required=True,
                    help="state_replay.csv from any prior Test 2/replay run")
    ap.add_argument("--idx", type=int, default=None,
                    help="replay idx (column 'idx'); if omitted, auto-picks a corner snapshot")
    ap.add_argument("--seed", type=int, default=7,
                    help="seed for auto corner pick")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--past-steps", type=int, default=100,
                    help="rear-facing window for vehicle path and reference (default 12)")
    ap.add_argument("--extended-span", type=int, default=140,
                    help="rows on each side of the snapshot to sample for the "
                         "extended track corridor (default 140)")
    args = ap.parse_args()

    repo_root = Path(__file__).resolve().parents[3]
    state_csv = Path(args.state_csv).resolve()
    out_dir = Path(args.out_dir); out_dir.mkdir(parents=True, exist_ok=True)

    helper_bin = ensure_export_binary(repo_root)

    rows = load_state_rows(state_csv)
    if args.idx is None:
        row_idx, replay_idx = pick_corner_snapshot(rows, args.seed)
        print(f"auto-picked corner snapshot: row {row_idx} -> replay idx {replay_idx}")
    else:
        replay_idx = args.idx
        idx_lookup = {int(r["idx"]): i for i, r in enumerate(rows)}
        if replay_idx not in idx_lookup:
            raise SystemExit(f"replay idx {replay_idx} not found in {state_csv}")
        row_idx = idx_lookup[replay_idx]
        print(f"using snapshot: row {row_idx} -> replay idx {replay_idx}")

    with_csv = out_dir / f"plan_with_affine_idx{replay_idx}.csv"
    without_csv = out_dir / f"plan_without_affine_idx{replay_idx}.csv"
    run_plan_export(helper_bin, state_csv, replay_idx, with_csv,    affine_scale=1.0)
    run_plan_export(helper_bin, state_csv, replay_idx, without_csv, affine_scale=0.0)

    meta_w, plan_w = read_plan_snapshot(with_csv)
    meta_n, plan_n = read_plan_snapshot(without_csv)
    print(f"WITH d_affine   : iterations={meta_w.get('iterations')}, status={meta_w.get('solver_status')}")
    print(f"WITHOUT d_affine: iterations={meta_n.get('iterations')}, status={meta_n.get('solver_status')}")

    # Quantify the divergence
    px_w, py_w = plan_xy(plan_w)
    px_n, py_n = plan_xy(plan_n)
    div = np.sqrt((px_w - px_n) ** 2 + (py_w - py_n) ** 2)
    print(f"position divergence at k=  0: {div[0]:.3f} m")
    print(f"position divergence at k= 10: {div[10]:.3f} m")
    print(f"position divergence at k= 20: {div[20]:.3f} m   (= end-of-horizon split between the two plans)")

    track_out    = out_dir / f"affine_track_idx{replay_idx}.svg"
    tracking_out = out_dir / f"affine_tracking_idx{replay_idx}.svg"
    make_track_plot(plan_w, plan_n, track_out, meta_w,
                    state_rows=rows, row_idx=row_idx,
                    past_steps=args.past_steps,
                    extended_span=args.extended_span)
    make_tracking_plot(plan_w, plan_n, tracking_out, meta_w)
    print(f"Wrote {track_out}")
    print(f"Wrote {tracking_out}")


if __name__ == "__main__":
    main()
