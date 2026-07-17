#!/usr/bin/env python3
import argparse
import csv
import math
import os
import random
import subprocess
import tempfile
from datetime import datetime
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

plt.rcParams.update({
    "font.size": 18,
    "axes.labelsize": 18,
    "xtick.labelsize": 18,
    "ytick.labelsize": 18,
    "legend.fontsize": 18,
})


QP_SCALE = 262144.0
HORIZON = 20
DEFAULT_WALL_MARGIN_M = 0.20
DEFAULT_WALL_BIAS_CLEAR_M = 0.05
DEFAULT_WALL_BOUND_WINDOW = 3
PAST_COLOR = "#2b6f9e"
PREDICTED_COLOR = "#c4972f"
REFERENCE_COLOR = "#8a8a8a"


def fp_to_float(value: str | int) -> float:
    return int(value) / QP_SCALE


def load_state_rows(path: Path) -> list[dict]:
    with path.open() as f:
        return list(csv.DictReader(f))


def pick_snapshot(rows: list[dict], seed: int, past_steps: int) -> int:
    rng = random.Random(seed)
    candidates = []
    for i, row in enumerate(rows):
        if i < past_steps + 2:
            continue
        vx = fp_to_float(row["velocity_fp"])
        kappa = abs(fp_to_float(row["ref_kappa_fp_5"]))
        heading_span = abs(fp_to_float(row["ref_psi_fp_19"]) - fp_to_float(row["ref_psi_fp_0"]))
        if vx > 1.0 and (kappa > 0.05 or heading_span > 0.12):
            candidates.append(i)
    if not candidates:
        for i, row in enumerate(rows):
            if i < past_steps + 2:
                continue
            vx = fp_to_float(row["velocity_fp"])
            if vx > 0.8:
                candidates.append(i)
    if not candidates:
        raise RuntimeError("Could not find a usable replay row for plotting.")
    return candidates[rng.randrange(len(candidates))]


def ensure_export_helper(repo_root: Path) -> Path:
    helper_src = repo_root / "tools/mpc_replay/helper/export_mpc_plan_snapshot.c"
    helper_bin = repo_root / "tools/mpc_replay/helper/export_mpc_plan_snapshot"
    dependencies = [
        helper_src,
        repo_root / "MPC/src/util_math.c",
        repo_root / "MPC/src/vehicle_model.c",
        repo_root / "MPC/src/riccati_solver.c",
        repo_root / "MPC/src/mpc.c",
        repo_root / "MPC/include/mpc.h",
    ]
    needs_build = (not helper_bin.exists()) or any(
        dep.stat().st_mtime > helper_bin.stat().st_mtime for dep in dependencies
    )
    if needs_build:
        cmd = [
            "gcc",
            "-O3",
            f"-I{repo_root / 'MPC/include'}",
            f"-I{repo_root / 'FPGA_Implementations/MPC_FPGA_Kria/include'}",
            str(helper_src),
            str(repo_root / "MPC/src/util_math.c"),
            str(repo_root / "MPC/src/vehicle_model.c"),
            str(repo_root / "MPC/src/riccati_solver.c"),
            str(repo_root / "MPC/src/mpc.c"),
            "-lm",
            "-o",
            str(helper_bin),
        ]
        subprocess.run(cmd, check=True, cwd=repo_root)
    return helper_bin


def export_plan_snapshot(helper_bin: Path, state_csv: Path, idx: int, out_csv: Path) -> None:
    proc = subprocess.run(
        [str(helper_bin), str(state_csv), str(idx), str(out_csv)],
        text=True,
        capture_output=True,
    )
    if proc.returncode != 0:
        raise RuntimeError(
            f"Plan export failed for idx {idx}.\nstdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
        )


def read_plan_snapshot(path: Path) -> tuple[dict, list[dict]]:
    metadata: dict[str, str] = {}
    rows: list[dict] = []
    with path.open() as f:
        line = f.readline()
        while line and line.strip():
            key, value = line.strip().split(",", 1)
            metadata[key] = value
            line = f.readline()
        reader = csv.DictReader(f)
        rows = list(reader)
    return metadata, rows


def build_corridor_xy(plan_rows: list[dict]) -> tuple[np.ndarray, np.ndarray]:
    ref_x = np.array([float(r["ref_x"]) for r in plan_rows[:-1]])
    ref_y = np.array([float(r["ref_y"]) for r in plan_rows[:-1]])
    ref_psi = np.array([float(r["ref_psi"]) for r in plan_rows[:-1]])
    left = np.array([float(r["left_bound"]) for r in plan_rows[:-1]])
    right = np.array([float(r["right_bound"]) for r in plan_rows[:-1]])
    nx = -np.sin(ref_psi)
    ny = np.cos(ref_psi)
    left_xy = np.column_stack([ref_x + nx * left, ref_y + ny * left])
    right_xy = np.column_stack([ref_x - nx * right, ref_y - ny * right])
    return left_xy, right_xy


def build_constrained_corridor_xy(
    plan_rows: list[dict],
    wall_margin: float,
    wall_bias_clear: float,
    wall_bound_window: int,
) -> tuple[np.ndarray, np.ndarray]:
    ref_x = np.array([float(r["ref_x"]) for r in plan_rows[:-1]])
    ref_y = np.array([float(r["ref_y"]) for r in plan_rows[:-1]])
    ref_psi = np.array([float(r["ref_psi"]) for r in plan_rows[:-1]])
    left_raw = np.array([float(r["left_bound"]) for r in plan_rows[:-1]])
    right_raw = np.array([float(r["right_bound"]) for r in plan_rows[:-1]])
    nx = -np.sin(ref_psi)
    ny = np.cos(ref_psi)

    left_con = []
    right_con = []
    for k in range(HORIZON):
        j0 = max(0, k - wall_bound_window)
        j1 = min(HORIZON - 1, k + wall_bound_window)
        left_bound_k = np.min(left_raw[j0:j1 + 1])
        right_bound_k = np.min(right_raw[j0:j1 + 1])

        x_lb = wall_margin - right_bound_k
        x_ub = left_bound_k - wall_margin
        if wall_bias_clear > 0.0:
            x_lb_con = x_lb + wall_bias_clear
            x_ub_con = x_ub - wall_bias_clear
            if x_lb_con > x_ub_con:
                x_lb_con = x_lb
                x_ub_con = x_ub
        else:
            x_lb_con = x_lb
            x_ub_con = x_ub

        left_con.append(x_ub_con)
        right_con.append(-x_lb_con)

    left_con = np.asarray(left_con, dtype=float)
    right_con = np.asarray(right_con, dtype=float)
    left_xy = np.column_stack([ref_x + nx * left_con, ref_y + ny * left_con])
    right_xy = np.column_stack([ref_x - nx * right_con, ref_y - ny * right_con])
    return left_xy, right_xy


def get_replay_value(row: dict | None, key: str, fallback: float = 0.0) -> float:
    if row is None or key not in row or row[key] == "":
        return fallback
    return fp_to_float(row[key])


def clip_polyline_x(poly: np.ndarray, x_min: float | None = None, x_max: float | None = None) -> np.ndarray:
    points: list[np.ndarray] = [p.copy() for p in poly]

    def clip_boundary(pts: list[np.ndarray], boundary: float, keep_ge: bool) -> list[np.ndarray]:
        if not pts:
            return []
        out: list[np.ndarray] = []
        prev = pts[0]
        prev_in = (prev[0] >= boundary) if keep_ge else (prev[0] <= boundary)
        if prev_in:
            out.append(prev.copy())
        for curr in pts[1:]:
            curr_in = (curr[0] >= boundary) if keep_ge else (curr[0] <= boundary)
            if prev_in != curr_in:
                dx = curr[0] - prev[0]
                if abs(dx) > 1e-9:
                    t = (boundary - prev[0]) / dx
                    inter = prev + t * (curr - prev)
                    inter[0] = boundary
                    out.append(inter)
            if curr_in:
                out.append(curr.copy())
            prev = curr
            prev_in = curr_in
        return out

    if x_min is not None:
        points = clip_boundary(points, x_min, keep_ge=True)
    if x_max is not None:
        points = clip_boundary(points, x_max, keep_ge=False)
    return np.asarray(points, dtype=float)


def extend_polyline_to_x(poly: np.ndarray, x_target: float, at_start: bool) -> np.ndarray:
    if poly.shape[0] < 2:
        return poly

    if at_start:
        p0 = poly[0].copy()
        idx = 1
        while idx < poly.shape[0] and abs(poly[idx, 0] - p0[0]) < 1e-9:
            idx += 1
        if idx >= poly.shape[0]:
            return poly
        p1 = poly[idx].copy()
        t = (x_target - p0[0]) / (p1[0] - p0[0])
        extended = p0 + t * (p1 - p0)
        extended[0] = x_target
        if (p0[0] - x_target) * (p1[0] - x_target) <= 0.0:
            poly = clip_polyline_x(poly, x_min=x_target)
            if poly.shape[0] > 0 and abs(poly[0, 0] - x_target) < 1e-6:
                return poly
        return np.vstack([extended, poly])

    p1 = poly[-1].copy()
    idx = poly.shape[0] - 2
    while idx >= 0 and abs(poly[idx, 0] - p1[0]) < 1e-9:
        idx -= 1
    if idx < 0:
        return poly
    p0 = poly[idx].copy()
    t = (x_target - p1[0]) / (p1[0] - p0[0])
    extended = p1 + t * (p1 - p0)
    extended[0] = x_target
    if (p0[0] - x_target) * (p1[0] - x_target) <= 0.0:
        poly = clip_polyline_x(poly, x_max=x_target)
        if poly.shape[0] > 0 and abs(poly[-1, 0] - x_target) < 1e-6:
            return poly
    return np.vstack([poly, extended])


def build_extended_track_from_replay(
    state_rows: list[dict],
    selected_row_idx: int,
    span_rows: int,
    wall_margin: float,
    wall_bias_clear: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    j0 = max(0, selected_row_idx - span_rows)
    j1 = min(len(state_rows), selected_row_idx + span_rows + 1)
    rows = state_rows[j0:j1]

    center_x = np.array([fp_to_float(r["ref_x_fp_0"]) for r in rows], dtype=float)
    center_y = np.array([fp_to_float(r["ref_y_fp_0"]) for r in rows], dtype=float)
    psi = np.array([fp_to_float(r["ref_psi_fp_0"]) for r in rows], dtype=float)
    left = np.array([fp_to_float(r["ref_left_bound_fp_0"]) for r in rows], dtype=float)
    right = np.array([fp_to_float(r["ref_right_bound_fp_0"]) for r in rows], dtype=float)

    nx = -np.sin(psi)
    ny = np.cos(psi)

    track_left = np.column_stack([center_x + nx * left, center_y + ny * left])
    track_right = np.column_stack([center_x - nx * right, center_y - ny * right])

    x_lb = wall_margin - right
    x_ub = left - wall_margin
    x_lb_con = x_lb + wall_bias_clear
    x_ub_con = x_ub - wall_bias_clear
    infeasible = x_lb_con > x_ub_con
    x_lb_con[infeasible] = x_lb[infeasible]
    x_ub_con[infeasible] = x_ub[infeasible]

    constrained_left = np.column_stack([center_x + nx * x_ub_con, center_y + ny * x_ub_con])
    constrained_right = np.column_stack([center_x - nx * (-x_lb_con), center_y - ny * (-x_lb_con)])
    return track_left, track_right, constrained_left, constrained_right


def make_track_plot(
    state_rows: list[dict],
    replay_by_idx: dict[int, dict],
    selected_row_idx: int,
    selected_idx_value: int,
    plan_meta: dict,
    plan_rows: list[dict],
    out_path: Path,
    past_steps: int,
) -> None:
    left_crop_x = 1.5
    track_past_steps = max(past_steps, 18)
    ref_x = np.array([float(r["ref_x"]) for r in plan_rows[:-1]])
    ref_y = np.array([float(r["ref_y"]) for r in plan_rows[:-1]])
    pred_x = np.array([float(r["pred_x"]) for r in plan_rows])
    pred_y = np.array([float(r["pred_y"]) for r in plan_rows])
    ref_psi = np.array([float(r["ref_psi"]) for r in plan_rows[:-1]])
    pred_psi0 = float(plan_rows[0]["pred_psi"])
    current_x = pred_x[0]
    current_y = pred_y[0]
    left_xy, right_xy = build_corridor_xy(plan_rows)
    wall_margin = float(os.getenv("MPC_WALL_MARGIN", DEFAULT_WALL_MARGIN_M))
    wall_bias_clear = float(os.getenv("MPC_WALL_BIAS_CLEAR_M", DEFAULT_WALL_BIAS_CLEAR_M))
    wall_bound_window = int(os.getenv("MPC_WALL_BOUND_WINDOW", DEFAULT_WALL_BOUND_WINDOW))
    left_con_xy, right_con_xy = build_constrained_corridor_xy(
        plan_rows, wall_margin, wall_bias_clear, wall_bound_window
    )
    ext_left_xy, ext_right_xy, ext_left_con_xy, ext_right_con_xy = build_extended_track_from_replay(
        state_rows=state_rows,
        selected_row_idx=selected_row_idx,
        span_rows=140,
        wall_margin=wall_margin,
        wall_bias_clear=wall_bias_clear,
    )

    right_crop_x = max(np.max(ref_x), np.max(pred_x))

    ext_left_xy = clip_polyline_x(ext_left_xy, x_min=left_crop_x, x_max=right_crop_x)
    ext_right_xy = clip_polyline_x(ext_right_xy, x_min=left_crop_x, x_max=right_crop_x)
    ext_left_con_xy = clip_polyline_x(ext_left_con_xy, x_min=left_crop_x, x_max=right_crop_x)
    ext_right_con_xy = clip_polyline_x(ext_right_con_xy, x_min=left_crop_x, x_max=right_crop_x)
    ext_left_xy = extend_polyline_to_x(ext_left_xy, left_crop_x, at_start=True)
    ext_right_xy = extend_polyline_to_x(ext_right_xy, left_crop_x, at_start=True)
    ext_left_con_xy = extend_polyline_to_x(ext_left_con_xy, left_crop_x, at_start=True)
    ext_right_con_xy = extend_polyline_to_x(ext_right_con_xy, left_crop_x, at_start=True)
    ext_left_xy = extend_polyline_to_x(ext_left_xy, right_crop_x, at_start=False)
    ext_right_xy = extend_polyline_to_x(ext_right_xy, right_crop_x, at_start=False)
    ext_left_con_xy = extend_polyline_to_x(ext_left_con_xy, right_crop_x, at_start=False)
    ext_right_con_xy = extend_polyline_to_x(ext_right_con_xy, right_crop_x, at_start=False)

    past_slice = state_rows[max(0, selected_row_idx - track_past_steps):selected_row_idx + 1]
    past_vehicle_xy = np.column_stack([
        np.array([fp_to_float(r["x_fp"]) for r in past_slice], dtype=float),
        np.array([fp_to_float(r["y_fp"]) for r in past_slice], dtype=float),
    ])
    past_vehicle_xy = extend_polyline_to_x(past_vehicle_xy, left_crop_x, at_start=True)
    past_x = past_vehicle_xy[:, 0]
    past_y = past_vehicle_xy[:, 1]
    past_ref_x = np.array([fp_to_float(r["ref_x_fp_0"]) for r in past_slice[:-1]], dtype=float)
    past_ref_y = np.array([fp_to_float(r["ref_y_fp_0"]) for r in past_slice[:-1]], dtype=float)
    past_ref_xy = np.column_stack([past_ref_x, past_ref_y])
    past_ref_xy = extend_polyline_to_x(past_ref_xy, left_crop_x, at_start=True)
    past_ref_x = past_ref_xy[:, 0]
    past_ref_y = past_ref_xy[:, 1]

    fig = plt.figure(figsize=(11, 8.6), constrained_layout=True)
    ax_map = fig.add_subplot(1, 1, 1)

    ax_map.plot(ext_left_xy[:, 0], ext_left_xy[:, 1], color="#cfcfcf", lw=1.2, ls="--", alpha=0.95, zorder=1)
    ax_map.plot(ext_right_xy[:, 0], ext_right_xy[:, 1], color="#cfcfcf", lw=1.2, ls="--", alpha=0.95, zorder=1)
    ax_map.fill(
        np.r_[ext_left_xy[:, 0], ext_right_xy[::-1, 0]],
        np.r_[ext_left_xy[:, 1], ext_right_xy[::-1, 1]],
        color="#ececec",
        alpha=0.75,
        zorder=1.2,
        label="Track bounds",
    )
    ax_map.plot(ext_left_con_xy[:, 0], ext_left_con_xy[:, 1], color="#8bcddd", lw=1.2, ls="--", alpha=0.95, zorder=2)
    ax_map.plot(ext_right_con_xy[:, 0], ext_right_con_xy[:, 1], color="#8bcddd", lw=1.2, ls="--", alpha=0.95, zorder=2)
    ax_map.fill(
        np.r_[ext_left_con_xy[:, 0], ext_right_con_xy[::-1, 0]],
        np.r_[ext_left_con_xy[:, 1], ext_right_con_xy[::-1, 1]],
        color="#d7f2f4",
        alpha=0.75,
        zorder=2.2,
        label="Constrained bounds",
    )
    ax_map.plot(past_x, past_y, color="#2b6f9e", lw=2.2, label="Past vehicle path", zorder=3)
    ax_map.plot(past_ref_x, past_ref_y, color="#6f8fb7", lw=2.0, alpha=0.95, label="Past reference horizon", zorder=3.2)
    ax_map.plot(ref_x, ref_y, color="#c63d2f", lw=2.4, marker="o", ms=4.2,
                label="Reference horizon", zorder=5)
    ax_map.plot(pred_x, pred_y, color="#c4972f", lw=2.2, marker="o", ms=3.8,
                label="Predicted horizon", zorder=6)

    for rx, ry, px, py in zip(ref_x, ref_y, pred_x[:-1], pred_y[:-1]):
        ax_map.plot([px, rx], [py, ry], color="#5d5d5d", lw=0.8, alpha=0.35, zorder=4)

    ax_map.scatter([current_x], [current_y], s=70, color="black", zorder=7, label="Current state k")
    arrow_len = 0.35
    ax_map.arrow(
        current_x,
        current_y,
        arrow_len * math.cos(pred_psi0),
        arrow_len * math.sin(pred_psi0),
        width=0.018,
        head_width=0.11,
        head_length=0.13,
        color="black",
        length_includes_head=True,
        zorder=7,
    )
    ax_map.annotate(
        "k",
        xy=(current_x, current_y),
        xytext=(10, 10),
        textcoords="offset points",
        fontsize=13,
        fontweight="bold",
    )
    ax_map.annotate(
        f"k+N ({len(plan_rows)-1} steps)",
        xy=(pred_x[-1], pred_y[-1]),
        xytext=(10, -18),
        textcoords="offset points",
        fontsize=11,
        color="#6f5400",
    )

    all_x = np.r_[past_x, ref_x, pred_x, ext_left_xy[:, 0], ext_right_xy[:, 0], ext_left_con_xy[:, 0], ext_right_con_xy[:, 0]]
    all_y = np.r_[past_y, ref_y, pred_y, ext_left_xy[:, 1], ext_right_xy[:, 1], ext_left_con_xy[:, 1], ext_right_con_xy[:, 1]]
    ax_map.set_xlim(left_crop_x, right_crop_x)
    ax_map.set_ylim(all_y.min(), all_y.max())
    ax_map.set_aspect("equal", adjustable="box")
    ax_map.margins(x=0.0, y=0.0)
    ax_map.set_xlabel("x [m]")
    ax_map.set_ylabel("y [m]")
    ax_map.grid(color="#efefef", linewidth=0.8)
    ax_map.legend(loc="upper right", frameon=True, framealpha=0.95)

    fig.savefig(out_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def make_tracking_plot(
    state_rows: list[dict],
    replay_by_idx: dict[int, dict],
    selected_row_idx: int,
    selected_idx_value: int,
    plan_rows: list[dict],
    out_path: Path,
    past_steps: int,
) -> None:
    past_slice = state_rows[max(0, selected_row_idx - past_steps):selected_row_idx + 1]
    past_ctrl_x = np.arange(-(len(past_slice) - 1), 1)
    past_steer = []
    past_accel = []
    past_vx = []
    past_ey = []
    past_epsi = []
    past_ref_vx = []
    for row in past_slice:
        replay_row = replay_by_idx.get(int(row["idx"]))
        past_steer.append(
            get_replay_value(replay_row, "out_steer_fp", fp_to_float(row["steering_angle_fp"]))
        )
        past_accel.append(get_replay_value(replay_row, "out_accel_fp", 0.0))
        past_vx.append(get_replay_value(replay_row, "vx_fp", fp_to_float(row["velocity_fp"])))
        past_ey.append(get_replay_value(replay_row, "ey_fp", 0.0))
        past_epsi.append(get_replay_value(replay_row, "epsi_fp", 0.0))
        past_ref_vx.append(fp_to_float(row["ref_vx_fp_0"]))
    past_steer = np.asarray(past_steer, dtype=float)
    past_accel = np.asarray(past_accel, dtype=float)
    past_vx = np.asarray(past_vx, dtype=float)
    past_ey = np.asarray(past_ey, dtype=float)
    past_epsi = np.asarray(past_epsi, dtype=float)
    past_ref_vx = np.asarray(past_ref_vx, dtype=float)

    predicted_stage = np.arange(HORIZON)
    predicted_state_stage = np.arange(HORIZON + 1)
    predicted_steer = np.array(
        [float(plan_rows[k + 1]["plan_delta_cmd"]) for k in range(HORIZON)]
    )
    predicted_accel = np.array([float(plan_rows[k]["u_accel"]) for k in range(HORIZON)])
    predicted_vx = np.array([float(r["plan_vx"]) for r in plan_rows])
    reference_vx = np.array([float(r["ref_vx"]) for r in plan_rows[:-1]])
    predicted_ey = np.array([float(r["plan_ey"]) for r in plan_rows])
    predicted_epsi = np.array([float(r["plan_epsi"]) for r in plan_rows])

    fig = plt.figure(figsize=(12, 12), constrained_layout=True)
    gs = fig.add_gridspec(5, 1, height_ratios=[1.0, 1.0, 1.0, 1.0, 1.0])
    ax_steer = fig.add_subplot(gs[0, 0])
    ax_accel = fig.add_subplot(gs[1, 0], sharex=ax_steer)
    ax_vx = fig.add_subplot(gs[2, 0], sharex=ax_steer)
    ax_ey = fig.add_subplot(gs[3, 0], sharex=ax_steer)
    ax_epsi = fig.add_subplot(gs[4, 0], sharex=ax_steer)

    shade_end = HORIZON - 0.2
    for axis in (ax_steer, ax_accel, ax_vx, ax_ey, ax_epsi):
        axis.axvline(0.0, color="black", lw=3.0)
        axis.axvspan(0.0, shade_end, color="#d7f2f4", alpha=0.25)
        axis.grid(color="#efefef", linewidth=0.8)
        axis.set_xlim(-past_steps, HORIZON)

    ax_steer.step(past_ctrl_x, past_steer, where="post", color=PAST_COLOR, lw=3.0)
    ax_steer.step(predicted_stage, predicted_steer, where="post", color=PREDICTED_COLOR, lw=3.2)
    ax_steer.set_ylabel("steering [rad]")

    ax_accel.step(past_ctrl_x, past_accel, where="post", color=PAST_COLOR, lw=3.0)
    ax_accel.step(predicted_stage, predicted_accel, where="post", color=PREDICTED_COLOR, lw=3.2)
    ax_accel.set_ylabel("accel [m/s²]")

    ax_vx.plot(past_ctrl_x, past_vx, color=PAST_COLOR, lw=3.0)
    ax_vx.plot(predicted_state_stage, predicted_vx, color=PREDICTED_COLOR, lw=3.2, marker="o", ms=3.5)
    ax_vx.step(past_ctrl_x, past_ref_vx, where="post", color=REFERENCE_COLOR, lw=3.0, ls="--", alpha=0.95)
    ax_vx.step(predicted_stage, reference_vx, where="post", color=REFERENCE_COLOR, lw=3.2, ls="--")
    ax_vx.set_ylabel("v_x [m/s]")

    ax_ey.plot(past_ctrl_x, past_ey, color=PAST_COLOR, lw=3.0)
    ax_ey.plot(predicted_state_stage, predicted_ey, color=PREDICTED_COLOR, lw=3.2, marker="o", ms=3.8)
    ax_ey.axhline(0.0, color=REFERENCE_COLOR, lw=3.0, ls="--")
    ax_ey.set_ylabel("e_y [m]")

    ax_epsi.plot(past_ctrl_x, past_epsi, color=PAST_COLOR, lw=3.0)
    ax_epsi.plot(predicted_state_stage, predicted_epsi, color=PREDICTED_COLOR, lw=3.2, marker="o", ms=3.8)
    ax_epsi.axhline(0.0, color=REFERENCE_COLOR, lw=3.0, ls="--")
    ax_epsi.set_ylabel("e_psi [rad]")

    legend_handles = [
        Line2D([0], [0], color=PAST_COLOR, lw=3.0, label="Past value"),
        Line2D([0], [0], color=PREDICTED_COLOR, lw=3.2, marker="o", ms=4, label="Predicted value"),
        Line2D([0], [0], color=REFERENCE_COLOR, lw=3.0, ls="--", label="Reference value"),
    ]
    ax_steer.legend(
        handles=legend_handles,
        loc="lower right",
        ncol=1,
        frameon=True,
        framealpha=0.95,
    )

    fig.savefig(out_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot one MPC receding-horizon snapshot from state_replay.csv.")
    parser.add_argument("--state-csv", default="tools/output/mpc_deep_diag_20260519_164731/state_replay.csv")
    parser.add_argument("--replay-cpu-csv", default="tools/output/mpc_deep_diag_20260519_164731/replay_cpu_out.csv")
    parser.add_argument("--idx", type=int, default=None, help="Replay idx to visualize. If omitted, a deterministic random snapshot is chosen.")
    parser.add_argument("--seed", type=int, default=13, help="Seed used when picking a random snapshot.")
    parser.add_argument("--past-steps", type=int, default=10)
    parser.add_argument("--out-prefix", default=None, help="Base output path prefix. Generates *_track.png and *_tracking.png.")
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parent.parent
    state_csv = (repo_root / args.state_csv).resolve()
    replay_cpu_csv = (repo_root / args.replay_cpu_csv).resolve()

    if args.out_prefix is None:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_prefix = repo_root / f"tools/output/mpc_receding_horizon_{ts}"
    else:
        out_prefix = Path(args.out_prefix).resolve()
    out_prefix.parent.mkdir(parents=True, exist_ok=True)
    track_out = out_prefix.with_name(f"{out_prefix.name}_track.png")
    tracking_out = out_prefix.with_name(f"{out_prefix.name}_tracking.png")

    state_rows = load_state_rows(state_csv)
    selected_row_idx = pick_snapshot(state_rows, args.seed, args.past_steps) if args.idx is None else None
    if args.idx is not None:
        index_lookup = {int(row["idx"]): i for i, row in enumerate(state_rows)}
        if args.idx not in index_lookup:
            raise RuntimeError(f"Replay idx {args.idx} not found in {state_csv}")
        selected_row_idx = index_lookup[args.idx]
        selected_idx_value = args.idx
    else:
        selected_idx_value = int(state_rows[selected_row_idx]["idx"])

    helper_bin = ensure_export_helper(repo_root)
    with tempfile.TemporaryDirectory(prefix="mpc_plan_snapshot_", dir=str(repo_root / "tools/output")) as tmpdir:
        plan_csv = Path(tmpdir) / f"plan_snapshot_idx_{selected_idx_value}.csv"
        export_plan_snapshot(helper_bin, state_csv, selected_idx_value, plan_csv)
        plan_meta, plan_rows = read_plan_snapshot(plan_csv)

    replay_by_idx: dict[int, dict] = {}
    if replay_cpu_csv.exists():
        with replay_cpu_csv.open() as f:
            replay_by_idx = {int(r["idx"]): r for r in csv.DictReader(f)}

    make_track_plot(
        state_rows=state_rows,
        replay_by_idx=replay_by_idx,
        selected_row_idx=selected_row_idx,
        selected_idx_value=selected_idx_value,
        plan_meta=plan_meta,
        plan_rows=plan_rows,
        out_path=track_out,
        past_steps=args.past_steps,
    )
    make_tracking_plot(
        state_rows=state_rows,
        replay_by_idx=replay_by_idx,
        selected_row_idx=selected_row_idx,
        selected_idx_value=selected_idx_value,
        plan_rows=plan_rows,
        out_path=tracking_out,
        past_steps=args.past_steps,
    )

    print(f"Saved track plot to {track_out}")
    print(f"Saved tracking plot to {tracking_out}")
    print(f"Selected replay idx: {selected_idx_value}")


if __name__ == "__main__":
    main()
