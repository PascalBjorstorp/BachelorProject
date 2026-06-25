#!/usr/bin/env python3
"""Continuous open-loop replay of the whole recorded run through the fitted plant.

This is the most direct "does the simulation reproduce real life" test: drive the
identified plant with the EXACT recorded /drive commands, starting from the real
initial state, and integrate continuously (no per-window resets) for the entire
run. Because it uses the real commands it cannot crash and it follows the real
control history. The output is the position/heading error versus the ICP truth and
an overlay plot.

A continuous open-loop integration accumulates drift, so this is reported both as
the continuous whole-run error and as a periodically re-anchored error (reset to
truth every --reanchor seconds), which reflects the short-horizon predictive
accuracy that governs closed-loop behaviour.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from geometry import wrap_angle
from plant_model import PlantParameters, initial_state_from_observation, rollout


def replay(observed: pd.DataFrame, drive: pd.DataFrame, p: PlantParameters,
           sim_dt: float, reanchor_s: float) -> pd.DataFrame:
    cmd_t = drive.t_s.to_numpy()
    cmd_delta = drive.steering_angle.to_numpy()
    cmd_accel = drive.acceleration.to_numpy()
    t = observed.t_s.to_numpy()
    n = len(observed)
    x_pred = np.empty(n); y_pred = np.empty(n); yaw_pred = np.empty(n)
    speed_pred = np.empty(n)
    anchor_idx = 0
    while anchor_idx < n - 1:
        # Re-seed from truth at each anchor, then integrate forward continuously.
        end_t = t[anchor_idx] + reanchor_s
        end_idx = int(np.searchsorted(t, end_t, side="right") - 1)
        end_idx = min(max(end_idx, anchor_idx + 1), n - 1)
        sl = observed.iloc[anchor_idx:end_idx + 1]
        pred = rollout(sl.t_s.to_numpy(), cmd_t, cmd_delta, cmd_accel,
                       initial_state_from_observation(sl.iloc[0], p), p, sim_dt=sim_dt)
        rng = slice(anchor_idx, end_idx + 1)
        x_pred[rng] = pred["x"]; y_pred[rng] = pred["y"]
        yaw_pred[rng] = pred["yaw"]; speed_pred[rng] = pred["speed"]
        anchor_idx = end_idx
    out = observed[["t_s", "x", "y", "yaw"]].copy()
    out["x_pred"] = x_pred; out["y_pred"] = y_pred
    out["yaw_pred"] = yaw_pred; out["speed_pred"] = speed_pred
    out["pos_err"] = np.hypot(out.x_pred - out.x, out.y_pred - out.y)
    out["yaw_err"] = np.abs(wrap_angle(out.yaw_pred.to_numpy() - out.yaw.to_numpy()))
    return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--observed", type=Path, required=True)
    parser.add_argument("--drive", type=Path, required=True)
    parser.add_argument("--parameters", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--metrics", type=Path, required=True)
    parser.add_argument("--plot", type=Path)
    parser.add_argument("--sim-dt", type=float, default=0.002)
    parser.add_argument("--reanchor", type=float, default=2.0,
                        help="Re-seed from truth every N seconds (use a huge value for fully continuous).")
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    observed = pd.read_csv(args.observed)
    drive = pd.read_csv(args.drive).sort_values("t_s")
    p = PlantParameters.from_json(args.parameters)
    out = replay(observed, drive, p, args.sim_dt, args.reanchor)
    out.to_csv(args.output, index=False)

    metrics = {
        "reanchor_s": args.reanchor, "n_samples": int(len(out)),
        "position_rmse_m": float(np.sqrt(np.mean(out.pos_err ** 2))),
        "position_mae_m": float(np.mean(out.pos_err)),
        "position_p95_m": float(np.percentile(out.pos_err, 95)),
        "yaw_rmse_deg": float(np.degrees(np.sqrt(np.mean(out.yaw_err ** 2)))),
        "yaw_median_deg": float(np.degrees(np.median(out.yaw_err))),
    }
    args.metrics.write_text(json.dumps(metrics, indent=2) + "\n")
    print(json.dumps(metrics, indent=2))

    if args.plot:
        fig, ax = plt.subplots(1, 2, figsize=(15, 6))
        ax[0].plot(out.x, out.y, label="real (ICP)", linewidth=1.5)
        ax[0].plot(out.x_pred, out.y_pred, "--", label="simulation", linewidth=1.0)
        ax[0].set_aspect("equal", adjustable="box"); ax[0].legend()
        ax[0].set_title(f"Open-loop replay (re-anchor {args.reanchor}s): sim vs real")
        ax[0].set_xlabel("x [m]"); ax[0].set_ylabel("y [m]")
        ax[1].plot(out.t_s, out.pos_err, label="position error [m]")
        ax[1].plot(out.t_s, np.degrees(out.yaw_err) / 100.0, label="heading error [deg/100]")
        ax[1].set_xlabel("time [s]"); ax[1].legend(); ax[1].set_title("Replay error vs time")
        fig.tight_layout(); fig.savefig(args.plot, dpi=150); plt.close(fig)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
