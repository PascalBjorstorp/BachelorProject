#!/usr/bin/env python3
"""Plot localization, open-loop replay, and residual diagnostics."""
from __future__ import annotations

import argparse
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from geometry import wrap_angle


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--observed", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--predictions", type=Path)
    parser.add_argument("--raceline", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    observed = pd.read_csv(args.observed)

    fig, ax = plt.subplots(figsize=(8, 7))
    if args.raceline:
        r = np.loadtxt(args.raceline, delimiter=",", comments="#")
        ax.plot(r[:, 1], r[:, 2], linewidth=1.0, label="raceline")
    sc = ax.scatter(observed.x, observed.y, c=observed.rmse_m, s=3, label="ICP pose")
    fig.colorbar(sc, ax=ax, label="ICP RMSE [m]")
    ax.set_aspect("equal", adjustable="box"); ax.set_xlabel("map x [m]"); ax.set_ylabel("map y [m]")
    ax.legend(); ax.set_title("Scan-to-map ICP trajectory and quality")
    fig.tight_layout(); fig.savefig(args.output / "icp_trajectory_quality.png", dpi=200); plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(observed.t_s, observed.rmse_m, label="ICP RMSE")
    ax.plot(observed.t_s, np.hypot(observed.delta_x_from_prior, observed.delta_y_from_prior), label="ICP-EKF correction")
    ax.set_xlabel("time [s]"); ax.set_ylabel("m"); ax.legend(); ax.set_title("Localization quality")
    fig.tight_layout(); fig.savefig(args.output / "icp_quality_time.png", dpi=200); plt.close(fig)

    if args.predictions and args.predictions.exists():
        p = pd.read_csv(args.predictions)
        pos_error = np.hypot(p.x_pred - p.x_obs, p.y_pred - p.y_obs)
        yaw_error = wrap_angle(p.yaw_pred - p.yaw_obs)
        fig, ax = plt.subplots(figsize=(8, 7))
        ax.plot(p.x_obs, p.y_obs, label="observed")
        ax.plot(p.x_pred, p.y_pred, label="open-loop model")
        ax.set_aspect("equal", adjustable="box"); ax.set_xlabel("map x [m]"); ax.set_ylabel("map y [m]")
        ax.legend(); ax.set_title("Multiple-shooting open-loop prediction")
        fig.tight_layout(); fig.savefig(args.output / "open_loop_overlay.png", dpi=200); plt.close(fig)

        fig, ax = plt.subplots(figsize=(11, 6))
        ax.plot(p.t_s, pos_error, label="position error [m]")
        ax.plot(p.t_s, np.abs(yaw_error), label="|heading error| [rad]")
        ax.plot(p.t_s, np.abs(p.speed_pred - p.speed_obs), label="|speed error| [m/s]")
        ax.set_xlabel("time [s]"); ax.legend(); ax.set_title("Open-loop residuals")
        fig.tight_layout(); fig.savefig(args.output / "open_loop_residuals.png", dpi=200); plt.close(fig)

        fig, ax = plt.subplots(figsize=(8, 5))
        ax.scatter(p.speed_obs, pos_error, s=4, alpha=0.6)
        ax.set_xlabel("observed speed [m/s]"); ax.set_ylabel("position error [m]")
        ax.set_title("Position residual versus speed")
        fig.tight_layout(); fig.savefig(args.output / "residual_vs_speed.png", dpi=200); plt.close(fig)
    print(f"Plots written to {args.output}")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
