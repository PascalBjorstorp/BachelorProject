#!/usr/bin/env python3
"""Build multiple-shooting train/validation/test segments without crossing laps."""
from __future__ import annotations

import argparse
from pathlib import Path
import numpy as np
import pandas as pd


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("observed", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    # Shorter windows than before: less integration drift, more independent
    # samples, and they measure the one-to-two control-cycle predictive accuracy
    # that actually governs closed-loop MPC parity. Whole-lap drift is reported
    # separately by the replay diagnostics.
    parser.add_argument("--duration", type=float, default=0.6)
    parser.add_argument("--spacing", type=float, default=0.25)
    parser.add_argument("--minimum-speed", type=float, default=1.5)
    parser.add_argument("--maximum-position-correction", type=float, default=0.30)
    # Validity envelope. Below ~1.8 m/s the velocity state differentiated from the
    # ICP pose is dominated by noise, and above ~35 deg sideslip the car is in a
    # near-spin outside single-track-model validity. Both occur only at the
    # full-lock hairpin apex; including them corrupts the tyre/geometry fit and
    # produces an RMSE tail that is reference noise, not model error. They are
    # excluded so the fit and the reported metrics reflect the trustworthy
    # racing envelope. Raise --max-sideslip-deg to 180 to disable.
    parser.add_argument("--window-minimum-speed", type=float, default=1.8)
    parser.add_argument("--max-sideslip-deg", type=float, default=35.0)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    d = pd.read_csv(args.observed).reset_index(drop=True)
    track_length = max(d.path_s.max() - d.path_s.min(), 1.0)
    # path_s itself is wrapped. Reconstruct lap crossings from a large negative jump.
    lap = np.zeros(len(d), dtype=int)
    current = 0
    for i in range(1, len(d)):
        if d.path_s.iloc[i] - d.path_s.iloc[i - 1] < -0.5 * track_length:
            current += 1
        lap[i] = current
    unique_laps = sorted(np.unique(lap))
    rows = []
    next_time = -np.inf
    for i, row in d.iterrows():
        if row.t_s < next_time or row.speed_body_mps < args.minimum_speed:
            continue
        if np.hypot(row.delta_x_from_prior, row.delta_y_from_prior) > args.maximum_position_correction:
            continue
        t_end = row.t_s + args.duration
        j = int(np.searchsorted(d.t_s.to_numpy(), t_end, side="right") - 1)
        if j <= i + 5 or lap[j] != lap[i]:
            continue
        window = d.iloc[i:j + 1]
        # Reject windows outside the reliable-reference / single-track validity
        # envelope (full-lock near-spins at the hairpin apex).
        if "icp_speed" in d.columns:
            if float(window.icp_speed.min()) < args.window_minimum_speed:
                continue
            if float(np.degrees(window.icp_beta.abs().max())) > args.max_sideslip_deg:
                continue
        # Retain both straight/longitudinal and turning/lateral segments; fit stages filter later.
        split = "train" if lap[i] < max(unique_laps) - 1 else ("validation" if lap[i] == max(unique_laps) - 1 else "test")
        rows.append({
            "segment_id": len(rows), "start_index": i, "end_index": j,
            "t_start_s": row.t_s, "t_end_s": d.t_s.iloc[j], "duration_s": d.t_s.iloc[j] - row.t_s,
            "lap": int(lap[i]), "split": split,
            "mean_abs_steer": float(np.mean(np.abs(d.cmd_steering_angle.iloc[i:j + 1]))),
            "mean_speed": float(np.mean(d.speed_body_mps.iloc[i:j + 1])),
            "mean_abs_yaw_rate": float(np.mean(np.abs(d.odom_wz.iloc[i:j + 1]))),
        })
        next_time = row.t_s + args.spacing
    out = pd.DataFrame(rows)
    out.to_csv(args.output, index=False)
    print(f"Wrote {len(out)} segments over {len(unique_laps)} lap indices to {args.output}")
    print(out.groupby(["split"]).size())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
