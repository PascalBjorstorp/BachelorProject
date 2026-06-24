#!/usr/bin/env python3
"""
reconstruct_raceline_csv_from_local_raceline.py
===============================================
Reconstructs the global raceline CSV from a bag's /local_raceline topic.

The /local_raceline topic is a nav_msgs/Path publishing a rolling LOCAL window
of the global optimized raceline. The fields are packed into each PoseStamped:

    position.x      -> x         (m)
    position.y      -> y         (m)
    position.z      -> vx        (m/s, reference velocity)
    orientation.x   -> d_left    (m, left wall distance)
    orientation.y   -> d_right   (m, right wall distance)

s (arc-length), psi (heading) and kappa (curvature) are NOT stored in the
message; they are geometric and are recomputed here. ax is not transmitted
either, so it is reconstructed as the longitudinal acceleration ax = v*dv/ds.

Because each message only covers a short window, we recover the full loop by
taking the first pose of every message (a clean ~2cm-spaced sample of the
global line as the car drives around), isolating one lap, then resampling to a
uniform arc-length grid. The published line is static across laps, so one lap
fully reconstructs the original CSV.

Output column order matches the repo's raceline format:
    s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2, d_left_m, d_right_m

Usage:
    python3 tools/reconstruct_raceline_csv_from_local_raceline.py BAG_DIR_OR_MCAP [OUT_CSV]
"""

import os
import sys
import math
import numpy as np

from scipy.signal import savgol_filter

import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

DS_OUT = 0.02            # output arc-length resolution (m), matches original raceline
START_RADIUS = 0.30      # m, lap-return detection radius
SG_WIN_M = 0.60          # m, Savitzky-Golay window for path/derivative smoothing
SG_ORDER = 3             # Savitzky-Golay polynomial order


def find_mcap(path):
    if os.path.isfile(path):
        return path
    for f in sorted(os.listdir(path)):
        if f.endswith(".mcap"):
            return os.path.join(path, f)
    raise FileNotFoundError(f"No .mcap found in {path}")


def read_first_poses(mcap_path, topic="/local_raceline"):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=mcap_path, storage_id="mcap"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    tmap = {t.name: t.type for t in r.get_all_topics_and_types()}
    if topic not in tmap:
        raise KeyError(f"Topic {topic} not in bag (have: {sorted(tmap)})")
    RL = get_message(tmap[topic])
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    rows = []  # x, y, v, d_left, d_right
    while r.has_next():
        _, data, _ = r.read_next()
        m = deserialize_message(data, RL)
        if not m.poses:
            continue
        p = m.poses[0].pose
        rows.append((p.position.x, p.position.y, p.position.z,
                     p.orientation.x, p.orientation.y))
    return np.asarray(rows, dtype=float)


def lap_boundaries(F):
    """Indices where the path returns near its starting point -> lap boundaries."""
    x, y = F[:, 0], F[:, 1]
    # Anchor on the first sample after the car has left the startup region, so the
    # first detected boundary is a genuine lap return rather than the parked start.
    moved = np.hypot(x - x[0], y - y[0]) > 1.0
    start = int(np.argmax(moved)) if moved.any() else 0
    ax, ay = x[start], y[start]
    near = np.hypot(x - ax, y - ay) < START_RADIUS
    # rising edges into the start region, after having left it once
    bounds = []
    outside = False
    for i in range(start, len(F)):
        if not near[i]:
            outside = True
        elif outside:
            bounds.append(i)
            outside = False
    return start, bounds


def isolate_one_lap(F, lap_index=None):
    """Return the slice of F for one clean MIDDLE lap (skips the ramp-up lap)."""
    start, bounds = lap_boundaries(F)
    # boundaries split the run into laps: [start..b0], [b0..b1], ...
    edges = [start] + bounds
    n_laps = len(edges) - 1
    if n_laps < 1:
        return F[start:]
    if lap_index is None:
        lap_index = n_laps // 2          # a middle lap, never the ramp-up first lap
    lap_index = max(1, min(lap_index, n_laps - 1)) if n_laps > 1 else 0
    a, b = edges[lap_index], edges[lap_index + 1]
    print(f"  detected {n_laps} laps; using lap {lap_index} (samples {a}..{b})")
    return F[a:b + 1]


def resample_closed(lap, ds_out):
    """Resample a closed loop to uniform arc-length spacing."""
    xy = lap[:, :2]
    # drop consecutive duplicate points
    keep = np.concatenate([[True], np.any(np.diff(xy, axis=0) != 0, axis=1)])
    lap = lap[keep]
    xy = lap[:, :2]
    # close the loop
    if np.hypot(*(xy[0] - xy[-1])) > 1e-6:
        lap = np.vstack([lap, lap[0]])
        xy = lap[:, :2]
    seg = np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = s[-1]
    n = int(round(total / ds_out))
    s_new = np.linspace(0.0, total, n, endpoint=False)
    out = {}
    out["s"] = s_new
    out["x"] = np.interp(s_new, s, lap[:, 0])
    out["y"] = np.interp(s_new, s, lap[:, 1])
    out["v"] = np.interp(s_new, s, lap[:, 2])
    out["dl"] = np.interp(s_new, s, lap[:, 3])
    out["dr"] = np.interp(s_new, s, lap[:, 4])
    out["total"] = total
    return out


def sg_win(n):
    """Odd Savitzky-Golay window in samples, clamped to the data length."""
    w = int(round(SG_WIN_M / DS_OUT))
    w |= 1                     # force odd
    w = max(SG_ORDER + 2 | 1, min(w, (n // 2) * 2 - 1))
    return max(w, SG_ORDER + 2)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    bag = sys.argv[1]
    mcap = find_mcap(bag)
    out_csv = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
        bag if os.path.isdir(bag) else os.path.dirname(bag),
        "reconstructed_raceline.csv")

    print(f"Reading {mcap} ...")
    F = read_first_poses(mcap)
    print(f"  {len(F)} /local_raceline messages")

    lap = isolate_one_lap(F)
    print(f"  isolated one lap: {len(lap)} samples")

    g = resample_closed(lap, DS_OUT)
    n = len(g["s"])
    print(f"  track length: {g['total']:.3f} m  ->  {n} points @ {DS_OUT} m")

    x, y, v = g["x"], g["y"], g["v"]
    s = g["s"]
    w = sg_win(n)
    print(f"  smoothing window: {w} samples ({w * DS_OUT:.2f} m)")

    # Smooth the path (closed loop via wrap) before differentiating: the sampled
    # first-poses carry ~cm jitter that would otherwise blow up the curvature.
    xs = savgol_filter(x, w, SG_ORDER, mode="wrap")
    ys = savgol_filter(y, w, SG_ORDER, mode="wrap")

    # Arc-length derivatives (uniform spacing -> delta=DS_OUT)
    dx = savgol_filter(x, w, SG_ORDER, deriv=1, delta=DS_OUT, mode="wrap")
    dy = savgol_filter(y, w, SG_ORDER, deriv=1, delta=DS_OUT, mode="wrap")
    ddx = savgol_filter(x, w, SG_ORDER, deriv=2, delta=DS_OUT, mode="wrap")
    ddy = savgol_filter(y, w, SG_ORDER, deriv=2, delta=DS_OUT, mode="wrap")

    psi = np.arctan2(dy, dx)
    speed2 = dx * dx + dy * dy
    kappa = (dx * ddy - dy * ddx) / np.power(np.maximum(speed2, 1e-12), 1.5)

    # longitudinal accel ax = v * dv/ds (not transmitted; reconstructed)
    vs = savgol_filter(v, w, SG_ORDER, mode="wrap")
    dv = savgol_filter(v, w, SG_ORDER, deriv=1, delta=DS_OUT, mode="wrap")
    ax = vs * dv
    x, y = xs, ys

    with open(out_csv, "w") as f:
        f.write("# Reconstructed from bag /local_raceline (x,y,vx,d_left,d_right published; "
                "s,psi,kappa,ax recomputed)\n")
        f.write("# s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2,d_left_m,d_right_m\n")
        for i in range(n):
            f.write(f"{s[i]:.6f},{x[i]:.6f},{y[i]:.6f},{psi[i]:.6f},"
                    f"{kappa[i]:.6f},{v[i]:.6f},{ax[i]:.6f},"
                    f"{g['dl'][i]:.6f},{g['dr'][i]:.6f}\n")
    print(f"\n  Saved: {out_csv}  ({n} rows)")
    print(f"  v range:     {v.min():.2f}..{v.max():.2f} m/s")
    print(f"  kappa range: {kappa.min():.3f}..{kappa.max():.3f} rad/m")
    print(f"  d_left:      {g['dl'].min():.2f}..{g['dl'].max():.2f} m")
    print(f"  d_right:     {g['dr'].min():.2f}..{g['dr'].max():.2f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
