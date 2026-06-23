#!/usr/bin/env python3
"""Convert a fitted race-track CSV (centerline + left/right widths) into a
ROS occupancy-grid map (.pgm + .yaml) holding ONE clean drivable corridor
that loops, usable for raceline optimisation.

CSV columns (semicolon separated, comma-separated header):
    x_ref_m, y_ref_m, width_left_m, width_right_m,
    psi_racetraj_rad, s_racetraj_m, kappa_racetraj_radpm, vx_racetraj_mps

The raw left/right distances in this CSV over-reach: in the switchbacks the
listed half-width is larger than the gap to the rest of the track, so a naive
fill swallows the infield and merges passes into a blob.  To get a single,
non-self-overlapping looping corridor we clamp each side's half-width to the
local *clearance* (distance to the nearest non-adjacent part of the track),
then fill the corridor between the clamped walls.
"""
import argparse
import os
import numpy as np
from scipy.spatial import cKDTree
from scipy.ndimage import uniform_filter1d
from PIL import Image, ImageDraw

# ROS map pixel convention: 254 = free (white), 0 = occupied (black).
FREE = 254
OCCUPIED = 0


def load_csv(path):
    rows = []
    with open(path) as f:
        next(f)  # header
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = [p for p in line.replace(";", " ").split() if p]
            rows.append([float(p) for p in parts[:6]])
    a = np.array(rows)
    return a[:, 0], a[:, 1], a[:, 2], a[:, 3], a[:, 4], a[:, 5]  # x,y,wl,wr,psi,s


def resample_loop(x, y, wl, wr, s, ds):
    """Resample the closed track to a fine, even arc-length spacing `ds`.
    The CSV waypoints are sparse (~0.6 m); without this the corridor pinches
    to slivers between sections. Heading is recomputed from the dense
    centerline so the wall normals stay accurate."""
    close_seg = np.hypot(x[0] - x[-1], y[0] - y[-1])
    total = s[-1] + close_seg
    s_ext = np.append(s, total)
    fx = lambda v: np.interp(np.arange(0, total, ds), s_ext, np.append(v, v[0]))
    xf, yf, wlf, wrf = fx(x), fx(y), fx(wl), fx(wr)
    # periodic central-difference tangent -> heading
    dx = np.roll(xf, -1) - np.roll(xf, 1)
    dy = np.roll(yf, -1) - np.roll(yf, 1)
    psif = np.arctan2(dy, dx)
    sf = np.arange(0, total, ds)
    return xf, yf, wlf, wrf, psif, sf


def clearance(x, y, s, guard):
    """For each point, distance to the nearest centerline point that is more
    than `guard` metres away ALONG the track (so true neighbours are ignored
    and only other passes / the opposite wall count)."""
    pts = np.column_stack((x, y))
    tree = cKDTree(pts)
    n = len(x)
    total = s[-1] + (s[-1] - s[-2])  # approx loop length
    clr = np.full(n, np.inf)
    for i in range(n):
        # arc distance to every other point (wrapped)
        ds = np.abs(s - s[i])
        ds = np.minimum(ds, total - ds)
        far = ds > guard
        if not np.any(far):
            continue
        d = np.hypot(x[far] - x[i], y[far] - y[i])
        clr[i] = d.min()
    return clr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("-o", "--out", default=None,
                    help="output basename (default: csv name)")
    ap.add_argument("-r", "--resolution", type=float, default=0.025,
                    help="metres per pixel (default 0.025)")
    ap.add_argument("-m", "--margin", type=float, default=0.6,
                    help="border margin in metres (default 0.6)")
    ap.add_argument("--clamp", type=float, default=0.45,
                    help="max half-width as fraction of local clearance "
                         "(default 0.45; <0.5 keeps a wall between passes)")
    ap.add_argument("--guard", type=float, default=2.5,
                    help="arc-length guard (m) for clearance neighbours")
    ap.add_argument("--smooth", type=int, default=5,
                    help="moving-average window (samples) on the half-widths")
    ap.add_argument("--no-clamp", action="store_true",
                    help="use raw widths (will overlap)")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="multiply both half-widths (widen/narrow corridor)")
    ap.add_argument("--ds", type=float, default=None,
                    help="resample spacing in m (default: 1 pixel)")
    ap.add_argument("--mode", choices=("fill", "walls"), default="fill")
    ap.add_argument("--wall", type=float, default=0.05,
                    help="wall thickness in metres for --mode walls")
    args = ap.parse_args()

    x, y, wl, wr, psi, s = load_csv(args.csv)
    out = args.out or os.path.splitext(args.csv)[0]
    res = args.resolution

    # --- resample the sparse waypoints to ~1 px so the corridor never pinches
    ds = args.ds if args.ds else res
    raw_spacing = (s[-1] + np.hypot(x[0] - x[-1], y[0] - y[-1])) / len(x)
    x, y, wl, wr, psi, s = resample_loop(x, y, wl, wr, s, ds)
    # smoothing window given in samples scales with the new density
    smooth = max(1, int(round(args.smooth * raw_spacing / ds)))

    wl = wl * args.scale
    wr = wr * args.scale

    # --- clamp each side to the local clearance so the corridor never merges
    if not args.no_clamp:
        clr = clearance(x, y, s, args.guard)
        cap = args.clamp * clr
        wl = np.minimum(wl, cap)
        wr = np.minimum(wr, cap)

    # light smoothing to remove per-sample spikes (wrap-around for the loop)
    if smooth > 1:
        wl = uniform_filter1d(wl, smooth, mode="wrap")
        wr = uniform_filter1d(wr, smooth, mode="wrap")

    # unit normal pointing left of travel
    nx, ny = -np.sin(psi), np.cos(psi)
    left = np.column_stack((x + wl * nx, y + wl * ny))
    right = np.column_stack((x - wr * nx, y - wr * ny))

    # close the loop
    left = np.vstack((left, left[0]))
    right = np.vstack((right, right[0]))

    all_pts = np.vstack((left, right))
    min_x, min_y = all_pts.min(axis=0) - args.margin
    max_x, max_y = all_pts.max(axis=0) + args.margin
    width_px = int(np.ceil((max_x - min_x) / res))
    height_px = int(np.ceil((max_y - min_y) / res))
    origin_x, origin_y = min_x, min_y

    def to_px(pt):
        col = (pt[:, 0] - origin_x) / res
        row = (max_y - pt[:, 1]) / res
        return np.column_stack((col, row))

    left_px, right_px = to_px(left), to_px(right)

    if args.mode == "fill":
        img = Image.new("L", (width_px, height_px), OCCUPIED)
        draw = ImageDraw.Draw(img)
        for i in range(len(left_px) - 1):
            quad = [tuple(left_px[i]), tuple(left_px[i + 1]),
                    tuple(right_px[i + 1]), tuple(right_px[i])]
            draw.polygon(quad, fill=FREE)
    else:
        img = Image.new("L", (width_px, height_px), FREE)
        draw = ImageDraw.Draw(img)
        wall_px = max(1, int(round(args.wall / res)))
        r = wall_px / 2.0
        for pts in (left_px, right_px):
            draw.line([tuple(p) for p in pts], fill=OCCUPIED,
                      width=wall_px, joint="curve")
            for cx, cy in pts:
                draw.ellipse((cx - r, cy - r, cx + r, cy + r), fill=OCCUPIED)

    pgm_path, yaml_path = out + ".pgm", out + ".yaml"
    img.save(pgm_path)
    with open(yaml_path, "w") as f:
        f.write(f"image: {os.path.basename(pgm_path)}\n")
        f.write("mode: trinary\n")
        f.write(f"resolution: {res}\n")
        f.write(f"origin: [{origin_x:.6f}, {origin_y:.6f}, 0.000000]\n")
        f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")

    free_frac = np.count_nonzero(np.asarray(img) == FREE) / (width_px * height_px)
    print(f"wrote {pgm_path}  ({width_px}x{height_px} px @ {res} m/px, mode={args.mode})")
    print(f"wrote {yaml_path}  origin=({origin_x:.3f}, {origin_y:.3f})")
    print(f"corridor half-width: left {wl.min():.2f}-{wl.max():.2f} m, "
          f"right {wr.min():.2f}-{wr.max():.2f} m")
    print(f"free area: {free_frac*100:.1f}% of image")


if __name__ == "__main__":
    main()
