#!/usr/bin/env python3
"""Merge the width profiles of two track CSVs that describe the SAME loop.

At every point along the track we lean toward whichever file's width is
locally smoother (less spiky), with a gentle crossfade at the transitions
-- "take the cleaner parts from each". The two files may have different
point density / start offset / heading sign, so the files are aligned by
nearest position along the centreline, not by row index.

Geometry (x, y, s) is taken from the denser `--base` file; only the widths
are blended. Output is a CSV in the same format, ready for csv_to_map.py.
"""
import argparse
import numpy as np
from scipy.spatial import cKDTree
from scipy.ndimage import uniform_filter1d


def load(path):
    rows = []
    with open(path) as f:
        next(f)
        for line in f:
            line = line.strip()
            if not line:
                continue
            p = [float(v) for v in line.replace(";", " ").split()[:6]]
            rows.append(p)
    a = np.array(rows)
    return a[:, 0], a[:, 1], a[:, 2], a[:, 3], a[:, 5]  # x, y, wl, wr, s


def roughness(v, win):
    """Local spikiness: smoothed magnitude of the 2nd difference (periodic)."""
    d = np.abs(np.roll(v, -1) - 2.0 * v + np.roll(v, 1))
    return uniform_filter1d(d, win, mode="wrap")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("base", help="denser file -> provides geometry")
    ap.add_argument("other", help="second file -> width donor")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--rough-win", type=int, default=9,
                    help="window (samples) for the roughness estimate")
    ap.add_argument("--blend-win", type=int, default=21,
                    help="window (samples) to soften the file-selection seams")
    args = ap.parse_args()

    bx, by, bwl, bwr, bs = load(args.base)
    ox, oy, owl, owr, os_ = load(args.other)

    # map the OTHER file's widths onto the base points by nearest position
    idx = cKDTree(np.column_stack((ox, oy))).query(np.column_stack((bx, by)))[1]
    owl_b, owr_b = owl[idx], owr[idx]

    # combined roughness of each file on the base grid (lower = cleaner)
    rb = roughness(bwl, args.rough_win) + roughness(bwr, args.rough_win)
    ro = roughness(owl_b, args.rough_win) + roughness(owr_b, args.rough_win)

    # weight toward the BASE file where the OTHER is rougher; crossfade
    w = ro / (rb + ro + 1e-9)
    w = uniform_filter1d(w, args.blend_win, mode="wrap")

    mwl = w * bwl + (1 - w) * owl_b
    mwr = w * bwr + (1 - w) * owr_b

    with open(args.out, "w") as f:
        f.write("x_ref_m; y_ref_m; width_left_m; width_right_m; "
                "psi_racetraj_rad; s_racetraj_m\n")
        for i in range(len(bx)):
            f.write(f"{bx[i]:.4f}; {by[i]:.4f}; {mwl[i]:.4f}; {mwr[i]:.4f}; "
                    f"0.0000; {bs[i]:.4f}\n")

    print(f"wrote {args.out}  ({len(bx)} pts)")
    print(f"mean blend weight toward base ({args.base.split('/')[-1]}): "
          f"{w.mean():.2f}  (1.0 = all base, 0.0 = all other)")
    print(f"merged half-width: left {mwl.min():.2f}-{mwl.max():.2f} m, "
          f"right {mwr.min():.2f}-{mwr.max():.2f} m")


if __name__ == "__main__":
    main()
