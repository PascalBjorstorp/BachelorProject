#!/usr/bin/env python3
"""
Minimum-time one-click trajectory optimization pipeline for F1Tenth MPC.

Works with ANY map (.pgm/.yaml) -- no pre-made TUM track CSV required.
This copy is intentionally fixed to TUM's ``mintime`` optimization method.

Pipeline steps:
  Step 0: Extract centerline + track widths from map image  (NEW)
  Step 1: Run TUM global_racetrajectory_optimization with opt_type='mintime'
  Step 2: Convert to MPC format (psi += pi/2, delimiter, clamp velocity)
  Step 3: Add wall distances via ray-cast
  Step 4: Verify output trajectory

Output CSV format (7 or 9 columns):
  s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2 [, d_left_m, d_right_m]

Usage:
    # Edit the User settings block in main(), then run:
    python3 optimize_trajectory_mintime.py

Requirements:
    pip install numpy opencv-python scipy pyyaml
    pip install -r global_racetrajectory_optimization/requirements.txt
"""

import argparse
import csv
import math
import os
import re
import subprocess
import sys

os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib')
os.makedirs(os.environ['MPLCONFIGDIR'], exist_ok=True)

import cv2
import numpy as np
import yaml
from scipy.interpolate import splprep, splev, CubicSpline
from scipy.interpolate import splprep, splev, CubicSpline, UnivariateSpline
from scipy.ndimage import gaussian_filter1d


MIN_TIME_OPT_TYPE = 'mintime'


def thin_binary_mask(binary_mask):
    """Return a 1-pixel skeleton from a binary mask using best available method."""
    mask_u8 = (binary_mask.astype(np.uint8) > 0).astype(np.uint8) * 255

    # Preferred: OpenCV contrib Zhang-Suen thinning.
    if hasattr(cv2, 'ximgproc') and hasattr(cv2.ximgproc, 'thinning'):
        skel = cv2.ximgproc.thinning(
            mask_u8,
            thinningType=cv2.ximgproc.THINNING_ZHANGSUEN,
        )
        return skel, 'cv2.ximgproc.THINNING_ZHANGSUEN'

    # Fallback: morphological skeletonization (no extra dependency).
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    img = mask_u8.copy()
    skel = np.zeros_like(img)
    max_iter = img.size
    for _ in range(max_iter):
        eroded = cv2.erode(img, element)
        opened = cv2.dilate(eroded, element)
        temp = cv2.subtract(img, opened)
        skel = cv2.bitwise_or(skel, temp)
        img = eroded
        if cv2.countNonZero(img) == 0:
            break

    return skel, 'morphological_skeleton_fallback'

# =============================================================================
#  Utility helpers (kept from original)
# =============================================================================

def find_workspace_root():
    """Walk up from this script to find the workspace root (contains f1tenth_planning/)."""
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(10):
        if os.path.isdir(os.path.join(d, 'f1tenth_planning')):
            return d
        d = os.path.dirname(d)
    return None


def run_step(label, cmd, cwd=None):
    """Run a subprocess, stream output, and abort on failure."""
    print(f"\n{'=' * 64}")
    print(f"  {label}")
    print(f"{'=' * 64}")
    print(f"  Command: {' '.join(cmd)}")
    if cwd:
        print(f"  Working dir: {cwd}")
    print()
    env = os.environ.copy()
    env['MPLBACKEND'] = 'Agg'  # Prevent matplotlib from blocking on plt.show()
    env.setdefault('MPLCONFIGDIR', '/tmp/matplotlib')
    os.makedirs(env['MPLCONFIGDIR'], exist_ok=True)
    result = subprocess.run(cmd, cwd=cwd, env=env)
    if result.returncode != 0:
        print(f"\n  ERROR: {label} failed (exit code {result.returncode})")
        sys.exit(result.returncode)


def max_abs_curvature_closed(path_xy):
    """Return max absolute curvature for a closed polyline."""
    pts = np.asarray(path_xy, dtype=float)
    if pts.ndim != 2 or pts.shape[0] < 3:
        return 0.0

    p_prev = np.roll(pts, 1, axis=0)
    p_next = np.roll(pts, -1, axis=0)
    a = pts - p_prev
    b = p_next - pts
    c = p_next - p_prev

    cross = a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]
    denom = (np.linalg.norm(a, axis=1)
             * np.linalg.norm(b, axis=1)
             * np.linalg.norm(c, axis=1))
    kappa = np.abs(2.0 * cross / np.maximum(denom, 1e-9))
    return float(np.max(kappa))
def closed_path_arclength(points_xy):
    """
    Return cumulative arc length for a closed path, including the duplicated
    final point.
    """
    pts = np.asarray(points_xy, dtype=float)
    closed = np.vstack([pts, pts[0]])
    ds = np.linalg.norm(np.diff(closed, axis=0), axis=1)

    if np.any(ds < 1e-9):
        keep = np.concatenate([[True], ds[:-1] > 1e-9])
        pts = pts[keep]
        closed = np.vstack([pts, pts[0]])
        ds = np.linalg.norm(np.diff(closed, axis=0), axis=1)

    s = np.concatenate([[0.0], np.cumsum(ds)])
    return closed, s


def analytical_curvature_from_periodic_splines(path_xy, sample_spacing=0.01, smooth_s=0.0):
    """
    Compute true geometric curvature from periodic x(s), y(s) splines.

    This is independent of waypoint spacing and catches the actual curvature
    that a spline-based optimizer will see.
    """
    pts = np.asarray(path_xy, dtype=float)

    if len(pts) < 6:
        return np.array([0.0]), np.array([[0.0, 0.0]])

    closed, s = closed_path_arclength(pts)
    total_length = float(s[-1])

    if total_length <= 1e-6:
        return np.array([0.0]), pts.copy()

    # CubicSpline with periodic boundary conditions requires the first and last
    # values to match exactly. closed already duplicates pts[0].
    sx = CubicSpline(s, closed[:, 0], bc_type='periodic')
    sy = CubicSpline(s, closed[:, 1], bc_type='periodic')

    n_samples = max(50, int(np.ceil(total_length / sample_spacing)))
    s_eval = np.linspace(0.0, total_length, n_samples, endpoint=False)

    dx = sx(s_eval, 1)
    dy = sy(s_eval, 1)
    ddx = sx(s_eval, 2)
    ddy = sy(s_eval, 2)

    denom = np.maximum((dx * dx + dy * dy) ** 1.5, 1e-9)
    kappa = (dx * ddy - dy * ddx) / denom

    xy_eval = np.column_stack([sx(s_eval), sy(s_eval)])
    return kappa, xy_eval


def max_abs_analytical_curvature(path_xy, sample_spacing=0.01):
    """
    Max absolute curvature of a closed path using periodic analytical splines.
    """
    kappa, _ = analytical_curvature_from_periodic_splines(
        path_xy,
        sample_spacing=sample_spacing,
    )
    return float(np.max(np.abs(kappa)))


def remove_closed_path_duplicates(path_xy, min_dist=1e-4):
    """
    Remove consecutive duplicate or near-duplicate points from a closed path.
    """
    pts = np.asarray(path_xy, dtype=float)

    if len(pts) == 0:
        return pts

    cleaned = [pts[0]]
    for pt in pts[1:]:
        if np.linalg.norm(pt - cleaned[-1]) > min_dist:
            cleaned.append(pt)

    if len(cleaned) > 1 and np.linalg.norm(cleaned[-1] - cleaned[0]) < min_dist:
        cleaned.pop()

    return np.asarray(cleaned, dtype=float)


def resample_closed_path_periodic(path_xy, spacing):
    """
    Resample closed path using periodic cubic splines and uniform arc length.
    """
    pts = remove_closed_path_duplicates(path_xy)

    if len(pts) < 6:
        raise ValueError("Need at least 6 unique points for periodic resampling")

    closed, s = closed_path_arclength(pts)
    total_length = float(s[-1])

    sx = CubicSpline(s, closed[:, 0], bc_type='periodic')
    sy = CubicSpline(s, closed[:, 1], bc_type='periodic')

    n_points = max(20, int(np.ceil(total_length / spacing)))
    s_new = np.linspace(0.0, total_length, n_points, endpoint=False)

    out = np.column_stack([sx(s_new), sy(s_new)])
    return out, total_length, total_length / n_points


def smooth_closed_path_gaussian(path_xy, sigma_points):
    """
    Simple wrap-around Gaussian smoothing for closed paths.

    This is good at removing pixel stair-steps before the higher-quality
    spline smoother runs.
    """
    pts = np.asarray(path_xy, dtype=float)

    if sigma_points <= 0.0:
        return pts.copy()

    x = gaussian_filter1d(pts[:, 0], sigma=sigma_points, mode='wrap')
    y = gaussian_filter1d(pts[:, 1], sigma=sigma_points, mode='wrap')

    return np.column_stack([x, y])


def smooth_closed_path_by_arclength(path_xy, spacing, smooth_s):
    """
    Smooth x(s), y(s) as periodic splines and resample uniformly.

    Larger smooth_s means more smoothing. This removes hard pixel-angle turns.
    """
    pts = remove_closed_path_duplicates(path_xy)

    if len(pts) < 8:
        raise ValueError("Need at least 8 points for spline smoothing")

    closed, s = closed_path_arclength(pts)
    total_length = float(s[-1])

    # Use periodic cubic splines for final geometric path.
    # If smooth_s <= 0, this becomes interpolation.
    if smooth_s <= 0.0:
        sx = CubicSpline(s, closed[:, 0], bc_type='periodic')
        sy = CubicSpline(s, closed[:, 1], bc_type='periodic')
    else:
        # UnivariateSpline does not support periodic constraints directly.
        # To avoid seam artifacts, extend the signal by wrapping it.
        s_core = s[:-1]
        x_core = pts[:, 0]
        y_core = pts[:, 1]

        s_ext = np.concatenate([
            s_core - total_length,
            s_core,
            s_core + total_length,
        ])
        x_ext = np.concatenate([x_core, x_core, x_core])
        y_ext = np.concatenate([y_core, y_core, y_core])

        sx_raw = UnivariateSpline(s_ext, x_ext, k=3, s=smooth_s)
        sy_raw = UnivariateSpline(s_ext, y_ext, k=3, s=smooth_s)

        n_dense = max(200, int(np.ceil(total_length / max(spacing * 0.5, 0.005))))
        s_dense = np.linspace(0.0, total_length, n_dense, endpoint=False)
        dense = np.column_stack([sx_raw(s_dense), sy_raw(s_dense)])

        # Refit periodic cubic spline to enforce exact closure.
        dense_closed, dense_s = closed_path_arclength(dense)
        sx = CubicSpline(dense_s, dense_closed[:, 0], bc_type='periodic')
        sy = CubicSpline(dense_s, dense_closed[:, 1], bc_type='periodic')
        total_length = float(dense_s[-1])

    n_points = max(20, int(np.ceil(total_length / spacing)))
    s_new = np.linspace(0.0, total_length, n_points, endpoint=False)

    out = np.column_stack([sx(s_new), sy(s_new)])
    return out, total_length, total_length / n_points


def centerline_clearance_ok(centerline, map_img, resolution, origin,
                            wall_thresh, max_ray_distance,
                            car_width, wall_clearance):
    """
    Check whether a candidate centerline stays far enough from both walls.
    """
    w_right, w_left = measure_track_widths(
        centerline=centerline,
        map_img=map_img,
        resolution=resolution,
        origin=origin,
        max_dist=max_ray_distance,
        wall_thresh=wall_thresh,
    )

    min_required = 0.5 * float(car_width) + float(wall_clearance)

    min_right = float(np.min(w_right))
    min_left = float(np.min(w_left))

    ok = min_right >= min_required and min_left >= min_required

    return ok, min_right, min_left


def condition_centerline_for_curvature_and_clearance(
    centerline,
    map_img,
    resolution,
    origin,
    wall_thresh,
    curvlim,
    target_spacing,
    max_ray_distance,
    car_width,
    wall_clearance,
    kappa_sample_spacing=0.01,
):
    """
    Robust centerline conditioner.

    Purpose:
      - remove 90-degree pixel turns
      - enforce analytical curvature <= curvlim
      - reject candidates that move too close to walls
      - return a uniformly sampled closed path

    This should run immediately after GVD centerline extraction.
    """
    if curvlim is None or curvlim <= 0.0:
        print("  Centerline conditioning: curvlim missing; only resampling.")
        out, _, _ = resample_closed_path_periodic(centerline, target_spacing)
        return out

    print("\n  --- Curvature-aware centerline conditioning ---")

    raw = remove_closed_path_duplicates(centerline)
    raw, raw_len, raw_spacing = resample_closed_path_periodic(raw, target_spacing)

    raw_kappa = max_abs_analytical_curvature(
        raw,
        sample_spacing=kappa_sample_spacing,
    )

    print(
        f"  Raw centerline after uniform resampling: "
        f"{len(raw)} points, length={raw_len:.2f} m, "
        f"spacing={raw_spacing:.3f} m, "
        f"true_max_kappa={raw_kappa:.3f} 1/m"
    )

    # First stage: cheap circular Gaussian smoothing. This kills pixel staircases
    # and 90-degree skeleton artifacts before spline smoothing.
    best = None

    gaussian_sigmas_m = [
        0.00,
        0.05,
        0.08,
        0.12,
        0.16,
        0.22,
        0.30,
        0.40,
        0.55,
        0.75,
    ]

    for sigma_m in gaussian_sigmas_m:
        sigma_points = sigma_m / max(raw_spacing, 1e-6)
        candidate = smooth_closed_path_gaussian(raw, sigma_points)
        candidate, _, _ = resample_closed_path_periodic(candidate, target_spacing)

        kappa = max_abs_analytical_curvature(
            candidate,
            sample_spacing=kappa_sample_spacing,
        )

        clearance_ok, min_right, min_left = centerline_clearance_ok(
            candidate,
            map_img,
            resolution,
            origin,
            wall_thresh,
            max_ray_distance,
            car_width,
            wall_clearance,
        )

        print(
            f"  Gaussian candidate sigma={sigma_m:.2f} m: "
            f"true_max_kappa={kappa:.3f}, "
            f"min_right={min_right:.3f}, min_left={min_left:.3f}, "
            f"clearance_ok={clearance_ok}"
        )

        if clearance_ok:
            if best is None or kappa < best["kappa"]:
                best = {
                    "path": candidate,
                    "method": f"gaussian_sigma_{sigma_m:.2f}m",
                    "kappa": kappa,
                    "min_right": min_right,
                    "min_left": min_left,
                }

        if kappa <= curvlim and clearance_ok:
            print(
                f"  Accepted centerline conditioning: gaussian sigma={sigma_m:.2f} m, "
                f"true_max_kappa={kappa:.3f} <= {curvlim:.3f}"
            )
            return candidate

    # Second stage: stronger arc-length spline smoothing.
    #
    # The scale of smooth_s depends on track length and coordinate scale.
    # These values intentionally cover a wide range.
    spline_s_values = [
        0.001,
        0.003,
        0.01,
        0.03,
        0.10,
        0.30,
        1.00,
        3.00,
        10.0,
        30.0,
        100.0,
        300.0,
        1000.0,
    ]

    for smooth_s in spline_s_values:
        try:
            candidate, cand_len, cand_spacing = smooth_closed_path_by_arclength(
                raw,
                spacing=target_spacing,
                smooth_s=smooth_s,
            )
        except Exception as exc:
            print(
                f"  Spline candidate s={smooth_s:.4g} failed: "
                f"{type(exc).__name__}: {exc}"
            )
            continue

        kappa = max_abs_analytical_curvature(
            candidate,
            sample_spacing=kappa_sample_spacing,
        )

        clearance_ok, min_right, min_left = centerline_clearance_ok(
            candidate,
            map_img,
            resolution,
            origin,
            wall_thresh,
            max_ray_distance,
            car_width,
            wall_clearance,
        )

        print(
            f"  Spline candidate s={smooth_s:.4g}: "
            f"true_max_kappa={kappa:.3f}, "
            f"length={cand_len:.2f} m, spacing={cand_spacing:.3f} m, "
            f"min_right={min_right:.3f}, min_left={min_left:.3f}, "
            f"clearance_ok={clearance_ok}"
        )

        if clearance_ok:
            if best is None or kappa < best["kappa"]:
                best = {
                    "path": candidate,
                    "method": f"spline_s_{smooth_s:.4g}",
                    "kappa": kappa,
                    "min_right": min_right,
                    "min_left": min_left,
                }

        if kappa <= curvlim and clearance_ok:
            print(
                f"  Accepted centerline conditioning: spline s={smooth_s:.4g}, "
                f"true_max_kappa={kappa:.3f} <= {curvlim:.3f}"
            )
            return candidate

    if best is not None:
        print(
            "  WARNING: No candidate fully satisfied curvlim. "
            f"Using best clearance-valid candidate from {best['method']}: "
            f"true_max_kappa={best['kappa']:.3f}, "
            f"curvlim={curvlim:.3f}, "
            f"min_right={best['min_right']:.3f}, "
            f"min_left={best['min_left']:.3f}"
        )
        return best["path"]

    raise RuntimeError(
        "Centerline conditioning failed: no smoothed candidate had enough "
        "wall clearance. The extracted GVD line may be too close to walls, "
        "or wall_clearance/car_width is too conservative for this map."
    )

def read_curvlim_from_racecar_ini(ini_path):
    """Parse curvlim from racecar.ini; return None if unavailable."""
    if not ini_path or not os.path.exists(ini_path):
        return None
    with open(ini_path, 'r') as f:
        content = f.read()
    number = r'([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)'
    match = re.search(r'"curvlim"\s*:\s*' + number, content)
    if not match:
        return None
    try:
        return float(match.group(1))
    except ValueError:
        return None


def max_abs_kappa_from_tum_csv(csv_path):
    """Return max absolute kappa from a TUM traj_race_cl.csv file."""
    max_kappa = None
    with open(csv_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = [p.strip() for p in line.split(';')]
            if len(parts) < 5:
                continue
            try:
                kappa = abs(float(parts[4]))
            except ValueError:
                continue
            max_kappa = kappa if max_kappa is None else max(max_kappa, kappa)
    return max_kappa


# =============================================================================
#  Step 0 -- Map loading, boundary extraction, centerline, track widths
# =============================================================================

def compute_centerline_thinned_loop(
    map_img,
    resolution,
    origin,
    wall_thresh,
    num_points,
    gvd_debug_path=None,
):
    """
    Compute the track centerline using a GVD (Generalised Voronoi Diagram):

      1. Label wall obstacle components (outer wall vs inner island)
      2. Per-obstacle Euclidean distance transforms → GVD boundary
         (free pixels equidistant from both obstacles)
      3. Zhang-Suen thinning of the GVD boundary → clean 1-px skeleton ring
      4. Walk the skeleton ring from the widest point (max EDT) →
         ordered pixel loop
      5. B-spline smooth + uniform resample

    Parameters
    ----------
    map_img    : uint8 grayscale — free pixels >= wall_thresh
    resolution : float — metres per pixel
    origin     : [x0, y0, …] — world-frame origin of pixel (0,0)
    wall_thresh: int — pixel value >= this counts as free space
    num_points : int — desired uniformly-spaced output waypoints

    Returns
    -------
    centerline  : (num_points, 2) world-frame coordinates
    half_widths : (num_points,)   EDT-based half-track-width in metres
    """
    from scipy.ndimage import label as _sc_label
    from scipy.ndimage import distance_transform_edt as _edt_scipy

    h, w = map_img.shape[:2]
    free = (map_img >= wall_thresh).astype(np.uint8)
    edt  = cv2.distanceTransform(free, cv2.DIST_L2, 5)

    def _save_gvd_debug(pixel_path=None, start_rc=None):
        if not gvd_debug_path:
            return
        os.makedirs(os.path.dirname(gvd_debug_path), exist_ok=True)
        vis = cv2.cvtColor(map_img, cv2.COLOR_GRAY2BGR)
        vis[gvd_mask] = (0, 220, 255)   # GVD boundary in yellow
        vis[skel_mat > 0] = (0, 0, 255) # Skeleton in red
        if pixel_path is not None:
            for rr, cc in pixel_path:
                vis[rr, cc] = (0, 255, 0)  # Ring walk in green
        if start_rc is not None:
            cv2.circle(vis, (start_rc[1], start_rc[0]), 4, (255, 0, 0), -1)
        if cv2.imwrite(gvd_debug_path, vis):
            print(f"  Saved GVD debug visualization: {gvd_debug_path}")
        else:
            print(f"  WARNING: Failed to save GVD debug visualization: {gvd_debug_path}")

    # ---- 1. Label wall obstacles -----------------------------------------
    wall = (1 - free).astype(np.uint8)
    wall_labeled, n_wall = _sc_label(wall, structure=np.ones((3, 3), int))
    print(f"  Wall obstacle components: {n_wall}")
    if n_wall < 2:
        raise RuntimeError(
            f"Expected ≥2 wall components (outer + inner), got {n_wall}. "
            "Map may not have a ring-shaped corridor."
        )

    # ---- 2. Per-obstacle EDT → GVD boundary ------------------------------
    # Distance from every pixel to each obstacle
    comp_sizes = []
    for wid in range(1, n_wall + 1):
        comp_sizes.append((wid, int((wall_labeled == wid).sum())))
    comp_sizes.sort(key=lambda x: x[1], reverse=True)
    keep_ids = [wid for wid, _ in comp_sizes[:2]]
    print(f"  Using wall components for GVD: {keep_ids} (sizes: {[s for _, s in comp_sizes[:2]]})")

    dists = []
    for wid in keep_ids:
        # _edt_scipy: distance to nearest 0-pixel; invert so obstacle=0
        dists.append(_edt_scipy((wall_labeled != wid).astype(np.uint8)))

    # GVD boundary: free pixels where the two closest obstacles are
    # nearly equidistant.  For 2 obstacles this is simply |d1 - d2| < thr.
    d1, d2 = dists[0], dists[1]
    diff_map = np.abs(d1.astype(float) - d2.astype(float))
    gvd_thresh = 2.0   # pixels
    gvd_mask = (free == 1) & (diff_map < gvd_thresh)
    n_gvd = int(gvd_mask.sum())
    print(f"  GVD boundary pixels (thresh={gvd_thresh}): {n_gvd}")

    # ---- 3. Zhang-Suen thinning of GVD boundary -------------------------
    skel_mat, thin_method = thin_binary_mask(gvd_mask)
    print(f"  Thinning method: {thin_method}")
    skel_set = set(map(tuple, np.column_stack(np.where(skel_mat > 0)).tolist()))
    n_skel = len(skel_set)
    print(f"  GVD skeleton: {n_skel} pixels")
    _save_gvd_debug(pixel_path=None, start_rc=None)
    if n_skel < 20:
        raise RuntimeError("GVD skeleton too sparse; check wall_thresh or map.")

    # ---- 4. Extract the main ring from the skeleton ----------------------
    # The skeleton is one connected component forming a ring (centerline)
    # with optional short cross-links at junctions.  A simple greedy walk
    # can get trapped at multi-pixel junctions.
    #
    # Robust approach:
    #   1. Build adjacency graph.
    #   2. Find *all* simple cycles using a DFS.
    #   3. Pick the longest cycle — that's the main ring.
    #
    # Since the graph is sparse (~2k nodes, mostly degree 2), this is fast.

    from collections import deque

    def _nbrs8(rc, pset):
        """8-connected neighbours of pixel rc that are in pset."""
        r, c = rc
        return [(r+dr, c+dc)
                for dr in (-1, 0, 1) for dc in (-1, 0, 1)
                if (dr or dc) and (r+dr, c+dc) in pset]

    # Build adjacency dict
    adj = {p: set(_nbrs8(p, skel_set)) for p in skel_set}

    # Identify junction pixels (degree >= 3)
    junc_pixels = {p for p in adj if len(adj[p]) >= 3}
    print(f"  Junction pixels (degree>=3): {len(junc_pixels)}")

    # Strategy: simplify the graph by contracting degree-2 chains into
    # single edges between junctions.  Then find the main cycle on the
    # simplified graph.  Finally expand back.
    #
    # If there are no junctions (pure ring), just walk it directly.

    if not junc_pixels:
        # Pure ring — simple walk
        ring_arr = np.array(list(skel_set))
        edt_vals = edt[ring_arr[:, 0], ring_arr[:, 1]]
        start_idx = int(np.argmax(edt_vals))
        start_rc = tuple(ring_arr[start_idx].tolist())
        visited_w = {start_rc}
        pixel_path = [start_rc]
        cur = start_rc
        closed_loop = False
        while True:
            unvis = [(edt[n[0], n[1]], n) for n in adj[cur] if n not in visited_w]
            if unvis:
                unvis.sort(reverse=True)
                nxt = unvis[0][1]
                visited_w.add(nxt)
                pixel_path.append(nxt)
                cur = nxt
            else:
                if start_rc in adj[cur] and len(pixel_path) > 10:
                    closed_loop = True
                break
    else:
        # --- Contract degree-2 chains into edges ---
        # Group junction pixels into clusters (connected via other junctions)
        junc_remaining = set(junc_pixels)
        junc_clusters = []
        while junc_remaining:
            seed = next(iter(junc_remaining))
            cluster = set()
            q = deque([seed])
            while q:
                p = q.popleft()
                if p in cluster:
                    continue
                cluster.add(p)
                junc_remaining.discard(p)
                for nb in adj[p]:
                    if nb in junc_remaining:
                        q.append(nb)
            junc_clusters.append(frozenset(cluster))

        # Map pixel → cluster id
        px_to_cid = {}
        for cid, cl in enumerate(junc_clusters):
            for p in cl:
                px_to_cid[p] = cid

        print(f"  Junction clusters: {len(junc_clusters)}, "
              f"sizes: {sorted(len(c) for c in junc_clusters)}")

        # Trace chains: start from each junction pixel, follow degree-2
        # pixels until reaching another junction pixel.
        chains = []  # (cid_a, cid_b, [pixel list including endpoints])
        chain_visited_px = set()

        for cid_a, cluster_a in enumerate(junc_clusters):
            for jp in cluster_a:
                for nb in adj[jp]:
                    if nb in junc_pixels:
                        continue  # skip junction-junction direct edges for now
                    if nb in chain_visited_px:
                        continue
                    # Trace a chain from jp through nb
                    chain = [jp, nb]
                    chain_visited_px.add(nb)
                    cur = nb
                    prev = jp
                    while True:
                        nxts = [n for n in adj[cur] if n != prev and n not in chain_visited_px]
                        # Filter: if we hit a junction pixel, stop
                        junc_nxts = [n for n in nxts if n in junc_pixels]
                        non_junc = [n for n in nxts if n not in junc_pixels]
                        if junc_nxts:
                            # Reached a junction — pick the one that continues the ring
                            chain.append(junc_nxts[0])
                            cid_b = px_to_cid[junc_nxts[0]]
                            chains.append((cid_a, cid_b, chain))
                            break
                        elif non_junc:
                            nxt = non_junc[0]
                            chain_visited_px.add(nxt)
                            chain.append(nxt)
                            prev = cur
                            cur = nxt
                        else:
                            # Dead end (spur) — discard this chain
                            break

        # Also add direct junction-to-junction edges (between different clusters)
        direct_edges = set()
        for cid_a, cluster_a in enumerate(junc_clusters):
            for jp in cluster_a:
                for nb in adj[jp]:
                    if nb in junc_pixels and px_to_cid[nb] != cid_a:
                        pair = (min(cid_a, px_to_cid[nb]), max(cid_a, px_to_cid[nb]))
                        if pair not in direct_edges:
                            direct_edges.add(pair)
                            chains.append((pair[0], pair[1], [jp, nb]))

        # Filter out self-loops (chains from cluster X back to cluster X)
        ring_chains = [(a, b, c) for a, b, c in chains if a != b]
        self_loops  = [(a, b, c) for a, b, c in chains if a == b]
        if self_loops:
            print(f"  Dropped {len(self_loops)} self-loop chain(s)")
        chains = ring_chains

        print(f"  Chains: {len(chains)}")
        for i, (ca, cb, ch) in enumerate(chains):
            print(f"    Chain {i}: cluster {ca}→{cb}, {len(ch)} px")

        # Build a simplified multigraph: nodes = cluster ids, edges = chains
        n_clusters = len(junc_clusters)
        cluster_adj = {cid: [] for cid in range(n_clusters)}
        for ci, (ca, cb, ch) in enumerate(chains):
            cluster_adj[ca].append((ci, cb))
            cluster_adj[cb].append((ci, ca))

        # Find the main cycle using DFS on the simplified graph
        # We want the longest simple cycle
        best_cycle = None

        def dfs_cycle(start_cid):
            nonlocal best_cycle
            # stack: (current_cluster, list_of_chain_indices, set_of_used_chains)
            stack = [(start_cid, [], set())]
            while stack:
                cur_cid, path, used = stack.pop()
                for ci, next_cid in cluster_adj[cur_cid]:
                    if ci in used:
                        continue
                    if next_cid == start_cid and len(path) > 0:
                        # Found a cycle
                        cycle = path + [ci]
                        if best_cycle is None or len(cycle) > len(best_cycle):
                            best_cycle = cycle
                        continue
                    # Avoid revisiting clusters (except start)
                    visited_clusters = {start_cid}
                    for pci in path:
                        ca, cb, _ = chains[pci]
                        visited_clusters.add(ca)
                        visited_clusters.add(cb)
                    if next_cid in visited_clusters and next_cid != start_cid:
                        continue
                    new_used = used | {ci}
                    stack.append((next_cid, path + [ci], new_used))

        # Try starting from each cluster to find the best cycle
        for start in range(n_clusters):
            dfs_cycle(start)
            if best_cycle and len(best_cycle) >= n_clusters:
                break  # found a Hamiltonian cycle, can't do better

        if best_cycle is None:
            # Fallback: use the longest contour on the skeleton image.
            skel_bin = (skel_mat > 0).astype(np.uint8)
            contours, _ = cv2.findContours(
                skel_bin, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE
            )
            if contours:
                best_cnt = max(contours, key=lambda c: len(c))
                cnt = best_cnt.reshape(-1, 2)  # x, y
                pixel_path = [(int(y), int(x)) for x, y in cnt]
                closed_loop = len(pixel_path) > 20
                print(f"  Fallback contour walk: {len(pixel_path)} pixels")
            else:
                _save_gvd_debug(pixel_path=None, start_rc=None)
                raise RuntimeError("Could not find a cycle in the simplified skeleton graph.")
        else:
            print(f"  Best cycle: {len(best_cycle)} chains (of {len(chains)})")

            # Reconstruct pixel path from the chain cycle
            # Determine correct chain orientation for each step
            pixel_path = []
            # Determine traversal direction for each chain in the cycle
            first_chain = chains[best_cycle[0]]
            # Start cluster for the cycle
            if len(best_cycle) > 1:
                second_chain = chains[best_cycle[1]]
                # Figure out shared cluster between chain 0 and chain 1
                c0_set = {first_chain[0], first_chain[1]}
                c1_set = {second_chain[0], second_chain[1]}
                shared = c0_set & c1_set
                if shared:
                    # Chain 0 should end at the shared cluster
                    end_cluster = next(iter(shared))
                    if first_chain[1] == end_cluster:
                        pixel_path.extend(first_chain[2])
                    else:
                        pixel_path.extend(reversed(first_chain[2]))
                else:
                    pixel_path.extend(first_chain[2])
            else:
                pixel_path.extend(first_chain[2])

            for step in range(1, len(best_cycle)):
                ci = best_cycle[step]
                ca, cb, ch = chains[ci]
                # Connect to previous path
                last_px = pixel_path[-1]
                # Check which end of chain is closer to last_px
                d_start = abs(ch[0][0] - last_px[0]) + abs(ch[0][1] - last_px[1])
                d_end = abs(ch[-1][0] - last_px[0]) + abs(ch[-1][1] - last_px[1])
                if d_start <= d_end:
                    # Forward direction: skip first pixel if it overlaps
                    start_idx = 1 if ch[0] == last_px or d_start == 0 else 0
                    pixel_path.extend(ch[start_idx:])
                else:
                    # Reverse direction
                    rev = list(reversed(ch))
                    start_idx = 1 if rev[0] == last_px or d_end == 0 else 0
                    pixel_path.extend(rev[start_idx:])

            closed_loop = True  # we found a cycle

    # Re-order path to start from the pixel with highest EDT
    path_arr = np.array(pixel_path)
    path_edt = edt[path_arr[:, 0], path_arr[:, 1]]
    best_start = int(np.argmax(path_edt))
    pixel_path = pixel_path[best_start:] + pixel_path[:best_start]

    nr0, nc0 = pixel_path[0]
    print(f"  Start pixel ({nr0},{nc0}), EDT={edt[nr0,nc0]:.1f}px, "
          f"world=({origin[0]+nc0*resolution:.2f},"
          f"{origin[1]+(h-1-nr0)*resolution:.2f})")
    print(f"  Ring walk: {len(pixel_path)} pixels, closed={closed_loop}")

    _save_gvd_debug(pixel_path=pixel_path, start_rc=(nr0, nc0))

    if not closed_loop:
        raise RuntimeError(
            f"Ring walk collected {len(pixel_path)} pixels but could not "
            "close the loop.  The GVD skeleton may be disconnected."
        )

    # ---- 5. Convert to world coords + half-widths -----------------------
    pixel_path = np.array(pixel_path)
    rows = pixel_path[:, 0]
    cols = pixel_path[:, 1]

    wx = origin[0] + cols * resolution
    wy = origin[1] + (h - 1 - rows) * resolution
    hw = edt[rows, cols] * resolution   # half-widths in metres

    skel_world = np.column_stack([wx, wy])
    pg     = float(np.linalg.norm(skel_world[-1] - skel_world[0]))
    px_len = float(np.sum(np.linalg.norm(np.diff(skel_world, axis=0), axis=1)))
    print(f"  Pixel path: {len(pixel_path)} pts, {px_len:.1f} m, "
          f"loop_gap={pg:.3f} m")

    # ---- 6. Remove near-duplicates → B-spline smooth + resample ---------
    dd   = np.linalg.norm(np.diff(skel_world, axis=0), axis=1)
    keep = np.concatenate([[True], dd > 0.005])
    skel_world = skel_world[keep]
    hw         = hw[keep]

    closed = np.vstack([skel_world, skel_world[0]])
    try:
        pixel_noise_m = max(0.5 * resolution, 0.01)
        s_gvd = len(closed) * pixel_noise_m**2

        tck, _ = splprep(
            [closed[:, 0], closed[:, 1]],
            s=s_gvd,
            per=True,
        )
    except Exception as exc:
        raise RuntimeError(f"B-spline on GVD skeleton loop failed: {exc}") from exc

    u_new = np.linspace(0, 1, num_points, endpoint=False)
    centerline      = np.array(splev(u_new, tck)).T
    half_widths_out = np.interp(u_new,
                                np.linspace(0, 1, len(hw), endpoint=False),
                                hw)
    return centerline, half_widths_out

def load_map(map_yaml_path):
    """
    Load a ROS2 map from its YAML descriptor and PGM image.

    Returns
    -------
    img : np.ndarray  -- grayscale image (uint8)
    resolution : float -- meters per pixel
    origin : np.ndarray -- [x, y, theta] world-frame origin
    """
    with open(map_yaml_path, 'r') as f:
        config = yaml.safe_load(f)

    resolution = float(config['resolution'])
    origin = np.array(config['origin'], dtype=float)

    image_path = config['image']
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(map_yaml_path), image_path)

    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        if not os.path.exists(image_path):
            raise FileNotFoundError(f"Map image file does not exist: {image_path}")
        try:
            from PIL import Image
            pil_img = Image.open(image_path).convert('L')
            img = np.array(pil_img)
            print(f"  Warning: cv2.imread failed, loaded via PIL instead")
        except Exception:
            raise FileNotFoundError(
                f"Could not load map image: {image_path}\n"
                f"  File exists but cv2.imread returned None.\n"
                f"  Check: pip install opencv-python (or opencv-python-headless)"
            )

    # Read occupied_thresh from map YAML (ROS convention).
    # Used to compute a consistent wall threshold across the pipeline.
    occupied_thresh = float(config.get('occupied_thresh', 0.65))

    # --- SLAM map preprocessing ---
    # 0. Convert grey "unknown" pixels to wall (black).
    #    SLAM maps use: 0=occupied (wall), 205=unknown, 254=free.
    #    Unknown regions are outside the track and must be treated as wall
    #    so the GVD sees exactly 2 obstacle components (outer + inner).
    free_thresh = 240  # pixels >= this are free space
    grey_mask = (img > 0) & (img < free_thresh)
    if grey_mask.sum() > 0:
        img[grey_mask] = 0
        print(f"  Converted {grey_mask.sum()} grey/unknown pixels to wall (SLAM cleanup)")

    # 1. Close small gaps in walls (SLAM maps often have 1-2 pixel wall gaps)
    wall_thresh_tmp = int(255 * (1.0 - occupied_thresh))
    wall_mask = (img < wall_thresh_tmp).astype(np.uint8)
    kernel = np.ones((3, 3), np.uint8)
    wall_closed = cv2.morphologyEx(wall_mask, cv2.MORPH_CLOSE, kernel, iterations=1)
    gap_pixels = (wall_closed > 0) & (wall_mask == 0) & (img != 0)
    if gap_pixels.sum() > 0:
        # Fill gaps: set gap pixels to wall (black)
        img[gap_pixels] = 0
        print(f"  Closed {gap_pixels.sum()} wall gap pixels")

    # 2. Pad map if free space touches any edge (SLAM map cropped too tight)
    h, w = img.shape
    free_mask = (img >= 240)
    pad_needed = [
        free_mask[0, :].any(),   # top
        free_mask[-1, :].any(),  # bottom
        free_mask[:, 0].any(),   # left
        free_mask[:, -1].any(),  # right
    ]
    if any(pad_needed):
        pad_px = 15  # pad by 15 pixels = 0.75m at 0.05m/px
        # Pad with grey (205) = non-free, treated as wall
        new_img = np.full((h + 2 * pad_px, w + 2 * pad_px), 205, dtype=np.uint8)
        new_img[pad_px:pad_px + h, pad_px:pad_px + w] = img
        img = new_img
        # Adjust origin to account for the padding
        origin[0] -= pad_px * resolution
        origin[1] -= pad_px * resolution
        sides = ['top', 'bottom', 'left', 'right']
        padded_sides = [s for s, p in zip(sides, pad_needed) if p]
        print(f"  Padded map by {pad_px}px ({padded_sides}): "
              f"new size {img.shape[1]}x{img.shape[0]}")

    return img, resolution, origin, occupied_thresh


def extract_outer_boundary(img, resolution, origin, wall_thresh=140):
    """
    Extract the outer track boundary from a SLAM map image.

    The outer wall is the largest free-space contour.  This is used to
    estimate the track perimeter for auto-scaling the number of centerline
    waypoints.

    Parameters
    ----------
    wall_thresh : int
        Pixel values >= wall_thresh are considered free space.

    Returns
    -------
    outer_world : np.ndarray, shape (N, 2) -- outer boundary in world coords
    """
    free_space = (img >= wall_thresh).astype(np.uint8)
    contours, _ = cv2.findContours(
        free_space, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
    )
    if not contours:
        raise RuntimeError("No contours found in the map image")

    # Largest contour = outer wall of the track
    outer = max(contours, key=cv2.contourArea).reshape(-1, 2)

    # Convert pixel coords → world coords
    outer_world = np.zeros((len(outer), 2))
    outer_world[:, 0] = outer[:, 0] * resolution + origin[0]
    outer_world[:, 1] = (img.shape[0] - outer[:, 1]) * resolution + origin[1]
    print(f"  Outer boundary: {len(outer_world)} points")
    return outer_world





def detect_winding_direction(centerline):
    """
    Detect the winding direction (CW vs CCW) of a closed centerline.

    Uses the signed area (shoelace formula).
    Returns 'ccw' if counter-clockwise, 'cw' if clockwise.
    """
    n = len(centerline)
    signed_area = 0.0
    for i in range(n):
        j = (i + 1) % n
        signed_area += centerline[i, 0] * centerline[j, 1]
        signed_area -= centerline[j, 0] * centerline[i, 1]
    return 'ccw' if signed_area > 0 else 'cw'


def measure_track_widths(centerline, map_img, resolution, origin,
                         max_dist=5.0, wall_thresh=140,
                         left_normals=None,
                         boundary_smooth_sigma_px=4.0):
    """
    Measure track widths by intersecting centerline normals with the extracted
    track boundary contours.  Falls back to pixel ray-casting only if a normal
    does not intersect a boundary cleanly.

    Parameters
    ----------
    centerline : np.ndarray, shape (N, 2)
    map_img : np.ndarray -- grayscale image
    resolution : float -- meters per pixel
    origin : np.ndarray -- map origin [x, y, theta]
    max_dist : float -- maximum ray-cast distance in metres
    wall_thresh : int -- pixel values >= this are free space (must match
        compute_wall_distances.py threshold to avoid grey-zone errors)
    left_normals : np.ndarray, optional -- normalized normals pointing to the
        left side of the track.  If omitted, normals are estimated from the
        centerline.  Supplying these lets the width measurement match the
        exact spline normals used by the optimizer.
    boundary_smooth_sigma_px : float -- smoothing applied to extracted wall
        contours before normal intersection.  This removes pixel/branch spikes
        from optimizer constraints without smoothing the centerline.

    Returns
    -------
    w_right : np.ndarray, shape (N,) -- distance to right wall
    w_left  : np.ndarray, shape (N,) -- distance to left wall
    """
    # Consistent wall threshold: pixels below wall_thresh are walls.
    free = (map_img >= wall_thresh).astype(np.uint8)

    # Robust casting mask: close tiny wall gaps / pinholes for ray tests.
    free_cast = cv2.morphologyEx(
        free,
        cv2.MORPH_CLOSE,
        np.ones((3, 3), np.uint8),
        iterations=1,
    )

    n = len(centerline)
    h, w = map_img.shape[:2]

    def _world_to_grid_xy(pt_xy):
        """World [x,y] -> image [row,col], with nearest-pixel rounding."""
        col_f = (pt_xy[0] - origin[0]) / resolution
        row_f = h - (pt_xy[1] - origin[1]) / resolution
        return int(np.round(row_f)), int(np.round(col_f))

    def _cross2(a, b):
        """2D cross product with numpy broadcasting."""
        return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]

    # Extract all white free-space boundary loops: the outer edge plus holes
    # such as the inner island.  These are the walls normals are allowed to hit.
    contours, _ = cv2.findContours(
        (free * 255).astype(np.uint8),
        cv2.RETR_LIST,
        cv2.CHAIN_APPROX_NONE,
    )
    segment_starts = []
    segment_ends = []
    kept_contours = 0
    for contour in contours:
        contour_xy_raw = contour.reshape(-1, 2)
        if len(contour_xy_raw) < 3 or abs(cv2.contourArea(contour_xy_raw)) < 5.0:
            continue
        contour_xy = contour_xy_raw.astype(float)
        if boundary_smooth_sigma_px and len(contour_xy) > 8:
            from scipy.ndimage import gaussian_filter1d
            contour_xy = np.column_stack([
                gaussian_filter1d(
                    contour_xy[:, 0],
                    sigma=boundary_smooth_sigma_px,
                    mode='wrap',
                ),
                gaussian_filter1d(
                    contour_xy[:, 1],
                    sigma=boundary_smooth_sigma_px,
                    mode='wrap',
                ),
            ])
        pts_world = np.column_stack([
            origin[0] + contour_xy[:, 0].astype(float) * resolution,
            origin[1] + (h - contour_xy[:, 1].astype(float)) * resolution,
        ])
        segment_starts.append(pts_world)
        segment_ends.append(np.roll(pts_world, -1, axis=0))
        kept_contours += 1

    if not segment_starts:
        raise RuntimeError("No free-space boundary contours found for width measurement.")

    seg_a = np.vstack(segment_starts)
    seg_b = np.vstack(segment_ends)
    seg_vec = seg_b - seg_a
    print(f"  Boundary contours: {kept_contours}, segments: {len(seg_a)}")

    if left_normals is None:
        # Compute tangent vectors via periodic central differences on a lightly
        # smoothed centerline to reduce normal jitter in raw width estimates.
        # This is less noisy than forward differences and avoids a hard normal
        # discontinuity at the wrap-around index.
        from scipy.ndimage import gaussian_filter1d
        cx = gaussian_filter1d(centerline[:, 0], sigma=1.2, mode='wrap')
        cy = gaussian_filter1d(centerline[:, 1], sigma=1.2, mode='wrap')
        cl_smooth = np.column_stack([cx, cy])
        tangents = np.roll(cl_smooth, -1, axis=0) - np.roll(cl_smooth, 1, axis=0)
        t_len = np.linalg.norm(tangents, axis=1, keepdims=True)
        tangents = tangents / np.maximum(t_len, 1e-6)

        # Normal: rotate tangent 90 deg CCW  ->  (-ty, tx)
        normals = np.column_stack([-tangents[:, 1], tangents[:, 0]])
    else:
        normals = np.asarray(left_normals, dtype=float)
        if normals.shape != centerline.shape:
            raise ValueError(
                "left_normals must have the same shape as centerline "
                f"({centerline.shape}), got {normals.shape}"
            )
        norm_len = np.linalg.norm(normals, axis=1, keepdims=True)
        normals = normals / np.maximum(norm_len, 1e-9)

    # Enforce normal sign continuity to avoid local +/- flips that create
    # side-swaps and sharp spikes in measured boundaries.
    for i in range(1, n):
        if np.dot(normals[i], normals[i - 1]) < 0.0:
            normals[i] *= -1.0
    # Keep wrap-around consistent with index 0 orientation.
    if np.dot(normals[-1], normals[0]) < 0.0:
        normals[-1] *= -1.0

    w_right = np.zeros(n)
    w_left = np.zeros(n)
    # Sub-pixel stepping: smaller step for better corner hit detection.
    step_size = max(resolution * 0.10, 0.0010)

    def _raycast_distance(center_pt, direction):
        for d in np.arange(step_size, max_dist, step_size):
            pt = center_pt + d * direction
            row, col = _world_to_grid_xy(pt)
            if not (0 <= row < h and 0 <= col < w) or not free_cast[row, col]:
                return d
        return max_dist

    def _contour_candidates(center_pt, direction, max_candidates=8):
        rel_seg = seg_a - center_pt
        denom = _cross2(direction, seg_vec)
        valid = np.abs(denom) > 1e-9
        t_seg = np.full(len(seg_a), np.inf)
        u_seg = np.full(len(seg_a), np.inf)
        t_seg[valid] = _cross2(rel_seg[valid], seg_vec[valid]) / denom[valid]
        u_seg[valid] = _cross2(rel_seg[valid], direction) / denom[valid]
        valid_hit = (
            valid
            & (t_seg > 0.0)
            & (t_seg <= max_dist)
            & (u_seg >= -1e-9)
            & (u_seg <= 1.0 + 1e-9)
        )

        if np.any(valid_hit):
            dists = t_seg[valid_hit]
            order = np.argsort(dists)[:max_candidates]
            dists = dists[order]
            points = center_pt + dists[:, None] * direction
            return dists.astype(float), points.astype(float), False

        dist = _raycast_distance(center_pt, direction)
        point = center_pt + dist * direction
        return np.array([dist], dtype=float), np.array([point], dtype=float), True

    def _select_continuous_widths(side_name, directions):
        cand_dists = []
        cand_points = []
        fallback_count = 0
        for i in range(n):
            dists_i, points_i, used_fallback = _contour_candidates(
                centerline[i],
                directions[i],
            )
            cand_dists.append(dists_i)
            cand_points.append(points_i)
            fallback_count += int(used_fallback)

        # Start DP at the most constrained normal to avoid an arbitrary seam
        # deciding which wall branch the sequence follows.
        seed = int(np.argmin([len(d) for d in cand_dists]))
        order = [(seed + k) % n for k in range(n)]
        d_rot = [cand_dists[i] for i in order]
        p_rot = [cand_points[i] for i in order]

        dist_weight = 0.05
        width_jump_weight = 1.5
        backward_weight = 6.0
        closure_weight = 1.0
        best_total = np.inf
        best_path = None
        seed_count = len(d_rot[0])

        for seed_choice in range(seed_count):
            dp_prev = np.full(seed_count, np.inf)
            dp_prev[seed_choice] = dist_weight * d_rot[0][seed_choice]
            parents = []

            for k in range(1, n):
                prev_pts = p_rot[k - 1]
                cur_pts = p_rot[k]
                move = cur_pts[None, :, :] - prev_pts[:, None, :]
                continuity = np.linalg.norm(move, axis=2)

                cl_step = centerline[order[k]] - centerline[order[k - 1]]
                cl_step_norm = float(np.linalg.norm(cl_step))
                if cl_step_norm > 1e-9:
                    cl_dir = cl_step / cl_step_norm
                    progress = np.einsum('ijk,k->ij', move, cl_dir)
                    backward_penalty = backward_weight * np.maximum(0.0, -progress)
                else:
                    backward_penalty = 0.0

                width_jump = np.abs(d_rot[k - 1][:, None] - d_rot[k][None, :])
                cost = (
                    dp_prev[:, None]
                    + continuity
                    + backward_penalty
                    + width_jump_weight * width_jump
                    + dist_weight * d_rot[k][None, :]
                )
                parent = np.argmin(cost, axis=0)
                dp_cur = cost[parent, np.arange(len(d_rot[k]))]
                parents.append(parent)
                dp_prev = dp_cur

            closure = np.linalg.norm(
                p_rot[-1] - p_rot[0][seed_choice],
                axis=1,
            )
            total_cost = dp_prev + closure_weight * closure
            end_choice = int(np.argmin(total_cost))
            total = float(total_cost[end_choice])
            if total < best_total:
                best_total = total
                path = [0] * n
                path[-1] = end_choice
                for k in range(n - 2, -1, -1):
                    path[k] = int(parents[k][path[k + 1]])
                path[0] = seed_choice
                best_path = path

        widths_rot = np.array([
            d_rot[k][best_path[k]]
            for k in range(n)
        ])
        widths = np.empty(n, dtype=float)
        for k, original_i in enumerate(order):
            widths[original_i] = widths_rot[k]

        # The first intersection along a normal is the physical wall of the
        # current driveable corridor.  The continuity tracker above can
        # occasionally lock onto a farther contour branch on dense tracks,
        # creating cross-track boundary spikes.  Keep continuity when it agrees
        # with the nearest hit, but reject skipped-nearest-wall solutions.
        nearest_widths = np.array([float(d[0]) for d in cand_dists])
        skipped_nearest = widths > np.maximum(
            nearest_widths + 0.20,
            nearest_widths * 1.25,
        )
        if np.any(skipped_nearest):
            widths[skipped_nearest] = nearest_widths[skipped_nearest]
            print(
                f"  Width nearest-wall guard ({side_name}): "
                f"{int(skipped_nearest.sum())} branch hops corrected"
            )

        raycast_widths = np.array([
            _raycast_distance(centerline[i], directions[i])
            for i in range(n)
        ])
        raycast_caps = widths > np.maximum(
            raycast_widths + 0.05,
            raycast_widths * 1.10,
        )
        if np.any(raycast_caps):
            widths[raycast_caps] = raycast_widths[raycast_caps]
            print(
                f"  Width ray-cast guard ({side_name}): "
                f"{int(raycast_caps.sum())} contour misses corrected"
            )

        if fallback_count:
            print(f"  Width fallback ray-casts ({side_name}): {fallback_count}")
        print(
            f"  Width continuity tracking ({side_name}): "
            f"seed={seed}, candidates={sum(len(d) for d in cand_dists)}"
        )
        return widths

    w_left = _select_continuous_widths('left', normals)
    w_right = _select_continuous_widths('right', -normals)

    def _clamp_long_outliers_to_local_median(widths, side_name, window=21, factor=1.5):
        out = np.asarray(widths, dtype=float).copy()
        half = window // 2
        n_clamped = 0
        for i in range(n):
            idx = [(i + k) % n for k in range(-half, half + 1) if k != 0]
            local_median = float(np.median(out[idx]))
            if local_median > 1e-6 and out[i] > factor * local_median:
                out[i] = local_median
                n_clamped += 1
        if n_clamped:
            print(
                f"  Width outlier clamp ({side_name}): "
                f"{n_clamped} points > {factor:.1f}x local median"
            )
        return out

    w_left = _clamp_long_outliers_to_local_median(w_left, 'left')
    w_right = _clamp_long_outliers_to_local_median(w_right, 'right')

    return w_right, w_left



def _world_to_pixel(points_xy, map_img, resolution, origin):
    """Convert Nx2 world coordinates to pixel coordinates for plotting."""
    px = (points_xy[:, 0] - origin[0]) / resolution
    py = map_img.shape[0] - (points_xy[:, 1] - origin[1]) / resolution
    return px, py


def plot_prepared_optimizer_track(map_img, resolution, origin,
                                  prepared_track, output_path):
    """Plot the exact reference spline and bounds sent to the optimizer."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    reftrack = prepared_track['reftrack_interp']
    normals = prepared_track['normvec_normalized_interp']
    coeffs_x = prepared_track['coeffs_x_interp']
    coeffs_y = prepared_track['coeffs_y_interp']

    samples_per_segment = 8
    t = np.linspace(0.0, 1.0, samples_per_segment, endpoint=False)
    spline_pts = []
    for cx, cy in zip(coeffs_x, coeffs_y):
        x = cx[0] + cx[1] * t + cx[2] * t**2 + cx[3] * t**3
        y = cy[0] + cy[1] * t + cy[2] * t**2 + cy[3] * t**3
        spline_pts.append(np.column_stack([x, y]))
    spline_pts = np.vstack(spline_pts)

    right_boundary = reftrack[:, :2] + normals * reftrack[:, 2:3]
    left_boundary = reftrack[:, :2] - normals * reftrack[:, 3:4]
    center_steps = np.linalg.norm(
        np.diff(np.vstack([reftrack[:, :2], reftrack[0, :2]]), axis=0),
        axis=1,
    )
    boundary_plot_jump = max(0.30, 3.0 * float(np.median(center_steps)))
    free_for_plot = map_img >= 240

    def _segment_crosses_free_space(p0, p1, samples=9):
        ts = np.linspace(0.0, 1.0, samples + 2)[1:-1]
        pts = p0[None, :] + ts[:, None] * (p1 - p0)[None, :]
        cols = np.rint((pts[:, 0] - origin[0]) / resolution).astype(int)
        rows = np.rint(map_img.shape[0] - (pts[:, 1] - origin[1]) / resolution).astype(int)
        in_bounds = (
            (rows >= 0) & (rows < map_img.shape[0])
            & (cols >= 0) & (cols < map_img.shape[1])
        )
        if not np.any(in_bounds):
            return True
        return np.mean(free_for_plot[rows[in_bounds], cols[in_bounds]]) > 0.35

    def _plot_boundary_with_gaps(boundary_world, color, label):
        boundary_closed = np.vstack([boundary_world, boundary_world[0]])
        jumps = np.linalg.norm(np.diff(boundary_closed, axis=0), axis=1)
        plot_pts = []
        for i, pt in enumerate(boundary_world):
            plot_pts.append(pt)
            next_pt = boundary_closed[i + 1]
            if (jumps[i] > boundary_plot_jump
                    or _segment_crosses_free_space(pt, next_pt)):
                plot_pts.append([np.nan, np.nan])
        plot_pts = np.asarray(plot_pts, dtype=float)
        ax.plot(
            *_world_to_pixel(plot_pts, map_img, resolution, origin),
            color=color,
            linewidth=1.0,
            label=label,
        )

    spline_px = _world_to_pixel(spline_pts, map_img, resolution, origin)
    center_px = _world_to_pixel(reftrack[:, :2], map_img, resolution, origin)

    fig, ax = plt.subplots(figsize=(12, 12))
    ax.imshow(map_img, cmap='gray')
    ax.plot(*spline_px, color='lime', linewidth=1.6, label='optimizer spline')
    ax.scatter(*center_px, s=8, color='blue', label='optimizer points', zorder=4)
    _plot_boundary_with_gaps(right_boundary, 'red', 'right boundary')
    _plot_boundary_with_gaps(left_boundary, 'deepskyblue', 'left boundary')

    # Draw the exact normal-width segments used as optimizer constraints.
    for c, r, l in zip(reftrack[:, :2], right_boundary, left_boundary):
        cr_px = _world_to_pixel(np.vstack([l, c, r]), map_img, resolution, origin)
        ax.plot(*cr_px, color='white', linewidth=0.25, alpha=0.35)

    ax.set_aspect('equal')
    ax.set_title('Prepared optimizer spline and bounds')
    ax.legend(loc='upper right')
    ax.axis('off')
    plt.tight_layout()
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    plt.savefig(output_path, dpi=160)
    plt.close(fig)
    print(f"  TEMP DEBUG: Saved prepared optimizer spline plot: {output_path}")


def _plot_width_boundaries(ax, centerline, w_right, w_left, color_r, color_l):
    """Plot left/right boundary curves from centerline plus side widths."""
    from scipy.ndimage import gaussian_filter1d

    # Match the normal construction used by width measurement to ensure
    # debug plots reflect actual measured geometry (no plotting-only hooks).
    cx = gaussian_filter1d(centerline[:, 0], sigma=1.2, mode='wrap')
    cy = gaussian_filter1d(centerline[:, 1], sigma=1.2, mode='wrap')
    cl_smooth = np.column_stack([cx, cy])
    tangent = np.roll(cl_smooth, -1, axis=0) - np.roll(cl_smooth, 1, axis=0)
    tnorm = np.linalg.norm(tangent, axis=1, keepdims=True)
    tangent = tangent / np.maximum(tnorm, 1e-9)
    normal = np.column_stack([-tangent[:, 1], tangent[:, 0]])

    # Enforce sign continuity around the loop.
    for i in range(1, len(normal)):
        if np.dot(normal[i], normal[i - 1]) < 0.0:
            normal[i] *= -1.0
    if np.dot(normal[-1], normal[0]) < 0.0:
        normal[-1] *= -1.0

    right_b = centerline - normal * np.asarray(w_right)[:, None]
    left_b = centerline + normal * np.asarray(w_left)[:, None]
    return right_b, left_b


def save_tum_track_csv(centerline, w_right, w_left, output_path):
    """
    Save centerline + track widths in the TUM global optimizer input format.

    Format: ``# x_m, y_m, w_tr_right_m, w_tr_left_m``
    """
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, 'w') as f:
        f.write('# x_m, y_m, w_tr_right_m, w_tr_left_m\n')
        for i in range(len(centerline)):
            f.write(
                f'{centerline[i, 0]:.6f}, {centerline[i, 1]:.6f}, '
                f'{w_right[i]:.4f}, {w_left[i]:.4f}\n'
            )
    print(f"  Saved TUM track CSV: {output_path}")
    print(f"  Points: {len(centerline)}, "
          f"avg width right: {w_right.mean():.3f} m, "
          f"avg width left: {w_left.mean():.3f} m")


def resample_closed_centerline(centerline, spacing):
    """
    Resample an ordered closed centerline by arc length.

    This is deliberately simpler than TUM's ``spline_approximation`` step:
    Step 0 already created a good centerline from the map, so here we only
    choose a stable point spacing for the nonlinear optimizer.
    """
    pts = np.asarray(centerline, dtype=float)
    if spacing <= 0.0:
        raise ValueError("spacing must be positive")
    if pts.ndim != 2 or pts.shape[1] != 2:
        raise ValueError("centerline must be an Nx2 array")
    if len(pts) < 4:
        raise ValueError("Need at least 4 centerline points for closed resampling")

    # Drop duplicated seam/near-duplicate samples before building the periodic
    # spline parameter.  CubicSpline requires strictly increasing arc length.
    min_dist = 1e-4
    cleaned = [pts[0]]
    for pt in pts[1:]:
        if np.linalg.norm(pt - cleaned[-1]) > min_dist:
            cleaned.append(pt)
    if len(cleaned) > 1 and np.linalg.norm(cleaned[-1] - cleaned[0]) < min_dist:
        cleaned.pop()
    pts = np.asarray(cleaned, dtype=float)
    if len(pts) < 4:
        raise ValueError("Too few unique centerline points after duplicate removal")

    closed = np.vstack([pts, pts[0]])
    seg_lengths = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    if np.any(seg_lengths <= 1e-6):
        raise ValueError("Centerline still contains zero-length segments")

    s = np.concatenate([[0.0], np.cumsum(seg_lengths)])
    total_length = float(s[-1])
    n_points = max(20, int(np.ceil(total_length / spacing)))
    s_new = np.linspace(0.0, total_length, n_points, endpoint=False)

    spline_x = CubicSpline(s, closed[:, 0], bc_type='periodic')
    spline_y = CubicSpline(s, closed[:, 1], bc_type='periodic')
    resampled = np.column_stack([spline_x(s_new), spline_y(s_new)])
    actual_spacing = total_length / n_points

    return resampled, total_length, actual_spacing


def interpolated_spline_max_abs_kappa(path_xy, spacing):
    """Return max abs analytical curvature after closed spline interpolation."""
    import trajectory_planning_helpers as tph

    path = np.asarray(path_xy, dtype=float)
    if path.ndim != 2 or path.shape[1] != 2 or len(path) < 4:
        return 0.0

    coeffs_x, coeffs_y, _, _ = tph.calc_splines.calc_splines(
        path=np.vstack([path, path[0]]),
    )
    spline_lengths = tph.calc_spline_lengths.calc_spline_lengths(
        coeffs_x=coeffs_x,
        coeffs_y=coeffs_y,
    )
    _, spline_inds, t_vals, _ = tph.interp_splines.interp_splines(
        spline_lengths=spline_lengths,
        coeffs_x=coeffs_x,
        coeffs_y=coeffs_y,
        incl_last_point=False,
        stepsize_approx=spacing,
    )
    _, kappa = tph.calc_head_curv_an.calc_head_curv_an(
        coeffs_x=coeffs_x,
        coeffs_y=coeffs_y,
        ind_spls=spline_inds,
        t_spls=t_vals,
    )
    return float(np.max(np.abs(kappa)))


def mincurv_condition_centerline(reftrack_interp, normvec_right, a_interp,
                                 curvlim, optimizer_width,
                                 optimizer_spacing, wall_clearance=0.0,
                                 kappa_spacing=0.02):
    """
    Project the prepared centerline through TUM's min-curvature QP and keep
    only solutions whose true interpolated spline curvature satisfies curvlim.
    """
    import trajectory_planning_helpers as tph

    if curvlim is None or curvlim <= 0.0:
        return None

    wall_clearance = max(0.0, float(wall_clearance or 0.0))
    clearance_reftrack = reftrack_interp.copy()
    clearance_reftrack[:, 2:4] -= wall_clearance

    min_side_width = float(np.min(clearance_reftrack[:, 2:4]))
    max_feasible_width = max(0.0, 2.0 * min_side_width - 0.005)
    width_candidates = []
    if optimizer_width > 0.0 and optimizer_width <= max_feasible_width + 1e-9:
        width_candidates = [round(float(optimizer_width), 4)]

    if not width_candidates:
        print(
            "  Mincurv pre-pass skipped: configured width_opt "
            f"{optimizer_width:.3f}m is infeasible for minimum side width "
            f"{min_side_width:.3f}m after wall_clearance="
            f"{wall_clearance:.3f}m"
        )
        return None

    print(
        f"  Mincurv pre-pass width_opt fixed at {width_candidates[0]:.3f}m, "
        f"effective_clearance={wall_clearance:.3f}m"
    )

    kappa_candidates = [
        curvlim * factor
        for factor in (1.0, 0.80, 0.65, 0.57, 0.50, 0.43)
    ]

    best_path = None
    best_kappa = np.inf
    best_meta = None
    print("  Running min-curvature pre-pass before mintime...")
    for kappa_bound in kappa_candidates:
        for width_opt in width_candidates:
            try:
                alpha, curv_error = tph.opt_min_curv.opt_min_curv(
                    reftrack=clearance_reftrack,
                    normvectors=normvec_right,
                    A=a_interp,
                    kappa_bound=kappa_bound,
                    w_veh=width_opt,
                    print_debug=False,
                    plot_debug=False,
                )
            except Exception as exc:
                print(
                    "  Mincurv pre-pass attempt failed: "
                    f"kappa_bound={kappa_bound:.3f}, "
                    f"width_opt={width_opt:.3f}: "
                    f"{type(exc).__name__}: {exc}"
                )
                continue

            projected = (
                reftrack_interp[:, :2]
                + np.expand_dims(alpha, 1) * normvec_right
            )
            projected, _, _ = resample_closed_centerline(
                projected,
                optimizer_spacing,
            )
            actual_kappa = interpolated_spline_max_abs_kappa(
                projected,
                kappa_spacing,
            )
            print(
                "  Mincurv pre-pass attempt: "
                f"kappa_bound={kappa_bound:.3f}, "
                f"width_opt={width_opt:.3f}, "
                f"actual_max_kappa={actual_kappa:.3f}, "
                f"lin_error={curv_error:.3f}"
            )

            if actual_kappa < best_kappa:
                best_path = projected
                best_kappa = actual_kappa
                best_meta = (kappa_bound, width_opt)

            if actual_kappa <= curvlim:
                print(
                    "  Mincurv pre-pass accepted: "
                    f"actual_max_kappa={actual_kappa:.3f} <= "
                    f"curvlim={curvlim:.3f}"
                )
                return projected

    if best_path is not None:
        print(
            "  WARNING: Mincurv pre-pass could not satisfy curvlim; "
            f"best actual_max_kappa={best_kappa:.3f} "
            f"(kappa_bound={best_meta[0]:.3f}, width_opt={best_meta[1]:.3f})"
        )
    return None


def build_prepared_optimizer_track(centerline, map_img, resolution, origin,
                                   wall_thresh, max_ray_distance,
                                   car_width, optimizer_spacing,
                                   optimizer_width=None,
                                   wall_clearance=0.0,
                                   optimizer_smoothing_s=0.0,
                                   optimizer_smoothing_k=2,
                                   optimizer_smoothing_prep_spacing=0.02,
                                   curvlim=None):
    """
    Build the exact reference-track arrays consumed by TUM's mintime optimizer.

    The important bit: we bypass TUM's full ``prep_track`` loader, but still
    use TUM's spline smoother and ``tph.calc_splines`` so the optimizer
    receives the coefficient matrices and normal-vector convention it expects.
    """
    import trajectory_planning_helpers as tph

    if optimizer_width is None:
        optimizer_width = car_width

    opt_centerline, track_length, actual_spacing = resample_closed_centerline(
        centerline,
        optimizer_spacing,
    )
    print(
        f"  Prepared optimizer centerline: {len(opt_centerline)} points, "
        f"{track_length:.1f} m, spacing={actual_spacing:.3f} m"
    )

    base_max_kappa = max_abs_analytical_curvature(
        opt_centerline,
        sample_spacing=0.01,
    )
    if curvlim is not None:
        print(
            f"  Base max curvature: {base_max_kappa:.3f} 1/m "
            f"(curvlim={curvlim:.3f} 1/m)"
        )

    if optimizer_smoothing_s and optimizer_smoothing_s > 0.0:
        print(
            "  Applying TUM spline smoother to optimizer centerline: "
            f"k={optimizer_smoothing_k}, s={optimizer_smoothing_s:.3f}"
        )
        def _smooth_centerline(s_reg):
            dummy_width = np.ones(len(opt_centerline), dtype=float)
            smooth_input = np.column_stack([opt_centerline, dummy_width, dummy_width])
            smooth_track = tph.spline_approximation.spline_approximation(
                track=smooth_input,
                k_reg=optimizer_smoothing_k,
                s_reg=s_reg,
                stepsize_prep=optimizer_smoothing_prep_spacing,
                stepsize_reg=optimizer_spacing,
                debug=True,
            )
            smoothed = smooth_track[:, :2]
            smooth_closed = np.vstack([smoothed, smoothed[0]])
            smooth_lengths = np.linalg.norm(np.diff(smooth_closed, axis=0), axis=1)
            smoothed_len = float(np.sum(smooth_lengths))
            smoothed_spacing = smoothed_len / len(smoothed)
            return smoothed, smoothed_len, smoothed_spacing

        if curvlim is None or curvlim <= 0.0:
            opt_centerline, track_length, actual_spacing = _smooth_centerline(
                optimizer_smoothing_s
            )
            print(
                f"  Smoothed optimizer centerline: {len(opt_centerline)} points, "
                f"{track_length:.1f} m, spacing={actual_spacing:.3f} m"
            )
        else:
            s_reg = float(optimizer_smoothing_s)
            max_attempts = 8
            accepted = False
            for attempt in range(max_attempts):
                smoothed, smoothed_len, smoothed_spacing = _smooth_centerline(s_reg)
                smoothed_max_kappa = max_abs_analytical_curvature(
                    smoothed,
                    sample_spacing=0.01,
                )
                print(
                    f"  Smoothing attempt {attempt + 1}/{max_attempts}: "
                    f"s={s_reg:.4f}, max_kappa={smoothed_max_kappa:.3f} 1/m"
                )
                if smoothed_max_kappa <= curvlim:
                    opt_centerline = smoothed
                    track_length = smoothed_len
                    actual_spacing = smoothed_spacing
                    accepted = True
                    break
                s_reg *= 1.5

            if accepted:
                print(
                    f"  Smoothed optimizer centerline: {len(opt_centerline)} points, "
                    f"{track_length:.1f} m, spacing={actual_spacing:.3f} m"
                )
            else:
                print(
                    "  WARNING: Smoothing exceeds curvlim; keeping unsmoothed "
                    f"centerline (max_kappa={base_max_kappa:.3f} 1/m)"
                )

    refpath_closed = np.vstack([opt_centerline, opt_centerline[0]])
    coeffs_x, coeffs_y, a_interp, normvec_right = tph.calc_splines.calc_splines(
        path=refpath_closed,
    )

    # TPH normals point to the right side of the track.  The width measurer
    # accepts left normals, so use the opposite direction here.  This keeps
    # width_right aligned with +normvec_right inside opt_mintime().
    print("  Measuring optimizer-track widths with final spline normals...")
    w_right, w_left = measure_track_widths(
        opt_centerline,
        map_img,
        resolution,
        origin,
        max_dist=max_ray_distance,
        wall_thresh=wall_thresh,
        left_normals=-normvec_right,
    )

    car_half = car_width / 2.0
    w_right = np.maximum(w_right, car_half)
    w_left = np.maximum(w_left, car_half)

    reftrack_interp = np.column_stack([opt_centerline, w_right, w_left])

    conditioned_centerline = mincurv_condition_centerline(
        reftrack_interp=reftrack_interp,
        normvec_right=normvec_right,
        a_interp=a_interp,
        curvlim=curvlim,
        optimizer_width=optimizer_width,
        optimizer_spacing=optimizer_spacing,
        wall_clearance=wall_clearance,
    )
    if conditioned_centerline is not None:
        opt_centerline = conditioned_centerline
        refpath_closed = np.vstack([opt_centerline, opt_centerline[0]])
        coeffs_x, coeffs_y, a_interp, normvec_right = (
            tph.calc_splines.calc_splines(path=refpath_closed)
        )
        true_kappa = interpolated_spline_max_abs_kappa(opt_centerline, 0.02)
        print(
            "  Prepared centerline after mincurv pre-pass: "
            f"{len(opt_centerline)} points, "
            f"true_max_kappa={true_kappa:.3f} 1/m"
        )
        print("  Re-measuring optimizer-track widths after mincurv pre-pass...")
        w_right, w_left = measure_track_widths(
            opt_centerline,
            map_img,
            resolution,
            origin,
            max_dist=max_ray_distance,
            wall_thresh=wall_thresh,
            left_normals=-normvec_right,
        )
        w_right = np.maximum(w_right, car_half)
        w_left = np.maximum(w_left, car_half)
        reftrack_interp = np.column_stack([opt_centerline, w_right, w_left])

    normals_crossing = tph.check_normals_crossing.check_normals_crossing(
        track=reftrack_interp,
        normvec_normalized=normvec_right,
        horizon=10,
    )
    if normals_crossing:
        print(
            "  WARNING: Prepared optimizer track has normal crossings. "
            "Continuing without local width caps to avoid artificial spikes."
        )

    total_width = w_right + w_left
    print(
        f"  Prepared optimizer widths: "
        f"right=[{np.min(w_right):.3f}, {np.max(w_right):.3f}] m, "
        f"left=[{np.min(w_left):.3f}, {np.max(w_left):.3f}] m, "
        f"total_min={np.min(total_width):.3f} m"
    )

    return {
        'reftrack_interp': reftrack_interp,
        'normvec_normalized_interp': normvec_right,
        'a_interp': a_interp,
        'coeffs_x_interp': coeffs_x,
        'coeffs_y_interp': coeffs_y,
    }


def save_prepared_optimizer_track(prepared_track, output_path):
    """Save prepared optimizer arrays for loading inside main_globaltraj.py."""
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    np.savez(
        output_path,
        reftrack_interp=prepared_track['reftrack_interp'],
        normvec_normalized_interp=prepared_track['normvec_normalized_interp'],
        a_interp=prepared_track['a_interp'],
        coeffs_x_interp=prepared_track['coeffs_x_interp'],
        coeffs_y_interp=prepared_track['coeffs_y_interp'],
    )
    print(f"  Saved prepared optimizer track: {output_path}")


# =============================================================================
#  Step 1 -- TUM optimizer helpers (kept from original)
# =============================================================================

def _is_track_name_assignment(line):
    """Return True only for lines that ASSIGN file_paths["track_name"]."""
    # Strip comments for the test
    code = line.split('#')[0]
    # Must be an assignment: file_paths["track_name"] = "..."
    return 'file_paths["track_name"]' in code and '=' in code and '+' not in code


def set_track_in_main(main_py_path, track_name):
    """Set the track name in main_globaltraj.py."""
    with open(main_py_path, 'r') as f:
        content = f.read()

    lines = content.split('\n')
    new_lines = []
    track_set = False
    for line in lines:
        if _is_track_name_assignment(line):
            stripped = line.lstrip()
            if stripped.startswith('#'):
                # Commented line -- uncomment if it matches our track
                if f'"{track_name}"' in line:
                    idx = line.index('#')
                    new_lines.append(line[:idx] + stripped[2:])
                    track_set = True
                else:
                    new_lines.append(line)
            else:
                # Active line -- keep if it matches, else comment out
                if f'"{track_name}"' in line:
                    new_lines.append(line)
                    track_set = True
                else:
                    new_lines.append('# ' + line)
        else:
            new_lines.append(line)

    if not track_set:
        # Add track line after last track_name assignment (including commented)
        # Only match lines that look like assignments (not usage like + ".csv")
        for i in range(len(new_lines) - 1, -1, -1):
            stripped = new_lines[i].lstrip().lstrip('#').lstrip()
            if (stripped.startswith('file_paths["track_name"]')
                    and '=' in stripped
                    and '+' not in stripped):
                new_lines.insert(
                    i + 1,
                    f'file_paths["track_name"] = "{track_name}"'
                    f'                                    # set by optimize_trajectory.py',
                )
                break

    with open(main_py_path, 'w') as f:
        f.write('\n'.join(new_lines))


def set_opt_type_in_main(main_py_path, opt_type):
    """Set the optimization type in main_globaltraj.py."""
    with open(main_py_path, 'r') as f:
        content = f.read()

    content = re.sub(
        r"^(opt_type\s*=\s*).*$",
        f"opt_type = '{opt_type}'",
        content,
        flags=re.MULTILINE,
    )

    with open(main_py_path, 'w') as f:
        f.write(content)


def set_mintime_bool_option_in_main(main_py_path, option_name, enabled):
    """Toggle a boolean option in TUM's ``mintime_opts`` dict."""
    with open(main_py_path, 'r') as f:
        content = f.read()

    replacement = f'"{option_name}": {str(enabled)}'
    content, count = re.subn(
        rf'"{re.escape(option_name)}":\s*(True|False)',
        replacement,
        content,
        count=1,
    )
    if count != 1:
        raise RuntimeError(f"Could not find {option_name} in main_globaltraj.py")

    with open(main_py_path, 'w') as f:
        f.write(content)


def set_strict_curvlim_kappa_candidates(main_py_path, enabled,
                                         reopt_kappa_factor=1.0):
    """Force mintime reoptimization to use only curvlim as kappa bound."""
    if not enabled:
        return
    with open(main_py_path, 'r') as f:
        content = f.read()

    replacement = (
        'kappa_candidates = [pars["veh_params"]["curvlim"] '
        f'* {float(reopt_kappa_factor):.6g}] '
        '# set by optimize_trajectory_mintime.py'
    )
    content, count = re.subn(
        r'kappa_candidates\s*=\s*\[\s*'
        r'pars\["veh_params"\]\["curvlim"\]\s*\*\s*factor\s*'
        r'for\s+factor\s+in\s*\([^)]*\)\s*'
        r'\]',
        replacement,
        content,
        count=1,
        flags=re.DOTALL,
    )
    if count != 1:
        raise RuntimeError("Could not find kappa_candidates in main_globaltraj.py")

    with open(main_py_path, 'w') as f:
        f.write(content)


def patch_mintime_optim_numeric_option(ini_content, option_name, value,
                                       insert_after='penalty_F'):
    """Patch or insert a numeric option in optim_opts_mintime."""
    number = r'[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?'
    value_text = f"{float(value):.6g}"

    key_pattern = rf'("{re.escape(option_name)}"\s*:\s*){number}'
    patched, count = re.subn(
        key_pattern,
        rf'\g<1>{value_text}',
        ini_content,
        count=1,
    )
    if count:
        return patched

    insert_pattern = rf'("{re.escape(insert_after)}"\s*:\s*{number},)'
    patched, count = re.subn(
        insert_pattern,
        rf'\1\n                    "{option_name}": {value_text},',
        ini_content,
        count=1,
    )
    if not count:
        raise RuntimeError(
            f"Could not patch optim_opts_mintime option '{option_name}'"
        )
    return patched


def patch_mintime_optim_bool_option(ini_content, option_name, value,
                                    insert_after='penalty_F'):
    """Patch or insert a boolean option in optim_opts_mintime."""
    value_text = "true" if value else "false"

    key_pattern = rf'("{re.escape(option_name)}"\s*:\s*)(true|false)'
    patched, count = re.subn(
        key_pattern,
        rf'\g<1>{value_text}',
        ini_content,
        count=1,
        flags=re.IGNORECASE,
    )
    if count:
        return patched

    number = r'[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?'
    insert_pattern = rf'("{re.escape(insert_after)}"\s*:\s*{number},)'
    patched, count = re.subn(
        insert_pattern,
        rf'\1\n                    "{option_name}": {value_text},',
        ini_content,
        count=1,
    )
    if not count:
        raise RuntimeError(
            f"Could not patch optim_opts_mintime option '{option_name}'"
        )
    return patched


def _original_prep_track_block():
    """Return TUM's original reference-track preparation block."""
    return [
        'reftrack_interp, normvec_normalized_interp, a_interp, coeffs_x_interp, coeffs_y_interp = \\',
        '    helper_funcs_glob.src.prep_track.prep_track(reftrack_imp=reftrack_imp,',
        '                                                reg_smooth_opts=pars["reg_smooth_opts"],',
        '                                                stepsize_opts=pars["stepsize_opts"],',
        '                                                debug=debug,',
        '                                                min_width=imp_opts["min_track_width"])',
        '',
    ]


def restore_main_prep_track_if_needed(main_py_path):
    """
    Restore ``main_globaltraj.py`` if a previous interrupted run left our
    prepared-track loader patch in place.
    """
    with open(main_py_path, 'r') as f:
        lines = f.read().splitlines()

    start = None
    for i, line in enumerate(lines):
        if line.startswith('prepared_track_file = r"'):
            start = i
            break
    if start is None:
        return False

    end = None
    for i in range(start + 1, len(lines)):
        if lines[i].startswith('# ----------------------------------------------------------------------------------------------------------------------'):
            end = i
            break
    if end is None:
        raise RuntimeError("Could not find end of prepared-track patch in main_globaltraj.py")

    lines[start:end] = _original_prep_track_block()
    with open(main_py_path, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    return True


def patch_main_to_load_prepared_track(main_py_path, prepared_track_path):
    """
    Patch TUM's main script so it loads our prepared arrays instead of calling
    helper_funcs_glob.src.prep_track.prep_track().

    The wrapper restores the original file in a ``finally`` block after the
    subprocess exits.
    """
    with open(main_py_path, 'r') as f:
        lines = f.read().splitlines()

    start = None
    for i, line in enumerate(lines):
        if line.startswith('reftrack_interp, normvec_normalized_interp, a_interp, coeffs_x_interp, coeffs_y_interp ='):
            start = i
            break
    if start is None:
        if restore_main_prep_track_if_needed(main_py_path):
            with open(main_py_path, 'r') as f:
                lines = f.read().splitlines()
            for i, line in enumerate(lines):
                if line.startswith('reftrack_interp, normvec_normalized_interp, a_interp, coeffs_x_interp, coeffs_y_interp ='):
                    start = i
                    break
        if start is None:
            raise RuntimeError("Could not find prep_track assignment in main_globaltraj.py")

    end = None
    for i in range(start, len(lines)):
        if 'min_width=imp_opts["min_track_width"])' in lines[i]:
            end = i + 1
            break
    if end is None:
        raise RuntimeError("Could not find end of prep_track call in main_globaltraj.py")

    replacement = [
        f'prepared_track_file = r"{prepared_track_path}"',
        'if not os.path.exists(prepared_track_file):',
        '    raise FileNotFoundError(f"Prepared track file not found: {prepared_track_file}")',
        'prepared_track = np.load(prepared_track_file)',
        'reftrack_interp = prepared_track["reftrack_interp"]',
        'normvec_normalized_interp = prepared_track["normvec_normalized_interp"]',
        'a_interp = prepared_track["a_interp"]',
        'coeffs_x_interp = prepared_track["coeffs_x_interp"]',
        'coeffs_y_interp = prepared_track["coeffs_y_interp"]',
        'if debug:',
        '    print(f"INFO: Loaded prepared reference track: {prepared_track_file}")',
        '    print(f"INFO: Prepared reference points: {reftrack_interp.shape[0]}")',
    ]

    lines[start:end] = replacement
    with open(main_py_path, 'w') as f:
        f.write('\n'.join(lines) + '\n')


# =============================================================================
#  Step 2 -- TUM -> MPC format conversion (kept from original)
# =============================================================================

def convert_tum_to_mpc(input_csv, output_csv, max_speed=None, min_speed=None):
    """
    Convert TUM global optimizer output to MPC-compatible CSV.

    Changes:
      - Delimiter: semicolon -> comma
      - Heading: psi += pi/2, wrapped to [-pi, pi]
      - Header: standardised to ``# s_m,x_m,y_m,...``
      - Optional: clamp velocity to [min_speed, max_speed]
    """
    rows = []
    with open(input_csv, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = [p.strip() for p in line.split(';')]
            if len(parts) < 7:
                continue
            rows.append(parts)

    clamped_hi = 0
    clamped_lo = 0
    with open(output_csv, 'w') as f:
        f.write('# s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2\n')
        for row in rows:
            # Apply psi + pi/2 heading correction
            psi = float(row[3])
            psi_corrected = psi + math.pi / 2.0
            # Wrap to [-pi, pi]
            psi_corrected = (psi_corrected + math.pi) % (2 * math.pi) - math.pi
            row[3] = f"{psi_corrected:.7f}"

            # Clamp velocity to [min_speed, max_speed]
            vx = float(row[5])
            if max_speed is not None and vx > max_speed:
                row[5] = f"{max_speed:.7f}"
                clamped_hi += 1
            elif min_speed is not None and vx < min_speed:
                row[5] = f"{min_speed:.7f}"
                clamped_lo += 1

            f.write(','.join(row[:7]) + '\n')

    print(f"  Converted {len(rows)} waypoints (psi += pi/2)")
    if clamped_hi > 0:
        print(f"  Clamped {clamped_hi}/{len(rows)} velocities to max {max_speed:.1f} m/s")
    if clamped_lo > 0:
        print(f"  Clamped {clamped_lo}/{len(rows)} velocities to min {min_speed:.1f} m/s")
    return len(rows)


def write_mpc_trajectory_from_path(path_xy, output_csv, waypoint_spacing,
                                   max_speed=None, min_speed=None,
                                   lateral_acc_limit=3.8,
                                   accel_limit=3.8,
                                   decel_limit=4.2,
                                   curvature_window_m=1.0,
                                   curvature_smooth_sigma_m=0.25,
                                   curvlim=None):
    """Write a closed path directly to the MPC CSV format."""
    import trajectory_planning_helpers as tph

    path = np.asarray(path_xy, dtype=float)
    coeffs_x, coeffs_y, _, _ = tph.calc_splines.calc_splines(
        path=np.vstack([path, path[0]]),
    )
    spline_lengths = tph.calc_spline_lengths.calc_spline_lengths(
        coeffs_x=coeffs_x,
        coeffs_y=coeffs_y,
    )
    raceline, spline_inds, t_vals, s_points = tph.interp_splines.interp_splines(
        spline_lengths=spline_lengths,
        coeffs_x=coeffs_x,
        coeffs_y=coeffs_y,
        incl_last_point=False,
        stepsize_approx=waypoint_spacing,
    )
    psi_tum, kappa = tph.calc_head_curv_an.calc_head_curv_an(
        coeffs_x=coeffs_x,
        coeffs_y=coeffs_y,
        ind_spls=spline_inds,
        t_spls=t_vals,
    )
    psi_mpc = (psi_tum + math.pi / 2.0 + math.pi) % (2 * math.pi) - math.pi

    max_abs_kappa = float(np.max(np.abs(kappa)))
    if curvlim is not None and max_abs_kappa > curvlim:
        raise RuntimeError(
            "Strict fallback path exceeds curvlim "
            f"({max_abs_kappa:.3f} > {curvlim:.3f})"
        )

    total_length = float(np.sum(spline_lengths))
    el_lengths = np.diff(s_points)
    el_lengths = np.append(el_lengths, total_length - s_points[-1])

    abs_kappa = np.abs(kappa)
    mean_step = max(total_length / max(len(abs_kappa), 1), 1e-6)
    window_pts = max(3, int(round(curvature_window_m / mean_step)))
    if window_pts % 2 == 0:
        window_pts += 1
    sigma_pts = max(1.0, curvature_smooth_sigma_m / mean_step)

    from scipy.ndimage import gaussian_filter1d, maximum_filter1d
    kappa_envelope = maximum_filter1d(
        abs_kappa,
        size=window_pts,
        mode='wrap',
    )
    kappa_envelope = np.maximum(
        abs_kappa,
        gaussian_filter1d(kappa_envelope, sigma=sigma_pts, mode='wrap'),
    )
    vx = np.sqrt(lateral_acc_limit / np.maximum(kappa_envelope, 1e-6))
    if max_speed is not None:
        vx = np.minimum(vx, max_speed)
    if min_speed is not None:
        vx = np.maximum(vx, min_speed)

    for _ in range(200):
        prev_vx = vx.copy()
        for i in range(len(vx)):
            j = (i + 1) % len(vx)
            v_next_max = math.sqrt(max(vx[i] ** 2
                                       + 2.0 * accel_limit * el_lengths[i],
                                       0.0))
            if vx[j] > v_next_max:
                vx[j] = v_next_max
        for j in range(len(vx) - 1, -1, -1):
            i = (j - 1) % len(vx)
            v_prev_max = math.sqrt(max(vx[j] ** 2
                                       + 2.0 * decel_limit * el_lengths[i],
                                       0.0))
            if vx[i] > v_prev_max:
                vx[i] = v_prev_max
        if float(np.max(np.abs(vx - prev_vx))) < 1e-5:
            break

    vx_next = np.roll(vx, -1)
    ax = (vx_next**2 - vx**2) / np.maximum(2.0 * el_lengths, 1e-6)

    with open(output_csv, 'w') as f:
        f.write('# s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2\n')
        for s, xy, psi, kap, vel, acc in zip(
                s_points, raceline, psi_mpc, kappa, vx, ax):
            f.write(
                f"{s:.6f},{xy[0]:.6f},{xy[1]:.6f},"
                f"{psi:.7f},{kap:.7f},{vel:.7f},{acc:.7f}\n"
            )

    print(
        f"  Wrote strict curvature-limited fallback: {len(raceline)} waypoints, "
        f"max_abs_kappa={max_abs_kappa:.3f} 1/m, "
        f"speed_window={curvature_window_m:.1f}m"
    )
    return len(raceline)


# =============================================================================
#  Step 4 -- Verification (kept from original)
# =============================================================================

def verify_output(csv_path, curvlim=None, car_width=None, wall_clearance=0.0):
    """Sanity-check the final trajectory CSV."""
    waypoints = []
    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].startswith('#'):
                continue
            if len(row) < 7:
                continue
            waypoints.append([float(v) for v in row])

    if not waypoints:
        print("  ERROR: No waypoints found!")
        return False

    w = np.array(waypoints)
    n = len(w)
    ncols = w.shape[1]

    # Check heading vs atan2(dy, dx) using periodic central differences
    # (np.gradient with default mode gives wrong answer at endpoints for a
    # closed track; use explicit wrap-around central differences instead).
    n_pts = len(w)
    dx = np.zeros(n_pts)
    dy = np.zeros(n_pts)
    for i in range(n_pts):
        i_prev = (i - 1) % n_pts
        i_next = (i + 1) % n_pts
        dx[i] = w[i_next, 1] - w[i_prev, 1]
        dy[i] = w[i_next, 2] - w[i_prev, 2]
    recomputed_psi = np.arctan2(dy, dx)
    psi_diff = (w[:, 3] - recomputed_psi + np.pi) % (2 * np.pi) - np.pi
    max_psi_err = np.abs(psi_diff).max()

    print(f"\n  --- Trajectory Summary ---")
    print(f"  Waypoints:      {n}")
    print(f"  Columns:        {ncols}")
    print(f"  Track length:   {w[-1, 0]:.1f} m")
    print(f"  X range:        [{w[:, 1].min():.2f}, {w[:, 1].max():.2f}] m")
    print(f"  Y range:        [{w[:, 2].min():.2f}, {w[:, 2].max():.2f}] m")
    print(f"  Psi range:      [{w[:, 3].min():.4f}, {w[:, 3].max():.4f}] rad")
    print(f"  Psi vs atan2:   max err = {math.degrees(max_psi_err):.2f} deg")
    print(f"  Kappa range:    [{w[:, 4].min():.4f}, {w[:, 4].max():.4f}] 1/m")
    print(f"  Velocity range: [{w[:, 5].min():.2f}, {w[:, 5].max():.2f}] m/s")

    if ncols >= 9:
        print(f"  Left wall:      [{w[:, 7].min():.3f}, {w[:, 7].max():.3f}] m")
        print(f"  Right wall:     [{w[:, 8].min():.3f}, {w[:, 8].max():.3f}] m")
        if car_width is not None:
            car_half = float(car_width) / 2.0
            min_left_clearance = float(w[:, 7].min()) - car_half
            min_right_clearance = float(w[:, 8].min()) - car_half
            print(
                f"  Side clearance: "
                f"left_min={min_left_clearance:.3f} m, "
                f"right_min={min_right_clearance:.3f} m"
            )

    ok = True
    if max_psi_err > math.radians(5):
        print(
            f"  WARNING: Heading deviates from atan2(dy,dx) by up to "
            f"{math.degrees(max_psi_err):.1f} deg"
        )
    if ncols >= 9 and car_width is not None:
        min_center_distance = float(car_width) / 2.0 + float(wall_clearance or 0.0)
        if w[:, 7].min() < min_center_distance or w[:, 8].min() < min_center_distance:
            print(
                "  ERROR: Some center-to-wall distances are below "
                f"half car width + clearance ({min_center_distance:.3f} m)"
            )
            ok = False
    if n < 100:
        print(f"  WARNING: Very few waypoints ({n})")
        ok = False
    if curvlim is not None:
        max_abs_kappa = float(np.max(np.abs(w[:, 4])))
        if max_abs_kappa > curvlim:
            print(
                "  ERROR: Curvature limit violated: "
                f"{max_abs_kappa:.4f} > {curvlim:.4f} 1/m"
            )
            ok = False
    return ok


# =============================================================================
#  Visualization
# =============================================================================

def visualize_raceline(map_yaml_path, csv_path, output_path):
    """Create visualization of racing line on track map, colored by velocity."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    with open(map_yaml_path, 'r') as f:
        config = yaml.safe_load(f)

    img_path = config['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(map_yaml_path), img_path)

    img = cv2.imread(img_path)
    resolution = config['resolution']
    origin = np.array(config['origin'])
    img_height = img.shape[0]

    # Load trajectory
    data = np.loadtxt(csv_path, delimiter=',', comments='#')
    xy = data[:, 1:3]
    velocities = data[:, 5]

    # Convert world coords to pixel coords
    px = ((xy[:, 0] - origin[0]) / resolution).astype(int)
    py = (img_height - (xy[:, 1] - origin[1]) / resolution).astype(int)

    fig, ax = plt.subplots(figsize=(16, 16))
    ax.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))

    scatter = ax.scatter(px, py, c=velocities, cmap='RdYlGn', s=20,
                         vmin=velocities.min(), vmax=velocities.max())
    cbar = plt.colorbar(scatter, ax=ax, shrink=0.6)
    cbar.set_label('Velocity (m/s)', fontsize=12)

    ax.scatter(px[0], py[0], c='blue', s=300, marker='*', label='Start', zorder=10)

    # Direction arrows
    for i in range(0, len(px), max(1, len(px) // 8)):
        if i + 1 < len(px):
            dx = px[i + 1] - px[i]
            dy = py[i + 1] - py[i]
            length = np.sqrt(dx**2 + dy**2)
            if length > 0:
                dx, dy = dx / length * 15, dy / length * 15
                ax.annotate('', xy=(px[i] + dx, py[i] + dy),
                            xytext=(px[i], py[i]),
                            arrowprops=dict(arrowstyle='->', color='white', lw=2))

    ax.legend(loc='upper right', fontsize=12)
    ax.set_title('Optimized Racing Line', fontsize=14)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"  Saved visualization: {output_path}")


# =============================================================================
#  Main pipeline
# =============================================================================

def main():
    workspace = find_workspace_root()
    if not workspace:
        print(
            "ERROR: Could not find workspace root "
            "(no f1tenth_planning/ found)"
        )
        sys.exit(1)

    scripts_dir = os.path.dirname(os.path.abspath(__file__))
    global_opt_dir = os.path.join(workspace, 'f1tenth_planning', 'global_racetrajectory_optimization')
    default_output = os.path.join(workspace, 'f1tenth_planning', 'trajectories')
    racecar_ini = os.path.join(global_opt_dir, 'params', 'racecar.ini')

    # ---- User settings -------------------------------------------------------
    # This mintime copy is configured directly here instead of through command
    # line arguments.  Change these values before running the script.
    if len(sys.argv) > 1:
        print("ERROR: This script uses in-code settings instead of launch arguments.")
        print("  Edit the User settings block near the top of main().")
        sys.exit(2)

    args = argparse.Namespace(
        # Map and output paths
        map=os.path.join(workspace, 'f1tenth_planning', 'maps', 'my_track_map.yaml'),
        track_name='my_track',
        output=default_output,

        # Minimum-time optimizer settings
        opt_type=MIN_TIME_OPT_TYPE,
        max_speed=12.0,         # m/s (set to None for no clamping)
        min_speed=1.5,          # m/s (set to None for no clamping)
        waypoint_spacing=0.02,  
        reopt_mintime_solution=True,        # Try false false or true true
        recalc_vel_profile_by_tph=True,    # If true follow the files ax max and ggv

        # Centerline extraction settings
        centerline_spacing=0.05,     # target spacing for centerline points (in metres)
        optimizer_spacing=0.08,      # prepared reference-track spacing for TUM mintime
        optimizer_smoothing_s=2.0,  # TUM spline smoothing factor before width measurement
        optimizer_smoothing_k=2,    # TUM spline order (mirrors racecar.ini)
        optimizer_smoothing_prep_spacing=0.02,
        direction='cw',             # 'auto', 'cw', or 'ccw'

        # Vehicle width and separate center-to-wall clearance constraint.
        # width_opt remains car_width; wall_clearance reduces the allowed
        # centerline corridor by this extra margin on each side.
        car_width=0.30,
        wall_clearance=0.15,
        wall_clearance_guard=0.1,
        reopt_free_dev=0.005,
        reopt_kappa_factor=0.95,
        max_ray_distance=8.0,

        # Enforce curvature limit at all times
        strict_curvlim=True,
        curvature_penalty_weight=10000.0,
        curvature_penalty_margin=0.85,
    )

    # Verify map path
    if not os.path.exists(args.map):
        print(f"ERROR: Map file not found: {args.map}")
        sys.exit(1)

    curvlim = read_curvlim_from_racecar_ini(racecar_ini)
    if args.strict_curvlim and (curvlim is None or curvlim <= 0.0):
        print("ERROR: strict_curvlim enabled but curvlim is missing or invalid in racecar.ini")
        sys.exit(1)
    if curvlim is None:
        print("  WARNING: curvlim not found in racecar.ini; smoothing guard disabled")

    if args.strict_curvlim and not args.reopt_mintime_solution:
        print("  INFO: strict_curvlim enabled; forcing reopt_mintime_solution=True")
        args.reopt_mintime_solution = True


    track_name = args.track_name
    track_csv = os.path.join(global_opt_dir, 'inputs', 'tracks', f'{track_name}.csv')
    prepared_track_npz = os.path.join(
        global_opt_dir, 'inputs', 'tracks', f'{track_name}_prepared.npz'
    )
    output_csv = os.path.join(args.output, f'{track_name}_raceline.csv')
    gvd_debug_path = output_csv.replace('.csv', '_gvd.png')
    prepared_spline_debug_path = output_csv.replace(
        '.csv',
        '_prepared_splines_debug.png',
    )


    # ---- Banner --------------------------------------------------------------
    print("=" * 64)
    print("  F1Tenth Minimum-Time Trajectory Optimization Pipeline")
    print("=" * 64)
    print(f"  Map:              {args.map}")
    print(f"  Track name:       {track_name}")
    print(f"  Opt type:         {args.opt_type}")
    print(f"  Reopt mintime:    {args.reopt_mintime_solution}")
    print(f"  Strict curvlim:   {args.strict_curvlim}")
    recalc_label = str(args.recalc_vel_profile_by_tph)
    print(f"  Recalc velocity:  {recalc_label}")
    print(f"  Max speed:        {args.max_speed} m/s")
    print(f"  Centerline spacing: {args.centerline_spacing} m")
    print(f"  Optimizer spacing:  {args.optimizer_spacing} m")
    print(f"  Optimizer smoothing: s={args.optimizer_smoothing_s}, "
          f"k={args.optimizer_smoothing_k}")
    print(f"  Direction:        {args.direction}")
    print(f"  Car width:        {args.car_width} m")
    print(f"  Wall clearance:   {args.wall_clearance} m")
    print(f"  Clearance guard:  {args.wall_clearance_guard} m")
    print(f"  Reopt free dev:   {args.reopt_free_dev} m")
    print(f"  Reopt kappa fac:  {args.reopt_kappa_factor}")
    print(f"  Optimizer width:  {args.car_width:.3f} m")
    print(f"  Curv penalty:     weight={args.curvature_penalty_weight:g}, "
          f"margin={args.curvature_penalty_margin:.2f}")
    print(f"  Wall distances:   yes")
    print(f"  Output:           {output_csv}")

    print(f"\n{'=' * 64}")
    print(f"  Step 0: Extract centerline from map")
    print(f"{'=' * 64}")


    # -------- Step 0: Extract centerline and measure widths --------------------------------
    map_img, resolution, origin, _occupied_thresh = load_map(args.map)

    # Known map format: white is driveable, black walls and grey unknown
    # outside the racetrack are not driveable.
    wall_thresh = 240
    print(f"  Wall threshold: {wall_thresh} (only white pixels are free)")

    print(f"  Image size: {map_img.shape[1]}x{map_img.shape[0]} px, "
          f"resolution: {resolution} m/px")

    outer_world = extract_outer_boundary(map_img, resolution, origin, wall_thresh=wall_thresh)

    # Choose the number of centerline points from the estimated track length.
    outer_closed = np.vstack([outer_world, outer_world[0]])
    perimeter = np.sum(np.linalg.norm(np.diff(outer_closed, axis=0), axis=1))
    num_pts = max(100, int(np.ceil(perimeter / args.centerline_spacing)))
    print(
        f"  Estimated perimeter: {perimeter:.1f} m "
        f"→ using {num_pts} centerline points "
        f"(~{args.centerline_spacing:.3f} m spacing)"
    )

    # GVD centerline: label wall obstacles, find equidistant boundary,
    # thin it, walk the ring, B-spline smooth.
    centerline, _ = compute_centerline_thinned_loop(
        map_img, resolution, origin,
        wall_thresh=wall_thresh,
        num_points=num_pts,
        gvd_debug_path=gvd_debug_path,
    )

    centerline = condition_centerline_for_curvature_and_clearance(
        centerline=centerline,
        map_img=map_img,
        resolution=resolution,
        origin=origin,
        wall_thresh=wall_thresh,
        curvlim=curvlim if args.strict_curvlim else None,
        target_spacing=args.centerline_spacing,
        max_ray_distance=args.max_ray_distance,
        car_width=args.car_width,
        wall_clearance=args.wall_clearance + args.wall_clearance_guard,
        kappa_sample_spacing=0.01,
    )

    # Determine direction
    winding = detect_winding_direction(centerline)
    print(f"  Detected winding: {winding}")
    if args.direction == 'auto':
        direction = winding
    else:
        direction = args.direction

    # Reverse centerline if user wants opposite direction
    if direction != winding:
        print(f"  Reversing centerline to match requested direction: {direction}")
        centerline = centerline[::-1].copy()


    prepared_track = build_prepared_optimizer_track(
        centerline=centerline,
        map_img=map_img,
        resolution=resolution,
        origin=origin,
        wall_thresh=wall_thresh,
        max_ray_distance=args.max_ray_distance,
        car_width=args.car_width,
        optimizer_spacing=args.optimizer_spacing,
        optimizer_width=args.car_width,
        wall_clearance=args.wall_clearance + args.wall_clearance_guard,
        optimizer_smoothing_s=args.optimizer_smoothing_s,
        optimizer_smoothing_k=args.optimizer_smoothing_k,
        optimizer_smoothing_prep_spacing=args.optimizer_smoothing_prep_spacing,
        curvlim=curvlim,
    )
    reftrack_interp = prepared_track['reftrack_interp']
    plot_prepared_optimizer_track(
        map_img,
        resolution,
        origin,
        prepared_track,
        prepared_spline_debug_path,
    )

    # Keep a CSV copy for inspection/fallback, but the optimizer itself loads
    # the prepared .npz and skips TUM's prep_track spline approximation.
    save_tum_track_csv(
        reftrack_interp[:, :2],
        reftrack_interp[:, 2],
        reftrack_interp[:, 3],
        track_csv,
    )
    save_prepared_optimizer_track(prepared_track, prepared_track_npz)

    # ---- Step 1: Run TUM global_racetrajectory_optimization -----------------
    tum_output = os.path.join(global_opt_dir, 'outputs', 'traj_race_cl.csv')

    main_py = os.path.join(global_opt_dir, 'main_globaltraj.py')

    if restore_main_prep_track_if_needed(main_py):
        print("  Restored stale prepared-track patch in main_globaltraj.py")

    # Save original content for restoration
    with open(main_py, 'r') as f:
        original_content = f.read()
    with open(racecar_ini, 'r') as f:
        original_ini_content = f.read()

    try:
        set_track_in_main(main_py, track_name)
        set_opt_type_in_main(main_py, args.opt_type)
        recalc_vel_profile = (
            False
            if args.recalc_vel_profile_by_tph is None
            else args.recalc_vel_profile_by_tph
        )
        set_mintime_bool_option_in_main(
            main_py,
            "reopt_mintime_solution",
            args.reopt_mintime_solution,
        )
        set_mintime_bool_option_in_main(
            main_py,
            "recalc_vel_profile_by_tph",
            recalc_vel_profile,
        )
        set_strict_curvlim_kappa_candidates(
            main_py,
            args.strict_curvlim,
            args.reopt_kappa_factor,
        )
        patch_main_to_load_prepared_track(main_py, prepared_track_npz)
        print("  Patched main_globaltraj.py: using prepared Step 0 splines")
        print(
            "  Patched main_globaltraj.py: "
            f"reopt_mintime_solution -> {args.reopt_mintime_solution}"
        )
        print(
            "  Patched main_globaltraj.py: "
            f"recalc_vel_profile_by_tph -> {recalc_vel_profile}"
        )
        if args.strict_curvlim:
            print(
                "  Patched main_globaltraj.py: strict curvlim enabled "
                f"(reopt factor={args.reopt_kappa_factor:.3f})"
            )

        # Patch width_opt in racecar.ini to the configured optimizer width.
        # wall_clearance is patched separately and tightens the lateral center
        # bounds without changing width_opt.
        optimizer_width = args.car_width
        patched_ini = re.sub(
            r'(optim_opts_mintime\s*=\s*\{"width_opt":\s*)[\d.]+',
            rf'\g<1>{optimizer_width:.3f}',
            original_ini_content,
        )

        print(f"  Patched racecar.ini: width_opt -> {optimizer_width:.3f} "
              f"(configured optimizer width)")

        patched_ini = patch_mintime_optim_bool_option(
            patched_ini,
            "preserve_width_opt",
            True,
        )
        print("  Patched racecar.ini: preserve_width_opt -> true")

        patched_ini = patch_mintime_optim_bool_option(
            patched_ini,
            "direct_reopt_tube",
            True,
        )
        print(
            "  Patched racecar.ini: direct_reopt_tube -> true "
            "(preserve mintime reference for curvature repair)"
        )

        patched_ini = patch_mintime_optim_numeric_option(
            patched_ini,
            "wall_clearance",
            args.wall_clearance,
        )
        patched_ini = patch_mintime_optim_numeric_option(
            patched_ini,
            "wall_clearance_guard",
            args.wall_clearance_guard,
        )
        print(
            f"  Patched racecar.ini: wall_clearance -> "
            f"{args.wall_clearance:.3f}, guard -> "
            f"{args.wall_clearance_guard:.3f} "
            "(guard is an internal optimizer buffer)"
        )

        patched_ini = patch_mintime_optim_numeric_option(
            patched_ini,
            "w_veh_reopt",
            optimizer_width,
        )
        patched_ini = patch_mintime_optim_numeric_option(
            patched_ini,
            "w_tr_reopt",
            optimizer_width + 2.0 * args.reopt_free_dev,
        )
        print(
            "  Patched racecar.ini: reopt widths -> "
            f"w_veh_reopt={optimizer_width:.3f}, "
            f"w_tr_reopt={optimizer_width + 2.0 * args.reopt_free_dev:.3f} "
            f"(free_dev={args.reopt_free_dev:.3f}m)"
        )

        patched_ini = patch_mintime_optim_numeric_option(
            patched_ini,
            "penalty_raceline_curvature",
            args.curvature_penalty_weight if args.strict_curvlim else 0.0,
        )
        patched_ini = patch_mintime_optim_numeric_option(
            patched_ini,
            "curvlim_cost_margin",
            args.curvature_penalty_margin,
        )
        print(
            "  Patched racecar.ini: raceline curvature penalty -> "
            f"{args.curvature_penalty_weight:g}, "
            f"margin -> {args.curvature_penalty_margin:.2f}"
        )

        if args.max_speed is not None:
            patched_ini = re.sub(
                r'("v_max":\s*)[\d.]+',
                rf'\g<1>{args.max_speed:.3f}',
                patched_ini,
            )
            print(f"  Patched racecar.ini: v_max -> {args.max_speed:.3f}m/s")

        # Match TUM's optimization step size to the prepared .npz reference
        # spacing.  The dense exported CSV spacing is patched separately below.
        patched_ini = re.sub(
            r'("stepsize_reg":\s*)[\d.]+',
            rf'\g<1>{args.optimizer_spacing:.3f}',
            patched_ini,
        )
        print(f"  Patched racecar.ini: stepsize_reg -> {args.optimizer_spacing:.3f}m")

        # Patch stepsize_interp_after_opt to produce denser output waypoints.
        patched_ini = re.sub(
            r'("stepsize_interp_after_opt":\s*)[\d.]+',
            rf'\g<1>{args.waypoint_spacing:.3f}',
            patched_ini,
        )
        print(f"  Patched racecar.ini: stepsize_interp_after_opt -> {args.waypoint_spacing:.3f}m")
        # Write all ini patches at once
        with open(racecar_ini, 'w') as f:
            f.write(patched_ini)

        run_step(
            f"Step 1: Optimize trajectory ({args.opt_type}, {track_name})",
            [sys.executable, 'main_globaltraj.py'],
            cwd=global_opt_dir,
        )
    finally:
        # Always restore original files
        with open(main_py, 'w') as f:
            f.write(original_content)
        with open(racecar_ini, 'w') as f:
            f.write(original_ini_content)

    if not os.path.exists(tum_output):
        print(f"  ERROR: TUM output not found: {tum_output}")
        sys.exit(1)

    if args.strict_curvlim:
        max_kappa = max_abs_kappa_from_tum_csv(tum_output)
        if max_kappa is None:
            print("  ERROR: Could not parse kappa from TUM output CSV")
            sys.exit(1)
        if max_kappa > curvlim:
            print(
                "  ERROR: TUM output still exceeds curvlim "
                f"({max_kappa:.3f} > {curvlim:.3f} rad/m); "
                "not exporting a centerline fallback. Increase "
                "curvature_penalty_weight or lower curvature_penalty_margin."
            )
            sys.exit(1)

    # ---- Step 2: Convert to MPC format --------------------------------------
    print(f"\n{'=' * 64}")
    print(f"  Step 2: Convert to MPC format (psi += pi/2, ';' -> ',')")
    print(f"{'=' * 64}")

    os.makedirs(os.path.dirname(output_csv), exist_ok=True)

    # Intermediate 7-column CSV
    intermediate_csv = output_csv + '.7col'
    n_waypoints = convert_tum_to_mpc(
        tum_output, intermediate_csv,
        max_speed=args.max_speed, min_speed=args.min_speed
    )

    # ---- Step 3: Add wall distances -----------------------------------------
    wall_script = os.path.join(scripts_dir, 'compute_wall_distances.py')
    if not os.path.exists(wall_script):
        print(f"  ERROR: Wall script not found: {wall_script}")
        sys.exit(1)

    wall_cmd = [
        sys.executable, wall_script,
        '--map', args.map,
        '--trajectory', intermediate_csv,
        '--output', output_csv,
        '--max-distance', str(args.max_ray_distance),
        '--car-width', str(args.car_width),
        '--wall-clearance', str(args.wall_clearance),
    ]
    run_step("Step 3: Compute ray-cast wall distances", wall_cmd)

    # Clean up intermediate file
    if os.path.exists(intermediate_csv):
        os.remove(intermediate_csv)

    # ---- Step 4: Verify ------------------------------------------------------
    print(f"\n{'=' * 64}")
    print(f"  Step 4: Verify output")
    print(f"{'=' * 64}")
    ok = verify_output(
        output_csv,
        curvlim=curvlim if args.strict_curvlim else None,
        car_width=args.car_width,
        wall_clearance=args.wall_clearance,
    )

    # ---- Visualization --------------------------------------------------------
    viz_path = output_csv.replace('.csv', '_viz.png')
    try:
        visualize_raceline(args.map, output_csv, viz_path)
    except Exception as e:
        print(f"  WARNING: Visualization failed: {e}")

    # ---- Done ----------------------------------------------------------------
    print(f"\n{'=' * 64}")
    if ok:
        print(f"  SUCCESS: Trajectory ready at {output_csv}")
    else:
        print(f"  DONE (with warnings): {output_csv}")
    print(f"  Waypoints: {n_waypoints}")
    print(f"{'=' * 64}\n")

    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
