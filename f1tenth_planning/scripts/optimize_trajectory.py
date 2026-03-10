#!/usr/bin/env python3
"""
Universal one-click trajectory optimization pipeline for F1Tenth MPC.

Works with ANY map (.pgm/.yaml) -- no pre-made TUM track CSV required.

Pipeline steps:
  Step 0: Extract centerline + track widths from map image  (NEW)
  Step 1: Run TUM global_racetrajectory_optimization
  Step 2: Convert to MPC format (psi += pi/2, delimiter, clamp velocity)
  Step 3: Add wall distances via ray-cast  (optional, off by default)
  Step 4: Verify output trajectory

Output CSV format (7 or 9 columns):
  s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2 [, d_left_m, d_right_m]

Usage:
    # Full pipeline from a map file
    python3 optimize_trajectory.py --map f1tenth_sim/maps/Spielberg_map.yaml
    python3 optimize_trajectory.py --map f1tenth_sim/maps/my_track_map.yaml --max-speed 8.0

    # With wall distances (enables 9-col output)
    python3 optimize_trajectory.py --map f1tenth_sim/maps/my_track_map.yaml --with-walls

    # Skip extraction if TUM track CSV already exists
    python3 optimize_trajectory.py --map f1tenth_sim/maps/Spielberg_map.yaml --skip-extract

    # Skip both extraction and optimization (re-convert only)
    python3 optimize_trajectory.py --map f1tenth_sim/maps/Spielberg_map.yaml --skip-extract --skip-optimize

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

import cv2
import numpy as np
import yaml
from scipy.interpolate import splprep, splev
from scipy.spatial import cKDTree


# =============================================================================
#  Utility helpers (kept from original)
# =============================================================================

def find_workspace_root():
    """Walk up from this script to find the workspace root (contains global_racetrajectory_optimization/)."""
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(10):
        if os.path.isdir(os.path.join(d, 'global_racetrajectory_optimization')):
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
    result = subprocess.run(cmd, cwd=cwd, env=env)
    if result.returncode != 0:
        print(f"\n  ERROR: {label} failed (exit code {result.returncode})")
        sys.exit(result.returncode)


# =============================================================================
#  Step 0 -- Map loading, boundary extraction, centerline, track widths
# =============================================================================

def compute_centerline_thinned_loop(map_img, resolution, origin, wall_thresh, num_points):
    """
    Compute the track centerline using a GVD (Generalised Voronoi Diagram) approach:

      1. Label the two wall obstacle components (outer walls + inner island)
      2. Compute per-obstacle Euclidean distance transforms → GVD boundary
         = free pixels equidistant (within gvd_thresh px) from BOTH obstacles
      3. Zhang-Suen thinning of the GVD boundary → 1-px skeleton ring
         (far fewer junctions than thinning the full free space)
      4. Junction-cluster contraction: collapse dense junction pixels into nodes;
         degree-2 chains become graph edges
      5. DFS on the cluster graph: find the longest simple loop back to start
      6. Expand to pixel path, B-spline smooth + uniform resample

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
    from scipy.ndimage import label as _sc_label, distance_transform_edt as _edt2
    from collections import defaultdict

    h, w = map_img.shape[:2]
    free = (map_img >= wall_thresh).astype(np.uint8)
    edt  = cv2.distanceTransform(free, cv2.DIST_L2, 5)

    # 1. Zhang-Suen thinning ------------------------------------------------
    skel_mat = cv2.ximgproc.thinning(free * 255,
                                      thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)
    skel_set = set(map(tuple, np.column_stack(np.where(skel_mat > 0)).tolist()))
    n_skel   = len(skel_set)
    print(f"  Thinned skeleton: {n_skel} pixels")
    if n_skel < 20:
        raise RuntimeError("Skeleton too sparse; check wall_thresh or map quality.")

    # 2. Identify junction pixels (degree ≠ 2) and cluster them ------------
    def _deg(r, c):
        return sum(1 for dr in (-1, 0, 1) for dc in (-1, 0, 1)
                   if (dr or dc) and (r+dr, c+dc) in skel_set)

    junc_mask = np.zeros((h, w), dtype=np.uint8)
    for rc in skel_set:
        if _deg(*rc) != 2:
            junc_mask[rc] = 1

    junc_labeled, n_cl = _sc_label(junc_mask, structure=np.ones((3, 3), dtype=int))

    # Representative pixel for each cluster: the one with max EDT
    cl_rep = {}          # cluster_id -> (r, c) representative pixel
    for cid in range(1, n_cl + 1):
        ys, xs = np.where(junc_labeled == cid)
        best   = int(np.argmax(edt[ys, xs]))
        cl_rep[cid] = (int(ys[best]), int(xs[best]))

    print(f"  Junction clusters: {n_cl}")

    # 3. Walk degree-2 chains between clusters → graph edges ---------------
    # chain_edges: list of (cluster_a, cluster_b, length_px, pixel_list)
    walked       = set()
    chain_edges  = []

    for rc in skel_set:
        cid = int(junc_labeled[rc])
        if cid == 0:
            continue          # degree-2 pixel; chains are started from clusters
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if not (dr or dc):
                    continue
                first = (rc[0]+dr, rc[1]+dc)
                if first not in skel_set:
                    continue
                if int(junc_labeled[first]) > 0:
                    continue  # cluster→cluster direct connection (< 2px chain)

                # Unique key so we don't walk the same chain twice
                chain_key = (rc, first)  # ordered: start from cluster pixel
                if chain_key in walked:
                    continue
                walked.add(chain_key)

                # Walk the degree-2 chain
                path = [rc, first]
                plen = float(np.hypot(dc, dr))
                prev, cur = rc, first
                while True:
                    nbrs = [(cur[0]+dr2, cur[1]+dc2)
                            for dr2 in (-1, 0, 1) for dc2 in (-1, 0, 1)
                            if (dr2 or dc2)
                            and (cur[0]+dr2, cur[1]+dc2) in skel_set
                            and (cur[0]+dr2, cur[1]+dc2) != prev]
                    if not nbrs:
                        break   # dangling end — not connected to another cluster
                    nxt = nbrs[0]
                    plen += float(np.hypot(nxt[1]-cur[1], nxt[0]-cur[0]))
                    path.append(nxt)
                    prev, cur = cur, nxt
                    other_cid = int(junc_labeled[cur])
                    if other_cid > 0:
                        if other_cid != cid:
                            chain_edges.append((cid, other_cid, plen, path))
                        break

    print(f"  Chain edges: {len(chain_edges)}")

    # Build adjacency list (multi-graph allowed)
    adj = defaultdict(list)  # cluster_id -> [(other_cid, weight, edge_idx)]
    for ei, (a, b, wt, _) in enumerate(chain_edges):
        adj[a].append((b, wt, ei))
        adj[b].append((a, wt, ei))

    # 4. Start cluster: closest to the pixel with maximum EDT --------------
    skel_arr = np.array(list(skel_set))
    edt_vals = edt[skel_arr[:, 0], skel_arr[:, 1]]
    max_rc   = tuple(skel_arr[int(np.argmax(edt_vals))].tolist())
    start_cl = min(range(1, n_cl + 1),
                   key=lambda c: (cl_rep[c][0]-max_rc[0])**2
                               + (cl_rep[c][1]-max_rc[1])**2)

    nr0, nc0 = cl_rep[start_cl]
    print(f"  Start cluster {start_cl} at pixel ({nr0},{nc0}), "
          f"EDT={edt[nr0,nc0]:.1f}px, "
          f"world=({origin[0]+nc0*resolution:.2f},"
          f"{origin[1]+(h-1-nr0)*resolution:.2f})")

    # 5. DFS to find the longest simple loop back to start_cl --------------
    # Approach: for each outgoing edge from start_cl, do a DFS (banning start_cl)
    # and look for paths that end at start_cl via an unused edge.
    banned    = {start_cl}
    best_path = []     # list of (cluster_id, edge_index) pairs on the loop
    best_len  = 0.0

    def _dfs(cur, path, used_edges, path_set, total_w):
        nonlocal best_path, best_len
        # Check if we can close the loop back to start_cl
        for nb2, w2, ei2 in adj[cur]:
            if nb2 == start_cl and ei2 not in used_edges:
                loop_w = total_w + w2
                if loop_w > best_len:
                    best_len = loop_w
                    best_path = path + [(nb2, ei2)]
                # Don't return yet; might find a longer loop by continuing
        # Explore further
        for nb, wt, ei in adj[cur]:
            if nb in banned or nb in path_set or ei in used_edges:
                continue
            path_set.add(nb)
            used_edges.add(ei)
            _dfs(nb, path + [(nb, ei)], used_edges, path_set, total_w + wt)
            path_set.discard(nb)
            used_edges.discard(ei)

    for nb0, w0, ei0 in adj[start_cl]:
        _dfs(nb0, [(nb0, ei0)], {ei0}, {nb0}, w0)

    if not best_path:
        raise RuntimeError(
            "No loop found in the skeleton graph.  "
            "The map may not have a closed corridor; try a different wall_clearance."
        )

    loop_len_m = best_len * resolution
    print(f"  Best loop: {len(best_path)} nodes, {loop_len_m:.1f} m")

    # 6. Expand to full pixel path -----------------------------------------
    pixel_path = [cl_rep[start_cl]]
    prev_cl    = start_cl
    for next_cl, ei in best_path:
        pix_list = chain_edges[ei][3]
        # Orient pixel list from prev_cl toward next_cl
        if int(junc_labeled[pix_list[0]]) == prev_cl:
            seg = pix_list
        elif int(junc_labeled[pix_list[-1]]) == prev_cl:
            seg = list(reversed(pix_list))
        else:
            seg = pix_list   # fallback; may be slightly mis-oriented
        # Skip the first pixel of seg (duplicate of last added pixel)
        pixel_path.extend(seg[1:])
        prev_cl = next_cl

    pixel_path = np.array(pixel_path)   # shape (M, 2)
    rows = pixel_path[:, 0]
    cols = pixel_path[:, 1]

    wx  = origin[0] + cols * resolution
    wy  = origin[1] + (h - 1 - rows) * resolution
    hw  = edt[rows, cols] * resolution   # half-widths [m]

    skel_world = np.column_stack([wx, wy])
    pg          = float(np.linalg.norm(skel_world[-1] - skel_world[0]))
    px_len      = float(np.sum(np.linalg.norm(np.diff(skel_world, axis=0), axis=1)))
    print(f"  Pixel path: {len(pixel_path)} pts, {px_len:.1f} m, loop_gap={pg:.3f} m")

    # 7. Remove near-duplicates → B-spline smooth + uniform resample ------
    dd   = np.linalg.norm(np.diff(skel_world, axis=0), axis=1)
    keep = np.concatenate([[True], dd > 0.005])
    skel_world = skel_world[keep]
    hw         = hw[keep]

    closed = np.vstack([skel_world, skel_world[0]])
    try:
        tck, _ = splprep([closed[:, 0], closed[:, 1]],
                         s=max(5.0, len(skel_world) * 0.1), per=True)
    except Exception as exc:
        raise RuntimeError(f"B-spline on skeleton loop failed: {exc}") from exc

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
        raise FileNotFoundError(f"Could not load map image: {image_path}")

    # Read occupied_thresh from map YAML (ROS convention).
    # Used to compute a consistent wall threshold across the pipeline.
    occupied_thresh = float(config.get('occupied_thresh', 0.65))

    return img, resolution, origin, occupied_thresh


def extract_boundaries(img, resolution, origin, wall_thresh=140):
    """
    Extract outer and inner track boundaries from a map image.

    Handles two map types automatically:
      - Designed maps (e.g. Spielberg): white background, boundary touches
        image edges.  Outer track wall = largest child of the image-edge
        contour, inner wall = smallest leaf contour.
      - SLAM maps (e.g. my_track_map): grey background, track "floats" in
        the image.  Outer wall = largest contour, inner wall = its largest
        child.

    Parameters
    ----------
    wall_thresh : int
        Pixel values >= wall_thresh are considered free space.  Must match
        the threshold used by compute_wall_distances.py to avoid a grey-zone
        that silently eats into the safety budget.

    Returns
    -------
    outer_world : np.ndarray, shape (N, 2) -- outer boundary in world coords
    inner_world : np.ndarray, shape (M, 2) -- inner boundary in world coords
    """
    # Use the same wall threshold as compute_wall_distances.py:
    #   wall_thresh = int(255 * (1.0 - occupied_thresh))
    # Pixels below this value are walls.  Using a consistent threshold
    # prevents a ~1-2 pixel grey zone from eating into the safety budget.
    free_space = (img >= wall_thresh).astype(np.uint8)
    contours, hierarchy = cv2.findContours(
        free_space, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE
    )

    if not contours:
        raise RuntimeError("No contours found in the map image")

    # Sort contours by area (descending)
    areas = [(i, cv2.contourArea(c)) for i, c in enumerate(contours)]
    areas.sort(key=lambda x: x[1], reverse=True)

    # Detect map type: does the largest contour touch all four image edges?
    largest_pts = contours[areas[0][0]].reshape(-1, 2)
    is_image_boundary = (
        np.any(largest_pts[:, 0] == 0)
        and np.any(largest_pts[:, 0] == img.shape[1] - 1)
        and np.any(largest_pts[:, 1] == 0)
        and np.any(largest_pts[:, 1] == img.shape[0] - 1)
    )

    if is_image_boundary:
        # Designed map ---------------------------------------------------------
        # Outer wall = largest child of the image-boundary contour
        outer_idx = next(
            idx for idx, _ in areas[1:]
            if hierarchy[0][idx][3] == areas[0][0]
        )
        # Inner wall = smallest leaf contour (no children)
        inner_idx = next(
            idx for idx, _ in reversed(areas)
            if hierarchy[0][idx][2] == -1
        )
        print("  Map type: designed (image-edge boundary detected)")
        slam_map = False
    else:
        # SLAM map -------------------------------------------------------------
        outer_idx = areas[0][0]
        # Inner walls = ALL children of the outer contour (all inner obstacles,
        # e.g. the central island AND any smaller obstacles like a U-notch).
        # Using only the largest child leaves smaller inner obstacles undetected,
        # causing the midpoint-centerline to pass through them.
        children = [
            (idx, a) for idx, a in areas[1:]
            if hierarchy[0][idx][3] == outer_idx
        ]
        if not children:
            raise RuntimeError(
                "Could not find inner boundary -- is this a closed track?"
            )
        print("  Map type: SLAM (floating track)")
        slam_map = True

    def pixel_to_world(points):
        world = np.zeros((len(points), 2))
        world[:, 0] = points[:, 0] * resolution + origin[0]
        world[:, 1] = (img.shape[0] - points[:, 1]) * resolution + origin[1]
        return world

    outer_world = pixel_to_world(contours[outer_idx].reshape(-1, 2))

    if not slam_map:
        # Designed map: single inner boundary
        inner_world = pixel_to_world(contours[inner_idx].reshape(-1, 2))
        print(f"  Outer boundary: {len(outer_world)} points")
        print(f"  Inner boundary: {len(inner_world)} points")
    else:
        # SLAM map: merge ALL inner obstacle boundaries
        inner_parts = []
        for child_idx, child_area in sorted(children, key=lambda x: x[1], reverse=True):
            pts = pixel_to_world(contours[child_idx].reshape(-1, 2))
            inner_parts.append(pts)
        inner_world = np.vstack(inner_parts)
        total_inner_pts = sum(len(p) for p in inner_parts)
        print(f"  Outer boundary: {len(outer_world)} points")
        print(f"  Inner boundary: {total_inner_pts} points "
              f"({len(inner_parts)} obstacle(s))")

    return outer_world, inner_world, slam_map


def compute_centerline(outer_world, inner_world, num_points=1000,
                       map_img=None, map_resolution=None, map_origin=None,
                       wall_thresh=140):
    """
    Compute a smooth, periodic centerline between the two boundaries.

    For each sampled point on the outer boundary, finds the closest point on
    the inner boundary and takes the midpoint.  The result is smoothed with a
    periodic B-spline and optionally corrected to avoid wall collisions using
    a distance-transform.

    Returns
    -------
    centerline : np.ndarray, shape (num_points, 2)
    inner_tree : cKDTree -- KD-tree built from inner_world
    outer_tree : cKDTree -- KD-tree built from outer_world
    """
    outer_tree = cKDTree(outer_world)
    inner_tree = cKDTree(inner_world)

    step = max(1, len(outer_world) // num_points)
    outer_sampled = outer_world[::step]

    centerline_raw = []
    for outer_pt in outer_sampled:
        d_inner, idx = inner_tree.query(outer_pt)
        inner_pt = inner_world[idx]
        centerline_raw.append((inner_pt + outer_pt) / 2.0)
    centerline_raw = np.array(centerline_raw)

    # Remove near-duplicate points
    diff = np.diff(centerline_raw, axis=0)
    dist = np.linalg.norm(diff, axis=1)
    mask = np.concatenate([[True], dist > 0.01])
    centerline_raw = centerline_raw[mask]

    # ---- Pre-smoothing collision correction + medial-axis projection --------
    # Applied to the RAW centerline BEFORE the B-spline so that the spline
    # interpolates smoothly between all corrected points (instead of making
    # a sudden jump between corrected and uncorrected neighbours).
    #
    # For each raw point that lands inside a wall (EDT < min_clearance_px):
    #   Step 1: jump to nearest free pixel (wide 3m search radius)
    #   Step 2: gradient ascent toward local EDT maximum (corridor medial axis)
    #
    # Points already in free space are left UNCHANGED.
    if map_img is not None:
        free = (map_img >= wall_thresh).astype(np.uint8)
        dist_transform = cv2.distanceTransform(free, cv2.DIST_L2, 5)
        img_h, img_w = map_img.shape
        min_clearance_px = 2.0      # 2px = 0.10m minimum EDT to be "in free space"
        max_ascent_steps  = 80      # max 80 gradient steps = 4m max travel

        n_fixed = 0
        for i in range(len(centerline_raw)):
            col = int((centerline_raw[i, 0] - map_origin[0]) / map_resolution)
            row = int(img_h - (centerline_raw[i, 1] - map_origin[1]) / map_resolution)

            # Skip points already in free space with sufficient clearance
            if (0 <= row < img_h and 0 <= col < img_w
                    and dist_transform[row, col] >= min_clearance_px):
                continue

            # --- Step 1: jump to nearest free pixel ---
            best_r, best_c = row, col
            best_sq = float('inf')
            found = False
            search_r = 60       # 60px = 3m search radius
            for dr in range(-search_r, search_r + 1):
                for dc in range(-search_r, search_r + 1):
                    nr, nc = row + dr, col + dc
                    sq = dr * dr + dc * dc
                    if sq >= best_sq:
                        continue
                    if (0 <= nr < img_h and 0 <= nc < img_w
                            and dist_transform[nr, nc] >= min_clearance_px):
                        best_sq = sq
                        best_r, best_c = nr, nc
                        found = True
            if not found:
                continue
            row, col = best_r, best_c

            # --- Step 2: gradient ascent toward the local EDT maximum ---
            for _step in range(max_ascent_steps):
                best_val = dist_transform[row, col]
                best_r2, best_c2 = row, col
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        if dr == 0 and dc == 0:
                            continue
                        nr, nc = row + dr, col + dc
                        if (0 <= nr < img_h and 0 <= nc < img_w
                                and dist_transform[nr, nc] > best_val):
                            best_val = dist_transform[nr, nc]
                            best_r2, best_c2 = nr, nc
                if best_r2 == row and best_c2 == col:
                    break   # at local EDT maximum (medial axis)
                row, col = best_r2, best_c2

            centerline_raw[i, 0] = col * map_resolution + map_origin[0]
            centerline_raw[i, 1] = (img_h - row) * map_resolution + map_origin[1]
            n_fixed += 1

        if n_fixed > 0:
            print(f"  Collision correction + medial-axis: fixed {n_fixed}/{len(centerline_raw)} raw pts")
            # Re-apply near-duplicate removal after correction (corrected points
            # can converge to the same local EDT maximum, creating duplicates).
            diff2 = np.diff(centerline_raw, axis=0)
            dist2 = np.linalg.norm(diff2, axis=1)
            mask2 = np.concatenate([[True], dist2 > 0.01])
            centerline_raw = centerline_raw[mask2]

    # Smooth with a periodic B-spline AFTER correction.
    # Using the corrected raw points as input ensures the spline smoothly
    # interpolates between the fixed-up positions (no sudden kinks).
    centerline_closed = np.vstack([centerline_raw, centerline_raw[0]])
    tck, _u = splprep(
        [centerline_closed[:, 0], centerline_closed[:, 1]],
        s=max(5.0, len(centerline_raw) * 0.5),   # 0.5x N: smooth yet accurate
        per=True,
    )
    u_new = np.linspace(0, 1, num_points, endpoint=False)
    centerline = np.array(splev(u_new, tck)).T

    return centerline, inner_tree, outer_tree


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
                         max_dist=5.0, wall_thresh=140):
    """
    Ray-cast from each centerline point perpendicular to path direction to
    find wall distances (right and left).

    Parameters
    ----------
    centerline : np.ndarray, shape (N, 2)
    map_img : np.ndarray -- grayscale image
    resolution : float -- meters per pixel
    origin : np.ndarray -- map origin [x, y, theta]
    max_dist : float -- maximum ray-cast distance in metres
    wall_thresh : int -- pixel values >= this are free space (must match
        compute_wall_distances.py threshold to avoid grey-zone errors)

    Returns
    -------
    w_right : np.ndarray, shape (N,) -- distance to right wall
    w_left  : np.ndarray, shape (N,) -- distance to left wall
    """
    # Consistent wall threshold: pixels below wall_thresh are walls.
    free = (map_img >= wall_thresh).astype(np.uint8)
    n = len(centerline)

    # Compute tangent vectors (periodic)
    tangents = np.zeros_like(centerline)
    tangents[:-1] = np.diff(centerline, axis=0)
    tangents[-1] = centerline[0] - centerline[-1]
    t_len = np.linalg.norm(tangents, axis=1, keepdims=True)
    tangents = tangents / np.maximum(t_len, 1e-6)

    # Normal: rotate tangent 90 deg CCW  ->  (-ty, tx)
    normals = np.column_stack([-tangents[:, 1], tangents[:, 0]])

    w_right = np.zeros(n)
    w_left = np.zeros(n)
    step_size = resolution * 0.5

    for i in range(n):
        # +normal direction -> left
        for d in np.arange(step_size, max_dist, step_size):
            pt = centerline[i] + d * normals[i]
            col = int((pt[0] - origin[0]) / resolution)
            row = int(map_img.shape[0] - (pt[1] - origin[1]) / resolution)
            if not (0 <= row < map_img.shape[0] and 0 <= col < map_img.shape[1]) or not free[row, col]:
                w_left[i] = d
                break
        else:
            w_left[i] = max_dist

        # -normal direction -> right
        for d in np.arange(step_size, max_dist, step_size):
            pt = centerline[i] - d * normals[i]
            col = int((pt[0] - origin[0]) / resolution)
            row = int(map_img.shape[0] - (pt[1] - origin[1]) / resolution)
            if not (0 <= row < map_img.shape[0] and 0 <= col < map_img.shape[1]) or not free[row, col]:
                w_right[i] = d
                break
        else:
            w_right[i] = max_dist

    return w_right, w_left


def smooth_track_widths(centerline, w_right, w_left, min_width=0.3):
    """
    Post-process measured track widths to avoid TUM optimizer spline-normal
    crossing errors.

    1. **Curvature cap** -- at tight corners the track width must be less than
       the radius of curvature, otherwise normals cross.  We enforce
       ``w <= 0.8 * R`` where ``R = 1 / |kappa|``.
    2. **Smoothing** -- Gaussian-filter to remove measurement noise.
    3. **Minimum enforcement** -- clamp to *min_width* so the optimizer has
       some room to work with.
    """
    from scipy.ndimage import uniform_filter1d, gaussian_filter1d

    n = len(centerline)

    # Compute curvature on a Gaussian-smoothed copy of the centerline so that
    # SLAM noise spikes (which create artificially high kappa) don't cause
    # over-aggressive width capping.  sigma=3 removes pixel-scale noise while
    # preserving genuine corners.
    sigma = 3.0
    cx_smooth = gaussian_filter1d(centerline[:, 0], sigma=sigma, mode='wrap')
    cy_smooth = gaussian_filter1d(centerline[:, 1], sigma=sigma, mode='wrap')
    dx = np.gradient(cx_smooth)
    dy = np.gradient(cy_smooth)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    kappa = (dx * ddy - dy * ddx) / np.maximum((dx ** 2 + dy ** 2) ** 1.5, 1e-10)
    R = 1.0 / np.maximum(np.abs(kappa), 1e-6)

    # Cap widths to 45% of curvature radius PER SIDE (total ≤ 0.9 * R).
    # TUM's prep_track checks normals within horizon=10 pts; for crossings
    # not to occur each boundary must stay within R of the centreline.
    # Using 0.45*R per side (total 0.9*R) gives a comfortable safety margin.
    cap = 0.45 * R
    w_right_c = np.minimum(w_right, cap)
    w_left_c = np.minimum(w_left, cap)

    # Smooth with a moving average (wrap-around for closed track)
    kernel = max(5, n // 100)
    w_right_s = uniform_filter1d(w_right_c, size=kernel, mode='wrap')
    w_left_s = uniform_filter1d(w_left_c, size=kernel, mode='wrap')

    # Enforce minimum width
    w_right_s = np.maximum(w_right_s, min_width)
    w_left_s = np.maximum(w_left_s, min_width)

    n_capped_r = int(np.sum(w_right_c < w_right))
    n_capped_l = int(np.sum(w_left_c < w_left))
    if n_capped_r + n_capped_l > 0:
        print(f"  Width curvature-capped: right {n_capped_r}, left {n_capped_l}")
    print(f"  Width after smoothing: right [{w_right_s.min():.3f}, {w_right_s.max():.3f}] m, "
          f"left [{w_left_s.min():.3f}, {w_left_s.max():.3f}] m")

    return w_right_s, w_left_s


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


# =============================================================================
#  Step 2 -- TUM -> MPC format conversion (kept from original)
# =============================================================================

def convert_tum_to_mpc(input_csv, output_csv, max_speed=None):
    """
    Convert TUM global optimizer output to MPC-compatible CSV.

    Changes:
      - Delimiter: semicolon -> comma
      - Heading: psi += pi/2, wrapped to [-pi, pi]
      - Header: standardised to ``# s_m,x_m,y_m,...``
      - Optional: clamp velocity to max_speed
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

    clamped = 0
    with open(output_csv, 'w') as f:
        f.write('# s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2\n')
        for row in rows:
            # Apply psi + pi/2 heading correction
            psi = float(row[3])
            psi_corrected = psi + math.pi / 2.0
            # Wrap to [-pi, pi]
            psi_corrected = (psi_corrected + math.pi) % (2 * math.pi) - math.pi
            row[3] = f"{psi_corrected:.7f}"

            # Clamp velocity
            if max_speed is not None:
                vx = float(row[5])
                if vx > max_speed:
                    row[5] = f"{max_speed:.7f}"
                    clamped += 1

            f.write(','.join(row[:7]) + '\n')

    print(f"  Converted {len(rows)} waypoints (psi += pi/2)")
    if clamped > 0:
        print(f"  Clamped {clamped}/{len(rows)} velocities to {max_speed:.1f} m/s")
    return len(rows)


# =============================================================================
#  Velocity profile recomputation (forward-backward pass)
# =============================================================================

def recompute_velocity_profile(csv_path, max_speed, ay_max=7.3, ax_max=7.3, ax_min=-7.3):
    """
    Recompute the vx and ax columns in a trajectory CSV using the actual kappa.

    Uses a 3-pass forward-backward algorithm:
      1. Point speed limit: v[i] = sqrt(ay_max / |kappa[i]|), clamped to max_speed
      2. Forward pass: enforce acceleration limit
      3. Backward pass: enforce deceleration limit

    Parameters
    ----------
    ay_max : float  lateral acceleration limit [m/s²] (from GGV)
    ax_max : float  max longitudinal accel [m/s²]
    ax_min : float  max longitudinal decel [m/s²] (negative)
    """
    # Read CSV
    lines_in = open(csv_path).readlines()
    header_lines = [ln for ln in lines_in if ln.strip().startswith('#')]
    data_lines   = [ln for ln in lines_in if ln.strip() and not ln.strip().startswith('#')]

    rows = [[float(v) for v in ln.strip().split(',')] for ln in data_lines]
    n = len(rows)
    if n == 0:
        return

    # Arc-length spacing from s_m column
    s = np.array([r[0] for r in rows])
    ds = np.diff(s)
    ds = np.append(ds, ds[-1])  # pad last step with same as preceding
    ds = np.maximum(ds, 1e-4)

    kappa = np.array([r[4] for r in rows])

    # 1. Point speed limit from lateral friction
    v_lat = np.sqrt(ay_max / np.maximum(np.abs(kappa), 1e-6))
    v_max = np.minimum(v_lat, max_speed)

    # 2. Forward pass (enforce ax_max)
    v_fwd = v_max.copy()
    for i in range(1, n):
        v_fwd[i] = min(v_max[i], math.sqrt(v_fwd[i-1]**2 + 2.0 * ax_max * ds[i-1]))

    # 3. Backward pass (enforce ax_min = deceleration)
    v_bwd = v_max.copy()
    for i in range(n - 2, -1, -1):
        v_bwd[i] = min(v_max[i], math.sqrt(v_bwd[i+1]**2 - 2.0 * ax_min * ds[i]))

    v_final = np.minimum(v_fwd, v_bwd)

    # 4. Compute ax from vx profile
    ax_final = np.zeros(n)
    for i in range(n - 1):
        ax_final[i] = (v_final[i+1]**2 - v_final[i]**2) / (2.0 * ds[i])
    ax_final[-1] = ax_final[-2]

    # 5. Write back to CSV (update columns 5 and 6)
    ncols = len(rows[0])
    for i, r in enumerate(rows):
        r[5] = v_final[i]
        r[6] = ax_final[i]

    with open(csv_path, 'w') as fout:
        fout.writelines(header_lines)
        for r in rows:
            fout.write(','.join(f'{v:.7f}' for v in r) + '\n')

    over = int(np.sum(v_final**2 * np.abs(kappa) > ay_max * 1.01))
    print(f"  Velocity profile recomputed: [{v_final.min():.2f}, {v_final.max():.2f}] m/s  "
          f"({over} lateral-limit violations remain)")


# =============================================================================
#  Step 4 -- Verification (kept from original)
# =============================================================================

def verify_output(csv_path):
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

    ok = True
    if max_psi_err > math.radians(5):
        print(
            f"  WARNING: Heading deviates from atan2(dy,dx) by up to "
            f"{math.degrees(max_psi_err):.1f} deg"
        )
    if ncols >= 9 and (w[:, 7].min() < 0.2 or w[:, 8].min() < 0.2):
        print(f"  WARNING: Some wall distances < 0.2 m")
    if n < 100:
        print(f"  WARNING: Very few waypoints ({n})")
        ok = False
    return ok


# =============================================================================
#  Main pipeline
# =============================================================================

def main():
    workspace = find_workspace_root()
    if not workspace:
        print(
            "ERROR: Could not find workspace root "
            "(no global_racetrajectory_optimization/ found)"
        )
        sys.exit(1)

    scripts_dir = os.path.dirname(os.path.abspath(__file__))
    global_opt_dir = os.path.join(workspace, 'global_racetrajectory_optimization')
    default_output = os.path.join(workspace, 'f1tenth_planning', 'trajectories')

    # ---- Argument parsing ----------------------------------------------------
    parser = argparse.ArgumentParser(
        description='Universal one-click trajectory optimization for F1Tenth MPC',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )

    parser.add_argument(
        '--map', '-m', required=True,
        help='Path to map .yaml file (e.g. f1tenth_sim/maps/Spielberg_map.yaml)',
    )
    parser.add_argument(
        '--track-name', '-t', default=None,
        help='Track name (default: derived from map filename)',
    )
    parser.add_argument(
        '--opt-type',
        choices=['shortest_path', 'mincurv', 'mincurv_iqp', 'mintime'],
        default='mincurv',
        help='Optimization type (default: mincurv)',
    )
    parser.add_argument(
        '--max-speed', type=float, default=12.0,
        help='Clamp velocity to this value [m/s] (default: 12.0)',
    )
    parser.add_argument(
        '--output', '-o', default=default_output,
        help='Output directory (default: f1tenth_planning/trajectories/)',
    )
    parser.add_argument(
        '--centerline-points', type=int, default=1000,
        help='Number of centerline points to sample (default: 1000)',
    )
    parser.add_argument(
        '--skip-extract', action='store_true',
        help='Skip centerline extraction (use existing TUM track CSV)',
    )
    parser.add_argument(
        '--skip-optimize', action='store_true',
        help='Skip TUM optimization (use existing traj_race_cl.csv)',
    )
    parser.add_argument(
        '--skip-walls', action='store_true', default=True,
        help='Skip wall distance computation (default: True, 7-col output)',
    )
    parser.add_argument(
        '--with-walls', action='store_true',
        help='Compute ray-cast wall distances (9-col output)',
    )
    parser.add_argument(
        '--car-width', type=float, default=0.30,
        help='Physical car width [m] used for wall distance margin (default: 0.30)',
    )
    parser.add_argument(
        '--wall-clearance', type=float, default=0.45,
        help='Extra clearance from walls beyond car width on each side [m] (default: 0.45). '
             'Optimizer width_opt = car_width + 2*wall_clearance',
    )
    parser.add_argument(
        '--max-ray-distance', type=float, default=5.0,
        help='Max wall ray-cast distance [m] (default: 5.0)',
    )
    parser.add_argument(
        '--min-track-width', type=float, default=None,
        help='Minimum enforced track width [m] in TUM optimizer (default: auto = 2*wall_clearance + car_width). '
             'Narrow track sections are widened to this value before optimization.',
    )
    parser.add_argument(
        '--direction', default='auto',
        choices=['auto', 'cw', 'ccw'],
        help='Track direction: auto-detect from winding order, or force cw/ccw (default: auto)',
    )
    parser.add_argument(
        '--no-fix', action='store_true',
        help='Disable automatic wall-clearance fix post-processing (default: enabled when --with-walls)',
    )
    parser.add_argument(
        '--min-clearance', type=float, default=None,
        help='Minimum wall clearance [m] for the fix step (default: same as --wall-clearance)',
    )
    parser.add_argument(
        '--smooth-factor', type=float, default=None,
        help='Spline smoothing factor s_reg for TUM optimizer (default: auto = max(25, N_points)). '
             'Increase if you get "spline normals crossed" errors on tight SLAM tracks.',
    )
    parser.add_argument(
        '--waypoint-spacing', type=float, default=0.15,
        help='Waypoint spacing [m] for the final trajectory (stepsize_interp_after_opt, default: 0.15). '
             'Smaller values produce denser waypoints (recommended for MPC).',
    )
    parser.add_argument(
        '--fix-iterations', type=int, default=10,
        help='Number of wall-clearance fix iterations (default: 10).',
    )

    args = parser.parse_args()

    # Resolve map path
    if not os.path.isabs(args.map):
        args.map = os.path.join(workspace, args.map)
    if not os.path.exists(args.map):
        print(f"ERROR: Map file not found: {args.map}")
        sys.exit(1)

    # Derive track name from map filename if not specified
    if args.track_name is None:
        basename = os.path.splitext(os.path.basename(args.map))[0]
        # Strip common suffixes like _map
        if basename.endswith('_map'):
            args.track_name = basename[:-4]
        else:
            args.track_name = basename
    track_name = args.track_name

    # Override skip_walls if --with-walls was explicitly given
    if args.with_walls:
        args.skip_walls = False

    # Fix clearance defaults: enabled when --with-walls, unless --no-fix
    args.fix_clearance = (not args.skip_walls) and (not args.no_fix)

    # Default min-clearance to wall-clearance if not set
    if args.min_clearance is None:
        args.min_clearance = args.wall_clearance

    track_csv = os.path.join(
        global_opt_dir, 'inputs', 'tracks', f'{track_name}.csv'
    )
    output_name = f'{track_name}_raceline.csv'
    # If --output looks like a file path (ends with .csv), use it directly;
    # otherwise treat it as a directory and append the default filename.
    if args.output.lower().endswith('.csv'):
        output_csv = args.output
    else:
        output_csv = os.path.join(args.output, output_name)

    # ---- Banner --------------------------------------------------------------
    print("=" * 64)
    print("  F1Tenth Universal Trajectory Optimization Pipeline")
    print("=" * 64)
    print(f"  Map:              {args.map}")
    # Compute minimum track width if not explicitly set
    if args.min_track_width is None:
        args.min_track_width = args.car_width + 2.0 * args.wall_clearance

    print(f"  Track name:       {track_name}")
    print(f"  Opt type:         {args.opt_type}")
    print(f"  Max speed:        {args.max_speed} m/s")
    print(f"  Centerline pts:   {args.centerline_points}")
    print(f"  Direction:        {args.direction}")
    print(f"  Car width:        {args.car_width} m")
    print(f"  Wall clearance:   {args.wall_clearance} m")
    print(f"  Optimizer width:  {args.car_width + 2*args.wall_clearance:.3f} m")
    print(f"  Min track width:  {args.min_track_width:.3f} m")
    print(f"  Wall distances:   {'yes' if not args.skip_walls else 'no (default)'}")
    print(f"  Fix clearance:    {'yes' if args.fix_clearance else 'no'}")
    if args.fix_clearance:
        print(f"  Min clearance:    {args.min_clearance} m")
    print(f"  Output:           {output_csv}")
    print(f"  Skip extract:     {args.skip_extract}")
    print(f"  Skip optimize:    {args.skip_optimize}")

    # ---- Step 0: Extract centerline + track widths from map ------------------
    if not args.skip_extract:
        print(f"\n{'=' * 64}")
        print(f"  Step 0: Extract centerline from map")
        print(f"{'=' * 64}")

        map_img, resolution, origin, occupied_thresh = load_map(args.map)

        # Compute wall threshold from map YAML's occupied_thresh, matching
        # the convention in compute_wall_distances.py:
        #   ROS: p = (255 - pixel) / 255; occupied if p > occupied_thresh
        #   => pixel < 255*(1 - occupied_thresh) is a wall
        wall_thresh = int(255 * (1.0 - occupied_thresh))
        print(f"  Wall threshold: {wall_thresh} (occupied_thresh={occupied_thresh:.2f})")
        print(f"  Image size: {map_img.shape[1]}x{map_img.shape[0]} px, "
              f"resolution: {resolution} m/px")

        outer_world, inner_world, is_slam_map = extract_boundaries(map_img, resolution, origin, wall_thresh=wall_thresh)

        # Auto-scale number of centerline points based on track perimeter
        # Estimate perimeter from outer boundary
        outer_closed = np.vstack([outer_world, outer_world[0]])
        perimeter = np.sum(np.linalg.norm(np.diff(outer_closed, axis=0), axis=1))
        # Target ~0.3-0.5m spacing
        auto_pts = int(perimeter / 0.4)
        auto_pts = max(100, min(auto_pts, args.centerline_points))
        num_pts = auto_pts if args.centerline_points == 1000 else args.centerline_points
        print(f"  Estimated perimeter: {perimeter:.1f} m → using {num_pts} centerline points")

        if is_slam_map:
            # SLAM map: thinned-skeleton + graph + longest-loop approach.
            # Correctly routes through ALL corridor sections (U-notches, bays)
            # by finding the longest simple loop in the skeleton graph.
            centerline, _ = compute_centerline_thinned_loop(
                map_img, resolution, origin,
                wall_thresh=wall_thresh,
                num_points=num_pts,
            )
            inner_tree = outer_tree = None   # not used downstream
        else:
            # Designed map: use midpoint-between-boundaries approach (robust for
            # simple oval or closed-loop tracks with well-defined inner contours)
            centerline, inner_tree, outer_tree = compute_centerline(
                outer_world, inner_world,
                num_points=num_pts,
                map_img=map_img,
                map_resolution=resolution,
                map_origin=origin,
                wall_thresh=wall_thresh,
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

        # Measure track widths via ray-cast
        print("  Measuring track widths (ray-cast)...")
        w_right, w_left = measure_track_widths(
            centerline, map_img, resolution, origin,
            max_dist=args.max_ray_distance,
            wall_thresh=wall_thresh,
        )

        # Smooth and cap widths to prevent spline normal crossings.
        # Use car half-width as the absolute minimum per side (car must physically fit).
        # Do NOT inflate to width_opt/2 — instead let the optimizer's width_opt
        # constraint naturally push the raceline away from narrow walls.
        # This is safe because total_width >= width_opt at every point (checked below).
        car_half = args.car_width / 2.0
        w_right, w_left = smooth_track_widths(
            centerline, w_right, w_left, min_width=car_half
        )

        # Verify the optimizer can find a feasible solution
        total_w = np.array(w_right) + np.array(w_left)
        min_total = total_w.min()
        if min_total < args.min_track_width:
            print(f"  WARNING: Narrowest total width ({min_total:.3f}m) < "
                  f"min_track_width ({args.min_track_width:.3f}m)")
            print(f"  Optimizer may fail at these points. Consider reducing --wall-clearance.")
        else:
            print(f"  Track feasibility OK: narrowest total={min_total:.3f}m >= "
                  f"min_track_width={args.min_track_width:.3f}m")

        # Save TUM-format track CSV
        save_tum_track_csv(centerline, w_right, w_left, track_csv)
    else:
        print(f"\n  Skipping extraction (--skip-extract)")
        if not os.path.exists(track_csv):
            available = [
                f.replace('.csv', '')
                for f in os.listdir(os.path.join(global_opt_dir, 'inputs', 'tracks'))
                if f.endswith('.csv')
            ]
            print(
                f"  ERROR: Track CSV not found: {track_csv}\n"
                f"  Available tracks: {', '.join(sorted(available))}"
            )
            sys.exit(1)
        print(f"  Using existing track CSV: {track_csv}")

    # ---- Step 1: Run TUM global_racetrajectory_optimization -----------------
    tum_output = os.path.join(global_opt_dir, 'outputs', 'traj_race_cl.csv')

    if not args.skip_optimize:
        main_py = os.path.join(global_opt_dir, 'main_globaltraj.py')
        racecar_ini = os.path.join(global_opt_dir, 'params', 'racecar.ini')

        # Save original content for restoration
        with open(main_py, 'r') as f:
            original_content = f.read()
        with open(racecar_ini, 'r') as f:
            original_ini_content = f.read()

        try:
            set_track_in_main(main_py, track_name)
            set_opt_type_in_main(main_py, args.opt_type)

            # Patch width_opt in racecar.ini: car_width + 2*wall_clearance
            # This ensures the optimizer keeps the raceline far enough
            # from track boundaries that after subtracting car_half_width
            # in wall distance computation, there is wall_clearance of
            # driveable room for the MPC on each side.
            optimizer_width = args.car_width + 2.0 * args.wall_clearance
            patched_ini = re.sub(
                r'(optim_opts_mincurv\s*=\s*\{"width_opt":\s*)[\d.]+',
                rf'\g<1>{optimizer_width:.3f}',
                original_ini_content,
            )

            # Patch s_reg only if explicitly requested via --smooth-factor.
            # Otherwise leave the racecar.ini default (25) which is robust.
            if args.smooth_factor is not None:
                s_reg_val = float(args.smooth_factor)
                patched_ini = re.sub(
                    r'("s_reg":\s*)[\d.]+',
                    rf'\g<1>{s_reg_val:.1f}',
                    patched_ini,
                )
                print(f"  Patched racecar.ini: s_reg -> {s_reg_val:.1f} (--smooth-factor)")

            print(f"  Patched racecar.ini: width_opt -> {optimizer_width:.3f} "
                  f"(car_width={args.car_width:.2f} + 2*clearance={args.wall_clearance:.2f})")

            # Patch stepsize_interp_after_opt to produce denser waypoints
            patched_ini = re.sub(
                r'("stepsize_interp_after_opt":\s*)[\d.]+',
                rf'\g<1>{args.waypoint_spacing:.3f}',
                patched_ini,
            )
            print(f"  Patched racecar.ini: stepsize_interp_after_opt -> {args.waypoint_spacing:.3f}m")
            # Write all ini patches at once
            with open(racecar_ini, 'w') as f:
                f.write(patched_ini)

            # Patch min_track_width in main_globaltraj.py
            # This widens narrow track sections so the optimizer can find a valid solution.
            patched_main = original_content
            with open(main_py, 'r') as f:
                patched_main = f.read()
            patched_main = re.sub(
                r'("min_track_width":\s*)(None|[\d.]+)',
                rf'\g<1>{args.min_track_width:.3f}',
                patched_main,
            )
            with open(main_py, 'w') as f:
                f.write(patched_main)
            print(f"  Patched main_globaltraj.py: min_track_width -> {args.min_track_width:.3f}")

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
    else:
        print(f"\n  Skipping optimization (--skip-optimize)")

    if not os.path.exists(tum_output):
        print(f"  ERROR: TUM output not found: {tum_output}")
        sys.exit(1)

    # ---- Step 2: Convert to MPC format --------------------------------------
    print(f"\n{'=' * 64}")
    print(f"  Step 2: Convert to MPC format (psi += pi/2, ';' -> ',')")
    print(f"{'=' * 64}")

    os.makedirs(os.path.dirname(output_csv), exist_ok=True)

    # Intermediate 7-column CSV
    intermediate_csv = output_csv + '.7col'
    n_waypoints = convert_tum_to_mpc(
        tum_output, intermediate_csv, max_speed=args.max_speed
    )

    # ---- Step 3: Add wall distances (optional) ------------------------------
    if not args.skip_walls:
        wall_script = os.path.join(scripts_dir, 'compute_wall_distances.py')
        if not os.path.exists(wall_script):
            print(f"  WARNING: Wall script not found: {wall_script}")
            print(f"  Falling back to 7-column output")
            os.replace(intermediate_csv, output_csv)
        else:
            wall_cmd = [
                sys.executable, wall_script,
                '--map', args.map,
                '--trajectory', intermediate_csv,
                '--output', output_csv,
                '--max-distance', str(args.max_ray_distance),
                '--car-width', str(args.car_width),
            ]
            run_step("Step 3: Compute ray-cast wall distances", wall_cmd)

            # Clean up intermediate file
            if os.path.exists(intermediate_csv):
                os.remove(intermediate_csv)
    else:
        os.replace(intermediate_csv, output_csv)
        print(f"\n  Skipped wall computation (7-column output)")

    # ---- Step 4: Verify ------------------------------------------------------
    print(f"\n{'=' * 64}")
    print(f"  Step 4: Verify output")
    print(f"{'=' * 64}")
    ok = verify_output(output_csv)

    # ---- Step 5: Fix wall clearance (optional) -------------------------------
    if args.fix_clearance and not args.skip_walls:
        fix_script = os.path.join(scripts_dir, 'fix_raceline_clearance.py')
        if not os.path.exists(fix_script):
            print(f"\n  WARNING: Fix script not found: {fix_script}")
            print(f"  Skipping wall clearance fix.")
        else:
            fixed_tmp = output_csv + '.fixed'
            fix_cmd = [
                sys.executable, fix_script,
                '--input', output_csv,
                '--output', fixed_tmp,
                '--map', args.map,
                '--min-clearance', str(args.min_clearance),
                '--car-width', str(args.car_width),
                '--iterations', str(args.fix_iterations),
            ]
            run_step("Step 5: Fix wall clearance", fix_cmd)

            if os.path.exists(fixed_tmp):
                os.replace(fixed_tmp, output_csv)
                print(f"  Clearance-fixed raceline saved to {output_csv}")
            else:
                print(f"  WARNING: Fix script did not produce output: {fixed_tmp}")

            # ---- Step 6: Recompute wall distances on fixed raceline ----------
            wall_script = os.path.join(scripts_dir, 'compute_wall_distances.py')
            if not os.path.exists(wall_script):
                print(f"\n  WARNING: Wall script not found: {wall_script}")
                print(f"  Skipping wall distance recomputation.")
            else:
                # Strip to 7-col (s,x,y,psi,kappa,vx,ax) for recomputation
                stripped_tmp = output_csv + '.7col'
                with open(output_csv, 'r') as f_in, open(stripped_tmp, 'w') as f_out:
                    for line in f_in:
                        line = line.strip()
                        if not line or line.startswith('#'):
                            f_out.write(line + '\n')
                            continue
                        fields = line.split(',')
                        if len(fields) >= 7:
                            f_out.write(','.join(fields[:7]) + '\n')
                        else:
                            f_out.write(line + '\n')

                recompute_cmd = [
                    sys.executable, wall_script,
                    '--map', args.map,
                    '--trajectory', stripped_tmp,
                    '--output', output_csv,
                    '--max-distance', str(args.max_ray_distance),
                    '--car-width', str(args.car_width),
                ]
                run_step("Step 6: Recompute wall distances on fixed raceline", recompute_cmd)

                # Clean up temp file
                if os.path.exists(stripped_tmp):
                    os.remove(stripped_tmp)

                # Verify again after fix
                print(f"\n{'=' * 64}")
                print(f"  Step 6b: Verify fixed output")
                print(f"{'=' * 64}")
                ok = verify_output(output_csv)

    # ---- Step 7: Smooth kappa column ----------------------------------------
    # The clearance-fix step can create artificial kappa spikes at shifted
    # waypoints.  Apply a short moving-average to the kappa column so the MPC
    # feedforward steering doesn't chatter.  Positions, headings, and velocities
    # are untouched.
    if not args.skip_walls and os.path.exists(output_csv):
        from scipy.ndimage import uniform_filter1d as _uf1d
        lines_in = open(output_csv).readlines()
        header_lines, data_lines = [], []
        for ln in lines_in:
            (header_lines if ln.strip().startswith('#') else data_lines).append(ln)

        rows_kappa = [[float(v) for v in ln.strip().split(',')] for ln in data_lines if ln.strip()]
        if rows_kappa:
            kappa = np.array([r[4] for r in rows_kappa])
            kappa_smooth = _uf1d(kappa, size=5, mode='wrap')
            for i, r in enumerate(rows_kappa):
                r[4] = kappa_smooth[i]
            with open(output_csv, 'w') as fout:
                fout.writelines(header_lines)
                for r in rows_kappa:
                    fout.write(','.join(f'{v:.7f}' for v in r) + '\n')
            print(f"\n  Step 7: Smoothed kappa (window=5); "
                  f"range [{kappa_smooth.min():.4f}, {kappa_smooth.max():.4f}] rad/m")

    # ---- Step 8: Recompute velocity profile based on kappa ------------------
    # The clearance-fix and kappa smoothing may change the curvature from the
    # original TUM output.  Recompute vx and ax with a forward-backward pass
    # so that lateral friction limits are respected at every waypoint.
    if not args.skip_walls and os.path.exists(output_csv):
        print(f"\n  Step 8: Recompute velocity profile (ay_max=7.3 m/s², ax_max=7.3 m/s²)")
        recompute_velocity_profile(
            output_csv,
            max_speed=args.max_speed,
            ay_max=7.3,
            ax_max=7.3,
            ax_min=-7.3,
        )

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
