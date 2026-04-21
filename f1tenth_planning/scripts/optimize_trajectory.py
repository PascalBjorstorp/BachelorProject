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
    python3 optimize_trajectory.py --map f1tenth_planning/maps/Spielberg_map.yaml
    python3 optimize_trajectory.py --map f1tenth_planning/maps/my_track_map.yaml --max-speed 8.0

    # With wall distances (enables 9-col output)
    python3 optimize_trajectory.py --map f1tenth_planning/maps/my_track_map.yaml --with-walls

    # Skip extraction if TUM track CSV already exists
    python3 optimize_trajectory.py --map f1tenth_planning/maps/Spielberg_map.yaml --skip-extract

    # Skip both extraction and optimization (re-convert only)
    python3 optimize_trajectory.py --map f1tenth_planning/maps/Spielberg_map.yaml --skip-extract --skip-optimize

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
    result = subprocess.run(cmd, cwd=cwd, env=env)
    if result.returncode != 0:
        print(f"\n  ERROR: {label} failed (exit code {result.returncode})")
        sys.exit(result.returncode)


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
        tck, _ = splprep([closed[:, 0], closed[:, 1]],
                         s=0, per=True)
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

    # Cap widths to the distance-transform value at each point.
    # This prevents inflated widths from rays escaping through wall gaps
    # into exterior free space.  After ridge-snapping the centerline to
    # the white-space centre, the cap should only trim outlier rays.
    dist_transform = cv2.distanceTransform(free, cv2.DIST_L2, 5)
    n_capped = 0
    for i in range(n):
        col = int((centerline[i, 0] - origin[0]) / resolution)
        row = int(map_img.shape[0] - (centerline[i, 1] - origin[1]) / resolution)
        if 0 <= row < map_img.shape[0] and 0 <= col < map_img.shape[1]:
            dt_val = dist_transform[row, col] * resolution
            # Cap ray-cast width to the DT value (distance to nearest
            # non-free pixel).  This prevents rays from escaping through
            # wall gaps into the exterior free space.
            cap = dt_val * 1.0
            if w_right[i] > cap:
                w_right[i] = cap
                n_capped += 1
            if w_left[i] > cap:
                w_left[i] = cap
                n_capped += 1
    if n_capped > 0:
        print(f"  Width DT-capped: {n_capped} sides")

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

    # Cap widths to 90% of curvature radius (prevents normal crossings)
    cap = 0.9 * R
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

    # ---- Argument parsing ----------------------------------------------------
    parser = argparse.ArgumentParser(
        description='Universal one-click trajectory optimization for F1Tenth MPC',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )

    parser.add_argument(
        '--map', '-m', default=None,
        help='Path to map .yaml file (default: auto-detect my_track_map.yaml in f1tenth_planning/maps/)',
    )
    parser.add_argument(
        '--track-name', '-t', default=None,
        help='Track name (default: derived from map filename)',
    )
    parser.add_argument(
        '--opt-type',
        choices=['shortest_path', 'mincurv', 'mincurv_iqp', 'mintime'],
        default='mintime',
        help='Optimization type (default: mincurv)',
    )
    parser.add_argument(
        '--max-speed', type=float, default=12.0,
        help='Clamp velocity to this value [m/s] (default: 12.0)',
    )
    parser.add_argument(
        '--min-speed', type=float, default=2.0,
        help='Minimum velocity [m/s] (default: 2.0). DC motors struggle below this.',
    )
    parser.add_argument(
        '--output', '-o', default=default_output,
        help='Output directory (default: f1tenth_planning/trajectories/)',
    )
    parser.add_argument(
        '--centerline-points', type=int, default=2000,
        help='Number of centerline points to sample (default: 300)',
    )
    parser.add_argument(
        '--skip-extract', action='store_true',
        help='Skip centerline extraction (use existing TUM track CSV)',
    )
    parser.add_argument(
        '--car-width', type=float, default=0.3,
        help='Physical car width [m] used for wall distance margin (default: 0.273, F1Tenth)',
    )
    parser.add_argument(
        '--wall-clearance', type=float, default=0.05,
        help='Extra clearance from walls beyond car width on each side [m] (default: 0.05). '
             'Optimizer width_opt = car_width + 2*wall_clearance',
    )
    parser.add_argument(
        '--max-ray-distance', type=float, default=2.0,
        help='Max wall ray-cast distance [m] (default: 5.0)',
    )
    parser.add_argument(
        '--min-track-width', type=float, default=None,
        help='Minimum enforced track width [m] in TUM optimizer (default: auto = 2*wall_clearance + car_width). '
             'Narrow track sections are widened to this value before optimization.',
    )
    parser.add_argument(
        '--direction', default='cw',
        choices=['auto', 'cw', 'ccw'],
        help='Track direction: auto-detect from winding order, or force cw/ccw (default: cw)',
    )
    parser.add_argument(
        '--smooth-factor', type=float, default=8.0,
        help='Spline smoothing factor s_reg for TUM optimizer (default: 2.0). '
             'Lower values preserve centerline shape better; higher values smooth more. '
             '0 = exact interpolation (safest but slowest), 2 = good balance for small tracks.',
    )
    parser.add_argument(
        '--waypoint-spacing', type=float, default=0.05,
        help='Waypoint spacing [m] for the final trajectory (stepsize_interp_after_opt, default: 0.15). '
             'Smaller values produce denser waypoints (recommended for MPC).',
    )
    args = parser.parse_args()
    args.skip_walls = False

    # Auto-detect map if not specified
    if args.map is None:
        default_map = os.path.join(workspace, 'f1tenth_planning', 'maps', 'my_track_map.yaml')
        if os.path.exists(default_map):
            args.map = default_map
        else:
            print("ERROR: No --map specified and default my_track_map.yaml not found.")
            print(f"  Looked at: {default_map}")
            sys.exit(1)

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
    gvd_debug_path = output_csv.replace('.csv', '_gvd.png')

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
    print(f"  Output:           {output_csv}")
    print(f"  Skip extract:     {args.skip_extract}")

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

        # SLAM maps have grey (typically ~205) for unknown/unexplored areas.
        # These must be treated as walls, not free space.
        unique_vals, counts = np.unique(map_img, return_counts=True)
        grey_mask = (unique_vals > wall_thresh) & (unique_vals < 240)
        grey_pct = counts[grey_mask].sum() / map_img.size
        if grey_pct > 0.10:
            # Find the lowest "white" (free space) pixel value
            white_mask = unique_vals >= 240
            if white_mask.any():
                white_min = unique_vals[white_mask].min()
                wall_thresh = int(white_min) - 5
                print(f"  SLAM grey detected ({grey_pct:.0%}): raised wall_thresh to {wall_thresh} "
                      f"(only pixels >= {wall_thresh} are free)")

        print(f"  Image size: {map_img.shape[1]}x{map_img.shape[0]} px, "
              f"resolution: {resolution} m/px")

        outer_world = extract_outer_boundary(map_img, resolution, origin, wall_thresh=wall_thresh)

        # Auto-scale number of centerline points based on track perimeter
        outer_closed = np.vstack([outer_world, outer_world[0]])
        perimeter = np.sum(np.linalg.norm(np.diff(outer_closed, axis=0), axis=1))
        auto_pts = max(100, min(int(perimeter / 0.4), args.centerline_points))
        num_pts = auto_pts if args.centerline_points == 1000 else args.centerline_points
        print(f"  Estimated perimeter: {perimeter:.1f} m → using {num_pts} centerline points")

        # GVD centerline: label wall obstacles, find equidistant boundary,
        # thin it, walk the ring, B-spline smooth.
        centerline, _ = compute_centerline_thinned_loop(
            map_img, resolution, origin,
            wall_thresh=wall_thresh,
            num_points=num_pts,
            gvd_debug_path=gvd_debug_path,
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

        # Save TUM-format track CSV
        # Remove duplicate/near-duplicate points and resample uniformly
        ds = np.sqrt(np.sum(np.diff(centerline, axis=0)**2, axis=1))
        keep = np.concatenate([[True], ds > 0.01])  # drop points < 1cm apart
        if not np.all(keep):
            n_dup = np.sum(~keep)
            centerline = centerline[keep]
            w_right = np.array(w_right)[keep]
            w_left = np.array(w_left)[keep]
            print(f"  Removed {n_dup} near-duplicate centerline points")
            # Resample to uniform spacing via spline
            cl_closed = np.vstack([centerline, centerline[0]])
            tck_resamp, _ = splprep(
                [cl_closed[:, 0], cl_closed[:, 1]],
                s=len(centerline) * 0.1, per=True,
            )
            u_new = np.linspace(0, 1, len(centerline), endpoint=False)
            centerline = np.array(splev(u_new, tck_resamp)).T
            # Re-measure widths at new positions
            w_right, w_left = measure_track_widths(
                centerline, map_img, resolution, origin,
                max_dist=args.max_ray_distance,
                wall_thresh=wall_thresh,
            )
            w_right, w_left = smooth_track_widths(
                centerline, w_right, w_left, min_width=car_half
            )

        # Verify the optimizer can find a feasible solution (after resampling)
        total_w = np.array(w_right) + np.array(w_left)
        min_total = total_w.min()
        if min_total < args.min_track_width:
            print(f"  WARNING: Narrowest total width ({min_total:.3f}m) < "
                  f"min_track_width ({args.min_track_width:.3f}m)")
            # Auto-reduce min_track_width and optimizer width to avoid infeasible QP
            actual_max_width = float(min_total) - 0.01
            args.min_track_width = actual_max_width
            # Recompute wall clearance from available space
            effective_clearance = (actual_max_width - args.car_width) / 2.0
            args.wall_clearance = max(0.05, effective_clearance)
            print(f"  Auto-reduced min_track_width to {args.min_track_width:.3f}m, "
                  f"wall_clearance to {args.wall_clearance:.3f}m")
        else:
            print(f"  Track feasibility OK: narrowest total={min_total:.3f}m >= "
                  f"min_track_width={args.min_track_width:.3f}m")

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
        # Ensure width_opt doesn't exceed minimum track width (infeasible QP)
        track_data_check = np.loadtxt(track_csv, delimiter=',', comments='#')
        min_total_w = (track_data_check[:, 2] + track_data_check[:, 3]).min()
        if optimizer_width > min_total_w:
            # Leave 5% margin for the TUM's internal spline resampling
            optimizer_width = max(args.car_width * 0.95, min_total_w * 0.90)
            print(f"  Clamped width_opt to {optimizer_width:.3f}m "
                  f"(min track width = {min_total_w:.3f}m)")
        patched_ini = re.sub(
            r'(optim_opts_mincurv\s*=\s*\{"width_opt":\s*)[\d.]+',
            rf'\g<1>{optimizer_width:.3f}',
            original_ini_content,
        )

        # Patch s_reg (spline smoothing factor)
        s_reg_val = float(args.smooth_factor)
        patched_ini = re.sub(
            r'("s_reg":\s*)[\d.]+',
            rf'\g<1>{s_reg_val:.1f}',
            patched_ini,
        )
        print(f"  Patched racecar.ini: s_reg -> {s_reg_val:.1f}")

        print(f"  Patched racecar.ini: width_opt -> {optimizer_width:.3f} "
              f"(car_width={args.car_width:.2f} + 2*clearance={args.wall_clearance:.2f})")

        # Auto-reduce stepsize_prep and stepsize_reg for small tracks
        # (TUM defaults are 0.3m and 0.5m, designed for 100m+ tracks)
        patched_ini = re.sub(
            r'("stepsize_prep":\s*)[\d.]+',
            rf'\g<1>0.150',
            patched_ini,
        )
        patched_ini = re.sub(
            r'("stepsize_reg":\s*)[\d.]+',
            rf'\g<1>0.200',
            patched_ini,
        )
        print(f"  Patched racecar.ini: stepsize_prep -> 0.150m, stepsize_reg -> 0.200m")

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
        tum_output, intermediate_csv,
        max_speed=args.max_speed, min_speed=args.min_speed
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
