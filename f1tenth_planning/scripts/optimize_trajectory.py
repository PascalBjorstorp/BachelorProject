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
    else:
        # SLAM map -------------------------------------------------------------
        outer_idx = areas[0][0]
        # Inner wall = largest child of the outer contour
        children = [
            (idx, a) for idx, a in areas[1:]
            if hierarchy[0][idx][3] == outer_idx
        ]
        if not children:
            raise RuntimeError(
                "Could not find inner boundary -- is this a closed track?"
            )
        inner_idx = max(children, key=lambda x: x[1])[0]
        print("  Map type: SLAM (floating track)")

    def pixel_to_world(points):
        world = np.zeros((len(points), 2))
        world[:, 0] = points[:, 0] * resolution + origin[0]
        world[:, 1] = (img.shape[0] - points[:, 1]) * resolution + origin[1]
        return world

    outer_world = pixel_to_world(contours[outer_idx].reshape(-1, 2))
    inner_world = pixel_to_world(contours[inner_idx].reshape(-1, 2))

    print(f"  Outer boundary: {len(outer_world)} points")
    print(f"  Inner boundary: {len(inner_world)} points")

    return outer_world, inner_world


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

    # Smooth with a periodic B-spline (higher smoothing for fewer points / noisier data)
    centerline_closed = np.vstack([centerline_raw, centerline_raw[0]])
    tck, _u = splprep(
        [centerline_closed[:, 0], centerline_closed[:, 1]],
        s=len(centerline_raw) * 0.1,       # was 0.01; smoother for TUM input
        per=True,
    )
    u_new = np.linspace(0, 1, num_points, endpoint=False)
    centerline = np.array(splev(u_new, tck)).T

    # Collision correction via distance transform
    if map_img is not None:
        # Use consistent wall threshold (see extract_boundaries docstring)
        free = (map_img >= wall_thresh).astype(np.uint8)
        dist_transform = cv2.distanceTransform(free, cv2.DIST_L2, 5)
        min_clearance_px = 2.0
        n_fixed = 0
        for i in range(len(centerline)):
            col = int((centerline[i, 0] - map_origin[0]) / map_resolution)
            row = int(map_img.shape[0] - (centerline[i, 1] - map_origin[1]) / map_resolution)
            if (0 <= row < map_img.shape[0]
                    and 0 <= col < map_img.shape[1]
                    and dist_transform[row, col] >= min_clearance_px):
                continue
            # Search for nearest free pixel with enough clearance
            best_r, best_c = row, col
            best_sq = float('inf')
            found = False
            for dr in range(-30, 31):
                for dc in range(-30, 31):
                    nr, nc = row + dr, col + dc
                    sq = dr * dr + dc * dc
                    if sq >= best_sq:
                        continue
                    if (0 <= nr < map_img.shape[0]
                            and 0 <= nc < map_img.shape[1]
                            and dist_transform[nr, nc] >= min_clearance_px):
                        best_sq = sq
                        best_r, best_c = nr, nc
                        found = True
            if found:
                centerline[i, 0] = best_c * map_resolution + map_origin[0]
                centerline[i, 1] = (map_img.shape[0] - best_r) * map_resolution + map_origin[1]
                n_fixed += 1
        if n_fixed > 0:
            print(f"  Collision correction: fixed {n_fixed}/{len(centerline)} points")

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
    from scipy.ndimage import uniform_filter1d

    n = len(centerline)

    # Compute curvature of the centerline
    dx = np.gradient(centerline[:, 0])
    dy = np.gradient(centerline[:, 1])
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    kappa = (dx * ddy - dy * ddx) / np.maximum((dx ** 2 + dy ** 2) ** 1.5, 1e-10)
    R = 1.0 / np.maximum(np.abs(kappa), 1e-6)

    # Cap widths to 80% of curvature radius (prevents normal crossings)
    cap = 0.8 * R
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

    # Check heading vs atan2(dy, dx)
    dx = np.gradient(w[:, 1])
    dy = np.gradient(w[:, 2])
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

        outer_world, inner_world = extract_boundaries(map_img, resolution, origin, wall_thresh=wall_thresh)

        # Auto-scale number of centerline points based on track perimeter
        # Estimate perimeter from outer boundary
        outer_closed = np.vstack([outer_world, outer_world[0]])
        perimeter = np.sum(np.linalg.norm(np.diff(outer_closed, axis=0), axis=1))
        # Target ~0.3-0.5m spacing
        auto_pts = int(perimeter / 0.4)
        auto_pts = max(100, min(auto_pts, args.centerline_points))
        num_pts = auto_pts if args.centerline_points == 1000 else args.centerline_points
        print(f"  Estimated perimeter: {perimeter:.1f} m → using {num_pts} centerline points")

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
            with open(racecar_ini, 'w') as f:
                f.write(patched_ini)
            print(f"  Patched racecar.ini: width_opt -> {optimizer_width:.3f} "
                  f"(car_width={args.car_width:.2f} + 2*clearance={args.wall_clearance:.2f})")

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
                '--iterations', '5',
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
