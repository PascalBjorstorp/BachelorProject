#!/usr/bin/env python3
"""
Post-process a raceline CSV to enforce minimum wall clearance.

For waypoints where d_left or d_right is below min_clearance,
shifts the waypoint toward the opposite wall. Recomputes heading
and curvature after the shift. Also re-raycasts wall distances
to verify the result.

Usage:
    python3 fix_raceline_clearance.py [--min-clearance 0.45] [--input FILE] [--output FILE]
"""

import argparse
import csv
import math
import os
import sys

import cv2
import numpy as np
import yaml


def load_raceline(path):
    """Load raceline CSV, return header + list of dicts."""
    rows = []
    with open(path) as f:
        reader = csv.reader(f)
        header = next(reader)
        header = [h.strip().lstrip('#').strip() for h in header]
        for line in reader:
            if not line or line[0].startswith('#'):
                continue
            r = {}
            for i, h in enumerate(header):
                r[h] = float(line[i])
            rows.append(r)
    return header, rows


def save_raceline(path, header, rows):
    """Save raceline CSV."""
    with open(path, 'w', newline='') as f:
        f.write('# ' + ','.join(header) + '\n')
        for r in rows:
            vals = [f"{r[h]:.6f}" for h in header]
            f.write(','.join(vals) + '\n')


def load_map(yaml_path):
    """Load map image and metadata."""
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)

    map_dir = os.path.dirname(yaml_path)
    img_path = os.path.join(map_dir, meta['image'])
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)

    resolution = meta['resolution']
    origin = meta['origin']  # [x, y, theta]
    # Bug fix: use occupied_thresh (not free_thresh) to match compute_wall_distances.py
    occupied_thresh = meta.get('occupied_thresh', 0.45)

    # Create wall mask (occupied pixels)
    # Pixels with value <= 255*(1 - occupied_thresh) are considered walls
    occ_thresh = int(255 * (1.0 - occupied_thresh))
    wall_mask = (img <= occ_thresh).astype(np.uint8)

    return wall_mask, resolution, origin


def world_to_pixel(x, y, resolution, origin):
    """Convert world coordinates to pixel coordinates."""
    px = int((x - origin[0]) / resolution)
    py = int((y - origin[1]) / resolution)
    # Flip y (image y is inverted)
    return px, py


def raycast_distance(wall_mask, x, y, angle, resolution, origin, max_dist=5.0):
    """Cast a ray from (x,y) in direction angle, return distance to first wall."""
    h, w = wall_mask.shape
    step = resolution * 0.5  # half-pixel steps
    dist = 0.0
    dx = math.cos(angle) * step
    dy = math.sin(angle) * step
    cx, cy = x, y

    while dist < max_dist:
        cx += dx
        cy += dy
        dist += step
        px, py = world_to_pixel(cx, cy, resolution, origin)
        # Flip y for image coordinates
        img_y = h - 1 - py
        if img_y < 0 or img_y >= h or px < 0 or px >= w:
            return dist  # Out of bounds = far wall
        if wall_mask[img_y, px]:
            return dist

    return max_dist


def recompute_wall_distances(rows, wall_mask, resolution, origin, car_half_width=0.15):
    """Recompute d_left and d_right for all waypoints via ray-cast.

    Stores RAW center-to-wall distances (NOT car-half-subtracted), matching
    the convention used by compute_wall_distances.py and expected by the MPC.
    """
    for r in rows:
        x, y, psi = r['x_m'], r['y_m'], r['psi_rad']
        # Left = perpendicular left of heading
        left_angle = psi + math.pi / 2
        right_angle = psi - math.pi / 2
        d_left = raycast_distance(wall_mask, x, y, left_angle, resolution, origin)
        d_right = raycast_distance(wall_mask, x, y, right_angle, resolution, origin)
        r['d_left_m'] = d_left
        r['d_right_m'] = d_right


def recompute_heading_and_curvature(rows):
    """Recompute s_m, psi and kappa from x,y positions (closed track)."""
    n = len(rows)

    # 1. Recompute arc length from actual inter-point distances
    rows[0]['s_m'] = 0.0
    for i in range(1, n):
        dx = rows[i]['x_m'] - rows[i - 1]['x_m']
        dy = rows[i]['y_m'] - rows[i - 1]['y_m']
        rows[i]['s_m'] = rows[i - 1]['s_m'] + math.sqrt(dx * dx + dy * dy)

    # 2. Recompute heading using central differences (closed track)
    for i in range(n):
        i_prev = (i - 1) % n
        i_next = (i + 1) % n
        dx = rows[i_next]['x_m'] - rows[i_prev]['x_m']
        dy = rows[i_next]['y_m'] - rows[i_prev]['y_m']
        rows[i]['psi_rad'] = math.atan2(dy, dx)

    # 3. Recompute curvature from heading change / arc length
    for i in range(n):
        i_prev = (i - 1) % n
        i_next = (i + 1) % n
        ds_prev = rows[i]['s_m'] - rows[i_prev]['s_m']
        ds_next = rows[i_next]['s_m'] - rows[i]['s_m']
        # Handle wrap-around: use closing segment length
        if ds_prev <= 0:
            dx = rows[0]['x_m'] - rows[n - 1]['x_m']
            dy = rows[0]['y_m'] - rows[n - 1]['y_m']
            ds_prev = math.sqrt(dx * dx + dy * dy)
        if ds_next <= 0:
            dx = rows[0]['x_m'] - rows[n - 1]['x_m']
            dy = rows[0]['y_m'] - rows[n - 1]['y_m']
            ds_next = math.sqrt(dx * dx + dy * dy)
        ds_total = ds_prev + ds_next
        if ds_total > 0:
            dpsi = rows[i_next]['psi_rad'] - rows[i_prev]['psi_rad']
            # Wrap to [-pi, pi]
            dpsi = (dpsi + math.pi) % (2 * math.pi) - math.pi
            rows[i]['kappa_radpm'] = dpsi / ds_total


def main():
    parser = argparse.ArgumentParser(description="Fix raceline wall clearance")
    parser.add_argument('--input', default=None,
                        help='Input raceline CSV (default: Spielberg_raceline.csv)')
    parser.add_argument('--output', default=None,
                        help='Output raceline CSV (default: overwrite input)')
    parser.add_argument('--map', default=None,
                        help='Map YAML file for ray-casting')
    parser.add_argument('--min-clearance', type=float, default=0.45,
                        help='Minimum wall clearance in meters (default: 0.45)')
    parser.add_argument('--max-shift', type=float, default=0.40,
                        help='Maximum shift per waypoint in meters (default: 0.40)')
    parser.add_argument('--smooth-radius', type=int, default=5,
                        help='Smoothing radius for shifted points (default: 5)')
    parser.add_argument('--car-width', type=float, default=0.30,
                        help='Car width in meters for half-width subtraction (default: 0.30)')
    parser.add_argument('--iterations', type=int, default=3,
                        help='Number of shift+smooth iterations (default: 3)')
    parser.add_argument('--dry-run', action='store_true',
                        help='Just show what would change, don\'t modify')
    args = parser.parse_args()

    # Default paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    workspace = os.path.dirname(os.path.dirname(script_dir))

    if args.input is None:
        args.input = os.path.join(workspace, 'f1tenth_planning', 'trajectories', 'Spielberg_raceline.csv')
    if args.output is None:
        args.output = args.input
    if args.map is None:
        args.map = os.path.join(workspace, 'f1tenth_sim', 'maps', 'Spielberg_map.yaml')

    car_half_width = args.car_width / 2.0

    print(f"Input:         {args.input}")
    print(f"Output:        {args.output}")
    print(f"Map:           {args.map}")
    print(f"Min clearance: {args.min_clearance}m")
    print(f"Max shift:     {args.max_shift}m")
    print(f"Car width:     {args.car_width}m (half={car_half_width}m)")
    print()

    header, rows = load_raceline(args.input)
    wall_mask, resolution, origin = load_map(args.map)
    n = len(rows)

    print(f"Loaded {n} waypoints, map resolution={resolution}m/px")

    # Raw distances are center-to-wall; effective min = body clearance + car half
    eff_min = args.min_clearance + car_half_width
    print(f"Effective min center-to-wall distance: {eff_min:.3f}m "
          f"(body clearance {args.min_clearance}m + car half {car_half_width}m)")

    # --- Show current tight spots ---
    tight_right = [(i, r) for i, r in enumerate(rows) if r['d_right_m'] < eff_min]
    tight_left = [(i, r) for i, r in enumerate(rows) if r['d_left_m'] < eff_min]
    print(f"\nBEFORE: {len(tight_right)} right-tight, {len(tight_left)} left-tight waypoints")

    if tight_right:
        print("  Worst right-wall spots:")
        for i, r in sorted(tight_right, key=lambda x: x[1]['d_right_m'])[:10]:
            print(f"    wp={i:4d}  d_right={r['d_right_m']:.3f}m  d_left={r['d_left_m']:.3f}m  v={r['vx_mps']:.1f}")

    if tight_left:
        print("  Worst left-wall spots:")
        for i, r in sorted(tight_left, key=lambda x: x[1]['d_left_m'])[:10]:
            print(f"    wp={i:4d}  d_left={r['d_left_m']:.3f}m  d_right={r['d_right_m']:.3f}m  v={r['vx_mps']:.1f}")

    if args.dry_run:
        return

    # --- Cosine-profiled shift for smooth clearance enforcement ---
    # Instead of point shifts + smoothing, compute the required displacement at
    # each tight spot and spread it as a half-cosine bump over ±blend_radius
    # waypoints. This avoids the shift-smooth tug-of-war.

    blend_radius = args.smooth_radius * 3  # Spread over wider area for smoothness
    if blend_radius < 8:
        blend_radius = 8

    for iteration in range(args.iterations):
        shift_count = 0
        total_shift = 0.0

        # Collect all tight spots and their needed shifts
        tight_spots = []
        recenter_count = 0
        for i in range(n):
            r = rows[i]
            need_shift = 0.0
            shift_dir = 0.0
            d_total = r['d_left_m'] + r['d_right_m']

            if r['d_right_m'] < eff_min or r['d_left_m'] < eff_min:
                if d_total < 2 * eff_min:
                    # Physically too tight to satisfy eff_min on both sides.
                    # Best action: re-center the waypoint (move to midpoint).
                    # Compute shift toward midpoint (d_left == d_right == d_total/2).
                    d_mid = d_total / 2.0
                    if r['d_right_m'] < r['d_left_m']:
                        # Right wall closer → push LEFT toward midpoint
                        need_shift = max(0, d_mid - r['d_right_m'])
                        shift_dir = r['psi_rad'] + math.pi / 2  # LEFT
                    else:
                        # Left wall closer → push RIGHT toward midpoint
                        need_shift = max(0, d_mid - r['d_left_m'])
                        shift_dir = r['psi_rad'] - math.pi / 2  # RIGHT
                    if need_shift > 1e-4:
                        recenter_count += 1
                elif r['d_right_m'] < eff_min:
                    need_shift = eff_min - r['d_right_m'] + 0.02  # +2cm overshoot to survive smoothing
                    shift_dir = r['psi_rad'] + math.pi / 2  # LEFT
                else:
                    need_shift = eff_min - r['d_left_m'] + 0.02
                    shift_dir = r['psi_rad'] - math.pi / 2  # RIGHT

            if need_shift > 0:
                need_shift = min(need_shift, args.max_shift)
                tight_spots.append((i, need_shift, shift_dir))

        if recenter_count > 0:
            print(f"  Iteration {iteration+1}: {recenter_count} physically-tight pts re-centered (total width < 2×eff_min)")

        if not tight_spots:
            print(f"\n  Iteration {iteration+1}: no tight spots remaining")
            break

        # Apply cosine-profiled displacement bumps
        # Accumulate x,y offsets across all bumps, then apply once
        dx_acc = [0.0] * n
        dy_acc = [0.0] * n

        for (center_idx, peak_shift, shift_dir) in tight_spots:
            shift_count += 1
            total_shift += peak_shift
            cos_dir = math.cos(shift_dir)
            sin_dir = math.sin(shift_dir)

            for j in range(-blend_radius, blend_radius + 1):
                idx = (center_idx + j) % n
                # Half-cosine profile: 1.0 at center, 0.0 at ±blend_radius
                t = abs(j) / blend_radius
                weight = 0.5 * (1.0 + math.cos(math.pi * t))  # cosine bell
                dx_acc[idx] += cos_dir * peak_shift * weight
                dy_acc[idx] += sin_dir * peak_shift * weight

        # Bug fix: clamp accumulated shift magnitude to max_shift so
        # overlapping cosine bumps cannot push a waypoint further than allowed.
        for i in range(n):
            mag = math.sqrt(dx_acc[i]**2 + dy_acc[i]**2)
            if mag > args.max_shift:
                scale = args.max_shift / mag
                dx_acc[i] *= scale
                dy_acc[i] *= scale

        # Apply accumulated offsets (after clamping)
        for i in range(n):
            rows[i]['x_m'] += dx_acc[i]
            rows[i]['y_m'] += dy_acc[i]

        # Recompute wall distances after shift
        recompute_wall_distances(rows, wall_mask, resolution, origin, car_half_width)

        print(f"\n  Iteration {iteration+1}: {shift_count} tight spots, "
              f"avg shift={total_shift/max(shift_count,1):.3f}m")

    # --- Recompute heading and curvature (also recomputes s_m) ---
    recompute_heading_and_curvature(rows)

    # --- Final check ---
    tight_right_after = [(i, r) for i, r in enumerate(rows) if r['d_right_m'] < eff_min]
    tight_left_after = [(i, r) for i, r in enumerate(rows) if r['d_left_m'] < eff_min]

    print(f"\nAFTER: {len(tight_right_after)} right-tight, {len(tight_left_after)} left-tight waypoints")

    if tight_right_after:
        print("  Remaining right-tight:")
        for i, r in sorted(tight_right_after, key=lambda x: x[1]['d_right_m'])[:10]:
            print(f"    wp={i:4d}  d_right={r['d_right_m']:.3f}m  d_left={r['d_left_m']:.3f}m")

    if tight_left_after:
        print("  Remaining left-tight:")
        for i, r in sorted(tight_left_after, key=lambda x: x[1]['d_left_m'])[:10]:
            print(f"    wp={i:4d}  d_left={r['d_left_m']:.3f}m  d_right={r['d_right_m']:.3f}m")

    # Print overall stats
    d_rights = [r['d_right_m'] for r in rows]
    d_lefts = [r['d_left_m'] for r in rows]
    print(f"\n  Right wall: min={min(d_rights):.3f}m, avg={sum(d_rights)/n:.3f}m")
    print(f"  Left wall:  min={min(d_lefts):.3f}m, avg={sum(d_lefts)/n:.3f}m")
    print(f"  Track length: {rows[-1]['s_m']:.1f}m")
    print(f"  Velocity range: [{min(r['vx_mps'] for r in rows):.1f}, {max(r['vx_mps'] for r in rows):.1f}] m/s")

    # Save
    save_raceline(args.output, header, rows)
    print(f"\nSaved to: {args.output}")


if __name__ == '__main__':
    main()
