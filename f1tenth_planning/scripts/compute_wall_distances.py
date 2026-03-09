#!/usr/bin/env python3
"""
Compute wall distances for each waypoint in a trajectory CSV using the track map.

For each waypoint, casts rays perpendicular to the path direction (left and right)
to find the distance to the nearest wall. Outputs a 9-column trajectory CSV:
  s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2, d_left_m, d_right_m

Usage:
  python3 compute_wall_distances.py \
      --map ../f1tenth_sim/maps/Spielberg_map.yaml \
      --trajectory trajectories/Spielberg_raceline.csv \
      --output trajectories/Spielberg_raceline_walls.csv
"""

import argparse
import csv
import math
import os
import sys
import yaml
import numpy as np
from PIL import Image


def load_map(yaml_path):
    """Load ROS2 map from YAML + image file."""
    with open(yaml_path, 'r') as f:
        map_info = yaml.safe_load(f)

    # Load image
    map_dir = os.path.dirname(yaml_path)
    image_path = os.path.join(map_dir, map_info['image'])
    img = Image.open(image_path).convert('L')
    map_array = np.array(img)

    resolution = float(map_info['resolution'])
    origin = map_info['origin']  # [x, y, theta]
    origin_x = float(origin[0])
    origin_y = float(origin[1])
    negate = int(map_info.get('negate', 0))
    occupied_thresh = float(map_info.get('occupied_thresh', 0.65))

    # Build occupancy grid: True = wall (occupied)
    if negate == 0:
        # Standard: black = wall, white = free
        # ROS convention: p = (255 - pixel) / 255
        # occupied if p > occupied_thresh
        threshold = int(255 * (1.0 - occupied_thresh))
        is_wall = map_array < threshold  # pixel < threshold → wall
    else:
        # Inverted: white = wall, black = free
        threshold = int(255 * occupied_thresh)
        is_wall = map_array > threshold

    return is_wall, resolution, origin_x, origin_y


def world_to_pixel(wx, wy, origin_x, origin_y, resolution, map_height):
    """Convert world coordinates to pixel coordinates."""
    px = int((wx - origin_x) / resolution)
    py = map_height - 1 - int((wy - origin_y) / resolution)
    return px, py


def cast_ray(start_x, start_y, dir_x, dir_y, is_wall, origin_x, origin_y,
             resolution, map_height, map_width, max_distance=5.0, step_size=None):
    """
    Cast a ray from (start_x, start_y) in direction (dir_x, dir_y) and find
    the distance to the first wall pixel.
    
    Returns distance in meters, capped at max_distance.
    """
    if step_size is None:
        step_size = resolution * 0.5  # Half-pixel steps for accuracy

    distance = 0.0
    x, y = start_x, start_y

    while distance < max_distance:
        distance += step_size
        x = start_x + dir_x * distance
        y = start_y + dir_y * distance

        px, py = world_to_pixel(x, y, origin_x, origin_y, resolution, map_height)

        # Check bounds
        if px < 0 or px >= map_width or py < 0 or py >= map_height:
            return distance  # Out of map bounds

        if is_wall[py, px]:
            return distance

    return max_distance


def compute_wall_distances(trajectory, is_wall, origin_x, origin_y,
                           resolution, map_height, map_width,
                           max_distance=5.0, car_half_width=0.15):
    """
    Compute left and right wall distances for each trajectory waypoint.
    
    Left normal direction: (-sin(psi), cos(psi))  → positive e_y direction
    Right normal direction: (sin(psi), -cos(psi))  → negative e_y direction
    
    Returns list of (d_left, d_right) tuples.
    Distances are RAW center-to-wall (NOT car-half-subtracted), matching
    the convention expected by the MPC controller and test harness.
    """
    wall_distances = []

    for wp in trajectory:
        x, y, psi = wp['x'], wp['y'], wp['psi']

        # Left perpendicular (positive e_y direction)
        left_dx = -math.sin(psi)
        left_dy = math.cos(psi)

        # Right perpendicular (negative e_y direction)
        right_dx = math.sin(psi)
        right_dy = -math.cos(psi)

        d_left = cast_ray(x, y, left_dx, left_dy, is_wall,
                          origin_x, origin_y, resolution,
                          map_height, map_width, max_distance)

        d_right = cast_ray(x, y, right_dx, right_dy, is_wall,
                           origin_x, origin_y, resolution,
                           map_height, map_width, max_distance)

        wall_distances.append((d_left, d_right))

    return wall_distances


def load_trajectory(csv_path):
    """Load trajectory CSV (7-column TUM format)."""
    waypoints = []
    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].startswith('#'):
                continue
            if len(row) < 7:
                continue
            waypoints.append({
                's': float(row[0]),
                'x': float(row[1]),
                'y': float(row[2]),
                'psi': float(row[3]),
                'kappa': float(row[4]),
                'vx': float(row[5]),
                'ax': float(row[6]),
            })
    return waypoints


def save_trajectory_with_walls(csv_path, waypoints, wall_distances):
    """Save 9-column trajectory CSV."""
    with open(csv_path, 'w', newline='') as f:
        f.write('# s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2,d_left_m,d_right_m\n')
        for wp, (d_left, d_right) in zip(waypoints, wall_distances):
            f.write(f"{wp['s']:.6f},{wp['x']:.6f},{wp['y']:.6f},"
                    f"{wp['psi']:.6f},{wp['kappa']:.6f},{wp['vx']:.6f},"
                    f"{wp['ax']:.6f},{d_left:.6f},{d_right:.6f}\n")


def main():
    parser = argparse.ArgumentParser(description='Compute wall distances for trajectory waypoints')
    parser.add_argument('--map', required=True, help='Path to map YAML file')
    parser.add_argument('--trajectory', required=True, help='Path to input trajectory CSV (7 columns)')
    parser.add_argument('--output', required=True, help='Path to output trajectory CSV (9 columns)')
    parser.add_argument('--max-distance', type=float, default=5.0,
                        help='Maximum raycasting distance (meters)')
    parser.add_argument('--car-width', type=float, default=0.30,
                        help='Car width for safety margin (meters)')
    args = parser.parse_args()

    print(f"Loading map from: {args.map}")
    is_wall, resolution, origin_x, origin_y = load_map(args.map)
    map_height, map_width = is_wall.shape
    print(f"  Map: {map_width}x{map_height}, resolution: {resolution:.4f} m/px")
    print(f"  Origin: ({origin_x:.2f}, {origin_y:.2f})")
    print(f"  Wall pixels: {np.sum(is_wall)} / {is_wall.size} "
          f"({100*np.sum(is_wall)/is_wall.size:.1f}%)")

    print(f"\nLoading trajectory from: {args.trajectory}")
    waypoints = load_trajectory(args.trajectory)
    print(f"  Waypoints: {len(waypoints)}")

    print(f"\nComputing wall distances (max={args.max_distance}m, "
          f"car_width={args.car_width}m)...")
    wall_distances = compute_wall_distances(
        waypoints, is_wall, origin_x, origin_y, resolution,
        map_height, map_width,
        max_distance=args.max_distance,
        car_half_width=args.car_width / 2.0)

    # Print statistics
    d_lefts = [d[0] for d in wall_distances]
    d_rights = [d[1] for d in wall_distances]
    print(f"  Left wall:  min={min(d_lefts):.2f}m, max={max(d_lefts):.2f}m, "
          f"avg={sum(d_lefts)/len(d_lefts):.2f}m")
    print(f"  Right wall: min={min(d_rights):.2f}m, max={max(d_rights):.2f}m, "
          f"avg={sum(d_rights)/len(d_rights):.2f}m")

    # Check for suspiciously small distances
    narrow = sum(1 for dl, dr in wall_distances if dl < 0.3 or dr < 0.3)
    if narrow > 0:
        print(f"  WARNING: {narrow} waypoints have wall distance < 0.3m")

    print(f"\nSaving 9-column trajectory to: {args.output}")
    save_trajectory_with_walls(args.output, waypoints, wall_distances)
    print("Done!")


if __name__ == '__main__':
    main()
