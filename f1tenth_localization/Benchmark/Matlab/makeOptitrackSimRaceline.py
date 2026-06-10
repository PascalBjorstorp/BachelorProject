#!/usr/bin/env python3
"""Create a simulation-safe trajectory from an OptiTrack-extracted raceline."""

from __future__ import annotations

import argparse
import csv
import math
import os
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt, gaussian_filter1d, map_coordinates


HEADER = '# s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2,d_left_m,d_right_m'


def wrap_angle(angle: np.ndarray) -> np.ndarray:
    return np.arctan2(np.sin(angle), np.cos(angle))


def load_map(yaml_path: Path):
    with yaml_path.open('r') as handle:
        info = yaml.safe_load(handle)

    image_path = yaml_path.parent / info['image']
    image = np.array(Image.open(image_path).convert('L'))
    occupied_thresh = float(info.get('occupied_thresh', 0.65))
    threshold = int(255 * (1.0 - occupied_thresh))
    is_wall = image < threshold
    resolution = float(info['resolution'])
    origin = np.array([float(info['origin'][0]), float(info['origin'][1])])
    return is_wall, resolution, origin


def load_trajectory(csv_path: Path) -> np.ndarray:
    rows = []
    with csv_path.open('r', newline='') as handle:
        reader = csv.reader(handle)
        for row in reader:
            if not row or row[0].startswith('#'):
                continue
            if len(row) < 7:
                continue
            rows.append([float(value) for value in row[:9]])
    if not rows:
        raise RuntimeError(f'No trajectory rows found in {csv_path}')
    return np.asarray(rows, dtype=float)


def continuous_row_col(xy: np.ndarray, origin: np.ndarray, resolution: float, height: int):
    col = (xy[:, 0] - origin[0]) / resolution
    row = height - 1 - (xy[:, 1] - origin[1]) / resolution
    return row, col


def pixel_to_world(row: np.ndarray, col: np.ndarray, origin: np.ndarray,
                   resolution: float, height: int):
    x = origin[0] + (col + 0.5) * resolution
    y = origin[1] + (height - 1 - row + 0.5) * resolution
    return np.column_stack([x, y])


def circular_smooth(values: np.ndarray, sigma: float) -> np.ndarray:
    if sigma <= 0:
        return values
    pad = int(max(8, sigma * 5))
    extended = np.vstack([values[-pad:], values, values[:pad]])
    smoothed = np.empty_like(values)
    for col in range(values.shape[1]):
        smoothed[:, col] = gaussian_filter1d(
            extended[:, col], sigma=sigma, mode='nearest')[pad:pad + len(values)]
    return smoothed


def enforce_wall_clearance(xy: np.ndarray, is_wall: np.ndarray, resolution: float,
                           origin: np.ndarray, center_clearance_m: float,
                           margin_m: float, iterations: int,
                           smooth_sigma: float):
    height, width = is_wall.shape
    free = ~is_wall
    distance_px, nearest = distance_transform_edt(free, return_indices=True)
    distance_m = distance_px * resolution

    def sample_distance(points: np.ndarray):
        row, col = continuous_row_col(points, origin, resolution, height)
        return map_coordinates(
            distance_m, [row, col], order=1, mode='constant', cval=0.0)

    def away_from_wall(points: np.ndarray):
        row, col = continuous_row_col(points, origin, resolution, height)
        row_i = np.clip(np.rint(row).astype(int), 0, height - 1)
        col_i = np.clip(np.rint(col).astype(int), 0, width - 1)
        wall_row = nearest[0, row_i, col_i]
        wall_col = nearest[1, row_i, col_i]
        wall_xy = pixel_to_world(wall_row, wall_col, origin, resolution, height)
        direction = points - wall_xy
        norm = np.linalg.norm(direction, axis=1)
        valid = norm > 1e-9
        direction[valid] /= norm[valid, None]

        if not np.all(valid):
            grad_row, grad_col = np.gradient(distance_m, resolution, resolution)
            grad_x = map_coordinates(grad_col, [row[~valid], col[~valid]], order=1)
            grad_y = -map_coordinates(grad_row, [row[~valid], col[~valid]], order=1)
            fallback = np.column_stack([grad_x, grad_y])
            fallback_norm = np.linalg.norm(fallback, axis=1)
            ok = fallback_norm > 1e-9
            fallback[ok] /= fallback_norm[ok, None]
            direction[~valid] = fallback

        return direction

    adjusted = xy.copy()
    target = center_clearance_m + margin_m
    for _ in range(iterations):
        distance = sample_distance(adjusted)
        deficit = np.maximum(0.0, target - distance)
        if float(np.max(deficit)) < 1e-4:
            break
        shift = away_from_wall(adjusted) * deficit[:, None]
        adjusted += circular_smooth(shift, smooth_sigma)

    for _ in range(6):
        distance = sample_distance(adjusted)
        deficit = np.maximum(0.0, target - distance)
        if float(np.max(deficit)) < 1e-4:
            break
        adjusted += away_from_wall(adjusted) * deficit[:, None]

    return adjusted, sample_distance(adjusted)


def recompute_geometry(xy: np.ndarray):
    prev_xy = np.vstack([xy[-1], xy[:-1]])
    next_xy = np.vstack([xy[1:], xy[0]])
    ds_step = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(ds_step)])

    yaw = np.unwrap(np.arctan2(next_xy[:, 1] - prev_xy[:, 1],
                              next_xy[:, 0] - prev_xy[:, 0]))
    curvature = np.zeros(len(xy), dtype=float)
    for idx in range(len(xy)):
        prev_idx = (idx - 1) % len(xy)
        next_idx = (idx + 1) % len(xy)
        ds_local = np.linalg.norm(xy[next_idx] - xy[prev_idx])
        if ds_local > 1e-6:
            dyaw = math.atan2(
                math.sin(yaw[next_idx] - yaw[prev_idx]),
                math.cos(yaw[next_idx] - yaw[prev_idx]))
            curvature[idx] = dyaw / ds_local
    return s, wrap_angle(yaw), curvature


def resample_speed_profile(reference_csv: Path, s_out: np.ndarray,
                           max_speed: float | None):
    reference = load_trajectory(reference_csv)
    s_ref = reference[:, 0]
    vx_ref = reference[:, 5]
    length_ref = float(s_ref[-1])
    length_out = float(s_out[-1])
    s_query = (s_out / max(length_out, 1e-9)) * length_ref
    vx = np.interp(s_query, s_ref, vx_ref)
    if max_speed is not None and max_speed > 0:
        vx = np.minimum(vx, max_speed)

    dv_ds = np.gradient(vx, s_out, edge_order=1)
    ax = vx * dv_ds
    return vx, ax


def cast_ray(x: float, y: float, dx: float, dy: float, is_wall: np.ndarray,
             origin: np.ndarray, resolution: float, max_distance: float):
    height, width = is_wall.shape
    step = resolution * 0.5
    distance = 0.0
    while distance < max_distance:
        distance += step
        px = x + dx * distance
        py = y + dy * distance
        col = int((px - origin[0]) / resolution)
        row = height - 1 - int((py - origin[1]) / resolution)
        if col < 0 or col >= width or row < 0 or row >= height:
            return distance
        if is_wall[row, col]:
            return distance
    return max_distance


def compute_wall_distances(xy: np.ndarray, yaw: np.ndarray, is_wall: np.ndarray,
                           origin: np.ndarray, resolution: float):
    distances = np.zeros((len(xy), 2), dtype=float)
    for idx, ((x, y), psi) in enumerate(zip(xy, yaw)):
        left_dx = -math.sin(float(psi))
        left_dy = math.cos(float(psi))
        right_dx = math.sin(float(psi))
        right_dy = -math.cos(float(psi))
        distances[idx, 0] = cast_ray(
            float(x), float(y), left_dx, left_dy, is_wall, origin, resolution, 5.0)
        distances[idx, 1] = cast_ray(
            float(x), float(y), right_dx, right_dy, is_wall, origin, resolution, 5.0)
    return distances


def write_trajectory(csv_path: Path, s: np.ndarray, xy: np.ndarray, yaw: np.ndarray,
                     curvature: np.ndarray, vx: np.ndarray, ax: np.ndarray,
                     walls: np.ndarray):
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open('w', newline='') as handle:
        handle.write(HEADER + '\n')
        for values in zip(s, xy[:, 0], xy[:, 1], yaw, curvature,
                          vx, ax, walls[:, 0], walls[:, 1]):
            handle.write(','.join(f'{float(value):.6f}' for value in values) + '\n')


def main():
    repo = Path(__file__).resolve().parents[3]
    default_extract_dir = (
        repo / 'f1tenth_localization' / 'Benchmark' / 'Matlab' / 'plots' /
        'BagMapAndRaceline' / 'OptitrackBenchmark_20260430_120324')

    parser = argparse.ArgumentParser()
    parser.add_argument('--map', default=str(default_extract_dir / 'bag_map.yaml'))
    parser.add_argument('--input', default=str(default_extract_dir / 'bag_local_raceline_trajectory.csv'))
    parser.add_argument('--speed-profile',
                        default=str(repo / 'f1tenth_planning' / 'trajectories' / 'my_track_raceline.csv'))
    parser.add_argument('--output',
                        default=str(default_extract_dir / 'bag_local_raceline_trajectory_clearance15cm_normal_speed.csv'))
    parser.add_argument('--car-width', type=float, default=0.273)
    parser.add_argument('--side-clearance', type=float, default=0.15)
    parser.add_argument('--clearance-margin', type=float, default=0.03)
    parser.add_argument('--smooth-sigma', type=float, default=4.0)
    parser.add_argument('--iterations', type=int, default=12)
    parser.add_argument('--max-speed', type=float, default=0.0,
                        help='Cap restored normal profile; <=0 disables cap.')
    args = parser.parse_args()

    map_yaml = Path(args.map)
    input_csv = Path(args.input)
    output_csv = Path(args.output)
    speed_csv = Path(args.speed_profile)
    max_speed = args.max_speed if args.max_speed > 0.0 else None

    is_wall, resolution, origin = load_map(map_yaml)
    trajectory = load_trajectory(input_csv)
    input_xy = trajectory[:, 1:3]
    center_clearance = args.car_width * 0.5 + args.side_clearance

    adjusted_xy, nearest_distance = enforce_wall_clearance(
        input_xy, is_wall, resolution, origin, center_clearance,
        args.clearance_margin, args.iterations, args.smooth_sigma)
    s, yaw, curvature = recompute_geometry(adjusted_xy)
    vx, ax = resample_speed_profile(speed_csv, s, max_speed)
    walls = compute_wall_distances(adjusted_xy, yaw, is_wall, origin, resolution)
    write_trajectory(output_csv, s, adjusted_xy, yaw, curvature, vx, ax, walls)

    displacement = np.linalg.norm(adjusted_xy - input_xy, axis=1)
    side_clearance = np.min(walls) - args.car_width * 0.5
    print(f'Wrote: {output_csv}')
    print(f'Waypoints: {len(adjusted_xy)}')
    print(f'Min nearest center-wall distance: {np.min(nearest_distance):.3f} m')
    print(f'Min raycast side clearance after half car width: {side_clearance:.3f} m')
    print(f'Moved points: {int(np.count_nonzero(displacement > 1e-4))}')
    print(f'Max displacement: {np.max(displacement):.3f} m')
    print(f'Speed profile: min={np.min(vx):.2f} m/s mean={np.mean(vx):.2f} m/s max={np.max(vx):.2f} m/s')


if __name__ == '__main__':
    main()
