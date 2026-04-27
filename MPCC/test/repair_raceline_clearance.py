#!/usr/bin/env python3

import argparse
import csv
import math
from dataclasses import dataclass


@dataclass
class Waypoint:
    s: float
    x: float
    y: float
    psi: float
    kappa: float
    vx: float
    ax: float
    left_bound: float
    right_bound: float


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def read_raceline(path: str) -> list[Waypoint]:
    points: list[Waypoint] = []
    with open(path, newline="", encoding="utf-8") as handle:
        reader = csv.reader(handle)
        for row in reader:
            if not row:
                continue
            if row[0].startswith("#"):
                continue
            values = [float(value) for value in row[:9]]
            points.append(Waypoint(*values))
    return points


def write_raceline(path: str, points: list[Waypoint]) -> None:
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        for point in points:
            writer.writerow([
                f"{point.s:.6f}",
                f"{point.x:.6f}",
                f"{point.y:.6f}",
                f"{point.psi:.6f}",
                f"{point.kappa:.6f}",
                f"{point.vx:.6f}",
                f"{point.ax:.6f}",
                f"{point.left_bound:.6f}",
                f"{point.right_bound:.6f}",
            ])


def spread_requirement(target: list[float], center: int, required: float, window: int, closed: bool) -> None:
    if required <= 0.0:
        return
    n_points = len(target)
    for offset in range(-window, window + 1):
        index = center + offset
        if closed:
            index %= n_points
        elif index < 0 or index >= n_points:
            continue
        weight = 1.0 - (abs(offset) / float(window + 1))
        candidate = required * weight
        if candidate > target[index]:
            target[index] = candidate


def recompute_geometry(points: list[Waypoint], closed: bool) -> None:
    n_points = len(points)
    psi_new = [point.psi for point in points]
    kappa_new = [point.kappa for point in points]

    for i in range(n_points):
        if closed:
            prev_i = (i - 1) % n_points
            next_i = (i + 1) % n_points
            dx = points[next_i].x - points[prev_i].x
            dy = points[next_i].y - points[prev_i].y
        elif i == 0:
            dx = points[1].x - points[0].x
            dy = points[1].y - points[0].y
        elif i == n_points - 1:
            dx = points[i].x - points[i - 1].x
            dy = points[i].y - points[i - 1].y
        else:
            dx = points[i + 1].x - points[i - 1].x
            dy = points[i + 1].y - points[i - 1].y

        if abs(dx) + abs(dy) > 1.0e-9:
            psi_new[i] = math.atan2(dy, dx)

    for i in range(n_points):
        if closed:
            prev_i = (i - 1) % n_points
            next_i = (i + 1) % n_points
            ds_forward = points[next_i].s - points[i].s
            ds_backward = points[i].s - points[prev_i].s
            if ds_forward < 0.0:
                ds_forward += points[-1].s
            if ds_backward < 0.0:
                ds_backward += points[-1].s
            ds = ds_forward + ds_backward
            dpsi = wrap_angle(psi_new[next_i] - psi_new[prev_i])
        elif i == 0:
            ds = points[1].s - points[0].s
            dpsi = wrap_angle(psi_new[1] - psi_new[0])
        elif i == n_points - 1:
            ds = points[i].s - points[i - 1].s
            dpsi = wrap_angle(psi_new[i] - psi_new[i - 1])
        else:
            ds = points[i + 1].s - points[i - 1].s
            dpsi = wrap_angle(psi_new[i + 1] - psi_new[i - 1])

        if abs(ds) > 1.0e-9:
            kappa_new[i] = dpsi / ds

    for i, point in enumerate(points):
        point.psi = psi_new[i]
        point.kappa = kappa_new[i]


def repair_raceline(
    points: list[Waypoint],
    vehicle_half_width: float,
    safety_margin: float,
    min_clearance: float,
    smoothing_window: int,
    closed: bool,
) -> tuple[int, int, float]:
    eff_left = [point.left_bound - vehicle_half_width - safety_margin for point in points]
    eff_right = [point.right_bound - vehicle_half_width - safety_margin for point in points]
    right_shift_need = [0.0] * len(points)
    left_shift_need = [0.0] * len(points)

    invalid_before = 0
    for i in range(len(points)):
        if eff_left[i] < min_clearance:
            spread_requirement(right_shift_need, i, min_clearance - eff_left[i], smoothing_window, closed)
        if eff_right[i] < min_clearance:
            spread_requirement(left_shift_need, i, min_clearance - eff_right[i], smoothing_window, closed)
        if eff_left[i] < 0.0 or eff_right[i] < 0.0:
            invalid_before += 1

    shifted_points = 0
    max_shift = 0.0
    for i, point in enumerate(points):
        offset = right_shift_need[i] - left_shift_need[i]
        lower = min_clearance - eff_left[i]
        upper = eff_right[i] - min_clearance

        if lower <= upper:
            offset = min(max(offset, lower), upper)
        else:
            offset = 0.5 * (lower + upper)

        if abs(offset) > 1.0e-6:
            point.x += math.sin(point.psi) * offset
            point.y -= math.cos(point.psi) * offset
            point.left_bound += offset
            point.right_bound -= offset
            shifted_points += 1
            max_shift = max(max_shift, abs(offset))

    recompute_geometry(points, closed)

    invalid_after = 0
    for point in points:
        left = point.left_bound - vehicle_half_width - safety_margin
        right = point.right_bound - vehicle_half_width - safety_margin
        if left < 0.0 or right < 0.0:
            invalid_after += 1

    return invalid_before, invalid_after, max_shift


def main() -> int:
    parser = argparse.ArgumentParser(description="Repair wall-hugging raceline slices by shifting the reference inside the feasible corridor.")
    parser.add_argument("input_csv")
    parser.add_argument("output_csv")
    parser.add_argument("--vehicle-half-width", type=float, default=0.155)
    parser.add_argument("--safety-margin", type=float, default=0.01)
    parser.add_argument("--min-clearance", type=float, default=0.02)
    parser.add_argument("--smoothing-window", type=int, default=4)
    parser.add_argument("--open-path", action="store_true")
    args = parser.parse_args()

    points = read_raceline(args.input_csv)
    invalid_before, invalid_after, max_shift = repair_raceline(
        points,
        vehicle_half_width=args.vehicle_half_width,
        safety_margin=args.safety_margin,
        min_clearance=args.min_clearance,
        smoothing_window=args.smoothing_window,
        closed=not args.open_path,
    )
    write_raceline(args.output_csv, points)

    print(
        f"Repaired raceline: invalid points before={invalid_before}, after={invalid_after}, "
        f"max_shift={max_shift:.4f} m, output={args.output_csv}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())