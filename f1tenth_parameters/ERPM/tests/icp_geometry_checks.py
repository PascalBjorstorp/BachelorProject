#!/usr/bin/env python3
"""Synthetic-geometry checks for the scan-matching core.

These run without ROS or a bag. They validate the three properties the ICP
improvements rely on:

1. point-to-line ICP recovers a known rigid scan-to-scan motion to sub-mm /
   sub-mrad accuracy on geometrically rich (L-shaped) scenery;
2. seeding the translation with the constant-velocity prediction does not move
   the converged solution (it only changes the starting point); and
3. ``deskew_scan`` exactly inverts the constant-velocity within-scan skew model.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "analysis"))

from estimate_lidar_motion import deskew_scan, pca_normals, robust_point_to_line_icp, rot  # noqa: E402


def _l_shaped_scene(spacing: float = 0.03) -> np.ndarray:
    """Two perpendicular walls plus a short diagonal: well-conditioned geometry."""
    xs = np.arange(-3.0, 3.0, spacing)
    ys = np.arange(-2.0, 3.0, spacing)
    ds = np.arange(-2.0, -0.5, spacing)
    wall_x = np.column_stack((xs, np.full(len(xs), 2.5)))
    wall_y = np.column_stack((np.full(len(ys), 3.0), ys))
    diag = np.column_stack((ds, ds * 0.5 - 1.0))
    return np.vstack((wall_x, wall_y, diag))


def _icp(source: np.ndarray, target: np.ndarray, yaw_seed: float, translation_seed=None):
    normals = pca_normals(target, 9)
    return robust_point_to_line_icp(
        source, target, normals,
        yaw_seed=yaw_seed, max_iter=80, max_corr=0.3, trim_fraction=0.9, min_corr=50,
        translation_tolerance_m=1e-7, rotation_tolerance_rad=1e-7, relative_cost_tolerance=1e-9,
        translation_seed=translation_seed,
    )


def check_rigid_recovery() -> None:
    prev = _l_shaped_scene()
    dyaw, d = 0.05, np.array([0.08, -0.02])
    # Current scan observes the same world points from pose (R(dyaw), d).
    curr = (prev - d) @ rot(dyaw)  # == R(dyaw)^T (prev - d) since (R^T x^T)^T = x R
    result = _icp(curr, prev, yaw_seed=dyaw)
    recovered_yaw = math.atan2(result.R[1, 0], result.R[0, 0])
    assert abs(recovered_yaw - dyaw) < 1e-3, recovered_yaw
    assert np.allclose(result.t, d, atol=1e-3), result.t


def check_translation_seed_is_neutral() -> None:
    prev = _l_shaped_scene()
    dyaw, d = -0.03, np.array([0.12, 0.04])
    curr = (prev - d) @ rot(dyaw)
    cold = _icp(curr, prev, yaw_seed=dyaw, translation_seed=None)
    warm = _icp(curr, prev, yaw_seed=dyaw, translation_seed=d)
    # Same converged motion regardless of starting translation.
    assert np.allclose(cold.t, warm.t, atol=1e-4), (cold.t, warm.t)
    assert abs(math.atan2(cold.R[1, 0], cold.R[0, 0]) - math.atan2(warm.R[1, 0], warm.R[0, 0])) < 1e-4


def check_deskew_inverts_skew() -> None:
    p0 = _l_shaped_scene()
    omega, v = 0.6, np.array([2.5, 0.1])  # rad/s and m/s in the sensor frame
    rel_t = np.linspace(0.0, 0.025, len(p0))  # 40 Hz scan
    # Forward skew model: beam at time tau is measured in the sensor@tau frame.
    measured = np.empty_like(p0)
    for i, tau in enumerate(rel_t):
        measured[i] = rot(omega * tau).T @ (p0[i] - v * tau)
    recovered = deskew_scan(measured, rel_t, v, omega)
    assert np.max(np.abs(recovered - p0)) < 1e-9, float(np.max(np.abs(recovered - p0)))
    # No per-beam time or zero velocity must be a strict no-op.
    assert np.array_equal(deskew_scan(measured, np.zeros(len(p0)), v, omega), measured)


def main() -> int:
    check_rigid_recovery()
    check_translation_seed_is_neutral()
    check_deskew_inverts_skew()
    print("icp geometry checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
