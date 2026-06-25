#!/usr/bin/env python3
"""SE(2), angle, and raceline-projection utilities."""
from __future__ import annotations
import numpy as np
from scipy.spatial import cKDTree


def wrap_angle(angle):
    return (np.asarray(angle) + np.pi) % (2.0 * np.pi) - np.pi


def transform_points(points: np.ndarray, pose: np.ndarray) -> np.ndarray:
    """Transform Nx2 local points to world using [x,y,yaw]."""
    c, s = np.cos(pose[2]), np.sin(pose[2])
    out = np.empty_like(points, dtype=float)
    out[:, 0] = pose[0] + c * points[:, 0] - s * points[:, 1]
    out[:, 1] = pose[1] + s * points[:, 0] + c * points[:, 1]
    return out


def fit_se2(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Least-squares rigid transform mapping corresponding source to target."""
    if len(source) < 2:
        raise ValueError("Need at least two correspondences")
    cs = source.mean(axis=0)
    ct = target.mean(axis=0)
    h = (source - cs).T @ (target - ct)
    u, _singular, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0.0:
        vt[-1, :] *= -1.0
        r = vt.T @ u.T
    yaw = float(np.arctan2(r[1, 0], r[0, 0]))
    t = ct - r @ cs
    return np.array([t[0], t[1], yaw], dtype=float)


def interpolate_pose(times: np.ndarray, xs: np.ndarray, ys: np.ndarray, yaws: np.ndarray, query: np.ndarray) -> np.ndarray:
    """Linear xy interpolation and unwrap/interpolate/wrap yaw."""
    if len(times) < 2:
        raise ValueError("Need at least two poses")
    y_unwrapped = np.unwrap(yaws)
    return np.column_stack([
        np.interp(query, times, xs),
        np.interp(query, times, ys),
        wrap_angle(np.interp(query, times, y_unwrapped)),
    ])


def project_to_raceline(points: np.ndarray, raceline: np.ndarray) -> dict[str, np.ndarray]:
    """Project points onto a closed raceline.

    raceline columns: s,x,y,psi,kappa,vx,ax,d_left,d_right.
    """
    xy = raceline[:, 1:3]
    n = len(raceline)
    tree = cKDTree(xy)
    _, nearest = tree.query(points)
    projected = np.empty_like(points, dtype=float)
    s_out = np.empty(len(points))
    psi_out = np.empty(len(points))
    kappa_out = np.empty(len(points))
    vref_out = np.empty(len(points))
    ey_out = np.empty(len(points))
    left_out = np.empty(len(points))
    right_out = np.empty(len(points))
    for j, p in enumerate(points):
        i = int(nearest[j])
        candidates = [(i - 1) % n, i]
        best_d2 = np.inf
        best = None
        for a in candidates:
            b = (a + 1) % n
            pa = xy[a]
            pb = xy[b]
            ab = pb - pa
            denom = float(ab @ ab)
            u = 0.0 if denom < 1e-12 else float(np.clip(((p - pa) @ ab) / denom, 0.0, 1.0))
            q = pa + u * ab
            d2 = float(np.sum((p - q) ** 2))
            if d2 < best_d2:
                best_d2, best = d2, (a, b, u, q)
        a, b, u, q = best
        projected[j] = q
        ds = raceline[b, 0] - raceline[a, 0]
        if b == 0 and ds < 0.0:
            ds += raceline[-1, 0] - raceline[0, 0]
        s_out[j] = raceline[a, 0] + u * ds
        dpsi = wrap_angle(raceline[b, 3] - raceline[a, 3])
        psi = wrap_angle(raceline[a, 3] + u * dpsi)
        psi_out[j] = psi
        kappa_out[j] = raceline[a, 4] + u * (raceline[b, 4] - raceline[a, 4])
        vref_out[j] = raceline[a, 5] + u * (raceline[b, 5] - raceline[a, 5])
        left_out[j] = raceline[a, 7] + u * (raceline[b, 7] - raceline[a, 7])
        right_out[j] = raceline[a, 8] + u * (raceline[b, 8] - raceline[a, 8])
        d = p - q
        ey_out[j] = -d[0] * np.sin(psi) + d[1] * np.cos(psi)
    return {
        "path_x": projected[:, 0], "path_y": projected[:, 1], "path_s": s_out,
        "path_yaw": psi_out, "path_kappa": kappa_out, "path_vref": vref_out,
        "ey": ey_out, "left_bound": left_out, "right_bound": right_out,
    }
