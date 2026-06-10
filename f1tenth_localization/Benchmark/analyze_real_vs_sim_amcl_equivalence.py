#!/usr/bin/env python3
"""Equivalence analysis for real OptiTrack AMCL vs OptiTrack-map simulation.

The statistical unit is one physical lap for the real bags and one one-lap
simulation run for the simulated bags. This avoids treating correlated
high-rate samples as independent observations.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import numpy as np
import pandas as pd
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

try:
    from scipy import stats
except ImportError as exc:  # pragma: no cover - scipy is expected in this workspace
    raise SystemExit("scipy is required for the TOST analysis") from exc

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError as exc:  # pragma: no cover
    raise SystemExit("matplotlib is required for the equivalence plots") from exc


PARTICLES = (100, 200, 400, 600, 800, 1000, 1500)
METRIC_MARGINS = {
    "median_position_cm": 5.0,
    "p95_position_cm": 10.0,
    "rmse_position_cm": 10.0,
    "median_abs_yaw_deg": 1.5,
    "p95_abs_yaw_deg": 3.0,
    "rmse_yaw_deg": 2.0,
}


def qnorm(q: Sequence[float]) -> np.ndarray:
    q_arr = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q_arr)
    return q_arr / norm if norm > 0.0 else q_arr


def qmul(q1: Sequence[float], q2: Sequence[float]) -> np.ndarray:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return qnorm([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def quat_to_rot(q: Sequence[float]) -> np.ndarray:
    x, y, z, w = qnorm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=float)


def yaw_from_quat(q: Sequence[float]) -> float:
    x, y, z, w = qnorm(q)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def wrap_angle(values: np.ndarray) -> np.ndarray:
    return np.arctan2(np.sin(values), np.cos(values))


def interp_yaw(t_src: np.ndarray, yaw_src: np.ndarray, t_dst: np.ndarray) -> np.ndarray:
    return np.arctan2(
        np.interp(t_dst, t_src, np.sin(yaw_src)),
        np.interp(t_dst, t_src, np.cos(yaw_src)),
    )


def circular_mean(values: np.ndarray) -> float:
    values = values[np.isfinite(values)]
    if values.size == 0:
        return 0.0
    return math.atan2(np.mean(np.sin(values)), np.mean(np.cos(values)))


def open_reader(bag_dir: Path) -> Tuple[rosbag2_py.SequentialReader, Dict[str, str]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    return reader, topic_types


def read_pose_series(bag_dir: Path, topic: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    reader, topic_types = open_reader(bag_dir)
    msg_type = get_message(topic_types[topic])
    stamps: List[float] = []
    positions: List[List[float]] = []
    quats: List[List[float]] = []

    while reader.has_next():
        topic_name, data, _ = reader.read_next()
        if topic_name != topic:
            continue
        msg = deserialize_message(data, msg_type)
        stamp = msg.header.stamp
        pose = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
        pos = pose.position
        quat = pose.orientation
        stamps.append(stamp.sec + stamp.nanosec * 1e-9)
        positions.append([pos.x, pos.y, pos.z])
        quats.append([quat.x, quat.y, quat.z, quat.w])

    t = np.asarray(stamps, dtype=float)
    pos = np.asarray(positions, dtype=float)
    quat = np.asarray(quats, dtype=float)
    order = np.argsort(t)
    t = t[order]
    pos = pos[order]
    quat = quat[order]
    keep = np.r_[True, np.diff(t) > 0.0]
    return t[keep], pos[keep], quat[keep]


def read_map_world_tf(bag_dir: Path) -> Tuple[np.ndarray, np.ndarray] | None:
    reader, topic_types = open_reader(bag_dir)
    if "/tf_static" not in topic_types:
        return None
    msg_type = get_message(topic_types["/tf_static"])
    while reader.has_next():
        topic_name, data, _ = reader.read_next()
        if topic_name != "/tf_static":
            continue
        msg = deserialize_message(data, msg_type)
        for transform in msg.transforms:
            if (transform.header.frame_id.strip("/") == "map" and
                    transform.child_frame_id.strip("/") == "world"):
                trans = transform.transform.translation
                rot = transform.transform.rotation
                return (
                    np.array([trans.x, trans.y, trans.z], dtype=float),
                    qnorm([rot.x, rot.y, rot.z, rot.w]),
                )
    return None


def filter_optitrack(
    t: np.ndarray,
    pos: np.ndarray,
    quat: np.ndarray,
    max_speed_mps: float = 12.0,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    keep = (
        np.isfinite(t)
        & np.all(np.isfinite(pos), axis=1)
        & np.all(np.isfinite(quat), axis=1)
        & (np.linalg.norm(pos[:, :2], axis=1) > 1e-6)
    )
    t = t[keep]
    pos = pos[keep]
    quat = quat[keep]

    if t.size > 1:
        step = np.linalg.norm(np.diff(pos[:, :2], axis=0), axis=1)
        keep = np.r_[True, step > 1e-6]
        t = t[keep]
        pos = pos[keep]
        quat = quat[keep]

    for _ in range(3):
        if t.size < 2:
            break
        dt = np.diff(t)
        step = np.linalg.norm(np.diff(pos[:, :2], axis=0), axis=1)
        speed = step / np.maximum(dt, np.finfo(float).eps)
        bad = np.r_[False, (dt > 0.0) & (speed > max_speed_mps)]
        if not np.any(bad):
            break
        t = t[~bad]
        pos = pos[~bad]
        quat = quat[~bad]
    return t, pos, quat


def real_bag_for_particles(real_root: Path, particles: int) -> Path:
    matches = sorted((real_root / f"ParticleCount{particles}").glob("OptitrackBenchmark_*"))
    if len(matches) != 1:
        raise RuntimeError(f"Expected one real bag for {particles}, got {matches}")
    return matches[0]


def roi_mask(x: np.ndarray, y: np.ndarray, roi: Tuple[float, float, float, float]) -> np.ndarray:
    min_x, max_x, min_y, max_y = roi
    return (x >= min_x) & (x <= max_x) & (y >= min_y) & (y <= max_y)


def roi_mask_any(
    x: np.ndarray,
    y: np.ndarray,
    rois: Sequence[Tuple[float, float, float, float]],
) -> np.ndarray:
    mask = np.zeros_like(x, dtype=bool)
    for roi in rois:
        mask |= roi_mask(x, y, roi)
    return mask


def roi_bounds(rois: Sequence[Tuple[float, float, float, float]]) -> Tuple[float, float, float, float]:
    return (
        min(roi[0] for roi in rois),
        max(roi[1] for roi in rois),
        min(roi[2] for roi in rois),
        max(roi[3] for roi in rois),
    )


def progress_since_run_start(progress_s: np.ndarray, progress_ratio: np.ndarray | None) -> np.ndarray:
    progress_s = np.asarray(progress_s, dtype=float)
    if progress_s.size == 0:
        return progress_s

    track_length = math.nan
    if progress_ratio is not None:
        ratio = np.asarray(progress_ratio, dtype=float)
        valid = np.isfinite(progress_s) & np.isfinite(ratio) & (np.abs(ratio) > 1e-6)
        if np.any(valid):
            candidates = progress_s[valid] / ratio[valid]
            candidates = candidates[np.isfinite(candidates) & (candidates > 0.0)]
            if candidates.size:
                track_length = float(np.median(candidates))

    relative = progress_s - progress_s[0]
    if math.isfinite(track_length) and track_length > 0.0:
        relative = np.mod(relative, track_length)
    return relative


def start_calibrate(
    t: np.ndarray,
    gt_xy: np.ndarray,
    gt_yaw: np.ndarray,
    est_xy: np.ndarray,
    est_yaw: np.ndarray,
    duration_s: float = 3.0,
) -> Tuple[np.ndarray, np.ndarray, Dict[str, float]]:
    cal = (t >= t[0]) & (t <= t[0] + duration_s)
    info = {
        "calibration_samples": float(np.count_nonzero(cal)),
        "calibration_applied": 0.0,
    }
    if np.count_nonzero(cal) < 50:
        return gt_xy, gt_yaw, info

    delta_yaw = circular_mean(wrap_angle(est_yaw[cal] - gt_yaw[cal]))
    rot = np.array([
        [math.cos(delta_yaw), -math.sin(delta_yaw)],
        [math.sin(delta_yaw), math.cos(delta_yaw)],
    ])
    gt_mean = np.mean(gt_xy[cal], axis=0)
    est_mean = np.mean(est_xy[cal], axis=0)
    gt_xy_out = (rot @ (gt_xy - gt_mean).T).T + est_mean
    gt_yaw_out = wrap_angle(gt_yaw + delta_yaw)
    info["calibration_applied"] = 1.0
    return gt_xy_out, gt_yaw_out, info


def apply_se2_correction(
    xy: np.ndarray,
    yaw: np.ndarray,
    dx_m: float,
    dy_m: float,
    dyaw_rad: float,
) -> Tuple[np.ndarray, np.ndarray]:
    if dx_m == 0.0 and dy_m == 0.0 and dyaw_rad == 0.0:
        return xy, yaw
    c = math.cos(dyaw_rad)
    s = math.sin(dyaw_rad)
    xy_out = np.column_stack([
        c * xy[:, 0] - s * xy[:, 1] + dx_m,
        s * xy[:, 0] + c * xy[:, 1] + dy_m,
    ])
    yaw_out = wrap_angle(yaw + dyaw_rad)
    return xy_out, yaw_out


def lap_segments(
    t: np.ndarray,
    xy: np.ndarray,
    detection_hz: float = 50.0,
    candidate_step_s: float = 0.50,
    ignore_edge_s: float = 2.0,
    radius_candidates_m: Sequence[float] = (0.35, 0.45, 0.60, 0.80),
    min_lap_time_s: float = 8.0,
    max_lap_time_s: float = 20.0,
    max_complete_laps: int = 5,
    min_complete_laps_expected: int = 4,
) -> List[Tuple[int, int]]:
    if t.size < 10:
        return []

    tq = np.arange(t[0], t[-1], 1.0 / detection_hz)
    if tq.size < 10:
        return []
    xq = np.interp(tq, t, xy[:, 0])
    yq = np.interp(tq, t, xy[:, 1])
    smooth_window = max(3, int(2 * math.floor(0.12 * detection_hz / 2.0) + 1))
    xq = pd.Series(xq).rolling(smooth_window, center=True, min_periods=1).median().to_numpy()
    yq = pd.Series(yq).rolling(smooth_window, center=True, min_periods=1).median().to_numpy()
    xyq = np.column_stack([xq, yq])

    first_candidate = int(np.searchsorted(tq, tq[0] + ignore_edge_s, side="left"))
    last_candidate = int(np.searchsorted(tq, tq[-1] - ignore_edge_s, side="right")) - 1
    if first_candidate >= last_candidate:
        return []

    candidate_step = max(1, int(round(candidate_step_s * detection_hz)))
    best_score = None
    best_cross_times: np.ndarray | None = None

    for radius in radius_candidates_m:
        for candidate_idx in range(first_candidate, last_candidate + 1, candidate_step):
            cross_times, cross_distances = crossings_for_point(
                tq, xyq, xyq[candidate_idx, :], radius, min_lap_time_s)
            if cross_times.size < 2:
                continue
            intervals = np.diff(cross_times)
            interval_ok = (intervals >= min_lap_time_s) & (intervals <= max_lap_time_s)
            run_start, run_end = longest_true_run(interval_ok)
            if run_end < run_start:
                continue
            selected_times = cross_times[run_start:run_end + 2]
            selected_distances = cross_distances[run_start:run_end + 2]
            n_laps = selected_times.size - 1
            if n_laps > max_complete_laps:
                selected_times = selected_times[-(max_complete_laps + 1):]
                selected_distances = selected_distances[-(max_complete_laps + 1):]
                n_laps = max_complete_laps
            lap_intervals = np.diff(selected_times)
            score = (
                n_laps,
                selected_times[-1] - selected_times[0],
                -float(np.std(lap_intervals)),
                -float(np.mean(selected_distances)),
                -radius,
            )
            if best_score is None or score > best_score:
                best_score = score
                best_cross_times = selected_times
        if best_score is not None and best_score[0] >= min_complete_laps_expected:
            break

    if best_cross_times is None:
        return []
    segments: List[Tuple[int, int]] = []
    for start_t, end_t in zip(best_cross_times[:-1], best_cross_times[1:]):
        start_idx = int(np.searchsorted(t, start_t, side="left"))
        end_idx = int(np.searchsorted(t, end_t, side="right")) - 1
        if 0 <= start_idx < end_idx < t.size:
            segments.append((start_idx, end_idx))
    return segments


def crossings_for_point(
    t: np.ndarray,
    xy: np.ndarray,
    point: np.ndarray,
    radius_m: float,
    min_lap_time_s: float,
) -> Tuple[np.ndarray, np.ndarray]:
    distance = np.linalg.norm(xy - point, axis=1)
    inside = distance <= radius_m
    edge = np.diff(np.r_[False, inside, False].astype(int))
    starts = np.where(edge == 1)[0]
    ends = np.where(edge == -1)[0] - 1
    cross_times: List[float] = []
    cross_distances: List[float] = []
    for start, end in zip(starts, ends):
        local_idx = int(np.argmin(distance[start:end + 1]))
        idx = start + local_idx
        cross_time = float(t[idx])
        cross_distance = float(distance[idx])
        if not cross_times:
            cross_times.append(cross_time)
            cross_distances.append(cross_distance)
            continue
        if cross_time - cross_times[-1] < 0.5 * min_lap_time_s:
            if cross_distance < cross_distances[-1]:
                cross_times[-1] = cross_time
                cross_distances[-1] = cross_distance
        else:
            cross_times.append(cross_time)
            cross_distances.append(cross_distance)
    return np.asarray(cross_times), np.asarray(cross_distances)


def longest_true_run(values: np.ndarray) -> Tuple[int, int]:
    best_start = 0
    best_end = -1
    current_start: int | None = None
    for i, value in enumerate(values):
        if value and current_start is None:
            current_start = i
        if (not value or i == values.size - 1) and current_start is not None:
            current_end = i if value and i == values.size - 1 else i - 1
            if current_end - current_start > best_end - best_start:
                best_start = current_start
                best_end = current_end
            current_start = None
    return best_start, best_end


def metric_row(
    particles: int,
    source: str,
    unit: str,
    xy_error_cm: np.ndarray,
    yaw_error_deg: np.ndarray,
    n_roi_samples: int,
) -> Dict[str, float | int | str]:
    yaw_abs = np.abs(yaw_error_deg)
    return {
        "particles": particles,
        "source": source,
        "unit": unit,
        "n_roi_samples": n_roi_samples,
        "median_position_cm": float(np.median(xy_error_cm)),
        "p95_position_cm": float(np.percentile(xy_error_cm, 95)),
        "rmse_position_cm": float(math.sqrt(np.mean(xy_error_cm ** 2))),
        "median_abs_yaw_deg": float(np.median(yaw_abs)),
        "p95_abs_yaw_deg": float(np.percentile(yaw_abs, 95)),
        "rmse_yaw_deg": float(math.sqrt(np.mean(yaw_abs ** 2))),
    }


def load_real_unit_metrics(
    real_root: Path,
    particles: int,
    rois: Sequence[Tuple[float, float, float, float]],
    min_speed_mps: float,
    fallback_tf: Tuple[np.ndarray, np.ndarray],
    real_tf_correction: Tuple[float, float, float],
    real_start_calibration_s: float,
    estimator: str,
) -> Tuple[List[Dict[str, float | int | str]], pd.DataFrame, int]:
    bag_dir = real_bag_for_particles(real_root, particles)
    tf = read_map_world_tf(bag_dir)
    tf_source_missing = 0
    if tf is None:
        tf = fallback_tf
        tf_source_missing = 1
    trans, q_tf = tf
    rot = quat_to_rot(q_tf)

    t_opti, opti_raw, q_opti_raw = read_pose_series(bag_dir, "/vrpn_mocap/Car2/pose")
    estimator_topic = "/ekf_pose" if estimator == "ekf" else "/amcl_pose"
    t_est, est_pos, q_est = read_pose_series(bag_dir, estimator_topic)
    t_opti, opti_raw, q_opti_raw = filter_optitrack(t_opti, opti_raw, q_opti_raw)

    gt_pos = (rot @ opti_raw.T).T + trans
    gt_yaw = np.array([yaw_from_quat(qmul(q_tf, q)) for q in q_opti_raw])
    est_yaw_all = np.array([yaw_from_quat(q) for q in q_est])

    t0 = max(float(np.min(t_opti)), float(np.min(t_est)))
    t1 = min(float(np.max(t_opti)), float(np.max(t_est)))
    time_mask = (t_opti >= t0) & (t_opti <= t1)
    t = t_opti[time_mask]
    gt_xy_raw = gt_pos[time_mask, :2].copy()
    gt_yaw_raw = gt_yaw[time_mask].copy()
    gt_xy_raw, gt_yaw_raw = apply_se2_correction(
        gt_xy_raw, gt_yaw_raw, *real_tf_correction)
    est_xy = np.column_stack([
        np.interp(t, t_est, est_pos[:, 0]),
        np.interp(t, t_est, est_pos[:, 1]),
    ])
    est_yaw = interp_yaw(t_est, est_yaw_all, t)
    if real_start_calibration_s > 0.0:
        gt_xy_error, gt_yaw_error, _ = start_calibrate(
            t, gt_xy_raw.copy(), gt_yaw_raw.copy(), est_xy, est_yaw,
            duration_s=real_start_calibration_s)
    else:
        gt_xy_error = gt_xy_raw
        gt_yaw_error = gt_yaw_raw

    speed = np.r_[
        np.nan,
        np.linalg.norm(np.diff(gt_xy_raw, axis=0), axis=1) /
        np.maximum(np.diff(t), np.finfo(float).eps),
    ]
    xy_error_cm = 100.0 * np.linalg.norm(est_xy - gt_xy_error, axis=1)
    yaw_error_deg = np.degrees(wrap_angle(est_yaw - gt_yaw_error))
    segments = lap_segments(t, gt_xy_raw)

    metrics: List[Dict[str, float | int | str]] = []
    samples: List[Dict[str, float | int | str]] = []
    for lap_idx, (start, end) in enumerate(segments, start=1):
        idx = np.arange(start, end + 1)
        mask = (
            roi_mask_any(gt_xy_raw[idx, 0], gt_xy_raw[idx, 1], rois)
            & (speed[idx] >= min_speed_mps)
            & np.isfinite(xy_error_cm[idx])
            & np.isfinite(yaw_error_deg[idx])
        )
        idx = idx[mask]
        if idx.size < 10:
            continue
        unit = f"real_lap_{lap_idx:02d}"
        metrics.append(metric_row(
            particles, "real", unit, xy_error_cm[idx], yaw_error_deg[idx], int(idx.size)))
        for i in idx:
            samples.append({
                "particles": particles,
                "source": "real",
                "unit": unit,
                "x_m": gt_xy_raw[i, 0],
                "y_m": gt_xy_raw[i, 1],
                "position_error_cm": xy_error_cm[i],
                "yaw_error_deg": yaw_error_deg[i],
            })
    return metrics, pd.DataFrame(samples), tf_source_missing


def load_sim_unit_metrics(
    sim_root: Path,
    particles: int,
    rois: Sequence[Tuple[float, float, float, float]],
    min_speed_mps: float,
    runs: Iterable[int],
    sim_max_amcl_age_s: float,
    sim_min_progress_m: float,
    sim_min_roi_samples: int,
    estimator: str,
) -> Tuple[List[Dict[str, float | int | str]], pd.DataFrame]:
    metrics: List[Dict[str, float | int | str]] = []
    samples: List[Dict[str, float | int | str]] = []
    for run_idx in runs:
        csv_path = (
            sim_root / f"particles_{particles:04d}" / f"run_{run_idx:02d}" /
            "AMCL_benchmark" / "AMCL_benchmark.csv"
        )
        if not csv_path.exists():
            continue
        table = pd.read_csv(csv_path)
        numeric_columns = [
            "gt_vx", "gt_vy", "gt_x", "gt_y", "gt_yaw",
            "amcl_x", "amcl_y", "amcl_yaw",
            "ekf_x", "ekf_y", "ekf_yaw",
            "collision", "wall_time_ns", "amcl_stamp_ns",
            "ekf_stamp_ns",
            "progress_s", "progress_ratio",
        ]
        for column in numeric_columns:
            if column in table.columns:
                table[column] = pd.to_numeric(table[column], errors="coerce")
        speed = np.hypot(table["gt_vx"].to_numpy(), table["gt_vy"].to_numpy())
        gt_x = table["gt_x"].to_numpy()
        gt_y = table["gt_y"].to_numpy()
        gt_yaw = table["gt_yaw"].to_numpy()
        est_x = table[f"{estimator}_x"].to_numpy()
        est_y = table[f"{estimator}_y"].to_numpy()
        est_yaw = table[f"{estimator}_yaw"].to_numpy()
        mask = (
            roi_mask_any(gt_x, gt_y, rois)
            & (speed >= min_speed_mps)
            & np.isfinite(est_x)
            & np.isfinite(est_y)
            & np.isfinite(est_yaw)
        )
        if "collision" in table.columns:
            mask &= table["collision"].to_numpy() == 0
        if math.isfinite(sim_max_amcl_age_s):
            stamp_column = f"{estimator}_stamp_ns"
            if "wall_time_ns" not in table.columns or stamp_column not in table.columns:
                raise RuntimeError(f"sim_max_amcl_age_s requires wall_time_ns and {stamp_column}")
            age_s = (table["wall_time_ns"].to_numpy() - table[stamp_column].to_numpy()) * 1e-9
            mask &= age_s <= sim_max_amcl_age_s
        if math.isfinite(sim_min_progress_m):
            if "progress_s" not in table.columns:
                raise RuntimeError("sim_min_progress_m requires progress_s")
            progress_ratio = (
                table["progress_ratio"].to_numpy()
                if "progress_ratio" in table.columns
                else None
            )
            travelled_m = progress_since_run_start(table["progress_s"].to_numpy(), progress_ratio)
            mask &= travelled_m >= sim_min_progress_m
        if np.count_nonzero(mask) < sim_min_roi_samples:
            continue
        pos_cm = 100.0 * np.hypot(est_x[mask] - gt_x[mask], est_y[mask] - gt_y[mask])
        yaw_deg = np.degrees(wrap_angle(est_yaw[mask] - gt_yaw[mask]))
        unit = f"sim_run_{run_idx:02d}"
        metrics.append(metric_row(
            particles, "sim", unit, pos_cm, yaw_deg, int(np.count_nonzero(mask))))
        for x, y, pos, yaw in zip(
            gt_x[mask],
            gt_y[mask],
            pos_cm,
            yaw_deg,
        ):
            samples.append({
                "particles": particles,
                "source": "sim",
                "unit": unit,
                "x_m": x,
                "y_m": y,
                "position_error_cm": pos,
                "yaw_error_deg": yaw,
            })
    return metrics, pd.DataFrame(samples)


def welch_tost(
    sim_values: np.ndarray,
    real_values: np.ndarray,
    margin: float,
    alpha: float = 0.05,
) -> Dict[str, float | int]:
    sim_values = sim_values[np.isfinite(sim_values)]
    real_values = real_values[np.isfinite(real_values)]
    n_sim = sim_values.size
    n_real = real_values.size
    diff = float(np.mean(sim_values) - np.mean(real_values))
    var_sim = float(np.var(sim_values, ddof=1))
    var_real = float(np.var(real_values, ddof=1))
    se = math.sqrt(var_sim / n_sim + var_real / n_real)
    df_num = (var_sim / n_sim + var_real / n_real) ** 2
    df_den = ((var_sim / n_sim) ** 2 / (n_sim - 1)) + ((var_real / n_real) ** 2 / (n_real - 1))
    df = df_num / df_den if df_den > 0.0 else math.inf
    t_low = (diff + margin) / se if se > 0.0 else math.inf
    t_high = (diff - margin) / se if se > 0.0 else -math.inf
    p_low = 1.0 - stats.t.cdf(t_low, df)
    p_high = stats.t.cdf(t_high, df)
    tcrit = stats.t.ppf(1.0 - alpha, df)
    ci_low = diff - tcrit * se
    ci_high = diff + tcrit * se
    p_tost = max(float(p_low), float(p_high))
    return {
        "n_sim": int(n_sim),
        "n_real": int(n_real),
        "mean_diff_sim_minus_real": diff,
        "margin": margin,
        "welch_df": float(df),
        "tost_p_low": float(p_low),
        "tost_p_high": float(p_high),
        "tost_p": p_tost,
        "ci90_low": float(ci_low),
        "ci90_high": float(ci_high),
        "equivalent_tost": int(p_tost < alpha and ci_low > -margin and ci_high < margin),
    }


def bootstrap_ci(
    sim_values: np.ndarray,
    real_values: np.ndarray,
    rng: np.random.Generator,
    n_boot: int = 20_000,
) -> Tuple[float, float]:
    sim_values = sim_values[np.isfinite(sim_values)]
    real_values = real_values[np.isfinite(real_values)]
    diffs = np.empty(n_boot, dtype=float)
    for i in range(n_boot):
        sim_sample = rng.choice(sim_values, size=sim_values.size, replace=True)
        real_sample = rng.choice(real_values, size=real_values.size, replace=True)
        diffs[i] = np.mean(sim_sample) - np.mean(real_sample)
    return float(np.percentile(diffs, 5)), float(np.percentile(diffs, 95))


def spatial_bin_summary(samples: pd.DataFrame, roi: Tuple[float, float, float, float]) -> pd.DataFrame:
    rows: List[Dict[str, float | int]] = []
    bin_size = 0.5
    for particles, group in samples.groupby("particles"):
        real = group[group["source"] == "real"].copy()
        sim = group[group["source"] == "sim"].copy()
        if real.empty or sim.empty:
            continue
        for frame in (real, sim):
            frame["bin_x"] = np.floor((frame["x_m"] - roi[0]) / bin_size).astype(int)
            frame["bin_y"] = np.floor((frame["y_m"] - roi[2]) / bin_size).astype(int)
        real_bins = real.groupby(["bin_x", "bin_y"])["position_error_cm"].median()
        sim_bins = sim.groupby(["bin_x", "bin_y"])["position_error_cm"].median()
        common = sorted(set(real_bins.index) & set(sim_bins.index))
        if len(common) < 3:
            continue
        real_vals = np.asarray([real_bins.loc[idx] for idx in common], dtype=float)
        sim_vals = np.asarray([sim_bins.loc[idx] for idx in common], dtype=float)
        diff = sim_vals - real_vals
        corr = float(np.corrcoef(real_vals, sim_vals)[0, 1]) if np.std(real_vals) > 0 and np.std(sim_vals) > 0 else math.nan
        rows.append({
            "particles": int(particles),
            "n_common_bins": len(common),
            "pearson_corr": corr,
            "bias_sim_minus_real_cm": float(np.mean(diff)),
            "median_abs_bin_diff_cm": float(np.median(np.abs(diff))),
            "loa_low_cm": float(np.mean(diff) - 1.96 * np.std(diff, ddof=1)),
            "loa_high_cm": float(np.mean(diff) + 1.96 * np.std(diff, ddof=1)),
        })
    return pd.DataFrame(rows)


def plot_metric_boxes(metrics: pd.DataFrame, output_dir: Path) -> None:
    plot_specs = [
        ("median_position_cm", "median position error [cm]", "equivalence_position_median_boxplot.png"),
        ("p95_position_cm", "p95 position error [cm]", "equivalence_position_p95_boxplot.png"),
        ("median_abs_yaw_deg", "median absolute yaw error [deg]", "equivalence_yaw_median_boxplot.png"),
    ]
    for metric, ylabel, filename in plot_specs:
        fig, ax = plt.subplots(figsize=(10, 4.8))
        positions: List[float] = []
        labels: List[str] = []
        data: List[np.ndarray] = []
        for i, particles in enumerate(PARTICLES):
            real_vals = metrics[(metrics["particles"] == particles) & (metrics["source"] == "real")][metric].to_numpy()
            sim_vals = metrics[(metrics["particles"] == particles) & (metrics["source"] == "sim")][metric].to_numpy()
            positions.extend([i * 3.0 + 1.0, i * 3.0 + 1.8])
            labels.extend([f"{particles}\nreal", f"{particles}\nsim"])
            data.extend([real_vals, sim_vals])
        ax.boxplot(data, positions=positions, widths=0.55, showfliers=True)
        ax.set_xticks(positions)
        ax.set_xticklabels(labels, rotation=45, ha="right")
        ax.set_ylabel(ylabel)
        ax.grid(True, axis="y", alpha=0.35)
        fig.tight_layout()
        fig.savefig(output_dir / filename, dpi=300)
        plt.close(fig)


def plot_tost_summary(tost: pd.DataFrame, output_dir: Path) -> None:
    focus_metrics = ["median_position_cm", "p95_position_cm", "median_abs_yaw_deg", "p95_abs_yaw_deg"]
    frame = tost[tost["metric"].isin(focus_metrics)].copy()
    fig, axes = plt.subplots(len(focus_metrics), 1, figsize=(9, 9), sharex=True)
    for ax, metric in zip(axes, focus_metrics):
        sub = frame[frame["metric"] == metric].sort_values("particles")
        margin = float(sub["margin"].iloc[0])
        x = np.arange(len(sub))
        ax.axhspan(-margin, margin, color="#d7f0d2", alpha=0.55)
        ax.errorbar(
            x,
            sub["mean_diff_sim_minus_real"],
            yerr=[
                sub["mean_diff_sim_minus_real"] - sub["ci90_low"],
                sub["ci90_high"] - sub["mean_diff_sim_minus_real"],
            ],
            fmt="o",
            color="#1f4e79",
            capsize=4,
        )
        ax.axhline(0.0, color="black", linewidth=0.8)
        ax.set_ylabel(metric.replace("_", " "))
        ax.grid(True, axis="y", alpha=0.3)
        for xi, ok in zip(x, sub["equivalent_tost"]):
            ax.text(xi, ax.get_ylim()[1], "pass" if ok else "fail", ha="center", va="top", fontsize=8)
    axes[-1].set_xticks(np.arange(len(PARTICLES)))
    axes[-1].set_xticklabels([str(p) for p in PARTICLES])
    axes[-1].set_xlabel("particles")
    fig.tight_layout()
    fig.savefig(output_dir / "equivalence_tost_ci_summary.png", dpi=300)
    plt.close(fig)


def first_available_tf(real_root: Path) -> Tuple[np.ndarray, np.ndarray]:
    for particles in PARTICLES:
        tf = read_map_world_tf(real_bag_for_particles(real_root, particles))
        if tf is not None:
            return tf
    raise RuntimeError("No map->world transform found in real bags")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[2]
    default_sim_root = (
        repo_root / "f1tenth_localization" / "Benchmark" / "Matlab" /
        "sim_benchmark" / "optitrack_map_roi_particle_match_5laps"
    )
    default_real_root = (
        repo_root / "f1tenth_localization" / "Benchmark" / "bags" /
        "OptitrackBags" / "AMCL"
    )
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim-root", type=Path, default=default_sim_root)
    parser.add_argument("--real-root", type=Path, default=default_real_root)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--min-speed-mps", type=float, default=0.2)
    parser.add_argument("--roi-min-x", type=float, default=0.0)
    parser.add_argument("--roi-max-x", type=float, default=8.0)
    parser.add_argument("--roi-min-y", type=float, default=-1.4)
    parser.add_argument("--roi-max-y", type=float, default=math.inf)
    parser.add_argument("--roi2-min-x", type=float, default=math.nan)
    parser.add_argument("--roi2-max-x", type=float, default=math.nan)
    parser.add_argument("--roi2-min-y", type=float, default=math.nan)
    parser.add_argument("--roi2-max-y", type=float, default=math.nan)
    parser.add_argument("--runs", default="1,2,3,4,5")
    parser.add_argument("--bootstrap-samples", type=int, default=20_000)
    parser.add_argument("--real-tf-correction-dx-m", type=float, default=0.0)
    parser.add_argument("--real-tf-correction-dy-m", type=float, default=0.0)
    parser.add_argument("--real-tf-correction-dyaw-deg", type=float, default=0.0)
    parser.add_argument("--real-start-calibration-s", type=float, default=3.0)
    parser.add_argument("--sim-max-amcl-age-s", type=float, default=math.inf)
    parser.add_argument("--sim-min-progress-m", type=float, default=-math.inf)
    parser.add_argument("--sim-min-roi-samples", type=int, default=10)
    parser.add_argument("--estimator", choices=("amcl", "ekf"), default="amcl")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    output_dir = args.output_dir or args.sim_root
    output_dir.mkdir(parents=True, exist_ok=True)
    rois = [(args.roi_min_x, args.roi_max_x, args.roi_min_y, args.roi_max_y)]
    roi2_values = [args.roi2_min_x, args.roi2_max_x, args.roi2_min_y, args.roi2_max_y]
    if any(math.isfinite(v) for v in roi2_values):
        if not all(math.isfinite(v) for v in roi2_values):
            raise ValueError("All roi2 bounds must be finite when a second ROI is provided")
        rois.append((args.roi2_min_x, args.roi2_max_x, args.roi2_min_y, args.roi2_max_y))
    roi = roi_bounds(rois)
    runs = [int(v.strip()) for v in args.runs.split(",") if v.strip()]
    rng = np.random.default_rng(7)
    real_tf_correction = (
        args.real_tf_correction_dx_m,
        args.real_tf_correction_dy_m,
        math.radians(args.real_tf_correction_dyaw_deg),
    )

    fallback_tf = first_available_tf(args.real_root)
    metric_rows: List[Dict[str, float | int | str]] = []
    all_samples: List[pd.DataFrame] = []
    tf_fallback_rows: List[Dict[str, int]] = []

    for particles in PARTICLES:
        real_metrics, real_samples, tf_fallback = load_real_unit_metrics(
            args.real_root, particles, rois, args.min_speed_mps, fallback_tf,
            real_tf_correction, args.real_start_calibration_s, args.estimator)
        sim_metrics, sim_samples = load_sim_unit_metrics(
            args.sim_root, particles, rois, args.min_speed_mps, runs,
            args.sim_max_amcl_age_s, args.sim_min_progress_m, args.sim_min_roi_samples,
            args.estimator)
        metric_rows.extend(real_metrics)
        metric_rows.extend(sim_metrics)
        all_samples.extend([real_samples, sim_samples])
        tf_fallback_rows.append({"particles": particles, "used_tf_fallback": tf_fallback})

    metrics = pd.DataFrame(metric_rows)
    samples = pd.concat(all_samples, ignore_index=True)
    spatial = spatial_bin_summary(samples, roi)

    tost_rows: List[Dict[str, float | int | str]] = []
    for particles in PARTICLES:
        real = metrics[(metrics["particles"] == particles) & (metrics["source"] == "real")]
        sim = metrics[(metrics["particles"] == particles) & (metrics["source"] == "sim")]
        for metric, margin in METRIC_MARGINS.items():
            real_values = real[metric].to_numpy(dtype=float)
            sim_values = sim[metric].to_numpy(dtype=float)
            result = welch_tost(sim_values, real_values, margin)
            boot_low, boot_high = bootstrap_ci(
                sim_values, real_values, rng, n_boot=args.bootstrap_samples)
            result.update({
                "particles": particles,
                "metric": metric,
                "bootstrap90_low": boot_low,
                "bootstrap90_high": boot_high,
                "equivalent_bootstrap90": int(boot_low > -margin and boot_high < margin),
                "real_mean": float(np.mean(real_values)),
                "sim_mean": float(np.mean(sim_values)),
                "real_median_of_units": float(np.median(real_values)),
                "sim_median_of_units": float(np.median(sim_values)),
            })
            tost_rows.append(result)
    tost = pd.DataFrame(tost_rows)

    metrics.to_csv(output_dir / "Equivalence_LapRun_Metrics.csv", index=False)
    samples.to_csv(output_dir / "Equivalence_ROI_Samples.csv", index=False)
    tost.to_csv(output_dir / "Equivalence_TOST_Summary.csv", index=False)
    spatial.to_csv(output_dir / "Equivalence_Spatial_Bin_Summary.csv", index=False)
    pd.DataFrame(tf_fallback_rows).to_csv(output_dir / "Equivalence_TF_Fallback_Summary.csv", index=False)

    plot_metric_boxes(metrics, output_dir)
    plot_tost_summary(tost, output_dir)

    print(f"Estimator: {args.estimator}")
    print("Wrote:")
    for name in [
        "Equivalence_LapRun_Metrics.csv",
        "Equivalence_TOST_Summary.csv",
        "Equivalence_Spatial_Bin_Summary.csv",
        "equivalence_tost_ci_summary.png",
    ]:
        print(output_dir / name)

    position = tost[tost["metric"].isin(["median_position_cm", "p95_position_cm"])]
    print("\nPosition equivalence summary:")
    print(position[[
        "particles", "metric", "mean_diff_sim_minus_real", "margin",
        "ci90_low", "ci90_high", "tost_p", "equivalent_tost",
        "equivalent_bootstrap90",
    ]].to_string(index=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
