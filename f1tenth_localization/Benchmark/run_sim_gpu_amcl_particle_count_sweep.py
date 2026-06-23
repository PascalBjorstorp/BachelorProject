#!/usr/bin/env python3
"""Run fixed-particle GPU AMCL particle-count sweep in simulation."""

from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import statistics
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Sequence


def percentile(values: Sequence[float], q: float) -> float:
    finite = sorted(v for v in values if math.isfinite(v))
    if not finite:
        return math.nan
    if len(finite) == 1:
        return finite[0]
    pos = (len(finite) - 1) * q
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return finite[lo]
    weight = pos - lo
    return finite[lo] * (1.0 - weight) + finite[hi] * weight


def mean_or_nan(values: Sequence[float]) -> float:
    finite = [v for v in values if math.isfinite(v)]
    return statistics.fmean(finite) if finite else math.nan


def median_or_nan(values: Sequence[float]) -> float:
    finite = [v for v in values if math.isfinite(v)]
    return statistics.median(finite) if finite else math.nan


def min_or_nan(values: Sequence[float]) -> float:
    finite = [v for v in values if math.isfinite(v)]
    return min(finite) if finite else math.nan


def max_or_nan(values: Sequence[float]) -> float:
    finite = [v for v in values if math.isfinite(v)]
    return max(finite) if finite else math.nan


def float_field(row: Dict[str, str], name: str) -> float:
    try:
        value = float(row.get(name, ""))
    except (TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def parse_particles(text: str) -> List[int]:
    text = text.strip()
    if ":" in text:
        parts = [int(v.strip()) for v in text.split(":")]
        if len(parts) != 3:
            raise ValueError("range syntax is start:step:end")
        start, step, end = parts
        if step <= 0:
            raise ValueError("particle step must be positive")
        return list(range(start, end + 1, step))
    values = [int(v.strip()) for v in text.split(",") if v.strip()]
    if not values:
        raise ValueError("At least one particle count is required")
    return values


def condition_name(particles: int) -> str:
    return f"particles_{particles:04d}"


def latest_pipeline_csv(run_root: Path) -> Path | None:
    matches = sorted(run_root.glob("AMCL_benchmark/pipeline/pipeline_latency_*.csv"))
    if matches:
        return matches[-1]
    matches = sorted(run_root.glob("AMCL_benchmark/pipeline/Pipeline_*.csv"))
    return matches[-1] if matches else None


def system_csv(run_root: Path, name: str) -> Path:
    return run_root / "AMCL_benchmark" / "system" / name


def benchmark_csv(run_root: Path) -> Path:
    return run_root / "AMCL_benchmark" / "AMCL_benchmark.csv"


def status_json(run_root: Path) -> Path:
    return run_root / "AMCL_benchmark" / "AMCL_benchmark_status.json"


def load_pipeline_csv(path: Path, skip_first_sec: float) -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    with path.open(newline="") as handle:
        raw_rows = list(csv.DictReader(handle))
    if not raw_rows:
        return rows

    first_wall = float_field(raw_rows[0], "wall_time_ns")
    for raw in raw_rows:
        wall_ns = float_field(raw, "wall_time_ns")
        t_rel = (wall_ns - first_wall) * 1e-9 if math.isfinite(wall_ns) and math.isfinite(first_wall) else math.nan
        if math.isfinite(t_rel) and t_rel < skip_first_sec:
            continue
        scan_to_amcl = float_field(raw, "scan_to_amcl_ms")
        if not math.isfinite(scan_to_amcl):
            continue
        rows.append({
            "t_rel_s": t_rel,
            "scan_to_amcl_ms": scan_to_amcl,
            "amcl_processing_ms": float_field(raw, "amcl_processing_ms"),
            "amcl_pose_compute_ms": float_field(raw, "amcl_pose_compute_ms"),
            "cpu_to_gpu_scan_ms": float_field(raw, "cpu_to_gpu_scan_ms"),
            "gpu_to_cpu_particles_ms": float_field(raw, "gpu_to_cpu_particles_ms"),
            "gpu_to_cpu_weights_ms": float_field(raw, "gpu_to_cpu_weights_ms"),
            "cpu_gpu_transfer_total_ms": float_field(raw, "cpu_gpu_transfer_total_ms"),
            "cpu_to_gpu_scan_bytes": float_field(raw, "cpu_to_gpu_scan_bytes"),
            "gpu_to_cpu_particles_bytes": float_field(raw, "gpu_to_cpu_particles_bytes"),
            "gpu_to_cpu_weights_bytes": float_field(raw, "gpu_to_cpu_weights_bytes"),
            "amcl_particle_count": float_field(raw, "amcl_particle_count"),
            "amcl_predict_ms": float_field(raw, "amcl_predict_ms"),
            "amcl_sensor_model_ms": float_field(raw, "amcl_sensor_model_ms"),
            "amcl_normalize_ms": float_field(raw, "amcl_normalize_ms"),
            "amcl_scan_confidence_ms": float_field(raw, "amcl_scan_confidence_ms"),
            "amcl_update_weights_total_ms": float_field(raw, "amcl_update_weights_total_ms"),
            "amcl_cluster_estimate_ms": float_field(raw, "amcl_cluster_estimate_ms"),
            "amcl_resample_ms": float_field(raw, "amcl_resample_ms"),
            "amcl_kld_target_ms": float_field(raw, "amcl_kld_target_ms"),
            "amcl_full_compute_ms": float_field(raw, "amcl_full_compute_ms"),
            "amcl_callback_to_pose_publish_ms": float_field(raw, "amcl_callback_to_pose_publish_ms"),
            "amcl_pose_published": float_field(raw, "amcl_pose_published"),
            "amcl_cluster_weight": float_field(raw, "amcl_cluster_weight"),
            "amcl_raycast_setup_ms": float_field(raw, "amcl_raycast_setup_ms"),
            "amcl_raycast_score_ms": float_field(raw, "amcl_raycast_score_ms"),
            "amcl_raycast_correction_ms": float_field(raw, "amcl_raycast_correction_ms"),
        })
    return rows


def load_system_usage_csv(path: Path, skip_first_sec: float, fields: Sequence[str]) -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    if not path.exists():
        return rows
    with path.open(newline="") as handle:
        raw_rows = list(csv.DictReader(handle))
    if not raw_rows:
        return rows

    first_wall = float_field(raw_rows[0], "monotonic_time_ns")
    for raw in raw_rows:
        wall_ns = float_field(raw, "monotonic_time_ns")
        t_rel = (wall_ns - first_wall) * 1e-9 if math.isfinite(wall_ns) and math.isfinite(first_wall) else math.nan
        if math.isfinite(t_rel) and t_rel < skip_first_sec:
            continue
        row = {"t_rel_s": t_rel}
        for field in fields:
            row[field] = float_field(raw, field)
        rows.append(row)
    return rows


def load_node_cpu_csv(path: Path, skip_first_sec: float, node_name: str = "gpu_amcl_cpp") -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    if not path.exists():
        return rows
    with path.open(newline="") as handle:
        raw_rows = list(csv.DictReader(handle))
    if not raw_rows:
        return rows

    first_wall = float_field(raw_rows[0], "monotonic_time_ns")
    for raw in raw_rows:
        if raw.get("node_name") != node_name:
            continue
        wall_ns = float_field(raw, "monotonic_time_ns")
        t_rel = (wall_ns - first_wall) * 1e-9 if math.isfinite(wall_ns) and math.isfinite(first_wall) else math.nan
        if math.isfinite(t_rel) and t_rel < skip_first_sec:
            continue
        rows.append({
            "t_rel_s": t_rel,
            "cpu_percent": float_field(raw, "cpu_percent"),
        })
    return rows


def load_cache_csv(path: Path, skip_first_sec: float, node_name: str = "gpu_amcl_cpp") -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    if not path.exists():
        return rows
    with path.open(newline="") as handle:
        raw_rows = list(csv.DictReader(handle))
    if not raw_rows:
        return rows

    first_wall = float_field(raw_rows[0], "monotonic_time_ns")
    for raw in raw_rows:
        if raw.get("node_name") != node_name:
            continue
        wall_ns = float_field(raw, "monotonic_time_ns")
        t_rel = (wall_ns - first_wall) * 1e-9 if math.isfinite(wall_ns) and math.isfinite(first_wall) else math.nan
        if math.isfinite(t_rel) and t_rel < skip_first_sec:
            continue
        rows.append({
            "t_rel_s": t_rel,
            "cache_references": float_field(raw, "cache_references"),
            "cache_misses": float_field(raw, "cache_misses"),
            "cache_reference_delta": float_field(raw, "cache_reference_delta"),
            "cache_miss_delta": float_field(raw, "cache_miss_delta"),
            "cache_miss_rate_percent": float_field(raw, "cache_miss_rate_percent"),
            "cache_hit_rate_percent": float_field(raw, "cache_hit_rate_percent"),
            "counters_valid": 1.0 if raw.get("counters_valid", "").lower() == "true" else 0.0,
        })
    return rows


def load_memory_controller_csv(path: Path, skip_first_sec: float) -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    if not path.exists():
        return rows
    with path.open(newline="") as handle:
        raw_rows = list(csv.DictReader(handle))
    if not raw_rows:
        return rows

    first_wall = float_field(raw_rows[0], "monotonic_time_ns")
    for raw in raw_rows:
        wall_ns = float_field(raw, "monotonic_time_ns")
        t_rel = (wall_ns - first_wall) * 1e-9 if math.isfinite(wall_ns) and math.isfinite(first_wall) else math.nan
        if math.isfinite(t_rel) and t_rel < skip_first_sec:
            continue
        rows.append({
            "t_rel_s": t_rel,
            "emc_util_percent": float_field(raw, "emc_util_percent"),
            "emc_freq_mhz": float_field(raw, "emc_freq_mhz"),
            "emc_peak_bandwidth_mib_s": float_field(raw, "emc_peak_bandwidth_mib_s"),
            "emc_estimated_bandwidth_mib_s": float_field(raw, "emc_estimated_bandwidth_mib_s"),
            "valid": 1.0 if raw.get("valid", "").lower() == "true" else 0.0,
        })
    return rows


def load_benchmark_csv(path: Path, skip_first_sec: float) -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    with path.open(newline="") as handle:
        raw_rows = list(csv.DictReader(handle))
    if not raw_rows:
        return rows

    first_wall = float_field(raw_rows[0], "wall_time_ns")
    for raw in raw_rows:
        wall_ns = float_field(raw, "wall_time_ns")
        t_rel = (wall_ns - first_wall) * 1e-9 if math.isfinite(wall_ns) and math.isfinite(first_wall) else math.nan
        if math.isfinite(t_rel) and t_rel < skip_first_sec:
            continue

        gt_x = float_field(raw, "gt_x")
        gt_y = float_field(raw, "gt_y")
        gt_yaw = float_field(raw, "gt_yaw")
        amcl_x = float_field(raw, "amcl_x")
        amcl_y = float_field(raw, "amcl_y")
        amcl_yaw = float_field(raw, "amcl_yaw")
        if not all(math.isfinite(v) for v in (gt_x, gt_y, gt_yaw, amcl_x, amcl_y, amcl_yaw)):
            continue
        row = {
            "t_rel_s": t_rel,
            "amcl_error_xy_m": math.hypot(amcl_x - gt_x, amcl_y - gt_y),
            "amcl_abs_yaw_error_rad": abs(angle_diff(amcl_yaw, gt_yaw)),
        }
        for name in (
            "ekf_cov_x", "ekf_cov_y", "ekf_cov_yaw",
            "amcl_cov_x", "amcl_cov_y", "amcl_cov_yaw",
            "odom_cov_x", "odom_cov_y", "odom_cov_yaw",
        ):
            row[name] = float_field(raw, name)
        rows.append(row)
    return rows


def run_duration(rows: Sequence[Dict[str, float]]) -> float:
    times = [r["t_rel_s"] for r in rows if math.isfinite(r["t_rel_s"])]
    if len(times) < 2:
        return math.nan
    return max(times) - min(times)


def covariance_stats(prefix: str, rows: Sequence[Dict[str, float]]) -> Dict[str, object]:
    cov_x = [r[f"{prefix}_cov_x"] for r in rows if math.isfinite(r.get(f"{prefix}_cov_x", math.nan))]
    cov_y = [r[f"{prefix}_cov_y"] for r in rows if math.isfinite(r.get(f"{prefix}_cov_y", math.nan))]
    cov_yaw = [r[f"{prefix}_cov_yaw"] for r in rows if math.isfinite(r.get(f"{prefix}_cov_yaw", math.nan))]
    pos_sigma = [
        math.sqrt(max(0.0, x) + max(0.0, y))
        for x, y in zip(cov_x, cov_y)
        if math.isfinite(x) and math.isfinite(y)
    ]
    yaw_sigma = [math.sqrt(max(0.0, v)) for v in cov_yaw if math.isfinite(v)]
    return {
        f"{prefix}_cov_x_median": median_or_nan(cov_x),
        f"{prefix}_cov_y_median": median_or_nan(cov_y),
        f"{prefix}_cov_yaw_median": median_or_nan(cov_yaw),
        f"{prefix}_cov_x_min": min_or_nan(cov_x),
        f"{prefix}_cov_x_max": max_or_nan(cov_x),
        f"{prefix}_cov_y_min": min_or_nan(cov_y),
        f"{prefix}_cov_y_max": max_or_nan(cov_y),
        f"{prefix}_cov_yaw_min": min_or_nan(cov_yaw),
        f"{prefix}_cov_yaw_max": max_or_nan(cov_yaw),
        f"{prefix}_pos_sigma_median_m": median_or_nan(pos_sigma),
        f"{prefix}_yaw_sigma_median_deg": math.degrees(median_or_nan(yaw_sigma)),
    }


def summarize(particles: int,
              run_name: str,
              pipeline_rows: Sequence[Dict[str, float]],
              benchmark_rows: Sequence[Dict[str, float]],
              system_rows: Sequence[Dict[str, float]],
              gpu_rows: Sequence[Dict[str, float]],
              node_cpu_rows: Sequence[Dict[str, float]],
              memory_rows: Sequence[Dict[str, float]],
              memory_controller_rows: Sequence[Dict[str, float]],
              cache_rows: Sequence[Dict[str, float]],
              status: Dict[str, object]) -> Dict[str, object]:
    latency = [r["scan_to_amcl_ms"] for r in pipeline_rows if math.isfinite(r["scan_to_amcl_ms"])]
    processing = [
        r["amcl_processing_ms"] for r in pipeline_rows
        if math.isfinite(r.get("amcl_processing_ms", math.nan)) and r["amcl_processing_ms"] >= 0.0
    ]
    pose_compute = [
        r["amcl_pose_compute_ms"] for r in pipeline_rows
        if math.isfinite(r.get("amcl_pose_compute_ms", math.nan)) and r["amcl_pose_compute_ms"] >= 0.0
    ]
    cpu_to_gpu_scan = [
        r["cpu_to_gpu_scan_ms"] for r in pipeline_rows
        if math.isfinite(r.get("cpu_to_gpu_scan_ms", math.nan)) and r["cpu_to_gpu_scan_ms"] >= 0.0
    ]
    gpu_to_cpu_particles = [
        r["gpu_to_cpu_particles_ms"] for r in pipeline_rows
        if math.isfinite(r.get("gpu_to_cpu_particles_ms", math.nan)) and r["gpu_to_cpu_particles_ms"] >= 0.0
    ]
    gpu_to_cpu_weights = [
        r["gpu_to_cpu_weights_ms"] for r in pipeline_rows
        if math.isfinite(r.get("gpu_to_cpu_weights_ms", math.nan)) and r["gpu_to_cpu_weights_ms"] >= 0.0
    ]
    transfer_total = [
        r["cpu_gpu_transfer_total_ms"] for r in pipeline_rows
        if math.isfinite(r.get("cpu_gpu_transfer_total_ms", math.nan)) and r["cpu_gpu_transfer_total_ms"] >= 0.0
    ]
    scan_upload_bytes = [
        r["cpu_to_gpu_scan_bytes"] for r in pipeline_rows
        if math.isfinite(r.get("cpu_to_gpu_scan_bytes", math.nan)) and r["cpu_to_gpu_scan_bytes"] >= 0.0
    ]
    particle_download_bytes = [
        r["gpu_to_cpu_particles_bytes"] for r in pipeline_rows
        if math.isfinite(r.get("gpu_to_cpu_particles_bytes", math.nan)) and r["gpu_to_cpu_particles_bytes"] >= 0.0
    ]
    weight_download_bytes = [
        r["gpu_to_cpu_weights_bytes"] for r in pipeline_rows
        if math.isfinite(r.get("gpu_to_cpu_weights_bytes", math.nan)) and r["gpu_to_cpu_weights_bytes"] >= 0.0
    ]
    particle_counts = [
        r["amcl_particle_count"] for r in pipeline_rows
        if math.isfinite(r["amcl_particle_count"]) and r["amcl_particle_count"] >= 0
    ]
    xy_errors = [r["amcl_error_xy_m"] for r in benchmark_rows if math.isfinite(r["amcl_error_xy_m"])]
    yaw_errors = [
        r["amcl_abs_yaw_error_rad"] for r in benchmark_rows
        if math.isfinite(r["amcl_abs_yaw_error_rad"])
    ]
    system_cpu = [
        r["cpu_short_window_percent"] for r in system_rows
        if math.isfinite(r.get("cpu_short_window_percent", math.nan)) and r["cpu_short_window_percent"] >= 0.0
    ]
    gpu_usage = [
        r["gpu_percent"] for r in gpu_rows
        if math.isfinite(r.get("gpu_percent", math.nan)) and r["gpu_percent"] >= 0.0
    ]
    gpu_amcl_cpu = [
        r["cpu_percent"] for r in node_cpu_rows
        if math.isfinite(r.get("cpu_percent", math.nan)) and r["cpu_percent"] >= 0.0
    ]
    cpu_mem_used = [
        r["cpu_mem_used_mib"] for r in memory_rows
        if math.isfinite(r.get("cpu_mem_used_mib", math.nan)) and r["cpu_mem_used_mib"] >= 0.0
    ]
    cpu_mem_available = [
        r["cpu_mem_available_mib"] for r in memory_rows
        if math.isfinite(r.get("cpu_mem_available_mib", math.nan)) and r["cpu_mem_available_mib"] >= 0.0
    ]
    cpu_page_cache = [
        r["cpu_page_cache_mib"] for r in memory_rows
        if math.isfinite(r.get("cpu_page_cache_mib", math.nan)) and r["cpu_page_cache_mib"] >= 0.0
    ]
    cpu_cached = [
        r["cpu_cached_mib"] for r in memory_rows
        if math.isfinite(r.get("cpu_cached_mib", math.nan)) and r["cpu_cached_mib"] >= 0.0
    ]
    emc_util = [
        r["emc_util_percent"] for r in memory_controller_rows
        if math.isfinite(r.get("emc_util_percent", math.nan)) and r["emc_util_percent"] >= 0.0
    ]
    emc_freq = [
        r["emc_freq_mhz"] for r in memory_controller_rows
        if math.isfinite(r.get("emc_freq_mhz", math.nan)) and r["emc_freq_mhz"] >= 0.0
    ]
    emc_bandwidth = [
        r["emc_estimated_bandwidth_mib_s"] for r in memory_controller_rows
        if math.isfinite(r.get("emc_estimated_bandwidth_mib_s", math.nan)) and
        r["emc_estimated_bandwidth_mib_s"] >= 0.0
    ]
    emc_valid = [
        r["valid"] for r in memory_controller_rows
        if math.isfinite(r.get("valid", math.nan))
    ]
    cache_reference_delta = [
        r["cache_reference_delta"] for r in cache_rows
        if math.isfinite(r.get("cache_reference_delta", math.nan)) and r["cache_reference_delta"] >= 0.0
    ]
    cache_miss_delta = [
        r["cache_miss_delta"] for r in cache_rows
        if math.isfinite(r.get("cache_miss_delta", math.nan)) and r["cache_miss_delta"] >= 0.0
    ]
    cache_valid = [
        r["counters_valid"] for r in cache_rows
        if math.isfinite(r.get("counters_valid", math.nan))
    ]
    cache_hit_rate = [
        r["cache_hit_rate_percent"] for r in cache_rows
        if math.isfinite(r.get("cache_hit_rate_percent", math.nan)) and
        r["cache_hit_rate_percent"] >= 0.0
    ]
    cache_duration_s = run_duration(cache_rows)
    cache_reference_total = sum(cache_reference_delta)
    cache_miss_total = sum(cache_miss_delta)
    cache_miss_rate = (
        100.0 * cache_miss_total / cache_reference_total
        if cache_reference_total > 0.0 else math.nan
    )
    stage_series = {
        name: [
            r[name] for r in pipeline_rows
            if math.isfinite(r.get(name, math.nan)) and r[name] >= 0.0
        ]
        for name in (
            "amcl_predict_ms",
            "amcl_sensor_model_ms",
            "amcl_normalize_ms",
            "amcl_scan_confidence_ms",
            "amcl_update_weights_total_ms",
            "amcl_cluster_estimate_ms",
            "amcl_resample_ms",
            "amcl_kld_target_ms",
            "amcl_full_compute_ms",
            "amcl_callback_to_pose_publish_ms",
            "amcl_cluster_weight",
            "amcl_raycast_setup_ms",
            "amcl_raycast_score_ms",
            "amcl_raycast_correction_ms",
        )
    }
    row: Dict[str, object] = {
        "particles": particles,
        "condition": condition_name(particles),
        "label": f"{particles}",
        "run": run_name,
        "status_reason": status.get("reason", ""),
        "status_laps": status.get("laps", ""),
        "n_latency_samples": len(latency),
        "n_error_samples": len(xy_errors),
        "duration_s": run_duration(pipeline_rows),
        "scan_to_amcl_mean_ms": mean_or_nan(latency),
        "scan_to_amcl_median_ms": median_or_nan(latency),
        "scan_to_amcl_p95_ms": percentile(latency, 0.95),
        "scan_to_amcl_max_ms": max(latency) if latency else math.nan,
        "amcl_processing_mean_ms": mean_or_nan(processing),
        "amcl_processing_median_ms": median_or_nan(processing),
        "amcl_processing_p95_ms": percentile(processing, 0.95),
        "amcl_processing_max_ms": max(processing) if processing else math.nan,
        "amcl_pose_compute_mean_ms": mean_or_nan(pose_compute),
        "amcl_pose_compute_median_ms": median_or_nan(pose_compute),
        "amcl_pose_compute_p95_ms": percentile(pose_compute, 0.95),
        "amcl_pose_compute_max_ms": max(pose_compute) if pose_compute else math.nan,
        "cpu_to_gpu_scan_median_ms": median_or_nan(cpu_to_gpu_scan),
        "cpu_to_gpu_scan_p95_ms": percentile(cpu_to_gpu_scan, 0.95),
        "gpu_to_cpu_particles_median_ms": median_or_nan(gpu_to_cpu_particles),
        "gpu_to_cpu_particles_p95_ms": percentile(gpu_to_cpu_particles, 0.95),
        "gpu_to_cpu_weights_median_ms": median_or_nan(gpu_to_cpu_weights),
        "gpu_to_cpu_weights_p95_ms": percentile(gpu_to_cpu_weights, 0.95),
        "cpu_gpu_transfer_total_median_ms": median_or_nan(transfer_total),
        "cpu_gpu_transfer_total_p95_ms": percentile(transfer_total, 0.95),
        "cpu_to_gpu_scan_bytes_median": median_or_nan(scan_upload_bytes),
        "gpu_to_cpu_particles_bytes_median": median_or_nan(particle_download_bytes),
        "gpu_to_cpu_weights_bytes_median": median_or_nan(weight_download_bytes),
        "amcl_position_error_median_m": median_or_nan(xy_errors),
        "amcl_position_error_p95_m": percentile(xy_errors, 0.95),
        "amcl_yaw_error_median_rad": median_or_nan(yaw_errors),
        "amcl_yaw_error_p95_rad": percentile(yaw_errors, 0.95),
        "system_cpu_mean_percent": mean_or_nan(system_cpu),
        "system_cpu_p95_percent": percentile(system_cpu, 0.95),
        "gpu_mean_percent": mean_or_nan(gpu_usage),
        "gpu_p95_percent": percentile(gpu_usage, 0.95),
        "gpu_amcl_cpu_mean_percent": mean_or_nan(gpu_amcl_cpu),
        "gpu_amcl_cpu_p95_percent": percentile(gpu_amcl_cpu, 0.95),
        "cpu_mem_used_median_mib": median_or_nan(cpu_mem_used),
        "cpu_mem_used_max_mib": max_or_nan(cpu_mem_used),
        "cpu_mem_available_min_mib": min_or_nan(cpu_mem_available),
        "cpu_page_cache_median_mib": median_or_nan(cpu_page_cache),
        "cpu_page_cache_max_mib": max_or_nan(cpu_page_cache),
        "cpu_page_cache_delta_mib": (
            max(cpu_page_cache) - min(cpu_page_cache) if cpu_page_cache else math.nan
        ),
        "cpu_cached_median_mib": median_or_nan(cpu_cached),
        "cpu_cached_max_mib": max_or_nan(cpu_cached),
        "cpu_cached_delta_mib": (
            max(cpu_cached) - min(cpu_cached) if cpu_cached else math.nan
        ),
        "emc_util_median_percent": median_or_nan(emc_util),
        "emc_util_p95_percent": percentile(emc_util, 0.95),
        "emc_freq_median_mhz": median_or_nan(emc_freq),
        "emc_estimated_bandwidth_median_mib_s": median_or_nan(emc_bandwidth),
        "emc_estimated_bandwidth_p95_mib_s": percentile(emc_bandwidth, 0.95),
        "emc_valid_fraction": mean_or_nan(emc_valid),
        "gpu_amcl_cache_reference_delta_total": cache_reference_total,
        "gpu_amcl_cache_miss_delta_total": cache_miss_total,
        "gpu_amcl_cache_miss_rate_percent": cache_miss_rate,
        "gpu_amcl_cache_hit_rate_median_percent": median_or_nan(cache_hit_rate),
        "gpu_amcl_cache_hit_rate_p95_percent": percentile(cache_hit_rate, 0.95),
        "gpu_amcl_cache_misses_per_second": (
            cache_miss_total / cache_duration_s
            if math.isfinite(cache_duration_s) and cache_duration_s > 0.0 else math.nan
        ),
        "gpu_amcl_cache_counters_valid_fraction": mean_or_nan(cache_valid),
        "particle_mean": mean_or_nan(particle_counts),
        "particle_min": min(particle_counts) if particle_counts else math.nan,
        "particle_max": max(particle_counts) if particle_counts else math.nan,
    }
    for name, values in stage_series.items():
        row[f"{name}_median"] = median_or_nan(values)
        row[f"{name}_p95"] = percentile(values, 0.95)
        row[f"{name}_max"] = max(values) if values else math.nan
    for prefix in ("ekf", "amcl", "odom"):
        row.update(covariance_stats(prefix, benchmark_rows))
    return row


def write_csv(path: Path, rows: Sequence[Dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    keys: List[str] = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def read_status(path: Path) -> Dict[str, object]:
    if not path.exists():
        return {}
    try:
        with path.open() as handle:
            return json.load(handle)
    except (OSError, json.JSONDecodeError):
        return {}


def run_root_for(output_root: Path, particles: int, run_index: int) -> Path:
    return output_root / condition_name(particles) / f"run_{run_index:02d}"


def build_command(args: argparse.Namespace, particles: int, run_index: int) -> List[str]:
    benchmark_script = Path(__file__).with_name("run_sim_gpu_amcl_benchmark.py")
    run_output_root = run_root_for(args.output_root, particles, run_index)
    cmd = [
        sys.executable,
        str(benchmark_script),
        "--output-root", str(run_output_root),
        "--laps", str(args.laps),
        "--max-duration-sec", str(args.max_duration_sec),
        "--process-timeout-sec", str(args.process_timeout_sec),
        "--map-file", str(args.map_file),
        "--trajectory-file", str(args.trajectory_file),
        "--initial-pose-x", str(args.initial_pose_x),
        "--initial-pose-y", str(args.initial_pose_y),
        "--headless",
        "--realistic-plant",
        "--sim-odom-source", args.sim_odom_source,
        "--sim-drive-input-mode", args.sim_drive_input_mode,
        "--sim-drive-uses-acceleration-field",
        "--control-start-delay-sec", str(args.control_start_delay_sec),
        "--mpc-raceline-speed-margin", str(args.mpc_raceline_speed_margin),
        "--cloud-publish-rate", str(args.cloud_publish_rate),
        "--system-monitor-cpu-sample-hz", str(args.system_monitor_cpu_sample_hz),
        "--system-monitor-gpu-sample-hz", str(args.system_monitor_gpu_sample_hz),
        "--system-monitor-csv-log-hz", str(args.system_monitor_csv_log_hz),
        "--system-monitor-long-csv-log-hz", str(args.system_monitor_long_csv_log_hz),
        "--system-monitor-memory-log-hz", str(args.system_monitor_memory_log_hz),
        "--system-monitor-memory-controller-log-hz",
        str(args.system_monitor_memory_controller_log_hz),
        "--system-monitor-emc-peak-bandwidth-mib-s",
        str(args.system_monitor_emc_peak_bandwidth_mib_s),
        "--amcl-num-particles", str(particles),
        "--amcl-min-particles", str(particles),
        "--amcl-max-particles", str(particles),
        "--amcl-max-beams", str(args.max_beams),
        "--amcl-update-min-d", str(args.update_min_d),
        "--amcl-update-min-a", str(args.update_min_a),
        "--amcl-likelihood-scale", str(args.likelihood_scale),
        "--amcl-alpha1", str(args.amcl_alpha1),
        "--amcl-alpha2", str(args.amcl_alpha2),
        "--amcl-alpha3", str(args.amcl_alpha3),
        "--amcl-alpha4", str(args.amcl_alpha4),
        "--amcl-z-hit", str(args.amcl_z_hit),
        "--amcl-z-rand", str(args.amcl_z_rand),
        "--amcl-sigma-hit", str(args.amcl_sigma_hit),
        "--amcl-resample-threshold", str(args.amcl_resample_threshold),
        "--amcl-cluster-publish-min-weight", str(args.cluster_publish_min_weight),
        "--ekf-process-noise-scale", str(args.ekf_process_noise_scale),
        "--no-debug-pre-resample-particles",
    ]
    if args.initial_pose_yaw is not None:
        cmd.extend(["--initial-pose-yaw", str(args.initial_pose_yaw)])
    if args.use_cluster_estimate:
        cmd.append("--amcl-use-cluster-estimate")
    else:
        cmd.append("--no-amcl-use-cluster-estimate")
    for extra in args.extra_launch_arg:
        cmd.extend(["--extra-launch-arg", extra])
    return cmd


def delete_bag(run_root: Path) -> None:
    bag_dir = run_root / "AMCL_benchmark" / "AMCL_benchmark"
    if bag_dir.exists():
        shutil.rmtree(bag_dir)


def run_cases(args: argparse.Namespace, particles: Sequence[int]) -> None:
    for run_index in range(1, args.runs_per_particle + 1):
        for particle_count in particles:
            cmd = build_command(args, particle_count, run_index)
            print(f"\n=== particles={particle_count} run {run_index:02d}/{args.runs_per_particle} ===")
            print(" ".join(cmd))
            if args.dry_run:
                continue
            run_root = run_root_for(args.output_root, particle_count, run_index)
            run_root.mkdir(parents=True, exist_ok=True)
            log_path = run_root / "benchmark_stdout.log"
            with log_path.open("w") as log:
                log.write(" ".join(cmd) + "\n\n")
                log.flush()
                result = subprocess.run(
                    cmd,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    text=True,
                    check=False)
            if args.delete_bags:
                delete_bag(run_root)
            if result.returncode != 0:
                print(
                    f"[warn] particles={particle_count} run {run_index:02d} "
                    f"failed with code {result.returncode}; continuing sweep")


def collect_results(args: argparse.Namespace, particles: Sequence[int]) -> None:
    rows: List[Dict[str, object]] = []
    sample_rows: List[Dict[str, object]] = []
    for particle_count in particles:
        for run_index in range(1, args.runs_per_particle + 1):
            run_root = run_root_for(args.output_root, particle_count, run_index)
            pipeline_path = latest_pipeline_csv(run_root)
            if pipeline_path is None:
                print(f"[warn] missing pipeline CSV: {run_root}")
                continue
            pipeline_rows = load_pipeline_csv(pipeline_path, args.skip_first_sec)
            benchmark_path = benchmark_csv(run_root)
            benchmark_rows = (
                load_benchmark_csv(benchmark_path, args.skip_first_sec)
                if benchmark_path.exists() else []
            )
            system_rows = load_system_usage_csv(
                system_csv(run_root, "SystemUsageShort.csv"),
                args.skip_first_sec,
                ("cpu_short_window_percent", "gpu_percent"))
            gpu_rows = load_system_usage_csv(
                system_csv(run_root, "SystemUsageGpu.csv"),
                args.skip_first_sec,
                ("gpu_percent",))
            node_cpu_rows = load_node_cpu_csv(
                system_csv(run_root, "SystemUsageNodeProcesses.csv"),
                args.skip_first_sec)
            memory_rows = load_system_usage_csv(
                system_csv(run_root, "SystemUsageMemory.csv"),
                args.skip_first_sec,
                (
                    "cpu_mem_used_mib",
                    "cpu_mem_available_mib",
                    "cpu_cached_mib",
                    "cpu_page_cache_mib",
                ))
            cache_rows = load_cache_csv(
                system_csv(run_root, "SystemUsageCache.csv"),
                args.skip_first_sec)
            memory_controller_rows = load_memory_controller_csv(
                system_csv(run_root, "SystemUsageMemoryController.csv"),
                args.skip_first_sec)
            status = read_status(status_json(run_root))
            rows.append(summarize(
                particle_count, f"run_{run_index:02d}", pipeline_rows, benchmark_rows,
                system_rows, gpu_rows, node_cpu_rows, memory_rows,
                memory_controller_rows, cache_rows, status))
            for row in pipeline_rows:
                sample_rows.append({
                    "particles": particle_count,
                    "condition": condition_name(particle_count),
                    "run": f"run_{run_index:02d}",
                    **row,
                })
    write_csv(args.output_root / "Particle_Count_Sweep_Summary.csv", rows)
    write_csv(args.output_root / "Particle_Count_Sweep_Latency_Samples.csv", sample_rows)


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    repo_root = script_dir.parents[1]
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    default_output_root = (
        repo_root / "f1tenth_localization" / "Benchmark" / "Matlab" /
        "sim_benchmark" / f"gpu_amcl_particle_count_sweep_{timestamp}"
    )

    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", type=Path, default=default_output_root)
    parser.add_argument("--map-file", type=Path,
                        default=repo_root / "f1tenth_planning" / "maps" / "my_track_map.yaml")
    parser.add_argument("--trajectory-file", type=Path,
                        default=repo_root / "f1tenth_localization" / "Benchmark" /
                        "Matlab" / "sim_benchmark" / "my_track_raceline_vcap_4p0.csv")
    parser.add_argument("--initial-pose-x", default="0.5")
    parser.add_argument("--initial-pose-y", default="0.2")
    parser.add_argument("--initial-pose-yaw", default=None)
    parser.add_argument("--particles", default="100:100:1500")
    parser.add_argument("--runs-per-particle", type=int, default=1)
    parser.add_argument("--laps", type=int, default=10)
    parser.add_argument("--max-duration-sec", type=float, default=0.0)
    parser.add_argument("--process-timeout-sec", type=float, default=300.0)
    parser.add_argument("--max-beams", type=int, default=270)
    parser.add_argument("--update-min-d", type=float, default=0.04)
    parser.add_argument("--update-min-a", type=float, default=0.035)
    parser.add_argument("--likelihood-scale", type=float, default=4.0)
    parser.add_argument("--amcl-alpha1", type=float, default=0.1)
    parser.add_argument("--amcl-alpha2", type=float, default=0.2)
    parser.add_argument("--amcl-alpha3", type=float, default=0.2)
    parser.add_argument("--amcl-alpha4", type=float, default=0.25)
    parser.add_argument("--amcl-z-hit", type=float, default=0.90)
    parser.add_argument("--amcl-z-rand", type=float, default=0.10)
    parser.add_argument("--amcl-sigma-hit", type=float, default=0.05)
    parser.add_argument("--amcl-resample-threshold", type=float, default=0.5)
    parser.add_argument("--ekf-process-noise-scale", type=float, default=0.2)
    parser.add_argument("--cloud-publish-rate", type=float, default=0.0)
    parser.add_argument("--system-monitor-cpu-sample-hz", type=float, default=100.0)
    parser.add_argument("--system-monitor-gpu-sample-hz", type=float, default=50.0)
    parser.add_argument("--system-monitor-csv-log-hz", type=float, default=50.0)
    parser.add_argument("--system-monitor-long-csv-log-hz", type=float, default=1.0)
    parser.add_argument("--system-monitor-memory-log-hz", type=float, default=1.0)
    parser.add_argument("--system-monitor-memory-controller-log-hz", type=float, default=1.0)
    parser.add_argument("--system-monitor-emc-peak-bandwidth-mib-s", type=float, default=0.0)
    parser.add_argument("--cluster-publish-min-weight", type=float, default=0.60)
    parser.add_argument("--use-cluster-estimate", action=argparse.BooleanOptionalAction,
                        default=True)
    parser.add_argument(
        "--sim-odom-source",
        choices=("vesc", "ground_truth", "calibrated_drift"),
        default="vesc")
    parser.add_argument("--sim-drive-input-mode", choices=("vesc", "ackermann"), default="vesc")
    parser.add_argument("--control-start-delay-sec", type=float, default=3.0)
    parser.add_argument("--mpc-raceline-speed-margin", type=float, default=0.0)
    parser.add_argument("--skip-first-sec", type=float, default=5.0)
    parser.add_argument("--delete-bags", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--plot-only", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--extra-launch-arg", action="append", default=[])
    return parser.parse_args(argv)


def main(argv: Sequence[str]) -> int:
    args = parse_args(argv)
    args.output_root = args.output_root.resolve()
    args.map_file = args.map_file.resolve()
    args.trajectory_file = args.trajectory_file.resolve()
    particles = parse_particles(args.particles)

    print(f"Output root: {args.output_root}")
    print(f"Map        : {args.map_file}")
    print(f"Trajectory : {args.trajectory_file}")
    print(f"Particles  : {', '.join(str(v) for v in particles)}")
    print(f"Runs/laps  : {args.runs_per_particle} x {args.laps}")
    print(f"Sim setup  : odom={args.sim_odom_source}, drive={args.sim_drive_input_mode}")

    if not args.plot_only:
        run_cases(args, particles)
    if args.dry_run:
        return 0
    collect_results(args, particles)
    print(f"Summary        : {args.output_root / 'Particle_Count_Sweep_Summary.csv'}")
    print(f"Latency samples: {args.output_root / 'Particle_Count_Sweep_Latency_Samples.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
