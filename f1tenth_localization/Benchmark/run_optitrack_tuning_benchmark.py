#!/usr/bin/env python3
"""
Multi-bag OptiTrack benchmark for C++ GPU AMCL localization.

What this script does:
1) Verifies NVIDIA runtime (unless --allow-no-amcl is used).
2) Runs localization replay for each bag inside OptiTrackBags (or a custom folder).
3) Computes ego-vs-OptiTrack paired errors.
4) Writes deterministic output filenames (overwrite behavior).
5) Generates per-bag plots and one combined variance plot.
6) Writes one combined summary with tuning suggestions.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

try:
    import yaml
except Exception:  # pragma: no cover
    yaml = None

try:
    import matplotlib.pyplot as plt
except Exception:  # pragma: no cover
    plt = None


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_WORKSPACE = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
DEFAULT_BAGS_DIR = "/home/pascal/Documents/BachelorProject/f1tenth_localization/Benchmark/bags/benchmarkBags/OptiTrackBags"
DEFAULT_PARAMS_FILE = os.path.join(
    DEFAULT_WORKSPACE,
    "f1tenth_localization",
    "config",
    "gpu_amcl_cpp_params.yaml",
)
DEFAULT_TF_PUBLISHER = os.path.join(
    DEFAULT_WORKSPACE,
    "f1tenth_localization",
    "launch",
    "tf_pose_publisher.py",
)
DEFAULT_ODOM_TF_PUBLISHER = os.path.join(
    DEFAULT_WORKSPACE,
    "f1tenth_localization",
    "Benchmark",
    "JetsonFiles",
    "odom_tf_publisher.py",
)
DEFAULT_OUTPUT_DIR = "/home/pascal/Documents/BachelorProject/f1tenth_localization/Benchmark/optitrack_results"


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a: float, b: float) -> float:
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))


def safe_name(name: str) -> str:
    return "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in name)


@dataclass
class PoseSample:
    t: float
    x: float
    y: float
    yaw: float


@dataclass
class PairSample:
    t_ref: float
    dt: float
    gt_x: float
    gt_y: float
    ego_x: float
    ego_y: float
    gt_yaw: float
    ego_yaw: float
    ex: float
    ey: float
    epos: float
    eyaw: float


class OptitrackComparator(Node):
    def __init__(
        self,
        ego_source: str,
        ego_topic: str,
        ekf_topic: str,
        gt_topic: str,
        map_frame: str,
        world_frame: str,
    ):
        super().__init__("optitrack_comparator")
        self.ego_source = ego_source
        self.map_frame = map_frame
        self.world_frame = world_frame
        self.ego_samples: List[PoseSample] = []
        self.gt_samples: List[PoseSample] = []
        self.amcl_msg_count: int = 0
        self._map_world_tf: Optional[Tuple[float, float, float]] = None
        self._world_map_tf: Optional[Tuple[float, float, float]] = None
        self.map_msg: Optional[OccupancyGrid] = None
        self.first_scan_t: Optional[float] = None
        self.first_odom_t: Optional[float] = None
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

        if ego_source == "tf_topic":
            self.create_subscription(PoseStamped, ego_topic, self._ego_cb, 30)
        else:
            self.create_subscription(PoseWithCovarianceStamped, ekf_topic, self._ekf_cb, 30)

        # Needed for map/world conversion and OptiTrack-based AMCL initialization.
        self.create_subscription(TFMessage, "/tf_static", self._tf_static_cb, 10)
        self.create_subscription(OccupancyGrid, "/map", self._map_cb, 10)
        self.create_subscription(LaserScan, "/scan_walls", self._scan_cb, 30)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, 30)
        self.create_subscription(Odometry, "/ego_racecar/odom", self._odom_cb, 30)

        self.create_subscription(PoseStamped, gt_topic, self._gt_cb, 30)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._amcl_cb, 30)

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _ego_cb(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        self.ego_samples.append(
            PoseSample(
                t=self._stamp_to_sec(msg.header.stamp),
                x=msg.pose.position.x,
                y=msg.pose.position.y,
                yaw=quat_to_yaw(q.x, q.y, q.z, q.w),
            )
        )

    def _gt_cb(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        self.gt_samples.append(
            PoseSample(
                t=self._stamp_to_sec(msg.header.stamp),
                x=msg.pose.position.x,
                y=msg.pose.position.y,
                yaw=quat_to_yaw(q.x, q.y, q.z, q.w),
            )
        )

    def _amcl_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self.amcl_msg_count += 1

    def _tf_static_cb(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            parent = t.header.frame_id
            child = t.child_frame_id
            yaw = quat_to_yaw(
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            )
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            if parent == self.map_frame and child == self.world_frame:
                self._map_world_tf = (tx, ty, yaw)
            elif parent == self.world_frame and child == self.map_frame:
                self._world_map_tf = (tx, ty, yaw)

    def _ekf_cb(self, msg: PoseWithCovarianceStamped) -> None:
        if self._map_world_tf is None and self._world_map_tf is None:
            return

        x_map = msg.pose.pose.position.x
        y_map = msg.pose.pose.position.y
        yaw_map = quat_to_yaw(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

        if self._map_world_tf is not None:
            tx, ty, yaw = self._map_world_tf
            c = math.cos(yaw)
            s = math.sin(yaw)
            dx = x_map - tx
            dy = y_map - ty
            x_world = c * dx + s * dy
            y_world = -s * dx + c * dy
            yaw_world = yaw_map - yaw
        else:
            tx, ty, yaw = self._world_map_tf
            c = math.cos(yaw)
            s = math.sin(yaw)
            x_world = c * x_map - s * y_map + tx
            y_world = s * x_map + c * y_map + ty
            yaw_world = yaw_map + yaw

        self.ego_samples.append(
            PoseSample(
                t=self._stamp_to_sec(msg.header.stamp),
                x=x_world,
                y=y_world,
                yaw=math.atan2(math.sin(yaw_world), math.cos(yaw_world)),
            )
        )

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def _scan_cb(self, msg: LaserScan) -> None:
        if self.first_scan_t is None:
            self.first_scan_t = self._stamp_to_sec(msg.header.stamp)

    def _odom_cb(self, msg: Odometry) -> None:
        if self.first_odom_t is None:
            self.first_odom_t = self._stamp_to_sec(msg.header.stamp)


def world_to_map_pose(
    x_world: float,
    y_world: float,
    yaw_world: float,
    map_world_tf: Optional[Tuple[float, float, float]],
    world_map_tf: Optional[Tuple[float, float, float]],
) -> Optional[Tuple[float, float, float]]:
    if world_map_tf is not None:
        tx, ty, yaw = world_map_tf
        c = math.cos(yaw)
        s = math.sin(yaw)
        x_map = c * x_world - s * y_world + tx
        y_map = s * x_world + c * y_world + ty
        yaw_map = yaw_world + yaw
        return x_map, y_map, math.atan2(math.sin(yaw_map), math.cos(yaw_map))

    if map_world_tf is not None:
        tx, ty, yaw = map_world_tf
        c = math.cos(yaw)
        s = math.sin(yaw)
        dx = x_world - tx
        dy = y_world - ty
        x_map = c * dx + s * dy
        y_map = -s * dx + c * dy
        yaw_map = yaw_world - yaw
        return x_map, y_map, math.atan2(math.sin(yaw_map), math.cos(yaw_map))

    return None


def publish_initialpose(node: OptitrackComparator, x_map: float, y_map: float, yaw_map: float, map_frame: str, repeats: int) -> None:
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = map_frame
    msg.pose.pose.position.x = x_map
    msg.pose.pose.position.y = y_map
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.z = math.sin(0.5 * yaw_map)
    msg.pose.pose.orientation.w = math.cos(0.5 * yaw_map)

    # Conservative covariance for AMCL initialization.
    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.12

    # Use zero stamp to avoid sim-time reset issues between preroll and full replay.
    msg.header.stamp.sec = 0
    msg.header.stamp.nanosec = 0

    for _ in range(max(1, repeats)):
        node.initialpose_pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)


def pose_inside_map(msg: Optional[OccupancyGrid], x_map: float, y_map: float) -> Optional[bool]:
    if msg is None:
        return None

    res = float(msg.info.resolution)
    if res <= 0.0:
        return None

    ox = float(msg.info.origin.position.x)
    oy = float(msg.info.origin.position.y)
    w = int(msg.info.width)
    h = int(msg.info.height)
    if w <= 0 or h <= 0:
        return None

    mx = int(math.floor((x_map - ox) / res))
    my = int(math.floor((y_map - oy) / res))
    return 0 <= mx < w and 0 <= my < h


def run_bash(command: str, workspace: str, quiet: bool = True) -> subprocess.Popen:
    stdout = subprocess.DEVNULL if quiet else None
    stderr = subprocess.DEVNULL if quiet else None
    return subprocess.Popen(
        ["bash", "-lc", command],
        cwd=workspace,
        stdout=stdout,
        stderr=stderr,
        text=True,
        preexec_fn=os.setsid,
    )


def terminate_process(proc: Optional[subprocess.Popen], timeout_s: float = 5.0) -> None:
    if proc is None or proc.poll() is not None:
        return
    init_pose_for_main: Optional[Tuple[float, float, float]] = None

    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=timeout_s)
    except Exception:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=2.0)
        except Exception:
            pass


def nvidia_runtime_ok() -> bool:
    try:
        proc = subprocess.run(
            ["nvidia-smi", "-L"],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=5.0,
        )
        return proc.returncode == 0
    except Exception:
        return False


def pair_samples(
    gt_samples: List[PoseSample],
    ego_samples: List[PoseSample],
    max_pair_dt_s: float,
) -> List[PairSample]:
    if not gt_samples or not ego_samples:
        return []

    gt_sorted = sorted(gt_samples, key=lambda s: s.t)
    ego_sorted = sorted(ego_samples, key=lambda s: s.t)

    pairs: List[PairSample] = []
    j = 0
    n = len(ego_sorted)

    for g in gt_sorted:
        while j + 1 < n and abs(ego_sorted[j + 1].t - g.t) <= abs(ego_sorted[j].t - g.t):
            j += 1
        e = ego_sorted[j]
        dt_s = e.t - g.t
        if abs(dt_s) > max_pair_dt_s:
            continue

        ex = e.x - g.x
        ey = e.y - g.y
        eyaw = angle_diff(e.yaw, g.yaw)
        pairs.append(
            PairSample(
                t_ref=g.t,
                dt=dt_s,
                gt_x=g.x,
                gt_y=g.y,
                ego_x=e.x,
                ego_y=e.y,
                gt_yaw=g.yaw,
                ego_yaw=e.yaw,
                ex=ex,
                ey=ey,
                epos=math.hypot(ex, ey),
                eyaw=eyaw,
            )
        )
    return pairs


def pair_samples_interpolated(
    gt_samples: List[PoseSample],
    ego_samples: List[PoseSample],
    max_pair_dt_s: float,
) -> List[PairSample]:
    """
    MATLAB-equivalent alignment:
    1) Use overlapping time interval.
    2) Use GT timestamps inside overlap as common base.
    3) Interpolate ego x/y/yaw onto those timestamps.
    """
    if not gt_samples or not ego_samples:
        return []

    gt_sorted = sorted(gt_samples, key=lambda s: s.t)
    ego_sorted = sorted(ego_samples, key=lambda s: s.t)

    t_start = max(gt_sorted[0].t, ego_sorted[0].t)
    t_end = min(gt_sorted[-1].t, ego_sorted[-1].t)
    if t_end <= t_start:
        return []

    gt_in = [g for g in gt_sorted if t_start <= g.t <= t_end]
    if not gt_in:
        return []

    ego_t: List[float] = []
    ego_x: List[float] = []
    ego_y: List[float] = []
    ego_yaw: List[float] = []
    for s in ego_sorted:
        if ego_t and abs(s.t - ego_t[-1]) < 1e-12:
            ego_t[-1] = s.t
            ego_x[-1] = s.x
            ego_y[-1] = s.y
            ego_yaw[-1] = s.yaw
        else:
            ego_t.append(s.t)
            ego_x.append(s.x)
            ego_y.append(s.y)
            ego_yaw.append(s.yaw)

    if len(ego_t) < 2:
        return []

    def interp_linear(ts: List[float], vs: List[float], t: float) -> float:
        lo = 0
        hi = len(ts) - 1
        while lo + 1 < hi:
            mid = (lo + hi) // 2
            if ts[mid] <= t:
                lo = mid
            else:
                hi = mid
        t0, t1 = ts[lo], ts[hi]
        v0, v1 = vs[lo], vs[hi]
        if abs(t1 - t0) < 1e-12:
            return v0
        a = (t - t0) / (t1 - t0)
        return v0 + a * (v1 - v0)

    # Unwrap yaw before interpolation, then wrap back to [-pi, pi].
    ego_yaw_unwrapped: List[float] = [ego_yaw[0]]
    for i in range(1, len(ego_yaw)):
        dy = angle_diff(ego_yaw[i], ego_yaw[i - 1])
        ego_yaw_unwrapped.append(ego_yaw_unwrapped[-1] + dy)

    pairs: List[PairSample] = []
    j_near = 0
    for g in gt_in:
        while j_near + 1 < len(ego_t) and abs(ego_t[j_near + 1] - g.t) <= abs(ego_t[j_near] - g.t):
            j_near += 1
        dt_s = ego_t[j_near] - g.t
        if abs(dt_s) > max_pair_dt_s:
            continue

        ex_i = interp_linear(ego_t, ego_x, g.t)
        ey_i = interp_linear(ego_t, ego_y, g.t)
        eyaw_i_u = interp_linear(ego_t, ego_yaw_unwrapped, g.t)
        eyaw_i = math.atan2(math.sin(eyaw_i_u), math.cos(eyaw_i_u))

        ex = ex_i - g.x
        ey = ey_i - g.y
        pairs.append(
            PairSample(
                t_ref=g.t,
                dt=dt_s,
                gt_x=g.x,
                gt_y=g.y,
                ego_x=ex_i,
                ego_y=ey_i,
                gt_yaw=g.yaw,
                ego_yaw=eyaw_i,
                ex=ex,
                ey=ey,
                epos=math.hypot(ex, ey),
                eyaw=angle_diff(eyaw_i, g.yaw),
            )
        )

    return pairs


def shift_samples(samples: List[PoseSample], dt_s: float) -> List[PoseSample]:
    if abs(dt_s) < 1e-9:
        return samples
    return [PoseSample(t=s.t - dt_s, x=s.x, y=s.y, yaw=s.yaw) for s in samples]


def estimate_time_offset(
    gt_samples: List[PoseSample],
    ego_samples: List[PoseSample],
    max_pair_dt_s: float,
    search_abs_s: float = 0.8,
    step_s: float = 0.01,
) -> float:
    best_offset = 0.0
    best_score = float("inf")
    best_count = 0
    n_steps = int(search_abs_s / step_s)

    for k in range(-n_steps, n_steps + 1):
        offset = k * step_s
        shifted = shift_samples(ego_samples, offset)
        pairs = pair_samples(gt_samples, shifted, max_pair_dt_s=max_pair_dt_s)
        if not pairs:
            continue

        count = len(pairs)
        if count < 100:
            continue

        # Use spatial matching quality, not only timestamp density.
        score = mean([p.epos for p in pairs])
        if score < best_score:
            best_score = score
            best_count = count
            best_offset = offset

    if best_count == 0:
        return 0.0
    return best_offset


def estimate_se2_bias_from_pairs(pairs: List[PairSample]) -> Tuple[float, float, float]:
    """
    Estimate rigid transform (theta, tx, ty) mapping ego points into gt points.
    gt ~= R(theta) * ego + t
    """
    if not pairs:
        return 0.0, 0.0, 0.0

    ex = [p.ego_x for p in pairs]
    ey = [p.ego_y for p in pairs]
    gx = [p.gt_x for p in pairs]
    gy = [p.gt_y for p in pairs]

    me_x = mean(ex)
    me_y = mean(ey)
    mg_x = mean(gx)
    mg_y = mean(gy)

    a = 0.0
    b = 0.0
    for p in pairs:
        ecx = p.ego_x - me_x
        ecy = p.ego_y - me_y
        gcx = p.gt_x - mg_x
        gcy = p.gt_y - mg_y
        a += ecx * gcx + ecy * gcy
        b += ecx * gcy - ecy * gcx

    theta = math.atan2(b, a)
    c = math.cos(theta)
    s = math.sin(theta)
    tx = mg_x - (c * me_x - s * me_y)
    ty = mg_y - (s * me_x + c * me_y)
    return theta, tx, ty


def apply_se2_bias_to_pairs(pairs: List[PairSample], theta: float, tx: float, ty: float) -> List[PairSample]:
    c = math.cos(theta)
    s = math.sin(theta)
    out: List[PairSample] = []

    for p in pairs:
        ego_x = c * p.ego_x - s * p.ego_y + tx
        ego_y = s * p.ego_x + c * p.ego_y + ty
        ego_yaw = math.atan2(math.sin(p.ego_yaw + theta), math.cos(p.ego_yaw + theta))

        ex = ego_x - p.gt_x
        ey = ego_y - p.gt_y
        out.append(
            PairSample(
                t_ref=p.t_ref,
                dt=p.dt,
                gt_x=p.gt_x,
                gt_y=p.gt_y,
                ego_x=ego_x,
                ego_y=ego_y,
                gt_yaw=p.gt_yaw,
                ego_yaw=ego_yaw,
                ex=ex,
                ey=ey,
                epos=math.hypot(ex, ey),
                eyaw=angle_diff(ego_yaw, p.gt_yaw),
            )
        )

    return out


def mean(xs: List[float]) -> float:
    if not xs:
        return float("nan")
    return sum(xs) / len(xs)


def rmse(xs: List[float]) -> float:
    if not xs:
        return float("nan")
    return math.sqrt(sum(v * v for v in xs) / len(xs))


def std(xs: List[float]) -> float:
    if len(xs) < 2:
        return 0.0
    m = mean(xs)
    return math.sqrt(sum((v - m) ** 2 for v in xs) / (len(xs) - 1))


def load_params(params_file: str) -> dict:
    if yaml is None:
        return {}
    try:
        with open(params_file, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return data.get("gpu_amcl_cpp", {}).get("ros__parameters", {}) or {}
    except Exception:
        return {}


def tuning_suggestions(metrics: dict, params: dict) -> List[str]:
    suggestions: List[str] = []
    rmse_xy = metrics.get("rmse_xy", float("nan"))
    rmse_yaw = metrics.get("rmse_yaw", float("nan"))
    std_xy = metrics.get("std_epos", float("nan"))
    bias_x = metrics.get("mean_ex", 0.0)
    bias_y = metrics.get("mean_ey", 0.0)

    num_particles = params.get("num_particles")
    max_beams = params.get("max_beams")
    sigma_hit = params.get("sigma_hit")

    if abs(bias_x) > 0.15 or abs(bias_y) > 0.15:
        suggestions.append(
            "Large position bias: verify map/world alignment and initial_pose_x/y/a before particle or alpha sweeps."
        )

    if not math.isnan(rmse_yaw) and rmse_yaw > 0.20:
        suggestions.append(
            "Yaw RMSE is high: reduce alpha1/alpha2 by about 20-40% and compare again."
        )

    if not math.isnan(rmse_xy) and rmse_xy > 0.20 and not math.isnan(std_xy) and std_xy > 0.10:
        suggestions.append(
            "High noisy position error: increase num_particles and/or max_beams to improve scan likelihood robustness."
        )

    if sigma_hit is not None and not math.isnan(rmse_xy) and rmse_xy > 0.25:
        suggestions.append(
            f"Sweep sigma_hit around {sigma_hit:.3f}, e.g. [{max(0.05, sigma_hit - 0.08):.2f}, {sigma_hit:.2f}, {sigma_hit + 0.08:.2f}]."
        )

    if num_particles is not None and max_beams is not None:
        suggestions.append(
            f"Baseline for this summary: num_particles={num_particles}, max_beams={max_beams}. Change one knob per iteration."
        )

    if not suggestions:
        suggestions.append("No strong single-direction signal. Keep parameters and gather more laps for confidence.")

    return suggestions


def list_bags(bags_dir: str) -> List[Tuple[str, str]]:
    if os.path.isfile(os.path.join(bags_dir, "metadata.yaml")):
        return [(os.path.basename(os.path.normpath(bags_dir)), bags_dir)]

    out: List[Tuple[str, str]] = []
    for name in sorted(os.listdir(bags_dir)):
        path = os.path.join(bags_dir, name)
        if not os.path.isdir(path):
            continue
        if os.path.isfile(os.path.join(path, "metadata.yaml")):
            out.append((name, path))
    return out


def write_per_bag_csv(output_dir: str, bag_name: str, pairs: List[PairSample]) -> str:
    os.makedirs(output_dir, exist_ok=True)
    path = os.path.join(output_dir, f"{safe_name(bag_name)}_paired_errors.csv")
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([
            "t_ref_s", "pair_dt_s", "gt_x_m", "gt_y_m", "ego_x_m", "ego_y_m",
            "gt_yaw_rad", "ego_yaw_rad", "error_x_m", "error_y_m", "error_pos_m", "error_yaw_rad",
        ])
        for p in pairs:
            w.writerow([
                f"{p.t_ref:.9f}", f"{p.dt:.9f}",
                f"{p.gt_x:.6f}", f"{p.gt_y:.6f}", f"{p.ego_x:.6f}", f"{p.ego_y:.6f}",
                f"{p.gt_yaw:.6f}", f"{p.ego_yaw:.6f}",
                f"{p.ex:.6f}", f"{p.ey:.6f}", f"{p.epos:.6f}", f"{p.eyaw:.6f}",
            ])
    return path


def generate_per_bag_plots(output_dir: str, bag_name: str, pairs: List[PairSample], ego_label: str) -> List[str]:
    if plt is None or not pairs:
        return []

    bag_key = safe_name(bag_name)
    t0 = pairs[0].t_ref
    ts = [p.t_ref - t0 for p in pairs]

    gt_x = [p.gt_x for p in pairs]
    gt_y = [p.gt_y for p in pairs]
    ego_x = [p.ego_x for p in pairs]
    ego_y = [p.ego_y for p in pairs]

    epos = [p.epos for p in pairs]
    eyaw = [p.eyaw for p in pairs]

    out: List[str] = []

    fig = plt.figure(figsize=(8.0, 6.5))
    ax = fig.add_subplot(1, 1, 1)
    ax.plot(gt_x, gt_y, label="OptiTrack", linewidth=2.0)
    ax.plot(ego_x, ego_y, label=ego_label, linewidth=1.4)
    ax.set_title(f"Trajectory Overlay - {bag_name}")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
    p1 = os.path.join(output_dir, f"{bag_key}_trajectory_overlay.png")
    fig.tight_layout()
    fig.savefig(p1, dpi=150)
    plt.close(fig)
    out.append(p1)

    fig = plt.figure(figsize=(11.0, 8.0))
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.plot(ts, epos, color="tab:blue", linewidth=1.2)
    ax1.set_title("Position Error vs Time")
    ax1.set_xlabel("time [s]")
    ax1.set_ylabel("error_pos [m]")
    ax1.grid(True, alpha=0.3)

    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(ts, eyaw, color="tab:red", linewidth=1.2)
    ax2.set_title("Yaw Error vs Time")
    ax2.set_xlabel("time [s]")
    ax2.set_ylabel("error_yaw [rad]")
    ax2.grid(True, alpha=0.3)

    ax3 = fig.add_subplot(2, 2, 3)
    ax3.hist(epos, bins=60, color="tab:blue", alpha=0.85)
    ax3.set_title("Position Error Distribution")
    ax3.set_xlabel("error_pos [m]")
    ax3.set_ylabel("count")
    ax3.grid(True, alpha=0.25)

    ax4 = fig.add_subplot(2, 2, 4)
    ax4.hist(eyaw, bins=60, color="tab:red", alpha=0.85)
    ax4.set_title("Yaw Error Distribution")
    ax4.set_xlabel("error_yaw [rad]")
    ax4.set_ylabel("count")
    ax4.grid(True, alpha=0.25)

    p2 = os.path.join(output_dir, f"{bag_key}_error_2x2.png")
    fig.tight_layout()
    fig.savefig(p2, dpi=150)
    plt.close(fig)
    out.append(p2)

    return out


def generate_combined_variance_plot(output_dir: str, rows: List[dict], file_prefix: str = "combined") -> Optional[str]:
    if plt is None or not rows:
        return None

    names = [r["bag_name"] for r in rows]
    mean_pos = [r["mean_epos"] for r in rows]
    std_pos = [r["std_epos"] for r in rows]
    mean_yaw = [r["mean_abs_eyaw"] for r in rows]
    std_yaw = [r["std_abs_eyaw"] for r in rows]

    all_mean_pos = mean(mean_pos)
    all_std_pos = std(mean_pos)
    all_mean_yaw = mean(mean_yaw)
    all_std_yaw = std(mean_yaw)

    x = list(range(len(names)))

    fig = plt.figure(figsize=(12.0, 8.0))

    ax1 = fig.add_subplot(2, 1, 1)
    ax1.errorbar(x, mean_pos, yerr=std_pos, fmt="o", capsize=4, color="tab:blue")
    ax1.axhline(all_mean_pos, color="black", linestyle="--", linewidth=1.2, label="overall mean")
    ax1.axhspan(all_mean_pos - all_std_pos, all_mean_pos + all_std_pos, color="gray", alpha=0.15, label="overall +/-1 std")
    ax1.set_title("Position Error Across Bags (mean with variance)")
    ax1.set_ylabel("error_pos [m]")
    ax1.set_xticks(x)
    ax1.set_xticklabels(names, rotation=25, ha="right")
    ax1.grid(True, alpha=0.3)
    ax1.legend()

    ax2 = fig.add_subplot(2, 1, 2)
    ax2.errorbar(x, mean_yaw, yerr=std_yaw, fmt="o", capsize=4, color="tab:red")
    ax2.axhline(all_mean_yaw, color="black", linestyle="--", linewidth=1.2, label="overall mean")
    ax2.axhspan(all_mean_yaw - all_std_yaw, all_mean_yaw + all_std_yaw, color="gray", alpha=0.15, label="overall +/-1 std")
    ax2.set_title("Absolute Yaw Error Across Bags (mean with variance)")
    ax2.set_ylabel("|error_yaw| [rad]")
    ax2.set_xticks(x)
    ax2.set_xticklabels(names, rotation=25, ha="right")
    ax2.grid(True, alpha=0.3)
    ax2.legend()

    fig.tight_layout()
    out = os.path.join(output_dir, f"{file_prefix}_error_variance.png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out


def write_combined_outputs(
    output_dir: str,
    per_bag_rows: List[dict],
    all_pairs: List[Tuple[str, PairSample]],
    combined_metrics: dict,
    suggestions: List[str],
    file_prefix: str = "combined",
) -> Tuple[str, str]:
    os.makedirs(output_dir, exist_ok=True)

    combined_csv = os.path.join(output_dir, f"{file_prefix}_paired_errors.csv")
    with open(combined_csv, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([
            "bag_name", "t_ref_s", "pair_dt_s", "gt_x_m", "gt_y_m", "ego_x_m", "ego_y_m",
            "gt_yaw_rad", "ego_yaw_rad", "error_x_m", "error_y_m", "error_pos_m", "error_yaw_rad",
        ])
        for bag_name, p in all_pairs:
            w.writerow([
                bag_name,
                f"{p.t_ref:.9f}", f"{p.dt:.9f}",
                f"{p.gt_x:.6f}", f"{p.gt_y:.6f}", f"{p.ego_x:.6f}", f"{p.ego_y:.6f}",
                f"{p.gt_yaw:.6f}", f"{p.ego_yaw:.6f}",
                f"{p.ex:.6f}", f"{p.ey:.6f}", f"{p.epos:.6f}", f"{p.eyaw:.6f}",
            ])

    combined_summary = os.path.join(output_dir, f"{file_prefix}_summary.txt")
    with open(combined_summary, "w", encoding="utf-8") as f:
        f.write("Combined OptiTrack Benchmark Summary\n")
        f.write("===================================\n\n")
        f.write("Per-bag metrics\n")
        f.write("---------------\n")
        for row in per_bag_rows:
            f.write(
                f"{row['bag_name']}: pairs={row['n_pairs']}, rmse_xy={row['rmse_xy']:.4f}, "
                f"rmse_yaw={row['rmse_yaw']:.4f}, mean_dt_ms={row['mean_pair_dt_ms']:.2f}, "
                f"amcl_msgs={row['n_amcl_msgs']}\n"
            )

        f.write("\nCombined metrics\n")
        f.write("----------------\n")
        for k, v in combined_metrics.items():
            if isinstance(v, float):
                f.write(f"{k}: {v:.6f}\n")
            else:
                f.write(f"{k}: {v}\n")

        f.write("\nSuggested next parameter changes (combined)\n")
        f.write("-------------------------------------------\n")
        for i, s in enumerate(suggestions, start=1):
            f.write(f"{i}. {s}\n")

    return combined_csv, combined_summary


def compute_metrics_for_pairs(
    bag_name: str,
    node: OptitrackComparator,
    pairs: List[PairSample],
    source_label: str,
) -> dict:
    ex = [p.ex for p in pairs]
    ey = [p.ey for p in pairs]
    epos = [p.epos for p in pairs]
    eyaw = [p.eyaw for p in pairs]
    abs_eyaw = [abs(v) for v in eyaw]
    dt_ms = [1000.0 * p.dt for p in pairs]

    return {
        "bag_name": bag_name,
        "n_gt_samples": len(node.gt_samples),
        "n_ego_samples": len(node.ego_samples),
        "n_amcl_msgs": node.amcl_msg_count,
        "n_pairs": len(pairs),
        "mean_pair_dt_ms": mean(dt_ms),
        "std_pair_dt_ms": std(dt_ms),
        "applied_time_offset_s": 0.0,
        "se2_bias_yaw_rad": 0.0,
        "se2_bias_x_m": 0.0,
        "se2_bias_y_m": 0.0,
        "mean_ex": mean(ex),
        "mean_ey": mean(ey),
        "rmse_x": rmse(ex),
        "rmse_y": rmse(ey),
        "rmse_xy": rmse(epos),
        "mean_epos": mean(epos),
        "mae_xy": mean([abs(v) for v in epos]),
        "std_epos": std(epos),
        "rmse_yaw": rmse(eyaw),
        "mae_yaw": mean(abs_eyaw),
        "mean_abs_eyaw": mean(abs_eyaw),
        "std_abs_eyaw": std(abs_eyaw),
    }


def run_one_bag(
    args: argparse.Namespace,
    bag_name: str,
    bag_path: str,
    enforce_gate: bool,
) -> Tuple[List[PairSample], dict]:
    rclpy.init(args=None)
    node = OptitrackComparator(
        ego_source=args.ego_source,
        ego_topic=args.ego_topic,
        ekf_topic=args.ekf_topic,
        gt_topic=args.gt_topic,
        map_frame=args.map_frame,
        world_frame=args.world_frame,
    )

    launch_proc: Optional[subprocess.Popen] = None
    tf_proc: Optional[subprocess.Popen] = None
    odom_tf_proc: Optional[subprocess.Popen] = None
    static_tf_proc: Optional[subprocess.Popen] = None
    bag_proc: Optional[subprocess.Popen] = None
    init_pose_published = False

    required_topics = [args.gt_topic]
    if args.recompute_localization:
        required_topics.extend([
            "/scan_walls",
            "/scan",
            "/scan_obstacles",
            "/ego_racecar/odom",
            "/map",
            "/tf_static",
        ])
    elif args.ego_source == "tf_topic":
        # MATLAB-equivalent path compares recorded ego topic from the bag.
        required_topics.append(args.ego_topic)
    all_topics = list(dict.fromkeys(required_topics + args.extra_topics))

    try:
        if args.recompute_localization:
            launch_cmd = (
                "source install/setup.bash && "
                "ros2 launch f1tenth_localization {launch} "
                "params_file:={params} use_sim_time:=true"
            ).format(launch=args.launch_file, params=args.params_file)
            launch_proc = run_bash(launch_cmd, args.workspace, quiet=False)

            if args.ego_source == "tf_topic":
                tf_cmd = f"source install/setup.bash && python3 {args.tf_publisher_script}"
                tf_proc = run_bash(tf_cmd, args.workspace)

            if args.run_odom_tf_publisher:
                odom_tf_cmd = (
                    f"source install/setup.bash && python3 {args.odom_tf_publisher_script} "
                    "--ros-args "
                    "-p odom_topic:=/ego_racecar/odom "
                    "-p odom_frame:=ego_racecar/odom "
                    "-p base_frame:=ego_racecar/base_link"
                )
                odom_tf_proc = run_bash(odom_tf_cmd, args.workspace)

        if args.world_map_xyz_ypr is not None:
            x, y, z, yaw, pitch, roll = args.world_map_xyz_ypr
            static_cmd = (
                "source install/setup.bash && "
                "ros2 run tf2_ros static_transform_publisher "
                f"{x} {y} {z} {yaw} {pitch} {roll} "
                f"{args.world_frame} {args.map_frame}"
            )
            static_tf_proc = run_bash(static_cmd, args.workspace)
            # Also cache this transform locally for initialization if /tf_static latching is delayed.
            node._world_map_tf = (x, y, yaw)

        warmup_until = time.time() + 3.0
        while time.time() < warmup_until:
            rclpy.spin_once(node, timeout_sec=0.1)

        if launch_proc is not None and launch_proc.poll() is not None:
            raise RuntimeError(
                f"Localization launch exited early with code {launch_proc.returncode}. See launch output above."
            )

        topics_args = " ".join(all_topics)
        print(f"[INFO] {bag_name}: replay topics = {topics_args}")
        bag_cmd = (
            "source install/setup.bash && "
            f"ros2 bag play {bag_path} --clock --rate {args.bag_rate} --topics {topics_args}"
        )
        bag_proc = run_bash(bag_cmd, args.workspace)

        while bag_proc.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.1)

            if args.use_optitrack_initial_pose and args.recompute_localization and (not init_pose_published):
                have_gt = len(node.gt_samples) > 0
                have_tf = (node._map_world_tf is not None) or (node._world_map_tf is not None)
                have_map = node.map_msg is not None
                if have_gt and have_tf and have_map:
                    # Use the freshest OptiTrack sample at initialization time.
                    gt0 = max(node.gt_samples, key=lambda s: s.t)
                    init_map = world_to_map_pose(
                        gt0.x,
                        gt0.y,
                        gt0.yaw,
                        node._map_world_tf,
                        node._world_map_tf,
                    )
                    if init_map is None:
                        print(f"[WARN] {bag_name}: no map/world TF available for OptiTrack initialization.")
                        init_pose_published = True
                    else:
                        x0, y0, yaw0 = init_map
                        # If TF conversion collapses near origin but raw OptiTrack is clearly away,
                        # prefer raw values to avoid wrong-side initialization.
                        if (abs(x0) < 0.5 and abs(y0) < 0.5) and (abs(gt0.x) > 0.8 or abs(gt0.y) > 0.8):
                            print(
                                f"[WARN] {bag_name}: TF-based init looks suspicious near origin; "
                                f"using raw OptiTrack pose instead (x={gt0.x:.3f}, y={gt0.y:.3f}, yaw={gt0.yaw:.3f})."
                            )
                            x0, y0, yaw0 = gt0.x, gt0.y, gt0.yaw
                        in_map = pose_inside_map(node.map_msg, x0, y0)
                        if in_map is False:
                            print(
                                f"[WARN] {bag_name}: OptiTrack-derived init pose outside map bounds "
                                f"(map x={x0:.3f}, y={y0:.3f}); skipping /initialpose."
                            )
                            init_pose_published = True
                        else:
                            print(
                                f"[INFO] {bag_name}: publishing /initialpose from OptiTrack "
                                f"(map x={x0:.3f}, y={y0:.3f}, yaw={yaw0:.3f} rad)"
                            )
                            publish_initialpose(node, x0, y0, yaw0, args.map_frame, args.initial_pose_publish_count)
                            init_pose_published = True

        drain_until = time.time() + 1.5
        while time.time() < drain_until:
            rclpy.spin_once(node, timeout_sec=0.1)

    finally:
        terminate_process(bag_proc)
        terminate_process(static_tf_proc)
        terminate_process(odom_tf_proc)
        terminate_process(tf_proc)
        terminate_process(launch_proc)
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    pairs_ekf = pair_samples_interpolated(node.gt_samples, node.ego_samples, max_pair_dt_s=args.max_pair_dt)
    if not pairs_ekf:
        for fallback_dt in (0.30, 0.50):
            pairs_ekf = pair_samples_interpolated(node.gt_samples, node.ego_samples, max_pair_dt_s=fallback_dt)
            if pairs_ekf:
                break

    if not pairs_ekf:
        raise RuntimeError(
            f"No paired samples for bag '{bag_name}'. gt={len(node.gt_samples)}, ego={len(node.ego_samples)}"
        )

    if enforce_gate and node.amcl_msg_count == 0:
        print(
            f"[WARN] {bag_name}: no /amcl_pose observed during replay; continuing with EKF-vs-OptiTrack scoring."
        )

    metrics_ekf = compute_metrics_for_pairs(bag_name, node, pairs_ekf, "ekf")
    return pairs_ekf, metrics_ekf


def build_cli() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Multi-bag OptiTrack benchmark with deterministic outputs")
    parser.add_argument("--workspace", default=DEFAULT_WORKSPACE, help="Workspace root")
    parser.add_argument("--bags-dir", default=DEFAULT_BAGS_DIR, help="Directory containing bag folders")
    parser.add_argument("--params-file", default=DEFAULT_PARAMS_FILE, help="Path to gpu_amcl_cpp_params.yaml")
    parser.add_argument("--launch-file", default="cpp_localization.launch.py", help="Launch file in f1tenth_localization")
    parser.add_argument("--ego-source", choices=["direct_ekf", "tf_topic"], default="direct_ekf")
    parser.add_argument("--tf-publisher-script", default=DEFAULT_TF_PUBLISHER)
    parser.add_argument("--odom-tf-publisher-script", default=DEFAULT_ODOM_TF_PUBLISHER)
    parser.add_argument("--run-odom-tf-publisher", action="store_true", default=True)
    parser.add_argument("--ego-topic", default="/ego_pose_world")
    parser.add_argument("--ekf-topic", default="/ekf_pose")
    parser.add_argument("--gt-topic", default="/vrpn_mocap/car_pos/pose")
    parser.add_argument("--max-pair-dt", type=float, default=0.20)
    parser.add_argument("--auto-time-align", action="store_true", default=False)
    parser.add_argument("--fit-se2-bias", dest="fit_se2_bias", action="store_true", default=False)
    parser.add_argument("--no-fit-se2-bias", dest="fit_se2_bias", action="store_false")
    parser.add_argument("--time-align-search", type=float, default=0.8)
    parser.add_argument("--bag-rate", type=float, default=1.0)
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--extra-topics", nargs="*", default=[])
    parser.add_argument("--world-frame", default="world")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument(
        "--world-map-xyz-ypr",
        nargs=6,
        type=float,
        default=None,
        metavar=("X", "Y", "Z", "YAW", "PITCH", "ROLL"),
    )
    parser.add_argument("--allow-no-amcl", action="store_true", default=False)
    parser.add_argument("--max-bags", type=int, default=0, help="Optional debug limit; 0 means all bags")
    parser.add_argument("--use-optitrack-initial-pose", action="store_true", default=True)
    parser.add_argument("--initial-pose-preroll-s", type=float, default=12.0)
    parser.add_argument("--initial-pose-publish-count", type=int, default=6)
    return parser.parse_args()


def main() -> int:
    args = build_cli()

    args.workspace = os.path.abspath(args.workspace)
    args.bags_dir = os.path.abspath(args.bags_dir)
    args.params_file = os.path.abspath(args.params_file)
    args.output_dir = os.path.abspath(args.output_dir)
    args.tf_publisher_script = os.path.abspath(args.tf_publisher_script)
    args.odom_tf_publisher_script = os.path.abspath(args.odom_tf_publisher_script)
    args.recompute_localization = True

    enforce_gate = args.recompute_localization and (not args.allow_no_amcl)

    if not os.path.isdir(args.workspace):
        print(f"ERROR: workspace not found: {args.workspace}")
        return 2
    if not os.path.isdir(args.bags_dir):
        print(f"ERROR: bags directory not found: {args.bags_dir}")
        return 2
    if not os.path.isfile(args.params_file):
        print(f"ERROR: params file not found: {args.params_file}")
        return 2

    if enforce_gate and not nvidia_runtime_ok():
        print("GATE_FAIL: NVIDIA runtime not healthy (`nvidia-smi` failed).")
        print("Use --allow-no-amcl only for debugging, not tuning.")
        return 3

    bag_entries = list_bags(args.bags_dir)
    if args.max_bags > 0:
        bag_entries = bag_entries[: args.max_bags]

    if not bag_entries:
        print(f"ERROR: no ROS2 bag folders found in {args.bags_dir}")
        return 2

    print("[INFO] NVIDIA runtime check passed")
    print("[INFO] Mode: tuning (recompute localization on bag replay)")
    print(f"[INFO] Running benchmark for {len(bag_entries)} bag(s)")

    os.makedirs(args.output_dir, exist_ok=True)

    all_pairs: List[Tuple[str, PairSample]] = []
    per_bag_rows: List[dict] = []

    ego_label = "EKF map->world (direct)" if args.ego_source == "direct_ekf" else "EKF /ego_pose_world"

    for bag_name, bag_path in bag_entries:
        print(f"\n[INFO] Running bag: {bag_name}")
        try:
            pairs, metrics = run_one_bag(
                args,
                bag_name,
                bag_path,
                enforce_gate=enforce_gate,
            )
        except Exception as e:
            print(f"ERROR: bag '{bag_name}' failed: {e}")
            return 1

        all_pairs.extend((bag_name, p) for p in pairs)
        per_bag_rows.append(metrics)

        csv_path = write_per_bag_csv(args.output_dir, bag_name, pairs)
        plot_paths = generate_per_bag_plots(args.output_dir, bag_name, pairs, ego_label)

        print(
            f"[INFO] {bag_name}: pairs={metrics['n_pairs']}, "
            f"rmse_xy={metrics['rmse_xy']:.4f}, rmse_yaw={metrics['rmse_yaw']:.4f}, "
            f"amcl_msgs={metrics['n_amcl_msgs']}"
        )
        print(f"[INFO] per-bag csv: {csv_path}")
        for p in plot_paths:
            print(f"[INFO] per-bag plot: {p}")

    all_epos = [p.epos for _, p in all_pairs]
    all_eyaw = [p.eyaw for _, p in all_pairs]
    all_abs_eyaw = [abs(v) for v in all_eyaw]
    all_ex = [p.ex for _, p in all_pairs]
    all_ey = [p.ey for _, p in all_pairs]
    all_dt_ms = [1000.0 * p.dt for _, p in all_pairs]

    combined_metrics = {
        "n_bags": len(per_bag_rows),
        "n_total_pairs": len(all_pairs),
        "mean_pair_dt_ms": mean(all_dt_ms),
        "std_pair_dt_ms": std(all_dt_ms),
        "mean_ex": mean(all_ex),
        "mean_ey": mean(all_ey),
        "rmse_x": rmse(all_ex),
        "rmse_y": rmse(all_ey),
        "rmse_xy": rmse(all_epos),
        "mae_xy": mean([abs(v) for v in all_epos]),
        "std_epos": std(all_epos),
        "rmse_yaw": rmse(all_eyaw),
        "mae_yaw": mean(all_abs_eyaw),
        "mean_abs_eyaw": mean(all_abs_eyaw),
        "std_abs_eyaw": std(all_abs_eyaw),
    }

    params = load_params(args.params_file)
    suggestions = tuning_suggestions(combined_metrics, params)
    combined_csv, combined_summary = write_combined_outputs(
        args.output_dir,
        per_bag_rows,
        all_pairs,
        combined_metrics,
        suggestions,
    )
    combined_plot = generate_combined_variance_plot(args.output_dir, per_bag_rows)

    print("\n=== Combined benchmark complete ===")
    for k, v in combined_metrics.items():
        if isinstance(v, float):
            print(f"{k:>18}: {v:.6f}")
        else:
            print(f"{k:>18}: {v}")
    print(f"combined_csv    : {combined_csv}")
    print(f"combined_summary: {combined_summary}")
    if combined_plot:
        print(f"combined_plot   : {combined_plot}")
    else:
        print("combined_plot   : (skipped; matplotlib unavailable)")

    print("\nCombined suggested next changes:")
    for i, s in enumerate(suggestions, start=1):
        print(f"{i}. {s}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
