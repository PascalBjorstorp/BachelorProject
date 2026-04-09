#!/usr/bin/env python3
"""Quick IMU health report from a ROS2 bag.

Focuses on metrics that explain odom instability in aggressive driving:
- gyro bias/noise while nearly stationary
- lateral acceleration spikes
- whether IMU linear acceleration appears gravity-compensated
"""

from __future__ import annotations

import argparse
import bisect
import math
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore


@dataclass
class ImuSample:
    t_ns: int
    gx: float
    gy: float
    gz: float
    ax: float
    ay: float
    az: float


@dataclass
class OdomSample:
    t_ns: int
    vx: float


def to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def mean(values: Sequence[float]) -> float:
    return sum(values) / len(values) if values else float("nan")


def stdev(values: Sequence[float], mu: Optional[float] = None) -> float:
    if len(values) < 2:
        return 0.0
    if mu is None:
        mu = mean(values)
    var = sum((v - mu) ** 2 for v in values) / (len(values) - 1)
    return math.sqrt(max(var, 0.0))


def percentile(values: Sequence[float], p: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    idx = min(len(ordered) - 1, max(0, int(round((p / 100.0) * (len(ordered) - 1)))))
    return ordered[idx]


def nearest_vx(odom_times: Sequence[int], odom_vx: Sequence[float], t_ns: int) -> Optional[float]:
    if not odom_times:
        return None
    i = bisect.bisect_left(odom_times, t_ns)
    if i == 0:
        return odom_vx[0]
    if i >= len(odom_times):
        return odom_vx[-1]
    before = i - 1
    after = i
    if (t_ns - odom_times[before]) <= (odom_times[after] - t_ns):
        return odom_vx[before]
    return odom_vx[after]


def collect_samples(
    bag_path: str,
    imu_topic: str,
    odom_topic: str,
) -> Tuple[List[ImuSample], List[OdomSample]]:
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    imu_samples: List[ImuSample] = []
    odom_samples: List[OdomSample] = []

    with Reader(bag_path) as reader:
        connections = [c for c in reader.connections if c.topic in {imu_topic, odom_topic}]
        for conn, _, raw in reader.messages(connections=connections):
            msg = typestore.deserialize_cdr(raw, conn.msgtype)
            if conn.topic == imu_topic:
                imu_samples.append(
                    ImuSample(
                        t_ns=to_ns(msg.header.stamp),
                        gx=float(msg.angular_velocity.x),
                        gy=float(msg.angular_velocity.y),
                        gz=float(msg.angular_velocity.z),
                        ax=float(msg.linear_acceleration.x),
                        ay=float(msg.linear_acceleration.y),
                        az=float(msg.linear_acceleration.z),
                    )
                )
            elif conn.topic == odom_topic:
                odom_samples.append(
                    OdomSample(
                        t_ns=to_ns(msg.header.stamp),
                        vx=float(msg.twist.twist.linear.x),
                    )
                )

    imu_samples.sort(key=lambda s: s.t_ns)
    odom_samples.sort(key=lambda s: s.t_ns)
    return imu_samples, odom_samples


def report(
    imu_samples: Sequence[ImuSample],
    odom_samples: Sequence[OdomSample],
    static_speed_thresh: float,
    accel_spike_thresh: float,
) -> str:
    if not imu_samples:
        return "No IMU samples found."

    odom_t = [s.t_ns for s in odom_samples]
    odom_vx = [s.vx for s in odom_samples]

    static_imu: List[ImuSample] = []
    moving_imu: List[ImuSample] = []

    for s in imu_samples:
        vx = nearest_vx(odom_t, odom_vx, s.t_ns)
        if vx is None:
            continue
        if abs(vx) <= static_speed_thresh:
            static_imu.append(s)
        else:
            moving_imu.append(s)

    gz_static = [s.gz for s in static_imu]
    ay_abs_moving = [abs(s.ay) for s in moving_imu]
    accel_norm_static = [math.sqrt(s.ax * s.ax + s.ay * s.ay + s.az * s.az) for s in static_imu]

    spike_count = sum(1 for s in imu_samples if abs(s.ay) > accel_spike_thresh)

    lines: List[str] = []
    lines.append("IMU Health Report")
    lines.append(f"imu samples: {len(imu_samples)}")
    lines.append(f"odom samples: {len(odom_samples)}")
    lines.append(f"static imu samples (|vx| <= {static_speed_thresh:.3f} m/s): {len(static_imu)}")
    lines.append(f"moving imu samples: {len(moving_imu)}")
    lines.append("")

    if static_imu:
        gz_mu = mean(gz_static)
        gz_sd = stdev(gz_static, gz_mu)
        norm_mu = mean(accel_norm_static)
        norm_sd = stdev(accel_norm_static, norm_mu)

        lines.append(f"static gyro z mean: {gz_mu:.5f} rad/s")
        lines.append(f"static gyro z std : {gz_sd:.5f} rad/s")
        lines.append(f"static accel norm mean: {norm_mu:.3f} m/s^2")
        lines.append(f"static accel norm std : {norm_sd:.3f} m/s^2")

        if abs(gz_mu) > 0.05:
            lines.append("warning: high gyro bias magnitude (>|0.05| rad/s)")

        if 8.0 <= norm_mu <= 11.5:
            lines.append(
                "note: static accel norm near gravity. Your IMU linear_acceleration likely contains gravity."
            )
    else:
        lines.append("warning: no static window found. Include standstill segment in bag.")

    lines.append("")
    if moving_imu:
        lines.append(f"moving |a_y| p95: {percentile(ay_abs_moving, 95):.3f} m/s^2")
        lines.append(f"moving |a_y| p99: {percentile(ay_abs_moving, 99):.3f} m/s^2")
        lines.append(f"moving |a_y| max: {max(ay_abs_moving):.3f} m/s^2")

    lines.append(f"|a_y| spikes above {accel_spike_thresh:.1f} m/s^2: {spike_count}")

    if spike_count > 0:
        lines.append(
            "warning: large lateral accel spikes detected; prefer yaw-residual-based slip detection or improve IMU filtering/mounting."
        )

    return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze IMU health from ROS2 bag")
    parser.add_argument("bag", help="Path to rosbag2 folder")
    parser.add_argument("--imu-topic", default="/sensors/imu/raw")
    parser.add_argument("--odom-topic", default="/ego_racecar/odom")
    parser.add_argument("--static-speed-thresh", type=float, default=0.10)
    parser.add_argument("--accel-spike-thresh", type=float, default=12.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    imu_samples, odom_samples = collect_samples(args.bag, args.imu_topic, args.odom_topic)
    print(
        report(
            imu_samples,
            odom_samples,
            static_speed_thresh=args.static_speed_thresh,
            accel_spike_thresh=args.accel_spike_thresh,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
