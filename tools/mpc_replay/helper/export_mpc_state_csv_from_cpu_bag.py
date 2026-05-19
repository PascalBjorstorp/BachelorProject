#!/usr/bin/env python3
"""
Export fixed-width replay CSV from a CPU-driving bag that does NOT contain /mpc_state.

Inputs (defaults):
  - /ekf_pose (geometry_msgs/PoseWithCovarianceStamped)
  - /ego_racecar/odom (nav_msgs/Odometry)
  - /local_raceline (nav_msgs/Path)
  - Steering source: /sensors/servo_position_command OR fallback /drive

Output format matches tools/mpc_replay/export_mpc_state_csv.py.
"""

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from typing import List, Optional

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

HORIZON_DEFAULT = 20

# Keep aligned with mpc_fpga_constants.h
MPC_FPGA_QP_WIDTH = 32
MPC_FPGA_QP_INT_BITS = 14
MPC_FPGA_QP_FRAC_BITS = MPC_FPGA_QP_WIDTH - MPC_FPGA_QP_INT_BITS  # 18 -> Q18
MPC_FPGA_QP_SCALE = float(1 << MPC_FPGA_QP_FRAC_BITS)             # 262144.0 (kernel/state_publisher)
MPC_FPGA_MIN_VEL_MPS = 0.5
MPC_FPGA_MAX_VEL_MPS = 20.0
MPC_FPGA_PREDICTION_DT_S = 0.03
MPC_FPGA_SERVO_GAIN = -0.7284
MPC_FPGA_SERVO_OFFSET = 0.55
MPC_FPGA_STEER_CORRECTION_C2 = 0.589566
MPC_FPGA_STEER_CORRECTION_C1 = 0.918061
MPC_FPGA_STEER_CORRECTION_C0 = 0.001490

ARRAY_PREFIXES = [
    "ref_ey_fp",
    "ref_epsi_fp",
    "ref_x_fp",
    "ref_y_fp",
    "ref_psi_fp",
    "ref_vx_fp",
    "ref_vy_fp",
    "ref_omega_ref_fp",
    "ref_kappa_fp",
    "ref_left_bound_fp",
    "ref_right_bound_fp",
]


@dataclass
class RefWaypoint:
    s: float = 0.0
    x: float = 0.0
    y: float = 0.0
    psi: float = 0.0
    vx: float = MPC_FPGA_MIN_VEL_MPS
    kappa: float = 0.0
    left_bound: float = 0.5
    right_bound: float = 0.5


def to_qp(v: float) -> int:
    if not math.isfinite(v):
        return 0
    return int(v * MPC_FPGA_QP_SCALE + 0.5) if v >= 0 else int(v * MPC_FPGA_QP_SCALE - 0.5)


def fixed_len(arr, n):
    out = list(arr[:n])
    if len(out) < n:
        out.extend([0] * (n - len(out)))
    return out


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def decode_servo_to_steer(raw_servo: float) -> float:
    corrected = (raw_servo - MPC_FPGA_SERVO_OFFSET) / MPC_FPGA_SERVO_GAIN
    abs_corr = abs(corrected)
    if MPC_FPGA_STEER_CORRECTION_C2 != 0.0:
        disc = (
            MPC_FPGA_STEER_CORRECTION_C1 * MPC_FPGA_STEER_CORRECTION_C1
            - 4.0 * MPC_FPGA_STEER_CORRECTION_C2 * (MPC_FPGA_STEER_CORRECTION_C0 - abs_corr)
        )
        if disc >= 0.0:
            t = (-MPC_FPGA_STEER_CORRECTION_C1 + math.sqrt(disc)) / (2.0 * MPC_FPGA_STEER_CORRECTION_C2)
            return math.copysign(t, corrected)
    return corrected


def build_raceline_from_path(path_msg) -> List[RefWaypoint]:
    poses = path_msg.poses
    if len(poses) < 2:
        return []

    out: List[RefWaypoint] = []
    cumulative_s = 0.0
    prev_x = 0.0
    prev_y = 0.0

    # pass 1
    for i, p in enumerate(poses):
        wp = RefWaypoint()
        x = float(p.pose.position.x)
        y = float(p.pose.position.y)
        wp.x = x
        wp.y = y
        if i > 0:
            cumulative_s += math.hypot(x - prev_x, y - prev_y)
        wp.s = cumulative_s

        v_ref = abs(float(p.pose.position.z))
        if not math.isfinite(v_ref) or v_ref < MPC_FPGA_MIN_VEL_MPS:
            v_ref = MPC_FPGA_MIN_VEL_MPS
        if v_ref > MPC_FPGA_MAX_VEL_MPS:
            v_ref = MPC_FPGA_MAX_VEL_MPS
        wp.vx = v_ref

        left = float(p.pose.orientation.x)
        right = float(p.pose.orientation.y)
        if (not math.isfinite(left)) or left <= 0.0:
            left = 0.5
        if (not math.isfinite(right)) or right <= 0.0:
            right = 0.5
        wp.left_bound = left
        wp.right_bound = right

        out.append(wp)
        prev_x = x
        prev_y = y

    # pass 2 heading
    n = len(out)
    for i in range(n):
        i_prev = 0 if i == 0 else i - 1
        i_next = n - 1 if i + 1 >= n else i + 1
        dx = out[i_next].x - out[i_prev].x
        dy = out[i_next].y - out[i_prev].y
        if dx * dx + dy * dy > 1e-12:
            out[i].psi = math.atan2(dy, dx)
        elif i > 0:
            out[i].psi = out[i - 1].psi

    # pass 3 curvature
    if n >= 3:
        for i in range(1, n - 1):
            dpsi = normalize_angle(out[i + 1].psi - out[i - 1].psi)
            ds = out[i + 1].s - out[i - 1].s
            ds_safe = ds if ds > 1e-6 else 1e-6
            out[i].kappa = dpsi / ds_safe
        out[0].kappa = out[1].kappa
        out[-1].kappa = out[-2].kappa

    return out


def sample_raceline_by_s(raceline: List[RefWaypoint], target_s: float) -> RefWaypoint:
    if not raceline:
        return RefWaypoint()
    s0 = raceline[0].s
    sN = raceline[-1].s
    if sN - s0 < 1e-6:
        return raceline[0]
    if target_s <= s0:
        target_s = s0
    if target_s >= sN:
        target_s = sN

    idx = 0
    for i in range(len(raceline) - 1):
        if raceline[i].s <= target_s <= raceline[i + 1].s:
            idx = i
            break
    a = raceline[idx]
    b = raceline[idx + 1] if idx + 1 < len(raceline) else raceline[idx]
    ds = b.s - a.s
    t = ((target_s - a.s) / ds) if ds > 1e-9 else 0.0

    out = RefWaypoint()
    out.s = a.s + t * (b.s - a.s)
    out.x = a.x + t * (b.x - a.x)
    out.y = a.y + t * (b.y - a.y)
    dpsi = normalize_angle(b.psi - a.psi)
    out.psi = a.psi + t * dpsi
    out.vx = a.vx + t * (b.vx - a.vx)
    out.kappa = a.kappa + t * (b.kappa - a.kappa)
    out.left_bound = a.left_bound + t * (b.left_bound - a.left_bound)
    out.right_bound = a.right_bound + t * (b.right_bound - a.right_bound)
    return out


def build_horizon_arrays(raceline: List[RefWaypoint], horizon: int):
    if not raceline:
        return None

    v_ref_base = raceline[0].vx
    if v_ref_base <= 0.0:
        v_ref_base = max(MPC_FPGA_MIN_VEL_MPS, min(MPC_FPGA_MAX_VEL_MPS, MPC_FPGA_MAX_VEL_MPS * 0.7))

    ref_ey_fp = [0] * horizon
    ref_epsi_fp = [0] * horizon
    ref_x_fp = [0] * horizon
    ref_y_fp = [0] * horizon
    ref_psi_fp = [0] * horizon
    ref_vx_fp = [0] * horizon
    ref_vy_fp = [0] * horizon
    ref_omega_ref_fp = [0] * horizon
    ref_kappa_fp = [0] * horizon
    ref_left_bound_fp = [0] * horizon
    ref_right_bound_fp = [0] * horizon

    for step in range(horizon):
        target_s = v_ref_base * MPC_FPGA_PREDICTION_DT_S * float(step)
        wp = sample_raceline_by_s(raceline, target_s)

        ref_x_fp[step] = to_qp(wp.x)
        ref_y_fp[step] = to_qp(wp.y)
        ref_psi_fp[step] = to_qp(wp.psi)
        ref_vx_fp[step] = to_qp(wp.vx)
        ref_vy_fp[step] = to_qp(0.0)
        ref_omega_ref_fp[step] = to_qp(wp.vx * wp.kappa)
        ref_kappa_fp[step] = to_qp(wp.kappa)
        ref_left_bound_fp[step] = to_qp(wp.left_bound)
        ref_right_bound_fp[step] = to_qp(wp.right_bound)

    return {
        "ref_ey_fp": ref_ey_fp,
        "ref_epsi_fp": ref_epsi_fp,
        "ref_x_fp": ref_x_fp,
        "ref_y_fp": ref_y_fp,
        "ref_psi_fp": ref_psi_fp,
        "ref_vx_fp": ref_vx_fp,
        "ref_vy_fp": ref_vy_fp,
        "ref_omega_ref_fp": ref_omega_ref_fp,
        "ref_kappa_fp": ref_kappa_fp,
        "ref_left_bound_fp": ref_left_bound_fp,
        "ref_right_bound_fp": ref_right_bound_fp,
    }


def compute_frenet_errors_fp(raceline: List[RefWaypoint], x: float, y: float, theta: float):
    if len(raceline) < 2:
        return 0, 0

    best_dist2 = float("inf")
    best_ey = 0.0
    best_epsi = 0.0

    for i in range(len(raceline) - 1):
        a = raceline[i]
        b = raceline[i + 1]

        abx = b.x - a.x
        aby = b.y - a.y
        apx = x - a.x
        apy = y - a.y
        ab_len2 = abx * abx + aby * aby

        t = 0.0
        if ab_len2 > 1e-12:
            t = (apx * abx + apy * aby) / ab_len2
        if t < 0.0:
            t = 0.0
        if t > 1.0:
            t = 1.0

        path_x = a.x + t * abx
        path_y = a.y + t * aby
        path_psi = a.psi + t * normalize_angle(b.psi - a.psi)

        dx = x - path_x
        dy = y - path_y
        dist2 = dx * dx + dy * dy
        if dist2 < best_dist2:
            best_dist2 = dist2
            best_ey = -math.sin(path_psi) * dx + math.cos(path_psi) * dy
            best_epsi = normalize_angle(theta - path_psi)

    return to_qp(best_ey), to_qp(best_epsi)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--horizon", type=int, default=HORIZON_DEFAULT)
    ap.add_argument("--pose-topic", default="/ekf_pose")
    ap.add_argument("--odom-topic", default="/ego_racecar/odom")
    ap.add_argument("--raceline-topic", default="/local_raceline")
    ap.add_argument("--servo-topic", default="/sensors/servo_position_command")
    ap.add_argument("--drive-topic", default="/drive")
    ap.add_argument("--ackermann-topic", default="/ackermann_cmd")
    ap.add_argument("--steering-source", choices=["auto", "servo", "drive", "ackermann", "zero"], default="auto")
    args = ap.parse_args()

    if args.horizon < 2:
        print("horizon must be >= 2", file=sys.stderr)
        return 2

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=args.bag, storage_id="mcap"),
        ConverterOptions("cdr", "cdr"),
    )

    topics = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topics}

    required = [args.pose_topic, args.odom_topic, args.raceline_topic]
    for t in required:
        if t not in type_map:
            print(f"Required topic missing: {t}", file=sys.stderr)
            return 3

    if args.steering_source == "servo" and args.servo_topic not in type_map:
        print(
            f"Requested --steering-source=servo, but topic is missing: {args.servo_topic}",
            file=sys.stderr,
        )
        return 3
    if args.steering_source == "drive" and args.drive_topic not in type_map:
        print(
            f"Requested --steering-source=drive, but topic is missing: {args.drive_topic}",
            file=sys.stderr,
        )
        return 3
    if args.steering_source == "ackermann" and args.ackermann_topic not in type_map:
        print(
            f"Requested --steering-source=ackermann, but topic is missing: {args.ackermann_topic}",
            file=sys.stderr,
        )
        return 3

    pose_type = get_message(type_map[args.pose_topic])
    odom_type = get_message(type_map[args.odom_topic])
    raceline_type = get_message(type_map[args.raceline_topic])

    servo_type = get_message(type_map[args.servo_topic]) if args.servo_topic in type_map else None
    drive_type = get_message(type_map[args.drive_topic]) if args.drive_topic in type_map else None
    ackermann_type = get_message(type_map[args.ackermann_topic]) if args.ackermann_topic in type_map else None

    header = [
        "idx", "stamp_sec", "stamp_nsec", "stamp_ns",
        "x_fp", "y_fp", "theta_fp", "velocity_fp", "vy_fp", "omega_fp",
        "steering_angle_fp", "horizon_length_msg",
    ]
    for prefix in ARRAY_PREFIXES:
        for i in range(args.horizon):
            header.append(f"{prefix}_{i}")
    header.extend(["e_y_fp", "e_psi_fp"])

    latest_vx = 0.0
    latest_vy = 0.0
    latest_omega = 0.0
    has_odom = False
    current_steering = 0.0
    have_servo = False
    latest_drive_steer: Optional[float] = None
    latest_ack_steer: Optional[float] = None
    raceline: List[RefWaypoint] = []

    count = 0
    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)

        while reader.has_next():
            topic, data, _ = reader.read_next()

            if topic == args.raceline_topic:
                msg = deserialize_message(data, raceline_type)
                raceline = build_raceline_from_path(msg)
                continue

            if topic == args.odom_topic:
                msg = deserialize_message(data, odom_type)
                latest_vx = float(msg.twist.twist.linear.x)
                latest_vy = float(msg.twist.twist.linear.y)
                latest_omega = float(msg.twist.twist.angular.z)
                has_odom = True
                continue

            if servo_type is not None and topic == args.servo_topic:
                msg = deserialize_message(data, servo_type)
                current_steering = decode_servo_to_steer(float(msg.data))
                have_servo = True
                continue

            if drive_type is not None and topic == args.drive_topic:
                msg = deserialize_message(data, drive_type)
                latest_drive_steer = float(msg.drive.steering_angle)
                continue

            if ackermann_type is not None and topic == args.ackermann_topic:
                msg = deserialize_message(data, ackermann_type)
                latest_ack_steer = float(msg.drive.steering_angle)
                continue

            if topic != args.pose_topic:
                continue

            if not has_odom or not raceline:
                continue

            msg = deserialize_message(data, pose_type)

            # steering source selection
            if args.steering_source == "servo":
                steer = current_steering if have_servo else 0.0
            elif args.steering_source == "drive":
                steer = latest_drive_steer if latest_drive_steer is not None else 0.0
            elif args.steering_source == "ackermann":
                steer = latest_ack_steer if latest_ack_steer is not None else 0.0
            elif args.steering_source == "zero":
                steer = 0.0
            else:  # auto
                if have_servo:
                    steer = current_steering
                elif latest_drive_steer is not None:
                    steer = latest_drive_steer
                elif latest_ack_steer is not None:
                    steer = latest_ack_steer
                else:
                    steer = 0.0

            horizon_arrays = build_horizon_arrays(raceline, args.horizon)
            if horizon_arrays is None:
                continue

            x = float(msg.pose.pose.position.x)
            y = float(msg.pose.pose.position.y)
            q = msg.pose.pose.orientation
            theta = quaternion_to_yaw(float(q.x), float(q.y), float(q.z), float(q.w))
            e_y_fp, e_psi_fp = compute_frenet_errors_fp(raceline, x, y, theta)

            sec = int(msg.header.stamp.sec)
            nsec = int(msg.header.stamp.nanosec)
            stamp_ns = sec * 1_000_000_000 + nsec

            count += 1
            row = [
                count, sec, nsec, stamp_ns,
                to_qp(x), to_qp(y), to_qp(theta),
                to_qp(latest_vx), to_qp(latest_vy), to_qp(latest_omega),
                to_qp(steer), args.horizon,
            ]

            for prefix in ARRAY_PREFIXES:
                row.extend(fixed_len(horizon_arrays[prefix], args.horizon))
            row.extend([e_y_fp, e_psi_fp])

            w.writerow(row)

    print(f"Exported {count} synthesized rows to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
