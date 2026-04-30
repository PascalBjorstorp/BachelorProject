#!/usr/bin/env python3

import math
import os
from typing import Dict, List, Tuple

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PointStamped, TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


def _as_point(value, label: str) -> Tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) < 2:
        raise ValueError(f"{label} must be a list with [x, y] or [x, y, z]")
    if len(value) > 3:
        raise ValueError(f"{label} must contain only [x, y] or [x, y, z]")
    return tuple(float(v) for v in value)


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def _quaternion_from_matrix(rotation: np.ndarray) -> Tuple[float, float, float, float]:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (rotation[2, 1] - rotation[1, 2]) / scale
        qy = (rotation[0, 2] - rotation[2, 0]) / scale
        qz = (rotation[1, 0] - rotation[0, 1]) / scale
    else:
        diagonal = np.diag(rotation)
        if diagonal[0] > diagonal[1] and diagonal[0] > diagonal[2]:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            qw = (rotation[2, 1] - rotation[1, 2]) / scale
            qx = 0.25 * scale
            qy = (rotation[0, 1] + rotation[1, 0]) / scale
            qz = (rotation[0, 2] + rotation[2, 0]) / scale
        elif diagonal[1] > diagonal[2]:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            qw = (rotation[0, 2] - rotation[2, 0]) / scale
            qx = (rotation[0, 1] + rotation[1, 0]) / scale
            qy = 0.25 * scale
            qz = (rotation[1, 2] + rotation[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            qw = (rotation[1, 0] - rotation[0, 1]) / scale
            qx = (rotation[0, 2] + rotation[2, 0]) / scale
            qy = (rotation[1, 2] + rotation[2, 1]) / scale
            qz = 0.25 * scale

    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    return qx / norm, qy / norm, qz / norm, qw / norm


def _euler_from_matrix(rotation: np.ndarray) -> Tuple[float, float, float]:
    sy = math.hypot(rotation[0, 0], rotation[1, 0])
    if sy > 1e-9:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def fit_2d_rigid_transform(
    optitrack_points: List[Tuple[float, float]],
    map_points: List[Tuple[float, float]],
) -> Dict:
    if len(optitrack_points) != len(map_points):
        raise ValueError("OptiTrack and map point counts differ")
    if len(optitrack_points) < 2:
        raise ValueError("At least two matched points are required")

    n = float(len(optitrack_points))
    opt_cx = sum(p[0] for p in optitrack_points) / n
    opt_cy = sum(p[1] for p in optitrack_points) / n
    map_cx = sum(p[0] for p in map_points) / n
    map_cy = sum(p[1] for p in map_points) / n

    dot_sum = 0.0
    cross_sum = 0.0
    spread = 0.0

    for opt, mp in zip(optitrack_points, map_points):
        ox = opt[0] - opt_cx
        oy = opt[1] - opt_cy
        mx = mp[0] - map_cx
        my = mp[1] - map_cy

        dot_sum += ox * mx + oy * my
        cross_sum += ox * my - oy * mx
        spread += ox * ox + oy * oy

    if spread < 1e-9:
        raise ValueError("OptiTrack points are too close together")

    yaw = _normalize_angle(math.atan2(cross_sum, dot_sum))
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    tx = map_cx - (cos_yaw * opt_cx - sin_yaw * opt_cy)
    ty = map_cy - (sin_yaw * opt_cx + cos_yaw * opt_cy)

    residuals = []
    sum_sq = 0.0
    for opt, mp in zip(optitrack_points, map_points):
        px = cos_yaw * opt[0] - sin_yaw * opt[1] + tx
        py = sin_yaw * opt[0] + cos_yaw * opt[1] + ty
        error = math.hypot(mp[0] - px, mp[1] - py)
        residuals.append({
            "predicted_map": [px, py],
            "measured_map": [mp[0], mp[1]],
            "error_m": error,
        })
        sum_sq += error * error

    return {
        "mode": "2d",
        "x": tx,
        "y": ty,
        "z": 0.0,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": yaw,
        "qx": 0.0,
        "qy": 0.0,
        "qz": math.sin(yaw / 2.0),
        "qw": math.cos(yaw / 2.0),
        "rotation_matrix": [
            [cos_yaw, -sin_yaw, 0.0],
            [sin_yaw, cos_yaw, 0.0],
            [0.0, 0.0, 1.0],
        ],
        "rms_error_m": math.sqrt(sum_sq / n),
        "max_error_m": max(item["error_m"] for item in residuals),
        "residuals": residuals,
    }


def fit_3d_rigid_transform(
    optitrack_points: List[Tuple[float, float, float]],
    map_points: List[Tuple[float, float, float]],
) -> Dict:
    if len(optitrack_points) != len(map_points):
        raise ValueError("OptiTrack and map point counts differ")
    if len(optitrack_points) < 3:
        raise ValueError("At least three matched points are required for 3D")

    source = np.asarray(optitrack_points, dtype=float)
    target = np.asarray(map_points, dtype=float)

    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    source_centered = source - source_center
    target_centered = target - target_center

    if np.linalg.matrix_rank(source_centered, tol=1e-9) < 2:
        raise ValueError("3D calibration points must not all be collinear")

    covariance = source_centered.T @ target_centered
    u_mat, _singular_values, vt_mat = np.linalg.svd(covariance)
    rotation = vt_mat.T @ u_mat.T

    if np.linalg.det(rotation) < 0.0:
        vt_mat[-1, :] *= -1.0
        rotation = vt_mat.T @ u_mat.T

    translation = target_center - rotation @ source_center
    qx, qy, qz, qw = _quaternion_from_matrix(rotation)
    roll, pitch, yaw = _euler_from_matrix(rotation)

    predicted = (rotation @ source.T).T + translation
    errors = np.linalg.norm(target - predicted, axis=1)
    residuals = []
    for pred, measured, opt, error in zip(predicted, target, source, errors):
        residuals.append({
            "predicted_map": pred.tolist(),
            "measured_map": measured.tolist(),
            "optitrack": opt.tolist(),
            "error_m": float(error),
        })

    return {
        "mode": "3d",
        "x": float(translation[0]),
        "y": float(translation[1]),
        "z": float(translation[2]),
        "roll": float(roll),
        "pitch": float(pitch),
        "yaw": float(yaw),
        "qx": float(qx),
        "qy": float(qy),
        "qz": float(qz),
        "qw": float(qw),
        "rotation_matrix": rotation.tolist(),
        "rms_error_m": float(math.sqrt(np.mean(errors * errors))),
        "max_error_m": float(np.max(errors)),
        "residuals": residuals,
    }


class OptitrackMapCalibrator(Node):
    def __init__(self):
        super().__init__("optitrack_map_calibrator")

        self.declare_parameter("landmarks_file", "")
        self.declare_parameter("output_file", "/tmp/optitrack_map_transform.yaml")
        self.declare_parameter("map_frame", "")
        self.declare_parameter("optitrack_frame", "")
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("calibration_mode", "auto")
        self.declare_parameter("map_point_z", 0.0)
        self.declare_parameter("use_clicked_z", False)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("keep_alive_after_fit", True)
        self.declare_parameter("z_translation", 0.0)

        landmarks_file = self.get_parameter("landmarks_file").value
        if not landmarks_file:
            raise RuntimeError("Parameter landmarks_file is required")
        landmarks_file = os.path.expanduser(str(landmarks_file))

        self.output_file = os.path.expanduser(
            str(self.get_parameter("output_file").value)
        )
        self.publish_tf = _as_bool(self.get_parameter("publish_tf").value)
        self.keep_alive_after_fit = _as_bool(
            self.get_parameter("keep_alive_after_fit").value
        )
        self.z_translation = float(self.get_parameter("z_translation").value)
        self.map_point_z = float(self.get_parameter("map_point_z").value)
        self.use_clicked_z = _as_bool(self.get_parameter("use_clicked_z").value)

        self.landmarks, file_map_frame, file_optitrack_frame = self._load_landmarks(
            landmarks_file
        )
        self.map_frame = (
            str(self.get_parameter("map_frame").value) or file_map_frame
        ).lstrip("/")
        self.optitrack_frame = (
            str(self.get_parameter("optitrack_frame").value) or file_optitrack_frame
        ).lstrip("/")
        self.calibration_mode = self._resolve_calibration_mode(
            str(self.get_parameter("calibration_mode").value)
        )

        self.clicked_points: List[Tuple[float, ...]] = []
        self.fit_done = False
        self.static_broadcaster = StaticTransformBroadcaster(self)

        topic = str(self.get_parameter("clicked_point_topic").value)
        self.subscription = self.create_subscription(
            PointStamped,
            topic,
            self._clicked_point_callback,
            10,
        )

        self.get_logger().info(
            f"Loaded {len(self.landmarks)} OptiTrack landmarks from {landmarks_file}"
        )
        self.get_logger().info(f"Using {self.calibration_mode.upper()} calibration")
        self.get_logger().info(
            f"Set RViz Fixed Frame to '{self.map_frame}', select Publish Point, "
            "then click landmarks in this order."
        )
        self._log_next_click()

    def _load_landmarks(self, path: str):
        with open(path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}

        entries = data.get("landmarks", data.get("points", []))
        if not isinstance(entries, list) or len(entries) < 2:
            raise ValueError("YAML must contain at least two landmarks")

        landmarks = []
        for index, item in enumerate(entries, start=1):
            if not isinstance(item, dict):
                raise ValueError(f"landmark {index} must be a mapping")
            name = str(item.get("name", f"landmark_{index}"))
            opt_point = _as_point(item.get("optitrack"), f"{name}.optitrack")
            landmarks.append({"name": name, "optitrack": opt_point})

        return (
            landmarks,
            str(data.get("map_frame", "map")),
            str(data.get("optitrack_frame", "world")),
        )

    def _resolve_calibration_mode(self, requested: str) -> str:
        requested = requested.strip().lower()
        if requested not in ("auto", "2d", "3d"):
            raise ValueError("calibration_mode must be auto, 2d, or 3d")

        all_3d = all(len(item["optitrack"]) == 3 for item in self.landmarks)
        all_2d = all(len(item["optitrack"]) == 2 for item in self.landmarks)
        if not (all_2d or all_3d):
            raise ValueError("All landmarks must use the same dimension")

        if requested == "auto":
            return "3d" if all_3d else "2d"
        if requested == "3d" and not all_3d:
            raise ValueError("3D calibration requires optitrack: [x, y, z]")
        if requested == "2d" and not all_2d:
            raise ValueError("2D calibration requires optitrack: [x, y]")
        return requested

    def _log_next_click(self):
        index = len(self.clicked_points)
        if index >= len(self.landmarks):
            return
        landmark = self.landmarks[index]
        opt = landmark["optitrack"]
        if self.calibration_mode == "3d":
            opt_text = f"x={opt[0]:.4f}, y={opt[1]:.4f}, z={opt[2]:.4f}"
        else:
            opt_text = f"x={opt[0]:.4f}, y={opt[1]:.4f}"
        self.get_logger().info(
            f"Click {index + 1}/{len(self.landmarks)}: {landmark['name']} "
            f"(OptiTrack {opt_text})"
        )

    def _clicked_point_callback(self, msg: PointStamped):
        if self.fit_done:
            return

        frame_id = msg.header.frame_id.lstrip("/")
        if frame_id != self.map_frame:
            self.get_logger().warn(
                f"Ignoring clicked point in frame '{msg.header.frame_id}'. "
                f"Expected '{self.map_frame}'. Set RViz Fixed Frame to '{self.map_frame}'."
            )
            return

        if self.calibration_mode == "3d":
            z_value = float(msg.point.z) if self.use_clicked_z else self.map_point_z
            self.clicked_points.append((float(msg.point.x), float(msg.point.y), z_value))
        else:
            self.clicked_points.append((float(msg.point.x), float(msg.point.y)))

        landmark = self.landmarks[len(self.clicked_points) - 1]
        clicked = self.clicked_points[-1]
        if self.calibration_mode == "3d":
            self.get_logger().info(
                f"Stored {landmark['name']}: map x={clicked[0]:.4f}, "
                f"y={clicked[1]:.4f}, z={clicked[2]:.4f}"
            )
        else:
            self.get_logger().info(
                f"Stored {landmark['name']}: map x={clicked[0]:.4f}, "
                f"y={clicked[1]:.4f}"
            )

        if len(self.clicked_points) == len(self.landmarks):
            self._compute_and_save()
            return

        self._log_next_click()

    def _compute_and_save(self):
        opt_points = [item["optitrack"] for item in self.landmarks]
        if self.calibration_mode == "3d":
            result = fit_3d_rigid_transform(opt_points, self.clicked_points)
        else:
            result = fit_2d_rigid_transform(opt_points, self.clicked_points)
            result["z"] = self.z_translation
        result["map_frame"] = self.map_frame
        result["optitrack_frame"] = self.optitrack_frame

        for landmark, residual in zip(self.landmarks, result["residuals"]):
            residual["name"] = landmark["name"]
            residual["optitrack"] = list(landmark["optitrack"])

        result["static_transform_publisher"] = (
            "ros2 run tf2_ros static_transform_publisher "
            f"--x {result['x']:.9f} --y {result['y']:.9f} "
            f"--z {result['z']:.9f} --qx {result['qx']:.9f} "
            f"--qy {result['qy']:.9f} --qz {result['qz']:.9f} "
            f"--qw {result['qw']:.9f} --frame-id {self.map_frame} "
            f"--child-frame-id {self.optitrack_frame}"
        )

        self._write_result(result)

        if self.publish_tf:
            self._publish_static_tf(result)

        self.fit_done = True
        self.get_logger().info(
            f"Fit complete ({result['mode']}): {self.map_frame} -> "
            f"{self.optitrack_frame} "
            f"x={result['x']:.4f}, y={result['y']:.4f}, "
            f"z={result['z']:.4f}, roll={result['roll']:.6f}, "
            f"pitch={result['pitch']:.6f}, yaw={result['yaw']:.6f} rad, "
            f"RMS={result['rms_error_m']:.4f} m, "
            f"max={result['max_error_m']:.4f} m"
        )
        self.get_logger().info(f"Saved calibration to {self.output_file}")
        self.get_logger().info(result["static_transform_publisher"])

        if not self.keep_alive_after_fit:
            rclpy.shutdown()

    def _write_result(self, result: Dict):
        output_dir = os.path.dirname(self.output_file)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
        with open(self.output_file, "w", encoding="utf-8") as handle:
            yaml.safe_dump(result, handle, sort_keys=False)

    def _publish_static_tf(self, result: Dict):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.optitrack_frame
        transform.transform.translation.x = result["x"]
        transform.transform.translation.y = result["y"]
        transform.transform.translation.z = result["z"]
        transform.transform.rotation.x = result["qx"]
        transform.transform.rotation.y = result["qy"]
        transform.transform.rotation.z = result["qz"]
        transform.transform.rotation.w = result["qw"]
        self.static_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = OptitrackMapCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
