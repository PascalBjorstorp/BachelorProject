"""Stage runner for the canonical room-limited calibration campaign.

The runner intentionally keeps the legacy stage implementations in place.  It
provides the missing campaign-level contract around them: one stage per
invocation, scoped recording, offline quality gates, reversible configuration
updates, and a persistent dependency manifest.
"""
from __future__ import annotations

import copy
import datetime as dt
import hashlib
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

import yaml

ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = ROOT.parents[1]
STEERING_ROOT = ROOT.parent / "steering"
ERPM_ROOT = ROOT.parent / "ERPM"


def _load(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(value, dict):
        raise ValueError(f"expected YAML mapping: {path}")
    return value


def _dump(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")


def _json_dump(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True, default=str) + "\n", encoding="utf-8")


def _deep_merge(base: dict[str, Any], update: dict[str, Any]) -> dict[str, Any]:
    result = copy.deepcopy(base)
    for key, value in update.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def _now_id() -> str:
    return dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_vehicle_calibration"


@dataclass(frozen=True)
class StageSpec:
    key: str
    directory: str
    kind: str
    topic_group: str
    runtime_name: str
    dependencies: tuple[str, ...] = ()


STAGES: tuple[StageSpec, ...] = (
    StageSpec("steering_command_audit", "00_command_chain_audit", "steering", "full_stationary", "raw_command_path_audit"),
    # Direct metrology comes before every motion stage.  In particular, the
    # measured LiDAR-to-base transform is frozen into the session snapshots
    # before scan motion is used as an analysis measurement.
    StageSpec("physical_metrology", "00a_physical_metrology", "manual", "manual", "physical_metrology", ("steering_command_audit",)),
    # The scene/ICP preflight is deliberately before the centre fit.  A centre
    # candidate must never be collected before the reference sensor has proved
    # it can produce robust moving-window measurements in the actual room.
    StageSpec("steering_observability", "03_sensor_observability", "steering", "full_motion", "sensor_observability", ("physical_metrology",)),
    StageSpec("steering_centre", "01_zero_curvature_centre", "steering", "full_motion", "zero_curvature_centre", ("steering_observability",)),
    StageSpec("steering_centre_validation", "01a_zero_curvature_validation", "steering", "full_motion", "zero_curvature_validation", ("steering_centre",)),
    StageSpec("steering_endstops", "02_physical_endstops", "steering", "hardware_only", "physical_endstops", ("steering_centre_validation",)),
    StageSpec("steering_static_training", "04_static_map_training", "steering", "full_motion", "static_map_training", ("steering_endstops",)),
    StageSpec("steering_static_holdout", "05_static_map_holdout", "steering", "full_motion", "static_map_holdout", ("steering_static_training",)),
    StageSpec("steering_response", "06_command_to_curvature_response", "steering", "full_motion", "command_to_curvature_response", ("steering_static_holdout",)),
    StageSpec("steering_response_validation", "06a_command_to_curvature_response_validation", "steering", "full_motion", "command_to_curvature_response_validation", ("steering_response",)),
    StageSpec("motor_command_audit", "00b_motor_command_chain_audit", "erpm", "command_audit", "00_command_chain_audit", ("steering_response_validation",)),
    StageSpec("longitudinal_observability", "01_longitudinal_observability", "erpm", "ackermann_vel", "01_longitudinal_observability", ("motor_command_audit",)),
    StageSpec("low_speed_launch", "02_low_speed_launch", "erpm", "raw_erpm", "02_low_speed_launch", ("longitudinal_observability",)),
    StageSpec("erpm_map_training", "03_raw_erpm_map_training", "erpm", "raw_erpm", "03_raw_erpm_map_training", ("low_speed_launch",)),
    StageSpec("erpm_map_holdout", "04_raw_erpm_map_holdout", "erpm", "raw_erpm", "04_raw_erpm_map_holdout", ("erpm_map_training",)),
    StageSpec("vel_to_erpm_audit", "05_vel_to_erpm_pipeline_audit", "erpm", "ackermann_vel", "05_vel_to_erpm_pipeline_audit", ("erpm_map_holdout",)),
    StageSpec("erpm_response", "06_raw_erpm_response", "erpm", "raw_erpm", "06_raw_erpm_response", ("vel_to_erpm_audit",)),
    StageSpec("erpm_response_validation", "06a_raw_erpm_response_validation", "erpm", "raw_erpm", "06a_raw_erpm_response_validation", ("erpm_response",)),
    StageSpec("coastdown", "07_coastdown", "erpm", "raw_current", "07_coastdown", ("erpm_response_validation",)),
    StageSpec("coastdown_validation", "07a_coastdown_validation", "erpm", "raw_current", "07a_coastdown_validation", ("coastdown",)),
    StageSpec("current_training", "08_raw_current_training", "erpm", "raw_current", "08_raw_current_training", ("coastdown_validation",)),
    StageSpec("current_holdout", "09_raw_current_holdout", "erpm", "raw_current", "09_raw_current_holdout", ("current_training",)),
    StageSpec("accel_interface", "10_accel_to_current_interface", "erpm", "ackermann_accel", "10_accel_to_current_interface", ("current_holdout",)),
    StageSpec("accel_interface_validation", "10a_accel_to_current_interface_validation", "erpm", "ackermann_accel", "10a_accel_to_current_interface_validation", ("accel_interface",)),
    # The richer causal odometry candidate is selected only after all A/B
    # longitudinal data exists, then exercised by two genuinely fresh C sets.
    StageSpec("odometry_candidate_velocity_validation", "11_candidate_velocity_verification", "erpm", "candidate_ackermann_vel", "11_candidate_velocity_verification", ("accel_interface_validation",)),
    StageSpec("odometry_candidate_accel_validation", "12_candidate_accel_verification", "erpm", "candidate_ackermann_accel", "12_candidate_accel_verification", ("odometry_candidate_velocity_validation",)),
    StageSpec("lateral_stiffness_training", "12_quasi_steady_lateral_training", "erpm", "ackermann_vel", "12_quasi_steady_lateral_training", ("odometry_candidate_accel_validation",)),
    StageSpec("lateral_stiffness_validation", "12a_quasi_steady_lateral_validation", "erpm", "ackermann_vel", "12a_quasi_steady_lateral_validation", ("lateral_stiffness_training",)),
)

STAGE_BY_KEY = {stage.key: stage for stage in STAGES}
STAGE_BY_DIRECTORY = {stage.directory: stage for stage in STAGES}

# A fresh validation capture is useful when C was spoiled by a person, obstacle
# or sensor drop-out.  It is *not* useful when C says the fitted candidate is
# wrong.  In that case ``redo --from <validation>`` explicitly returns to A of
# the associated fitted parameter and discards the temporary candidate state.
VALIDATION_REDO_TARGETS = {
    "steering_centre_validation": "steering_centre",
    "steering_static_holdout": "steering_static_training",
    "steering_response_validation": "steering_response",
    "erpm_map_holdout": "erpm_map_training",
    "erpm_response_validation": "erpm_response",
    "coastdown_validation": "coastdown",
    "current_holdout": "current_training",
    "accel_interface_validation": "accel_interface",
    # A failed candidate C must collect new fit data; rerunning identical C
    # cannot change the selected model and would create an endless loop.
    "odometry_candidate_velocity_validation": "erpm_map_training",
    "odometry_candidate_accel_validation": "current_training",
    "lateral_stiffness_validation": "lateral_stiffness_training",
}


class ConfigManager:
    """Apply temporary, build-verified patches and restore exact source bytes."""

    LOCK_NAME = ".vehicle_calibration_recovery.json"

    def __init__(self, runner: "SuiteRunner", source_path: Path) -> None:
        self.runner = runner
        self.workspace = runner.workspace
        self.source_path = source_path
        self.transaction_dir = runner.session / "vesc_config_transaction"
        self.backup_path = self.transaction_dir / "vesc.yaml.before_invocation"
        self.lock_path = self.workspace / self.LOCK_NAME
        self.build_command = list(runner.campaign["build_command"])
        self.started = False

    @staticmethod
    def _sha(path: Path) -> str:
        return hashlib.sha256(path.read_bytes()).hexdigest()

    def _build(self, label: str) -> None:
        if shutil.which(self.build_command[0]) is None:
            raise RuntimeError(f"build executable not found: {self.build_command[0]}")
        log = self.transaction_dir / f"{label}.log"
        with log.open("w", encoding="utf-8") as handle:
            result = subprocess.run(self.build_command, cwd=self.workspace, stdout=handle,
                                    stderr=subprocess.STDOUT, text=True, check=False)
        if result.returncode:
            tail = "\n".join(log.read_text(encoding="utf-8", errors="replace").splitlines()[-80:])
            raise RuntimeError(f"colcon build failed; inspect {log}\n{tail}")

    def begin(self) -> None:
        if self.lock_path.exists():
            raise RuntimeError(f"unrestored calibration transaction exists; run {Path(__file__).resolve().parents[1] / 'run_suite.py'} recover")
        if not self.source_path.is_file():
            raise FileNotFoundError(self.source_path)
        self.transaction_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(self.source_path, self.backup_path)
        _json_dump(self.lock_path, {
            "state": "active",
            "source_config": str(self.source_path),
            "workspace": str(self.workspace),
            "backup": str(self.backup_path),
            "original_sha256": self._sha(self.backup_path),
            "build_command": self.build_command,
            "session": str(self.runner.session),
        })
        self.started = True

    def apply(self, patch: dict[str, Any], label: str) -> None:
        if not self.started:
            raise RuntimeError("configuration transaction has not started")
        document = _load(self.source_path)
        for node, values in patch.items():
            if node == "global":
                node = "/**"
            section = document.setdefault(node, {})
            params = section.setdefault("ros__parameters", {})
            if not isinstance(params, dict) or not isinstance(values, dict):
                raise ValueError(f"invalid VESC patch section: {node}")
            params.update(values)
        with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=self.source_path.parent,
                                         prefix=".vehicle_calibration_", suffix=".tmp", delete=False) as handle:
            yaml.safe_dump(document, handle, sort_keys=False)
            temporary = Path(handle.name)
        os.replace(temporary, self.source_path)
        self._build(label)
        _json_dump(self.transaction_dir / "last_applied_patch.json", {
            "label": label,
            "patch": patch,
            "source_sha256": self._sha(self.source_path),
            "utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        })

    def restore(self) -> None:
        if not self.started:
            return
        shutil.copy2(self.backup_path, self.source_path)
        self._build("build_restore")
        _json_dump(self.lock_path, {
            "state": "restored",
            "source_config": str(self.source_path),
            "backup": str(self.backup_path),
            "restored_sha256": self._sha(self.source_path),
            "utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        })
        self.lock_path.unlink(missing_ok=True)
        self.started = False

    @classmethod
    def recover(cls, workspace: Path | None) -> None:
        candidates = [workspace.expanduser().resolve()] if workspace else []
        candidates.append(WORKSPACE_ROOT)
        candidates.extend(WORKSPACE_ROOT.parents)
        lock = next((candidate / cls.LOCK_NAME for candidate in candidates
                     if (candidate / cls.LOCK_NAME).is_file()), None)
        if lock is None:
            raise RuntimeError("no pending unified vehicle-calibration transaction found")
        metadata = _load(lock)
        source = Path(metadata["source_config"])
        backup = Path(metadata["backup"])
        if not backup.is_file():
            raise RuntimeError(f"recovery backup missing: {backup}")
        shutil.copy2(backup, source)
        command = list(metadata.get("build_command", ["colcon", "build", "--symlink-install"]))
        log = backup.parent / "build_recovery.log"
        with log.open("w", encoding="utf-8") as handle:
            result = subprocess.run(command, cwd=Path(metadata["workspace"]), stdout=handle,
                                    stderr=subprocess.STDOUT, text=True, check=False)
        if result.returncode:
            raise RuntimeError(f"recovery restored source but build failed; inspect {log}")
        lock.unlink()
        print("Vehicle-calibration recovery complete; source VESC configuration restored.")


class StackProcess:
    def __init__(self, runner: "SuiteRunner", kind: str, name: str) -> None:
        self.runner = runner
        self.kind = kind
        self.name = name
        self.process: subprocess.Popen[str] | None = None
        self.handle = None

    def start(self) -> None:
        cfg = self.runner.steering_cfg if self.kind == "steering" else self.runner.erpm_cfg
        if self.kind == "steering":
            command = [sys.executable, str(STEERING_ROOT / "launch" / "calibration_stack.py"),
                       "--config", str(self.runner.session / "steering_calibration_config_snapshot.yaml"),
                       "--raw-min", "0.0", "--raw-max", "1.0"]
        else:
            command = [sys.executable, str(ERPM_ROOT / "launch" / "calibration_stack.py"),
                       "--config", str(self.runner.session / "erpm_calibration_config_snapshot.yaml")]
            candidate_mode = {
                "odometry_candidate_velocity_validation": "velocity",
                "odometry_candidate_accel_validation": "accel",
            }.get(self.name)
            if candidate_mode:
                candidate = self.runner.session / "analysis" / "selected_odometry_candidate_patch.yaml"
                if not candidate.is_file():
                    raise RuntimeError(f"selected odometry candidate is missing: {candidate}")
                command += ["--candidate-patch", str(candidate), "--candidate-mode", candidate_mode]
        log = self.runner.session / "stack_logs" / f"{self.name}.log"
        log.parent.mkdir(parents=True, exist_ok=True)
        self.handle = log.open("w", encoding="utf-8")
        self.process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=self.handle,
                                        stderr=subprocess.STDOUT, start_new_session=True, text=True)
        time.sleep(2.0)
        if self.process.poll() is not None:
            detail = log.read_text(encoding="utf-8", errors="replace")
            self.handle.close()
            self.handle = None
            raise RuntimeError(f"{self.kind} calibration stack failed to start:\n{detail}")

    def stop(self) -> None:
        if self.process and self.process.poll() is None:
            try:
                os.killpg(self.process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                self.process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(self.process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                self.process.wait(timeout=5)
        if self.handle:
            self.handle.close()
            self.handle = None
        self.process = None


class SuiteRunner:
    STAGES = STAGES

    def __init__(self, config_path: Path, session: Path | None = None, workspace: Path | None = None) -> None:
        self.config_path = config_path.resolve()
        self.suite = _load(self.config_path)
        self.workspace = (workspace or WORKSPACE_ROOT).expanduser().resolve()
        self.campaign = self.suite["campaign"]
        self.base_dir = self.config_path.parent
        self.session = self._prepare_session(session)
        existing = (self.session / "session_manifest.yaml").exists()
        if existing:
            # A resumed campaign must use the exact seed and thresholds that
            # governed its earlier captures, even if the source profile changed
            # after the session was created.
            steering_snapshot = self.session / "steering_calibration_config_snapshot.yaml"
            erpm_snapshot = self.session / "erpm_calibration_config_snapshot.yaml"
            if not steering_snapshot.is_file() or not erpm_snapshot.is_file():
                raise RuntimeError("existing session is missing a frozen calibration configuration snapshot")
            self.steering_cfg = _load(steering_snapshot)
            self.erpm_cfg = _load(erpm_snapshot)
            combined_snapshot = self.session / "calibration_config_snapshot.yaml"
            self.combined_cfg = _load(combined_snapshot) if combined_snapshot.is_file() else _deep_merge(self.erpm_cfg, self.steering_cfg)
        else:
            self.steering_cfg = self._load_variant("steering")
            self.erpm_cfg = self._load_variant("erpm")
            self._resolve_steering_seed_from_source()
            self._resolve_geometry_from_source()
            self.combined_cfg = _deep_merge(self.erpm_cfg, self.steering_cfg)
            self.combined_cfg["site"] = _deep_merge(self.erpm_cfg.get("site", {}), self.suite.get("site", {}))
            self.combined_cfg["campaign"] = copy.deepcopy(self.campaign)
            self._configure_room_capture_policy()
            # Rebuild after inserting the generated, condition-specific timing
            # maps so the combined snapshot is an exact view of both runtime
            # snapshots rather than only the source overrides.
            self.combined_cfg = _deep_merge(self.erpm_cfg, self.steering_cfg)
            self.combined_cfg["site"] = _deep_merge(self.erpm_cfg.get("site", {}), self.suite.get("site", {}))
            self.combined_cfg["campaign"] = copy.deepcopy(self.campaign)
        self._write_snapshots_if_new()

    def _load_variant(self, name: str) -> dict[str, Any]:
        relative = Path(self.suite["base_configs"][name])
        source = (self.base_dir / relative).resolve()
        variant = _load(source)
        variant = _deep_merge(variant, self.suite.get("overrides", {}).get(name, {}))
        return variant

    def _resolve_steering_seed_from_source(self) -> None:
        """Use the currently deployed steering offset as a fresh-campaign prior.

        Raw-servo collection remains independent of the deployed map.  This
        value only locates a compact, symmetric search grid; the frozen session
        snapshot records both the resolved value and its provenance.
        """
        initial = self.steering_cfg.setdefault("initial", {})
        source = str(initial.get("raw_servo_seed_source", "configured")).strip().lower()
        if source in {"", "configured", "fixed"}:
            initial["raw_servo_seed_provenance"] = "configured fallback"
            return
        if source != "deployed_vesc_offset":
            raise RuntimeError(f"unknown steering raw-servo seed source: {source}")
        params = self._read_source_params()
        try:
            value = float(params["steering_angle_to_servo_offset"])
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError("deployed VESC configuration has no finite steering_angle_to_servo_offset") from exc
        bounds = self.steering_cfg.get("endstops", {})
        lower = float(bounds.get("raw_servo_domain_min", 0.0))
        upper = float(bounds.get("raw_servo_domain_max", 1.0))
        if not math.isfinite(value) or not lower <= value <= upper:
            raise RuntimeError(
                f"deployed steering_angle_to_servo_offset {value!r} is outside raw-servo domain "
                f"[{lower:.5f}, {upper:.5f}]"
            )
        initial["raw_servo_seed"] = value
        initial["raw_servo_seed_provenance"] = "deployed vesc.yaml steering_angle_to_servo_offset"

    def _deployed_geometry(self) -> dict[str, float]:
        document = _load(self._source_path())
        raw = document.get("vehicle_geometry", {}).get("ros__parameters", {})
        required = (
            "laser_to_base_x_m", "laser_to_base_y_m", "laser_to_base_z_m",
            "laser_to_base_yaw_rad", "imu_to_base_x_m", "imu_to_base_y_m",
            "imu_to_base_z_m", "imu_to_base_yaw_rad",
            "base_link_to_rear_axle_x_m", "base_link_to_rear_axle_y_m",
        )
        geometry: dict[str, float] = {}
        for key in required:
            try:
                value = float(raw[key])
            except (KeyError, TypeError, ValueError) as exc:
                raise RuntimeError(f"deployed vehicle_geometry has no finite {key}") from exc
            if not math.isfinite(value):
                raise RuntimeError(f"deployed vehicle_geometry {key} is not finite")
            geometry[key] = value
        return geometry

    def _resolve_geometry_from_source(self) -> None:
        """Freeze the already-measured deployed transforms into new sessions."""
        geometry = self._deployed_geometry()
        mapping = {
            "laser_to_base_x_m": "laser_to_base_x_m",
            "laser_to_base_y_m": "laser_to_base_y_m",
            "laser_to_base_z_m": "laser_to_base_z_m",
            "laser_to_base_yaw_rad": "laser_to_base_yaw_rad",
            "imu_to_base_x_m": "imu_to_base_x_m",
            "imu_to_base_y_m": "imu_to_base_y_m",
            "imu_to_base_z_m": "imu_to_base_z_m",
            "imu_to_base_yaw_rad": "imu_to_base_yaw_rad",
            "base_link_to_rear_axle_x_m": "base_link_to_rear_axle_x_m",
            "base_link_to_rear_axle_y_m": "base_link_to_rear_axle_y_m",
        }
        source = str(self._source_path())
        for config in (self.steering_cfg, self.erpm_cfg):
            hardware = config.setdefault("hardware", {})
            for hardware_key, geometry_key in mapping.items():
                hardware[hardware_key] = geometry[geometry_key]
            hardware["geometry_source"] = source

    def _configure_room_capture_policy(self) -> None:
        """Derive useful capture times from the reviewed room geometry.

        A single duration is wasteful at low speed and unsafe at high speed.
        New sessions therefore freeze an explicit duration for every steady
        straight condition.  The calculation reserves startup, command-path
        settling, the configured active-stop interval, a conservative braking
        distance and the complete vehicle body.  Short excitation pulses are
        deliberately untouched because pulse length is part of their input.
        """
        site = self.combined_cfg.get("site", self.suite["site"])
        room_length = float(site["room_length_m"])
        room_width = float(site.get("room_width_m", room_length))
        clearance = float(site.get("wall_clearance_m", 0.0))
        clear_length = room_length - 2.0 * clearance
        clear_width = room_width - 2.0 * clearance
        vehicle_length = float(site["vehicle_length_m"])
        vehicle_width = float(site.get("vehicle_width_m", vehicle_length))
        heading = math.radians(float(site.get("straight_lane_heading_deg", 0.0)))
        utilization = float(site.get("capture_room_utilization", 0.82))
        maximum_capture_s = float(site.get("max_useful_straight_capture_s", 30.0))
        revolutions = float(site.get("full_circle_revolutions", 1.0))
        braking = float(site["conservative_braking_mps2"])
        if not 0.0 < utilization < 1.0:
            raise RuntimeError("site.capture_room_utilization must lie strictly between zero and one")
        if maximum_capture_s <= 0.0 or revolutions <= 0.0 or braking <= 0.0:
            raise RuntimeError("room capture duration, revolutions and braking floor must be positive")

        cos_heading = abs(math.cos(heading))
        sin_heading = abs(math.sin(heading))
        target_length = utilization * clear_length
        target_width = utilization * clear_width
        target_envelope = min(
            (target_length - vehicle_width * sin_heading) / max(cos_heading, 1.0e-12)
            if cos_heading > 1.0e-12 else math.inf,
            (target_width - vehicle_width * cos_heading) / max(sin_heading, 1.0e-12)
            if sin_heading > 1.0e-12 else math.inf,
        )
        target_motion = target_envelope - vehicle_length
        if target_motion <= 0.0:
            raise RuntimeError("capture_room_utilization leaves no room for vehicle motion")

        steering_startup = self.steering_cfg["motion_startup"]
        erpm_startup = self.erpm_cfg["motion_startup"]
        steering_startup_s = (
            float(steering_startup["minimum_startup_s"])
            + float(steering_startup["stability_window_s"])
        )
        erpm_startup_s = (
            float(erpm_startup["minimum_startup_s"])
            + float(erpm_startup["stability_window_s"])
        )
        raw_switch_s = float(erpm_startup.get("raw_erpm_switch_settle_s", 0.0))
        active_stop_s = float(erpm_startup.get(
            "active_stop_max_brake_s", erpm_startup.get("active_stop_brake_s", 0.0)
        ))
        policy: dict[str, Any] = {
            "source": "generated from frozen site and motion-startup configuration",
            "target_room_utilization": utilization,
            "target_straight_envelope_m": target_envelope,
            "target_straight_motion_distance_m": target_motion,
            "target_circle_side_m": min(target_length, target_width),
            "clear_room_length_m": clear_length,
            "clear_room_width_m": clear_width,
            "straight_lane_heading_deg": float(site.get("straight_lane_heading_deg", 0.0)),
            "vehicle_length_m": vehicle_length,
            "vehicle_width_m": vehicle_width,
            "vehicle_circumscribed_radius_m": 0.5 * math.hypot(vehicle_length, vehicle_width),
            "conservative_braking_mps2": braking,
            "maximum_useful_capture_s": maximum_capture_s,
            "full_circle_revolutions": revolutions,
            "steering_startup_planning_s": steering_startup_s,
            "erpm_startup_planning_s": erpm_startup_s,
            "erpm_raw_switch_settle_s": raw_switch_s,
            "erpm_active_stop_reserve_s": active_stop_s,
        }
        self.steering_cfg["room_capture_policy"] = copy.deepcopy(policy)
        self.erpm_cfg["room_capture_policy"] = copy.deepcopy(policy)

        def settle_s(speed: float) -> float:
            distance = max(0.0, float(erpm_startup.get("pre_capture_settle_distance_m", 0.0)))
            minimum = max(0.0, float(erpm_startup.get("pre_capture_settle_min_s", 0.0)))
            maximum = max(minimum, float(erpm_startup.get("pre_capture_settle_max_s", minimum)))
            if distance <= 0.0:
                return minimum
            return min(maximum, max(minimum, distance / max(abs(float(speed)), 0.05)))

        def planned_capture(speed: float, minimum_s: float, *, raw_erpm: bool,
                            extra_moving_s: float = 0.0,
                            include_pre_capture_settle: bool = True) -> float:
            speed = abs(float(speed))
            if speed <= 0.0:
                raise RuntimeError("steady room capture speed must be positive")
            fixed_s = erpm_startup_s + active_stop_s + max(0.0, float(extra_moving_s))
            if raw_erpm:
                fixed_s += raw_switch_s
            if include_pre_capture_settle:
                fixed_s += settle_s(speed)
            braking_distance = speed * speed / (2.0 * braking)
            available = (target_motion - braking_distance) / speed - fixed_s
            duration = min(maximum_capture_s, available)
            # Existing minima are analysis requirements. If one ever exceeds
            # the room-derived value, retain it and let _validate_room produce
            # a concrete geometry failure instead of silently weakening data.
            return round(max(float(minimum_s), duration), 3)

        def capture_map(speeds: list[float], minimum_s: float, *, raw_erpm: bool,
                        extra_moving_s: float = 0.0,
                        include_pre_capture_settle: bool = True) -> dict[str, float]:
            return {
                f"{float(speed):.6g}": planned_capture(
                    float(speed), minimum_s, raw_erpm=raw_erpm,
                    extra_moving_s=extra_moving_s,
                    include_pre_capture_settle=include_pre_capture_settle,
                )
                for speed in speeds
            }

        observability = self.erpm_cfg["observability"]
        observability["straight_probe_capture_s_by_speed"] = capture_map(
            list(map(float, observability["straight_probe_speeds_mps"])),
            float(observability["straight_probe_capture_s"]), raw_erpm=False,
        )
        for section in ("low_speed_launch", "raw_erpm_map_training", "raw_erpm_map_holdout"):
            spec = self.erpm_cfg[section]
            spec["capture_s_by_speed"] = capture_map(
                list(map(float, spec["nominal_speeds_mps"])),
                float(spec["capture_s"]), raw_erpm=True,
            )
        audit = self.erpm_cfg["vel_to_erpm_pipeline_audit"]
        audit["capture_s_by_speed"] = capture_map(
            list(map(float, audit["speed_commands_mps"])),
            float(audit["capture_s"]), raw_erpm=False,
        )
        response = self.erpm_cfg["raw_erpm_response"]
        response_targets = sorted({
            float(pair[1])
            for pair in [*response["steps_mps"], *response.get("validation_steps_mps", [])]
        })
        response["response_capture_s_by_target_speed"] = capture_map(
            response_targets, float(response["response_capture_s"]), raw_erpm=True,
            extra_moving_s=float(response["pre_step_hold_s"]),
            include_pre_capture_settle=False,
        )
        coast = self.erpm_cfg["coastdown"]
        coast_speeds = sorted({
            *map(float, coast["initial_speeds_mps"]),
            *map(float, coast.get("validation_initial_speeds_mps", [])),
        })
        coast_minimum = max(
            float(coast["coast_capture_s"]),
            float(coast.get("validation_coast_capture_s", coast["coast_capture_s"])),
        )
        coast["coast_capture_s_by_initial_speed"] = capture_map(
            coast_speeds, coast_minimum, raw_erpm=True,
            extra_moving_s=float(coast["pre_coast_hold_s"]),
            include_pre_capture_settle=False,
        )
        candidate = self.erpm_cfg["candidate_verification"]
        candidate["velocity_capture_s_by_speed"] = capture_map(
            list(map(float, candidate["velocity_holdout_commands_mps"])),
            float(candidate["capture_s"]), raw_erpm=False,
        )

    def _prepare_session(self, session: Path | None) -> Path:
        if session is not None:
            path = session.expanduser().resolve()
            path.mkdir(parents=True, exist_ok=True)
            return path
        runs = (self.base_dir.parent / self.suite.get("campaign", {}).get("runs_dir", "runs")).resolve()
        path = runs / _now_id()
        path.mkdir(parents=True, exist_ok=False)
        return path

    def _campaign_budget(self) -> dict[str, Any]:
        """Calculate an auditable nominal campaign size from the frozen grid.

        This deliberately counts *accepted driving passes*, not highly
        correlated 40 Hz scan registrations.  A spoiled pass or an explicit
        recalibration naturally adds time, so the operator estimate is a range
        rather than a false promise of a single exact duration.
        """
        rows: list[dict[str, Any]] = []

        def add(key: str, trials_min: int, trials_max: int | None = None, *, note: str = "") -> None:
            rows.append({
                "stage": key,
                "nominal_driving_trials_min": int(trials_min),
                "nominal_driving_trials_max": int(trials_min if trials_max is None else trials_max),
                "note": note,
            })

        steering = self.steering_cfg
        centre = steering["centre_trim"]
        observability = steering["observability"]
        static = steering["static_map"]
        response = steering["response"]
        probe_count = len(centre.get("onboard_probe_offsets_servo", []))
        fine_count = len(centre.get("fine_grid_offsets_servo", []))
        fine_repetitions = int(centre.get("candidate_repetitions", 1))
        # The deployed seed is normally already the zero-offset grid point. A
        # guided shift can place it outside the fine grid, adding one retained
        # seed measurement.  Report both values rather than concealing it.
        add("steering_observability", len(observability["straight_speeds_mps"]) * int(observability["straight_repetitions"]), note="plus one stationary LiDAR diagnostic")
        add("steering_centre", probe_count + fine_count * fine_repetitions,
            probe_count + (fine_count + 1) * fine_repetitions,
            note="three guided on-board probes plus repeated LiDAR fine grid")
        add("steering_centre_validation", sum(int(item.get("repetitions", 1)) for item in centre.get("validation_conditions", [])))
        training_targets = 2 * len(static["training_fractions"])
        static_training = training_targets * 2 * int(static["training_sweep_repetitions"])
        static_holdout = 2 * len(static["validation_fractions"]) * int(static["validation_repetitions"])
        add("steering_static_training", static_training, note="outward and inward sweeps on both sides")
        add("steering_static_holdout", static_holdout, note="shuffled independent conditions")

        def steering_response_count(conditions: list[dict[str, Any]]) -> int:
            return sum(2 * len(item["target_fractions"]) * int(item.get("repetitions", 1)) for item in conditions)

        add("steering_response", steering_response_count(list(response["conditions"])))
        add("steering_response_validation", steering_response_count(list(response.get("validation_conditions", []))))

        erpm = self.erpm_cfg
        add("longitudinal_observability", len(erpm["observability"]["straight_probe_speeds_mps"]) * int(erpm["observability"]["straight_probe_repetitions"]), note="plus one stationary IMU/LiDAR diagnostic")
        add("low_speed_launch", len(erpm["low_speed_launch"]["nominal_speeds_mps"]) * int(erpm["low_speed_launch"]["repetitions"]))
        add("erpm_map_training", len(erpm["raw_erpm_map_training"]["nominal_speeds_mps"]) * int(erpm["raw_erpm_map_training"]["repetitions"]))
        add("erpm_map_holdout", len(erpm["raw_erpm_map_holdout"]["nominal_speeds_mps"]) * int(erpm["raw_erpm_map_holdout"]["repetitions"]))
        add("vel_to_erpm_audit", len(erpm["vel_to_erpm_pipeline_audit"]["speed_commands_mps"]) * int(erpm["vel_to_erpm_pipeline_audit"]["repetitions"]))
        add("erpm_response", len(erpm["raw_erpm_response"]["steps_mps"]) * int(erpm["raw_erpm_response"]["repetitions"]))
        add("erpm_response_validation", len(erpm["raw_erpm_response"].get("validation_steps_mps", [])) * int(erpm["raw_erpm_response"].get("validation_repetitions", erpm["raw_erpm_response"]["repetitions"])))
        add("coastdown", len(erpm["coastdown"]["initial_speeds_mps"]) * int(erpm["coastdown"]["repetitions"]))
        add("coastdown_validation", len(erpm["coastdown"].get("validation_initial_speeds_mps", [])) * int(erpm["coastdown"].get("validation_repetitions", erpm["coastdown"]["repetitions"])))

        def current_count(section: str) -> int:
            spec = erpm[section]
            conditions = sum(
                len(item["current_fractions"])
                for polarity in ("drive_conditions", "brake_conditions")
                for item in spec[polarity]
            )
            return conditions * int(spec["repetitions"])

        add("current_training", current_count("raw_current_training"))
        add("current_holdout", current_count("raw_current_holdout"))
        accel = erpm["accel_to_current_interface"]
        add("accel_interface", len(accel["initial_speeds_mps"]) * len(accel["acceleration_commands_mps2"]) * int(accel["repetitions"]))
        add("accel_interface_validation", len(accel.get("validation_initial_speeds_mps", [])) * len(accel.get("validation_acceleration_commands_mps2", [])) * int(accel.get("validation_repetitions", accel["repetitions"])))
        candidate = erpm["candidate_verification"]
        add("odometry_candidate_velocity_validation", len(candidate["velocity_holdout_commands_mps"]) * int(candidate["velocity_repetitions"]), note="fresh C speed cells for selected shadow odometry")
        add("odometry_candidate_accel_validation", len(candidate["acceleration_initial_speeds_mps"]) * len(candidate["acceleration_holdout_commands_mps2"]) * int(candidate["acceleration_repetitions"]), note="fresh C transient cells for selected shadow odometry")
        lateral = erpm["lateral_stiffness"]
        add("lateral_stiffness_training", len(lateral["speeds_mps"]) * len(lateral["steering_angles_rad"]) * len(lateral.get("signs", [-1.0, 1.0])) * int(lateral["repetitions"]))
        add("lateral_stiffness_validation", len(lateral.get("validation_speeds_mps", [])) * len(lateral.get("validation_steering_angles_rad", [])) * len(lateral.get("validation_signs", lateral.get("signs", [-1.0, 1.0]))) * int(lateral.get("validation_repetitions", lateral["repetitions"])))

        total_min = sum(int(row["nominal_driving_trials_min"]) for row in rows)
        total_max = sum(int(row["nominal_driving_trials_max"]) for row in rows)
        # Room-optimised steady straights and complete circles add about 56
        # minutes of actual recording versus the former minimum-duration grid.
        # Keep that visible instead of hiding it inside repositioning time.
        extended_capture_allowance_h = 1.0 if self.erpm_cfg.get("room_capture_policy") else 0.0
        minimum_hours = float(math.ceil(
            total_min * 45.0 / 3600.0 + 2.5 + extended_capture_allowance_h
        ))
        maximum_hours = float(math.ceil(
            total_max * 60.0 / 3600.0 + 4.0 + extended_capture_allowance_h
        ))
        return {
            "measurement_unit": "accepted driving pass; never individual LiDAR scan registrations",
            "nominal_driving_trials_min": total_min,
            "nominal_driving_trials_max": total_max,
            "stationary_capture_blocks": 2,
            "manual_stages": ["physical_metrology", "steering_endstops"],
            "per_stage": rows,
            "operator_time_estimate_hours_without_rework": [minimum_hours, maximum_hours],
            "room_optimized_capture_allowance_hours": extended_capture_allowance_h,
            "operator_time_assumptions": (
                "45–60 s average setup/repositioning per accepted driving pass, plus 2.5–4 h "
                "for direct metrology, builds, plot review and safety checks, plus 1 h for extended "
                "steady captures/full circles; spoiled captures and REDO are extra"
            ),
        }

    def _write_snapshots_if_new(self) -> None:
        if not (self.session / "session_manifest.yaml").exists():
            self._validate_room()
            _dump(self.session / "steering_calibration_config_snapshot.yaml", self.steering_cfg)
            _dump(self.session / "erpm_calibration_config_snapshot.yaml", self.erpm_cfg)
            _dump(self.session / "calibration_config_snapshot.yaml", self.combined_cfg)
            # Direct physical values cannot be inferred reliably from bags.
            # Freeze a session-local, editable measurement form beside the
            # captured data so the later lateral fit has clear provenance.
            from .metrology import ensure_measurement_sheet
            ensure_measurement_sheet(
                self.session,
                deployed_geometry=self._deployed_geometry(),
                geometry_source=self._source_path(),
            )
            self._write_policy()
            _dump(self.session / "session_manifest.yaml", {
                "session_id": self.session.name,
                "status": "created",
                "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
                "workspace": str(self.workspace),
                "suite_config": str(self.config_path),
                "stages": {},
                "stage_order": [stage.key for stage in STAGES],
                "room_preflight": self._room_report,
                "campaign_budget": self._campaign_budget(),
                "recalibration_events": [],
            })
            _json_dump(self.session / "runtime_state.json", {
                "status": "created",
                "active_patch": {},
                "centre": {"centre_servo_raw": float(self.steering_cfg["initial"]["raw_servo_seed"])},
                "limits": None,
                "speed_command_patch": None,
                "accel_patch": None,
            })
            # The source document exists from session creation onward, with one
            # intentionally pending page per stage.  A PDF is compiled after
            # each completed stage when pdflatex is installed.
            from .latex_report import write_latex_document
            write_latex_document(self, compile_pdf=False)
        else:
            manifest = _load(self.session / "session_manifest.yaml")
            expected_order = [stage.key for stage in STAGES]
            if manifest.get("stage_order") != expected_order:
                raise RuntimeError(
                    "this session uses an older calibration stage sequence; keep it for inspection "
                    "and create a new session for the current campaign"
                )
            from .metrology import ensure_measurement_sheet
            ensure_measurement_sheet(
                self.session,
                deployed_geometry=self._deployed_geometry(),
                geometry_source=self._source_path(),
            )
            policy = self.session / "recording_policy_snapshot.yaml"
            if not policy.is_file():
                raise RuntimeError(f"existing session is missing {policy}")
            self._recording = _load(policy).get("recording", {})

    @property
    def manifest(self) -> dict[str, Any]:
        return _load(self.session / "session_manifest.yaml")

    @property
    def state(self) -> dict[str, Any]:
        path = self.session / "runtime_state.json"
        return json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}

    def _save_state(self, state: dict[str, Any]) -> None:
        _json_dump(self.session / "runtime_state.json", state)

    def _save_manifest(self, manifest: dict[str, Any]) -> None:
        _dump(self.session / "session_manifest.yaml", manifest)

    def _room_steering_angle_cap(self) -> float:
        """Return the frozen steering bound used for room-turn geometry.

        The static-map and response captures use raw servo before a new map is
        known, so their room requirement cannot honestly be inferred from the
        map being fitted.  The profile therefore has an explicit conservative
        road-wheel-angle cap.  Once the human survey has measured the real
        safe limits, reject a campaign whose observed range exceeds that
        reviewed bound instead of allowing an unexpectedly tight arc.
        """
        site = self.combined_cfg.get("site", self.suite["site"])
        try:
            cap = float(site.get("max_room_steering_angle_rad", 0.50))
        except (TypeError, ValueError) as exc:
            raise RuntimeError("site.max_room_steering_angle_rad must be a finite positive angle") from exc
        if not math.isfinite(cap) or not 0.0 < cap < 0.5 * math.pi:
            raise RuntimeError("site.max_room_steering_angle_rad must lie between 0 and pi/2")
        limits = self.state.get("limits")
        if isinstance(limits, dict):
            observed: list[float] = []
            for key in ("observed_low_wheel_angle_rad", "observed_high_wheel_angle_rad"):
                try:
                    value = float(limits.get(key))
                except (TypeError, ValueError):
                    continue
                if math.isfinite(value):
                    observed.append(abs(value))
            if observed and max(observed) > cap + 1.0e-9:
                raise RuntimeError(
                    "human end-stop survey recorded a road-wheel angle outside the frozen room profile "
                    f"({max(observed):.3f} rad > {cap:.3f} rad). Reduce the static/response span or create a "
                    "reviewed room profile before driving the steering arcs."
                )
        return cap

    def _validate_room(self) -> None:
        site = self.combined_cfg.get("site", self.suite["site"])
        # New profiles describe the physical walls.  The fallback keeps old,
        # frozen sessions readable by interpreting their safety margin as a
        # total two-ended allowance around the old usable rectangle.
        if "room_length_m" in site:
            room_length = float(site["room_length_m"])
            room_width = float(site.get("room_width_m", room_length))
            clearance = float(site.get("wall_clearance_m", 0.0))
            heading_deg = float(site.get("straight_lane_heading_deg", 0.0))
        else:
            legacy_clearance = 0.5 * float(site.get("safety_margin_m", 0.0))
            room_length = float(site["usable_straight_m"]) + 2.0 * legacy_clearance
            room_width = float(site.get("usable_width_m", site["usable_straight_m"])) + 2.0 * legacy_clearance
            clearance = legacy_clearance
            heading_deg = 0.0
        vehicle_length = float(site["vehicle_length_m"])
        vehicle_width = float(site.get("vehicle_width_m", vehicle_length))
        values = (room_length, room_width, clearance, vehicle_length, vehicle_width, heading_deg)
        if not all(math.isfinite(value) for value in values):
            raise RuntimeError("room dimensions, clearance, heading and vehicle footprint must be finite")
        if min(room_length, room_width, vehicle_length, vehicle_width) <= 0.0 or clearance < 0.0:
            raise RuntimeError("room dimensions and vehicle footprint must be positive; wall clearance cannot be negative")
        clear_length = room_length - 2.0 * clearance
        clear_width = room_width - 2.0 * clearance
        if clear_length <= vehicle_length or clear_width <= vehicle_width:
            raise RuntimeError("room profile leaves no vehicle-sized operating area after wall clearance")
        heading = math.radians(heading_deg)
        cos_heading = abs(math.cos(heading))
        sin_heading = abs(math.sin(heading))
        if max(cos_heading, sin_heading) <= 1.0e-12:
            raise RuntimeError("straight-lane heading is invalid")
        steering_angle_cap = self._room_steering_angle_cap()
        cases = self._room_cases(steering_angle_cap)
        for room_case in cases:
            envelope_length = float(room_case["estimated_distance_m"])
            envelope_width = float(room_case.get("required_width_m", vehicle_width))
            if room_case.get("room_geometry") == "axis_aligned_circle":
                # A complete circle is invariant to lane heading. Its values
                # already describe the axis-aligned swept footprint plus the
                # explicitly reserved approach/stop extension.
                projected_length = envelope_length
                projected_width = envelope_width
            else:
                projected_length = envelope_length * cos_heading + envelope_width * sin_heading
                projected_width = envelope_length * sin_heading + envelope_width * cos_heading
            room_case.update({
                "planned_heading_deg": heading_deg,
                "envelope_length_m": envelope_length,
                "envelope_width_m": envelope_width,
                "projected_room_length_m": projected_length,
                "projected_room_width_m": projected_width,
                "clear_room_length_m": clear_length,
                "clear_room_width_m": clear_width,
                "room_utilization": max(projected_length / clear_length, projected_width / clear_width),
            })
        failures = [case for case in cases if float(case["room_utilization"]) > 1.0]
        if failures:
            worst = max(failures, key=lambda item: float(item["room_utilization"]))
            raise RuntimeError(
                f"room preflight failed for {worst['name']}: its footprint-aware envelope projects to "
                f"{float(worst['projected_room_length_m']):.2f} x "
                f"{float(worst['projected_room_width_m']):.2f} m at {heading_deg:.1f} degrees, but the "
                f"clear room is only {clear_length:.2f} x {clear_width:.2f} m"
            )
        worst_distance = max(cases, key=lambda item: float(item["estimated_distance_m"]))
        worst_utilization = max(cases, key=lambda item: float(item["room_utilization"]))
        envelope_capacity = min(
            (clear_length - vehicle_width * sin_heading) / max(cos_heading, 1.0e-12)
            if cos_heading > 1.0e-12 else math.inf,
            (clear_width - vehicle_width * cos_heading) / max(sin_heading, 1.0e-12)
            if sin_heading > 1.0e-12 else math.inf,
        )
        motion_capacity = max(0.0, envelope_capacity - vehicle_length)
        capture_target = float(site.get("capture_room_utilization", 1.0))
        target_failures = [
            case for case in cases
            if bool(case.get("enforce_capture_target", False))
            and float(case["room_utilization"]) > capture_target + 2.0e-4
        ]
        if target_failures:
            worst = max(target_failures, key=lambda item: float(item["room_utilization"]))
            raise RuntimeError(
                f"room comfort target failed for {worst['name']}: utilization "
                f"{float(worst['room_utilization']):.3f} exceeds the reviewed "
                f"{capture_target:.3f} capture target"
            )
        self._room_report = {
            "physical_room_length_m": room_length,
            "physical_room_width_m": room_width,
            "wall_clearance_m": clearance,
            "clear_room_length_m": clear_length,
            "clear_room_width_m": clear_width,
            "physical_room_diagonal_m": math.hypot(room_length, room_width),
            "clear_room_diagonal_m": math.hypot(clear_length, clear_width),
            "planned_straight_lane_heading_deg": heading_deg,
            "usable_straight_m": motion_capacity,
            "usable_width_m": min(clear_length, clear_width),
            "max_test_speed_mps": float(site["max_test_speed_mps"]),
            "conservative_braking_mps2": float(site["conservative_braking_mps2"]),
            "capture_room_utilization_target": capture_target,
            "max_useful_straight_capture_s": float(site.get("max_useful_straight_capture_s", 0.0)),
            "full_circle_revolutions": float(site.get("full_circle_revolutions", 0.0)),
            "max_room_steering_angle_rad": steering_angle_cap,
            "conservative_one_pass_distance_m": worst_distance["estimated_distance_m"],
            "worst_case": worst_distance["name"],
            "highest_room_utilization_case": worst_utilization["name"],
            "highest_room_utilization": worst_utilization["room_utilization"],
            "stage_cases": cases,
            "fixed_lidar_features_required": bool(site.get("require_fixed_lidar_features", True)),
            "status": "pass",
        }

    def _room_cases(self, steering_angle_cap: float | None = None) -> list[dict[str, float | str]]:
        """Conservatively assess the actual motion windows, not a fake global max.

        A 2 s centre capture occurs at 0.8 m/s, whereas a 3 m/s ERPM plateau is
        only 0.4 s.  Treating those as one 3 m/s, 2 s manoeuvre would reject a
        safe campaign, while retaining the old 0.6 s global value would hide
        the longer steering pass.  These cases are derived from the frozen
        profile and are written to the manifest for audit.
        """
        site = self.combined_cfg.get("site", self.suite["site"])
        vehicle = float(site["vehicle_length_m"])
        vehicle_width = float(site.get("vehicle_width_m", vehicle))
        braking = float(site.get("conservative_braking_mps2", 0.0))
        if braking <= 0.0:
            raise RuntimeError("site.conservative_braking_mps2 must be positive for room preflight")

        def case(name: str, speed: float, moving_s: float, *, radius_m: float | None = None,
                 turn_duration_s: float | None = None,
                 enforce_capture_target: bool = False) -> dict[str, Any]:
            if speed <= 0.0 or moving_s <= 0.0:
                raise RuntimeError(f"room case {name} has non-positive speed or duration")
            brake_distance = float(speed) * float(speed) / (2.0 * braking)
            result: dict[str, float | str] = {
                "name": name,
                "speed_mps": float(speed),
                "moving_s": float(moving_s),
                "conservative_braking_mps2": braking,
                "braking_distance_m": brake_distance,
                # Complete along-lane envelope: centre travel, emergency
                # braking and the vehicle body. Wall clearance is applied once
                # by _validate_room around the physical room, not hidden here.
                "motion_distance_m": float(speed) * float(moving_s) + brake_distance,
                "estimated_distance_m": float(speed) * float(moving_s) + brake_distance + vehicle,
                "required_width_m": vehicle_width,
                "enforce_capture_target": bool(enforce_capture_target),
            }
            if radius_m is not None:
                if radius_m <= 0.0:
                    raise RuntimeError(f"room case {name} has invalid turn radius")
                # A re-positioned left/right arc needs only the swept lateral
                # envelope of that finite manoeuvre, not the diameter of a
                # hypothetical full circle.  Once it reaches pi rad, the
                # lateral excursion has already reached its maximum 2R.
                if turn_duration_s is None:
                    arc_angle = math.pi
                else:
                    if turn_duration_s <= 0.0:
                        raise RuntimeError(f"room case {name} has invalid turn duration")
                    arc_angle = min(math.pi, abs(float(speed) * float(turn_duration_s) / float(radius_m)))
                lateral_excursion = float(radius_m) * (1.0 - math.cos(arc_angle))
                result.update({
                    "turn_radius_m": float(radius_m),
                    "turn_arc_angle_rad": float(arc_angle),
                    "turn_lateral_excursion_m": lateral_excursion,
                    "required_width_m": lateral_excursion + vehicle_width,
                })
            return result

        def circle_case(name: str, speed: float, radius_m: float, capture_s: float, *,
                        extra_tangent_m: float = 0.0,
                        preserve_steering_during_stop: bool = False) -> dict[str, Any]:
            """Footprint-aware complete-circle envelope.

            The circumscribed body radius accounts for every vehicle yaw.  A
            centred stop stays on the circle when steering is preserved; other
            stages reserve a conservative tangent braking extension.
            """
            if min(speed, radius_m, capture_s) <= 0.0:
                raise RuntimeError(f"room circle case {name} has invalid geometry")
            body_radius = 0.5 * math.hypot(vehicle, vehicle_width)
            brake_distance = float(speed) * float(speed) / (2.0 * braking)
            stop_extension = 0.0 if preserve_steering_during_stop else brake_distance
            swept_diameter = 2.0 * float(radius_m) + 2.0 * body_radius
            required_side = swept_diameter + max(0.0, float(extra_tangent_m)) + stop_extension
            return {
                "name": name,
                "speed_mps": float(speed),
                "moving_s": float(capture_s),
                "capture_s": float(capture_s),
                "capture_mode": "full_circle",
                "planned_revolutions": float(self.erpm_cfg.get("room_capture_policy", {}).get(
                    "full_circle_revolutions", 1.0
                )),
                "turn_radius_m": float(radius_m),
                "turn_arc_angle_rad": 2.0 * math.pi,
                "turn_lateral_excursion_m": 2.0 * float(radius_m),
                "vehicle_circumscribed_radius_m": body_radius,
                "braking_distance_m": brake_distance,
                "planned_stop_extension_m": stop_extension,
                "extra_tangent_m": max(0.0, float(extra_tangent_m)),
                "motion_distance_m": 2.0 * math.pi * float(radius_m),
                "estimated_distance_m": required_side,
                "required_width_m": required_side,
                "room_geometry": "axis_aligned_circle",
                "enforce_capture_target": True,
            }

        def pulse_case(name: str, initial_speed: float, *, startup_s: float,
                       pre_pulse_s: float, pulse_s: float, recovery_s: float,
                       drive_accel_mps2: float) -> dict[str, float | str]:
            """Bound a current/acceleration pulse by a conservative speed profile.

            The former room calculation treated the initial speed as constant.
            That is unsafe for a raw-current pulse: this profile assumes the
            car is already at the initial speed during setup, accelerates at the
            configured safe upper bound throughout the pulse, then continues at
            the resulting peak until active braking starts.
            """
            if initial_speed <= 0.0 or min(startup_s, pre_pulse_s, pulse_s, recovery_s) < 0.0:
                raise RuntimeError(f"room pulse case {name} has invalid speed or duration")
            acceleration = max(0.0, float(drive_accel_mps2))
            peak_speed = float(initial_speed) + acceleration * float(pulse_s)
            distance = (
                float(initial_speed) * (float(startup_s) + float(pre_pulse_s) + float(pulse_s))
                + 0.5 * acceleration * float(pulse_s) ** 2
                + peak_speed * float(recovery_s)
            )
            brake_distance = peak_speed * peak_speed / (2.0 * braking)
            return {
                "name": name,
                "initial_speed_mps": float(initial_speed),
                "assumed_drive_accel_mps2": acceleration,
                "pulse_s": float(pulse_s),
                "peak_speed_mps": peak_speed,
                "conservative_braking_mps2": braking,
                "braking_distance_m": brake_distance,
                "motion_distance_m": distance + brake_distance,
                "estimated_distance_m": distance + brake_distance + vehicle,
                "required_width_m": vehicle_width,
            }

        steering = self.steering_cfg
        centre = steering["centre_trim"]
        observability = steering["observability"]
        static = steering["static_map"]
        response = steering["response"]
        if steering_angle_cap is None:
            steering_angle_cap = self._room_steering_angle_cap()
        try:
            wheelbase = float(steering["hardware"]["wheelbase_m"])
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError("steering hardware wheelbase is required for room-turn geometry") from exc
        if not math.isfinite(wheelbase) or wheelbase <= 0.0:
            raise RuntimeError("steering hardware wheelbase must be a finite positive value")

        def turn_radius(fraction: float) -> float:
            angle = abs(float(fraction)) * float(steering_angle_cap)
            if not 0.0 < angle < 0.5 * math.pi:
                raise RuntimeError("room-turn steering fraction produces an invalid road-wheel angle")
            return wheelbase / math.tan(angle)

        steering_policy = steering.get("room_capture_policy", {})
        steering_startup_s = float(steering_policy.get(
            "steering_startup_planning_s",
            float(steering["motion_startup"]["minimum_startup_s"])
            + float(steering["motion_startup"]["stability_window_s"]),
        ))
        clear_length_fallback = (
            float(site["room_length_m"]) - 2.0 * float(site.get("wall_clearance_m", 0.0))
        )
        clear_width_fallback = (
            float(site.get("room_width_m", site["room_length_m"]))
            - 2.0 * float(site.get("wall_clearance_m", 0.0))
        )
        target_utilization = float(steering_policy.get(
            "target_room_utilization", site.get("capture_room_utilization", 1.0)
        ))
        target_length = float(steering_policy.get(
            "clear_room_length_m", clear_length_fallback
        )) * target_utilization
        target_width = float(steering_policy.get(
            "clear_room_width_m", clear_width_fallback
        )) * target_utilization
        target_circle_side = float(steering_policy.get(
            "target_circle_side_m", min(target_length, target_width)
        ))
        body_radius = 0.5 * math.hypot(vehicle, vehicle_width)
        maximum_capture_s = float(steering_policy.get("maximum_useful_capture_s", 30.0))
        revolutions = float(steering_policy.get("full_circle_revolutions", 1.0))
        heading = math.radians(float(site.get("straight_lane_heading_deg", 0.0)))
        cos_heading = abs(math.cos(heading))
        sin_heading = abs(math.sin(heading))

        def steering_capture_plan(fraction: float, speed: float, minimum_s: float, *,
                                  centre_before_s: float = 0.0,
                                  return_after_s: float = 0.0) -> dict[str, Any]:
            radius = turn_radius(fraction)
            brake_distance = speed * speed / (2.0 * braking)
            tangent = speed * (
                centre_before_s + return_after_s
                + (steering_startup_s if centre_before_s > 0.0 else 0.0)
            )
            maximum_radius = 0.5 * (
                target_circle_side - 2.0 * body_radius - tangent - brake_distance
            )
            if radius <= maximum_radius + 1.0e-9:
                return {
                    "capture_mode": "full_circle",
                    "capture_s": max(minimum_s, revolutions * 2.0 * math.pi * radius / speed),
                    "radius_m": radius,
                    "extra_tangent_m": tangent,
                }

            def bounded_fits(capture_s: float) -> bool:
                straight_s = steering_startup_s + centre_before_s + return_after_s
                turn_s = capture_s if centre_before_s > 0.0 else steering_startup_s + capture_s
                envelope_length = speed * (straight_s + capture_s) + brake_distance + vehicle
                arc_angle = min(math.pi, speed * turn_s / radius)
                envelope_width = vehicle_width + radius * (1.0 - math.cos(arc_angle))
                projected_length = envelope_length * cos_heading + envelope_width * sin_heading
                projected_width = envelope_length * sin_heading + envelope_width * cos_heading
                return projected_length <= target_length and projected_width <= target_width

            if not bounded_fits(minimum_s):
                capture_s = minimum_s
            elif bounded_fits(maximum_capture_s):
                capture_s = maximum_capture_s
            else:
                low = minimum_s
                high = maximum_capture_s
                for _ in range(48):
                    midpoint = 0.5 * (low + high)
                    if bounded_fits(midpoint):
                        low = midpoint
                    else:
                        high = midpoint
                capture_s = low
            return {
                "capture_mode": "bounded_arc",
                "capture_s": capture_s,
                "radius_m": radius,
                "extra_tangent_m": tangent,
            }

        steering_cases: list[dict[str, Any]] = [
            case(
                "steering centre training", float(centre["speed_mps"]),
                steering_startup_s + float(centre["capture_s"]),
            ),
            case(
                "steering centre validation",
                max(float(item["speed_mps"]) for item in centre.get(
                    "validation_conditions",
                    [{"speed_mps": centre.get("validation_speed_mps", centre["speed_mps"])}],
                )),
                steering_startup_s + float(centre.get("validation_capture_s", centre["capture_s"])),
            ),
            case(
                "steering observability straight",
                max(map(float, observability["straight_speeds_mps"])),
                steering_startup_s + float(observability["straight_capture_s"]),
            ),
            case(
                "steering observability turn", float(observability["turn_speed_mps"]),
                steering_startup_s + float(observability["turn_capture_s"]),
                radius_m=turn_radius(float(observability["gentle_turn_fraction_of_safe_span"])),
                turn_duration_s=steering_startup_s + float(observability["turn_capture_s"]),
            ),
        ]

        for family, fractions in (
            ("training", static.get("training_fractions", [])),
            ("holdout", static.get("validation_fractions", [])),
        ):
            for fraction_value in map(float, fractions):
                speed = float(static["speed_mps"])
                plan = steering_capture_plan(fraction_value, speed, float(static["capture_s"]))
                name = f"steering static map {family} fraction {fraction_value:.2f}"
                if plan["capture_mode"] == "full_circle":
                    planned = circle_case(
                        name, speed, float(plan["radius_m"]), float(plan["capture_s"]),
                    )
                else:
                    planned = case(
                        name, speed, steering_startup_s + float(plan["capture_s"]),
                        radius_m=float(plan["radius_m"]),
                        turn_duration_s=steering_startup_s + float(plan["capture_s"]),
                        enforce_capture_target=True,
                    )
                    planned.update({
                        "capture_mode": "bounded_arc",
                        "capture_s": float(plan["capture_s"]),
                        "planned_revolutions": 0.0,
                    })
                steering_cases.append(planned)

        for family, conditions in (
            ("training", response["conditions"]),
            ("validation", response.get("validation_conditions", response["conditions"])),
        ):
            for condition in conditions:
                speed = float(condition["speed_mps"])
                for fraction_value in map(float, condition["target_fractions"]):
                    plan = steering_capture_plan(
                        fraction_value, speed, float(response["step_hold_s"]),
                        centre_before_s=float(response["centre_hold_s"]),
                        return_after_s=float(response["return_hold_s"]),
                    )
                    name = (
                        f"steering response {family} v={speed:.2f} "
                        f"fraction={fraction_value:.2f}"
                    )
                    if plan["capture_mode"] == "full_circle":
                        planned = circle_case(
                            name, speed, float(plan["radius_m"]), float(plan["capture_s"]),
                            extra_tangent_m=float(plan["extra_tangent_m"]),
                        )
                    else:
                        moving_s = (
                            steering_startup_s + float(response["centre_hold_s"])
                            + float(plan["capture_s"]) + float(response["return_hold_s"])
                        )
                        planned = case(
                            name, speed, moving_s,
                            radius_m=float(plan["radius_m"]),
                            turn_duration_s=float(plan["capture_s"]),
                            enforce_capture_target=True,
                        )
                        planned.update({
                            "capture_mode": "bounded_arc",
                            "capture_s": float(plan["capture_s"]),
                            "planned_revolutions": 0.0,
                        })
                    steering_cases.append(planned)

        erpm = self.erpm_cfg
        startup = erpm["motion_startup"]
        policy = erpm.get("room_capture_policy", {})
        erpm_startup_s = float(policy.get(
            "erpm_startup_planning_s",
            float(startup["minimum_startup_s"]) + float(startup["stability_window_s"]),
        ))
        raw_switch_s = float(policy.get(
            "erpm_raw_switch_settle_s", startup.get("raw_erpm_switch_settle_s", 0.0)
        ))
        active_stop_s = float(policy.get(
            "erpm_active_stop_reserve_s",
            startup.get("active_stop_max_brake_s", startup.get("active_stop_brake_s", 0.0)),
        ))
        envelope = erpm["operating_envelope"]

        def mapped_duration(spec: dict[str, Any], map_key: str,
                            default_key: str, speed: float) -> float:
            values = spec.get(map_key, {})
            if isinstance(values, dict):
                candidates: list[tuple[float, float]] = []
                for key, value in values.items():
                    try:
                        candidates.append((abs(float(key) - speed), float(value)))
                    except (TypeError, ValueError):
                        continue
                if candidates:
                    error, duration = min(candidates, key=lambda item: item[0])
                    if error <= 1.0e-6 * max(1.0, abs(speed)):
                        return duration
            return float(spec[default_key])

        def settle_s(speed: float) -> float:
            distance = max(0.0, float(startup.get("pre_capture_settle_distance_m", 0.0)))
            minimum = max(0.0, float(startup.get("pre_capture_settle_min_s", 0.0)))
            maximum = max(minimum, float(startup.get("pre_capture_settle_max_s", minimum)))
            return min(maximum, max(minimum, distance / max(abs(speed), 0.05)))

        erpm_cases: list[dict[str, Any]] = []

        def add_plateaus(name: str, spec: dict[str, Any], speeds_key: str,
                         map_key: str, default_key: str, *, raw_erpm: bool) -> None:
            for speed in map(float, spec[speeds_key]):
                capture_s = mapped_duration(spec, map_key, default_key, speed)
                moving_s = (
                    erpm_startup_s + settle_s(speed) + capture_s + active_stop_s
                    + (raw_switch_s if raw_erpm else 0.0)
                )
                planned = case(
                    f"{name} v={speed:.2f}", speed, moving_s,
                    enforce_capture_target=True,
                )
                planned.update({"capture_s": capture_s, "capture_mode": "extended_straight"})
                erpm_cases.append(planned)

        add_plateaus(
            "ERPM moving-sensor observability", erpm["observability"],
            "straight_probe_speeds_mps", "straight_probe_capture_s_by_speed",
            "straight_probe_capture_s", raw_erpm=False,
        )
        for name, section, raw in (
            ("ERPM low-speed launch", "low_speed_launch", True),
            ("ERPM map training", "raw_erpm_map_training", True),
            ("ERPM map hold-out", "raw_erpm_map_holdout", True),
            ("VEL_TO_ERPM pipeline audit", "vel_to_erpm_pipeline_audit", False),
        ):
            spec = erpm[section]
            speeds_key = "speed_commands_mps" if section == "vel_to_erpm_pipeline_audit" else "nominal_speeds_mps"
            add_plateaus(name, spec, speeds_key, "capture_s_by_speed", "capture_s", raw_erpm=raw)

        response_spec = erpm["raw_erpm_response"]
        for family, steps in (
            ("training", response_spec["steps_mps"]),
            ("validation", response_spec.get("validation_steps_mps", [])),
        ):
            for baseline_speed, target_speed in steps:
                target = float(target_speed)
                capture_s = mapped_duration(
                    response_spec, "response_capture_s_by_target_speed",
                    "response_capture_s", target,
                )
                moving_s = (
                    erpm_startup_s + raw_switch_s
                    + float(response_spec["pre_step_hold_s"])
                    + capture_s + active_stop_s
                )
                planned = case(
                    f"ERPM response {family} {float(baseline_speed):.2f}->{target:.2f}",
                    target, moving_s, enforce_capture_target=True,
                )
                planned.update({"capture_s": capture_s, "capture_mode": "extended_straight"})
                erpm_cases.append(planned)

        coast = erpm["coastdown"]
        for family, speeds, default_key in (
            ("training", coast["initial_speeds_mps"], "coast_capture_s"),
            ("validation", coast.get("validation_initial_speeds_mps", []), "validation_coast_capture_s"),
        ):
            if default_key not in coast:
                default_key = "coast_capture_s"
            for speed in map(float, speeds):
                capture_s = mapped_duration(
                    coast, "coast_capture_s_by_initial_speed", default_key, speed,
                )
                moving_s = (
                    erpm_startup_s + raw_switch_s + float(coast["pre_coast_hold_s"])
                    + capture_s + active_stop_s
                )
                planned = case(
                    f"ERPM coast-down {family} v={speed:.2f}", speed, moving_s,
                    enforce_capture_target=True,
                )
                planned.update({"capture_s": capture_s, "capture_mode": "extended_coastdown"})
                erpm_cases.append(planned)

        current_drive_speeds = [
            *(float(item["initial_speed_mps"])
              for section in ("raw_current_training", "raw_current_holdout")
              for item in erpm[section]["drive_conditions"]),
        ]
        accel = erpm["accel_to_current_interface"]
        accel_speeds = [
            *map(float, accel["initial_speeds_mps"]),
            *map(float, accel.get("validation_initial_speeds_mps", [])),
        ]
        erpm_cases.extend([
            pulse_case(
                "ERPM raw-current drive pulse", max(current_drive_speeds),
                startup_s=erpm_startup_s + raw_switch_s,
                pre_pulse_s=max(
                    float(erpm["raw_current_training"]["pre_pulse_hold_s"]),
                    float(erpm["raw_current_holdout"]["pre_pulse_hold_s"]),
                ),
                pulse_s=max(map(float, envelope["high_demand_pulse_duration_s"].values())),
                recovery_s=max(
                    float(erpm["raw_current_training"].get("post_pulse_capture_s", 0.0)),
                    float(erpm["raw_current_holdout"].get("post_pulse_capture_s", 0.0)),
                ) + active_stop_s,
                drive_accel_mps2=float(envelope.get(
                    "max_raw_current_drive_accel_mps2",
                    envelope.get("maximum_test_accel_mps2", 0.0),
                )),
            ),
            pulse_case(
                "ERPM acceleration interface and hold-out", max(accel_speeds),
                startup_s=erpm_startup_s + raw_switch_s,
                pre_pulse_s=float(accel["pre_pulse_hold_s"]),
                pulse_s=max(
                    float(accel.get("pulse_capture_s", envelope["dynamic_capture_min_s"])),
                    float(accel.get("validation_pulse_capture_s", 0.0)),
                ),
                recovery_s=float(accel["pre_pulse_hold_s"]) + active_stop_s,
                drive_accel_mps2=max(
                    0.0,
                    max(map(float, accel["acceleration_commands_mps2"])),
                    max(map(float, accel.get("validation_acceleration_commands_mps2", [0.0]))),
                ),
            ),
        ])

        candidate = erpm["candidate_verification"]
        for speed in map(float, candidate["velocity_holdout_commands_mps"]):
            capture_s = mapped_duration(
                candidate, "velocity_capture_s_by_speed", "capture_s", speed,
            )
            moving_s = erpm_startup_s + settle_s(speed) + capture_s + active_stop_s
            planned = case(
                f"selected odometry velocity hold-out v={speed:.2f}", speed, moving_s,
                enforce_capture_target=True,
            )
            planned.update({"capture_s": capture_s, "capture_mode": "extended_straight"})
            erpm_cases.append(planned)
        erpm_cases.append(pulse_case(
            "selected odometry acceleration hold-out",
            max(map(float, candidate["acceleration_initial_speeds_mps"])),
            startup_s=erpm_startup_s + raw_switch_s,
            pre_pulse_s=1.2,
            pulse_s=float(candidate["capture_s"]),
            recovery_s=active_stop_s,
            drive_accel_mps2=max(
                0.0, max(map(float, candidate["acceleration_holdout_commands_mps2"])),
            ),
        ))

        lateral = erpm.get("lateral_stiffness", {})
        if lateral:
            nominal_wheelbase = float(lateral.get("nominal_wheelbase_m", 0.324))
            for family, speeds, angles, minimum_key in (
                ("training", lateral.get("speeds_mps", []), lateral.get("steering_angles_rad", []), "capture_s"),
                ("validation", lateral.get("validation_speeds_mps", []), lateral.get("validation_steering_angles_rad", []), "validation_capture_s"),
            ):
                minimum_s = float(lateral.get(minimum_key, lateral.get("capture_s", 0.0)))
                for speed in map(float, speeds):
                    for angle in map(float, angles):
                        radius = nominal_wheelbase / abs(math.tan(angle))
                        maximum_radius = 0.5 * (
                            target_circle_side - 2.0 * body_radius
                            - speed * speed / (2.0 * braking)
                        )
                        name = f"quasi-steady lateral {family} v={speed:.2f} delta={angle:.2f}"
                        if radius <= maximum_radius + 1.0e-9:
                            capture_s = max(
                                minimum_s, revolutions * 2.0 * math.pi * radius / speed,
                            )
                            erpm_cases.append(circle_case(
                                name, speed, radius, capture_s,
                                preserve_steering_during_stop=True,
                            ))
                        else:
                            moving_s = (
                                erpm_startup_s + float(lateral.get("pre_turn_hold_s", 0.0))
                                + float(lateral.get("turn_settle_s", 0.0))
                                + minimum_s + active_stop_s
                            )
                            planned = case(
                                name, speed, moving_s, radius_m=radius,
                                turn_duration_s=float(lateral.get("turn_settle_s", 0.0)) + minimum_s,
                                enforce_capture_target=True,
                            )
                            planned.update({
                                "capture_mode": "bounded_arc", "capture_s": minimum_s,
                                "planned_revolutions": 0.0,
                            })
                            erpm_cases.append(planned)
        return steering_cases + erpm_cases

    def _write_policy(self) -> None:
        steering = _load(STEERING_ROOT / "config" / "topics.yaml")
        erpm = _load(ERPM_ROOT / "config" / "topics.yaml")
        recording = copy.deepcopy(erpm.get("recording", {}))
        self._recording = recording
        redundancy = list(dict.fromkeys([
            *steering.get("redundancy_topics", []),
            *erpm.get("redundancy_topics", []),
        ]))
        policy = {
            "recording": recording,
            "steering_required": steering.get("required", {}),
            "erpm_required": erpm.get("required", {}),
            "redundancy_topics": redundancy,
        }
        _dump(self.session / "recording_policy_snapshot.yaml", policy)

    def _required_topics(self, stage: StageSpec) -> tuple[list[str], list[str]]:
        policy = _load(self.session / "recording_policy_snapshot.yaml")
        groups = policy["steering_required"] if stage.kind == "steering" else policy["erpm_required"]
        required = list(groups[stage.topic_group])
        recorded = list(dict.fromkeys(required + list(policy.get("redundancy_topics", []))))
        return required, recorded

    def _source_path(self) -> Path:
        return self.workspace / str(self.campaign["config_relpath"])

    def _active_patch(self) -> dict[str, Any]:
        return copy.deepcopy(self.state.get("active_patch", {}))

    @staticmethod
    def _merge_patch(base: dict[str, Any], update: dict[str, Any]) -> dict[str, Any]:
        result = copy.deepcopy(base)
        for key, values in update.items():
            if isinstance(values, dict) and isinstance(result.get(key), dict):
                result[key].update(values)
            else:
                result[key] = copy.deepcopy(values)
        return result

    def _temporary_setup(self, tx: ConfigManager, stage: StageSpec) -> None:
        if stage.kind == "manual":
            return
        state = self.state
        active = self._active_patch()
        if active:
            tx.apply(active, "build_restore_session_patch")
        setup: dict[str, Any] = {
            "global": {"servo_min": 0.0, "servo_max": 1.0, "speed_to_erpm_offset": 0.0},
            "ackermann_to_vesc_node": {
                "operation_mode": "VEL_TO_ERPM",
                "accel_to_current_gain": 0.0,
                "accel_to_brake_gain": 0.0,
            },
        }
        if stage.kind == "erpm" and stage.key in {"accel_interface", "accel_interface_validation"} and state.get("accel_patch"):
            setup["ackermann_to_vesc_node"].update(state["accel_patch"].get("ackermann_to_vesc_node", {}))
            setup["ackermann_to_vesc_node"]["operation_mode"] = "ACCEL_TO_CURRENT"
        tx.apply(setup, f"build_stage_{stage.key}_setup")

    def _launch_and_bag(self, stage: StageSpec, capture: Callable[[Path], dict[str, Any]]) -> dict[str, Any]:
        if stage.kind == "manual":
            stage_dir = self.session / stage.directory
            if stage_dir.exists():
                archive = self.session / "incomplete_attempts" / f"{stage.directory}_{dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')}"
                archive.parent.mkdir(parents=True, exist_ok=True)
                shutil.move(str(stage_dir), str(archive))
            stage_dir.mkdir(parents=True, exist_ok=True)
            result = capture(stage_dir)
            _json_dump(stage_dir / "runtime_result.json", result)
            return {
                "runtime": result,
                "bag_verification": {"ok": True, "kind": "manual", "note": "No ROS bag is expected for direct physical metrology."},
            }
        required, recorded = self._required_topics(stage)
        # Importing the ERPM bagger avoids a second recording implementation; it
        # is topic-agnostic and records the exact scoped list supplied here.
        sys.path.insert(0, str(ERPM_ROOT))
        from erpm_calibration.bagging import start_bag, stop_bag

        stage_dir = self.session / stage.directory
        if stage_dir.exists():
            archive = self.session / "incomplete_attempts" / f"{stage.directory}_{dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')}"
            archive.parent.mkdir(parents=True, exist_ok=True)
            shutil.move(str(stage_dir), str(archive))
        bag = None
        stack = StackProcess(self, stage.kind, stage.key)
        result: dict[str, Any] | None = None
        verification: dict[str, Any] | None = None
        try:
            stack.start()
            bag = start_bag(stage_dir, self._recording, required, recorded, ERPM_ROOT)
            result = capture(stage_dir)
        finally:
            if bag is not None:
                verification = stop_bag(bag)
            stack.stop()
        if verification is not None and not verification.get("ok", False):
            raise RuntimeError(f"{stage.key} missing required topics: {verification.get('missing_or_empty_required_topics')}")
        if result is None:
            raise RuntimeError(f"stage did not return a runtime result: {stage.key}")
        return {"runtime": result, "bag_verification": verification}

    def _run_script(self, root: Path, script: str, args: list[str], log_name: str) -> None:
        log = self.session / "analysis" / log_name
        log.parent.mkdir(parents=True, exist_ok=True)
        command = [sys.executable, str(root / "analysis" / script), *args]
        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, check=False, cwd=root / "analysis")
        log.write_text(result.stdout, encoding="utf-8")
        if result.returncode:
            raise RuntimeError(f"{script} failed; inspect {log}")

    def _complete_odometry_candidate_patch(self) -> dict[str, Any]:
        """Combine selected odometry with the already gated drive candidates.

        The shadow verification launch consumes one self-contained file.  The
        odometry selector owns command/wheel/fusion parameters; the previously
        validated speed, drag and current stages own the acceleration adapter.
        """
        analysis = self.session / "analysis"
        patch = _load(analysis / "selected_odometry_candidate_patch.yaml")
        speed = _load(analysis / "erpm_speed_map_report.yaml")
        coast = _load(analysis / "coastdown_drag_report.yaml")
        current = _load(analysis / "current_acceleration_report.yaml")
        ack = patch.setdefault("ackermann_to_vesc_node", {}).setdefault("ros__parameters", {})
        odom = patch.setdefault("vesc_to_odom_node", {}).setdefault("ros__parameters", {})
        ack.update({
            "acceleration_command_model": "scalar",
            "accel_to_current_gain": float(current["candidate_accel_to_current_gain"]),
            "accel_to_brake_gain": float(current["candidate_accel_to_brake_gain"]),
            "accel_deadzone": float(current["candidate_accel_deadzone_mps2"]),
            "accel_drag_coulomb": float(coast["accel_drag_coulomb_mps2"]),
            "accel_drag_viscous": float(coast["accel_drag_viscous_per_s"]),
            "accel_drag_quadratic": float(coast["accel_drag_quadratic_per_m"]),
            "hold_speed_min_mps": float(self.erpm_cfg["operating_envelope"].get("hold_speed_min_mps", 0.12)),
            "max_drive_current": float(self.erpm_cfg["operating_envelope"]["approved_drive_test_current_a"]),
            "max_brake_current": float(self.erpm_cfg["operating_envelope"]["approved_brake_test_current_a"]),
            "stop_speed_deadzone": float(speed["candidate_stop_speed_deadzone_mps"]),
        })
        odom["speed_deadband"] = float(speed["candidate_odom_speed_deadband_mps"])
        _dump(analysis / "selected_odometry_candidate_patch.yaml", patch)
        return patch

    def _run_lidar_motion(self, stage: StageSpec, bag: Path) -> None:
        """Run the unified displacement-baseline estimator for this stage."""
        log = self.session / "analysis" / f"{stage.key}_lidar.log"
        snapshot = self.session / (
            "steering_calibration_config_snapshot.yaml" if stage.kind == "steering"
            else "erpm_calibration_config_snapshot.yaml"
        )
        command = [
            sys.executable,
            str(ROOT / "calibration_suite" / "lidar_motion.py"),
            str(bag),
            "--config", str(snapshot),
            "--kind", stage.kind,
        ]
        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, check=False, cwd=ROOT)
        log.parent.mkdir(parents=True, exist_ok=True)
        log.write_text(result.stdout, encoding="utf-8")
        if result.returncode:
            raise RuntimeError(f"unified LiDAR motion analysis failed; inspect {log}")

    def _validate_full_resolution_scan(self, stage: StageSpec) -> dict[str, Any]:
        """Reject a motion capture that did not retain the native Hokuyo scan."""
        import pandas as pd

        scan_path = self.session / stage.directory / "derived" / "scan_index.parquet"
        report_path = self.session / "analysis" / f"{stage.key}_scan_resolution_report.json"
        failures: list[str] = []
        if not scan_path.is_file():
            report = {
                "accepted": False,
                "required_minimum_ranges_per_scan": 1080,
                "required_median_rate_hz": 35.0,
                "failures": ["scan_index.parquet is missing"],
            }
            _json_dump(report_path, report)
            raise RuntimeError(
                f"{stage.key} has no exported LiDAR scan index; inspect {report_path}"
            )

        scans = pd.read_parquet(scan_path).sort_values("bag_ns")
        counts = scans.get("range_count", pd.Series(dtype=float)).to_numpy(dtype=float)
        counts = counts[~pd.isna(counts)]
        timestamps = scans.get("bag_ns", pd.Series(dtype=float)).to_numpy(dtype=float)
        timestamps = timestamps[~pd.isna(timestamps)]
        positive_dt = pd.Series(timestamps).diff().to_numpy(dtype=float) / 1.0e9
        positive_dt = positive_dt[(positive_dt > 0.0) & (positive_dt < 1.0)]
        median_rate_hz = float(1.0 / pd.Series(positive_dt).median()) if len(positive_dt) else math.nan
        minimum_count = int(counts.min()) if len(counts) else 0
        median_count = float(pd.Series(counts).median()) if len(counts) else 0.0
        if minimum_count < 1080:
            failures.append(
                f"at least one scan has only {minimum_count} ranges; 1080 are required"
            )
        if not math.isfinite(median_rate_hz) or median_rate_hz < 35.0:
            failures.append(
                f"median scan rate is {median_rate_hz:.2f} Hz; at least 35 Hz is required"
            )
        report = {
            "accepted": not failures,
            "scan_count": int(len(scans)),
            "required_minimum_ranges_per_scan": 1080,
            "minimum_ranges_per_scan": minimum_count,
            "median_ranges_per_scan": median_count,
            "required_median_rate_hz": 35.0,
            "median_rate_hz": median_rate_hz,
            "failures": failures,
        }
        _json_dump(report_path, report)
        if failures:
            raise RuntimeError(
                f"{stage.key} did not retain the full 1080-point, 40 Hz LiDAR stream; "
                f"inspect {report_path}"
            )
        return report

    def _analyse(self, stage: StageSpec) -> dict[str, Any]:
        if stage.kind == "manual":
            if stage.key != "physical_metrology":
                raise RuntimeError(f"unknown manual stage: {stage.key}")
            from .metrology import analyse_measurements
            report = analyse_measurements(self.session)
            if not bool(report.get("accepted_for_lateral_identification", False)):
                raise RuntimeError(
                    "physical metrology is incomplete; fill the session-local physical_measurements.yaml "
                    "and rerun this stage"
                )
            # Metrology intentionally updates only the frozen session inputs
            # (wheelbase and LiDAR-to-base geometry), never source VESC files.
            # Reload them here so every subsequent runtime/analysis uses the
            # measured transform rather than the profile placeholder.
            self.steering_cfg = _load(self.session / "steering_calibration_config_snapshot.yaml")
            self.erpm_cfg = _load(self.session / "erpm_calibration_config_snapshot.yaml")
            self.combined_cfg = _load(self.session / "calibration_config_snapshot.yaml")
            return {"physical_metrology": report}
        bag = self.session / stage.directory / "bag"
        root = STEERING_ROOT if stage.kind == "steering" else ERPM_ROOT
        exporter = "export_bag.py"
        self._run_script(root, exporter, [str(bag)], f"{stage.key}_export.log")
        lidar_motion_stage = (
            stage.topic_group == "full_motion"
            or stage.kind == "erpm"
            and stage.topic_group in {
                "raw_erpm", "raw_current", "ackermann_vel", "ackermann_accel",
                "candidate_ackermann_vel", "candidate_ackermann_accel",
            }
        )
        if lidar_motion_stage:
            self._validate_full_resolution_scan(stage)
            self._run_lidar_motion(stage, bag)

        if stage.key == "steering_centre":
            self._run_script(STEERING_ROOT, "fit_centre.py", [str(self.session)], "fit_centre.log")
            centre = json.loads((self.session / "analysis" / "centre_trim_offline.json").read_text(encoding="utf-8"))
            if not bool(centre.get("accepted_for_update", False)):
                raise RuntimeError("steering centre fit did not pass the independent offline gate")
            return {"centre": centre}
        if stage.key == "steering_centre_validation":
            self._run_script(STEERING_ROOT, "validate_centre.py", [str(self.session)], "validate_centre.log")
            validation = json.loads((self.session / "analysis" / "centre_validation_report.json").read_text(encoding="utf-8"))
            if not bool(validation.get("accepted_for_validation", False)):
                raise RuntimeError("steering centre physical straightness validation failed")
            return {"centre_validation": validation}
        if stage.key == "steering_observability":
            self._run_script(STEERING_ROOT, "assess_icp_quality.py", [str(self.session)], "assess_icp_quality.log")
            icp = json.loads((self.session / "analysis" / "icp_observability_report.json").read_text(encoding="utf-8"))
            icp_gates = self.steering_cfg.get("analysis", {}).get("icp_observability", {})
            icp_failures = []
            # The centre/static fits consume robust motion windows, so the
            # preflight is deliberately based on those same moving windows. A
            # stationary point-to-line registration can be degenerate even in a
            # perfectly usable scene and remains a diagnostic rather than a
            # false hard block.
            if int(icp.get("moving_valid_windows", 0)) < int(icp_gates.get("min_moving_valid_windows", 12)):
                icp_failures.append("too few valid moving LiDAR windows")
            if float(icp.get("moving_window_valid_fraction") or 0.0) < float(icp_gates.get("min_moving_window_valid_fraction", 0.70)):
                icp_failures.append("moving LiDAR-window valid fraction is below the gate")
            if float(icp.get("moving_window_speed_median_mps") or 0.0) < float(icp_gates.get("min_moving_window_speed_mps", 0.20)):
                icp_failures.append("moving LiDAR windows do not show observable forward motion")
            moving_rmse = icp.get("moving_window_point_to_line_rmse_p95_m")
            if moving_rmse is None or float(moving_rmse) > float(icp_gates.get("max_moving_point_to_line_rmse_p95_m", 0.03)):
                icp_failures.append("moving LiDAR-window point-to-line RMSE exceeds the gate")
            if icp_failures:
                raise RuntimeError("steering observability gate failed: " + "; ".join(icp_failures))
            # Do not silently refit or move the already physically validated
            # centre here.  This stage characterises ICP noise for later fits;
            # any future centre change must go through the explicit validation
            # stage again.
            return {"observability": icp}
        if stage.key == "steering_static_training":
            self._run_script(STEERING_ROOT, "fit_static_map_training.py", [str(self.session), "--config", str(self.session / "steering_calibration_config_snapshot.yaml")], "fit_static_map_training.log")
            candidate = json.loads((self.session / "analysis" / "candidate_static_steering_map.json").read_text(encoding="utf-8"))
            if not bool(candidate.get("accepted_for_update", False)):
                raise RuntimeError("steering static-map training did not produce an update candidate")
            return {"static_map": candidate}
        if stage.key == "steering_static_holdout":
            self._run_script(STEERING_ROOT, "fit_static_map.py", [str(self.session), "--config", str(self.session / "steering_calibration_config_snapshot.yaml")], "fit_static_map.log")
            candidate = json.loads((self.session / "analysis" / "candidate_static_steering_map.json").read_text(encoding="utf-8"))
            if not bool(candidate.get("accepted_for_deployment", False)):
                raise RuntimeError("steering static-map hold-out rejected the candidate")
            self._run_script(STEERING_ROOT, "validate_static_map_applied.py", [str(self.session)], "validate_static_map_applied.log")
            applied = json.loads((self.session / "analysis" / "static_map_applied_validation.json").read_text(encoding="utf-8"))
            # ``fit_static_map.py`` intentionally reconstructs the frozen
            # training interpolation to evaluate C.  Preserve the actual
            # rebuilt linear patch alongside that evidence so the final
            # parameter inventory reports what was genuinely exercised, not
            # merely the diagnostic piecewise map.
            applied_patch = _load(self.session / "analysis" / "steering_map_vesc_patch.yaml")
            candidate["deployable_linear_patch"] = applied_patch
            candidate["applied_patch_validation"] = applied
            candidate["accepted_for_deployment"] = bool(candidate.get("accepted_for_deployment", False) and applied.get("accepted_for_validation", False))
            (self.session / "analysis" / "candidate_static_steering_map.json").write_text(json.dumps(candidate, indent=2) + "\n", encoding="utf-8")
            if not bool(applied.get("accepted_for_validation", False)):
                raise RuntimeError("rebuilt steering map patch failed its independent hold-out validation")
            return {"static_map_validation": candidate}
        if stage.key == "steering_response":
            self._run_script(STEERING_ROOT, "fit_response.py", [str(self.session)], "fit_steering_response.log")
            self._run_script(STEERING_ROOT, "summarize_simulation_seeds.py", [str(self.session)], "summarize_steering_response.log")
            response = json.loads((self.session / "analysis" / "command_to_effective_steering_response_summary.json").read_text(encoding="utf-8"))
            minimum = int(self.steering_cfg.get("analysis", {}).get("response", {}).get("min_valid_segments", 1))
            if int(response.get("segments_with_valid_effective_response", 0)) < minimum:
                raise RuntimeError(f"steering response has fewer than {minimum} valid effective-response segments")
            return {"response": response}
        if stage.key == "steering_response_validation":
            self._run_script(
                STEERING_ROOT, "fit_response.py",
                [str(self.session), "--stage-directory", stage.directory, "--output-prefix", "validation"],
                "fit_steering_response_validation.log",
            )
            self._run_script(STEERING_ROOT, "validate_response.py", [str(self.session)], "validate_steering_response.log")
            validation = json.loads((self.session / "analysis" / "steering_response_validation_report.json").read_text(encoding="utf-8"))
            if not bool(validation.get("accepted_for_validation", False)):
                raise RuntimeError("steering response hold-out rejected the training response model")
            return {"response_validation": validation}
        if stage.key == "longitudinal_observability":
            self._run_script(ERPM_ROOT, "assess_longitudinal_observability.py", [str(self.session)], "assess_longitudinal_observability.log")
            report = _load(self.session / "analysis" / "longitudinal_observability_report.yaml")
            if not bool(report.get("accepted_for_downstream", False)):
                raise RuntimeError("longitudinal moving-sensor observability gate failed")
            return {"longitudinal_observability": report}
        if stage.key == "low_speed_launch":
            self._run_script(ERPM_ROOT, "assess_low_speed_launch.py", [str(self.session)], "assess_low_speed_launch.log")
            report = _load(self.session / "analysis" / "low_speed_launch_report.yaml")
            if not bool(report.get("accepted_for_downstream", False)):
                raise RuntimeError("low-speed launch/dead-band capture did not meet its immediate analysis gate")
            return {"low_speed_launch": report}
        if stage.key == "erpm_map_training":
            self._run_script(ERPM_ROOT, "fit_speed_map_training.py", [str(self.session)], "fit_speed_map_training.log")
            report = _load(self.session / "analysis" / "erpm_speed_map_training_report.yaml")
            if not bool(report.get("accepted_for_update", False)):
                raise RuntimeError("ERPM training data did not produce an update candidate")
            return {"speed_map": report}
        if stage.key == "erpm_map_holdout":
            self._run_script(ERPM_ROOT, "validate_speed_map.py", [str(self.session)], "validate_speed_map.log")
            report = _load(self.session / "analysis" / "erpm_speed_map_validation_report.yaml")
            if not bool(report.get("accepted_for_validation", False)):
                raise RuntimeError("ERPM training candidate failed its independent hold-out validation")
            return {"speed_map_validation": report}
        if stage.key == "vel_to_erpm_audit":
            self._run_script(ERPM_ROOT, "fit_speed_map.py", [str(self.session)], "fit_speed_map.log")
            self._run_script(ERPM_ROOT, "assess_voltage_temperature.py", [str(self.session)], "assess_voltage_temperature.log")
            report = _load(self.session / "analysis" / "erpm_speed_map_report.yaml")
            if not bool(report.get("accepted_for_candidate", False)):
                raise RuntimeError("ERPM map hold-out rejected the candidate")
            return {"speed_map_audit": report}
        if stage.key == "erpm_response":
            self._run_script(ERPM_ROOT, "fit_erpm_response.py", [str(self.session)], "fit_erpm_response.log")
            report = _load(self.session / "analysis" / "erpm_response_report.yaml")
            if not bool(report.get("accepted_for_candidate", False)):
                raise RuntimeError("ERPM response timing/coverage rejected the candidate")
            return {"erpm_response": report}
        if stage.key == "erpm_response_validation":
            self._run_script(
                ERPM_ROOT, "fit_erpm_response.py",
                [
                    str(self.session), "--stage-directory", stage.directory, "--validation",
                    "--output-name", "erpm_response_validation_observed_report.yaml",
                    "--table-name", "erpm_response_validation_trials.parquet",
                    "--coverage-name", "erpm_response_validation_coverage.parquet",
                ],
                "fit_erpm_response_validation.log",
            )
            self._run_script(ERPM_ROOT, "validate_erpm_response.py", [str(self.session)], "validate_erpm_response.log")
            report = _load(self.session / "analysis" / "erpm_response_validation_report.yaml")
            if not bool(report.get("accepted_for_validation", False)):
                raise RuntimeError("ERPM response hold-out rejected the training response model")
            return {"erpm_response_validation": report}
        if stage.key == "coastdown":
            self._run_script(ERPM_ROOT, "fit_coastdown.py", [str(self.session)], "fit_coastdown.log")
            report = _load(self.session / "analysis" / "coastdown_drag_report.yaml")
            if not bool(report.get("accepted_for_candidate", False)):
                raise RuntimeError("coast-down fit failed its coverage/R² gate")
            return {"coastdown": report}
        if stage.key == "coastdown_validation":
            self._run_script(ERPM_ROOT, "validate_coastdown.py", [str(self.session)], "validate_coastdown.log")
            report = _load(self.session / "analysis" / "coastdown_validation_report.yaml")
            if not bool(report.get("accepted_for_validation", False)):
                raise RuntimeError("coast-down hold-out rejected the frozen drag model")
            return {"coastdown_validation": report}
        if stage.key == "current_training":
            self._run_script(ERPM_ROOT, "fit_current_model_training.py", [str(self.session)], "fit_current_model_training.log")
            report = _load(self.session / "analysis" / "current_acceleration_training_report.yaml")
            if not bool(report.get("accepted_for_update", False)):
                raise RuntimeError("current training data did not produce an update candidate")
            return {"current": report}
        if stage.key == "current_holdout":
            self._run_script(ERPM_ROOT, "fit_current_model.py", [str(self.session)], "fit_current_model.log")
            self._run_script(ERPM_ROOT, "fit_traction_transients.py", [str(self.session)], "fit_traction_transients.log")
            report = _load(self.session / "analysis" / "current_acceleration_report.yaml")
            traction = _load(self.session / "analysis" / "traction_transient_report.yaml")
            if not bool(report.get("accepted_for_candidate", False)):
                raise RuntimeError("current/traction hold-out rejected the candidate")
            if bool(traction.get("requires_dynamic_longitudinal_slip_model", False)):
                raise RuntimeError(
                    "traction hold-out requires a dynamic longitudinal slip model; "
                    "implement and validate that bounded runtime model before continuing"
                )
            return {"current_validation": report, "traction": traction}
        if stage.key == "accel_interface":
            self._run_script(ERPM_ROOT, "fit_accel_interface.py", [str(self.session)], "fit_accel_interface.log")
            report = _load(self.session / "analysis" / "accel_to_current_interface_report.yaml")
            if not bool(report.get("accepted_for_candidate", False)):
                raise RuntimeError("acceleration-to-current interface rejected the candidate")
            return {"interface": report}
        if stage.key == "accel_interface_validation":
            self._run_script(
                ERPM_ROOT, "fit_accel_interface.py",
                [
                    str(self.session), "--stage-directory", stage.directory, "--validation",
                    "--output-name", "accel_to_current_interface_validation_report.yaml",
                ],
                "fit_accel_interface_validation.log",
            )
            report = _load(self.session / "analysis" / "accel_to_current_interface_validation_report.yaml")
            if not bool(report.get("accepted_for_validation", False)):
                raise RuntimeError("ACCEL_TO_CURRENT hold-out rejected the candidate interface")
            # Select a causal odometry model only after all static/dynamic A/B
            # evidence exists. The next two stages collect untouched C data.
            self._run_script(ERPM_ROOT, "fit_odom_model_selection.py", [str(self.session)], "fit_odom_model_selection.log")
            odometry = _load(self.session / "analysis" / "odometry_model_selection_report.yaml")
            if not bool(odometry.get("accepted_for_shadow_deployment_verification", False)):
                raise RuntimeError("no causal longitudinal odometry candidate passed the A/B model-selection gates")
            self._complete_odometry_candidate_patch()
            return {"interface_validation": report, "odometry_model_selection": odometry}
        if stage.key == "odometry_candidate_velocity_validation":
            self._run_script(
                ERPM_ROOT, "validate_odometry_candidate_velocity.py", [str(self.session)],
                "validate_odometry_candidate_velocity.log",
            )
            report = _load(self.session / "analysis" / "odometry_candidate_velocity_validation_report.yaml")
            if not bool(report.get("accepted_for_validation", False)):
                raise RuntimeError("fresh velocity hold-outs rejected the selected command/odometry candidate")
            return {"odometry_candidate_velocity_validation": report}
        if stage.key == "odometry_candidate_accel_validation":
            self._run_script(ERPM_ROOT, "verify_candidate.py", [str(self.session)], "verify_odometry_candidate.log")
            report = _load(self.session / "analysis" / "candidate_deployment_verification_report.yaml")
            if not bool(report.get("accepted_for_permanent_review", False)):
                raise RuntimeError("fresh acceleration hold-outs rejected the selected causal odometry candidate")
            return {"odometry_candidate_accel_validation": report}
        if stage.key == "lateral_stiffness_training":
            self._run_script(ERPM_ROOT, "fit_lateral_stiffness.py", [str(self.session)], "fit_lateral_stiffness.log")
            report = _load(self.session / "analysis" / "lateral_stiffness_training_report.yaml")
            if not bool(report.get("accepted_for_candidate", False)):
                raise RuntimeError("lateral tyre-stiffness training did not produce an identifiable candidate")
            return {"lateral_stiffness": report}
        if stage.key == "lateral_stiffness_validation":
            self._run_script(ERPM_ROOT, "validate_lateral_stiffness.py", [str(self.session)], "validate_lateral_stiffness.log")
            report = _load(self.session / "analysis" / "lateral_stiffness_validation_report.yaml")
            if not bool(report.get("accepted_for_validation", False)):
                raise RuntimeError("lateral tyre-stiffness hold-out rejected the training model")
            return {"lateral_stiffness_validation": report}
        # Audits and the human end-stop survey do not have a separate fitting
        # script, but their runtime result is still first-class evidence.  Feed
        # it into the stage report/LaTeX page so a completed audit cannot look
        # like a blank analysis stage merely because it produced no model.
        runtime_result = self.session / stage.directory / "runtime_result.json"
        if runtime_result.is_file():
            try:
                result = json.loads(runtime_result.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as exc:
                raise RuntimeError(f"cannot read runtime result for {stage.key}: {runtime_result}") from exc
            if isinstance(result, dict):
                return {"runtime": result}
        return {"runtime": {"status": "no_runtime_analysis_artifact"}}

    def _capture_function(self, stage: StageSpec) -> Callable[[Path], dict[str, Any]]:
        if stage.kind == "manual":
            from .metrology import ensure_measurement_sheet
            return lambda directory: {
                "manual": True,
                "measurement_sheet": str(ensure_measurement_sheet(self.session)),
                "instruction": "Fill and save the session-local physical_measurements.yaml before this stage can pass.",
            }
        if stage.kind == "steering":
            sys.path.insert(0, str(STEERING_ROOT))
            from steering_calibration import stages as impl
            cfg = copy.deepcopy(self.steering_cfg)
            cfg["session"]["event_topic"] = "/steering_calibration/event"
            applied_global = self._active_patch().get("global", {})
            if isinstance(applied_global, dict):
                try:
                    applied_gain = float(applied_global["steering_angle_to_servo_gain"])
                    applied_offset = float(applied_global["steering_angle_to_servo_offset"])
                except (KeyError, TypeError, ValueError):
                    pass
                else:
                    if math.isfinite(applied_gain) and abs(applied_gain) > 1.0e-12 and math.isfinite(applied_offset):
                        cfg["applied_steering_map"] = {
                            "steering_angle_to_servo_gain": applied_gain,
                            "steering_angle_to_servo_offset": applied_offset,
                            "source": "current reversible session patch",
                        }
            if stage.key == "steering_centre":
                return lambda directory: impl.zero_curvature_centre(cfg, directory)
            if stage.key == "steering_centre_validation":
                return lambda directory: impl.zero_curvature_validation(cfg, directory, self.state["centre"])
            if stage.key == "steering_observability":
                # Runs before centre/endstops by design.  It measures only the
                # stationary and straight scene at the deployed raw seed; the
                # optional turn diagnostic requires verified limits and is not
                # a prerequisite for a straight-line centre fit.
                return lambda directory: impl.sensor_observability(cfg, directory, self.state["centre"], None)
            if stage.key == "steering_endstops":
                return lambda directory: impl.physical_endstops(cfg, directory, self.state["centre"])
            if stage.key == "steering_static_training":
                return lambda directory: impl.static_map(cfg, directory, self.state["centre"], self.state["limits"], validation=False)
            if stage.key == "steering_static_holdout":
                return lambda directory: impl.static_map(cfg, directory, self.state["centre"], self.state["limits"], validation=True)
            if stage.key == "steering_response":
                return lambda directory: impl.command_to_curvature_response(cfg, directory, self.state["centre"], self.state["limits"])
            if stage.key == "steering_response_validation":
                return lambda directory: impl.command_to_curvature_response(
                    cfg, directory, self.state["centre"], self.state["limits"], validation=True
                )
            return lambda directory: impl.raw_command_path_audit(cfg, directory)

        sys.path.insert(0, str(ERPM_ROOT))
        from erpm_calibration import stages as impl
        from erpm_calibration.stages import TrialCounter
        cfg = copy.deepcopy(self.erpm_cfg)
        cfg["session"]["event_topic"] = "/erpm_calibration/event"
        state = self.state
        gain = float(state.get("initial_speed_gain", self._read_source_params().get("speed_to_erpm_gain", 1.0)))
        speed_patch = state.get("speed_command_patch")
        counter = TrialCounter(cfg)
        if stage.key == "odometry_candidate_velocity_validation":
            return lambda directory: impl.candidate_velocity_verification(cfg, directory, counter)
        if stage.key == "odometry_candidate_accel_validation":
            candidate = _load(self.session / "analysis" / "selected_odometry_candidate_patch.yaml")
            params = candidate.get("ackermann_to_vesc_node", {}).get("ros__parameters", {})
            gain = float(params["speed_to_erpm_gain"])
            return lambda directory: impl.candidate_accel_verification(
                cfg, directory, gain, 0.0, counter, candidate,
            )
        return lambda directory: impl.run_stage(stage.runtime_name, cfg, directory, gain, 0.0, counter,
                                                speed_command_patch=speed_patch)

    def _read_source_params(self) -> dict[str, Any]:
        document = _load(self._source_path())
        return document.get("/**", {}).get("ros__parameters", {})

    def _update_after_analysis(self, stage: StageSpec, analysis: dict[str, Any], tx: ConfigManager) -> None:
        state = self.state
        active = self._active_patch()
        patch: dict[str, Any] | None = None
        if "physical_metrology" in analysis:
            report = analysis["physical_metrology"]
            try:
                mass = float(report["mass_kg"])
                wheelbase = float(report["wheelbase_m"])
                lf = float(report["cg_to_front_axle_lf_m"])
                lr = float(report["cg_to_rear_axle_lr_m"])
                lidar_x = float(report["lidar_to_base"]["x_m"])
                lidar_y = float(report["lidar_to_base"]["y_m"])
                lidar_yaw = float(report["lidar_to_base"]["yaw_rad"])
                rear_x = float(report["rear_axle_in_base_link"]["x_m"])
                rear_y = float(report["rear_axle_in_base_link"]["y_m"])
                imu_x = float(report["imu_to_base"]["x_m"])
                imu_y = float(report["imu_to_base"]["y_m"])
                imu_z = float(report["imu_to_base"]["z_m"])
                imu_yaw = float(report["imu_to_base"]["yaw_rad"])
            except (KeyError, TypeError, ValueError) as exc:
                raise RuntimeError("accepted physical metrology has incomplete dynamic-model geometry") from exc
            if not all(math.isfinite(value) for value in (
                lidar_x, lidar_y, lidar_yaw, rear_x, rear_y, imu_x, imu_y, imu_z, imu_yaw,
            )):
                raise RuntimeError("accepted physical metrology has non-finite sensor/reference geometry")
            dynamic = {
                "vehicle_mass": mass,
                "l_f": lf,
                "l_r": lr,
                "imu_to_base_yaw_rad": imu_yaw,
            }
            yaw_inertia = report.get("selected_yaw_inertia_kg_m2")
            try:
                yaw_inertia = float(yaw_inertia)
            except (TypeError, ValueError):
                yaw_inertia = math.nan
            if math.isfinite(yaw_inertia) and yaw_inertia > 0.0:
                dynamic["vehicle_Iz"] = yaw_inertia
            patch = {
                # The standard racing launch reads this config-only section to
                # publish the same measured static transforms used during
                # calibration. It is promoted atomically with the model patch.
                "vehicle_geometry": {
                    "laser_to_base_x_m": lidar_x,
                    "laser_to_base_y_m": lidar_y,
                    "laser_to_base_yaw_rad": lidar_yaw,
                    "imu_to_base_x_m": imu_x,
                    "imu_to_base_y_m": imu_y,
                    "imu_to_base_z_m": imu_z,
                    "imu_to_base_yaw_rad": imu_yaw,
                    "base_link_to_rear_axle_x_m": rear_x,
                    "base_link_to_rear_axle_y_m": rear_y,
                },
                "vesc_to_odom_node": dynamic,
                # The legacy analytical node still exposes wheelbase directly;
                # carrying the measured value prevents an accidental old-odom
                # launch from silently retaining a stale geometry number.
                "vesc_to_odom_old_node": {"wheelbase": wheelbase},
            }
            _dump(self.session / "analysis" / "physical_vehicle_vesc_patch.yaml", patch)
            tx.apply(patch, "build_physical_vehicle_geometry_update")
        elif "centre" in analysis:
            centre = analysis["centre"]
            raw = float(centre["centre_servo_raw"])
            state["centre"] = {"centre_servo_raw": raw}
            patch = {
                "global": {"steering_angle_to_servo_offset": raw},
                "vesc_to_odom_node": {"steering_angle_to_servo_offset": raw},
            }
            _dump(self.session / "analysis" / f"{stage.key}_vesc_patch.yaml", patch)
            tx.apply(patch, f"build_{stage.key}_centre_update")
        elif stage.key == "steering_endstops":
            result = json.loads((self.session / stage.directory / "runtime_result.json").read_text(encoding="utf-8"))
            state["limits"] = result
        elif "static_map" in analysis:
            candidate = analysis["static_map"]
            x = candidate.get("raw_servo", [])
            y = candidate.get("delta_eq_rad", [])
            if len(x) < 3 or len(x) != len(y):
                raise RuntimeError("accepted steering map has insufficient points for deployable linear patch")
            import numpy as np
            map_delta = np.asarray(y, dtype=float)
            map_raw = np.asarray(x, dtype=float)
            centre_raw = float((state.get("centre") or {}).get("centre_servo_raw", float("nan")))
            denominator = float(np.dot(map_delta, map_delta))
            if not np.isfinite(centre_raw) or denominator <= 1e-12:
                raise RuntimeError("steering map cannot be anchored at the validated zero-curvature centre")
            # Keep the independently gated zero-curvature centre exact. An
            # unconstrained line fit can move its intercept enough to recreate
            # the large steering bias this campaign is intended to remove.
            gain = float(np.dot(map_delta, map_raw - centre_raw) / denominator)
            offset = centre_raw
            if not np.isfinite(gain) or gain >= 0:
                raise RuntimeError("steering deployable map has invalid sign")
            limits = state.get("limits") or {}
            # The new direct raw-servo fit is a deliberately linear production
            # candidate.  Leaving a historical polynomial correction active
            # would silently make C exercise a different map than B fitted.
            # If the independent hold-out prefers a nonlinear map, the stage
            # writes an explicit implementation request instead of retaining a
            # stale quadratic from a previous campaign.
            linear_correction = {
                "steering_correction_c2": 0.0,
                "steering_correction_c1": 1.0,
                "steering_correction_c0": 0.0,
            }
            patch = {
                "global": {
                    "steering_angle_to_servo_gain": float(gain),
                    "steering_angle_to_servo_offset": float(offset),
                    "servo_min": float(limits.get("raw_low_safe", min(x))),
                    "servo_max": float(limits.get("raw_high_safe", max(x))),
                    **linear_correction,
                },
                "ackermann_to_vesc_node": {
                    "steering_angle_to_servo_gain": float(gain),
                    "steering_angle_to_servo_offset": float(offset),
                    **linear_correction,
                },
                "vesc_to_odom_node": {
                    "steering_angle_to_servo_gain": float(gain),
                    "steering_angle_to_servo_offset": float(offset),
                    **linear_correction,
                },
                "vesc_to_odom_old_node": {
                    "steering_angle_to_servo_gain": float(gain),
                    "steering_angle_to_servo_offset": float(offset),
                    **linear_correction,
                },
            }
            candidate["deployable_linear_patch"] = patch
            _json_dump(self.session / "analysis" / "candidate_static_steering_map.json", candidate)
            _dump(self.session / "analysis" / "steering_map_vesc_patch.yaml", patch)
            tx.apply(patch, "build_steering_static_map_update")
        elif "speed_map" in analysis:
            report = analysis["speed_map"]
            gain = float(report["candidate_speed_to_erpm_gain"])
            odom_scale = float(report.get("candidate_odom_speed_scale", 1.0))
            patch = {
                "global": {"speed_to_erpm_gain": gain, "speed_to_erpm_offset": 0.0},
                "ackermann_to_vesc_node": {
                    "speed_to_erpm_gain": gain,
                    "speed_to_erpm_offset": 0.0,
                    "slow_start_threshold": float(report["candidate_slow_start_threshold_mps"]),
                    "slow_start_increment": float(report["candidate_slow_start_increment_mps"]),
                    "stop_speed_deadzone": float(report["candidate_stop_speed_deadzone_mps"]),
                },
                "vesc_to_odom_node": {
                    "speed_to_erpm_gain": gain,
                    "speed_to_erpm_offset": 0.0,
                    "odom_speed_scale": odom_scale,
                    "speed_deadband": float(report["candidate_odom_speed_deadband_mps"]),
                },
            }
            state["speed_command_patch"] = {
                "ackermann_to_vesc_node": {"ros__parameters": patch["ackermann_to_vesc_node"]}
            }
            _dump(self.session / "analysis" / "speed_map_vesc_patch.yaml", patch)
            tx.apply(patch, "build_erpm_speed_map_update")
        elif "coastdown" in analysis:
            report = analysis["coastdown"]
            patch = {"ackermann_to_vesc_node": {
                "accel_drag_coulomb": float(report["accel_drag_coulomb_mps2"]),
                "accel_drag_viscous": float(report["accel_drag_viscous_per_s"]),
                "accel_drag_quadratic": float(report["accel_drag_quadratic_per_m"]),
            }}
            _dump(self.session / "analysis" / "coastdown_vesc_patch.yaml", patch)
            tx.apply(patch, "build_coastdown_update")
        elif "current" in analysis:
            report = analysis["current"]
            patch = {"ackermann_to_vesc_node": {
                "accel_to_current_gain": float(report["candidate_accel_to_current_gain"]),
                "accel_to_brake_gain": float(report["candidate_accel_to_brake_gain"]),
                "accel_deadzone": float(report["candidate_accel_deadzone_mps2"]),
            }}
            state["accel_patch"] = patch
            _dump(self.session / "analysis" / "current_vesc_patch.yaml", patch)
            tx.apply(patch, "build_current_model_update")
        elif stage.key == "odometry_candidate_accel_validation":
            # The two preceding C stages exercised the selected shadow model.
            # Install it in the production node only when its exact wheel and
            # command forms are representable by the current C++ scalar path.
            selection = _load(self.session / "analysis" / "odometry_model_selection_report.yaml")
            candidate = _load(self.session / "analysis" / "selected_odometry_candidate_patch.yaml")
            selected_family = str(selection.get("selected_family", ""))
            selected_command = str(selection.get("command_map_selected", ""))
            params = candidate.get("vesc_to_odom_node", {}).get("ros__parameters", {})
            selected_wheel = str(params.get("odom_wheel_model", ""))
            representable = (
                selected_family in {"legacy_scalar", "static_linear"}
                and selected_wheel == "linear"
                and selected_command == "linear"
            )
            if not representable:
                request = {
                    "required": True,
                    "selected_family": selected_family,
                    "selected_wheel_model": selected_wheel,
                    "selected_command_model": selected_command,
                    "reason": (
                        "Fresh shadow hold-outs selected a longitudinal model that the production C++ nodes "
                        "cannot reproduce exactly. Port the emitted selected_odometry_candidate_patch.yaml "
                        "contract, then redo both odometry C stages before continuing."
                    ),
                }
                _dump(self.session / "analysis" / "odometry_runtime_model_request.yaml", request)
                raise RuntimeError(
                    "selected longitudinal odometry/command model requires a production implementation; "
                    "see analysis/odometry_runtime_model_request.yaml"
                )
            try:
                erpm_to_speed = float(params["odom_erpm_to_speed_linear"])
                speed_gain = float(_load(self.session / "analysis" / "erpm_speed_map_report.yaml")["candidate_speed_to_erpm_gain"])
            except (KeyError, TypeError, ValueError) as exc:
                raise RuntimeError("selected static odometry candidate has incomplete production coefficients") from exc
            odom_scale = erpm_to_speed * speed_gain
            if not math.isfinite(odom_scale) or odom_scale <= 0.0:
                raise RuntimeError("selected static odometry candidate implies an invalid production odom scale")
            patch = {"vesc_to_odom_node": {
                "speed_to_erpm_gain": speed_gain,
                "speed_to_erpm_offset": 0.0,
                "odom_speed_scale": odom_scale,
                "speed_deadband": float(params.get("speed_deadband", 0.05)),
            }}
            _dump(self.session / "analysis" / "odometry_runtime_vesc_patch.yaml", patch)
            tx.apply(patch, "build_selected_static_odometry_update")
        elif "lateral_stiffness" in analysis:
            report = analysis["lateral_stiffness"]
            try:
                front = float(report["front_tyre"]["linear"]["cornering_stiffness_N_per_rad"])
                rear = float(report["rear_tyre"]["linear"]["cornering_stiffness_N_per_rad"])
                steering_scale = float(report["steering_model_scale_candidate"])
            except (KeyError, TypeError, ValueError) as exc:
                raise RuntimeError("accepted lateral training has no finite front/rear stiffness or steering-scale candidate") from exc
            if not all(math.isfinite(value) and value > 0.0 for value in (front, rear, steering_scale)):
                raise RuntimeError("accepted lateral training has invalid front/rear stiffness or steering-scale candidate")
            turn_slip = report.get("cornering_longitudinal_slip", {})
            if not isinstance(turn_slip, dict) or not bool(turn_slip.get("accepted_for_candidate", False)):
                raise RuntimeError("accepted lateral training has no valid cornering longitudinal-slip candidate")
            patch = {"vesc_to_odom_node": {
                "c_alpha_f": front,
                "c_alpha_r": rear,
                "steering_model_scale": steering_scale,
                "pacejka_shape_factor": float(
                    report.get("runtime_lateral_tyre_model", {}).get("pacejka_shape_factor", 1.9)
                ),
                "odom_turn_slip_coeff_per_mps2": float(turn_slip.get("selected_coefficient_per_mps2", 0.0)),
                "odom_turn_slip_clip_fraction": float(turn_slip.get("clip_fraction", 0.25)),
                "odom_turn_slip_accepted": bool(turn_slip.get("correction_active", False)),
            }}
            _dump(self.session / "analysis" / "lateral_stiffness_vesc_patch.yaml", patch)
            tx.apply(patch, "build_lateral_stiffness_update")
        if patch:
            state["active_patch"] = self._merge_patch(active, patch)
        self._save_state(state)

    def _initial_state(self) -> dict[str, Any]:
        return {
            "status": "in_progress",
            "active_patch": {},
            "centre": {"centre_servo_raw": float(self.steering_cfg["initial"]["raw_servo_seed"])},
            "limits": None,
            "speed_command_patch": None,
            "accel_patch": None,
        }

    def _patch_for_completed_stage(self, entry: dict[str, Any], stage: StageSpec) -> dict[str, Any] | None:
        relative = entry.get("patch_file")
        candidates: list[Path] = []
        if relative:
            candidates.append(self.session / str(relative))
        fallback = {
            "steering_centre": "steering_centre_vesc_patch.yaml",
            "physical_metrology": "physical_vehicle_vesc_patch.yaml",
            "steering_static_training": "steering_map_vesc_patch.yaml",
            "erpm_map_training": "speed_map_vesc_patch.yaml",
            "coastdown": "coastdown_vesc_patch.yaml",
            "current_training": "current_vesc_patch.yaml",
            "odometry_candidate_accel_validation": "odometry_runtime_vesc_patch.yaml",
            "lateral_stiffness_training": "lateral_stiffness_vesc_patch.yaml",
        }.get(stage.key)
        if fallback:
            candidates.append(self.session / "analysis" / fallback)
        for path in candidates:
            if path.is_file():
                value = _load(path)
                if isinstance(value, dict):
                    return value
        return None

    def _state_before_stage(self, index: int, manifest: dict[str, Any]) -> dict[str, Any]:
        """Rehydrate only the approved candidates that precede ``index``.

        A redo must not accidentally retain the failed candidate.  New sessions
        store a snapshot after each completed stage; older sessions are replayed
        from their immutable patch/runtime artifacts as a compatibility path.
        """
        state = self._initial_state()
        entries = manifest.get("stages", {})
        for prior in STAGES[:index]:
            entry = entries.get(prior.key, {})
            if entry.get("status") != "completed":
                raise RuntimeError(f"cannot rebuild state: prerequisite {prior.key} is not completed")
            snapshot = entry.get("state_after")
            if isinstance(snapshot, dict):
                state = copy.deepcopy(snapshot)
                state["status"] = "in_progress"
                continue
            analysis = entry.get("analysis", {}) if isinstance(entry.get("analysis"), dict) else {}
            if prior.key == "steering_centre" and isinstance(analysis.get("centre"), dict):
                raw = analysis["centre"].get("centre_servo_raw")
                if raw is not None:
                    state["centre"] = {"centre_servo_raw": float(raw)}
            if prior.key == "steering_endstops" and isinstance(entry.get("runtime"), dict):
                state["limits"] = copy.deepcopy(entry["runtime"])
            patch = self._patch_for_completed_stage(entry, prior)
            if patch:
                state["active_patch"] = self._merge_patch(state.get("active_patch", {}), patch)
                if prior.key == "erpm_map_training":
                    params = patch.get("ackermann_to_vesc_node", {})
                    state["speed_command_patch"] = {
                        "ackermann_to_vesc_node": {"ros__parameters": copy.deepcopy(params)}
                    }
                if prior.key == "current_training":
                    state["accel_patch"] = copy.deepcopy(patch)
        state["status"] = "in_progress"
        return state

    def redo_from(self, requested_stage: str) -> None:
        """Archive a failed branch and restart A for the affected parameter.

        ``run --stage <validation>`` is intentionally a same-candidate C retry.
        This method is the explicit, auditable alternative when validation says
        B produced the wrong candidate: it clears that candidate and all
        downstream state, then makes the next invocation collect fresh A data.
        """
        requested = STAGE_BY_KEY.get(requested_stage) or STAGE_BY_DIRECTORY.get(requested_stage)
        if requested is None:
            raise ValueError(f"unknown stage {requested_stage}; choose one of: {', '.join(STAGE_BY_KEY)}")
        target_key = VALIDATION_REDO_TARGETS.get(requested.key, requested.key)
        target = STAGE_BY_KEY[target_key]
        target_index = STAGES.index(target)
        manifest = self.manifest
        if not any(manifest.get("stages", {}).get(stage.key) for stage in STAGES[target_index:]):
            raise RuntimeError(f"there is no recorded branch beginning at {target.key} to redo")
        rebuilt_state = self._state_before_stage(target_index, manifest)
        timestamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        archive = self.session / "recalibration_history" / f"{timestamp}_{target.key}"
        archive.mkdir(parents=True, exist_ok=False)
        _dump(archive / "session_manifest_before_redo.yaml", manifest)
        _json_dump(archive / "runtime_state_before_redo.json", self.state)
        analysis = self.session / "analysis"
        if analysis.exists():
            shutil.copytree(analysis, archive / "analysis_before_redo", dirs_exist_ok=True)
        moved: list[str] = []
        for stage in STAGES[target_index:]:
            source = self.session / stage.directory
            if source.exists():
                destination = archive / stage.directory
                shutil.move(str(source), str(destination))
                moved.append(stage.directory)
            for source in (
                analysis / "stage_reports" / f"{stage.key}.md",
                self.session / "plots" / "stages" / f"{stage.key}.png",
            ):
                if source.exists():
                    destination = archive / source.relative_to(self.session)
                    destination.parent.mkdir(parents=True, exist_ok=True)
                    shutil.move(str(source), str(destination))

        entries = manifest.setdefault("stages", {})
        removed = [stage.key for stage in STAGES[target_index:] if entries.pop(stage.key, None) is not None]
        event = {
            "utc": dt.datetime.now(dt.timezone.utc).isoformat(),
            "requested_from": requested.key,
            "restarted_at": target.key,
            "archive": str(archive.relative_to(self.session)),
            "archived_stage_directories": moved,
            "cleared_manifest_stages": removed,
            "reason": "explicit fresh A/B recalibration requested after validation or fit failure",
        }
        manifest.setdefault("recalibration_events", []).append(event)
        manifest["status"] = "in_progress"
        self._save_manifest(manifest)
        self._save_state(rebuilt_state)
        from .stage_report import write_campaign_log
        from .latex_report import write_latex_document
        write_campaign_log(self)
        write_latex_document(self, compile_pdf=True)
        if requested.key != target.key:
            print(
                f"Archived the failed {requested.key} branch and cleared its candidate. "
                f"Next run starts fresh A at {target.key}; the old branch is in {archive}."
            )
        else:
            print(f"Archived branch from {target.key}. Next run starts fresh A at {target.key}; archive: {archive}")

    def _mark(self, stage: StageSpec, status: str, **data: Any) -> None:
        manifest = self.manifest
        manifest.setdefault("stages", {})[stage.key] = {
            "directory": stage.directory,
            "status": status,
            "updated_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
            **data,
        }
        manifest["status"] = "in_progress" if status == "completed" else status
        self._save_manifest(manifest)

    def _dependencies_ok(self, stage: StageSpec) -> None:
        manifest = self.manifest
        for dependency in stage.dependencies:
            if manifest.get("stages", {}).get(dependency, {}).get("status") != "completed":
                raise RuntimeError(f"stage {stage.key} is blocked by incomplete dependency {dependency}")

    def run_one(self, stage_name: str) -> None:
        stage = STAGE_BY_KEY.get(stage_name) or STAGE_BY_DIRECTORY.get(stage_name)
        if stage is None:
            raise ValueError(f"unknown stage {stage_name}; choose one of: {', '.join(STAGE_BY_KEY)}")
        self._validate_room()
        self._dependencies_ok(stage)
        if self.manifest.get("stages", {}).get(stage.key, {}).get("status") == "completed":
            print(f"Stage already completed: {stage.key}")
            return
        tx = ConfigManager(self, self._source_path())
        self._mark(stage, "running")
        try:
            # Direct physical metrology is the only manual stage that creates
            # runtime dynamic-model parameters.  Give it the same reversible
            # patch/build transaction as an automatic B stage; the source file
            # is still restored before returning to the operator.
            if stage.kind != "manual" or stage.key == "physical_metrology":
                tx.begin()
                if stage.kind != "manual":
                    self._temporary_setup(tx, stage)
            capture = self._launch_and_bag(stage, self._capture_function(stage))
            patch_files_before = set((self.session / "analysis").glob("*patch.yaml"))
            analysis = self._analyse(stage)
            # Every stage gets the same descriptive statistical record. This
            # never creates a new fit or gate: it makes sample independence,
            # repeat coverage, spread, confidence intervals and existing model
            # accuracy metrics visible in the GUI/LaTeX evidence.
            from .statistics import summarize_stage_statistics
            analysis["statistical_evidence"] = summarize_stage_statistics(
                self.session, stage, analysis,
            )
            if stage.kind != "manual" or stage.key == "physical_metrology":
                self._update_after_analysis(stage, analysis, tx)
            from .stage_report import write_campaign_log, write_stage_report
            stage_report = write_stage_report(self, stage, analysis)
            patch_files_after = sorted(set((self.session / "analysis").glob("*patch.yaml")) - patch_files_before)
            self._mark(stage, "completed", runtime=capture["runtime"],
                       bag_verification=capture["bag_verification"], analysis=analysis,
                       stage_report=stage_report,
                       patch_file=str(patch_files_after[0].relative_to(self.session)) if patch_files_after else None,
                       state_after={**copy.deepcopy(self.state), "status": "in_progress"})
            self._save_state({**self.state, "status": "in_progress"})
            write_campaign_log(self)
            from .latex_report import write_latex_document
            write_latex_document(self, compile_pdf=True)
            if stage.key == "physical_metrology":
                print(f"COMPLETED {stage.key}: direct measurements validated; dynamic geometry candidate rebuilt and recorded.")
            elif stage.kind == "manual":
                print(f"COMPLETED {stage.key}: manual stage recorded.")
            else:
                print(f"COMPLETED {stage.key}: capture, analysis gate, patch/update and rebuild recorded.")
        except BaseException as exc:
            retry = None
            if stage.key in VALIDATION_REDO_TARGETS:
                retry = {
                    "same_candidate_retry": f"run --stage {stage.key}",
                    "fresh_recalibration": f"redo --from {stage.key}",
                    "note": "A same-candidate retry changes no values. Use redo only when the validation shows the candidate itself is wrong.",
                }
            # A rejected gate is still an analysis result. Preserve its page
            # and plot (when the analysis script produced plot inputs) before
            # marking the stage failed, so nonlinear/model-upgrade stops do not
            # disappear from the cumulative evidence document.
            failed_analysis: dict[str, Any] = {
                "accepted_for_stage": False,
                "failure": str(exc),
            }
            failed_report = None
            try:
                from .statistics import summarize_stage_statistics
                failed_analysis["statistical_evidence"] = summarize_stage_statistics(
                    self.session, stage, failed_analysis,
                )
                from .stage_report import write_stage_report
                failed_report = write_stage_report(self, stage, failed_analysis)
            except Exception as report_exc:
                failed_analysis["stage_report_error"] = repr(report_exc)
            self._mark(
                stage, "failed", error=repr(exc), retry_guidance=retry,
                analysis=failed_analysis, stage_report=failed_report,
            )
            from .stage_report import write_campaign_log
            from .latex_report import write_latex_document
            write_campaign_log(self)
            write_latex_document(self, compile_pdf=True)
            if retry:
                print(
                    f"FAILED {stage.key}. Repeat only C with `run --stage {stage.key}` after a spoiled capture, "
                    f"or run `redo --from {stage.key}` to clear the candidate and collect a new A/B fit."
                )
            raise
        finally:
            if tx.started:
                tx.restore()

    def run_next(self) -> None:
        manifest = self.manifest
        for stage in STAGES:
            if manifest.get("stages", {}).get(stage.key, {}).get("status") != "completed":
                self.run_one(stage.key)
                return
        print("All calibration stages are complete. Run report to assemble the final document.")

    def build_report(self, promote: bool = False) -> None:
        from .report import build_report
        build_report(self, promote=promote)

    @classmethod
    def recover(cls, workspace: Path | None) -> None:
        ConfigManager.recover(workspace)
