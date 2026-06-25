"""Single-command orchestration of the operator-guided steering session.

The runner owns a reversible VESC configuration transaction: it snapshots the
source VESC YAML, temporarily enables VEL_TO_ERPM and full numeric servo-domain
access, builds the workspace, runs the campaign, then restores the exact source
file and builds again. The third-party operator never edits ROS configuration.
"""
from __future__ import annotations

import datetime as dt
import hashlib
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Callable

from .bagging import BagProcess, start_bag, stop_bag
from .config import copy_config_file, dump_json, dump_yaml, load_yaml
from .config_transaction import VescConfigTransaction
from .stages import (
    command_to_curvature_response,
    imu_bias_ground,
    physical_endstops,
    raw_command_path_audit,
    sensor_observability,
    static_map,
    zero_curvature_centre,
)
from .ui import banner, disk_line, require_ready


class StackProcess:
    def __init__(self, script: Path, calibration_config_path: Path, mode: str, log_path: Path,
                 raw_min: float, raw_max: float) -> None:
        self.script = script
        self.calibration_config_path = calibration_config_path.resolve()
        self.mode = mode
        self.log_path = log_path
        self.raw_min = float(raw_min)
        self.raw_max = float(raw_max)
        self.process: subprocess.Popen | None = None
        self._handle = None

    def start(self) -> None:
        command = [
            sys.executable, str(self.script),
            "--config", str(self.calibration_config_path),
            "--raw-min", str(self.raw_min), "--raw-max", str(self.raw_max),
        ]
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self._handle = self.log_path.open("w", encoding="utf-8")
        self.process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=self._handle,
                                        stderr=subprocess.STDOUT, start_new_session=True, text=True)
        time.sleep(3.0)
        if self.process.poll() is not None:
            detail = self.log_path.read_text(encoding="utf-8", errors="replace")
            raise RuntimeError(f"{self.mode} launch failed:\n{detail}")

    def stop(self) -> None:
        if self.process and self.process.poll() is None:
            try:
                os.killpg(self.process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                self.process.wait(timeout=12.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(self.process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                self.process.wait(timeout=5.0)
        if self._handle:
            self._handle.close()


class SessionRunner:
    """Owns stage bags, launch modes and configuration/recording evidence."""

    def __init__(self, root: Path, config_path: Path, runs_dir: Path | None = None,
                 resume: Path | None = None, workspace: Path | None = None) -> None:
        self.root = root.resolve()
        self.config_path = config_path.resolve()
        self.config = load_yaml(self.config_path)
        self.runs_dir = (runs_dir or (self.root / self.config["session"]["runs_dir"])).resolve()
        self.resume = resume
        self.workspace = self._resolve_workspace(workspace)
        self.session_dir = self._prepare_session()
        self.topic_policy = load_yaml(self.root / "config" / "topics.yaml")
        self.stack: StackProcess | None = None
        self.runtime: dict[str, Any] = self._load_runtime()
        self._launch_index = 0
        self.config_transaction = VescConfigTransaction(
            steering_root=self.root,
            session_dir=self.session_dir,
            workspace=self.workspace,
            config_relpath=self.config.get("workspace", {}).get(
                "vesc_config_relpath", "f1tenth_system/f1tenth_stack/config/vesc.yaml"
            ),
            build_command=list(self.config.get("workspace", {}).get(
                "colcon_build_command", ["colcon", "build", "--symlink-install", "--packages-ignore state_receiver_udp"]
            )),
        )

    def _resolve_workspace(self, workspace: Path | None) -> Path:
        if workspace:
            return workspace.expanduser().resolve()
        # steering/ -> f1tenth_parameters/ -> BachelorProject/
        return self.root.parents[1]

    def _prepare_session(self) -> Path:
        if self.resume:
            path = self.resume.resolve()
            if not (path / "session_manifest.yaml").exists():
                raise FileNotFoundError(f"not a steering-calibration session: {path}")
            return path
        stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        path = self.runs_dir / f"{stamp}_steering_calibration"
        path.mkdir(parents=True, exist_ok=False)
        copy_config_file(self.config_path, path / "calibration_config_snapshot.yaml")
        copy_config_file(self.root / "config" / "topics.yaml", path / "recording_policy_snapshot.yaml")
        copy_config_file(self.root / "config" / "bag_qos_overrides.yaml", path / "bag_qos_overrides_snapshot.yaml")
        manifest = {
            "session_id": path.name,
            "created_utc": stamp,
            "status": "in_progress",
            "steering_root": str(self.root),
            "workspace": str(self.workspace),
            "stages": {},
        }
        dump_yaml(path / "session_manifest.yaml", manifest)
        return path

    def _manifest(self) -> dict[str, Any]:
        return load_yaml(self.session_dir / "session_manifest.yaml")

    def _update_manifest(self, **updates: Any) -> None:
        manifest = self._manifest()
        manifest.update(updates)
        dump_yaml(self.session_dir / "session_manifest.yaml", manifest)

    def _load_runtime(self) -> dict[str, Any]:
        path = self.session_dir / "runtime_state.json"
        if path.exists():
            import json
            return json.loads(path.read_text(encoding="utf-8"))
        return {}

    def _save_runtime(self) -> None:
        dump_json(self.session_dir / "runtime_state.json", self.runtime)

    def _archive_vesc_source_config(self) -> None:
        rel = self.config.get("workspace", {}).get(
            "vesc_config_relpath", "f1tenth_system/f1tenth_stack/config/vesc.yaml"
        )
        source = self.workspace / rel
        result: dict[str, Any] = {"source_path": str(source), "exists": source.is_file()}
        if source.is_file():
            evidence = self.session_dir / "environment" / "vesc.yaml_at_session_start"
            evidence.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, evidence)
            result["session_copy"] = str(evidence.relative_to(self.session_dir))
            result["sha256"] = hashlib.sha256(source.read_bytes()).hexdigest()
        dump_yaml(self.session_dir / "environment" / "vesc_config_evidence.yaml", result)

    def _check_disk(self) -> None:
        free_gb = shutil.disk_usage(self.session_dir).free / (1024 ** 3)
        required = float(self.config["session"]["min_free_disk_gb"])
        print(disk_line(str(self.session_dir)))
        if free_gb < required:
            raise RuntimeError(f"Need at least {required:.1f} GiB free disk for raw MCAP recording")

    def _launch(self, mode: str, *, raw_min: float, raw_max: float) -> None:
        self._stop_stack()
        self._launch_index += 1
        script_name = "hardware_only.py" if mode == "hardware_only" else "calibration_stack.py"
        log = self.session_dir / "stack_logs" / f"{self._launch_index:02d}_{mode}_{raw_min:.4f}_{raw_max:.4f}.log"
        snapshot = self.session_dir / "calibration_config_snapshot.yaml"
        if not snapshot.is_file():
            raise FileNotFoundError(f"missing immutable calibration configuration snapshot: {snapshot}")
        hardware = load_yaml(snapshot).get("hardware", {})
        geometry_evidence = {
            "launch_index": self._launch_index,
            "mode": mode,
            "calibration_config_snapshot": str(snapshot),
            "hardware_geometry_used_by_launch": hardware,
            "selector_raw_min": float(raw_min),
            "selector_raw_max": float(raw_max),
        }
        dump_yaml(self.session_dir / "environment" / f"launch_{self._launch_index:02d}_geometry.yaml", geometry_evidence)
        self.stack = StackProcess(self.root / "launch" / script_name, snapshot, mode, log, raw_min, raw_max)
        self.stack.start()

    def _stop_stack(self) -> None:
        if self.stack:
            self.stack.stop()
            self.stack = None

    def _run_cli(self, args: list[str], out: Path) -> str:
        out.parent.mkdir(parents=True, exist_ok=True)
        try:
            result = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, timeout=12.0, check=False)
            out.write_text(result.stdout, encoding="utf-8")
            if result.returncode != 0:
                raise RuntimeError(result.stdout.strip())
            return result.stdout
        except subprocess.TimeoutExpired as exc:
            out.write_text(f"TIMEOUT: {args!r}\n{exc!r}\n", encoding="utf-8")
            raise RuntimeError(f"timed out running {' '.join(args)}") from exc

    @staticmethod
    def _extract_float(text: str) -> float:
        matches = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", text)
        if not matches:
            raise ValueError(f"no numeric value in parameter response: {text!r}")
        return float(matches[-1])

    def _get_live_parameter(self, node: str, name: str, directory: Path) -> float:
        text = self._run_cli(["ros2", "param", "get", node, name], directory / f"{node.strip('/').replace('/', '__')}__{name}.txt")
        return self._extract_float(text)

    def _verify_calibration_configuration(self, *, selector_min: float, selector_max: float) -> None:
        """Fail early unless the temporary calibration patch reached live nodes."""
        evidence = self.session_dir / "preflight" / f"launch_{self._launch_index:02d}"
        expected = self.config["preflight"]
        values = {
            "accel_to_current_gain": self._get_live_parameter("/ackermann_to_vesc_node", "accel_to_current_gain", evidence),
            "accel_to_brake_gain": self._get_live_parameter("/ackermann_to_vesc_node", "accel_to_brake_gain", evidence),
            "servo_min": self._get_live_parameter("/vesc_driver_node", "servo_min", evidence),
            "servo_max": self._get_live_parameter("/vesc_driver_node", "servo_max", evidence),
        }
        tolerances = 1e-9
        required = {
            "accel_to_current_gain": float(expected["required_accel_to_current_gain"]),
            "accel_to_brake_gain": float(expected["required_accel_to_brake_gain"]),
            "servo_min": float(expected["required_servo_min"]),
            "servo_max": float(expected["required_servo_max"]),
        }
        errors = {key: {"required": required[key], "observed": values[key]}
                  for key in required if abs(values[key] - required[key]) > tolerances}
        # Selector ownership must be singular: no direct raw publisher may race it.
        topic_info = self._run_cli(["ros2", "topic", "info", "--verbose", "/commands/servo/position"],
                                   evidence / "commands_servo_position_info.txt")
        count_match = re.search(r"Publisher count:\s*(\d+)", topic_info)
        publishers = int(count_match.group(1)) if count_match else None
        report = {
            "temporary_calibration_configuration_required": required,
            "observed": values,
            "selector_numeric_range": {"raw_min": selector_min, "raw_max": selector_max},
            "commands_servo_position_publisher_count": publishers,
            "errors": errors,
        }
        dump_yaml(evidence / "calibration_configuration_check.yaml", report)
        if errors:
            raise RuntimeError(
                "Calibration preflight failed after the temporary VESC configuration transaction. "
                "The live nodes do not expose VEL_TO_ERPM/full numeric servo domain as expected. "
                f"Observed: {errors}"
            )
        if publishers is not None and publishers != 1:
            raise RuntimeError(
                f"Expected exactly one publisher on /commands/servo/position (servo_selector); found {publishers}."
            )

    def _stage_done(self, stage_name: str) -> bool:
        return self._manifest().get("stages", {}).get(stage_name, {}).get("status") == "completed"

    def _completed_stage_result(self, stage_name: str) -> dict[str, Any]:
        result = self.runtime.get(stage_name)
        if result is not None:
            return result
        result_path = self.session_dir / stage_name / "runtime_result.json"
        if result_path.is_file():
            import json
            result = json.loads(result_path.read_text(encoding="utf-8"))
            self.runtime[stage_name] = result
            self._save_runtime()
            return result
        raise RuntimeError(
            f"Stage {stage_name} is marked completed, but no runtime result was found. "
            f"Expected {result_path} or an entry in runtime_state.json."
        )

    def _run_stage(self, stage_name: str, topic_group: str, fn: Callable[[Path], dict[str, Any]]) -> dict[str, Any] | None:
        if self._stage_done(stage_name):
            print(f"Skipping completed stage: {stage_name}")
            return self._completed_stage_result(stage_name)
        stage_dir = self.session_dir / stage_name
        if stage_dir.exists():
            stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            archive = self.session_dir / "incomplete_attempts" / f"{stage_name}_{stamp}"
            archive.parent.mkdir(parents=True, exist_ok=True)
            shutil.move(str(stage_dir), str(archive))
            manifest = self._manifest()
            manifest.setdefault("incomplete_attempts", []).append({
                "stage": stage_name, "archived_to": str(archive.relative_to(self.session_dir)),
            })
            dump_yaml(self.session_dir / "session_manifest.yaml", manifest)
        self._check_disk()
        bag: BagProcess | None = None
        result: dict[str, Any] | None = None
        status = "failed"
        verification: dict[str, Any] | None = None
        body_error: BaseException | None = None
        try:
            bag = start_bag(stage_dir, self.topic_policy["recording"],
                            list(self.topic_policy["required"][topic_group]), self.root,
                            redundancy_topics=list(self.topic_policy.get("redundancy_topics", [])))
            print(f"\nMCAP recording started: {bag.bag_dir}")
            result = fn(stage_dir)
            status = "completed"
        except KeyboardInterrupt as exc:
            status = "interrupted"
            body_error = exc
        except BaseException as exc:
            status = "failed"
            body_error = exc
        finally:
            if bag:
                verification = stop_bag(bag)
                if status == "completed" and not verification["ok"]:
                    status = "failed"
            stage_summary = {
                "status": status,
                "directory": str(stage_dir.relative_to(self.session_dir)),
                "bag_verification": verification,
            }
            manifest = self._manifest()
            manifest.setdefault("stages", {})[stage_name] = stage_summary
            dump_yaml(self.session_dir / "session_manifest.yaml", manifest)
            print(f"MCAP recording stopped for {stage_name}: {status}")
        if body_error:
            raise body_error
        if verification and not verification["ok"]:
            raise RuntimeError(
                f"Stage {stage_name} recorded an incomplete topic set. Missing/empty required topics: "
                f"{verification['missing_or_empty_required_topics']}. Raw bag preserved; do not use it for fitting."
            )
        assert result is not None
        self.runtime[stage_name] = result
        self._save_runtime()
        return result

    def run(self) -> None:
        full_low = float(self.config["endstops"]["raw_servo_domain_min"])
        full_high = float(self.config["endstops"]["raw_servo_domain_max"])
        banner(
            "PREPARING STEERING CALIBRATION",
            "The runner will snapshot VESC configuration, apply the temporary calibration settings, build, run, restore, and rebuild.",
        )
        print(f"Session directory: {self.session_dir}")
        print("The operator does not edit vesc.yaml, run colcon, launch ROS, or start rosbag manually.")
        self._archive_vesc_source_config()
        try:
            transaction = self.config_transaction.activate()
            self._update_manifest(vesc_config_transaction=transaction)
            require_ready("Type READY to begin the automatic preflight, or ABORT to stop")
            self._launch("calibration_stack", raw_min=full_low, raw_max=full_high)
            self._verify_calibration_configuration(selector_min=full_low, selector_max=full_high)
            audit = self._run_stage("00_command_chain_audit", "full_stationary", lambda directory: raw_command_path_audit(self.config, directory))
            centre = self._run_stage("01_zero_curvature_centre", "full_motion", lambda directory: zero_curvature_centre(self.config, directory))
            if audit is None or centre is None:
                raise RuntimeError("required raw-command audit or centre result unavailable")
            # On-ground stationary IMU-bias epoch. Runs here while still on the
            # ground (calibration_stack active) so it adds no stand<->ground move.
            self._run_stage("01b_imu_bias_ground", "imu_stationary",
                            lambda directory: imu_bias_ground(self.config, directory, centre))

            self._launch("hardware_only", raw_min=full_low, raw_max=full_high)
            limits = self._run_stage("02_physical_endstops", "hardware_only",
                                     lambda directory: physical_endstops(self.config, directory, centre))
            if limits is None:
                raise RuntimeError("physical end-stop result unavailable")
            active_limits = {
                "raw_low_safe": float(limits["raw_low_safe"]),
                "raw_high_safe": float(limits["raw_high_safe"]),
                "centre_servo_raw": float(limits["centre_servo_raw"]),
                "applied_after_stage": "02_physical_endstops",
            }
            dump_yaml(self.session_dir / "active_selector_limits.yaml", active_limits)
            # Deliberately a proposal only: candidate safety limits are reviewed
            # with the raw bags before they are installed in normal vehicle config.
            dump_yaml(self.session_dir / "proposed_post_session_vesc_servo_limits.yaml", {
                "/**": {"ros__parameters": {
                    "servo_min": active_limits["raw_low_safe"],
                    "servo_max": active_limits["raw_high_safe"],
                }}
            })
            self._update_manifest(active_selector_limits=active_limits)

            # Restart selector with discovered SAFE limits. From this point the
            # temporary full [0,1] VESC numeric domain cannot induce a raw command
            # outside the physical range discovered in Stage 2.
            self._launch("calibration_stack", raw_min=active_limits["raw_low_safe"], raw_max=active_limits["raw_high_safe"])
            self._verify_calibration_configuration(selector_min=active_limits["raw_low_safe"], selector_max=active_limits["raw_high_safe"])
            self._run_stage("03_sensor_observability", "full_motion", lambda directory: sensor_observability(self.config, directory, centre, limits))
            self._run_stage("04_static_map_training", "full_motion", lambda directory: static_map(self.config, directory, centre, limits, validation=False))
            self._run_stage("05_static_map_holdout", "full_motion", lambda directory: static_map(self.config, directory, centre, limits, validation=True))
            self._run_stage("06_command_to_curvature_response", "full_motion", lambda directory: command_to_curvature_response(self.config, directory, centre, limits))
            self._update_manifest(status="completed", completed_utc=dt.datetime.now(dt.timezone.utc).isoformat())
            banner("SESSION COMPLETE")
            print(f"All raw MCAP bags and evidence snapshots are in:\n  {self.session_dir}")
            print("Next: python3 analysis/run_analysis.py <session_directory>")
            print("The original VESC source configuration will now be restored automatically.")
            print("The discovered endpoint proposal is preserved for analyst review at:")
            print(f"  {self.session_dir / 'proposed_post_session_vesc_servo_limits.yaml'}")
        except KeyboardInterrupt:
            self._update_manifest(status="interrupted")
            print("\nSession interrupted. Complete and incomplete bags were preserved. Resume with:")
            print(f"  python3 steer_calibration.py --resume {self.session_dir}")
            raise
        except Exception:
            self._update_manifest(status="failed")
            raise
        finally:
            self._stop_stack()
            restored = self.config_transaction.restore(build=True)
            if restored is not None:
                self._update_manifest(vesc_config_transaction=restored)
                print("Original VESC configuration restored and workspace rebuilt.")
