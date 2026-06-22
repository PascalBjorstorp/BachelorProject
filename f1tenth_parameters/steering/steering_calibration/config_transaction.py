"""Reversible VESC calibration configuration transaction.

The steering calibration needs two temporary VESC settings:
- VEL_TO_ERPM selection (both acceleration-to-current gains set to zero), and
- the full numeric servo command domain [0.0, 1.0] so configured limits do
  not mask the physical end-stop survey.

This module never infers a safe physical servo range.  It only removes the
software limits during the manual survey and restores the exact original
``vesc.yaml`` after the session.
"""
from __future__ import annotations

import hashlib
import json
import os
import shutil
import subprocess
import tempfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml


class ConfigTransactionError(RuntimeError):
    """Raised when temporary calibration configuration cannot be managed safely."""


@dataclass(frozen=True)
class CalibrationPatch:
    operation_mode: str = "VEL_TO_ERPM"
    accel_to_current_gain: float = 0.0
    accel_to_brake_gain: float = 0.0
    servo_min: float = 0.0
    servo_max: float = 1.0

    def as_dict(self) -> dict[str, Any]:
        return {
            "operation_mode": self.operation_mode,
            "accel_to_current_gain": self.accel_to_current_gain,
            "accel_to_brake_gain": self.accel_to_brake_gain,
            "servo_min": self.servo_min,
            "servo_max": self.servo_max,
        }


class VescConfigTransaction:
    """Own the temporary edit/build/restore cycle for a calibration session."""

    LOCK_NAME = ".steering_calibration_vesc_config_recovery.json"

    def __init__(
        self,
        *,
        steering_root: Path,
        session_dir: Path,
        workspace: Path | None = None,
        config_relpath: str = "f1tenth_system/f1tenth_stack/config/vesc.yaml",
        build_command: list[str] | None = None,
    ) -> None:
        self.steering_root = steering_root.resolve()
        self.session_dir = session_dir.resolve()
        self.workspace = self._resolve_workspace(workspace, config_relpath)
        self.config_path = (self.workspace / config_relpath).resolve()
        self.build_command = build_command or ["colcon", "build", "--symlink-install"]
        self.patch = CalibrationPatch()
        self.transaction_dir = self.session_dir / "vesc_config_transaction"
        self.backup_path = self.transaction_dir / "vesc.yaml.before_calibration"
        self.metadata_path = self.transaction_dir / "transaction.json"
        self.lock_path = self.workspace / self.LOCK_NAME
        self.active = False
        self._restored = False

    @staticmethod
    def _sha256(path: Path) -> str:
        digest = hashlib.sha256()
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()

    def _resolve_workspace(self, requested: Path | None, config_relpath: str) -> Path:
        candidates: list[Path] = []
        if requested is not None:
            candidates.append(requested.expanduser().resolve())
        env_workspace = os.environ.get("STEERING_CALIBRATION_WORKSPACE")
        if env_workspace:
            candidates.append(Path(env_workspace).expanduser().resolve())
        # steering/ is expected at <workspace>/f1tenth_parameters/steering
        candidates.extend(self.steering_root.parents)
        for candidate in candidates:
            if (candidate / config_relpath).is_file():
                return candidate
        raise ConfigTransactionError(
            "Cannot locate f1tenth_system/f1tenth_stack/config/vesc.yaml. "
            "Run from the BachelorProject workspace or pass --workspace <path>."
        )

    def _load(self) -> dict[str, Any]:
        try:
            value = yaml.safe_load(self.config_path.read_text(encoding="utf-8"))
        except OSError as exc:
            raise ConfigTransactionError(f"Cannot read {self.config_path}: {exc}") from exc
        if not isinstance(value, dict):
            raise ConfigTransactionError(f"Expected mapping YAML in {self.config_path}")
        return value

    @staticmethod
    def _params(root: dict[str, Any], node: str) -> dict[str, Any]:
        node_cfg = root.get(node)
        if not isinstance(node_cfg, dict):
            raise ConfigTransactionError(f"Missing YAML node section: {node}")
        params = node_cfg.get("ros__parameters")
        if not isinstance(params, dict):
            raise ConfigTransactionError(f"Missing ros__parameters under {node}")
        return params

    def _atomic_dump(self, value: dict[str, Any]) -> None:
        self.config_path.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=self.config_path.parent,
            prefix=f".{self.config_path.name}.", suffix=".tmp", delete=False,
        ) as handle:
            yaml.safe_dump(value, handle, sort_keys=False)
            tmp = Path(handle.name)
        os.replace(tmp, self.config_path)

    def _write_lock(self, metadata: dict[str, Any]) -> None:
        self.lock_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    def _run_build(self, log_name: str) -> None:
        if shutil.which(self.build_command[0]) is None:
            raise ConfigTransactionError(f"Cannot find build command: {self.build_command[0]}")
        self.transaction_dir.mkdir(parents=True, exist_ok=True)
        log_path = self.transaction_dir / log_name
        print("\n" + "=" * 68)
        print("BUILDING ROS WORKSPACE FOR STEERING CALIBRATION")
        print("=" * 68)
        print("Command:", " ".join(self.build_command))
        print("Workspace:", self.workspace)
        print("Build log:", log_path)
        with log_path.open("w", encoding="utf-8") as handle:
            result = subprocess.run(
                self.build_command,
                cwd=self.workspace,
                stdout=handle,
                stderr=subprocess.STDOUT,
                text=True,
                check=False,
            )
        if result.returncode != 0:
            tail = log_path.read_text(encoding="utf-8", errors="replace").splitlines()[-60:]
            raise ConfigTransactionError(
                f"colcon build failed (exit {result.returncode}). Last build-log lines:\n" + "\n".join(tail)
            )
        print("Build completed successfully.")

    def _metadata(self, original_values: dict[str, Any], original_sha: str, state: str) -> dict[str, Any]:
        return {
            "state": state,
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "workspace": str(self.workspace),
            "source_config": str(self.config_path),
            "backup": str(self.backup_path),
            "original_sha256": original_sha,
            "original_values": original_values,
            "calibration_patch": self.patch.as_dict(),
            "build_command": self.build_command,
            "restore_required": state != "restored",
        }

    def _archive_previous_restored_transaction(self) -> None:
        """Preserve previous restored transaction evidence when resuming a session."""
        if not self.transaction_dir.exists() or not self.metadata_path.is_file():
            return
        try:
            metadata = json.loads(self.metadata_path.read_text(encoding="utf-8"))
        except Exception:
            return
        if metadata.get("state") != "restored":
            return
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        history = self.session_dir / "vesc_config_transaction_history" / stamp
        history.parent.mkdir(parents=True, exist_ok=True)
        shutil.move(str(self.transaction_dir), str(history))

    def activate(self) -> dict[str, Any]:
        """Back up, patch and build the calibration VESC configuration."""
        if self.lock_path.exists():
            raise ConfigTransactionError(
                "A previous steering-calibration configuration transaction was not restored. "
                f"Run: python3 {self.steering_root / 'steer_calibration.py'} --recover "
                f"--workspace {self.workspace}"
            )
        if not self.config_path.is_file():
            raise ConfigTransactionError(f"VESC config does not exist: {self.config_path}")

        self._archive_previous_restored_transaction()
        self.transaction_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(self.config_path, self.backup_path)
        original_sha = self._sha256(self.backup_path)
        source = self._load()
        global_params = self._params(source, "/**")
        ackermann_params = self._params(source, "ackermann_to_vesc_node")
        original_values = {
            "operation_mode": ackermann_params.get("operation_mode"),
            "accel_to_current_gain": ackermann_params.get("accel_to_current_gain"),
            "accel_to_brake_gain": ackermann_params.get("accel_to_brake_gain"),
            "servo_min": global_params.get("servo_min"),
            "servo_max": global_params.get("servo_max"),
        }

        # ``operation_mode`` is recorded for clarity. The current C++ selector
        # actually uses the two acceleration gains, so both are set to zero.
        ackermann_params["operation_mode"] = self.patch.operation_mode
        ackermann_params["accel_to_current_gain"] = self.patch.accel_to_current_gain
        ackermann_params["accel_to_brake_gain"] = self.patch.accel_to_brake_gain
        global_params["servo_min"] = self.patch.servo_min
        global_params["servo_max"] = self.patch.servo_max

        metadata = self._metadata(original_values, original_sha, "patch_written")
        self._write_lock(metadata)
        self.metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        try:
            self._atomic_dump(source)
            self._run_build("build_apply.log")
        except BaseException:
            # Source configuration must not remain modified after a failed or
            # interrupted build. A recovery lock remains if restore fails.
            try:
                self.restore(build=True)
            except Exception:
                pass
            raise

        self.active = True
        metadata["state"] = "active"
        metadata["activated_utc"] = datetime.now(timezone.utc).isoformat()
        self._write_lock(metadata)
        self.metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return metadata

    def restore(self, *, build: bool = True) -> dict[str, Any] | None:
        """Restore byte-exact source config and rebuild the installed workspace."""
        if self._restored:
            if self.metadata_path.is_file():
                return json.loads(self.metadata_path.read_text(encoding="utf-8"))
            return None
        if not self.backup_path.is_file():
            return None
        shutil.copy2(self.backup_path, self.config_path)
        metadata = json.loads(self.metadata_path.read_text(encoding="utf-8")) if self.metadata_path.is_file() else {}
        metadata["state"] = "source_restored"
        metadata["source_restored_utc"] = datetime.now(timezone.utc).isoformat()
        metadata["restore_required"] = False
        self.metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        if build:
            try:
                self._run_build("build_restore.log")
            except Exception as exc:
                # Source is safe but installed/share config may still be the
                # calibration version. Keep the recovery lock and fail loudly.
                metadata["state"] = "restore_build_failed"
                metadata["restore_required"] = True
                metadata["restore_error"] = str(exc)
                self.metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
                self._write_lock(metadata)
                raise ConfigTransactionError(
                    "Original source vesc.yaml was restored, but rebuild of the installed workspace failed. "
                    "Do not drive the car. Fix the build and run --recover.\n" + str(exc)
                ) from exc
        metadata["state"] = "restored"
        metadata["restored_utc"] = datetime.now(timezone.utc).isoformat()
        metadata["restore_required"] = False
        self.metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        try:
            self.lock_path.unlink()
        except FileNotFoundError:
            pass
        self.active = False
        self._restored = True
        return metadata

    @classmethod
    def recover(cls, *, steering_root: Path, workspace: Path | None = None) -> None:
        """Restore a configuration left active by a crash, power loss or kill."""
        candidates = []
        if workspace is not None:
            candidates.append(workspace.expanduser().resolve())
        env_workspace = os.environ.get("STEERING_CALIBRATION_WORKSPACE")
        if env_workspace:
            candidates.append(Path(env_workspace).expanduser().resolve())
        candidates.extend(steering_root.resolve().parents)
        lock_path: Path | None = None
        for candidate in candidates:
            maybe = candidate / cls.LOCK_NAME
            if maybe.is_file():
                lock_path = maybe
                break
        if lock_path is None:
            raise ConfigTransactionError("No pending steering-calibration recovery file was found.")
        metadata = json.loads(lock_path.read_text(encoding="utf-8"))
        source = Path(metadata["source_config"])
        backup = Path(metadata["backup"])
        workspace_path = Path(metadata["workspace"])
        if not backup.is_file():
            raise ConfigTransactionError(f"Cannot recover; backup does not exist: {backup}")
        shutil.copy2(backup, source)
        command = list(metadata.get("build_command", ["colcon", "build", "--symlink-install"]))
        print("Restoring original VESC configuration from:", backup)
        print("Rebuilding with:", " ".join(command))
        result = subprocess.run(command, cwd=workspace_path, check=False)
        if result.returncode != 0:
            raise ConfigTransactionError(
                "Source vesc.yaml is restored but colcon build failed. Do not drive the car until build succeeds."
            )
        lock_path.unlink()
        print("Recovery complete: original VESC configuration restored and workspace rebuilt.")
