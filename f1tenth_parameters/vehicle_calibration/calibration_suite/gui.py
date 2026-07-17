"""Local, dependency-light web interface for the calibration campaign.

The browser is presentation only.  Every stage still runs through
``run_suite.py`` in a pseudo-terminal, so the tested runner owns dependency
gates, ROS launch, recording, analysis, reversible updates and acceptance.
"""
from __future__ import annotations

import codecs
import copy
import datetime as dt
import errno
import json
import math
import mimetypes
import os
import pty
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
import urllib.parse
import webbrowser
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import yaml

from .guidance import guide_for
from .runner import STAGES, STAGE_BY_KEY, VALIDATION_REDO_TARGETS, SuiteRunner
from .stage_report import _scalar_lines


ROOT = Path(__file__).resolve().parents[1]
STATIC_ROOT = ROOT / "gui_static"
GRAVITY_MPS2 = 9.80665
ANSI_ESCAPE = re.compile(r"\x1b(?:\[[0-?]*[ -/]*[@-~]|\][^\x07]*(?:\x07|\x1b\\))")
ALLOWED_SESSION_SUFFIXES = {
    ".json", ".yaml", ".yml", ".md", ".txt", ".log", ".tex", ".pdf", ".png", ".svg", ".csv",
}
EDITABLE_REQUIRED = ("mass_kg", "wheelbase_m", "front_axle_load_N", "rear_axle_load_N")
EDITABLE_RECOMMENDED = (
    "vehicle_length_m", "vehicle_width_m", "vehicle_height_m", "track_front_m", "track_rear_m",
    "loaded_wheel_radius_m", "wheel_width_m", "cg_height_m", "yaw_inertia_kg_m2",
)
EDITABLE_BIFILAR = ("rope_spacing_m", "rope_length_m", "period_s")
EDITABLE_CONTEXT = ("operator", "measured_utc", "surface", "battery_state", "notes")


def _current_metrology_schema(document: dict[str, Any]) -> dict[str, Any]:
    """Return the value-only metrology schema, including for old sessions."""
    result = copy.deepcopy(document)
    for section_name in ("required", "recommended", "bifilar_yaw_inertia"):
        section = result.get(section_name, {})
        if not isinstance(section, dict):
            continue
        for item in section.values():
            if not isinstance(item, dict):
                continue
            item.pop("stddev", None)
            method = item.get("method")
            if isinstance(method, str):
                item["method"] = method.replace(
                    "; confirm and enter scale one-sigma uncertainty",
                    "; confirm against the race-ready car",
                ).replace(
                    "; confirm and enter one-sigma uncertainty",
                    "; confirm against the race-ready car",
                )
    context = result.get("measurement_context", {})
    if isinstance(context, dict) and isinstance(context.get("notes"), str):
        context["notes"] = context["notes"].replace(
            "enter the missing uncertainties and axle-scale readings",
            "enter the axle-scale readings",
        )
    return result


def _load_yaml(path: Path) -> dict[str, Any]:
    """Read a mapping, retrying briefly if another process is replacing it."""
    for attempt in range(3):
        try:
            value = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            if not isinstance(value, dict):
                raise ValueError(f"expected YAML mapping: {path}")
            return value
        except (OSError, yaml.YAMLError):
            if attempt == 2:
                raise
            time.sleep(0.02)
    return {}


def _write_yaml_atomic(path: Path, value: dict[str, Any]) -> None:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")
    os.replace(temporary, path)


def _number(value: Any, *, positive: bool = False) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(result) or (positive and result <= 0.0):
        return None
    return result


def validate_metrology(document: dict[str, Any]) -> dict[str, Any]:
    """Validate the editable form without producing analysis artifacts."""
    errors: dict[str, str] = {}
    required = document.get("required", {})
    if not isinstance(required, dict):
        return {"ready": False, "errors": {"required": "Required measurements are missing."}}

    for name in EDITABLE_REQUIRED:
        item = required.get(name, {})
        value = _number(item.get("value") if isinstance(item, dict) else None, positive=True)
        if value is None:
            errors[f"required.{name}.value"] = "Enter a positive SI value."

    # Geometry is read-only in the GUI but remains a hard prerequisite.
    signed_geometry = {
        "lidar_to_base_x_m", "lidar_to_base_y_m", "lidar_to_base_yaw_rad",
        "base_link_to_rear_axle_x_m", "base_link_to_rear_axle_y_m",
        "imu_to_base_x_m", "imu_to_base_y_m", "imu_to_base_z_m", "imu_to_base_yaw_rad",
    }
    for name in signed_geometry:
        item = required.get(name, {})
        if _number(item.get("value") if isinstance(item, dict) else None) is None:
            errors[f"required.{name}.value"] = "Authoritative deployed geometry was not imported."

    mass_item = required.get("mass_kg", {})
    front_item = required.get("front_axle_load_N", {})
    rear_item = required.get("rear_axle_load_N", {})
    mass = _number(mass_item.get("value") if isinstance(mass_item, dict) else None, positive=True)
    front = _number(front_item.get("value") if isinstance(front_item, dict) else None, positive=True)
    rear = _number(rear_item.get("value") if isinstance(rear_item, dict) else None, positive=True)
    closure: dict[str, Any] = {}
    if None not in (mass, front, rear):
        expected = float(mass) * GRAVITY_MPS2
        measured = float(front) + float(rear)
        error = measured - expected
        # This is a practical consistency check, not a fabricated uncertainty
        # estimate. Five percent catches unit or scale mistakes without asking
        # the operator for meaningless standard deviations.
        tolerance = 0.05 * expected
        closure = {
            "expected_weight_N": expected,
            "entered_axle_sum_N": measured,
            "error_N": error,
            "tolerance_N": tolerance,
            "accepted": abs(error) <= tolerance,
        }
        if abs(error) > tolerance:
            errors["required.axle_load_closure"] = (
                f"Front + rear differs from mass × g by {error:+.2f} N; allowed ±{tolerance:.2f} N."
            )

    bifilar = document.get("bifilar_yaw_inertia", {})
    if isinstance(bifilar, dict):
        entered = [bifilar.get(name, {}).get("value") if isinstance(bifilar.get(name), dict) else None for name in EDITABLE_BIFILAR]
        if any(value not in (None, "") for value in entered):
            for name in EDITABLE_BIFILAR:
                item = bifilar.get(name, {})
                if _number(item.get("value") if isinstance(item, dict) else None, positive=True) is None:
                    errors[f"bifilar_yaw_inertia.{name}.value"] = "Complete all three positive bifilar measurements or clear all three."

    return {"ready": not errors, "errors": errors, "axle_load_closure": closure}


class TerminalJob:
    """Run one interactive runner command behind a pseudo-terminal."""

    MAX_LOG_CHARS = 400_000

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self.process: subprocess.Popen[bytes] | None = None
        self.master_fd: int | None = None
        self.label: str | None = None
        self.command: list[str] = []
        self.status = "idle"
        self.exit_code: int | None = None
        self.started_utc: str | None = None
        self.finished_utc: str | None = None
        self._log = ""
        self._base_cursor = 0

    @property
    def running(self) -> bool:
        with self._lock:
            return self.process is not None and self.process.poll() is None

    def _append(self, text: str) -> None:
        clean = ANSI_ESCAPE.sub("", text).replace("\r\n", "\n")
        with self._lock:
            self._log += clean
            if len(self._log) > self.MAX_LOG_CHARS:
                removed = len(self._log) - self.MAX_LOG_CHARS
                self._log = self._log[removed:]
                self._base_cursor += removed

    def start(self, command: list[str], *, cwd: Path, label: str) -> None:
        with self._lock:
            if self.running:
                raise RuntimeError(f"job already running: {self.label}")
            master_fd, slave_fd = pty.openpty()
            environment = dict(os.environ)
            environment["PYTHONUNBUFFERED"] = "1"
            try:
                process = subprocess.Popen(
                    command,
                    cwd=cwd,
                    stdin=slave_fd,
                    stdout=slave_fd,
                    stderr=slave_fd,
                    env=environment,
                    start_new_session=True,
                    close_fds=True,
                )
            except Exception:
                os.close(master_fd)
                os.close(slave_fd)
                raise
            os.close(slave_fd)
            self.process = process
            self.master_fd = master_fd
            self.label = label
            self.command = list(command)
            self.status = "running"
            self.exit_code = None
            self.started_utc = dt.datetime.now(dt.timezone.utc).isoformat()
            self.finished_utc = None
            self._log = ""
            self._base_cursor = 0
            self._append(f"Starting {label}\n")
            threading.Thread(target=self._read_loop, name="calibration-gui-pty", daemon=True).start()
            threading.Thread(target=self._wait_loop, name="calibration-gui-wait", daemon=True).start()

    def _read_loop(self) -> None:
        decoder = codecs.getincrementaldecoder("utf-8")(errors="replace")
        while True:
            with self._lock:
                fd = self.master_fd
                process = self.process
            if fd is None:
                break
            try:
                chunk = os.read(fd, 4096)
            except OSError as exc:
                if exc.errno in {errno.EIO, errno.EBADF}:
                    break
                self._append(f"\n[GUI PTY read error: {exc}]\n")
                break
            if not chunk:
                break
            self._append(decoder.decode(chunk))
            if process is not None and process.poll() is not None and len(chunk) == 0:
                break
        final = decoder.decode(b"", final=True)
        if final:
            self._append(final)

    def _wait_loop(self) -> None:
        with self._lock:
            process = self.process
        if process is None:
            return
        code = process.wait()
        # Give the reader a moment to drain the final PTY output.
        time.sleep(0.05)
        with self._lock:
            self.exit_code = code
            if self.status != "stopping":
                self.status = "succeeded" if code == 0 else "failed"
            else:
                self.status = "stopped" if code != 0 else "succeeded"
            self.finished_utc = dt.datetime.now(dt.timezone.utc).isoformat()
            fd = self.master_fd
            self.master_fd = None
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass
        self._append(f"\n{self.label or 'Job'} finished with exit code {code}.\n")

    def write(self, response: str, *, newline: bool = True) -> None:
        if not isinstance(response, str) or not response or len(response) > 200:
            raise ValueError("response must contain 1–200 characters")
        if any(ord(character) < 32 and character not in "\t" for character in response):
            raise ValueError("response contains unsupported control characters")
        with self._lock:
            if not self.running or self.master_fd is None:
                raise RuntimeError("no interactive job is running")
            payload = response + ("\n" if newline else "")
            os.write(self.master_fd, payload.encode("utf-8"))

    def stop(self) -> None:
        with self._lock:
            if not self.running or self.process is None:
                raise RuntimeError("no job is running")
            self.status = "stopping"
            pid = self.process.pid
        os.killpg(pid, signal.SIGINT)

    def wait(self, timeout: float | None = None) -> int | None:
        with self._lock:
            process = self.process
        if process is None:
            return self.exit_code
        return process.wait(timeout=timeout)

    def snapshot(self, cursor: int | None = None) -> dict[str, Any]:
        with self._lock:
            requested = self._base_cursor if cursor is None else max(int(cursor), self._base_cursor)
            local = requested - self._base_cursor
            chunk = self._log[local:]
            next_cursor = self._base_cursor + len(self._log)
            return {
                "label": self.label,
                "status": self.status,
                "running": self.running,
                "exit_code": self.exit_code,
                "started_utc": self.started_utc,
                "finished_utc": self.finished_utc,
                "log": chunk,
                "cursor": next_cursor,
                "cursor_reset": cursor is not None and cursor < self._base_cursor,
            }


class GuiController:
    """Safe state and command boundary used by the HTTP handler."""

    def __init__(
        self,
        config_path: Path,
        workspace: Path,
        session: Path | None = None,
        *,
        runs_root: Path | None = None,
    ) -> None:
        self.config_path = config_path.expanduser().resolve()
        self.workspace = workspace.expanduser().resolve()
        suite = _load_yaml(self.config_path)
        configured_runs = str(suite.get("campaign", {}).get("runs_dir", "runs"))
        self.runs_root = (runs_root or (self.config_path.parent.parent / configured_runs)).expanduser().resolve()
        self.runs_root.mkdir(parents=True, exist_ok=True)
        self.active_session: Path | None = None
        self.job = TerminalJob()
        self._lock = threading.RLock()
        if session is not None:
            candidate = session.expanduser().resolve()
            if not (candidate / "session_manifest.yaml").is_file():
                raise FileNotFoundError(f"not a calibration session: {candidate}")
            self.active_session = candidate

    def _runner_command(self, command: str, *extra: str) -> list[str]:
        args = [sys.executable, str(ROOT / "run_suite.py"), command, "--config", str(self.config_path)]
        if command != "recover":
            if self.active_session is None:
                raise RuntimeError("create or select a session first")
            args += ["--session", str(self.active_session)]
        args += ["--workspace", str(self.workspace), *extra]
        return args

    @staticmethod
    def _manifest_compatibility(manifest: dict[str, Any]) -> tuple[bool, str | None]:
        expected = [stage.key for stage in STAGES]
        if manifest.get("stage_order") != expected:
            return False, (
                "This session uses an older test sequence. Inspect its files or "
                "create a new session to run the current campaign."
            )
        return True, None

    def sessions(self) -> list[dict[str, Any]]:
        rows: list[dict[str, Any]] = []
        for manifest_path in sorted(self.runs_root.glob("*/session_manifest.yaml"), reverse=True):
            try:
                manifest = _load_yaml(manifest_path)
            except Exception:
                continue
            entries = manifest.get("stages", {}) if isinstance(manifest.get("stages"), dict) else {}
            completed = sum(isinstance(value, dict) and value.get("status") == "completed" for value in entries.values())
            compatible, incompatibility_reason = self._manifest_compatibility(manifest)
            frozen_order = manifest.get("stage_order")
            frozen_total = len(frozen_order) if isinstance(frozen_order, list) else len(STAGES)
            rows.append({
                "id": manifest_path.parent.name,
                "status": manifest.get("status", "unknown"),
                "created_utc": manifest.get("created_utc"),
                "completed": completed,
                "total": frozen_total,
                "active": self.active_session == manifest_path.parent.resolve(),
                "compatible": compatible,
                "incompatibility_reason": incompatibility_reason,
            })
        return rows

    def create_session(self) -> Path:
        with self._lock:
            if self.job.running:
                raise RuntimeError("cannot create a session while a job is running")
            stem = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ_vehicle_calibration")
            target = self.runs_root / stem
            index = 1
            while target.exists():
                target = self.runs_root / f"{stem}_{index:02d}"
                index += 1
            runner = SuiteRunner(self.config_path, target, self.workspace)
            self.active_session = runner.session.resolve()
            return self.active_session

    def select_session(self, session_id: str) -> Path:
        if self.job.running:
            raise RuntimeError("cannot change session while a job is running")
        if not isinstance(session_id, str) or not session_id or Path(session_id).name != session_id:
            raise ValueError("invalid session id")
        target = (self.runs_root / session_id).resolve()
        if target.parent != self.runs_root or not (target / "session_manifest.yaml").is_file():
            raise FileNotFoundError("calibration session not found")
        self.active_session = target
        return target

    def _sheet(self) -> tuple[Path, dict[str, Any]]:
        if self.active_session is None:
            raise RuntimeError("create or select a session first")
        path = self.active_session / "physical_measurements.yaml"
        if not path.is_file():
            raise FileNotFoundError(path)
        return path, _load_yaml(path)

    @staticmethod
    def _parse_form_number(value: Any, *, optional: bool) -> float | None:
        if value in (None, ""):
            if optional:
                return None
            raise ValueError("a numeric value is required")
        number = _number(value)
        if number is None:
            raise ValueError(f"not a finite number: {value!r}")
        return number

    def update_metrology(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self.job.running:
            raise RuntimeError("measurements cannot be edited while a job is running")
        path, document = self._sheet()
        for section, names in (
            ("required", EDITABLE_REQUIRED),
            ("recommended", EDITABLE_RECOMMENDED),
            ("bifilar_yaw_inertia", EDITABLE_BIFILAR),
        ):
            incoming = payload.get(section, {})
            if incoming is None:
                continue
            if not isinstance(incoming, dict):
                raise ValueError(f"{section} must be a mapping")
            target_section = document.setdefault(section, {})
            if not isinstance(target_section, dict):
                raise ValueError(f"measurement sheet {section} is not a mapping")
            unknown = set(incoming) - set(names) - ({"repetitions"} if section == "bifilar_yaw_inertia" else set())
            if unknown:
                raise ValueError(f"unsupported {section} fields: {', '.join(sorted(unknown))}")
            for name in names:
                if name not in incoming:
                    continue
                values = incoming[name]
                if not isinstance(values, dict) or set(values) - {"value"}:
                    raise ValueError(f"{section}.{name} accepts only a value")
                item = target_section.setdefault(name, {})
                if not isinstance(item, dict):
                    raise ValueError(f"{section}.{name} is not editable")
                if "value" in values:
                    item["value"] = self._parse_form_number(values["value"], optional=True)
            if section == "bifilar_yaw_inertia" and "repetitions" in incoming:
                value = incoming["repetitions"]
                if value in (None, ""):
                    target_section["repetitions"] = None
                else:
                    repetitions = int(value)
                    if repetitions <= 0:
                        raise ValueError("bifilar repetitions must be positive")
                    target_section["repetitions"] = repetitions

        context = payload.get("measurement_context")
        if context is not None:
            if not isinstance(context, dict) or set(context) - set(EDITABLE_CONTEXT):
                raise ValueError("measurement_context contains unsupported fields")
            target = document.setdefault("measurement_context", {})
            if not isinstance(target, dict):
                raise ValueError("measurement_context is not editable")
            for name, value in context.items():
                if value is not None and not isinstance(value, str):
                    raise ValueError(f"measurement_context.{name} must be text")
                target[name] = value or None

        # Saving an older, not-yet-run session also migrates its obsolete
        # standard-deviation fields. Direct ruler/scale values are value-only.
        document = _current_metrology_schema(document)
        _write_yaml_atomic(path, document)
        return validate_metrology(document)

    def _manifest(self) -> dict[str, Any]:
        if self.active_session is None:
            return {}
        return _load_yaml(self.active_session / "session_manifest.yaml")

    def _next_stage(self, manifest: dict[str, Any]) -> str | None:
        entries = manifest.get("stages", {}) if isinstance(manifest.get("stages"), dict) else {}
        for stage in STAGES:
            entry = entries.get(stage.key, {})
            if not isinstance(entry, dict) or entry.get("status") != "completed":
                return stage.key
        return None

    def _confirmations_for(self, category: str) -> list[str]:
        if category == "manual":
            return ["instructions_read", "measurements_confirmed"]
        if category == "stationary":
            return ["instructions_read", "car_secured", "area_clear", "estop_ready"]
        return ["instructions_read", "car_positioned", "area_clear", "estop_ready"]

    def run_stage(self, stage_key: str, confirmations: list[str]) -> None:
        if stage_key not in STAGE_BY_KEY:
            raise ValueError("unknown calibration stage")
        manifest = self._manifest()
        compatible, reason = self._manifest_compatibility(manifest)
        if not compatible:
            raise RuntimeError(reason or "session is incompatible with the current campaign")
        expected = self._next_stage(manifest)
        if stage_key != expected:
            raise RuntimeError(f"only the next incomplete stage may run ({expected or 'campaign complete'})")
        guide = guide_for(stage_key)
        required = set(self._confirmations_for(str(guide["category"])))
        supplied = {str(item) for item in confirmations}
        missing = sorted(required - supplied)
        if missing:
            raise ValueError(f"missing safety confirmations: {', '.join(missing)}")
        if stage_key == "physical_metrology":
            _, sheet = self._sheet()
            validation = validate_metrology(sheet)
            if not validation["ready"]:
                raise RuntimeError("physical measurement form is incomplete or inconsistent")
        self.job.start(
            self._runner_command("run", "--stage", stage_key),
            cwd=self.workspace,
            label=f"stage: {stage_key}",
        )

    def build_report(self, *, promote: bool = False, confirmation: str | None = None) -> None:
        if promote:
            manifest = self._manifest()
            if self._next_stage(manifest) is not None:
                raise RuntimeError("promotion is blocked until all stages pass")
            if confirmation != "PROMOTE":
                raise ValueError("type PROMOTE to confirm the permanent configuration update")
        command = self._runner_command("report", *( ["--promote"] if promote else [] ))
        self.job.start(command, cwd=self.workspace, label="final report and promotion" if promote else "report regeneration")

    def redo(self, stage_key: str, confirmation: str | None) -> None:
        if stage_key not in VALIDATION_REDO_TARGETS:
            raise ValueError("this stage has no fitted A-stage redo target")
        entry = self._manifest().get("stages", {}).get(stage_key, {})
        if not isinstance(entry, dict) or entry.get("status") != "failed":
            raise RuntimeError("REDO is available only for a failed validation stage")
        if confirmation != "REDO":
            raise ValueError("type REDO to archive the failed branch and restart its A stage")
        self.job.start(
            self._runner_command("redo", "--from", stage_key),
            cwd=self.workspace,
            label=f"fresh recalibration from {stage_key}",
        )

    def recover(self, confirmation: str | None) -> None:
        if confirmation != "RECOVER":
            raise ValueError("type RECOVER to restore an interrupted configuration transaction")
        self.job.start(self._runner_command("recover"), cwd=self.workspace, label="configuration recovery")

    def _readiness(self) -> dict[str, Any]:
        usage = shutil.disk_usage(self.active_session or self.runs_root)
        return {
            "ros2_cli": bool(shutil.which("ros2")),
            "colcon": bool(shutil.which("colcon")),
            "pdflatex": bool(shutil.which("pdflatex")),
            "workspace_install": (self.workspace / "install" / "setup.bash").is_file(),
            "ros_environment_sourced": bool(os.environ.get("AMENT_PREFIX_PATH")),
            "free_disk_gb": usage.free / (1024.0 ** 3),
            "recovery_lock": (self.workspace / ".vehicle_calibration_recovery.json").is_file(),
            "localhost_only": True,
        }

    def _file_url(self, path: Path) -> str | None:
        if self.active_session is None or not path.is_file():
            return None
        relative = path.resolve().relative_to(self.active_session)
        return "/session-file/" + urllib.parse.quote(relative.as_posix(), safe="/")

    def safe_session_file(self, relative: str) -> Path:
        if self.active_session is None:
            raise FileNotFoundError("no active session")
        decoded = urllib.parse.unquote(relative)
        candidate = (self.active_session / decoded).resolve()
        try:
            candidate.relative_to(self.active_session)
        except ValueError as exc:
            raise PermissionError("path leaves active session") from exc
        if not candidate.is_file() or candidate.suffix.lower() not in ALLOWED_SESSION_SUFFIXES:
            raise FileNotFoundError(candidate)
        return candidate

    def snapshot(self) -> dict[str, Any]:
        sessions = self.sessions()
        base = {
            "active_session": str(self.active_session) if self.active_session else None,
            "sessions": sessions,
            "readiness": self._readiness(),
            "job": {key: value for key, value in self.job.snapshot().items() if key != "log"},
        }
        if self.active_session is None:
            return {**base, "manifest": None, "stages": [], "next_stage": None, "metrology": None, "artifacts": {}}

        manifest = self._manifest()
        compatible, incompatibility_reason = self._manifest_compatibility(manifest)
        entries = manifest.get("stages", {}) if isinstance(manifest.get("stages"), dict) else {}
        next_stage = self._next_stage(manifest)
        budget = manifest.get("campaign_budget", {}) if isinstance(manifest.get("campaign_budget"), dict) else {}
        budget_rows = {
            row.get("stage"): row for row in budget.get("per_stage", []) if isinstance(row, dict) and row.get("stage")
        }
        _, sheet = self._sheet()
        sheet = _current_metrology_schema(sheet)
        metrology_validation = validate_metrology(sheet)
        stage_rows: list[dict[str, Any]] = []
        for index, stage in enumerate(STAGES, start=1):
            entry = entries.get(stage.key)
            entry = entry if isinstance(entry, dict) else {}
            status = str(entry.get("status", "pending"))
            guide = guide_for(stage.key)
            plot = self.active_session / "plots" / "stages" / f"{stage.key}.png"
            statistics_path = self.active_session / "analysis" / "statistics" / f"{stage.key}.yaml"
            report_path = self.active_session / str(entry.get("stage_report", "")) if entry.get("stage_report") else None
            trials = budget_rows.get(stage.key, {})
            stage_rows.append({
                "index": index,
                "key": stage.key,
                "directory": stage.directory,
                "kind": stage.kind,
                "dependencies": list(stage.dependencies),
                "status": status,
                "guide": guide,
                "nominal_trials_min": trials.get("nominal_driving_trials_min", 0),
                "nominal_trials_max": trials.get("nominal_driving_trials_max", 0),
                "trial_note": trials.get("note", ""),
                "headline_values": _scalar_lines(entry.get("analysis", {}), limit=6),
                "statistical_summary": (
                    entry.get("analysis", {}).get("statistical_evidence", {})
                    if isinstance(entry.get("analysis"), dict) else {}
                ),
                "error": entry.get("error"),
                "retry_guidance": entry.get("retry_guidance"),
                "plot_url": self._file_url(plot),
                "statistics_url": self._file_url(statistics_path),
                "report_url": self._file_url(report_path) if report_path else None,
                "can_run": stage.key == next_stage and not self.job.running and (
                    stage.key != "physical_metrology" or metrology_validation["ready"]
                ) and compatible,
                "required_confirmations": self._confirmations_for(str(guide["category"])),
                "redo_target": VALIDATION_REDO_TARGETS.get(stage.key) if status == "failed" else None,
            })

        plots = sorted((self.active_session / "plots" / "stages").glob("*.png"))
        analysis = self.active_session / "analysis"
        artifacts = {
            "pdf": self._file_url(analysis / "vehicle_calibration_report.pdf"),
            "latex": self._file_url(analysis / "vehicle_calibration_report.tex"),
            "markdown": self._file_url(analysis / "vehicle_calibration_report.md"),
            "inventory": self._file_url(analysis / "vehicle_parameter_inventory.yaml"),
            "mpc_bundle": self._file_url(analysis / "mpc_simulation_parameter_bundle.yaml"),
            "campaign_log": self._file_url(analysis / "vehicle_calibration_log.md"),
            "plots": [{"name": path.stem, "url": self._file_url(path)} for path in plots],
        }
        completed = sum(row["status"] == "completed" for row in stage_rows)
        frozen_order = manifest.get("stage_order")
        frozen_total = len(frozen_order) if isinstance(frozen_order, list) else len(STAGES)
        return {
            **base,
            "manifest": {
                "session_id": manifest.get("session_id"),
                "status": manifest.get("status"),
                "created_utc": manifest.get("created_utc"),
                "completed": completed,
                "total": frozen_total,
                "progress_percent": 100.0 * completed / max(frozen_total, 1),
                "campaign_budget": budget,
                "room_preflight": manifest.get("room_preflight"),
                "compatible": compatible,
                "incompatibility_reason": incompatibility_reason,
            },
            "next_stage": next_stage,
            "stages": stage_rows,
            "metrology": {
                "document": sheet,
                "validation": metrology_validation,
                "editable": {
                    "required": list(EDITABLE_REQUIRED),
                    "recommended": list(EDITABLE_RECOMMENDED),
                    "bifilar": list(EDITABLE_BIFILAR),
                    "context": list(EDITABLE_CONTEXT),
                },
            },
            "artifacts": artifacts,
        }

    def shutdown(self) -> None:
        if self.job.running:
            try:
                self.job.stop()
                # A source restore includes a colcon rebuild. Let that safety
                # transaction finish instead of abandoning it because the GUI
                # window was closed during a slow build.
                self.job.wait(timeout=180.0)
            except Exception:
                pass


class CalibrationGuiHandler(BaseHTTPRequestHandler):
    server_version = "F1TENTHCalibrationGUI/1.0"

    @property
    def controller(self) -> GuiController:
        return self.server.controller  # type: ignore[attr-defined]

    def log_message(self, format: str, *args: Any) -> None:
        # Keep stage terminal output readable; HTTP access logging adds little
        # value for a localhost-only single-user interface.
        return

    def _send_json(self, value: Any, status: int = HTTPStatus.OK) -> None:
        payload = json.dumps(value, default=str).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def _error(self, exc: Exception) -> None:
        status = HTTPStatus.CONFLICT if isinstance(exc, RuntimeError) else HTTPStatus.BAD_REQUEST
        if isinstance(exc, (FileNotFoundError, PermissionError)):
            status = HTTPStatus.NOT_FOUND
        self._send_json({"ok": False, "error": str(exc), "type": type(exc).__name__}, status)

    def _json_body(self) -> dict[str, Any]:
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError as exc:
            raise ValueError("invalid Content-Length") from exc
        if length > 1_000_000:
            raise ValueError("request body is too large")
        raw = self.rfile.read(length) if length else b"{}"
        value = json.loads(raw.decode("utf-8"))
        if not isinstance(value, dict):
            raise ValueError("request JSON must be an object")
        return value

    def _serve_path(self, path: Path, *, cache: bool = False) -> None:
        if not path.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        payload = path.read_bytes()
        mime = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "public, max-age=60" if cache else "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", "default-src 'self'; style-src 'self'; script-src 'self'; img-src 'self' data:; frame-src 'self'")
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        try:
            parsed = urllib.parse.urlparse(self.path)
            if parsed.path == "/api/health":
                self._send_json({"ok": True, "service": "vehicle-calibration-gui"})
                return
            if parsed.path == "/api/status":
                self._send_json(self.controller.snapshot())
                return
            if parsed.path == "/api/job":
                query = urllib.parse.parse_qs(parsed.query)
                cursor = int(query.get("cursor", ["0"])[0])
                self._send_json(self.controller.job.snapshot(cursor))
                return
            if parsed.path.startswith("/session-file/"):
                path = self.controller.safe_session_file(parsed.path[len("/session-file/"):])
                self._serve_path(path)
                return
            if parsed.path in {"/", "/index.html"}:
                self._serve_path(STATIC_ROOT / "index.html")
                return
            if parsed.path.startswith("/static/"):
                name = parsed.path[len("/static/"):]
                if Path(name).name != name:
                    raise PermissionError("invalid static path")
                self._serve_path(STATIC_ROOT / name)
                return
            self.send_error(HTTPStatus.NOT_FOUND)
        except Exception as exc:
            self._error(exc)

    def do_POST(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        try:
            body = self._json_body()
            if self.path == "/api/session/new":
                session = self.controller.create_session()
                self._send_json({"ok": True, "session": str(session)}, HTTPStatus.CREATED)
            elif self.path == "/api/session/select":
                session = self.controller.select_session(str(body.get("session_id", "")))
                self._send_json({"ok": True, "session": str(session)})
            elif self.path == "/api/metrology":
                validation = self.controller.update_metrology(body)
                self._send_json({"ok": True, "validation": validation})
            elif self.path == "/api/run":
                confirmations = body.get("confirmations", [])
                if not isinstance(confirmations, list):
                    raise ValueError("confirmations must be a list")
                self.controller.run_stage(str(body.get("stage", "")), confirmations)
                self._send_json({"ok": True}, HTTPStatus.ACCEPTED)
            elif self.path == "/api/report":
                self.controller.build_report()
                self._send_json({"ok": True}, HTTPStatus.ACCEPTED)
            elif self.path == "/api/promote":
                self.controller.build_report(promote=True, confirmation=body.get("confirmation"))
                self._send_json({"ok": True}, HTTPStatus.ACCEPTED)
            elif self.path == "/api/redo":
                self.controller.redo(str(body.get("stage", "")), body.get("confirmation"))
                self._send_json({"ok": True}, HTTPStatus.ACCEPTED)
            elif self.path == "/api/recover":
                self.controller.recover(body.get("confirmation"))
                self._send_json({"ok": True}, HTTPStatus.ACCEPTED)
            elif self.path == "/api/job/input":
                self.controller.job.write(str(body.get("response", "")), newline=bool(body.get("newline", True)))
                self._send_json({"ok": True})
            elif self.path == "/api/job/stop":
                self.controller.job.stop()
                self._send_json({"ok": True}, HTTPStatus.ACCEPTED)
            else:
                self.send_error(HTTPStatus.NOT_FOUND)
        except Exception as exc:
            self._error(exc)


class CalibrationGuiServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address: tuple[str, int], controller: GuiController) -> None:
        self.controller = controller
        super().__init__(address, CalibrationGuiHandler)


def serve_gui(
    config_path: Path,
    workspace: Path,
    session: Path | None,
    *,
    host: str = "127.0.0.1",
    port: int = 8765,
    open_browser: bool = True,
) -> None:
    """Serve the operator GUI until interrupted."""
    if host not in {"127.0.0.1", "localhost", "::1"}:
        raise ValueError("the command-capable calibration GUI is restricted to localhost")
    if not STATIC_ROOT.is_dir():
        raise FileNotFoundError(STATIC_ROOT)
    controller = GuiController(config_path, workspace, session)
    server = CalibrationGuiServer((host, int(port)), controller)
    bound_host, bound_port = server.server_address[:2]
    display_host = "127.0.0.1" if bound_host in {"0.0.0.0", "::"} else bound_host
    url = f"http://{display_host}:{bound_port}/"
    print(f"Vehicle calibration GUI: {url}")
    print("Keep this terminal open. Ctrl+C performs a controlled stop if a stage is active.")
    if open_browser:
        threading.Timer(0.4, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        print("\nStopping calibration GUI...")
    finally:
        server.shutdown()
        server.server_close()
        controller.shutdown()
