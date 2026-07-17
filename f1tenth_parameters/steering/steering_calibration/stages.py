"""Operator-guided, raw-servo steering-calibration stages.

No stage uses the existing steering-angle-to-servo calibration to collect data.
The raw-servo selector supplies steering; AckermannToVesc remains only for the
low-speed motor command. Every raw attempt is bagged and receives a final
ACCEPT / REDO / SKIP disposition event.
"""
from __future__ import annotations

import math
import sys
import termios
import time
import tty
from pathlib import Path
from typing import Any

import numpy as np
import rclpy

from .config import dump_json
from .centre_guidance import choose_provisional_centre, fine_grid_targets, fit_onboard_zero
from .runtime import CalibrationNode
from .ui import banner, checklist, note, pause_for_reposition, require_ready, review_trial, warn

WINDOW_FIELDS = ("imu_gz", "imu_ax", "imu_ay", "servo_echo", "servo_selected", "servo_bus", "odom_vx", "odom_wz")


def _single_key() -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


_SENSOR_STREAMS = {"imu", "vesc", "odom", "scan"}
_COMMAND_ECHO_STREAMS = {"servo_echo", "servo_selected", "servo_bus"}


def _start_node(name: str, config: dict[str, Any], required: set[str]) -> CalibrationNode:
    rclpy.init(args=None)
    node = CalibrationNode(name, config)
    try:
        # Free-running sensor streams (imu/vesc/odom/scan) appear as soon as their
        # drivers are up, so wait for those first.
        node.wait_for(required & _SENSOR_STREAMS)
        node.set_steering_mode("raw")
        # The selector output, VESC command bus and servo echo only publish once a
        # raw-servo command has flowed through the chain. Prime it with a neutral
        # hold at the configured seed (speed 0, no motor demand) so those echo
        # streams come alive, then confirm them. Without this the startup
        # deadlocks: wait_for would block on echoes that never appear before the
        # first command is sent.
        echo_required = required & _COMMAND_ECHO_STREAMS
        if echo_required:
            seed = float(config["initial"]["raw_servo_seed"])
            warmup_deadline = time.monotonic() + 3.0
            while time.monotonic() < warmup_deadline:
                node.command(0.0, seed)
                node.spin(0.05)
                if echo_required.issubset(node.latest.seen):
                    break
            node.wait_for(echo_required, timeout_s=5.0)
        node.spin(0.10)
    except Exception:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        raise
    return node


def _finish_node(node: CalibrationNode, centre_raw: float | None) -> None:
    try:
        node.set_steering_mode("raw")
        node.neutral_drive(centre_raw)
        time.sleep(0.15)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _trial_id(condition_id: str, attempt: int) -> str:
    return f"{condition_id}__attempt_{attempt:02d}"


def _disposition(node: CalibrationNode, *, stage: str, condition_id: str, trial_id: str,
                 attempt: int, automatic_ok: bool, automatic_summary: dict[str, Any]) -> str:
    decision = review_trial(label=f"{stage}: {condition_id}, attempt {attempt}",
                            automatic_ok=automatic_ok, automatic_summary=automatic_summary)
    node.event.emit("trial_decision", stage=stage, condition_id=condition_id,
                    trial_id=trial_id, attempt=attempt, decision=decision,
                    accepted=(decision == "accepted"), automatic_ok=automatic_ok)
    return decision


def _capture_after_startup(
    node: CalibrationNode,
    *, speed_mps: float,
    raw_servo: float,
    centre_raw: float,
    phase: str,
    segment_id: str,
    trial_id: str,
    capture_s: float,
    metadata: dict[str, Any],
) -> tuple[dict[str, Any], dict[str, Any] | None]:
    startup = node.establish_speed(speed_mps=speed_mps, raw_servo=raw_servo,
                                   centre_raw_servo=centre_raw, segment_id=segment_id,
                                   trial_id=trial_id)
    if not bool(startup["stable"]):
        return startup, None
    hold_metadata = {
        key: value for key, value in metadata.items()
        if key not in {
            "speed_mps", "raw_servo", "duration_s", "phase", "segment_id",
            "capture", "centre_raw_servo", "begin_window_fields", "trial_id",
        }
    }
    summary = node.hold(speed_mps=speed_mps, raw_servo=raw_servo, duration_s=capture_s,
                        phase=phase, segment_id=segment_id, capture=True,
                        centre_raw_servo=centre_raw, begin_window_fields=WINDOW_FIELDS,
                        trial_id=trial_id, **hold_metadata)
    return startup, summary


def _settle_raw_servo(
    node: CalibrationNode,
    *,
    raw_servo: float,
    centre_raw: float,
    phase: str,
    segment_id: str,
    trial_id: str,
    tolerance: float,
    initial_s: float,
    timeout_s: float,
    final_window_s: float,
    metadata: dict[str, Any],
) -> dict[str, Any]:
    """Wait until the final servo echo window, not the full transient, matches."""
    hz = float(node.cfg["session"]["command_publish_hz"])
    period = 1.0 / hz
    start = time.monotonic()
    deadline = start + max(initial_s, timeout_s)
    initial_deadline = start + initial_s
    samples: list[tuple[float, float]] = []
    node.event.emit("phase_start", phase=phase, segment_id=segment_id,
                    trial_id=trial_id, capture=True, raw_servo_target=raw_servo,
                    **metadata)
    try:
        while time.monotonic() < deadline:
            node.command(0.0, raw_servo)
            node.spin(period)
            now = time.monotonic()
            if math.isfinite(node.latest.servo_echo):
                samples.append((now, float(node.latest.servo_echo)))
            while samples and now - samples[0][0] > final_window_s:
                samples.pop(0)
            if now < initial_deadline or len(samples) < 3:
                continue
            values = np.asarray([value for _, value in samples], dtype=float)
            error = abs(float(np.mean(values)) - raw_servo)
            final_window_max_error = float(np.max(np.abs(values - raw_servo)))
            if error <= tolerance and final_window_max_error <= tolerance:
                summary = {
                    "servo_echo_mean": float(np.mean(values)),
                    "servo_echo_std": float(np.std(values)),
                    "servo_echo_count": int(values.size),
                    "settle_echo_error": error,
                    "settle_echo_max_error": final_window_max_error,
                    "settle_elapsed_s": float(now - start),
                    "settle_within_tolerance": True,
                }
                node.event.emit("phase_end", phase=phase, segment_id=segment_id,
                                trial_id=trial_id, capture=True, **summary, **metadata)
                return summary
    except Exception as exc:
        node.event.emit("safety_abort", phase=phase, segment_id=segment_id,
                        trial_id=trial_id, reason=repr(exc))
        node.neutral_drive(centre_raw)
        raise

    values = np.asarray([value for _, value in samples], dtype=float)
    mean = float(np.mean(values)) if values.size else math.nan
    summary = {
        "servo_echo_mean": mean,
        "servo_echo_std": float(np.std(values)) if values.size else math.nan,
        "servo_echo_count": int(values.size),
        "settle_echo_error": abs(mean - raw_servo) if values.size else math.inf,
        "settle_echo_max_error": float(np.max(np.abs(values - raw_servo))) if values.size else math.inf,
        "settle_elapsed_s": float(time.monotonic() - start),
        "settle_within_tolerance": False,
    }
    node.event.emit("phase_end", phase=phase, segment_id=segment_id,
                    trial_id=trial_id, capture=True, **summary, **metadata)
    return summary


def _motion_loop(
    node: CalibrationNode,
    *, stage: str,
    condition_id: str,
    operator_message: str,
    speed_mps: float,
    raw_servo: float,
    centre_raw: float,
    phase: str,
    capture_s: float,
    metadata: dict[str, Any],
) -> list[dict[str, Any]]:
    """Run a condition with explicit operator acceptance and retry semantics."""
    records: list[dict[str, Any]] = []
    attempt = 1
    while True:
        trial_id = _trial_id(condition_id, attempt)
        pause_for_reposition(operator_message + f"\nAttempt {attempt}. REDO has no retry limit.")
        # metadata may itself carry raw_servo_target/side/etc.; merge into one
        # dict (last value wins) so emit() never gets a duplicated keyword.
        trial_payload = {"stage": stage, "condition_id": condition_id,
                         "trial_id": trial_id, "attempt": attempt,
                         "speed_mps": speed_mps, "raw_servo_target": raw_servo, **metadata}
        node.event.emit("trial_start", **trial_payload)
        startup, summary = _capture_after_startup(
            node, speed_mps=speed_mps, raw_servo=raw_servo, centre_raw=centre_raw,
            phase=phase, segment_id=condition_id, trial_id=trial_id,
            capture_s=capture_s, metadata=metadata,
        )
        node.neutral_drive(centre_raw)
        automatic_ok = bool(startup["stable"]) and summary is not None
        decision = _disposition(node, stage=stage, condition_id=condition_id,
                                trial_id=trial_id, attempt=attempt,
                                automatic_ok=automatic_ok,
                                automatic_summary={"startup": startup, "capture": summary or {}})
        record = {"condition_id": condition_id, "trial_id": trial_id, "attempt": attempt,
                  "raw_servo_target": raw_servo, "startup": startup, "capture": summary,
                  "decision": decision, **metadata}
        records.append(record)
        if decision in {"accepted", "skipped"}:
            return records
        attempt += 1


def raw_command_path_audit(config: dict[str, Any], stage_dir: Path) -> dict[str, Any]:
    """Verify selector ownership and raw-command/echo agreement while stationary."""
    banner("STAGE 0 OF 6 — RAW SERVO COMMAND-PATH AUDIT", "The car remains stationary.")
    checklist([
        "Car is on a stand or driven wheels cannot move.",
        "No MPC, MPCC, planner, teleoperation, or other command publisher is running.",
        "The calibration stack is running; selector mode will be RAW.",
    ])
    require_ready()
    node = _start_node("steering_raw_command_path_audit", config, {"imu", "vesc", "odom", "servo_echo", "servo_selected", "servo_bus", "scan"})
    seed = float(config["initial"]["raw_servo_seed"])
    p = config["preflight"]
    delta = float(p["raw_servo_audit_delta"])
    low_guard = float(config["endstops"]["raw_servo_domain_min"])
    high_guard = float(config["endstops"]["raw_servo_domain_max"])
    values = [("centre_a", seed), ("high", seed + delta), ("centre_b", seed),
              ("low", seed - delta), ("centre_c", seed)]
    samples: list[dict[str, Any]] = []
    try:
        for label, raw in values:
            if not low_guard <= raw <= high_guard:
                raise RuntimeError("raw-servo audit target exceeds numeric raw-servo domain")
            summary = node.hold(speed_mps=0.0, raw_servo=raw, duration_s=0.8,
                                phase="raw_command_path_audit", segment_id=label,
                                capture=True, centre_raw_servo=seed,
                                begin_window_fields=("servo_echo", "servo_selected", "servo_bus"),
                                trial_id=f"raw_audit_{label}", raw_servo_target=raw)
            samples.append({"label": label, "raw_servo_target": raw, **summary})
        high = next(x for x in samples if x["label"] == "high")
        low = next(x for x in samples if x["label"] == "low")
        max_echo_error = max(abs(x["servo_echo_mean"] - x["raw_servo_target"]) for x in samples)
        max_selected_error = max(abs(x["servo_selected_mean"] - x["raw_servo_target"]) for x in samples)
        max_bus_error = max(abs(x["servo_bus_mean"] - x["raw_servo_target"]) for x in samples)
        max_selected_bus_error = max(abs(x["servo_selected_mean"] - x["servo_bus_mean"]) for x in samples)
        span = abs(high["servo_echo_mean"] - low["servo_echo_mean"])
        if max_echo_error > float(p["raw_servo_echo_tolerance"]):
            raise RuntimeError(f"raw selector/echo error {max_echo_error:.5f} exceeds tolerance")
        if max_selected_error > float(p["raw_servo_selected_tolerance"]):
            raise RuntimeError(f"raw selector output error {max_selected_error:.5f} exceeds tolerance")
        if max_bus_error > float(p["raw_servo_bus_tolerance"]):
            raise RuntimeError(f"actual servo bus error {max_bus_error:.5f} exceeds tolerance")
        if max_selected_bus_error > float(p["raw_servo_bus_tolerance"]):
            raise RuntimeError(f"selector/servo-bus discrepancy {max_selected_bus_error:.5f} exceeds tolerance")
        if span < float(p["min_echo_span_servo"]):
            raise RuntimeError("raw servo source did not produce a measurable echo span")
        max_command_path_error = max(
            max_echo_error,
            max_selected_error,
            max_bus_error,
            max_selected_bus_error,
        )
        result = {"status": "pass", "raw_servo_seed": seed,
                  "max_echo_error_servo": max_echo_error,
                  "max_selected_error_servo": max_selected_error,
                  "max_bus_error_servo": max_bus_error,
                  "max_selected_bus_error_servo": max_selected_bus_error,
                  "max_command_path_error_servo": max_command_path_error,
                  "echo_span_servo": span, "samples": samples}
        dump_json(stage_dir / "runtime_result.json", result)
        print(f"\nPASS — maximum command-chain error: {max_command_path_error:.5f} servo units")
        return result
    finally:
        _finish_node(node, seed)


def imu_bias_ground(config: dict[str, Any], stage_dir: Path, centre: dict[str, Any]) -> dict[str, Any]:
    """Stationary on-ground IMU-bias capture with the true resting attitude.

    The gyro-z bias and the ay/ax gravity-projection offsets must be measured in
    the attitude the car actually drives in. This is therefore captured on the
    ground (never on a stand), and provides the early epoch the bias-drift model
    pairs with the on-ground Stage 3 baseline.
    """
    banner("STAGE 1b — ON-GROUND IMU BIAS", "Stationary IMU characterisation on the ground.")
    checklist([
        "Car is on the ground on a flat, level surface (NOT on a stand).",
        "Wheels are physically straight and nothing leans on or touches the car.",
        "No MPC, MPCC, planner, teleoperation, or other command publisher is running.",
    ])
    require_ready()
    node = _start_node("steering_imu_bias_ground", config, {"imu", "vesc", "odom"})
    c = float(centre["centre_servo_raw"])
    duration = float(config["imu_bias"]["stationary_s"])
    try:
        pause_for_reposition("ON-GROUND IMU BIAS CAPTURE\nDo not touch the car during recording.")
        node.event.emit("trial_start", stage="imu_bias_ground", condition_id="imu_bias_stationary",
                        trial_id="imu_bias_stationary", attempt=1, raw_servo_target=c)
        summary = node.hold(speed_mps=0.0, raw_servo=c, duration_s=duration,
                            phase="imu_bias_stationary", segment_id="imu_bias_stationary",
                            capture=True, centre_raw_servo=c, begin_window_fields=WINDOW_FIELDS,
                            trial_id="imu_bias_stationary")
        result = {"status": "pass", "centre_servo_raw": c, "stationary_s": duration,
                  "capture": summary}
        dump_json(stage_dir / "runtime_result.json", result)
        print(f"\nPASS — on-ground stationary IMU capture ({duration:.0f}s)")
        return result
    finally:
        _finish_node(node, c)


def _accepted_yaw(records: list[dict[str, Any]]) -> tuple[float, list[float]]:
    yaw = [float(r["capture"]["imu_gz_mean"]) for r in records
           if r.get("decision") == "accepted" and r.get("capture") is not None]
    if not yaw:
        raise RuntimeError("no accepted yaw captures for centre-search candidate")
    return float(np.median(yaw)), yaw


def _find_sign_bracket(samples: list[dict[str, Any]]) -> tuple[dict[str, Any], dict[str, Any]] | None:
    ordered = sorted(samples, key=lambda x: x["raw_servo"])
    for left, right in zip(ordered[:-1], ordered[1:]):
        yl, yr = float(left["yaw_median"]), float(right["yaw_median"])
        if yl == 0.0:
            return left, left
        if yr == 0.0:
            return right, right
        if yl * yr < 0.0:
            return left, right
    return None


def _centre_fixed_lidar_grid(config: dict[str, Any], stage_dir: Path) -> dict[str, Any]:
    """Collect an on-board-seeded grid without letting a shortcut certify centre.

    This is the canonical unified-campaign acquisition path.  It deliberately
    performs only A (data collection): B is the LiDAR-primary offline fit and C
    is the separate physical straightness validation after the resulting patch
    has been rebuilt.  IMU and odometry are deliberately retained as useful
    coarse estimates, but neither can end the search at an approximate offset.
    """
    banner("STAGE 1 OF 6 — ZERO-CURVATURE CENTRE DATA COLLECTION",
           "On-board-seeded raw-servo grid; LiDAR fine fit and physical validation follow.")
    checklist([
        "Clear the 12 m straight lane and keep fixed LiDAR features visible.",
        "No heading-hold/PID is active: raw servo is intentionally fixed for this test.",
        "IMU and odometry provide a coarse estimate; LiDAR and new validation decide the final centre.",
        "Reposition before every pass; ACCEPT only clean, usable captures.",
    ])
    require_ready()
    node = _start_node("steering_zero_curvature_centre", config,
                       {"imu", "vesc", "odom", "servo_echo", "servo_selected", "servo_bus", "scan"})
    p = config["centre_trim"]
    seed = float(config["initial"]["raw_servo_seed"])
    low_guard = float(config["endstops"]["raw_servo_domain_min"])
    high_guard = float(config["endstops"]["raw_servo_domain_max"])
    probe_offsets = [float(value) for value in p.get("onboard_probe_offsets_servo", [-0.03, 0.0, 0.03])]
    offsets = [float(value) for value in p.get("fine_grid_offsets_servo", p.get("training_offsets_servo", []))]
    repetitions = int(p["candidate_repetitions"])
    if len(set(round(value, 8) for value in offsets)) < 4:
        raise RuntimeError("guided centre fine grid needs at least four distinct offsets")
    if len(set(round(value, 8) for value in probe_offsets)) < 3:
        raise RuntimeError("on-board centre probe needs at least three distinct offsets")
    if repetitions < 1:
        raise RuntimeError("centre candidate_repetitions must be positive")
    records: list[dict[str, Any]] = []
    targets: list[float] = []
    probe_records: list[dict[str, Any]] = []
    try:
        # The short symmetric probe is intentionally directional: the fitted
        # IMU/odom yaw slope tells us whether a larger or smaller raw command is
        # needed before collecting the LiDAR fine grid.  It is never a final
        # calibration result.
        for index, offset in enumerate(probe_offsets, start=1):
            raw = seed + offset
            if not low_guard <= raw <= high_guard:
                raise RuntimeError(
                    f"on-board probe point {raw:.5f} is outside raw-servo domain "
                    f"[{low_guard:.5f}, {high_guard:.5f}]"
                )
            condition = f"centre_onboard_probe_{index:02d}"
            captured = _motion_loop(
                node,
                stage="zero_curvature_centre",
                condition_id=condition,
                operator_message=(
                    f"ON-BOARD DIRECTION PROBE {index}/{len(probe_offsets)}\n"
                    f"Raw servo target: {raw:.5f} (deployed seed {seed:.5f} + {offset:+.5f}).\n"
                    "This short pass tells the suite which direction to move the later LiDAR grid. "
                    "It cannot approve a centre by itself."
                ),
                speed_mps=float(p["speed_mps"]), raw_servo=raw, centre_raw=seed,
                phase="centre_trim_capture", capture_s=float(p.get("onboard_probe_capture_s", p["capture_s"])),
                metadata={
                    "raw_servo_target": raw,
                    "capture_role": "onboard_direction_probe",
                    "probe_offset_servo": offset,
                    "probe_index": index,
                    "acquisition_mode": "onboard_guided_lidar_grid",
                },
            )
            records.extend(captured)
            probe_records.extend(captured)

        def values_from_probe(field: str) -> tuple[list[float], list[float]]:
            raw_values: list[float] = []
            yaw_values: list[float] = []
            for record in probe_records:
                capture = record.get("capture") or {}
                if record.get("decision") != "accepted" or not isinstance(capture, dict):
                    continue
                raw_value = capture.get("servo_echo_mean", record.get("raw_servo_target"))
                yaw_value = capture.get(field)
                try:
                    raw_numeric, yaw_numeric = float(raw_value), float(yaw_value)
                except (TypeError, ValueError):
                    continue
                if math.isfinite(raw_numeric) and math.isfinite(yaw_numeric):
                    raw_values.append(raw_numeric)
                    yaw_values.append(yaw_numeric)
            return raw_values, yaw_values

        imu_raw, imu_yaw = values_from_probe("imu_gz_mean")
        odom_raw, odom_yaw = values_from_probe("odom_wz_mean")
        imu_probe = fit_onboard_zero(imu_raw, imu_yaw, p)
        odom_probe = fit_onboard_zero(odom_raw, odom_yaw, p)
        guidance = choose_provisional_centre(seed, imu_probe, odom_probe, p)
        fine_seed = float(guidance["centre_servo_raw"])
        if guidance.get("warning"):
            warn(str(guidance["warning"]))
        note(
            f"On-board directional estimate: {fine_seed:.5f} from {guidance['source']} "
            f"({guidance['shift_from_seed_servo']:+.5f} from deployed seed). "
            "Collecting the LiDAR fine grid around it now."
        )

        fine_targets = fine_grid_targets(seed, fine_seed, offsets, low_guard, high_guard)
        targets.extend(fine_targets)
        for index, raw in enumerate(fine_targets, start=1):
            for rep in range(1, repetitions + 1):
                condition = f"centre_lidar_grid_{index:02d}_rep_{rep:02d}"
                records.extend(_motion_loop(
                    node,
                    stage="zero_curvature_centre",
                    condition_id=condition,
                    operator_message=(
                        f"LIDAR FINE-GRID POINT {index}/{len(fine_targets)}, repetition {rep}/{repetitions}\n"
                        f"Raw servo target: {raw:.5f} (guided centre {fine_seed:.5f}, offset {raw - fine_seed:+.5f}).\n"
                        "This pass gathers LiDAR, IMU, and odometry data. Do not accept a centre from one sensor alone."
                    ),
                    speed_mps=float(p["speed_mps"]),
                    raw_servo=raw, centre_raw=fine_seed,
                    phase="centre_trim_capture",
                    capture_s=float(p["capture_s"]),
                    metadata={
                        "raw_servo_target": raw,
                        "capture_role": "lidar_fine_grid",
                        "grid_centre_servo": fine_seed,
                        "grid_offset_servo": raw - fine_seed,
                        "grid_index": index,
                        "repetition": rep,
                        "acquisition_mode": "onboard_guided_lidar_grid",
                    },
                ))
        accepted = sum(1 for record in records if record.get("decision") == "accepted")
        result = {
            "status": "captured",
            "acquisition_mode": "onboard_guided_lidar_grid",
            "seed_servo_raw": seed,
            "onboard_probe": {"imu": imu_probe, "odom": odom_probe, "guidance": guidance},
            "provisional_grid_centre_servo_raw": fine_seed,
            "raw_servo_targets": targets,
            "candidate_repetitions": repetitions,
            "probe_requested_trials": len(probe_offsets),
            "fine_grid_requested_trials": len(fine_targets) * repetitions,
            "requested_trials": len(probe_offsets) + len(fine_targets) * repetitions,
            "accepted_trials": accepted,
            "records": records,
            "note": (
                "The on-board probe only directed the LiDAR grid. The next offline stage still fits LiDAR ICP "
                "yaw versus raw servo, then independent physical validation decides whether the value is usable."
            ),
        }
        dump_json(stage_dir / "runtime_result.json", result)
        return result
    finally:
        _finish_node(node, fine_seed if 'fine_seed' in locals() else seed)


def zero_curvature_centre(config: dict[str, Any], stage_dir: Path) -> dict[str, Any]:
    """Adaptive raw-servo search for the yaw-rate zero crossing.

    The runner first brackets a sign change in mean IMU yaw rate, then applies
    safeguarded secant/bisection steps. Each candidate has repeated accepted
    runs. All values are raw servo values; no old steering-angle map is used.
    """
    if str(config["centre_trim"].get("acquisition_mode", "adaptive_imu")).lower() == "fixed_lidar_grid":
        return _centre_fixed_lidar_grid(config, stage_dir)

    banner("STAGE 1 OF 6 — ZERO-CURVATURE CENTRE TRIM", "Adaptive repeated low-speed straight passes.")
    checklist([
        "Clear straight test lane of at least 6 m.",
        "Flat, dry surface and fixed LiDAR features.",
        "Physical emergency stop is available.",
    ])
    require_ready()
    node = _start_node("steering_zero_curvature_centre", config, {"imu", "vesc", "odom", "servo_echo", "servo_selected", "servo_bus", "scan"})
    p = config["centre_trim"]
    seed = float(config["initial"]["raw_servo_seed"])
    low_guard = float(config["endstops"]["raw_servo_domain_min"])
    high_guard = float(config["endstops"]["raw_servo_domain_max"])
    all_records: list[dict[str, Any]] = []
    measured: list[dict[str, Any]] = []

    def measure(raw: float, label: str, repetitions: int) -> dict[str, Any]:
        raw = float(raw)
        if not np.isfinite(raw) or raw < low_guard or raw > high_guard:
            raise RuntimeError(
                f"centre-search candidate {raw!r} is outside the raw-servo domain "
                f"[{low_guard:.5f}, {high_guard:.5f}]; refusing silent clipping"
            )
        candidate_records: list[dict[str, Any]] = []
        for rep in range(1, repetitions + 1):
            condition = f"centre_{label}_rep_{rep:02d}"
            candidate_records.extend(_motion_loop(
                node, stage="zero_curvature_centre", condition_id=condition,
                operator_message=(f"CENTRE SEARCH — {label}, repetition {rep}/{repetitions}\n"
                                  f"Raw servo target: {raw:.5f}. The car will collect a stable straight pass."),
                speed_mps=float(p["speed_mps"]), raw_servo=raw, centre_raw=seed,
                phase="centre_trim_capture", capture_s=float(p["capture_s"]),
                metadata={"raw_servo_target": raw, "candidate_label": label, "repetition": rep},
            ))
        yaw_median, yaw_values = _accepted_yaw(candidate_records)
        item = {"raw_servo": raw, "label": label, "yaw_median": yaw_median,
                "yaw_values": yaw_values, "records": candidate_records}
        all_records.extend(candidate_records)
        measured.append(item)
        node.event.emit("centre_candidate_estimate", raw_servo_target=raw, label=label,
                        yaw_median_rad_s=yaw_median, accepted_repetitions=len(yaw_values))
        return item

    try:
        half = float(p["initial_half_width_servo"])
        # Coarse, symmetric bracketing around a seed. The seed is not assumed correct.
        measure(seed - half, "bracket_initial_low", int(p["candidate_repetitions"]))
        measure(seed, "bracket_seed", int(p["candidate_repetitions"]))
        measure(seed + half, "bracket_initial_high", int(p["candidate_repetitions"]))
        bracket = _find_sign_bracket(measured)
        for expansion in range(1, int(p["max_bracket_expansions"]) + 1):
            if bracket is not None:
                break
            width = half + expansion * float(p["expansion_step_servo"])
            measure(seed - width, f"bracket_expand_{expansion}_low", int(p["candidate_repetitions"]))
            measure(seed + width, f"bracket_expand_{expansion}_high", int(p["candidate_repetitions"]))
            bracket = _find_sign_bracket(measured)
        if bracket is None:
            raise RuntimeError("centre search could not bracket a yaw-rate sign change; inspect raw data and sign routing")

        candidate_yaw_limit = float(p["max_abs_yaw_rate_for_candidate_rad_s"])
        servo_search_tolerance = float(p["search_tolerance_servo"])
        left, right = bracket
        refinement_converged = False
        best_candidate = min(measured, key=lambda item: abs(float(item["yaw_median"])))
        if left is right:
            centre_raw = float(left["raw_servo"])
            refinement_converged = abs(float(left["yaw_median"])) <= candidate_yaw_limit
        else:
            for iteration in range(1, int(p["max_refinement_iterations"]) + 1):
                x0, y0 = float(left["raw_servo"]), float(left["yaw_median"])
                x1, y1 = float(right["raw_servo"]), float(right["yaw_median"])
                denominator = y1 - y0
                secant = x0 - y0 * (x1 - x0) / denominator if abs(denominator) > 1e-12 else 0.5 * (x0 + x1)
                # Safeguard secant against noisy values that place the new point
                # too close to a bracket end: fall back to bisection.
                lo, hi = min(x0, x1), max(x0, x1)
                margin = 0.20 * (hi - lo)
                candidate = 0.5 * (lo + hi) if not (lo + margin <= secant <= hi - margin) else secant
                item = measure(candidate, f"refine_{iteration:02d}", int(p["candidate_repetitions"]))
                y = float(item["yaw_median"])
                if abs(y) < abs(float(best_candidate["yaw_median"])):
                    best_candidate = item
                if y0 * y <= 0.0:
                    right = item
                else:
                    left = item
                bracket_width = abs(float(right["raw_servo"]) - float(left["raw_servo"]))
                candidate_gate_passed = abs(float(best_candidate["yaw_median"])) <= candidate_yaw_limit
                node.event.emit(
                    "centre_refinement",
                    iteration=iteration,
                    raw_servo_target=candidate,
                    bracket_low=min(float(left["raw_servo"]), float(right["raw_servo"])),
                    bracket_high=max(float(left["raw_servo"]), float(right["raw_servo"])),
                    yaw_median_rad_s=y,
                    best_candidate_raw_servo=float(best_candidate["raw_servo"]),
                    best_candidate_abs_yaw_rad_s=abs(float(best_candidate["yaw_median"])),
                    candidate_yaw_gate_limit_rad_s=candidate_yaw_limit,
                    candidate_yaw_gate_passed=candidate_gate_passed,
                    bracket_width_servo=bracket_width,
                    bracket_width_gate_passed=bracket_width <= servo_search_tolerance,
                )
                if candidate_gate_passed and bracket_width <= servo_search_tolerance:
                    refinement_converged = True
                    break
            # This is a real quality gate. A sign bracket alone is not enough:
            # the search must resolve the bracket and include an observed,
            # low-yaw candidate before the calculated zero crossing is trusted.
            final_width = abs(float(right["raw_servo"]) - float(left["raw_servo"]))
            if not refinement_converged:
                failures = []
                if final_width > servo_search_tolerance:
                    failures.append(f"bracket width {final_width:.6f} > {servo_search_tolerance:.6f}")
                best_abs_yaw = abs(float(best_candidate["yaw_median"]))
                if best_abs_yaw > candidate_yaw_limit:
                    failures.append(f"best observed |yaw rate| {best_abs_yaw:.5f} > {candidate_yaw_limit:.5f} rad/s")
                raise RuntimeError("centre-search refinement gate failed: " + "; ".join(failures))
            # Fit all local accepted candidates around the final bracket to avoid
            # throwing away useful repeated observations.
            local = [m for m in measured if min(left["raw_servo"], right["raw_servo"]) - 0.02 <= m["raw_servo"] <= max(left["raw_servo"], right["raw_servo"]) + 0.02]
            x = np.asarray([m["raw_servo"] for m in local], dtype=float)
            y = np.asarray([m["yaw_median"] for m in local], dtype=float)
            slope, intercept = np.polyfit(x, y, 1)
            if abs(slope) < 1e-8:
                raise RuntimeError("centre local yaw-vs-servo slope is too small")
            centre_raw = float(-intercept / slope)
            local_predicted = slope * x + intercept
            local_residual = y - local_predicted
            ss_total = float(np.sum((y - np.mean(y)) ** 2))
            ss_residual = float(np.sum(local_residual ** 2))
            local_r2 = 1.0 if ss_total <= 1e-15 and ss_residual <= 1e-15 else (1.0 - ss_residual / ss_total if ss_total > 0.0 else float("-inf"))
            local_rmse = float(np.sqrt(np.mean(local_residual ** 2)))
            bracket_low = min(float(left["raw_servo"]), float(right["raw_servo"]))
            bracket_high = max(float(left["raw_servo"]), float(right["raw_servo"]))
            max_extrapolation = float(p.get("max_fit_extrapolation_servo", 0.0))
            extrapolation = max(bracket_low - centre_raw, centre_raw - bracket_high, 0.0)
            if extrapolation > max_extrapolation:
                raise RuntimeError(
                    f"fitted centre {centre_raw:.6f} extrapolates {extrapolation:.6f} "
                    f"outside final observed bracket; refusing approximate offset"
                )
            if abs(float(np.max(x) - np.min(x))) < float(p.get("min_training_span_servo", 0.0)):
                raise RuntimeError("centre local fit does not span the configured servo interval")
            expected_sign = int(p.get("expected_yaw_rate_slope_sign", -1))
            if expected_sign and np.sign(slope) != np.sign(expected_sign):
                raise RuntimeError(f"centre yaw-vs-servo slope sign {slope:.6f} is unexpected")
            if local_r2 < float(p.get("min_fit_r2", 0.0)):
                raise RuntimeError(f"centre local fit R² {local_r2:.4f} is below the configured gate")
            if local_rmse > float(p.get("max_fit_rmse_rad_s", float("inf"))):
                raise RuntimeError(f"centre local fit RMSE {local_rmse:.6f} exceeds the configured gate")
        if not np.isfinite(centre_raw) or centre_raw < low_guard or centre_raw > high_guard:
            raise RuntimeError(
                f"centre candidate {centre_raw!r} is outside the raw-servo domain; "
                "refusing to clip an invalid calibration"
            )

        confirmations: list[dict[str, Any]] = []
        for rep in range(1, int(p["confirmation_repetitions"]) + 1):
            confirmations.extend(_motion_loop(
                node, stage="zero_curvature_centre", condition_id=f"centre_confirmation_{rep:02d}",
                operator_message=(f"CENTRE CONFIRMATION {rep}/{p['confirmation_repetitions']}\n"
                                  f"Raw servo target: {centre_raw:.5f}."),
                speed_mps=float(p["speed_mps"]), raw_servo=centre_raw, centre_raw=centre_raw,
                phase="centre_confirm_capture", capture_s=float(p["capture_s"]),
                metadata={"raw_servo_target": centre_raw, "repetition": rep},
            ))
        confirm_yaw = [float(r["capture"]["imu_gz_mean"]) for r in confirmations
                       if r.get("decision") == "accepted" and r.get("capture")]
        if len(confirm_yaw) < 3:
            raise RuntimeError("fewer than three accepted centre confirmations")
        result = {
            "status": "pass", "centre_servo_raw": centre_raw,
            "seed_servo_raw": seed, "candidate_records": all_records,
            "candidate_estimates": measured, "confirmation_records": confirmations,
            "accepted_confirmation_trials": len(confirm_yaw),
            "best_candidate_raw_servo": float(best_candidate["raw_servo"]),
            "best_candidate_abs_yaw_rad_s": abs(float(best_candidate["yaw_median"])),
            "candidate_yaw_gate_limit_rad_s": candidate_yaw_limit,
            "confirmation_abs_yaw_mean_rad_s": float(np.mean(np.abs(confirm_yaw)),),
            "confirmation_yaw_std_rad_s": float(np.std(confirm_yaw)),
        }
        confirmation_limit = float(p["max_abs_yaw_rate_for_confirmation_rad_s"])
        if result["confirmation_abs_yaw_mean_rad_s"] > confirmation_limit:
            result["status"] = "fail"
            dump_json(stage_dir / "runtime_result.json", result)
            raise RuntimeError(
                "centre confirmation gate failed: mean |yaw rate| "
                f"{result['confirmation_abs_yaw_mean_rad_s']:.5f} > {confirmation_limit:.5f} rad/s"
            )
        dump_json(stage_dir / "runtime_result.json", result)
        print(f"\nProvisional raw-servo centre: {centre_raw:.5f}")
        return result
    finally:
        _finish_node(node, seed)


def zero_curvature_validation(config: dict[str, Any], stage_dir: Path,
                              centre: dict[str, Any]) -> dict[str, Any]:
    """Run an independent physical straightness check at the fitted centre.

    The centre-search runtime gate is intentionally only a startup/data-quality
    gate.  It cannot tell whether the car visibly drifts in the room, and the
    old campaign accepted a numerically neat fit despite that failure.  This
    stage therefore repeats the centre command after the candidate has been
    applied, records LiDAR motion, and puts the operator's view of the car on
    the same acceptance path as the offline checks.
    """
    banner("STAGE 1a — ZERO-CURVATURE PHYSICAL VALIDATION",
           "Independent straight passes after applying the fitted centre.")
    checklist([
        "Use the 12 m straight lane with fixed LiDAR features.",
        "Watch the vehicle itself, not only the screen or odometry.",
        "ACCEPT only if the car physically tracks straight; REDO if it visibly drifts left or right.",
        "Keep the emergency stop ready and reposition the car between passes.",
    ])
    require_ready()
    node = _start_node("steering_zero_curvature_validation", config,
                       {"imu", "vesc", "odom", "servo_echo", "servo_selected", "servo_bus", "scan"})
    p = config["centre_trim"]
    c = float(centre["centre_servo_raw"])
    capture_s = float(p.get("validation_capture_s", p["capture_s"]))
    conditions = list(p.get("validation_conditions", []))
    if not conditions:
        conditions = [{
            "lane_direction": "outbound",
            "speed_mps": float(p.get("validation_speed_mps", p["speed_mps"])),
            "repetitions": int(p.get("validation_repetitions", 4)),
        }]
    requested = sum(int(condition.get("repetitions", 1)) for condition in conditions)
    if requested < 1:
        raise RuntimeError("centre validation requires at least one configured pass")
    records: list[dict[str, Any]] = []
    try:
        pass_index = 0
        for condition_index, condition in enumerate(conditions, start=1):
            speed = float(condition["speed_mps"])
            direction = str(condition.get("lane_direction", f"condition_{condition_index}"))
            repetitions = int(condition.get("repetitions", 1))
            for rep in range(1, repetitions + 1):
                pass_index += 1
                condition_id = f"centre_validation_{direction}_{speed:.2f}_rep_{rep:02d}"
                records.extend(_motion_loop(
                    node, stage="zero_curvature_validation",
                    condition_id=condition_id,
                    operator_message=(
                        f"VALIDATION PASS {pass_index}/{requested}: {direction.upper()}, {speed:.2f} m/s\n"
                        f"Raw servo target: {c:.5f}. Turn the vehicle to face the {direction} lane direction.\n"
                        "WATCH THE CAR: ACCEPT only when it physically tracks straight. "
                        "REDO any visible left/right drift, even if the automatic summary says OK."
                    ),
                    speed_mps=speed, raw_servo=c, centre_raw=c,
                    phase="centre_validation_capture", capture_s=capture_s,
                    metadata={
                        "raw_servo_target": c,
                        "repetition": rep,
                        "validation_condition_index": condition_index,
                        "validation_lane_direction": direction,
                        "validation_speed_mps": speed,
                    },
                ))
        accepted = sum(1 for record in records if record.get("decision") == "accepted")
        result = {
            "status": "captured",
            "centre_servo_raw": c,
            "capture_s": capture_s,
            "validation_conditions": conditions,
            "requested_repetitions": requested,
            "accepted_repetitions": accepted,
            "records": records,
            "note": "Offline validation must confirm LiDAR straightness and operator physical observation.",
        }
        dump_json(stage_dir / "runtime_result.json", result)
        return result
    finally:
        _finish_node(node, c)


def _confirm_endstop(value: float, side: str) -> bool:
    while True:
        answer = input(
            f"Recorded {side} last-free raw servo = {value:.5f}. "
            "Type CONFIRM to accept or REDO to search this side again: "
        ).strip().upper()
        if answer in {"CONFIRM", "C"}:
            return True
        if answer in {"REDO", "R"}:
            return False
        if answer in {"ABORT", "QUIT", "Q"}:
            raise KeyboardInterrupt("operator aborted end-stop survey")
        print("Type CONFIRM, REDO, or ABORT.")


def _record_observed_wheel_angle(side: str, required: bool) -> float | None:
    """Record the human-measured road-wheel angle at a confirmed free limit."""
    while True:
        answer = input(
            f"Measure the signed road-wheel angle at {side} (degrees; left/right must use one convention). "
            "Enter a number, REDO to repeat this end-stop side, or ABORT: "
        ).strip().upper()
        if answer in {"ABORT", "QUIT", "Q"}:
            raise KeyboardInterrupt("operator aborted human steering-angle survey")
        if answer in {"REDO", "R"}:
            return None
        if answer in {"SKIP", "S"} and not required:
            return float("nan")
        try:
            value = float(answer)
        except ValueError:
            print("Enter a signed finite angle in degrees, REDO, or ABORT.")
            continue
        if math.isfinite(value):
            return value
        print("The observed wheel angle must be finite.")


def physical_endstops(config: dict[str, Any], stage_dir: Path, centre: dict[str, Any]) -> dict[str, Any]:
    banner("STAGE 2 OF 6 — HUMAN STEERING-LIMIT / ANGLE SURVEY",
           "Operator-confirmed raw limits plus measured physical wheel angles.")
    checklist([
        "Car is on a stand; driven wheels cannot move.",
        "Inspect linkage and servo continuously.",
        "Do not deliberately force a mechanical stop or servo buzz.",
    ])
    require_ready()
    node = _start_node("steering_physical_endstops", config, {"vesc", "servo_echo", "servo_selected", "servo_bus"})
    c = float(centre["centre_servo_raw"])
    p = config["endstops"]
    low_guard, high_guard = float(p["raw_servo_domain_min"]), float(p["raw_servo_domain_max"])
    result_raw: dict[str, float] = {}
    observed_angles_deg: dict[str, float] = {}
    require_angles = bool(p.get("require_observed_wheel_angles", False))
    try:
        banner("END-STOP CONTROLS")
        note("a/d: coarse -/+   z/c: fine -/+   x/v: ultra-fine -/+\n"
             "l: record last free position   r: return centre   q: abort")
        for side, direction in (("high_raw", +1.0), ("low_raw", -1.0)):
            while side not in result_raw:
                current = c
                node.raw_servo(current)
                node.event.emit("endstop_survey_start", side=side, centre_servo=c)
                note(f"\n{side.upper()} — begin at raw centre {c:.5f}. Move only outward.")
                while True:
                    node.raw_servo(current)
                    node.spin(0.03)
                    print(f"\rraw servo={current:.5f}   [a/d z/c x/v l r q] ", end="", flush=True)
                    key = _single_key()
                    if key == "q":
                        raise KeyboardInterrupt("operator aborted end-stop survey")
                    if key == "l":
                        print()
                        node.event.emit("endstop_last_free_candidate", side=side, raw_servo=current)
                        if _confirm_endstop(current, side):
                            angle_deg = _record_observed_wheel_angle(side, require_angles)
                            if angle_deg is None:
                                current = c
                                continue
                            result_raw[side] = current
                            if math.isfinite(angle_deg):
                                observed_angles_deg[side] = angle_deg
                                node.event.emit("endstop_observed_wheel_angle", side=side, raw_servo=current,
                                                observed_wheel_angle_deg=angle_deg)
                            node.event.emit("endstop_last_free_confirmed", side=side, raw_servo=current)
                            break
                        current = c
                        continue
                    if key == "r":
                        current = c
                    elif key in {"a", "d", "z", "c", "x", "v"}:
                        mag = {"a": p["coarse_step_servo"], "d": p["coarse_step_servo"],
                               "z": p["fine_step_servo"], "c": p["fine_step_servo"],
                               "x": p["ultra_fine_step_servo"], "v": p["ultra_fine_step_servo"]}[key]
                        sign = -1.0 if key in {"a", "z", "x"} else +1.0
                        candidate = current + sign * float(mag)
                        # Prevent moving toward the wrong side or outside hard guard.
                        if direction * (candidate - c) < 0.0:
                            continue
                        current = float(np.clip(candidate, low_guard, high_guard))
        safe_low = float(result_raw["low_raw"] + float(p["safety_margin_servo"]))
        safe_high = float(result_raw["high_raw"] - float(p["safety_margin_servo"]))
        if not safe_low < c < safe_high:
            raise RuntimeError("operator-confirmed end-stops do not enclose centre after safety margin")
        low_angle = observed_angles_deg.get("low_raw", float("nan"))
        high_angle = observed_angles_deg.get("high_raw", float("nan"))
        min_abs_angle = float(p.get("min_abs_observed_wheel_angle_deg", 0.0))
        if require_angles and (not math.isfinite(low_angle) or not math.isfinite(high_angle)):
            raise RuntimeError("human steering-limit survey requires observed wheel angles on both sides")
        if math.isfinite(low_angle) and math.isfinite(high_angle):
            if low_angle * high_angle >= 0.0:
                raise RuntimeError("human-measured end-stop angles must lie on opposite sides of straight ahead")
            if min(abs(low_angle), abs(high_angle)) < min_abs_angle:
                raise RuntimeError(
                    f"human-measured end-stop angle is below {min_abs_angle:.2f} deg; inspect measurement convention"
                )
        result = {"status": "confirmed", "centre_servo_raw": c,
                  "raw_low_last_free": result_raw["low_raw"], "raw_high_last_free": result_raw["high_raw"],
                  "raw_low_safe": safe_low, "raw_high_safe": safe_high,
                  "safety_margin_servo": float(p["safety_margin_servo"]),
                  "observed_low_wheel_angle_deg": low_angle if math.isfinite(low_angle) else None,
                  "observed_high_wheel_angle_deg": high_angle if math.isfinite(high_angle) else None,
                  "observed_low_wheel_angle_rad": math.radians(low_angle) if math.isfinite(low_angle) else None,
                  "observed_high_wheel_angle_rad": math.radians(high_angle) if math.isfinite(high_angle) else None,
                  "human_angle_measurement_required": require_angles}
        dump_json(stage_dir / "runtime_result.json", result)
        print("\nPhysical limits confirmed:")
        print(f"  low raw safe:  {safe_low:.5f}\n  centre raw:    {c:.5f}\n  high raw safe: {safe_high:.5f}")
        return result
    finally:
        _finish_node(node, c)


def _raw_targets(limits: dict[str, Any], fractions: list[float]) -> list[tuple[str, float, float]]:
    c = float(limits["centre_servo_raw"])
    out: list[tuple[str, float, float]] = []
    for side, endpoint in (("high_raw", float(limits["raw_high_safe"])),
                           ("low_raw", float(limits["raw_low_safe"]))):
        for fraction in fractions:
            out.append((side, float(fraction), float(c + float(fraction) * (endpoint - c))))
    return out


def _steering_capture_plan(
    config: dict[str, Any], limits: dict[str, Any], *, side: str,
    fraction: float, speed_mps: float, minimum_capture_s: float,
    centre_before_s: float = 0.0, return_after_s: float = 0.0,
    raw_servo: float | None = None, radius_override_m: float | None = None,
) -> dict[str, Any]:
    """Use a full circle when the measured end-stop geometry permits it.

    The human wheel-angle survey is only a room-planning estimate here; LiDAR
    remains the calibration reference.  If a shallow setting cannot complete a
    circle inside the target envelope, a numerical bounded-arc plan uses as
    much of the diagonal as possible without pretending the circle fits.
    """
    policy = config.get("room_capture_policy", {})
    observed_key = (
        "observed_high_wheel_angle_rad" if side == "high_raw"
        else "observed_low_wheel_angle_rad"
    )
    try:
        endpoint_angle = abs(float(limits.get(observed_key)))
        wheelbase = float(config["hardware"]["wheelbase_m"])
    except (KeyError, TypeError, ValueError):
        endpoint_angle = math.nan
        wheelbase = math.nan
    angle = endpoint_angle * abs(float(fraction))
    planning_source = "human end-stop wheel angle scaled by raw safe-span fraction"
    applied = config.get("applied_steering_map", {})
    if raw_servo is not None and isinstance(applied, dict):
        try:
            gain = float(applied["steering_angle_to_servo_gain"])
            offset = float(applied["steering_angle_to_servo_offset"])
            mapped_angle = abs((float(raw_servo) - offset) / gain)
        except (KeyError, TypeError, ValueError, ZeroDivisionError):
            mapped_angle = math.nan
        if math.isfinite(mapped_angle) and mapped_angle > 0.0:
            angle = mapped_angle
            planning_source = "applied independently validated steering map"
    if (
        not isinstance(policy, dict) or not policy
        or not math.isfinite(angle) or not 0.0 < angle < 0.5 * math.pi
        or not math.isfinite(wheelbase) or wheelbase <= 0.0
    ):
        return {
            "capture_mode": "bounded_arc",
            "capture_s": float(minimum_capture_s),
            "planned_revolutions": 0.0,
            "planning_wheel_angle_rad": angle,
            "nominal_radius_m": math.nan,
            "planning_source": planning_source,
        }

    speed = abs(float(speed_mps))
    try:
        override_radius = float(radius_override_m)
    except (TypeError, ValueError):
        override_radius = math.nan
    if math.isfinite(override_radius) and override_radius > 0.0:
        radius = override_radius
        angle = math.atan(wheelbase / radius)
        planning_source = "conservative onboard IMU/odometry turn-radius estimate after startup"
    else:
        radius = wheelbase / math.tan(angle)
    braking = float(policy["conservative_braking_mps2"])
    brake_distance = speed * speed / (2.0 * braking)
    body_radius = float(policy["vehicle_circumscribed_radius_m"])
    target_side = float(policy["target_circle_side_m"])
    startup_s = float(policy.get("steering_startup_planning_s", 0.0))
    tangent_distance = speed * (
        float(centre_before_s) + float(return_after_s)
        + (startup_s if centre_before_s > 0.0 else 0.0)
    ) + brake_distance
    max_circle_radius = 0.5 * (target_side - 2.0 * body_radius - tangent_distance)
    revolutions = float(policy.get("full_circle_revolutions", 1.0))
    if radius <= max_circle_radius + 1.0e-9:
        capture_s = max(
            float(minimum_capture_s),
            revolutions * 2.0 * math.pi * radius / speed,
        )
        return {
            "capture_mode": "full_circle",
            "capture_s": capture_s,
            "planned_revolutions": revolutions,
            "planning_wheel_angle_rad": angle,
            "nominal_radius_m": radius,
            "maximum_fitting_radius_m": max_circle_radius,
            "planning_source": planning_source,
        }

    target_length = float(policy["clear_room_length_m"]) * float(policy["target_room_utilization"])
    target_width = float(policy["clear_room_width_m"]) * float(policy["target_room_utilization"])
    vehicle_length = float(policy["vehicle_length_m"])
    vehicle_width = float(policy["vehicle_width_m"])
    heading = math.radians(float(policy["straight_lane_heading_deg"]))
    cos_heading = abs(math.cos(heading))
    sin_heading = abs(math.sin(heading))
    maximum_s = float(policy.get("maximum_useful_capture_s", minimum_capture_s))

    def fits(duration_s: float) -> bool:
        straight_s = startup_s + float(centre_before_s) + float(return_after_s)
        if centre_before_s <= 0.0:
            turn_duration_s = startup_s + duration_s
        else:
            turn_duration_s = duration_s
        envelope_length = (
            speed * (straight_s + duration_s) + brake_distance + vehicle_length
        )
        arc_angle = min(math.pi, speed * turn_duration_s / radius)
        lateral = radius * (1.0 - math.cos(arc_angle))
        envelope_width = vehicle_width + lateral
        projected_length = envelope_length * cos_heading + envelope_width * sin_heading
        projected_width = envelope_length * sin_heading + envelope_width * cos_heading
        return projected_length <= target_length and projected_width <= target_width

    minimum_s = float(minimum_capture_s)
    if not fits(minimum_s):
        bounded_s = minimum_s
    elif fits(maximum_s):
        bounded_s = maximum_s
    else:
        low = minimum_s
        high = maximum_s
        for _ in range(48):
            midpoint = 0.5 * (low + high)
            if fits(midpoint):
                low = midpoint
            else:
                high = midpoint
        bounded_s = low
    return {
        "capture_mode": "bounded_arc",
        "capture_s": bounded_s,
        "planned_revolutions": 0.0,
        "planning_wheel_angle_rad": angle,
        "nominal_radius_m": radius,
        "maximum_fitting_radius_m": max_circle_radius,
        "planning_source": planning_source,
    }


def sensor_observability(config: dict[str, Any], stage_dir: Path, centre: dict[str, Any],
                         limits: dict[str, Any] | None = None) -> dict[str, Any]:
    """Establish LiDAR/IMU motion observability before any LiDAR-derived fit.

    The first invocation deliberately runs before the steering centre is known.
    It therefore uses the deployed raw-servo seed for stationary/straight
    passes and omits the optional gentle-turn diagnostic until mechanical limits
    exist.  This prevents a bad scan scene from consuming a centre-fit run.
    """
    banner("LIDAR/IMU OBSERVABILITY PREFLIGHT", "Raw sensor characterisation; no parameters are fitted at runtime.")
    checklist(["Clear test lane and fixed LiDAR surroundings.", "No moving people or objects in scan field.",
               "Vehicle can be repositioned between passes."])
    require_ready()
    node = _start_node("steering_sensor_observability", config, {"imu", "vesc", "odom", "servo_echo", "servo_selected", "servo_bus", "scan"})
    p = config["observability"]
    c = float(centre["centre_servo_raw"])
    records: list[dict[str, Any]] = []
    try:
        # One accepted stationary baseline establishes the ICP noise floor. REDO is unlimited.
        attempt = 1
        while True:
            trial_id = _trial_id("observability_stationary", attempt)
            pause_for_reposition("STATIONARY OBSERVABILITY CAPTURE\nDo not touch the car during recording.")
            node.event.emit("trial_start", stage="sensor_observability", condition_id="observability_stationary",
                            trial_id=trial_id, attempt=attempt, raw_servo_target=c)
            summary = node.hold(speed_mps=0.0, raw_servo=c, duration_s=float(p["stationary_s"]),
                                phase="observability_stationary", segment_id="observability_stationary",
                                capture=True, centre_raw_servo=c, begin_window_fields=WINDOW_FIELDS,
                                trial_id=trial_id)
            decision = _disposition(node, stage="sensor_observability", condition_id="observability_stationary",
                                    trial_id=trial_id, attempt=attempt, automatic_ok=True,
                                    automatic_summary={"capture": summary})
            records.append({"condition_id": "observability_stationary", "trial_id": trial_id,
                            "attempt": attempt, "capture": summary, "decision": decision})
            if decision != "redo":
                break
            attempt += 1
        for idx, speed in enumerate(p["straight_speeds_mps"]):
            for rep in range(1, int(p["straight_repetitions"]) + 1):
                records.extend(_motion_loop(node, stage="sensor_observability",
                    condition_id=f"observability_straight_{idx:02d}_rep_{rep:02d}",
                    operator_message=(f"STRAIGHT OBSERVABILITY {idx+1}/{len(p['straight_speeds_mps'])}, "
                                      f"repetition {rep}/{p['straight_repetitions']}"),
                    speed_mps=float(speed), raw_servo=c, centre_raw=c,
                    phase="observability_straight_capture", capture_s=float(p["straight_capture_s"]),
                    metadata={"speed_index": idx, "speed_mps": float(speed), "repetition": rep}))
        if limits is not None:
            gentle_fraction = float(p["gentle_turn_fraction_of_safe_span"])
            for side, fraction, raw in _raw_targets(limits, [gentle_fraction]):
                for rep in range(1, int(p["turn_repetitions"]) + 1):
                    records.extend(_motion_loop(node, stage="sensor_observability",
                        condition_id=f"observability_turn_{side}_rep_{rep:02d}",
                        operator_message=f"GENTLE {side.upper()} OBSERVABILITY TURN, repetition {rep}/{p['turn_repetitions']}",
                        speed_mps=float(p["turn_speed_mps"]), raw_servo=raw, centre_raw=c,
                        phase="observability_turn_capture", capture_s=float(p["turn_capture_s"]),
                        metadata={"side": side, "fraction": fraction, "raw_servo_target": raw, "repetition": rep}))
        result = {"status": "captured", "records": records,
                  "preflight_only": limits is None}
        dump_json(stage_dir / "runtime_result.json", result)
        return result
    finally:
        _finish_node(node, c)


def _static_sequence(limits: dict[str, Any], params: dict[str, Any], validation: bool) -> list[tuple[str, float, float, str, int]]:
    fractions = list(params["validation_fractions"] if validation else params["training_fractions"])
    repeats = int(params["validation_repetitions"] if validation else params["training_sweep_repetitions"])
    by_side: dict[str, list[tuple[str, float, float]]] = {"high_raw": [], "low_raw": []}
    for target in _raw_targets(limits, fractions):
        by_side[target[0]].append(target)
    sequence: list[tuple[str, float, float, str, int]] = []
    rng = np.random.default_rng(20260622 if validation else 20260621)
    for sweep in range(1, repeats + 1):
        for side in ("high_raw", "low_raw"):
            values = by_side[side]
            if validation:
                shuffled = list(values)
                rng.shuffle(shuffled)
                for s, f, target in shuffled:
                    sequence.append((s, f, target, "shuffled", sweep))
            else:
                for s, f, target in values:
                    sequence.append((s, f, target, "outward", sweep))
                for s, f, target in reversed(values):
                    sequence.append((s, f, target, "inward", sweep))
    return sequence


def static_map(config: dict[str, Any], stage_dir: Path, centre: dict[str, Any], limits: dict[str, Any], *, validation: bool) -> dict[str, Any]:
    title = "STAGE 5 OF 6 — STATIC-MAP HOLD-OUT VALIDATION" if validation else "STAGE 4 OF 6 — STATIC STEERING MAP"
    subtitle = "Independent shuffled hold-out captures." if validation else "Repeated outward/inward sweeps per side."
    banner(title, subtitle)
    checklist(["Clear area around vehicle.", "Flat, dry surface and fixed LiDAR surroundings.",
               "Vehicle can be repositioned after every arc.", "Physical emergency stop is available."])
    require_ready()
    node = _start_node("steering_static_map_validation" if validation else "steering_static_map", config,
                       {"imu", "vesc", "odom", "servo_echo", "servo_selected", "servo_bus", "scan"})
    p = config["static_map"]
    c = float(centre["centre_servo_raw"])
    records: list[dict[str, Any]] = []
    try:
        sequence = _static_sequence(limits, p, validation)
        for index, (side, fraction, raw, approach, sweep) in enumerate(sequence, start=1):
            condition = f"{'validation' if validation else 'training'}_{side}_{fraction:.2f}_{approach}_sweep_{sweep:02d}"
            capture_plan = _steering_capture_plan(
                config, limits, side=side, fraction=fraction,
                speed_mps=float(p["speed_mps"]),
                minimum_capture_s=float(p["capture_s"]),
                raw_servo=raw,
            )
            capture_s = float(capture_plan["capture_s"])
            attempt = 1
            while True:
                trial_id = _trial_id(condition, attempt)
                pause_for_reposition(
                    f"{'VALIDATION' if validation else 'STATIC MAP'} POINT {index}/{len(sequence)}\n"
                    f"Raw servo {raw:.5f}; {side}; {approach}; sweep {sweep}.\n"
                    f"Plan: {capture_plan['capture_mode'].replace('_', ' ')}, {capture_s:.1f} s recorded.\n"
                    "When READY is pressed, steering will move while stationary, then the vehicle will stabilise and capture."
                )
                node.event.emit("trial_start", stage="static_map", condition_id=condition, trial_id=trial_id,
                                attempt=attempt, speed_mps=float(p["speed_mps"]), side=side, fraction=fraction,
                                raw_servo_target=raw, approach=approach, sweep=sweep, validation=validation,
                                capture_mode=str(capture_plan["capture_mode"]),
                                planned_capture_s=capture_s,
                                planned_revolutions=float(capture_plan["planned_revolutions"]))
                node.event.emit("static_map_target", condition_id=condition, trial_id=trial_id, side=side,
                                fraction=fraction, raw_servo_target=raw, approach=approach, sweep=sweep,
                                validation=validation)
                settle = _settle_raw_servo(
                    node, raw_servo=raw, centre_raw=c,
                    phase="static_map_steering_settle", segment_id=condition,
                    trial_id=trial_id, tolerance=float(p["raw_servo_echo_tolerance"]),
                    initial_s=float(p["steering_settle_s"]),
                    timeout_s=float(p.get("steering_settle_timeout_s", p["steering_settle_s"])),
                    final_window_s=float(p.get("steering_settle_final_window_s", p["steering_settle_s"])),
                    metadata={"side": side, "fraction": fraction, "approach": approach,
                              "sweep": sweep, "validation": validation},
                )
                echo_error = float(settle["settle_echo_error"])
                if not bool(settle["settle_within_tolerance"]):
                    node.neutral_drive(c)
                    node.event.emit("static_map_settle_echo_mismatch", condition_id=condition,
                                    trial_id=trial_id, side=side, fraction=fraction,
                                    raw_servo_target=raw, approach=approach, sweep=sweep,
                                    validation=validation, settle_echo_error=echo_error,
                                    settle_echo_max_error=float(settle["settle_echo_max_error"]),
                                    settle_elapsed_s=float(settle["settle_elapsed_s"]),
                                    tolerance=float(p["raw_servo_echo_tolerance"]))
                    decision = _disposition(
                        node, stage="static_map", condition_id=condition, trial_id=trial_id,
                        attempt=attempt, automatic_ok=False,
                        automatic_summary={
                            "settle_echo_error": echo_error,
                            "settle_echo_max_error": float(settle["settle_echo_max_error"]),
                            "settle_elapsed_s": float(settle["settle_elapsed_s"]),
                            "tolerance": float(p["raw_servo_echo_tolerance"]),
                            "reason": "raw servo command clipped or stale",
                        },
                    )
                    records.append({"condition_id": condition, "trial_id": trial_id,
                                    "attempt": attempt, "side": side, "fraction": fraction,
                                    "raw_servo_target": raw, "approach": approach,
                                    "sweep": sweep, "validation": validation,
                                    "startup": None, "capture": None, "decision": decision,
                                    "settle": settle, "settle_echo_error": echo_error,
                                    "reason": "raw_servo_echo_mismatch"})
                    if decision != "redo":
                        break
                    attempt += 1
                    continue
                startup = node.establish_speed(
                    speed_mps=float(p["speed_mps"]), raw_servo=raw,
                    centre_raw_servo=c, segment_id=condition, trial_id=trial_id,
                )
                active_plan = dict(capture_plan)
                summary: dict[str, Any] | None = None
                if bool(startup["stable"]):
                    yaw_candidates = [
                        abs(float(value)) for value in (node.latest.imu_gz, node.latest.odom_wz)
                        if math.isfinite(float(value)) and abs(float(value)) >= 0.02
                    ]
                    if yaw_candidates:
                        # The smaller observed yaw rate gives the larger, safer
                        # radius estimate. It is used only for room scheduling;
                        # LiDAR still supplies the fitted steering angle.
                        onboard_radius = abs(float(p["speed_mps"])) / min(yaw_candidates)
                        active_plan = _steering_capture_plan(
                            config, limits, side=side, fraction=fraction,
                            speed_mps=float(p["speed_mps"]),
                            minimum_capture_s=float(p["capture_s"]),
                            raw_servo=raw, radius_override_m=onboard_radius,
                        )
                        if (
                            active_plan["capture_mode"] != capture_plan["capture_mode"]
                            or abs(float(active_plan["capture_s"]) - capture_s) > 0.05
                        ):
                            note(
                                "Onboard turn estimate refined this pass to "
                                f"{active_plan['capture_mode'].replace('_', ' ')} for "
                                f"{float(active_plan['capture_s']):.1f} s."
                            )
                    active_capture_s = float(active_plan["capture_s"])
                    yaw_target = (
                        2.0 * math.pi * float(active_plan["planned_revolutions"])
                        if active_plan["capture_mode"] == "full_circle" else None
                    )
                    node.event.emit(
                        "static_map_capture_plan", condition_id=condition,
                        trial_id=trial_id, **active_plan,
                    )
                    summary = node.hold(
                        speed_mps=float(p["speed_mps"]), raw_servo=raw,
                        duration_s=active_capture_s, phase="static_map_capture",
                        segment_id=condition, capture=True, centre_raw_servo=c,
                        begin_window_fields=WINDOW_FIELDS, trial_id=trial_id,
                        side=side, fraction=fraction, raw_servo_target=raw,
                        approach=approach, sweep=sweep, validation=validation,
                        settle_echo_error=echo_error,
                        capture_mode=str(active_plan["capture_mode"]),
                        planned_capture_s=active_capture_s,
                        planned_revolutions=float(active_plan["planned_revolutions"]),
                        planning_source=str(active_plan["planning_source"]),
                        nominal_radius_m=float(active_plan["nominal_radius_m"]),
                        target_abs_yaw_change_rad=yaw_target,
                        minimum_duration_s=(0.50 * active_capture_s if yaw_target is not None else 0.0),
                        maximum_duration_s=(1.50 * active_capture_s if yaw_target is not None else None),
                    )
                node.neutral_drive(c)
                circle_complete = (
                    active_plan["capture_mode"] != "full_circle"
                    or bool((summary or {}).get("yaw_target_reached", False))
                )
                automatic_ok = bool(startup["stable"]) and summary is not None and circle_complete
                decision = _disposition(node, stage="static_map", condition_id=condition, trial_id=trial_id,
                                        attempt=attempt, automatic_ok=automatic_ok,
                                        automatic_summary={"startup": startup, "capture": summary or {},
                                                           "settle_echo_error": echo_error,
                                                           "circle_complete": circle_complete})
                records.append({"condition_id": condition, "trial_id": trial_id, "attempt": attempt,
                                "side": side, "fraction": fraction, "raw_servo_target": raw,
                                "approach": approach, "sweep": sweep, "validation": validation,
                                "startup": startup, "capture": summary, "decision": decision,
                                "settle_echo_error": echo_error, **active_plan})
                if decision != "redo":
                    break
                attempt += 1
        result = {"status": "captured", "validation": validation, "records": records}
        dump_json(stage_dir / "runtime_result.json", result)
        return result
    finally:
        _finish_node(node, c)


def _response_sequence(config: dict[str, Any], limits: dict[str, Any], *,
                       validation: bool = False) -> list[tuple[float, str, float, float, int, int]]:
    """Expand configured response conditions into speed/side/fraction targets."""
    sequence: list[tuple[float, str, float, float, int, int]] = []
    condition_key = "validation_conditions" if validation else "conditions"
    if condition_key in config:
        for condition in config[condition_key]:
            speed = float(condition["speed_mps"])
            repetitions = int(condition.get("repetitions", config.get("repetitions", 1)))
            for side, fraction, raw in _raw_targets(limits, list(condition["target_fractions"])):
                for rep in range(1, repetitions + 1):
                    sequence.append((speed, side, fraction, raw, rep, repetitions))
        return sequence
    for speed in config["speeds_mps"]:
        repetitions = int(config["repetitions"])
        for side, fraction, raw in _raw_targets(limits, list(config["target_fractions"])):
            for rep in range(1, repetitions + 1):
                sequence.append((float(speed), side, fraction, raw, rep, repetitions))
    return sequence


def command_to_curvature_response(config: dict[str, Any], stage_dir: Path, centre: dict[str, Any],
                                  limits: dict[str, Any], *, validation: bool = False) -> dict[str, Any]:
    banner("STEERING RESPONSE HOLD-OUT VALIDATION" if validation else "COMMAND-TO-CURVATURE RESPONSE TRAINING",
           "Distinct raw-servo steps at selected speeds; the hold-out is never used to fit the response candidate." if validation
           else "Targeted raw-servo steps at selected speeds.")
    checklist(["Clear test area and static LiDAR surroundings.", "Vehicle can be repositioned between trials.",
               "Physical emergency stop is available."])
    require_ready()
    node = _start_node("steering_command_to_curvature_response_validation" if validation else "steering_command_to_curvature_response", config, {"imu", "vesc", "odom", "servo_echo", "servo_selected", "servo_bus", "scan"})
    p = config["response"]
    c = float(centre["centre_servo_raw"])
    records: list[dict[str, Any]] = []
    index = 0
    try:
        sequence = _response_sequence(p, limits, validation=validation)
        for speed, side, fraction, raw_target, rep, repetitions in sequence:
            condition = f"{'response_validation' if validation else 'response'}_{speed:.2f}_{side}_{fraction:.2f}_rep_{rep:02d}"
            capture_plan = _steering_capture_plan(
                config, limits, side=side, fraction=fraction,
                speed_mps=speed, minimum_capture_s=float(p["step_hold_s"]),
                centre_before_s=float(p["centre_hold_s"]),
                return_after_s=float(p["return_hold_s"]),
                raw_servo=raw_target,
            )
            step_hold_s = float(capture_plan["capture_s"])
            attempt = 1
            while True:
                trial_id = _trial_id(condition, attempt)
                pause_for_reposition(
                    f"RESPONSE TRIAL {index + 1}/{len(sequence)}\nSpeed {speed:.2f} m/s; {side}; "
                    f"{fraction:.2f} safe-span; repetition {rep}/{repetitions}.\n"
                    f"The steering step records a {capture_plan['capture_mode'].replace('_', ' ')} "
                    f"for {step_hold_s:.1f} s at planned radius "
                    f"{float(capture_plan['nominal_radius_m']):.2f} m, returns to centre, then stops."
                )
                node.event.emit("trial_start", stage="command_to_curvature_response_validation" if validation else "command_to_curvature_response", condition_id=condition,
                                trial_id=trial_id, attempt=attempt, speed_mps=speed, side=side,
                                fraction=fraction, raw_servo_target=raw_target, repetition=rep,
                                capture_mode=str(capture_plan["capture_mode"]),
                                planned_step_hold_s=step_hold_s,
                                planned_revolutions=float(capture_plan["planned_revolutions"]),
                                nominal_radius_m=float(capture_plan["nominal_radius_m"]),
                                planning_source=str(capture_plan["planning_source"]))
                startup = node.establish_speed(speed_mps=speed, raw_servo=c, centre_raw_servo=c,
                                               segment_id=condition, trial_id=trial_id)
                if bool(startup["stable"]):
                    centre_summary = node.hold(speed_mps=speed, raw_servo=c,
                        duration_s=float(p["centre_hold_s"]), phase="response_centre", segment_id=condition,
                        capture=True, centre_raw_servo=c, begin_window_fields=WINDOW_FIELDS, trial_id=trial_id,
                        side=side, fraction=fraction, repetition=rep)
                    node.event.emit("response_step_command", trial_id=trial_id, condition_id=condition,
                                    trial_index=index, speed_mps=speed, side=side, fraction=fraction,
                                    raw_target=raw_target, repetition=rep)
                    step_summary = node.hold(speed_mps=speed, raw_servo=raw_target,
                        duration_s=step_hold_s, phase="response_step", segment_id=condition,
                        capture=True, centre_raw_servo=c, begin_window_fields=WINDOW_FIELDS, trial_id=trial_id,
                        side=side, fraction=fraction, repetition=rep,
                        capture_mode=str(capture_plan["capture_mode"]),
                        planned_capture_s=step_hold_s,
                        planned_revolutions=float(capture_plan["planned_revolutions"]),
                        nominal_radius_m=float(capture_plan["nominal_radius_m"]),
                        planning_source=str(capture_plan["planning_source"]),
                        target_abs_yaw_change_rad=(
                            2.0 * math.pi * float(capture_plan["planned_revolutions"])
                            if capture_plan["capture_mode"] == "full_circle" else None
                        ),
                        minimum_duration_s=(
                            0.50 * step_hold_s if capture_plan["capture_mode"] == "full_circle" else 0.0
                        ),
                        maximum_duration_s=(
                            1.50 * step_hold_s if capture_plan["capture_mode"] == "full_circle" else None
                        ))
                    node.event.emit("response_return_command", trial_id=trial_id, condition_id=condition,
                                    trial_index=index, raw_target=c)
                    return_summary = node.hold(speed_mps=speed, raw_servo=c,
                        duration_s=float(p["return_hold_s"]), phase="response_return", segment_id=condition,
                        capture=True, centre_raw_servo=c, begin_window_fields=WINDOW_FIELDS, trial_id=trial_id,
                        side=side, fraction=fraction, repetition=rep)
                else:
                    centre_summary = step_summary = return_summary = None
                node.neutral_drive(c)
                circle_complete = (
                    capture_plan["capture_mode"] != "full_circle"
                    or bool((step_summary or {}).get("yaw_target_reached", False))
                )
                automatic_ok = bool(startup["stable"]) and circle_complete
                decision = _disposition(node, stage="command_to_curvature_response_validation" if validation else "command_to_curvature_response",
                    condition_id=condition, trial_id=trial_id, attempt=attempt, automatic_ok=automatic_ok,
                    automatic_summary={"startup": startup, "centre": centre_summary or {},
                                       "step": step_summary or {}, "return": return_summary or {},
                                       "circle_complete": circle_complete})
                records.append({"trial_index": index, "condition_id": condition, "trial_id": trial_id,
                                "attempt": attempt, "speed_mps": speed, "side": side,
                                "fraction": fraction, "raw_target": raw_target, "startup": startup,
                                "centre_summary": centre_summary, "step_summary": step_summary,
                                "return_summary": return_summary, "decision": decision,
                                **capture_plan})
                if decision != "redo":
                    break
                attempt += 1
            index += 1
        result = {"status": "captured", "validation": validation, "records": records}
        dump_json(stage_dir / "runtime_result.json", result)
        return result
    finally:
        _finish_node(node, c)
