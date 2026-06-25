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
from .runtime import CalibrationNode
from .ui import banner, checklist, note, pause_for_reposition, require_ready, review_trial, warn

WINDOW_FIELDS = ("imu_gz", "imu_ax", "imu_ay", "servo_echo", "servo_selected", "servo_bus", "odom_vx")


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


def zero_curvature_centre(config: dict[str, Any], stage_dir: Path) -> dict[str, Any]:
    """Adaptive raw-servo search for the yaw-rate zero crossing.

    The runner first brackets a sign change in mean IMU yaw rate, then applies
    safeguarded secant/bisection steps. Each candidate has repeated accepted
    runs. All values are raw servo values; no old steering-angle map is used.
    """
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
        raw = float(np.clip(raw, low_guard, high_guard))
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
        centre_raw = float(np.clip(centre_raw, low_guard, high_guard))

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


def physical_endstops(config: dict[str, Any], stage_dir: Path, centre: dict[str, Any]) -> dict[str, Any]:
    banner("STAGE 2 OF 6 — PHYSICAL END-STOP SURVEY", "One explicit operator-confirmed last-free value per side.")
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
                            result_raw[side] = current
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
        result = {"status": "confirmed", "centre_servo_raw": c,
                  "raw_low_last_free": result_raw["low_raw"], "raw_high_last_free": result_raw["high_raw"],
                  "raw_low_safe": safe_low, "raw_high_safe": safe_high,
                  "safety_margin_servo": float(p["safety_margin_servo"])}
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


def sensor_observability(config: dict[str, Any], stage_dir: Path, centre: dict[str, Any], limits: dict[str, Any]) -> dict[str, Any]:
    banner("STAGE 3 OF 6 — SENSOR OBSERVABILITY", "Raw sensor characterisation; no parameters are fitted at runtime.")
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
        gentle_fraction = float(p["gentle_turn_fraction_of_safe_span"])
        for side, fraction, raw in _raw_targets(limits, [gentle_fraction]):
            for rep in range(1, int(p["turn_repetitions"]) + 1):
                records.extend(_motion_loop(node, stage="sensor_observability",
                    condition_id=f"observability_turn_{side}_rep_{rep:02d}",
                    operator_message=f"GENTLE {side.upper()} OBSERVABILITY TURN, repetition {rep}/{p['turn_repetitions']}",
                    speed_mps=float(p["turn_speed_mps"]), raw_servo=raw, centre_raw=c,
                    phase="observability_turn_capture", capture_s=float(p["turn_capture_s"]),
                    metadata={"side": side, "fraction": fraction, "raw_servo_target": raw, "repetition": rep}))
        result = {"status": "captured", "records": records}
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
    subtitle = "Independent shuffled hold-out captures." if validation else "Four repeated outward/inward sweeps per side."
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
            attempt = 1
            while True:
                trial_id = _trial_id(condition, attempt)
                pause_for_reposition(
                    f"{'VALIDATION' if validation else 'STATIC MAP'} POINT {index}/{len(sequence)}\n"
                    f"Raw servo {raw:.5f}; {side}; {approach}; sweep {sweep}.\n"
                    "When READY is pressed, steering will move while stationary, then the vehicle will stabilise and capture."
                )
                node.event.emit("trial_start", stage="static_map", condition_id=condition, trial_id=trial_id,
                                attempt=attempt, speed_mps=float(p["speed_mps"]), side=side, fraction=fraction,
                                raw_servo_target=raw, approach=approach, sweep=sweep, validation=validation)
                node.event.emit("static_map_target", condition_id=condition, trial_id=trial_id, side=side,
                                fraction=fraction, raw_servo_target=raw, approach=approach, sweep=sweep,
                                validation=validation)
                settle = node.hold(speed_mps=0.0, raw_servo=raw, duration_s=float(p["steering_settle_s"]),
                                   phase="static_map_steering_settle", segment_id=condition, capture=True,
                                   centre_raw_servo=c, begin_window_fields=("servo_echo",), trial_id=trial_id,
                                   side=side, fraction=fraction, approach=approach, sweep=sweep)
                echo_error = abs(float(settle["servo_echo_mean"]) - raw)
                if echo_error > float(p["raw_servo_echo_tolerance"]):
                    node.neutral_drive(c)
                    raise RuntimeError(f"raw servo command clipped or stale at {condition}: error={echo_error:.5f}")
                startup, summary = _capture_after_startup(
                    node, speed_mps=float(p["speed_mps"]), raw_servo=raw, centre_raw=c,
                    phase="static_map_capture", segment_id=condition, trial_id=trial_id,
                    capture_s=float(p["capture_s"]), metadata={"side": side, "fraction": fraction,
                    "raw_servo_target": raw, "approach": approach, "sweep": sweep,
                    "validation": validation, "settle_echo_error": echo_error},
                )
                node.neutral_drive(c)
                automatic_ok = bool(startup["stable"]) and summary is not None
                decision = _disposition(node, stage="static_map", condition_id=condition, trial_id=trial_id,
                                        attempt=attempt, automatic_ok=automatic_ok,
                                        automatic_summary={"startup": startup, "capture": summary or {},
                                                           "settle_echo_error": echo_error})
                records.append({"condition_id": condition, "trial_id": trial_id, "attempt": attempt,
                                "side": side, "fraction": fraction, "raw_servo_target": raw,
                                "approach": approach, "sweep": sweep, "validation": validation,
                                "startup": startup, "capture": summary, "decision": decision,
                                "settle_echo_error": echo_error})
                if decision != "redo":
                    break
                attempt += 1
        result = {"status": "captured", "validation": validation, "records": records}
        dump_json(stage_dir / "runtime_result.json", result)
        return result
    finally:
        _finish_node(node, c)


def command_to_curvature_response(config: dict[str, Any], stage_dir: Path, centre: dict[str, Any], limits: dict[str, Any]) -> dict[str, Any]:
    banner("STAGE 6 OF 6 — COMMAND-TO-CURVATURE RESPONSE", "Repeated raw-servo steps at two low speeds.")
    checklist(["Clear test area and static LiDAR surroundings.", "Vehicle can be repositioned between trials.",
               "Physical emergency stop is available."])
    require_ready()
    node = _start_node("steering_command_to_curvature_response", config, {"imu", "vesc", "odom", "servo_echo", "servo_selected", "servo_bus", "scan"})
    p = config["response"]
    c = float(centre["centre_servo_raw"])
    records: list[dict[str, Any]] = []
    index = 0
    try:
        for speed in p["speeds_mps"]:
            for side, fraction, raw_target in _raw_targets(limits, list(p["target_fractions"])):
                for rep in range(1, int(p["repetitions"]) + 1):
                    condition = f"response_{float(speed):.2f}_{side}_{fraction:.2f}_rep_{rep:02d}"
                    attempt = 1
                    while True:
                        trial_id = _trial_id(condition, attempt)
                        pause_for_reposition(
                            f"RESPONSE TRIAL {index + 1}\nSpeed {float(speed):.2f} m/s; {side}; "
                            f"{fraction:.2f} safe-span; repetition {rep}/{p['repetitions']}.\n"
                            "The car stabilises at centre, steps raw steering, returns to centre, then stops."
                        )
                        node.event.emit("trial_start", stage="command_to_curvature_response", condition_id=condition,
                                        trial_id=trial_id, attempt=attempt, speed_mps=float(speed), side=side,
                                        fraction=fraction, raw_servo_target=raw_target, repetition=rep)
                        startup = node.establish_speed(speed_mps=float(speed), raw_servo=c, centre_raw_servo=c,
                                                       segment_id=condition, trial_id=trial_id)
                        if bool(startup["stable"]):
                            centre_summary = node.hold(speed_mps=float(speed), raw_servo=c,
                                duration_s=float(p["centre_hold_s"]), phase="response_centre", segment_id=condition,
                                capture=True, centre_raw_servo=c, begin_window_fields=WINDOW_FIELDS, trial_id=trial_id,
                                side=side, fraction=fraction, repetition=rep)
                            node.event.emit("response_step_command", trial_id=trial_id, condition_id=condition,
                                            trial_index=index, speed_mps=float(speed), side=side, fraction=fraction,
                                            raw_target=raw_target, repetition=rep)
                            step_summary = node.hold(speed_mps=float(speed), raw_servo=raw_target,
                                duration_s=float(p["step_hold_s"]), phase="response_step", segment_id=condition,
                                capture=True, centre_raw_servo=c, begin_window_fields=WINDOW_FIELDS, trial_id=trial_id,
                                side=side, fraction=fraction, repetition=rep)
                            node.event.emit("response_return_command", trial_id=trial_id, condition_id=condition,
                                            trial_index=index, raw_target=c)
                            return_summary = node.hold(speed_mps=float(speed), raw_servo=c,
                                duration_s=float(p["return_hold_s"]), phase="response_return", segment_id=condition,
                                capture=True, centre_raw_servo=c, begin_window_fields=WINDOW_FIELDS, trial_id=trial_id,
                                side=side, fraction=fraction, repetition=rep)
                        else:
                            centre_summary = step_summary = return_summary = None
                        node.neutral_drive(c)
                        automatic_ok = bool(startup["stable"])
                        decision = _disposition(node, stage="command_to_curvature_response",
                            condition_id=condition, trial_id=trial_id, attempt=attempt, automatic_ok=automatic_ok,
                            automatic_summary={"startup": startup, "centre": centre_summary or {},
                                               "step": step_summary or {}, "return": return_summary or {}})
                        records.append({"trial_index": index, "condition_id": condition, "trial_id": trial_id,
                                        "attempt": attempt, "speed_mps": float(speed), "side": side,
                                        "fraction": fraction, "raw_target": raw_target, "startup": startup,
                                        "centre_summary": centre_summary, "step_summary": step_summary,
                                        "return_summary": return_summary, "decision": decision})
                        if decision != "redo":
                            break
                        attempt += 1
                    index += 1
        result = {"status": "captured", "records": records}
        dump_json(stage_dir / "runtime_result.json", result)
        return result
    finally:
        _finish_node(node, c)
