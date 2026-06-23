"""Lossless-enough MCAP recording and per-stage evidence capture.

The calibration stack is dedicated, so the bagger records *all* visible and
hidden topics instead of relying on a hand-maintained shortlist. A required
subset is verified from metadata after recording stops; the complete topic
inventory, ROS graph, parameter dumps and bag topic counts are stored beside
each MCAP bag.
"""
from __future__ import annotations

import os
import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass
class BagProcess:
    stage_dir: Path
    bag_dir: Path
    process: subprocess.Popen
    stdout_handle: object
    required_topics: list[str]


def _run_capture(command: list[str], output: Path, *, timeout_s: float = 12.0) -> None:
    """Capture a ROS CLI diagnostic without making optional diagnostics fatal."""
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, timeout=timeout_s, check=False)
        output.write_text(result.stdout, encoding="utf-8")
    except Exception as exc:  # graph snapshots are supporting evidence only
        output.write_text(f"COMMAND FAILED: {command!r}\n{exc!r}\n", encoding="utf-8")


def snapshot_ros_graph(stage_dir: Path, phase: str, required_topics: list[str]) -> None:
    """Save enough launch/graph evidence to diagnose a missing topic afterwards."""
    directory = stage_dir / f"ros_graph_{phase}"
    directory.mkdir(parents=True, exist_ok=True)
    _run_capture(["ros2", "node", "list"], directory / "nodes.txt")
    _run_capture(["ros2", "topic", "list", "-t"], directory / "topics_and_types.txt")
    _run_capture(["ros2", "service", "list", "-t"], directory / "services_and_types.txt")
    _run_capture(["ros2", "param", "dump", "/ackermann_to_vesc_node"], directory / "ackermann_to_vesc_params.yaml")
    _run_capture(["ros2", "param", "dump", "/vesc_driver_node"], directory / "vesc_driver_params.yaml")
    _run_capture(["ros2", "param", "dump", "/servo_selector"], directory / "servo_selector_params.yaml")
    for topic in required_topics:
        safe_name = topic.strip("/").replace("/", "__") or "root"
        _run_capture(["ros2", "topic", "info", "--verbose", topic], directory / f"topic_{safe_name}.txt")


def _metadata_topic_counts(bag_dir: Path) -> dict[str, int]:
    metadata_path = bag_dir / "metadata.yaml"
    if not metadata_path.is_file():
        return {}
    document = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
    info = document.get("rosbag2_bagfile_information", document)
    counts: dict[str, int] = {}
    for entry in info.get("topics_with_message_count", []) or []:
        meta = entry.get("topic_metadata", {}) or {}
        name = meta.get("name")
        if name:
            counts[str(name)] = int(entry.get("message_count", 0) or 0)
    return counts


def verify_bag_topics(bag_dir: Path, required_topics: list[str]) -> dict[str, Any]:
    counts = _metadata_topic_counts(bag_dir)
    missing = [topic for topic in required_topics if counts.get(topic, 0) <= 0]
    result = {
        "bag_dir": str(bag_dir),
        "topic_count": len(counts),
        "required_topics": required_topics,
        "missing_or_empty_required_topics": missing,
        "required_topic_counts": {topic: int(counts.get(topic, 0)) for topic in required_topics},
        "all_topic_counts": dict(sorted(counts.items())),
        "ok": not missing,
    }
    (bag_dir.parent / "bag_topic_counts.yaml").write_text(
        yaml.safe_dump(result, sort_keys=False), encoding="utf-8"
    )
    return result


def start_bag(
    stage_dir: Path,
    recording: dict[str, Any],
    required_topics: list[str],
    steering_root: Path,
) -> BagProcess:
    """Start MCAP recording before the stage runtime node exists.

    The explicit ALL-topics strategy is deliberate. The required topic list is
    only a post-run integrity check; it is not a restrictive allowlist.
    """
    stage_dir.mkdir(parents=True, exist_ok=True)
    bag_dir = stage_dir / "bag"
    if bag_dir.exists():
        raise FileExistsError(f"refusing to overwrite existing bag: {bag_dir}")
    qos_rel = recording.get("qos_profile_overrides")
    qos_path = (steering_root / qos_rel).resolve() if qos_rel else None
    if qos_path and not qos_path.is_file():
        raise FileNotFoundError(f"bag QoS override file missing: {qos_path}")
    plan = {
        "storage": "mcap",
        "record_all_topics": bool(recording.get("record_all_topics", True)),
        "include_hidden_topics": bool(recording.get("include_hidden_topics", True)),
        "qos_profile_overrides": str(qos_path) if qos_path else None,
        "required_topics": required_topics,
    }
    (stage_dir / "recording_plan.yaml").write_text(yaml.safe_dump(plan, sort_keys=False), encoding="utf-8")
    snapshot_ros_graph(stage_dir, "before", required_topics)
    log_path = stage_dir / "rosbag_record.log"
    handle = log_path.open("w", encoding="utf-8")
    command = ["ros2", "bag", "record", "-s", "mcap", "-o", str(bag_dir)]
    if bool(recording.get("record_all_topics", True)):
        command.append("-a")
    if bool(recording.get("include_hidden_topics", True)):
        command.append("--include-hidden-topics")
    if qos_path:
        command += ["--qos-profile-overrides-path", str(qos_path)]
    process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT,
                               start_new_session=True, text=True)
    time.sleep(1.2)
    if process.poll() is not None:
        handle.close()
        detail = log_path.read_text(encoding="utf-8", errors="replace")
        raise RuntimeError(f"rosbag failed to start:\n{detail}")
    return BagProcess(stage_dir=stage_dir, bag_dir=bag_dir, process=process,
                      stdout_handle=handle, required_topics=list(required_topics))


def stop_bag(session: BagProcess, timeout_s: float = 20.0) -> dict[str, Any]:
    if session.process.poll() is None:
        try:
            os.killpg(session.process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            session.process.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(session.process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            session.process.wait(timeout=5.0)
    session.stdout_handle.close()
    snapshot_ros_graph(session.stage_dir, "after", session.required_topics)
    return verify_bag_topics(session.bag_dir, session.required_topics)
