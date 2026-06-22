#!/usr/bin/env python3
"""Audit stage completion, bag-topic completeness and trial dispositions."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml

from trials import accepted_trial_ids

EXPECTED = [
    "00_command_chain_audit", "01_zero_curvature_centre", "02_physical_endstops",
    "03_sensor_observability", "04_static_map_training", "05_static_map_holdout",
    "06_command_to_curvature_response",
]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    manifest = yaml.safe_load((session / "session_manifest.yaml").read_text(encoding="utf-8")) or {}
    policy_snapshot = session / "recording_policy_snapshot.yaml"
    if policy_snapshot.exists():
        topic_policy = yaml.safe_load(policy_snapshot.read_text(encoding="utf-8")) or {}
    else:
        root = Path(manifest.get("steering_root", Path(__file__).resolve().parents[1]))
        topic_policy = yaml.safe_load((root / "config" / "topics.yaml").read_text(encoding="utf-8")) or {}
    rows = []
    for stage in EXPECTED:
        directory = session / stage
        bag_ok = (directory / "bag" / "metadata.yaml").exists()
        verification_path = directory / "bag_topic_counts.yaml"
        verification = yaml.safe_load(verification_path.read_text(encoding="utf-8")) if verification_path.exists() else None
        derived = directory / "derived" / "export_summary.json"
        exported = json.loads(derived.read_text()) if derived.exists() else None
        events_path = directory / "derived" / "events.parquet"
        accepted = redo = skipped = None
        if events_path.exists():
            import pandas as pd
            events = pd.read_parquet(events_path)
            decisions = events[events.get("event") == "trial_decision"] if "event" in events else events.iloc[0:0]
            accepted = len(accepted_trial_ids(events))
            redo = int((decisions.get("decision") == "redo").sum()) if len(decisions) else 0
            skipped = int((decisions.get("decision") == "skipped").sum()) if len(decisions) else 0
        rows.append({
            "stage": stage,
            "manifest_status": manifest.get("stages", {}).get(stage, {}).get("status"),
            "mcap_present": bag_ok,
            "all_topics_recording": bool(topic_policy.get("recording", {}).get("record_all_topics", False)),
            "topic_verification_ok": None if verification is None else verification.get("ok"),
            "missing_required_topics": None if verification is None else verification.get("missing_or_empty_required_topics"),
            "recorded_topic_count": None if verification is None else verification.get("topic_count"),
            "exported": exported is not None,
            "accepted_trials": accepted,
            "redo_attempts": redo,
            "skipped_conditions": skipped,
        })
    report = {"session": str(session), "rows": rows}
    out = session / "analysis" / "capture_completeness_report.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
