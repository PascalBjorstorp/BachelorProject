"""Generate the reviewable one-page-per-stage LaTeX calibration document."""
from __future__ import annotations

import math
import shutil
import subprocess
from pathlib import Path
from typing import Any

import yaml

from .stage_report import _scalar_lines


STAGE_TITLES = {
    "steering_command_audit": "Steering command-chain audit",
    "steering_centre": "Steering offset: training capture and fit",
    "steering_centre_validation": "Steering offset: independent straightness validation",
    "steering_endstops": "Human steering-limit survey",
    "steering_observability": "LiDAR/IMU observability check",
    "steering_static_training": "Servo-to-effective-steering training",
    "steering_static_holdout": "Servo-to-effective-steering hold-out validation",
    "steering_response": "Steering response: training characterisation",
    "steering_response_validation": "Steering response: independent hold-out",
    "motor_command_audit": "Motor command-chain audit",
    "longitudinal_observability": "Longitudinal sensor observability",
    "low_speed_launch": "Low-speed launch characterisation",
    "erpm_map_training": "ERPM-to-ground-speed training",
    "erpm_map_holdout": "ERPM-to-ground-speed hold-out validation",
    "vel_to_erpm_audit": "Velocity-to-ERPM command-path validation",
    "erpm_response": "ERPM response characterisation",
    "erpm_response_validation": "ERPM response independent hold-out",
    "coastdown": "Coast-down drag fit",
    "coastdown_validation": "Coast-down drag independent hold-out",
    "current_training": "Current-to-acceleration training",
    "current_holdout": "Current-to-acceleration hold-out validation",
    "accel_interface": "Acceleration-command interface training audit",
    "accel_interface_validation": "Acceleration-command interface independent hold-out",
    "odometry_candidate_velocity_validation": "Selected odometry model: fresh steady-speed validation",
    "odometry_candidate_accel_validation": "Selected odometry model: fresh transient validation",
    "physical_metrology": "Direct physical vehicle metrology",
    "lateral_stiffness_training": "Effective tyre stiffness: quasi-steady training circles",
    "lateral_stiffness_validation": "Effective tyre stiffness: independent hold-out circles",
}


def _escape(value: Any) -> str:
    text = str(value)
    replacements = {
        "\\": r"\textbackslash{}",
        "&": r"\&",
        "%": r"\%",
        "$": r"\$",
        "#": r"\#",
        "_": r"\_",
        "{": r"\{",
        "}": r"\}",
        "~": r"\textasciitilde{}",
        "^": r"\textasciicircum{}",
    }
    return "".join(replacements.get(character, character) for character in text)


def _status_text(entry: dict[str, Any] | None) -> str:
    if entry is None:
        return "pending"
    return str(entry.get("status", "unknown"))


def _workflow_text(stage_key: str) -> tuple[str, str, str]:
    if stage_key in {"steering_centre", "steering_static_training", "erpm_map_training", "current_training", "coastdown"}:
        return (
            "A — Record independent training captures.",
            "B — Fit, gate, apply the candidate patch, and rebuild inside the reversible transaction.",
            "C — The following hold-out/validation stage evaluates the applied candidate on new data.",
        )
    if stage_key in {
        "steering_centre_validation", "steering_static_holdout", "steering_response_validation",
        "erpm_map_holdout", "erpm_response_validation", "coastdown_validation", "current_holdout",
        "accel_interface_validation", "lateral_stiffness_validation",
        "odometry_candidate_velocity_validation", "odometry_candidate_accel_validation",
    }:
        return (
            "A — Record a fresh validation data set after the preceding candidate was applied.",
            "B — Analyse against the already fitted candidate; do not refit from the hold-out.",
            "C — Pass only when the new-data gates and operator acceptance agree.",
        )
    if stage_key == "steering_endstops":
        return (
            "A — Human records the last mechanically free steering limit on each side.",
            "B — The safe raw-servo envelope is stored and checked against the validated centre.",
            "C — This is a manual safety prerequisite, not a statistical fitted model.",
        )
    if stage_key == "physical_metrology":
        return (
            "A — Measure the race-ready car using the session-local physical-measurements sheet.",
            "B — Check SI units and mass/axle-load closure; derive l_f/l_r, freeze LiDAR/rear-axle/IMU reference geometry, apply the reversible dynamic-model geometry patch, and rebuild.",
            "C — This is direct metrology, not a fitted time-series model; later independent motion stages use the rebuilt geometry and it remains an audited prerequisite for lateral identification.",
        )
    if stage_key == "lateral_stiffness_training":
        return (
            "A — Record one complete, quasi-steady left/right circle for every training speed-and-steering condition after physical metrology.",
            "B — Fit effective low-slip front/rear stiffness, dynamic steering scale, and a causal cornering wheel-slip correction; apply the reversible runtime patch and retain a diagnostic nonlinear tyre candidate.",
            "C — The following distinct circles validate every frozen value and verify runtime odometry against the cornering-speed equation.",
        )
    if stage_key in {"erpm_response", "accel_interface", "steering_response"}:
        return (
            "A — Record the stage-specific candidate/training command grid.",
            "B — Estimate timing/routing metrics and preserve the candidate evidence; no hidden refit is made on later C data.",
            "C — The following explicitly named hold-out stage records different commands and evaluates the frozen result.",
        )
    if stage_key == "longitudinal_observability":
        return (
            "A — Capture a stationary diagnostic and repeated low/medium/high-speed straight passes.",
            "B — Gate the robust moving LiDAR-window measurement quality and write the stationary noise diagnostic.",
            "C — Later ERPM training is blocked unless this independent moving-sensor preflight passed.",
        )
    if stage_key == "low_speed_launch":
        return (
            "A — Capture repeated raw-ERPM launches at the low-speed grid.",
            "B — Report condition coverage and the first repeatable LiDAR ground-motion threshold; do not prematurely fit the full speed map.",
            "C — The later rebuilt VEL_TO_ERPM pipeline audit checks the resulting slow-start setting on new commands.",
        )
    return (
        "A — Record the stage-specific data.",
        "B — Run its quality analysis and preserve the result.",
        "C — Use the result as a prerequisite or independent validation for downstream fitted stages.",
    )


def _number_lines(analysis: Any) -> list[str]:
    values = _scalar_lines(analysis, limit=10)
    # Keep every stage reliably on one A4 page even when a failure message is
    # verbose.  The complete JSON/YAML remains linked from the page.
    return [value if len(value) <= 150 else value[:147] + "..." for value in values] or ["No scalar result has been written yet."]


def _statistics_lines(analysis: Any) -> list[str]:
    if not isinstance(analysis, dict):
        return ["Statistical evidence is pending."]
    statistics = analysis.get("statistical_evidence")
    if not isinstance(statistics, dict) or not statistics:
        return ["Statistical evidence is pending."]
    if statistics.get("sampling") == "direct":
        return [
            "Direct/manual measurement record.",
            "No repeated-sample standard deviation or fitted confidence interval is claimed.",
        ]
    values = [
        f"accepted rows: {statistics.get('accepted_rows', 0)} / {statistics.get('total_rows', 0)}",
        f"independent trials/units: {statistics.get('independent_units', 0)}",
    ]
    if statistics.get("condition_count") is not None:
        values.append(
            f"conditions: {statistics['condition_count']}; minimum repetitions/condition: "
            f"{statistics.get('minimum_per_condition', 'n/a')}"
        )
    headline = statistics.get("headline_distribution")
    if isinstance(headline, dict) and headline.get("field"):
        values.append(
            f"{headline['field']} median: {headline.get('median')}; bootstrap 95%: "
            f"{headline.get('bootstrap_median_95pct')}"
        )
    values.append(f"reported fit/validation accuracy metrics: {statistics.get('reported_accuracy_metric_count', 0)}")
    return values


def _stage_page(runner: Any, stage: Any, entry: dict[str, Any] | None) -> list[str]:
    session = Path(runner.session)
    title = STAGE_TITLES.get(stage.key, stage.key.replace("_", " ").title())
    status = _status_text(entry)
    a_text, b_text, c_text = _workflow_text(stage.key)
    analysis = entry.get("analysis", {}) if isinstance(entry, dict) else {}
    if isinstance(entry, dict) and entry.get("error"):
        analysis = {**(analysis if isinstance(analysis, dict) else {}), "stage_error": entry["error"]}
        if entry.get("retry_guidance"):
            analysis["retry_guidance"] = entry["retry_guidance"]
    plot = session / "plots" / "stages" / f"{stage.key}.png"
    lines = [
        r"\clearpage",
        r"\small",
        rf"\section*{{{_escape(title)}}}",
        rf"\addcontentsline{{toc}}{{section}}{{{_escape(title)}}}",
        rf"\textbf{{Stage key:}} \texttt{{{_escape(stage.key)}}}\\",
        rf"\textbf{{Status:}} {_escape(status)}\\",
        "",
        r"\subsection*{A --- Capture}",
        _escape(a_text),
        "",
        r"\subsection*{B --- Analysis and candidate update}",
        _escape(b_text),
        "",
        r"\subsection*{C --- Independent validation}",
        _escape(c_text),
        "",
        r"\subsection*{Measured / fitted values}",
        r"\begin{itemize}\setlength\itemsep{0pt}",
    ]
    lines.extend(rf"\item \texttt{{{_escape(value)}}}" for value in _number_lines(analysis))
    lines += [r"\end{itemize}", "", r"\subsection*{Statistical support}",
              r"\begin{itemize}\setlength\itemsep{0pt}"]
    lines.extend(rf"\item \texttt{{{_escape(value)}}}" for value in _statistics_lines(analysis))
    lines += [r"\end{itemize}", ""]
    if plot.exists():
        lines += [
            r"\begin{center}",
            rf"\includegraphics[width=0.78\textwidth,height=0.28\textheight,keepaspectratio]{{../plots/stages/{_escape(stage.key)}.png}}\\",
            rf"\textit{{Stage plot: {_escape(title)}}}",
            r"\end{center}",
            "",
        ]
    else:
        lines += [r"\textit{No plot has been generated for this stage yet.}", ""]
    lines += [
        r"\vfill",
        rf"\textbf{{Raw data:}} \texttt{{../{_escape(stage.directory)}/}}\\",
        rf"\textbf{{Stage report:}} \texttt{{stage\_reports/{_escape(stage.key)}.md}}\\",
        rf"\textbf{{Statistics:}} \texttt{{statistics/{_escape(stage.key)}.yaml}}",
    ]
    return lines


def _inventory_page(session: Path) -> list[str]:
    """Add a concise final page distinguishing identified and unidentifiable values."""
    path = session / "analysis" / "vehicle_parameter_inventory.yaml"
    if not path.is_file():
        return []
    try:
        inventory = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError):
        return []
    if not isinstance(inventory, dict):
        return []
    direct = inventory.get("direct_metrology", {})
    identified = inventory.get("identified_and_validated", {})
    flags = inventory.get("model_selection_flags", {})
    unavailable = inventory.get("not_identifiable_from_this_safe_room_campaign", {})
    values = _scalar_lines({"direct": direct, "identified": identified, "flags": flags}, limit=24)
    unavailable_names = list(unavailable.keys()) if isinstance(unavailable, dict) else []
    lines = [
        r"\clearpage",
        r"\small",
        r"\section*{Parameter inventory and identifiability}",
        r"\addcontentsline{toc}{section}{Parameter inventory and identifiability}",
        _escape(inventory.get("scope", "")),
        "",
        r"\subsection*{Measured and effective identified values}",
        r"\begin{itemize}\setlength\itemsep{0pt}",
    ]
    lines.extend(rf"\item \texttt{{{_escape(value)}}}" for value in values)
    lines += [r"\end{itemize}", "", r"\subsection*{Explicitly not claimed by this campaign}", r"\begin{itemize}\setlength\itemsep{0pt}"]
    for name in unavailable_names:
        explanation = unavailable.get(name)
        text = f"{name}: {explanation}"
        lines.append(rf"\item {_escape(text[:320] + ('...' if len(text) > 320 else ''))}")
    lines += [
        r"\end{itemize}",
        r"\vfill",
        r"\textbf{Full structured inventory:} \texttt{vehicle\_parameter\_inventory.yaml}",
    ]
    return lines


def write_latex_document(runner: Any, *, compile_pdf: bool = False) -> Path:
    """Write a self-contained LaTeX source file; optionally compile it to PDF."""
    session = Path(runner.session)
    analysis_dir = session / "analysis"
    analysis_dir.mkdir(parents=True, exist_ok=True)
    manifest = runner.manifest
    stage_entries = manifest.get("stages", {})
    lines = [
        r"\documentclass[11pt,a4paper]{article}",
        r"\usepackage[margin=20mm]{geometry}",
        r"\usepackage[T1]{fontenc}",
        r"\usepackage[utf8]{inputenc}",
        r"\usepackage{graphicx}",
        r"\usepackage{hyperref}",
        r"\usepackage{float}",
        r"\setlength{\parindent}{0pt}",
        r"\begin{document}",
        r"\begin{titlepage}",
        r"\centering",
        r"{\LARGE Unified vehicle calibration report\par}",
        r"\vspace{1cm}",
        rf"{{\large Session: \texttt{{{_escape(session.name)}}}\par}}",
        r"\vfill",
        r"This document is regenerated after every completed stage. Each following page records the capture, analysis/update, and independent-validation role of one stage.",
        r"\end{titlepage}",
        r"\tableofcontents",
        r"\clearpage",
    ]
    for stage in runner.STAGES:
        entry = stage_entries.get(stage.key)
        lines.extend(_stage_page(runner, stage, entry if isinstance(entry, dict) else None))
    lines.extend(_inventory_page(session))
    lines += [r"\clearpage", r"\end{document}", ""]
    source = analysis_dir / "vehicle_calibration_report.tex"
    source.write_text("\n".join(lines), encoding="utf-8")

    if compile_pdf and shutil.which("pdflatex"):
        log = analysis_dir / "vehicle_calibration_latex.log"
        command = ["pdflatex", "-interaction=nonstopmode", "-halt-on-error", source.name]
        with log.open("w", encoding="utf-8") as handle:
            first = subprocess.run(command, cwd=analysis_dir, stdout=handle, stderr=subprocess.STDOUT, check=False)
            second = subprocess.run(command, cwd=analysis_dir, stdout=handle, stderr=subprocess.STDOUT, check=False) if first.returncode == 0 else first
        if first.returncode != 0 or second.returncode != 0:
            raise RuntimeError(f"LaTeX compilation failed; inspect {log}")
    return source
