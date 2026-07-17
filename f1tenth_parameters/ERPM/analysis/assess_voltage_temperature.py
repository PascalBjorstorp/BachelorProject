#!/usr/bin/env python3
"""Report voltage/thermal context and speed-map residual sensitivity."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd

from common import analysis_dir, dump_yaml, load_yaml


def _range(frame: pd.DataFrame, column: str) -> list[float] | None:
    if column not in frame:
        return None
    values = frame[column].to_numpy(float)
    values = values[np.isfinite(values)]
    return [float(values.min()), float(values.max())] if len(values) else None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    output = analysis_dir(session)
    train = pd.read_parquet(output / "erpm_map_training_trials.parquet")
    hold = pd.read_parquet(output / "erpm_map_holdout_trials.parquet")
    combined = pd.concat([train.assign(split="training"), hold.assign(split="holdout")], ignore_index=True)
    speed_report = load_yaml(output / "erpm_speed_map_report.yaml")
    selected = str(speed_report.get("measured_erpm_odometry_map", {}).get("selected_model", "linear"))
    prediction = f"vx_pred_measured_{selected}_mps"
    if prediction not in combined:
        raise SystemExit(f"speed-map trial tables contain no selected prediction column {prediction}")
    combined["residual_mps"] = combined["vx_lidar_mps"].to_numpy(float) - combined[prediction].to_numpy(float)
    finite = combined.replace([np.inf, -np.inf], np.nan).dropna(subset=["battery_voltage_v", "residual_mps"])
    voltage_span = float(finite.battery_voltage_v.max() - finite.battery_voltage_v.min()) if len(finite) else math.nan
    if len(finite) >= 4 and float(finite.battery_voltage_v.std()) > 1e-6:
        slope, _ = np.polyfit(finite.battery_voltage_v.to_numpy(float), finite.residual_mps.to_numpy(float), 1)
        correlation = float(np.corrcoef(finite.battery_voltage_v, finite.residual_mps)[0, 1])
    else:
        slope = correlation = math.nan
    cfg = load_yaml(session / "calibration_config_snapshot.yaml")
    min_span = float(cfg["analysis"]["voltage_stratification_min_span_v"])
    report = {
        "samples": int(len(combined)),
        "selected_measured_erpm_model": selected,
        "residual_definition": f"vx_lidar_mps - {prediction}",
        "voltage_span_v": voltage_span,
        "minimum_voltage_span_for_sensitivity_assessment_v": min_span,
        "voltage_span_sufficient_for_sensitivity_assessment": bool(math.isfinite(voltage_span) and voltage_span >= min_span),
        "speed_map_residual_vs_voltage_slope_mps_per_v": float(slope),
        "speed_map_residual_voltage_correlation": correlation,
        "motor_temp_range_c": _range(combined, "motor_temp_c"),
        "fet_temp_range_c": _range(combined, "fet_temp_c"),
        "interpretation": (
            "Voltage and temperature are context diagnostics, not silently added to the deployed speed map. "
            "A material residual trend requests an explicit covariate model and fresh hold-out validation."
        ),
    }
    dump_yaml(output / "voltage_temperature_report.yaml", report)
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
