"""Validate direct physical measurements and derive bicycle-model geometry.

The calibration bags can identify actuator and effective dynamic parameters, but
they cannot manufacture a trustworthy mass, wheelbase, axle-load split or yaw
inertia.  This module makes that boundary explicit and records the measured
values in the same session/report as the dynamic fits.
"""
from __future__ import annotations

import datetime as dt
import math
import shutil
from pathlib import Path
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
TEMPLATE = ROOT / "config" / "physical_measurements.yaml"
GRAVITY_MPS2 = 9.80665

def ensure_measurement_sheet(
    session: Path,
    *,
    deployed_geometry: dict[str, float] | None = None,
    geometry_source: Path | None = None,
) -> Path:
    """Create the sheet and import missing geometry from deployed VESC config.

    Existing non-null values are never overwritten. This lets an older,
    unstarted session gain the canonical geometry automatically while
    preserving any already-recorded campaign measurements.
    """
    target = session / "physical_measurements.yaml"
    if not target.exists():
        if not TEMPLATE.is_file():
            raise FileNotFoundError(TEMPLATE)
        shutil.copy2(TEMPLATE, target)
    if deployed_geometry:
        document = _load(target)
        required = document.setdefault("required", {})
        mapping = {
            "lidar_to_base_x_m": "laser_to_base_x_m",
            "lidar_to_base_y_m": "laser_to_base_y_m",
            "lidar_to_base_yaw_rad": "laser_to_base_yaw_rad",
            "base_link_to_rear_axle_x_m": "base_link_to_rear_axle_x_m",
            "base_link_to_rear_axle_y_m": "base_link_to_rear_axle_y_m",
            "imu_to_base_x_m": "imu_to_base_x_m",
            "imu_to_base_y_m": "imu_to_base_y_m",
            "imu_to_base_z_m": "imu_to_base_z_m",
            "imu_to_base_yaw_rad": "imu_to_base_yaw_rad",
        }
        source_text = str(geometry_source.resolve()) if geometry_source else "deployed vehicle_geometry"
        changed = False
        for sheet_key, source_key in mapping.items():
            if source_key not in deployed_geometry:
                raise KeyError(f"deployed vehicle geometry is missing {source_key}")
            item = required.setdefault(sheet_key, {})
            if not isinstance(item, dict):
                raise ValueError(f"{sheet_key} is not a measurement mapping")
            if item.get("value") is None:
                item["value"] = float(deployed_geometry[source_key])
                item["method"] = f"imported from {source_text} vehicle_geometry"
                changed = True
        context = document.setdefault("measurement_context", {})
        if not isinstance(context, dict):
            raise ValueError("measurement_context is not a mapping")
        if changed or "sensor_geometry_source" not in context:
            context["sensor_geometry_source"] = source_text
            context["sensor_geometry_imported_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
            context["sensor_geometry_policy"] = (
                "authoritative deployed vehicle_geometry; no duplicate operator entry required"
            )
            changed = True
        if changed:
            _write(target, document)
    return target


def _load(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(value, dict):
        raise ValueError(f"expected YAML mapping: {path}")
    return value


def _number(section: dict[str, Any], name: str, *, required: bool,
            failures: list[str], positive: bool = True) -> tuple[float | None, float | None]:
    item = section.get(name, {})
    if not isinstance(item, dict):
        if required:
            failures.append(f"{name} is missing its measurement mapping")
        return None, None
    try:
        value = float(item.get("value"))
    except (TypeError, ValueError):
        value = math.nan
    if not math.isfinite(value) or (positive and value <= 0.0):
        if required:
            qualifier = "positive finite" if positive else "finite"
            failures.append(f"{name}.value must be a {qualifier} SI value")
        return None, None
    # Direct ruler/scale values are recorded as entered. A standard deviation
    # is neither requested nor fabricated without repeated raw measurements.
    return value, None


def _bifilar_yaw_inertia(document: dict[str, Any], mass: float | None,
                         warnings: list[str]) -> dict[str, Any]:
    """Derive yaw inertia from an optional bifilar-pendulum measurement.

    The formula is the same small-angle bifilar relation used in the supplied
    thesis.  It is deliberately optional: the quasi-steady cornering-stiffness
    fit does not require ``I_z``, while a later transient bicycle model does.
    """
    raw = document.get("bifilar_yaw_inertia", {})
    if not isinstance(raw, dict):
        return {"available": False, "reason": "bifilar_yaw_inertia is not a mapping"}
    entries = {}
    any_entered = False
    for name in ("rope_spacing_m", "rope_length_m", "period_s"):
        item = raw.get(name, {})
        if isinstance(item, dict) and item.get("value") is not None:
            any_entered = True
        entries[name] = _number(raw, name, required=False, failures=[], positive=True)
    if not any_entered:
        return {"available": False, "reason": "not entered"}
    values = {name: pair[0] for name, pair in entries.items()}
    if mass is None or any(value is None for value in values.values()):
        warnings.append("bifilar yaw-inertia data is partial; enter mass, rope spacing, rope length and period to derive I_z")
        return {"available": False, "reason": "partial measurement"}
    spacing = float(values["rope_spacing_m"])
    length = float(values["rope_length_m"])
    period = float(values["period_s"])
    inertia = mass * GRAVITY_MPS2 * spacing * spacing * period * period / (16.0 * math.pi * math.pi * length)
    try:
        repetitions = int(raw.get("repetitions"))
    except (TypeError, ValueError):
        repetitions = None
    return {
        "available": True,
        "method": "bifilar_pendulum_small_angle",
        "yaw_inertia_kg_m2": inertia,
        "rope_spacing_m": spacing,
        "rope_length_m": length,
        "period_s": period,
        "repetitions": repetitions,
        "formula": "I_z = m*g*D^2*T^2/(16*pi^2*L)",
    }


def _write(path: Path, value: dict[str, Any]) -> None:
    path.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")


def _apply_session_hardware(session: Path, hardware: dict[str, float],
                            site_update: dict[str, float]) -> list[str]:
    """Apply direct geometry only to the frozen session snapshots.

    This writes only frozen session inputs.  The runner separately builds a
    reversible dynamic-model geometry patch after this report passes, so these
    direct values are traceable without permanently mutating the source tree.
    """
    changed: list[str] = []
    for name in (
        "steering_calibration_config_snapshot.yaml",
        "erpm_calibration_config_snapshot.yaml",
        "calibration_config_snapshot.yaml",
    ):
        path = session / name
        if not path.is_file():
            continue
        cfg = _load(path)
        cfg.setdefault("hardware", {}).update(hardware)
        if name == "calibration_config_snapshot.yaml" and site_update:
            cfg.setdefault("site", {}).update(site_update)
        if name != "steering_calibration_config_snapshot.yaml":
            lateral = cfg.get("lateral_stiffness")
            if isinstance(lateral, dict):
                lateral["nominal_wheelbase_m"] = float(hardware["wheelbase_m"])
        _write(path, cfg)
        changed.append(str(path))
    return changed


def analyse_measurements(session: Path) -> dict[str, Any]:
    """Validate the session sheet and write a derived physical-parameter file."""
    sheet = ensure_measurement_sheet(session)
    document = _load(sheet)
    required = document.get("required", {})
    recommended = document.get("recommended", {})
    if not isinstance(required, dict) or not isinstance(recommended, dict):
        raise ValueError("physical measurement sheet requires required/recommended mappings")

    failures: list[str] = []
    warnings: list[str] = []
    mass, _ = _number(required, "mass_kg", required=True, failures=failures)
    wheelbase, _ = _number(required, "wheelbase_m", required=True, failures=failures)
    front, _ = _number(required, "front_axle_load_N", required=True, failures=failures)
    rear, _ = _number(required, "rear_axle_load_N", required=True, failures=failures)
    lidar_x, _ = _number(required, "lidar_to_base_x_m", required=True, failures=failures, positive=False)
    lidar_y, _ = _number(required, "lidar_to_base_y_m", required=True, failures=failures, positive=False)
    lidar_yaw, _ = _number(required, "lidar_to_base_yaw_rad", required=True, failures=failures, positive=False)
    rear_base_x, _ = _number(required, "base_link_to_rear_axle_x_m", required=True, failures=failures, positive=False)
    rear_base_y, _ = _number(required, "base_link_to_rear_axle_y_m", required=True, failures=failures, positive=False)
    imu_x, _ = _number(required, "imu_to_base_x_m", required=True, failures=failures, positive=False)
    imu_y, _ = _number(required, "imu_to_base_y_m", required=True, failures=failures, positive=False)
    imu_z, _ = _number(required, "imu_to_base_z_m", required=True, failures=failures, positive=False)
    imu_yaw, _ = _number(required, "imu_to_base_yaw_rad", required=True, failures=failures, positive=False)

    optional: dict[str, dict[str, Any]] = {}
    signed_optional = {"lidar_to_base_x_m", "lidar_to_base_y_m", "lidar_to_base_yaw_rad"}
    for key in recommended:
        value, _ = _number(
            recommended, key, required=False, failures=failures,
            positive=key not in signed_optional,
        )
        item = recommended.get(key, {})
        optional[key] = {
            "value": value,
            "source_kind": item.get("source_kind") if isinstance(item, dict) else None,
            "method": item.get("method") if isinstance(item, dict) else None,
        }

    total_load = front + rear if front is not None and rear is not None else math.nan
    expected_load = mass * GRAVITY_MPS2 if mass is not None else math.nan
    load_error = total_load - expected_load if math.isfinite(total_load) and math.isfinite(expected_load) else math.nan
    # A practical 5% closure gate catches unit/scale/setup mistakes without
    # pretending that unreported instrument precision is a measured statistic.
    load_tolerance = 0.05 * expected_load if math.isfinite(expected_load) else math.nan
    if math.isfinite(load_error) and math.isfinite(load_tolerance) and abs(load_error) > load_tolerance:
        failures.append(
            f"front + rear axle load differs from mass*g by {load_error:.2f} N "
            f"(allowed {load_tolerance:.2f} N)"
        )

    lf = lr = math.nan
    if wheelbase is not None and math.isfinite(total_load) and total_load > 0.0 and front is not None and rear is not None:
        # Static load equilibrium: F_front = m g l_r/L; F_rear = m g l_f/L.
        lf = wheelbase * rear / total_load
        lr = wheelbase * front / total_load
        if min(lf, lr) <= 0.0 or abs((lf + lr) - wheelbase) > 1e-9:
            failures.append("derived CG-to-axle distances are physically invalid")

    bifilar = _bifilar_yaw_inertia(document, mass, warnings)
    entered_yaw_inertia = optional.get("yaw_inertia_kg_m2", {}).get("value")
    entered_yaw_source_kind = optional.get("yaw_inertia_kg_m2", {}).get("source_kind")
    is_cad_yaw_prior = entered_yaw_source_kind == "cad_prior"
    yaw_inertia = entered_yaw_inertia
    yaw_inertia_source = (
        "cad_prior" if entered_yaw_inertia is not None and is_cad_yaw_prior
        else "direct_measurement" if entered_yaw_inertia is not None
        else None
    )
    if is_cad_yaw_prior and entered_yaw_inertia is not None:
        warnings.append("yaw inertia is an Inventor CAD prior; enter bifilar data to replace it with a campaign measurement")
    if (yaw_inertia is None or is_cad_yaw_prior) and bool(bifilar.get("available", False)):
        yaw_inertia = bifilar.get("yaw_inertia_kg_m2")
        yaw_inertia_source = "derived_from_bifilar"
    elif yaw_inertia is not None and bool(bifilar.get("available", False)):
        derived = float(bifilar["yaw_inertia_kg_m2"])
        direct = float(yaw_inertia)
        if abs(direct - derived) > 0.20 * max(abs(direct), abs(derived)):
            warnings.append("direct and bifilar yaw-inertia values disagree materially; inspect the pendulum geometry before using a transient model")

    hardware_update = {}
    if all(value is not None for value in (
        wheelbase, lidar_x, lidar_y, lidar_yaw, rear_base_x, rear_base_y,
        imu_x, imu_y, imu_z, imu_yaw,
    )):
        hardware_update = {
            "wheelbase_m": float(wheelbase),
            "laser_to_base_x_m": float(lidar_x),
            "laser_to_base_y_m": float(lidar_y),
            "laser_to_base_yaw_rad": float(lidar_yaw),
            "base_link_to_rear_axle_x_m": float(rear_base_x),
            "base_link_to_rear_axle_y_m": float(rear_base_y),
            "imu_to_base_x_m": float(imu_x),
            "imu_to_base_y_m": float(imu_y),
            "imu_to_base_z_m": float(imu_z),
            "imu_to_base_yaw_rad": float(imu_yaw),
        }
    site_update = {
        key: float(optional[key]["value"])
        for key in ("vehicle_length_m", "vehicle_width_m")
        if optional.get(key, {}).get("value") is not None
    }
    snapshots_updated = _apply_session_hardware(session, hardware_update, site_update) if not failures and hardware_update else []

    context = document.get("measurement_context", {})
    report = {
        "measurement_sheet": str(sheet),
        "accepted_for_lateral_identification": not failures,
        "failures": failures,
        "warnings": warnings,
        "context": context if isinstance(context, dict) else {},
        "gravity_mps2": GRAVITY_MPS2,
        "mass_kg": mass,
        "wheelbase_m": wheelbase,
        "front_axle_load_N": front,
        "rear_axle_load_N": rear,
        "axle_load_sum_N": total_load if math.isfinite(total_load) else None,
        "mass_times_gravity_N": expected_load if math.isfinite(expected_load) else None,
        "axle_load_closure_error_N": load_error if math.isfinite(load_error) else None,
        "axle_load_closure_tolerance_N": load_tolerance if math.isfinite(load_tolerance) else None,
        "cg_to_front_axle_lf_m": lf if math.isfinite(lf) else None,
        "cg_to_rear_axle_lr_m": lr if math.isfinite(lr) else None,
        "lidar_to_base": {
            "x_m": lidar_x, "y_m": lidar_y, "yaw_rad": lidar_yaw,
        },
        "rear_axle_in_base_link": {
            "x_m": rear_base_x, "y_m": rear_base_y,
        },
        "imu_to_base": {
            "x_m": imu_x, "y_m": imu_y, "z_m": imu_z,
            "z_role": "assumed CoG height for TF completeness; unused by planar calibration fits",
            "yaw_rad": imu_yaw,
        },
        "cg_in_base_link": {
            "x_m": (rear_base_x + lf) if rear_base_x is not None and math.isfinite(lf) else None,
            "y_m": rear_base_y,
            "method": "rear_axle_in_base_link + static-load-derived l_f",
        },
        "session_hardware_update": hardware_update,
        "session_site_update": site_update,
        "session_snapshot_files_updated": snapshots_updated,
        "recommended_measurements": optional,
        "bifilar_yaw_inertia": bifilar,
        "selected_yaw_inertia_kg_m2": yaw_inertia,
        "selected_yaw_inertia_source": yaw_inertia_source,
        "dynamic_bicycle_transient_ready": bool(
            yaw_inertia is not None
            and optional.get("cg_height_m", {}).get("value") is not None
        ),
        "scope": (
            "Mass, wheelbase and axle-load split are direct physical measurements. "
            "LiDAR/base, IMU/base and base/CG reference geometry are prerequisites for the effective quasi-steady tyre-stiffness fit; "
            "yaw inertia/CG height are additionally needed for a transient dynamic-bicycle fit."
        ),
    }
    analysis = session / "analysis"
    analysis.mkdir(parents=True, exist_ok=True)
    output = analysis / "physical_vehicle_parameters.yaml"
    _write(output, report)
    return report
