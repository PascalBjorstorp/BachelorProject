#!/usr/bin/env python3
"""Visualize the deterministic tuning scenarios used by tune_realistic_v2.py."""

import argparse
import importlib.util
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_module(module_path: Path):
    spec = importlib.util.spec_from_file_location("tune_realistic_v2", str(module_path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def corridor_edges(samples):
    left_x, left_y, right_x, right_y = [], [], [], []
    for wp in samples:
        normal = wp["psi"] + math.pi / 2.0
        left_x.append(wp["x"] + wp["left"] * math.cos(normal))
        left_y.append(wp["y"] + wp["left"] * math.sin(normal))
        right_x.append(wp["x"] - wp["right"] * math.cos(normal))
        right_y.append(wp["y"] - wp["right"] * math.sin(normal))
    return left_x, left_y, right_x, right_y


def min_signed_distance_to_box(samples, box):
    """Minimum signed distance from raceline samples to an oriented box.

    Negative value means at least one sample lies inside the box.
    """
    c = math.cos(box["yaw"])
    s = math.sin(box["yaw"])
    hl = box["half_length"]
    hw = box["half_width"]

    best = float("inf")
    for wp in samples:
        dx = wp["x"] - box["x"]
        dy = wp["y"] - box["y"]
        lx = dx * c + dy * s
        ly = -dx * s + dy * c

        ex = max(abs(lx) - hl, 0.0)
        ey = max(abs(ly) - hw, 0.0)
        outside = math.hypot(ex, ey)
        inside = max(abs(lx) - hl, abs(ly) - hw)
        signed = outside if inside > 0.0 else inside
        if signed < best:
            best = signed
    return best


def start_pose(sample0, env):
    lat = float(env.get("START_OFFSET_LAT", 0.0))
    dx = float(env.get("START_OFFSET_X", 0.0))
    dy = float(env.get("START_OFFSET_Y", 0.0))
    dpsi = float(env.get("START_HEADING_OFFSET", 0.0))

    normal = sample0["psi"] + math.pi / 2.0
    x = sample0["x"] + lat * math.cos(normal) + dx
    y = sample0["y"] + lat * math.sin(normal) + dy
    target_dx = sample0["x"] - x
    target_dy = sample0["y"] - y
    if math.hypot(target_dx, target_dy) > 1e-6:
        psi = math.atan2(target_dy, target_dx) + dpsi
    else:
        psi = sample0["psi"] + dpsi
    return x, y, psi


def align_samples_to_reference(samples, reference_samples):
    """Rigidly align samples to the first pose of a reference raceline."""
    if not samples or not reference_samples:
        return samples

    ref = reference_samples[0]
    cur = samples[0]
    dpsi = math.atan2(math.sin(ref["psi"] - cur["psi"]), math.cos(ref["psi"] - cur["psi"]))
    c = math.cos(dpsi)
    s = math.sin(dpsi)

    aligned = []
    for wp in samples:
        dx = wp["x"] - cur["x"]
        dy = wp["y"] - cur["y"]
        aligned.append({
            **wp,
            "x": ref["x"] + dx * c - dy * s,
            "y": ref["y"] + dx * s + dy * c,
            "psi": math.atan2(math.sin(wp["psi"] + dpsi), math.cos(wp["psi"] + dpsi)),
        })
    return aligned


def obstacle_points(samples, objects):
    track_len = max(samples[-1]["s"] - samples[0]["s"], 0.0)
    pts = []
    for obj in objects:
        target_s = samples[0]["s"] + float(obj["s_fraction"]) * track_len
        idx = min(range(len(samples)), key=lambda i: abs(samples[i]["s"] - target_s))
        wp = samples[idx]
        normal = wp["psi"] + math.pi / 2.0
        offset = float(obj["lateral_offset"])
        ox = wp["x"] + offset * math.cos(normal)
        oy = wp["y"] + offset * math.sin(normal)
        pts.append((ox, oy))
    return pts


def box_outline(box):
    """Return closed polygon points for an oriented rectangle obstacle box."""
    c = math.cos(box["yaw"])
    s = math.sin(box["yaw"])
    hl = box["half_length"]
    hw = box["half_width"]
    local = [
        (+hl, +hw),
        (+hl, -hw),
        (-hl, -hw),
        (-hl, +hw),
        (+hl, +hw),
    ]
    out = []
    for lx, ly in local:
        wx = box["x"] + lx * c - ly * s
        wy = box["y"] + lx * s + ly * c
        out.append((wx, wy))
    return out


def main():
    parser = argparse.ArgumentParser(description="Visualize MPC tuning scenarios.")
    parser.add_argument(
        "--output",
        default="test/tuning_scenarios_visualization.png",
        help="Output image path, relative to MPC folder.",
    )
    args = parser.parse_args()

    mpc_dir = Path(__file__).resolve().parents[1]
    tune_path = mpc_dir / "test" / "tune_realistic_v2.py"
    tune = load_module(tune_path)

    tune.RACELINE_PATH = str(Path(tune.RACELINE_PATH).resolve())
    tune.SCENARIO_RACELINE_PATHS = tune.build_scenario_raceline_paths(tune.RACELINE_PATH)
    scenarios = tune.build_eval_scenarios(include_obstacles=True)
    base_samples = tune.load_raceline_samples(tune.SCENARIO_RACELINE_PATHS["base"])
    base_cx = [wp["x"] for wp in base_samples]
    base_cy = [wp["y"] for wp in base_samples]
    base_lx, base_ly, base_rx, base_ry = corridor_edges(base_samples)

    scenario_order = ["race", "avoid_single", "avoid_double"]
    scenario_by_name = {s["name"]: s for s in scenarios}
    selected = [scenario_by_name[name] for name in scenario_order if name in scenario_by_name]

    fig, axes = plt.subplots(1, len(selected), figsize=(6 * len(selected), 6), constrained_layout=True)
    if len(selected) == 1:
        axes = [axes]

    for ax, scenario in zip(axes, selected):
        name = scenario["name"]
        samples = tune.load_raceline_samples(scenario["raceline_path"])
        samples = align_samples_to_reference(samples, base_samples)
        required_half_width = 0.5 * float(tune.PLANNER_CAR_WIDTH_M)
        min_wall_clearance = min(min(float(wp["left"]), float(wp["right"])) for wp in samples)

        cx = [wp["x"] for wp in samples]
        cy = [wp["y"] for wp in samples]
        slx, sly, srx, sry = corridor_edges(samples)

        # Map walls always come from the unedited baseline raceline.
        ax.plot(base_lx, base_ly, color="#111827", linewidth=1.5, label="map left wall")
        ax.plot(base_rx, base_ry, color="#111827", linewidth=1.5, label="map right wall")
        ax.plot(base_cx, base_cy, color="#9ca3af", linewidth=1.2, label="baseline raceline")

        # Scenario-adjusted walls include obstacle clipping edits.
        ax.plot(slx, sly, color="#475569", linewidth=1.0, linestyle="--", label="scenario left bound")
        ax.plot(srx, sry, color="#475569", linewidth=1.0, linestyle="--", label="scenario right bound")
        ax.plot(cx, cy, color="#0f766e", linewidth=2.2, label="scenario raceline")

        sx, sy, spsi = start_pose(samples[0], scenario["env"])
        ax.scatter([samples[0]["x"]], [samples[0]["y"]], c="#2563eb", s=30, label="raceline start")
        ax.scatter([sx], [sy], c="#dc2626", s=45, label="spawn")
        ax.arrow(
            sx,
            sy,
            0.45 * math.cos(spsi),
            0.45 * math.sin(spsi),
            width=0.015,
            color="#dc2626",
            length_includes_head=True,
            zorder=5,
        )

        profile = tune.DETERMINISTIC_OBSTACLE_PROFILES.get(name)
        min_box_clearance = None
        if profile:
            boxes = tune.build_obstacle_boxes(base_samples, profile["objects"])
            for i, box in enumerate(boxes):
                poly = box_outline(box)
                xs = [p[0] for p in poly]
                ys = [p[1] for p in poly]
                label = "obstacle box" if i == 0 else None
                ax.fill(xs, ys, color="#f59e0b", alpha=0.25, label=label)
                ax.plot(xs, ys, color="#b45309", linewidth=1.4)

                signed_d = min_signed_distance_to_box(samples, box)
                if min_box_clearance is None or signed_d < min_box_clearance:
                    min_box_clearance = signed_d

        duration = float(scenario["env"].get("SIM_DURATION", 0.0))
        weight = float(scenario.get("weight", 0.0))
        ax.set_title(f"{name} | dur={duration:.0f}s | w={weight:.2f}")
        if min_box_clearance is not None:
            ax.text(
                0.02,
                0.98,
                f"min raceline-box d = {min_box_clearance:.3f} m",
                transform=ax.transAxes,
                ha="left",
                va="top",
                fontsize=8,
                bbox={"facecolor": "white", "alpha": 0.75, "edgecolor": "#cbd5e1"},
            )
        clr_ok = min_wall_clearance >= required_half_width
        ax.text(
            0.02,
            0.90,
            f"min wall clearance = {min_wall_clearance:.3f} m (req >= {required_half_width:.3f})",
            transform=ax.transAxes,
            ha="left",
            va="top",
            fontsize=8,
            bbox={"facecolor": "#ecfdf5" if clr_ok else "#fef2f2", "alpha": 0.8, "edgecolor": "#cbd5e1"},
        )
        ax.set_aspect("equal", adjustable="box")
        ax.grid(alpha=0.2)
        ax.legend(loc="best", fontsize=8)

    fig.suptitle("MPC Tuning Scenarios: raceline, corridor, spawn, and obstacles", fontsize=13)

    output_path = (mpc_dir / args.output).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=220)

    print(str(output_path))


if __name__ == "__main__":
    main()
