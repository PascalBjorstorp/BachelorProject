"""Dependency-free regression checks for the unified calibration contract."""
from __future__ import annotations

import json
import shutil
import sys
import tempfile
import time
import unittest
import subprocess
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
sys.path.insert(0, str(ROOT))
sys.path[:0] = [
    str(REPO / "f1tenth_parameters" / "steering" / "analysis"),
    str(REPO / "f1tenth_parameters" / "steering"),
    str(REPO / "f1tenth_parameters" / "ERPM" / "analysis"),
]

# Both legacy analysis trees have an ``estimate_lidar_motion`` module.  Import
# the steering implementation before any compatibility imports can populate
# that generic module name from the ERPM tree.
from estimate_lidar_motion import capture_windows  # noqa: E402
from calibration_suite.lidar_motion import aggregate_motion_windows, select_baseline_index  # noqa: E402
from calibration_suite.latex_report import write_latex_document  # noqa: E402
from calibration_suite.metrology import analyse_measurements, ensure_measurement_sheet  # noqa: E402
from calibration_suite.runner import STAGES, VALIDATION_REDO_TARGETS, SuiteRunner  # noqa: E402
from fit_centre import bootstrap_centre_fit, evaluate_centre_fit, onboard_centre_consensus  # noqa: E402
from fit_current_model_training import _bootstrap_current_gain  # noqa: E402
from fit_lateral_stiffness import fit_cornering_longitudinal_slip, fit_front_tyre_and_steering_scale  # noqa: E402
from fit_speed_map_training import _bootstrap_origin_gain  # noqa: E402
from fit_static_map import rear_axle_velocity  # noqa: E402
from imu_bias import ImuBias  # noqa: E402
from steering_calibration.centre_guidance import choose_provisional_centre, fine_grid_targets, fit_onboard_zero  # noqa: E402
from steering_calibration.runtime import CalibrationNode as SteeringCalibrationNode  # noqa: E402
from steering_calibration.stages import _steering_capture_plan  # noqa: E402
from validate_static_map_applied import _inverse_steering_correction  # noqa: E402


class CalibrationContractTests(unittest.TestCase):
    def test_cli_requires_an_explicit_single_stage(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            result = subprocess.run(
                [
                    sys.executable, str(ROOT / "run_suite.py"), "run",
                    "--session", str(Path(temporary) / "session"), "--all",
                ],
                cwd=REPO,
                text=True,
                capture_output=True,
                check=False,
            )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("unrecognized arguments: --all", result.stderr)

    def test_stage_dependencies_are_forward_only(self) -> None:
        positions = {stage.key: index for index, stage in enumerate(STAGES)}
        self.assertEqual(len(STAGES), 28)
        self.assertEqual(len(positions), len(STAGES))
        self.assertNotIn("imu_bias", positions)
        self.assertEqual(STAGES[1].key, "physical_metrology")
        self.assertEqual(STAGES[2].key, "steering_observability")
        self.assertEqual(STAGES[3].key, "steering_centre")
        self.assertEqual(STAGES[4].key, "steering_centre_validation")
        self.assertEqual(STAGES[4].dependencies, ("steering_centre",))
        self.assertLess(positions["physical_metrology"], positions["steering_observability"])
        self.assertLess(positions["steering_observability"], positions["steering_centre"])
        self.assertEqual(STAGES[-1].key, "lateral_stiffness_validation")
        for stage in STAGES:
            for dependency in stage.dependencies:
                self.assertLess(positions[dependency], positions[stage.key])

    def test_runner_rejects_an_old_frozen_stage_sequence(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = Path(temporary) / "session"
            SuiteRunner(ROOT / "config" / "suite.yaml", session)
            manifest_path = session / "session_manifest.yaml"
            manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
            manifest["stage_order"] = manifest["stage_order"][:-1]
            manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError, "older calibration stage sequence"):
                SuiteRunner(ROOT / "config" / "suite.yaml", session)

    def test_latex_report_has_a_page_for_every_pending_stage(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            runner = SuiteRunner(ROOT / "config" / "suite.yaml", Path(temporary) / "session")
            document = write_latex_document(runner, compile_pdf=False)
            content = document.read_text(encoding="utf-8")
            self.assertGreaterEqual(content.count(r"\clearpage"), len(STAGES))
            self.assertIn("Direct physical vehicle metrology", content)
            self.assertIn("independent hold-out circles", content)

    def test_room_profile_fits_conservative_one_pass(self) -> None:
        config = yaml.safe_load((ROOT / "config" / "suite.yaml").read_text(encoding="utf-8"))
        with tempfile.TemporaryDirectory() as temporary:
            runner = SuiteRunner(ROOT / "config" / "suite.yaml", Path(temporary) / "session")
            report = runner.manifest["room_preflight"]
            recording = yaml.safe_load(
                (runner.session / "recording_policy_snapshot.yaml").read_text(encoding="utf-8")
            )
            self.assertTrue({"/tf", "/diagnostics", "/commands/motor/current", "/commands/motor/brake"}.issubset(
                set(recording["redundancy_topics"])
            ))
            self.assertEqual(report["physical_room_length_m"], 14.0)
            self.assertEqual(report["physical_room_width_m"], 14.0)
            self.assertEqual(report["wall_clearance_m"], 1.0)
            self.assertEqual(report["clear_room_length_m"], 12.0)
            self.assertEqual(report["clear_room_width_m"], 12.0)
            self.assertAlmostEqual(report["planned_straight_lane_heading_deg"], 45.0)
            self.assertAlmostEqual(report["clear_room_diagonal_m"], np.hypot(12.0, 12.0))
            self.assertGreater(report["usable_straight_m"], 16.0)
            self.assertLessEqual(report["conservative_one_pass_distance_m"], report["usable_straight_m"])
            self.assertEqual(report["conservative_braking_mps2"], 1.5)
            self.assertEqual(report["capture_room_utilization_target"], 0.82)
            self.assertLessEqual(report["highest_room_utilization"], 0.8201)
            self.assertIn("ERPM", report["worst_case"])
            self.assertEqual(report["max_room_steering_angle_rad"], config["site"]["max_room_steering_angle_rad"])
            current_case = next(case for case in report["stage_cases"] if case["name"] == "ERPM raw-current drive pulse")
            self.assertLessEqual(current_case["peak_speed_mps"], config["site"]["max_test_speed_mps"])
            straight = [
                case for case in report["stage_cases"]
                if case.get("capture_mode") == "extended_straight"
            ]
            self.assertGreater(len(straight), 20)
            self.assertTrue(all(case["capture_s"] > 1.0 for case in straight))
            low_launch = next(
                case for case in straight if case["name"] == "ERPM low-speed launch v=0.40"
            )
            fast_candidate = next(
                case for case in straight
                if case["name"] == "selected odometry velocity hold-out v=2.80"
            )
            self.assertGreater(low_launch["capture_s"], 29.0)
            self.assertGreater(low_launch["capture_s"], fast_candidate["capture_s"])

            lateral_circles = [
                case for case in report["stage_cases"]
                if case["name"].startswith("quasi-steady lateral")
            ]
            self.assertEqual(len(lateral_circles), 13)
            self.assertTrue(all(case["capture_mode"] == "full_circle" for case in lateral_circles))
            self.assertTrue(all(case["turn_arc_angle_rad"] == 2.0 * np.pi for case in lateral_circles))
            self.assertTrue(all(case["required_width_m"] <= report["usable_width_m"] for case in lateral_circles))

            static_cases = [
                case for case in report["stage_cases"]
                if case["name"].startswith("steering static map")
            ]
            self.assertEqual(len(static_cases), 11)
            shallow = next(
                case for case in static_cases
                if case["name"] == "steering static map training fraction 0.10"
            )
            self.assertEqual(shallow["capture_mode"], "bounded_arc")
            self.assertGreater(
                shallow["capture_s"],
                config["overrides"]["steering"]["static_map"]["capture_s"],
            )
            self.assertTrue(any(case["capture_mode"] == "full_circle" for case in static_cases))

            response_cases = [
                case for case in report["stage_cases"]
                if case["name"].startswith("steering response")
            ]
            self.assertEqual(len(response_cases), 7)
            self.assertTrue(all(case["capture_mode"] == "full_circle" for case in response_cases))
            for case in report["stage_cases"]:
                self.assertLessEqual(case["projected_room_length_m"], report["clear_room_length_m"])
                self.assertLessEqual(case["projected_room_width_m"], report["clear_room_width_m"])
                self.assertLessEqual(case["room_utilization"], 1.0)
                if case.get("enforce_capture_target"):
                    self.assertLessEqual(case["room_utilization"], 0.8201)
            deployed = float(runner._read_source_params()["steering_angle_to_servo_offset"])
            self.assertAlmostEqual(runner.steering_cfg["initial"]["raw_servo_seed"], deployed)
            self.assertEqual(
                runner.steering_cfg["initial"]["raw_servo_seed_provenance"],
                "deployed vesc.yaml steering_angle_to_servo_offset",
            )
            budget = runner.manifest["campaign_budget"]
            self.assertEqual(budget["nominal_driving_trials_min"], 504)
            self.assertEqual(budget["nominal_driving_trials_max"], 507)
            self.assertEqual(budget["operator_time_estimate_hours_without_rework"], [10.0, 14.0])
            self.assertEqual(budget["room_optimized_capture_allowance_hours"], 1.0)
        centre = config["overrides"]["steering"]["centre_trim"]
        self.assertEqual(centre["max_fit_extrapolation_servo"], 0.0)
        self.assertGreater(centre["capture_s"], 2.0 * centre["fit_trim_s"])
        self.assertEqual(centre["onboard_probe_offsets_servo"], [-0.03, 0.0, 0.03])
        self.assertEqual(centre["fine_grid_offsets_servo"], [-0.02, -0.01, -0.005, 0.0, 0.005, 0.01, 0.02])
        self.assertFalse(config["overrides"]["steering"]["analysis"]["imu_bias"]["apply_stationary_correction"])
        self.assertEqual(config["overrides"]["erpm"]["session"]["hard_erpm_limit"], 13500.0)
        static = config["overrides"]["steering"]["static_map"]
        self.assertGreater(static["capture_s"], 2.0 * config["overrides"]["steering"]["analysis"]["map"]["trim_s"])

    def test_every_moving_bag_has_immediate_offline_observability(self) -> None:
        steering_topics = yaml.safe_load(
            (REPO / "f1tenth_parameters" / "steering" / "config" / "topics.yaml").read_text(encoding="utf-8")
        )["required"]
        erpm_topics = yaml.safe_load(
            (REPO / "f1tenth_parameters" / "ERPM" / "config" / "topics.yaml").read_text(encoding="utf-8")
        )["required"]
        motion_core = {"/sensors/imu/raw", "/sensors/core", "/ego_racecar/odom", "/scan", "/tf_static"}
        for stage in STAGES:
            if stage.kind == "manual":
                continue
            topics = (steering_topics if stage.kind == "steering" else erpm_topics)[stage.topic_group]
            if (stage.kind == "steering" and stage.topic_group != "full_motion") or stage.topic_group == "command_audit":
                # Stationary command audits deliberately omit unnecessary motion channels.
                continue
            self.assertTrue(motion_core.issubset(topics), f"{stage.key} lacks full motion evidence")
            self.assertTrue(any("event" in topic for topic in topics), f"{stage.key} lacks event markers")

        for group in ("candidate_ackermann_vel", "candidate_ackermann_accel"):
            topics = set(erpm_topics[group])
            self.assertIn("/erpm_calibration/candidate_odom", topics)
            self.assertIn("/erpm_calibration/candidate_odom/speed_estimate", topics)

    def test_endstop_survey_cannot_exceed_the_frozen_room_turn_bound(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            runner = SuiteRunner(ROOT / "config" / "suite.yaml", Path(temporary) / "session")
            state = runner.state
            state["limits"] = {
                "observed_low_wheel_angle_rad": -0.51,
                "observed_high_wheel_angle_rad": 0.48,
            }
            runner._save_state(state)
            with self.assertRaisesRegex(RuntimeError, "outside the frozen room profile"):
                runner._validate_room()

    def test_runtime_steering_plan_uses_room_and_onboard_radius(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            runner = SuiteRunner(ROOT / "config" / "suite.yaml", Path(temporary) / "session")
            limits = {
                "observed_low_wheel_angle_rad": -0.50,
                "observed_high_wheel_angle_rad": 0.50,
            }
            fitting = _steering_capture_plan(
                runner.steering_cfg, limits, side="high_raw", fraction=0.25,
                speed_mps=0.7, minimum_capture_s=2.4,
            )
            self.assertEqual(fitting["capture_mode"], "full_circle")
            self.assertEqual(fitting["planned_revolutions"], 1.0)
            self.assertGreater(fitting["capture_s"], 20.0)

            shallow = _steering_capture_plan(
                runner.steering_cfg, limits, side="high_raw", fraction=0.10,
                speed_mps=0.7, minimum_capture_s=2.4,
            )
            self.assertEqual(shallow["capture_mode"], "bounded_arc")
            self.assertGreater(shallow["capture_s"], 2.4)

            onboard = _steering_capture_plan(
                runner.steering_cfg, limits, side="high_raw", fraction=0.25,
                speed_mps=0.7, minimum_capture_s=2.4, radius_override_m=8.0,
            )
            self.assertEqual(onboard["capture_mode"], "bounded_arc")
            self.assertIn("onboard", onboard["planning_source"])

    def test_circle_hold_stops_on_measured_complete_yaw(self) -> None:
        class Event:
            def emit(self, *_args, **_kwargs) -> None:
                pass

        class FakeNode:
            cfg = {"session": {"command_publish_hz": 500.0}}
            latest = SimpleNamespace(imu_gz=50.0)
            event = Event()

            def command(self, *_args, **_kwargs) -> None:
                pass

            def spin(self, seconds: float) -> None:
                time.sleep(min(seconds, 0.001))

            def check_safety(self, **_kwargs) -> None:
                pass

            def neutral_drive(self, *_args, **_kwargs) -> None:
                pass

        result = SteeringCalibrationNode.hold(
            FakeNode(), speed_mps=0.7, raw_servo=0.45,
            duration_s=0.05, phase="circle_test", segment_id="circle",
            capture=True, target_abs_yaw_change_rad=0.20,
            minimum_duration_s=0.002, maximum_duration_s=0.08,
        )
        self.assertTrue(result["yaw_target_reached"])
        self.assertGreaterEqual(result["integrated_abs_imu_yaw_rad"], 0.20)
        self.assertLess(result["hold_elapsed_s"], 0.05)

    def test_holdout_grids_do_not_reuse_training_conditions(self) -> None:
        config = yaml.safe_load((ROOT / "config" / "suite.yaml").read_text(encoding="utf-8"))
        steering = config["overrides"]["steering"]
        erpm = config["overrides"]["erpm"]

        centre = steering["centre_trim"]
        self.assertNotIn(float(centre["speed_mps"]), {
            float(condition["speed_mps"]) for condition in centre["validation_conditions"]
        })
        static = steering["static_map"]
        self.assertTrue(set(static["training_fractions"]).isdisjoint(static["validation_fractions"]))
        response_train = {
            (float(condition["speed_mps"]), float(target))
            for condition in steering["response"]["conditions"]
            for target in condition["target_fractions"]
        }
        response_validation = {
            (float(condition["speed_mps"]), float(target))
            for condition in steering["response"]["validation_conditions"]
            for target in condition["target_fractions"]
        }
        self.assertTrue(response_train.isdisjoint(response_validation))

        self.assertTrue(set(erpm["raw_erpm_map_training"]["nominal_speeds_mps"]).isdisjoint(
            erpm["raw_erpm_map_holdout"]["nominal_speeds_mps"]
        ))
        self.assertTrue({tuple(step) for step in erpm["raw_erpm_response"]["steps_mps"]}.isdisjoint(
            {tuple(step) for step in erpm["raw_erpm_response"]["validation_steps_mps"]}
        ))
        self.assertTrue(set(erpm["coastdown"]["initial_speeds_mps"]).isdisjoint(
            erpm["coastdown"]["validation_initial_speeds_mps"]
        ))

        def current_conditions(section: str) -> set[tuple[str, float, float]]:
            return {
                (polarity, float(condition["initial_speed_mps"]), float(fraction))
                for polarity in ("drive", "brake")
                for condition in erpm[section][f"{polarity}_conditions"]
                for fraction in condition["current_fractions"]
            }

        self.assertTrue(current_conditions("raw_current_training").isdisjoint(
            current_conditions("raw_current_holdout")
        ))
        accel_train = {
            (float(speed), float(command))
            for speed in erpm["accel_to_current_interface"]["initial_speeds_mps"]
            for command in erpm["accel_to_current_interface"]["acceleration_commands_mps2"]
        }
        accel_validation = {
            (float(speed), float(command))
            for speed in erpm["accel_to_current_interface"]["validation_initial_speeds_mps"]
            for command in erpm["accel_to_current_interface"]["validation_acceleration_commands_mps2"]
        }
        self.assertTrue(accel_train.isdisjoint(accel_validation))
        lateral = erpm["lateral_stiffness"]
        self.assertTrue(set(lateral["speeds_mps"]).isdisjoint(lateral["validation_speeds_mps"]))
        self.assertTrue(set(lateral["steering_angles_rad"]).isdisjoint(
            lateral["validation_steering_angles_rad"]
        ))

    def test_validation_redo_routes_to_a_new_fit(self) -> None:
        self.assertEqual(VALIDATION_REDO_TARGETS["steering_centre_validation"], "steering_centre")
        self.assertEqual(VALIDATION_REDO_TARGETS["steering_static_holdout"], "steering_static_training")
        self.assertEqual(VALIDATION_REDO_TARGETS["erpm_map_holdout"], "erpm_map_training")
        self.assertEqual(VALIDATION_REDO_TARGETS["erpm_response_validation"], "erpm_response")
        self.assertEqual(VALIDATION_REDO_TARGETS["coastdown_validation"], "coastdown")
        self.assertEqual(VALIDATION_REDO_TARGETS["current_holdout"], "current_training")
        self.assertEqual(VALIDATION_REDO_TARGETS["accel_interface_validation"], "accel_interface")
        self.assertEqual(VALIDATION_REDO_TARGETS["lateral_stiffness_validation"], "lateral_stiffness_training")

    def test_dynamic_traction_requirement_stops_the_campaign(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            runner = SuiteRunner(ROOT / "config" / "suite.yaml", Path(temporary) / "session")
            analysis = runner.session / "analysis"
            analysis.mkdir(exist_ok=True)
            (analysis / "current_acceleration_report.yaml").write_text(
                yaml.safe_dump({"accepted_for_candidate": True}), encoding="utf-8"
            )
            (analysis / "traction_transient_report.yaml").write_text(
                yaml.safe_dump({"requires_dynamic_longitudinal_slip_model": True}), encoding="utf-8"
            )
            runner._run_script = lambda *args, **kwargs: None  # type: ignore[method-assign]
            runner._validate_full_resolution_scan = lambda *args, **kwargs: {"accepted": True}  # type: ignore[method-assign]
            runner._run_lidar_motion = lambda *args, **kwargs: None  # type: ignore[method-assign]
            stage = next(stage for stage in STAGES if stage.key == "current_holdout")
            with self.assertRaisesRegex(RuntimeError, "dynamic longitudinal slip model"):
                runner._analyse(stage)

    def test_validated_static_odometry_is_converted_to_the_runtime_scale(self) -> None:
        class Transaction:
            def __init__(self) -> None:
                self.applied = None

            def apply(self, patch, _label) -> None:
                self.applied = patch

        with tempfile.TemporaryDirectory() as temporary:
            runner = SuiteRunner(ROOT / "config" / "suite.yaml", Path(temporary) / "session")
            analysis_dir = runner.session / "analysis"
            analysis_dir.mkdir(exist_ok=True)
            (analysis_dir / "odometry_model_selection_report.yaml").write_text(yaml.safe_dump({
                "selected_family": "static_linear",
                "command_map_selected": "linear",
            }), encoding="utf-8")
            (analysis_dir / "selected_odometry_candidate_patch.yaml").write_text(yaml.safe_dump({
                "vesc_to_odom_node": {"ros__parameters": {
                    "odom_wheel_model": "linear",
                    "odom_erpm_to_speed_linear": 0.00025,
                    "speed_deadband": 0.06,
                }},
            }), encoding="utf-8")
            (analysis_dir / "erpm_speed_map_report.yaml").write_text(yaml.safe_dump({
                "candidate_speed_to_erpm_gain": 4000.0,
            }), encoding="utf-8")
            transaction = Transaction()
            stage = next(item for item in STAGES if item.key == "odometry_candidate_accel_validation")
            runner._update_after_analysis(
                stage, {"odometry_candidate_accel_validation": {"accepted_for_permanent_review": True}},
                transaction,  # type: ignore[arg-type]
            )
            patch = yaml.safe_load((analysis_dir / "odometry_runtime_vesc_patch.yaml").read_text(encoding="utf-8"))
            self.assertEqual(transaction.applied, patch)
            self.assertAlmostEqual(patch["vesc_to_odom_node"]["odom_speed_scale"], 1.0)
            self.assertEqual(patch["vesc_to_odom_node"]["speed_deadband"], 0.06)

    def test_failed_gate_still_writes_its_stage_report_and_latex_page(self) -> None:
        class FakeTransaction:
            def __init__(self, *args, **kwargs) -> None:
                self.started = False

            def begin(self) -> None:
                self.started = True

            def apply(self, *args, **kwargs) -> None:
                pass

            def restore(self) -> None:
                self.started = False

        with tempfile.TemporaryDirectory() as temporary:
            runner = SuiteRunner(ROOT / "config" / "suite.yaml", Path(temporary) / "session")
            runner._launch_and_bag = (  # type: ignore[method-assign]
                lambda *args, **kwargs: (_ for _ in ()).throw(RuntimeError("synthetic gate rejection"))
            )
            import calibration_suite.runner as runner_module
            original = runner_module.ConfigManager
            runner_module.ConfigManager = FakeTransaction  # type: ignore[assignment]
            try:
                with self.assertRaisesRegex(RuntimeError, "synthetic gate rejection"):
                    runner.run_one("steering_command_audit")
            finally:
                runner_module.ConfigManager = original

            entry = runner.manifest["stages"]["steering_command_audit"]
            self.assertEqual(entry["status"], "failed")
            self.assertIn("synthetic gate rejection", entry["analysis"]["failure"])
            report = runner.session / entry["stage_report"]
            self.assertTrue(report.is_file())
            latex = (runner.session / "analysis" / "vehicle_calibration_report.tex").read_text(
                encoding="utf-8"
            )
            self.assertIn("synthetic gate rejection", latex)
            if shutil.which("pdflatex"):
                self.assertTrue(
                    (runner.session / "analysis" / "vehicle_calibration_report.pdf").is_file()
                )

    def test_redo_archives_validation_and_clears_candidate_branch(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = Path(temporary) / "session"
            runner = SuiteRunner(ROOT / "config" / "suite.yaml", session)
            manifest = runner.manifest
            manifest["stages"] = {
                "steering_command_audit": {"status": "completed", "directory": "00_command_chain_audit"},
                "physical_metrology": {"status": "completed", "directory": "00a_physical_metrology"},
                "steering_observability": {"status": "completed", "directory": "03_sensor_observability"},
                "steering_centre": {"status": "completed", "directory": "01_zero_curvature_centre"},
                "steering_centre_validation": {"status": "failed", "directory": "01a_zero_curvature_validation"},
            }
            runner._save_manifest(manifest)
            for directory in ("01_zero_curvature_centre", "01a_zero_curvature_validation"):
                path = session / directory
                path.mkdir(parents=True)
                (path / "marker.txt").write_text(directory, encoding="utf-8")
            runner.redo_from("steering_centre_validation")
            after = runner.manifest
            self.assertIn("steering_command_audit", after["stages"])
            self.assertIn("physical_metrology", after["stages"])
            self.assertIn("steering_observability", after["stages"])
            self.assertNotIn("steering_centre", after["stages"])
            self.assertNotIn("steering_centre_validation", after["stages"])
            event = after["recalibration_events"][-1]
            self.assertEqual(event["restarted_at"], "steering_centre")
            self.assertTrue((session / event["archive"] / "01_zero_curvature_centre" / "marker.txt").exists())

    def test_displacement_baseline_reaches_observable_motion(self) -> None:
        timestamps = [index * 25_000_000 for index in range(13)]  # 40 Hz through 0.30 s
        selected = select_baseline_index(
            timestamps,
            predicted_speed_mps=0.6,
            target_displacement_m=0.12,
            min_baseline_s=0.04,
            max_baseline_s=0.32,
            predicted_speed_floor_mps=0.30,
        )
        self.assertIsNotNone(selected)
        dt_s = (timestamps[-1] - timestamps[selected]) * 1e-9
        self.assertAlmostEqual(dt_s, 0.20, places=6)
        fast = select_baseline_index(
            timestamps,
            predicted_speed_mps=3.0,
            target_displacement_m=0.12,
            min_baseline_s=0.04,
            max_baseline_s=0.32,
            predicted_speed_floor_mps=0.30,
        )
        self.assertIsNotNone(fast)
        self.assertAlmostEqual((timestamps[-1] - timestamps[fast]) * 1e-9, 0.05, places=6)

    def test_steering_capture_windows_preserve_declared_speed_for_baseline_selection(self) -> None:
        """A nominal test speed may schedule pairing but cannot become ICP input."""
        import pandas as pd

        with tempfile.TemporaryDirectory() as temporary:
            derived = Path(temporary) / "derived"
            derived.mkdir()
            pd.DataFrame([
                {
                    "event": "phase_start", "capture": True, "trial_id": "trial-1",
                    "phase": "static_map_capture", "segment_id": "segment-1",
                    "bag_ns": 1_000_000_000, "speed_mps": 0.6,
                },
                {
                    "event": "phase_end", "capture": True, "trial_id": "trial-1",
                    "phase": "static_map_capture", "segment_id": "segment-1",
                    "bag_ns": 2_000_000_000,
                },
                {"event": "trial_decision", "trial_id": "trial-1", "decision": "accepted", "bag_ns": 2_100_000_000},
            ]).to_parquet(derived / "events.parquet", index=False)
            windows = capture_windows(derived)
        self.assertEqual(len(windows), 1)
        self.assertEqual(windows[0]["speed_mps"], 0.6)

    def test_lidar_pair_registrations_become_robust_window_measurements(self) -> None:
        import pandas as pd
        table = pd.DataFrame({
            "bag_ns": [100_000_000, 200_000_000, 300_000_000, 400_000_000, 500_000_000],
            "valid": [True, True, True, True, True],
            "vx": [0.9, 1.0, 1.1, 20.0, 1.0],
            "vy": [0.0, 0.01, 0.0, 0.0, 0.01],
            "yaw_rate_icp": [0.0, 0.01, 0.0, 0.0, 0.01],
            "icp_rmse_m": [0.01] * 5,
            "hessian_condition_number": [100.0] * 5,
            "yaw_seed_residual_rad": [0.0] * 5,
        })
        windows = aggregate_motion_windows(table, [{"start_ns": 0, "end_ns": 600_000_000, "trial_id": "t", "phase": "p", "segment_id": "s"}], {
            "enabled": True, "window_s": 0.50, "stride_s": 0.50, "minimum_window_s": 0.35,
            "min_valid_pairs": 3, "min_valid_fraction": 0.65,
        })
        self.assertGreaterEqual(len(windows), 1)
        self.assertTrue(bool(windows.iloc[0].valid))
        self.assertAlmostEqual(float(windows.iloc[0].vx), 1.0, places=6)

    def test_metrology_derives_cg_distances_from_axle_loads(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = Path(temporary)
            sheet = ensure_measurement_sheet(session)
            values = yaml.safe_load(sheet.read_text(encoding="utf-8"))
            values["required"]["mass_kg"].update(value=3.5)
            values["required"]["wheelbase_m"].update(value=0.324)
            values["required"]["front_axle_load_N"].update(value=18.0)
            values["required"]["rear_axle_load_N"].update(value=3.5 * 9.80665 - 18.0)
            values["required"]["lidar_to_base_x_m"].update(value=0.265)
            values["required"]["lidar_to_base_y_m"].update(value=0.0)
            values["required"]["lidar_to_base_yaw_rad"].update(value=0.0)
            values["required"]["base_link_to_rear_axle_x_m"].update(value=-0.11)
            values["required"]["base_link_to_rear_axle_y_m"].update(value=0.004)
            values["required"]["imu_to_base_x_m"].update(value=0.08)
            values["required"]["imu_to_base_y_m"].update(value=-0.006)
            values["required"]["imu_to_base_z_m"].update(value=0.071)
            values["required"]["imu_to_base_yaw_rad"].update(value=0.015)
            values["bifilar_yaw_inertia"]["rope_spacing_m"].update(value=0.40)
            values["bifilar_yaw_inertia"]["rope_length_m"].update(value=1.00)
            values["bifilar_yaw_inertia"]["period_s"].update(value=1.20)
            values["bifilar_yaw_inertia"]["repetitions"] = 20
            sheet.write_text(yaml.safe_dump(values, sort_keys=False), encoding="utf-8")
            report = analyse_measurements(session)
            self.assertTrue(report["accepted_for_lateral_identification"])
            self.assertAlmostEqual(report["cg_to_front_axle_lf_m"] + report["cg_to_rear_axle_lr_m"], 0.324)
            self.assertAlmostEqual(report["lidar_to_base"]["x_m"], 0.265)
            self.assertAlmostEqual(report["imu_to_base"]["x_m"], 0.08)
            self.assertAlmostEqual(report["imu_to_base"]["z_m"], 0.071)
            self.assertAlmostEqual(report["imu_to_base"]["yaw_rad"], 0.015)
            self.assertAlmostEqual(report["cg_in_base_link"]["x_m"], -0.11 + report["cg_to_front_axle_lf_m"])
            self.assertEqual(report["selected_yaw_inertia_source"], "derived_from_bifilar")
            self.assertGreater(report["selected_yaw_inertia_kg_m2"], 0.0)
            self.assertNotIn("stddev", json.dumps(report))

    def test_metrology_updates_frozen_geometry_before_motion(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = Path(temporary) / "session"
            runner = SuiteRunner(ROOT / "config" / "suite.yaml", session)
            sheet = session / "physical_measurements.yaml"
            values = yaml.safe_load(sheet.read_text(encoding="utf-8"))
            values["required"]["mass_kg"].update(value=3.5)
            values["required"]["wheelbase_m"].update(value=0.326)
            values["required"]["front_axle_load_N"].update(value=18.0)
            values["required"]["rear_axle_load_N"].update(value=3.5 * 9.80665 - 18.0)
            values["required"]["lidar_to_base_x_m"].update(value=0.260)
            values["required"]["lidar_to_base_y_m"].update(value=-0.004)
            values["required"]["lidar_to_base_yaw_rad"].update(value=0.006)
            values["required"]["base_link_to_rear_axle_x_m"].update(value=-0.11)
            values["required"]["base_link_to_rear_axle_y_m"].update(value=0.004)
            values["required"]["imu_to_base_x_m"].update(value=0.08)
            values["required"]["imu_to_base_y_m"].update(value=-0.006)
            values["required"]["imu_to_base_z_m"].update(value=0.071)
            values["required"]["imu_to_base_yaw_rad"].update(value=0.015)
            sheet.write_text(yaml.safe_dump(values, sort_keys=False), encoding="utf-8")
            report = runner._analyse(STAGES[1])["physical_metrology"]
            self.assertTrue(report["accepted_for_lateral_identification"])
            self.assertAlmostEqual(report["selected_yaw_inertia_kg_m2"], 0.035)
            self.assertEqual(report["selected_yaw_inertia_source"], "cad_prior")
            self.assertAlmostEqual(runner.steering_cfg["hardware"]["wheelbase_m"], 0.326)
            self.assertAlmostEqual(runner.erpm_cfg["hardware"]["laser_to_base_x_m"], 0.260)
            self.assertAlmostEqual(runner.erpm_cfg["hardware"]["laser_to_base_y_m"], -0.004)
            self.assertAlmostEqual(runner.erpm_cfg["hardware"]["base_link_to_rear_axle_x_m"], -0.11)
            self.assertAlmostEqual(runner.erpm_cfg["hardware"]["base_link_to_rear_axle_y_m"], 0.004)
            self.assertAlmostEqual(runner.erpm_cfg["hardware"]["imu_to_base_x_m"], 0.08)
            self.assertAlmostEqual(runner.erpm_cfg["hardware"]["imu_to_base_z_m"], 0.071)
            self.assertAlmostEqual(runner.erpm_cfg["hardware"]["imu_to_base_yaw_rad"], 0.015)
            self.assertAlmostEqual(runner.combined_cfg["lateral_stiffness"]["nominal_wheelbase_m"], 0.326)

            class RecordingTransaction:
                patch: dict | None = None
                label: str | None = None

                def apply(self, patch: dict, label: str) -> None:
                    self.patch = patch
                    self.label = label

            transaction = RecordingTransaction()
            runner._update_after_analysis(
                STAGES[1], {"physical_metrology": report}, transaction  # type: ignore[arg-type]
            )
            self.assertEqual(transaction.label, "build_physical_vehicle_geometry_update")
            self.assertIsNotNone(transaction.patch)
            geometry = transaction.patch["vehicle_geometry"]  # type: ignore[index]
            self.assertAlmostEqual(geometry["laser_to_base_x_m"], 0.260)
            self.assertAlmostEqual(geometry["laser_to_base_yaw_rad"], 0.006)
            self.assertAlmostEqual(geometry["imu_to_base_x_m"], 0.08)
            self.assertAlmostEqual(geometry["imu_to_base_z_m"], 0.071)
            self.assertAlmostEqual(geometry["imu_to_base_yaw_rad"], 0.015)
            self.assertAlmostEqual(geometry["base_link_to_rear_axle_x_m"], -0.11)
            self.assertAlmostEqual(
                transaction.patch["vesc_to_odom_node"]["imu_to_base_yaw_rad"], 0.015  # type: ignore[index]
            )

    def test_promoted_geometry_reaches_standard_launches(self) -> None:
        source = REPO / "f1tenth_system" / "f1tenth_stack" / "config" / "vesc.yaml"
        geometry = yaml.safe_load(source.read_text(encoding="utf-8"))["vehicle_geometry"]["ros__parameters"]
        expected = {
            "laser_to_base_x_m", "laser_to_base_y_m", "laser_to_base_z_m", "laser_to_base_yaw_rad",
            "imu_to_base_x_m", "imu_to_base_y_m", "imu_to_base_z_m", "imu_to_base_yaw_rad",
            "base_link_to_rear_axle_x_m", "base_link_to_rear_axle_y_m",
        }
        self.assertTrue(expected.issubset(geometry))
        for launch in ("bringup_launch.py", "System_launch.py"):
            text = (REPO / "f1tenth_system" / "f1tenth_stack" / "launch" / launch).read_text(encoding="utf-8")
            self.assertIn("def _geometry_defaults", text)
            self.assertIn("vehicle_geometry", text)
            for key in expected - {"base_link_to_rear_axle_x_m", "base_link_to_rear_axle_y_m"}:
                self.assertIn(f"LaunchConfiguration('{key}')", text)

    def test_known_frame_and_lidar_geometry_are_consistent(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            runner = SuiteRunner(ROOT / "config" / "suite.yaml", Path(temporary) / "session")
            sheet = yaml.safe_load(
                (runner.session / "physical_measurements.yaml").read_text(encoding="utf-8")
            )
            required = sheet["required"]
            self.assertEqual(required["base_link_to_rear_axle_x_m"]["value"], 0.0)
            self.assertEqual(required["base_link_to_rear_axle_y_m"]["value"], 0.0)
            self.assertEqual(required["lidar_to_base_x_m"]["value"], 0.265)
            self.assertEqual(required["imu_to_base_x_m"]["value"], 0.160)
            self.assertEqual(required["imu_to_base_y_m"]["value"], 0.0)
            self.assertIn("f1tenth_stack/config/vesc.yaml", sheet["measurement_context"]["sensor_geometry_source"])
            self.assertEqual(runner.steering_cfg["hardware"]["imu_to_base_x_m"], 0.160)
            self.assertEqual(runner.steering_cfg["hardware"]["imu_to_base_z_m"], 0.0703)
            self.assertEqual(runner.erpm_cfg["hardware"]["laser_to_base_x_m"], 0.265)

        sim = yaml.safe_load(
            (REPO / "f1tenth_sim" / "config" / "sim.yaml").read_text(encoding="utf-8")
        )["bridge"]["ros__parameters"]
        self.assertEqual(sim["scan_distance_to_base_link"], 0.265)
        self.assertEqual(sim["scan_beams"], 1080)
        self.assertEqual(sim["map_path"], "calibration_room_map")
        self.assertTrue(sim["headless"])
        self.assertEqual((float(sim["sx"]), float(sim["sy"])), (0.0, 0.0))
        self.assertAlmostEqual(float(sim["stheta"]), np.pi / 4.0)
        self.assertTrue((REPO / "f1tenth_sim" / "maps" / "calibration_room_map.yaml").is_file())
        self.assertTrue((REPO / "f1tenth_sim" / "maps" / "calibration_room_map.pgm").is_file())
        from PIL import Image
        room = np.asarray(Image.open(REPO / "f1tenth_sim" / "maps" / "calibration_room_map.pgm"))
        self.assertEqual(room.shape, (140, 140))
        # At 0.1 m/pixel with origin -7.0, this is the clear central 12 x 12 m.
        self.assertTrue(np.all(room[10:130, 10:130] >= 250))
        self.assertTrue(np.all(room[:3, :] == 0))
        self.assertGreater(int(np.count_nonzero(room == 0)), 140 * 4)
        map_config = yaml.safe_load(
            (REPO / "f1tenth_sim" / "maps" / "calibration_room_map.yaml").read_text(encoding="utf-8")
        )
        self.assertEqual(map_config["origin"], [-7.0, -7.0, 0.0])
        bridge = (REPO / "f1tenth_sim" / "f1tenth_gym_ros" / "gym_bridge.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("declare_parameter('scan_distance_to_base_link', 0.265)", bridge)

        lidar_config = yaml.safe_load(
            (REPO / "f1tenth_system" / "f1tenth_lidar" / "config" / "hokuyo_ust10lx.yaml").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(lidar_config["hokuyo_scip_driver"]["ros__parameters"]["cluster"], 4)
        self.assertEqual(lidar_config["urg_node"]["ros__parameters"]["cluster"], 1)
        self.assertEqual(
            yaml.safe_load(
                (REPO / "f1tenth_localization" / "config" / "gpu_amcl_cpp_params.yaml").read_text(
                    encoding="utf-8"
                )
            )["gpu_amcl_cpp"]["ros__parameters"]["max_beams"],
            270,
        )
        for calibration_launch in (
            REPO / "f1tenth_parameters" / "steering" / "launch" / "calibration_stack.py",
            REPO / "f1tenth_parameters" / "ERPM" / "launch" / "calibration_stack.py",
        ):
            launch_text = calibration_launch.read_text(encoding="utf-8").replace(" ", "")
            self.assertIn("'cluster':1", launch_text.replace('"', "'"))
        system_launch = (
            REPO / "f1tenth_system" / "f1tenth_stack" / "launch" / "System_launch.py"
        ).read_text(encoding="utf-8")
        self.assertIn("default_value='4'", system_launch)
        self.assertIn("'scan_topic': '/scan_full'", system_launch)
        self.assertIn("'cluster': lidar_cluster_arg", system_launch)
        benchmark_launch = (
            REPO / "f1tenth_system" / "f1tenth_stack" / "launch" / "sim_amcl_benchmark.launch.py"
        ).read_text(encoding="utf-8")
        self.assertIn("'sim_scan_beams',", benchmark_launch)
        self.assertIn("'scan_beams': _int_config('sim_scan_beams')", benchmark_launch)

    def test_reduced_lidar_capture_is_rejected_before_fitting(self) -> None:
        import pandas as pd

        with tempfile.TemporaryDirectory() as temporary:
            runner = SuiteRunner(ROOT / "config" / "suite.yaml", Path(temporary) / "session")
            stage = next(item for item in STAGES if item.key == "steering_observability")
            derived = runner.session / stage.directory / "derived"
            derived.mkdir(parents=True)
            pd.DataFrame({
                "bag_ns": [0, 25_000_000, 50_000_000, 75_000_000],
                "range_count": [270, 270, 270, 270],
            }).to_parquet(derived / "scan_index.parquet", index=False)
            with self.assertRaisesRegex(RuntimeError, "full 1080-point"):
                runner._validate_full_resolution_scan(stage)
            report = json.loads(
                (runner.session / "analysis" / "steering_observability_scan_resolution_report.json").read_text(
                    encoding="utf-8"
                )
            )
            self.assertFalse(report["accepted"])
            self.assertEqual(report["minimum_ranges_per_scan"], 270)

    def test_static_map_uses_the_measured_rear_axle_velocity(self) -> None:
        vx_rear, vy_rear = rear_axle_velocity(
            vx_base=1.0, vy_base=0.0, yaw_rate=2.0,
            rear_axle_x_m=-0.11, rear_axle_y_m=0.05,
        )
        self.assertAlmostEqual(vx_rear, 0.90)
        self.assertAlmostEqual(vy_rear, -0.22)

    def test_good_centre_fit_is_accepted(self) -> None:
        x = np.array([0.48, 0.50, 0.52, 0.53, 0.54, 0.56, 0.58])
        y = -2.8 * (x - 0.53)
        result = evaluate_centre_fit(x, y, {
            "min_training_span_servo": 0.04,
            "expected_yaw_rate_slope_sign": -1,
            "min_fit_r2": 0.80,
            "max_fit_rmse_rad_s": 0.012,
            "max_fit_extrapolation_servo": 0.0,
        })
        self.assertTrue(result["accepted_for_update"])
        self.assertAlmostEqual(result["centre_servo_raw"], 0.53, places=8)

    def test_parameter_bootstraps_use_independent_trial_rows(self) -> None:
        servo = np.repeat(np.linspace(0.50, 0.56, 7), 3)
        yaw = -1.8 * (servo - 0.53) + np.tile([-0.001, 0.0, 0.001], 7)
        centre = bootstrap_centre_fit(servo, yaw, resamples=300, seed=7)
        self.assertEqual(centre["valid_resamples"], 300)
        self.assertLessEqual(centre["centre_servo_raw_95pct"][0], 0.53)
        self.assertGreaterEqual(centre["centre_servo_raw_95pct"][1], 0.53)

        speed = np.repeat(np.array([0.4, 0.8, 1.2, 1.6, 2.0]), 3)
        erpm = 4000.0 * speed + np.tile([-30.0, 0.0, 30.0], 5)
        gain = _bootstrap_origin_gain(erpm, speed, resamples=300, seed=8)
        self.assertEqual(gain["valid_resamples"], 300)
        self.assertLessEqual(gain["gain_erpm_per_mps_95pct"][0], 4000.0)
        self.assertGreaterEqual(gain["gain_erpm_per_mps_95pct"][1], 4000.0)

        import pandas as pd
        current = np.repeat(np.array([1.0, 2.0, 3.0, 4.0]), 3)
        frame = pd.DataFrame({
            "polarity": "drive",
            "selected_actuation_a": current,
            "net_accel_mps2": 0.5 * current + np.tile([-0.01, 0.0, 0.01], 4),
            "current_fraction": np.repeat([0.15, 0.25, 0.45, 0.65], 3),
            "longitudinal_slip_ratio": 0.01,
        })
        current_gain = _bootstrap_current_gain(
            frame, "drive", {"analysis": {"slip": {"high_slip_ratio_threshold": 0.2}}},
            resamples=300, seed=9,
        )
        self.assertEqual(current_gain["valid_resamples"], 300)
        self.assertLessEqual(current_gain["gain_a_per_mps2_95pct"][0], 2.0)
        self.assertGreaterEqual(current_gain["gain_a_per_mps2_95pct"][1], 2.0)

        a_lat = np.repeat(np.linspace(0.25, 1.50, 8), 3)
        wheel = np.tile([1.0, 1.3, 1.6], 8)
        slip = 0.04 * a_lat + np.tile([-0.001, 0.0, 0.001], 8)
        turn = pd.DataFrame({
            "trial_id": [f"turn_{index:02d}" for index in range(len(a_lat))],
            "turn_slip_measurement_valid": True,
            "turn_slip_lateral_accel_regressor_mps2": a_lat,
            "turn_slip_fraction": slip,
            "v_wheel_frozen_mps": wheel,
            "vx_lidar_mps": wheel * (1.0 - slip),
        })
        turn_fit = fit_cornering_longitudinal_slip(
            turn, clip_fraction=0.25, min_coefficient=0.002,
            min_improvement_fraction=0.05, bootstrap_resamples=300, seed=10,
        )
        self.assertTrue(turn_fit["correction_active"])
        self.assertLessEqual(turn_fit["coefficient_bootstrap_95pct_per_mps2"][0], 0.04)
        self.assertGreaterEqual(turn_fit["coefficient_bootstrap_95pct_per_mps2"][1], 0.04)
        self.assertLess(
            turn_fit["selected_training"]["rmse_mps"],
            turn_fit["uncorrected_training"]["rmse_mps"],
        )
        no_effect = turn.copy()
        no_effect["turn_slip_fraction"] = np.tile([-0.001, 0.0, 0.001], 8)
        no_effect["vx_lidar_mps"] = wheel * (1.0 - no_effect.turn_slip_fraction)
        zero_fit = fit_cornering_longitudinal_slip(
            no_effect, clip_fraction=0.25, min_coefficient=0.002,
            min_improvement_fraction=0.05, bootstrap_resamples=300, seed=11,
        )
        self.assertFalse(zero_fit["correction_active"])
        self.assertEqual(zero_fit["selected_coefficient_per_mps2"], 0.0)

    def test_extrapolated_centre_is_rejected(self) -> None:
        x = np.array([0.48, 0.50, 0.52, 0.54, 0.56, 0.58])
        y = -2.8 * (x - 0.62)
        result = evaluate_centre_fit(x, y, {
            "min_training_span_servo": 0.04,
            "expected_yaw_rate_slope_sign": -1,
            "min_fit_r2": 0.80,
            "max_fit_rmse_rad_s": 0.012,
            "max_fit_extrapolation_servo": 0.0,
        })
        self.assertFalse(result["accepted_for_update"])
        self.assertTrue(any("extrapolates" in failure for failure in result["failures"]))

    def test_onboard_consensus_requires_two_agreeing_sensors(self) -> None:
        accepted = {"accepted_for_update": True, "centre_servo_raw": 0.521}
        nearby = {"accepted_for_update": True, "centre_servo_raw": 0.529}
        consensus = onboard_centre_consensus(accepted, nearby, {"max_onboard_sensor_spread_servo": 0.02})
        self.assertTrue(consensus["reliable_crosscheck"])
        self.assertAlmostEqual(consensus["centre_servo_raw"], 0.525)
        far = {"accepted_for_update": True, "centre_servo_raw": 0.56}
        disagreement = onboard_centre_consensus(accepted, far, {"max_onboard_sensor_spread_servo": 0.02})
        self.assertFalse(disagreement["reliable_crosscheck"])

    def test_onboard_probe_directs_the_fine_grid(self) -> None:
        raw = [0.4813, 0.5113, 0.5413]
        imu = [-2.8 * (value - 0.532) for value in raw]
        odom = [-2.7 * (value - 0.531) for value in raw]
        config = {
            "expected_yaw_rate_slope_sign": -1,
            "min_onboard_fit_r2": 0.50,
            "max_onboard_fit_rmse_rad_s": 0.050,
            "max_onboard_probe_extrapolation_servo": 0.0,
            "max_onboard_sensor_spread_servo": 0.020,
            "max_onboard_guided_shift_servo": 0.030,
        }
        imu_fit = fit_onboard_zero(raw, imu, config)
        odom_fit = fit_onboard_zero(raw, odom, config)
        guidance = choose_provisional_centre(0.5113, imu_fit, odom_fit, config)
        self.assertEqual(guidance["source"], "imu_odom_consensus")
        self.assertAlmostEqual(guidance["centre_servo_raw"], 0.5315, places=6)
        targets = fine_grid_targets(0.5113, guidance["centre_servo_raw"], [-0.02, -0.01, 0.0, 0.01, 0.02], 0.0, 1.0)
        self.assertIn(0.5113, targets)
        self.assertTrue(any(abs(value - 0.5315) < 1e-7 for value in targets))

    def test_stationary_imu_epoch_can_be_diagnostic_without_stale_correction(self) -> None:
        bias = ImuBias(gz_intercept=0.012, ay_bias=0.08, ax_bias=-0.04,
                       model="constant", correction_applied=False)
        self.assertEqual(bias.gz_at(123), 0.0)
        self.assertEqual(float(bias.gz_at(np.array([1.0, 2.0]))[0]), 0.0)
        report = bias.to_dict()
        self.assertFalse(report["correction_applied"])
        self.assertAlmostEqual(float(report["gz_bias_intercept_rad_s"]), 0.012)

    def test_front_stiffness_fit_recovers_dynamic_steering_scale(self) -> None:
        import pandas as pd

        command = np.array([-0.16, -0.12, -0.08, -0.10, 0.08, 0.10, 0.12, 0.16, -0.14, 0.14])
        kinematic = np.array([0.012, 0.010, 0.007, 0.009, -0.007, -0.009, -0.010, -0.012, 0.011, -0.011])
        expected_scale, expected_stiffness = 0.86, 52.0
        force = expected_stiffness * (expected_scale * command + kinematic)
        frame = pd.DataFrame({
            "trial_id": [f"t{index}" for index in range(len(command))],
            "steering_angle_rad": command,
            "front_kinematic_slip_term_rad": kinematic,
            "front_lateral_force_base_N": force * np.cos(expected_scale * command),
        })
        fitted = fit_front_tyre_and_steering_scale(frame)
        self.assertAlmostEqual(float(fitted["steering_model_scale_candidate"]), expected_scale, places=3)
        self.assertAlmostEqual(float(fitted["linear"]["cornering_stiffness_N_per_rad"]), expected_stiffness, places=1)

    def test_applied_static_map_inverts_the_actual_correction(self) -> None:
        angle = np.array([-0.4, -0.1, 0.0, 0.1, 0.4])
        corrected = np.sign(angle) * (0.05 * np.abs(angle) ** 2 + np.abs(angle))
        recovered = _inverse_steering_correction(corrected, 0.05, 1.0, 0.0)
        self.assertTrue(np.allclose(recovered, angle, atol=1e-10))


if __name__ == "__main__":
    unittest.main()
