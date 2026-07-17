"""Exercise every dedicated GUI plot that is not covered by fit-contract tests."""
from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

import pandas as pd
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from calibration_suite.runner import STAGE_BY_KEY  # noqa: E402
from calibration_suite.stage_report import _plot_stage  # noqa: E402
from calibration_suite.statistics import (  # noqa: E402
    CONDITION_FIELDS,
    DIRECT_STAGES,
    RUNTIME_SAMPLE_STAGES,
    TABLE_CANDIDATES,
    summarize_stage_statistics,
)


class StagePlotTests(unittest.TestCase):
    def test_gui_diagnostic_stages_generate_scientific_plots(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = Path(temporary) / "session"
            analysis = session / "analysis"
            plots = session / "plots" / "stages"
            analysis.mkdir(parents=True)

            steering_audit = STAGE_BY_KEY["steering_command_audit"]
            steering_audit_dir = session / steering_audit.directory
            steering_audit_dir.mkdir(parents=True)
            (steering_audit_dir / "runtime_result.json").write_text(json.dumps({
                "samples": [
                    {"label": "centre", "raw_servo_target": 0.52, "servo_selected_mean": 0.5201,
                     "servo_bus_mean": 0.5199, "servo_echo_mean": 0.5202},
                    {"label": "high", "raw_servo_target": 0.56, "servo_selected_mean": 0.5601,
                     "servo_bus_mean": 0.5600, "servo_echo_mean": 0.5598},
                ],
            }), encoding="utf-8")

            endstops = STAGE_BY_KEY["steering_endstops"]
            endstop_dir = session / endstops.directory
            endstop_dir.mkdir(parents=True)
            (endstop_dir / "runtime_result.json").write_text(json.dumps({
                "centre_servo_raw": 0.53, "raw_low_last_free": 0.20, "raw_high_last_free": 0.80,
                "raw_low_safe": 0.22, "raw_high_safe": 0.78, "safety_margin_servo": 0.02,
                "observed_low_wheel_angle_deg": -27.0, "observed_high_wheel_angle_deg": 25.5,
            }), encoding="utf-8")

            observability = STAGE_BY_KEY["steering_observability"]
            observability_derived = session / observability.directory / "derived"
            observability_derived.mkdir(parents=True)
            pd.DataFrame({
                "vx": [0.0, 0.8, 0.82, 0.7], "vy": [0.0, 0.01, -0.01, 0.2],
                "yaw_rate_icp": [0.0, 0.01, -0.01, 0.3], "icp_rmse_m": [0.003, 0.004, 0.005, 0.04],
                "valid": [True, True, True, False],
            }).to_parquet(observability_derived / "lidar_window_motion.parquet", index=False)

            response_rows = pd.DataFrame({
                "speed_mps": [0.8, 1.0, 1.2, 1.4], "side": ["left", "right", "left", "right"],
                "effective_response_valid": [True, True, True, True],
                "effective_delay_10pct_s": [0.08, 0.09, 0.07, 0.10],
                "effective_rise_10_90_s": [0.20, 0.22, 0.18, 0.24],
                "fopdt_tau_s": [0.10, 0.11, 0.09, 0.12],
                "fopdt_rmse_normalized": [0.04, 0.05, 0.03, 0.06],
            })
            response_rows.to_parquet(analysis / "command_to_effective_steering_response_metrics.parquet", index=False)
            response_rows.assign(speed_mps=[0.9, 1.1, 1.3, 1.5]).to_parquet(
                analysis / "validation_command_to_effective_steering_response_metrics.parquet", index=False,
            )

            pd.DataFrame({
                "raw_servo_echo": [0.51, 0.53, 0.55],
                "yaw_rate_icp_rad_s": [0.04, 0.0, -0.04],
                "yaw_rate_imu_rad_s": [0.038, 0.002, -0.039],
                "yaw_rate_odom_rad_s": [0.035, 0.001, -0.037],
            }).to_parquet(analysis / "centre_trim_points.parquet", index=False)
            (analysis / "centre_trim_offline.json").write_text(
                json.dumps({
                    "centre_servo_raw": 0.53,
                    "yaw_vs_servo_slope_rad_s_per_servo": -2.0,
                    "bootstrap": {"centre_servo_raw_95pct": [0.528, 0.532]},
                }), encoding="utf-8",
            )
            pd.DataFrame({
                "trial_id": ["centre_validation_rep_01", "centre_validation_rep_02"],
                "accepted_for_validation": [True, True],
                "yaw_rate_icp_rad_s": [0.005, -0.004], "lidar_vy_mps": [0.01, -0.01],
            }).to_parquet(analysis / "centre_validation_trials.parquet", index=False)
            static_rows = pd.DataFrame({
                "raw_servo_echo": [0.30, 0.42, 0.53, 0.64, 0.76],
                "delta_eq_rad": [0.35, 0.18, 0.0, -0.17, -0.34],
                "accepted": [True, True, True, True, True],
            })
            static_rows.to_parquet(analysis / "static_map_training_segments.parquet", index=False)
            static_rows.assign(raw_servo_echo=[0.32, 0.44, 0.53, 0.62, 0.74]).to_parquet(
                analysis / "static_map_holdout_segments.parquet", index=False,
            )
            (analysis / "candidate_static_steering_map.json").write_text(json.dumps({
                "raw_servo": static_rows.raw_servo_echo.tolist(),
                "delta_eq_rad": static_rows.delta_eq_rad.tolist(),
            }), encoding="utf-8")

            motor_audit = STAGE_BY_KEY["motor_command_audit"]
            motor_dir = session / motor_audit.directory
            motor_dir.mkdir(parents=True)
            (motor_dir / "runtime_result.json").write_text(json.dumps({
                "samples": [
                    {"label": "positive", "command_kind": "raw_erpm", "target": 500.0,
                     "selected_speed_erpm_mean": 499.0},
                    {"label": "current", "command_kind": "raw_current", "target": 1.0,
                     "selected_current_a_mean": 0.99},
                    {"label": "brake", "command_kind": "raw_brake", "target": 1.0,
                     "selected_brake_a_mean": 1.01},
                ],
            }), encoding="utf-8")

            longitudinal = pd.DataFrame({
                "speed_command_mps": [0.4, 0.8, 1.2], "vx_lidar_mps": [0.39, 0.79, 1.18],
                "lidar_valid_fraction": [0.95, 0.92, 0.90], "imu_gz_rad_s": [0.01, -0.01, 0.015],
            })
            longitudinal.to_parquet(analysis / "longitudinal_observability_trials.parquet", index=False)
            longitudinal.to_parquet(analysis / "longitudinal_observability_all_trials.parquet", index=False)

            low_speed = pd.DataFrame({
                "nominal_speed_mps": [0.10, 0.15, 0.20], "vx_lidar_mps": [0.02, 0.11, 0.18],
                "erpm_measured": [400.0, 600.0, 800.0],
            })
            low_speed.to_parquet(analysis / "low_speed_launch_trials.parquet", index=False)

            pipeline = pd.DataFrame({
                "speed_command_mps": [0.3, 0.8, 1.3], "vx_lidar_mps": [0.28, 0.78, 1.26],
                "selected_speed_erpm": [1200.0, 3200.0, 5200.0], "erpm_measured": [1180.0, 3170.0, 5150.0],
            })
            pipeline.to_parquet(analysis / "vel_to_erpm_pipeline_audit_trials.parquet", index=False)

            erpm_map = pd.DataFrame({
                "vx_lidar_mps": [0.4, 0.8, 1.2, 1.6], "erpm_measured": [1600.0, 3200.0, 4800.0, 6400.0],
            })
            erpm_map.to_parquet(analysis / "erpm_map_training_trials.parquet", index=False)
            erpm_map.assign(vx_lidar_mps=[0.5, 0.9, 1.3, 1.7],
                            erpm_measured=[2000.0, 3600.0, 5200.0, 6800.0]).to_parquet(
                analysis / "erpm_map_holdout_trials.parquet", index=False,
            )
            (analysis / "erpm_speed_map_training_report.yaml").write_text(
                yaml.safe_dump({
                    "candidate_speed_to_erpm_gain": 4000.0,
                    "candidate_speed_to_erpm_gain_bootstrap": {
                        "gain_erpm_per_mps_95pct": [3900.0, 4100.0],
                    },
                }), encoding="utf-8",
            )

            erpm_response = pd.DataFrame({
                "baseline_speed_mps": [0.3, 0.6, 0.9], "target_speed_mps": [0.8, 1.1, 1.4],
                "erpm_delay_10pct_s": [0.04, 0.05, 0.04],
                "ground_speed_delay_10pct_s": [0.12, 0.14, 0.13],
                "ground_speed_tau_s": [0.20, 0.22, 0.21],
            })
            erpm_response.to_parquet(analysis / "erpm_response_trials.parquet", index=False)
            erpm_response.assign(baseline_speed_mps=[0.4, 0.7, 1.0]).to_parquet(
                analysis / "erpm_response_validation_trials.parquet", index=False,
            )

            coast_rows = pd.DataFrame({
                "trial_id": ["coast_1"] * 5, "t_s": [0.0, 0.2, 0.4, 0.6, 0.8],
                "vx_mps": [1.5, 1.25, 1.0, 0.75, 0.5], "vx_model_mps": [1.5, 1.27, 1.02, 0.76, 0.49],
            })
            coast_rows.to_parquet(analysis / "coastdown_samples.parquet", index=False)
            coast_rows.assign(trial_id="coast_holdout").to_parquet(
                analysis / "coastdown_validation_samples.parquet", index=False,
            )
            current_rows = pd.DataFrame({
                "current_command_a": [2.0, 1.0, 1.0, 2.0],
                "selected_actuation_a": [2.0, 1.0, 1.0, 2.0],
                "net_accel_mps2": [1.2, 0.6, 0.5, 1.1],
                "polarity": ["brake", "brake", "drive", "drive"],
            })
            current_rows.to_parquet(analysis / "current_model_training_trials.parquet", index=False)
            current_rows.assign(current_command_a=[-1.8, -0.8, 0.8, 1.8]).to_parquet(
                analysis / "current_model_holdout_trials.parquet", index=False,
            )
            (analysis / "current_acceleration_training_report.yaml").write_text(yaml.safe_dump({
                "low_slip_scalar_fit": {
                    "drive": {"accel_per_amp": 0.55},
                    "brake": {"accel_per_amp": 0.60},
                },
            }), encoding="utf-8")
            accel_rows = pd.DataFrame({
                "expected_ground_accel_mps2": [-1.0, -0.5, 0.5, 1.0],
                "observed_ground_accel_mps2": [-0.9, -0.55, 0.48, 0.95],
                "route": ["brake", "brake", "drive", "drive"],
            })
            accel_rows.to_parquet(analysis / "accel_to_current_interface_trials.parquet", index=False)
            accel_rows.assign(observed_ground_accel_mps2=[-0.95, -0.45, 0.52, 1.02]).to_parquet(
                analysis / "accel_to_current_interface_validation_trials.parquet", index=False,
            )
            pd.DataFrame({
                "speed_command_mps": [0.65, 1.35, 2.35, 2.80],
                "vx_lidar_mps": [0.63, 1.33, 2.31, 2.75],
                "candidate_odom_vx_mps": [0.64, 1.34, 2.32, 2.77],
            }).to_parquet(analysis / "odometry_candidate_velocity_trials.parquet", index=False)
            pd.DataFrame({
                "vx": [0.7, 0.9, 1.2, 1.5, 1.3, 1.0],
                "candidate_vx_mps": [0.69, 0.91, 1.18, 1.48, 1.31, 1.02],
                "imu_ax": [0.6, 0.5, 0.4, 0.0, -0.5, -0.6],
            }).to_parquet(analysis / "candidate_dynamic_speed_samples.parquet", index=False)
            lateral_rows = pd.DataFrame({
                "alpha_front_rad": [-0.08, -0.04, 0.04, 0.08], "fy_front_N": [-7.5, -3.8, 3.9, 7.6],
                "alpha_rear_rad": [-0.06, -0.03, 0.03, 0.06], "fy_rear_N": [-6.0, -3.1, 3.0, 6.1],
                "measurement_valid": [True, True, True, True],
                "turn_slip_lateral_accel_regressor_mps2": [0.25, 0.50, 0.75, 1.00],
                "turn_slip_fraction": [0.011, 0.019, 0.031, 0.039],
                "turn_slip_measurement_valid": [True, True, True, True],
            })
            lateral_rows.to_parquet(analysis / "lateral_stiffness_training_trials.parquet", index=False)
            lateral_rows.assign(alpha_front_rad=[-0.07, -0.035, 0.035, 0.07]).to_parquet(
                analysis / "lateral_stiffness_validation_trials.parquet", index=False,
            )
            (analysis / "lateral_stiffness_training_report.yaml").write_text(yaml.safe_dump({
                "front_tyre": {
                    "linear": {"cornering_stiffness_N_per_rad": 95.0},
                    "runtime_bounded": {"cornering_stiffness_N_per_rad": 95.0,
                                        "pacejka_shape_factor": 1.9},
                    "nonlinear": {"cornering_stiffness_N_per_rad": 105.0,
                                  "quadratic_saturation_N_per_rad2": -100.0},
                },
                "rear_tyre": {
                    "linear": {"cornering_stiffness_N_per_rad": 100.0},
                    "runtime_bounded": {"cornering_stiffness_N_per_rad": 100.0,
                                        "pacejka_shape_factor": 1.9},
                    "nonlinear": {"cornering_stiffness_N_per_rad": 108.0,
                                  "quadratic_saturation_N_per_rad2": -120.0},
                },
                "bootstrap_linear_cornering_stiffness_95pct_N_per_rad": {
                    "front": [88.0, 103.0], "rear": [92.0, 109.0],
                },
                "cornering_longitudinal_slip": {
                    "selected_coefficient_per_mps2": 0.04,
                    "coefficient_bootstrap_95pct_per_mps2": [0.032, 0.048],
                    "clip_fraction": 0.25,
                    "correction_active": True,
                    "accepted_for_candidate": True,
                },
            }), encoding="utf-8")
            (analysis / "physical_vehicle_parameters.yaml").write_text(yaml.safe_dump({
                "mass_kg": 3.5, "wheelbase_m": 0.33, "cg_to_front_axle_lf_m": 0.17,
                "cg_to_rear_axle_lr_m": 0.16, "selected_yaw_inertia_kg_m2": 0.05,
                "lidar_to_base": {"x_m": 0.15, "y_m": 0.0},
                "rear_axle_in_base_link": {"x_m": 0.0, "y_m": 0.0},
                "imu_to_base": {"x_m": 0.16, "y_m": 0.0, "z_m": 0.0, "yaw_rad": 0.0},
                "cg_in_base_link": {"x_m": 0.16},
            }), encoding="utf-8")

            keys = tuple(STAGE_BY_KEY)
            self.assertEqual(len(keys), 28)
            self.assertEqual(
                set(keys), set(TABLE_CANDIDATES) | DIRECT_STAGES | RUNTIME_SAMPLE_STAGES,
            )
            self.assertEqual(set(keys) - DIRECT_STAGES, set(CONDITION_FIELDS))
            for key in keys:
                with self.subTest(stage=key):
                    statistics = summarize_stage_statistics(session, STAGE_BY_KEY[key], {})
                    self.assertTrue((session / statistics["artifact"]).is_file())
                    relative = _plot_stage(session, STAGE_BY_KEY[key], plots)
                    self.assertEqual(relative, f"plots/stages/{key}.png")
                    output = session / str(relative)
                    self.assertTrue(output.is_file())
                    self.assertGreater(output.stat().st_size, 1000)


if __name__ == "__main__":
    unittest.main()
