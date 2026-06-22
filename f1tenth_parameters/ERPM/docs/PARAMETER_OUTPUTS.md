# Outputs and acceptance evidence

The top-level session contains one separate MCAP bag per stage. Every bag
contains all visible and hidden ROS topics, raw LaserScan arrays, TF,
parameter-events, diagnostics, selected motor commands, raw motor commands,
VESC telemetry, IMU, odometry, drive messages and structured trial events.

The offline analysis creates:

```text
analysis/
  capture_completeness_report.yaml
  erpm_map_training_trials.parquet
  erpm_map_holdout_trials.parquet
  erpm_map_training_coverage.parquet
  erpm_map_holdout_coverage.parquet
  erpm_map_nominal_condition_summary.parquet
  vel_to_erpm_pipeline_audit_trials.parquet
  vel_to_erpm_pipeline_coverage.parquet
  erpm_speed_map_report.yaml
  coastdown_samples.parquet
  coastdown_drag_report.yaml
  current_model_training_trials.parquet
  current_model_holdout_trials.parquet
  current_acceleration_report.yaml
  erpm_response_trials.parquet
  erpm_response_report.yaml
  accel_to_current_interface_trials.parquet
  accel_to_current_interface_report.yaml
  voltage_temperature_report.yaml
  candidate_vesc_patch.yaml
  longitudinal_candidate_summary.yaml
  candidate_velocity_verification_trials.parquet
  candidate_accel_verification_trials.parquet
  candidate_deployment_verification_report.yaml
```

`candidate_vesc_patch.yaml` is applied only inside the temporary candidate
verification portion of the run. The transaction always restores the exact
original source `vesc.yaml` afterwards. A technical reviewer decides whether to
install a candidate permanently.
