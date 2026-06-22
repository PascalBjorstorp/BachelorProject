# Outputs and acceptance evidence

## Core model-selection files

```text
analysis/odometry_static_training_coverage.parquet
analysis/odometry_static_holdout_coverage.parquet
analysis/odometry_model_leaderboard.parquet
analysis/odometry_model_regimes_<candidate>.parquet
analysis/odometry_model_selection_report.yaml
analysis/selected_odometry_candidate_patch.yaml
analysis/longitudinal_candidate_summary.yaml
analysis/forward_motion_summary.yaml
analysis/required_full_stack_upgrade.yaml
```

The selected candidate is scored separately in low-, medium- and high-speed,
calm/coasting, high-drive-acceleration and high-brake-acceleration regimes.
The report records whole-trajectory hold-out error; it does not mix random
samples from the same trial between training and test data.

## Supporting physical-model outputs

```text
analysis/erpm_speed_map_report.yaml
analysis/coastdown_drag_report.yaml
analysis/current_acceleration_report.yaml
analysis/longitudinal_traction_surface.yaml
analysis/traction_transient_report.yaml
analysis/erpm_response_report.yaml
analysis/accel_to_current_interface_report.yaml
```

## Live candidate verification evidence

```text
analysis/candidate_velocity_verification_coverage.parquet
analysis/candidate_accel_verification_coverage.parquet
analysis/candidate_cross_axis_coverage.parquet
analysis/candidate_cross_axis_speed_samples.parquet
analysis/candidate_deployment_verification_report.yaml
```

A candidate is rejected when any configured final condition lacks enough
accepted usable trials, or when it fails RMSE, signed-bias, p95, high-drive,
high-brake, acceleration-derivative, or non-zero-steering gates.

## Deployment meaning

`accepted_for_permanent_review: true` means the **shadow** model has passed
all physical validation. It does not mean production C++ was changed. Apply
the port contract, replay the same bags to prove numerical agreement, then
repeat final candidate stages with the C++ implementation before permanent use.


### Candidate acceleration map

`acceleration_command_model` is `scalar` only when the low-slip gain remains
adequate. Otherwise it is `traction_surface`, with separate bounded drive and
brake surface coefficients. This is tested reversibly in Stage 12; it is not
permanently installed by the calibration runner.

`forward_motion_summary.yaml` is the one-file synthesis for handoff. It
collects the selected speed-command terms, odometry terms, drag coefficients,
acceleration/current gains, traction/slip findings, response delays, and the
maximum observed straight-line drive/brake accelerations with simple `mu`
estimates from accepted Stage 8/9 runs.
