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

### Cornering longitudinal-slip correction

```text
analysis/turn_slip_report.yaml
analysis/turn_slip_coverage.parquet
analysis/turn_slip_samples.parquet
```

The driven wheels over-read ground speed while turning (real-bag analysis: the
gap grows with cornering load, sideslip is negligible, no time lag), so a single
ERPM gain is accurate on straights but not in turns. `fit_turn_slip.py` fits a
causal correction from the Stage 13 steady arcs using window-averaged LiDAR
speed as truth:

```text
a_lat  = v_wheel * |yaw_rate|
v_odom = v_wheel * (1 - clip(c1 * a_lat, 0, clip_fraction))
```

It is constrained through the origin in `a_lat`, so straight-line motion is left
exactly as the validated ERPM map predicts — the correction only removes speed
in turns. `c1` is fit on a training subset of arc cells and validated on
held-out cells (`turn_slip_report.yaml` carries train R², the held-out
uncorrected-vs-corrected RMSE/bias, coverage and the accept gates). On
acceptance, `odom_turn_slip_coeff_per_mps2` / `odom_turn_slip_clip_fraction`
are appended to `selected_odometry_candidate_patch.yaml` for the production port.

A candidate is rejected when any configured final condition lacks enough
accepted usable trials, or when it fails RMSE, signed-bias, p95, high-drive,
high-brake, acceleration-derivative, hold-speed (`a=0`), or non-zero-steering
gates.

**Hold-speed (`a=0`).** Stage 12 now includes an `acceleration_command = 0`
condition at each verification speed. Because the candidate ACCEL→current map
applies a drag feed-forward, commanding `a=0` holds speed instead of coasting;
the residual ground acceleration there is gated by
`max_hold_speed_accel_mps2`. Below `operating_envelope.hold_speed_min_mps` the
map commands zero current so the car can still come to rest.

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
