# Review fixes and enforced acceptance gates

This package incorporates the four findings raised during the source review.

## 1. Stage-0 runtime exception

`raw_command_path_audit()` now computes `max_command_path_error` from the four
measured command-chain discrepancies before printing it. The stale undefined
`max_error` name was removed. A passing Stage 0 no longer terminates the
session with `NameError`.

## 2. One LiDAR geometry source

The runtime launch no longer contains a hard-coded laser transform. `SessionRunner`
passes the immutable `calibration_config_snapshot.yaml` to the launch script;
the launch reads LiDAR IP, base/laser/IMU frame IDs, and laser x/y/z/yaw from
that snapshot. Offline ICP is invoked with the same snapshot.

The planar LiDAR-to-base conversion now applies the configured yaw as well as
x/y translation. The z coordinate is recorded for TF evidence but has no
role in the 2-D ICP correction.

Per-launch geometry evidence is saved under:

```text
environment/launch_XX_geometry.yaml
```

## 3. Hold-out is a real deployment gate

`analysis/fit_static_map.py` now writes `static_map_validation.json` and sets:

```yaml
accepted_for_deployment: true | false
```

It fails the analysis pipeline when any configured gate fails:

- accepted training count,
- accepted hold-out count,
- hold-out RMSE,
- absolute hold-out bias,
- median approach-direction hysteresis,
- median repeated-capture standard deviation.

The raw bags, candidate map, and validation diagnostics are still retained
when it fails. No parameter file is written or installed automatically.

## 4. Centre-search candidate gate is active

The centre search requires both:

- a sufficiently narrow raw-servo sign bracket, and
- an observed candidate with median absolute yaw rate within the configured
  `max_abs_yaw_rate_for_candidate_rad_s` limit.

The final centre confirmations are also a hard gate. A failure preserves the
bag but stops the session before later stages use an unreliable centre.

## Regression check

Run:

```bash
python3 tests/review_regression_checks.py
```

This is a source-level regression check. The full test requirement remains a
stand-only hardware dry run followed by review of the recorded MCAP evidence.

## 5. Static-map grouping and coverage are keyed by commanded condition

Static-map repeatability and hysteresis are now keyed by the **nominal commanded
condition**, never by an exact floating-point command echo:

```text
side + configured safe-span fraction + approach
```

`raw_servo_echo` remains a measured quantity within each group. The candidate
map uses the median echo of each nominal side/fraction condition as its
interpolation coordinate; individual hold-out points are then evaluated against
the map using their measured echo.

This prevents small run-to-run echo variation from splitting repeated
observations into single-sample buckets or preventing outward/inward pairing.

The validation is also stricter. The offline analysis now writes:

```text
analysis/static_map_condition_coverage.parquet
analysis/static_map_nominal_condition_summary.parquet
```

and fails when the thorough-profile coverage is incomplete:

- training: four accepted captures for every `side × training fraction × {outward,inward}` condition;
- hold-out: three accepted captures for every `side × validation fraction × shuffled` condition.

Consequently, `NaN` repeatability or hysteresis is never interpreted as a
passing quality metric. It is an explicit coverage failure.

## 6. Stale configuration/documentation removed

`max_center_servo_spread` was removed because confirmation passes all use the
same centre command, making a *servo-command spread* neither measured nor
meaningful. Centre quality is instead gated by the observed yaw-rate criteria
that the runtime actually evaluates.

`measurements_per_side` was removed because Stage 2 intentionally uses one
operator-confirmed last-free endpoint per side; the field was not part of the
runtime behaviour.

The README now matches the configured five response repetitions.

## Additional regression check

Run:

```bash
python3 tests/static_map_condition_grouping_checks.py
```

It creates deliberately jittered command-echo values for repeated nominal
conditions and verifies finite repeatability, paired hysteresis, complete
condition coverage, and the absence of stale configuration fields.
