# Longitudinal tuneable-parameter coverage

This campaign is intentionally **not** a generic driving demonstration. Every
identified parameter is tied to a raw input, a non-circular reference, a
hold-out or integration check, and a candidate output. The authoritative
reference for ground speed is scan-to-scan LiDAR ICP; ERPM-derived odometry is
recorded at 200 Hz but is never used to fit itself.

| Configuration parameter | Evidence / stage | Candidate output | Confirmation |
|---|---|---|---|
| `speed_to_erpm_gain`, `speed_to_erpm_offset` | Raw selected-ERPM plateaus, Stage 3 (VESC measured ERPM retained separately for drivetrain/odometry) | `analysis/candidate_vesc_patch.yaml` | Stage 4 raw-ERPM hold-out and Stage 11 temporary VEL_TO_ERPM deployment verification |
| `odom_speed_scale` | LiDAR-vs-VESC-measured-ERPM relation over raw ERPM plateaus | Candidate Odom scale relative to the candidate ERPM map | Stage 4 held-out error and Stage 11 velocity verification |
| `vesc_to_odom_node.speed_deadband` | Stationary LiDAR velocity noise plus low-speed launch data | Candidate speed deadband | Low-speed Stage 2 and Stage 11 review |
| `slow_start_threshold`, `slow_start_increment` | Raw launch evidence plus low-speed `VEL_TO_ERPM` pipeline audit, Stages 2 and 5 | Candidate startup values | Stage 5 current pipeline audit and Stage 11 deployment verification |
| `stop_speed_deadzone` | Stationary/low-speed noise and `VEL_TO_ERPM` zero-speed transition evidence | Candidate stop deadband | VEL_TO_ERPM pipeline records and candidate verification |
| `accel_drag_coulomb`, `accel_drag_viscous`, `accel_drag_quadratic` | Current-zero coastdown, Stage 7 | Non-negative drag model | Coast-down fit R² gate and use in current-model hold-out |
| `accel_to_current_gain` | Raw drive-current pulses, Stage 8 | A/(m/s²) gain | Stage 9 independent raw-current hold-out and Stage 12 temporary ACCEL_TO_CURRENT verification |
| `accel_to_brake_gain` | Raw brake-current pulses, Stage 8 | A/(m/s²) brake gain | Stage 9 independent raw-brake hold-out and Stage 12 temporary ACCEL_TO_CURRENT verification |
| `accel_deadzone` | Ground-acceleration noise floor around current pulses | Candidate interface deadzone | Stage 10 interface residual report |
| `speed_to_braking_max` | Derived from identified brake gain and an explicit desired stop deceleration | Safe candidate stop-brake level | Candidate Stage 11/12 reports; not a thermal safety maximum |

## Recorded and audited, but not scientifically identified as “safe maxima”

The campaign logs current, voltage, motor temperature, FET temperature, duty
cycle, and VESC fault code for every test. It never claims to identify safe
values for:

```text
max_drive_current
max_brake_current
max_regen_input_current
speed_min / speed_max
current_min / current_max
brake_min / brake_max
```

These are electrical, battery, thermal, mechanical, or firmware protection
limits. Determining their **maximum safe values** needs motor/ESC/battery
specifications, thermal qualification and a separate safety test—not only a
short indoor calibration run. The output instead reports the maximum current
and temperature demanded by the calibrated manoeuvres, which establishes the
minimum capability exercised.

## Parameters not used by the present `AckermannToVesc` code path

`speed_to_braking_gain`, `speed_to_braking_center`, and `speed_to_braking_min`
are retained in the existing YAML but are not used by the current stop branch,
which applies `speed_to_braking_max` for a VEL_TO_ERPM stop/direction change.
They are recorded in the configuration snapshot and explicitly excluded from
candidate fitting rather than being given meaningless numbers.
