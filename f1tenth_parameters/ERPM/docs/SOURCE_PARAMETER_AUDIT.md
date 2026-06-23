# Longitudinal source-parameter audit

This is deliberately part of the ERPM campaign. A parameter is not described
as calibrated merely because it exists in `vesc.yaml`; it must be either
exercised by a stage, classified as a safety limit, or identified as unused in
the present source tree.

## Actively identified by this campaign

| Runtime responsibility | Current/required implementation | Evidence |
|---|---|---|
| Desired speed -> ERPM | New decoupled zero-intercept command model: linear, quadratic or monotone LUT | Stages 3/4, 6, 11 |
| Measured ERPM -> forward speed | New independent wheel observation plus optional adaptive/fused estimator | Stages 1, 3/4, 8/9, 11-13 |
| Static odom deadband | `speed_deadband` | Stages 1/2 plus held-out low-speed checks |
| Sensorless launch behaviour | `slow_start_threshold`, `slow_start_increment` | Stages 2/5/11 |
| Coast-down drag | `accel_drag_coulomb`, `accel_drag_viscous`, `accel_drag_quadratic` | Stage 7 |
| Low-slip acceleration scaling | `accel_to_current_gain`, `accel_to_brake_gain`, `accel_deadzone` | Stages 8-10 |
| Full-envelope acceleration command | New `acceleration_command_model` and optional traction surface | Stages 8/9/12 |
| Stop braking | `speed_to_braking_max` is verified only in its actual stop/direction-change path | Stage 5/11 stop events, not a generic fit |

## Safety limits: deliberately not inferred

The following affect whether the car can move, but this identification campaign
must not invent their safe values:

```text
max_drive_current
max_brake_current
max_regen_input_current
VESC driver current/brake/speed limits
battery-current limits
motor/FET thermal limits
firmware ERPM and duty-cycle limits
```

They are explicit reviewed ceilings supplied once by the developer. The runner
uses them to cap Stage 8/9/12 commands and records them with every bag.

## Present-code fields that do not implement a distinct longitudinal model

1. `operation_mode` must not be trusted as the mode switch by itself. Current
   `AckermannToVesc` selects `ACCEL_TO_CURRENT` when both acceleration gains
   are non-zero, otherwise it follows VEL_TO_ERPM logic. The transaction sets
   the gains and verifies live parameters; it never relies on a YAML label.

2. `speed_to_braking_gain`, `speed_to_braking_center`, and
   `speed_to_braking_min` are currently declared/read but are not used by the
   VEL_TO_ERPM callback. The active stop/direction-change path uses only
   `speed_to_braking_max`. They should be removed or implemented deliberately
   in the production port; this suite does not fabricate calibration values for
   code that does not consume them.

3. The legacy shared `speed_to_erpm_gain`, `speed_to_erpm_offset`, and
   `odom_speed_scale` coupling cannot express the selected architecture.
   `speed_to_erpm_offset` is constrained to zero. The production port must add
   distinct command-map and wheel-observation parameters.

## Acceptance rule

A final candidate report contains both the selected values and this audit
status. A parameter cannot be called "confirmed" unless the relevant code path
was exercised and its hold-out acceptance gate passed.
