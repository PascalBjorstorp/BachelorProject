# Longitudinal parameter and full-stack coverage

## Command conversion — selected independently

| Parameter / structure | Evidence | Deployment rule |
|---|---|---|
| `speed_to_erpm_gain` | Stage 3/4 zero-intercept command hold-out | Candidate value only |
| `speed_to_erpm_quadratic` | Same, selected only when it materially improves unseen delivery error | Requires C++ map support |
| command LUT | Same | Requires C++ map support |
| `speed_to_erpm_offset` | Physical boundary condition | Forced to `0.0`; never fitted |
| `slow_start_threshold`, `slow_start_increment`, `stop_speed_deadzone` | Stage 2 + Stage 5 low-speed behaviour | Candidate values; not a substitute for an ERPM offset |

## Ground-speed estimator — selected on held-out dynamics

| Parameter / structure | Evidence |
|---|---|
| Static measured-ERPM map | Stage 3/4, independent from command-map selection |
| `odom_erpm_to_speed_linear/quadratic` or LUT | Selected wheel-speed observation |
| Stationary IMU longitudinal bias + acceleration filter time constant | Stage 1 neutral capture plus Stage 6/8/10 training versus Stage 9 whole-trajectory hold-out |
| Drive/brake acceleration/current correction terms | Same dynamic model selection |
| Fusion wheel trust schedule | High-drive/high-brake held-out ground-speed error |
| `speed_deadband` | Stationary LiDAR noise plus low-speed ladder |

## ACCEL_TO_CURRENT and traction

| Parameter / structure | Evidence |
|---|---|
| Drag coefficients | Stage 7 current-zero coast-down |
| `accel_to_current_gain`, `accel_to_brake_gain` | Low-slip subset of Stage 8, checked on Stage 9 |
| Nonlinear current/speed traction surface | Stage 8/9 full current grid; retained when scalar gains fail |
| `accel_deadzone` | Near-zero acceleration noise evidence |
| Maximum observed straight-line ground accel/decel and simple `mu` estimate | Summarized from accepted Stage 8/9 runs | Diagnostic evidence, not a firmware parameter |

A nonlinear traction surface is experimental evidence for MPC/longitudinal
model improvement. It must not be silently compressed into scalar
`ACCEL_TO_CURRENT` gains.

## Not claimed as safe maxima

The campaign does not establish VESC/battery/motor current limits, regen
limits, firmware speed bounds or thermal safety maxima. Those must be reviewed
before setting `approved_*_test_current_a`, and require a separate electrical
and thermal qualification campaign.
