# Steering calibration outputs and observability limits

The session is designed to identify the vehicle steering behaviour needed by an
MPC implementation. It does **not** claim to identify a physical servo's shaft
motion unless a real position/angle sensor is added.

## Delivered parameters

| Product | Output | Meaning |
|---|---|---|
| Safe servo interval | `proposed_post_session_vesc_servo_limits.yaml` | Operator-confirmed last-free endpoints with inward safety margin. |
| Raw centre | `centre_trim_offline.json` | Raw servo command at zero steady yaw/curvature. |
| Static map | `candidate_static_steering_map.json` | Raw servo to effective bicycle steering angle, including local map slope. |
| Map quality | `static_map_holdout_evaluated.parquet` | Independent hold-out error. |
| Hysteresis / backlash proxy | `static_map_hysteresis.parquet` | Difference between outward and inward approach at equal raw command. |
| Repeatability | `static_map_repeatability.parquet` | Scatter across repeated accepted captures. |
| Command-path timing | `command_to_effective_steering_response_metrics.parquet` | Raw request to selector, bus and driver-command echo timing. |
| Effective dynamics | same response table | Command-to-equivalent-steering delay, rise, settling, overshoot, effective peak rate and FOPDT fit. |
| Geometry confidence | `icp_observability_report.json` and `lidar_velocity.parquet` | LiDAR/ICP noise floor, residual, conditioning and scan-pair validity. |

## Meaning of “effective dynamics”

The dynamic output is derived as:

\[
\kappa(t)=\frac{r_{\mathrm{IMU}}(t)}{v_{x,\mathrm{LiDAR}}(t)},\qquad
\delta_{\mathrm{eq}}(t)=\arctan\left(L\kappa(t)\right).
\]

It therefore contains the combined behaviour of command transport, servo,
linkage, tyres and vehicle yaw response. This is the useful delay/rate for an
MPC plant which commands steering but observes vehicle motion.

The response fit also reports a first-order-plus-dead-time model:

\[
G(s)=\frac{K\,e^{-Ls}}{\tau s+1}.
\]

`L` is the **effective** dead time and `tau` the **effective** time constant.
They are reported by side, speed, step magnitude, repeat and return direction.

## Not identifiable without another sensor

`/sensors/servo_position_command` is a command echo, not measured servo
position. With the current sensor set, the session cannot separately identify:

- servo-shaft angle, speed or acceleration;
- road-wheel angle;
- servo electronics delay apart from ROS/VESC command transport;
- linkage backlash separately from tyre and vehicle response.

Adding a measured steering-angle sensor at the servo/rack or wheel converts
those quantities into directly identifiable actuator parameters. The current
pipeline still reports the combined vehicle-level steering dynamics needed for
control design.
