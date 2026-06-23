# Production longitudinal odometry port contract

The calibration suite selects a model with a Python shadow node first. It does
**not** silently assume that the current scalar production code can implement a
nonlinear or adaptive winner.

The production change is required in two nodes.

## 1. `AckermannToVesc`: command map only

The command map is *not* the odometry map. Replace the current scalar
conversion:

```cpp
ERPM = speed_to_erpm_gain * v_ref + speed_to_erpm_offset;
```

with a zero-intercept selectable model:

```cpp
ERPM_cmd(0) = 0
```

Supported modes:

```text
linear:       E = k1 v
quadratic:    E = k1 v + k2 v |v|
monotone_lut: E = interp(|v|, speed_knots, erpm_knots) sign(v)
```

Required parameters under `ackermann_to_vesc_node.ros__parameters`:

```yaml
speed_command_model: linear | quadratic | lut
speed_to_erpm_gain: <k1>
speed_to_erpm_quadratic: <k2>
speed_command_lut_speed_mps: [0.0, ...]
speed_command_lut_erpm: [0.0, ...]
```

`speed_to_erpm_offset` remains as a backward-compatible declared parameter,
but it must be ignored by the new command conversion and trigger a warning if
non-zero. Low-speed launch belongs in `slow_start_*`, not a global ERPM
intercept.

## 2. `VescToOdom`: independent wheel observation and causal fusion

Do not derive odometry from `speed_to_erpm_gain` used by the command node.
Add a `longitudinal_speed_model` parameter:

```text
legacy_scalar
static_linear
static_quadratic
static_lut
adaptive_wheel
fused_adaptive
```

The emitted patch also carries `odom_imu_ax_bias_mps2`, `odom_accel_filter_tau_s`, bounded correction terms and the fusion trust schedule.

The static wheel observation is either:

```text
v_wheel = b1 E + b2 E |E|
```

or a monotone LUT in measured ERPM. It must be exactly zero at ERPM = 0.

For `adaptive_wheel`, form a bounded correction:

```text
v_obs = v_wheel + gate(v_wheel) * clip(
  c_ap max(a_f, 0)
+ c_an max(-a_f, 0)
+ c_ip max(I_motor, 0)
+ c_ib max(-I_motor, 0)
+ c_avp max(a_f, 0)|v_wheel|
+ c_avn max(-a_f, 0)|v_wheel|,
  -limit, +limit)
```

where `gate(v) = clamp(|v| / 0.20, 0, 1)`, and `a_f` is a causal first-order
filtered IMU longitudinal acceleration after subtracting the Stage 1 stationary
parameter `odom_imu_ax_bias_mps2`.

For `fused_adaptive`, use the causal update:

```text
v_pred = v_hat_previous + dt * a_f
w = clamp(w_high + (w_coast - w_high) exp(-|a_f|/a_transition), w_min, 1)
v_hat = v_pred + w (v_obs - v_pred)
```

Required odometry parameters are exactly those emitted in
`analysis/selected_odometry_candidate_patch.yaml`.

## 3. Required debug topics

The production C++ port must publish these `std_msgs/Float64` topics at every
VESC state update:

```text
/ego_racecar/odom_debug/wheel_static
/ego_racecar/odom_debug/wheel_corrected
/ego_racecar/odom_debug/imu_ax_filtered
/ego_racecar/odom_debug/wheel_weight
/ego_racecar/odom_debug/speed_estimate
```

They are a deployment audit, not optional diagnostics.

## 4. Port acceptance test

Before replacing the shadow estimator, replay the same MCAP candidate bags
through both implementations. The production C++ port may be installed only
when, sample-for-sample after timestamp alignment:

```text
max |v_cpp - v_shadow| <= 1e-4 m/s
no non-finite output
v_cpp(ERPM=0, stationary) = 0
positive effective wheel/command-map slopes over deployed range
```

Then re-run the candidate verification stages using the C++ node, not the
shadow node. The candidate is permanently eligible only after both the shadow
and production implementations pass the same coverage, RMSE, bias and
high-demand-regime gates.


## ACCEL_TO_CURRENT traction-surface port

When `acceleration_command_model: traction_surface` is selected, port the
shadow mapper's bounded inversion exactly. For each polarity, evaluate
`a_net(I,v)=c1 I+c2 I^2+c3 I v+c4 I v^2`, construct its monotone envelope on
`[0, max_current]`, and invert that bounded envelope for the requested net
acceleration. Positive acceleration uses `a_cmd + drag(v)`; braking uses
`max(0, |a_cmd|-drag(v))`. Emit the selected drive/brake currents and debug
values as ROS topics. Do not use an unconstrained symbolic quadratic root.

