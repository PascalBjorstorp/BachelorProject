# MPC Riccati-ADMM Code Review
**F1/10th Vehicle — Full Codebase Audit**
*Reviewed: mpc.c, riccati_solver.c, vehicle_model.c, util_math.c, mpc_hardware_node.c, all headers*

---

## Summary

The overall architecture is sound: a Frenet-frame MPC with an 8-state augmented formulation (5 Frenet + delta_actual + 2 previous controls), Riccati-ADMM solver, dynamic bicycle model with Pacejka-like tire linearization, and a ROS2 hardware node. The solver structure, ADMM update rules, and augmented-state rate-penalty formulation are all mathematically correct in their broad strokes.

However, there are several concrete bugs — some affecting accuracy at every single solve, others affecting safety and correctness in specific edge cases. They are organized below from most critical to least.

---

## CRITICAL — Calculation Errors That Affect Every Solve

### 1. Frenet Kinematic Jacobian Is Wrong Away From Zero Error

**Location:** `vehicle_model.c` → `vehicle_model_compute_frenet_linearization`, rows 0–1 of `state_matrix_A`

The code linearizes the Frenet kinematics assuming the operating-point heading error `e_psi_op = 0`:

```c
state_matrix_A[0][1] = time_step * vx;       // A[0][1] = dt * vx
state_matrix_A[0][3] = time_step;             // A[0][3] = dt
// A[0][2] left at zero
```

The comment acknowledges this: *"A[0][2] = 0 (∂/∂v_x = sin(e_psi) ≈ 0 at e_psi=0)"*. But the linearization operating point passed in is `lin_state = *frenet` — the **actual measured state**, which has nonzero `e_psi`. The full Jacobian at the actual operating point is:

```
e_y_dot = vx * sin(e_psi) + vy * cos(e_psi)

∂/∂e_psi = dt * (vx*cos(e_psi_op) - vy*sin(e_psi_op))   ← code has just dt*vx
∂/∂vx    = dt * sin(e_psi_op)                              ← code has 0
∂/∂vy    = dt * cos(e_psi_op)                              ← code has dt (only correct at e_psi_op=0)
```

Similarly for row 1 (`e_psi_dot`), the `(1 - kappa*e_y)` denominator is always dropped, which is wrong at large lateral errors near walls. In a race, heading errors of even 5–10° are common in aggressive cornering, and lateral errors near the wall bounds can reach 0.2–0.3 m with kappa as large as 1–2 rad/m. The linearization error for both rows is first-order in `e_psi` and `kappa*e_y`, making it non-negligible in exactly the situations the MPC is supposed to handle.

**Fix:** Evaluate rows 0–1 at the actual `frenet_state->fhead_error` and `frenet_state->flat_error`:
```c
float cp = cosf(frenet_state->fhead_error);
float sp = sinf(frenet_state->fhead_error);
float denom = 1.0f - path_curvature * frenet_state->flat_error;
if (fabsf(denom) < 1e-3f) denom = (denom >= 0.0f) ? 1e-3f : -1e-3f;

state_matrix_A[0][1] = time_step * (vx * cp - vy * sp);
state_matrix_A[0][2] = time_step * sp;
state_matrix_A[0][3] = time_step * cp;

state_matrix_A[1][2] = -time_step * path_curvature * cp / denom;
```

---

### 2. B Matrix Missing Acceleration → Lateral Dynamics Cross-Coupling

**Location:** `vehicle_model.c` → `vehicle_model_compute_frenet_linearization`, B matrix column 1

The linearized B matrix has `B[3][1] = B[4][1] = 0`, meaning the model says acceleration has zero effect on lateral velocity and yaw rate. This is wrong. Longitudinal force `Fx = m*a_cmd` changes the normal load distribution:

```
dF_zf/d(a) = -m*h / L
dF_zr/d(a) = +m*h / L
```

This changes lateral forces:

```
dF_yf/d(a) = C_Sf * (dF_zf/d(a)) * alpha_f = C_Sf * (-m*h/L) * alpha_f
dF_yr/d(a) = C_Sr * (dF_zr/d(a)) * alpha_r = C_Sr * (+m*h/L) * alpha_r
```

Which propagates into:

```
B[3][1] = dt * (dFyf/da * cos(delta) + dFyr/da) / m
B[4][1] = dt * (lf * dFyf/da * cos(delta) - lr * dFyr/da) / Iz
```

The nonlinear prediction path in `vehicle_model_predict_next_state` computes load transfer correctly, so the linearized model diverges from the predicted model under acceleration or braking. For racing with large acceleration commands, this is a meaningful mismatch.

**Fix:** Add these terms to the B column-1 computation in `vehicle_model_compute_frenet_linearization`:

```c
float dFzf_da = -(VP_MASS_KG * VP_CG_HEIGHT_M) / VP_WHEELBASE_M;
float dFzr_da = -dFzf_da;
float dFyf_da = C_Sf_Fzf / F_zf * dFzf_da * slip_terms.alpha_front;  // = C_Sf_norm * alpha_f * dFzf_da
float dFyr_da = C_Sr_Fzr / F_zr * dFzr_da * slip_terms.alpha_rear;

input_matrix_B[3][1] = time_step * ((dFyf_da * cos_delta + dFyr_da) * VP_INV_MASS_1_PER_KG);
input_matrix_B[4][1] = time_step * ((VP_CG_TO_FRONT_AXLE_M * dFyf_da * cos_delta
                                    - VP_CG_TO_REAR_AXLE_M * dFyr_da) * VP_INV_YAW_INERTIA_1_PER_KGM2);
```

---

### 3. Terminal State Uses Wrong Constraint Bounds

**Location:** `riccati_solver.c` → `riccati_solver_pass`, terminal cost initialization block

```c
const RiccatiStepData_t *last_sd = &step_data[N - 1];
// Uses last_sd to detect and apply constraints to terminal state x_N
P[s][s] = terminal_Q[s] + rho;
p[s] = terminal_q[s] - rho * (z_x[N][s] - y_x[N][s]);
```

And identically in `riccati_admm_solve`:
```c
const RiccatiStepData_t *sd = (k < N) ? &step_data[k] : &step_data[N - 1];
```

The terminal state `x_N` is being constrained with the bounds from stage `N-1`. The wall bounds for the terminal state come from `reference_trajectory[N-1]`, which is the same as stage `N-1` by construction in `mpc_compute_optimal_control`. While this happens to give the same bounds in the current implementation, it is semantically wrong: terminal state constraints should be independently declared, and future changes to either the terminal stage or the constraint setup will silently produce wrong results.

**Fix:** Pass a separate `terminal_step_data` (or terminal bounds struct) and use it explicitly for the `k = N` indexing.

---

### 4. `lf + lr ≠ Wheelbase`

**Location:** `mpc_types.h`

```c
#define VP_CG_TO_FRONT_AXLE_M 0.166f   // lf
#define VP_CG_TO_REAR_AXLE_M  0.16f    // lr
#define VP_WHEELBASE_M         0.324f   // L = lf + lr

// Actual sum: 0.166 + 0.160 = 0.326 ≠ 0.324
```

The wheelbase appears in the Ackermann feedforward `atan(L * kappa)` and in normal load formulas `VP_NORM_LOAD_F = m*g*lr/L`, `VP_NORM_LOAD_R = m*g*lf/L`. Using `L=0.324` while `lf+lr=0.326` introduces a ~0.6% inconsistency into every tire force and feedforward computation. Small in isolation, but it violates the fundamental geometric identity and will produce subtly wrong CG-forward / CG-rearward load distributions.

**Fix:** Make `VP_WHEELBASE_M` a derived constant:
```c
#define VP_CG_TO_FRONT_AXLE_M 0.166f
#define VP_CG_TO_REAR_AXLE_M  0.160f
#define VP_WHEELBASE_M (VP_CG_TO_FRONT_AXLE_M + VP_CG_TO_REAR_AXLE_M)
```

---

## SIGNIFICANT — Meaningful Accuracy or Logic Errors

### 5. `result->final_cost` Stores the Primal Residual, Not the Objective

**Location:** `mpc.c` → `mpc_compute_optimal_control`

```c
result->final_cost = riccati_sol.primal_residual;  // ← this is a residual, not cost
```

The `MpcSolverResult_t` documentation states `final_cost` is the "Final objective value reported by the solver." The primal residual is not the cost — it is the maximum constraint violation in the ADMM iterates. A user monitoring `final_cost` expecting the quadratic objective value will get nonsense. The telemetry logging in `mpc_hardware_node.c` calls it `primal_res` correctly, so at least the hardware node is honest, but any external tool using the ROS output will be misled.

**Fix:** Rename the field to `final_primal_residual` in the struct, or add a `float final_cost` field and compute the actual stage cost separately.

---

### 6. Linearization Around Zero Acceleration — Misses Feedforward Operating Point

**Location:** `mpc.c` → `mpc_compute_optimal_control`, per-step linearization loop

```c
lin_control.long_acc = 0;
```

All linearizations use zero acceleration as the operating point, regardless of the reference velocity profile. The nonlinear prediction and the linearization both compute load transfer via `vehicle_model_compute_normal_loads(Fx, ...)` with `Fx = m*a_cmd`. With `a_cmd = 0`, normal loads remain static and the tire Jacobians are computed at the static load distribution. In aggressive braking or high-speed acceleration, the actual normal load shift is significant and the linearization error compounds with the missing B[3][1]/B[4][1] terms (Issue 2).

**Fix:** Initialize `lin_control.long_acc` from the reference acceleration if available, or use a filtered estimate of the vehicle's current longitudinal demand.

---

### 7. Inconsistent Tire Models Between Prediction and Linearization

**Location:** `vehicle_model.c`, `mpc.c`

The nonlinear prediction in `vehicle_model_predict_next_state` uses a **linear tire model**:
```c
float F_yf = VP_FRICTION_COEFF * VP_FRONT_CORNERING_STIFFNESS * slip_terms.alpha_front * F_zf;
```

The linearization rollout in `mpc_predict_frenet_next_state` (called to produce the operating-point trajectory) uses the **Pacejka effective stiffness** via `vehicle_model_compute_effective_lateral_stiffness`, which saturates at high slip angles.

This creates two separate "reality" models: the nonlinear predictor and the linearization rollout disagree on tire forces at high slip angles. In hard cornering (slip angles > ~0.1 rad), the Pacejka model gives lower forces than the linear model. The MPC optimizes a plan that is inconsistent with what the physical model says will happen. The linearization should always be taken around the same tire model used for prediction.

**Fix:** Use either the linear or the Pacejka model consistently. The Pacejka model is more accurate and should be preferred for both prediction and linearization.

---

### 8. Slip Angle Jacobian Incorrect Below Velocity Floor

**Location:** `vehicle_model.c` → `vehicle_model_compute_frenet_linearization`

When `vx < MIN_SLIP_VELOCITY = 0.5 m/s`, the code applies the floor `vx_safe = 0.5` in slip angle computation but still computes Jacobians using `vx_safe`:

```c
float daf_dvx = front_num * inv_D_f;  // Uses vx_safe in denominator
```

The actual model clamps `vx_safe = MIN_SLIP_VELOCITY` regardless of `vx`, so the derivative of `alpha_f` with respect to `vx` should be **zero** when `vx < MIN_SLIP_VELOCITY` (since `vx_safe` no longer changes with `vx`). The code returns a nonzero Jacobian. This means the A matrix falsely shows velocity-slip coupling during slow maneuvers, causing the optimizer to make incorrect predictions about how velocity changes affect lateral forces at low speed.

---

### 9. `lin_state.flong_vel` Forced to Reference Velocity at Each Step

**Location:** `mpc.c` → `mpc_compute_optimal_control`

```c
lin_state = lin_state_next;
lin_state.flong_vel = reference_trajectory[k].reference_velocity;   // overrides prediction
```

After rolling out the linearization trajectory with `mpc_predict_frenet_next_state`, the velocity is overwritten with the reference velocity. If the vehicle is significantly slower than the reference (e.g., recovering from a disturbance or emerging from a corner), the linearization uses an overly optimistic velocity. This makes the steering Jacobians too large (higher velocity = stiffer Frenet coupling) and the acceleration demands too low, systematically biasing the optimizer away from the correct solution.

---

### 10. Wall Constraints Permanently Inactive in Local Raceline Mode

**Location:** `mpc_hardware_node.c` → `local_raceline_callback`

When a local raceline message arrives, waypoints are initialized with:
```c
wp->left_bound_meters = BIG_BOUND;   // 100.0 m
wp->right_bound_meters = BIG_BOUND;  // 100.0 m
```

These values flow directly into `global_reference_trajectory[step].left_wall_bound` and `right_wall_bound` via `build_reference_from_local_raceline`. The ADMM constraint on `x_k[0]` (lateral error) is then bounded by `±(100.0 - WALL_MARGIN)`, which is effectively infinite. Wall avoidance is completely disabled in local-raceline mode.

If the local raceline publisher provides wall bound data (in the position `z` channel or a separate field), it is silently ignored here.

---

## MODERATE — Logic Errors and Correctness Concerns

### 11. `MPC_W_TORQUE_RATE` Silently Overrides `MPC_W_ACCEL_RATE`

**Location:** `mpc.c` → `get_default_configuration`

```c
cfg.weight_acceleration_rate = get_env_float("MPC_W_ACCEL_RATE",   cfg.weight_acceleration_rate);
cfg.weight_acceleration_rate = get_env_float("MPC_W_TORQUE_RATE",  cfg.weight_acceleration_rate);
```

If a user sets `MPC_W_ACCEL_RATE=2.0` expecting it to take effect, but `MPC_W_TORQUE_RATE` is set to any value anywhere (e.g., in a launch file from a prior experiment), `MPC_W_TORQUE_RATE` silently wins. There is no warning. This is a silent mis-configuration trap.

**Fix:** Remove the `MPC_W_TORQUE_RATE` alias or log a deprecation warning when it is used.

---

### 12. `mpc_set_actual_previous_control` Leaves Previous Delta-Rate Stale

**Location:** `mpc.c` → `mpc_set_actual_previous_control`

```c
void mpc_set_actual_previous_control(const ControlInput_t *actual) {
    if (actual) {
        actual_steering_angle = actual->steer_ang;   // updates delta_actual ✓
        prev_control.long_acc = actual->long_acc;    // updates accel prev ✓
        // prev_control.steer_ang = ??? (delta_rate_prev — never updated from hardware)
    }
}
```

`prev_control.steer_ang` stores the previous delta-rate (steering rate command), which feeds `x0[IDX_DRATE_PREV]` in the augmented initial state. When the servo saturates (common in sharp corners), the actual delta-rate is lower than commanded, but the MPC continues to believe the full commanded rate was applied. This causes systematic error in the rate-of-rate jerk penalty and can lead to overcorrection.

---

### 13. IMU Yaw Rate Has No Staleness Check

**Location:** `mpc_hardware_node.c` → `odometry_subscription_callback`

```c
if (g_imu_received) {
    global_vehicle_state.yaw_rate = g_imu_yaw_rate;   // used even if stale
}
```

The watchdog only checks odometry staleness. If the IMU callback (`/imu/filtered_angular_velocity`) stops publishing — due to sensor failure, node crash, or network loss — the last known yaw rate is used indefinitely. For a 200 Hz controller, a stale yaw rate of even 50 ms is a significant portion of the dynamic response time.

**Fix:** Add a timestamp to `g_imu_yaw_rate` and fall back to odometry yaw rate if the IMU message is older than a threshold (e.g., 50 ms).

---

### 14. Heading Interpolation in `sample_waypoint_by_s` Not Wrapped

**Location:** `mpc_hardware_node.c` → `sample_waypoint_by_s`

```c
out->heading_radians = w0->heading_radians + (w1->heading_radians - w0->heading_radians) * t;
```

No angle wrapping is applied. At track sections crossing the ±π boundary (e.g., heading going from +3.1 to −3.1 radians), the linear interpolation will pass through 0 instead of continuing through ±π. The interpolated heading would be wildly wrong over the entire waypoint pair. The curvature derived from this heading will also be wrong.

**Fix:**
```c
double dh = w1->heading_radians - w0->heading_radians;
while (dh >  M_PI) dh -= 2*M_PI;
while (dh < -M_PI) dh += 2*M_PI;
out->heading_radians = w0->heading_radians + t * dh;
```

---

### 15. Interpolated Path Normal Not Re-Normalized

**Location:** `mpc_hardware_node.c` → `convert_to_frenet_state`

```c
double sin_h = global_trajectory[idx0].sin_heading
             + t * (global_trajectory[idx1].sin_heading - global_trajectory[idx0].sin_heading);
double cos_h = global_trajectory[idx0].cos_heading
             + t * (global_trajectory[idx1].cos_heading - global_trajectory[idx0].cos_heading);
double lateral_error = -dx * sin_h + dy * cos_h;
```

The linearly interpolated `(sin_h, cos_h)` pair does not have unit magnitude unless the two headings are identical. For adjacent waypoints separated by, say, 0.1 rad, the magnitude at `t = 0.5` is `cos(0.05) ≈ 0.9988` — tiny error. But at a sharp corner with 0.3 rad heading change between waypoints, the magnitude can drop to ~0.989, giving a signed lateral error that is systematically ~1% too small at the midpoint. Over many waypoints on a fast track, this accumulates into a small but consistent bias in the Frenet tracking error.

**Fix:** Re-normalize after interpolation: `double norm = hypot(sin_h, cos_h); sin_h /= norm; cos_h /= norm;`

---

### 16. Warm-Start Reset Condition Is Too Crude

**Location:** `mpc.c` → `mpc_compute_optimal_control`

```c
float kappa_diff = fabsf(cur_curvature - warm_start_prev_curvature);
if (!admm_state.initialized || kappa_diff > WARMSTART_CURVATURE_RESET_THRESHOLD) {
    riccati_admm_state_init(&admm_state);
}
```

`WARMSTART_CURVATURE_RESET_THRESHOLD = 0.5 rad/m` is very large — this allows warm-starts to persist through moderate corners and many meaningful track transitions. More importantly, curvature is not the only reason warm-starts become invalid: a large lateral-error jump (vehicle returning from an off-track excursion), a speed change (lap start vs. mid-lap), or a track reset (new lap or override) can all make the previous ADMM iterates a poor starting point or even divergence-inducing.

---

## MINOR — Code Quality, Documentation, and Defensive Issues

### 17. `VehicleParameters_t` Struct Is Dead Code

**Location:** `mpc_types.h`

The struct is fully defined with all vehicle parameters but is never used anywhere in the model, controller, or hardware node. All model code accesses vehicle parameters via compile-time macros (`VP_MASS_KG`, `VP_CG_TO_FRONT_AXLE_M`, etc.). There is no mechanism to pass a `VehicleParameters_t` at runtime to affect any computation. If someone assumes they can configure the vehicle by populating this struct, nothing will change.

**Fix:** Either remove the struct, or refactor the model to consume it and expose `mpc_initialize_with_vehicle_parameters(const VehicleParameters_t *)`.

---

### 18. `RICCATI_COST_FACTOR = 2.0` Is Undocumented

**Location:** `mpc_types.h`

```c
#define RICCATI_COST_FACTOR 2.0f   /* Global scaling factor applied to stage and terminal costs. */
```

The comment gives no rationale. The factor doubles all cost weights, which changes the relative importance of the ADMM penalty parameter `rho` vs. the objective. If the Riccati solver normalizes internally (which it does not), this would be harmless. Without normalization, it means the tuned `rho` values only work at this specific cost scale. If anyone removes or changes this factor, all ADMM convergence behavior changes unpredictably.

**Fix:** Document explicitly why 2.0 is used and what it implies for the rho calibration.

---

### 19. EMA Filter Blends Heading Error Without Normalization

**Location:** `mpc.c` → `mpc_compute_optimal_control`

```c
filtered_state.fhead_error =
    a * current_frenet_state->fhead_error +
    b * ema_prev_filtered.fhead_error;
```

If the heading error were to transition from near `+π` to near `−π` (a sign flip at the ±π boundary — unlikely but possible on tight corners), the linear blend would pass through 0 instead of staying near ±π. The result would be a transient but large false heading error injected into the MPC.

---

### 20. `VP_INV_MASS_1_PER_KG` and `VP_INV_YAW_INERTIA` Call Functions at Use Sites

**Location:** `mpc_types.h`

```c
#define VP_INV_MASS_1_PER_KG util_recip(VP_MASS_KG)
#define VP_INV_YAW_INERTIA_1_PER_KGM2 util_recip(VP_YAW_INERTIA_KGM2)
```

`util_recip` is an inlined function, not a compile-time constant, so these macros expand to a function call every time they appear. In tight hot-path loops (Riccati backward pass, Jacobian computation), these will be inlined and optimized away by any modern compiler. However, it is cleaner and safer to define these as literal constants computed once:

```c
#define VP_INV_MASS_1_PER_KG      (1.0f / VP_MASS_KG)
#define VP_INV_YAW_INERTIA_1_PER_KGM2 (1.0f / VP_YAW_INERTIA_KGM2)
```

---

### 21. Riccati `P` Matrix Not Symmetrized

**Location:** `riccati_solver.c` → `riccati_solver_pass`

The backward Riccati pass computes `P_k = Q + A^T P A + G^T K`. Mathematically P is symmetric, but floating-point arithmetic does not preserve symmetry exactly. Over N=10 stages, the accumulated asymmetry is negligible in practice, but if the horizon ever increases or the system becomes stiffer, asymmetry drift can cause subtle gain instability. Adding `P[i][j] = P[j][i] = 0.5f*(P[i][j] + P[j][i])` after each stage update costs very little and is standard defensive practice.

---

### 22. No NULL Checks in `vehicle_model_predict_next_state` or `vehicle_model_predict_trajectory`

**Location:** `vehicle_model.c`

Both functions dereference their pointer arguments immediately without checking for NULL:
```c
VehicleState_t vehicle_model_predict_next_state(
    const VehicleState_t *current_state,
    const ControlInput_t *control_input,
    float time_step) {
    ControlInput_t saturated_control = vehicle_model_saturate_control(control_input);
    // ...dereferences current_state immediately...
```

`util_mat_vec_mul` and similar utilities check for NULL, but the model functions do not, which is inconsistent with the rest of the codebase. A NULL dereference from a caller in the hardware callback chain would immediately crash the node.

---

### 23. Hardcoded `rel_tolerance = 0.02f` in Solver

**Location:** `riccati_solver.c` → `riccati_admm_solve`

```c
const float rel_tolerance = 0.02f;   // not user-configurable
```

The absolute tolerance is exposed via `RiccatiAdmmConfig_t.tolerance`, but the relative tolerance is hidden. With a state primal scale of 10 m/s (velocity channel), `eps_primal ≈ tol + 0.02*10 = tol + 0.2`, which can dominate the convergence check and allow large residuals when the solver is running under load. This should be exposed in the config struct.

---

### 24. Global State Variables Not Protected Against Race Conditions

**Location:** `mpc_hardware_node.c`

Multiple ROS2 callbacks (`odometry_subscription_callback`, `servo_feedback_callback`, `imu_callback`, `local_raceline_callback`) write to shared global state (`global_vehicle_state`, `global_actual_steering_angle`, `g_imu_yaw_rate`, `global_trajectory`, etc.) while `amcl_pose_callback` reads them all. The executor serializes callbacks by default in single-threaded `rclc_executor_spin`, so under normal operation this is safe. However, if anyone switches to a multi-threaded executor in the future (common as the codebase grows), all of these accesses become data races. The lack of any synchronization primitives or `_Atomic` qualifiers makes this a latent defect.

---

### 25. Curvature Computation in `local_raceline_callback` Uses Endpoint Extrapolation for Boundary Points

**Location:** `mpc_hardware_node.c` → `local_raceline_callback`

```c
global_trajectory[0].curvature_radians_per_meter =
    global_trajectory[1].curvature_radians_per_meter;
global_trajectory[waypoint_count - 1].curvature_radians_per_meter =
    global_trajectory[waypoint_count - 2].curvature_radians_per_meter;
```

The first and last waypoints copy their neighbor's curvature. For the last waypoint this is acceptable, but for the first waypoint, the vehicle is typically already at or just past that point. If the reference starts on a straight (`kappa[1] ≈ 0`) and the actual entry into the first corner is captured in `kappa[0]`, the first stage of the MPC horizon uses incorrect curvature, degrading the feedforward steering and the heading error Jacobian.

---

### 26. `build_reference_from_trajectory` Arc-Length Integration Uses One-Step Lag

**Location:** `mpc_hardware_node.c` → `build_reference_from_trajectory`

```c
s_query += step_velocity * pred_dt;          // advance by velocity at current step
TrajectoryWaypoint_t wp;
sample_waypoint_by_s(s_query, &wp);
step_velocity = traj_vel;                    // update velocity for next step
```

The arc-length advance at step `k` uses the velocity from step `k-1`. This is a one-step Euler lag. At high speeds (10 m/s) with `pred_dt = 0.032 s`, the spacing between reference points is ~0.32 m. The velocity change between consecutive waypoints on a raceline can be significant (braking zones), making this approximation inaccurate for the horizon tail.

---

### 27. `WEIGHT_STEER_EFFORT` Name Is Misleading in a Rate-Control Formulation

**Location:** `mpc_types.h`, `mpc.c`

In the augmented system, the control `u[0] = delta_rate` (steering rate), not steering angle. `WEIGHT_STEER_EFFORT * delta_rate^2` penalizes how fast the steering is moving, not the absolute steering magnitude. This is more accurately a "steering velocity" penalty. Classical "steering effort" penalizes the angle magnitude `delta^2`, which is captured differently (through the `delta_actual` state cost `WEIGHT_DELTA_ACTUAL`). The naming inconsistency makes tuning harder — an engineer expecting to reduce large steering angles by increasing `WEIGHT_STEER_EFFORT` will find it makes steering sluggish instead.

---

### 28. `TRAJECTORY_SPEED_GAIN` Defined but Unused

**Location:** `mpc_types.h`

```c
#define TRAJECTORY_SPEED_GAIN 1.0f   /* Global multiplier applied to reference speed profile. */
```

This constant is defined with a meaningful description but never referenced in any of the source files. Reference velocity is used directly from the CSV without applying this gain.

---

## Correctness Verification Checklist

| Component | Verdict | Notes |
|---|---|---|
| SE(2) pose integration | ✅ Correct | Analytical integration with omega fallback is correct |
| Slip angle definitions | ✅ Correct | alpha_f, alpha_r signs match bicycle model convention |
| Slip angle derivatives (daf/dvx, daf/dvy, daf/domega) | ✅ Correct | Closed-form atan derivatives verified |
| Tire Jacobians dFyf/dvx, dFyr/dvx, etc. | ✅ Correct | Chain rule application is correct |
| dvx/dt, dvy/dt, domega/dt equations | ✅ Correct | Match standard dynamic bicycle model |
| A[2..4][2..4] Jacobian entries | ✅ Correct | Coriolis terms included |
| B matrix for steering (u[0]) column | ✅ Correct | d(dvx)/d(delta), d(dvy)/d(delta), d(domega)/d(delta) verified |
| Augmented state structure (8 states) | ✅ Correct | delta_actual integrator, prev controls correctly placed |
| Cross-cost N matrix formulation | ✅ Correct | W*(u-u_prev)^2 correctly decomposed via Q, R, N |
| ADMM primal update (z = clip(x+y, lb, ub)) | ✅ Correct | Standard ADMM projection |
| ADMM dual update (y += x - z) | ✅ Correct | Standard dual ascent |
| Riccati backward pass formulas | ✅ Correct | M, S, G, K, kk, P, p all verified |
| Riccati forward pass | ✅ Correct | x_{k+1} = A*x_k + B*u_k + d_k |
| 2×2 matrix inverse | ✅ Correct | Standard formula |
| Normal load formula | ✅ Correct | Static distribution with load transfer |
| Frenet kinematics rows 0–1 linearization | ❌ Wrong | Linearized at zero errors, not at actual op. point |
| Acceleration cross-coupling in B | ❌ Missing | B[3][1] and B[4][1] are zero |
| Terminal state bounds | ❌ Wrong | Uses stage N-1 bounds |
| lf + lr = L identity | ❌ Violated | Off by 2 mm |

---

## Priority Fix Order

1. **Frenet kinematic Jacobian at actual operating point** (Issue 1) — affects every solve at nonzero heading error
2. **Acceleration → lateral dynamics in B matrix** (Issue 2) — affects every solve under braking/throttle
3. **Wall constraints disabled in local raceline mode** (Issue 10) — safety-critical for hardware deployment
4. **Terminal state uses wrong bounds** (Issue 3) — subtle solver inconsistency
5. **`lf + lr ≠ L`** (Issue 4) — model inconsistency affecting every tire calculation
6. **Heading interpolation not wrapped** (Issue 14) — crash-level error at ±π track boundary crossings
7. **`result->final_cost` stores residual not cost** (Issue 5) — telemetry/monitoring correctness
8. **IMU staleness check** (Issue 13) — sensor fusion safety
9. **Tire model consistency** (Issue 7) — prediction vs. linearization correctness

---

*End of review.*
