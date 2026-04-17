# MPC Edit Log and MPC_FPGA_Kria Implementation Guide

Date: 2026-04-16
Scope: Edits completed in MPC during the iterative defect-fix loop.

## 1. Summary

This document records all MPC-side edits completed in this session and marks what should be implemented in MPC_FPGA_Kria where relevant.

Validation status after edits:
- Built and ran `MPC/test/test_sim_drive.c`
- Result: 6 passed, 0 failed
- Repeated sub-agent sweep result: no actionable errors found (with ALPHA removal treated as intentional)

## 2. MPC Files Edited and What Changed

### A) `MPC/src/vehicle_model.c`

Changes made:
- Fixed low-speed slip Jacobian behavior:
  - When `vx <= MIN_SLIP_VELOCITY`, forced `d(alpha)/d(vx) = 0`.
- Upgraded Frenet kinematic Jacobian rows to operating-point form:
  - Added exact dependence on `e_psi` and `e_y` for A[0,*] and A[1,*].
  - Included denominator protection for `1 - kappa * e_y`.
- Added missing acceleration-to-lateral/yaw coupling in B-matrix:
  - Added `B[3][1]` and `B[4][1]` terms from load-transfer sensitivity.

MPC_FPGA_Kria relevance:
- Yes (high).
- Port equivalent Jacobian/B-matrix corrections to HLS vehicle linearization path.

---

### B) `MPC/src/mpc.c`

Changes made:
- Removed legacy override alias behavior for acceleration-rate weight:
  - Kept `MPC_W_ACCEL_RATE` path; removed `MPC_W_TORQUE_RATE` override from default config flow.
- Improved previous-control feedback handling:
  - In `mpc_set_actual_previous_control`, compute and clamp steering-rate from actual steering angle difference.
- Improved EMA heading filtering:
  - Applied wrap-aware heading blending in `[-pi, pi]` to avoid seam artifacts.
- Fixed stage linearization rollout consistency:
  - Stopped forcing `lin_state.flong_vel` to reference each step.
  - Kept rollout velocity from predicted next state.
- Added affine bias plumbing support in MPC setup:
  - Filled `sd->d[]` based on stage operating-point residual (`x_next - (A x + B u)`).
- Added explicit terminal constraints setup:
  - Created and populated `terminal_x_lb[]`, `terminal_x_ub[]`.
- Hardened runtime config setter:
  - Clamp horizon to `[1, PREDICTION_HORIZON]`.
  - Ensure positive `time_step` fallback to default.
- Hardened solver error behavior:
  - If Riccati solve returns non-optimal/non-max-iter status, do not use stale ADMM projected control.
  - Return safe fallback command (`steer=actual`, `accel=0`).

MPC_FPGA_Kria relevance:
- Yes (high) for:
  - Affine bias generation and usage consistency.
  - Terminal-bound handling semantics.
  - Avoiding stale-control-on-error behavior.
- Partial for EMA/config topics (depends on whether mirrored in FPGA host-side wrapper or testbench).

---

### C) `MPC/src/riccati_solver.c`

Changes made:
- Added affine-bias-consistent backward recursion:
  - Computed `p_shift = p + P*d` per stage.
  - Used `p_shift` in `B^T p` term for `kk`.
  - Used `p_shift` in `A^T p` term for `p` update.
- Forward pass already uses affine bias in dynamics:
  - `x_{k+1} = A x_k + B u_k + d_k` for dense states and augmented previous-control states.
- Added explicit terminal bound API usage:
  - Replaced implicit use of `step_data[N-1]` at terminal stage with `terminal_x_lb/ub` arrays.

MPC_FPGA_Kria relevance:
- Yes (critical).
- This is one of the highest-priority ports if not already mirrored.

---

### D) `MPC/include/riccati_solver.h`

Changes made:
- Updated Riccati API signatures to include explicit terminal state bounds:
  - Added `terminal_x_lb`, `terminal_x_ub` in both `riccati_solver_pass` and `riccati_admm_solve`.

MPC_FPGA_Kria relevance:
- Yes (interface parity).
- Mirror equivalent API or internal data path in HLS/C wrapper boundary.

---

### E) `MPC/include/mpc_types.h`

Changes made:
- Added affine bias storage in step data:
  - `float d[RICCATI_MAX_NX]` in `RiccatiStepData_t`.
- Fixed wheelbase consistency:
  - Defined `VP_WHEELBASE_M` from `lf + lr`.
- Switched reciprocal macros to literal reciprocal constants:
  - `1.0f / ...` style for mass/inertia/shape reciprocals.
- Clarified solver result field meaning:
  - `final_cost` documentation changed to reflect current stored metric (primal residual).

MPC_FPGA_Kria relevance:
- Yes (high) for `d[]` and wheelbase-consistency constants.
- Medium for reciprocal macro style and docs.

---

### F) `MPC/src/mpc_hardware_node.c`

Changes made:
- Improved heading interpolation in waypoint sampling:
  - Added shortest-angle interpolation with wrapping.
- Improved Frenet lateral-error basis accuracy:
  - Normalized interpolated `(sin_h, cos_h)` basis before use.
- Added IMU freshness guard:
  - Added `g_last_imu_time`, timestamped IMU callback.
  - Used IMU yaw rate only when fresh (<= 50 ms), fallback to odometry otherwise.
- Local raceline safety bound hardening:
  - Replaced unconstrained `BIG_BOUND` defaults with finite wall bounds (`1.5 m`) for local raceline points.

MPC_FPGA_Kria relevance:
- Mostly host/node-side (not HLS kernel).
- Implement where FPGA deployment uses equivalent host-side reference preparation and sensor fusion logic.

---

### G) `MPC/sim/mpc_ros2_node.c`

Changes made:
- Fixed heading interpolation wrap in `sample_waypoint_by_s` (both regular and wraparound segment).
- Startup order fix:
  - Initialize MPC before reading config for startup printouts.
- Steering-lag simulation consistency:
  - Used fixed control-step (`CONTROL_DT_SECONDS`) in servo-lag update path.

MPC_FPGA_Kria relevance:
- Indirect.
- Useful for FPGA-in-loop simulation harness parity, not core kernel math.

---

### H) `MPC/test/test_sim_drive.c`

Changes made:
- Fixed heading interpolation wrap in raceline sampler.
- Updated map naming/path defaults to `my_track_raceline` and safer fallback paths.

MPC_FPGA_Kria relevance:
- Yes for regression harness consistency.
- Not required in kernel, but recommended in FPGA validation testbench path.

## 3. Priority Port List for MPC_FPGA_Kria

Implement first:
1. Affine bias consistency in Riccati recursion (`p_shift = p + P*d`) and forward dynamics with `+d`.
2. Presence and propagation of `d[]` in per-stage step data.
3. Frenet Jacobian operating-point fixes and missing acceleration cross-coupling terms.
4. Explicit terminal-bound handling for terminal state stage (avoid implicit `N-1` reuse semantics).
5. Wheelbase consistency (`L = lf + lr`) in all feedforward/vehicle constants.

Implement second:
1. Host-side stale-signal handling (IMU freshness fallback).
2. Host-side local raceline finite safety bounds.
3. Heading wrap interpolation consistency in test/sim/reference samplers.

## 4. Notes on ALPHA

- ALPHA over-relaxation support was intentionally removed/kept absent by design.
- This was treated as intentional in final audit criteria.

## 5. Verification Performed

Command run:
- `gcc -D_GNU_SOURCE -O3 -std=c99 -Wall -Wextra -ffast-math -Wno-unused-variable -Wno-unused-but-set-variable -Iinclude test/test_sim_drive.c src/mpc.c src/riccati_solver.c src/vehicle_model.c src/util_math.c -o test_sim_drive -lm && ./test_sim_drive`

Observed result:
- `=== RESULTS: 6 passed, 0 failed ===`

## 6. Implementation Checklist for Kria Team

- [ ] Add/verify `d[]` term in Kria step-data type.
- [ ] Add/verify `p_shift = p + P*d` usage in backward pass (`kk` and `p` updates).
- [ ] Add/verify forward rollout includes `+d` for all relevant states.
- [ ] Port corrected Frenet Jacobian rows and B cross-coupling terms.
- [ ] Ensure terminal bounds are explicit at stage `N`.
- [ ] Ensure wheelbase constant derives from axle distances.
- [ ] Mirror host-side safety/interpolation fixes where FPGA stack shares those paths.
