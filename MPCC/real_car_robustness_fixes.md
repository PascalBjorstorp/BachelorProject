# MPCC Real-Car Robustness Fixes

This note records the retained MPCC fixes for the real car, the fixes that were tested but not kept, and the current launch profile that matches the validated `track_racer` baseline on `my_track_centerline_smooth.csv`.

## Problem Summary

The real-car MPCC was showing chaotic behavior, especially in corners and near the end of the horizon:

- predicted trajectories becoming infeasible near the horizon end
- corner behavior that looked unstable or directionally inconsistent
- hardware runtime behavior diverging from the older simulation-safe baseline
- the old conservative 6/6 baseline no longer reproducing on the newest raceline

The key lesson from this round of work is that robustness required both controller/runtime guardrails and a fresh raceline-specific launch profile. Old sweep winners could not be trusted without reproduction on the current code and current raceline.

## Retained Fixes

### 1. Hardware launch is now separated from stale defaults

File:

- `MPCC/launch/mpcc_hardware.launch.py`

Retained changes:

- The hardware launch now owns an explicit MPCC tuning profile instead of silently inheriting whatever generic defaults happen to be compiled in.
- Trajectory autodetection now searches both install-space and workspace-style layouts before falling back to the node's internal autodetect.
- The launch exposes all relevant hardware tuning values as arguments, so runtime tuning changes do not require code edits.

Current default hardware profile in the launch file:

```text
MPCC_PROFILE=track_racer
HORIZON=80
DT=0.03
Q_CONTOURING=80.0
Q_LAG=120.0
Q_WALL_CLEARANCE=3200.0
WALL_CLEARANCE_MARGIN=0.02
MPCC_TRACK_BUFFER=0.05
Q_PROGRESS=8.0
Q_VX=0.0
VX_REF=0.0
MPCC_USE_RACELINE_VX_REF=0
MPCC_USE_RACELINE_VX_LIMIT=0
MPCC_RACELINE_VX_LIMIT_SCALE=1.0
Q_VY=1.0
Q_OMEGA=3.0
R_DELTA=8.0
R_AX=1.0
R_VTHETA=0.2
W_DELTA_RATE=2.0
W_AX_RATE=3.0
W_VTHETA_RATE=0.8
Q_CONTOURING_TERM=75.0
Q_LAG_TERM=60.0
Q_PROGRESS_TERM=10.0
ADMM_RHO=15.0
ADMM_RHO_U=8.0
ADMM_MAX_ITER=300
ADMM_TOL=0.02
ADMM_ADAPTIVE_RHO=1
ADMM_ALPHA_RELAX=1.6
MPCC_ACCEPT_MAX_ITER=0
MPCC_MAX_ITER_PRIMAL_TOL=0.01
MPCC_MAX_ITER_DUAL_TOL=0.01
MPCC_MAX_ITER_TRACK_TOL=0.005
V_THETA_MAX=8.0
MPCC_CROSS_CALL_SCALE=0.166667
MPCC_ADAPT_CROSS_CALL_SCALE=0
MPCC_VX_MIN_CMD=0.1
```

This is the profile that now mirrors the current simulation `track_racer` baseline while keeping the hardware-only guardrails.

### 2. Hardware-node runtime guardrails

File:

- `MPCC/src/mpcc_hardware_node.c`

Retained changes:

- Lowered the default minimum positive speed command from `1.0` to `0.1` m/s.
- Added `MPCC_ADAPT_CROSS_CALL_SCALE` so the hardware-safe fixed scale is the default behavior.
- Added parsing of `MPCC_CONTROL_PERIOD_MS` as the nominal control cadence used for rate scaling when no explicit scale is set.
- If adaptive scaling is enabled, the filtered control interval is bounded to stay within `[0.5x, 2.0x]` of the nominal control cadence.
- Added an odometry staleness guard: the solver now skips updates when odometry is older than the watchdog timeout.
- Changed solver-failure handling from "hold the previous command exactly" to a safer fallback:
  - steering is reduced to half of the previous command
  - braking is applied gently
  - speed is recomputed from the current measured velocity
- Added a warning when the loaded CSV corridor becomes very tight.

These changes are meant to reduce real-car chaos when timing, sensing, or solver convergence degrades.

### 3. Progress continuity fix in the core

Files:

- `MPCC/src/mpcc.c`
- `MPCC/include/mpcc.h`

Retained changes:

- `mpcc_state_from_vehicle_state()` now uses the supplied `s_hint` instead of discarding it.
- The selected `s` value is clamped against non-physical jumps relative to the previous progress estimate.
- The public API comment now correctly documents that `s_hint` is used for continuity.

This retained fix addresses one of the most plausible causes of corner-time chaos: the controller can otherwise match the vehicle to the wrong arc-length neighborhood and start optimizing against the wrong part of the track.

## Investigated But Not Kept

These changes were explored but intentionally removed after revalidation.

### 1. Low-speed vehicle-model change

File investigated:

- `MPCC/src/mpcc_vehicle_model.c`

What was tried:

- reducing the low-speed `vx_safe` floor
- forcing the affine term to use the same stabilized low-speed velocity model as the Jacobians

Why it was removed:

- it did not improve the reproduced current-raceline candidate during validation
- it reduced confidence in the already reproduced hardware candidate without proving a real-car benefit

Conclusion:

- the low-speed model mismatch is still a valid investigation topic, but this particular patch was not retained because it did not survive validation

### 2. Stage and terminal geometry jump limiting inside QP construction

Files investigated:

- `MPCC/src/mpcc.c`

What was tried:

- clamping stage geometry re-anchoring in `build_qp_problem()`
- clamping terminal geometry re-anchoring
- clamping warm-start terminal reprojection inside `shift_warm_start()`

Why it was removed:

- it broke the reproduced current-raceline candidate row that had previously validated cleanly
- the retained `s_hint` fix in `mpcc_state_from_vehicle_state()` was the safer continuity fix to keep

Conclusion:

- more aggressive geometry clamping may still be useful later, but not in this form

## Math Audit Result

A direct sign audit of the MPCC math did **not** find a simple `+/-` bug in the progression logic.

Specifically:

- the progress reward sign on `v_theta` is correct
- the terminal progress sign on `s` is correct
- the contouring and lag equations use standard MPCC sign conventions
- no left/right sign flip was found in the Cartesian transforms or heading wrap logic

So the "wrong direction" feeling is more likely to come from state/path continuity problems, stale runtime data, or solver/runtime mismatch than from a single flipped sign in the objective.

## Validation Results

### Reproduced current track-racer candidate

Validation command:

```bash
cd MPCC
gcc -D_GNU_SOURCE -O3 -std=c99 -Wall -ffast-math \
  -Wno-unused-variable -Wno-unused-but-set-variable \
  -Iinclude -I../MPC/include \
  test/test_sim_drive.c src/mpcc.c src/mpcc_vehicle_model.c src/qp_solver_mpcc.c \
  -o test_sim_drive -lm

RACELINE_PATH=/home/jonathan/Documents/GitHub/BachelorProject/f1tenth_planning/trajectories/my_track_centerline_smooth.csv \
MPCC_PROFILE=track_racer DT=0.03 HORIZON=80 \
Q_CONTOURING=80 Q_LAG=120 Q_WALL_CLEARANCE=3200 WALL_CLEARANCE_MARGIN=0.02 MPCC_TRACK_BUFFER=0.05 \
Q_PROGRESS=8 Q_VX=0 VX_REF=0 MPCC_USE_RACELINE_VX_REF=0 MPCC_USE_RACELINE_VX_LIMIT=0 MPCC_RACELINE_VX_LIMIT_SCALE=1.0 \
Q_VY=1.0 Q_OMEGA=3.0 R_DELTA=8 R_AX=1.0 R_VTHETA=0.2 \
W_DELTA_RATE=2.0 W_AX_RATE=3.0 W_VTHETA_RATE=0.8 \
Q_CONTOURING_TERM=75 Q_LAG_TERM=60 Q_PROGRESS_TERM=10 \
ADMM_RHO=15 ADMM_RHO_U=8 ADMM_MAX_ITER=300 ADMM_TOL=0.02 ADMM_ADAPTIVE_RHO=1 ADMM_ALPHA_RELAX=1.6 \
MPCC_ACCEPT_MAX_ITER=0 MPCC_MAX_ITER_PRIMAL_TOL=0.01 MPCC_MAX_ITER_DUAL_TOL=0.01 MPCC_MAX_ITER_TRACK_TOL=0.005 \
V_THETA_MAX=8.0 CROSS_CALL_SCALE=0.166667 MPCC_TUNING_CSV=1 ./test_sim_drive
```

Validated result:

```text
CSV,6,0,0.3881,0.1284,0.7923,0.5595,6.63,2281.8,4926.2,0,1.1,4.9188,1.7154,245.1,5.000,5.000,0.00,0.00,0.6344,1,12.9000
```

Key points:

- `tests_passed = 6`
- `tests_failed = 0`
- `wall_collisions = 0`
- `lap_count = 1`
- `best_lap_time = 12.9 s`

This is the currently reproduced profile for `my_track_centerline_smooth.csv`.

### Old conservative baseline no longer reproduces cleanly

The older conservative hardware-safe April baseline does not remain a 6/6 on the newest raceline. It is no longer a trustworthy default for the current trajectory.

### ROS package build

Final retained code state builds cleanly:

```bash
cd /home/jonathan/Documents/GitHub/BachelorProject
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mpcc_f1_10th --symlink-install
```

Observed result:

```text
Summary: 1 package finished
```

## Recommended First Hardware Run

Use the current launch defaults as they now stand.

Recommended procedure:

1. Run in a low-risk space first.
2. Keep an immediate stop ready.
3. Watch the first two corners closely.
4. If it still behaves chaotically, capture the exact symptom type:
   - wrong-side path matching
   - sudden steering reversal
   - low-speed oscillation
   - solver-failure fallback activation
   - horizon-end constraint violation in one specific corner

Those details will determine the next fix much more reliably than another blind sweep.

## Practical Conclusion

The retained robustness solution is:

- keep the runtime guardrails in the hardware node
- keep the `s_hint` continuity fix in the core
- use the current `track_racer`-derived launch profile by default
- do not trust older MPCC sweep winners unless they reproduce on the current code and current raceline

That is the cleanest validated state reached in this iteration.