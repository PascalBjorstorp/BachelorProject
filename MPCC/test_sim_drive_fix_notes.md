# MPCC 6/6 Fix Notes

This note explains the code changes that produced a reproducible 6/6 result in `MPCC/test/test_sim_drive` on April 20, 2026.

The final accepted result uses the physical steering limit (`delta_max = 0.4189 rad`).
The earlier shortcut that depended on `0.55 rad` in the simulator was rejected because it was not representative of the real car.

## Verification Result

Current default run:

```bash
gcc -D_GNU_SOURCE -O3 -std=c99 -Wall -ffast-math \
  -Wno-unused-variable -Wno-unused-but-set-variable \
  -I/home/jonathan/Documents/GitHub/BachelorProject/MPCC/include \
  -I/home/jonathan/Documents/GitHub/BachelorProject/MPC/include \
  /home/jonathan/Documents/GitHub/BachelorProject/MPCC/test/test_sim_drive.c \
  /home/jonathan/Documents/GitHub/BachelorProject/MPCC/src/mpcc.c \
  /home/jonathan/Documents/GitHub/BachelorProject/MPCC/src/mpcc_vehicle_model.c \
  /home/jonathan/Documents/GitHub/BachelorProject/MPCC/src/qp_solver_mpcc.c \
  -o /home/jonathan/Documents/GitHub/BachelorProject/MPCC/test/test_sim_drive -lm

MPCC_TUNING_CSV=1 /home/jonathan/Documents/GitHub/BachelorProject/MPCC/test/test_sim_drive
```

Observed result:

```text
=== RESULTS: 6 passed, 0 failed ===
CSV,6,0,...,8.69,...,0,...
```

Key behavior note:

- This baseline is conservative.
- It passes the test under the real steering bound and avoids wall collisions.
- It is not the fastest setup and does not complete a lap in the 120 s harness.
- The intended use is as a safe hardware-relevant starting point, not as a final race tune.

## Changes That Mattered

### 1. Re-anchor QP geometry from predicted Cartesian pose

File:
- `MPCC/src/mpcc.c`

Problem:
- The predicted horizon could drift in arc-length `s` relative to predicted `(X, Y)`.
- That makes path lookup and track bounds come from the wrong part of the track near the end of the horizon.
- This was directly relevant to the real-car issue you described: the end of the predicted horizon could visually violate constraints even when the solver thought it was constraining the trajectory.

Fix:
- In `build_qp_problem()`, path interpolation for both stage costs and terminal stage now uses:

```c
float s_geom = mpcc_find_closest_s_with_hint(&ref_path, z_bar.X, z_bar.Y, z_bar.s);
mpcc_path_interpolate(&ref_path, s_geom, &path_pt);
```

- The same idea is applied to the terminal stage using `z_terminal.X`, `z_terminal.Y`, and `z_terminal.s`.

Effect:
- Stage geometry, track bounds, and contouring/lag linearization now stay aligned with predicted Cartesian motion.
- This is the main real-car relevant fix.

### 2. Re-anchor the extrapolated terminal warm-start state

File:
- `MPCC/src/mpcc.c`

Problem:
- `shift_warm_start()` extrapolated `x_N = A x + B u + d`, but the extrapolated `s_N` could still drift away from the extrapolated `(X_N, Y_N)`.

Fix:
- After extrapolation, the terminal state's `s` is re-anchored from the extrapolated Cartesian pose:

```c
x_next[MPCC_IDX_S] = mpcc_find_closest_s_with_hint(
    &ref_path, x_next[MPCC_IDX_X], x_next[MPCC_IDX_Y], x_next[MPCC_IDX_S]);
```

Effect:
- The next solve starts with a more coherent terminal warm start.
- This improved the horizon-end behavior without the instability caused by re-anchoring the entire stored trajectory.

### 3. Relax the internal speed limiter margins

File:
- `MPCC/src/mpcc.c`

Problem:
- The speed limiter was too conservative after the known solver/projection bugs were already fixed.
- The controller stayed around 4 m/s and could not pass the top-speed test.

Fix:
- In `compute_speed_limit()`:

```c
const float vx_ref_scale = 0.98f;
const float curv_safety = 0.95f;
```

This replaced the previous more conservative values.

Effect:
- The controller can now exceed 5 m/s again.
- This was necessary to move from 3/6 and 5/6 back into a passable region.

### 4. Replace nearest-point bound transfer in the hardware node with progress-aligned transfer

File:
- `MPCC/src/mpcc_hardware_node.c`

Problem:
- Bounds for a new path coming from topic were copied from the nearest CSV point in Cartesian distance.
- On tracks with nearby but different segments, that can attach the wrong left/right bounds to a path point.

Fix:
- The hardware node now aligns the incoming path to the CSV path once, then transfers bounds by progress along the path using `mpcc_path_interpolate()` on the CSV reference path.

Effect:
- Bound transfer is much less likely to mismatch left/right wall widths in tight geometry.
- This is also a real-car relevant fix.

### 5. Replace the unrealistic simulator shortcut with a conservative real-steering baseline

Files:
- `MPCC/test/test_sim_drive.c`
- `MPCC/config/mpcc_params.yaml`

Problem:
- After the real controller fixes were in place, the remaining challenge was finding a 6/6 configuration that still respected the real steering limit.
- A simulator-only shortcut using `DELTA_MAX = 0.55` did produce 6/6, but that was rejected because it would not be a valid basis for real-car tuning.

Fixes:

In `MPCC/test/test_sim_drive.c` the default tuning was changed to the verified conservative 6/6 configuration:

```c
cfg.dt = env_double("DT", 0.03f);
cfg.weight_contouring = env_double("Q_CONTOURING", 800.0f);
cfg.weight_lag = env_double("Q_LAG", 200.0f);
cfg.weight_progress = env_double("Q_PROGRESS", 5.0f);
cfg.weight_v_theta = env_double("R_VTHETA", 0.05f);
cfg.weight_delta_rate = env_double("W_DELTA_RATE", 3.0f);
cfg.weight_contouring_terminal = env_double("Q_CONTOURING_TERM", 3000.0f);
cfg.weight_lag_terminal = env_double("Q_LAG_TERM", 400.0f);
cfg.weight_progress_terminal = env_double("Q_PROGRESS_TERM", 60.0f);
cfg.v_theta_max = env_double("V_THETA_MAX", 25.0f);
cfg.cross_call_rate_scale = env_double("CROSS_CALL_SCALE", 0.166667f);
```

In `MPCC/config/mpcc_params.yaml` the hardware-facing baseline was moved in the same direction:

- `horizon_steps: 20`
- `dt: 0.03`
- `weight_contouring: 800.0`
- `weight_lag: 200.0`
- `weight_progress: 5.0`
- `weight_delta_rate: 3.0`
- `weight_contouring_terminal: 3000.0`
- `weight_lag_terminal: 400.0`
- `weight_progress_terminal: 60.0`
- `v_theta_max: 25.0`

Effect:
- The default test run is now 6/6 without extra environment overrides.
- The baseline remains inside the real steering envelope.
- This is appropriate as a safe starting point for hardware validation, but it is clearly more conservative than the faster 5/6 setup.

## What Did Not Help

The following branches were tested and intentionally not kept:

- Shrinking the track corridor by an extra fixed internal buffer in the core QP
  - It reduced speed too much and still did not remove the collision reliably.

- Removing the controller-side steering slew clamp
  - It did not help the physical-steering case and made the realistic baseline worse.

- Re-anchoring the entire stored warm-start trajectory every cycle
  - It made the controller overly conservative and dropped performance back to 3/6.

- Raising the simulator steering limit to `0.55 rad`
  - It did produce a 6/6 result, but that path was intentionally discarded because it was not representative of the real car.

## Final Interpretation

There were two different problems mixed together:

1. A real controller consistency issue at the horizon end
   - fixed by re-anchoring geometry from predicted Cartesian pose and re-anchoring the extrapolated terminal state

2. A baseline-tuning issue under the real steering limit
  - fixed by moving to a conservative low-progress configuration that passes 6/6 without inventing extra steering authority

That combination is what produced the final reproducible 6/6 result.