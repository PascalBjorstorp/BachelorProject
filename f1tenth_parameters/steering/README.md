# Bag-first steering calibration

Install this directory at:

```text
BachelorProject/f1tenth_parameters/steering/
```

The third-party operator runs one command:

```bash
cd ~/BachelorProject/f1tenth_parameters/steering
source ~/BachelorProject/install/setup.bash
python3 steer_calibration.py --workspace ~/BachelorProject
```

They do **not** edit `vesc.yaml`, run `colcon`, launch normal bringup, start MPC/MPCC/planners/teleoperation, or start `ros2 bag record`. The runner performs the reversible VESC configuration transaction, full build, dedicated launch, stage-by-stage MCAP recording, and automatic restoration.

## Automatic temporary VESC configuration

Before launching any ROS nodes, the runner:

1. creates a byte-exact backup of `f1tenth_system/f1tenth_stack/config/vesc.yaml` inside the session directory;
2. creates a persistent recovery lock in the workspace;
3. temporarily sets `operation_mode: VEL_TO_ERPM`, `accel_to_current_gain: 0.0`, `accel_to_brake_gain: 0.0`, `servo_min: 0.0`, and `servo_max: 1.0`;
4. runs `colcon build --symlink-install` for the workspace;
5. runs the calibration suite;
6. restores the byte-exact original `vesc.yaml` and performs the full build again, including after `ABORT`, Ctrl+C, and ordinary run failures.

The temporary `[0.0, 1.0]` range removes possible VESC software clipping only while the operator finds the physical endpoint. It does **not** command either endpoint automatically. After Stage 2 the calibration selector is restarted with the discovered safe interval, so all later driving tests are clipped to it.

If a process is killed or the machine loses power while the temporary configuration is active, do not drive. Run:

```bash
cd ~/BachelorProject/f1tenth_parameters/steering
source ~/BachelorProject/install/setup.bash
python3 steer_calibration.py --recover --workspace ~/BachelorProject
```

The recovery routine restores the saved source file and rebuilds the workspace. It clears the recovery lock only after a successful rebuild.

No candidate steering mapping or endpoint limit is installed automatically after analysis. The raw session contains `proposed_post_session_vesc_servo_limits.yaml` for later technical review.

---

## Room preparation

Use the supplied layout as a guide:

```text
docs/room_layout.png
```

The room should be **asymmetric, static and geometrically rich**. Place fixed panels, boxes, pillars, cones, an L-shaped barrier and non-parallel objects around the paths, but never in the safety margin or manoeuvre area. Do not move objects or allow people to cross the LiDAR field during a capture.

The goal is not global localisation. Raw LiDAR scans are matched only to consecutive scans to estimate relative body motion. Corners, wall ends and non-parallel surfaces constrain that relative motion better than an empty room or a long corridor with parallel walls.

---

## Data integrity: ACCEPT, REDO, SKIP and ABORT

Every vehicle manoeuvre is a separately identified **trial attempt** inside its stage MCAP bag.

After every attempt the terminal shows the startup/stability result and asks:

```text
ACCEPT  keep this trial for offline fitting
REDO    keep raw data for audit, exclude this attempt, repeat the condition
SKIP    keep raw data for audit, exclude this condition, continue
ABORT   stop safely; preserve all complete and incomplete bags
```

A trial that fails the automatic startup-stability test cannot be accepted. It may only be redone, skipped, or abort the session. There is **no retry limit**: REDO continues until the operator accepts, skips, or aborts.

Raw data from rejected and redone attempts are **not deleted**, but the runner publishes a structured `trial_decision` event. Offline fitting only uses intervals whose final decision is `ACCEPT`. Consequently a failed trial cannot silently corrupt the fitted centre, static map, or response model.

Use resume after a stop:

```bash
python3 steer_calibration.py --resume runs/<session-id>
```

Completed stages remain intact. An interrupted stage is archived and re-run into a fresh stage directory.

---

## Startup jitter and capture timing

No driving capture begins immediately when a speed command is issued.

Every driving attempt follows:

```text
command target steering and speed
→ excluded startup period: at least 0.75 s
→ rolling operational stability check
→ raw-data capture begins only after the check passes
→ vehicle stops
→ operator ACCEPT / REDO / SKIP / ABORT
```

The stability test uses encoder-derived odometry and IMU longitudinal acceleration only to decide when a vehicle has completed its initial transient:

- median odometry speed close to the commanded speed;
- low variation over a 0.60 s window;
- low median longitudinal acceleration.

That runtime use of odometry does **not** make odometry the reference used for the calibration. The offline static steering fit uses accepted raw LiDAR scan-matching velocity with IMU yaw rate. The IMU yaw rate has its stationary gyro-z bias removed first (estimated from the Stage 3 stationary capture, falling back to the Stage 0 stand); the value used is recorded in `analysis/imu_bias.json`. A single stationary estimate cannot track in-run thermal drift of the gyro.

All timing thresholds are in `config/steering_calibration.yaml` under `motion_startup`.

---

## Session stages

| Stage | Vehicle state | Main purpose | Repetitions in the thorough profile |
|---:|---|---|---:|
| 0 | Stand | Command-chain / echo interlock | 5 stationary commands |
| 1 | Ground | Zero-curvature raw-servo centre | 7 requests × 3, then 5 confirmations |
| 2 | Stand | Safe physical end-stops | One operator-confirmed last-free value per side |
| 3 | Ground | Sensor observability and stationary ICP noise floor | 3 straight runs per speed; 3 turns per side |
| 4 | Ground | Static raw-servo → curvature training map | 4 outward/inward sweeps per side |
| 5 | Ground | Hold-out static-map validation | 3 shuffled repetitions per side |
| 6 | Ground | Command-to-curvature response dynamics | 5 repetitions per speed / side / step size |

The number of conditions is deliberately high. The static-map fit must be repeatable across approach direction and across independent hold-out data, not merely interpolate one sweep.

Static-map repeatability and hysteresis are grouped by the **nominal commanded condition**—side, configured safe-span fraction and approach direction. The measured servo echo remains a recorded regressor and interpolation axis, but it is never used as an exact floating-point grouping key.

---

## MCAP output

One session directory is produced:

```text
runs/<session-id>/
  00_command_chain_audit/bag/
  01_zero_curvature_centre/bag/
  02_physical_endstops/bag/
  03_sensor_observability/bag/
  04_static_map_training/bag/
  05_static_map_holdout/bag/
  06_command_to_curvature_response/bag/
  runtime_state.json
  session_manifest.yaml
  calibration_config_snapshot.yaml
```

Each bag stores raw IMU, VESC state, ERPM odometry, raw `LaserScan`, commands, servo command echo, TF and `/steering_calibration/event` markers. There is no runtime CSV logging.

The analysis receives the entire session directory:

```bash
source ~/BachelorProject/install/setup.bash
cd ~/BachelorProject/f1tenth_parameters/steering
python3 -m pip install -r requirements-offline.txt
python3 analysis/run_analysis.py runs/<session-id>
```

The pipeline exports scalar data to Parquet, preserves full scans in MCAP, estimates LiDAR motion, produces an ICP observability report, fits the centre, fits the static map only from accepted trials, validates it on hold-out trials, and estimates response dynamics.

## Steering parameters produced

The full output list is in `docs/STEERING_PARAMETER_OUTPUTS.md`.

The dynamic stage reports both:

- **command-path timing**: raw request → selector → servo command bus → VESC-driver command echo; and
- **effective steering dynamics**: raw-servo step → IMU/LiDAR curvature → equivalent bicycle steering response.

The latter yields delay, 10–90% rise time, 5% settling time, overshoot, peak effective steering rate, local steady gain, and a first-order-plus-dead-time fit `K exp(-Ls)/(tau s + 1)`, separately by left/right, step magnitude, speed, repeat and return direction.

For early simulator or lateral-model work, the offline analysis now also emits
`analysis/steering_simulation_seed_report.json`. That report packages the
vehicle-level quantities the current sensors can actually support: effective
steering gain, curvature gain, yaw-rate gain, lateral-acceleration gain,
hysteresis, repeatability, and effective lag by speed/side/step. It is useful
for seeding a better bicycle/simulation model, but it is not presented as full
Pacejka identification.

`/sensors/servo_position_command` is only a command echo, **not a measured servo-shaft or wheel angle**. Therefore this session cannot truthfully identify mechanical servo-shaft delay/rate in isolation. It identifies the combined steering-to-vehicle response used by the MPC plant. A real steering-angle sensor is required to split servo, linkage, tyre and vehicle-yaw dynamics.

---

## ICP: solver accuracy versus measurement accuracy

The ICP solver is intentionally configured with tight numerical convergence settings:

```yaml
max_iterations: 80
translation_update_tolerance_m: 1.0e-6
rotation_update_tolerance_rad: 1.0e-6
relative_cost_tolerance: 1.0e-7
```

Those values ensure the iterative optimizer has converged. They do **not** imply micrometre-level physical motion accuracy; LiDAR range noise, scan geometry, timing and scene structure dominate that limit.

Every scan pair therefore also receives independent quality gates:

- numerical convergence before the iteration limit;
- number of correspondences and inlier ratio;
- point-to-line RMSE;
- Hessian condition number, which detects weak geometry / aperture problems;
- disagreement between ICP rotation and the IMU yaw seed;
- uncertainty estimates from the final normal-equation covariance.

Only scan pairs passing all gates are marked `valid`. A static-map capture is accepted for fitting only when at least 70% of its retained scan pairs are valid, and its LiDAR-speed and yaw-rate variation are low.

At 40 Hz the LiDAR sweeps each beam at a slightly different instant while the
car is moving, so a single scan is not a rigid snapshot. The motion estimator
therefore (a) **deskews** each scan to its first-beam time using the
constant-velocity prediction carried from the previous pair, and (b) **warm
starts** the ICP translation from that same prediction so the first
correspondence search begins near the true per-frame displacement instead of at
zero. Both are on by default (`analysis.icp.motion_deskew`,
`analysis.icp.constant_velocity_seed`) and reported in `lidar_motion_summary.json`;
set them false to reproduce the original behaviour. They matter most at the
higher steering speeds.

The stationary section of Stage 3 generates `analysis/icp_observability_report.json`; inspect its stationary apparent-motion noise before trusting the later map.

---

## Dedicated launch topology

`steer_calibration.py` starts:

- VESC driver;
- VESC-to-odometry;
- Ackermann-to-VESC;
- `ackermann_mux`;
- LiDAR driver;
- static transforms;
- calibration runtime node and MCAP bag process.

It intentionally does **not** start MPC, MPCC, map server, planner or teleoperation.

The physical end-stop stage switches to a hardware-only stack where `AckermannToVesc` is absent; the calibration runner becomes the sole publisher of `/commands/servo/position`.

---

## Before first use

1. Copy `steering/` into `f1tenth_parameters/`.
2. Source the ROS workspace. The calibration runner performs its own temporary full build.
3. Check that the MCAP bag plugin works:
   ```bash
   ros2 bag record -s mcap -o /tmp/mcap_test /sensors/imu/raw
   ```
4. Verify `config/steering_calibration.yaml` values for wheelbase, LiDAR transform, the 3S battery limit (10.50 V) and speed limit.
5. Run Stage 0 and Stage 2 with the vehicle on a stand before any on-ground motion.
6. Review every rejected / skipped condition in `runtime_state.json` and every final candidate in `runs/<session-id>/analysis/`.

## Calibration ownership and configuration changes

All measurement stages use **raw-servo selector mode**. The calibration stack
remaps `AckermannToVesc` servo output to
`/steering_calibration/servo_from_ackermann`; only the selector publishes the
real `/commands/servo/position`. Therefore existing steering gain, offset,
polynomial correction, and angle limits cannot constrain the raw-servo data.
`AckermannToVesc` remains running only to supply the low-speed motor command.

Values produced inside one session are passed automatically to later stages:
`centre_servo_raw` → physical end-stop survey → safe raw spans → observability,
static-map, validation, and response targets. Do **not** edit `vesc.yaml`,
rebuild, or restart between stages. Apply a reviewed candidate map only after
offline analysis, rebuild/re-source the normal stack if your installation
requires it, then run a separate post-install verification.

`command_publish_hz` is command-refresh rate, not recording or sensor rate.
The default is **100 Hz** (10 ms command timing). `/ego_racecar/odom` arrives at
**200 Hz** and is retained by MCAP at its native timestamps; it is used at
runtime only for the startup/safety scheduling gate.

## Dedicated motor command mode

The ordinary VESC configuration currently enables `ACCEL_TO_CURRENT`. That mode
interprets `drive.acceleration`, while this calibration runner issues
`drive.speed` setpoints. Before the ROS stack starts, `steer_calibration.py` performs a reversible
configuration transaction on the workspace source `vesc.yaml`:

1. copies the exact original file into the session directory;
2. sets `operation_mode: VEL_TO_ERPM`, `accel_to_current_gain: 0.0`, and
   `accel_to_brake_gain: 0.0`;
3. sets `servo_min: 0.0` and `servo_max: 1.0`;
4. runs `colcon build --symlink-install` for the full workspace;
5. runs the calibration session;
6. restores the byte-exact original `vesc.yaml` and builds the workspace again.

The VEL-to-ERPM selection is a repeatable low-speed command interface, not a
ground-speed reference: offline steering identification still uses raw LiDAR
scan matching and raw IMU yaw rate. A crash or power loss leaves a persistent
recovery file; before driving, restore with:

```bash
python3 steer_calibration.py --recover --workspace <BachelorProject-workspace>
```

## Raw-servo numeric domain versus safe physical range

The ROS raw-servo command protocol is limited numerically to `[0.0, 1.0]` only.
That is not a mechanical safety limit. The session runner temporarily sets the VESC servo domain to the same full
numeric interval in the source configuration before building, so the current
`servo_min`/`servo_max` cannot truncate the physical end-stop survey.

Stage 2 is the physical protection: the operator moves outward manually from
the identified centre and records the last clearly free position. The resulting
inward-offset safe interval is then used for every later manoeuvre.
