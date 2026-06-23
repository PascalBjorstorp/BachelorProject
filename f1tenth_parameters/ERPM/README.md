# Full-envelope ERPM / longitudinal odometry model-selection campaign

Install this directory at:

```text
BachelorProject/f1tenth_parameters/ERPM/
```

This campaign runs **after steering calibration**. Its primary objective is not
"find an ERPM gain". It is to select the simplest **causal, deployable**
estimator that best predicts actual longitudinal ground speed from the signals
available on the car:

```text
VESC measured ERPM + VESC motor current + IMU longitudinal acceleration
→ estimated vehicle speed
```

LiDAR scan matching is the independent ground-speed reference used to train
and score the models. Existing ERPM odometry is never used as its own truth.

## Critical site requirement

The supplied default configuration is a **full-envelope** campaign:

```text
maximum speed:                 3.0 m/s
maximum drive acceleration:    5.5 m/s²
maximum brake deceleration:    5.0 m/s²
test-current ceiling:          explicitly configured after safety review
```

It requires a static-featured, controlled straight with at least the configured
usable length, currently **30 m minimum**, and explicitly approved drive/brake
test-current ceilings. The runner checks these developer-set declarations before
touching `vesc.yaml`.

The shipped configuration keeps
`operating_envelope.approved_drive_test_current_a` and
`operating_envelope.approved_brake_test_current_a` at `0.0` until a responsible
developer replaces them with reviewed non-zero limits. That placeholder is
intentional: the full-envelope runner refuses to invent safe current maxima.

A 15 m room is suitable for steering and reduced low-speed work. It is **not**
a defensible site for this full high-speed, high-current acceleration/braking
identification. Do not lower the site value merely to bypass the preflight;
create a deliberately limited-envelope configuration instead.

## One-command operator workflow

```bash
cd ~/BachelorProject/f1tenth_parameters/ERPM
source ~/BachelorProject/install/setup.bash
python3 erpm_calibration.py --workspace ~/BachelorProject
```

Optional no-drive preflight before handing the car to a third party:

```bash
python3 erpm_calibration.py --workspace ~/BachelorProject --preflight-only
```

The third-party operator does not edit `vesc.yaml`, run `colcon`, launch ROS,
start normal bringup, or start bag recording. The runner:

1. creates a byte-exact `vesc.yaml` backup and a recovery lock;
2. applies temporary controller profiles and rebuilds automatically;
3. launches a dedicated calibration graph and records the stage-required ROS
   topics plus a small shared debug bundle into stage-level MCAP bags;
4. runs a strict offline model-selection analysis;
5. validates the selected candidate **inside the live ROS command graph** in
   shadow mode; and
6. restores the exact original VESC configuration and rebuilds, regardless of
   pass, rejection, abort, or ordinary exception.

Two internal checkpoints are automatic. After Stage 5 the runner exports and
fits the settled-speed evidence, then relaunches with an interim speed/odometry
patch so Stages 6-9 use better setup-speed targets and runtime odometry.
After Stage 9 it fits drag and bootstrap acceleration/current terms, then
relaunches Stage 10 with that learned forward-motion patch instead of a blind
fallback gain.

Recovery after power loss or forced termination:

```bash
python3 erpm_calibration.py --recover --workspace ~/BachelorProject
```

## What is selected

The campaign deliberately separates three different mappings:

| Mapping | Selection rule | Zero condition |
|---|---|---|
| Desired speed → ERPM command | Stage 3/4 command hold-outs | `ERPM_cmd(0) = 0` exactly |
| Measured ERPM → wheel-speed observation | Stage 3/4 + dynamic hold-outs | `v_wheel(ERPM=0) = 0` exactly |
| Wheel/IMU/current → estimated ground speed | Complete high-demand Stage 9 trajectories | Causal, bounded, no future samples |

The command map candidates are:

```text
linear:       ERPM = k1 v
quadratic:    ERPM = k1 v + k2 v |v|
monotone LUT: zero-anchored piecewise-linear interpolation
```

The odometry candidates are compared on the same independent ground-truth
hold-outs:

```text
legacy_scalar  installed scalar wheel model, as baseline
static_linear  measured ERPM static linear observation
static_quadratic
static_lut
adaptive_wheel static observation + bounded IMU/current correction
fused_adaptive causal IMU prediction + acceleration-weighted wheel observation
```

The selected model is not the one with the lowest training error. It must meet
coverage, RMSE, signed-bias, 95th-percentile error, high-drive, high-brake,
and non-zero-steering hold-out gates.

## Experimental stages

| Stage | State / experiment | Purpose |
|---:|---|---|
| 0 | Stand-only motor command audit | Proves raw ERPM/current/brake request → selector → VESC command ownership. |
| 1 | Stationary + straight observability | Measures LiDAR velocity noise and verifies scan/IMU/VESC/odom capture. |
| 2 | Low-speed launch ladder | Identifies practical motion threshold, deadband and sensorless start evidence. |
| 3 | Full settled-speed training grid | Fits zero-intercept command and wheel-observation candidates across the speed envelope. |
| 4 | Independent settled-speed hold-out | Rejects static maps that do not generalise. |
| 5 | Existing `VEL_TO_ERPM` audit | Characterises the installed speed path before candidate deployment. |
| 6 | Full-envelope raw-ERPM steps | Measures command/VESC/ground-speed delay and high-speed transient response. |
| 7 | Current-zero coast-down ladder | Identifies Coulomb, viscous and quadratic drag. |
| 8 | Full drive/brake current training schedule | Covers low/mid/high entry speed and fractions up to the approved current ceiling; condition-specific durations prevent speed or stopping-envelope violations and record slip onset/recovery. |
| 9 | Independent current hold-out schedule | Selects the odometry estimator on unseen high-acceleration and braking trajectories. |
| 10 | Learned `ACCEL_TO_CURRENT` routing audit | Audits requested acceleration → current/brake routing using the drag/current patch learned from Stages 6-9. |
| offline | Model selection | Compares every causal estimator family using whole held-out trajectories. |
| 11 | Candidate `VEL_TO_ERPM` shadow validation | Candidate command map and candidate odom feed the live speed loop; plateau hold-outs. |
| 12 | Candidate `ACCEL_TO_CURRENT` shadow validation | Candidate odom feeds the live acceleration loop; dynamic hold-outs. |
| 13 | Candidate non-zero-steering speed hold-out | Optional and disabled by default in the fast campaign; verifies speed accuracy on small calibrated steering arcs and never refits longitudinal parameters. |

Each requested physical trial can be `ACCEPT`, `REDO`, `SKIP`, or `ABORT`.
`REDO` is unlimited. Rejected attempts are retained in MCAP; only accepted
attempts enter fit/validation. Missing accepted repetitions are an explicit
coverage failure, never a silent `NaN` metric.

## Full-stack candidate validation

The existing C++ stack uses a common scalar `speed_to_erpm_gain`/offset for
both command conversion and ERPM odometry. That architecture cannot represent
an independently selected command map and acceleration-aware wheel observation
cleanly.

During Stages 11–13 the suite therefore launches:

```text
candidate_command_map.py
adaptive_odom_shadow.py
candidate_accel_map.py
```

`AckermannToVesc` consumes the candidate odometry stream in the real command
path. For velocity verification, the candidate zero-intercept command map is
the sole ERPM motor-speed source. For acceleration verification, the core C++
node remains responsible for steering while `candidate_accel_map.py` becomes
the sole motor source, applying either the scalar low-slip map or a bounded
full-envelope traction-surface inverse. This validates timing and closed-loop
behaviour without permanently changing the production C++ code.

A candidate can be *accepted for permanent review* only after the shadow model
passes every final hold-out. Permanent installation still requires the C++ port
specified in `full_stack/PRODUCTION_PORT_CONTRACT.md`, replay agreement with
the shadow reference, and a final re-run of Stages 11–13 using the C++ node.

## Evidence and stage-targeted recording

Every stage invokes:

```bash
ros2 bag record -s mcap <required stage topics...>
```

Required-topic checks are still enforced. The bags retain raw `/scan`, `/tf`,
`/tf_static`, parameter events, ROS graph snapshots, VESC telemetry, command
mirrors, raw and selected motor paths, IMU, production odom, candidate
odom/debug signals when active, `/drive`, `/ackermann_cmd`, and structured
trial events without recording unrelated ROS traffic.

Key model-selection outputs are listed in `docs/PARAMETER_OUTPUTS.md`. `docs/SOURCE_PARAMETER_AUDIT.md` identifies active parameters, safety-only limits, and longitudinal fields that are currently unused by the production source.
