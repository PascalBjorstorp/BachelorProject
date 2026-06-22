# Bag-first ERPM / longitudinal calibration

Install this directory at:

```text
BachelorProject/f1tenth_parameters/ERPM/
```

This is the speed-side companion to the steering campaign. It is run **after**
steering calibration has been completed and installed, so the car is commanded
at zero steering angle and straight-line LiDAR motion is a valid longitudinal
reference. The default LiDAR geometry is copied from the steering campaign: `192.168.10.10`, `(x, y, z) = (0.265, 0, 0.05) m`, and yaw `0 rad`. The same immutable session snapshot drives both the runtime TF and offline ICP.

## One-command operator workflow

```bash
cd ~/BachelorProject/f1tenth_parameters/ERPM
source ~/BachelorProject/install/setup.bash
python3 erpm_calibration.py --workspace ~/BachelorProject
```

The third-party operator does not edit source configuration, run `colcon`,
launch ROS nodes, run normal bringup, or start rosbag recording. The runner:

1. copies `f1tenth_system/f1tenth_stack/config/vesc.yaml` byte-for-byte;
2. creates a persistent recovery lock;
3. applies a temporary `VEL_TO_ERPM` profile, builds and launches an exclusive
   calibration stack;
4. records every topic in separate MCAP bags for all velocity/ERPM/drag/current
   stages;
5. applies a temporary `ACCEL_TO_CURRENT` bootstrap profile and runs interface
   checks;
6. performs strict offline fitting and validation;
7. when the candidate passes every gate, applies it **temporarily**, builds,
   and captures independent VEL_TO_ERPM and ACCEL_TO_CURRENT deployment
   verification stages;
8. restores the exact original VESC YAML and builds again.

The candidate is never installed permanently. Power loss or a forced kill is
recovered with:

```bash
python3 erpm_calibration.py --recover --workspace ~/BachelorProject
```

## Stage design

| Stage | Purpose | Primary fitted/confirmed quantities |
|---|---|---|
| 0 | Motor command-chain audit, wheels elevated | Raw selector ownership and motor topic delivery |
| 1 | Stationary + straight sensor observability | LiDAR ICP noise floor and straight-motion quality |
| 2 | Low-speed launch sweep | Minimum stable motion, launch threshold, deadband evidence |
| 3 | Raw ERPM training plateaus | ERPM-to-ground-speed map |
| 4 | Raw ERPM hold-out plateaus | Independent map RMSE/bias gate and Odom check |
| 5 | Existing `VEL_TO_ERPM` pipeline audit | Current desired-speed → ERPM → ground-speed behaviour |
| 6 | Raw ERPM steps | Command-path, VESC ERPM, and ground-speed delay/rate |
| 7 | Current-zero coastdown | Coulomb, viscous and quadratic drag |
| 8 | Raw drive/brake current training pulses | Drivetrain acceleration-per-amp model |
| 9 | Raw current hold-out pulses | Independent drive/brake model gate |
| 10 | Bootstrap `ACCEL_TO_CURRENT` interface audit | Acceleration command → selected current/brake routing |
| 11 | Temporary candidate `VEL_TO_ERPM` verification | Candidate velocity hold-out |
| 12 | Temporary candidate `ACCEL_TO_CURRENT` verification | Candidate acceleration hold-out |

## Non-circular identification

```text
LiDAR scan matching + IMU yaw seed  → ground-referenced vx
VESC telemetry                      → measured ERPM/current/voltage/temperature
ERPM odometry                        → diagnostic and final Odom calibration target
```

The ICP estimator does not receive odometry position, odometry velocity or
ERPM translation as an initialisation. IMU yaw is only a rotational seed.
Odometry is deliberately **not** used as the speed reference because it is
derived from the same drivetrain signal being calibrated.

## What is and is not identified

See [docs/TUNABLE_PARAMETER_COVERAGE.md](docs/TUNABLE_PARAMETER_COVERAGE.md).

`max_drive_current`, `max_brake_current`, VESC electrical limits and regen
limits are captured and audited but are not fitted as “safe maxima”; that would
require a separate thermal/electrical qualification campaign.

## All-topics recording and redundancy

Every stage runs:

```bash
ros2 bag record -s mcap -a --include-hidden-topics
```

Each stage then verifies a required subset. The bag also preserves:

- raw `/scan` arrays;
- `/tf` and `/tf_static`;
- `/parameter_events`;
- ROS graph snapshots before and after recording;
- dumps of VESC, Ackermann, Odom and motor-selector parameters;
- VESC telemetry: ERPM, motor/input current, voltage, motor/FET temperatures,
  duty cycle and fault code;
- direct raw motor requests, selector outputs and actual VESC command topics;
- IMU and 200 Hz odometry;
- all structured stage, phase, trial and `ACCEPT`/`REDO`/`SKIP` events.

A missing required topic marks the affected stage failed; the raw bag remains
preserved but is excluded from fitting.

## Offline requirements

On the target machine, install once in the ROS Python environment:

```bash
python3 -m pip install -r requirements-offline.txt
```

The run invokes offline analysis automatically before temporary candidate
verification. Missing analysis dependencies therefore stop before the candidate
is applied; all raw collection bags remain intact.
