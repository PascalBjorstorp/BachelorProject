# Steering calibration — operator card

## Before starting

- Use a charged Traxxas 3S battery.
- Arrange the fixed room geometry and keep people and movable objects outside the LiDAR field during captures.
- Keep a physical emergency stop available.
- Do **not** edit `vesc.yaml`, run `colcon`, start bringup, MPC, MPCC, teleoperation, a planner, or `ros2 bag record`.
- Start from a sourced workspace only:

```bash
cd ~/BachelorProject/f1tenth_parameters/steering
source ~/BachelorProject/install/setup.bash
python3 steer_calibration.py --workspace ~/BachelorProject
```

The runner temporarily selects `VEL_TO_ERPM`, opens the numeric servo domain for endpoint discovery, builds the workspace, and restores the original configuration automatically after the session.

If the terminal reports an unfinished recovery lock, do not drive. Run the displayed `--recover` command first.

## Prompts

Follow the terminal exactly. Type only the requested words:

```text
READY   start the stated stage or trial
ACCEPT  use this completed driving attempt in later analysis
REDO    repeat the same condition; the failed attempt is preserved but excluded
SKIP    preserve the attempt but exclude this condition
ABORT   stop safely and preserve the session
```

## What changes between stages

| Stage | Put the car… |
|---|---|
| 0: command-path audit | On a stand; wheels cannot drive |
| 1: centre trim | On the ground, at the start of a clear straight lane |
| 1b: on-ground IMU bias | On the ground, flat and level; follows Stage 1 without leaving the ground; hold still |
| 2: end-stops | On a stand; inspect linkage continuously |
| 3–6: all driving tests | On the ground, in the prepared test area |

## End-stop controls

Only during Stage 2:

```text
a / d  coarse negative / positive movement
z / c  fine negative / positive movement
x / v  ultra-fine negative / positive movement
l      mark the current point as last clearly free
r      return to experimentally found centre
q      ABORT
```

Never force a mechanical stop. Mark the last point with visibly free motion and no binding, rubbing, or servo strain. The script asks `CONFIRM` or `REDO`.

## After the session

The terminal prints the session directory. Do not delete anything in it. Give the entire directory to the analyst. The raw MCAP bags include rejected attempts and all ROS topics; the final ACCEPT/REDO/SKIP decision is stored in the event stream.

The runner has already restored the original VESC configuration and rebuilt the workspace. Do not apply the proposed physical endpoint limits until the raw data have been reviewed.
