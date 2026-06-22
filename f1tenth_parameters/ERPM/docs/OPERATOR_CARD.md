# ERPM / longitudinal model-selection operator card

## Only command

```bash
cd ~/BachelorProject/f1tenth_parameters/ERPM
source ~/BachelorProject/install/setup.bash
python3 erpm_calibration.py --workspace ~/BachelorProject
```

Optional no-drive preflight:

```bash
python3 erpm_calibration.py --workspace ~/BachelorProject --preflight-only
```

Do **not** edit `vesc.yaml`, run `colcon`, start normal bringup, start MPC,
MPCC, teleoperation, planners, or `ros2 bag record`. The runner owns temporary
configuration, builds, the dedicated launch, stage-targeted MCAP bags, and
restoration.

After Stage 5 and Stage 9 the runner will spend some time exporting data,
running analysis, and relaunching with an improved temporary profile. That is
automatic; the operator just waits for the next `READY` prompt.

## Before `READY`

- Steering calibration is installed and the car drives straight at zero steering.
- The developer has set `site.straight_usable_length_m` for the actual test
  site. The default full-envelope campaign requires at least 30 m usable
  straight; do not use a 15 m room for the high-demand stages.
- The developer has set
  `operating_envelope.approved_drive_test_current_a` and
  `operating_envelope.approved_brake_test_current_a` to reviewed non-zero
  values. The shipped `0.0` placeholders are an intentional hard stop.
- The site has fixed, geometrically rich LiDAR features and no moving people or
  objects in the measurement zone.
- Battery, tyres, gearing, vehicle mass and payload are normal and documented.
- Emergency stop is reachable. The approved drive/brake test currents in the
  configuration have been reviewed by the responsible developer.

## At every prompt

- `READY`: the vehicle is placed at the requested marked start.
- `ACCEPT`: attempt was clean and safe.
- `REDO`: retain the bagged attempt but exclude it from fitting; repeat without
  a retry cap.
- `SKIP`: retain but omit the condition. Required coverage will later fail if
  an expected condition lacks enough accepted usable captures.
- `ABORT`: stop safely. Raw evidence is preserved and the source VESC YAML is
  restored automatically.

Do not move LiDAR features while a capture is active.

## After forced stop / power loss

Do not drive. Run:

```bash
python3 erpm_calibration.py --recover --workspace ~/BachelorProject
```

This restores the byte-exact original VESC YAML and rebuilds.
