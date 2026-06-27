# ERPM / longitudinal model-selection operator card

## Commands

Tier 1 — config-deployable calibration (run this first):

```bash
cd ~/BachelorProject/f1tenth_parameters/ERPM
source ~/BachelorProject/install/setup.bash
python3 erpm_config_calibration.py --workspace ~/BachelorProject
```

Tier 2 — candidate shadow verification (only when validating a C++ port), on the
Tier-1 session:

```bash
python3 erpm_candidate_port.py --session runs/<session-id> --workspace ~/BachelorProject
```

Optional no-drive preflight:

```bash
python3 erpm_config_calibration.py --workspace ~/BachelorProject --preflight-only
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
  site. The default high-demand campaign requires at least 20 m usable straight;
  25-30 m is helpful if 10 m/s settled captures are desired.
- The developer has set
  `operating_envelope.approved_drive_test_current_a` and
  `operating_envelope.approved_brake_test_current_a` to positive command-scale
  values.
- The site has fixed, geometrically rich LiDAR features and no moving people or
  objects in the measurement zone.
- Stage 13 uses straight run-up plus short turn segments up to the configured
  7.5 m/s² expected lateral-acceleration budget; it does not require full
  circles.
- Stage 14 runs after candidate verification and uses steering-step captures
  for later high-speed steering/yaw dynamics calibration.
- Battery, tyres, gearing, vehicle mass and payload are normal and documented.
- Emergency stop is reachable. The raw-current command scale in the
  configuration has been reviewed by the responsible developer.

## At every prompt

- `READY`: the vehicle is placed at the requested marked start.
- `ACCEPT`: attempt was clean and safe.
- `REDO`: retain the bagged attempt but exclude it from fitting; repeat without
  a retry cap.
- `SKIP`: retain but omit the condition. Required coverage will later fail if
  an expected condition lacks enough accepted usable captures.
- `ABORT`: stop safely. Raw evidence is preserved and the source VESC YAML is
  restored automatically.

Do not move LiDAR features while a capture is active. Do not leave READY/review
prompts open longer than needed: each stage bag stays open while waiting, and
the LiDAR stream is still being recorded.

## After forced stop / power loss

Do not drive. Run:

```bash
python3 erpm_config_calibration.py --recover --workspace ~/BachelorProject
```

This restores the byte-exact original VESC YAML and rebuilds.
