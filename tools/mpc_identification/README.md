# F1TENTH hardware-to-simulation identification

This is a reproducible pipeline for fitting your **open-loop vehicle plant** to
the recorded five-lap MPC run. It uses the recorded `/map` occupancy grid in the
MCAP as the static scan-matching map, the supplied LiDAR-to-`base_link`
transform, and **multiple shooting** so that controller compensation cannot hide
plant error.

It runs in Python; no ROS installation is required for the supplied MCAP. The
final output is a shell file that can be sourced before compiling/running your
existing `test_sim_drive.c` simulation.

## Inputs

You need:

- `shortened_baseline_0.mcap` extracted from `Baseline.zip`;
- the exact `raceline.csv` used for the run;
- this directory.

The fixed LiDAR extrinsic encoded in the ICP command is:

```text
base_link <- laser: x=+0.265 m, y=0.0 m, yaw=0.0 rad
```

## Install

```bash
cd mpc_identification
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt
```

## Run

### One command: prepare, fit, held-out test, plots

```bash
./run_all.sh \
  /absolute/path/to/shortened_baseline_0.mcap \
  /absolute/path/to/raceline.csv
```

### Two-stage run: stop after data preparation if inspection is needed

```bash
./run_prepare.sh /absolute/path/to/shortened_baseline_0.mcap /absolute/path/to/raceline.csv
./run_fit.sh output
```

The preparation stage writes:

- `output/extracted/manifest.json` — actual topic inventory and message counts;
- `output/icp_map/` — map points and preview;
- `output/icp_poses.csv` — per-scan scan-to-map ICP pose and quality;
- `output/observed_run.csv` — timestamp-aligned state/input table;
- `output/segments.csv` — five-lap train/validation/test multiple-shooting windows;
- `output/actuator_fit.json` — steering command-domain fit.

The default preparation run omits `/local_raceline` snapshots because decoding all 9,339 ROS `Path` messages is slow and not required for open-loop plant fitting. Export them only before the later closed-loop historical-reference replay:

```bash
./run_prepare.sh --include-local-raceline /absolute/path/to/shortened_baseline_0.mcap /absolute/path/to/raceline.csv output
```

The fitting stage writes:

- `output/longitudinal_metrics.json`;
- `output/lateral_metrics.json`;
- `output/test_metrics.json` — held-out fifth-lap evaluation;
- `output/test_predictions.csv`;
- `output/plots/`;
- `output/fitted_sim_env.sh` — environment overrides for `test_sim_drive.c`.

## Return results for analysis

Create a compact archive:

```bash
./package_results.sh output identification_results.zip
```

Upload `identification_results.zip`. Keep the original MCAP locally; it is not
included in this compact archive.

## Using fitted parameters in the C simulator

The fit is deliberately performed in open loop. Only after the held-out metrics
are inspected should you assess closed-loop MPC parity.

```bash
source output/fitted_sim_env.sh
# Run the compile command already documented at the top of test_sim_drive.c.
SIM_DURATION=46.8 SIM_DT=0.002 MPC_DT=0.005 PRED_DT=0.03 ./test_sim_drive
```

`fitted_sim_env.sh` exports exactly the `SIM_*`, `DRAG_*`, and `ACCEL_*`
variables read by your current `test_sim_drive.c`. It does **not** export
`SIM_MU` or `SIM_PACEJKA_C`, because the active C force model uses
front/rear-specific parameters instead.

### Tyre relaxation (model change)

The identified plant now includes first-order **tyre relaxation**: lateral force
follows the slip angle with time constant `sigma/v`. This is the dominant
transient-yaw effect and is worth roughly +9 deg yaw and +15 cm position on the
held-out lap, so `MPC/test/test_sim_drive.c` was extended to integrate it (two
extra plant states `alpha_f_lag`, `alpha_r_lag`). It is controlled by
`SIM_REL_LEN_FRONT` / `SIM_REL_LEN_REAR`, both exported by `fitted_sim_env.sh`.
With either set to `0` (the default when the file is not sourced) the plant
recovers the previous instantaneous-force behaviour exactly, so existing runs are
unaffected.

## Closed-loop fit (match the recorded lap, no crash)

The open-loop fit gives a physically faithful plant, but the goal is that the
**closed-loop** simulation reproduces the recorded run: the real controller drives
the simulated car around the lap matching the ICP position and speed, without
crashing. Two things are required:

1. **One sanctioned controller change**: remove the `*1.4` multiplier from
   `VP_MAX_ACCEL_MPS2` in `MPC/include/mpc_types.h`. The bag was recorded without
   it; with it the controller over-drives the car and crashes. This is the only
   permitted change to the controller.
2. Run the closed-loop search, which drives the actual compiled simulator and
   tunes plant parameters to minimise the sim-vs-ICP position and speed error
   (odom is never used as truth — its ERPM-derived speed reads high in corners),
   with a hard penalty for any lap it fails to finish:

```bash
./run_closed_loop.sh output            # after run_all.sh
# then:
source output/fitted_sim_env.sh
SIM_DURATION=60 SIM_DT=0.002 MPC_DT=0.005 PRED_DT=0.03 \
  REALISTIC_TIRES=1 REALISTIC_ACTUATION=1 REALISTIC_DRIVE=1 ./test_sim_drive
```

Result on the supplied bag: the car completes laps with ~6–7 cm lateral tracking
of the real line and simulated average speed within ~0.05 m/s of the real car.
See `output/plots/closed_loop_vs_real.png`.

## Method constraints

- Scan-to-map ICP uses EKF only as an initial seed; every pose carries RMSE,
  inlier count, and a geometry-condition metric.
- ICP is a high-quality pose source, not an assertion of perfect ground truth.
- The velocity state (speed, sideslip, yaw rate) used to initialise and score
  each multiple-shooting window is derived from the ICP pose itself (smoothed and
  differentiated), not from odometry. odom is biased relative to the ICP position
  truth (speed +0.24 m/s, sideslip understated ~2x, yaw rate +0.07 rad/s), and
  those biases were the dominant fit error until this was changed. odom/IMU remain
  available in `observed_run.csv` as a cross-check and as a fallback.
- Shooting windows are short (0.6 s) so the metric reflects the per-control-cycle
  predictive accuracy that governs closed-loop MPC parity, rather than whole-lap
  open-loop drift.
- The steer-feedback result may represent command-domain echo feedback. The
  merge step therefore does not automatically replace the known steering rate
  limit or introduce a steering bias into the plant.
- The optimizer fits actuator timing, longitudinal response, then lateral tyre
  parameters. It resets the simulator at every 1.25 s segment start.
