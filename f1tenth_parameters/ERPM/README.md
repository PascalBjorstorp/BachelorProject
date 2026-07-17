# ERPM calibration implementation library

The operator entry point is the unified [vehicle-calibration campaign](../vehicle_calibration/README.md). Do not run the legacy ERPM CLI as a separate campaign in the 14 × 14 m room.

This directory remains the implementation library used by the unified runner:

- `erpm_calibration/` provides scoped ROS capture stages, motor-selector safety, straight assist, and bag recording.
- `analysis/` provides LiDAR-windowed longitudinal, drag, current, acceleration-interface, and lateral-dynamics analysis.
- `full_stack/` contains candidate adapters and model-port contracts for behaviour that the production stack cannot express.

## Canonical workflow

Start a unified session from the repository root:

```bash
source install/setup.bash
python3 f1tenth_parameters/vehicle_calibration/run_suite.py new
python3 f1tenth_parameters/vehicle_calibration/run_suite.py run \
  --session f1tenth_parameters/vehicle_calibration/runs/<session> --next
```

The canonical campaign owns the order, room preflight, temporary VESC patches, build/recovery transaction, A/B/C validation contract, plots, and LaTex report. It keeps the source `vesc.yaml` unchanged after each command.

## What this library measures

The unified sequence uses this library to distinguish:

| Quantity | Reference / fit | Deployment scope |
| --- | --- | --- |
| ERPM ↔ ground speed | LiDAR multi-registration windows, A/B/C hold-out | Wheel odometry and setup speed only; not racing throttle. |
| Command timing | Raw ERPM response A/C | Diagnostic/model timing. |
| Drag | Integrated LiDAR coast-down trajectories A/C | `ACCEL_TO_CURRENT` drag feed-forward. |
| Desired acceleration ↔ selector current/brake | LiDAR pulse acceleration plus selector echo, A/C | Scalar gains only if new-data validation passes. |
| ACCEL_TO_CURRENT interface | Fresh requested-acceleration grid | Race-control command routing and realised ground acceleration. |
| Effective tyre stiffness | Quasi-steady, balanced LiDAR/IMU arcs A/C | Dynamic bicycle `C_f`, `C_r`, and steering scale within the measured low-slip envelope. |

No ERPM or odometry estimate is used as its own ground-truth reference. The 40 Hz LiDAR is processed as displacement-targeted registrations followed by robust 0.5 s motion windows; raw scan pairs are diagnostics, not independent observations.

## Safety and model-selection rules

The canonical 12 m profile limits tests to 3.0 m/s, 8 A raw drive current, and 15 A raw brake current. It applies a second ERPM interlock independently of the odometry calibration.

Straight-line stages may use a small, recorded heading assist only after physical steering-centre validation. It is disabled for intentional turns. The longitudinal stages use `ACCEL_TO_CURRENT` for race-control validation; the static ERPM map remains only an odometry/setup aid.

If independent validation materially prefers a model that the production stack cannot represent, the campaign fails closed and writes an implementation request. Examples include a nonlinear ERPM speed map, a non-scalar acceleration/current surface, or a nonlinear lateral tyre model. Implement that bounded model first, then restart fresh A/B data with `redo`.

## Legacy configuration

`config/erpm_calibration.yaml` is retained for the implementation library and its regression checks. Its 20 m reference profile is capped at 5.5 m/s, but it is not the room operator workflow. The unified profile in `vehicle_calibration/config/suite.yaml` uses the 14 × 14 m physical room, a 1 m wall band, the clear-area diagonal, and staged A/B/C grids.

For recovery from an interrupted unified temporary configuration transaction:

```bash
python3 f1tenth_parameters/vehicle_calibration/run_suite.py recover \
  --workspace /home/akselmo/Documents/GitHub/BachelorProject
```
