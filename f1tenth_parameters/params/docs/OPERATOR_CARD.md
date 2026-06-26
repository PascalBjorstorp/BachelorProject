# Params Identification Operator Card

## Before Running

- Steering calibration is deployed.
- ERPM/current/accel/brake calibration is deployed.
- LiDAR ground-motion estimation is working.
- Battery is charged and tyres/surface are representative.
- Mark the `9 m x 9 m` skidpad boundary and a preferred `10 m x 10 m` boundary.
- Mark the straight transient corridor with at least `9 m` after stable entry.

## Stage 0: Physical Measurements

Measure and record:

- complete race-ready mass;
- front and rear axle loads;
- wheelbase;
- CG height or a defensible measured prior;
- yaw-inertia prior.

Do not let the optimiser infer these indirectly from tyre data.

## Stage 1: Steady Lateral

- Use fixed effective steering.
- Do not run a path controller that continuously corrects steering.
- Start at `0.15 g`, `0.30 g`, `0.45 g`, `0.60 g`.
- Increase near the limit by `0.05 g` only after reviewing the previous point.
- Stop at first repeatable saturation onset; do not chase a spin.

Initial matrix: `32` captures before near-limit escalation.

## Stage 2: Steering Transients

- Enter the measured corridor already stable at target speed.
- Capture sequence is `0.55 s`.
- Worst `8 m/s` condition needs about `8.65 m` after stable entry for capture
  and braking.

Initial matrix: `36` captures.

## Stage 3: Combined Slip

- Establish steady cornering first.
- Keep steering fixed.
- Apply one short drive or brake pulse.
- Fit only lateral-force reduction from the pure-lateral model.

Initial matrix: `48` captures.

## Stage 4: Limit Laps

Run diversified laps:

- `4` moderate pace;
- `4` coasting-corner laps;
- `4` brake-biased entry laps;
- `4` drive-biased exit laps;
- `8` near-limit laps.

Hold out whole laps during validation. Do not randomly split samples from the
same lap into both fit and validation data.
