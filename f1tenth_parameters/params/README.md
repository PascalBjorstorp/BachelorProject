# Lateral Dynamics Parameter Identification

This suite is the next identification campaign after steering, ERPM/current,
LiDAR motion, and straight-line speed estimation are calibrated.

It is deliberately config-first. The stage contracts define what data must be
recorded before an optimiser is allowed to fit tyre and chassis parameters.
The important rule is parameter separation: do not fit all tyre, relaxation,
combined-slip, and race residual parameters at once.

## Stages

1. `physical_identification`
   Directly measure mass, axle loads, wheelbase, CG height, and yaw inertia
   prior. These values constrain later tyre fitting.

2. `steady_lateral`
   Fixed-steering circles or arcs on a `9 m x 9 m` minimum pad, preferably
   `10 m x 10 m`. Initial core matrix is `32` captures. Near-limit escalation is
   operator-controlled and stops at first repeatable saturation onset.

3. `steering_transients`
   Short steering doublets at `4`, `6`, and `8 m/s` for yaw inertia and tyre
   relaxation. The worst post-stable corridor requirement is about `8.65 m`,
   excluding speed acquisition.

4. `combined_slip`
   Drive/brake pulses while already cornering. This identifies lateral-force
   reduction under longitudinal demand while pure-lateral parameters remain
   fixed.

5. `limit_laps`
   `24` diversified laps for race-envelope residuals only. Laps are not used to
   refit the full tyre model.

## Commands

Inspect the matrix:

```bash
python3 f1tenth_parameters/params/params_identification.py --check
```

Run the static suite checks:

```bash
python3 f1tenth_parameters/params/tests/file_contract_checks.py
```

## Physical Space

The skidpad stages require a clear `9 m x 9 m` minimum area. The transient
stage assumes the car enters the measured corridor already stable at target
speed; for `8 m/s`, keep about `9 m` after stable entry for capture and braking.
