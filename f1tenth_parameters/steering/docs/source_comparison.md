# External practice informing this steering-calibration suite

This suite is not copied from one team. It combines the operational patterns
that are defensible for this vehicle and sensor set.

## F1TENTH ecosystem

- The standard F1TENTH VESC interface exposes a steering-angle-to-servo
  calibration interface. This confirms that F1TENTH deployments normally need
  vehicle-specific steering conversion rather than relying on a universal
  geometric constant.
- The ForzaETH race stack publishes a dedicated `SysID` checklist, indicating
  that a competitive F1TENTH team treats identification as an explicit
  procedure rather than an ad-hoc controller adjustment.
- The ETH PBL MAP Controller repository contains an explicit `steering_lookup`
  subsystem. That is consistent with retaining a lookup-table representation
  when one linear conversion is not accurate enough.

These repository observations establish architectural precedent. They do not,
by themselves, establish the exact calibration manoeuvres used by those teams.

## Research evidence

1. Gonultas, Mukherjee, Poyrazoglu and Isler (2023), *System Identification
   and Control of Front-Steered Ackermann Vehicles through Differentiable
   Physics*. The paper identifies parameters of a front-steered Ackermann
   vehicle from real driving data and validates control behaviour on F1TENTH.
   It supports logged-motion identification and held-out validation.

2. Rödönyi et al. (2021), *Identification of the nonlinear steering dynamics
   of an autonomous vehicle*. The paper identifies steering dynamics from
   measured data and finds that a simple nominal component model can be
   inadequate. It supports identifying static behaviour and dynamics
   separately rather than assuming one gain.

3. Evans et al. (2024), *Unifying F1TENTH Autonomous Racing: Survey, Methods
   and Benchmarks*. The survey identifies localisation accuracy and control
   frequency as relevant experimental factors. It supports retaining raw data,
   validating timing, and separating estimation quality from controller tuning.

## Design choices derived here

| Requirement | Suite mechanism |
|---|---|
| Unknown straight-ahead point | Low-speed yaw-rate sweep, not servo midpoint |
| Asymmetric mechanical range | Separate low/high raw-servo end-stop measurements |
| Unknown static map | Raw-servo targets, independent curvature estimate and side-specific LUT |
| Unknown actuator/vehicle lag | Dedicated command-to-curvature step tests |
| Avoid re-running tests for changed estimator | MCAP raw bags, raw scans retained |
| Avoid fitting to own data | Separate hold-out static-map stage |

## Source links

- https://github.com/f1tenth/f1tenth_system
- https://github.com/ForzaETH/race_stack/blob/master/stack_master/checklists/SysID.md
- https://github.com/ETH-PBL/MAP-Controller/tree/main/steering_lookup
- https://arxiv.org/abs/2308.03898
- https://arxiv.org/abs/2105.04529
- https://arxiv.org/abs/2402.18558
