"""Light incremental straight-line steering assist for longitudinal stages.

The straight-running stages must actually go straight, not merely abort when
they drift. Open-loop "command 0 steering" cannot do that: mechanical slack and
a small centre offset make the car curve slowly over a longer run. This is a
deliberately small heading-hold: a gentle proportional steering nudge that holds
the car on the heading it had when the run started.

Feedback signal — read this, it is the important design choice:

    The reference is **absolute odometry heading** (``odom`` yaw), anchored once
    at the start of each straight run. It is NOT the integral of the raw gyro
    rate. Integrating ``imu_gz`` accumulates gyro bias (the car slowly steers off
    even when going straight) and feeds gyro noise into the steering (it weaves).
    Odometry yaw is already a smooth, bias-free integrated heading, so a plain
    proportional term on the heading error behaves and does not weave. (This is
    only used to keep the car straight; the longitudinal velocity calibration
    still uses LiDAR scan-matching, never odometry.)

Control law, evaluated each command tick while moving:

    error  = wrap(odom_yaw - yaw_ref)          # heading drift since the run start
    error  = deadband(error, deadband_rad)     # ignore tiny errors -> no jitter
    integ  = clamp(integ + ki_heading*error*dt, -max_integral, +max_integral)
    target = -steer_sign * (kp_heading*error + integ)  # P + bounded I
    target = clamp(target, -max_trim, +max_trim)
    trim   = slew(previous_trim, target)       # tiny servo changes only

Why a bounded integral. A pure proportional term cannot fully reject a *constant*
disturbance (a mechanical steering-centre offset, a slight camber): it settles at
a steady heading error and the car keeps creeping to one side. A small integral
term drives that residual to zero, so the assist is robust to a centre offset
rather than just fighting it. It is made safe by three independent hard bounds,
none of which the integral can defeat:

  * ``max_integral_rad`` clamps the integral's own contribution;
  * ``max_trim_rad`` clamps the *total* P+I output (the assist can never command a
    real turn, only a tiny nudge);
  * the slew/step limiter below caps how fast ``trim`` can move per tick.

Anti-windup: the integral is only accumulated when the combined output is not
already saturated, so it can never wind up past the cap and then lag.

The slew limiter is still the key safety net. Even if heading error appears late,
the assist cannot jump to a large trim; it can only keep nudging the servo by a
small amount each command tick. It is disabled whenever an intentional non-zero
steering angle is commanded (e.g. the cornering arcs). ``steer_sign`` flips the
correction direction in one place if the car's steering convention is inverted
(symptom: the assist immediately drives the car to one side instead of
correcting). Set ``ki_heading`` to 0 to recover the original pure-P behaviour.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


def _wrap(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class StraightAssist:
    enabled: bool = True
    kp_heading: float = 0.16      # rad steering per rad of heading error (the P term)
    ki_heading: float = 0.0       # rad steering per (rad*s) of heading error (bounded I term; 0 disables)
    deadband_rad: float = 0.0025  # ignore only sensor noise; keep making tiny corrections
    max_trim_rad: float = 0.025   # hard bound: assist can never command a real turn
    max_integral_rad: float = 0.02  # hard bound on the integral's own contribution (<= max_trim_rad)
    max_trim_rate_rad_s: float = 0.08  # servo trim slew limit
    max_trim_step_rad: float = 0.0015  # extra per-command jump limit
    min_speed_mps: float = 0.15   # below this, hold trim at zero and re-anchor the heading
    steer_sign: float = 1.0       # flip to -1.0 if the assist steers the car the wrong way
    yaw_ref: float | None = None  # absolute heading captured at the start of the run
    trim_rad: float = 0.0
    integral_rad: float = 0.0     # accumulated integral contribution (pre steer_sign), clamped

    def reset(self) -> None:
        self.yaw_ref = None
        self.trim_rad = 0.0
        self.integral_rad = 0.0

    def step(self, *, heading: float, speed: float, dt: float) -> float:
        """Return the corrective steering trim (rad) to hold a straight line.

        ``heading`` is the absolute odometry yaw (rad). ``dt`` controls the
        slew-limit step so the servo trim changes gradually.
        """
        if not self.enabled or not math.isfinite(heading):
            self.trim_rad = 0.0
            self.integral_rad = 0.0
            return 0.0
        if not (math.isfinite(speed) and abs(speed) > self.min_speed_mps):
            # Not moving: re-anchor the straight direction when motion next starts.
            self.yaw_ref = None
            self.trim_rad = 0.0
            self.integral_rad = 0.0
            return 0.0
        if self.yaw_ref is None:
            # First moving sample of this run: this heading is "straight ahead".
            self.yaw_ref = heading
            self.trim_rad = 0.0
            self.integral_rad = 0.0
            return 0.0
        if not math.isfinite(dt) or dt <= 0.0:
            dt = 0.01
        error = _wrap(heading - self.yaw_ref)
        if abs(error) <= self.deadband_rad:
            err_eff = 0.0
        else:
            # Continuous past the deadband so there is no jump at the threshold.
            err_eff = error - math.copysign(self.deadband_rad, error)
        # Bounded integral with anti-windup: tentatively integrate, hard-clamp the
        # integral's own contribution, and only commit the growth if the combined
        # P+I output is still inside the trim cap. This rejects a constant centre
        # offset without ever winding up past the bound.
        i_max = max(0.0, self.max_integral_rad)
        candidate_integral = self.integral_rad + self.ki_heading * err_eff * min(dt, 0.05)
        candidate_integral = max(-i_max, min(i_max, candidate_integral))
        candidate_target = -self.steer_sign * (self.kp_heading * err_eff + candidate_integral)
        if abs(candidate_target) <= self.max_trim_rad:
            self.integral_rad = candidate_integral
        target = -self.steer_sign * (self.kp_heading * err_eff + self.integral_rad)
        target = max(-self.max_trim_rad, min(self.max_trim_rad, target))
        max_step = min(
            self.max_trim_step_rad,
            max(0.0, self.max_trim_rate_rad_s) * min(dt, 0.05),
        )
        delta = max(-max_step, min(max_step, target - self.trim_rad))
        self.trim_rad = max(-self.max_trim_rad, min(self.max_trim_rad, self.trim_rad + delta))
        return self.trim_rad


def from_config(cfg: dict) -> StraightAssist:
    sa = (cfg or {}).get("straight_assist", {}) or {}
    # Backward-compatible aliases let old session snapshots/config snippets use
    # the safer incremental controller instead of silently falling back.
    kp = sa.get("kp_heading_rad_per_rad", sa.get("ki_heading_rad_per_rad", 0.16))
    max_rate = sa.get("max_trim_rate_rad_s", sa.get("max_rate_rad_s", 0.08))
    return StraightAssist(
        enabled=bool(sa.get("enabled", True)),
        kp_heading=float(kp),
        ki_heading=float(sa.get("ki_heading_rad_per_rad_s", 0.0)),
        deadband_rad=float(sa.get("deadband_rad", 0.0025)),
        max_trim_rad=float(sa.get("max_trim_rad", 0.025)),
        max_integral_rad=float(sa.get("max_integral_rad", 0.02)),
        max_trim_rate_rad_s=float(max_rate),
        max_trim_step_rad=float(sa.get("max_trim_step_rad", 0.0015)),
        min_speed_mps=float(sa.get("min_speed_mps", 0.15)),
        steer_sign=float(sa.get("steer_sign", 1.0)),
    )
