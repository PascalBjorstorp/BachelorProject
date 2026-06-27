"""Light proportional straight-line steering assist for the longitudinal stages.

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

    error = wrap(odom_yaw - yaw_ref)          # heading drift since the run start
    error = deadband(error, deadband_rad)     # ignore tiny errors -> no jitter
    trim  = -steer_sign * kp_heading * error  # small proportional correction
    trim  = clamp(trim, -max_trim, +max_trim) # never a real turn

It is intentionally *just* a P term, with a deadband so small errors are left
alone. It is disabled whenever an intentional non-zero steering angle is
commanded (e.g. the cornering arcs). ``steer_sign`` flips the correction
direction in one place if the car's steering convention is inverted (symptom:
the assist immediately drives the car hard to one side instead of correcting).
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
    kp_heading: float = 0.35      # rad steering per rad of heading error (the P term)
    deadband_rad: float = 0.0087  # ignore heading errors below ~0.5 deg (no jitter)
    max_trim_rad: float = 0.07    # hard bound: assist can never command a real turn
    min_speed_mps: float = 0.3    # below this, hold trim at zero and re-anchor the heading
    steer_sign: float = 1.0       # flip to -1.0 if the assist steers the car the wrong way
    yaw_ref: float | None = None  # absolute heading captured at the start of the run

    def reset(self) -> None:
        self.yaw_ref = None

    def step(self, *, heading: float, speed: float, dt: float) -> float:
        """Return the corrective steering trim (rad) to hold a straight line.

        ``heading`` is the absolute odometry yaw (rad). ``dt`` is accepted for
        call-site compatibility but the proportional hold does not need it.
        """
        if not self.enabled or not math.isfinite(heading):
            return 0.0
        if not (math.isfinite(speed) and abs(speed) > self.min_speed_mps):
            # Not moving: re-anchor the straight direction when motion next starts.
            self.yaw_ref = None
            return 0.0
        if self.yaw_ref is None:
            # First moving sample of this run: this heading is "straight ahead".
            self.yaw_ref = heading
            return 0.0
        error = _wrap(heading - self.yaw_ref)
        if abs(error) <= self.deadband_rad:
            return 0.0
        # Continuous past the deadband so there is no jump at the threshold.
        error -= math.copysign(self.deadband_rad, error)
        trim = -self.steer_sign * self.kp_heading * error
        return max(-self.max_trim_rad, min(self.max_trim_rad, trim))


def from_config(cfg: dict) -> StraightAssist:
    sa = (cfg or {}).get("straight_assist", {}) or {}
    return StraightAssist(
        enabled=bool(sa.get("enabled", True)),
        kp_heading=float(sa.get("kp_heading_rad_per_rad", 0.35)),
        deadband_rad=float(sa.get("deadband_rad", 0.0087)),
        max_trim_rad=float(sa.get("max_trim_rad", 0.07)),
        min_speed_mps=float(sa.get("min_speed_mps", 0.3)),
        steer_sign=float(sa.get("steer_sign", 1.0)),
    )
