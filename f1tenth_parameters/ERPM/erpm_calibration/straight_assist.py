"""Closed-loop straight-line steering assist for the longitudinal stages.

The straight-running stages must actually go straight, not merely abort when
they drift. Open-loop "command 0 steering" cannot do that: mechanical slack and
a centre offset make the car curve, and the operator is left re-running and
hoping. This is a small heading-hold autopilot that trims the steering to null
rotation and lateral motion while keeping the commanded longitudinal behaviour
untouched.

Control law (bounded, rate-limited), evaluated each command tick:

    heading += yaw_rate * dt                      # heading drift since the run start
    trim = -(kp_yaw*yaw_rate + ki_heading*heading + kvy*lateral_velocity)
    trim = clamp(trim, -max_trim, +max_trim)      # never a real turn
    trim = rate_limit(trim, max_rate*dt)

The integral (``ki_heading``) term is what absorbs the steering slack / centre
offset: it converges to exactly the steady trim that holds the heading, instead
of relying on the centre being perfectly calibrated. The proportional and
lateral terms damp transient drift. It is pure feedback on measured motion, so
it only ever *removes* sideways motion; it is disabled whenever an intentional
non-zero steering angle is commanded (e.g. the cornering arcs).
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class StraightAssist:
    enabled: bool = True
    kp_yaw: float = 0.35          # rad steering per (rad/s) yaw rate
    ki_heading: float = 0.8       # rad steering per rad heading error
    kvy: float = 0.10             # rad steering per (m/s) lateral velocity
    max_trim_rad: float = 0.07    # hard bound: assist can never command a real turn
    max_rate_rad_s: float = 0.6   # slew limit on the trim
    min_speed_mps: float = 0.2    # below this, hold trim at zero (no windup at rest)
    heading: float = 0.0
    last_trim: float = 0.0

    def reset(self) -> None:
        self.heading = 0.0
        self.last_trim = 0.0

    def step(self, *, yaw_rate: float, lateral_velocity: float, speed: float, dt: float) -> float:
        """Return the corrective steering trim (rad) to hold a straight line."""
        if not self.enabled:
            return 0.0
        dt = max(0.0, min(0.1, dt)) if math.isfinite(dt) else 0.0
        moving = math.isfinite(speed) and abs(speed) > self.min_speed_mps
        if not (moving and math.isfinite(yaw_rate)):
            # No integral wind-up while stationary; start each run from neutral.
            self.heading = 0.0
            self.last_trim = 0.0
            return 0.0
        self.heading += yaw_rate * dt
        vy = lateral_velocity if math.isfinite(lateral_velocity) else 0.0
        trim = -(self.kp_yaw * yaw_rate + self.ki_heading * self.heading + self.kvy * vy)
        trim = max(-self.max_trim_rad, min(self.max_trim_rad, trim))
        if dt > 0.0:
            step = self.max_rate_rad_s * dt
            trim = max(self.last_trim - step, min(self.last_trim + step, trim))
        self.last_trim = trim
        return trim


def from_config(cfg: dict) -> StraightAssist:
    sa = (cfg or {}).get("straight_assist", {}) or {}
    return StraightAssist(
        enabled=bool(sa.get("enabled", True)),
        kp_yaw=float(sa.get("kp_yaw_rad_per_rad_s", 0.35)),
        ki_heading=float(sa.get("ki_heading_rad_per_rad", 0.8)),
        kvy=float(sa.get("kvy_rad_per_mps", 0.10)),
        max_trim_rad=float(sa.get("max_trim_rad", 0.07)),
        max_rate_rad_s=float(sa.get("max_rate_rad_s", 0.6)),
        min_speed_mps=float(sa.get("min_speed_mps", 0.2)),
    )
