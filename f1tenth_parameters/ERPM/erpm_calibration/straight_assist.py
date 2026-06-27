"""Light proportional straight-line steering assist for the longitudinal stages.

The straight-running stages must actually go straight, not merely abort when
they drift. Open-loop "command 0 steering" cannot do that: mechanical slack and
a small centre offset make the car curve slowly over a longer run. This is a
deliberately small heading-hold: a gentle proportional steering nudge that
opposes the heading the car has accumulated since the run started.

Control law (proportional, bounded, rate-limited), evaluated each command tick:

    heading += yaw_rate * dt                 # heading drift since the run start
    trim = -kp_heading * heading             # small proportional correction
    trim = clamp(trim, -max_trim, +max_trim) # never a real turn
    trim = rate_limit(trim, max_rate*dt)

It is intentionally *just* a P term on the heading drift. There is no integral
(which previously wound up against the trim bound and over-corrected, swinging
the car past straight) and no proportional term on the raw yaw rate (which fed
gyro noise straight into the steering and made it weave). The proportional
heading term alone settles to a small steady counter-steer that cancels the
slack/centre offset, leaving only a small residual drift — which is the accepted
behaviour. An optional ``kd_yaw`` damping term is available but defaults to off.

It is pure feedback on measured motion, so it only ever *removes* drift, and it
is disabled whenever an intentional non-zero steering angle is commanded (e.g.
the cornering arcs).
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class StraightAssist:
    enabled: bool = True
    kp_heading: float = 0.35      # rad steering per rad of accumulated heading drift (the P term)
    kd_yaw: float = 0.0           # optional damping on yaw rate; leave 0 for a pure-P correction
    max_trim_rad: float = 0.07    # hard bound: assist can never command a real turn
    max_rate_rad_s: float = 0.4   # slew limit on the trim
    min_speed_mps: float = 0.3    # below this, hold trim at zero (no drift integration at rest)
    heading: float = 0.0
    last_trim: float = 0.0

    def reset(self) -> None:
        self.heading = 0.0
        self.last_trim = 0.0

    def step(self, *, yaw_rate: float, lateral_velocity: float, speed: float, dt: float) -> float:
        """Return the corrective steering trim (rad) to hold a straight line.

        ``lateral_velocity`` is accepted for call-site compatibility but the
        proportional heading-hold does not use it.
        """
        if not self.enabled:
            return 0.0
        dt = max(0.0, min(0.1, dt)) if math.isfinite(dt) else 0.0
        moving = math.isfinite(speed) and abs(speed) > self.min_speed_mps
        if not (moving and math.isfinite(yaw_rate)):
            # Start each run from neutral; do not integrate heading while stopped.
            self.heading = 0.0
            self.last_trim = 0.0
            return 0.0
        self.heading += yaw_rate * dt
        trim = -(self.kp_heading * self.heading + self.kd_yaw * yaw_rate)
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
        kp_heading=float(sa.get("kp_heading_rad_per_rad", 0.35)),
        kd_yaw=float(sa.get("kd_yaw_rad_per_rad_s", 0.0)),
        max_trim_rad=float(sa.get("max_trim_rad", 0.07)),
        max_rate_rad_s=float(sa.get("max_rate_rad_s", 0.4)),
        min_speed_mps=float(sa.get("min_speed_mps", 0.3)),
    )
