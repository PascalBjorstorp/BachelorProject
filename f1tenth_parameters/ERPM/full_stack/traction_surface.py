"""Pure, testable helpers for bounded traction-surface current inversion."""
from __future__ import annotations

import math
from typing import Sequence

EPS = 1e-9


def surface_accel(current_a: float, speed_mps: float, coeff: Sequence[float]) -> float:
    c1, c2, c3, c4 = map(float, coeff)
    i = max(0.0, float(current_a))
    v = abs(float(speed_mps))
    return c1 * i + c2 * i * i + c3 * i * v + c4 * i * v * v


def invert_monotone_envelope(
    target_accel_mps2: float,
    speed_mps: float,
    coeff: Sequence[float],
    current_limit_a: float,
    *,
    samples: int = 401,
) -> float:
    """Bounded deterministic inverse of a potentially noisy empirical surface.

    The least-squares quadratic can contain non-monotonic local noise.  The
    physical actuator inverse must remain monotone in demanded longitudinal
    force, so the cumulative maximum force envelope is inverted instead.
    """
    target = float(target_accel_mps2)
    limit = float(current_limit_a)
    if not (math.isfinite(target) and math.isfinite(limit) and limit > 0.0) or target <= 0.0:
        return 0.0
    n = max(int(samples), 3)
    grid = [limit * k / (n - 1) for k in range(n)]
    values = [max(0.0, surface_accel(i, speed_mps, coeff)) for i in grid]
    env: list[float] = []
    running = 0.0
    for value in values:
        running = max(running, value)
        env.append(running)
    if target >= env[-1]:
        return limit
    for k in range(1, n):
        if env[k] >= target:
            lo_i, hi_i = grid[k - 1], grid[k]
            lo_a, hi_a = env[k - 1], env[k]
            if hi_a <= lo_a + EPS:
                return hi_i
            return lo_i + (target - lo_a) * (hi_i - lo_i) / (hi_a - lo_a)
    return limit
