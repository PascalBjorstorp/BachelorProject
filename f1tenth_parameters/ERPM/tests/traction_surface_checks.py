#!/usr/bin/env python3
"""Numerical guards for bounded traction-surface inversion."""
from __future__ import annotations
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'full_stack'))
from traction_surface import invert_monotone_envelope, surface_accel

# A nominal monotone surface should invert accurately across speed/current.
coeff=(0.20, 0.0015, -0.008, -0.001)
for v in (0.2,1.0,2.5):
    previous=-1.0
    for requested in (0.0,0.2,0.5,1.0,1.5):
        current=invert_monotone_envelope(requested,v,coeff,20.0)
        assert 0.0<=current<=20.0
        assert current>=previous-1e-9
        previous=current
        if requested>0.0 and current<20.0:
            # The discrete inverse is deliberately bounded and approximate.
            assert abs(surface_accel(current,v,coeff)-requested)<0.03

# Noisy/non-monotone quadratic still gives a monotone, bounded command.
noisy=(0.22,-0.020,0.0,0.0)
values=[invert_monotone_envelope(q,1.0,noisy,15.0) for q in (0.0,0.2,0.5,0.9,1.2)]
assert all(0.0<=x<=15.0 for x in values)
assert all(b>=a-1e-9 for a,b in zip(values,values[1:]))
assert invert_monotone_envelope(999.0,1.0,noisy,15.0)==15.0
print('traction-surface inversion checks passed')
