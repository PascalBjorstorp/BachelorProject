# F1Tenth Planning

Racing line optimization and trajectory planning for F1Tenth autonomous racing.

## Overview

This package computes time-optimal racing trajectories for the F1Tenth car by:
1. Extracting track boundaries from occupancy grid maps
2. Computing the minimum curvature racing line
3. Generating velocity profiles respecting vehicle dynamics
4. Outputting waypoints for path-following controllers

## Theory

### Racing Line Optimization

The optimal racing line minimizes lap time by finding the path that allows the highest average speed. The key insight is that **cornering speed is limited by tire grip**, and grip depends on the turn radius:

$$v_{max} = \sqrt{\mu \cdot g \cdot R}$$

Where:
- $v_{max}$ = maximum cornering speed
- $\mu$ = tire friction coefficient  
- $g$ = gravitational acceleration (9.81 m/s²)
- $R$ = turn radius (= 1/curvature)

Therefore, minimizing curvature maximizes speed.

### Minimum Curvature Formulation

Given track boundaries, we parameterize the path using lateral deviation $\alpha \in [-1, 1]$ from the centerline:

$$\text{path}(s) = \text{center}(s) + \alpha(s) \cdot \frac{w(s)}{2} \cdot \mathbf{n}(s)$$

Where:
- $s$ = arc length along centerline
- $w(s)$ = track width at position s
- $\mathbf{n}(s)$ = normal vector (perpendicular to centerline)

The optimization minimizes:
$$J = \sum_{i=1}^{N} \kappa_i^2 + \lambda \sum_{i=1}^{N-1} (\alpha_{i+1} - \alpha_i)^2$$

Subject to: $-1 + \epsilon \leq \alpha_i \leq 1 - \epsilon$ (stay within track with safety margin)

### Velocity Profile Generation

Once we have the racing line geometry, we compute the velocity at each point:

1. **Curvature-limited speed**: $v_i = \sqrt{\mu g / \kappa_i}$
2. **Forward pass** (acceleration limited): $v_i = \min(v_i, \sqrt{v_{i-1}^2 + 2 a_{max} ds})$
3. **Backward pass** (braking limited): $v_i = \min(v_i, \sqrt{v_{i+1}^2 + 2 a_{brake} ds})$

### Friction Circle Model (Advanced)

The basic velocity profile treats lateral and longitudinal accelerations independently. The **friction circle** model properly couples them - total tire force is limited:

$$\sqrt{a_x^2 + a_y^2} \leq \mu \cdot g$$

Where $a_y = v^2 \cdot \kappa$ is lateral (centripetal) acceleration.

When cornering, the available longitudinal acceleration is reduced:
$$a_{x,available} = \sqrt{(\mu g)^2 - (v^2 \cdot \kappa)^2}$$

This means you can't accelerate as hard when already using grip for turning.

## Vehicle Parameters

The following parameters are considered:

| Parameter | Symbol | Description |
|-----------|--------|-------------|
| Friction coefficient | $\mu$ | Tire-road friction (0.5-1.0 typical) |
| Max acceleration | $a_{max}$ | Motor power limit |
| Max deceleration | $a_{brake}$ | Braking capability |
| Car width | $w_{car}$ | Safety margin calculation |
| Wheelbase | $L$ | For curvature → steering conversion |
| Max steering | $\delta_{max}$ | Physical steering limit |

## Algorithm

```
1. LOAD track map (occupancy grid)
2. EXTRACT left and right track boundaries
3. COMPUTE centerline as midpoint of boundaries
4. OPTIMIZE racing line using minimum curvature
5. SMOOTH path using cubic splines
6. COMPUTE velocity profile
7. OUTPUT waypoints as (x, y, velocity, heading)
```

## References

1. Heilmeier, A., et al. "Minimum Curvature Trajectory Planning and Control for an Autonomous Race Car." IEEE Transactions on Intelligent Vehicles (2020).
   - https://github.com/TUMFTM/global_racetrajectory_optimization

2. Verschueren, R., et al. "Practical Time-Optimal Trajectory Planning for Robots: a Convex Optimization Approach." IEEE TRO (2014).

3. Christ, F., et al. "Time-Optimal Trajectory Planning for a Race Car Considering Variable Tyre-Road Friction Coefficients." Vehicle System Dynamics (2019).

4. Liniger, A., et al. "Optimization-based autonomous racing of 1:43 scale RC cars." Optimal Control Applications and Methods (2015).
   - Classic paper on miniature car racing

5. F1TENTH Community Resources:
   - https://f1tenth.org/learn.html
   - https://github.com/f1tenth

## Usage

```bash
# Build the package
cd ~/Documents/GitHub/BachelorProject
colcon build --packages-select f1tenth_planning

# Generate racing line for a track (with friction circle model)
python3 f1tenth_planning/scripts/generate_raceline.py \
    --map f1tenth_sim/maps/Spielberg_map.yaml \
    --output f1tenth_planning/trajectories \
    --visualize

# Without friction circle (faster but less realistic)
python3 f1tenth_planning/scripts/generate_raceline.py \
    --map f1tenth_sim/maps/Spielberg_map.yaml \
    --output f1tenth_planning/trajectories \
    --no-friction-circle
```

## Output Format

The generated trajectory is saved as CSV and NPZ files:

**CSV format (TUM compatible):**
```
# s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2
0.000000, -74.123456, 48.654321, 1.234567, 0.012345, 7.500000, 2.100000
0.342000, -73.987654, 48.321098, 1.245678, 0.023456, 7.600000, 1.500000
...
```

**NPZ format keys:**
- `positions`: Nx2 array of (x, y) coordinates
- `velocities`: N array of target velocities
- `accelerations`: N array of target accelerations
- `heading`: N array of heading angles (radians)
- `curvature`: N array of path curvature
- `arc_length`: N array of cumulative arc length

## Files

- `scripts/generate_raceline.py` - Main script for racing line generation
- `config/vehicle_params.yaml` - Vehicle parameters (friction, speed limits, etc.)
- `trajectories/` - Generated trajectory files
