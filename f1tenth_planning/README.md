# F1Tenth Planning

Trajectory optimization for the F1Tenth car. Takes a SLAM map and produces an optimized raceline CSV that the MPC controller follows.

## Quick Start

```bash
# Default: minimum-curvature optimization on my_track_map
python3 f1tenth_planning/scripts/optimize_trajectory.py

# Time-optimal (mintime) using IPOPT/CasADi — slower but better velocity profile
python3 f1tenth_planning/scripts/optimize_trajectory.py --opt-type mintime
```

The output lands in `f1tenth_planning/trajectories/<track>_raceline.csv`.

## What the Pipeline Does

`optimize_trajectory.py` runs four steps automatically:

1. **Extract centerline** — reads the `.pgm` map, finds track boundaries, computes the GVD centerline, and measures track widths via ray-cast.
2. **Optimize raceline** — feeds the centerline + widths into the [TUM global trajectory optimizer](global_racetrajectory_optimization/) which finds the racing line that minimizes curvature (or lap time in mintime mode).
3. **Convert to MPC format** — rotates headings by π/2 (TUM→F1Tenth convention), clamps velocities, and writes a comma-separated CSV.
4. **Add wall distances** — ray-casts perpendicular to each waypoint to get `d_left` and `d_right` for the MPC's lateral error tracking.

## Optimization Modes

| Mode | Flag | Description |
|------|------|-------------|
| Minimum curvature | `--opt-type mincurv` (default) | Minimizes path curvature → maximizes cornering speed. Fast, good results. |
| Minimum curvature IQP | `--opt-type mincurv_iqp` | Iterative QP variant. Similar quality, sometimes tighter to boundaries. |
| Shortest path | `--opt-type shortest_path` | Minimizes path length. Aggressive corner cutting. |
| Minimum time | `--opt-type mintime` | Full vehicle dynamics optimization via IPOPT/CasADi. Accounts for tire limits, motor torque curve, and friction circle. Best velocity profile but requires `pip install casadi`. |

## Common Options

```bash
python3 f1tenth_planning/scripts/optimize_trajectory.py \
    --map f1tenth_sim/maps/my_track_map.yaml \  # Map file (auto-detected if omitted)
    --opt-type mincurv \                         # Optimization mode
    --max-speed 8.0 \                            # Velocity clamp [m/s]
    --min-speed 2.0 \                            # Velocity floor [m/s]
    --smooth-factor 2.0 \                        # Spline smoothing (s_reg). Lower = safer, higher = smoother
    --centerline-points 300 \                    # Centerline resolution
    --car-width 0.273 \                          # Physical car width [m]
    --wall-clearance 0.02 \                      # Extra gap from walls [m]
    --direction cw \                             # Track direction (cw / ccw / auto)
    --waypoint-spacing 0.15                      # Final waypoint density [m]
```

### Tuning Tips

- **`--smooth-factor`** (`s_reg`): Controls how much the raceline smooths the centerline shape. `0.5` = safe/conservative, `2.0` = good balance, `10+` = aggressive (may cut corners too close on small tracks).
- **`--centerline-points`**: 300 works well for small tracks (~22 m). More points is not always better — it can make the optimizer sluggish and produce worse results on short tracks.
- **`--wall-clearance`**: `0.02` leaves 2 cm beyond the car edge. Increase if the MPC overshoots corners.

### Skipping Steps

```bash
# Reuse previously extracted centerline (skip step 0)
python3 f1tenth_planning/scripts/optimize_trajectory.py --skip-extract

# Reuse previous optimization, only re-convert to MPC format (skip steps 0+1)
python3 f1tenth_planning/scripts/optimize_trajectory.py --skip-extract --skip-optimize
```

## Output Format

The raceline CSV has 9 columns:

```
# s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2, d_left_m, d_right_m
0.000, 4.317, -4.834, -3.035, 0.012, 3.50, 1.20, 0.18, 0.22
...
```

| Column | Description |
|--------|-------------|
| `s_m` | Arc length along the raceline [m] |
| `x_m`, `y_m` | World coordinates [m] |
| `psi_rad` | Heading angle [rad] |
| `kappa_radpm` | Curvature [rad/m] |
| `vx_mps` | Target velocity [m/s] |
| `ax_mps2` | Target longitudinal acceleration [m/s²] |
| `d_left_m` | Distance to left wall [m] |
| `d_right_m` | Distance to right wall [m] |

## Vehicle Parameters

Vehicle parameters for the optimizer live in two places:

- **`config/vehicle_params.yaml`** — used by the pipeline for car width, friction, speed limits.
- **`global_racetrajectory_optimization/params/racecar.ini`** — used by TUM optimizer for mintime mode (tire model, motor curve, mass, inertia). The pipeline auto-patches `width_opt`, `s_reg`, and step sizes into this file during optimization and restores it afterwards.

## Files

```
f1tenth_planning/
├── scripts/
│   ├── optimize_trajectory.py    # Main pipeline — this is what you run
│   └── compute_wall_distances.py # Wall ray-cast helper (called by pipeline)
├── config/
│   └── vehicle_params.yaml       # Vehicle parameters
├── global_racetrajectory_optimization/  # TUM optimizer (submodule)
│   ├── main_globaltraj.py
│   └── params/racecar.ini
├── trajectories/                 # Output directory for raceline CSVs
├── launch/                       # ROS2 launch files
└── CMakeLists.txt
```

## Dependencies

```bash
pip install numpy opencv-contrib-python scipy pyyaml matplotlib

# Only needed for --opt-type mintime:
pip install casadi
```
