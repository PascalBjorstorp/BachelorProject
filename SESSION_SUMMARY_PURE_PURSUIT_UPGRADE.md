# Pure Pursuit Controller Upgrade Session Summary

## Objective
Build the best possible Pure Pursuit (PP) controller for F1Tenth autonomous racing that:
- Tracks pre-computed raceline accurately
- Achieves maximum sustainable average speed
- Remains callback-driven (no timer loop introduced)
- Respects vehicle physical limits and footprint constraints

---

## Problem Statement
Initial PP controller had no explicit width-awareness or corridor regulation:
- Single-point lookahead geometry ignored vehicle footprint (0.273m width)
- No consideration of track corridor boundaries
- Lookahead values (0.74–1.80m) appeared unrealistically high for tight corners
- Speed regulation lacked physics-grounded lateral acceleration limits or corridor constraints

---

## Vehicle Physical Constants (from `f1tenth_parameters/vehicle_params.yaml`)
- **Wheelbase**: 0.324 m (measured, verified)
- **Vehicle half-width**: 0.1365 m (0.273m total width)
- **Max lateral acceleration**: 7.27 m/s² (μ × g = 0.745 × 9.82)
- **Max steering rate**: 2.8492 rad/s (tested)
- **Max steering angle**: 0.4189 rad (24.0°)

All controller defaults synchronized to these measured/tested values.

---

## Work Completed

### 1. Physics-Aware Speed Regulation
**Files**: `pure_pursuit.hpp`, `pure_pursuit.cpp`
- Added `max_lateral_accel` config parameter (default 7.27 m/s²)
- Implemented lateral acceleration cap: `v_max = sqrt(a_lat_max / |kappa|)`
- Previews both closest-point curvature and commanded steering curvature
- Ensures speed respects physics limits before entering corners

### 2. Callback-Driven Command Shaping  
**Files**: `pure_pursuit_node.cpp`, `pure_pursuit_node.hpp`
- Added steering rate limiter (max 2.8 rad/s, configurable)
- Added acceleration/deceleration command limiters (3.0 / 5.0 m/s², configurable)
- Maintains history of last command + timestamp for smooth transitions
- Resets on enable/disable for deterministic behavior
- All state protected by mutex for thread-safety

### 3. Runtime Parameter Validation
**Files**: `pure_pursuit_node.cpp`
- Strict reject-on-invalid policy for all dynamic parameter updates
- Validates finite values, ranges, and consistency
- Fail-fast error reporting with detailed reason strings

### 4. Safety Hardening
**Files**: `pure_pursuit_node.cpp`, `pure_pursuit.cpp`
- Pose freshness watchdog (configurable timeout, default 0.1s)
- Odometry freshness watchdog (configurable timeout, default 0.2s)
- Non-finite command sanitization before publishing `/drive`
- Explicit safe-stop when disabled or trajectory unavailable
- Trajectory quality checks: rejects non-finite values, too-short paths

### 5. Improved Simulation Fidelity
**Files**: `test_sim_drive_pure_pursuit.cpp`
- Added soft-start behavior to match node startup (2s at 1.0 m/s max)
- Kinematic bicycle model with commanded steering/speed delay
- Lateral acceleration grip scaling (emulates understeer under high demand)
- Track projection for continuous CTE computation
- Wall collision detection using vehicle footprint + safety margin

### 6. Parameter Sweep Infrastructure
**Files**: `test_sim_drive_pure_pursuit.cpp`
- Baseline: launch-like defaults
- Conservative seed: safe parameters for validation
- Random sampling + perturbation around local winners
- Scoring: prioritizes collision-free full-lap runs, then progress and tracking quality
- 500-iteration sweep found stable collision-free 2-lap solution (avg 2.956 m/s)

### 7. Initial Parameter Set (Derived from Sweep)
**Launch defaults now**:
```
min_lookahead: 0.74 m
max_lookahead: 1.80 m
lookahead_gain: 0.11
cte_lookahead_weight: 1.5
cte_lookahead_gain: 0.03
curvature_lookahead_gain: 0.18
curvature_speed_factor: 0.10
curvature_speed_floor_ratio: 0.52
cte_speed_factor: 0.36
cte_speed_floor_ratio: 0.37
max_lateral_accel: 7.27 m/s²
max_steering_rate: 2.8 rad/s
max_accel_cmd: 3.0 m/s²
max_decel_cmd: 5.0 m/s²
```

### 8. Removed Dead Code
- Unused `findLookaheadTarget()` method (replaced by segment-based lookahead)
- Unused launch imports (`PathJoinSubstitution`, `FindPackageShare`)

---

## Current Work: Vehicle Width & Corridor Awareness (Partial Implementation)

### Problem Identified
User observation: lookahead values (0.74–1.80m) seem unrealistically high for real hardware performance, especially in tight corners. Root cause: controller geometry is single-point, agnostic to vehicle footprint and corridor constraints.

### Solution Design
**Extend trajectory format and controller to consider corridor bounds:**

#### Trajectory Extension
- **New CSV columns** (7, 8): `d_left_m`, `d_right_m` (lateral corridor half-widths)
- `TrajectoryPoint` struct now holds `left_bound` and `right_bound` (default infinity if not provided)
- Simulator already parses these from hardware raceline CSV

#### Controller Extension
**New `PurePursuitConfig` parameters**:
```cpp
double vehicle_half_width{0.1365};        // [m] Half physical width
double wall_safety_margin{0.03};          // [m] Extra clearance margin
double corridor_half_width_ref{0.25};     // [m] Reference for full speed
double corridor_speed_floor_ratio{0.20};  // [0..1] Floor for corridor slowdown
double corridor_lookahead_factor{2.0};    // [m/m] Lookahead scaling per usable width
```

#### Runtime Regulation
1. **Footprint clearance check** at closest point:
   - Required half-width = `vehicle_half_width + wall_safety_margin`
   - Available half-width per corridor bounds
   - If either bound violated → speed = 0 (collision)

2. **Lookahead limiting**:
   - Clamp dynamic lookahead by available corridor width
   - Tighter corridors → shorter lookahead → lower speed naturally follows

3. **Speed scaling by corridor**:
   - Scale factor = `usable_half_width / corridor_ref`
   - Apply floor to ensure minimum speed in narrow sections

### Status: Partial Implementation
✅ **Complete**:
- TrajectoryPoint struct extended
- PurePursuitConfig parameters added
- CSV parsing for bounds (columns 7, 8)
- Algorithm compute() updated with corridor checks and regulation

⏳ **Remaining**:
- Wire new parameters through `PurePursuitNode::declareParameters()`
- Wire new parameters through `loadParameters()` and validation callback
- Add to launch file declarations and node parameters
- Update simulator harness to use corridor-aware regulation
- Rebuild and rerun sweep to find new parameter ceiling with width constraints

---

## Files Modified

### Core Algorithm
- `include/f1tenth_control/common/types.hpp` — TrajectoryPoint bounds
- `include/f1tenth_control/algorithms/pure_pursuit.hpp` — PurePursuitConfig corridor params
- `src/algorithms/pure_pursuit.cpp` — Corridor-aware lookahead & speed logic

### Node & Integration
- `include/f1tenth_control/nodes/pure_pursuit_node.hpp` — Command-shaping state, new timeouts
- `src/nodes/pure_pursuit_node.cpp` — Full parameter wiring, validation, command shaping

### Launch & Testing
- `launch/pure_pursuit_launch.py` — All new parameters declared and passed to node
- `test/test_sim_drive_pure_pursuit.cpp` — Soft-start, grip scaling, corridor bounds parsing
- `CMakeLists.txt` — No changes (all .cpp/.hpp modifications)

---

## Build Status
✅ **Current**: Compiles cleanly (colcon build successful)

---

## Next Steps for Claude Opus

### 1. Complete Corridor Parameter Wiring (Node)
**File**: `src/nodes/pure_pursuit_node.cpp`
- Add to `declareParameters()`:
  - `vehicle_half_width`, `wall_safety_margin`, `corridor_half_width_ref`, `corridor_speed_floor_ratio`, `corridor_lookahead_factor`
- Add to `loadParameters()`:
  - Load and clamp all new parameters
- Add to `parametersCallback()`:
  - Track candidates for new parameters
  - Validate finite, positive values where needed
  - Apply to config on success

### 2. Complete Corridor Parameter Wiring (Launch)
**File**: `launch/pure_pursuit_launch.py`
- Declare 5 new `LaunchArgument` entries with defaults matching config
- Add to node parameters dict

### 3. Update Simulator Harness
**File**: `test/test_sim_drive_pure_pursuit.cpp`
- If trajectory bounds available:
  - Enforce footprint + margin clearance in sim collision detection
  - This already happens, verify it's being used in scoring

### 4. Rebuild & Rerun Sweep
Command:
```bash
colcon build --packages-select f1tenth_control --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
./build/f1tenth_control/test_sim_drive_pure_pursuit \
  --trajectory f1tenth_planning/trajectories/hardware_raceline.csv \
  --iterations 500 --laps 2
```

### 5. Interpret & Apply Results
- Expected: Narrower lookahead peaks and lower average speed (respecting physical footprint)
- Take TOP 1 candidate
- Update launch defaults to new ceiling
- Document why values differ from previous iteration

### 6. Iterate if Needed
- Manually tune `corridor_lookahead_factor`, `wall_safety_margin` if available corridor margin is too conservative
- Re-sweep if desired
- Converge on a production-ready parameter set

---

## Key Design Principles Applied

1. **Callback-driven throughout**: No timer loop added; all control runs from pose callback
2. **Physics-grounded**: Limits derived from vehicle specs, not arbitrary tuning
3. **Width-aware**: Footprint and corridor constraints now explicit in geometry
4. **Safe-by-default**: Strict validation, fail-fast errors, explicit safe-stop paths
5. **Simulator fidelity**: Soft-start, delays, grip scaling match real node behavior
6. **Parameter sweep as validation**: Large iterative search to find collision-free baseline

---

## Expected Outcome

A production-ready Pure Pursuit controller that:
- Respects vehicle width constraints in real time
- Adapts lookahead to available corridor space
- Scales speed appropriately in tight corners
- Maintains maximum safe average speed over full laps
- Operates callback-driven with smooth, physically-realistic command shaping
- Compiles cleanly and passes simulation validation

---

## References

- Vehicle parameters: `f1tenth_parameters/vehicle_params.yaml`
- Trajectory format: `f1tenth_planning/trajectories/hardware_raceline.csv` (columns 0–8: s, x, y, psi, kappa, vx, ax, d_left, d_right)
- Copilot instructions: `.github/copilot-instructions.md`
