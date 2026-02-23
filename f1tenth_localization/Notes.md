# Lateral Planner – Design Notes

## Problem

When following a pre-computed raceline (Pure Pursuit / Stanley), the car has **zero awareness of dynamic obstacles**. If an opponent car sits on the racing line, the controller will drive straight into it. We need a node that:

1. Detects objects that are **not part of the static map** (i.e., the opponent car)
2. Generates a **locally modified raceline** that avoids the opponent
3. Feeds the modified raceline to the existing path-following controllers

## Architecture — Three-Node Pipeline

The system is split into three independent nodes across two new packages:

```
  ┌─────────────────────────────────────────────────────────────────┐
  │ f1tenth_lidar package                                           │
  │                                                                 │
  │  /scan ──► ┌───────────────────┐ ──► /scan_walls ──► GPU AMCL  │
  │            │   Scan Splitter   │                                │
  │  /map ───► │  (distance field  │ ──► /scan_obstacles            │
  │  TF ─────► │   classification) │                        │       │
  │            └───────────────────┘                        │       │
  └─────────────────────────────────────────────────────────┼───────┘
                                                            │
  ┌─────────────────────────────────────────────────────────┼───────┐
  │ f1tenth_lateral_planner package                         ▼       │
  │                                                                 │
  │  /scan_obstacles ──► ┌───────────────────┐                      │
  │  /odom ────────────► │  Lateral Planner  │ ──► /local_raceline  │
  │  TF ──────────────► │  (opponent detect  │ ──► /opponent_marker │
  │  Raceline CSV ────► │   + raceline shift)│                      │
  │                      └───────────────────┘                      │
  └─────────────────────────────────────────────────────────────────┘

                        ┌──────────────┐
  /local_raceline ─────►│ Pure Pursuit │──────► /drive
  /odom ───────────────►│  or Stanley  │
                        └──────────────┘
```

**Key design choices**:

1. **Scan Splitter** is a separate node in `f1tenth_lidar` (not inside the Hokuyo driver). This way it works identically in simulation (where the sim publishes `/scan` directly) and on the real robot (where the Hokuyo driver publishes `/scan`). The splitter uses a precomputed distance field — one lookup per beam, ~0.1 ms total.

2. **AMCL never sees obstacle beams.** The splitter publishes `/scan_walls` (obstacle beams set to `inf`), which AMCL's `valid_mask` filter naturally drops. No AMCL code changes needed.

3. **Lateral Planner** consumes only pre-classified obstacle beams — no redundant scan-to-map work. It clusters them, detects the opponent, and publishes a shifted raceline segment.

4. **Controllers are trajectory-agnostic.** They just follow whatever `/local_raceline` provides. If no opponent is detected, the planner publishes the unmodified raceline segment.

**Controller change required**: Pure Pursuit and Stanley currently load the trajectory CSV once at startup. They need a small addition to also accept a `/local_raceline` topic that overrides the upcoming segment of the global trajectory. Both already have `setTrajectory()` methods that can be called from a new subscriber callback.

---

## Assumptions

- **Exactly 1 opponent car** on track at any time
- The static map is known and served by `nav2_map_server`
- The car is localized in the map frame (via GPU AMCL or Nav2 AMCL)
- The global raceline is pre-computed and available as a CSV

---

## Decision Summary

| Decision | Choice |
|----------|--------|
| Architecture | Three-node pipeline: Scan Splitter → Lateral Planner → Controllers |
| Max obstacles | 1 (opponent car) |
| Detection method | Distance-field lookup at beam endpoints (O(1) per beam) |
| Avoidance method | Lateral raceline shift (pass left or right) with cosine blend |
| Output topic | `/local_raceline` — `nav_msgs/Path` (velocity encoded in `position.z`) |
| Controller changes | Minimal — accept live trajectory override topic via existing `setTrajectory()` |
| Scan Splitter package | `f1tenth_lidar` (also contains Hokuyo driver and diagnostic) |
| Lateral Planner package | `f1tenth_lateral_planner` |
| Language | Python (reuses AMCL distance field logic, fast prototyping) |

---

## Detection: Distance-Field Classification (Scan Splitter)

### Concept

Every LiDAR beam endpoint that lands **far from any known wall** in the precomputed distance field indicates a new object. This is O(1) per beam (a single array lookup), much cheaper than ray-casting which is O(n) per beam.

### Precomputation (once, on map receipt)

The scan splitter subscribes to `/map`, converts the occupancy grid to a binary free/occupied mask, and runs `scipy.ndimage.distance_transform_edt` to get a 2D array where each cell holds the distance (in meters) to the nearest wall. This is identical to the distance field already used in GPU AMCL's sensor model.

### Steps (each scan cycle, ~40 Hz)

1. **Get the laser's pose** in the map frame (from TF: `map → laser`)
2. **Transform scan endpoints** to the map frame
   - For each beam angle θ and range r:
     ```
     x_map = x_laser + r · cos(θ + yaw_laser)
     y_map = y_laser + r · sin(θ + yaw_laser)
     ```
3. **Look up distance field** at each endpoint's map pixel (single array index)
   - If `distance_to_nearest_wall > threshold` → this beam hit something **not in the map** (obstacle candidate)
   - Threshold ≈ 0.3 m (configurable ROS param) to account for localization error and map inaccuracy
4. **Cluster filtering**
   - Require ≥ `min_cluster_size` adjacent obstacle beams to suppress noise from localization jitter

### Output

Two `LaserScan` topics:
- `/scan_walls` — obstacle beams replaced with `inf` (consumed by GPU AMCL)
- `/scan_obstacles` — wall beams replaced with `inf` (consumed by Lateral Planner)

### Implementation

The scan splitter node lives in `f1tenth_lidar/src/scan_splitter_node.py`. All vectorized with NumPy — no Python loops over beams.

---

## Avoidance: Lateral Raceline Shift

### Concept

Once the opponent is detected, temporarily shift the global raceline laterally to pass the opponent on the side with more room.

### Steps

1. **Project opponent onto raceline**
   - Find the closest waypoint on the global raceline to the opponent's position
   - Compute the opponent's **lateral offset** from the raceline (signed: positive = left, negative = right)

2. **Decide passing side**
   - Compare the opponent's position to the track boundaries (available from the raceline generation step)
   - Pass on the side with **more clearance**
   - If both sides are roughly equal, prefer the inside of the upcoming corner

3. **Compute lateral shift profile**
   - Define a window around the opponent along the raceline: from `s_start` to `s_end` (e.g., 2–4 m before and after)
   - Apply a smooth lateral offset within this window using a cosine blend:
     ```
     offset(s) = d_max · 0.5 · (1 - cos(2π · (s - s_start) / (s_end - s_start)))
     ```
     where `d_max` is the maximum lateral shift (enough to clear the opponent + safety margin, e.g., opponent width / 2 + 0.3 m)
   - The offset is zero outside the window → smoothly rejoins the global raceline

4. **Apply offset to raceline waypoints**
   - For each waypoint in the window, shift perpendicular to the raceline heading:
     ```
     x_new = x_orig + offset(s) · cos(heading + π/2)
     y_new = y_orig + offset(s) · sin(heading + π/2)
     ```
   - Recompute heading and curvature for the shifted segment
   - Optionally reduce velocity in the shifted section (tighter path → lower speed)

5. **Publish** the modified trajectory on `/local_raceline`

### Safety Checks

- The shifted path must stay **within track boundaries** (check against the occupancy grid or boundary data)
- If no feasible lateral shift exists (opponent blocks the full track width), fall back to **braking / slowing down** behind the opponent
- Apply a **minimum re-plan distance**: don't start shifting if the opponent is already too close (< 1 m); in that case, emergency brake

---

## Data Flow Summary

```
                    ┌──────────────────┐
  /scan ───────────►│  Scan Splitter   │──► /scan_walls ──────► GPU AMCL ──► Pose (TF)
  /map ────────────►│  (f1tenth_lidar) │──► /scan_obstacles ──┐
  TF ──────────────►│                  │                      │
                    └──────────────────┘                      │
                                                              ▼
                    ┌──────────────────────┐
  /odom ───────────►│   Lateral Planner    │──► /local_raceline ──► PP / Stanley / MPC
  TF ──────────────►│ (f1tenth_lateral_    │──► /opponent_marker    ──► /drive
  Raceline CSV ───►│      planner)        │     (RViz viz)
                    └──────────────────────┘
```

---

## Resolved Decisions

- **Package location**: `f1tenth_lidar` for scan splitting, `f1tenth_lateral_planner` for avoidance planning.
- **Language**: Python — fast prototyping, reuses GPU AMCL distance field logic.
- **Detection**: Distance-field lookup (not ray-casting) — O(1) per beam, reuses existing infrastructure.

## Open Questions

- **Opponent velocity tracking**: Is it worth predicting where the opponent will be, or is the current position sufficient at ~40 Hz update rate?
- **MPC integration**: MPC should consume `/local_raceline` like PP/Stanley for consistency. Needs a subscriber added to the MPC C node.
- **FTG interaction**: Lateral planner has an `/lateral_planner_enable` topic. Disable it when FTG is active (reactive mode doesn't use racelines).

---

## AMCL & Opponent Interference

### Problem

The opponent car blocks LiDAR beams that would normally hit walls. These "unexpected" beams land in free space on the map, giving a near-zero likelihood under the Gaussian component. Only the `z_rand` floor (0.005 per beam) prevents them from killing particle weights entirely. With ~25 blocked beams out of 120, the sensor model becomes significantly noisier and effective sample size drops.

### Solution: Scan Splitter Pre-filters Beams

The scan splitter (in `f1tenth_lidar`) already classifies obstacle vs. wall beams. GPU AMCL subscribes to `/scan_walls` instead of raw `/scan`:

```
  /scan ──► Scan Splitter ──► /scan_walls (obstacle beams = inf) ──► GPU AMCL
```

The AMCL sensor model already filters these out:
```python
valid_mask = (selected_ranges > 0.1) & (selected_ranges < self.config.max_range)
```
So obstacle beams (set to `inf`) become neutral (weight factor = 1.0) instead of destructive (weight factor ≈ 0.005). **No AMCL code changes needed** — just change the subscribed topic from `/scan` to `/scan_walls` in the launch file or config.

### Fallback: Increase z_rand

If pre-filtering is too complex initially, increasing `z_rand` from 0.05 to 0.15–0.20 raises the probability floor for outlier beams. The cost is slightly less precise localization (the sensor model trusts the map less). This can be a quick parameter tune while developing the proper pre-filter.

---

## GPU AMCL: Python vs. C++ Rewrite

### Current Bottlenecks

The scan-to-pose pipeline latency consists of:
1. **Scan deserialization + Python callback entry** — ~1-2 ms (Python overhead, GIL)
2. **Beam subsampling + CPU→GPU transfer** — ~0.3 ms
3. **GPU sensor model** (beam transform, distance field lookup, Gaussian, log-sum) — the main compute
4. **GPU resampling** — ~0.2 ms
5. **Pose extraction + GPU→CPU transfer** — ~0.3 ms
6. **Pose publish** — ~0.2 ms

Steps 3-4 are already on the GPU, so C++ wouldn't help there. A C++ rewrite primarily improves steps 1, 2, 5, and 6 (saving ~2-3 ms total per cycle).

### When C++ Becomes Worth It

If increasing `max_beams` substantially (e.g., from 120 → 270 or 540), the CPU-side overhead of preparing beam arrays and transferring them to GPU becomes a larger fraction. Additionally:

- **More beams → better localization** (especially with opponent beams masked out, more wall beams = more information)
- **More beams → more GPU work** — but the GPU sensor model scales well (it's embarrassingly parallel)
- **C++ + CUDA** eliminates the CuPy kernel launch overhead (~0.1 ms per CuPy op, adds up with many ops)
- **C++ rclcpp** has zero-copy intra-process transport for scans from the LiDAR driver (if both run in the same process via composable nodes) — this alone would save the ~1 ms scan deserialization

### Recommendation

If minimizing scan-to-pose latency is a priority and you want to use more beams:

| Approach | Effort | Latency Improvement |
|----------|--------|-------------------|
| Keep Python, increase `max_beams` to 270 | Trivial (config change) | Slightly worse (more GPU work, same Python overhead) |
| C++ node with CUDA kernels | High (full rewrite) | Best (~2-3 ms saved + zero-copy scan + direct CUDA) |
| C++ node with CuPy via pybind11 | Medium (hybrid) | Moderate (C++ callback + existing GPU kernels) |
| Python but with custom CUDA kernels (CuPy RawKernel) | Medium | Moderate (fuse multiple ops into one kernel launch) |

For a bachelor project timeline, the pragmatic path is:
1. **Now**: Increase `max_beams` to 270 in config, benchmark on Jetson
2. **If latency is too high**: Write a fused CUDA RawKernel for the sensor model (replaces ~6 CuPy calls with 1 kernel launch)
3. **If that's still not enough**: C++ rewrite with CUDA

---

## Relevant Code

| Component | Location | Relevance |
|-----------|----------|-----------|
| **Scan Splitter** | `f1tenth_lidar/src/scan_splitter_node.py` | Classifies beams via distance field, publishes `/scan_walls` + `/scan_obstacles` |
| **Lateral Planner** | `f1tenth_lateral_planner/src/lateral_planner_node.py` | Opponent detection + cosine-blend raceline shift |
| Hokuyo driver | `f1tenth_lidar/src/hokuyo_scip_driver.py` | Raw `/scan` publisher (moved from localization pkg) |
| GPU AMCL  | `f1tenth_localization/gpu_amcl/` | Has distance field code reusable by splitter; subscribes to `/scan_walls` |
| Pure Pursuit | `f1tenth_control/src/nodes/pure_pursuit_node.cpp` | Has `setTrajectory()` — needs subscriber for `/local_raceline` |
| Stanley | `f1tenth_control/src/nodes/stanley_node.cpp` | Same change as Pure Pursuit |
| Raceline generator | `f1tenth_planning/scripts/generate_raceline.py` | Produces the global raceline CSV + track boundaries |
| Vehicle params | `f1tenth_planning/config/vehicle_params.yaml` | Wheelbase, friction, speed limits for feasibility checks |
| Splitter config | `f1tenth_lidar/config/scan_splitter.yaml` | Threshold, cluster size, topic names |
| Planner config | `f1tenth_lateral_planner/config/lateral_planner.yaml` | Safety margin, window size, shift limits |