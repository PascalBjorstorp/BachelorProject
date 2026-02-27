# F1Tenth Real Robot — Command Reference (ROS2 Humble)

> **Hardware:** Jetson Orin + Hokuyo UST-10LX + VESC  
> **Workspace:** `~/BachelorProject` (or wherever you built)  
> **Always source first:** `source install/setup.bash`

---

## PHASE 1: Mapping with FTG + SLAM Toolbox

Drive the car around the track using Follow The Gap while simultaneously building a map with SLAM Toolbox.

### Terminal 1 — VESC Driver Stack (on Jetson)
```bash
source install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py use_scan_splitter:=false use_lateral_planner:=false
```
> This starts: VESC driver, ackermann mux, and the custom **Hokuyo SCIP 2.0 LiDAR** at full **40 Hz**.
> Scan splitter and lateral planner are disabled for mapping (no map available yet).


### Terminal 2 — SLAM Toolbox (on Jetson)
```bash
source install/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/home/f1tenth/BachelorProject/f1tenth_system/f1tenth_stack/config/slam_params.yaml \
  use_sim_time:=false
```
> Uses SLAM Toolbox in async mode for real-time 2D mapping.  
> Config: `f1tenth_system/f1tenth_stack/config/slam_params.yaml`  
> Frames: `ego_racecar/odom`, `ego_racecar/base_link`
> scan topic `/scan`.

### Terminal 3 — FTG Autonomous Driving (on Jetson)
```bash
source install/setup.bash
ros2 launch f1tenth_control ftg_hardware_launch.py max_speed:=2.0 mapping_mode:=true
```
> The car will avoid obstacles and explore the track.  
> `mapping_mode:=true` enables track boundary extraction.

### Terminal 4 — RViz Visualization (on PC, same ROS_DOMAIN_ID)
```bash
source install/setup.bash
rviz2
```
> Add these displays:
> - Map
> - LaserScan (`/scan`)
> - TF
> - Odometry (`/odom`)
>
> Watch the map build in real-time.

### Save the Map (after driving the full track)
```bash
# On Jetson or PC (wherever map_server/slam is running)
# NOTE: map_subscribe_transient_local:=true is REQUIRED — SLAM Toolbox publishes
# /map with TRANSIENT_LOCAL QoS; without this flag map_saver hangs forever.
ros2 run nav2_map_server map_saver_cli \
  -f ~/BachelorProject/f1tenth_sim/maps/my_track_map \
  --ros-args -p save_map_timeout:=10000.0 -p map_subscribe_transient_local:=true
```

> This saves `my_track_map.yaml` + `my_track_map.pgm` into `f1tenth_sim/maps`.  
> **Keep these files** — you need them for localization and planning.

---

## PHASE 2: Generate Racing Line from the SLAM Map

Run the raceline planner on your PC (no need for the car to be on).

### Generate the Racing Line
```bash
python3 f1tenth_planning/scripts/generate_raceline.py \
  --map ~/maps/my_track_map.yaml \
  --output f1tenth_planning/trajectories \
  --visualize
```

> This will:
> 1. Extract track boundaries from your SLAM map
> 2. Compute the minimum curvature racing line
> 3. Generate velocity profile (using friction circle model)
> 4. Save CSV + NPZ to `f1tenth_planning/trajectories/`
>
> The `--visualize` flag opens matplotlib plots showing the extracted track boundaries, the computed racing line overlaid on the map, and the velocity profile along the trajectory. Useful for verifying the result before deploying on the car.

### Without friction circle (faster but less realistic):
```bash
python3 f1tenth_planning/scripts/generate_raceline.py \
  --map ~/maps/my_track_map.yaml \
  --output f1tenth_planning/trajectories \
  --no-friction-circle
```

### Check Vehicle Parameters (tune before generating)
```bash
nano f1tenth_planning/config/vehicle_params.yaml
```
> Key parameters to verify (ask Aksel for measured values):
>
> | Parameter | Description |
> |-----------|-------------|
> | Friction coefficient (μ) | Tyre–surface grip limit |
> | Max acceleration | Longitudinal acceleration cap [m/s²] |
> | Max deceleration | Braking limit [m/s²] |
> | Car width | Used for track-boundary clearance [m] |
> | Wheelbase | Axle-to-axle distance [m] |
> | Max steering angle | Ackermann steering limit [rad] |

---

## PHASE 3: Autonomous Racing — C++ GPU AMCL + Lateral Planner + Pure Pursuit + ROS Bag

Run the car autonomously using the SLAM map for localization, the lateral planner for opponent avoidance, and pure pursuit for path following. Record everything.

### Architecture Overview
The new stack uses three key pipelines launched across two terminals:

1. **Bringup** (Terminal 1): VESC drivers, Hokuyo LiDAR, **scan splitter** (`/scan` → `/scan_walls` + `/scan_obstacles`), and **lateral planner** (opponent avoidance, publishes `/local_raceline`).
2. **C++ GPU AMCL** (Terminal 2): Map server, GPU-accelerated particle filter (`gpu_amcl_cpp`), odometry fusion (`odom_fused`), and EKF sensor fusion (`ekf_localization`). Subscribes to `/scan_walls` (wall-only beams) for robust localization.
3. **Pure Pursuit** (Terminal 3): Follows the `/local_raceline` produced by the lateral planner (or a static trajectory CSV).

### Terminal 1 — VESC Driver Stack + Scan Splitter + Lateral Planner (on Jetson)
```bash
source install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py \
  trajectory_file:=$HOME/f1tenth_ws/src/BachelorProject/f1tenth_planning/trajectories/my_track_raceline.csv
```
> This starts: VESC driver, ackermann mux, Hokuyo LiDAR (40 Hz), **scan splitter** (classifies beams as wall/obstacle), and **lateral planner** (opponent avoidance).
>
> The scan splitter requires `/map` from the localization stack (Terminal 2). It will wait until the map is available.

### Terminal 2 — Localization: C++ GPU AMCL (on Jetson)
```bash
source install/setup.bash

ros2 launch f1tenth_localization cpp_localization.launch.py \
  map_file:=$HOME/maps/my_track_map.yaml
```

> This launches the full C++ GPU AMCL localization stack:
> - **map_server** — serves the static map to AMCL and the scan splitter
> - **gpu_amcl_cpp** — CUDA-accelerated particle filter (subscribes to `/scan_walls`)
> - **odom_fused** — IMU + wheel odom fusion at 200 Hz
> - **ekf_localization** — EKF sensor fusion + TF broadcast at 200 Hz
>
> All parameters are in `f1tenth_localization/config/gpu_amcl_cpp_params.yaml`.

### Terminal 2b — Set Initial Pose (IMPORTANT!)
AMCL needs an initial pose estimate. Either:

**Option A:** Use RViz — click "2D Pose Estimate" and place the arrow on the map.

**Option B:** Publish from terminal:
```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{
  header: {frame_id: "map"},
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}'
```
> Adjust x, y, and orientation to match where you placed the car on the track.
>
> **Tip:** The C++ AMCL also supports setting the initial pose in the YAML file (`initial_pose_x`, `initial_pose_y`, `initial_pose_a` under `gpu_amcl_cpp`). This avoids needing to publish manually each time.

### Terminal 3 — Pure Pursuit Controller (on Jetson)
```bash
source install/setup.bash

ros2 launch f1tenth_control pure_pursuit_launch.py \
  trajectory_file:=$HOME/f1tenth_ws/src/BachelorProject/f1tenth_planning/trajectories/my_track_raceline.csv \
  max_speed:=3.0 \
  min_lookahead:=0.2 \
  max_lookahead:=1.5 \
  lookahead_gain:=0.10
```
> **Start with low speed** (3.0 m/s) and increase gradually!
>
> When the lateral planner is active (from bringup), it publishes a modified raceline on `/local_raceline` that avoids detected opponents. Pure pursuit can subscribe to this topic for dynamic path updates.

### Terminal 4 — ROS BAG Recording (on Jetson)
```bash
source install/setup.bash

# Record EVERYTHING (large files, but complete):
ros2 bag record -a -o ~/bags/race_run_$(date +%Y%m%d_%H%M%S)
```

### Terminal 5 — RViz on PC (for live monitoring)
```bash
source install/setup.bash
rviz2
```
> Add these displays:
> - Map
> - LaserScan (`/scan_walls`)
> - TF
> - Path (`/local_raceline`)
> - PoseArray (particle cloud)
> - Odometry (`/ekf_pose`)

---

## PHASE 4: Replay Bags on PC (Foxglove / RViz)

### Copy Bag from Jetson to PC
```bash
# From your PC:
scp -r f1tenth@<JETSON_IP>:~/bags/pure_pursuit_run_* ~/bags/
```

### Replay in RViz
```bash
source install/setup.bash

# Play the bag:
ros2 bag play ~/bags/pure_pursuit_run_20260219_143000 --clock

# In another terminal, open RViz with sim time:
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```
> Add same displays as before. The `--clock` flag publishes `/clock` so all timestamps match.

### Replay in Foxglove Studio
1. Open [Foxglove Studio](https://foxglove.dev/)
2. **Open local file** → select the bag folder (or the `.db3` file inside)
3. OR use **Foxglove WebSocket bridge** for live streaming:
   ```bash
   # Install bridge:
   sudo apt install ros-humble-foxglove-bridge
   
   # Run bridge while playing bag:
   ros2 launch foxglove_bridge foxglove_bridge_launch.xml
   
   # In another terminal:
   ros2 bag play ~/bags/pure_pursuit_run_20260219_143000 --clock
   ```
4. In Foxglove: Connect to `ws://localhost:8765`

### Useful Foxglove Panels
- **3D Panel**: Map + LaserScan + TF + Path
- **Plot Panel**: `/drive.drive.speed`, `/drive.drive.steering_angle`
- **Raw Messages**: `/amcl_pose`, `/odom`
- **State Transitions**: Compare commanded vs actual speed

---

## Quick Reference — Topic Cheat Sheet

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | LaserScan | Raw Hokuyo LiDAR data (all beams) |
| `/scan_walls` | LaserScan | Wall-only beams (from scan splitter → used by AMCL) |
| `/scan_obstacles` | LaserScan | Obstacle-only beams (from scan splitter → used by lateral planner) |
| `/odom` | Odometry | Wheel odometry (from VESC) |
| `/ego_racecar/odom` | Odometry | Namespaced odom (used by some nodes) |
| `/odom_pose` | Odometry | Fused IMU + wheel odom at 200 Hz (from odom_fused) |
| `/ekf_pose` | PoseStamped | EKF-fused pose at 200 Hz (from ekf_localization) |
| `/drive` | AckermannDriveStamped | Autonomous drive commands |
| `/ackermann_cmd` | AckermannDriveStamped | Mux output → VESC |
| `/amcl_pose` | PoseWithCovarianceStamped | Localized pose from C++ GPU AMCL |
| `/local_raceline` | Path / custom | Modified raceline from lateral planner (avoids opponents) |
| `/map` | OccupancyGrid | Static map from map_server |
| `/tf`, `/tf_static` | TFMessage | All coordinate transforms |
| `/sensors/imu/raw` | Imu | Raw IMU from VESC |
| `/sensors/core` | VescStateStamped | VESC telemetry (RPM, voltage) |

---

## Network Setup (Jetson ↔ PC)

```bash
# Both machines must have the same ROS_DOMAIN_ID:
export ROS_DOMAIN_ID=0  # Add to ~/.bashrc on both machines

# Test connectivity:
ping JETSON_IP  # from PC
ros2 topic list     # should show topics from both machines
```

---

## Troubleshooting

### SLAM map looks bad
- Drive **slowly** (1-2 m/s) during mapping
- Make sure odom TF frames match Cartographer Lua config (`ego_racecar/odom`, `ego_racecar/base_link`)
- Tune `real_time_correlative_scan_matcher.linear_search_window` in the Lua config if the car drives fast
- Check the Cartographer Lua file: `f1tenth_system/f1tenth_stack/config/cartographer_f1tenth.lua`

### AMCL won't localize / particles diverge
- Set the initial pose first (RViz "2D Pose Estimate") or set `initial_pose_x/y/a` in `gpu_amcl_cpp_params.yaml`
- Check that `/scan_walls` is being published (scan splitter needs `/map` and `/scan`)
- Increase `num_particles` in the YAML (default: 2500)
- Check map quality — poor maps = poor localization
- Verify the EKF is publishing TF: `ros2 run tf2_ros tf2_echo map ego_racecar/base_link`

### Pure Pursuit oscillates or goes off-track
- Reduce `max_speed`
- Increase `min_lookahead` (e.g., 0.5-1.0m)
- Check that the trajectory CSV coordinates match your map frame
- Verify the `/amcl_pose` is accurate before enabling

### Car doesn't respond to /drive commands
- Check `ros2 topic echo /ackermann_cmd` — is the mux forwarding?
- Check VESC connection: `ros2 topic echo /sensors/core`

### Bag files are too large
- Record only the topics listed above instead of `-a`
- Use `ros2 bag record --compression-mode file --compression-format zstd` for compression
