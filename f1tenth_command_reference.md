# F1Tenth Real Robot — Command Reference (ROS2 Humble)

## PHASE 1: Mapping with FTG + SLAM Toolbox

Drive the car around the track using Follow The Gap while simultaneously building a map with SLAM Toolbox.

### Terminal 1 — VESC Driver Stack
```bash
ros2 launch f1tenth_stack bringup_launch.py mapping_mode:=true
```
> This starts: VESC driver, IMU, ackermann mux, and the custom **Hokuyo SCIP 2.0 LiDAR** at **40 Hz** with 1080 beams.
>
> Mapping mode disables scan splitter and lateral planner.


### Terminal 2 — SLAM Toolbox
```bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/home/f1tenth/BachelorProject/f1tenth_system/f1tenth_stack/config/slam_params.yaml \
  use_sim_time:=false
```
> Uses SLAM Toolbox in async mode for real-time 2D mapping.  
> Config: `f1tenth_system/f1tenth_stack/config/slam_params.yaml`  
> Frames: `ego_racecar/odom`, `ego_racecar/base_link`
> scan topic `/scan`.

### Terminal 3 — FTG Autonomous Driving 
```bash
ros2 launch f1tenth_control ftg_hardware_launch.py max_speed:=2.0 mapping_mode:=true
```
> The car will explore the track.  
> `mapping_mode:=true` enables track boundary extraction.

### Terminal 4 - Save the Map 
```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/BachelorProject/f1tenth_sim/maps/my_track_map \
  --ros-args -p save_map_timeout:=10000.0 -p map_subscribe_transient_local:=true
```

> This saves `my_track_map.yaml` + `my_track_map.pgm` into `f1tenth_sim/maps`.  
---

## PHASE 2: Generate Racing Line from the SLAM Map

Run the raceline planner.

### Generate the Racing Line
```bash
python3 f1tenth_planning/scripts/optimize_trajectory.py
```
> Save CSV to `f1tenth_planning/trajectories/`

---

## PHASE 3: Autonomous Racing — C++ GPU AMCL + Lateral Planner + Pure Pursuit + ROS Bag

Run the car autonomously using the SLAM map for localization, the lateral planner for opponent avoidance, and pure pursuit for path following.

### Architecture Overview
The new stack uses three key pipelines launched across two terminals:

1. **Bringup** (Terminal 1): VESC drivers, Hokuyo LiDAR, **scan splitter** (`/scan` → `/scan_walls` + `/scan_obstacles`), and **lateral planner** (opponent avoidance, publishes `/local_raceline`).
2. **C++ GPU AMCL** (Terminal 2): Map server, GPU-accelerated particle filter (`gpu_amcl_cpp`), odometry fusion (`odom_fused`), and EKF sensor fusion (`ekf_localization`). Subscribes to `/scan_walls` (wall-only beams) for robust localization.
3. **Pure Pursuit** (Terminal 3): Follows the `/local_raceline` produced by the lateral planner (or a static trajectory CSV).

### Terminal 1 — VESC Driver Stack + Scan Splitter + Lateral Planner (on Jetson)
```bash
ros2 launch f1tenth_stack bringup_launch.py
```
> This starts: VESC driver, ackermann mux, Hokuyo LiDAR (40 Hz), **scan splitter** (classifies beams as wall/obstacle), and **lateral planner** (opponent avoidance).
>
> The scan splitter requires `/map` from the localization stack (Terminal 2). It will wait until the map is available.

### Terminal 2 — Localization: C++ GPU AMCL (on Jetson)
```bash
ros2 launch f1tenth_localization cpp_localization.launch.py
```

> This launches the full C++ GPU AMCL localization stack:
> - **gpu_amcl_cpp** — CUDA-accelerated particle filter (subscribes to `/scan_walls`)
> - **odom_fused** — IMU + wheel odom fusion at 200 Hz
> - **ekf_localization** — EKF sensor fusion + TF broadcast at 200 Hz
>
> All parameters are in `f1tenth_localization/config/gpu_amcl_cpp_params.yaml`.

---
### Terminal 3 — Pure Pursuit Controller (on Jetson)
```bash
ros2 launch f1tenth_control pure_pursuit_launch.py \
  trajectory_file:=/home/f1tenth/BachelorProject/f1tenth_planning/trajectories/my_track_raceline.csv \
  max_speed:=3.0 \
  min_lookahead:=1.0 \
  max_lookahead:=2.0 \
  lookahead_gain:=0.10
```

### Terminal 4 — ROS BAG Recording (on Jetson)
```bash
# Record EVERYTHING (large files, but complete):
ros2 bag record -a -o ~/bags/race_run_$(date +%Y%m%d_%H%M%S)
```
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

## Jetson Performance Monitoring

Useful commands for checking system load during mapping or racing.

### CPU + GPU + Memory (all-in-one)
```bash
sudo tegrastats
```
> Live feed showing per-core CPU %, GPU %, RAM, thermals.
> Output: `RAM 3456/7620MB | CPU [45%@1420,32%@1420,...] | GPU 12%@510 | ...`

### CPU only — per-core usage
```bash
htop
```
> Or without htop:
> ```bash
> watch -n 1 'cat /proc/stat | head -9'
> ```

### GPU only
```bash
watch -n 1 'cat /sys/devices/gpu.0/load'
```
> Value is in per-mille (500 = 50%).

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

