# F1Tenth Real Robot — Command Reference (ROS2 Humble)

> **Hardware:** Jetson Orin + Hokuyo UST-10LX + VESC  
> **Workspace:** `~/f1tenth_ws` (or wherever you built)  
> **Always source first:** `source ~/f1tenth_ws/install/setup.bash`

---

## PHASE 1: Mapping with FTG + SLAM Toolbox

Drive the car around the track using Follow The Gap while simultaneously building a map with SLAM Toolbox.

### Terminal 1 — VESC Driver Stack (on Jetson)
```bash
source ~/f1tenth_ws/install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```
> This starts: VESC driver, IMU, joystick teleop, ackermann mux, LiDAR.  
> **Hold L1** on the DS4 controller = manual driving (safety override).  
> **Hold R1** = enable autonomous commands from `/drive`.

### Terminal 2 — SLAM Toolbox (on Jetson)
```bash
source ~/f1tenth_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/path/to/your/slam_params.yaml \
  use_sim_time:=false
```

If you don't have a custom SLAM params file, use the default:
```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false
```

**Important SLAM params to check** (create `slam_params.yaml` if needed):
```yaml
slam_toolbox:
  ros__parameters:
    odom_frame: ego_racecar/odom
    base_frame: ego_racecar/base_link
    map_frame: map
    scan_topic: /scan
    use_sim_time: false
    mode: mapping
    resolution: 0.05
    max_laser_range: 10.0
```

### Terminal 3 — FTG Autonomous Driving (on Jetson)
```bash
source ~/f1tenth_ws/install/setup.bash
ros2 launch f1tenth_control ftg_hardware_launch.py max_speed:=2.0 mapping_mode:=true
```
> Start with **low speed** (2.0 m/s). Hold **R1** on the controller to let FTG drive.  
> The car will avoid obstacles and explore the track.  
> `mapping_mode:=true` enables track boundary extraction.

### Terminal 4 — RViz Visualization (on PC, same ROS_DOMAIN_ID)
```bash
source ~/f1tenth_ws/install/setup.bash
rviz2
```
> Add displays: Map, LaserScan (`/scan`), TF, Odometry (`/odom`).  
> Watch the map build in real-time.

### Save the Map (after driving the full track)
```bash
# On Jetson or PC (wherever map_server/slam is running)
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_track_map --ros-args -p save_map_timeout:=10000
```
> This saves `my_track_map.yaml` + `my_track_map.pgm`.  
> **Keep these files** — you need them for localization and planning.

### Optional: Record a Bag of the Mapping Session
```bash
ros2 bag record -a -o ~/bags/mapping_session
# Or record specific topics to save space:
ros2 bag record -o ~/bags/mapping_session \
  /scan /odom /ego_racecar/odom /tf /tf_static /map \
  /drive /ackermann_cmd /sensors/imu/raw /sensors/core
```

---

## PHASE 2: Generate Racing Line from the SLAM Map

Run the raceline planner on your PC (no need for the car to be on).

### Generate the Racing Line
```bash
cd ~/f1tenth_ws/src/BachelorProject

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
> 5. Show visualization plots

### Without friction circle (faster but less realistic):
```bash
python3 f1tenth_planning/scripts/generate_raceline.py \
  --map ~/maps/my_track_map.yaml \
  --output f1tenth_planning/trajectories \
  --no-friction-circle
```

### Check Vehicle Parameters (tune before generating)
```bash
# Edit vehicle params used by the planner:
nano f1tenth_planning/config/vehicle_params.yaml
```
> Important values: friction coefficient (μ), max acceleration, max deceleration, car width, wheelbase (0.3302m), max steering (0.42 rad).

### Verify the Trajectory
```bash
# The output CSV should look like:
# s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2
head -5 f1tenth_planning/trajectories/*_raceline.csv
```

---

## PHASE 3: Autonomous Racing — GPU AMCL + Pure Pursuit + ROS Bag

Run the car autonomously using the SLAM map for localization and the raceline for path following. Record everything.

### Terminal 1 — VESC Driver Stack (on Jetson)
```bash
source ~/f1tenth_ws/install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

### Terminal 2 — Localization: GPU AMCL (on Jetson)
```bash
source ~/f1tenth_ws/install/setup.bash

# Using gpu_amcl (default, recommended on Jetson with CUDA):
ros2 launch f1tenth_localization real_localization.launch.py \
  map_file:=$HOME/maps/my_track_map.yaml \
  amcl_type:=gpu_amcl

# OR using nav2_amcl (if gpu_amcl has issues):
ros2 launch f1tenth_localization real_localization.launch.py \
  map_file:=$HOME/maps/my_track_map.yaml \
  amcl_type:=nav2_amcl
```

> **Tune particle count if needed:**
> ```bash
> ros2 launch f1tenth_localization real_localization.launch.py \
>   map_file:=$HOME/maps/my_track_map.yaml \
>   amcl_type:=gpu_amcl \
>   min_particles:=500 \
>   max_particles:=2000 \
>   max_beams:=120
> ```

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

### Terminal 3 — Pure Pursuit Controller (on Jetson)
```bash
source ~/f1tenth_ws/install/setup.bash

ros2 launch f1tenth_control pure_pursuit_launch.py \
  trajectory_file:=$HOME/f1tenth_ws/src/BachelorProject/f1tenth_planning/trajectories/my_track_raceline.csv \
  max_speed:=3.0 \
  min_lookahead:=0.2 \
  max_lookahead:=1.5 \
  lookahead_gain:=0.10
```
> **Start with low speed** (3.0 m/s) and increase gradually!  
> Hold **R1** on the controller to enable autonomous driving.  
> **L1** = manual override (grab control back anytime).

### Terminal 4 — ROS BAG Recording (on Jetson)
```bash
source ~/f1tenth_ws/install/setup.bash

# Record EVERYTHING (large files, but complete):
ros2 bag record -a -o ~/bags/pure_pursuit_run_$(date +%Y%m%d_%H%M%S)

# OR record specific topics (recommended, smaller files):
ros2 bag record -o ~/bags/pure_pursuit_run_$(date +%Y%m%d_%H%M%S) \
  /scan \
  /odom \
  /ego_racecar/odom \
  /tf \
  /tf_static \
  /map \
  /drive \
  /ackermann_cmd \
  /sensors/imu/raw \
  /sensors/core \
  /amcl_pose \
  /particle_cloud \
  /pure_pursuit/visualization \
  /diagnostics
```

### Terminal 5 — RViz on PC (for live monitoring)
```bash
source ~/f1tenth_ws/install/setup.bash
rviz2
```
> Add: Map, LaserScan, TF, Path (from pure pursuit viz), PoseArray (particle cloud), Odometry.

---

## PHASE 4: Replay Bags on PC (Foxglove / RViz)

### Copy Bag from Jetson to PC
```bash
# From your PC:
scp -r jetson@<JETSON_IP>:~/bags/pure_pursuit_run_* ~/bags/
```

### Replay in RViz
```bash
source ~/f1tenth_ws/install/setup.bash

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
| `/scan` | LaserScan | Hokuyo LiDAR data |
| `/odom` | Odometry | Wheel odometry (from VESC) |
| `/ego_racecar/odom` | Odometry | Namespaced odom (used by some nodes) |
| `/drive` | AckermannDriveStamped | Autonomous drive commands |
| `/teleop` | AckermannDriveStamped | Joystick commands |
| `/ackermann_cmd` | AckermannDriveStamped | Mux output → VESC |
| `/amcl_pose` | PoseWithCovarianceStamped | Localized pose from AMCL |
| `/particle_cloud` | PoseArray | AMCL particle visualization |
| `/map` | OccupancyGrid | Static map from map_server |
| `/tf`, `/tf_static` | TFMessage | All coordinate transforms |
| `/sensors/imu/raw` | Imu | Raw IMU from VESC |
| `/sensors/core` | VescStateStamped | VESC telemetry (RPM, voltage) |

---

## Network Setup (Jetson ↔ PC)

```bash
# Both machines must have the same ROS_DOMAIN_ID:
export ROS_DOMAIN_ID=0  # Add to ~/.bashrc on both machines

# If using Ethernet directly to Jetson:
# On Jetson:
sudo ip addr add 192.168.0.10/24 dev eth0
# On PC:
sudo ip addr add 192.168.0.15/24 dev eth0

# Test connectivity:
ping 192.168.0.10  # from PC
ros2 topic list     # should show topics from both machines
```

---

## Troubleshooting

### SLAM map looks bad
- Drive **slowly** (1-2 m/s) during mapping
- Make sure odom TF frames match SLAM config (`ego_racecar/odom`, `ego_racecar/base_link`)
- Try `slam_toolbox` instead of `cartographer` if you have issues

### AMCL won't localize / particles diverge
- Set the initial pose first (RViz "2D Pose Estimate")
- Check that `/scan` frame matches the AMCL config
- Increase `min_particles` and `max_particles`
- Check map quality — poor maps = poor localization

### Pure Pursuit oscillates or goes off-track
- Reduce `max_speed`
- Increase `min_lookahead` (e.g., 0.5-1.0m)
- Check that the trajectory CSV coordinates match your map frame
- Verify the `/amcl_pose` is accurate before enabling

### Car doesn't respond to /drive commands
- Make sure **R1 is held** on the controller (mux priority)
- Check `ros2 topic echo /ackermann_cmd` — is the mux forwarding?
- Check VESC connection: `ros2 topic echo /sensors/core`

### Bag files are too large
- Record only the topics listed above instead of `-a`
- Use `ros2 bag record --compression-mode file --compression-format zstd` for compression
