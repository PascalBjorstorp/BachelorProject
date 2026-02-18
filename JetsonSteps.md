# Project Plan: F1TENTH Localization on Jetson

Things to ask opus
1. Could there be made a setup where there is both a amcl loop (40hz) and a control loop

2. Can AMCL be made global 


## 1. Simulation Setup & Baseline Driving
**Status:** Already running simulation with Follow-the-Gap (FTG); Pure Pursuit can be swapped in later (doesn’t affect localization benchmarking).
**Goal:** Have a repeatable, controllable simulation environment for localization experiments.

## 2. Performance Logging Script
**What to log:**
- CPU usage (system-wide and per-process)
- GPU usage (Jetson-specific, e.g., using tegrastats or jtop)
- Lidar-to-pose latency: Time from /scan message received to new pose estimate published (can be measured by timestamping in a ROS node)

**How:**
- Write a Python or C++ ROS node that:
    - Subscribes to /scan and pose output (e.g., /amcl_pose)
    - Logs timestamps for each message and computes latency
    - Periodically queries CPU/GPU usage (see below)
    - Writes all data to a CSV or similar for later plotting
- For CPU/GPU usage:
    - Use Python’s psutil for CPU
    - Use subprocess to call tegrastats or jtop for GPU (Jetson)
    - Optionally, use ROS 2 diagnostics or system monitor nodes

## 3. Benchmark nav2 AMCL
- Run nav2 AMCL in simulation
- Log:
    - CPU/GPU usage
    - Lidar-to-pose latency
    - Pose accuracy (compare AMCL output to simulation ground truth)
- Plot results in Python or MATLAB

## 4. Develop Your Own GPU-based AMCL
- Start with a CPU-based MCL (Monte Carlo Localization) in Python or C++ for correctness.
- Port the particle update and likelihood computation to GPU (using CUDA, Numba, or OpenCL).
- Integrate with ROS 2 for /scan, /map, /odom, and pose output.
- Benchmark as above.

## 5. Implement Scan Matching
- Develop or adapt a scan matching algorithm (e.g., NDT, ICP, or CSM) using GPU acceleration if possible.
- Integrate and benchmark.

## 6. Comparative Testing
- Test all three approaches (nav2 AMCL, your GPU AMCL, scan matching) under identical simulation conditions.
- Log and plot:
    - Resource usage
    - Latency
    - Accuracy
    - Robustness (e.g., recovery from localization loss)

## 7. Hybrid System Integration
- Develop logic to switch between global (AMCL) and local (scan matching) localization.
- Test system under normal and recovery scenarios.

---

## Script/Tool Suggestions

**CPU usage:**
- Use Python’s psutil or ROS 2 diagnostics

**GPU usage (Jetson):**
- Use tegrastats: `sudo tegrastats`
- Or jtop: `sudo -E jtop`

**ROS 2 message timing:**
- Subscribe to /scan and /amcl_pose, log header.stamp and wall time.

---

## Summary Table

| Step                | Tool/Script Needed                        | Output/Goal                  |
|---------------------|-------------------------------------------|------------------------------|
| Sim + FTG           | Already running                           | Baseline driving             |
| Logging             | Python/C++ ROS node + psutil/tegrastats   | CSV of CPU/GPU/latency       |
| nav2 AMCL           | Standard ROS 2 node                       | Baseline localization        |
| GPU AMCL            | Custom node (start CPU, port to GPU)      | Improved performance         |
| Scan Matching       | Custom node (GPU if possible)             | Local tracking               |
| Comparative Test    | Python/MATLAB plotting                    | Performance/accuracy plots   |
| Hybrid System       | Custom logic                              | Robust localization          |

---

You’re on the right track! If you want example code for logging, or advice on GPU programming for Jetson, just ask!
---

## Current Progress Status (2026-02-01)

### What's Been Done:
1. **TF Frame Configuration Fixed**
   - Modified `f1tenth_sim/f1tenth_gym_ros/gym_bridge.py` to have configurable TF frames
   - Added `tf_frame_id` and `odom_frame_id` parameters (both default to `'odom'`)
   - Updated `f1tenth_sim/config/sim.yaml` with new frame settings
   - TF tree is now: `map → odom (AMCL) → ego_racecar/base_link (simulation)`

2. **Launch Files Simplified** (in `f1tenth_localization/launch/`):
   - `amcl.launch.py` - AMCL only, configurable particles and update thresholds
   - `performance_monitor_launch.py` - Just the performance monitor
   - `sim_headless.launch.py` - Simulation without GUI (optional)

3. **Performance Monitor Created** (`f1tenth_localization/scripts/performance_monitor.py`):
   - Tracks AMCL-specific CPU usage (not just system-wide)
   - Shows: instant CPU %, running average, peak
   - Logs to CSV at `/tmp/f1tenth_performance/`
   - Configurable sample rate (default 50 Hz to catch spikes)
   - Jetson GPU monitoring via sysfs

4. **AMCL Launch Configurable**:
   - `min_particles` / `max_particles` - particle count
   - `max_beams` - laser beams used
   - `update_min_d` / `update_min_a` - movement thresholds (lower = more frequent updates)

### Known Issues:
1. **DDS Cross-Version Errors**
   - "sequence size exceeds remaining buffer" spam from ROS 2 Jazzy (PC) ↔ Humble (Jetson)
   - Partial workaround: Export `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` on both machines
   - Still getting some serdata.cpp errors

2. **AMCL CPU Reading Shows 0%**
   - Performance monitor finds AMCL process but CPU reads 0%
   - Pose rate showing 100 Hz is suspicious (AMCL can't do 100 Hz)
   - **Likely cause**: The poses might be coming from simulation, not AMCL
   - **To verify**: Run `ros2 topic info /amcl_pose` to check publisher

3. **Sample Rate Not Updated**
   - Last test showed "Sample rate: 10.0 Hz" instead of 50 Hz
   - **Fix**: Rebuild package: `colcon build --packages-select f1tenth_localization && source install/setup.bash`

### Where to Pick Up Next Time:

#### Immediate Debugging Steps:
1. **Verify AMCL is actually publishing poses**:
   ```bash
   ros2 topic info /amcl_pose
   ros2 topic echo /amcl_pose --once
   ```

2. **Check what nodes are running**:
   ```bash
   ros2 node list | grep amcl
   ros2 node info /amcl
   ```

3. **Rebuild packages after changes**:
   ```bash
   cd ~/BachelorProject
   colcon build --packages-select f1tenth_localization
   source install/setup.bash
   ```

4. **Test AMCL with higher particle count to stress CPU**:
   ```bash
   ros2 launch f1tenth_localization amcl.launch.py min_particles:=5000 max_particles:=10000
   ```

#### Multi-Machine Setup Reminder:
- **PC (simulation)**: ROS 2 Jazzy, Ubuntu 24.04
  - Run: `ros2 launch f1tenth_gym_ros gym_bridge_launch.py`
  - Make sure: `tf_frame_id: 'odom'` in sim.yaml
  
- **Jetson (AMCL)**: ROS 2 Humble, Ubuntu 22.04
  - Export: `export ROS_DOMAIN_ID=42` (both machines must match)
  - Run: `ros2 launch f1tenth_localization amcl.launch.py`
  - Monitor: `ros2 run f1tenth_localization performance_monitor.py`

#### Alternative: Run Everything on Jetson
To eliminate network overhead/DDS issues, run simulation on Jetson:
```bash
# On Jetson (slower but eliminates cross-machine issues):
ros2 launch f1tenth_gym_ros gym_bridge_launch.py &
ros2 launch f1tenth_localization amcl.launch.py min_particles:=5000 &
ros2 run f1tenth_localization performance_monitor.py
```

### Goal Reminder:
- **Objective**: Stress test CPU AMCL to show it hits CPU limits
- **Then**: Compare with GPU-accelerated localization
- **Metrics needed**: AMCL CPU % at various particle counts, pose rate, latency

### Files Changed (Summary):
| File | Changes |
|------|---------|
| `f1tenth_sim/f1tenth_gym_ros/gym_bridge.py` | Added `tf_frame_id`, `odom_frame_id` parameters |
| `f1tenth_sim/config/sim.yaml` | Set frame IDs to 'odom' |
| `f1tenth_localization/launch/amcl.launch.py` | Simplified, added `update_min_d/a` params |
| `f1tenth_localization/scripts/performance_monitor.py` | AMCL process tracking, peak detection, 50 Hz sampling |
