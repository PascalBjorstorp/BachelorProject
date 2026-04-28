# F1TENTH Localization Test Guide

This guide is for the OptiTrack localization tests on Tuesday, April 28, 2026.

## Commands

Build and source from the workspace root:

```bash
colcon build --packages-select f1tenth_localization f1tenth_stack
source install/setup.bash
```

Record one localization bag. Use a unique output path for each run:

```bash
./f1tenth_system/f1tenth_stack/scripts/record_localization_bag.sh \
  /home/pascal/Documents/BachelorProject/bags/Localization_gpu_p0400_kldfalse_b270_run01
```

GPU AMCL fixed-particle launch. Set `P` to the particle count and `BEAMS` to
the AMCL beam count:

```bash
ros2 launch f1tenth_stack System_launch.py \
  lidar_cluster:=4 \
  amcl_num_particles:=400 \
  amcl_min_particles:=400 \
  amcl_max_particles:=400 \
  amcl_max_beams:=270 \
  amcl_use_kld:=false
```

GPU AMCL KLD/adaptive launch. Use this after the fixed-particle sweep:

```bash
ros2 launch f1tenth_stack System_launch.py \
  lidar_cluster:=4 \
  amcl_num_particles:=400 \
  amcl_min_particles:=100 \
  amcl_max_particles:=1500 \
  amcl_max_beams:=270 \
  amcl_use_kld:=true
```

GPU AMCL full-resolution LiDAR launch for beam tests:

```bash
ros2 launch f1tenth_stack System_launch.py \
  lidar_cluster:=1 \
  amcl_num_particles:=800 \
  amcl_min_particles:=800 \
  amcl_max_particles:=800 \
  amcl_max_beams:=540 \
  amcl_use_kld:=false
```

Nav2 AMCL fixed-particle launch. Terminal 1 starts the base stack without your
GPU AMCL/EKF:

```bash
ros2 launch f1tenth_stack System_no_localization.launch.py lidar_cluster:=4
```

Terminal 2 starts Nav2 AMCL with the same particle and beam values:

```bash
ros2 launch f1tenth_localization nav2_amcl.launch.py \
  min_particles:=400 \
  max_particles:=400 \
  max_beams:=270
```

Nav2 AMCL full-resolution LiDAR launch for beam tests. Terminal 1:

```bash
ros2 launch f1tenth_stack System_no_localization.launch.py lidar_cluster:=1
```

Terminal 2:

```bash
ros2 launch f1tenth_localization nav2_amcl.launch.py \
  min_particles:=800 \
  max_particles:=800 \
  max_beams:=540
```

Pure odom launch. Terminal 1:

```bash
ros2 launch f1tenth_stack System_no_localization.launch.py
```

Terminal 2 publishes `/odom_pose` and `/ekf_pose` without AMCL corrections:

```bash
ros2 launch f1tenth_localization odom_only.launch.py
```

## Particle Sweep

Run the same fixed-particle sweep for both your GPU AMCL and Nav2 AMCL:

```text
100, 150, 250, 400, 600, 800, 1000, 1500
```

Run each setting 5 times. Keep LiDAR in normal 270-beam mode:

```text
lidar_cluster:=4
amcl_max_beams/max_beams:=270
```

For GPU AMCL fixed-particle runs, set:

```text
amcl_num_particles:=P
amcl_min_particles:=P
amcl_max_particles:=P
amcl_use_kld:=false
```

For Nav2 AMCL fixed-particle runs, set:

```text
min_particles:=P
max_particles:=P
max_beams:=270
```

Suggested naming:

```text
Localization_gpu_p0100_kldfalse_b270_run01
Localization_nav2_p0100_b270_run01
```

Use `record_localization_bag.sh` for every run. It records redundant topics so
the same recorder works for GPU AMCL, Nav2 AMCL, and pure odom.

## Beam Sweep

Do this after the particle sweep. Pick 1-3 useful particle counts, for example
the lower bound, the knee point, and the high-quality setting.

Use full-resolution LiDAR so every AMCL beam setting samples from the same
1080-beam scan:

```text
lidar_cluster:=1
```

Beam settings:

```text
90, 135, 270, 540, 1080
```

For GPU AMCL:

```bash
ros2 launch f1tenth_stack System_launch.py \
  lidar_cluster:=1 \
  amcl_num_particles:=800 \
  amcl_min_particles:=800 \
  amcl_max_particles:=800 \
  amcl_max_beams:=1080 \
  amcl_use_kld:=false
```

For Nav2 AMCL, run the base stack with `lidar_cluster:=1`, then:

```bash
ros2 launch f1tenth_localization nav2_amcl.launch.py \
  min_particles:=800 \
  max_particles:=800 \
  max_beams:=1080
```

## Pure Odom

Terminal 1, base stack without localization:

```bash
ros2 launch f1tenth_stack System_no_localization.launch.py
```

Terminal 2, odom relay plus EKF prediction only:

```bash
ros2 launch f1tenth_localization odom_only.launch.py
```

This publishes `/odom_pose` and `/ekf_pose`, but no AMCL corrections. Treat it
as pure odom in the analysis: align the first odom/EKF pose to OptiTrack and
measure drift over distance, lap, and yaw.

## Parameters

Do not mix these into the first particle sweep. Change one family at a time.

KLD parameters only matter when `amcl_use_kld:=true`:

```text
kld_epsilon: 0.02, 0.05, 0.10
kld_z:       1.96, 2.33
```

Start KLD testing with:

```text
amcl_min_particles:=100
amcl_max_particles:=1500
amcl_use_kld:=true
```

`z_hit`, `z_rand`, and `sigma_hit` are sensor-model parameters. Keep
`z_hit + z_rand` close to 1.0 when `z_short` and `z_max` are zero. If scan
matching looks noisy after particle/beam sweeps, test:

```text
sigma_hit: 0.03, 0.05, 0.08, 0.12
z_hit/z_rand: 0.95/0.05, 0.975/0.025, 0.99/0.01
```

## Metrics

Report at least:

- XY RMSE, median, p95, p99, max
- yaw RMSE and yaw p95
- per-axis bias: mean error X/Y
- precision after bias removal: standard deviation of X/Y/yaw error
- convergence time after start or initial pose
- failure/jump count
- AMCL processing time: mean, p95, p99, max
- scan-to-AMCL and scan-to-EKF latency
- CPU/GPU usage and per-node CPU

Useful plots:

- OptiTrack trajectory vs estimate trajectory
- X error vs Y error scatter
- XY error over time
- yaw error over time
- AMCL timing histogram
- AMCL timing vs sample index, colored by `/amcl_particle_count`

The GPU AMCL node publishes:

```text
/amcl_particle_count  std_msgs/msg/Int32
```

For the timing/particle heat plot, pair `/amcl_timing` with
`/amcl_particle_count` by bag receive order or receive time.

## Extra Tests

Stationary: leave the car still for 60-120 seconds. Measure pose jitter and yaw
drift.

Odom model: run straight lines, constant-radius turns, and figure-eights.
Compare `/ego_racecar/odom` and `/ekf_pose` against OptiTrack.

Initial pose robustness: local AMCL is fine for racing if start pose is known.
It is not a kidnapping/global recovery system. Test offsets only if time
allows: 0.2 m / 10 deg and 0.5 m / 30 deg.

Before trusting any result, make the OptiTrack `world` to `map` transform as
accurate as possible. A constant map/world bias can hide real AMCL differences.
