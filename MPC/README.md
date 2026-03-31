# MPC Riccati-ADMM (ROS2 C)

This package contains the Riccati-ADMM MPC controller core and ROS2 nodes used
for both simulation and hardware execution in the F1TENTH stack.

## Package Overview

Package name:
- `mpc_riccati`

Core library:
- `src/mpc.c`
- `src/riccati_solver.c`
- `src/vehicle_model.c`
- `src/util_math.c`
- Public headers in `include/`

ROS2 nodes:
- Simulator node: `sim/mpc_ros2_node.c` (executable: `mpc_node`)
- Hardware node: `src/mpc_hardware_node.c` (executable: `mpc_hardware_node`)

Launch files:
- Simulation: `launch/mpc_launch.py`
- Hardware: `launch/mpc_hardware.launch.py`

## Build

From workspace root:

```bash
colcon build --packages-select mpc_riccati
source install/setup.bash
```

## Run

Simulation (gym bridge must already be running):

```bash
ros2 launch mpc_riccati mpc_launch.py trajectory_file:=/path/to/trajectory.csv
```

Hardware:

```bash
ros2 launch mpc_riccati mpc_hardware.launch.py trajectory_file:=/path/to/trajectory.csv
```

Useful hardware launch arguments:
- `odom_topic`
- `drive_topic`
- `servo_topic`
- `imu_topic`
- `pose_topic`
- `verbose`
- `watchdog_timeout`

## Runtime Configuration

The controller reads key tuning values from environment variables in the core
MPC layer (`mpc.c`) and solve path (`riccati_solver.c`). Typical variables:

- Weights: `Q_LAT`, `Q_HDG`, `Q_VEL`, `Q_LAT_VEL`, `Q_YAW`, `R_STEER`,
  `R_ACCEL`, `W_JERK`, `W_ACCEL_RATE`
- Solver: `RHO`, `RHO_U`, `ALPHA`, `TOL`, `MAX_ITER`
- Horizon/timestep: `HORIZON`, `PRED_DT`
- Wall handling: `WALL_END`, `WALL_STRIDE`, `WALL_MARGIN`

Note:
- The effective maximum horizon is bounded by compile-time array limits in
  `include/mpc_types.h` (`PREDICTION_HORIZON`).

## Tests and Tuning

Primary test/tuning files:
- `test/test_sim_drive.c`
- `test/tune_realistic_v2.py`

Standalone test build example:

```bash
gcc -D_GNU_SOURCE -O3 -std=c99 -Wall -ffast-math \
  -Wno-unused-variable -Wno-unused-but-set-variable \
  -Iinclude test/test_sim_drive.c src/mpc.c src/riccati_solver.c \
  src/vehicle_model.c src/util_math.c -o test_sim_drive -lm
```

Run tuner:

```bash
python3 test/tune_realistic_v2.py --objective tracker
python3 test/tune_realistic_v2.py --objective fastest --jobs 8
```

Horizon sweep note:
- Horizon is swept like any other parameter using:
  `[10, 14, 18, 20, 22, 26, 30, 34, 40, 50]`.
- Package default `PREDICTION_HORIZON` in `include/mpc_types.h` is set to 50,
  so the sweep values above are effective without extra flags.

Recent review updates in this package include:
- Null/invalid-input hardening in solver and utility layers.
- Shared trajectory waypoint type centralized in `include/mpc_types.h`.
- Sim and hardware ROS nodes aligned to direct define usage for timing/horizon
  paths where required.
- Legacy dead paths and ineffective sweep ranges removed from test/tuning flow.
