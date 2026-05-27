# mpc_riccati — CPU MPC for the F1TENTH Stack

Riccati-ADMM Model Predictive Controller in C. Provides the simulation and hardware ROS2 nodes for the path-tracking MPC compared against the FPGA implementation in [../FPGA_Implementations/MPC_FPGA_Kria/](../FPGA_Implementations/MPC_FPGA_Kria/).

ROS2 package name: **`mpc_riccati`**.

## Package Layout

```
MPC/
├── include/                       # Public headers (mpc_types.h, mpc.h, …)
├── src/
│   ├── mpc.c                      # MPC orchestration: env config, QP setup, status reporting
│   ├── riccati_solver.c           # Riccati backward/forward + ADMM iterations
│   ├── vehicle_model.c            # Pacejka-based vehicle linearization (Frenet)
│   ├── util_math.c                # Math helpers shared by solver and node
│   └── mpc_hardware_node.c        # Hardware ROS2 node (executable: mpc_hardware_node)
├── sim/
│   └── mpc_ros2_node.c            # Simulator ROS2 node (executable: mpc_node)
├── launch/
│   ├── mpc_launch.py              # Simulation launch
│   └── mpc_hardware.launch.py     # Hardware launch
├── test/
│   ├── test_sim_drive.c           # Closed-loop test harness (no ROS dependency)
│   └── tune_realistic_v2.py       # Weight / horizon tuner
└── trajectories/                  # Sample raceline CSVs
```

## Build

From the workspace root:

```bash
colcon build --packages-select mpc_riccati
source install/setup.bash
```

## Run

Simulation (with `f1tenth_gym_ros` bridge already running):

```bash
ros2 launch mpc_riccati mpc_launch.py \
    trajectory_file:=/path/to/raceline.csv
```

Hardware:

```bash
ros2 launch mpc_riccati mpc_hardware.launch.py \
    trajectory_file:=/path/to/raceline.csv
```

Hardware launch arguments (declared in `launch/mpc_hardware.launch.py`):

| Argument                          | Purpose |
|-----------------------------------|---------|
| `trajectory_file`                 | Raceline CSV (positional) |
| `use_local_raceline`              | Subscribe to a Path topic instead of the static CSV |
| `local_raceline_topic`            | Topic name when `use_local_raceline = true` |
| `odom_topic`, `pose_topic`        | Vehicle state inputs |
| `drive_topic`, `servo_topic`      | Outputs |
| `verbose`, `watchdog_timeout`     | Debug / safety knobs |
| `low_speed_brake_inhibit_vx`, `low_speed_min_accel` | Low-speed throttle handling |
| `recovery_epsi`, `recovery_ey`, `recovery_vref_cap` | Recovery thresholds |
| `drive_republish_period`          | Auto-republish for keep-alive |
| `log_local_raceline_snapshots`    | Periodic raceline dumps for offline analysis |
| `solver_csv_log`                  | Per-tick solver log (timestamps, status, iterations) |

## Runtime Configuration (environment variables)

Tuning values are read from environment variables in `mpc.c` and `riccati_solver.c` at startup. Defaults come from `include/mpc_types.h`.

| Group           | Variables (verified in `src/mpc.c`) |
|-----------------|-------------------------------------|
| Stage weights   | `MPC_W_LAT_ERROR`, `MPC_W_HEADING`, `MPC_W_VELOCITY`, `MPC_W_LAT_VEL`, `MPC_W_YAW_RATE` |
| Effort weights  | `MPC_W_STEER_EFFORT`, `MPC_W_ACCEL_EFFORT`, `MPC_W_DELTA_ACTUAL` |
| Rate weights    | `MPC_W_STEER_RATE`, `MPC_W_ACCEL_RATE` |
| Solver          | `RHO`, `RHO_U`, `TOL`, `MAX_ITER`, `MPC_SHARED_RHO`, `MPC_ADAPTIVE_RHO` |
| Timing          | `PRED_DT`, `MPC_CROSS_CALL_SCALE` |
| Wall handling   | `MPC_WALL_MARGIN` (or `WALL_MARGIN`), `MPC_WALL_BIAS_CLEAR_M`, `MPC_WALL_BIAS_MAX_M`, `MPC_WALL_REF_CLEAR_M`, `MPC_WALL_BOUND_WINDOW` |
| Misc            | `MPC_AFFINE_SCALE` |

Compile-time bounds (from `include/mpc_types.h`):

- `PREDICTION_HORIZON = 20` — fixed at compile time; raise the define to lengthen the horizon.
- `PREDICTION_DT_SECONDS = 0.03f` — nominal step size used for model rollout (matches the FPGA kernel).
- `CONTROL_RATE_HZ = 200.0f` — control sample rate; `CONTROL_DT_SECONDS = 1 / CONTROL_RATE_HZ`.

## Tests and Tuning

Standalone closed-loop test (no ROS dependency):

```bash
gcc -D_GNU_SOURCE -O3 -std=c99 -Wall -ffast-math \
    -Wno-unused-variable -Wno-unused-but-set-variable \
    -Iinclude test/test_sim_drive.c \
        src/mpc.c src/riccati_solver.c \
        src/vehicle_model.c src/util_math.c \
    -o test_sim_drive -lm
./test_sim_drive
```

Weight tuner (uses the standalone binary):

```bash
python3 test/tune_realistic_v2.py --objective tracker
python3 test/tune_realistic_v2.py --objective fastest --jobs 8
```

Tuning output is written next to `tune_realistic_v2.py` as timestamped CSVs (e.g. `tuning_hardware_base_<timestamp>.csv`).
