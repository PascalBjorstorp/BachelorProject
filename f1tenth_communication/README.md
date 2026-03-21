# F1/10th Communication

Communication packages for Jetson ↔ Ultra96 data transfer.

## Architecture

```
┌──────────────────────────────────┐     ┌─────────────────────────────────┐
│           JETSON                 │     │           ULTRA96               │
│                                  │     │                                 │
│  Localization → state_publisher  │────→│  mpc_receiver / pp_receiver     │
│  (/odom)         (KD-tree)       │     │  (/mpc_state subscriber)        │
│                  ↓               │     │  ↓                              │
│              /mpc_state          │     │  Load streamed horizon to FPGA  │
│              (MpcState.msg)      │     │  ↓                              │
│              [Q16.16 fixed-pt]   │     │  MPC → /drive                   │
└──────────────────────────────────┘     └─────────────────────────────────┘
```

## Packages

### f1tenth_msgs

Custom ROS2 message definitions:

- **MpcState.msg** (Q16.16 Fixed-Point):
  - `x_fp, y_fp, theta_fp, velocity_fp`: Vehicle state in Q16.16
  - `waypoint_index`: Nearest waypoint index
    - `horizon_length` + `ref_*_fp[20]`: Streaming MPC reference horizon
  - `timestamp_ms`: For latency measurement

### state_publisher (Jetson)

ROS2 node that runs on **Jetson**:
1. Loads trajectory CSV file
2. Subscribes to `/ego_racecar/odom`
3. Performs KD-tree lookup
4. Publishes `MpcState` with Q16.16 fixed-point values
5. Streams next N reference waypoints in each message
6. Uses Best Effort QoS for low latency

### state_receiver (Ultra96)

ROS2 node that runs on **Ultra96**:
1. Subscribes to `/mpc_state` (Best Effort QoS)
2. Loads only the streamed horizon waypoints into FPGA BRAM each cycle
3. Writes current vehicle state registers and runs FPGA MPC
4. Publishes `/drive` commands

## Setup

### 1. Build all packages

```bash
cd ~/BachelorProject
colcon build --packages-select f1tenth_msgs state_publisher state_receiver
source install/setup.bash
```

### 2. Sync trajectory file

Ensure same trajectory CSV on both systems:
```bash
scp ~/BachelorProject/f1tenth_planning/trajectories/Spielberg_raceline.csv \
    xilinx@192.168.50.182:~/trajectories/
```

### 3. Run on Jetson

```bash
ros2 launch state_publisher state_publisher_launch.py
```

### 4. Run on Ultra96

```bash
ros2 launch state_receiver mpc_launch.py
```

## Q16.16 Fixed-Point Format

Conversion:
```cpp
// Float to Q16.16
int32_t x_fp = static_cast<int32_t>(x * 65536.0);

// Q16.16 to Float
float x = static_cast<float>(x_fp) / 65536.0f;

// Direct to FPGA (no conversion)
int32_t fpga_x = msg->x_fp;  // Already Q16.16!
```

Range: ±32767.99998 with precision ~0.000015

## QoS Configuration

Both nodes use **Best Effort QoS** for lowest latency:
```cpp
auto qos = rclcpp::QoS(1)
    .best_effort()
    .durability_volatile();
```

## Latency Measurement

The `timestamp_ms` field enables end-to-end latency measurement:
```cpp
auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
double latency_ms = now_ms - msg->timestamp_ms;
```

Expected latencies:
- KD-tree lookup: ~1-5 µs
- Network + serialization: ~0.5-2 ms
- Total: < 3 ms typical
