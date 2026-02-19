# F1TENTH Stack

Driver stack for the F1TENTH autonomous racing platform.

## Overview

This package launches and configures the real car hardware:
- VESC motor controller (drive commands, odometry, IMU)
- DS4 joystick teleop
- Hokuyo LiDAR
- Ackermann command multiplexer
- Static transforms (base_link → laser, base_link → imu)

## Quick Start

```bash
# Build
cd ~/f1tenth_ws && colcon build && source install/setup.bash

# Full stack (default): VESC + joystick + LiDAR + mux + TFs
ros2 launch f1tenth_stack bringup_launch.py

# Teleop only (no LiDAR):
ros2 launch f1tenth_stack bringup_launch.py use_lidar:=false

# VESC only (testing motor/odom, no joystick, no LiDAR):
ros2 launch f1tenth_stack bringup_launch.py use_teleop:=false use_lidar:=false
```

### Calibrating Your Car

Run the parameter updater script in the workspace root to set wheelbase,
LiDAR offset, IMU orientation, and all other vehicle dimensions consistently
across every config file:

```bash
python3 update_vehicle_params.py
```

## DS4 Controller Mapping

| Button | Function |
|--------|----------|
| **L1** | Dead man's switch — manual driving |
| **R1** | Dead man's switch — enable autonomous |
| **Circle** | Emergency stop |
| **L1 + Square** | Slow mode (1 m/s max) |
| **Left Stick Y** | Speed (when L1 held) |
| **Right Stick X** | Steering (when L1 held) |

## Configuration Files

| File | Purpose |
|------|---------|
| `config/vesc.yaml` | VESC calibration (ERPM gain, servo gain, wheelbase, IMU, odometry) |
| `config/joy_teleop.yaml` | DS4 button mapping and speed limits |
| `config/sensors.yaml` | Hokuyo LiDAR connection and scan parameters |
| `config/mux.yaml` | Ackermann mux topic priorities |

### Key VESC Parameters (`config/vesc.yaml`)

```yaml
speed_to_erpm_gain: 4450.0            # ERPM per m/s
steering_angle_to_servo_gain: -0.915   # Servo units per radian
steering_angle_to_servo_offset: 0.468  # Servo center position
wheelbase: 0.324                       # Rear-to-front axle distance (m)
```

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/drive` | `AckermannDriveStamped` | In | Autonomous commands (mux priority 10) |
| `/teleop` | `AckermannDriveStamped` | In | Joystick commands (mux priority 100) |
| `/odom` | `Odometry` | Out | Wheel odometry from VESC |
| `/sensors/core` | `VescStateStamped` | Out | VESC telemetry |
| `/sensors/imu/raw` | `Imu` | Out | IMU data from VESC |
| `/scan` | `LaserScan` | Out | LiDAR scans |

## Writing Your AI

Publish `AckermannDriveStamped` to `/drive`:

```python
msg = AckermannDriveStamped()
msg.drive.speed = 2.0           # m/s
msg.drive.steering_angle = 0.1  # radians
self.drive_pub.publish(msg)
```

Hold **R1** to allow autonomous commands through the mux.
Hold **L1** to override with manual control. Press **Circle** to stop.

## Optional: Throttle Interpolator

A `throttle_interpolator` node is included but **not launched by default**.
It smooths motor/servo commands to limit acceleration jerk. To use it:

```bash
ros2 run f1tenth_stack throttle_interpolator --ros-args --params-file config/vesc.yaml
```

## Troubleshooting

| Problem | Fix |
|---------|-----|
| Controller not detected | `ls /dev/input/js*` / `jstest /dev/input/js0` |
| VESC not connecting | `ls /dev/ttyACM*` — check USB cable |
| No LiDAR data | Run `scripts/test_lidar.sh` to diagnose |
