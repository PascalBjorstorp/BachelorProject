# F1TENTH System - Enhanced

Complete driver stack for F1TENTH autonomous racing platform with enhanced VESC support.

## Cloning
If you clone this repository, make sure to clone submodules (if any):

```bash
git clone --recursive <repo-url>
# or if already cloned:
git submodule update --init --recursive
```

## Features

This is an **enhanced version** of the F1TENTH VESC stack with:

### VESC Driver Enhancements
- ✅ **200Hz IMU update rate**
- ✅ **Configurable covariances** for sensor fusion (EKF)
- ✅ **Proper IMU frame_id** support

### Odometry Enhancements  
- ✅ **IMU-based yaw estimation** with low-pass filtering
- ✅ **Multiple integration methods** (euler, trapezoidal, analytical)
- ✅ **Configurable odometry covariances**

### Control Enhancements
- ✅ **Braking system** with sigmoid deceleration curve
- ✅ **Multiple operation modes** (ERPM, current-based)
- ✅ **Odometry feedback** for velocity control

## Structure

```
f1tenth_system/
├── ackermann_mux/      # Priority-based command multiplexer
├── f1tenth_stack/      # Launch files and configurations
│   ├── config/
│   │   ├── vesc.yaml           # VESC & odometry config
│   │   ├── mux.yaml            # Mux priorities
│   │   ├── joy_teleop.yaml     # DS4 controller mapping
│   │   └── sensors.yaml        # LiDAR config
│   └── launch/
│       ├── bringup_launch.py   # Full stack
│       ├── teleop_launch.py    # Teleop + VESC
│       └── vesc_only_launch.py # VESC testing
├── vesc/               # Enhanced VESC driver packages
│   ├── vesc/               # Meta-package
│   ├── vesc_driver/        # Serial communication, IMU
│   ├── vesc_ackermann/     # Ackermann conversion, odometry
│   └── vesc_msgs/          # Message definitions
└── scripts/            # Docker scripts
```

## Quick Start

### Prerequisites

**Option 1: Use the dependency installer script (recommended)**
```bash
# Supports both ROS2 Jazzy and Humble
cd f1tenth_system
./install_dependencies.sh
```

**Option 2: Manual installation**
```bash
# Install ROS2 dependencies (replace $ROS_DISTRO with jazzy or humble)
sudo apt install ros-$ROS_DISTRO-joy ros-$ROS_DISTRO-joy-teleop ros-$ROS_DISTRO-urg-node
```

### Build
```bash
# Create workspace
mkdir -p ~/f1tenth_ws/src
cd ~/f1tenth_ws/src

# Clone this repository
git clone <this-repo-url> f1tenth_system

# Install dependencies
cd ~/f1tenth_ws
rosdep update && rosdep install --from-paths src -i -y

# Build
colcon build
source install/setup.bash
```

### Run
```bash
# Full stack (joystick + VESC + LiDAR)
ros2 launch f1tenth_stack bringup_launch.py

# Teleop only (no LiDAR)
ros2 launch f1tenth_stack teleop_launch.py

# VESC only (for testing)
ros2 launch f1tenth_stack vesc_only_launch.py
```

## Controller Mapping (DualShock 4)

| Button | Function |
|--------|----------|
| **L1** | Hold for manual driving |
| **R1** | Hold to enable autonomous |
| **Circle** | Emergency stop |
| **L1 + Square** | Slow mode (1 m/s) |
| **Left Stick Y** | Speed |
| **Right Stick X** | Steering |

## Topics

### Input Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/drive` | `AckermannDriveStamped` | Autonomous commands (priority 10) |
| `/teleop` | `AckermannDriveStamped` | Joystick commands (priority 100) |

### Output Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | `Odometry` | Wheel odometry |
| `/sensors/core` | `VescStateStamped` | VESC telemetry |
| `/sensors/imu/raw` | `Imu` | IMU data |
| `/scan` | `LaserScan` | LiDAR data |
| `/ackermann_drive` | `AckermannDriveStamped` | Mux output to VESC |

## Configuration

Edit `f1tenth_stack/config/vesc.yaml` to calibrate for your car:

```yaml
speed_to_erpm_gain: 4450.0      # ERPM per m/s
steering_angle_to_servo_gain: -0.915
steering_angle_to_servo_offset: 0.468
wheelbase: 0.324                # meters
```

## Docker (Optional)

For containerized deployment on Jetson:

```bash
# Start a new container
./scripts/run_container.sh

# Resume an existing container
./scripts/resume_container.sh f1tenth_container
```

## Documentation

See [`f1tenth_stack/README.md`](f1tenth_stack/README.md) for detailed documentation.

## License

MIT License
