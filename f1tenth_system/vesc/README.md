# F1TENTH System - Enhanced VESC Driver Stack

Complete driver stack for F1TENTH autonomous racing platform with enhanced VESC motor controller support.

## ✨ Features

This is an **enhanced version** of the F1TENTH VESC stack with:

### VESC Driver Enhancements
- ✅ **Proper IMU unit conversions** (g → m/s², deg/s → rad/s)
- ✅ **200Hz IMU update rate** (was 50Hz)
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

## 📦 Packages

| Package | Description |
|---------|-------------|
| `vesc_driver` | Serial communication with VESC, IMU data |
| `vesc_ackermann` | Ackermann ↔ VESC command conversion, odometry |
| `vesc_msgs` | Custom message types |
| `vesc` | Meta-package |
| `ackermann_mux` | Priority-based command multiplexer |
| `f1tenth_stack` | Launch files and configurations |

## 🚀 Quick Start

### Prerequisites
```bash
# Install ROS2 dependencies
sudo apt install ros-humble-joy ros-humble-joy-teleop ros-humble-urg-node
```

### Build
```bash
# Create workspace
mkdir -p ~/f1tenth_ws/src
cd ~/f1tenth_ws/src

# Clone this repository
git clone <this-repo-url>

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

# Teleop only
ros2 launch f1tenth_stack teleop_launch.py

# VESC only (testing)
ros2 launch f1tenth_stack vesc_only_launch.py
```

## 🎮 Controller Mapping (DualShock 4)

| Button | Function |
|--------|----------|
| **L1** | Dead man's switch - Manual driving |
| **R1** | Dead man's switch - Enable autonomous |
| **Circle** | Emergency stop |
| **L1 + Square** | Slow mode (1 m/s max) |
| **Left Stick Y** | Speed (when L1 held) |
| **Right Stick X** | Steering (when L1 held) |

## 🔧 Configuration

Edit `f1tenth_stack/config/vesc.yaml` to calibrate for your car:

```yaml
speed_to_erpm_gain: 4450.0      # ERPM per m/s
steering_angle_to_servo_gain: -0.915
steering_angle_to_servo_offset: 0.468
wheelbase: 0.324                # meters
```

## 📖 Documentation

See [`f1tenth_stack/README.md`](f1tenth_stack/README.md) for detailed documentation.

## Original README

### How to test (VESC driver only)

1. Clone this repository and [transport drivers](https://github.com/ros-drivers/transport_drivers) into `src`.
2. `rosdep update && rosdep install --from-paths src -i -y`
3. Plug in the VESC with a USB cable.
4. Modify `vesc_driver/params/vesc_config.yaml` to reflect any changes.
5. Build the packages `colcon build`
6. `ros2 launch vesc_driver vesc_driver_node.launch.py`
7. If prompted "permission denied" on the serial port: `sudo chmod 777 /dev/ttyACM0`
