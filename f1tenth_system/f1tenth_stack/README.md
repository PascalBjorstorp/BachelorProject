# F1TENTH Stack

Complete driver stack for F1TENTH autonomous racing platform with enhanced VESC support.

## Overview

This package provides launch files and configurations to bring up the full F1TENTH car including:
- DualShock 4 (DS4) controller teleop
- VESC motor controller with IMU integration
- Hokuyo LiDAR
- Ackermann command multiplexing
- Static transforms

## Features

### Enhanced VESC Support
- ✅ Proper IMU unit conversions (g → m/s², deg/s → rad/s)
- ✅ 200Hz IMU update rate
- ✅ Configurable covariances for sensor fusion
- ✅ Multiple odometry integration methods (euler, trapezoidal, analytical)
- ✅ IMU-based yaw estimation with low-pass filtering
- ✅ Braking system with sigmoid deceleration curve

### DS4 Controller Mapping
| Button | Function |
|--------|----------|
| **L1** | Dead man's switch - Manual driving |
| **R1** | Dead man's switch - Enable autonomous |
| **Circle** | Emergency stop |
| **L1 + Square** | Slow mode (1 m/s max) |
| **Left Stick Y** | Speed (when L1 held) |
| **Right Stick X** | Steering (when L1 held) |

## Quick Start

### 1. Build the workspace
```bash
cd ~/f1tenth_ws
colcon build
source install/setup.bash
```

### 2. Launch everything
```bash
# Full stack (joystick + VESC + LiDAR + mux)
ros2 launch f1tenth_stack bringup_launch.py

# Teleop only (no LiDAR)
ros2 launch f1tenth_stack teleop_launch.py

# VESC only (for testing)
ros2 launch f1tenth_stack vesc_only_launch.py
```

### 3. Drive the car
1. Power on the car
2. Connect DS4 controller via Bluetooth
3. Hold **L1** and use joysticks to drive

## Configuration

### VESC Calibration (`config/vesc.yaml`)
You **must** calibrate these values for your specific car:

```yaml
speed_to_erpm_gain: 4450.0      # ERPM per m/s
steering_angle_to_servo_gain: -0.915  # Servo units per radian
steering_angle_to_servo_offset: 0.468 # Servo center position
wheelbase: 0.324                # Car wheelbase in meters
```

**How to calibrate:**
1. `speed_to_erpm_gain`: Drive at known speed, record ERPM from `/sensors/core`
2. `steering_*`: Measure steering angle at servo min/max positions
3. `wheelbase`: Measure from front axle to rear axle

### Joystick (`config/joy_teleop.yaml`)
Adjust speed limits and button mappings for your preference.

### LiDAR (`config/sensors.yaml`)
Configure serial port and scan parameters.

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

## Writing Your AI

Your autonomous driving node should:

1. **Subscribe to:**
   - `/scan` - LiDAR data
   - `/odom` - Wheel odometry

2. **Publish to:**
   - `/drive` - `AckermannDriveStamped` commands

3. **Example:**
```python
from ackermann_msgs.msg import AckermannDriveStamped

msg = AckermannDriveStamped()
msg.header.stamp = self.get_clock().now().to_msg()
msg.drive.speed = 2.0  # m/s
msg.drive.steering_angle = 0.1  # radians
self.drive_pub.publish(msg)
```

4. **Control flow:**
   - Hold **R1** on controller to enable autonomous
   - Hold **L1** to take manual control (overrides AI)
   - Press **Circle** for emergency stop

## Troubleshooting

### Controller not detected
```bash
# Check if controller is connected
ls /dev/input/js*

# Test controller
jstest /dev/input/js0
```

### VESC not connecting
```bash
# Check USB connection
ls /dev/ttyACM*

# Create udev rule for persistent naming
# See vesc_driver/scripts/99-vesc6.rules
```

### No LiDAR data
```bash
# Check serial port
ls /dev/ttyUSB* /dev/ttyACM*

# Test connection
ros2 run urg_node urg_node_driver --ros-args -p serial_port:=/dev/ttyACM0
```

## License

MIT License
