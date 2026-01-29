# F1Tenth Control Package

A modular, high-performance C++ package for F1Tenth autonomous racing algorithms.

## Overview

This package provides reusable components and algorithms for F1Tenth autonomous racing. It's designed with modularity in mind, allowing components like LiDAR processing and math utilities to be shared across different algorithms.

## Package Structure

```
f1tenth_control/
├── include/f1tenth_control/
│   ├── common/              # Reusable components
│   │   ├── types.hpp        # Common data types (Point2D, Pose2D, Gap, etc.)
│   │   ├── math_utils.hpp   # Math utilities (angle normalization, transforms, filters)
│   │   └── lidar_processor.hpp  # LiDAR processing (gap finding, disparity extension)
│   ├── algorithms/          # Racing algorithms
│   │   └── follow_the_gap.hpp   # Follow The Gap algorithm
│   └── nodes/               # ROS2 node wrappers
│       └── ftg_node.hpp     # FTG ROS2 node
├── src/                     # Implementation files
├── config/                  # Configuration files
│   └── ftg_params.yaml      # FTG parameters
└── launch/                  # Launch files
    └── ftg_launch.py        # FTG launch file
```

## Algorithms

### Follow The Gap (FTG)

A reactive obstacle avoidance algorithm that:
1. Processes LiDAR scan data with noise filtering
2. Finds the closest obstacle and creates a safety bubble
3. Applies disparity extension to avoid narrow gaps
4. Finds the largest/deepest gap in the scan
5. Steers toward the best gap while adjusting speed

**Features:**
- Configurable speed based on gap distance
- Emergency braking for close obstacles
- Preference for straight-ahead gaps (configurable)
- Mapping mode for track boundary extraction
- Real-time visualization support

## Usage

### Building

```bash
cd ~/f1tenth_ws
colcon build --packages-select f1tenth_control
source install/setup.bash
```

### Running FTG

```bash
# With default parameters
ros2 launch f1tenth_control ftg_launch.py

# With custom speed
ros2 launch f1tenth_control ftg_launch.py max_speed:=3.0

# With mapping mode enabled
ros2 launch f1tenth_control ftg_launch.py mapping_mode:=true
```

### Topics

**Subscribed:**
- `/scan` (sensor_msgs/LaserScan) - LiDAR scan data
- `/odom` (nav_msgs/Odometry) - Vehicle odometry
- `ftg/enable` (std_msgs/Bool) - Enable/disable the algorithm

**Published:**
- `/drive` (ackermann_msgs/AckermannDriveStamped) - Drive commands
- `ftg/visualization` (visualization_msgs/MarkerArray) - RViz visualization

### Parameters

All parameters are configurable via `config/ftg_params.yaml` or launch arguments. Key parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_speed` | 8.0 | Maximum speed (m/s) |
| `min_speed` | 1.0 | Minimum speed (m/s) |
| `max_steering_angle` | 0.4 | Max steering angle (rad) |
| `emergency_brake_distance` | 0.3 | Emergency stop distance (m) |
| `lidar.gap_threshold` | 3.0 | Minimum range for gap detection (m) |
| `mapping_mode` | false | Enable boundary point extraction |

### Dynamic Reconfiguration

Parameters can be changed at runtime:

```bash
ros2 param set /ftg_node max_speed 2.0
ros2 param set /ftg_node lidar.gap_threshold 2.5
```

## Reusable Components

### LidarProcessor

The `LidarProcessor` class provides reusable LiDAR processing:

```cpp
#include "f1tenth_control/common/lidar_processor.hpp"

f1tenth_control::LidarProcessor processor(config);

// Process scan
auto processed = processor.processScan(ranges, angle_min, angle_max, angle_increment);

// Find gaps
auto gaps = processor.findGaps(processed);

// Apply safety measures
processor.applyDisparityExtension(processed, car_width);
processor.applySafetyBubble(processed);
```

### Math Utilities

Common math functions for robotics:

```cpp
#include "f1tenth_control/common/math_utils.hpp"

// Angle normalization
double normalized = f1tenth_control::math::normalizeAngle(angle);

// Coordinate transforms
Point2D global = f1tenth_control::math::localToGlobal(local_point, robot_pose);

// Filtering
f1tenth_control::math::movingAverageFilter(data, window_size);
auto filtered = f1tenth_control::math::medianFilter(data, window_size);
```

## Adding New Algorithms

1. Create header in `include/f1tenth_control/algorithms/`
2. Create implementation in `src/algorithms/`
3. Create ROS2 node wrapper in `src/nodes/`
4. Add to CMakeLists.txt
5. Create config file and launch file

The `LidarProcessor` and math utilities can be reused. For example, Pure Pursuit could use:
- `LidarProcessor::scanToCartesian()` for obstacle visualization
- `math::localToGlobal()` for waypoint transforms
- `math::purePursuitSteering()` for steering calculation

## Hardware

Designed for F1Tenth cars with:
- **Computer:** NVIDIA Jetson Orin (8GB RAM)
- **LiDAR:** Hokuyo UST-10LX (270° FOV, 40Hz)
- **Motor Controller:** VESC with Ackermann steering

## License

MIT License
