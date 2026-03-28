# F1Tenth Control Package

A modular, high-performance C++ package for F1Tenth autonomous racing algorithms.

## Overview

This package provides reusable components and algorithms for F1Tenth autonomous racing. It's designed with modularity in mind, allowing components like LiDAR processing and math utilities to be shared across different algorithms.

## Package Structure

```
f1tenth_control/
├── include/f1tenth_control/
│   ├── common/              # Reusable components
│   │   ├── types.hpp        # Common data types (Point2D, Pose2D, TrajectoryPoint, Gap, etc.)
│   │   ├── math_utils.hpp   # Math utilities (angle normalization, transforms, filters)
│   │   └── lidar_processor.hpp  # LiDAR preprocessing (filtering, validation)
│   ├── algorithms/          # Racing algorithms
│   │   ├── follow_the_gap.hpp   # Follow The Gap (reactive)
│   │   ├── pure_pursuit.hpp     # Pure Pursuit (path following)
│   │   └── stanley.hpp          # Stanley controller (path following)
│   └── nodes/               # ROS2 node wrappers
│       ├── ftg_node.hpp
│       ├── pure_pursuit_node.hpp
│       └── stanley_node.hpp
├── src/                     # Implementation files
├── config/                  # Configuration files
│   └── ftg_params.yaml      # FTG parameters
├── launch/                  # Launch files
│   ├── ftg_hardware_launch.py
│   ├── pure_pursuit_launch.py
│   └── stanley_launch.py
└── test/                    # Unit tests
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
# Hardware mode with default parameters
ros2 launch f1tenth_control ftg_hardware_launch.py

# With custom speed
ros2 launch f1tenth_control ftg_hardware_launch.py max_speed:=3.0

# With mapping mode enabled
ros2 launch f1tenth_control ftg_hardware_launch.py mapping_mode:=true
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
| `max_steering` | 0.4 | Max steering angle (rad) |
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

The `LidarProcessor` class provides reusable LiDAR preprocessing:

```cpp
#include "f1tenth_control/common/lidar_processor.hpp"

f1tenth_control::LidarProcessor processor(config);

// Process scan (range filtering, median filter, angle computation)
auto processed = processor.processScan(ranges, angle_min, angle_max, angle_increment);

// Find closest point
size_t closest = processor.findClosestPoint(processed);

// Convert to Cartesian
Point2D point = processor.scanPointToCartesian(processed, index);

// Extract boundary points for mapping
auto boundaries = processor.extractBoundaryPoints(processed, robot_pose, timestamp);
```

Algorithm-specific processing (gap finding, disparity extension, safety bubbles) is in `FollowTheGap`.

### Math Utilities

Common math functions for robotics:

```cpp
#include "f1tenth_control/common/math_utils.hpp"

// Angle normalization
double normalized = f1tenth_control::math::normalizeAngle(angle);

// Coordinate transforms
Point2D global = f1tenth_control::math::localToGlobal(local_point, robot_pose);

// Filtering
auto filtered = f1tenth_control::math::medianFilter(data, window_size);

// Basic math
double d = f1tenth_control::math::distance(point_a, point_b);
double v = f1tenth_control::math::clamp(value, min_val, max_val);
double i = f1tenth_control::math::lerp(a, b, t);
```

## Adding New Algorithms

1. Create header in `include/f1tenth_control/algorithms/`
2. Create implementation in `src/algorithms/`
3. Create ROS2 node wrapper in `src/nodes/`
4. Add to CMakeLists.txt
5. Create config file and launch file

The `LidarProcessor` and math utilities can be reused. For example:
- `LidarProcessor::scanPointToCartesian()` for obstacle visualization
- `math::localToGlobal()` for waypoint transforms
- `math::normalizeAngle()` for angle wrapping

## Hardware

Designed for F1Tenth cars with:
- **Computer:** NVIDIA Jetson Orin (8GB RAM)
- **LiDAR:** Hokuyo UST-10LX (270° FOV, 40Hz)
- **Motor Controller:** VESC with Ackermann steering

## License

MIT License
