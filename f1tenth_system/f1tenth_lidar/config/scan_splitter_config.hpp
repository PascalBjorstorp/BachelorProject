// Copyright (c) 2025 Pascal — MIT License
#pragma once

// Compile-time scan splitter configuration.
// Edit these defines and rebuild f1tenth_lidar to apply changes.

#define SCAN_SPLITTER_ENABLE_SPLITTING true
// Fallback endpoint-distance splitter. Used only when raycast splitting is disabled.
#define SCAN_SPLITTER_OBSTACLE_THRESHOLD_M 0.18
#define SCAN_SPLITTER_MIN_CLUSTER_SIZE 4
#define SCAN_SPLITTER_MAX_CLUSTER_GAP_BEAMS 1
#define SCAN_SPLITTER_MAX_OBSTACLE_WIDTH_M 0.65

// Endpoints this close to mapped walls are treated as wall noise, not obstacle.
#define SCAN_SPLITTER_WALL_TOLERANCE_M 0.18

// Raycast obstacle extraction. A beam is obstacle only when it is clearly
// before the expected mapped wall and outside the wall tolerance band.
#define SCAN_SPLITTER_ENABLE_RAYCAST_SPLITTING true
#define SCAN_SPLITTER_OCCLUSION_THRESHOLD_M 0.25
#define SCAN_SPLITTER_RAYCAST_WALL_HIT_TOLERANCE_M 0.15
#define SCAN_SPLITTER_RAYCAST_STEP_M 0.0
#define SCAN_SPLITTER_RAYCAST_MAX_RANGE_M 8.0

// Obstacle output field of view for the lateral planner. The raw /scan remains
// unchanged; only /scan_obstacles is masked outside this sector.
#define SCAN_SPLITTER_LIMIT_OBSTACLE_FOV true
#define SCAN_SPLITTER_OBSTACLE_ANGLE_MIN_RAD -1.57079632679
#define SCAN_SPLITTER_OBSTACLE_ANGLE_MAX_RAD  1.57079632679

#define SCAN_SPLITTER_SCAN_TOPIC "/scan"
#define SCAN_SPLITTER_OBSTACLES_TOPIC "/scan_obstacles"

#define SCAN_SPLITTER_LASER_FRAME "ego_racecar/laser"
#define SCAN_SPLITTER_MAP_FRAME "map"
#define SCAN_SPLITTER_MAP_TOPIC "/map"
