// Copyright (c) 2025 Pascal — MIT License
#pragma once

// Compile-time scan splitter configuration.
// Edit these defines and rebuild f1tenth_lidar to apply changes.

#define SCAN_SPLITTER_ENABLE_SPLITTING true
#define SCAN_SPLITTER_OBSTACLE_THRESHOLD_M 0.15
#define SCAN_SPLITTER_MIN_CLUSTER_SIZE 3
#define SCAN_SPLITTER_MAX_CLUSTER_GAP_BEAMS 1

#define SCAN_SPLITTER_SCAN_TOPIC "/scan"
#define SCAN_SPLITTER_WALLS_TOPIC "/scan_walls"
#define SCAN_SPLITTER_OBSTACLES_TOPIC "/scan_obstacles"

#define SCAN_SPLITTER_LASER_FRAME "ego_racecar/laser"
#define SCAN_SPLITTER_MAP_FRAME "map"
#define SCAN_SPLITTER_MAP_TOPIC "/map"
