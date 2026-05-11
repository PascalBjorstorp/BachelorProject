// Copyright (c) 2026 Pascal — MIT License
#pragma once

// Compile-time lateral planner configuration.
// Edit these defines and rebuild f1tenth_lateral_planner to apply changes.

// Trajectory source:
// 1) If TRAJECTORY_FILE is non-empty, that exact path is used.
// 2) Otherwise path resolves to <TRAJECTORY_PACKAGE share>/<TRAJECTORY_REL_PATH>.
#define TRAJECTORY_FILE ""
#define TRAJECTORY_PACKAGE "f1tenth_planning"
#define TRAJECTORY_REL_PATH "trajectories/my_track_raceline.csv"

// Car model
#define CAR_WIDTH_M 0.31
#define OPPONENT_LENGTH_M 0.58

// Clearance values
#define CLEARANCE_TOLERANCE_M 0.1
#define PLANNING_TOLERANCE_SCALE 2.0

// Avoidance window shape
#define MIN_WINDOW_M 8.0
#define WINDOW_TIME_S 2.0
#define WINDOW_LEAD_RATIO 0.5
#define MAX_LATERAL_SHIFT_M 0.8
#define MIN_AVOIDANCE_SHIFT_M 0.30
#define AVOIDANCE_EXTRA_CLEARANCE_M 0.15
#define MAX_AVOIDANCE_KAPPA 2.5
#define PASS_COMPLETE_MARGIN_M 2.0
#define OBSTACLE_MISSED_SCAN_HOLD 20

// Curvature-aware speed regulation for modified raceline
// Formula mirrors pure pursuit style:
//   scale = 1 / (1 + factor * max_preview_curvature)
// with a floor and a lateral-acceleration cap.
#define CURVATURE_SPEED_FACTOR 0.10
#define CURVATURE_SPEED_FLOOR_RATIO 0.43
#define CURVATURE_SPEED_PREVIEW_POINTS 8
#define MAX_LATERAL_ACCEL_MPS2 7.27
#define MIN_REGULATED_SPEED_MPS 0.30
#define OBSTACLE_DETECTED_MAX_SPEED_MPS 6.0

// Published path extent
#define LOOKAHEAD_POINTS 300
#define PATH_START_OFFSET_POINTS 0

// Node behavior
#define PUBLISH_RATE_HZ 200.0

// Obstacle behavior
// true  -> obstacle avoidance enabled
// false -> publish baseline raceline segment without lateral obstacle offsets
#define LATERAL_PLANNER_ENABLE_AVOIDANCE true
