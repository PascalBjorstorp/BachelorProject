// Copyright (c) 2025 Pascal — MIT License
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "scan_splitter_config.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace f1tenth_lidar
{

/**
 * @brief Extracts obstacle beams from raw LiDAR scans.
 *
 * Subscribes to raw /scan and the static occupancy-grid map, then for each
 * beam checks whether the endpoint lies near a known wall (using a
 * precomputed distance field).  Beams that hit something NOT in the map are
 * classified as obstacle beams (likely the opponent car).
 *
 * Publishes one LaserScan topic:
 *   /scan_obstacles — wall beams replaced with inf (for lateral planner)
 */
class ScanSplitterNode : public rclcpp::Node
{
public:
  explicit ScanSplitterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── Callbacks ──────────────────────────────────────────────────────
  void map_callback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg);
  void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan);

  // ── Helpers ────────────────────────────────────────────────────────
  /**
    * Compute map distance-to-wall (in metres) from a binary occupancy grid
    * using 8-connected Dijkstra propagation from occupied cells.
    *
    * Distances are metric (already in metres), suitable for thresholding.
   */
  void compute_distance_field(const nav_msgs::msg::OccupancyGrid & grid);

  float distance_to_wall(float wx, float wy) const;

  float raycast_wall_range(
    float laser_x,
    float laser_y,
    float world_angle,
    float range_min,
    float range_max) const;

  /**
   * Keep only obstacle runs with >= min_cluster_size beams.
   *
   * Small false gaps (<= max_gap beams) between two obstacle runs can be
   * bridged before size filtering to reduce sensitivity to sparse scans.
   */
  void filter_clusters(
    std::vector<bool> & mask,
    const std::vector<float> & ranges,
    const std::vector<float> & angles,
    int min_size,
    int max_gap,
    double max_width_m) const;

  // ── Parameters ─────────────────────────────────────────────────────
  bool   enable_splitting_{SCAN_SPLITTER_ENABLE_SPLITTING};
  double obstacle_threshold_{SCAN_SPLITTER_OBSTACLE_THRESHOLD_M};
  double wall_tolerance_{SCAN_SPLITTER_WALL_TOLERANCE_M};
  int    min_cluster_size_{SCAN_SPLITTER_MIN_CLUSTER_SIZE};
  int    max_cluster_gap_beams_{SCAN_SPLITTER_MAX_CLUSTER_GAP_BEAMS};
  double max_cluster_width_{SCAN_SPLITTER_MAX_OBSTACLE_WIDTH_M};
  bool   enable_raycast_splitting_{SCAN_SPLITTER_ENABLE_RAYCAST_SPLITTING};
  double occlusion_threshold_{SCAN_SPLITTER_OCCLUSION_THRESHOLD_M};
  double raycast_wall_hit_tolerance_{SCAN_SPLITTER_RAYCAST_WALL_HIT_TOLERANCE_M};
  double raycast_step_{SCAN_SPLITTER_RAYCAST_STEP_M};
  double raycast_max_range_{SCAN_SPLITTER_RAYCAST_MAX_RANGE_M};
  std::string scan_topic_{SCAN_SPLITTER_SCAN_TOPIC};
  std::string obstacles_topic_{SCAN_SPLITTER_OBSTACLES_TOPIC};
  std::string laser_frame_{SCAN_SPLITTER_LASER_FRAME};
  std::string map_frame_{SCAN_SPLITTER_MAP_FRAME};

  // ── Map state ──────────────────────────────────────────────────────
  bool   map_ready_{false};
  std::vector<float> distance_field_;   // metres to nearest wall per cell
  std::vector<uint8_t> occupied_map_;
  double map_origin_x_{0.0};
  double map_origin_y_{0.0};
  double map_resolution_{0.05};
  int    map_width_{0};
  int    map_height_{0};

  // ── Pre-allocated work buffers (avoid per-callback heap alloc) ────
  std::vector<float> angles_;
  std::vector<bool>  is_obstacle_;
  std::vector<float> obstacle_ranges_;

  // ── ROS interfaces ────────────────────────────────────────────────
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr  scan_sub_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr obstacles_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr splitter_timing_pub_;
};

}  // namespace f1tenth_lidar
