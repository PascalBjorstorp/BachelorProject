// Copyright (c) 2025 Pascal — MIT License
#include "f1tenth_lidar/scan_splitter_node.hpp"

#include <cmath>
#include <limits>
#include <queue>
#include <algorithm>
#include <chrono>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/exceptions.h>

namespace f1tenth_lidar
{

// ────────────────────────────────────────────────────────────────────────────
//  Construction
// ────────────────────────────────────────────────────────────────────────────

ScanSplitterNode::ScanSplitterNode(const rclcpp::NodeOptions & options)
: Node("scan_splitter_node", options)
{
  // ── Declare parameters ────────────────────────────────────────────
  this->declare_parameter("enable_splitting", true);
  this->declare_parameter("obstacle_threshold_m", 0.05);
  this->declare_parameter("min_cluster_size", 1);
  this->declare_parameter("max_cluster_gap_beams", 0);
  this->declare_parameter("scan_topic", std::string("/scan"));
  this->declare_parameter("walls_topic", std::string("/scan_walls"));
  this->declare_parameter("obstacles_topic", std::string("/scan_obstacles"));
  this->declare_parameter("robot_frame", std::string("ego_racecar/base_link"));
  this->declare_parameter("laser_frame", std::string("ego_racecar/laser"));
  this->declare_parameter("map_frame", std::string("map"));

  // ── Read parameters ───────────────────────────────────────────────
  enable_splitting_   = this->get_parameter("enable_splitting").as_bool();
  obstacle_threshold_ = this->get_parameter("obstacle_threshold_m").as_double();
  min_cluster_size_   = this->get_parameter("min_cluster_size").as_int();
  max_cluster_gap_beams_ = this->get_parameter("max_cluster_gap_beams").as_int();
  scan_topic_         = this->get_parameter("scan_topic").as_string();
  walls_topic_        = this->get_parameter("walls_topic").as_string();
  obstacles_topic_    = this->get_parameter("obstacles_topic").as_string();
  robot_frame_        = this->get_parameter("robot_frame").as_string();
  laser_frame_        = this->get_parameter("laser_frame").as_string();
  map_frame_          = this->get_parameter("map_frame").as_string();

  // ── TF ────────────────────────────────────────────────────────────
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ── Subscribers ───────────────────────────────────────────────────
  // Map: RELIABLE + TRANSIENT_LOCAL (latched)
  rclcpp::QoS map_qos(1);
  map_qos.reliable().transient_local();
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", map_qos,
    std::bind(&ScanSplitterNode::map_callback, this, std::placeholders::_1));

  // Scan: BEST_EFFORT sensor QoS for lowest latency
  rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();
  sensor_qos.keep_last(5);
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, sensor_qos,
    std::bind(&ScanSplitterNode::scan_callback, this, std::placeholders::_1));

  // ── Publishers ────────────────────────────────────────────────────
  // RELIABLE so RViz and any RELIABLE subscriber can receive; latency
  // difference vs BEST_EFFORT is negligible on localhost / shared memory.
  rclcpp::QoS pub_qos(5);
  pub_qos.reliable();
  walls_pub_     = this->create_publisher<sensor_msgs::msg::LaserScan>(walls_topic_, pub_qos);
  obstacles_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(obstacles_topic_, pub_qos);

  // ── Timing publisher ─────────────────────────────────────────────
  splitter_timing_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "splitter_timing", rclcpp::QoS(10));

  RCLCPP_INFO(this->get_logger(), "Scan Splitter Node (C++) initialized");
  RCLCPP_INFO(this->get_logger(), "  Splitting enabled: %s", enable_splitting_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  Input:  %s", scan_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Walls:  %s", walls_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Obstacles: %s", obstacles_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Threshold: %.2f m", obstacle_threshold_);
  RCLCPP_INFO(this->get_logger(), "  Min cluster size: %d beams", min_cluster_size_);
  RCLCPP_INFO(this->get_logger(), "  Max cluster gap: %d beams", max_cluster_gap_beams_);
}

// ────────────────────────────────────────────────────────────────────────────
//  Map handling — precompute distance field with BFS
// ────────────────────────────────────────────────────────────────────────────

void ScanSplitterNode::compute_distance_field(
  const nav_msgs::msg::OccupancyGrid & grid)
{
  const int w = static_cast<int>(grid.info.width);
  const int h = static_cast<int>(grid.info.height);
  const float res = static_cast<float>(grid.info.resolution);
  const auto & data = grid.data;

  // squared-distance field in cell units (INT_MAX = unvisited)
  std::vector<int> sq_dist(w * h, std::numeric_limits<int>::max());
  std::queue<int> bfs;

  // Seed: every occupied cell (value >= 50) has distance 0
  for (int i = 0; i < w * h; ++i) {
    int8_t v = static_cast<int8_t>(data[i]);
    if (v >= 50 || v < 0) {        // occupied or unknown → treat as wall
      sq_dist[i] = 0;
      bfs.push(i);
    }
  }

  // 8-connected BFS (Chamfer-style); not perfectly Euclidean but very fast
  // and accurate enough for a threshold test.
  // Directions: 4 cardinal (cost² = 1) + 4 diagonal (cost² = 2)
  static constexpr int dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static constexpr int dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  static constexpr int cost2[8] = {1, 1, 1, 1, 2, 2, 2, 2};

  while (!bfs.empty()) {
    const int idx = bfs.front();
    bfs.pop();
    const int cx = idx % w;
    const int cy = idx / w;
    const int cur_d2 = sq_dist[idx];

    for (int d = 0; d < 8; ++d) {
      const int nx = cx + dx8[d];
      const int ny = cy + dy8[d];
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
      const int nidx = ny * w + nx;
      const int new_d2 = cur_d2 + cost2[d];
      if (new_d2 < sq_dist[nidx]) {
        sq_dist[nidx] = new_d2;
        bfs.push(nidx);
      }
    }
  }

  // Convert squared-cell-distance → metres
  distance_field_.resize(w * h);
  for (int i = 0; i < w * h; ++i) {
    distance_field_[i] = std::sqrt(static_cast<float>(sq_dist[i])) * res;
  }
}

void ScanSplitterNode::map_callback(
  const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg)
{
  map_resolution_ = msg->info.resolution;
  map_width_      = static_cast<int>(msg->info.width);
  map_height_     = static_cast<int>(msg->info.height);
  map_origin_x_   = msg->info.origin.position.x;
  map_origin_y_   = msg->info.origin.position.y;

  compute_distance_field(*msg);

  map_ready_ = true;
  RCLCPP_INFO(this->get_logger(),
    "Map received: %d×%d (res=%.3f m)", map_width_, map_height_, map_resolution_);
}

// ────────────────────────────────────────────────────────────────────────────
//  Scan processing — hot path, zero allocations after first call
// ────────────────────────────────────────────────────────────────────────────

void ScanSplitterNode::scan_callback(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan)
{
  const auto t_start = std::chrono::high_resolution_clock::now();
  const size_t n = scan->ranges.size();
  constexpr float INF = std::numeric_limits<float>::infinity();

  // ── Passthrough mode ──────────────────────────────────────────────
  if (!enable_splitting_) {
    walls_pub_->publish(*scan);

    auto empty = *scan;
    empty.ranges.assign(n, INF);
    obstacles_pub_->publish(empty);
    return;
  }

  if (!map_ready_) {
    walls_pub_->publish(*scan);
    return;
  }

  // ── Get laser pose in map frame ───────────────────────────────────
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(
      map_frame_, laser_frame_, tf2::TimePointZero,
      tf2::durationFromSec(0.02));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "TF lookup failed: %s — passing through raw scan", ex.what());
    walls_pub_->publish(*scan);
    return;
  }

  const float laser_x   = static_cast<float>(tf.transform.translation.x);
  const float laser_y   = static_cast<float>(tf.transform.translation.y);
  const auto & q        = tf.transform.rotation;
  const float siny_cosp = 2.0f * (static_cast<float>(q.w * q.z + q.x * q.y));
  const float cosy_cosp = 1.0f - 2.0f * (static_cast<float>(q.y * q.y + q.z * q.z));
  const float laser_yaw = std::atan2(siny_cosp, cosy_cosp);

  // ── Resize work buffers (no-op after first call with same size) ──
  if (angles_.size() != n) {
    angles_.resize(n);
    world_angles_.resize(n);
    endpoints_x_.resize(n);
    endpoints_y_.resize(n);
    is_obstacle_.resize(n);
    wall_ranges_.resize(n);
    obstacle_ranges_.resize(n);

    // Precompute beam angles (constant for a given scanner config)
    for (size_t i = 0; i < n; ++i) {
      angles_[i] = scan->angle_min + static_cast<float>(i) * scan->angle_increment;
    }
  }

  const float * __restrict__ ranges = scan->ranges.data();
  const float range_min = scan->range_min;
  const float range_max = scan->range_max;
  const float inv_res   = 1.0f / static_cast<float>(map_resolution_);
  const float ox        = static_cast<float>(map_origin_x_);
  const float oy        = static_cast<float>(map_origin_y_);
  const int w           = map_width_;
  const int h           = map_height_;
  const float threshold = static_cast<float>(obstacle_threshold_);
  const float * __restrict__ df = distance_field_.data();

  // ── Compute beam classification in a single tight loop ────────────
  for (size_t i = 0; i < n; ++i) {
    const float r = ranges[i];

    // Default: not obstacle
    is_obstacle_[i] = false;

    // Skip invalid beams
    if (!std::isfinite(r) || r <= range_min || r >= range_max) {
      continue;
    }

    const float world_angle = angles_[i] + laser_yaw;
    const float ex = laser_x + r * std::cos(world_angle);
    const float ey = laser_y + r * std::sin(world_angle);

    // Map pixel coordinates
    int px = static_cast<int>((ex - ox) * inv_res);
    int py = static_cast<int>((ey - oy) * inv_res);

    // Clamp
    px = std::clamp(px, 0, w - 1);
    py = std::clamp(py, 0, h - 1);

    // Distance-field lookup
    const float dist_to_wall = df[py * w + px];

    if (dist_to_wall > threshold) {
      is_obstacle_[i] = true;
    }
  }

  // ── Cluster filtering ─────────────────────────────────────────────
  if (min_cluster_size_ > 1) {
    filter_clusters(is_obstacle_, min_cluster_size_, max_cluster_gap_beams_);
  }

  // ── Build output scans ────────────────────────────────────────────
  // Walls scan: obstacle beams → inf
  for (size_t i = 0; i < n; ++i) {
    wall_ranges_[i] = is_obstacle_[i] ? INF : ranges[i];
  }

  // Obstacles scan: wall beams → inf
  for (size_t i = 0; i < n; ++i) {
    obstacle_ranges_[i] = is_obstacle_[i] ? ranges[i] : INF;
  }

  // ── Publish ───────────────────────────────────────────────────────
  sensor_msgs::msg::LaserScan walls_msg;
  walls_msg.header          = scan->header;
  walls_msg.angle_min       = scan->angle_min;
  walls_msg.angle_max       = scan->angle_max;
  walls_msg.angle_increment = scan->angle_increment;
  walls_msg.time_increment  = scan->time_increment;
  walls_msg.scan_time       = scan->scan_time;
  walls_msg.range_min       = scan->range_min;
  walls_msg.range_max       = scan->range_max;
  walls_msg.ranges.assign(wall_ranges_.begin(), wall_ranges_.end());

  sensor_msgs::msg::LaserScan obs_msg;
  obs_msg.header          = scan->header;
  obs_msg.angle_min       = scan->angle_min;
  obs_msg.angle_max       = scan->angle_max;
  obs_msg.angle_increment = scan->angle_increment;
  obs_msg.time_increment  = scan->time_increment;
  obs_msg.scan_time       = scan->scan_time;
  obs_msg.range_min       = scan->range_min;
  obs_msg.range_max       = scan->range_max;
  obs_msg.ranges.assign(obstacle_ranges_.begin(), obstacle_ranges_.end());

  walls_pub_->publish(walls_msg);
  obstacles_pub_->publish(obs_msg);

  // ── Timing ──────────────────────────────────────────────────────
  const auto t_end = std::chrono::high_resolution_clock::now();
  const double proc_ms = std::chrono::duration<double, std::milli>(
      t_end - t_start).count();

  std_msgs::msg::Float64 timing_msg;
  timing_msg.data = proc_ms;
  splitter_timing_pub_->publish(timing_msg);
}

// ────────────────────────────────────────────────────────────────────────────
//  Cluster filter — keeps only obstacle runs >= min_size
// ────────────────────────────────────────────────────────────────────────────

void ScanSplitterNode::filter_clusters(std::vector<bool> & mask, int min_size, int max_gap) const
{
  const int n = static_cast<int>(mask.size());

  // Bridge tiny false gaps between obstacle runs so sparse/decimated scans
  // do not fragment a single object into many one-beam clusters.
  if (max_gap > 0) {
    int i = 0;
    while (i < n) {
      while (i < n && mask[i]) ++i;
      const int gap_start = i;
      while (i < n && !mask[i]) ++i;
      const int gap_end = i;
      const int gap_len = gap_end - gap_start;

      if (
        gap_len > 0 && gap_len <= max_gap &&
        gap_start > 0 && gap_end < n &&
        mask[gap_start - 1] && mask[gap_end])
      {
        for (int k = gap_start; k < gap_end; ++k) {
          mask[k] = true;
        }
      }
    }
  }

  int i = 0;
  while (i < n) {
    if (mask[i]) {
      int j = i;
      while (j < n && mask[j]) ++j;
      if (j - i < min_size) {
        for (int k = i; k < j; ++k) mask[k] = false;
      }
      i = j;
    } else {
      ++i;
    }
  }
}

}  // namespace f1tenth_lidar

// ────────────────────────────────────────────────────────────────────────────
//  Main
// ────────────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<f1tenth_lidar::ScanSplitterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
