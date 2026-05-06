// Copyright (c) 2025 Pascal — MIT License
#include "f1tenth_lidar/scan_splitter_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <chrono>
#include <functional>

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
  // Configuration comes from compile-time defines in scan_splitter_config.hpp.

  // ── TF ────────────────────────────────────────────────────────────
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ── Subscribers ───────────────────────────────────────────────────
  // Map: RELIABLE + TRANSIENT_LOCAL (latched)
  rclcpp::QoS map_qos(1);
  map_qos.reliable().transient_local();
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    SCAN_SPLITTER_MAP_TOPIC, map_qos,
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
  obstacles_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(obstacles_topic_, pub_qos);

  // ── Timing publisher ─────────────────────────────────────────────
  splitter_timing_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "splitter_timing", rclcpp::QoS(10));

  RCLCPP_INFO(this->get_logger(), "Scan Splitter Node (C++) initialized");
  RCLCPP_INFO(this->get_logger(), "  Splitting enabled: %s", enable_splitting_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  Input:  %s", scan_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Obstacles: %s", obstacles_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Wall reject radius: %.2f m", obstacle_threshold_);
  RCLCPP_INFO(this->get_logger(), "  Wall tolerance band: %.2f m", wall_tolerance_);
  RCLCPP_INFO(this->get_logger(), "  Min cluster size: %d beams", min_cluster_size_);
  RCLCPP_INFO(this->get_logger(), "  Max cluster gap: %d beams", max_cluster_gap_beams_);
  RCLCPP_INFO(this->get_logger(), "  Max obstacle cluster width: %.2f m", max_cluster_width_);
  RCLCPP_INFO(this->get_logger(), "  Raycast splitting: %s", enable_raycast_splitting_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  Occlusion threshold: %.2f m", occlusion_threshold_);
  RCLCPP_INFO(this->get_logger(), "  Raycast wall hit tolerance: %.2f m", raycast_wall_hit_tolerance_);
}

// ────────────────────────────────────────────────────────────────────────────
//  Map handling — precompute metric distance field with Dijkstra
// ────────────────────────────────────────────────────────────────────────────

void ScanSplitterNode::compute_distance_field(
  const nav_msgs::msg::OccupancyGrid & grid)
{
  const int w = static_cast<int>(grid.info.width);
  const int h = static_cast<int>(grid.info.height);
  const float res = static_cast<float>(grid.info.resolution);
  const auto & data = grid.data;

  // Distances are stored directly in metres.
  std::vector<float> dist_m(w * h, std::numeric_limits<float>::infinity());
  using QueueItem = std::pair<float, int>;  // (distance_m, flat_index)
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> pq;

  // Seed: occupied cells (value >= 50) have distance 0.
  // Unknown cells are treated as free to avoid overly conservative wall distances.
  for (int i = 0; i < w * h; ++i) {
    int8_t v = static_cast<int8_t>(data[i]);
    if (v >= 50) {
      dist_m[i] = 0.0f;
      pq.emplace(0.0f, i);
    }
  }

  // 8-connected propagation with metric edge costs.
  static constexpr int dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static constexpr int dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  const float diag_cost = std::sqrt(2.0f) * res;
  const float step_cost[8] = {res, res, res, res, diag_cost, diag_cost, diag_cost, diag_cost};

  while (!pq.empty()) {
    const auto [cur_dist, idx] = pq.top();
    pq.pop();
    if (cur_dist > dist_m[idx]) {
      continue;
    }

    const int cx = idx % w;
    const int cy = idx / w;

    for (int d = 0; d < 8; ++d) {
      const int nx = cx + dx8[d];
      const int ny = cy + dy8[d];
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
      const int nidx = ny * w + nx;
      const float new_dist = cur_dist + step_cost[d];
      if (new_dist < dist_m[nidx]) {
        dist_m[nidx] = new_dist;
        pq.emplace(new_dist, nidx);
      }
    }
  }

  distance_field_ = std::move(dist_m);
}

void ScanSplitterNode::map_callback(
  const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg)
{
  map_resolution_ = msg->info.resolution;
  map_width_      = static_cast<int>(msg->info.width);
  map_height_     = static_cast<int>(msg->info.height);
  map_origin_x_   = msg->info.origin.position.x;
  map_origin_y_   = msg->info.origin.position.y;

  occupied_map_.assign(msg->data.size(), 0);
  for (size_t i = 0; i < msg->data.size(); ++i) {
    occupied_map_[i] = (static_cast<int8_t>(msg->data[i]) >= 50) ? 1 : 0;
  }

  compute_distance_field(*msg);

  map_ready_ = true;
  RCLCPP_INFO(this->get_logger(),
    "Map received: %d×%d (res=%.3f m)", map_width_, map_height_, map_resolution_);
}

float ScanSplitterNode::distance_to_wall(float wx, float wy) const
{
  if (distance_field_.empty() || map_width_ <= 0 || map_height_ <= 0 || map_resolution_ <= 0.0) {
    return std::numeric_limits<float>::infinity();
  }

  const float inv_res = 1.0f / static_cast<float>(map_resolution_);
  const int px = static_cast<int>(std::floor((wx - static_cast<float>(map_origin_x_)) * inv_res));
  const int py = static_cast<int>(std::floor((wy - static_cast<float>(map_origin_y_)) * inv_res));
  if (px < 0 || px >= map_width_ || py < 0 || py >= map_height_) {
    return std::numeric_limits<float>::infinity();
  }

  return distance_field_[static_cast<size_t>(py * map_width_ + px)];
}

float ScanSplitterNode::raycast_wall_range(
  float laser_x,
  float laser_y,
  float world_angle,
  float range_min,
  float range_max) const
{
  if (distance_field_.empty() || map_width_ <= 0 || map_height_ <= 0 || map_resolution_ <= 0.0) {
    return std::numeric_limits<float>::infinity();
  }

  const float max_config_range = static_cast<float>(raycast_max_range_);
  const float max_range = max_config_range > 0.0f ? std::min(range_max, max_config_range) : range_max;
  const float configured_step = static_cast<float>(raycast_step_);
  const float step = configured_step > 0.0f ?
    configured_step :
    std::max(0.5f * static_cast<float>(map_resolution_), 0.01f);
  const float start = std::max(range_min, step);
  const float c = std::cos(world_angle);
  const float s = std::sin(world_angle);
  const float inv_res = 1.0f / static_cast<float>(map_resolution_);
  const float ox = static_cast<float>(map_origin_x_);
  const float oy = static_cast<float>(map_origin_y_);
  const float wall_hit_tolerance = static_cast<float>(
    std::max(0.0, raycast_wall_hit_tolerance_));

  for (float r = start; r <= max_range; r += step) {
    const float wx = laser_x + r * c;
    const float wy = laser_y + r * s;
    const int px = static_cast<int>(std::floor((wx - ox) * inv_res));
    const int py = static_cast<int>(std::floor((wy - oy) * inv_res));
    if (px < 0 || px >= map_width_ || py < 0 || py >= map_height_) {
      return std::numeric_limits<float>::infinity();
    }

    const size_t idx = static_cast<size_t>(py * map_width_ + px);
    if ((!occupied_map_.empty() && occupied_map_[idx] != 0) ||
        distance_field_[idx] <= wall_hit_tolerance)
    {
      return r;
    }
  }

  return std::numeric_limits<float>::infinity();
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
    auto empty = *scan;
    empty.ranges.assign(n, INF);
    obstacles_pub_->publish(empty);
    return;
  }

  if (!map_ready_) {
    auto empty = *scan;
    empty.ranges.assign(n, INF);
    obstacles_pub_->publish(empty);
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
      "TF lookup failed: %s — publishing empty obstacle scan", ex.what());

    auto empty = *scan;
    empty.ranges.assign(n, INF);
    obstacles_pub_->publish(empty);
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
    is_obstacle_.resize(n);
    obstacle_ranges_.resize(n);

    // Precompute beam angles (constant for a given scanner config)
    for (size_t i = 0; i < n; ++i) {
      angles_[i] = scan->angle_min + static_cast<float>(i) * scan->angle_increment;
    }
  }

  const float * __restrict__ ranges = scan->ranges.data();
  const float range_min = scan->range_min;
  const float range_max = scan->range_max;
  const float threshold = static_cast<float>(obstacle_threshold_);
  const float wall_tolerance = static_cast<float>(wall_tolerance_);
  const float occlusion_threshold = static_cast<float>(occlusion_threshold_);
  const float corrected_x = laser_x;
  const float corrected_y = laser_y;
  const float corrected_yaw = laser_yaw;

  // ── Classify beams using TF pose only; no local scan-to-map correction ──
  for (size_t i = 0; i < n; ++i) {
    const float r = ranges[i];

    // Default state for this beam.
    is_obstacle_[i] = false;

    // Skip invalid beams
    if (!std::isfinite(r) || r <= range_min || r >= range_max) {
      continue;
    }

    const float world_angle = angles_[i] + corrected_yaw;
    const float ex = corrected_x + r * std::cos(world_angle);
    const float ey = corrected_y + r * std::sin(world_angle);
    const float dist_to_wall = distance_to_wall(ex, ey);

    if (std::isfinite(dist_to_wall) && dist_to_wall <= wall_tolerance) {
      is_obstacle_[i] = false;
      continue;
    }

    bool obstacle = false;
    bool classified_by_raycast = false;

    if (enable_raycast_splitting_) {
      const float expected_wall_range = raycast_wall_range(
        corrected_x, corrected_y, world_angle, range_min, range_max);
      if (std::isfinite(expected_wall_range)) {
        const float occlusion_depth = expected_wall_range - r;
        obstacle =
          std::isfinite(dist_to_wall) &&
          dist_to_wall > wall_tolerance &&
          occlusion_depth > occlusion_threshold;
      }
      classified_by_raycast = true;
    }

    if (!classified_by_raycast) {
      obstacle = std::isfinite(dist_to_wall) && dist_to_wall > threshold;
    }

    is_obstacle_[i] = obstacle;
  }

  // ── Cluster filtering ─────────────────────────────────────────────
  if (min_cluster_size_ > 1 || max_cluster_width_ > 0.0) {
    filter_clusters(
      is_obstacle_, scan->ranges, angles_,
      min_cluster_size_, max_cluster_gap_beams_, max_cluster_width_);
  }

  // ── Build output scans ────────────────────────────────────────────
  for (size_t i = 0; i < n; ++i) {
    if (is_obstacle_[i]) {
      obstacle_ranges_[i] = ranges[i];
    } else {
      obstacle_ranges_[i] = INF;
    }
  }

  // ── Publish ───────────────────────────────────────────────────────
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

void ScanSplitterNode::filter_clusters(
  std::vector<bool> & mask,
  const std::vector<float> & ranges,
  const std::vector<float> & angles,
  int min_size,
  int max_gap,
  double max_width_m) const
{
  const int n = static_cast<int>(mask.size());
  const double max_width = std::max(0.0, max_width_m);

  auto cluster_width = [&](int start, int end) -> double {
    if (start < 0 || end <= start || end > n ||
        ranges.size() != mask.size() || angles.size() != mask.size())
    {
      return 0.0;
    }

    const int last = end - 1;
    const float r0 = ranges[static_cast<size_t>(start)];
    const float r1 = ranges[static_cast<size_t>(last)];
    if (!std::isfinite(r0) || !std::isfinite(r1)) {
      return 0.0;
    }

    const double x0 = static_cast<double>(r0) * std::cos(angles[static_cast<size_t>(start)]);
    const double y0 = static_cast<double>(r0) * std::sin(angles[static_cast<size_t>(start)]);
    const double x1 = static_cast<double>(r1) * std::cos(angles[static_cast<size_t>(last)]);
    const double y1 = static_cast<double>(r1) * std::sin(angles[static_cast<size_t>(last)]);
    return std::hypot(x1 - x0, y1 - y0);
  };

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
      const bool too_small = j - i < min_size;
      const bool too_wide = max_width > 0.0 && cluster_width(i, j) > max_width;
      if (too_small || too_wide) {
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
