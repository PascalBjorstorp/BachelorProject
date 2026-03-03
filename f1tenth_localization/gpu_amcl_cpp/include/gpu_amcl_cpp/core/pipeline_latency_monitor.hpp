// Copyright (c) 2025 Pascal — MIT License
#pragma once

#include <unordered_map>
#include <string>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace f1tenth_localization
{

/**
 * @brief Measures per-stage and total pipeline latency.
 *
 * Subscribes to:
 *   /scan         — raw LiDAR (pipeline start)
 *   /scan_walls   — scan splitter output (same header.stamp as /scan)
 *   /amcl_pose    — AMCL output (same header.stamp as /scan)
 *   /ekf_pose     — EKF output (uses now() as stamp, matched temporally)
 *
 * Prints per-cycle:
 *   scan → scan_walls   : X.XX ms
 *   scan_walls → amcl   : X.XX ms
 *   amcl → ekf          : X.XX ms
 *   scan → ekf (total)  : X.XX ms
 */
class PipelineLatencyMonitor : public rclcpp::Node
{
public:
  explicit PipelineLatencyMonitor(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Key = nanosecond timestamp from header.stamp (int64_t)
  struct PipelineEntry {
    double scan_recv_ns{0.0};
    double walls_recv_ns{0.0};
    double amcl_recv_ns{0.0};
    double ekf_recv_ns{0.0};
    bool   has_scan{false};
    bool   has_walls{false};
    bool   has_amcl{false};
    bool   has_ekf{false};
  };

  void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & msg);
  void walls_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & msg);
  void amcl_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg);
  void ekf_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg);

  void try_report(int64_t key);
  void cleanup_old_entries();

  double wall_clock_ns() const;
  int64_t stamp_to_key(const builtin_interfaces::msg::Time & stamp) const;

  std::mutex mutex_;
  std::unordered_map<int64_t, PipelineEntry> entries_;

  // For EKF matching: last AMCL stamp that we're waiting for EKF on
  int64_t last_amcl_key_{0};

  // Configurable topics
  std::string scan_topic_;
  std::string walls_topic_;
  std::string amcl_topic_;
  std::string ekf_topic_;

  // Print rate limiting
  int print_every_{1};
  int cycle_count_{0};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr walls_sub_;
  rclcpp::Subscription<
    geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Subscription<
    geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ekf_sub_;
};

}  // namespace f1tenth_localization
