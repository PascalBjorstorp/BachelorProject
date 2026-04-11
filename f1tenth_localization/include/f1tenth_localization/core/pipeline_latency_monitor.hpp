// Copyright (c) 2025 Pascal — MIT License
#pragma once

#include <unordered_map>
#include <vector>
#include <string>
#include <mutex>
#include <fstream>
#include <deque>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

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
  ~PipelineLatencyMonitor() override;

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
  void command_callback(const std_msgs::msg::Float64::ConstSharedPtr & msg);

  void try_report(int64_t key, double ekf_to_command_ms = -1.0);
  void cleanup_old_entries();
  void initialize_csv_logging();
  void write_csv_row(
    int64_t stamp_ns,
    double scan_to_walls_ms,
    double walls_to_amcl_ms,
    double amcl_to_ekf_ms,
    double scan_to_ekf_ms,
    double ekf_to_command_ms);

  double wall_clock_ns() const;
  int64_t stamp_to_key(const builtin_interfaces::msg::Time & stamp) const;

  std::mutex mutex_;
  std::unordered_map<int64_t, PipelineEntry> entries_;

  // AMCL-complete entries waiting for EKF in arrival order.
  std::deque<int64_t> pending_ekf_keys_;

  // Configurable topics
  std::string scan_topic_;
  std::string walls_topic_;
  std::string amcl_topic_;
  std::string ekf_topic_;
  std::string command_topic_;
  double command_match_max_ms_{20.0};
  bool strict_mode_{false};

  uint64_t strict_queue_overrun_count_{0};
  uint64_t strict_stale_unmatched_count_{0};
  uint64_t strict_command_without_pending_count_{0};

  // CSV logging
  bool log_to_csv_{true};
  std::string csv_output_dir_;
  std::string csv_path_;
  std::ofstream csv_file_;

  // Print rate limiting
  int print_every_{1};
  int cycle_count_{0};

  // Accumulators for mean/variance over print_every_ cycles
  std::vector<double> acc_scan_to_walls_;
  std::vector<double> acc_walls_to_amcl_;
  std::vector<double> acc_amcl_to_ekf_;
  std::vector<double> acc_scan_to_ekf_;
  std::vector<double> acc_ekf_to_command_;
  std::vector<double> acc_scan_to_command_;

  struct PendingCommandEntry {
    int64_t key{0};
    double ekf_recv_ns{0.0};
  };

  // EKF-complete pipeline entries waiting for corresponding motor command.
  std::vector<PendingCommandEntry> pending_command_entries_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr walls_sub_;
  rclcpp::Subscription<
    geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Subscription<
    geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ekf_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr command_sub_;
};

}  // namespace f1tenth_localization
