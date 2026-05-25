// Copyright (c) 2025 Pascal — MIT License
#pragma once

#include <unordered_map>
#include <vector>
#include <string>
#include <mutex>
#include <fstream>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>

namespace f1tenth_localization
{

/**
 * @brief Measures per-stage and total pipeline latency.
 *
 * Subscribes to:
 *   /scan         — raw LiDAR (pipeline start)
 *   /amcl_pose    — AMCL output (same header.stamp as /scan)
 *   /ekf_pose     — EKF output matched by scan/amcl stamp or first post-AMCL output
 *   /drive        — controller output matched by EKF header stamp
 *   /ackermann_cmd — mux output matched by drive header stamp
 *   /amcl_particle_count — AMCL active particle count
 *
 * Prints per-cycle:
 *   scan_stamp → scan_rx : X.XX ms
 *   scan → amcl         : X.XX ms
 *   amcl → ekf          : X.XX ms
 *   ekf → drive         : X.XX ms
 *   drive → ackermann   : X.XX ms
 *   scan → ackermann    : X.XX ms
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
    double amcl_recv_ns{0.0};
    double ekf_recv_ns{0.0};
    double drive_recv_ns{0.0};
    double ackermann_recv_ns{0.0};
    int64_t ekf_stamp_key{0};
    int64_t drive_stamp_key{0};
    int64_t ackermann_stamp_key{0};
    int32_t amcl_particle_count{-1};
    double amcl_processing_ms{-1.0};
    double amcl_pose_compute_ms{-1.0};
    double cpu_to_gpu_scan_ms{-1.0};
    double gpu_to_cpu_particles_ms{-1.0};
    double gpu_to_cpu_weights_ms{-1.0};
    double cpu_gpu_transfer_total_ms{-1.0};
    double cpu_to_gpu_scan_bytes{-1.0};
    double gpu_to_cpu_particles_bytes{-1.0};
    double gpu_to_cpu_weights_bytes{-1.0};
    double amcl_predict_ms{-1.0};
    double amcl_sensor_model_ms{-1.0};
    double amcl_normalize_ms{-1.0};
    double amcl_scan_confidence_ms{-1.0};
    double amcl_update_weights_total_ms{-1.0};
    double amcl_cluster_estimate_ms{-1.0};
    double amcl_resample_ms{-1.0};
    double amcl_kld_target_ms{-1.0};
    double amcl_full_compute_ms{-1.0};
    double amcl_callback_to_pose_publish_ms{-1.0};
    double amcl_pose_published{-1.0};
    double amcl_cluster_weight{-1.0};
    bool   has_scan{false};
    bool   has_amcl{false};
    bool   has_ekf{false};
    bool   has_drive{false};
    bool   has_ackermann{false};
  };

  void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & msg);
  void amcl_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg);
  void amcl_particle_count_callback(const std_msgs::msg::Int32::ConstSharedPtr & msg);
  void amcl_timing_callback(const std_msgs::msg::Float64::ConstSharedPtr & msg);
  void amcl_gpu_timing_callback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr & msg);
  void ekf_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg);
  void drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr & msg);
  void ackermann_callback(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr & msg);

  void try_report(int64_t key, double drive_to_ackermann_ms = -1.0);
  void cleanup_old_entries();
  void initialize_csv_logging();
  void write_csv_row(
    int64_t stamp_ns,
    double scan_stamp_to_scan_ms,
    int32_t amcl_particle_count,
    double amcl_processing_ms,
    double amcl_pose_compute_ms,
    double cpu_to_gpu_scan_ms,
    double gpu_to_cpu_particles_ms,
    double gpu_to_cpu_weights_ms,
    double cpu_gpu_transfer_total_ms,
    double cpu_to_gpu_scan_bytes,
    double gpu_to_cpu_particles_bytes,
    double gpu_to_cpu_weights_bytes,
    double amcl_predict_ms,
    double amcl_sensor_model_ms,
    double amcl_normalize_ms,
    double amcl_scan_confidence_ms,
    double amcl_update_weights_total_ms,
    double amcl_cluster_estimate_ms,
    double amcl_resample_ms,
    double amcl_kld_target_ms,
    double amcl_full_compute_ms,
    double amcl_callback_to_pose_publish_ms,
    double amcl_pose_published,
    double amcl_cluster_weight,
    double scan_to_amcl_ms,
    double amcl_to_ekf_ms,
    double scan_to_ekf_ms,
    double ekf_to_drive_ms,
    double drive_to_ackermann_ms,
    double scan_to_ackermann_ms);

  double wall_clock_ns() const;
  int64_t stamp_to_key(const builtin_interfaces::msg::Time & stamp) const;

  std::mutex mutex_;
  std::unordered_map<int64_t, PipelineEntry> entries_;

  // Configurable topics
  std::string scan_topic_;
  std::string amcl_topic_;
  std::string amcl_particle_count_topic_;
  std::string amcl_timing_topic_;
  std::string amcl_gpu_timing_topic_;
  std::string ekf_topic_;
  std::string drive_topic_;
  std::string ackermann_topic_;
  double stage_match_max_ms_{20.0};
  double amcl_aux_max_age_ms_{100.0};
  bool strict_mode_{false};

  uint64_t strict_queue_overrun_count_{0};
  uint64_t strict_stale_unmatched_count_{0};
  uint64_t strict_drive_without_pending_count_{0};
  uint64_t strict_ackermann_without_pending_count_{0};

  // CSV logging
  bool log_to_csv_{true};
  std::string csv_output_dir_;
  std::string csv_path_;
  std::ofstream csv_file_;

  // Print rate limiting
  int print_every_{1};
  int cycle_count_{0};

  int32_t latest_amcl_particle_count_{-1};
  double latest_amcl_particle_count_recv_ns_{0.0};
  double latest_amcl_processing_ms_{-1.0};
  double latest_amcl_processing_recv_ns_{0.0};
  double latest_amcl_pose_compute_ms_{-1.0};
  double latest_cpu_to_gpu_scan_ms_{-1.0};
  double latest_gpu_to_cpu_particles_ms_{-1.0};
  double latest_gpu_to_cpu_weights_ms_{-1.0};
  double latest_cpu_gpu_transfer_total_ms_{-1.0};
  double latest_cpu_to_gpu_scan_bytes_{-1.0};
  double latest_gpu_to_cpu_particles_bytes_{-1.0};
  double latest_gpu_to_cpu_weights_bytes_{-1.0};
  double latest_amcl_predict_ms_{-1.0};
  double latest_amcl_sensor_model_ms_{-1.0};
  double latest_amcl_normalize_ms_{-1.0};
  double latest_amcl_scan_confidence_ms_{-1.0};
  double latest_amcl_update_weights_total_ms_{-1.0};
  double latest_amcl_cluster_estimate_ms_{-1.0};
  double latest_amcl_resample_ms_{-1.0};
  double latest_amcl_kld_target_ms_{-1.0};
  double latest_amcl_full_compute_ms_{-1.0};
  double latest_amcl_callback_to_pose_publish_ms_{-1.0};
  double latest_amcl_pose_published_{-1.0};
  double latest_amcl_cluster_weight_{-1.0};
  double latest_amcl_gpu_timing_recv_ns_{0.0};

  // Accumulators for mean/variance over print_every_ cycles
  std::vector<double> acc_scan_stamp_to_scan_;
  std::vector<double> acc_scan_to_amcl_;
  std::vector<double> acc_amcl_to_ekf_;
  std::vector<double> acc_scan_to_ekf_;
  std::vector<double> acc_ekf_to_drive_;
  std::vector<double> acc_drive_to_ackermann_;
  std::vector<double> acc_scan_to_ackermann_;

  struct PendingStageEntry {
    int64_t key{0};          // scan/amcl pipeline key used for final reporting
    int64_t stamp_key{0};    // header stamp expected on the next stage
    double stage_recv_ns{0.0};
  };

  // EKF-complete pipeline entries waiting for corresponding /drive message.
  std::vector<PendingStageEntry> pending_drive_entries_;

  // Drive-complete pipeline entries waiting for /ackermann_cmd message.
  std::vector<PendingStageEntry> pending_ackermann_entries_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<
    geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr amcl_particle_count_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr amcl_timing_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr amcl_gpu_timing_sub_;
  rclcpp::Subscription<
    geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ekf_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_sub_;
};

}  // namespace f1tenth_localization
