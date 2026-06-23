// Copyright (c) 2025 Pascal — MIT License
#include "f1tenth_localization/core/pipeline_latency_monitor.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <algorithm>

namespace f1tenth_localization
{

PipelineLatencyMonitor::PipelineLatencyMonitor(
  const rclcpp::NodeOptions & options)
: Node("pipeline_latency_monitor", options)
{
  declare_parameter("scan_topic", std::string("/scan"));
  declare_parameter("amcl_topic", std::string("/amcl_pose"));
  declare_parameter("amcl_particle_count_topic", std::string("/amcl_particle_count"));
  declare_parameter("amcl_timing_topic", std::string("/amcl_timing"));
  declare_parameter("amcl_gpu_timing_topic", std::string("/amcl_gpu_timing"));
  declare_parameter("ekf_topic", std::string("/ekf_pose"));
  declare_parameter("drive_topic", std::string("/drive"));
  declare_parameter("ackermann_topic", std::string("/ackermann_cmd"));
  declare_parameter("stage_match_max_ms", 20.0);
  declare_parameter("amcl_aux_max_age_ms", 100.0);
  declare_parameter("strict_mode", false);
  declare_parameter("print_every", 40);  // print every N cycles (~1 Hz at 40 Hz)
  declare_parameter("log_to_csv", true);
  declare_parameter("csv_output_dir", std::string("f1tenth_localization/Benchmark/Matlab/csv"));

  scan_topic_ = get_parameter("scan_topic").as_string();
  amcl_topic_ = get_parameter("amcl_topic").as_string();
  amcl_particle_count_topic_ = get_parameter("amcl_particle_count_topic").as_string();
  amcl_timing_topic_ = get_parameter("amcl_timing_topic").as_string();
  amcl_gpu_timing_topic_ = get_parameter("amcl_gpu_timing_topic").as_string();
  ekf_topic_ = get_parameter("ekf_topic").as_string();
  drive_topic_ = get_parameter("drive_topic").as_string();
  ackermann_topic_ = get_parameter("ackermann_topic").as_string();
  stage_match_max_ms_ = get_parameter("stage_match_max_ms").as_double();
  amcl_aux_max_age_ms_ = get_parameter("amcl_aux_max_age_ms").as_double();
  strict_mode_ = get_parameter("strict_mode").as_bool();
  print_every_ = get_parameter("print_every").as_int();
  log_to_csv_ = get_parameter("log_to_csv").as_bool();
  csv_output_dir_ = get_parameter("csv_output_dir").as_string();

  initialize_csv_logging();

  auto sensor_qos = rclcpp::SensorDataQoS();

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, sensor_qos,
    std::bind(&PipelineLatencyMonitor::scan_callback, this,
              std::placeholders::_1));

  amcl_sub_ = create_subscription<
    geometry_msgs::msg::PoseWithCovarianceStamped>(
    amcl_topic_, rclcpp::QoS(10),
    std::bind(&PipelineLatencyMonitor::amcl_callback, this,
              std::placeholders::_1));

  amcl_particle_count_sub_ = create_subscription<std_msgs::msg::Int32>(
    amcl_particle_count_topic_, rclcpp::QoS(50),
    std::bind(&PipelineLatencyMonitor::amcl_particle_count_callback, this,
              std::placeholders::_1));

  amcl_timing_sub_ = create_subscription<std_msgs::msg::Float64>(
    amcl_timing_topic_, rclcpp::QoS(50),
    std::bind(&PipelineLatencyMonitor::amcl_timing_callback, this,
              std::placeholders::_1));

  amcl_gpu_timing_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
    amcl_gpu_timing_topic_, rclcpp::QoS(50),
    std::bind(&PipelineLatencyMonitor::amcl_gpu_timing_callback, this,
              std::placeholders::_1));

  ekf_sub_ = create_subscription<
    geometry_msgs::msg::PoseWithCovarianceStamped>(
    ekf_topic_, rclcpp::QoS(10),
    std::bind(&PipelineLatencyMonitor::ekf_callback, this,
              std::placeholders::_1));

  drive_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    drive_topic_, rclcpp::QoS(20),
    std::bind(&PipelineLatencyMonitor::drive_callback, this,
              std::placeholders::_1));

  ackermann_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    ackermann_topic_, rclcpp::QoS(20),
    std::bind(&PipelineLatencyMonitor::ackermann_callback, this,
              std::placeholders::_1));

  RCLCPP_INFO(get_logger(),
    "Pipeline Latency Monitor started (print every %d cycles)", print_every_);
  RCLCPP_INFO(get_logger(),
    "  Tracking: %s -> %s -> %s -> %s -> %s",
    scan_topic_.c_str(), amcl_topic_.c_str(),
    ekf_topic_.c_str(), drive_topic_.c_str(), ackermann_topic_.c_str());
  RCLCPP_INFO(get_logger(),
    "  AMCL aux topics: particle_count=%s timing=%s gpu_timing=%s",
    amcl_particle_count_topic_.c_str(), amcl_timing_topic_.c_str(),
    amcl_gpu_timing_topic_.c_str());
  RCLCPP_INFO(get_logger(), "  Strict mode: %s", strict_mode_ ? "ON" : "OFF");

  if (log_to_csv_) {
    if (!csv_path_.empty()) {
      RCLCPP_INFO(get_logger(), "  CSV logging: %s", csv_path_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "  CSV logging requested, but file could not be created");
    }
  }
}

PipelineLatencyMonitor::~PipelineLatencyMonitor()
{
  if (csv_file_.is_open()) {
    csv_file_.flush();
    csv_file_.close();
  }
}

int64_t PipelineLatencyMonitor::stamp_to_key(
  const builtin_interfaces::msg::Time & stamp) const
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL +
         static_cast<int64_t>(stamp.nanosec);
}

double PipelineLatencyMonitor::wall_clock_ns() const
{
  return static_cast<double>(this->now().nanoseconds());
}

void PipelineLatencyMonitor::scan_callback(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & msg)
{
  const double recv = wall_clock_ns();
  const int64_t key = stamp_to_key(msg->header.stamp);

  std::lock_guard<std::mutex> lk(mutex_);
  auto & e = entries_[key];
  e.scan_recv_ns = recv;
  e.has_scan = true;

  cleanup_old_entries();
}

void PipelineLatencyMonitor::amcl_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg)
{
  const double recv = wall_clock_ns();
  const int64_t key = stamp_to_key(msg->header.stamp);

  std::lock_guard<std::mutex> lk(mutex_);
  auto it = entries_.find(key);
  if (it == entries_.end()) return;

  if (it->second.has_amcl) {
    return;
  }

  it->second.amcl_recv_ns = recv;
  it->second.has_amcl = true;

  const double max_aux_age_ns = amcl_aux_max_age_ms_ * 1e6;
  if (latest_amcl_particle_count_ >= 0 &&
      (amcl_aux_max_age_ms_ <= 0.0 ||
       std::fabs(recv - latest_amcl_particle_count_recv_ns_) <= max_aux_age_ns))
  {
    it->second.amcl_particle_count = latest_amcl_particle_count_;
  }
  if (latest_amcl_processing_ms_ >= 0.0 &&
      (amcl_aux_max_age_ms_ <= 0.0 ||
       std::fabs(recv - latest_amcl_processing_recv_ns_) <= max_aux_age_ns))
  {
    it->second.amcl_processing_ms = latest_amcl_processing_ms_;
  }
  if (latest_amcl_gpu_timing_recv_ns_ > 0.0 &&
      (amcl_aux_max_age_ms_ <= 0.0 ||
       std::fabs(recv - latest_amcl_gpu_timing_recv_ns_) <= max_aux_age_ns))
  {
    it->second.cpu_to_gpu_scan_ms = latest_cpu_to_gpu_scan_ms_;
    it->second.gpu_to_cpu_particles_ms = latest_gpu_to_cpu_particles_ms_;
    it->second.gpu_to_cpu_weights_ms = latest_gpu_to_cpu_weights_ms_;
    it->second.cpu_gpu_transfer_total_ms = latest_cpu_gpu_transfer_total_ms_;
    it->second.cpu_to_gpu_scan_bytes = latest_cpu_to_gpu_scan_bytes_;
    it->second.gpu_to_cpu_particles_bytes = latest_gpu_to_cpu_particles_bytes_;
    it->second.gpu_to_cpu_weights_bytes = latest_gpu_to_cpu_weights_bytes_;
    it->second.amcl_pose_compute_ms = latest_amcl_pose_compute_ms_;
    it->second.amcl_predict_ms = latest_amcl_predict_ms_;
    it->second.amcl_sensor_model_ms = latest_amcl_sensor_model_ms_;
    it->second.amcl_normalize_ms = latest_amcl_normalize_ms_;
    it->second.amcl_scan_confidence_ms = latest_amcl_scan_confidence_ms_;
    it->second.amcl_update_weights_total_ms = latest_amcl_update_weights_total_ms_;
    it->second.amcl_cluster_estimate_ms = latest_amcl_cluster_estimate_ms_;
    it->second.amcl_resample_ms = latest_amcl_resample_ms_;
    it->second.amcl_kld_target_ms = latest_amcl_kld_target_ms_;
    it->second.amcl_full_compute_ms = latest_amcl_full_compute_ms_;
    it->second.amcl_callback_to_pose_publish_ms = latest_amcl_callback_to_pose_publish_ms_;
    it->second.amcl_pose_published = latest_amcl_pose_published_;
    it->second.amcl_cluster_weight = latest_amcl_cluster_weight_;
    it->second.amcl_raycast_setup_ms = latest_amcl_raycast_setup_ms_;
    it->second.amcl_raycast_score_ms = latest_amcl_raycast_score_ms_;
    it->second.amcl_raycast_correction_ms = latest_amcl_raycast_correction_ms_;
  }
}

void PipelineLatencyMonitor::amcl_particle_count_callback(
  const std_msgs::msg::Int32::ConstSharedPtr & msg)
{
  const double recv = wall_clock_ns();
  std::lock_guard<std::mutex> lk(mutex_);
  if (msg->data >= 0) {
    latest_amcl_particle_count_ = msg->data;
    latest_amcl_particle_count_recv_ns_ = recv;
  }
}

void PipelineLatencyMonitor::amcl_timing_callback(
  const std_msgs::msg::Float64::ConstSharedPtr & msg)
{
  const double recv = wall_clock_ns();
  std::lock_guard<std::mutex> lk(mutex_);
  if (std::isfinite(msg->data) && msg->data >= 0.0) {
    latest_amcl_processing_ms_ = msg->data;
    latest_amcl_processing_recv_ns_ = recv;
  }
}

void PipelineLatencyMonitor::amcl_gpu_timing_callback(
  const std_msgs::msg::Float64MultiArray::ConstSharedPtr & msg)
{
  if (msg->data.size() < 9) {
    return;
  }
  const double recv = wall_clock_ns();
  std::lock_guard<std::mutex> lk(mutex_);
  latest_cpu_to_gpu_scan_ms_ = msg->data[0];
  latest_gpu_to_cpu_particles_ms_ = msg->data[1];
  latest_gpu_to_cpu_weights_ms_ = msg->data[2];
  latest_cpu_gpu_transfer_total_ms_ = msg->data[3];
  latest_cpu_to_gpu_scan_bytes_ = msg->data[4];
  latest_gpu_to_cpu_particles_bytes_ = msg->data[5];
  latest_gpu_to_cpu_weights_bytes_ = msg->data[6];
  latest_amcl_pose_compute_ms_ = msg->data.size() > 9 ? msg->data[9] : -1.0;
  latest_amcl_predict_ms_ = msg->data.size() > 10 ? msg->data[10] : -1.0;
  latest_amcl_sensor_model_ms_ = msg->data.size() > 11 ? msg->data[11] : -1.0;
  latest_amcl_normalize_ms_ = msg->data.size() > 12 ? msg->data[12] : -1.0;
  latest_amcl_scan_confidence_ms_ = msg->data.size() > 13 ? msg->data[13] : -1.0;
  latest_amcl_update_weights_total_ms_ = msg->data.size() > 14 ? msg->data[14] : -1.0;
  latest_amcl_cluster_estimate_ms_ = msg->data.size() > 15 ? msg->data[15] : -1.0;
  latest_amcl_resample_ms_ = msg->data.size() > 16 ? msg->data[16] : -1.0;
  latest_amcl_kld_target_ms_ = msg->data.size() > 17 ? msg->data[17] : -1.0;
  latest_amcl_full_compute_ms_ = msg->data.size() > 18 ? msg->data[18] : -1.0;
  latest_amcl_callback_to_pose_publish_ms_ = msg->data.size() > 19 ? msg->data[19] : -1.0;
  latest_amcl_pose_published_ = msg->data.size() > 20 ? msg->data[20] : -1.0;
  latest_amcl_cluster_weight_ = msg->data.size() > 21 ? msg->data[21] : -1.0;
  latest_amcl_raycast_setup_ms_ = msg->data.size() > 22 ? msg->data[22] : -1.0;
  latest_amcl_raycast_score_ms_ = msg->data.size() > 23 ? msg->data[23] : -1.0;
  latest_amcl_raycast_correction_ms_ = msg->data.size() > 24 ? msg->data[24] : -1.0;
  latest_amcl_gpu_timing_recv_ns_ = recv;
}

void PipelineLatencyMonitor::ekf_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg)
{
  const double recv = wall_clock_ns();
  const int64_t ekf_key = stamp_to_key(msg->header.stamp);

  std::lock_guard<std::mutex> lk(mutex_);

  auto mark_ekf = [this, recv](int64_t key, int64_t stamp_key) {
      auto it = entries_.find(key);
      if (it == entries_.end()) {
        return false;
      }
      if (!it->second.has_scan || !it->second.has_amcl || it->second.has_ekf) {
        return false;
      }

      it->second.ekf_recv_ns = recv;
      it->second.ekf_stamp_key = stamp_key;
      it->second.has_ekf = true;

      pending_drive_entries_.push_back(PendingStageEntry{key, stamp_key, recv});
      if (pending_drive_entries_.size() > 2000) {
        if (strict_mode_) {
          strict_queue_overrun_count_ += 1000;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Strict mode: pending drive queue overrun, dropped 1000 entries (total=%llu)",
            static_cast<unsigned long long>(strict_queue_overrun_count_));
        }
        pending_drive_entries_.erase(
          pending_drive_entries_.begin(), pending_drive_entries_.begin() + 1000);
      }
      return true;
    };

  if (ekf_key > 0 && mark_ekf(ekf_key, ekf_key)) {
    return;
  }

  if (ekf_key <= 0 || stage_match_max_ms_ <= 0.0) {
    return;
  }

  const double max_window_ns = stage_match_max_ms_ * 1e6;
  int64_t fallback_key = 0;
  double oldest_amcl_recv_ns = 0.0;

  for (const auto & [key, entry] : entries_) {
    if (!entry.has_scan || !entry.has_amcl || entry.has_ekf) {
      continue;
    }
    if (recv < entry.amcl_recv_ns) {
      continue;
    }
    if ((recv - entry.amcl_recv_ns) > max_window_ns) {
      continue;
    }
    if (fallback_key == 0 || entry.amcl_recv_ns < oldest_amcl_recv_ns) {
      fallback_key = key;
      oldest_amcl_recv_ns = entry.amcl_recv_ns;
    }
  }

  if (fallback_key != 0) {
    (void)mark_ekf(fallback_key, ekf_key);
  }
}

void PipelineLatencyMonitor::drive_callback(
  const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr & msg)
{
  const double recv = wall_clock_ns();
  const int64_t drive_key = stamp_to_key(msg->header.stamp);

  std::lock_guard<std::mutex> lk(mutex_);
  if (pending_drive_entries_.empty()) {
    // EKF can publish faster than AMCL; only scan-linked EKF outputs are tracked here.
    return;
  }

  if (stage_match_max_ms_ <= 0.0) {
    return;
  }

  const double max_window_ns = stage_match_max_ms_ * 1e6;

  PendingStageEntry matched;
  if (drive_key > 0) {
    const auto match_it = std::find_if(
      pending_drive_entries_.begin(), pending_drive_entries_.end(),
      [drive_key](const PendingStageEntry & entry) {
        return entry.stamp_key == drive_key;
      });

    if (match_it == pending_drive_entries_.end()) {
      pending_drive_entries_.erase(
        std::remove_if(
          pending_drive_entries_.begin(), pending_drive_entries_.end(),
          [this, recv, max_window_ns](const PendingStageEntry & entry) {
            if ((recv - entry.stage_recv_ns) > max_window_ns) {
              if (strict_mode_) {
                ++strict_stale_unmatched_count_;
                RCLCPP_WARN_THROTTLE(
                  get_logger(), *get_clock(), 2000,
                  "Strict mode: stale unmatched EKF entry dropped before /drive match (total=%llu)",
                  static_cast<unsigned long long>(strict_stale_unmatched_count_));
              }
              try_report(entry.key, -1.0);
              return true;
            }
            return false;
          }),
        pending_drive_entries_.end());

      if (strict_mode_) {
        ++strict_drive_without_pending_count_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Strict mode: /drive stamp did not match any pending EKF entry (total=%llu)",
          static_cast<unsigned long long>(strict_drive_without_pending_count_));
      }
      return;
    }

    matched = *match_it;
    pending_drive_entries_.erase(match_it);
  } else {
    while (!pending_drive_entries_.empty() &&
           (recv - pending_drive_entries_.front().stage_recv_ns) > max_window_ns)
    {
      const auto stale = pending_drive_entries_.front();
      pending_drive_entries_.erase(pending_drive_entries_.begin());
      if (strict_mode_) {
        ++strict_stale_unmatched_count_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Strict mode: stale unmatched EKF entry dropped before /drive match (total=%llu)",
          static_cast<unsigned long long>(strict_stale_unmatched_count_));
      }
      try_report(stale.key, -1.0);
    }

    if (pending_drive_entries_.empty()) {
      return;
    }

    matched = pending_drive_entries_.front();
    pending_drive_entries_.erase(pending_drive_entries_.begin());
  }

  const double measured_ms = (recv - matched.stage_recv_ns) * 1e-6;
  if (measured_ms < 0.0 || measured_ms > 5000.0) {
    try_report(matched.key, -1.0);
    return;
  }

  auto it = entries_.find(matched.key);
  if (it == entries_.end()) {
    return;
  }

  it->second.drive_recv_ns = recv;
  it->second.drive_stamp_key = drive_key;
  it->second.has_drive = true;

  acc_ekf_to_drive_.push_back(measured_ms);

  pending_ackermann_entries_.push_back(PendingStageEntry{matched.key, drive_key, recv});
  if (pending_ackermann_entries_.size() > 2000) {
    if (strict_mode_) {
      strict_queue_overrun_count_ += 1000;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Strict mode: pending ackermann queue overrun, dropped 1000 entries (total=%llu)",
        static_cast<unsigned long long>(strict_queue_overrun_count_));
    }
    pending_ackermann_entries_.erase(
      pending_ackermann_entries_.begin(), pending_ackermann_entries_.begin() + 1000);
  }
}

void PipelineLatencyMonitor::ackermann_callback(
  const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr & msg)
{
  const double recv = wall_clock_ns();
  const int64_t ackermann_key = stamp_to_key(msg->header.stamp);

  std::lock_guard<std::mutex> lk(mutex_);
  if (pending_ackermann_entries_.empty()) {
    // Mux can forward high-rate controls that are not tied to a fresh AMCL sample.
    return;
  }

  if (stage_match_max_ms_ <= 0.0) {
    return;
  }

  const double max_window_ns = stage_match_max_ms_ * 1e6;

  PendingStageEntry matched;
  if (ackermann_key > 0) {
    const auto match_it = std::find_if(
      pending_ackermann_entries_.begin(), pending_ackermann_entries_.end(),
      [ackermann_key](const PendingStageEntry & entry) {
        return entry.stamp_key == ackermann_key;
      });

    if (match_it == pending_ackermann_entries_.end()) {
      pending_ackermann_entries_.erase(
        std::remove_if(
          pending_ackermann_entries_.begin(), pending_ackermann_entries_.end(),
          [this, recv, max_window_ns](const PendingStageEntry & entry) {
            if ((recv - entry.stage_recv_ns) > max_window_ns) {
              if (strict_mode_) {
                ++strict_stale_unmatched_count_;
                RCLCPP_WARN_THROTTLE(
                  get_logger(), *get_clock(), 2000,
                  "Strict mode: stale unmatched /drive entry dropped before /ackermann_cmd match (total=%llu)",
                  static_cast<unsigned long long>(strict_stale_unmatched_count_));
              }
              try_report(entry.key, -1.0);
              return true;
            }
            return false;
          }),
        pending_ackermann_entries_.end());

      if (strict_mode_) {
        ++strict_ackermann_without_pending_count_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Strict mode: /ackermann_cmd stamp did not match any pending /drive entry (total=%llu)",
          static_cast<unsigned long long>(strict_ackermann_without_pending_count_));
      }
      return;
    }

    matched = *match_it;
    pending_ackermann_entries_.erase(match_it);
  } else {
    while (!pending_ackermann_entries_.empty() &&
           (recv - pending_ackermann_entries_.front().stage_recv_ns) > max_window_ns)
    {
      const auto stale = pending_ackermann_entries_.front();
      pending_ackermann_entries_.erase(pending_ackermann_entries_.begin());
      if (strict_mode_) {
        ++strict_stale_unmatched_count_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Strict mode: stale unmatched /drive entry dropped before /ackermann_cmd match (total=%llu)",
          static_cast<unsigned long long>(strict_stale_unmatched_count_));
      }
      try_report(stale.key, -1.0);
    }

    if (pending_ackermann_entries_.empty()) {
      return;
    }

    matched = pending_ackermann_entries_.front();
    pending_ackermann_entries_.erase(pending_ackermann_entries_.begin());
  }

  const double drive_to_ackermann_ms = (recv - matched.stage_recv_ns) * 1e-6;
  if (drive_to_ackermann_ms < 0.0 || drive_to_ackermann_ms > 5000.0) {
    try_report(matched.key, -1.0);
    return;
  }

  auto it = entries_.find(matched.key);
  if (it == entries_.end()) {
    return;
  }

  it->second.ackermann_recv_ns = recv;
  it->second.ackermann_stamp_key = ackermann_key;
  it->second.has_ackermann = true;
  acc_drive_to_ackermann_.push_back(drive_to_ackermann_ms);

  try_report(matched.key, drive_to_ackermann_ms);
}

static double vec_mean(const std::vector<double> & v)
{
  if (v.empty()) return 0.0;
  return std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
}

static double vec_var(const std::vector<double> & v, double mean)
{
  if (v.size() < 2) return 0.0;
  double sum_sq = 0.0;
  for (const auto x : v) {
    const double d = x - mean;
    sum_sq += d * d;
  }
  return sum_sq / static_cast<double>(v.size() - 1);
}

static double vec_percentile(std::vector<double> v, double pct)
{
  if (v.empty()) return 0.0;
  pct = std::clamp(pct, 0.0, 100.0);
  const double pos = (pct / 100.0) * static_cast<double>(v.size() - 1);
  const size_t idx = static_cast<size_t>(pos);
  std::nth_element(v.begin(), v.begin() + idx, v.end());
  return v[idx];
}

void PipelineLatencyMonitor::try_report(int64_t key, double drive_to_ackermann_ms)
{
  auto it = entries_.find(key);
  if (it == entries_.end()) return;

  const auto & e = it->second;
  if (!e.has_scan || !e.has_amcl || !e.has_ekf) return;

  const double ns_to_ms = 1e-6;
  double scan_stamp_to_scan_ms = (e.scan_recv_ns - static_cast<double>(key)) * ns_to_ms;
  if (scan_stamp_to_scan_ms < 0.0 || scan_stamp_to_scan_ms > 10000.0) {
    scan_stamp_to_scan_ms = -1.0;
  }
  const double scan_to_amcl_ms = (e.amcl_recv_ns - e.scan_recv_ns) * ns_to_ms;
  const double amcl_to_ekf_ms = (e.ekf_recv_ns - e.amcl_recv_ns) * ns_to_ms;
  const double scan_to_ekf_ms = (e.ekf_recv_ns - e.scan_recv_ns) * ns_to_ms;
  const int32_t amcl_particle_count = e.amcl_particle_count;
  const double amcl_processing_ms = e.amcl_processing_ms;
  const double amcl_pose_compute_ms = e.amcl_pose_compute_ms;
  const double cpu_to_gpu_scan_ms = e.cpu_to_gpu_scan_ms;
  const double gpu_to_cpu_particles_ms = e.gpu_to_cpu_particles_ms;
  const double gpu_to_cpu_weights_ms = e.gpu_to_cpu_weights_ms;
  const double cpu_gpu_transfer_total_ms = e.cpu_gpu_transfer_total_ms;
  const double cpu_to_gpu_scan_bytes = e.cpu_to_gpu_scan_bytes;
  const double gpu_to_cpu_particles_bytes = e.gpu_to_cpu_particles_bytes;
  const double gpu_to_cpu_weights_bytes = e.gpu_to_cpu_weights_bytes;
  const double amcl_predict_ms = e.amcl_predict_ms;
  const double amcl_sensor_model_ms = e.amcl_sensor_model_ms;
  const double amcl_normalize_ms = e.amcl_normalize_ms;
  const double amcl_scan_confidence_ms = e.amcl_scan_confidence_ms;
  const double amcl_update_weights_total_ms = e.amcl_update_weights_total_ms;
  const double amcl_cluster_estimate_ms = e.amcl_cluster_estimate_ms;
  const double amcl_resample_ms = e.amcl_resample_ms;
  const double amcl_kld_target_ms = e.amcl_kld_target_ms;
  const double amcl_full_compute_ms = e.amcl_full_compute_ms;
  const double amcl_callback_to_pose_publish_ms = e.amcl_callback_to_pose_publish_ms;
  const double amcl_pose_published = e.amcl_pose_published;
  const double amcl_cluster_weight = e.amcl_cluster_weight;
  const double amcl_raycast_setup_ms = e.amcl_raycast_setup_ms;
  const double amcl_raycast_score_ms = e.amcl_raycast_score_ms;
  const double amcl_raycast_correction_ms = e.amcl_raycast_correction_ms;

  double ekf_to_drive_ms = -1.0;
  if (e.has_drive) {
    const double measured = (e.drive_recv_ns - e.ekf_recv_ns) * ns_to_ms;
    if (measured >= 0.0 && measured < 5000.0) {
      ekf_to_drive_ms = measured;
    }
  }

  double scan_to_ackermann_ms = -1.0;
  if (ekf_to_drive_ms >= 0.0 && drive_to_ackermann_ms >= 0.0) {
    scan_to_ackermann_ms = scan_to_ekf_ms + ekf_to_drive_ms + drive_to_ackermann_ms;
    acc_scan_to_ackermann_.push_back(scan_to_ackermann_ms);
  }

  if (scan_stamp_to_scan_ms >= 0.0) {
    acc_scan_stamp_to_scan_.push_back(scan_stamp_to_scan_ms);
  }
  acc_scan_to_amcl_.push_back(scan_to_amcl_ms);
  acc_amcl_to_ekf_.push_back(amcl_to_ekf_ms);
  acc_scan_to_ekf_.push_back(scan_to_ekf_ms);

  write_csv_row(
    key,
    scan_stamp_to_scan_ms,
    amcl_particle_count,
    amcl_processing_ms,
    amcl_pose_compute_ms,
    cpu_to_gpu_scan_ms,
    gpu_to_cpu_particles_ms,
    gpu_to_cpu_weights_ms,
    cpu_gpu_transfer_total_ms,
    cpu_to_gpu_scan_bytes,
    gpu_to_cpu_particles_bytes,
    gpu_to_cpu_weights_bytes,
    amcl_predict_ms,
    amcl_sensor_model_ms,
    amcl_normalize_ms,
    amcl_scan_confidence_ms,
    amcl_update_weights_total_ms,
    amcl_cluster_estimate_ms,
    amcl_resample_ms,
    amcl_kld_target_ms,
    amcl_full_compute_ms,
    amcl_callback_to_pose_publish_ms,
    amcl_pose_published,
    amcl_cluster_weight,
    amcl_raycast_setup_ms,
    amcl_raycast_score_ms,
    amcl_raycast_correction_ms,
    scan_to_amcl_ms,
    amcl_to_ekf_ms,
    scan_to_ekf_ms,
    ekf_to_drive_ms,
    drive_to_ackermann_ms,
    scan_to_ackermann_ms);

  entries_.erase(it);

  ++cycle_count_;
  if (cycle_count_ % print_every_ != 0) return;

  const double m_ss = vec_mean(acc_scan_stamp_to_scan_);
  const double m_sa = vec_mean(acc_scan_to_amcl_);
  const double m_ae = vec_mean(acc_amcl_to_ekf_);
  const double m_se = vec_mean(acc_scan_to_ekf_);
  const double m_ed = vec_mean(acc_ekf_to_drive_);
  const double m_da = vec_mean(acc_drive_to_ackermann_);
  const double m_sa2 = vec_mean(acc_scan_to_ackermann_);

  const double v_ss = vec_var(acc_scan_stamp_to_scan_, m_ss);
  const double v_sa = vec_var(acc_scan_to_amcl_, m_sa);
  const double v_ae = vec_var(acc_amcl_to_ekf_, m_ae);
  const double v_se = vec_var(acc_scan_to_ekf_, m_se);
  const double v_ed = vec_var(acc_ekf_to_drive_, m_ed);
  const double v_da = vec_var(acc_drive_to_ackermann_, m_da);
  const double v_sa2 = vec_var(acc_scan_to_ackermann_, m_sa2);

  const double p95_ss = vec_percentile(acc_scan_stamp_to_scan_, 95.0);
  const double p95_sa = vec_percentile(acc_scan_to_amcl_, 95.0);
  const double p95_ae = vec_percentile(acc_amcl_to_ekf_, 95.0);
  const double p95_se = vec_percentile(acc_scan_to_ekf_, 95.0);
  const double p95_ed = vec_percentile(acc_ekf_to_drive_, 95.0);
  const double p95_da = vec_percentile(acc_drive_to_ackermann_, 95.0);
  const double p95_sa2 = vec_percentile(acc_scan_to_ackermann_, 95.0);

  const int n = static_cast<int>(acc_scan_to_ekf_.size());

  RCLCPP_INFO(get_logger(),
    "\n"
    "  ┌─── Pipeline Latency (n=%d) ────────────────────────────────┐\n"
    "  │                         mean        var         p95      │\n"
    "  │ scan_stamp -> scan_rx : %7.2f ms   %7.2f ms²   %7.2f ms   │\n"
    "  │ scan -> amcl        : %7.2f ms   %7.2f ms²   %7.2f ms   │\n"
    "  │ amcl -> ekf         : %7.2f ms   %7.2f ms²   %7.2f ms   │\n"
    "  │ scan -> ekf (total) : %7.2f ms   %7.2f ms²   %7.2f ms   │\n"
    "  │ ekf -> drive        : %7.2f ms   %7.2f ms²   %7.2f ms   │\n"
    "  │ drive -> ackermann  : %7.2f ms   %7.2f ms²   %7.2f ms   │\n"
    "  │ scan -> ackermann   : %7.2f ms   %7.2f ms²   %7.2f ms   │\n"
    "  └───────────────────────────────────────────────────────────┘",
    n,
    m_ss, v_ss, p95_ss,
    m_sa, v_sa, p95_sa,
    m_ae, v_ae, p95_ae,
    m_se, v_se, p95_se,
    m_ed, v_ed, p95_ed,
    m_da, v_da, p95_da,
    m_sa2, v_sa2, p95_sa2);

  if (strict_mode_) {
    RCLCPP_INFO(
      get_logger(),
      "  Strict mismatch counters: queue_overrun=%llu  stale_unmatched=%llu  drive_without_pending=%llu  ackermann_without_pending=%llu",
      static_cast<unsigned long long>(strict_queue_overrun_count_),
      static_cast<unsigned long long>(strict_stale_unmatched_count_),
      static_cast<unsigned long long>(strict_drive_without_pending_count_),
      static_cast<unsigned long long>(strict_ackermann_without_pending_count_));
  }

  acc_scan_stamp_to_scan_.clear();
  acc_scan_to_amcl_.clear();
  acc_amcl_to_ekf_.clear();
  acc_scan_to_ekf_.clear();
  acc_ekf_to_drive_.clear();
  acc_drive_to_ackermann_.clear();
  acc_scan_to_ackermann_.clear();
}

void PipelineLatencyMonitor::cleanup_old_entries()
{
  if (entries_.size() <= 200) {
    return;
  }

  const size_t to_remove = entries_.size() / 2;
  for (size_t i = 0; i < to_remove; ++i) {
    auto it = entries_.begin();
    if (it == entries_.end()) {
      break;
    }
    entries_.erase(it);
  }
}

void PipelineLatencyMonitor::initialize_csv_logging()
{
  if (!log_to_csv_) {
    return;
  }

  try {
    if (csv_output_dir_.empty()) {
      csv_output_dir_ = "f1tenth_localization/Benchmark/Matlab/csv";
    }

    std::filesystem::create_directories(csv_output_dir_);

    const auto now = std::chrono::system_clock::now();
    const auto secs = std::chrono::duration_cast<std::chrono::seconds>(
      now.time_since_epoch()).count();

    std::ostringstream filename;
    filename << "pipeline_latency_" << secs << ".csv";
    csv_path_ = (std::filesystem::path(csv_output_dir_) / filename.str()).string();

    csv_file_.open(csv_path_, std::ios::out | std::ios::trunc);
    if (!csv_file_.is_open()) {
      csv_path_.clear();
      RCLCPP_ERROR(get_logger(), "Failed to open CSV file for latency logging");
      return;
    }

    csv_file_ << "wall_time_ns,scan_stamp_ns,scan_stamp_to_scan_ms,amcl_particle_count,amcl_processing_ms,amcl_pose_compute_ms,cpu_to_gpu_scan_ms,gpu_to_cpu_particles_ms,gpu_to_cpu_weights_ms,cpu_gpu_transfer_total_ms,cpu_to_gpu_scan_bytes,gpu_to_cpu_particles_bytes,gpu_to_cpu_weights_bytes,amcl_predict_ms,amcl_sensor_model_ms,amcl_normalize_ms,amcl_scan_confidence_ms,amcl_update_weights_total_ms,amcl_cluster_estimate_ms,amcl_resample_ms,amcl_kld_target_ms,amcl_full_compute_ms,amcl_callback_to_pose_publish_ms,amcl_pose_published,amcl_cluster_weight,amcl_raycast_setup_ms,amcl_raycast_score_ms,amcl_raycast_correction_ms,scan_to_amcl_ms,amcl_to_ekf_ms,scan_to_ekf_ms,ekf_to_drive_ms,drive_to_ackermann_ms,scan_to_ackermann_ms\n";
    csv_file_.flush();
  } catch (const std::exception & e) {
    csv_path_.clear();
    RCLCPP_ERROR(get_logger(), "CSV logging init failed: %s", e.what());
  }
}

void PipelineLatencyMonitor::write_csv_row(
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
  double amcl_raycast_setup_ms,
  double amcl_raycast_score_ms,
  double amcl_raycast_correction_ms,
  double scan_to_amcl_ms,
  double amcl_to_ekf_ms,
  double scan_to_ekf_ms,
  double ekf_to_drive_ms,
  double drive_to_ackermann_ms,
  double scan_to_ackermann_ms)
{
  if (!log_to_csv_ || !csv_file_.is_open()) {
    return;
  }

  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::high_resolution_clock::now().time_since_epoch()).count();

  csv_file_ << now_ns << ','
            << stamp_ns << ','
            << std::fixed << std::setprecision(3)
            << scan_stamp_to_scan_ms << ','
            << amcl_particle_count << ','
            << amcl_processing_ms << ','
            << amcl_pose_compute_ms << ','
            << cpu_to_gpu_scan_ms << ','
            << gpu_to_cpu_particles_ms << ','
            << gpu_to_cpu_weights_ms << ','
            << cpu_gpu_transfer_total_ms << ','
            << cpu_to_gpu_scan_bytes << ','
            << gpu_to_cpu_particles_bytes << ','
            << gpu_to_cpu_weights_bytes << ','
            << amcl_predict_ms << ','
            << amcl_sensor_model_ms << ','
            << amcl_normalize_ms << ','
            << amcl_scan_confidence_ms << ','
            << amcl_update_weights_total_ms << ','
            << amcl_cluster_estimate_ms << ','
            << amcl_resample_ms << ','
            << amcl_kld_target_ms << ','
            << amcl_full_compute_ms << ','
            << amcl_callback_to_pose_publish_ms << ','
            << amcl_pose_published << ','
            << amcl_cluster_weight << ','
            << amcl_raycast_setup_ms << ','
            << amcl_raycast_score_ms << ','
            << amcl_raycast_correction_ms << ','
            << scan_to_amcl_ms << ','
            << amcl_to_ekf_ms << ','
            << scan_to_ekf_ms << ','
            << ekf_to_drive_ms << ','
            << drive_to_ackermann_ms << ','
            << scan_to_ackermann_ms << '\n';

  if (cycle_count_ % 50 == 0) {
    csv_file_.flush();
  }
}

}  // namespace f1tenth_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<f1tenth_localization::PipelineLatencyMonitor>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
