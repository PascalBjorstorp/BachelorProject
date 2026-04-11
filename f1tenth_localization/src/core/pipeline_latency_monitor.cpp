// Copyright (c) 2025 Pascal — MIT License
#include "f1tenth_localization/core/pipeline_latency_monitor.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <algorithm>
#include <thread>

namespace f1tenth_localization
{

static double wall_ns()
{
  return static_cast<double>(
    std::chrono::high_resolution_clock::now().time_since_epoch().count());
}

PipelineLatencyMonitor::PipelineLatencyMonitor(
  const rclcpp::NodeOptions & options)
: Node("pipeline_latency_monitor", options)
{
  declare_parameter("scan_topic", std::string("/scan"));
  declare_parameter("walls_topic", std::string("/scan_walls"));
  declare_parameter("amcl_topic", std::string("/amcl_pose"));
  declare_parameter("ekf_topic", std::string("/ekf_pose"));
  declare_parameter("command_topic", std::string("/commands/motor/speed"));
  declare_parameter("command_match_max_ms", 20.0);
  declare_parameter("strict_mode", false);
  declare_parameter("print_every", 40);  // print every N cycles (~1 Hz at 40 Hz)
  declare_parameter("log_to_csv", true);
  declare_parameter("csv_output_dir", std::string("f1tenth_localization/Benchmark/Matlab/csv"));

  scan_topic_  = get_parameter("scan_topic").as_string();
  walls_topic_ = get_parameter("walls_topic").as_string();
  amcl_topic_  = get_parameter("amcl_topic").as_string();
  ekf_topic_   = get_parameter("ekf_topic").as_string();
  command_topic_ = get_parameter("command_topic").as_string();
  command_match_max_ms_ = get_parameter("command_match_max_ms").as_double();
  strict_mode_ = get_parameter("strict_mode").as_bool();
  print_every_ = get_parameter("print_every").as_int();
  log_to_csv_ = get_parameter("log_to_csv").as_bool();
  csv_output_dir_ = get_parameter("csv_output_dir").as_string();

  initialize_csv_logging();

  // All subscriptions use BEST_EFFORT to match sensor data QoS
  auto sensor_qos = rclcpp::SensorDataQoS();

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, sensor_qos,
    std::bind(&PipelineLatencyMonitor::scan_callback, this,
              std::placeholders::_1));

  walls_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    walls_topic_, rclcpp::QoS(10),  // RELIABLE to match splitter publisher
    std::bind(&PipelineLatencyMonitor::walls_callback, this,
              std::placeholders::_1));

  amcl_sub_ = create_subscription<
    geometry_msgs::msg::PoseWithCovarianceStamped>(
    amcl_topic_, rclcpp::QoS(10),
    std::bind(&PipelineLatencyMonitor::amcl_callback, this,
              std::placeholders::_1));

  ekf_sub_ = create_subscription<
    geometry_msgs::msg::PoseWithCovarianceStamped>(
    ekf_topic_, rclcpp::QoS(10),
    std::bind(&PipelineLatencyMonitor::ekf_callback, this,
              std::placeholders::_1));

  command_sub_ = create_subscription<std_msgs::msg::Float64>(
    command_topic_, rclcpp::QoS(20),
    std::bind(&PipelineLatencyMonitor::command_callback, this,
              std::placeholders::_1));

  RCLCPP_INFO(get_logger(),
    "Pipeline Latency Monitor started (print every %d cycles)", print_every_);
  RCLCPP_INFO(get_logger(),
    "  Tracking: %s → %s → %s → %s → %s",
    scan_topic_.c_str(), walls_topic_.c_str(),
    amcl_topic_.c_str(), ekf_topic_.c_str(), command_topic_.c_str());
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

// ────────────────────────────────────────────────────────────────────────────
//  Callbacks — record wall-clock receive times
// ────────────────────────────────────────────────────────────────────────────

void PipelineLatencyMonitor::scan_callback(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & msg)
{
  const double recv = wall_ns();
  const int64_t key = stamp_to_key(msg->header.stamp);

  std::lock_guard<std::mutex> lk(mutex_);
  auto & e = entries_[key];
  e.scan_recv_ns = recv;
  e.has_scan = true;

  cleanup_old_entries();
}

void PipelineLatencyMonitor::walls_callback(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & msg)
{
  const double recv = wall_ns();
  const int64_t key = stamp_to_key(msg->header.stamp);

  std::lock_guard<std::mutex> lk(mutex_);
  auto it = entries_.find(key);
  if (it == entries_.end()) return;  // no matching scan

  it->second.walls_recv_ns = recv;
  it->second.has_walls = true;
}

void PipelineLatencyMonitor::amcl_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg)
{
  const double recv = wall_ns();
  const int64_t key = stamp_to_key(msg->header.stamp);

  std::lock_guard<std::mutex> lk(mutex_);
  auto it = entries_.find(key);
  if (it == entries_.end()) return;  // no matching scan

  if (it->second.has_amcl) {
    return;
  }

  it->second.amcl_recv_ns = recv;
  it->second.has_amcl = true;

  pending_ekf_keys_.push_back(key);
  if (pending_ekf_keys_.size() > 2000) {
    pending_ekf_keys_.erase(pending_ekf_keys_.begin(), pending_ekf_keys_.begin() + 1000);
  }
}

void PipelineLatencyMonitor::ekf_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg)
{
  const double recv = wall_ns();
  const int64_t ekf_key = stamp_to_key(msg->header.stamp);

  std::lock_guard<std::mutex> lk(mutex_);

  auto mark_ekf = [this, recv](int64_t key) {
      auto it = entries_.find(key);
      if (it == entries_.end()) {
        return false;
      }
      if (!it->second.has_scan || !it->second.has_amcl || it->second.has_ekf) {
        return false;
      }

      it->second.ekf_recv_ns = recv;
      it->second.has_ekf = true;

      pending_command_entries_.push_back(PendingCommandEntry{key, recv});
      if (pending_command_entries_.size() > 2000) {
        if (strict_mode_) {
          strict_queue_overrun_count_ += 1000;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Strict mode: pending command queue overrun, dropped 1000 entries (total=%llu)",
            static_cast<unsigned long long>(strict_queue_overrun_count_));
        }
        pending_command_entries_.erase(
          pending_command_entries_.begin(), pending_command_entries_.begin() + 1000);
      }
      return true;
    };

  // Prefer exact stamp key if EKF preserves input stamp.
  if (ekf_key > 0 && mark_ekf(ekf_key)) {
    return;
  }

  // Fallback for EKF outputs that overwrite stamps: consume oldest AMCL-ready key.
  while (!pending_ekf_keys_.empty()) {
    const int64_t key = pending_ekf_keys_.front();
    pending_ekf_keys_.pop_front();
    if (mark_ekf(key)) {
      return;
    }
  }
}

void PipelineLatencyMonitor::command_callback(
  const std_msgs::msg::Float64::ConstSharedPtr & /*msg*/)
{
  const double recv = wall_ns();

  std::lock_guard<std::mutex> lk(mutex_);
  if (pending_command_entries_.empty()) {
    if (strict_mode_) {
      ++strict_command_without_pending_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Strict mode: command arrived with no pending EKF entry (total=%llu)",
        static_cast<unsigned long long>(strict_command_without_pending_count_));
    }
    return;
  }

  if (command_match_max_ms_ <= 0.0) {
    return;
  }

  const double max_window_ns = command_match_max_ms_ * 1e6;

  // Drop stale EKF-complete entries and report them without command latency.
  while (!pending_command_entries_.empty() &&
         (recv - pending_command_entries_.front().ekf_recv_ns) > max_window_ns)
  {
    const auto stale = pending_command_entries_.front();
    pending_command_entries_.erase(pending_command_entries_.begin());
    if (strict_mode_) {
      ++strict_stale_unmatched_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Strict mode: stale unmatched EKF entry dropped before command match (total=%llu)",
        static_cast<unsigned long long>(strict_stale_unmatched_count_));
    }
    try_report(stale.key, -1.0);
  }

  if (pending_command_entries_.empty()) {
    return;
  }

  // Match to the oldest outstanding EKF-complete entry (FIFO pairing).
  const auto matched = pending_command_entries_.front();
  pending_command_entries_.erase(pending_command_entries_.begin());

  double ekf_to_command_ms = -1.0;
  const double measured_ms = (recv - matched.ekf_recv_ns) * 1e-6;
  if (measured_ms >= 0.0 && measured_ms < 5000.0) {
    ekf_to_command_ms = measured_ms;
    acc_ekf_to_command_.push_back(ekf_to_command_ms);
  }

  try_report(matched.key, ekf_to_command_ms);
}

// ────────────────────────────────────────────────────────────────────────────
//  Report
// ────────────────────────────────────────────────────────────────────────────

static double vec_mean(const std::vector<double> & v)
{
  if (v.empty()) return 0.0;
  return std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
}

static double vec_var(const std::vector<double> & v, double mean)
{
  if (v.size() < 2) return 0.0;
  double sum_sq = 0.0;
  for (auto x : v) {
    double d = x - mean;
    sum_sq += d * d;
  }
  return sum_sq / static_cast<double>(v.size() - 1);  // sample variance
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

void PipelineLatencyMonitor::try_report(int64_t key, double ekf_to_command_ms)
{
  // Caller holds mutex_
  auto it = entries_.find(key);
  if (it == entries_.end()) return;

  const auto & e = it->second;
  if (!e.has_scan || !e.has_walls || !e.has_amcl || !e.has_ekf) return;

  // Convert nanoseconds to milliseconds and accumulate
  const double ns_to_ms = 1e-6;
  const double scan_to_walls_ms = (e.walls_recv_ns - e.scan_recv_ns) * ns_to_ms;
  const double walls_to_amcl_ms = (e.amcl_recv_ns - e.walls_recv_ns) * ns_to_ms;
  const double amcl_to_ekf_ms = (e.ekf_recv_ns   - e.amcl_recv_ns)  * ns_to_ms;
  const double scan_to_ekf_ms = (e.ekf_recv_ns   - e.scan_recv_ns)  * ns_to_ms;

  if (ekf_to_command_ms >= 0.0) {
    acc_scan_to_command_.push_back(scan_to_ekf_ms + ekf_to_command_ms);
  }

  acc_scan_to_walls_.push_back(scan_to_walls_ms);
  acc_walls_to_amcl_.push_back(walls_to_amcl_ms);
  acc_amcl_to_ekf_.push_back(amcl_to_ekf_ms);
  acc_scan_to_ekf_.push_back(scan_to_ekf_ms);

  write_csv_row(
    key, scan_to_walls_ms, walls_to_amcl_ms, amcl_to_ekf_ms, scan_to_ekf_ms,
    ekf_to_command_ms);

  entries_.erase(it);

  ++cycle_count_;
  if (cycle_count_ % print_every_ != 0) return;

  // Compute mean and variance for each stage
  const double m_sw = vec_mean(acc_scan_to_walls_);
  const double m_wa = vec_mean(acc_walls_to_amcl_);
  const double m_ae = vec_mean(acc_amcl_to_ekf_);
  const double m_se = vec_mean(acc_scan_to_ekf_);
  const double m_ec = vec_mean(acc_ekf_to_command_);
  const double m_sc = vec_mean(acc_scan_to_command_);

  const double v_sw = vec_var(acc_scan_to_walls_, m_sw);
  const double v_wa = vec_var(acc_walls_to_amcl_, m_wa);
  const double v_ae = vec_var(acc_amcl_to_ekf_,   m_ae);
  const double v_se = vec_var(acc_scan_to_ekf_,   m_se);
  const double v_ec = vec_var(acc_ekf_to_command_,  m_ec);
  const double v_sc = vec_var(acc_scan_to_command_, m_sc);

  const double p95_sw = vec_percentile(acc_scan_to_walls_, 95.0);
  const double p95_wa = vec_percentile(acc_walls_to_amcl_, 95.0);
  const double p95_ae = vec_percentile(acc_amcl_to_ekf_, 95.0);
  const double p95_se = vec_percentile(acc_scan_to_ekf_, 95.0);
  const double p95_ec = vec_percentile(acc_ekf_to_command_, 95.0);
  const double p95_sc = vec_percentile(acc_scan_to_command_, 95.0);

  const int n = static_cast<int>(acc_scan_to_ekf_.size());

  RCLCPP_INFO(get_logger(),
    "\n"
    "  ┌─── Pipeline Latency (n=%d) ────────────────────────────────┐\n"
    "  │                         mean        var         p95      │\n"
    "  │ scan → scan_walls  : %7.2f ms   %7.2f ms²   %7.2f ms   │\n"
    "  │ scan_walls → amcl  : %7.2f ms   %7.2f ms²   %7.2f ms   │\n"
    "  │ amcl → ekf         : %7.2f ms   %7.2f ms²   %7.2f ms   │\n"
    "  │ scan → ekf (total) : %7.2f ms   %7.2f ms²   %7.2f ms   │\n"
    "  │ ekf → motor_cmd    : %7.2f ms   %7.2f ms²   %7.2f ms   │\n"
    "  │ scan → motor_cmd   : %7.2f ms   %7.2f ms²   %7.2f ms   │\n"
    "  └───────────────────────────────────────────────────────────┘",
    n,
    m_sw, v_sw, p95_sw,
    m_wa, v_wa, p95_wa,
    m_ae, v_ae, p95_ae,
    m_se, v_se, p95_se,
    m_ec, v_ec, p95_ec,
    m_sc, v_sc, p95_sc);

  if (strict_mode_) {
    RCLCPP_INFO(
      get_logger(),
      "  Strict mismatch counters: queue_overrun=%llu  stale_unmatched=%llu  command_without_pending=%llu",
      static_cast<unsigned long long>(strict_queue_overrun_count_),
      static_cast<unsigned long long>(strict_stale_unmatched_count_),
      static_cast<unsigned long long>(strict_command_without_pending_count_));
  }

  // Reset accumulators
  acc_scan_to_walls_.clear();
  acc_walls_to_amcl_.clear();
  acc_amcl_to_ekf_.clear();
  acc_scan_to_ekf_.clear();
  acc_ekf_to_command_.clear();
  acc_scan_to_command_.clear();
}

void PipelineLatencyMonitor::cleanup_old_entries()
{
  // Caller holds mutex_
  // Remove entries older than 2 seconds (avoid memory leak for dropped scans)
  if (entries_.size() > 200) {
    // Find the oldest entries and remove them
    auto oldest = entries_.begin();
    for (size_t i = 0; i < entries_.size() / 2; ++i) {
      entries_.erase(oldest++);
    }
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

    csv_file_ << "wall_time_ns,scan_stamp_ns,scan_to_scan_walls_ms,walls_to_amcl_ms,amcl_to_ekf_ms,scan_to_ekf_ms,ekf_to_command_ms,scan_to_command_ms\n";
    csv_file_.flush();
  } catch (const std::exception & e) {
    csv_path_.clear();
    RCLCPP_ERROR(get_logger(), "CSV logging init failed: %s", e.what());
  }
}

void PipelineLatencyMonitor::write_csv_row(
  int64_t stamp_ns,
  double scan_to_walls_ms,
  double walls_to_amcl_ms,
  double amcl_to_ekf_ms,
  double scan_to_ekf_ms,
  double ekf_to_command_ms)
{
  if (!log_to_csv_ || !csv_file_.is_open()) {
    return;
  }

  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::high_resolution_clock::now().time_since_epoch()).count();
  const double scan_to_command_ms =
    (ekf_to_command_ms >= 0.0) ? (scan_to_ekf_ms + ekf_to_command_ms) : -1.0;

  csv_file_ << now_ns << ','
            << stamp_ns << ','
            << std::fixed << std::setprecision(3)
            << scan_to_walls_ms << ','
            << walls_to_amcl_ms << ','
            << amcl_to_ekf_ms << ','
            << scan_to_ekf_ms << ','
            << ekf_to_command_ms << ','
            << scan_to_command_ms << '\n';

  if (cycle_count_ % 50 == 0) {
    csv_file_.flush();
  }
}

}  // namespace f1tenth_localization

// ────────────────────────────────────────────────────────────────────────────
//  Main
// ────────────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<f1tenth_localization::PipelineLatencyMonitor>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
