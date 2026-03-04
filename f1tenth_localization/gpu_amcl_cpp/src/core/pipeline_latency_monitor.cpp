// Copyright (c) 2025 Pascal — MIT License
#include "gpu_amcl_cpp/core/pipeline_latency_monitor.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <numeric>

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
  declare_parameter("print_every", 40);  // print every N cycles (~1 Hz at 40 Hz)

  scan_topic_  = get_parameter("scan_topic").as_string();
  walls_topic_ = get_parameter("walls_topic").as_string();
  amcl_topic_  = get_parameter("amcl_topic").as_string();
  ekf_topic_   = get_parameter("ekf_topic").as_string();
  print_every_ = get_parameter("print_every").as_int();

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

  RCLCPP_INFO(get_logger(),
    "Pipeline Latency Monitor started (print every %d cycles)", print_every_);
  RCLCPP_INFO(get_logger(),
    "  Tracking: %s → %s → %s → %s",
    scan_topic_.c_str(), walls_topic_.c_str(),
    amcl_topic_.c_str(), ekf_topic_.c_str());
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

  it->second.amcl_recv_ns = recv;
  it->second.has_amcl = true;

  // Remember this key so EKF callback can match to it
  last_amcl_key_ = key;
}

void PipelineLatencyMonitor::ekf_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & /*msg*/)
{
  const double recv = wall_ns();

  std::lock_guard<std::mutex> lk(mutex_);
  if (last_amcl_key_ == 0) return;

  auto it = entries_.find(last_amcl_key_);
  if (it == entries_.end()) return;
  if (!it->second.has_amcl) return;  // safety
  if (it->second.has_ekf) return;    // already matched

  it->second.ekf_recv_ns = recv;
  it->second.has_ekf = true;

  // This entry is now complete — report it
  try_report(last_amcl_key_);
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

void PipelineLatencyMonitor::try_report(int64_t key)
{
  // Caller holds mutex_
  auto it = entries_.find(key);
  if (it == entries_.end()) return;

  const auto & e = it->second;
  if (!e.has_scan || !e.has_walls || !e.has_amcl || !e.has_ekf) return;

  // Convert nanoseconds to milliseconds and accumulate
  const double ns_to_ms = 1e-6;
  acc_scan_to_walls_.push_back((e.walls_recv_ns - e.scan_recv_ns) * ns_to_ms);
  acc_walls_to_amcl_.push_back((e.amcl_recv_ns - e.walls_recv_ns) * ns_to_ms);
  acc_amcl_to_ekf_.push_back((e.ekf_recv_ns   - e.amcl_recv_ns)  * ns_to_ms);
  acc_scan_to_ekf_.push_back((e.ekf_recv_ns   - e.scan_recv_ns)  * ns_to_ms);

  entries_.erase(it);

  ++cycle_count_;
  if (cycle_count_ % print_every_ != 0) return;

  // Compute mean and variance for each stage
  const double m_sw = vec_mean(acc_scan_to_walls_);
  const double m_wa = vec_mean(acc_walls_to_amcl_);
  const double m_ae = vec_mean(acc_amcl_to_ekf_);
  const double m_se = vec_mean(acc_scan_to_ekf_);

  const double v_sw = vec_var(acc_scan_to_walls_, m_sw);
  const double v_wa = vec_var(acc_walls_to_amcl_, m_wa);
  const double v_ae = vec_var(acc_amcl_to_ekf_,   m_ae);
  const double v_se = vec_var(acc_scan_to_ekf_,   m_se);

  const int n = static_cast<int>(acc_scan_to_ekf_.size());

  RCLCPP_INFO(get_logger(),
    "\n"
    "  ┌─── Pipeline Latency (n=%d) ────────────────────────────────┐\n"
    "  │                         mean        var                    │\n"
    "  │ scan → scan_walls  : %7.2f ms   %7.2f ms²              │\n"
    "  │ scan_walls → amcl  : %7.2f ms   %7.2f ms²              │\n"
    "  │ amcl → ekf         : %7.2f ms   %7.2f ms²              │\n"
    "  │ scan → ekf (total) : %7.2f ms   %7.2f ms²              │\n"
    "  └───────────────────────────────────────────────────────────┘",
    n,
    m_sw, v_sw,
    m_wa, v_wa,
    m_ae, v_ae,
    m_se, v_se);

  // Reset accumulators
  acc_scan_to_walls_.clear();
  acc_walls_to_amcl_.clear();
  acc_amcl_to_ekf_.clear();
  acc_scan_to_ekf_.clear();
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
