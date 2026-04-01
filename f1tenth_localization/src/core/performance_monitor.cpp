// Copyright (c) 2026 Pascal - MIT License
#include "f1tenth_localization/core/performance_monitor.hpp"
#include "SystemMonitor.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace f1tenth_localization
{

void PerformanceMonitor::RollingWindow::configure(size_t count)
{
  max_samples = std::max<size_t>(1, count);
  samples.clear();
  sum = 0.0;
}

void PerformanceMonitor::RollingWindow::push(double value)
{
  samples.push_back(value);
  sum += value;

  while (samples.size() > max_samples) {
    sum -= samples.front();
    samples.pop_front();
  }
}

double PerformanceMonitor::RollingWindow::average() const
{
  if (samples.empty()) {
    return 0.0;
  }
  return sum / static_cast<double>(samples.size());
}

PerformanceMonitor::PerformanceMonitor(const rclcpp::NodeOptions & options)
: Node("performance_monitor", options)
{
  output_dir_ = system_monitor_config::kOutputDir;
  cpu_sample_hz_ = system_monitor_config::kCpuSampleHz;
  csv_log_hz_ = system_monitor_config::kShortCsvLogHz;
  long_csv_log_hz_ = system_monitor_config::kLongCsvLogHz;
  print_hz_ = system_monitor_config::kPrintHz;
  rolling_window_long_sec_ = system_monitor_config::kRollingWindowLongSec;
  rolling_window_short_sec_ = system_monitor_config::kRollingWindowShortSec;

  const size_t long_samples = static_cast<size_t>(std::ceil(rolling_window_long_sec_ * cpu_sample_hz_));
  const size_t short_samples = static_cast<size_t>(std::ceil(rolling_window_short_sec_ * cpu_sample_hz_));

  long_window_.configure(long_samples);
  short_window_.configure(short_samples);

  const auto initial_snapshot = read_cpu_snapshot();
  if (!initial_snapshot.cores.empty()) {
    latest_per_core_cpu_percent_.assign(initial_snapshot.cores.size(), 0.0);
  } else {
    latest_per_core_cpu_percent_.assign(1, 0.0);
  }

  initialize_csv();

  running_.store(true);
  sampler_thread_ = std::thread(&PerformanceMonitor::sampling_loop, this);
  logger_thread_ = std::thread(&PerformanceMonitor::logging_loop, this);

  const auto print_period = std::chrono::duration<double>(1.0 / print_hz_);
  status_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(print_period),
    [this]() {
      print_status();
    });
}

PerformanceMonitor::~PerformanceMonitor()
{
  running_.store(false);

  if (sampler_thread_.joinable()) {
    sampler_thread_.join();
  }
  if (logger_thread_.joinable()) {
    logger_thread_.join();
  }

  if (long_csv_file_.is_open()) {
    long_csv_file_.flush();
    long_csv_file_.close();
  }
  if (short_csv_file_.is_open()) {
    short_csv_file_.flush();
    short_csv_file_.close();
  }
  if (per_core_csv_file_.is_open()) {
    per_core_csv_file_.flush();
    per_core_csv_file_.close();
  }
}

void PerformanceMonitor::initialize_csv()
{
  std::filesystem::create_directories(output_dir_);

  long_csv_path_ = (
    std::filesystem::path(output_dir_) / system_monitor_config::kLongCsvFileName).string();
  short_csv_path_ = (
    std::filesystem::path(output_dir_) / system_monitor_config::kShortCsvFileName).string();
  per_core_csv_path_ = (
    std::filesystem::path(output_dir_) / system_monitor_config::kPerCoreCsvFileName).string();

  long_csv_file_.open(long_csv_path_, std::ios::out | std::ios::trunc);
  if (!long_csv_file_.is_open()) {
    throw std::runtime_error("Failed to open long-window CPU CSV output file");
  }
  short_csv_file_.open(short_csv_path_, std::ios::out | std::ios::trunc);
  if (!short_csv_file_.is_open()) {
    throw std::runtime_error("Failed to open short-window CPU CSV output file");
  }

  per_core_csv_file_.open(per_core_csv_path_, std::ios::out | std::ios::trunc);
  if (!per_core_csv_file_.is_open()) {
    throw std::runtime_error("Failed to open per-core CPU CSV output file");
  }

  long_csv_file_ << "monotonic_time_ns,ros_time_sec,ros_time_nsec,cpu_long_window_percent\n";
  long_csv_file_.flush();

  short_csv_file_ << "monotonic_time_ns,ros_time_sec,ros_time_nsec,cpu_short_window_percent\n";
  short_csv_file_.flush();

  per_core_csv_file_ << "monotonic_time_ns,ros_time_sec,ros_time_nsec";
  for (size_t i = 0; i < latest_per_core_cpu_percent_.size(); ++i) {
    per_core_csv_file_ << ",cpu_core_" << i << "_percent";
  }
  per_core_csv_file_ << "\n";
  per_core_csv_file_.flush();
}

PerformanceMonitor::CpuSnapshot PerformanceMonitor::read_cpu_snapshot() const
{
  CpuSnapshot snapshot;

  std::ifstream stat_file("/proc/stat");
  if (!stat_file.is_open()) {
    return snapshot;
  }

  std::string line;
  while (std::getline(stat_file, line)) {
    if (line.rfind("cpu", 0) != 0) {
      break;
    }

    std::istringstream iss(line);
    std::string cpu_label;
    uint64_t user = 0;
    uint64_t nice = 0;
    uint64_t system = 0;
    uint64_t idle = 0;
    uint64_t iowait = 0;
    uint64_t irq = 0;
    uint64_t softirq = 0;
    uint64_t steal = 0;

    if (!(iss >> cpu_label >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal)) {
      continue;
    }

    const uint64_t idle_all = idle + iowait;
    const uint64_t total = user + nice + system + idle + iowait + irq + softirq + steal;

    if (cpu_label == "cpu") {
      snapshot.aggregate = CpuTimes{idle_all, total};
      snapshot.has_aggregate = true;
      continue;
    }

    if (cpu_label.size() > 3 && cpu_label[0] == 'c' && cpu_label[1] == 'p' && cpu_label[2] == 'u' &&
      cpu_label[3] >= '0' && cpu_label[3] <= '9')
    {
      snapshot.cores.push_back(CpuTimes{idle_all, total});
    }
  }

  return snapshot;
}

double PerformanceMonitor::compute_cpu_usage(const CpuTimes & current, double previous_usage)
{
  if (current.total == 0) {
    return previous_usage;
  }

  if (!cpu_initialized_) {
    prev_cpu_times_ = current;
    cpu_initialized_ = true;
    return previous_usage;
  }

  if (current.total < prev_cpu_times_.total || current.idle < prev_cpu_times_.idle) {
    prev_cpu_times_ = current;
    return previous_usage;
  }

  const uint64_t total_delta = current.total - prev_cpu_times_.total;
  const uint64_t idle_delta = current.idle - prev_cpu_times_.idle;
  prev_cpu_times_ = current;

  if (total_delta == 0) {
    return previous_usage;
  }

  const double busy = 1.0 - (static_cast<double>(idle_delta) / static_cast<double>(total_delta));
  return std::clamp(busy * 100.0, 0.0, 100.0);
}

std::vector<double> PerformanceMonitor::compute_per_core_usage(
  const std::vector<CpuTimes> & current,
  const std::vector<double> & previous_usage)
{
  std::vector<double> usage = previous_usage;
  if (usage.empty()) {
    usage.resize(current.size(), 0.0);
  }

  if (current.empty()) {
    return usage;
  }

  if (!per_core_initialized_) {
    prev_core_times_ = current;
    per_core_initialized_ = true;
    return usage;
  }

  if (prev_core_times_.size() != current.size() || current.size() != usage.size()) {
    prev_core_times_ = current;
    std::fill(usage.begin(), usage.end(), 0.0);
    return usage;
  }

  for (size_t i = 0; i < current.size(); ++i) {
    if (current[i].total < prev_core_times_[i].total || current[i].idle < prev_core_times_[i].idle) {
      continue;
    }

    const uint64_t total_delta = current[i].total - prev_core_times_[i].total;
    const uint64_t idle_delta = current[i].idle - prev_core_times_[i].idle;

    if (total_delta == 0) {
      continue;
    }

    const double busy = 1.0 - (static_cast<double>(idle_delta) / static_cast<double>(total_delta));
    usage[i] = std::clamp(busy * 100.0, 0.0, 100.0);
  }

  prev_core_times_ = current;
  return usage;
}

void PerformanceMonitor::sampling_loop()
{
  const auto period = std::chrono::duration<double>(1.0 / cpu_sample_hz_);
  const auto min_step = std::chrono::steady_clock::duration(1);
  const auto sample_step = std::max(
    min_step,
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(period));
  const auto sample_step_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(sample_step).count();

  auto next_tick = std::chrono::steady_clock::now();
  double cached_usage = 0.0;
  std::vector<double> cached_per_core = latest_per_core_cpu_percent_;

  auto advance_tick = [](std::chrono::steady_clock::time_point & tick,
      const std::chrono::steady_clock::duration & step,
      const std::chrono::steady_clock::time_point & now) {
      do {
        tick += step;
      } while (tick <= now);
    };

  while (running_.load()) {
    const auto now = std::chrono::steady_clock::now();

    if (now >= next_tick) {
      const auto lag_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now - next_tick).count();
      if (lag_ns > sample_step_ns) {
        const long long missed_ticks = sample_step_ns > 0 ? (lag_ns / sample_step_ns) : 0;
        RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Sampling loop lag: %.3f ms behind schedule (missed %lld ticks). System may be overloaded.",
          static_cast<double>(lag_ns) / 1e6,
          missed_ticks);
      }

      const auto snapshot = read_cpu_snapshot();
      if (snapshot.has_aggregate) {
        cached_usage = compute_cpu_usage(snapshot.aggregate, cached_usage);
      } else {
        RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Failed to read aggregate CPU counters from /proc/stat.");
      }
      if (!snapshot.cores.empty()) {
        cached_per_core = compute_per_core_usage(snapshot.cores, cached_per_core);
      }

      long_window_.push(cached_usage);
      short_window_.push(cached_usage);

      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_long_cpu_percent_ = long_window_.average();
        latest_short_cpu_percent_ = short_window_.average();
        latest_per_core_cpu_percent_ = cached_per_core;
      }

      advance_tick(next_tick, sample_step, now);
    }

    std::this_thread::sleep_until(next_tick);
  }
}

void PerformanceMonitor::logging_loop()
{
  const auto short_period = std::chrono::duration<double>(1.0 / csv_log_hz_);
  const auto long_period = std::chrono::duration<double>(1.0 / long_csv_log_hz_);
  const auto min_step = std::chrono::steady_clock::duration(1);
  const auto short_log_step = std::max(
    min_step,
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(short_period));
  const auto long_log_step = std::max(
    min_step,
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(long_period));
  const auto short_log_step_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(short_log_step).count();
  const auto long_log_step_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(long_log_step).count();

  auto next_short_tick = std::chrono::steady_clock::now();
  auto next_long_tick = next_short_tick;

  auto advance_tick = [](std::chrono::steady_clock::time_point & tick,
      const std::chrono::steady_clock::duration & step,
      const std::chrono::steady_clock::time_point & now) {
      do {
        tick += step;
      } while (tick <= now);
    };

  while (running_.load()) {
    const auto now = std::chrono::steady_clock::now();
    bool write_short = false;
    bool write_long = false;

    if (now >= next_short_tick) {
      const auto short_lag_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now - next_short_tick).count();
      if (short_lag_ns > short_log_step_ns) {
        const long long missed_ticks = short_log_step_ns > 0 ? (short_lag_ns / short_log_step_ns) : 0;
        RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Short CSV logging lag: %.3f ms behind schedule (missed %lld ticks).",
          static_cast<double>(short_lag_ns) / 1e6,
          missed_ticks);
      }
      write_short = true;
      advance_tick(next_short_tick, short_log_step, now);
    }
    if (now >= next_long_tick) {
      const auto long_lag_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now - next_long_tick).count();
      if (long_lag_ns > long_log_step_ns) {
        const long long missed_ticks = long_log_step_ns > 0 ? (long_lag_ns / long_log_step_ns) : 0;
        RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Long CSV logging lag: %.3f ms behind schedule (missed %lld ticks).",
          static_cast<double>(long_lag_ns) / 1e6,
          missed_ticks);
      }
      write_long = true;
      advance_tick(next_long_tick, long_log_step, now);
    }

    if (write_short || write_long) {
      const auto monotonic_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()).count();
      const auto ros_now = get_clock()->now();
      const int64_t ros_total_ns = ros_now.nanoseconds();
      const int64_t ros_sec = ros_total_ns / 1000000000LL;
      const int64_t ros_nsec = ros_total_ns % 1000000000LL;

      double long_cpu = 0.0;
      double short_cpu = 0.0;
      std::vector<double> per_core_cpu;
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        long_cpu = latest_long_cpu_percent_;
        short_cpu = latest_short_cpu_percent_;
        per_core_cpu = latest_per_core_cpu_percent_;
      }

      if (write_long) {
        long_csv_file_ << monotonic_ns << ','
                       << ros_sec << ','
                       << ros_nsec << ','
                       << std::fixed << std::setprecision(3)
                       << long_cpu << '\n';
      }

      if (write_short) {
        short_csv_file_ << monotonic_ns << ','
                        << ros_sec << ','
                        << ros_nsec << ','
                        << std::fixed << std::setprecision(3)
                        << short_cpu << '\n';

        per_core_csv_file_ << monotonic_ns << ','
                           << ros_sec << ','
                           << ros_nsec;
        for (const auto usage : per_core_cpu) {
          per_core_csv_file_ << ',' << std::fixed << std::setprecision(3) << usage;
        }
        per_core_csv_file_ << '\n';
      }

      if (write_long) {
        long_csv_file_.flush();
        short_csv_file_.flush();
        per_core_csv_file_.flush();
      }
    }

    const auto wake_up = std::min(next_short_tick, next_long_tick);
    std::this_thread::sleep_until(wake_up);
  }
}

void PerformanceMonitor::print_status()
{
  double long_cpu = 0.0;
  double short_cpu = 0.0;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    long_cpu = latest_long_cpu_percent_;
    short_cpu = latest_short_cpu_percent_;
  }

  RCLCPP_INFO(
    get_logger(),
    "CPU usage: long(%.3fs)=%.1f%% | short(%.6fs)=%.1f%%",
    rolling_window_long_sec_,
    long_cpu,
    rolling_window_short_sec_,
    short_cpu);
}

}  // namespace f1tenth_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<f1tenth_localization::PerformanceMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
