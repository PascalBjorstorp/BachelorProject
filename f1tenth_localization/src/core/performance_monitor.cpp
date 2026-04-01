// Copyright (c) 2026 Pascal - MIT License
#include "f1tenth_localization/core/performance_monitor.hpp"
#include "SystemMonitor.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include <unistd.h>

namespace f1tenth_localization
{

namespace
{

bool starts_with(const std::string & value, const std::string & prefix)
{
  return value.size() >= prefix.size() && value.compare(0, prefix.size(), prefix) == 0;
}

bool is_numeric_name(const std::string & value)
{
  if (value.empty()) {
    return false;
  }

  return std::all_of(value.begin(), value.end(), [](unsigned char ch) {
    return std::isdigit(ch) != 0;
  });
}

std::string sanitize_csv_field(const std::string & value)
{
  std::string out = value;
  std::replace(out.begin(), out.end(), ',', '_');
  return out;
}

}  // namespace

void PerformanceMonitor::RollingWindow::configure(double window_sec)
{
  const auto seconds = std::chrono::duration<double>(std::max(1e-6, window_sec));
  horizon = std::chrono::duration_cast<std::chrono::steady_clock::duration>(seconds);
  samples.clear();
  sum = 0.0;
}

void PerformanceMonitor::RollingWindow::push(
  const std::chrono::steady_clock::time_point & stamp,
  double value)
{
  samples.emplace_back(stamp, value);
  sum += value;

  while (!samples.empty() && (stamp - samples.front().first) > horizon) {
    sum -= samples.front().second;
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
  gpu_sample_hz_ = system_monitor_config::kGpuSampleHz;
  node_process_sample_hz_ = system_monitor_config::kNodeProcessSampleHz;
  node_process_discovery_hz_ = system_monitor_config::kNodeProcessDiscoveryHz;
  csv_log_hz_ = system_monitor_config::kShortCsvLogHz;
  long_csv_log_hz_ = system_monitor_config::kLongCsvLogHz;
  print_hz_ = system_monitor_config::kPrintHz;
  rolling_window_long_sec_ = system_monitor_config::kRollingWindowLongSec;
  rolling_window_short_sec_ = system_monitor_config::kRollingWindowShortSec;

  if (gpu_sample_hz_ <= 0.0) {
    gpu_sample_hz_ = cpu_sample_hz_;
  }

  long_window_.configure(rolling_window_long_sec_);
  short_window_.configure(rolling_window_short_sec_);

  const auto initial_snapshot = read_cpu_snapshot();
  if (!initial_snapshot.cores.empty()) {
    latest_per_core_cpu_percent_.assign(initial_snapshot.cores.size(), 0.0);
  } else {
    latest_per_core_cpu_percent_.assign(1, 0.0);
  }

  initialize_gpu_source();

  initialize_csv();

  running_.store(true);
  sampler_thread_ = std::thread(&PerformanceMonitor::sampling_loop, this);
  if (gpu_source_ != GpuSource::none) {
    gpu_thread_ = std::thread(&PerformanceMonitor::gpu_loop, this);
  }
  if (node_process_sample_hz_ > 0.0) {
    process_thread_ = std::thread(&PerformanceMonitor::process_loop, this);
  }
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
  if (gpu_thread_.joinable()) {
    gpu_thread_.join();
  }
  if (process_thread_.joinable()) {
    process_thread_.join();
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
  if (gpu_csv_file_.is_open()) {
    gpu_csv_file_.flush();
    gpu_csv_file_.close();
  }
  if (node_process_csv_file_.is_open()) {
    node_process_csv_file_.flush();
    node_process_csv_file_.close();
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
  gpu_csv_path_ = (
    std::filesystem::path(output_dir_) / system_monitor_config::kGpuCsvFileName).string();
  node_process_csv_path_ = (
    std::filesystem::path(output_dir_) / system_monitor_config::kNodeProcessCsvFileName).string();

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

  gpu_csv_file_.open(gpu_csv_path_, std::ios::out | std::ios::trunc);
  if (!gpu_csv_file_.is_open()) {
    throw std::runtime_error("Failed to open GPU CSV output file");
  }

  node_process_csv_file_.open(node_process_csv_path_, std::ios::out | std::ios::trunc);
  if (!node_process_csv_file_.is_open()) {
    throw std::runtime_error("Failed to open node-process CSV output file");
  }

  long_csv_file_ << "monotonic_time_ns,ros_time_sec,ros_time_nsec,cpu_long_window_percent,gpu_percent\n";
  long_csv_file_.flush();

  short_csv_file_ << "monotonic_time_ns,ros_time_sec,ros_time_nsec,cpu_short_window_percent,gpu_percent\n";
  short_csv_file_.flush();

  per_core_csv_file_ << "monotonic_time_ns,ros_time_sec,ros_time_nsec";
  for (size_t i = 0; i < latest_per_core_cpu_percent_.size(); ++i) {
    per_core_csv_file_ << ",cpu_core_" << i << "_percent";
  }
  per_core_csv_file_ << "\n";
  per_core_csv_file_.flush();

  gpu_csv_file_ << "monotonic_time_ns,ros_time_sec,ros_time_nsec,gpu_percent\n";
  gpu_csv_file_.flush();

  node_process_csv_file_ <<
    "monotonic_time_ns,ros_time_sec,ros_time_nsec,node_name,pid,cpu_percent\n";
  node_process_csv_file_.flush();
}

void PerformanceMonitor::initialize_gpu_source()
{
  static constexpr std::array<const char *, 8> kJetsonGpuPaths = {
    "/sys/devices/gpu.0/load",
    "/sys/devices/platform/gpu.0/load",
    "/sys/devices/17000000.ga10b/load",
    "/sys/devices/platform/17000000.ga10b/load",
    "/sys/class/devfreq/17000000.ga10b/load",
    "/sys/devices/17000000.gv11b/load",
    "/sys/devices/platform/17000000.gv11b/load",
    "/sys/class/devfreq/17000000.gv11b/load"
  };

  for (const auto * path : kJetsonGpuPaths) {
    if (std::filesystem::exists(path)) {
      gpu_source_ = GpuSource::sysfs;
      gpu_load_path_ = path;
      return;
    }
  }

  if (std::system("command -v nvidia-smi > /dev/null 2>&1") == 0) {
    gpu_source_ = GpuSource::nvidia_smi;
    return;
  }

  gpu_source_ = GpuSource::none;
}

double PerformanceMonitor::read_gpu_percent() const
{
  switch (gpu_source_) {
    case GpuSource::sysfs:
      return read_gpu_percent_from_sysfs();
    case GpuSource::nvidia_smi:
      return read_gpu_percent_from_nvidia_smi();
    case GpuSource::none:
    default:
      return 0.0;
  }
}

double PerformanceMonitor::read_gpu_percent_from_sysfs() const
{
  if (gpu_load_path_.empty()) {
    return 0.0;
  }

  std::ifstream gpu_file(gpu_load_path_);
  if (!gpu_file.is_open()) {
    return 0.0;
  }

  double raw = 0.0;
  gpu_file >> raw;
  if (!gpu_file.good() && !gpu_file.eof()) {
    return 0.0;
  }

  if (raw > 100.0) {
    raw /= 10.0;
  }
  return std::clamp(raw, 0.0, 100.0);
}

double PerformanceMonitor::read_gpu_percent_from_nvidia_smi() const
{
  FILE * pipe = popen(
    "nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader,nounits 2>/dev/null",
    "r");
  if (pipe == nullptr) {
    return 0.0;
  }

  char buffer[256];
  double sum = 0.0;
  size_t count = 0;
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    try {
      const double value = std::stod(std::string(buffer));
      sum += value;
      ++count;
    } catch (...) {
      continue;
    }
  }
  pclose(pipe);

  if (count == 0) {
    return 0.0;
  }

  return std::clamp(sum / static_cast<double>(count), 0.0, 100.0);
}

std::vector<std::string> PerformanceMonitor::read_cmdline_tokens(int pid)
{
  std::vector<std::string> tokens;
  std::ifstream cmdline_file("/proc/" + std::to_string(pid) + "/cmdline", std::ios::binary);
  if (!cmdline_file.is_open()) {
    return tokens;
  }

  std::string token;
  while (std::getline(cmdline_file, token, '\0')) {
    if (!token.empty()) {
      tokens.push_back(token);
    }
  }

  return tokens;
}

std::string PerformanceMonitor::basename_from_path(const std::string & path)
{
  if (path.empty()) {
    return {};
  }

  const auto pos = path.find_last_of('/');
  if (pos == std::string::npos || pos + 1 >= path.size()) {
    return path;
  }

  return path.substr(pos + 1);
}

std::string PerformanceMonitor::extract_ros_node_name(const std::vector<std::string> & tokens)
{
  if (tokens.empty()) {
    return {};
  }

  bool has_ros_args = false;
  std::string node_name;
  std::string node_ns;

  auto parse_remap_token = [&node_name, &node_ns](const std::string & remap) {
      if (starts_with(remap, "__node:=") && remap.size() > 8) {
        node_name = remap.substr(8);
      } else if (starts_with(remap, "__ns:=") && remap.size() > 6) {
        node_ns = remap.substr(6);
      }
    };

  for (size_t i = 0; i < tokens.size(); ++i) {
    const auto & token = tokens[i];

    if (token == "--ros-args") {
      has_ros_args = true;
    }

    parse_remap_token(token);

    if ((token == "-r" || token == "--remap") && (i + 1) < tokens.size()) {
      parse_remap_token(tokens[i + 1]);
      continue;
    }

    if (starts_with(token, "--remap=") && token.size() > 8) {
      parse_remap_token(token.substr(8));
    }
  }

  if (!has_ros_args && node_name.empty()) {
    return {};
  }

  if (node_name.empty()) {
    node_name = basename_from_path(tokens.front());
  }

  if (node_name.empty()) {
    return {};
  }

  if (!node_ns.empty() && node_name.front() != '/') {
    if (node_ns.front() != '/') {
      node_ns.insert(node_ns.begin(), '/');
    }

    if (!node_ns.empty() && node_ns.back() == '/') {
      node_ns.pop_back();
    }

    if (!node_ns.empty() && node_ns != "/") {
      node_name = node_ns + "/" + node_name;
    }
  }

  return node_name;
}

std::vector<PerformanceMonitor::RosProcess> PerformanceMonitor::discover_ros_processes() const
{
  std::vector<RosProcess> processes;

  std::error_code ec;
  for (const auto & entry : std::filesystem::directory_iterator("/proc", ec)) {
    if (ec) {
      break;
    }
    if (!entry.is_directory()) {
      continue;
    }

    const auto pid_name = entry.path().filename().string();
    if (!is_numeric_name(pid_name)) {
      continue;
    }

    int pid = 0;
    try {
      pid = std::stoi(pid_name);
    } catch (...) {
      continue;
    }

    const auto tokens = read_cmdline_tokens(pid);
    const auto node_name = extract_ros_node_name(tokens);
    if (node_name.empty()) {
      continue;
    }

    processes.push_back(RosProcess{pid, node_name});
  }

  std::sort(
    processes.begin(),
    processes.end(),
    [](const RosProcess & a, const RosProcess & b) {
      if (a.node_name == b.node_name) {
        return a.pid < b.pid;
      }
      return a.node_name < b.node_name;
    });

  return processes;
}

bool PerformanceMonitor::read_process_cpu_times(int pid, ProcessCpuTimes & times) const
{
  std::ifstream stat_file("/proc/" + std::to_string(pid) + "/stat");
  if (!stat_file.is_open()) {
    return false;
  }

  std::string line;
  if (!std::getline(stat_file, line)) {
    return false;
  }

  const auto close_paren = line.rfind(')');
  if (close_paren == std::string::npos || (close_paren + 2) >= line.size()) {
    return false;
  }

  const std::string remainder = line.substr(close_paren + 2);
  std::istringstream iss(remainder);

  char state = 0;
  long long ppid = 0;
  long long pgrp = 0;
  long long session = 0;
  long long tty_nr = 0;
  long long tpgid = 0;
  unsigned long long flags = 0;
  unsigned long long minflt = 0;
  unsigned long long cminflt = 0;
  unsigned long long majflt = 0;
  unsigned long long cmajflt = 0;
  unsigned long long utime = 0;
  unsigned long long stime = 0;
  unsigned long long cutime = 0;
  unsigned long long cstime = 0;
  long long priority = 0;
  long long nice = 0;
  long long num_threads = 0;
  long long itrealvalue = 0;
  unsigned long long starttime = 0;

  if (!(iss >> state >> ppid >> pgrp >> session >> tty_nr >> tpgid >> flags >> minflt >> cminflt >>
    majflt >> cmajflt >> utime >> stime >> cutime >> cstime >> priority >> nice >> num_threads >>
    itrealvalue >> starttime))
  {
    return false;
  }

  (void)state;
  (void)ppid;
  (void)pgrp;
  (void)session;
  (void)tty_nr;
  (void)tpgid;
  (void)flags;
  (void)minflt;
  (void)cminflt;
  (void)majflt;
  (void)cmajflt;
  (void)cutime;
  (void)cstime;
  (void)priority;
  (void)nice;
  (void)num_threads;
  (void)itrealvalue;

  times.utime_ticks = static_cast<uint64_t>(utime);
  times.stime_ticks = static_cast<uint64_t>(stime);
  times.starttime_ticks = static_cast<uint64_t>(starttime);

  return true;
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

      long_window_.push(now, cached_usage);
      short_window_.push(now, cached_usage);

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

void PerformanceMonitor::gpu_loop()
{
  const auto period = std::chrono::duration<double>(1.0 / gpu_sample_hz_);
  const auto min_step = std::chrono::steady_clock::duration(1);
  auto gpu_step = std::max(
    min_step,
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(period));

  if (gpu_source_ == GpuSource::nvidia_smi) {
    const auto nvidia_smi_min_step =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::milliseconds(50));
    gpu_step = std::max(gpu_step, nvidia_smi_min_step);
  }

  const auto gpu_step_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(gpu_step).count();
  auto next_tick = std::chrono::steady_clock::now();

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
      if (lag_ns > gpu_step_ns) {
        const long long missed_ticks = gpu_step_ns > 0 ? (lag_ns / gpu_step_ns) : 0;
        RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "GPU loop lag: %.3f ms behind schedule (missed %lld ticks).",
          static_cast<double>(lag_ns) / 1e6,
          missed_ticks);
      }

      const double gpu = read_gpu_percent();
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_gpu_percent_ = gpu;
      }

      advance_tick(next_tick, gpu_step, now);
    }

    std::this_thread::sleep_until(next_tick);
  }
}

void PerformanceMonitor::process_loop()
{
  const auto sample_period = std::chrono::duration<double>(1.0 / std::max(1e-3, node_process_sample_hz_));
  const auto discovery_period =
    std::chrono::duration<double>(1.0 / std::max(1e-3, node_process_discovery_hz_));
  const auto min_step = std::chrono::steady_clock::duration(1);
  const auto sample_step = std::max(
    min_step,
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(sample_period));
  const auto discovery_step = std::max(
    min_step,
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(discovery_period));
  const auto sample_step_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(sample_step).count();

  const long ticks_per_second_raw = ::sysconf(_SC_CLK_TCK);
  const double ticks_per_second = ticks_per_second_raw > 0 ? static_cast<double>(ticks_per_second_raw) : 100.0;
  const unsigned int cpu_count_raw = std::thread::hardware_concurrency();
  const double cpu_count = static_cast<double>(std::max(1u, cpu_count_raw));
  const double max_cpu_percent = 100.0;

  struct PrevSample
  {
    ProcessCpuTimes times;
    std::chrono::steady_clock::time_point stamp;
  };

  std::unordered_map<std::string, PrevSample> prev_samples;
  std::vector<RosProcess> tracked_processes;

  auto next_sample_tick = std::chrono::steady_clock::now();
  auto next_discovery_tick = next_sample_tick;
  uint64_t flush_counter = 0;

  auto advance_tick = [](std::chrono::steady_clock::time_point & tick,
      const std::chrono::steady_clock::duration & step,
      const std::chrono::steady_clock::time_point & now) {
      do {
        tick += step;
      } while (tick <= now);
    };

  while (running_.load()) {
    const auto now = std::chrono::steady_clock::now();

    if (now >= next_discovery_tick) {
      tracked_processes = discover_ros_processes();
      advance_tick(next_discovery_tick, discovery_step, now);
    }

    if (now >= next_sample_tick) {
      const auto lag_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now - next_sample_tick).count();
      if (lag_ns > sample_step_ns) {
        const long long missed_ticks = sample_step_ns > 0 ? (lag_ns / sample_step_ns) : 0;
        RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Node process loop lag: %.3f ms behind schedule (missed %lld ticks).",
          static_cast<double>(lag_ns) / 1e6,
          missed_ticks);
      }

      const auto monotonic_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
      const auto ros_now = get_clock()->now();
      const int64_t ros_total_ns = ros_now.nanoseconds();
      const int64_t ros_sec = ros_total_ns / 1000000000LL;
      const int64_t ros_nsec = ros_total_ns % 1000000000LL;

      std::unordered_set<std::string> seen_keys;
      for (const auto & process : tracked_processes) {
        ProcessCpuTimes current_times;
        if (!read_process_cpu_times(process.pid, current_times)) {
          continue;
        }

        const std::string key = std::to_string(process.pid) + ":" +
          std::to_string(current_times.starttime_ticks);
        seen_keys.insert(key);

        double cpu_percent = 0.0;
        const auto prev_it = prev_samples.find(key);
        if (prev_it != prev_samples.end()) {
          const auto elapsed_sec =
            std::chrono::duration_cast<std::chrono::duration<double>>(now - prev_it->second.stamp).count();
          if (elapsed_sec > 1e-6) {
            const uint64_t prev_ticks =
              prev_it->second.times.utime_ticks + prev_it->second.times.stime_ticks;
            const uint64_t curr_ticks = current_times.utime_ticks + current_times.stime_ticks;
            if (curr_ticks >= prev_ticks) {
              const uint64_t delta_ticks = curr_ticks - prev_ticks;
              const double cpu_seconds = static_cast<double>(delta_ticks) / ticks_per_second;
                const double cpu_percent_all_cores = (cpu_seconds / elapsed_sec) * 100.0;
                cpu_percent = std::clamp(cpu_percent_all_cores / cpu_count, 0.0, max_cpu_percent);
            }
          }
        }

        prev_samples[key] = PrevSample{current_times, now};

        node_process_csv_file_ << monotonic_ns << ','
                               << ros_sec << ','
                               << ros_nsec << ','
                               << sanitize_csv_field(process.node_name) << ','
                               << process.pid << ','
                               << std::fixed << std::setprecision(3)
                               << cpu_percent << '\n';
      }

      for (auto it = prev_samples.begin(); it != prev_samples.end();) {
        if (seen_keys.find(it->first) == seen_keys.end()) {
          it = prev_samples.erase(it);
        } else {
          ++it;
        }
      }

      ++flush_counter;
      const uint64_t flush_interval = static_cast<uint64_t>(std::max(1.0, node_process_sample_hz_));
      if ((flush_counter % flush_interval) == 0) {
        node_process_csv_file_.flush();
      }

      advance_tick(next_sample_tick, sample_step, now);
    }

    const auto wake_up = std::min(next_sample_tick, next_discovery_tick);
    std::this_thread::sleep_until(wake_up);
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
      double gpu = 0.0;
      std::vector<double> per_core_cpu;
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        long_cpu = latest_long_cpu_percent_;
        short_cpu = latest_short_cpu_percent_;
        gpu = latest_gpu_percent_;
        per_core_cpu = latest_per_core_cpu_percent_;
      }

      if (write_long) {
        long_csv_file_ << monotonic_ns << ','
                       << ros_sec << ','
                       << ros_nsec << ','
                       << std::fixed << std::setprecision(3)
                       << long_cpu << ','
                       << gpu << '\n';
      }

      if (write_short) {
        short_csv_file_ << monotonic_ns << ','
                        << ros_sec << ','
                        << ros_nsec << ','
                        << std::fixed << std::setprecision(3)
                        << short_cpu << ','
                        << gpu << '\n';

        gpu_csv_file_ << monotonic_ns << ','
                      << ros_sec << ','
                      << ros_nsec << ','
                      << std::fixed << std::setprecision(3)
                      << gpu << '\n';

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
        gpu_csv_file_.flush();
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
  double gpu = 0.0;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    long_cpu = latest_long_cpu_percent_;
    short_cpu = latest_short_cpu_percent_;
    gpu = latest_gpu_percent_;
  }

  RCLCPP_INFO(
    get_logger(),
    "CPU usage: long(%.3fs)=%.1f%% | short(%.6fs)=%.1f%% | gpu=%.1f%%",
    rolling_window_long_sec_,
    long_cpu,
    rolling_window_short_sec_,
    short_cpu,
    gpu);
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
