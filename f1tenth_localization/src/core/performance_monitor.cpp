// Copyright (c) 2026 Pascal — MIT License
#include "f1tenth_localization/core/performance_monitor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <sstream>

namespace f1tenth_localization
{

PerformanceMonitor::PerformanceMonitor(const rclcpp::NodeOptions & options)
: Node("performance_monitor", options)
{
  declare_parameter("output_dir", std::string("f1tenth_localization/Benchmark/Matlab/csv"));
  declare_parameter("high_rate_sample_hz", 100.0);
  declare_parameter("log_rate_hz", 50.0);
  declare_parameter("print_rate_hz", 1.0);
  declare_parameter("cpu_usage_update_hz", 20.0);
  declare_parameter("gpu_usage_update_hz", 10.0);
  // Compatibility parameters to keep existing launch files working.
  declare_parameter("sample_rate_hz", 5.0);
  declare_parameter("enable_high_rate_system_logging", true);
  declare_parameter("scan_topic", std::string("/scan"));
  declare_parameter("amcl_pose_topic", std::string("/amcl_pose"));
  declare_parameter("odom_topic", std::string("/ego_racecar/odom"));
  declare_parameter("amcl_type", std::string("gpu_amcl"));
  declare_parameter("min_particles", 500);
  declare_parameter("max_particles", 2000);
  declare_parameter("max_beams", 120);

  output_dir_ = get_parameter("output_dir").as_string();
  high_rate_sample_hz_ = get_parameter("high_rate_sample_hz").as_double();
  log_rate_hz_ = get_parameter("log_rate_hz").as_double();
  print_rate_hz_ = get_parameter("print_rate_hz").as_double();
  cpu_usage_update_hz_ = get_parameter("cpu_usage_update_hz").as_double();
  gpu_usage_update_hz_ = get_parameter("gpu_usage_update_hz").as_double();

  if (high_rate_sample_hz_ <= 0.0) {
    high_rate_sample_hz_ = 100.0;
  }
  if (log_rate_hz_ <= 0.0) {
    log_rate_hz_ = 50.0;
  }
  if (print_rate_hz_ <= 0.0) {
    print_rate_hz_ = 1.0;
  }
  if (cpu_usage_update_hz_ <= 0.0) {
    cpu_usage_update_hz_ = 20.0;
  }
  if (gpu_usage_update_hz_ <= 0.0) {
    gpu_usage_update_hz_ = 10.0;
  }
  if (cpu_usage_update_hz_ > 100.0) {
    RCLCPP_WARN(
      get_logger(),
      "cpu_usage_update_hz=%.1f is higher than typical /proc/stat tick resolution; values may appear quantized",
      cpu_usage_update_hz_);
  }
  if (gpu_usage_update_hz_ > 50.0) {
    RCLCPP_WARN(
      get_logger(),
      "gpu_usage_update_hz=%.1f is high for sysfs polling; this may add overhead",
      gpu_usage_update_hz_);
  }

  num_cores_ = std::max<size_t>(1, std::thread::hardware_concurrency());
  const auto initial_cpu_times = read_cpu_times();
  if (!initial_cpu_times.empty()) {
    // Prefer /proc/stat core count to avoid mismatches on constrained runtimes.
    num_cores_ = initial_cpu_times.size();
  }
  prev_cpu_times_.clear();
  prev_cpu_aggregate_ = CpuTimes{};

  initialize_gpu_path();
  initialize_csv();
  latest_core_usage_.assign(num_cores_, 0.0);

  running_.store(true);
  sampler_thread_ = std::thread(&PerformanceMonitor::sampling_loop, this);
  logger_thread_ = std::thread(&PerformanceMonitor::logging_loop, this);

  const auto period = std::chrono::duration<double>(1.0 / print_rate_hz_);
  status_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&PerformanceMonitor::print_status, this));

  RCLCPP_INFO(get_logger(), "Performance monitor started");
  RCLCPP_INFO(get_logger(), "  CSV output: %s", csv_path_.c_str());
  RCLCPP_INFO(get_logger(), "  Sampling loop: %.1f Hz", high_rate_sample_hz_);
  RCLCPP_INFO(get_logger(), "  CSV logging: %.1f Hz", log_rate_hz_);
  RCLCPP_INFO(get_logger(), "  CPU usage update: %.1f Hz", cpu_usage_update_hz_);
  RCLCPP_INFO(get_logger(), "  GPU usage update: %.1f Hz", gpu_usage_update_hz_);
  RCLCPP_INFO(get_logger(), "  CPU source: /proc/stat (system-wide, no sudo required)");
  RCLCPP_INFO(get_logger(), "  CSV per-core columns: cpu_core_0_percent ... cpu_core_%zu_percent", num_cores_ - 1);
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

  if (csv_file_.is_open()) {
    csv_file_.flush();
    csv_file_.close();
  }
}

void PerformanceMonitor::initialize_csv()
{
  std::filesystem::create_directories(output_dir_);

  const auto now = std::chrono::system_clock::now();
  const auto secs = std::chrono::duration_cast<std::chrono::seconds>(
    now.time_since_epoch()).count();

  std::ostringstream filename;
  filename << "system_usage_highrate_" << secs << ".csv";
  csv_path_ = (std::filesystem::path(output_dir_) / filename.str()).string();

  csv_file_.open(csv_path_, std::ios::out | std::ios::trunc);
  if (!csv_file_.is_open()) {
    throw std::runtime_error("Failed to open high-rate CSV output file");
  }

  csv_file_ << std::fixed << std::setprecision(2);

  csv_file_ << "monotonic_time_ns,timestamp_sec,timestamp_nsec,gpu_percent,cpu_aggregate_percent";
  for (size_t i = 0; i < num_cores_; ++i) {
    csv_file_ << ",cpu_core_" << i << "_percent";
  }
  csv_file_ << "\n";
  csv_file_.flush();
}

void PerformanceMonitor::initialize_gpu_path()
{
  const std::vector<std::string> candidates = {
    "/sys/devices/gpu.0/load",
    "/sys/devices/platform/gpu.0/load",
    "/sys/devices/17000000.ga10b/load",
    "/sys/devices/17000000.gv11b/load",
    "/sys/devices/platform/17000000.ga10b/load",
    "/sys/devices/platform/17000000.gv11b/load",
  };

  for (const auto & path : candidates) {
    if (std::filesystem::exists(path)) {
      gpu_load_path_ = path;
      RCLCPP_INFO(get_logger(), "GPU sysfs load path: %s", gpu_load_path_.c_str());
      return;
    }
  }

  gpu_load_path_.clear();
  RCLCPP_WARN(get_logger(), "No GPU load sysfs path found; gpu_percent will stay 0.0");
}

std::vector<PerformanceMonitor::CpuTimes> PerformanceMonitor::read_cpu_times() const
{
  std::ifstream stat_file("/proc/stat");
  std::vector<CpuTimes> times;
  if (!stat_file.is_open()) {
    return times;
  }

  std::string line;
  while (std::getline(stat_file, line)) {
    if (line.rfind("cpu", 0) != 0) {
      break;
    }
    if (line.size() < 4 || line[3] < '0' || line[3] > '9') {
      continue;
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
    times.push_back(CpuTimes{idle_all, total});
  }

  return times;
}

PerformanceMonitor::CpuTimes PerformanceMonitor::read_aggregate_cpu_times() const
{
  std::ifstream stat_file("/proc/stat");
  if (!stat_file.is_open()) {
    return CpuTimes{};
  }

  std::string line;
  while (std::getline(stat_file, line)) {
    if (line.rfind("cpu ", 0) != 0) {
      continue;
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
      return CpuTimes{};
    }

    const uint64_t idle_all = idle + iowait;
    const uint64_t total = user + nice + system + idle + iowait + irq + softirq + steal;
    return CpuTimes{idle_all, total};
  }

  return CpuTimes{};
}

double PerformanceMonitor::compute_aggregate_usage(const CpuTimes & current, double previous_usage)
{
  if (current.total == 0) {
    return previous_usage;
  }

  if (!aggregate_cpu_initialized_) {
    prev_cpu_aggregate_ = current;
    aggregate_cpu_initialized_ = true;
    return previous_usage;
  }

  if (current.total < prev_cpu_aggregate_.total || current.idle < prev_cpu_aggregate_.idle) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Detected regressing aggregate CPU counters from /proc/stat; holding previous sample for this cycle");
    prev_cpu_aggregate_ = current;
    return previous_usage;
  }

  const uint64_t total_delta = current.total - prev_cpu_aggregate_.total;
  const uint64_t idle_delta = current.idle - prev_cpu_aggregate_.idle;
  prev_cpu_aggregate_ = current;

  if (total_delta == 0) {
    // /proc/stat updates in coarse ticks; keep last valid estimate between ticks.
    return previous_usage;
  }

  const double busy = 1.0 - (static_cast<double>(idle_delta) / static_cast<double>(total_delta));
  return std::clamp(busy * 100.0, 0.0, 100.0);
}

std::vector<double> PerformanceMonitor::compute_per_core_usage(
  const std::vector<CpuTimes> & current,
  const std::vector<double> & previous_usage)
{
  if (current.empty()) {
    return std::vector<double>{};
  }

  std::vector<double> usage(current.size(), 0.0);
  const size_t copy_count = std::min(current.size(), previous_usage.size());
  std::copy_n(previous_usage.begin(), copy_count, usage.begin());

  if (!per_core_cpu_initialized_) {
    prev_cpu_times_ = current;
    per_core_cpu_initialized_ = true;
    return usage;
  }

  if (prev_cpu_times_.size() != current.size()) {
    if (!warned_cpu_core_count_change_) {
      RCLCPP_WARN(
        get_logger(),
        "Detected CPU core count change in /proc/stat (%zu -> %zu); resetting per-core deltas",
        prev_cpu_times_.size(),
        current.size());
      warned_cpu_core_count_change_ = true;
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "CPU core count mismatch in /proc/stat persists (%zu -> %zu); resetting per-core deltas for this cycle",
        prev_cpu_times_.size(),
        current.size());
    }
    prev_cpu_times_ = current;
    std::fill(usage.begin(), usage.end(), 0.0);
    return usage;
  }

  bool saw_counter_regression = false;
  for (size_t i = 0; i < current.size(); ++i) {
    if (current[i].total < prev_cpu_times_[i].total || current[i].idle < prev_cpu_times_[i].idle) {
      saw_counter_regression = true;
      continue;
    }

    const uint64_t total_delta = current[i].total - prev_cpu_times_[i].total;
    const uint64_t idle_delta = current[i].idle - prev_cpu_times_[i].idle;

    if (total_delta == 0) {
      continue;
    }

    const double busy = 1.0 - (static_cast<double>(idle_delta) / static_cast<double>(total_delta));
    usage[i] = std::clamp(busy * 100.0, 0.0, 100.0);
  }

  if (saw_counter_regression) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Detected regressing CPU counters from /proc/stat; held previous per-core samples for this cycle");
  }

  prev_cpu_times_ = current;
  return usage;
}

double PerformanceMonitor::read_gpu_percent() const
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

  // Jetson sysfs is typically 0.1% units.
  return std::clamp(raw / 10.0, 0.0, 100.0);
}

void PerformanceMonitor::sampling_loop()
{
  const auto period = std::chrono::duration<double>(1.0 / high_rate_sample_hz_);
  const auto cpu_period = std::chrono::duration<double>(1.0 / cpu_usage_update_hz_);
  const auto gpu_period = std::chrono::duration<double>(1.0 / gpu_usage_update_hz_);

  const auto min_step = std::chrono::steady_clock::duration(1);
  const auto sample_step = std::max(
    min_step,
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(period));
  const auto cpu_step = std::max(
    min_step,
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(cpu_period));
  const auto gpu_step = std::max(
    min_step,
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(gpu_period));

  auto next_tick = std::chrono::steady_clock::now();
  auto next_cpu_tick = next_tick;
  auto next_gpu_tick = next_tick;
  double cached_aggregate_usage = latest_aggregate_usage_;
  std::vector<double> cached_core_usage = latest_core_usage_;
  double cached_gpu_percent = latest_gpu_percent_;

  auto advance_tick = [](std::chrono::steady_clock::time_point & tick,
      const std::chrono::steady_clock::duration & step,
      const std::chrono::steady_clock::time_point & now) {
      do {
        tick += step;
      } while (tick <= now);
    };

  while (running_.load()) {
    const auto steady_now = std::chrono::steady_clock::now();

    if (steady_now >= next_cpu_tick) {
      const auto current_cpu = read_cpu_times();
      cached_core_usage = compute_per_core_usage(current_cpu, cached_core_usage);
      const auto current_aggregate = read_aggregate_cpu_times();
      cached_aggregate_usage = compute_aggregate_usage(current_aggregate, cached_aggregate_usage);
      if (cached_core_usage.size() > num_cores_) {
        cached_core_usage.resize(num_cores_);
      } else if (cached_core_usage.size() < num_cores_) {
        cached_core_usage.resize(num_cores_, 0.0);
      }
      advance_tick(next_cpu_tick, cpu_step, steady_now);
    }

    if (steady_now >= next_gpu_tick) {
      cached_gpu_percent = read_gpu_percent();
      advance_tick(next_gpu_tick, gpu_step, steady_now);
    }

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_aggregate_usage_ = cached_aggregate_usage;
      latest_core_usage_ = cached_core_usage;
      latest_gpu_percent_ = cached_gpu_percent;
    }

    const auto post_work = std::chrono::steady_clock::now();
    advance_tick(next_tick, sample_step, post_work);
    std::this_thread::sleep_until(next_tick);
  }
}

void PerformanceMonitor::logging_loop()
{
  const auto period = std::chrono::duration<double>(1.0 / log_rate_hz_);
  const auto min_step = std::chrono::steady_clock::duration(1);
  const auto log_step = std::max(
    min_step,
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(period));
  auto next_tick = std::chrono::steady_clock::now();

  auto advance_tick = [](std::chrono::steady_clock::time_point & tick,
      const std::chrono::steady_clock::duration & step,
      const std::chrono::steady_clock::time_point & now) {
      do {
        tick += step;
      } while (tick <= now);
    };

  while (running_.load()) {
    const auto steady_now = std::chrono::steady_clock::now();
    const auto steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      steady_now.time_since_epoch()).count();
    const auto ros_now = get_clock()->now();

    std::vector<double> core_usage;
    double aggregate_cpu = 0.0;
    double gpu = 0.0;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      aggregate_cpu = latest_aggregate_usage_;
      core_usage = latest_core_usage_;
      gpu = latest_gpu_percent_;
    }

    csv_file_ << steady_ns << ','
              << ros_now.seconds() << ','
              << ros_now.nanoseconds() % 1000000000LL << ','
              << gpu << ','
              << aggregate_cpu;

    for (const auto usage : core_usage) {
      csv_file_ << ',' << usage;
    }
    csv_file_ << '\n';

    ++flush_counter_;
    if (flush_counter_ >= 100) {
      csv_file_.flush();
      flush_counter_ = 0;
    }

    const auto post_write = std::chrono::steady_clock::now();
    if (post_write > next_tick + log_step) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "CSV logging loop is lagging behind log_rate_hz=%.1f; consider lowering log_rate_hz or sampling rates",
        log_rate_hz_);
    }

    advance_tick(next_tick, log_step, post_write);
    std::this_thread::sleep_until(next_tick);
  }
}

void PerformanceMonitor::print_status()
{
  std::vector<double> core_usage;
  double aggregate_cpu = 0.0;
  double gpu = 0.0;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    aggregate_cpu = latest_aggregate_usage_;
    core_usage = latest_core_usage_;
    gpu = latest_gpu_percent_;
  }

  if (core_usage.empty()) {
    RCLCPP_INFO(get_logger(), "CPU/GPU sample: waiting for first data");
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "System usage: CPU aggregate=%.1f%% across %zu cores | GPU=%.1f%%",
    aggregate_cpu,
    core_usage.size(),
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
