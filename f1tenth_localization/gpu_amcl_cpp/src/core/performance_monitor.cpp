// Copyright (c) 2026 Pascal — MIT License
#include "gpu_amcl_cpp/core/performance_monitor.hpp"

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
  declare_parameter("high_rate_sample_hz", 1000.0);
  declare_parameter("print_rate_hz", 1.0);
  declare_parameter("cpu_usage_update_hz", 50.0);
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
  print_rate_hz_ = get_parameter("print_rate_hz").as_double();
  cpu_usage_update_hz_ = get_parameter("cpu_usage_update_hz").as_double();

  if (high_rate_sample_hz_ <= 0.0) {
    high_rate_sample_hz_ = 1000.0;
  }
  if (print_rate_hz_ <= 0.0) {
    print_rate_hz_ = 1.0;
  }
  if (cpu_usage_update_hz_ <= 0.0) {
    cpu_usage_update_hz_ = 50.0;
  }

  num_cores_ = std::max<size_t>(1, std::thread::hardware_concurrency());
  initialize_gpu_path();
  initialize_csv();

  prev_cpu_times_ = read_cpu_times();
  latest_core_usage_.assign(num_cores_, 0.0);

  running_.store(true);
  sampler_thread_ = std::thread(&PerformanceMonitor::sampling_loop, this);

  const auto period = std::chrono::duration<double>(1.0 / print_rate_hz_);
  status_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&PerformanceMonitor::print_status, this));

  RCLCPP_INFO(get_logger(), "Performance monitor started");
  RCLCPP_INFO(get_logger(), "  CSV output: %s", csv_path_.c_str());
  RCLCPP_INFO(get_logger(), "  High-rate sampling: %.1f Hz", high_rate_sample_hz_);
  RCLCPP_INFO(get_logger(), "  CPU usage update: %.1f Hz", cpu_usage_update_hz_);
}

PerformanceMonitor::~PerformanceMonitor()
{
  running_.store(false);
  if (sampler_thread_.joinable()) {
    sampler_thread_.join();
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

  csv_file_ << "monotonic_time_ns,timestamp_sec,timestamp_nsec,gpu_percent";
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
    uint64_t guest = 0;
    uint64_t guest_nice = 0;

    iss >> cpu_label >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal >> guest >> guest_nice;

    const uint64_t idle_all = idle + iowait;
    const uint64_t total = user + nice + system + idle + iowait + irq + softirq + steal + guest + guest_nice;
    times.push_back(CpuTimes{idle_all, total});
  }

  return times;
}

std::vector<double> PerformanceMonitor::compute_per_core_usage(const std::vector<CpuTimes> & current)
{
  std::vector<double> usage;
  if (current.empty()) {
    return usage;
  }

  if (prev_cpu_times_.size() != current.size()) {
    prev_cpu_times_ = current;
    usage.assign(current.size(), 0.0);
    return usage;
  }

  usage.resize(current.size(), 0.0);
  for (size_t i = 0; i < current.size(); ++i) {
    const uint64_t total_delta = current[i].total - prev_cpu_times_[i].total;
    const uint64_t idle_delta = current[i].idle - prev_cpu_times_[i].idle;

    if (total_delta == 0) {
      usage[i] = 0.0;
      continue;
    }

    const double busy = 1.0 - (static_cast<double>(idle_delta) / static_cast<double>(total_delta));
    usage[i] = std::clamp(busy * 100.0, 0.0, 100.0);
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
  auto next_tick = std::chrono::steady_clock::now();
  auto next_cpu_tick = next_tick;
  std::vector<double> cached_core_usage = latest_core_usage_;

  while (running_.load()) {
    const auto steady_now = std::chrono::steady_clock::now();
    const auto steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      steady_now.time_since_epoch()).count();
    const auto ros_now = get_clock()->now();

    if (steady_now >= next_cpu_tick) {
      const auto current_cpu = read_cpu_times();
      cached_core_usage = compute_per_core_usage(current_cpu);
      if (cached_core_usage.size() > num_cores_) {
        cached_core_usage.resize(num_cores_);
      } else if (cached_core_usage.size() < num_cores_) {
        cached_core_usage.resize(num_cores_, 0.0);
      }
      next_cpu_tick = steady_now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(cpu_period);
    }

    const auto & core_usage = cached_core_usage;
    const double gpu_percent = read_gpu_percent();

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_core_usage_ = core_usage;
      latest_gpu_percent_ = gpu_percent;
    }

    csv_file_ << steady_ns << ','
              << ros_now.seconds() << ','
              << ros_now.nanoseconds() % 1000000000LL << ','
              << std::fixed << std::setprecision(2)
              << gpu_percent;

    for (const auto usage : core_usage) {
      csv_file_ << ',' << usage;
    }
    csv_file_ << '\n';

    ++flush_counter_;
    if (flush_counter_ >= 100) {
      csv_file_.flush();
      flush_counter_ = 0;
    }

    next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    const auto now = std::chrono::steady_clock::now();
    if (next_tick > now) {
      std::this_thread::sleep_until(next_tick);
    } else {
      next_tick = now;
    }
  }
}

void PerformanceMonitor::print_status()
{
  std::vector<double> core_usage;
  double gpu = 0.0;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    core_usage = latest_core_usage_;
    gpu = latest_gpu_percent_;
  }

  if (core_usage.empty()) {
    RCLCPP_INFO(get_logger(), "CPU/GPU sample: waiting for first data");
    return;
  }

  double sum = 0.0;
  for (const auto value : core_usage) {
    sum += value;
  }
  const double avg_cpu = sum / static_cast<double>(core_usage.size());

  RCLCPP_INFO(
    get_logger(),
    "System usage: CPU avg=%.1f%% across %zu cores | GPU=%.1f%%",
    avg_cpu,
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
