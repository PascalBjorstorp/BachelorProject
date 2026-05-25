// Copyright (c) 2026 Pascal - MIT License
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace f1tenth_localization
{

class PerformanceMonitor : public rclcpp::Node
{
public:
  explicit PerformanceMonitor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PerformanceMonitor() override;

private:
  struct CpuTimes
  {
    uint64_t idle{0};
    uint64_t total{0};
  };

  struct RollingWindow
  {
    std::deque<std::pair<std::chrono::steady_clock::time_point, double>> samples;
    std::chrono::steady_clock::duration horizon{std::chrono::milliseconds(1)};
    double sum{0.0};

    void configure(double window_sec);
    void push(const std::chrono::steady_clock::time_point & stamp, double value);
    double average() const;
  };

  struct CpuSnapshot
  {
    CpuTimes aggregate;
    bool has_aggregate{false};
    std::vector<CpuTimes> cores;
  };

  struct ProcessCpuTimes
  {
    uint64_t utime_ticks{0};
    uint64_t stime_ticks{0};
    uint64_t starttime_ticks{0};
  };

  struct RosProcess
  {
    int pid{0};
    std::string node_name;
  };

  struct CacheCounterFds
  {
    int references_fd{-1};
    int misses_fd{-1};
  };

  struct CacheCounterSample
  {
    bool valid{false};
    uint64_t references{0};
    uint64_t misses{0};
  };

  struct MemorySnapshot
  {
    double cpu_mem_total_mib{-1.0};
    double cpu_mem_available_mib{-1.0};
    double cpu_mem_used_mib{-1.0};
    double cpu_buffers_mib{-1.0};
    double cpu_cached_mib{-1.0};
    double cpu_sreclaimable_mib{-1.0};
    double cpu_shmem_mib{-1.0};
    double cpu_page_cache_mib{-1.0};
  };

  struct MemoryControllerSnapshot
  {
    bool valid{false};
    double emc_util_percent{-1.0};
    double emc_freq_mhz{-1.0};
    double emc_peak_bandwidth_mib_s{-1.0};
    double emc_estimated_bandwidth_mib_s{-1.0};
    std::string source;
  };

  enum class GpuSource
  {
    none,
    sysfs,
    nvidia_smi
  };

  void initialize_csv();
  void initialize_gpu_source();
  CpuSnapshot read_cpu_snapshot() const;
  double read_gpu_percent() const;
  double read_gpu_percent_from_sysfs() const;
  double read_gpu_percent_from_nvidia_smi() const;
  MemorySnapshot read_memory_snapshot() const;
  MemoryControllerSnapshot read_memory_controller_snapshot() const;
  MemoryControllerSnapshot read_memory_controller_from_tegrastats() const;
  std::vector<RosProcess> discover_ros_processes() const;
  bool read_process_cpu_times(int pid, ProcessCpuTimes & times) const;
  CacheCounterFds open_cache_counters(int pid);
  CacheCounterSample read_cache_counters(const CacheCounterFds & counters) const;
  static void close_cache_counters(CacheCounterFds & counters);
  void close_all_cache_counters();
  static std::vector<std::string> read_cmdline_tokens(int pid);
  static std::string extract_ros_node_name(const std::vector<std::string> & tokens);
  static std::string basename_from_path(const std::string & path);
  double compute_cpu_usage(const CpuTimes & current, double previous_usage);
  std::vector<double> compute_per_core_usage(
    const std::vector<CpuTimes> & current,
    const std::vector<double> & previous_usage);

  void sampling_loop();
  void gpu_loop();
  void process_loop();
  void logging_loop();
  void print_status();

  std::string output_dir_;
  double cpu_sample_hz_{200.0};
  double gpu_sample_hz_{10.0};
  double node_process_sample_hz_{5.0};
  double node_process_discovery_hz_{1.0};
  double csv_log_hz_{200.0};
  double long_csv_log_hz_{1.0};
  double memory_log_hz_{1.0};
  double memory_controller_log_hz_{1.0};
  double emc_peak_bandwidth_mib_s_{0.0};
  double print_hz_{1.0};
  double rolling_window_long_sec_{1.0};
  double rolling_window_short_sec_{0.005};
  GpuSource gpu_source_{GpuSource::none};
  std::string gpu_load_path_;

  std::ofstream long_csv_file_;
  std::string long_csv_path_;
  std::ofstream short_csv_file_;
  std::string short_csv_path_;
  std::ofstream per_core_csv_file_;
  std::string per_core_csv_path_;
  std::ofstream gpu_csv_file_;
  std::string gpu_csv_path_;
  std::ofstream node_process_csv_file_;
  std::string node_process_csv_path_;
  std::ofstream memory_csv_file_;
  std::string memory_csv_path_;
  std::ofstream cache_csv_file_;
  std::string cache_csv_path_;
  std::ofstream memory_controller_csv_file_;
  std::string memory_controller_csv_path_;
  std::unordered_map<std::string, CacheCounterFds> cache_counter_fds_;
  bool cache_counter_warning_printed_{false};

  std::mutex data_mutex_;
  RollingWindow long_window_;
  RollingWindow short_window_;
  CpuTimes prev_cpu_times_;
  bool cpu_initialized_{false};
  std::vector<CpuTimes> prev_core_times_;
  bool per_core_initialized_{false};
  double latest_long_cpu_percent_{0.0};
  double latest_short_cpu_percent_{0.0};
  double latest_gpu_percent_{0.0};
  std::vector<double> latest_per_core_cpu_percent_;

  std::atomic<bool> running_{false};
  std::thread sampler_thread_;
  std::thread gpu_thread_;
  std::thread process_thread_;
  std::thread logger_thread_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace f1tenth_localization
