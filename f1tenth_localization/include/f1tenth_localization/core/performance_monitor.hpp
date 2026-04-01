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
  double compute_cpu_usage(const CpuTimes & current, double previous_usage);
  std::vector<double> compute_per_core_usage(
    const std::vector<CpuTimes> & current,
    const std::vector<double> & previous_usage);

  void sampling_loop();
  void gpu_loop();
  void logging_loop();
  void print_status();

  std::string output_dir_;
  double cpu_sample_hz_{200.0};
  double gpu_sample_hz_{10.0};
  double csv_log_hz_{200.0};
  double long_csv_log_hz_{1.0};
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
  std::thread logger_thread_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace f1tenth_localization
