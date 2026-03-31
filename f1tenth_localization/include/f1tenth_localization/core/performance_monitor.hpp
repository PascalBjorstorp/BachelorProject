// Copyright (c) 2026 Pascal — MIT License
#pragma once

#include <atomic>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
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

  void initialize_csv();
  void initialize_gpu_path();

  CpuTimes read_aggregate_cpu_times() const;
  std::vector<CpuTimes> read_cpu_times() const;
  double compute_aggregate_usage(const CpuTimes & current, double previous_usage);
  std::vector<double> compute_per_core_usage(
    const std::vector<CpuTimes> & current,
    const std::vector<double> & previous_usage);
  double read_gpu_percent() const;

  void sampling_loop();
  void logging_loop();
  void print_status();

  std::string output_dir_;
  double high_rate_sample_hz_{100.0};
  double print_rate_hz_{1.0};
  double cpu_usage_update_hz_{20.0};
  double gpu_usage_update_hz_{10.0};
  double log_rate_hz_{50.0};

  size_t num_cores_{1};
  std::string gpu_load_path_;

  std::ofstream csv_file_;
  std::string csv_path_;
  size_t flush_counter_{0};

  std::mutex data_mutex_;
  bool aggregate_cpu_initialized_{false};
  bool per_core_cpu_initialized_{false};
  CpuTimes prev_cpu_aggregate_;
  std::vector<CpuTimes> prev_cpu_times_;
  double latest_aggregate_usage_{0.0};
  std::vector<double> latest_core_usage_;
  double latest_gpu_percent_{0.0};
  bool warned_cpu_core_count_change_{false};

  std::atomic<bool> running_{false};
  std::thread sampler_thread_;
  std::thread logger_thread_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace f1tenth_localization
