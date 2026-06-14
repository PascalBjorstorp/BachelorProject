// Copyright (c) 2025 Pascal — MIT License
#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace f1tenth_lidar
{

/**
 * @brief High-performance SCIP 2.0 driver for the Hokuyo UST-10LX.
 *
 * Streams continuous distance data at the full 40 Hz sensor rate via the
 * MD command, bypassing urg_node's synchronous request/reply limitation.
 *
 * Parameters (loaded from config/hokuyo_ust10lx.yaml):
 *   ip_address      — sensor IP  (default "192.168.0.10")
 *   ip_port         — sensor TCP port (default 10940)
 *   laser_frame_id  — TF frame   (default "ego_racecar/laser")
 *   scan_topic      — publish topic (default "/scan")
 *   angle_min/max   — angular range (radians)
 *   range_min/max   — range filter (metres)
 *   cluster         — merge N adjacent steps (1 / 4 / …)
 *   skip            — SCIP skip interval (0 = every scan)
 */
class HokuyoScipDriver : public rclcpp::Node
{
public:
  explicit HokuyoScipDriver(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~HokuyoScipDriver() override;

private:
  // ── Connection ─────────────────────────────────────────────────────
  bool connect();
  void send_command(const std::string & cmd);
  std::string read_response(double timeout_sec = 2.0);
  void parse_sensor_params(const std::string & response);
  void apply_angle_limits();

  // ── Streaming ──────────────────────────────────────────────────────
  void start_streaming();
  void receive_loop();
  void process_md_response(const std::string & message, const rclcpp::Time & rx_stamp);

  // ── SCIP decode ────────────────────────────────────────────────────
  /// Decode SCIP 2.0 three-character encoded data into integer distances (mm).
  static std::vector<int> scip_decode(const std::string & data);

  // ── Parameters ─────────────────────────────────────────────────────
  std::string ip_address_;
  int         ip_port_{10940};
  std::string frame_id_;
  double      angle_min_{-2.356194};
  double      angle_max_{ 2.356194};
  double      range_min_{0.1};
  double      range_max_{10.0};
  int         cluster_{4};
  int         skip_{0};

  // ── Sensor specs (updated from PP response) ────────────────────────
  int    sensor_step_min_{0};
  int    sensor_step_max_{1080};
  int    step_min_{0};
  int    step_max_{1080};
  double scan_angle_min_{-2.356194};
  double scan_angle_max_{ 2.356194};
  double angular_resolution_{0.004363323};  // 0.25° in radians
  double scan_time_{0.025};                 // 25 ms = 40 Hz

  // ── Socket & threading ─────────────────────────────────────────────
  int  sock_fd_{-1};
  std::atomic<bool> running_{false};
  std::thread recv_thread_;

  // ── ROS publisher ──────────────────────────────────────────────────
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
};

}  // namespace f1tenth_lidar
