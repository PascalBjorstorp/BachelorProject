// Copyright (c) 2025 Pascal — MIT License
#include "f1tenth_lidar/hokuyo_scip_driver.hpp"

#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <sstream>
#include <stdexcept>

namespace f1tenth_lidar
{

// ═════════════════════════════════════════════════════════════════════════════
//  Construction / Destruction
// ═════════════════════════════════════════════════════════════════════════════

HokuyoScipDriver::HokuyoScipDriver(const rclcpp::NodeOptions & options)
: Node("hokuyo_scip_driver", options)
{
  // Declare parameters (defaults mirror config/hokuyo_ust10lx.yaml)
  declare_parameter("ip_address", std::string("192.168.10.10"));
  declare_parameter("ip_port", 10940);
  declare_parameter("laser_frame_id", std::string("ego_racecar/laser"));
  declare_parameter("angle_min", -2.356194);
  declare_parameter("angle_max",  2.356194);
  declare_parameter("range_min", 0.1);
  declare_parameter("range_max", 10.0);
  declare_parameter("scan_topic", std::string("/scan"));
  declare_parameter("cluster", 4);
  declare_parameter("skip", 0);

  ip_address_ = get_parameter("ip_address").as_string();
  ip_port_    = get_parameter("ip_port").as_int();
  frame_id_   = get_parameter("laser_frame_id").as_string();
  angle_min_  = get_parameter("angle_min").as_double();
  angle_max_  = get_parameter("angle_max").as_double();
  range_min_  = get_parameter("range_min").as_double();
  range_max_  = get_parameter("range_max").as_double();
  cluster_    = get_parameter("cluster").as_int();
  skip_       = get_parameter("skip").as_int();
  const auto scan_topic = get_parameter("scan_topic").as_string();

  // Publisher — RELIABLE / VOLATILE / depth 5 (matches Python driver)
  auto qos = rclcpp::QoS(5)
    .reliable()
    .durability_volatile();
  scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, qos);

  RCLCPP_INFO(get_logger(), "Hokuyo SCIP 2.0 Direct Driver (C++)");
  RCLCPP_INFO(get_logger(), "  Target: %s:%d", ip_address_.c_str(), ip_port_);
  RCLCPP_INFO(get_logger(), "  Frame:  %s", frame_id_.c_str());
  RCLCPP_INFO(get_logger(), "  Topic:  %s", scan_topic.c_str());
  RCLCPP_INFO(get_logger(), "  Cluster: %d,  Skip: %d", cluster_, skip_);

  if (connect()) {
    start_streaming();
  }
}

HokuyoScipDriver::~HokuyoScipDriver()
{
  running_ = false;
  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }
  if (sock_fd_ >= 0) {
    // Send QT to stop streaming before closing
    try { send_command("QT\n"); } catch (...) {}
    ::close(sock_fd_);
    sock_fd_ = -1;
  }
}

// ═════════════════════════════════════════════════════════════════════════════
//  TCP Connection
// ═════════════════════════════════════════════════════════════════════════════

bool HokuyoScipDriver::connect()
{
  sock_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (sock_fd_ < 0) {
    RCLCPP_ERROR(get_logger(), "socket() failed: %s", std::strerror(errno));
    return false;
  }

  // Disable Nagle for snappier command exchange
  int flag = 1;
  ::setsockopt(sock_fd_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

  // Connect with timeout
  struct sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port   = htons(static_cast<uint16_t>(ip_port_));
  if (::inet_pton(AF_INET, ip_address_.c_str(), &addr.sin_addr) != 1) {
    RCLCPP_ERROR(get_logger(), "Invalid IP: %s", ip_address_.c_str());
    ::close(sock_fd_);
    sock_fd_ = -1;
    return false;
  }

  struct timeval tv{5, 0};  // 5 s connect timeout
  ::setsockopt(sock_fd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  ::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  if (::connect(sock_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    RCLCPP_ERROR(get_logger(), "connect() failed: %s", std::strerror(errno));
    ::close(sock_fd_);
    sock_fd_ = -1;
    return false;
  }
  RCLCPP_INFO(get_logger(), "Connected to sensor");

  // Switch to SCIP 2.0 mode
  send_command("SCIP2.0\n");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  read_response();

  // Query sensor info (VV)
  send_command("VV\n");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  const auto vv = read_response();
  std::istringstream vv_stream(vv);
  std::string line;
  while (std::getline(vv_stream, line)) {
    if (line.rfind("PROD:", 0) == 0 ||
        line.rfind("FIRM:", 0) == 0 ||
        line.rfind("SERI:", 0) == 0) {
      // Trim trailing whitespace
      while (!line.empty() && (line.back() == '\r' || line.back() == '\n' || line.back() == ' '))
        line.pop_back();
      RCLCPP_INFO(get_logger(), "  %s", line.c_str());
    }
  }

  // Query sensor parameters (PP)
  send_command("PP\n");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  const auto pp = read_response();
  parse_sensor_params(pp);

  return true;
}

void HokuyoScipDriver::send_command(const std::string & cmd)
{
  if (sock_fd_ < 0) return;
  ssize_t n = ::send(sock_fd_, cmd.data(), cmd.size(), MSG_NOSIGNAL);
  if (n < 0) {
    RCLCPP_WARN(get_logger(), "send() failed: %s", std::strerror(errno));
  }
}

std::string HokuyoScipDriver::read_response(double timeout_sec)
{
  std::string buf;
  buf.reserve(4096);

  struct timeval tv{};
  tv.tv_sec  = static_cast<long>(timeout_sec);
  tv.tv_usec = static_cast<long>((timeout_sec - tv.tv_sec) * 1e6);
  ::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  char chunk[4096];
  while (true) {
    ssize_t n = ::recv(sock_fd_, chunk, sizeof(chunk), 0);
    if (n <= 0) break;
    buf.append(chunk, static_cast<size_t>(n));
    // SCIP responses end with "\n\n"
    if (buf.size() >= 2 && buf[buf.size() - 1] == '\n' && buf[buf.size() - 2] == '\n') {
      break;
    }
  }
  return buf;
}

void HokuyoScipDriver::parse_sensor_params(const std::string & response)
{
  std::istringstream ss(response);
  std::string line;
  while (std::getline(ss, line)) {
    // Remove trailing CR / whitespace
    while (!line.empty() && (line.back() == '\r' || line.back() == ' '))
      line.pop_back();

    auto value_of = [&](const std::string & prefix) -> int {
      if (line.rfind(prefix, 0) != 0) return -1;
      auto pos = line.find(':');
      if (pos == std::string::npos) return -1;
      auto semi = line.find(';', pos);
      std::string val_str = (semi != std::string::npos)
        ? line.substr(pos + 1, semi - pos - 1)
        : line.substr(pos + 1);
      try { return std::stoi(val_str); } catch (...) { return -1; }
    };

    if (line.rfind("AMIN:", 0) == 0) {
      int v = value_of("AMIN:");
      if (v >= 0) sensor_step_min_ = v;
    } else if (line.rfind("AMAX:", 0) == 0) {
      int v = value_of("AMAX:");
      if (v >= 0) sensor_step_max_ = v;
    } else if (line.rfind("ARES:", 0) == 0) {
      int v = value_of("ARES:");
      if (v > 0) angular_resolution_ = (2.0 * M_PI) / v;
    } else if (line.rfind("SCAN:", 0) == 0) {
      int v = value_of("SCAN:");
      if (v > 0) scan_time_ = 60.0 / v;
    }
  }

  apply_angle_limits();

  int num_steps = sensor_step_max_ - sensor_step_min_;
  RCLCPP_INFO(get_logger(),
    "  Sensor: steps %d–%d (%d points), scan time %.1f ms",
    sensor_step_min_, sensor_step_max_, num_steps, scan_time_ * 1000.0);
  RCLCPP_INFO(get_logger(),
    "  Requested scan: steps %d–%d, angles %.1f to %.1f deg",
    step_min_, step_max_,
    scan_angle_min_ * 180.0 / M_PI,
    scan_angle_max_ * 180.0 / M_PI);
}

void HokuyoScipDriver::apply_angle_limits()
{
  const double min_angle = std::min(angle_min_, angle_max_);
  const double max_angle = std::max(angle_min_, angle_max_);
  const double center_step =
    0.5 * (static_cast<double>(sensor_step_min_) + static_cast<double>(sensor_step_max_));

  int requested_min = static_cast<int>(std::lround(center_step + min_angle / angular_resolution_));
  int requested_max = static_cast<int>(std::lround(center_step + max_angle / angular_resolution_));

  requested_min = std::clamp(requested_min, sensor_step_min_, sensor_step_max_);
  requested_max = std::clamp(requested_max, sensor_step_min_, sensor_step_max_);

  if (requested_max <= requested_min) {
    RCLCPP_WARN(get_logger(),
      "Invalid angle limits %.3f..%.3f rad; using full sensor FOV",
      angle_min_, angle_max_);
    requested_min = sensor_step_min_;
    requested_max = sensor_step_max_;
  }

  step_min_ = requested_min;
  step_max_ = requested_max;
  scan_angle_min_ = (static_cast<double>(step_min_) - center_step) * angular_resolution_;
  scan_angle_max_ = (static_cast<double>(step_max_) - center_step) * angular_resolution_;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Continuous Streaming (MD command)
// ═════════════════════════════════════════════════════════════════════════════

void HokuyoScipDriver::start_streaming()
{
  // MD command: MDsssseeeecc0ii\n
  char cmd[32];
  std::snprintf(cmd, sizeof(cmd), "MD%04d%04d%02d%01d00\n",
                step_min_, step_max_, cluster_, skip_);

  RCLCPP_INFO(get_logger(), "Starting continuous stream: %s", cmd);
  send_command(cmd);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  const auto status = read_response();

  // Parse status (second line, first 2 chars)
  std::istringstream ss(status);
  std::string line0, line1;
  std::getline(ss, line0);
  if (std::getline(ss, line1) && line1.size() >= 2) {
    std::string code = line1.substr(0, 2);
    if (code == "00" || code == "99") {
      RCLCPP_INFO(get_logger(), "Continuous streaming started (40 Hz)");
    } else {
      RCLCPP_WARN(get_logger(), "MD status: %s", code.c_str());
    }
  }

  // Launch receive thread
  running_ = true;

  // Set recv timeout for the streaming loop (1 s)
  struct timeval tv{1, 0};
  ::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  recv_thread_ = std::thread(&HokuyoScipDriver::receive_loop, this);
}

void HokuyoScipDriver::receive_loop()
{
  std::string buf;
  buf.reserve(16384);
  char chunk[8192];

  while (running_ && rclcpp::ok()) {
    ssize_t n = ::recv(sock_fd_, chunk, sizeof(chunk), 0);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) continue;  // timeout
      if (running_) {
        RCLCPP_ERROR(get_logger(), "recv error: %s", std::strerror(errno));
      }
      break;
    }
    if (n == 0) {
      RCLCPP_ERROR(get_logger(), "Connection lost");
      break;
    }

    buf.append(chunk, static_cast<size_t>(n));

    // Process complete messages (delimited by "\n\n")
    std::string::size_type pos;
    while ((pos = buf.find("\n\n")) != std::string::npos) {
      std::string message = buf.substr(0, pos);
      buf.erase(0, pos + 2);
      const auto rx_stamp = now();
      process_md_response(message, rx_stamp);
    }
  }
}

// ═════════════════════════════════════════════════════════════════════════════
//  SCIP 2.0 Decode
// ═════════════════════════════════════════════════════════════════════════════

std::vector<int> HokuyoScipDriver::scip_decode(const std::string & data)
{
  // Each line ends with a checksum character; strip it, then concatenate
  // everything and decode groups of 3 characters (6 bits each).
  std::string clean;
  clean.reserve(data.size());

  std::string::size_type start = 0;
  while (start < data.size()) {
    auto lf = data.find('\n', start);
    std::string::size_type end = (lf != std::string::npos) ? lf : data.size();
    if (end > start + 1) {  // at least 1 real char + checksum
      clean.append(data, start, end - start - 1);  // strip trailing checksum
    }
    start = (lf != std::string::npos) ? lf + 1 : data.size();
  }

  std::vector<int> values;
  values.reserve(clean.size() / 3);

  for (std::string::size_type i = 0; i + 3 <= clean.size(); i += 3) {
    int val = 0;
    for (int j = 0; j < 3; ++j) {
      val = (val << 6) + (static_cast<int>(clean[i + j]) - 0x30);
    }
    values.push_back(val);
  }
  return values;
}

// ═════════════════════════════════════════════════════════════════════════════
//  MD Response → LaserScan
// ═════════════════════════════════════════════════════════════════════════════

void HokuyoScipDriver::process_md_response(
  const std::string & message,
  const rclcpp::Time & rx_stamp)
{
  // Split into lines
  std::vector<std::string> lines;
  {
    std::istringstream ss(message);
    std::string l;
    while (std::getline(ss, l)) {
      // Remove trailing CR
      if (!l.empty() && l.back() == '\r') l.pop_back();
      lines.push_back(std::move(l));
    }
  }
  if (lines.size() < 4) return;

  // Line 0: echo of MD command
  if (lines[0].rfind("MD", 0) != 0) return;

  // Line 1: status (first 2 chars)
  if (lines[1].size() < 2 || lines[1].substr(0, 2) != "99") return;

  // Line 2: sensor timestamp (not used directly here).
  // We anchor scan.header.stamp at the driver receive time (rx_stamp)
  // so downstream latency monitors include in-driver handling.

  // Lines 3+: distance data
  std::string data_str;
  for (size_t i = 3; i < lines.size(); ++i) {
    if (!lines[i].empty()) {
      data_str += lines[i];
      data_str += '\n';
    }
  }
  if (data_str.empty()) return;

  const auto distances_mm = scip_decode(data_str);
  if (distances_mm.empty()) return;

  // Build LaserScan
  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp    = rx_stamp;
  scan.header.frame_id = frame_id_;

  const int    num_points           = static_cast<int>(distances_mm.size());
  const double effective_resolution = angular_resolution_ * cluster_;

  scan.angle_min       = static_cast<float>(scan_angle_min_);
  scan.angle_max       = static_cast<float>(scan_angle_max_);
  scan.angle_increment = static_cast<float>(effective_resolution);
  scan.time_increment  = static_cast<float>(scan_time_ / num_points);
  scan.scan_time       = static_cast<float>(scan_time_);
  scan.range_min       = static_cast<float>(range_min_);
  scan.range_max       = static_cast<float>(range_max_);

  scan.ranges.resize(static_cast<size_t>(num_points));
  const float inf = std::numeric_limits<float>::infinity();
  const float rmin = static_cast<float>(range_min_);
  const float rmax = static_cast<float>(range_max_);

  for (int i = 0; i < num_points; ++i) {
    const int d = distances_mm[i];
    if (d <= 20) {          // invalid reading
      scan.ranges[i] = inf;
    } else {
      const float r = static_cast<float>(d) * 0.001f;  // mm → m
      scan.ranges[i] = (r < rmin || r > rmax) ? inf : r;
    }
  }

  scan_pub_->publish(scan);
}

}  // namespace f1tenth_lidar

// ═════════════════════════════════════════════════════════════════════════════
//  Main
// ═════════════════════════════════════════════════════════════════════════════
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<f1tenth_lidar::HokuyoScipDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
