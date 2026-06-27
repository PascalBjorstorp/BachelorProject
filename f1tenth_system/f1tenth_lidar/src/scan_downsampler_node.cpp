// Copyright (c) 2026 — MIT License
//
// scan_downsampler_node
// ─────────────────────
// Middleman so the racing pipeline can keep consuming a reduced-beam scan while
// a full-resolution scan is still published (and bagged) for offline analysis
// (ICP ground-truth velocity/yaw at high speed / high curvature).
//
// The Hokuyo driver clusters in the sensor request, so it can emit only ONE
// resolution per scan. Run the driver at cluster=1 (1080 beams) on `input_topic`
// (e.g. /scan_full), and this node republishes every `cluster`-th beam on
// `output_topic` (e.g. /scan, 270 beams) for AMCL / planner / MPC. Bag
// /scan_full for analysis; the pipeline is unchanged (still 270 beams).

#include <algorithm>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanDownsampler : public rclcpp::Node
{
public:
  ScanDownsampler()
  : Node("scan_downsampler")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/scan_full");
    output_topic_ = declare_parameter<std::string>("output_topic", "/scan");
    cluster_ = std::max<int>(1, declare_parameter<int>("cluster", 4));

    // BEST_EFFORT sensor QoS on both sides, matching the driver and the splitter.
    auto qos = rclcpp::SensorDataQoS();
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, qos);
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic_, qos,
      std::bind(&ScanDownsampler::onScan, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "scan_downsampler: %s -> %s (every %d-th beam)",
                input_topic_.c_str(), output_topic_.c_str(), cluster_);
  }

private:
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr in)
  {
    if (cluster_ <= 1) {
      pub_->publish(*in);  // pass-through
      return;
    }
    sensor_msgs::msg::LaserScan out;
    out.header = in->header;
    out.angle_min = in->angle_min;
    out.angle_increment = in->angle_increment * static_cast<float>(cluster_);
    out.time_increment = in->time_increment * static_cast<float>(cluster_);
    out.scan_time = in->scan_time;
    out.range_min = in->range_min;
    out.range_max = in->range_max;

    const size_t n = in->ranges.size();
    out.ranges.reserve((n + cluster_ - 1) / cluster_);
    const bool has_int = in->intensities.size() == n;
    if (has_int) out.intensities.reserve((n + cluster_ - 1) / cluster_);
    for (size_t i = 0; i < n; i += static_cast<size_t>(cluster_)) {
      out.ranges.push_back(in->ranges[i]);
      if (has_int) out.intensities.push_back(in->intensities[i]);
    }
    if (!out.ranges.empty()) {
      out.angle_max = out.angle_min +
        out.angle_increment * static_cast<float>(out.ranges.size() - 1);
    } else {
      out.angle_max = in->angle_max;
    }
    pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  int cluster_{4};
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanDownsampler>());
  rclcpp::shutdown();
  return 0;
}
