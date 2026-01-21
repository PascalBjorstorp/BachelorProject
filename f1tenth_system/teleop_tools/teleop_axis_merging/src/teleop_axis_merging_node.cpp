#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class TeleopAxisMergingNode : public rclcpp::Node
{
public:
  TeleopAxisMergingNode()
  : Node("teleop_axis_merging_node")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("modified_joy", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&TeleopAxisMergingNode::joy_callback, this, std::placeholders::_1));
    
    // Declare and get deadzone parameter
    deadzone_ = this->declare_parameter<double>("deadzone", 0.08);
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Filter function
    auto deadzone_filter = [this](double value) {
      double abs_value = fabs(value);
      if (abs_value > 0 && abs_value < deadzone_) {
        return 0.0;
      }
      return value;
    };

    // Compute lt_value and rt_value
    double lt_value = (msg->axes[2] + 1) / 2;
    double rt_value = -(msg->axes[5] + 1) / 2;

    // Combine lt_value and rt_value to get the desired speed value
    double combined_value = rt_value + lt_value;

    // Set the combined value in the modified message's axes
    msg->axes[2] = deadzone_filter(combined_value);
    msg->axes[5] = 0.0;

    publisher_->publish(*msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_;
  double deadzone_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopAxisMergingNode>());
  rclcpp::shutdown();
  return 0;
}
