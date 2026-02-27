#include "gpu_amcl_cpp/core/ekf_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gpu_amcl_cpp::EkfNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
