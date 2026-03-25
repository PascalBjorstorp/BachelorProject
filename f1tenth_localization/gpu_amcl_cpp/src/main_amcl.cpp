#include "gpu_amcl_cpp/core/amcl_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<gpu_amcl_cpp::AmclNode>();
    
    // MultiThreadedExecutor with 4 threads
    // Allows scan_callback and odom_callback to run in parallel
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
