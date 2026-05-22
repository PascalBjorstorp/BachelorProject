#include "gpu_amcl_cpp/core/amcl_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<gpu_amcl_cpp::AmclNode>();
    
    // MultiThreadedExecutor with 2 threads.
    // Allows scan_callback and odom_callback callback groups to both be serviced.
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
