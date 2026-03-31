#ifndef F1TENTH_CONTROL_STANLEY_NODE_HPP_
#define F1TENTH_CONTROL_STANLEY_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/utils.h>

#include "algorithms/stanley.hpp"
#include <memory>
#include <mutex>

namespace f1tenth_control {

/**
 * @brief ROS2 node for Stanley path-following controller
 */
class StanleyNode : public rclcpp::Node {
public:
    /**
    * @brief Construct a Stanley controller ROS2 node.
    * @param options ROS2 node options for component configuration.
     */
    explicit StanleyNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
private:
    // Parameters

    /**
     * @brief Declare node and controller parameters with default values.
     */
    void declareParameters();

    /**
     * @brief Load declared parameters into runtime configuration.
     */
    void loadParameters();

    /**
     * @brief Validate and apply runtime parameter updates.
     * @param parameters Proposed parameter updates.
     * @return Result that accepts or rejects the update transaction.
     */
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter>& parameters);
    
    // Callbacks

    /**
     * @brief Process odometry updates used by the Stanley controller.
     * @param msg Incoming odometry message.
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Enable or disable autonomous control output.
     * @param msg Incoming enable flag.
     */
    void enableCallback(const std_msgs::msg::Bool::SharedPtr msg);

    /**
     * @brief Execute one timer-driven control step and publish a drive command.
     */
    void controlLoop();
    
    // Metrics

    /**
     * @brief Update controller performance metrics.
     * @param output Latest Stanley controller output.
     */
    void updateMetrics(const StanleyOutput& output);

    /**
     * @brief Detect and count lap transitions from trajectory index progress.
     * @param current_idx Current nearest trajectory index.
     */
    void checkLapCompletion(size_t current_idx);
    
    // Controller
    std::unique_ptr<Stanley> controller_;
    StanleyConfig config_;
    
    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // Parameter callback handle
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    // State
    VehicleState current_state_;
    std::mutex state_mutex_;
    bool state_received_{false};
    bool enabled_{true};
    
    // Metrics
    double total_cte_{0.0};
    double max_cte_{0.0};
    size_t cte_count_{0};
    size_t last_lap_idx_{0};
    double lap_start_time_{0.0};
    int lap_count_{0};
    bool crossed_start_{false};
    
    // Config
    std::string trajectory_file_;
    double control_rate_{200.0};
};

}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_STANLEY_NODE_HPP_
