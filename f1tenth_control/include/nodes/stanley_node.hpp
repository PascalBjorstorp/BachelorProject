#ifndef F1TENTH_CONTROL_STANLEY_NODE_HPP_
#define F1TENTH_CONTROL_STANLEY_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/bool.hpp>

#include "f1tenth_control/algorithms/stanley.hpp"
#include <memory>
#include <mutex>

namespace f1tenth_control {

/**
 * @brief ROS2 node for Stanley path-following controller
 */
class StanleyNode : public rclcpp::Node {
public:
    /**
     * Inputs:
     * - options: ROS2 node options controlling runtime behavior and parameter handling.
     *
     * Purpose:
     * - Construct ROS2 wrapper for Stanley controller and initialize communications,
     *   timers, and parameter callbacks.
     *
     * Outputs:
     * - Creates active node instance ready for odometry-driven control.
     */
    explicit StanleyNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
private:
    // Parameters

    /**
     * Inputs:
     * - None.
     *
     * Purpose:
     * - Declare configurable node/controller parameters with defaults.
     *
     * Outputs:
     * - Registers parameter keys in ROS parameter interface.
     */
    void declareParameters();

    /**
     * Inputs:
     * - None.
     *
     * Purpose:
     * - Read declared parameter values and apply them to runtime config/controller.
     *
     * Outputs:
     * - Updates config_ and related node behavior fields.
     */
    void loadParameters();

    /**
     * Inputs:
     * - parameters: Proposed dynamic parameter updates.
     *
     * Purpose:
     * - Validate and apply runtime parameter changes.
     *
     * Outputs:
     * - Returns acceptance result for parameter transaction.
     */
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter>& parameters);
    
    // Callbacks

    /**
     * Inputs:
     * - msg: Odometry message.
     *
     * Purpose:
     * - Update vehicle state used by Stanley control law.
     *
     * Outputs:
     * - Mutates current_state_ and state_received_ flags.
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * Inputs:
     * - msg: Enable/disable command.
     *
     * Purpose:
     * - Gate autonomous control output from external supervision.
     *
     * Outputs:
     * - Updates enabled_ state.
     */
    void enableCallback(const std_msgs::msg::Bool::SharedPtr msg);

    /**
     * Inputs:
     * - None (timer-driven loop).
     *
     * Purpose:
     * - Execute one Stanley control step and publish command outputs.
     *
     * Outputs:
     * - Publishes drive command and updates visualization/metrics side effects.
     */
    void controlLoop();
    
    // Visualization

    /**
     * Inputs:
     * - output: Stanley controller output for current cycle.
     *
     * Purpose:
     * - Publish debug markers for controller state inspection.
     *
     * Outputs:
     * - Publishes MarkerArray visualization payload.
     */
    void publishVisualization(const StanleyOutput& output);

    /**
     * Inputs:
     * - None.
     *
     * Purpose:
     * - Publish reference trajectory as nav_msgs/Path for monitoring.
     *
     * Outputs:
     * - Publishes trajectory path message.
     */
    void publishTrajectoryPath();
    
    // Metrics

    /**
     * Inputs:
     * - output: Stanley output containing tracking errors.
     *
     * Purpose:
     * - Accumulate quality metrics for longitudinal performance evaluation.
     *
     * Outputs:
     * - Updates running CTE and lap-related metric accumulators.
     */
    void updateMetrics(const StanleyOutput& output);

    /**
     * Inputs:
     * - current_idx: Current nearest trajectory index.
     *
     * Purpose:
     * - Detect lap transitions from trajectory index progress.
     *
     * Outputs:
     * - Updates lap_count_ and lap timing state when lap completion is detected.
     */
    void checkLapCompletion(size_t current_idx);
    
    // Controller
    std::unique_ptr<Stanley> controller_;
    StanleyConfig config_;
    
    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
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
    bool publish_visualization_{true};
    double control_rate_{200.0};
};

}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_STANLEY_NODE_HPP_
