#ifndef F1TENTH_CONTROL_FTG_NODE_HPP_
#define F1TENTH_CONTROL_FTG_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/bool.hpp>

#include "f1tenth_control/algorithms/follow_the_gap.hpp"
#include <memory>
#include <mutex>
#include <deque>
#include <chrono>

namespace f1tenth_control {

/**
 * @brief ROS2 Node wrapper for Follow The Gap algorithm
 * 
 * This node:
 * - Subscribes to /scan (LiDAR) and /odom (odometry)
 * - Publishes drive commands to /drive
 * - Optionally publishes visualization markers
 * - Supports dynamic parameter reconfiguration
 */
class FTGNode : public rclcpp::Node {
public:
    /**
     * Inputs:
     * - options: ROS2 node construction options (namespace, parameters, intra-process settings).
     *
     * Purpose:
     * - Construct ROS2 wrapper around Follow The Gap controller and wire subscriptions,
     *   publishers, timers, and parameter callbacks.
     *
     * Outputs:
     * - Creates an operational node instance ready to process scan/odom streams.
     */
    explicit FTGNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // Algorithm
    std::unique_ptr<FollowTheGap> ftg_;
    FTGConfig config_;
    
    // State
    VehicleState current_state_;
    std::mutex state_mutex_;
    bool enabled_{true};
    std::string laser_frame_id_{"laser"};  // Frame ID for visualization markers
    
    // Steering smoothing
    double last_steering_{0.0};
    
    // Visualization throttling — decoupled from scan callback
    rclcpp::TimerBase::SharedPtr viz_timer_;
    FTGOutput latest_output_;        // Cached for viz timer
    ProcessedScan latest_scan_;      // Cached for viz timer
    std::mutex viz_mutex_;           // Protects latest_output_ / latest_scan_
    bool viz_data_ready_{false};     // Flag: new data available for viz
    static constexpr double VIZ_RATE_HZ = 10.0;  // Max viz publish rate
    
    // Recovery state
    int stuck_counter_{0};
    int recovery_counter_{0};
    bool in_recovery_mode_{false};
    double recovery_steer_direction_{1.0};  // 1.0 = right, -1.0 = left
    static constexpr int STUCK_THRESHOLD = 100;  // ~5 seconds at 20Hz LiDAR
    static constexpr int RECOVERY_DURATION = 80;  // ~4 seconds of recovery
    
    // Performance metrics
    struct PerformanceMetrics {
        double total_distance{0.0};
        double total_time{0.0};
        double average_speed{0.0};
        double steering_variance{0.0};
        int emergency_stops{0};
        int recovery_events{0};
        double min_obstacle_dist{100.0};
        bool crashed{false};
        double start_x{0.0};
        double start_y{0.0};
        double last_x{0.0};
        double last_y{0.0};
        int lap_count{0};
        double lap_time{0.0};
        double last_lap_distance{0.0};  // Distance when last lap was counted (debounce)
        bool was_near_start{true};      // Flag to track leaving/entering start zone
        std::deque<double> steering_history;
        std::deque<double> speed_history;
    };
    PerformanceMetrics metrics_;
    rclcpp::Time metrics_start_time_;
    bool metrics_initialized_{false};
    static constexpr double CRASH_THRESHOLD = 0.15;  // If obstacle closer than this, consider it crash
    static constexpr int METRIC_HISTORY_SIZE = 100;
    
    // ROS2 Communication
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
    
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
    
    // Parameters

    /**
     * Inputs:
     * - None.
     *
     * Purpose:
     * - Declare all node parameters and their defaults in ROS parameter server.
     *
     * Outputs:
     * - Registers parameter keys for runtime overrides and dynamic updates.
     */
    void declareParameters();

    /**
     * Inputs:
     * - None.
     *
     * Purpose:
     * - Read currently declared parameter values into internal FTGConfig.
     *
     * Outputs:
     * - Updates config_ to match ROS parameter state.
     */
    void loadParameters();

    /**
     * Inputs:
     * - parameters: Proposed runtime parameter updates.
     *
     * Purpose:
     * - Validate and apply dynamic parameter changes.
     *
     * Outputs:
     * - Returns SetParametersResult indicating acceptance/rejection.
     */
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter>& parameters
    );
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    // Callbacks (ConstSharedPtr for zero-copy intra-process)

    /**
     * Inputs:
     * - msg: Incoming LiDAR scan message.
     *
     * Purpose:
     * - Execute FTG control cycle using most recent odometry state.
     *
     * Outputs:
     * - Publishes drive command and updates visualization/performance buffers.
     */
    void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);

    /**
     * Inputs:
     * - msg: Incoming odometry message.
     *
     * Purpose:
     * - Refresh internal vehicle state used by control computations.
     *
     * Outputs:
     * - Updates current_state_ under mutex protection.
     */
    void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

    /**
     * Inputs:
     * - msg: Enable/disable command message.
     *
     * Purpose:
     * - Gate autonomous FTG command generation from external supervision.
     *
     * Outputs:
     * - Updates enabled_ flag and impacts subsequent control output behavior.
     */
    void enableCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);

    /**
     * Inputs:
     * - None (timer-driven).
     *
     * Purpose:
     * - Publish cached visualization at bounded frequency independent of control loop.
     *
     * Outputs:
     * - Emits visualization markers when new scan/output data is available.
     */
    void vizTimerCallback();  // Throttled visualization publisher
    
    // Publishing

    /**
     * Inputs:
     * - cmd: Drive command generated by FTG algorithm.
     *
     * Purpose:
     * - Convert internal command struct to ROS message and publish to actuator topic.
     *
     * Outputs:
     * - Publishes AckermannDriveStamped on drive topic.
     */
    void publishDriveCommand(const DriveCommand& cmd);

    /**
     * Inputs:
     * - output: FTG algorithm output payload.
     * - scan: Processed scan used for algorithm step.
     *
     * Purpose:
     * - Build and publish marker set for runtime introspection.
     *
     * Outputs:
     * - Publishes MarkerArray on visualization topic.
     */
    void publishVisualization(const FTGOutput& output, const ProcessedScan& scan);
    
    // Helpers

    /**
     * Inputs:
     * - gap: Gap descriptor to visualize.
     * - scan: Processed scan metadata.
     * - id: Marker ID within namespace.
     * - selected: Whether this gap is currently selected.
     *
     * Purpose:
     * - Construct visualization marker for one candidate gap segment.
     *
     * Outputs:
     * - Returns configured Marker object.
     */
    visualization_msgs::msg::Marker createGapMarker(
        const Gap& gap, 
        const ProcessedScan& scan,
        int id, 
        bool selected
    );

    /**
     * Inputs:
     * - scan: Processed scan metadata.
     * - idx: Beam index of closest point.
     * - marker_id: Marker ID.
     *
     * Purpose:
     * - Construct marker highlighting nearest obstacle sample.
     *
     * Outputs:
     * - Returns configured Marker object.
     */
    visualization_msgs::msg::Marker createClosestPointMarker(
        const ProcessedScan& scan,
        size_t idx,
        int marker_id
    );

    /**
     * Inputs:
     * - scan: Processed scan.
     * - marker_id: Marker ID.
     *
     * Purpose:
     * - Construct marker set representing valid scan points.
     *
     * Outputs:
     * - Returns configured Marker object.
     */
    visualization_msgs::msg::Marker createValidScanMarker(
        const ProcessedScan& scan,
        int marker_id
    );

    /**
     * Inputs:
     * - scan: Processed scan.
     * - marker_id: Marker ID.
     *
     * Purpose:
     * - Construct marker set for points masked by disparity safety expansion.
     *
     * Outputs:
     * - Returns configured Marker object.
     */
    visualization_msgs::msg::Marker createDisparityBlockedMarker(
        const ProcessedScan& scan,
        int marker_id
    );

    /**
     * Inputs:
     * - scan: Processed scan.
     * - marker_id: Marker ID.
     *
     * Purpose:
     * - Construct marker set for points masked by bubble safety logic.
     *
     * Outputs:
     * - Returns configured Marker object.
     */
    visualization_msgs::msg::Marker createBubbleBlockedMarker(
        const ProcessedScan& scan,
        int marker_id
    );

    /**
     * Inputs:
     * - gap: Gap descriptor.
     * - scan: Processed scan metadata.
     * - marker_id: Marker ID.
     *
     * Purpose:
     * - Construct marker for deepest beam inside chosen gap.
     *
     * Outputs:
     * - Returns configured Marker object.
     */
    visualization_msgs::msg::Marker createDeepestPointMarker(
        const Gap& gap,
        const ProcessedScan& scan,
        int marker_id
    );

    /**
     * Inputs:
     * - gap: Gap descriptor.
     * - scan: Processed scan metadata.
     * - marker_id: Marker ID.
     *
     * Purpose:
     * - Construct marker for steering target point derived from selected gap.
     *
     * Outputs:
     * - Returns configured Marker object.
     */
    visualization_msgs::msg::Marker createTargetPointMarker(
        const Gap& gap,
        const ProcessedScan& scan,
        int marker_id
    );
    
    // Performance tracking

    /**
     * Inputs:
     * - output: FTG algorithm output for current cycle.
     * - cmd: Published drive command.
     *
     * Purpose:
     * - Update long-horizon runtime metrics for validation and regression tracking.
     *
     * Outputs:
     * - Mutates metrics_ accumulators and event counters.
     */
    void updatePerformanceMetrics(const FTGOutput& output, const DriveCommand& cmd);

    /**
     * Inputs:
     * - None.
     *
     * Purpose:
     * - Emit summarized performance metrics to logs.
     *
     * Outputs:
     * - Writes metrics summary via ROS logging side effects.
     */
    void printPerformanceSummary();

    /**
     * Inputs:
     * - None.
     *
     * Purpose:
     * - Compute steering variability metric from bounded command history.
     *
     * Outputs:
     * - Returns steering variance estimate.
     */
    double calculateSteeringVariance() const;
};

}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_FTG_NODE_HPP_
