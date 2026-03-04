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
#include <cmath>
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
    void declareParameters();
    void loadParameters();
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter>& parameters
    );
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    // Callbacks (ConstSharedPtr for zero-copy intra-process)
    void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void enableCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);
    void vizTimerCallback();  // Throttled visualization publisher
    
    // Publishing
    void publishDriveCommand(const DriveCommand& cmd);
    void publishVisualization(const FTGOutput& output, const ProcessedScan& scan);
    
    // Helpers
    visualization_msgs::msg::Marker createGapMarker(
        const Gap& gap, 
        const ProcessedScan& scan,
        int id, 
        bool selected
    );
    visualization_msgs::msg::Marker createClosestPointMarker(
        const ProcessedScan& scan,
        size_t idx,
        int marker_id
    );
    visualization_msgs::msg::Marker createValidScanMarker(
        const ProcessedScan& scan,
        int marker_id
    );
    visualization_msgs::msg::Marker createDisparityBlockedMarker(
        const ProcessedScan& scan,
        int marker_id
    );
    visualization_msgs::msg::Marker createBubbleBlockedMarker(
        const ProcessedScan& scan,
        int marker_id
    );
    visualization_msgs::msg::Marker createDeepestPointMarker(
        const Gap& gap,
        const ProcessedScan& scan,
        int marker_id
    );
    visualization_msgs::msg::Marker createTargetPointMarker(
        const Gap& gap,
        const ProcessedScan& scan,
        int marker_id
    );
    
    // Performance tracking
    void updatePerformanceMetrics(const FTGOutput& output, const DriveCommand& cmd);
    void printPerformanceSummary();
    double calculateSteeringVariance() const;
};

}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_FTG_NODE_HPP_
