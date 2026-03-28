#ifndef F1TENTH_CONTROL_FTG_NODE_HPP_
#define F1TENTH_CONTROL_FTG_NODE_HPP_

/**
 * @file ftg_node.hpp
 * @brief ROS2 node wrapper for the Follow-The-Gap reactive controller.
 * @details Manages scan-driven control loop, visualization rate throttling,
 *          stuck/recovery state machine, and long-horizon performance metrics.
 *          Uses composable node architecture with zero-copy intra-process comms.
 *          Visualization is published at VIZ_RATE_HZ independent of control rate.
 * @dependencies follow_the_gap.hpp, rclcpp, sensor_msgs, nav_msgs, ackermann_msgs, visualization_msgs, std_msgs, memory, mutex, deque, chrono
 */
#include "algorithms/follow_the_gap.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/utils.h>

#include <memory>
#include <mutex>
#include <deque>
#include <chrono>
#include <algorithm>


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
     * @brief Construct a Follow The Gap node instance.
     * @param options ROS2 node options for configuration and execution behavior.
     */
    explicit FTGNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // Algorithm
    std::unique_ptr<FollowTheGap> ftg_; // Core FTG controller instance
    FTGConfig config_;                  // Active configuration loaded from parameters

    // State    
    VehicleState current_state_;            // Latest vehicle state from odometry
    std::mutex state_mutex_;                // Protects access to current_state_
    bool enabled_{true};                    // Whether autonomous control is enabled
    std::string laser_frame_id_{"laser"};   // Frame ID for visualization markers
    
    // Steering smoothing
    double last_steering_{0.0}; // Last steering angle for rate limiting and smoothing
    
    // Visualization throttling — decoupled from scan callback
    rclcpp::TimerBase::SharedPtr viz_timer_;
    FTGOutput latest_output_;                   // Cached for viz timer
    ProcessedScan latest_scan_;                 // Cached for viz timer
    std::mutex viz_mutex_;                      // Protects latest_output_ / latest_scan_
    bool viz_data_ready_{false};                // Flag: new data available for viz
    static constexpr double VIZ_RATE_HZ = 10.0; // Max viz publish rate
    
    // Recovery state
    int stuck_counter_{0};                          // Counter for how many cycles we've been "stuck" (obstacle too close)
    int recovery_counter_{0};                       // Counter for how long we've been in recovery mode
    bool in_recovery_mode_{false};                  // Whether currently executing recovery maneuver
    double recovery_steer_direction_{1.0};          // 1.0 = right, -1.0 = left
    static constexpr int STUCK_THRESHOLD = 100;     // ~5 seconds at 20Hz LiDAR
    static constexpr int RECOVERY_DURATION = 80;    // ~4 seconds of recovery
    
    /**
     * @brief Performance metrics tracking structure.
     */
    struct PerformanceMetrics {
        double total_distance{0.0};         // Total distance traveled, accumulated from odometry
        double total_time{0.0};             // Total elapsed time since start
        double average_speed{0.0};          // Average speed computed from distance/time
        double steering_variance{0.0};      // Variance of steering angle, computed from history
        int emergency_stops{0};             // Count of how many times emergency stop was triggered
        int recovery_events{0};             // Count of how many recovery maneuvers were executed
        double min_obstacle_dist{100.0};    // Minimum distance to obstacle observed during run    
        bool crashed{false};                // Whether a crash condition was detected (obstacle too close)
        double start_x{0.0};                // X coordinate of start position
        double start_y{0.0};                // Y coordinate of start position
        double last_x{0.0};                 // X coordinate of last known position
        double last_y{0.0};                 // Y coordinate of last known position
        int lap_count{0};                   // Count of completed laps
        double lap_time{0.0};               // Time taken for the current lap
        double last_lap_distance{0.0};      // Distance when last lap was counted (debounce)
        bool was_near_start{true};          // Flag to track leaving/entering start zone
        std::deque<double> steering_history;// History of recent steering angles for variance calculation
        std::deque<double> speed_history;   // History of recent speeds for variance calculation
    };

    PerformanceMetrics metrics_;                    // Instance of performance metrics struct
    rclcpp::Time metrics_start_time_;               // Time when metrics tracking started
    bool metrics_initialized_{false};               // Whether initial metrics state has been set
    static constexpr double CRASH_THRESHOLD = 0.15; // If obstacle closer than this, consider it crash
    static constexpr int METRIC_HISTORY_SIZE = 100; // Number of recent samples to keep for variance calculations
    
    // ROS2 Communication
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_; // Subscription for LiDAR scans
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;     // Subscription for odometry
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;       // Subscription for enable/disable commands
    
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;    // Publisher for drive commands
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;            // Publisher for visualization markers
    
    // Parameters

    /**
     * @brief Declare ROS parameters for FTGNode.
     * This function registers all parameters with the ROS parameter server, including defaults.
     * Parameters include vehicle dimensions, control gains, speed limits, and visualization options.
     */
    void declareParameters();

    /**
     * @brief Load parameters from ROS parameter server into config struct.
     * This function reads the current values of all relevant parameters and updates the internal config_ struct.
     * It should be called at startup and whenever parameters are dynamically updated.
     */
    void loadParameters();

    /**
     * @brief Callback for dynamic parameter updates.
     * This function is called whenever parameters are changed at runtime. It should validate and apply new parameter values.
     * @param parameters Vector of parameters that were updated.
     * @return SetParametersResult indicating success or failure of parameter update.
     */
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter>& parameters
    );

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;   // Handle for parameter callback registration
    
    // Callbacks (ConstSharedPtr for zero-copy intra-process)

    /**
     * @brief Callback for incoming LiDAR scans.
     * This function processes incoming LaserScan messages, runs the FTG algorithm, and publishes drive
     * @param msg Incoming LaserScan message containing range and angle data.
     * @return None
     */
    void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);

    /**
     * @brief Callback for incoming odometry messages.
     * This function updates the current vehicle state based on Odometry messages, which may be used for performance metrics and recovery logic.
     * @param msg Incoming Odometry message containing pose and velocity information.
     * @return None
     */
    void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

    /**
     * @brief Callback for enable/disable commands.
     * This function enables or disables autonomous control based on incoming Bool messages. When disabled, it should stop the vehicle safely.
     * @param msg Incoming Bool message where true = enable control, false = disable control.
     * @return None
     */
    void enableCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);

    /**
     * @brief Timer callback for visualization publishing.
     * @return None
     */
    void vizTimerCallback();  // Throttled visualization publisher
    
    // Publishing

    /**
     * @brief Publish drive command to /drive topic.
     * This function converts a DriveCommand struct into an AckermannDriveStamped message and publishes it.
     * @param cmd DriveCommand containing target speed and steering angle.
     * @return None
     */
    void publishDriveCommand(const DriveCommand& cmd);

    /**
     * @brief Publish visualization markers to RViz.
     * This function constructs a MarkerArray message based on the latest FTG output and processed scan, and publishes it for visualization in RViz.
     * @param output Latest FTGOutput containing command and gap information.
     * @param scan Latest ProcessedScan containing scan metadata and validity masks.
     * @return None
     */
    void publishVisualization(const FTGOutput& output, const ProcessedScan& scan);
    
    // Helpers

    /**
     * @brief Create a visualization marker for a detected gap.
     * This function constructs a Marker message that visually represents a detected gap in the LiDAR scan, with different styling if it's the selected gap.
     * @param gap Gap descriptor containing indices and angles of the gap.
     * @param scan ProcessedScan containing metadata for angle/range calculations.
     * @param id Unique ID for the marker.
     * @param selected Whether this gap is the currently selected one for navigation (affects styling).
     * @return Configured Marker message representing the gap.
     */
    visualization_msgs::msg::Marker createGapMarker(
        const Gap& gap, 
        const ProcessedScan& scan,
        int id, 
        bool selected
    );

    /**
     * @brief Create a visualization marker for the closest detected point.
     * This function constructs a Marker message that represents the closest valid point detected in the LiDAR scan.
     * @param scan ProcessedScan containing range and angle data.
     * @param idx Index of the closest point in the scan.
     * @param marker_id Unique ID for the marker.
     * @return Configured Marker message representing the closest point.
     */
    visualization_msgs::msg::Marker createClosestPointMarker(
        const ProcessedScan& scan,
        size_t idx,
        int marker_id
    );

    /**
     * @brief Create a visualization marker for valid scan points.
     * This function constructs a Marker message that represents all valid points in the LiDAR scan.
     * @param scan ProcessedScan containing range and angle data.
     * @param marker_id Unique ID for the marker.
     * @return Configured Marker message representing the valid scan points.
     */
    visualization_msgs::msg::Marker createValidScanMarker(
        const ProcessedScan& scan,
        int marker_id
    );

    /**
     * @brief Create a visualization marker for disparity-blocked scan points.
     * This function constructs a Marker message that represents points in the LiDAR scan that are blocked by disparity safety logic.
     * @param scan ProcessedScan containing range and angle data.
     * @param marker_id Unique ID for the marker.
     * @return Configured Marker message representing the disparity-blocked points.
     */
    visualization_msgs::msg::Marker createDisparityBlockedMarker(
        const ProcessedScan& scan,
        int marker_id
    );

    /**
     * @brief Create a visualization marker for bubble-blocked scan points.
     * This function constructs a Marker message that represents points in the LiDAR scan that are blocked by bubble safety logic.
     * @param scan ProcessedScan containing range and angle data.
     * @param marker_id Unique ID for the marker.
     * @return Configured Marker message representing the bubble-blocked points.
     */
    visualization_msgs::msg::Marker createBubbleBlockedMarker(
        const ProcessedScan& scan,
        int marker_id
    );

    /**
     * @brief Create a visualization marker for the deepest point in the selected gap.
     * This function constructs a Marker message that represents the deepest point within the currently selected gap, which is often the steering target.
     * @param gap Gap descriptor containing indices and angles of the gap.
     * @param scan ProcessedScan containing metadata for angle/range calculations.
     * @param marker_id Unique ID for the marker.
     * @return Configured Marker message representing the deepest point in the selected gap.
     */
    visualization_msgs::msg::Marker createDeepestPointMarker(
        const Gap& gap,
        const ProcessedScan& scan,
        int marker_id
    );

    /**
     * @brief Create a visualization marker for the target point (steering target).
     * This function constructs a Marker message that represents the target point that the vehicle is steering towards, which may be the deepest point in the selected gap or another computed target.
     * @param gap Gap descriptor containing indices and angles of the gap.
     * @param scan ProcessedScan containing metadata for angle/range calculations.
     * @param marker_id Unique ID for the marker.
     * @return Configured Marker message representing the steering target point.
     */
    visualization_msgs::msg::Marker createTargetPointMarker(
        const Gap& gap,
        const ProcessedScan& scan,
        int marker_id
    );
    
    // Performance tracking

    /**
     * @brief Update performance metrics based on latest output and command.
     * This function updates the internal PerformanceMetrics struct with new data from the latest FTG output and the drive command that was issued, allowing for long-term performance tracking.
     * @param output Latest FTGOutput containing command and gap information.
     * @param cmd DriveCommand that was published based on the output.
     * @return None
     */
    void updatePerformanceMetrics(const FTGOutput& output, const DriveCommand& cmd);

    /**
     * @brief Print a summary of performance metrics to the console.
     * This function computes and prints a summary of the tracked performance metrics, such as total distance, average speed, steering variance, and any crash events.
     * @return None
     */
    void printPerformanceSummary();

    /**
     * @brief Calculate the variance of recent steering angles.
     * This function computes the variance of the steering angles stored in the steering_history deque, which can be used as a measure of control stability.
     * @return Variance of recent steering angles.
     */
    double calculateSteeringVariance() const;
};

}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_FTG_NODE_HPP_
