#ifndef F1TENTH_CONTROL_FOLLOW_THE_GAP_HPP_
#define F1TENTH_CONTROL_FOLLOW_THE_GAP_HPP_

#include "f1tenth_control/common/types.hpp"
#include "f1tenth_control/common/lidar_processor.hpp"
#include <memory>

namespace f1tenth_control {

/**
 * @brief Configuration for Follow The Gap algorithm
 */
struct FTGConfig {
    // Vehicle parameters
    double wheelbase{0.324};         // Vehicle wheelbase (m) - from vesc.yaml
    double car_width{0.30};          // Vehicle width (m) for disparity extension
    
    // Speed control
    double max_speed{4.0};           // Maximum speed (m/s)
    double min_speed{1.0};           // Minimum speed (m/s)
    double speed_range_factor{0.5};  // Speed = factor * gap_distance (capped)
    double nominal_gap_width{1.0};   // Nominal gap width for speed scaling (rad, ~57 deg)
    
    // Steering control
    double max_steering_angle{0.4};  // Maximum steering angle (rad) ~23 degrees
    double steering_gain{1.0};       // Gain for steering toward gap center
    
    // Gap selection
    bool prefer_straight{true};      // Prefer gaps closer to straight ahead
    double straight_weight{0.3};     // Weight for straight-ahead preference (0-1)
    
    // Safety
    double emergency_brake_distance{0.3};  // Brake if obstacle closer than this (m)
    double slowdown_distance{1.5};         // Start slowing down at this distance (m)
    
    // LiDAR processing config
    LidarProcessorConfig lidar_config;
    
    // Mapping mode
    bool mapping_mode{false};        // Enable boundary point extraction for mapping
    double mapping_sample_rate{10.0}; // Hz for boundary point sampling
};

/**
 * @brief FTG algorithm output
 */
struct FTGOutput {
    DriveCommand command;            // Drive command
    Gap selected_gap;                // The gap that was selected
    size_t closest_point_idx{0};     // Index of closest point
    double closest_point_dist{0.0};  // Distance to closest point
    bool emergency_stop{false};      // Whether emergency stop is active
    std::vector<Gap> all_gaps;       // All detected gaps (for visualization)
    std::vector<BoundaryPoint> boundary_points; // For mapping mode
};

/**
 * @brief Follow The Gap (FTG) Algorithm
 * 
 * A reactive algorithm that:
 * 1. Finds the closest obstacle
 * 2. Creates a "safety bubble" around it
 * 3. Finds the largest gap in the LiDAR scan
 * 4. Steers toward the deepest point in the gap
 * 
 * This implementation includes:
 * - Disparity extension for safety at narrow passages
 * - Configurable speed based on gap distance
 * - Emergency braking for close obstacles
 * - Mapping mode for track boundary extraction
 */
class FollowTheGap {
public:
    explicit FollowTheGap(const FTGConfig& config = FTGConfig());
    
    /**
     * @brief Update algorithm configuration
     */
    void setConfig(const FTGConfig& config);
    const FTGConfig& getConfig() const { return config_; }
    
    /**
     * @brief Compute drive command from LiDAR scan
     * @param ranges Raw LiDAR ranges
     * @param angle_min Minimum scan angle (rad)
     * @param angle_max Maximum scan angle (rad)
     * @param angle_increment Angular increment (rad)
     * @param current_pose Current robot pose (for mapping mode)
     * @param timestamp Current timestamp (for mapping mode)
     * @return FTG output including drive command and debug info
     */
    FTGOutput compute(
        const std::vector<float>& ranges,
        double angle_min,
        double angle_max,
        double angle_increment,
        const Pose2D& current_pose = Pose2D(),
        double timestamp = 0.0
    );
    
    /**
     * @brief Get the LiDAR processor (for external use/visualization)
     */
    LidarProcessor& getLidarProcessor() { return lidar_processor_; }
    const LidarProcessor& getLidarProcessor() const { return lidar_processor_; }
    
    /**
     * @brief Reset internal state (if any)
     */
    void reset();

private:
    FTGConfig config_;
    LidarProcessor lidar_processor_;
    
    /**
     * @brief Calculate target angle toward gap
     */
    double calculateTargetAngle(const Gap& gap, const ProcessedScan& scan);
    
    /**
     * @brief Calculate speed based on gap and obstacles
     */
    double calculateSpeed(const Gap& gap, double closest_distance);
    
    /**
     * @brief Score a gap for selection
     * Higher score = better gap
     */
    double scoreGap(const Gap& gap);
};

}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_FOLLOW_THE_GAP_HPP_
