#ifndef F1TENTH_CONTROL_FOLLOW_THE_GAP_HPP_
#define F1TENTH_CONTROL_FOLLOW_THE_GAP_HPP_

#include "f1tenth_control/common/types.hpp"
#include "f1tenth_control/common/lidar_processor.hpp"
#include <memory>
#include <functional>

namespace f1tenth_control {

/**
 * @brief Configuration for Follow The Gap algorithm
 * 
 * Contains both generic LiDAR processing config and FTG-specific parameters.
 */
struct FTGConfig {
    // Vehicle parameters
    double wheelbase{0.324};         // Vehicle wheelbase (m) - from vesc.yaml
    double car_width{0.30};          // Vehicle width (m) for disparity extension
    
    // Speed control (reference FTG formula)
    double max_speed{6.0};           // Maximum speed (m/s)
    double min_speed{2.0};           // Minimum speed (m/s)
    double speed_full_range{9.0};    // Range at which full speed is allowed (m)
    double steer_slowdown_gain{0.7}; // How much steering reduces speed (0-1)
    
    // Steering control
    double max_steering_angle{0.4};  // Maximum steering angle (rad) ~23 degrees
    double steering_gain{0.8};       // Gain for steering toward gap (reduced for stability)
    double max_steering_delta{0.05}; // Maximum steering change per update (rad) for smoothing
    double target_angle_smoothing{0.3}; // Smoothing factor for target angle (0=no smoothing, 1=full smoothing)
    
    // Safety
    double emergency_brake_distance{0.3};  // Brake if obstacle closer than this (m)
    
    // FTG-specific LiDAR processing parameters
    double disparity_threshold{0.3}; // Threshold for disparity extension (m)
    double gap_threshold{3.0};       // Minimum range to consider as gap (m)
    double min_gap_width{0.3};       // Minimum angular width of gap (rad)
    double bubble_radius{0.2};       // Safety bubble radius around closest point (m)
    bool apply_bubble{true};         // Whether to apply safety bubble
    
    // Generic LiDAR processing config (for preprocessing only)
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
    double last_steering_{0.0};  // For steering rate limiting
    double last_target_angle_{0.0};  // For target angle smoothing (reduces oscillation)
    
    // ============================================
    // FTG-Specific LiDAR Processing (moved from LidarProcessor)
    // ============================================
    
    /**
     * @brief Apply disparity extension to ranges
     * 
     * Extends obstacles at disparity points (sudden range changes) to prevent
     * the robot from driving into narrow gaps that it cannot fit through.
     * 
     * @param scan Processed scan to modify (in-place)
     */
    void applyDisparityExtension(ProcessedScan& scan);
    
    /**
     * @brief Apply safety bubble around closest point
     * 
     * Zeros out ranges within bubble_radius of the closest point to ensure
     * the robot doesn't drive toward the nearest obstacle.
     * 
     * @param scan Processed scan to modify (in-place)
     */
    void applySafetyBubble(ProcessedScan& scan);
    
    /**
     * @brief Find all gaps in the processed scan
     * @param scan Processed scan
     * @return Vector of detected gaps
     */
    std::vector<Gap> findGaps(const ProcessedScan& scan);
    
    /**
     * @brief Find the best gap based on scoring function
     * @param gaps Vector of gaps to search
     * @return Best gap, or invalid gap if none found
     */
    Gap findBestGap(const std::vector<Gap>& gaps);
    
    // ============================================
    // Control Calculations
    // ============================================
    
    /**
     * @brief Calculate target angle toward gap (deepest point)
     */
    double calculateTargetAngle(const Gap& gap, const ProcessedScan& scan);
    
    /**
     * @brief Calculate speed based on range and steering
     */
    double calculateSpeed(const Gap& gap, double steering_angle);
    
    /**
     * @brief Score a gap for selection
     * Higher score = better gap (depth × width)
     */
    double scoreGap(const Gap& gap);
    
    /**
     * @brief Smooth steering with rate limiting
     */
    double smoothSteering(double target_steering, double last_steering);
};

}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_FOLLOW_THE_GAP_HPP_
