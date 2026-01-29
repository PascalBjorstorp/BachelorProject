#include "f1tenth_control/algorithms/follow_the_gap.hpp"
#include "f1tenth_control/common/math_utils.hpp"
#include <cmath>
#include <algorithm>

namespace f1tenth_control {

FollowTheGap::FollowTheGap(const FTGConfig& config)
    : config_(config), lidar_processor_(config.lidar_config) {}

void FollowTheGap::setConfig(const FTGConfig& config) {
    config_ = config;
    lidar_processor_.setConfig(config.lidar_config);
}

void FollowTheGap::reset() {
    // Reset any internal state if needed
    // Currently stateless, but useful for future extensions
}

FTGOutput FollowTheGap::compute(
    const std::vector<float>& ranges,
    double angle_min,
    double angle_max,
    double angle_increment,
    const Pose2D& current_pose,
    double timestamp
) {
    FTGOutput output;
    
    // Step 1: Process the LiDAR scan
    ProcessedScan scan = lidar_processor_.processScan(
        ranges, angle_min, angle_max, angle_increment
    );
    
    // Step 2: Find closest point (before any modifications)
    output.closest_point_idx = lidar_processor_.findClosestPoint(scan);
    output.closest_point_dist = scan.filtered_ranges[output.closest_point_idx];
    
    // Step 3: Check for emergency stop
    if (output.closest_point_dist < config_.emergency_brake_distance) {
        output.emergency_stop = true;
        output.command = DriveCommand(0.0, 0.0);
        return output;
    }
    
    // Step 4: Apply disparity extension for safety
    lidar_processor_.applyDisparityExtension(scan, config_.car_width);
    
    // Step 5: Apply safety bubble around closest point
    lidar_processor_.applySafetyBubble(scan);
    
    // Step 6: Find all gaps
    output.all_gaps = lidar_processor_.findGaps(scan);
    
    // Step 7: Find best gap using our scoring function
    if (output.all_gaps.empty()) {
        // No gaps found - emergency stop or reverse
        output.emergency_stop = true;
        output.command = DriveCommand(0.0, 0.0);
        return output;
    }
    
    // Custom scorer that considers gap depth, width, and straight preference
    auto scorer = [this](const Gap& gap) {
        return scoreGap(gap);
    };
    
    output.selected_gap = lidar_processor_.findBestGap(output.all_gaps, scorer);
    
    // Step 8: Calculate steering toward the gap
    double target_angle = calculateTargetAngle(output.selected_gap, scan);
    double steering = math::clamp(
        config_.steering_gain * target_angle,
        -config_.max_steering_angle,
        config_.max_steering_angle
    );
    
    // Step 9: Calculate speed based on gap and obstacles
    double speed = calculateSpeed(output.selected_gap, output.closest_point_dist);
    
    output.command = DriveCommand(speed, steering);
    
    // Step 10: Extract boundary points if in mapping mode
    if (config_.mapping_mode) {
        output.boundary_points = lidar_processor_.extractBoundaryPoints(
            scan, current_pose, timestamp
        );
    }
    
    return output;
}

double FollowTheGap::calculateTargetAngle(const Gap& gap, const ProcessedScan& scan) {
    // Option 1: Aim for the deepest point in the gap
    // This is generally safer as it aims for the most open space
    double deepest_angle = scan.angles[gap.deepest_idx];
    
    // Option 2: Aim for the center of the gap
    double center_angle = gap.centerAngle();
    
    // Blend based on configuration
    // Using deepest point is more reactive, center is smoother
    double target = deepest_angle;  // Default to deepest
    
    return target;
}

double FollowTheGap::calculateSpeed(const Gap& gap, double closest_distance) {
    double speed = config_.max_speed;
    
    // Method 1: Speed based on gap distance (how far we can see)
    double gap_based_speed = config_.speed_range_factor * gap.deepest_range;
    speed = std::min(speed, gap_based_speed);
    
    // Method 2: Slow down if close to obstacles
    if (closest_distance < config_.slowdown_distance) {
        double slowdown_factor = closest_distance / config_.slowdown_distance;
        slowdown_factor = math::clamp(slowdown_factor, 0.2, 1.0);
        speed *= slowdown_factor;
    }
    
    // Method 3: Reduce speed in narrow gaps (higher steering required)
    // Smaller gaps = slower speed for safety
    // nominal_gap_width: configurable value (~1.0 rad ≈ 57 deg) considered a "normal" gap
    double width_factor = std::min(1.0, gap.angular_width / config_.nominal_gap_width);
    speed *= (0.5 + 0.5 * width_factor);
    
    // Clamp to configured limits
    speed = math::clamp(speed, config_.min_speed, config_.max_speed);
    
    return speed;
}

double FollowTheGap::scoreGap(const Gap& gap) {
    // Base score from gap quality
    double depth_score = gap.deepest_range;
    double width_score = gap.angular_width;
    
    // Combined base score
    double score = depth_score * width_score;
    
    // Bonus for being close to straight ahead if configured
    if (config_.prefer_straight) {
        double center_angle = gap.centerAngle();
        double straight_bonus = 1.0 - (std::abs(center_angle) / constants::PI);
        score *= (1.0 + config_.straight_weight * straight_bonus);
    }
    
    return score;
}

}  // namespace f1tenth_control
