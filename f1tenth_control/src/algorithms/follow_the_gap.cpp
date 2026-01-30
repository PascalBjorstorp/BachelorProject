#include "f1tenth_control/algorithms/follow_the_gap.hpp"
#include "f1tenth_control/common/math_utils.hpp"
#include <cmath>
#include <algorithm>

namespace f1tenth_control {

FollowTheGap::FollowTheGap(const FTGConfig& config)
     : config_(config), lidar_processor_(config.lidar_config), last_steering_(0.0) {}

void FollowTheGap::setConfig(const FTGConfig& config) {
    config_ = config;
    lidar_processor_.setConfig(config.lidar_config);
}

void FollowTheGap::reset() {
    // Reset internal state
    last_steering_ = 0.0;
    last_target_angle_ = 0.0;
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

    // Handle empty scan
    if (ranges.empty()) {
        output.emergency_stop = true;
        output.command = DriveCommand(0.0, 0.0);
        return output;
    }

    // Step 1: Process the LiDAR scan
    ProcessedScan scan = lidar_processor_.processScan(
        ranges, angle_min, angle_max, angle_increment
    );

    // Handle empty processed scan
    if (scan.filtered_ranges.empty()) {
        output.emergency_stop = true;
        output.command = DriveCommand(0.0, 0.0);
        return output;
    }

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
    
    // Step 8: Calculate steering toward the gap (using deepest point)
    double raw_target_angle = calculateTargetAngle(output.selected_gap, scan);
    
    // Apply target angle smoothing (exponential moving average)
    // This reduces oscillation caused by the deepest point jumping around
    double target_angle = last_target_angle_ + 
        (1.0 - config_.target_angle_smoothing) * (raw_target_angle - last_target_angle_);
    last_target_angle_ = target_angle;
    
    double raw_steering = math::clamp(
        config_.steering_gain * target_angle,
        -config_.max_steering_angle,
        config_.max_steering_angle
    );
    
    // Apply steering rate limiting for smooth control
    double steering = smoothSteering(raw_steering, last_steering_);
    last_steering_ = steering;
    
    // Step 9: Calculate speed based on range and steering (reference formula)
    double speed = calculateSpeed(output.selected_gap, steering);
    
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
    // Pure FTG: Aim for the DEEPEST point in the gap (furthest range)
    // This is the core principle - drive toward where we can see the furthest
    double deepest_angle = scan.angles[gap.deepest_idx];
    
    return deepest_angle;
}

double FollowTheGap::calculateSpeed(const Gap& gap, double steering_angle) {
    // Reference FTG speed formula:
    // 1. Range factor: scale speed based on how far we can see
    double range_factor = std::min(1.0, gap.deepest_range / config_.speed_full_range);
    
    // 2. Steering factor: slow down when turning sharply
    double abs_steer = std::abs(steering_angle);
    double steer_factor = 1.0 - config_.steer_slowdown_gain * (abs_steer / config_.max_steering_angle);
    steer_factor = math::clamp(steer_factor, 0.3, 1.0);  // Never reduce below 30%
    
    // Combined speed calculation
    double speed = config_.min_speed + (config_.max_speed - config_.min_speed) * range_factor * steer_factor;
    
    // Clamp to configured limits
    speed = math::clamp(speed, config_.min_speed, config_.max_speed);
    
    return speed;
}

double FollowTheGap::scoreGap(const Gap& gap) {
    // Pure FTG scoring: prefer gaps that are wide and deep
    double depth_score = gap.deepest_range;
    double width_score = gap.angular_width;
    
    // Combined score: depth × width gives us the "best" gap
    return depth_score * width_score;
}

double FollowTheGap::smoothSteering(double target_steering, double last_steering) {
    // Rate-limit steering changes for smooth control
    // This prevents jerky movements and improves stability
    double delta = target_steering - last_steering;
    
    if (std::abs(delta) > config_.max_steering_delta) {
        // Limit the change rate
        double sign = (delta > 0) ? 1.0 : -1.0;
        return last_steering + sign * config_.max_steering_delta;
    }
    
    return target_steering;
}

}  // namespace f1tenth_control
