#include "f1tenth_control/algorithms/follow_the_gap.hpp"
#include "f1tenth_control/common/math_utils.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

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

    // Step 1: Process the LiDAR scan (generic preprocessing)
    ProcessedScan scan = lidar_processor_.processScan(
        ranges, angle_min, angle_max, angle_increment
    );

    // Handle empty processed scan
    if (scan.filtered_ranges.empty()) {
        output.emergency_stop = true;
        output.command = DriveCommand(0.0, 0.0);
        return output;
    }

    // Step 2: Find closest point (before any FTG modifications)
    output.closest_point_idx = lidar_processor_.findClosestPoint(scan);
    output.closest_point_dist = scan.filtered_ranges[output.closest_point_idx];

    // Step 3: Check for emergency stop
    if (output.closest_point_dist < config_.emergency_brake_distance) {
        output.emergency_stop = true;
        output.command = DriveCommand(0.0, 0.0);
        return output;
    }

    // Step 4: Apply disparity extension for safety (FTG-specific)
    applyDisparityExtension(scan);
    
    // Step 5: Apply safety bubble around closest point (FTG-specific)
    applySafetyBubble(scan);
    
    // Step 6: Find all gaps (FTG-specific)
    output.all_gaps = findGaps(scan);
    
    // Step 7: Find best gap using our scoring function
    if (output.all_gaps.empty()) {
        // No gaps found - emergency stop or reverse
        output.emergency_stop = true;
        output.command = DriveCommand(0.0, 0.0);
        return output;
    }
    
    output.selected_gap = findBestGap(output.all_gaps);
    
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

// ============================================
// FTG-Specific LiDAR Processing
// ============================================

void FollowTheGap::applyDisparityExtension(ProcessedScan& scan) {
    if (scan.filtered_ranges.size() < 2) return;
    
    std::vector<double>& ranges = scan.filtered_ranges;
    const double half_car = config_.car_width / 2.0;
    
    for (size_t i = 1; i < ranges.size(); ++i) {
        double diff = std::abs(ranges[i] - ranges[i-1]);
        
        if (diff > config_.disparity_threshold) {
            // Found a disparity - extend the closer reading
            size_t closer_idx = (ranges[i] < ranges[i-1]) ? i : i-1;
            double closer_range = ranges[closer_idx];
            
            // Calculate how many indices to extend based on car width
            double angle_to_extend = std::atan2(half_car, closer_range);
            int indices_to_extend = static_cast<int>(
                std::ceil(angle_to_extend / std::abs(scan.angle_increment))
            );
            
            // Extend in the appropriate direction
            if (closer_idx == i) {
                // Closer point is on the right, extend left
                for (int j = 0; j < indices_to_extend && static_cast<int>(i) - j >= 0; ++j) {
                    size_t idx = i - j;
                    if (ranges[idx] > closer_range) {
                        ranges[idx] = closer_range;
                    }
                }
            } else {
                // Closer point is on the left, extend right
                size_t closer_point_idx = i - 1;
                for (int j = 0; j < indices_to_extend && closer_point_idx + j < ranges.size(); ++j) {
                    size_t idx = closer_point_idx + j;
                    if (ranges[idx] > closer_range) {
                        ranges[idx] = closer_range;
                    }
                }
            }
        }
    }
}

void FollowTheGap::applySafetyBubble(ProcessedScan& scan) {
    if (!config_.apply_bubble || scan.filtered_ranges.empty()) return;
    
    // Find closest point
    size_t closest_idx = lidar_processor_.findClosestPoint(scan);
    double closest_range = scan.filtered_ranges[closest_idx];
    
    if (closest_range >= lidar_processor_.getConfig().range_max) return;
    
    // Calculate angular extent of bubble
    double bubble_angle = std::atan2(config_.bubble_radius, closest_range);
    int indices_to_zero = static_cast<int>(
        std::ceil(bubble_angle / std::abs(scan.angle_increment))
    );
    
    // Zero out ranges in bubble
    for (int i = -indices_to_zero; i <= indices_to_zero; ++i) {
        int idx = static_cast<int>(closest_idx) + i;
        if (idx >= 0 && idx < static_cast<int>(scan.filtered_ranges.size())) {
            scan.filtered_ranges[idx] = 0.0;
            scan.valid[idx] = false;
        }
    }
}

std::vector<Gap> FollowTheGap::findGaps(const ProcessedScan& scan) {
    std::vector<Gap> gaps;
    if (scan.filtered_ranges.empty()) return gaps;
    
    const auto& lidar_config = lidar_processor_.getConfig();
    bool in_gap = false;
    Gap current_gap;
    
    for (size_t i = 0; i < scan.filtered_ranges.size(); ++i) {
        double range = scan.filtered_ranges[i];
        double angle = scan.angles[i];
        
        // Check if within our angular processing range
        if (angle < lidar_config.angle_min || angle > lidar_config.angle_max) {
            continue;
        }
        
        bool is_gap_point = range >= config_.gap_threshold && scan.valid[i];
        
        if (is_gap_point && !in_gap) {
            // Start of new gap
            in_gap = true;
            current_gap = Gap();
            current_gap.start_idx = i;
            current_gap.start_angle = angle;
            current_gap.min_range = range;
            current_gap.max_range = range;
            current_gap.deepest_idx = i;
            current_gap.deepest_range = range;
        } else if (is_gap_point && in_gap) {
            // Continue gap
            current_gap.min_range = std::min(current_gap.min_range, range);
            if (range > current_gap.deepest_range) {
                current_gap.deepest_range = range;
                current_gap.deepest_idx = i;
            }
            current_gap.max_range = std::max(current_gap.max_range, range);
        } else if (!is_gap_point && in_gap) {
            // End of gap
            in_gap = false;
            current_gap.end_idx = i - 1;
            current_gap.end_angle = scan.angles[i - 1];
            current_gap.angular_width = current_gap.end_angle - current_gap.start_angle;
            
            // Calculate average range
            double sum = 0.0;
            size_t count = 0;
            for (size_t j = current_gap.start_idx; j <= current_gap.end_idx; ++j) {
                if (scan.valid[j]) {
                    sum += scan.filtered_ranges[j];
                    ++count;
                }
            }
            current_gap.avg_range = (count > 0) ? sum / count : 0.0;
            
            // Only add if gap is wide enough
            if (current_gap.angular_width >= config_.min_gap_width) {
                gaps.push_back(current_gap);
            }
        }
    }
    
    // Handle gap that extends to the end
    if (in_gap) {
        current_gap.end_idx = scan.filtered_ranges.size() - 1;
        current_gap.end_angle = scan.angles.back();
        current_gap.angular_width = current_gap.end_angle - current_gap.start_angle;
        
        double sum = 0.0;
        size_t count = 0;
        for (size_t j = current_gap.start_idx; j <= current_gap.end_idx; ++j) {
            if (scan.valid[j]) {
                sum += scan.filtered_ranges[j];
                ++count;
            }
        }
        current_gap.avg_range = (count > 0) ? sum / count : 0.0;
        
        if (current_gap.angular_width >= config_.min_gap_width) {
            gaps.push_back(current_gap);
        }
    }
    
    return gaps;
}

Gap FollowTheGap::findBestGap(const std::vector<Gap>& gaps) {
    if (gaps.empty()) {
        return Gap();  // Return invalid gap
    }
    
    double best_score = -std::numeric_limits<double>::infinity();
    const Gap* best_gap = &gaps[0];
    
    for (const auto& gap : gaps) {
        double score = scoreGap(gap);
        if (score > best_score) {
            best_score = score;
            best_gap = &gap;
        }
    }
    
    return *best_gap;
}

// ============================================
// Control Calculations
// ============================================

double FollowTheGap::calculateTargetAngle(const Gap& gap, const ProcessedScan& scan) {
    // Pure FTG: Aim for the DEEPEST point in the gap (furthest range)
    double deepest_angle = scan.angles[gap.deepest_idx];
    return deepest_angle;
}

double FollowTheGap::calculateSpeed(const Gap& gap, double steering_angle) {
    // Range factor: scale speed based on how far we can see
    double range_factor = std::min(1.0, gap.deepest_range / config_.speed_full_range);
    
    // Steering factor: slow down when turning sharply
    double abs_steer = std::abs(steering_angle);
    double steer_factor = 1.0 - config_.steer_slowdown_gain * (abs_steer / config_.max_steering_angle);
    steer_factor = math::clamp(steer_factor, 0.3, 1.0);
    
    // Combined speed calculation
    double speed = config_.min_speed + (config_.max_speed - config_.min_speed) * range_factor * steer_factor;
    
    return math::clamp(speed, config_.min_speed, config_.max_speed);
}

double FollowTheGap::scoreGap(const Gap& gap) {
    // Pure FTG scoring: prefer gaps that are wide and deep
    return gap.deepest_range * gap.angular_width;
}

double FollowTheGap::smoothSteering(double target_steering, double last_steering) {
    double delta = target_steering - last_steering;
    
    if (std::abs(delta) > config_.max_steering_delta) {
        double sign = (delta > 0) ? 1.0 : -1.0;
        return last_steering + sign * config_.max_steering_delta;
    }
    
    return target_steering;
}

}  // namespace f1tenth_control
