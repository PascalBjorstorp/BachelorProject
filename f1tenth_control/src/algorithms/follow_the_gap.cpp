#include "f1tenth_control/algorithms/follow_the_gap.hpp"
#include "f1tenth_control/common/math_utils.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <chrono>

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
    smoothed_target_ = 0.0;
    first_compute_ = true;
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

    // Time-based rate limiting: compute dt
    auto now = std::chrono::steady_clock::now();
    double dt = 0.025;  // default 25ms (40 Hz)
    if (!first_compute_) {
        dt = std::chrono::duration<double>(now - last_compute_time_).count();
        dt = math::clamp(dt, 0.001, 0.5);  // Guard against crazy values
    }
    last_compute_time_ = now;

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

    // Initialize blocking flags early (needed for visualization even on emergency stop)
    scan.disparity_blocked.assign(scan.filtered_ranges.size(), false);
    scan.bubble_blocked.assign(scan.filtered_ranges.size(), false);

    // Step 2: Apply wall margin (shrink all readings for safety along parallel walls)
    applyWallMargin(scan);

    // Step 3: Find closest point (before any FTG modifications)
    output.closest_point_idx = lidar_processor_.findClosestPoint(scan);
    output.closest_point_dist = scan.filtered_ranges[output.closest_point_idx];

    // Step 5: Apply disparity extension for safety (FTG-specific)
    applyDisparityExtension(scan);
    
    // Step 6: Apply safety bubble around closest point (FTG-specific)
    applySafetyBubble(scan, output.closest_point_idx);
    
    // Step 7: Find all gaps (FTG-specific)
    output.all_gaps = findGaps(scan);
    
    // Step 8: Find best gap using our scoring function
    if (output.all_gaps.empty()) {
        // No gaps found - emergency stop or reverse
        output.emergency_stop = true;
        output.command = DriveCommand(0.0, 0.0);
        output.processed_scan = scan;  // Store scan for visualization
        return output;
    }
    
    output.selected_gap = findBestGap(output.all_gaps);
    
    // Step 9: Calculate steering toward the gap (using deepest point)
    double target_angle = calculateTargetAngle(output.selected_gap, scan);
    
    // Apply EMA smoothing to target angle to prevent jitter
    if (first_compute_) {
        smoothed_target_ = target_angle;
        first_compute_ = false;
    } else {
        double alpha = math::clamp(config_.target_ema_alpha, 0.05, 1.0);
        smoothed_target_ = alpha * target_angle + (1.0 - alpha) * smoothed_target_;
    }
    
    double raw_steering = math::clamp(
        config_.steering_gain * smoothed_target_,
        -config_.max_steering,
        config_.max_steering
    );
    
    // Apply time-based steering rate limiting for smooth control
    double steering = smoothSteering(raw_steering, last_steering_, dt);
    last_steering_ = steering;
    
    // Step 10: Calculate speed based on range and steering (reference formula)
    double speed = calculateSpeed(output.selected_gap, steering);
    
    output.command = DriveCommand(speed, steering);
    
    // Step 11: Store processed scan for visualization
    output.processed_scan = scan;
    
    // Step 12: Extract boundary points if in mapping mode
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

void FollowTheGap::applyWallMargin(ProcessedScan& scan) {
    if (config_.wall_margin <= 0.0) return;  // Disabled
    
    for (size_t i = 0; i < scan.filtered_ranges.size(); ++i) {
        if (scan.valid[i] && scan.filtered_ranges[i] > config_.wall_margin) {
            scan.filtered_ranges[i] -= config_.wall_margin;
        } else if (scan.valid[i]) {
            // Range would go negative/zero - mark as invalid
            scan.filtered_ranges[i] = 0.0;
            scan.valid[i] = false;
        }
    }
}

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
            
            // Skip artificial disparities: if the closer reading was already
            // set by a previous extension, this is a cascade boundary, not a
            // real obstacle edge. Without this check, each extension's
            // boundary triggers a new extension, chaining across the scan.
            if (scan.disparity_blocked[closer_idx]) continue;
            
            // Calculate how many indices to extend based on car width
            double angle_to_extend = std::atan2(half_car, closer_range);
            int indices_to_extend = static_cast<int>(
                std::ceil(angle_to_extend / std::abs(scan.angle_increment))
            );
            
            // Extend in the appropriate direction
            if (closer_idx == i) {
                // Closer point is on the right, extend left (into the gap behind)
                for (int j = 0; j < indices_to_extend && static_cast<int>(i) - j >= 0; ++j) {
                    size_t idx = i - j;
                    if (ranges[idx] > closer_range) {
                        scan.disparity_blocked[idx] = true;  // Mark as blocked
                        ranges[idx] = closer_range;
                    }
                }
            } else {
                // Closer point is on the left, extend right (into the gap ahead)
                size_t closer_point_idx = i - 1;
                for (int j = 0; j < indices_to_extend && closer_point_idx + j < ranges.size(); ++j) {
                    size_t idx = closer_point_idx + j;
                    if (ranges[idx] > closer_range) {
                        scan.disparity_blocked[idx] = true;  // Mark as blocked
                        ranges[idx] = closer_range;
                    }
                }
            }
        }
    }
}

void FollowTheGap::applySafetyBubble(ProcessedScan& scan, size_t closest_idx) {
    if (!config_.apply_bubble || scan.filtered_ranges.empty()) return;
    
    // Use the pre-computed closest point index (already found in compute())
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
            scan.bubble_blocked[idx] = true;  // Mark as blocked by bubble
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
    double running_sum = 0.0;
    size_t running_count = 0;
    
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
            running_sum = range;
            running_count = 1;
        } else if (is_gap_point && in_gap) {
            // Continue gap (incremental avg_range)
            current_gap.min_range = std::min(current_gap.min_range, range);
            if (range > current_gap.deepest_range) {
                current_gap.deepest_range = range;
                current_gap.deepest_idx = i;
            }
            current_gap.max_range = std::max(current_gap.max_range, range);
            running_sum += range;
            ++running_count;
        } else if (!is_gap_point && in_gap) {
            // End of gap
            in_gap = false;
            current_gap.end_idx = i - 1;
            current_gap.end_angle = scan.angles[i - 1];
            current_gap.angular_width = current_gap.end_angle - current_gap.start_angle;
            
            // Use incrementally-computed average range (O(1) instead of O(n))
            current_gap.avg_range = (running_count > 0) ? running_sum / running_count : 0.0;
            running_sum = 0.0;
            running_count = 0;
            
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
        
        // Use incrementally-computed average range
        current_gap.avg_range = (running_count > 0) ? running_sum / running_count : 0.0;
        
        if (current_gap.angular_width >= config_.min_gap_width) {
            gaps.push_back(current_gap);
        }
    }
    
    // Trim gap edges: discard first/last N indices from each gap to avoid
    // driving toward extremities near obstacles. Recompute deepest point
    // within the trimmed range.
    if (config_.gap_edge_trim > 0) {
        for (auto& gap : gaps) {
            size_t trim = static_cast<size_t>(config_.gap_edge_trim);
            size_t gap_width = gap.end_idx - gap.start_idx + 1;
            
            // Only trim if gap is wide enough (need at least 1 index after trimming)
            if (gap_width > 2 * trim) {
                gap.start_idx += trim;
                gap.end_idx -= trim;
                gap.start_angle = scan.angles[gap.start_idx];
                gap.end_angle = scan.angles[gap.end_idx];
                gap.angular_width = gap.end_angle - gap.start_angle;
                
                // Recompute deepest point within trimmed range
                gap.deepest_range = 0.0;
                gap.min_range = std::numeric_limits<double>::infinity();
                gap.max_range = 0.0;
                for (size_t j = gap.start_idx; j <= gap.end_idx; ++j) {
                    double r = scan.filtered_ranges[j];
                    if (r > gap.deepest_range) {
                        gap.deepest_range = r;
                        gap.deepest_idx = j;
                    }
                    gap.min_range = std::min(gap.min_range, r);
                    gap.max_range = std::max(gap.max_range, r);
                }
            }
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
    // Target the deepest point in the gap (standard FTG behavior)
    return scan.angles[gap.deepest_idx];
}

double FollowTheGap::calculateSpeed(const Gap& gap, double steering_angle) {
    // Range factor: scale speed based on how far we can see
    double range_factor = std::min(1.0, gap.deepest_range / config_.speed_full_range);
    
    // Steering factor: slow down when turning sharply
    double abs_steer = std::abs(steering_angle);
    double steer_factor = 1.0 - config_.steer_slowdown_gain * (abs_steer / config_.max_steering);
    steer_factor = math::clamp(steer_factor, 0.3, 1.0);
    
    // Combined speed calculation
    double speed = config_.min_speed + (config_.max_speed - config_.min_speed) * range_factor * steer_factor;
    
    return math::clamp(speed, config_.min_speed, config_.max_speed);
}

double FollowTheGap::scoreGap(const Gap& gap) {
    // Base score: prefer gaps that are wide and deep
    double base_score = gap.deepest_range * gap.angular_width;
    
    // Heading bias: prefer gaps near current heading (angle 0 = straight ahead)
    // This reduces gap-flipping between similar-scoring gaps on left/right
    if (config_.heading_bias_weight > 0.0) {
        double center = gap.centerAngle();
        double heading_penalty = config_.heading_bias_weight * std::abs(center);
        base_score *= std::exp(-heading_penalty);
    }
    
    return base_score;
}

double FollowTheGap::smoothSteering(double target_steering, double last_steering, double dt) {
    double max_change = config_.max_steering_rate * dt;  // rad/s * s = rad
    double delta = target_steering - last_steering;
    
    if (std::abs(delta) > max_change) {
        double sign = (delta > 0) ? 1.0 : -1.0;
        return last_steering + sign * max_change;
    }
    
    return target_steering;
}

}  // namespace f1tenth_control
