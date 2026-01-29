#include "f1tenth_control/common/lidar_processor.hpp"
#include "f1tenth_control/common/math_utils.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace f1tenth_control {

LidarProcessor::LidarProcessor(const LidarProcessorConfig& config)
    : config_(config) {}

void LidarProcessor::setConfig(const LidarProcessorConfig& config) {
    config_ = config;
}

ProcessedScan LidarProcessor::processScan(
    const std::vector<float>& ranges,
    double angle_min,
    double angle_max,
    double angle_increment
) {
    ProcessedScan scan;
    scan.angle_min = angle_min;
    scan.angle_max = angle_max;
    scan.angle_increment = angle_increment;
    scan.range_min = config_.range_min;
    scan.range_max = config_.range_max;
    
    // Convert to double and compute angles
    scan.ranges.reserve(ranges.size());
    scan.angles.reserve(ranges.size());
    scan.valid.reserve(ranges.size());
    
    for (size_t i = 0; i < ranges.size(); ++i) {
        double angle = angle_min + i * angle_increment;
        double range = static_cast<double>(ranges[i]);
        
        // Check if within angular range we care about
        bool in_range = (angle >= config_.angle_min && angle <= config_.angle_max);
        
        // Check if range is valid
        bool valid_range = std::isfinite(range) && 
                          range >= config_.range_min && 
                          range <= config_.range_max;
        
        scan.ranges.push_back(range);
        scan.angles.push_back(angle);
        scan.valid.push_back(in_range && valid_range);
    }
    
    // Apply filtering
    scan.filtered_ranges = scan.ranges;
    
    if (config_.apply_median_filter && config_.median_window_size > 1) {
        applyMedianFilter(scan.filtered_ranges);
    }
    
    // Validate and clip
    validateRanges(scan);
    
    return scan;
}

void LidarProcessor::applyMedianFilter(std::vector<double>& ranges) {
    ranges = math::medianFilter(ranges, config_.median_window_size);
}

void LidarProcessor::validateRanges(ProcessedScan& scan) {
    for (size_t i = 0; i < scan.filtered_ranges.size(); ++i) {
        double& range = scan.filtered_ranges[i];
        
        if (!std::isfinite(range) || range < config_.range_min) {
            range = config_.range_min;
            scan.valid[i] = false;
        } else if (range > config_.range_max) {
            range = config_.range_max;
        }
    }
}

void LidarProcessor::applyDisparityExtension(ProcessedScan& scan, double car_width) {
    if (scan.filtered_ranges.size() < 2) return;
    
    std::vector<double>& ranges = scan.filtered_ranges;
    const double half_car = car_width / 2.0;
    
    for (size_t i = 1; i < ranges.size(); ++i) {
        double diff = std::abs(ranges[i] - ranges[i-1]);
        
        if (diff > config_.disparity_threshold) {
            // Found a disparity - extend the closer reading
            size_t closer_idx = (ranges[i] < ranges[i-1]) ? i : i-1;
            double closer_range = ranges[closer_idx];
            
            // Calculate how many indices to extend based on car width
            // angle_to_extend = atan(car_width / 2 / closer_range)
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
                // Closer point is on the left (at i-1), extend right
                // Safe: we know i >= 1 since we start loop at i = 1
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

void LidarProcessor::applySafetyBubble(ProcessedScan& scan) {
    if (!config_.apply_bubble || scan.filtered_ranges.empty()) return;
    
    // Find closest point
    size_t closest_idx = findClosestPoint(scan);
    double closest_range = scan.filtered_ranges[closest_idx];
    
    if (closest_range >= config_.range_max) return;  // No close obstacles
    
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

std::vector<Gap> LidarProcessor::findGaps(const ProcessedScan& scan) {
    std::vector<Gap> gaps;
    if (scan.filtered_ranges.empty()) return gaps;
    
    bool in_gap = false;
    Gap current_gap;
    
    for (size_t i = 0; i < scan.filtered_ranges.size(); ++i) {
        double range = scan.filtered_ranges[i];
        double angle = scan.angles[i];
        
        // Check if within our angular processing range
        if (angle < config_.angle_min || angle > config_.angle_max) {
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

Gap LidarProcessor::findBestGap(
    const std::vector<Gap>& gaps,
    std::function<double(const Gap&)> scorer
) {
    if (gaps.empty()) {
        return Gap();  // Return invalid gap
    }
    
    // Default scorer: prefer deepest gaps
    if (!scorer) {
        scorer = [](const Gap& g) {
            return g.deepest_range * g.angular_width;
        };
    }
    
    double best_score = -std::numeric_limits<double>::infinity();
    const Gap* best_gap = &gaps[0];
    
    for (const auto& gap : gaps) {
        double score = scorer(gap);
        if (score > best_score) {
            best_score = score;
            best_gap = &gap;
        }
    }
    
    return *best_gap;
}

size_t LidarProcessor::findClosestPoint(const ProcessedScan& scan) {
    if (scan.filtered_ranges.empty()) return 0;
    
    size_t closest_idx = 0;
    double min_range = std::numeric_limits<double>::infinity();
    
    for (size_t i = 0; i < scan.filtered_ranges.size(); ++i) {
        double angle = scan.angles[i];
        if (angle < config_.angle_min || angle > config_.angle_max) {
            continue;
        }
        
        if (scan.valid[i] && scan.filtered_ranges[i] < min_range) {
            min_range = scan.filtered_ranges[i];
            closest_idx = i;
        }
    }
    
    return closest_idx;
}

size_t LidarProcessor::findFurthestPoint(const ProcessedScan& scan) {
    if (scan.filtered_ranges.empty()) return 0;
    
    size_t furthest_idx = 0;
    double max_range = 0.0;
    
    for (size_t i = 0; i < scan.filtered_ranges.size(); ++i) {
        double angle = scan.angles[i];
        if (angle < config_.angle_min || angle > config_.angle_max) {
            continue;
        }
        
        if (scan.valid[i] && scan.filtered_ranges[i] > max_range) {
            max_range = scan.filtered_ranges[i];
            furthest_idx = i;
        }
    }
    
    return furthest_idx;
}

double LidarProcessor::getRangeAtAngle(const ProcessedScan& scan, double angle) {
    if (scan.angles.empty()) return 0.0;
    
    // Find the two nearest angle indices and interpolate
    double angle_normalized = math::normalizeAngle(angle);
    
    // Binary search for closest angle
    auto it = std::lower_bound(scan.angles.begin(), scan.angles.end(), angle_normalized);
    
    if (it == scan.angles.end()) {
        return scan.filtered_ranges.back();
    }
    if (it == scan.angles.begin()) {
        return scan.filtered_ranges.front();
    }
    
    size_t idx_high = std::distance(scan.angles.begin(), it);
    size_t idx_low = idx_high - 1;
    
    // Linear interpolation
    double t = (angle_normalized - scan.angles[idx_low]) / 
               (scan.angles[idx_high] - scan.angles[idx_low]);
    
    return math::lerp(scan.filtered_ranges[idx_low], scan.filtered_ranges[idx_high], t);
}

Point2D LidarProcessor::scanPointToCartesian(const ProcessedScan& scan, size_t index) {
    if (index >= scan.filtered_ranges.size()) {
        return Point2D();
    }
    
    double range = scan.filtered_ranges[index];
    double angle = scan.angles[index];
    
    return Point2D(range * std::cos(angle), range * std::sin(angle));
}

std::vector<Point2D> LidarProcessor::scanToCartesian(const ProcessedScan& scan) {
    std::vector<Point2D> points;
    points.reserve(scan.filtered_ranges.size());
    
    for (size_t i = 0; i < scan.filtered_ranges.size(); ++i) {
        if (scan.valid[i]) {
            points.push_back(scanPointToCartesian(scan, i));
        }
    }
    
    return points;
}

std::vector<BoundaryPoint> LidarProcessor::extractBoundaryPoints(
    const ProcessedScan& scan,
    const Pose2D& robot_pose,
    double timestamp
) {
    std::vector<BoundaryPoint> boundary_points;
    boundary_points.reserve(scan.filtered_ranges.size());
    
    for (size_t i = 0; i < scan.filtered_ranges.size(); ++i) {
        if (!scan.valid[i]) continue;
        
        double angle = scan.angles[i];
        if (angle < config_.angle_min || angle > config_.angle_max) continue;
        
        // Convert to Cartesian in robot frame
        Point2D local_point = scanPointToCartesian(scan, i);
        
        // Transform to global/map frame
        Point2D global_point = math::localToGlobal(local_point, robot_pose);
        
        BoundaryPoint bp;
        bp.position = global_point;
        bp.timestamp = timestamp;
        bp.is_left_wall = (angle > 0);  // Positive angles are left side
        bp.confidence = 1.0;  // Could be based on range or other factors
        
        boundary_points.push_back(bp);
    }
    
    return boundary_points;
}

}  // namespace f1tenth_control
