#include "f1tenth_control/common/lidar_processor.hpp"
#include "f1tenth_control/common/math_utils.hpp"
#include <cmath>
#include <limits>

namespace f1tenth_control {

// Inputs:
// - config: LiDAR preprocessing configuration.
// Purpose:
// - Construct generic LiDAR processor with caller-provided settings.
// Outputs:
// - Initializes processor ready for processScan() calls.
LidarProcessor::LidarProcessor(const LidarProcessorConfig& config)
    : config_(config) {}

// Inputs:
// - config: Updated LiDAR preprocessing configuration.
// Purpose:
// - Apply runtime configuration updates without reconstructing processor.
// Outputs:
// - Replaces active configuration used in subsequent processing.
void LidarProcessor::setConfig(const LidarProcessorConfig& config) {
    config_ = config;
}

// Inputs:
// - ranges: Raw LiDAR ranges.
// - angle_min/angle_max/angle_increment: Scan angular metadata.
// Purpose:
// - Convert raw scan into filtered, clipped, and validity-tagged representation.
// Outputs:
// - Returns ProcessedScan structure for downstream algorithm use.
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
    
    // Convert to double and compute angles (resize + direct writes for cache efficiency)
    const size_t n = ranges.size();
    scan.ranges.resize(n);
    scan.angles.resize(n);
    scan.valid.resize(n);
    
    for (size_t i = 0; i < n; ++i) {
        double angle = angle_min + i * angle_increment;
        double range = static_cast<double>(ranges[i]);
        
        // Check if within angular range we care about
        bool in_range = (angle >= config_.angle_min && angle <= config_.angle_max);
        
        // Check if range is valid (finite and not too close)
        // Note: range > range_max is valid (it's just far away) - we'll clip it later
        bool valid_range = std::isfinite(range) && range >= config_.range_min;
        
        scan.ranges[i] = range;
        scan.angles[i] = angle;
        scan.valid[i] = (in_range && valid_range);
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

    // Inputs:
    // - ranges: Range vector to filter in place.
    // Purpose:
    // - Apply median filtering for impulsive-noise suppression.
    // Outputs:
    // - Mutates ranges with median-filtered values.
void LidarProcessor::applyMedianFilter(std::vector<double>& ranges) {
    ranges = math::medianFilter(ranges, config_.median_window_size);
}

    // Inputs:
    // - scan: Processed scan to validate in place.
    // Purpose:
    // - Enforce finite values and configured range limits while updating validity mask.
    // Outputs:
    // - Mutates scan.filtered_ranges and scan.valid.
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

// Inputs:
// - scan: Processed LiDAR scan.
// Purpose:
// - Locate nearest valid obstacle sample within configured angular sector.
// Outputs:
// - Returns index of closest valid point (0 when no valid points are found).
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

    // Inputs:
    // - scan: Processed LiDAR scan.
    // - index: Beam index to convert.
    // Purpose:
    // - Convert one scan sample from polar to Cartesian robot-frame coordinates.
    // Outputs:
    // - Returns corresponding Point2D (origin when index is out of range).
Point2D LidarProcessor::scanPointToCartesian(const ProcessedScan& scan, size_t index) {
    if (index >= scan.filtered_ranges.size()) {
        return Point2D();
    }
    
    double range = scan.filtered_ranges[index];
    double angle = scan.angles[index];
    
    return Point2D(range * std::cos(angle), range * std::sin(angle));
}

    // Inputs:
    // - scan: Processed LiDAR scan.
    // - robot_pose: Robot pose in global/map frame.
    // - timestamp: Time tag assigned to extracted points.
    // Purpose:
    // - Project valid scan points into global frame for mapping/boundary extraction.
    // Outputs:
    // - Returns vector of BoundaryPoint records.
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
