#ifndef F1TENTH_CONTROL_LIDAR_PROCESSOR_HPP_
#define F1TENTH_CONTROL_LIDAR_PROCESSOR_HPP_

/**
 * @file lidar_processor.hpp
 * @brief Generic LiDAR scan preprocessing shared across algorithms.
 * @details Converts raw float range vectors into validated, filtered ProcessedScan
 *          representations. Algorithm-specific processing (disparity extension,
 *          safety bubbles) is deliberately excluded and belongs in the calling
 *          algorithm class. Thread-safe for concurrent reads after construction;
 *          setConfig() is not thread-safe.
 * @dependencies types.hpp, math_utils.hpp, <vector>, <cmath> and <limits>
 */

#include "common/types.hpp"
#include "common/math_utils.hpp"
#include <vector>
#include <cmath>
#include <limits>

namespace f1tenth_control {

/**
 * @brief Configuration for LiDAR processing
 * 
 * Contains generic preprocessing parameters.
 */
struct LidarProcessorConfig {
    // Range filtering
    double range_min{0.1};           // Minimum valid range (m)
    double range_max{10.0};          // Maximum valid range (m)
    
    // Angular filtering (process only this range)
    double angle_min{-2.35};         // Minimum angle to process (rad)
    double angle_max{2.35};          // Maximum angle to process (rad)
    
    // Noise filtering
    bool apply_median_filter{true};  // Apply median filter
    size_t median_window_size{3};    // Median filter window size
};

/**
 * @brief Generic LiDAR processing utilities
 * 
 * This class provides reusable LiDAR processing functions that can be used
 * by any algorithm
 */
class LidarProcessor {
public:
    /**
     * @brief Construct a LiDAR processor with specified configuration.
     * @param config Preprocessing bounds and filtering settings.
     */
    explicit LidarProcessor(const LidarProcessorConfig& config = LidarProcessorConfig());
    
    /**
     * @brief Update LiDAR preprocessing configuration at runtime.
     * @param config New preprocessing settings to apply.
     */
    void setConfig(const LidarProcessorConfig& config);

    /**
     * @brief Get current LiDAR preprocessing configuration.
     * @return Const reference to active configuration.
     */
    const LidarProcessorConfig& getConfig() const { return config_; }
    
    /**
     * @brief Process raw LiDAR scan into validated and filtered representation.
     * @param ranges Raw LiDAR range measurements.
     * @param angle_min Minimum angle of the scan.
     * @param angle_max Maximum angle of the scan.
     * @param angle_increment Angular step between beams.
     * @return ProcessedScan with clipped ranges, validity mask, and beam angles.
     */
    ProcessedScan processScan(
        const std::vector<float>& ranges,
        double angle_min,
        double angle_max,
        double angle_increment
    );
    
    /**
     * @brief Find the index of the closest valid point in a processed LiDAR scan.
     * @param scan Preprocessed LiDAR scan with validity mask.
     * @return Index of the closest valid beam in the scan arrays.
     */
    size_t findClosestPoint(const ProcessedScan& scan);

    /**
     * @brief Convert a polar scan point to Cartesian coordinates in robot frame.
     * @param scan Preprocessed LiDAR scan containing angles and ranges.
     * @param index Beam index to convert.
     * @return Point2D corresponding to the selected beam.
     */
    Point2D scanPointToCartesian(const ProcessedScan& scan, size_t index);
    
    /**
     * @brief Extract boundary points from a processed LiDAR scan.
     * @param scan Preprocessed LiDAR scan with validity and angle/range data.
     * @param robot_pose Current pose of the robot for transforming points to map frame.
     * @param timestamp Timestamp to associate with extracted boundary points.
     * @return Vector of BoundaryPoint representing detected track boundaries.
     */
    std::vector<BoundaryPoint> extractBoundaryPoints(
        const ProcessedScan& scan,
        const Pose2D& robot_pose,
        double timestamp
    );

private:
    LidarProcessorConfig config_;
    
    /**
     * @brief Apply median filter to a sequence of range measurements.
     * @param ranges Input range vector to filter (in-place).
     */
    void applyMedianFilter(std::vector<double>& ranges);
    
    /**
     * @brief Validate and clip range measurements based on configured limits.
     * @param scan ProcessedScan to validate and modify in place.
     * @return None (mutates scan.ranges and scan.valid).
     */
    void validateRanges(ProcessedScan& scan);
};

}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_LIDAR_PROCESSOR_HPP_
