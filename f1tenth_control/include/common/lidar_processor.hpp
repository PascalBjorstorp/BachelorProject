#ifndef F1TENTH_CONTROL_LIDAR_PROCESSOR_HPP_
#define F1TENTH_CONTROL_LIDAR_PROCESSOR_HPP_

#include "f1tenth_control/common/types.hpp"
#include <vector>

namespace f1tenth_control {

/**
 * @brief Configuration for LiDAR processing
 * 
 * Contains only generic preprocessing parameters.
 * Algorithm-specific parameters (gap detection, safety bubble, etc.)
 * should be in the respective algorithm's config.
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
 * by any algorithm (FTG, Pure Pursuit, obstacle detection, mapping, etc.)
 * 
 * Design principle: Only generic preprocessing here. Algorithm-specific
 * processing (gap detection, safety bubbles) belongs in the algorithm class.
 */
class LidarProcessor {
public:
    /**
     * Inputs:
     * - config: Preprocessing bounds and filtering settings.
     *
     * Purpose:
     * - Construct reusable LiDAR preprocessing utility with deterministic defaults.
     *
     * Outputs:
     * - Creates a processor instance ready for scan conversion and filtering.
     */
    explicit LidarProcessor(const LidarProcessorConfig& config = LidarProcessorConfig());
    
    /**
     * Inputs:
     * - config: New LiDAR preprocessing configuration.
     *
     * Purpose:
     * - Update preprocessing behavior without reconstructing processor instance.
     *
     * Outputs:
     * - Replaces active preprocessing configuration.
     */
    void setConfig(const LidarProcessorConfig& config);

    /**
     * Inputs:
     * - None.
     *
     * Purpose:
     * - Expose current preprocessing configuration for diagnostics.
     *
     * Outputs:
     * - Returns const reference to active LidarProcessorConfig.
     */
    const LidarProcessorConfig& getConfig() const { return config_; }
    
    /**
     * Inputs:
     * - ranges: Raw LiDAR range measurements.
     * - angle_min: Minimum scan angle.
     * - angle_max: Maximum scan angle.
     * - angle_increment: Angular step between beams.
     *
     * Purpose:
     * - Convert raw sensor ranges into filtered and validated scan representation
     *   suitable for downstream control/planning algorithms.
     *
     * Outputs:
     * - Returns ProcessedScan with clipped ranges, validity mask, and beam angles.
     */
    ProcessedScan processScan(
        const std::vector<float>& ranges,
        double angle_min,
        double angle_max,
        double angle_increment
    );
    
    /**
     * Inputs:
     * - scan: Preprocessed LiDAR scan with validity mask.
     *
     * Purpose:
     * - Identify nearest valid obstacle/range sample in the current scan.
     *
     * Outputs:
     * - Returns index of closest valid beam in scan arrays.
     */
    size_t findClosestPoint(const ProcessedScan& scan);
    
    /**
     * Inputs:
     * - scan: Preprocessed LiDAR scan.
     * - index: Beam index to convert.
     *
     * Purpose:
     * - Convert one polar scan sample into robot-frame Cartesian coordinates.
     *
     * Outputs:
     * - Returns Point2D corresponding to selected beam.
     */
    Point2D scanPointToCartesian(const ProcessedScan& scan, size_t index);
    
    /**
     * Inputs:
     * - scan: Preprocessed LiDAR scan.
     * - robot_pose: Vehicle pose used for frame transform to map/world frame.
     * - timestamp: Time tag for output boundary points.
     *
     * Purpose:
     * - Extract valid scan endpoints and project them into mapping frame.
     *
     * Outputs:
     * - Returns boundary points with transformed coordinates and timestamp.
     */
    std::vector<BoundaryPoint> extractBoundaryPoints(
        const ProcessedScan& scan,
        const Pose2D& robot_pose,
        double timestamp
    );

private:
    LidarProcessorConfig config_;
    
    /**
     * Inputs:
     * - ranges: Range vector to smooth in place.
     *
     * Purpose:
     * - Suppress isolated measurement spikes with local median filtering.
     *
     * Outputs:
     * - Mutates input range vector with filtered values.
     */
    void applyMedianFilter(std::vector<double>& ranges);
    
    /**
     * Inputs:
     * - scan: Processed scan object to validate in place.
     *
     * Purpose:
     * - Enforce configured range bounds and update validity indicators.
     *
     * Outputs:
     * - Mutates scan ranges/flags to reflect validated clipping results.
     */
    void validateRanges(ProcessedScan& scan);
};

}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_LIDAR_PROCESSOR_HPP_
