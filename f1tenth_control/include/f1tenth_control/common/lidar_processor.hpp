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
    explicit LidarProcessor(const LidarProcessorConfig& config = LidarProcessorConfig());
    
    /**
     * @brief Update configuration
     */
    void setConfig(const LidarProcessorConfig& config);
    const LidarProcessorConfig& getConfig() const { return config_; }
    
    /**
     * @brief Process raw LiDAR scan
     * 
     * Performs generic preprocessing:
     * 1. Converts float ranges to doubles
     * 2. Filters angles to configured range
     * 3. Applies median filter (if enabled)
     * 4. Marks invalid readings
     * 5. Computes angle for each beam
     * 
     * @param ranges Raw range measurements
     * @param angle_min Minimum angle of scan
     * @param angle_max Maximum angle of scan
     * @param angle_increment Angular increment between measurements
     * @return Processed scan data
     */
    ProcessedScan processScan(
        const std::vector<float>& ranges,
        double angle_min,
        double angle_max,
        double angle_increment
    );
    
    /**
     * @brief Find closest valid point in scan
     * @param scan Processed scan
     * @return Index of closest point
     */
    size_t findClosestPoint(const ProcessedScan& scan);
    
    /**
     * @brief Convert scan point to Cartesian coordinates (robot frame)
     * @param scan Processed scan
     * @param index Index of point
     * @return Point in Cartesian coordinates
     */
    Point2D scanPointToCartesian(const ProcessedScan& scan, size_t index);
    
    /**
     * @brief Extract boundary points from scan for mapping
     * @param scan Processed scan
     * @param robot_pose Current robot pose in map frame
     * @param timestamp Current timestamp
     * @return Vector of boundary points in map frame
     */
    std::vector<BoundaryPoint> extractBoundaryPoints(
        const ProcessedScan& scan,
        const Pose2D& robot_pose,
        double timestamp
    );

private:
    LidarProcessorConfig config_;
    
    /**
     * @brief Apply median filter to ranges
     */
    void applyMedianFilter(std::vector<double>& ranges);
    
    /**
     * @brief Validate and clip ranges
     */
    void validateRanges(ProcessedScan& scan);
};

}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_LIDAR_PROCESSOR_HPP_
