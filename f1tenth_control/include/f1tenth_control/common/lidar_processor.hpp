#ifndef F1TENTH_CONTROL_LIDAR_PROCESSOR_HPP_
#define F1TENTH_CONTROL_LIDAR_PROCESSOR_HPP_

#include "f1tenth_control/common/types.hpp"
#include <vector>
#include <functional>

namespace f1tenth_control {

/**
 * @brief Configuration for LiDAR processing
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
    
    // Gap detection
    double disparity_threshold{0.3}; // Threshold for disparity extension (m)
    double gap_threshold{3.0};       // Minimum range to consider as gap (m)
    double min_gap_width{0.3};       // Minimum angular width of gap (rad)
    
    // Safety bubble
    double bubble_radius{0.2};       // Radius around closest point to zero out (m)
    bool apply_bubble{true};         // Whether to apply safety bubble
};

/**
 * @brief Reusable LiDAR processing utilities
 * 
 * This class provides common LiDAR processing functions that can be used
 * by multiple algorithms (FTG, Pure Pursuit visualization, mapping, etc.)
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
     * @brief Apply disparity extension to ranges
     * 
     * Extends obstacles at disparity points (sudden range changes) to prevent
     * the robot from driving into narrow gaps that it cannot fit through.
     * 
     * @param scan Processed scan to modify (in-place)
     * @param car_width Width of the car for extension calculation
     */
    void applyDisparityExtension(ProcessedScan& scan, double car_width);
    
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
     * @brief Find the best gap (widest, deepest, or custom metric)
     * @param gaps Vector of gaps to search
     * @param scorer Custom scoring function (higher = better)
     * @return Best gap, or invalid gap if none found
     */
    Gap findBestGap(
        const std::vector<Gap>& gaps,
        std::function<double(const Gap&)> scorer = nullptr
    );
    
    /**
     * @brief Find closest point in scan
     * @param scan Processed scan
     * @return Index of closest point
     */
    size_t findClosestPoint(const ProcessedScan& scan);
    
    /**
     * @brief Find furthest point in scan
     * @param scan Processed scan
     * @return Index of furthest point
     */
    size_t findFurthestPoint(const ProcessedScan& scan);
    
    /**
     * @brief Get range at specific angle
     * @param scan Processed scan
     * @param angle Target angle in radians
     * @return Interpolated range at angle
     */
    double getRangeAtAngle(const ProcessedScan& scan, double angle);
    
    /**
     * @brief Convert scan point to Cartesian coordinates
     * @param scan Processed scan
     * @param index Index of point
     * @return Point in Cartesian coordinates (robot frame)
     */
    Point2D scanPointToCartesian(const ProcessedScan& scan, size_t index);
    
    /**
     * @brief Get all valid points as Cartesian coordinates
     * @param scan Processed scan
     * @return Vector of Cartesian points
     */
    std::vector<Point2D> scanToCartesian(const ProcessedScan& scan);
    
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
