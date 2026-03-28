#ifndef F1TENTH_CONTROL_FOLLOW_THE_GAP_HPP_
#define F1TENTH_CONTROL_FOLLOW_THE_GAP_HPP_

#include "f1tenth_control/common/types.hpp"
#include "f1tenth_control/common/lidar_processor.hpp"
#include <chrono>
#include <vector>

namespace f1tenth_control {

/**
 * @brief Configuration parameters for the weighted free-space FTG controller.
 *
 * The algorithm computes an effective clearance for each beam direction,
 * scores candidate directions, and selects steering by weighted centroiding.
 */
struct FTGConfig {
    // -- Vehicle parameters ---------------------------------------------------
    double wheelbase{0.3302};        // Distance between axles (m)
    double car_width{0.273};         // Vehicle width for clearance cone (m)

    // -- Speed control --------------------------------------------------------
    double max_speed{2.0};           // Maximum speed (m/s)
    double min_speed{1.0};           // Minimum speed (m/s)
    double speed_full_range{4.0};    // Range (m) at which full speed is used
    double steer_slowdown_gain{0.5}; // How much steering reduces speed (0-1)

    // -- Steering control -----------------------------------------------------
    double max_steering{0.4189};     // [rad] Maximum steering angle (~24 deg, calibrated)
    double steering_gain{1.0};       // Proportional gain on target angle
    double max_steering_rate{2.8492};// [rad/s] Maximum steering change rate
    double target_ema_alpha{0.35};   // EMA smoothing for target angle (lower = smoother)

    // -- Weighted free-space scoring ------------------------------------------
    double heading_weight{1.0};      // Exponential decay for non-forward dirs
    double score_power{2.0};         // Raise effective clearance to this power
    double clearance_cone_scale{1.5};// Multiplier on car half-width for cone
    double min_score_range{0.3};     // Beams shorter than this get zero score (m)

    // -- Safety ---------------------------------------------------------------
    double emergency_brake_distance{0.15}; // Brake if any beam closer (m)

    // -- LiDAR processing -----------------------------------------------------
    double disparity_threshold{0.5}; // Threshold for disparity extension (m)
    double wall_margin{0.15};        // Shrink all readings by this (m)
    double gap_threshold{0.5};       // Min range to count as "gap" in viz (m)
    double min_gap_width{0.15};      // Min angular width of gap for viz (rad)

    // -- Generic LiDAR preprocessing -----------------------------------------
    LidarProcessorConfig lidar_config;
};

/**
 * @brief Output of one FTG compute cycle.
 */
struct FTGOutput {
    DriveCommand command;            // Drive command generated for current cycle.
    Gap selected_gap;                // Best visualization gap selected from all_gaps.
    size_t closest_point_idx{0};     // Index of nearest detected point in processed scan.
    double closest_point_dist{0.0};  // Distance to nearest detected point.
    bool emergency_stop{false};      // True when emergency brake condition is active.
    std::vector<Gap> all_gaps;       // All detected gaps used for diagnostics/visualization.
    ProcessedScan processed_scan;    // Fully processed scan used for command generation.
};

/**
 * @brief Weighted free-space Follow-The-Gap controller.
 * The controller: 
 * 1. Preprocesses LiDAR data. 
 * 2. Applies conservative safety shaping.
 * 3. Computes direction-wise drivability scores.
 * 4. Outputs bounded steering/speed commands.
 */
class FollowTheGap {
public:
    /**
     * @brief Construct a FollowTheGap controller instance.
     * @param config FTG configuration values used by the controller.
     */
    explicit FollowTheGap(const FTGConfig& config);

    /**
     * @brief Update runtime FTG configuration.
     * @param config New FTG configuration values.
     * @return None.
     */
    void setConfig(const FTGConfig& config);

    /**
     * @brief Get current FTG configuration.
     * @return Const reference to active configuration.
     */
    const FTGConfig& getConfig() const { return config_; }

    /**
     * @brief Compute FTG command output from one LiDAR scan.
     * @param ranges Raw LiDAR range vector.
     * @param angle_min Start angle of LiDAR scan.
     * @param angle_max End angle of LiDAR scan.
     * @param angle_increment Angular increment between neighboring beams.
     * @return FTGOutput with command and diagnostics for the current cycle.
     */
    FTGOutput compute(
        const std::vector<float>& ranges,
        double angle_min,
        double angle_max,
        double angle_increment
    );

    /**
     * @brief Get read-only access to shared LiDAR preprocessor.
     * @return Const reference to internal LidarProcessor.
     */
    const LidarProcessor& getLidarProcessor() const { return lidar_processor_; }

    /**
     * @brief Reset temporal controller state.
     */
    void reset();

private:
    FTGConfig config_;
    LidarProcessor lidar_processor_;
    double last_steering_{0.0};
    double smoothed_target_{0.0};
    bool first_compute_{true};
    std::chrono::steady_clock::time_point last_compute_time_;

    // -- LiDAR safety processing ----------------------------------------------

    /**
     * @brief Inflate scan around disparity edges for safety.
     * @param scan Processed scan to modify in place.
     */
    void applyDisparityExtension(ProcessedScan& scan);

    /**
     * @brief Apply uniform wall safety margin to scan ranges.
     * @param scan Processed scan to modify in place.
     */
    void applyWallMargin(ProcessedScan& scan);

    // -- Weighted free-space core ---------------------------------------------

    /**
     * @brief Compute per-beam effective clearance profile.
     * @param scan Safety-processed LiDAR scan.
     * @return Effective clearance value for each beam.
     */
    std::vector<double> computeEffectiveClearance(const ProcessedScan& scan);

    /**
     * @brief Compute weighted-centroid steering target.
     * @param scan Safety-processed LiDAR scan.
     * @param eff_clearance Effective clearance profile.
     * @return Target steering angle before smoothing/rate limiting.
     */
    double computeTargetAngle(const ProcessedScan& scan,
                              const std::vector<double>& eff_clearance);

    // -- Gap detection (lightweight, for visualisation only) ------------------

    /**
     * @brief Detect contiguous free-space gap segments for visualization.
     * @param scan Safety-processed LiDAR scan.
     * @return Vector of detected gap descriptors.
     */
    std::vector<Gap> findGapsForViz(const ProcessedScan& scan);

    /**
     * @brief Select representative gap from detected candidates.
     * @param gaps Candidate visualization gaps.
     * @return Selected gap descriptor.
     */
    Gap findBestGapForViz(const std::vector<Gap>& gaps);

    // -- Control --------------------------------------------------------------

    /**
     * @brief Compute bounded longitudinal speed command.
     * @param forward_clearance Estimated drivable clearance ahead.
     * @param steering_angle Current steering demand magnitude.
     * @return Commanded longitudinal speed.
     */
    double calculateSpeed(double forward_clearance, double steering_angle);

    /**
     * @brief Apply steering rate limiting.
     * @param target Desired steering angle before rate limiting.
     * @param last Previously emitted steering command.
     * @param dt Control time step.
     * @return Rate-limited steering command.
     */
    double smoothSteering(double target, double last, double dt);
};

}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_FOLLOW_THE_GAP_HPP_
