#ifndef F1TENTH_CONTROL_FOLLOW_THE_GAP_HPP_
#define F1TENTH_CONTROL_FOLLOW_THE_GAP_HPP_

#include "f1tenth_control/common/types.hpp"
#include "f1tenth_control/common/lidar_processor.hpp"
#include <memory>
#include <functional>
#include <chrono>
#include <vector>

namespace f1tenth_control {

/**
 * @brief Configuration for the Weighted Free-Space FTG algorithm
 *
 * This algorithm replaces discrete gap-finding with a continuous drivability
 * scoring approach.  For every beam direction it computes an "effective
 * clearance" (the minimum range within a cone that accounts for car width),
 * scores directions with an exponential heading preference, and steers toward
 * the *weighted centroid* of all high-scoring directions.  This naturally
 * centres the car in corridors and follows curves smoothly.
 */
struct FTGConfig {
    // -- Vehicle parameters ---------------------------------------------------
    double wheelbase{0.3302};        // Distance between axles (m)
    double car_width{0.30};          // Vehicle width for clearance cone (m)

    // -- Speed control --------------------------------------------------------
    double max_speed{2.0};           // Maximum speed (m/s)
    double min_speed{1.0};           // Minimum speed (m/s)
    double speed_full_range{4.0};    // Range (m) at which full speed is used
    double steer_slowdown_gain{0.5}; // How much steering reduces speed (0-1)

    // -- Steering control -----------------------------------------------------
    double max_steering{0.4262};     // [rad] Maximum steering angle (~24 deg)
    double steering_gain{1.0};       // Proportional gain on target angle
    double max_steering_rate{3.5};   // [rad/s] Maximum steering change rate
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

    // -- Generic LiDAR preprocessing ------------------------------------------
    LidarProcessorConfig lidar_config;

    // -- Mapping mode ---------------------------------------------------------
    bool mapping_mode{false};
    double mapping_sample_rate{10.0};
};

/**
 * @brief FTG algorithm output (unchanged interface for node compatibility)
 */
struct FTGOutput {
    DriveCommand command;
    Gap selected_gap;
    size_t closest_point_idx{0};
    double closest_point_dist{0.0};
    bool emergency_stop{false};
    std::vector<Gap> all_gaps;
    std::vector<BoundaryPoint> boundary_points;
    ProcessedScan processed_scan;
};

/**
 * @brief Weighted Free-Space Follow The Gap Algorithm
 *
 * Instead of discrete gap detection, this algorithm:
 *  1. Preprocesses the LiDAR scan (median filter, range clip).
 *  2. Applies disparity extension for safety near narrow passages.
 *  3. Computes an "effective clearance" for every beam direction --
 *     the minimum range within a cone that accounts for the car's width.
 *  4. Scores each direction:  score = eff_clearance^power * exp(-w|theta|)
 *  5. Computes the target angle as the weighted centroid of all scored beams.
 *  6. Applies EMA smoothing + rate limiting for smooth steering.
 *  7. Sets speed proportional to forward clearance and inversely to steering.
 *
 * The weighted-centroid approach is inherently smooth (one noisy beam barely
 * moves the average) and naturally centres the car in corridors because
 * symmetric clearance produces a centroid at theta ~ 0.
 */
class FollowTheGap {
public:
    explicit FollowTheGap(const FTGConfig& config = FTGConfig());

    void setConfig(const FTGConfig& config);
    const FTGConfig& getConfig() const { return config_; }

    /**
     * @brief Compute drive command from LiDAR scan
     */
    FTGOutput compute(
        const std::vector<float>& ranges,
        double angle_min,
        double angle_max,
        double angle_increment,
        const Pose2D& current_pose = Pose2D(),
        double timestamp = 0.0
    );

    LidarProcessor& getLidarProcessor() { return lidar_processor_; }
    const LidarProcessor& getLidarProcessor() const { return lidar_processor_; }

    void reset();

private:
    FTGConfig config_;
    LidarProcessor lidar_processor_;
    double last_steering_{0.0};
    double smoothed_target_{0.0};
    bool first_compute_{true};
    std::chrono::steady_clock::time_point last_compute_time_;

    // -- LiDAR safety processing ----------------------------------------------
    void applyDisparityExtension(ProcessedScan& scan);
    void applyWallMargin(ProcessedScan& scan);

    // -- Weighted free-space core ---------------------------------------------

    /**
     * @brief Compute effective clearance for every beam.
     *
     * For beam i the effective clearance is the minimum filtered range within
     * a cone of +/- atan(car_half_width * scale / range) indices around i.
     */
    std::vector<double> computeEffectiveClearance(const ProcessedScan& scan);

    /**
     * @brief Compute weighted-centroid target angle from effective clearances.
     *
     * score_i = max(0, clearance_i - min_score_range)^power
     *         * exp(-heading_weight * |angle_i|)
     * target  = sum(angle_i * score_i) / sum(score_i)
     */
    double computeTargetAngle(const ProcessedScan& scan,
                              const std::vector<double>& eff_clearance);

    // -- Gap detection (lightweight, for visualisation only) ------------------
    std::vector<Gap> findGapsForViz(const ProcessedScan& scan);
    Gap findBestGapForViz(const std::vector<Gap>& gaps);

    // -- Control --------------------------------------------------------------
    double calculateSpeed(double forward_clearance, double steering_angle);
    double smoothSteering(double target, double last, double dt);
};

}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_FOLLOW_THE_GAP_HPP_
