#ifndef F1TENTH_CONTROL_STANLEY_HPP_
#define F1TENTH_CONTROL_STANLEY_HPP_

#include "f1tenth_control/common/types.hpp"
#include "f1tenth_control/common/math_utils.hpp"
#include <vector>
#include <string>

namespace f1tenth_control {

/**
 * @brief Configuration for Stanley controller
 */
struct StanleyConfig {
    // Gain parameters
    double k_e{2.5};                // Cross-track error gain
    double k_h{1.0};                // Heading error gain (typically 1.0)
    double k_s{1.0};                // Softening constant for low speed (prevents division by zero)
    double k_d{0.1};                // Damping gain (reduces oscillation using angular velocity)
    
    // Speed control
    double max_speed{15.0};         // [m/s] Maximum speed
    double min_speed{1.0};          // [m/s] Minimum speed
    double speed_gain{1.0};         // Multiplier for trajectory target speed
    
    // Steering limits
    double max_steering{0.4189};    // [rad] Maximum steering angle (~24°)
    double max_steering_rate{0.5};  // [rad/s] Maximum steering rate (prevents sudden changes)
    
    // Vehicle parameters
    double wheelbase{0.3302};       // [m] Distance between axles
    
    // Path tracking
    double position_tolerance{0.5}; // [m] Max deviation before re-finding closest point
    
    // Feedforward
    bool use_feedforward{true};     // Use trajectory curvature for feedforward steering
    double feedforward_gain{1.0};   // Feedforward curvature gain
    
    // Speed adaptation
    double curvature_speed_factor{0.8}; // Speed reduction based on path curvature
    
    // Control rate (for steering rate limiting)
    double control_rate{200.0};         // [Hz] Control loop frequency
};

// TrajectoryPoint is defined in common/types.hpp

/**
 * @brief Output from Stanley controller
 */
struct StanleyOutput {
    double steering_angle{0.0};     // [rad] Commanded steering angle
    double target_speed{0.0};       // [m/s] Commanded speed
    
    // Debug info
    double cross_track_error{0.0};  // [m] Lateral error from path
    double heading_error{0.0};      // [rad] Heading error
    double feedforward_steering{0.0}; // [rad] Feedforward component
    double heading_term{0.0};       // [rad] Heading contribution to steering
    double cte_term{0.0};           // [rad] CTE contribution to steering
    size_t closest_idx{0};          // Index of closest waypoint
    bool valid{false};
};

/**
 * @brief Stanley path-following controller
 * 
 * The Stanley controller computes steering based on:
 * 1. Heading error (align with path tangent)
 * 2. Cross-track error (move toward path)
 * 3. Optional feedforward from path curvature
 * 
 * Steering formula:
 *   δ = θ_e + atan(k_e * e / (k_s + v)) + k_ff * κ * L
 * 
 * Where:
 *   θ_e = heading error (path heading - vehicle heading)
 *   e   = cross-track error (signed distance to path)
 *   v   = vehicle velocity
 *   κ   = path curvature
 *   L   = wheelbase
 *   k_e = cross-track gain
 *   k_s = softening constant
 *   k_ff = feedforward gain
 * 
 * Note: Stanley uses the front axle position, not rear axle.
 */
class Stanley {
public:
    /**
        * @brief Construct a Stanley controller using caller-provided configuration.
        * @param config Initial Stanley tuning and limit parameters.
        * @return None.
     */
    explicit Stanley(const StanleyConfig& config);
    
    /**
     * @brief Parse and store a track trajectory for Stanley path tracking.
     * @param csv_path Path to trajectory CSV file in expected format.
     * @return True when loading succeeds and trajectory storage is updated.
     */
    bool loadTrajectory(const std::string& csv_path);
    
    /**
     * @brief Replace trajectory data from an in-memory waypoint sequence.
     * @param trajectory Ordered waypoints describing the reference path.
     * @return None.
     */
    void setTrajectory(const std::vector<TrajectoryPoint>& trajectory);
    
    /**
     * @brief Compute Stanley steering command and regulated target speed.
     * @param state Current vehicle state (rear-axle pose convention).
     * @return StanleyOutput with command values and diagnostic terms.
     */
    StanleyOutput compute(const VehicleState& state);
    
    /**
     * @brief Update control gains and limits without rebuilding the object.
     * @param config New Stanley configuration values.
     * @return None.
     */
    void setConfig(const StanleyConfig& config) { config_ = config; }

    /**
     * @brief Expose active controller parameters for diagnostics.
     * @return Const reference to current StanleyConfig.
     */
    const StanleyConfig& getConfig() const { return config_; }
    
    /**
     * @brief Provide read-only access to loaded trajectory waypoints.
     * @return Const reference to internal trajectory vector.
     */
    const std::vector<TrajectoryPoint>& getTrajectory() const { return trajectory_; }
    
    /**
     * @brief Report whether controller has enough path data to operate.
     * @return True when trajectory storage is non-empty.
     */
    bool hasTrajectory() const { return !trajectory_.empty(); }
    
    /**
     * @brief Compute cumulative path length for reporting and validation.
     * @return Total trajectory arc length in meters.
     */
    double getTrajectoryLength() const;
    
private:
    StanleyConfig config_;
    std::vector<TrajectoryPoint> trajectory_;
    size_t last_closest_idx_{0};
    double last_steering_{0.0};     // For steering rate limiting
    bool search_initialized_{false}; // True after first findClosestPoint call
    
    /**
     * @brief Select the nearest path index while enforcing heading-consistent segment selection.
     * @param front_axle_pos Vehicle front-axle position in world coordinates.
     * @param vehicle_heading Current vehicle heading angle.
     * @return Closest trajectory index used by control law computation.
     */
    size_t findClosestPoint(const Point2D& front_axle_pos, double vehicle_heading);
    
    /**
     * @brief Compute signed lateral displacement used by the Stanley correction term.
     * @param front_axle_pos Vehicle front-axle position.
     * @param closest_idx Reference trajectory index near current position.
     * @return Signed cross-track error in meters.
     */
    double computeCrossTrackError(const Point2D& front_axle_pos, size_t closest_idx);
    
    /**
     * @brief Normalize angular differences for stable control arithmetic.
     * @param angle Raw angle value in radians.
     * @return Angle wrapped to canonical interval [-pi, pi].
     */
    static double normalizeAngle(double angle) {
        return math::normalizeAngle(angle);
    }
};

}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_STANLEY_HPP_
