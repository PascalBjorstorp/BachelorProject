#ifndef F1TENTH_CONTROL_PURE_PURSUIT_HPP_
#define F1TENTH_CONTROL_PURE_PURSUIT_HPP_

#include "f1tenth_control/common/types.hpp"
#include "f1tenth_control/common/math_utils.hpp"
#include <vector>
#include <string>
#include <cmath>

namespace f1tenth_control {

/**
 * @brief Configuration for Pure Pursuit controller
 */
struct PurePursuitConfig {
    // Lookahead parameters
    double min_lookahead{0.48};      // [m] Minimum lookahead distance
    double max_lookahead{1.20};      // [m] Maximum lookahead distance
    double lookahead_gain{0.15};     // Velocity-proportional gain: L = min_L + k*v
    double cte_lookahead_weight{1.0}; // [unitless] Weight on |CTE| contribution
    double cte_lookahead_gain{0.05}; // [m/m] Reduce lookahead with cross-track error
    double curvature_lookahead_gain{1.34}; // [m·m] L_max = L_curv_gain / |κ| (turn-radius-based limit)

    // Speed control
    double curvature_speed_factor{0.10}; // [unitless] Aggressiveness of curvature-based slowdown
    double curvature_speed_floor_ratio{0.43}; // [0..1] Minimum speed ratio after slowdown
    double cte_speed_factor{0.10}; // [unitless] Slowdown gain based on |CTE|
    double cte_speed_floor_ratio{0.50}; // [0..1] Minimum speed ratio from CTE slowdown
    double max_lateral_accel{7.27}; // [m/s^2] Physics-aware cornering speed cap
    double min_regulated_speed{0.30}; // [m/s] Lower bound after speed regulation
    double curvature_preview_factor{1.2}; // [unitless] Preview N× lookahead for braking

    // Footprint-aware corridor regulation
    double vehicle_half_width{0.1365}; // [m] Half of physical car width
    double wall_safety_margin{0.03}; // [m] Additional static wall clearance margin
    double corridor_half_width_ref{0.25}; // [m] Reference usable half-width for full speed
    double corridor_speed_floor_ratio{0.20}; // [0..1] Floor for corridor-based speed scaling
    double corridor_lookahead_factor{2.0}; // [m/m] Additional lookahead allowed per usable half-width
    
    // Steering limits
    double max_steering{0.4189};    // [rad] Maximum steering angle (~24°)
    
    // Vehicle parameters
    double wheelbase{0.324};        // [m] Distance between axles
    
    // Path tracking
    double position_tolerance{0.5}; // [m] Max deviation before re-finding closest point
};

// TrajectoryPoint is defined in common/types.hpp

/**
 * @brief Output from Pure Pursuit controller
 */
struct PurePursuitOutput {
    double steering_angle{0.0};  // [rad] Commanded steering angle
    double target_speed{0.0};    // [m/s] Commanded speed
    
    // Debug info
    size_t closest_idx{0};       // Index of closest waypoint
    size_t target_idx{0};        // Index of lookahead target
    Point2D target_point;        // Lookahead point in world frame
    double lookahead_distance{0.0};
    double cross_track_error{0.0};  // Lateral error from path
    bool valid{false};           // Whether output is valid
};

/**
 * @brief Pure Pursuit path-following controller
 * 
 * Pure Pursuit finds a target point at a lookahead distance on the path
 * and computes the steering angle to arc towards it.
 * 
 * The steering angle is computed as:
 *   delta = atan2(2 * L * sin(alpha) / lookahead_dist, 1)
 * 
 * Where:
 *   L = wheelbase
 *   alpha = angle to target point from vehicle heading
 *   lookahead_dist = distance to target point
 */
class PurePursuit {
public:
    PurePursuit();
    explicit PurePursuit(const PurePursuitConfig& config);
    
    /**
     * @brief Load trajectory from file
     * @param csv_path Path to CSV file (TUM format)
     * @return true if loaded successfully
     */
    bool loadTrajectory(const std::string& csv_path);
    
    /**
     * @brief Set trajectory directly
     * @param trajectory Vector of waypoints
     */
    void setTrajectory(const std::vector<TrajectoryPoint>& trajectory);
    
    /**
     * @brief Compute steering and speed commands
     * @param state Current vehicle state
     * @return Control output (steering, speed, debug info)
     */
    PurePursuitOutput compute(const VehicleState& state);
    
    /**
     * @brief Update configuration
     */
    void setConfig(const PurePursuitConfig& config) { config_ = config; }
    const PurePursuitConfig& getConfig() const { return config_; }
    
    /**
     * @brief Get loaded trajectory
     */
    const std::vector<TrajectoryPoint>& getTrajectory() const { return trajectory_; }
    
    /**
     * @brief Check if trajectory is loaded
     */
    bool hasTrajectory() const { return !trajectory_.empty(); }
    
    /**
     * @brief Get total trajectory length
     */
    double getTrajectoryLength() const;
    
private:
    PurePursuitConfig config_;
    std::vector<TrajectoryPoint> trajectory_;
    size_t last_closest_idx_{0};  // For efficient search
    double current_heading_{0.0}; // Car heading for direction-aware search
    
    /**
     * @brief Find closest point on trajectory to position
     * @param position Current position
     */
    size_t findClosestPoint(const Point2D& position);
    
    /**
     * @brief Interpolate between two trajectory points
     */
    TrajectoryPoint interpolate(size_t idx1, size_t idx2, double t) const;

    /**
     * @brief Whether current trajectory should be treated as a closed loop.
     */
    bool isTrajectoryClosed() const;
};

}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_PURE_PURSUIT_HPP_
