#ifndef F1TENTH_CONTROL_PURE_PURSUIT_HPP_
#define F1TENTH_CONTROL_PURE_PURSUIT_HPP_

#include "f1tenth_control/common/types.hpp"
#include <vector>
#include <string>

namespace f1tenth_control {

/**
 * @brief Configuration for Pure Pursuit controller.
 *
 * Tunables are grouped by lookahead shaping, speed regulation, vehicle
 * geometry, and path-tracking behavior.
 */
struct PurePursuitConfig {
    // -- Lookahead shaping ---------------------------------------------------
    double min_lookahead{0.48};                 // [m] Minimum lookahead distance.
    double max_lookahead{1.20};                 // [m] Maximum lookahead distance.
    double lookahead_gain{0.15};                // [m/s] Velocity-proportional lookahead gain.
    double cte_lookahead_weight{1.0};           // [unitless] Weight on |CTE| contribution.
    double cte_lookahead_gain{0.05};            // [m/m] Reduce lookahead with cross-track error.
    double curvature_lookahead_gain{1.34};      // [m*m] Turn-radius-based lookahead limit.
    
    // -- Speed control ------------------------------------------------------
    double curvature_speed_factor{0.10};        // [unitless] Curvature slowdown aggressiveness.
    double curvature_speed_floor_ratio{0.43};   // [0..1] Minimum speed ratio after curvature slowdown.
    double cte_speed_factor{0.10};              // [unitless] Slowdown gain based on |CTE|.
    double cte_speed_floor_ratio{0.50};         // [0..1] Minimum speed ratio from CTE slowdown.
    double max_lateral_accel{7.27};             // [m/s^2] Physics-aware cornering speed cap.
    double min_regulated_speed{0.30};           // [m/s] Lower bound after speed regulation.
    double curvature_preview_factor{1.2};       // [unitless] Preview multiple for curvature braking.

    // -- Footprint-aware corridor regulation --------------------------------
    double vehicle_half_width{0.1365};          // [m] Half of the vehicle width.
    double wall_safety_margin{0.03};            // [m] Static wall clearance margin.
    double corridor_half_width_ref{0.25};       // [m] Reference usable half-width for full speed.
    double corridor_speed_floor_ratio{0.20};    // [0..1] Floor for corridor-based speed scaling.
    double corridor_lookahead_factor{2.0};      // [m/m] Extra lookahead per usable half-width.

    // -- Steering limits ----------------------------------------------------
    double max_steering{0.4189};                // [rad] Maximum steering angle (~24 deg).

    // -- Vehicle parameters -------------------------------------------------
    double wheelbase{0.324};                    // [m] Distance between axles.

    // -- Path tracking ------------------------------------------------------
    double position_tolerance{0.5};             // [m] Max deviation before re-finding closest point.
};

/**
 * @brief Output from Pure Pursuit controller.
 */
struct PurePursuitOutput {
    double steering_angle{0.0};         // [rad] Commanded steering angle.
    double target_speed{0.0};           // [m/s] Commanded speed.

    size_t closest_idx{0};              // Index of closest waypoint.
    size_t target_idx{0};               // Index of lookahead target.
    Point2D target_point;               // Lookahead point in world frame.
    double lookahead_distance{0.0};     // [m] Selected lookahead distance.
    double cross_track_error{0.0};      // [m] Lateral error from path.
    bool valid{false};                  // Whether the output is valid.
};

/**
 * @brief Pure Pursuit path-following controller.
 *
 * The controller selects a lookahead target on the active trajectory and
 * computes steering to arc toward that point. Speed is then regulated using
 * curvature, cross-track error, and corridor-aware limits.
 * @details
 * The steering law follows the standard bicycle-model form:
 * delta = atan2(2 * L * sin(alpha), lookahead_dist)
 * where L is the wheelbase and alpha is the target angle in the vehicle frame.
 */
class PurePursuit {
public:
    /**
     * @brief Construct a controller instance with caller-provided tuning values.
     * @param config Initial controller tuning parameters.
     * @return None.
     */
    explicit PurePursuit(const PurePursuitConfig& config);
    
    /**
     * @brief Load and parse an external trajectory for tracking.
     * @param csv_path Path to a trajectory CSV file in expected input format.
     * @return True when parsing succeeds and internal trajectory storage is updated.
     */
    bool loadTrajectory(const std::string& csv_path);
    
    /**
     * @brief Replace the internal reference path from in-memory data.
     * @param trajectory Ordered trajectory waypoints in world coordinates.
     * @return None.
     */
    void setTrajectory(const std::vector<TrajectoryPoint>& trajectory);
    
    /**
     * @brief Compute steering and speed commands for the current vehicle state.
     * @param state Vehicle pose and motion state.
     * @return PurePursuitOutput with command values and diagnostic metadata.
     */
    PurePursuitOutput compute(const VehicleState& state);
    
    /**
     * @brief Update configuration parameters at runtime.
     * @param config New controller tuning parameters.
     * @return None.
     */
    void setConfig(const PurePursuitConfig& config) { config_ = config; }

    /**
     * @brief Expose current tuning values for diagnostics and runtime introspection.
     * @return Const reference to active PurePursuitConfig.
     */
    const PurePursuitConfig& getConfig() const { return config_; }
    
    /**
     * @brief Provide read-only access to the currently loaded trajectory.
     * @return Const reference to stored trajectory waypoints.
     */
    const std::vector<TrajectoryPoint>& getTrajectory() const { return trajectory_; }
    
    /**
     * @brief Indicate whether path tracking can be performed.
     * @return True when at least one waypoint is available.
     */
    bool hasTrajectory() const { return !trajectory_.empty(); }
    
    /**
     * @brief Compute geometric path length for diagnostics and planning logic.
     * @return Cumulative trajectory length in meters.
     */
    double getTrajectoryLength() const;
    
private:
    PurePursuitConfig config_;
    std::vector<TrajectoryPoint> trajectory_;
    size_t last_closest_idx_{0};   // Search anchor for closest-point lookup.
    double current_heading_{0.0};  // Current vehicle heading for search gating.
    
    /**
     * @brief Find nearest waypoint index to anchor lookahead target selection.
     * @param position Vehicle position in world coordinates.
     * @return Index of the selected closest trajectory point.
     */
    size_t findClosestPoint(const Point2D& position);
    
    /**
     * @brief Create a continuous target point along a trajectory segment.
     * @param idx1 Index of first trajectory point.
     * @param idx2 Index of second trajectory point.
     * @param t Interpolation factor between the two points.
     * @return Interpolated TrajectoryPoint in world coordinates.
     */
    TrajectoryPoint interpolate(size_t idx1, size_t idx2, double t) const;

    /**
     * @brief Determine whether index wrapping logic should be applied for target search.
     * @return True when trajectory geometry indicates closed-loop tracking.
     */
    bool isTrajectoryClosed() const;
};

}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_PURE_PURSUIT_HPP_
