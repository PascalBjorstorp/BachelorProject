#ifndef F1TENTH_CONTROL_STANLEY_HPP_
#define F1TENTH_CONTROL_STANLEY_HPP_

#include "f1tenth_control/common/types.hpp"
#include <vector>
#include <string>
#include <cmath>

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
};

/**
 * @brief Trajectory waypoint for path following
 */
struct StanleyTrajectoryPoint {
    double x{0.0};           // [m] X position
    double y{0.0};           // [m] Y position
    double heading{0.0};     // [rad] Heading angle (tangent to path)
    double velocity{0.0};    // [m/s] Target velocity
    double curvature{0.0};   // [1/m] Path curvature
    double arc_length{0.0};  // [m] Distance along path
};

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
    Stanley();
    explicit Stanley(const StanleyConfig& config);
    
    /**
     * @brief Load trajectory from CSV file (TUM format)
     */
    bool loadTrajectory(const std::string& csv_path);
    
    /**
     * @brief Set trajectory directly
     */
    void setTrajectory(const std::vector<StanleyTrajectoryPoint>& trajectory);
    
    /**
     * @brief Compute steering and speed commands
     * @param state Current vehicle state (assumes rear axle position)
     * @return Control output
     */
    StanleyOutput compute(const VehicleState& state);
    
    /**
     * @brief Update configuration
     */
    void setConfig(const StanleyConfig& config) { config_ = config; }
    const StanleyConfig& getConfig() const { return config_; }
    
    /**
     * @brief Get loaded trajectory
     */
    const std::vector<StanleyTrajectoryPoint>& getTrajectory() const { return trajectory_; }
    
    /**
     * @brief Check if trajectory is loaded
     */
    bool hasTrajectory() const { return !trajectory_.empty(); }
    
    /**
     * @brief Get total trajectory length
     */
    double getTrajectoryLength() const;
    
private:
    StanleyConfig config_;
    std::vector<StanleyTrajectoryPoint> trajectory_;
    size_t last_closest_idx_{0};
    double last_steering_{0.0};     // For steering rate limiting
    double last_time_{-1.0};        // Last compute time
    
    /**
     * @brief Find closest point to front axle position (heading-aware)
     * Uses both position and heading to find the correct track segment
     */
    size_t findClosestPoint(const Point2D& front_axle_pos, double vehicle_heading);
    
    /**
     * @brief Compute cross-track error (signed)
     * Positive = vehicle is to the left of path
     * Negative = vehicle is to the right of path
     */
    double computeCrossTrackError(const Point2D& front_axle_pos, size_t closest_idx);
    
    /**
     * @brief Normalize angle to [-pi, pi]
     */
    static double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    /**
     * @brief Distance helper
     */
    static double distance(double x1, double y1, double x2, double y2) {
        return std::hypot(x2 - x1, y2 - y1);
    }
};

}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_STANLEY_HPP_
