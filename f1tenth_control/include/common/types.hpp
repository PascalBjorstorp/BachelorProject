#ifndef F1TENTH_CONTROL_TYPES_HPP_
#define F1TENTH_CONTROL_TYPES_HPP_

#include <vector>
#include <cmath>
#include <limits>

namespace f1tenth_control {

/**
 * @brief 2D Point structure for efficient computation
 */
struct Point2D {
    double x{0.0};
    double y{0.0};
    
    Point2D() = default;
    Point2D(double x_, double y_) : x(x_), y(y_) {}
    
    /**
     * Inputs:
     * - None.
     *
     * Purpose:
     * - Compute Euclidean magnitude of the 2D vector.
     *
     * Outputs:
     * - Returns vector norm in coordinate units.
     */
    double norm() const { return std::sqrt(x * x + y * y); }

    /**
     * Inputs:
     * - other: Second point/vector.
     *
     * Purpose:
     * - Compute dot product for projection/alignment calculations.
     *
     * Outputs:
     * - Returns scalar dot product.
     */
    double dot(const Point2D& other) const { return x * other.x + y * other.y; }
    
    /**
     * Inputs:
     * - other: Point/vector to add.
     *
     * Purpose:
     * - Translate by vector addition.
     *
     * Outputs:
     * - Returns summed point/vector.
     */
    Point2D operator+(const Point2D& other) const { return {x + other.x, y + other.y}; }

    /**
     * Inputs:
     * - other: Point/vector to subtract.
     *
     * Purpose:
     * - Compute relative displacement.
     *
     * Outputs:
     * - Returns difference vector.
     */
    Point2D operator-(const Point2D& other) const { return {x - other.x, y - other.y}; }

    /**
     * Inputs:
     * - scalar: Multiplicative scale factor.
     *
     * Purpose:
     * - Scale vector magnitude while preserving direction.
     *
     * Outputs:
     * - Returns scaled vector.
     */
    Point2D operator*(double scalar) const { return {x * scalar, y * scalar}; }
};

/**
 * @brief Pose in 2D space (x, y, theta)
 */
struct Pose2D {
    double x{0.0};
    double y{0.0};
    double theta{0.0};  // Heading in radians
    
    Pose2D() = default;
    Pose2D(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}
    
    /**
     * Inputs:
     * - None.
     *
     * Purpose:
     * - Access translational component of pose without heading.
     *
     * Outputs:
     * - Returns Point2D containing {x, y}.
     */
    Point2D position() const { return {x, y}; }
};

/**
 * @brief Vehicle state including velocity
 */
struct VehicleState {
    Pose2D pose;
    double velocity{0.0};       // Linear velocity (m/s)
    double angular_velocity{0.0}; // Angular velocity (rad/s)
    double steering_angle{0.0}; // Current steering angle (rad)
};

/**
 * @brief Drive command output
 */
struct DriveCommand {
    double speed{0.0};          // Target speed (m/s)
    double steering_angle{0.0}; // Target steering angle (rad)
    
    DriveCommand() = default;
    DriveCommand(double speed_, double angle_) : speed(speed_), steering_angle(angle_) {}
};

/**
 * @brief Polar point from LiDAR
 */
struct PolarPoint {
    double range{0.0};    // Distance in meters
    double angle{0.0};    // Angle in radians
    bool valid{true};     // Whether the measurement is valid
    
    PolarPoint() = default;
    PolarPoint(double r, double a, bool v = true) : range(r), angle(a), valid(v) {}
    
    /**
     * Inputs:
     * - None.
     *
     * Purpose:
     * - Convert polar measurement into Cartesian robot-frame point.
     *
     * Outputs:
     * - Returns Point2D {x, y} corresponding to {range, angle}.
     */
    Point2D toCartesian() const {
        return {range * std::cos(angle), range * std::sin(angle)};
    }
};

/**
 * @brief Represents a gap in LiDAR scan
 */
struct Gap {
    size_t start_idx{0};      // Start index in scan
    size_t end_idx{0};        // End index in scan
    double start_angle{0.0};  // Start angle (rad)
    double end_angle{0.0};    // End angle (rad)
    double min_range{0.0};    // Minimum range within gap
    double max_range{0.0};    // Maximum range within gap
    double avg_range{0.0};    // Average range within gap
    size_t deepest_idx{0};    // Index of deepest point
    double deepest_range{0.0}; // Range at deepest point
    double angular_width{0.0}; // Angular width (rad)
    
    /**
     * Inputs:
     * - None.
     *
     * Purpose:
     * - Validate that a gap spans a non-empty angular interval.
     *
     * Outputs:
     * - Returns true when gap indices and width represent a usable segment.
     */
    bool isValid() const { return end_idx > start_idx && angular_width > 0.0; }
    
    /**
     * Inputs:
     * - None.
     *
     * Purpose:
     * - Compute midpoint heading of the gap interval.
     *
     * Outputs:
     * - Returns center angle in radians.
     */
    double centerAngle() const { return (start_angle + end_angle) / 2.0; }
};

/**
 * @brief Processed LiDAR scan data
 */
struct ProcessedScan {
    std::vector<double> ranges;           // Original ranges
    std::vector<double> filtered_ranges;  // After filtering
    std::vector<double> angles;           // Corresponding angles
    std::vector<bool> valid;              // Validity mask
    std::vector<bool> disparity_blocked;  // Points blocked by disparity extension
    std::vector<bool> bubble_blocked;     // Points blocked by safety bubble
    double angle_min{0.0};
    double angle_max{0.0};
    double angle_increment{0.0};
    double range_min{0.0};
    double range_max{0.0};
    
    /**
     * Inputs:
     * - None.
     *
     * Purpose:
     * - Report number of scan samples represented in this container.
     *
     * Outputs:
     * - Returns sample count from the canonical ranges array.
     */
    size_t size() const { return ranges.size(); }
};

/**
 * @brief Track boundary point for mapping
 */
struct BoundaryPoint {
    Point2D position;         // Position in map frame
    double timestamp{0.0};    // When it was recorded
    bool is_left_wall{false}; // Left or right track boundary
    double confidence{1.0};   // Measurement confidence
};

/**
 * @brief Trajectory waypoint for path following (shared by Pure Pursuit and Stanley)
 */
struct TrajectoryPoint {
    double x{0.0};           // [m] X position
    double y{0.0};           // [m] Y position
    double heading{0.0};     // [rad] Heading angle
    double velocity{0.0};    // [m/s] Target velocity
    double curvature{0.0};   // [1/m] Path curvature
    double arc_length{0.0};  // [m] Distance along path
    double left_bound{std::numeric_limits<double>::infinity()};   // [m] Left corridor half-width
    double right_bound{std::numeric_limits<double>::infinity()};  // [m] Right corridor half-width
    
    TrajectoryPoint() = default;

    /**
     * Inputs:
     * - x_, y_: Waypoint position.
     * - h: Waypoint heading.
     * - v: Target velocity.
     * - k: Path curvature.
     * - s: Arc length from path start.
     * - l: Left corridor half-width limit.
     * - r: Right corridor half-width limit.
     *
     * Purpose:
     * - Construct trajectory waypoint with full geometric and speed metadata.
     *
     * Outputs:
     * - Initializes TrajectoryPoint fields for path-tracking controllers.
     */
    TrajectoryPoint(double x_, double y_, double h, double v, double k, double s,
                    double l = std::numeric_limits<double>::infinity(),
                    double r = std::numeric_limits<double>::infinity())
        : x(x_), y(y_), heading(h), velocity(v), curvature(k), arc_length(s),
          left_bound(l), right_bound(r) {}
};

/**
 * @brief Constants for the algorithms
 */
namespace constants {
    constexpr double PI = 3.14159265358979323846;
    constexpr double TWO_PI = 2.0 * PI;
    constexpr double DEG_TO_RAD = PI / 180.0;
    constexpr double RAD_TO_DEG = 180.0 / PI;
    constexpr double INF = std::numeric_limits<double>::infinity();
    constexpr double EPSILON = 1e-9;
}

}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_TYPES_HPP_
