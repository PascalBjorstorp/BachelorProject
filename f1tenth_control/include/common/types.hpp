#ifndef F1TENTH_CONTROL_TYPES_HPP_
#define F1TENTH_CONTROL_TYPES_HPP_

/**
 * @file types.hpp
 * @brief Shared geometric, sensor, and trajectory data types for f1tenth_control.
 * @details Provides the canonical type vocabulary consumed by all algorithms,
 *          nodes, and utilities in the package. All types are plain data structures
 *          with no dependencies on ROS or external libraries. Numeric constants are
 *          in the nested ::constants namespace.
 */

#include <vector>
#include <cmath>
#include <limits>

namespace f1tenth_control {

/**
 * @brief 2D Point structure
 */
struct Point2D {
    double x{0.0};  // X coordinate
    double y{0.0};  // Y coordinate

    Point2D() = default;
    
    /**
     * @brief Construct a 2D point.
     * @param x_ X coordinate.
     * @param y_ Y coordinate.
     */
    Point2D(double x_, double y_) : x(x_), y(y_) {}
    
    /**
     * @brief Compute Euclidean norm of the point/vector.
     * @return Euclidean norm.
     */
    double norm() const { return std::sqrt(x * x + y * y); }

    /**
     * @brief Compute dot product with another Point2D.
     * @param other Another Point2D to compute the dot product with.
     * @return Scalar dot product value.
     */
    double dot(const Point2D& other) const { return x * other.x + y * other.y; }
    
    /**
     * @brief Add another Point2D to this point.
     * @param other Another Point2D to add.
     * @return Summed point.
     */
    Point2D operator+(const Point2D& other) const { return {x + other.x, y + other.y}; }

    /**
     * @brief Subtract another Point2D from this point.
     * @param other Another Point2D to subtract.
     * @return Difference point.
     */
    Point2D operator-(const Point2D& other) const { return {x - other.x, y - other.y}; }

    /**
     * @brief Scale this point by a scalar factor.
     * This holds true for scalar > 0. Negative scalars reflects the vector.
     * @param scalar Multiplicative scale factor.
     * @return Scaled point.
     */
    Point2D operator*(double scalar) const { return {x * scalar, y * scalar}; }
};

/**
 * @brief Pose in 2D space (x, y, theta)
 */
struct Pose2D {
    double x{0.0};      // X coordinate
    double y{0.0};      // Y coordinate
    double theta{0.0};  // Heading in radians

    Pose2D() = default;
    
    /**
     * @brief Construct a 2D pose.
     * @param x_ X coordinate.
     * @param y_ Y coordinate.
     * @param theta_ Heading in radians.
     */
    Pose2D(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}
    
    /**
     * @brief Access translational component of pose without heading.
     * @return Point2D containing {x, y}.
     */
    Point2D position() const { return {x, y}; }
};

/**
 * @brief Vehicle state including velocity and steering angle
 */
struct VehicleState {
    Pose2D pose;                    // Vehicle pose (position + heading)
    double velocity{0.0};           // Linear velocity (m/s)
    double angular_velocity{0.0};   // Angular velocity (rad/s)
    double steering_angle{0.0};     // Current steering angle (rad)
};

/**
 * @brief Drive command output
 */
struct DriveCommand {
    double speed{0.0};          // Target speed (m/s)
    double steering_angle{0.0}; // Target steering angle (rad)

    DriveCommand() = default;
    
    /**
     * @brief Construct a drive command.
     * @param speed_ Target speed in m/s.
     * @param angle_ Target steering angle in radians.
     * @return DriveCommand with specified speed and steering angle.
     */
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

    /**
     * @brief Construct a polar point.
     * @param r Range measurement in meters.
     * @param a Angle measurement in radians.
     * @param v Validity of the measurement (default true).
     * @return PolarPoint with specified range, angle, and validity.
     */
    PolarPoint(double r, double a, bool v = true) : range(r), angle(a), valid(v) {}
    
    /**
     * @brief Convert polar point to Cartesian coordinates.
     * @return Point2D in Cartesian coordinates.
     */
    Point2D toCartesian() const {
        return {range * std::cos(angle), range * std::sin(angle)};
    }
};

/**
 * @brief Represents a gap in LiDAR scan
 */
struct Gap {
    size_t start_idx{0};        // Start index in scan
    size_t end_idx{0};          // End index in scan
    double start_angle{0.0};    // Start angle (rad)
    double end_angle{0.0};      // End angle (rad)
    double min_range{0.0};      // Minimum range within gap
    double max_range{0.0};      // Maximum range within gap
    double avg_range{0.0};      // Average range within gap
    size_t deepest_idx{0};      // Index of deepest point
    double deepest_range{0.0};  // Range at deepest point
    double angular_width{0.0};  // Angular width (rad)
    
    /**
     * @brief Validate that gap indices and width represent a usable segment.
     * @return True if the gap is valid and can be used for navigation.
     */
    bool isValid() const { return end_idx > start_idx && angular_width > 0.0; }
    
    /**
     * @brief Compute center angle of the gap.
     * @return Center angle in radians.
     */
    double centerAngle() const { return (start_angle + end_angle) / 2.0; }
};

/**
 * @brief Processed LiDAR scan data
 */
struct ProcessedScan {
    std::vector<double> ranges{};             // Original ranges
    std::vector<double> filtered_ranges;    // After filtering
    std::vector<double> angles;             // Corresponding angles
    std::vector<bool> valid;                // Validity mask
    std::vector<bool> disparity_blocked;    // Points blocked by disparity extension
                                            // Disparity blocked points are points that are invalidated 
                                            // due to being within the "shadow" of a closer obstacle, 
                                            // as determined by the disparity extension algorithm.

    std::vector<bool> bubble_blocked;       // Points blocked by safety bubble
                                            // Bubble blocked points are points that are invalidated 
                                            // because they fall within a safety "bubble" around the vehicle, 
                                            // which is used to enforce a minimum clearance from obstacles.
                                            
    double angle_min{0.0};                  // Minimum angle of the scan
    double angle_max{0.0};                  // Maximum angle of the scan
    double angle_increment{0.0};            // Angular increment between beams
    double range_min{0.0};                  // Minimum valid range
    double range_max{0.0};                  // Maximum valid range
    
    /**
    * @brief Get the number of beams in the scan.
    * @return Number of range measurements in the scan.
    */
    size_t size() const { return ranges.size(); }
};

/**
 * @brief Track boundary point for mapping
 */
struct BoundaryPoint {
    Point2D position = Point2D(0.0, 0.0);   // Position in map frame
    double timestamp{0.0};                  // When it was recorded
    bool is_left_wall{false};               // Left or right track boundary
    double confidence{1.0};                 // Measurement confidence
};

/**
 * @brief Trajectory waypoint for path following (shared by Pure Pursuit and Stanley)
 */
struct TrajectoryPoint {
    double x{0.0};                                                  // [m] X position
    double y{0.0};                                                  // [m] Y position
    double heading{0.0};                                            // [rad] Heading angle
    double velocity{0.0};                                           // [m/s] Target velocity
    double curvature{0.0};                                          // [1/m] Path curvature
    double arc_length{0.0};                                         // [m] Distance along path
    double left_bound{std::numeric_limits<double>::infinity()};     // [m] Left corridor half-width
    double right_bound{std::numeric_limits<double>::infinity()};    // [m] Right corridor half-width

    TrajectoryPoint() = default;
    
    /**
     * @brief Construct trajectory waypoint with full geometric and speed metadata.
     * @param x_ X position
     * @param y_ Y position
     * @param h Heading angle
     * @param v Target velocity
     * @param k Path curvature
     * @param s Arc length from path start
     * @param l Left corridor half-width limit
     * @param r Right corridor half-width limit
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
