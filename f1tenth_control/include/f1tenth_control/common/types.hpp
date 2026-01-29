#ifndef F1TENTH_CONTROL_TYPES_HPP_
#define F1TENTH_CONTROL_TYPES_HPP_

#include <cstdint>
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
    
    double norm() const { return std::sqrt(x * x + y * y); }
    double dot(const Point2D& other) const { return x * other.x + y * other.y; }
    
    Point2D operator+(const Point2D& other) const { return {x + other.x, y + other.y}; }
    Point2D operator-(const Point2D& other) const { return {x - other.x, y - other.y}; }
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
    
    // Convert to Cartesian
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
    
    bool isValid() const { return end_idx > start_idx && angular_width > 0.0; }
    
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
    double angle_min{0.0};
    double angle_max{0.0};
    double angle_increment{0.0};
    double range_min{0.0};
    double range_max{0.0};
    
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
