#ifndef F1TENTH_CONTROL_MATH_UTILS_HPP_
#define F1TENTH_CONTROL_MATH_UTILS_HPP_

#include "f1tenth_control/common/types.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace f1tenth_control {
namespace math {

// =============================================
// Core math utilities for FTG algorithm
// Only includes functions actively used in production
// =============================================

/**
 * @brief Normalize angle to [-PI, PI] using efficient fmod
 */
inline double normalizeAngle(double angle) {
    angle = std::fmod(angle + constants::PI, constants::TWO_PI);
    return angle < 0 ? angle + constants::PI : angle - constants::PI;
}

/**
 * @brief Clamp value between min and max
 * Note: C++17 has std::clamp, but this avoids potential issues with argument order
 */
template<typename T>
inline constexpr T clamp(T value, T min_val, T max_val) {
    return (value < min_val) ? min_val : (value > max_val) ? max_val : value;
}

/**
 * @brief Linear interpolation: a + t*(b-a)
 */
inline constexpr double lerp(double a, double b, double t) {
    return a + t * (b - a);
}

/**
 * @brief Euclidean distance between two points
 */
inline double distance(const Point2D& a, const Point2D& b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * @brief Euclidean distance between two points given coordinates
 */
inline double distance(double x1, double y1, double x2, double y2) {
    return std::hypot(x2 - x1, y2 - y1);
}

/**
 * @brief Transform point from local (robot) frame to global frame
 * Used for mapping mode boundary point extraction
 */
inline Point2D localToGlobal(const Point2D& local_point, const Pose2D& robot_pose) {
    const double cos_theta = std::cos(robot_pose.theta);
    const double sin_theta = std::sin(robot_pose.theta);
    return {
        robot_pose.x + local_point.x * cos_theta - local_point.y * sin_theta,
        robot_pose.y + local_point.x * sin_theta + local_point.y * cos_theta
    };
}

/**
 * @brief Median filter for LiDAR noise reduction
 * Uses nth_element for O(n) median finding per window
 */
std::vector<double> medianFilter(const std::vector<double>& data, size_t window_size);

}  // namespace math
}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_MATH_UTILS_HPP_
