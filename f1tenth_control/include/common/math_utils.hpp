#ifndef F1TENTH_CONTROL_MATH_UTILS_HPP_
#define F1TENTH_CONTROL_MATH_UTILS_HPP_

/**
 * @file math_utils.hpp
 * @brief Inline mathematical utilities shared across f1tenth_control algorithms.
 * @details Covers angle normalization, clamping, interpolation, distance computation,
 *          rigid-body frame transforms, and sliding-window median filtering.
 *          All functions are pure (no side effects). Median filter declaration is
 *          here; implementation is in math_utils.cpp.
 * @dependencies types.hpp (Point2D, Pose2D), <cmath>, <vector>
 * @assumptions All angles are in radians. All positions are in meters.
 *              Caller is responsible for ensuring non-NaN/Inf inputs where noted.
 */

#include "common/types.hpp"
#include <cmath>
#include <vector>

namespace f1tenth_control {
namespace math {

// =============================================
// Core math utilities
// Only includes functions actively used in production
// =============================================

/**
 * @brief Normalize angle to [-PI, PI).
 * @param angle Raw angle value in radians.
 * @return Normalized angle in [-PI, PI).
 */
inline double normalizeAngle(double angle) {
    angle = std::fmod(angle + constants::PI, constants::TWO_PI);
    return angle < 0 ? angle + constants::PI : angle - constants::PI;
}

/**
 * @brief Linearly interpolate between two values.
 * @param a Start value.
 * @param b End value.
 * @param t Interpolation factor.
 * @return Interpolated value.
 */
inline constexpr double lerp(double a, double b, double t) {
    return a + t * (b - a);
}

/**
 * @brief Compute Euclidean separation between two points.
 * @param a First 2D point.
 * @param b Second 2D point.
 * @return Distance in same length units as point coordinates.
 */
inline double distance(const Point2D& a, const Point2D& b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * @brief Compute Euclidean separation without constructing Point2D objects.
 * @param x1, y1 Coordinates of first point.
 * @param x2, y2 Coordinates of second point.
 * @return Distance in same units as input coordinates.
 */
inline double distance(double x1, double y1, double x2, double y2) {
    return std::hypot(x2 - x1, y2 - y1);
}

/**
 * @brief Apply rigid-body transform from local robot coordinates to global/map frame.
 * @param local_point Point in robot-local frame.
 * @param robot_pose Robot pose in global frame.
 * @return Transformed global-frame point.
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
 * @brief Reduce impulsive noise while preserving edge-like structure in sampled data.
 * @param data Input scalar sequence.
 * @param window_size Sliding window width used for median filtering.
 * @return Filtered sequence with same length as input.
 */
std::vector<double> medianFilter(const std::vector<double>& data, size_t window_size);

}  // namespace math
}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_MATH_UTILS_HPP_
