#ifndef F1TENTH_CONTROL_MATH_UTILS_HPP_
#define F1TENTH_CONTROL_MATH_UTILS_HPP_

#include "f1tenth_control/common/types.hpp"
#include <cmath>
#include <vector>

namespace f1tenth_control {
namespace math {

// =============================================
// Core math utilities for FTG algorithm
// Only includes functions actively used in production
// =============================================

/**
 * Inputs:
 * - angle: Raw angle value in radians.
 *
 * Purpose:
 * - Wrap angular values into canonical interval for stable control arithmetic.
 *
 * Outputs:
 * - Returns normalized angle in [-PI, PI).
 */
inline double normalizeAngle(double angle) {
    angle = std::fmod(angle + constants::PI, constants::TWO_PI);
    return angle < 0 ? angle + constants::PI : angle - constants::PI;
}

/**
 * Inputs:
 * - value: Candidate value to constrain.
 * - min_val: Lower bound.
 * - max_val: Upper bound.
 *
 * Purpose:
 * - Enforce inclusive bounds on scalar values.
 *
 * Outputs:
 * - Returns value clipped to [min_val, max_val].
 */
template<typename T>
inline constexpr T clamp(T value, T min_val, T max_val) {
    return (value < min_val) ? min_val : (value > max_val) ? max_val : value;
}

/**
 * Inputs:
 * - a: Start value.
 * - b: End value.
 * - t: Interpolation factor.
 *
 * Purpose:
 * - Compute linear interpolation/extrapolation between two scalar values.
 *
 * Outputs:
 * - Returns a + t * (b - a).
 */
inline constexpr double lerp(double a, double b, double t) {
    return a + t * (b - a);
}

/**
 * Inputs:
 * - a: First 2D point.
 * - b: Second 2D point.
 *
 * Purpose:
 * - Compute Euclidean separation between two points.
 *
 * Outputs:
 * - Returns distance in same length units as point coordinates.
 */
inline double distance(const Point2D& a, const Point2D& b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * Inputs:
 * - x1, y1: Coordinates of first point.
 * - x2, y2: Coordinates of second point.
 *
 * Purpose:
 * - Compute Euclidean separation without constructing Point2D objects.
 *
 * Outputs:
 * - Returns distance in same units as input coordinates.
 */
inline double distance(double x1, double y1, double x2, double y2) {
    return std::hypot(x2 - x1, y2 - y1);
}

/**
 * Inputs:
 * - local_point: Point in robot-local frame.
 * - robot_pose: Robot pose in global frame.
 *
 * Purpose:
 * - Apply rigid-body transform from local robot coordinates to global/map frame.
 *
 * Outputs:
 * - Returns transformed global-frame point.
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
 * Inputs:
 * - data: Input scalar sequence.
 * - window_size: Sliding window width used for median filtering.
 *
 * Purpose:
 * - Reduce impulsive noise while preserving edge-like structure in sampled data.
 *
 * Outputs:
 * - Returns filtered sequence with same length as input.
 */
std::vector<double> medianFilter(const std::vector<double>& data, size_t window_size);

}  // namespace math
}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_MATH_UTILS_HPP_
