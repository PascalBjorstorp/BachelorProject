#ifndef F1TENTH_CONTROL_MATH_UTILS_HPP_
#define F1TENTH_CONTROL_MATH_UTILS_HPP_

#include "f1tenth_control/common/types.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace f1tenth_control {
namespace math {

/**
 * @brief Normalize angle to [-PI, PI]
 * @param angle Input angle in radians
 * @return Normalized angle
 */
inline double normalizeAngle(double angle) {
    while (angle > constants::PI) angle -= constants::TWO_PI;
    while (angle < -constants::PI) angle += constants::TWO_PI;
    return angle;
}

/**
 * @brief Normalize angle to [0, 2*PI]
 * @param angle Input angle in radians
 * @return Normalized angle
 */
inline double normalizeAnglePositive(double angle) {
    while (angle >= constants::TWO_PI) angle -= constants::TWO_PI;
    while (angle < 0.0) angle += constants::TWO_PI;
    return angle;
}

/**
 * @brief Clamp value between min and max
 */
template<typename T>
inline T clamp(T value, T min_val, T max_val) {
    return std::max(min_val, std::min(value, max_val));
}

/**
 * @brief Linear interpolation
 */
inline double lerp(double a, double b, double t) {
    return a + t * (b - a);
}

/**
 * @brief Map value from one range to another
 */
inline double mapRange(double value, double in_min, double in_max, double out_min, double out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief Euclidean distance between two points
 */
inline double distance(const Point2D& a, const Point2D& b) {
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * @brief Distance from pose to point
 */
inline double distance(const Pose2D& pose, const Point2D& point) {
    return distance(pose.position(), point);
}

/**
 * @brief Angle from one point to another
 */
inline double angleTo(const Point2D& from, const Point2D& to) {
    return std::atan2(to.y - from.y, to.x - from.x);
}

/**
 * @brief Angle from pose to point
 */
inline double angleTo(const Pose2D& from, const Point2D& to) {
    return std::atan2(to.y - from.y, to.x - from.x);
}

/**
 * @brief Transform point from local frame to global frame
 * @param local_point Point in local (robot) frame
 * @param robot_pose Robot pose in global frame
 * @return Point in global frame
 */
inline Point2D localToGlobal(const Point2D& local_point, const Pose2D& robot_pose) {
    double cos_theta = std::cos(robot_pose.theta);
    double sin_theta = std::sin(robot_pose.theta);
    return {
        robot_pose.x + local_point.x * cos_theta - local_point.y * sin_theta,
        robot_pose.y + local_point.x * sin_theta + local_point.y * cos_theta
    };
}

/**
 * @brief Transform point from global frame to local frame
 * @param global_point Point in global frame
 * @param robot_pose Robot pose in global frame
 * @return Point in local (robot) frame
 */
inline Point2D globalToLocal(const Point2D& global_point, const Pose2D& robot_pose) {
    double dx = global_point.x - robot_pose.x;
    double dy = global_point.y - robot_pose.y;
    double cos_theta = std::cos(robot_pose.theta);
    double sin_theta = std::sin(robot_pose.theta);
    return {
        dx * cos_theta + dy * sin_theta,
        -dx * sin_theta + dy * cos_theta
    };
}

/**
 * @brief Sign function
 */
template<typename T>
inline int sign(T value) {
    return (T(0) < value) - (value < T(0));
}

/**
 * @brief Moving average filter (in-place)
 * @param data Input/output data vector
 * @param window_size Size of moving average window (should be odd)
 */
void movingAverageFilter(std::vector<double>& data, size_t window_size);

/**
 * @brief Median filter (returns new vector)
 * @param data Input data vector
 * @param window_size Size of median window (should be odd)
 * @return Filtered data
 */
std::vector<double> medianFilter(const std::vector<double>& data, size_t window_size);

/**
 * @brief Find the index of minimum value in range
 */
template<typename T>
inline size_t argmin(const std::vector<T>& vec, size_t start = 0, size_t end = 0) {
    if (end == 0) end = vec.size();
    auto it = std::min_element(vec.begin() + start, vec.begin() + end);
    return std::distance(vec.begin(), it);
}

/**
 * @brief Find the index of maximum value in range
 */
template<typename T>
inline size_t argmax(const std::vector<T>& vec, size_t start = 0, size_t end = 0) {
    if (end == 0) end = vec.size();
    auto it = std::max_element(vec.begin() + start, vec.begin() + end);
    return std::distance(vec.begin(), it);
}

/**
 * @brief Calculate curvature given three points
 * Uses Menger curvature formula
 */
double curvature(const Point2D& p1, const Point2D& p2, const Point2D& p3);

/**
 * @brief Calculate steering angle for pure pursuit
 * @param lookahead_distance Distance to lookahead point
 * @param lateral_error Lateral offset to lookahead point
 * @param wheelbase Vehicle wheelbase
 * @return Steering angle in radians
 */
inline double purePursuitSteering(double lookahead_distance, double lateral_error, double wheelbase) {
    // Curvature = 2 * y / L^2
    // Steering angle = atan(curvature * wheelbase)
    double curvature = 2.0 * lateral_error / (lookahead_distance * lookahead_distance);
    return std::atan(curvature * wheelbase);
}

}  // namespace math
}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_MATH_UTILS_HPP_
