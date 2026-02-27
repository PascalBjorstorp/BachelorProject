#pragma once

#include <cmath>
#include <array>
#include <Eigen/Core>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

namespace gpu_amcl_cpp {

/**
 * @brief Shared math utilities for localization.
 *
 * All functions are stateless and work on standard types (doubles, Eigen).
 * Used by AMCL, Odom and EKF nodes.
 */
namespace math_utils {

/// Wrap angle to [-pi, pi].
inline double normalize_angle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

/// Shortest signed difference: normalize(a - b).
inline double angle_diff(double a, double b) {
    return normalize_angle(a - b);
}

/// Convert quaternion to yaw (Z-axis rotation only).
inline double quaternion_to_yaw(const geometry_msgs::msg::Quaternion& q) {
    // yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

/// Convert yaw to quaternion (rotation about Z only).
inline geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw) {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw / 2.0);
    q.w = std::cos(yaw / 2.0);
    return q;
}

/// Extract (x, y, yaw) from a geometry_msgs::Pose.
inline Eigen::Vector3d pose_to_vec(const geometry_msgs::msg::Pose& p) {
    return {p.position.x, p.position.y, quaternion_to_yaw(p.orientation)};
}

/// Build a geometry_msgs::Pose from (x, y, yaw).
inline geometry_msgs::msg::Pose vec_to_pose(const Eigen::Vector3d& v) {
    geometry_msgs::msg::Pose p;
    p.position.x = v[0];
    p.position.y = v[1];
    p.position.z = 0.0;
    p.orientation = yaw_to_quaternion(v[2]);
    return p;
}

/**
 * @brief SE(2) composition: T_ab * T_bc = T_ac.
 *
 * Given pose a and a relative transform delta (in a's frame),
 * returns the composed world-frame pose.
 */
inline Eigen::Vector3d se2_compose(const Eigen::Vector3d& a,
                                   const Eigen::Vector3d& delta) {
    double c = std::cos(a[2]);
    double s = std::sin(a[2]);
    return {
        a[0] + delta[0] * c - delta[1] * s,
        a[1] + delta[0] * s + delta[1] * c,
        normalize_angle(a[2] + delta[2])
    };
}

/**
 * @brief SE(2) inverse: given T_ab, return T_ba.
 */
inline Eigen::Vector3d se2_inverse(const Eigen::Vector3d& pose) {
    double c = std::cos(pose[2]);
    double s = std::sin(pose[2]);
    return {
        -(pose[0] * c + pose[1] * s),
        pose[0] * s - pose[1] * c,
        normalize_angle(-pose[2])
    };
}

/**
 * @brief Compute the relative transform between two SE(2) poses.
 *
 * Returns delta such that se2_compose(from, delta) ≈ to.
 */
inline Eigen::Vector3d se2_relative(const Eigen::Vector3d& from,
                                    const Eigen::Vector3d& to) {
    return se2_compose(se2_inverse(from), {to[0], to[1], to[2]});
}

/**
 * @brief Weighted circular mean for an angle column.
 *
 * @param angles  Pointer to N angles.
 * @param weights Pointer to N weights (must be normalised).
 * @param n       Number of elements.
 */
inline double weighted_circular_mean(const double* angles,
                                     const double* weights,
                                     int n) {
    double s = 0.0, c = 0.0;
    for (int i = 0; i < n; ++i) {
        s += weights[i] * std::sin(angles[i]);
        c += weights[i] * std::cos(angles[i]);
    }
    return std::atan2(s, c);
}

}  // namespace math_utils
}  // namespace gpu_amcl_cpp
