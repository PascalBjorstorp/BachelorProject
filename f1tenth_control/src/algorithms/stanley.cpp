#include "algorithms/stanley.hpp"

#include "common/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>


namespace f1tenth_control {
Stanley::Stanley(const StanleyConfig& config) : config_(config) {}
bool Stanley::loadTrajectory(const std::string& csv_path) {
    std::ifstream file(csv_path);
    if (!file.is_open()) {
        return false;
    }
    
    trajectory_.clear();
    std::string line;
    
    // Skip header line (starts with #)
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        
        std::stringstream ss(line);
        std::string token;
        std::vector<double> values;
        bool parse_failed = false;
        
        // Parse CSV: s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2
        while (std::getline(ss, token, ',')) {
            try {
                values.push_back(std::stod(token));
            } catch (...) {
                parse_failed = true;
                break;
            }
        }

        if (parse_failed) {
            continue;
        }
        
        if (values.size() >= 6) {
            TrajectoryPoint pt;
            pt.arc_length = values[0];
            pt.x = values[1];
            pt.y = values[2];
            pt.heading = values[3];
            pt.curvature = values[4];
            pt.velocity = values[5];

            if (!std::isfinite(pt.arc_length) ||
                !std::isfinite(pt.x) ||
                !std::isfinite(pt.y) ||
                !std::isfinite(pt.heading) ||
                !std::isfinite(pt.curvature) ||
                !std::isfinite(pt.velocity)) {
                continue;
            }

            trajectory_.push_back(pt);
        }
    }
    
    file.close();

    // Avoid duplicate terminal waypoint creating a zero-length seam segment.
    if (trajectory_.size() > 2) {
        const auto& first = trajectory_.front();
        const auto& last = trajectory_.back();
        const double seam_dist = math::distance(first.x, first.y, last.x, last.y);
        if (seam_dist < 1e-4) {
            trajectory_.pop_back();
        }
    }

    last_closest_idx_ = 0;
    search_initialized_ = false;
    last_steering_ = 0.0;
    
    if (trajectory_.size() < 3) {
        trajectory_.clear();
        return false;
    }

    return true;
}
void Stanley::setTrajectory(const std::vector<TrajectoryPoint>& trajectory) {
    trajectory_ = trajectory;

    if (trajectory_.size() > 2) {
        const auto& first = trajectory_.front();
        const auto& last = trajectory_.back();
        const double seam_dist = math::distance(first.x, first.y, last.x, last.y);
        if (seam_dist < 1e-4) {
            trajectory_.pop_back();
        }
    }

    if (trajectory_.size() < 3) {
        trajectory_.clear();
    }

    last_closest_idx_ = 0;
    search_initialized_ = false;
    last_steering_ = 0.0;
}
double Stanley::getTrajectoryLength() const {
    if (trajectory_.empty()) return 0.0;
    return trajectory_.back().arc_length;
}
size_t Stanley::findClosestPoint(const Point2D& front_axle_pos, double vehicle_heading) {
    if (trajectory_.empty()) return 0;
    
    const size_t n = trajectory_.size();
    double min_cost = std::numeric_limits<double>::max();
    size_t closest_idx = last_closest_idx_;
    
    // Penalize heading-opposed points to avoid latching to the wrong segment.
    constexpr double heading_weight = 2.0;
    
    // Check if we need full search (first call or far from path)
    const double dist_to_last = math::distance(front_axle_pos.x, front_axle_pos.y,
                                         trajectory_[last_closest_idx_].x,
                                         trajectory_[last_closest_idx_].y);
    const bool full_search = !search_initialized_ || (dist_to_last > config_.position_tolerance * 2);
    
    if (full_search) {
        for (size_t i = 0; i < n; ++i) {
            const auto& pt = trajectory_[i];
            const double d = math::distance(front_axle_pos.x, front_axle_pos.y, pt.x, pt.y);
            const double heading_diff = math::normalizeAngle(pt.heading - vehicle_heading);
            const double cost = d + heading_weight * std::abs(heading_diff);

            if (cost < min_cost) {
                min_cost = cost;
                closest_idx = i;
            }
        }
    } else {
        // Local wrap-around search around previous index for looped tracks.
        constexpr int search_radius = 100;
        const int n_i = static_cast<int>(n);
        const int center = static_cast<int>(last_closest_idx_);

        for (int offset = -search_radius; offset <= search_radius; ++offset) {
            int idx_i = (center + offset) % n_i;
            if (idx_i < 0) {
                idx_i += n_i;
            }
            const size_t i = static_cast<size_t>(idx_i);

            const auto& pt = trajectory_[i];
            const double d = math::distance(front_axle_pos.x, front_axle_pos.y, pt.x, pt.y);
            const double heading_diff = math::normalizeAngle(pt.heading - vehicle_heading);
            const double cost = d + heading_weight * std::abs(heading_diff);

            if (cost < min_cost) {
                min_cost = cost;
                closest_idx = i;
            }
        }
    }
    
    last_closest_idx_ = closest_idx;
    search_initialized_ = true;
    return closest_idx;
}
double Stanley::computeCrossTrackError(const Point2D& front_axle_pos, size_t closest_idx) {
    const auto& closest_pt = trajectory_[closest_idx];
    
    // Vector from closest point to front axle
    double dx = front_axle_pos.x - closest_pt.x;
    double dy = front_axle_pos.y - closest_pt.y;
    
    // Cross-track error is perpendicular distance to path
    // Positive = vehicle to the left of path (in path frame)
    // Use path heading to determine sign
    double path_heading = closest_pt.heading;
    
    // Cross product: path_tangent × (front_axle - closest_pt)
    // path_tangent = (cos(heading), sin(heading))
    // cross = cos(h) * dy - sin(h) * dx
    double cross_track_error = std::cos(path_heading) * dy - std::sin(path_heading) * dx;
    
    return cross_track_error;
}
StanleyOutput Stanley::compute(const VehicleState& state) {
    StanleyOutput output;
    output.valid = false;
    
    if (!hasTrajectory()) {
        return output;
    }
    
    // Stanley uses front axle position
    // Calculate front axle position from rear axle (state.pose is typically rear axle)
    double front_axle_x = state.pose.x + config_.wheelbase * std::cos(state.pose.theta);
    double front_axle_y = state.pose.y + config_.wheelbase * std::sin(state.pose.theta);
    Point2D front_axle_pos{front_axle_x, front_axle_y};
    
    double vehicle_heading = state.pose.theta;
    double velocity = std::max(std::abs(state.velocity), 0.01);  // Prevent zero velocity issues

    // Find closest point to front axle (heading-aware to prevent wrong segment matching)
    size_t closest_idx = findClosestPoint(front_axle_pos, vehicle_heading);
    const auto& closest_pt = trajectory_[closest_idx];
    
    // Compute cross-track error
    double cross_track_error = computeCrossTrackError(front_axle_pos, closest_idx);
    output.cross_track_error = cross_track_error;
    
    // Compute heading error (desired - actual)
    double path_heading = closest_pt.heading;
    double heading_error = math::normalizeAngle(path_heading - vehicle_heading);
    output.heading_error = heading_error;
    
    // === Stanley Steering Law with Velocity-Adaptive Gains ===
    // At high speeds, reduce gains to prevent oscillation
    // This is analytically motivated: higher velocity means faster error dynamics
    
    // Velocity-adaptive heading gain: reduce at high speed to prevent overshoot
    // k_h_effective = k_h * (v_ref / max(v, v_ref))
    // where v_ref is a reference speed (around 5 m/s for F1Tenth)
    // Reference speed where heading gain remains fully active.
    const double v_ref = 5.0;
    double k_h_effective = config_.k_h * std::min(1.0, v_ref / velocity);
    
    // Term 1: Heading error correction (velocity-adaptive)
    double heading_term = k_h_effective * heading_error;
    
    // Term 2: Cross-track error correction (with velocity damping built into formula)
    // The atan(k_e * e / (k_s + v)) naturally reduces at high speed
    // Negate CTE: if car is LEFT of path (positive CTE), we need negative steering (turn right)
    double cte_term = std::atan(-config_.k_e * cross_track_error / (config_.k_s + velocity));
    
    // Term 3: Feedforward from path curvature (optional but recommended)
    double feedforward_term = 0.0;
    if (config_.use_feedforward) {
        // Feedforward: steer proportional to path curvature
        // δ_ff = κ * L (Ackermann steering geometry)
        feedforward_term = config_.feedforward_gain * closest_pt.curvature * config_.wheelbase;
        output.feedforward_steering = feedforward_term;
    }
    
    // Term 4: Damping term using vehicle angular velocity to suppress oscillation
    // This acts like a derivative term in a PD controller
    double damping_term = -config_.k_d * state.angular_velocity;
    
    // Debug: store individual terms
    output.heading_term = heading_term;
    output.cte_term = cte_term;
    
    // Combine all terms
    double steering_angle = heading_term + cte_term + feedforward_term + damping_term;
    
    // Apply steering rate limiting to prevent sudden changes that cause loss of traction
    // Use configured control rate (default 200 Hz → dt = 0.005s)
    const double safe_control_rate = std::max(config_.control_rate, 1e-3);
    const double dt = 1.0 / safe_control_rate;
    double max_delta = config_.max_steering_rate * dt;
    double steering_delta = steering_angle - last_steering_;
    steering_delta = std::clamp(steering_delta, -max_delta, max_delta);
    steering_angle = last_steering_ + steering_delta;
    last_steering_ = steering_angle;
    
    // Clamp steering
    steering_angle = std::clamp(steering_angle, -config_.max_steering, config_.max_steering);
    
    // === Speed Control ===
    // Use trajectory velocity, scaled by gain
    double target_speed = closest_pt.velocity * config_.speed_gain;
    
    // Look ahead for curvature and reduce speed before sharp turns
    double max_upcoming_curvature = std::abs(closest_pt.curvature);
    // Preview horizon expressed in waypoints to detect upcoming tight turns.
    const size_t lookahead_points = 30;
    for (size_t i = 1; i < lookahead_points; ++i) {
        size_t idx = closest_idx + i;
        if (idx >= trajectory_.size()) {
            idx %= trajectory_.size();
        }
        max_upcoming_curvature = std::max(max_upcoming_curvature, 
                                          std::abs(trajectory_[idx].curvature));
    }
    
    // Reduce speed based on curvature (tighter turn = slower)
    double curvature_limit = config_.max_speed / (1.0 + config_.curvature_speed_factor * max_upcoming_curvature * 10.0);
    target_speed = std::min(target_speed, curvature_limit);
    
    // Apply limits
    const double min_speed = std::min(config_.min_speed, config_.max_speed);
    const double max_speed = std::max(config_.min_speed, config_.max_speed);
    target_speed = std::clamp(target_speed, min_speed, max_speed);
    
    // Fill output
    output.steering_angle = steering_angle;
    output.target_speed = target_speed;
    output.closest_idx = closest_idx;
    output.valid = true;
    
    return output;
}

}  // namespace f1tenth_control
