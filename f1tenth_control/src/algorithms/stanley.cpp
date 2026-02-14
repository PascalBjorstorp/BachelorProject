#include "f1tenth_control/algorithms/stanley.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <limits>

namespace f1tenth_control {

Stanley::Stanley() : config_() {}

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
        
        // Parse CSV: s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2
        while (std::getline(ss, token, ',')) {
            try {
                values.push_back(std::stod(token));
            } catch (...) {
                continue;
            }
        }
        
        if (values.size() >= 6) {
            TrajectoryPoint pt;
            pt.arc_length = values[0];
            pt.x = values[1];
            pt.y = values[2];
            pt.heading = values[3];
            pt.curvature = values[4];
            pt.velocity = values[5];
            trajectory_.push_back(pt);
        }
    }
    
    file.close();
    last_closest_idx_ = 0;
    
    return !trajectory_.empty();
}

void Stanley::setTrajectory(const std::vector<TrajectoryPoint>& trajectory) {
    trajectory_ = trajectory;
    last_closest_idx_ = 0;
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
    
    // Heading weight: penalize points with wrong heading direction
    constexpr double heading_weight = 2.0;
    
    // Check if we need full search (first call or far from path)
    const double dist_to_last = math::distance(front_axle_pos.x, front_axle_pos.y,
                                         trajectory_[last_closest_idx_].x,
                                         trajectory_[last_closest_idx_].y);
    const bool full_search = !search_initialized_ || (dist_to_last > config_.position_tolerance * 2);
    
    // Determine search range
    size_t start_idx = 0;
    size_t end_idx = n - 1;
    
    if (!full_search) {
        // Local search: ±100 points around last position
        constexpr size_t search_radius = 100;
        start_idx = (last_closest_idx_ > search_radius) ? last_closest_idx_ - search_radius : 0;
        end_idx = std::min(last_closest_idx_ + search_radius, n - 1);
    }
    
    // Search for closest point with heading-aware cost
    for (size_t i = start_idx; i <= end_idx; ++i) {
        const auto& pt = trajectory_[i];
        const double d = math::distance(front_axle_pos.x, front_axle_pos.y, pt.x, pt.y);
        const double heading_diff = normalizeAngle(pt.heading - vehicle_heading);
        const double cost = d + heading_weight * std::abs(heading_diff);
        
        if (cost < min_cost) {
            min_cost = cost;
            closest_idx = i;
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
    
    if (trajectory_.empty()) {
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
    double heading_error = normalizeAngle(path_heading - vehicle_heading);
    output.heading_error = heading_error;
    
    // === Stanley Steering Law with Velocity-Adaptive Gains ===
    // At high speeds, reduce gains to prevent oscillation
    // This is analytically motivated: higher velocity means faster error dynamics
    
    // Velocity-adaptive heading gain: reduce at high speed to prevent overshoot
    // k_h_effective = k_h * (v_ref / max(v, v_ref))
    // where v_ref is a reference speed (around 5 m/s for F1Tenth)
    const double v_ref = 5.0;  // Reference velocity for gain scheduling
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
    const double dt = 1.0 / config_.control_rate;
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
    const size_t lookahead_points = 30;  // Look ahead
    for (size_t i = 1; i < lookahead_points && (closest_idx + i) < trajectory_.size(); ++i) {
        size_t idx = (closest_idx + i) % trajectory_.size();
        max_upcoming_curvature = std::max(max_upcoming_curvature, 
                                          std::abs(trajectory_[idx].curvature));
    }
    
    // Reduce speed based on curvature (tighter turn = slower)
    double curvature_limit = config_.max_speed / (1.0 + config_.curvature_speed_factor * max_upcoming_curvature * 10.0);
    target_speed = std::min(target_speed, curvature_limit);
    
    // Apply limits
    target_speed = std::clamp(target_speed, config_.min_speed, config_.max_speed);
    
    // Fill output
    output.steering_angle = steering_angle;
    output.target_speed = target_speed;
    output.closest_idx = closest_idx;
    output.valid = true;
    
    return output;
}

}  // namespace f1tenth_control
