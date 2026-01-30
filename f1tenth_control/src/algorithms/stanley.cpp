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
            StanleyTrajectoryPoint pt;
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

void Stanley::setTrajectory(const std::vector<StanleyTrajectoryPoint>& trajectory) {
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
    size_t closest_idx = 0;
    
    // Heading weight: penalize points with wrong heading direction
    // This prevents matching wrong track segment when track passes near itself
    const double heading_weight = 2.0;  // Weight for heading mismatch in cost function
    
    // On first call or if far from path, do full search
    bool full_search = (last_closest_idx_ == 0) || 
                       (distance(front_axle_pos.x, front_axle_pos.y,
                                trajectory_[last_closest_idx_].x,
                                trajectory_[last_closest_idx_].y) > config_.position_tolerance * 2);
    
    auto compute_cost = [&](size_t i) -> double {
        double d = distance(front_axle_pos.x, front_axle_pos.y, 
                           trajectory_[i].x, trajectory_[i].y);
        
        // Heading difference (normalized to [-pi, pi])
        double heading_diff = normalizeAngle(trajectory_[i].heading - vehicle_heading);
        
        // Cost = distance + weighted heading penalty
        // If heading differs by more than 90 degrees, heavily penalize
        double heading_penalty = heading_weight * std::abs(heading_diff);
        
        return d + heading_penalty;
    };
    
    if (full_search) {
        // Full trajectory search with heading-aware cost
        for (size_t i = 0; i < n; ++i) {
            double cost = compute_cost(i);
            if (cost < min_cost) {
                min_cost = cost;
                closest_idx = i;
            }
        }
    } else {
        // Local search around last known position
        const size_t search_radius = std::min(n / 2, size_t(100));
        size_t start_idx = (last_closest_idx_ > search_radius) 
                           ? last_closest_idx_ - search_radius : 0;
        size_t end_idx = std::min(last_closest_idx_ + search_radius, n - 1);
        
        closest_idx = last_closest_idx_;
        for (size_t i = start_idx; i <= end_idx; ++i) {
            double cost = compute_cost(i);
            if (cost < min_cost) {
                min_cost = cost;
                closest_idx = i;
            }
        }
    }
    
    last_closest_idx_ = closest_idx;
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
    // Assume 50 Hz control rate (dt = 0.02s)
    const double dt = 0.02;
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
