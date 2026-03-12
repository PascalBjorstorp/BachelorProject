#include "f1tenth_control/algorithms/pure_pursuit.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace f1tenth_control {

PurePursuit::PurePursuit() : config_() {}

PurePursuit::PurePursuit(const PurePursuitConfig& config) : config_(config) {}

bool PurePursuit::loadTrajectory(const std::string& csv_path) {
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
                continue;  // Skip malformed values
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

void PurePursuit::setTrajectory(const std::vector<TrajectoryPoint>& trajectory) {
    trajectory_ = trajectory;
    last_closest_idx_ = 0;
}

double PurePursuit::getTrajectoryLength() const {
    if (trajectory_.empty()) return 0.0;
    return trajectory_.back().arc_length;
}

size_t PurePursuit::findClosestPoint(const Point2D& position) {
    if (trajectory_.empty()) return 0;
    
    // Start search from last known closest point for efficiency
    const size_t n = trajectory_.size();
    const size_t search_radius = std::min(n / 2, size_t(100));
    
    size_t start_idx = (last_closest_idx_ > search_radius) 
                       ? last_closest_idx_ - search_radius : 0;
    size_t end_idx = std::min(last_closest_idx_ + search_radius, n - 1);
    
    double min_dist = std::numeric_limits<double>::max();
    size_t closest_idx = last_closest_idx_;
    
    // Search local region
    for (size_t i = start_idx; i <= end_idx; ++i) {
        double d = math::distance(position.x, position.y, trajectory_[i].x, trajectory_[i].y);
        if (d < min_dist) {
            min_dist = d;
            closest_idx = i;
        }
    }
    
    // If we're too far from path, do a full search
    if (min_dist > config_.position_tolerance * 2) {
        for (size_t i = 0; i < n; ++i) {
            double d = math::distance(position.x, position.y, trajectory_[i].x, trajectory_[i].y);
            if (d < min_dist) {
                min_dist = d;
                closest_idx = i;
            }
        }
    }
    
    last_closest_idx_ = closest_idx;
    return closest_idx;
}

size_t PurePursuit::findLookaheadTarget(size_t closest_idx, double lookahead_dist) {
    if (trajectory_.empty()) return 0;
    
    const size_t n = trajectory_.size();
    double accumulated_dist = 0.0;
    size_t target_idx = closest_idx;
    
    // Walk forward along trajectory until we reach lookahead distance
    for (size_t i = closest_idx; i < closest_idx + n; ++i) {
        size_t curr_idx = i % n;
        size_t next_idx = (i + 1) % n;
        
        double segment_dist = math::distance(
            trajectory_[curr_idx].x, trajectory_[curr_idx].y,
            trajectory_[next_idx].x, trajectory_[next_idx].y
        );
        
        if (accumulated_dist + segment_dist >= lookahead_dist) {
            target_idx = next_idx;
            break;
        }
        
        accumulated_dist += segment_dist;
        target_idx = next_idx;
    }
    
    return target_idx;
}

TrajectoryPoint PurePursuit::interpolate(size_t idx1, size_t idx2, double t) const {
    const auto& p1 = trajectory_[idx1];
    const auto& p2 = trajectory_[idx2];
    
    TrajectoryPoint result;
    result.x = p1.x + t * (p2.x - p1.x);
    result.y = p1.y + t * (p2.y - p1.y);
    // Wrap heading difference to [-π, π] to avoid interpolating the long way
    double heading_diff = p2.heading - p1.heading;
    heading_diff = std::atan2(std::sin(heading_diff), std::cos(heading_diff));
    result.heading = p1.heading + t * heading_diff;
    result.velocity = p1.velocity + t * (p2.velocity - p1.velocity);
    result.curvature = p1.curvature + t * (p2.curvature - p1.curvature);
    result.arc_length = p1.arc_length + t * (p2.arc_length - p1.arc_length);
    
    return result;
}

PurePursuitOutput PurePursuit::compute(const VehicleState& state) {
    PurePursuitOutput output;
    output.valid = false;
    
    if (trajectory_.empty()) {
        return output;
    }
    
    // Get current position and heading
    Point2D position{state.pose.x, state.pose.y};
    double heading = state.pose.theta;
    double current_speed = std::abs(state.velocity);
    
    // Find closest point on trajectory
    size_t closest_idx = findClosestPoint(position);
    const auto& closest_pt = trajectory_[closest_idx];
    
    // Compute cross-track error (signed distance to path)
    double dx = closest_pt.x - position.x;
    double dy = closest_pt.y - position.y;
    double path_heading = closest_pt.heading;
    output.cross_track_error = -std::sin(path_heading) * dx + std::cos(path_heading) * dy;
    
    // Compute adaptive lookahead distance
    double lookahead_dist = config_.min_lookahead + config_.lookahead_gain * current_speed;
    lookahead_dist = std::clamp(lookahead_dist, config_.min_lookahead, config_.max_lookahead);
    
    // Find lookahead target point
    size_t target_idx = findLookaheadTarget(closest_idx, lookahead_dist);
    const auto& target_pt = trajectory_[target_idx];
    
    // Compute target point relative to vehicle
    double tx = target_pt.x - position.x;
    double ty = target_pt.y - position.y;
    
    // Transform to vehicle frame
    double cos_h = std::cos(-heading);
    double sin_h = std::sin(-heading);
    double target_x_vehicle = cos_h * tx - sin_h * ty;
    double target_y_vehicle = sin_h * tx + cos_h * ty;
    
    // Actual lookahead distance
    double actual_lookahead = std::hypot(target_x_vehicle, target_y_vehicle);
    if (actual_lookahead < 0.01) {
        actual_lookahead = 0.01;  // Prevent division by zero
    }
    
    // Pure Pursuit steering law
    // curvature = 2 * y / L^2 where y is lateral offset in vehicle frame
    double curvature = 2.0 * target_y_vehicle / (actual_lookahead * actual_lookahead);
    
    // Convert curvature to steering angle using bicycle model
    double steering_angle = std::atan(config_.wheelbase * curvature);
    
    // Clamp steering (hardware servo enforces its own rate limit)
    steering_angle = std::clamp(steering_angle, -config_.max_steering, config_.max_steering);
    
    // Compute target speed from trajectory
    double target_speed = target_pt.velocity * config_.speed_gain;
    
    // Speed reduction based on path curvature ahead
    // Look at curvature along the next few waypoints
    double max_upcoming_curvature = 0.0;
    const size_t lookahead_points = 20;  // Look ~2 seconds ahead at ~10 points/sec
    for (size_t i = 0; i < lookahead_points && (closest_idx + i) < trajectory_.size(); ++i) {
        size_t idx = (closest_idx + i) % trajectory_.size();
        max_upcoming_curvature = std::max(max_upcoming_curvature, std::abs(trajectory_[idx].curvature));
    }
    
    // Reduce speed based on upcoming curvature
    // Higher curvature = sharper turn = need to slow down
    double curvature_speed_limit = config_.max_speed / (1.0 + config_.curvature_speed_factor * max_upcoming_curvature * 10.0);
    target_speed = std::min(target_speed, curvature_speed_limit);
    
    // Also reduce speed based on steering magnitude
    double steer_ratio = std::abs(steering_angle) / config_.max_steering;
    double speed_reduction = 1.0 - 0.3 * steer_ratio;
    target_speed *= speed_reduction;
    
    // Apply speed limits
    target_speed = std::clamp(target_speed, config_.min_speed, config_.max_speed);
    
    // Fill output
    output.steering_angle = steering_angle;
    output.target_speed = target_speed;
    output.closest_idx = closest_idx;
    output.target_idx = target_idx;
    output.target_point = Point2D{target_pt.x, target_pt.y};
    output.lookahead_distance = actual_lookahead;
    output.valid = true;
    
    return output;
}

}  // namespace f1tenth_control
