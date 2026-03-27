#include "f1tenth_control/algorithms/pure_pursuit.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>

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
            if (values.size() >= 9) {
                pt.left_bound = values[7];
                pt.right_bound = values[8];
            }

            if (!std::isfinite(pt.arc_length) ||
                !std::isfinite(pt.x) ||
                !std::isfinite(pt.y) ||
                !std::isfinite(pt.heading) ||
                !std::isfinite(pt.curvature) ||
                !std::isfinite(pt.velocity)) {
                continue;
            }

            if (std::isfinite(pt.left_bound) && pt.left_bound < 0.0) {
                pt.left_bound = std::numeric_limits<double>::infinity();
            }
            if (std::isfinite(pt.right_bound) && pt.right_bound < 0.0) {
                pt.right_bound = std::numeric_limits<double>::infinity();
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
    
    if (trajectory_.size() < 3) {
        trajectory_.clear();
        return false;
    }

    return true;
}

void PurePursuit::setTrajectory(const std::vector<TrajectoryPoint>& trajectory) {
    trajectory_ = trajectory;

    if (trajectory_.size() > 2) {
        const auto& first = trajectory_.front();
        const auto& last = trajectory_.back();
        const double seam_dist = math::distance(first.x, first.y, last.x, last.y);
        if (seam_dist < 1e-4) {
            trajectory_.pop_back();
        }
    }

    last_closest_idx_ = 0;
}

double PurePursuit::getTrajectoryLength() const {
    if (trajectory_.empty()) return 0.0;
    return trajectory_.back().arc_length;
}

bool PurePursuit::isTrajectoryClosed() const {
    if (trajectory_.size() < 3) {
        return false;
    }

    const auto& first = trajectory_.front();
    const auto& last = trajectory_.back();
    const double seam_dist = math::distance(first.x, first.y, last.x, last.y);
    const double closure_threshold = std::max(0.25, config_.min_lookahead);
    return seam_dist <= closure_threshold;
}

size_t PurePursuit::findClosestPoint(const Point2D& position) {
    if (trajectory_.empty()) return 0;
    
    const size_t n = trajectory_.size();
    const size_t search_radius = std::min(n / 2, size_t(100));
    const bool closed_loop = isTrajectoryClosed();
    
    double min_dist = std::numeric_limits<double>::max();
    size_t closest_idx = last_closest_idx_;
    bool found_heading_candidate = false;
    
    // Search local region — only consider forward-facing waypoints
    // to prevent snapping to the return leg on a closed track
    auto heading_ok = [&](size_t idx) {
        // Accept if heading difference is within ±90° of car heading
        double dh = trajectory_[idx].heading - current_heading_;
        dh = std::atan2(std::sin(dh), std::cos(dh));
        return std::abs(dh) < M_PI_2;
    };
    
    if (closed_loop) {
        const int center = static_cast<int>(last_closest_idx_);
        const int radius = static_cast<int>(search_radius);
        const int n_i = static_cast<int>(n);
        for (int off = -radius; off <= radius; ++off) {
            int idx_i = (center + off) % n_i;
            if (idx_i < 0) {
                idx_i += n_i;
            }
            const size_t i = static_cast<size_t>(idx_i);
            if (!heading_ok(i)) continue;
            const double d = math::distance(position.x, position.y, trajectory_[i].x, trajectory_[i].y);
            if (d < min_dist) {
                min_dist = d;
                closest_idx = i;
                found_heading_candidate = true;
            }
        }
    } else {
        const size_t start_idx = (last_closest_idx_ > search_radius)
                                 ? last_closest_idx_ - search_radius : 0;
        const size_t end_idx = std::min(last_closest_idx_ + search_radius, n - 1);

        for (size_t i = start_idx; i <= end_idx; ++i) {
            if (!heading_ok(i)) continue;
            const double d = math::distance(position.x, position.y, trajectory_[i].x, trajectory_[i].y);
            if (d < min_dist) {
                min_dist = d;
                closest_idx = i;
                found_heading_candidate = true;
            }
        }
    }
    
    // If we're too far from path, do a full search (still heading-filtered)
    const bool seam_region = closed_loop &&
        (last_closest_idx_ < search_radius || last_closest_idx_ + search_radius >= n);
    if (min_dist > config_.position_tolerance * 2 || seam_region) {
        for (size_t i = 0; i < n; ++i) {
            if (!heading_ok(i)) continue;
            double d = math::distance(position.x, position.y, trajectory_[i].x, trajectory_[i].y);
            if (d < min_dist) {
                min_dist = d;
                closest_idx = i;
                found_heading_candidate = true;
            }
        }
    }

    // Recovery fallback: if heading gating rejected everything (e.g. spun car
    // or large transient heading error), reacquire using pure distance search.
    if (!found_heading_candidate) {
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
    
    const Point2D position{state.pose.x, state.pose.y};
    const double heading = state.pose.theta;
    const double current_speed = std::abs(state.velocity);
    current_heading_ = heading;
    
    const size_t n = trajectory_.size();
    const bool closed_loop = isTrajectoryClosed();
    
    // Helper to get next index (wrap-aware)
    auto next_idx = [n, closed_loop](size_t i) -> size_t {
        return closed_loop ? ((i + 1) % n) : std::min(i + 1, n - 1);
    };
    
    const size_t closest_idx = findClosestPoint(position);
    const auto& closest_pt = trajectory_[closest_idx];
    
    // Cross-track error (signed)
    const double path_heading = closest_pt.heading;
    const double dx = closest_pt.x - position.x;
    const double dy = closest_pt.y - position.y;
    output.cross_track_error = -std::sin(path_heading) * dx + std::cos(path_heading) * dy;

    // Corridor clearance
    const bool have_bounds = std::isfinite(closest_pt.left_bound) && std::isfinite(closest_pt.right_bound);
    const double required_half_width = config_.vehicle_half_width + config_.wall_safety_margin;
    double usable_half_width = std::numeric_limits<double>::infinity();
    if (have_bounds) {
        const double corridor_clearance = std::min(
            closest_pt.left_bound - output.cross_track_error,
            closest_pt.right_bound + output.cross_track_error);
        usable_half_width = corridor_clearance - required_half_width;
    }
    
    // Adaptive lookahead
    double lookahead_dist = config_.min_lookahead + config_.lookahead_gain * current_speed;
    lookahead_dist -= config_.cte_lookahead_gain * config_.cte_lookahead_weight * std::abs(output.cross_track_error);
    
    // Turn-radius limit
    const double abs_curvature = std::abs(closest_pt.curvature);
    if (abs_curvature > 0.05) {
        lookahead_dist = std::min(lookahead_dist, config_.curvature_lookahead_gain / abs_curvature);
    }
    
    // Corridor limit
    if (have_bounds) {
        const double corridor_limit = config_.min_lookahead + 
            std::max(0.0, usable_half_width) * config_.corridor_lookahead_factor;
        lookahead_dist = std::min(lookahead_dist, corridor_limit);
    }
    
    lookahead_dist = std::clamp(lookahead_dist, config_.min_lookahead, config_.max_lookahead);

    // Find lookahead target with interpolation
    size_t target_idx = closest_idx;
    TrajectoryPoint target_pt = closest_pt;
    double accumulated_dist = 0.0;
    const size_t max_steps = closed_loop ? n : (n - closest_idx - 1);
    
    for (size_t step = 0; step < max_steps; ++step) {
        const size_t curr = closed_loop ? ((closest_idx + step) % n) : (closest_idx + step);
        const size_t next = next_idx(curr);
        if (next == curr) break;
        
        const double seg_dist = math::distance(
            trajectory_[curr].x, trajectory_[curr].y,
            trajectory_[next].x, trajectory_[next].y);
        
        if (seg_dist > 1e-9 && accumulated_dist + seg_dist >= lookahead_dist) {
            const double t = std::clamp((lookahead_dist - accumulated_dist) / seg_dist, 0.0, 1.0);
            target_pt = interpolate(curr, next, t);
            target_idx = next;
            break;
        }
        accumulated_dist += seg_dist;
        target_idx = next;
        target_pt = trajectory_[next];
    }
    
    // Transform target to vehicle frame
    const double cos_h = std::cos(-heading);
    const double sin_h = std::sin(-heading);
    auto toVehicle = [&](double px, double py) -> Point2D {
        const double tx = px - position.x;
        const double ty = py - position.y;
        return {cos_h * tx - sin_h * ty, sin_h * tx + cos_h * ty};
    };
    
    Point2D target_vehicle = toVehicle(target_pt.x, target_pt.y);
    
    // If target is behind, find forward waypoint
    if (target_vehicle.x <= 0.0) {
        bool found = false;
        for (size_t step = 1; step < max_steps; ++step) {
            const size_t idx = closed_loop ? ((closest_idx + step) % n) : (closest_idx + step);
            const Point2D cand = toVehicle(trajectory_[idx].x, trajectory_[idx].y);
            if (cand.x > 0.05) {
                target_idx = idx;
                target_pt = trajectory_[idx];
                target_vehicle = cand;
                found = true;
                break;
            }
        }
        if (!found) return output;
    }
    
    // Steering computation
    const double actual_lookahead = std::max(0.01, std::hypot(target_vehicle.x, target_vehicle.y));
    const double curvature = 2.0 * target_vehicle.y / (actual_lookahead * actual_lookahead);
    double steering_angle = std::atan(config_.wheelbase * curvature);
    steering_angle = std::clamp(steering_angle, -config_.max_steering, config_.max_steering);
    
    // Speed regulation
    double target_speed = target_pt.velocity;
    
    // Preview curvature for braking
    const double preview_target = std::max(lookahead_dist * config_.curvature_preview_factor, config_.min_lookahead);
    double max_curvature = std::abs(closest_pt.curvature);
    double preview_dist = 0.0;
    
    for (size_t step = 0; step < max_steps && preview_dist < preview_target; ++step) {
        const size_t curr = closed_loop ? ((closest_idx + step) % n) : (closest_idx + step);
        const size_t next = next_idx(curr);
        if (next == curr) break;
        
        max_curvature = std::max(max_curvature, std::abs(trajectory_[curr].curvature));
        const double seg_dist = math::distance(
            trajectory_[curr].x, trajectory_[curr].y,
            trajectory_[next].x, trajectory_[next].y);
        
        if (seg_dist <= 1e-9) continue;
        
        const double remaining = preview_target - preview_dist;
        if (remaining <= seg_dist) {
            const double t = remaining / seg_dist;
            const double k_interp = trajectory_[curr].curvature + 
                t * (trajectory_[next].curvature - trajectory_[curr].curvature);
            max_curvature = std::max(max_curvature, std::abs(k_interp));
            break;
        }
        max_curvature = std::max(max_curvature, std::abs(trajectory_[next].curvature));
        preview_dist += seg_dist;
    }
    
    // Apply speed scaling factors
    const double curv_scale = std::clamp(
        1.0 / (1.0 + config_.curvature_speed_factor * max_curvature),
        config_.curvature_speed_floor_ratio, 1.0);
    const double cte_scale = std::clamp(
        1.0 / (1.0 + config_.cte_speed_factor * std::abs(output.cross_track_error)),
        config_.cte_speed_floor_ratio, 1.0);
    target_speed *= curv_scale * cte_scale;
    
    // Corridor speed scaling
    if (have_bounds) {
        if (usable_half_width <= 0.0) {
            target_speed = 0.0;
        } else {
            const double corridor_scale = std::clamp(
                usable_half_width / config_.corridor_half_width_ref,
                config_.corridor_speed_floor_ratio, 1.0);
            target_speed *= corridor_scale;
        }
    }
    
    // Physics limit
    const double kappa = std::max(max_curvature, std::abs(curvature));
    if (config_.max_lateral_accel > 1e-3 && kappa > 1e-5) {
        target_speed = std::min(target_speed, std::sqrt(config_.max_lateral_accel / kappa));
    }
    
    target_speed = std::max(config_.min_regulated_speed, target_speed);
    
    // Output
    output.steering_angle = steering_angle;
    output.target_speed = target_speed;
    output.closest_idx = closest_idx;
    output.target_idx = target_idx;
    output.target_point = {target_pt.x, target_pt.y};
    output.lookahead_distance = actual_lookahead;
    output.valid = true;
    
    return output;
}

}  // namespace f1tenth_control
