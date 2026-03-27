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
    
    // Get current position and heading
    Point2D position{state.pose.x, state.pose.y};
    double heading = state.pose.theta;
    double current_speed = std::abs(state.velocity);
    current_heading_ = heading;  // Store for heading-aware closest point search
    
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
    lookahead_dist -= config_.cte_lookahead_gain * config_.cte_lookahead_weight * std::abs(output.cross_track_error);
    lookahead_dist -= config_.curvature_lookahead_gain * std::abs(closest_pt.curvature);
    lookahead_dist = std::clamp(lookahead_dist, config_.min_lookahead, config_.max_lookahead);

    const bool closed_loop = isTrajectoryClosed();

    // Find lookahead target and interpolate for continuous target tracking.
    size_t target_idx = closest_idx;
    size_t target_seg_start_idx = closest_idx;
    size_t target_seg_end_idx = closest_idx;
    double target_seg_t = 0.0;
    const size_t n = trajectory_.size();
    double accumulated_dist = 0.0;
    bool found_target = false;
    if (closed_loop) {
        for (size_t i = closest_idx; i < closest_idx + n; ++i) {
            size_t curr_idx = i % n;
            size_t next_idx = (i + 1) % n;
            double segment_dist = math::distance(
                trajectory_[curr_idx].x, trajectory_[curr_idx].y,
                trajectory_[next_idx].x, trajectory_[next_idx].y
            );

            if (segment_dist > 1e-9 && accumulated_dist + segment_dist >= lookahead_dist) {
                target_seg_start_idx = curr_idx;
                target_seg_end_idx = next_idx;
                target_seg_t = (lookahead_dist - accumulated_dist) / segment_dist;
                target_seg_t = std::clamp(target_seg_t, 0.0, 1.0);
                target_idx = next_idx;
                found_target = true;
                break;
            }

            accumulated_dist += segment_dist;
            target_idx = next_idx;
        }
    } else {
        for (size_t curr_idx = closest_idx; curr_idx + 1 < n; ++curr_idx) {
            const size_t next_idx = curr_idx + 1;
            const double segment_dist = math::distance(
                trajectory_[curr_idx].x, trajectory_[curr_idx].y,
                trajectory_[next_idx].x, trajectory_[next_idx].y
            );

            if (segment_dist > 1e-9 && accumulated_dist + segment_dist >= lookahead_dist) {
                target_seg_start_idx = curr_idx;
                target_seg_end_idx = next_idx;
                target_seg_t = (lookahead_dist - accumulated_dist) / segment_dist;
                target_seg_t = std::clamp(target_seg_t, 0.0, 1.0);
                target_idx = next_idx;
                found_target = true;
                break;
            }

            accumulated_dist += segment_dist;
            target_idx = next_idx;
        }
    }

    TrajectoryPoint target_pt;
    if (found_target) {
        target_pt = interpolate(target_seg_start_idx, target_seg_end_idx, target_seg_t);
    } else {
        target_pt = trajectory_[target_idx];
    }
    
    // Transform to vehicle frame
    double cos_h = std::cos(-heading);
    double sin_h = std::sin(-heading);
    auto targetToVehicleFrame = [&](const TrajectoryPoint& pt) {
        const double tx = pt.x - position.x;
        const double ty = pt.y - position.y;
        return Point2D{
            cos_h * tx - sin_h * ty,
            sin_h * tx + cos_h * ty
        };
    };

    Point2D target_vehicle = targetToVehicleFrame(target_pt);
    double target_x_vehicle = target_vehicle.x;
    double target_y_vehicle = target_vehicle.y;

    // Guard against behind-target geometry which can yield near-straight steering.
    if (target_x_vehicle <= 0.0) {
        bool found_forward_target = false;
        if (closed_loop) {
            for (size_t step = 1; step < n; ++step) {
                const size_t idx = (closest_idx + step) % n;
                const Point2D candidate = targetToVehicleFrame(trajectory_[idx]);
                if (candidate.x > 0.05) {
                    target_idx = idx;
                    target_pt = trajectory_[idx];
                    target_x_vehicle = candidate.x;
                    target_y_vehicle = candidate.y;
                    found_forward_target = true;
                    break;
                }
            }
        } else {
            for (size_t idx = closest_idx + 1; idx < n; ++idx) {
                const Point2D candidate = targetToVehicleFrame(trajectory_[idx]);
                if (candidate.x > 0.05) {
                    target_idx = idx;
                    target_pt = trajectory_[idx];
                    target_x_vehicle = candidate.x;
                    target_y_vehicle = candidate.y;
                    found_forward_target = true;
                    break;
                }
            }
        }

        if (!found_forward_target) {
            return output;
        }
    }
    
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
    
    // Base speed from trajectory, then apply lookahead-based curvature slowdown.
    double target_speed = target_pt.velocity;

    // Preview curvature over approximately one dynamic lookahead distance.
    // This slows the car before entering tighter turns on real hardware.
    double max_upcoming_curvature = std::abs(closest_pt.curvature);
    double preview_distance = 0.0;
    const double preview_target = std::max(lookahead_dist, config_.min_lookahead);
    if (closed_loop) {
        for (size_t i = closest_idx; i < closest_idx + n && preview_distance < preview_target; ++i) {
            const size_t curr_idx = i % n;
            const size_t next_idx = (i + 1) % n;

            const double k0 = trajectory_[curr_idx].curvature;
            const double k1 = trajectory_[next_idx].curvature;
            max_upcoming_curvature = std::max(max_upcoming_curvature, std::abs(k0));

            const double segment_dist = math::distance(
                trajectory_[curr_idx].x, trajectory_[curr_idx].y,
                trajectory_[next_idx].x, trajectory_[next_idx].y
            );

            if (segment_dist <= 1e-9) {
                continue;
            }

            const double remaining = preview_target - preview_distance;
            if (remaining <= segment_dist) {
                const double t = std::clamp(remaining / segment_dist, 0.0, 1.0);
                const double k_interp = k0 + t * (k1 - k0);
                max_upcoming_curvature = std::max(max_upcoming_curvature, std::abs(k_interp));
                preview_distance = preview_target;
                break;
            }

            max_upcoming_curvature = std::max(max_upcoming_curvature, std::abs(k1));
            preview_distance += segment_dist;
        }
    } else {
        for (size_t curr_idx = closest_idx; curr_idx + 1 < n && preview_distance < preview_target; ++curr_idx) {
            const size_t next_idx = curr_idx + 1;

            const double k0 = trajectory_[curr_idx].curvature;
            const double k1 = trajectory_[next_idx].curvature;
            max_upcoming_curvature = std::max(max_upcoming_curvature, std::abs(k0));

            const double segment_dist = math::distance(
                trajectory_[curr_idx].x, trajectory_[curr_idx].y,
                trajectory_[next_idx].x, trajectory_[next_idx].y
            );

            if (segment_dist <= 1e-9) {
                continue;
            }

            const double remaining = preview_target - preview_distance;
            if (remaining <= segment_dist) {
                const double t = std::clamp(remaining / segment_dist, 0.0, 1.0);
                const double k_interp = k0 + t * (k1 - k0);
                max_upcoming_curvature = std::max(max_upcoming_curvature, std::abs(k_interp));
                preview_distance = preview_target;
                break;
            }

            max_upcoming_curvature = std::max(max_upcoming_curvature, std::abs(k1));
            preview_distance += segment_dist;
        }
    }

    const double floor_ratio = std::clamp(config_.curvature_speed_floor_ratio, 0.0, 1.0);
    double curvature_speed_scale =
        1.0 / (1.0 + config_.curvature_speed_factor * max_upcoming_curvature);
    curvature_speed_scale = std::clamp(curvature_speed_scale, floor_ratio, 1.0);
    target_speed *= curvature_speed_scale;

    // Additional slowdown when cross-track error grows, improving robustness
    // against lap-to-lap drift at higher speeds.
    const double cte_floor_ratio = std::clamp(config_.cte_speed_floor_ratio, 0.0, 1.0);
    double cte_speed_scale = 1.0 / (1.0 + config_.cte_speed_factor * std::abs(output.cross_track_error));
    cte_speed_scale = std::clamp(cte_speed_scale, cte_floor_ratio, 1.0);
    target_speed *= cte_speed_scale;

    // Physics-aware speed cap from lateral acceleration: v <= sqrt(a_lat_max / |kappa|).
    const double kappa_preview = std::max(max_upcoming_curvature, std::abs(curvature));
    if (config_.max_lateral_accel > 1e-3 && kappa_preview > 1e-5) {
        const double v_lat_limit = std::sqrt(config_.max_lateral_accel / kappa_preview);
        target_speed = std::min(target_speed, v_lat_limit);
    }

    target_speed = std::max(config_.min_regulated_speed, target_speed);
    
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
