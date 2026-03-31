#include "algorithms/pure_pursuit.hpp"

namespace f1tenth_control {

PurePursuit::PurePursuit(const PurePursuitConfig& config) : config_(config) {}

bool PurePursuit::loadTrajectory(const std::string& csv_path) {
    // Open the file
    std::ifstream file(csv_path);
    if (!file.is_open()) {
        return false;
    }
    
    // Clear any existing trajectory data
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
        
        // Parse CSV: s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2, left_bound_m, right_bound_m
        while (std::getline(ss, token, ',')) {
            try {
                values.push_back(std::stod(token));
            } catch (...) {
                parse_failed = true;
                break;
            }
        }

        // If parsing failed, skip this line
        if (parse_failed) {
            continue;
        }
        
        // Create TrajectoryPoint if we have enough values
        if (values.size() >= 6) {
            TrajectoryPoint pt;
            pt.arc_length = values[0];
            pt.x = values[1];
            pt.y = values[2];
            pt.heading = values[3];
            pt.curvature = values[4];
            pt.velocity = values[5];
            // Optional bounds if provided
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
    // Trajectory length is arc_length of last point, or 0 if no trajectory loaded
    if (trajectory_.empty()) return 0.0;
    return trajectory_.back().arc_length;
}

size_t PurePursuit::findClosestPoint(const Point2D& position) {
    // If no trajectory is loaded, return 0 as a safe default index
    if (trajectory_.empty()) return 0;
    
    // Search parameters
    const size_t n = trajectory_.size();
    const size_t search_radius = std::min(n / 2, size_t(100));
    
    // Initialize search with last known closest index for efficiency
    double min_dist = std::numeric_limits<double>::max();
    size_t closest_idx = last_closest_idx_;
    bool found_heading_candidate = false;
    
    // Search local region — only consider forward-facing waypoints
    // to prevent snapping to the return leg on a closed track
    auto heading_ok = [&](size_t idx) {
        // Accept if heading difference is within ±90° of car heading
        double dh = trajectory_[idx].heading - current_heading_;
        dh = std::atan2(std::sin(dh), std::cos(dh));
        return std::abs(dh) < (0.5 * constants::PI);
    };
    
   
    // First try a local search around the last closest index for efficiency
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

    // If the car is too far from path, do a full search (still heading-filtered)
    const bool seam_region = (last_closest_idx_ < search_radius || last_closest_idx_ + search_radius >= n);
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
    // Linear interpolation between two trajectory points for continuous target tracking.
    const auto& p1 = trajectory_[idx1];
    const auto& p2 = trajectory_[idx2];
    
    // Interpolate position, heading, velocity, curvature, and arc length.
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

    // Get current position and heading.
    const Point2D position{state.pose.x, state.pose.y};
    const double heading = state.pose.theta;
    const double current_speed = std::abs(state.velocity);
    current_heading_ = heading;

    // Find closest point on trajectory.
    const size_t closest_idx = findClosestPoint(position);
    const auto& closest_pt = trajectory_[closest_idx];

    // Compute cross-track error (signed distance to path).
    const double dx = closest_pt.x - position.x;
    const double dy = closest_pt.y - position.y;
    const double path_heading = closest_pt.heading;
    output.cross_track_error = -std::sin(path_heading) * dx + std::cos(path_heading) * dy;

    // Corridor-aware footprint clearance at closest point.
    const bool have_bounds = std::isfinite(closest_pt.left_bound) && std::isfinite(closest_pt.right_bound);
    const double required_half_width = std::max(0.0, config_.vehicle_half_width) +
                                       std::max(0.0, config_.wall_safety_margin);
    double usable_half_width = std::numeric_limits<double>::infinity();
    if (have_bounds) {
        const double left_clearance = closest_pt.left_bound - output.cross_track_error;
        const double right_clearance = closest_pt.right_bound + output.cross_track_error;
        const double corridor_clearance = std::min(left_clearance, right_clearance);
        usable_half_width = corridor_clearance - required_half_width;
    }

    // Compute adaptive lookahead distance.
    // Base: min + velocity-proportional gain.
    double lookahead_dist = config_.min_lookahead + config_.lookahead_gain * current_speed;
    // Reduce for cross-track error (tighter tracking when off-path).
    lookahead_dist -= config_.cte_lookahead_gain * config_.cte_lookahead_weight * std::abs(output.cross_track_error);

    // Turn-radius-based limiting: lookahead should not exceed a fraction of turn radius.
    // Turn radius R = 1/|kappa|.
    const double abs_curvature = std::abs(closest_pt.curvature);
    // Only apply for meaningful curvature to avoid over-limiting on straights.
    if (abs_curvature > 0.05) {
        const double curvature_limited_lookahead = config_.curvature_lookahead_gain / abs_curvature;
        lookahead_dist = std::min(lookahead_dist, curvature_limited_lookahead);
    }

    // Clamp lookahead by available corridor width so the controller does not
    // over-preview through tight corners with limited vehicle clearance.
    if (have_bounds) {
        const double corridor_limited_lookahead = config_.min_lookahead +
            std::max(0.0, usable_half_width) * std::max(0.0, config_.corridor_lookahead_factor);
        lookahead_dist = std::min(lookahead_dist, corridor_limited_lookahead);
    }

    // Enforce absolute min/max lookahead limits.
    lookahead_dist = std::clamp(lookahead_dist, config_.min_lookahead, config_.max_lookahead);

    // Find lookahead target and interpolate for continuous target tracking.
    size_t target_idx = closest_idx;
    size_t target_seg_start_idx = closest_idx;
    size_t target_seg_end_idx = closest_idx;
    double target_seg_t = 0.0;
    const size_t n = trajectory_.size();
    double accumulated_dist = 0.0;
    bool found_target = false;

    // Search forward until accumulated arc distance reaches lookahead distance.
    for (size_t i = closest_idx; i < closest_idx + n; ++i) {
        const size_t curr_idx = i % n;
        const size_t next_idx = (i + 1) % n;
        const double segment_dist = math::distance(
            trajectory_[curr_idx].x, trajectory_[curr_idx].y,
            trajectory_[next_idx].x, trajectory_[next_idx].y
        );

        // Skip degenerate segments to avoid numerical issues.
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

    // Interpolate when target falls inside a segment.
    TrajectoryPoint target_pt;
    if (found_target) {
        target_pt = interpolate(target_seg_start_idx, target_seg_end_idx, target_seg_t);
    } else {
        target_pt = trajectory_[target_idx];
    }

    // Transform target to vehicle frame.
    const double cos_h = std::cos(-heading);
    const double sin_h = std::sin(-heading);
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

    // If the target is behind the vehicle, search forward for the first point ahead.
    if (target_x_vehicle <= 0.0) {
        bool found_forward_target = false;
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

        if (!found_forward_target) {
            return output;
        }
    }

    target_vehicle = Point2D{target_x_vehicle, target_y_vehicle};

    // Actual lookahead distance in vehicle frame.
    double actual_lookahead = std::hypot(target_vehicle.x, target_vehicle.y);
    if (actual_lookahead < 0.01) {
        actual_lookahead = 0.01;
    }

    // Pure Pursuit steering law:
    // curvature = 2 * y / L^2, where y is lateral offset in vehicle frame.
    const double curvature = 2.0 * target_vehicle.y / (actual_lookahead * actual_lookahead);
    double steering_angle = std::atan(config_.wheelbase * curvature);
    steering_angle = std::clamp(steering_angle, -config_.max_steering, config_.max_steering);

    // Base speed from trajectory, then apply preview-based regulation.
    double target_speed = target_pt.velocity;

    // Preview curvature over an extended distance (preview_factor * lookahead)
    // to allow braking well before entering tight corners.
    double max_upcoming_curvature = std::abs(closest_pt.curvature);
    double preview_distance = 0.0;
    const double preview_factor = std::max(1.0, config_.curvature_preview_factor);
    const double preview_target = std::max(lookahead_dist * preview_factor, config_.min_lookahead);

    // Search forward until we cover preview_target distance.
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

        // Skip degenerate segments to avoid numerical issues.
        if (segment_dist <= 1e-9) {
            continue;
        }

        // If preview cutoff is inside this segment, interpolate curvature.
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

    // Curvature-based speed scaling with a floor to prevent excessive slowdown.
    const double floor_ratio = std::clamp(config_.curvature_speed_floor_ratio, 0.0, 1.0);
    double curvature_speed_scale =
        1.0 / (1.0 + config_.curvature_speed_factor * max_upcoming_curvature);
    curvature_speed_scale = std::clamp(curvature_speed_scale, floor_ratio, 1.0);
    target_speed *= curvature_speed_scale;

    // Additional slowdown when cross-track error grows.
    const double cte_floor_ratio = std::clamp(config_.cte_speed_floor_ratio, 0.0, 1.0);
    double cte_speed_scale = 1.0 / (1.0 + config_.cte_speed_factor * std::abs(output.cross_track_error));
    cte_speed_scale = std::clamp(cte_speed_scale, cte_floor_ratio, 1.0);
    target_speed *= cte_speed_scale;

    // Corridor-aware speed scaling in narrow sections.
    if (have_bounds) {
        if (usable_half_width <= 0.0) {
            target_speed = 0.0;
        } else {
            const double corridor_ref = std::max(0.05, config_.corridor_half_width_ref);
            const double corridor_floor = std::clamp(config_.corridor_speed_floor_ratio, 0.0, 1.0);
            double corridor_speed_scale = usable_half_width / corridor_ref;
            corridor_speed_scale = std::clamp(corridor_speed_scale, corridor_floor, 1.0);
            target_speed *= corridor_speed_scale;
        }
    }

    // Physics-aware lateral acceleration cap: v <= sqrt(a_lat_max / |kappa|).
    const double kappa_preview = std::max(max_upcoming_curvature, std::abs(curvature));
    if (config_.max_lateral_accel > 1e-3 && kappa_preview > 1e-5) {
        const double v_lat_limit = std::sqrt(config_.max_lateral_accel / kappa_preview);
        target_speed = std::min(target_speed, v_lat_limit);
    }

    target_speed = std::max(config_.min_regulated_speed, target_speed);

    // Fill output.
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
