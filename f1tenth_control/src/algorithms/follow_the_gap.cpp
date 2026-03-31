#include "algorithms/follow_the_gap.hpp"

namespace f1tenth_control {

// =====================================================================
// Construction / configuration
// =====================================================================

FollowTheGap::FollowTheGap(const FTGConfig& config)
    : config_(config), lidar_processor_(config.lidar_config), last_steering_(0.0) {}

void FollowTheGap::setConfig(const FTGConfig& config) {
    config_ = config;
    lidar_processor_.setConfig(config.lidar_config);
}

void FollowTheGap::reset() {
    last_steering_ = 0.0;
    smoothed_target_ = 0.0;
    first_compute_ = true;
}

// =====================================================================
// Main compute
// =====================================================================

FTGOutput FollowTheGap::compute(
    const std::vector<float>& ranges,
    double angle_min,
    double angle_max,
    double angle_increment
) {
    FTGOutput output;

    // --- Time step ---
    auto now = std::chrono::steady_clock::now();
    double dt = 0.025;  // default ~40 Hz

    // Calculate actual dt on subsequent calls for time-based rate limiting
    if (!first_compute_) {
        dt = std::chrono::duration<double>(now - last_compute_time_).count();
        dt = std::clamp(dt, 0.001, 0.5);
    }
    last_compute_time_ = now;

    // --- Handle empty scan ---
    if (ranges.empty()) {
        output.emergency_stop = true;
        output.command = DriveCommand(0.0, 0.0);
        return output;
    }

    // --- Step 1: Generic LiDAR preprocessing (median filter, range clip) ---
    ProcessedScan scan = lidar_processor_.processScan(
        ranges, angle_min, angle_max, angle_increment
    );

    // If no valid points after preprocessing, trigger emergency stop
    if (scan.filtered_ranges.empty()) {
        output.emergency_stop = true;
        output.command = DriveCommand(0.0, 0.0);
        return output;
    }

    // Initialise blocking flags (for visualisation)
    scan.disparity_blocked.assign(scan.filtered_ranges.size(), false);
    scan.bubble_blocked.assign(scan.filtered_ranges.size(), false);

    // --- Step 2: Wall margin  ---
    applyWallMargin(scan);

    // --- Step 3: Closest-point detection ---
    output.closest_point_idx = lidar_processor_.findClosestPoint(scan);
    output.closest_point_dist = scan.filtered_ranges[output.closest_point_idx];

    // Check if closest point is below emergency brake threshold
    if (output.closest_point_dist < config_.emergency_brake_distance) {
        output.emergency_stop = true;
        output.command = DriveCommand(0.0, 0.0);
        output.processed_scan = scan;

        // Still populate gaps for viz
        output.all_gaps = findGapsForViz(scan);
        return output;
    }

    // --- Step 4: Disparity extension ---
    applyDisparityExtension(scan);

    // --- Step 5: Compute effective clearance per beam ---
    std::vector<double> eff_clearance = computeEffectiveClearance(scan);

    // --- Step 6: Compute weighted-centroid target angle ---
    double target_angle = computeTargetAngle(scan, eff_clearance);

    // --- Step 7: EMA smoothing on the target angle ---
    // Not applied on first compute
    // Then creates a smoothing effect that helps prevent 
    // oscillations when target angle changes rapidly between 
    // different directions.
    if (first_compute_) {
        smoothed_target_ = target_angle;
        first_compute_ = false;
    } else {
        smoothed_target_ = config_.target_ema_alpha * target_angle + (1.0 - config_.target_ema_alpha) * smoothed_target_;
    }

    // Find raw steering command from smoothed target, applying gain and saturation
    double raw_steering = std::clamp(
        config_.steering_gain * smoothed_target_,
        -config_.max_steering,
        config_.max_steering
    );

    // --- Step 8: Time-based steering rate limiting ---
    double steering = rateLimitSteering(raw_steering, last_steering_, dt);
    last_steering_ = steering;

    // --- Step 9: Speed from forward clearance + steering ---
    // Forward clearance: average effective clearance in the central ±10 deg cone
    double fwd_clearance = 0.0;
    int fwd_count = 0;
    const double fwd_cone = 0.175;  // ~10 degrees

    // Loop through all beams and average effective clearance 
    // for valid beams within the forward cone
    for (size_t i = 0; i < scan.filtered_ranges.size(); ++i) {
        if (std::abs(scan.angles[i]) <= fwd_cone && scan.valid[i]) {
            fwd_clearance += eff_clearance[i];
            ++fwd_count;
        }
    }
    // If no valid beams in forward cone, assume some small clearance to avoid zero speed
    fwd_clearance = (fwd_count > 0) ? fwd_clearance / fwd_count : 0.5;

    // Calculate speed command based on forward clearance and steering angle
    double speed = calculateSpeed(fwd_clearance, steering);
    output.command = DriveCommand(speed, steering);

    // --- Step 10: Populate gaps for visualisation ---
    output.all_gaps = findGapsForViz(scan);
    output.selected_gap = findBestGapForViz(output.all_gaps);
    output.processed_scan = scan;

    return output;
}

// =====================================================================
// LiDAR safety processing
// =====================================================================

void FollowTheGap::applyWallMargin(ProcessedScan& scan) {
    // If wall margin is zero or negative, skip this step
    if (config_.wall_margin <= 0.0) return;

    // Loop through all beams and shrink valid ranges by wall margin
    for (size_t i = 0; i < scan.filtered_ranges.size(); ++i) {
        if (scan.valid[i] && scan.filtered_ranges[i] > config_.wall_margin) {
            scan.filtered_ranges[i] -= config_.wall_margin;
        } else if (scan.valid[i]) {
            scan.filtered_ranges[i] = 0.0;
            scan.valid[i] = false;
        }
    }
}

void FollowTheGap::applyDisparityExtension(ProcessedScan& scan) {
    // If scan has fewer than 2 beams, skip disparity extension
    if (scan.filtered_ranges.size() < 2) return;

    // For convenience, create a reference to the filtered ranges vector
    std::vector<double>& ranges = scan.filtered_ranges;
    const double half_car = config_.car_width / 2.0;
    const auto& lidar_config = lidar_processor_.getConfig();

    // Cap extension to ~15 degrees worth of beams
    const int max_extension = static_cast<int>(
        std::ceil(0.26 / std::abs(scan.angle_increment))
    );

    // Loop through beams and look for large jumps in range (disparities)
    for (size_t i = 1; i < ranges.size(); ++i) {
        if (!scan.valid[i] || !scan.valid[i - 1]) continue;

        // Check if either beam is outside the configured angular processing range
        double angle_i   = scan.angles[i];
        double angle_im1 = scan.angles[i - 1];
        if (angle_i   < lidar_config.angle_min || angle_i   > lidar_config.angle_max) continue;
        if (angle_im1 < lidar_config.angle_min || angle_im1 > lidar_config.angle_max) continue;

        // Check for large disparity
        double diff = std::abs(ranges[i] - ranges[i - 1]);
        if (diff <= config_.disparity_threshold) continue;

        // Determine which beam is closer and calculate how many neighboring beams to block
        size_t closer_idx   = (ranges[i] < ranges[i - 1]) ? i : i - 1;
        double closer_range = ranges[closer_idx];

        // Skip cascade extensions
        if (scan.disparity_blocked[closer_idx]) continue;

        // Calculate angle to extend based on geometry (car width and closer range)
        double angle_to_extend = std::atan2(half_car, closer_range);
        int indices_to_extend  = std::min(
            static_cast<int>(std::ceil(angle_to_extend / std::abs(scan.angle_increment))),
            max_extension
        );

        // Extend the closer beam's range to the farther beam and mark intermediate beams as blocked
        if (closer_idx == i) {
            // Closer on the right -> extend left
            for (int j = 0; j < indices_to_extend && static_cast<int>(i) - j >= 0; ++j) {
                size_t idx = i - j;
                if (scan.valid[idx] && ranges[idx] > closer_range) {
                    scan.disparity_blocked[idx] = true;
                    ranges[idx] = closer_range;
                }
            }
        } else {
            // Closer on the left -> extend right
            size_t base = i - 1;
            for (int j = 0; j < indices_to_extend && base + j < ranges.size(); ++j) {
                size_t idx = base + j;
                if (scan.valid[idx] && ranges[idx] > closer_range) {
                    scan.disparity_blocked[idx] = true;
                    ranges[idx] = closer_range;
                }
            }
        }
    }
}

// =====================================================================
// Weighted free-space core
// =====================================================================

std::vector<double> FollowTheGap::computeEffectiveClearance(const ProcessedScan& scan) {
    // If scan is empty, return empty effective clearance vector
    const size_t n = scan.filtered_ranges.size();
    std::vector<double> eff(n, 0.0);
    if (n == 0) return eff;

    // Precompute constants for cone calculation
    const double half_car = (config_.car_width / 2.0) * config_.clearance_cone_scale;
    const double abs_inc  = std::abs(scan.angle_increment);
    const auto& lidar_config = lidar_processor_.getConfig();

    // Loop through each beam and compute effective clearance 
    // based on minimum range in the clearance cone
    for (size_t i = 0; i < n; ++i) {
        double angle = scan.angles[i];
        // Skip beams outside the configured angular processing range or invalid beams
        if (angle < lidar_config.angle_min || angle > lidar_config.angle_max) {
            eff[i] = 0.0;
            continue;
        }
        // If beam is invalid, effective clearance is zero
        if (!scan.valid[i]) {
            eff[i] = 0.0;
            continue;
        }
        // If range is below minimum scoring range, effective clearance is zero
        double range_i = scan.filtered_ranges[i];
        if (range_i < config_.min_score_range) {
            eff[i] = 0.0;
            continue;
        }

        // Number of neighbouring beams to check (based on car width at this range)
        double cone_angle = std::atan2(half_car, range_i);
        int cone_idx = static_cast<int>(std::ceil(cone_angle / abs_inc));
        cone_idx = std::max(cone_idx, 1);  // at least 1 neighbour

        // Find minimum range in the cone ("effective" clearance)
        double min_range = range_i;
        for (int d = -cone_idx; d <= cone_idx; ++d) {
            int idx = static_cast<int>(i) + d;
            if (idx < 0 || idx >= static_cast<int>(n)) continue;
            double r = scan.filtered_ranges[idx];
            // Include invalid beams as obstacles (range 0)
            if (!scan.valid[idx]) r = 0.0;
            min_range = std::min(min_range, r);
        }

        // Effective clearance for this beam is the minimum range in its clearance cone
        eff[i] = min_range;
    }

    return eff;
}

double FollowTheGap::computeTargetAngle(
    const ProcessedScan& scan,
    const std::vector<double>& eff_clearance
) {
    // Initialize accumulators for weighted average
    const size_t n = scan.filtered_ranges.size();
    const auto& lidar_config = lidar_processor_.getConfig();

    double sum_score = 0.0;
    double sum_weighted_angle = 0.0;

    // Loop through all beams and compute score based on effective clearance and heading
    for (size_t i = 0; i < n; ++i) {
        double angle = scan.angles[i];

        // Skip beams outside the configured angular processing range or invalid beams
        if (angle < lidar_config.angle_min || angle > lidar_config.angle_max) continue;

        double clearance = eff_clearance[i];
        // Skip beams that are too close to be considered drivable
        if (clearance < config_.min_score_range) continue;

        // Score = (clearance - min_score_range) ^ power  *  exp(-heading_weight * |angle|)
        double base = clearance - config_.min_score_range;
        double score = std::pow(base, config_.score_power)
                     * std::exp(-config_.heading_weight * std::abs(angle));

        // Accumulate weighted angle and total score
        sum_score += score;
        sum_weighted_angle += angle * score;
    }

    if (sum_score < 1e-9) {
        // No drivable direction found — default to straight ahead
        return 0.0;
    }

    return sum_weighted_angle / sum_score;
}

// =====================================================================
// Gap detection (visualisation only)
// =====================================================================

std::vector<Gap> FollowTheGap::findGapsForViz(const ProcessedScan& scan) {
    std::vector<Gap> gaps;
    if (scan.filtered_ranges.empty()) return gaps;

    const auto& lidar_config = lidar_processor_.getConfig();
    bool in_gap = false;
    Gap current_gap;
    size_t current_gap_count = 0;

    for (size_t i = 0; i < scan.filtered_ranges.size(); ++i) {
        double range = scan.filtered_ranges[i];
        double angle = scan.angles[i];
        if (angle < lidar_config.angle_min || angle > lidar_config.angle_max) continue;

        bool is_gap = (range >= config_.gap_threshold && scan.valid[i]);

        if (is_gap && !in_gap) {
            in_gap = true;
            current_gap = Gap();
            current_gap.start_idx = i;
            current_gap.start_angle = angle;
            current_gap.min_range = range;
            current_gap.max_range = range;
            current_gap.deepest_idx = i;
            current_gap.deepest_range = range;
            current_gap.avg_range = range;
            current_gap_count = 1;
        } else if (is_gap && in_gap) {
            current_gap.min_range = std::min(current_gap.min_range, range);
            if (range > current_gap.deepest_range) {
                current_gap.deepest_range = range;
                current_gap.deepest_idx = i;
            }
            current_gap.max_range = std::max(current_gap.max_range, range);
            current_gap.avg_range += range;
            ++current_gap_count;
        } else if (!is_gap && in_gap) {
            in_gap = false;
            current_gap.end_idx = i - 1;
            current_gap.end_angle = scan.angles[i - 1];
            current_gap.angular_width = current_gap.end_angle - current_gap.start_angle;
            if (current_gap_count > 0) {
                current_gap.avg_range /= static_cast<double>(current_gap_count);
            }
            if (current_gap.angular_width >= config_.min_gap_width) {
                gaps.push_back(current_gap);
            }
        }
    }

    if (in_gap) {
        current_gap.end_idx = scan.filtered_ranges.size() - 1;
        current_gap.end_angle = scan.angles.back();
        current_gap.angular_width = current_gap.end_angle - current_gap.start_angle;
        if (current_gap_count > 0) {
            current_gap.avg_range /= static_cast<double>(current_gap_count);
        }
        if (current_gap.angular_width >= config_.min_gap_width) {
            gaps.push_back(current_gap);
        }
    }

    return gaps;
}

Gap FollowTheGap::findBestGapForViz(const std::vector<Gap>& gaps) {
    if (gaps.empty()) return Gap();

    // Pick widest gap that is closest to straight ahead
    double best = -std::numeric_limits<double>::infinity();
    const Gap* best_gap = &gaps[0];
    for (const auto& g : gaps) {
        double score = g.angular_width * g.deepest_range
                     * std::exp(-0.5 * std::abs(g.centerAngle()));
        if (score > best) {
            best = score;
            best_gap = &g;
        }
    }
    return *best_gap;
}

// =====================================================================
// Control
// =====================================================================

double FollowTheGap::calculateSpeed(double forward_clearance, double steering_angle) {
    // Range factor: how far ahead is clear
    double range_factor = std::clamp(forward_clearance / config_.speed_full_range, 0.0, 1.0);

    // Steering factor: slow down when turning
    double abs_steer = std::abs(steering_angle);
    double steer_factor = 1.0 - config_.steer_slowdown_gain * (abs_steer / config_.max_steering);
    steer_factor = std::clamp(steer_factor, 0.3, 1.0);

    double speed = config_.min_speed + (config_.max_speed - config_.min_speed) * range_factor * steer_factor;
    return std::clamp(speed, config_.min_speed, config_.max_speed);
}

double FollowTheGap::rateLimitSteering(double target, double last, double dt) {
    // Calculate maximum allowed change based on configured max steering rate
    // Might be redundant when driving on a real car since the physical 
    // steering mechanism already has rate limits,
    double max_change = config_.max_steering_rate * dt;
    double delta = target - last;
    if (std::abs(delta) > max_change) {
        return last + ((delta > 0) ? max_change : -max_change);
    }
    return target;
}

}  // namespace f1tenth_control
