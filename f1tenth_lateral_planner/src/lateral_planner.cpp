#include "f1tenth_lateral_planner/lateral_planner.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace f1tenth_lateral_planner {

// ── Construction ──────────────────────────────────────────────────────────────

LateralPlanner::LateralPlanner(const LateralPlannerConfig& config)
    : config_(config)
{
}

// ── Raceline loading ──────────────────────────────────────────────────────────

bool LateralPlanner::loadRaceline(const std::string& csv_path)
{
    std::ifstream file(csv_path);
    if (!file.is_open()) {
        return false;
    }

    raceline_.clear();
    std::string line;
    while (std::getline(file, line)) {
        // Skip blank lines and comment lines
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::stringstream ss(line);
        std::string token;
        std::vector<double> vals;
        while (std::getline(ss, token, ',')) {
            try {
                vals.push_back(std::stod(token));
            } catch (...) {
                break;
            }
        }

        // Require at least 7 columns: s, x, y, psi, kappa, vx, ax
        if (vals.size() >= 7) {
            Waypoint wp;
            wp.s     = vals[0];
            wp.x     = vals[1];
            wp.y     = vals[2];
            wp.psi   = vals[3];
            wp.kappa = vals[4];
            wp.vx    = vals[5];
            wp.ax    = vals[6];
            raceline_.push_back(wp);
        }
    }

    return !raceline_.empty();
}

// ── Closest-waypoint search ───────────────────────────────────────────────────

size_t LateralPlanner::findClosestWaypoint(double x, double y, size_t hint) const
{
    if (raceline_.empty()) {
        return 0;
    }

    const size_t n = raceline_.size();
    const size_t search_radius = 30;

    // Local search window around hint
    const size_t lo = (hint > search_radius) ? (hint - search_radius) : 0;
    const size_t hi = std::min(n, hint + search_radius);

    size_t best_idx = lo;
    double best_dist_sq = std::numeric_limits<double>::max();

    for (size_t i = lo; i < hi; ++i) {
        double dx = raceline_[i].x - x;
        double dy = raceline_[i].y - y;
        double dist_sq = dx * dx + dy * dy;
        if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            best_idx = i;
        }
    }

    // If the best point is at the boundary, fall back to a full search
    if (best_idx == lo || best_idx == hi - 1) {
        for (size_t i = 0; i < n; ++i) {
            double dx = raceline_[i].x - x;
            double dy = raceline_[i].y - y;
            double dist_sq = dx * dx + dy * dy;
            if (dist_sq < best_dist_sq) {
                best_dist_sq = dist_sq;
                best_idx = i;
            }
        }
    }

    return best_idx;
}

// ── Total length ──────────────────────────────────────────────────────────────

double LateralPlanner::totalLength() const
{
    return raceline_.empty() ? 0.0 : raceline_.back().s;
}

// ── Pass-side decision ────────────────────────────────────────────────────────

double LateralPlanner::decidePassDirection(
    double opp_x, double opp_y, size_t opp_idx) const
{
    // Project opponent onto the normal of the nearest raceline point
    const Waypoint& wp = raceline_[opp_idx];
    double normal_angle = wp.psi + M_PI / 2.0;
    double cn = std::cos(normal_angle);
    double sn = std::sin(normal_angle);
    double opp_lateral = (opp_x - wp.x) * cn + (opp_y - wp.y) * sn;

    // Opponent to the left (+) → pass on the right (-1)
    // Opponent to the right (-) → pass on the left (+1)
    return (opp_lateral > 0.0) ? -1.0 : 1.0;
}

// ── Cosine-blend offset computation ──────────────────────────────────────────

std::vector<double> LateralPlanner::computeOffsets(
    double robot_s, double opp_s,
    double total_s, double d_max,
    double current_speed) const
{
    const size_t n = raceline_.size();
    std::vector<double> offsets(n, 0.0);

    // Half-window: how far past the opponent to keep the shift before ramping back
    double half_window = std::max(
        config_.min_window_m, current_speed * config_.window_time_s);

    // Ramp-up length: distance from robot to opponent along the raceline
    double ramp_up_raw = opp_s - robot_s;
    if (ramp_up_raw < 0.0) {
        ramp_up_raw += total_s;
    }
    double ramp_up_len   = std::max(ramp_up_raw, config_.min_window_m);
    double ramp_down_len = half_window;
    double s_anchor      = opp_s - ramp_up_len;

    for (size_t i = 0; i < n; ++i) {
        // Arc-length relative to the start of the ramp-up (mod total track)
        double s_rel = std::fmod(
            raceline_[i].s - s_anchor + total_s * 2.0, total_s);

        double offset = 0.0;
        if (s_rel <= ramp_up_len) {
            // Ramp-up: 0 → d_max using a raised-cosine
            double progress = std::min(s_rel / ramp_up_len, 1.0);
            offset = d_max * 0.5 * (1.0 - std::cos(M_PI * progress));
        } else {
            double s_past_peak = s_rel - ramp_up_len;
            if (s_past_peak <= ramp_down_len) {
                // Ramp-down: d_max → 0 using a raised-cosine
                double progress = std::min(s_past_peak / ramp_down_len, 1.0);
                offset = d_max * 0.5 * (1.0 + std::cos(M_PI * progress));
            }
        }

        // Scale down the shift in high-curvature zones (corners)
        double curvature_scale = 1.0 / (1.0 + 5.0 * std::abs(raceline_[i].kappa));
        offsets[i] = offset * curvature_scale;
    }

    return offsets;
}

// ── Apply perpendicular offsets to the raceline ───────────────────────────────

PlannerOutput LateralPlanner::applyOffsets(const std::vector<double>& offsets) const
{
    PlannerOutput out;
    const size_t n = raceline_.size();
    out.xs.resize(n);
    out.ys.resize(n);
    out.psis.resize(n);
    out.vxs.resize(n);

    for (size_t i = 0; i < n; ++i) {
        double normal_angle = raceline_[i].psi + M_PI / 2.0;
        out.xs[i]   = raceline_[i].x + offsets[i] * std::cos(normal_angle);
        out.ys[i]   = raceline_[i].y + offsets[i] * std::sin(normal_angle);
        out.psis[i] = raceline_[i].psi;
        out.vxs[i]  = raceline_[i].vx;
    }

    return out;
}

// ── Full unshifted raceline ───────────────────────────────────────────────────

PlannerOutput LateralPlanner::fullRaceline() const
{
    PlannerOutput out;
    const size_t n = raceline_.size();
    out.xs.resize(n);
    out.ys.resize(n);
    out.psis.resize(n);
    out.vxs.resize(n);

    for (size_t i = 0; i < n; ++i) {
        out.xs[i]   = raceline_[i].x;
        out.ys[i]   = raceline_[i].y;
        out.psis[i] = raceline_[i].psi;
        out.vxs[i]  = raceline_[i].vx;
    }

    return out;
}

// ── Main planning update ──────────────────────────────────────────────────────

PlannerOutput LateralPlanner::update(
    double robot_x, double robot_y,
    bool opp_detected,
    double opp_x, double opp_y, double opp_width,
    double current_speed,
    size_t& robot_hint,
    size_t& opp_hint)
{
    const double total_s = totalLength();

    // Locate the robot on the raceline
    size_t robot_idx = findClosestWaypoint(robot_x, robot_y, robot_hint);
    robot_hint = robot_idx;
    double robot_s = raceline_[robot_idx].s;

    double target_d_max = 0.0;

    if (opp_detected) {
        double dx = opp_x - robot_x;
        double dy = opp_y - robot_y;
        double dist_to_opp = std::sqrt(dx * dx + dy * dy);

        if (dist_to_opp >= config_.min_replan_dist_m) {
            // Find where the opponent sits on the raceline
            size_t opp_idx = findClosestWaypoint(opp_x, opp_y, opp_hint);
            opp_hint = opp_idx;
            double opp_s = raceline_[opp_idx].s;

            // Determine whether the opponent is ahead of the robot
            double s_diff = opp_s - robot_s;
            if (s_diff < 0.0) {
                s_diff += total_s;
            }
            bool opp_ahead = (s_diff <= total_s * 0.5);

            if (opp_ahead) {
                // Lock pass direction on first detection
                if (!avoidance_active_) {
                    locked_pass_dir_  = decidePassDirection(opp_x, opp_y, opp_idx);
                    avoidance_active_ = true;
                }

                // Track the opponent's arc-length position
                locked_opp_s_ = opp_s;

                double shift_mag = std::min(
                    opp_width / 2.0 + config_.safety_margin_m,
                    config_.max_lateral_shift_m);
                target_d_max = locked_pass_dir_ * shift_mag;
            } else {
                // Opponent is behind — avoidance complete
                avoidance_active_ = false;
            }
        } else {
            // Opponent too close to plan around safely
            avoidance_active_ = false;
        }
    } else {
        // No opponent detected
        avoidance_active_ = false;
    }

    // Smooth the shift magnitude with a first-order filter
    smooth_d_max_ += config_.blend_rate * (target_d_max - smooth_d_max_);
    if (std::abs(smooth_d_max_) < 0.005) {
        smooth_d_max_ = 0.0;
    }

    // Publish the plain raceline when no shift is needed
    if (std::abs(smooth_d_max_) < 0.001) {
        return fullRaceline();
    }

    // Compute per-waypoint offsets and apply them
    auto offsets = computeOffsets(
        robot_s, locked_opp_s_, total_s, smooth_d_max_, current_speed);
    return applyOffsets(offsets);
}

}  // namespace f1tenth_lateral_planner
