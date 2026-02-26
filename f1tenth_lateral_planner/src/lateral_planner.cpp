#include "f1tenth_lateral_planner/lateral_planner.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <limits>

namespace f1tenth_lateral_planner
{

// ═════════════════════════════════════════════════════════════════════
//  Construction
// ═════════════════════════════════════════════════════════════════════

LateralPlanner::LateralPlanner(rclcpp::Logger logger, const Parameters & params)
: logger_(logger), params_(params)
{
}

// ═════════════════════════════════════════════════════════════════════
//  Raceline I/O
// ═════════════════════════════════════════════════════════════════════

bool LateralPlanner::loadTrajectory(const std::string & csv_path)
{
  waypoints_.clear();
  modified_raceline_.clear();

  std::ifstream file(csv_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(logger_, "Cannot open trajectory file: %s", csv_path.c_str());
    return false;
  }

  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::istringstream ss(line);
    std::string token;
    std::vector<double> values;

    while (std::getline(ss, token, ',')) {
      try {
        values.push_back(std::stod(token));
      } catch (...) {
        break;
      }
    }

    if (values.size() >= 7) {
      Waypoint wp;
      wp.s     = values[0];
      wp.x     = values[1];
      wp.y     = values[2];
      wp.psi   = values[3];
      wp.kappa = values[4];
      wp.vx    = values[5];
      wp.ax    = values[6];
      waypoints_.push_back(wp);
    }
  }

  // Start with an unmodified copy
  modified_raceline_ = waypoints_;

  RCLCPP_INFO(logger_, "Loaded %zu waypoints from %s", waypoints_.size(), csv_path.c_str());
  if (!waypoints_.empty()) {
    RCLCPP_INFO(logger_, "  Track length: %.1f m", waypoints_.back().s);
  }
  return !waypoints_.empty();
}

// ═════════════════════════════════════════════════════════════════════
//  State updates
// ═════════════════════════════════════════════════════════════════════

void LateralPlanner::updateRobotPose(double x, double y, double yaw)
{
  robot_.x   = x;
  robot_.y   = y;
  robot_.yaw = yaw;
}

void LateralPlanner::updateSpeed(double speed)
{
  current_speed_ = speed;
}

void LateralPlanner::processObstacleScan(
  const std::vector<float> & ranges,
  float angle_min, float angle_increment,
  float range_min, float range_max,
  double laser_x, double laser_y, double laser_yaw)
{
  std::vector<double> xs, ys;

  for (size_t i = 0; i < ranges.size(); ++i) {
    float r = ranges[i];
    if (!std::isfinite(r) || r <= range_min || r >= range_max) {
      continue;
    }

    double angle = static_cast<double>(angle_min) +
                   static_cast<double>(i) * static_cast<double>(angle_increment);
    double world_angle = angle + laser_yaw;

    xs.push_back(laser_x + r * std::cos(world_angle));
    ys.push_back(laser_y + r * std::sin(world_angle));
  }

  if (xs.empty()) {
    clearOpponent();
    return;
  }

  // Compute centroid
  double cx = 0.0, cy = 0.0;
  for (size_t i = 0; i < xs.size(); ++i) {
    cx += xs[i];
    cy += ys[i];
  }
  cx /= static_cast<double>(xs.size());
  cy /= static_cast<double>(ys.size());

  // Estimate width
  double first_angle = 0.0, last_angle = 0.0;
  double mean_range = 0.0;
  size_t count = 0;
  for (size_t i = 0; i < ranges.size(); ++i) {
    float r = ranges[i];
    if (!std::isfinite(r) || r <= range_min || r >= range_max) {
      continue;
    }
    double angle = static_cast<double>(angle_min) +
                   static_cast<double>(i) * static_cast<double>(angle_increment) +
                   laser_yaw;
    if (count == 0) first_angle = angle;
    last_angle = angle;
    mean_range += r;
    ++count;
  }
  mean_range /= static_cast<double>(count);
  double width = std::abs(last_angle - first_angle) * mean_range;
  width = std::max(width, 0.15);

  opponent_.x        = cx;
  opponent_.y        = cy;
  opponent_.width    = width;
  opponent_.detected = true;
}

void LateralPlanner::clearOpponent()
{
  opponent_.detected = false;
  // If opponent disappears, unlock the avoidance path
  if (avoidance_active_) {
    RCLCPP_INFO(logger_, "Opponent lost — returning to original raceline");
    avoidance_active_ = false;
    committed_side_   = 0.0;
    modified_raceline_ = waypoints_;
  }
}

// ═════════════════════════════════════════════════════════════════════
//  Planning — top-level
// ═════════════════════════════════════════════════════════════════════

std::vector<Waypoint> LateralPlanner::computePath()
{
  if (waypoints_.empty()) {
    return {};
  }

  if (!opponent_.detected) {
    // No obstacle → use original raceline
    if (avoidance_active_) {
      avoidance_active_ = false;
      committed_side_ = 0.0;
      modified_raceline_ = waypoints_;
    }
    return extractSegment();
  }

  // ── Opponent detected ──────────────────────────────────────────

  // Check if we should unlock (car has passed opponent)
  if (avoidance_active_ && hasPassedOpponent()) {
    RCLCPP_INFO(logger_, "Passed opponent — returning to original raceline");
    avoidance_active_ = false;
    committed_side_ = 0.0;
    modified_raceline_ = waypoints_;
    return extractSegment();
  }

  // Distance check — don't plan if too close (and not already locked)
  double dx = opponent_.x - robot_.x;
  double dy = opponent_.y - robot_.y;
  double dist = std::sqrt(dx * dx + dy * dy);
  if (dist < params_.min_replan_dist_m && !avoidance_active_) {
    return extractSegmentFromModified();
  }

  // If avoidance is already active and opponent hasn't moved much, keep locked path
  if (avoidance_active_ && !hasOpponentMoved()) {
    return extractSegmentFromModified();
  }

  // Build (or rebuild) the avoidance path
  buildAvoidancePath();
  return extractSegmentFromModified();
}

// ═════════════════════════════════════════════════════════════════════
//  Segment extraction (with wraparound for closed tracks)
// ═════════════════════════════════════════════════════════════════════

std::vector<Waypoint> LateralPlanner::extractSegment() const
{
  const size_t n = waypoints_.size();
  if (n == 0) return {};

  const size_t robot_idx = closestWaypoint(robot_.x, robot_.y);
  const size_t behind = static_cast<size_t>(params_.lookbehind_points);
  const size_t ahead  = static_cast<size_t>(params_.lookahead_points);

  std::vector<Waypoint> result;
  result.reserve(behind + ahead);

  for (size_t k = 0; k < behind + ahead; ++k) {
    // Wraparound index: start from (robot_idx - behind), modulo n
    size_t idx = (robot_idx + n - behind + k) % n;
    result.push_back(waypoints_[idx]);
  }

  return result;
}

std::vector<Waypoint> LateralPlanner::extractSegmentFromModified() const
{
  const size_t n = modified_raceline_.size();
  if (n == 0) return {};

  const size_t robot_idx = closestWaypoint(robot_.x, robot_.y);
  const size_t behind = static_cast<size_t>(params_.lookbehind_points);
  const size_t ahead  = static_cast<size_t>(params_.lookahead_points);

  std::vector<Waypoint> result;
  result.reserve(behind + ahead);

  for (size_t k = 0; k < behind + ahead; ++k) {
    size_t idx = (robot_idx + n - behind + k) % n;
    result.push_back(modified_raceline_[idx]);
  }

  return result;
}

// ═════════════════════════════════════════════════════════════════════
//  Avoidance path building — called once per detection, then locked
// ═════════════════════════════════════════════════════════════════════

void LateralPlanner::buildAvoidancePath()
{
  // Start from the pristine original raceline
  modified_raceline_ = waypoints_;

  // Project opponent and car onto raceline
  size_t opp_idx = closestWaypoint(opponent_.x, opponent_.y);
  const Waypoint & opp_wp = waypoints_[opp_idx];
  size_t car_idx = closestWaypoint(robot_.x, robot_.y);
  const Waypoint & car_wp = waypoints_[car_idx];

  // Decide passing side — if already committed, keep that side
  double pass_dir;
  if (committed_side_ != 0.0) {
    // Check if the committed side is still safe:
    // Opponent lateral offset relative to the raceline
    double dx = opponent_.x - opp_wp.x;
    double dy = opponent_.y - opp_wp.y;
    double normal_angle = opp_wp.psi + M_PI / 2.0;
    double opp_lateral = dx * std::cos(normal_angle) + dy * std::sin(normal_angle);

    // If opponent has moved to block our committed side, switch
    // (opponent is on the same side we want to pass on)
    bool blocked = (committed_side_ > 0.0 && opp_lateral < -0.3) ||
                   (committed_side_ < 0.0 && opp_lateral > 0.3);
    if (blocked) {
      pass_dir = -committed_side_;  // switch to the other side
      RCLCPP_WARN(logger_, "Committed side blocked — switching to %.0f", pass_dir);
    } else {
      pass_dir = committed_side_;  // keep committed side
    }
  } else {
    pass_dir = decidePassingSide(opp_idx);
  }

  double shift_mag = computeShiftMagnitude(opp_idx);
  double d_max = pass_dir * shift_mag;

  // Lead distance: from car to opponent
  double total_s = waypoints_.back().s;
  double dist_to_opp = opp_wp.s - car_wp.s;
  if (dist_to_opp < 0.0) dist_to_opp += total_s;  // wraparound

  // Shift starts at the car's position → path starts on the raceline
  double s_start = car_wp.s;
  double lead_dist = dist_to_opp;

  // Trail distance after the opponent
  double speed_trail = current_speed_ * params_.window_time_s;
  double trail_dist = std::max(params_.min_window_m * 0.3, speed_trail);
  double s_end = opp_wp.s + trail_dist;

  RCLCPP_INFO(logger_,
    "Avoidance: side=%.0f, shift=%.2fm, lead=%.1fm, trail=%.1fm",
    pass_dir, shift_mag, lead_dist, trail_dist);

  // Apply the shift to modified_raceline_
  applyLateralShift(s_start, opp_wp.s, s_end, d_max);

  // Lock in the avoidance state
  avoidance_active_  = true;
  committed_side_    = pass_dir;
  committed_opp_idx_ = opp_idx;
  committed_opp_x_   = opponent_.x;
  committed_opp_y_   = opponent_.y;
}

// ═════════════════════════════════════════════════════════════════════
//  Lateral shift — piecewise half-cosine (peak at opponent)
// ═════════════════════════════════════════════════════════════════════

void LateralPlanner::applyLateralShift(
  double s_start, double opp_s, double s_end, double d_max)
{
  const double total_s = waypoints_.back().s;

  // Compute lead/trail distances with wraparound normalization
  double lead_dist  = opp_s - s_start;
  if (lead_dist < 0.0) lead_dist += total_s;
  double trail_dist = s_end - opp_s;
  if (trail_dist < 0.0) trail_dist += total_s;
  double window_len = lead_dist + trail_dist;
  size_t affected = 0;

  for (size_t i = 0; i < modified_raceline_.size(); ++i) {
    const Waypoint & orig = waypoints_[i];

    // Compute arc-length distance from s_start, handling wraparound
    double s_rel = orig.s - s_start;
    // Normalize to [0, total_s)
    while (s_rel < 0.0)      s_rel += total_s;
    while (s_rel >= total_s)  s_rel -= total_s;

    double offset = 0.0;
    if (s_rel >= 0.0 && s_rel <= window_len && window_len > 0.0) {
      if (s_rel <= lead_dist && lead_dist > 0.0) {
        // Ramp-up phase: 0 → d_max using half-cosine
        offset = d_max * 0.5 * (1.0 - std::cos(M_PI * s_rel / lead_dist));
      } else if (trail_dist > 0.0) {
        // Ramp-down phase: d_max → 0 using half-cosine
        double t = s_rel - lead_dist;
        offset = d_max * 0.5 * (1.0 + std::cos(M_PI * t / trail_dist));
      }

      // Per-waypoint wall clamp: keep |offset| within wall limit
      double wall_limit = maxAllowedShift();
      if (std::abs(offset) > wall_limit) {
        offset = std::copysign(wall_limit, offset);
      }

      ++affected;
    }

    // Shift perpendicular to heading
    double normal = orig.psi + M_PI / 2.0;
    modified_raceline_[i].x = orig.x + offset * std::cos(normal);
    modified_raceline_[i].y = orig.y + offset * std::sin(normal);

    // Reduce velocity in shifted sections
    if (params_.max_lateral_shift_m > 0.0 && std::abs(offset) > 0.001) {
      double speed_scale = 1.0 - 0.2 * std::abs(offset / params_.max_lateral_shift_m);
      modified_raceline_[i].vx = orig.vx * speed_scale;
    }
  }

  RCLCPP_INFO(logger_, "Shifted %zu waypoints (lead=%.1fm, trail=%.1fm)",
              affected, lead_dist, trail_dist);
}

// ═════════════════════════════════════════════════════════════════════
//  Side decision and shift magnitude
// ═════════════════════════════════════════════════════════════════════

double LateralPlanner::decidePassingSide(size_t opp_idx) const
{
  const Waypoint & opp_wp = waypoints_[opp_idx];

  double dx = opponent_.x - opp_wp.x;
  double dy = opponent_.y - opp_wp.y;
  double normal_angle = opp_wp.psi + M_PI / 2.0;
  double opp_lateral = dx * std::cos(normal_angle) + dy * std::sin(normal_angle);

  // Maximum shift we can achieve (wall limit)
  double wall_limit = maxAllowedShift();

  // Needed shift = half opponent + safety + how far it is off-centre
  double needed = opponent_.width / 2.0 + params_.safety_margin_m + std::abs(opp_lateral);

  // Preferred side: opposite to opponent's lateral offset
  double preferred = (opp_lateral > 0.0) ? -1.0 : 1.0;

  // If the preferred side has enough room, use it
  if (needed <= wall_limit) {
    return preferred;
  }

  // Not enough room on preferred side — try the other side
  // (other side only needs to clear the opponent's extent on that side)
  RCLCPP_WARN(logger_,
    "Preferred side (%.0f) needs %.2fm but wall allows %.2fm — trying other side",
    preferred, needed, wall_limit);
  return -preferred;
}

double LateralPlanner::maxAllowedShift() const
{
  // Max offset from raceline before hitting the wall
  // track_half_width - half car width - wall safety
  return std::max(
    params_.track_half_width_m - params_.car_width_m / 2.0 - params_.wall_safety_margin_m,
    0.05);
}

double LateralPlanner::computeShiftMagnitude(size_t opp_idx) const
{
  const Waypoint & opp_wp = waypoints_[opp_idx];

  double dx = opponent_.x - opp_wp.x;
  double dy = opponent_.y - opp_wp.y;
  double normal_angle = opp_wp.psi + M_PI / 2.0;
  double opp_lateral = dx * std::cos(normal_angle) + dy * std::sin(normal_angle);

  double shift = opponent_.width / 2.0 + params_.safety_margin_m + std::abs(opp_lateral);

  // Clamp to the lesser of user max and wall-limited max
  double wall_limit = maxAllowedShift();
  return std::min({shift, params_.max_lateral_shift_m, wall_limit});
}

// ═════════════════════════════════════════════════════════════════════
//  Lock management
// ═════════════════════════════════════════════════════════════════════

bool LateralPlanner::hasPassedOpponent() const
{
  size_t robot_idx = closestWaypoint(robot_.x, robot_.y);
  double robot_s   = waypoints_[robot_idx].s;
  double opp_s     = waypoints_[committed_opp_idx_].s;

  double delta_s = robot_s - opp_s;

  // Handle wraparound (closed track)
  double total_s = waypoints_.back().s;
  if (delta_s < -total_s / 2.0) delta_s += total_s;
  if (delta_s >  total_s / 2.0) delta_s -= total_s;

  return delta_s > params_.pass_complete_margin;
}

bool LateralPlanner::hasOpponentMoved() const
{
  double dx = opponent_.x - committed_opp_x_;
  double dy = opponent_.y - committed_opp_y_;
  double dist = std::sqrt(dx * dx + dy * dy);
  return dist > params_.opponent_move_thresh;
}

// ═════════════════════════════════════════════════════════════════════
//  Helpers
// ═════════════════════════════════════════════════════════════════════

size_t LateralPlanner::closestWaypoint(double x, double y) const
{
  size_t best = 0;
  double best_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < waypoints_.size(); ++i) {
    double dx = waypoints_[i].x - x;
    double dy = waypoints_[i].y - y;
    double d2 = dx * dx + dy * dy;
    if (d2 < best_dist) {
      best_dist = d2;
      best = i;
    }
  }

  return best;
}

}  // namespace f1tenth_lateral_planner
