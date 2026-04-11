#include "f1tenth_lateral_planner/lateral_planner.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <limits>

namespace f1tenth_lateral_planner
{
namespace
{
constexpr int kMinClusterPoints = 3;
}  // namespace

// =============================================================
//  Construction
// =============================================================

LateralPlanner::LateralPlanner(rclcpp::Logger logger, const Parameters & params)
: logger_(logger), params_(params)
{
}

// =============================================================
//  Raceline I/O
// =============================================================

bool LateralPlanner::loadTrajectory(const std::string & csv_path)
{
  waypoints_.clear();
  modified_raceline_.clear();
  resetAvoidance();

  std::ifstream file(csv_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(logger_, "Cannot open trajectory file: %s", csv_path.c_str());
    return false;
  }

  std::string line;
  double prev_s = -std::numeric_limits<double>::infinity();
  bool warned_non_monotonic = false;

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
        values.clear();
        break;
      }
    }

    if (values.size() < 9) {
      continue;
    }

    Waypoint wp;
    wp.s     = values[0];
    wp.x     = values[1];
    wp.y     = values[2];
    wp.psi   = values[3];
    wp.kappa = values[4];
    wp.vx    = values[5];
    wp.ax    = values[6];
    wp.d_left = values[7];
    wp.d_right = values[8];

    if (!std::isfinite(wp.s) || !std::isfinite(wp.x) || !std::isfinite(wp.y) ||
        !std::isfinite(wp.psi) || !std::isfinite(wp.kappa) ||
      !std::isfinite(wp.vx) || !std::isfinite(wp.ax) ||
      !std::isfinite(wp.d_left) || !std::isfinite(wp.d_right))
    {
      continue;
    }

    wp.d_left = std::max(0.0, wp.d_left);
    wp.d_right = std::max(0.0, wp.d_right);

    if (wp.s < prev_s) {
      if (!warned_non_monotonic) {
        RCLCPP_WARN(logger_, "Trajectory s is not monotonic; dropping non-monotonic rows");
        warned_non_monotonic = true;
      }
      continue;
    }

    prev_s = wp.s;
    waypoints_.push_back(wp);
  }

  if (waypoints_.size() < 2) {
    RCLCPP_ERROR(logger_, "Trajectory invalid: need at least 2 waypoints");
    waypoints_.clear();
    return false;
  }

  modified_raceline_ = waypoints_;

  RCLCPP_INFO(logger_, "Loaded %zu waypoints from %s", waypoints_.size(), csv_path.c_str());
  RCLCPP_INFO(logger_, "  Track length: %.2f m", waypoints_.back().s);
  return true;
}

// =============================================================
//  State updates
// =============================================================

void LateralPlanner::updateRobotPose(double x, double y, double yaw)
{
  robot_.x   = x;
  robot_.y   = y;
  robot_.yaw = yaw;
}

void LateralPlanner::updateSpeed(double speed)
{
  if (std::isfinite(speed)) {
    current_speed_ = std::max(0.0, speed);
  }
}

void LateralPlanner::processObstacleScan(
  const std::vector<float> & ranges,
  float angle_min, float angle_increment,
  float range_min, float range_max,
  double laser_x, double laser_y, double laser_yaw)
{
  struct Cluster
  {
    int count = 0;
    double sum_x = 0.0;
    double sum_y = 0.0;
  };

  std::vector<Cluster> clusters;
  Cluster current;
  bool in_cluster = false;

  auto flush_cluster = [&]() {
    if (in_cluster && current.count >= kMinClusterPoints) {
      clusters.push_back(current);
    }
    current = Cluster{};
    in_cluster = false;
  };

  for (size_t i = 0; i < ranges.size(); ++i) {
    const float r = ranges[i];
    const bool valid = std::isfinite(r) && r > range_min && r < range_max;

    if (!valid) {
      flush_cluster();
      continue;
    }

    const double beam_angle = static_cast<double>(angle_min) +
      static_cast<double>(i) * static_cast<double>(angle_increment);
    const double world_angle = beam_angle + laser_yaw;
    const double px = laser_x + static_cast<double>(r) * std::cos(world_angle);
    const double py = laser_y + static_cast<double>(r) * std::sin(world_angle);

    if (!in_cluster) {
      in_cluster = true;
    }

    ++current.count;
    current.sum_x += px;
    current.sum_y += py;
  }
  flush_cluster();

  if (clusters.empty()) {
    clearOpponent();
    return;
  }

  size_t best_idx = 0;
  for (size_t i = 1; i < clusters.size(); ++i) {
    const Cluster & a = clusters[i];
    const Cluster & b = clusters[best_idx];
    if (a.count > b.count) {
      best_idx = i;
      continue;
    }

    if (a.count == b.count) {
      const double a_cx = a.sum_x / static_cast<double>(a.count);
      const double a_cy = a.sum_y / static_cast<double>(a.count);
      const double b_cx = b.sum_x / static_cast<double>(b.count);
      const double b_cy = b.sum_y / static_cast<double>(b.count);
      const double a_dx = a_cx - laser_x;
      const double a_dy = a_cy - laser_y;
      const double b_dx = b_cx - laser_x;
      const double b_dy = b_cy - laser_y;
      const double a_dist2 = a_dx * a_dx + a_dy * a_dy;
      const double b_dist2 = b_dx * b_dx + b_dy * b_dy;
      if (a_dist2 < b_dist2) {
        best_idx = i;
      }
    }
  }

  const Cluster & best = clusters[best_idx];
  const double inv_count = 1.0 / static_cast<double>(best.count);
  const double cx = best.sum_x * inv_count;
  const double cy = best.sum_y * inv_count;

  double opponent_yaw = robot_.yaw;
  if (!waypoints_.empty()) {
    const size_t back_idx = closestWaypoint(cx, cy);
    opponent_yaw = waypoints_[back_idx].psi;
  }
  const double half_length = 0.5 * params_.opponent_length_m;

  opponent_.back_x   = cx;
  opponent_.back_y   = cy;
  opponent_.yaw      = opponent_yaw;
  opponent_.x        = cx + half_length * std::cos(opponent_yaw);
  opponent_.y        = cy + half_length * std::sin(opponent_yaw);
  opponent_.width    = params_.car_width_m;
  opponent_.length   = params_.opponent_length_m;
  opponent_.detected = true;
}

void LateralPlanner::clearOpponent()
{
  opponent_.detected = false;
  if (avoidance_active_ && !merge_back_active_) {
    startMergeBack();
  }
}

// =============================================================
//  Planning - top-level
// =============================================================

std::vector<Waypoint> LateralPlanner::computePath()
{
  if (waypoints_.empty()) {
    return {};
  }

  if (modified_raceline_.size() != waypoints_.size()) {
    modified_raceline_ = waypoints_;
  }

  if (!opponent_.detected) {
    if (avoidance_active_ && !merge_back_active_) {
      startMergeBack();
    }

    if (merge_back_active_) {
      updateMergeBackPath();
      if (mergeBackProgress() >= 1.0) {
        resetAvoidance();
        return extractSegment();
      }
      return extractSegmentFromModified();
    }
    return extractSegment();
  }

  const size_t car_idx = closestWaypoint(robot_.x, robot_.y);
  const size_t opp_idx = closestWaypoint(opponent_.x, opponent_.y);

  double forward_dist = 0.0;
  double lateral_offset = 0.0;
  const bool collision_predicted = isCollisionPredicted(
    car_idx, opp_idx, &forward_dist, &lateral_offset);

  if (!collision_predicted) {
    if (avoidance_active_) {
      const double horizon = std::max(
        params_.min_window_m,
        std::max(current_speed_, 0.0) * params_.window_time_s) +
        2.0 * params_.pass_complete_margin;
      if (hasPassedOpponent() || forward_dist > horizon) {
        if (!merge_back_active_) {
          startMergeBack();
        }
      }
    }

    if (merge_back_active_) {
      updateMergeBackPath();
      if (mergeBackProgress() >= 1.0) {
        resetAvoidance();
        return extractSegment();
      }
      return extractSegmentFromModified();
    }

    if (avoidance_active_) {
      return extractSegmentFromModified();
    }
    return extractSegment();
  }

  // If collision remains and the opponent is still near its committed state,
  // keep the locked path to avoid frame-to-frame jitter.
  if (merge_back_active_) {
    merge_back_active_ = false;
    merge_from_raceline_.clear();
  }

  if (avoidance_active_ && !hasOpponentMoved()) {
    return extractSegmentFromModified();
  }

  buildAvoidancePath();
  return extractSegmentFromModified();
}

// =============================================================
//  Segment extraction (with wraparound for closed tracks)
// =============================================================

std::vector<Waypoint> LateralPlanner::extractSegment() const
{
  const size_t n = waypoints_.size();
  if (n == 0) {
    return {};
  }

  const size_t robot_idx = closestWaypointInPath(modified_raceline_, robot_.x, robot_.y);
  const size_t ahead  = static_cast<size_t>(params_.lookahead_points);

  std::vector<Waypoint> result;
  result.reserve(ahead);

  for (size_t k = 0; k < ahead; ++k) {
    const size_t idx = (robot_idx + k) % n;
    result.push_back(waypoints_[idx]);
  }

  return result;
}

std::vector<Waypoint> LateralPlanner::extractSegmentFromModified() const
{
  const size_t n = modified_raceline_.size();
  if (n == 0) {
    return {};
  }

  const size_t robot_idx = closestWaypoint(robot_.x, robot_.y);
  const size_t ahead  = static_cast<size_t>(params_.lookahead_points);

  std::vector<Waypoint> result;
  result.reserve(ahead);

  for (size_t k = 0; k < ahead; ++k) {
    const size_t idx = (robot_idx + k) % n;
    result.push_back(modified_raceline_[idx]);
  }

  return result;
}

// =============================================================
//  Avoidance path building - collision-driven arc shift
// =============================================================

void LateralPlanner::buildAvoidancePath()
{
  if (waypoints_.empty()) {
    return;
  }

  merge_back_active_ = false;
  merge_from_raceline_.clear();
  modified_raceline_ = waypoints_;

  const size_t opp_idx = closestWaypoint(opponent_.x, opponent_.y);
  const size_t car_idx = closestWaypoint(robot_.x, robot_.y);
  const Waypoint & opp_wp = waypoints_[opp_idx];
  const Waypoint & car_wp = waypoints_[car_idx];

  const double pass_dir = decidePassingSide(opp_idx);

  // No feasible side available: keep original line and wait behind opponent.
  if (pass_dir == 0.0) {
    resetAvoidance();
    RCLCPP_WARN(logger_, "No feasible passing side available; holding behind opponent");
    return;
  }

  const double shift_mag = computeShiftMagnitude(opp_idx);
  if (shift_mag <= 1e-3) {
    resetAvoidance();
    return;
  }

  const double d_max = pass_dir * shift_mag;

  const double window_dist = std::max(
    params_.min_window_m,
    std::max(current_speed_, 0.0) * params_.window_time_s);
  const double lead_ratio = std::clamp(params_.window_lead_ratio, 0.1, 0.9);

  double lead_dist = std::max(0.75, window_dist * lead_ratio);
  const double trail_dist = std::max(0.75, window_dist * (1.0 - lead_ratio));

  // Keep a full lead window even when the opponent is close so the path can
  // already be shifted at the current car position (instead of shifting too late).
  const double car_to_opp = wrapForwardDistance(car_wp.s, opp_wp.s);
  if (car_to_opp < 0.2) {
    lead_dist = std::max(lead_dist, 0.2);
  }

  const double s_start = opp_wp.s - lead_dist;
  const double s_end   = opp_wp.s + trail_dist;

  applyLateralShift(s_start, opp_wp.s, s_end, d_max);

  avoidance_active_  = true;
  committed_side_    = pass_dir;
  committed_opp_idx_ = opp_idx;
  committed_opp_x_   = opponent_.x;
  committed_opp_y_   = opponent_.y;

  RCLCPP_INFO(logger_,
    "Avoidance locked: side=%.0f shift=%.2fm lead=%.2fm trail=%.2fm",
    pass_dir, shift_mag, lead_dist, trail_dist);
}

// =============================================================
//  Lateral shift - piecewise half-cosine (arc out and back)
// =============================================================

void LateralPlanner::applyLateralShift(
  double s_start, double opp_s, double s_end, double d_max)
{
  if (waypoints_.empty()) {
    return;
  }

  const double total_s = waypoints_.back().s;
  if (total_s <= 0.0) {
    return;
  }

  const double lead_dist = wrapForwardDistance(s_start, opp_s);
  const double trail_dist = wrapForwardDistance(opp_s, s_end);
  const double window_len = lead_dist + trail_dist;

  if (window_len <= 1e-6) {
    return;
  }

  const double wall_limit = std::abs(params_.max_lateral_shift_m);

  for (size_t i = 0; i < modified_raceline_.size(); ++i) {
    const Waypoint & orig = waypoints_[i];
    Waypoint shifted = orig;

    const double s_rel = wrapForwardDistance(s_start, orig.s);
    double offset = 0.0;

    if (s_rel <= window_len) {
      if (s_rel <= lead_dist && lead_dist > 1e-6) {
        offset = d_max * 0.5 * (1.0 - std::cos(M_PI * s_rel / lead_dist));
      } else if (trail_dist > 1e-6) {
        const double t = s_rel - lead_dist;
        offset = d_max * 0.5 * (1.0 + std::cos(M_PI * t / trail_dist));
      }

      if (std::abs(offset) > wall_limit) {
        offset = std::copysign(wall_limit, offset);
      }
    }

    const double normal = orig.psi + M_PI / 2.0;
    shifted.x = orig.x + offset * std::cos(normal);
    shifted.y = orig.y + offset * std::sin(normal);

    modified_raceline_[i] = shifted;
  }

  // Recompute heading and curvature so geometry is self-consistent.
  const size_t n = modified_raceline_.size();
  if (n >= 3) {
    for (size_t i = 0; i < n; ++i) {
      const Waypoint & prev = modified_raceline_[(i + n - 1) % n];
      Waypoint & curr = modified_raceline_[i];
      const Waypoint & next = modified_raceline_[(i + 1) % n];

      const double dx = next.x - prev.x;
      const double dy = next.y - prev.y;
      if (std::hypot(dx, dy) > 1e-6) {
        curr.psi = std::atan2(dy, dx);
      }

      const double psi_prev = std::atan2(curr.y - prev.y, curr.x - prev.x);
      const double psi_next = std::atan2(next.y - curr.y, next.x - curr.x);
      const double dpsi = std::atan2(
        std::sin(psi_next - psi_prev),
        std::cos(psi_next - psi_prev));
      const double ds_prev = std::hypot(curr.x - prev.x, curr.y - prev.y);
      const double ds_next = std::hypot(next.x - curr.x, next.y - curr.y);
      const double ds = std::max(0.5 * (ds_prev + ds_next), 1e-3);
      curr.kappa = dpsi / ds;
    }
  }

  // Update wall distances from lateral shift in the original raceline-normal frame.
  for (size_t i = 0; i < n; ++i) {
    const Waypoint & orig = waypoints_[i];
    Waypoint & curr = modified_raceline_[i];

    const double orig_normal = orig.psi + M_PI / 2.0;
    const double orig_nx = std::cos(orig_normal);
    const double orig_ny = std::sin(orig_normal);
    const double path_lateral =
      (curr.x - orig.x) * orig_nx + (curr.y - orig.y) * orig_ny;

    // Positive path_lateral means shifted left in the orig-normal frame.
    curr.d_left = std::max(0.0, orig.d_left - path_lateral);
    curr.d_right = std::max(0.0, orig.d_right + path_lateral);
  }

  updateRacelineSpeedsFromCurvature(modified_raceline_);
}

// =============================================================
//  Side decision and shift magnitude
// =============================================================

double LateralPlanner::decidePassingSide(size_t opp_idx) const
{
  if (opp_idx >= waypoints_.size()) {
    return (committed_side_ != 0.0) ? committed_side_ : 1.0;
  }

  const Waypoint & opp_wp = waypoints_[opp_idx];
  const double opp_lateral = lateralOffsetAtWaypoint(opp_idx, opponent_.x, opponent_.y);
  const double inflated_tolerance =
    params_.clearance_tolerance_m * std::max(1.0, params_.planning_tolerance_scale);

  const double clearance =
    params_.car_width_m / 2.0 +
    params_.car_width_m / 2.0 +
    inflated_tolerance;

  const double left_limit = std::max(
    opp_wp.d_left - params_.car_width_m / 2.0 - inflated_tolerance,
    0.05);
  const double right_limit = std::max(
    opp_wp.d_right - params_.car_width_m / 2.0 - inflated_tolerance,
    0.05);

  const double needed_left = std::abs(opp_lateral + clearance);
  const double needed_right = std::abs(opp_lateral - clearance);
  const bool left_feasible = left_limit >= needed_left;
  const bool right_feasible = right_limit >= needed_right;

  // Bias toward currently committed side, but never force it.
  if (committed_side_ > 0.0) {
    if (left_feasible) {
      return 1.0;
    }
    if (right_feasible) {
      return -1.0;
    }
    return 0.0;
  }
  if (committed_side_ < 0.0) {
    if (right_feasible) {
      return -1.0;
    }
    if (left_feasible) {
      return 1.0;
    }
    return 0.0;
  }

  if (left_feasible && !right_feasible) {
    return 1.0;
  }
  if (right_feasible && !left_feasible) {
    return -1.0;
  }
  if (left_feasible && right_feasible) {
    // If both are feasible, prefer the side opposite the opponent offset.
    return (opp_lateral >= 0.0) ? -1.0 : 1.0;
  }

  // Neither side is fully feasible: do not attempt an unsafe overtake.
  return 0.0;
}

double LateralPlanner::computeShiftMagnitude(size_t opp_idx) const
{
  if (opp_idx >= waypoints_.size()) {
    return 0.0;
  }

  const double pass_dir = decidePassingSide(opp_idx);
  if (pass_dir == 0.0) {
    return 0.0;
  }
  const double opp_lateral = lateralOffsetAtWaypoint(opp_idx, opponent_.x, opponent_.y);
  const Waypoint & opp_wp = waypoints_[opp_idx];
  const double inflated_tolerance =
    params_.clearance_tolerance_m * std::max(1.0, params_.planning_tolerance_scale);
  const double wall_margin = params_.clearance_tolerance_m;

  const double clearance =
    params_.car_width_m / 2.0 +
    params_.car_width_m / 2.0 +
    inflated_tolerance;

  // Desired shifted lane center in raceline-normal coordinates.
  const double target_lateral = opp_lateral + pass_dir * clearance;
  const double required_shift = std::abs(target_lateral);

  const double left_limit = std::max(
    opp_wp.d_left - params_.car_width_m / 2.0 - wall_margin,
    0.05);
  const double right_limit = std::max(
    opp_wp.d_right - params_.car_width_m / 2.0 - wall_margin,
    0.05);
  const double directional_limit = (pass_dir >= 0.0) ? left_limit : right_limit;

  return std::min({required_shift, std::abs(params_.max_lateral_shift_m), directional_limit});
}

// =============================================================
//  Avoidance state helpers
// =============================================================

double LateralPlanner::wrapForwardDistance(double s_from, double s_to) const
{
  if (waypoints_.empty()) {
    return 0.0;
  }

  const double total_s = waypoints_.back().s;
  if (total_s <= 0.0) {
    return 0.0;
  }

  double d = std::fmod(s_to - s_from, total_s);
  if (d < 0.0) {
    d += total_s;
  }
  return d;
}

double LateralPlanner::lateralOffsetAtWaypoint(size_t idx, double x, double y) const
{
  if (waypoints_.empty()) {
    return 0.0;
  }

  if (idx >= waypoints_.size()) {
    idx = waypoints_.size() - 1;
  }

  const Waypoint & wp = waypoints_[idx];
  const double normal = wp.psi + M_PI / 2.0;
  const double dx = x - wp.x;
  const double dy = y - wp.y;
  return dx * std::cos(normal) + dy * std::sin(normal);
}

bool LateralPlanner::isCollisionPredicted(
  size_t car_idx, size_t opp_idx,
  double * forward_dist,
  double * lateral_offset) const
{
  if (waypoints_.empty()) {
    return false;
  }

  if (car_idx >= waypoints_.size() || opp_idx >= waypoints_.size()) {
    return false;
  }

  const Waypoint & car_wp = waypoints_[car_idx];
  const Waypoint & opp_wp = waypoints_[opp_idx];
  const double total_s = waypoints_.back().s;
  if (total_s <= 0.0) {
    return false;
  }

  const double forward = wrapForwardDistance(car_wp.s, opp_wp.s);
  if (forward_dist != nullptr) {
    *forward_dist = forward;
  }

  // Ignore opponents behind us (would require > half lap forward distance).
  if (forward > total_s * 0.5) {
    return false;
  }

  const double horizon = std::max(
    params_.min_window_m,
    std::max(current_speed_, 0.0) * params_.window_time_s) +
    params_.pass_complete_margin + 0.5 * params_.opponent_length_m;
  if (forward > horizon) {
    return false;
  }

  const double lat = lateralOffsetAtWaypoint(opp_idx, opponent_.x, opponent_.y);
  if (lateral_offset != nullptr) {
    *lateral_offset = lat;
  }

  const double collision_corridor =
    params_.car_width_m / 2.0 +
    params_.car_width_m / 2.0 +
    params_.clearance_tolerance_m * std::max(1.0, params_.planning_tolerance_scale);

  return std::abs(lat) <= collision_corridor;
}

void LateralPlanner::resetAvoidance()
{
  avoidance_active_  = false;
  merge_back_active_ = false;
  committed_side_    = 0.0;
  committed_opp_idx_ = 0;
  committed_opp_x_   = 0.0;
  committed_opp_y_   = 0.0;
  merge_start_s_     = 0.0;
  merge_distance_m_  = 0.0;
  merge_from_raceline_.clear();
  modified_raceline_ = waypoints_;
}

void LateralPlanner::startMergeBack()
{
  if (!avoidance_active_ || waypoints_.empty() || merge_back_active_) {
    return;
  }

  if (modified_raceline_.size() != waypoints_.size()) {
    modified_raceline_ = waypoints_;
    return;
  }

  merge_back_active_ = true;
  merge_from_raceline_ = modified_raceline_;

  const size_t robot_idx = closestWaypoint(robot_.x, robot_.y);
  merge_start_s_ = waypoints_[robot_idx].s;

  merge_distance_m_ = std::max(
    params_.min_window_m,
    std::max(current_speed_, 0.0) * params_.window_time_s) +
    params_.pass_complete_margin;

  const double total_s = waypoints_.back().s;
  if (total_s > 0.0) {
    merge_distance_m_ = std::clamp(merge_distance_m_, 0.5, total_s);
  } else {
    merge_distance_m_ = std::max(merge_distance_m_, 0.5);
  }

  RCLCPP_INFO(logger_, "Merging back to baseline raceline over %.2f m", merge_distance_m_);
}

void LateralPlanner::updateMergeBackPath()
{
  if (!merge_back_active_) {
    return;
  }

  if (waypoints_.empty() || merge_from_raceline_.size() != waypoints_.size()) {
    resetAvoidance();
    return;
  }

  const double progress = mergeBackProgress();
  const double blend = 0.5 * (1.0 - std::cos(M_PI * progress));

  if (modified_raceline_.size() != waypoints_.size()) {
    modified_raceline_.resize(waypoints_.size());
  }

  for (size_t i = 0; i < waypoints_.size(); ++i) {
    const Waypoint & from = merge_from_raceline_[i];
    const Waypoint & base = waypoints_[i];

    Waypoint merged = from;
    merged.x = from.x + blend * (base.x - from.x);
    merged.y = from.y + blend * (base.y - from.y);

    const double dpsi = std::atan2(
      std::sin(base.psi - from.psi),
      std::cos(base.psi - from.psi));
    merged.psi = from.psi + blend * dpsi;

    merged.kappa = from.kappa + blend * (base.kappa - from.kappa);
    merged.vx = from.vx + blend * (base.vx - from.vx);
    merged.ax = from.ax + blend * (base.ax - from.ax);
    merged.d_left = from.d_left + blend * (base.d_left - from.d_left);
    merged.d_right = from.d_right + blend * (base.d_right - from.d_right);

    modified_raceline_[i] = merged;
  }

  updateRacelineSpeedsFromCurvature(modified_raceline_);
}

void LateralPlanner::updateRacelineSpeedsFromCurvature(std::vector<Waypoint> & path) const
{
  const size_t n = path.size();
  if (n == 0) {
    return;
  }

  const double curvature_factor = std::max(0.0, params_.curvature_speed_factor);
  const double floor_ratio = std::clamp(params_.curvature_speed_floor_ratio, 0.0, 1.0);
  const int preview_points = std::max(0, params_.speed_preview_points);
  const double min_regulated = std::max(0.0, params_.min_regulated_speed);
  const double max_lateral_accel = std::max(0.0, params_.max_lateral_accel);

  for (size_t i = 0; i < n; ++i) {
    const double nominal_vx = std::max(0.0, path[i].vx);
    if (!std::isfinite(nominal_vx) || nominal_vx <= 0.0) {
      path[i].vx = 0.0;
      continue;
    }

    double max_preview_curvature = std::abs(path[i].kappa);
    for (int j = 1; j <= preview_points; ++j) {
      const size_t idx = (i + static_cast<size_t>(j)) % n;
      max_preview_curvature = std::max(max_preview_curvature, std::abs(path[idx].kappa));
    }

    double speed_scale = 1.0;
    if (curvature_factor > 0.0) {
      speed_scale = 1.0 / (1.0 + curvature_factor * max_preview_curvature);
      speed_scale = std::clamp(speed_scale, floor_ratio, 1.0);
    }

    double regulated_vx = nominal_vx * speed_scale;

    // Lateral acceleration limit: v <= sqrt(a_lat_max / |kappa|).
    if (max_lateral_accel > 1e-6 && max_preview_curvature > 1e-6) {
      const double lateral_cap = std::sqrt(max_lateral_accel / max_preview_curvature);
      regulated_vx = std::min(regulated_vx, lateral_cap);
    }

    const double lower_bound = std::min(min_regulated, nominal_vx);
    path[i].vx = std::clamp(regulated_vx, lower_bound, nominal_vx);
  }
}

double LateralPlanner::mergeBackProgress() const
{
  if (!merge_back_active_ || waypoints_.empty() || merge_distance_m_ <= 1e-6) {
    return 1.0;
  }

  const size_t robot_idx = closestWaypoint(robot_.x, robot_.y);
  const double robot_s = waypoints_[robot_idx].s;
  const double traveled = wrapForwardDistance(merge_start_s_, robot_s);
  return std::clamp(traveled / merge_distance_m_, 0.0, 1.0);
}

bool LateralPlanner::hasPassedOpponent() const
{
  if (!avoidance_active_ || waypoints_.empty() || committed_opp_idx_ >= waypoints_.size()) {
    return false;
  }

  const double total_s = waypoints_.back().s;
  if (total_s <= 0.0) {
    return false;
  }

  const size_t robot_idx = closestWaypoint(robot_.x, robot_.y);
  const double opp_s = waypoints_[committed_opp_idx_].s;
  const double robot_s = waypoints_[robot_idx].s;
  const double forward_from_opp = wrapForwardDistance(opp_s, robot_s);

  // Passed if robot is ahead by margin, but not a full-loop wraparound artifact.
  return forward_from_opp > params_.pass_complete_margin &&
         forward_from_opp < total_s * 0.5;
}

bool LateralPlanner::hasOpponentMoved() const
{
  if (!opponent_.detected) {
    return true;
  }

  if (modified_raceline_.empty()) {
    return true;
  }

  // Hysteresis: only trigger replan when opponent intrudes close to current
  // committed line (or moves a lot globally), which avoids jitter.
  const size_t idx = closestWaypoint(opponent_.x, opponent_.y);
  if (idx >= modified_raceline_.size()) {
    return true;
  }

  const Waypoint & line_wp = modified_raceline_[idx];
  const double normal = line_wp.psi + M_PI / 2.0;
  const double lat_to_line = std::abs(
    (opponent_.x - line_wp.x) * std::cos(normal) +
    (opponent_.y - line_wp.y) * std::sin(normal));
  const double replan_band =
    params_.car_width_m / 2.0 +
    params_.car_width_m / 2.0 +
    params_.clearance_tolerance_m;
  if (lat_to_line <= replan_band) {
    return true;
  }

  return false;
}

// =============================================================
//  Helpers
// =============================================================

size_t LateralPlanner::closestWaypointInPath(
  const std::vector<Waypoint> & path,
  double x, double y) const
{
  if (path.empty()) {
    return 0;
  }

  size_t best = 0;
  double best_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < path.size(); ++i) {
    const double dx = path[i].x - x;
    const double dy = path[i].y - y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_dist) {
      best_dist = d2;
      best = i;
    }
  }

  return best;
}

size_t LateralPlanner::closestWaypoint(double x, double y) const
{
  return closestWaypointInPath(waypoints_, x, y);
}

}  // namespace f1tenth_lateral_planner
