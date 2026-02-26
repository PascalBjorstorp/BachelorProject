#pragma once

#include <string>
#include <vector>
#include <optional>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace f1tenth_lateral_planner
{

// ─────────────────────────────────────────────────────────────────────
//  Data types
// ─────────────────────────────────────────────────────────────────────

/// Single raceline waypoint matching the CSV columns.
struct Waypoint
{
  double s     = 0.0;   ///< Arc length [m]
  double x     = 0.0;   ///< X position [m]
  double y     = 0.0;   ///< Y position [m]
  double psi   = 0.0;   ///< Heading [rad]
  double kappa = 0.0;   ///< Curvature [1/m]
  double vx    = 0.0;   ///< Velocity [m/s]
  double ax    = 0.0;   ///< Acceleration [m/s²]
};

/// Detected opponent position and geometry.
struct OpponentState
{
  double x       = 0.0;
  double y       = 0.0;
  double width   = 0.3;   ///< Estimated width [m]
  bool   detected = false;
};

/// Robot pose in map frame.
struct RobotPose
{
  double x   = 0.0;
  double y   = 0.0;
  double yaw = 0.0;
};

// ─────────────────────────────────────────────────────────────────────
//  LateralPlanner class
// ─────────────────────────────────────────────────────────────────────

/**
 * @brief Core lateral planner logic — loads a raceline, detects opponents,
 *        and generates avoidance paths.
 *
 * When an opponent is detected, the planner computes an avoidance path
 * and **locks it in**. The locked path is published until the car has
 * passed the opponent or the opponent disappears. This prevents
 * frame-to-frame jitter from recomputing the avoidance every cycle.
 */
class LateralPlanner
{
public:
  // ── Construction ──────────────────────────────────────────────────

  struct Parameters
  {
    double safety_margin_m       = 0.3;
    double min_window_m          = 3.0;
    double window_time_s         = 0.8;
    double max_lateral_shift_m   = 0.8;
    double min_replan_dist_m     = 1.0;
    int    lookahead_points      = 80;   ///< Waypoints to publish ahead
    int    lookbehind_points     = 5;    ///< Waypoints to publish behind
    double opponent_move_thresh  = 0.5;  ///< Recompute if opponent moved more than this [m]
    double pass_complete_margin  = 2.0;  ///< Car must be this far past opponent to unlock [m]
    double window_lead_ratio     = 0.7;  ///< Fraction of window before the opponent [0..1]
    double car_width_m           = 0.31; ///< Own car width [m]
    double wall_safety_margin_m  = 0.15; ///< Min distance from path edge to wall [m]
    double track_half_width_m    = 1.5;  ///< Approx half-width of track from raceline [m]
  };

  explicit LateralPlanner(rclcpp::Logger logger, const Parameters & params);

  // ── Raceline I/O ──────────────────────────────────────────────────

  bool loadTrajectory(const std::string & csv_path);
  size_t waypointCount() const { return waypoints_.size(); }

  // ── State updates ─────────────────────────────────────────────────

  void updateRobotPose(double x, double y, double yaw);
  void updateSpeed(double speed);

  void processObstacleScan(
    const std::vector<float> & ranges,
    float angle_min, float angle_increment,
    float range_min, float range_max,
    double laser_x, double laser_y, double laser_yaw);

  void clearOpponent();

  // ── Planning ──────────────────────────────────────────────────────

  /// Compute the path to publish this cycle.
  std::vector<Waypoint> computePath();

  // ── Accessors ─────────────────────────────────────────────────────

  const OpponentState & opponent() const { return opponent_; }
  const RobotPose     & robotPose() const { return robot_; }
  const std::vector<Waypoint> & waypoints() const { return waypoints_; }

private:
  // ── Helpers ───────────────────────────────────────────────────────

  size_t closestWaypoint(double x, double y) const;
  std::vector<Waypoint> extractSegment() const;
  std::vector<Waypoint> extractSegmentFromModified() const;

  /// Build (or rebuild) the full modified raceline for avoidance.
  void buildAvoidancePath();

  /// Apply piecewise half-cosine lateral shift.
  /// Peaks at opp_s, ramps from s_start to opp_s and back to s_end.
  void applyLateralShift(
    double s_start, double opp_s, double s_end, double d_max);

  /// Decide which side to pass (+1 = left, −1 = right).
  double decidePassingSide(size_t opp_idx) const;

  /// Compute the required shift magnitude (clamped to wall clearance).
  double computeShiftMagnitude(size_t opp_idx) const;

  /// Max lateral offset allowed before hitting the wall.
  double maxAllowedShift() const;

  /// Check if the car has passed the opponent and the path can be unlocked.
  bool hasPassedOpponent() const;

  /// Check if the opponent has moved significantly from the locked position.
  bool hasOpponentMoved() const;

  // ── Data ──────────────────────────────────────────────────────────

  rclcpp::Logger logger_;
  Parameters     params_;

  std::vector<Waypoint> waypoints_;           ///< Original global raceline
  std::vector<Waypoint> modified_raceline_;   ///< Avoidance-modified raceline (full copy)
  OpponentState         opponent_;
  RobotPose             robot_;
  double                current_speed_ = 0.0;

  // ── Committed avoidance state ─────────────────────────────────────

  bool   avoidance_active_     = false;  ///< Is an avoidance path currently locked?
  double committed_side_       = 0.0;    ///< +1 left, −1 right
  size_t committed_opp_idx_    = 0;      ///< Raceline index of opponent when locked
  double committed_opp_x_      = 0.0;    ///< Opponent position when locked
  double committed_opp_y_      = 0.0;
};

}  // namespace f1tenth_lateral_planner
