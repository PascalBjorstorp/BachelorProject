#pragma once

#include <string>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
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
  double d_left  = 0.0; ///< Distance to left wall [m]
  double d_right = 0.0; ///< Distance to right wall [m]
};

/// Detected opponent position and geometry.
struct OpponentState
{
  double x       = 0.0;   ///< Body center x [m]
  double y       = 0.0;   ///< Body center y [m]
  double back_x  = 0.0;   ///< Detected rear-point x [m]
  double back_y  = 0.0;   ///< Detected rear-point y [m]
  double yaw     = 0.0;   ///< Body heading [rad]
  double width   = 0.3;   ///< Fixed body width [m]
  double length  = 0.5;   ///< Fixed body length [m]
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
    double min_window_m          = 3.0;
    double window_time_s         = 0.8;
    double max_lateral_shift_m   = 0.8;
    int    lookahead_points      = 80;   ///< Waypoints to publish ahead
    double pass_complete_margin  = 2.0;  ///< Car must be this far past opponent to unlock [m]
    double window_lead_ratio     = 0.7;  ///< Fraction of window before the opponent [0..1]
    double opponent_length_m     = 0.58; ///< Known opponent length [m]
    double clearance_tolerance_m = 0.15; ///< Extra clearance to walls/opponent [m]
    double planning_tolerance_scale = 2.0;  ///< Multiplier for line-generation tolerance
    double car_width_m           = 0.31; ///< Own car width [m]
    double curvature_speed_factor = 0.10;     ///< Slowdown gain on preview curvature
    double curvature_speed_floor_ratio = 0.43; ///< Min speed ratio after slowdown [0..1]
    int speed_preview_points = 8;             ///< Curvature preview horizon in waypoints
    double max_lateral_accel = 7.27;          ///< Physics cap for v^2*kappa [m/s^2]
    double min_regulated_speed = 0.30;        ///< Lower bound when nominal speed is nonzero [m/s]
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

  size_t closestWaypointInPath(const std::vector<Waypoint> & path, double x, double y) const;
  size_t closestWaypoint(double x, double y) const;
  std::vector<Waypoint> extractSegment() const;
  std::vector<Waypoint> extractSegmentFromModified() const;

  /// Build (or rebuild) the full modified raceline for avoidance.
  void buildAvoidancePath();

  /// Apply piecewise half-cosine lateral shift.
  /// Peaks at opp_s, ramps from s_start to opp_s and back to s_end.
  void applyLateralShift(
    double s_start, double opp_s, double s_end, double d_max);

  /// Decide which side to pass (+1 = left, −1 = right), never 0.
  double decidePassingSide(size_t opp_idx) const;

  /// Compute the required shift magnitude (clamped to wall clearance).
  double computeShiftMagnitude(size_t opp_idx) const;

  /// Forward arc distance from s_from to s_to on a closed track [m].
  double wrapForwardDistance(double s_from, double s_to) const;

  /// Signed opponent lateral offset relative to waypoint tangent normal [m].
  double lateralOffsetAtWaypoint(size_t idx, double x, double y) const;

  /// Collision predicate between current raceline and detected opponent.
  bool isCollisionPredicted(
    size_t car_idx, size_t opp_idx,
    double * forward_dist = nullptr,
    double * lateral_offset = nullptr) const;

  /// Reset all avoidance state and return to original raceline.
  void resetAvoidance();

  /// Begin smooth blend from current modified line back to baseline raceline.
  void startMergeBack();

  /// Build a fixed merge-back raceline from current line to baseline.
  void buildMergeBackPath();

  /// Apply curvature-aware speed limits to a path in-place.
  void updateRacelineSpeedsFromCurvature(std::vector<Waypoint> & path) const;

  /// Progress of merge-back in [0, 1]. Returns 1 when not active.
  double mergeBackProgress() const;

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
  bool   merge_back_active_    = false;  ///< Is a merge back to baseline in progress?
  double committed_side_       = 0.0;    ///< +1 left, −1 right
  size_t committed_opp_idx_    = 0;      ///< Raceline index of opponent when locked
  double committed_opp_x_      = 0.0;    ///< Opponent position when locked
  double committed_opp_y_      = 0.0;

  double               merge_start_s_   = 0.0;
  double               merge_distance_m_ = 0.0;
  std::vector<Waypoint> merge_from_raceline_;
};

}  // namespace f1tenth_lateral_planner
