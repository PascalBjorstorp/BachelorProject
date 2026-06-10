#include <memory>
#include <string>
#include <cmath>
#include <mutex>
#include <exception>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "f1tenth_lateral_planner/lateral_planner.hpp"
#include "frames.h"
#include "topics.h"
#include "lateral_planner_config.hpp"

namespace f1tenth_lateral_planner
{

/// Extract yaw from a geometry_msgs quaternion.
static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

/// Resolve trajectory path from compile-time config.
static std::string resolveTrajectoryFile(rclcpp::Logger logger)
{
  const std::string configured_path = TRAJECTORY_FILE;
  if (!configured_path.empty()) {
    return configured_path;
  }

  try {
    const std::string planning_share_dir =
      ament_index_cpp::get_package_share_directory(TRAJECTORY_PACKAGE);
    return planning_share_dir + "/" + TRAJECTORY_REL_PATH;
  } catch (const std::exception & ex) {
    RCLCPP_WARN(
      logger,
      "Failed to resolve default trajectory from package '%s': %s",
      TRAJECTORY_PACKAGE,
      ex.what());
    return "";
  }
}

// ═════════════════════════════════════════════════════════════════════
//  ROS2 Node
// ═════════════════════════════════════════════════════════════════════

class LateralPlannerNode : public rclcpp::Node
{
public:
  LateralPlannerNode()
  : Node("lateral_planner_node")
  {
    avoidance_enabled_ = declare_parameter<bool>(
      "avoidance_enabled", LATERAL_PLANNER_ENABLE_AVOIDANCE);
    trajectory_file_ = declare_parameter<std::string>("trajectory_file", "");
    startup_speed_initial_scale_ = declare_parameter<double>(
      "startup_speed_initial_scale", 0.5);
    startup_speed_ramp_duration_sec_ = declare_parameter<double>(
      "startup_speed_ramp_duration_sec", 15.0);

    initPlanner();
    setupSubscribers();
    setupPublishers();
    setupTimer();

    RCLCPP_INFO(get_logger(), "Lateral Planner Node (C++) initialized");
    RCLCPP_INFO(get_logger(), "  Waypoints: %zu", planner_->waypointCount());
  }

private:
  // ── Planner initialization ────────────────────────────────────────

  void initPlanner()
  {
    LateralPlanner::Parameters params;
    params.min_window_m          = MIN_WINDOW_M;
    params.window_time_s         = WINDOW_TIME_S;
    params.max_lateral_shift_m   = MAX_LATERAL_SHIFT_M;
    params.lookahead_points      = LOOKAHEAD_POINTS;
    params.path_start_offset_points = PATH_START_OFFSET_POINTS;
    params.pass_complete_margin  = PASS_COMPLETE_MARGIN_M;
    params.window_lead_ratio     = WINDOW_LEAD_RATIO;
    params.max_avoidance_kappa   = MAX_AVOIDANCE_KAPPA;
    params.opponent_length_m     = OPPONENT_LENGTH_M;
    params.clearance_tolerance_m = CLEARANCE_TOLERANCE_M;
    params.planning_tolerance_scale = PLANNING_TOLERANCE_SCALE;
    params.car_width_m           = CAR_WIDTH_M;
    params.curvature_speed_factor = CURVATURE_SPEED_FACTOR;
    params.curvature_speed_floor_ratio = CURVATURE_SPEED_FLOOR_RATIO;
    params.speed_preview_points = CURVATURE_SPEED_PREVIEW_POINTS;
    params.max_lateral_accel = MAX_LATERAL_ACCEL_MPS2;
    params.min_regulated_speed = MIN_REGULATED_SPEED_MPS;
    params.obstacle_speed_cap_mps = OBSTACLE_DETECTED_MAX_SPEED_MPS;

    planner_ = std::make_unique<LateralPlanner>(get_logger(), params);

    // Load the trajectory CSV
    const std::string traj_file = trajectory_file_.empty()
      ? resolveTrajectoryFile(get_logger())
      : trajectory_file_;
    if (!traj_file.empty()) {
      planner_->loadTrajectory(traj_file);
      RCLCPP_INFO(get_logger(), "  Trajectory: %s", traj_file.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "No trajectory path configured in lateral_planner_config.hpp");
    }

    RCLCPP_INFO(get_logger(), "  Avoidance enabled: %s", avoidance_enabled_ ? "true" : "false");
    map_frame_ = FRAME_MAP;
    laser_frame_ = FRAME_LASER;
  }

  // ── Subscribers ───────────────────────────────────────────────────

  void setupSubscribers()
  {
    // TF
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Obstacle scan — best-effort QoS for sensor data
    auto sensor_qos = rclcpp::QoS(5)
      .reliability(rclcpp::ReliabilityPolicy::BestEffort)
      .durability(rclcpp::DurabilityPolicy::Volatile);

    if (avoidance_enabled_) {
      obstacle_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        TOPIC_SCAN_OBSTACLES, sensor_qos,
        std::bind(&LateralPlannerNode::obstacleCallback, this, std::placeholders::_1));
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Obstacle avoidance disabled; publishing baseline raceline on %s",
        TOPIC_LOCAL_RACELINE);
    }

    // Odometry
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      TOPIC_EGO_ODOM, 10,
      std::bind(&LateralPlannerNode::odomCallback, this, std::placeholders::_1));
  }

  // ── Publishers ────────────────────────────────────────────────────

  void setupPublishers()
  {
    const std::string raceline_topic = TOPIC_LOCAL_RACELINE;
    raceline_pub_ = create_publisher<nav_msgs::msg::Path>(raceline_topic, 10);
    raceline_viz_pub_ = create_publisher<nav_msgs::msg::Path>(raceline_topic + "_viz", 10);
    wall_distance_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      raceline_topic + "_wall_distance_markers", 10);
    marker_pub_   = create_publisher<visualization_msgs::msg::MarkerArray>("/opponent_marker", 10);
  }

  // ── Timer ─────────────────────────────────────────────────────────

  void setupTimer()
  {
    double rate = PUBLISH_RATE_HZ;
    if (rate <= 0.0) {
      RCLCPP_WARN(get_logger(), "Invalid PUBLISH_RATE_HZ (%.3f), using 200.0", rate);
      rate = 200.0;
    }
    auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&LateralPlannerNode::planLoop, this));
  }

  // ── Callbacks ─────────────────────────────────────────────────────

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    std::lock_guard<std::mutex> lock(planner_mutex_);
    planner_->updateSpeed(std::sqrt(vx * vx + vy * vy));
  }

  void obstacleCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    if (!avoidance_enabled_) {
      return;
    }

    // Look up laser pose in map frame
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        map_frame_, laser_frame_,
        rclcpp::Time(scan->header.stamp),
        tf2::durationFromSec(0.02));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(get_logger(), "TF lookup failed: %s", ex.what());
      return;
    }

    double lx  = tf.transform.translation.x;
    double ly  = tf.transform.translation.y;
    double lyaw = yawFromQuaternion(tf.transform.rotation);

    std::lock_guard<std::mutex> lock(planner_mutex_);
    planner_->processObstacleScan(
      scan->ranges,
      scan->angle_min, scan->angle_increment,
      scan->range_min, scan->range_max,
      lx, ly, lyaw);
  }

  // ── Main loop ─────────────────────────────────────────────────────

  void planLoop()
  {
    {
      std::lock_guard<std::mutex> lock(planner_mutex_);
      if (planner_->waypointCount() == 0) {
        return;
      }
    }

    // Update planning reference pose from TF
    if (!updateRobotPoseFromTF()) {
      return;
    }

    std::vector<Waypoint> path_waypoints;
    OpponentState opponent_snapshot;
    {
      // Compute path (handles both normal and avoidance cases).
      std::lock_guard<std::mutex> lock(planner_mutex_);
      if (!avoidance_enabled_) {
        planner_->clearOpponent();
      }
      path_waypoints = planner_->computePath();
      opponent_snapshot = planner_->opponent();
    }

    // Publish controller path (velocity in z) and visualization path (z=0)
    publishPath(path_waypoints);
    publishPathViz(path_waypoints);
    publishWallDistanceMarkers(path_waypoints, opponent_snapshot);
    publishOpponentMarker(opponent_snapshot);
  }

  // ── TF pose update ────────────────────────────────────────────────

  bool updateRobotPoseFromTF()
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        map_frame_, laser_frame_,
        tf2::TimePointZero,
        tf2::durationFromSec(0.02));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(get_logger(), "Laser TF lookup failed: %s", ex.what());
      return false;
    }

    double x   = tf.transform.translation.x;
    double y   = tf.transform.translation.y;
    double yaw = yawFromQuaternion(tf.transform.rotation);

    std::lock_guard<std::mutex> lock(planner_mutex_);
    planner_->updateRobotPose(x, y, yaw);
    return true;
  }

  // ── Startup speed ramp helper ─────────────────────────────────────

  double getStartupSpeedScale()
  {
    const double initial_scale = std::clamp(startup_speed_initial_scale_, 0.0, 1.0);
    const double ramp_duration = std::max(0.0, startup_speed_ramp_duration_sec_);
    if (ramp_duration <= 1e-6) {
      return 1.0;
    }

    if (!speed_ramp_started_) {
      speed_ramp_start_time_ = now();
      speed_ramp_started_ = true;
    }

    const double elapsed = (now() - speed_ramp_start_time_).seconds();
    const double alpha = std::clamp(elapsed / ramp_duration, 0.0, 1.0);
    return initial_scale + (1.0 - initial_scale) * alpha;
  }

  // ── Publishing ────────────────────────────────────────────────────

  void publishPath(const std::vector<Waypoint> & waypoints)
  {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp    = now();
    path_msg.header.frame_id = map_frame_;
    path_msg.poses.reserve(waypoints.size());

    const double speed_scale = getStartupSpeedScale();

    for (const auto & wp : waypoints) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = wp.x;
      pose.pose.position.y = wp.y;
      pose.pose.position.z = wp.vx * speed_scale;  // velocity encoded in z (controller convention)
      pose.pose.orientation.x = wp.d_left;          // left wall distance encoded for MPC
      pose.pose.orientation.y = wp.d_right;         // right wall distance encoded for MPC
      pose.pose.orientation.z = std::sin(wp.psi / 2.0);
      pose.pose.orientation.w = std::cos(wp.psi / 2.0);
      path_msg.poses.push_back(pose);
    }

    raceline_pub_->publish(path_msg);
  }

  /// Publish a flat (z=0) path for RViz visualization.
  void publishPathViz(const std::vector<Waypoint> & waypoints)
  {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp    = now();
    path_msg.header.frame_id = map_frame_;
    path_msg.poses.reserve(waypoints.size());

    for (const auto & wp : waypoints) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = wp.x;
      pose.pose.position.y = wp.y;
      pose.pose.position.z = 0.0;  // flat for RViz
      pose.pose.orientation.z = std::sin(wp.psi / 2.0);
      pose.pose.orientation.w = std::cos(wp.psi / 2.0);
      path_msg.poses.push_back(pose);
    }

    raceline_viz_pub_->publish(path_msg);
  }

  /// Publish per-waypoint line segments to left/right wall distances for RViz.
  void publishWallDistanceMarkers(
    const std::vector<Waypoint> & waypoints,
    const OpponentState & opponent)
  {
    visualization_msgs::msg::MarkerArray markers;
    const auto & reference_waypoints = planner_->waypoints();

    auto make_base_marker = [&](int id, float r, float g, float b) {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = now();
      marker.header.frame_id = map_frame_;
      marker.ns = "wall_distance";
      marker.id = id;
      marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.015;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.color.a = 0.85f;
      return marker;
    };

    auto left_marker = make_base_marker(0, 0.1f, 0.9f, 0.2f);
    auto right_marker = make_base_marker(1, 0.2f, 0.5f, 1.0f);
    left_marker.points.reserve(waypoints.size() * 2);
    right_marker.points.reserve(waypoints.size() * 2);

    auto clipToOpponentBody = [&](const geometry_msgs::msg::Point & start,
                                  geometry_msgs::msg::Point * end) {
      if (!opponent.detected || end == nullptr) {
        return;
      }

      const double half_len = std::max(0.5 * opponent.length, 0.05);
      const double half_wid = std::max(0.5 * opponent.width, 0.05);
      const double c = std::cos(opponent.yaw);
      const double s = std::sin(opponent.yaw);

      auto to_local = [&](double wx, double wy, double & lx, double & ly) {
        const double dx = wx - opponent.x;
        const double dy = wy - opponent.y;
        lx = dx * c + dy * s;
        ly = -dx * s + dy * c;
      };

      auto to_world = [&](double lx, double ly, double & wx, double & wy) {
        wx = opponent.x + lx * c - ly * s;
        wy = opponent.y + lx * s + ly * c;
      };

      double x0 = 0.0;
      double y0 = 0.0;
      double x1 = 0.0;
      double y1 = 0.0;
      to_local(start.x, start.y, x0, y0);
      to_local(end->x, end->y, x1, y1);

      const double dx = x1 - x0;
      const double dy = y1 - y0;
      double t_min = 0.0;
      double t_max = 1.0;

      auto clip_axis = [&](double p0, double d, double min_v, double max_v) -> bool {
        if (std::abs(d) < 1e-9) {
          return p0 >= min_v && p0 <= max_v;
        }
        double t1 = (min_v - p0) / d;
        double t2 = (max_v - p0) / d;
        if (t1 > t2) {
          std::swap(t1, t2);
        }
        t_min = std::max(t_min, t1);
        t_max = std::min(t_max, t2);
        return t_min <= t_max;
      };

      if (!clip_axis(x0, dx, -half_len, half_len) ||
          !clip_axis(y0, dy, -half_wid, half_wid))
      {
        return;
      }

      if (t_max < 0.0 || t_min > 1.0) {
        return;
      }

      const double t_hit = std::clamp(t_min, 0.0, 1.0);
      if (t_hit >= 1.0) {
        return;
      }

      const double seg_len = std::hypot(dx, dy);
      const double backoff_t = (seg_len > 1e-6) ? (0.01 / seg_len) : 0.0;
      const double t_clip = std::max(0.0, t_hit - backoff_t);

      double clip_x = 0.0;
      double clip_y = 0.0;
      to_world(x0 + t_clip * dx, y0 + t_clip * dy, clip_x, clip_y);
      end->x = clip_x;
      end->y = clip_y;
    };

    auto nearestReferenceByS = [&](double s) -> const Waypoint * {
      if (reference_waypoints.empty()) {
        return nullptr;
      }

      auto it = std::lower_bound(
        reference_waypoints.begin(), reference_waypoints.end(), s,
        [](const Waypoint & candidate, double value) {
          return candidate.s < value;
        });

      if (it == reference_waypoints.begin()) {
        return &(*it);
      }
      if (it == reference_waypoints.end()) {
        return &reference_waypoints.back();
      }

      const Waypoint & hi = *it;
      const Waypoint & lo = *(it - 1);
      return (std::abs(hi.s - s) < std::abs(s - lo.s)) ? &hi : &lo;
    };

    for (const auto & wp : waypoints) {
      const Waypoint * ref_wp = nearestReferenceByS(wp.s);
      const double normal = (ref_wp != nullptr ? ref_wp->psi : wp.psi) + M_PI / 2.0;
      const double nx = std::cos(normal);
      const double ny = std::sin(normal);
      const double d_left = std::clamp(wp.d_left, 0.0, 10.0);
      const double d_right = std::clamp(wp.d_right, 0.0, 10.0);

      geometry_msgs::msg::Point center;
      center.x = wp.x;
      center.y = wp.y;
      center.z = 0.02;

      geometry_msgs::msg::Point left;
      left.x = wp.x + d_left * nx;
      left.y = wp.y + d_left * ny;
      left.z = 0.02;

      geometry_msgs::msg::Point right;
      right.x = wp.x - d_right * nx;
      right.y = wp.y - d_right * ny;
      right.z = 0.02;

      clipToOpponentBody(center, &left);
      clipToOpponentBody(center, &right);

      left_marker.points.push_back(center);
      left_marker.points.push_back(left);
      right_marker.points.push_back(center);
      right_marker.points.push_back(right);
    }

    markers.markers.push_back(left_marker);
    markers.markers.push_back(right_marker);
    wall_distance_marker_pub_->publish(markers);
  }

  void publishOpponentMarker(const OpponentState & opponent)
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker body;
    visualization_msgs::msg::Marker rear_point;

    body.header.stamp = now();
    body.header.frame_id = map_frame_;
    body.ns = "opponent";
    body.id = 0;
    body.type = visualization_msgs::msg::Marker::CUBE;

    rear_point.header = body.header;
    rear_point.ns = "opponent";
    rear_point.id = 1;
    rear_point.type = visualization_msgs::msg::Marker::SPHERE;

    if (opponent.detected) {
      body.action = visualization_msgs::msg::Marker::ADD;
      body.pose.position.x = opponent.x;
      body.pose.position.y = opponent.y;
      body.pose.position.z = 0.1;
      body.pose.orientation.z = std::sin(opponent.yaw * 0.5);
      body.pose.orientation.w = std::cos(opponent.yaw * 0.5);
      body.scale.x = opponent.length;
      body.scale.y = opponent.width;
      body.scale.z = 0.2;
      body.color.r = 1.0f;
      body.color.g = 0.0f;
      body.color.b = 0.0f;
      body.color.a = 0.8f;

      rear_point.action = visualization_msgs::msg::Marker::ADD;
      rear_point.pose.position.x = opponent.back_x;
      rear_point.pose.position.y = opponent.back_y;
      rear_point.pose.position.z = 0.12;
      rear_point.scale.x = 0.08;
      rear_point.scale.y = 0.08;
      rear_point.scale.z = 0.08;
      rear_point.color.r = 1.0f;
      rear_point.color.g = 1.0f;
      rear_point.color.b = 0.0f;
      rear_point.color.a = 0.9f;
    } else {
      body.action = visualization_msgs::msg::Marker::DELETE;
      rear_point.action = visualization_msgs::msg::Marker::DELETE;
    }

    markers.markers.push_back(body);
    markers.markers.push_back(rear_point);
    marker_pub_->publish(markers);
  }

  // ── Members ───────────────────────────────────────────────────────

  std::unique_ptr<LateralPlanner> planner_;
  std::mutex planner_mutex_;
  bool avoidance_enabled_{true};
  std::string trajectory_file_;
  double startup_speed_initial_scale_{0.5};
  double startup_speed_ramp_duration_sec_{15.0};

  // Frame IDs
  std::string map_frame_;
  std::string laser_frame_;

  // TF
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_sub_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                raceline_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                raceline_viz_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_distance_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Startup speed ramp
  rclcpp::Time speed_ramp_start_time_;
  bool speed_ramp_started_{false};
};

}  // namespace f1tenth_lateral_planner


// ═════════════════════════════════════════════════════════════════════
//  Main
// ═════════════════════════════════════════════════════════════════════

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<f1tenth_lateral_planner::LateralPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
