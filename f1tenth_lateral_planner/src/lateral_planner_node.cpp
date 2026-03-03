#include <memory>
#include <string>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "f1tenth_lateral_planner/lateral_planner.hpp"

namespace f1tenth_lateral_planner
{

/// Extract yaw from a geometry_msgs quaternion.
static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
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
    declareParameters();
    initPlanner();
    setupSubscribers();
    setupPublishers();
    setupTimer();

    RCLCPP_INFO(get_logger(), "Lateral Planner Node (C++) initialized");
    RCLCPP_INFO(get_logger(), "  Waypoints: %zu", planner_->waypointCount());
  }

private:
  // ── Parameter declaration ─────────────────────────────────────────

  void declareParameters()
  {
    declare_parameter("trajectory_file", std::string(""));
    declare_parameter("safety_margin_m", 0.3);
    declare_parameter("min_window_m", 3.0);
    declare_parameter("window_time_s", 0.8);
    declare_parameter("max_lateral_shift_m", 0.8);
    declare_parameter("min_replan_dist_m", 1.0);
    declare_parameter("lookahead_points", 80);
    declare_parameter("lookbehind_points", 5);
    declare_parameter("opponent_move_thresh", 0.5);
    declare_parameter("pass_complete_margin", 2.0);
    declare_parameter("window_lead_ratio", 0.7);
    declare_parameter("car_width_m", 0.31);
    declare_parameter("wall_safety_margin_m", 0.15);
    declare_parameter("track_half_width_m", 1.5);
    declare_parameter("publish_rate_hz", 40.0);
    declare_parameter("enabled", true);
    declare_parameter("map_frame", std::string("map"));
    declare_parameter("base_frame", std::string("ego_racecar/base_link"));
    declare_parameter("laser_frame", std::string("ego_racecar/laser"));
    declare_parameter("obstacles_topic", std::string("/scan_obstacles"));
    declare_parameter("odom_topic", std::string("/odom"));
    declare_parameter("raceline_topic", std::string("/local_raceline"));
    declare_parameter("enable_topic", std::string("/lateral_planner_enable"));
  }

  // ── Planner initialization ────────────────────────────────────────

  void initPlanner()
  {
    LateralPlanner::Parameters params;
    params.safety_margin_m     = get_parameter("safety_margin_m").as_double();
    params.min_window_m        = get_parameter("min_window_m").as_double();
    params.window_time_s       = get_parameter("window_time_s").as_double();
    params.max_lateral_shift_m = get_parameter("max_lateral_shift_m").as_double();
    params.min_replan_dist_m   = get_parameter("min_replan_dist_m").as_double();
    params.lookahead_points      = get_parameter("lookahead_points").as_int();
    params.lookbehind_points     = get_parameter("lookbehind_points").as_int();
    params.opponent_move_thresh  = get_parameter("opponent_move_thresh").as_double();
    params.pass_complete_margin  = get_parameter("pass_complete_margin").as_double();
    params.window_lead_ratio     = get_parameter("window_lead_ratio").as_double();
    params.car_width_m           = get_parameter("car_width_m").as_double();
    params.wall_safety_margin_m  = get_parameter("wall_safety_margin_m").as_double();
    params.track_half_width_m    = get_parameter("track_half_width_m").as_double();

    planner_ = std::make_unique<LateralPlanner>(get_logger(), params);

    // Load the trajectory CSV
    std::string traj_file = get_parameter("trajectory_file").as_string();
    if (!traj_file.empty()) {
      planner_->loadTrajectory(traj_file);
    } else {
      RCLCPP_WARN(get_logger(), "No trajectory_file specified");
    }

    enabled_    = get_parameter("enabled").as_bool();
    RCLCPP_INFO(get_logger(), "  Avoidance enabled: %s", enabled_ ? "true" : "false");
    map_frame_  = get_parameter("map_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    laser_frame_ = get_parameter("laser_frame").as_string();
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

    std::string obstacles_topic = get_parameter("obstacles_topic").as_string();
    obstacle_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      obstacles_topic, sensor_qos,
      std::bind(&LateralPlannerNode::obstacleCallback, this, std::placeholders::_1));

    // Odometry
    std::string odom_topic = get_parameter("odom_topic").as_string();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      std::bind(&LateralPlannerNode::odomCallback, this, std::placeholders::_1));

    // Enable/disable
    std::string enable_topic = get_parameter("enable_topic").as_string();
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      enable_topic, 10,
      std::bind(&LateralPlannerNode::enableCallback, this, std::placeholders::_1));
  }

  // ── Publishers ────────────────────────────────────────────────────

  void setupPublishers()
  {
    std::string raceline_topic = get_parameter("raceline_topic").as_string();
    raceline_pub_ = create_publisher<nav_msgs::msg::Path>(raceline_topic, 10);
    raceline_viz_pub_ = create_publisher<nav_msgs::msg::Path>(raceline_topic + "_viz", 10);
    marker_pub_   = create_publisher<visualization_msgs::msg::MarkerArray>("/opponent_marker", 10);
  }

  // ── Timer ─────────────────────────────────────────────────────────

  void setupTimer()
  {
    double rate = get_parameter("publish_rate_hz").as_double();
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
    planner_->updateSpeed(std::sqrt(vx * vx + vy * vy));
  }

  void enableCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    enabled_ = msg->data;
    RCLCPP_INFO(get_logger(), "Lateral planner %s",
                enabled_ ? "enabled" : "disabled");
  }

  void obstacleCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    // Skip obstacle processing when planner is disabled (passthrough mode)
    if (!enabled_) {
      return;
    }

    // Look up laser pose in map frame
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        map_frame_, laser_frame_,
        tf2::TimePointZero,
        tf2::durationFromSec(0.02));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(get_logger(), "TF lookup failed: %s", ex.what());
      return;
    }

    double lx  = tf.transform.translation.x;
    double ly  = tf.transform.translation.y;
    double lyaw = yawFromQuaternion(tf.transform.rotation);

    planner_->processObstacleScan(
      scan->ranges,
      scan->angle_min, scan->angle_increment,
      scan->range_min, scan->range_max,
      lx, ly, lyaw);
  }

  // ── Main loop ─────────────────────────────────────────────────────

  void planLoop()
  {
    if (planner_->waypointCount() == 0) {
      return;
    }

    // Update robot pose from TF
    if (!updateRobotPoseFromTF()) {
      return;
    }

    // Compute path (handles both normal and avoidance cases)
    // When disabled, obstacle processing is skipped so computePath()
    // always sees no opponent and returns the original raceline.
    auto path_waypoints = planner_->computePath();

    // Publish controller path (velocity in z) and visualization path (z=0)
    publishPath(path_waypoints);
    publishPathViz(path_waypoints);

    if (enabled_) {
      publishOpponentMarker(planner_->opponent().detected);
    }
  }

  // ── TF pose update ────────────────────────────────────────────────

  bool updateRobotPoseFromTF()
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        map_frame_, base_frame_,
        tf2::TimePointZero,
        tf2::durationFromSec(0.02));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(get_logger(), "Robot TF lookup failed: %s", ex.what());
      return false;
    }

    double x   = tf.transform.translation.x;
    double y   = tf.transform.translation.y;
    double yaw = yawFromQuaternion(tf.transform.rotation);
    planner_->updateRobotPose(x, y, yaw);
    return true;
  }

  // ── Publishing ────────────────────────────────────────────────────

  void publishPath(const std::vector<Waypoint> & waypoints)
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
      pose.pose.position.z = wp.vx;  // velocity encoded in z (controller convention)
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

  void publishOpponentMarker(bool detected)
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker marker;

    marker.header.stamp    = now();
    marker.header.frame_id = map_frame_;
    marker.ns   = "opponent";
    marker.id   = 0;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;

    if (detected) {
      const auto & opp = planner_->opponent();
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = opp.x;
      marker.pose.position.y = opp.y;
      marker.pose.position.z = 0.1;
      marker.scale.x = opp.width;
      marker.scale.y = opp.width;
      marker.scale.z = 0.2;
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.8f;
    } else {
      marker.action = visualization_msgs::msg::Marker::DELETE;
    }

    markers.markers.push_back(marker);
    marker_pub_->publish(markers);
  }

  // ── Members ───────────────────────────────────────────────────────

  std::unique_ptr<LateralPlanner> planner_;
  bool enabled_ = true;

  // Frame IDs
  std::string map_frame_;
  std::string base_frame_;
  std::string laser_frame_;

  // TF
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr         enable_sub_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                raceline_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                raceline_viz_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
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
