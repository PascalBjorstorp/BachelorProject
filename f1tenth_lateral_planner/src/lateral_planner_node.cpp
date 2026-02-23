#include "f1tenth_lateral_planner/lateral_planner.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace f1tenth_lateral_planner {

/**
 * @brief ROS2 node that wraps the LateralPlanner algorithm.
 *
 * Subscriptions:
 *   /scan_obstacles  (sensor_msgs/LaserScan) — obstacle-only LiDAR beams
 *   /odom            (nav_msgs/Odometry)     — robot speed
 *   TF map→base_link                         — robot pose
 *
 * Publications:
 *   /local_raceline    (nav_msgs/Path)              — path for Pure Pursuit / Stanley
 *   /opponent_marker   (visualization_msgs/MarkerArray) — RViz cylinder
 *
 * Parameters (see config/lateral_planner.yaml for defaults):
 *   trajectory_file, safety_margin_m, min_window_m, window_time_s,
 *   max_lateral_shift_m, min_replan_dist_m, blend_rate,
 *   publish_rate_hz, map_frame, laser_frame, base_link_frame,
 *   obstacles_topic, odom_topic, raceline_topic, enable_topic
 */
class LateralPlannerNode : public rclcpp::Node {
public:
    explicit LateralPlannerNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("lateral_planner_node", options)
    {
        declareParameters();
        loadParameters();

        planner_ = std::make_unique<LateralPlanner>(config_);
        loadRaceline();

        // TF
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscribers
        using std::placeholders::_1;
        auto sensor_qos = rclcpp::QoS(5).best_effort();

        const std::string obstacles_topic =
            get_parameter("obstacles_topic").as_string();
        obstacle_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            obstacles_topic, sensor_qos,
            std::bind(&LateralPlannerNode::obstacleCallback, this, _1));

        const std::string odom_topic = get_parameter("odom_topic").as_string();
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            std::bind(&LateralPlannerNode::odomCallback, this, _1));

        const std::string enable_topic = get_parameter("enable_topic").as_string();
        enable_sub_ = create_subscription<std_msgs::msg::Bool>(
            enable_topic, 10,
            std::bind(&LateralPlannerNode::enableCallback, this, _1));

        // Publishers
        const std::string raceline_topic =
            get_parameter("raceline_topic").as_string();
        raceline_pub_ =
            create_publisher<nav_msgs::msg::Path>(raceline_topic, 10);
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/opponent_marker", 10);

        // Planning timer
        const double rate = get_parameter("publish_rate_hz").as_double();
        plan_timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / rate),
            std::bind(&LateralPlannerNode::planLoop, this));

        RCLCPP_INFO(get_logger(), "Lateral Planner Node initialised");
        RCLCPP_INFO(get_logger(), "  Raceline: %zu waypoints (%.1f m)",
                    planner_->size(), planner_->totalLength());
        RCLCPP_INFO(get_logger(), "  Safety margin: %.2f m  Max shift: %.2f m",
                    config_.safety_margin_m, config_.max_lateral_shift_m);
        RCLCPP_INFO(get_logger(), "  Rate: %.0f Hz  Frames: %s → %s",
                    rate, map_frame_.c_str(), base_link_frame_.c_str());
    }

private:
    // ── Algorithm ─────────────────────────────────────────────────────────
    std::unique_ptr<LateralPlanner> planner_;
    LateralPlannerConfig config_;

    // ── Node state ────────────────────────────────────────────────────────
    std::mutex state_mutex_;
    bool enabled_{true};

    // Opponent state (written by obstacle callback, read by plan loop)
    bool   opp_detected_{false};
    double opp_x_{0.0};
    double opp_y_{0.0};
    double opp_width_{0.35};

    // Robot state (written by odom callback and TF lookup)
    double robot_x_{0.0};
    double robot_y_{0.0};
    double current_speed_{0.0};

    // Cached waypoint indices for O(1) local searches
    size_t robot_hint_{0};
    size_t opp_hint_{0};

    // ── TF ────────────────────────────────────────────────────────────────
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ── ROS communication ─────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr  obstacle_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr          enable_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr             raceline_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr plan_timer_;

    // ── Parameters ────────────────────────────────────────────────────────
    std::string map_frame_{"map"};
    std::string laser_frame_{"ego_racecar/laser"};
    std::string base_link_frame_{"ego_racecar/base_link"};

    // ─────────────────────────────────────────────────────────────────────
    //  Parameter handling
    // ─────────────────────────────────────────────────────────────────────

    void declareParameters()
    {
        declare_parameter("trajectory_file", "");
        declare_parameter("safety_margin_m", config_.safety_margin_m);
        declare_parameter("min_window_m", config_.min_window_m);
        declare_parameter("window_time_s", config_.window_time_s);
        declare_parameter("max_lateral_shift_m", config_.max_lateral_shift_m);
        declare_parameter("min_replan_dist_m", config_.min_replan_dist_m);
        declare_parameter("blend_rate", config_.blend_rate);
        declare_parameter("publish_rate_hz", 40.0);
        declare_parameter("enabled", true);
        declare_parameter("map_frame", map_frame_);
        declare_parameter("laser_frame", laser_frame_);
        declare_parameter("base_link_frame", base_link_frame_);
        declare_parameter("obstacles_topic", "/scan_obstacles");
        declare_parameter("odom_topic", "/odom");
        declare_parameter("raceline_topic", "/local_raceline");
        declare_parameter("enable_topic", "/lateral_planner_enable");
    }

    void loadParameters()
    {
        config_.safety_margin_m    = get_parameter("safety_margin_m").as_double();
        config_.min_window_m       = get_parameter("min_window_m").as_double();
        config_.window_time_s      = get_parameter("window_time_s").as_double();
        config_.max_lateral_shift_m = get_parameter("max_lateral_shift_m").as_double();
        config_.min_replan_dist_m  = get_parameter("min_replan_dist_m").as_double();
        config_.blend_rate         = get_parameter("blend_rate").as_double();
        enabled_                   = get_parameter("enabled").as_bool();
        map_frame_                 = get_parameter("map_frame").as_string();
        laser_frame_               = get_parameter("laser_frame").as_string();
        base_link_frame_           = get_parameter("base_link_frame").as_string();
    }

    // ─────────────────────────────────────────────────────────────────────
    //  Raceline loading
    // ─────────────────────────────────────────────────────────────────────

    void loadRaceline()
    {
        const std::string traj_file = get_parameter("trajectory_file").as_string();
        if (traj_file.empty()) {
            RCLCPP_WARN(get_logger(), "No trajectory_file parameter set");
            return;
        }
        if (!planner_->loadRaceline(traj_file)) {
            RCLCPP_ERROR(get_logger(), "Failed to load raceline from: %s",
                         traj_file.c_str());
        } else {
            RCLCPP_INFO(get_logger(), "Loaded %zu waypoints from %s",
                        planner_->size(), traj_file.c_str());
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    //  Callbacks
    // ─────────────────────────────────────────────────────────────────────

    /**
     * @brief Extract opponent centroid and width from obstacle-only LiDAR scan.
     * Transforms scan points into the map frame using TF.
     */
    void obstacleCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Look up laser pose in map frame
        geometry_msgs::msg::TransformStamped tf_stamped;
        try {
            tf_stamped = tf_buffer_->lookupTransform(
                map_frame_, laser_frame_,
                rclcpp::Time(0),
                rclcpp::Duration::from_seconds(0.02));
        } catch (const tf2::TransformException& ex) {
            return;
        }

        const double lx = tf_stamped.transform.translation.x;
        const double ly = tf_stamped.transform.translation.y;
        const auto& q   = tf_stamped.transform.rotation;
        const double laser_yaw =
            std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                       1.0 - 2.0 * (q.y * q.y + q.z * q.z));

        // Collect valid returns
        std::vector<double> pts_x, pts_y;
        const size_t n = msg->ranges.size();
        pts_x.reserve(n);
        pts_y.reserve(n);

        for (size_t i = 0; i < n; ++i) {
            float r = msg->ranges[i];
            if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) {
                continue;
            }
            double angle = msg->angle_min +
                           static_cast<double>(i) * msg->angle_increment +
                           laser_yaw;
            pts_x.push_back(lx + r * std::cos(angle));
            pts_y.push_back(ly + r * std::sin(angle));
        }

        std::lock_guard<std::mutex> lock(state_mutex_);
        if (pts_x.empty()) {
            opp_detected_ = false;
            return;
        }

        // Centroid
        double sum_x = 0.0, sum_y = 0.0;
        for (size_t i = 0; i < pts_x.size(); ++i) {
            sum_x += pts_x[i];
            sum_y += pts_y[i];
        }
        opp_x_ = sum_x / static_cast<double>(pts_x.size());
        opp_y_ = sum_y / static_cast<double>(pts_y.size());

        // Approximate width from angular extent × mean range
        double mean_range = 0.0;
        for (size_t i = 0; i < n; ++i) {
            float r = msg->ranges[i];
            if (std::isfinite(r) && r >= msg->range_min && r < msg->range_max) {
                mean_range += r;
            }
        }
        mean_range /= static_cast<double>(pts_x.size());
        // Angular extent of valid beams
        size_t first_valid = 0, last_valid = 0;
        bool found_first = false;
        for (size_t i = 0; i < n; ++i) {
            float r = msg->ranges[i];
            if (std::isfinite(r) && r >= msg->range_min && r < msg->range_max) {
                if (!found_first) { first_valid = i; found_first = true; }
                last_valid = i;
            }
        }
        double angular_extent = (last_valid - first_valid) * msg->angle_increment;
        opp_width_    = std::max(std::abs(angular_extent) * mean_range, 0.15);
        opp_detected_ = true;
    }

    /**
     * @brief Update the current robot speed from odometry.
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const double vx = msg->twist.twist.linear.x;
        const double vy = msg->twist.twist.linear.y;
        current_speed_ = std::sqrt(vx * vx + vy * vy);
    }

    /**
     * @brief Enable or disable the planner at runtime.
     */
    void enableCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        enabled_ = msg->data;
        RCLCPP_INFO(get_logger(), "Lateral planner %s",
                    enabled_ ? "ENABLED" : "DISABLED");
    }

    // ─────────────────────────────────────────────────────────────────────
    //  Main planning loop (called by wall timer)
    // ─────────────────────────────────────────────────────────────────────

    void planLoop()
    {
        if (!enabled_ || !planner_->hasRaceline()) {
            return;
        }

        // Update robot pose from TF
        if (!updateRobotPose()) {
            return;
        }

        // Snapshot mutable state under lock
        bool   opp_detected;
        double opp_x, opp_y, opp_width, current_speed;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            opp_detected  = opp_detected_;
            opp_x         = opp_x_;
            opp_y         = opp_y_;
            opp_width     = opp_width_;
            current_speed = current_speed_;
        }

        // Run the planner
        PlannerOutput output = planner_->update(
            robot_x_, robot_y_,
            opp_detected, opp_x, opp_y, opp_width,
            current_speed,
            robot_hint_, opp_hint_);

        publishPath(output);
        publishOpponentMarker(opp_detected, opp_x, opp_y, opp_width);
    }

    // ─────────────────────────────────────────────────────────────────────
    //  TF pose lookup
    // ─────────────────────────────────────────────────────────────────────

    /**
     * @brief Fetch the robot pose from TF (map → base_link).
     * @return true if the transform was available
     */
    bool updateRobotPose()
    {
        geometry_msgs::msg::TransformStamped tf_stamped;
        try {
            tf_stamped = tf_buffer_->lookupTransform(
                map_frame_, base_link_frame_,
                rclcpp::Time(0),
                rclcpp::Duration::from_seconds(0.02));
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "TF lookup failed: %s", ex.what());
            return false;
        }

        robot_x_ = tf_stamped.transform.translation.x;
        robot_y_ = tf_stamped.transform.translation.y;
        return true;
    }

    // ─────────────────────────────────────────────────────────────────────
    //  Publishing
    // ─────────────────────────────────────────────────────────────────────

    /**
     * @brief Build and publish a nav_msgs/Path from the planner output.
     * Velocity is encoded in pose.orientation.x (consumed by Pure Pursuit /
     * Stanley nodes).  Heading is encoded as a yaw-only quaternion.
     */
    void publishPath(const PlannerOutput& output)
    {
        nav_msgs::msg::Path path;
        path.header.stamp    = now();
        path.header.frame_id = map_frame_;
        path.poses.reserve(output.xs.size());

        for (size_t i = 0; i < output.xs.size(); ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x    = output.xs[i];
            pose.pose.position.y    = output.ys[i];
            pose.pose.position.z    = 0.0;
            // Encode velocity in orientation.x (convention shared with Python node)
            pose.pose.orientation.x = output.vxs[i];
            // Yaw-only quaternion: qz = sin(psi/2), qw = cos(psi/2)
            pose.pose.orientation.z = std::sin(output.psis[i] * 0.5);
            pose.pose.orientation.w = std::cos(output.psis[i] * 0.5);
            path.poses.push_back(pose);
        }

        raceline_pub_->publish(path);
    }

    /**
     * @brief Publish a cylinder marker at the opponent's position, or delete
     *        the marker when no opponent is visible.
     */
    void publishOpponentMarker(
        bool detected, double x, double y, double width)
    {
        visualization_msgs::msg::MarkerArray markers;
        visualization_msgs::msg::Marker marker;
        marker.header.stamp    = now();
        marker.header.frame_id = map_frame_;
        marker.ns              = "opponent";
        marker.id              = 0;
        marker.type            = visualization_msgs::msg::Marker::CYLINDER;

        if (detected) {
            marker.action            = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x   = x;
            marker.pose.position.y   = y;
            marker.pose.position.z   = 0.1;
            marker.pose.orientation.w = 1.0;
            marker.scale.x           = width;
            marker.scale.y           = width;
            marker.scale.z           = 0.2;
            marker.color.r           = 1.0f;
            marker.color.g           = 0.0f;
            marker.color.b           = 0.0f;
            marker.color.a           = 0.8f;
        } else {
            marker.action = visualization_msgs::msg::Marker::DELETE;
        }

        markers.markers.push_back(marker);
        marker_pub_->publish(markers);
    }
};

}  // namespace f1tenth_lateral_planner

// ── Entry point ───────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node =
        std::make_shared<f1tenth_lateral_planner::LateralPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
