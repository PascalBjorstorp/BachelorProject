#pragma once

#include "gpu_amcl_cpp/core/particle_filter.hpp"
#include "gpu_amcl_cpp/helpers/map_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <mutex>
#include <atomic>

namespace gpu_amcl_cpp {

/**
 * @brief AMCL ROS 2 node — GPU-accelerated particle-filter localisation.
 *
 * Subscribes to laser scans and odometry, publishes pose estimates
 * and a particle cloud.  Does NOT broadcast TF — that is handled
 * by the EKF node.
 */
class AmclNode : public rclcpp::Node {
public:
    explicit AmclNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // ── Callbacks ──────────────────────────────────────────────────
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void initialpose_callback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void publish_timer_callback();

    // ── Helpers ────────────────────────────────────────────────────
    void declare_all_parameters();
    void load_parameters();
    void publish_pose(const PoseEstimate& est, const rclcpp::Time& stamp);

    // ── ROS I/O ────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr       scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr           odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr      map_sub_;
    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr      initpose_sub_;

    rclcpp::Publisher<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr      pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr         cloud_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr               timing_pub_;

    rclcpp::TimerBase::SharedPtr publish_timer_;

    // ── Core ───────────────────────────────────────────────────────
    ParticleFilter pf_;
    MapProcessor   map_;

    // Motion tracking
    bool   odom_received_ = false;
    double prev_x_ = 0, prev_y_ = 0, prev_theta_ = 0;
    double update_min_d_ = 0.001;
    double update_min_a_ = 0.001;
    double max_scan_age_ = 0.05;

    // Thread safety
    std::mutex pf_mutex_;
    std::atomic<bool> processing_scan_{false};

    // Cached estimate for decoupled publishing
    PoseEstimate cached_estimate_;
    std::mutex   estimate_mutex_;

    // Frame IDs
    std::string base_frame_;
    std::string odom_frame_;
    std::string global_frame_;

    // Callback groups (§10.4) — allow scan and odom to run in parallel.
    rclcpp::CallbackGroup::SharedPtr scan_cb_group_;
    rclcpp::CallbackGroup::SharedPtr odom_cb_group_;
};

}  // namespace gpu_amcl_cpp
