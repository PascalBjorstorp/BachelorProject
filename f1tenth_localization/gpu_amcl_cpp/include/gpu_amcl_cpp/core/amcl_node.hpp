#pragma once

#include "gpu_amcl_cpp/core/particle_filter.hpp"
#include "gpu_amcl_cpp/helpers/map_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <mutex>
#include <atomic>
#include <deque>

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
    void publish_particle_cloud(const rclcpp::Time& stamp);
    void publish_pre_resample_weighted_cloud(const rclcpp::Time& stamp);

    // ── Helpers ────────────────────────────────────────────────────
    void declare_all_parameters();
    void load_parameters();
    void publish_pose(const PoseEstimate& est, const rclcpp::Time& stamp);
    std::string resolve_global_heading_trajectory_file() const;
    std::vector<ParticleFilter::TrackHeadingPoint> load_global_heading_points() const;
    double raceline_heading_near_pose(
        double x,
        double y,
        double fallback_yaw,
        const std::vector<ParticleFilter::TrackHeadingPoint>& heading_points) const;
    void push_odom_sample(const rclcpp::Time& stamp,
                          double x,
                          double y,
                          double theta);
    bool interpolate_odom_pose(const rclcpp::Time& stamp,
                               double& x,
                               double& y,
                               double& theta) const;

    // ── ROS I/O ────────────────────────────────────────────────────
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initpose_sub_;

    // Publishers    
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;  // /amcl_pose
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr cloud_pub_;                 // /particlecloud
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pre_resample_cloud_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr timing_pub_;                       // /amcl_timing
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr particle_count_pub_;                  // /amcl_particle_count
    
    // ── Core ───────────────────────────────────────────────────────
    ParticleFilter pf_;     // The particle filter
    MapProcessor   map_;    // Map + distance field

    // Motion tracking
    bool   odom_received_ = false;
    double prev_x_ = 0;
    double prev_y_ = 0;
    double prev_theta_ = 0;
    double update_min_d_ = 0.001;
    double update_min_a_ = 0.001;
    double max_scan_age_ = 0.05;
    double cloud_publish_rate_ = 2.0;
    rclcpp::Time last_cloud_publish_time_;
    bool debug_pre_resample_particles_ = false;
    bool initial_heading_from_raceline_ = true;

    // Global-localization acceptance gate. While active, scan evidence is
    // accumulated without resampling so early wrong modes cannot kill others.
    bool global_localization_active_ = false;
    int global_init_scan_count_ = 0;
    int global_init_stable_count_ = 0;
    double global_init_motion_m_ = 0.0;
    bool global_init_last_est_valid_ = false;
    PoseEstimate global_init_last_est_;
    int global_init_min_scans_ = 8;
    int global_init_required_stable_scans_ = 3;
    double global_init_min_motion_m_ = 0.25;
    double global_init_publish_min_weight_ = 0.85;
    double global_init_min_weight_margin_ = 0.25;
    double global_init_stability_xy_m_ = 0.35;
    double global_init_stability_yaw_rad_ = 0.45;

    // Slip-aware noise scaling
    double slip_angular_threshold_ = 1.0;  // rad/s
    double slip_noise_multiplier_ = 2.0;
    rclcpp::Time last_scan_time_;          // For dt calculation

    // Prediction baseline (reset on reinit)
    bool prediction_baseline_ready_ = false;
    double pred_last_x_ = 0;
    double pred_last_y_ = 0;
    double pred_last_theta_ = 0;

    struct OdomSample {
        rclcpp::Time stamp;
        double x;
        double y;
        double theta;
    };

    std::deque<OdomSample> odom_history_;
    double odom_history_duration_s_ = 0.2;

    // Thread safety
    std::mutex pf_mutex_;                       // Protects pf_ during GPU ops
    std::atomic<bool> processing_scan_{false};  // Drop scans during processing

    // Cached estimate for decoupled publishing
    PoseEstimate cached_estimate_;
    std::mutex   estimate_mutex_;

    // Frame IDs and topic names
    std::string base_frame_;
    std::string odom_frame_;
    std::string global_frame_;
    std::string scan_topic_;
    std::string odom_topic_;

    // Callback groups for parallel execution
    rclcpp::CallbackGroup::SharedPtr scan_cb_group_;
    rclcpp::CallbackGroup::SharedPtr odom_cb_group_;
};

}  // namespace gpu_amcl_cpp
