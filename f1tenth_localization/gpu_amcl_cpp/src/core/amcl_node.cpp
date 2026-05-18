#include "gpu_amcl_cpp/core/amcl_node.hpp"
#include "gpu_amcl_cpp/helpers/math_utils.hpp"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <exception>
#include <fstream>
#include <limits>
#include <sstream>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

namespace gpu_amcl_cpp {

AmclNode::AmclNode(const rclcpp::NodeOptions& options)
    : Node("gpu_amcl_cpp", options) {

    declare_all_parameters();
    load_parameters();

    // ── Publishers ─────────────────────────────────────────────────
    pose_pub_  = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", rclcpp::QoS(10));
    cloud_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("particlecloud", rclcpp::QoS(2));
    pre_resample_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "particlecloud_weighted_pre_resample", rclcpp::QoS(2));
    timing_pub_ = create_publisher<std_msgs::msg::Float64>("amcl_timing", rclcpp::QoS(10));
    particle_count_pub_ = create_publisher<std_msgs::msg::Int32>("amcl_particle_count", rclcpp::QoS(10));

    // ── Callback groups ──
    // MutuallyExclusive: callbacks in SAME group don't run concurrently
    // But callbacks in DIFFERENT groups CAN run in parallel
    scan_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    odom_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions scan_opts;
    scan_opts.callback_group = scan_cb_group_;
    rclcpp::SubscriptionOptions odom_opts;
    odom_opts.callback_group = odom_cb_group_;

    // ── Subscribers ────────────────────────────────────────────────
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        std::bind(&AmclNode::scan_callback, this, std::placeholders::_1),
        scan_opts); // Uses scan callback group with Best-effort, keep latest

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS(),
        std::bind(&AmclNode::odom_callback, this, std::placeholders::_1),
        odom_opts); // Uses odom callback group with Best-effort, keep latest

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", rclcpp::QoS(1).transient_local(),
        std::bind(&AmclNode::map_callback, this, std::placeholders::_1)); // Wait for map

    initpose_sub_ = create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", rclcpp::QoS(10),
        std::bind(&AmclNode::initialpose_callback, this,
                  std::placeholders::_1));

    if (!std::isfinite(cloud_publish_rate_) || cloud_publish_rate_ <= 0.0) {
        RCLCPP_INFO(get_logger(),
            "Particle cloud publishing disabled (cloud_publish_rate <= 0)");
    } else if (cloud_publish_rate_ >= 39.5) {
        RCLCPP_INFO(get_logger(),
            "Particle cloud publishing every accepted scan (configured %.1f Hz)",
            cloud_publish_rate_);
    } else {
        RCLCPP_INFO(get_logger(),
            "Particle cloud publishing scan-synchronized, max %.1f Hz",
            cloud_publish_rate_);
    }

    RCLCPP_INFO(get_logger(), "GPU AMCL C++ node created — waiting for map...");
}

// ─── Parameter declaration ──────────────────────────────────────────
void AmclNode::declare_all_parameters() {
    // Particles
    declare_parameter<int>("num_particles", 2000);
    declare_parameter<int>("min_particles", 100);
    declare_parameter<int>("max_particles", 5000);

    // KLD
    declare_parameter<bool>("use_kld_sampling", false);
    declare_parameter<double>("kld_epsilon", 0.05);
    declare_parameter<double>("kld_z", 2.33);
    declare_parameter<double>("kld_bin_x", 0.5);
    declare_parameter<double>("kld_bin_y", 0.5);
    declare_parameter<double>("kld_bin_theta", 0.1);

    // Frames
    declare_parameter<std::string>("base_frame_id", "ego_racecar/base_link");
    declare_parameter<std::string>("odom_frame_id", "ego_racecar/odom");
    declare_parameter<std::string>("global_frame_id", "map");

    // Topics
    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<std::string>("odom_topic", "/ego_racecar/odom");

    // Update thresholds
    declare_parameter<double>("update_min_d", 0.001);
    declare_parameter<double>("update_min_a", 0.001);

    // Scan freshness
    declare_parameter<double>("max_scan_age", 0.012);

    // Motion model
    declare_parameter<double>("alpha1", 0.1);
    declare_parameter<double>("alpha2", 0.1);
    declare_parameter<double>("alpha3", 0.2);
    declare_parameter<double>("alpha4", 0.2);

    // Slip-aware noise scaling
    declare_parameter<double>("slip_angular_threshold", 1.0);  // rad/s — above this, increase noise
    declare_parameter<double>("slip_noise_multiplier", 2.0);   // multiplier when slip detected

    // Sensor model
    declare_parameter<int>("max_beams", 270);
    declare_parameter<double>("z_hit", 0.95);
    declare_parameter<double>("z_rand", 0.05);
    declare_parameter<double>("sigma_hit", 0.2);
    declare_parameter<double>("laser_max_range", 10.0);
    declare_parameter<double>("laser_offset_x", 0.265);
    declare_parameter<double>("laser_offset_y", 0.0);
    declare_parameter<bool>("normalize_likelihood_by_beams", true);
    declare_parameter<double>("likelihood_scale", 1.0);

    // Resampling
    declare_parameter<double>("resample_threshold", 0.5);
    declare_parameter<bool>("enable_recovery_injection", false);
    declare_parameter<double>("recovery_injection_ratio", 0.05);
    declare_parameter<bool>("enable_local_roughening", true);
    declare_parameter<double>("local_roughening_ratio", 0.20);
    declare_parameter<double>("local_roughening_xy_std_m", 0.12);
    declare_parameter<double>("local_roughening_yaw_std_rad", 0.0872664626);
    declare_parameter<double>("local_roughening_bad_log_weight_per_beam", -1.0);
    declare_parameter<double>("local_roughening_max_cloud_std_m", 0.75);
    declare_parameter<bool>("use_cluster_estimate", true);
    declare_parameter<double>("cluster_xy_bin_m", 0.25);
    declare_parameter<double>("cluster_radius_m", 0.75);
    declare_parameter<int>("cluster_iterations", 3);
    declare_parameter<double>("cluster_min_covariance", 1e-4);
    declare_parameter<double>("cluster_publish_min_weight", 0.60);

    // Initial pose
    declare_parameter<bool>("global_initialization", false);
    declare_parameter<bool>("initial_heading_from_raceline", true);
    declare_parameter<double>("global_heading_cone_rad", 0.5235987756);
    declare_parameter<double>("global_track_margin_m", 0.15);
    declare_parameter<double>("global_max_lateral_offset_m", 0.55);
    declare_parameter<std::string>("global_heading_trajectory_file", "");
    declare_parameter<std::string>("global_heading_trajectory_package", "f1tenth_planning");
    declare_parameter<std::string>("global_heading_trajectory_rel_path", "trajectories/my_track_raceline.csv");
    declare_parameter<int>("global_init_min_scans", 8);
    declare_parameter<int>("global_init_required_stable_scans", 3);
    declare_parameter<double>("global_init_min_motion_m", 0.25);
    declare_parameter<double>("global_init_publish_min_weight", 0.85);
    declare_parameter<double>("global_init_min_weight_margin", 0.25);
    declare_parameter<double>("global_init_stability_xy_m", 0.35);
    declare_parameter<double>("global_init_stability_yaw_rad", 0.45);
    declare_parameter<double>("initial_pose_x", 0.0);
    declare_parameter<double>("initial_pose_y", 0.0);
    declare_parameter<double>("initial_pose_a", 0.0);
    declare_parameter<double>("initial_cov_xx", 0.5);
    declare_parameter<double>("initial_cov_yy", 0.5);
    declare_parameter<double>("initial_cov_aa", 0.2);

    // Publishing
    declare_parameter<double>("cloud_publish_rate", 2.0);  // Hz; <= 0 disables particle-cloud publishing
    declare_parameter<bool>("debug_pre_resample_particles", false);

    // Odom alignment
    declare_parameter<double>("odom_history_duration_s", 0.2);
}

void AmclNode::load_parameters() {
    base_frame_   = get_parameter("base_frame_id").as_string();
    odom_frame_   = get_parameter("odom_frame_id").as_string();
    global_frame_ = get_parameter("global_frame_id").as_string();
    scan_topic_   = get_parameter("scan_topic").as_string();
    odom_topic_   = get_parameter("odom_topic").as_string();
    update_min_d_ = get_parameter("update_min_d").as_double();
    update_min_a_ = get_parameter("update_min_a").as_double();
    max_scan_age_ = get_parameter("max_scan_age").as_double();
    cloud_publish_rate_ = get_parameter("cloud_publish_rate").as_double();
    debug_pre_resample_particles_ =
        get_parameter("debug_pre_resample_particles").as_bool();
    initial_heading_from_raceline_ =
        get_parameter("initial_heading_from_raceline").as_bool();
    global_init_min_scans_ =
        std::max(1, static_cast<int>(get_parameter("global_init_min_scans").as_int()));
    global_init_required_stable_scans_ =
        std::max(1, static_cast<int>(
                    get_parameter("global_init_required_stable_scans").as_int()));
    global_init_min_motion_m_ =
        std::max(0.0, get_parameter("global_init_min_motion_m").as_double());
    global_init_publish_min_weight_ =
        std::clamp(get_parameter("global_init_publish_min_weight").as_double(), 0.0, 1.0);
    global_init_min_weight_margin_ =
        std::clamp(get_parameter("global_init_min_weight_margin").as_double(), 0.0, 1.0);
    global_init_stability_xy_m_ =
        std::max(0.0, get_parameter("global_init_stability_xy_m").as_double());
    global_init_stability_yaw_rad_ =
        std::max(0.0, get_parameter("global_init_stability_yaw_rad").as_double());
    slip_angular_threshold_ = get_parameter("slip_angular_threshold").as_double();
    slip_noise_multiplier_  = get_parameter("slip_noise_multiplier").as_double();
    odom_history_duration_s_ = std::max(
        0.0, get_parameter("odom_history_duration_s").as_double());

    RCLCPP_INFO(get_logger(),
        "[AMCL] Parameters: update_min_d=%.5f, update_min_a=%.5f, "
        "max_scan_age=%.4f, odom_history=%.3fs, cloud_publish_rate=%.1f Hz, "
        "debug_pre_resample=%s, slip_threshold=%.2f rad/s, initial_raceline_heading=%s",
        update_min_d_, update_min_a_, max_scan_age_, odom_history_duration_s_,
        cloud_publish_rate_, debug_pre_resample_particles_ ? "true" : "false",
        slip_angular_threshold_,
        initial_heading_from_raceline_ ? "true" : "false");
    RCLCPP_INFO(get_logger(),
        "[AMCL] Global init gate: min_scans=%d, stable_scans=%d, min_motion=%.2f m, "
        "min_weight=%.2f, min_margin=%.2f, stable_xy=%.2f m, stable_yaw=%.2f rad",
        global_init_min_scans_, global_init_required_stable_scans_,
        global_init_min_motion_m_, global_init_publish_min_weight_,
        global_init_min_weight_margin_, global_init_stability_xy_m_,
        global_init_stability_yaw_rad_);
}

void AmclNode::push_odom_sample(const rclcpp::Time& stamp,
                                double x,
                                double y,
                                double theta) {
    odom_history_.push_back({stamp, x, y, theta});

    // Keep one interpolation interval even when the configured time window is very small.
    while (odom_history_.size() > 2 &&
           (stamp - odom_history_.front().stamp).seconds() > odom_history_duration_s_) {
        odom_history_.pop_front();
    }
}

bool AmclNode::interpolate_odom_pose(const rclcpp::Time& stamp,
                                     double& x,
                                     double& y,
                                     double& theta) const {
    if (odom_history_.empty()) {
        return false;
    }

    const auto& first = odom_history_.front();
    if (stamp <= first.stamp) {
        x = first.x;
        y = first.y;
        theta = first.theta;
        return true;
    }

    const auto& last = odom_history_.back();
    if (stamp >= last.stamp) {
        x = last.x;
        y = last.y;
        theta = last.theta;
        return true;
    }

    for (size_t i = 1; i < odom_history_.size(); ++i) {
        const auto& a = odom_history_[i - 1];
        const auto& b = odom_history_[i];

        if (stamp <= b.stamp) {
            const double dt = (b.stamp - a.stamp).seconds();
            if (dt <= 1e-9) {
                x = b.x;
                y = b.y;
                theta = b.theta;
                return true;
            }

            const double t = (stamp - a.stamp).seconds() / dt;
            x = a.x + t * (b.x - a.x);
            y = a.y + t * (b.y - a.y);

            const double dtheta = math_utils::angle_diff(b.theta, a.theta);
            theta = math_utils::normalize_angle(a.theta + t * dtheta);
            return true;
        }
    }

    return false;
}

std::string AmclNode::resolve_global_heading_trajectory_file() const {
    const std::string configured_path =
        get_parameter("global_heading_trajectory_file").as_string();
    if (!configured_path.empty()) {
        return configured_path;
    }

    const std::string package =
        get_parameter("global_heading_trajectory_package").as_string();
    const std::string rel_path =
        get_parameter("global_heading_trajectory_rel_path").as_string();
    if (package.empty() || rel_path.empty()) {
        return "";
    }

    try {
        return ament_index_cpp::get_package_share_directory(package) + "/" + rel_path;
    } catch (const std::exception& ex) {
        RCLCPP_WARN(get_logger(),
                    "Failed to resolve global heading trajectory from package '%s': %s",
                    package.c_str(), ex.what());
        return "";
    }
}

std::vector<ParticleFilter::TrackHeadingPoint> AmclNode::load_global_heading_points() const {
    std::vector<ParticleFilter::TrackHeadingPoint> points;
    const std::string path = resolve_global_heading_trajectory_file();
    if (path.empty()) {
        return points;
    }

    std::ifstream file(path);
    if (!file.is_open()) {
        RCLCPP_WARN(get_logger(), "Cannot open global heading trajectory: %s", path.c_str());
        return points;
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
                values.clear();
                break;
            }
        }

        if (values.size() < 4) {
            continue;
        }

        ParticleFilter::TrackHeadingPoint point;
        point.x = static_cast<float>(values[1]);
        point.y = static_cast<float>(values[2]);
        point.yaw = static_cast<float>(values[3]);
        if (values.size() >= 9) {
            point.d_left = static_cast<float>(std::max(0.0, values[7]));
            point.d_right = static_cast<float>(std::max(0.0, values[8]));
        }
        points.push_back(point);
    }

    RCLCPP_INFO(get_logger(), "Loaded %zu global heading points from %s",
                points.size(), path.c_str());
    return points;
}

double AmclNode::raceline_heading_near_pose(
    double x,
    double y,
    double fallback_yaw,
    const std::vector<ParticleFilter::TrackHeadingPoint>& heading_points) const {
    if (heading_points.empty()) {
        return math_utils::normalize_angle(fallback_yaw);
    }

    const auto* best = &heading_points.front();
    double best_d2 = std::numeric_limits<double>::infinity();
    for (const auto& point : heading_points) {
        const double dx = static_cast<double>(point.x) - x;
        const double dy = static_cast<double>(point.y) - y;
        const double d2 = dx * dx + dy * dy;
        if (d2 < best_d2) {
            best_d2 = d2;
            best = &point;
        }
    }

    return math_utils::normalize_angle(static_cast<double>(best->yaw));
}

// ─── Map callback ───────────────────────────────────────────────────
void AmclNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    static size_t map_msg_count = 0;  // Static local — persists across calls
    ++map_msg_count;

    // Log first vs subsequent maps (debugging — in practice, there should only be one map message (latched)).
    if (map_msg_count == 1) {
        RCLCPP_INFO(get_logger(),
                    "Received first map (%dx%d @ %.3f m/cell) — initialising PF",
                    msg->info.width, msg->info.height, msg->info.resolution);
    } else {
        RCLCPP_WARN(get_logger(),
                    "Received map update #%zu (%dx%d @ %.3f m/cell) — reinitialising PF",
                    map_msg_count, msg->info.width, msg->info.height,
                    msg->info.resolution);
    }

    std::lock_guard<std::mutex> lock(pf_mutex_);

    // 1. Load map into processor (computes distance field)
    map_.load_from_msg(*msg);

    // 2. Build particle filter config from ROS params
    ParticleFilter::Config pf_cfg;
    pf_cfg.num_particles     = get_parameter("num_particles").as_int();
    pf_cfg.min_particles     = get_parameter("min_particles").as_int();
    pf_cfg.max_particles     = get_parameter("max_particles").as_int();
    pf_cfg.resample_threshold = get_parameter("resample_threshold").as_double();
    pf_cfg.use_kld           = get_parameter("use_kld_sampling").as_bool();
    pf_cfg.kld_epsilon       = get_parameter("kld_epsilon").as_double();
    pf_cfg.kld_z             = get_parameter("kld_z").as_double();
    pf_cfg.kld_bin_x         = get_parameter("kld_bin_x").as_double();
    pf_cfg.kld_bin_y         = get_parameter("kld_bin_y").as_double();
    pf_cfg.kld_bin_theta     = get_parameter("kld_bin_theta").as_double();
    pf_cfg.enable_recovery_injection = get_parameter("enable_recovery_injection").as_bool();
    pf_cfg.recovery_injection_ratio = get_parameter("recovery_injection_ratio").as_double();
    pf_cfg.enable_local_roughening = get_parameter("enable_local_roughening").as_bool();
    pf_cfg.local_roughening_ratio = get_parameter("local_roughening_ratio").as_double();
    pf_cfg.local_roughening_xy_std_m = get_parameter("local_roughening_xy_std_m").as_double();
    pf_cfg.local_roughening_yaw_std_rad = get_parameter("local_roughening_yaw_std_rad").as_double();
    pf_cfg.local_roughening_bad_log_weight_per_beam =
        get_parameter("local_roughening_bad_log_weight_per_beam").as_double();
    pf_cfg.local_roughening_max_cloud_std_m =
        get_parameter("local_roughening_max_cloud_std_m").as_double();
    pf_cfg.use_cluster_estimate = get_parameter("use_cluster_estimate").as_bool();
    pf_cfg.cluster_xy_bin_m = get_parameter("cluster_xy_bin_m").as_double();
    pf_cfg.cluster_radius_m = get_parameter("cluster_radius_m").as_double();
    pf_cfg.cluster_iterations = get_parameter("cluster_iterations").as_int();
    pf_cfg.cluster_min_covariance = get_parameter("cluster_min_covariance").as_double();
    pf_cfg.cluster_publish_min_weight =
        get_parameter("cluster_publish_min_weight").as_double();
    pf_cfg.global_initialization = get_parameter("global_initialization").as_bool();
    pf_cfg.global_heading_cone_rad = get_parameter("global_heading_cone_rad").as_double();
    pf_cfg.global_track_margin_m = get_parameter("global_track_margin_m").as_double();
    pf_cfg.global_max_lateral_offset_m = get_parameter("global_max_lateral_offset_m").as_double();
    pf_cfg.init_x            = get_parameter("initial_pose_x").as_double();
    pf_cfg.init_y            = get_parameter("initial_pose_y").as_double();
    pf_cfg.init_a            = get_parameter("initial_pose_a").as_double();
    pf_cfg.init_cov_xx       = get_parameter("initial_cov_xx").as_double();
    pf_cfg.init_cov_yy       = get_parameter("initial_cov_yy").as_double();
    pf_cfg.init_cov_aa       = get_parameter("initial_cov_aa").as_double();

    std::vector<ParticleFilter::TrackHeadingPoint> heading_points;
    if (pf_cfg.global_initialization ||
        pf_cfg.enable_recovery_injection ||
        initial_heading_from_raceline_) {
        heading_points = load_global_heading_points();
    }
    if (pf_cfg.global_initialization || pf_cfg.enable_recovery_injection) {
        pf_cfg.global_heading_points = heading_points;
    }
    if (!pf_cfg.global_initialization && initial_heading_from_raceline_) {
        const double fallback_yaw = pf_cfg.init_a;
        pf_cfg.init_a = raceline_heading_near_pose(
            pf_cfg.init_x, pf_cfg.init_y, fallback_yaw, heading_points);
        if (heading_points.empty()) {
            RCLCPP_WARN(get_logger(),
                        "Initial raceline heading requested but no heading points loaded; using fallback yaw %.3f rad",
                        pf_cfg.init_a);
        } else {
            RCLCPP_INFO(get_logger(),
                        "Initial local pose uses raceline heading: x=%.2f y=%.2f yaw=%.3f rad",
                        pf_cfg.init_x, pf_cfg.init_y, pf_cfg.init_a);
        }
    }

    // 3. Build motion model config
    MotionModel::Config mm_cfg;
    mm_cfg.alpha1 = get_parameter("alpha1").as_double();
    mm_cfg.alpha2 = get_parameter("alpha2").as_double();
    mm_cfg.alpha3 = get_parameter("alpha3").as_double();
    mm_cfg.alpha4 = get_parameter("alpha4").as_double();

    // 4. Build sensor model config
    SensorModel::Config sm_cfg;
    sm_cfg.max_beams       = get_parameter("max_beams").as_int();
    sm_cfg.z_hit           = get_parameter("z_hit").as_double();
    sm_cfg.z_rand          = get_parameter("z_rand").as_double();
    sm_cfg.sigma_hit       = get_parameter("sigma_hit").as_double();
    sm_cfg.laser_max_range = get_parameter("laser_max_range").as_double();
    sm_cfg.laser_offset_x  = get_parameter("laser_offset_x").as_double();
    sm_cfg.laser_offset_y  = get_parameter("laser_offset_y").as_double();
    sm_cfg.normalize_likelihood_by_beams =
        get_parameter("normalize_likelihood_by_beams").as_bool();
    sm_cfg.likelihood_scale = get_parameter("likelihood_scale").as_double();

    // 5. Initialize particle filter with all configs
    pf_.init(pf_cfg, mm_cfg, sm_cfg, map_);

    prediction_baseline_ready_ = false;
    global_localization_active_ = pf_cfg.global_initialization;
    global_init_scan_count_ = 0;
    global_init_stable_count_ = 0;
    global_init_motion_m_ = 0.0;
    global_init_last_est_valid_ = false;
    RCLCPP_INFO(get_logger(), "Particle filter initialised with %d particles (%s)",
                pf_cfg.num_particles,
                pf_cfg.global_initialization ? "global" : "local");

    std_msgs::msg::Int32 particle_count_msg;
    particle_count_msg.data = pf_.num_particles();
    particle_count_pub_->publish(particle_count_msg);
    last_cloud_publish_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    publish_particle_cloud(now());

    // 6. Publish initial pose so EKF can bootstrap. For global particles, the
    // weighted mean is not meaningful until scan updates have concentrated them.
    if (!pf_cfg.global_initialization) {
        auto init_est = pf_.get_estimate();
        publish_pose(init_est, now());
        RCLCPP_INFO(get_logger(), "Published initial pose: (%.2f, %.2f, %.2f)",
                    init_est.x, init_est.y, init_est.theta);
    } else {
        RCLCPP_INFO(get_logger(),
                    "Global particle initialization active; waiting for scans before publishing pose");
    }
}

// ─── Odom callback ──────────────────────────────────────────────────
void AmclNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Extract pose from odom message
    double x     = msg->pose.pose.position.x;
    double y     = msg->pose.pose.position.y;
    double theta = math_utils::quaternion_to_yaw(msg->pose.pose.orientation);

    // Store absolute odom for delta computation in scan callback.
    std::lock_guard<std::mutex> lock(pf_mutex_);

    odom_received_ = true;

    // Update member variables
    prev_x_     = x;
    prev_y_     = y;
    prev_theta_ = theta;

    const auto clock_type = get_clock()->get_clock_type();
    const rclcpp::Time odom_stamp(msg->header.stamp, clock_type);
    push_odom_sample(odom_stamp, x, y, theta);
}

// ─── Scan callback (main PF loop) ──────────────────────────────────
void AmclNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // ── Guard 1: Skip if already processing a scan ──
    // Uses atomic exchange: sets to true, returns previous value
    if (processing_scan_.exchange(true)) {
        static size_t drop_count = 0;
        ++drop_count;
        RCLCPP_WARN(get_logger(), "Dropping scan #%zu — PF can't keep up", drop_count);
        return;
    }

    // ── Guard 2: Skip if not ready ──
    if (!map_.is_loaded() || !odom_received_) {
        processing_scan_ = false;
        return;
    }

    // ── Guard 3: Skip stale scans ──
    // Convert scan stamp to the node clock type to avoid mixed clock-source math.
    const auto clock_type = get_clock()->get_clock_type();
    const rclcpp::Time scan_time(msg->header.stamp, clock_type);
    auto age = (now() - scan_time).seconds();
    if (age > max_scan_age_) {
        processing_scan_ = false;
        return;
    }

    std::lock_guard<std::mutex> lock(pf_mutex_);

    double odom_x_at_scan = 0.0;
    double odom_y_at_scan = 0.0;
    double odom_theta_at_scan = 0.0;
    if (!interpolate_odom_pose(scan_time, odom_x_at_scan, odom_y_at_scan, odom_theta_at_scan)) {
        processing_scan_ = false;
        return;
    }

    // ── Baseline initialization ──
    // For the first scan after startup/reinit, just seed the odom baseline
    if (!prediction_baseline_ready_) {
        pred_last_x_ = odom_x_at_scan;
        pred_last_y_ = odom_y_at_scan;
        pred_last_theta_ = odom_theta_at_scan;
        prediction_baseline_ready_ = true;
        processing_scan_ = false;
        return;
    }

    // ── Compute odom delta (odom frame) ──
    double dx_odom = odom_x_at_scan - pred_last_x_;
    double dy_odom = odom_y_at_scan - pred_last_y_;
    double dtheta  = math_utils::angle_diff(odom_theta_at_scan, pred_last_theta_);

    // ── Guard 4: Skip update if robot hasn't moved enough ──
    double dist_moved = std::sqrt(dx_odom * dx_odom + dy_odom * dy_odom);
    if (dist_moved < update_min_d_ && std::abs(dtheta) < update_min_a_) {
        processing_scan_ = false;
        return;
    }

    // ── Transform odom-frame delta to robot-frame ──
    // Robot's last heading determines the rotation
    double c = std::cos(pred_last_theta_);
    double s = std::sin(pred_last_theta_);
    float dx_robot = static_cast<float>( dx_odom * c + dy_odom * s);
    float dy_robot = static_cast<float>(-dx_odom * s + dy_odom * c);

    // Update baseline for next iteration
    pred_last_x_     = odom_x_at_scan;
    pred_last_y_     = odom_y_at_scan;
    pred_last_theta_ = odom_theta_at_scan;

    // ── Timing start ──
    auto t_pf_start = std::chrono::high_resolution_clock::now();

    // ═══════════════════════════════════════════════════════════
    // STEP 1: PREDICT — Propagate particles by odom delta + noise
    // ═══════════════════════════════════════════════════════════

    // Slip-aware noise scaling: increase noise during aggressive turns
    double dt = 0.0;
    if (last_scan_time_.nanoseconds() != 0) {
        dt = (scan_time - last_scan_time_).seconds();
    }
    if (dt > 0.001 && dt < 1.0) {  // Valid dt range
        double angular_velocity = std::abs(dtheta) / dt;
        if (angular_velocity > slip_angular_threshold_) {
            pf_.motion_model().set_noise_multiplier(slip_noise_multiplier_);
        } else {
            pf_.motion_model().reset_noise_multiplier();
        }
    }
    last_scan_time_ = scan_time;

    pf_.predict(dx_robot, dy_robot, static_cast<float>(dtheta)); 

    // ═══════════════════════════════════════════════════════════
    // STEP 2: UPDATE — Weight particles by scan, debug, then resample
    // ═══════════════════════════════════════════════════════════
    const bool weights_updated = pf_.update_weights(
        msg->ranges.data(),
        static_cast<int>(msg->ranges.size()),
        msg->angle_min,
        msg->angle_increment);
    if (!weights_updated) {
        processing_scan_ = false;
        return;
    }
    publish_pre_resample_weighted_cloud(scan_time);

    // Estimate from the strongest local mode before resampling. A global
    // weighted mean is invalid while particles are split across plausible poses.
    double cluster_weight = 1.0;
    double second_cluster_weight = 0.0;
    auto est = pf_.get_cluster_estimate(&cluster_weight, &second_cluster_weight);

    const double min_cluster_weight =
        std::clamp(pf_.config().cluster_publish_min_weight, 0.0, 1.0);
    bool publish_cluster =
        !pf_.config().use_cluster_estimate || cluster_weight >= min_cluster_weight;
    bool resample_now = true;

    if (global_localization_active_ && pf_.config().global_initialization) {
        ++global_init_scan_count_;
        global_init_motion_m_ += dist_moved;

        if (global_init_last_est_valid_) {
            const double dx_est = est.x - global_init_last_est_.x;
            const double dy_est = est.y - global_init_last_est_.y;
            const double dxy_est = std::hypot(dx_est, dy_est);
            const double dyaw_est = std::abs(
                math_utils::angle_diff(est.theta, global_init_last_est_.theta));
            if (dxy_est <= global_init_stability_xy_m_ &&
                dyaw_est <= global_init_stability_yaw_rad_) {
                ++global_init_stable_count_;
            } else {
                global_init_stable_count_ = 1;
            }
        } else {
            global_init_stable_count_ = 1;
        }
        global_init_last_est_ = est;
        global_init_last_est_valid_ = true;

        const bool enough_scans =
            global_init_scan_count_ >= global_init_min_scans_;
        const bool enough_motion =
            global_init_motion_m_ >= global_init_min_motion_m_;
        const bool stable =
            global_init_stable_count_ >= global_init_required_stable_scans_;
        const bool strong_weight =
            cluster_weight >= global_init_publish_min_weight_;
        const bool clear_winner =
            (cluster_weight - second_cluster_weight) >= global_init_min_weight_margin_;

        publish_cluster =
            enough_scans && enough_motion && stable && strong_weight && clear_winner;
        resample_now = publish_cluster;

        if (publish_cluster) {
            global_localization_active_ = false;
            pf_.set_recovery_injection_enabled(false);
            RCLCPP_INFO(get_logger(),
                        "Global AMCL accepted: scans=%d motion=%.2fm stable=%d "
                        "weight=%.3f second=%.3f margin=%.3f pose=(%.2f, %.2f, %.2f)",
                        global_init_scan_count_, global_init_motion_m_,
                        global_init_stable_count_, cluster_weight,
                        second_cluster_weight, cluster_weight - second_cluster_weight,
                        est.x, est.y, est.theta);
        } else if ((global_init_scan_count_ % 20) == 0) {
            RCLCPP_INFO(get_logger(),
                        "Global AMCL waiting: scans=%d/%d motion=%.2f/%.2fm stable=%d/%d "
                        "weight=%.3f/%.3f second=%.3f margin=%.3f/%.3f",
                        global_init_scan_count_, global_init_min_scans_,
                        global_init_motion_m_, global_init_min_motion_m_,
                        global_init_stable_count_, global_init_required_stable_scans_,
                        cluster_weight, global_init_publish_min_weight_,
                        second_cluster_weight, cluster_weight - second_cluster_weight,
                        global_init_min_weight_margin_);
        }
    }

    if (resample_now) {
        pf_.resample_if_needed();
    }

    // ── Timing end — publish if subscribed ──
    auto t_pf_end = std::chrono::high_resolution_clock::now();
    if (timing_pub_->get_subscription_count() > 0) {
        double pf_ms = std::chrono::duration<double, std::milli>(
                           t_pf_end - t_pf_start).count();
        std_msgs::msg::Float64 timing_msg;
        timing_msg.data = pf_ms;
        timing_pub_->publish(timing_msg);
    }

    std_msgs::msg::Int32 particle_count_msg;
    particle_count_msg.data = pf_.num_particles();
    particle_count_pub_->publish(particle_count_msg);

    if (publish_cluster) {
        // Publish only when the best local mode is dominant enough. During
        // global localization, ambiguous clusters should stay visible in RViz
        // but not pull the EKF/controller to a wrong symmetric pose.
        publish_pose(est, msg->header.stamp);
    }

    // Cache estimate for particle cloud visualization
    if (publish_cluster) {
        std::lock_guard<std::mutex> lk(estimate_mutex_);
        cached_estimate_ = est;
    }
    publish_particle_cloud(scan_time);

    processing_scan_ = false; // Allow next scan
}

// ─── Initial-pose callback ──────────────────────────────────────────
void AmclNode::initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    // Used for manual reinitialization from RViz — re-seeds particles around new pose and resets odom baseline.
    // Activated by "2D Pose Estimate" tool in RViz
    double x     = msg->pose.pose.position.x;
    double y     = msg->pose.pose.position.y;
    double theta = math_utils::quaternion_to_yaw(msg->pose.pose.orientation);

    RCLCPP_INFO(get_logger(), "Re-initialising PF at (%.2f, %.2f, %.2f)", x, y, theta);

    std::lock_guard<std::mutex> lock(pf_mutex_);

    // Reinitialize particles around new pose
    pf_.reinitialize(x, y, theta,
                     get_parameter("initial_cov_xx").as_double(),
                     get_parameter("initial_cov_yy").as_double(),
                     get_parameter("initial_cov_aa").as_double());

    std_msgs::msg::Int32 particle_count_msg;
    particle_count_msg.data = pf_.num_particles();
    particle_count_pub_->publish(particle_count_msg);
    last_cloud_publish_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    publish_particle_cloud(now());

    // Reset odom baseline
    prediction_baseline_ready_ = false;
    global_localization_active_ = false;
    global_init_scan_count_ = 0;
    global_init_stable_count_ = 0;
    global_init_motion_m_ = 0.0;
    global_init_last_est_valid_ = false;
}

// ─── Direct pose publish (called from scan_callback) ────────────────
void AmclNode::publish_pose(const PoseEstimate& est, const rclcpp::Time& stamp) {
    auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    
    pose_msg.header.stamp    = stamp;           // Use scan's timestamp
    pose_msg.header.frame_id = global_frame_;   // "map"
    
    // Position
    pose_msg.pose.pose.position.x = est.x;
    pose_msg.pose.pose.position.y = est.y;
    pose_msg.pose.pose.position.z = 0.0;

    // Orientation (yaw → quaternion)
    pose_msg.pose.pose.orientation = math_utils::yaw_to_quaternion(est.theta);

    // 6×6 covariance: fill from 3×3
    // ROS uses indices: x=0, y=1, z=2, roll=3, pitch=4, yaw=5
    auto& cov = pose_msg.pose.covariance;
    std::fill(cov.begin(), cov.end(), 0.0);

    // Map 3×3 → 6×6:
    // [0]  = xx,  [1]  = xy,  [5]  = x-yaw
    // [6]  = yx,  [7]  = yy,  [11] = y-yaw
    // [30] = yaw-x, [31] = yaw-y, [35] = yaw-yaw
    cov[0]  = est.covariance(0, 0);  // xx
    cov[1]  = est.covariance(0, 1);  // xy
    cov[5]  = est.covariance(0, 2);  // x-yaw
    cov[6]  = est.covariance(1, 0);  // yx
    cov[7]  = est.covariance(1, 1);  // yy
    cov[11] = est.covariance(1, 2);  // y-yaw
    cov[30] = est.covariance(2, 0);  // yaw-x
    cov[31] = est.covariance(2, 1);  // yaw-y
    cov[35] = est.covariance(2, 2);  // yaw-yaw

    pose_pub_->publish(pose_msg);
}

// ─── Particle cloud publish (called after PF updates) ───────────────
void AmclNode::publish_particle_cloud(const rclcpp::Time& stamp) {
    if (!std::isfinite(cloud_publish_rate_) || cloud_publish_rate_ <= 0.0) {
        return;
    }
    if (cloud_pub_->get_subscription_count() == 0) {
        return;
    }

    // The benchmark uses 40 Hz scans. At 40 Hz or higher, publish every
    // accepted scan so visualization/recording does not drop clouds due to
    // timestamp jitter around exactly 25 ms.
    if (cloud_publish_rate_ < 39.5) {
        const double period_s = 1.0 / cloud_publish_rate_;
        if (last_cloud_publish_time_.nanoseconds() != 0) {
            const double elapsed_s = (stamp - last_cloud_publish_time_).seconds();
            if (elapsed_s >= 0.0 && elapsed_s + 1e-6 < period_s) {
                return;
            }
        }
    }

    std::vector<float> particles;
    pf_.get_particles(particles);

    auto cloud = geometry_msgs::msg::PoseArray();
    cloud.header.stamp = stamp;
    cloud.header.frame_id = global_frame_;

    const int np = static_cast<int>(particles.size() / 3);
    cloud.poses.reserve(np);

    for (int i = 0; i < np; ++i) {
        geometry_msgs::msg::Pose p;
        p.position.x = particles[i * 3 + 0];
        p.position.y = particles[i * 3 + 1];
        p.position.z = 0.0;
        p.orientation = math_utils::yaw_to_quaternion(particles[i * 3 + 2]);
        cloud.poses.push_back(p);
    }

    cloud_pub_->publish(cloud);
    last_cloud_publish_time_ = stamp;
}

void AmclNode::publish_pre_resample_weighted_cloud(const rclcpp::Time& stamp) {
    if (!debug_pre_resample_particles_) {
        return;
    }
    if (pre_resample_cloud_pub_->get_subscription_count() == 0) {
        return;
    }

    std::vector<float> particles;
    std::vector<float> weights;
    pf_.get_particles(particles, weights);

    const int np = static_cast<int>(weights.size());
    auto cloud = sensor_msgs::msg::PointCloud2();
    cloud.header.stamp = stamp;
    cloud.header.frame_id = global_frame_;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(np);
    cloud.is_bigendian = false;
    cloud.is_dense = true;

    const auto make_field = [](const char* name, uint32_t offset) {
        sensor_msgs::msg::PointField field;
        field.name = name;
        field.offset = offset;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        return field;
    };
    cloud.fields = {
        make_field("x", 0),
        make_field("y", 4),
        make_field("z", 8),
        make_field("yaw", 12),
        make_field("weight", 16),
    };
    cloud.point_step = 20;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data.resize(cloud.row_step);

    for (int i = 0; i < np; ++i) {
        const float values[5] = {
            particles[i * 3 + 0],
            particles[i * 3 + 1],
            0.0f,
            particles[i * 3 + 2],
            weights[i],
        };
        std::memcpy(&cloud.data[static_cast<size_t>(i) * cloud.point_step],
                    values,
                    sizeof(values));
    }

    pre_resample_cloud_pub_->publish(cloud);
}

}  // namespace gpu_amcl_cpp
