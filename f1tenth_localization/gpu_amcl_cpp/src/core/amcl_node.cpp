#include "gpu_amcl_cpp/core/amcl_node.hpp"
#include "gpu_amcl_cpp/helpers/math_utils.hpp"

#include <chrono>
#include <algorithm>
#include <std_msgs/msg/float64.hpp>

using namespace std::chrono_literals;

namespace gpu_amcl_cpp {

AmclNode::AmclNode(const rclcpp::NodeOptions& options)
    : Node("gpu_amcl_cpp", options) {

    declare_all_parameters();
    load_parameters();

    // ── Publishers ─────────────────────────────────────────────────
    pose_pub_  = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", rclcpp::QoS(10));
    cloud_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("particlecloud", rclcpp::QoS(2));
    timing_pub_ = create_publisher<std_msgs::msg::Float64>("amcl_timing", rclcpp::QoS(10));

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

    // ── Timer for particle cloud visualization ──
    // Uses separate rate to save GPU→CPU bandwidth (default 2 Hz)
    double cloud_rate = get_parameter("cloud_publish_rate").as_double();
    auto cloud_period = std::chrono::duration<double>(1.0 / cloud_rate);
    publish_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(cloud_period),
        std::bind(&AmclNode::publish_timer_callback, this));

    RCLCPP_INFO(get_logger(), "GPU AMCL C++ node created — waiting for map...");
}

// ─── Parameter declaration ──────────────────────────────────────────
void AmclNode::declare_all_parameters() {
    // General
    declare_parameter<bool>("use_gpu", true);

    // Particles
    declare_parameter<int>("num_particles", 2000);
    declare_parameter<int>("min_particles", 100);
    declare_parameter<int>("max_particles", 5000);

    // KLD
    declare_parameter<bool>("use_kld_sampling", false);
    declare_parameter<double>("kld_epsilon", 0.05);
    declare_parameter<double>("kld_z", 2.33);

    // Frames
    declare_parameter<std::string>("base_frame_id", "ego_racecar/base_link");
    declare_parameter<std::string>("odom_frame_id", "ego_racecar/odom");
    declare_parameter<std::string>("global_frame_id", "map");

    // Topics
    declare_parameter<std::string>("scan_topic", "/scan_walls");
    declare_parameter<std::string>("odom_topic", "/ego_racecar/odom");

    // Update thresholds
    declare_parameter<double>("update_min_d", 0.001);
    declare_parameter<double>("update_min_a", 0.001);

    // Scan freshness
    declare_parameter<double>("max_scan_age", 0.05);

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

    // Resampling
    declare_parameter<double>("resample_threshold", 0.5);

    // Initial pose
    declare_parameter<double>("initial_pose_x", 0.0);
    declare_parameter<double>("initial_pose_y", 0.0);
    declare_parameter<double>("initial_pose_a", 0.0);
    declare_parameter<double>("initial_cov_xx", 0.5);
    declare_parameter<double>("initial_cov_yy", 0.5);
    declare_parameter<double>("initial_cov_aa", 0.2);

    // Publishing
    declare_parameter<double>("publish_rate", 40.0);
    declare_parameter<double>("cloud_publish_rate", 2.0);  // Particle cloud rate (Hz) — lower to save bandwidth
    declare_parameter<double>("transform_tolerance", 1.0);

    // Odom alignment
    declare_parameter<int>("odom_history_max_size", 500);
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
    slip_angular_threshold_ = get_parameter("slip_angular_threshold").as_double();
    slip_noise_multiplier_  = get_parameter("slip_noise_multiplier").as_double();
    const int64_t odom_history_size_param =
        get_parameter("odom_history_max_size").as_int();
    odom_history_max_size_ = static_cast<size_t>(
        std::max<int64_t>(2, odom_history_size_param));

    RCLCPP_INFO(get_logger(),
        "[AMCL] Parameters: update_min_d=%.5f, update_min_a=%.5f, "
        "max_scan_age=%.4f, slip_threshold=%.2f rad/s",
        update_min_d_, update_min_a_, max_scan_age_, slip_angular_threshold_);
}

void AmclNode::push_odom_sample(const rclcpp::Time& stamp,
                                double x,
                                double y,
                                double theta) {
    odom_history_.push_back({stamp, x, y, theta});

    while (odom_history_.size() > odom_history_max_size_) {
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
    pf_cfg.init_x            = get_parameter("initial_pose_x").as_double();
    pf_cfg.init_y            = get_parameter("initial_pose_y").as_double();
    pf_cfg.init_a            = get_parameter("initial_pose_a").as_double();
    pf_cfg.init_cov_xx       = get_parameter("initial_cov_xx").as_double();
    pf_cfg.init_cov_yy       = get_parameter("initial_cov_yy").as_double();
    pf_cfg.init_cov_aa       = get_parameter("initial_cov_aa").as_double();

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

    // 5. Initialize particle filter with all configs
    pf_.init(pf_cfg, mm_cfg, sm_cfg, map_);

    prediction_baseline_ready_ = false;
    RCLCPP_INFO(get_logger(), "Particle filter initialised with %d particles", pf_cfg.num_particles);

    // 6. Publish initial pose so EKF can bootstrap
    auto init_est = pf_.get_estimate();
    publish_pose(init_est, now());

    RCLCPP_INFO(get_logger(), "Published initial pose: (%.2f, %.2f, %.2f)", init_est.x, init_est.y, init_est.theta);
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
    // STEP 2: UPDATE — Weight particles by scan + resample
    // ═══════════════════════════════════════════════════════════
    pf_.update(msg->ranges.data(),
               static_cast<int>(msg->ranges.size()),
               msg->angle_min, msg->angle_increment);

    // ═══════════════════════════════════════════════════════════
    // STEP 3: GET ESTIMATE — Weighted mean of particles
    // ═══════════════════════════════════════════════════════════
    auto est = pf_.get_estimate();

    // ── Timing end — publish if subscribed ──
    auto t_pf_end = std::chrono::high_resolution_clock::now();
    if (timing_pub_->get_subscription_count() > 0) {
        double pf_ms = std::chrono::duration<double, std::milli>(
                           t_pf_end - t_pf_start).count();
        std_msgs::msg::Float64 timing_msg;
        timing_msg.data = pf_ms;
        timing_pub_->publish(timing_msg);
    }

    // ═══════════════════════════════════════════════════════════
    // STEP 4: PUBLISH POSE — Immediately, with scan's timestamp
    // ═══════════════════════════════════════════════════════════
    publish_pose(est, msg->header.stamp);

    // Cache estimate for particle cloud visualization
    {
        std::lock_guard<std::mutex> lk(estimate_mutex_);
        cached_estimate_ = est;
    }

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

    // Reset odom baseline
    prediction_baseline_ready_ = false;
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

// ─── Publish timer (particle cloud visualisation only) ──────────────
void AmclNode::publish_timer_callback() {
    // Only publish if someone is subscribed (RViz)
    if (cloud_pub_->get_subscription_count() > 0) {
        std::vector<float> particles, weights;
        {
            std::lock_guard<std::mutex> lock(pf_mutex_);
            pf_.get_particles(particles, weights);
        }

        auto cloud = geometry_msgs::msg::PoseArray();
        cloud.header.stamp    = now();
        cloud.header.frame_id = global_frame_;

        int np = static_cast<int>(weights.size());
        int step = std::max(1, np / 100); // Downsample to ~100 particles

        for (int i = 0; i < np; i += step) {
            geometry_msgs::msg::Pose p;
            p.position.x = particles[i * 3 + 0];
            p.position.y = particles[i * 3 + 1];
            p.position.z = 0.0;
            p.orientation = math_utils::yaw_to_quaternion(particles[i * 3 + 2]);
            cloud.poses.push_back(p);
        }
        cloud_pub_->publish(cloud);
    }
}

}  // namespace gpu_amcl_cpp
