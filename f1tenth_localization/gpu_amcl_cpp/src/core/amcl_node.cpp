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
    pose_pub_  = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "amcl_pose", rclcpp::QoS(10));
    cloud_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
        "particlecloud", rclcpp::QoS(2));
    timing_pub_ = create_publisher<std_msgs::msg::Float64>(
        "amcl_timing", rclcpp::QoS(10));

    // ── Callback groups (§10.4) — allow parallel execution ───────
    scan_cb_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    odom_cb_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions scan_opts;
    scan_opts.callback_group = scan_cb_group_;
    rclcpp::SubscriptionOptions odom_opts;
    odom_opts.callback_group = odom_cb_group_;

    // ── Subscribers ────────────────────────────────────────────────
    auto scan_topic = get_parameter("scan_topic").as_string();
    auto odom_topic = get_parameter("odom_topic").as_string();

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, rclcpp::SensorDataQoS(),
        std::bind(&AmclNode::scan_callback, this, std::placeholders::_1),
        scan_opts);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::SensorDataQoS(),
        std::bind(&AmclNode::odom_callback, this, std::placeholders::_1),
        odom_opts);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", rclcpp::QoS(1).transient_local(),
        std::bind(&AmclNode::map_callback, this, std::placeholders::_1));

    initpose_sub_ = create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", rclcpp::QoS(10),
        std::bind(&AmclNode::initialpose_callback, this,
                  std::placeholders::_1));

    // ── Viz timer (particle cloud only, pose published from scan_callback) ──
    double rate = get_parameter("publish_rate").as_double();
    auto period = std::chrono::duration<double>(1.0 / rate);
    publish_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
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

    // Recovery
    declare_parameter<bool>("use_recovery", false);

    // Initial pose
    declare_parameter<double>("initial_pose_x", 0.0);
    declare_parameter<double>("initial_pose_y", 0.0);
    declare_parameter<double>("initial_pose_a", 0.0);
    declare_parameter<double>("initial_cov_xx", 0.5);
    declare_parameter<double>("initial_cov_yy", 0.5);
    declare_parameter<double>("initial_cov_aa", 0.2);

    // Publishing
    declare_parameter<double>("publish_rate", 40.0);
    declare_parameter<double>("transform_tolerance", 1.0);

    // IMU
    declare_parameter<bool>("use_imu_rotation", false);
    declare_parameter<double>("imu_gyro_weight", 0.8);
}

void AmclNode::load_parameters() {
    base_frame_   = get_parameter("base_frame_id").as_string();
    odom_frame_   = get_parameter("odom_frame_id").as_string();
    global_frame_ = get_parameter("global_frame_id").as_string();
    update_min_d_ = get_parameter("update_min_d").as_double();
    update_min_a_ = get_parameter("update_min_a").as_double();
    max_scan_age_ = get_parameter("max_scan_age").as_double();

    RCLCPP_INFO(get_logger(),
        "[AMCL] Parameters: update_min_d=%.5f, update_min_a=%.5f, "
        "max_scan_age=%.4f",
        update_min_d_, update_min_a_, max_scan_age_);
}

// ─── Map callback ───────────────────────────────────────────────────
void AmclNode::map_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "Received map (%dx%d @ %.3f m/cell)",
                msg->info.width, msg->info.height, msg->info.resolution);

    std::lock_guard<std::mutex> lock(pf_mutex_);
    map_.load_from_msg(*msg);

    // Build configs from parameters.
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
    pf_cfg.use_recovery      = get_parameter("use_recovery").as_bool();

    MotionModel::Config mm_cfg;
    mm_cfg.alpha1         = get_parameter("alpha1").as_double();
    mm_cfg.alpha2         = get_parameter("alpha2").as_double();
    mm_cfg.alpha3         = get_parameter("alpha3").as_double();
    mm_cfg.alpha4         = get_parameter("alpha4").as_double();
    mm_cfg.use_imu        = get_parameter("use_imu_rotation").as_bool();
    mm_cfg.imu_gyro_weight = get_parameter("imu_gyro_weight").as_double();

    SensorModel::Config sm_cfg;
    sm_cfg.max_beams       = get_parameter("max_beams").as_int();
    sm_cfg.z_hit           = get_parameter("z_hit").as_double();
    sm_cfg.z_rand          = get_parameter("z_rand").as_double();
    sm_cfg.sigma_hit       = get_parameter("sigma_hit").as_double();
    sm_cfg.laser_max_range = get_parameter("laser_max_range").as_double();
    sm_cfg.laser_offset_x  = get_parameter("laser_offset_x").as_double();
    sm_cfg.laser_offset_y  = get_parameter("laser_offset_y").as_double();

    pf_.init(pf_cfg, mm_cfg, sm_cfg, map_);
    RCLCPP_INFO(get_logger(), "Particle filter initialised with %d particles",
                pf_cfg.num_particles);

    // Publish the initial pose immediately so the EKF can bootstrap
    // the map→odom TF even before the car moves.
    auto init_est = pf_.get_estimate();
    publish_pose(init_est, now());
    RCLCPP_INFO(get_logger(),
                "Published initial pose: (%.2f, %.2f, %.2f)",
                init_est.x, init_est.y, init_est.theta);
}

// ─── Odom callback ──────────────────────────────────────────────────
void AmclNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double x     = msg->pose.pose.position.x;
    double y     = msg->pose.pose.position.y;
    double theta = math_utils::quaternion_to_yaw(msg->pose.pose.orientation);

    // Store absolute odom for delta computation in scan callback.
    std::lock_guard<std::mutex> lock(pf_mutex_);
    if (!odom_received_) {
        odom_received_ = true;
    }
    prev_x_     = x;
    prev_y_     = y;
    prev_theta_ = theta;
}

// ─── Scan callback (main PF loop) ──────────────────────────────────
void AmclNode::scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Drop scan if still processing the previous one.
    if (processing_scan_.exchange(true)) {
        return;
    }

    if (!map_.is_loaded() || !odom_received_) {
        processing_scan_ = false;
        return;
    }

    // Stale-scan guard.
    auto age = (now() - msg->header.stamp).seconds();
    if (age > max_scan_age_) {
        processing_scan_ = false;
        return;
    }

    std::lock_guard<std::mutex> lock(pf_mutex_);

    // Compute odom delta in robot frame.
    // (For the first scan we skip the update — just record the pose.)
    static bool first_scan = true;
    static double last_x = 0, last_y = 0, last_theta = 0;

    double dx_world = prev_x_ - last_x;
    double dy_world = prev_y_ - last_y;
    double dtheta   = math_utils::angle_diff(prev_theta_, last_theta);

    if (first_scan) {
        last_x = prev_x_;
        last_y = prev_y_;
        last_theta = prev_theta_;
        first_scan = false;
        processing_scan_ = false;
        return;
    }

    // Transform world-frame delta to robot-frame.
    double c = std::cos(last_theta);
    double s = std::sin(last_theta);
    float dx_robot = static_cast<float>( dx_world * c + dy_world * s);
    float dy_robot = static_cast<float>(-dx_world * s + dy_world * c);

    last_x     = prev_x_;
    last_y     = prev_y_;
    last_theta = prev_theta_;

    // ── Timing start ──
    auto t_pf_start = std::chrono::high_resolution_clock::now();

    // 1. Predict — always run with current odom delta (includes IMU).
    pf_.predict(dx_robot, dy_robot,
                static_cast<float>(dtheta), 0.0f);

    // 2. Update (sensor model + resampling).
    pf_.update(msg->ranges.data(),
               static_cast<int>(msg->ranges.size()),
               msg->angle_min, msg->angle_increment);

    // 3. Get estimate.
    auto est = pf_.get_estimate();

    // ── Timing end — publish processing time (ms) ──
    auto t_pf_end = std::chrono::high_resolution_clock::now();
    double pf_ms = std::chrono::duration<double, std::milli>(
                       t_pf_end - t_pf_start).count();
    std_msgs::msg::Float64 timing_msg;
    timing_msg.data = pf_ms;
    timing_pub_->publish(timing_msg);

    // 4. Publish pose IMMEDIATELY — no timer delay.
    //    Use scan's original timestamp for correct EKF time alignment.
    publish_pose(est, msg->header.stamp);

    // Cache for particle cloud visualisation timer.
    {
        std::lock_guard<std::mutex> lk(estimate_mutex_);
        cached_estimate_ = est;
    }

    processing_scan_ = false;
}

// ─── Initial-pose callback ──────────────────────────────────────────
void AmclNode::initialpose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    double x     = msg->pose.pose.position.x;
    double y     = msg->pose.pose.position.y;
    double theta = math_utils::quaternion_to_yaw(msg->pose.pose.orientation);

    RCLCPP_INFO(get_logger(),
                "Re-initialising PF at (%.2f, %.2f, %.2f)", x, y, theta);

    std::lock_guard<std::mutex> lock(pf_mutex_);
    pf_.reinitialize(x, y, theta,
                     get_parameter("initial_cov_xx").as_double(),
                     get_parameter("initial_cov_yy").as_double(),
                     get_parameter("initial_cov_aa").as_double());
}

// ─── Direct pose publish (called from scan_callback) ────────────────
void AmclNode::publish_pose(const PoseEstimate& est,
                            const rclcpp::Time& stamp) {
    auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    pose_msg.header.stamp    = stamp;   // original scan timestamp (§10.6)
    pose_msg.header.frame_id = global_frame_;
    pose_msg.pose.pose.position.x = est.x;
    pose_msg.pose.pose.position.y = est.y;
    pose_msg.pose.pose.position.z = 0.0;
    pose_msg.pose.pose.orientation = math_utils::yaw_to_quaternion(est.theta);

    // Fill 6×6 covariance from 3×3 (rows/cols 0,1,5 in ROS convention).
    auto& cov = pose_msg.pose.covariance;
    std::fill(cov.begin(), cov.end(), 0.0);
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
    // Particle cloud for RViz — only when someone is subscribed.
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
        int step = std::max(1, np / 100);
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
