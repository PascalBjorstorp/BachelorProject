#include "f1tenth_control/nodes/pure_pursuit_node.hpp"
#include <algorithm>
#include <cmath>

namespace f1tenth_control {

PurePursuitNode::PurePursuitNode(const rclcpp::NodeOptions& options)
    : Node("pure_pursuit_node", options)
{
    RCLCPP_INFO(get_logger(), "Initializing Pure Pursuit Node");
    
    // Declare and load parameters
    declareParameters();
    loadParameters();

    // Create controller
    controller_ = std::make_unique<PurePursuit>(config_);
    
    // Load trajectory
    if (!loadTrajectory()) {
        RCLCPP_ERROR(get_logger(), "Failed to load trajectory from: %s", trajectory_file_.c_str());
        RCLCPP_WARN(get_logger(), "Controller will be disabled until trajectory is loaded");
    }
    
    // Setup subscribers
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::SensorDataQoS(),
        std::bind(&PurePursuitNode::odomCallback, this, std::placeholders::_1)
    );

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic_, rclcpp::SensorDataQoS(),
        std::bind(&PurePursuitNode::poseCallback, this, std::placeholders::_1)
    );
    
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/pp_enable", 10,
        std::bind(&PurePursuitNode::enableCallback, this, std::placeholders::_1)
    );
    
    // Local raceline from lateral planner (overrides loaded trajectory when received)
    local_raceline_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/local_raceline", 10,
        std::bind(&PurePursuitNode::localRacelineCallback, this, std::placeholders::_1)
    );
    
    // Setup publishers
    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        "/drive", 10
    );
    
    if (publish_visualization_) {
        viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/pp_viz", 10
        );
    }
    
    // Setup parameter callback
    param_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&PurePursuitNode::parametersCallback, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(get_logger(), "Pure Pursuit Node initialized");
    RCLCPP_INFO(get_logger(), "  Trajectory: %s (%zu points)", 
                trajectory_file_.c_str(), controller_->getTrajectory().size());
    RCLCPP_INFO(get_logger(), "  Lookahead: %.2f - %.2f m (gain: %.2f)",
                config_.min_lookahead, config_.max_lookahead, config_.lookahead_gain);
    RCLCPP_INFO(get_logger(), "  Max speed cap: %.2f m/s", max_speed_);
    RCLCPP_INFO(get_logger(), "  Lookahead adapt: cte_weight=%.2f cte_gain=%.3f curvature_gain=%.3f",
                config_.cte_lookahead_weight, config_.cte_lookahead_gain, config_.curvature_lookahead_gain);
    RCLCPP_INFO(get_logger(), "  Speed adapt: curvature_factor=%.3f floor_ratio=%.2f",
                config_.curvature_speed_factor, config_.curvature_speed_floor_ratio);
    RCLCPP_INFO(get_logger(), "  Speed adapt (CTE): factor=%.3f floor_ratio=%.2f",
                config_.cte_speed_factor, config_.cte_speed_floor_ratio);
    RCLCPP_INFO(get_logger(), "  Speed limits: max_lat_accel=%.2f min_reg_speed=%.2f",
                config_.max_lateral_accel, config_.min_regulated_speed);
    RCLCPP_INFO(get_logger(), "  Command shaping: steer_rate=%.2f accel=%.2f decel=%.2f",
                max_steering_rate_, max_accel_cmd_, max_decel_cmd_);
}

void PurePursuitNode::declareParameters() {
    // Trajectory
    declare_parameter("trajectory_file", "");
    
    // Lookahead - sweep-optimized defaults
    declare_parameter("min_lookahead", 0.48);
    declare_parameter("max_lookahead", 1.20);
    declare_parameter("lookahead_gain", 0.15);
    declare_parameter("max_speed", 5.5);
    declare_parameter("cte_lookahead_weight", 1.0);
    declare_parameter("cte_lookahead_gain", 0.05);
    declare_parameter("curvature_lookahead_gain", 1.34);
    declare_parameter("curvature_speed_factor", 0.10);
    declare_parameter("curvature_speed_floor_ratio", 0.43);
    declare_parameter("cte_speed_factor", 0.10);
    declare_parameter("cte_speed_floor_ratio", 0.50);
    declare_parameter("max_lateral_accel", 7.27);
    declare_parameter("min_regulated_speed", 0.30);
    declare_parameter("curvature_preview_factor", 1.2);
    
    // Corridor-aware width regulation
    declare_parameter("vehicle_half_width", 0.1365);
    declare_parameter("wall_safety_margin", 0.03);
    declare_parameter("corridor_half_width_ref", 0.25);
    declare_parameter("corridor_speed_floor_ratio", 0.20);
    declare_parameter("corridor_lookahead_factor", 2.0);
    
    // Steering
    declare_parameter("max_steering", 0.4189);
    
    // Vehicle
    declare_parameter("wheelbase", 0.324);
    
    // Misc
    declare_parameter("publish_visualization", true);
    declare_parameter("pose_topic", std::string("/ekf_pose"));
    declare_parameter("pose_timeout_s", 0.1);
    declare_parameter("odom_timeout_s", 0.2);
    declare_parameter("max_steering_rate", 2.8);
    declare_parameter("max_accel_cmd", 3.0);
    declare_parameter("max_decel_cmd", 5.0);
}

void PurePursuitNode::loadParameters() {
    trajectory_file_ = get_parameter("trajectory_file").as_string();

    config_.min_lookahead = std::max(0.05, get_parameter("min_lookahead").as_double());
    config_.max_lookahead = std::max(config_.min_lookahead, get_parameter("max_lookahead").as_double());
    config_.lookahead_gain = std::max(0.0, get_parameter("lookahead_gain").as_double());
    max_speed_ = std::max(0.0, get_parameter("max_speed").as_double());
    config_.cte_lookahead_weight = std::max(0.0, get_parameter("cte_lookahead_weight").as_double());
    config_.cte_lookahead_gain = std::max(0.0, get_parameter("cte_lookahead_gain").as_double());
    config_.curvature_lookahead_gain = std::max(0.0, get_parameter("curvature_lookahead_gain").as_double());
    config_.curvature_speed_factor = std::max(0.0, get_parameter("curvature_speed_factor").as_double());
    config_.curvature_speed_floor_ratio = std::clamp(
        get_parameter("curvature_speed_floor_ratio").as_double(), 0.0, 1.0);
    config_.cte_speed_factor = std::max(0.0, get_parameter("cte_speed_factor").as_double());
    config_.cte_speed_floor_ratio = std::clamp(
        get_parameter("cte_speed_floor_ratio").as_double(), 0.0, 1.0);
    config_.max_lateral_accel = std::max(0.5, get_parameter("max_lateral_accel").as_double());
    config_.min_regulated_speed = std::max(0.0, get_parameter("min_regulated_speed").as_double());
    config_.curvature_preview_factor = std::max(1.0, get_parameter("curvature_preview_factor").as_double());
    
    // Corridor-aware width regulation
    config_.vehicle_half_width = std::max(0.01, get_parameter("vehicle_half_width").as_double());
    config_.wall_safety_margin = std::max(0.0, get_parameter("wall_safety_margin").as_double());
    config_.corridor_half_width_ref = std::max(0.01, get_parameter("corridor_half_width_ref").as_double());
    config_.corridor_speed_floor_ratio = std::clamp(
        get_parameter("corridor_speed_floor_ratio").as_double(), 0.0, 1.0);
    config_.corridor_lookahead_factor = std::max(0.0, get_parameter("corridor_lookahead_factor").as_double());

    config_.max_steering = std::max(1e-3, get_parameter("max_steering").as_double());
    config_.wheelbase = std::max(1e-3, get_parameter("wheelbase").as_double());
    
    publish_visualization_ = get_parameter("publish_visualization").as_bool();
    pose_topic_ = get_parameter("pose_topic").as_string();
    pose_timeout_s_ = std::max(0.01, get_parameter("pose_timeout_s").as_double());
    odom_timeout_s_ = std::max(0.01, get_parameter("odom_timeout_s").as_double());
    max_steering_rate_ = std::max(0.1, get_parameter("max_steering_rate").as_double());
    max_accel_cmd_ = std::max(0.1, get_parameter("max_accel_cmd").as_double());
    max_decel_cmd_ = std::max(0.1, get_parameter("max_decel_cmd").as_double());
}

rcl_interfaces::msg::SetParametersResult PurePursuitNode::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;

    PurePursuitConfig candidate = config_;
    double candidate_max_speed = max_speed_;
    double candidate_pose_timeout = pose_timeout_s_;
    double candidate_odom_timeout = odom_timeout_s_;
    double candidate_max_steering_rate = max_steering_rate_;
    double candidate_max_accel_cmd = max_accel_cmd_;
    double candidate_max_decel_cmd = max_decel_cmd_;

    for (const auto& param : parameters) {
        if (param.get_name() == "min_lookahead") {
            candidate.min_lookahead = param.as_double();
        } else if (param.get_name() == "max_lookahead") {
            candidate.max_lookahead = param.as_double();
        } else if (param.get_name() == "lookahead_gain") {
            candidate.lookahead_gain = param.as_double();
        } else if (param.get_name() == "max_speed") {
            candidate_max_speed = param.as_double();
        } else if (param.get_name() == "cte_lookahead_weight") {
            candidate.cte_lookahead_weight = param.as_double();
        } else if (param.get_name() == "cte_lookahead_gain") {
            candidate.cte_lookahead_gain = param.as_double();
        } else if (param.get_name() == "curvature_lookahead_gain") {
            candidate.curvature_lookahead_gain = param.as_double();
        } else if (param.get_name() == "curvature_speed_factor") {
            candidate.curvature_speed_factor = param.as_double();
        } else if (param.get_name() == "curvature_speed_floor_ratio") {
            candidate.curvature_speed_floor_ratio = param.as_double();
        } else if (param.get_name() == "cte_speed_factor") {
            candidate.cte_speed_factor = param.as_double();
        } else if (param.get_name() == "cte_speed_floor_ratio") {
            candidate.cte_speed_floor_ratio = param.as_double();
        } else if (param.get_name() == "max_lateral_accel") {
            candidate.max_lateral_accel = param.as_double();
        } else if (param.get_name() == "min_regulated_speed") {
            candidate.min_regulated_speed = param.as_double();
        } else if (param.get_name() == "curvature_preview_factor") {
            candidate.curvature_preview_factor = param.as_double();
        } else if (param.get_name() == "vehicle_half_width") {
            candidate.vehicle_half_width = param.as_double();
        } else if (param.get_name() == "wall_safety_margin") {
            candidate.wall_safety_margin = param.as_double();
        } else if (param.get_name() == "corridor_half_width_ref") {
            candidate.corridor_half_width_ref = param.as_double();
        } else if (param.get_name() == "corridor_speed_floor_ratio") {
            candidate.corridor_speed_floor_ratio = param.as_double();
        } else if (param.get_name() == "corridor_lookahead_factor") {
            candidate.corridor_lookahead_factor = param.as_double();
        } else if (param.get_name() == "max_steering") {
            candidate.max_steering = param.as_double();
        } else if (param.get_name() == "wheelbase") {
            candidate.wheelbase = param.as_double();
        } else if (param.get_name() == "pose_timeout_s") {
            candidate_pose_timeout = param.as_double();
        } else if (param.get_name() == "odom_timeout_s") {
            candidate_odom_timeout = param.as_double();
        } else if (param.get_name() == "max_steering_rate") {
            candidate_max_steering_rate = param.as_double();
        } else if (param.get_name() == "max_accel_cmd") {
            candidate_max_accel_cmd = param.as_double();
        } else if (param.get_name() == "max_decel_cmd") {
            candidate_max_decel_cmd = param.as_double();
        }
    }

    auto finite = [](double v) { return std::isfinite(v); };
    auto finite_and_nonnegative = [&](double v) { return finite(v) && v >= 0.0; };

    if (!finite(candidate.min_lookahead) || candidate.min_lookahead < 0.05) {
        result.reason = "min_lookahead must be finite and >= 0.05";
        return result;
    }
    if (!finite(candidate.max_lookahead) || candidate.max_lookahead < candidate.min_lookahead) {
        result.reason = "max_lookahead must be finite and >= min_lookahead";
        return result;
    }
    if (!finite_and_nonnegative(candidate.lookahead_gain)) {
        result.reason = "lookahead_gain must be finite and >= 0";
        return result;
    }
    if (!finite_and_nonnegative(candidate_max_speed)) {
        result.reason = "max_speed must be finite and >= 0";
        return result;
    }
    if (!finite_and_nonnegative(candidate.cte_lookahead_weight) ||
        !finite_and_nonnegative(candidate.cte_lookahead_gain) ||
        !finite_and_nonnegative(candidate.curvature_lookahead_gain) ||
        !finite_and_nonnegative(candidate.curvature_speed_factor) ||
        !finite_and_nonnegative(candidate.cte_speed_factor)) {
        result.reason = "lookahead/speed gains must be finite and >= 0";
        return result;
    }
    if (!finite(candidate.curvature_speed_floor_ratio) ||
        candidate.curvature_speed_floor_ratio < 0.0 ||
        candidate.curvature_speed_floor_ratio > 1.0) {
        result.reason = "curvature_speed_floor_ratio must be in [0,1]";
        return result;
    }
    if (!finite(candidate.cte_speed_floor_ratio) ||
        candidate.cte_speed_floor_ratio < 0.0 ||
        candidate.cte_speed_floor_ratio > 1.0) {
        result.reason = "cte_speed_floor_ratio must be in [0,1]";
        return result;
    }
    if (!finite(candidate.max_lateral_accel) || candidate.max_lateral_accel <= 0.1) {
        result.reason = "max_lateral_accel must be finite and > 0.1";
        return result;
    }
    if (!finite_and_nonnegative(candidate.min_regulated_speed)) {
        result.reason = "min_regulated_speed must be finite and >= 0";
        return result;
    }
    if (!finite(candidate.curvature_preview_factor) || candidate.curvature_preview_factor < 1.0) {
        result.reason = "curvature_preview_factor must be finite and >= 1.0";
        return result;
    }
    // Corridor-aware width regulation validation
    if (!finite(candidate.vehicle_half_width) || candidate.vehicle_half_width <= 0.01) {
        result.reason = "vehicle_half_width must be finite and > 0.01";
        return result;
    }
    if (!finite_and_nonnegative(candidate.wall_safety_margin)) {
        result.reason = "wall_safety_margin must be finite and >= 0";
        return result;
    }
    if (!finite(candidate.corridor_half_width_ref) || candidate.corridor_half_width_ref <= 0.01) {
        result.reason = "corridor_half_width_ref must be finite and > 0.01";
        return result;
    }
    if (!finite(candidate.corridor_speed_floor_ratio) ||
        candidate.corridor_speed_floor_ratio < 0.0 ||
        candidate.corridor_speed_floor_ratio > 1.0) {
        result.reason = "corridor_speed_floor_ratio must be in [0,1]";
        return result;
    }
    if (!finite_and_nonnegative(candidate.corridor_lookahead_factor)) {
        result.reason = "corridor_lookahead_factor must be finite and >= 0";
        return result;
    }
    if (!finite(candidate.max_steering) || candidate.max_steering <= 0.0) {
        result.reason = "max_steering must be finite and > 0";
        return result;
    }
    if (!finite(candidate.wheelbase) || candidate.wheelbase <= 0.0) {
        result.reason = "wheelbase must be finite and > 0";
        return result;
    }
    if (!finite(candidate_pose_timeout) || candidate_pose_timeout <= 0.0) {
        result.reason = "pose_timeout_s must be finite and > 0";
        return result;
    }
    if (!finite(candidate_odom_timeout) || candidate_odom_timeout <= 0.0) {
        result.reason = "odom_timeout_s must be finite and > 0";
        return result;
    }
    if (!finite(candidate_max_steering_rate) || candidate_max_steering_rate <= 0.0) {
        result.reason = "max_steering_rate must be finite and > 0";
        return result;
    }
    if (!finite(candidate_max_accel_cmd) || candidate_max_accel_cmd <= 0.0) {
        result.reason = "max_accel_cmd must be finite and > 0";
        return result;
    }
    if (!finite(candidate_max_decel_cmd) || candidate_max_decel_cmd <= 0.0) {
        result.reason = "max_decel_cmd must be finite and > 0";
        return result;
    }

    candidate.curvature_speed_floor_ratio = std::clamp(candidate.curvature_speed_floor_ratio, 0.0, 1.0);
    candidate.cte_speed_floor_ratio = std::clamp(candidate.cte_speed_floor_ratio, 0.0, 1.0);
    candidate.corridor_speed_floor_ratio = std::clamp(candidate.corridor_speed_floor_ratio, 0.0, 1.0);

    {
        std::scoped_lock lock(state_mutex_, controller_mutex_);
        config_ = candidate;
        max_speed_ = candidate_max_speed;
        pose_timeout_s_ = candidate_pose_timeout;
        odom_timeout_s_ = candidate_odom_timeout;
        max_steering_rate_ = candidate_max_steering_rate;
        max_accel_cmd_ = candidate_max_accel_cmd;
        max_decel_cmd_ = candidate_max_decel_cmd;
        if (controller_) {
            controller_->setConfig(config_);
        }
    }

    result.successful = true;
    return result;
}

bool PurePursuitNode::loadTrajectory() {
    if (trajectory_file_.empty()) {
        return false;
    }

    bool ok = false;
    {
        std::lock_guard<std::mutex> lock(controller_mutex_);
        ok = controller_->loadTrajectory(trajectory_file_);
    }

    if (ok) {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            trajectory_loaded_ = true;
        }
        size_t count = 0;
        double len = 0.0;
        {
            std::lock_guard<std::mutex> lock(controller_mutex_);
            count = controller_->getTrajectory().size();
            len = controller_->getTrajectoryLength();
        }
        RCLCPP_INFO(get_logger(), "Loaded trajectory with %zu waypoints (%.1f m)",
                    count, len);
        return true;
    }
    
    return false;
}

void PurePursuitNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state_.velocity = msg->twist.twist.linear.x;
        current_state_.angular_velocity = msg->twist.twist.angular.z;
        odom_received_ = true;
        last_odom_time_ = now();
    }
}

void PurePursuitNode::poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state_.pose.x = msg->pose.pose.position.x;
        current_state_.pose.y = msg->pose.pose.position.y;
        const double qx = msg->pose.pose.orientation.x;
        const double qy = msg->pose.pose.orientation.y;
        const double qz = msg->pose.pose.orientation.z;
        const double qw = msg->pose.pose.orientation.w;
        current_state_.pose.theta = std::atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz));
        pose_received_ = true;
        last_pose_time_ = now();
    }

    // Run control on every pose update (typically /ekf_pose).
    controlLoop();
}

void PurePursuitNode::enableCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    bool enabled = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        enabled_ = msg->data;
        enabled = enabled_;
        if (enabled_) {
            soft_start_initialized_ = false;
            cmd_history_initialized_ = false;
        }
    }

    if (enabled) {
        RCLCPP_INFO(get_logger(), "Pure Pursuit ENABLED");
    } else {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            cmd_history_initialized_ = false;
            last_cmd_speed_ = 0.0;
            last_cmd_steering_ = 0.0;
        }
        RCLCPP_INFO(get_logger(), "Pure Pursuit DISABLED");
        // Stop the car
        publishDriveCommand(0.0, 0.0);
    }
}

void PurePursuitNode::localRacelineCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.size() < 3) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Ignoring /local_raceline with too few poses: %zu",
                             msg->poses.size());
        return;
    }

    std::vector<TrajectoryPoint> new_traj;
    new_traj.reserve(msg->poses.size());

    double cumulative_s = 0.0;
    for (size_t i = 0; i < msg->poses.size(); ++i) {
        const auto& pose = msg->poses[i];
        if (!std::isfinite(pose.pose.position.x) ||
            !std::isfinite(pose.pose.position.y) ||
            !std::isfinite(pose.pose.position.z) ||
            !std::isfinite(pose.pose.orientation.x) ||
            !std::isfinite(pose.pose.orientation.y) ||
            !std::isfinite(pose.pose.orientation.z) ||
            !std::isfinite(pose.pose.orientation.w)) {
            continue;
        }

        TrajectoryPoint tp;
        tp.x = pose.pose.position.x;
        tp.y = pose.pose.position.y;
        // Velocity encoded in z by the lateral planner
        tp.velocity = std::max(0.0, pose.pose.position.z);
        // Heading from quaternion (yaw only)
        double siny = 2.0 * (pose.pose.orientation.w * pose.pose.orientation.z +
                              pose.pose.orientation.x * pose.pose.orientation.y);
        double cosy = 1.0 - 2.0 * (pose.pose.orientation.y * pose.pose.orientation.y +
                                     pose.pose.orientation.z * pose.pose.orientation.z);
        tp.heading = std::atan2(siny, cosy);

        // Compute arc length from consecutive points
        if (i > 0) {
            double dx = tp.x - new_traj.back().x;
            double dy = tp.y - new_traj.back().y;
            cumulative_s += std::sqrt(dx * dx + dy * dy);
        }
        tp.arc_length = cumulative_s;

        // Curvature from finite differences (computed after all points added)
        tp.curvature = 0.0;
        new_traj.push_back(tp);
    }

    if (new_traj.size() < 3) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Ignoring /local_raceline after filtering; only %zu valid points",
                             new_traj.size());
        return;
    }

    // Compute curvature from heading differences
    for (size_t i = 1; i + 1 < new_traj.size(); ++i) {
        double ds = new_traj[i + 1].arc_length - new_traj[i - 1].arc_length;
        if (ds > 1e-6) {
            double dtheta = new_traj[i + 1].heading - new_traj[i - 1].heading;
            // Normalize to [-pi, pi]
            while (dtheta > M_PI) dtheta -= 2.0 * M_PI;
            while (dtheta < -M_PI) dtheta += 2.0 * M_PI;
            new_traj[i].curvature = dtheta / ds;
        }
    }

    {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        std::lock_guard<std::mutex> lock(controller_mutex_);
        controller_->setTrajectory(new_traj);
        trajectory_loaded_ = true;
    }
}

void PurePursuitNode::controlLoop() {
    bool enabled = false;
    bool trajectory_loaded = false;
    bool pose_received = false;
    bool odom_received = false;
    rclcpp::Time last_pose_time;
    rclcpp::Time last_odom_time;
    double pose_timeout_s = 0.1;
    double odom_timeout_s = 0.2;
    double max_speed = 0.0;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        enabled = enabled_;
        trajectory_loaded = trajectory_loaded_;
        pose_received = pose_received_;
        odom_received = odom_received_;
        last_pose_time = last_pose_time_;
        last_odom_time = last_odom_time_;
        pose_timeout_s = pose_timeout_s_;
        odom_timeout_s = odom_timeout_s_;
        max_speed = max_speed_;
    }

    if (!enabled || !trajectory_loaded) {
        publishDriveCommand(0.0, 0.0);
        return;
    }

    if (!pose_received) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "No pose received on %s yet", pose_topic_.c_str());
        publishDriveCommand(0.0, 0.0);
        return;
    }

    if (!odom_received) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "No odometry received yet; issuing stop for fail-safe");
        publishDriveCommand(0.0, 0.0);
        return;
    }

    const double pose_age = (now() - last_pose_time).seconds();
    if (pose_age > pose_timeout_s) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Pose timeout %.3fs > %.3fs; issuing stop for fail-safe",
                             pose_age, pose_timeout_s);
        publishDriveCommand(0.0, 0.0);
        return;
    }

    const double odom_age = (now() - last_odom_time).seconds();
    if (odom_age > odom_timeout_s) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Odom timeout %.3fs > %.3fs; issuing stop for fail-safe",
                             odom_age, odom_timeout_s);
        publishDriveCommand(0.0, 0.0);
        return;
    }

    VehicleState state;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state = current_state_;
    }
    
    // Compute control (protected against concurrent trajectory/config updates)
    PurePursuitOutput output;
    {
        std::lock_guard<std::mutex> lock(controller_mutex_);
        if (!controller_ || !controller_->hasTrajectory()) {
            publishDriveCommand(0.0, 0.0);
            return;
        }
        output = controller_->compute(state);
    }
    
    if (output.valid) {
        // Soft start: cap speed to 1.0 m/s for the first 2 seconds
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!soft_start_initialized_) {
                soft_start_time_ = now();
                soft_start_initialized_ = true;
                RCLCPP_INFO(get_logger(), "Soft start: capping speed to 1.0 m/s for 2 seconds");
            }
        }
        double elapsed = 0.0;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            elapsed = (now() - soft_start_time_).seconds();
        }
        if (elapsed < 2.0) {
            output.target_speed = std::min(output.target_speed, 1.0);
        }

        output.target_speed = std::clamp(output.target_speed, 0.0, max_speed);

        const rclcpp::Time now_t = now();
        double dt_cmd = 0.01;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!cmd_history_initialized_) {
                last_cmd_time_ = now_t;
                last_cmd_steering_ = output.steering_angle;
                last_cmd_speed_ = std::min(output.target_speed, max_speed);
                cmd_history_initialized_ = true;
            }
            dt_cmd = std::max(1e-3, (now_t - last_cmd_time_).seconds());
        }

        const double max_delta_steer = max_steering_rate_ * dt_cmd;
        double cmd_steer = output.steering_angle;
        double cmd_speed = output.target_speed;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            const double steer_err = cmd_steer - last_cmd_steering_;
            const double steer_step = std::clamp(steer_err, -max_delta_steer, max_delta_steer);
            cmd_steer = last_cmd_steering_ + steer_step;

            const double max_speed_step_up = max_accel_cmd_ * dt_cmd;
            const double max_speed_step_down = max_decel_cmd_ * dt_cmd;
            const double speed_err = cmd_speed - last_cmd_speed_;
            if (speed_err >= 0.0) {
                cmd_speed = last_cmd_speed_ + std::min(speed_err, max_speed_step_up);
            } else {
                cmd_speed = last_cmd_speed_ + std::max(speed_err, -max_speed_step_down);
            }

            cmd_speed = std::clamp(cmd_speed, 0.0, max_speed);
            last_cmd_steering_ = cmd_steer;
            last_cmd_speed_ = cmd_speed;
            last_cmd_time_ = now_t;
        }
        
        publishDriveCommand(cmd_steer, cmd_speed);
        
        if (publish_visualization_) {
            publishLookaheadMarker(output);
        }
    } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                            "Invalid Pure Pursuit output");
        publishDriveCommand(0.0, 0.0);
    }
}

void PurePursuitNode::publishDriveCommand(double steering, double speed) {
    if (!std::isfinite(steering) || !std::isfinite(speed)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Non-finite command detected; publishing safe stop");
        steering = 0.0;
        speed = 0.0;
    }

    auto msg = ackermann_msgs::msg::AckermannDriveStamped();
    msg.header.stamp = now();
    msg.header.frame_id = "base_link";
    msg.drive.steering_angle = steering;
    msg.drive.speed = speed;
    drive_pub_->publish(msg);
}

void PurePursuitNode::publishLookaheadMarker(const PurePursuitOutput& output) {
    visualization_msgs::msg::MarkerArray markers;
    
    visualization_msgs::msg::Marker target_marker;
    target_marker.header.stamp = now();
    target_marker.header.frame_id = "map";
    target_marker.ns = "pure_pursuit";
    target_marker.id = 0;
    target_marker.type = visualization_msgs::msg::Marker::SPHERE;
    target_marker.action = visualization_msgs::msg::Marker::ADD;
    target_marker.pose.position.x = output.target_point.x;
    target_marker.pose.position.y = output.target_point.y;
    target_marker.pose.position.z = 0.1;
    target_marker.scale.x = 0.3;
    target_marker.scale.y = 0.3;
    target_marker.scale.z = 0.3;
    target_marker.color.r = 0.0;
    target_marker.color.g = 1.0;
    target_marker.color.b = 0.0;
    target_marker.color.a = 0.8;
    markers.markers.push_back(target_marker);
    
    viz_pub_->publish(markers);
}



}  // namespace f1tenth_control

// Component registration for composable nodes
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(f1tenth_control::PurePursuitNode)

// Main entry point for standalone executable
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<f1tenth_control::PurePursuitNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
