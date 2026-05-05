/**
 * @file state_publisher.cpp
 * @brief Publish vehicle state and streamed MPC references from Jetson.
 * @details Builds a KD-tree from trajectory waypoints, receives pose/odometry,
 *          and publishes Q16.16 `MpcState` packets for the Kria receiver.
 * @dependencies rclcpp, nav_msgs, geometry_msgs, std_msgs, f1tenth_msgs,
 *               state_publisher/kdtree.hpp, mpc_fpga_constants.h
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <f1tenth_msgs/msg/mpc_state.hpp>

#include "mpc_fpga_constants.h"
#include "kdtree.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <limits>
#include <cmath>
#include <chrono>

namespace f1tenth_communication {

/*===========================================================================
 * State Publisher Node
 *===========================================================================*/

/**
 * @brief ROS2 node that publishes current vehicle state and streamed MPC references.
 * @details Loads trajectory waypoints into a KD-tree, tracks pose and odometry,
 *          and publishes fixed-point `MpcState` packets for the FPGA receiver.
 */
class StatePublisherNode : public rclcpp::Node {
public:
    /**
     * @brief Construct and initialize the state publisher node.
     * @return None.
     */
    StatePublisherNode() : Node("state_publisher") {
        // Parameters
        this->declare_parameter("trajectory_file", "");
        this->declare_parameter("odom_topic", "/ego_racecar/odom");
        this->declare_parameter("pose_topic", "/ekf_pose");
        this->declare_parameter("output_topic", "/mpc_state");
        this->declare_parameter("servo_topic", "/sensors/servo_position_command");
        
        // Trajectory source
        std::string trajectory_file = this->get_parameter("trajectory_file").as_string();

        // Topics
        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        std::string pose_topic = this->get_parameter("pose_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        std::string servo_topic = this->get_parameter("servo_topic").as_string();

        // Check that trajectory file isn't empty before proceeding.
        if (trajectory_file.empty()) {
            throw std::runtime_error("state_publisher: no trajectory file specified");
        }

        if (!load_trajectory(trajectory_file)) {
            throw std::runtime_error("state_publisher: failed to load trajectory: " + trajectory_file);
        }
        
        RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from %s",
               kdtree_.size(), trajectory_file.c_str());
        RCLCPP_INFO(this->get_logger(),
            "Streaming mode: horizon-only (length=%zu)",
            static_cast<size_t>(MPC_FPGA_HORIZON_STEPS));
        
        // Best Effort + volatile minimizes control latency under packet loss.
        auto qos = rclcpp::QoS(1)
            .best_effort()
            .durability_volatile();
        pub_ = this->create_publisher<f1tenth_msgs::msg::MpcState>(output_topic, qos);
        
        // Subscribe to odometry with Best Effort QoS (velocity/yaw-rate cache only).
        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, qos,
            std::bind(&StatePublisherNode::odom_callback, this, std::placeholders::_1));

        // Subscribe to EKF pose: publish one state packet for each incoming pose.
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            pose_topic, qos,
            std::bind(&StatePublisherNode::pose_callback, this, std::placeholders::_1));
        
        // Subscribe to servo position feedback (for actual steering angle)
        if (!servo_topic.empty()) {
            servo_sub_ = this->create_subscription<std_msgs::msg::Float64>(
                servo_topic, qos,
                [this](const std_msgs::msg::Float64::SharedPtr msg) {
                    // Recover steering angle from calibrated VESC servo mapping.
                    double corrected = (msg->data - MPC_FPGA_SERVO_OFFSET) / MPC_FPGA_SERVO_GAIN;
                    double abs_corr = std::abs(corrected);
                    if (MPC_FPGA_STEER_CORRECTION_C2 != 0.0) {
                        double disc = MPC_FPGA_STEER_CORRECTION_C1 * MPC_FPGA_STEER_CORRECTION_C1
                                    - 4.0 * MPC_FPGA_STEER_CORRECTION_C2 * (MPC_FPGA_STEER_CORRECTION_C0 - abs_corr);
                        if (disc >= 0.0) {
                            double t = (-MPC_FPGA_STEER_CORRECTION_C1 + std::sqrt(disc))
                                     / (2.0 * MPC_FPGA_STEER_CORRECTION_C2);
                            current_steering_angle_ = std::copysign(t, corrected);
                        } else {
                            current_steering_angle_ = corrected;
                        }
                    } else {
                        current_steering_angle_ = corrected;
                    }
                    has_servo_feedback_ = true;
                });
            RCLCPP_INFO(this->get_logger(), "Subscribing to servo feedback: %s", servo_topic.c_str());
        }
        
        // Odom timeout watchdog: warn if no odom received for 500ms
        odom_watchdog_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(MPC_FPGA_PUBLISHER_ODOM_WATCHDOG_MS),
            [this]() {
                if (!odom_received_) return;  // Haven't received first message yet
                auto elapsed_ms = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - last_odom_time_).count();
                if (elapsed_ms > static_cast<double>(MPC_FPGA_PUBLISHER_ODOM_WATCHDOG_MS)) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "No odom received for %.0f ms — localization may be stalled", elapsed_ms);
                }
            });

        startup_diag_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                if (published_count_ > 0) {
                    startup_diag_timer_->cancel();
                    return;
                }
                if (!pose_received_) {
                    RCLCPP_WARN(this->get_logger(),
                        "State publisher waiting: no pose sample received on pose_topic yet");
                }
                if (!has_odom_dynamics_) {
                    RCLCPP_WARN(this->get_logger(),
                        "State publisher waiting: no valid odom dynamics sample yet");
                }
            });

        RCLCPP_INFO(this->get_logger(),
                "State publisher ready (Best Effort QoS). Odom cache: %s, pose trigger: %s, publishing: %s",
                odom_topic.c_str(), pose_topic.c_str(), output_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Forward waypoint lookahead: %d", MPC_FPGA_PUBLISHER_FORWARD_LOOKAHEAD);
    }
    
private:
    // --- ROS interfaces ------------------------------------------------------
    KDTree kdtree_;
    std::vector<Waypoint> trajectory_;
    rclcpp::Publisher<f1tenth_msgs::msg::MpcState>::SharedPtr pub_;                             // Publisher for fixed-point MPC state packets.
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;                              // Subscription for odometry messages to extract velocity and yaw rate.
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;   // Subscription for EKF pose messages to trigger state packet publication.
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr servo_sub_;                         // Subscription for servo position feedback to compute actual steering angle.               
    
    // --- Runtime state -------------------------------------------------------
    double current_steering_angle_ = 0.0;                           // Steering angle [rad] 
    bool has_servo_feedback_ = false;                               // Flag indicating if servo feedback has been received at least once.

    // --- Watchdog state -----------------------------------------------------
    rclcpp::TimerBase::SharedPtr odom_watchdog_timer_;                                          // Timer to check for odometry timeouts and emit warnings.
    rclcpp::TimerBase::SharedPtr startup_diag_timer_;                                            // Timer to report startup dependencies until first publish.
    std::chrono::steady_clock::time_point last_odom_time_ = std::chrono::steady_clock::now();   // Timestamp of the last received odometry message, used for watchdog timeout checks.
    bool odom_received_ = false;                                                                // Flag indicating if at least one odometry message has been received, used to suppress watchdog warnings until first message arrives.              

    // --- Cached dynamics from odometry -------------------------------------
    double latest_velocity_ = 0.0;      // Latest longitudinal velocity in m/s extracted from odometry messages, used for computing steering angle when servo feedback is unavailable.
    double latest_vy_ = 0.0;            // Latest lateral velocity in m/s extracted from odometry messages, included in state packets for MPC tracking error computation.
    double latest_omega_ = 0.0;         // Latest yaw rate in rad/s extracted from odometry messages, used for computing steering angle when servo feedback is unavailable and included in state packets for MPC tracking error computation.
    bool has_odom_dynamics_ = false;    // Flag indicating if valid odometry dynamics have been received at least once, used to determine if velocity and yaw rate can be used for steering angle computation when servo feedback is unavailable.
    bool pose_received_ = false;        // Flag indicating if at least one pose message has been received.
    uint64_t published_count_ = 0;      // Number of MpcState messages published.
    double total_length_m_ = 0.0;       // Total wrapped trajectory length for arc-length sampling.

    // --- Odometry processing helpers ----------------------------------------
    /**
     * @brief Validate incoming odometry values before they are used.
     * @param msg Incoming odometry message.
     * @param x Extracted x position from the message.
     * @param y Extracted y position from the message.
     * @return true when message values are finite and within sanity limits.
     */
    bool validate_odom_message(const nav_msgs::msg::Odometry::SharedPtr& msg,
                               double x,
                               double y) {
        const double qx = msg->pose.pose.orientation.x;
        const double qy = msg->pose.pose.orientation.y;
        const double qz = msg->pose.pose.orientation.z;
        const double qw = msg->pose.pose.orientation.w;
        const double vx = msg->twist.twist.linear.x;
        const double vy = msg->twist.twist.linear.y;
        const double wz = msg->twist.twist.angular.z;

        // Basic sanity checks to catch NaNs/Infs and unreasonable values before they propagate to MPC.
        auto ok = [](double v) { return std::isfinite(v); };
        if (!ok(x) || !ok(y) || !ok(qx) || !ok(qy) || !ok(qz) || !ok(qw) ||
            !ok(vx) || !ok(vy) || !ok(wz)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Dropping odom: NaN/Inf detected in incoming message");
            return false;
        }

        // Additional sanity limits to catch outliers (e.g. from EKF divergence) that could destabilize MPC.
        constexpr double POS_LIMIT = 50.0;   // ±50 m
        constexpr double VEL_LIMIT = 20.0;    // ±20 m/s
        if (std::abs(x) > POS_LIMIT || std::abs(y) > POS_LIMIT) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Dropping odom: position out of range (%.1f, %.1f)", x, y);
            return false;
        }
        if (std::abs(vx) > VEL_LIMIT || std::abs(vy) > VEL_LIMIT) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Dropping odom: velocity out of range (vx=%.1f, vy=%.1f)", vx, vy);
            return false;
        }

        return true;
    }

    /**
     * @brief Convert quaternion orientation to planar yaw.
     * @param qx Quaternion x component.
     * @param qy Quaternion y component.
     * @param qz Quaternion z component.
     * @param qw Quaternion w component.
     * @return Yaw angle in radians.
     */
    static double quaternion_to_yaw(double qx, double qy, double qz, double qw) {
        return std::atan2(2.0 * (qw * qz + qx * qy),
                          1.0 - 2.0 * (qy * qy + qz * qz));
    }

    /**
     * @brief Compute steering angle from servo feedback or bicycle fallback.
     * @param velocity Longitudinal velocity in m/s.
     * @param omega Yaw rate in rad/s.
     * @return Steering angle in radians.
     */
    double compute_steering_angle(double velocity, double omega) const {
        if (!has_servo_feedback_ && std::abs(velocity) > 0.1) {
            return std::atan2(static_cast<double>(MPC_FPGA_WHEELBASE_M) * omega, velocity);
        }
        return current_steering_angle_;
    }

    /**
     * @brief Apply heading-aware forward bias to nearest waypoint selection.
     * @param nearest_idx KD-tree nearest waypoint index.
     * @param x Current vehicle x position.
     * @param y Current vehicle y position.
     * @param theta Current vehicle heading in radians.
     * @return Forward-biased waypoint index for horizon generation.
     */
    size_t apply_forward_bias(size_t nearest_idx, double x, double y, double theta) const {
        const double cos_theta = std::cos(theta);
        const double sin_theta = std::sin(theta);
        const size_t N = kdtree_.size();
        size_t best_idx = nearest_idx;
        double best_dist = std::numeric_limits<double>::max();

        // Search forward along the trajectory from the nearest index,
        // checking up to MPC_FPGA_PUBLISHER_FORWARD_LOOKAHEAD waypoints.
        for (int i = 0; i <= MPC_FPGA_PUBLISHER_FORWARD_LOOKAHEAD; ++i) {
            size_t check_idx = (nearest_idx + static_cast<size_t>(i)) % N;
            const auto& wp = kdtree_.get_waypoint(check_idx);
            double dx_wp = wp.x - x;
            double dy_wp = wp.y - y;
            // Dot product with heading: positive means ahead.
            double ahead = dx_wp * cos_theta + dy_wp * sin_theta;
            if (ahead >= 0.0) {
                double dist = dx_wp * dx_wp + dy_wp * dy_wp;
                if (dist < best_dist) {
                    best_dist = dist;
                    best_idx = check_idx;
                }
            }
        }

        return best_idx;
    }

    /**
     * @brief Convert floating-point value to Q16.16 fixed-point representation.
     * @param v Floating-point input value.
     * @return Q16.16 fixed-point integer representation.
     */
    static int32_t to_fixed_q16(double v) {
        constexpr double FP_SCALE = MPC_FPGA_Q16_SCALE_F64;
        if (!std::isfinite(v)) {
            return 0;
        }
        return static_cast<int32_t>(v >= 0.0 ? v * FP_SCALE + 0.5 : v * FP_SCALE - 0.5);
    }

    static double normalize_angle(double angle) {
        while (angle > M_PI) {
            angle -= 2.0 * M_PI;
        }
        while (angle < -M_PI) {
            angle += 2.0 * M_PI;
        }
        return angle;
    }

    static double lerp_angle(double a0, double a1, double t) {
        return normalize_angle(a0 + normalize_angle(a1 - a0) * t);
    }

    Waypoint sample_by_arc_length(double s_query) const {
        if (trajectory_.empty()) {
            return Waypoint{};
        }
        if (trajectory_.size() == 1 || total_length_m_ <= 1e-6) {
            return trajectory_.front();
        }

        const double s0 = trajectory_.front().s;
        double s = s_query;
        while (s < s0) {
            s += total_length_m_;
        }
        while (s >= s0 + total_length_m_) {
            s -= total_length_m_;
        }

        auto it = std::lower_bound(
            trajectory_.begin(), trajectory_.end(), s,
            [](const Waypoint& wp, double target_s) { return wp.s < target_s; });

        if (it == trajectory_.begin()) {
            return *it;
        }

        Waypoint out{};
        if (it == trajectory_.end()) {
            const auto& w0 = trajectory_.back();
            const auto& w1 = trajectory_.front();
            const double s1 = w1.s + total_length_m_;
            const double denom = s1 - w0.s;
            const double t = (denom > 1e-9) ? ((s - w0.s) / denom) : 0.0;
            out.s = s;
            out.x = w0.x + (w1.x - w0.x) * t;
            out.y = w0.y + (w1.y - w0.y) * t;
            out.psi = lerp_angle(w0.psi, w1.psi, t);
            out.kappa = w0.kappa + (w1.kappa - w0.kappa) * t;
            out.vx = w0.vx + (w1.vx - w0.vx) * t;
            out.left_bound = w0.left_bound + (w1.left_bound - w0.left_bound) * t;
            out.right_bound = w0.right_bound + (w1.right_bound - w0.right_bound) * t;
            return out;
        }

        const auto& w1 = *it;
        const auto& w0 = *(it - 1);
        const double denom = w1.s - w0.s;
        const double t = (denom > 1e-9) ? ((s - w0.s) / denom) : 0.0;
        out.s = s;
        out.x = w0.x + (w1.x - w0.x) * t;
        out.y = w0.y + (w1.y - w0.y) * t;
        out.psi = lerp_angle(w0.psi, w1.psi, t);
        out.kappa = w0.kappa + (w1.kappa - w0.kappa) * t;
        out.vx = w0.vx + (w1.vx - w0.vx) * t;
        out.left_bound = w0.left_bound + (w1.left_bound - w0.left_bound) * t;
        out.right_bound = w0.right_bound + (w1.right_bound - w0.right_bound) * t;
        return out;
    }

    /**
     * @brief Populate streamed horizon reference fields in an outgoing message in place.
     * @param mpc_state Output message to populate.
     * @param waypoint_idx Starting waypoint index for the horizon window.
     * @return None.
     */
    void fill_horizon_references(f1tenth_msgs::msg::MpcState& mpc_state,
                                 size_t waypoint_idx) const {
        const size_t N = kdtree_.size();
        const size_t stream_count = std::min(static_cast<size_t>(MPC_FPGA_HORIZON_STEPS), N);
        mpc_state.horizon_length = static_cast<uint32_t>(stream_count);

        const auto& wp_seed = kdtree_.get_waypoint(waypoint_idx % N);
        const double v_ref_base = std::clamp(
            wp_seed.vx,
            static_cast<double>(MPC_FPGA_MIN_VEL_MPS),
            static_cast<double>(MPC_FPGA_MAX_VEL_MPS));
        const Waypoint wp0 = sample_by_arc_length(wp_seed.s);
        const Waypoint wp1 = sample_by_arc_length(
            wp_seed.s + v_ref_base * static_cast<double>(MPC_FPGA_PREDICTION_DT_S));
        mpc_state.ref_x_0_fp = to_fixed_q16(wp0.x);
        mpc_state.ref_y_0_fp = to_fixed_q16(wp0.y);
        mpc_state.ref_psi_0_fp = to_fixed_q16(wp0.psi);
        mpc_state.ref_x_1_fp = to_fixed_q16(wp1.x);
        mpc_state.ref_y_1_fp = to_fixed_q16(wp1.y);
        mpc_state.ref_psi_1_fp = to_fixed_q16(wp1.psi);

        // Resize only the arrays that go to FPGA
        mpc_state.ref_ey_fp.resize(stream_count);
        mpc_state.ref_vx_fp.resize(stream_count);
        mpc_state.ref_kappa_fp.resize(stream_count);
        mpc_state.ref_left_bound_fp.resize(stream_count);
        mpc_state.ref_right_bound_fp.resize(stream_count);

        // Advance horizon by iteratively sampling using each waypoint's speed
        double target_s = wp_seed.s;
        for (size_t i = 0; i < stream_count; ++i) {
            const Waypoint wp = sample_by_arc_length(target_s);
            mpc_state.ref_ey_fp[i] = to_fixed_q16(0.0);
            mpc_state.ref_vx_fp[i] = to_fixed_q16(wp.vx);
            mpc_state.ref_kappa_fp[i] = to_fixed_q16(wp.kappa);
            mpc_state.ref_left_bound_fp[i] = to_fixed_q16(wp.left_bound);
            mpc_state.ref_right_bound_fp[i] = to_fixed_q16(wp.right_bound);
            target_s += wp.vx * static_cast<double>(MPC_FPGA_PREDICTION_DT_S);
        }
    }

    // --- Trajectory I/O ------------------------------------------------------
    
    /**
     * @brief Load trajectory points from CSV and build KD-tree index.
     * @param filepath Absolute or relative path to trajectory CSV file.
     * @return true when trajectory was loaded and indexed successfully.
     */
    bool load_trajectory(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            return false;
        }
        
        // Expected CSV format (with header):
        // s,x,y,psi,kappa,vx,ax,left_bound,right_bound
        std::vector<Waypoint> waypoints;
        std::string line;
        size_t csv_line_number = 1;
        
        // Skip header
        std::getline(file, line);
        
        while (std::getline(file, line)) {
            ++csv_line_number;
            std::stringstream ss(line);
            std::string token;
            Waypoint wp;
            
            std::getline(ss, token, ','); wp.s = std::stod(token);
            std::getline(ss, token, ','); wp.x = std::stod(token);
            std::getline(ss, token, ','); wp.y = std::stod(token);
            std::getline(ss, token, ','); wp.psi = std::stod(token);
            std::getline(ss, token, ','); wp.kappa = std::stod(token);
            std::getline(ss, token, ',');
            wp.vx = std::clamp(
                std::stod(token),
                static_cast<double>(MPC_FPGA_MIN_VEL_MPS),
                static_cast<double>(MPC_FPGA_MAX_VEL_MPS));
            std::getline(ss, token, ',');
            if (!std::getline(ss, token, ',') || token.empty()) {
                RCLCPP_ERROR(this->get_logger(),
                    "Trajectory CSV missing required left_bound at line %zu", csv_line_number);
                return false;
            }
            wp.left_bound = std::stod(token);
            if (!std::getline(ss, token, ',') || token.empty()) {
                RCLCPP_ERROR(this->get_logger(),
                    "Trajectory CSV missing required right_bound at line %zu", csv_line_number);
                return false;
            }
            wp.right_bound = std::stod(token);
            
            waypoints.push_back(wp);
        }
        
        if (waypoints.empty()) {
            return false;
        }
        
        trajectory_ = waypoints;
        kdtree_.build(trajectory_);
        total_length_m_ = 0.0;
        if (trajectory_.size() > 1 && trajectory_.back().s > trajectory_.front().s) {
            total_length_m_ = trajectory_.back().s - trajectory_.front().s;
        }

        return true;
    }
    
    /**
     * @brief Cache latest odometry dynamics used by pose-triggered publishing.
     * @param msg Incoming odometry message.
     * @return None.
     */
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_odom_time_ = std::chrono::steady_clock::now();
        odom_received_ = true;

        // 1) Extract inputs
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        // 2) Validate inputs
        if (!validate_odom_message(msg, x, y)) {
            return;
        }
        
        // Cache dynamics for use by pose callback.
        latest_velocity_ = msg->twist.twist.linear.x;
        latest_vy_ = msg->twist.twist.linear.y;
        latest_omega_ = msg->twist.twist.angular.z;
        has_odom_dynamics_ = true;
    }

    /**
     * @brief Build and publish one MpcState packet for each incoming pose.
     * @param msg Incoming pose message in map frame.
     * @return None.
     */
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        pose_received_ = true;

        if (pub_->get_subscription_count() == 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Skipping publish: waiting for /mpc_state subscriber discovery");
            return;
        }

        const double x = msg->pose.pose.position.x;
        const double y = msg->pose.pose.position.y;
        const double qx = msg->pose.pose.orientation.x;
        const double qy = msg->pose.pose.orientation.y;
        const double qz = msg->pose.pose.orientation.z;
        const double qw = msg->pose.pose.orientation.w;

        // Validate pose inputs before processing to avoid publishing bad state to MPC.
        auto ok = [](double v) { return std::isfinite(v); };
        if (!ok(x) || !ok(y) || !ok(qx) || !ok(qy) || !ok(qz) || !ok(qw)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Dropping ekf_pose: NaN/Inf detected in incoming message");
            return;
        }
        if (!has_odom_dynamics_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Skipping publish: waiting for first odom dynamics sample");
            return;
        }

        // 1) Convert pose to state variables
        const double theta = quaternion_to_yaw(qx, qy, qz, qw);
        const double velocity = latest_velocity_;
        const double vy = latest_vy_;
        const double omega = latest_omega_;
        const double steering_angle = compute_steering_angle(velocity, omega);

        // 2) Find nearest waypoint and apply forward bias based on heading.
        auto start_time = std::chrono::high_resolution_clock::now();
        size_t waypoint_idx = kdtree_.find_nearest(x, y);
        waypoint_idx = apply_forward_bias(waypoint_idx, x, y, theta);
        auto end_time = std::chrono::high_resolution_clock::now();

        // 3) Build and publish MpcState message with fixed-point conversion and horizon references.
        auto mpc_state = f1tenth_msgs::msg::MpcState();
        mpc_state.header.stamp = msg->header.stamp;
        mpc_state.header.frame_id = "map";
        mpc_state.x_fp = to_fixed_q16(x);
        mpc_state.y_fp = to_fixed_q16(y);
        mpc_state.theta_fp = to_fixed_q16(theta);
        mpc_state.velocity_fp = to_fixed_q16(velocity);
        mpc_state.vy_fp = to_fixed_q16(vy);
        mpc_state.omega_fp = to_fixed_q16(omega);
        mpc_state.steering_angle_fp = to_fixed_q16(steering_angle);
        fill_horizon_references(mpc_state, waypoint_idx);

        // Publish the state message to the FPGA receiver.
        pub_->publish(mpc_state);
        published_count_++;

        // Debug logging
        static int count = 0;
        if (++count % MPC_FPGA_PUBLISHER_DEBUG_LOG_PERIOD == 0) {
            auto lookup_us = std::chrono::duration_cast<std::chrono::microseconds>(
                end_time - start_time).count();
            RCLCPP_DEBUG(this->get_logger(),
                        "Published MpcState on ekf_pose: pos=(%.2f, %.2f), waypoint=%zu, lookup=%ldus",
                        x, y, waypoint_idx, lookup_us);
        }
    }
};

} // namespace f1tenth_communication

/**
 * @brief Entry point for the state publisher process.
 * @param argc Argument count from process invocation.
 * @param argv Argument vector from process invocation.
 * @return Process exit code (0 on normal shutdown).
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<f1tenth_communication::StatePublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
