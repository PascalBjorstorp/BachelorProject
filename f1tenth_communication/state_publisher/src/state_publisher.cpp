/**
 * @file state_publisher.cpp
 * @brief ROS2 Node: Publishes vehicle state + waypoint index
 *
 * Runs on Jetson. Subscribes to localization, performs KD-tree lookup,
 * publishes MpcState for the Ultra96 MPC controller.
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <f1tenth_msgs/msg/mpc_state.hpp>

#include "state_publisher/kdtree.hpp"

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

class StatePublisherNode : public rclcpp::Node {
public:
    static constexpr size_t MAX_MPC_HORIZON = 10;

    StatePublisherNode() : Node("state_publisher") {
        // Parameters
        this->declare_parameter("trajectory_file", "");
        this->declare_parameter("odom_topic", "/ego_racecar/odom");
        this->declare_parameter("pose_topic", "/ekf_pose");
        this->declare_parameter("output_topic", "/mpc_state");
        this->declare_parameter("servo_topic", "/sensors/servo_position_command");
        this->declare_parameter("wheelbase", 0.324);
        // VESC servo → steering angle conversion
        // Forward: servo = gain * (c2·|δ|² + c1·|δ| + c0) + offset
        // Inverse: solve quadratic to recover δ from servo value
        this->declare_parameter("servo_gain", -0.7284);
        this->declare_parameter("servo_offset", 0.55);
        this->declare_parameter("steering_correction_c2", 0.589566);
        this->declare_parameter("steering_correction_c1", 0.918061);
        this->declare_parameter("steering_correction_c0", 0.001490);
        // Number of waypoints ahead of KD-tree nearest to check for forward bias
        this->declare_parameter("forward_lookahead", 3);
        this->declare_parameter("horizon", static_cast<int>(MAX_MPC_HORIZON));
        this->declare_parameter("default_left_bound", 2.0);
        this->declare_parameter("default_right_bound", 2.0);
        
        std::string trajectory_file = this->get_parameter("trajectory_file").as_string();
        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        std::string pose_topic = this->get_parameter("pose_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        std::string servo_topic = this->get_parameter("servo_topic").as_string();
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        servo_gain_ = this->get_parameter("servo_gain").as_double();
        servo_offset_ = this->get_parameter("servo_offset").as_double();
        steer_c2_ = this->get_parameter("steering_correction_c2").as_double();
        steer_c1_ = this->get_parameter("steering_correction_c1").as_double();
        steer_c0_ = this->get_parameter("steering_correction_c0").as_double();
        forward_lookahead_ = static_cast<int>(this->get_parameter("forward_lookahead").as_int());
        const int horizon_param = this->get_parameter("horizon").as_int();
        default_left_bound_ = this->get_parameter("default_left_bound").as_double();
        default_right_bound_ = this->get_parameter("default_right_bound").as_double();
        if (horizon_param != static_cast<int>(MAX_MPC_HORIZON)) {
            RCLCPP_WARN(this->get_logger(),
                "horizon=%d requested, but FPGA bitstream expects fixed MPC_HORIZON=%zu. Forcing %zu.",
                horizon_param, MAX_MPC_HORIZON, MAX_MPC_HORIZON);
        }
        horizon_ = MAX_MPC_HORIZON;
        
        if (trajectory_file.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No trajectory file specified!");
            return;
        }
        
        // Load trajectory
        if (!load_trajectory(trajectory_file)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load trajectory from: %s", 
                        trajectory_file.c_str());
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from %s (hash=0x%08X)",
                   kdtree_.size(), trajectory_file.c_str(), trajectory_hash_);
            RCLCPP_INFO(this->get_logger(),
                "Streaming mode: horizon-only (length=%zu)", horizon_);
        
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
                    double corrected = (msg->data - servo_offset_) / servo_gain_;
                    double abs_corr = std::abs(corrected);
                    if (steer_c2_ != 0.0) {
                        double disc = steer_c1_ * steer_c1_
                                    - 4.0 * steer_c2_ * (steer_c0_ - abs_corr);
                        if (disc >= 0.0) {
                            double t = (-steer_c1_ + std::sqrt(disc))
                                     / (2.0 * steer_c2_);
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
            std::chrono::milliseconds(500),
            [this]() {
                if (!odom_received_) return;  // Haven't received first message yet
                auto elapsed_ms = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - last_odom_time_).count();
                if (elapsed_ms > 500.0) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "No odom received for %.0f ms — localization may be stalled", elapsed_ms);
                }
            });

        RCLCPP_INFO(this->get_logger(),
                "State publisher ready (Best Effort QoS). Odom cache: %s, pose trigger: %s, publishing: %s",
                odom_topic.c_str(), pose_topic.c_str(), output_topic.c_str());
    }
    
private:
    // --- ROS interfaces ------------------------------------------------------
    KDTree kdtree_;
    rclcpp::Publisher<f1tenth_msgs::msg::MpcState>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr servo_sub_;
    
    // --- Runtime state -------------------------------------------------------
    double current_steering_angle_ = 0.0;  // Steering angle [rad] (converted from servo value)
    bool has_servo_feedback_ = false;
    double wheelbase_ = 0.0;
    double servo_gain_ = 0.0;
    double servo_offset_ = 0.0;
    double steer_c2_ = 0.0;
    double steer_c1_ = 0.0;
    double steer_c0_ = 0.0;
    int forward_lookahead_ = 0;
    size_t horizon_ = MAX_MPC_HORIZON;
    uint32_t trajectory_hash_ = 0;   // Checksum for cross-node trajectory verification
    double default_left_bound_ = 0.0;
    double default_right_bound_ = 0.0;

    // --- Watchdog state -----------------------------------------------------
    rclcpp::TimerBase::SharedPtr odom_watchdog_timer_;
    std::chrono::steady_clock::time_point last_odom_time_ = std::chrono::steady_clock::now();
    bool odom_received_ = false;

    // --- Cached dynamics from odometry -------------------------------------
    double latest_velocity_ = 0.0;
    double latest_vy_ = 0.0;
    double latest_omega_ = 0.0;
    bool has_odom_dynamics_ = false;

    // --- Odometry processing helpers ----------------------------------------
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

        auto ok = [](double v) { return std::isfinite(v); };
        if (!ok(x) || !ok(y) || !ok(qx) || !ok(qy) || !ok(qz) || !ok(qw) ||
            !ok(vx) || !ok(vy) || !ok(wz)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Dropping odom: NaN/Inf detected in incoming message");
            return false;
        }

        constexpr double POS_LIMIT = 500.0;   // ±500 m
        constexpr double VEL_LIMIT = 50.0;    // ±50 m/s
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

    // Convert quaternion to planar yaw.
    static double quaternion_to_yaw(double qx, double qy, double qz, double qw) {
        return std::atan2(2.0 * (qw * qz + qx * qy),
                          1.0 - 2.0 * (qy * qy + qz * qz));
    }

    // Prefer measured steering; fall back to bicycle-model estimate if needed.
    double compute_steering_angle(double velocity, double omega) const {
        if (!has_servo_feedback_ && std::abs(velocity) > 0.1) {
            return std::atan2(wheelbase_ * omega, velocity);
        }
        return current_steering_angle_;
    }

    // Choose a forward waypoint near the geometric nearest index.
    size_t apply_forward_bias(size_t nearest_idx, double x, double y, double theta) const {
        const double cos_theta = std::cos(theta);
        const double sin_theta = std::sin(theta);
        const size_t N = kdtree_.size();
        size_t best_idx = nearest_idx;
        double best_dist = std::numeric_limits<double>::max();

        for (int i = 0; i <= forward_lookahead_; ++i) {
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

    // Convert floating-point values to Q16.16 for FPGA transport.
    static int32_t to_fixed_q16(double v) {
        constexpr double FP_SCALE = 65536.0;
        if (!std::isfinite(v)) {
            return 0;
        }
        return static_cast<int32_t>(v >= 0.0 ? v * FP_SCALE + 0.5 : v * FP_SCALE - 0.5);
    }

    // Fill only the horizon window needed by the receiver/FPGA.
    void fill_horizon_references(f1tenth_msgs::msg::MpcState& mpc_state,
                                 size_t waypoint_idx) const {
        const size_t N = kdtree_.size();
        const size_t stream_count = std::min(horizon_, N);
        mpc_state.horizon_length = static_cast<uint32_t>(stream_count);

        // First waypoint for Frenet error computation (ARM-side)
        const auto& wp0 = kdtree_.get_waypoint(waypoint_idx % N);
        mpc_state.ref_x_0_fp = to_fixed_q16(wp0.x);
        mpc_state.ref_y_0_fp = to_fixed_q16(wp0.y);
        mpc_state.ref_psi_0_fp = to_fixed_q16(wp0.psi);

        // Resize only the arrays that go to FPGA
        mpc_state.ref_vx_fp.resize(stream_count);
        mpc_state.ref_kappa_fp.resize(stream_count);
        mpc_state.ref_left_bound_fp.resize(stream_count);
        mpc_state.ref_right_bound_fp.resize(stream_count);

        // Stream only the required horizon waypoints each cycle.
        for (size_t i = 0; i < stream_count; ++i) {
            const size_t idx = (waypoint_idx + i) % N;
            const auto& wp = kdtree_.get_waypoint(idx);
            mpc_state.ref_vx_fp[i] = to_fixed_q16(wp.vx);
            mpc_state.ref_kappa_fp[i] = to_fixed_q16(wp.kappa);
            mpc_state.ref_left_bound_fp[i] = to_fixed_q16(wp.left_bound);
            mpc_state.ref_right_bound_fp[i] = to_fixed_q16(wp.right_bound);
        }
    }

    // --- Trajectory I/O ------------------------------------------------------
    
    bool load_trajectory(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            return false;
        }
        
        std::vector<Waypoint> waypoints;
        std::string line;
        
        // Skip header
        std::getline(file, line);
        
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string token;
            Waypoint wp;
            
            std::getline(ss, token, ','); wp.s = std::stod(token);
            std::getline(ss, token, ','); wp.x = std::stod(token);
            std::getline(ss, token, ','); wp.y = std::stod(token);
            std::getline(ss, token, ','); wp.psi = std::stod(token);
            std::getline(ss, token, ','); wp.kappa = std::stod(token);
            std::getline(ss, token, ','); wp.vx = std::stod(token);
            std::getline(ss, token, ',');  // Legacy ax column kept for CSV compatibility
            wp.left_bound = default_left_bound_;
            wp.right_bound = default_right_bound_;

            // Optional bounds columns: left_bound,right_bound
            if (std::getline(ss, token, ',')) {
                if (!token.empty()) wp.left_bound = std::stod(token);
                if (std::getline(ss, token, ',')) {
                    if (!token.empty()) wp.right_bound = std::stod(token);
                }
            }
            
            waypoints.push_back(wp);
        }
        
        if (waypoints.empty()) {
            return false;
        }
        
        kdtree_.build(waypoints);

        // Compute trajectory checksum for cross-node verification
        trajectory_hash_ = 0;
        for (const auto& wp : waypoints) {
            trajectory_hash_ ^= static_cast<uint32_t>(wp.x * 65536.0)
                              ^ (static_cast<uint32_t>(wp.y * 65536.0) << 16);
        }
        return true;
    }
    
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

    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        const double x = msg->pose.pose.position.x;
        const double y = msg->pose.pose.position.y;
        const double qx = msg->pose.pose.orientation.x;
        const double qy = msg->pose.pose.orientation.y;
        const double qz = msg->pose.pose.orientation.z;
        const double qw = msg->pose.pose.orientation.w;

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

        const double theta = quaternion_to_yaw(qx, qy, qz, qw);
        const double velocity = latest_velocity_;
        const double vy = latest_vy_;
        const double omega = latest_omega_;
        const double steering_angle = compute_steering_angle(velocity, omega);

        auto start_time = std::chrono::high_resolution_clock::now();
        size_t waypoint_idx = kdtree_.find_nearest(x, y);
        waypoint_idx = apply_forward_bias(waypoint_idx, x, y, theta);
        auto end_time = std::chrono::high_resolution_clock::now();

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

        pub_->publish(mpc_state);

        static int count = 0;
        if (++count % 50 == 0) {
            auto lookup_us = std::chrono::duration_cast<std::chrono::microseconds>(
                end_time - start_time).count();
            RCLCPP_DEBUG(this->get_logger(),
                        "Published MpcState on ekf_pose: pos=(%.2f, %.2f), waypoint=%zu, lookup=%ldus",
                        x, y, waypoint_idx, lookup_us);
        }
    }
};

} // namespace f1tenth_communication

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<f1tenth_communication::StatePublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
