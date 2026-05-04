/**
 * @file state_publisher_v2.cpp
 * @brief Publish vehicle state and streamed MPC references from Jetson using local_raceline.
 * @details Subscribes to local_raceline path, receives pose/odometry, and publishes Q16.16
 *          `MpcState` packets for the Kria receiver. Follows mpc_hardware_node architecture.
 * @dependencies rclcpp, nav_msgs, geometry_msgs, std_msgs, f1tenth_msgs, mpc_fpga_constants.h
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <f1tenth_msgs/msg/mpc_state.hpp>

#include "mpc_fpga_constants.h"

#include <array>
#include <algorithm>
#include <limits>
#include <cmath>
#include <chrono>

namespace f1tenth_communication {

/*===========================================================================
 * Reference Waypoint Structure
 *===========================================================================*/

struct RefWaypoint {
    double s = 0.0;        // cumulative arc-length [m]
    double x = 0.0;
    double y = 0.0;
    double psi = 0.0;
    double vx = 0.5;
    double kappa = 0.0;
    double left_bound = 0.5;
    double right_bound = 0.5;
};

/* Interpolated sampled reference point used for horizon construction */
struct SampledRefPoint {
    double s = 0.0;
    double x = 0.0;
    double y = 0.0;
    double psi = 0.0;
    double vx = 0.0;
    double kappa = 0.0;
    double left_bound = 0.5;
    double right_bound = 0.5;
};

/*===========================================================================
 * State Publisher Node using Local Raceline
 *===========================================================================*/

class StatePublisherNode : public rclcpp::Node {
public:
    StatePublisherNode() : Node("state_publisher") {
        // Parameters
        this->declare_parameter("odom_topic", "/ego_racecar/odom");
        this->declare_parameter("pose_topic", "/ekf_pose");
        this->declare_parameter("raceline_topic", "/local_raceline");
        this->declare_parameter("output_topic", "/mpc_state");
        this->declare_parameter("servo_topic", "/sensors/servo_position_command");

        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        std::string pose_topic = this->get_parameter("pose_topic").as_string();
        std::string raceline_topic = this->get_parameter("raceline_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        std::string servo_topic = this->get_parameter("servo_topic").as_string();

        // Best Effort + volatile minimizes control latency under packet loss
        auto qos = rclcpp::QoS(1).best_effort().durability_volatile();
        
        pub_ = this->create_publisher<f1tenth_msgs::msg::MpcState>(output_topic, qos);

        // Subscribe to local_raceline (primary reference source)
        raceline_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            raceline_topic, qos,
            std::bind(&StatePublisherNode::raceline_callback, this, std::placeholders::_1));

        // Subscribe to odometry (velocity/yaw-rate cache)
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, qos,
            std::bind(&StatePublisherNode::odom_callback, this, std::placeholders::_1));

        // Subscribe to EKF pose (triggers publishing)
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            pose_topic, qos,
            std::bind(&StatePublisherNode::pose_callback, this, std::placeholders::_1));

        // Subscribe to servo feedback
        if (!servo_topic.empty()) {
            servo_sub_ = this->create_subscription<std_msgs::msg::Float64>(
                servo_topic, qos,
                [this](const std_msgs::msg::Float64::SharedPtr msg) {
                    // Simple linear mapping (adjust if needed for VESC nonlinearity)
                    current_steering_angle_ = msg->data;
                });
        }

        RCLCPP_INFO(this->get_logger(),
            "State publisher ready (local_raceline). Raceline: %s, Pose: %s, Odom: %s -> %s",
            raceline_topic.c_str(), pose_topic.c_str(), odom_topic.c_str(), output_topic.c_str());
    }

private:
    // --- ROS Interfaces ---
    rclcpp::Publisher<f1tenth_msgs::msg::MpcState>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr raceline_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr servo_sub_;

    // --- Runtime State ---
    std::vector<RefWaypoint> local_raceline_;
    double current_steering_angle_ = 0.0;

    // Cached odometry
    double latest_vx_ = 0.0;
    double latest_vy_ = 0.0;
    double latest_omega_ = 0.0;
    bool has_odom_ = false;

    uint64_t published_count_ = 0;

    // --- Helpers ---

    static int32_t to_fixed_q16(double v) {
        constexpr double SCALE = MPC_FPGA_Q16_SCALE_F64;
        if (!std::isfinite(v)) return 0;
        return static_cast<int32_t>(v >= 0.0 ? v * SCALE + 0.5 : v * SCALE - 0.5);
    }

    static double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    static double quaternion_to_yaw(double qx, double qy, double qz, double qw) {
        return std::atan2(2.0 * (qw * qz + qx * qy),
                          1.0 - 2.0 * (qy * qy + qz * qz));
    }

    // Build arc-length based horizon from local_raceline using first waypoint velocity
    // Matches mpc_hardware_node's build_reference_from_local_raceline() exactly
    void build_horizon_from_raceline(f1tenth_msgs::msg::MpcState& mpc_state) {
        if (local_raceline_.empty()) {
            mpc_state.horizon_length = 0;
            return;
        }

        const size_t N = local_raceline_.size();
        const size_t horizon = std::min(static_cast<size_t>(MPC_FPGA_HORIZON_STEPS), N);
        const double dt = MPC_FPGA_PREDICTION_DT_S;

        // Use FIRST waypoint's reference velocity for arc-length lookahead
        // This ensures horizon is computed from valid reference speed, not actual velocity
        double v_ref_base = local_raceline_[0].vx;
        if (v_ref_base <= 0.0) {
            v_ref_base = std::clamp(
                static_cast<double>(MPC_FPGA_MAX_VEL_MPS) * 0.7,
                static_cast<double>(MPC_FPGA_MIN_VEL_MPS),
                static_cast<double>(MPC_FPGA_MAX_VEL_MPS));
        }

        // Resize all horizon arrays (V2 message includes geometry arrays)
        mpc_state.horizon_length = static_cast<uint32_t>(horizon);
        mpc_state.ref_ey_fp.resize(horizon);
        mpc_state.ref_epsi_fp.resize(horizon);
        mpc_state.ref_x_fp.resize(horizon);
        mpc_state.ref_y_fp.resize(horizon);
        mpc_state.ref_psi_fp.resize(horizon);
        mpc_state.ref_vx_fp.resize(horizon);
        mpc_state.ref_vy_fp.resize(horizon);
        mpc_state.ref_omega_ref_fp.resize(horizon);
        mpc_state.ref_kappa_fp.resize(horizon);
        mpc_state.ref_left_bound_fp.resize(horizon);
        mpc_state.ref_right_bound_fp.resize(horizon);

        // Helper: sample raceline by arc-length with linear interpolation
        auto sample_raceline_by_s = [this, N](double target_s) {
            SampledRefPoint out;
            if (local_raceline_.empty()) return out;
            // Clamp target_s to raceline span
            double s0 = local_raceline_.front().s;
            double sN = local_raceline_.back().s;
            if (sN - s0 < 1e-6) {
                const auto& wp = local_raceline_.front();
                out.s = wp.s; out.x = wp.x; out.y = wp.y; out.psi = wp.psi;
                out.vx = wp.vx; out.kappa = wp.kappa; out.left_bound = wp.left_bound; out.right_bound = wp.right_bound;
                return out;
            }
            if (target_s <= s0) target_s = s0;
            if (target_s >= sN) target_s = sN;

            // Find lower index
            size_t idx = 0;
            for (size_t i = 0; i + 1 < local_raceline_.size(); ++i) {
                if (local_raceline_[i].s <= target_s && local_raceline_[i+1].s >= target_s) {
                    idx = i; break;
                }
            }
            const auto& a = local_raceline_[idx];
            const auto& b = (idx + 1 < local_raceline_.size()) ? local_raceline_[idx + 1] : local_raceline_[idx];
            double ds = b.s - a.s;
            double t = (ds > 1e-9) ? ((target_s - a.s) / ds) : 0.0;
            // Linear interp for x,y,vx,kappa,bounds
            out.s = a.s + t * (b.s - a.s);
            out.x = a.x + t * (b.x - a.x);
            out.y = a.y + t * (b.y - a.y);
            // Interpolate heading safely across +-pi
            double dpsi = b.psi - a.psi;
            while (dpsi > M_PI) dpsi -= 2.0 * M_PI;
            while (dpsi < -M_PI) dpsi += 2.0 * M_PI;
            out.psi = a.psi + t * dpsi;
            out.vx = a.vx + t * (b.vx - a.vx);
            out.kappa = a.kappa + t * (b.kappa - a.kappa);
            out.left_bound = a.left_bound + t * (b.left_bound - a.left_bound);
            out.right_bound = a.right_bound + t * (b.right_bound - a.right_bound);
            return out;
        };

        // Fill horizon using arc-length lookahead: target_s = v_ref * dt * step
        for (size_t step = 0; step < horizon; ++step) {
            double target_s = v_ref_base * dt * static_cast<double>(step);
            SampledRefPoint wp = sample_raceline_by_s(target_s);
            mpc_state.ref_ey_fp[step] = to_fixed_q16(0.0);
            mpc_state.ref_epsi_fp[step] = to_fixed_q16(0.0);

            mpc_state.ref_x_fp[step] = to_fixed_q16(wp.x);
            mpc_state.ref_y_fp[step] = to_fixed_q16(wp.y);
            mpc_state.ref_psi_fp[step] = to_fixed_q16(wp.psi);

            mpc_state.ref_vx_fp[step] = to_fixed_q16(wp.vx);
            mpc_state.ref_vy_fp[step] = to_fixed_q16(0.0);
            mpc_state.ref_omega_ref_fp[step] = to_fixed_q16(wp.vx * wp.kappa);

            mpc_state.ref_kappa_fp[step] = to_fixed_q16(wp.kappa);
            mpc_state.ref_left_bound_fp[step] = to_fixed_q16(wp.left_bound);
            mpc_state.ref_right_bound_fp[step] = to_fixed_q16(wp.right_bound);
        }
    }

    // --- Callbacks ---

    void raceline_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->poses.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Empty local_raceline received");
            return;
        }

        // Parse waypoints and compute arc-length, heading, curvature (like hardware_node)
        std::vector<RefWaypoint> new_raceline;
        double cumulative_s = 0.0;
        double prev_x = 0.0, prev_y = 0.0;
        size_t waypoint_count = msg->poses.size();

        // First pass: extract position, compute cumulative arc-length and velocity
        for (size_t i = 0; i < waypoint_count; ++i) {
            const auto& pose = msg->poses[i];
            const auto& pos = pose.pose.position;
            const auto& ori = pose.pose.orientation;

            RefWaypoint wp;
            wp.x = pos.x;
            wp.y = pos.y;

            // Compute cumulative arc-length
            if (i > 0) {
                double dx = wp.x - prev_x;
                double dy = wp.y - prev_y;
                cumulative_s += std::hypot(dx, dy);
            }
            wp.s = cumulative_s;

            // Extract reference velocity from position.z (like hardware_node)
            double v_ref = std::abs(pos.z);
            if (!std::isfinite(v_ref) || v_ref < MPC_FPGA_MIN_VEL_MPS) {
                v_ref = MPC_FPGA_MIN_VEL_MPS;
            }
            if (v_ref > MPC_FPGA_MAX_VEL_MPS) {
                v_ref = MPC_FPGA_MAX_VEL_MPS;
            }
            wp.vx = v_ref;

            // Extract track bounds from orientation.x/y (like hardware_node)
            double left_bound = ori.x;
            double right_bound = ori.y;
            if (!std::isfinite(left_bound) || left_bound <= 0.0) {
                left_bound = 0.5;  // Fallback
            }
            if (!std::isfinite(right_bound) || right_bound <= 0.0) {
                right_bound = 0.5;  // Fallback
            }
            wp.left_bound = left_bound;
            wp.right_bound = right_bound;

            // Placeholder for heading and curvature (computed in second pass)
            wp.psi = 0.0;
            wp.kappa = 0.0;

            new_raceline.push_back(wp);
            prev_x = wp.x;
            prev_y = wp.y;
        }

        // Second pass: compute heading from consecutive waypoints (like hardware_node)
        for (size_t i = 0; i < waypoint_count; ++i) {
            size_t i_prev = (i == 0) ? 0 : (i - 1);
            size_t i_next = (i + 1 < waypoint_count) ? (i + 1) : (waypoint_count - 1);

            double dx = new_raceline[i_next].x - new_raceline[i_prev].x;
            double dy = new_raceline[i_next].y - new_raceline[i_prev].y;

            double heading = 0.0;
            if ((dx * dx + dy * dy) > 1e-12) {
                heading = std::atan2(dy, dx);
            } else if (i > 0) {
                heading = new_raceline[i - 1].psi;
            }
            new_raceline[i].psi = heading;
        }

        // Third pass: compute curvature from heading difference (like hardware_node)
        if (waypoint_count >= 3) {
            for (size_t i = 1; i + 1 < waypoint_count; ++i) {
                double dpsi = normalize_angle(
                    new_raceline[i + 1].psi - new_raceline[i - 1].psi);
                double ds = new_raceline[i + 1].s - new_raceline[i - 1].s;
                double ds_safe = (ds > 1e-6) ? ds : 1e-6;
                new_raceline[i].kappa = dpsi / ds_safe;
            }
            // Extrapolate curvature at endpoints
            new_raceline[0].kappa = new_raceline[1].kappa;
            new_raceline[waypoint_count - 1].kappa = new_raceline[waypoint_count - 2].kappa;
        }

        local_raceline_ = new_raceline;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Updated local_raceline with %zu waypoints, length=%.2f m",
            local_raceline_.size(),
            waypoint_count > 1 ? (local_raceline_.back().s - local_raceline_.front().s) : 0.0);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_vx_ = msg->twist.twist.linear.x;
        latest_vy_ = msg->twist.twist.linear.y;
        latest_omega_ = msg->twist.twist.angular.z;
        has_odom_ = true;
    }

    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        if (local_raceline_.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Pose received but no local_raceline available yet");
            return;
        }

        if (!has_odom_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Pose received but no odometry available yet");
            return;
        }

        // Build MpcState message
        auto mpc_state = std::make_unique<f1tenth_msgs::msg::MpcState>();
        mpc_state->header.stamp = msg->header.stamp;
        mpc_state->header.frame_id = msg->header.frame_id;

        // Current state
        const double x = msg->pose.pose.position.x;
        const double y = msg->pose.pose.position.y;
        const double theta = quaternion_to_yaw(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

        mpc_state->x_fp = to_fixed_q16(x);
        mpc_state->y_fp = to_fixed_q16(y);
        mpc_state->theta_fp = to_fixed_q16(theta);
        mpc_state->velocity_fp = to_fixed_q16(latest_vx_);
        mpc_state->vy_fp = to_fixed_q16(latest_vy_);
        mpc_state->omega_fp = to_fixed_q16(latest_omega_);
        mpc_state->steering_angle_fp = to_fixed_q16(current_steering_angle_);

        // Build horizon from first waypoint (index 0) using arc-length lookahead
        build_horizon_from_raceline(*mpc_state);

        uint32_t horizon_len = mpc_state->horizon_length;
        
        // Publish
        pub_->publish(std::move(mpc_state));
        published_count_++;

        if (published_count_ == 1) {
            RCLCPP_INFO(this->get_logger(), "First MpcState published (%u waypoints in horizon)",
                horizon_len);
        }
    }
};

}  // namespace f1tenth_communication

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<f1tenth_communication::StatePublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
