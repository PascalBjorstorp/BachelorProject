/**
 * @file state_publisher_v2.cpp
 * @brief Publish vehicle state and streamed MPC references from Jetson using local_raceline.
 * @details Subscribes to local_raceline path, receives pose/odometry, and publishes raw-QP
 *          `MpcState` packets for the Kria receiver. Follows mpc_hardware_node architecture.
 * @dependencies rclcpp, nav_msgs, geometry_msgs, std_msgs, f1tenth_msgs, mpc_fpga_constants.h
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <f1tenth_msgs/msg/mpc_state.hpp>

#include "mpc_fpga_constants.h"

#include <array>
#include <algorithm>
#include <limits>
#include <cmath>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <sys/stat.h>

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

struct FrenetErrorsFp {
    int32_t e_y_fp = 0;
    int32_t e_psi_fp = 0;
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
        this->declare_parameter("drive_topic", "/drive");

        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        std::string pose_topic = this->get_parameter("pose_topic").as_string();
        std::string raceline_topic = this->get_parameter("raceline_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        std::string servo_topic = this->get_parameter("servo_topic").as_string();
        std::string drive_topic = this->get_parameter("drive_topic").as_string();

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

        // Subscribe to /drive feedback for round-trip latency measurement
        drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic, qos,
            std::bind(&StatePublisherNode::drive_callback, this, std::placeholders::_1));

        // Subscribe to servo feedback
        if (!servo_topic.empty()) {
            servo_sub_ = this->create_subscription<std_msgs::msg::Float64>(
                servo_topic, qos,
                [this](const std_msgs::msg::Float64::SharedPtr msg) {
                    const double corrected =
                        (msg->data - static_cast<double>(MPC_FPGA_SERVO_OFFSET)) /
                        static_cast<double>(MPC_FPGA_SERVO_GAIN);
                    const double abs_corr = std::abs(corrected);
                    if (static_cast<double>(MPC_FPGA_STEER_CORRECTION_C2) != 0.0) {
                        const double disc =
                            static_cast<double>(MPC_FPGA_STEER_CORRECTION_C1) *
                                static_cast<double>(MPC_FPGA_STEER_CORRECTION_C1) -
                            4.0 * static_cast<double>(MPC_FPGA_STEER_CORRECTION_C2) *
                                (static_cast<double>(MPC_FPGA_STEER_CORRECTION_C0) - abs_corr);
                        if (disc >= 0.0) {
                            const double t =
                                (-static_cast<double>(MPC_FPGA_STEER_CORRECTION_C1) + std::sqrt(disc)) /
                                (2.0 * static_cast<double>(MPC_FPGA_STEER_CORRECTION_C2));
                            current_steering_angle_ = std::copysign(t, corrected);
                        } else {
                            current_steering_angle_ = corrected;
                        }
                    } else {
                        current_steering_angle_ = corrected;
                    }
                });
        }

        RCLCPP_INFO(this->get_logger(),
            "State publisher ready (local_raceline). Raceline: %s, Pose: %s, Odom: %s -> %s",
            raceline_topic.c_str(), pose_topic.c_str(), odom_topic.c_str(), output_topic.c_str());
        RCLCPP_INFO(this->get_logger(),
            "Round-trip latency tracking active: /drive topic = %s",
            drive_topic.c_str());

        open_roundtrip_csv_file();
    }

    ~StatePublisherNode() override {
        if (rt_csv_file_ != nullptr) {
            fclose(rt_csv_file_);
            rt_csv_file_ = nullptr;
        }
    }

private:
    // --- ROS Interfaces ---
    rclcpp::Publisher<f1tenth_msgs::msg::MpcState>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr raceline_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;
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
    uint64_t rt_window_count_ = 0;
    double rt_window_sum_us_ = 0.0;
    double rt_window_min_us_ = 1e12;
    double rt_window_max_us_ = 0.0;
    uint64_t rt_csv_idx_ = 0;
    std::chrono::steady_clock::time_point rt_last_print_time_ = std::chrono::steady_clock::now();
    FILE* rt_csv_file_ = nullptr;

    static constexpr double kRoundtripPrintIntervalSec = 5.0;

    // --- Helpers ---

    static int32_t to_fixed_qp(double v) {
        constexpr double SCALE = MPC_FPGA_QP_SCALE_F64;
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

    FrenetErrorsFp compute_frenet_errors(double x, double y, double theta) const {
        if (local_raceline_.size() < 2) {
            return {};
        }

        const size_t max_search = std::min(local_raceline_.size() - 1, static_cast<size_t>(16));
        double best_e_y = 0.0;
        double best_e_psi = 0.0;
        double best_dist2 = std::numeric_limits<double>::max();

        for (size_t i = 0; i < max_search; ++i) {
            const auto& a = local_raceline_[i];
            const auto& b = local_raceline_[i + 1];

            const double abx = b.x - a.x;
            const double aby = b.y - a.y;
            const double apx = x - a.x;
            const double apy = y - a.y;
            const double ab_len2 = abx * abx + aby * aby;
            double t = 0.0;
            if (ab_len2 > 1e-12) {
                t = (apx * abx + apy * aby) / ab_len2;
            }
            t = std::clamp(t, 0.0, 1.0);

            double dpsi_path = normalize_angle(b.psi - a.psi);
            const double path_psi = a.psi + t * dpsi_path;
            const double path_x = a.x + t * abx;
            const double path_y = a.y + t * aby;
            const double dx = x - path_x;
            const double dy = y - path_y;
            const double dist2 = dx * dx + dy * dy;

            if (dist2 < best_dist2) {
                best_dist2 = dist2;
                best_e_y = -std::sin(path_psi) * dx + std::cos(path_psi) * dy;
                best_e_psi = normalize_angle(theta - path_psi);
            }
        }

        return {to_fixed_qp(best_e_y), to_fixed_qp(best_e_psi)};
    }

    void open_roundtrip_csv_file() {
        const char* log_dir = "log";
        mkdir(log_dir, 0755);

        time_t now = time(nullptr);
        struct tm tm_now;
        localtime_r(&now, &tm_now);

        char timestamp[64];
        strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &tm_now);

        char csv_path[512];
        snprintf(csv_path, sizeof(csv_path), "%s/jetson_roundtrip_%s.csv", log_dir, timestamp);

        rt_csv_file_ = fopen(csv_path, "w");
        if (rt_csv_file_ == nullptr) {
            RCLCPP_WARN(this->get_logger(),
                "Failed to open round-trip CSV file: %s", csv_path);
            return;
        }

        fprintf(rt_csv_file_, "idx,roundtrip_us\n");
        fflush(rt_csv_file_);
        RCLCPP_INFO(this->get_logger(), "Round-trip CSV log: %s", csv_path);
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

        // Resize horizon arrays consumed by the FPGA.
        mpc_state.horizon_length = static_cast<uint32_t>(horizon);
        mpc_state.ref_ey_fp.resize(horizon);
        mpc_state.ref_epsi_fp.resize(horizon);
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
            mpc_state.ref_ey_fp[step] = to_fixed_qp(0.0);
            mpc_state.ref_epsi_fp[step] = to_fixed_qp(0.0);

            mpc_state.ref_vx_fp[step] = to_fixed_qp(wp.vx);
            mpc_state.ref_vy_fp[step] = to_fixed_qp(0.0);
            mpc_state.ref_omega_ref_fp[step] = to_fixed_qp(wp.vx * wp.kappa);

            mpc_state.ref_kappa_fp[step] = to_fixed_qp(wp.kappa);
            mpc_state.ref_left_bound_fp[step] = to_fixed_qp(wp.left_bound);
            mpc_state.ref_right_bound_fp[step] = to_fixed_qp(wp.right_bound);
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

    void drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
        const rclcpp::Time src_time(msg->header.stamp);
        if (src_time.nanoseconds() <= 0) {
            return;
        }

        const double rt_us = (this->now() - src_time).seconds() * 1e6;
        if (rt_us < 0.0) {
            return;
        }

        if (rt_csv_file_ != nullptr) {
            rt_csv_idx_++;
            fprintf(rt_csv_file_, "%lu,%.1f\n", rt_csv_idx_, rt_us);
            fflush(rt_csv_file_);
        }

        rt_window_count_++;
        rt_window_sum_us_ += rt_us;
        if (rt_us < rt_window_min_us_) rt_window_min_us_ = rt_us;
        if (rt_us > rt_window_max_us_) rt_window_max_us_ = rt_us;

        const auto now = std::chrono::steady_clock::now();
        const double elapsed_sec =
            std::chrono::duration<double>(now - rt_last_print_time_).count();
        if (elapsed_sec >= kRoundtripPrintIntervalSec && rt_window_count_ > 0) {
            const double avg_us = rt_window_sum_us_ / static_cast<double>(rt_window_count_);
            RCLCPP_INFO(this->get_logger(),
                "[RoundTrip] Stats (last %.1fs, %lu samples): min=%.1f us, avg=%.1f us, max=%.1f us",
                elapsed_sec, rt_window_count_, rt_window_min_us_, avg_us, rt_window_max_us_);

            rt_window_count_ = 0;
            rt_window_sum_us_ = 0.0;
            rt_window_min_us_ = 1e12;
            rt_window_max_us_ = 0.0;
            rt_last_print_time_ = now;
        }
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

        const FrenetErrorsFp errors = compute_frenet_errors(x, y, theta);
        mpc_state->e_y_fp = errors.e_y_fp;
        mpc_state->e_psi_fp = errors.e_psi_fp;
        mpc_state->velocity_fp = to_fixed_qp(latest_vx_);
        mpc_state->vy_fp = to_fixed_qp(latest_vy_);
        mpc_state->omega_fp = to_fixed_qp(latest_omega_);
        mpc_state->steering_angle_fp = to_fixed_qp(current_steering_angle_);

        // Build horizon from first waypoint (index 0) using arc-length lookahead
        build_horizon_from_raceline(*mpc_state);

        uint32_t horizon_len = mpc_state->horizon_length;

        /* Keep the EKF pose stamp as the pipeline monitor token. */
        
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
