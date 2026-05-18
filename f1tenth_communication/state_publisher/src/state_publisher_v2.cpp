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
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <string>
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
        this->declare_parameter("enable_mpc_state_debug_csv", true);
        this->declare_parameter("drop_stale_raceline", false);
        this->declare_parameter("max_pose_raceline_age_ms", 150.0);

        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        std::string pose_topic = this->get_parameter("pose_topic").as_string();
        std::string raceline_topic = this->get_parameter("raceline_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        std::string servo_topic = this->get_parameter("servo_topic").as_string();
        std::string drive_topic = this->get_parameter("drive_topic").as_string();
        mpc_state_debug_csv_enabled_ =
            this->get_parameter("enable_mpc_state_debug_csv").as_bool();
        drop_stale_raceline_ =
            this->get_parameter("drop_stale_raceline").as_bool();
        max_pose_raceline_age_ms_ =
            this->get_parameter("max_pose_raceline_age_ms").as_double();

        // Keep transport output lightweight, but match MPC input subscriptions.
        auto pub_qos = rclcpp::QoS(1).best_effort().durability_volatile();
        auto sub_qos = rclcpp::QoS(10).reliable().durability_volatile();

        pub_ = this->create_publisher<f1tenth_msgs::msg::MpcState>(output_topic, pub_qos);

        // Subscribe to local_raceline (primary reference source)
        raceline_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            raceline_topic, sub_qos,
            std::bind(&StatePublisherNode::raceline_callback, this, std::placeholders::_1));

        // Subscribe to odometry (velocity/yaw-rate cache)
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, sub_qos,
            std::bind(&StatePublisherNode::odom_callback, this, std::placeholders::_1));

        // Subscribe to EKF pose (triggers publishing)
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            pose_topic, sub_qos,
            std::bind(&StatePublisherNode::pose_callback, this, std::placeholders::_1));

        // Subscribe to /drive feedback for round-trip latency measurement
        drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic, sub_qos,
            std::bind(&StatePublisherNode::drive_callback, this, std::placeholders::_1));

        // Subscribe to servo feedback
        if (!servo_topic.empty()) {
            servo_sub_ = this->create_subscription<std_msgs::msg::Float64>(
                servo_topic, sub_qos,
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
        if (mpc_state_debug_csv_enabled_) {
            open_mpc_state_debug_csv_file();
        }
    }

    ~StatePublisherNode() override {
        if (rt_csv_file_ != nullptr) {
            fclose(rt_csv_file_);
            rt_csv_file_ = nullptr;
        }
        if (mpc_state_csv_file_ != nullptr) {
            fclose(mpc_state_csv_file_);
            mpc_state_csv_file_ = nullptr;
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
    int64_t latest_odom_stamp_ns_ = 0;
    int64_t latest_raceline_stamp_ns_ = 0;
    std::string latest_raceline_frame_;
    uint64_t latest_raceline_seq_ = 0;

    uint64_t published_count_ = 0;
    uint64_t rt_window_count_ = 0;
    double rt_window_sum_us_ = 0.0;
    double rt_window_min_us_ = 1e12;
    double rt_window_max_us_ = 0.0;
    uint64_t rt_csv_idx_ = 0;
    std::chrono::steady_clock::time_point rt_last_print_time_ = std::chrono::steady_clock::now();
    FILE* rt_csv_file_ = nullptr;
    FILE* mpc_state_csv_file_ = nullptr;
    uint64_t mpc_state_csv_idx_ = 0;
    bool mpc_state_debug_csv_enabled_ = true;
    bool drop_stale_raceline_ = false;
    double max_pose_raceline_age_ms_ = 150.0;

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

    static int64_t stamp_to_ns(const builtin_interfaces::msg::Time& stamp) {
        return static_cast<int64_t>(stamp.sec) * 1000000000LL +
               static_cast<int64_t>(stamp.nanosec);
    }

    static double fp_to_double(int32_t raw) {
        return static_cast<double>(raw) / MPC_FPGA_QP_SCALE_F64;
    }

    FrenetErrorsFp compute_frenet_errors(double x, double y, double theta) const {
        if (local_raceline_.size() < 2) {
            return {};
        }

        /* The local raceline is republished every cycle anchored at the car,
         * so segment [0,1] IS the current reference origin. Project onto it
         * directly: a global nearest-segment scan adds per-cycle cost and can
         * snap to an offset/curved-back segment, biasing e_y. (Matches the
         * MPC hardware node's closest=0 anchoring.) */
        const auto& a = local_raceline_[0];
        const auto& b = local_raceline_[1];

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

        const double dpsi_path = normalize_angle(b.psi - a.psi);
        const double path_psi = a.psi + t * dpsi_path;
        const double path_x = a.x + t * abx;
        const double path_y = a.y + t * aby;
        const double dx = x - path_x;
        const double dy = y - path_y;

        const double best_e_y = -std::sin(path_psi) * dx + std::cos(path_psi) * dy;
        const double best_e_psi = normalize_angle(theta - path_psi);

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

    void open_mpc_state_debug_csv_file() {
        const char* log_dir = "log";
        mkdir(log_dir, 0755);

        time_t now = time(nullptr);
        struct tm tm_now;
        localtime_r(&now, &tm_now);

        char timestamp[64];
        strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &tm_now);

        char csv_path[512];
        snprintf(csv_path, sizeof(csv_path), "%s/jetson_mpc_state_debug_%s.csv", log_dir, timestamp);

        mpc_state_csv_file_ = fopen(csv_path, "w");
        if (mpc_state_csv_file_ == nullptr) {
            RCLCPP_WARN(this->get_logger(),
                "Failed to open MpcState debug CSV file: %s", csv_path);
            return;
        }

        fprintf(mpc_state_csv_file_,
            "idx,pose_stamp_ns,pose_frame,raceline_stamp_ns,raceline_frame,"
            "pose_raceline_age_ms,odom_age_ms,raceline_seq,waypoint_count,"
            "pose_x,pose_y,pose_theta,"
            "wp0_x,wp0_y,wp0_psi,wp0_vx,wp0_kappa,wp0_left,wp0_right,"
            "wp1_x,wp1_y,wp1_psi,"
            "e_y,e_psi,vx,vy,omega,delta,"
            "ref_vx_0,ref_kappa_0,ref_omega_0\n");
        fflush(mpc_state_csv_file_);
        RCLCPP_INFO(this->get_logger(), "MpcState debug CSV log: %s", csv_path);
    }

    // Build arc-length based horizon from local_raceline using first waypoint velocity.
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

        // Helper: sample raceline by arc-length with linear interpolation.
        auto sample_raceline_by_s = [this, N](double target_s) {
            SampledRefPoint out;
            if (local_raceline_.empty()) return out;

            double s0 = local_raceline_.front().s;
            double sN = local_raceline_.back().s;
            if (sN - s0 < 1e-6) {
                const auto& wp = local_raceline_.front();
                out.s = wp.s;
                out.x = wp.x;
                out.y = wp.y;
                out.psi = wp.psi;
                out.vx = wp.vx;
                out.kappa = wp.kappa;
                out.left_bound = wp.left_bound;
                out.right_bound = wp.right_bound;
                return out;
            }

            if (target_s <= s0) target_s = s0;
            if (target_s >= sN) target_s = sN;

            size_t idx = 0;
            for (size_t i = 0; i + 1 < N; ++i) {
                if (local_raceline_[i].s <= target_s &&
                    local_raceline_[i + 1].s >= target_s) {
                    idx = i;
                    break;
                }
            }

            const auto& a = local_raceline_[idx];
            const auto& b =
                (idx + 1 < N) ? local_raceline_[idx + 1] : local_raceline_[idx];
            const double ds = b.s - a.s;
            const double t = (ds > 1e-9) ? ((target_s - a.s) / ds) : 0.0;

            out.s = a.s + t * (b.s - a.s);
            out.x = a.x + t * (b.x - a.x);
            out.y = a.y + t * (b.y - a.y);

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

        // Fill horizon using arc-length lookahead: target_s = v_ref * dt * step.
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
        latest_raceline_stamp_ns_ = stamp_to_ns(msg->header.stamp);
        latest_raceline_frame_ = msg->header.frame_id;
        latest_raceline_seq_++;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Updated local_raceline with %zu waypoints, length=%.2f m",
            local_raceline_.size(),
            waypoint_count > 1 ? (local_raceline_.back().s - local_raceline_.front().s) : 0.0);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_vx_ = msg->twist.twist.linear.x;
        latest_vy_ = msg->twist.twist.linear.y;
        latest_omega_ = msg->twist.twist.angular.z;
        latest_odom_stamp_ns_ = stamp_to_ns(msg->header.stamp);
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
        const int64_t pose_stamp_ns = stamp_to_ns(msg->header.stamp);
        const double x = msg->pose.pose.position.x;
        const double y = msg->pose.pose.position.y;
        const double theta = quaternion_to_yaw(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        const double pose_raceline_age_ms =
            static_cast<double>(pose_stamp_ns - latest_raceline_stamp_ns_) / 1e6;
        const double odom_age_ms =
            (latest_odom_stamp_ns_ > 0)
                ? static_cast<double>(pose_stamp_ns - latest_odom_stamp_ns_) / 1e6
                : std::numeric_limits<double>::quiet_NaN();

        if (!latest_raceline_frame_.empty() &&
            !msg->header.frame_id.empty() &&
            latest_raceline_frame_ != msg->header.frame_id) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Frame mismatch: pose frame='%s', local_raceline frame='%s'",
                msg->header.frame_id.c_str(), latest_raceline_frame_.c_str());
        }

        if (std::abs(pose_raceline_age_ms) > max_pose_raceline_age_ms_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Stale local_raceline for pose: age=%.1f ms (limit=%.1f ms, drop=%s)",
                pose_raceline_age_ms, max_pose_raceline_age_ms_,
                drop_stale_raceline_ ? "true" : "false");
            if (drop_stale_raceline_) {
                return;
            }
        }

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
        const double ey = fp_to_double(mpc_state->e_y_fp);
        const double epsi = fp_to_double(mpc_state->e_psi_fp);
        const double ref_vx0 =
            (horizon_len > 0) ? fp_to_double(mpc_state->ref_vx_fp[0]) : 0.0;
        const double ref_kappa0 =
            (horizon_len > 0) ? fp_to_double(mpc_state->ref_kappa_fp[0]) : 0.0;
        const double ref_omega0 =
            (horizon_len > 0) ? fp_to_double(mpc_state->ref_omega_ref_fp[0]) : 0.0;
        const RefWaypoint& wp0 = local_raceline_.front();
        const RefWaypoint& wp1 =
            (local_raceline_.size() > 1) ? local_raceline_[1] : local_raceline_.front();

        /* Keep the EKF pose stamp as the pipeline monitor token. */

        if (mpc_state_csv_file_ != nullptr) {
            mpc_state_csv_idx_++;
            fprintf(mpc_state_csv_file_,
                "%lu,%lld,\"%s\",%lld,\"%s\",%.3f,%.3f,%lu,%zu,"
                "%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f\n",
                mpc_state_csv_idx_,
                static_cast<long long>(pose_stamp_ns),
                msg->header.frame_id.c_str(),
                static_cast<long long>(latest_raceline_stamp_ns_),
                latest_raceline_frame_.c_str(),
                pose_raceline_age_ms,
                odom_age_ms,
                latest_raceline_seq_,
                local_raceline_.size(),
                x, y, theta,
                wp0.x, wp0.y, wp0.psi, wp0.vx, wp0.kappa, wp0.left_bound, wp0.right_bound,
                wp1.x, wp1.y, wp1.psi,
                ey, epsi, latest_vx_, latest_vy_, latest_omega_, current_steering_angle_,
                ref_vx0, ref_kappa0, ref_omega0);
            fflush(mpc_state_csv_file_);
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[MpcState] ey=%.3f epsi=%.3f vx=%.3f ref_v0=%.3f kappa0=%.3f "
            "pose_frame=%s path_frame=%s path_age=%.1fms odom_age=%.1fms wp0=(%.3f,%.3f,psi=%.3f)",
            ey, epsi, latest_vx_, ref_vx0, ref_kappa0,
            msg->header.frame_id.c_str(), latest_raceline_frame_.c_str(),
            pose_raceline_age_ms, odom_age_ms, wp0.x, wp0.y, wp0.psi);

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
