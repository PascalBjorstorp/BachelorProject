/**
 * @file ros2_udp_sender.cpp
 * @brief Jetson-side ROS2 to UDP state packet sender.
 * @details Subscribes to pose/odometry topics and /local_raceline, builds the
 *          MPC horizon reference (V2 layout) and transmits fixed-size UDP state
 *          packets to Kria.
 * @dependencies rclcpp, nav_msgs, geometry_msgs, std_msgs,
 *               state_transport_udp/state_packet.hpp, POSIX sockets
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include "state_transport_udp/state_packet.hpp"
#include "mpc_fpga_constants.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace state_transport_udp {

struct RefWaypoint {
    double s = 0.0;
    double x = 0.0;
    double y = 0.0;
    double psi = 0.0;
    double vx = MPC_FPGA_MIN_VEL_MPS;
    double kappa = 0.0;
    double left_bound = 0.5;
    double right_bound = 0.5;
};

struct SampledRefPoint {
    double x = 0.0;
    double y = 0.0;
    double psi = 0.0;
    double vx = MPC_FPGA_MIN_VEL_MPS;
    double kappa = 0.0;
    double left_bound = 0.5;
    double right_bound = 0.5;
};

struct FrenetErrorsFp {
    int32_t e_y_fp = 0;
    int32_t e_psi_fp = 0;
};

class Ros2UdpSender final : public rclcpp::Node {
public:
    Ros2UdpSender() : Node("ros2_udp_sender") {
        declare_parameter<std::string>("odom_topic", "/ego_racecar/odom");
        declare_parameter<std::string>("pose_topic", "/ekf_pose");
        declare_parameter<std::string>("raceline_topic", "/local_raceline");
        declare_parameter<std::string>("servo_topic", "/sensors/servo_position_command");
        declare_parameter<std::string>("dest_ip", "10.23.0.2");
        declare_parameter<int>("dest_port", 49000);

        const std::string odom_topic = get_parameter("odom_topic").as_string();
        const std::string pose_topic = get_parameter("pose_topic").as_string();
        const std::string raceline_topic = get_parameter("raceline_topic").as_string();
        const std::string servo_topic = get_parameter("servo_topic").as_string();
        const std::string dest_ip = get_parameter("dest_ip").as_string();
        const int dest_port = get_parameter("dest_port").as_int();

        if (!is_packet_layout_valid()) {
            throw std::runtime_error("StatePacket layout invalid");
        }

        sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_fd_ < 0) {
            throw std::runtime_error("Failed to create UDP socket");
        }

        std::memset(&dest_addr_, 0, sizeof(dest_addr_));
        dest_addr_.sin_family = AF_INET;
        dest_addr_.sin_port = htons(static_cast<uint16_t>(dest_port));
        if (::inet_pton(AF_INET, dest_ip.c_str(), &dest_addr_.sin_addr) != 1) {
            throw std::runtime_error("Invalid dest_ip");
        }

        auto sub_qos = rclcpp::QoS(10).reliable().durability_volatile();
        raceline_sub_ = create_subscription<nav_msgs::msg::Path>(
            raceline_topic, sub_qos,
            std::bind(&Ros2UdpSender::raceline_callback, this, std::placeholders::_1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, sub_qos,
            std::bind(&Ros2UdpSender::odom_callback, this, std::placeholders::_1));

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            pose_topic, sub_qos,
            std::bind(&Ros2UdpSender::pose_callback, this, std::placeholders::_1));

        if (!servo_topic.empty()) {
            servo_sub_ = create_subscription<std_msgs::msg::Float64>(
                servo_topic, sub_qos,
                std::bind(&Ros2UdpSender::servo_callback, this, std::placeholders::_1));
        }

    }

    ~Ros2UdpSender() override {
        if (sock_fd_ >= 0) {
            ::close(sock_fd_);
            sock_fd_ = -1;
        }
    }

private:
    static int32_t to_fp(double v) {
        constexpr double kScale = MPC_FPGA_QP_SCALE_F64;
        if (!std::isfinite(v)) return 0;
        return static_cast<int32_t>(v >= 0.0 ? v * kScale + 0.5 : v * kScale - 0.5);
    }

    static double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    FrenetErrorsFp compute_frenet_errors(double x, double y, double theta) const {
        if (local_raceline_.size() < 2) {
            return {};
        }

        /* Local raceline is republished each cycle anchored at the car, so
         * segment [0,1] is the current reference origin. Project onto it
         * directly instead of an unconstrained global nearest-segment scan
         * (which costs per cycle and can snap to an offset segment, biasing
         * e_y). Matches the MPC hardware node's closest=0 anchoring. */
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

        const double path_psi = a.psi + t * normalize_angle(b.psi - a.psi);
        const double path_x = a.x + t * abx;
        const double path_y = a.y + t * aby;
        const double dx = x - path_x;
        const double dy = y - path_y;

        const double best_e_y = -std::sin(path_psi) * dx + std::cos(path_psi) * dy;
        const double best_e_psi = normalize_angle(theta - path_psi);

        return {to_fp(best_e_y), to_fp(best_e_psi)};
    }

    SampledRefPoint sample_raceline_by_s(double target_s) const {
        SampledRefPoint out{};
        if (local_raceline_.empty()) return out;

        const double s0 = local_raceline_.front().s;
        const double sN = local_raceline_.back().s;
        if (target_s <= s0) {
            const auto& wp = local_raceline_.front();
            out.x = wp.x;
            out.y = wp.y;
            out.psi = wp.psi;
            out.vx = wp.vx;
            out.kappa = wp.kappa;
            out.left_bound = wp.left_bound;
            out.right_bound = wp.right_bound;
            return out;
        }
        if (target_s >= sN) {
            const auto& wp = local_raceline_.back();
            out.x = wp.x;
            out.y = wp.y;
            out.psi = wp.psi;
            out.vx = wp.vx;
            out.kappa = wp.kappa;
            out.left_bound = wp.left_bound;
            out.right_bound = wp.right_bound;
            return out;
        }

        size_t idx = 0;
        for (size_t i = 0; i + 1 < local_raceline_.size(); ++i) {
            if (local_raceline_[i].s <= target_s && local_raceline_[i + 1].s >= target_s) {
                idx = i;
                break;
            }
        }

        const auto& a = local_raceline_[idx];
        const auto& b = local_raceline_[idx + 1];
        const double denom = b.s - a.s;
        const double t = denom > 1e-9 ? (target_s - a.s) / denom : 0.0;

        auto lerp = [](double v0, double v1, double tval) { return v0 + (v1 - v0) * tval; };
        auto lerp_angle = [](double a0, double a1, double tval) {
            double d = a1 - a0;
            while (d > M_PI) d -= 2.0 * M_PI;
            while (d < -M_PI) d += 2.0 * M_PI;
            return a0 + d * tval;
        };

        out.x = lerp(a.x, b.x, t);
        out.y = lerp(a.y, b.y, t);
        out.psi = lerp_angle(a.psi, b.psi, t);
        out.vx = lerp(a.vx, b.vx, t);
        out.kappa = lerp(a.kappa, b.kappa, t);
        out.left_bound = lerp(a.left_bound, b.left_bound, t);
        out.right_bound = lerp(a.right_bound, b.right_bound, t);
        return out;
    }

    void build_horizon_from_raceline(StatePacket& packet) const {
        if (local_raceline_.empty()) {
            packet.horizon_length = 0;
            return;
        }

        const size_t horizon = MPC_HORIZON;
        const double dt = MPC_FPGA_PREDICTION_DT_S;

        double v_ref_base = local_raceline_[0].vx;
        if (v_ref_base <= 0.0) {
            v_ref_base = std::clamp(
                static_cast<double>(MPC_FPGA_MAX_VEL_MPS) * 0.7,
                static_cast<double>(MPC_FPGA_MIN_VEL_MPS),
                static_cast<double>(MPC_FPGA_MAX_VEL_MPS));
        }
        const double ds = std::max(1e-3, v_ref_base * dt);

        packet.horizon_length = static_cast<uint32_t>(horizon);
        for (size_t step = 0; step < horizon; ++step) {
            const double target_s = local_raceline_[0].s + ds * static_cast<double>(step);
            const SampledRefPoint wp = sample_raceline_by_s(target_s);

            packet.ref_ey_fp[step] = to_fp(0.0);
            packet.ref_epsi_fp[step] = to_fp(0.0);
            packet.ref_vx_fp[step] = to_fp(wp.vx);
            packet.ref_vy_fp[step] = to_fp(0.0);
            packet.ref_omega_ref_fp[step] = to_fp(wp.vx * wp.kappa);
            packet.ref_kappa_fp[step] = to_fp(wp.kappa);
            packet.ref_left_bound_fp[step] = to_fp(wp.left_bound);
            packet.ref_right_bound_fp[step] = to_fp(wp.right_bound);
        }
    }

    void raceline_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->poses.empty()) {
            return;
        }

        std::vector<RefWaypoint> new_raceline;
        double cumulative_s = 0.0;
        double prev_x = 0.0, prev_y = 0.0;
        const size_t waypoint_count = msg->poses.size();
        new_raceline.reserve(waypoint_count);

        for (size_t i = 0; i < waypoint_count; ++i) {
            const auto& pose = msg->poses[i];
            const auto& pos = pose.pose.position;
            const auto& ori = pose.pose.orientation;

            RefWaypoint wp;
            wp.x = pos.x;
            wp.y = pos.y;

            if (i > 0) {
                const double dx = wp.x - prev_x;
                const double dy = wp.y - prev_y;
                cumulative_s += std::hypot(dx, dy);
            }
            wp.s = cumulative_s;

            double v_ref = std::abs(pos.z);
            if (!std::isfinite(v_ref) || v_ref < MPC_FPGA_MIN_VEL_MPS) v_ref = MPC_FPGA_MIN_VEL_MPS;
            if (v_ref > MPC_FPGA_MAX_VEL_MPS) v_ref = MPC_FPGA_MAX_VEL_MPS;
            wp.vx = v_ref;

            double left_bound = ori.x;
            double right_bound = ori.y;
            if (!std::isfinite(left_bound) || left_bound <= 0.0) left_bound = 0.5;
            if (!std::isfinite(right_bound) || right_bound <= 0.0) right_bound = 0.5;
            wp.left_bound = left_bound;
            wp.right_bound = right_bound;

            new_raceline.push_back(wp);
            prev_x = wp.x;
            prev_y = wp.y;
        }

        for (size_t i = 0; i < waypoint_count; ++i) {
            const size_t i_prev = (i == 0) ? 0 : (i - 1);
            const size_t i_next = (i + 1 < waypoint_count) ? (i + 1) : (waypoint_count - 1);
            const double dx = new_raceline[i_next].x - new_raceline[i_prev].x;
            const double dy = new_raceline[i_next].y - new_raceline[i_prev].y;

            double heading = 0.0;
            if ((dx * dx + dy * dy) > 1e-12) {
                heading = std::atan2(dy, dx);
            } else if (i > 0) {
                heading = new_raceline[i - 1].psi;
            }
            new_raceline[i].psi = heading;
        }

        if (waypoint_count >= 3) {
            for (size_t i = 1; i + 1 < waypoint_count; ++i) {
                const double dpsi = normalize_angle(new_raceline[i + 1].psi - new_raceline[i - 1].psi);
                const double ds = new_raceline[i + 1].s - new_raceline[i - 1].s;
                const double ds_safe = (ds > 1e-6) ? ds : 1e-6;
                new_raceline[i].kappa = dpsi / ds_safe;
            }
            new_raceline[0].kappa = new_raceline[1].kappa;
            new_raceline[waypoint_count - 1].kappa = new_raceline[waypoint_count - 2].kappa;
        }

        local_raceline_ = std::move(new_raceline);
    }

    void servo_callback(const std_msgs::msg::Float64::SharedPtr msg) {
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
        has_servo_feedback_ = true;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_vx_ = msg->twist.twist.linear.x;
        latest_vy_ = msg->twist.twist.linear.y;
        latest_omega_ = msg->twist.twist.angular.z;
        has_odom_dynamics_ = std::isfinite(latest_vx_) && std::isfinite(latest_vy_) && std::isfinite(latest_omega_);
    }

    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        send_state_packet(msg->pose.pose.position.x,
                          msg->pose.pose.position.y,
                          msg->pose.pose.orientation.x,
                          msg->pose.pose.orientation.y,
                          msg->pose.pose.orientation.z,
                          msg->pose.pose.orientation.w,
                          msg->header.stamp.sec,
                          msg->header.stamp.nanosec);
    }

    void send_state_packet(double x,
                           double y,
                           double qx,
                           double qy,
                           double qz,
                           double qw,
                           int32_t source_stamp_sec,
        uint32_t source_stamp_nanosec) {
        if (local_raceline_.empty()) return;
        if (!has_odom_dynamics_) {
            return;
        }

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(qx) ||
            !std::isfinite(qy) || !std::isfinite(qz) || !std::isfinite(qw)) {
            return;
        }

        const double theta = std::atan2(2.0 * (qw * qz + qx * qy),
                                        1.0 - 2.0 * (qy * qy + qz * qz));

        const double vx = latest_vx_;
        const double vy = latest_vy_;
        const double omega = latest_omega_;

        double steering_angle = current_steering_angle_;
        if (!has_servo_feedback_ && std::abs(vx) > 0.1) {
            steering_angle = std::atan2(static_cast<double>(MPC_FPGA_WHEELBASE_M) * omega, vx);
        }

        StatePacket packet{};
        packet.magic = PACKET_MAGIC;
        packet.version = PACKET_VERSION;
        packet.flags = 0;
        packet.sequence = sequence_++;
        packet.sender_time_ms = static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count() & 0xFFFFFFFFu);
        packet.sender_mono_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
        packet.source_stamp_sec = source_stamp_sec;
        packet.source_stamp_nanosec = source_stamp_nanosec;

        const FrenetErrorsFp errors = compute_frenet_errors(x, y, theta);
        packet.e_y_fp = errors.e_y_fp;
        packet.e_psi_fp = errors.e_psi_fp;
        packet.velocity_fp = to_fp(vx);
        packet.vy_fp = to_fp(vy);
        packet.omega_fp = to_fp(omega);
        packet.steering_angle_fp = to_fp(steering_angle);

        build_horizon_from_raceline(packet);

        packet.crc32 = 0;
        packet.crc32 = crc32_ieee(reinterpret_cast<const uint8_t*>(&packet),
                                  sizeof(StatePacket) - sizeof(packet.crc32));

        const ssize_t sent = ::sendto(
            sock_fd_,
            &packet,
            sizeof(packet),
            0,
            reinterpret_cast<const sockaddr*>(&dest_addr_),
            sizeof(dest_addr_));

        if (sent != static_cast<ssize_t>(sizeof(packet))) {
            return;
        }
    }

    int sock_fd_{-1};
    sockaddr_in dest_addr_{};
    uint32_t sequence_{0};

    std::vector<RefWaypoint> local_raceline_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr raceline_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr servo_sub_;

    double current_steering_angle_{0.0};
    bool has_servo_feedback_{false};

    double latest_vx_{0.0};
    double latest_vy_{0.0};
    double latest_omega_{0.0};
    bool has_odom_dynamics_{false};
};

}  // namespace state_transport_udp

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<state_transport_udp::Ros2UdpSender>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
