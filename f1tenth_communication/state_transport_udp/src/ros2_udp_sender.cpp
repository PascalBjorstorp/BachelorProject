#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>

#include "state_transport_udp/state_packet.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace state_transport_udp {

/*===========================================================================
 * KD-Tree waypoint indexer
 *
 * The sender runs nearest-waypoint lookup at odometry rate, so query cost must
 * remain stable when trajectory size grows. KD-tree keeps lookup near O(log N)
 * and avoids worst-case full scans when localization jumps.
 *===========================================================================*/

struct Waypoint {
    double s;
    double x;
    double y;
    double psi;
    double kappa;
    double vx;
    double ax;
};

struct KDNode {
    double x;
    double y;
    size_t index;
};

class KDTree {
public:
    void build(const std::vector<Waypoint>& waypoints) {
        waypoints_ = waypoints;
        nodes_.clear();
        nodes_.reserve(waypoints.size());

        for (size_t i = 0; i < waypoints.size(); ++i) {
            nodes_.push_back({waypoints[i].x, waypoints[i].y, i});
        }

        buildRecursive(0, nodes_.size(), 0);
    }

    size_t findNearest(double x, double y) const {
        if (nodes_.empty()) {
            return 0;
        }

        size_t best_idx = 0;
        double best_dist = std::numeric_limits<double>::max();
        searchRecursive(0, nodes_.size(), 0, x, y, best_idx, best_dist);
        return nodes_[best_idx].index;
    }

    const Waypoint& getWaypoint(size_t idx) const {
        return waypoints_[idx];
    }

    size_t size() const {
        return waypoints_.size();
    }

private:
    void buildRecursive(size_t start, size_t end, int depth) {
        if (end - start <= 1) {
            return;
        }

        const size_t mid = start + (end - start) / 2;
        if (depth % 2 == 0) {
            std::nth_element(nodes_.begin() + static_cast<std::ptrdiff_t>(start),
                             nodes_.begin() + static_cast<std::ptrdiff_t>(mid),
                             nodes_.begin() + static_cast<std::ptrdiff_t>(end),
                             [](const KDNode& a, const KDNode& b) { return a.x < b.x; });
        } else {
            std::nth_element(nodes_.begin() + static_cast<std::ptrdiff_t>(start),
                             nodes_.begin() + static_cast<std::ptrdiff_t>(mid),
                             nodes_.begin() + static_cast<std::ptrdiff_t>(end),
                             [](const KDNode& a, const KDNode& b) { return a.y < b.y; });
        }

        buildRecursive(start, mid, depth + 1);
        buildRecursive(mid + 1, end, depth + 1);
    }

    void searchRecursive(size_t start,
                         size_t end,
                         int depth,
                         double x,
                         double y,
                         size_t& best_idx,
                         double& best_dist) const {
        if (start >= end) {
            return;
        }

        const size_t mid = start + (end - start) / 2;
        const KDNode& node = nodes_[mid];

        const double dx = x - node.x;
        const double dy = y - node.y;
        const double dist = dx * dx + dy * dy;
        if (dist < best_dist) {
            best_dist = dist;
            best_idx = mid;
        }

        const double split_val = (depth % 2 == 0) ? node.x : node.y;
        const double query_val = (depth % 2 == 0) ? x : y;
        const double diff = query_val - split_val;

        if (diff < 0) {
            searchRecursive(start, mid, depth + 1, x, y, best_idx, best_dist);
            if (diff * diff < best_dist) {
                searchRecursive(mid + 1, end, depth + 1, x, y, best_idx, best_dist);
            }
        } else {
            searchRecursive(mid + 1, end, depth + 1, x, y, best_idx, best_dist);
            if (diff * diff < best_dist) {
                searchRecursive(start, mid, depth + 1, x, y, best_idx, best_dist);
            }
        }
    }

    std::vector<KDNode> nodes_;
    std::vector<Waypoint> waypoints_;
};

class Ros2UdpSender : public rclcpp::Node {
public:
    Ros2UdpSender() : Node("ros2_udp_sender") {
        declare_parameter<std::string>("trajectory_file", "");
        declare_parameter<std::string>("odom_topic", "/ego_racecar/odom");
        declare_parameter<std::string>("servo_topic", "/sensors/servo_position_command");
        declare_parameter<double>("wheelbase", 0.324);
        declare_parameter<double>("servo_gain", -0.7284);
        declare_parameter<double>("servo_offset", 0.55);
        declare_parameter<double>("steering_correction_c2", 0.589566);
        declare_parameter<double>("steering_correction_c1", 0.918061);
        declare_parameter<double>("steering_correction_c0", 0.001490);
        declare_parameter<int>("forward_lookahead", 3);
        declare_parameter<int>("horizon", static_cast<int>(MPC_HORIZON));
        declare_parameter<bool>("interpolate_horizon", true);
        declare_parameter<double>("horizon_step_m", 0.0);

        declare_parameter<std::string>("dest_ip", "192.168.50.182");
        declare_parameter<int>("dest_port", 49000);

        const std::string trajectory_file = get_parameter("trajectory_file").as_string();
        const std::string odom_topic = get_parameter("odom_topic").as_string();
        const std::string servo_topic = get_parameter("servo_topic").as_string();
        const std::string dest_ip = get_parameter("dest_ip").as_string();
        const int dest_port = get_parameter("dest_port").as_int();

        wheelbase_ = get_parameter("wheelbase").as_double();
        servo_gain_ = get_parameter("servo_gain").as_double();
        servo_offset_ = get_parameter("servo_offset").as_double();
        steer_c2_ = get_parameter("steering_correction_c2").as_double();
        steer_c1_ = get_parameter("steering_correction_c1").as_double();
        steer_c0_ = get_parameter("steering_correction_c0").as_double();
        forward_lookahead_ = static_cast<int>(get_parameter("forward_lookahead").as_int());
        interpolate_horizon_ = get_parameter("interpolate_horizon").as_bool();
        horizon_step_m_ = get_parameter("horizon_step_m").as_double();
        const int horizon_param = get_parameter("horizon").as_int();
        if (horizon_param != static_cast<int>(MPC_HORIZON)) {
            RCLCPP_WARN(get_logger(),
                        "horizon=%d requested, but packet format uses fixed MPC_HORIZON=%zu. Forcing %zu.",
                        horizon_param,
                        static_cast<size_t>(MPC_HORIZON),
                        static_cast<size_t>(MPC_HORIZON));
        }

        if (trajectory_file.empty() || !loadTrajectory(trajectory_file)) {
            throw std::runtime_error("Failed to load trajectory_file for ros2_udp_sender");
        }

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

        auto qos = rclcpp::QoS(1).best_effort().durability_volatile();
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic,
            qos,
            std::bind(&Ros2UdpSender::odomCallback, this, std::placeholders::_1));

        if (!servo_topic.empty()) {
            servo_sub_ = create_subscription<std_msgs::msg::Float64>(
                servo_topic,
                qos,
                std::bind(&Ros2UdpSender::servoCallback, this, std::placeholders::_1));
        }

        RCLCPP_INFO(get_logger(),
                    "ROS2->UDP sender ready: odom=%s traj_points=%zu interp=%s step=%.3f -> %s:%d",
                    odom_topic.c_str(),
                    kdtree_.size(),
                    interpolate_horizon_ ? "on" : "off",
                    horizon_step_m_,
                    dest_ip.c_str(),
                    dest_port);
    }

    ~Ros2UdpSender() override {
        if (sock_fd_ >= 0) {
            ::close(sock_fd_);
        }
    }

private:
    static int32_t toFp(double v) {
        constexpr double kScale = 65536.0;
        if (!std::isfinite(v)) {
            return 0;
        }
        return static_cast<int32_t>(v >= 0.0 ? v * kScale + 0.5 : v * kScale - 0.5);
    }

    bool loadTrajectory(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            return false;
        }

        std::vector<Waypoint> waypoints;
        std::string line;
        std::getline(file, line);
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string token;
            Waypoint wp{};

            std::getline(ss, token, ','); wp.s = std::stod(token);
            std::getline(ss, token, ','); wp.x = std::stod(token);
            std::getline(ss, token, ','); wp.y = std::stod(token);
            std::getline(ss, token, ','); wp.psi = std::stod(token);
            std::getline(ss, token, ','); wp.kappa = std::stod(token);
            std::getline(ss, token, ','); wp.vx = std::stod(token);
            std::getline(ss, token, ','); wp.ax = std::stod(token);

            waypoints.push_back(wp);
        }

        if (waypoints.empty()) {
            return false;
        }

        trajectory_ = waypoints;
        kdtree_.build(waypoints);

        total_length_m_ = 0.0;
        average_spacing_m_ = 0.0;
        if (trajectory_.size() > 1) {
            if (trajectory_.back().s > trajectory_.front().s) {
                total_length_m_ = trajectory_.back().s - trajectory_.front().s;
            }
            for (size_t i = 1; i < trajectory_.size(); ++i) {
                average_spacing_m_ += std::max(0.0, trajectory_[i].s - trajectory_[i - 1].s);
            }
            average_spacing_m_ /= static_cast<double>(trajectory_.size() - 1);
        }

        if (horizon_step_m_ <= 0.0) {
            horizon_step_m_ = (average_spacing_m_ > 1e-4) ? average_spacing_m_ : 0.2;
        }
        return true;
    }

    static double normalizeAngle(double a) {
        constexpr double kPi = 3.14159265358979323846;
        while (a > kPi) {
            a -= 2.0 * kPi;
        }
        while (a < -kPi) {
            a += 2.0 * kPi;
        }
        return a;
    }

    static double lerpAngle(double a0, double a1, double t) {
        const double d = normalizeAngle(a1 - a0);
        return normalizeAngle(a0 + d * t);
    }

    Waypoint sampleByArcLength(double s_query) const {
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

        if (it == trajectory_.end()) {
            const Waypoint& w0 = trajectory_.back();
            const Waypoint& w1 = trajectory_.front();
            const double s1 = w1.s + total_length_m_;
            const double denom = s1 - w0.s;
            const double t = (denom > 1e-9) ? ((s - w0.s) / denom) : 0.0;
            Waypoint out{};
            out.s = s;
            out.x = w0.x + (w1.x - w0.x) * t;
            out.y = w0.y + (w1.y - w0.y) * t;
            out.psi = lerpAngle(w0.psi, w1.psi, t);
            out.kappa = w0.kappa + (w1.kappa - w0.kappa) * t;
            out.vx = w0.vx + (w1.vx - w0.vx) * t;
            out.ax = w0.ax + (w1.ax - w0.ax) * t;
            return out;
        }

        const Waypoint& w1 = *it;
        const Waypoint& w0 = *(it - 1);
        const double denom = w1.s - w0.s;
        const double t = (denom > 1e-9) ? ((s - w0.s) / denom) : 0.0;

        Waypoint out{};
        out.s = s;
        out.x = w0.x + (w1.x - w0.x) * t;
        out.y = w0.y + (w1.y - w0.y) * t;
        out.psi = lerpAngle(w0.psi, w1.psi, t);
        out.kappa = w0.kappa + (w1.kappa - w0.kappa) * t;
        out.vx = w0.vx + (w1.vx - w0.vx) * t;
        out.ax = w0.ax + (w1.ax - w0.ax) * t;
        return out;
    }

    void servoCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        const double corrected = (msg->data - servo_offset_) / servo_gain_;
        const double abs_corr = std::abs(corrected);
        if (steer_c2_ != 0.0) {
            const double disc = steer_c1_ * steer_c1_ - 4.0 * steer_c2_ * (steer_c0_ - abs_corr);
            if (disc >= 0.0) {
                const double t = (-steer_c1_ + std::sqrt(disc)) / (2.0 * steer_c2_);
                current_steering_angle_ = std::copysign(t, corrected);
            } else {
                current_steering_angle_ = corrected;
            }
        } else {
            current_steering_angle_ = corrected;
        }
        has_servo_feedback_ = true;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (kdtree_.size() == 0) {
            return;
        }

        const double x = msg->pose.pose.position.x;
        const double y = msg->pose.pose.position.y;

        const double qx = msg->pose.pose.orientation.x;
        const double qy = msg->pose.pose.orientation.y;
        const double qz = msg->pose.pose.orientation.z;
        const double qw = msg->pose.pose.orientation.w;
        const double theta = std::atan2(2.0 * (qw * qz + qx * qy),
                                        1.0 - 2.0 * (qy * qy + qz * qz));

        const double vx = msg->twist.twist.linear.x;
        const double vy = msg->twist.twist.linear.y;
        const double omega = msg->twist.twist.angular.z;

        constexpr double kWheelRadius = 0.0545;
        const double wheel_speed = (vx > 0.01) ? (vx / kWheelRadius) : 0.0;

        double steering_angle = current_steering_angle_;
        if (!has_servo_feedback_ && std::abs(vx) > 0.1) {
            steering_angle = std::atan2(wheelbase_ * omega, vx);
        }

        // Choose nearest waypoint, then bias forward using heading projection
        // to avoid selecting points slightly behind the vehicle at speed.
        size_t waypoint_idx = kdtree_.findNearest(x, y);
        {
            const double cos_theta = std::cos(theta);
            const double sin_theta = std::sin(theta);
            const size_t n = kdtree_.size();
            size_t best_idx = waypoint_idx;
            double best_dist = std::numeric_limits<double>::max();

            for (int i = 0; i <= forward_lookahead_; ++i) {
                const size_t check_idx = (waypoint_idx + static_cast<size_t>(i)) % n;
                const auto& wp = kdtree_.getWaypoint(check_idx);
                const double dx_wp = wp.x - x;
                const double dy_wp = wp.y - y;
                const double ahead = dx_wp * cos_theta + dy_wp * sin_theta;
                if (ahead >= 0.0) {
                    const double dist = dx_wp * dx_wp + dy_wp * dy_wp;
                    if (dist < best_dist) {
                        best_dist = dist;
                        best_idx = check_idx;
                    }
                }
            }
            waypoint_idx = best_idx;
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

        packet.x_fp = toFp(x);
        packet.y_fp = toFp(y);
        packet.theta_fp = toFp(theta);
        packet.velocity_fp = toFp(vx);
        packet.vy_fp = toFp(vy);
        packet.omega_fp = toFp(omega);
        packet.wheel_speed_fp = toFp(wheel_speed);
        packet.steering_angle_fp = toFp(steering_angle);
        packet.waypoint_index = static_cast<uint32_t>(waypoint_idx);
        packet.horizon_length = static_cast<uint32_t>(MPC_HORIZON);

        if (interpolate_horizon_ && !trajectory_.empty()) {
            const double base_s = kdtree_.getWaypoint(waypoint_idx).s;
            for (size_t i = 0; i < MPC_HORIZON; ++i) {
                const Waypoint wp = sampleByArcLength(base_s + horizon_step_m_ * static_cast<double>(i));
                packet.ref_x_fp[i] = toFp(wp.x);
                packet.ref_y_fp[i] = toFp(wp.y);
                packet.ref_psi_fp[i] = toFp(wp.psi);
                packet.ref_vx_fp[i] = toFp(wp.vx);
                packet.ref_kappa_fp[i] = toFp(wp.kappa);
                packet.ref_ax_fp[i] = toFp(wp.ax);
            }
        } else {
            const size_t n = kdtree_.size();
            for (size_t i = 0; i < MPC_HORIZON; ++i) {
                const size_t idx = (waypoint_idx + i) % n;
                const auto& wp = kdtree_.getWaypoint(idx);
                packet.ref_x_fp[i] = toFp(wp.x);
                packet.ref_y_fp[i] = toFp(wp.y);
                packet.ref_psi_fp[i] = toFp(wp.psi);
                packet.ref_vx_fp[i] = toFp(wp.vx);
                packet.ref_kappa_fp[i] = toFp(wp.kappa);
                packet.ref_ax_fp[i] = toFp(wp.ax);
            }
        }

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
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "UDP send short/failed: %ld", static_cast<long>(sent));
        }
    }

    int sock_fd_{-1};
    sockaddr_in dest_addr_{};
    uint32_t sequence_{0};

    double current_steering_angle_{0.0};
    bool has_servo_feedback_{false};
    double wheelbase_{0.324};
    double servo_gain_{-0.7284};
    double servo_offset_{0.55};
    double steer_c2_{0.589566};
    double steer_c1_{0.918061};
    double steer_c0_{0.001490};
    int forward_lookahead_{3};
    bool interpolate_horizon_{true};
    double horizon_step_m_{0.0};
    double total_length_m_{0.0};
    double average_spacing_m_{0.2};

    std::vector<Waypoint> trajectory_;
    KDTree kdtree_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr servo_sub_;
};

}  // namespace state_transport_udp

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<state_transport_udp::Ros2UdpSender>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
