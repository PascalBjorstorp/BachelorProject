/**
 * @file ros2_udp_sender.cpp
 * @brief Jetson-side ROS2 to UDP state packet sender.
 * @details Subscribes to pose/odometry topics, builds reference horizon from
 *          trajectory, and transmits fixed-size state packets to Kria.
 * @dependencies rclcpp, nav_msgs, geometry_msgs, std_msgs,
 *               state_transport_udp/state_packet.hpp, POSIX sockets
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include "state_transport_udp/state_packet.hpp"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <sys/stat.h>
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
    double left_bound;
    double right_bound;
};

struct KDNode {
    double x;
    double y;
    size_t index;
};

class KDTree {
public:
    /**
     * @brief Build KD-tree index from waypoint coordinates.
     * @param waypoints Waypoint vector to index.
     * @return None
     */
    void build(const std::vector<Waypoint>& waypoints) {
        waypoints_ = waypoints;
        nodes_.clear();
        nodes_.reserve(waypoints.size());

        for (size_t i = 0; i < waypoints.size(); ++i) {
            nodes_.push_back({waypoints[i].x, waypoints[i].y, i});
        }

        buildRecursive(0, nodes_.size(), 0);
    }

    /**
     * @brief Query nearest waypoint index for a 2D point.
     * @param x Query x coordinate.
     * @param y Query y coordinate.
     * @return Index of nearest waypoint in stored waypoint array.
     */
    size_t findNearest(double x, double y) const {
        if (nodes_.empty()) {
            return 0;
        }

        size_t best_idx = 0;
        double best_dist = std::numeric_limits<double>::max();
        searchRecursive(0, nodes_.size(), 0, x, y, best_idx, best_dist);
        return nodes_[best_idx].index;
    }

    /**
     * @brief Access waypoint by index.
     * @param idx Waypoint index in stored array.
     * @return Constant reference to waypoint at `idx`.
     */
    const Waypoint& getWaypoint(size_t idx) const {
        return waypoints_[idx];
    }

    /**
     * @brief Get number of stored waypoints.
     * @return Current waypoint count.
     */
    size_t size() const {
        return waypoints_.size();
    }

private:
    /**
     * @brief Recursively partition KD-tree node array.
     * @param start Inclusive start index.
     * @param end Exclusive end index.
     * @param depth Current recursion depth used for split axis.
     * @return None
     */
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

    /**
     * @brief Recursively search nearest neighbor candidate in KD-tree.
     * @param start Inclusive start index.
     * @param end Exclusive end index.
     * @param depth Current recursion depth used for split axis.
     * @param x Query x coordinate.
     * @param y Query y coordinate.
     * @param best_idx In/out current best node index.
     * @param best_dist In/out current best squared distance.
     * @return None
     */
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
    /**
     * @brief Construct ROS2-to-UDP sender node and initialize runtime interfaces.
     * @return None
     */
    Ros2UdpSender() : Node("ros2_udp_sender") {
        declare_parameter<std::string>(
            "trajectory_file",
            "/home/f1tenth/BachelorProject/f1tenth_planning/trajectories/my_track_raceline.csv");
        declare_parameter<std::string>("odom_topic", "/ego_racecar/odom");
        declare_parameter<std::string>("pose_topic", "/ekf_pose");
        declare_parameter<std::string>("servo_topic", "/sensors/servo_position_command");
        declare_parameter<int>("horizon", static_cast<int>(MPC_HORIZON));
        declare_parameter<bool>("interpolate_horizon", true);
        declare_parameter<double>("horizon_step_m", 0.0);

        declare_parameter<std::string>("dest_ip", "10.23.0.2");
        declare_parameter<int>("dest_port", 49000);

        const std::string trajectory_file = get_parameter("trajectory_file").as_string();
        const std::string odom_topic = get_parameter("odom_topic").as_string();
        const std::string pose_topic = get_parameter("pose_topic").as_string();
        const std::string servo_topic = get_parameter("servo_topic").as_string();
        const std::string dest_ip = get_parameter("dest_ip").as_string();
        const int dest_port = get_parameter("dest_port").as_int();

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

        auto sub_qos = rclcpp::QoS(10).reliable().durability_volatile();
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic,
            sub_qos,
            std::bind(&Ros2UdpSender::odomCallback, this, std::placeholders::_1));

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            pose_topic,
            sub_qos,
            std::bind(&Ros2UdpSender::poseCallback, this, std::placeholders::_1));

        if (!servo_topic.empty()) {
            servo_sub_ = create_subscription<std_msgs::msg::Float64>(
                servo_topic,
                sub_qos,
                std::bind(&Ros2UdpSender::servoCallback, this, std::placeholders::_1));
        }

        startup_diag_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&Ros2UdpSender::startupDiagnostics, this));

        RCLCPP_INFO(get_logger(),
                "ROS2->UDP sender ready: odom cache=%s pose trigger=%s traj_points=%zu interp=%s step=%.3f -> %s:%d",
                    odom_topic.c_str(),
                pose_topic.c_str(),
                    kdtree_.size(),
                    interpolate_horizon_ ? "on" : "off",
                    horizon_step_m_,
                    dest_ip.c_str(),
                    dest_port);

                openSendStatsCsv();
    }

    /**
     * @brief Destroy sender node and close UDP socket.
     * @return None
     */
    ~Ros2UdpSender() override {
        if (sock_fd_ >= 0) {
            ::close(sock_fd_);
        }
        if (send_stats_csv_ != nullptr) {
            std::fclose(send_stats_csv_);
            send_stats_csv_ = nullptr;
        }
    }

private:
    /**
     * @brief Convert floating-point value to raw QP fixed-point.
     * @param v Floating-point input value.
     * @return Raw QP integer representation of `v`.
     */
    static int32_t toFp(double v) {
        constexpr double kScale = MPC_FPGA_QP_SCALE_F64;
        if (!std::isfinite(v)) {
            return 0;
        }
        return static_cast<int32_t>(v >= 0.0 ? v * kScale + 0.5 : v * kScale - 0.5);
    }

    void openSendStatsCsv() {
        const char* log_dir = "log";
        ::mkdir(log_dir, 0755);

        const std::time_t now = std::time(nullptr);
        struct tm tm_now;
        localtime_r(&now, &tm_now);

        char timestamp[64];
        std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &tm_now);

        char csv_path[512];
        std::snprintf(csv_path, sizeof(csv_path), "%s/ros2_udp_sender_%s.csv", log_dir, timestamp);

        send_stats_csv_ = std::fopen(csv_path, "w");
        if (send_stats_csv_ == nullptr) {
            RCLCPP_WARN(get_logger(), "Failed to open sender stats CSV file: %s", csv_path);
            return;
        }

        std::fprintf(send_stats_csv_, "idx,send_time_us\n");
        std::fflush(send_stats_csv_);
        RCLCPP_INFO(get_logger(), "Sender stats CSV log: %s", csv_path);
    }

    /**
     * @brief Load trajectory CSV and build KD-tree plus spacing metadata.
     * @param filepath Path to trajectory CSV file.
     * @return true when trajectory data loads and validates successfully.
     */
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
            std::vector<double> cols;
            while (std::getline(ss, token, ',')) {
                if (token.empty()) {
                    continue;
                }
                cols.push_back(std::stod(token));
            }

            if (cols.size() < 7) {
                continue;
            }

            if (cols.size() < 9) {
                RCLCPP_ERROR(get_logger(),
                             "Trajectory row missing left/right bounds. Expected columns: "
                             "s,x,y,psi,kappa,vx,ax,left_bound,right_bound");
                return false;
            }

            Waypoint wp{};
            wp.s = cols[0];
            wp.x = cols[1];
            wp.y = cols[2];
            wp.psi = cols[3];
            wp.kappa = cols[4];
            wp.vx = cols[5];
            wp.ax = cols[6];
            wp.left_bound = cols[7];
            wp.right_bound = cols[8];

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

    /**
     * @brief Normalize angle to [-pi, pi] range.
     * @param a Input angle in radians.
     * @return Normalized angle in radians.
     */
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

    /**
     * @brief Interpolate between two angles using shortest wrapped difference.
     * @param a0 Start angle in radians.
     * @param a1 End angle in radians.
     * @param t Interpolation factor in [0, 1].
     * @return Interpolated angle in radians.
     */
    static double lerpAngle(double a0, double a1, double t) {
        const double d = normalizeAngle(a1 - a0);
        return normalizeAngle(a0 + d * t);
    }

    /**
     * @brief Periodic startup diagnostics until first UDP packet is sent.
     * @return None
     */
    void startupDiagnostics() {
        if (sequence_ > 0) {
            startup_diag_timer_->cancel();
            return;
        }

        if (!has_odom_dynamics_) {
            RCLCPP_WARN(get_logger(),
                        "UDP sender waiting: no valid odom dynamics yet on odom_topic");
            return;
        }

        if (!has_pose_topic_sample_) {
            RCLCPP_WARN(get_logger(),
                        "UDP sender waiting: pose_topic has not produced a sample yet");
        }
    }

    /**
     * @brief Sample trajectory at requested arc length with wrap-around interpolation.
     * @param s_query Arc-length query in meters.
     * @return Interpolated waypoint sample.
     */
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
            out.left_bound = w0.left_bound + (w1.left_bound - w0.left_bound) * t;
            out.right_bound = w0.right_bound + (w1.right_bound - w0.right_bound) * t;
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
        out.left_bound = w0.left_bound + (w1.left_bound - w0.left_bound) * t;
        out.right_bound = w0.right_bound + (w1.right_bound - w0.right_bound) * t;
        return out;
    }

    /**
     * @brief Update steering estimate from servo feedback topic.
     * @param msg Incoming servo-position message.
     * @return None
     */
    void servoCallback(const std_msgs::msg::Float64::SharedPtr msg) {
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

    /**
     * @brief Cache latest odometry dynamics used by pose-triggered sender path.
     * @param msg Incoming odometry message.
     * @return None
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (kdtree_.size() == 0) {
            return;
        }

        latest_vx_ = msg->twist.twist.linear.x;
        latest_vy_ = msg->twist.twist.linear.y;
        latest_omega_ = msg->twist.twist.angular.z;
        has_odom_dynamics_ = std::isfinite(latest_vx_) && std::isfinite(latest_vy_) && std::isfinite(latest_omega_);
    }

    /**
     * @brief Build and transmit one UDP state packet using supplied pose + cached dynamics.
     * @param x Global x position in meters.
     * @param y Global y position in meters.
     * @param qx Quaternion x component.
     * @param qy Quaternion y component.
     * @param qz Quaternion z component.
     * @param qw Quaternion w component.
     * @return None
     */
    void sendStatePacket(double x,
                         double y,
                         double qx,
                         double qy,
                         double qz,
                         double qw) {
        const auto t_start = std::chrono::steady_clock::now();

        if (kdtree_.size() == 0) {
            return;
        }
        if (!has_odom_dynamics_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "Skipping UDP send: waiting for valid odom dynamics sample");
            return;
        }

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(qx) ||
            !std::isfinite(qy) || !std::isfinite(qz) || !std::isfinite(qw)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "Skipping UDP send: invalid pose values");
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

        // Choose nearest waypoint, then bias forward using heading projection
        // to avoid selecting points slightly behind the vehicle at speed.
        size_t waypoint_idx = kdtree_.findNearest(x, y);
        {
            const double cos_theta = std::cos(theta);
            const double sin_theta = std::sin(theta);
            const size_t n = kdtree_.size();
            size_t best_idx = waypoint_idx;
            double best_dist = std::numeric_limits<double>::max();

            for (int i = 0; i <= MPC_FPGA_PUBLISHER_FORWARD_LOOKAHEAD; ++i) {
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

        {
            const size_t n = kdtree_.size();
            const auto& a = kdtree_.getWaypoint(waypoint_idx);
            const auto& b = kdtree_.getWaypoint((waypoint_idx + 1) % n);
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
            const double path_x = a.x + t * abx;
            const double path_y = a.y + t * aby;
            const double path_psi = lerpAngle(a.psi, b.psi, t);
            const double dx = x - path_x;
            const double dy = y - path_y;
            packet.e_y_fp = toFp(-std::sin(path_psi) * dx + std::cos(path_psi) * dy);
            packet.e_psi_fp = toFp(normalizeAngle(theta - path_psi));
        }
        packet.velocity_fp = toFp(vx);
        packet.vy_fp = toFp(vy);
        packet.omega_fp = toFp(omega);
        packet.steering_angle_fp = toFp(steering_angle);

        packet.horizon_length = static_cast<uint32_t>(MPC_HORIZON);

        if (interpolate_horizon_ && !trajectory_.empty()) {
            const double base_s = kdtree_.getWaypoint(waypoint_idx).s;
            for (size_t i = 0; i < MPC_HORIZON; ++i) {
                const Waypoint wp = sampleByArcLength(base_s + horizon_step_m_ * static_cast<double>(i));
                packet.ref_ey_fp[i] = toFp(0.0);
                packet.ref_vx_fp[i] = toFp(wp.vx);
                packet.ref_kappa_fp[i] = toFp(wp.kappa);
                packet.ref_left_bound_fp[i] = toFp(wp.left_bound);
                packet.ref_right_bound_fp[i] = toFp(wp.right_bound);
            }
        } else {
            const size_t n = kdtree_.size();
            for (size_t i = 0; i < MPC_HORIZON; ++i) {
                const size_t idx = (waypoint_idx + i) % n;
                const auto& wp = kdtree_.getWaypoint(idx);
                packet.ref_ey_fp[i] = toFp(0.0);
                packet.ref_vx_fp[i] = toFp(wp.vx);
                packet.ref_kappa_fp[i] = toFp(wp.kappa);
                packet.ref_left_bound_fp[i] = toFp(wp.left_bound);
                packet.ref_right_bound_fp[i] = toFp(wp.right_bound);
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
            return;
        }

        const auto t_end = std::chrono::steady_clock::now();
        const double send_us = static_cast<double>(
            std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count());

        if (send_stats_csv_ != nullptr) {
            send_stats_idx_++;
            std::fprintf(send_stats_csv_, "%lu,%.1f\n", send_stats_idx_, send_us);
            std::fflush(send_stats_csv_);
        }

        send_window_count_++;
        send_window_sum_us_ += send_us;
        send_window_min_us_ = std::min(send_window_min_us_, send_us);
        send_window_max_us_ = std::max(send_window_max_us_, send_us);

        const auto now_tp = std::chrono::steady_clock::now();
        const double elapsed_sec = std::chrono::duration<double>(now_tp - send_last_print_time_).count();
        if (elapsed_sec >= kSendStatsIntervalSec && send_window_count_ > 0) {
            const double avg_us = send_window_sum_us_ / static_cast<double>(send_window_count_);
            RCLCPP_INFO(get_logger(),
                        "[UDP Sender] Stats (last %.1fs, %lu calls): min=%.1f us, avg=%.1f us, max=%.1f us",
                        elapsed_sec,
                        send_window_count_,
                        send_window_min_us_,
                        avg_us,
                        send_window_max_us_);

            send_window_count_ = 0;
            send_window_sum_us_ = 0.0;
            send_window_min_us_ = std::numeric_limits<double>::infinity();
            send_window_max_us_ = 0.0;
            send_last_print_time_ = now_tp;
        }
    }

    /**
     * @brief Build and transmit one UDP state packet on each pose trigger.
     * @param msg Incoming pose message.
     * @return None
     */
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        has_pose_topic_sample_ = true;
        sendStatePacket(msg->pose.pose.position.x,
                        msg->pose.pose.position.y,
                        msg->pose.pose.orientation.x,
                        msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.z,
                        msg->pose.pose.orientation.w);
    }

    int sock_fd_{-1};
    sockaddr_in dest_addr_{};
    uint32_t sequence_{0};

    uint64_t send_stats_idx_{0};
    uint64_t send_window_count_{0};
    double send_window_sum_us_{0.0};
    double send_window_min_us_{std::numeric_limits<double>::infinity()};
    double send_window_max_us_{0.0};
    std::chrono::steady_clock::time_point send_last_print_time_{std::chrono::steady_clock::now()};
    static constexpr double kSendStatsIntervalSec = 5.0;
    FILE* send_stats_csv_{nullptr};

    double current_steering_angle_{0.0};
    bool has_servo_feedback_{false};
    bool interpolate_horizon_{true};
    double horizon_step_m_{0.0};
    double total_length_m_{0.0};
    double average_spacing_m_{0.2};

    std::vector<Waypoint> trajectory_;
    KDTree kdtree_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr servo_sub_;
    rclcpp::TimerBase::SharedPtr startup_diag_timer_;

    double latest_vx_{0.0};
    double latest_vy_{0.0};
    double latest_omega_{0.0};
    bool has_odom_dynamics_{false};
    bool has_pose_topic_sample_{false};
};

}  // namespace state_transport_udp

/**
 * @brief Entry point for ROS2 UDP sender process.
 * @param argc Argument count from process invocation.
 * @param argv Argument vector from process invocation.
 * @return Process exit code (0 on normal shutdown).
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<state_transport_udp::Ros2UdpSender>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
