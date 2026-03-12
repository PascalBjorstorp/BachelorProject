/**
 * @file state_publisher.cpp
 * @brief ROS2 Node: Publishes vehicle state + waypoint index
 *
 * Runs on Jetson. Subscribes to localization, performs KD-tree lookup,
 * publishes MpcState for the Ultra96 MPC controller.
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <f1tenth_msgs/msg/mpc_state.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>

namespace f1tenth_communication {

/*===========================================================================
 * KD-Tree Implementation (embedded for portability)
 *===========================================================================*/

struct Waypoint {
    double s;       // Arc length
    double x;       // Position X
    double y;       // Position Y
    double psi;     // Heading
    double kappa;   // Curvature
    double vx;      // Target velocity
    double ax;      // Acceleration
};

struct KDNode {
    double x, y;
    size_t index;
};

class KDTree {
public:
    void build(const std::vector<Waypoint>& waypoints) {
        waypoints_ = waypoints;
        nodes_.clear();
        nodes_.reserve(waypoints.size());
        
        // Create nodes with original indices
        for (size_t i = 0; i < waypoints.size(); i++) {
            nodes_.push_back({waypoints[i].x, waypoints[i].y, i});
        }
        
        // Build tree
        build_recursive(0, nodes_.size(), 0);
    }
    
    size_t find_nearest(double x, double y) const {
        if (nodes_.empty()) return 0;
        
        size_t best_idx = 0;
        double best_dist = std::numeric_limits<double>::max();
        search_recursive(0, nodes_.size(), 0, x, y, best_idx, best_dist);
        return nodes_[best_idx].index;
    }
    
    const Waypoint& get_waypoint(size_t idx) const {
        return waypoints_[idx];
    }
    
    size_t size() const { return waypoints_.size(); }
    
private:
    std::vector<KDNode> nodes_;
    std::vector<Waypoint> waypoints_;
    
    void build_recursive(size_t start, size_t end, int depth) {
        if (end - start <= 1) return;
        
        size_t mid = start + (end - start) / 2;
        
        // Sort by appropriate dimension
        if (depth % 2 == 0) {
            std::nth_element(nodes_.begin() + start, nodes_.begin() + mid, 
                           nodes_.begin() + end,
                           [](const KDNode& a, const KDNode& b) { return a.x < b.x; });
        } else {
            std::nth_element(nodes_.begin() + start, nodes_.begin() + mid,
                           nodes_.begin() + end,
                           [](const KDNode& a, const KDNode& b) { return a.y < b.y; });
        }
        
        build_recursive(start, mid, depth + 1);
        build_recursive(mid + 1, end, depth + 1);
    }
    
    void search_recursive(size_t start, size_t end, int depth,
                         double x, double y,
                         size_t& best_idx, double& best_dist) const {
        if (start >= end) return;
        
        size_t mid = start + (end - start) / 2;
        const KDNode& node = nodes_[mid];
        
        // Check this node
        double dx = x - node.x;
        double dy = y - node.y;
        double dist = dx * dx + dy * dy;
        
        if (dist < best_dist) {
            best_dist = dist;
            best_idx = mid;
        }
        
        // Determine which side to search first
        double split_val = (depth % 2 == 0) ? node.x : node.y;
        double query_val = (depth % 2 == 0) ? x : y;
        double diff = query_val - split_val;
        
        if (diff < 0) {
            search_recursive(start, mid, depth + 1, x, y, best_idx, best_dist);
            if (diff * diff < best_dist) {
                search_recursive(mid + 1, end, depth + 1, x, y, best_idx, best_dist);
            }
        } else {
            search_recursive(mid + 1, end, depth + 1, x, y, best_idx, best_dist);
            if (diff * diff < best_dist) {
                search_recursive(start, mid, depth + 1, x, y, best_idx, best_dist);
            }
        }
    }
};

/*===========================================================================
 * State Publisher Node
 *===========================================================================*/

class StatePublisherNode : public rclcpp::Node {
public:
    StatePublisherNode() : Node("state_publisher") {
        // Parameters
        this->declare_parameter("trajectory_file", "");
        this->declare_parameter("odom_topic", "/ego_racecar/odom");
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
        
        std::string trajectory_file = this->get_parameter("trajectory_file").as_string();
        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        std::string servo_topic = this->get_parameter("servo_topic").as_string();
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        servo_gain_ = this->get_parameter("servo_gain").as_double();
        servo_offset_ = this->get_parameter("servo_offset").as_double();
        steer_c2_ = this->get_parameter("steering_correction_c2").as_double();
        steer_c1_ = this->get_parameter("steering_correction_c1").as_double();
        steer_c0_ = this->get_parameter("steering_correction_c0").as_double();
        forward_lookahead_ = static_cast<int>(this->get_parameter("forward_lookahead").as_int());
        
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
        
        // Create publisher with Best Effort QoS (lower latency)
        auto qos = rclcpp::QoS(1)
            .best_effort()           // Don't retry failed packets
            .durability_volatile();  // Don't store messages
        pub_ = this->create_publisher<f1tenth_msgs::msg::MpcState>(output_topic, qos);
        
        // Subscribe to odometry with Best Effort QoS
        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, qos,
            std::bind(&StatePublisherNode::odom_callback, this, std::placeholders::_1));
        
        // Subscribe to servo position feedback (for actual steering angle)
        if (!servo_topic.empty()) {
            servo_sub_ = this->create_subscription<std_msgs::msg::Float64>(
                servo_topic, qos,
                [this](const std_msgs::msg::Float64::SharedPtr msg) {
                    // msg->data is the VESC servo command value (0.0-1.0)
                    // Invert: corrected = (servo - offset) / gain
                    // Then solve polynomial: c2·t² + c1·t + c0 = corrected for t
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

        RCLCPP_INFO(this->get_logger(), "State publisher ready (Best Effort QoS). Subscribing to %s, publishing to %s",
                   odom_topic.c_str(), output_topic.c_str());
    }
    
private:
    KDTree kdtree_;
    rclcpp::Publisher<f1tenth_msgs::msg::MpcState>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr servo_sub_;
    
    double current_steering_angle_ = 0.0;  // Steering angle [rad] (converted from servo value)
    bool has_servo_feedback_ = false;
    double wheelbase_ = 0.324;
    double servo_gain_ = -0.7284;    // VESC servo → steering gain
    double servo_offset_ = 0.55;     // VESC servo center offset
    double steer_c2_ = 0.589566;     // Steering correction polynomial c2
    double steer_c1_ = 0.918061;     // Steering correction polynomial c1
    double steer_c0_ = 0.001490;     // Steering correction polynomial c0
    int forward_lookahead_ = 3;      // Waypoints ahead to check for forward bias
    uint32_t trajectory_hash_ = 0;   // Checksum for cross-node trajectory verification

    // Odom timeout detection
    rclcpp::TimerBase::SharedPtr odom_watchdog_timer_;
    std::chrono::steady_clock::time_point last_odom_time_ = std::chrono::steady_clock::now();
    bool odom_received_ = false;
    
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
            std::getline(ss, token, ','); wp.ax = std::stod(token);
            
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

        // Extract position
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        // --- Input bounds validation ---
        // Skip this message if any key value is NaN/Inf or clearly out of range.
        {
            double qx = msg->pose.pose.orientation.x;
            double qy = msg->pose.pose.orientation.y;
            double qz = msg->pose.pose.orientation.z;
            double qw = msg->pose.pose.orientation.w;
            double vx = msg->twist.twist.linear.x;
            double vy = msg->twist.twist.linear.y;
            double wz = msg->twist.twist.angular.z;

            auto ok = [](double v) { return std::isfinite(v); };
            if (!ok(x) || !ok(y) || !ok(qx) || !ok(qy) || !ok(qz) || !ok(qw) ||
                !ok(vx) || !ok(vy) || !ok(wz)) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Dropping odom: NaN/Inf detected in incoming message");
                return;
            }
            constexpr double POS_LIMIT = 500.0;   // ±500 m
            constexpr double VEL_LIMIT = 50.0;    // ±50 m/s
            if (std::abs(x) > POS_LIMIT || std::abs(y) > POS_LIMIT) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Dropping odom: position out of range (%.1f, %.1f)", x, y);
                return;
            }
            if (std::abs(vx) > VEL_LIMIT || std::abs(vy) > VEL_LIMIT) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Dropping odom: velocity out of range (vx=%.1f, vy=%.1f)", vx, vy);
                return;
            }
        }
        
        // Extract yaw from quaternion
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        double theta = std::atan2(2.0 * (qw * qz + qx * qy), 
                                  1.0 - 2.0 * (qy * qy + qz * qz));
        
        // Extract velocity (body-frame twist from EKF / odometry)
        double velocity = msg->twist.twist.linear.x;   // Longitudinal v_x
        double vy = msg->twist.twist.linear.y;          // Lateral v_y
        double omega = msg->twist.twist.angular.z;       // Yaw rate ω
        
        // Compute wheel speed from longitudinal velocity (zero-slip assumption)
        // ω_w = v_x / R_w where R_w = 0.0545m (Traxxas Slash 4x4 VXL)
        constexpr double WHEEL_RADIUS = 0.0545;
        double wheel_speed = (velocity > 0.01) ? (velocity / WHEEL_RADIUS) : 0.0;
        
        // Determine steering angle: use servo feedback if available,
        // otherwise estimate from bicycle model: δ ≈ atan(L * ω / v_x)
        double steering_angle = current_steering_angle_;
        if (!has_servo_feedback_ && std::abs(velocity) > 0.1) {
            steering_angle = std::atan2(wheelbase_ * omega, velocity);
        }
        
        // KD-tree lookup + forward-biased waypoint selection
        auto start_time = std::chrono::high_resolution_clock::now();
        size_t waypoint_idx = kdtree_.find_nearest(x, y);
        
        // Forward bias: at high speed the closest waypoint may be slightly
        // behind the car.  Check the next few waypoints and prefer the
        // nearest one that is *ahead* of the vehicle heading.
        {
            const double cos_theta = std::cos(theta);
            const double sin_theta = std::sin(theta);
            const size_t N = kdtree_.size();
            size_t best_idx = waypoint_idx;
            double best_dist = std::numeric_limits<double>::max();
            
            for (int i = 0; i <= forward_lookahead_; ++i) {
                size_t check_idx = (waypoint_idx + static_cast<size_t>(i)) % N;
                const auto& wp = kdtree_.get_waypoint(check_idx);
                double dx_wp = wp.x - x;
                double dy_wp = wp.y - y;
                // Dot product with heading: positive means ahead
                double ahead = dx_wp * cos_theta + dy_wp * sin_theta;
                if (ahead >= 0.0) {
                    double dist = dx_wp * dx_wp + dy_wp * dy_wp;
                    if (dist < best_dist) {
                        best_dist = dist;
                        best_idx = check_idx;
                    }
                }
            }
            waypoint_idx = best_idx;
        }
        auto end_time = std::chrono::high_resolution_clock::now();
        
        // Create and publish message (using Q16.16 fixed-point for FPGA efficiency)
        // Q16.16: multiply by 65536 (2^16) to convert float to fixed-point
        constexpr double FP_SCALE = 65536.0;
        
        // Round to nearest (avoid truncation bias in Q16.16 conversion)
        auto to_fp = [](double v) -> int32_t {
            if (!std::isfinite(v)) return 0;  // Guard against NaN/Inf from localization
            return static_cast<int32_t>(v >= 0.0 ? v * FP_SCALE + 0.5 : v * FP_SCALE - 0.5);
        };
        
        auto mpc_state = f1tenth_msgs::msg::MpcState();
        mpc_state.header.stamp = msg->header.stamp;
        mpc_state.header.frame_id = "map";
        mpc_state.x_fp = to_fp(x);
        mpc_state.y_fp = to_fp(y);
        mpc_state.theta_fp = to_fp(theta);
        mpc_state.velocity_fp = to_fp(velocity);
        mpc_state.vy_fp = to_fp(vy);
        mpc_state.omega_fp = to_fp(omega);
        mpc_state.wheel_speed_fp = to_fp(wheel_speed);
        mpc_state.steering_angle_fp = to_fp(steering_angle);
        mpc_state.waypoint_index = static_cast<uint32_t>(waypoint_idx);
        mpc_state.timestamp_ms = static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count() & 0xFFFFFFFF);
        
        pub_->publish(mpc_state);
        
        // Debug logging (throttled)
        static int count = 0;
        if (++count % 50 == 0) {
            auto lookup_us = std::chrono::duration_cast<std::chrono::microseconds>(
                end_time - start_time).count();
            RCLCPP_DEBUG(this->get_logger(), 
                        "Published MpcState: pos=(%.2f, %.2f), waypoint=%u, lookup=%ldus",
                        x, y, mpc_state.waypoint_index, lookup_us);
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
