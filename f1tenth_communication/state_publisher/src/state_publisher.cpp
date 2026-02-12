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
        this->declare_parameter("odom_topic", "/odom");
        this->declare_parameter("output_topic", "/mpc_state");
        this->declare_parameter("publish_rate_hz", 50.0);
        
        std::string trajectory_file = this->get_parameter("trajectory_file").as_string();
        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        
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
        
        RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from %s",
                   kdtree_.size(), trajectory_file.c_str());
        
        // Create publisher with Best Effort QoS (lower latency)
        auto qos = rclcpp::QoS(1)
            .best_effort()           // Don't retry failed packets
            .durability_volatile();  // Don't store messages
        pub_ = this->create_publisher<f1tenth_msgs::msg::MpcState>(output_topic, qos);
        
        // Subscribe to odometry with Best Effort QoS
        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, qos,
            std::bind(&StatePublisherNode::odom_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "State publisher ready (Best Effort QoS). Subscribing to %s, publishing to %s",
                   odom_topic.c_str(), output_topic.c_str());
    }
    
private:
    KDTree kdtree_;
    rclcpp::Publisher<f1tenth_msgs::msg::MpcState>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    
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
        return true;
    }
    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Extract position
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        
        // Extract yaw from quaternion
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        double theta = std::atan2(2.0 * (qw * qz + qx * qy), 
                                  1.0 - 2.0 * (qy * qy + qz * qz));
        
        // Extract velocity
        double velocity = msg->twist.twist.linear.x;
        
        // KD-tree lookup
        auto start_time = std::chrono::high_resolution_clock::now();
        size_t waypoint_idx = kdtree_.find_nearest(x, y);
        auto end_time = std::chrono::high_resolution_clock::now();
        
        // Create and publish message (using Q16.16 fixed-point for FPGA efficiency)
        // Q16.16: multiply by 65536 (2^16) to convert float to fixed-point
        constexpr double FP_SCALE = 65536.0;
        
        auto mpc_state = f1tenth_msgs::msg::MpcState();
        mpc_state.header.stamp = msg->header.stamp;
        mpc_state.header.frame_id = "map";
        mpc_state.x_fp = static_cast<int32_t>(x * FP_SCALE);
        mpc_state.y_fp = static_cast<int32_t>(y * FP_SCALE);
        mpc_state.theta_fp = static_cast<int32_t>(theta * FP_SCALE);
        mpc_state.velocity_fp = static_cast<int32_t>(velocity * FP_SCALE);
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
