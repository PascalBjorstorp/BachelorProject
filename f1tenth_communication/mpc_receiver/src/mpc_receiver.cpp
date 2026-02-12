/**
 * @file mpc_receiver.cpp
 * @brief ROS2 Node: Receives vehicle state + waypoint index, prepares MPC reference
 *
 * Runs on Ultra96. Subscribes to MpcState, extracts trajectory reference,
 * prepares data for MPC controller or FPGA.
 * 
 * ALL DATA STAYS IN Q16.16 FIXED-POINT for FPGA efficiency.
 * Float conversions only used for debug logging.
 */

#include <rclcpp/rclcpp.hpp>
#include <f1tenth_msgs/msg/mpc_state.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>

namespace f1tenth_communication {

/*===========================================================================
 * Fixed-Point Constants (Q16.16)
 *===========================================================================*/

constexpr int32_t FP_SCALE = 65536;
constexpr int32_t FP_ONE = 65536;

// For debug logging only
inline float fp_to_float_debug(int32_t fp) {
    return static_cast<float>(fp) / static_cast<float>(FP_SCALE);
}

inline int32_t float_to_fp(float f) {
    return static_cast<int32_t>(f * static_cast<float>(FP_SCALE));
}

/*===========================================================================
 * Fixed-Point Waypoint Structure (FPGA-ready)
 *===========================================================================*/

struct WaypointFP {
    int32_t s_fp;      // Arc length [m], Q16.16
    int32_t x_fp;      // Position X [m], Q16.16
    int32_t y_fp;      // Position Y [m], Q16.16
    int32_t psi_fp;    // Heading [rad], Q16.16
    int32_t kappa_fp;  // Curvature [1/m], Q16.16
    int32_t vx_fp;     // Target velocity [m/s], Q16.16
    int32_t ax_fp;     // Acceleration [m/s²], Q16.16
};

/*===========================================================================
 * MPC Reference Structure - ALL FIXED-POINT
 *===========================================================================*/

struct MpcReferenceFP {
    // Current state (from message, Q16.16)
    int32_t x_fp;
    int32_t y_fp;
    int32_t theta_fp;
    int32_t velocity_fp;
    
    // Reference trajectory (next N waypoints, Q16.16)
    static constexpr size_t HORIZON = 10;
    int32_t ref_x_fp[HORIZON];
    int32_t ref_y_fp[HORIZON];
    int32_t ref_theta_fp[HORIZON];
    int32_t ref_v_fp[HORIZON];
    int32_t ref_kappa_fp[HORIZON];   // Curvature for feedforward
    
    // Start index in trajectory
    uint32_t start_index;
    
    // Number of valid reference points
    size_t horizon_size;
};

/*===========================================================================
 * MPC Receiver Node
 *===========================================================================*/

class MpcReceiverNode : public rclcpp::Node {
public:
    MpcReceiverNode() : Node("mpc_receiver") {
        // Parameters
        this->declare_parameter("trajectory_file", "");
        this->declare_parameter("input_topic", "/mpc_state");
        this->declare_parameter("drive_topic", "/drive");
        this->declare_parameter("horizon", 10);
        
        std::string trajectory_file = this->get_parameter("trajectory_file").as_string();
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string drive_topic = this->get_parameter("drive_topic").as_string();
        horizon_ = static_cast<size_t>(this->get_parameter("horizon").as_int());
        
        if (trajectory_file.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No trajectory file specified!");
            return;
        }
        
        // Load trajectory (directly into fixed-point format)
        if (!load_trajectory(trajectory_file)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load trajectory from: %s", 
                        trajectory_file.c_str());
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints (Q16.16 fixed-point) from %s",
                   trajectory_fp_.size(), trajectory_file.c_str());
        
        // Create publisher for drive commands
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic, 10);
        
        // Subscribe to MpcState with Best Effort QoS (must match publisher)
        auto qos = rclcpp::QoS(1)
            .best_effort()
            .durability_volatile();
        
        sub_ = this->create_subscription<f1tenth_msgs::msg::MpcState>(
            input_topic, qos,
            std::bind(&MpcReceiverNode::mpc_state_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), 
                   "MPC Receiver ready (Fixed-Point, Best Effort QoS). Subscribing to %s",
                   input_topic.c_str());
    }
    
private:
    std::vector<WaypointFP> trajectory_fp_;  // Only fixed-point trajectory
    size_t horizon_ = 10;
    
    rclcpp::Subscription<f1tenth_msgs::msg::MpcState>::SharedPtr sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    
    // Current MPC reference (fixed-point, ready for FPGA)
    MpcReferenceFP current_ref_;
    
    // Statistics
    uint64_t msg_count_ = 0;
    double total_latency_ms_ = 0.0;
    
    bool load_trajectory(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            return false;
        }
        
        trajectory_fp_.clear();
        std::string line;
        
        // Skip header
        std::getline(file, line);
        
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string token;
            WaypointFP wp_fp;
            
            // Parse CSV and convert directly to Q16.16
            std::getline(ss, token, ','); wp_fp.s_fp = float_to_fp(std::stof(token));
            std::getline(ss, token, ','); wp_fp.x_fp = float_to_fp(std::stof(token));
            std::getline(ss, token, ','); wp_fp.y_fp = float_to_fp(std::stof(token));
            std::getline(ss, token, ','); wp_fp.psi_fp = float_to_fp(std::stof(token));
            std::getline(ss, token, ','); wp_fp.kappa_fp = float_to_fp(std::stof(token));
            std::getline(ss, token, ','); wp_fp.vx_fp = float_to_fp(std::stof(token));
            std::getline(ss, token, ','); wp_fp.ax_fp = float_to_fp(std::stof(token));
            
            trajectory_fp_.push_back(wp_fp);
        }
        
        return !trajectory_fp_.empty();
    }
    
    void mpc_state_callback(const f1tenth_msgs::msg::MpcState::SharedPtr msg) {
        auto recv_time = std::chrono::system_clock::now();
        
        // === ALL DATA STAYS IN FIXED-POINT (Q16.16) ===
        
        // Store current state directly (no conversion!)
        current_ref_.x_fp = msg->x_fp;
        current_ref_.y_fp = msg->y_fp;
        current_ref_.theta_fp = msg->theta_fp;
        current_ref_.velocity_fp = msg->velocity_fp;
        current_ref_.start_index = msg->waypoint_index;
        
        // Extract reference trajectory (next N waypoints in fixed-point)
        size_t traj_size = trajectory_fp_.size();
        size_t h = std::min(horizon_, MpcReferenceFP::HORIZON);
        current_ref_.horizon_size = h;
        
        uint32_t wp_idx = msg->waypoint_index;
        for (size_t i = 0; i < h; i++) {
            size_t idx = (wp_idx + i) % traj_size;  // Wrap around for closed loop
            current_ref_.ref_x_fp[i] = trajectory_fp_[idx].x_fp;
            current_ref_.ref_y_fp[i] = trajectory_fp_[idx].y_fp;
            current_ref_.ref_theta_fp[i] = trajectory_fp_[idx].psi_fp;
            current_ref_.ref_v_fp[i] = trajectory_fp_[idx].vx_fp;
            current_ref_.ref_kappa_fp[i] = trajectory_fp_[idx].kappa_fp;
        }
        
        // === YOUR MPC CALL GOES HERE (ALL FIXED-POINT) ===
        // The current_ref_ structure is ready for:
        // 1. Your fixed-point MPC computation
        // 2. Direct DMA transfer to FPGA
        // 
        // Example: mpc_compute_fp(&current_ref_, &steering_fp, &speed_fp);
        
        // For now: placeholder that converts for drive command
        // (In final version, FPGA returns fixed-point control)
        int32_t steering_fp = current_ref_.ref_theta_fp[0];  // TODO: MPC output (Q16.16)
        int32_t speed_fp = current_ref_.ref_v_fp[0];  // Use first target velocity
        
        // Convert to float ONLY for ROS message (this is the only float conversion)
        float steering = fp_to_float_debug(steering_fp);
        float speed = fp_to_float_debug(speed_fp);
        
        // Publish drive command
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = "base_link";
        drive_msg.drive.steering_angle = steering;
        drive_msg.drive.speed = speed;
        drive_pub_->publish(drive_msg);
        
        // Calculate latency
        msg_count_++;
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            recv_time.time_since_epoch()).count();
        double latency_ms = static_cast<double>(now_ms - msg->timestamp_ms);
        total_latency_ms_ += latency_ms;
        
        // Debug logging (throttled) - converts for display only
        if (msg_count_ % 50 == 0) {
            double avg_latency = total_latency_ms_ / msg_count_;
            RCLCPP_INFO(this->get_logger(), 
                       "State: pos=(%.2f, %.2f) theta=%.2f° vel=%.2f m/s | "
                       "Waypoint: %u | Target vel: %.2f m/s | "
                       "Latency: %.1f ms (avg: %.1f ms)",
                       fp_to_float_debug(msg->x_fp), 
                       fp_to_float_debug(msg->y_fp),
                       fp_to_float_debug(msg->theta_fp) * 180.0 / 3.14159,
                       fp_to_float_debug(msg->velocity_fp),
                       wp_idx, speed,
                       latency_ms, avg_latency);
        }
    }
    
public:
    // Getters for FPGA integration
    const std::vector<WaypointFP>& get_trajectory_fp() const { return trajectory_fp_; }
    const MpcReferenceFP& get_current_reference() const { return current_ref_; }
    size_t get_trajectory_size() const { return trajectory_fp_.size(); }
};

} // namespace f1tenth_communication

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<f1tenth_communication::MpcReceiverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
