/**
 * @file mpc_receiver_fpga.cpp
 * @brief MPC Receiver with FPGA Integration - Trajectory Stored in FPGA BRAM
 *
 * Loads trajectory to FPGA BRAM once at startup for minimal per-cycle overhead.
 * Each cycle only sends vehicle state (32 bytes) instead of full waypoint block.
 *
 * Flow:
 *   Startup: Load trajectory CSV → DMA to FPGA BRAM (once)
 *   Runtime: Receive MpcState → Send state to FPGA → Get steering → Publish /drive
 */

#include <rclcpp/rclcpp.hpp>
#include <f1tenth_msgs/msg/mpc_state.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include <cstring>
#include <cmath>

// Linux memory-mapped I/O
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

// FPGA interface definitions
extern "C" {
    #include "fpga_interface.h"
}

namespace f1tenth_communication {

/*===========================================================================
 * Fixed-Point Helpers (Q16.16 format: 16 integer bits, 16 fractional bits)
 * FP_SCALE = 2^16 = 65536, used for float<->fixed conversion
 *===========================================================================*/

constexpr int32_t FP_SCALE = 65536;  // = 2^16

inline float fp_to_float(int32_t fp) {
    return static_cast<float>(fp) / static_cast<float>(FP_SCALE);
}

inline int32_t float_to_fp(float f) {
    return static_cast<int32_t>(f * static_cast<float>(FP_SCALE));
}

/*===========================================================================
 * Waypoint Structure (for loading from CSV)
 *===========================================================================*/

struct WaypointCSV {
    float s, x, y, psi, kappa, vx, ax;
};

/*===========================================================================
 * FPGA Interface - Trajectory in BRAM
 *===========================================================================*/

class FpgaInterface {
public:
    FpgaInterface() : mem_fd_(-1), fpga_base_(nullptr), initialized_(false) {}
    
    ~FpgaInterface() { close_device(); }
    
    bool initialize(uint32_t base_addr = FPGA_BASE_ADDR, size_t map_size = 0x100000) {
        base_addr_ = base_addr;
        map_size_ = map_size;
        
        mem_fd_ = open("/dev/mem", O_RDWR | O_SYNC);
        if (mem_fd_ < 0) {
            return false;
        }
        
        fpga_base_ = mmap(nullptr, map_size_, PROT_READ | PROT_WRITE,
                          MAP_SHARED, mem_fd_, base_addr_);
        if (fpga_base_ == MAP_FAILED) {
            close(mem_fd_);
            mem_fd_ = -1;
            return false;
        }
        
        initialized_ = true;
        return true;
    }
    
    void close_device() {
        if (fpga_base_ && fpga_base_ != MAP_FAILED) {
            munmap(fpga_base_, map_size_);
            fpga_base_ = nullptr;
        }
        if (mem_fd_ >= 0) {
            close(mem_fd_);
            mem_fd_ = -1;
        }
        initialized_ = false;
    }
    
    bool is_ready() const { return initialized_; }
    
    /**
     * @brief Load trajectory to FPGA BRAM (called once at startup)
     */
    bool load_trajectory(const std::vector<WaypointCSV>& waypoints) {
        if (!initialized_) return false;
        
        volatile uint8_t* base = static_cast<volatile uint8_t*>(fpga_base_);
        volatile FpgaWaypoint_t* traj_bram = 
            reinterpret_cast<volatile FpgaWaypoint_t*>(base + FPGA_TRAJ_OFFSET);
        
        size_t count = std::min(waypoints.size(), static_cast<size_t>(MAX_TRAJECTORY_SIZE));
        
        // Copy waypoints to FPGA BRAM
        for (size_t i = 0; i < count; i++) {
            FpgaWaypoint_t wp;
            wp.x_fp = float_to_fp(waypoints[i].x);
            wp.y_fp = float_to_fp(waypoints[i].y);
            wp.theta_fp = float_to_fp(waypoints[i].psi);
            wp.velocity_fp = float_to_fp(waypoints[i].vx);
            wp.kappa_fp = float_to_fp(waypoints[i].kappa);
            wp.reserved[0] = wp.reserved[1] = wp.reserved[2] = 0;
            
            memcpy(const_cast<FpgaWaypoint_t*>(&traj_bram[i]), &wp, sizeof(FpgaWaypoint_t));
        }
        
        __sync_synchronize();
        
        trajectory_size_ = count;
        trajectory_loaded_ = true;
        
        return true;
    }
    
    /**
     * @brief Configure control parameters
     */
    void set_parameters(const FpgaParams_t& params) {
        if (!initialized_) return;
        
        volatile uint8_t* base = static_cast<volatile uint8_t*>(fpga_base_);
        volatile FpgaParams_t* fpga_params = 
            reinterpret_cast<volatile FpgaParams_t*>(base + FPGA_PARAMS_OFFSET);
        
        FpgaParams_t p = params;
        p.trajectory_size = trajectory_size_;
        
        std::memcpy(const_cast<FpgaParams_t*>(fpga_params), &p, sizeof(FpgaParams_t));
        __sync_synchronize();
    }
    
    /**
     * @brief Compute control output (called every cycle)
     * 
     * Only sends 32 bytes of state data!
     */
    bool compute(const FpgaStateInput_t& state, FpgaOutput_t& output) {
        if (!initialized_ || !trajectory_loaded_) return false;
        
        volatile uint8_t* base = static_cast<volatile uint8_t*>(fpga_base_);
        volatile uint32_t* ctrl = reinterpret_cast<volatile uint32_t*>(base + FPGA_CTRL_OFFSET);
        volatile FpgaStateInput_t* fpga_state = 
            reinterpret_cast<volatile FpgaStateInput_t*>(base + FPGA_STATE_OFFSET);
        volatile FpgaOutput_t* fpga_output = 
            reinterpret_cast<volatile FpgaOutput_t*>(base + FPGA_OUTPUT_OFFSET);
        
        // Wait for idle
        int timeout = 1000;
        while ((ctrl[REG_IDLE/4] == 0) && (timeout-- > 0));
        if (timeout <= 0) return false;
        
        // Write state (only 32 bytes!)
        std::memcpy(const_cast<FpgaStateInput_t*>(fpga_state), &state, sizeof(FpgaStateInput_t));
        __sync_synchronize();
        
        // Start computation
        ctrl[REG_START/4] = 1;
        
        // Wait for done
        timeout = 10000;
        while ((ctrl[REG_DONE/4] == 0) && (timeout-- > 0));
        if (timeout <= 0) return false;
        
        // Read output
        std::memcpy(&output, const_cast<FpgaOutput_t*>(fpga_output), sizeof(FpgaOutput_t));
        
        return true;
    }
    
    size_t get_trajectory_size() const { return trajectory_size_; }
    bool is_trajectory_loaded() const { return trajectory_loaded_; }
    
private:
    int mem_fd_;
    void* fpga_base_;
    uint32_t base_addr_;
    size_t map_size_;
    bool initialized_;
    size_t trajectory_size_ = 0;
    bool trajectory_loaded_ = false;
};

/*===========================================================================
 * Software Fallback (Pure Pursuit on ARM)
 *===========================================================================*/

class SoftwarePurePursuit {
public:
    void set_trajectory(const std::vector<WaypointCSV>& waypoints) {
        trajectory_ = waypoints;
    }
    
    void set_parameters(const FpgaParams_t& params) {
        params_ = params;
    }
    
    void compute(const FpgaStateInput_t& state, FpgaOutput_t& output) {
        output.status = STATUS_OK;
        output.sequence_number = state.sequence_number;
        output.timestamp_ms = state.timestamp_ms;
        
        if (trajectory_.empty()) {
            output.status = STATUS_NO_TRAJECTORY;
            return;
        }
        
        // Compute lookahead
        float speed = std::abs(fp_to_float(state.velocity_fp));
        float lookahead = fp_to_float(params_.min_lookahead_fp) + 
                         fp_to_float(params_.lookahead_gain_fp) * speed;
        lookahead = std::clamp(lookahead, 
                               fp_to_float(params_.min_lookahead_fp),
                               fp_to_float(params_.max_lookahead_fp));
        output.lookahead_dist_fp = float_to_fp(lookahead);
        
        // Find target waypoint
        float x = fp_to_float(state.x_fp);
        float y = fp_to_float(state.y_fp);
        float theta = fp_to_float(state.theta_fp);
        
        size_t wp_idx = state.waypoint_index % trajectory_.size();
        size_t target_idx = wp_idx;
        float lookahead_sq = lookahead * lookahead;
        
        for (size_t i = 0; i < params_.lookahead_points && i < trajectory_.size(); i++) {
            size_t idx = (wp_idx + i) % trajectory_.size();
            float dx = trajectory_[idx].x - x;
            float dy = trajectory_[idx].y - y;
            float dist_sq = dx*dx + dy*dy;
            float forward = dx * std::cos(theta) + dy * std::sin(theta);
            
            if (forward > 0 && dist_sq >= lookahead_sq) {
                target_idx = idx;
                break;
            }
            if (forward > 0) target_idx = idx;
        }
        output.target_waypoint_idx = target_idx;
        
        // Pure Pursuit steering
        const auto& target = trajectory_[target_idx];
        float dx = target.x - x;
        float dy = target.y - y;
        float y_vehicle = -std::sin(theta) * dx + std::cos(theta) * dy;
        float L_sq = dx*dx + dy*dy;
        
        float steering = 0.0f;
        if (L_sq > 0.01f) {
            float curvature = 2.0f * y_vehicle / L_sq;
            float wheelbase = fp_to_float(params_.wheelbase_fp);
            steering = std::atan(curvature * wheelbase);
            float max_steer = fp_to_float(params_.max_steering_fp);
            steering = std::clamp(steering, -max_steer, max_steer);
        }
        output.steering_angle_fp = float_to_fp(steering);
        
        // Velocity
        float max_vel = fp_to_float(params_.max_velocity_fp);
        output.velocity_fp = float_to_fp(std::min(target.vx, max_vel));
        
        // CTE
        const auto& closest = trajectory_[wp_idx];
        float dx_c = x - closest.x;
        float dy_c = y - closest.y;
        float cte = -std::sin(closest.psi) * dx_c + std::cos(closest.psi) * dy_c;
        output.cross_track_error_fp = float_to_fp(cte);
        
        // Heading error
        float h_err = target.psi - theta;
        while (h_err > M_PI) h_err -= 2*M_PI;
        while (h_err < -M_PI) h_err += 2*M_PI;
        output.heading_error_fp = float_to_fp(h_err);
    }
    
private:
    std::vector<WaypointCSV> trajectory_;
    FpgaParams_t params_;
};

/*===========================================================================
 * MPC Receiver FPGA Node
 *===========================================================================*/

class MpcReceiverFpgaNode : public rclcpp::Node {
public:
    MpcReceiverFpgaNode() : Node("mpc_receiver_fpga") {
        // Declare parameters
        declare_parameter("trajectory_file", "");
        declare_parameter("input_topic", "/mpc_state");
        declare_parameter("drive_topic", "/drive");
        declare_parameter("use_fpga", true);
        declare_parameter("fpga_base_address", static_cast<int64_t>(FPGA_BASE_ADDR));
        
        // Pure Pursuit parameters
        declare_parameter("min_lookahead", 0.5);
        declare_parameter("max_lookahead", 2.0);
        declare_parameter("lookahead_gain", 0.3);
        declare_parameter("lookahead_points", 20);
        declare_parameter("wheelbase", 0.324);
        declare_parameter("max_steering", 0.42);
        declare_parameter("max_velocity", 6.0);
        
        // Get parameters
        std::string trajectory_file = get_parameter("trajectory_file").as_string();
        std::string input_topic = get_parameter("input_topic").as_string();
        std::string drive_topic = get_parameter("drive_topic").as_string();
        use_fpga_ = get_parameter("use_fpga").as_bool();
        
        // Load trajectory from CSV
        if (trajectory_file.empty()) {
            RCLCPP_ERROR(get_logger(), "No trajectory file specified!");
            return;
        }
        
        if (!load_trajectory(trajectory_file)) {
            RCLCPP_ERROR(get_logger(), "Failed to load trajectory: %s", trajectory_file.c_str());
            return;
        }
        
        RCLCPP_INFO(get_logger(), "Loaded %zu waypoints from %s", 
                   trajectory_.size(), trajectory_file.c_str());
        
        // Setup control parameters
        init_parameters();
        
        // Initialize FPGA
        if (use_fpga_) {
            uint32_t addr = static_cast<uint32_t>(get_parameter("fpga_base_address").as_int());
            
            if (fpga_.initialize(addr)) {
                // Load trajectory to FPGA BRAM (once!)
                if (fpga_.load_trajectory(trajectory_)) {
                    RCLCPP_INFO(get_logger(), 
                               "FPGA initialized at 0x%08X, %zu waypoints loaded to BRAM",
                               addr, fpga_.get_trajectory_size());
                    fpga_.set_parameters(params_);
                } else {
                    RCLCPP_WARN(get_logger(), "Failed to load trajectory to FPGA");
                    use_fpga_ = false;
                }
            } else {
                RCLCPP_WARN(get_logger(), "FPGA init failed, using software fallback");
                use_fpga_ = false;
            }
        }
        
        if (!use_fpga_) {
            sw_fallback_.set_trajectory(trajectory_);
            sw_fallback_.set_parameters(params_);
            RCLCPP_INFO(get_logger(), "Using software Pure Pursuit");
        }
        
        // Create publisher
        drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic, 10);
        
        // Subscribe
        auto qos = rclcpp::QoS(1).best_effort().durability_volatile();
        sub_ = create_subscription<f1tenth_msgs::msg::MpcState>(
            input_topic, qos,
            std::bind(&MpcReceiverFpgaNode::state_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(get_logger(), "MPC Receiver FPGA ready [%s]. Subscribing to %s",
                   use_fpga_ ? "FPGA" : "SOFTWARE", input_topic.c_str());
    }
    
private:
    std::vector<WaypointCSV> trajectory_;
    FpgaParams_t params_;
    bool use_fpga_ = true;
    
    FpgaInterface fpga_;
    SoftwarePurePursuit sw_fallback_;
    
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<f1tenth_msgs::msg::MpcState>::SharedPtr sub_;
    
    uint64_t msg_count_ = 0;
    double total_latency_ms_ = 0.0;
    
    void init_parameters() {
        params_.min_lookahead_fp = float_to_fp(get_parameter("min_lookahead").as_double());
        params_.max_lookahead_fp = float_to_fp(get_parameter("max_lookahead").as_double());
        params_.lookahead_gain_fp = float_to_fp(get_parameter("lookahead_gain").as_double());
        params_.wheelbase_fp = float_to_fp(get_parameter("wheelbase").as_double());
        params_.max_steering_fp = float_to_fp(get_parameter("max_steering").as_double());
        params_.max_velocity_fp = float_to_fp(get_parameter("max_velocity").as_double());
        params_.trajectory_size = 0;  // Set by FPGA
        params_.lookahead_points = get_parameter("lookahead_points").as_int();
    }
    
    bool load_trajectory(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) return false;
        
        trajectory_.clear();
        std::string line;
        std::getline(file, line);  // Skip header
        
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string token;
            WaypointCSV wp;
            
            std::getline(ss, token, ','); wp.s = std::stof(token);
            std::getline(ss, token, ','); wp.x = std::stof(token);
            std::getline(ss, token, ','); wp.y = std::stof(token);
            std::getline(ss, token, ','); wp.psi = std::stof(token);
            std::getline(ss, token, ','); wp.kappa = std::stof(token);
            std::getline(ss, token, ','); wp.vx = std::stof(token);
            std::getline(ss, token, ','); wp.ax = std::stof(token);
            
            trajectory_.push_back(wp);
        }
        
        return !trajectory_.empty();
    }
    
    void state_callback(const f1tenth_msgs::msg::MpcState::SharedPtr msg) {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Prepare minimal state input (only 32 bytes!)
        FpgaStateInput_t state;
        state.x_fp = msg->x_fp;
        state.y_fp = msg->y_fp;
        state.theta_fp = msg->theta_fp;
        state.velocity_fp = msg->velocity_fp;
        state.waypoint_index = msg->waypoint_index;
        state.timestamp_ms = msg->timestamp_ms;
        state.sequence_number = static_cast<uint32_t>(msg_count_);
        state.reserved = 0;
        
        // Compute control
        FpgaOutput_t output;
        
        if (use_fpga_) {
            if (!fpga_.compute(state, output)) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "FPGA compute failed");
                sw_fallback_.compute(state, output);
            }
        } else {
            sw_fallback_.compute(state, output);
        }
        
        // Publish drive command
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = now();
        drive_msg.header.frame_id = "base_link";
        drive_msg.drive.steering_angle = fp_to_float(output.steering_angle_fp);
        drive_msg.drive.speed = fp_to_float(output.velocity_fp);
        drive_pub_->publish(drive_msg);
        
        // Timing
        auto end_time = std::chrono::high_resolution_clock::now();
        auto compute_us = std::chrono::duration_cast<std::chrono::microseconds>(
            end_time - start_time).count();
        
        msg_count_++;
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        double latency_ms = static_cast<double>(now_ms - msg->timestamp_ms);
        total_latency_ms_ += latency_ms;
        
        if (msg_count_ % 100 == 0) {
            double avg_latency = total_latency_ms_ / msg_count_;
            RCLCPP_INFO(get_logger(),
                "[%s] Pos=(%.2f,%.2f) Vel=%.1f | Steer=%.1f° Speed=%.1f | "
                "CTE=%.3fm | %ldμs | Lat=%.1fms (avg=%.1f)",
                use_fpga_ ? "FPGA" : "SW",
                fp_to_float(msg->x_fp), fp_to_float(msg->y_fp),
                fp_to_float(msg->velocity_fp),
                fp_to_float(output.steering_angle_fp) * 57.3f,
                fp_to_float(output.velocity_fp),
                fp_to_float(output.cross_track_error_fp),
                compute_us, latency_ms, avg_latency);
        }
    }
};

} // namespace f1tenth_communication

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<f1tenth_communication::MpcReceiverFpgaNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
