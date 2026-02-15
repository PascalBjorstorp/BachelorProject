/**
 * @file mpc_receiver_fpga.cpp
 * @brief MPC Receiver with FPGA Integration - Trajectory Stored in FPGA BRAM
 *
 * Loads trajectory to FPGA BRAM at startup via AXI-Lite register writes.
 * Each waypoint is loaded one at a time (mode=1), then finalized (mode=2).
 * Runtime: Only writes vehicle state registers (mode=0), gets steering output.
 *
 * Flow:
 *   Startup: Load trajectory CSV → Per-register writes to FPGA BRAM (once)
 *   Runtime: Receive MpcState → Write state regs → Start FPGA → Read output → Publish /drive
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
 * AP_CTRL bits (standard Vitis HLS AXI-Lite control)
 *===========================================================================*/
#define AP_START  0x01
#define AP_DONE   0x02
#define AP_IDLE   0x04

/*===========================================================================
 * FPGA Interface - AXI-Lite Register Access
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
     * @brief Load trajectory to FPGA BRAM via AXI-Lite registers
     * 
     * Each waypoint is loaded individually (mode=1), then finalized (mode=2).
     * This is done once at startup.
     */
    bool load_trajectory(const std::vector<WaypointCSV>& waypoints) {
        if (!initialized_) return false;
        
        size_t count = std::min(waypoints.size(), static_cast<size_t>(MAX_TRAJECTORY_SIZE));
        
        // Load each waypoint one at a time (mode=1)
        for (size_t i = 0; i < count; i++) {
            // Wait for idle
            if (!wait_idle(1000)) return false;
            
            // Write waypoint registers
            write_reg(REG_MODE,     1);  // mode = LOAD_WAYPOINT
            write_reg(REG_WP_INDEX, static_cast<uint32_t>(i));
            write_reg(REG_WP_X,     static_cast<uint32_t>(float_to_fp(waypoints[i].x)));
            write_reg(REG_WP_Y,     static_cast<uint32_t>(float_to_fp(waypoints[i].y)));
            write_reg(REG_WP_THETA, static_cast<uint32_t>(float_to_fp(waypoints[i].psi)));
            write_reg(REG_WP_VEL,   static_cast<uint32_t>(float_to_fp(waypoints[i].vx)));
            write_reg(REG_WP_KAPPA, static_cast<uint32_t>(float_to_fp(waypoints[i].kappa)));
            write_reg(REG_WP_TOTAL, static_cast<uint32_t>(count));
            __sync_synchronize();
            
            // Start
            write_reg(REG_AP_CTRL, AP_START);
            
            // Wait for done
            if (!wait_done(10000)) return false;
        }
        
        // Finalize trajectory (mode=2)
        if (!wait_idle(1000)) return false;
        write_reg(REG_MODE,     2);
        write_reg(REG_WP_TOTAL, static_cast<uint32_t>(count));
        __sync_synchronize();
        write_reg(REG_AP_CTRL, AP_START);
        if (!wait_done(10000)) return false;
        
        // Check that trajectory was loaded
        uint32_t loaded = read_reg(REG_OUT_TRAJ_LOADED);
        uint32_t size   = read_reg(REG_OUT_TRAJ_SIZE);
        
        trajectory_size_ = size;
        trajectory_loaded_ = (loaded == 1);
        
        return trajectory_loaded_;
    }
    
    /**
     * @brief Write control parameters to FPGA registers (call once at startup,
     *        or when tuning changes). AXI-Lite registers retain their values
     *        between transactions, so these don't need to be re-sent each cycle.
     */
    void set_parameters(const FpgaParams_t& params) {
        params_ = params;
        params_.trajectory_size = trajectory_size_;
        
        if (!initialized_) return;
        
        // Wait for idle before writing
        wait_idle(1000);
        
        // Write parameter registers (persist until overwritten)
        write_reg(REG_P_MIN_LA,    static_cast<uint32_t>(params_.min_lookahead_fp));
        write_reg(REG_P_MAX_LA,    static_cast<uint32_t>(params_.max_lookahead_fp));
        write_reg(REG_P_LA_GAIN,   static_cast<uint32_t>(params_.lookahead_gain_fp));
        write_reg(REG_P_WHEELBASE, static_cast<uint32_t>(params_.wheelbase_fp));
        write_reg(REG_P_MAX_STEER, static_cast<uint32_t>(params_.max_steering_fp));
        write_reg(REG_P_MAX_VEL,   static_cast<uint32_t>(params_.max_velocity_fp));
        write_reg(REG_P_LA_POINTS, params_.lookahead_points);
        
        // Pre-set mode=0 for compute (persists for all subsequent calls)
        write_reg(REG_MODE, 0);
        __sync_synchronize();
    }
    
    /**
     * @brief Compute control output via FPGA (called every cycle)
     * 
     * Only writes the 5 vehicle state registers that change each cycle.
     * Mode and parameter registers persist from set_parameters().
     * 
     * Optionally measures FPGA-only compute time (AP_START → AP_DONE).
     */
    bool compute(const FpgaStateInput_t& state, FpgaOutput_t& output) {
        if (!initialized_ || !trajectory_loaded_) return false;
        
        // Wait for idle
        if (!wait_idle(1000)) return false;
        
        // Only write vehicle state registers (5 writes per cycle)
        write_reg(REG_ST_X,      static_cast<uint32_t>(state.x_fp));
        write_reg(REG_ST_Y,      static_cast<uint32_t>(state.y_fp));
        write_reg(REG_ST_THETA,  static_cast<uint32_t>(state.theta_fp));
        write_reg(REG_ST_VEL,    static_cast<uint32_t>(state.velocity_fp));
        write_reg(REG_ST_WP_IDX, state.waypoint_index);
        __sync_synchronize();
        
        // Measure FPGA compute time (AP_START → AP_DONE)
        struct timespec t_start, t_done;
        clock_gettime(CLOCK_MONOTONIC_RAW, &t_start);
        
        // Start computation
        write_reg(REG_AP_CTRL, AP_START);
        
        // Wait for done
        if (!wait_done(10000)) return false;
        
        clock_gettime(CLOCK_MONOTONIC_RAW, &t_done);
        last_fpga_compute_ns_ = (t_done.tv_sec - t_start.tv_sec) * 1000000000LL
                              + (t_done.tv_nsec - t_start.tv_nsec);
        
        // Read output registers
        output.steering_angle_fp    = static_cast<int32_t>(read_reg(REG_OUT_STEERING));
        output.velocity_fp          = static_cast<int32_t>(read_reg(REG_OUT_VELOCITY));
        output.cross_track_error_fp = static_cast<int32_t>(read_reg(REG_OUT_CTE));
        output.heading_error_fp     = static_cast<int32_t>(read_reg(REG_OUT_HEADING_ERR));
        output.lookahead_dist_fp    = static_cast<int32_t>(read_reg(REG_OUT_LOOKAHEAD));
        output.target_waypoint_idx  = read_reg(REG_OUT_TARGET_WP);
        output.status               = read_reg(REG_OUT_STATUS);
        output.sequence_number      = state.sequence_number;
        output.timestamp_ms         = state.timestamp_ms;
        
        return true;
    }
    
    size_t get_trajectory_size() const { return trajectory_size_; }
    bool is_trajectory_loaded() const { return trajectory_loaded_; }
    
    /** Get last FPGA compute time (AP_START → AP_DONE) in nanoseconds */
    int64_t get_last_compute_ns() const { return last_fpga_compute_ns_; }
    
private:
    int mem_fd_;
    void* fpga_base_;
    uint32_t base_addr_;
    size_t map_size_;
    bool initialized_;
    size_t trajectory_size_ = 0;
    bool trajectory_loaded_ = false;
    FpgaParams_t params_;
    int64_t last_fpga_compute_ns_ = 0;
    
    /** Write a 32-bit value to an AXI-Lite register */
    void write_reg(uint32_t offset, uint32_t value) {
        volatile uint32_t* reg = reinterpret_cast<volatile uint32_t*>(
            static_cast<volatile uint8_t*>(fpga_base_) + offset);
        *reg = value;
    }
    
    /** Read a 32-bit value from an AXI-Lite register */
    uint32_t read_reg(uint32_t offset) {
        volatile uint32_t* reg = reinterpret_cast<volatile uint32_t*>(
            static_cast<volatile uint8_t*>(fpga_base_) + offset);
        return *reg;
    }
    
    /** Wait for AP_CTRL idle bit */
    bool wait_idle(int timeout) {
        while (timeout-- > 0) {
            if (read_reg(REG_AP_CTRL) & AP_IDLE) return true;
        }
        return false;
    }
    
    /** Wait for AP_CTRL done bit */
    bool wait_done(int timeout) {
        while (timeout-- > 0) {
            if (read_reg(REG_AP_CTRL) & AP_DONE) return true;
        }
        return false;
    }
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
            int64_t fpga_ns = use_fpga_ ? fpga_.get_last_compute_ns() : 0;
            RCLCPP_INFO(get_logger(),
                "[%s] Pos=(%.2f,%.2f) Vel=%.1f | Steer=%.1f° Speed=%.1f | "
                "CTE=%.3fm | Total=%ldμs FPGA=%ldns | Lat=%.1fms (avg=%.1f)",
                use_fpga_ ? "FPGA" : "SW",
                fp_to_float(msg->x_fp), fp_to_float(msg->y_fp),
                fp_to_float(msg->velocity_fp),
                fp_to_float(output.steering_angle_fp) * 57.3f,
                fp_to_float(output.velocity_fp),
                fp_to_float(output.cross_track_error_fp),
                compute_us, fpga_ns, latency_ms, avg_latency);
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
