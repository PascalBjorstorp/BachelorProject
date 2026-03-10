/**
 * @file mpc_receiver_mpc_fpga.cpp
 * @brief MPC Receiver with Riccati-ADMM FPGA Integration
 *
 * Runs on Ultra96-V2. Interfaces with the MPC FPGA IP core via /dev/mem mmap.
 *
 * Startup:
 *   1. Load trajectory CSV → write each waypoint to FPGA BRAM (mode=1)
 *   2. Finalize trajectory (mode=2)
 *
 * Runtime (per MpcState message):
 *   1. Write vehicle state to 8 AXI-Lite registers (mode=0)
 *   2. Start FPGA (AP_START)
 *   3. Wait for done (AP_DONE)
 *   4. Read steering + accel from output registers
 *   5. Publish AckermannDriveStamped to /drive
 *
 * When FPGA is unavailable, falls back to proportional Frenet controller.
 *
 * Register map: mpc_fpga_interface.h
 * Data format:  Q16.16 fixed-point (int32_t)
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
#include <cerrno>
#include <algorithm>

// Linux memory-mapped I/O
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

// MPC FPGA register map
#include "mpc_fpga_interface.h"

namespace f1tenth_communication {

/* Portability: some systems (e.g. Windows) don't define CLOCK_MONOTONIC_RAW.
 * Provide a safe fallback to CLOCK_MONOTONIC when RAW is unavailable. */
#ifndef CLOCK_MONOTONIC_RAW
#ifdef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC_RAW CLOCK_MONOTONIC
#endif
#endif


/*===========================================================================
 * Fixed-Point Helpers (Q16.16)
 *===========================================================================*/

static constexpr int32_t FP_SCALE = 65536;

inline float fp_to_float(int32_t fp) {
    return static_cast<float>(fp) / static_cast<float>(FP_SCALE);
}

inline int32_t float_to_fp(float f) {
    return static_cast<int32_t>(f * static_cast<float>(FP_SCALE));
}

/*===========================================================================
 * AP_CTRL bits (standard Vitis HLS AXI-Lite control)
 *===========================================================================*/

static constexpr uint32_t AP_START = 0x01;
static constexpr uint32_t AP_DONE  = 0x02;
static constexpr uint32_t AP_IDLE  = 0x04;

/*===========================================================================
 * Waypoint from CSV
 *===========================================================================*/

struct WaypointCSV {
    float s, x, y, psi, kappa, vx, ax;
};

/*===========================================================================
 * MPC FPGA AXI-Lite Interface
 *
 * Communicates with the Riccati-ADMM MPC IP via memory-mapped registers.
 * Uses the register map defined in mpc_fpga_interface.h.
 *===========================================================================*/

class MpcFpgaInterface {
public:
    MpcFpgaInterface() = default;
    ~MpcFpgaInterface() { close_device(); }

    MpcFpgaInterface(const MpcFpgaInterface&) = delete;
    MpcFpgaInterface& operator=(const MpcFpgaInterface&) = delete;

    /**
     * Open /dev/mem and mmap the FPGA register space.
     */
    bool initialize(uint32_t base_addr = MPC_FPGA_BASE_ADDR,
                    size_t map_size = 0x10000) {
        base_addr_ = base_addr;
        map_size_  = map_size;

        mem_fd_ = open("/dev/mem", O_RDWR | O_SYNC);
        if (mem_fd_ < 0) {
            fprintf(stderr, "MPC-FPGA: open /dev/mem failed: %s\n",
                    strerror(errno));
            return false;
        }

        fpga_base_ = mmap(nullptr, map_size_,
                           PROT_READ | PROT_WRITE,
                           MAP_SHARED, mem_fd_, base_addr_);
        if (fpga_base_ == MAP_FAILED) {
            fprintf(stderr, "MPC-FPGA: mmap at 0x%08X failed: %s\n",
                    base_addr_, strerror(errno));
            ::close(mem_fd_);
            mem_fd_ = -1;
            return false;
        }

        uint32_t ctrl = read_reg(REG_AP_CTRL);
        fprintf(stderr, "MPC-FPGA: mmap OK at 0x%08X, AP_CTRL=0x%08X\n",
                base_addr_, ctrl);

        initialized_ = true;
        return true;
    }

    void close_device() {
        if (fpga_base_ && fpga_base_ != MAP_FAILED) {
            munmap(fpga_base_, map_size_);
            fpga_base_ = nullptr;
        }
        if (mem_fd_ >= 0) {
            ::close(mem_fd_);
            mem_fd_ = -1;
        }
        initialized_ = false;
    }

    bool is_ready() const { return initialized_; }

    /**
     * Load full trajectory to FPGA BRAM.
     * Each waypoint written individually (mode=1), then finalized (mode=2).
     * Called once at startup.
     */
    bool load_trajectory(const std::vector<WaypointCSV>& waypoints,
                         int32_t default_left_bound_fp,
                         int32_t default_right_bound_fp) {
        if (!initialized_) return false;

        const size_t count = std::min(waypoints.size(),
                                       static_cast<size_t>(MPC_FPGA_MAX_TRAJECTORY_SIZE));

        for (size_t i = 0; i < count; ++i) {
            if (!wait_idle(50000)) {
                fprintf(stderr, "MPC-FPGA: idle timeout at wp %zu\n", i);
                return false;
            }

            write_reg(REG_MODE,            1);
            write_reg(REG_WP_INDEX,        static_cast<uint32_t>(i));
            write_reg(REG_WP_X,            static_cast<uint32_t>(float_to_fp(waypoints[i].x)));
            write_reg(REG_WP_Y,            static_cast<uint32_t>(float_to_fp(waypoints[i].y)));
            write_reg(REG_WP_PSI,          static_cast<uint32_t>(float_to_fp(waypoints[i].psi)));
            write_reg(REG_WP_VX,           static_cast<uint32_t>(float_to_fp(waypoints[i].vx)));
            write_reg(REG_WP_KAPPA,        static_cast<uint32_t>(float_to_fp(waypoints[i].kappa)));
            write_reg(REG_WP_AX,           static_cast<uint32_t>(float_to_fp(waypoints[i].ax)));
            write_reg(REG_WP_LEFT_BOUND,   static_cast<uint32_t>(default_left_bound_fp));
            write_reg(REG_WP_RIGHT_BOUND,  static_cast<uint32_t>(default_right_bound_fp));
            write_reg(REG_WP_TOTAL,        static_cast<uint32_t>(count));
            __sync_synchronize();

            write_reg(REG_AP_CTRL, AP_START);

            if (!wait_done(100000)) {
                fprintf(stderr, "MPC-FPGA: done timeout at wp %zu\n", i);
                return false;
            }
        }

        // Finalize trajectory (mode=2)
        if (!wait_idle(50000)) return false;
        write_reg(REG_MODE,     2);
        write_reg(REG_WP_TOTAL, static_cast<uint32_t>(count));
        __sync_synchronize();
        write_reg(REG_AP_CTRL, AP_START);
        if (!wait_done(100000)) return false;

        trajectory_size_   = count;
        trajectory_loaded_ = true;

        // Pre-set mode=0 for compute cycles
        write_reg(REG_MODE, 0);
        __sync_synchronize();

        return true;
    }

    /**
     * Run one MPC compute cycle.
     * Writes 8 vehicle-state registers, starts FPGA, reads 4 output registers.
     * Total register I/O: 8 writes + 1 start + 4 reads = minimal latency.
     */
    bool compute(int32_t x_fp, int32_t y_fp, int32_t theta_fp,
                 int32_t vx_fp, int32_t vy_fp, int32_t omega_fp,
                 int32_t steering_fp, uint32_t wp_idx,
                 int32_t& out_steering_fp, int32_t& out_accel_fp,
                 uint32_t& out_status, uint32_t& out_iterations) {
        if (!initialized_ || !trajectory_loaded_) return false;

        if (!wait_idle(10000)) return false;

        // --- Write mode=0 explicitly for robustness (in case of spurious resets) ---
        write_reg(REG_MODE, 0);

        // --- Write vehicle state (8 registers, mode=0 persists) ---
        write_reg(REG_ST_X,        static_cast<uint32_t>(x_fp));
        write_reg(REG_ST_Y,        static_cast<uint32_t>(y_fp));
        write_reg(REG_ST_THETA,    static_cast<uint32_t>(theta_fp));
        write_reg(REG_ST_VX,       static_cast<uint32_t>(vx_fp));
        write_reg(REG_ST_VY,       static_cast<uint32_t>(vy_fp));
        write_reg(REG_ST_OMEGA,    static_cast<uint32_t>(omega_fp));
        write_reg(REG_ST_STEERING, static_cast<uint32_t>(steering_fp));
        write_reg(REG_ST_WP_IDX,   wp_idx);
        __sync_synchronize();

        // --- Measure FPGA compute time ---
        struct timespec t0, t1;
        clock_gettime(CLOCK_MONOTONIC_RAW, &t0);

        write_reg(REG_AP_CTRL, AP_START);

        if (!wait_done(200000)) {
            last_compute_ns_ = -1;
            return false;
        }

        clock_gettime(CLOCK_MONOTONIC_RAW, &t1);
        last_compute_ns_ = (t1.tv_sec - t0.tv_sec) * 1000000000LL
                          + (t1.tv_nsec - t0.tv_nsec);

        // --- Read output registers ---
        out_steering_fp = static_cast<int32_t>(read_reg(REG_OUT_STEERING));
        out_accel_fp    = static_cast<int32_t>(read_reg(REG_OUT_ACCEL));
        out_status      = read_reg(REG_OUT_STATUS);
        out_iterations  = read_reg(REG_OUT_ITERATIONS);

        return true;
    }

    int64_t get_last_compute_ns()   const { return last_compute_ns_; }
    size_t  get_trajectory_size()   const { return trajectory_size_; }
    bool    is_trajectory_loaded()  const { return trajectory_loaded_; }

private:
    int     mem_fd_       = -1;
    void*   fpga_base_    = nullptr;
    uint32_t base_addr_   = 0;
    size_t  map_size_     = 0;
    bool    initialized_  = false;
    size_t  trajectory_size_   = 0;
    bool    trajectory_loaded_ = false;
    int64_t last_compute_ns_   = 0;

    void write_reg(uint32_t offset, uint32_t value) {
        volatile uint32_t* reg = reinterpret_cast<volatile uint32_t*>(
            static_cast<volatile uint8_t*>(fpga_base_) + offset);
        *reg = value;
    }

    uint32_t read_reg(uint32_t offset) {
        volatile uint32_t* reg = reinterpret_cast<volatile uint32_t*>(
            static_cast<volatile uint8_t*>(fpga_base_) + offset);
        return *reg;
    }

    bool wait_idle(int timeout_cycles) {
        while (timeout_cycles-- > 0) {
            if (read_reg(REG_AP_CTRL) & AP_IDLE) return true;
        }
        return false;
    }

    bool wait_done(int timeout_cycles) {
        while (timeout_cycles-- > 0) {
            if (read_reg(REG_AP_CTRL) & AP_DONE) return true;
        }
        return false;
    }
};

/*===========================================================================
 * Software Fallback —  Proportional Frenet Controller
 * Used when FPGA is unavailable or initialization fails.
 *===========================================================================*/

class SoftwareFallback {
public:
    void set_trajectory(const std::vector<WaypointCSV>& wp) { traj_ = wp; }

    void compute(float x, float y, float theta, float /* vx */,
                 uint32_t wp_idx,
                 float max_steer, float max_vel,
                 float K_ey, float K_epsi, float K_slow,
                 float& out_steering, float& out_speed) {
        if (traj_.empty()) {
            out_steering = 0.0f;
            out_speed    = 0.0f;
            return;
        }

        const size_t idx = wp_idx % traj_.size();
        const float wx   = traj_[idx].x;
        const float wy   = traj_[idx].y;
        const float wpsi = traj_[idx].psi;

        const float dx  = x - wx;
        const float dy  = y - wy;
        const float e_y = -std::sin(wpsi) * dx + std::cos(wpsi) * dy;

        float e_psi = theta - wpsi;
        while (e_psi >  static_cast<float>(M_PI)) e_psi -= 2.0f * static_cast<float>(M_PI);
        while (e_psi < -static_cast<float>(M_PI)) e_psi += 2.0f * static_cast<float>(M_PI);

        out_steering = std::clamp(-K_ey * e_y - K_epsi * e_psi,
                                   -max_steer, max_steer);

        const float v_ref = std::min(traj_[idx].vx, max_vel);
        out_speed = v_ref * (1.0f - K_slow * std::min(std::abs(e_y), 1.0f));
        out_speed = std::max(out_speed, 0.0f);
    }

private:
    std::vector<WaypointCSV> traj_;
};

/*===========================================================================
 * MPC Receiver MPC FPGA Node
 *===========================================================================*/

class MpcReceiverMpcFpgaNode : public rclcpp::Node {
public:
    MpcReceiverMpcFpgaNode() : Node("mpc_receiver_mpc_fpga") {
        // --- Parameters ---
        declare_parameter("trajectory_file", "");
        declare_parameter("input_topic", "/mpc_state");
        declare_parameter("drive_topic", "/drive");
        declare_parameter("use_fpga", true);
        declare_parameter("fpga_base_address",
                           static_cast<int64_t>(MPC_FPGA_BASE_ADDR));
        declare_parameter("horizon", 20);

        // Vehicle / controller
        declare_parameter("max_steering", 0.4189);
        declare_parameter("max_velocity", 6.0);
        declare_parameter("wheelbase", 0.324);
        declare_parameter("track_half_width", 2.0);  // Default track bounds [m]

        // Software fallback gains
        declare_parameter("K_ey", 1.0);
        declare_parameter("K_epsi", 1.5);
        declare_parameter("K_slowdown", 0.5);

        // Control interval for speed = vx + accel * dt
        declare_parameter("control_dt", 0.02);  // [s] (default 50 Hz state rate)

        // Watchdog timeout: zero drive if no state received
        declare_parameter("watchdog_timeout_ms", 100.0);  // [ms]

        // --- Read parameters ---
        auto trajectory_file = get_parameter("trajectory_file").as_string();
        auto input_topic     = get_parameter("input_topic").as_string();
        auto drive_topic     = get_parameter("drive_topic").as_string();
        use_fpga_            = get_parameter("use_fpga").as_bool();
        max_steering_        = static_cast<float>(get_parameter("max_steering").as_double());
        max_velocity_        = static_cast<float>(get_parameter("max_velocity").as_double());
        control_dt_          = static_cast<float>(get_parameter("control_dt").as_double());

        // Cache software fallback gains at startup (avoids per-callback parameter lookups)
        K_ey_       = static_cast<float>(get_parameter("K_ey").as_double());
        K_epsi_     = static_cast<float>(get_parameter("K_epsi").as_double());
        K_slowdown_ = static_cast<float>(get_parameter("K_slowdown").as_double());

        const float track_hw = static_cast<float>(
            get_parameter("track_half_width").as_double());
        const int32_t left_bound_fp  = float_to_fp(track_hw);
        const int32_t right_bound_fp = float_to_fp(track_hw);

        if (trajectory_file.empty()) {
            RCLCPP_ERROR(get_logger(), "No trajectory file specified!");
            return;
        }

        if (!load_trajectory(trajectory_file)) {
            RCLCPP_ERROR(get_logger(), "Failed to load trajectory: %s",
                         trajectory_file.c_str());
            return;
        }

        RCLCPP_INFO(get_logger(), "Loaded %zu waypoints from %s (hash=0x%08X)",
                     trajectory_.size(), trajectory_file.c_str(), trajectory_hash_);

        // --- Initialize FPGA ---
        if (use_fpga_) {
            const uint32_t addr = static_cast<uint32_t>(
                get_parameter("fpga_base_address").as_int());

            if (fpga_.initialize(addr)) {
                if (fpga_.load_trajectory(trajectory_,
                                           left_bound_fp, right_bound_fp)) {
                    RCLCPP_INFO(get_logger(),
                        "MPC FPGA init OK at 0x%08X, %zu waypoints in BRAM",
                        addr, fpga_.get_trajectory_size());
                } else {
                    RCLCPP_WARN(get_logger(),
                        "Failed to load trajectory to MPC FPGA — SW fallback");
                    use_fpga_ = false;
                }
            } else {
                RCLCPP_WARN(get_logger(),
                    "MPC FPGA init failed — SW fallback");
                use_fpga_ = false;
            }
        }

        if (!use_fpga_) {
            sw_fallback_.set_trajectory(trajectory_);
            RCLCPP_INFO(get_logger(), "Using software Frenet controller fallback");
        }

        // --- Create publisher & subscriber ---
        // Use SystemDefaultsQoS (Reliable) to match ackermann_mux subscriber
        drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic, rclcpp::SystemDefaultsQoS());

        auto qos = rclcpp::QoS(1).best_effort().durability_volatile();
        sub_ = create_subscription<f1tenth_msgs::msg::MpcState>(
            input_topic, qos,
            std::bind(&MpcReceiverMpcFpgaNode::state_callback, this,
                      std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
            "MPC Receiver [%s] ready.  %s → %s",
            use_fpga_ ? "FPGA" : "SW", input_topic.c_str(), drive_topic.c_str());

        // --- Safety watchdog timer ---
        double watchdog_ms = get_parameter("watchdog_timeout_ms").as_double();
        watchdog_timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(watchdog_ms)),
            [this]() {
                auto now = std::chrono::steady_clock::now();
                double elapsed_ms = std::chrono::duration<double, std::milli>(
                    now - last_msg_time_).count();
                double timeout_ms = get_parameter("watchdog_timeout_ms").as_double();
                if (elapsed_ms > timeout_ms && msg_count_ > 0) {
                    // State messages have stopped — zero the command for safety
                    auto drive = ackermann_msgs::msg::AckermannDriveStamped();
                    drive.header.stamp = this->now();
                    drive.header.frame_id = "base_link";
                    drive.drive.steering_angle = 0.0f;
                    drive.drive.speed = 0.0f;
                    drive.drive.acceleration = 0.0f;
                    drive_pub_->publish(drive);
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                        "WATCHDOG: No state received for %.0f ms — zeroing drive", elapsed_ms);
                }
            });
        RCLCPP_INFO(get_logger(), "Watchdog timer: %.0f ms timeout", watchdog_ms);
    }

private:
    std::vector<WaypointCSV> trajectory_;
    uint32_t trajectory_hash_ = 0;  // Checksum for cross-node trajectory verification
    bool  use_fpga_      = true;
    float max_steering_   = 0.4189f;
    float max_velocity_   = 6.0f;
    float control_dt_     = 0.02f;   // Default control interval for speed integration [s]

    // Cached software fallback gains (read once at startup)
    float K_ey_       = 1.0f;
    float K_epsi_     = 1.5f;
    float K_slowdown_ = 0.5f;

    MpcFpgaInterface    fpga_;
    SoftwareFallback    sw_fallback_;

    rclcpp::Subscription<f1tenth_msgs::msg::MpcState>::SharedPtr sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    uint64_t msg_count_       = 0;
    double   total_latency_ms_ = 0.0;
    std::chrono::steady_clock::time_point last_msg_time_ = std::chrono::steady_clock::now();
    rclcpp::Time last_callback_time_;   // For computing actual elapsed dt
    bool has_prev_callback_ = false;    // True after first callback

    /*-----------------------------------------------------------------------
     * Load trajectory CSV
     *---------------------------------------------------------------------*/
    bool load_trajectory(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) return false;

        trajectory_.clear();
        std::string line;
        std::getline(file, line);  // skip header

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string tok;
            WaypointCSV wp{};

            std::getline(ss, tok, ','); wp.s     = std::stof(tok);
            std::getline(ss, tok, ','); wp.x     = std::stof(tok);
            std::getline(ss, tok, ','); wp.y     = std::stof(tok);
            std::getline(ss, tok, ','); wp.psi   = std::stof(tok);
            std::getline(ss, tok, ','); wp.kappa = std::stof(tok);
            std::getline(ss, tok, ','); wp.vx    = std::stof(tok);
            std::getline(ss, tok, ','); wp.ax    = std::stof(tok);

            trajectory_.push_back(wp);
        }

        // Compute trajectory checksum for cross-node verification
        trajectory_hash_ = 0;
        for (const auto& wp : trajectory_) {
            trajectory_hash_ ^= static_cast<uint32_t>(wp.x * 65536.0f)
                              ^ (static_cast<uint32_t>(wp.y * 65536.0f) << 16);
        }
        return !trajectory_.empty();
    }

    /*-----------------------------------------------------------------------
     * State callback  —  FPGA compute → publish drive
     *---------------------------------------------------------------------*/
    void state_callback(const f1tenth_msgs::msg::MpcState::SharedPtr msg) {
        auto t_start = std::chrono::high_resolution_clock::now();
        last_msg_time_ = std::chrono::steady_clock::now();

        // Compute actual elapsed dt from message timestamps for speed integration.
        // Falls back to the configured control_dt_ for the first callback.
        float actual_dt = control_dt_;
        rclcpp::Time msg_time(msg->header.stamp);
        if (has_prev_callback_) {
            double dt_sec = (msg_time - last_callback_time_).seconds();
            // Sanity-check: only use measured dt if it is within [0.1 ms, 200 ms]
            if (dt_sec > 0.0001 && dt_sec < 0.2) {
                actual_dt = static_cast<float>(dt_sec);
            }
        }
        last_callback_time_ = msg_time;
        has_prev_callback_ = true;

        float    steering = 0.0f;
        float    speed    = 0.0f;
        float    accel    = 0.0f;
        uint32_t status   = 0;
        uint32_t iters    = 0;

        if (use_fpga_) {
            int32_t out_steer_fp = 0;
            int32_t out_accel_fp = 0;

            bool ok = fpga_.compute(
                msg->x_fp, msg->y_fp, msg->theta_fp,
                msg->velocity_fp, msg->vy_fp, msg->omega_fp,
                msg->steering_angle_fp, msg->waypoint_index,
                out_steer_fp, out_accel_fp, status, iters);

            if (ok) {
                steering = fp_to_float(out_steer_fp);
                accel    = fp_to_float(out_accel_fp);

                // Speed: integrate current velocity + MPC acceleration output.
                // The VESC interprets drive.speed as target velocity and uses
                // its own PID to reach it.  This mirrors the approach in MPC/.
                float vx = fp_to_float(msg->velocity_fp);
                float v_target = vx + accel * actual_dt;
                speed = std::max(0.0f, std::min(v_target, max_velocity_));
            } else {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "FPGA compute failed — using SW fallback");
                run_sw_fallback(msg, steering, speed);
            }
        } else {
            run_sw_fallback(msg, steering, speed);
        }

        // Clamp outputs
        steering = std::clamp(steering, -max_steering_, max_steering_);
        speed    = std::clamp(speed,    0.0f,           max_velocity_);

        // --- Publish AckermannDriveStamped ---
        auto drive = ackermann_msgs::msg::AckermannDriveStamped();
        drive.header.stamp    = now();
        drive.header.frame_id = "base_link";
        drive.drive.steering_angle = steering;
        drive.drive.speed          = speed;
        drive.drive.acceleration   = accel;
        drive_pub_->publish(drive);

        // --- Timing & logging ---
        auto t_end      = std::chrono::high_resolution_clock::now();
        auto compute_us = std::chrono::duration_cast<std::chrono::microseconds>(
                              t_end - t_start).count();
        msg_count_++;

        auto now_ms = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
        double latency_ms = static_cast<double>(now_ms - msg->timestamp_ms);
        total_latency_ms_ += latency_ms;

        if (msg_count_ % 100 == 0) {
            double avg = total_latency_ms_ / static_cast<double>(msg_count_);
            int64_t fpga_ns = use_fpga_ ? fpga_.get_last_compute_ns() : 0;
            RCLCPP_INFO(get_logger(),
                "[%s] WP=%u  delta=%.1f deg  v=%.1f  a=%.1f | "
                "Status=%u  Iter=%u | Total=%ld us  FPGA=%ld ns | "
                "Lat %.1f ms (avg %.1f)",
                use_fpga_ ? "FPGA" : "SW",
                msg->waypoint_index,
                steering * 57.2958f, speed, accel,
                status, iters,
                compute_us, fpga_ns,
                latency_ms, avg);
        }
    }

    /*-----------------------------------------------------------------------
     * Software fallback path
     *---------------------------------------------------------------------*/
    void run_sw_fallback(const f1tenth_msgs::msg::MpcState::SharedPtr& msg,
                         float& steering, float& speed) {
        const float x     = fp_to_float(msg->x_fp);
        const float y     = fp_to_float(msg->y_fp);
        const float theta = fp_to_float(msg->theta_fp);
        const float vx    = fp_to_float(msg->velocity_fp);

        sw_fallback_.compute(
            x, y, theta, vx, msg->waypoint_index,
            max_steering_, max_velocity_,
            K_ey_, K_epsi_, K_slowdown_,
            steering, speed);
    }
};

}  // namespace f1tenth_communication

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<f1tenth_communication::MpcReceiverMpcFpgaNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
