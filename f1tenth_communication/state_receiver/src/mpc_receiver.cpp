/**
 * @file mpc_receiver.cpp
 * @brief MPC Receiver with Riccati-ADMM FPGA Integration
 *
 * Runs on Ultra96-V2. Interfaces with the MPC FPGA IP core via /dev/mem mmap.
 *
 * Startup:
 *   1. Initialize FPGA register interface
 *
 * Runtime (per MpcState message):
 *   1. Copy streamed horizon references into mapped BRAM buffers
 *   2. Write state + buffer pointers through AXI-Lite
 *   3. Start FPGA (AP_START)
 *   4. Wait for done (AP_DONE)
 *   5. Read steering + accel from output registers
 *   6. Publish AckermannDriveStamped to /drive
 *
 * When FPGA is unavailable, falls back to proportional Frenet controller.
 *
 * Register map: mpc_fpga_interface.h 
 * Data format:  Q16.16 fixed-point (int32_t)
 */

#include <rclcpp/rclcpp.hpp>
#include <f1tenth_msgs/msg/mpc_state.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include <string>
#include <vector>
#include <chrono>
#include <cstring>
#include <cmath>
#include <cerrno>
#include <algorithm>
#include <limits>
#include <stdexcept>

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
        if (ref_vx_map_ && ref_vx_map_ != MAP_FAILED) {
            munmap(ref_vx_map_, ref_vx_map_size_);
            ref_vx_map_ = nullptr;
        }
        if (ref_kappa_map_ && ref_kappa_map_ != MAP_FAILED) {
            munmap(ref_kappa_map_, ref_kappa_map_size_);
            ref_kappa_map_ = nullptr;
        }
        if (ref_left_map_ && ref_left_map_ != MAP_FAILED) {
            munmap(ref_left_map_, ref_left_map_size_);
            ref_left_map_ = nullptr;
        }
        if (ref_right_map_ && ref_right_map_ != MAP_FAILED) {
            munmap(ref_right_map_, ref_right_map_size_);
            ref_right_map_ = nullptr;
        }
        ref_vx_buf_ = nullptr;
        ref_kappa_buf_ = nullptr;
        ref_left_buf_ = nullptr;
        ref_right_buf_ = nullptr;

        if (fpga_base_ && fpga_base_ != MAP_FAILED) {
            munmap(fpga_base_, map_size_);
            fpga_base_ = nullptr;
        }
        if (mem_fd_ >= 0) {
            ::close(mem_fd_);
            mem_fd_ = -1;
        }
        initialized_ = false;
        buffers_ready_ = false;
        ref_count_ = 0;
    }

    bool is_ready() const { return initialized_; }

    bool configure_reference_buffers(uint64_t ref_vx_phys,
                                     uint64_t ref_kappa_phys,
                                     uint64_t ref_left_phys,
                                     uint64_t ref_right_phys,
                                     size_t capacity_points) {
        if (!initialized_) return false;
        if (capacity_points == 0 || capacity_points > MPC_FPGA_MAX_REF_POINTS) return false;
        if (ref_vx_phys == 0 || ref_kappa_phys == 0 || ref_left_phys == 0 || ref_right_phys == 0) return false;

        const size_t bytes = capacity_points * sizeof(int32_t);
        ref_capacity_ = capacity_points;
        ref_vx_phys_ = ref_vx_phys;
        ref_kappa_phys_ = ref_kappa_phys;
        ref_left_phys_ = ref_left_phys;
        ref_right_phys_ = ref_right_phys;

        ref_vx_buf_ = static_cast<int32_t*>(map_physical_buffer(ref_vx_phys_, bytes, ref_vx_map_, ref_vx_map_size_));
        if (!ref_vx_buf_) return false;
        ref_kappa_buf_ = static_cast<int32_t*>(map_physical_buffer(ref_kappa_phys_, bytes, ref_kappa_map_, ref_kappa_map_size_));
        if (!ref_kappa_buf_) return false;
        ref_left_buf_ = static_cast<int32_t*>(map_physical_buffer(ref_left_phys_, bytes, ref_left_map_, ref_left_map_size_));
        if (!ref_left_buf_) return false;
        ref_right_buf_ = static_cast<int32_t*>(map_physical_buffer(ref_right_phys_, bytes, ref_right_map_, ref_right_map_size_));
        if (!ref_right_buf_) return false;

        buffers_ready_ = true;
        return true;
    }

    /**
     * Fill mapped reference buffers for one horizon frame.
     */
    bool load_horizon(const f1tenth_msgs::msg::MpcState& msg) {
        if (!initialized_ || !buffers_ready_) return false;

        size_t count = static_cast<size_t>(msg.horizon_length);
        count = std::min(count, msg.ref_x_fp.size());
        count = std::min(count, msg.ref_y_fp.size());
        count = std::min(count, msg.ref_psi_fp.size());
        count = std::min(count, msg.ref_vx_fp.size());
        count = std::min(count, msg.ref_kappa_fp.size());
        count = std::min(count, msg.ref_left_bound_fp.size());
        count = std::min(count, msg.ref_right_bound_fp.size());
        count = std::min(count, ref_capacity_);
        if (count == 0) return false;

        for (size_t i = 0; i < count; ++i) {
            ref_vx_buf_[i] = msg.ref_vx_fp[i];
            ref_kappa_buf_[i] = msg.ref_kappa_fp[i];
            ref_left_buf_[i] = msg.ref_left_bound_fp[i];
            ref_right_buf_[i] = msg.ref_right_bound_fp[i];
        }
        __sync_synchronize();

        ref_count_ = count;

        return true;
    }

    /**
     * Run one MPC compute cycle.
        * Writes vehicle-state registers, starts FPGA, reads 4 output registers.
     */
    bool compute(int32_t x_fp, int32_t theta_fp,
                 int32_t vx_fp, int32_t vy_fp, int32_t omega_fp,
                 int32_t steering_fp,
                 int32_t& out_steering_fp, int32_t& out_accel_fp,
                 uint32_t& out_status, uint32_t& out_iterations) {
        if (!initialized_ || !buffers_ready_ || ref_count_ == 0) return false;

        if (!wait_idle(10000)) return false;

        // --- Write bulk reference pointers and count ---
        write_reg64(REG_REF_VX_MEM_LO, ref_vx_phys_);
        write_reg64(REG_REF_KAPPA_MEM_LO, ref_kappa_phys_);
        write_reg64(REG_REF_LEFT_MEM_LO, ref_left_phys_);
        write_reg64(REG_REF_RIGHT_MEM_LO, ref_right_phys_);
        write_reg(REG_REF_COUNT, static_cast<uint32_t>(ref_count_));

        // --- Write vehicle state registers ---
        write_reg(REG_ST_X,        static_cast<uint32_t>(x_fp));
        write_reg(REG_ST_THETA,    static_cast<uint32_t>(theta_fp));
        write_reg(REG_ST_VX,       static_cast<uint32_t>(vx_fp));
        write_reg(REG_ST_VY,       static_cast<uint32_t>(vy_fp));
        write_reg(REG_ST_OMEGA,    static_cast<uint32_t>(omega_fp));
        write_reg(REG_ST_STEERING, static_cast<uint32_t>(steering_fp));
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
    size_t  get_reference_count() const { return ref_count_; }
    bool    has_reference_frame() const { return ref_count_ > 0; }

private:
    int     mem_fd_       = -1;
    void*   fpga_base_    = nullptr;
    uint32_t base_addr_   = 0;
    size_t  map_size_     = 0;
    bool    initialized_  = false;
    size_t  ref_count_        = 0;
    size_t  ref_capacity_     = 0;
    bool    buffers_ready_    = false;
    int64_t last_compute_ns_   = 0;

    uint64_t ref_vx_phys_     = 0;
    uint64_t ref_kappa_phys_  = 0;
    uint64_t ref_left_phys_   = 0;
    uint64_t ref_right_phys_  = 0;

    void* ref_vx_map_ = nullptr;
    void* ref_kappa_map_ = nullptr;
    void* ref_left_map_ = nullptr;
    void* ref_right_map_ = nullptr;
    size_t ref_vx_map_size_ = 0;
    size_t ref_kappa_map_size_ = 0;
    size_t ref_left_map_size_ = 0;
    size_t ref_right_map_size_ = 0;

    int32_t* ref_vx_buf_ = nullptr;
    int32_t* ref_kappa_buf_ = nullptr;
    int32_t* ref_left_buf_ = nullptr;
    int32_t* ref_right_buf_ = nullptr;

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

    void write_reg64(uint32_t low_offset, uint64_t value) {
        write_reg(low_offset, static_cast<uint32_t>(value & 0xFFFFFFFFULL));
        write_reg(low_offset + 4, static_cast<uint32_t>((value >> 32) & 0xFFFFFFFFULL));
    }

    void* map_physical_buffer(uint64_t phys_addr, size_t bytes, void*& map_base, size_t& map_size) {
        const long page_size = sysconf(_SC_PAGESIZE);
        const uint64_t page_mask = static_cast<uint64_t>(page_size - 1);
        const uint64_t page_base = phys_addr & ~page_mask;
        const uint64_t page_off = phys_addr - page_base;
        const size_t total_map = static_cast<size_t>(page_off + bytes);

        map_base = mmap(nullptr, total_map, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd_, static_cast<off_t>(page_base));
        if (map_base == MAP_FAILED) {
            map_base = nullptr;
            map_size = 0;
            return nullptr;
        }
        map_size = total_map;
        return static_cast<void*>(static_cast<uint8_t*>(map_base) + page_off);
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
 * MPC Receiver MPC FPGA Node
 *===========================================================================*/

class MpcReceiverFpgaNode : public rclcpp::Node {
public:
    MpcReceiverFpgaNode() : Node("mpc_receiver") {
        // --- Parameters ---
        declare_parameter("input_topic", "/mpc_state");
        declare_parameter("drive_topic", "/drive");
        declare_parameter("fpga_base_address",
                           static_cast<int64_t>(MPC_FPGA_BASE_ADDR));
        declare_parameter("ref_vx_phys_addr", static_cast<int64_t>(0));
        declare_parameter("ref_kappa_phys_addr", static_cast<int64_t>(0));
        declare_parameter("ref_left_phys_addr", static_cast<int64_t>(0));
        declare_parameter("ref_right_phys_addr", static_cast<int64_t>(0));
        declare_parameter("ref_buffer_capacity", static_cast<int>(64));

        // Vehicle / controller
        declare_parameter("max_steering", 0.4189);
        declare_parameter("max_velocity", 6.0);

        // Control interval for speed = vx + accel * dt
        declare_parameter("control_dt", 0.02);  // [s] (default 50 Hz state rate)

        // Watchdog timeout: zero drive if no state received
        declare_parameter("watchdog_timeout_ms", 100.0);  // [ms]

        // --- Read parameters ---
        auto input_topic     = get_parameter("input_topic").as_string();
        auto drive_topic     = get_parameter("drive_topic").as_string();
        max_steering_        = static_cast<float>(get_parameter("max_steering").as_double());
        max_velocity_        = static_cast<float>(get_parameter("max_velocity").as_double());
        control_dt_          = static_cast<float>(get_parameter("control_dt").as_double());

        // Bounds are streamed inside each MpcState horizon frame.

        // --- Initialize FPGA (required) ---
        const uint32_t addr = static_cast<uint32_t>(
            get_parameter("fpga_base_address").as_int());

        if (!fpga_.initialize(addr)) {
            throw std::runtime_error("MPC FPGA init failed");
        }

        const uint64_t ref_vx_phys = static_cast<uint64_t>(get_parameter("ref_vx_phys_addr").as_int());
        const uint64_t ref_kappa_phys = static_cast<uint64_t>(get_parameter("ref_kappa_phys_addr").as_int());
        const uint64_t ref_left_phys = static_cast<uint64_t>(get_parameter("ref_left_phys_addr").as_int());
        const uint64_t ref_right_phys = static_cast<uint64_t>(get_parameter("ref_right_phys_addr").as_int());
        const int ref_capacity = static_cast<int>(get_parameter("ref_buffer_capacity").as_int());

        if (!fpga_.configure_reference_buffers(ref_vx_phys, ref_kappa_phys,
                                               ref_left_phys, ref_right_phys,
                                               static_cast<size_t>(std::max(0, ref_capacity)))) {
            throw std::runtime_error("MPC FPGA reference buffer mapping failed");
        }

        RCLCPP_INFO(get_logger(), "MPC FPGA init OK at 0x%08X (bulk memory horizon mode)", addr);

        // --- Create publisher & subscriber ---
        // Use SystemDefaultsQoS (Reliable) to match ackermann_mux subscriber
        drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic, rclcpp::SystemDefaultsQoS());

        auto qos = rclcpp::QoS(1).best_effort().durability_volatile();
        sub_ = create_subscription<f1tenth_msgs::msg::MpcState>(
            input_topic, qos,
            std::bind(&MpcReceiverFpgaNode::state_callback, this,
                      std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
            "MPC Receiver [FPGA] ready.  %s → %s",
            input_topic.c_str(), drive_topic.c_str());

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
    // --- Configurable limits -------------------------------------------------
    float max_steering_   = 0.4189f;
    float max_velocity_   = 6.0f;
    float control_dt_     = 0.02f;   // Default control interval for speed integration [s]

    // --- FPGA + ROS interfaces ----------------------------------------------
    MpcFpgaInterface    fpga_;

    rclcpp::Subscription<f1tenth_msgs::msg::MpcState>::SharedPtr sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    // --- Runtime statistics/state -------------------------------------------
    uint64_t msg_count_       = 0;
    double   total_latency_ms_ = 0.0;
    double   total_loop_us_ = 0.0;
    double   min_loop_us_ = std::numeric_limits<double>::infinity();
    double   max_loop_us_ = 0.0;
    std::chrono::steady_clock::time_point last_msg_time_ = std::chrono::steady_clock::now();
    rclcpp::Time last_callback_time_;   // For computing actual elapsed dt
    bool has_prev_callback_ = false;    // True after first callback

    struct FrenetErrorsFp {
        int32_t e_y_fp;
        int32_t e_psi_fp;
    };

    // Compute elapsed dt from message timestamps; falls back to configured dt.
    float compute_actual_dt(const rclcpp::Time& msg_time) {
        float actual_dt = control_dt_;
        if (has_prev_callback_) {
            double dt_sec = (msg_time - last_callback_time_).seconds();
            // Use measured dt only if it stays within [0.1 ms, 200 ms].
            if (dt_sec > 0.0001 && dt_sec < 0.2) {
                actual_dt = static_cast<float>(dt_sec);
            }
        }
        last_callback_time_ = msg_time;
        has_prev_callback_ = true;
        return actual_dt;
    }

    bool has_required_horizon_data(const f1tenth_msgs::msg::MpcState::SharedPtr& msg) const {
        return !(msg->ref_x_fp.empty() || msg->ref_y_fp.empty() || msg->ref_psi_fp.empty());
    }

    // Compute first-point Frenet tracking errors for FPGA state input.
    static FrenetErrorsFp compute_frenet_errors(const f1tenth_msgs::msg::MpcState::SharedPtr& msg) {
        const float x = fp_to_float(msg->x_fp);
        const float y = fp_to_float(msg->y_fp);
        const float theta = fp_to_float(msg->theta_fp);
        const float wx = fp_to_float(msg->ref_x_fp[0]);
        const float wy = fp_to_float(msg->ref_y_fp[0]);
        const float wpsi = fp_to_float(msg->ref_psi_fp[0]);

        const float dx = x - wx;
        const float dy = y - wy;
        const float e_y = -std::sin(wpsi) * dx + std::cos(wpsi) * dy;

        float e_psi = theta - wpsi;
        while (e_psi > static_cast<float>(M_PI)) e_psi -= 2.0f * static_cast<float>(M_PI);
        while (e_psi < -static_cast<float>(M_PI)) e_psi += 2.0f * static_cast<float>(M_PI);

        return FrenetErrorsFp{float_to_fp(e_y), float_to_fp(e_psi)};
    }

    float compute_target_speed(const f1tenth_msgs::msg::MpcState::SharedPtr& msg,
                               float accel,
                               float actual_dt) const {
        const float vx = fp_to_float(msg->velocity_fp);
        const float v_target = vx + accel * actual_dt;
        return std::max(0.0f, std::min(v_target, max_velocity_));
    }

    void publish_drive_command(float steering, float speed, float accel) {
        auto drive = ackermann_msgs::msg::AckermannDriveStamped();
        drive.header.stamp = now();
        drive.header.frame_id = "base_link";
        drive.drive.steering_angle = steering;
        drive.drive.speed = speed;
        drive.drive.acceleration = accel;
        drive_pub_->publish(drive);
    }

    void update_timing_and_log(const f1tenth_msgs::msg::MpcState::SharedPtr& msg,
                               const std::chrono::high_resolution_clock::time_point& t_start,
                               const std::chrono::high_resolution_clock::time_point& t_end,
                               float steering,
                               float speed,
                               float accel,
                               uint32_t status,
                               uint32_t iters) {
        const auto compute_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                t_end - t_start).count();
        msg_count_++;
        total_loop_us_ += static_cast<double>(compute_us);
        min_loop_us_ = std::min(min_loop_us_, static_cast<double>(compute_us));
        max_loop_us_ = std::max(max_loop_us_, static_cast<double>(compute_us));

        const auto now_ms = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
        const double latency_ms = static_cast<double>(now_ms - msg->timestamp_ms);
        total_latency_ms_ += latency_ms;

        if (msg_count_ % 100 == 0) {
            const double avg = total_latency_ms_ / static_cast<double>(msg_count_);
            const double avg_loop_us = total_loop_us_ / static_cast<double>(msg_count_);
            const int64_t fpga_ns = fpga_.get_last_compute_ns();
            RCLCPP_INFO(get_logger(),
                "[%s] WP=%u  delta=%.1f deg  v=%.1f  a=%.1f | "
                "Status=%u  Iter=%u | Total=%ld us  FPGA=%ld ns | "
                "Loop us avg/min/max=%.1f/%.1f/%.1f | Lat %.1f ms (avg %.1f)",
                "FPGA",
                msg->waypoint_index,
                steering * 57.2958f, speed, accel,
                status, iters,
                compute_us, fpga_ns,
                avg_loop_us, min_loop_us_, max_loop_us_,
                latency_ms, avg);
        }
    }

    // --- State callback -----------------------------------------------------
    void state_callback(const f1tenth_msgs::msg::MpcState::SharedPtr msg) {
        auto t_start = std::chrono::high_resolution_clock::now();
        last_msg_time_ = std::chrono::steady_clock::now();

        // 1) Update time base
        const rclcpp::Time msg_time(msg->header.stamp);
        const float actual_dt = compute_actual_dt(msg_time);

        float    steering = 0.0f;
        float    speed    = 0.0f;
        float    accel    = 0.0f;
        uint32_t status   = 0;
        uint32_t iters    = 0;

        int32_t out_steer_fp = 0;
        int32_t out_accel_fp = 0;

        // 2) Validate inputs
        if (!has_required_horizon_data(msg)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "No streamed waypoint data in message");
            return;
        }

        const bool horizon_loaded = fpga_.load_horizon(*msg);
        if (!horizon_loaded) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "FPGA horizon load failed");
            return;
        }

        // 3) Build tracking errors
        const FrenetErrorsFp errors = compute_frenet_errors(msg);

        const bool ok = fpga_.compute(
            errors.e_y_fp, errors.e_psi_fp,
            msg->velocity_fp, msg->vy_fp, msg->omega_fp,
            msg->steering_angle_fp,
            out_steer_fp, out_accel_fp, status, iters);

        if (!ok) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "FPGA compute failed");
            return;
        }

        steering = fp_to_float(out_steer_fp);
        accel    = fp_to_float(out_accel_fp);

        // 4) Post-process command
        speed = compute_target_speed(msg, accel, actual_dt);

        // Clamp outputs
        steering = std::clamp(steering, -max_steering_, max_steering_);
        speed    = std::clamp(speed,    0.0f,           max_velocity_);

        // 5) Publish command
        publish_drive_command(steering, speed, accel);

        // 6) Update timing stats and logs
        auto t_end = std::chrono::high_resolution_clock::now();
        update_timing_and_log(msg, t_start, t_end, steering, speed, accel, status, iters);
    }

};

}  // namespace f1tenth_communication

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<f1tenth_communication::MpcReceiverFpgaNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
