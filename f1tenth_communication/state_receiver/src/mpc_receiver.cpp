/**
 * @file mpc_receiver.cpp
 * @brief MPC Receiver with Riccati-ADMM FPGA Integration (AXI-Stream DMA)
 *
 * Runs on Ultra96-V2. Interfaces with the MPC FPGA IP core via:
 *   - AXI DMA for streaming state + horizon data (ultra-low latency)
 *   - AXI-Lite for control and reading output registers
 *
 * Startup:
 *   1. Initialize FPGA AXI-Lite register interface
 *   2. Initialize AXI DMA controller
 *   3. Map contiguous DMA buffer (reuses reserved memory at 0x70000000)
 *
 * Runtime (per MpcState message):
 *   1. Pack state + horizon into contiguous DMA buffer (336 bytes)
 *   2. Trigger MM2S DMA transfer to FPGA AXI-Stream input
 *   3. Wait for DMA + FPGA completion
 *   4. Read steering + accel from output registers
 *   5. Publish AckermannDriveStamped to /drive
 *
 * AXI-Stream Format (128-bit words):
 *   Beat 0: [e_y | e_psi | vx | vy]
 *   Beat 1: [omega | steering | horizon_length | reserved]
 *   Beat 2..20: [ref_vx[i] | ref_kappa[i] | ref_left[i] | ref_right[i]]
 *   Total: 21 beats × 16 bytes = 336 bytes
 *
 * Transfer time: ~210 ns (vs ~1850 ns DDR path)
 *
 * Data format: Q16.16 fixed-point (int32_t)
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
#include <fstream>

// Linux memory-mapped I/O
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

// MPC FPGA AXI-Lite register map and constants
#define MPC_FPGA_BASE_ADDR      0xA0000000

// AXI DMA IP base address (configure in Vivado, typical default)
#define AXI_DMA_BASE_ADDR       0xA0010000

// MPC IP control registers (AXI-Lite)
#define REG_AP_CTRL             0x000
#define REG_GIE                 0x004
#define REG_IER                 0x008
#define REG_ISR                 0x00C

// Output registers (AXI-Lite, from HLS)
#define REG_OUT_STEERING        0x010
#define REG_OUT_STEERING_VLD    0x014
#define REG_OUT_ACCEL           0x020
#define REG_OUT_ACCEL_VLD       0x024
#define REG_OUT_STATUS          0x030
#define REG_OUT_STATUS_VLD      0x034
#define REG_OUT_ITERATIONS      0x040
#define REG_OUT_ITERATIONS_VLD  0x044

// AXI DMA MM2S (Memory-Mapped to Stream) control registers
#define DMA_MM2S_CTRL           0x00    // MM2S Control register
#define DMA_MM2S_STATUS         0x04    // MM2S Status register
#define DMA_MM2S_SRC_LO         0x18    // MM2S Source Address (low 32 bits)
#define DMA_MM2S_SRC_HI         0x1C    // MM2S Source Address (high 32 bits)
#define DMA_MM2S_LENGTH         0x28    // MM2S Transfer Length

// DMA control bits
#define DMA_CTRL_RUN            0x0001  // Start DMA
#define DMA_CTRL_RESET          0x0004  // Reset DMA channel
#define DMA_STATUS_HALTED       0x0001  // DMA halted
#define DMA_STATUS_IDLE         0x0002  // DMA idle (transfer complete)
#define DMA_STATUS_ERR_INT      0x4000  // Error interrupt
#define DMA_STATUS_ERR_SLV      0x0020  // Slave error
#define DMA_STATUS_ERR_DEC      0x0040  // Decode error

// MPC horizon parameters
#define MPC_HORIZON             19
#define DMA_BUFFER_BEATS        (2 + MPC_HORIZON)   // 21 beats
#define DMA_BUFFER_BYTES        (DMA_BUFFER_BEATS * 16)  // 336 bytes

// DMA buffer physical address (reuse reserved memory region)
#define DMA_BUFFER_PHYS         0x70000000

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
 * Communicates with the Riccati-ADMM MPC IP via:
 *   - AXI DMA for streaming state + horizon data to FPGA
 *   - AXI-Lite registers for reading outputs
 *===========================================================================*/

class MpcFpgaInterface {
public:
    MpcFpgaInterface() = default;
    ~MpcFpgaInterface() { close_device(); }

    MpcFpgaInterface(const MpcFpgaInterface&) = delete;
    MpcFpgaInterface& operator=(const MpcFpgaInterface&) = delete;

    /**
     * Initialize FPGA and DMA interfaces.
     * Maps: MPC IP registers, DMA controller registers, and DMA buffer.
     */
    bool initialize(uint32_t mpc_base_addr = MPC_FPGA_BASE_ADDR,
                    uint32_t dma_base_addr = AXI_DMA_BASE_ADDR,
                    uint64_t dma_buf_phys = DMA_BUFFER_PHYS) {
        if (!is_fpga_operating()) {
            fprintf(stderr, "MPC-FPGA: fpga_manager state is not operating; refusing /dev/mem init\n");
            return false;
        }

        mpc_base_addr_ = mpc_base_addr;
        dma_base_addr_ = dma_base_addr;
        dma_buf_phys_  = dma_buf_phys;

        mem_fd_ = open("/dev/mem", O_RDWR | O_SYNC);
        if (mem_fd_ < 0) {
            fprintf(stderr, "MPC-FPGA: open /dev/mem failed: %s\n", strerror(errno));
            return false;
        }

        // Map MPC IP AXI-Lite registers
        mpc_regs_ = mmap(nullptr, 0x1000, PROT_READ | PROT_WRITE,
                         MAP_SHARED, mem_fd_, mpc_base_addr_);
        if (mpc_regs_ == MAP_FAILED) {
            fprintf(stderr, "MPC-FPGA: mmap MPC registers at 0x%08X failed: %s\n",
                    mpc_base_addr_, strerror(errno));
            close_device();
            return false;
        }

        // Map AXI DMA controller registers
        dma_regs_ = mmap(nullptr, 0x1000, PROT_READ | PROT_WRITE,
                         MAP_SHARED, mem_fd_, dma_base_addr_);
        if (dma_regs_ == MAP_FAILED) {
            fprintf(stderr, "MPC-FPGA: mmap DMA registers at 0x%08X failed: %s\n",
                    dma_base_addr_, strerror(errno));
            close_device();
            return false;
        }

        // Map DMA buffer (contiguous physical memory for streaming data)
        dma_buf_ = mmap(nullptr, DMA_BUFFER_BYTES, PROT_READ | PROT_WRITE,
                        MAP_SHARED, mem_fd_, static_cast<off_t>(dma_buf_phys_));
        if (dma_buf_ == MAP_FAILED) {
            fprintf(stderr, "MPC-FPGA: mmap DMA buffer at 0x%lX failed: %s\n",
                    (unsigned long)dma_buf_phys_, strerror(errno));
            close_device();
            return false;
        }

        // Reset DMA and verify it's ready
        if (!reset_dma()) {
            fprintf(stderr, "MPC-FPGA: DMA reset/init failed\n");
            close_device();
            return false;
        }

        uint32_t ctrl = mpc_read(REG_AP_CTRL);
        uint32_t dma_status = dma_read(DMA_MM2S_STATUS);
        fprintf(stderr, "MPC-FPGA: Init OK - MPC@0x%08X (AP_CTRL=0x%08X), "
                        "DMA@0x%08X (STATUS=0x%08X), Buffer@0x%lX\n",
                mpc_base_addr_, ctrl, dma_base_addr_, dma_status,
                (unsigned long)dma_buf_phys_);

        initialized_ = true;
        return true;
    }

    void close_device() {
        if (dma_buf_ && dma_buf_ != MAP_FAILED) {
            munmap(dma_buf_, DMA_BUFFER_BYTES);
            dma_buf_ = nullptr;
        }
        if (dma_regs_ && dma_regs_ != MAP_FAILED) {
            munmap(dma_regs_, 0x1000);
            dma_regs_ = nullptr;
        }
        if (mpc_regs_ && mpc_regs_ != MAP_FAILED) {
            munmap(mpc_regs_, 0x1000);
            mpc_regs_ = nullptr;
        }
        if (mem_fd_ >= 0) {
            ::close(mem_fd_);
            mem_fd_ = -1;
        }
        initialized_ = false;
    }

    bool is_ready() const { return initialized_; }

    /**
     * Run one MPC compute cycle via AXI-Stream DMA.
     * 
     * Packs state + horizon into DMA buffer, transfers via DMA,
     * waits for FPGA completion, reads outputs.
     */
    bool compute(int32_t e_y_fp, int32_t e_psi_fp,
                 int32_t vx_fp, int32_t vy_fp, int32_t omega_fp,
                 int32_t steering_fp,
                 const f1tenth_msgs::msg::MpcState& msg,
                 int32_t& out_steering_fp, int32_t& out_accel_fp,
                 uint32_t& out_status, uint32_t& out_iterations) {
        if (!is_fpga_operating()) return false;
        if (!initialized_) return false;

        const size_t horizon = std::min(static_cast<size_t>(msg.horizon_length),
                                        static_cast<size_t>(MPC_HORIZON));
        if (horizon == 0) return false;

        // --- Pack data into DMA buffer (128-bit aligned words) ---
        volatile uint32_t* buf = static_cast<volatile uint32_t*>(dma_buf_);
        
        // Beat 0: [e_y | e_psi | vx | vy]
        buf[0] = static_cast<uint32_t>(e_y_fp);
        buf[1] = static_cast<uint32_t>(e_psi_fp);
        buf[2] = static_cast<uint32_t>(vx_fp);
        buf[3] = static_cast<uint32_t>(vy_fp);
        
        // Beat 1: [omega | steering | horizon_length | reserved]
        buf[4] = static_cast<uint32_t>(omega_fp);
        buf[5] = static_cast<uint32_t>(steering_fp);
        buf[6] = static_cast<uint32_t>(horizon);
        buf[7] = 0;  // Reserved
        
        // Beats 2..N+1: [ref_vx[i] | ref_kappa[i] | ref_left[i] | ref_right[i]]
        for (size_t i = 0; i < MPC_HORIZON; i++) {
            size_t base = 8 + (i * 4);  // Each beat is 4 × uint32_t
            if (i < horizon) {
                buf[base + 0] = static_cast<uint32_t>(msg.ref_vx_fp[i]);
                buf[base + 1] = static_cast<uint32_t>(msg.ref_kappa_fp[i]);
                buf[base + 2] = static_cast<uint32_t>(msg.ref_left_bound_fp[i]);
                buf[base + 3] = static_cast<uint32_t>(msg.ref_right_bound_fp[i]);
            } else {
                // Pad with zeros for remaining slots
                buf[base + 0] = 0;
                buf[base + 1] = 0;
                buf[base + 2] = 0;
                buf[base + 3] = 0;
            }
        }
        
        // Memory barrier to ensure all buffer writes are visible
        __sync_synchronize();

        // --- Measure compute time ---
        struct timespec t0, t1;
        clock_gettime(CLOCK_MONOTONIC_RAW, &t0);

        // --- Start DMA transfer ---
        // Configure source address (physical address of our buffer)
        dma_write(DMA_MM2S_SRC_LO, static_cast<uint32_t>(dma_buf_phys_ & 0xFFFFFFFFULL));
        dma_write(DMA_MM2S_SRC_HI, static_cast<uint32_t>(dma_buf_phys_ >> 32));
        
        // Start DMA engine (if not already running)
        dma_write(DMA_MM2S_CTRL, DMA_CTRL_RUN);
        
        // Barrier before triggering transfer
        __sync_synchronize();
        
        // Write length to start transfer (this triggers the actual DMA)
        dma_write(DMA_MM2S_LENGTH, DMA_BUFFER_BYTES);

        // --- Wait for DMA completion ---
        if (!wait_dma_complete(100000)) {
            fprintf(stderr, "MPC-FPGA: DMA transfer timeout\n");
            last_compute_ns_ = -1;
            return false;
        }

        // --- Wait for MPC compute completion ---
        // With AXI-Stream, FPGA starts automatically when data arrives
        // AP_DONE indicates MPC computation finished
        if (!wait_mpc_done(200000)) {
            fprintf(stderr, "MPC-FPGA: MPC compute timeout (DMA OK)\n");
            last_compute_ns_ = -1;
            return false;
        }

        clock_gettime(CLOCK_MONOTONIC_RAW, &t1);
        last_compute_ns_ = (t1.tv_sec - t0.tv_sec) * 1000000000LL
                          + (t1.tv_nsec - t0.tv_nsec);

        // --- Read output registers ---
        out_steering_fp = static_cast<int32_t>(mpc_read(REG_OUT_STEERING));
        out_accel_fp    = static_cast<int32_t>(mpc_read(REG_OUT_ACCEL));
        out_status      = mpc_read(REG_OUT_STATUS);
        out_iterations  = mpc_read(REG_OUT_ITERATIONS);

        return true;
    }

    int64_t get_last_compute_ns() const { return last_compute_ns_; }

private:
    int      mem_fd_        = -1;
    void*    mpc_regs_      = nullptr;  // MPC IP AXI-Lite registers
    void*    dma_regs_      = nullptr;  // AXI DMA controller registers
    void*    dma_buf_       = nullptr;  // DMA buffer (userspace mapping)
    uint32_t mpc_base_addr_ = 0;
    uint32_t dma_base_addr_ = 0;
    uint64_t dma_buf_phys_  = 0;        // DMA buffer physical address
    bool     initialized_   = false;
    int64_t  last_compute_ns_ = 0;

    // MPC IP register access
    void mpc_write(uint32_t offset, uint32_t value) {
        volatile uint32_t* reg = reinterpret_cast<volatile uint32_t*>(
            static_cast<volatile uint8_t*>(mpc_regs_) + offset);
        *reg = value;
    }

    uint32_t mpc_read(uint32_t offset) {
        volatile uint32_t* reg = reinterpret_cast<volatile uint32_t*>(
            static_cast<volatile uint8_t*>(mpc_regs_) + offset);
        return *reg;
    }

    // DMA register access
    void dma_write(uint32_t offset, uint32_t value) {
        volatile uint32_t* reg = reinterpret_cast<volatile uint32_t*>(
            static_cast<volatile uint8_t*>(dma_regs_) + offset);
        *reg = value;
    }

    uint32_t dma_read(uint32_t offset) {
        volatile uint32_t* reg = reinterpret_cast<volatile uint32_t*>(
            static_cast<volatile uint8_t*>(dma_regs_) + offset);
        return *reg;
    }

    bool reset_dma() {
        // Reset MM2S channel
        dma_write(DMA_MM2S_CTRL, DMA_CTRL_RESET);
        
        // Wait for reset to complete (halted bit should clear)
        int timeout = 10000;
        while (timeout-- > 0) {
            uint32_t ctrl = dma_read(DMA_MM2S_CTRL);
            if (!(ctrl & DMA_CTRL_RESET)) break;
        }
        if (timeout <= 0) return false;

        // Check for halted state (normal after reset)
        uint32_t status = dma_read(DMA_MM2S_STATUS);
        if (status & (DMA_STATUS_ERR_INT | DMA_STATUS_ERR_SLV | DMA_STATUS_ERR_DEC)) {
            fprintf(stderr, "MPC-FPGA: DMA has error flags after reset: 0x%08X\n", status);
            return false;
        }

        return true;
    }

    bool wait_dma_complete(int timeout_cycles) {
        while (timeout_cycles-- > 0) {
            uint32_t status = dma_read(DMA_MM2S_STATUS);
            
            // Check for errors
            if (status & (DMA_STATUS_ERR_INT | DMA_STATUS_ERR_SLV | DMA_STATUS_ERR_DEC)) {
                fprintf(stderr, "MPC-FPGA: DMA error status=0x%08X\n", status);
                reset_dma();  // Try to recover
                return false;
            }
            
            // Check for idle (transfer complete)
            if (status & DMA_STATUS_IDLE) return true;
        }
        return false;
    }

    bool wait_mpc_done(int timeout_cycles) {
        while (timeout_cycles-- > 0) {
            if (mpc_read(REG_AP_CTRL) & AP_DONE) return true;
        }
        return false;
    }

    static bool is_fpga_operating() {
        std::ifstream f("/sys/class/fpga_manager/fpga0/state");
        if (!f.is_open()) return false;
        std::string state;
        std::getline(f, state);
        return state == "operating";
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
        declare_parameter("mpc_base_address",
                           static_cast<int64_t>(MPC_FPGA_BASE_ADDR));
        declare_parameter("dma_base_address",
                           static_cast<int64_t>(AXI_DMA_BASE_ADDR));
        declare_parameter("dma_buffer_phys_addr",
                           static_cast<int64_t>(DMA_BUFFER_PHYS));

        // Vehicle / controller
        declare_parameter("max_steering", 0.4189);
        declare_parameter("max_velocity", 20.0);

        // Control interval for speed = vx + accel * dt
        declare_parameter("control_dt", 0.04);  // [s] (default 50 Hz state rate)

        // --- Read parameters ---
        auto input_topic     = get_parameter("input_topic").as_string();
        auto drive_topic     = get_parameter("drive_topic").as_string();
        max_steering_        = static_cast<float>(get_parameter("max_steering").as_double());
        max_velocity_        = static_cast<float>(get_parameter("max_velocity").as_double());
        control_dt_          = static_cast<float>(get_parameter("control_dt").as_double());

        // --- Initialize FPGA + DMA (required) ---
        const uint32_t mpc_addr = static_cast<uint32_t>(
            get_parameter("mpc_base_address").as_int());
        const uint32_t dma_addr = static_cast<uint32_t>(
            get_parameter("dma_base_address").as_int());
        const uint64_t dma_buf_phys = static_cast<uint64_t>(
            get_parameter("dma_buffer_phys_addr").as_int());

        if (!fpga_.initialize(mpc_addr, dma_addr, dma_buf_phys)) {
            throw std::runtime_error("MPC FPGA + DMA init failed");
        }

        RCLCPP_INFO(get_logger(), 
            "MPC FPGA init OK - MPC@0x%08X, DMA@0x%08X, Buffer@0x%lX (AXI-Stream mode)",
            mpc_addr, dma_addr, (unsigned long)dma_buf_phys);

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
            "MPC Receiver [FPGA AXI-Stream] ready.  %s → %s",
            input_topic.c_str(), drive_topic.c_str());
    }

private:
    // --- Configurable limits -------------------------------------------------
    float max_steering_   = 0.4189f;
    float max_velocity_   = 20.0f;
    float control_dt_     = 0.04f;   // Default control interval for speed integration [s]

    // --- FPGA + ROS interfaces ----------------------------------------------
    MpcFpgaInterface    fpga_;

    rclcpp::Subscription<f1tenth_msgs::msg::MpcState>::SharedPtr sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    // --- Runtime statistics/state -------------------------------------------
    uint64_t msg_count_       = 0;
    uint64_t latency_count_   = 0;
    double   total_latency_ms_ = 0.0;
    double   total_loop_us_ = 0.0;
    double   min_loop_us_ = std::numeric_limits<double>::infinity();
    double   max_loop_us_ = 0.0;
    std::chrono::steady_clock::time_point last_msg_time_ = std::chrono::steady_clock::now();
    rclcpp::Time last_callback_time_;   // For computing actual elapsed dt
    bool has_prev_callback_ = false;    // True after first callback
    float latest_vx_mps_ = 0.0f;

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
        return msg->horizon_length > 0 && !msg->ref_vx_fp.empty();
    }

    // Compute first-point Frenet tracking errors for FPGA state input.
    static FrenetErrorsFp compute_frenet_errors(const f1tenth_msgs::msg::MpcState::SharedPtr& msg) {
        const float x = fp_to_float(msg->x_fp);
        const float y = fp_to_float(msg->y_fp);
        const float theta = fp_to_float(msg->theta_fp);
        const float wx = fp_to_float(msg->ref_x_0_fp);
        const float wy = fp_to_float(msg->ref_y_0_fp);
        const float wpsi = fp_to_float(msg->ref_psi_0_fp);

        const float dx = x - wx;
        const float dy = y - wy;
        const float e_y = -std::sin(wpsi) * dx + std::cos(wpsi) * dy;

        float e_psi = theta - wpsi;
        while (e_psi > static_cast<float>(M_PI)) e_psi -= 2.0f * static_cast<float>(M_PI);
        while (e_psi < -static_cast<float>(M_PI)) e_psi += 2.0f * static_cast<float>(M_PI);

        return FrenetErrorsFp{float_to_fp(e_y), float_to_fp(e_psi)};
    }

    float compute_target_speed(float accel,
                               float actual_dt) const {
        (void)actual_dt;
        // Match MPC/src/mpc_hardware_node.c behavior: integrate accel over
        // one MPC prediction step rather than callback-period dt.
        constexpr float kMpcPredictionStepSeconds = 0.04f;
        const float v_target = latest_vx_mps_ + accel * kMpcPredictionStepSeconds;
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

        // Compute latency from ROS header timestamp
        double latency_ms = -1.0;
        const rclcpp::Time msg_time(msg->header.stamp);
        if (msg_time.nanoseconds() > 0) {
            latency_ms = (this->now() - msg_time).seconds() * 1000.0;
        }
        if (latency_ms >= 0.0) {
            total_latency_ms_ += latency_ms;
            latency_count_++;
        }

        if (msg_count_ % 100 == 0) {
            const double avg = (latency_count_ > 0)
                ? (total_latency_ms_ / static_cast<double>(latency_count_))
                : -1.0;
            const double avg_loop_us = total_loop_us_ / static_cast<double>(msg_count_);
            const int64_t fpga_ns = fpga_.get_last_compute_ns();
            if (latency_ms >= 0.0 && avg >= 0.0) {
                RCLCPP_INFO(get_logger(),
                    "[FPGA] delta=%.1f deg  v=%.1f  a=%.1f | "
                    "Status=%u  Iter=%u | Total=%ld us  FPGA=%ld ns | "
                    "Loop us avg/min/max=%.1f/%.1f/%.1f | Lat %.1f ms (avg %.1f)",
                    steering * 57.2958f, speed, accel,
                    status, iters,
                    compute_us, fpga_ns,
                    avg_loop_us, min_loop_us_, max_loop_us_,
                    latency_ms, avg);
            } else {
                RCLCPP_INFO(get_logger(),
                    "[FPGA] delta=%.1f deg  v=%.1f  a=%.1f | "
                    "Status=%u  Iter=%u | Total=%ld us  FPGA=%ld ns | "
                    "Loop us avg/min/max=%.1f/%.1f/%.1f | Lat N/A",
                    steering * 57.2958f, speed, accel,
                    status, iters,
                    compute_us, fpga_ns,
                    avg_loop_us, min_loop_us_, max_loop_us_);
            }
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

        // Track latest odometry-equivalent longitudinal speed (matches MPC node
        // use of latest vx sample in speed integration).
        latest_vx_mps_ = fp_to_float(msg->velocity_fp);

        // 3) Build tracking errors
        const FrenetErrorsFp errors = compute_frenet_errors(msg);

        // 4) Run MPC via DMA - packs state+horizon, transfers via AXI-Stream, computes
        const bool ok = fpga_.compute(
            errors.e_y_fp, errors.e_psi_fp,
            msg->velocity_fp, msg->vy_fp, msg->omega_fp,
            msg->steering_angle_fp,
            *msg,  // Pass full message for horizon data
            out_steer_fp, out_accel_fp, status, iters);

        if (!ok) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "FPGA DMA/compute failed");
            return;
        }

        steering = fp_to_float(out_steer_fp);
        accel    = fp_to_float(out_accel_fp);

        // 5) Post-process command
        speed = compute_target_speed(accel, actual_dt);

        // Clamp outputs
        steering = std::clamp(steering, -max_steering_, max_steering_);
        speed    = std::clamp(speed,    0.0f,           max_velocity_);

        // 6) Publish command
        publish_drive_command(steering, speed, accel);

        // 7) Update timing stats and logs
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
