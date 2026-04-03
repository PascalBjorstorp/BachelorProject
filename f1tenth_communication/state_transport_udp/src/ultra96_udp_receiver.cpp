/**
 * @file ultra96_udp_receiver.cpp
 * @brief Ultra96 UDP state receiver and FPGA MPC executor.
 * @details Receives UDP state packets from Jetson, computes Frenet errors,
 *          streams state+horizon through AXI DMA, and sends control packets back.
 */

#include "state_transport_udp/state_packet.hpp"
#include "mpc_fpga_interface.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>

namespace state_transport_udp {

static constexpr int32_t FP_SCALE = 65536;

/**
 * @brief Convert float to Q16.16 fixed-point integer.
 * @param f Floating-point input value.
 * @return Q16.16 fixed-point representation of `f`.
 */
inline int32_t float_to_fp(float f) {
    return static_cast<int32_t>(f * static_cast<float>(FP_SCALE));
}

/**
 * @brief Convert Q16.16 fixed-point integer to float.
 * @param fp Fixed-point input value.
 * @return Floating-point representation of `fp`.
 */
inline float fp_to_float(int32_t fp) {
    return static_cast<float>(fp) / static_cast<float>(FP_SCALE);
}

/**
 * @brief AXI-Lite + AXI-DMA interface for one-shot MPC solves from UDP packets.
 */
class MpcFpgaInterface {
public:
    MpcFpgaInterface() = default;

    /**
     * @brief Destroy interface and release mapped FPGA resources.
     * @return None
     */
    ~MpcFpgaInterface() { close_device(); }

    MpcFpgaInterface(const MpcFpgaInterface&) = delete;
    MpcFpgaInterface& operator=(const MpcFpgaInterface&) = delete;

    /**
     * @brief Initialize all /dev/mem mappings required for DMA-backed MPC compute.
     * @param mpc_base_addr AXI-Lite base for MPC IP.
     * @param dma_base_addr AXI-Lite base for DMA IP.
     * @param dma_buf_phys Physical base address for reserved DMA source buffer.
     * @return true on successful initialization.
     */
    bool initialize(uint32_t mpc_base_addr, uint32_t dma_base_addr, uint64_t dma_buf_phys) {
        if (!is_fpga_operating()) {
            std::fprintf(stderr, "UDP-FPGA: fpga_manager state is not operating\n");
            return false;
        }

        mpc_base_addr_ = mpc_base_addr;
        dma_base_addr_ = dma_base_addr;
        dma_buf_phys_ = dma_buf_phys;

        mem_fd_ = ::open("/dev/mem", O_RDWR | O_SYNC);
        if (mem_fd_ < 0) {
            std::fprintf(stderr, "UDP-FPGA: open /dev/mem failed: %s\n", std::strerror(errno));
            return false;
        }

        mpc_regs_ = ::mmap(nullptr, 0x1000, PROT_READ | PROT_WRITE,
                           MAP_SHARED, mem_fd_, mpc_base_addr_);
        if (mpc_regs_ == MAP_FAILED) {
            std::fprintf(stderr, "UDP-FPGA: mmap MPC regs failed: %s\n", std::strerror(errno));
            close_device();
            return false;
        }

        dma_regs_ = ::mmap(nullptr, 0x1000, PROT_READ | PROT_WRITE,
                           MAP_SHARED, mem_fd_, dma_base_addr_);
        if (dma_regs_ == MAP_FAILED) {
            std::fprintf(stderr, "UDP-FPGA: mmap DMA regs failed: %s\n", std::strerror(errno));
            close_device();
            return false;
        }

        dma_buf_ = ::mmap(nullptr, DMA_BUFFER_BYTES, PROT_READ | PROT_WRITE,
                          MAP_SHARED, mem_fd_, static_cast<off_t>(dma_buf_phys_));
        if (dma_buf_ == MAP_FAILED) {
            std::fprintf(stderr, "UDP-FPGA: mmap DMA buffer failed: %s\n", std::strerror(errno));
            close_device();
            return false;
        }

        if (!reset_dma()) {
            std::fprintf(stderr, "UDP-FPGA: DMA reset failed\n");
            close_device();
            return false;
        }

        initialized_ = true;
        return true;
    }

    /**
     * @brief Unmap all mapped regions and close /dev/mem.
     */
    void close_device() {
        if (dma_buf_ && dma_buf_ != MAP_FAILED) {
            ::munmap(dma_buf_, DMA_BUFFER_BYTES);
            dma_buf_ = nullptr;
        }
        if (dma_regs_ && dma_regs_ != MAP_FAILED) {
            ::munmap(dma_regs_, 0x1000);
            dma_regs_ = nullptr;
        }
        if (mpc_regs_ && mpc_regs_ != MAP_FAILED) {
            ::munmap(mpc_regs_, 0x1000);
            mpc_regs_ = nullptr;
        }
        if (mem_fd_ >= 0) {
            ::close(mem_fd_);
            mem_fd_ = -1;
        }
        initialized_ = false;
    }

    /**
     * @brief Run one DMA-fed MPC compute call from current Frenet errors and horizon packet.
     * @return true when DMA transfer, solver run, and output readback succeed.
     */
    bool compute(int32_t e_y_fp,
                 int32_t e_psi_fp,
                 int32_t vx_fp,
                 int32_t vy_fp,
                 int32_t omega_fp,
                 int32_t steering_fp,
                 const StatePacket& packet,
                 int32_t& out_steering_fp,
                 int32_t& out_accel_fp,
                 uint32_t& out_status,
                 uint32_t& out_iterations) {
        if (!initialized_) {
            return false;
        }

        const size_t horizon = std::min(static_cast<size_t>(packet.horizon_length),
                                        static_cast<size_t>(MPC_HORIZON));
        if (horizon == 0) {
            return false;
        }

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
        buf[7] = 0;

        // Beats 2..: [ref_vx | ref_kappa | ref_left | ref_right]
        for (size_t i = 0; i < MPC_HORIZON; ++i) {
            const size_t base = 8 + (i * 4);
            if (i < horizon) {
                buf[base + 0] = static_cast<uint32_t>(packet.ref_vx_fp[i]);
                buf[base + 1] = static_cast<uint32_t>(packet.ref_kappa_fp[i]);
                buf[base + 2] = static_cast<uint32_t>(packet.ref_left_bound_fp[i]);
                buf[base + 3] = static_cast<uint32_t>(packet.ref_right_bound_fp[i]);
            } else {
                buf[base + 0] = 0;
                buf[base + 1] = 0;
                buf[base + 2] = 0;
                buf[base + 3] = 0;
            }
        }

        __sync_synchronize();

        // Clear stale completion/valid flags from the previous transaction.
        (void)mpc_read(REG_AP_CTRL);
        (void)mpc_read(REG_OUT_STEERING_VLD);
        (void)mpc_read(REG_OUT_ACCEL_VLD);
        (void)mpc_read(REG_OUT_STATUS_VLD);
        (void)mpc_read(REG_OUT_ITERATIONS_VLD);

        // Clear stale interrupt bits before start.
        const uint32_t isr = mpc_read(REG_ISR);
        if (isr) {
            mpc_write(REG_ISR, isr);
        }

        mpc_write(REG_AP_CTRL, AP_START);

        dma_write(DMA_MM2S_SRC_LO, static_cast<uint32_t>(dma_buf_phys_ & 0xFFFFFFFFULL));
        dma_write(DMA_MM2S_SRC_HI, static_cast<uint32_t>(dma_buf_phys_ >> 32));
        dma_write(DMA_MM2S_CTRL, DMA_CTRL_RUN);
        __sync_synchronize();
        dma_write(DMA_MM2S_LENGTH, DMA_BUFFER_BYTES);

        if (!wait_dma_complete(MPC_FPGA_DMA_TRANSFER_TIMEOUT_CYCLES)) {
            std::fprintf(stderr, "UDP-FPGA: DMA transfer timeout\n");
            return false;
        }

        if (!wait_mpc_done(MPC_FPGA_MPC_DONE_TIMEOUT_CYCLES)) {
            std::fprintf(stderr, "UDP-FPGA: MPC done timeout\n");
            reset_dma();
            return false;
        }

        if (!wait_output_valid(MPC_FPGA_OUTPUT_VALID_TIMEOUT_CYCLES)) {
            std::fprintf(stderr, "UDP-FPGA: output valid timeout\n");
            return false;
        }

        out_steering_fp = static_cast<int32_t>(mpc_read(REG_OUT_STEERING));
        out_accel_fp = static_cast<int32_t>(mpc_read(REG_OUT_ACCEL));
        out_status = mpc_read(REG_OUT_STATUS);
        out_iterations = mpc_read(REG_OUT_ITERATIONS);
        return true;
    }

private:
    int mem_fd_{-1};
    void* mpc_regs_{nullptr};
    void* dma_regs_{nullptr};
    void* dma_buf_{nullptr};
    uint32_t mpc_base_addr_{0};
    uint32_t dma_base_addr_{0};
    uint64_t dma_buf_phys_{0};
    bool initialized_{false};

    /** @brief Write MPC AXI-Lite register. */
    void mpc_write(uint32_t offset, uint32_t value) {
        volatile uint32_t* reg = reinterpret_cast<volatile uint32_t*>(
            static_cast<volatile uint8_t*>(mpc_regs_) + offset);
        *reg = value;
    }

    /** @brief Read MPC AXI-Lite register. */
    uint32_t mpc_read(uint32_t offset) const {
        volatile uint32_t* reg = reinterpret_cast<volatile uint32_t*>(
            static_cast<volatile uint8_t*>(mpc_regs_) + offset);
        return *reg;
    }

    /** @brief Write DMA AXI-Lite register. */
    void dma_write(uint32_t offset, uint32_t value) {
        volatile uint32_t* reg = reinterpret_cast<volatile uint32_t*>(
            static_cast<volatile uint8_t*>(dma_regs_) + offset);
        *reg = value;
    }

    /** @brief Read DMA AXI-Lite register. */
    uint32_t dma_read(uint32_t offset) const {
        volatile uint32_t* reg = reinterpret_cast<volatile uint32_t*>(
            static_cast<volatile uint8_t*>(dma_regs_) + offset);
        return *reg;
    }

    /** @brief Reset MM2S channel and validate status. */
    bool reset_dma() {
        dma_write(DMA_MM2S_CTRL, DMA_CTRL_RESET);

        int timeout = MPC_FPGA_DMA_RESET_TIMEOUT_CYCLES;
        while (timeout-- > 0) {
            const uint32_t ctrl = dma_read(DMA_MM2S_CTRL);
            if (!(ctrl & DMA_CTRL_RESET)) {
                break;
            }
        }
        if (timeout <= 0) {
            return false;
        }

        const uint32_t status = dma_read(DMA_MM2S_STATUS);
        if (status & (DMA_STATUS_ERR_INT | DMA_STATUS_ERR_SLV | DMA_STATUS_ERR_DEC)) {
            std::fprintf(stderr, "UDP-FPGA: DMA error flags after reset: 0x%08X\n", status);
            return false;
        }
        return true;
    }

    /** @brief Wait until DMA transfer completes or fails. */
    bool wait_dma_complete(int timeout_cycles) {
        while (timeout_cycles-- > 0) {
            const uint32_t status = dma_read(DMA_MM2S_STATUS);
            if (status & (DMA_STATUS_ERR_INT | DMA_STATUS_ERR_SLV | DMA_STATUS_ERR_DEC)) {
                std::fprintf(stderr, "UDP-FPGA: DMA error status=0x%08X\n", status);
                reset_dma();
                return false;
            }
            if (status & DMA_STATUS_IDLE) {
                return true;
            }
        }
        return false;
    }

    /** @brief Wait until AP_DONE is observed. */
    bool wait_mpc_done(int timeout_cycles) const {
        while (timeout_cycles-- > 0) {
            if (mpc_read(REG_AP_CTRL) & AP_DONE) {
                return true;
            }
        }
        return false;
    }

    /** @brief Wait until steering and accel outputs are valid. */
    bool wait_output_valid(int timeout_cycles) const {
        bool steer_seen = false;
        bool accel_seen = false;
        while (timeout_cycles-- > 0) {
            if (!steer_seen) {
                steer_seen = (mpc_read(REG_OUT_STEERING_VLD) & 0x1u) != 0u;
            }
            if (!accel_seen) {
                accel_seen = (mpc_read(REG_OUT_ACCEL_VLD) & 0x1u) != 0u;
            }
            if (steer_seen && accel_seen) {
                return true;
            }
        }
        return false;
    }

    /** @brief Check FPGA manager state before allowing /dev/mem access. */
    static bool is_fpga_operating() {
        std::ifstream f("/sys/class/fpga_manager/fpga0/state");
        if (!f.is_open()) {
            return false;
        }
        std::string state;
        std::getline(f, state);
        return state == "operating";
    }
};

}  // namespace state_transport_udp

namespace {

volatile sig_atomic_t g_running = 1;

/** @brief Handle termination signals and request clean shutdown. */
void signal_handler(int) {
    g_running = 0;
}

/** @brief Return monotonic raw timestamp in nanoseconds. */
uint64_t monotonic_now_ns() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ull + static_cast<uint64_t>(ts.tv_nsec);
}

/** @brief Read uint32 environment variable with fallback default. */
uint32_t env_u32(const char* name, uint32_t fallback) {
    const char* raw = std::getenv(name);
    if (!raw || !*raw) {
        return fallback;
    }
    return static_cast<uint32_t>(std::strtoul(raw, nullptr, 0));
}

/** @brief Read uint64 environment variable with fallback default. */
uint64_t env_u64(const char* name, uint64_t fallback) {
    const char* raw = std::getenv(name);
    if (!raw || !*raw) {
        return fallback;
    }
    return static_cast<uint64_t>(std::strtoull(raw, nullptr, 0));
}

/** @brief Read float environment variable with fallback default. */
float env_f32(const char* name, float fallback) {
    const char* raw = std::getenv(name);
    if (!raw || !*raw) {
        return fallback;
    }
    return std::strtof(raw, nullptr);
}

/**
 * @brief Compute Frenet tracking errors from packet state and first reference point.
 */
void compute_frenet_errors(const state_transport_udp::StatePacket& packet,
                          int32_t& out_e_y_fp,
                          int32_t& out_e_psi_fp) {
    const float x = state_transport_udp::fp_to_float(packet.x_fp);
    const float y = state_transport_udp::fp_to_float(packet.y_fp);
    const float theta = state_transport_udp::fp_to_float(packet.theta_fp);
    const float wx = state_transport_udp::fp_to_float(packet.ref_x_0_fp);
    const float wy = state_transport_udp::fp_to_float(packet.ref_y_0_fp);
    const float wpsi = state_transport_udp::fp_to_float(packet.ref_psi_0_fp);

    const float dx = x - wx;
    const float dy = y - wy;
    const float e_y = -std::sin(wpsi) * dx + std::cos(wpsi) * dy;

    float e_psi = theta - wpsi;
    while (e_psi > static_cast<float>(M_PI)) {
        e_psi -= 2.0f * static_cast<float>(M_PI);
    }
    while (e_psi < -static_cast<float>(M_PI)) {
        e_psi += 2.0f * static_cast<float>(M_PI);
    }

    out_e_y_fp = state_transport_udp::float_to_fp(e_y);
    out_e_psi_fp = state_transport_udp::float_to_fp(e_psi);
}

}  // namespace

/**
 * @brief Entry point for Ultra96 UDP receiver process.
 */
int main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    const uint16_t state_port = static_cast<uint16_t>(env_u32("UDP_STATE_PORT", 49000));
    const uint16_t control_port = static_cast<uint16_t>(env_u32("UDP_CONTROL_PORT", 49001));
    const float control_dt = env_f32("UDP_CONTROL_DT_S", static_cast<float>(MPC_FPGA_PREDICTION_DT_S));
    const float max_velocity = env_f32("UDP_MAX_VEL_MPS", static_cast<float>(MPC_FPGA_MAX_VEL_MPS));

    const uint32_t mpc_base_addr = env_u32("MPC_BASE_ADDR", static_cast<uint32_t>(MPC_FPGA_BASE_ADDR));
    const uint32_t dma_base_addr = env_u32("DMA_BASE_ADDR", static_cast<uint32_t>(AXI_DMA_BASE_ADDR));
    const uint64_t dma_buffer_phys = env_u64("DMA_BUFFER_PHYS_ADDR", static_cast<uint64_t>(DMA_BUFFER_PHYS_ADDR));

    // Create and bind UDP socket for receiving state packets and sending control packets.
    int rx_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (rx_fd < 0) {
        std::fprintf(stderr, "UDP receiver socket creation failed: %s\n", std::strerror(errno));
        return 1;
    }

    int tx_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (tx_fd < 0) {
        std::fprintf(stderr, "UDP control socket creation failed: %s\n", std::strerror(errno));
        ::close(rx_fd);
        return 1;
    }

    sockaddr_in bind_addr{};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(state_port);
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (::bind(rx_fd, reinterpret_cast<const sockaddr*>(&bind_addr), sizeof(bind_addr)) < 0) {
        std::fprintf(stderr, "UDP bind failed on %u: %s\n", state_port, std::strerror(errno));
        ::close(rx_fd);
        ::close(tx_fd);
        return 1;
    }

    state_transport_udp::MpcFpgaInterface fpga;
    if (!fpga.initialize(mpc_base_addr, dma_base_addr, dma_buffer_phys)) {
        std::fprintf(stderr, "Failed to initialize FPGA DMA interface\n");
        ::close(rx_fd);
        ::close(tx_fd);
        return 1;
    }

    std::fprintf(stdout,
                 "Ultra96 UDP receiver listening on %u (control return %u)\n",
                 state_port,
                 control_port);

    uint32_t last_seq = 0;
    bool have_seq = false;
    uint64_t packet_count = 0;

    while (g_running) {
        state_transport_udp::StatePacket packet{};
        sockaddr_in peer_addr{};
        socklen_t peer_len = sizeof(peer_addr);

        const ssize_t n = ::recvfrom(
            rx_fd,
            &packet,
            sizeof(packet),
            0,
            reinterpret_cast<sockaddr*>(&peer_addr),
            &peer_len);

        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            std::fprintf(stderr, "recvfrom failed: %s\n", std::strerror(errno));
            continue;
        }

        if (n != static_cast<ssize_t>(sizeof(packet))) {
            std::fprintf(stderr, "Dropped packet: expected %zu bytes, got %ld\n",
                         sizeof(packet), static_cast<long>(n));
            continue;
        }

        if (packet.magic != state_transport_udp::PACKET_MAGIC ||
            packet.version != state_transport_udp::PACKET_VERSION) {
            std::fprintf(stderr, "Dropped packet: bad magic/version\n");
            continue;
        }

        const uint32_t rx_crc = packet.crc32;
        packet.crc32 = 0;
        const uint32_t calc_crc = state_transport_udp::crc32_ieee(
            reinterpret_cast<const uint8_t*>(&packet),
            sizeof(packet) - sizeof(packet.crc32));

        if (rx_crc != calc_crc) {
            std::fprintf(stderr, "Dropped packet: CRC mismatch\n");
            continue;
        }

        if (have_seq && packet.sequence != last_seq + 1) {
            std::fprintf(stderr, "Sequence gap: last=%u now=%u\n", last_seq, packet.sequence);
        }
        have_seq = true;
        last_seq = packet.sequence;

        int32_t e_y_fp = 0;
        int32_t e_psi_fp = 0;
        compute_frenet_errors(packet, e_y_fp, e_psi_fp);
        const uint64_t ultra_rx_start_ns = monotonic_now_ns();

        int32_t out_steering_fp = 0;
        int32_t out_accel_fp = 0;
        uint32_t out_status = 0;
        uint32_t out_iterations = 0;

        if (!fpga.compute(e_y_fp,
                          e_psi_fp,
                          packet.velocity_fp,
                          packet.vy_fp,
                          packet.omega_fp,
                          packet.steering_angle_fp,
                          packet,
                          out_steering_fp,
                          out_accel_fp,
                          out_status,
                          out_iterations)) {
            std::fprintf(stderr, "FPGA compute failed\n");
            continue;
        }

        const float vx = state_transport_udp::fp_to_float(packet.velocity_fp);
        const float accel = state_transport_udp::fp_to_float(out_accel_fp);
        float speed = vx + accel * control_dt;
        speed = std::clamp(speed, 0.0f, max_velocity);

        state_transport_udp::ControlPacket ctrl{};
        ctrl.magic = state_transport_udp::PACKET_MAGIC;
        ctrl.version = state_transport_udp::PACKET_VERSION;
        ctrl.flags = 0;
        ctrl.sequence = packet.sequence;
        ctrl.receiver_time_ms = packet.sender_time_ms;
        ctrl.sender_mono_ns = packet.sender_mono_ns;
        ctrl.steering_fp = out_steering_fp;
        ctrl.speed_fp = state_transport_udp::float_to_fp(speed);
        ctrl.accel_fp = out_accel_fp;
        ctrl.solver_status = out_status;
        ctrl.solver_iterations = out_iterations;
        const uint64_t ultra_tx_ns = monotonic_now_ns();
        const uint64_t ultra_delta_ns = ultra_tx_ns - ultra_rx_start_ns;
        ctrl.ultra_process_us = static_cast<uint32_t>(std::min<uint64_t>(ultra_delta_ns / 1000ull, 0xFFFFFFFFull));
        ctrl.reserved = 0;
        ctrl.crc32 = 0;
        ctrl.crc32 = state_transport_udp::crc32_ieee(
            reinterpret_cast<const uint8_t*>(&ctrl),
            sizeof(ctrl) - sizeof(ctrl.crc32));

        sockaddr_in control_dest_addr{};
        control_dest_addr.sin_family = AF_INET;
        control_dest_addr.sin_port = htons(control_port);
        control_dest_addr.sin_addr = peer_addr.sin_addr;

        const ssize_t sent = ::sendto(
            tx_fd,
            &ctrl,
            sizeof(ctrl),
            0,
            reinterpret_cast<const sockaddr*>(&control_dest_addr),
            sizeof(control_dest_addr));

        if (sent != static_cast<ssize_t>(sizeof(ctrl))) {
            std::fprintf(stderr, "Control UDP send failed/short: %ld\n", static_cast<long>(sent));
            continue;
        }

        packet_count++;
        if (packet_count == 1 || packet_count % 100 == 0) {
            std::fprintf(stdout,
                         "UDP RX seq=%u -> steer_fp=%d speed_fp=%d status=%u iters=%u proc=%u us\n",
                         packet.sequence,
                         ctrl.steering_fp,
                         ctrl.speed_fp,
                         ctrl.solver_status,
                         ctrl.solver_iterations,
                         ctrl.ultra_process_us);
        }
    }

    ::close(rx_fd);
    ::close(tx_fd);
    std::fprintf(stdout, "Ultra96 UDP receiver stopped\n");
    return 0;
}
