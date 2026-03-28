/**
 * @file ultra96_udp_receiver.cpp
 * @brief Ultra96-side UDP state receiver and FPGA MPC executor.
 * @details Receives UDP state packets from Jetson, loads horizon/state into
 *          FPGA registers, runs MPC compute, and sends UDP control packets back.
 * @dependencies state_packet.hpp, mpc_fpga_interface.h, POSIX sockets, /dev/mem
 */

#include "state_transport_udp/state_packet.hpp"
#include "mpc_fpga_interface.h"

#include <arpa/inet.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>

#include <algorithm>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <cstring>
#include <string>

namespace state_transport_udp {

static constexpr int32_t FP_SCALE = 65536;
static constexpr uint32_t AP_START = 0x01;
static constexpr uint32_t AP_DONE = 0x02;
static constexpr uint32_t AP_IDLE = 0x04;

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

class MpcFpgaInterface {
public:
    /**
     * @brief Destroy interface and release mapped FPGA resources.
     * @return None
     */
    ~MpcFpgaInterface() { close_device(); }

    /**
     * @brief Initialize FPGA memory mapping through `/dev/mem`.
     * @param base_addr Physical AXI base address for FPGA register block.
     * @param map_size Size in bytes of memory region to map.
     * @return true when mapping succeeds and interface is ready.
     */
    bool initialize(uint32_t base_addr = MPC_FPGA_BASE_ADDR, size_t map_size = 0x10000) {
        base_addr_ = base_addr;
        map_size_ = map_size;

        mem_fd_ = ::open("/dev/mem", O_RDWR | O_SYNC);
        if (mem_fd_ < 0) {
            std::fprintf(stderr, "MPC-FPGA: open /dev/mem failed: %s\n", std::strerror(errno));
            return false;
        }

        fpga_base_ = ::mmap(nullptr, map_size_, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd_, base_addr_);
        if (fpga_base_ == MAP_FAILED) {
            std::fprintf(stderr, "MPC-FPGA: mmap failed: %s\n", std::strerror(errno));
            ::close(mem_fd_);
            mem_fd_ = -1;
            fpga_base_ = nullptr;
            return false;
        }

        initialized_ = true;
        return true;
    }

    /**
     * @brief Close FPGA mapping and reset internal interface state.
     * @return None
     */
    void close_device() {
        if (fpga_base_ && fpga_base_ != MAP_FAILED) {
            ::munmap(fpga_base_, map_size_);
            fpga_base_ = nullptr;
        }
        if (mem_fd_ >= 0) {
            ::close(mem_fd_);
            mem_fd_ = -1;
        }
        initialized_ = false;
        trajectory_loaded_ = false;
    }

    /**
     * @brief Load streamed horizon waypoints into FPGA waypoint memory.
     * @param pkt Incoming state packet with horizon arrays.
     * @param left_bound_fp Left track bound in Q16.16.
     * @param right_bound_fp Right track bound in Q16.16.
     * @return true when full waypoint load and commit complete successfully.
     */
    bool load_horizon(const StatePacket& pkt, int32_t left_bound_fp, int32_t right_bound_fp) {
        if (!initialized_) {
            return false;
        }

        const size_t count = std::min(static_cast<size_t>(pkt.horizon_length), MPC_HORIZON);
        if (count == 0) {
            return false;
        }

        for (size_t i = 0; i < count; ++i) {
            if (!wait_idle(50000)) {
                return false;
            }

            write_reg(REG_MODE, 1);
            write_reg(REG_WP_INDEX, static_cast<uint32_t>(i));
            write_reg(REG_WP_X, static_cast<uint32_t>(pkt.ref_x_fp[i]));
            write_reg(REG_WP_Y, static_cast<uint32_t>(pkt.ref_y_fp[i]));
            write_reg(REG_WP_PSI, static_cast<uint32_t>(pkt.ref_psi_fp[i]));
            write_reg(REG_WP_VX, static_cast<uint32_t>(pkt.ref_vx_fp[i]));
            write_reg(REG_WP_KAPPA, static_cast<uint32_t>(pkt.ref_kappa_fp[i]));
            write_reg(REG_WP_AX, static_cast<uint32_t>(pkt.ref_ax_fp[i]));
            write_reg(REG_WP_LEFT_BOUND, static_cast<uint32_t>(left_bound_fp));
            write_reg(REG_WP_RIGHT_BOUND, static_cast<uint32_t>(right_bound_fp));
            write_reg(REG_WP_TOTAL, static_cast<uint32_t>(count));
            __sync_synchronize();

            write_reg(REG_AP_CTRL, AP_START);
            if (!wait_done(100000)) {
                return false;
            }
        }

        if (!wait_idle(50000)) {
            return false;
        }
        write_reg(REG_MODE, 2);
        write_reg(REG_WP_TOTAL, static_cast<uint32_t>(count));
        __sync_synchronize();
        write_reg(REG_AP_CTRL, AP_START);
        if (!wait_done(100000)) {
            return false;
        }

        write_reg(REG_MODE, 0);
        __sync_synchronize();

        trajectory_loaded_ = true;
        return true;
    }

    /**
     * @brief Run one FPGA MPC compute step from state packet input.
     * @param pkt Incoming state packet with current vehicle state.
     * @param out_steering_fp Output steering command in Q16.16.
     * @param out_accel_fp Output acceleration command in Q16.16.
     * @param out_status Output solver status flags.
     * @param out_iterations Output solver iteration count.
     * @return true when compute and output-readback succeed.
     */
    bool compute(const StatePacket& pkt,
                 int32_t& out_steering_fp,
                 int32_t& out_accel_fp,
                 uint32_t& out_status,
                 uint32_t& out_iterations) {
        if (!initialized_ || !trajectory_loaded_) {
            return false;
        }

        if (!wait_idle(10000)) {
            return false;
        }

        write_reg(REG_MODE, 0);
        write_reg(REG_ST_X, static_cast<uint32_t>(pkt.x_fp));
        write_reg(REG_ST_Y, static_cast<uint32_t>(pkt.y_fp));
        write_reg(REG_ST_THETA, static_cast<uint32_t>(pkt.theta_fp));
        write_reg(REG_ST_VX, static_cast<uint32_t>(pkt.velocity_fp));
        write_reg(REG_ST_VY, static_cast<uint32_t>(pkt.vy_fp));
        write_reg(REG_ST_OMEGA, static_cast<uint32_t>(pkt.omega_fp));
        write_reg(REG_ST_STEERING, static_cast<uint32_t>(pkt.steering_angle_fp));
        write_reg(REG_ST_WP_IDX, pkt.waypoint_index);
        __sync_synchronize();

        write_reg(REG_AP_CTRL, AP_START);
        if (!wait_done(200000)) {
            return false;
        }

        if (!wait_output_valid(50000)) {
            return false;
        }

        out_steering_fp = static_cast<int32_t>(read_reg(REG_OUT_STEERING));
        out_accel_fp = static_cast<int32_t>(read_reg(REG_OUT_ACCEL));
        out_status = read_reg(REG_OUT_STATUS);
        out_iterations = read_reg(REG_OUT_ITERATIONS);
        return true;
    }

private:
    /**
     * @brief Write value to mapped FPGA register offset.
     * @param offset Register byte offset.
     * @param value Value to write.
     * @return None
     */
    void write_reg(uint32_t offset, uint32_t value) {
        volatile uint32_t* reg = reinterpret_cast<volatile uint32_t*>(
            static_cast<volatile uint8_t*>(fpga_base_) + offset);
        *reg = value;
    }

    /**
     * @brief Read value from mapped FPGA register offset.
     * @param offset Register byte offset.
     * @return Register value read from mapped region.
     */
    uint32_t read_reg(uint32_t offset) {
        volatile uint32_t* reg = reinterpret_cast<volatile uint32_t*>(
            static_cast<volatile uint8_t*>(fpga_base_) + offset);
        return *reg;
    }

    /**
     * @brief Wait for FPGA core idle state.
     * @param timeout_cycles Maximum polling cycles.
     * @return true when AP_IDLE is observed before timeout.
     */
    bool wait_idle(int timeout_cycles) {
        while (timeout_cycles-- > 0) {
            if (read_reg(REG_AP_CTRL) & AP_IDLE) {
                return true;
            }
        }
        return false;
    }

    /**
     * @brief Wait for FPGA core done state.
     * @param timeout_cycles Maximum polling cycles.
     * @return true when AP_DONE is observed before timeout.
     */
    bool wait_done(int timeout_cycles) {
        while (timeout_cycles-- > 0) {
            if (read_reg(REG_AP_CTRL) & AP_DONE) {
                return true;
            }
        }
        return false;
    }

    /**
     * @brief Wait for steering and acceleration output valid flags.
     * @param timeout_cycles Maximum polling cycles.
     * @return true when both valid flags are asserted.
     */
    bool wait_output_valid(int timeout_cycles) {
        while (timeout_cycles-- > 0) {
            const uint32_t steer_vld = read_reg(REG_OUT_STEERING_VLD);
            const uint32_t accel_vld = read_reg(REG_OUT_ACCEL_VLD);
            if ((steer_vld & 0x1u) && (accel_vld & 0x1u)) {
                return true;
            }
        }
        return false;
    }

    int mem_fd_{-1};
    void* fpga_base_{nullptr};
    uint32_t base_addr_{0};
    size_t map_size_{0};
    bool initialized_{false};
    bool trajectory_loaded_{false};
};

}  // namespace state_transport_udp

namespace {
volatile sig_atomic_t g_running = 1;

/**
 * @brief Handle process termination signals.
 * @param Unused signal number.
 * @return None
 */
void signal_handler(int) {
    g_running = 0;
}

/**
 * @brief Read monotonic clock in nanoseconds.
 * @return Monotonic timestamp in nanoseconds.
 */
uint64_t monotonic_now_ns() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ull + static_cast<uint64_t>(ts.tv_nsec);
}

}  // namespace

/**
 * @brief Entry point for Ultra96 UDP receiver process.
 * @param argc Argument count from process invocation.
 * @param argv Argument vector from process invocation.
 * @return Process exit code (0 on normal shutdown, non-zero on startup failure).
 */
int main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    constexpr uint16_t kStatePort = 49000;
    constexpr uint16_t kControlPort = 49001;
    constexpr float kControlDt = 0.02f;
    constexpr float kMaxVelocity = 12.0f;

    const int32_t left_bound_fp = state_transport_udp::float_to_fp(2.0f);
    const int32_t right_bound_fp = state_transport_udp::float_to_fp(2.0f);

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
    bind_addr.sin_port = htons(kStatePort);
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (::bind(rx_fd, reinterpret_cast<const sockaddr*>(&bind_addr), sizeof(bind_addr)) < 0) {
        std::fprintf(stderr, "UDP bind failed on %u: %s\n", kStatePort, std::strerror(errno));
        ::close(rx_fd);
        ::close(tx_fd);
        return 1;
    }

    state_transport_udp::MpcFpgaInterface fpga;
    if (!fpga.initialize(MPC_FPGA_BASE_ADDR)) {
        std::fprintf(stderr, "Failed to initialize FPGA interface\n");
        ::close(rx_fd);
        ::close(tx_fd);
        return 1;
    }

    std::fprintf(stdout, "Ultra96 UDP receiver listening on port %u\n", kStatePort);

    uint32_t last_seq = 0;
    bool have_seq = false;

    while (g_running) {
        const uint64_t ultra_rx_start_ns = monotonic_now_ns();
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

        if (!fpga.load_horizon(packet, left_bound_fp, right_bound_fp)) {
            std::fprintf(stderr, "FPGA load_horizon failed\n");
            continue;
        }

        int32_t out_steering_fp = 0;
        int32_t out_accel_fp = 0;
        uint32_t out_status = 0;
        uint32_t out_iterations = 0;
        if (!fpga.compute(packet, out_steering_fp, out_accel_fp, out_status, out_iterations)) {
            std::fprintf(stderr, "FPGA compute failed\n");
            continue;
        }

        const float vx = state_transport_udp::fp_to_float(packet.velocity_fp);
        const float accel = state_transport_udp::fp_to_float(out_accel_fp);
        float speed = vx + accel * kControlDt;
        speed = std::clamp(speed, 0.0f, kMaxVelocity);

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
        control_dest_addr.sin_port = htons(kControlPort);
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
        }

        std::fprintf(stdout,
                     "RX seq=%u wp=%u -> steer_fp=%d speed_fp=%d status=%u iters=%u proc=%u us\n",
                     packet.sequence,
                     packet.waypoint_index,
                     ctrl.steering_fp,
                     ctrl.speed_fp,
                     ctrl.solver_status,
                     ctrl.solver_iterations,
                     ctrl.ultra_process_us);
    }

    ::close(rx_fd);
    ::close(tx_fd);
    std::fprintf(stdout, "Ultra96 UDP receiver stopped\n");
    return 0;
}
