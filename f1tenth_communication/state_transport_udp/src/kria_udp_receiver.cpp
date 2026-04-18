/**
 * @file kria_udp_receiver.cpp
 * @brief Kria UDP state receiver and FPGA MPC executor via OpenCL.
 * @details Receives UDP state packets from Jetson, computes Frenet errors,
 *          executes OpenCL kernel-backed MPC, and sends control packets back.
 */

#include "state_transport_udp/state_packet.hpp"
#include "mpc_fpga_interface.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <vitis_common/common/ros_opencl_120.hpp>
#include <vitis_common/common/utilities.hpp>

namespace state_transport_udp {

static constexpr int32_t FP_SCALE = 65536;

inline int32_t float_to_fp(float f) {
    return static_cast<int32_t>(f * static_cast<float>(FP_SCALE));
}

inline float fp_to_float(int32_t fp) {
    return static_cast<float>(fp) / static_cast<float>(FP_SCALE);
}

class MpcFpgaInterface {
public:
    MpcFpgaInterface() = default;

    bool initialize(const std::string& xclbin_path,
                    const std::string& kernel_name,
                    int device_index) {
        cl_int err = CL_SUCCESS;
        std::vector<cl::Device> devices = get_xilinx_devices();
        if (devices.empty()) {
            std::fprintf(stderr, "UDP-FPGA-OpenCL: no Xilinx OpenCL devices found\n");
            return false;
        }

        if (device_index < 0 || static_cast<size_t>(device_index) >= devices.size()) {
            std::fprintf(stderr,
                         "UDP-FPGA-OpenCL: device_index=%d out of range (devices=%zu)\n",
                         device_index,
                         devices.size());
            return false;
        }

        device_ = devices[static_cast<size_t>(device_index)];
        context_ = cl::Context(device_, nullptr, nullptr, nullptr, &err);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "UDP-FPGA-OpenCL: context creation failed (%d)\n", err);
            return false;
        }

        queue_ = cl::CommandQueue(context_, device_, CL_QUEUE_PROFILING_ENABLE, &err);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "UDP-FPGA-OpenCL: command queue creation failed (%d)\n", err);
            return false;
        }

        unsigned int file_buf_size = 0;
        std::unique_ptr<char[]> file_buf(read_binary_file(xclbin_path, file_buf_size));
        if (!file_buf || file_buf_size == 0) {
            std::fprintf(stderr, "UDP-FPGA-OpenCL: failed to read xclbin: %s\n", xclbin_path.c_str());
            return false;
        }

        cl::Program::Binaries bins{{file_buf.get(), file_buf_size}};
        std::vector<cl::Device> program_devices{device_};
        program_ = cl::Program(context_, program_devices, bins, nullptr, &err);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "UDP-FPGA-OpenCL: program load failed (%d)\n", err);
            return false;
        }

        kernel_ = cl::Kernel(program_, kernel_name.c_str(), &err);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr,
                         "UDP-FPGA-OpenCL: kernel '%s' creation failed (%d)\n",
                         kernel_name.c_str(), err);
            return false;
        }

        input_buffer_ = cl::Buffer(context_, CL_MEM_READ_ONLY, INPUT_BUFFER_BYTES_512, nullptr, &err);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "UDP-FPGA-OpenCL: input buffer allocation failed (%d)\n", err);
            return false;
        }

        output_buffer_ = cl::Buffer(context_, CL_MEM_WRITE_ONLY, sizeof(uint32_t) * 4, nullptr, &err);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "UDP-FPGA-OpenCL: output buffer allocation failed (%d)\n", err);
            return false;
        }

        err = kernel_.setArg(0, input_buffer_);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "UDP-FPGA-OpenCL: setArg(0) failed (%d)\n", err);
            return false;
        }
        err = kernel_.setArg(1, output_buffer_);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "UDP-FPGA-OpenCL: setArg(1) failed (%d)\n", err);
            return false;
        }

        initialized_ = true;
        return true;
    }

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

        std::fill(input_words_.begin(), input_words_.end(), 0u);

        // Group 0: [e_y | e_psi | vx | vy]
        input_words_[0] = static_cast<uint32_t>(e_y_fp);
        input_words_[1] = static_cast<uint32_t>(e_psi_fp);
        input_words_[2] = static_cast<uint32_t>(vx_fp);
        input_words_[3] = static_cast<uint32_t>(vy_fp);

        // Group 1: [omega | steering | horizon_length | reserved]
        input_words_[4] = static_cast<uint32_t>(omega_fp);
        input_words_[5] = static_cast<uint32_t>(steering_fp);
        input_words_[6] = static_cast<uint32_t>(horizon);
        input_words_[7] = 0;

        // Groups 2..N+1: [ref_vx | ref_kappa | ref_left | ref_right]
        for (size_t i = 0; i < MPC_HORIZON; ++i) {
            const size_t base = 8 + (i * 4);
            if (i < horizon) {
                input_words_[base + 0] = static_cast<uint32_t>(packet.ref_vx_fp[i]);
                input_words_[base + 1] = static_cast<uint32_t>(packet.ref_kappa_fp[i]);
                input_words_[base + 2] = static_cast<uint32_t>(packet.ref_left_bound_fp[i]);
                input_words_[base + 3] = static_cast<uint32_t>(packet.ref_right_bound_fp[i]);
            }
        }

        cl_int err = CL_SUCCESS;
        err = queue_.enqueueWriteBuffer(input_buffer_, CL_FALSE, 0,
                                        INPUT_BUFFER_BYTES_512, input_words_.data());
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "UDP-FPGA-OpenCL: enqueueWriteBuffer failed (%d)\n", err);
            return false;
        }

        err = queue_.enqueueTask(kernel_);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "UDP-FPGA-OpenCL: enqueueTask failed (%d)\n", err);
            return false;
        }

        err = queue_.enqueueReadBuffer(output_buffer_, CL_FALSE, 0,
                                       sizeof(uint32_t) * 4, output_words_.data());
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "UDP-FPGA-OpenCL: enqueueReadBuffer failed (%d)\n", err);
            return false;
        }

        err = queue_.finish();
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "UDP-FPGA-OpenCL: queue finish failed (%d)\n", err);
            return false;
        }

        out_steering_fp = static_cast<int32_t>(output_words_[0]);
        out_accel_fp = static_cast<int32_t>(output_words_[1]);
        out_status = output_words_[2];
        out_iterations = output_words_[3];
        return true;
    }

private:
    bool initialized_{false};

    cl::Device device_;
    cl::Context context_;
    cl::Program program_;
    cl::Kernel kernel_;
    cl::CommandQueue queue_;
    cl::Buffer input_buffer_;
    cl::Buffer output_buffer_;

    static constexpr size_t INPUT_WORDS = INPUT_BUFFER_WORDS_32_PAD;
    std::array<uint32_t, INPUT_WORDS> input_words_{};
    std::array<uint32_t, 4> output_words_{};
};

}  // namespace state_transport_udp

namespace {

volatile sig_atomic_t g_running = 1;

void signal_handler(int) {
    g_running = 0;
}

uint64_t monotonic_now_ns() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ull + static_cast<uint64_t>(ts.tv_nsec);
}

uint32_t env_u32(const char* name, uint32_t fallback) {
    const char* raw = std::getenv(name);
    if (!raw || !*raw) {
        return fallback;
    }
    return static_cast<uint32_t>(std::strtoul(raw, nullptr, 0));
}

float env_f32(const char* name, float fallback) {
    const char* raw = std::getenv(name);
    if (!raw || !*raw) {
        return fallback;
    }
    return std::strtof(raw, nullptr);
}

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

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    const uint16_t state_port = static_cast<uint16_t>(env_u32("UDP_STATE_PORT", 49000));
    const uint16_t control_port = static_cast<uint16_t>(env_u32("UDP_CONTROL_PORT", 49001));
    const float control_dt = env_f32("UDP_CONTROL_DT_S", static_cast<float>(MPC_FPGA_PREDICTION_DT_S));
    const float max_velocity = env_f32("UDP_MAX_VEL_MPS", static_cast<float>(MPC_FPGA_MAX_VEL_MPS));

    const std::string xclbin_path = std::getenv("MPC_XCLBIN")
        ? std::getenv("MPC_XCLBIN")
        : std::string("/lib/firmware/mpc_fpga_top_opencl.xclbin");
    const std::string kernel_name = std::getenv("MPC_KERNEL_NAME")
        ? std::getenv("MPC_KERNEL_NAME")
        : std::string("mpc_fpga_top_opencl");
    const int device_index = static_cast<int>(env_u32("MPC_DEVICE_INDEX", 0));

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
    if (!fpga.initialize(xclbin_path, kernel_name, device_index)) {
        std::fprintf(stderr, "Failed to initialize FPGA OpenCL interface\n");
        ::close(rx_fd);
        ::close(tx_fd);
        return 1;
    }

    std::fprintf(stdout,
                 "Kria UDP receiver (OpenCL) listening on %u (control return %u)\n",
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
        const uint64_t kria_rx_start_ns = monotonic_now_ns();

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
            std::fprintf(stderr, "FPGA OpenCL compute failed\n");
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
        const uint64_t kria_tx_ns = monotonic_now_ns();
        const uint64_t kria_delta_ns = kria_tx_ns - kria_rx_start_ns;
        ctrl.ultra_process_us = static_cast<uint32_t>(std::min<uint64_t>(kria_delta_ns / 1000ull, 0xFFFFFFFFull));
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
    std::fprintf(stdout, "Kria UDP receiver stopped\n");
    return 0;
}
