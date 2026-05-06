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
#include <ctime>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <vector>

#include <vitis_common/common/ros_opencl_120.hpp>
#include <vitis_common/common/utilities.hpp>

namespace state_transport_udp {

static constexpr int32_t FP_SCALE = 65536;

static std::vector<unsigned char> read_file_bytes(const std::string& path) {
    std::ifstream ifs(path, std::ios::binary | std::ios::ate);
    if (!ifs) {
        throw std::runtime_error("Failed to open file: " + path);
    }
    const std::streamoff size = ifs.tellg();
    if (size <= 0) {
        throw std::runtime_error("Empty file: " + path);
    }
    std::vector<unsigned char> buf(static_cast<size_t>(size));
    ifs.seekg(0, std::ios::beg);
    if (!ifs.read(reinterpret_cast<char*>(buf.data()), size)) {
        throw std::runtime_error("Failed to read file: " + path);
    }
    return buf;
}

inline int32_t float_to_fp(float f) {
    return static_cast<int32_t>(
        f >= 0.0f
            ? f * static_cast<float>(FP_SCALE) + 0.5f
            : f * static_cast<float>(FP_SCALE) - 0.5f);
}

inline float fp_to_float(int32_t fp) {
    return static_cast<float>(fp) / static_cast<float>(FP_SCALE);
}

class MpcFpgaInterface {
public:
    MpcFpgaInterface() = default;
    void set_prev_accel_fp(int32_t prev_accel_fp) { prev_accel_fp_ = prev_accel_fp; }

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

        std::vector<unsigned char> file_buf;
        try {
            file_buf = read_file_bytes(xclbin_path);
        } catch (const std::exception& e) {
            std::fprintf(stderr, "UDP-FPGA-OpenCL: xclbin read failed: %s (%s)\n",
                         xclbin_path.c_str(), e.what());
            return false;
        }

        cl::Program::Binaries bins{{file_buf.data(), file_buf.size()}};
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

        input_buffer_ = cl::Buffer(
            context_,
            static_cast<cl_mem_flags>(CL_MEM_READ_ONLY | CL_MEM_ALLOC_HOST_PTR),
            INPUT_BUFFER_BYTES_512,
            nullptr,
            &err);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "UDP-FPGA-OpenCL: input buffer allocation failed (%d)\n", err);
            return false;
        }

        output_buffer_ = cl::Buffer(
            context_,
            static_cast<cl_mem_flags>(CL_MEM_WRITE_ONLY | CL_MEM_ALLOC_HOST_PTR),
            sizeof(uint32_t) * 4,
            nullptr,
            &err);
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

        cl_int err = CL_SUCCESS;
        void* mapped = queue_.enqueueMapBuffer(
            input_buffer_,
            CL_TRUE,
            CL_MAP_WRITE,
            0,
            INPUT_BUFFER_BYTES_512,
            nullptr,
            nullptr,
            &err);
        if (err != CL_SUCCESS || mapped == nullptr) {
            std::fprintf(stderr, "UDP-FPGA-OpenCL: enqueueMapBuffer(input) failed (%d)\n", err);
            return false;
        }

        auto* input_words = reinterpret_cast<uint32_t*>(mapped);
        std::fill(input_words, input_words + INPUT_WORDS, 0u);

        // Group 0: [e_y | e_psi | vx | vy]
        input_words[0] = static_cast<uint32_t>(e_y_fp);
        input_words[1] = static_cast<uint32_t>(e_psi_fp);
        input_words[2] = static_cast<uint32_t>(vx_fp);
        input_words[3] = static_cast<uint32_t>(vy_fp);

        // Group 1: [omega | steering | horizon_length | prev_accel]
        input_words[4] = static_cast<uint32_t>(omega_fp);
        input_words[5] = static_cast<uint32_t>(steering_fp);
        input_words[6] = static_cast<uint32_t>(horizon);
        input_words[7] = static_cast<uint32_t>(prev_accel_fp_);

        // Groups 2..N+1(+): V2 layout, 8 words per step:
        // [ref_ey | ref_epsi | ref_vx | ref_vy | ref_omega_ref | ref_kappa | ref_left | ref_right]
        for (size_t i = 0; i < MPC_HORIZON; ++i) {
            const size_t base = 8 + (i * 8);
            if (i < horizon) {
                input_words[base + 0] = static_cast<uint32_t>(packet.ref_ey_fp[i]);
                /* ref_epsi: not supplied by sender (optional) */
                input_words[base + 1] = static_cast<uint32_t>(0u);
                input_words[base + 2] = static_cast<uint32_t>(packet.ref_vx_fp[i]);
                /* ref_vy: not supplied by sender (optional) */
                input_words[base + 3] = static_cast<uint32_t>(0u);
                /* ref_omega_ref: derive as v * kappa */
                {
                    const float v = state_transport_udp::fp_to_float(packet.ref_vx_fp[i]);
                    const float kap = state_transport_udp::fp_to_float(packet.ref_kappa_fp[i]);
                    const int32_t omega_fp = state_transport_udp::float_to_fp(v * kap);
                    input_words[base + 4] = static_cast<uint32_t>(omega_fp);
                }
                input_words[base + 5] = static_cast<uint32_t>(packet.ref_kappa_fp[i]);
                input_words[base + 6] = static_cast<uint32_t>(packet.ref_left_bound_fp[i]);
                input_words[base + 7] = static_cast<uint32_t>(packet.ref_right_bound_fp[i]);
            }
        }

        err = queue_.enqueueUnmapMemObject(input_buffer_, mapped);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "UDP-FPGA-OpenCL: enqueueUnmapMemObject(input) failed (%d)\n", err);
            return false;
        }

        err = queue_.enqueueMigrateMemObjects({input_buffer_}, 0);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "UDP-FPGA-OpenCL: migrate(input) failed (%d)\n", err);
            return false;
        }

        err = queue_.enqueueTask(kernel_);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "UDP-FPGA-OpenCL: enqueueTask failed (%d)\n", err);
            return false;
        }

        err = queue_.enqueueMigrateMemObjects({output_buffer_}, CL_MIGRATE_MEM_OBJECT_HOST);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "UDP-FPGA-OpenCL: migrate(output) failed (%d)\n", err);
            return false;
        }

        err = queue_.finish();
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "UDP-FPGA-OpenCL: queue finish failed (%d)\n", err);
            return false;
        }

        void* out_mapped = queue_.enqueueMapBuffer(
            output_buffer_,
            CL_TRUE,
            CL_MAP_READ,
            0,
            sizeof(uint32_t) * 4,
            nullptr,
            nullptr,
            &err);
        if (err != CL_SUCCESS || out_mapped == nullptr) {
            std::fprintf(stderr, "UDP-FPGA-OpenCL: enqueueMapBuffer(output) failed (%d)\n", err);
            return false;
        }

        const auto* out_words = reinterpret_cast<const uint32_t*>(out_mapped);
        output_words_[0] = out_words[0];
        output_words_[1] = out_words[1];
        output_words_[2] = out_words[2];
        output_words_[3] = out_words[3];

        err = queue_.enqueueUnmapMemObject(output_buffer_, out_mapped);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "UDP-FPGA-OpenCL: enqueueUnmapMemObject(output) failed (%d)\n", err);
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
    static_assert(INPUT_WORDS * sizeof(uint32_t) == INPUT_BUFFER_BYTES_512,
                  "Host DMA buffer words must match OpenCL input buffer bytes");
    std::array<uint32_t, INPUT_WORDS> input_words_{};
    std::array<uint32_t, 4> output_words_{};
    int32_t prev_accel_fp_{0};
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

    const size_t horizon = static_cast<size_t>(std::max<uint32_t>(1u, packet.horizon_length));
    const size_t max_search = std::min(horizon > 0 ? horizon - 1 : 0, static_cast<size_t>(16));

    float best_e_y = 0.0f;
    float best_e_psi = 0.0f;
    float best_dist2 = 1e18f;

    for (size_t i = 0; i < max_search; ++i) {
        const float ax = state_transport_udp::fp_to_float(packet.ref_x_fp[i]);
        const float ay = state_transport_udp::fp_to_float(packet.ref_y_fp[i]);
        const float bx = state_transport_udp::fp_to_float(packet.ref_x_fp[i + 1]);
        const float by = state_transport_udp::fp_to_float(packet.ref_y_fp[i + 1]);
        const float h0 = state_transport_udp::fp_to_float(packet.ref_psi_fp[i]);
        const float h1 = state_transport_udp::fp_to_float(packet.ref_psi_fp[i + 1]);

        const float abx = bx - ax;
        const float aby = by - ay;
        const float apx = x - ax;
        const float apy = y - ay;
        const float ab_len2 = abx * abx + aby * aby;
        float t = 0.0f;
        if (ab_len2 > 1e-12f) {
            t = (apx * abx + apy * aby) / ab_len2;
        }
        t = std::clamp(t, 0.0f, 1.0f);

        const float wx = ax + t * abx;
        const float wy = ay + t * aby;
        float dpsi_path = h1 - h0;
        while (dpsi_path > static_cast<float>(M_PI)) dpsi_path -= 2.0f * static_cast<float>(M_PI);
        while (dpsi_path < -static_cast<float>(M_PI)) dpsi_path += 2.0f * static_cast<float>(M_PI);
        const float wpsi = h0 + t * dpsi_path;

        const float dx = x - wx;
        const float dy = y - wy;
        const float dist2 = dx * dx + dy * dy;

        if (dist2 < best_dist2) {
            best_dist2 = dist2;
            const float e_y = -std::sin(wpsi) * dx + std::cos(wpsi) * dy;
            float e_psi = theta - wpsi;
            while (e_psi > static_cast<float>(M_PI)) e_psi -= 2.0f * static_cast<float>(M_PI);
            while (e_psi < -static_cast<float>(M_PI)) e_psi += 2.0f * static_cast<float>(M_PI);
            best_e_y = e_y;
            best_e_psi = e_psi;
        }
    }

    out_e_y_fp = state_transport_udp::float_to_fp(best_e_y);
    out_e_psi_fp = state_transport_udp::float_to_fp(best_e_psi);
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

    FILE* stats_csv_file = nullptr;
    {
        const char* log_dir = "log";
        ::mkdir(log_dir, 0755);

        const std::time_t now = std::time(nullptr);
        struct tm tm_now;
        localtime_r(&now, &tm_now);

        char timestamp[64];
        std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &tm_now);

        char csv_path[512];
        std::snprintf(csv_path, sizeof(csv_path), "%s/kria_udp_receiver_%s.csv", log_dir, timestamp);

        stats_csv_file = std::fopen(csv_path, "w");
        if (stats_csv_file != nullptr) {
            std::fprintf(stats_csv_file, "idx,iterations,solve_time_us\n");
            std::fflush(stats_csv_file);
            std::fprintf(stdout, "Kria UDP receiver stats CSV: %s\n", csv_path);
        }
    }

    uint64_t stats_idx = 0;
    uint64_t stats_window_count = 0;
    double solve_sum_us = 0.0;
    double solve_min_us = std::numeric_limits<double>::infinity();
    double solve_max_us = 0.0;
    double iter_sum = 0.0;
    uint32_t iter_min = 0xFFFFFFFFu;
    uint32_t iter_max = 0u;
    uint64_t optimal_count = 0;
    uint64_t max_iter_count = 0;
    uint64_t stats_last_print_ns = monotonic_now_ns();
    constexpr uint64_t kStatsPrintIntervalNs = 5ull * 1000000000ull;

    int32_t last_good_steering_fp = 0;
    int32_t last_good_accel_fp = 0;
    int32_t last_good_speed_fp =
        state_transport_udp::float_to_fp(static_cast<float>(MPC_FPGA_MIN_VEL_MPS));

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
        const uint64_t solve_start_ns = monotonic_now_ns();

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
        const uint64_t solve_end_ns = monotonic_now_ns();
        const double solve_us = static_cast<double>(solve_end_ns - solve_start_ns) / 1e3;

        const float vx = state_transport_udp::fp_to_float(packet.velocity_fp);

        const bool bad_status =
            (out_status == MPC_FPGA_STATUS_ERROR) ||
            (out_status == MPC_FPGA_STATUS_NO_TRAJECTORY);

        state_transport_udp::ControlPacket ctrl{};
        ctrl.magic = state_transport_udp::PACKET_MAGIC;
        ctrl.version = state_transport_udp::PACKET_VERSION;
        ctrl.flags = 0;
        ctrl.sequence = packet.sequence;
        ctrl.receiver_time_ms = packet.sender_time_ms;
        ctrl.sender_mono_ns = packet.sender_mono_ns;
        ctrl.solver_status = out_status;
        ctrl.solver_iterations = out_iterations;

        if (bad_status) {
            ctrl.steering_fp = last_good_steering_fp;
            ctrl.speed_fp = last_good_speed_fp;
            ctrl.accel_fp = last_good_accel_fp;
            fpga.set_prev_accel_fp(last_good_accel_fp);

            std::fprintf(stderr,
                         "Solver bad status=%u at seq=%u, holding last good command\n",
                         out_status,
                         packet.sequence);
        } else {
            const float accel = state_transport_udp::fp_to_float(out_accel_fp);
            float speed = vx + accel * control_dt;
            speed = std::clamp(speed,
                               static_cast<float>(MPC_FPGA_MIN_VEL_MPS),
                               max_velocity);

            ctrl.steering_fp = out_steering_fp;
            ctrl.speed_fp = state_transport_udp::float_to_fp(speed);
            ctrl.accel_fp = out_accel_fp;

            last_good_steering_fp = ctrl.steering_fp;
            last_good_speed_fp = ctrl.speed_fp;
            last_good_accel_fp = ctrl.accel_fp;

            fpga.set_prev_accel_fp(out_accel_fp);
        }
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

        stats_idx++;
        stats_window_count++;
        solve_sum_us += solve_us;
        solve_min_us = std::min(solve_min_us, solve_us);
        solve_max_us = std::max(solve_max_us, solve_us);
        iter_sum += static_cast<double>(out_iterations);
        iter_min = std::min(iter_min, out_iterations);
        iter_max = std::max(iter_max, out_iterations);
        if (out_status == MPC_FPGA_STATUS_OK) {
            optimal_count++;
        }
        if (out_status == MPC_FPGA_STATUS_MAX_ITER) {
            max_iter_count++;
        }

        if (stats_csv_file != nullptr) {
            std::fprintf(stats_csv_file, "%lu,%u,%.1f\n", stats_idx, out_iterations, solve_us);
            std::fflush(stats_csv_file);
        }

        const uint64_t now_ns = monotonic_now_ns();
        if (now_ns - stats_last_print_ns >= kStatsPrintIntervalNs && stats_window_count > 0) {
            const double elapsed_sec = static_cast<double>(now_ns - stats_last_print_ns) / 1e9;
            const double avg_iter = iter_sum / static_cast<double>(stats_window_count);
            const double avg_solve_us = solve_sum_us / static_cast<double>(stats_window_count);
            const double optimal_pct = (optimal_count * 100.0) / static_cast<double>(stats_window_count);
            const double max_iter_pct = (max_iter_count * 100.0) / static_cast<double>(stats_window_count);

            std::fprintf(stdout,
                         "[Kria UDP] Stats (last %.1fs, %lu calls):\n"
                         "  Iterations: min=%u, avg=%.1f, max=%u\n"
                         "  Solve time: min=%.1f us, avg=%.1f us, max=%.1f us\n"
                         "  Optimal: %.1f%%, Max iter: %.1f%%\n",
                         elapsed_sec,
                         stats_window_count,
                         iter_min,
                         avg_iter,
                         iter_max,
                         solve_min_us,
                         avg_solve_us,
                         solve_max_us,
                         optimal_pct,
                         max_iter_pct);

            stats_window_count = 0;
            solve_sum_us = 0.0;
            solve_min_us = std::numeric_limits<double>::infinity();
            solve_max_us = 0.0;
            iter_sum = 0.0;
            iter_min = 0xFFFFFFFFu;
            iter_max = 0u;
            optimal_count = 0;
            max_iter_count = 0;
            stats_last_print_ns = now_ns;
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
    if (stats_csv_file != nullptr) {
        std::fclose(stats_csv_file);
        stats_csv_file = nullptr;
    }
    std::fprintf(stdout, "Kria UDP receiver stopped\n");
    return 0;
}
