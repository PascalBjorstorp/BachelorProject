/**
 * @file kria_udp_receiver.cpp
 * @brief Kria UDP state receiver and FPGA MPC executor via OpenCL.
 * @details Receives UDP state packets from Jetson, executes OpenCL
 *          kernel-backed MPC, and sends control packets back.
 *
 * Transport: UDP (StatePacket -> ControlPacket). Launchable via ROS2 for
 * convenience, but data path Jetson<->Kria remains UDP.
 */

#include <rclcpp/rclcpp.hpp>

#include "state_transport_udp/state_packet.hpp"
#include "mpc_fpga_interface.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <vector>

#include <vitis_common/common/ros_opencl_120.hpp>
#include <vitis_common/common/utilities.hpp>

namespace state_transport_udp {

static constexpr int32_t FP_SCALE = MPC_FPGA_QP_SCALE_I32;

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
    struct TimingProfile {
        int64_t total_call_ns = 0;
        int64_t input_map_ns = 0;
        int64_t input_pack_ns = 0;
        int64_t input_unmap_ns = 0;
        int64_t input_migrate_ns = -1;
        int64_t kernel_ns = -1;
        int64_t output_migrate_ns = -1;
        int64_t output_map_ns = 0;
        int64_t output_unpack_ns = 0;
        int64_t output_unmap_ns = 0;
    };

    MpcFpgaInterface() = default;
    void set_prev_accel_fp(int32_t prev_accel_fp) { prev_accel_fp_ = prev_accel_fp; }
    const TimingProfile& get_last_profile() const { return last_profile_; }
    int64_t get_last_compute_ns() const { return last_compute_ns_; }

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
        if (!initialized_) return false;

        last_profile_ = TimingProfile{};
        cl_int err = CL_SUCCESS;
        const auto total_t0 = std::chrono::high_resolution_clock::now();
        const auto input_pack_t0 = total_t0;
        auto* input_words = input_words_.data();
        std::fill(input_words_.begin(), input_words_.end(), 0);

        input_words[MPC_FPGA_WORD_EY] = static_cast<uint32_t>(e_y_fp);
        input_words[MPC_FPGA_WORD_EPSI] = static_cast<uint32_t>(e_psi_fp);
        input_words[MPC_FPGA_WORD_VX] = static_cast<uint32_t>(vx_fp);
        input_words[MPC_FPGA_WORD_VY] = static_cast<uint32_t>(vy_fp);
        input_words[MPC_FPGA_WORD_OMEGA] = static_cast<uint32_t>(omega_fp);
        input_words[MPC_FPGA_WORD_STEERING] = static_cast<uint32_t>(steering_fp);
        input_words[MPC_FPGA_WORD_CONTROL_FLAGS] = MPC_FPGA_CTRL_FLAGS_NONE;
        input_words[MPC_FPGA_WORD_PREV_ACCEL] = static_cast<uint32_t>(prev_accel_fp_);

        for (size_t i = 0; i < MPC_HORIZON; ++i) {
            const size_t base = MPC_FPGA_HEADER_WORDS + (i * MPC_FPGA_REF_WORDS);
            input_words[base + 0] = static_cast<uint32_t>(packet.ref_ey_fp[i]);
            input_words[base + 1] = static_cast<uint32_t>(packet.ref_epsi_fp[i]);
            input_words[base + 2] = static_cast<uint32_t>(packet.ref_vx_fp[i]);
            input_words[base + 3] = static_cast<uint32_t>(packet.ref_vy_fp[i]);
            input_words[base + 4] = static_cast<uint32_t>(packet.ref_omega_ref_fp[i]);
            input_words[base + 5] = static_cast<uint32_t>(packet.ref_kappa_fp[i]);
            input_words[base + 6] = static_cast<uint32_t>(packet.ref_left_bound_fp[i]);
            input_words[base + 7] = static_cast<uint32_t>(packet.ref_right_bound_fp[i]);
        }
        const auto input_pack_t1 = std::chrono::high_resolution_clock::now();
        last_profile_.input_pack_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            input_pack_t1 - input_pack_t0).count();

        cl::Event input_migrate_event;
        cl::Event kernel_event;
        cl::Event output_migrate_event;

        err = queue_.enqueueWriteBuffer(
            input_buffer_,
            CL_FALSE,
            0,
            INPUT_BUFFER_BYTES_512,
            input_words_.data(),
            nullptr,
            &input_migrate_event);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "UDP-FPGA-OpenCL: write(input) failed (%d)\n", err);
            last_compute_ns_ = -1;
            return false;
        }

        const auto t0 = std::chrono::high_resolution_clock::now();
        std::vector<cl::Event> wait_input{input_migrate_event};
        err = queue_.enqueueTask(kernel_, &wait_input, &kernel_event);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "UDP-FPGA-OpenCL: enqueueTask failed (%d)\n", err);
            last_compute_ns_ = -1;
            return false;
        }

        std::vector<cl::Event> wait_kernel{kernel_event};
        err = queue_.enqueueReadBuffer(
            output_buffer_,
            CL_FALSE,
            0,
            sizeof(uint32_t) * 4,
            output_words_.data(),
            &wait_kernel,
            &output_migrate_event);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "UDP-FPGA-OpenCL: read(output) failed (%d)\n", err);
            last_compute_ns_ = -1;
            return false;
        }

        err = cl::Event::waitForEvents({output_migrate_event});
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "UDP-FPGA-OpenCL: output wait failed (%d)\n", err);
            last_compute_ns_ = -1;
            return false;
        }
        const auto t1 = std::chrono::high_resolution_clock::now();
        last_compute_ns_ =
            std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
        last_profile_.input_migrate_ns = get_event_duration_ns(input_migrate_event);
        last_profile_.kernel_ns = get_event_duration_ns(kernel_event);
        last_profile_.output_migrate_ns = get_event_duration_ns(output_migrate_event);

        out_steering_fp = static_cast<int32_t>(output_words_[0]);
        out_accel_fp = static_cast<int32_t>(output_words_[1]);
        out_status = output_words_[2];
        out_iterations = output_words_[3];
        last_profile_.total_call_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            t1 - total_t0).count();
        return true;
    }

private:
    static int64_t get_event_duration_ns(const cl::Event& event) {
        cl_int err = CL_SUCCESS;
        const cl_ulong start = event.getProfilingInfo<CL_PROFILING_COMMAND_START>(&err);
        if (err != CL_SUCCESS) {
            return -1;
        }
        const cl_ulong end = event.getProfilingInfo<CL_PROFILING_COMMAND_END>(&err);
        if (err != CL_SUCCESS) {
            return -1;
        }
        return end >= start ? static_cast<int64_t>(end - start) : -1;
    }

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
    TimingProfile last_profile_{};
    int64_t last_compute_ns_{0};
};

class KriaUdpReceiverNode final : public rclcpp::Node {
public:
    KriaUdpReceiverNode() : Node("kria_udp_receiver") {
        declare_parameter<int>("state_port", 49000);
        declare_parameter<int>("control_port", 49001);
        declare_parameter<std::string>(
            "xclbin_path",
            "/lib/firmware/xilinx/kr260_mpc_app/mpc_fpga_top_opencl.xclbin");
        declare_parameter<std::string>("kernel_name", "mpc_fpga_top_opencl");
        declare_parameter<int>("device_index", 0);
        declare_parameter<int>("poll_period_us", static_cast<int>(MPC_FPGA_BRIDGE_POLL_PERIOD_US));

        state_port_ = static_cast<uint16_t>(get_parameter("state_port").as_int());
        control_port_ = static_cast<uint16_t>(get_parameter("control_port").as_int());
        const std::string xclbin_path = get_parameter("xclbin_path").as_string();
        const std::string kernel_name = get_parameter("kernel_name").as_string();
        const int device_index = get_parameter("device_index").as_int();
        const int poll_us =
            std::max(50, static_cast<int>(get_parameter("poll_period_us").as_int()));

        if (!fpga_.initialize(xclbin_path, kernel_name, device_index)) {
            throw std::runtime_error("UDP-FPGA-OpenCL init failed");
        }

        rx_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (rx_fd_ < 0) throw std::runtime_error("Failed to create UDP RX socket");
        tx_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (tx_fd_ < 0) throw std::runtime_error("Failed to create UDP TX socket");

        int flags = ::fcntl(rx_fd_, F_GETFL, 0);
        if (flags < 0 || ::fcntl(rx_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
            throw std::runtime_error("Failed to set RX socket non-blocking");
        }

        sockaddr_in bind_addr{};
        bind_addr.sin_family = AF_INET;
        bind_addr.sin_port = htons(state_port_);
        bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        if (::bind(rx_fd_, reinterpret_cast<const sockaddr*>(&bind_addr), sizeof(bind_addr)) < 0) {
            throw std::runtime_error("Failed to bind UDP RX socket");
        }

        stats_csv_file_ = open_stats_csv();

        recv_timer_ = create_wall_timer(
            std::chrono::microseconds(poll_us),
            std::bind(&KriaUdpReceiverNode::poll_socket, this));

        stats_timer_ = create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&KriaUdpReceiverNode::print_stats, this));

        RCLCPP_INFO(get_logger(),
                    "Kria UDP receiver ready: state_port=%u control_port=%u poll=%d us",
                    static_cast<unsigned>(state_port_),
                    static_cast<unsigned>(control_port_),
                    poll_us);
    }

    ~KriaUdpReceiverNode() override {
        if (rx_fd_ >= 0) ::close(rx_fd_);
        if (tx_fd_ >= 0) ::close(tx_fd_);
        if (stats_csv_file_ != nullptr) std::fclose(stats_csv_file_);
    }

private:
    static constexpr double TERMINAL_STATS_PRINT_INTERVAL_SECONDS = 5.0;

    static FILE* open_stats_csv() {
        const char* log_dir = "log";
        ::mkdir(log_dir, 0755);

        const std::time_t now = std::time(nullptr);
        struct tm tm_now;
        localtime_r(&now, &tm_now);
        char timestamp[64];
        std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &tm_now);

        char csv_path[512];
        std::snprintf(csv_path, sizeof(csv_path), "%s/kria_udp_receiver_%s.stats.csv", log_dir, timestamp);
        FILE* f = std::fopen(csv_path, "w");
        if (f != nullptr) {
            std::fprintf(f,
                         "idx,iterations,solve_time_us,total_call_us,kernel_time_us,"
                         "input_migrate_us,output_migrate_us,host_prep_us,host_readback_us,"
                         "opencl_overhead_us,input_map_us,input_pack_us,input_unmap_us,"
                         "output_map_us,output_unpack_us,output_unmap_us\n");
            std::fflush(f);
        }
        return f;
    }

    void poll_socket() {
        StatePacket packet{};
        sockaddr_in peer_addr{};
        socklen_t peer_len = sizeof(peer_addr);

        while (true) {
            const ssize_t n = ::recvfrom(
                rx_fd_,
                &packet,
                sizeof(packet),
                0,
                reinterpret_cast<sockaddr*>(&peer_addr),
                &peer_len);

            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) break;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "UDP recvfrom error: %s", std::strerror(errno));
                break;
            }
            if (n != static_cast<ssize_t>(sizeof(packet))) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "Dropped state packet: expected %zu bytes, got %ld",
                                     sizeof(packet), static_cast<long>(n));
                continue;
            }
            if (packet.magic != PACKET_MAGIC || packet.version != PACKET_VERSION) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "Dropped state packet: bad magic/version");
                continue;
            }

            const uint32_t rx_crc = packet.crc32;
            packet.crc32 = 0;
            const uint32_t calc_crc = crc32_ieee(
                reinterpret_cast<const uint8_t*>(&packet),
                sizeof(packet) - sizeof(packet.crc32));
            if (rx_crc != calc_crc) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "Dropped state packet: CRC mismatch");
                continue;
            }

            last_peer_addr_ = peer_addr;
            have_peer_ = true;
            handle_packet(packet);
        }
    }

    void handle_packet(const StatePacket& packet) {
        const int32_t e_y_fp = packet.e_y_fp;
        const int32_t e_psi_fp = packet.e_psi_fp;
        const int32_t vx_fp = packet.velocity_fp;
        const int32_t vy_fp = packet.vy_fp;
        const int32_t omega_fp = packet.omega_fp;
        const int32_t steering_fp = packet.steering_angle_fp;

        int32_t out_steering_fp = 0;
        int32_t out_accel_fp = 0;
        uint32_t out_status = MPC_FPGA_STATUS_ERROR;
        uint32_t out_iterations = 0;

        const bool ok = fpga_.compute(
            e_y_fp, e_psi_fp, vx_fp, vy_fp, omega_fp, steering_fp,
            packet,
            out_steering_fp, out_accel_fp, out_status, out_iterations);
        const int64_t compute_ns = fpga_.get_last_compute_ns();
        const auto& profile = fpga_.get_last_profile();
        const double solve_us = compute_ns >= 0 ? static_cast<double>(compute_ns) / 1000.0 : -1.0;

        ControlPacket ctrl{};
        ctrl.magic = PACKET_MAGIC;
        ctrl.version = PACKET_VERSION;
        ctrl.flags = 0;
        ctrl.sequence = packet.sequence;
        ctrl.receiver_time_ms = static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count() & 0xFFFFFFFFu);
        ctrl.sender_mono_ns = packet.sender_mono_ns;
        ctrl.source_stamp_sec = packet.source_stamp_sec;
        ctrl.source_stamp_nanosec = packet.source_stamp_nanosec;
        ctrl.solver_status = out_status;
        ctrl.solver_iterations = out_iterations;
        ctrl.reserved = 0;

        if (!ok || out_status == MPC_FPGA_STATUS_ERROR || out_status == MPC_FPGA_STATUS_NO_TRAJECTORY) {
            ctrl.steering_fp = last_good_steering_fp_;
            ctrl.accel_fp = 0;
            ctrl.speed_fp = last_good_speed_fp_;
        } else {
            const float vx = fp_to_float(packet.velocity_fp);
            const float accel = fp_to_float(out_accel_fp);
            float speed = vx + accel * static_cast<float>(MPC_FPGA_PREDICTION_DT_S);
            speed = std::clamp(speed,
                               static_cast<float>(MPC_FPGA_MIN_VEL_MPS),
                               static_cast<float>(MPC_FPGA_MAX_VEL_MPS));
            ctrl.steering_fp = out_steering_fp;
            ctrl.accel_fp = out_accel_fp;
            ctrl.speed_fp = float_to_fp(speed);

            last_good_steering_fp_ = ctrl.steering_fp;
            last_good_speed_fp_ = ctrl.speed_fp;
            fpga_.set_prev_accel_fp(out_accel_fp);
        }

        ctrl.ultra_process_us =
            static_cast<uint32_t>(std::clamp(solve_us, 0.0, static_cast<double>(0xFFFFFFFFu)));
        ctrl.crc32 = 0;
        ctrl.crc32 = crc32_ieee(reinterpret_cast<const uint8_t*>(&ctrl),
                                sizeof(ctrl) - sizeof(ctrl.crc32));

        if (have_peer_) {
            sockaddr_in control_dest_addr{};
            control_dest_addr.sin_family = AF_INET;
            control_dest_addr.sin_port = htons(control_port_);
            control_dest_addr.sin_addr = last_peer_addr_.sin_addr;

            const ssize_t sent = ::sendto(
                tx_fd_,
                &ctrl,
                sizeof(ctrl),
                0,
                reinterpret_cast<const sockaddr*>(&control_dest_addr),
                sizeof(control_dest_addr));

            if (sent != static_cast<ssize_t>(sizeof(ctrl))) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "Control UDP send failed/short: %ld", static_cast<long>(sent));
            }
        }

        stats_idx_++;
        stats_window_count_++;
        if (solve_us >= 0.0) {
            stats_solve_sum_us_ += solve_us;
            stats_solve_min_us_ = std::min(stats_solve_min_us_, solve_us);
            stats_solve_max_us_ = std::max(stats_solve_max_us_, solve_us);
        }
        stats_iter_sum_ += static_cast<double>(out_iterations);
        stats_iter_min_ = std::min(stats_iter_min_, out_iterations);
        stats_iter_max_ = std::max(stats_iter_max_, out_iterations);
        update_stats(profile, out_iterations, out_status, solve_us);
    }

    void print_stats() {
        if (stats_window_count_ == 0) return;
        const double avg_us = stats_valid_solve_count_ > 0
            ? (stats_solve_sum_us_ / static_cast<double>(stats_valid_solve_count_))
            : -1.0;
        const double avg_iter = stats_iter_sum_ / static_cast<double>(stats_window_count_);
        RCLCPP_INFO(get_logger(),
                    "[FPGA-Stats] (last %.1fs, %lu calls):\n"
                    "  Iterations: min=%u, avg=%.1f, max=%u\n"
                    "  Solve time: min=%.1f us, avg=%.1f us, max=%.1f us\n"
                    "  Profile avg: total=%.1f us, kernel=%.1f us, in_mig=%.1f us, out_mig=%.1f us, host_prep=%.1f us, overhead=%.1f us\n"
                    "  Optimal: %.1f%%, Max iter: %.1f%%",
                    TERMINAL_STATS_PRINT_INTERVAL_SECONDS,
                    stats_window_count_,
                    stats_iter_min_, avg_iter, stats_iter_max_,
                    stats_solve_min_us_, avg_us, stats_solve_max_us_,
                    avg_or_na(stats_total_call_sum_us_, stats_profile_count_),
                    avg_or_na(stats_kernel_sum_us_, stats_profile_count_),
                    avg_or_na(stats_input_migrate_sum_us_, stats_profile_count_),
                    avg_or_na(stats_output_migrate_sum_us_, stats_profile_count_),
                    avg_or_na(stats_host_prep_sum_us_, stats_profile_count_),
                    avg_or_na(stats_overhead_sum_us_, stats_profile_count_),
                    stats_window_count_ > 0 ? (stats_optimal_count_ * 100.0) / static_cast<double>(stats_window_count_) : 0.0,
                    stats_window_count_ > 0 ? (stats_max_iter_count_ * 100.0) / static_cast<double>(stats_window_count_) : 0.0);

        stats_window_count_ = 0;
        stats_valid_solve_count_ = 0;
        stats_solve_sum_us_ = 0.0;
        stats_solve_min_us_ = std::numeric_limits<double>::infinity();
        stats_solve_max_us_ = 0.0;
        stats_iter_sum_ = 0.0;
        stats_iter_min_ = 0xFFFFFFFFu;
        stats_iter_max_ = 0u;
        stats_total_call_sum_us_ = 0.0;
        stats_kernel_sum_us_ = 0.0;
        stats_input_migrate_sum_us_ = 0.0;
        stats_output_migrate_sum_us_ = 0.0;
        stats_host_prep_sum_us_ = 0.0;
        stats_host_readback_sum_us_ = 0.0;
        stats_overhead_sum_us_ = 0.0;
        stats_profile_count_ = 0;
        stats_optimal_count_ = 0;
        stats_max_iter_count_ = 0;
    }

    static double ns_to_us(int64_t ns) {
        return ns >= 0 ? static_cast<double>(ns) / 1000.0 : -1.0;
    }

    static double avg_or_na(double sum, uint64_t count) {
        return count > 0 ? (sum / static_cast<double>(count)) : -1.0;
    }

    void update_stats(const MpcFpgaInterface::TimingProfile& profile,
                      uint32_t iterations,
                      uint32_t status,
                      double solve_us) {
        const double total_call_us = ns_to_us(profile.total_call_ns);
        const double kernel_us = ns_to_us(profile.kernel_ns);
        const double input_migrate_us = ns_to_us(profile.input_migrate_ns);
        const double output_migrate_us = ns_to_us(profile.output_migrate_ns);
        const double host_prep_us = ns_to_us(
            profile.input_map_ns + profile.input_pack_ns + profile.input_unmap_ns);
        const double host_readback_us = ns_to_us(
            profile.output_map_ns + profile.output_unpack_ns + profile.output_unmap_ns);
        const double overhead_us = (profile.total_call_ns > 0 && profile.kernel_ns >= 0)
            ? static_cast<double>(profile.total_call_ns - profile.kernel_ns) / 1000.0
            : -1.0;

        if (total_call_us >= 0.0) stats_total_call_sum_us_ += total_call_us;
        if (kernel_us >= 0.0) stats_kernel_sum_us_ += kernel_us;
        if (input_migrate_us >= 0.0) stats_input_migrate_sum_us_ += input_migrate_us;
        if (output_migrate_us >= 0.0) stats_output_migrate_sum_us_ += output_migrate_us;
        if (host_prep_us >= 0.0) stats_host_prep_sum_us_ += host_prep_us;
        if (host_readback_us >= 0.0) stats_host_readback_sum_us_ += host_readback_us;
        if (overhead_us >= 0.0) stats_overhead_sum_us_ += overhead_us;
        if (solve_us >= 0.0) stats_valid_solve_count_++;
        stats_profile_count_++;
        if (status == MPC_FPGA_STATUS_OK) stats_optimal_count_++;
        if (status == MPC_FPGA_STATUS_MAX_ITER) stats_max_iter_count_++;

        if (stats_csv_file_ != nullptr) {
            std::fprintf(stats_csv_file_,
                         "%lu,%u,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
                         stats_idx_,
                         iterations,
                         solve_us,
                         total_call_us,
                         kernel_us,
                         input_migrate_us,
                         output_migrate_us,
                         host_prep_us,
                         host_readback_us,
                         overhead_us,
                         ns_to_us(profile.input_map_ns),
                         ns_to_us(profile.input_pack_ns),
                         ns_to_us(profile.input_unmap_ns),
                         ns_to_us(profile.output_map_ns),
                         ns_to_us(profile.output_unpack_ns),
                         ns_to_us(profile.output_unmap_ns));
            std::fflush(stats_csv_file_);
        }
    }

    uint16_t state_port_{49000};
    uint16_t control_port_{49001};
    int rx_fd_{-1};
    int tx_fd_{-1};
    rclcpp::TimerBase::SharedPtr recv_timer_;
    rclcpp::TimerBase::SharedPtr stats_timer_;

    MpcFpgaInterface fpga_;

    bool have_peer_{false};
    sockaddr_in last_peer_addr_{};

    int32_t last_good_steering_fp_{0};
    int32_t last_good_speed_fp_{float_to_fp(static_cast<float>(MPC_FPGA_MIN_VEL_MPS))};

    uint64_t stats_idx_{0};
    uint64_t stats_window_count_{0};
    uint64_t stats_valid_solve_count_{0};
    double stats_solve_sum_us_{0.0};
    double stats_solve_min_us_{std::numeric_limits<double>::infinity()};
    double stats_solve_max_us_{0.0};
    double stats_iter_sum_{0.0};
    uint32_t stats_iter_min_{0xFFFFFFFFu};
    uint32_t stats_iter_max_{0u};
    double stats_total_call_sum_us_{0.0};
    double stats_kernel_sum_us_{0.0};
    double stats_input_migrate_sum_us_{0.0};
    double stats_output_migrate_sum_us_{0.0};
    double stats_host_prep_sum_us_{0.0};
    double stats_host_readback_sum_us_{0.0};
    double stats_overhead_sum_us_{0.0};
    uint64_t stats_profile_count_{0};
    uint64_t stats_optimal_count_{0};
    uint64_t stats_max_iter_count_{0};
    FILE* stats_csv_file_{nullptr};
};

}  // namespace state_transport_udp

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<state_transport_udp::KriaUdpReceiverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
