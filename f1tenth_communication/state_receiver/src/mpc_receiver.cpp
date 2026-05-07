/**
 * @file mpc_receiver.cpp
 * @brief Receive streamed MPC state and execute FPGA solve via OpenCL.
 * @details Runs on Kria and bridges ROS `MpcState` input to FPGA MPC
 *          compute, then publishes `AckermannDriveStamped` control commands.
 *          Data path uses OpenCL/XRT-managed buffers and kernel launches.
 */

#include <rclcpp/rclcpp.hpp>
#include <f1tenth_msgs/msg/mpc_state.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

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
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <sys/stat.h>

#include <vitis_common/common/ros_opencl_120.hpp>
#include <vitis_common/common/utilities.hpp>

#include "mpc_fpga_interface.h"

namespace f1tenth_communication {

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

/*===========================================================================
 * Fixed-Point Helpers (Q16.16)
 *===========================================================================*/

inline float fp_to_float(int32_t fp) {
    return static_cast<float>(fp) / static_cast<float>(MPC_FPGA_Q16_SCALE_I32);
}

inline int32_t float_to_fp(float f) {
    return static_cast<int32_t>(f >= 0.0f ? 
        f * static_cast<float>(MPC_FPGA_Q16_SCALE_I32) + 0.5f : 
        f * static_cast<float>(MPC_FPGA_Q16_SCALE_I32) - 0.5f);
}

/*===========================================================================
 * OpenCL MPC Interface
 *===========================================================================*/

class MpcFpgaInterface {
public:
    MpcFpgaInterface() = default;

    bool initialize(const std::string& xclbin_path,
                    const std::string& kernel_name,
                    int device_index) {
        cl_int err = CL_SUCCESS;
        std::vector<cl::Device> devices = get_xilinx_devices();
        if (devices.empty()) {
            std::fprintf(stderr, "MPC-FPGA-OpenCL: no Xilinx OpenCL devices found\n");
            return false;
        }

        if (device_index < 0 || static_cast<size_t>(device_index) >= devices.size()) {
            std::fprintf(stderr,
                         "MPC-FPGA-OpenCL: device_index=%d out of range (devices=%zu)\n",
                         device_index, devices.size());
            return false;
        }

        device_ = devices[static_cast<size_t>(device_index)];
        context_ = cl::Context(device_, nullptr, nullptr, nullptr, &err);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "MPC-FPGA-OpenCL: context creation failed (%d)\n", err);
            return false;
        }

        queue_ = cl::CommandQueue(context_, device_, CL_QUEUE_PROFILING_ENABLE, &err);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "MPC-FPGA-OpenCL: command queue creation failed (%d)\n", err);
            return false;
        }

        std::vector<unsigned char> file_buf;
        try {
            file_buf = read_file_bytes(xclbin_path);
        } catch (const std::exception& e) {
            std::fprintf(stderr, "MPC-FPGA-OpenCL: xclbin read failed: %s (%s)\n",
                         xclbin_path.c_str(), e.what());
            return false;
        }

        cl::Program::Binaries bins{{file_buf.data(), file_buf.size()}};
        std::vector<cl::Device> program_devices{device_};
        program_ = cl::Program(context_, program_devices, bins, nullptr, &err);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "MPC-FPGA-OpenCL: program load failed (%d)\n", err);
            return false;
        }

        kernel_ = cl::Kernel(program_, kernel_name.c_str(), &err);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr,
                         "MPC-FPGA-OpenCL: kernel '%s' creation failed (%d)\n",
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
            std::fprintf(stderr, "MPC-FPGA-OpenCL: input buffer allocation failed (%d)\n", err);
            return false;
        }

        output_buffer_ = cl::Buffer(
            context_,
            static_cast<cl_mem_flags>(CL_MEM_WRITE_ONLY | CL_MEM_ALLOC_HOST_PTR),
            sizeof(int32_t) * 4,
            nullptr,
            &err);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "MPC-FPGA-OpenCL: output buffer allocation failed (%d)\n", err);
            return false;
        }

        err = kernel_.setArg(0, input_buffer_);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "MPC-FPGA-OpenCL: setArg(0) failed (%d)\n", err);
            return false;
        }
        err = kernel_.setArg(1, output_buffer_);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "MPC-FPGA-OpenCL: setArg(1) failed (%d)\n", err);
            return false;
        }

        initialized_ = true;
        return true;
    }

    bool is_ready() const { return initialized_; }

    void set_prev_accel_fp(int32_t prev_accel_fp) { prev_accel_fp_ = prev_accel_fp; }

    bool compute(int32_t e_y_fp, int32_t e_psi_fp,
                 int32_t vx_fp, int32_t vy_fp, int32_t omega_fp,
                 int32_t steering_fp,
                 const f1tenth_msgs::msg::MpcState& msg,
                 int32_t& out_steering_fp, int32_t& out_accel_fp,
                 uint32_t& out_status, uint32_t& out_iterations) {
        if (!initialized_) return false;

        const size_t horizon = std::min(static_cast<size_t>(msg.horizon_length),
                                        static_cast<size_t>(MPC_HORIZON));
        if (horizon == 0) return false;

        std::fill(input_words_.begin(), input_words_.end(), 0);

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
            std::fprintf(stderr, "MPC-FPGA-OpenCL: enqueueMapBuffer(input) failed (%d)\n", err);
            last_compute_ns_ = -1;
            return false;
        }
        auto* input_words = reinterpret_cast<int32_t*>(mapped);
        std::fill(input_words, input_words + DMA_BUFFER_WORDS, 0);

        // Group 0: [e_y | e_psi | vx | vy]
        input_words[0] = e_y_fp;
        input_words[1] = e_psi_fp;
        input_words[2] = vx_fp;
        input_words[3] = vy_fp;

        // Group 1: [omega | steering | horizon_length | prev_accel]
        input_words[4] = omega_fp;
        input_words[5] = steering_fp;
        input_words[6] = static_cast<int32_t>(horizon);
        input_words[7] = prev_accel_fp_;

        // Groups 2..N+1(+): 8 words per step V2
        // [ref_ey | ref_epsi | ref_vx | ref_vy | ref_omega_ref | ref_kappa | ref_left | ref_right]
        for (size_t i = 0; i < MPC_HORIZON; ++i) {
            const size_t base = 8 + (i * 8);
            if (i < horizon) {
                input_words[base + 0] = msg.ref_ey_fp[i];
                input_words[base + 1] = msg.ref_epsi_fp[i];
                input_words[base + 2] = msg.ref_vx_fp[i];
                input_words[base + 3] = msg.ref_vy_fp[i];
                input_words[base + 4] = msg.ref_omega_ref_fp[i];
                input_words[base + 5] = msg.ref_kappa_fp[i];
                input_words[base + 6] = msg.ref_left_bound_fp[i];
                input_words[base + 7] = msg.ref_right_bound_fp[i];
            }
        }

        err = queue_.enqueueUnmapMemObject(input_buffer_, mapped);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "MPC-FPGA-OpenCL: enqueueUnmapMemObject(input) failed (%d)\n", err);
            last_compute_ns_ = -1;
            return false;
        }

        // Embedded XRT: prefer explicit migration before/after kernel.
        err = queue_.enqueueMigrateMemObjects({input_buffer_}, 0);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "MPC-FPGA-OpenCL: migrate(input) failed (%d)\n", err);
            last_compute_ns_ = -1;
            return false;
        }

        const auto t0 = std::chrono::high_resolution_clock::now();
        err = queue_.enqueueTask(kernel_);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "MPC-FPGA-OpenCL: enqueueTask failed (%d)\n", err);
            last_compute_ns_ = -1;
            return false;
        }

        err = queue_.enqueueMigrateMemObjects({output_buffer_}, CL_MIGRATE_MEM_OBJECT_HOST);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "MPC-FPGA-OpenCL: migrate(output) failed (%d)\n", err);
            last_compute_ns_ = -1;
            return false;
        }

        err = queue_.finish();
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "MPC-FPGA-OpenCL: queue finish failed (%d)\n", err);
            last_compute_ns_ = -1;
            return false;
        }

        const auto t1 = std::chrono::high_resolution_clock::now();
        last_compute_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();

        void* out_mapped = queue_.enqueueMapBuffer(
            output_buffer_,
            CL_TRUE,
            CL_MAP_READ,
            0,
            sizeof(int32_t) * 4,
            nullptr,
            nullptr,
            &err);
        if (err != CL_SUCCESS || out_mapped == nullptr) {
            std::fprintf(stderr, "MPC-FPGA-OpenCL: enqueueMapBuffer(output) failed (%d)\n", err);
            last_compute_ns_ = -1;
            return false;
        }

        const auto* out_words = reinterpret_cast<const int32_t*>(out_mapped);
        output_words_[0] = out_words[0];
        output_words_[1] = out_words[1];
        output_words_[2] = out_words[2];
        output_words_[3] = out_words[3];

        err = queue_.enqueueUnmapMemObject(output_buffer_, out_mapped);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "MPC-FPGA-OpenCL: enqueueUnmapMemObject(output) failed (%d)\n", err);
            last_compute_ns_ = -1;
            return false;
        }

        out_steering_fp = output_words_[0];
        out_accel_fp = output_words_[1];
        out_status = static_cast<uint32_t>(output_words_[2]);
        out_iterations = static_cast<uint32_t>(output_words_[3]);
        return true;
    }

    int64_t get_last_compute_ns() const { return last_compute_ns_; }

private:
    bool initialized_ = false;

    cl::Device device_;
    cl::Context context_;
    cl::Program program_;
    cl::Kernel kernel_;
    cl::CommandQueue queue_;
    cl::Buffer input_buffer_;
    cl::Buffer output_buffer_;

    static constexpr size_t DMA_BUFFER_WORDS = INPUT_BUFFER_WORDS_32_PAD;
    static_assert(DMA_BUFFER_WORDS * sizeof(int32_t) == INPUT_BUFFER_BYTES_512,
                  "Host DMA buffer words must match OpenCL input buffer bytes");
    std::vector<int32_t> input_words_{DMA_BUFFER_WORDS, 0};
    std::array<int32_t, 4> output_words_{{0, 0, 0, 0}};
    int32_t prev_accel_fp_{0};

    int64_t last_compute_ns_ = 0;
};

/*===========================================================================
 * MPC Receiver Node
 *===========================================================================*/

class MpcReceiverFpgaNode : public rclcpp::Node {
public:
    MpcReceiverFpgaNode() : Node("mpc_receiver") {
        // Parameters
        declare_parameter("input_topic", "/mpc_state");
        declare_parameter("drive_topic", "/drive");
        declare_parameter("xclbin_path", std::string("/lib/firmware/mpc_fpga_top_opencl.xclbin"));
        declare_parameter("kernel_name", std::string("mpc_fpga_top_opencl"));
        declare_parameter("device_index", 0);

        const auto input_topic = get_parameter("input_topic").as_string();
        const auto drive_topic = get_parameter("drive_topic").as_string();
        const auto xclbin_path = get_parameter("xclbin_path").as_string();
        const auto kernel_name = get_parameter("kernel_name").as_string();
        const int device_index = get_parameter("device_index").as_int();

        if (!fpga_.initialize(xclbin_path, kernel_name, device_index)) {
            throw std::runtime_error("MPC FPGA OpenCL init failed");
        }

        RCLCPP_INFO(get_logger(),
                    "MPC FPGA OpenCL init OK - xclbin=%s kernel=%s device_index=%d",
                    xclbin_path.c_str(), kernel_name.c_str(), device_index);

        drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic, rclcpp::SystemDefaultsQoS());

        auto qos = rclcpp::QoS(1).best_effort().durability_volatile();
        sub_ = create_subscription<f1tenth_msgs::msg::MpcState>(
            input_topic, qos,
            std::bind(&MpcReceiverFpgaNode::state_callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
                    "MPC Receiver [FPGA OpenCL] ready. %s -> %s",
                    input_topic.c_str(), drive_topic.c_str());

        /* Initialize stats CSV file */
        open_stats_csv_file();

        /* Initialize terminal output timer */
        last_terminal_print_time_ = std::chrono::steady_clock::now();
    }

private:
    static constexpr float kRacelineSpeedMarginMps = 0.5f;
    static constexpr double TERMINAL_STATS_PRINT_INTERVAL_SECONDS = 5.0;

    float max_steering_ = MPC_FPGA_MAX_STEER_RAD;
    float max_velocity_ = MPC_FPGA_MAX_VEL_MPS;

    MpcFpgaInterface fpga_;
    rclcpp::Subscription<f1tenth_msgs::msg::MpcState>::SharedPtr sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    uint64_t msg_count_ = 0;
    uint64_t rx_count_ = 0;
    uint64_t pub_count_ = 0;
    uint64_t drop_no_horizon_ = 0;
    uint64_t drop_bad_arrays_ = 0;
    uint64_t drop_compute_ = 0;
    double total_loop_us_ = 0.0;
    double min_loop_us_ = std::numeric_limits<double>::infinity();
    double max_loop_us_ = 0.0;
    std::chrono::steady_clock::time_point last_msg_time_ = std::chrono::steady_clock::now();
    float latest_vx_mps_ = 0.0f;
    float last_speed_cmd_mps_ = MPC_FPGA_MIN_VEL_MPS;
    float last_steering_cmd_rad_ = 0.0f;
    float last_accel_cmd_mps2_ = 0.0f;

    /* Stats tracking for terminal output and CSV logging */
    double stats_solve_time_sum_us_ = 0.0;
    double stats_solve_time_min_us_ = 1e9;
    double stats_solve_time_max_us_ = 0.0;
    uint32_t stats_iter_count_min_ = 0xFFFFFFFFU;
    uint32_t stats_iter_count_max_ = 0;
    double stats_iter_count_sum_ = 0.0;
    uint64_t stats_cycle_count_ = 0;
    uint64_t stats_optimal_count_ = 0;
    uint64_t stats_max_iter_count_ = 0;
    std::chrono::steady_clock::time_point last_terminal_print_time_;
    FILE* stats_csv_file_ = nullptr;

    struct FrenetErrorsFp {
        int32_t e_y_fp;
        int32_t e_psi_fp;
    };

    void open_stats_csv_file() {
        const char* stats_dir = "log";
        mkdir(stats_dir, 0755);
        
        time_t now = time(nullptr);
        char timestamp[64];
        struct tm* tm_now = localtime(&now);
        strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", tm_now);
        
        char csv_path[512];
        snprintf(csv_path, sizeof(csv_path), "%s/mpc_fpga_receiver_%s.stats.csv", stats_dir, timestamp);
        
        stats_csv_file_ = fopen(csv_path, "w");
        if (stats_csv_file_ != nullptr) {
            fprintf(stats_csv_file_, "idx,iterations,solve_time_us\n");
            fflush(stats_csv_file_);
            RCLCPP_INFO(get_logger(), "Stats CSV log: %s", csv_path);
        } else {
            RCLCPP_WARN(get_logger(), "Failed to open stats CSV file: %s", csv_path);
        }
    }

    void update_stats(int64_t compute_ns, uint32_t iterations, uint32_t status) {
        const double compute_us = static_cast<double>(compute_ns) / 1000.0;
        
        /* Track solve time statistics */
        stats_solve_time_sum_us_ += compute_us;
        if (compute_us < stats_solve_time_min_us_) stats_solve_time_min_us_ = compute_us;
        if (compute_us > stats_solve_time_max_us_) stats_solve_time_max_us_ = compute_us;
        
        /* Track iteration statistics */
        stats_iter_count_sum_ += static_cast<double>(iterations);
        if (iterations < stats_iter_count_min_) stats_iter_count_min_ = iterations;
        if (iterations > stats_iter_count_max_) stats_iter_count_max_ = iterations;

        /* Count optimal solutions and max-iter cases */
        if (status == MPC_FPGA_STATUS_OK) stats_optimal_count_++;
        if (status == MPC_FPGA_STATUS_MAX_ITER) stats_max_iter_count_++;
        
        stats_cycle_count_++;
        
        /* Write to stats CSV file */
        if (stats_csv_file_ != nullptr) {
            fprintf(stats_csv_file_, "%lu,%u,%.1f\n", stats_cycle_count_, iterations, compute_us);
            fflush(stats_csv_file_);
        }
        
        /* Print terminal stats every 5 seconds */
        auto now = std::chrono::steady_clock::now();
        double elapsed_sec = std::chrono::duration<double>(now - last_terminal_print_time_).count();
        
        if (elapsed_sec >= TERMINAL_STATS_PRINT_INTERVAL_SECONDS && stats_cycle_count_ > 0) {
            double avg_iter = stats_iter_count_sum_ / static_cast<double>(stats_cycle_count_);
            double avg_time = stats_solve_time_sum_us_ / static_cast<double>(stats_cycle_count_);
            double optimal_pct = (stats_optimal_count_ * 100.0) / static_cast<double>(stats_cycle_count_);
            double max_iter_pct = (stats_max_iter_count_ * 100.0) / static_cast<double>(stats_cycle_count_);
            
            RCLCPP_INFO(get_logger(),
                "[FPGA-Stats] (last %.1fs, %lu calls):\n"
                "  Iterations: min=%u, avg=%.1f, max=%u\n"
                "  Solve time: min=%.1f us, avg=%.1f us, max=%.1f us\n"
                "  Optimal: %.1f%%, Max iter: %.1f%%",
                elapsed_sec, stats_cycle_count_,
                stats_iter_count_min_, avg_iter, stats_iter_count_max_,
                stats_solve_time_min_us_, avg_time, stats_solve_time_max_us_,
                optimal_pct, max_iter_pct);
            
            /* Reset stats */
            stats_solve_time_sum_us_ = 0.0;
            stats_solve_time_min_us_ = 1e9;
            stats_solve_time_max_us_ = 0.0;
            stats_iter_count_min_ = 0xFFFFFFFFU;
            stats_iter_count_max_ = 0;
            stats_iter_count_sum_ = 0.0;
            stats_cycle_count_ = 0;
            stats_optimal_count_ = 0;
            stats_max_iter_count_ = 0;
            last_terminal_print_time_ = now;
        }
    }

    bool has_required_horizon_data(const f1tenth_msgs::msg::MpcState::SharedPtr& msg) const {
        return msg->horizon_length > 0 &&
               !msg->ref_x_fp.empty() && !msg->ref_y_fp.empty() && !msg->ref_psi_fp.empty() &&
               !msg->ref_vx_fp.empty() && !msg->ref_kappa_fp.empty() &&
               !msg->ref_left_bound_fp.empty() && !msg->ref_right_bound_fp.empty();
    }

    static FrenetErrorsFp compute_frenet_errors(const f1tenth_msgs::msg::MpcState::SharedPtr& msg) {
        const float x = fp_to_float(msg->x_fp);
        const float y = fp_to_float(msg->y_fp);
        const float theta = fp_to_float(msg->theta_fp);

        const size_t horizon = static_cast<size_t>(std::max<uint32_t>(1u, msg->horizon_length));
        const size_t max_search = std::min(horizon > 0 ? horizon - 1 : 0, static_cast<size_t>(16));

        float best_e_y = 0.0f;
        float best_e_psi = 0.0f;
        float best_dist2 = 1e18f;

        for (size_t i = 0; i < max_search; ++i) {
            const float ax = fp_to_float(msg->ref_x_fp[i]);
            const float ay = fp_to_float(msg->ref_y_fp[i]);
            const float bx = fp_to_float(msg->ref_x_fp[i + 1]);
            const float by = fp_to_float(msg->ref_y_fp[i + 1]);
            const float h0 = fp_to_float(msg->ref_psi_fp[i]);
            const float h1 = fp_to_float(msg->ref_psi_fp[i + 1]);

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

        return FrenetErrorsFp{float_to_fp(best_e_y), float_to_fp(best_e_psi)};
    }

    float compute_target_speed(float accel, const f1tenth_msgs::msg::MpcState::SharedPtr& msg) const {
        const float speed_cmd_from_accel = latest_vx_mps_ + accel * MPC_FPGA_PREDICTION_DT_S;
        float speed = std::clamp(speed_cmd_from_accel, MPC_FPGA_MIN_VEL_MPS, max_velocity_);
        if (!msg->ref_vx_fp.empty()) {
            const float v_ref0 = fp_to_float(msg->ref_vx_fp[0]);
            if (std::isfinite(v_ref0) && v_ref0 > 0.0f) {
                speed = std::min(speed, v_ref0 + kRacelineSpeedMarginMps);
            }
        }
        return std::clamp(speed, MPC_FPGA_MIN_VEL_MPS, max_velocity_);
    }

    void publish_drive_command(float steering,
                               float speed,
                               float accel,
                               const builtin_interfaces::msg::Time& source_stamp) {
        auto drive = ackermann_msgs::msg::AckermannDriveStamped();
        /* Preserve Jetson source timestamp for end-to-end latency measurement. */
        drive.header.stamp = source_stamp;
        drive.header.frame_id = "base_link";
        drive.drive.steering_angle = steering;
        drive.drive.speed = speed;
        drive.drive.acceleration = accel;
        drive_pub_->publish(drive);
    }

    void update_timing_and_log(const f1tenth_msgs::msg::MpcState::SharedPtr& msg,
                               const std::chrono::high_resolution_clock::time_point& t_start,
                               const std::chrono::high_resolution_clock::time_point& t_end,
                               float e_y_m,
                               float e_psi_rad,
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

        if (msg_count_ % MPC_FPGA_RECEIVER_LOG_PERIOD_MSGS == 0) {
            const double avg_loop_us = total_loop_us_ / static_cast<double>(msg_count_);
            const int64_t fpga_ns = fpga_.get_last_compute_ns();
            RCLCPP_INFO(get_logger(),
                "[FPGA-OpenCL] ey=%.3f m epsi=%.1f deg delta=%.1f deg v=%.1f a=%.1f | "
                "Status=%u Iter=%u | Total=%ld us FPGA=%ld ns | "
                "Loop us avg/min/max=%.1f/%.1f/%.1f | "
                "rx=%lu pub=%lu drop(h=%lu a=%lu c=%lu)",
                e_y_m,
                e_psi_rad * MPC_FPGA_RAD_TO_DEG,
                steering * MPC_FPGA_RAD_TO_DEG, speed, accel,
                status, iters,
                compute_us, fpga_ns,
                avg_loop_us, min_loop_us_, max_loop_us_,
                rx_count_, pub_count_, drop_no_horizon_, drop_bad_arrays_, drop_compute_);
        }
    }

    void state_callback(const f1tenth_msgs::msg::MpcState::SharedPtr msg) {
        auto t_start = std::chrono::high_resolution_clock::now();
        last_msg_time_ = std::chrono::steady_clock::now();
        rx_count_++;

        float steering = 0.0f;
        float speed = 0.0f;
        float accel = 0.0f;
        uint32_t status = 0;
        uint32_t iters = 0;

        int32_t out_steer_fp = 0;
        int32_t out_accel_fp = 0;

        if (!has_required_horizon_data(msg)) {
            drop_no_horizon_++;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "No streamed waypoint data in message");
            return;
        }

        const size_t horizon = std::min(static_cast<size_t>(msg->horizon_length),
                                        static_cast<size_t>(MPC_HORIZON));
        if (msg->ref_ey_fp.size() < horizon ||
            msg->ref_epsi_fp.size() < horizon ||
            msg->ref_x_fp.size() < horizon ||
            msg->ref_y_fp.size() < horizon ||
            msg->ref_psi_fp.size() < horizon ||
            msg->ref_vx_fp.size() < horizon ||
            msg->ref_vy_fp.size() < horizon ||
            msg->ref_omega_ref_fp.size() < horizon ||
            msg->ref_kappa_fp.size() < horizon ||
            msg->ref_left_bound_fp.size() < horizon ||
            msg->ref_right_bound_fp.size() < horizon) {
            drop_bad_arrays_++;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "Horizon arrays too small for horizon_length=%zu, got: ey=%zu epsi=%zu x=%zu y=%zu psi=%zu vx=%zu vy=%zu omega_ref=%zu kappa=%zu left=%zu right=%zu",
                horizon,
                msg->ref_ey_fp.size(),
                msg->ref_epsi_fp.size(),
                msg->ref_x_fp.size(),
                msg->ref_y_fp.size(),
                msg->ref_psi_fp.size(),
                msg->ref_vx_fp.size(),
                msg->ref_vy_fp.size(),
                msg->ref_omega_ref_fp.size(),
                msg->ref_kappa_fp.size(),
                msg->ref_left_bound_fp.size(),
                msg->ref_right_bound_fp.size());
            return;
        }

        latest_vx_mps_ = fp_to_float(msg->velocity_fp);
        const FrenetErrorsFp errors = compute_frenet_errors(msg);
        const float e_y_m = fp_to_float(errors.e_y_fp);
        const float e_psi_rad = fp_to_float(errors.e_psi_fp);

        const bool ok = fpga_.compute(
            errors.e_y_fp, errors.e_psi_fp,
            msg->velocity_fp, msg->vy_fp, msg->omega_fp,
            msg->steering_angle_fp,
            *msg,
            out_steer_fp, out_accel_fp, status, iters);

        if (!ok) {
            drop_compute_++;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "FPGA OpenCL compute failed");
            return;
        }

        /* Update statistics */
        update_stats(fpga_.get_last_compute_ns(), iters, status);

        steering = fp_to_float(out_steer_fp);
        accel = fp_to_float(out_accel_fp);

        if (status == MPC_FPGA_STATUS_ERROR || status == MPC_FPGA_STATUS_NO_TRAJECTORY) {
            steering = last_steering_cmd_rad_;
            speed = last_speed_cmd_mps_;
            accel = last_accel_cmd_mps2_;
            fpga_.set_prev_accel_fp(float_to_fp(accel));
        } else {
            speed = compute_target_speed(accel, msg);
            steering = std::clamp(steering, -max_steering_, max_steering_);
            speed = std::clamp(speed, MPC_FPGA_MIN_VEL_MPS, max_velocity_);
            last_steering_cmd_rad_ = steering;
            last_speed_cmd_mps_ = speed;
            last_accel_cmd_mps2_ = accel;
            fpga_.set_prev_accel_fp(float_to_fp(accel));
        }

        publish_drive_command(steering, speed, accel, msg->header.stamp);
        pub_count_++;
        if (pub_count_ == 1) {
            RCLCPP_INFO(get_logger(),
                "First /drive published after rx=%lu messages (drops: horizon=%lu arrays=%lu compute=%lu)",
                rx_count_, drop_no_horizon_, drop_bad_arrays_, drop_compute_);
        }

        auto t_end = std::chrono::high_resolution_clock::now();
        update_timing_and_log(msg, t_start, t_end, e_y_m, e_psi_rad,
                              steering, speed, accel, status, iters);
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
