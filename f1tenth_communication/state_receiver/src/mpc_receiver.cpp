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
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <vitis_common/common/ros_opencl_120.hpp>
#include <vitis_common/common/utilities.hpp>

#include "mpc_fpga_interface.h"

namespace f1tenth_communication {

/*===========================================================================
 * Fixed-Point Helpers (Q16.16)
 *===========================================================================*/

inline float fp_to_float(int32_t fp) {
    return static_cast<float>(fp) / static_cast<float>(MPC_FPGA_Q16_SCALE_I32);
}

inline int32_t float_to_fp(float f) {
    return static_cast<int32_t>(f * static_cast<float>(MPC_FPGA_Q16_SCALE_I32));
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

        unsigned int file_buf_size = 0;
        std::unique_ptr<char[]> file_buf(read_binary_file(xclbin_path, file_buf_size));
        if (!file_buf || file_buf_size == 0) {
            std::fprintf(stderr, "MPC-FPGA-OpenCL: failed to read xclbin: %s\n", xclbin_path.c_str());
            return false;
        }

        cl::Program::Binaries bins{{file_buf.get(), file_buf_size}};
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

            input_buffer_ = cl::Buffer(context_, CL_MEM_READ_ONLY, INPUT_BUFFER_BYTES_512, nullptr, &err);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "MPC-FPGA-OpenCL: input buffer allocation failed (%d)\n", err);
            return false;
        }

        output_buffer_ = cl::Buffer(context_, CL_MEM_WRITE_ONLY, sizeof(int32_t) * 4, nullptr, &err);
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

        // Group 0: [e_y | e_psi | vx | vy]
        input_words_[0] = e_y_fp;
        input_words_[1] = e_psi_fp;
        input_words_[2] = vx_fp;
        input_words_[3] = vy_fp;

        // Group 1: [omega | steering | horizon_length | reserved]
        input_words_[4] = omega_fp;
        input_words_[5] = steering_fp;
        input_words_[6] = static_cast<int32_t>(horizon);
        input_words_[7] = 0;

        // Groups 2..N+1: [ref_vx | ref_kappa | ref_left | ref_right]
        for (size_t i = 0; i < MPC_HORIZON; ++i) {
            const size_t base = 8 + (i * 4);
            if (i < horizon) {
                input_words_[base + 0] = msg.ref_vx_fp[i];
                input_words_[base + 1] = msg.ref_kappa_fp[i];
                input_words_[base + 2] = msg.ref_left_bound_fp[i];
                input_words_[base + 3] = msg.ref_right_bound_fp[i];
            }
        }

        const auto t0 = std::chrono::high_resolution_clock::now();
        cl_int err = CL_SUCCESS;
        err = queue_.enqueueWriteBuffer(input_buffer_, CL_FALSE, 0,
                                            INPUT_BUFFER_BYTES_512, input_words_.data());
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "MPC-FPGA-OpenCL: enqueueWriteBuffer failed (%d)\n", err);
            last_compute_ns_ = -1;
            return false;
        }

        err = queue_.enqueueTask(kernel_);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "MPC-FPGA-OpenCL: enqueueTask failed (%d)\n", err);
            last_compute_ns_ = -1;
            return false;
        }

        err = queue_.enqueueReadBuffer(output_buffer_, CL_FALSE, 0,
                                       sizeof(int32_t) * 4, output_words_.data());
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "MPC-FPGA-OpenCL: enqueueReadBuffer failed (%d)\n", err);
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
    std::vector<int32_t> input_words_{DMA_BUFFER_WORDS, 0};
    std::array<int32_t, 4> output_words_{{0, 0, 0, 0}};

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
    }

private:
    float max_steering_ = MPC_FPGA_MAX_STEER_RAD;
    float max_velocity_ = MPC_FPGA_MAX_VEL_MPS;

    MpcFpgaInterface fpga_;
    rclcpp::Subscription<f1tenth_msgs::msg::MpcState>::SharedPtr sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    uint64_t msg_count_ = 0;
    uint64_t latency_count_ = 0;
    uint64_t rx_count_ = 0;
    uint64_t pub_count_ = 0;
    uint64_t drop_no_horizon_ = 0;
    uint64_t drop_bad_arrays_ = 0;
    uint64_t drop_compute_ = 0;
    double total_latency_ms_ = 0.0;
    double total_loop_us_ = 0.0;
    double min_loop_us_ = std::numeric_limits<double>::infinity();
    double max_loop_us_ = 0.0;
    std::chrono::steady_clock::time_point last_msg_time_ = std::chrono::steady_clock::now();
    float latest_vx_mps_ = 0.0f;

    struct FrenetErrorsFp {
        int32_t e_y_fp;
        int32_t e_psi_fp;
    };

    bool has_required_horizon_data(const f1tenth_msgs::msg::MpcState::SharedPtr& msg) const {
        return msg->horizon_length > 0 && !msg->ref_vx_fp.empty();
    }

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

    float compute_target_speed(float accel) const {
        const float v_target = latest_vx_mps_ + accel * MPC_FPGA_PREDICTION_DT_S;
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

        double latency_ms = -1.0;
        const rclcpp::Time msg_time(msg->header.stamp);
        if (msg_time.nanoseconds() > 0) {
            latency_ms = (this->now() - msg_time).seconds() * 1000.0;
        }

        if (latency_ms >= 0.0) {
            total_latency_ms_ += latency_ms;
            latency_count_++;
        }

        if (msg_count_ % MPC_FPGA_RECEIVER_LOG_PERIOD_MSGS == 0) {
            const double avg = (latency_count_ > 0)
                ? (total_latency_ms_ / static_cast<double>(latency_count_))
                : -1.0;
            const double avg_loop_us = total_loop_us_ / static_cast<double>(msg_count_);
            const int64_t fpga_ns = fpga_.get_last_compute_ns();
            if (latency_ms >= 0.0 && avg >= 0.0) {
                RCLCPP_INFO(get_logger(),
                    "[FPGA-OpenCL] ey=%.3f m epsi=%.1f deg delta=%.1f deg v=%.1f a=%.1f | "
                    "Status=%u Iter=%u | Total=%ld us FPGA=%ld ns | "
                    "Loop us avg/min/max=%.1f/%.1f/%.1f | Lat %.1f ms (avg %.1f) | "
                    "rx=%lu pub=%lu drop(h=%lu a=%lu c=%lu)",
                    e_y_m,
                    e_psi_rad * MPC_FPGA_RAD_TO_DEG,
                    steering * MPC_FPGA_RAD_TO_DEG, speed, accel,
                    status, iters,
                    compute_us, fpga_ns,
                    avg_loop_us, min_loop_us_, max_loop_us_,
                    latency_ms, avg,
                    rx_count_, pub_count_, drop_no_horizon_, drop_bad_arrays_, drop_compute_);
            } else {
                RCLCPP_INFO(get_logger(),
                    "[FPGA-OpenCL] ey=%.3f m epsi=%.1f deg delta=%.1f deg v=%.1f a=%.1f | "
                    "Status=%u Iter=%u | Total=%ld us FPGA=%ld ns | "
                    "Loop us avg/min/max=%.1f/%.1f/%.1f | Lat N/A | "
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
        if (msg->ref_vx_fp.size() < horizon ||
            msg->ref_kappa_fp.size() < horizon ||
            msg->ref_left_bound_fp.size() < horizon ||
            msg->ref_right_bound_fp.size() < horizon) {
            drop_bad_arrays_++;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "Horizon arrays too small for horizon_length=%zu, got: vx=%zu kappa=%zu left=%zu right=%zu",
                horizon, msg->ref_vx_fp.size(), msg->ref_kappa_fp.size(),
                msg->ref_left_bound_fp.size(), msg->ref_right_bound_fp.size());
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

        steering = fp_to_float(out_steer_fp);
        accel = fp_to_float(out_accel_fp);

        speed = compute_target_speed(accel);
        steering = std::clamp(steering, -max_steering_, max_steering_);
        speed = std::clamp(speed, 0.0f, max_velocity_);

        publish_drive_command(steering, speed, accel);
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
