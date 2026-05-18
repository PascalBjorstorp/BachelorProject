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
#include <cinttypes>
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
 * Fixed-Point Helpers (raw QP transport)
 *===========================================================================*/

inline float fp_to_float(int32_t fp) {
    return static_cast<float>(fp) / static_cast<float>(MPC_FPGA_QP_SCALE_I32);
}

inline int32_t float_to_fp(float f) {
    return static_cast<int32_t>(f >= 0.0f ? 
        f * static_cast<float>(MPC_FPGA_QP_SCALE_I32) + 0.5f :
        f * static_cast<float>(MPC_FPGA_QP_SCALE_I32) - 0.5f);
}

/*===========================================================================
 * OpenCL MPC Interface
 *===========================================================================*/

class MpcFpgaInterface {
public:
    struct TimingProfile {
        /* Host wall-clock (chrono) segments of one compute() call. */
        int64_t total_call_ns = 0;
        int64_t input_pack_ns = 0;             // pack directly into mapped DMA buffer
        int64_t input_migrate_enqueue_ns = 0;  // host time inside enqueueMigrate(in)
        int64_t kernel_enqueue_ns = 0;         // host time inside enqueueTask
        int64_t output_migrate_enqueue_ns = 0; // host time inside enqueueMigrate(out)
        int64_t wait_ns = 0;                   // host time inside waitForEvents
        int64_t output_unpack_ns = 0;          // host time reading mapped output
        /* Device-side OpenCL profiling (COMMAND_END - COMMAND_START). */
        int64_t input_migrate_ns = -1;
        int64_t kernel_ns = -1;
        int64_t output_migrate_ns = -1;
        /* Device-side scheduling latency (from absolute event timestamps). */
        int64_t input_migrate_submit_ns = -1;  // SUBMIT - QUEUED
        int64_t input_migrate_sched_ns = -1;   // START  - SUBMIT
        int64_t gap_in_to_kernel_ns = -1;      // kernel START - in-migrate END
        int64_t kernel_submit_ns = -1;         // SUBMIT - QUEUED
        int64_t kernel_sched_ns = -1;          // START  - SUBMIT
        int64_t gap_kernel_to_out_ns = -1;     // out-migrate START - kernel END
        int64_t output_migrate_submit_ns = -1; // SUBMIT - QUEUED
        int64_t output_migrate_sched_ns = -1;  // START  - SUBMIT
        /* Aggregate. */
        int64_t opencl_overhead_ns = -1;       // total_call - kernel
    };

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

        /* Persistently map the host-allocated buffers. On the Kria the PS and
         * PL share DDR, so CL_MEM_ALLOC_HOST_PTR + a one-time map gives a real
         * zero-copy pointer; per-cycle sync is done with enqueueMigrate, not
         * Write/ReadBuffer. The map is never released for the node's lifetime. */
        input_ptr_ = static_cast<int32_t*>(queue_.enqueueMapBuffer(
            input_buffer_,
            CL_TRUE,
            static_cast<cl_map_flags>(CL_MAP_WRITE),
            0,
            INPUT_BUFFER_BYTES_512,
            nullptr,
            nullptr,
            &err));
        if (err != CL_SUCCESS || input_ptr_ == nullptr) {
            std::fprintf(stderr, "MPC-FPGA-OpenCL: input map failed (%d)\n", err);
            return false;
        }

        output_ptr_ = static_cast<int32_t*>(queue_.enqueueMapBuffer(
            output_buffer_,
            CL_TRUE,
            static_cast<cl_map_flags>(CL_MAP_READ),
            0,
            sizeof(int32_t) * 4,
            nullptr,
            nullptr,
            &err));
        if (err != CL_SUCCESS || output_ptr_ == nullptr) {
            std::fprintf(stderr, "MPC-FPGA-OpenCL: output map failed (%d)\n", err);
            return false;
        }

        /* Pad lane words [INPUT_BUFFER_WORDS_32 .. INPUT_BUFFER_WORDS_32_PAD)
         * exist only to round the payload up to a 512-bit beat. They are never
         * written per cycle, so zero them exactly once here instead of clearing
         * the whole buffer on every compute() call. */
        for (size_t i = INPUT_BUFFER_WORDS_32;
             i < INPUT_BUFFER_WORDS_32_PAD; ++i) {
            input_ptr_[i] = 0;
        }

        initialized_ = true;
        return true;
    }

    bool is_ready() const { return initialized_; }

    void set_prev_accel_fp(int32_t prev_accel_fp) { prev_accel_fp_ = prev_accel_fp; }
    int32_t get_prev_accel_fp() const { return prev_accel_fp_; }
    const TimingProfile& get_last_profile() const { return last_profile_; }

    bool compute(int32_t e_y_fp, int32_t e_psi_fp,
                 int32_t vx_fp, int32_t vy_fp, int32_t omega_fp,
                 int32_t steering_fp,
                 const f1tenth_msgs::msg::MpcState& msg,
                 uint32_t control_flags,
                 int32_t& out_steering_fp, int32_t& out_accel_fp,
                 uint32_t& out_status, uint32_t& out_iterations) {
        if (!initialized_) return false;

        last_profile_ = TimingProfile{};

        cl_int err = CL_SUCCESS;
        const auto total_t0 = std::chrono::high_resolution_clock::now();
        const auto input_pack_t0 = total_t0;
        /* Pack straight into the mapped DMA buffer; no staging array, no
         * full-buffer zeroing (pad words were cleared once at init). */
        int32_t* input_words = input_ptr_;

        input_words[MPC_FPGA_WORD_EY] = e_y_fp;
        input_words[MPC_FPGA_WORD_EPSI] = e_psi_fp;
        input_words[MPC_FPGA_WORD_VX] = vx_fp;
        input_words[MPC_FPGA_WORD_VY] = vy_fp;
        input_words[MPC_FPGA_WORD_OMEGA] = omega_fp;
        input_words[MPC_FPGA_WORD_STEERING] = steering_fp;
        input_words[MPC_FPGA_WORD_CONTROL_FLAGS] = static_cast<int32_t>(control_flags);
        input_words[MPC_FPGA_WORD_PREV_ACCEL] = prev_accel_fp_;

        // Groups 2..N+1(+): 8 words per step V2
        // [ref_ey | ref_epsi | ref_vx | ref_vy | ref_omega_ref | ref_kappa | ref_left | ref_right]
        for (size_t i = 0; i < MPC_HORIZON; ++i) {
            const size_t base = MPC_FPGA_HEADER_WORDS + (i * MPC_FPGA_REF_WORDS);
            input_words[base + 0] = msg.ref_ey_fp[i];
            input_words[base + 1] = msg.ref_epsi_fp[i];
            input_words[base + 2] = msg.ref_vx_fp[i];
            input_words[base + 3] = msg.ref_vy_fp[i];
            input_words[base + 4] = msg.ref_omega_ref_fp[i];
            input_words[base + 5] = msg.ref_kappa_fp[i];
            input_words[base + 6] = msg.ref_left_bound_fp[i];
            input_words[base + 7] = msg.ref_right_bound_fp[i];
        }
        const auto input_pack_t1 = std::chrono::high_resolution_clock::now();
        last_profile_.input_pack_ns = ns_between(input_pack_t0, input_pack_t1);

        cl::Event input_migrate_event;
        cl::Event kernel_event;
        cl::Event output_migrate_event;
        const std::vector<cl::Memory> input_mem{input_buffer_};
        const std::vector<cl::Memory> output_mem{output_buffer_};

        /* Sync the freshly packed input down to PL-visible DDR. */
        const auto enq_in_t0 = std::chrono::high_resolution_clock::now();
        err = queue_.enqueueMigrateMemObjects(
            input_mem, 0 /* to device */, nullptr, &input_migrate_event);
        const auto enq_in_t1 = std::chrono::high_resolution_clock::now();
        last_profile_.input_migrate_enqueue_ns = ns_between(enq_in_t0, enq_in_t1);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "MPC-FPGA-OpenCL: migrate(input) failed (%d)\n", err);
            last_compute_ns_ = -1;
            return false;
        }

        /* Device round-trip starts once the input migrate is enqueued. */
        const auto t0 = enq_in_t1;

        std::vector<cl::Event> wait_input{input_migrate_event};
        const auto enq_k_t0 = std::chrono::high_resolution_clock::now();
        err = queue_.enqueueTask(kernel_, &wait_input, &kernel_event);
        const auto enq_k_t1 = std::chrono::high_resolution_clock::now();
        last_profile_.kernel_enqueue_ns = ns_between(enq_k_t0, enq_k_t1);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "MPC-FPGA-OpenCL: enqueueTask failed (%d)\n", err);
            last_compute_ns_ = -1;
            return false;
        }

        std::vector<cl::Event> wait_kernel{kernel_event};
        const auto enq_out_t0 = std::chrono::high_resolution_clock::now();
        err = queue_.enqueueMigrateMemObjects(
            output_mem, CL_MIGRATE_MEM_OBJECT_HOST, &wait_kernel,
            &output_migrate_event);
        const auto enq_out_t1 = std::chrono::high_resolution_clock::now();
        last_profile_.output_migrate_enqueue_ns = ns_between(enq_out_t0, enq_out_t1);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "MPC-FPGA-OpenCL: migrate(output) failed (%d)\n", err);
            last_compute_ns_ = -1;
            return false;
        }

        const auto wait_t0 = std::chrono::high_resolution_clock::now();
        err = cl::Event::waitForEvents({output_migrate_event});
        const auto wait_t1 = std::chrono::high_resolution_clock::now();
        last_profile_.wait_ns = ns_between(wait_t0, wait_t1);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "MPC-FPGA-OpenCL: output wait failed (%d)\n", err);
            last_compute_ns_ = -1;
            return false;
        }

        const auto t1 = wait_t1;
        last_compute_ns_ = ns_between(t0, t1);

        const EventProfile ip = get_event_profile(input_migrate_event);
        const EventProfile kp = get_event_profile(kernel_event);
        const EventProfile op = get_event_profile(output_migrate_event);
        if (ip.valid) {
            last_profile_.input_migrate_ns = static_cast<int64_t>(ip.end - ip.start);
            last_profile_.input_migrate_submit_ns =
                static_cast<int64_t>(ip.submit - ip.queued);
            last_profile_.input_migrate_sched_ns =
                static_cast<int64_t>(ip.start - ip.submit);
        }
        if (kp.valid) {
            last_profile_.kernel_ns = static_cast<int64_t>(kp.end - kp.start);
            last_profile_.kernel_submit_ns =
                static_cast<int64_t>(kp.submit - kp.queued);
            last_profile_.kernel_sched_ns =
                static_cast<int64_t>(kp.start - kp.submit);
        }
        if (op.valid) {
            last_profile_.output_migrate_ns = static_cast<int64_t>(op.end - op.start);
            last_profile_.output_migrate_submit_ns =
                static_cast<int64_t>(op.submit - op.queued);
            last_profile_.output_migrate_sched_ns =
                static_cast<int64_t>(op.start - op.submit);
        }
        if (ip.valid && kp.valid && kp.start >= ip.end) {
            last_profile_.gap_in_to_kernel_ns =
                static_cast<int64_t>(kp.start - ip.end);
        }
        if (kp.valid && op.valid && op.start >= kp.end) {
            last_profile_.gap_kernel_to_out_ns =
                static_cast<int64_t>(op.start - kp.end);
        }

        const auto unpack_t0 = std::chrono::high_resolution_clock::now();
        out_steering_fp = output_ptr_[0];
        out_accel_fp = output_ptr_[1];
        out_status = static_cast<uint32_t>(output_ptr_[2]);
        out_iterations = static_cast<uint32_t>(output_ptr_[3]);
        const auto unpack_t1 = std::chrono::high_resolution_clock::now();
        last_profile_.output_unpack_ns = ns_between(unpack_t0, unpack_t1);

        last_profile_.total_call_ns = ns_between(total_t0, unpack_t1);
        if (last_profile_.kernel_ns >= 0) {
            last_profile_.opencl_overhead_ns =
                last_profile_.total_call_ns - last_profile_.kernel_ns;
        }
        return true;
    }

    int64_t get_last_compute_ns() const { return last_compute_ns_; }

private:
    struct EventProfile {
        cl_ulong queued = 0;
        cl_ulong submit = 0;
        cl_ulong start = 0;
        cl_ulong end = 0;
        bool valid = false;
    };

    template <typename TP>
    static int64_t ns_between(const TP& a, const TP& b) {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(b - a).count();
    }

    static EventProfile get_event_profile(const cl::Event& event) {
        EventProfile p;
        cl_int err = CL_SUCCESS;
        p.queued = event.getProfilingInfo<CL_PROFILING_COMMAND_QUEUED>(&err);
        if (err != CL_SUCCESS) return p;
        p.submit = event.getProfilingInfo<CL_PROFILING_COMMAND_SUBMIT>(&err);
        if (err != CL_SUCCESS) return p;
        p.start = event.getProfilingInfo<CL_PROFILING_COMMAND_START>(&err);
        if (err != CL_SUCCESS) return p;
        p.end = event.getProfilingInfo<CL_PROFILING_COMMAND_END>(&err);
        if (err != CL_SUCCESS) return p;
        p.valid = (p.end >= p.start) && (p.submit >= p.queued) &&
                  (p.start >= p.submit);
        return p;
    }

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
    /* Persistent zero-copy mappings of the host-allocated OpenCL buffers. */
    int32_t* input_ptr_{nullptr};
    int32_t* output_ptr_{nullptr};
    int32_t prev_accel_fp_{0};

    TimingProfile last_profile_{};
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
        declare_parameter("debug_raw_log_enabled", true);
        declare_parameter("debug_raw_log_stride", 1);

        const auto input_topic = get_parameter("input_topic").as_string();
        const auto drive_topic = get_parameter("drive_topic").as_string();
        const auto xclbin_path = get_parameter("xclbin_path").as_string();
        const auto kernel_name = get_parameter("kernel_name").as_string();
        const int device_index = get_parameter("device_index").as_int();
        raw_debug_enabled_ = get_parameter("debug_raw_log_enabled").as_bool();
        raw_debug_stride_ = static_cast<uint32_t>(
            std::max<int64_t>(1, get_parameter("debug_raw_log_stride").as_int()));

        if (!fpga_.initialize(xclbin_path, kernel_name, device_index)) {
            throw std::runtime_error("MPC FPGA OpenCL init failed");
        }

        drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic, rclcpp::SystemDefaultsQoS());

        auto qos = rclcpp::QoS(1).best_effort().durability_volatile();
        sub_ = create_subscription<f1tenth_msgs::msg::MpcState>(
            input_topic, qos,
            std::bind(&MpcReceiverFpgaNode::state_callback, this, std::placeholders::_1));

        /* Initialize stats CSV file */
        open_stats_csv_file();
        open_raw_debug_csv_file();

        /* Initialize terminal output timer */
        last_terminal_print_time_ = std::chrono::steady_clock::now();
    }

    ~MpcReceiverFpgaNode() override {
        if (stats_csv_file_ != nullptr) {
            fclose(stats_csv_file_);
            stats_csv_file_ = nullptr;
        }
        if (raw_debug_csv_file_ != nullptr) {
            fclose(raw_debug_csv_file_);
            raw_debug_csv_file_ = nullptr;
        }
    }

private:
    static constexpr float kRacelineSpeedMarginMps = 0.5f;
    static constexpr float kStandstillVxThreshMps = 0.15f;
    static constexpr float kStandstillAccelOverrideMps2 = 0.5f;
    static constexpr float kStandstillBrakeEpsMps2 = 0.05f;
    static constexpr double TERMINAL_STATS_PRINT_INTERVAL_SECONDS = 5.0;

    float max_steering_ = MPC_FPGA_MAX_STEER_RAD;
    float max_velocity_ = MPC_FPGA_MAX_VEL_MPS;

    MpcFpgaInterface fpga_;
    rclcpp::Subscription<f1tenth_msgs::msg::MpcState>::SharedPtr sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    uint64_t rx_count_ = 0;
    uint64_t pub_count_ = 0;
    uint64_t drop_no_horizon_ = 0;
    uint64_t drop_bad_arrays_ = 0;
    uint64_t drop_compute_ = 0;
    std::chrono::steady_clock::time_point last_msg_time_ = std::chrono::steady_clock::now();
    float latest_vx_mps_ = 0.0f;
    float last_speed_cmd_mps_ = MPC_FPGA_MIN_VEL_MPS;
    float last_steering_cmd_rad_ = 0.0f;
    float last_accel_cmd_mps2_ = 0.0f;
    bool fpga_reset_pending_ = true;

    /* Stats tracking for terminal output and CSV logging */
    double stats_solve_time_sum_us_ = 0.0;
    double stats_solve_time_min_us_ = 1e9;
    double stats_solve_time_max_us_ = 0.0;
    double stats_total_call_sum_us_ = 0.0;
    double stats_kernel_sum_us_ = 0.0;
    double stats_input_migrate_sum_us_ = 0.0;
    double stats_output_migrate_sum_us_ = 0.0;
    double stats_input_pack_sum_us_ = 0.0;
    double stats_enqueue_sum_us_ = 0.0;
    double stats_overhead_sum_us_ = 0.0;
    uint32_t stats_iter_count_min_ = 0xFFFFFFFFU;
    uint32_t stats_iter_count_max_ = 0;
    double stats_iter_count_sum_ = 0.0;
    uint64_t stats_cycle_count_ = 0;
    uint64_t stats_optimal_count_ = 0;
    uint64_t stats_max_iter_count_ = 0;
    uint64_t stats_error_count_ = 0;
    uint64_t stats_no_trajectory_count_ = 0;
    uint64_t stats_other_status_count_ = 0;
    uint64_t stats_max_iter_streak_ = 0;
    uint64_t stats_max_iter_streak_max_ = 0;
    std::chrono::steady_clock::time_point last_terminal_print_time_;
    FILE* stats_csv_file_ = nullptr;
    FILE* raw_debug_csv_file_ = nullptr;
    bool raw_debug_enabled_ = true;
    uint32_t raw_debug_stride_ = 1;
    uint64_t raw_debug_idx_ = 0;

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
            fprintf(stats_csv_file_,
                    "idx,iterations,status,"
                    "solve_time_us,total_call_us,opencl_overhead_us,"
                    "input_pack_us,input_migrate_enq_us,kernel_enq_us,"
                    "output_migrate_enq_us,wait_us,output_unpack_us,"
                    "input_migrate_us,kernel_us,output_migrate_us,"
                    "in_mig_submit_us,in_mig_sched_us,gap_in_to_kernel_us,"
                    "kernel_submit_us,kernel_sched_us,gap_kernel_to_out_us,"
                    "out_mig_submit_us,out_mig_sched_us\n");
            fflush(stats_csv_file_);
        } else {
            RCLCPP_WARN(get_logger(), "Failed to open stats CSV file: %s", csv_path);
        }
    }

    void open_raw_debug_csv_file() {
        if (!raw_debug_enabled_) {
            return;
        }

        const char* log_dir = "log";
        mkdir(log_dir, 0755);

        time_t now = time(nullptr);
        char timestamp[64];
        struct tm* tm_now = localtime(&now);
        strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", tm_now);

        char csv_path[512];
        snprintf(csv_path, sizeof(csv_path), "%s/mpc_fpga_receiver_raw_%s.csv", log_dir, timestamp);

        raw_debug_csv_file_ = fopen(csv_path, "w");
        if (raw_debug_csv_file_ != nullptr) {
            fprintf(raw_debug_csv_file_,
                    "idx,stamp_sec,stamp_nsec,status,iters,used_fallback,max_iter_streak,"
                    "ey_fp,epsi_fp,vx_fp,vy_fp,omega_fp,steer_meas_fp,prev_accel_in_fp,"
                    "out_steer_fp,out_accel_fp,pub_steer,pub_speed,pub_accel,"
                    "ref_vx0_fp,ref_kappa0_fp,ref_left0_fp,ref_right0_fp\n");
            fflush(raw_debug_csv_file_);
            RCLCPP_INFO(get_logger(),
                        "Raw FPGA debug CSV enabled (stride=%u): %s",
                        raw_debug_stride_,
                        csv_path);
        } else {
            RCLCPP_WARN(get_logger(), "Failed to open raw debug CSV file: %s", csv_path);
        }
    }

    void log_raw_debug_cycle(const f1tenth_msgs::msg::MpcState::SharedPtr& msg,
                             const FrenetErrorsFp& errors,
                             int32_t prev_accel_in_fp,
                             int32_t out_steer_fp,
                             int32_t out_accel_fp,
                             uint32_t status,
                             uint32_t iters,
                             bool used_fallback,
                             float published_steer,
                             float published_speed,
                             float published_accel) {
        if (!raw_debug_enabled_ || raw_debug_csv_file_ == nullptr) {
            return;
        }

        raw_debug_idx_++;
        if ((raw_debug_idx_ % raw_debug_stride_) != 0) {
            return;
        }

        const int32_t ref_vx0_fp = msg->ref_vx_fp.empty() ? 0 : msg->ref_vx_fp[0];
        const int32_t ref_kappa0_fp = msg->ref_kappa_fp.empty() ? 0 : msg->ref_kappa_fp[0];
        const int32_t ref_left0_fp = msg->ref_left_bound_fp.empty() ? 0 : msg->ref_left_bound_fp[0];
        const int32_t ref_right0_fp = msg->ref_right_bound_fp.empty() ? 0 : msg->ref_right_bound_fp[0];

        fprintf(raw_debug_csv_file_,
                "%llu,%d,%u,%u,%u,%u,%llu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.7f,%.7f,%.7f,%d,%d,%d,%d\n",
                static_cast<unsigned long long>(raw_debug_idx_),
                msg->header.stamp.sec,
                msg->header.stamp.nanosec,
                status,
                iters,
                used_fallback ? 1u : 0u,
                static_cast<unsigned long long>(stats_max_iter_streak_),
                errors.e_y_fp,
                errors.e_psi_fp,
                msg->velocity_fp,
                msg->vy_fp,
                msg->omega_fp,
                msg->steering_angle_fp,
                prev_accel_in_fp,
                out_steer_fp,
                out_accel_fp,
                static_cast<double>(published_steer),
                static_cast<double>(published_speed),
                static_cast<double>(published_accel),
                ref_vx0_fp,
                ref_kappa0_fp,
                ref_left0_fp,
                ref_right0_fp);
        fflush(raw_debug_csv_file_);
    }

    void update_stats(int64_t compute_ns,
                      const MpcFpgaInterface::TimingProfile& profile,
                      uint32_t iterations,
                      uint32_t status) {
        const auto ns_to_us = [](int64_t ns) {
            return ns >= 0 ? static_cast<double>(ns) / 1000.0 : -1.0;
        };
        const double compute_us =
            (profile.kernel_ns >= 0) ? ns_to_us(profile.kernel_ns)
                                     : static_cast<double>(compute_ns) / 1000.0;
        const double total_call_us = ns_to_us(profile.total_call_ns);
        const double overhead_us = ns_to_us(profile.opencl_overhead_ns);
        const double input_pack_us = ns_to_us(profile.input_pack_ns);
        const double input_migrate_enq_us = ns_to_us(profile.input_migrate_enqueue_ns);
        const double kernel_enq_us = ns_to_us(profile.kernel_enqueue_ns);
        const double output_migrate_enq_us = ns_to_us(profile.output_migrate_enqueue_ns);
        const double wait_us = ns_to_us(profile.wait_ns);
        const double output_unpack_us = ns_to_us(profile.output_unpack_ns);
        const double input_migrate_us = ns_to_us(profile.input_migrate_ns);
        const double kernel_us = ns_to_us(profile.kernel_ns);
        const double output_migrate_us = ns_to_us(profile.output_migrate_ns);
        const double in_mig_submit_us = ns_to_us(profile.input_migrate_submit_ns);
        const double in_mig_sched_us = ns_to_us(profile.input_migrate_sched_ns);
        const double gap_in_to_kernel_us = ns_to_us(profile.gap_in_to_kernel_ns);
        const double kernel_submit_us = ns_to_us(profile.kernel_submit_ns);
        const double kernel_sched_us = ns_to_us(profile.kernel_sched_ns);
        const double gap_kernel_to_out_us = ns_to_us(profile.gap_kernel_to_out_ns);
        const double out_mig_submit_us = ns_to_us(profile.output_migrate_submit_ns);
        const double out_mig_sched_us = ns_to_us(profile.output_migrate_sched_ns);
        const double enqueue_us =
            input_migrate_enq_us + kernel_enq_us + output_migrate_enq_us;

        /* Track solve time statistics */
        stats_solve_time_sum_us_ += compute_us;
        if (compute_us < stats_solve_time_min_us_) stats_solve_time_min_us_ = compute_us;
        if (compute_us > stats_solve_time_max_us_) stats_solve_time_max_us_ = compute_us;
        if (total_call_us >= 0.0) stats_total_call_sum_us_ += total_call_us;
        if (kernel_us >= 0.0) stats_kernel_sum_us_ += kernel_us;
        if (input_migrate_us >= 0.0) stats_input_migrate_sum_us_ += input_migrate_us;
        if (output_migrate_us >= 0.0) stats_output_migrate_sum_us_ += output_migrate_us;
        if (input_pack_us >= 0.0) stats_input_pack_sum_us_ += input_pack_us;
        if (enqueue_us >= 0.0) stats_enqueue_sum_us_ += enqueue_us;
        if (overhead_us >= 0.0) stats_overhead_sum_us_ += overhead_us;
        
        /* Track iteration statistics */
        stats_iter_count_sum_ += static_cast<double>(iterations);
        if (iterations < stats_iter_count_min_) stats_iter_count_min_ = iterations;
        if (iterations > stats_iter_count_max_) stats_iter_count_max_ = iterations;

        /* Count optimal solutions and max-iter cases */
        if (status == MPC_FPGA_STATUS_OK) stats_optimal_count_++;
        if (status == MPC_FPGA_STATUS_MAX_ITER) stats_max_iter_count_++;
        if (status == MPC_FPGA_STATUS_ERROR) stats_error_count_++;
        if (status == MPC_FPGA_STATUS_NO_TRAJECTORY) stats_no_trajectory_count_++;
        if (status != MPC_FPGA_STATUS_OK &&
            status != MPC_FPGA_STATUS_MAX_ITER &&
            status != MPC_FPGA_STATUS_ERROR &&
            status != MPC_FPGA_STATUS_NO_TRAJECTORY) {
            stats_other_status_count_++;
        }
        if (status == MPC_FPGA_STATUS_MAX_ITER) {
            stats_max_iter_streak_++;
            if (stats_max_iter_streak_ > stats_max_iter_streak_max_) {
                stats_max_iter_streak_max_ = stats_max_iter_streak_;
            }
        } else {
            stats_max_iter_streak_ = 0;
        }
        
        stats_cycle_count_++;
        
        /* Write to stats CSV file */
        if (stats_csv_file_ != nullptr) {
            fprintf(stats_csv_file_,
                    "%lu,%u,%u,"
                    "%.3f,%.3f,%.3f,"
                    "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
                    "%.3f,%.3f,%.3f,"
                    "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                    stats_cycle_count_, iterations, status,
                    compute_us, total_call_us, overhead_us,
                    input_pack_us, input_migrate_enq_us, kernel_enq_us,
                    output_migrate_enq_us, wait_us, output_unpack_us,
                    input_migrate_us, kernel_us, output_migrate_us,
                    in_mig_submit_us, in_mig_sched_us, gap_in_to_kernel_us,
                    kernel_submit_us, kernel_sched_us, gap_kernel_to_out_us,
                    out_mig_submit_us, out_mig_sched_us);
            fflush(stats_csv_file_);
        }
        
        /* Print terminal stats every 5 seconds */
        auto now = std::chrono::steady_clock::now();
        double elapsed_sec = std::chrono::duration<double>(now - last_terminal_print_time_).count();
        
        if (elapsed_sec >= TERMINAL_STATS_PRINT_INTERVAL_SECONDS && stats_cycle_count_ > 0) {
            const auto avg_or_na = [this](double sum_us) {
                return sum_us > 0.0 ? (sum_us / static_cast<double>(stats_cycle_count_)) : -1.0;
            };
            double avg_iter = stats_iter_count_sum_ / static_cast<double>(stats_cycle_count_);
            double avg_time = stats_solve_time_sum_us_ / static_cast<double>(stats_cycle_count_);
            double avg_total_call = avg_or_na(stats_total_call_sum_us_);
            double avg_kernel = avg_or_na(stats_kernel_sum_us_);
            double avg_input_migrate = avg_or_na(stats_input_migrate_sum_us_);
            double avg_output_migrate = avg_or_na(stats_output_migrate_sum_us_);
            double avg_input_pack = avg_or_na(stats_input_pack_sum_us_);
            double avg_enqueue = avg_or_na(stats_enqueue_sum_us_);
            double avg_overhead = avg_or_na(stats_overhead_sum_us_);
            double optimal_pct = (stats_optimal_count_ * 100.0) / static_cast<double>(stats_cycle_count_);
            double max_iter_pct = (stats_max_iter_count_ * 100.0) / static_cast<double>(stats_cycle_count_);
            
            RCLCPP_INFO(get_logger(),
                "[FPGA-Stats] (last %.1fs, %lu calls):\n"
                "  Iterations: min=%u, avg=%.1f, max=%u\n"
                "  Solve time: min=%.1f us, avg=%.1f us, max=%.1f us\n"
                "  Profile avg: total=%.1f us, kernel=%.1f us, in_mig=%.1f us, out_mig=%.1f us, pack=%.1f us, enqueue=%.1f us, overhead=%.1f us\n"
                "  Status: ok=%llu, max_iter=%llu, err=%llu, no_traj=%llu, other=%llu, max_iter_streak_max=%llu\n"
                "  Optimal: %.1f%%, Max iter: %.1f%%",
                elapsed_sec, stats_cycle_count_,
                stats_iter_count_min_, avg_iter, stats_iter_count_max_,
                stats_solve_time_min_us_, avg_time, stats_solve_time_max_us_,
                avg_total_call, avg_kernel, avg_input_migrate, avg_output_migrate,
                avg_input_pack, avg_enqueue, avg_overhead,
                static_cast<unsigned long long>(stats_optimal_count_),
                static_cast<unsigned long long>(stats_max_iter_count_),
                static_cast<unsigned long long>(stats_error_count_),
                static_cast<unsigned long long>(stats_no_trajectory_count_),
                static_cast<unsigned long long>(stats_other_status_count_),
                static_cast<unsigned long long>(stats_max_iter_streak_max_),
                optimal_pct, max_iter_pct);
            
            /* Reset stats */
            stats_solve_time_sum_us_ = 0.0;
            stats_solve_time_min_us_ = 1e9;
            stats_solve_time_max_us_ = 0.0;
            stats_total_call_sum_us_ = 0.0;
            stats_kernel_sum_us_ = 0.0;
            stats_input_migrate_sum_us_ = 0.0;
            stats_output_migrate_sum_us_ = 0.0;
            stats_input_pack_sum_us_ = 0.0;
            stats_enqueue_sum_us_ = 0.0;
            stats_overhead_sum_us_ = 0.0;
            stats_iter_count_min_ = 0xFFFFFFFFU;
            stats_iter_count_max_ = 0;
            stats_iter_count_sum_ = 0.0;
            stats_cycle_count_ = 0;
            stats_optimal_count_ = 0;
            stats_max_iter_count_ = 0;
            stats_error_count_ = 0;
            stats_no_trajectory_count_ = 0;
            stats_other_status_count_ = 0;
            stats_max_iter_streak_max_ = stats_max_iter_streak_;
            last_terminal_print_time_ = now;
        }
    }

    bool has_required_horizon_data(const f1tenth_msgs::msg::MpcState::SharedPtr& msg) const {
        return msg->horizon_length >= MPC_HORIZON &&
               msg->ref_ey_fp.size() >= MPC_HORIZON &&
               msg->ref_epsi_fp.size() >= MPC_HORIZON &&
               msg->ref_vx_fp.size() >= MPC_HORIZON &&
               msg->ref_vy_fp.size() >= MPC_HORIZON &&
               msg->ref_omega_ref_fp.size() >= MPC_HORIZON &&
               msg->ref_kappa_fp.size() >= MPC_HORIZON &&
               msg->ref_left_bound_fp.size() >= MPC_HORIZON &&
               msg->ref_right_bound_fp.size() >= MPC_HORIZON;
    }

    float apply_standstill_brake_override(float vx_mps, float accel_cmd_mps2) {
        constexpr float kMinAccelMps2 = -(MPC_FPGA_MU * MPC_FPGA_GRAVITY_MS2);
        const bool standstill = std::isfinite(vx_mps) && std::fabs(vx_mps) < kStandstillVxThreshMps;
        const bool max_brake_cmd = std::isfinite(accel_cmd_mps2) &&
            (accel_cmd_mps2 <= (kMinAccelMps2 + kStandstillBrakeEpsMps2));

        if (standstill && max_brake_cmd) {
            return kStandstillAccelOverrideMps2;
        }
        return accel_cmd_mps2;
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

    void state_callback(const f1tenth_msgs::msg::MpcState::SharedPtr msg) {
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
                "Fixed-horizon MPC requires %d populated steps, got horizon=%u arrays: ey=%zu epsi=%zu vx=%zu vy=%zu omega_ref=%zu kappa=%zu left=%zu right=%zu",
                MPC_HORIZON,
                msg->horizon_length,
                msg->ref_ey_fp.size(),
                msg->ref_epsi_fp.size(),
                msg->ref_vx_fp.size(),
                msg->ref_vy_fp.size(),
                msg->ref_omega_ref_fp.size(),
                msg->ref_kappa_fp.size(),
                msg->ref_left_bound_fp.size(),
                msg->ref_right_bound_fp.size());
            return;
        }

        latest_vx_mps_ = fp_to_float(msg->velocity_fp);
        const FrenetErrorsFp errors{msg->e_y_fp, msg->e_psi_fp};
        const bool send_reset = fpga_reset_pending_;
        const uint32_t control_flags =
            send_reset ? MPC_FPGA_CTRL_FLAG_RESET_STATE
                       : MPC_FPGA_CTRL_FLAGS_NONE;
        const int32_t prev_accel_in_fp = send_reset ? 0 : fpga_.get_prev_accel_fp();

        if (send_reset) {
            last_steering_cmd_rad_ = 0.0f;
            last_speed_cmd_mps_ = MPC_FPGA_MIN_VEL_MPS;
            last_accel_cmd_mps2_ = 0.0f;
            fpga_.set_prev_accel_fp(0);
            RCLCPP_INFO(get_logger(),
                        "Resetting FPGA MPC state on first valid Jetson message");
        }

        const bool ok = fpga_.compute(
            errors.e_y_fp, errors.e_psi_fp,
            msg->velocity_fp, msg->vy_fp, msg->omega_fp,
            msg->steering_angle_fp,
            *msg,
            control_flags,
            out_steer_fp, out_accel_fp, status, iters);

        if (!ok) {
            drop_compute_++;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "FPGA OpenCL compute failed");
            return;
        }

        if (send_reset) {
            fpga_reset_pending_ = false;
        }

        /* Update statistics */
        update_stats(fpga_.get_last_compute_ns(), fpga_.get_last_profile(), iters, status);

        steering = fp_to_float(out_steer_fp);
        accel = fp_to_float(out_accel_fp);
        bool used_fallback = false;

        if (status == MPC_FPGA_STATUS_ERROR || status == MPC_FPGA_STATUS_NO_TRAJECTORY) {
            used_fallback = true;
            steering = last_steering_cmd_rad_;
            speed = last_speed_cmd_mps_;
            accel = last_accel_cmd_mps2_;
        } else {
            accel = apply_standstill_brake_override(latest_vx_mps_, accel);
            speed = compute_target_speed(accel, msg);
            steering = std::clamp(steering, -max_steering_, max_steering_);
            speed = std::clamp(speed, MPC_FPGA_MIN_VEL_MPS, max_velocity_);
            last_steering_cmd_rad_ = steering;
            last_speed_cmd_mps_ = speed;
            last_accel_cmd_mps2_ = accel;
            fpga_.set_prev_accel_fp(float_to_fp(accel));
        }

        if (status == MPC_FPGA_STATUS_MAX_ITER && stats_max_iter_streak_ >= 20) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "Sustained MAX_ITER streak=%llu (iters=%u, out_steer_fp=%d, out_accel_fp=%d, ey_fp=%d, epsi_fp=%d)",
                                 static_cast<unsigned long long>(stats_max_iter_streak_),
                                 iters,
                                 out_steer_fp,
                                 out_accel_fp,
                                 errors.e_y_fp,
                                 errors.e_psi_fp);
        }

        log_raw_debug_cycle(msg,
                            errors,
                            prev_accel_in_fp,
                            out_steer_fp,
                            out_accel_fp,
                            status,
                            iters,
                            used_fallback,
                            steering,
                            speed,
                            accel);

        publish_drive_command(steering, speed, accel, msg->header.stamp);
        pub_count_++;
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
