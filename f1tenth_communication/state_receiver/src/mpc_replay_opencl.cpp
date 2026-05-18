/**
 * @file mpc_replay_opencl.cpp
 * @brief Replay exported MpcState CSV rows through the live Kria OpenCL path.
 * @details Built inside the ROS2 state_receiver package so it uses the same
 *          OpenCL wrapper stack and kernel launch sequence as mpc_receiver.
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <vitis_common/common/ros_opencl_120.hpp>
#include <vitis_common/common/utilities.hpp>

#include "mpc_fpga_interface.h"

namespace {

struct ReplayRow {
    uint64_t idx = 0;
    int64_t stamp_ns = 0;
    int32_t x_fp = 0;
    int32_t y_fp = 0;
    int32_t theta_fp = 0;
    int32_t velocity_fp = 0;
    int32_t vy_fp = 0;
    int32_t omega_fp = 0;
    int32_t steering_angle_fp = 0;
    uint32_t horizon_length_msg = 0;
    std::array<int32_t, MPC_HORIZON> ref_ey_fp{};
    std::array<int32_t, MPC_HORIZON> ref_epsi_fp{};
    std::array<int32_t, MPC_HORIZON> ref_x_fp{};
    std::array<int32_t, MPC_HORIZON> ref_y_fp{};
    std::array<int32_t, MPC_HORIZON> ref_psi_fp{};
    std::array<int32_t, MPC_HORIZON> ref_vx_fp{};
    std::array<int32_t, MPC_HORIZON> ref_vy_fp{};
    std::array<int32_t, MPC_HORIZON> ref_omega_ref_fp{};
    std::array<int32_t, MPC_HORIZON> ref_kappa_fp{};
    std::array<int32_t, MPC_HORIZON> ref_left_bound_fp{};
    std::array<int32_t, MPC_HORIZON> ref_right_bound_fp{};
    int32_t input_e_y_fp = 0;
    int32_t input_epsi_fp = 0;
    bool has_input_frenet = false;
};

struct Options {
    std::string input_csv;
    std::string output_csv;
    std::string xclbin_path = "/lib/firmware/xilinx/MPC_FPGA/mpc_fpga_top_opencl.xclbin";
    std::string kernel_name = "mpc_fpga_top_opencl";
    int device_index = 0;
    bool reset_first = true;
    bool debug_echo_first = false;
    bool debug_echo_all = false;
};

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

inline float fp_to_float(int32_t fp) {
    return static_cast<float>(fp) / static_cast<float>(MPC_FPGA_QP_SCALE_I32);
}

inline int32_t float_to_fp(float f) {
    return static_cast<int32_t>(f >= 0.0f
                                    ? f * static_cast<float>(MPC_FPGA_QP_SCALE_I32) + 0.5f
                                    : f * static_cast<float>(MPC_FPGA_QP_SCALE_I32) - 0.5f);
}

static inline float wrap_pi(float a) {
    while (a > static_cast<float>(M_PI)) a -= 2.0f * static_cast<float>(M_PI);
    while (a < -static_cast<float>(M_PI)) a += 2.0f * static_cast<float>(M_PI);
    return a;
}

static void compute_frenet_errors_fp(const ReplayRow& row, int32_t& ey_fp, int32_t& epsi_fp) {
    const float x = fp_to_float(row.x_fp);
    const float y = fp_to_float(row.y_fp);
    const float theta = fp_to_float(row.theta_fp);
    const float ax = fp_to_float(row.ref_x_fp[0]);
    const float ay = fp_to_float(row.ref_y_fp[0]);
    const float bx = fp_to_float(row.ref_x_fp[1]);
    const float by = fp_to_float(row.ref_y_fp[1]);
    const float h0 = fp_to_float(row.ref_psi_fp[0]);
    const float h1 = fp_to_float(row.ref_psi_fp[1]);

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
    const float wpsi = h0 + t * wrap_pi(h1 - h0);
    const float dx = x - wx;
    const float dy = y - wy;

    ey_fp = float_to_fp(-std::sin(wpsi) * dx + std::cos(wpsi) * dy);
    epsi_fp = float_to_fp(wrap_pi(theta - wpsi));
}

static int next_long(char** ctx, long* out) {
    char* tok = ::strtok_r(nullptr, ",", ctx);
    char* end = nullptr;
    if (!tok) return 0;
    *out = std::strtol(tok, &end, 10);
    return end && (*end == '\0' || *end == '\n' || *end == '\r');
}

static int next_ll(char** ctx, long long* out) {
    char* tok = ::strtok_r(nullptr, ",", ctx);
    char* end = nullptr;
    if (!tok) return 0;
    *out = std::strtoll(tok, &end, 10);
    return end && (*end == '\0' || *end == '\n' || *end == '\r');
}

static int parse_row(char* line, ReplayRow& row) {
    char* ctx = nullptr;
    char* tok = ::strtok_r(line, ",", &ctx);
    long v = 0;
    long long vll = 0;

    if (!tok) return 0;
    row.idx = static_cast<uint64_t>(std::strtoull(tok, nullptr, 10));
    if (!next_long(&ctx, &v)) return 0;  // reserved source row counter
    if (!next_long(&ctx, &v)) return 0;  // reserved source stamp sec
    if (!next_ll(&ctx, &vll)) return 0;
    row.stamp_ns = static_cast<int64_t>(vll);

    if (!next_long(&ctx, &v)) return 0;
    row.x_fp = static_cast<int32_t>(v);
    if (!next_long(&ctx, &v)) return 0;
    row.y_fp = static_cast<int32_t>(v);
    if (!next_long(&ctx, &v)) return 0;
    row.theta_fp = static_cast<int32_t>(v);
    if (!next_long(&ctx, &v)) return 0;
    row.velocity_fp = static_cast<int32_t>(v);
    if (!next_long(&ctx, &v)) return 0;
    row.vy_fp = static_cast<int32_t>(v);
    if (!next_long(&ctx, &v)) return 0;
    row.omega_fp = static_cast<int32_t>(v);
    if (!next_long(&ctx, &v)) return 0;
    row.steering_angle_fp = static_cast<int32_t>(v);
    if (!next_long(&ctx, &v)) return 0;
    row.horizon_length_msg = static_cast<uint32_t>(v);

    for (size_t i = 0; i < MPC_HORIZON; ++i) {
        if (!next_long(&ctx, &v)) return 0;
        row.ref_ey_fp[i] = static_cast<int32_t>(v);
    }
    for (size_t i = 0; i < MPC_HORIZON; ++i) {
        if (!next_long(&ctx, &v)) return 0;
        row.ref_epsi_fp[i] = static_cast<int32_t>(v);
    }
    for (size_t i = 0; i < MPC_HORIZON; ++i) {
        if (!next_long(&ctx, &v)) return 0;
        row.ref_x_fp[i] = static_cast<int32_t>(v);
    }
    for (size_t i = 0; i < MPC_HORIZON; ++i) {
        if (!next_long(&ctx, &v)) return 0;
        row.ref_y_fp[i] = static_cast<int32_t>(v);
    }
    for (size_t i = 0; i < MPC_HORIZON; ++i) {
        if (!next_long(&ctx, &v)) return 0;
        row.ref_psi_fp[i] = static_cast<int32_t>(v);
    }
    for (size_t i = 0; i < MPC_HORIZON; ++i) {
        if (!next_long(&ctx, &v)) return 0;
        row.ref_vx_fp[i] = static_cast<int32_t>(v);
    }
    for (size_t i = 0; i < MPC_HORIZON; ++i) {
        if (!next_long(&ctx, &v)) return 0;
        row.ref_vy_fp[i] = static_cast<int32_t>(v);
    }
    for (size_t i = 0; i < MPC_HORIZON; ++i) {
        if (!next_long(&ctx, &v)) return 0;
        row.ref_omega_ref_fp[i] = static_cast<int32_t>(v);
    }
    for (size_t i = 0; i < MPC_HORIZON; ++i) {
        if (!next_long(&ctx, &v)) return 0;
        row.ref_kappa_fp[i] = static_cast<int32_t>(v);
    }
    for (size_t i = 0; i < MPC_HORIZON; ++i) {
        if (!next_long(&ctx, &v)) return 0;
        row.ref_left_bound_fp[i] = static_cast<int32_t>(v);
    }
    for (size_t i = 0; i < MPC_HORIZON; ++i) {
        if (!next_long(&ctx, &v)) return 0;
        row.ref_right_bound_fp[i] = static_cast<int32_t>(v);
    }

    row.has_input_frenet = false;
    if (next_long(&ctx, &v)) {
        row.input_e_y_fp = static_cast<int32_t>(v);
        if (!next_long(&ctx, &v)) return 0;
        row.input_epsi_fp = static_cast<int32_t>(v);
        row.has_input_frenet = true;
    }
    return 1;
}

class MpcFpgaReplayInterface {
public:
    struct TimingProfile {
        int64_t total_call_ns = 0;
        int64_t input_pack_ns = 0;
        int64_t input_migrate_enqueue_ns = 0;
        int64_t kernel_enqueue_ns = 0;
        int64_t output_migrate_enqueue_ns = 0;
        int64_t wait_ns = 0;
        int64_t output_unpack_ns = 0;
        int64_t input_migrate_ns = -1;
        int64_t kernel_ns = -1;
        int64_t output_migrate_ns = -1;
        int64_t opencl_overhead_ns = -1;
    };

    bool initialize(const std::string& xclbin_path,
                    const std::string& kernel_name,
                    int device_index) {
        cl_int err = CL_SUCCESS;
        std::vector<cl::Device> devices = get_xilinx_devices();
        if (devices.empty()) {
            std::fprintf(stderr, "MPC replay: no Xilinx OpenCL devices found\n");
            return false;
        }
        if (device_index < 0 || static_cast<size_t>(device_index) >= devices.size()) {
            std::fprintf(stderr, "MPC replay: device_index=%d out of range (devices=%zu)\n",
                         device_index, devices.size());
            return false;
        }

        device_ = devices[static_cast<size_t>(device_index)];
        context_ = cl::Context(device_, nullptr, nullptr, nullptr, &err);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "MPC replay: context creation failed (%d)\n", err);
            return false;
        }

        queue_ = cl::CommandQueue(context_, device_, CL_QUEUE_PROFILING_ENABLE, &err);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "MPC replay: command queue creation failed (%d)\n", err);
            return false;
        }

        std::vector<unsigned char> file_buf;
        try {
            file_buf = read_file_bytes(xclbin_path);
        } catch (const std::exception& e) {
            std::fprintf(stderr, "MPC replay: xclbin read failed: %s (%s)\n",
                         xclbin_path.c_str(), e.what());
            return false;
        }

        cl::Program::Binaries bins{{file_buf.data(), file_buf.size()}};
        std::vector<cl::Device> program_devices{device_};
        program_ = cl::Program(context_, program_devices, bins, nullptr, &err);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "MPC replay: program load failed (%d)\n", err);
            return false;
        }

        kernel_ = cl::Kernel(program_, kernel_name.c_str(), &err);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "MPC replay: kernel '%s' creation failed (%d)\n",
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
            std::fprintf(stderr, "MPC replay: input buffer allocation failed (%d)\n", err);
            return false;
        }

        output_buffer_ = cl::Buffer(
            context_,
            static_cast<cl_mem_flags>(CL_MEM_WRITE_ONLY | CL_MEM_ALLOC_HOST_PTR),
            sizeof(int32_t) * 4,
            nullptr,
            &err);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "MPC replay: output buffer allocation failed (%d)\n", err);
            return false;
        }

        err = kernel_.setArg(0, input_buffer_);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "MPC replay: setArg(0) failed (%d)\n", err);
            return false;
        }
        err = kernel_.setArg(1, output_buffer_);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "MPC replay: setArg(1) failed (%d)\n", err);
            return false;
        }

        input_ptr_ = static_cast<int32_t*>(queue_.enqueueMapBuffer(
            input_buffer_, CL_TRUE, static_cast<cl_map_flags>(CL_MAP_WRITE), 0,
            INPUT_BUFFER_BYTES_512, nullptr, nullptr, &err));
        if (err != CL_SUCCESS || input_ptr_ == nullptr) {
            std::fprintf(stderr, "MPC replay: input map failed (%d)\n", err);
            return false;
        }

        output_ptr_ = static_cast<int32_t*>(queue_.enqueueMapBuffer(
            output_buffer_, CL_TRUE, static_cast<cl_map_flags>(CL_MAP_READ), 0,
            sizeof(int32_t) * 4, nullptr, nullptr, &err));
        if (err != CL_SUCCESS || output_ptr_ == nullptr) {
            std::fprintf(stderr, "MPC replay: output map failed (%d)\n", err);
            return false;
        }

        for (size_t i = INPUT_BUFFER_WORDS_32; i < INPUT_BUFFER_WORDS_32_PAD; ++i) {
            input_ptr_[i] = 0;
        }

        initialized_ = true;
        return true;
    }

    void set_prev_accel_fp(int32_t prev_accel_fp) { prev_accel_fp_ = prev_accel_fp; }
    int32_t get_prev_accel_fp() const { return prev_accel_fp_; }
    const TimingProfile& last_profile() const { return last_profile_; }

    bool compute(const ReplayRow& row,
                 int32_t ey_fp,
                 int32_t epsi_fp,
                 uint32_t control_flags,
                 int32_t& out_steer_fp,
                 int32_t& out_accel_fp,
                 uint32_t& out_status,
                 uint32_t& out_iters) {
        if (!initialized_) return false;

        last_profile_ = TimingProfile{};
        cl_int err = CL_SUCCESS;
        const auto total_t0 = std::chrono::high_resolution_clock::now();
        const auto pack_t0 = total_t0;

        input_ptr_[MPC_FPGA_WORD_EY] = ey_fp;
        input_ptr_[MPC_FPGA_WORD_EPSI] = epsi_fp;
        input_ptr_[MPC_FPGA_WORD_VX] = row.velocity_fp;
        input_ptr_[MPC_FPGA_WORD_VY] = row.vy_fp;
        input_ptr_[MPC_FPGA_WORD_OMEGA] = row.omega_fp;
        input_ptr_[MPC_FPGA_WORD_STEERING] = row.steering_angle_fp;
        input_ptr_[MPC_FPGA_WORD_CONTROL_FLAGS] = static_cast<int32_t>(control_flags);
        input_ptr_[MPC_FPGA_WORD_PREV_ACCEL] = prev_accel_fp_;

        for (size_t i = 0; i < MPC_HORIZON; ++i) {
            const size_t base = MPC_FPGA_HEADER_WORDS + (i * MPC_FPGA_REF_WORDS);
            input_ptr_[base + 0] = row.ref_ey_fp[i];
            input_ptr_[base + 1] = row.ref_epsi_fp[i];
            input_ptr_[base + 2] = row.ref_vx_fp[i];
            input_ptr_[base + 3] = row.ref_vy_fp[i];
            input_ptr_[base + 4] = row.ref_omega_ref_fp[i];
            input_ptr_[base + 5] = row.ref_kappa_fp[i];
            input_ptr_[base + 6] = row.ref_left_bound_fp[i];
            input_ptr_[base + 7] = row.ref_right_bound_fp[i];
        }
        const auto pack_t1 = std::chrono::high_resolution_clock::now();
        last_profile_.input_pack_ns = ns_between(pack_t0, pack_t1);

        cl::Event in_event;
        cl::Event kernel_event;
        cl::Event out_event;
        const std::vector<cl::Memory> input_mem{input_buffer_};
        const std::vector<cl::Memory> output_mem{output_buffer_};

        const auto in_t0 = std::chrono::high_resolution_clock::now();
        err = queue_.enqueueMigrateMemObjects(input_mem, 0, nullptr, &in_event);
        const auto in_t1 = std::chrono::high_resolution_clock::now();
        last_profile_.input_migrate_enqueue_ns = ns_between(in_t0, in_t1);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "MPC replay: migrate(input) failed (%d)\n", err);
            return false;
        }

        std::vector<cl::Event> wait_input{in_event};
        const auto k_t0 = std::chrono::high_resolution_clock::now();
        err = queue_.enqueueTask(kernel_, &wait_input, &kernel_event);
        const auto k_t1 = std::chrono::high_resolution_clock::now();
        last_profile_.kernel_enqueue_ns = ns_between(k_t0, k_t1);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "MPC replay: enqueueTask failed (%d)\n", err);
            return false;
        }

        std::vector<cl::Event> wait_kernel{kernel_event};
        const auto out_t0 = std::chrono::high_resolution_clock::now();
        err = queue_.enqueueMigrateMemObjects(
            output_mem, CL_MIGRATE_MEM_OBJECT_HOST, &wait_kernel, &out_event);
        const auto out_t1 = std::chrono::high_resolution_clock::now();
        last_profile_.output_migrate_enqueue_ns = ns_between(out_t0, out_t1);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "MPC replay: migrate(output) failed (%d)\n", err);
            return false;
        }

        const auto wait_t0 = std::chrono::high_resolution_clock::now();
        err = cl::Event::waitForEvents({out_event});
        const auto wait_t1 = std::chrono::high_resolution_clock::now();
        last_profile_.wait_ns = ns_between(wait_t0, wait_t1);
        if (err != CL_SUCCESS) {
            std::fprintf(stderr, "MPC replay: wait failed (%d)\n", err);
            return false;
        }

        last_profile_.input_migrate_ns = get_profile_duration_ns(in_event);
        last_profile_.kernel_ns = get_profile_duration_ns(kernel_event);
        last_profile_.output_migrate_ns = get_profile_duration_ns(out_event);

        const auto unpack_t0 = std::chrono::high_resolution_clock::now();
        out_steer_fp = output_ptr_[0];
        out_accel_fp = output_ptr_[1];
        out_status = static_cast<uint32_t>(output_ptr_[2]);
        out_iters = static_cast<uint32_t>(output_ptr_[3]);
        const auto unpack_t1 = std::chrono::high_resolution_clock::now();
        last_profile_.output_unpack_ns = ns_between(unpack_t0, unpack_t1);

        last_profile_.total_call_ns = ns_between(total_t0, unpack_t1);
        if (last_profile_.kernel_ns >= 0) {
            last_profile_.opencl_overhead_ns =
                last_profile_.total_call_ns - last_profile_.kernel_ns;
        }
        return true;
    }

private:
    template <typename TP>
    static int64_t ns_between(const TP& a, const TP& b) {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(b - a).count();
    }

    static int64_t get_profile_duration_ns(const cl::Event& event) {
        cl_int err = CL_SUCCESS;
        cl_ulong start = event.getProfilingInfo<CL_PROFILING_COMMAND_START>(&err);
        if (err != CL_SUCCESS) return -1;
        cl_ulong end = event.getProfilingInfo<CL_PROFILING_COMMAND_END>(&err);
        if (err != CL_SUCCESS || end < start) return -1;
        return static_cast<int64_t>(end - start);
    }

    bool initialized_ = false;
    cl::Device device_;
    cl::Context context_;
    cl::Program program_;
    cl::Kernel kernel_;
    cl::CommandQueue queue_;
    cl::Buffer input_buffer_;
    cl::Buffer output_buffer_;
    int32_t* input_ptr_ = nullptr;
    int32_t* output_ptr_ = nullptr;
    int32_t prev_accel_fp_ = 0;
    TimingProfile last_profile_{};
};

static void usage(const char* argv0) {
    std::fprintf(stderr,
                 "Usage: %s <state_replay.csv> <out.csv> [--xclbin PATH] "
                 "[--kernel NAME] [--device-index N] [--no-reset-first] "
                 "[--debug-echo-first] [--debug-echo-all]\n",
                 argv0);
}

static bool parse_args(int argc, char** argv, Options& opts) {
    if (argc < 3) {
        usage(argv[0]);
        return false;
    }
    opts.input_csv = argv[1];
    opts.output_csv = argv[2];
    for (int i = 3; i < argc; ++i) {
        if (std::strcmp(argv[i], "--xclbin") == 0 && (i + 1) < argc) {
            opts.xclbin_path = argv[++i];
        } else if (std::strcmp(argv[i], "--kernel") == 0 && (i + 1) < argc) {
            opts.kernel_name = argv[++i];
        } else if (std::strcmp(argv[i], "--device-index") == 0 && (i + 1) < argc) {
            opts.device_index = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--no-reset-first") == 0) {
            opts.reset_first = false;
        } else if (std::strcmp(argv[i], "--debug-echo-first") == 0) {
            opts.debug_echo_first = true;
        } else if (std::strcmp(argv[i], "--debug-echo-all") == 0) {
            opts.debug_echo_all = true;
        } else {
            std::fprintf(stderr, "Unknown arg: %s\n", argv[i]);
            usage(argv[0]);
            return false;
        }
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    Options opts;
    if (!parse_args(argc, argv, opts)) {
        return 2;
    }

    std::FILE* in = std::fopen(opts.input_csv.c_str(), "r");
    if (!in) {
        std::perror("open input");
        return 3;
    }
    std::FILE* out = std::fopen(opts.output_csv.c_str(), "w");
    if (!out) {
        std::perror("open output");
        std::fclose(in);
        return 4;
    }

    MpcFpgaReplayInterface fpga;
    if (!fpga.initialize(opts.xclbin_path, opts.kernel_name, opts.device_index)) {
        std::fclose(in);
        std::fclose(out);
        return 5;
    }

    char line[65536];
    if (!std::fgets(line, sizeof(line), in)) {
        std::fprintf(stderr, "Input CSV empty\n");
        std::fclose(in);
        std::fclose(out);
        return 6;
    }

    std::fprintf(out,
                 "idx,stamp_ns,compute_ok,status,iters,control_flags,"
                 "out_steer_fp,out_accel_fp,ey_fp,epsi_fp,vx_fp,vy_fp,omega_fp,steer_meas_fp,"
                 "prev_accel_in_fp,ref_vx0_fp,ref_kappa0_fp,total_call_ns,kernel_ns,wait_ns\n");

    bool first_row = true;
    uint64_t row_count = 0;

    while (std::fgets(line, sizeof(line), in)) {
        ReplayRow row{};
        if (!parse_row(line, row)) {
            std::fprintf(stderr, "Skipping malformed row\n");
            continue;
        }
        if (row.horizon_length_msg < static_cast<uint32_t>(MPC_HORIZON)) {
            std::fprintf(stderr, "Skipping row %llu: horizon_length=%u < %d\n",
                         static_cast<unsigned long long>(row.idx),
                         row.horizon_length_msg,
                         MPC_HORIZON);
            continue;
        }

        int32_t ey_fp = 0;
        int32_t epsi_fp = 0;
        if (row.has_input_frenet) {
            ey_fp = row.input_e_y_fp;
            epsi_fp = row.input_epsi_fp;
        } else {
            compute_frenet_errors_fp(row, ey_fp, epsi_fp);
        }

        uint32_t control_flags =
            (first_row && opts.reset_first) ? MPC_FPGA_CTRL_FLAG_RESET_STATE : 0u;
        if (opts.debug_echo_all || (opts.debug_echo_first && first_row)) {
            control_flags |= MPC_FPGA_CTRL_FLAG_DEBUG_ECHO_INPUTS;
        }
        const int32_t prev_accel_in_fp =
            (first_row && opts.reset_first) ? 0 : fpga.get_prev_accel_fp();

        if (first_row && opts.reset_first) {
            fpga.set_prev_accel_fp(0);
        }

        int32_t out_steer_fp = 0;
        int32_t out_accel_fp = 0;
        uint32_t status = 0;
        uint32_t iters = 0;
        const bool ok = fpga.compute(
            row, ey_fp, epsi_fp, control_flags, out_steer_fp, out_accel_fp, status, iters);

        if (ok) {
            fpga.set_prev_accel_fp(out_accel_fp);
        }

        const auto& prof = fpga.last_profile();
        std::fprintf(out,
                     "%llu,%lld,%u,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%lld,%lld,%lld\n",
                     static_cast<unsigned long long>(row.idx),
                     static_cast<long long>(row.stamp_ns),
                     ok ? 1u : 0u,
                     status,
                     iters,
                     control_flags,
                     out_steer_fp,
                     out_accel_fp,
                     ey_fp,
                     epsi_fp,
                     row.velocity_fp,
                     row.vy_fp,
                     row.omega_fp,
                     row.steering_angle_fp,
                     prev_accel_in_fp,
                     row.ref_vx_fp[0],
                     row.ref_kappa_fp[0],
                     static_cast<long long>(prof.total_call_ns),
                     static_cast<long long>(prof.kernel_ns),
                     static_cast<long long>(prof.wait_ns));

        first_row = false;
        ++row_count;
    }

    std::fclose(in);
    std::fclose(out);
    std::fprintf(stderr, "Wrote %llu OpenCL replay rows to %s\n",
                 static_cast<unsigned long long>(row_count),
                 opts.output_csv.c_str());
    return 0;
}
