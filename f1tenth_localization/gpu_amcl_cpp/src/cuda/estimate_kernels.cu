#include "gpu_amcl_cpp/helpers/cuda_utils.hpp"
#include <cub/cub.cuh>
#include <cmath>

namespace gpu_amcl_cpp {

// ─── Intermediate struct for per-particle contributions ─────────────
struct MeanAccum {
    float wx;      // w * x
    float wy;      // w * y
    float w_sin;   // w * sin(θ)
    float w_cos;   // w * cos(θ)
};

struct CovAccum {
    float c00, c01, c02;  // Row 0
    float c11, c12;       // Row 1 (c10 = c01)
    float c22;            // Row 2 (c20 = c02, c21 = c12)
};

// ─── CUB reduction operator for MeanAccum ────────────────────────────
struct MeanAccumOp {
    __device__ __forceinline__
    MeanAccum operator()(const MeanAccum& a, const MeanAccum& b) const {
        return MeanAccum{
            a.wx + b.wx,
            a.wy + b.wy,
            a.w_sin + b.w_sin,
            a.w_cos + b.w_cos
        };
    }
};

// ─── CUB reduction operator for CovAccum ─────────────────────────────
struct CovAccumOp {
    __device__ __forceinline__
    CovAccum operator()(const CovAccum& a, const CovAccum& b) const {
        return CovAccum{
            a.c00 + b.c00, a.c01 + b.c01, a.c02 + b.c02,
            a.c11 + b.c11, a.c12 + b.c12,
            a.c22 + b.c22
        };
    }
};

// ─── Kernel: compute per-particle mean contributions ─────────────────
__global__ void kernel_compute_mean_contrib(
    const float* __restrict__ particles,
    const float* __restrict__ weights,
    MeanAccum* __restrict__ contrib,
    int n
) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;

    float w = weights[i];
    float x = particles[i * 3 + 0];
    float y = particles[i * 3 + 1];
    float theta = particles[i * 3 + 2];

    contrib[i] = MeanAccum{
        w * x,
        w * y,
        w * sinf(theta),
        w * cosf(theta)
    };
}

// ─── Kernel: compute per-particle covariance contributions ───────────
__global__ void kernel_compute_cov_contrib(
    const float* __restrict__ particles,
    const float* __restrict__ weights,
    float mean_x, float mean_y, float mean_theta,
    CovAccum* __restrict__ contrib,
    int n
) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;

    float w = weights[i];
    float dx = particles[i * 3 + 0] - mean_x;
    float dy = particles[i * 3 + 1] - mean_y;

    // Angle difference with proper wraparound
    float raw_dtheta = particles[i * 3 + 2] - mean_theta;
    float dtheta = atan2f(sinf(raw_dtheta), cosf(raw_dtheta));

    // Covariance contribution: w * (diff * diff^T)
    // Matrix is symmetric, only store upper triangle
    contrib[i] = CovAccum{
        w * dx * dx,      w * dx * dy,      w * dx * dtheta,
        w * dy * dy,      w * dy * dtheta,
        w * dtheta * dtheta
    };
}

// ─── Query temp storage sizes ────────────────────────────────────────
size_t query_estimate_mean_temp_bytes(int n) {
    size_t temp_bytes = 0;
    MeanAccum* d_in = nullptr;
    MeanAccum* d_out = nullptr;
    cub::DeviceReduce::Reduce(nullptr, temp_bytes, d_in, d_out, n,
                               MeanAccumOp(), MeanAccum{0,0,0,0});
    return temp_bytes;
}

size_t query_estimate_cov_temp_bytes(int n) {
    size_t temp_bytes = 0;
    CovAccum* d_in = nullptr;
    CovAccum* d_out = nullptr;
    cub::DeviceReduce::Reduce(nullptr, temp_bytes, d_in, d_out, n,
                               CovAccumOp(), CovAccum{0,0,0,0,0,0});
    return temp_bytes;
}

// ─── Launch mean computation ─────────────────────────────────────────
void launch_gpu_compute_mean(
    const float* d_particles,
    const float* d_weights,
    void* d_mean_contrib_v,       // MeanAccum[n]
    void* d_mean_result_v,        // MeanAccum[1]
    void* d_temp,
    size_t temp_bytes,
    int n,
    cudaStream_t stream
) {
    MeanAccum* d_mean_contrib = static_cast<MeanAccum*>(d_mean_contrib_v);
    MeanAccum* d_mean_result = static_cast<MeanAccum*>(d_mean_result_v);

    if (n <= 0) return;
    const auto launch = make_adaptive_launch_config(n);

    // Step 1: Compute per-particle contributions
    kernel_compute_mean_contrib<<<launch.grid, launch.block, 0, stream>>>(
        d_particles, d_weights, d_mean_contrib, n);

    // Step 2: CUB reduction
    cub::DeviceReduce::Reduce(d_temp, temp_bytes,
                               d_mean_contrib, d_mean_result, n,
                               MeanAccumOp(), MeanAccum{0,0,0,0}, stream);
}

// ─── Launch covariance computation ───────────────────────────────────
void launch_gpu_compute_covariance(
    const float* d_particles,
    const float* d_weights,
    float mean_x, float mean_y, float mean_theta,
    void* d_cov_contrib_v,        // CovAccum[n]
    void* d_cov_result_v,         // CovAccum[1]
    void* d_temp,
    size_t temp_bytes,
    int n,
    cudaStream_t stream
) {
    CovAccum* d_cov_contrib = static_cast<CovAccum*>(d_cov_contrib_v);
    CovAccum* d_cov_result = static_cast<CovAccum*>(d_cov_result_v);

    if (n <= 0) return;
    const auto launch = make_adaptive_launch_config(n);

    // Step 1: Compute per-particle contributions
    kernel_compute_cov_contrib<<<launch.grid, launch.block, 0, stream>>>(
        d_particles, d_weights, mean_x, mean_y, mean_theta, d_cov_contrib, n);

    // Step 2: CUB reduction
    cub::DeviceReduce::Reduce(d_temp, temp_bytes,
                               d_cov_contrib, d_cov_result, n,
                               CovAccumOp(), CovAccum{0,0,0,0,0,0}, stream);
}

}  // namespace gpu_amcl_cpp
