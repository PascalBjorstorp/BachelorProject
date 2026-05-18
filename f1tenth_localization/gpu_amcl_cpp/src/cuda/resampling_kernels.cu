/**
 * @file resampling_kernels.cu
 * @brief CUDA kernels for low-variance systematic resampling.
 *
 * 1. Inclusive prefix-sum of weights  — CUB DeviceScan.
 * 2. Systematic resampling using a single random offset.
 * 3. Sum-of-squares reduction for N_eff — CUB DeviceReduce.
 */

#include "gpu_amcl_cpp/helpers/cuda_utils.hpp"
#include <cub/cub.cuh>
#include <cmath>

namespace gpu_amcl_cpp {

// ─── CUB-based inclusive prefix-sum ─────────────────────────────
// Replaces single-thread O(N) sequential scan with CUB's work-efficient
// parallel scan (~10-15× faster for N=5000).

size_t query_scan_temp_bytes(int n) {
    size_t temp_bytes = 0;
    cub::DeviceScan::InclusiveSum(
        nullptr, temp_bytes,
        static_cast<const float*>(nullptr),
        static_cast<float*>(nullptr), n);
    return temp_bytes;
}

size_t query_sum_temp_bytes(int n) {
    size_t temp_bytes = 0;
    cub::DeviceReduce::Sum(
        nullptr, temp_bytes,
        static_cast<const float*>(nullptr),
        static_cast<float*>(nullptr), n);
    return temp_bytes;
}

void launch_inclusive_scan(const float* weights, float* cumsum,
                           int n,
                           void* d_temp, size_t temp_bytes,
                           cudaStream_t stream) {
    cub::DeviceScan::InclusiveSum(d_temp, temp_bytes,
                                  weights, cumsum, n, stream);
    CUDA_CHECK(cudaGetLastError());
}

__global__
void kernel_prepare_resample_weights(const float* __restrict__ weights,
                                     float* __restrict__ out_weights,
                                     int n,
                                     float weight_power,
                                     float uniform_floor) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;

    float w = fmaxf(weights[i], 0.0f);
    if (weight_power > 0.0f && fabsf(weight_power - 1.0f) > 1e-6f) {
        w = powf(w, weight_power);
    }
    w += fmaxf(uniform_floor, 0.0f) / static_cast<float>(n);
    out_weights[i] = w;
}

double launch_prepare_resample_weights(const float* weights,
                                       float* out_weights,
                                       int n,
                                       float weight_power,
                                       float uniform_floor,
                                       float* d_result,
                                       void* d_temp,
                                       size_t temp_bytes,
                                       cudaStream_t stream) {
    int block = 256;
    int grid = (n + block - 1) / block;
    kernel_prepare_resample_weights<<<grid, block, 0, stream>>>(
        weights, out_weights, n, weight_power, uniform_floor);
    CUDA_CHECK(cudaGetLastError());

    cub::DeviceReduce::Sum(d_temp, temp_bytes,
                           out_weights, d_result, n, stream);
    CUDA_CHECK(cudaGetLastError());

    float h_result = 0.0f;
    CUDA_CHECK(cudaMemcpyAsync(&h_result, d_result, sizeof(float),
                               cudaMemcpyDeviceToHost, stream));
    CUDA_CHECK(cudaStreamSynchronize(stream));
    return static_cast<double>(h_result);
}

// ─── Systematic resampling ──────────────────────────────────────────
__global__
void kernel_systematic_resample(const float* __restrict__ cumsum,
                                const float* __restrict__ old_particles,
                                float* __restrict__ new_particles,
                                float* __restrict__ new_weights,
                                int old_n, int new_n,
                                float total_weight,
                                float random_offset) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= new_n) return;

    float step = total_weight / static_cast<float>(new_n);
    float target = random_offset + i * step;

    // Binary search in cumsum.
    int lo = 0, hi = old_n - 1;
    while (lo < hi) {
        int mid = (lo + hi) / 2;
        if (cumsum[mid] < target)
            lo = mid + 1;
        else
            hi = mid;
    }

    // Copy selected particle.
    new_particles[i * 3 + 0] = old_particles[lo * 3 + 0];
    new_particles[i * 3 + 1] = old_particles[lo * 3 + 1];
    new_particles[i * 3 + 2] = old_particles[lo * 3 + 2];
    new_weights[i] = 1.0f / static_cast<float>(new_n);
}

void launch_systematic_resample(const float* cumsum,
                                const float* old_particles,
                                float* new_particles,
                                float* new_weights,
                                int old_n, int new_n,
                                float total_weight,
                                float random_offset,
                                cudaStream_t stream) {
    int block = 256;
    int grid  = (new_n + block - 1) / block;
    kernel_systematic_resample<<<grid, block, 0, stream>>>(
        cumsum, old_particles, new_particles, new_weights,
        old_n, new_n, total_weight, random_offset);
    CUDA_CHECK(cudaGetLastError());
}

// ─── CUB-based sum of squared weights (for N_eff) ──────────────
// Uses TransformInputIterator to fuse the square operation with the
// reduction — no temporary buffer needed for squared values.

struct SquareOp {
    __host__ __device__ __forceinline__
    float operator()(float x) const { return x * x; }
};

using SquareIter = cub::TransformInputIterator<float, SquareOp, const float*>;

size_t query_sumsq_temp_bytes(int n) {
    size_t temp_bytes = 0;
    SquareIter sq_iter(nullptr, SquareOp{});
    cub::DeviceReduce::Sum(nullptr, temp_bytes,
                           sq_iter, static_cast<float*>(nullptr), n);
    return temp_bytes;
}

double launch_sum_sq_weights(const float* weights, int n,
                             float* d_result,
                             void* d_temp, size_t temp_bytes,
                             cudaStream_t stream) {
    SquareIter sq_iter(weights, SquareOp{});
    cub::DeviceReduce::Sum(d_temp, temp_bytes,
                           sq_iter, d_result, n, stream);
    CUDA_CHECK(cudaGetLastError());

    float h_result;
    CUDA_CHECK(cudaMemcpyAsync(&h_result, d_result, sizeof(float),
                               cudaMemcpyDeviceToHost, stream));
    CUDA_CHECK(cudaStreamSynchronize(stream));
    return static_cast<double>(h_result);
}

}  // namespace gpu_amcl_cpp
