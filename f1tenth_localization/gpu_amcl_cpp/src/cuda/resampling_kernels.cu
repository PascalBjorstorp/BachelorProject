/**
 * @file resampling_kernels.cu
 * @brief CUDA kernels for low-variance systematic resampling.
 *
 * 1. Inclusive prefix-sum of weights.
 * 2. Systematic resampling using a single random offset.
 * 3. Sum-of-squares reduction for N_eff computation.
 */

#include "gpu_amcl_cpp/helpers/cuda_utils.hpp"
#include <cmath>

namespace gpu_amcl_cpp {

// ─── Inclusive prefix-sum ────────────────────────────────────────────
// Simple work-efficient scan for up to ~5000 particles.
// For larger arrays, use Thrust or CUB in production.
__global__
void kernel_inclusive_scan(const float* __restrict__ weights,
                           float* __restrict__ cumsum,
                           int n) {
    // Single-block sequential scan — sufficient for ≤5000 particles.
    if (threadIdx.x != 0 || blockIdx.x != 0) return;
    float s = 0.0f;
    for (int i = 0; i < n; ++i) {
        s += weights[i];
        cumsum[i] = s;
    }
}

void launch_inclusive_scan(const float* weights, float* cumsum,
                           int n, cudaStream_t stream) {
    kernel_inclusive_scan<<<1, 1, 0, stream>>>(weights, cumsum, n);
    CUDA_CHECK(cudaGetLastError());
}

// ─── Systematic resampling ──────────────────────────────────────────
__global__
void kernel_systematic_resample(const float* __restrict__ cumsum,
                                const float* __restrict__ old_particles,
                                float* __restrict__ new_particles,
                                float* __restrict__ new_weights,
                                int old_n, int new_n,
                                float random_offset) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= new_n) return;

    float step = 1.0f / static_cast<float>(new_n);
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
                                float random_offset,
                                cudaStream_t stream) {
    int block = 256;
    int grid  = (new_n + block - 1) / block;
    kernel_systematic_resample<<<grid, block, 0, stream>>>(
        cumsum, old_particles, new_particles, new_weights,
        old_n, new_n, random_offset);
    CUDA_CHECK(cudaGetLastError());
}

// ─── Sum of squared weights (for N_eff) ─────────────────────────────
__global__
void kernel_sum_sq(const float* __restrict__ weights, int n,
                   float* __restrict__ result) {
    // Single-block reduction — fine for ≤5000 particles.
    if (threadIdx.x != 0 || blockIdx.x != 0) return;
    float s = 0.0f;
    for (int i = 0; i < n; ++i) {
        s += weights[i] * weights[i];
    }
    result[0] = s;
}

double launch_sum_sq_weights(const float* weights, int n,
                             float* d_scratch, cudaStream_t stream) {
    kernel_sum_sq<<<1, 1, 0, stream>>>(weights, n, d_scratch);
    CUDA_CHECK(cudaGetLastError());

    float h_result;
    CUDA_CHECK(cudaMemcpyAsync(&h_result, d_scratch, sizeof(float),
                               cudaMemcpyDeviceToHost, stream));
    CUDA_CHECK(cudaStreamSynchronize(stream));
    return static_cast<double>(h_result);
}

}  // namespace gpu_amcl_cpp
