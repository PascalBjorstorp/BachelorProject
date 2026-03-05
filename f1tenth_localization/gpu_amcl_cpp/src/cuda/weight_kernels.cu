/**
 * @file weight_kernels.cu
 * @brief CUDA kernels for GPU-side weight normalization (§1).
 *
 * Replaces CPU-side log-weight normalisation with fully GPU-resident
 * operations using CUB DeviceReduce and custom element-wise kernels.
 *
 * Pipeline (all on GPU, zero host↔device transfers):
 *   1. CUB::DeviceReduce::Max(d_log_w) → d_max
 *   2. kernel: w[i] = exp(log_w[i] - max) * old_w[i]
 *   3. CUB::DeviceReduce::Sum(d_weights) → d_sum
 *   4. kernel: w[i] /= sum
 */

#include "gpu_amcl_cpp/helpers/cuda_utils.hpp"
#include <cub/cub.cuh>
#include <cmath>
#include <cfloat>

namespace gpu_amcl_cpp {

// ─── Element-wise kernels ───────────────────────────────────────────

/// Fused exp-shift-multiply: w[i] = exp(log_w[i] - max_lw) * old_w[i]
__global__
void kernel_exp_shift_mul(const float* __restrict__ log_w,
                          const float* __restrict__ old_w,
                          float* __restrict__ out_w,
                          const float* __restrict__ d_max,
                          int n) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    out_w[i] = expf(log_w[i] - *d_max) * old_w[i];
}

/// Normalize in-place: w[i] /= sum.  Falls back to uniform if sum ≤ 0.
__global__
void kernel_normalize(float* __restrict__ w,
                      const float* __restrict__ d_sum,
                      int n) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;

    float s = *d_sum;
    if (s > 0.0f) {
        w[i] /= s;
    } else {
        w[i] = 1.0f / static_cast<float>(n);
    }
}

// ─── CUB temp-storage query ─────────────────────────────────────────

size_t query_cub_normalize_temp_bytes(int max_n) {
    size_t temp_max = 0, temp_sum = 0;
    float* dummy = nullptr;

    cub::DeviceReduce::Max(nullptr, temp_max, dummy, dummy, max_n);
    cub::DeviceReduce::Sum(nullptr, temp_sum, dummy, dummy, max_n);

    return std::max(temp_max, temp_sum);
}

// ─── GPU-side normalize launch ──────────────────────────────────────

void launch_gpu_normalize_weights(
        const float* d_log_w,
        float* d_old_w,          // existing weights (read, then overwritten)
        float* d_scratch_w,      // scratch buffer (receives exp-shift-mul result)
        float* d_max_val,        // device scalar for CUB max output
        float* d_sum_val,        // device scalar for CUB sum output
        void* d_cub_temp,        // CUB temp storage
        size_t cub_temp_bytes,
        int n,
        cudaStream_t stream) {
    int block = 256;
    int grid  = (n + block - 1) / block;

    // Step 1: Find max log-weight (for numerical stability).
    cub::DeviceReduce::Max(d_cub_temp, cub_temp_bytes,
                           d_log_w, d_max_val, n, stream);

    // Step 2: exp(log_w[i] - max) * old_w[i]
    kernel_exp_shift_mul<<<grid, block, 0, stream>>>(
        d_log_w, d_old_w, d_scratch_w, d_max_val, n);
    CUDA_CHECK(cudaGetLastError());

    // Step 3: Sum of unnormalised weights.
    cub::DeviceReduce::Sum(d_cub_temp, cub_temp_bytes,
                           d_scratch_w, d_sum_val, n, stream);

    // Step 4: Normalize in-place.
    kernel_normalize<<<grid, block, 0, stream>>>(d_scratch_w, d_sum_val, n);
    CUDA_CHECK(cudaGetLastError());

    // d_scratch_w now holds the normalised weights.
    // Caller is responsible for swapping d_scratch_w ↔ d_old_w pointers
    // (or copying, but swap is preferred for zero overhead).
}

}  // namespace gpu_amcl_cpp
