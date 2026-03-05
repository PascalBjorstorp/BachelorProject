#include "gpu_amcl_cpp/core/resampling.hpp"

#include <random>

namespace gpu_amcl_cpp {

void Resampler::init(int max_particles) {
    capacity_ = max_particles;
    d_cumsum_.allocate(max_particles);
    d_new_particles_.allocate(max_particles * 3);
    d_scratch_.allocate(1);  // for reduction results

    // §2: Allocate CUB temp storage for InclusiveSum
    scan_temp_bytes_ = query_scan_temp_bytes(max_particles);
    d_scan_temp_.allocate(scan_temp_bytes_);

    // §2: Allocate CUB temp storage for sum-of-squares (DeviceReduce::Sum)
    sumsq_temp_bytes_ = query_sumsq_temp_bytes(max_particles);
    d_sumsq_temp_.allocate(sumsq_temp_bytes_);
}

void Resampler::ensure_capacity(int n) {
    if (n <= capacity_) return;
    capacity_ = n;
    d_cumsum_.allocate(n);
    d_new_particles_.allocate(n * 3);

    // §2: Reallocate CUB temp storage for larger N
    size_t new_scan = query_scan_temp_bytes(n);
    if (new_scan > scan_temp_bytes_) {
        scan_temp_bytes_ = new_scan;
        d_scan_temp_.allocate(scan_temp_bytes_);
    }
    size_t new_sumsq = query_sumsq_temp_bytes(n);
    if (new_sumsq > sumsq_temp_bytes_) {
        sumsq_temp_bytes_ = new_sumsq;
        d_sumsq_temp_.allocate(sumsq_temp_bytes_);
    }
}

int Resampler::resample(float* d_particles, float* d_weights,
                        int n, int target_n,
                        cudaStream_t stream) {
    int new_n = (target_n > 0) ? target_n : n;
    ensure_capacity(std::max(n, new_n));

    // 1. Inclusive prefix-sum of weights (§2: CUB DeviceScan).
    launch_inclusive_scan(d_weights, d_cumsum_.ptr(), n,
                         d_scan_temp_.ptr(), scan_temp_bytes_, stream);

    // 2. Draw single random offset ∈ [0, 1/new_n).
    static thread_local std::mt19937 rng(42);
    std::uniform_real_distribution<float> dist(0.0f,
                                               1.0f / static_cast<float>(new_n));
    float r = dist(rng);

    // 3. Systematic resample on GPU.
    launch_systematic_resample(
        d_cumsum_.ptr(), d_particles,
        d_new_particles_.ptr(), d_weights,
        n, new_n, r, stream);

    // 4. Copy new particles back into the main buffer.
    CUDA_CHECK(cudaMemcpyAsync(d_particles, d_new_particles_.ptr(),
                               new_n * 3 * sizeof(float),
                               cudaMemcpyDeviceToDevice, stream));

    CUDA_CHECK(cudaStreamSynchronize(stream));
    return new_n;
}

int Resampler::resample_to(const float* d_particles, float* d_weights,
                           float* d_out_particles,
                           int n, int target_n,
                           cudaStream_t stream) {
    int new_n = (target_n > 0) ? target_n : n;
    ensure_capacity(std::max(n, new_n));

    // 1. Inclusive prefix-sum of weights (§2: CUB DeviceScan).
    launch_inclusive_scan(d_weights, d_cumsum_.ptr(), n,
                         d_scan_temp_.ptr(), scan_temp_bytes_, stream);

    // 2. Draw single random offset ∈ [0, 1/new_n).
    static thread_local std::mt19937 rng(42);
    std::uniform_real_distribution<float> dist(0.0f,
                                               1.0f / static_cast<float>(new_n));
    float r = dist(rng);

    // 3. Systematic resample on GPU — write directly to caller's output buffer.
    launch_systematic_resample(
        d_cumsum_.ptr(), d_particles,
        d_out_particles, d_weights,
        n, new_n, r, stream);

    // §5: No D→D copy needed — caller swaps pointers (double-buffer).
    // Still need sync so weights (1/n) are visible before next step.
    CUDA_CHECK(cudaStreamSynchronize(stream));
    return new_n;
}

double Resampler::effective_sample_size(const float* d_weights, int n,
                                        cudaStream_t stream) {
    // §2: CUB DeviceReduce::Sum with TransformInputIterator (square + sum)
    double sum_sq = launch_sum_sq_weights(d_weights, n,
                                          d_scratch_.ptr(),
                                          d_sumsq_temp_.ptr(),
                                          sumsq_temp_bytes_, stream);
    return (sum_sq > 0.0) ? 1.0 / sum_sq : static_cast<double>(n);
}

}  // namespace gpu_amcl_cpp
