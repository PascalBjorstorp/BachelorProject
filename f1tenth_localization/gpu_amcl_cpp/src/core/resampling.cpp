#include "gpu_amcl_cpp/core/resampling.hpp"

#include <random>

namespace gpu_amcl_cpp {

void Resampler::init(int max_particles) {
    capacity_ = max_particles;
    d_cumsum_.allocate(max_particles);
    d_new_particles_.allocate(max_particles * 3);
    d_scratch_.allocate(1);  // for reduction results
}

void Resampler::ensure_capacity(int n) {
    if (n <= capacity_) return;
    capacity_ = n;
    d_cumsum_.allocate(n);
    d_new_particles_.allocate(n * 3);
}

int Resampler::resample(float* d_particles, float* d_weights,
                        int n, int target_n,
                        cudaStream_t stream) {
    int new_n = (target_n > 0) ? target_n : n;
    ensure_capacity(std::max(n, new_n));

    // 1. Inclusive prefix-sum of weights.
    launch_inclusive_scan(d_weights, d_cumsum_.ptr(), n, stream);

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

double Resampler::effective_sample_size(const float* d_weights, int n,
                                        cudaStream_t stream) {
    double sum_sq = launch_sum_sq_weights(d_weights, n,
                                          d_scratch_.ptr(), stream);
    return (sum_sq > 0.0) ? 1.0 / sum_sq : static_cast<double>(n);
}

}  // namespace gpu_amcl_cpp
