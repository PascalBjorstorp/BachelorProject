#pragma once

#include "gpu_amcl_cpp/helpers/cuda_utils.hpp"

namespace gpu_amcl_cpp {

/**
 * @brief GPU-accelerated low-variance (systematic) resampler.
 *
 * Also provides effective-sample-size computation used to decide
 * when resampling is necessary.
 */
class Resampler {
public:
    Resampler() = default;

    /// Pre-allocate scratch buffers for up to `max_particles` particles.
    void init(int max_particles);

    /**
     * @brief Resample particles in-place.
     *
     * Uses low-variance systematic resampling.
     * After resampling, weights are set to 1/n.
     *
     * @param d_particles  Device Nx3 float array.
     * @param d_weights    Device N float array (normalised).
     * @param n            Current particle count.
     * @param target_n     Target count after resampling (for KLD).
     *                     If <= 0, keeps n unchanged.
     * @param stream       CUDA stream.
     * @return             Number of particles after resampling.
     */
    int resample(float* d_particles, float* d_weights,
                 int n, int target_n = -1,
                 cudaStream_t stream = nullptr);

    /**
     * @brief Compute N_eff = 1 / Σ(w_i²).
     *
     * Performed on GPU.  Result is copied back to host.
     */
    double effective_sample_size(const float* d_weights, int n,
                                 cudaStream_t stream = nullptr);

    /// Ensure buffers are large enough for `n`.
    void ensure_capacity(int n);

private:
    DeviceBuffer<float> d_cumsum_;
    DeviceBuffer<float> d_new_particles_;
    DeviceBuffer<float> d_scratch_;  ///< for reductions
    int                 capacity_ = 0;
};

// ─── CUDA kernel declarations (defined in .cu) ─────────────────────
void launch_inclusive_scan(const float* weights, float* cumsum,
                           int n, cudaStream_t stream);

void launch_systematic_resample(const float* cumsum,
                                const float* old_particles,
                                float* new_particles,
                                float* new_weights,
                                int old_n, int new_n,
                                float random_offset,
                                cudaStream_t stream);

double launch_sum_sq_weights(const float* weights, int n,
                             float* d_scratch, cudaStream_t stream);

}  // namespace gpu_amcl_cpp
