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
                 double weight_power = 1.0,
                 double uniform_floor = 0.0,
                 cudaStream_t stream = nullptr);

    /**
     * @brief Resample particles to an external output buffer (§5 double-buffer).
     *
     * Writes resampled particles to d_out_particles (caller-owned).
     * Weights are reset to 1/n in d_weights.
     * No D→D copy or stream sync required — caller swaps pointers.
     *
     * @param d_particles      Device Nx3 float array (source).
     * @param d_weights        Device N float array (normalised, reset to 1/n).
     * @param d_out_particles  Device output Nx3 float array (caller-owned).
     * @param n                Current particle count.
     * @param target_n         Target count after resampling.
     * @param stream           CUDA stream.
     * @return                 Number of particles after resampling.
     */
    int resample_to(const float* d_particles, float* d_weights,
                    float* d_out_particles,
                    int n, int target_n = -1,
                    double weight_power = 1.0,
                    double uniform_floor = 0.0,
                    cudaStream_t stream = nullptr);

    /**
     * @brief Compute N_eff = 1 / Σ(w_i²).
     *
     * Performed on GPU using CUB DeviceReduce (§2).
     */
    double effective_sample_size(const float* d_weights, int n,
                                 cudaStream_t stream = nullptr);

private:
    DeviceBuffer<float> d_cumsum_;
    DeviceBuffer<float> d_resample_weights_;
    DeviceBuffer<float> d_new_particles_;
    DeviceBuffer<float> d_scratch_;  ///< for reduction results (1 float)
    int                 capacity_ = 0;

    // CUB temp storage for DeviceScan::InclusiveSum
    DeviceBuffer<uint8_t> d_scan_temp_;
    size_t                scan_temp_bytes_ = 0;

    // CUB temp storage for DeviceReduce::Sum (sum-of-squares)
    DeviceBuffer<uint8_t> d_sumsq_temp_;
    size_t                sumsq_temp_bytes_ = 0;

    // CUB temp storage for DeviceReduce::Sum (tempered resampling weights)
    DeviceBuffer<uint8_t> d_sum_temp_;
    size_t                sum_temp_bytes_ = 0;
};

// ─── CUDA kernel declarations (defined in .cu) ─────────────────────
// CUB temp-storage size queries
size_t query_scan_temp_bytes(int n);
size_t query_sum_temp_bytes(int n);
size_t query_sumsq_temp_bytes(int n);

double launch_prepare_resample_weights(const float* weights,
                                        float* out_weights,
                                        int n,
                                        float weight_power,
                                        float uniform_floor,
                                        float* d_result,
                                        void* d_temp,
                                        size_t temp_bytes,
                                        cudaStream_t stream);

void launch_inclusive_scan(const float* weights, float* cumsum,
                           int n,
                           void* d_temp, size_t temp_bytes,
                           cudaStream_t stream);

void launch_systematic_resample(const float* cumsum,
                                const float* old_particles,
                                float* new_particles,
                                float* new_weights,
                                int old_n, int new_n,
                                float total_weight,
                                float random_offset,
                                cudaStream_t stream);

double launch_sum_sq_weights(const float* weights, int n,
                             float* d_result,
                             void* d_temp, size_t temp_bytes,
                             cudaStream_t stream);

}  // namespace gpu_amcl_cpp
