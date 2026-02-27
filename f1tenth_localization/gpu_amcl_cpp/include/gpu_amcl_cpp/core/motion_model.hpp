#pragma once

#include "gpu_amcl_cpp/helpers/cuda_utils.hpp"
#include <curand_kernel.h>

namespace gpu_amcl_cpp {

/**
 * @brief GPU-accelerated odometry motion model (Probabilistic Robotics §5.4).
 *
 * Applies noisy odometry deltas to all particles in parallel on the GPU.
 * Noise is parameterised by four alpha values loaded from YAML.
 */
class MotionModel {
public:
    struct Config {
        double alpha1 = 0.1;   ///< rotation  → rotation  noise
        double alpha2 = 0.1;   ///< translation → rotation  noise
        double alpha3 = 0.2;   ///< translation → translation noise
        double alpha4 = 0.2;   ///< rotation  → translation noise
        bool   use_imu = false;
        double imu_gyro_weight = 0.8;
    };

    MotionModel() = default;

    /// Initialise CUDA resources for `num_particles` particles.
    void init(int num_particles, const Config& cfg, unsigned long long seed = 42);

    /**
     * @brief Apply motion model to all particles on GPU.
     *
     * @param d_particles  Device pointer to Nx3 float array (x, y, θ).
     * @param n            Number of particles.
     * @param dx           Robot-frame forward displacement.
     * @param dy           Robot-frame lateral displacement.
     * @param dtheta       Robot-frame rotation.
     * @param imu_dtheta   IMU-measured rotation (ignored if !use_imu).
     * @param stream       CUDA stream.
     */
    void apply(float* d_particles, int n,
               float dx, float dy, float dtheta,
               float imu_dtheta,
               cudaStream_t stream = nullptr);

    /// Temporarily scale noise (e.g. during slip).
    void set_noise_multiplier(double mult) { noise_mult_ = mult; }
    void reset_noise_multiplier()          { noise_mult_ = 1.0; }

    /// Reallocate if particle count changed (KLD).
    void ensure_capacity(int n);

    Config& config() { return cfg_; }

private:
    Config                    cfg_;
    double                    noise_mult_ = 1.0;
    DeviceBuffer<curandState> d_rng_states_;
    int                       capacity_ = 0;
};

// ─── CUDA kernel declarations (defined in .cu) ─────────────────────
void launch_init_rng(curandState* states, int n,
                     unsigned long long seed,
                     cudaStream_t stream);

void launch_motion_update(float* particles, int n,
                          float dx, float dy, float dtheta,
                          float alpha1, float alpha2,
                          float alpha3, float alpha4,
                          bool use_imu, float imu_weight,
                          float imu_dtheta,
                          curandState* rng,
                          cudaStream_t stream);

}  // namespace gpu_amcl_cpp
