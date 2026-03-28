#include "gpu_amcl_cpp/core/motion_model.hpp"

namespace gpu_amcl_cpp {

void MotionModel::init(int max_particles, const Config& cfg,
                       unsigned long long seed) {
    cfg_ = cfg;

    // Allocate RNG states for max_particles upfront (no dynamic growth needed)
    d_rng_states_.allocate(max_particles);
    launch_init_rng(d_rng_states_.ptr(), max_particles, seed, nullptr);
    CUDA_CHECK(cudaDeviceSynchronize());
}

void MotionModel::apply(float* d_particles, int n,
                        float dx, float dy, float dtheta,
                        cudaStream_t stream) {
    // Scale alpha values by noise multiplier
    float a1 = static_cast<float>(cfg_.alpha1 * noise_mult_);
    float a2 = static_cast<float>(cfg_.alpha2 * noise_mult_);
    float a3 = static_cast<float>(cfg_.alpha3 * noise_mult_);
    float a4 = static_cast<float>(cfg_.alpha4 * noise_mult_);

    // Launch GPU kernel to update all particles in parallel
    launch_motion_update(d_particles, n,
                         dx, dy, dtheta,
                         a1, a2, a3, a4,
                         d_rng_states_.ptr(),
                         stream);
}

}  // namespace gpu_amcl_cpp
