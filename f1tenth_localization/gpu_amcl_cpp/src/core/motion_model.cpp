#include "gpu_amcl_cpp/core/motion_model.hpp"

namespace gpu_amcl_cpp {

void MotionModel::init(int num_particles, const Config& cfg,
                       unsigned long long seed) {
    cfg_      = cfg;
    capacity_ = num_particles;
    d_rng_states_.allocate(num_particles);
    launch_init_rng(d_rng_states_.ptr(), num_particles, seed, nullptr);
    CUDA_CHECK(cudaDeviceSynchronize());
}

void MotionModel::ensure_capacity(int n) {
    if (n <= capacity_) return;
    // Preserve existing states and extend.
    DeviceBuffer<curandState> new_states(n);
    if (capacity_ > 0) {
        CUDA_CHECK(cudaMemcpy(new_states.ptr(), d_rng_states_.ptr(),
                              capacity_ * sizeof(curandState),
                              cudaMemcpyDeviceToDevice));
    }
    // Initialise only the new states.
    launch_init_rng(new_states.ptr() + capacity_, n - capacity_,
                    42ULL + capacity_, nullptr);
    CUDA_CHECK(cudaDeviceSynchronize());
    d_rng_states_ = std::move(new_states);
    capacity_     = n;
}

void MotionModel::apply(float* d_particles, int n,
                        float dx, float dy, float dtheta,
                        float imu_dtheta,
                        cudaStream_t stream) {
    ensure_capacity(n);

    float a1 = static_cast<float>(cfg_.alpha1 * noise_mult_);
    float a2 = static_cast<float>(cfg_.alpha2 * noise_mult_);
    float a3 = static_cast<float>(cfg_.alpha3 * noise_mult_);
    float a4 = static_cast<float>(cfg_.alpha4 * noise_mult_);

    launch_motion_update(d_particles, n,
                         dx, dy, dtheta,
                         a1, a2, a3, a4,
                         cfg_.use_imu,
                         static_cast<float>(cfg_.imu_gyro_weight),
                         imu_dtheta,
                         d_rng_states_.ptr(),
                         stream);
}

}  // namespace gpu_amcl_cpp
