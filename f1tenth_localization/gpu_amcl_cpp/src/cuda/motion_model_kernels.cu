/**
 * @file motion_model_kernels.cu
 * @brief CUDA kernels for the odometry motion model.
 *
 * Each particle is independent — perfect for GPU parallelism.
 */

#include "gpu_amcl_cpp/helpers/cuda_utils.hpp"
#include <curand_kernel.h>
#include <cmath>

namespace gpu_amcl_cpp {

// ─── RNG initialisation ─────────────────────────────────────────────
__global__
void kernel_init_rng(curandState* states, int n,
                     unsigned long long seed) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    curand_init(seed, i, 0, &states[i]);
}

void launch_init_rng(curandState* states, int n,
                     unsigned long long seed,
                     cudaStream_t stream) {
    int block = 256;
    int grid  = (n + block - 1) / block;
    kernel_init_rng<<<grid, block, 0, stream>>>(states, n, seed);
    CUDA_CHECK(cudaGetLastError());
}

// ─── Motion update ──────────────────────────────────────────────────
/**
 * Probabilistic Robotics §5.4, sample-based odometry model.
 *
 * particles[i*3+0] = x,  particles[i*3+1] = y,  particles[i*3+2] = θ
 */
__global__
void kernel_motion_update(float* particles, int n,
                          float dx, float dy, float dtheta,
                          float a1, float a2, float a3, float a4,
                          bool use_imu, float imu_w, float imu_dtheta,
                          curandState* rng) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;

    curandState local_rng = rng[i];

    // Optionally fuse IMU rotation.
    float dt = dtheta;
    if (use_imu) {
        dt = imu_w * imu_dtheta + (1.0f - imu_w) * dtheta;
    }

    float trans = sqrtf(dx * dx + dy * dy);

    // Noise standard deviations.
    float sig_rot   = sqrtf(a1 * dt * dt + a2 * trans * trans);
    float sig_trans = sqrtf(a3 * trans * trans + a4 * dt * dt);

    // Clamp minimum noise to avoid degenerate distributions.
    sig_rot   = fmaxf(sig_rot, 1e-6f);
    sig_trans = fmaxf(sig_trans, 1e-6f);

    // Sample noise.
    float rot_noise     = curand_normal(&local_rng) * sig_rot;
    float trans_x_noise = curand_normal(&local_rng) * sig_trans;
    float trans_y_noise = curand_normal(&local_rng) * sig_trans * 0.1f;
    float rot2_noise    = curand_normal(&local_rng) * sig_rot * 0.5f;

    // Noisy delta in robot frame.
    float noisy_dx = dx + trans_x_noise;
    float noisy_dy = dy + trans_y_noise;
    float noisy_dt = dt + rot_noise + rot2_noise;

    // Transform to world frame.
    float theta = particles[i * 3 + 2];
    float ct    = cosf(theta);
    float st    = sinf(theta);

    particles[i * 3 + 0] += noisy_dx * ct - noisy_dy * st;
    particles[i * 3 + 1] += noisy_dx * st + noisy_dy * ct;
    particles[i * 3 + 2] += noisy_dt;

    // Normalise angle to [-π, π].
    float a = particles[i * 3 + 2];
    particles[i * 3 + 2] = atan2f(sinf(a), cosf(a));

    rng[i] = local_rng;
}

void launch_motion_update(float* particles, int n,
                          float dx, float dy, float dtheta,
                          float alpha1, float alpha2,
                          float alpha3, float alpha4,
                          bool use_imu, float imu_weight,
                          float imu_dtheta,
                          curandState* rng,
                          cudaStream_t stream) {
    int block = 256;
    int grid  = (n + block - 1) / block;
    kernel_motion_update<<<grid, block, 0, stream>>>(
        particles, n, dx, dy, dtheta,
        alpha1, alpha2, alpha3, alpha4,
        use_imu, imu_weight, imu_dtheta, rng);
    CUDA_CHECK(cudaGetLastError());
}

}  // namespace gpu_amcl_cpp
