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

    // curand_init(seed, sequence, offset, state)
    curand_init(seed, i, 0, &states[i]);
}

void launch_init_rng(curandState* states, int n,
                     unsigned long long seed,
                     cudaStream_t stream) {
    if (n <= 0) return;
    const auto launch = make_adaptive_launch_config(n);
    kernel_init_rng<<<launch.grid, launch.block, 0, stream>>>(states, n, seed);
    CUDA_CHECK(cudaGetLastError());
}

// ─── Motion update ──────────────────────────────────────────────────
/**
 * Probabilistic Robotics sample-based odometry model.
 *
 * particles[i*3+0] = x,  particles[i*3+1] = y,  particles[i*3+2] = θ
 */
__global__
void kernel_motion_update(float* particles, int n,
                          float dx, float dy, float dtheta,
                          float a1, float a2, float a3, float a4,
                          curandState* rng) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    
    // Copy RNG state to local memory (faster access)
    curandState local_rng = rng[i];

    // Compute translation magnitude
    float trans = sqrtf(dx * dx + dy * dy);

    // ─── Noise standard deviations (Probabilistic Robotics) ───
    float sig_rot   = sqrtf(a1 * dtheta * dtheta + a2 * trans * trans);    // rotation noise depends on rotation and translation
    float sig_trans = sqrtf(a3 * trans * trans + a4 * dtheta * dtheta);    // translation noise depends on translation and rotation

    // Clamp to avoid zero variance (degenerate case)
    sig_rot   = fmaxf(sig_rot, 1e-6f);
    sig_trans = fmaxf(sig_trans, 1e-6f);

    // ─── Sample Gaussian noise ───
    float rot_noise     = curand_normal(&local_rng) * sig_rot;          // First rotation noise (mainly affects orientation)
    float trans_x_noise = curand_normal(&local_rng) * sig_trans;        // First translation noise (mainly affects x)
    float trans_y_noise = curand_normal(&local_rng) * sig_trans * 0.1f; // Lateral translation noise (smaller, affects y)
    float rot2_noise    = curand_normal(&local_rng) * sig_rot * 0.5f;   // Second rotation noise (smaller, accounts for additional uncertainty)

    // ─── Apply noise to odometry delta ───
    float noisy_dx = dx + trans_x_noise;
    float noisy_dy = dy + trans_y_noise;
    float noisy_dt = dtheta + rot_noise + rot2_noise;

    // ─── Transform robot-frame delta to world frame ───
    float theta = particles[i * 3 + 2];
    float ct    = cosf(theta);
    float st    = sinf(theta);

    // Rotation matrix: [cos -sin; sin cos] * [dx; dy]
    particles[i * 3 + 0] += noisy_dx * ct - noisy_dy * st;  // Update x
    particles[i * 3 + 1] += noisy_dx * st + noisy_dy * ct;  // Update y
    particles[i * 3 + 2] += noisy_dt;                       // Update θ

    // Normalise angle to [-π, π].
    float a = particles[i * 3 + 2];
    particles[i * 3 + 2] = atan2f(sinf(a), cosf(a));

    // Save RNG state back for next call
    rng[i] = local_rng;
}

void launch_motion_update(float* particles, int n,
                          float dx, float dy, float dtheta,
                          float alpha1, float alpha2,
                          float alpha3, float alpha4,
                          curandState* rng,
                          cudaStream_t stream) {
    if (n <= 0) return;
    const auto launch = make_adaptive_launch_config(n);
    kernel_motion_update<<<launch.grid, launch.block, 0, stream>>>(
        particles, n, dx, dy, dtheta,
        alpha1, alpha2, alpha3, alpha4, rng);
    CUDA_CHECK(cudaGetLastError());
}

}  // namespace gpu_amcl_cpp
