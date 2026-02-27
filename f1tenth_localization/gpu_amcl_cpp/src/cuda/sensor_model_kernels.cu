/**
 * @file sensor_model_kernels.cu
 * @brief CUDA kernels for the likelihood-field sensor model.
 *
 * Each thread handles one particle; it iterates over the subsampled
 * beams, transforms endpoints to world coordinates, looks up the
 * distance field, and accumulates a log-likelihood.
 */

#include "gpu_amcl_cpp/helpers/cuda_utils.hpp"
#include <cmath>

namespace gpu_amcl_cpp {

__global__
void kernel_sensor_weights(const float* __restrict__ particles, int n,
                           const float* __restrict__ ranges, int num_ranges,
                           int max_beams,
                           float angle_min, float angle_inc,
                           float z_hit, float z_rand,
                           float sigma_hit, float laser_max,
                           float laser_ox, float laser_oy,
                           const float* __restrict__ dist_field,
                           int map_w, int map_h,
                           float map_res, float map_ox, float map_oy,
                           float* __restrict__ out_weights) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;

    float px     = particles[i * 3 + 0];
    float py     = particles[i * 3 + 1];
    float ptheta = particles[i * 3 + 2];

    float ct = cosf(ptheta);
    float st = sinf(ptheta);

    // Laser origin in world frame (apply offset).
    float lx = px + laser_ox * ct - laser_oy * st;
    float ly = py + laser_ox * st + laser_oy * ct;

    // Pre-compute Gaussian terms.
    float inv_2sig2 = -0.5f / (sigma_hit * sigma_hit);
    float norm      = 1.0f / (sqrtf(2.0f * 3.14159265f) * sigma_hit);

    // Beam subsampling step.
    int step = max(1, num_ranges / max_beams);

    float log_w = 0.0f;

    for (int b = 0; b < num_ranges; b += step) {
        float r = ranges[b];

        // Skip invalid beams.
        if (r < 0.1f || r > laser_max) continue;

        float beam_angle = angle_min + b * angle_inc + ptheta;
        float ex = lx + r * cosf(beam_angle);
        float ey = ly + r * sinf(beam_angle);

        // World → map grid.
        int mx = __float2int_rd((ex - map_ox) / map_res);
        int my = __float2int_rd((ey - map_oy) / map_res);

        float dist;
        if (mx >= 0 && mx < map_w && my >= 0 && my < map_h) {
            dist = dist_field[my * map_w + mx];
        } else {
            dist = laser_max;  // out-of-bounds → max distance
        }

        // Likelihood: z_hit * N(dist; 0, σ) + z_rand / laser_max
        float p = z_hit * norm * expf(inv_2sig2 * dist * dist)
                + z_rand / laser_max;

        log_w += logf(fmaxf(p, 1e-30f));
    }

    out_weights[i] = log_w;
}

void launch_sensor_weights(const float* particles, int n,
                           const float* ranges, int num_ranges,
                           int max_beams,
                           float angle_min, float angle_inc,
                           float z_hit, float z_rand,
                           float sigma_hit, float laser_max_range,
                           float laser_offset_x, float laser_offset_y,
                           const float* distance_field,
                           int map_w, int map_h,
                           float map_res, float map_ox, float map_oy,
                           float* out_weights,
                           cudaStream_t stream) {
    int block = 256;
    int grid  = (n + block - 1) / block;
    kernel_sensor_weights<<<grid, block, 0, stream>>>(
        particles, n,
        ranges, num_ranges, max_beams,
        angle_min, angle_inc,
        z_hit, z_rand, sigma_hit,
        laser_max_range, laser_offset_x, laser_offset_y,
        distance_field, map_w, map_h,
        map_res, map_ox, map_oy,
        out_weights);
    CUDA_CHECK(cudaGetLastError());
}

}  // namespace gpu_amcl_cpp
