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
#include <cstdint>

namespace gpu_amcl_cpp {

__global__
void kernel_sensor_weights(const float* __restrict__ particles, int n,
                           const float* __restrict__ ranges, int num_ranges,
                           int max_beams,
                           float angle_min, float angle_inc,
                           float z_hit, float z_rand,
                           float sigma_hit, float laser_max,
                           float laser_ox, float laser_oy,
                           bool normalize_likelihood_by_beams,
                           float likelihood_scale,
                           const float* __restrict__ dist_field,
                           int map_w, int map_h,
                           float map_res, float map_ox, float map_oy,
                           float* __restrict__ out_weights) {
    // §8: Load ranges + beam angles into shared memory cooperatively.
    // All threads in a block read the same ranges[b] — shared memory gives
    // ~6× lower latency (5 cycles vs ~30 for L1 hit) and frees L1 cache
    // for distance-field lookups. 270 beams × 4B × 2 arrays = ~2 KB.
    extern __shared__ float smem[];
    float* s_ranges = smem;
    float* s_cos    = &smem[num_ranges];
    float* s_sin    = &smem[2 * num_ranges];
    for (int j = threadIdx.x; j < num_ranges; j += blockDim.x) {
        float a = angle_min + j * angle_inc;
        s_ranges[j] = ranges[j];
        s_cos[j]    = cosf(a);  // precompute beam cosines once per block
        s_sin[j]    = sinf(a);  // precompute beam sines once per block
    }
    __syncthreads(); // ensures all threads in the block see fully loaded arrays before using them.

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
    int valid_beams = 0;

    for (int b = 0; b < num_ranges; b += step) {
        float r = s_ranges[b];  // §8: read from shared memory

        // Skip invalid beams.
        if (r < 0.1f || r > laser_max) continue;
        ++valid_beams;

        float cb = s_cos[b];
        float sb = s_sin[b];
        float beam_cos = cb * ct - sb * st;
        float beam_sin = sb * ct + cb * st;
        float ex = lx + r * beam_cos;
        float ey = ly + r * beam_sin;

        // World → continuous map coordinates for bilinear interpolation.
        float fx = (ex - map_ox) / map_res - 0.5f;
        float fy = (ey - map_oy) / map_res - 0.5f;

        float dist;
        // Check bounds (need fx in [-0.5, map_w-0.5) for valid interpolation)
        if (fx >= -0.5f && fx < (float)(map_w) - 0.5f &&
            fy >= -0.5f && fy < (float)(map_h) - 0.5f) {
            // Bilinear interpolation of the distance field.
            int x0 = __float2int_rd(fx);  // floor
            int y0 = __float2int_rd(fy);
            int x1 = x0 + 1;
            int y1 = y0 + 1;
            float sx = fx - (float)x0;    // fractional part [0,1)
            float sy = fy - (float)y0;

            // Clamp to valid grid range [0, dim-1]
            x0 = max(0, min(x0, map_w - 1));
            x1 = max(0, min(x1, map_w - 1));
            y0 = max(0, min(y0, map_h - 1));
            y1 = max(0, min(y1, map_h - 1));

            // §7: Use __ldg() to load through the read-only data cache.
            // The texture/RO cache (48 KB on sm_87/89) is separate from L1,
            // effectively increasing total cache capacity and reducing
            // eviction of other data (particles, ranges). Guarantees LDG
            // instruction even if compiler doesn't auto-detect const.
            float d00 = __ldg(&dist_field[y0 * map_w + x0]);
            float d10 = __ldg(&dist_field[y0 * map_w + x1]);
            float d01 = __ldg(&dist_field[y1 * map_w + x0]);
            float d11 = __ldg(&dist_field[y1 * map_w + x1]);

            // Bilinear blend
            float d0 = d00 + sx * (d10 - d00);   // top edge
            float d1 = d01 + sx * (d11 - d01);   // bottom edge
            dist = d0 + sy * (d1 - d0);           // vertical blend
        } else {
            dist = laser_max;  // out-of-bounds → max distance
        }

        // Likelihood: z_hit * N(dist; 0, σ) + z_rand / laser_max
        float p = z_hit * norm * expf(inv_2sig2 * dist * dist)
                + z_rand / laser_max;

        log_w += logf(fmaxf(p, 1e-30f));
    }

    if (normalize_likelihood_by_beams && valid_beams > 0) {
        log_w /= static_cast<float>(valid_beams);
    }
    out_weights[i] = log_w * fmaxf(likelihood_scale, 0.0f);
}

void launch_sensor_weights(const float* particles, int n,
                           const float* ranges, int num_ranges,
                           int max_beams,
                           float angle_min, float angle_inc,
                           float z_hit, float z_rand,
                           float sigma_hit, float laser_max_range,
                           float laser_offset_x, float laser_offset_y,
                           bool normalize_likelihood_by_beams,
                           float likelihood_scale,
                           const float* distance_field,
                           int map_w, int map_h,
                           float map_res, float map_ox, float map_oy,
                           float* out_weights,
                           cudaStream_t stream) {
    if (n <= 0 || num_ranges <= 0) return;
    const auto launch = make_adaptive_launch_config(n);

    // §8: Shared memory for ranges + precomputed beam cos/sin (3 arrays of num_ranges floats)
    size_t smem_bytes = 3 * num_ranges * sizeof(float);
    kernel_sensor_weights<<<launch.grid, launch.block, smem_bytes, stream>>>(
        particles, n,
        ranges, num_ranges, max_beams,
        angle_min, angle_inc,
        z_hit, z_rand, sigma_hit,
        laser_max_range, laser_offset_x, laser_offset_y,
        normalize_likelihood_by_beams, likelihood_scale,
        distance_field, map_w, map_h,
        map_res, map_ox, map_oy,
        out_weights);
    CUDA_CHECK(cudaGetLastError());
}

__device__ __forceinline__
float angle_diff_rad(float a, float b) {
    const float d = a - b;
    return atan2f(sinf(d), cosf(d));
}

__device__ __forceinline__
bool map_occupied_or_out(float wx,
                         float wy,
                         const int8_t* __restrict__ occupancy,
                         int map_w,
                         int map_h,
                         float map_res,
                         float map_ox,
                         float map_oy,
                         bool* out_of_bounds) {
    const int mx = __float2int_rd((wx - map_ox) / map_res);
    const int my = __float2int_rd((wy - map_oy) / map_res);
    if (mx < 0 || mx >= map_w || my < 0 || my >= map_h) {
        *out_of_bounds = true;
        return true;
    }
    *out_of_bounds = false;
    return occupancy[my * map_w + mx] >= 50;
}

__device__
float raycast_expected_range(float lx,
                             float ly,
                             float beam_cos,
                             float beam_sin,
                             float laser_max,
                             float step_m,
                             const int8_t* __restrict__ occupancy,
                             int map_w,
                             int map_h,
                             float map_res,
                             float map_ox,
                             float map_oy) {
    const float ray_step = fmaxf(step_m, fmaxf(0.01f, map_res * 0.5f));
    const int max_steps = max(1, static_cast<int>(ceilf(laser_max / ray_step)));
    for (int step = 1; step <= max_steps; ++step) {
        const float t = fminf(static_cast<float>(step) * ray_step, laser_max);
        const float wx = lx + t * beam_cos;
        const float wy = ly + t * beam_sin;
        bool out_of_bounds = false;
        if (map_occupied_or_out(wx, wy, occupancy, map_w, map_h,
                                map_res, map_ox, map_oy, &out_of_bounds)) {
            return out_of_bounds ? laser_max : t;
        }
    }
    return laser_max;
}

__global__
void kernel_raycast_scores(const float* __restrict__ particles,
                           const int* __restrict__ candidate_indices,
                           int num_candidates,
                           const float* __restrict__ ranges,
                           int num_ranges,
                           int beam_step,
                           int beam_count,
                           float angle_min,
                           float angle_inc,
                           float z_hit,
                           float z_rand,
                           float sigma_hit,
                           float laser_max,
                           float laser_ox,
                           float laser_oy,
                           float step_m,
                           const int8_t* __restrict__ occupancy,
                           int map_w,
                           int map_h,
                           float map_res,
                           float map_ox,
                           float map_oy,
                           float* __restrict__ out_scores,
                           int* __restrict__ out_counts) {
    const int work = blockIdx.x * blockDim.x + threadIdx.x;
    const int total = num_candidates * beam_count;
    if (work >= total) return;

    const int candidate = work / beam_count;
    const int beam_slot = work - candidate * beam_count;
    const int b = beam_slot * beam_step;
    if (candidate >= num_candidates || b >= num_ranges) return;

    const float observed = ranges[b];
    if (!isfinite(observed) || observed < 0.1f || observed > laser_max) {
        return;
    }

    const int particle_index = candidate_indices[candidate];
    const float px = particles[particle_index * 3 + 0];
    const float py = particles[particle_index * 3 + 1];
    const float ptheta = particles[particle_index * 3 + 2];
    const float ct = cosf(ptheta);
    const float st = sinf(ptheta);

    const float lx = px + laser_ox * ct - laser_oy * st;
    const float ly = py + laser_ox * st + laser_oy * ct;

    const float beam_angle = angle_min + static_cast<float>(b) * angle_inc;
    const float cb = cosf(beam_angle);
    const float sb = sinf(beam_angle);
    const float beam_cos = cb * ct - sb * st;
    const float beam_sin = sb * ct + cb * st;

    const float expected = raycast_expected_range(
        lx, ly, beam_cos, beam_sin, laser_max, step_m,
        occupancy, map_w, map_h, map_res, map_ox, map_oy);

    const float error = observed - expected;
    const float inv_2sig2 = -0.5f / (sigma_hit * sigma_hit);
    const float norm = 1.0f / (sqrtf(2.0f * 3.14159265f) * sigma_hit);
    const float p = z_hit * norm * expf(inv_2sig2 * error * error)
                  + z_rand / laser_max;
    atomicAdd(&out_scores[candidate], logf(fmaxf(p, 1e-30f)));
    atomicAdd(&out_counts[candidate], 1);
}

void launch_raycast_scores(const float* particles,
                           const int* candidate_indices,
                           int num_candidates,
                           const float* ranges,
                           int num_ranges,
                           int max_beams,
                           float angle_min, float angle_inc,
                           float z_hit, float z_rand,
                           float sigma_hit, float laser_max_range,
                           float laser_offset_x, float laser_offset_y,
                           float step_m,
                           const int8_t* occupancy,
                           int map_w, int map_h,
                           float map_res, float map_ox, float map_oy,
                           float* out_scores,
                           int* out_counts,
                           cudaStream_t stream) {
    if (num_candidates <= 0 || num_ranges <= 0 || max_beams <= 0) return;

    CUDA_CHECK(cudaMemsetAsync(out_scores, 0, num_candidates * sizeof(float), stream));
    CUDA_CHECK(cudaMemsetAsync(out_counts, 0, num_candidates * sizeof(int), stream));

    const int beam_step = max(1, num_ranges / max_beams);
    const int beam_count = max(1, (num_ranges + beam_step - 1) / beam_step);
    const int total = num_candidates * beam_count;
    const auto launch = make_adaptive_launch_config(total);
    kernel_raycast_scores<<<launch.grid, launch.block, 0, stream>>>(
        particles,
        candidate_indices,
        num_candidates,
        ranges,
        num_ranges,
        beam_step,
        beam_count,
        angle_min,
        angle_inc,
        z_hit,
        z_rand,
        sigma_hit,
        laser_max_range,
        laser_offset_x,
        laser_offset_y,
        step_m,
        occupancy,
        map_w,
        map_h,
        map_res,
        map_ox,
        map_oy,
        out_scores,
        out_counts);
    CUDA_CHECK(cudaGetLastError());
}

__global__
void kernel_apply_cluster_log_factors(const float* __restrict__ particles,
                                      int n,
                                      const float* __restrict__ cluster_centers,
                                      const float* __restrict__ cluster_log_factors,
                                      int num_clusters,
                                      float radius2,
                                      float yaw_tolerance,
                                      float* __restrict__ out_log_factors) {
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;

    const float x = particles[i * 3 + 0];
    const float y = particles[i * 3 + 1];
    const float yaw = particles[i * 3 + 2];
    float best_d2 = radius2;
    float factor = 0.0f;

    for (int k = 0; k < num_clusters; ++k) {
        const float cx = cluster_centers[k * 3 + 0];
        const float cy = cluster_centers[k * 3 + 1];
        const float cyaw = cluster_centers[k * 3 + 2];
        const float dx = x - cx;
        const float dy = y - cy;
        const float d2 = dx * dx + dy * dy;
        const float dyaw = fabsf(angle_diff_rad(yaw, cyaw));
        if (d2 <= best_d2 && dyaw <= yaw_tolerance) {
            best_d2 = d2;
            factor = cluster_log_factors[k];
        }
    }

    out_log_factors[i] = factor;
}

void launch_apply_cluster_log_factors(const float* particles,
                                      int n,
                                      const float* cluster_centers,
                                      const float* cluster_log_factors,
                                      int num_clusters,
                                      float radius2,
                                      float yaw_tolerance,
                                      float* out_log_factors,
                                      cudaStream_t stream) {
    if (n <= 0) return;
    if (num_clusters <= 0) {
        CUDA_CHECK(cudaMemsetAsync(out_log_factors, 0, n * sizeof(float), stream));
        return;
    }
    const auto launch = make_adaptive_launch_config(n);
    kernel_apply_cluster_log_factors<<<launch.grid, launch.block, 0, stream>>>(
        particles,
        n,
        cluster_centers,
        cluster_log_factors,
        num_clusters,
        radius2,
        yaw_tolerance,
        out_log_factors);
    CUDA_CHECK(cudaGetLastError());
}

}  // namespace gpu_amcl_cpp
