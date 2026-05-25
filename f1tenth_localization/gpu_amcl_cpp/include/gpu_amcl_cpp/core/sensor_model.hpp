#pragma once

#include "gpu_amcl_cpp/helpers/cuda_utils.hpp"
#include "gpu_amcl_cpp/helpers/map_utils.hpp"
#include <vector>

namespace gpu_amcl_cpp {

/**
 * @brief GPU-accelerated likelihood-field sensor model
 *        (Probabilistic Robotics §6.4).
 *
 * Pre-computes a Euclidean distance field from the occupancy map,
 * uploads it to the GPU, then evaluates per-particle weights by
 * ray-casting into the field — fully parallel across particles × beams.
 */
class SensorModel {
public:
    struct Config {
        int    max_beams       = 270;
        double z_hit           = 0.95;
        double z_rand          = 0.05;
        double sigma_hit       = 0.2;
        double laser_max_range = 10.0;
        double laser_offset_x  = 0.265;
        double laser_offset_y  = 0.0;
        bool   normalize_likelihood_by_beams = true;
        double likelihood_scale = 1.0;
    };

    SensorModel() = default;

    /// Build the distance field from the host-side MapProcessor and upload.
    void init(const MapProcessor& map, const Config& cfg);

    /**
     * @brief Compute unnormalised log-weights for all particles.
     *
     * @param d_particles   Device Nx3 float array (x, y, θ).
     * @param n             Number of particles.
     * @param d_ranges      Device array of M range readings.
     * @param num_ranges    Total laser beams in the scan.
     * @param angle_min     Start angle of the scan (rad).
     * @param angle_inc     Angle increment per beam (rad).
     * @param d_weights     Device output array of N floats (log-weights).
     * @param stream        CUDA stream.
     */
    void compute_weights(const float* d_particles, int n,
                         const float* d_ranges, int num_ranges,
                         float angle_min, float angle_inc,
                         float* d_weights,
                         cudaStream_t stream = nullptr);

    Config& config() { return cfg_; }

private:
    Config              cfg_;
    DeviceBuffer<float> d_distance_field_;
    int                 map_width_  = 0;
    int                 map_height_ = 0;
    float               map_res_    = 0.0f;
    float               map_ox_     = 0.0f;
    float               map_oy_     = 0.0f;
};

// ─── CUDA kernel declarations (defined in .cu) ─────────────────────
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
                           cudaStream_t stream);

}  // namespace gpu_amcl_cpp
