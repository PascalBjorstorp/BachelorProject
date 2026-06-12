#include "gpu_amcl_cpp/core/sensor_model.hpp"

namespace gpu_amcl_cpp {

void SensorModel::init(const MapProcessor& map, const Config& cfg) {
    cfg_        = cfg;
    map_width_  = map.width();
    map_height_ = map.height();
    map_res_    = map.resolution();
    map_ox_     = map.origin_x();
    map_oy_     = map.origin_y();

    // Upload distance field to GPU.
    const auto& df = map.distance_field();
    const auto& occupancy = map.occupancy();
    try {
        d_distance_field_.upload(df.data(), df.size());
        d_occupancy_.upload(occupancy.data(), occupancy.size());
    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("SensorModel::init GPU upload failed: ") + e.what());
    }
}

void SensorModel::compute_weights(const float* d_particles, int n,
                                  const float* d_ranges, int num_ranges,
                                  float angle_min, float angle_inc,
                                  float* d_weights,
                                  cudaStream_t stream) {
    launch_sensor_weights(
        d_particles, n,
        d_ranges, num_ranges,
        cfg_.max_beams,
        angle_min, angle_inc,
        static_cast<float>(cfg_.z_hit),
        static_cast<float>(cfg_.z_rand),
        static_cast<float>(cfg_.sigma_hit),
        static_cast<float>(cfg_.laser_max_range),
        static_cast<float>(cfg_.laser_offset_x),
        static_cast<float>(cfg_.laser_offset_y),
        cfg_.normalize_likelihood_by_beams,
        static_cast<float>(cfg_.likelihood_scale),
        d_distance_field_.ptr(),
        map_width_, map_height_,
        map_res_, map_ox_, map_oy_,
        d_weights, stream);
}

void SensorModel::compute_raycast_scores(const float* d_particles,
                                         const int* d_candidate_indices,
                                         int num_candidates,
                                         const float* d_ranges,
                                         int num_ranges,
                                         float angle_min,
                                         float angle_inc,
                                         int max_beams,
                                         float step_m,
                                         float* d_scores,
                                         int* d_counts,
                                         cudaStream_t stream) const {
    launch_raycast_scores(
        d_particles,
        d_candidate_indices,
        num_candidates,
        d_ranges,
        num_ranges,
        max_beams,
        angle_min,
        angle_inc,
        static_cast<float>(cfg_.z_hit),
        static_cast<float>(cfg_.z_rand),
        static_cast<float>(cfg_.sigma_hit),
        static_cast<float>(cfg_.laser_max_range),
        static_cast<float>(cfg_.laser_offset_x),
        static_cast<float>(cfg_.laser_offset_y),
        step_m,
        d_occupancy_.ptr(),
        map_width_,
        map_height_,
        map_res_,
        map_ox_,
        map_oy_,
        d_scores,
        d_counts,
        stream);
}

void SensorModel::refine_startup_particles(float* d_particles,
                                           int n,
                                           const float* d_ranges,
                                           int num_ranges,
                                           float angle_min,
                                           float angle_inc,
                                           int max_beams,
                                           int iterations,
                                           float max_match_distance_m,
                                           float max_translation_m,
                                           float max_yaw_rad,
                                           float max_step_translation_m,
                                           float max_step_yaw_rad,
                                           float* d_scores,
                                           int* d_counts,
                                           cudaStream_t stream) const {
    launch_startup_scan_refinement(
        d_particles,
        n,
        d_ranges,
        num_ranges,
        max_beams,
        angle_min,
        angle_inc,
        static_cast<float>(cfg_.laser_max_range),
        static_cast<float>(cfg_.laser_offset_x),
        static_cast<float>(cfg_.laser_offset_y),
        d_distance_field_.ptr(),
        d_occupancy_.ptr(),
        map_width_,
        map_height_,
        map_res_,
        map_ox_,
        map_oy_,
        iterations,
        max_match_distance_m,
        max_translation_m,
        max_yaw_rad,
        max_step_translation_m,
        max_step_yaw_rad,
        d_scores,
        d_counts,
        stream);
}

}  // namespace gpu_amcl_cpp
