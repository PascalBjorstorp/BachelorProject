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
    d_distance_field_.upload(df.data(), df.size());
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
        d_distance_field_.ptr(),
        map_width_, map_height_,
        map_res_, map_ox_, map_oy_,
        d_weights, stream);
}

}  // namespace gpu_amcl_cpp
