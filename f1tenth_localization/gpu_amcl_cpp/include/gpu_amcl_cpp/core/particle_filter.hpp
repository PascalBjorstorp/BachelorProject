#pragma once

#include "gpu_amcl_cpp/core/motion_model.hpp"
#include "gpu_amcl_cpp/core/sensor_model.hpp"
#include "gpu_amcl_cpp/core/resampling.hpp"
#include "gpu_amcl_cpp/helpers/cuda_utils.hpp"
#include "gpu_amcl_cpp/helpers/map_utils.hpp"

#include <Eigen/Core>
#include <vector>
#include <random>

namespace gpu_amcl_cpp {

// ─── CUDA kernel declarations for GPU-side weight normalization (§1) ──
size_t query_cub_normalize_temp_bytes(int max_n);

void launch_gpu_normalize_weights(
        const float* d_log_w,
        float* d_old_w,
        float* d_scratch_w,
        float* d_max_val,
        float* d_sum_val,
        void* d_cub_temp,
        size_t cub_temp_bytes,
        int n,
        cudaStream_t stream);

/**
 * @brief Pose estimate with 3×3 covariance [x, y, θ].
 */
struct PoseEstimate {
    double x = 0.0, y = 0.0, theta = 0.0;
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
};

/**
 * @brief GPU-accelerated particle filter orchestrator.
 *
 * Owns the particle/weight arrays on the GPU and delegates
 * prediction, measurement update and resampling to the
 * respective classes.
 */
class ParticleFilter {
public:
    struct Config {
        int    num_particles       = 2000;
        int    min_particles       = 100;
        int    max_particles       = 5000;

        // Initial pose
        double init_x  = 0.0; 
        double init_y  = 0.0;
        double init_a  = 0.0;
        double init_cov_xx = 0.5; 
        double init_cov_yy = 0.5; 
        double init_cov_aa = 0.2;

        // Resampling
        double resample_threshold   = 0.5;

        // KLD adaptive sampling
        bool   use_kld              = false;
        double kld_epsilon          = 0.05;
        double kld_z                = 2.33;
        double kld_bin_x            = 0.5;  ///< metres
        double kld_bin_y            = 0.5;
        double kld_bin_theta        = 0.1;  ///< radians
    };

    ParticleFilter() = default;
    ~ParticleFilter();

    // Non-copyable (owns GPU + pinned memory)
    ParticleFilter(const ParticleFilter&) = delete;
    ParticleFilter& operator=(const ParticleFilter&) = delete;

    /**
     * @brief Initialise particles and sub-components.
     *
     * Must be called after the map is available.
     */
    void init(const Config& pf_cfg,
              const MotionModel::Config& mm_cfg,
              const SensorModel::Config& sm_cfg,
              const MapProcessor& map);

    /// Re-initialise around a given pose (e.g. from /initialpose).
    void reinitialize(double x, double y, double theta,
                      double cov_xx, double cov_yy, double cov_aa);

    /// Prediction step: propagate particles by odom delta.
    void predict(float dx, float dy, float dtheta);

    /// Update step: compute particle weights from a new scan.
    void update(const float* ranges, int num_ranges,
                float angle_min, float angle_inc);

    /// Get the weighted-mean pose estimate (computed on CPU).
    PoseEstimate get_estimate();

    /// Download particles + weights to host for visualisation.
    void get_particles(std::vector<float>& particles,
                       std::vector<float>& weights);

    /// Current number of active particles.
    int num_particles() const { return n_; }

    /// Expose sub-models for runtime tuning.
    MotionModel& motion_model() { return motion_; }
    SensorModel& sensor_model() { return sensor_; }

private:
    void check_resample();
    void do_resample(int target_n);
    int  compute_kld_target();

    Config          cfg_;
    MotionModel     motion_;
    SensorModel     sensor_;
    Resampler       resampler_;
    CudaStream      stream_;

    // GPU double-buffering strategy
    // Two particle buffers to avoid data race: one reads, one writes
    DeviceBuffer<float> d_particles_a_;      // Buffer A
    DeviceBuffer<float> d_particles_b_;      // Buffer B
    float* d_active_particles_ = nullptr;    // Points to active (a or b)
    
    DeviceBuffer<float> d_weights_;
    DeviceBuffer<float> d_ranges_;           // Persisted for async
    DeviceBuffer<float> d_log_w_;            // For numerical stability
    DeviceBuffer<float> d_scratch_w_;        // Swap during normalizatio

    // CUB temp storage for GPU-side reductions
    void* d_cub_temp_ = nullptr;             // CUB DeviceReduce temp
    size_t cub_temp_bytes_ = 0;
    float* d_max_val_ = nullptr;             // Device scalars
    float* d_sum_val_ = nullptr;

    // Pinned memory for async GPU↔CPU transfers
    float* h_ranges_pinned_ = nullptr;       // Input: ranges CPU→GPU
    float* h_particles_pinned_ = nullptr;    // Output: particles GPU→CPU
    float* h_weights_pinned_ = nullptr;      // Output: weights GPU→CPU

    int n_ = 0;                              // Current particle count
    int max_ranges_ = 0;                     // Allocated range buffer capacity

    std::mt19937 rng_{42};                   // For reinitialize() Gaussian sampling
};

}  // namespace gpu_amcl_cpp
