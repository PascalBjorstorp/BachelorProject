#pragma once

#include "gpu_amcl_cpp/core/motion_model.hpp"
#include "gpu_amcl_cpp/core/sensor_model.hpp"
#include "gpu_amcl_cpp/core/resampling.hpp"
#include "gpu_amcl_cpp/helpers/cuda_utils.hpp"
#include "gpu_amcl_cpp/helpers/map_utils.hpp"

#include <Eigen/Core>
#include <vector>
#include <random>
#include <mutex>

namespace gpu_amcl_cpp {

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
        double init_x  = 0.0, init_y  = 0.0, init_a  = 0.0;
        double init_cov_xx = 0.5, init_cov_yy = 0.5, init_cov_aa = 0.2;

        // Resampling
        double resample_threshold   = 0.5;

        // KLD adaptive sampling
        bool   use_kld              = false;
        double kld_epsilon          = 0.05;
        double kld_z                = 2.33;
        double kld_bin_x            = 0.5;  ///< metres
        double kld_bin_y            = 0.5;
        double kld_bin_theta        = 0.1;  ///< radians

        // Recovery (nav2-style, slow/fast weight tracking)
        bool   use_recovery         = false;
        double recovery_alpha_slow  = 0.001;
        double recovery_alpha_fast  = 0.1;
        double recovery_random_max  = 0.05;
    };

    ParticleFilter() = default;

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
    void predict(float dx, float dy, float dtheta, float imu_dtheta);

    /**
     * @brief Measurement (sensor model) update + conditional resampling.
     *
     * @param ranges      Host array of laser range readings.
     * @param num_ranges  Number of readings.
     * @param angle_min   First beam angle (rad).
     * @param angle_inc   Angle increment (rad).
     */
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
    void inject_random_particles(double fraction);

    Config          cfg_;
    MotionModel     motion_;
    SensorModel     sensor_;
    Resampler       resampler_;
    CudaStream      stream_;

    // ── Persistent GPU buffers (§5: allocated once in init, reused every frame) ──
    DeviceBuffer<float> d_particles_a_;   ///< Nx3 particle buffer A
    DeviceBuffer<float> d_particles_b_;   ///< Nx3 particle buffer B (double-buffer)
    float* d_active_particles_ = nullptr; ///< Points to active buffer (a or b)

    DeviceBuffer<float> d_weights_;       ///< N normalised weights
    DeviceBuffer<float> d_ranges_;        ///< Persistent scan buffer (max beams)
    DeviceBuffer<float> d_log_w_;         ///< Persistent log-weight buffer
    DeviceBuffer<float> d_scratch_w_;     ///< Scratch buffer for normalization swap

    int n_ = 0;  ///< current active particle count.
    int max_ranges_ = 0; ///< allocated range buffer capacity

    // Recovery state
    double w_slow_ = 0.0;
    double w_fast_ = 0.0;

    // Free-space cells for random injection
    const MapProcessor* map_ = nullptr;
    std::mt19937        rng_{42};
};

}  // namespace gpu_amcl_cpp
