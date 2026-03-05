#include "gpu_amcl_cpp/core/particle_filter.hpp"
#include "gpu_amcl_cpp/helpers/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <unordered_set>

namespace gpu_amcl_cpp {

// ─── Initialise ─────────────────────────────────────────────────────
void ParticleFilter::init(const Config& pf_cfg,
                          const MotionModel::Config& mm_cfg,
                          const SensorModel::Config& sm_cfg,
                          const MapProcessor& map) {
    cfg_ = pf_cfg;
    map_ = &map;
    n_   = cfg_.num_particles;

    // ── Allocate persistent GPU buffers (§5: done once, reused every frame) ──
    d_particles_a_.allocate(cfg_.max_particles * 3);
    d_particles_b_.allocate(cfg_.max_particles * 3);
    d_active_particles_ = d_particles_a_.ptr();

    d_weights_.allocate(cfg_.max_particles);
    d_log_w_.allocate(cfg_.max_particles);
    d_scratch_w_.allocate(cfg_.max_particles);

    // Pre-allocate range buffer for typical LiDAR (will grow if needed).
    max_ranges_ = 1080;  // typical LiDAR beam count
    d_ranges_.allocate(max_ranges_);

    // Scatter particles around initial pose.
    reinitialize(cfg_.init_x, cfg_.init_y, cfg_.init_a,
                 cfg_.init_cov_xx, cfg_.init_cov_yy, cfg_.init_cov_aa);

    // Init sub-components.
    motion_.init(cfg_.max_particles, mm_cfg);
    sensor_.init(map, sm_cfg);
    resampler_.init(cfg_.max_particles);

    // Reset recovery.
    w_slow_ = 0.0;
    w_fast_ = 0.0;
}

void ParticleFilter::reinitialize(double x, double y, double theta,
                                  double cov_xx, double cov_yy,
                                  double cov_aa) {
    n_ = cfg_.num_particles;

    std::vector<float> particles(n_ * 3);
    std::vector<float> weights(n_, 1.0f / n_);

    std::normal_distribution<float> dx(0, static_cast<float>(std::sqrt(cov_xx)));
    std::normal_distribution<float> dy(0, static_cast<float>(std::sqrt(cov_yy)));
    std::normal_distribution<float> da(0, static_cast<float>(std::sqrt(cov_aa)));

    for (int i = 0; i < n_; ++i) {
        particles[i * 3 + 0] = static_cast<float>(x) + dx(rng_);
        particles[i * 3 + 1] = static_cast<float>(y) + dy(rng_);
        float a = static_cast<float>(theta) + da(rng_);
        particles[i * 3 + 2] = std::atan2(std::sin(a), std::cos(a));
    }

    CUDA_CHECK(cudaMemcpy(d_active_particles_, particles.data(),
                          n_ * 3 * sizeof(float), cudaMemcpyHostToDevice));
    d_weights_.upload(weights.data(), n_);

    w_slow_ = 0.0;
    w_fast_ = 0.0;
}

// ─── Predict ────────────────────────────────────────────────────────
void ParticleFilter::predict(float dx, float dy, float dtheta,
                             float imu_dtheta) {
    motion_.apply(d_active_particles_, n_,
                  dx, dy, dtheta, imu_dtheta,
                  stream_.get());
}

// ─── Update ─────────────────────────────────────────────────────────
void ParticleFilter::update(const float* ranges, int num_ranges,
                            float angle_min, float angle_inc) {
    // §5: Reuse persistent range buffer (grow if needed, never per-frame alloc).
    if (num_ranges > max_ranges_) {
        max_ranges_ = num_ranges;
        d_ranges_.allocate(max_ranges_);
    }
    d_ranges_.upload(ranges, num_ranges);

    // §5: Reuse persistent log-weight buffer (no per-frame alloc).
    sensor_.compute_weights(d_active_particles_, n_,
                            d_ranges_.ptr(), num_ranges,
                            angle_min, angle_inc,
                            d_log_w_.ptr(), stream_.get());

    // Download log-weights to host for normalisation & recovery bookkeeping.
    std::vector<float> log_w(n_);
    d_log_w_.download(log_w.data(), n_);

    // Subtract max for numerical stability, exponentiate.
    float max_lw = *std::max_element(log_w.begin(), log_w.end());
    std::vector<float> weights(n_);
    for (int i = 0; i < n_; ++i) {
        weights[i] = std::exp(log_w[i] - max_lw);
    }

    // Multiplicative update with existing weights.
    std::vector<float> old_w(n_);
    d_weights_.download(old_w.data(), n_);
    for (int i = 0; i < n_; ++i) {
        weights[i] *= old_w[i];
    }

    // Normalise.
    float sum = std::accumulate(weights.begin(), weights.end(), 0.0f);
    if (sum > 0.0f) {
        float inv_sum = 1.0f / sum;
        for (auto& w : weights) w *= inv_sum;
    } else {
        float inv_n = 1.0f / n_;
        std::fill(weights.begin(), weights.end(), inv_n);
    }

    // Recovery: track w_slow / w_fast (only if recovery is enabled).
    if (cfg_.use_recovery) {
        float w_avg = sum / n_;
        if (w_slow_ == 0.0) {
            w_slow_ = w_avg;
            w_fast_ = w_avg;
        } else {
            w_slow_ += cfg_.recovery_alpha_slow * (w_avg - w_slow_);
            w_fast_ += cfg_.recovery_alpha_fast * (w_avg - w_fast_);
        }
    }

    // Upload normalised weights.
    d_weights_.upload(weights.data(), n_);

    // Conditionally resample.
    check_resample();
}

// ─── Resampling ─────────────────────────────────────────────────────
void ParticleFilter::check_resample() {
    double n_eff = resampler_.effective_sample_size(
        d_weights_.ptr(), n_, stream_.get());

    if (n_eff < cfg_.resample_threshold * n_) {
        int target = cfg_.use_kld ? compute_kld_target() : n_;
        do_resample(target);
    }
}
}

void ParticleFilter::do_resample(int target_n) {
    target_n = std::clamp(target_n, cfg_.min_particles, cfg_.max_particles);

    // §5: Double-buffer resample — write to inactive buffer, then swap pointers.
    float* inactive = (d_active_particles_ == d_particles_a_.ptr())
                      ? d_particles_b_.ptr() : d_particles_a_.ptr();

    n_ = resampler_.resample_to(d_active_particles_, d_weights_.ptr(),
                                inactive, n_, target_n, stream_.get());

    // Pointer swap — no D→D memcpy, no sync needed.
    d_active_particles_ = inactive;

    // Recovery: inject random particles if filter is degrading.
    if (cfg_.use_recovery && w_slow_ > 0.0) {
        double ratio = 1.0 - w_fast_ / w_slow_;
        if (ratio > 0.0) {
            double frac = std::min(ratio, cfg_.recovery_random_max);
            inject_random_particles(frac);
        }
    }
}

int ParticleFilter::compute_kld_target() {
    // Download particles.
    std::vector<float> particles(n_ * 3);
    CUDA_CHECK(cudaMemcpy(particles.data(), d_active_particles_,
                          n_ * 3 * sizeof(float), cudaMemcpyDeviceToHost));

    // Bin particles into (x, y, θ) histogram.
    std::unordered_set<long long> bins;
    for (int i = 0; i < n_; ++i) {
        long long bx = static_cast<long long>(
            std::floor(particles[i * 3 + 0] / cfg_.kld_bin_x));
        long long by = static_cast<long long>(
            std::floor(particles[i * 3 + 1] / cfg_.kld_bin_y));
        long long bt = static_cast<long long>(
            std::floor(particles[i * 3 + 2] / cfg_.kld_bin_theta));
        // Simple hash.
        bins.insert(bx * 100000LL * 100000LL + by * 100000LL + bt);
    }

    int k = static_cast<int>(bins.size());
    if (k <= 1) return cfg_.min_particles;

    // Fox et al. KLD formula.
    double eps = cfg_.kld_epsilon;
    double z   = cfg_.kld_z;
    double km1 = k - 1.0;
    double term = 1.0 - 2.0 / (9.0 * km1)
                + std::sqrt(2.0 / (9.0 * km1)) * z;
    double target = (km1 / (2.0 * eps)) * term * term * term;

    return std::clamp(static_cast<int>(std::ceil(target)),
                      cfg_.min_particles, cfg_.max_particles);
}

void ParticleFilter::inject_random_particles(double fraction) {
    if (!map_ || map_->free_cells().empty()) return;

    int num_inject = static_cast<int>(fraction * n_);
    if (num_inject <= 0) return;

    // Download current particles + weights.
    std::vector<float> particles(n_ * 3);
    std::vector<float> weights(n_);
    CUDA_CHECK(cudaMemcpy(particles.data(), d_active_particles_,
                          n_ * 3 * sizeof(float), cudaMemcpyDeviceToHost));
    d_weights_.download(weights.data(), n_);

    // Find indices of lowest-weight particles.
    std::vector<int> idx(n_);
    std::iota(idx.begin(), idx.end(), 0);
    std::partial_sort(idx.begin(), idx.begin() + num_inject, idx.end(),
                      [&](int a, int b) { return weights[a] < weights[b]; });

    const auto& free = map_->free_cells();
    std::uniform_int_distribution<int> cell_dist(0,
                                                 static_cast<int>(free.size()) - 1);
    std::uniform_real_distribution<float> angle_dist(
        -static_cast<float>(M_PI), static_cast<float>(M_PI));

    for (int j = 0; j < num_inject; ++j) {
        int fi = free[cell_dist(rng_)];
        double wx, wy;
        map_->map_to_world(fi % map_->width(), fi / map_->width(), wx, wy);
        int pi = idx[j];
        particles[pi * 3 + 0] = static_cast<float>(wx);
        particles[pi * 3 + 1] = static_cast<float>(wy);
        particles[pi * 3 + 2] = angle_dist(rng_);
        weights[pi] = 1.0f / n_;
    }

    CUDA_CHECK(cudaMemcpy(d_active_particles_, particles.data(),
                          n_ * 3 * sizeof(float), cudaMemcpyHostToDevice));
    d_weights_.upload(weights.data(), n_);
}

// ─── Estimate ───────────────────────────────────────────────────────
PoseEstimate ParticleFilter::get_estimate() {
    std::vector<float> particles(n_ * 3);
    std::vector<float> weights(n_);
    CUDA_CHECK(cudaMemcpy(particles.data(), d_active_particles_,
                          n_ * 3 * sizeof(float), cudaMemcpyDeviceToHost));
    d_weights_.download(weights.data(), n_);

    PoseEstimate est;

    // Weighted mean (x, y).
    double wx = 0, wy = 0;
    for (int i = 0; i < n_; ++i) {
        wx += weights[i] * particles[i * 3 + 0];
        wy += weights[i] * particles[i * 3 + 1];
    }
    est.x = wx;
    est.y = wy;

    // Circular mean for θ.
    std::vector<double> angles(n_);
    std::vector<double> wd(n_);
    for (int i = 0; i < n_; ++i) {
        angles[i] = particles[i * 3 + 2];
        wd[i]     = weights[i];
    }
    est.theta = math_utils::weighted_circular_mean(
        angles.data(), wd.data(), n_);

    // 3×3 weighted covariance.
    Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
    for (int i = 0; i < n_; ++i) {
        Eigen::Vector3d diff;
        diff[0] = particles[i * 3 + 0] - est.x;
        diff[1] = particles[i * 3 + 1] - est.y;
        diff[2] = math_utils::angle_diff(particles[i * 3 + 2], est.theta);
        C += weights[i] * (diff * diff.transpose());
    }
    est.covariance = C;

    return est;
}

void ParticleFilter::get_particles(std::vector<float>& particles,
                                   std::vector<float>& weights) {
    particles.resize(n_ * 3);
    weights.resize(n_);
    CUDA_CHECK(cudaMemcpy(particles.data(), d_active_particles_,
                          n_ * 3 * sizeof(float), cudaMemcpyDeviceToHost));
    d_weights_.download(weights.data(), n_);
}

}  // namespace gpu_amcl_cpp
