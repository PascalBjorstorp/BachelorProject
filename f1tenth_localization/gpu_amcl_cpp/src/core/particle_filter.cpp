#include "gpu_amcl_cpp/core/particle_filter.hpp"
#include "gpu_amcl_cpp/helpers/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <unordered_set>

namespace gpu_amcl_cpp {

// ─── Destructor (free raw CUDA allocations) ─────────────────────────
ParticleFilter::~ParticleFilter() {
    // CUB temp storage
    if (d_cub_temp_) cudaFree(d_cub_temp_);
    if (d_max_val_)  cudaFree(d_max_val_);
    if (d_sum_val_)  cudaFree(d_sum_val_);
    // Pinned host buffers
    if (h_ranges_pinned_)    cudaFreeHost(h_ranges_pinned_);
    if (h_particles_pinned_) cudaFreeHost(h_particles_pinned_);
    if (h_weights_pinned_)   cudaFreeHost(h_weights_pinned_);
}

// ─── Initialise ─────────────────────────────────────────────────────
void ParticleFilter::init(const Config& pf_cfg,
                          const MotionModel::Config& mm_cfg,
                          const SensorModel::Config& sm_cfg,
                          const MapProcessor& map) {
    cfg_ = pf_cfg;
    n_   = cfg_.num_particles;

    // ── GPU Buffer Allocations ──
    // Double-buffer for particles (A and B)
    d_particles_a_.allocate(cfg_.max_particles * 3);    // x,y,θ per particle
    d_particles_b_.allocate(cfg_.max_particles * 3);
    d_active_particles_ = d_particles_a_.ptr();         // Start with buffer A

    d_weights_.allocate(cfg_.max_particles);
    d_log_w_.allocate(cfg_.max_particles);
    d_scratch_w_.allocate(cfg_.max_particles);

    // Pre-allocate range buffer based on max_beams config
    max_ranges_ = sm_cfg.max_beams;
    d_ranges_.allocate(max_ranges_);

    // CUB temp storage for GPU reductions
    cub_temp_bytes_ = query_cub_normalize_temp_bytes(cfg_.max_particles);
    CUDA_CHECK(cudaMalloc(&d_cub_temp_, cub_temp_bytes_));
    CUDA_CHECK(cudaMalloc(&d_max_val_, sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_sum_val_, sizeof(float)));

    // Pinned memory for async transfers
    CUDA_CHECK(cudaMallocHost(&h_ranges_pinned_,    max_ranges_ * sizeof(float)));
    CUDA_CHECK(cudaMallocHost(&h_particles_pinned_, cfg_.max_particles * 3 * sizeof(float)));
    CUDA_CHECK(cudaMallocHost(&h_weights_pinned_,   cfg_.max_particles * sizeof(float)));

    // Initialize particles around starting pose
    reinitialize(cfg_.init_x, cfg_.init_y, cfg_.init_a,
                 cfg_.init_cov_xx, cfg_.init_cov_yy, cfg_.init_cov_aa);

    // Initialize sub-components
    motion_.init(cfg_.max_particles, mm_cfg);
    sensor_.init(map, sm_cfg);
    resampler_.init(cfg_.max_particles);
}

void ParticleFilter::reinitialize(double x, double y, double theta,
                                  double cov_xx, double cov_yy,
                                  double cov_aa) {
    n_ = cfg_.num_particles;    // Reset to initial number of particles

    // Create CPU arrays for particles and weights
    std::vector<float> particles(n_ * 3);       // x, y, θ for each particle
    std::vector<float> weights(n_, 1.0f / n_);  // Uniform weights

    // Create Gaussian distributions for noise
    std::normal_distribution<float> dx(0, static_cast<float>(std::sqrt(cov_xx)));
    std::normal_distribution<float> dy(0, static_cast<float>(std::sqrt(cov_yy)));
    std::normal_distribution<float> da(0, static_cast<float>(std::sqrt(cov_aa)));

    // Generate particles around the pose
    for (int i = 0; i < n_; ++i) {
        particles[i * 3 + 0] = static_cast<float>(x) + dx(rng_);        // Add noise to x
        particles[i * 3 + 1] = static_cast<float>(y) + dy(rng_);        // Add noise to y
        float a = static_cast<float>(theta) + da(rng_);                 // Add noise to θ
        particles[i * 3 + 2] = std::atan2(std::sin(a), std::cos(a));    // Normalize angle to [-π, π]
    }

    // Upload to GPU
    CUDA_CHECK(cudaMemcpy(d_active_particles_, particles.data(), n_ * 3 * sizeof(float), cudaMemcpyHostToDevice));
    d_weights_.upload(weights.data(), n_);
}

// ─── Predict ────────────────────────────────────────────────────────
void ParticleFilter::predict(float dx, float dy, float dtheta) {
    motion_.apply(d_active_particles_, n_,
                  dx, dy, dtheta,
                  stream_.get());
}

// ─── Update ─────────────────────────────────────────────────────────
void ParticleFilter::update(const float* ranges, int num_ranges,
                            float angle_min, float angle_inc) {
    // Guard: ensure scan fits in pre-allocated buffer (set by max_beams param)
    if (num_ranges > max_ranges_) {
        std::fprintf(stderr,
                     "[gpu_amcl_cpp][ParticleFilter] ERROR: num_ranges (%d) exceeds max_ranges_ (%d). "
                     "Increase max_beams in config.\n",
                     num_ranges, max_ranges_);
        return;
    }

    // Copy to pinned staging, then async DMA to device.
    memcpy(h_ranges_pinned_, ranges, num_ranges * sizeof(float));
    CUDA_CHECK(cudaMemcpyAsync(d_ranges_.ptr(), h_ranges_pinned_,
                               num_ranges * sizeof(float),
                               cudaMemcpyHostToDevice, stream_.get()));

    // §5: Reuse persistent log-weight buffer (no per-frame alloc).
    sensor_.compute_weights(d_active_particles_, n_,
                            d_ranges_.ptr(), num_ranges,
                            angle_min, angle_inc,
                            d_log_w_.ptr(), stream_.get());

    // §1: GPU-side weight normalization — all on GPU, no host transfers.
    // Steps: CUB::Max(log_w) → exp-shift-mul → CUB::Sum → normalize.
    launch_gpu_normalize_weights(
        d_log_w_.ptr(), d_weights_.ptr(), d_scratch_w_.ptr(),
        d_max_val_, d_sum_val_,
        d_cub_temp_, cub_temp_bytes_,
        n_, stream_.get());

    // After normalize, d_scratch_w_ holds the normalised weights — swap.
    std::swap(d_weights_, d_scratch_w_); // Pointer swap, no copy, no sync needed. d_weights_ now has normalised weights for resampling.

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

void ParticleFilter::do_resample(int target_n) {
    target_n = std::clamp(target_n, cfg_.min_particles, cfg_.max_particles);

    // §5: Double-buffer resample — write to inactive buffer, then swap pointers.
    float* inactive = (d_active_particles_ == d_particles_a_.ptr())
                      ? d_particles_b_.ptr() : d_particles_a_.ptr();

    n_ = resampler_.resample_to(d_active_particles_, d_weights_.ptr(),
                                inactive, n_, target_n, stream_.get());

    // Pointer swap — no D→D memcpy, no sync needed.
    d_active_particles_ = inactive;
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

// ─── Estimate ───────────────────────────────────────────────────────
PoseEstimate ParticleFilter::get_estimate() {
    // §4: Use pinned staging buffers for D→H transfers (truly async-capable).
    CUDA_CHECK(cudaMemcpyAsync(h_particles_pinned_, d_active_particles_,
                               n_ * 3 * sizeof(float),
                               cudaMemcpyDeviceToHost, stream_.get()));
    CUDA_CHECK(cudaMemcpyAsync(h_weights_pinned_, d_weights_.ptr(),
                               n_ * sizeof(float),
                               cudaMemcpyDeviceToHost, stream_.get()));
    CUDA_CHECK(cudaStreamSynchronize(stream_.get()));

    const float* particles = h_particles_pinned_;
    const float* weights   = h_weights_pinned_;

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
    // §4: Use pinned staging for D→H, then copy to caller's vectors.
    CUDA_CHECK(cudaMemcpyAsync(h_particles_pinned_, d_active_particles_,
                               n_ * 3 * sizeof(float),
                               cudaMemcpyDeviceToHost, stream_.get()));
    CUDA_CHECK(cudaMemcpyAsync(h_weights_pinned_, d_weights_.ptr(),
                               n_ * sizeof(float),
                               cudaMemcpyDeviceToHost, stream_.get()));
    CUDA_CHECK(cudaStreamSynchronize(stream_.get()));
    memcpy(particles.data(), h_particles_pinned_, n_ * 3 * sizeof(float));
    memcpy(weights.data(), h_weights_pinned_, n_ * sizeof(float));
}

}  // namespace gpu_amcl_cpp
