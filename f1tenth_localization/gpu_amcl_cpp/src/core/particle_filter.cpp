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
    // GPU estimate buffers
    if (d_mean_contrib_)  cudaFree(d_mean_contrib_);
    if (d_mean_result_)   cudaFreeHost(d_mean_result_);
    if (d_cov_contrib_)   cudaFree(d_cov_contrib_);
    if (d_cov_result_)    cudaFreeHost(d_cov_result_);
    if (d_est_temp_)      cudaFree(d_est_temp_);
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
    max_ranges_ = sm_cfg.max_beams + 1;
    d_ranges_.allocate(max_ranges_);

    // CUB temp storage for GPU reductions
    cub_temp_bytes_ = query_cub_normalize_temp_bytes(cfg_.max_particles);
    CUDA_CHECK(cudaMalloc(&d_cub_temp_, cub_temp_bytes_));
    CUDA_CHECK(cudaMalloc(&d_max_val_, sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_sum_val_, sizeof(float)));

    // GPU buffers for estimate computation (MeanAccum=16 bytes, CovAccum=24 bytes)
    const size_t mean_accum_size = 16;  // 4 floats
    const size_t cov_accum_size = 24;   // 6 floats
    CUDA_CHECK(cudaMalloc(&d_mean_contrib_, cfg_.max_particles * mean_accum_size));
    CUDA_CHECK(cudaMallocHost(&d_mean_result_, mean_accum_size));  // pinned for fast D→H
    CUDA_CHECK(cudaMalloc(&d_cov_contrib_, cfg_.max_particles * cov_accum_size));
    CUDA_CHECK(cudaMallocHost(&d_cov_result_, cov_accum_size));    // pinned for fast D→H
    // CUB temp for estimate reductions (take max of mean and cov temp requirements)
    size_t mean_temp = query_estimate_mean_temp_bytes(cfg_.max_particles);
    size_t cov_temp = query_estimate_cov_temp_bytes(cfg_.max_particles);
    est_temp_bytes_ = std::max(mean_temp, cov_temp);
    CUDA_CHECK(cudaMalloc(&d_est_temp_, est_temp_bytes_));

    // Pinned memory for async transfers
    CUDA_CHECK(cudaMallocHost(&h_ranges_pinned_,    max_ranges_ * sizeof(float)));
    CUDA_CHECK(cudaMallocHost(&h_particles_pinned_, cfg_.max_particles * 3 * sizeof(float)));
    CUDA_CHECK(cudaMallocHost(&h_weights_pinned_,   cfg_.max_particles * sizeof(float)));

    // Pre-allocate buffers for get_estimate() (avoid per-call allocation)
    est_angles_.resize(cfg_.max_particles);
    est_weights_.resize(cfg_.max_particles);

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
    // Compute N_eff = 1 / Σ(w_i²)
    // - If all weights equal (w=1/N): N_eff = N (perfect distribution)
    // - If one weight = 1, rest = 0: N_eff = 1 (completely degenerate)
    double n_eff = resampler_.effective_sample_size(
        d_weights_.ptr(), n_, stream_.get());

    // E.g., threshold=0.5 means resample when < 50% effective particles
    if (n_eff < cfg_.resample_threshold * n_) {
        // If KLD enabled: compute optimal particle count adaptively
        int target = cfg_.use_kld ? compute_kld_target() : n_;
        do_resample(target);
    }
}

void ParticleFilter::do_resample(int target_n) {
    // Clamp target to valid range [min_particles, max_particles]
    target_n = std::clamp(target_n, cfg_.min_particles, cfg_.max_particles);

    // - d_particles_a_ and d_particles_b_ are two separate GPU buffers
    // - One is "active" (being read), other is "inactive" (being written)
    // - This avoids race conditions: kernel reads from A while writing to B
    float* inactive;
    if (d_active_particles_ == d_particles_a_.ptr()) {
        // Active is A, so use B as the write target
        inactive = d_particles_b_.ptr();
    } else {
        // Active is B, so use A as the write target
        inactive = d_particles_a_.ptr();
    }

    // - Reads particles from d_active_particles_ (weighted by d_weights_)
    // - Writes resampled particles to inactive buffer
    // - Returns actual number of particles written
    n_ = resampler_.resample_to(d_active_particles_, d_weights_.ptr(),
                                inactive, n_, target_n, stream_.get());

    // Pointer swap — O(1), no GPU memory copy
    // The previously inactive buffer is now active
    d_active_particles_ = inactive;
}

int ParticleFilter::compute_kld_target() {
    // Download particles from GPU → CPU using pinned memory
    CUDA_CHECK(cudaMemcpyAsync(h_particles_pinned_, d_active_particles_,
                               n_ * 3 * sizeof(float),
                               cudaMemcpyDeviceToHost, stream_.get()));
    CUDA_CHECK(cudaStreamSynchronize(stream_.get()));

    const float* particles = h_particles_pinned_;

    // Bin particles into a 3D histogram (x, y, θ)
    std::unordered_set<long long> bins;
    for (int i = 0; i < n_; ++i) {
        long long bx = static_cast<long long>(
            std::floor(particles[i * 3 + 0] / cfg_.kld_bin_x));
        long long by = static_cast<long long>(
            std::floor(particles[i * 3 + 1] / cfg_.kld_bin_y));
        long long bt = static_cast<long long>(
            std::floor(particles[i * 3 + 2] / cfg_.kld_bin_theta));
        // Hash: assumes max 1000 bins per dimension (supports tracks up to 500m × 500m with 0.5m bins)
        bins.insert(bx * 1000LL * 1000LL + by * 1000LL + bt);
    }

    // k = number of occupied bins (support of the distribution)
    int k = static_cast<int>(bins.size());
    if (k <= 1) return cfg_.min_particles;

    // Fox et al. KLD formula.
    // n = (k-1) / (2ε) * [1 - 2/(9(k-1)) + sqrt(2/(9(k-1))) * z]³
    double eps = cfg_.kld_epsilon;
    double z   = cfg_.kld_z;
    double km1 = k - 1.0;
    double term = 1.0 - 2.0 / (9.0 * km1) + std::sqrt(2.0 / (9.0 * km1)) * z;
    double target = (km1 / (2.0 * eps)) * term * term * term;

    // Clamp target to valid range [min_particles, max_particles]
    return std::clamp(static_cast<int>(std::ceil(target)), cfg_.min_particles, cfg_.max_particles);
}

// ─── Estimate (GPU-accelerated) ─────────────────────────────────────
PoseEstimate ParticleFilter::get_estimate() {
    // Local structs matching CUDA kernel definitions
    struct MeanAccum { float wx, wy, w_sin, w_cos; };
    struct CovAccum  { float c00, c01, c02, c11, c12, c22; };

    // Step 1: Compute weighted mean on GPU
    launch_gpu_compute_mean(
        d_active_particles_, d_weights_.ptr(),
        d_mean_contrib_, d_mean_result_,
        d_est_temp_, est_temp_bytes_,
        n_, stream_.get());

    // Copy mean result (pinned memory for fast D→H)
    MeanAccum* h_mean = static_cast<MeanAccum*>(d_mean_result_);
    CUDA_CHECK(cudaStreamSynchronize(stream_.get()));

    // CPU reads the result from pinned memory
    PoseEstimate est;
    est.x = h_mean->wx;
    est.y = h_mean->wy;
    est.theta = std::atan2(h_mean->w_sin, h_mean->w_cos); // Circular mean: θ = atan2(Σw*sin(θ), Σw*cos(θ))

    // Step 2: Compute covariance on GPU (needs mean values)
    launch_gpu_compute_covariance(
        d_active_particles_, d_weights_.ptr(),
        static_cast<float>(est.x), static_cast<float>(est.y), static_cast<float>(est.theta),
        d_cov_contrib_, d_cov_result_,
        d_est_temp_, est_temp_bytes_,
        n_, stream_.get());

    // Copy covariance result
    CovAccum* h_cov = static_cast<CovAccum*>(d_cov_result_);
    CUDA_CHECK(cudaStreamSynchronize(stream_.get()));

    // Build symmetric 3×3 covariance matrix
    est.covariance(0, 0) = h_cov->c00;
    est.covariance(0, 1) = h_cov->c01;
    est.covariance(0, 2) = h_cov->c02;
    est.covariance(1, 0) = h_cov->c01;  // symmetric
    est.covariance(1, 1) = h_cov->c11;
    est.covariance(1, 2) = h_cov->c12;
    est.covariance(2, 0) = h_cov->c02;  // symmetric
    est.covariance(2, 1) = h_cov->c12;  // symmetric
    est.covariance(2, 2) = h_cov->c22;

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
