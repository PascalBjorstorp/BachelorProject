#include "gpu_amcl_cpp/core/particle_filter.hpp"
#include "gpu_amcl_cpp/helpers/math_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace gpu_amcl_cpp {

namespace {

double timed_memcpy_async(
    void* dst,
    const void* src,
    size_t bytes,
    cudaMemcpyKind kind,
    cudaStream_t stream) {
    if (bytes == 0) {
        return 0.0;
    }

    const auto start = std::chrono::high_resolution_clock::now();
    CUDA_CHECK(cudaMemcpyAsync(dst, src, bytes, kind, stream));
    CUDA_CHECK(cudaStreamSynchronize(stream));
    const auto stop = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::milli>(stop - start).count();
}

template <typename Fn>
double timed_stream_stage(cudaStream_t stream, Fn&& fn) {
    const auto start = std::chrono::high_resolution_clock::now();
    fn();
    CUDA_CHECK(cudaStreamSynchronize(stream));
    const auto stop = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::milli>(stop - start).count();
}

}  // namespace

void ParticleFilter::reset_stage_timing() {
    last_stage_diag_ = StageTimingDiagnostics{};
}

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
    // GPU cluster-estimate buffers
    if (d_cluster_scores_)       cudaFree(d_cluster_scores_);
    if (d_cluster_best_)         cudaFree(d_cluster_best_);
    if (d_cluster_second_)       cudaFree(d_cluster_second_);
    if (h_cluster_best_)         cudaFreeHost(h_cluster_best_);
    if (h_cluster_second_)       cudaFreeHost(h_cluster_second_);
    if (d_cluster_mean_contrib_) cudaFree(d_cluster_mean_contrib_);
    if (d_cluster_mean_result_)  cudaFree(d_cluster_mean_result_);
    if (h_cluster_mean_result_)  cudaFreeHost(h_cluster_mean_result_);
    if (d_cluster_cov_contrib_)  cudaFree(d_cluster_cov_contrib_);
    if (d_cluster_cov_result_)   cudaFree(d_cluster_cov_result_);
    if (h_cluster_cov_result_)   cudaFreeHost(h_cluster_cov_result_);
    if (d_cluster_temp_)         cudaFree(d_cluster_temp_);
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
    map_ = &map;

    // ── GPU Buffer Allocations ──
    // Double-buffer for particles (A and B)
    d_particles_a_.allocate(cfg_.max_particles * 3);    // x,y,θ per particle
    d_particles_b_.allocate(cfg_.max_particles * 3);
    d_active_particles_ = d_particles_a_.ptr();         // Start with buffer A

    d_weights_.allocate(cfg_.max_particles);
    d_log_w_.allocate(cfg_.max_particles);
    d_scratch_w_.allocate(cfg_.max_particles);

    // Pre-allocate range buffer based on max_beams config
    sensor_max_beams_ = sm_cfg.max_beams;
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

    // GPU buffers for local cluster estimate.
    CUDA_CHECK(cudaMalloc(&d_cluster_scores_, cfg_.max_particles * sizeof(ClusterScoreResult)));
    CUDA_CHECK(cudaMalloc(&d_cluster_best_, sizeof(ClusterScoreResult)));
    CUDA_CHECK(cudaMalloc(&d_cluster_second_, sizeof(ClusterScoreResult)));
    CUDA_CHECK(cudaMallocHost(&h_cluster_best_, sizeof(ClusterScoreResult)));
    CUDA_CHECK(cudaMallocHost(&h_cluster_second_, sizeof(ClusterScoreResult)));
    CUDA_CHECK(cudaMalloc(&d_cluster_mean_contrib_, cfg_.max_particles * sizeof(ClusterMeanAccum)));
    CUDA_CHECK(cudaMalloc(&d_cluster_mean_result_, sizeof(ClusterMeanAccum)));
    CUDA_CHECK(cudaMallocHost(&h_cluster_mean_result_, sizeof(ClusterMeanAccum)));
    CUDA_CHECK(cudaMalloc(&d_cluster_cov_contrib_, cfg_.max_particles * sizeof(ClusterCovAccum)));
    CUDA_CHECK(cudaMalloc(&d_cluster_cov_result_, sizeof(ClusterCovAccum)));
    CUDA_CHECK(cudaMallocHost(&h_cluster_cov_result_, sizeof(ClusterCovAccum)));
    const size_t cluster_score_temp = query_cluster_score_temp_bytes(cfg_.max_particles);
    const size_t cluster_mean_temp = query_cluster_mean_temp_bytes(cfg_.max_particles);
    const size_t cluster_cov_temp = query_cluster_cov_temp_bytes(cfg_.max_particles);
    cluster_temp_bytes_ = std::max(cluster_score_temp, std::max(cluster_mean_temp, cluster_cov_temp));
    CUDA_CHECK(cudaMalloc(&d_cluster_temp_, cluster_temp_bytes_));

    // Pinned memory for async transfers
    CUDA_CHECK(cudaMallocHost(&h_ranges_pinned_,    max_ranges_ * sizeof(float)));
    CUDA_CHECK(cudaMallocHost(&h_particles_pinned_, cfg_.max_particles * 3 * sizeof(float)));
    CUDA_CHECK(cudaMallocHost(&h_weights_pinned_,   cfg_.max_particles * sizeof(float)));

    // Pre-allocate buffers for get_estimate() (avoid per-call allocation)
    est_angles_.resize(cfg_.max_particles);
    est_weights_.resize(cfg_.max_particles);

    if (cfg_.global_initialization) {
        reinitialize_global(map);
    } else {
        reinitialize(cfg_.init_x, cfg_.init_y, cfg_.init_a,
                     cfg_.init_cov_xx, cfg_.init_cov_yy, cfg_.init_cov_aa);
    }

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

float ParticleFilter::nearest_global_heading(float wx, float wy) const {
    if (cfg_.global_heading_points.empty()) {
        return static_cast<float>(cfg_.init_a);
    }

    const auto* best = &cfg_.global_heading_points.front();
    float best_d2 = std::numeric_limits<float>::max();
    for (const auto& point : cfg_.global_heading_points) {
        const float dx = point.x - wx;
        const float dy = point.y - wy;
        const float d2 = dx * dx + dy * dy;
        if (d2 < best_d2) {
            best_d2 = d2;
            best = &point;
        }
    }
    return best->yaw;
}

bool ParticleFilter::sample_global_particle(const MapProcessor& map,
                                            float& wx,
                                            float& wy,
                                            float& yaw) {
    const auto normalize_yaw = [](float a) {
        return std::atan2(std::sin(a), std::cos(a));
    };

    const float heading_cone = static_cast<float>(
        std::max(0.0, cfg_.global_heading_cone_rad));
    std::uniform_real_distribution<float> yaw_noise(-heading_cone, heading_cone);

    if (!cfg_.global_heading_points.empty()) {
        std::uniform_int_distribution<int> point_dist(
            0, static_cast<int>(cfg_.global_heading_points.size()) - 1);

        const float max_lateral = static_cast<float>(
            cfg_.global_max_lateral_offset_m > 0.0 ? cfg_.global_max_lateral_offset_m : 0.55);
        const float wall_margin = static_cast<float>(std::max(0.0, cfg_.global_track_margin_m));
        const auto& distance_field = map.distance_field();

        for (int attempt = 0; attempt < 200; ++attempt) {
            const auto& point =
                cfg_.global_heading_points[static_cast<size_t>(point_dist(rng_))];

            float left = point.d_left > 0.0f ? std::min(point.d_left, max_lateral) : max_lateral;
            float right = point.d_right > 0.0f ? std::min(point.d_right, max_lateral) : max_lateral;
            left = std::max(0.0f, left - wall_margin);
            right = std::max(0.0f, right - wall_margin);
            if (left <= 0.0f && right <= 0.0f) {
                continue;
            }

            std::uniform_real_distribution<float> lateral_dist(-right, left);
            const float lateral = lateral_dist(rng_);
            const float nx_left = -std::sin(point.yaw);
            const float ny_left = std::cos(point.yaw);
            const float cand_x = point.x + lateral * nx_left;
            const float cand_y = point.y + lateral * ny_left;

            int mx = 0;
            int my = 0;
            map.world_to_map(cand_x, cand_y, mx, my);
            if (!map.is_free(mx, my)) {
                continue;
            }

            const int flat_idx = my * map.width() + mx;
            if (wall_margin > 0.0f &&
                flat_idx >= 0 &&
                static_cast<size_t>(flat_idx) < distance_field.size() &&
                distance_field[static_cast<size_t>(flat_idx)] < wall_margin) {
                continue;
            }

            wx = cand_x;
            wy = cand_y;
            yaw = normalize_yaw(point.yaw + yaw_noise(rng_));
            return true;
        }
    }

    const auto& free_cells = map.free_cells();
    if (free_cells.empty()) {
        return false;
    }

    std::uniform_int_distribution<int> cell_dist(
        0, static_cast<int>(free_cells.size()) - 1);
    const int flat_idx = free_cells[static_cast<size_t>(cell_dist(rng_))];
    const int mx = flat_idx % map.width();
    const int my = flat_idx / map.width();

    double wx_d = 0.0;
    double wy_d = 0.0;
    map.map_to_world(mx, my, wx_d, wy_d);
    wx = static_cast<float>(wx_d);
    wy = static_cast<float>(wy_d);
    yaw = normalize_yaw(nearest_global_heading(wx, wy) + yaw_noise(rng_));
    return true;
}

void ParticleFilter::reinitialize_global(const MapProcessor& map) {
    if (map.free_cells().empty()) {
        std::fprintf(stderr,
                     "[gpu_amcl_cpp][ParticleFilter] WARNING: no free cells for global initialization; "
                     "falling back to configured initial pose.\n");
        reinitialize(cfg_.init_x, cfg_.init_y, cfg_.init_a,
                     cfg_.init_cov_xx, cfg_.init_cov_yy, cfg_.init_cov_aa);
        return;
    }

    n_ = cfg_.num_particles;

    std::vector<float> particles(n_ * 3);
    std::vector<float> weights(n_, 1.0f / n_);

    if (cfg_.global_heading_points.empty()) {
        std::fprintf(stderr,
                     "[gpu_amcl_cpp][ParticleFilter] WARNING: global initialization has no heading path; "
                     "using initial_pose_a plus heading cone.\n");
    }

    for (int i = 0; i < n_; ++i) {
        float wx = 0.0f;
        float wy = 0.0f;
        float yaw = 0.0f;

        if (!sample_global_particle(map, wx, wy, yaw)) {
            wx = static_cast<float>(cfg_.init_x);
            wy = static_cast<float>(cfg_.init_y);
            yaw = static_cast<float>(cfg_.init_a);
        }

        particles[i * 3 + 0] = wx;
        particles[i * 3 + 1] = wy;
        particles[i * 3 + 2] = yaw;
    }

    CUDA_CHECK(cudaMemcpy(
        d_active_particles_, particles.data(), n_ * 3 * sizeof(float), cudaMemcpyHostToDevice));
    d_weights_.upload(weights.data(), n_);
}

void ParticleFilter::inject_recovery_particles() {
    if (!cfg_.enable_recovery_injection ||
        cfg_.recovery_injection_ratio <= 0.0 ||
        n_ <= 0 ||
        map_ == nullptr ||
        !map_->is_loaded() ||
        cfg_.global_heading_points.empty()) {
        return;
    }

    const double ratio = std::clamp(cfg_.recovery_injection_ratio, 0.0, 1.0);
    int inject_count = static_cast<int>(std::round(ratio * static_cast<double>(n_)));
    inject_count = std::clamp(inject_count, 1, n_);

    std::vector<float> particles(static_cast<size_t>(inject_count) * 3);
    int filled = 0;
    for (int i = 0; i < inject_count; ++i) {
        float wx = 0.0f;
        float wy = 0.0f;
        float yaw = 0.0f;
        if (!sample_global_particle(*map_, wx, wy, yaw)) {
            continue;
        }

        particles[filled * 3 + 0] = wx;
        particles[filled * 3 + 1] = wy;
        particles[filled * 3 + 2] = yaw;
        ++filled;
    }

    if (filled <= 0) {
        return;
    }

    CUDA_CHECK(cudaMemcpyAsync(
        d_active_particles_ + (n_ - filled) * 3,
        particles.data(),
        static_cast<size_t>(filled) * 3 * sizeof(float),
        cudaMemcpyHostToDevice,
        stream_.get()));
    CUDA_CHECK(cudaStreamSynchronize(stream_.get()));
}

void ParticleFilter::roughen_local_particles() {
    if (!cfg_.enable_local_roughening ||
        !last_scan_confidence_bad_ ||
        cfg_.local_roughening_ratio <= 0.0 ||
        cfg_.local_roughening_xy_std_m <= 0.0 ||
        cfg_.local_roughening_yaw_std_rad <= 0.0 ||
        n_ <= 0 ||
        map_ == nullptr ||
        !map_->is_loaded()) {
        return;
    }

    struct MeanAccum { float wx, wy, w_sin, w_cos; };
    struct CovAccum  { float c00, c01, c02, c11, c12, c22; };

    launch_gpu_compute_mean(
        d_active_particles_, d_weights_.ptr(),
        d_mean_contrib_, d_mean_result_,
        d_est_temp_, est_temp_bytes_,
        n_, stream_.get());
    CUDA_CHECK(cudaStreamSynchronize(stream_.get()));

    const MeanAccum* h_mean = static_cast<const MeanAccum*>(d_mean_result_);
    const double mean_x = static_cast<double>(h_mean->wx);
    const double mean_y = static_cast<double>(h_mean->wy);
    const double mean_yaw = std::atan2(
        static_cast<double>(h_mean->w_sin),
        static_cast<double>(h_mean->w_cos));

    launch_gpu_compute_covariance(
        d_active_particles_, d_weights_.ptr(),
        static_cast<float>(mean_x),
        static_cast<float>(mean_y),
        static_cast<float>(mean_yaw),
        d_cov_contrib_, d_cov_result_,
        d_est_temp_, est_temp_bytes_,
        n_, stream_.get());
    CUDA_CHECK(cudaStreamSynchronize(stream_.get()));

    const CovAccum* h_cov = static_cast<const CovAccum*>(d_cov_result_);
    const double cloud_std = std::sqrt(
        std::max(0.0, static_cast<double>(h_cov->c00 + h_cov->c11)));
    if (cloud_std > cfg_.local_roughening_max_cloud_std_m) {
        return;
    }

    const double ratio = std::clamp(cfg_.local_roughening_ratio, 0.0, 1.0);
    int roughen_count = static_cast<int>(std::round(ratio * static_cast<double>(n_)));
    roughen_count = std::clamp(roughen_count, 1, n_);

    std::normal_distribution<float> xy_noise(
        0.0f, static_cast<float>(cfg_.local_roughening_xy_std_m));
    std::normal_distribution<float> yaw_noise(
        0.0f, static_cast<float>(cfg_.local_roughening_yaw_std_rad));

    std::vector<float> particles(static_cast<size_t>(roughen_count) * 3);
    int filled = 0;
    for (int attempt = 0; attempt < roughen_count * 10 && filled < roughen_count; ++attempt) {
        const float wx = static_cast<float>(mean_x) + xy_noise(rng_);
        const float wy = static_cast<float>(mean_y) + xy_noise(rng_);
        const float yaw_raw = static_cast<float>(mean_yaw) + yaw_noise(rng_);
        const float yaw = std::atan2(std::sin(yaw_raw), std::cos(yaw_raw));

        int mx = 0;
        int my = 0;
        map_->world_to_map(wx, wy, mx, my);
        if (!map_->is_free(mx, my)) {
            continue;
        }

        particles[filled * 3 + 0] = wx;
        particles[filled * 3 + 1] = wy;
        particles[filled * 3 + 2] = yaw;
        ++filled;
    }

    if (filled <= 0) {
        return;
    }

    CUDA_CHECK(cudaMemcpyAsync(
        d_active_particles_ + (n_ - filled) * 3,
        particles.data(),
        static_cast<size_t>(filled) * 3 * sizeof(float),
        cudaMemcpyHostToDevice,
        stream_.get()));
    CUDA_CHECK(cudaStreamSynchronize(stream_.get()));
}

// ─── Predict ────────────────────────────────────────────────────────
void ParticleFilter::predict(float dx, float dy, float dtheta) {
    last_stage_diag_.predict_ms = timed_stream_stage(stream_.get(), [&]() {
        motion_.apply(d_active_particles_, n_,
                      dx, dy, dtheta,
                      stream_.get());
    });
}

// ─── Update ─────────────────────────────────────────────────────────
void ParticleFilter::update(const float* ranges, int num_ranges,
                            float angle_min, float angle_inc) {
    if (update_weights(ranges, num_ranges, angle_min, angle_inc)) {
        resample_if_needed();
    }
}

bool ParticleFilter::update_weights(const float* ranges, int num_ranges,
                                    float angle_min, float angle_inc) {
    const auto update_start = std::chrono::high_resolution_clock::now();

    // Guard: ensure scan fits in pre-allocated buffer (set by max_beams param)
    if (num_ranges > max_ranges_) {
        std::fprintf(stderr,
                     "[gpu_amcl_cpp][ParticleFilter] ERROR: num_ranges (%d) exceeds max_ranges_ (%d). "
                     "Increase max_beams in config.\n",
                     num_ranges, max_ranges_);
        return false;
    }

    // Copy to pinned staging, then async DMA to device.
    memcpy(h_ranges_pinned_, ranges, num_ranges * sizeof(float));
    const size_t scan_bytes = static_cast<size_t>(num_ranges) * sizeof(float);
    last_transfer_diag_.scan_upload_ms = timed_memcpy_async(
        d_ranges_.ptr(), h_ranges_pinned_, scan_bytes,
        cudaMemcpyHostToDevice, stream_.get());
    last_transfer_diag_.scan_upload_bytes = scan_bytes;
    last_transfer_diag_.particle_download_ms = std::numeric_limits<double>::quiet_NaN();
    last_transfer_diag_.weight_download_ms = std::numeric_limits<double>::quiet_NaN();
    last_transfer_diag_.particle_download_bytes = 0;
    last_transfer_diag_.weight_download_bytes = 0;
    last_transfer_diag_.active_particles = n_;
    ++last_transfer_diag_.sequence;

    // §5: Reuse persistent log-weight buffer (no per-frame alloc).
    sensor_.compute_weights(d_active_particles_, n_,
                            d_ranges_.ptr(), num_ranges,
                            angle_min, angle_inc,
                            d_log_w_.ptr(), stream_.get());
    last_stage_diag_.sensor_model_ms = timed_stream_stage(stream_.get(), []() {});

    // §1: GPU-side weight normalization — all on GPU, no host transfers.
    // Steps: CUB::Max(log_w) → exp-shift-mul → CUB::Sum → normalize.
    last_stage_diag_.normalize_ms = timed_stream_stage(stream_.get(), [&]() {
        launch_gpu_normalize_weights(
            d_log_w_.ptr(), d_weights_.ptr(), d_scratch_w_.ptr(),
            d_max_val_, d_sum_val_,
            d_cub_temp_, cub_temp_bytes_,
            n_, stream_.get());
    });

    // After normalize, d_scratch_w_ holds the normalised weights — swap.
    std::swap(d_weights_, d_scratch_w_); // Pointer swap, no copy, no sync needed. d_weights_ now has normalised weights for resampling.

    const auto confidence_start = std::chrono::high_resolution_clock::now();
    update_scan_confidence(num_ranges);
    const auto confidence_stop = std::chrono::high_resolution_clock::now();
    last_stage_diag_.scan_confidence_ms =
        std::chrono::duration<double, std::milli>(
            confidence_stop - confidence_start).count();

    const auto update_stop = std::chrono::high_resolution_clock::now();
    last_stage_diag_.update_weights_total_ms =
        std::chrono::duration<double, std::milli>(
            update_stop - update_start).count();
    return true;
}

void ParticleFilter::resample_if_needed() {
    const auto start = std::chrono::high_resolution_clock::now();
    check_resample();
    const auto stop = std::chrono::high_resolution_clock::now();
    last_stage_diag_.resample_ms =
        std::chrono::duration<double, std::milli>(stop - start).count();
}

// ─── Resampling ─────────────────────────────────────────────────────
void ParticleFilter::update_scan_confidence(int num_ranges) {
    last_scan_confidence_bad_ = false;
    last_scan_log_weight_per_beam_ = 0.0;
    if (!cfg_.enable_local_roughening || num_ranges <= 0 || sensor_max_beams_ <= 0) {
        return;
    }

    float max_log_weight = 0.0f;
    CUDA_CHECK(cudaMemcpyAsync(&max_log_weight, d_max_val_, sizeof(float),
                               cudaMemcpyDeviceToHost, stream_.get()));
    CUDA_CHECK(cudaStreamSynchronize(stream_.get()));

    const int step = std::max(1, num_ranges / sensor_max_beams_);
    const int beam_count = std::max(1, (num_ranges + step - 1) / step);
    last_scan_log_weight_per_beam_ =
        static_cast<double>(max_log_weight) / static_cast<double>(beam_count);
    last_scan_confidence_bad_ =
        std::isfinite(last_scan_log_weight_per_beam_) &&
        last_scan_log_weight_per_beam_ < cfg_.local_roughening_bad_log_weight_per_beam;
}

void ParticleFilter::check_resample() {
    // Compute N_eff = 1 / Σ(w_i²)
    // - If all weights equal (w=1/N): N_eff = N (perfect distribution)
    // - If one weight = 1, rest = 0: N_eff = 1 (completely degenerate)
    double n_eff = resampler_.effective_sample_size(
        d_weights_.ptr(), n_, stream_.get());

    // E.g., threshold=0.5 means resample when < 50% effective particles
    if (n_eff < cfg_.resample_threshold * n_ || last_scan_confidence_bad_) {
        // If KLD enabled: compute optimal particle count adaptively
        int target = n_;
        if (cfg_.use_kld) {
            const auto start = std::chrono::high_resolution_clock::now();
            target = compute_kld_target();
            const auto stop = std::chrono::high_resolution_clock::now();
            last_stage_diag_.kld_target_ms =
                std::chrono::duration<double, std::milli>(stop - start).count();
        }
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

    roughen_local_particles();
    inject_recovery_particles();
}

int ParticleFilter::compute_kld_target() {
    const int pre_resample_particles = n_;

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
    double target = static_cast<double>(cfg_.min_particles);
    int clamped_target = cfg_.min_particles;

    if (k > 1) {
        // Fox et al. KLD formula.
        // n = (k-1) / (2ε) * [1 - 2/(9(k-1)) + sqrt(2/(9(k-1))) * z]³
        double eps = cfg_.kld_epsilon;
        double z   = cfg_.kld_z;
        double km1 = k - 1.0;
        double term = 1.0 - 2.0 / (9.0 * km1) + std::sqrt(2.0 / (9.0 * km1)) * z;
        target = (km1 / (2.0 * eps)) * term * term * term;
        clamped_target = std::clamp(
            static_cast<int>(std::ceil(target)), cfg_.min_particles, cfg_.max_particles);
    }

    last_kld_diag_.pre_resample_particles = pre_resample_particles;
    last_kld_diag_.occupied_bins = k;
    last_kld_diag_.target_unclamped = target;
    last_kld_diag_.target_clamped = clamped_target;
    last_kld_diag_.epsilon = cfg_.kld_epsilon;
    last_kld_diag_.z = cfg_.kld_z;
    last_kld_diag_.bin_x = cfg_.kld_bin_x;
    last_kld_diag_.bin_y = cfg_.kld_bin_y;
    last_kld_diag_.bin_theta = cfg_.kld_bin_theta;
    ++last_kld_diag_.sequence;

    return clamped_target;
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

PoseEstimate ParticleFilter::get_cluster_estimate(double* cluster_weight_out,
                                                  double* second_cluster_weight_out) {
    if (cluster_weight_out != nullptr) {
        *cluster_weight_out = 1.0;
    }
    if (second_cluster_weight_out != nullptr) {
        *second_cluster_weight_out = 0.0;
    }
    if (!cfg_.use_cluster_estimate ||
        cfg_.cluster_xy_bin_m <= 0.0 ||
        cfg_.cluster_radius_m <= 0.0 ||
        n_ <= 0) {
        return get_estimate();
    }

    const float radius = static_cast<float>(cfg_.cluster_radius_m);
    const float radius2 = radius * radius;
    launch_gpu_find_cluster_seed(
        d_active_particles_, d_weights_.ptr(),
        d_cluster_scores_,
        d_cluster_best_, d_cluster_second_,
        d_cluster_temp_, cluster_temp_bytes_,
        n_, radius2, stream_.get());
    CUDA_CHECK(cudaMemcpyAsync(h_cluster_best_, d_cluster_best_,
                               sizeof(ClusterScoreResult),
                               cudaMemcpyDeviceToHost, stream_.get()));
    CUDA_CHECK(cudaMemcpyAsync(h_cluster_second_, d_cluster_second_,
                               sizeof(ClusterScoreResult),
                               cudaMemcpyDeviceToHost, stream_.get()));
    CUDA_CHECK(cudaStreamSynchronize(stream_.get()));

    if (!(h_cluster_best_->score > 0.0f) || h_cluster_best_->index < 0) {
        if (cluster_weight_out != nullptr) {
            *cluster_weight_out = 0.0;
        }
        return get_estimate();
    }

    if (second_cluster_weight_out != nullptr) {
        *second_cluster_weight_out = static_cast<double>(
            std::max(0.0f, h_cluster_second_->score));
    }

    double cx = static_cast<double>(h_cluster_best_->x);
    double cy = static_cast<double>(h_cluster_best_->y);
    double ctheta = 0.0;
    const int iterations = std::clamp(cfg_.cluster_iterations, 1, 10);

    double cluster_weight = 0.0;
    for (int iter = 0; iter < iterations; ++iter) {
        launch_gpu_compute_cluster_mean(
            d_active_particles_, d_weights_.ptr(),
            static_cast<float>(cx), static_cast<float>(cy), radius2,
            d_cluster_mean_contrib_, d_cluster_mean_result_,
            d_cluster_temp_, cluster_temp_bytes_,
            n_, stream_.get());
        CUDA_CHECK(cudaMemcpyAsync(h_cluster_mean_result_, d_cluster_mean_result_,
                                   sizeof(ClusterMeanAccum),
                                   cudaMemcpyDeviceToHost, stream_.get()));
        CUDA_CHECK(cudaStreamSynchronize(stream_.get()));

        const double sum_w = static_cast<double>(h_cluster_mean_result_->w);
        if (!(sum_w > 0.0)) {
            break;
        }
        cx = static_cast<double>(h_cluster_mean_result_->wx) / sum_w;
        cy = static_cast<double>(h_cluster_mean_result_->wy) / sum_w;
        ctheta = std::atan2(
            static_cast<double>(h_cluster_mean_result_->w_sin),
            static_cast<double>(h_cluster_mean_result_->w_cos));
        cluster_weight = sum_w;
    }

    if (cluster_weight <= 0.0) {
        if (cluster_weight_out != nullptr) {
            *cluster_weight_out = 0.0;
        }
        return get_estimate();
    }
    if (cluster_weight_out != nullptr) {
        *cluster_weight_out = cluster_weight;
    }

    PoseEstimate est;
    est.x = cx;
    est.y = cy;
    est.theta = ctheta;

    launch_gpu_compute_cluster_covariance(
        d_active_particles_, d_weights_.ptr(),
        static_cast<float>(cx),
        static_cast<float>(cy),
        static_cast<float>(ctheta),
        radius2,
        d_cluster_cov_contrib_, d_cluster_cov_result_,
        d_cluster_temp_, cluster_temp_bytes_,
        n_, stream_.get());
    CUDA_CHECK(cudaMemcpyAsync(h_cluster_cov_result_, d_cluster_cov_result_,
                               sizeof(ClusterCovAccum),
                               cudaMemcpyDeviceToHost, stream_.get()));
    CUDA_CHECK(cudaStreamSynchronize(stream_.get()));

    const double sum_w = static_cast<double>(h_cluster_cov_result_->w);
    if (sum_w <= 0.0) {
        return get_estimate();
    }

    const double inv_w = 1.0 / sum_w;
    const double confidence = std::clamp(sum_w, 1e-3, 1.0);
    const double confidence_scale = std::clamp(1.0 / confidence, 1.0, 25.0);
    const double min_cov = std::max(0.0, cfg_.cluster_min_covariance);
    est.covariance(0, 0) = std::max(
        min_cov, static_cast<double>(h_cluster_cov_result_->c00) * inv_w * confidence_scale);
    est.covariance(0, 1) =
        static_cast<double>(h_cluster_cov_result_->c01) * inv_w * confidence_scale;
    est.covariance(0, 2) =
        static_cast<double>(h_cluster_cov_result_->c02) * inv_w * confidence_scale;
    est.covariance(1, 0) = est.covariance(0, 1);
    est.covariance(1, 1) = std::max(
        min_cov, static_cast<double>(h_cluster_cov_result_->c11) * inv_w * confidence_scale);
    est.covariance(1, 2) =
        static_cast<double>(h_cluster_cov_result_->c12) * inv_w * confidence_scale;
    est.covariance(2, 0) = est.covariance(0, 2);
    est.covariance(2, 1) = est.covariance(1, 2);
    est.covariance(2, 2) = std::max(
        min_cov, static_cast<double>(h_cluster_cov_result_->c22) * inv_w * confidence_scale);

    return est;
}

void ParticleFilter::get_particles(std::vector<float>& particles) {
    particles.resize(n_ * 3);
    // Use pinned staging for D->H, then copy to caller's vector.
    const size_t particle_bytes = static_cast<size_t>(n_) * 3 * sizeof(float);
    last_transfer_diag_.particle_download_ms = timed_memcpy_async(
        h_particles_pinned_, d_active_particles_, particle_bytes,
        cudaMemcpyDeviceToHost, stream_.get());
    last_transfer_diag_.particle_download_bytes = particle_bytes;
    last_transfer_diag_.weight_download_ms = std::numeric_limits<double>::quiet_NaN();
    last_transfer_diag_.weight_download_bytes = 0;
    last_transfer_diag_.active_particles = n_;
    memcpy(particles.data(), h_particles_pinned_, n_ * 3 * sizeof(float));
}

void ParticleFilter::get_particles(std::vector<float>& particles,
                                   std::vector<float>& weights) {
    particles.resize(n_ * 3);
    weights.resize(n_);
    const size_t particle_bytes = static_cast<size_t>(n_) * 3 * sizeof(float);
    const size_t weight_bytes = static_cast<size_t>(n_) * sizeof(float);
    last_transfer_diag_.particle_download_ms = timed_memcpy_async(
        h_particles_pinned_, d_active_particles_, particle_bytes,
        cudaMemcpyDeviceToHost, stream_.get());
    last_transfer_diag_.weight_download_ms = timed_memcpy_async(
        h_weights_pinned_, d_weights_.ptr(), weight_bytes,
        cudaMemcpyDeviceToHost, stream_.get());
    last_transfer_diag_.particle_download_bytes = particle_bytes;
    last_transfer_diag_.weight_download_bytes = weight_bytes;
    last_transfer_diag_.active_particles = n_;
    memcpy(particles.data(), h_particles_pinned_, n_ * 3 * sizeof(float));
    memcpy(weights.data(), h_weights_pinned_, n_ * sizeof(float));
}

}  // namespace gpu_amcl_cpp
