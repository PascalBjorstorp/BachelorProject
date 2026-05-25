#include "gpu_amcl_cpp/core/cluster_kernels.hpp"
#include "gpu_amcl_cpp/helpers/cuda_utils.hpp"

#include <cub/cub.cuh>
#include <cmath>

namespace gpu_amcl_cpp {

namespace {

struct ClusterScoreOp {
    __host__ __device__ __forceinline__
    ClusterScoreResult operator()(const ClusterScoreResult& a,
                                  const ClusterScoreResult& b) const {
        if (b.score > a.score) {
            return b;
        }
        if (b.score == a.score && (a.index < 0 || (b.index >= 0 && b.index < a.index))) {
            return b;
        }
        return a;
    }
};

struct ClusterMeanOp {
    __host__ __device__ __forceinline__
    ClusterMeanAccum operator()(const ClusterMeanAccum& a,
                                const ClusterMeanAccum& b) const {
        return ClusterMeanAccum{
            a.w + b.w,
            a.wx + b.wx,
            a.wy + b.wy,
            a.w_sin + b.w_sin,
            a.w_cos + b.w_cos
        };
    }
};

struct ClusterCovOp {
    __host__ __device__ __forceinline__
    ClusterCovAccum operator()(const ClusterCovAccum& a,
                               const ClusterCovAccum& b) const {
        return ClusterCovAccum{
            a.w + b.w,
            a.c00 + b.c00,
            a.c01 + b.c01,
            a.c02 + b.c02,
            a.c11 + b.c11,
            a.c12 + b.c12,
            a.c22 + b.c22
        };
    }
};

__global__ void kernel_score_cluster_candidates(
    const float* __restrict__ particles,
    const float* __restrict__ weights,
    ClusterScoreResult* __restrict__ scores,
    int n,
    float radius2) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;

    float cx = particles[i * 3 + 0];
    float cy = particles[i * 3 + 1];
    float wi = weights[i];
    if (!(wi > 0.0f) || !isfinite(wi)) {
        scores[i] = ClusterScoreResult{0.0f, i, cx, cy};
        return;
    }

    float score = 0.0f;
    for (int j = 0; j < n; ++j) {
        float w = weights[j];
        if (!(w > 0.0f) || !isfinite(w)) {
            continue;
        }
        float dx = particles[j * 3 + 0] - cx;
        float dy = particles[j * 3 + 1] - cy;
        if (dx * dx + dy * dy <= radius2) {
            score += w;
        }
    }
    scores[i] = ClusterScoreResult{score, i, cx, cy};
}

__global__ void kernel_filter_second_cluster_scores(
    const ClusterScoreResult* __restrict__ in_scores,
    ClusterScoreResult* __restrict__ out_scores,
    const ClusterScoreResult* __restrict__ best,
    int n,
    float separate_radius2) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;

    ClusterScoreResult score = in_scores[i];
    float dx = score.x - best->x;
    float dy = score.y - best->y;
    if (dx * dx + dy * dy <= separate_radius2) {
        score.score = 0.0f;
    }
    out_scores[i] = score;
}

__global__ void kernel_compute_cluster_mean_contrib(
    const float* __restrict__ particles,
    const float* __restrict__ weights,
    float cx,
    float cy,
    float radius2,
    ClusterMeanAccum* __restrict__ contrib,
    int n) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;

    float w = weights[i];
    float x = particles[i * 3 + 0];
    float y = particles[i * 3 + 1];
    float dx = x - cx;
    float dy = y - cy;
    if (!(w > 0.0f) || !isfinite(w) || dx * dx + dy * dy > radius2) {
        contrib[i] = ClusterMeanAccum{0, 0, 0, 0, 0};
        return;
    }

    float theta = particles[i * 3 + 2];
    contrib[i] = ClusterMeanAccum{
        w,
        w * x,
        w * y,
        w * sinf(theta),
        w * cosf(theta)
    };
}

__global__ void kernel_compute_cluster_cov_contrib(
    const float* __restrict__ particles,
    const float* __restrict__ weights,
    float cx,
    float cy,
    float ctheta,
    float radius2,
    ClusterCovAccum* __restrict__ contrib,
    int n) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;

    float w = weights[i];
    float x = particles[i * 3 + 0];
    float y = particles[i * 3 + 1];
    float dx = x - cx;
    float dy = y - cy;
    if (!(w > 0.0f) || !isfinite(w) || dx * dx + dy * dy > radius2) {
        contrib[i] = ClusterCovAccum{0, 0, 0, 0, 0, 0, 0};
        return;
    }

    float raw_dtheta = particles[i * 3 + 2] - ctheta;
    float dtheta = atan2f(sinf(raw_dtheta), cosf(raw_dtheta));
    contrib[i] = ClusterCovAccum{
        w,
        w * dx * dx,
        w * dx * dy,
        w * dx * dtheta,
        w * dy * dy,
        w * dy * dtheta,
        w * dtheta * dtheta
    };
}

}  // namespace

size_t query_cluster_score_temp_bytes(int n) {
    size_t temp_bytes = 0;
    ClusterScoreResult* d_in = nullptr;
    ClusterScoreResult* d_out = nullptr;
    cub::DeviceReduce::Reduce(nullptr, temp_bytes,
                              d_in, d_out, n,
                              ClusterScoreOp(),
                              ClusterScoreResult{-1.0f, -1, 0.0f, 0.0f});
    return temp_bytes;
}

size_t query_cluster_mean_temp_bytes(int n) {
    size_t temp_bytes = 0;
    ClusterMeanAccum* d_in = nullptr;
    ClusterMeanAccum* d_out = nullptr;
    cub::DeviceReduce::Reduce(nullptr, temp_bytes,
                              d_in, d_out, n,
                              ClusterMeanOp(),
                              ClusterMeanAccum{0, 0, 0, 0, 0});
    return temp_bytes;
}

size_t query_cluster_cov_temp_bytes(int n) {
    size_t temp_bytes = 0;
    ClusterCovAccum* d_in = nullptr;
    ClusterCovAccum* d_out = nullptr;
    cub::DeviceReduce::Reduce(nullptr, temp_bytes,
                              d_in, d_out, n,
                              ClusterCovOp(),
                              ClusterCovAccum{0, 0, 0, 0, 0, 0, 0});
    return temp_bytes;
}

void launch_gpu_find_cluster_seed(
    const float* d_particles,
    const float* d_weights,
    void* d_scores_v,
    ClusterScoreResult* d_best,
    ClusterScoreResult* d_second,
    void* d_temp,
    size_t temp_bytes,
    int n,
    float radius2,
    cudaStream_t stream) {
    if (n <= 0) return;

    auto* d_scores = static_cast<ClusterScoreResult*>(d_scores_v);
    const auto launch = make_adaptive_launch_config(n);
    kernel_score_cluster_candidates<<<launch.grid, launch.block, 0, stream>>>(
        d_particles, d_weights, d_scores, n, radius2);
    CUDA_CHECK(cudaGetLastError());

    CUDA_CHECK(cub::DeviceReduce::Reduce(
        d_temp, temp_bytes,
        d_scores, d_best, n,
        ClusterScoreOp(),
        ClusterScoreResult{-1.0f, -1, 0.0f, 0.0f},
        stream));

    kernel_filter_second_cluster_scores<<<launch.grid, launch.block, 0, stream>>>(
        d_scores, d_scores, d_best, n, radius2);
    CUDA_CHECK(cudaGetLastError());

    CUDA_CHECK(cub::DeviceReduce::Reduce(
        d_temp, temp_bytes,
        d_scores, d_second, n,
        ClusterScoreOp(),
        ClusterScoreResult{0.0f, -1, 0.0f, 0.0f},
        stream));
}

void launch_gpu_compute_cluster_mean(
    const float* d_particles,
    const float* d_weights,
    float cx,
    float cy,
    float radius2,
    void* d_mean_contrib_v,
    ClusterMeanAccum* d_mean_result,
    void* d_temp,
    size_t temp_bytes,
    int n,
    cudaStream_t stream) {
    if (n <= 0) return;

    auto* d_mean_contrib = static_cast<ClusterMeanAccum*>(d_mean_contrib_v);
    const auto launch = make_adaptive_launch_config(n);
    kernel_compute_cluster_mean_contrib<<<launch.grid, launch.block, 0, stream>>>(
        d_particles, d_weights, cx, cy, radius2, d_mean_contrib, n);
    CUDA_CHECK(cudaGetLastError());

    CUDA_CHECK(cub::DeviceReduce::Reduce(
        d_temp, temp_bytes,
        d_mean_contrib, d_mean_result, n,
        ClusterMeanOp(),
        ClusterMeanAccum{0, 0, 0, 0, 0},
        stream));
}

void launch_gpu_compute_cluster_covariance(
    const float* d_particles,
    const float* d_weights,
    float cx,
    float cy,
    float ctheta,
    float radius2,
    void* d_cov_contrib_v,
    ClusterCovAccum* d_cov_result,
    void* d_temp,
    size_t temp_bytes,
    int n,
    cudaStream_t stream) {
    if (n <= 0) return;

    auto* d_cov_contrib = static_cast<ClusterCovAccum*>(d_cov_contrib_v);
    const auto launch = make_adaptive_launch_config(n);
    kernel_compute_cluster_cov_contrib<<<launch.grid, launch.block, 0, stream>>>(
        d_particles, d_weights, cx, cy, ctheta, radius2, d_cov_contrib, n);
    CUDA_CHECK(cudaGetLastError());

    CUDA_CHECK(cub::DeviceReduce::Reduce(
        d_temp, temp_bytes,
        d_cov_contrib, d_cov_result, n,
        ClusterCovOp(),
        ClusterCovAccum{0, 0, 0, 0, 0, 0, 0},
        stream));
}

}  // namespace gpu_amcl_cpp
