#pragma once

#include <cuda_runtime.h>
#include <cstddef>

namespace gpu_amcl_cpp {

struct ClusterScoreResult {
    float score = 0.0f;
    int index = -1;
    float x = 0.0f;
    float y = 0.0f;
};

struct ClusterMeanAccum {
    float w = 0.0f;
    float wx = 0.0f;
    float wy = 0.0f;
    float w_sin = 0.0f;
    float w_cos = 0.0f;
};

struct ClusterCovAccum {
    float w = 0.0f;
    float c00 = 0.0f;
    float c01 = 0.0f;
    float c02 = 0.0f;
    float c11 = 0.0f;
    float c12 = 0.0f;
    float c22 = 0.0f;
};

size_t query_cluster_score_temp_bytes(int n);
size_t query_cluster_mean_temp_bytes(int n);
size_t query_cluster_cov_temp_bytes(int n);

void launch_gpu_find_cluster_seed(
    const float* d_particles,
    const float* d_weights,
    void* d_scores,
    ClusterScoreResult* d_best,
    ClusterScoreResult* d_second,
    void* d_temp,
    size_t temp_bytes,
    int n,
    float radius2,
    cudaStream_t stream);

void launch_gpu_compute_cluster_mean(
    const float* d_particles,
    const float* d_weights,
    float cx,
    float cy,
    float radius2,
    void* d_mean_contrib,
    ClusterMeanAccum* d_mean_result,
    void* d_temp,
    size_t temp_bytes,
    int n,
    cudaStream_t stream);

void launch_gpu_compute_cluster_covariance(
    const float* d_particles,
    const float* d_weights,
    float cx,
    float cy,
    float ctheta,
    float radius2,
    void* d_cov_contrib,
    ClusterCovAccum* d_cov_result,
    void* d_temp,
    size_t temp_bytes,
    int n,
    cudaStream_t stream);

}  // namespace gpu_amcl_cpp
