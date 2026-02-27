# CUDA Optimization Research for GPU-Accelerated AMCL

**Target hardware:** NVIDIA Jetson Orin (Ampere, sm_87) / RTX 4070 Laptop (Ada Lovelace, sm_89)  
**CUDA version:** 12.x  
**Workload:** 1000–5000 particles, 270 laser beams, 2000×2000 distance field  
**Date:** 2026-02-26

---

## Table of Contents

| # | Optimization | Impact | Section |
|---|-------------|--------|---------|
| 1 | GPU-side weight normalization (CUB DeviceReduce) | **HIGH** | [§1](#1-gpu-side-weight-normalization) |
| 2 | CUB DeviceScan for prefix sums | **HIGH** | [§2](#2-cub-devicescan-for-prefix-sums) |
| 3 | Kernel fusion (predict+update, normalize+resample) | **HIGH** | [§3](#3-kernel-fusion) |
| 4 | Pinned memory (cudaMallocHost) | **MEDIUM-HIGH** | [§4](#4-pinned-memory) |
| 5 | Persistent buffer reuse vs per-frame allocation | **MEDIUM-HIGH** | [§5](#5-persistent-buffer-reuse) |
| 6 | cudaMemcpyAsync with multiple streams | **MEDIUM** | [§6](#6-async-memcpy-with-multiple-streams) |
| 7 | Texture memory / `__ldg()` for distance field | **MEDIUM** | [§7](#7-texture-memory--__ldg-for-distance-field) |
| 8 | Shared memory tiling for scan ranges | **MEDIUM** | [§8](#8-shared-memory-tiling-for-scan-ranges) |
| 9 | Warp-level primitives for reductions | **LOW-MEDIUM** | [§9](#9-warp-level-primitives) |
| **10** | **Pipeline latency reduction (scan→EKF)** | **CRITICAL** | [§10](#10-pipeline-latency-reduction) |

*Impact ranked by expected wall-clock improvement for this specific workload (≤5000 particles, 270 beams, 2000×2000 map) on Jetson.*

> **NOTE:** Section 10 addresses the **end-to-end pipeline latency** (scan→EKF pose), which at ~13ms dwarfs the GPU kernel time (~0.4ms). This is the #1 priority for racing performance.

---

## 1. GPU-Side Weight Normalization

### Current Bottleneck

In `particle_filter.cpp` (`update()`, lines 73–131), the weight normalization pipeline is:

```
GPU: compute log-weights  →  cudaMemcpy D→H  →  CPU: find max, subtract, exp, multiply old_w, sum, normalize  →  cudaMemcpy H→D
```

This involves **two synchronous GPU↔CPU transfers** (download log_w + download old_w, upload final weights) plus CPU-side sequential loops over N particles. For 5000 particles, that's:
- 2× download: `5000 × 4B = 20 KB` each (40 KB total)
- 1× upload: 20 KB
- 3× sequential CPU loops over 5000 elements
- 2× implicit `cudaDeviceSynchronize()` from synchronous `cudaMemcpy`

The **latency** of each synchronous transfer is dominated by launch overhead (~5–15 µs on Jetson, even for tiny payloads), not bandwidth. The CPU loops add ~5–20 µs depending on cache state. Total: **~30–80 µs** per update, which is significant when the entire PF cycle targets <1 ms.

### Solution: CUB DeviceReduce + Element-wise Kernels

Replace the entire CPU normalization with three GPU operations:

```
Step 1:  CUB::DeviceReduce::Max(d_log_w)        → d_max_lw
Step 2:  Custom kernel: w[i] = exp(log_w[i] - max_lw) * old_w[i]
Step 3:  CUB::DeviceReduce::Sum(d_weights)       → d_sum
Step 4:  Custom kernel: w[i] /= sum
```

All four steps stay on GPU. Zero host↔device transfers for normalization.

### Implementation

```cpp
#include <cub/cub.cuh>

// One-time setup (in init()):
void* d_temp_storage = nullptr;
size_t temp_bytes = 0;

// Query temp storage for Max (use the larger of Max/Sum)
cub::DeviceReduce::Max(nullptr, temp_bytes, d_log_w, d_max, n);
size_t max_temp = temp_bytes;
cub::DeviceReduce::Sum(nullptr, temp_bytes, d_weights, d_sum, n);
temp_bytes = std::max(temp_bytes, max_temp);
cudaMalloc(&d_temp_storage, temp_bytes);

// Also allocate scalar outputs on device:
float* d_max_val;  cudaMalloc(&d_max_val, sizeof(float));
float* d_sum_val;  cudaMalloc(&d_sum_val, sizeof(float));
```

```cuda
// Fused exp-shift-multiply kernel
__global__
void kernel_exp_shift_mul(const float* __restrict__ log_w,
                          const float* __restrict__ old_w,
                          float* __restrict__ out_w,
                          const float* __restrict__ d_max,
                          int n) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    out_w[i] = expf(log_w[i] - *d_max) * old_w[i];
}

// Normalize kernel
__global__
void kernel_normalize(float* __restrict__ w,
                      const float* __restrict__ d_sum,
                      int n) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    w[i] /= *d_sum;
}
```

```cpp
// In update():
// Step 1: find max log-weight
cub::DeviceReduce::Max(d_temp_, temp_bytes_, d_log_w, d_max_val_, n, stream);

// Step 2: exp-shift and multiply with existing weights
kernel_exp_shift_mul<<<grid, 256, 0, stream>>>(
    d_log_w, d_weights_, d_scratch_w_, d_max_val_, n);

// Step 3: sum
cub::DeviceReduce::Sum(d_temp_, temp_bytes_, d_scratch_w_, d_sum_val_, n, stream);

// Step 4: normalize in-place
kernel_normalize<<<grid, 256, 0, stream>>>(d_scratch_w_, d_sum_val_, n);

// d_scratch_w_ is now the normalized weight array — swap into d_weights_
std::swap(d_weights_, d_scratch_w_);
```

### Why It's Fast

| Aspect | Before (CPU) | After (GPU CUB) |
|--------|-------------|-----------------|
| Max finding | `std::max_element` O(N) CPU | CUB single-pass parallel reduction |
| Log-sum-exp | Sequential CPU loop | 1 kernel launch, N threads |
| Sum | `std::accumulate` O(N) CPU | CUB single-pass parallel reduction |
| Normalize | Sequential CPU loop | 1 kernel launch, N threads |
| Host↔Device | 3 × `cudaMemcpy` syncs | **0 transfers** |
| Sync points | 3 implicit syncs | 0 (all async on stream) |

**CUB DeviceReduce** uses a multi-level tree reduction that runs in O(N/P) time per SM. For N=5000, it completes in ~2–5 µs on Jetson Orin. The whole 4-step pipeline takes ~10–20 µs vs ~30–80 µs before.

### Expected Speedup

- **Eliminates ~30–80 µs** of synchronous transfers + CPU work per PF update
- On Jetson (where CPU is weaker and PCIe overhead is higher), expect **~50–70 µs savings**
- Percentage of total cycle: if full PF cycle is ~200–500 µs, this is a **15–35% improvement**
- Bonus: removes 2–3 synchronization points, enabling overlap with other work

### References

- [CUB documentation — DeviceReduce](https://nvidia.github.io/cccl/cub/api/structcub_1_1DeviceReduce.html)
- [CUB is header-only since CUDA 11, bundled in CCCL](https://github.com/NVIDIA/cccl)
- Merrill & Grimshaw, "Parallel Scan for Stream Architectures" (2009) — foundation of CUB's algorithms
- NVIDIA Best Practices Guide §9.1: "Minimize data transfers between host and device"

---

## 2. CUB DeviceScan for Prefix Sums

### Current Bottleneck

In `resampling_kernels.cu`, the inclusive prefix sum is:

```cuda
__global__
void kernel_inclusive_scan(const float* weights, float* cumsum, int n) {
    if (threadIdx.x != 0 || blockIdx.x != 0) return;  // SINGLE THREAD
    float s = 0.0f;
    for (int i = 0; i < n; ++i) {
        s += weights[i];
        cumsum[i] = s;
    }
}
```

This is a **sequential O(N) kernel running on a single GPU thread**. A single CUDA thread runs at ~1–1.5 GHz (Jetson Orin) with no ILP, so for N=5000:
- 5000 iterations × ~3 instructions × ~1 ns each ≈ **~15 µs**
- But worse: only 1 of 2048+ cores is active → essentially 0% GPU utilization
- The GPU scheduler keeps the entire SM occupied with 1 warp doing sequential work

### Solution: CUB DeviceScan::InclusiveSum

```cpp
#include <cub/cub.cuh>

// One-time: query temp storage
size_t scan_temp_bytes = 0;
cub::DeviceScan::InclusiveSum(nullptr, scan_temp_bytes, d_weights, d_cumsum, n);
cudaMalloc(&d_scan_temp, scan_temp_bytes);

// Per-frame:
cub::DeviceScan::InclusiveSum(d_scan_temp, scan_temp_bytes,
                              d_weights, d_cumsum, n, stream);
```

### Why CUB Is Superior

CUB implements the **Blelloch (1990) work-efficient parallel scan** with hardware-optimized modifications:

1. **Three-phase algorithm:**
   - **Upsweep (reduce):** Each block reduces its tile using shared memory; results stored to global memory
   - **Spine scan:** Reduce block totals (single block for small N)
   - **Downsweep:** Apply inter-block offsets

2. **For N=5000 (single-block case):** CUB detects this fits in one block and uses a single-block scan — no inter-block coordination needed. Uses warp-level `__shfl_up_sync()` for the intra-warp scan, then shared memory for inter-warp combining.

3. **Throughput:** CUB scan achieves ~20–40 GB/s on Jetson Orin, meaning 5000 × 4B = 20 KB takes ~1 µs.

| Implementation | Time (est., N=5000) | Speedup |
|---------------|-------|---------|
| Single-thread sequential kernel | ~15 µs | 1× |
| CUB DeviceScan::InclusiveSum | ~1–2 µs | **~10–15×** |

### Also Replace `kernel_sum_sq`

The `kernel_sum_sq` for N_eff has the same single-thread problem. Replace with:

```cpp
// Fuse into: square weights in a kernel, then CUB::DeviceReduce::Sum
__global__
void kernel_square(const float* in, float* out, int n) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) out[i] = in[i] * in[i];
}

// Then:
kernel_square<<<grid, 256, 0, stream>>>(d_weights, d_sq_weights, n);
cub::DeviceReduce::Sum(d_temp, temp_bytes, d_sq_weights, d_sumsq, n, stream);
```

Or use `cub::TransformInputIterator` to avoid the temporary:

```cpp
struct SquareOp {
    __device__ float operator()(float x) const { return x * x; }
};

cub::TransformInputIterator<float, SquareOp, const float*>
    sq_iter(d_weights, SquareOp{});
cub::DeviceReduce::Sum(d_temp, temp_bytes, sq_iter, d_sumsq, n, stream);
```

### Expected Speedup

- Prefix sum: **~13 µs savings** per resample
- Sum-of-squares: **~10 µs savings** per N_eff check
- Combined: **~23 µs / update cycle**

### References

- [CUB DeviceScan documentation](https://nvidia.github.io/cccl/cub/api/structcub_1_1DeviceScan.html)
- Blelloch, "Prefix Sums and Their Applications" (1990), CMU-CS-90-190
- Merrill & Garland, "Single-pass Parallel Prefix Scan with Decoupled Look-back" (2016) — the algorithm CUB uses internally
- Harris, Sengupta & Owens, "Parallel Prefix Sum (Scan) with CUDA" (GPU Gems 3, Ch. 39)

---

## 3. Kernel Fusion

### Current Launch Sequence

Each PF update cycle currently launches **at minimum** these kernels (from `particle_filter.cpp::update()`):

```
1. kernel_motion_update          (predict)
2. cudaMemcpy H→D               (upload scan ranges)
3. kernel_sensor_weights         (sensor model → log_w)
4. cudaMemcpy D→H               (download log_w)
5. [CPU normalization]
6. cudaMemcpy D→H               (download old_w)
7. cudaMemcpy H→D               (upload normalized_w)
8. kernel_sum_sq                 (N_eff)
9. cudaMemcpy D→H               (download N_eff result)
10. kernel_inclusive_scan         (prefix sum)
11. kernel_systematic_resample   (resample)
12. cudaMemcpyAsync D→D          (copy new→old particles)
```

That's **6 kernel launches + 5 memcpy operations + at least 3 synchronization points**.

Each CUDA kernel launch has overhead:
- **Driver-side:** ~3–10 µs on Linux (CUDA driver runtime)
- **On Jetson (tegra):** ~5–15 µs per launch due to unified memory architecture overhead
- For 6 launches: **~30–90 µs** of pure launch overhead

### Fusion Opportunity A: Normalize + Prefix-Sum + Resample

After implementing GPU-side normalization (§1), the normalize → scan → resample sequence becomes 4 kernel launches that operate on the same N-element array sequentially. These can be partially fused:

**Fused Normalize-and-Scan Kernel:**

```cuda
// Fuses: exp-shift-multiply + normalize + inclusive scan
// into a single kernel for small N (≤ one block = ≤ 1024 threads)
__global__
void kernel_normalize_and_scan(const float* __restrict__ log_w,
                               const float* __restrict__ old_w,
                               float* __restrict__ norm_w,
                               float* __restrict__ cumsum,
                               float max_lw,  // from prior CUB reduce
                               int n) {
    extern __shared__ float smem[];  // size = 2 * blockDim.x

    int tid = threadIdx.x;
    int i = blockIdx.x * blockDim.x + tid;

    // Phase 1: exp-shift-multiply (each thread loads its value)
    float val = 0.0f;
    if (i < n) {
        val = expf(log_w[i] - max_lw) * old_w[i];
    }
    smem[tid] = val;
    __syncthreads();

    // Phase 2: block-level sum reduction for normalization constant
    // (Separate shared mem region for reduction)
    float* reduce_buf = &smem[blockDim.x];
    reduce_buf[tid] = val;
    __syncthreads();
    for (int s = blockDim.x / 2; s > 0; s >>= 1) {
        if (tid < s) reduce_buf[tid] += reduce_buf[tid + s];
        __syncthreads();
    }
    float total = reduce_buf[0];

    // Phase 3: normalize
    if (total > 0.0f) val /= total;
    smem[tid] = val;
    __syncthreads();

    // Phase 4: inclusive scan (Hillis-Steele, in-place in shared mem)
    for (int offset = 1; offset < blockDim.x; offset <<= 1) {
        float temp = (tid >= offset) ? smem[tid - offset] : 0.0f;
        __syncthreads();
        smem[tid] += temp;
        __syncthreads();
    }

    // Write results
    if (i < n) {
        norm_w[i] = val;
        cumsum[i] = smem[tid];
    }
}
```

**Caveat:** This single-block approach only works for N ≤ 1024 (one block). For N=2000–5000, you'd need a multi-block approach or stick with separate CUB calls. For the common case of N=2000, use two blocks with a block-level scan + inter-block fixup.

**Practical recommendation for N=1000–5000:** Keep CUB for the reductions (§1) but fuse the normalize kernel and scan kernel if N is small enough, or simply accept 4 separate launches with CUB providing optimal throughput. The main win is already from removing host transfers.

### Fusion Opportunity B: Predict + Update (NOT Recommended)

The motion model kernel (`kernel_motion_update`) and sensor model kernel (`kernel_sensor_weights`) operate on overlapping data (`d_particles`). However, fusing them has problems:
- Motion writes and sensor reads `d_particles` → same kernel, no sync needed
- But sensor also needs `d_ranges` which must be uploaded first
- And they have very different register pressure (motion uses curand → ~40 regs; sensor has a loop over 270 beams)

**Verdict:** Fusing predict+update is possible but increases register pressure, likely reducing occupancy. **Not recommended.** Instead, rely on the stream's implicit ordering — back-to-back launches on the same stream have only ~3–5 µs gap.

### Fusion Opportunity C: Resample Binary Search + Copy

The systematic resample kernel and the subsequent D→D memcpy can be fused by writing directly to the output buffer:

```cuda
// Already writes to new_particles — just swap pointers instead of memcpy
// Use double-buffering (see §5)
```

This eliminates the `cudaMemcpyAsync D→D` and the `cudaStreamSynchronize` after it.

### Expected Speedup

- Eliminating 2 kernel launches: **~10–30 µs**
- Eliminating D→D memcpy + sync: **~5–10 µs**
- Total: **~15–40 µs** savings per update

### References

- NVIDIA Best Practices Guide §12.1: "Kernel Launch Overhead"
- Volkov, "Understanding Latency Hiding on GPUs" (2016) — analysis of launch overhead
- NVIDIA blog: "CUDA Pro Tip: Increase Performance with Vectorized Memory Access"

---

## 4. Pinned Memory

### Current Bottleneck

All host↔device transfers currently use **pageable** host memory (standard `std::vector<float>` / stack allocations). In `particle_filter.cpp`:

```cpp
std::vector<float> log_w(n_);          // pageable
d_log_w.download(log_w.data(), n_);    // cudaMemcpy with pageable host ptr

std::vector<float> weights(n_);        // pageable
d_weights_.upload(weights.data(), n_); // cudaMemcpy with pageable host ptr
```

With pageable memory, `cudaMemcpy` must:
1. Allocate (or reuse) an internal pinned staging buffer
2. Copy from pageable → pinned (CPU memcpy, page-fault prone)
3. DMA from pinned → GPU
4. The call cannot return until step 3 completes (it's synchronous)

With **pinned (page-locked) memory**, `cudaMemcpy` can DMA directly — no staging copy.

### Solution

```cpp
// Allocate pinned host buffers (once, in init)
float* h_particles_pinned;
float* h_weights_pinned;
float* h_ranges_pinned;

cudaMallocHost(&h_particles_pinned, max_particles * 3 * sizeof(float));
cudaMallocHost(&h_weights_pinned,   max_particles * sizeof(float));
cudaMallocHost(&h_ranges_pinned,    max_beams * sizeof(float));  // 270 beams

// Use in update():
// Instead of: d_ranges.upload(ranges, num_ranges);
memcpy(h_ranges_pinned, ranges, num_ranges * sizeof(float));
cudaMemcpyAsync(d_ranges, h_ranges_pinned,
                num_ranges * sizeof(float),
                cudaMemcpyHostToDevice, stream);

// Clean up in destructor:
cudaFreeHost(h_particles_pinned);
cudaFreeHost(h_weights_pinned);
cudaFreeHost(h_ranges_pinned);
```

### Why It's Faster

| Transfer Type | Mechanism | Overhead |
|--------------|-----------|----------|
| Pageable → Device | CPU copy to staging + DMA | ~8–12 µs (small), bandwidth limited for larger |
| Pinned → Device | Direct DMA | ~3–5 µs (small), full bandwidth |
| Pageable (async) | **Not truly async** — falls back to sync | Same as pageable sync |
| Pinned (async) | **Truly async** — returns immediately | 0 µs CPU-side (DMA in background) |

**Critical insight:** `cudaMemcpyAsync` only works asynchronously with **pinned** host memory. With pageable memory, it silently degrades to synchronous behavior. This is documented in the CUDA Programming Guide §3.2.6.

### Benchmarks from NVIDIA Documentation

From the CUDA C++ Programming Guide and various benchmarks:

| Transfer Size | Pageable Bandwidth | Pinned Bandwidth | Speedup |
|--------------|-------------------|------------------|---------|
| 4 KB (1000 floats) | ~0.5 GB/s | ~2 GB/s | ~4× |
| 20 KB (5000 floats) | ~1.5 GB/s | ~6 GB/s | ~4× |
| 60 KB (5000×3 floats) | ~3 GB/s | ~10 GB/s | ~3× |
| 16 MB (4M floats) | ~6 GB/s | ~12 GB/s | ~2× |

For **small transfers** (which is our case — 4–80 KB), the speedup is **larger** because launch overhead dominates and pinned memory avoids the staging copy entirely.

On **Jetson Orin** (unified memory architecture), the difference is smaller since CPU and GPU share physical memory, but pinned memory still prevents page migration/faults:
- Pageable: ~2–5 µs overhead per transfer (page fault handling)
- Pinned: ~0.5–1 µs overhead (direct access)

### Pinned Memory Caveats

1. **Don't over-allocate:** Pinned memory is non-pageable, reducing available system RAM. On Jetson (8–32 GB shared), keep total pinned allocation under ~100 MB.
2. **Allocation is slow:** `cudaMallocHost` takes ~1 ms — always pre-allocate, never allocate per-frame.
3. **Our budget:** `5000 × 3 × 4B + 5000 × 4B + 270 × 4B ≈ 81 KB` — negligible.

### Expected Speedup

- Per transfer: ~3–8 µs savings (for our small sizes)
- 3–5 transfers per cycle: **~10–30 µs total savings**
- After implementing §1 (GPU-side normalization), only 1–2 transfers remain (scan upload, estimate download), so savings reduce to **~5–15 µs**
- **Main enabler for §6** (async streams) — without pinned memory, async transfers don't actually work

### References

- CUDA C++ Programming Guide §3.2.6: "Page-Locked Host Memory"
- NVIDIA Best Practices Guide §9.1.2: "Pinned Memory"
- [CUDA Samples — bandwidthTest](https://github.com/NVIDIA/cuda-samples/tree/master/Samples/1_Utilities/bandwidthTest) — benchmark tool
- Harris, "How to Optimize Data Transfers in CUDA C/C++" (NVIDIA Developer Blog, 2012)

---

## 5. Persistent Buffer Reuse

### Current Bottleneck

In `particle_filter.cpp::update()`:

```cpp
void ParticleFilter::update(const float* ranges, int num_ranges,
                            float angle_min, float angle_inc) {
    // Per-frame allocation!
    DeviceBuffer<float> d_ranges(num_ranges);    // cudaMalloc
    d_ranges.upload(ranges, num_ranges);

    DeviceBuffer<float> d_log_w(n_);             // cudaMalloc
    sensor_.compute_weights(..., d_log_w.ptr(), ...);
    // d_ranges and d_log_w destructed → cudaFree
}
```

**Every single update cycle** calls `cudaMalloc` twice and `cudaFree` twice. `cudaMalloc` / `cudaFree` are globally synchronizing operations:

- `cudaMalloc` grabs the device memory allocator mutex, which may block other streams
- On Jetson, `cudaMalloc` takes **~50–200 µs** depending on fragmentation
- `cudaFree` similarly takes **~10–50 µs**
- Total: **~120–500 µs per update** just from allocations!

This is very likely the **single largest bottleneck** in the current implementation.

### Solution: Pre-allocate All Buffers

```cpp
// In ParticleFilter::init() or a separate allocate_buffers():
class ParticleFilter {
    // ... existing members ...
    DeviceBuffer<float> d_ranges_;     // persistent scan buffer
    DeviceBuffer<float> d_log_w_;      // persistent log-weight buffer
    DeviceBuffer<float> d_scratch_w_;  // for normalize swap (§1)

    // Double-buffer for particles (eliminates D→D copy in resample)
    DeviceBuffer<float> d_particles_a_;
    DeviceBuffer<float> d_particles_b_;
    float* d_active_particles_;  // points to a_ or b_
};

void ParticleFilter::init(...) {
    // Allocate for worst case
    d_ranges_.allocate(max_num_ranges);  // e.g., 1080 for a typical LiDAR
    d_log_w_.allocate(cfg_.max_particles);
    d_scratch_w_.allocate(cfg_.max_particles);

    d_particles_a_.allocate(cfg_.max_particles * 3);
    d_particles_b_.allocate(cfg_.max_particles * 3);
    d_active_particles_ = d_particles_a_.ptr();
}

void ParticleFilter::update(const float* ranges, int num_ranges,
                            float angle_min, float angle_inc) {
    // Reuse existing buffer — zero allocations!
    d_ranges_.upload(ranges, num_ranges);  // just memcpy, no malloc

    sensor_.compute_weights(d_active_particles_, n_,
                            d_ranges_.ptr(), num_ranges,
                            angle_min, angle_inc,
                            d_log_w_.ptr(), stream_.get());
    // ... rest of update, no allocations ...
}
```

### Double-Buffering for Resample

Currently in `resampling.cpp`:

```cpp
// Resample writes to d_new_particles_
launch_systematic_resample(cumsum, d_particles, d_new_particles_, ...);
// Then copies back:
cudaMemcpyAsync(d_particles, d_new_particles_, ...);  // D→D copy
cudaStreamSynchronize(stream);  // blocks!
```

With double-buffering, the resample kernel writes to the inactive buffer, then you just swap pointers:

```cpp
// Resample writes to inactive buffer
float* inactive = (d_active_particles_ == d_particles_a_.ptr())
                  ? d_particles_b_.ptr() : d_particles_a_.ptr();
launch_systematic_resample(cumsum, d_active_particles_, inactive, ...);
// Swap — no copy, no sync
d_active_particles_ = inactive;
```

This eliminates:
- The `cudaMemcpyAsync` D→D (20–60 KB transfer): ~5–10 µs
- The `cudaStreamSynchronize`: ~1–3 µs

### Expected Speedup

- Eliminating 2× `cudaMalloc` per frame: **~100–400 µs savings**
- Eliminating 2× `cudaFree` per frame: **~20–100 µs savings**
- Eliminating D→D memcpy via double-buffering: **~5–10 µs**
- Eliminating 1 sync point: **~1–3 µs**
- Total: **~130–510 µs / update** — potentially the **largest single improvement**

### References

- CUDA Best Practices Guide §9.3: "Avoid unnecessary allocation/deallocation"
- NVIDIA blog: "CUDA Pro Tip: Use caching allocators for faster memory management"
- CUDA Memory Management docs: cudaMalloc is synchronizing
- `cudaMemPool` (CUDA 11.2+) as an alternative for amortized allocation cost

---

## 6. Async Memcpy with Multiple Streams

### Current Bottleneck

The current implementation uses a single CUDA stream and synchronous transfers. Each `cudaMemcpy` blocks until complete. The particle filter cycle is fully serial:

```
[upload ranges]──[sensor kernel]──[download log_w]──[CPU work]──[upload weights]──[scan kernel]──[resample kernel]──[download estimate]
```

### Solution: Multi-Stream Pipeline

With pinned memory (§4) and GPU-side normalization (§1), the remaining transfers can overlap with compute:

```
Stream 1 (compute):  [motion_kernel]──[sensor_kernel]──[normalize]──[scan]──[resample]
Stream 2 (transfer):      [upload ranges]──────────────[download estimate for publish]
                          ↑                             ↑
                          H2D overlaps with motion      D2H overlaps with next motion
```

```cpp
// Create two streams
cudaStream_t compute_stream, transfer_stream;
cudaStreamCreate(&compute_stream);
cudaStreamCreate(&transfer_stream);

// Create events for synchronization
cudaEvent_t ranges_uploaded, weights_ready;
cudaEventCreate(&ranges_uploaded);
cudaEventCreate(&weights_ready);

// --- Per-frame pipeline ---

// 1. Start uploading ranges on transfer stream (truly async with pinned mem)
cudaMemcpyAsync(d_ranges, h_ranges_pinned, size,
                cudaMemcpyHostToDevice, transfer_stream);
cudaEventRecord(ranges_uploaded, transfer_stream);

// 2. Meanwhile, run motion update on compute stream
launch_motion_update(..., compute_stream);

// 3. Compute stream waits for ranges upload before sensor model
cudaStreamWaitEvent(compute_stream, ranges_uploaded);
launch_sensor_weights(..., compute_stream);

// 4. Rest of pipeline on compute stream (all GPU-side with §1)
// [normalize] → [scan] → [resample]

// 5. Record when weights are ready
cudaEventRecord(weights_ready, compute_stream);

// 6. Download estimate on transfer stream (overlaps with next cycle's upload)
cudaStreamWaitEvent(transfer_stream, weights_ready);
cudaMemcpyAsync(h_estimate_pinned, d_estimate, size,
                cudaMemcpyDeviceToHost, transfer_stream);
```

### Why Multiple Streams Help

NVIDIA GPUs have separate hardware engines:
- **Copy engine(s):** H2D and D2H can overlap with each other and with compute (Jetson Orin has 1 copy engine; RTX 4070 has 2)
- **Compute engine:** Runs kernels

With single stream, these are serialized. With multiple streams, they can overlap:

```
Time →
Single stream:  [H2D][kernel][D2H][H2D][kernel][D2H]
Two streams:    [H2D][kernel][D2H]
                     [H2D][kernel][D2H]
                                       ^ overlapped
```

### Practical Limitations for This Workload

For our small data sizes (20–80 KB), the transfers are so fast (~5–10 µs) that overlap savings are modest. The main benefit is:

1. **Motion kernel overlaps with range upload:** Motion takes ~5–10 µs, range upload takes ~3–5 µs → saves **~3–5 µs** (full overlap)
2. **Estimate download overlaps with next cycle's early work:** Saves **~3–5 µs** per cycle
3. **Reduces total sync points:** Fewer places where CPU blocks waiting for GPU

### Expected Speedup

- **~5–15 µs** per cycle from overlap (modest for small transfers)
- **Prerequisite:** Pinned memory (§4) — without it, async transfers are actually sync
- More impactful at higher particle counts (N > 10000) where transfers are larger

### References

- CUDA C++ Programming Guide §3.2.6.5: "Overlap of Data Transfer and Kernel Execution"
- CUDA Best Practices Guide §9.1.3: "Asynchronous Transfers and Overlapping"
- NVIDIA blog: "How to Overlap Data Transfers in CUDA C/C++" (Harris, 2012)
- [CUDA Samples — simpleMultiCopy](https://github.com/NVIDIA/cuda-samples) — demonstrates concurrent copy + compute

---

## 7. Texture Memory / `__ldg()` for Distance Field

### Current Bottleneck

In `sensor_model_kernels.cu`, the distance field lookup is:

```cuda
dist = dist_field[my * map_w + mx];  // Global memory read
```

For a 2000×2000 map of `float`, the distance field is `2000 × 2000 × 4B = 16 MB`. This doesn't fit in L1 cache (128 KB per SM on Orin) or L2 (4 MB on Orin, 48 MB on RTX 4070).

Each particle (thread) looks up ~270 locations in the distance field. Adjacent particles have nearby poses, so their beam endpoints are **spatially close** → there is **2D spatial locality** in the access pattern.

Global memory loads go through L1 → L2 → DRAM. But global memory loads by default use the L1 cache strategy optimized for temporal locality (LRU), not spatial/2D locality.

### Solution A: `__ldg()` Intrinsic (Minimal Change)

```cuda
// Before:
dist = dist_field[my * map_w + mx];

// After — use read-only data cache path:
dist = __ldg(&dist_field[my * map_w + mx]);
```

`__ldg()` loads through the **texture/read-only cache** (separate from L1) which:
- Has its own cache hierarchy (48 KB on sm_87/89)
- Is optimized for non-uniform, scattered read patterns
- Keeps global L1 cache free for other data (particles, ranges)

Since `dist_field` is `const float* __restrict__`, the compiler **may** already generate `LDG` (load-global non-coherent) instructions. Adding explicit `__ldg()` **guarantees** it.

### Solution B: CUDA Texture Object (Best 2D Locality)

```cpp
// In sensor_model init:
cudaTextureObject_t tex_dist_field;

// Create a 2D CUDA array
cudaChannelFormatDesc desc = cudaCreateChannelDesc<float>();
cudaArray_t d_array;
cudaMallocArray(&d_array, &desc, map_w, map_h);
cudaMemcpy2DToArray(d_array, 0, 0, h_dist_field,
                    map_w * sizeof(float),
                    map_w * sizeof(float), map_h,
                    cudaMemcpyHostToDevice);

// Create texture object
cudaResourceDesc resDesc = {};
resDesc.resType = cudaResourceTypeArray;
resDesc.res.array.array = d_array;

cudaTextureDesc texDesc = {};
texDesc.addressMode[0] = cudaAddressModeBorder;
texDesc.addressMode[1] = cudaAddressModeBorder;
texDesc.filterMode = cudaFilterModePoint;  // nearest-neighbor
texDesc.readMode = cudaReadModeElementType;
texDesc.normalizedCoords = false;

cudaCreateTextureObject(&tex_dist_field, &resDesc, &texDesc, nullptr);
```

```cuda
// In kernel — replace global memory lookup:
// Before:
int mx = __float2int_rd((ex - map_ox) / map_res);
int my = __float2int_rd((ey - map_oy) / map_res);
float dist;
if (mx >= 0 && mx < map_w && my >= 0 && my < map_h) {
    dist = dist_field[my * map_w + mx];
} else {
    dist = laser_max;
}

// After — texture handles bounds checking (border mode returns 0):
float fx = (ex - map_ox) / map_res + 0.5f;
float fy = (ey - map_oy) / map_res + 0.5f;
float dist = tex2D<float>(tex_dist_field, fx, fy);
if (dist == 0.0f && (fx < 0.5f || fx >= map_w + 0.5f ||
                      fy < 0.5f || fy >= map_h + 0.5f))
    dist = laser_max;
```

### Why Texture Memory Helps

1. **2D spatial caching:** CUDA arrays store data in a space-filling curve (Morton/Z-order), so 2D-nearby elements are also nearby in memory. The texture cache exploits this with 2D-aware cache lines.

2. **Separate cache:** Texture cache doesn't compete with L1 for cache space, effectively increasing total cache capacity.

3. **Built-in bounds handling:** `cudaAddressModeBorder` returns 0 for out-of-bounds reads — eliminates the `if (mx >= 0 && ...)` branch.

4. **Free interpolation** (if desired): Setting `filterMode = cudaFilterModeLinear` gives hardware bilinear interpolation at no extra cost — could improve localization quality.

### Expected Speedup

Access pattern for 270 beams × 5000 particles = 1.35M lookups per update:

| Method | Cache hit rate (est.) | Effective bandwidth | Time (est.) |
|--------|---------------------|--------------------|----|
| Global mem (no __ldg) | ~30–50% L2 hit | ~100 GB/s | ~15–25 µs |
| `__ldg()` | ~40–60% L2 + RO cache | ~150 GB/s | ~10–18 µs |
| Texture 2D | ~50–70% tex cache | ~200 GB/s | ~8–15 µs |

For the sensor model kernel which is **compute-bound** (270× `cosf`, `sinf`, `expf`, `logf` per thread), the memory access improvement provides a **modest 10–30% speedup** on the kernel itself (~5–15 µs).

**Recommendation:** Start with `__ldg()` (1-line change), profile, then consider texture 2D if memory-bound.

### References

- CUDA C++ Programming Guide §3.2.14: "Texture and Surface Memory"
- CUDA Best Practices Guide §9.2.6: "Texture Memory"
- [NVIDIA blog: "CUDA Pro Tip: Increase Performance with Read-Only Data"](https://developer.nvidia.com/blog/cuda-pro-tip-increase-performance-with-vectorized-memory-access/)
- Volkov & Demmel, "Benchmarking GPUs to Tune Dense Linear Algebra" (2008) — texture vs global memory analysis

---

## 8. Shared Memory Tiling for Scan Ranges

### Current Bottleneck

In `kernel_sensor_weights`, each thread (particle) loops over all beams:

```cuda
for (int b = 0; b < num_ranges; b += step) {
    float r = ranges[b];  // Global memory read — same address for ALL threads!
    ...
}
```

With 5000 particles in a thread block of 256 threads, **all 256 threads read the same `ranges[b]` simultaneously**. This is a **broadcast pattern** that the L1 cache handles reasonably well (one cache line serves all threads in a warp), but:

1. The 270 beam ranges (270 × 4B = 1080 B) easily fit in shared memory (48–100 KB available)
2. L1 cache lookup still has ~30 cycle latency per access; shared memory has ~5 cycles
3. The ranges data competes with distance field data for L1 cache space

### Solution: Load Ranges into Shared Memory

```cuda
__global__
void kernel_sensor_weights(const float* __restrict__ particles, int n,
                           const float* __restrict__ ranges, int num_ranges,
                           int max_beams,
                           /* ... other params ... */
                           float* __restrict__ out_weights) {
    // Load ranges into shared memory cooperatively
    extern __shared__ float s_ranges[];
    for (int j = threadIdx.x; j < num_ranges; j += blockDim.x) {
        s_ranges[j] = ranges[j];
    }
    __syncthreads();

    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;

    // ... same as before, but use s_ranges[b] instead of ranges[b] ...
    for (int b = 0; b < num_ranges; b += step) {
        float r = s_ranges[b];  // Shared memory — ~6× lower latency
        // ...
    }
}
```

Launch configuration:

```cpp
int block = 256;
int grid = (n + block - 1) / block;
size_t smem_bytes = num_ranges * sizeof(float);  // 270 * 4 = 1080 bytes
kernel_sensor_weights<<<grid, block, smem_bytes, stream>>>(...);
```

### Why It Helps

| Aspect | Global/L1 | Shared Memory |
|--------|----------|---------------|
| Latency | ~30 cycles (L1 hit), ~200+ (miss) | ~5 cycles |
| Bandwidth | 128 B/cycle/SM | 128 B/cycle/SM (bank-conflict free) |
| Cache competition | Competes with dist_field loads | Separate from L1 |
| Guaranteed hit | No (may be evicted) | Yes (explicitly managed) |

For 270 beam iterations per thread, saving ~25 cycles × 270 = ~6750 cycles ≈ **~5–7 µs** per SM at 1 GHz.

### Bank Conflict Analysis

Shared memory on Ampere/Ada has 32 banks, each 4 bytes wide. When all threads in a warp read the same address (broadcast), it's a **single broadcast read** — no bank conflict. So the access pattern `s_ranges[b]` (same index for all threads) is optimal.

### Precompute Beam Angles in Shared Memory

Further optimization: precompute beam angles (which are the same for all particles):

```cuda
extern __shared__ float smem[];
float* s_ranges = smem;
float* s_angles = &smem[num_ranges];

// Cooperative load
for (int j = threadIdx.x; j < num_ranges; j += blockDim.x) {
    s_ranges[j] = ranges[j];
    s_angles[j] = angle_min + j * angle_inc;  // precompute
}
__syncthreads();

// In loop:
float beam_angle = s_angles[b] + ptheta;  // save a multiply per beam
```

Shared memory usage: `270 × 4 × 2 = 2160 bytes` — well within limits.

### Expected Speedup

- Sensor model kernel: **~5–15% faster** (compute-bound kernel with memory savings freeing L1 for distance field)
- Absolute: **~3–10 µs** savings on the sensor kernel

### References

- CUDA C++ Programming Guide §3.2.4: "Shared Memory"
- NVIDIA Best Practices Guide §9.2.3: "Shared Memory"
- Volkov, "Better Performance at Lower Occupancy" (GTC 2010) — shared memory tiling patterns

---

## 9. Warp-Level Primitives for Reductions

### Background

Warp-level primitives (`__shfl_down_sync`, `__shfl_up_sync`, etc.) allow threads within a 32-thread warp to exchange register values **without shared memory or synchronization barriers**. Latency: ~1 cycle (same as a register read).

### Application: Custom Warp-Level Reduction for N_eff

If not using CUB (which already uses warp shuffles internally), you can build efficient small reductions:

```cuda
// Warp-level sum reduction
__device__ float warp_reduce_sum(float val) {
    for (int offset = 16; offset > 0; offset >>= 1) {
        val += __shfl_down_sync(0xFFFFFFFF, val, offset);
    }
    return val;  // Result valid in lane 0
}

// Block-level reduction using warp primitives
__device__ float block_reduce_sum(float val) {
    __shared__ float warp_sums[32];  // max 32 warps per block

    int lane = threadIdx.x % 32;
    int warp = threadIdx.x / 32;

    val = warp_reduce_sum(val);

    if (lane == 0) warp_sums[warp] = val;
    __syncthreads();

    // First warp reduces the warp sums
    val = (threadIdx.x < blockDim.x / 32) ? warp_sums[lane] : 0.0f;
    if (warp == 0) val = warp_reduce_sum(val);

    return val;  // Result in thread 0
}
```

### Application: Warp-Level Scan for Small Arrays

For N ≤ 1024, an entire prefix scan can use warp shuffles:

```cuda
__device__ float warp_inclusive_scan(float val) {
    for (int offset = 1; offset < 32; offset <<= 1) {
        float n = __shfl_up_sync(0xFFFFFFFF, val, offset);
        if (threadIdx.x % 32 >= offset) val += n;
    }
    return val;
}
```

### Practical Relevance

Given that we recommend CUB (§1, §2) which already uses warp primitives internally, explicit warp-level coding has limited incremental value. It's useful for:

1. **Custom fused kernels** (§3) where CUB can't be used (e.g., inside a kernel that does other work)
2. **The fused normalize-and-scan kernel** from §3 — warp scan for the scan phase
3. **Education/understanding** of what CUB does under the hood

### Expected Speedup

- **Standalone (replacing single-thread kernels):** ~10–15× for N=5000 (same as CUB, §2)
- **Incremental over CUB:** ~0–5% (CUB is already optimal)
- **In fused kernels:** Enables fusion that wouldn't be possible otherwise

### References

- CUDA C++ Programming Guide §B.15: "Warp Shuffle Functions"
- [NVIDIA blog: "Using CUDA Warp-Level Primitives" (2018)](https://developer.nvidia.com/blog/using-cuda-warp-level-primitives/)
- Rhu & Erez, "The Dual-Path Execution Model for Efficient GPU Control Flow" (2013) — warp-level efficiency
- CUB source code: `cub/warp/warp_reduce.cuh` and `cub/warp/warp_scan.cuh`

---

## Summary: Implementation Priority

| Priority | Optimization | Est. Savings | Complexity | Dependencies |
|----------|-------------|-------------|------------|-------------|
| **P0** | §5 Persistent buffer reuse | 130–510 µs | Low | None |
| **P0** | §1 GPU-side normalization (CUB) | 50–70 µs | Medium | None |
| **P1** | §2 CUB DeviceScan + Reduce | 23 µs | Low | CUB header |
| **P1** | §4 Pinned memory | 10–30 µs | Low | None |
| **P2** | §3 Kernel fusion (double-buffer) | 15–40 µs | Medium | §5 |
| **P2** | §7 `__ldg()` / Texture | 5–15 µs | Low/Medium | None |
| **P2** | §8 Shared mem for ranges | 3–10 µs | Low | None |
| **P3** | §6 Multi-stream overlap | 5–15 µs | Medium | §4 |
| **P3** | §9 Warp primitives | 0–5 µs | Medium | §3 |

### Estimated Total Improvement

**Before optimization (estimated full cycle):** ~400–900 µs  
(dominated by per-frame cudaMalloc + CPU normalization + sequential scan)

**After all optimizations (estimated):** ~50–150 µs  
(**~4–8× overall speedup**)

### Implementation Order

1. **First:** §5 (persistent buffers) — biggest bang, easiest fix
2. **Second:** §1 + §2 (CUB reductions + scan) — eliminate CPU transfers
3. **Third:** §4 (pinned memory) — enables async, small code change
4. **Fourth:** §7 + §8 (`__ldg` + shared mem) — one-liner improvements
5. **Fifth:** §3 + §6 (fusion + streams) — polish for maximum throughput
6. **Optional:** §9 (warp primitives) — only if building custom fused kernels

---

## Appendix A: Build Configuration for CUB

CUB is header-only and bundled with CUDA Toolkit 11+ (via CCCL). No extra installation needed:

```cmake
# In CMakeLists.txt — CUB comes with CUDA, just make sure CUDA is found
find_package(CUDAToolkit REQUIRED)

# CUB headers are in CUDA include path automatically
# Just #include <cub/cub.cuh> in your .cu files
```

If using an older CUDA, install CCCL separately:
```bash
git clone https://github.com/NVIDIA/cccl.git
# Add cccl/cub to your include path
```

## Appendix B: Profiling Commands

```bash
# Profile kernel execution time
nsys profile --stats=true ./gpu_amcl_node

# Detailed kernel metrics (occupancy, memory throughput)
ncu --set full -k kernel_sensor_weights ./gpu_amcl_node

# Memory transfer analysis
nsys profile --trace=cuda,osrt ./gpu_amcl_node

# On Jetson (tegra), use:
nsys profile --trace=cuda,nvtx --output=report ./gpu_amcl_node
```

## Appendix C: Jetson-Specific Notes

1. **Unified memory architecture:** Jetson CPU and GPU share physical DRAM. `cudaMemcpy` on Jetson doesn't cross PCIe — it's a memcpy within the same memory. However, cache coherency and page migration still add overhead. Pinned memory (§4) prevents page migration.

2. **Lower clock speeds:** Jetson Orin GPU runs at ~625–1300 MHz vs RTX 4070 at ~1500–2475 MHz. This means:
   - Kernel execution takes ~2× longer on Jetson
   - Launch overhead is proportionally larger (fixed µs cost / longer kernel = higher fraction)
   - Buffer reuse (§5) is even more important since `cudaMalloc` is slower

3. **Fewer SMs:** Orin has 16 SMs vs RTX 4070's 46. For 5000 particles at 256 threads/block = 20 blocks, Orin needs 2 waves while RTX 4070 runs in <1 wave. CUB adapts automatically.

4. **Power budget:** Jetson runs at 15–60W. GPU optimizations that reduce total work (fewer kernels, fewer transfers) directly translate to power savings — important for a battery-powered F1Tenth car.

5. **Max clock for benchmarking:**
   ```bash
   sudo jetson_clocks  # Lock CPU/GPU/EMC to max frequency
   ```

---

## 10. Pipeline Latency Reduction (Scan → EKF Pose)

### The Problem: 13ms End-to-End Latency

Benchmark data (RTX 4070 laptop, 1500 particles, Spielberg map) shows:

| Metric | Value |
|--------|-------|
| **AMCL PF computation** | 0.39 ms mean |
| **Scan → EKF latency** | 12.9 ms mean |
| **Position error** | 0.35 m mean RMS |

The GPU particle filter completes in **0.4ms** but the pose update doesn't reach the EKF for **13ms** — a **32× overhead** from the software pipeline alone. At 8 m/s racing speed, 13ms of latency translates to **10.4 cm of position staleness** even without any estimation error.

### Root Cause Analysis

The current scan→EKF pipeline has three sequential timer-based stages:

```
Scan arrives ──┐
               │ scan_callback: PF predict+update+estimate ......... 0.4 ms
               │ Result stored in cached_estimate_
               │
               ├── WAIT for publish_timer_ (40 Hz = 25ms period) ... 0-25 ms (avg 12.5 ms)  ← DOMINANT
               │
               │ publish_timer_callback: build msg + publish
               │
               ├── DDS transport ............................ ~0.1 ms (intraprocess)
               │
               │ EKF amcl_callback: correct() .............. ~0.01 ms
               │
               ├── WAIT for EKF publish_timer_ (200 Hz = 5ms) ... 0-5 ms (avg 2.5 ms)
               │
               │ EKF publish_timer_callback: publish + TF broadcast
               │
               ▼ /ekf_pose arrives
```

**Total expected latency: 0.4 + 12.5 + 0.1 + 0.01 + 2.5 = ~15.5 ms (avg)**

Measured 12.9ms is consistent because the benchmark only measures `scan_arrival → ekf_arrival`, and the EKF correction happens immediately in `amcl_callback` (which updates internal state that the next EKF timer tick publishes).

### Latency Budget Breakdown

| Stage | Current | Target | Fix |
|-------|---------|--------|-----|
| PF computation | 0.4 ms | 0.4 ms (already fast) | §1-9 for further GPU opts |
| **AMCL publish timer wait** | **12.5 ms avg** | **0 ms** | **Direct publish from scan_callback** |
| DDS transport | 0.1 ms | ~0 ms | Intraprocess pub/sub |
| **EKF publish timer wait** | **2.5 ms avg** | **0 ms** | **Event-driven EKF publish** |
| EKF correction + publish | 0.02 ms | 0.02 ms | Already fast |
| **Total** | **~15.5 ms** | **~0.5 ms** | **31× reduction** |

---

### 10.1 Direct Publish from AMCL scan_callback (CRITICAL — eliminates 12.5ms)

**Current code** (`amcl_node.cpp`):
```cpp
// scan_callback stores result in cached_estimate_
auto est = pf_.get_estimate();
{
    std::lock_guard<std::mutex> lk(estimate_mutex_);
    cached_estimate_ = est;
}
// ... then a separate 40 Hz timer reads cached_estimate_ and publishes
```

**Problem:** The PF finishes in 0.4ms, but the result sits in `cached_estimate_` for an average of 12.5ms before the timer publishes it. This is the single largest latency source.

**Fix:** Publish the AMCL pose **directly** at the end of `scan_callback`, right after getting the estimate. Keep the timer only for particle cloud visualisation (which is for RViz, not latency-critical).

**Changed code:**
```cpp
void AmclNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // ... movement gating, stale guard, PF predict+update ...

    auto est = pf_.get_estimate();

    // ── Publish IMMEDIATELY (no timer delay) ──
    publish_pose(est, msg->header.stamp);   // Use scan's timestamp, not now()

    // Cache for particle cloud timer (visualisation only)
    {
        std::lock_guard<std::mutex> lk(estimate_mutex_);
        cached_estimate_ = est;
    }
    processing_scan_ = false;
}

void AmclNode::publish_pose(const PoseEstimate& est,
                            const rclcpp::Time& stamp) {
    auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    pose_msg.header.stamp    = stamp;  // scan timestamp for proper EKF time alignment
    pose_msg.header.frame_id = global_frame_;
    // ... fill in position, orientation, covariance ...
    pose_pub_->publish(pose_msg);
}
```

**Why use the scan's timestamp, not `now()`:** The EKF can correctly associate the AMCL correction with the time the measurement was actually taken, not when it was published. This makes the EKF's dead-reckoning between corrections more accurate.

**Expected improvement:** Eliminates **12.5ms average** latency (the entire timer wait).

**Risk:** None. The timer-based publish was only there as a design pattern; there's no rate-limiting benefit since scans already arrive at ~40Hz.

**Files to change:**
- `amcl_node.cpp`: Move pose building/publishing into `scan_callback` (or a helper called from it). Repurpose `publish_timer_callback` for particle cloud only.
- `amcl_node.hpp`: Add `publish_pose()` helper method declaration.

---

### 10.2 Event-Driven EKF Publish (eliminates 2.5ms)

**Current code** (`ekf_node.cpp`):
```cpp
// amcl_callback: just does correct(), doesn't publish
void EkfNode::amcl_callback(...) {
    correct(z, R);
}
// A separate 200 Hz timer does the actual publishing
void EkfNode::publish_timer_callback() {
    pose_pub_->publish(msg);
    broadcast_tf(msg.header.stamp);
}
```

**Problem:** After the EKF `correct()` in `amcl_callback`, the corrected state waits up to 5ms for the publish timer.

**Fix — Option A (simple):** Publish directly from `amcl_callback` after `correct()`:
```cpp
void EkfNode::amcl_callback(...) {
    correct(z, R);
    publish_and_broadcast();  // Immediate publish after AMCL correction
}
```
Keep the 200Hz timer for odom-driven predictions (these still need regular publishing for smooth TF).

**Fix — Option B (event-driven everything):** Replace the EKF publish timer entirely. Publish from both `odom_callback` and `amcl_callback`:
```cpp
void EkfNode::odom_callback(...) {
    predict(delta, Q);
    publish_and_broadcast();  // Publish every odom update
}
void EkfNode::amcl_callback(...) {
    correct(z, R);
    publish_and_broadcast();  // Publish every AMCL correction
}
```
This eliminates the EKF timer completely. Publish rate equals odom rate (which is already high enough) plus extra publishes on each AMCL correction.

**Expected improvement:** Eliminates **2.5ms average** latency.

**Recommended:** Option B. It's cleaner and ensures zero latency from both correction and prediction events. The publish timer becomes unnecessary.

**Files to change:**
- `ekf_node.cpp`: Move publish logic from `publish_timer_callback` into a `publish_and_broadcast()` helper. Call it from both `odom_callback` and `amcl_callback`. Remove the timer.
- `ekf_node.hpp`: Add `publish_and_broadcast()` method. Remove `publish_timer_` member.

---

### 10.3 ROS 2 Intraprocess Communication (eliminates DDS serialization)

**Current:** All three nodes (AMCL, Odom, EKF) run as separate processes. Messages are serialized through the DDS middleware even though they're on the same machine.

**Fix:** Use ROS 2 **intraprocess communication** by composing all nodes into a single process with a shared executor:

```cpp
// main_localization.cpp — single process for all three nodes
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    auto amcl_node = std::make_shared<gpu_amcl_cpp::AmclNode>(options);
    auto odom_node = std::make_shared<gpu_amcl_cpp::OdomNode>(options);
    auto ekf_node  = std::make_shared<gpu_amcl_cpp::EkfNode>(options);

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 4);
    executor.add_node(amcl_node);
    executor.add_node(odom_node);
    executor.add_node(ekf_node);
    executor.spin();
    rclcpp::shutdown();
}
```

**Benefit:**
- Eliminates DDS serialization/deserialization (~50–100 µs per message)
- Messages are passed as shared pointers (zero-copy)
- Reduced scheduling jitter — executor dispatches callbacks directly

**Requirement:** Publishers must use `UniquePtr` messages:`
```cpp
auto msg = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
// fill msg...
pose_pub_->publish(std::move(msg));
```

**Expected improvement:** ~0.1–0.3ms and reduced jitter.

**Files to change:**
- New: `main_localization.cpp` (combined launcher)
- `CMakeLists.txt`: Add new executable
- All nodes: Change publishers to use `UniquePtr` for intraprocess zero-copy
- `cpp_localization.launch.py`: Add option to launch single-process or multi-process

---

### 10.4 Callback-Group Threading for AMCL (reduces scheduling delay)

**Current:** AMCL uses `MultiThreadedExecutor` with 4 threads, but all callbacks are in the default `MutuallyExclusiveCallbackGroup`. This means the scan callback, odom callback, and publish timer cannot run concurrently.

**Fix:** Use `ReentrantCallbackGroup` or separate callback groups:
```cpp
// In AmclNode constructor:
scan_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
timer_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

rclcpp::SubscriptionOptions scan_opts;
scan_opts.callback_group = scan_cb_group_;
scan_sub_ = create_subscription<LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(), 
    std::bind(&AmclNode::scan_callback, this, _1), scan_opts);
```

**Benefit:** The scan callback executes immediately when a scan arrives, without waiting for the odom callback or timer to finish. With the `processing_scan_` atomic guard, this is already safe.

**Expected improvement:** Removes ~0.1–1ms scheduling jitter depending on executor load.

**Files to change:**
- `amcl_node.cpp`: Create separate callback groups for scan, odom, and timer
- `amcl_node.hpp`: Add callback group members

---

### 10.5 Reduced Scan Decimation for Lower Latency (trade-off)

**Current:** The sensor model processes all 270 beams per update. Each beam involves a distance field lookup on GPU.

**Option:** Process every Nth beam (e.g., N=2 → 135 beams). This halves the sensor model kernel time.

```yaml
# gpu_amcl_cpp_params.yaml
laser_max_beams: 135   # Use 135 of 270 beams (every other beam)
```

**Trade-off:** Slight accuracy reduction for faster cycle time. At 270 beams over 270°, beam spacing is 1°. At 135 beams, spacing is 2° — still adequate for indoor racing.

**Expected improvement:** ~30–50% sensor model speedup (already fast at 0.4ms, but helps on Jetson).

---

### 10.6 Scan Header Timestamp Propagation (correctness improvement)

**Current:** AMCL publishes with `now()` timestamp. EKF has no way to know when the scan was actually captured.

**Fix:** Propagate the original scan timestamp through the AMCL→EKF pipeline:
```cpp
// In scan_callback after PF:
pose_msg.header.stamp = msg->header.stamp;  // Use scan's original timestamp
```

**Why this matters:** The EKF's dead-reckoning between AMCL corrections relies on knowing the exact time of each correction. If AMCL publishes with `now()` (which includes 0.4ms of processing), the EKF attributes the correction to a time 0.4ms later than reality. At 8 m/s, this is 3.2mm — small but systematic.

More importantly, when combined with direct publish (§10.1), the EKF receives corrections with the correct timestamps, enabling it to handle out-of-order measurements or retroactive corrections if needed.

**Files to change:**
- `amcl_node.cpp`: Change `pose_msg.header.stamp = now()` to `msg->header.stamp`

---

### 10.7 Summary: Full Low-Latency Pipeline

After implementing §10.1–10.6, the pipeline becomes:

```
Scan arrives (timestamp T_scan) ──┐
   │ scan_callback: PF predict+update+estimate ......... 0.4 ms
   │ publish_pose() with T_scan timestamp .............. immediate
   │
   ├── intraprocess: zero-copy UniquePtr ............... ~0 ms
   │
   │ EKF amcl_callback: correct() + publish_and_broadcast() ... 0.02 ms
   │
   ▼ /ekf_pose arrives with T_scan reference time
```

**Total: ~0.42 ms** (down from ~15.5 ms = **37× improvement**)

At 8 m/s:
- **Before:** 15.5 ms × 8 m/s = **12.4 cm** position staleness
- **After:** 0.42 ms × 8 m/s = **0.34 cm** position staleness

### Implementation Priority

| Step | Change | Effort | Impact | Dependencies |
|------|--------|--------|--------|-------------|
| **10.1** | Direct publish from scan_callback | 30 min | **-12.5 ms** | None |
| **10.2** | Event-driven EKF publish | 30 min | **-2.5 ms** | None |
| **10.6** | Scan timestamp propagation | 5 min | Correctness | Best with 10.1 |
| **10.4** | Callback group threading | 20 min | **-0.1–1 ms jitter** | None |
| **10.3** | Intraprocess comms (single process) | 1 hr | **-0.1–0.3 ms** | Requires refactor |
| **10.5** | Beam decimation | 10 min | GPU speedup | None |

**Recommended order:** 10.1 → 10.6 → 10.2 → 10.4 → 10.3 → 10.5

The first two steps (10.1 + 10.6) take 35 minutes and eliminate 90% of the latency.
