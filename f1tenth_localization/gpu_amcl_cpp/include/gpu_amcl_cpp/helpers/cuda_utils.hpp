#pragma once

#include <cuda_runtime.h>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <memory>

namespace gpu_amcl_cpp {

// ─── Error-checking macro ───────────────────────────────────────────
#define CUDA_CHECK(call)                                                    \
    do {                                                                    \
        cudaError_t err = (call);                                           \
        if (err != cudaSuccess) {                                           \
            fprintf(stderr, "CUDA error at %s:%d — %s\n",                  \
                    __FILE__, __LINE__, cudaGetErrorString(err));            \
            throw std::runtime_error(cudaGetErrorString(err));              \
        }                                                                   \
    } while (0)

// ─── RAII wrapper for device memory ─────────────────────────────────
/**
 * @brief Typed RAII container for a CUDA device allocation.
 *
 * Use `upload()` / `download()` to transfer host ↔ device.
 * The pointer is freed automatically on destruction.
 */
template <typename T>
class DeviceBuffer {
public:
    DeviceBuffer() = default;

    explicit DeviceBuffer(size_t count) { allocate(count); }

    ~DeviceBuffer() { free(); }

    // Move-only.
    DeviceBuffer(DeviceBuffer&& o) noexcept
        : ptr_(o.ptr_), count_(o.count_) {
        o.ptr_   = nullptr;
        o.count_ = 0;
    }
    DeviceBuffer& operator=(DeviceBuffer&& o) noexcept {
        if (this != &o) {
            free();
            ptr_     = o.ptr_;
            count_   = o.count_;
            o.ptr_   = nullptr;
            o.count_ = 0;
        }
        return *this;
    }
    DeviceBuffer(const DeviceBuffer&)            = delete;
    DeviceBuffer& operator=(const DeviceBuffer&) = delete;

    /// (Re-)allocate device memory for `count` elements.
    void allocate(size_t count) {
        free();
        count_ = count;
        CUDA_CHECK(cudaMalloc(&ptr_, count * sizeof(T)));
    }

    /// Free device memory.
    void free() {
        if (ptr_) {
            cudaFree(ptr_);
            ptr_   = nullptr;
            count_ = 0;
        }
    }

    /// Upload host data to device.
    void upload(const T* host_data, size_t count) {
        if (count > count_) allocate(count);
        CUDA_CHECK(cudaMemcpy(ptr_, host_data,
                              count * sizeof(T),
                              cudaMemcpyHostToDevice));
    }

    /// Download device data to host.
    void download(T* host_data, size_t count) const {
        CUDA_CHECK(cudaMemcpy(host_data, ptr_,
                              count * sizeof(T),
                              cudaMemcpyDeviceToHost));
    }

    /// Upload host data (async, on given stream).
    void upload_async(const T* host_data, size_t count,
                      cudaStream_t stream) {
        if (count > count_) allocate(count);
        CUDA_CHECK(cudaMemcpyAsync(ptr_, host_data,
                                   count * sizeof(T),
                                   cudaMemcpyHostToDevice, stream));
    }

    /// Download device data (async, on given stream).
    void download_async(T* host_data, size_t count,
                        cudaStream_t stream) const {
        CUDA_CHECK(cudaMemcpyAsync(host_data, ptr_,
                                   count * sizeof(T),
                                   cudaMemcpyDeviceToHost, stream));
    }

    /// Set all bytes to zero.
    void zero() {
        if (ptr_)
            CUDA_CHECK(cudaMemset(ptr_, 0, count_ * sizeof(T)));
    }

    T*     ptr()   const { return ptr_; }
    size_t count() const { return count_; }
    size_t bytes() const { return count_ * sizeof(T); }

private:
    T*     ptr_   = nullptr;
    size_t count_ = 0;
};

// ─── CUDA stream RAII ───────────────────────────────────────────────
class CudaStream {
public:
    CudaStream()  { CUDA_CHECK(cudaStreamCreate(&s_)); }
    ~CudaStream() { if (s_) cudaStreamDestroy(s_); }

    CudaStream(CudaStream&& o) noexcept : s_(o.s_) { o.s_ = nullptr; }
    CudaStream& operator=(CudaStream&& o) noexcept {
        if (this != &o) {
            if (s_) cudaStreamDestroy(s_);
            s_   = o.s_;
            o.s_ = nullptr;
        }
        return *this;
    }
    CudaStream(const CudaStream&)            = delete;
    CudaStream& operator=(const CudaStream&) = delete;

    cudaStream_t get() const { return s_; }

    void sync() const { CUDA_CHECK(cudaStreamSynchronize(s_)); }

private:
    cudaStream_t s_ = nullptr;
};

// ─── Helpers ────────────────────────────────────────────────────────
/// Choose a reasonable 1-D grid size for N threads.
inline int grid_size(int n, int block_size = 256) {
    return (n + block_size - 1) / block_size;
}

}  // namespace gpu_amcl_cpp
