#pragma once

#include <cuda_runtime.h>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <limits>
#include <memory>

namespace gpu_amcl_cpp {

/**
 * @brief Execute a CUDA runtime call and throw on failure.
 *
 * Input:
 *   - call: CUDA runtime expression returning cudaError_t.
 * Output:
 *   - None on success.
 *   - Throws std::runtime_error on failure and logs file/line + CUDA error text.
 */
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
    /**
     * @brief Construct an empty device buffer without allocating GPU memory for initialization.
     *
     * Input:
     *   - None.
     * Output:
     *   - Buffer starts with ptr_ == nullptr and count_ == 0.
     */
    DeviceBuffer() = default;

    /**
     * @brief Construct a device buffer and allocate storage for count elements.
     *
     * Input:
     *   - count: Number of elements to allocate on device.
     * Output:
     *   - Allocated device buffer with capacity for count elements.
     *   - Throws std::runtime_error if cudaMalloc fails.
     */
    explicit DeviceBuffer(size_t count) { 
        allocate(count); 
    }
    /**
     * @brief Destroy the buffer and release owned device memory.
     *
     * Input:
     *   - None.
     * Output:
     *   - Device allocation is freed if present.
     */
    ~DeviceBuffer() { free(); }

    /**
     * @brief Move-construct a buffer by transferring ownership from another buffer.
     *
     * Input:
     *   - o: Source buffer whose device pointer ownership is transferred.
     * Output:
     *   - This buffer owns o's previous device allocation and count.
     *   - Source buffer is reset to empty state.
     */
    DeviceBuffer(DeviceBuffer&& o) noexcept
        : ptr_(o.ptr_), count_(o.count_) {
        o.ptr_   = nullptr;
        o.count_ = 0;
    }
    /**
     * @brief Move-assign a buffer by releasing current memory and taking ownership from source.
     *
     * Input:
     *   - o: Source buffer whose allocation is moved into this object.
     * Output:
     *   - Previous allocation owned by this object is released.
     *   - This object owns source allocation and count.
     *   - Source buffer is reset to empty state.
     */
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

    /**
     * @brief Allocate or re-allocate device memory for count elements.
     *
     * Input:
     *   - count: Number of elements to reserve on device.
     * Output:
     *   - Existing allocation is released first.
     *   - Buffer owns a new allocation sized for count elements.
     *   - Throws std::runtime_error if cudaMalloc fails.
     */
    void allocate(size_t count) {
        free();
        if (count > (std::numeric_limits<size_t>::max() / sizeof(T))) {
            throw std::overflow_error("DeviceBuffer allocation size overflow");
        }

        T* new_ptr = nullptr;
        CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&new_ptr), count * sizeof(T)));

        ptr_ = new_ptr;
        count_ = count;
    }

    /**
     * @brief Release the currently owned device allocation, if any.
     *
     * Input:
     *   - None.
     * Output:
     *   - If allocated, device memory is freed.
     *   - Buffer state is reset to ptr_ == nullptr and count_ == 0.
     */
    void free() {
        if (ptr_) {
            cudaFree(ptr_);
            ptr_   = nullptr;
            count_ = 0;
        }
    }

    /**
     * @brief Copy host data to device memory, growing the buffer if needed.
     *
     * Input:
     *   - host_data: Pointer to source data in host memory.
     *   - count: Number of elements to copy.
     * Output:
     *   - Device buffer contains count copied elements from host_data.
     *   - Buffer may be reallocated if count exceeds current capacity.
     *   - Throws std::runtime_error on CUDA API failure.
     */
    void upload(const T* host_data, size_t count) {
        if (count > count_) allocate(count);
        CUDA_CHECK(cudaMemcpy(ptr_, host_data,
                              count * sizeof(T),
                              cudaMemcpyHostToDevice));
    }

    /**
     * @brief Copy device data to host memory.
     *
     * Input:
     *   - host_data: Pointer to destination data in host memory.
     *   - count: Number of elements to copy.
     * Output:
     *   - host_data receives count elements from device buffer.
     *   - Throws std::runtime_error on CUDA API failure.
     */
    void download(T* host_data, size_t count) const {
        CUDA_CHECK(cudaMemcpy(host_data, ptr_,
                              count * sizeof(T),
                              cudaMemcpyDeviceToHost));
    }

    /**
     * @brief Asynchronously copy host data to device on a provided CUDA stream.
     *
     * Input:
     *   - host_data: Pointer to source data in host memory.
     *   - count: Number of elements to copy.
     *   - stream: CUDA stream used for asynchronous transfer.
     * Output:
     *   - Transfer is queued on stream.
     *   - Buffer may be reallocated if count exceeds current capacity.
     *   - Throws std::runtime_error on CUDA API failure.
     */
    void upload_async(const T* host_data, size_t count,
                      cudaStream_t stream) {
        if (count > count_) allocate(count);
        CUDA_CHECK(cudaMemcpyAsync(ptr_, host_data,
                                   count * sizeof(T),
                                   cudaMemcpyHostToDevice, stream));
    }

    /**
     * @brief Asynchronously copy device data to host on a provided CUDA stream.
     *
     * Input:
     *   - host_data: Pointer to destination data in host memory.
     *   - count: Number of elements to copy.
     *   - stream: CUDA stream used for asynchronous transfer.
     * Output:
     *   - Transfer is queued on stream.
     *   - host_data is valid only after stream completion.
     *   - Throws std::runtime_error on CUDA API failure.
     */
    void download_async(T* host_data, size_t count,
                        cudaStream_t stream) const {
        CUDA_CHECK(cudaMemcpyAsync(host_data, ptr_,
                                   count * sizeof(T),
                                   cudaMemcpyDeviceToHost, stream));
    }

    /**
     * @brief Set the entire allocated device buffer to zero bytes.
     *
     * Input:
     *   - None.
     * Output:
     *   - If allocated, all bytes in the buffer are set to zero.
     *   - No action is taken when buffer is empty.
     */
    void zero() {
        if (ptr_)
            CUDA_CHECK(cudaMemset(ptr_, 0, count_ * sizeof(T)));
    }

    /**
     * @brief Return the raw device pointer managed by this buffer.
     *
     * Input:
     *   - None.
     * Output:
     *   - Device pointer to the allocation, or nullptr if empty.
     */
    T*     ptr()   const { return ptr_; }
    /**
     * @brief Return the current element capacity tracked by this buffer.
     *
     * Input:
     *   - None.
     * Output:
     *   - Number of elements currently allocated.
     */
    size_t count() const { return count_; }
    /**
     * @brief Return the current allocation size in bytes.
     *
     * Input:
     *   - None.
     * Output:
     *   - Number of allocated bytes, computed as count() * sizeof(T).
     */
    size_t bytes() const { return count_ * sizeof(T); }

private:
    T*     ptr_   = nullptr;
    size_t count_ = 0;
};

// ─── CUDA stream RAII ───────────────────────────────────────────────
class CudaStream {
public:
    /**
     * @brief Create and own a CUDA stream.
     *
     * Input:
     *   - None.
     * Output:
     *   - Valid CUDA stream handle stored in this object.
     *   - Throws std::runtime_error if stream creation fails.
     */
    CudaStream(){ 
        int device_count = 0;
        CUDA_CHECK(cudaGetDeviceCount(&device_count));
        if (device_count <= 0) {
            throw std::runtime_error("No CUDA devices available");
        }
        CUDA_CHECK(cudaSetDevice(0));
        CUDA_CHECK(cudaFree(0));
        cudaError_t stream_status = cudaStreamCreate(&s_);
        if (stream_status == cudaErrorNotSupported) {
            s_ = 0;
            std::fprintf(stderr,
                         "[gpu_amcl_cpp] CUDA stream creation not supported; using default stream\n");
            return;
        }
        CUDA_CHECK(stream_status);
    }
    /**
     * @brief Destroy the owned CUDA stream if it exists.
     *
     * Input:
     *   - None.
     * Output:
     *   - Owned stream handle is destroyed and resources are released.
     */
    ~CudaStream(){ 
        if (s_) cudaStreamDestroy(s_); 
    }

    /**
     * @brief Move-construct a stream wrapper by transferring stream ownership.
     *
     * Input:
     *   - o: Source stream wrapper whose handle ownership is transferred.
     * Output:
     *   - This object owns source stream handle.
     *   - Source wrapper is reset to empty state.
     */
    CudaStream(CudaStream&& o) noexcept : s_(o.s_){
        o.s_ = nullptr; 
    }
    /**
     * @brief Move-assign stream ownership into an existing wrapper.
     *
     * Input:
     *   - o: Source stream wrapper whose handle is moved.
     * Output:
     *   - Existing owned stream (if any) is destroyed.
     *   - This wrapper owns source stream handle.
     *   - Source wrapper is reset to empty state.
     */
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

    /**
     * @brief Return the owned CUDA stream handle.
     *
     * Input:
     *   - None.
     * Output:
     *   - Raw cudaStream_t handle, or nullptr if empty.
     */
    cudaStream_t get() const{ 
        return s_; 
    }

    /**
     * @brief Block until all previously queued operations in this stream complete.
     *
     * Input:
     *   - None.
     * Output:
     *   - Returns after stream work has finished.
     *   - Throws std::runtime_error on CUDA API failure.
     */
    void sync() const{
        CUDA_CHECK(cudaStreamSynchronize(s_)); 
    }

private:
    cudaStream_t s_ = nullptr;
};

// ─── Helpers ────────────────────────────────────────────────────────

/**
 * @brief Compute 1-D CUDA grid dimension for n threads and block_size threads per block.
 *
 * Input:
 *   - n: Total number of threads/elements to cover.
 *   - block_size: Threads per CUDA block.
 * Output:
 *   - Number of blocks required to cover n elements.
 */
inline int grid_size(int n, int block_size = 256) {
    return (n + block_size - 1) / block_size;
}

}  // namespace gpu_amcl_cpp
