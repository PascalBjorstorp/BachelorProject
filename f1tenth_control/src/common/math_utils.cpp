#include "f1tenth_control/common/math_utils.hpp"
#include <algorithm>

namespace f1tenth_control {
namespace math {

// Inputs:
// - data: Input scalar sequence.
// - window_size: Median filter window size.
// Purpose:
// - Suppress impulsive noise using sliding-window median filtering.
// Outputs:
// - Returns filtered sequence with same length as input.
std::vector<double> medianFilter(const std::vector<double>& data, size_t window_size) {
    if (data.empty()) return {};
    if (window_size <= 1) return data;
    
    // Ensure odd window size for symmetric filtering
    window_size |= 1;  // Faster than if/else for making odd
    
    const size_t half_window = window_size / 2;
    const size_t n = data.size();
    std::vector<double> result(n);
    std::vector<double> window(window_size);  // Pre-allocate once outside loop
    
    for (size_t i = 0; i < n; ++i) {
        // Calculate window bounds with boundary handling
        const size_t start = (i >= half_window) ? i - half_window : 0;
        const size_t end = std::min(i + half_window + 1, n);
        const size_t wlen = end - start;
        
        // Copy window data (reuse pre-allocated buffer)
        std::copy(data.begin() + start, data.begin() + end, window.begin());
        
        // Find median using partial sort - O(n) average
        const size_t mid = wlen / 2;
        std::nth_element(window.begin(), window.begin() + mid, window.begin() + wlen);
        result[i] = window[mid];
    }
    
    return result;
}

}  // namespace math
}  // namespace f1tenth_control
