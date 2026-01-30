#include "f1tenth_control/common/math_utils.hpp"
#include <algorithm>

namespace f1tenth_control {
namespace math {

std::vector<double> medianFilter(const std::vector<double>& data, size_t window_size) {
    if (data.empty()) return {};
    if (window_size <= 1) return data;
    
    // Ensure odd window size for symmetric filtering
    window_size |= 1;  // Faster than if/else for making odd
    
    const size_t half_window = window_size / 2;
    const size_t n = data.size();
    std::vector<double> result(n);
    std::vector<double> window;
    window.reserve(window_size);
    
    for (size_t i = 0; i < n; ++i) {
        window.clear();
        
        // Calculate window bounds with boundary handling
        const size_t start = (i >= half_window) ? i - half_window : 0;
        const size_t end = std::min(i + half_window + 1, n);
        
        // Copy window data
        for (size_t j = start; j < end; ++j) {
            window.push_back(data[j]);
        }
        
        // Find median using partial sort - O(n) average
        const size_t mid = window.size() / 2;
        std::nth_element(window.begin(), window.begin() + mid, window.end());
        result[i] = window[mid];
    }
    
    return result;
}

}  // namespace math
}  // namespace f1tenth_control
