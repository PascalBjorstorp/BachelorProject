#include "f1tenth_control/common/math_utils.hpp"
#include <algorithm>
#include <numeric>

namespace f1tenth_control {
namespace math {

void movingAverageFilter(std::vector<double>& data, size_t window_size) {
    if (data.empty() || window_size <= 1) return;
    
    // Ensure odd window size
    if (window_size % 2 == 0) window_size++;
    
    const size_t half_window = window_size / 2;
    std::vector<double> result(data.size());
    
    for (size_t i = 0; i < data.size(); ++i) {
        size_t start = (i >= half_window) ? i - half_window : 0;
        size_t end = std::min(i + half_window + 1, data.size());
        
        double sum = 0.0;
        size_t count = 0;
        for (size_t j = start; j < end; ++j) {
            sum += data[j];
            ++count;
        }
        result[i] = sum / static_cast<double>(count);
    }
    
    data = std::move(result);
}

std::vector<double> medianFilter(const std::vector<double>& data, size_t window_size) {
    if (data.empty()) return {};
    if (window_size <= 1) return data;
    
    // Ensure odd window size
    if (window_size % 2 == 0) window_size++;
    
    const size_t half_window = window_size / 2;
    std::vector<double> result(data.size());
    std::vector<double> window;
    window.reserve(window_size);
    
    for (size_t i = 0; i < data.size(); ++i) {
        window.clear();
        
        size_t start = (i >= half_window) ? i - half_window : 0;
        size_t end = std::min(i + half_window + 1, data.size());
        
        for (size_t j = start; j < end; ++j) {
            window.push_back(data[j]);
        }
        
        // Find median
        std::nth_element(window.begin(), window.begin() + window.size() / 2, window.end());
        result[i] = window[window.size() / 2];
    }
    
    return result;
}

double curvature(const Point2D& p1, const Point2D& p2, const Point2D& p3) {
    // Menger curvature: 4 * area / (|p1-p2| * |p2-p3| * |p3-p1|)
    double d12 = distance(p1, p2);
    double d23 = distance(p2, p3);
    double d31 = distance(p3, p1);
    
    // Area using cross product
    double area = 0.5 * std::abs(
        (p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y)
    );
    
    double denom = d12 * d23 * d31;
    if (denom < constants::EPSILON) {
        return 0.0;  // Points are collinear or coincident
    }
    
    return 4.0 * area / denom;
}

}  // namespace math
}  // namespace f1tenth_control
