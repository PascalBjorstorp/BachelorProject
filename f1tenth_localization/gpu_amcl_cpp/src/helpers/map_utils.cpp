#include "gpu_amcl_cpp/helpers/map_utils.hpp"

#include <cmath>
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace {

/**
 * @brief Compute intersection x-coordinate between two EDT parabolas.
 *
 * Input:
 *   - f: 1D cost array.
 *   - i: Index of first parabola site.
 *   - u: Index of second parabola site.
 * Output:
 *   - x-position where parabola(u) becomes better than parabola(i).
 */
inline float edt_intersection(const std::vector<float>& f, int i, int u) {
    const double fi = static_cast<double>(f[i]);
    const double fu = static_cast<double>(f[u]);
    const double ii = static_cast<double>(i) * static_cast<double>(i);
    const double uu = static_cast<double>(u) * static_cast<double>(u);
    return static_cast<float>(((fu + uu) - (fi + ii)) /
                              (2.0 * static_cast<double>(u - i)));
}

/**
 * @brief Compute exact 1D squared Euclidean distance transform using Felzenszwalb-Huttenlocher.
 *
 * Input:
 *   - f: Input 1D costs (0 at sites, large value elsewhere).
 *   - v: Work buffer for site indices in lower envelope.
 *   - z: Work buffer for envelope boundary positions.
 *   - n: Active length of the 1D slice.
 *   - inf_value: Sentinel used for non-site values.
 * Output:
 *   - d: Output 1D squared distances to nearest site.
 */
void edt_1d_felzenszwalb(const std::vector<float>& f,
                         std::vector<float>& d,
                         std::vector<int>& v,
                         std::vector<float>& z,
                         int n,
                         float inf_value) {
    int first_site = -1;
    for (int i = 0; i < n; ++i) {
        if (f[i] < inf_value) {
            first_site = i;
            break;
        }
    }

    if (first_site < 0) {
        std::fill(d.begin(), d.begin() + n, inf_value);
        return;
    }

    int k = 0;
    v[0] = first_site;
    z[0] = -std::numeric_limits<float>::infinity();
    z[1] = std::numeric_limits<float>::infinity();

    for (int q = first_site + 1; q < n; ++q) {
        if (f[q] >= inf_value) {
            continue;
        }

        float s = edt_intersection(f, v[k], q);
        while (s <= z[k]) {
            --k;
            s = edt_intersection(f, v[k], q);
        }

        ++k;
        v[k] = q;
        z[k] = s;
        z[k + 1] = std::numeric_limits<float>::infinity();
    }

    k = 0;
    for (int q = 0; q < n; ++q) {
        while (z[k + 1] < static_cast<float>(q)) {
            ++k;
        }

        const float diff = static_cast<float>(q - v[k]);
        d[q] = diff * diff + f[v[k]];
    }
}

}  // namespace

namespace gpu_amcl_cpp {

/**
 * @brief Load occupancy map metadata/data and trigger map preprocessing.
 *
 * Input:
 *   - msg: ROS OccupancyGrid containing map dimensions, origin, resolution, and occupancy values.
 * Output:
 *   - Updates map metadata and occupancy_.
 *   - Recomputes distance_field_ and free_cells_.
 *   - Sets loaded_ to true.
 */
void MapProcessor::load_from_msg(const nav_msgs::msg::OccupancyGrid& msg) {
    width_      = static_cast<int>(msg.info.width);
    height_     = static_cast<int>(msg.info.height);
    map_size_   = width_ * height_;
    resolution_ = static_cast<float>(msg.info.resolution);
    origin_x_   = static_cast<float>(msg.info.origin.position.x);
    origin_y_   = static_cast<float>(msg.info.origin.position.y);

    if (msg.data.size() != static_cast<size_t>(map_size_)) {
        throw std::runtime_error("OccupancyGrid data size does not match width*height");
    }

    occupancy_.resize(map_size_);
    for (int i = 0; i < map_size_; ++i) {
        occupancy_[i] = msg.data[i];
    }

    compute_distance_field();
    find_free_cells();
    loaded_ = true;
}

/**
 * @brief Compute exact Euclidean distance transform (EDT) for obstacle distances.
 *
 * Input:
 *   - Uses occupancy_, width_, height_, map_size_, and resolution_.
 * Output:
 *   - Updates distance_field_ with exact Euclidean obstacle distances in meters.
 *
 * Notes:
 *   - Uses separable 1D Felzenszwalb-Huttenlocher EDT in two passes (rows then columns).
 */
void MapProcessor::compute_distance_field() {
    const int size = map_size_;

    if (size <= 0 || width_ <= 0 || height_ <= 0) {
        distance_field_.clear();
        return;
    }

    static constexpr float kInf = 1e20f;
    distance_field_.assign(size, kInf);

    // Initialize distance field with 0 for occupied cells and track if any occupied cells exist.
    bool has_occupied = false;
    for (int i = 0; i < size; ++i) {
        if (occupancy_[i] >= 50) {
            distance_field_[i] = 0.0f;
            has_occupied = true;
        }
    }

    if (!has_occupied) {
        return; // No occupied cells, so all distances remain at infinity.
    }

    // Temporary buffers for 1D EDT computations.
    const int max_dim = std::max(width_, height_);
    std::vector<float> f(max_dim);
    std::vector<float> d(max_dim);
    std::vector<int> v(max_dim);
    std::vector<float> z(max_dim + 1);

    // Pass 1: EDT across rows.
    for (int y = 0; y < height_; ++y) {                 // Each row
        const int row_offset = y * width_;             
        for (int x = 0; x < width_; ++x) {              // Each column in the row
            f[x] = distance_field_[row_offset + x];     // Initialize f with current distances for this row.
        }

        edt_1d_felzenszwalb(f, d, v, z, width_, kInf);  // Compute EDT for this row.

        for (int x = 0; x < width_; ++x) {
            distance_field_[row_offset + x] = d[x];     // Update distance field with row-wise EDT results.
        }
    }

    // Pass 2: EDT across columns.
    for (int x = 0; x < width_; ++x) {                  // Each column
        for (int y = 0; y < height_; ++y) {             // Each row in the column
            f[y] = distance_field_[y * width_ + x];
        }

        edt_1d_felzenszwalb(f, d, v, z, height_, kInf); // Compute EDT for this column.

        for (int y = 0; y < height_; ++y) {
            distance_field_[y * width_ + x] = d[y];     // Update distance field with column-wise EDT results.
        }
    }

    // Convert squared cell distances to metres.
    for (float& dist_sq : distance_field_) {
        dist_sq = std::sqrt(dist_sq) * resolution_;
    }
}

/**
 * @brief Extract flattened indices for all free cells in the occupancy grid.
 *
 * Input:
 *   - Uses occupancy_ and map_size_.
 * Output:
 *   - Updates free_cells_ with indices where occupancy value equals 0.
 */
void MapProcessor::find_free_cells() {
    free_cells_.clear();
    const int size = map_size_;
    for (int i = 0; i < size; ++i) {
        if (occupancy_[i] == 0) {
            free_cells_.push_back(i);
        }
    }
}

}  // namespace gpu_amcl_cpp
