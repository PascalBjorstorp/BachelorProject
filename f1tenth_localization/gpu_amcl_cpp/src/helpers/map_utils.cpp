#include "gpu_amcl_cpp/helpers/map_utils.hpp"

#include <queue>
#include <cmath>
#include <algorithm>
#include <limits>

namespace gpu_amcl_cpp {

void MapProcessor::load_from_msg(const nav_msgs::msg::OccupancyGrid& msg) {
    width_      = static_cast<int>(msg.info.width);
    height_     = static_cast<int>(msg.info.height);
    resolution_ = static_cast<float>(msg.info.resolution);
    origin_x_   = static_cast<float>(msg.info.origin.position.x);
    origin_y_   = static_cast<float>(msg.info.origin.position.y);

    occupancy_.resize(width_ * height_);
    for (int i = 0; i < width_ * height_; ++i) {
        occupancy_[i] = msg.data[i];
    }

    compute_distance_field();
    find_free_cells();
    loaded_ = true;
}

// ── Euclidean distance transform via BFS ─────────────────────────
// Exact EDT would use the Felzenszwalb algorithm, but 8-connected
// BFS gives a good approximation and is fast enough at init time.
void MapProcessor::compute_distance_field() {
    const int size = width_ * height_;
    distance_field_.assign(size, std::numeric_limits<float>::max());

    // Seed: all occupied cells have distance 0.
    std::queue<int> q;
    for (int i = 0; i < size; ++i) {
        if (occupancy_[i] >= 50) {    // occupied
            distance_field_[i] = 0.0f;
            q.push(i);
        }
    }

    // 8-connected BFS with costs 1.0 / sqrt(2).
    static constexpr int dx8[] = {-1, 1,  0, 0, -1, -1, 1,  1};
    static constexpr int dy8[] = { 0, 0, -1, 1, -1,  1, -1, 1};
    static constexpr float cost8[] = {
        1.0f, 1.0f, 1.0f, 1.0f,
        1.4142f, 1.4142f, 1.4142f, 1.4142f
    };

    while (!q.empty()) {
        int idx = q.front();
        q.pop();
        int cx = idx % width_;
        int cy = idx / width_;
        float cd = distance_field_[idx];

        for (int d = 0; d < 8; ++d) {
            int nx = cx + dx8[d];
            int ny = cy + dy8[d];
            if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;
            int ni = ny * width_ + nx;
            float nd = cd + cost8[d];
            if (nd < distance_field_[ni]) {
                distance_field_[ni] = nd;
                q.push(ni);
            }
        }
    }

    // Convert cell distances to metres.
    for (auto& d : distance_field_) {
        d *= resolution_;
    }
}

void MapProcessor::find_free_cells() {
    free_cells_.clear();
    const int size = width_ * height_;
    for (int i = 0; i < size; ++i) {
        if (occupancy_[i] == 0) {
            free_cells_.push_back(i);
        }
    }
}

}  // namespace gpu_amcl_cpp
