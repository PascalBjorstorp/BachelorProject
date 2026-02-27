#pragma once

#include <vector>
#include <cstdint>
#include <Eigen/Core>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace gpu_amcl_cpp {

/**
 * @brief Occupancy map processor and coordinate converter.
 *
 * Holds the occupancy grid and its Euclidean distance transform.
 * Shared by the AMCL sensor model (distance-field lookups) and
 * by any code that needs world ↔ map conversions.
 */
class MapProcessor {
public:
    MapProcessor() = default;

    /// Load map from a nav_msgs/OccupancyGrid message.
    void load_from_msg(const nav_msgs::msg::OccupancyGrid& msg);

    /// Get the pre-computed Euclidean distance field (in cells).
    const std::vector<float>& distance_field() const { return distance_field_; }

    /// Get the raw occupancy data (0 = free, 100 = occupied, -1 = unknown).
    const std::vector<int8_t>& occupancy() const { return occupancy_; }

    /// Get all free-cell indices (for random particle injection).
    const std::vector<int>& free_cells() const { return free_cells_; }

    // ---- Geometry ----
    int width()  const { return width_; }
    int height() const { return height_; }
    float resolution() const { return resolution_; }
    float origin_x() const { return origin_x_; }
    float origin_y() const { return origin_y_; }
    bool  is_loaded() const { return loaded_; }

    /// Convert world (x, y) to map grid indices (col, row).
    inline void world_to_map(double wx, double wy,
                             int& mx, int& my) const {
        mx = static_cast<int>((wx - origin_x_) / resolution_);
        my = static_cast<int>((wy - origin_y_) / resolution_);
    }

    /// Convert map grid indices to world coordinates (cell center).
    inline void map_to_world(int mx, int my,
                             double& wx, double& wy) const {
        wx = origin_x_ + (mx + 0.5) * resolution_;
        wy = origin_y_ + (my + 0.5) * resolution_;
    }

    /// Check if grid indices are inside the map.
    inline bool is_in_bounds(int mx, int my) const {
        return mx >= 0 && mx < width_ && my >= 0 && my < height_;
    }

    /// Check if a cell is free (value == 0).
    inline bool is_free(int mx, int my) const {
        return is_in_bounds(mx, my) &&
               occupancy_[my * width_ + mx] == 0;
    }

    /// Check if a cell is occupied (value >= 50).
    inline bool is_occupied(int mx, int my) const {
        return is_in_bounds(mx, my) &&
               occupancy_[my * width_ + mx] >= 50;
    }

private:
    void compute_distance_field();
    void find_free_cells();

    std::vector<int8_t> occupancy_;
    std::vector<float>  distance_field_;
    std::vector<int>    free_cells_;  ///< flat indices of free cells.

    int   width_      = 0;
    int   height_     = 0;
    float resolution_ = 0.0f;
    float origin_x_   = 0.0f;
    float origin_y_   = 0.0f;
    bool  loaded_     = false;
};

}  // namespace gpu_amcl_cpp
