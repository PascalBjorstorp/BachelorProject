#pragma once

#include <vector>
#include <cstdint>
#include <cmath>
#include <cassert>
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
    /**
     * @brief Construct an empty map processor with no loaded map.
     *
     * Input:
     *   - None.
     * Output:
     *   - Map metadata remains at default values and loaded_ is false.
     */
    MapProcessor() = default;

    /**
     * @brief Load map data and metadata from an OccupancyGrid message.
     *
     * Input:
     *   - msg: ROS OccupancyGrid containing dimensions, origin, resolution, and occupancy data.
     * Output:
     *   - Internal occupancy grid and map metadata are updated.
     *   - Distance field and free-cell lists can be recomputed by implementation.
     */
    void load_from_msg(const nav_msgs::msg::OccupancyGrid& msg);

    /**
     * @brief Return the precomputed Euclidean distance field.
     *
     * Input:
     *   - None.
     * Output:
     *   - Const reference to distance values (in map-cell units).
     */
    const std::vector<float>& distance_field() const {
        return distance_field_; 
    }

    /**
     * @brief Return raw occupancy grid values.
     *
     * Input:
     *   - None.
     * Output:
     *   - Const reference to occupancy values (0 free, 100 occupied, -1 unknown).
     */
    const std::vector<int8_t>& occupancy() const {
        return occupancy_; 
    }

    /**
     * @brief Return flattened indices for all free cells in the map.
     *
     * Input:
     *   - None.
     * Output:
     *   - Const reference to free-cell index list.
     */
    const std::vector<int>& free_cells() const {
        return free_cells_; 
    }

    // ---- Geometry ----
    /**
     * @brief Return map width in cells.
     *
     * Input:
     *   - None.
     * Output:
     *   - Number of map columns.
     */
    int width()  const { 
        return width_; 
    }
    /**
     * @brief Return map height in cells.
     *
     * Input:
     *   - None.
     * Output:
     *   - Number of map rows.
     */
    int height() const { 
        return height_; 
    }
    /**
     * @brief Return map resolution in meters per cell.
     *
     * Input:
     *   - None.
     * Output:
     *   - Cell resolution in meters.
     */
    float resolution() const { 
        return resolution_; 
    }
    /**
     * @brief Return map origin X in world coordinates.
     *
     * Input:
     *   - None.
     * Output:
     *   - World-frame X coordinate of map origin.
     */
    float origin_x() const { 
        return origin_x_; 
    }
    /**
     * @brief Return map origin Y in world coordinates.
     *
     * Input:
     *   - None.
     * Output:
     *   - World-frame Y coordinate of map origin.
     */
    float origin_y() const { 
        return origin_y_; 
    }
    /**
     * @brief Return whether a map has been loaded into this processor.
     *
     * Input:
     *   - None.
     * Output:
     *   - true if map data/metadata are loaded, otherwise false.
     */
    bool  is_loaded() const { 
        return loaded_; 
    }

    /**
     * @brief Convert world coordinates to map grid indices.
     *
     * Input:
     *   - wx: World-frame X coordinate.
     *   - wy: World-frame Y coordinate.
     * Output:
     *   - mx: Map X index (column).
     *   - my: Map Y index (row).
     */
    inline void world_to_map(double wx, double wy,
                             int& mx, int& my) const {
        assert(loaded_ && "world_to_map requires a loaded map");
        assert(resolution_ > 0.0f && "world_to_map requires resolution_ > 0");
        mx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
        my = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
    }

    /**
     * @brief Convert map grid indices to world-frame coordinates at cell center.
     *
     * Input:
     *   - mx: Map X index (column).
     *   - my: Map Y index (row).
     * Output:
     *   - wx: World-frame X coordinate.
     *   - wy: World-frame Y coordinate.
     */
    inline void map_to_world(int mx, int my,
                             double& wx, double& wy) const {
        wx = origin_x_ + (mx + 0.5) * resolution_;
        wy = origin_y_ + (my + 0.5) * resolution_;
    }

    /**
     * @brief Check whether map indices lie inside valid map bounds.
     *
     * Input:
     *   - mx: Map X index (column).
     *   - my: Map Y index (row).
     * Output:
     *   - true if (mx, my) is inside [0, width) x [0, height), else false.
     */
    inline bool is_in_bounds(int mx, int my) const {
        return mx >= 0 && mx < width_ && my >= 0 && my < height_;
    }

    /**
     * @brief Check whether a map cell is free.
     *
     * Input:
     *   - mx: Map X index (column).
     *   - my: Map Y index (row).
     * Output:
     *   - true if index is in bounds and occupancy value equals 0.
     */
    inline bool is_free(int mx, int my) const {
        return is_in_bounds(mx, my) &&
               occupancy_[my * width_ + mx] == 0;
    }

    /**
     * @brief Check whether a map cell is occupied.
     *
     * Input:
     *   - mx: Map X index (column).
     *   - my: Map Y index (row).
     * Output:
     *   - true if index is in bounds and occupancy value is at least 50.
     */
    inline bool is_occupied(int mx, int my) const {
        return is_in_bounds(mx, my) &&
               occupancy_[my * width_ + mx] >= 50;
    }

private:
    /**
     * @brief Compute Euclidean distance-from-obstacle values for all map cells.
     *
     * Input:
     *   - Uses current occupancy map dimensions and occupancy values.
     * Output:
     *   - Updates internal distance_field_ values.
     */
    void compute_distance_field();
    /**
     * @brief Collect flattened indices of free cells from occupancy data.
     *
     * Input:
     *   - Uses current occupancy map values.
     * Output:
     *   - Updates internal free_cells_ index list.
     */
    void find_free_cells();

    std::vector<int8_t> occupancy_;
    std::vector<float>  distance_field_;
    std::vector<int>    free_cells_;  ///< flat indices of free cells.

    int   width_      = 0;
    int   height_     = 0;
    int   map_size_   = 0;
    float resolution_ = 0.0f;
    float origin_x_   = 0.0f;
    float origin_y_   = 0.0f;
    bool  loaded_     = false;
};

}  // namespace gpu_amcl_cpp
