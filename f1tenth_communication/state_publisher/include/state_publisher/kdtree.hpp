#pragma once

/**
 * @file kdtree.hpp
 * @brief KD-tree interface used for nearest-waypoint lookup.
 * @details Declares waypoint storage types and KD-tree query/build routines
 *          used by communication publisher paths.
 * @dependencies cstddef, vector
 * @return Not applicable (file-level documentation block).
 */

#include <cstddef>
#include <vector>

namespace f1tenth_communication {

struct Waypoint {
    double s;       // Arc length along track
    double x;       // Position X in map frame
    double y;       // Position Y in map frame
    double psi;     // Heading
    double kappa;   // Curvature
    double vx;      // Target speed
    double left_bound;   // Left lateral bound [m]
    double right_bound;  // Right lateral bound [m]
};

struct KDNode {
    double x;
    double y;
    std::size_t index;  // Index into the original waypoint array
};

class KDTree {
public:
    /**
     * @brief Build a KD-tree index from waypoint coordinates.
     * @param waypoints Waypoint set to index for nearest-neighbor lookup.
        * @return No direct return value. Rebuilds internal node and waypoint storage.
     */
    void build(const std::vector<Waypoint>& waypoints);

    /**
     * @brief Find the nearest indexed waypoint to a query position.
     * @param x Query X coordinate in map frame.
     * @param y Query Y coordinate in map frame.
     * @return Index of the nearest waypoint in the stored waypoint vector.
     */
    std::size_t find_nearest(double x, double y) const;

    /**
     * @brief Access a waypoint by index.
     * @param idx Waypoint index in the stored waypoint vector.
     * @return Constant reference to the requested waypoint.
     */
    const Waypoint& get_waypoint(std::size_t idx) const;

    /**
     * @brief Get the number of stored waypoints.
     * @return Waypoint count currently indexed by the tree.
     */
    std::size_t size() const;

private:
    std::vector<KDNode> nodes_;
    std::vector<Waypoint> waypoints_;

    /**
     * @brief Recursively partition node storage into balanced KD-tree layout.
     * @param start Inclusive start index of the partition range.
     * @param end Exclusive end index of the partition range.
     * @param depth Recursion depth used to select split axis.
     * @return None.
     */
    void build_recursive(std::size_t start, std::size_t end, int depth);

    /**
     * @brief Recursively search KD-tree partitions for nearest neighbor.
     * @param start Inclusive start index of the partition range.
     * @param end Exclusive end index of the partition range.
     * @param depth Recursion depth used to select split axis.
     * @param x Query X coordinate.
     * @param y Query Y coordinate.
     * @param best_idx In/out index of current best candidate in `nodes_`.
     * @param best_dist In/out best squared distance.
     * @return None
     */
    void search_recursive(std::size_t start, std::size_t end, int depth,
                          double x, double y,
                          std::size_t& best_idx, double& best_dist) const;
};

}  // namespace f1tenth_communication
