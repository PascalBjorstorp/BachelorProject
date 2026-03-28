/**
 * @file kdtree.cpp
 * @brief KD-tree implementation for nearest-waypoint lookup.
 * @details Implements KD-tree build and query routines used by state_publisher.
 *          Public and private function contracts are documented in kdtree.hpp.
 * @dependencies state_publisher/kdtree.hpp, algorithm, limits
 * @return Not applicable (file-level documentation block).
 */

#include "state_publisher/kdtree.hpp"

#include <algorithm>
#include <limits>

namespace f1tenth_communication {

/**
 * @brief Build KD-tree nodes from waypoint coordinates.
 * @param waypoints Waypoint array used to initialize tree storage.
 * @return None
 */
void KDTree::build(const std::vector<Waypoint>& waypoints) {
    waypoints_ = waypoints;
    nodes_.clear();
    nodes_.reserve(waypoints.size());

    for (std::size_t i = 0; i < waypoints.size(); ++i) {
        nodes_.push_back({waypoints[i].x, waypoints[i].y, i});
    }

    // Build balanced partitions by alternating x/y split dimensions.
    build_recursive(0, nodes_.size(), 0);
}

/**
 * @brief Find nearest waypoint index for a query point.
 * @param x Query x coordinate.
 * @param y Query y coordinate.
 * @return Index of nearest waypoint in stored waypoint array.
 */
std::size_t KDTree::find_nearest(double x, double y) const {
    if (nodes_.empty()) {
        return 0;
    }

    std::size_t best_idx = 0;
    double best_dist = std::numeric_limits<double>::max();
    search_recursive(0, nodes_.size(), 0, x, y, best_idx, best_dist);
    return nodes_[best_idx].index;
}

/**
 * @brief Access waypoint by index.
 * @param idx Waypoint index.
 * @return Constant reference to waypoint at `idx`.
 */
const Waypoint& KDTree::get_waypoint(std::size_t idx) const {
    return waypoints_[idx];
}

/**
 * @brief Get number of stored waypoints.
 * @return Current waypoint count.
 */
std::size_t KDTree::size() const {
    return waypoints_.size();
}

/**
 * @brief Recursively partition node array into balanced KD-tree layout.
 * @param start Inclusive start index.
 * @param end Exclusive end index.
 * @param depth Recursion depth used to pick split axis.
 * @return None
 */
void KDTree::build_recursive(std::size_t start, std::size_t end, int depth) {
    if (end - start <= 1) {
        return;
    }

    std::size_t mid = start + (end - start) / 2;

    if (depth % 2 == 0) {
        std::nth_element(nodes_.begin() + start, nodes_.begin() + mid,
                         nodes_.begin() + end,
                         [](const KDNode& a, const KDNode& b) {
                             return a.x < b.x;
                         });
    } else {
        std::nth_element(nodes_.begin() + start, nodes_.begin() + mid,
                         nodes_.begin() + end,
                         [](const KDNode& a, const KDNode& b) {
                             return a.y < b.y;
                         });
    }

    build_recursive(start, mid, depth + 1);
    build_recursive(mid + 1, end, depth + 1);
}

/**
 * @brief Recursively search KD-tree for nearest candidate.
 * @param start Inclusive start index.
 * @param end Exclusive end index.
 * @param depth Recursion depth used to pick split axis.
 * @param x Query x coordinate.
 * @param y Query y coordinate.
 * @param best_idx In/out best node index.
 * @param best_dist In/out best squared distance.
 * @return None
 */
void KDTree::search_recursive(std::size_t start, std::size_t end, int depth,
                              double x, double y,
                              std::size_t& best_idx, double& best_dist) const {
    if (start >= end) {
        return;
    }

    std::size_t mid = start + (end - start) / 2;
    const KDNode& node = nodes_[mid];

    // Compare candidate point with current best.
    const double dx = x - node.x;
    const double dy = y - node.y;
    const double dist = dx * dx + dy * dy;

    if (dist < best_dist) {
        best_dist = dist;
        best_idx = mid;
    }

    // Visit the most promising side first, then prune the far side unless needed.
    const double split_val = (depth % 2 == 0) ? node.x : node.y;
    const double query_val = (depth % 2 == 0) ? x : y;
    const double diff = query_val - split_val;

    if (diff < 0.0) {
        search_recursive(start, mid, depth + 1, x, y, best_idx, best_dist);
        if (diff * diff < best_dist) {
            search_recursive(mid + 1, end, depth + 1, x, y, best_idx, best_dist);
        }
    } else {
        search_recursive(mid + 1, end, depth + 1, x, y, best_idx, best_dist);
        if (diff * diff < best_dist) {
            search_recursive(start, mid, depth + 1, x, y, best_idx, best_dist);
        }
    }
}

}  // namespace f1tenth_communication
