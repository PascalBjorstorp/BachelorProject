#pragma once

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
    void build(const std::vector<Waypoint>& waypoints);
    std::size_t find_nearest(double x, double y) const;
    const Waypoint& get_waypoint(std::size_t idx) const;
    std::size_t size() const;

private:
    std::vector<KDNode> nodes_;
    std::vector<Waypoint> waypoints_;

    void build_recursive(std::size_t start, std::size_t end, int depth);
    void search_recursive(std::size_t start, std::size_t end, int depth,
                          double x, double y,
                          std::size_t& best_idx, double& best_dist) const;
};

}  // namespace f1tenth_communication
